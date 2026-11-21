//! Cannonball drag with a stochastic atmospheric density on MuJoCo dynamics:
//! port of upstream `examples/mujoco/scenarioStochasticDrag.py`. Needs the
//! `mujoco` feature.
//!
//! A 1 kg point-mass `ball` in an `MJScene` on a 250 km circular orbit, with
//! Earth's point-mass gravity (upstream `gravFactory.addBodiesTo(scene)`),
//! 10 m² of cannonball drag (`CannonballDrag`, `C_D` 2.2) through a force
//! actuator at its center of mass, and an exponential atmosphere (in the
//! top-level task, as upstream) whose density is scaled by a stochastic
//! factor integrated with the scene, for 7.1 orbits at 10 s steps:
//!
//! | `--igbm` | density model | integrator | CSV |
//! |---|---|---|---|
//! | no (default) | `StochasticAtmDensity`, `ρ (1 + δ)` with δ Ornstein-Uhlenbeck | [`Sosra`] | `scenarioStochasticDrag.csv` |
//! | yes | `IgbmAtmDensity`, `max(0, ρ x)` with x an IGBM of mean 1 | [`Sriw1`] | `scenarioStochasticDrag_igbm.csv` |
//!
//! Both use a stationary standard deviation of 0.15 and a 108 s time
//! constant. As upstream, the density model runs in the scene's dynamics
//! and diffusion tasks ([`MjScene::add_model_to_dynamics_and_diffusion_tasks`]),
//! so its diffusion is evaluated at each diffusion stage. `--all` runs both,
//! as `test_scenarioMujoco.py` does (`{}` and `{"useIgbm": True}`); `--seed`
//! sets the integrator seed (basilisk-rs's generator is not upstream's, so a
//! realization never matches upstream's draw for draw).
//!
//! The CSV holds, every step, the ball origin state, the exponential and the
//! stochastic densities and the drag force (upstream's four recorders). As
//! upstream sets no `extraEoMCall`, the density and drag outputs are those
//! of the step's last integrator stage, not of its final state: their
//! statistics are biased (a smaller mean factor for the IGBM, a larger
//! spread for the OU process).
//!
//! `test_scenarioMujoco.py` only runs both cases; `tests` checks that
//! `extraEoMCall` leaves the trajectory unchanged, that with it the density
//! factor follows the configured stationary process, and that the orbit
//! decays. Upstream's Vizard call (`saveFile` commented out) is not ported.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/mj_scenarios.rs"]
mod mj_scenarios;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/shared.rs"]
mod shared;

use basilisk_rs::connect;
use basilisk_rs::environment::atmosphere::exponential_atmosphere::{
    ExponentialAtmosphere, ExponentialAtmosphereConfig,
};
use basilisk_rs::integrators::Integrator;
use basilisk_rs::integrators::stochastic::{Sosra, Sriw1};
use basilisk_rs::messages::{
    AtmosphereMsg, BodyForceCommandMsg, DragGeometryMsg, Output, SpacecraftStateMsg,
};
use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::mujoco_dynamics::cannonball_drag::CannonballDrag;
use basilisk_rs::mujoco_dynamics::igbm_atm_density::IgbmAtmDensity;
use basilisk_rs::mujoco_dynamics::stochastic_atm_density::StochasticAtmDensity;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{MU_EARTH, REQ_EARTH};
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use cli::Cli;
use common::{seconds, vector_columns};
use message_log::MessageLog;
use mj_scenarios::{
    GravitySourceSpec, add_gravity_to_all_bodies, epoch, scalar_column, site_state_columns,
    written_csv,
};
use nalgebra::Vector3;
use orbit_tools::{orbit_period_s, sec2nano};
use recording::scenario_csv;
use shared::Shared;

const USAGE: &str = "\
scenario_stochastic_drag [--igbm] [--seed N]
scenario_stochastic_drag --all     run every upstream test case";

/// Upstream `dt`.
const STEP_NANOS: u64 = seconds(10);
/// Upstream `setStationaryStd`.
const STATIONARY_STD: f64 = 0.15;
/// Upstream `setTimeConstant` \[s\].
const TIME_CONSTANT_S: f64 = 1.8 * 60.0;

/// Upstream `CANNONBALL_SCENE_XML`.
const CANNONBALL_SCENE_XML: &str = r#"
<mujoco>
	<worldbody>
		<body name="ball">
			<freejoint/>
			<inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
		</body>
	</worldbody>
</mujoco>
"#;

/// Upstream `run(showPlots, useIgbm)`.
#[derive(Clone, Copy, Debug)]
struct Config {
    igbm: bool,
    seed: Option<u64>,
}

impl Config {
    fn file_stem(&self) -> &'static str {
        if self.igbm {
            "scenarioStochasticDrag_igbm"
        } else {
            "scenarioStochasticDrag"
        }
    }
}

/// `test_scenarioMujoco.py`'s runs: `{}` and `{"useIgbm": True}`.
const TEST_CASES: [Config; 2] = [
    Config {
        igbm: false,
        seed: None,
    },
    Config {
        igbm: true,
        seed: None,
    },
];

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    states: Vec<(u64, SpacecraftStateMsg)>,
    deterministic_density: Vec<(u64, AtmosphereMsg)>,
    density: Vec<(u64, AtmosphereMsg)>,
    drag: Vec<(u64, BodyForceCommandMsg)>,
}

/// Upstream `run`; `committed_outputs` sets `extraEoMCall` (upstream leaves
/// it off), which re-evaluates the dynamics task at each step's final state
/// without changing the dynamics.
fn run(config: Config, write_csv: bool, committed_outputs: bool) -> Results {
    let mu = MU_EARTH * 1e9;
    let radius = REQ_EARTH * 1e3;
    let (position_m, velocity_mps) = elements_to_state(
        mu,
        &ClassicElements {
            semi_major_axis: radius + 250.0 * 1000.0,
            eccentricity: 0.0,
            inclination_rad: 33.3_f64.to_radians(),
            raan_rad: 48.2_f64.to_radians(),
            arg_periapsis_rad: 347.8_f64.to_radians(),
            true_anomaly_rad: 85.3_f64.to_radians(),
            ..Default::default()
        },
    );
    let elements = state_to_elements(mu, position_m, velocity_mps);
    let duration_nanos = sec2nano(7.1 * orbit_period_s(mu, elements.semi_major_axis));

    let mut scene = MjScene::new(CANNONBALL_SCENE_XML).expect("valid cannonball scene");
    // A strong integrator for the single sample path: SOSRA for the OU's
    // additive noise, SRIW1 for the IGBM's multiplicative noise.
    let integrator: Box<dyn Integrator<MjScene>> = match (config.igbm, config.seed) {
        (false, None) => Box::new(Sosra::new()),
        (false, Some(seed)) => Box::new(Sosra::new().with_seed(seed)),
        (true, None) => Box::new(Sriw1::new()),
        (true, Some(seed)) => Box::new(Sriw1::new().with_seed(seed)),
    };
    scene.set_boxed_integrator(integrator);
    scene.extra_eom_call = committed_outputs;
    add_gravity_to_all_bodies(
        &mut scene,
        &[GravitySourceSpec {
            name: "earth",
            mu_m3ps2: mu,
            is_central_body: true,
            ephemeris: None,
        }],
    );
    let drag_point = scene.body("ball").center_of_mass().clone();

    // Upstream `simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")`.
    let mut atmosphere = ExponentialAtmosphere::new(ExponentialAtmosphereConfig {
        name: "ExpAtmo".to_string(),
        planet_radius_m: 6_378_136.6,
        reference_altitude_m: 0.0,
        reference_density_kgpm3: 1.217,
        scale_height_m: 8_500.0,
    });
    drag_point
        .state_out_msg
        .connect_to(&mut atmosphere.input_state_msg);

    let density_out = if config.igbm {
        let mut model = IgbmAtmDensity {
            name: "StochasticExpAtmo".to_string(),
            ..IgbmAtmDensity::default()
        };
        model.igbm.set_mean(1.0);
        model.igbm.set_stationary_std(STATIONARY_STD);
        model.igbm.set_time_constant(TIME_CONSTANT_S);
        atmosphere
            .output_atmosphere_msg
            .connect_to(&mut model.atmo_dens_in_msg);
        let out = model.atmo_dens_out_msg.clone();
        scene.add_model_to_dynamics_and_diffusion_tasks(model, -1, -1);
        out
    } else {
        let mut model = StochasticAtmDensity {
            name: "StochasticExpAtmo".to_string(),
            ..StochasticAtmDensity::default()
        };
        model.noise.set_stationary_std(STATIONARY_STD);
        model.noise.set_time_constant(TIME_CONSTANT_S);
        atmosphere
            .output_atmosphere_msg
            .connect_to(&mut model.atmo_dens_in_msg);
        let out = model.atmo_dens_out_msg.clone();
        scene.add_model_to_dynamics_and_diffusion_tasks(model, -1, -1);
        out
    };

    let mut drag = CannonballDrag {
        name: "DragEff".to_string(),
        ..CannonballDrag::default()
    };
    let geometry = Output::new(DragGeometryMsg {
        drag_coeff: 2.2,
        projected_area_m2: 10.0,
        r_cp_s_m: Vector3::zeros(),
    });
    geometry.connect_to(&mut drag.drag_geometry_in_msg);
    drag_point
        .state_out_msg
        .connect_to(&mut drag.reference_frame_state_in_msg);
    density_out.connect_to(&mut drag.atmo_dens_in_msg);
    drag.force_out_msg.connect_to(
        &mut scene
            .add_force_actuator("dragForce", drag_point.name())
            .force_in_msg,
    );
    let drag_out = drag.force_out_msg.clone();
    scene.add_model_to_dynamics_task(drag, -1);

    let ball_state = scene.body("ball").origin().state_out_msg.clone();
    let deterministic_out = atmosphere.output_atmosphere_msg.clone();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut deterministic_log = MessageLog::<AtmosphereMsg>::default();
    let mut density_log = MessageLog::<AtmosphereMsg>::default();
    let mut drag_log = MessageLog::<BodyForceCommandMsg>::default();
    let mut csv = write_csv.then(|| scenario_csv(config.file_stem()));
    let scene = Shared::new(scene);
    let mut scene_task = scene.handle();
    {
        let mut simulation = Simulation::new(epoch(), false);
        connect!(&simulation,
            &ball_state => &mut state_log.input,
            &deterministic_out => &mut deterministic_log.input,
            &density_out => &mut density_log.input,
            &drag_out => &mut drag_log.input,
        );
        // Upstream's single task in insertion order: scene, atmosphere,
        // recorders.
        simulation.add_module("scene", &mut scene_task, STEP_NANOS, 10);
        simulation.add_module("ExpAtmo", &mut atmosphere, STEP_NANOS, 9);
        simulation.add_module("bodyStateRecorder", &mut state_log, STEP_NANOS, 0);
        simulation.add_module(
            "deterministicDensityRecorder",
            &mut deterministic_log,
            STEP_NANOS,
            0,
        );
        simulation.add_module("densityRecorder", &mut density_log, STEP_NANOS, 0);
        simulation.add_module("dragRecorder", &mut drag_log, STEP_NANOS, 0);
        if let Some(csv) = csv.as_mut() {
            connect!(&simulation,
                &ball_state => csv.add_source::<SpacecraftStateMsg>(site_state_columns("")),
                &deterministic_out => csv.add_source::<AtmosphereMsg>(
                    scalar_column("neutral_density_kgpm3", "neutralDensityExponential_kg_per_m3"),
                ),
                &density_out => csv.add_source::<AtmosphereMsg>(
                    scalar_column("neutral_density_kgpm3", "neutralDensity_kg_per_m3"),
                ),
                &drag_out => csv.add_source::<BodyForceCommandMsg>(
                    CsvSourceConfig::columns(vector_columns("force_request_body_n", "force_S_N")),
                ),
            );
            simulation.add_module("csv", csv, STEP_NANOS, 0);
        }
        simulation.initialize();
        {
            let mut scene = scene.lock();
            let mut ball = scene.body_mut("ball");
            ball.set_position(position_m);
            ball.set_velocity(velocity_mps);
        }
        simulation.run_for(duration_nanos);
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    Results {
        states: state_log.into_samples(),
        deterministic_density: deterministic_log.into_samples(),
        density: density_log.into_samples(),
        drag: drag_log.into_samples(),
    }
}

fn main() {
    let mut cli = Cli::from_env(USAGE);
    let configs = if cli.switch("all") {
        TEST_CASES.to_vec()
    } else {
        let igbm = cli.flag("igbm", false);
        let seed = cli.value::<i64>("seed", -1);
        vec![Config {
            igbm,
            seed: u64::try_from(seed).ok(),
        }]
    };
    cli.finish();
    for config in configs {
        let results = run(config, true, false);
        let first = results.states[0].1.position_m.norm();
        let last = results
            .states
            .last()
            .expect("recorded states")
            .1
            .position_m
            .norm();
        println!("radius {:.3} -> {:.3} km", first / 1000.0, last / 1000.0);
        println!("wrote {}", written_csv(config.file_stem()).display());
    }
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` runs both cases without checks. The density
    //! factor must follow its stationary process (mean near 1, standard
    //! deviation near 0.15, over about 360 time constants) and drag must
    //! lower the orbit.

    use super::*;

    #[test]
    fn density_factor_statistics_and_orbit_decay() {
        for case in TEST_CASES {
            let config = Config {
                seed: Some(7),
                ..case
            };
            let stage_outputs = run(config, false, false);
            let results = run(config, false, true);
            // `extraEoMCall` changes the outputs, not the dynamics.
            let (_, last) = results.states.last().unwrap();
            assert_eq!(
                stage_outputs.states.last().unwrap().1.position_m,
                last.position_m
            );
            // The factor ρ_stoch / ρ_exp at each step's final state.
            let factors: Vec<f64> = results
                .density
                .iter()
                .zip(&results.deterministic_density)
                .skip(1)
                .map(|((_, stochastic), (_, nominal))| {
                    stochastic.neutral_density_kgpm3 / nominal.neutral_density_kgpm3
                })
                .collect();
            let n = factors.len() as f64;
            let mean = factors.iter().sum::<f64>() / n;
            let std = (factors.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n).sqrt();
            assert!(
                (mean - 1.0).abs() < 0.03,
                "{}: mean {mean}",
                config.file_stem()
            );
            assert!(
                (std / STATIONARY_STD - 1.0).abs() < 0.2,
                "{}: std {std}",
                config.file_stem()
            );
            assert!(factors.iter().all(|factor| *factor >= 0.0));
            let first = results.states[0].1.position_m.norm();
            assert!(
                last.position_m.norm() < first - 1000.0,
                "{}: {first} -> {}",
                config.file_stem(),
                last.position_m.norm()
            );
            assert!(
                results
                    .drag
                    .iter()
                    .skip(1)
                    .all(|(_, force)| force.force_request_body_n.norm() > 0.0)
            );
        }
    }
}
