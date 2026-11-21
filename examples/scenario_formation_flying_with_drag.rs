//! Chief-deputy formation flying under differential drag in one `MJScene`:
//! port of upstream `examples/mujoco/scenarioFormationFlyingWithDrag.py`.
//! Needs the `mujoco` feature.
//!
//! `CHIEF_DEPUTY_SCENE_XML`: two 1 kg point masses, `chief` and `deputy`,
//! each on three slide joints (translation only). Earth's point-mass gravity
//! on both (upstream `gravFactory.addBodiesTo(scene)`). With `drag` (the
//! default), each spacecraft has cannonball drag at its center of mass
//! (`CannonballDrag.applyTo(body)`, 1 m², `C_D` 30 for the chief and 100
//! for the deputy) in an exponential atmosphere. With `control` (the
//! default), `orbElemConvert` turns both states into classical elements,
//! `orbElemOffset` adds the target offset (δe = 0.0005, the rest zero, the
//! anomaly offset a mean anomaly) to the chief's, and
//! `OrbitalElementControl` (proportional gains diag(1e5, 1e5, 0, 0, 10, 10)
//! and, with `integralControl`, integral gains diag(100, 0, 0, 0, 0, 0)) maps
//! the deputy's element error to an inertial force, which
//! `CmdForceInertialToForceAtSite` rotates into a force actuator at the
//! deputy's center of mass. Everything but the recorders runs in the scene's
//! dynamics task. The chief starts on a 250 × 300 km orbit, the deputy
//! offset by small element differences; RKF45 with tolerances 1e-5, 10 s
//! task step, 10 orbits.
//!
//! Upstream's `exponentialAtmosphere` serves both spacecraft
//! (`addSpacecraftToModel` twice); basilisk-rs's model takes one state, so
//! there is one atmosphere per spacecraft, with the same parameters.
//!
//! | arguments | upstream `ScenarioConfig` | CSV |
//! |---|---|---|
//! | (none, `--all`) | drag, control, integral control (the tested default) | `scenarioFormationFlyingWithDrag.csv` |
//! | `--no-drag`, `--no-control`, `--no-integral-control` | the matching field false | `scenarioFormationFlyingWithDrag_<flags>.csv` |
//!
//! The CSV holds, every minute (upstream's `recordDt`), both centers of
//! mass' states, the chief's drag force and density, and the deputy's
//! control force (site frame).
//!
//! `test_scenarioMujoco.py` only runs the default case; `tests` checks
//! that the controller holds the deputy at its target element offsets
//! against the differential drag, and that without it the drag pulls the
//! formation apart. Upstream's Vizard call (`saveFile` commented out) is
//! not ported.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/mj_scenarios.rs"]
mod mj_scenarios;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/shared.rs"]
mod shared;

use basilisk_rs::connect;
use basilisk_rs::dynamics::dyn_output::orb_elem_convert::{OrbElemConvert, OrbElemConvertConfig};
use basilisk_rs::environment::atmosphere::exponential_atmosphere::{
    ExponentialAtmosphere, ExponentialAtmosphereConfig,
};
use basilisk_rs::fsw_algorithms::orbit_control::orb_elem_offset::{
    OrbElemOffset, OrbElemOffsetConfig,
};
use basilisk_rs::messages::{
    AtmosphereMsg, BodyForceCommandMsg, ClassicElementsMsg, DragGeometryMsg, Output,
    SpacecraftStateMsg,
};
use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::mujoco_dynamics::cannonball_drag::CannonballDrag;
use basilisk_rs::mujoco_dynamics::cmd_force_inertial_to_force_at_site::CmdForceInertialToForceAtSite;
use basilisk_rs::mujoco_dynamics::orbital_element_control::OrbitalElementControl;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{MU_EARTH, REQ_EARTH};
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, eccentric_to_mean_anomaly, eccentric_to_true_anomaly, elements_to_state,
    mean_to_eccentric_anomaly, state_to_elements, true_to_eccentric_anomaly,
};
use cli::Cli;
use common::{seconds, vector_columns};
use mj_scenarios::{
    GravitySourceSpec, add_gravity_to_all_bodies, epoch, rkf45, scalar_column, site_state_columns,
    written_csv,
};
use nalgebra::{DMatrix, Vector3};
use orbit_tools::{orbit_period_s, sec2nano};
use recording::{History, scenario_csv};
use shared::Shared;

const USAGE: &str = "\
scenario_formation_flying_with_drag [--no-drag] [--no-control] [--no-integral-control]
scenario_formation_flying_with_drag --all     run upstream's tested case";

/// Upstream `dt`.
const STEP_NANOS: u64 = seconds(10);
/// Upstream `recordDt` (1 min).
const RECORD_NANOS: u64 = seconds(60);
/// Upstream `kProp` and `kInt`.
const K_PROP: [f64; 6] = [1e5, 1e5, 0.0, 0.0, 10.0, 10.0];
const K_INT: [f64; 6] = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0];
/// Upstream `targetOeDiffPayload.e`; the other offsets are zero.
const TARGET_DE: f64 = 0.0005;
/// Upstream `initialDeltaOe`: δa/a, δe, δi, δΩ, δω, δM.
const INITIAL_DELTA_OE: [f64; 6] = [0.0001, 0.0002, -0.0003, 0.0005, 0.0005, 0.0006];

/// Upstream `CHIEF_DEPUTY_SCENE_XML`.
const CHIEF_DEPUTY_SCENE_XML: &str = r#"
<mujoco>
    <worldbody>
        <body name="chief">
            <joint name="chief_x" axis="1 0 0" type="slide"/>
            <joint name="chief_y" axis="0 1 0" type="slide"/>
            <joint name="chief_z" axis="0 0 1" type="slide"/>
            <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        </body>
        <body name="deputy">
            <joint name="deputy_x" axis="1 0 0" type="slide"/>
            <joint name="deputy_y" axis="0 1 0" type="slide"/>
            <joint name="deputy_z" axis="0 0 1" type="slide"/>
            <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        </body>
    </worldbody>
</mujoco>
"#;

/// Upstream `ScenarioConfig`.
#[derive(Clone, Copy, Debug)]
struct Config {
    drag: bool,
    control: bool,
    integral_control: bool,
}

impl Config {
    const DEFAULT: Self = Self {
        drag: true,
        control: true,
        integral_control: true,
    };

    fn file_stem(&self) -> String {
        let mut stem = "scenarioFormationFlyingWithDrag".to_string();
        for (on, name) in [
            (self.drag, "noDrag"),
            (self.control, "noControl"),
            (self.integral_control, "noIntegralControl"),
        ] {
            if !on {
                stem.push('_');
                stem.push_str(name);
            }
        }
        stem
    }
}

/// Upstream `simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")`.
fn exponential_atmosphere(name: &str) -> ExponentialAtmosphere {
    ExponentialAtmosphere::new(ExponentialAtmosphereConfig {
        name: name.to_string(),
        planet_radius_m: 6_378_136.6,
        reference_altitude_m: 0.0,
        reference_density_kgpm3: 1.217,
        scale_height_m: 8_500.0,
    })
}

/// A cannonball drag at `body`'s center of mass, with its geometry.
fn add_drag(
    scene: &mut MjScene,
    body: &str,
    drag_coeff: f64,
    density: &Output<AtmosphereMsg>,
) -> Output<BodyForceCommandMsg> {
    let mut drag = CannonballDrag {
        name: format!("drag{}", if body == "chief" { "Chief" } else { "Deputy" }),
        ..CannonballDrag::default()
    };
    drag.apply_to_body(scene, body);
    let geometry = Output::new(DragGeometryMsg {
        projected_area_m2: 1.0,
        drag_coeff,
        r_cp_s_m: Vector3::zeros(),
    });
    geometry.connect_to(&mut drag.drag_geometry_in_msg);
    density.connect_to(&mut drag.atmo_dens_in_msg);
    let force = drag.force_out_msg.clone();
    scene.add_model_to_dynamics_task(drag, -1);
    force
}

/// The initial chief and deputy orbits (upstream `setInitialConditions`)
/// and the orbit period \[s\].
fn initial_orbits(mu: f64, radius: f64) -> (ClassicElements, ClassicElements, f64) {
    let periapsis = radius + 250.0 * 1000.0;
    let apoapsis = radius + 300.0 * 1000.0;
    let a = (periapsis + apoapsis) / 2.0;
    let chief = ClassicElements {
        semi_major_axis: a,
        eccentricity: 1.0 - periapsis / a,
        inclination_rad: 33.3_f64.to_radians(),
        raan_rad: 48.2_f64.to_radians(),
        arg_periapsis_rad: 347.8_f64.to_radians(),
        true_anomaly_rad: 85.3_f64.to_radians(),
        ..Default::default()
    };
    let d = INITIAL_DELTA_OE;
    let mean_chief = eccentric_to_mean_anomaly(
        true_to_eccentric_anomaly(chief.true_anomaly_rad, chief.eccentricity),
        chief.eccentricity,
    );
    let e_deputy = chief.eccentricity + d[1];
    let deputy = ClassicElements {
        semi_major_axis: chief.semi_major_axis * (1.0 + d[0]),
        eccentricity: e_deputy,
        inclination_rad: chief.inclination_rad + d[2],
        raan_rad: chief.raan_rad + d[3],
        arg_periapsis_rad: chief.arg_periapsis_rad + d[4],
        true_anomaly_rad: eccentric_to_true_anomaly(
            mean_to_eccentric_anomaly(mean_chief + d[5], e_deputy),
            e_deputy,
        ),
        ..Default::default()
    };
    (chief, deputy, orbit_period_s(mu, a))
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    chief: Vec<(u64, SpacecraftStateMsg)>,
    deputy: Vec<(u64, SpacecraftStateMsg)>,
    control: Vec<(u64, BodyForceCommandMsg)>,
    mu: f64,
}

fn run(config: Config, write_csv: bool) -> Results {
    let mu = MU_EARTH * 1e9;
    let radius = REQ_EARTH * 1e3;
    let mut scene = MjScene::new(CHIEF_DEPUTY_SCENE_XML).expect("valid chief-deputy scene");
    scene.extra_eom_call = true;
    scene.set_integrator(rkf45(1e-5, 1e-5));
    add_gravity_to_all_bodies(
        &mut scene,
        &[GravitySourceSpec {
            name: "earth",
            mu_m3ps2: mu,
            is_central_body: true,
            ephemeris: None,
        }],
    );
    let chief_com = scene.body("chief").center_of_mass().state_out_msg.clone();
    let deputy_com = scene.body("deputy").center_of_mass().state_out_msg.clone();

    // The atmosphere at each spacecraft.
    let mut densities = Vec::new();
    for (name, state) in [("ExpAtmoChief", &chief_com), ("ExpAtmoDeputy", &deputy_com)] {
        let mut atmosphere = exponential_atmosphere(name);
        state.connect_to(&mut atmosphere.input_state_msg);
        densities.push(atmosphere.output_atmosphere_msg.clone());
        scene.add_module_to_dynamics_task(name, atmosphere, -1);
    }
    let chief_drag = config
        .drag
        .then(|| add_drag(&mut scene, "chief", 30.0, &densities[0]));
    if config.drag {
        add_drag(&mut scene, "deputy", 100.0, &densities[1]);
    }

    // Classical-element feedback on the deputy.
    let control_force = config.control.then(|| {
        let mut converters = Vec::new();
        for (name, state) in [
            ("orbElemConvertChief", &chief_com),
            ("orbElemConvertDeputy", &deputy_com),
        ] {
            let mut converter = OrbElemConvert::new(OrbElemConvertConfig {
                name: name.to_string(),
                mu_m3ps2: mu,
            });
            state.connect_to(&mut converter.sc_state_in_msg);
            converters.push(converter.elem_out_msg.clone());
            scene.add_module_to_dynamics_task(name, converter, -1);
        }
        let offset = Output::new(ClassicElementsMsg {
            semi_major_axis: 0.0,
            eccentricity: TARGET_DE,
            ..Default::default()
        });
        let mut target = OrbElemOffset::new(OrbElemOffsetConfig {
            name: "orbElemTarget".to_string(),
            use_mean_anomaly_offset: true,
        });
        converters[0].connect_to(&mut target.main_elements_in_msg);
        offset.connect_to(&mut target.offset_elements_in_msg);
        let mut feedback = OrbitalElementControl {
            name: "oeFeedback".to_string(),
            mu,
            ..OrbitalElementControl::default()
        };
        feedback.set_proportional_gain(DMatrix::from_diagonal(&nalgebra::DVector::from_row_slice(
            &K_PROP,
        )));
        if config.integral_control {
            feedback.set_integral_gain(DMatrix::from_diagonal(&nalgebra::DVector::from_row_slice(
                &K_INT,
            )));
        }
        target
            .elements_out_msg
            .connect_to(&mut feedback.target_oe_in_msg);
        converters[1].connect_to(&mut feedback.current_oe_in_msg);
        // The offset message stays alive with the target module's input.
        scene.add_module_to_dynamics_task("orbElemTarget", target, -1);
        drop(offset);
        let site = scene.body("deputy").center_of_mass().name().to_string();
        let mut to_site = CmdForceInertialToForceAtSite {
            name: "inertialToSiteForce".to_string(),
            ..CmdForceInertialToForceAtSite::default()
        };
        feedback
            .force_out_msg
            .connect_to(&mut to_site.cmd_force_inertial_in_msg);
        deputy_com.connect_to(&mut to_site.site_frame_state_in_msg);
        to_site.force_out_msg.connect_to(
            &mut scene
                .add_force_actuator("deputyControl", &site)
                .force_in_msg,
        );
        let force = to_site.force_out_msg.clone();
        scene.add_model_to_dynamics_task(feedback, -1);
        scene.add_model_to_dynamics_task(to_site, -1);
        force
    });

    let (chief_elements, deputy_elements, period) = initial_orbits(mu, radius);
    let (chief_r, chief_v) = elements_to_state(mu, &chief_elements);
    let (deputy_r, deputy_v) = elements_to_state(mu, &deputy_elements);

    let mut chief_log = History::<SpacecraftStateMsg>::new();
    let mut deputy_log = History::<SpacecraftStateMsg>::new();
    let mut control_log = History::<BodyForceCommandMsg>::new();
    let mut csv = write_csv.then(|| scenario_csv(&config.file_stem()));
    let scene = Shared::new(scene);
    let mut scene_task = scene.handle();
    {
        let mut simulation = Simulation::new(epoch(), false);
        connect!(&simulation,
            &chief_com => &mut chief_log.input,
            &deputy_com => &mut deputy_log.input,
        );
        if let Some(force) = &control_force {
            connect!(&simulation, force => &mut control_log.input);
        }
        simulation.add_module("scene", &mut scene_task, STEP_NANOS, 1);
        simulation.add_module("chiefStateRecorder", &mut chief_log, RECORD_NANOS, 0);
        simulation.add_module("deputyStateRecorder", &mut deputy_log, RECORD_NANOS, 0);
        simulation.add_module("oeFeedbackForceRecorder", &mut control_log, RECORD_NANOS, 0);
        if let Some(csv) = csv.as_mut() {
            connect!(&simulation,
                &chief_com => csv.add_source::<SpacecraftStateMsg>(site_state_columns("chief")),
                &deputy_com => csv.add_source::<SpacecraftStateMsg>(site_state_columns("deputy")),
            );
            if let Some(force) = &chief_drag {
                connect!(&simulation,
                    force => csv.add_source::<BodyForceCommandMsg>(CsvSourceConfig::columns(
                        vector_columns("force_request_body_n", "chief_drag_force_S_N"),
                    )),
                    &densities[0] => csv.add_source::<AtmosphereMsg>(
                        scalar_column("neutral_density_kgpm3", "chief_neutralDensity_kg_per_m3"),
                    ),
                );
            }
            if let Some(force) = &control_force {
                connect!(&simulation,
                    force => csv.add_source::<BodyForceCommandMsg>(CsvSourceConfig::columns(
                        vector_columns("force_request_body_n", "deputy_control_force_S_N"),
                    )),
                );
            }
            simulation.add_module("csv", csv, RECORD_NANOS, 0);
        }
        simulation.initialize();
        {
            let mut scene = scene.lock();
            {
                let mut chief = scene.body_mut("chief");
                chief.set_position(chief_r);
                chief.set_velocity(chief_v);
            }
            let mut deputy = scene.body_mut("deputy");
            deputy.set_position(deputy_r);
            deputy.set_velocity(deputy_v);
        }
        simulation.run_for(sec2nano(10.0 * period));
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    Results {
        chief: chief_log.into_samples(),
        deputy: deputy_log.into_samples(),
        control: control_log.into_samples(),
        mu,
    }
}

impl Results {
    /// Upstream's `oeDiff` at sample `index`: δa \[m\], δe, and the angle
    /// differences (mean anomaly for the last) wrapped to ±180° \[deg\].
    fn element_difference(&self, index: usize) -> [f64; 6] {
        let elements = |state: &SpacecraftStateMsg| {
            let oe = state_to_elements(self.mu, state.position_m, state.velocity_mps);
            let mean = eccentric_to_mean_anomaly(
                true_to_eccentric_anomaly(oe.true_anomaly_rad, oe.eccentricity),
                oe.eccentricity,
            );
            [
                oe.semi_major_axis,
                oe.eccentricity,
                oe.inclination_rad.to_degrees(),
                oe.raan_rad.to_degrees(),
                oe.arg_periapsis_rad.to_degrees(),
                mean.to_degrees(),
            ]
        };
        let chief = elements(&self.chief[index].1);
        let deputy = elements(&self.deputy[index].1);
        let mut difference = [0.0; 6];
        for k in 0..6 {
            difference[k] = deputy[k] - chief[k];
            if k >= 2 {
                difference[k] = (difference[k] + 180.0).rem_euclid(360.0) - 180.0;
            }
        }
        difference
    }
}

fn main() {
    let mut cli = Cli::from_env(USAGE);
    let config = if cli.switch("all") {
        Config::DEFAULT
    } else {
        Config {
            drag: cli.flag("drag", true),
            control: cli.flag("control", true),
            integral_control: cli.flag("integral-control", true),
        }
    };
    cli.finish();
    let results = run(config, true);
    let last = results.chief.len() - 1;
    println!(
        "final element difference (da [m], de, di, dOmega, domega, dM [deg]): {:.4?}",
        results.element_difference(last)
    );
    println!("wrote {}", written_csv(&config.file_stem()).display());
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` only runs the default case. These check the
    //! formation control it demonstrates.

    use super::*;

    #[test]
    fn the_controller_holds_the_formation_against_differential_drag() {
        let results = run(Config::DEFAULT, false);
        let last = results.chief.len() - 1;
        let difference = results.element_difference(last);
        // The controlled elements reach their targets: δa → 0 (integral
        // action against the drag), δe → 0.0005, δω and δM → 0.
        assert!(difference[0].abs() < 50.0, "da {}", difference[0]);
        assert!(
            (difference[1] - TARGET_DE).abs() < 1e-4,
            "de {}",
            difference[1]
        );
        assert!(
            difference[4].abs() < 0.05 && difference[5].abs() < 0.05,
            "{difference:?}"
        );
        // The control force stays finite and acts.
        assert!(
            results
                .control
                .iter()
                .any(|(_, force)| force.force_request_body_n.norm() > 0.0)
        );
        assert!(
            results
                .control
                .iter()
                .all(|(_, force)| force.force_request_body_n.norm().is_finite())
        );
    }

    #[test]
    fn without_control_the_drag_pulls_the_formation_apart() {
        let config = Config {
            control: false,
            ..Config::DEFAULT
        };
        let results = run(config, false);
        let last = results.chief.len() - 1;
        let start = results.element_difference(0);
        let end = results.element_difference(last);
        // The deputy's larger drag lowers its orbit faster than the chief's.
        assert!(end[0] < start[0] - 100.0, "da {} -> {}", start[0], end[0]);
        let separation =
            (results.chief[last].1.position_m - results.deputy[last].1.position_m).norm();
        let initial = (results.chief[0].1.position_m - results.deputy[0].1.position_m).norm();
        assert!(separation > 2.0 * initial, "{initial} -> {separation}");
    }
}
