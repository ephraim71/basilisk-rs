//! Port of upstream `examples/scenarioStochasticDragSpacecraft.py`: a 1 kg
//! spacecraft with 10 m² of cannonball drag on a 250 km circular orbit, in an
//! exponential atmosphere whose density is scaled by a stochastic correction
//! integrated with the spacecraft, for 7.1 orbits at 10 s steps.
//!
//! | `--igbm` | correction state | integrator | CSV |
//! |---|---|---|---|
//! | no (default) | Ornstein-Uhlenbeck, [`MeanRevertingNoiseStateEffector`] | [`Sosra`] | `scenarioStochasticDragSpacecraft.csv` |
//! | yes | inhomogeneous geometric Brownian motion, [`IgbmNoiseStateEffector`], mean 1 | [`Sriw1`] | `scenarioStochasticDragSpacecraft_igbm.csv` |
//!
//! Both use a stationary standard deviation of 0.15 and a 108 s time
//! constant. `--all` runs both, as `test_scenarioStochasticDragSpacecraft.py`
//! does; `--seed` sets the integrator seed (upstream `rngSeed`; basilisk-rs's
//! generator is not upstream's, so a realization never matches upstream's
//! draw for draw). The wind case is not tested upstream and not ported.
//!
//! Upstream's drag effector takes the correction by state name
//! (`densityCorrectionStateName`) and uses the density
//! `max(0, (1 + x) rho)`. basilisk-rs's [`DragDynamicEffector`] has no such
//! input: its loads depend only on the hub state it is handed. This example
//! couples the two itself: [`CorrectionTap`] wraps the noise state effector
//! and publishes the stage value of its state each time the spacecraft loads
//! one (before it evaluates any dynamic effector at that stage), and
//! [`CorrectedDrag`] scales the cannonball force, linear in the density, by
//! `max(0, 1 + x)`.
//!
//! The CSV holds the state, the drag force (the spacecraft diagnostics,
//! evaluated at the committed state of each step, where upstream logs the
//! force of the last integrator stage) and the exponential atmosphere's
//! density, every step; as upstream, the stochastic density follows from
//! the drag force.
//!
//! `test_scenarioStochasticDragSpacecraft.py` only runs both cases; `tests`
//! checks the statistics of the correction against the configured
//! stationary process and that the orbit decays.

#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/scenario_args.rs"]
mod scenario_args;

use std::any::Any;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use basilisk_rs::dynamics::drag_dynamic_effector::{
    DragDynamicEffector, DragDynamicEffectorConfig,
};
use basilisk_rs::dynamics::igbm_noise_state_effector::{
    IgbmNoiseStateEffector, IgbmNoiseStateEffectorConfig,
};
use basilisk_rs::dynamics::mean_reverting_noise_state_effector::{
    MeanRevertingNoiseStateEffector, MeanRevertingNoiseStateEffectorConfig,
};
use basilisk_rs::environment::atmosphere::exponential_atmosphere::{
    ExponentialAtmosphere, ExponentialAtmosphereConfig,
};
use basilisk_rs::integrators::Integrator;
use basilisk_rs::integrators::stochastic::{Sosra, Sriw1};
use basilisk_rs::messages::{AtmosphereMsg, SpacecraftDiagnosticsMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{
    BackSubMatrices, DynamicEffector, EffectorOutput, Spacecraft, SpacecraftConfig, StateEffector,
    StateEffectorMassProps,
};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use basilisk_rs::{connect, schedule};
use common::vector_columns;
use grav_factory::Body;
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{orbit_period_s, sec2nano};
use recording::{History, scenario_csv, state_columns};
use scenario_args::ScenarioArgs;

const USAGE: &str = "\
scenario_stochastic_drag_spacecraft [--igbm] [--seed N]
scenario_stochastic_drag_spacecraft --all     run every upstream test case";

/// Upstream `dt`.
const STEP_NANOS: u64 = 10_000_000_000;
/// Upstream `setStationaryStd`.
pub const STATIONARY_STD: f64 = 0.15;
/// Upstream `setTimeConstant` (s).
pub const TIME_CONSTANT_S: f64 = 1.8 * 60.0;

/// The stage value of a state effector's first state, shared between
/// [`CorrectionTap`] and [`CorrectedDrag`].
#[derive(Clone, Debug, Default)]
pub struct SharedValue(Arc<AtomicU64>);

impl SharedValue {
    fn set(&self, value: f64) {
        self.0.store(value.to_bits(), Ordering::Relaxed);
    }

    pub fn get(&self) -> f64 {
        f64::from_bits(self.0.load(Ordering::Relaxed))
    }
}

/// A state effector that publishes the value of its (first) state each
/// time the spacecraft loads one into it: the stand-in for upstream's
/// `states.getStateObject(densityCorrectionStateName)`.
#[derive(Debug)]
pub struct CorrectionTap<E> {
    pub inner: E,
    value: SharedValue,
}

impl<E: StateEffector> CorrectionTap<E> {
    pub fn new(inner: E) -> (Self, SharedValue) {
        let value = SharedValue::default();
        value.set(inner.initial_state()[0]);
        (
            Self {
                inner,
                value: value.clone(),
            },
            value,
        )
    }
}

impl<E: StateEffector + 'static> StateEffector for CorrectionTap<E> {
    fn name(&self) -> &str {
        self.inner.name()
    }
    fn state_len(&self) -> usize {
        self.inner.state_len()
    }
    fn initial_state(&self) -> Vec<f64> {
        self.inner.initial_state()
    }
    fn load_state(&mut self, state: &[f64]) {
        self.value.set(state[0]);
        self.inner.load_state(state);
    }
    fn pre_integration(&mut self, current_sim_nanos: u64, dt_seconds: f64) {
        self.inner.pre_integration(current_sim_nanos, dt_seconds);
    }
    fn update_contributions(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        self.inner.update_contributions(
            effector_state,
            body_omega_radps,
            gravity_body_mps2,
            back_sub,
        );
    }
    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        self.inner
            .compute_derivatives(effector_state, body_trans_accel_mps2, body_omega_dot_radps2)
    }
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        self.inner.mass_properties(effector_state)
    }
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        self.inner
            .rotational_angular_momentum_body(effector_state, body_omega_radps)
    }
    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        self.inner
            .rotational_energy_j(effector_state, body_omega_radps)
    }
    fn write_outputs(&mut self, current_sim_nanos: u64, hub_state: &SpacecraftStateMsg) {
        self.inner.write_outputs(current_sim_nanos, hub_state);
    }
    fn modify_states(&mut self, current_sim_nanos: u64, effector_state: &mut [f64]) {
        self.inner.modify_states(current_sim_nanos, effector_state);
    }
    fn as_any(&self) -> &dyn Any {
        self.inner.as_any()
    }
    fn begin_stage(&mut self, stage_nanos: u64) {
        self.inner.begin_stage(stage_nanos);
    }
    fn noise_source_count(&self) -> usize {
        self.inner.noise_source_count()
    }
    fn compute_diffusion(&self, effector_state: &[f64]) -> Vec<Vec<f64>> {
        self.inner.compute_diffusion(effector_state)
    }
}

/// Upstream `DragDynamicEffector` with `densityCorrectionStateName` set:
/// the cannonball force for the density `max(0, (1 + x) rho)`.
#[derive(Debug)]
pub struct CorrectedDrag {
    pub inner: DragDynamicEffector,
    correction: SharedValue,
}

impl DynamicEffector for CorrectedDrag {
    fn name(&self) -> &str {
        DynamicEffector::name(&self.inner)
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let factor = (1.0 + self.correction.get()).max(0.0);
        let output = self.inner.compute_output(state);
        EffectorOutput {
            force_inertial_n: output.force_inertial_n * factor,
            torque_body_nm: output.torque_body_nm * factor,
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// Upstream `run(showPlots, rngSeed, useWind=False, useIgbm)`.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub igbm: bool,
    pub seed: Option<u64>,
}

impl Config {
    pub fn file_stem(&self) -> &'static str {
        if self.igbm {
            "scenarioStochasticDragSpacecraft_igbm"
        } else {
            "scenarioStochasticDragSpacecraft"
        }
    }
}

/// `test_scenarioStochasticDragSpacecraft.py`'s runs: `{}` and
/// `{"useIgbm": True}`.
pub const TEST_CASES: [Config; 2] = [
    Config {
        igbm: false,
        seed: None,
    },
    Config {
        igbm: true,
        seed: None,
    },
];

pub struct Results {
    pub csv_path: std::path::PathBuf,
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    /// The density correction `x` at every step.
    pub corrections: Vec<f64>,
    pub mu_m3ps2: f64,
}

/// Records the committed correction at each update, after the spacecraft.
struct CorrectionLog {
    value: SharedValue,
    samples: Vec<f64>,
}

impl basilisk_rs::Module for CorrectionLog {
    fn init(&mut self) {}

    fn update(&mut self, _context: &basilisk_rs::SimulationContext) {
        self.samples.push(self.value.get());
    }
}

pub fn run(config: Config) -> Results {
    let earth = Body::Earth;
    let mu = earth.mu_m3ps2();
    let degrees = std::f64::consts::PI / 180.0;
    let (position_m, velocity_mps) = elements_to_state(
        mu,
        &ClassicElements {
            semi_major_axis: earth.radius_equator_m() + 250.0 * 1000.0,
            eccentricity: 0.0,
            inclination_rad: 33.3 * degrees,
            raan_rad: 48.2 * degrees,
            arg_periapsis_rad: 347.8 * degrees,
            true_anomaly_rad: 85.3 * degrees,
            ..Default::default()
        },
    );
    let elements = state_to_elements(mu, position_m, velocity_mps);
    let duration_nanos = sec2nano(7.1 * orbit_period_s(mu, elements.semi_major_axis));

    let integrator: Box<dyn Integrator<Spacecraft>> = match (config.igbm, config.seed) {
        (false, None) => Box::new(Sosra::new()),
        (false, Some(seed)) => Box::new(Sosra::new().with_seed(seed)),
        (true, None) => Box::new(Sriw1::new()),
        (true, Some(seed)) => Box::new(Sriw1::new().with_seed(seed)),
    };
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: Some(integrator),
    });
    spacecraft
        .add_grav_body(earth.point_mass(true))
        .expect("one central gravity body");

    // Upstream `simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")`.
    let mut atmosphere = ExponentialAtmosphere::new(ExponentialAtmosphereConfig {
        name: "ExpAtmo".to_string(),
        planet_radius_m: 6_378_136.6,
        reference_altitude_m: 0.0,
        reference_density_kgpm3: 1.217,
        scale_height_m: 8_500.0,
    });
    let correction = if config.igbm {
        let mut noise = IgbmNoiseStateEffectorConfig::new("StochasticExpAtmo");
        noise.mean = 1.0;
        noise.stationary_std = STATIONARY_STD;
        noise.time_constant_s = TIME_CONSTANT_S;
        let (tap, value) = CorrectionTap::new(IgbmNoiseStateEffector::new(noise));
        spacecraft.add_state_effector(tap);
        value
    } else {
        let mut noise = MeanRevertingNoiseStateEffectorConfig::new("StochasticExpAtmo");
        noise.stationary_std = STATIONARY_STD;
        noise.time_constant_s = TIME_CONSTANT_S;
        let (tap, value) = CorrectionTap::new(MeanRevertingNoiseStateEffector::new(noise));
        spacecraft.add_state_effector(tap);
        value
    };
    let mut drag = DragDynamicEffector::new(DragDynamicEffectorConfig {
        name: "DragEff".to_string(),
        projected_area_m2: 10.0,
        drag_coeff: 2.2,
        com_offset_m: Vector3::zeros(),
        planet_rotation_rate_radps: Vector3::zeros(),
    });
    let mut correction_log = CorrectionLog {
        value: correction.clone(),
        samples: Vec::new(),
    };

    let mut csv = scenario_csv(config.file_stem());
    let csv_path = csv.config.output_path.clone();
    let mut state_log = History::<SpacecraftStateMsg>::new();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        connect!(&simulation,
            &atmosphere.output_atmosphere_msg => &mut drag.input_atmosphere_msg,
            &spacecraft.state_out => &mut atmosphere.input_state_msg,
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &spacecraft.diagnostics_out => csv.add_source::<SpacecraftDiagnosticsMsg>(
                CsvSourceConfig::columns(vector_columns("drag_force_body_n", "forceExternal_B_N")),
            ),
            &atmosphere.output_atmosphere_msg => csv.add_source::<AtmosphereMsg>(
                CsvSourceConfig::columns([(
                    "neutral_density_kgpm3",
                    "neutralDensity_kg_per_m3",
                )]),
            ),
            &spacecraft.state_out => &mut state_log.input,
        );
        spacecraft.add_dynamic_effector(CorrectedDrag {
            inner: drag,
            correction,
        });
        schedule! { simulation,
            "spacecraft" => &mut spacecraft, period=STEP_NANOS, priority=20;
            "atmosphere" => &mut atmosphere, period=STEP_NANOS, priority=15;
            "correction_log" => &mut correction_log, period=STEP_NANOS, priority=10;
            "csv" => &mut csv, period=STEP_NANOS, priority=0;
            "state_log" => &mut state_log, period=STEP_NANOS, priority=0;
        }
        simulation.run_for(duration_nanos);
    }
    csv.flush().expect("flush scenario CSV");
    Results {
        csv_path,
        states: state_log.into_samples(),
        corrections: correction_log.samples,
        mu_m3ps2: mu,
    }
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let seed = args.flag("--seed").then(|| args.value("--seed", 0_u64));
    let configs = if args.flag("--all") {
        TEST_CASES.to_vec()
    } else {
        vec![Config {
            igbm: args.flag("--igbm"),
            seed,
        }]
    };
    for config in configs {
        let results = run(config);
        let first = results.states[0].1.position_m.norm();
        let last = results.states.last().unwrap().1.position_m.norm();
        println!(
            "wrote {} (radius {:.3} -> {:.3} km)",
            results.csv_path.display(),
            first / 1000.0,
            last / 1000.0
        );
    }
}

#[cfg(test)]
mod tests {
    use super::{STATIONARY_STD, TEST_CASES, run};

    /// `test_scenarioStochasticDragSpacecraft.py` runs both cases without
    /// checks. The correction must follow its stationary process (mean of
    /// the factor `1 + x` near 1, standard deviation near 0.15, over about
    /// 360 time constants) and drag must lower the orbit.
    #[test]
    fn correction_statistics_and_orbit_decay() {
        for config in TEST_CASES {
            let results = run(config);
            let n = results.corrections.len() as f64;
            let mean = results.corrections.iter().sum::<f64>() / n;
            let std = (results
                .corrections
                .iter()
                .map(|x| (x - mean).powi(2))
                .sum::<f64>()
                / n)
                .sqrt();
            assert!(mean.abs() < 0.03, "{}: mean {mean}", config.file_stem());
            assert!(
                (std / STATIONARY_STD - 1.0).abs() < 0.2,
                "{}: std {std}",
                config.file_stem()
            );
            assert!(results.corrections.iter().any(|x| *x != 0.0));
            let first = results.states[0].1.position_m.norm();
            let last = results.states.last().unwrap().1.position_m.norm();
            assert!(
                last < first - 1000.0,
                "{}: {first} -> {last}",
                config.file_stem()
            );
        }
    }
}
