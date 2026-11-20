//! Translation and non-Keplerian gravity estimation about Vesta with
//! `smallBodyNavUKF`: port of upstream `examples/scenarioSmallBodyNavUKF.py`.
//!
//! Vesta (`mu = 17.2882449693e9 m^3/s^2`) is the central body with upstream's
//! `VESTA20H.txt` spherical-harmonic field to degree 14, rotating with its
//! January 21st, 2022 pole (right ascension 309.03 deg, declination
//! 42.23 deg) and 5.3421 h spin on its heliocentric orbit
//! (`planetEphemeris`); there is no Sun gravity. The spacecraft starts on a
//! 500 km, `e = 0.001`, 175 deg retrograde orbit defined in Vesta's fixed
//! frame.
//!
//! The unscented Kalman filter estimates the spacecraft position, velocity
//! and the non-Keplerian acceleration in Vesta's fixed frame from `simpleNav`
//! position measurements (10 m, 0.1 m/s errors) and a perfect `planetNav`
//! ephemeris of Vesta, starting from the true initial state with
//! `P_k = diag(1e4, 0.1, 1e-4)`, process noise `diag(1000, 1, 1e-6)`, `R =
//! 100 I`, `alpha = 0`, `beta = 2`, `kappa = 1e-3`. One 15 s task, 4 days.
//!
//! Upstream `run(show_plots)` has no parameters. Writes
//! `examples/output/scenarios/scenarioSmallBodyNavUKF.csv` with the
//! spacecraft truth, Vesta's state, the filter state and covariance diagonal,
//! and the true Vesta-fixed position, velocity and non-Keplerian
//! acceleration upstream computes after the run; `plot_scenario_small_body_nav_ukf.py`
//! redraws upstream's eight figures from it. Needs `assets/gravity/VESTA20H.txt`
//! (`just fetch-small-body-data`).
//!
//! # Differences from upstream
//!
//! * basilisk-rs's `SimpleNavigation` has no error model, so the navigation
//!   is [`small_body_simple_nav::NoisySimpleNav`], upstream's 18-state
//!   `simpleNav` errors on the crate's `GaussMarkov` (upstream's model, as
//!   `planetNav`); the noise stream is not upstream's.
//! * Upstream evaluates the true Vesta-fixed state and the non-Keplerian
//!   acceleration (`spherHarm.computeField(r, 14, False)`) from its recorded
//!   histories after the run; here [`VestaFixedTruth`] does it at every step,
//!   so the CSV carries it.
//!
//! Upstream's test only runs the scenario (`test_scenarioTest.py`); `tests`
//! checks the filter's consistency with its covariance and that it recovers
//! the non-Keplerian acceleration of the inhomogeneous field.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/rw_factory.rs"]
mod rw_factory;
#[path = "support/small_body.rs"]
mod small_body;
#[path = "support/small_body_simple_nav.rs"]
mod small_body_simple_nav;

use std::path::{Path, PathBuf};

use attitude::scenario_recorder;
use basilisk_rs::dynamics::gravity::{GravBodyData, SphericalHarmonicsGravityModel};
use basilisk_rs::environment::planet_ephemeris::{
    PlanetEphemeris, PlanetEphemerisBody, PlanetEphemerisConfig, PlanetRotation,
};
use basilisk_rs::fsw_algorithms::small_body_navigation::small_body_nav_ukf::{
    SmallBodyNavUkf, SmallBodyNavUkfConfig, SmallBodyNavUkfMatrix, SmallBodyNavUkfState,
};
use basilisk_rs::messages::{
    EphemerisMsg, Input, NavigationTranslationMsg, Output, SmallBodyNavUkfMsg, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::{CsvSourceConfig, TelemetryField, TelemetryMessage};
use basilisk_rs::utilities::astro_constants::{AU, D2R};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::utilities::rigid_body_kinematics::{euler323_to_dcm, mrp_to_dcm};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use cli::Cli;
use common::{array_columns, vector_columns};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};
use small_body::{ephemeris_converter, planet_nav};
use small_body_simple_nav::{NavSigmas, NoisySimpleNav, grouped};

const STEM: &str = "scenarioSmallBodyNavUKF";
/// Upstream `simulationTimeStep = macros.sec2nano(15)`.
const STEP_NANOS: u64 = 15_000_000_000;
/// Upstream `simulationTime = macros.sec2nano(4 * 24 * 3600.0)`.
const DURATION_NANOS: u64 = 4 * 24 * 3600 * 1_000_000_000;
/// Upstream `mu = 17.2882449693 * 1e9` \[m^3/s^2\].
const VESTA_MU_M3PS2: f64 = 17.2882449693e9;
/// Upstream `nSpherHarm = 14`.
const DEGREE: usize = 14;
/// Upstream `VESTA20H.txt` (`just fetch-small-body-data`).
const VESTA20H: &str = "assets/gravity/VESTA20H.txt";
/// Upstream `rotRate = 360 deg / (5.3421 h)` \[rad/s\].
const VESTA_ROTATION_RATE_RADPS: f64 = 360.0 * D2R / (5.3421 * 3600.0);

/// Upstream `run(...)` has no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The true spacecraft state the filter estimates, in Vesta's fixed frame
/// `A`: upstream's post-processed `r_truth_A`, `v_truth_A` and `a_truth_A`.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct VestaFixedTruthMsg {
    /// `[AN] (r_BN_N - r_AN_N)` \[m\].
    position_m: Vector3<f64>,
    /// `[AN] (v_BN_N - v_AN_N) - omega_AN_A x r` \[m/s\].
    velocity_mps: Vector3<f64>,
    /// The field without its point-mass term at the position \[m/s^2\].
    acceleration_mps2: Vector3<f64>,
}

impl TelemetryMessage for VestaFixedTruthMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        [
            ("position_m", self.position_m),
            ("velocity_mps", self.velocity_mps),
            ("acceleration_mps2", self.acceleration_mps2),
        ]
        .into_iter()
        .flat_map(|(name, vector)| {
            ["x", "y", "z"]
                .into_iter()
                .enumerate()
                .map(move |(index, axis)| TelemetryField {
                    path: format!("{name}.{axis}"),
                    value: vector[index],
                })
        })
        .collect()
    }
}

/// Upstream's after-the-run truth loop, run at every step: the spacecraft
/// relative to Vesta in Vesta's fixed frame (attitude and rate from the
/// ephemeris converter), and `spherHarm.computeField(r_truth_A, 14, False)`.
struct VestaFixedTruth {
    spacecraft_state_in_msg: Input<SpacecraftStateMsg>,
    vesta_ephemeris_in_msg: Input<EphemerisMsg>,
    truth_out_msg: Output<VestaFixedTruthMsg>,
    field: SphericalHarmonicsGravityModel,
}

impl Module for VestaFixedTruth {
    fn init(&mut self) {
        self.truth_out_msg.write(VestaFixedTruthMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.spacecraft_state_in_msg.read();
        let vesta = self.vesta_ephemeris_in_msg.read();
        let dcm_an = mrp_to_dcm(vesta.sigma_bn);
        let position_m = dcm_an * (state.position_m - vesta.position_bdy_zero_n_m);
        let velocity_mps = dcm_an * (state.velocity_mps - vesta.velocity_bdy_zero_n_mps)
            - vesta.omega_bn_b_radps.cross(&position_m);
        let acceleration_mps2 = self
            .field
            .acceleration_at_degree_mps2(position_m, DEGREE, false)
            .expect("degree within the field");
        self.truth_out_msg.write(VestaFixedTruthMsg {
            position_m,
            velocity_mps,
            acceleration_mps2,
        });
    }
}

/// Upstream `asteroid.useSphericalHarmonicsGravityModel(VESTA20H, 14)`: the
/// file's field with the body's `mu`.
fn vesta_field() -> SphericalHarmonicsGravityModel {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(VESTA20H);
    SphericalHarmonicsGravityModel::from_file(&path, DEGREE)
        .unwrap_or_else(|error| {
            panic!(
                "cannot load {} (just fetch-small-body-data): {error}",
                path.display()
            )
        })
        .with_mu_m3ps2(VESTA_MU_M3PS2)
        .expect("positive mu")
}

/// The recorded histories (upstream's recorders, every step).
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    truth: MessageLog<VestaFixedTruthMsg>,
    estimate: MessageLog<SmallBodyNavUkfMsg>,
    csv_path: Option<PathBuf>,
}

impl Results {
    /// Truth minus estimate at sample `index`.
    fn error(&self, index: usize) -> SmallBodyNavUkfState {
        let truth = &self.truth.values[index];
        let mut state = SmallBodyNavUkfState::zeros();
        state.fixed_rows_mut::<3>(0).copy_from(&truth.position_m);
        state.fixed_rows_mut::<3>(3).copy_from(&truth.velocity_mps);
        state
            .fixed_rows_mut::<3>(6)
            .copy_from(&truth.acceleration_mps2);
        state - self.estimate.values[index].state
    }

    /// The filter's one-sigma bound of state `component` at sample `index`.
    fn sigma(&self, index: usize, component: usize) -> f64 {
        self.estimate.values[index].covar[(component, component)].sqrt()
    }
}

fn run(config: Config) -> Results {
    let right_ascension = 309.03 * D2R;
    let declination = 42.23 * D2R;
    let mut ephemeris = PlanetEphemeris::new(PlanetEphemerisConfig {
        name: "vestaEphemeris".to_string(),
        bodies: vec![PlanetEphemerisBody {
            name: "vesta".to_string(),
            // Upstream `oeAsteroid`, January 21st, 2022 (JPL Horizons).
            elements: ClassicElements {
                semi_major_axis: 2.3612 * AU * 1000.0,
                eccentricity: 0.08823,
                inclination_rad: 7.1417 * D2R,
                raan_rad: 103.8 * D2R,
                arg_periapsis_rad: 151.1 * D2R,
                true_anomaly_rad: 7.0315 * D2R,
                ..Default::default()
            },
            rotation: Some(PlanetRotation {
                right_ascension_rad: right_ascension,
                declination_rad: declination,
                lst0_rad: 0.0,
                rotation_rate_radps: VESTA_ROTATION_RATE_RADPS,
            }),
        }],
        zero_base: None,
    });
    // Upstream `dcm_AN = euler3232C([AR, pi/2 - dec, lst0])` and
    // `omega_AN_A`.
    let dcm_an = euler323_to_dcm(Vector3::new(
        right_ascension,
        std::f64::consts::FRAC_PI_2 - declination,
        0.0,
    ));
    let omega_an_a = Vector3::new(0.0, 0.0, VESTA_ROTATION_RATE_RADPS);

    // Upstream `oe`, in Vesta's fixed frame.
    let (r_ca_a, v_ca_a) = elements_to_state(
        VESTA_MU_M3PS2,
        &ClassicElements {
            semi_major_axis: 500.0 * 1000.0,
            eccentricity: 0.001,
            inclination_rad: 175.0 * D2R,
            raan_rad: 48.2 * D2R,
            arg_periapsis_rad: 347.8 * D2R,
            true_anomaly_rad: 85.3 * D2R,
            ..Default::default()
        },
    );
    // Upstream leaves the hub mass properties at their defaults.
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: dcm_an.transpose() * r_ca_a,
        initial_velocity_mps: dcm_an.transpose() * v_ca_a,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(
            GravBodyData::new("vesta", vesta_field())
                .expect("named gravity body")
                .central(),
        )
        .expect("one central body");

    let mut converter = ephemeris_converter("vesta");
    let mut navigation = NoisySimpleNav::new(
        "SimpleNav",
        NavSigmas {
            position: 10.0,
            velocity: 0.1,
            ..NavSigmas::default()
        }
        .p_matrix(),
        grouped([10.0, 0.1, 0.0, 0.0, 0.0, 0.0]),
    );
    // Upstream: "perfect state knowledge is assumed".
    let mut vesta_navigation = planet_nav("planetNav", [0.0; 4], [0.0; 4]);

    let mut ukf = SmallBodyNavUkf::new(SmallBodyNavUkfConfig {
        name: "smallBodyNavUKF".to_string(),
        mu_ast_m3ps2: VESTA_MU_M3PS2,
        process_noise: SmallBodyNavUkfMatrix::from_diagonal(&SmallBodyNavUkfState::from_fn(
            |index, _| [1000.0, 1.0, 1e-6][index / 3],
        )),
        measurement_noise: 100.0 * Matrix3::identity(),
        alpha: 0.0,
        beta: 2.0,
        kappa: 1e-3,
    });
    let mut initial = SmallBodyNavUkfState::zeros();
    initial.fixed_rows_mut::<3>(0).copy_from(&r_ca_a);
    initial
        .fixed_rows_mut::<3>(3)
        .copy_from(&(v_ca_a - omega_an_a.cross(&r_ca_a)));
    ukf.x_hat_k = initial;
    ukf.p_k = SmallBodyNavUkfMatrix::from_diagonal(&SmallBodyNavUkfState::from_fn(|index, _| {
        [1e4, 0.1, 1e-4][index / 3]
    }));

    let mut truth = VestaFixedTruth {
        spacecraft_state_in_msg: Input::default(),
        vesta_ephemeris_in_msg: Input::default(),
        truth_out_msg: Output::default(),
        field: vesta_field(),
    };
    let mut truth_log = MessageLog::<VestaFixedTruthMsg>::default();
    let mut estimate_log = MessageLog::<SmallBodyNavUkfMsg>::default();
    let (mut csv, csv_path) = scenario_recorder(STEM, config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2022, 1, 21), false);
    connect!(&simulation,
        &ephemeris.planet_out_msgs[0] => spacecraft.grav_body_input_mut("vesta").expect("vesta"),
        &ephemeris.planet_out_msgs[0] => &mut converter.planet_in_msgs[0],
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &converter.ephem_out_msgs[0] => &mut vesta_navigation.ephemeris_in_msg,
        &navigation.translation_out_msg => &mut ukf.nav_trans_in_msg,
        &vesta_navigation.ephemeris_out_msg => &mut ukf.asteroid_ephemeris_in_msg,
        &spacecraft.state_out => &mut truth.spacecraft_state_in_msg,
        &converter.ephem_out_msgs[0] => &mut truth.vesta_ephemeris_in_msg,
        &truth.truth_out_msg => &mut truth_log.input,
        &ukf.small_body_nav_ukf_out_msg => &mut estimate_log.input,
        &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
            vector_columns("position_m", "r_BN_N_m")
                .into_iter()
                .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s")),
        )),
        &converter.ephem_out_msgs[0] => csv.add_source::<EphemerisMsg>(CsvSourceConfig::columns(
            vector_columns("position_bdy_zero_n_m", "vesta_r_BdyZero_N_m")
                .into_iter()
                .chain(vector_columns("velocity_bdy_zero_n_mps", "vesta_v_BdyZero_N_m_per_s"))
                .chain(vector_columns("sigma_bn", "sigma_AN"))
                .chain(vector_columns("omega_bn_b_radps", "omega_AN_A_rad_per_s")),
        )),
        &navigation.translation_out_msg => csv.add_source::<NavigationTranslationMsg>(
            CsvSourceConfig::columns(vector_columns("position_bn_n_m", "r_BN_N_meas_m")),
        ),
        &ukf.small_body_nav_ukf_out_msg => csv.add_source::<SmallBodyNavUkfMsg>(
            CsvSourceConfig::columns(
                array_columns("state", "x_hat", 9)
                    .into_iter()
                    .chain((1..=9).map(|index| {
                        (format!("covar.m{index}{index}"), format!("P_{}", index - 1))
                    })),
            ),
        ),
        &truth.truth_out_msg => csv.add_source::<VestaFixedTruthMsg>(CsvSourceConfig::columns(
            vector_columns("position_m", "r_truth_A_m")
                .into_iter()
                .chain(vector_columns("velocity_mps", "v_truth_A_m_per_s"))
                .chain(vector_columns("acceleration_mps2", "a_truth_A_m_per_s2")),
        )),
    );
    // Upstream's priorities.
    schedule! { simulation,
        "vestaEphemeris" => &mut ephemeris, period=STEP_NANOS, priority=100;
        "ephemConverter" => &mut converter, period=STEP_NANOS, priority=99;
        "bskSat" => &mut spacecraft, period=STEP_NANOS, priority=98;
        "SimpleNav" => &mut navigation, period=STEP_NANOS, priority=97;
        "planetNav" => &mut vesta_navigation, period=STEP_NANOS, priority=96;
        "smallBodyNavUKF" => &mut ukf, period=STEP_NANOS, priority=95;
        "truth" => &mut truth, period=STEP_NANOS, priority=1;
        "sc_truth_recorder" => &mut truth_log, period=STEP_NANOS, priority=0;
        "sc_nav_recorder" => &mut estimate_log, period=STEP_NANOS, priority=0;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut csv, STEP_NANOS, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(csv);

    Results {
        truth: truth_log,
        estimate: estimate_log,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env("scenario_small_body_nav_ukf");
    cli.finish();
    let results = run(Config { write_csv: true });
    let last = results.estimate.values.len() - 1;
    let error = results.error(last);
    println!(
        "wrote {} (final errors: |r| = {:.2} m (3 sigma {:.2} m), |v| = {:.2e} m/s, |a| = {:.2e} m/s^2 of |a_truth| = {:.2e})",
        results.csv_path.as_ref().expect("CSV written").display(),
        error.fixed_rows::<3>(0).norm(),
        3.0 * results.sigma(last, 0),
        error.fixed_rows::<3>(3).norm(),
        error.fixed_rows::<3>(6).norm(),
        results.truth.last().acceleration_mps2.norm(),
    );
}

#[cfg(test)]
mod tests {
    //! Upstream's `test_scenarioTest.py` only runs this scenario. These tests
    //! check what it demonstrates: the filter stays consistent with its
    //! covariance, tracks the position within the measurement noise, and
    //! recovers most of the non-Keplerian acceleration of Vesta's field.

    use super::*;

    /// One hour of 15 s steps: the samples after the initial transient.
    const SETTLED: usize = 240;

    fn rms(results: &Results, value: impl Fn(usize) -> f64) -> f64 {
        let count = results.estimate.values.len();
        let sum: f64 = (SETTLED..count).map(|index| value(index).powi(2)).sum();
        (sum / (count - SETTLED) as f64).sqrt()
    }

    #[test]
    fn the_filter_tracks_the_state_and_the_field() {
        let results = run(Config { write_csv: false });
        let count = results.estimate.values.len();
        assert_eq!(count, 23_041);

        // Consistent: (nearly) every error inside its 3-sigma bound.
        for component in 0..9 {
            let inside = (0..count)
                .filter(|&index| {
                    results.error(index)[component].abs() <= 3.0 * results.sigma(index, component)
                })
                .count();
            assert!(
                inside as f64 >= 0.99 * count as f64,
                "state {component}: {inside} of {count} samples inside 3 sigma"
            );
        }

        // The position within the 10 m measurement noise; the velocity, only
        // observed through the positions under a 1 m^2/s^2 process noise,
        // to a few m/s on the 186 m/s orbit.
        let position = rms(&results, |index| {
            results.error(index).fixed_rows::<3>(0).norm()
        });
        assert!(position < 15.0, "rms position error {position} m");
        for index in 0..count {
            let error = results.error(index).fixed_rows::<3>(0).norm();
            assert!(error < 40.0, "sample {index}: position error {error} m");
        }
        let velocity = rms(&results, |index| {
            results.error(index).fixed_rows::<3>(3).norm()
        });
        assert!(velocity < 3.0, "rms velocity error {velocity} m/s");

        // The estimate carries most of the inhomogeneous-field acceleration:
        // its error is well below the acceleration itself.
        let acceleration = rms(&results, |index| {
            results.error(index).fixed_rows::<3>(6).norm()
        });
        let field = rms(&results, |index| {
            results.truth.values[index].acceleration_mps2.norm()
        });
        assert!(
            field > 1.0e-3,
            "rms non-Keplerian acceleration {field} m/s^2"
        );
        assert!(
            acceleration < 0.4 * field,
            "rms acceleration error {acceleration} of {field} m/s^2"
        );
    }
}
