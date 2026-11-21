//! Velocity-frame pointing on a hyperbolic flyby: port of upstream
//! `examples/scenarioAttGuideHyperbolic.py`.
//!
//! A 750 kg rigid spacecraft on an Earth hyperbola (`a = -150 000 km`,
//! `e = 1.5`, `i = 33.3°`, `Ω = 48.2°`, `ω = 347.8°`, starting at
//! `f = 30°`) starts at `sigma_BN = [0.1, 0.2, -0.3]`,
//! `omega_BN = [0.001, -0.01, 0.03] rad/s`. `velocityPoint` makes the
//! velocity frame (`r̂2` along the velocity, `r̂3` along the orbit normal) the
//! reference, `attTrackingError` forms the error and `mrpFeedback`
//! (`K = 3.5`, `P = 30`, no integral feedback) commands the torque an
//! `extForceTorque` effector applies. One 1 s task, 750 s, histories sampled
//! every `samplingTime(750 s, 1 s, 100)` = 7 s.
//!
//! Upstream `run(show_plots, useAltBodyFrame)` maps to `--alt-body-frame`:
//! align a corrected body frame, rotated 180° about `b3`
//! (`attError.sigma_R0R = [0, 0, -1]`), with the velocity frame instead of
//! the body frame itself. The upstream script's own default is off. `--all`
//! runs both cases, as upstream's `test_scenarioAttGuideHyperbolic.py` does.
//!
//! Writes `examples/output/scenarios/scenarioAttGuideHyperbolic<alt>.csv`.

#[path = "support/att_scenario.rs"]
mod att_scenario;
#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;

use std::path::PathBuf;

use att_scenario::{
    attitude_columns, guidance_columns, rigid_spacecraft, torque_columns, translation_columns,
};
use attitude::{
    HubState, INITIAL_OMEGA_BN_RADPS, INITIAL_SIGMA_BN, add_point_mass_earth, att_tracking_error,
    bit, external_disturbance, hub_inertia_kg_m2, mrp_feedback, sampling_period_nanos,
    scenario_recorder, vehicle_config,
};
use basilisk_rs::fsw_algorithms::att_guidance::velocity_point::{
    VelocityPoint, VelocityPointConfig,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, NavigationAttitudeMsg, NavigationTranslationMsg,
    Output,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{MU_EARTH_M3PS2, seconds};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;

/// Upstream `simulationTimeStep = macros.sec2nano(1.)`.
const STEP_NANOS: u64 = seconds(1);
/// Upstream `simulationTime = macros.sec2nano(750.)`.
const DURATION_NANOS: u64 = seconds(750);

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    use_alt_body_frame: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(use_alt_body_frame: bool) -> Self {
        Self {
            use_alt_body_frame,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!("scenarioAttGuideHyperbolic{}", bit(self.use_alt_body_frame))
    }
}

/// The parametrizations of upstream `test_scenarioAttGuideHyperbolic.py`.
const TEST_PARAMETRIZATIONS: [Config; 2] = [Config::new(false), Config::new(true)];

/// Upstream's hyperbola, `oe` of the script.
fn hyperbola() -> ClassicElements {
    ClassicElements {
        semi_major_axis: -150_000.0 * 1000.0,
        eccentricity: 1.5,
        inclination_rad: 33.3_f64.to_radians(),
        raan_rad: 48.2_f64.to_radians(),
        arg_periapsis_rad: 347.8_f64.to_radians(),
        true_anomaly_rad: 30.0_f64.to_radians(),
        ..Default::default()
    }
}

/// The recorded histories.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    torque: MessageLog<BodyTorqueCommandMsg>,
    attitude: MessageLog<NavigationAttitudeMsg>,
    translation: MessageLog<NavigationTranslationMsg>,
    csv_path: Option<PathBuf>,
}

impl Results {
    /// The projections `v̂ · b2` and `ĥ · b3` of the body axes on the
    /// velocity-frame axes at sample `index`.
    fn velocity_frame_projections(&self, index: usize) -> (f64, f64) {
        let translation = &self.translation.values[index];
        let v_hat = translation.velocity_bn_n_mps.normalize();
        let h_hat = translation
            .position_bn_n_m
            .cross(&translation.velocity_bn_n_mps)
            .normalize();
        let dcm_bn = mrp_to_dcm(self.attitude.values[index].sigma_bn);
        (
            v_hat.dot(&dcm_bn.row(1).transpose()),
            h_hat.dot(&dcm_bn.row(2).transpose()),
        )
    }
}

fn run(config: Config) -> Results {
    let inertia = hub_inertia_kg_m2();
    let (position_m, velocity_mps) = elements_to_state(MU_EARTH_M3PS2, &hyperbola());
    let initial = HubState {
        position_m,
        velocity_mps,
        sigma_bn: Vector3::from(INITIAL_SIGMA_BN),
        omega_bn_b_radps: Vector3::from(INITIAL_OMEGA_BN_RADPS),
    };
    let mut spacecraft = rigid_spacecraft(inertia, &initial, STEP_NANOS);
    add_point_mass_earth(&mut spacecraft);
    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut guidance = VelocityPoint::new(VelocityPointConfig {
        name: "velocityPoint".to_string(),
        mu_m3ps2: MU_EARTH_M3PS2,
    });
    let sigma_r0r = if config.use_alt_body_frame {
        Vector3::new(0.0, 0.0, -1.0)
    } else {
        Vector3::zeros()
    };
    let mut tracking_error = att_tracking_error(sigma_r0r);
    let mut controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let sample_period = sampling_period_nanos(DURATION_NANOS, STEP_NANOS, 100);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut torque_log = MessageLog::<BodyTorqueCommandMsg>::default();
    let mut attitude_log = MessageLog::<NavigationAttitudeMsg>::default();
    let mut translation_log = MessageLog::<NavigationTranslationMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg,
    );
    spacecraft.add_dynamic_effector(external_torque);
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.translation_out_msg => &mut guidance.trans_nav_in_msg,
        &guidance.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &controller.cmd_torque_out_msg => &mut torque_log.input,
        &navigation.attitude_out_msg => &mut attitude_log.input,
        &navigation.translation_out_msg => &mut translation_log.input,
        &controller.cmd_torque_out_msg =>
            recorder.add_source::<BodyTorqueCommandMsg>(torque_columns()),
        &tracking_error.att_guid_out_msg =>
            recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns()),
        &navigation.attitude_out_msg =>
            recorder.add_source::<NavigationAttitudeMsg>(attitude_columns()),
        &navigation.translation_out_msg =>
            recorder.add_source::<NavigationTranslationMsg>(translation_columns()),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=STEP_NANOS, priority=100;
        "navigation" => &mut navigation, period=STEP_NANOS, priority=90;
        "velocity_point" => &mut guidance, period=STEP_NANOS, priority=80;
        "tracking_error" => &mut tracking_error, period=STEP_NANOS, priority=70;
        "mrp_feedback" => &mut controller, period=STEP_NANOS, priority=60;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "torque_log" => &mut torque_log, period=sample_period, priority=0;
        "attitude_log" => &mut attitude_log, period=sample_period, priority=0;
        "translation_log" => &mut translation_log, period=sample_period, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        torque: torque_log,
        attitude: attitude_log,
        translation: translation_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_att_guide_hyperbolic [--[no-]alt-body-frame] [--all]");
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream's `__main__` runs `run(True, False)`.
        vec![Config::new(cli.flag("alt-body-frame", false))]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        let (along_velocity, along_normal) =
            results.velocity_frame_projections(results.translation.values.len() - 1);
        println!(
            "wrote {} (final |sigma_BR| = {:.3e}, v·b2 = {along_velocity:.6}, h·b3 = {along_normal:.6})",
            results.csv_path.as_ref().expect("CSV written").display(),
            results.guidance.last().sigma_br.norm(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttGuideHyperbolic.py` only checks that both
    //! cases run. These tests check the pointing the scenario demonstrates.

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    /// The hyperbolic flyby the script starts on.
    #[test]
    fn the_initial_state_is_on_the_hyperbola() {
        let (position_m, velocity_mps) = elements_to_state(MU_EARTH_M3PS2, &hyperbola());
        let elements = basilisk_rs::utilities::orbital_motion::state_to_elements(
            MU_EARTH_M3PS2,
            position_m,
            velocity_mps,
        );
        assert!((elements.eccentricity - 1.5).abs() < 1.0e-12);
        assert!((elements.semi_major_axis / -150.0e6 - 1.0).abs() < 1.0e-12);
    }

    /// The tracking error converges and `b2` ends along the velocity, `b3`
    /// along the orbit normal.
    #[test]
    fn the_body_frame_aligns_with_the_velocity_frame() {
        let results = quiet(TEST_PARAMETRIZATIONS[0]);
        assert_eq!(results.guidance.values.len(), 108);
        let last = results.guidance.last();
        assert!(last.sigma_br.norm() < 1.0e-6, "{}", last.sigma_br.norm());
        assert!(last.omega_br_b_radps.norm() < 1.0e-4);
        // A large initial command; once aligned, the slowly turning
        // velocity frame needs almost no torque.
        assert!(results.torque.values[0].torque_request_body_nm.norm() > 0.5);
        assert!(results.torque.last().torque_request_body_nm.norm() < 1.0e-4);
        let (along_velocity, along_normal) = results.velocity_frame_projections(107);
        assert!(along_velocity > 1.0 - 1.0e-6, "{along_velocity}");
        assert!(along_normal > 1.0 - 1.0e-6, "{along_normal}");
    }

    /// With `sigma_R0R = [0, 0, -1]` the corrected frame aligns instead:
    /// `b2` ends against the velocity, `b3` still along the orbit normal.
    #[test]
    fn the_corrected_body_frame_flips_the_velocity_axis() {
        let results = quiet(TEST_PARAMETRIZATIONS[1]);
        let last = results.guidance.last();
        assert!(last.sigma_br.norm() < 1.0e-6, "{}", last.sigma_br.norm());
        let (along_velocity, along_normal) = results.velocity_frame_projections(107);
        assert!(along_velocity < -1.0 + 1.0e-6, "{along_velocity}");
        assert!(along_normal > 1.0 - 1.0e-6, "{along_normal}");
    }
}
