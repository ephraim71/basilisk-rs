//! Hill-frame pointing on an elliptic orbit: port of upstream
//! `examples/scenarioAttitudeGuidance.py`.
//!
//! A 750 kg rigid spacecraft on a 10 000 km, `e = 0.1` Earth orbit starts at
//! `sigma_BN = [0.1, 0.2, -0.3]`, `omega_BN = [0.001, -0.01, 0.03] rad/s`.
//! `hillPoint` (with an all-zero Earth ephemeris message) makes the orbit's
//! Hill frame the reference, `attTrackingError` forms the error and
//! `mrpFeedback` (`K = 3.5`, `P = 30`, no integral feedback) commands the
//! torque that an `extForceTorque` effector applies. One 0.1 s task,
//! 10 minutes, histories sampled every 6 s.
//!
//! Upstream `run(show_plots, useAltBodyFrame)` maps to `--alt-body-frame`:
//! align a corrected body frame, rotated 180° about `b2`
//! (`attError.sigma_R0R = [0, 1, 0]`), with the Hill frame instead of the body
//! frame itself. The upstream script's own default is on. `--all` runs both
//! cases, as upstream's `test_scenarioAttitudeGuidance.py` does.
//!
//! Writes `examples/output/scenarios/scenarioAttitudeGuidance<alt>.csv`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;

use std::path::PathBuf;

use attitude::{
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, HubState, add_point_mass_earth, att_tracking_error, bit,
    external_disturbance, hill_point, hub_inertia_kg_m2, mrp_feedback, sampling_period_nanos,
    scenario_recorder, spacecraft, vehicle_config,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, EphemerisMsg, NavigationAttitudeMsg,
    NavigationTranslationMsg, Output,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;

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
        format!("scenarioAttitudeGuidance{}", bit(self.use_alt_body_frame))
    }
}

/// The parametrizations of upstream `test_scenarioAttitudeGuidance.py`.
const TEST_PARAMETRIZATIONS: [Config; 2] = [Config::new(false), Config::new(true)];

/// The recorded histories.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    attitude: MessageLog<NavigationAttitudeMsg>,
    translation: MessageLog<NavigationTranslationMsg>,
    csv_path: Option<PathBuf>,
}

impl Results {
    /// Upstream `plot_orientation`'s data: `i_r · b1`, `i_theta · b2` and
    /// `i_h · b3`, the projections of the body axes on the Hill axes.
    fn orientation(&self, index: usize) -> Vector3<f64> {
        let translation = &self.translation.values[index];
        let position = translation.position_bn_n_m;
        let i_r = position.normalize();
        let i_h = position.cross(&translation.velocity_bn_n_mps).normalize();
        let i_theta = i_h.cross(&i_r);
        let dcm_bn = mrp_to_dcm(self.attitude.values[index].sigma_bn);
        Vector3::new(
            i_r.dot(&dcm_bn.row(0).transpose()),
            i_theta.dot(&dcm_bn.row(1).transpose()),
            i_h.dot(&dcm_bn.row(2).transpose()),
        )
    }
}

fn run(config: Config) -> Results {
    let inertia = hub_inertia_kg_m2();
    let mut spacecraft = spacecraft(inertia, &HubState::on_orbit(0.1));
    add_point_mass_earth(&mut spacecraft);
    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut guidance = hill_point();
    // Upstream subscribes `celBodyInMsg` to a default (all-zero) payload.
    let earth_ephemeris_msg = Output::new(EphemerisMsg::default());
    let sigma_r0r = if config.use_alt_body_frame {
        Vector3::new(0.0, 1.0, 0.0)
    } else {
        Vector3::zeros()
    };
    let mut tracking_error = att_tracking_error(sigma_r0r);
    let mut controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 100);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
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
        &earth_ephemeris_msg => &mut guidance.cel_body_in_msg,
        &guidance.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &navigation.attitude_out_msg => &mut attitude_log.input,
        &navigation.translation_out_msg => &mut translation_log.input,
        &controller.cmd_torque_out_msg => recorder.add_source::<BodyTorqueCommandMsg>(
            CsvSourceConfig::columns(vector_columns(
                "torque_request_body_nm",
                "torqueRequestBody_Nm",
            )),
        ),
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
        &navigation.attitude_out_msg => recorder.add_source::<NavigationAttitudeMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_bn", "sigma_BN")),
        ),
        &navigation.translation_out_msg => recorder.add_source::<NavigationTranslationMsg>(
            CsvSourceConfig::columns(
                vector_columns("position_bn_n_m", "r_BN_N_m")
                    .into_iter()
                    .chain(vector_columns("velocity_bn_n_mps", "v_BN_N_m_per_s")),
            ),
        ),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        "hill_point" => &mut guidance, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "tracking_error" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "mrp_feedback" => &mut controller, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
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
        attitude: attitude_log,
        translation: translation_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_attitude_guidance [--[no-]alt-body-frame] [--all]");
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream's `__main__` runs `run(True, True)`.
        vec![Config::new(cli.flag("alt-body-frame", true))]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        let orientation = results.orientation(results.translation.values.len() - 1);
        println!(
            "wrote {} (final |sigma_BR| = {:.3e}, Hill-axis projections [{:.6}, {:.6}, {:.6}])",
            results.csv_path.as_ref().expect("CSV written").display(),
            results.guidance.last().sigma_br.norm(),
            orientation.x,
            orientation.y,
            orientation.z,
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttitudeGuidance.py` only checks that both
    //! cases run. These tests check the alignment the scenario documents.

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    /// "the projections of the body frame axes onto the Hill frame axes all
    /// converge to +1, indicating that B becomes asymptotically aligned with
    /// R as desired."
    #[test]
    fn the_body_frame_aligns_with_the_hill_frame() {
        let results = quiet(TEST_PARAMETRIZATIONS[0]);
        assert_eq!(results.guidance.values.len(), 101);
        assert!(results.guidance.last().sigma_br.norm() < 1.0e-4);
        let orientation = results.orientation(100);
        assert!(
            (orientation - Vector3::new(1.0, 1.0, 1.0)).norm() < 1.0e-6,
            "{orientation}"
        );
    }

    /// "the projections of the 2nd body frame axis onto the 2nd Hill frame
    /// axes converges to +1, while the other projections converge to -1."
    #[test]
    fn the_corrected_body_frame_flips_the_first_and_third_axes() {
        let results = quiet(TEST_PARAMETRIZATIONS[1]);
        assert!(results.guidance.last().sigma_br.norm() < 1.0e-4);
        let orientation = results.orientation(100);
        assert!(
            (orientation - Vector3::new(-1.0, 1.0, -1.0)).norm() < 1.0e-6,
            "{orientation}"
        );
    }
}
