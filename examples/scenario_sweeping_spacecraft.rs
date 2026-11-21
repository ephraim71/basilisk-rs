//! A spacecraft sweeping its attitude about the Hill frame at commanded
//! Euler angle rates: port of upstream `examples/scenarioSweepingSpacecraft.py`.
//!
//! The 750 kg rigid spacecraft of the attitude scenarios, on the
//! 10 000 km, `e = 0.1` Earth orbit, tracks a reference built by
//! `hillPoint` and then rotated by `eulerRotation` at constant 3-2-1 Euler
//! angle rates, with `attTrackingError`, `mrpFeedback` (`K = 3.5`,
//! `P = 30`, no integral feedback) and an `extForceTorque`. The rates are
//! commanded in four 10-minute segments, `[0, 0, 0]`, `[0, 0.002, 0]`,
//! `[0, -0.002, 0]` and `[0, 0, 0]` rad/s: hold the Hill frame, sweep about
//! the second axis and back, and hold. All modules run every 0.1 s.
//!
//! Upstream `run(show_plots, useAltBodyFrame, angle_rate_command,
//! time_command)` maps to `--alt-body-frame`: on, `eulerRotation` starts
//! from `angleSet = [0, π, 0]`, pointing the body's first and third axes
//! against the Hill frame's. The command sequence is upstream's; `--all`
//! runs both frames with it, as upstream's
//! `test_scenarioSweepingSpacecraft.py` does.
//!
//! Upstream sets `attGuidanceEuler.angleRates` between its runs. Here
//! `eulerRotation` is scheduled in a [`Staged`] that applies each segment's
//! rates before its first update after the previous segment's stop time,
//! the update upstream's changed module first runs. Upstream's
//! `extForceTorque` reads its torque command in its `UpdateState`, after
//! the spacecraft integrated, for the next step; basilisk-rs's reads it
//! during the integration, so a [`Latch`] scheduled where upstream's
//! effector runs holds the command it sees.
//!
//! The commanded torque, the tracking error and the navigation attitude
//! and orbit are recorded every `samplingTime(10 min, 0.1 s, 100)` = 6 s
//! to `examples/output/scenarios/scenarioSweepingSpacecraft<alt>.csv`.
//!
//! Upstream's test only checks that both frames run. `tests` checks what
//! the figures show: after the initial capture the body tracks the
//! sweeping reference, and its axes are those of the Hill frame (or their
//! flips) whenever the commanded sweep angle is back to zero.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/latch.rs"]
mod latch;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/staged.rs"]
mod staged;

use std::f64::consts::PI;
use std::path::PathBuf;

use attitude::{
    DYNAMICS_PERIOD_NANOS, HubState, add_point_mass_earth, att_tracking_error, bit,
    external_disturbance, hill_point, hub_inertia_kg_m2, mrp_feedback, sampling_period_nanos,
    scenario_recorder, spacecraft, vehicle_config,
};
use basilisk_rs::fsw_algorithms::att_guidance::euler_rotation::{
    EulerRotation, EulerRotationConfig,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, NavigationAttitudeMsg, NavigationTranslationMsg,
    Output,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use hifitime::Epoch;
use latch::Latch;
use message_log::MessageLog;
use nalgebra::Vector3;
use orbit_tools::{min2nano, run_until};
use staged::Staged;

/// Upstream `angle_rate_command` \[rad/s\] of the `__main__` call and the
/// test, one 3-2-1 rate set per segment.
const ANGLE_RATE_COMMAND: [[f64; 3]; 4] = [
    [0.0, 0.0, 0.0],
    [0.0, 0.002, 0.0],
    [0.0, -0.002, 0.0],
    [0.0, 0.0, 0.0],
];
/// Upstream `time_command` \[min\]: each segment's duration.
const TIME_COMMAND: [f64; 4] = [10.0, 10.0, 10.0, 10.0];

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
}

/// The parametrizations of upstream `test_scenarioSweepingSpacecraft.py`.
const TEST_PARAMETRIZATIONS: [Config; 2] = [Config::new(false), Config::new(true)];

/// The recorded histories, every 6 s.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    torque: MessageLog<BodyTorqueCommandMsg>,
    guidance: MessageLog<AttitudeGuidanceMsg>,
    attitude: MessageLog<NavigationAttitudeMsg>,
    translation: MessageLog<NavigationTranslationMsg>,
    /// End of each segment \[ns\].
    stop_times_nanos: Vec<u64>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let inertia = hub_inertia_kg_m2();
    let mut spacecraft = spacecraft(inertia, &HubState::on_orbit(0.1));
    add_point_mass_earth(&mut spacecraft);
    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut torque_latch = Latch::<BodyTorqueCommandMsg>::default();
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut hill_reference = hill_point();

    let mut stop_times_nanos = Vec::new();
    let mut stop_nanos = 0;
    for minutes in TIME_COMMAND {
        stop_nanos += min2nano(minutes);
        stop_times_nanos.push(stop_nanos);
    }
    let mut euler_rotation = Staged::new(EulerRotation::new(EulerRotationConfig {
        name: "eulerRotation".to_string(),
        angle_set_rad: if config.use_alt_body_frame {
            Vector3::new(0.0, PI, 0.0)
        } else {
            Vector3::zeros()
        },
        angle_rates_radps: Vector3::from(ANGLE_RATE_COMMAND[0]),
    }));
    for (stop, rates) in stop_times_nanos.iter().zip(&ANGLE_RATE_COMMAND[1..]) {
        let rates = Vector3::from(*rates);
        euler_rotation = euler_rotation.after(*stop, move |module: &mut EulerRotation, _| {
            module.angle_rates_radps = rates;
        });
    }
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let mut controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let sample_period =
        sampling_period_nanos(min2nano(TIME_COMMAND[0]), DYNAMICS_PERIOD_NANOS, 100);
    let mut torque_log = MessageLog::<BodyTorqueCommandMsg>::default();
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut attitude_log = MessageLog::<NavigationAttitudeMsg>::default();
    let mut translation_log = MessageLog::<NavigationTranslationMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(
        &format!(
            "scenarioSweepingSpacecraft{}",
            bit(config.use_alt_body_frame)
        ),
        config.write_csv,
    );

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &controller.cmd_torque_out_msg => &mut torque_latch.input,
        &torque_latch.output => &mut external_torque.cmd_torque_in_msg,
    );
    spacecraft.add_dynamic_effector(external_torque);
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.translation_out_msg => &mut hill_reference.trans_nav_in_msg,
        &hill_reference.att_ref_out_msg => &mut euler_rotation.module.att_ref_in_msg,
        &euler_rotation.module.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &controller.cmd_torque_out_msg => &mut torque_log.input,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &navigation.attitude_out_msg => &mut attitude_log.input,
        &navigation.translation_out_msg => &mut translation_log.input,
        &controller.cmd_torque_out_msg => recorder.add_source::<BodyTorqueCommandMsg>(
            CsvSourceConfig::columns(vector_columns("torque_request_body_nm", "torqueRequestBody_Nm")),
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
    // Upstream task order: scObject, extFT (its command read), sNav,
    // hillPoint, eulerRotation, attError, mrpControl.
    schedule! { simulation,
        "bsk-Sat" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "externalDisturbance" => &mut torque_latch, period=DYNAMICS_PERIOD_NANOS, priority=95;
        "SimpleNavigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        "hillPoint" => &mut hill_reference, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "eulerRotation" => &mut euler_rotation, period=DYNAMICS_PERIOD_NANOS, priority=75;
        "attErrorrotatingHill" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "mrpFeedback" => &mut controller, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "torque_log" => &mut torque_log, period=sample_period, priority=0;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "attitude_log" => &mut attitude_log, period=sample_period, priority=0;
        "translation_log" => &mut translation_log, period=sample_period, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    for stop in &stop_times_nanos {
        run_until(&mut simulation, *stop);
    }
    drop(simulation);
    drop(recorder);

    Results {
        torque: torque_log,
        guidance: guidance_log,
        attitude: attitude_log,
        translation: translation_log,
        stop_times_nanos,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_sweeping_spacecraft [--[no-]alt-body-frame] [--all]");
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream `__main__`: `useAltBodyFrame = True`.
        vec![Config::new(cli.flag("alt-body-frame", true))]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!(
            "wrote {} (final |sigma_BR| = {:.3e})",
            results.csv_path.expect("CSV written").display(),
            results.guidance.last().sigma_br.norm(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    /// Upstream's orientation figure: `[î_r · b̂1, î_θ · b̂2, î_h · b̂3]`.
    fn orientation(results: &Results, index: usize) -> Vector3<f64> {
        let translation = &results.translation.values[index];
        let r = translation.position_bn_n_m;
        let h = r.cross(&translation.velocity_bn_n_mps);
        let i_r = r.normalize();
        let i_h = h.normalize();
        let i_theta = i_h.cross(&i_r);
        let dcm_bn = mrp_to_dcm(results.attitude.values[index].sigma_bn);
        Vector3::new(
            i_r.dot(&dcm_bn.row(0).transpose()),
            i_theta.dot(&dcm_bn.row(1).transpose()),
            i_h.dot(&dcm_bn.row(2).transpose()),
        )
    }

    /// The samples cover the four segments, every 6 s.
    #[test]
    fn the_four_segments_are_recorded() {
        let results = quiet(TEST_PARAMETRIZATIONS[0]);
        assert_eq!(
            results.stop_times_nanos,
            vec![
                min2nano(10.0),
                min2nano(20.0),
                min2nano(30.0),
                min2nano(40.0)
            ]
        );
        assert_eq!(results.guidance.values.len(), 401);
        assert_eq!(
            *results.guidance.times_nanos.last().unwrap(),
            min2nano(40.0)
        );
    }

    /// Captured within the first segment, the body then tracks the sweeping
    /// reference closely, and settles on it before each segment ends; at the end of the hold and of the sweep back the
    /// body axes are the Hill frame's, flipped about the second axis in the
    /// alternate body frame. Halfway, after 10 minutes at 0.002 rad/s, the
    /// body has turned 1.2 rad about the Hill frame's along-track axis.
    #[test]
    fn the_body_sweeps_about_the_hill_frame() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            for (time, guidance) in results
                .guidance
                .times_nanos
                .iter()
                .zip(&results.guidance.values)
            {
                // A transient of about 3 times the rate step (in rad/s)
                // follows each rate step.
                if *time >= min2nano(8.0) {
                    assert!(
                        guidance.sigma_br.norm() < 2.0e-2,
                        "{config:?} at {time}: {guidance:?}"
                    );
                }
                if results.stop_times_nanos.contains(time) {
                    assert!(
                        guidance.sigma_br.norm() < 1.0e-3,
                        "{config:?} at {time}: {guidance:?}"
                    );
                }
            }
            let sign = if config.use_alt_body_frame { -1.0 } else { 1.0 };
            let aligned = Vector3::new(sign, 1.0, sign);
            for minutes in [10.0, 30.0, 40.0] {
                let index = results
                    .guidance
                    .times_nanos
                    .iter()
                    .position(|t| *t == min2nano(minutes))
                    .unwrap();
                let alignment = orientation(&results, index);
                assert!(
                    (alignment - aligned).norm() < 1.0e-2,
                    "{config:?} at {minutes} min: {alignment}"
                );
            }
            // The initial capture takes far more torque than the sweeps.
            let peak = |capture: bool| {
                results
                    .torque
                    .times_nanos
                    .iter()
                    .zip(&results.torque.values)
                    .filter(|(time, _)| (**time < min2nano(8.0)) == capture)
                    .fold(0.0_f64, |max, (_, torque)| {
                        max.max(torque.torque_request_body_nm.norm())
                    })
            };
            assert!(
                peak(false) < 0.5 * peak(true),
                "{config:?}: {} vs {}",
                peak(false),
                peak(true)
            );
            let index = results
                .guidance
                .times_nanos
                .iter()
                .position(|t| *t == min2nano(20.0))
                .unwrap();
            let alignment = orientation(&results, index);
            let swept = (0.002_f64 * 600.0).cos();
            assert!(
                (alignment - Vector3::new(sign * swept, 1.0, sign * swept)).norm() < 2.0e-2,
                "{config:?}: {alignment}"
            );
        }
    }
}
