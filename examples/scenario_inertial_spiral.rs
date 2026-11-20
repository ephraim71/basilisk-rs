//! Layered attitude references, a spiral scan: port of upstream
//! `examples/scenarioInertialSpiral.py`.
//!
//! A 750 kg rigid spacecraft on the 10 000 km, `e = 0.01` Earth orbit of the
//! attitude scenarios starts at rest at `sigma_BN = 0`. `inertial3D`
//! (`sigma_R0N = 0`) feeds two chained `eulerRotation` modules: the first
//! turns the reference at 0.02 rad/s about its 1-axis (3-2-1 rates
//! `[0, 0, 0.02]`), the second at 0.0001 rad/s about the 2-axis
//! (`[0, 0.0001, 0]`), an outward sweeping spiral. `attTrackingError`
//! (named `attError`) forms the error and `mrpFeedback` (`K = 3.5`,
//! `P = 30`, no integral feedback) commands the torque an `extForceTorque`
//! effector applies. One 0.1 s task, 10 minutes, histories sampled every
//! `samplingTime(10 min, 0.1 s, 100)` = 6 s.
//!
//! Upstream's `run(show_plots)` has no scenario parameters. Writes
//! `examples/output/scenarios/scenarioInertialSpiral.csv`.

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

use att_scenario::{attitude_columns, guidance_columns, torque_columns};
use attitude::{
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, HubState, ScenarioOrbit, add_point_mass_earth,
    external_disturbance, hub_inertia_kg_m2, mrp_feedback, sampling_period_nanos,
    scenario_recorder, spacecraft, vehicle_config,
};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::euler_rotation::{
    EulerRotation, EulerRotationConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, NavigationAttitudeMsg, Output,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;

/// CSV file stem, upstream's figure-name prefix.
const FILE_STEM: &str = "scenarioInertialSpiral";
/// Upstream `attGuidanceEuler1.angleRates` \[rad/s\] (3-2-1 order).
const FIRST_ROTATION_RATES_RADPS: [f64; 3] = [0.0, 0.0, 0.02];
/// Upstream `attGuidanceEuler2.angleRates` \[rad/s\].
const SECOND_ROTATION_RATES_RADPS: [f64; 3] = [0.0, 0.0001, 0.0];

/// The recorded histories.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    torque: MessageLog<BodyTorqueCommandMsg>,
    attitude: MessageLog<NavigationAttitudeMsg>,
    csv_path: Option<PathBuf>,
}

/// Upstream `eulerRotation` named `name`, at constant 3-2-1 `rates`.
fn euler_rotation(name: &str, rates: [f64; 3]) -> EulerRotation {
    EulerRotation::new(EulerRotationConfig {
        name: name.to_string(),
        angle_set_rad: Vector3::zeros(),
        angle_rates_radps: Vector3::from(rates),
    })
}

fn run(write_csv: bool) -> Results {
    let inertia = hub_inertia_kg_m2();
    let (position_m, velocity_mps) = ScenarioOrbit::new(0.01).state();
    let initial = HubState {
        position_m,
        velocity_mps,
        sigma_bn: Vector3::zeros(),
        omega_bn_b_radps: Vector3::zeros(),
    };
    let mut spacecraft = spacecraft(inertia, &initial);
    add_point_mass_earth(&mut spacecraft);
    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut inertial_reference = Inertial3D::new(Inertial3DConfig {
        name: "inertial3D".to_string(),
        sigma_r0n: Vector3::zeros(),
    });
    let mut first_rotation = euler_rotation("eulerRotation1", FIRST_ROTATION_RATES_RADPS);
    let mut second_rotation = euler_rotation("eulerRotation2", SECOND_ROTATION_RATES_RADPS);
    let mut tracking_error = AttTrackingError::new(AttTrackingErrorConfig {
        name: "attError".to_string(),
        sigma_r0r: Vector3::zeros(),
    });
    let mut controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 100);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut torque_log = MessageLog::<BodyTorqueCommandMsg>::default();
    let mut attitude_log = MessageLog::<NavigationAttitudeMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(FILE_STEM, write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg,
    );
    spacecraft.add_dynamic_effector(external_torque);
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &inertial_reference.att_ref_out_msg => &mut first_rotation.att_ref_in_msg,
        &first_rotation.att_ref_out_msg => &mut second_rotation.att_ref_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &second_rotation.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &controller.cmd_torque_out_msg => &mut torque_log.input,
        &navigation.attitude_out_msg => &mut attitude_log.input,
        &tracking_error.att_guid_out_msg =>
            recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns()),
        &controller.cmd_torque_out_msg =>
            recorder.add_source::<BodyTorqueCommandMsg>(torque_columns()),
        &navigation.attitude_out_msg =>
            recorder.add_source::<NavigationAttitudeMsg>(attitude_columns()),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        "inertial3D" => &mut inertial_reference, period=DYNAMICS_PERIOD_NANOS, priority=85;
        "eulerRotation1" => &mut first_rotation, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "eulerRotation2" => &mut second_rotation, period=DYNAMICS_PERIOD_NANOS, priority=75;
        "attError" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "mrpFeedback" => &mut controller, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "torque_log" => &mut torque_log, period=sample_period, priority=0;
        "attitude_log" => &mut attitude_log, period=sample_period, priority=0;
    }
    if write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        torque: torque_log,
        attitude: attitude_log,
        csv_path,
    }
}

fn main() {
    Cli::from_env("scenario_inertial_spiral").finish();
    let results = run(true);
    println!(
        "wrote {} (final |sigma_BR| = {:.3e})",
        results.csv_path.as_ref().expect("CSV written").display(),
        results.guidance.last().sigma_br.norm(),
    );
}

#[cfg(test)]
mod tests {
    //! Upstream runs this scenario in `test_scenarioTest.py`, which only
    //! checks that it completes. These tests check the spiral it draws.

    use basilisk_rs::utilities::rigid_body_kinematics::{
        dcm_to_mrp, euler321_to_dcm, mrp_to_dcm, mrp_to_euler321, sub_mrp,
    };

    use super::*;

    /// The layered reference at `t` \[s\]: the second rotation on top of the
    /// first, both started at zero, on the inertial frame.
    fn spiral_reference(t: f64) -> Vector3<f64> {
        let first = euler321_to_dcm(Vector3::from(FIRST_ROTATION_RATES_RADPS) * t);
        let second = euler321_to_dcm(Vector3::from(SECOND_ROTATION_RATES_RADPS) * t);
        dcm_to_mrp(second * first)
    }

    /// After the start-up transient the body tracks the spiral reference:
    /// the tracking error is small and the body attitude is the closed-form
    /// layered rotation.
    #[test]
    fn the_body_follows_the_spiral_reference() {
        let results = run(false);
        assert_eq!(results.guidance.values.len(), 101);
        // The reference starts turning at 0.02 rad/s with the body at rest,
        // so the first command is about `P * 0.02 = 0.6 N m` about b1.
        assert!(results.guidance.values[0].omega_br_b_radps.norm() > 0.019);
        let first_torque = results.torque.values[0].torque_request_body_nm;
        assert!((first_torque.x - 0.6).abs() < 1.0e-3, "{first_torque}");
        // Once tracking, only the small gyroscopic torque of the spiral.
        assert!(results.torque.last().torque_request_body_nm.norm() < 1.0e-2);
        // The underdamped transient settles within six minutes.
        for (time, guidance) in results
            .guidance
            .times_s()
            .iter()
            .zip(&results.guidance.values)
            .skip(60)
        {
            assert!(
                guidance.sigma_br.norm() < 5.0e-4,
                "t = {time}: {}",
                guidance.sigma_br.norm()
            );
        }
        let last = results.guidance.last();
        assert!(last.sigma_br.norm() < 1.0e-4, "{}", last.sigma_br.norm());
        assert!(last.omega_br_b_radps.norm() < 1.0e-5);

        // `eulerRotation` takes a zero prior time as "no prior call", so
        // (as upstream) its updates at 0 and 0.1 s both integrate a zero
        // step: the angles run one 0.1 s task step behind `rates * t`.
        let t_end = results.attitude.times_s()[100];
        let sigma_bn = results.attitude.last().sigma_bn;
        let reference = spiral_reference(t_end - DYNAMICS_PERIOD_NANOS as f64 * 1.0e-9);
        let error = sub_mrp(sigma_bn, reference);
        assert!(error.norm() < 1.0e-5, "{}", error.norm());
    }

    /// The spiral: the yaw-axis sweep grows linearly while the body spins
    /// about its 1-axis, so the Euler 3-2-1 angles of the body trace an
    /// outward spiral (upstream figure 5).
    #[test]
    fn the_euler_angles_trace_an_outward_spiral() {
        let results = run(false);
        let radius: Vec<f64> = results
            .attitude
            .values
            .iter()
            .map(|attitude| {
                let angles = mrp_to_euler321(attitude.sigma_bn);
                (angles[0] * angles[0] + angles[1] * angles[1]).sqrt()
            })
            .collect();
        // The 0.0001 rad/s tilt reaches 0.06 rad after 600 s.
        assert!((radius[100] - 0.06).abs() < 5.0e-3, "{}", radius[100]);
        assert!(radius[100] > radius[50] && radius[50] > radius[20]);
        // The spin about b1 stays aligned with the reference spin axis.
        let dcm_bn = mrp_to_dcm(results.attitude.last().sigma_bn);
        let spin_axis_n = mrp_to_dcm(spiral_reference(599.9)).transpose() * Vector3::x();
        assert!(dcm_bn.transpose().column(0).dot(&spin_axis_n) > 1.0 - 1.0e-9);
    }
}
