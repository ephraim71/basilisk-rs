//! The deep-space inertial-pointing simulation shared by upstream's
//! `scenarioAttitudePointing`, `scenarioAttitudePointingPy` and
//! `scenarioAttitudePointingNumba`, which differ only in the attitude
//! controller.
//!
//! A 750 kg rigid hub with no orbit (no gravity body, at rest at the origin)
//! starts at `sigma_BN = [0.1, 0.2, -0.3]`; `simpleNav`, `inertial3D` and
//! `attTrackingError` feed the controller, whose torque command an
//! `extForceTorque` effector applies. One 0.1 s task, 10 minutes, histories
//! sampled every `samplingTime(T, 0.1 s, 50)` = 12.2 s. The controller runs
//! last, as the scripts add it to the task last.

#![allow(dead_code)]

use std::path::PathBuf;

use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, Input, Output, VehicleConfigMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{Module, connect, schedule};
use hifitime::Epoch;
use nalgebra::Vector3;

use super::attitude::{
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, HubState, att_tracking_error, external_disturbance,
    hub_inertia_kg_m2, inertial_3d, sampling_period_nanos, scenario_recorder, spacecraft,
    vehicle_config,
};
use super::common::vector_columns;
use super::message_log::MessageLog;

/// The controller's ports. Ports are shared handles, so these may be clones
/// of the controller's own fields: connecting a clone connects the field.
pub struct ControllerPorts {
    pub guid_in_msg: Input<AttitudeGuidanceMsg>,
    pub cmd_torque_out_msg: Output<BodyTorqueCommandMsg>,
    /// The vehicle configuration input, for controllers that read one.
    pub veh_config_in_msg: Option<Input<VehicleConfigMsg>>,
}

/// The recorded histories.
pub struct PointingResults {
    pub guidance: MessageLog<AttitudeGuidanceMsg>,
    pub torque: MessageLog<BodyTorqueCommandMsg>,
    pub csv_path: Option<PathBuf>,
}

/// Runs the pointing simulation with `controller`, starting at the rate
/// `initial_omega_bn_b_radps`, and writes
/// `examples/output/scenarios/<csv_stem>.csv` when `write_csv`.
pub fn run_pointing(
    controller: &mut dyn Module,
    ports: ControllerPorts,
    initial_omega_bn_b_radps: Vector3<f64>,
    csv_stem: &str,
    write_csv: bool,
) -> PointingResults {
    let ControllerPorts {
        mut guid_in_msg,
        cmd_torque_out_msg,
        veh_config_in_msg,
    } = ports;
    let inertia = hub_inertia_kg_m2();
    let mut initial = HubState::in_deep_space();
    initial.omega_bn_b_radps = initial_omega_bn_b_radps;
    let mut spacecraft = spacecraft(inertia, &initial);
    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut inertial_reference = inertial_3d();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 50);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut torque_log = MessageLog::<BodyTorqueCommandMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(csv_stem, write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation, &cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg);
    spacecraft.add_dynamic_effector(external_torque);
    if let Some(mut veh_config_in_msg) = veh_config_in_msg {
        connect!(&simulation, &vehicle_config_msg => &mut veh_config_in_msg);
    }
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut guid_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &cmd_torque_out_msg => &mut torque_log.input,
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
        &cmd_torque_out_msg => recorder.add_source::<BodyTorqueCommandMsg>(
            CsvSourceConfig::columns(vector_columns(
                "torque_request_body_nm",
                "torqueRequestBody_Nm",
            )),
        ),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        "inertial_reference" => &mut inertial_reference, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "tracking_error" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "controller" => controller, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "torque_log" => &mut torque_log, period=sample_period, priority=0;
    }
    if write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    PointingResults {
        guidance: guidance_log,
        torque: torque_log,
        csv_path,
    }
}
