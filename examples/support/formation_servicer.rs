//! The reaction-wheel servicer of upstream's `scenarioFormationBasic` and
//! `scenarioRendezVous`: 750 kg, three Honeywell HR16 wheels and the
//! `mrpFeedback` / `rwMotorTorque` chain that drives them, configured as
//! those scripts configure them (both from `scenarioAttitudeFeedbackRW`).
//!
//! Include it with `common.rs`, `attitude.rs` and `rw_factory.rs`:
//!
//! ```ignore
//! #[path = "support/formation_servicer.rs"]
//! mod formation_servicer;
//! ```

#![allow(dead_code)]

use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffector,
};
use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::MrpFeedback;
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::messages::{Output, ReactionWheelStateMsg, RwArrayConfigMsg, RwSpeedMsg};
use nalgebra::Vector3;

use super::attitude::mrp_feedback;
use super::rw_factory::RwFactory;

/// Upstream servicer `mHub` \[kg\].
pub const SERVICER_MASS_KG: f64 = 750.0;
/// The servicer's three wheels.
pub const SERVICER_WHEELS: usize = 3;

/// The servicer's wheel cluster and the outputs a scenario records.
pub struct ServicerWheels {
    /// Upstream `rwStateEffector` (`"chiefRW"`), to add to the servicer.
    pub effector: ReactionWheelStateEffector,
    /// Upstream `fswRwMsg = rwFactory.getConfigMessage()`.
    pub config_msg: Output<RwArrayConfigMsg>,
    /// Upstream `rwStateEffector.rwSpeedOutMsg`.
    pub speed_msg: Output<RwSpeedMsg>,
    /// Upstream `rwStateEffector.rwOutMsgs`.
    pub state_msgs: Vec<Output<ReactionWheelStateMsg>>,
}

/// Upstream's `rwFactory.create('Honeywell_HR16', ...)` calls: 50 N m s
/// balanced wheels along `b1`, `b2` and `b3` at 100, 200 and 300 RPM, the
/// third at `[0.5, 0.5, 0.5] m`.
pub fn servicer_wheels() -> ServicerWheels {
    let model = ReactionWheelModel::BalancedWheels;
    let mut factory = RwFactory::default();
    factory.create_honeywell_hr16(Vector3::x(), 50.0, 100.0, Vector3::zeros(), model);
    factory.create_honeywell_hr16(Vector3::y(), 50.0, 200.0, Vector3::zeros(), model);
    factory.create_honeywell_hr16(
        Vector3::z(),
        50.0,
        300.0,
        Vector3::new(0.5, 0.5, 0.5),
        model,
    );
    let config_msg = Output::new(factory.config_message());
    let mut effector = ReactionWheelStateEffector::new("chiefRW");
    for wheel in factory.wheels {
        effector.add_reaction_wheel(wheel);
    }
    let speed_msg = effector.rw_speed_out_msg.clone();
    let state_msgs = effector
        .wheels()
        .iter()
        .map(|wheel| wheel.state_out.clone())
        .collect();
    ServicerWheels {
        effector,
        config_msg,
        speed_msg,
        state_msgs,
    }
}

/// The servicer's wheel control: `mrpFeedback` (`K = 3.5`, `P = 30`, no
/// integral feedback, `integralLimit = 2 / Ki * 0.1`), `rwMotorTorque`
/// controlling all three body axes, and the `arrayMotorTorqueToSingleActuators`
/// that hands each wheel its command (upstream subscribes the wheel effector
/// to the torque array directly). Schedule the splitter right after
/// `rwMotorTorque`.
pub fn wheel_control() -> (
    MrpFeedback,
    RwMotorTorque,
    ArrayMotorTorqueToSingleActuators,
) {
    let controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    let splitter =
        ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "rwMotorTorqueSplitter".to_string(),
            num_actuators: SERVICER_WHEELS,
        });
    (controller, rw_motor_torque, splitter)
}
