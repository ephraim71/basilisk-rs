//! Attitude control laws: turn a guidance error into a commanded torque or rate.

pub mod low_pass_filter_torque_command;
pub mod mrp_feedback;
pub mod mrp_pd;
pub mod mrp_steering;
pub mod mtb_feedforward;
pub mod mtb_momentum_management;
pub mod mtb_momentum_management_simple;
pub mod prv_steering;
pub mod rate_servo_full_nonlinear;
pub mod thr_momentum_management;
