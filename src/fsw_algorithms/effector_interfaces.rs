//! Effector interfaces: map body torques and forces onto wheels, thrusters, torque rods and motors.

pub mod dipole_mapping;
pub mod dv_att_effect;
pub mod force_torque_thr_force_mapping;
pub mod hinged_rigid_body_pid_motor;
pub mod prescribed_rot_2dof;
pub mod rw_motor_torque;
pub mod rw_motor_voltage;
pub mod rw_null_space;
pub mod solar_array_reference;
pub mod sun_safe_acs;
pub mod thr_firing_remainder;
pub mod thr_firing_schmitt;
pub mod thr_force_mapping;
pub mod thr_momentum_dumping;
pub mod thrust_rw_desat;
pub mod thruster_platform_reference;
pub mod thruster_platform_state;
pub mod torque2_dipole;
pub mod torque_scheduler;
