//! Orbit control: Lambert transfer guidance, orbit-element offsets and
//! small-body waypoint control.
//!
//! The Lambert modules chain through messages: [`lambert_planner`] poses the
//! problem, [`lambert_solver`] solves it, [`lambert_validator`] checks the
//! solution and commands the first delta-v, and [`lambert_second_dv`] commands
//! the arrival burn towards the velocity [`lambert_surface_relative_velocity`]
//! asks for. The solver itself is [`crate::utilities::lambert`].

pub mod lambert_planner;
pub mod lambert_second_dv;
pub mod lambert_solver;
pub mod lambert_surface_relative_velocity;
pub mod lambert_validator;
pub mod orb_elem_offset;
pub mod small_body_waypoint_feedback;

#[cfg(test)]
pub(crate) mod test_support;
