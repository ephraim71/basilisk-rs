//! Lambert problem definition (upstream `LambertProblemMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

pub use crate::utilities::lambert::{LambertSolverMethod, LambertTransferDirection};

use super::flatten::impl_telemetry;

/// Two positions, a flight time and a central body for a Lambert solver:
/// upstream `LambertProblemMsg`.
///
/// Upstream carries the method and direction as raw integers that the solver
/// validates, and the revolution count as a signed `int`; here their types
/// admit only the values upstream accepts.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LambertProblemMsg {
    /// Upstream `solverMethod`: Lambert algorithm (Izzo, the zeroed default,
    /// or Gooding).
    pub solver_method: LambertSolverMethod,
    /// Upstream `r1_N` \[m\]: position at the start of the transfer, inertial
    /// components.
    pub r1_n_m: Vector3<f64>,
    /// Upstream `r2_N` \[m\]: position at the end of the transfer, inertial
    /// components.
    pub r2_n_m: Vector3<f64>,
    /// Upstream `transferTime` \[s\]: flight time from `r1_n_m` to `r2_n_m`.
    pub transfer_time_s: f64,
    /// Upstream `mu` [m^3/s^2]: gravitational parameter of the central body.
    pub mu_m3ps2: f64,
    /// Upstream `numRevolutions` [-]: complete revolutions before arrival.
    pub num_revolutions: usize,
    /// Upstream `transferDirection`: sense of the transfer about the inertial
    /// +z axis (prograde, the zeroed default, or retrograde).
    pub transfer_direction: LambertTransferDirection,
}

impl_telemetry!(enum LambertSolverMethod as u8);
impl_telemetry!(enum LambertTransferDirection as u8);
impl_telemetry!(LambertProblemMsg {
    solver_method,
    r1_n_m,
    r2_n_m,
    transfer_time_s,
    mu_m3ps2,
    num_revolutions,
    transfer_direction,
});
