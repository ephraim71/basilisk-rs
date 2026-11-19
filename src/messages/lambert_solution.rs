//! Lambert solution velocities (upstream `LambertSolutionMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Which of the two Lambert solutions a consumer uses: upstream
/// `lambertSolutionSpecifier`, a `double` that must be 1 or 2.
///
/// A zero-revolution problem has only the first. A multi-revolution problem
/// has the first on the branch of the flight-time curve left of its minimum
/// and the second on the right.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum LambertSolutionSpecifier {
    /// Upstream `1`, the default: `v1_N`, `v2_N`, `valid` and `x`, `numIter`,
    /// `errX`.
    #[default]
    First = 1,
    /// Upstream `2`: `v1Sol2_N`, `v2Sol2_N`, `validSol2` and `xSol2`,
    /// `numIterSol2`, `errXSol2`.
    Second = 2,
}

/// Endpoint velocities of one Lambert solution.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LambertTransferVelocities {
    /// Upstream `v1_N` or `v1Sol2_N` [m/s]: velocity at the start of the
    /// transfer, inertial components.
    pub v1_n_mps: Vector3<f64>,
    /// Upstream `v2_N` or `v2Sol2_N` [m/s]: velocity at the end of the
    /// transfer, inertial components.
    pub v2_n_mps: Vector3<f64>,
    /// Upstream `valid` or `validSol2` (`int` 0/1): whether the solution
    /// exists. The velocities of an invalid solution are zero.
    pub valid: bool,
}

/// Both solutions of a Lambert problem: upstream `LambertSolutionMsg`, whose
/// six fields are grouped here by solution.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LambertSolutionMsg {
    /// Upstream `v1_N`, `v2_N`, `valid`: the first solution.
    pub first: LambertTransferVelocities,
    /// Upstream `v1Sol2_N`, `v2Sol2_N`, `validSol2`: the second solution,
    /// multi-revolution problems only.
    pub second: LambertTransferVelocities,
}

impl LambertSolutionMsg {
    /// The solution `which` selects.
    pub fn solution(&self, which: LambertSolutionSpecifier) -> &LambertTransferVelocities {
        match which {
            LambertSolutionSpecifier::First => &self.first,
            LambertSolutionSpecifier::Second => &self.second,
        }
    }
}

impl_telemetry!(enum LambertSolutionSpecifier as u8);
impl_telemetry!(LambertTransferVelocities {
    v1_n_mps,
    v2_n_mps,
    valid,
});
impl_telemetry!(LambertSolutionMsg { first, second });
