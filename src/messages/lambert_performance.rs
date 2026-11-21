//! Lambert root-finder diagnostics (upstream `LambertPerformanceMsgPayload`).

use serde::{Deserialize, Serialize};

use super::LambertSolutionSpecifier;
use super::flatten::impl_telemetry;

/// Root-finder result behind one Lambert solution.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LambertRootPerformance {
    /// Upstream `x` or `xSol2` [-]: the free variable of the solution.
    pub x: f64,
    /// Upstream `numIter` or `numIterSol2`: root-finder iterations taken.
    pub num_iter: usize,
    /// Upstream `errX` or `errXSol2` [-]: magnitude of the final step or of
    /// the converged correction in `x`.
    pub err_x: f64,
}

/// Root-finder diagnostics of both Lambert solutions: upstream
/// `LambertPerformanceMsg`, whose six fields are grouped here by solution. An
/// invalid solution has all-zero diagnostics.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LambertPerformanceMsg {
    /// Upstream `x`, `numIter`, `errX`: the first solution.
    pub first: LambertRootPerformance,
    /// Upstream `xSol2`, `numIterSol2`, `errXSol2`: the second solution.
    pub second: LambertRootPerformance,
}

impl LambertPerformanceMsg {
    /// The diagnostics of the solution `which` selects.
    pub fn root(&self, which: LambertSolutionSpecifier) -> &LambertRootPerformance {
        match which {
            LambertSolutionSpecifier::First => &self.first,
            LambertSolutionSpecifier::Second => &self.second,
        }
    }
}

impl_telemetry!(LambertRootPerformance { x, num_iter, err_x });
impl_telemetry!(LambertPerformanceMsg { first, second });
