//! Lambert validation diagnostics (upstream `LambertValidatorMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Why `lambertValidator` did or did not command its delta-v: upstream
/// `LambertValidatorMsg`. The `failed_*` flags are upstream `int` 0/1 flags.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LambertValidatorMsg {
    /// Upstream `failedValidLambert`: no valid Lambert solution.
    pub failed_valid_lambert: bool,
    /// Upstream `failedNumIterationsLambert`: the root finder took too many
    /// iterations.
    pub failed_num_iterations_lambert: bool,
    /// Upstream `failedXToleranceLambert`: the root finder's last step was too
    /// large.
    pub failed_x_tolerance_lambert: bool,
    /// Upstream `failedXSolutionConvergence`: `x` moved too far since the
    /// previous update.
    pub failed_x_solution_convergence: bool,
    /// Upstream `failedDvSolutionConvergence`: the delta-v moved too far since
    /// the previous update.
    pub failed_dv_solution_convergence: bool,
    /// Upstream `failedDistanceTargetConstraint`: a dispersed trajectory ends
    /// too far from the target.
    pub failed_distance_target_constraint: bool,
    /// Upstream `failedOrbitRadiusConstraint`: a dispersed trajectory comes
    /// too close to the central body.
    pub failed_orbit_radius_constraint: bool,
    /// Upstream `xSolutionDifference` [-]: change in `x` since the previous
    /// update.
    pub x_solution_difference: f64,
    /// Upstream `dvSolutionDifference` [m/s]: magnitude of the change in
    /// delta-v since the previous update.
    pub dv_solution_difference_mps: f64,
    /// Upstream `violationsDistanceTarget`: dispersed trajectories that miss
    /// the target.
    pub violations_distance_target: usize,
    /// Upstream `violationsOrbitRadius`: dispersed trajectories that come too
    /// close to the central body.
    pub violations_orbit_radius: usize,
    /// Upstream `dv_N` [m/s]: the delta-v that is commanded once every check
    /// passes, inertial components.
    pub dv_n_mps: Vector3<f64>,
}

impl_telemetry!(LambertValidatorMsg {
    failed_valid_lambert,
    failed_num_iterations_lambert,
    failed_x_tolerance_lambert,
    failed_x_solution_convergence,
    failed_dv_solution_convergence,
    failed_distance_target_constraint,
    failed_orbit_radius_constraint,
    x_solution_difference,
    dv_solution_difference_mps,
    violations_distance_target,
    violations_orbit_radius,
    dv_n_mps,
});
