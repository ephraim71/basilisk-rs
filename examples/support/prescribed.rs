//! Setup shared by the ports of upstream's prescribed-motion example
//! scenarios (`scenarioPrescribedMotionWith*Branching`,
//! `scenarioPrescribedScrewMotion`, `scenarioRotTransArrayDeployment`): the
//! box inertias they compute, the one-axis rotation and translation
//! profilers, and the angular momentum check of a spacecraft that starts at
//! rest.

#![allow(dead_code)]

use basilisk_rs::device_interface::prescribed_linear_translation::{
    PrescribedLinearTranslation, PrescribedLinearTranslationConfig,
};
use basilisk_rs::device_interface::prescribed_rotation_1dof::{
    PrescribedRotation1Dof, PrescribedRotation1DofConfig,
};
use basilisk_rs::messages::SpacecraftDiagnosticsMsg;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use nalgebra::{Matrix3, Vector3};

/// Upstream's box inertia about its center of mass:
/// `I11 = m (l² + d²) / 12`, `I22 = m (l² + w²) / 12`,
/// `I33 = m (w² + d²) / 12` \[kg m^2\].
pub fn box_inertia_kg_m2(mass_kg: f64, length_m: f64, width_m: f64, depth_m: f64) -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(
        mass_kg * (length_m * length_m + depth_m * depth_m) / 12.0,
        mass_kg * (length_m * length_m + width_m * width_m) / 12.0,
        mass_kg * (width_m * width_m + depth_m * depth_m) / 12.0,
    ))
}

/// A hub at rest at the origin, `r_BcB_B = 0`, integrated with RK4 (the
/// scenarios set no integrator).
pub fn hub_at_rest(mass_kg: f64, inertia_kg_m2: Matrix3<f64>, step_nanos: u64) -> Spacecraft {
    Spacecraft::new(SpacecraftConfig {
        mass_kg,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2,
        integration_step_nanos: step_nanos,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    })
}

/// Upstream `prescribedRotation1DOF.PrescribedRotation1DOF()` with
/// `setRotHat_M`, `setThetaDDotMax`, `setThetaInit`,
/// `setCoastOptionBangDuration` and `setSmoothingDuration`.
pub fn rotation_profiler(
    name: &str,
    rot_hat_m: Vector3<f64>,
    theta_ddot_max_radps2: f64,
    theta_init_rad: f64,
    coast_option_bang_duration_s: f64,
    smoothing_duration_s: f64,
) -> PrescribedRotation1Dof {
    PrescribedRotation1Dof::new(PrescribedRotation1DofConfig {
        name: name.to_string(),
        coast_option_bang_duration_s,
        smoothing_duration_s,
        theta_ddot_max_radps2,
        rot_hat_m,
        theta_init_rad,
        ..PrescribedRotation1DofConfig::default()
    })
}

/// Upstream `prescribedLinearTranslation.PrescribedLinearTranslation()` with
/// `setTransHat_M`, `setTransAccelMax`, `setTransPosInit`,
/// `setCoastOptionBangDuration` and `setSmoothingDuration`.
pub fn translation_profiler(
    name: &str,
    trans_hat_m: Vector3<f64>,
    trans_accel_max_mps2: f64,
    trans_pos_init_m: f64,
    coast_option_bang_duration_s: f64,
    smoothing_duration_s: f64,
) -> PrescribedLinearTranslation {
    PrescribedLinearTranslation::new(PrescribedLinearTranslationConfig {
        name: name.to_string(),
        coast_option_bang_duration_s,
        smoothing_duration_s,
        trans_accel_max_mps2,
        trans_hat_m,
        trans_pos_init_m,
    })
}

/// For a spacecraft that starts at rest, the total rotational angular
/// momentum about its center of mass at the last of `diagnostics`, once the
/// prescribed motion has ended, relative to `scale` (the momentum its
/// internal motion exchanges, such as the largest hub momentum): zero for
/// exact conservation.
///
/// While a profile accelerates, the reported momentum is off by the
/// prescribed bodies' inertia times `θ̈ Δt`: a prescribed body latches its
/// profiler's message once per step, so its rate leads the profile by one
/// step's change (see the `prescribed_rot_2dof_drives_the_prescribed_body`
/// test of `prescribed_motion_state_effector`). The offset vanishes when the
/// motion ends, but for a residual of the profile switches that fall
/// between steps, which is when this is checked.
pub fn final_relative_momentum(diagnostics: &[SpacecraftDiagnosticsMsg], scale: f64) -> f64 {
    diagnostics
        .last()
        .expect("diagnostics samples")
        .rotational_angular_momentum_inertial_kg_m2ps
        .norm()
        / scale
}
