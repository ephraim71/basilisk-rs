//! One-degree-of-freedom rotation profiler for a prescribed-motion body:
//! port of upstream Basilisk
//! `simulation/deviceInterface/prescribedRotation1DOF`.
//!
//! The body frame `P` spins relative to its mount frame `M` about the fixed
//! axis `ĥ` (`rot_hat_m`, `M` components) through the angle `θ`. When the
//! last rotation has completed, the module takes the reference angle from
//! `spinning_body_in_msg` and profiles `θ` from rest at its current value to
//! rest at the reference, with a bang-bang or bang-coast-bang acceleration of
//! magnitude `theta_ddot_max_radps2`, optionally smoothed; see
//! [`super::bang_coast_bang_profile`] for the shapes. A reference written
//! while a rotation is in progress is taken up once it completes.
//!
//! Each update publishes `θ` and `θ̇` as a [`HingedRigidBodyMsg`], and the
//! prescribed rotational states `ω_PM = θ̇ ĥ`, `ω'_PM = θ̈ ĥ` and `σ_PM` (the
//! MRP of `θ` about `ĥ`) as a [`PrescribedRotationMsg`] for the
//! prescribed-motion state effector. With a nonzero screw constant `c` the
//! body also translates along `ĥ`, `r_PM = (ρ_0 + c θ) ĥ`, published as a
//! [`PrescribedTranslationMsg`]; otherwise that output is never written.
//!
//! Upstream starts a rotation only when the reference message was written at
//! or before the current time. Stand-in: connected ⇒ written at the current
//! update, so the condition always holds; an unconnected reference reads as
//! the zeroed payload, an angle of zero, as upstream's unwritten one.
//!
//! As upstream, the configured initial angle is applied by `init`; a later
//! reset starts from the angle at which the last rotation began.

use nalgebra::Vector3;

use super::bang_coast_bang_profile::{BangCoastBangProfile, ProfileShape};
use crate::messages::{
    HingedRigidBodyMsg, Input, Output, PrescribedRotationMsg, PrescribedTranslationMsg,
};
use crate::time::NANO2SEC;
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, prv_to_dcm};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`PrescribedRotation1Dof`].
#[derive(Clone, Debug, PartialEq)]
pub struct PrescribedRotation1DofConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `coastOptionBangDuration` \[s\]: duration of each bang of the
    /// bang-coast-bang shapes; zero selects the bang-bang shapes.
    pub coast_option_bang_duration_s: f64,
    /// Upstream `smoothingDuration` \[s\]: duration of the acceleration
    /// ramps; zero leaves the bangs unsmoothed.
    pub smoothing_duration_s: f64,
    /// Upstream `thetaDDotMax` \[rad/s^2\]: bang angular acceleration.
    pub theta_ddot_max_radps2: f64,
    /// Upstream `rotHat_M` \[-\]: rotation axis, `M` components; normalized
    /// by [`PrescribedRotation1Dof::new`], as upstream's setter does.
    pub rot_hat_m: Vector3<f64>,
    /// Upstream `thetaInit` \[rad\]: angle of `P` relative to `M` before the
    /// first rotation.
    pub theta_init_rad: f64,
    /// Upstream `c_screw` \[m/rad\]: screw constant coupling a translation
    /// along `rot_hat_m` to the angle; zero for a pure rotation.
    pub screw_constant_m_per_rad: f64,
    /// Upstream `rhoInit` \[m\]: displacement of `P` along `rot_hat_m` at
    /// zero angle, used with a nonzero screw constant.
    pub init_displacement_m: f64,
}

impl Default for PrescribedRotation1DofConfig {
    fn default() -> Self {
        Self {
            name: "prescribedRotation1DOF".to_string(),
            coast_option_bang_duration_s: 0.0,
            smoothing_duration_s: 0.0,
            theta_ddot_max_radps2: 0.0,
            rot_hat_m: Vector3::x(),
            theta_init_rad: 0.0,
            screw_constant_m_per_rad: 0.0,
            init_displacement_m: 0.0,
        }
    }
}

impl PrescribedRotation1DofConfig {
    fn shape(&self) -> ProfileShape {
        ProfileShape {
            coast_option_bang_duration_s: self.coast_option_bang_duration_s,
            smoothing_duration_s: self.smoothing_duration_s,
            accel_max: self.theta_ddot_max_radps2,
        }
    }
}

/// Profiles a one-axis rotation: upstream `PrescribedRotation1DOF`.
#[derive(Clone, Debug)]
pub struct PrescribedRotation1Dof {
    /// Module configuration, read every update.
    pub config: PrescribedRotation1DofConfig,
    /// Upstream `spinningBodyInMsg`: the reference angle (`theta_rad`).
    pub spinning_body_in_msg: Input<HingedRigidBodyMsg>,
    /// Upstream `spinningBodyOutMsg`: the profiled angle and rate.
    pub spinning_body_out_msg: Output<HingedRigidBodyMsg>,
    /// Upstream `prescribedRotationOutMsg`: the prescribed rotational states
    /// of `P` relative to `M`.
    pub prescribed_rotation_out_msg: Output<PrescribedRotationMsg>,
    /// Upstream `prescribedTranslationOutMsg`: the screw translation of `P`
    /// relative to `M`, written only with a nonzero screw constant.
    pub prescribed_translation_out_msg: Output<PrescribedTranslationMsg>,
    profile: BangCoastBangProfile,
}

impl PrescribedRotation1Dof {
    /// Builds the profiler with unconnected ports, normalizing the rotation
    /// axis.
    ///
    /// # Panics
    ///
    /// If `config.rot_hat_m` is zero or not finite.
    pub fn new(mut config: PrescribedRotation1DofConfig) -> Self {
        let norm = config.rot_hat_m.norm();
        assert!(
            norm.is_finite() && norm > 0.0,
            "prescribedRotation1DOF rot_hat_m must be a finite nonzero vector, got {:?}",
            config.rot_hat_m
        );
        config.rot_hat_m /= norm;
        Self {
            config,
            spinning_body_in_msg: Input::default(),
            spinning_body_out_msg: Output::default(),
            prescribed_rotation_out_msg: Output::default(),
            prescribed_translation_out_msg: Output::default(),
            profile: BangCoastBangProfile::default(),
        }
    }

    /// Upstream `Reset`: at rest at the angle the last rotation began from.
    fn reset_profile(&mut self) {
        if !self.spinning_body_in_msg.is_connected() {
            log::error!(
                "prescribedRotation1DOF '{}' spinning_body_in_msg is not connected",
                self.config.name
            );
        }
        self.profile.reset(self.profile.init);
    }

    /// Upstream `computeSigma_PM`: `σ_PM` from `[PM] = [P P0][P0 M]`, the
    /// rotation since the start of the profile after the start attitude.
    fn sigma_pm(&self) -> Vector3<f64> {
        let rot_hat_m = self.config.rot_hat_m;
        let dcm_pp0 = prv_to_dcm((self.profile.pos - self.profile.init) * rot_hat_m);
        let dcm_p0m = prv_to_dcm(self.profile.init * rot_hat_m);
        dcm_to_mrp(dcm_pp0 * dcm_p0m)
    }

    /// Upstream `writeOutputMessages`.
    fn write_outputs(&self) {
        let rot_hat_m = self.config.rot_hat_m;
        let profile = &self.profile;
        self.spinning_body_out_msg.write(HingedRigidBodyMsg {
            theta_rad: profile.pos,
            theta_dot_radps: profile.vel,
        });
        self.prescribed_rotation_out_msg
            .write(PrescribedRotationMsg {
                omega_pm_p_radps: profile.vel * rot_hat_m,
                omega_prime_pm_p_radps2: profile.accel * rot_hat_m,
                sigma_pm: self.sigma_pm(),
            });
        let screw_constant = self.config.screw_constant_m_per_rad;
        if screw_constant != 0.0 {
            self.prescribed_translation_out_msg
                .write(PrescribedTranslationMsg {
                    r_pm_m_m: (self.config.init_displacement_m + screw_constant * profile.pos)
                        * rot_hat_m,
                    r_prime_pm_m_mps: screw_constant * profile.vel * rot_hat_m,
                    r_prime_prime_pm_m_mps2: screw_constant * profile.accel * rot_hat_m,
                });
        }
    }
}

impl Module for PrescribedRotation1Dof {
    fn init(&mut self) {
        self.profile.init = self.config.theta_init_rad;
        self.reset_profile();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_profile();
    }

    fn update(&mut self, context: &SimulationContext) {
        let reference_rad = self.spinning_body_in_msg.read().theta_rad;
        let t_s = context.current_sim_nanos as f64 * NANO2SEC;
        self.profile
            .update(&self.config.shape(), t_s, reference_rad);
        self.write_outputs();
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `prescribedRotation1DOF/_UnitTest/test_prescribedRotation1DOF.py`
    //! over its full parameter grid with its tolerances, and adds checks of
    //! the prescribed-motion outputs and of screw motion.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{PrescribedRotation1Dof, PrescribedRotation1DofConfig};
    use crate::device_interface::bang_coast_bang_profile::test_support::{
        Sample, check_two_segment_run, two_segment_run,
    };
    use crate::messages::{HingedRigidBodyMsg, Output};
    use crate::utilities::rigid_body_kinematics::prv_to_mrp;
    use crate::{Module, SimulationContext};

    const D2R: f64 = std::f64::consts::PI / 180.0;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn profiler(
        config: PrescribedRotation1DofConfig,
    ) -> (PrescribedRotation1Dof, Output<HingedRigidBodyMsg>) {
        let reference_out = Output::default();
        let mut module = PrescribedRotation1Dof::new(config);
        reference_out.connect_to(&mut module.spinning_body_in_msg);
        module.init();
        (module, reference_out)
    }

    /// Upstream `test_prescribedRotation1DOF` over its grid: two successive
    /// rotations about `x`, 300 s each at 0.1 s.
    #[test]
    fn rotations_reach_their_references_with_consistent_rates() {
        for coast in [0.0, 2.0] {
            for smoothing in [0.0, 2.0] {
                for theta_init in [0.0, -5.0 * D2R] {
                    for theta_ref_1 in [0.0, -10.0 * D2R] {
                        for theta_ddot_max in [0.05 * D2R, 0.1 * D2R] {
                            let theta_ref_2 = 5.0 * D2R;
                            let rot_axis_m = Vector3::x();
                            let (mut module, reference_out) =
                                profiler(PrescribedRotation1DofConfig {
                                    coast_option_bang_duration_s: coast,
                                    smoothing_duration_s: smoothing,
                                    theta_ddot_max_radps2: theta_ddot_max,
                                    rot_hat_m: rot_axis_m,
                                    theta_init_rad: theta_init,
                                    ..Default::default()
                                });
                            let samples =
                                two_segment_run(theta_ref_1, theta_ref_2, |t_nanos, reference| {
                                    reference_out.write(HingedRigidBodyMsg {
                                        theta_rad: reference,
                                        theta_dot_radps: 0.0,
                                    });
                                    module.update(&context(t_nanos));
                                    let angle = module.spinning_body_out_msg.read();
                                    let rotation = module.prescribed_rotation_out_msg.read();
                                    Sample {
                                        pos: angle.theta_rad,
                                        vel: angle.theta_dot_radps,
                                        accel: rotation.omega_prime_pm_p_radps2.dot(&rot_axis_m),
                                    }
                                });
                            check_two_segment_run(
                                &samples,
                                [theta_ref_1, theta_ref_2],
                                smoothing > 0.0,
                                1.0 / D2R,
                                1e-8,
                                &format!(
                                    "coast {coast}, smoothing {smoothing}, theta_init \
                                     {theta_init}, theta_ref_1 {theta_ref_1}, theta_ddot_max \
                                     {theta_ddot_max}"
                                ),
                            );
                        }
                    }
                }
            }
        }
    }

    /// The bang-bang profile in closed form: `θ̈ = ±a`, switching halfway
    /// through `T = 2 sqrt(|Δθ| / a)`.
    #[test]
    fn bang_bang_profile_matches_its_closed_form() {
        let theta_ddot_max = 0.01;
        let delta = 0.4;
        let (mut module, reference_out) = profiler(PrescribedRotation1DofConfig {
            theta_ddot_max_radps2: theta_ddot_max,
            rot_hat_m: Vector3::new(0.0, 0.0, 2.0),
            ..Default::default()
        });
        reference_out.write(HingedRigidBodyMsg {
            theta_rad: delta,
            theta_dot_radps: 0.0,
        });
        let total_s = 2.0 * (delta / theta_ddot_max).sqrt();
        for fraction in [0.0, 0.25, 0.4, 0.6, 0.9, 1.1] {
            let t_nanos = (fraction * total_s * 1e9).round() as u64;
            module.update(&context(t_nanos));
            let t_s = t_nanos as f64 * 1e-9;
            let (theta, theta_dot, theta_ddot) = if fraction > 1.0 {
                (delta, 0.0, 0.0)
            } else if fraction < 0.5 {
                (
                    0.5 * theta_ddot_max * t_s * t_s,
                    theta_ddot_max * t_s,
                    theta_ddot_max,
                )
            } else {
                let to_go = total_s - t_s;
                (
                    delta - 0.5 * theta_ddot_max * to_go * to_go,
                    theta_ddot_max * to_go,
                    -theta_ddot_max,
                )
            };
            let angle = module.spinning_body_out_msg.read();
            let rotation = module.prescribed_rotation_out_msg.read();
            assert!((angle.theta_rad - theta).abs() < 1e-12, "t = {t_s}");
            assert!(
                (angle.theta_dot_radps - theta_dot).abs() < 1e-12,
                "t = {t_s}"
            );
            assert!((rotation.omega_pm_p_radps - theta_dot * Vector3::z()).norm() < 1e-12);
            assert!((rotation.omega_prime_pm_p_radps2 - theta_ddot * Vector3::z()).norm() < 1e-12);
            assert!((rotation.sigma_pm - prv_to_mrp(theta * Vector3::z())).norm() < 1e-12);
        }
    }

    /// Screw motion: the translation output follows the angle along the
    /// axis, and stays unwritten without a screw constant.
    #[test]
    fn screw_motion_couples_a_translation_to_the_angle() {
        let screw_constant = 0.05;
        let init_displacement = 0.2;
        let (mut module, reference_out) = profiler(PrescribedRotation1DofConfig {
            theta_ddot_max_radps2: 0.02,
            smoothing_duration_s: 1.0,
            rot_hat_m: Vector3::y(),
            theta_init_rad: 0.1,
            screw_constant_m_per_rad: screw_constant,
            init_displacement_m: init_displacement,
            ..Default::default()
        });
        reference_out.write(HingedRigidBodyMsg {
            theta_rad: -0.3,
            theta_dot_radps: 0.0,
        });
        for tick in 0..100 {
            module.update(&context(tick * 100_000_000));
            let angle = module.spinning_body_out_msg.read();
            let rotation = module.prescribed_rotation_out_msg.read();
            let translation = module.prescribed_translation_out_msg.read();
            let theta_ddot = rotation.omega_prime_pm_p_radps2.y;
            assert_eq!(
                translation.r_pm_m_m,
                (init_displacement + screw_constant * angle.theta_rad) * Vector3::y()
            );
            assert_eq!(
                translation.r_prime_pm_m_mps,
                screw_constant * angle.theta_dot_radps * Vector3::y()
            );
            assert_eq!(
                translation.r_prime_prime_pm_m_mps2,
                screw_constant * theta_ddot * Vector3::y()
            );
        }

        let (mut pure, reference_out) = profiler(PrescribedRotation1DofConfig {
            theta_ddot_max_radps2: 0.02,
            ..Default::default()
        });
        reference_out.write(HingedRigidBodyMsg {
            theta_rad: 0.3,
            theta_dot_radps: 0.0,
        });
        pure.update(&context(1_000_000_000));
        assert_eq!(
            pure.prescribed_translation_out_msg.read(),
            Default::default()
        );
    }

    /// A reference changed mid-rotation is taken up only once the rotation
    /// completes; a later reset starts from where the last rotation began.
    #[test]
    fn a_new_reference_waits_for_the_rotation_in_progress() {
        let (mut module, reference_out) = profiler(PrescribedRotation1DofConfig {
            theta_ddot_max_radps2: 0.01,
            ..Default::default()
        });
        reference_out.write(HingedRigidBodyMsg {
            theta_rad: 0.04,
            theta_dot_radps: 0.0,
        });
        // T = 2 sqrt(0.04 / 0.01) = 4 s.
        module.update(&context(0));
        reference_out.write(HingedRigidBodyMsg {
            theta_rad: -1.0,
            theta_dot_radps: 0.0,
        });
        module.update(&context(2_000_000_000));
        assert!((module.spinning_body_out_msg.read().theta_rad - 0.02).abs() < 1e-15);
        module.update(&context(5_000_000_000));
        assert_eq!(module.spinning_body_out_msg.read().theta_rad, 0.04);
        // The new rotation starts here, from 0.04.
        module.update(&context(6_000_000_000));
        assert_eq!(module.spinning_body_out_msg.read().theta_rad, 0.04);
        module.update(&context(7_000_000_000));
        assert!(module.spinning_body_out_msg.read().theta_rad < 0.04);

        Module::reset(&mut module, &context(0));
        module.update(&context(0));
        assert_eq!(module.spinning_body_out_msg.read().theta_rad, 0.04);
    }

    #[test]
    #[should_panic(expected = "rot_hat_m must be a finite nonzero vector")]
    fn rejects_a_zero_rotation_axis() {
        let _ = PrescribedRotation1Dof::new(PrescribedRotation1DofConfig {
            rot_hat_m: Vector3::zeros(),
            ..Default::default()
        });
    }
}
