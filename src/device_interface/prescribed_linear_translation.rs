//! Linear translation profiler for a prescribed-motion body: port of upstream
//! Basilisk `simulation/deviceInterface/prescribedLinearTranslation`.
//!
//! The body frame `P` translates relative to its mount frame `M` along the
//! fixed axis `t̂` (`trans_hat_m`, `M` components) by the displacement `ρ`.
//! When the last translation has completed, the module takes the reference
//! displacement from `linear_translation_rigid_body_in_msg` and profiles `ρ`
//! from rest at its current value to rest at the reference, with a bang-bang
//! or bang-coast-bang acceleration of magnitude `trans_accel_max_mps2`,
//! optionally smoothed; see [`super::bang_coast_bang_profile`], the profiler
//! it shares with
//! [`PrescribedRotation1Dof`](super::prescribed_rotation_1dof::PrescribedRotation1Dof).
//! A reference written while a translation is in progress is taken up once
//! it completes.
//!
//! Each update publishes `ρ` and `ρ̇` as a [`LinearTranslationRigidBodyMsg`]
//! and `r_PM = ρ t̂`, `r'_PM = ρ̇ t̂`, `r''_PM = ρ̈ t̂` as a
//! [`PrescribedTranslationMsg`] for the prescribed-motion state effector.
//! `trans_hat_m` is used as given: upstream's setter does not normalize it.
//!
//! Upstream starts a translation only when the reference message was written
//! at or before the current time. Stand-in: connected ⇒ written at the
//! current update, so the condition always holds; an unconnected reference
//! reads as the zeroed payload, a displacement of zero, as upstream's
//! unwritten one.
//!
//! As upstream, the configured initial displacement is applied by `init`; a
//! later reset starts from the displacement at which the last translation
//! began.

use nalgebra::Vector3;

use super::bang_coast_bang_profile::{BangCoastBangProfile, ProfileShape};
use crate::messages::{Input, LinearTranslationRigidBodyMsg, Output, PrescribedTranslationMsg};
use crate::time::NANO2SEC;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`PrescribedLinearTranslation`].
#[derive(Clone, Debug, PartialEq)]
pub struct PrescribedLinearTranslationConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `coastOptionBangDuration` \[s\]: duration of each bang of the
    /// bang-coast-bang shapes; zero selects the bang-bang shapes.
    pub coast_option_bang_duration_s: f64,
    /// Upstream `smoothingDuration` \[s\]: duration of the acceleration
    /// ramps; zero leaves the bangs unsmoothed.
    pub smoothing_duration_s: f64,
    /// Upstream `transAccelMax` \[m/s^2\]: bang acceleration.
    pub trans_accel_max_mps2: f64,
    /// Upstream `transHat_M` \[-\]: translation axis, `M` components.
    pub trans_hat_m: Vector3<f64>,
    /// Upstream `transPosInit` \[m\]: displacement of `P` along
    /// `trans_hat_m` before the first translation.
    pub trans_pos_init_m: f64,
}

impl Default for PrescribedLinearTranslationConfig {
    fn default() -> Self {
        Self {
            name: "prescribedLinearTranslation".to_string(),
            coast_option_bang_duration_s: 0.0,
            smoothing_duration_s: 0.0,
            trans_accel_max_mps2: 0.0,
            trans_hat_m: Vector3::x(),
            trans_pos_init_m: 0.0,
        }
    }
}

impl PrescribedLinearTranslationConfig {
    fn shape(&self) -> ProfileShape {
        ProfileShape {
            coast_option_bang_duration_s: self.coast_option_bang_duration_s,
            smoothing_duration_s: self.smoothing_duration_s,
            accel_max: self.trans_accel_max_mps2,
        }
    }
}

/// Profiles a one-axis translation: upstream `PrescribedLinearTranslation`.
#[derive(Clone, Debug)]
pub struct PrescribedLinearTranslation {
    /// Module configuration, read every update.
    pub config: PrescribedLinearTranslationConfig,
    /// Upstream `linearTranslationRigidBodyInMsg`: the reference
    /// displacement (`rho_m`).
    pub linear_translation_rigid_body_in_msg: Input<LinearTranslationRigidBodyMsg>,
    /// Upstream `linearTranslationRigidBodyOutMsg`: the profiled displacement
    /// and rate.
    pub linear_translation_rigid_body_out_msg: Output<LinearTranslationRigidBodyMsg>,
    /// Upstream `prescribedTranslationOutMsg`: the prescribed translational
    /// states of `P` relative to `M`.
    pub prescribed_translation_out_msg: Output<PrescribedTranslationMsg>,
    profile: BangCoastBangProfile,
}

impl PrescribedLinearTranslation {
    /// Builds the profiler with unconnected ports.
    pub fn new(config: PrescribedLinearTranslationConfig) -> Self {
        Self {
            config,
            linear_translation_rigid_body_in_msg: Input::default(),
            linear_translation_rigid_body_out_msg: Output::default(),
            prescribed_translation_out_msg: Output::default(),
            profile: BangCoastBangProfile::default(),
        }
    }

    /// Upstream `Reset`: at rest at the displacement the last translation
    /// began from.
    fn reset_profile(&mut self) {
        if !self.linear_translation_rigid_body_in_msg.is_connected() {
            log::error!(
                "prescribedLinearTranslation '{}' linear_translation_rigid_body_in_msg is not \
                 connected",
                self.config.name
            );
        }
        self.profile.reset(self.profile.init);
    }

    /// Upstream `writeOutputMessages`.
    fn write_outputs(&self) {
        let trans_hat_m = self.config.trans_hat_m;
        let profile = &self.profile;
        self.prescribed_translation_out_msg
            .write(PrescribedTranslationMsg {
                r_pm_m_m: profile.pos * trans_hat_m,
                r_prime_pm_m_mps: profile.vel * trans_hat_m,
                r_prime_prime_pm_m_mps2: profile.accel * trans_hat_m,
            });
        self.linear_translation_rigid_body_out_msg
            .write(LinearTranslationRigidBodyMsg {
                rho_m: profile.pos,
                rho_dot_mps: profile.vel,
            });
    }
}

impl Module for PrescribedLinearTranslation {
    fn init(&mut self) {
        self.profile.init = self.config.trans_pos_init_m;
        self.reset_profile();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_profile();
    }

    fn update(&mut self, context: &SimulationContext) {
        let reference_m = self.linear_translation_rigid_body_in_msg.read().rho_m;
        let t_s = context.current_sim_nanos as f64 * NANO2SEC;
        self.profile.update(&self.config.shape(), t_s, reference_m);
        self.write_outputs();
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream
    //! `prescribedLinearTranslation/_UnitTest/test_prescribedLinearTranslation.py`
    //! over its full parameter grid with its tolerances.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{PrescribedLinearTranslation, PrescribedLinearTranslationConfig};
    use crate::device_interface::bang_coast_bang_profile::test_support::{
        Sample, check_two_segment_run, two_segment_run,
    };
    use crate::messages::{LinearTranslationRigidBodyMsg, Output};
    use crate::{Module, SimulationContext};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn profiler(
        config: PrescribedLinearTranslationConfig,
    ) -> (
        PrescribedLinearTranslation,
        Output<LinearTranslationRigidBodyMsg>,
    ) {
        let reference_out = Output::default();
        let mut module = PrescribedLinearTranslation::new(config);
        reference_out.connect_to(&mut module.linear_translation_rigid_body_in_msg);
        module.init();
        (module, reference_out)
    }

    /// Upstream `test_prescribedLinearTranslation` over its grid: two
    /// successive translations along `x`, 300 s each at 0.1 s.
    #[test]
    fn translations_reach_their_references_with_consistent_rates() {
        for coast in [0.0, 2.0] {
            for smoothing in [0.0, 2.0] {
                for trans_pos_init in [0.0, -0.5] {
                    for trans_pos_ref_1 in [0.0, -1.0] {
                        for trans_accel_max in [0.01, 0.005] {
                            let trans_pos_ref_2 = 0.5;
                            let trans_axis_m = Vector3::x();
                            let (mut module, reference_out) =
                                profiler(PrescribedLinearTranslationConfig {
                                    coast_option_bang_duration_s: coast,
                                    smoothing_duration_s: smoothing,
                                    trans_accel_max_mps2: trans_accel_max,
                                    trans_hat_m: trans_axis_m,
                                    trans_pos_init_m: trans_pos_init,
                                    ..Default::default()
                                });
                            let samples = two_segment_run(
                                trans_pos_ref_1,
                                trans_pos_ref_2,
                                |t_nanos, reference| {
                                    reference_out.write(LinearTranslationRigidBodyMsg {
                                        rho_m: reference,
                                        rho_dot_mps: 0.0,
                                    });
                                    module.update(&context(t_nanos));
                                    let states = module.prescribed_translation_out_msg.read();
                                    Sample {
                                        pos: states.r_pm_m_m.dot(&trans_axis_m),
                                        vel: states.r_prime_pm_m_mps.dot(&trans_axis_m),
                                        accel: states.r_prime_prime_pm_m_mps2.dot(&trans_axis_m),
                                    }
                                },
                            );
                            check_two_segment_run(
                                &samples,
                                [trans_pos_ref_1, trans_pos_ref_2],
                                smoothing > 0.0,
                                1.0,
                                1e-8,
                                &format!(
                                    "coast {coast}, smoothing {smoothing}, trans_pos_init \
                                     {trans_pos_init}, trans_pos_ref_1 {trans_pos_ref_1}, \
                                     trans_accel_max {trans_accel_max}"
                                ),
                            );
                        }
                    }
                }
            }
        }
    }

    /// The bang-coast-bang profile in closed form: `ρ̈ = a` for `t_b`, a coast
    /// at `a t_b`, then `ρ̈ = −a` for `t_b`.
    #[test]
    fn bang_coast_bang_profile_matches_its_closed_form() {
        let accel = 0.01;
        let t_bang = 2.0;
        let delta = 0.1;
        let (mut module, reference_out) = profiler(PrescribedLinearTranslationConfig {
            coast_option_bang_duration_s: t_bang,
            trans_accel_max_mps2: accel,
            trans_hat_m: Vector3::new(0.0, 3.0, 4.0),
            ..Default::default()
        });
        reference_out.write(LinearTranslationRigidBodyMsg {
            rho_m: delta,
            rho_dot_mps: 0.0,
        });
        let coast_speed = accel * t_bang;
        let bang_distance = 0.5 * accel * t_bang * t_bang;
        let t_coast_end = t_bang + (delta - 2.0 * bang_distance) / coast_speed;
        let total_s = t_coast_end + t_bang;
        for t_s in [0.0, 1.0, 3.0, 4.0, t_coast_end + 1.0, total_s + 1.0] {
            module.update(&context((t_s * 1e9) as u64));
            let (rho, rho_dot, rho_ddot) = if t_s <= t_bang {
                (0.5 * accel * t_s * t_s, accel * t_s, accel)
            } else if t_s <= t_coast_end {
                (
                    bang_distance + coast_speed * (t_s - t_bang),
                    coast_speed,
                    0.0,
                )
            } else if t_s <= total_s {
                let to_go = total_s - t_s;
                (delta - 0.5 * accel * to_go * to_go, accel * to_go, -accel)
            } else {
                (delta, 0.0, 0.0)
            };
            let scalar = module.linear_translation_rigid_body_out_msg.read();
            let states = module.prescribed_translation_out_msg.read();
            let axis = Vector3::new(0.0, 3.0, 4.0);
            assert!((scalar.rho_m - rho).abs() < 1e-12, "t = {t_s}");
            assert!((scalar.rho_dot_mps - rho_dot).abs() < 1e-12, "t = {t_s}");
            assert!((states.r_pm_m_m - rho * axis).norm() < 1e-12, "t = {t_s}");
            assert!((states.r_prime_pm_m_mps - rho_dot * axis).norm() < 1e-12);
            assert!((states.r_prime_prime_pm_m_mps2 - rho_ddot * axis).norm() < 1e-12);
        }
    }

    #[test]
    fn reset_starts_from_where_the_last_translation_began() {
        let (mut module, reference_out) = profiler(PrescribedLinearTranslationConfig {
            trans_accel_max_mps2: 0.01,
            ..Default::default()
        });
        let rho_at = |module: &PrescribedLinearTranslation| {
            module.linear_translation_rigid_body_out_msg.read().rho_m
        };
        // 0 → 0.25 m takes 2 sqrt(0.25 / 0.01) = 10 s.
        reference_out.write(LinearTranslationRigidBodyMsg {
            rho_m: 0.25,
            rho_dot_mps: 0.0,
        });
        module.update(&context(0));
        module.update(&context(11_000_000_000));
        assert_eq!(rho_at(&module), 0.25);
        reference_out.write(LinearTranslationRigidBodyMsg {
            rho_m: 1.0,
            rho_dot_mps: 0.0,
        });
        module.update(&context(12_000_000_000));
        module.update(&context(13_000_000_000));
        assert!(rho_at(&module) > 0.25);

        Module::reset(&mut module, &context(0));
        module.update(&context(0));
        assert_eq!(rho_at(&module), 0.25);

        module.init();
        module.update(&context(0));
        assert_eq!(rho_at(&module), 0.0);
    }
}
