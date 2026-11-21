//! Bang-bang rotational profile for a prescribed-motion body driven by two
//! successive reference rotations.
//!
//! Port of upstream Basilisk `prescribedRot2DOF`. Two reference angles about
//! two axes (the first in the mount frame M, the second in the intermediate
//! frame P1) are composed into a single principal rotation from the body's
//! current attitude to the final reference. That rotation is profiled with the
//! maximum angular acceleration for its first half and the opposite
//! deceleration for its second half, ending at rest. The profiled
//! `omega_PM_P`, `omegaPrime_PM_P` and `sigma_PM` are published as a
//! [`PrescribedRotationMsg`]. A new maneuver starts on the first update after
//! the previous one has completed.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{HingedRigidBodyMsg, Input, Output, PrescribedRotationMsg};
use crate::time::NANO2SEC;
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, dcm_to_prv, mrp_to_dcm, prv_to_dcm};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`PrescribedRot2Dof`]; upstream leaves all of
/// them zero-initialized and requires the user to set them.
#[derive(Clone, Debug)]
pub struct PrescribedRot2DofConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Maximum angular acceleration of the maneuver (upstream `phiDDotMax`)
    /// \[rad/s^2\].
    pub phi_ddot_max_radps2: f64,
    /// Axis of the first reference rotation, M frame (upstream `rotAxis1_M`).
    /// Normalized in place when a maneuver starts, as upstream.
    pub rot_axis1_m: Vector3<f64>,
    /// Axis of the second reference rotation, P1 frame (upstream
    /// `rotAxis2_P1`). Normalized in place when a maneuver starts.
    pub rot_axis2_p1: Vector3<f64>,
    /// Attitude of P relative to M before the first update (the value the
    /// user assigns to upstream `sigma_PM`). Applied by `init`; later resets
    /// keep the attitude reached so far, as upstream.
    pub initial_sigma_pm: Vector3<f64>,
}

impl Default for PrescribedRot2DofConfig {
    fn default() -> Self {
        Self {
            name: "prescribedRot2DOF".to_string(),
            phi_ddot_max_radps2: 0.0,
            rot_axis1_m: Vector3::zeros(),
            rot_axis2_p1: Vector3::zeros(),
            initial_sigma_pm: Vector3::zeros(),
        }
    }
}

/// Profiled PRV angle, rate and acceleration at one instant.
struct ProfileSample {
    phi_rad: f64,
    phi_dot_radps: f64,
    phi_ddot_radps2: f64,
}

/// Two-reference-angle rotational profiler.
#[derive(Clone, Debug)]
pub struct PrescribedRot2Dof {
    /// Module configuration.
    pub config: PrescribedRot2DofConfig,
    /// First reference angle (upstream `spinningBodyRef1InMsg`).
    pub spinning_body_ref1_in_msg: Input<HingedRigidBodyMsg>,
    /// Second reference angle (upstream `spinningBodyRef2InMsg`).
    pub spinning_body_ref2_in_msg: Input<HingedRigidBodyMsg>,
    /// Profiled rotational state (upstream `prescribedRotationOutMsg`).
    pub prescribed_rotation_out_msg: Output<PrescribedRotationMsg>,
    /// Current attitude of P relative to M (upstream `sigma_PM`).
    sigma_pm: Vector3<f64>,
    is_maneuver_complete: bool,
    maneuver_start_time_s: f64,
    maneuver_switch_time_s: f64,
    maneuver_end_time_s: f64,
    /// Principal axis of the current maneuver, M frame (upstream `rotAxis_M`).
    rot_axis_m: Vector3<f64>,
    phi_ref_rad: f64,
    phi_rad: f64,
    phi_ref_accum_rad: f64,
    phi_accum_rad: f64,
    /// Parabola constants of the acceleration and deceleration halves.
    a: f64,
    b: f64,
    /// Attitude of P relative to M at the start of the maneuver.
    dcm_p0m: Matrix3<f64>,
}

impl PrescribedRot2Dof {
    /// Creates the module with unconnected ports; `init` places it at
    /// `config.initial_sigma_pm`.
    pub fn new(config: PrescribedRot2DofConfig) -> Self {
        Self {
            config,
            spinning_body_ref1_in_msg: Input::default(),
            spinning_body_ref2_in_msg: Input::default(),
            prescribed_rotation_out_msg: Output::default(),
            sigma_pm: Vector3::zeros(),
            is_maneuver_complete: false,
            maneuver_start_time_s: 0.0,
            maneuver_switch_time_s: 0.0,
            maneuver_end_time_s: 0.0,
            rot_axis_m: Vector3::zeros(),
            phi_ref_rad: 0.0,
            phi_rad: 0.0,
            phi_ref_accum_rad: 0.0,
            phi_accum_rad: 0.0,
            a: 0.0,
            b: 0.0,
            dcm_p0m: Matrix3::zeros(),
        }
    }

    /// Current PRV angle swept in this maneuver (upstream `phi`) \[rad\].
    pub fn phi_rad(&self) -> f64 {
        self.phi_rad
    }

    /// PRV angle accumulated over all maneuvers (upstream `phiAccum`) \[rad\].
    pub fn phi_accum_rad(&self) -> f64 {
        self.phi_accum_rad
    }

    fn reset_at(&mut self, call_time_nanos: u64) {
        if !self.spinning_body_ref1_in_msg.is_connected() {
            log::error!("prescribedRot2DOF.spinning_body_ref1_in_msg is not connected");
        }
        if !self.spinning_body_ref2_in_msg.is_connected() {
            log::error!("prescribedRot2DOF.spinning_body_ref2_in_msg is not connected");
        }
        // Upstream quirk: only a negative value is flagged, so the unset
        // (zero) default passes and gives an infinitely long maneuver.
        if self.config.phi_ddot_max_radps2 < 0.0 {
            log::error!("prescribedRot2DOF.phi_ddot_max_radps2 wasn't set");
        }
        if self.config.rot_axis1_m.norm() < 1e-6 {
            log::error!("prescribedRot2DOF.rot_axis1_m wasn't set");
        }
        if self.config.rot_axis2_p1.norm() < 1e-6 {
            log::error!("prescribedRot2DOF.rot_axis2_p1 wasn't set");
        }

        self.maneuver_start_time_s = call_time_nanos as f64 * NANO2SEC;
        // Marked complete so that the first update starts a maneuver.
        self.is_maneuver_complete = true;
        self.phi_ref_rad = 0.0;
        self.phi_ref_accum_rad = 0.0;
        self.phi_accum_rad = 0.0;
    }

    /// Plans a maneuver from the current attitude to the attitude reached by
    /// rotating `theta1_ref` about axis 1 and then `theta2_ref` about axis 2.
    fn start_maneuver(&mut self, t_s: f64, theta1_ref_rad: f64, theta2_ref_rad: f64) {
        self.maneuver_start_time_s = t_s;
        let dcm_pm = mrp_to_dcm(self.sigma_pm);
        self.dcm_p0m = dcm_pm;

        self.config.rot_axis1_m = normalize_or_zero(self.config.rot_axis1_m);
        self.config.rot_axis2_p1 = normalize_or_zero(self.config.rot_axis2_p1);
        let dcm_p1m = prv_to_dcm(theta1_ref_rad * self.config.rot_axis1_m);
        let dcm_p2p1 = prv_to_dcm(theta2_ref_rad * self.config.rot_axis2_p1);
        let dcm_p2m = dcm_p2p1 * dcm_p1m;

        // Single principal rotation from the current attitude to the reference.
        let prv_p2p = dcm_to_prv(dcm_p2m * dcm_pm.transpose());
        self.rot_axis_m = normalize_or_zero(prv_p2p);
        self.phi_ref_rad = prv_p2p.dot(&self.rot_axis_m);
        self.phi_ref_accum_rad = self.phi_accum_rad;

        let conv_time_s = (self.phi_ref_rad.abs() * 4.0 / self.config.phi_ddot_max_radps2).sqrt();
        self.maneuver_end_time_s = self.maneuver_start_time_s + conv_time_s;
        self.maneuver_switch_time_s = conv_time_s / 2.0 + self.maneuver_start_time_s;

        // NaN for a zero-length maneuver, as upstream; the profile never uses
        // them then because the end time equals the start time.
        let first_half_s = self.maneuver_switch_time_s - self.maneuver_start_time_s;
        let second_half_s = self.maneuver_switch_time_s - self.maneuver_end_time_s;
        self.a = 0.5 * self.phi_ref_rad / (first_half_s * first_half_s);
        self.b = -0.5 * self.phi_ref_rad / (second_half_s * second_half_s);
        self.is_maneuver_complete = false;
    }

    /// Profile at time `t_s`; marks the maneuver complete once it has ended.
    fn profile_at(&mut self, t_s: f64) -> ProfileSample {
        let start = self.maneuver_start_time_s;
        let switch = self.maneuver_switch_time_s;
        let end = self.maneuver_end_time_s;
        let phi_ddot_max = self.config.phi_ddot_max_radps2;

        if t_s <= switch && end != start {
            ProfileSample {
                phi_rad: self.a * (t_s - start) * (t_s - start),
                phi_dot_radps: phi_ddot_max * (t_s - start),
                phi_ddot_radps2: phi_ddot_max,
            }
        } else if t_s > switch && t_s <= end && end != start {
            ProfileSample {
                phi_rad: self.b * (t_s - end) * (t_s - end) + self.phi_ref_rad,
                phi_dot_radps: -phi_ddot_max * (t_s - end),
                phi_ddot_radps2: -phi_ddot_max,
            }
        } else {
            self.is_maneuver_complete = true;
            ProfileSample {
                phi_rad: self.phi_ref_rad,
                phi_dot_radps: 0.0,
                phi_ddot_radps2: 0.0,
            }
        }
    }

    fn compute_rotation(&mut self, call_time_nanos: u64) -> PrescribedRotationMsg {
        let reference1 = self.spinning_body_ref1_in_msg.read();
        let reference2 = self.spinning_body_ref2_in_msg.read();
        let t_s = call_time_nanos as f64 * NANO2SEC;

        // Upstream also requires one reference message to have been written at
        // or before this call; messages here carry no write time and are never
        // written in the future, so that clause always holds.
        if self.is_maneuver_complete {
            self.start_maneuver(t_s, reference1.theta_rad, reference2.theta_rad);
        }

        let sample = self.profile_at(t_s);
        self.phi_rad = sample.phi_rad;
        self.phi_accum_rad = self.phi_ref_accum_rad + self.phi_rad;

        // Upstream renormalizes the stored axis on every update.
        self.rot_axis_m = normalize_or_zero(self.rot_axis_m);
        let dcm_pp0 = prv_to_dcm(self.phi_rad * self.rot_axis_m);
        self.sigma_pm = dcm_to_mrp(dcm_pp0 * self.dcm_p0m);

        PrescribedRotationMsg {
            omega_pm_p_radps: sample.phi_dot_radps * self.rot_axis_m,
            omega_prime_pm_p_radps2: sample.phi_ddot_radps2 * self.rot_axis_m,
            sigma_pm: self.sigma_pm,
        }
    }
}

impl Module for PrescribedRot2Dof {
    fn init(&mut self) {
        self.prescribed_rotation_out_msg
            .write(PrescribedRotationMsg::default());
        // Read here rather than in `new` so that a value set on `config`
        // before initialization takes effect; upstream Reset never touches
        // sigma_PM.
        self.sigma_pm = self.config.initial_sigma_pm;
        self.reset_at(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.reset_at(context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        let rotation = self.compute_rotation(context.current_sim_nanos);
        self.prescribed_rotation_out_msg.write(rotation);
    }
}

#[cfg(test)]
mod tests {
    // Not ported: the pytest's matplotlib plots of omega_PM_P, phi and
    // phiAccum (the logged phi/phiAccum feed only those plots).

    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{HingedRigidBodyMsg, Output, PrescribedRotationMsg};
    use crate::utilities::rigid_body_kinematics::{add_prv, prv_to_mrp, sub_prv};
    use crate::{Module, SimulationContext};

    use super::{PrescribedRot2Dof, PrescribedRot2DofConfig};

    const TASK_RATE_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn reference(theta_rad: f64) -> HingedRigidBodyMsg {
        HingedRigidBodyMsg {
            theta_rad,
            theta_dot_radps: 0.0,
        }
    }

    /// Upstream `macros.sec2nano`.
    fn sec2nano(time_s: f64) -> u64 {
        (time_s * 1e9 + 0.5) as u64
    }

    /// Runs the task ticks after `next_tick` up to and including `stop_nanos`,
    /// returning the last output and the next tick to run.
    fn run_until(
        module: &mut PrescribedRot2Dof,
        mut next_tick: u64,
        stop_nanos: u64,
    ) -> (PrescribedRotationMsg, u64) {
        let mut last = module.prescribed_rotation_out_msg.read();
        while next_tick <= stop_nanos {
            module.update(&context(next_tick));
            last = module.prescribed_rotation_out_msg.read();
            next_tick += TASK_RATE_NANOS;
        }
        (last, next_tick)
    }

    /// Port of `test_PrescribedRot2DOFTestFunction`, every parametrization
    /// (`thetaInit` only feeds the pytest's plots): 0.5 s task rate, two
    /// consecutive maneuvers, the second commanded by rewriting the reference
    /// messages.
    #[test]
    fn matches_upstream_prescribed_rot_2dof_unit_test() {
        let accuracy = 1e-5;
        let phi_ddot_max = 0.004;
        let rot_axis1_m = Vector3::new(0.0, 1.0, 0.0);
        let rot_axis2_p1 = Vector3::new(0.0, 0.0, 1.0);
        let theta_dot_ref = 0.0;

        for theta_ref1a in [0.0, 2.0 * PI / 3.0] {
            for theta_ref2a in [PI / 3.0, 2.0 * PI / 3.0] {
                for theta_ref1b in [0.0, 2.0 * PI / 3.0] {
                    for theta_ref2b in [PI / 3.0, 2.0 * PI / 3.0] {
                        let ref1 = Output::new(reference(theta_ref1a));
                        let ref2 = Output::new(reference(theta_ref2a));
                        let mut module = PrescribedRot2Dof::new(PrescribedRot2DofConfig {
                            phi_ddot_max_radps2: phi_ddot_max,
                            rot_axis1_m,
                            rot_axis2_p1,
                            initial_sigma_pm: Vector3::zeros(),
                            ..Default::default()
                        });
                        ref1.connect_to(&mut module.spinning_body_ref1_in_msg);
                        ref2.connect_to(&mut module.spinning_body_ref2_in_msg);
                        module.init();

                        // First rotation truth.
                        let prv_p0m_a = theta_ref1a * rot_axis1_m;
                        let prv_p1p0_a = theta_ref2a * rot_axis2_p1;
                        let (prv_p1m_a, phi_p1m_a, sigma_pm_ref1) =
                            if theta_ref1a == 0.0 && theta_ref2a == 0.0 {
                                (Vector3::zeros(), 0.0, Vector3::zeros())
                            } else {
                                let prv = add_prv(prv_p0m_a, prv_p1p0_a);
                                (prv, prv.norm(), prv_to_mrp(prv))
                            };
                        let sim_time1 =
                            ((0.5 * phi_p1m_a.abs()) * 8.0 / phi_ddot_max).sqrt() + 10.0;
                        let (first, next_tick) = run_until(&mut module, 0, sec2nano(sim_time1));

                        // Second rotation truth.
                        let prv_p2m_b = theta_ref1b * rot_axis1_m;
                        let prv_p3p2_b = theta_ref2b * rot_axis2_p1;
                        let prv_p3m_b = if theta_ref1b == 0.0 && theta_ref2b == 0.0 {
                            Vector3::zeros()
                        } else {
                            add_prv(prv_p2m_b, prv_p3p2_b)
                        };
                        let arrays_equal =
                            (0..3).all(|i| (prv_p1m_a[i] - prv_p3m_b[i]).abs() <= 1e-12);
                        let (prv_p3p1_b, sigma_pm_ref2) = if arrays_equal {
                            (Vector3::zeros(), sigma_pm_ref1)
                        } else {
                            (sub_prv(prv_p1m_a, prv_p3m_b), prv_to_mrp(prv_p3m_b))
                        };
                        let phi_p3p1_b = prv_p3p1_b.norm();

                        ref1.write(reference(theta_ref1b));
                        ref2.write(reference(theta_ref2b));
                        let sim_time2 =
                            ((0.5 * phi_p3p1_b.abs()) * 8.0 / phi_ddot_max).sqrt() + 10.0;
                        let (second, _) =
                            run_until(&mut module, next_tick, sec2nano(sim_time1 + sim_time2));

                        let label = format!(
                            "thetaRef1a={theta_ref1a}, thetaRef2a={theta_ref2a}, \
                             thetaRef1b={theta_ref1b}, thetaRef2b={theta_ref2b}"
                        );
                        let theta_dot_final = second.omega_pm_p_radps.norm();
                        assert!(
                            (theta_dot_final - theta_dot_ref).abs() <= accuracy,
                            "{label}: thetaDot_Final {theta_dot_final}"
                        );
                        for i in 0..3 {
                            assert!(
                                (first.sigma_pm[i] - sigma_pm_ref1[i]).abs() <= accuracy,
                                "{label}: sigma_PM_Final1 {:?} vs {:?}",
                                first.sigma_pm,
                                sigma_pm_ref1
                            );
                            assert!(
                                (second.sigma_pm[i] - sigma_pm_ref2[i]).abs() <= accuracy,
                                "{label}: sigma_PM_Final2 {:?} vs {:?}",
                                second.sigma_pm,
                                sigma_pm_ref2
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn follows_the_bang_bang_profile() {
        // One rotation of 0.5 rad about M's z axis at 0.02 rad/s^2: the
        // maneuver lasts 2 sqrt(0.5 / 0.02) = 10 s with the switch at 5 s.
        let ref1 = Output::new(reference(0.0));
        let ref2 = Output::new(reference(0.5));
        let mut module = PrescribedRot2Dof::new(PrescribedRot2DofConfig {
            phi_ddot_max_radps2: 0.02,
            rot_axis1_m: Vector3::new(0.0, 2.0, 0.0),
            rot_axis2_p1: Vector3::new(0.0, 0.0, 3.0),
            ..Default::default()
        });
        ref1.connect_to(&mut module.spinning_body_ref1_in_msg);
        ref2.connect_to(&mut module.spinning_body_ref2_in_msg);
        module.init();

        module.update(&context(0));
        assert_eq!(module.config.rot_axis1_m, Vector3::y());
        assert_eq!(module.config.rot_axis2_p1, Vector3::z());

        module.update(&context(2_000_000_000));
        let output = module.prescribed_rotation_out_msg.read();
        assert!((module.phi_rad() - 0.5 * 0.02 * 4.0).abs() <= 1e-12);
        assert!((output.omega_pm_p_radps - Vector3::new(0.0, 0.0, 0.04)).amax() <= 1e-12);
        assert!((output.omega_prime_pm_p_radps2 - Vector3::new(0.0, 0.0, 0.02)).amax() <= 1e-12);

        module.update(&context(8_000_000_000));
        let output = module.prescribed_rotation_out_msg.read();
        assert!((module.phi_rad() - (0.5 - 0.5 * 0.02 * 4.0)).abs() <= 1e-12);
        assert!((output.omega_pm_p_radps - Vector3::new(0.0, 0.0, 0.04)).amax() <= 1e-12);
        assert!((output.omega_prime_pm_p_radps2 + Vector3::new(0.0, 0.0, 0.02)).amax() <= 1e-12);

        module.update(&context(11_000_000_000));
        let output = module.prescribed_rotation_out_msg.read();
        assert!((module.phi_rad() - 0.5).abs() <= 1e-12);
        assert_eq!(output.omega_pm_p_radps, Vector3::zeros());
        assert_eq!(output.omega_prime_pm_p_radps2, Vector3::zeros());
        let expected_sigma = Vector3::new(0.0, 0.0, (0.5_f64 / 4.0).tan());
        assert!((output.sigma_pm - expected_sigma).amax() <= 1e-12);
        assert!((module.phi_accum_rad() - 0.5).abs() <= 1e-12);
    }

    #[test]
    fn reset_restarts_the_angle_bookkeeping_but_keeps_the_attitude() {
        let ref1 = Output::new(reference(0.0));
        let ref2 = Output::new(reference(0.5));
        let mut module = PrescribedRot2Dof::new(PrescribedRot2DofConfig {
            phi_ddot_max_radps2: 0.02,
            rot_axis1_m: Vector3::y(),
            rot_axis2_p1: Vector3::z(),
            ..Default::default()
        });
        ref1.connect_to(&mut module.spinning_body_ref1_in_msg);
        ref2.connect_to(&mut module.spinning_body_ref2_in_msg);
        module.init();
        module.update(&context(0));
        module.update(&context(2_000_000_000));
        let sigma_mid = module.prescribed_rotation_out_msg.read().sigma_pm;

        // After a reset the next update plans a new maneuver from the
        // attitude reached so far, not from the initial attitude.
        Module::reset(&mut module, &context(2_000_000_000));
        assert_eq!(module.phi_accum_rad(), 0.0);
        module.update(&context(2_000_000_000));
        let output = module.prescribed_rotation_out_msg.read();
        assert!((output.sigma_pm - sigma_mid).amax() <= 1e-15);
        assert_eq!(output.omega_pm_p_radps, Vector3::zeros());
    }

    #[test]
    fn initial_attitude_set_after_construction_takes_effect() {
        // Start 0.3 rad about z and command 0.5 rad about z: the maneuver
        // sweeps 0.2 rad and lasts 2 sqrt(0.2 / 0.02) ≈ 6.3 s.
        let ref1 = Output::new(reference(0.0));
        let ref2 = Output::new(reference(0.5));
        let mut module = PrescribedRot2Dof::new(PrescribedRot2DofConfig::default());
        module.config.phi_ddot_max_radps2 = 0.02;
        module.config.rot_axis1_m = Vector3::y();
        module.config.rot_axis2_p1 = Vector3::z();
        module.config.initial_sigma_pm = Vector3::new(0.0, 0.0, (0.3_f64 / 4.0).tan());
        ref1.connect_to(&mut module.spinning_body_ref1_in_msg);
        ref2.connect_to(&mut module.spinning_body_ref2_in_msg);
        module.init();

        module.update(&context(0));
        let output = module.prescribed_rotation_out_msg.read();
        assert!((output.sigma_pm - module.config.initial_sigma_pm).amax() <= 1e-15);

        module.update(&context(7_000_000_000));
        let output = module.prescribed_rotation_out_msg.read();
        assert!((module.phi_rad() - 0.2).abs() <= 1e-12);
        let expected_sigma = Vector3::new(0.0, 0.0, (0.5_f64 / 4.0).tan());
        assert!((output.sigma_pm - expected_sigma).amax() <= 1e-12);
    }
}
