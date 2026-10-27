//! Body-frame thruster configuration of a gimballed thruster platform: port of
//! upstream Basilisk `thrusterPlatformState`
//! (`fswAlgorithms/effectorInterfaces/thrusterPlatformState`).
//!
//! Given the measured tip and tilt angles of the platform (frame F, attached
//! to the hub through the mount frame M) and the thruster configuration in
//! platform coordinates, the module publishes the thrust direction and
//! application point in body coordinates. See also
//! [`super::thruster_platform_reference`], which computes the reference
//! angles for the same platform.

use nalgebra::Vector3;

use crate::messages::{HingedRigidBodyMsg, Input, Output, ThrusterConfigMsg};
use crate::utilities::rigid_body_kinematics::{euler123_to_dcm, mrp_to_dcm};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`ThrusterPlatformState`]. Defaults are
/// upstream's zero initialization.
#[derive(Clone, Debug)]
pub struct ThrusterPlatformStateConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `sigma_MB` [-]: MRP attitude of the mount frame M relative to
    /// the body frame B.
    pub sigma_mb: Vector3<f64>,
    /// Upstream `r_BM_M` \[m\]: position of the B origin relative to the M
    /// origin, M components.
    pub r_bm_m_m: Vector3<f64>,
    /// Upstream `r_FM_F` \[m\]: position of the F origin relative to the M
    /// origin, F components.
    pub r_fm_f_m: Vector3<f64>,
    /// Upstream `K` \[1/s\]: declared by upstream ("momentum dumping time
    /// constant") but not used by the algorithm.
    pub k: f64,
}

impl Default for ThrusterPlatformStateConfig {
    fn default() -> Self {
        Self {
            name: "thrusterPlatformState".to_string(),
            sigma_mb: Vector3::zeros(),
            r_bm_m_m: Vector3::zeros(),
            r_fm_f_m: Vector3::zeros(),
            k: 0.0,
        }
    }
}

/// Thruster-platform state to body-frame thruster configuration (upstream
/// `thrusterPlatformStateConfig`).
#[derive(Clone, Debug)]
pub struct ThrusterPlatformState {
    /// Module configuration, read at every update.
    pub config: ThrusterPlatformStateConfig,
    /// Upstream `thrusterConfigFInMsg`: thruster in platform (F) coordinates;
    /// its `position_thrust_b_m` is `r_TF_F`.
    pub thruster_config_f_in_msg: Input<ThrusterConfigMsg>,
    /// Upstream `hingedRigidBody1InMsg`: measured tip angle.
    pub hinged_rigid_body1_in_msg: Input<HingedRigidBodyMsg>,
    /// Upstream `hingedRigidBody2InMsg`: measured tilt angle.
    pub hinged_rigid_body2_in_msg: Input<HingedRigidBodyMsg>,
    /// Upstream `thrusterConfigBOutMsg`: thruster in body (B) coordinates; its
    /// `position_thrust_b_m` is `r_TB_B`.
    pub thruster_config_b_out_msg: Output<ThrusterConfigMsg>,
}

impl ThrusterPlatformState {
    /// Builds the module with unconnected inputs.
    pub fn new(config: ThrusterPlatformStateConfig) -> Self {
        Self {
            config,
            thruster_config_f_in_msg: Input::default(),
            hinged_rigid_body1_in_msg: Input::default(),
            hinged_rigid_body2_in_msg: Input::default(),
            thruster_config_b_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_thrusterPlatformState`: check the inputs and zero the
    /// output.
    pub fn reset(&mut self, _call_time_nanos: u64) {
        if !self.thruster_config_f_in_msg.is_connected() {
            log::error!("thrusterPlatformState.thruster_config_f_in_msg is not connected");
        }
        if !self.hinged_rigid_body1_in_msg.is_connected() {
            log::error!("thrusterPlatformState.hinged_rigid_body1_in_msg is not connected");
        }
        if !self.hinged_rigid_body2_in_msg.is_connected() {
            log::error!("thrusterPlatformState.hinged_rigid_body2_in_msg is not connected");
        }
        self.thruster_config_b_out_msg
            .write(ThrusterConfigMsg::default());
    }

    /// The thruster configuration in body coordinates for the measured
    /// platform angles.
    fn thruster_config_b(&self) -> ThrusterConfigMsg {
        let thruster_f = self.thruster_config_f_in_msg.read();
        let theta1_rad = self.hinged_rigid_body1_in_msg.read().theta_rad;
        let theta2_rad = self.hinged_rigid_body2_in_msg.read().theta_rad;

        let dcm_mb = mrp_to_dcm(self.config.sigma_mb);
        let r_tm_f = self.config.r_fm_f_m + thruster_f.position_thrust_b_m;
        let dcm_fm = euler123_to_dcm(Vector3::new(theta1_rad, theta2_rad, 0.0));
        let dcm_fb = dcm_fm * dcm_mb;

        let r_tm_b = dcm_fb.tr_mul(&r_tm_f);
        let r_bm_b = dcm_mb.tr_mul(&self.config.r_bm_m_m);
        ThrusterConfigMsg {
            position_thrust_b_m: r_tm_b - r_bm_b,
            t_hat_thrust_b: dcm_fb.tr_mul(&thruster_f.t_hat_thrust_b),
            max_thrust_n: thruster_f.max_thrust_n,
            swirl_torque_nm: thruster_f.swirl_torque_nm,
        }
    }
}

impl Module for ThrusterPlatformState {
    fn init(&mut self) {
        self.thruster_config_b_out_msg
            .write(ThrusterConfigMsg::default());
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        ThrusterPlatformState::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, _context: &SimulationContext) {
        let thruster_b = self.thruster_config_b();
        self.thruster_config_b_out_msg.write(thruster_b);
    }
}

#[cfg(test)]
mod tests {
    //! Port of `test_thrusterPlatformState.py`: `test_platformRotation` (all
    //! nine `theta1` x `theta2` parametrizations) and `test_reset`.

    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{HingedRigidBodyMsg, Output, ThrusterConfigMsg};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::rigid_body_kinematics::{euler123_to_dcm, mrp_to_dcm};
    use crate::{Module, SimulationContext};

    use super::{ThrusterPlatformState, ThrusterPlatformStateConfig};

    const R_BM_M: Vector3<f64> = Vector3::new(0.0, 0.1, 1.4);
    const R_FM_F: Vector3<f64> = Vector3::new(0.0, 0.0, -0.1);

    fn hinged(theta_rad: f64) -> HingedRigidBodyMsg {
        HingedRigidBodyMsg {
            theta_rad,
            theta_dot_radps: 0.0,
        }
    }

    fn platform_config() -> ThrusterPlatformStateConfig {
        ThrusterPlatformStateConfig {
            sigma_mb: Vector3::zeros(),
            r_bm_m_m: R_BM_M,
            r_fm_f_m: R_FM_F,
            ..Default::default()
        }
    }

    fn assert_close(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64, what: &str) {
        assert!(
            (actual - expected).amax() <= accuracy,
            "{what}: expected {expected:?}, got {actual:?}"
        );
    }

    #[test]
    fn matches_upstream_platform_rotation_unit_test() {
        let accuracy = 1e-10;
        let r_tf_f = Vector3::new(-0.01, 0.03, 0.02);
        let t_f = Vector3::new(1.0, 1.0, 10.0);
        let swirl_factor = 0.1;
        let sigma_mb = Vector3::zeros();

        for theta1 in [0.0, PI / 36.0, PI / 18.0] {
            for theta2 in [0.0, PI / 36.0, PI / 18.0] {
                let case = format!("theta1={theta1}, theta2={theta2}");
                let max_thrust_n = t_f.norm();
                let thruster_f = Output::new(ThrusterConfigMsg {
                    position_thrust_b_m: r_tf_f,
                    t_hat_thrust_b: t_f / max_thrust_n,
                    max_thrust_n,
                    swirl_torque_nm: max_thrust_n * swirl_factor,
                });
                let hinged1 = Output::new(hinged(theta1));
                let hinged2 = Output::new(hinged(theta2));
                let mut module = ThrusterPlatformState::new(platform_config());
                let mut recorder = MessageRecorder::default();
                let mut simulation =
                    Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
                simulation.connect(&thruster_f, &mut module.thruster_config_f_in_msg);
                simulation.connect(&hinged1, &mut module.hinged_rigid_body1_in_msg);
                simulation.connect(&hinged2, &mut module.hinged_rigid_body2_in_msg);
                simulation.connect(&module.thruster_config_b_out_msg, &mut recorder.input_msg);
                simulation.add_module("platformState", &mut module, 1_000_000_000, 0);
                simulation.add_module("recorder", &mut recorder, 1_000_000_000, 0);
                simulation.run_for(500_000_000);

                let output = &recorder.samples[0];
                let dcm_fm = euler123_to_dcm(Vector3::new(theta1, theta2, 0.0));
                let dcm_mb = mrp_to_dcm(sigma_mb);
                let dcm_fb = dcm_fm * dcm_mb;
                let r_tb_b = dcm_fb.transpose() * (r_tf_f + R_FM_F - dcm_fm * R_BM_M);
                let t_hat_b = dcm_fb.transpose() * t_f / t_f.norm();

                assert_close(
                    output.position_thrust_b_m,
                    r_tb_b,
                    accuracy,
                    &format!("{case}: rThrust_B"),
                );
                assert_close(
                    output.t_hat_thrust_b,
                    t_hat_b,
                    accuracy,
                    &format!("{case}: tHatThrust_B"),
                );
                assert!(
                    (output.max_thrust_n - t_f.norm()).abs() <= accuracy,
                    "{case}: maxThrust"
                );
                assert!(
                    (output.swirl_torque_nm - t_f.norm() * swirl_factor).abs() <= accuracy,
                    "{case}: swirlTorque"
                );
            }
        }
    }

    #[test]
    fn matches_upstream_reset_test() {
        let thruster_f = Output::new(ThrusterConfigMsg {
            position_thrust_b_m: Vector3::new(1.0, 2.0, 3.0),
            t_hat_thrust_b: Vector3::new(0.1, 0.2, 0.3),
            max_thrust_n: 10.0,
            swirl_torque_nm: 1.0,
        });
        let hinged1 = Output::new(hinged(PI / 36.0));
        let hinged2 = Output::new(hinged(PI / 36.0));
        let mut module = ThrusterPlatformState::new(platform_config());
        thruster_f.connect_to(&mut module.thruster_config_f_in_msg);
        hinged1.connect_to(&mut module.hinged_rigid_body1_in_msg);
        hinged2.connect_to(&mut module.hinged_rigid_body2_in_msg);
        module.init();
        module.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });
        assert_ne!(
            module.thruster_config_b_out_msg.read(),
            ThrusterConfigMsg::default()
        );

        module.reset(0);
        let output = module.thruster_config_b_out_msg.read();
        let accuracy = 1e-12;
        assert!(output.position_thrust_b_m.amax() <= accuracy);
        assert!(output.t_hat_thrust_b.amax() <= accuracy);
        assert!(output.max_thrust_n.abs() <= accuracy);
        assert!(output.swirl_torque_nm.abs() <= accuracy);
    }

    #[test]
    fn accounts_for_the_mount_frame_attitude() {
        // M is a quarter turn about the body z axis and the platform is at
        // rest, so F-frame vectors appear turned by +90 deg about z in B.
        let sigma_mb = Vector3::new(0.0, 0.0, (PI / 8.0).tan());
        let thruster_f = Output::new(ThrusterConfigMsg {
            position_thrust_b_m: Vector3::new(1.0, 0.0, 0.0),
            t_hat_thrust_b: Vector3::new(1.0, 0.0, 0.0),
            max_thrust_n: 2.0,
            swirl_torque_nm: 0.0,
        });
        let hinged1 = Output::new(hinged(0.0));
        let hinged2 = Output::new(hinged(0.0));
        let mut module = ThrusterPlatformState::new(ThrusterPlatformStateConfig {
            sigma_mb,
            r_bm_m_m: Vector3::new(0.0, 0.0, 1.0),
            r_fm_f_m: Vector3::zeros(),
            ..Default::default()
        });
        thruster_f.connect_to(&mut module.thruster_config_f_in_msg);
        hinged1.connect_to(&mut module.hinged_rigid_body1_in_msg);
        hinged2.connect_to(&mut module.hinged_rigid_body2_in_msg);
        module.init();
        module.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });

        // The M (and F) x axis is the body y axis; r_TB_B = r_TM_B - r_BM_B.
        let output = module.thruster_config_b_out_msg.read();
        assert_close(output.t_hat_thrust_b, Vector3::y(), 1e-15, "tHatThrust_B");
        assert_close(
            output.position_thrust_b_m,
            Vector3::new(0.0, 1.0, -1.0),
            1e-15,
            "rThrust_B",
        );
    }
}
