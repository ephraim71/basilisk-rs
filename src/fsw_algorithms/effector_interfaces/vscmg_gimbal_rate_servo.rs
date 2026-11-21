//! VSCMG gimbal-rate servo: port of upstream Basilisk
//! `fswAlgorithms/effectorInterfaces/vscmgGimbalRateServo`.
//!
//! Maps the desired wheel accelerations `Ω̇_d` and gimbal rates `γ̇_d` of each
//! variable-speed control moment gyroscope to wheel and gimbal motor torques
//! (Schaub & Junkins, *Analytical Mechanics of Space Systems*, section
//! 8.8.3). With `ω_s` and `ω_t` the body rate along the current spin and
//! transverse axes,
//!
//! ```text
//! u_s = I_ws (Ω̇_d + γ̇ ω_t)
//! u_g = J_g γ̈ - (J_s - J_t) ω_s ω_t - I_ws Ω ω_t,    γ̈ = -K_γ̇ (γ̇ - γ̇_d)
//! ```
//!
//! The gimbal law is a Lyapunov servo on the gimbal-rate error that drops
//! the `γ̈_d` feedforward; the wheel law is open loop, leaving wheel-speed
//! tracking to the outer attitude loop. Both drop the body angular
//! acceleration terms, small next to `Ω̇` and `γ̈`.
//!
//! The servo publishes one [`VscmgCommandMsg`] per VSCMG, registered with
//! [`VscmgGimbalRateServo::add_vscmg_command_output`], where upstream writes
//! one `VSCMGArrayTorqueMsg` (and its C-wrapped copy) for the cluster.
//!
//! **Header stand-in:** upstream falls back to the configured initial wheel
//! speeds, gimbal angles and gimbal rates while `speedsInMsg` has never been
//! written. Ports carry no header, so here the speeds input counts as written
//! when it is connected.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    Input, MAX_EFF_COUNT, NavigationAttitudeMsg, Output, VscmgArrayConfigMsg, VscmgCommandMsg,
    VscmgConfigElementMsg, VscmgReferenceStatesMsg, VscmgSpeedMsg,
};
use crate::utilities::rigid_body_kinematics::euler3;
use crate::{Module, SimulationContext};

/// Wheel speed \[rad/s\], gimbal angle \[rad\] and gimbal rate \[rad/s\] of one
/// VSCMG.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct VscmgState {
    pub(crate) wheel_speed_radps: f64,
    pub(crate) gimbal_angle_rad: f64,
    pub(crate) gimbal_rate_radps: f64,
}

/// The current state of each configured VSCMG: from the speeds input when it
/// is connected, the stand-in for upstream's `speedsInMsg.isWritten()`;
/// otherwise the configured initial states, as upstream before the first
/// speeds message.
pub(crate) fn vscmg_states(
    speeds_in_msg: &Input<VscmgSpeedMsg>,
    vscmgs: &[VscmgConfigElementMsg],
) -> Vec<VscmgState> {
    if speeds_in_msg.is_connected() {
        let speeds = speeds_in_msg.read();
        (0..vscmgs.len())
            .map(|i| VscmgState {
                wheel_speed_radps: speeds.wheel_speeds_radps[i],
                gimbal_angle_rad: speeds.gimbal_angles_rad[i],
                gimbal_rate_radps: speeds.gimbal_rates_radps[i],
            })
            .collect()
    } else {
        vscmgs
            .iter()
            .map(|vscmg| VscmgState {
                wheel_speed_radps: vscmg.wheel_speed0_radps,
                gimbal_angle_rad: vscmg.gimbal_angle0_rad,
                gimbal_rate_radps: vscmg.gimbal_rate0_radps,
            })
            .collect()
    }
}

/// DCM `[BG]` from the gimbal frame at `gimbal_angle_rad` to the body
/// frame: its columns are the current spin, transverse and gimbal axes,
/// `[BG] = [BG0] [M3(γ)]^T` (upstream `BG0 * eigenM3(gamma).transpose()`).
pub(crate) fn gimbal_to_body(vscmg: &VscmgConfigElementMsg, gimbal_angle_rad: f64) -> Matrix3<f64> {
    let bg0 = Matrix3::from_columns(&[vscmg.gs_hat0_b, vscmg.gt_hat0_b, vscmg.gg_hat_b]);
    bg0 * euler3(gimbal_angle_rad).transpose()
}

/// The VSCMGs in use of a cluster configuration, logging an error for more
/// than [`MAX_EFF_COUNT`] (upstream then overruns its arrays; the extra ones
/// are ignored here).
pub(crate) fn active_vscmgs(
    config: &VscmgArrayConfigMsg,
    module: &str,
) -> Vec<VscmgConfigElementMsg> {
    if config.num_vscmg > MAX_EFF_COUNT {
        log::error!("{module}: num_vscmg is larger than MAX_EFF_COUNT ({MAX_EFF_COUNT})");
    }
    config.active_vscmgs().to_vec()
}

/// User-settable parameters of [`VscmgGimbalRateServo`].
#[derive(Clone, Debug)]
pub struct VscmgGimbalRateServoConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `K_gammaDot` \[1/s\]: gimbal-rate error gain (default 0).
    pub k_gamma_dot_per_s: f64,
}

impl Default for VscmgGimbalRateServoConfig {
    fn default() -> Self {
        Self {
            name: "vscmgGimbalRateServo".to_string(),
            k_gamma_dot_per_s: 0.0,
        }
    }
}

/// Desired VSCMG wheel accelerations and gimbal rates to motor torques:
/// upstream `VscmgGimbalRateServo`.
#[derive(Clone, Debug)]
pub struct VscmgGimbalRateServo {
    /// Module configuration.
    pub config: VscmgGimbalRateServoConfig,
    /// Upstream `vsmcgParamsInMsg` (sic): VSCMG cluster configuration, read
    /// at reset.
    pub vscmg_params_in_msg: Input<VscmgArrayConfigMsg>,
    /// Upstream `vscmgRefStatesInMsg`: desired wheel accelerations and gimbal
    /// rates.
    pub vscmg_ref_states_in_msg: Input<VscmgReferenceStatesMsg>,
    /// Upstream `attInMsg`: body rate `ω_BN`.
    pub att_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `speedsInMsg`: current wheel speeds, gimbal angles and rates.
    pub speeds_in_msg: Input<VscmgSpeedMsg>,
    /// Upstream `cmdsOutMsg`: wheel and gimbal motor torques, one output per
    /// VSCMG in cluster order, registered with
    /// [`VscmgGimbalRateServo::add_vscmg_command_output`].
    pub cmds_out_msgs: Vec<Output<VscmgCommandMsg>>,
    /// Upstream `vscmgConfigParams`: the VSCMGs in use, read at reset.
    vscmgs: Vec<VscmgConfigElementMsg>,
}

impl VscmgGimbalRateServo {
    /// Builds the module with unconnected ports and no VSCMGs; the cluster
    /// is read at reset.
    pub fn new(config: VscmgGimbalRateServoConfig) -> Self {
        Self {
            config,
            vscmg_params_in_msg: Input::default(),
            vscmg_ref_states_in_msg: Input::default(),
            att_in_msg: Input::default(),
            speeds_in_msg: Input::default(),
            cmds_out_msgs: Vec::new(),
            vscmgs: Vec::new(),
        }
    }

    /// Registers one independently connectable VSCMG command output. The
    /// outputs follow the cluster order: the first one registered carries
    /// the torques of the first VSCMG.
    pub fn add_vscmg_command_output(&mut self) -> &Output<VscmgCommandMsg> {
        assert!(
            self.cmds_out_msgs.len() < MAX_EFF_COUNT,
            "at most {MAX_EFF_COUNT} VSCMG command outputs are supported"
        );
        self.cmds_out_msgs.push(Output::default());
        self.cmds_out_msgs
            .last()
            .expect("registered VSCMG command output should be available")
    }

    /// Upstream `Reset`: reports missing inputs and reads the cluster
    /// configuration.
    fn reset_module(&mut self) {
        for (connected, port) in [
            (
                self.vscmg_params_in_msg.is_connected(),
                "vscmg_params_in_msg",
            ),
            (
                self.vscmg_ref_states_in_msg.is_connected(),
                "vscmg_ref_states_in_msg",
            ),
            (self.att_in_msg.is_connected(), "att_in_msg"),
            (self.speeds_in_msg.is_connected(), "speeds_in_msg"),
        ] {
            if !connected {
                log::error!("vscmgGimbalRateServo.{port} is not connected");
            }
        }
        self.vscmgs = active_vscmgs(&self.vscmg_params_in_msg.read(), "vscmgGimbalRateServo");
        if self.cmds_out_msgs.len() < self.vscmgs.len() {
            log::error!(
                "vscmgGimbalRateServo has {} VSCMGs but only {} command outputs",
                self.vscmgs.len(),
                self.cmds_out_msgs.len()
            );
        }
    }

    /// Upstream `UpdateState`: the wheel and gimbal motor torques of each
    /// VSCMG in use.
    fn compute_commands(&self) -> Vec<VscmgCommandMsg> {
        let reference = self.vscmg_ref_states_in_msg.read();
        let omega_bn_b: Vector3<f64> = self.att_in_msg.read().omega_bn_b_radps;
        let k_gamma_dot = self.config.k_gamma_dot_per_s;
        self.vscmgs
            .iter()
            .zip(vscmg_states(&self.speeds_in_msg, &self.vscmgs))
            .enumerate()
            .map(|(i, (vscmg, state))| {
                let bg = gimbal_to_body(vscmg, state.gimbal_angle_rad);
                let gamma_ddot =
                    -k_gamma_dot * (state.gimbal_rate_radps - reference.gimbal_rates_radps[i]);
                let omega_s = bg.column(0).dot(&omega_bn_b);
                let omega_t = bg.column(1).dot(&omega_bn_b);
                VscmgCommandMsg {
                    u_s_cmd_nm: vscmg.iws_kg_m2
                        * (reference.wheel_accels_radps2[i] + state.gimbal_rate_radps * omega_t),
                    u_g_cmd_nm: vscmg.jg_kg_m2 * gamma_ddot
                        - (vscmg.js_kg_m2 - vscmg.jt_kg_m2) * omega_s * omega_t
                        - vscmg.iws_kg_m2 * state.wheel_speed_radps * omega_t,
                }
            })
            .collect()
    }
}

impl Module for VscmgGimbalRateServo {
    fn init(&mut self) {
        for output in &self.cmds_out_msgs {
            output.write(VscmgCommandMsg::default());
        }
        self.reset_module();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_module();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let commands = self.compute_commands();
        // Outputs past the VSCMGs in use get zero torques, as upstream's
        // zeroed payload.
        for (index, output) in self.cmds_out_msgs.iter().enumerate() {
            output.write(commands.get(index).copied().unwrap_or_default());
        }
    }
}

#[cfg(test)]
pub(crate) mod test_support {
    //! The VSCMG clusters of the upstream VSCMG pytests.

    use nalgebra::{Matrix3, Vector3};

    use crate::messages::VscmgConfigElementMsg;

    /// The pytests' `defaultVSCMG` with the axes, gimbal angle and inertias
    /// that `setupVSCMGs` and `fswSetupVSCMGs.create` give each VSCMG.
    #[derive(Clone, Copy, Debug)]
    pub(crate) struct UpstreamVscmg {
        pub(crate) gs_hat0_b: Vector3<f64>,
        pub(crate) gt_hat0_b: Vector3<f64>,
        pub(crate) gg_hat_b: Vector3<f64>,
        /// `Omega` \[rad/s\].
        pub(crate) wheel_speed_radps: f64,
        /// `gamma` \[rad\].
        pub(crate) gimbal_angle_rad: f64,
        /// `gammaDot` \[rad/s\].
        pub(crate) gimbal_rate_radps: f64,
        pub(crate) iw: [f64; 3],
        pub(crate) ig: [f64; 3],
    }

    impl UpstreamVscmg {
        /// `fswSetupVSCMGs.create`: `Js = IG1 + IW1`, `Jt = IG2 + IW2`,
        /// `Jg = IG3 + IW3` and `Iws = IW1`.
        pub(crate) fn config_element(&self) -> VscmgConfigElementMsg {
            VscmgConfigElementMsg {
                gs_hat0_b: self.gs_hat0_b,
                gt_hat0_b: self.gt_hat0_b,
                gg_hat_b: self.gg_hat_b,
                js_kg_m2: self.ig[0] + self.iw[0],
                jt_kg_m2: self.ig[1] + self.iw[1],
                jg_kg_m2: self.ig[2] + self.iw[2],
                iws_kg_m2: self.iw[0],
                wheel_speed0_radps: self.wheel_speed_radps,
                gimbal_angle0_rad: self.gimbal_angle_rad,
                gimbal_rate0_radps: self.gimbal_rate_radps,
            }
        }
    }

    fn default_vscmg() -> UpstreamVscmg {
        UpstreamVscmg {
            gs_hat0_b: Vector3::zeros(),
            gt_hat0_b: Vector3::zeros(),
            gg_hat_b: Vector3::zeros(),
            wheel_speed_radps: 14.4,
            gimbal_angle_rad: 0.0,
            gimbal_rate_radps: 0.0,
            iw: [0.1, 0.04, 0.03],
            ig: [0.03, 0.0, 0.0],
        }
    }

    /// The pytests' rotated pyramid VSCMG: zero-angle axes `[BG2] [GG02]`
    /// for a gimbal angle of `gamma`.
    fn rotated(gs_hat_t0: Vector3<f64>, gg_hat: Vector3<f64>, gamma: f64) -> UpstreamVscmg {
        let gt_hat_t0 = gg_hat.cross(&gs_hat_t0);
        let gt_hat_t0 = gt_hat_t0 / gt_hat_t0.norm();
        let bg = Matrix3::from_columns(&[gs_hat_t0, gt_hat_t0, gg_hat]);
        let gg0 = Matrix3::new(
            -gamma.cos(),
            gamma.sin(),
            0.0,
            -gamma.sin(),
            -gamma.cos(),
            0.0,
            0.0,
            0.0,
            1.0,
        );
        let bg0 = bg * gg0;
        UpstreamVscmg {
            gimbal_angle_rad: gamma,
            gs_hat0_b: bg0.column(0).into(),
            gt_hat0_b: bg0.column(1).into(),
            gg_hat_b: bg0.column(2).into(),
            ..default_vscmg()
        }
    }

    /// The pytests' `setupVSCMGs(numVSCMGs)` for 1, 2 or 4 VSCMGs.
    pub(crate) fn setup_vscmgs(num_vscmgs: usize) -> Vec<UpstreamVscmg> {
        let ang = 54.75 * std::f64::consts::PI / 180.0;
        let gg0 = Vector3::new(ang.cos(), 0.0, ang.sin());
        let gs0 = Vector3::new(0.0, 1.0, 0.0);
        let mut vscmgs = vec![UpstreamVscmg {
            gg_hat_b: gg0,
            gs_hat0_b: gs0,
            gt_hat0_b: gg0.cross(&gs0),
            ..default_vscmg()
        }];
        if num_vscmgs > 1 {
            let gg1 = Vector3::new(-ang.cos(), 0.0, ang.sin());
            let gs1 = Vector3::new(0.0, -1.0, 0.0);
            vscmgs.push(UpstreamVscmg {
                gs_hat0_b: gs1,
                gg_hat_b: gg1,
                gt_hat0_b: gg1.cross(&gs1),
                ..default_vscmg()
            });
            if num_vscmgs == 4 {
                vscmgs.push(rotated(
                    Vector3::new(1.0, 0.0, 0.0),
                    Vector3::new(0.0, ang.cos(), ang.sin()),
                    90.0_f64.to_radians(),
                ));
                vscmgs.push(rotated(
                    Vector3::new(-1.0, 0.0, 0.0),
                    Vector3::new(0.0, -ang.cos(), ang.sin()),
                    -(90.0_f64.to_radians()),
                ));
            }
        }
        vscmgs
    }

    /// The pytests' body-frame axes of a VSCMG at its gimbal angle:
    /// `BG = BG0 @ GG0.T` with `GG0` built element by element.
    pub(crate) fn upstream_bg(vscmg: &UpstreamVscmg) -> Matrix3<f64> {
        let bg0 = Matrix3::from_columns(&[vscmg.gs_hat0_b, vscmg.gt_hat0_b, vscmg.gg_hat_b]);
        let mut gg0 = Matrix3::identity();
        gg0[(0, 0)] = vscmg.gimbal_angle_rad.cos();
        gg0[(0, 1)] = vscmg.gimbal_angle_rad.sin();
        gg0[(1, 0)] = -gg0[(0, 1)];
        gg0[(1, 1)] = gg0[(0, 0)];
        bg0 * gg0.transpose()
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::test_support::{UpstreamVscmg, setup_vscmgs, upstream_bg};
    use super::{VscmgGimbalRateServo, VscmgGimbalRateServoConfig};
    use crate::messages::{
        MAX_EFF_COUNT, NavigationAttitudeMsg, Output, VscmgArrayConfigMsg, VscmgCommandMsg,
        VscmgReferenceStatesMsg, VscmgSpeedMsg,
    };
    use crate::{Module, SimulationContext};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The pytest's `computeTorques`: `(u_s, u_g)` of each VSCMG.
    fn upstream_truth(
        vscmgs: &[UpstreamVscmg],
        omega_bn_b: Vector3<f64>,
        k_gamma: f64,
        gamma_dot_d: &[f64],
        omega_dot_d: &[f64],
    ) -> Vec<(f64, f64)> {
        vscmgs
            .iter()
            .enumerate()
            .map(|(i, vscmg)| {
                let bg = upstream_bg(vscmg);
                let gs_hat = bg.column(0);
                let gt_hat = bg.column(1);
                let js = vscmg.ig[0] + vscmg.iw[0];
                let jt = vscmg.ig[1] + vscmg.iw[1];
                let jg = vscmg.ig[2] + vscmg.iw[2];
                let iws = vscmg.iw[0];
                let omega_s = gs_hat.dot(&omega_bn_b);
                let omega_t = gt_hat.dot(&omega_bn_b);
                let u_s = vscmg.iw[0] * (omega_dot_d[i] + vscmg.gimbal_rate_radps * omega_t);
                let gamma_ddot = -k_gamma * (vscmg.gimbal_rate_radps - gamma_dot_d[i]);
                let u_g = jg * gamma_ddot
                    - (js - jt) * omega_s * omega_t
                    - iws * vscmg.wheel_speed_radps * omega_t;
                (u_s, u_g)
            })
            .collect()
    }

    fn speeds(vscmgs: &[UpstreamVscmg]) -> VscmgSpeedMsg {
        let mut message = VscmgSpeedMsg::default();
        for (i, vscmg) in vscmgs.iter().enumerate() {
            message.wheel_speeds_radps[i] = vscmg.wheel_speed_radps;
            message.gimbal_angles_rad[i] = vscmg.gimbal_angle_rad;
            message.gimbal_rates_radps[i] = vscmg.gimbal_rate_radps;
        }
        message
    }

    fn cluster(vscmgs: &[UpstreamVscmg]) -> VscmgArrayConfigMsg {
        let elements: Vec<_> = vscmgs.iter().map(UpstreamVscmg::config_element).collect();
        VscmgArrayConfigMsg::from_active(&elements)
    }

    fn commands(module: &VscmgGimbalRateServo) -> Vec<VscmgCommandMsg> {
        module.cmds_out_msgs.iter().map(Output::read).collect()
    }

    /// Upstream `test_vscmgGimbalRateServo`, all `numVSCMGs` (1, 2, 4)
    /// parametrizations: one update with `K_gammaDot = 1`, the wheel and
    /// gimbal torques checked against the truth with the absolute accuracy
    /// 1e-12 (upstream also checks the identical C-wrapped output).
    #[test]
    fn matches_upstream_vscmg_gimbal_rate_servo_unit_test() {
        let omega_bn_b = Vector3::new(0.01, -0.01, 0.005);
        let rates_d = [0.0, 1.0, -1.0, -1.0].map(f64::to_radians);
        let k_gamma_dot = 1.0;
        for num_vscmgs in [1, 2, 4] {
            let vscmgs = setup_vscmgs(num_vscmgs);
            let mut reference = VscmgReferenceStatesMsg::default();
            reference.wheel_accels_radps2[..num_vscmgs].copy_from_slice(&rates_d[..num_vscmgs]);
            reference.gimbal_rates_radps[..num_vscmgs].copy_from_slice(&rates_d[..num_vscmgs]);

            let params_out = Output::new(cluster(&vscmgs));
            let reference_out = Output::new(reference);
            let att_out = Output::new(NavigationAttitudeMsg {
                omega_bn_b_radps: omega_bn_b,
                ..Default::default()
            });
            let speeds_out = Output::new(speeds(&vscmgs));
            let mut module = VscmgGimbalRateServo::new(VscmgGimbalRateServoConfig {
                k_gamma_dot_per_s: k_gamma_dot,
                ..Default::default()
            });
            params_out.connect_to(&mut module.vscmg_params_in_msg);
            reference_out.connect_to(&mut module.vscmg_ref_states_in_msg);
            att_out.connect_to(&mut module.att_in_msg);
            speeds_out.connect_to(&mut module.speeds_in_msg);
            for _ in 0..num_vscmgs {
                module.add_vscmg_command_output();
            }
            module.init();
            module.update(&context(0));

            let truth = upstream_truth(
                &vscmgs,
                omega_bn_b,
                k_gamma_dot,
                &rates_d[..num_vscmgs],
                &rates_d[..num_vscmgs],
            );
            for (i, (command, (u_s, u_g))) in commands(&module).iter().zip(truth).enumerate() {
                assert!(
                    (command.u_s_cmd_nm - u_s).abs() <= 1e-12,
                    "{num_vscmgs} VSCMGs, wheel {i}: {} vs {u_s}",
                    command.u_s_cmd_nm
                );
                assert!(
                    (command.u_g_cmd_nm - u_g).abs() <= 1e-12,
                    "{num_vscmgs} VSCMGs, gimbal {i}: {} vs {u_g}",
                    command.u_g_cmd_nm
                );
            }
        }
    }

    /// Upstream's `!speedsInMsg.isWritten()` branch, through the stand-in:
    /// without a speeds input the configured initial states are used.
    #[test]
    fn falls_back_to_the_initial_states_without_a_speeds_input() {
        let mut vscmgs = setup_vscmgs(4);
        for (i, vscmg) in vscmgs.iter_mut().enumerate() {
            vscmg.wheel_speed_radps = 10.0 + i as f64;
            vscmg.gimbal_rate_radps = 0.01 * (i as f64 + 1.0);
        }
        let params_out = Output::new(cluster(&vscmgs));
        let att_out = Output::new(NavigationAttitudeMsg {
            omega_bn_b_radps: Vector3::new(0.02, 0.01, -0.03),
            ..Default::default()
        });
        let mut module = VscmgGimbalRateServo::new(VscmgGimbalRateServoConfig {
            k_gamma_dot_per_s: 0.5,
            ..Default::default()
        });
        params_out.connect_to(&mut module.vscmg_params_in_msg);
        att_out.connect_to(&mut module.att_in_msg);
        for _ in 0..4 {
            module.add_vscmg_command_output();
        }
        module.init();
        module.update(&context(0));
        let truth = upstream_truth(
            &vscmgs,
            att_out.read().omega_bn_b_radps,
            0.5,
            &[0.0; 4],
            &[0.0; 4],
        );
        for (command, (u_s, u_g)) in commands(&module).iter().zip(truth) {
            assert!((command.u_s_cmd_nm - u_s).abs() <= 1e-15);
            assert!((command.u_g_cmd_nm - u_g).abs() <= 1e-15);
        }

        // Once connected, the speeds input wins over the initial states.
        let speeds_out = Output::new(VscmgSpeedMsg::default());
        speeds_out.connect_to(&mut module.speeds_in_msg);
        module.update(&context(100_000_000));
        let zero_state: Vec<_> = vscmgs
            .iter()
            .map(|vscmg| UpstreamVscmg {
                wheel_speed_radps: 0.0,
                gimbal_angle_rad: 0.0,
                gimbal_rate_radps: 0.0,
                ..*vscmg
            })
            .collect();
        let truth = upstream_truth(
            &zero_state,
            att_out.read().omega_bn_b_radps,
            0.5,
            &[0.0; 4],
            &[0.0; 4],
        );
        for (command, (u_s, u_g)) in commands(&module).iter().zip(truth) {
            assert!((command.u_s_cmd_nm - u_s).abs() <= 1e-15);
            assert!((command.u_g_cmd_nm - u_g).abs() <= 1e-15);
        }
    }

    #[test]
    fn outputs_past_the_cluster_get_zero_torques() {
        let vscmgs = setup_vscmgs(2);
        let params_out = Output::new(cluster(&vscmgs));
        let att_out = Output::new(NavigationAttitudeMsg {
            omega_bn_b_radps: Vector3::new(0.01, -0.01, 0.005),
            ..Default::default()
        });
        let mut module = VscmgGimbalRateServo::new(VscmgGimbalRateServoConfig::default());
        params_out.connect_to(&mut module.vscmg_params_in_msg);
        att_out.connect_to(&mut module.att_in_msg);
        for _ in 0..MAX_EFF_COUNT {
            module.add_vscmg_command_output();
        }
        module.init();
        module.update(&context(0));
        let output = commands(&module);
        assert!(output[..2].iter().all(|c| c.u_g_cmd_nm != 0.0));
        assert!(output[2..].iter().all(|c| *c == VscmgCommandMsg::default()));
    }
}
