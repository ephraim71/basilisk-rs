//! VSCMG velocity-based steering law: port of upstream Basilisk
//! `fswAlgorithms/effectorInterfaces/vscmgVelocitySteering`.
//!
//! Maps a required body control torque `L_r` onto desired wheel
//! accelerations `Ω̇` and gimbal rates `γ̇` of a cluster of variable-speed
//! control moment gyroscopes (Schaub & Junkins, *Analytical Mechanics of
//! Space Systems*, section 8.8). With, for each VSCMG `i`, the current spin,
//! transverse and gimbal axes `ĝ_s`, `ĝ_t`, `ĝ_g`, the body rate components
//! `ω_s`, `ω_t` and the reference rate `ω_R`, the 3×N matrices
//!
//! ```text
//! D0_i = I_ws ĝ_s
//! D1_i = (I_ws Ω + J_s ω_s / 2) ĝ_t + J_s ω_t / 2 ĝ_s
//! D2_i = J_t (ω_t ĝ_s + ω_s ĝ_t) / 2
//! D3_i = J_g (ω_t ĝ_s - ω_s ĝ_t)
//! D4_i = (J_s - J_t) (ĝ_s ĝ_t^T + ĝ_t ĝ_s^T) ω_R / 2
//! ```
//!
//! form `[Q] = [D0, D1 - D2 + D3 + D4]`, and the weighted minimum-norm
//! solution of `[Q] η̇ = -L_r` is
//!
//! ```text
//! η̇ = (Ω̇, γ̇) = [W] [Q]^T ([Q] [W] [Q]^T)^-1 (-L_r)
//! ```
//!
//! The diagonal weights are `W_g` for the gimbal rates and
//! `W_s = W0_s exp(-μ δ)` for the wheel accelerations, where
//! `δ = det([D1] [D1]^T / h̄²)` measures the distance to a CMG singularity:
//! the wheels take over as the gimbals approach one. `h̄` is the mean
//! nominal wheel momentum `J_s Ω0` of the cluster, computed at reset.
//!
//! **Header stand-in:** as [`super::vscmg_gimbal_rate_servo`], the speeds
//! input counts as written when it is connected; otherwise the configured
//! initial states are used.

use nalgebra::{DMatrix, DVector, Matrix3, Matrix3xX, Vector3};

use super::vscmg_gimbal_rate_servo::{active_vscmgs, gimbal_to_body, vscmg_states};
use crate::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, Input, NavigationAttitudeMsg, Output,
    VscmgArrayConfigMsg, VscmgConfigElementMsg, VscmgReferenceStatesMsg, VscmgSpeedMsg,
};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`VscmgVelocitySteering`].
#[derive(Clone, Debug)]
pub struct VscmgVelocitySteeringConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `mu` [-]: rate at which the wheel weights grow near a CMG
    /// singularity (default 0).
    pub mu: f64,
    /// Upstream `W0_s` [-]: nominal wheel-acceleration weight of each VSCMG,
    /// in cluster order. Missing entries are zero, as upstream's
    /// zero-initialized `MAX_EFF_CNT` array.
    pub w0_s: Vec<f64>,
    /// Upstream `W_g` [-]: gimbal-rate weight of each VSCMG, in cluster
    /// order; missing entries are zero.
    pub w_g: Vec<f64>,
}

impl Default for VscmgVelocitySteeringConfig {
    fn default() -> Self {
        Self {
            name: "vscmgVelocitySteering".to_string(),
            mu: 0.0,
            w0_s: Vec::new(),
            w_g: Vec::new(),
        }
    }
}

/// Required body torque to desired VSCMG wheel accelerations and gimbal
/// rates: upstream `VscmgVelocitySteering`.
#[derive(Clone, Debug)]
pub struct VscmgVelocitySteering {
    /// Module configuration.
    pub config: VscmgVelocitySteeringConfig,
    /// Upstream `vscmgParamsInMsg`: VSCMG cluster configuration, read at
    /// reset.
    pub vscmg_params_in_msg: Input<VscmgArrayConfigMsg>,
    /// Upstream `vehControlInMsg`: required control torque `L_r`, body
    /// components.
    pub veh_control_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `attNavInMsg`: body rate `ω_BN`.
    pub att_nav_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `attGuideInMsg`: reference rate `ω_RN`, body components.
    pub att_guide_in_msg: Input<AttitudeGuidanceMsg>,
    /// Upstream `speedsInMsg`: current wheel speeds, gimbal angles and rates.
    pub speeds_in_msg: Input<VscmgSpeedMsg>,
    /// Upstream `vscmgRefStatesOutMsg`: desired wheel accelerations and
    /// gimbal rates.
    pub vscmg_ref_states_out_msg: Output<VscmgReferenceStatesMsg>,
    /// Upstream `vscmgConfigParams`: the VSCMGs in use, read at reset.
    vscmgs: Vec<VscmgConfigElementMsg>,
    /// Upstream `h_bar_squared` \[kg^2 m^4/s^2\]: square of the mean nominal
    /// wheel momentum.
    h_bar_squared: f64,
}

impl VscmgVelocitySteering {
    /// Builds the module with unconnected ports and no VSCMGs; the cluster
    /// is read at reset.
    pub fn new(config: VscmgVelocitySteeringConfig) -> Self {
        Self {
            config,
            vscmg_params_in_msg: Input::default(),
            veh_control_in_msg: Input::default(),
            att_nav_in_msg: Input::default(),
            att_guide_in_msg: Input::default(),
            speeds_in_msg: Input::default(),
            vscmg_ref_states_out_msg: Output::default(),
            vscmgs: Vec::new(),
            h_bar_squared: 0.0,
        }
    }

    /// Upstream `Reset`: reports missing inputs, reads the cluster and
    /// computes the mean nominal wheel momentum `h̄ = mean(J_s Ω0)`.
    fn reset_module(&mut self) {
        for (connected, port) in [
            (
                self.vscmg_params_in_msg.is_connected(),
                "vscmg_params_in_msg",
            ),
            (self.veh_control_in_msg.is_connected(), "veh_control_in_msg"),
            (self.att_nav_in_msg.is_connected(), "att_nav_in_msg"),
            (self.att_guide_in_msg.is_connected(), "att_guide_in_msg"),
            (self.speeds_in_msg.is_connected(), "speeds_in_msg"),
        ] {
            if !connected {
                log::error!("vscmgVelocitySteering.{port} is not connected");
            }
        }
        self.vscmgs = active_vscmgs(&self.vscmg_params_in_msg.read(), "vscmgVelocitySteering");
        let mean_h_bar = self
            .vscmgs
            .iter()
            .map(|vscmg| vscmg.js_kg_m2 * vscmg.wheel_speed0_radps)
            .sum::<f64>()
            / self.vscmgs.len() as f64;
        self.h_bar_squared = mean_h_bar * mean_h_bar;
    }

    /// Upstream `UpdateState`: the desired wheel accelerations and gimbal
    /// rates of the VSCMGs in use.
    fn compute_reference_states(&self) -> VscmgReferenceStatesMsg {
        let n = self.vscmgs.len();
        let torque_request_b = self.veh_control_in_msg.read().torque_request_body_nm;
        let omega_bn_b = self.att_nav_in_msg.read().omega_bn_b_radps;
        let omega_rn_b = self.att_guide_in_msg.read().omega_rn_b_radps;

        let mut d0 = Matrix3xX::zeros(n);
        let mut d1 = Matrix3xX::zeros(n);
        let mut d2 = Matrix3xX::zeros(n);
        let mut d3 = Matrix3xX::zeros(n);
        let mut d4 = Matrix3xX::zeros(n);
        for (i, (vscmg, state)) in self
            .vscmgs
            .iter()
            .zip(vscmg_states(&self.speeds_in_msg, &self.vscmgs))
            .enumerate()
        {
            let bg = gimbal_to_body(vscmg, state.gimbal_angle_rad);
            let gs_hat: Vector3<f64> = bg.column(0).into();
            let gt_hat: Vector3<f64> = bg.column(1).into();
            let omega_s = gs_hat.dot(&omega_bn_b);
            let omega_t = gt_hat.dot(&omega_bn_b);
            let (js, jt, jg, iws) = (
                vscmg.js_kg_m2,
                vscmg.jt_kg_m2,
                vscmg.jg_kg_m2,
                vscmg.iws_kg_m2,
            );

            d0.set_column(i, &(gs_hat * iws));
            d1.set_column(
                i,
                &((iws * state.wheel_speed_radps + 0.5 * js * omega_s) * gt_hat
                    + 0.5 * js * omega_t * gs_hat),
            );
            d2.set_column(i, &(0.5 * jt * (omega_t * gs_hat + omega_s * gt_hat)));
            d3.set_column(i, &(jg * (omega_t * gs_hat - omega_s * gt_hat)));
            d4.set_column(
                i,
                &(0.5
                    * (js - jt)
                    * (gs_hat * gt_hat.transpose() * omega_rn_b
                        + gt_hat * gs_hat.transpose() * omega_rn_b)),
            );
        }
        let d = &d1 - &d2 + &d3 + &d4;
        let mut q = Matrix3xX::zeros(2 * n);
        q.columns_mut(0, n).copy_from(&d0);
        q.columns_mut(n, n).copy_from(&d);

        let delta = ((1.0 / self.h_bar_squared) * (&d1 * d1.transpose())).determinant();
        let weight = |weights: &[f64], i: usize| weights.get(i).copied().unwrap_or(0.0);
        let w = DMatrix::from_diagonal(&DVector::from_iterator(
            2 * n,
            (0..n)
                .map(|i| weight(&self.config.w0_s, i) * (-self.config.mu * delta).exp())
                .chain((0..n).map(|i| weight(&self.config.w_g, i))),
        ));

        let qwqt: Matrix3<f64> = &q * &w * q.transpose();
        let mut output = VscmgReferenceStatesMsg::default();
        let Some(solution) = solve_symmetric(qwqt, -torque_request_b) else {
            log::error!("vscmgVelocitySteering: [Q][W][Q]^T is singular");
            return output;
        };
        let eta_dot = &w * q.transpose() * solution;
        output.wheel_accels_radps2[..n].copy_from_slice(&eta_dot.as_slice()[..n]);
        output.gimbal_rates_radps[..n].copy_from_slice(&eta_dot.as_slice()[n..]);
        output
    }
}

/// `[A]^-1 b` for the symmetric `[A]`, by Cholesky factorization when `[A]`
/// is positive definite (upstream uses Eigen's LDLT), otherwise by LU.
fn solve_symmetric(a: Matrix3<f64>, b: Vector3<f64>) -> Option<Vector3<f64>> {
    match a.cholesky() {
        Some(cholesky) => Some(cholesky.solve(&b)),
        None => a.lu().solve(&b),
    }
}

impl Module for VscmgVelocitySteering {
    fn init(&mut self) {
        self.vscmg_ref_states_out_msg
            .write(VscmgReferenceStatesMsg::default());
        self.reset_module();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_module();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let output = self.compute_reference_states();
        self.vscmg_ref_states_out_msg.write(output);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{DMatrix, DVector, Matrix3xX, Vector3};

    use super::{VscmgVelocitySteering, VscmgVelocitySteeringConfig};
    use crate::fsw_algorithms::effector_interfaces::vscmg_gimbal_rate_servo::test_support::{
        UpstreamVscmg, setup_vscmgs, upstream_bg,
    };
    use crate::messages::{
        AttitudeGuidanceMsg, BodyTorqueCommandMsg, NavigationAttitudeMsg, Output,
        VscmgArrayConfigMsg, VscmgReferenceStatesMsg, VscmgSpeedMsg,
    };
    use crate::{Module, SimulationContext};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The pytest's `etaDot`: `[W] [Q]^T ([Q][W][Q]^T)^-1 (-L_r)`, wheel
    /// accelerations first.
    fn upstream_truth(
        vscmgs: &[UpstreamVscmg],
        omega_bn_b: Vector3<f64>,
        omega_rn_b: Vector3<f64>,
        mu: f64,
        w0_s: &[f64],
        w_g: &[f64],
        lr: Vector3<f64>,
    ) -> DVector<f64> {
        let n = vscmgs.len();
        let mut h_bar = Vec::with_capacity(n);
        let mut d0 = Matrix3xX::zeros(n);
        let mut d1 = Matrix3xX::zeros(n);
        let mut d2 = Matrix3xX::zeros(n);
        let mut d3 = Matrix3xX::zeros(n);
        let mut d4 = Matrix3xX::zeros(n);
        for (i, vscmg) in vscmgs.iter().enumerate() {
            let bg = upstream_bg(vscmg);
            let gs_hat: Vector3<f64> = bg.column(0).into();
            let gt_hat: Vector3<f64> = bg.column(1).into();
            let js = vscmg.ig[0] + vscmg.iw[0];
            let jt = vscmg.ig[1] + vscmg.iw[1];
            let jg = vscmg.ig[2] + vscmg.iw[2];
            let iws = vscmg.iw[0];
            let omega_s = gs_hat.dot(&omega_bn_b);
            let omega_t = gt_hat.dot(&omega_bn_b);
            let omega = vscmg.wheel_speed_radps;
            d0.set_column(i, &(gs_hat * iws));
            d1.set_column(
                i,
                &((iws * omega + js / 2.0 * omega_s) * gt_hat + js / 2.0 * omega_t * gs_hat),
            );
            d2.set_column(i, &(1.0 / 2.0 * jt * (omega_t * gs_hat + omega_s * gt_hat)));
            d3.set_column(i, &(jg * (omega_t * gs_hat - omega_s * gt_hat)));
            d4.set_column(
                i,
                &(1.0 / 2.0
                    * (js - jt)
                    * ((gs_hat * gt_hat.transpose()) * omega_rn_b
                        + (gt_hat * gs_hat.transpose()) * omega_rn_b)),
            );
            h_bar.push(js * omega);
        }
        let d = &d1 - &d2 + &d3 + &d4;
        let mut q = Matrix3xX::zeros(2 * n);
        q.columns_mut(0, n).copy_from(&d0);
        q.columns_mut(n, n).copy_from(&d);
        let mean_h_bar = h_bar.iter().sum::<f64>() / n as f64;
        let h_bar_squared = mean_h_bar.powi(2);
        let delta = (1.0 / h_bar_squared * (&d1 * d1.transpose())).determinant();
        let mut w = DMatrix::zeros(2 * n, 2 * n);
        for i in 0..n {
            w[(i, i)] = w0_s[i] * (-mu * delta).exp();
            w[(n + i, n + i)] = w_g[i];
        }
        let qwqt = &q * &w * q.transpose();
        // `np.linalg.solve`: an LU solve.
        let solution = qwqt.lu().solve(&-lr).expect("a regular [Q][W][Q]^T");
        &w * q.transpose() * solution
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

    struct Fixture {
        module: VscmgVelocitySteering,
        _outputs: (
            Output<VscmgArrayConfigMsg>,
            Output<BodyTorqueCommandMsg>,
            Output<NavigationAttitudeMsg>,
            Output<AttitudeGuidanceMsg>,
        ),
        speeds_out: Output<VscmgSpeedMsg>,
    }

    const OMEGA_BN_B: [f64; 3] = [0.01, -0.01, 0.005];
    const OMEGA_RN_B: [f64; 3] = [0.00649856, -0.01231292, -0.01172612];
    const LR: [f64; 3] = [-0.42470656, -0.82132484, -1.76165287];
    const MU: f64 = 1e-9;

    /// The pytest's module: `mu = 1e-9`, `W0_s = 200` and `W_g = 1` for
    /// every VSCMG.
    fn fixture(vscmgs: &[UpstreamVscmg], connect_speeds: bool) -> Fixture {
        let n = vscmgs.len();
        let mut module = VscmgVelocitySteering::new(VscmgVelocitySteeringConfig {
            mu: MU,
            w0_s: vec![200.0; n],
            w_g: vec![1.0; n],
            ..Default::default()
        });
        let elements: Vec<_> = vscmgs.iter().map(UpstreamVscmg::config_element).collect();
        let params_out = Output::new(VscmgArrayConfigMsg::from_active(&elements));
        let control_out = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::from(LR),
        });
        let att_out = Output::new(NavigationAttitudeMsg {
            omega_bn_b_radps: Vector3::from(OMEGA_BN_B),
            ..Default::default()
        });
        let guide_out = Output::new(AttitudeGuidanceMsg {
            omega_rn_b_radps: Vector3::from(OMEGA_RN_B),
            ..Default::default()
        });
        let speeds_out = Output::new(speeds(vscmgs));
        params_out.connect_to(&mut module.vscmg_params_in_msg);
        control_out.connect_to(&mut module.veh_control_in_msg);
        att_out.connect_to(&mut module.att_nav_in_msg);
        guide_out.connect_to(&mut module.att_guide_in_msg);
        if connect_speeds {
            speeds_out.connect_to(&mut module.speeds_in_msg);
        }
        module.init();
        Fixture {
            module,
            _outputs: (params_out, control_out, att_out, guide_out),
            speeds_out,
        }
    }

    fn assert_matches_truth(output: &VscmgReferenceStatesMsg, vscmgs: &[UpstreamVscmg]) {
        let n = vscmgs.len();
        let truth = upstream_truth(
            vscmgs,
            Vector3::from(OMEGA_BN_B),
            Vector3::from(OMEGA_RN_B),
            MU,
            &vec![200.0; n],
            &vec![1.0; n],
            Vector3::from(LR),
        );
        let actual = output.wheel_accels_radps2[..n]
            .iter()
            .chain(&output.gimbal_rates_radps[..n]);
        for (index, (value, expected)) in actual.zip(truth.iter()).enumerate() {
            assert!(
                (value - expected).abs() <= 1e-12,
                "{n} VSCMGs, entry {index}: {value:e} vs {expected:e}"
            );
        }
    }

    /// Upstream `test_vscmgVelocitySteering`, both `numVSCMGs` (2, 4)
    /// parametrizations: one update, the desired wheel accelerations and
    /// gimbal rates checked against the truth with the absolute accuracy
    /// 1e-12 (upstream also checks the identical C-wrapped output).
    #[test]
    fn matches_upstream_vscmg_velocity_steering_unit_test() {
        for num_vscmgs in [2, 4] {
            let vscmgs = setup_vscmgs(num_vscmgs);
            let mut fixture = fixture(&vscmgs, true);
            fixture.module.update(&context(0));
            let output = fixture.module.vscmg_ref_states_out_msg.read();
            assert_matches_truth(&output, &vscmgs);
            assert!(
                output.wheel_accels_radps2[num_vscmgs..]
                    .iter()
                    .all(|&x| x == 0.0)
            );
            assert!(
                output.gimbal_rates_radps[num_vscmgs..]
                    .iter()
                    .all(|&x| x == 0.0)
            );
        }
    }

    /// Upstream's `!speedsInMsg.isWritten()` branch, through the stand-in:
    /// without a speeds input the law steers from the configured initial
    /// states.
    #[test]
    fn falls_back_to_the_initial_states_without_a_speeds_input() {
        let vscmgs = setup_vscmgs(4);
        let mut connected = fixture(&vscmgs, true);
        let mut unconnected = fixture(&vscmgs, false);
        connected.module.update(&context(0));
        unconnected.module.update(&context(0));
        assert_eq!(
            connected.module.vscmg_ref_states_out_msg.read(),
            unconnected.module.vscmg_ref_states_out_msg.read()
        );

        // A written speeds message replaces the initial states.
        let mut moved = speeds(&vscmgs);
        moved.gimbal_angles_rad[0] += 0.3;
        connected.speeds_out.write(moved);
        connected.module.update(&context(100_000_000));
        assert_ne!(
            connected.module.vscmg_ref_states_out_msg.read(),
            unconnected.module.vscmg_ref_states_out_msg.read()
        );
    }

    /// Not upstream: the desired rates realize the requested torque,
    /// `[Q] η̇ = -L_r`.
    #[test]
    fn the_gimbal_and_wheel_rates_produce_the_requested_torque() {
        let vscmgs = setup_vscmgs(4);
        let mut fixture = fixture(&vscmgs, true);
        fixture.module.update(&context(0));
        let output = fixture.module.vscmg_ref_states_out_msg.read();
        let mut torque = Vector3::zeros();
        for (i, vscmg) in vscmgs.iter().enumerate() {
            let element = vscmg.config_element();
            let bg = upstream_bg(vscmg);
            let gs_hat: Vector3<f64> = bg.column(0).into();
            let gt_hat: Vector3<f64> = bg.column(1).into();
            let omega_bn_b = Vector3::from(OMEGA_BN_B);
            let omega_rn_b = Vector3::from(OMEGA_RN_B);
            let (omega_s, omega_t) = (gs_hat.dot(&omega_bn_b), gt_hat.dot(&omega_bn_b));
            let (js, jt, jg, iws) = (
                element.js_kg_m2,
                element.jt_kg_m2,
                element.jg_kg_m2,
                element.iws_kg_m2,
            );
            let d = (iws * vscmg.wheel_speed_radps + 0.5 * js * omega_s) * gt_hat
                + 0.5 * js * omega_t * gs_hat
                - 0.5 * jt * (omega_t * gs_hat + omega_s * gt_hat)
                + jg * (omega_t * gs_hat - omega_s * gt_hat)
                + 0.5
                    * (js - jt)
                    * (gs_hat * gt_hat.dot(&omega_rn_b) + gt_hat * gs_hat.dot(&omega_rn_b));
            torque +=
                iws * gs_hat * output.wheel_accels_radps2[i] + d * output.gimbal_rates_radps[i];
        }
        assert!((torque + Vector3::from(LR)).amax() < 1e-12, "{torque}");
    }
}
