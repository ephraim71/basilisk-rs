//! Variable-speed control moment gyroscopes: port of upstream Basilisk
//! `simulation/dynamics/VSCMGs` (`VSCMGStateEffector`).
//!
//! Each VSCMG is a wheel spinning at `Ω` about `ĝ_s` inside a gimbal turning
//! at `γ̇` about the body-fixed `ĝ_g`; the gimbal frame G (axes `ĝ_s`, `ĝ_t`,
//! `ĝ_g`) is `[BG] = [BG0] [M3(γ)]ᵀ`. Three models
//! ([`VscmgModel`]), as upstream:
//!
//! - balanced wheels: wheel and gimbal balanced, centers of mass at the
//!   gimbal origin;
//! - jitter simple: the balanced model plus the static (`U_s Ω²`) and
//!   dynamic (`U_d Ω²`) wheel imbalance as a force and torque along the
//!   wheel axis `ŵ2`, which turns with the wheel angle `θ`;
//! - jitter fully coupled: the wheel center of mass offset `d = U_s / m_W`
//!   and product of inertia `I_W13 = U_d`, the gimbal center of mass offset
//!   and inertia products, and the wheel offsets `l`, `L`, all coupled into
//!   the hub by back-substitution.
//!
//! State layout: `[Ω_1…Ω_n, γ_1…γ_n, γ̇_1…γ̇_n, θ…]`, the wheel angles `θ` of
//! the jitter VSCMGs last, in the order the VSCMGs were added.
//!
//! Each VSCMG reads its own [`VscmgCommandMsg`] before every integration step
//! (upstream reads one `VSCMGArrayTorqueMsg` for the cluster) and applies
//! the torque limits, deadbands, speed limit and Coulomb friction of upstream
//! `ConfigureVSCMGRequests`. After every step the effector publishes the
//! cluster's [`VscmgSpeedMsg`] (upstream `speedOutMsg`), what the FSW VSCMG
//! modules read, and one [`VscmgConfigMsg`] per VSCMG (upstream
//! `vscmgOutMsgs`).
//!
//! Differences from upstream:
//! - Upstream raises an error for an unlinked or unwritten command input.
//!   Here an unconnected command input leaves that VSCMG's applied torques
//!   at their current values (initially [`VscmgConfig::u_s_current_nm`] and
//!   [`VscmgConfig::u_g_current_nm`]), as upstream does for an effector
//!   whose commands are never processed.
//! - Upstream validates and derives the configuration in `Reset` and at
//!   state registration. Here the derived values are formed from the
//!   current [`VscmgConfig`] where they are used, and the configuration is
//!   validated when the spacecraft initializes ([`StateEffector::initial_state`])
//!   and before every step; an invalid one panics with upstream's message.

use nalgebra::{Matrix3, Vector3};
use std::any::Any;
use std::cell::RefCell;

use crate::kinematics::is_rotation_matrix;
use crate::messages::{
    Input, MAX_EFF_COUNT, Output, SpacecraftStateMsg, VscmgCommandMsg, VscmgConfigMsg, VscmgModel,
    VscmgSpeedMsg,
};
use crate::spacecraft::{BackSubMatrices, StateEffector, StateEffectorMassProps};
use crate::utilities::rigid_body_kinematics::{euler1, euler3};

/// Configuration of one VSCMG: the user-settable fields of upstream
/// `VSCMGConfigMsgPayload`.
#[derive(Clone, Debug, PartialEq)]
pub struct VscmgConfig {
    /// Device name.
    pub name: String,
    /// Upstream `VSCMGModel`.
    pub model: VscmgModel,
    /// Upstream `rGB_B` \[m\]: gimbal frame origin G relative to B.
    pub r_gb_b_m: Vector3<f64>,
    /// Upstream `gsHat0_B`: spin axis at zero gimbal angle.
    pub gs_hat0_b: Vector3<f64>,
    /// Upstream `gtHat0_B`: transverse axis at zero gimbal angle.
    pub gt_hat0_b: Vector3<f64>,
    /// Upstream `ggHat_B`: gimbal axis.
    pub gg_hat_b: Vector3<f64>,
    /// Upstream `massW` \[kg\]: wheel mass.
    pub mass_w_kg: f64,
    /// Upstream `massG` \[kg\]: gimbal mass.
    pub mass_g_kg: f64,
    /// Upstream `IW1`, `IW2`, `IW3` \[kg m^2\]: wheel inertias about its
    /// center of mass, wheel-frame axes.
    pub iw_kg_m2: [f64; 3],
    /// Upstream `IG1`, `IG2`, `IG3` \[kg m^2\]: gimbal inertias about its
    /// center of mass, gimbal-frame axes.
    pub ig_kg_m2: [f64; 3],
    /// Upstream `IG12`, `IG13`, `IG23` \[kg m^2\]: gimbal products of inertia
    /// (fully coupled model only).
    pub ig_products_kg_m2: [f64; 3],
    /// Upstream `U_s` \[kg m\]: static wheel imbalance.
    pub static_imbalance_kg_m: f64,
    /// Upstream `U_d` \[kg m^2\]: dynamic wheel imbalance.
    pub dynamic_imbalance_kg_m2: f64,
    /// Upstream `l` \[m\]: offset of the wheel third axis from the gimbal
    /// third axis (fully coupled model only).
    pub l_m: f64,
    /// Upstream `L` \[m\]: offset of the wheel first axis from the gimbal
    /// first axis (fully coupled model only).
    pub big_l_m: f64,
    /// Upstream `rGcG_G` \[m\]: gimbal center of mass relative to G, gimbal
    /// frame (fully coupled model only).
    pub r_gcg_g_m: Vector3<f64>,
    /// Upstream `Omega` \[rad/s\]: initial wheel speed.
    pub omega_init_radps: f64,
    /// Upstream `gamma` \[rad\]: initial gimbal angle.
    pub gamma_init_rad: f64,
    /// Upstream `gammaDot` \[rad/s\]: initial gimbal rate.
    pub gamma_dot_init_radps: f64,
    /// Upstream `u_s_current` \[N m\]: wheel motor torque before the first
    /// command.
    pub u_s_current_nm: f64,
    /// Upstream `u_g_current` \[N m\]: gimbal motor torque before the first
    /// command.
    pub u_g_current_nm: f64,
    /// Upstream `u_s_max` \[N m\]: wheel torque limit (default −1: none).
    pub u_s_max_nm: f64,
    /// Upstream `u_s_min` \[N m\]: wheel commands smaller in magnitude are
    /// dropped.
    pub u_s_min_nm: f64,
    /// Upstream `u_s_f` \[N m\]: wheel Coulomb friction torque.
    pub u_s_f_nm: f64,
    /// Upstream `Omega_max` \[rad/s\]: wheel speed past which wheel commands
    /// are dropped (default −1: none).
    pub omega_max_radps: f64,
    /// Upstream `wheelLinearFrictionRatio` [-]: fraction of `Omega_max` below
    /// which the wheel friction is linear in speed (non-positive: none).
    pub wheel_linear_friction_ratio: f64,
    /// Upstream `u_g_max` \[N m\]: gimbal torque limit (default −1: none).
    pub u_g_max_nm: f64,
    /// Upstream `u_g_min` \[N m\]: gimbal commands smaller in magnitude are
    /// dropped.
    pub u_g_min_nm: f64,
    /// Upstream `u_g_f` \[N m\]: gimbal Coulomb friction torque.
    pub u_g_f_nm: f64,
    /// Upstream `gammaDot_max` \[rad/s\]: gimbal rate scale of the linear
    /// friction band.
    pub gamma_dot_max_radps: f64,
    /// Upstream `gimbalLinearFrictionRatio` [-]: fraction of `gammaDot_max`
    /// below which the gimbal friction is linear in rate (non-positive:
    /// none).
    pub gimbal_linear_friction_ratio: f64,
}

impl VscmgConfig {
    /// A VSCMG with zero geometry and inertia and upstream's limits off.
    pub fn new(name: impl Into<String>, model: VscmgModel) -> Self {
        Self {
            name: name.into(),
            model,
            r_gb_b_m: Vector3::zeros(),
            gs_hat0_b: Vector3::zeros(),
            gt_hat0_b: Vector3::zeros(),
            gg_hat_b: Vector3::zeros(),
            mass_w_kg: 0.0,
            mass_g_kg: 0.0,
            iw_kg_m2: [0.0; 3],
            ig_kg_m2: [0.0; 3],
            ig_products_kg_m2: [0.0; 3],
            static_imbalance_kg_m: 0.0,
            dynamic_imbalance_kg_m2: 0.0,
            l_m: 0.0,
            big_l_m: 0.0,
            r_gcg_g_m: Vector3::zeros(),
            omega_init_radps: 0.0,
            gamma_init_rad: 0.0,
            gamma_dot_init_radps: 0.0,
            u_s_current_nm: 0.0,
            u_g_current_nm: 0.0,
            u_s_max_nm: -1.0,
            u_s_min_nm: 0.0,
            u_s_f_nm: 0.0,
            omega_max_radps: -1.0,
            wheel_linear_friction_ratio: 0.0,
            u_g_max_nm: -1.0,
            u_g_min_nm: 0.0,
            u_g_f_nm: 0.0,
            gamma_dot_max_radps: 0.0,
            gimbal_linear_friction_ratio: 0.0,
        }
    }

    fn is_jitter(&self) -> bool {
        self.model != VscmgModel::BalancedWheels
    }

    /// Upstream `initializeConfiguration`'s checks: finite values, the
    /// signs of masses and inertias, a right-handed orthonormal gimbal frame,
    /// and the divisors of the model nonzero.
    pub fn validate(&self) -> Result<(), String> {
        let finite = |value: f64, name: &str| {
            if value.is_finite() {
                Ok(())
            } else {
                Err(format!("{name} must be finite"))
            }
        };
        finite(self.mass_w_kg, "massW")?;
        finite(self.mass_g_kg, "massG")?;
        if self.mass_w_kg < 0.0 || self.mass_g_kg < 0.0 {
            return Err("massW and massG must be nonnegative".to_string());
        }
        if self.model == VscmgModel::JitterFullyCoupled && self.mass_w_kg == 0.0 {
            return Err("massW must be positive for fully coupled jitter".to_string());
        }
        for (axis, name) in [
            (self.gs_hat0_b, "gsHat0_B"),
            (self.gt_hat0_b, "gtHat0_B"),
            (self.gg_hat_b, "ggHat_B"),
        ] {
            if !axis.iter().all(|value| value.is_finite()) || axis.amax() == 0.0 {
                return Err(format!("{name} must be finite and nonzero"));
            }
        }
        let axes = normalized_axes(self);
        if !is_rotation_matrix(&Matrix3::from_columns(&[axes.0, axes.1, axes.2])) {
            return Err(
                "gsHat0_B, gtHat0_B and ggHat_B must form an orthogonal, right-handed frame"
                    .to_string(),
            );
        }
        for (value, name) in [
            (self.omega_init_radps, "Omega"),
            (self.gamma_init_rad, "gamma"),
            (self.gamma_dot_init_radps, "gammaDot"),
            (self.u_s_current_nm, "u_s_current"),
            (self.u_g_current_nm, "u_g_current"),
            (self.u_s_f_nm, "u_s_f"),
            (self.u_g_f_nm, "u_g_f"),
            (self.u_s_max_nm, "u_s_max"),
            (self.u_g_max_nm, "u_g_max"),
            (self.u_s_min_nm, "u_s_min"),
            (self.u_g_min_nm, "u_g_min"),
            (self.omega_max_radps, "Omega_max"),
            (self.gamma_dot_max_radps, "gammaDot_max"),
            (self.wheel_linear_friction_ratio, "wheelLinearFrictionRatio"),
            (
                self.gimbal_linear_friction_ratio,
                "gimbalLinearFrictionRatio",
            ),
            (self.iw_kg_m2[0], "IW1"),
            (self.iw_kg_m2[1], "IW2"),
            (self.iw_kg_m2[2], "IW3"),
            (self.ig_kg_m2[0], "IG1"),
            (self.ig_kg_m2[1], "IG2"),
            (self.ig_kg_m2[2], "IG3"),
        ] {
            finite(value, name)?;
        }
        if self
            .iw_kg_m2
            .iter()
            .chain(&self.ig_kg_m2)
            .any(|&inertia| inertia < 0.0)
        {
            return Err("wheel and gimbal diagonal inertias must be nonnegative".to_string());
        }
        if !self.r_gb_b_m.iter().all(|value| value.is_finite()) {
            return Err("rGB_B must be finite".to_string());
        }
        if self.is_jitter() {
            finite(self.static_imbalance_kg_m, "U_s")?;
            finite(self.dynamic_imbalance_kg_m2, "U_d")?;
        }
        let derived = VscmgDerived::new(self);
        if self.model == VscmgModel::JitterFullyCoupled {
            for (value, name) in [
                (self.ig_products_kg_m2[0], "IG12"),
                (self.ig_products_kg_m2[1], "IG13"),
                (self.ig_products_kg_m2[2], "IG23"),
                (self.l_m, "l"),
                (self.big_l_m, "L"),
            ] {
                finite(value, name)?;
            }
            if !self.r_gcg_g_m.iter().all(|value| value.is_finite()) {
                return Err("rGcG_G must be finite".to_string());
            }
            finite(derived.d_m, "derived d")?;
            require_divisor(
                self.iw_kg_m2[0] + self.mass_w_kg * derived.d_m * derived.d_m,
                "eOmega",
            )?;
            finite(derived.mass_v_kg, "derived massV")?;
            require_divisor(derived.mass_v_kg, "massV")?;
        } else {
            for (value, name) in [
                (derived.iv_kg_m2[0], "derived IV1"),
                (derived.iv_kg_m2[1], "derived IV2"),
                (derived.iv_kg_m2[2], "derived IV3"),
            ] {
                finite(value, name)?;
            }
            if self.iw_kg_m2[0] == 0.0 || derived.iv_kg_m2[2] == 0.0 {
                return Err(
                    "IW1 and IW3 + IG3 must be positive for balanced and simple jitter models"
                        .to_string(),
                );
            }
            finite(derived.mass_v_kg, "derived massV")?;
        }
        Ok(())
    }
}

/// Upstream `requireFiniteNonzeroDivisor`.
fn require_divisor(value: f64, name: &str) -> Result<(), String> {
    if !value.is_finite() || value == 0.0 || !(1.0 / value).is_finite() {
        return Err(format!(
            "{name} must be finite and nonzero with a finite reciprocal"
        ));
    }
    Ok(())
}

/// Upstream `normalizeAxis` of `initializeConfiguration`: each axis scaled
/// by its largest component first, then normalized.
fn normalized_axes(config: &VscmgConfig) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
    let normalize = |axis: Vector3<f64>| (axis / axis.amax()).normalize();
    (
        normalize(config.gs_hat0_b),
        normalize(config.gt_hat0_b),
        normalize(config.gg_hat_b),
    )
}

/// Values upstream `initializeConfiguration` derives from a
/// [`VscmgConfig`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VscmgDerived {
    /// `[BG0]`: columns `ĝ_s0`, `ĝ_t0`, `ĝ_g`, normalized.
    pub dcm_bg0: Matrix3<f64>,
    /// Upstream `massV` \[kg\]: wheel plus gimbal mass.
    pub mass_v_kg: f64,
    /// Upstream `rhoG` [-]: gimbal share of `massV` (zero when massless).
    pub rho_g: f64,
    /// Upstream `rhoW` [-]: wheel share of `massV` (zero when massless).
    pub rho_w: f64,
    /// Upstream `d` \[m\]: wheel center of mass offset `U_s / m_W` (fully
    /// coupled; zero otherwise).
    pub d_m: f64,
    /// Upstream `IW13` \[kg m^2\]: `U_d` (fully coupled; zero otherwise).
    pub iw13_kg_m2: f64,
    /// Upstream `IG12`, `IG13`, `IG23` (zero unless fully coupled).
    pub ig_products_kg_m2: [f64; 3],
    /// Upstream `IV1`, `IV2`, `IV3` \[kg m^2\]: wheel plus gimbal inertias.
    pub iv_kg_m2: [f64; 3],
}

impl VscmgDerived {
    fn new(config: &VscmgConfig) -> Self {
        let (gs0, gt0, gg) = normalized_axes(config);
        let fully_coupled = config.model == VscmgModel::JitterFullyCoupled;
        let mass_v_kg = config.mass_g_kg + config.mass_w_kg;
        let (rho_g, rho_w) = if mass_v_kg > 0.0 {
            (config.mass_g_kg / mass_v_kg, config.mass_w_kg / mass_v_kg)
        } else {
            (0.0, 0.0)
        };
        let [iw1, iw2, iw3] = config.iw_kg_m2;
        let [ig1, ig2, ig3] = config.ig_kg_m2;
        Self {
            dcm_bg0: Matrix3::from_columns(&[gs0, gt0, gg]),
            mass_v_kg,
            rho_g,
            rho_w,
            d_m: if fully_coupled {
                config.static_imbalance_kg_m / config.mass_w_kg
            } else {
                0.0
            },
            iw13_kg_m2: if fully_coupled {
                config.dynamic_imbalance_kg_m2
            } else {
                0.0
            },
            ig_products_kg_m2: if fully_coupled {
                config.ig_products_kg_m2
            } else {
                [0.0; 3]
            },
            iv_kg_m2: [iw1 + ig1, iw2 + ig2, iw3 + ig3],
        }
    }
}

/// Geometry and mass distribution of one VSCMG at a state (upstream
/// `updateEffectorMassProps` per device).
struct VscmgKinematics {
    derived: VscmgDerived,
    omega: f64,
    gamma_dot: f64,
    theta: f64,
    gs_hat: Vector3<f64>,
    gt_hat: Vector3<f64>,
    gg_hat: Vector3<f64>,
    w2_hat: Vector3<f64>,
    w3_hat: Vector3<f64>,
    /// Upstream `IGPntGc_B`, `IPrimeGPntGc_B`.
    ig_b: Matrix3<f64>,
    ig_prime_b: Matrix3<f64>,
    /// Upstream `IWPntWc_B`, `IPrimeWPntWc_B`.
    iw_b: Matrix3<f64>,
    iw_prime_b: Matrix3<f64>,
    /// Upstream `rGcG_B`, `rGcB_B`, `rPrimeGcB_B` (fully coupled).
    r_gcg_b: Vector3<f64>,
    r_gcb_b: Vector3<f64>,
    r_prime_gcb_b: Vector3<f64>,
    /// Upstream `rWcG_B`, `rWcB_B`, `rPrimeWcB_B`.
    r_wcg_b: Vector3<f64>,
    r_wcb_b: Vector3<f64>,
    r_prime_wcb_b: Vector3<f64>,
}

/// Wheel and gimbal acceleration terms of one VSCMG, from the
/// back-substitution of a stage, for its derivatives.
#[derive(Clone, Copy, Debug, Default)]
enum VscmgAccelerationTerms {
    #[default]
    None,
    /// Balanced and simple: upstream `omegas`, `omegat`.
    Uncoupled { omega_s: f64, omega_t: f64 },
    /// Fully coupled: upstream `p`, `q`, `s`, `agamma`, `bgamma`, `cgamma`,
    /// `dgamma`.
    Coupled {
        p: Vector3<f64>,
        q: Vector3<f64>,
        s: f64,
        a_gamma: Vector3<f64>,
        b_gamma: Vector3<f64>,
        c_gamma: f64,
        d_gamma: f64,
    },
}

/// One VSCMG of a [`VscmgStateEffector`], with its own ports.
#[derive(Debug)]
pub struct Vscmg {
    pub config: VscmgConfig,
    /// The VSCMG's wheel and gimbal torque command (upstream `cmdsInMsg`
    /// entry).
    pub command_in_msg: Input<VscmgCommandMsg>,
    /// Upstream `vscmgOutMsgs` entry, written after every step.
    pub vscmg_out_msg: Output<VscmgConfigMsg>,
    /// Upstream `u_s_current` \[N m\]: applied wheel motor torque.
    pub u_s_current_nm: f64,
    /// Upstream `u_g_current` \[N m\]: applied gimbal motor torque.
    pub u_g_current_nm: f64,
    /// Upstream `Omega` \[rad/s\] of the last loaded state.
    pub omega_radps: f64,
    /// Upstream `gamma` \[rad\] of the last loaded state.
    pub gamma_rad: f64,
    /// Upstream `gammaDot` \[rad/s\] of the last loaded state.
    pub gamma_dot_radps: f64,
    /// Upstream `theta` \[rad\] of the last loaded state (jitter models).
    pub theta_rad: f64,
    acceleration: RefCell<VscmgAccelerationTerms>,
}

impl Vscmg {
    /// Upstream `initializeConfiguration`'s derived values for the current
    /// configuration.
    pub fn derived(&self) -> VscmgDerived {
        VscmgDerived::new(&self.config)
    }

    fn new(config: VscmgConfig) -> Self {
        Self {
            u_s_current_nm: config.u_s_current_nm,
            u_g_current_nm: config.u_g_current_nm,
            omega_radps: config.omega_init_radps,
            gamma_rad: config.gamma_init_rad,
            gamma_dot_radps: config.gamma_dot_init_radps,
            theta_rad: 0.0,
            config,
            command_in_msg: Input::default(),
            vscmg_out_msg: Output::default(),
            acceleration: RefCell::new(VscmgAccelerationTerms::default()),
        }
    }

    /// Upstream `ConfigureVSCMGRequests` for this VSCMG: limits, deadbands,
    /// the wheel speed limit and Coulomb friction (linear inside the band
    /// `ratio × max` when the ratio is positive) turn `command` into the
    /// applied torques, at the last loaded wheel speed and gimbal rate.
    ///
    /// # Panics
    ///
    /// On a non-finite command, an invalid linear friction band, or a
    /// non-finite applied torque, with upstream's message.
    pub fn configure_request(&mut self, command: VscmgCommandMsg) {
        let config = &self.config;
        let name = &config.name;
        if !command.u_s_cmd_nm.is_finite() || !command.u_g_cmd_nm.is_finite() {
            panic!("vscmgStateEffector: device {name} u_s_cmd and u_g_cmd must be finite.");
        }
        let mut u_s_cmd = command.u_s_cmd_nm;
        let mut u_g_cmd = command.u_g_cmd_nm;
        if config.u_s_max_nm > 0.0 {
            u_s_cmd = u_s_cmd.clamp(-config.u_s_max_nm, config.u_s_max_nm);
        }
        if config.u_g_max_nm > 0.0 {
            u_g_cmd = u_g_cmd.clamp(-config.u_g_max_nm, config.u_g_max_nm);
        }
        if u_s_cmd.abs() < config.u_s_min_nm {
            u_s_cmd = 0.0;
        }
        if u_g_cmd.abs() < config.u_g_min_nm {
            u_g_cmd = 0.0;
        }
        if self.omega_radps.abs() >= config.omega_max_radps && config.omega_max_radps > 0.0 {
            u_s_cmd = 0.0;
        }
        let u_s = coulomb_friction(
            u_s_cmd,
            self.omega_radps,
            config.u_s_f_nm,
            config.omega_max_radps,
            config.wheel_linear_friction_ratio,
            || {
                format!(
                    "vscmgStateEffector: device {name} Omega_max * wheelLinearFrictionRatio must be finite and positive when linear friction is enabled."
                )
            },
        );
        let u_g = coulomb_friction(
            u_g_cmd,
            self.gamma_dot_radps,
            config.u_g_f_nm,
            config.gamma_dot_max_radps,
            config.gimbal_linear_friction_ratio,
            || {
                format!(
                    "vscmgStateEffector: device {name} gammaDot_max * gimbalLinearFrictionRatio must be finite and positive when linear friction is enabled."
                )
            },
        );
        if !u_s.is_finite() || !u_g.is_finite() {
            panic!(
                "vscmgStateEffector: device {name} motor processing produced a non-finite applied torque."
            );
        }
        self.u_s_current_nm = u_s;
        self.u_g_current_nm = u_g;
    }

    /// Geometry and mass distribution at wheel speed `omega`, gimbal angle
    /// `gamma`, gimbal rate `gamma_dot` and wheel angle `theta`.
    fn kinematics(&self, omega: f64, gamma: f64, gamma_dot: f64, theta: f64) -> VscmgKinematics {
        let config = &self.config;
        let derived = VscmgDerived::new(config);
        let dcm_gg0 = euler3(gamma);
        let dcm_bg = derived.dcm_bg0 * dcm_gg0.transpose();
        let gs_hat: Vector3<f64> = dcm_bg.column(0).into();
        let gt_hat: Vector3<f64> = dcm_bg.column(1).into();
        let gg_hat: Vector3<f64> = derived.dcm_bg0.column(2).into();
        let dcm_wg0 = euler1(theta) * dcm_gg0;
        let dcm_bw = derived.dcm_bg0 * dcm_wg0.transpose();
        let w2_hat: Vector3<f64> = dcm_bw.column(1).into();
        let w3_hat: Vector3<f64> = dcm_bw.column(2).into();
        let [iw1, iw2, iw3] = config.iw_kg_m2;
        let [ig1, ig2, ig3] = config.ig_kg_m2;
        let rotate = |dcm: &Matrix3<f64>, inertia: Matrix3<f64>| dcm * inertia * dcm.transpose();

        if config.model != VscmgModel::JitterFullyCoupled {
            let ig_prime_g = Matrix3::new(
                0.0,
                gamma_dot * (ig1 - ig2),
                0.0,
                gamma_dot * (ig1 - ig2),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            );
            let iw_prime_w = Matrix3::new(
                0.0,
                gamma_dot * (iw1 - iw2),
                0.0,
                gamma_dot * (iw1 - iw2),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            );
            // Points W, G, Wc and Gc coincide.
            return VscmgKinematics {
                derived,
                omega,
                gamma_dot,
                theta,
                gs_hat,
                gt_hat,
                gg_hat,
                w2_hat,
                w3_hat,
                ig_b: rotate(
                    &dcm_bg,
                    Matrix3::from_diagonal(&Vector3::new(ig1, ig2, ig3)),
                ),
                ig_prime_b: rotate(&dcm_bg, ig_prime_g),
                iw_b: rotate(
                    &dcm_bg,
                    Matrix3::from_diagonal(&Vector3::new(iw1, iw2, iw3)),
                ),
                iw_prime_b: rotate(&dcm_bg, iw_prime_w),
                r_gcg_b: Vector3::zeros(),
                r_gcb_b: config.r_gb_b_m,
                r_prime_gcb_b: Vector3::zeros(),
                r_wcg_b: Vector3::zeros(),
                r_wcb_b: config.r_gb_b_m,
                r_prime_wcb_b: Vector3::zeros(),
            };
        }

        let [ig12, ig13, ig23] = derived.ig_products_kg_m2;
        let iw13 = derived.iw13_kg_m2;
        let ig_g = Matrix3::new(ig1, ig12, ig13, ig12, ig2, ig23, ig13, ig23, ig3);
        let ig_prime_g = gamma_dot
            * Matrix3::new(
                -2.0 * ig12,
                ig1 - ig2,
                -ig23,
                ig1 - ig2,
                2.0 * ig12,
                ig13,
                -ig23,
                ig13,
                0.0,
            );
        let iw_w = Matrix3::new(iw1, 0.0, iw13, 0.0, iw2, 0.0, iw13, 0.0, iw3);
        let (sin_theta, cos_theta) = theta.sin_cos();
        let ia = iw1 - iw2;
        let ib = iw3 - iw1;
        let ic = iw2 - iw3;
        let p01 = gamma_dot * ia * cos_theta - iw13 * omega;
        let p02 = gamma_dot * ib * sin_theta;
        let p12 = gamma_dot * iw13 * cos_theta + ic * omega;
        let iw_prime_w = Matrix3::new(
            2.0 * gamma_dot * iw13 * sin_theta,
            p01,
            p02,
            p01,
            0.0,
            p12,
            p02,
            p12,
            -2.0 * iw13 * gamma_dot * sin_theta,
        );
        let d = derived.d_m;
        let r_gcg_b = dcm_bg * config.r_gcg_g_m;
        let r_wcg_b = config.big_l_m * gg_hat + config.l_m * gs_hat + d * w2_hat;
        VscmgKinematics {
            derived,
            omega,
            gamma_dot,
            theta,
            gs_hat,
            gt_hat,
            gg_hat,
            w2_hat,
            w3_hat,
            ig_b: rotate(&dcm_bg, ig_g),
            ig_prime_b: rotate(&dcm_bg, ig_prime_g),
            iw_b: rotate(&dcm_bw, iw_w),
            iw_prime_b: rotate(&dcm_bw, iw_prime_w),
            r_gcg_b,
            r_gcb_b: config.r_gb_b_m + r_gcg_b,
            r_prime_gcb_b: gamma_dot * gg_hat.cross(&r_gcg_b),
            r_wcg_b,
            r_wcb_b: config.r_gb_b_m + r_wcg_b,
            r_prime_wcb_b: d * omega * w3_hat - d * gamma_dot * cos_theta * gs_hat
                + config.l_m * gamma_dot * gt_hat,
        }
    }

    /// Upstream `updateContributions` for this VSCMG, caching its
    /// acceleration terms.
    fn add_contributions(
        &self,
        k: &VscmgKinematics,
        omega_bn_b: Vector3<f64>,
        g_b: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        let config = &self.config;
        let omega_squared = k.omega * k.omega;
        let omega_wb_b = k.gamma_dot * k.gg_hat + k.omega * k.gs_hat;
        let omega_s = k.gs_hat.dot(&omega_bn_b);
        let omega_t = k.gt_hat.dot(&omega_bn_b);
        let [iw1, _, _] = config.iw_kg_m2;
        let u_s = self.u_s_current_nm;
        let u_g = self.u_g_current_nm;

        if config.model != VscmgModel::JitterFullyCoupled {
            let [iv1, iv2, iv3] = k.derived.iv_kg_m2;
            back_sub.matrix_d -=
                iv3 * k.gg_hat * k.gg_hat.transpose() + iw1 * k.gs_hat * k.gs_hat.transpose();
            back_sub.vec_rot -= (u_s - iw1 * omega_t * k.gamma_dot) * k.gs_hat
                + iw1 * k.omega * k.gamma_dot * k.gt_hat
                + (u_g + (iv1 - iv2) * omega_s * omega_t + iw1 * k.omega * omega_t) * k.gg_hat
                + omega_bn_b.cross(&(k.ig_b * k.gamma_dot * k.gg_hat))
                + omega_bn_b.cross(&(k.iw_b * omega_wb_b));
            if config.model == VscmgModel::JitterSimple {
                // Static imbalance force U_s Ω² and its torque, dynamic
                // imbalance torque U_d Ω².
                let force = config.static_imbalance_kg_m * omega_squared * k.w2_hat;
                back_sub.vec_trans += force;
                back_sub.vec_rot += config.r_gb_b_m.cross(&force)
                    + config.dynamic_imbalance_kg_m2 * omega_squared * k.w2_hat;
            }
            self.acceleration
                .replace(VscmgAccelerationTerms::Uncoupled { omega_s, omega_t });
            return;
        }

        let name = &config.name;
        let mass_w = config.mass_w_kg;
        let mass_g = config.mass_g_kg;
        let mass_v = k.derived.mass_v_kg;
        let rho_g = k.derived.rho_g;
        let rho_w = k.derived.rho_w;
        let d = k.derived.d_m;
        let l = config.l_m;
        let iw13 = k.derived.iw13_kg_m2;
        let (sin_theta, cos_theta) = k.theta.sin_cos();
        let omega = omega_bn_b;
        let gravity_torque_wheel_s = k.gs_hat.dot(&(d * k.w2_hat.cross(&(mass_w * g_b))));
        let gravity_torque_gimbal_g = k.gg_hat.dot(&k.r_gcg_b.cross(&(mass_g * g_b)));

        let d_squared = d * d;
        let gamma_dot_squared = k.gamma_dot * k.gamma_dot;
        let omega_gn_b = omega + k.gamma_dot * k.gg_hat;
        let omega_wn_b = omega_gn_b + k.omega * k.gs_hat;

        let r_vcg_b = 1.0 / mass_v * (mass_g * k.r_gcg_b + mass_w * k.r_wcg_b);
        let r_vcb_b = r_vcg_b + config.r_gb_b_m;
        let r_tilde_vcg_b = r_vcg_b.cross_matrix();
        let r_gcvc_b = k.r_gcg_b - r_vcg_b;
        let r_wcvc_b = k.r_wcg_b - r_vcg_b;
        let r_tilde_gcvc_b = r_gcvc_b.cross_matrix();
        let r_tilde_wcvc_b = r_wcvc_b.cross_matrix();
        let iv_pnt_vc_b = k.ig_b
            + mass_g * r_tilde_gcvc_b * r_tilde_gcvc_b.transpose()
            + k.iw_b
            + mass_w * r_tilde_wcvc_b * r_tilde_wcvc_b.transpose();
        let r_prime_gcg_b = k.r_prime_gcb_b;
        let r_prime_wcg_b = k.r_prime_wcb_b;
        let r_prime_vcg_b = rho_g * r_prime_gcg_b + rho_w * r_prime_wcg_b;
        let r_prime_vcb_b = r_prime_vcg_b;
        let r_prime_gcvc_b = r_prime_gcg_b - r_prime_vcg_b;
        let r_prime_wcvc_b = r_prime_wcg_b - r_prime_vcg_b;

        let p_mat = mass_w * rho_g * r_tilde_wcvc_b - mass_g * rho_w * r_tilde_gcvc_b
            + mass_w * r_tilde_vcg_b;
        let q_mat = mass_g * rho_w * r_tilde_gcvc_b - mass_w * rho_g * r_tilde_wcvc_b
            + mass_g * r_tilde_vcg_b;

        let e_gamma = k.gg_hat.dot(
            &(k.ig_b * k.gg_hat
                + k.iw_b * k.gg_hat
                + p_mat * (l * k.gt_hat - d * cos_theta * k.gs_hat)
                + q_mat * k.gg_hat.cross(&k.r_gcg_b)),
        );
        require_divisor(e_gamma, "egamma")
            .unwrap_or_else(|message| panic!("vscmgStateEffector: device {name} {message}."));
        let a_gamma = 1.0 / e_gamma * mass_v * r_vcg_b.cross(&k.gg_hat);
        let b_gamma = -1.0 / e_gamma
            * (iv_pnt_vc_b * k.gg_hat - mass_v * r_vcb_b.cross(&r_vcg_b.cross(&k.gg_hat)));
        let c_gamma = -1.0 / e_gamma
            * (k.gg_hat.dot(&(k.iw_b * k.gs_hat)) + d * k.gg_hat.dot(&(p_mat * k.w3_hat)));
        let wheel_accel_terms =
            (2.0 * d * k.gamma_dot * k.omega * sin_theta - l * gamma_dot_squared) * k.gs_hat
                - d * gamma_dot_squared * cos_theta * k.gt_hat
                - d * omega_squared * k.w2_hat;
        let d_gamma = -1.0 / e_gamma
            * k.gg_hat.dot(
                &(k.gamma_dot * q_mat * k.gg_hat.cross(&r_prime_gcg_b)
                    + p_mat * wheel_accel_terms
                    + k.ig_prime_b * omega_gn_b
                    + omega.cross(&(k.ig_b * omega_gn_b))
                    + k.iw_b * k.omega * k.gamma_dot * k.gt_hat
                    + k.iw_prime_b * omega_wn_b
                    + omega.cross(&(k.iw_b * omega_wn_b))
                    + mass_g
                        * r_gcvc_b.cross(
                            &(2.0 * omega.cross(&r_prime_gcvc_b)
                                + omega.cross(&omega.cross(&r_gcvc_b))),
                        )
                    + mass_w
                        * r_wcvc_b.cross(
                            &(2.0 * omega.cross(&r_prime_wcvc_b)
                                + omega.cross(&omega.cross(&r_wcvc_b))),
                        )
                    + mass_v
                        * r_vcg_b.cross(
                            &(2.0 * omega.cross(&r_prime_vcb_b)
                                + omega.cross(&omega.cross(&r_vcb_b))),
                        )),
            )
            + 1.0 / e_gamma * (u_g + gravity_torque_gimbal_g);

        let e_omega = iw1 + mass_w * d_squared;
        require_divisor(e_omega, "eOmega")
            .unwrap_or_else(|message| panic!("vscmgStateEffector: device {name} {message}."));
        let a_omega = -1.0 / e_omega * mass_w * d * k.w3_hat;
        let b_omega = -1.0 / e_omega
            * (k.iw_b * k.gs_hat - mass_w * d * k.r_wcb_b.cross(&k.w2_hat.cross(&k.gs_hat)));
        let c_omega = -1.0 / e_omega * (iw13 * cos_theta - mass_w * d * l * sin_theta);
        let d_omega = -1.0 / e_omega
            * (k.gs_hat.dot(&(k.iw_prime_b * omega_wn_b))
                + k.gs_hat.dot(&omega.cross(&(k.iw_b * omega_wn_b)))
                + mass_w
                    * d
                    * k.gs_hat.dot(&k.w2_hat.cross(
                        &(2.0 * omega.cross(&k.r_prime_wcb_b)
                            + omega.cross(&omega.cross(&k.r_wcb_b))),
                    )))
            + (1.0 / e_omega)
                * (iw13 * sin_theta * k.omega * k.gamma_dot
                    - mass_w * d_squared * gamma_dot_squared * cos_theta * sin_theta
                    + u_s
                    + gravity_torque_wheel_s);

        let coupling_denominator = 1.0 - c_omega * c_gamma;
        require_divisor(coupling_denominator, "1 - cOmega*cgamma")
            .unwrap_or_else(|message| panic!("vscmgStateEffector: device {name} {message}."));
        let p = (a_omega + c_omega * a_gamma) / coupling_denominator;
        let q = (b_omega + c_omega * b_gamma) / coupling_denominator;
        let s = (d_omega + c_omega * d_gamma) / coupling_denominator;

        let ur = mass_g * k.gg_hat.cross(&k.r_gcg_b) - mass_w * d * cos_theta * k.gs_hat
            + mass_w * l * k.gt_hat;
        let vr = mass_w * d * k.w3_hat;
        let kr =
            mass_g * k.gamma_dot * k.gg_hat.cross(&k.r_prime_gcb_b) + mass_w * wheel_accel_terms;
        let u_omega = k.ig_b * k.gg_hat
            + mass_g * k.r_gcb_b.cross(&k.gg_hat.cross(&k.r_gcg_b))
            + k.iw_b * k.gg_hat
            + mass_w * k.r_wcb_b.cross(&(l * k.gt_hat - d * cos_theta * k.gs_hat));
        let v_omega = k.iw_b * k.gs_hat + mass_w * d * k.r_wcb_b.cross(&k.w3_hat);
        let k_omega = k.ig_prime_b * k.gamma_dot * k.gg_hat
            + omega.cross(&(k.ig_b * k.gamma_dot * k.gg_hat))
            + mass_g * omega.cross(&k.r_gcb_b.cross(&k.r_prime_gcb_b))
            + mass_g * k.gamma_dot * k.r_gcb_b.cross(&k.gg_hat.cross(&r_prime_gcg_b))
            + k.iw_b * k.omega * k.gamma_dot * k.gt_hat
            + k.iw_prime_b * omega_wb_b
            + omega.cross(&(k.iw_b * omega_wb_b))
            + mass_w * omega.cross(&k.r_wcb_b.cross(&k.r_prime_wcb_b))
            + mass_w * k.r_wcb_b.cross(&wheel_accel_terms);

        back_sub.matrix_a += ur * a_gamma.transpose() + (vr + ur * c_gamma) * p.transpose();
        back_sub.matrix_b += ur * b_gamma.transpose() + (vr + ur * c_gamma) * q.transpose();
        back_sub.matrix_c +=
            u_omega * a_gamma.transpose() + (v_omega + u_omega * c_gamma) * p.transpose();
        back_sub.matrix_d +=
            u_omega * b_gamma.transpose() + (v_omega + u_omega * c_gamma) * q.transpose();
        back_sub.vec_trans -= kr + ur * d_gamma + (vr + ur * c_gamma) * s;
        back_sub.vec_rot -= k_omega + u_omega * d_gamma + (v_omega + u_omega * c_gamma) * s;
        self.acceleration.replace(VscmgAccelerationTerms::Coupled {
            p,
            q,
            s,
            a_gamma,
            b_gamma,
            c_gamma,
            d_gamma,
        });
    }

    /// Upstream `computeDerivatives` for this VSCMG: `(Ω̇, γ̈)`.
    fn accelerations(
        &self,
        k: &VscmgKinematics,
        body_trans_accel_b: Vector3<f64>,
        body_omega_dot_b: Vector3<f64>,
    ) -> (f64, f64) {
        match *self.acceleration.borrow() {
            VscmgAccelerationTerms::Uncoupled { omega_s, omega_t } => {
                let iw1 = self.config.iw_kg_m2[0];
                let [iv1, iv2, iv3] = k.derived.iv_kg_m2;
                let omega_dot = -omega_t * k.gamma_dot - k.gs_hat.dot(&body_omega_dot_b)
                    + self.u_s_current_nm / iw1;
                let gamma_ddot = 1.0 / iv3
                    * (self.u_g_current_nm
                        + (iv1 - iv2) * omega_s * omega_t
                        + iw1 * k.omega * omega_t
                        - iv3 * k.gg_hat.dot(&body_omega_dot_b));
                (omega_dot, gamma_ddot)
            }
            VscmgAccelerationTerms::Coupled {
                p,
                q,
                s,
                a_gamma,
                b_gamma,
                c_gamma,
                d_gamma,
            } => {
                let omega_dot = p.dot(&body_trans_accel_b) + q.dot(&body_omega_dot_b) + s;
                let gamma_ddot = a_gamma.dot(&body_trans_accel_b)
                    + b_gamma.dot(&body_omega_dot_b)
                    + c_gamma * omega_dot
                    + d_gamma;
                (omega_dot, gamma_ddot)
            }
            VscmgAccelerationTerms::None => panic!(
                "vscmgStateEffector: derivatives of device {} requested before its contributions",
                self.config.name
            ),
        }
    }

    /// Upstream `WriteOutputMessages`'s per-device message.
    fn output(&self) -> VscmgConfigMsg {
        let config = &self.config;
        VscmgConfigMsg {
            model: config.model,
            u_s_current_nm: self.u_s_current_nm,
            u_s_max_nm: config.u_s_max_nm,
            u_s_min_nm: config.u_s_min_nm,
            u_s_f_nm: config.u_s_f_nm,
            u_g_current_nm: self.u_g_current_nm,
            u_g_max_nm: config.u_g_max_nm,
            u_g_min_nm: config.u_g_min_nm,
            u_g_f_nm: config.u_g_f_nm,
            theta_rad: if config.is_jitter() {
                self.theta_rad
            } else {
                0.0
            },
            omega_radps: self.omega_radps,
            omega_max_radps: config.omega_max_radps,
            gamma_rad: self.gamma_rad,
            gamma_dot_radps: self.gamma_dot_radps,
            gamma_dot_max_radps: config.gamma_dot_max_radps,
            iw1_kg_m2: config.iw_kg_m2[0],
            static_imbalance_kg_m: config.static_imbalance_kg_m,
            dynamic_imbalance_kg_m2: config.dynamic_imbalance_kg_m2,
        }
    }
}

/// Upstream's Coulomb friction of `ConfigureVSCMGRequests`: friction `f`
/// opposing `rate`, linear in `rate` inside `|rate| <= ratio × max` when
/// `ratio > 0`.
fn coulomb_friction(
    command: f64,
    rate: f64,
    friction: f64,
    max_rate: f64,
    ratio: f64,
    band_error: impl FnOnce() -> String,
) -> f64 {
    let critical = if ratio > 0.0 {
        let critical = max_rate * ratio;
        if !critical.is_finite() || critical <= 0.0 {
            panic!("{}", band_error());
        }
        critical
    } else {
        0.0
    };
    if rate > critical {
        command - friction
    } else if rate < -critical {
        command + friction
    } else if ratio > 0.0 {
        command - friction * rate / critical
    } else {
        command
    }
}

/// VSCMG cluster state effector (upstream `VSCMGStateEffector`).
#[derive(Debug)]
pub struct VscmgStateEffector {
    pub name: String,
    /// Upstream `speedOutMsg`: every VSCMG's wheel speed, gimbal angle and
    /// gimbal rate, written after every step.
    pub speed_out_msg: Output<VscmgSpeedMsg>,
    vscmgs: Vec<Vscmg>,
}

/// Indices of one VSCMG's states.
#[derive(Clone, Copy)]
struct StateIndices {
    omega: usize,
    gamma: usize,
    gamma_dot: usize,
    theta: Option<usize>,
}

impl VscmgStateEffector {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            speed_out_msg: Output::default(),
            vscmgs: Vec::new(),
        }
    }

    /// Upstream `AddVSCMG`: registers one VSCMG and returns its ports.
    pub fn add_vscmg(&mut self, config: VscmgConfig) -> &mut Vscmg {
        self.vscmgs.push(Vscmg::new(config));
        self.vscmgs
            .last_mut()
            .expect("registered VSCMG should be available")
    }

    /// The VSCMGs in cluster order.
    pub fn vscmgs(&self) -> &[Vscmg] {
        &self.vscmgs
    }

    /// Mutable VSCMG access for wiring and configuration.
    pub fn vscmgs_mut(&mut self) -> &mut [Vscmg] {
        &mut self.vscmgs
    }

    fn state_indices(&self) -> Vec<StateIndices> {
        let n = self.vscmgs.len();
        let mut next_theta = 3 * n;
        self.vscmgs
            .iter()
            .enumerate()
            .map(|(index, vscmg)| StateIndices {
                omega: index,
                gamma: n + index,
                gamma_dot: 2 * n + index,
                theta: vscmg.config.is_jitter().then(|| {
                    next_theta += 1;
                    next_theta - 1
                }),
            })
            .collect()
    }

    fn kinematics(&self, state: &[f64]) -> Vec<VscmgKinematics> {
        assert_eq!(state.len(), self.state_len(), "VSCMG state length mismatch");
        self.vscmgs
            .iter()
            .zip(self.state_indices())
            .map(|(vscmg, index)| {
                vscmg.kinematics(
                    state[index.omega],
                    state[index.gamma],
                    state[index.gamma_dot],
                    index.theta.map_or(vscmg.theta_rad, |theta| state[theta]),
                )
            })
            .collect()
    }

    /// Upstream `initializeConfiguration` (run by upstream `Reset`): the
    /// device count fits [`VscmgSpeedMsg`] and every VSCMG passes
    /// [`VscmgConfig::validate`]. Returns upstream's message for the first
    /// failure.
    pub fn validate(&self) -> Result<(), String> {
        if self.vscmgs.len() > MAX_EFF_COUNT {
            return Err(format!(
                "vscmgStateEffector: device count {} exceeds MAX_EFF_CNT ({MAX_EFF_COUNT}).",
                self.vscmgs.len()
            ));
        }
        for vscmg in &self.vscmgs {
            vscmg.config.validate().map_err(|message| {
                format!(
                    "vscmgStateEffector: device {} {message}.",
                    vscmg.config.name
                )
            })?;
        }
        Ok(())
    }

    fn assert_valid(&self) {
        if let Err(message) = self.validate() {
            panic!("{message}");
        }
    }
}

impl StateEffector for VscmgStateEffector {
    fn name(&self) -> &str {
        &self.name
    }

    fn state_len(&self) -> usize {
        3 * self.vscmgs.len()
            + self
                .vscmgs
                .iter()
                .filter(|vscmg| vscmg.config.is_jitter())
                .count()
    }

    /// Validates the configuration (upstream `registerStates`) and returns
    /// the initial wheel speeds, gimbal angles and rates, and zero wheel
    /// angles.
    fn initial_state(&self) -> Vec<f64> {
        self.assert_valid();
        let mut state = vec![0.0; self.state_len()];
        for (vscmg, index) in self.vscmgs.iter().zip(self.state_indices()) {
            state[index.omega] = vscmg.config.omega_init_radps;
            state[index.gamma] = vscmg.config.gamma_init_rad;
            state[index.gamma_dot] = vscmg.config.gamma_dot_init_radps;
        }
        state
    }

    fn load_state(&mut self, state: &[f64]) {
        assert_eq!(state.len(), self.state_len(), "VSCMG state length mismatch");
        let indices = self.state_indices();
        for (vscmg, index) in self.vscmgs.iter_mut().zip(indices) {
            vscmg.omega_radps = state[index.omega];
            vscmg.gamma_rad = state[index.gamma];
            vscmg.gamma_dot_radps = state[index.gamma_dot];
            if let Some(theta) = index.theta {
                vscmg.theta_rad = state[theta];
            }
        }
    }

    /// Upstream `UpdateState`'s `ReadInputs` and `ConfigureVSCMGRequests`:
    /// each VSCMG with a connected command input applies its command.
    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {
        self.assert_valid();
        for vscmg in &mut self.vscmgs {
            if vscmg.command_in_msg.is_connected() {
                let command = vscmg.command_in_msg.read();
                vscmg.configure_request(command);
            }
        }
    }

    /// Upstream `updateContributions`.
    fn update_contributions(
        &self,
        effector_state: &[f64],
        hub_state: &SpacecraftStateMsg,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        let body_omega_radps = hub_state.omega_radps;
        for (vscmg, k) in self.vscmgs.iter().zip(self.kinematics(effector_state)) {
            vscmg.add_contributions(&k, body_omega_radps, gravity_body_mps2, back_sub);
        }
    }

    /// Upstream `computeDerivatives`.
    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        let mut derivatives = vec![0.0; self.state_len()];
        for ((vscmg, k), index) in self
            .vscmgs
            .iter()
            .zip(self.kinematics(effector_state))
            .zip(self.state_indices())
        {
            let (omega_dot, gamma_ddot) =
                vscmg.accelerations(&k, body_trans_accel_mps2, body_omega_dot_radps2);
            derivatives[index.omega] = omega_dot;
            derivatives[index.gamma] = k.gamma_dot;
            derivatives[index.gamma_dot] = gamma_ddot;
            if let Some(theta) = index.theta {
                derivatives[theta] = k.omega;
            }
        }
        derivatives
    }

    /// Upstream `updateEffectorMassProps`.
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        let mut props = StateEffectorMassProps::default();
        let mut first_moment = Vector3::zeros();
        let mut first_moment_prime = Vector3::zeros();
        for (vscmg, k) in self.vscmgs.iter().zip(self.kinematics(effector_state)) {
            let mass_v = k.derived.mass_v_kg;
            props.mass_kg += mass_v;
            if vscmg.config.model != VscmgModel::JitterFullyCoupled {
                let r_tilde = k.r_wcb_b.cross_matrix();
                first_moment += mass_v * vscmg.config.r_gb_b_m;
                props.inertia_about_point_b_body_kg_m2 +=
                    k.iw_b + k.ig_b + mass_v * r_tilde * r_tilde.transpose();
                props.inertia_about_point_b_body_prime_kg_m2ps += k.iw_prime_b + k.ig_prime_b;
            } else {
                let mass_g = vscmg.config.mass_g_kg;
                let mass_w = vscmg.config.mass_w_kg;
                let rg_tilde = k.r_gcb_b.cross_matrix();
                let rw_tilde = k.r_wcb_b.cross_matrix();
                let rg_prime_tilde = k.r_prime_gcb_b.cross_matrix();
                let rw_prime_tilde = k.r_prime_wcb_b.cross_matrix();
                first_moment += mass_g * k.r_gcb_b + mass_w * k.r_wcb_b;
                first_moment_prime += mass_g * k.r_prime_gcb_b + mass_w * k.r_prime_wcb_b;
                props.inertia_about_point_b_body_kg_m2 += k.iw_b
                    + k.ig_b
                    + mass_g * rg_tilde * rg_tilde.transpose()
                    + mass_w * rw_tilde * rw_tilde.transpose();
                props.inertia_about_point_b_body_prime_kg_m2ps += k.ig_prime_b
                    + mass_g * rg_prime_tilde * rg_tilde.transpose()
                    + mass_g * rg_tilde * rg_prime_tilde.transpose()
                    + k.iw_prime_b
                    + mass_w * rw_prime_tilde * rw_tilde.transpose()
                    + mass_w * rw_tilde * rw_prime_tilde.transpose();
            }
        }
        if props.mass_kg > 0.0 {
            props.center_of_mass_body_m = first_moment / props.mass_kg;
            props.center_of_mass_prime_body_mps = first_moment_prime / props.mass_kg;
        }
        props
    }

    /// Upstream `updateEnergyMomContributions` (angular momentum about B).
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        let omega = body_omega_radps;
        let mut momentum = Vector3::zeros();
        for (vscmg, k) in self.vscmgs.iter().zip(self.kinematics(effector_state)) {
            let omega_gn = omega + k.gamma_dot * k.gg_hat;
            let omega_wn = omega_gn + k.omega * k.gs_hat;
            if vscmg.config.model != VscmgModel::JitterFullyCoupled {
                let r = vscmg.config.r_gb_b_m;
                let r_dot = omega.cross(&r);
                momentum +=
                    k.iw_b * omega_wn + k.ig_b * omega_gn + k.derived.mass_v_kg * r.cross(&r_dot);
            } else {
                let r_dot_wc = k.r_prime_wcb_b + omega.cross(&k.r_wcb_b);
                let r_dot_gc = k.r_prime_gcb_b + omega.cross(&k.r_gcb_b);
                momentum += k.iw_b * omega_wn
                    + vscmg.config.mass_w_kg * k.r_wcb_b.cross(&r_dot_wc)
                    + k.ig_b * omega_gn
                    + vscmg.config.mass_g_kg * k.r_gcb_b.cross(&r_dot_gc);
            }
        }
        momentum
    }

    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        let omega = body_omega_radps;
        let mut energy = 0.0;
        for (vscmg, k) in self.vscmgs.iter().zip(self.kinematics(effector_state)) {
            let omega_gn = omega + k.gamma_dot * k.gg_hat;
            let omega_wn = omega_gn + k.omega * k.gs_hat;
            if vscmg.config.model != VscmgModel::JitterFullyCoupled {
                let r_dot = omega.cross(&vscmg.config.r_gb_b_m);
                energy += 0.5 * omega_wn.dot(&(k.iw_b * omega_wn))
                    + 0.5 * omega_gn.dot(&(k.ig_b * omega_gn))
                    + 0.5 * k.derived.mass_v_kg * r_dot.dot(&r_dot);
            } else {
                let r_dot_wc = k.r_prime_wcb_b + omega.cross(&k.r_wcb_b);
                let r_dot_gc = k.r_prime_gcb_b + omega.cross(&k.r_gcb_b);
                energy += 0.5 * omega_wn.dot(&(k.iw_b * omega_wn))
                    + 0.5 * vscmg.config.mass_w_kg * r_dot_wc.dot(&r_dot_wc)
                    + 0.5 * omega_gn.dot(&(k.ig_b * omega_gn))
                    + 0.5 * vscmg.config.mass_g_kg * r_dot_gc.dot(&r_dot_gc);
            }
        }
        energy
    }

    /// Upstream `WriteOutputMessages`.
    fn write_outputs(&mut self, _current_sim_nanos: u64, _hub_state: &SpacecraftStateMsg) {
        let mut speeds = VscmgSpeedMsg::default();
        for (index, vscmg) in self.vscmgs.iter().enumerate().take(MAX_EFF_COUNT) {
            speeds.wheel_speeds_radps[index] = vscmg.omega_radps;
            speeds.gimbal_angles_rad[index] = vscmg.gamma_rad;
            speeds.gimbal_rates_radps[index] = vscmg.gamma_dot_radps;
        }
        for vscmg in &self.vscmgs {
            vscmg.vscmg_out_msg.write(vscmg.output());
        }
        self.speed_out_msg.write(speeds);
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_VSCMGStateEffector_integrated.py`,
    //! `test_VSCMGStateEffector_ConfigureVSCMGRequests.py` and
    //! `test_vscmgInitialization.py`, and an end-to-end test with the FSW
    //! [`VscmgGimbalRateServo`].
    //!
    //! Upstream's initialization errors come from `Reset` or from
    //! `InitializeSimulation`; here they come from
    //! [`VscmgStateEffector::validate`] (the `"reset"` path) and from the
    //! spacecraft's initialization (the `"attachment"` path), as panics. The
    //! coupled divisors, which upstream checks in the equations of motion
    //! that `InitializeSimulation` evaluates, panic at the first step.
    //!
    //! Not ported: the `VSCMGModel = 99` case (an enum cannot hold it), the
    //! scheduler-order variants of `test_attachment_matches_both_scheduler_orders`
    //! (the effector has no schedule of its own; its connected and
    //! unconnected command paths are compared instead), and the pending-command
    //! clearing of `test_repeated_reset_normalizes_axes_and_preserves_states`
    //! (commands are read when applied, never buffered).

    use super::{VscmgConfig, VscmgStateEffector};
    use crate::dynamics::state_effector_test_support::{
        GRAVITY_CASE_R_CN_N_M, GRAVITY_CASE_V_CN_N_MPS, HubSetup, add_point_mass_earth,
        assert_relative_equal, run_inspecting_every_step, run_logging_every_step,
        spacecraft_with_effectors,
    };
    use crate::fsw_algorithms::effector_interfaces::vscmg_gimbal_rate_servo::{
        VscmgGimbalRateServo, VscmgGimbalRateServoConfig,
    };
    use crate::messages::{
        MAX_EFF_COUNT, Output, VscmgArrayConfigMsg, VscmgCommandMsg, VscmgConfigElementMsg,
        VscmgModel, VscmgReferenceStatesMsg,
    };
    use crate::sensors::navigation::SimpleNavigation;
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, StateEffector};
    use hifitime::Epoch;
    use nalgebra::{DMatrix, DVector, Matrix3, Vector3};
    use std::f64::consts::PI;
    use std::panic::{AssertUnwindSafe, catch_unwind};

    /// Upstream `macros.RPM` \[rad/s per rpm\].
    const RPM: f64 = 2.0 * PI / 60.0;

    /// Upstream `defaultVSCMG` of `test_VSCMGStateEffector_integrated.py`.
    fn integrated_default(model: VscmgModel) -> VscmgConfig {
        let omega_max = 6000.0 * RPM;
        let iw1 = 100.0 / omega_max;
        VscmgConfig {
            u_s_max_nm: -1.0,
            u_s_min_nm: -1.0,
            wheel_linear_friction_ratio: -1.0,
            u_g_max_nm: -1.0,
            u_g_min_nm: -1.0,
            gimbal_linear_friction_ratio: -1.0,
            omega_max_radps: omega_max,
            gamma_dot_max_radps: -1.0,
            iw_kg_m2: [iw1, 0.5 * iw1, 0.5 * iw1],
            ig_kg_m2: [0.1, 0.2, 0.3],
            static_imbalance_kg_m: 4.8e-06 * 1e4,
            dynamic_imbalance_kg_m2: 1.54e-06 * 1e4,
            l_m: 0.01,
            big_l_m: 0.1,
            r_gcg_g_m: Vector3::new(0.0001, -0.02, 0.1),
            mass_w_kg: 6.0,
            mass_g_kg: 6.0,
            ..VscmgConfig::new("vscmg", model)
        }
    }

    /// The three VSCMGs of upstream `VSCMGIntegratedTest`.
    fn integrated_vscmgs(model: VscmgModel) -> [VscmgConfig; 3] {
        let ang = 54.75 * PI / 180.0;
        let gg1 = Vector3::new(ang.cos(), 0.0, ang.sin());
        let gg2 = Vector3::new(-ang.cos(), 0.0, ang.sin());
        [
            VscmgConfig {
                gs_hat0_b: Vector3::x(),
                gt_hat0_b: Vector3::y(),
                gg_hat_b: Vector3::z(),
                omega_init_radps: 2000.0 * RPM,
                gamma_dot_init_radps: 0.06,
                r_gb_b_m: Vector3::new(0.1, 0.002, -0.02),
                ..integrated_default(model)
            },
            VscmgConfig {
                gs_hat0_b: Vector3::y(),
                gg_hat_b: gg1,
                gt_hat0_b: gg1.cross(&Vector3::y()),
                omega_init_radps: 350.0 * RPM,
                gamma_dot_init_radps: 0.011,
                r_gb_b_m: Vector3::new(0.0, -0.05, 0.0),
                ..integrated_default(model)
            },
            VscmgConfig {
                gs_hat0_b: -Vector3::y(),
                gg_hat_b: gg2,
                gt_hat0_b: gg2.cross(&-Vector3::y()),
                omega_init_radps: -900.0 * RPM,
                gamma_dot_init_radps: -0.003,
                r_gb_b_m: Vector3::new(-0.1, 0.05, 0.05),
                ..integrated_default(model)
            },
        ]
    }

    /// `numpy.polyfit` of degree `order` evaluated at `t` (least squares on
    /// the column-scaled Vandermonde matrix, as numpy).
    fn polynomial_fit(t: &[f64], y: &[f64], order: usize) -> Vec<f64> {
        let mut vandermonde = DMatrix::from_fn(t.len(), order + 1, |row, column| {
            t[row].powi((order - column) as i32)
        });
        let scales: Vec<f64> = vandermonde
            .column_iter()
            .map(|column| column.norm())
            .collect();
        for (column, scale) in scales.iter().enumerate() {
            vandermonde.column_mut(column).unscale_mut(*scale);
        }
        let coefficients = vandermonde
            .clone()
            .svd(true, true)
            .solve(&DVector::from_column_slice(y), 1e-300)
            .expect("least-squares fit");
        let coefficients: Vec<f64> = coefficients
            .iter()
            .zip(&scales)
            .map(|(coefficient, scale)| coefficient / scale)
            .collect();
        t.iter()
            .map(|t| {
                coefficients
                    .iter()
                    .fold(0.0, |value, coefficient| value * t + coefficient)
            })
            .collect()
    }

    /// Upstream `computeFFT` and `findPeaks`: the frequencies \[Hz\] of the
    /// `count` largest interior peaks of `y`'s one-sided spectrum, ascending.
    fn spectral_peaks_hz(y: &[f64], dt: f64, count: usize) -> Vec<f64> {
        let n = y.len();
        let magnitude: Vec<f64> = (0..n / 2)
            .map(|k| {
                let (re, im) = y
                    .iter()
                    .enumerate()
                    .fold((0.0, 0.0), |(re, im), (j, value)| {
                        let angle = -2.0 * PI * (j * k) as f64 / n as f64;
                        (re + value * angle.cos(), im + value * angle.sin())
                    });
                (re * re + im * im).sqrt() / n as f64
            })
            .collect();
        let mut peaks: Vec<usize> = (1..magnitude.len() - 1)
            .filter(|&k| magnitude[k] > magnitude[k - 1] && magnitude[k] > magnitude[k + 1])
            .collect();
        let mut values: Vec<f64> = peaks.iter().map(|&k| magnitude[k]).collect();
        values.sort_by(|a, b| a.partial_cmp(b).expect("finite spectrum"));
        let threshold = values[values.len() - count];
        peaks.retain(|&k| magnitude[k] >= threshold);
        let duration = n as f64 * dt;
        peaks.into_iter().map(|k| k as f64 / duration).collect()
    }

    /// Upstream `VSCMGIntegratedTest`.
    fn vscmg_integrated_test(test_case: &str) {
        let model = match test_case {
            "BalancedWheels" => VscmgModel::BalancedWheels,
            "JitterSimple" => VscmgModel::JitterSimple,
            _ => VscmgModel::JitterFullyCoupled,
        };
        let fully_coupled_no_gravity = test_case == "JitterFullyCoupled";
        let (dt_nanos, steps) = if model == VscmgModel::JitterFullyCoupled {
            (10_000_u64, 1_000)
        } else {
            (1_000_000_u64, 1_000)
        };
        let mut vscmgs: Vec<VscmgConfig> = integrated_vscmgs(model).into();
        if fully_coupled_no_gravity {
            vscmgs.remove(1);
        }
        let (wheel_torque, gimbal_torque) =
            if test_case == "BalancedWheels" || fully_coupled_no_gravity {
                ([0.0; 3], [0.0; 3])
            } else {
                ([0.001, 0.005, -0.009], [0.008, -0.0015, -0.006])
            };
        let wheel_speeds_true: Vec<f64> = {
            let mut speeds: Vec<f64> = vscmgs
                .iter()
                .map(|vscmg| (vscmg.omega_init_radps / RPM).abs())
                .collect();
            speeds.sort_by(|a, b| a.partial_cmp(b).expect("finite speeds"));
            speeds
        };

        let mut effector = VscmgStateEffector::new("VSCMGs");
        let mut commands = Vec::new();
        for (index, config) in vscmgs.into_iter().enumerate() {
            let command = Output::new(VscmgCommandMsg {
                u_s_cmd_nm: wheel_torque[index],
                u_g_cmd_nm: gimbal_torque[index],
            });
            command.connect_to(&mut effector.add_vscmg(config).command_in_msg);
            commands.push(command);
        }

        let hub = HubSetup {
            mass_kg: 750.0,
            center_of_mass_body_m: Vector3::new(-0.0002, 0.0001, 0.1),
            inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
            r_cn_n_m: if fully_coupled_no_gravity {
                Vector3::new(0.1, -0.2, 0.3)
            } else {
                Vector3::from(GRAVITY_CASE_R_CN_N_M)
            },
            v_cn_n_mps: if fully_coupled_no_gravity {
                Vector3::new(-0.4, 0.5, -0.8)
            } else {
                Vector3::from(GRAVITY_CASE_V_CN_N_MPS)
            },
            sigma_bn: Vector3::zeros(),
            omega_bn_b_radps: Vector3::new(0.08, 0.01, 0.0),
        };
        let mut spacecraft = spacecraft_with_effectors(&hub, dt_nanos, vec![Box::new(effector)]);
        if !fully_coupled_no_gravity {
            add_point_mass_earth(&mut spacecraft);
        }
        let state_out = spacecraft.state_out.clone();
        let mut sigmas = Vec::new();
        let log = run_logging_every_step(&mut spacecraft, steps, || {
            sigmas.push(state_out.read().sigma_bn);
        });

        let final_state = state_out.read();
        let (true_pos, true_sigma) = match test_case {
            "BalancedWheels" => (
                [-4025537.664298894, 7487128.570444949, 5249339.643828076],
                [0.0185829763256, 0.00212563436704, -0.00118728497031],
            ),
            "JitterSimple" => (
                [-4025537.659558947, 7487128.570662447, 5249339.653774626],
                [
                    0.018774477186285467,
                    0.0018376842577357564,
                    -0.00023633044221463834,
                ],
            ),
            "JitterFullyCoupled" => (
                [0.0970572658434, -0.195562924079, 0.191874379545],
                [0.000201909373901, 2.9217809421e-05, 4.00231302121e-06],
            ),
            _ => (
                [-4020390.68802, 7490532.37502, 5248309.52745],
                [0.000201662012765, 2.92123940252e-05, 4.15606551702e-06],
            ),
        };
        assert_relative_equal(
            final_state.position_m.as_slice(),
            &true_pos,
            1e-7,
            "position",
        );
        assert_relative_equal(
            final_state.sigma_bn.as_slice(),
            &true_sigma,
            1e-7,
            "attitude",
        );

        if test_case == "JitterSimple" {
            // The jitter shows in the attitude at the wheel speeds.
            let dt = dt_nanos as f64 * 1e-9;
            let times: Vec<f64> = (0..sigmas.len()).map(|k| k as f64 * dt).collect();
            let theta: Vec<f64> = sigmas
                .iter()
                .map(|sigma| 4.0 * sigma.norm().atan())
                .collect();
            let fit = polynomial_fit(&times, &theta, 11);
            let residual: Vec<f64> = theta.iter().zip(&fit).map(|(t, f)| t - f).collect();
            let peaks_rpm: Vec<f64> = spectral_peaks_hz(&residual, dt, 3)
                .into_iter()
                .map(|frequency| frequency * 60.0)
                .collect();
            assert_eq!(peaks_rpm.len(), 3, "{peaks_rpm:?}");
            for (data, truth) in peaks_rpm.iter().zip(&wheel_speeds_true) {
                assert!(
                    ((data - truth) / truth).abs() < 0.09,
                    "jitter at {data} rpm, wheel at {truth} rpm"
                );
            }
        }

        if test_case == "BalancedWheels" || fully_coupled_no_gravity {
            let (first, last) = (&log[0], &log[log.len() - 1]);
            let accuracy = 1e-10;
            assert_relative_equal(
                last.orbital_angular_momentum_inertial_kg_m2ps.as_slice(),
                first.orbital_angular_momentum_inertial_kg_m2ps.as_slice(),
                accuracy,
                "orbital angular momentum",
            );
            assert_relative_equal(
                last.rotational_angular_momentum_inertial_kg_m2ps.as_slice(),
                first
                    .rotational_angular_momentum_inertial_kg_m2ps
                    .as_slice(),
                accuracy,
                "rotational angular momentum",
            );
            assert_relative_equal(
                &[last.orbital_energy_j],
                &[first.orbital_energy_j],
                accuracy,
                "orbital energy",
            );
            assert_relative_equal(
                &[last.rotational_energy_j],
                &[first.rotational_energy_j],
                accuracy,
                "rotational energy",
            );
        }
    }

    #[test]
    fn integrated_balanced_wheels() {
        vscmg_integrated_test("BalancedWheels");
    }

    #[test]
    fn integrated_jitter_simple() {
        vscmg_integrated_test("JitterSimple");
    }

    #[test]
    fn integrated_jitter_fully_coupled() {
        vscmg_integrated_test("JitterFullyCoupled");
    }

    #[test]
    fn integrated_jitter_fully_coupled_with_gravity() {
        vscmg_integrated_test("JitterFullyCoupledGravity");
    }

    /// Upstream `defaultVSCMG` of
    /// `test_VSCMGStateEffector_ConfigureVSCMGRequests.py` (left unvalidated,
    /// as upstream calls `ConfigureVSCMGRequests` directly).
    fn request_default() -> VscmgConfig {
        VscmgConfig {
            gs_hat0_b: Vector3::x(),
            gt_hat0_b: Vector3::x(),
            gg_hat_b: Vector3::x(),
            u_s_max_nm: 0.0,
            u_g_max_nm: 0.0,
            omega_max_radps: 1000.0,
            gamma_dot_max_radps: -1.0,
            ..VscmgConfig::new("VSCMG", VscmgModel::BalancedWheels)
        }
    }

    /// Upstream `unitSimVSCMG`: two VSCMGs process one command each.
    fn unit_sim_vscmg(
        configs: [VscmgConfig; 2],
        commands: [(f64, f64); 2],
        expected_u_s_current: [f64; 2],
    ) {
        let mut effector = VscmgStateEffector::new("VSCMG");
        for config in configs {
            effector.add_vscmg(config);
        }
        for (vscmg, (u_s_cmd_nm, u_g_cmd_nm)) in effector.vscmgs_mut().iter_mut().zip(commands) {
            vscmg.configure_request(VscmgCommandMsg {
                u_s_cmd_nm,
                u_g_cmd_nm,
            });
        }
        for (vscmg, expected) in effector.vscmgs().iter().zip(expected_u_s_current) {
            assert_eq!(vscmg.u_s_current_nm, expected);
        }
    }

    #[test]
    fn configure_requests_saturation() {
        let mut configs = [request_default(), request_default()];
        for (config, limit) in configs.iter_mut().zip([1.0, 2.0]) {
            config.u_s_max_nm = limit;
            config.u_g_max_nm = limit;
        }
        unit_sim_vscmg(configs, [(-1.2, -1.2), (1.5, 1.5)], [-1.0, 1.5]);
    }

    #[test]
    fn configure_requests_minimum() {
        let mut configs = [request_default(), request_default()];
        for (config, minimum) in configs.iter_mut().zip([0.1, 0.0]) {
            config.u_s_min_nm = minimum;
            config.u_g_min_nm = minimum;
        }
        unit_sim_vscmg(configs, [(-0.09, -0.09), (0.0001, 0.0001)], [0.0, 0.0001]);
    }

    #[test]
    fn configure_requests_friction() {
        let mut configs = [request_default(), request_default()];
        let u_s_f = [0.1, 0.0];
        for (index, config) in configs.iter_mut().enumerate() {
            config.u_s_f_nm = u_s_f[index];
            config.omega_init_radps = [-20.0, 0.0][index];
            config.omega_max_radps = [100.0, 0.0][index];
            config.wheel_linear_friction_ratio = [0.1, 0.0][index];
            config.u_g_f_nm = [0.1, 0.0][index];
            config.gamma_dot_init_radps = [-20.0, 0.0][index];
            config.gamma_dot_max_radps = [100.0, 0.0][index];
            config.gimbal_linear_friction_ratio = [0.1, 0.0][index];
        }
        let u_s_cmd = [-1.0, 0.0];
        unit_sim_vscmg(
            configs,
            [(u_s_cmd[0], -1.0), (u_s_cmd[1], 0.0)],
            [u_s_cmd[0] + u_s_f[0], u_s_cmd[1] + u_s_f[1]],
        );
    }

    /// Upstream `configuration` of `test_vscmgInitialization.py`.
    fn configuration(model: VscmgModel) -> VscmgConfig {
        VscmgConfig {
            gs_hat0_b: Vector3::x(),
            gt_hat0_b: Vector3::y(),
            gg_hat_b: Vector3::z(),
            mass_w_kg: 2.0,
            mass_g_kg: 1.0,
            iw_kg_m2: [0.2, 0.15, 0.15],
            ig_kg_m2: [0.1, 0.15, 0.2],
            static_imbalance_kg_m: 0.02,
            dynamic_imbalance_kg_m2: 0.001,
            r_gcg_g_m: Vector3::new(0.01, 0.02, 0.0),
            l_m: 0.01,
            big_l_m: 0.1,
            omega_init_radps: 1.0,
            gamma_init_rad: 0.3,
            gamma_dot_init_radps: 0.02,
            ..VscmgConfig::new("device", model)
        }
    }

    const MODELS: [VscmgModel; 3] = [
        VscmgModel::BalancedWheels,
        VscmgModel::JitterSimple,
        VscmgModel::JitterFullyCoupled,
    ];

    /// Upstream `simulation_with_effector`: a 100 kg hub at a 1 ms step.
    fn spacecraft_with(effector: VscmgStateEffector) -> Spacecraft {
        let hub = HubSetup {
            mass_kg: 100.0,
            inertia_kg_m2: Matrix3::identity() * 10.0,
            ..HubSetup::hinged_body_hub()
        };
        spacecraft_with_effectors(&hub, 1_000_000, vec![Box::new(effector)])
    }

    fn effector_with(configs: impl IntoIterator<Item = VscmgConfig>) -> VscmgStateEffector {
        let mut effector = VscmgStateEffector::new("VSCMGs");
        for config in configs {
            effector.add_vscmg(config);
        }
        effector
    }

    fn effector_of(spacecraft: &Spacecraft) -> &VscmgStateEffector {
        spacecraft.state_effectors[0]
            .as_any()
            .downcast_ref::<VscmgStateEffector>()
            .expect("VSCMG effector")
    }

    /// The hub position, attitude and rate after each step of `steps`.
    fn history(spacecraft: &mut Spacecraft, steps: usize) -> Vec<f64> {
        let state_out = spacecraft.state_out.clone();
        let mut history = Vec::new();
        run_inspecting_every_step(spacecraft, steps, |_| {
            let state = state_out.read();
            history.extend(
                state
                    .position_m
                    .iter()
                    .chain(&state.sigma_bn)
                    .chain(&state.omega_radps),
            );
        });
        history
    }

    /// The panic message of `f`, which must panic.
    fn panic_message(f: impl FnOnce()) -> String {
        let payload = catch_unwind(AssertUnwindSafe(f)).expect_err("expected a panic");
        payload
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| {
                payload
                    .downcast_ref::<&str>()
                    .map(|message| message.to_string())
            })
            .expect("panic message")
    }

    /// Upstream `test_attachment_matches_both_scheduler_orders`: the derived
    /// values, and the same propagation whether a zero command is connected
    /// or not.
    #[test]
    fn attachment_derives_configuration_and_propagates() {
        for model in MODELS {
            let derived = effector_with([configuration(model)]).vscmgs()[0].derived();
            assert!((derived.mass_v_kg - 3.0).abs() < 1e-12);
            assert!((derived.rho_w - 2.0 / 3.0).abs() < 1e-12);
            if model == VscmgModel::JitterFullyCoupled {
                assert!((derived.d_m - 0.01).abs() < 1e-12);
                assert!((derived.iw13_kg_m2 - 0.001).abs() < 1e-12);
            } else {
                assert!((derived.iv_kg_m2[0] - 0.3).abs() < 1e-12);
                assert!((derived.iv_kg_m2[2] - 0.35).abs() < 1e-12);
            }

            let mut histories = Vec::new();
            for connected in [false, true] {
                let mut effector = effector_with([configuration(model)]);
                let command = Output::new(VscmgCommandMsg::default());
                if connected {
                    command.connect_to(&mut effector.vscmgs_mut()[0].command_in_msg);
                }
                let mut spacecraft = spacecraft_with(effector);
                let history = history(&mut spacecraft, 20);
                assert!(history.iter().all(|value| value.is_finite()), "{model:?}");
                histories.push(history);
            }
            for (a, b) in histories[0].iter().zip(&histories[1]) {
                assert!(
                    (a - b).abs() <= 1e-14 + 1e-12 * b.abs(),
                    "{model:?}: {a} vs {b}"
                );
            }
        }
    }

    /// Upstream `test_massless_device_has_finite_derived_values`.
    #[test]
    fn massless_device_has_finite_derived_values() {
        for model in &MODELS[..2] {
            let mut config = configuration(*model);
            config.mass_w_kg = 0.0;
            config.mass_g_kg = 0.0;
            let effector = effector_with([config]);
            let derived = effector.vscmgs()[0].derived();
            assert_eq!(derived.mass_v_kg, 0.0);
            assert_eq!((derived.rho_w, derived.rho_g), (0.0, 0.0));
            let mut spacecraft = spacecraft_with(effector);
            assert!(
                history(&mut spacecraft, 2)
                    .iter()
                    .all(|value| value.is_finite())
            );
        }
    }

    /// Upstream `test_fully_coupled_device_allows_massless_gimbal`.
    #[test]
    fn fully_coupled_device_allows_massless_gimbal() {
        let mut config = configuration(VscmgModel::JitterFullyCoupled);
        config.mass_g_kg = 0.0;
        let effector = effector_with([config]);
        assert_eq!(effector.validate(), Ok(()));
        let derived = effector.vscmgs()[0].derived();
        assert_eq!((derived.rho_g, derived.rho_w), (0.0, 1.0));
    }

    /// Upstream `test_uncoupled_models_reject_zero_inertia_divisor`.
    #[test]
    fn uncoupled_models_reject_zero_inertia_divisor() {
        for model in &MODELS[..2] {
            for field in ["IW1", "IV3"] {
                let mut config = configuration(*model);
                if field == "IW1" {
                    config.iw_kg_m2[0] = 0.0;
                } else {
                    config.iw_kg_m2[2] = 0.0;
                    config.ig_kg_m2[2] = 0.0;
                }
                let message = panic_message(|| {
                    spacecraft_with(effector_with([config]));
                });
                assert!(
                    message.contains("must be positive"),
                    "{model:?} {field}: {message}"
                );
            }
        }
    }

    /// Upstream `test_invalid_configuration_rejected` (fully coupled device),
    /// through validation (upstream `Reset`) and spacecraft initialization.
    #[test]
    fn invalid_configuration_rejected() {
        type Edit = fn(&mut VscmgConfig);
        let cases: [(Edit, &str); 13] = [
            (|c| c.mass_w_kg = -1.0, "massW"),
            (|c| c.mass_g_kg = -1.0, "massG"),
            (|c| c.mass_w_kg = 0.0, "massW"),
            (|c| c.mass_g_kg = f64::NAN, "massG"),
            (|c| c.gs_hat0_b = Vector3::zeros(), "gsHat0_B"),
            (
                |c| c.gs_hat0_b = Vector3::new(f64::INFINITY, 0.0, 0.0),
                "gsHat0_B",
            ),
            (|c| c.gt_hat0_b = Vector3::x(), "orthogonal"),
            (|c| c.gg_hat_b = -Vector3::z(), "right-handed"),
            (|c| c.static_imbalance_kg_m = f64::INFINITY, "U_s"),
            (|c| c.gamma_init_rad = f64::NAN, "gamma"),
            (|c| c.iw_kg_m2[0] = -1.0, "inertias"),
            (|c| c.ig_products_kg_m2[1] = f64::NAN, "IG13"),
            (|c| c.r_gcg_g_m = Vector3::new(f64::NAN, 0.0, 0.0), "rGcG_G"),
        ];
        for (edit, expected) in cases {
            let mut config = configuration(VscmgModel::JitterFullyCoupled);
            edit(&mut config);
            let effector = effector_with([config]);
            let message = effector
                .validate()
                .expect_err("invalid configuration accepted");
            assert!(message.contains(expected), "{expected}: {message}");
            let message = panic_message(|| {
                spacecraft_with(effector).state_effectors[0].initial_state();
            });
            assert!(message.contains(expected), "{expected}: {message}");
        }
    }

    /// Upstream `test_repeated_reset_normalizes_axes_and_preserves_states`.
    #[test]
    fn repeated_validation_normalizes_axes_and_preserves_states() {
        for scale in [2.0, 1e300, 1e-300] {
            let mut config = configuration(VscmgModel::JitterFullyCoupled);
            config.gs_hat0_b = Vector3::new(scale, 0.0, 0.0);
            config.gt_hat0_b = Vector3::new(0.0, scale, 0.0);
            config.gg_hat_b = Vector3::new(0.0, 0.0, scale);
            config.u_s_current_nm = 0.1;
            let mut effector = effector_with([config]);
            let mut state = effector.initial_state();
            state[0] = 2.0;
            effector.load_state(&state);
            for _ in 0..3 {
                assert_eq!(effector.validate(), Ok(()));
                let vscmg = &effector.vscmgs()[0];
                let derived = vscmg.derived();
                assert!(
                    (derived.dcm_bg0 - Matrix3::identity()).norm() < 1e-15,
                    "{scale}"
                );
                assert!((derived.d_m - 0.01).abs() < 1e-12);
                assert_eq!(vscmg.u_s_current_nm, 0.1);
                assert_eq!(vscmg.omega_radps, 2.0);
            }
        }
    }

    /// Upstream `test_device_count_exceeding_payload_capacity_is_rejected`.
    #[test]
    fn device_count_exceeding_payload_capacity_is_rejected() {
        let effector = effector_with(
            std::iter::repeat_with(|| configuration(VscmgModel::BalancedWheels))
                .take(MAX_EFF_COUNT + 1),
        );
        let message = effector.validate().expect_err("oversized cluster accepted");
        assert!(
            message.contains("device count") && message.contains("MAX_EFF_CNT"),
            "{message}"
        );
        let message = panic_message(|| {
            effector.initial_state();
        });
        assert!(
            message.contains("device count") && message.contains("MAX_EFF_CNT"),
            "{message}"
        );
    }

    /// Upstream `test_device_count_at_payload_capacity_processes_last_command`.
    #[test]
    fn device_count_at_payload_capacity_processes_last_command() {
        for model in MODELS {
            let mut effector =
                effector_with(std::iter::repeat_with(|| configuration(model)).take(MAX_EFF_COUNT));
            let command = Output::new(VscmgCommandMsg {
                u_s_cmd_nm: 0.04,
                u_g_cmd_nm: -0.01,
            });
            command.connect_to(&mut effector.vscmgs_mut()[MAX_EFF_COUNT - 1].command_in_msg);
            let speed_out = effector.speed_out_msg.clone();
            let mut spacecraft = spacecraft_with(effector);
            history(&mut spacecraft, 2);
            let effector = effector_of(&spacecraft);
            let last = &effector.vscmgs()[MAX_EFF_COUNT - 1];
            assert!((last.u_s_current_nm - 0.04).abs() < 1e-12, "{model:?}");
            assert!((last.u_g_current_nm + 0.01).abs() < 1e-12, "{model:?}");
            let speeds = speed_out.read();
            for (index, vscmg) in effector.vscmgs().iter().enumerate() {
                assert!(vscmg.omega_radps.is_finite());
                assert_eq!(speeds.wheel_speeds_radps[index], vscmg.omega_radps);
            }
        }
    }

    /// Upstream `test_nonfinite_motor_configuration_is_rejected`.
    #[test]
    fn nonfinite_motor_configuration_is_rejected() {
        type Field = fn(&mut VscmgConfig) -> &mut f64;
        let fields: [(Field, &str); 12] = [
            (|c| &mut c.u_s_current_nm, "u_s_current"),
            (|c| &mut c.u_g_current_nm, "u_g_current"),
            (|c| &mut c.u_s_f_nm, "u_s_f"),
            (|c| &mut c.u_g_f_nm, "u_g_f"),
            (|c| &mut c.u_s_max_nm, "u_s_max"),
            (|c| &mut c.u_g_max_nm, "u_g_max"),
            (|c| &mut c.u_s_min_nm, "u_s_min"),
            (|c| &mut c.u_g_min_nm, "u_g_min"),
            (|c| &mut c.omega_max_radps, "Omega_max"),
            (|c| &mut c.gamma_dot_max_radps, "gammaDot_max"),
            (
                |c| &mut c.wheel_linear_friction_ratio,
                "wheelLinearFrictionRatio",
            ),
            (
                |c| &mut c.gimbal_linear_friction_ratio,
                "gimbalLinearFrictionRatio",
            ),
        ];
        for (field, name) in fields {
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut config = configuration(VscmgModel::BalancedWheels);
                *field(&mut config) = value;
                let effector = effector_with([config]);
                let message = effector
                    .validate()
                    .expect_err("non-finite setting accepted");
                assert!(message.contains(name), "{name}: {message}");
                let message = panic_message(|| {
                    effector.initial_state();
                });
                assert!(message.contains(name), "{name}: {message}");
            }
        }
    }

    /// Upstream `coupled_configuration_with_invalid_divisor`.
    fn coupled_configuration_with_invalid_divisor(case: &str) -> VscmgConfig {
        let mut config = configuration(VscmgModel::JitterFullyCoupled);
        config.gamma_init_rad = 0.0;
        config.static_imbalance_kg_m = 0.0;
        config.dynamic_imbalance_kg_m2 = 0.0;
        config.l_m = 0.0;
        config.big_l_m = 0.0;
        config.r_gcg_g_m = Vector3::zeros();
        match case {
            "zero_spin" => config.iw_kg_m2[0] = 0.0,
            "tiny_spin" => config.iw_kg_m2[0] = 1e-320,
            "overflow_spin" => config.static_imbalance_kg_m = 1e200,
            "tiny_mass" => {
                config.mass_w_kg = 1e-320;
                config.mass_g_kg = 0.0;
            }
            "zero_gimbal" => {
                config.iw_kg_m2[2] = 0.0;
                config.ig_kg_m2[2] = 0.0;
            }
            "overflow_gimbal" => {
                config.iw_kg_m2[2] = 1e308;
                config.ig_kg_m2[2] = 1e308;
            }
            "singular_coupling" => {
                config.iw_kg_m2[0] = 1.0;
                config.iw_kg_m2[2] = 1.0;
                config.ig_kg_m2[2] = 0.0;
                config.dynamic_imbalance_kg_m2 = 1.0;
            }
            _ => unreachable!("unknown case {case}"),
        }
        config
    }

    /// Upstream `test_invalid_coupled_divisors_stop_initialization`: the
    /// spacecraft stops before it propagates a singular device.
    #[test]
    fn invalid_coupled_divisors_stop_the_spacecraft() {
        for (case, divisor) in [
            ("zero_spin", "eOmega"),
            ("tiny_spin", "eOmega"),
            ("overflow_spin", "eOmega"),
            ("tiny_mass", "massV"),
            ("zero_gimbal", "egamma"),
            ("overflow_gimbal", "egamma"),
            ("singular_coupling", "1 - cOmega*cgamma"),
        ] {
            let effector = effector_with([coupled_configuration_with_invalid_divisor(case)]);
            let message = panic_message(move || {
                let mut spacecraft = spacecraft_with(effector);
                history(&mut spacecraft, 1);
            });
            assert!(message.contains(divisor), "{case}: {message}");
        }
    }

    /// Upstream `test_invalid_coupled_spin_divisor_is_rejected_by_reset`.
    #[test]
    fn invalid_coupled_spin_divisor_is_rejected_by_validation() {
        for case in ["zero_spin", "tiny_spin", "overflow_spin"] {
            let effector = effector_with([coupled_configuration_with_invalid_divisor(case)]);
            let message = effector.validate().expect_err("singular device accepted");
            assert!(message.contains("eOmega"), "{case}: {message}");
        }
    }

    /// Upstream `test_coupled_divisor_is_checked_again_during_propagation`: a
    /// device made singular after a valid start stops the next step, and the
    /// equations of motion check it too.
    #[test]
    fn coupled_divisor_is_checked_again_during_propagation() {
        let mut config = configuration(VscmgModel::JitterFullyCoupled);
        config.static_imbalance_kg_m = 0.0;
        config.dynamic_imbalance_kg_m2 = 0.0;
        let mut effector = effector_with([config]);
        let state = effector.initial_state();
        effector.vscmgs_mut()[0].config.iw_kg_m2[0] = 0.0;
        let message = panic_message(|| effector.pre_integration(0, 0.001));
        assert!(message.contains("eOmega"), "{message}");
        let message = panic_message(|| {
            effector.update_contributions(
                &state,
                &crate::messages::SpacecraftStateMsg::default(),
                Vector3::zeros(),
                &mut Default::default(),
            )
        });
        assert!(message.contains("eOmega"), "{message}");
    }

    /// Upstream `test_nonfinite_command_is_rejected_before_motor_processing`.
    #[test]
    fn nonfinite_command_is_rejected_before_motor_processing() {
        for wheel in [true, false] {
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut effector = effector_with([configuration(VscmgModel::BalancedWheels)]);
                let command = Output::new(VscmgCommandMsg {
                    u_s_cmd_nm: if wheel { value } else { 0.0 },
                    u_g_cmd_nm: if wheel { 0.0 } else { value },
                });
                command.connect_to(&mut effector.vscmgs_mut()[0].command_in_msg);
                let message = panic_message(|| effector.pre_integration(0, 0.001));
                assert!(
                    message.contains("u_s_cmd and u_g_cmd must be finite"),
                    "{message}"
                );
            }
        }
    }

    /// Upstream `test_invalid_linear_friction_threshold_is_rejected`.
    #[test]
    fn invalid_linear_friction_threshold_is_rejected() {
        for wheel in [true, false] {
            for (limit, ratio) in [(0.0, 0.1), (-1.0, 0.1), (1e-300, 1e-300), (1e300, 1e300)] {
                let mut config = configuration(VscmgModel::BalancedWheels);
                let name = if wheel {
                    config.omega_max_radps = limit;
                    config.wheel_linear_friction_ratio = ratio;
                    "wheelLinearFrictionRatio"
                } else {
                    config.gamma_dot_max_radps = limit;
                    config.gimbal_linear_friction_ratio = ratio;
                    "gimbalLinearFrictionRatio"
                };
                config.omega_init_radps = 0.0;
                config.gamma_dot_init_radps = 0.0;
                let mut effector = effector_with([config]);
                Output::new(VscmgCommandMsg::default())
                    .connect_to(&mut effector.vscmgs_mut()[0].command_in_msg);
                effector.initial_state();
                let message = panic_message(|| effector.pre_integration(0, 0.001));
                assert!(
                    message.contains(name),
                    "{name} ({limit}, {ratio}): {message}"
                );
            }
        }
    }

    /// Upstream `test_negative_motor_limits_and_friction_ratios_remain_supported`.
    #[test]
    fn negative_motor_limits_and_friction_ratios_remain_supported() {
        let mut config = configuration(VscmgModel::BalancedWheels);
        config.u_s_max_nm = -1.0;
        config.u_g_max_nm = -1.0;
        config.u_s_min_nm = -1.0;
        config.u_g_min_nm = -1.0;
        config.omega_max_radps = -1.0;
        config.gamma_dot_max_radps = -1.0;
        config.wheel_linear_friction_ratio = -1.0;
        config.gimbal_linear_friction_ratio = -1.0;
        let mut effector = effector_with([config]);
        let command = Output::new(VscmgCommandMsg {
            u_s_cmd_nm: 0.04,
            u_g_cmd_nm: -0.01,
        });
        command.connect_to(&mut effector.vscmgs_mut()[0].command_in_msg);
        let mut spacecraft = spacecraft_with(effector);
        history(&mut spacecraft, 2);
        let vscmg = &effector_of(&spacecraft).vscmgs()[0];
        assert!((vscmg.u_s_current_nm - 0.04).abs() < 1e-12);
        assert!((vscmg.u_g_current_nm + 0.01).abs() < 1e-12);
    }

    /// Upstream `test_gimbal_friction_uses_its_own_smoothing_ratio`.
    #[test]
    fn gimbal_friction_uses_its_own_smoothing_ratio() {
        for wheel_ratio in [0.0, 0.1] {
            let mut config = configuration(VscmgModel::BalancedWheels);
            config.wheel_linear_friction_ratio = wheel_ratio;
            config.gimbal_linear_friction_ratio = 0.5;
            config.omega_max_radps = 10.0;
            config.gamma_dot_max_radps = 10.0;
            config.gamma_dot_init_radps = 1.0;
            config.u_g_f_nm = 0.5;
            let mut effector = effector_with([config]);
            assert_eq!(effector.validate(), Ok(()));
            effector.vscmgs_mut()[0].configure_request(VscmgCommandMsg::default());
            assert!((effector.vscmgs()[0].u_g_current_nm + 0.1).abs() < 1e-12);
        }
    }

    /// Upstream `test_motor_arithmetic_overflow_is_rejected_before_applying_torque`.
    #[test]
    fn motor_arithmetic_overflow_is_rejected_before_applying_torque() {
        let mut config = configuration(VscmgModel::BalancedWheels);
        config.omega_init_radps = -1.0;
        config.u_s_f_nm = 1e308;
        let mut effector = effector_with([config]);
        let message = panic_message(|| {
            effector.vscmgs_mut()[0].configure_request(VscmgCommandMsg {
                u_s_cmd_nm: 1e308,
                u_g_cmd_nm: 0.0,
            })
        });
        assert!(message.contains("non-finite applied torque"), "{message}");
        assert_eq!(effector.vscmgs()[0].u_s_current_nm, 0.0);
    }

    /// Upstream `test_mixed_models_publish_only_their_own_jitter_angles`.
    #[test]
    fn mixed_models_publish_only_their_own_jitter_angles() {
        for models in [MODELS, [MODELS[2], MODELS[1], MODELS[0]]] {
            let mut effector = effector_with(models.map(configuration));
            let mut state = effector.initial_state();
            let angles = [0.2, 0.4];
            let theta_start = state.len() - 2;
            state[theta_start..].copy_from_slice(&angles);
            effector.load_state(&state);
            effector.write_outputs(0, &Default::default());
            let mut jitter_index = 0;
            for (vscmg, model) in effector.vscmgs().iter().zip(models) {
                let expected = if model == VscmgModel::BalancedWheels {
                    0.0
                } else {
                    jitter_index += 1;
                    angles[jitter_index - 1]
                };
                assert_eq!(vscmg.vscmg_out_msg.read().theta_rad, expected, "{models:?}");
            }
        }
    }

    /// End to end with the FSW: [`VscmgGimbalRateServo`] reads the
    /// effector's speeds and the navigation rate and commands each VSCMG; the
    /// gimbal rates settle on the reference and the wheels accelerate as
    /// asked. The servo leaves out the hub's angular acceleration, which
    /// leaves a gimbal-rate error of about `ĝ_g·ω̇ / K`, and holds its
    /// gyroscopic compensation over each control period while the hub turns;
    /// the hub is heavy and turns slowly so that both stay below the
    /// tolerance.
    #[test]
    fn gimbal_rate_servo_drives_the_vscmgs() {
        let wheel_accel_ref = [0.5, -0.2];
        let gimbal_rate_ref = [0.05, -0.03];
        let k_gamma_dot = 2.0;
        let configs = [
            VscmgConfig {
                gs_hat0_b: Vector3::x(),
                gt_hat0_b: Vector3::y(),
                gg_hat_b: Vector3::z(),
                omega_init_radps: 200.0,
                gamma_dot_init_radps: 0.01,
                ..configuration(VscmgModel::BalancedWheels)
            },
            VscmgConfig {
                gs_hat0_b: Vector3::y(),
                gt_hat0_b: Vector3::z(),
                gg_hat_b: Vector3::x(),
                omega_init_radps: -150.0,
                gamma_init_rad: 0.2,
                gamma_dot_init_radps: 0.0,
                ..configuration(VscmgModel::BalancedWheels)
            },
        ];
        // `fswSetupVSCMGs`: Js = IG1 + IW1, Jt = IG2 + IW2, Jg = IG3 + IW3,
        // Iws = IW1.
        let elements: Vec<VscmgConfigElementMsg> = configs
            .iter()
            .map(|config| VscmgConfigElementMsg {
                gs_hat0_b: config.gs_hat0_b,
                gt_hat0_b: config.gt_hat0_b,
                gg_hat_b: config.gg_hat_b,
                js_kg_m2: config.ig_kg_m2[0] + config.iw_kg_m2[0],
                jt_kg_m2: config.ig_kg_m2[1] + config.iw_kg_m2[1],
                jg_kg_m2: config.ig_kg_m2[2] + config.iw_kg_m2[2],
                iws_kg_m2: config.iw_kg_m2[0],
                wheel_speed0_radps: config.omega_init_radps,
                gimbal_angle0_rad: config.gamma_init_rad,
                gimbal_rate0_radps: config.gamma_dot_init_radps,
            })
            .collect();
        let initial_speeds: Vec<f64> = configs.iter().map(|c| c.omega_init_radps).collect();

        let mut servo = VscmgGimbalRateServo::new(VscmgGimbalRateServoConfig {
            k_gamma_dot_per_s: k_gamma_dot,
            ..VscmgGimbalRateServoConfig::default()
        });
        let mut effector = VscmgStateEffector::new("VSCMGs");
        for config in configs {
            let vscmg = effector.add_vscmg(config);
            servo
                .add_vscmg_command_output()
                .connect_to(&mut vscmg.command_in_msg);
        }
        let params = Output::new(VscmgArrayConfigMsg::from_active(&elements));
        params.connect_to(&mut servo.vscmg_params_in_msg);
        let mut reference = VscmgReferenceStatesMsg::default();
        reference.wheel_accels_radps2[..2].copy_from_slice(&wheel_accel_ref);
        reference.gimbal_rates_radps[..2].copy_from_slice(&gimbal_rate_ref);
        let reference = Output::new(reference);
        reference.connect_to(&mut servo.vscmg_ref_states_in_msg);
        effector.speed_out_msg.connect_to(&mut servo.speeds_in_msg);
        let speed_out = effector.speed_out_msg.clone();

        let hub = HubSetup {
            mass_kg: 500.0,
            inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(1.0e5, 9.0e4, 8.0e4)),
            omega_bn_b_radps: Vector3::new(0.002, -0.004, 0.001),
            ..HubSetup::hinged_body_hub()
        };
        let mut spacecraft = spacecraft_with_effectors(&hub, 10_000_000, vec![Box::new(effector)]);
        let mut navigation = SimpleNavigation::new("simpleNav");
        spacecraft
            .state_out
            .connect_to(&mut navigation.spacecraft_state_in_msg);
        navigation
            .attitude_out_msg
            .connect_to(&mut servo.att_in_msg);

        let duration_s = 5.0;
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("spacecraft", &mut spacecraft, 10_000_000, 3);
            sim.add_module("simpleNav", &mut navigation, 10_000_000, 2);
            sim.add_module("vscmgGimbalRateServo", &mut servo, 10_000_000, 1);
            sim.run_for((duration_s * 1e9) as u64);
        }

        let speeds = speed_out.read();
        for index in 0..2 {
            let rate_error = speeds.gimbal_rates_radps[index] - gimbal_rate_ref[index];
            assert!(
                rate_error.abs() < 1e-4,
                "gimbal {index}: rate error {rate_error:e}"
            );
            let wheel_change = speeds.wheel_speeds_radps[index] - initial_speeds[index];
            let expected_change = wheel_accel_ref[index] * duration_s;
            assert!(
                ((wheel_change - expected_change) / expected_change).abs() < 1e-2,
                "wheel {index}: speed change {wheel_change} vs {expected_change}"
            );
        }
    }
}
