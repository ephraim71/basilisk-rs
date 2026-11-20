//! A chain of N identical hinged rigid panels: port of upstream Basilisk
//! `simulation/dynamics/NHingedRigidBodies` (`NHingedRigidBodyStateEffector`).
//!
//! The first panel hinges on the hub at H (`hinge_position_body_m`, frame
//! `body_to_hinge_dcm`); each further panel hinges on the far end of the one
//! before it. Panel `i`'s angle `θi` is measured about the common hinge axis
//! relative to panel `i − 1`, so its frame is `[Si B] = M2(θ1 + … + θi) [H B]`.
//! Each panel's center of mass lies `d` along −ŝi1 from its hinge, and the next
//! hinge `2d` along −ŝi1. Hinge `i` carries a torsional spring–damper acting on
//! `θi − θi,0`.
//!
//! The equations of motion factor one panel mass and one half-length `d` out
//! of the sums over the chain, so every panel must have the same mass and the
//! same `d`; their inertias may differ. The configuration is checked when the
//! spacecraft initializes ([`NHingedRigidBodyStateEffector::validate`]).
//!
//! State layout, as upstream registers it: `[θ1 … θN, θ̇1 … θ̇N]`.
//!
//! [`NHingedRigidBodyStateEffector::add_dynamic_effector`] attaches a
//! [`DynamicEffector`] to a panel (upstream `addDynamicEffector(effector,
//! segment)`), as
//! [`HingedRigidBodyStateEffector::add_dynamic_effector`](super::hinged_rigid_body_state_effector::HingedRigidBodyStateEffector::add_dynamic_effector)
//! does to the single panel. At every derivative evaluation the effector
//! sees the panel frame with its origin at the panel's hinge, and returns an
//! inertial force and a torque about the hinge in panel axes. The loads act
//! on the panel and, through the hinges, on the panels inboard and the hub
//! (upstream `extForce_B`, `extTorquePntH_B` and `attBodyForce_B`).

use nalgebra::{DMatrix, DVector, Matrix3, MatrixXx3, Vector3};
use std::any::Any;
use std::cell::RefCell;

use crate::kinematics::is_rotation_matrix;
use crate::messages::{HingedRigidBodyMsg, Output, SpacecraftStateMsg};
use crate::spacecraft::{BackSubMatrices, DynamicEffector, StateEffector, StateEffectorMassProps};
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, euler2, mrp_to_dcm};

/// Upstream `NHingedRigidBodyStateEffector::validateConfiguration` tolerance
/// on equal panel masses and half-lengths, relative to the first panel's.
const UNIFORM_CHAIN_RELATIVE_TOLERANCE: f64 = 1e-9;

/// One panel of the chain (upstream `HingedPanel`).
#[derive(Clone, Debug)]
pub struct HingedPanel {
    /// Upstream `mass` \[kg\] (default 1).
    pub mass_kg: f64,
    /// Upstream `d` \[m\]: distance from the panel's hinge to its center of
    /// mass, half the hinge-to-hinge length (default 1).
    pub center_of_mass_offset_m: f64,
    /// Upstream `k` \[N m/rad\]: torsional spring constant of the panel's hinge
    /// (default 1).
    pub spring_constant_nm_per_rad: f64,
    /// Upstream `c` \[N m s/rad\]: rotational damping coefficient of the
    /// panel's hinge (default 0).
    pub damping_nm_s_per_rad: f64,
    /// Upstream `IPntS_S` \[kg m^2\]: inertia about the panel center of mass in
    /// panel-frame components. Upstream leaves it uninitialized; the default
    /// here is the identity.
    pub inertia_about_panel_com_panel_kg_m2: Matrix3<f64>,
    /// Upstream `thetaInit` \[rad\] (default 0).
    pub theta_init_rad: f64,
    /// Upstream `thetaDotInit` \[rad/s\] (default 0).
    pub theta_dot_init_radps: f64,
    /// Upstream `theta_0` \[rad\]: spring rest angle (default 0).
    pub theta_rest_rad: f64,
}

impl Default for HingedPanel {
    fn default() -> Self {
        Self {
            mass_kg: 1.0,
            center_of_mass_offset_m: 1.0,
            spring_constant_nm_per_rad: 1.0,
            damping_nm_s_per_rad: 0.0,
            inertia_about_panel_com_panel_kg_m2: Matrix3::identity(),
            theta_init_rad: 0.0,
            theta_dot_init_radps: 0.0,
            theta_rest_rad: 0.0,
        }
    }
}

/// Chain-level parameters of [`NHingedRigidBodyStateEffector`].
#[derive(Clone, Debug)]
pub struct NHingedRigidBodyConfig {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `r_HB_B` \[m\]: first hinge relative to the body origin
    /// (default zero).
    pub hinge_position_body_m: Vector3<f64>,
    /// Upstream `dcm_HB`: DCM from the body frame to the first hinge frame
    /// (default identity).
    pub body_to_hinge_dcm: Matrix3<f64>,
}

impl NHingedRigidBodyConfig {
    /// A configuration with upstream's default values.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            hinge_position_body_m: Vector3::zeros(),
            body_to_hinge_dcm: Matrix3::identity(),
        }
    }
}

/// A chain of hinged rigid panels (upstream `NHingedRigidBodyStateEffector`).
pub struct NHingedRigidBodyStateEffector {
    pub config: NHingedRigidBodyConfig,
    /// Upstream `nHingedRigidBodyOutMsgs`: angle and rate of each hinge, in
    /// chain order.
    pub hinged_rigid_body_out: Vec<Output<HingedRigidBodyMsg>>,
    /// Upstream `nHingedRigidBodyConfigLogOutMsgs`: inertial state of each
    /// panel, with the panel frame as body frame and its center of mass as
    /// position.
    pub panel_state_out: Vec<Output<SpacecraftStateMsg>>,
    panels: Vec<HingedPanel>,
    /// Effectors attached to each panel, in chain order. They see the panel
    /// frame with its origin at the panel's hinge, and return an inertial
    /// force and a torque about the hinge in panel axes.
    dynamic_effectors: Vec<Vec<Box<dyn DynamicEffector>>>,
    /// Time of the current derivative evaluation, for the attached effectors.
    stage_nanos: u64,
    /// Last loaded `[θ1 … θN, θ̇1 … θ̇N]`.
    state: Vec<f64>,
    back_sub: RefCell<NHingedBackSubTerms>,
}

impl std::fmt::Debug for NHingedRigidBodyStateEffector {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NHingedRigidBodyStateEffector")
            .field("config", &self.config)
            .field("panels", &self.panels)
            .field("state", &self.state)
            .field(
                "dynamic_effectors",
                &self
                    .dynamic_effectors
                    .iter()
                    .map(Vec::len)
                    .collect::<Vec<_>>(),
            )
            .finish_non_exhaustive()
    }
}

impl NHingedRigidBodyStateEffector {
    /// An effector without panels; add them with
    /// [`add_hinged_panel`](Self::add_hinged_panel).
    pub fn new(config: NHingedRigidBodyConfig) -> Self {
        Self {
            config,
            hinged_rigid_body_out: Vec::new(),
            panel_state_out: Vec::new(),
            panels: Vec::new(),
            dynamic_effectors: Vec::new(),
            stage_nanos: 0,
            state: Vec::new(),
            back_sub: RefCell::new(NHingedBackSubTerms::default()),
        }
    }

    /// Appends `panel` to the outboard end of the chain, with its output
    /// messages (upstream `addHingedPanel`).
    pub fn add_hinged_panel(&mut self, panel: HingedPanel) {
        self.panels.push(panel);
        self.dynamic_effectors.push(Vec::new());
        self.hinged_rigid_body_out.push(Output::default());
        self.panel_state_out.push(Output::default());
        self.state = self.configured_initial_state();
    }

    /// The panels, from the hub outward.
    pub fn panels(&self) -> &[HingedPanel] {
        &self.panels
    }

    /// Attaches an effector to panel `index`, counting from the hub (upstream
    /// `addDynamicEffector(effector, index + 1)`). Any effector is accepted,
    /// so it must make sense for a body whose state is the panel's.
    ///
    /// # Panics
    ///
    /// If the chain has no panel `index` (upstream: "specifying attachment
    /// to a non-existent panel").
    pub fn add_dynamic_effector<E>(&mut self, index: usize, effector: E)
    where
        E: DynamicEffector + 'static,
    {
        assert!(
            index < self.panels.len(),
            "NHingedRigidBodyStateEffector '{}': specifying attachment to a non-existent panel \
             {index}",
            self.config.name
        );
        self.dynamic_effectors[index].push(Box::new(effector));
    }

    /// The effectors attached to panel `index`.
    pub fn dynamic_effectors(&self, index: usize) -> &[Box<dyn DynamicEffector>] {
        &self.dynamic_effectors[index]
    }

    /// Each panel's attached loads, as a force and a torque about its hinge
    /// in body axes (upstream `extForce_B`, `extTorquePntH_B`).
    fn attached_loads(
        &self,
        kinematics: &[PanelKinematics],
        hub_state: &SpacecraftStateMsg,
    ) -> Vec<(Vector3<f64>, Vector3<f64>)> {
        let dcm_bn = mrp_to_dcm(hub_state.sigma_bn);
        let dcm_nb = dcm_bn.transpose();
        let omega_bn_b = hub_state.omega_radps;
        self.panels
            .iter()
            .zip(kinematics)
            .zip(&self.dynamic_effectors)
            .map(|((panel, kin), effectors)| {
                if effectors.is_empty() {
                    return (Vector3::zeros(), Vector3::zeros());
                }
                let d = panel.center_of_mass_offset_m;
                let r_hb_b = kin.r_sb_b + d * kin.s1;
                let r_prime_hb_b = kin.r_prime_sb_b - d * kin.sum_theta_dot_radps * kin.s3;
                let hinge_state = SpacecraftStateMsg {
                    position_m: hub_state.position_m + dcm_nb * r_hb_b,
                    velocity_mps: hub_state.velocity_mps
                        + dcm_nb * (r_prime_hb_b + omega_bn_b.cross(&r_hb_b)),
                    sigma_bn: dcm_to_mrp(kin.dcm_sb * dcm_bn),
                    omega_radps: kin.dcm_sb * (omega_bn_b + kin.omega_sb_b),
                    ..Default::default()
                };
                let panel_to_body = kin.dcm_sb.transpose();
                effectors.iter().fold(
                    (Vector3::zeros(), Vector3::zeros()),
                    |(force, torque), effector| {
                        let output = effector.compute_output_at(&hinge_state, self.stage_nanos);
                        (
                            force + dcm_bn * output.force_inertial_n,
                            torque + panel_to_body * output.torque_body_nm,
                        )
                    },
                )
            })
            .collect()
    }

    /// Upstream `validateConfiguration`: at least one panel, every mass finite
    /// and positive, every `d` finite, all masses and `d`s equal to the first
    /// panel's within a relative 1e-9, and `body_to_hinge_dcm` a rotation
    /// matrix.
    pub fn validate(&self) -> Result<(), String> {
        let Some(first) = self.panels.first() else {
            return Err("at least one hinged panel is required".to_string());
        };
        let mut total_mass_kg = 0.0;
        for panel in &self.panels {
            if !panel.mass_kg.is_finite() || panel.mass_kg <= 0.0 {
                return Err("every panel mass must be finite and greater than 0".to_string());
            }
            total_mass_kg += panel.mass_kg;
            if !panel.center_of_mass_offset_m.is_finite() {
                return Err("every panel distance d must be finite".to_string());
            }
            let tolerance = UNIFORM_CHAIN_RELATIVE_TOLERANCE;
            if (panel.mass_kg - first.mass_kg).abs() > tolerance * first.mass_kg.abs()
                || (panel.center_of_mass_offset_m - first.center_of_mass_offset_m).abs()
                    > tolerance * first.center_of_mass_offset_m.abs()
            {
                return Err(
                    "every panel must carry the same mass and the same hinge to center \
                     of mass distance d"
                        .to_string(),
                );
            }
        }
        if !total_mass_kg.is_finite() {
            return Err("total panel mass must be finite".to_string());
        }
        if !is_rotation_matrix(&self.config.body_to_hinge_dcm) {
            return Err(
                "body_to_hinge_dcm must be a finite, orthogonal, right-handed rotation matrix"
                    .to_string(),
            );
        }
        Ok(())
    }

    /// Inertial state of every panel (upstream `computePanelInertialStates`)
    /// for the last loaded chain state and the given hub state.
    pub fn panel_inertial_states(&self, hub_state: &SpacecraftStateMsg) -> Vec<SpacecraftStateMsg> {
        let dcm_bn = mrp_to_dcm(hub_state.sigma_bn);
        let dcm_nb = dcm_bn.transpose();
        let omega_bn_b = hub_state.omega_radps;
        self.kinematics(&self.state)
            .iter()
            .map(|panel| SpacecraftStateMsg {
                position_m: hub_state.position_m + dcm_nb * panel.r_sb_b,
                velocity_mps: hub_state.velocity_mps
                    + dcm_nb * (panel.r_prime_sb_b + omega_bn_b.cross(&panel.r_sb_b)),
                sigma_bn: dcm_to_mrp(panel.dcm_sb * dcm_bn),
                omega_radps: panel.dcm_sb * (omega_bn_b + panel.omega_sb_b),
                ..Default::default()
            })
            .collect()
    }

    fn configured_initial_state(&self) -> Vec<f64> {
        self.panels
            .iter()
            .map(|panel| panel.theta_init_rad)
            .chain(self.panels.iter().map(|panel| panel.theta_dot_init_radps))
            .collect()
    }

    fn kinematics(&self, effector_state: &[f64]) -> Vec<PanelKinematics> {
        assert_eq!(
            effector_state.len(),
            self.state_len(),
            "N hinged rigid body state length mismatch"
        );
        let n = self.panels.len();
        let (theta, theta_dot) = effector_state.split_at(n);
        let mut sum_theta = 0.0;
        let mut sum_theta_dot = 0.0;
        let mut sum_r_h = Vector3::zeros();
        let mut sum_r_prime_h = Vector3::zeros();
        self.panels
            .iter()
            .zip(theta.iter().zip(theta_dot))
            .map(|(panel, (&theta_rad, &theta_dot_radps))| {
                let d = panel.center_of_mass_offset_m;
                sum_theta += theta_rad;
                let dcm_sb = euler2(sum_theta) * self.config.body_to_hinge_dcm;
                let s1: Vector3<f64> = dcm_sb.row(0).transpose();
                let s2: Vector3<f64> = dcm_sb.row(1).transpose();
                let s3: Vector3<f64> = dcm_sb.row(2).transpose();
                let r_sb_b = self.config.hinge_position_body_m - d * s1 + sum_r_h;
                sum_r_h += -2.0 * d * s1;
                sum_theta_dot += theta_dot_radps;
                let r_prime_sb_b = d * (sum_theta_dot * s3 + sum_r_prime_h);
                sum_r_prime_h += 2.0 * s3 * sum_theta_dot;
                PanelKinematics {
                    theta_rad,
                    theta_dot_radps,
                    dcm_sb,
                    s1,
                    s2,
                    s3,
                    r_sb_b,
                    r_prime_sb_b,
                    sum_theta_dot_radps: sum_theta_dot,
                    omega_sb_b: sum_theta_dot * s2,
                }
            })
            .collect()
    }

    fn write_hinge_states(&self) {
        let n = self.panels.len();
        for (index, output) in self.hinged_rigid_body_out.iter().enumerate() {
            output.write(HingedRigidBodyMsg {
                theta_rad: self.state[index],
                theta_dot_radps: self.state[n + index],
            });
        }
    }

    /// Upstream `updateContributions`, with 0-based panel indices where
    /// upstream counts from 1.
    fn back_sub_terms(
        &self,
        kinematics: &[PanelKinematics],
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
        attached_loads_b: &[(Vector3<f64>, Vector3<f64>)],
    ) -> NHingedBackSubTerms {
        let panels = &self.panels;
        let n = panels.len();
        let omega = body_omega_radps;
        let heaviside = |cond: f64| if cond < 0.0 { 0.0 } else { 1.0 };
        let total_mass_kg: f64 = panels.iter().map(|panel| panel.mass_kg).sum();
        let iyy = |index: usize| panels[index].inertia_about_panel_com_panel_kg_m2[(1, 1)];

        // Upstream `attBodyForce_B`, `attBodyTorquePntB_B`: the attached
        // loads, the forces acting at the hinges `r_HB_B = r_SB_B + d ŝ1`.
        let att_body_force_b: Vector3<f64> = attached_loads_b.iter().map(|(force, _)| force).sum();
        let att_body_torque_b: Vector3<f64> = attached_loads_b
            .iter()
            .zip(panels.iter().zip(kinematics))
            .map(|((force, torque), (panel, kin))| {
                torque + (kin.r_sb_b + panel.center_of_mass_offset_m * kin.s1).cross(force)
            })
            .sum();

        // Panel equations `A θ̈ = F r̈_B + G ω̇ + v`.
        let mut matrix_a = DMatrix::zeros(n, n);
        for j in 0..n {
            let (panel_j, kin_j) = (&panels[j], &kinematics[j]);
            for i in 0..n {
                let mut sum_term1 = Vector3::zeros();
                for (k, kin_k) in kinematics.iter().enumerate().skip(i) {
                    sum_term1 += 2.0 * kin_k.s3 + 4.0 * kin_k.s3 * (n - 1 - j) as f64
                        - heaviside(k as f64 - j as f64) * 4.0 * kin_k.s3 * (k as f64 - j as f64);
                }
                let h_ji = heaviside(j as f64 - i as f64);
                matrix_a[(j, i)] = iyy(j) * h_ji
                    + panel_j.mass_kg
                        * panel_j.center_of_mass_offset_m.powi(2)
                        * kin_j.s3.dot(&(sum_term1 - h_ji * kin_j.s3));
            }
        }
        let matrix_e = matrix_a
            .try_inverse()
            .expect("N hinged rigid body panel mass matrix must be invertible");

        let mut matrix_f = MatrixXx3::zeros(n);
        let mut matrix_g = MatrixXx3::zeros(n);
        for j in 0..n {
            let (panel_j, kin_j) = (&panels[j], &kinematics[j]);
            let m_d = panel_j.mass_kg * panel_j.center_of_mass_offset_m;
            let sum_term2_f: Vector3<f64> = (j + 1..n).map(|_| 2.0 * m_d * kin_j.s3).sum();
            matrix_f.set_row(j, &(-m_d * kin_j.s3 - sum_term2_f).transpose());
            let sum_term2_g: Vector3<f64> = kinematics[j + 1..]
                .iter()
                .map(|kin_i| 2.0 * m_d * kin_j.s3.cross(&kin_i.r_sb_b))
                .sum();
            let row = -(iyy(j) * kin_j.s2 - m_d * kin_j.s3.cross(&kin_j.r_sb_b) - sum_term2_g);
            matrix_g.set_row(j, &row.transpose());
        }

        let mut vector_v = DVector::zeros(n);
        let mut mass_of_current_panel_and_before = 0.0;
        let mut ext_force_current_panel_and_before = Vector3::zeros();
        for j in 0..n {
            let (panel_j, kin_j) = (&panels[j], &kinematics[j]);
            mass_of_current_panel_and_before += panel_j.mass_kg;
            ext_force_current_panel_and_before += attached_loads_b[j].0;
            let mut spring_term = -panel_j.spring_constant_nm_per_rad
                * (kin_j.theta_rad - panel_j.theta_rest_rad)
                - panel_j.damping_nm_s_per_rad * kin_j.theta_dot_radps;
            if let (Some(panel_next), Some(kin_next)) = (panels.get(j + 1), kinematics.get(j + 1)) {
                spring_term += panel_next.spring_constant_nm_per_rad
                    * (kin_next.theta_rad - panel_next.theta_rest_rad)
                    + panel_next.damping_nm_s_per_rad * kin_next.theta_dot_radps;
            }
            let sum_term2: Vector3<f64> = kinematics[j + 1..]
                .iter()
                .map(|kin_i| {
                    4.0 * omega.cross(&kin_i.r_prime_sb_b)
                        + 2.0 * omega.cross(&omega.cross(&kin_i.r_sb_b))
                })
                .sum();
            let mut sum_term3 = Vector3::zeros();
            let mut sum_theta_dot = 0.0;
            for (i, (panel_i, kin_i)) in panels.iter().zip(kinematics).enumerate() {
                sum_theta_dot += kin_i.theta_dot_radps;
                sum_term3 += sum_theta_dot.powi(2)
                    * panel_i.center_of_mass_offset_m
                    * (2.0 * kin_i.s1 + 4.0 * kin_i.s1 * (n - 1 - j) as f64
                        - heaviside(i as f64 - j as f64) * 4.0 * kin_i.s1 * (i as f64 - j as f64));
            }
            let sum_theta_dot: f64 = kinematics[..=j].iter().map(|kin| kin.theta_dot_radps).sum();
            sum_term3 -= sum_theta_dot.powi(2) * panel_j.center_of_mass_offset_m * kin_j.s1;
            let inertia_j = &panel_j.inertia_about_panel_com_panel_kg_m2;
            let omega_bn_s = kin_j.dcm_sb * omega;
            let sum_term1 = spring_term
                - (inertia_j[(0, 0)] - inertia_j[(2, 2)]) * omega_bn_s.z * omega_bn_s.x
                - panel_j.mass_kg
                    * panel_j.center_of_mass_offset_m
                    * kin_j.s3.dot(
                        &(2.0 * omega.cross(&kin_j.r_prime_sb_b)
                            + omega.cross(&omega.cross(&kin_j.r_sb_b))
                            + sum_term2
                            + sum_term3),
                    );
            let grav_torque_cur_panel = -panel_j.center_of_mass_offset_m
                * kin_j.s1.cross(&(panel_j.mass_kg * gravity_body_mps2));
            let grav_force_rest_of_panels =
                (total_mass_kg - mass_of_current_panel_and_before) * gravity_body_mps2;
            // An attached torque on a panel outboard of j cancels between the
            // balances of rows j and j + 1; only its force's moment arm
            // remains (upstream's comment).
            let ext_force_rest_of_panels = att_body_force_b - ext_force_current_panel_and_before;
            vector_v[j] = sum_term1
                + kin_j
                    .s2
                    .dot(&(grav_torque_cur_panel + attached_loads_b[j].1))
                + 2.0
                    * panel_j.center_of_mass_offset_m
                    * kin_j
                        .s3
                        .dot(&(grav_force_rest_of_panels + ext_force_rest_of_panels));
        }

        let theta_ddot_accel = &matrix_e * matrix_f;
        let theta_ddot_omega_dot = &matrix_e * matrix_g;
        let theta_ddot_bias = &matrix_e * vector_v;

        let mut terms = NHingedBackSubTerms::default();

        // Translational contributions.
        let mut sum_theta_dot = 0.0;
        for j in 0..n {
            let (panel_j, kin_j) = (&panels[j], &kinematics[j]);
            sum_theta_dot += kin_j.theta_dot_radps;
            let sum_term1: Vector3<f64> = panels
                .iter()
                .zip(kinematics)
                .enumerate()
                .skip(j)
                .map(|(k, (panel_k, kin_k))| {
                    (2.0 * (n - 1 - k) as f64 + 1.0)
                        * panel_k.mass_kg
                        * panel_k.center_of_mass_offset_m
                        * kin_k.s3
                })
                .sum();
            let sum_term2 = sum_theta_dot.powi(2)
                * (2.0 * (n - 1 - j) as f64 + 1.0)
                * panel_j.mass_kg
                * panel_j.center_of_mass_offset_m
                * kin_j.s1;
            terms.matrix_a += sum_term1 * theta_ddot_accel.row(j);
            terms.matrix_b += sum_term1 * theta_ddot_omega_dot.row(j);
            terms.vec_trans += -sum_term2 - sum_term1 * theta_ddot_bias[j];
        }
        terms.vec_trans += att_body_force_b;

        // Rotational contributions.
        let mut sum_theta_dot = 0.0;
        for j in 0..n {
            let (panel_j, kin_j) = (&panels[j], &kinematics[j]);
            sum_theta_dot += kin_j.theta_dot_radps;
            let mut sum_term1 = Vector3::zeros();
            for k in j..n {
                let (panel_k, kin_k) = (&panels[k], &kinematics[k]);
                let sum_term3: Vector3<f64> = kinematics[k + 1..]
                    .iter()
                    .map(|kin_n| 2.0 * kin_n.r_sb_b.cross(&kin_k.s3))
                    .sum();
                sum_term1 += iyy(k) * kin_k.s2
                    + panel_k.mass_kg
                        * panel_k.center_of_mass_offset_m
                        * (kin_k.r_sb_b.cross(&kin_k.s3) + sum_term3);
            }
            let sum_term3: Vector3<f64> = kinematics[j + 1..]
                .iter()
                .map(|kin_n| 2.0 * kin_n.r_sb_b.cross(&kin_j.s1))
                .sum();
            let sum_term2 = panel_j.mass_kg * omega.cross(&kin_j.r_sb_b.cross(&kin_j.r_prime_sb_b))
                + sum_theta_dot.powi(2)
                    * panel_j.mass_kg
                    * panel_j.center_of_mass_offset_m
                    * (kin_j.r_sb_b.cross(&kin_j.s1) + sum_term3)
                + iyy(j) * sum_theta_dot * omega.cross(&kin_j.s2);
            terms.matrix_c += sum_term1 * theta_ddot_accel.row(j);
            terms.matrix_d += sum_term1 * theta_ddot_omega_dot.row(j);
            terms.vec_rot += -sum_term2 - sum_term1 * theta_ddot_bias[j];
        }
        terms.vec_rot += att_body_torque_b;

        terms.theta_ddot_accel = theta_ddot_accel;
        terms.theta_ddot_omega_dot = theta_ddot_omega_dot;
        terms.theta_ddot_bias = theta_ddot_bias;
        terms
    }

    /// Each panel's inertia about its center of mass, inertial angular velocity
    /// and center-of-mass velocity relative to B, in body-frame components.
    fn panel_motion(
        &self,
        kinematics: &[PanelKinematics],
        body_omega_radps: Vector3<f64>,
    ) -> Vec<(Matrix3<f64>, Vector3<f64>, Vector3<f64>)> {
        self.panels
            .iter()
            .zip(kinematics)
            .map(|(panel, kin)| {
                (
                    kin.dcm_sb.transpose() * panel.inertia_about_panel_com_panel_kg_m2 * kin.dcm_sb,
                    kin.omega_sb_b + body_omega_radps,
                    kin.r_prime_sb_b + body_omega_radps.cross(&kin.r_sb_b),
                )
            })
            .collect()
    }
}

impl StateEffector for NHingedRigidBodyStateEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn state_len(&self) -> usize {
        2 * self.panels.len()
    }

    /// Validates the chain (upstream `registerStates`) and returns the initial
    /// `[θ1 … θN, θ̇1 … θ̇N]`.
    ///
    /// # Panics
    ///
    /// If [`NHingedRigidBodyStateEffector::validate`] fails.
    fn initial_state(&self) -> Vec<f64> {
        if let Err(message) = self.validate() {
            panic!(
                "invalid NHingedRigidBodyStateEffector '{}': {message}",
                self.config.name
            );
        }
        self.configured_initial_state()
    }

    fn load_state(&mut self, state: &[f64]) {
        assert_eq!(
            state.len(),
            self.state_len(),
            "N hinged rigid body state length mismatch"
        );
        self.state.clear();
        self.state.extend_from_slice(state);
        self.write_hinge_states();
    }

    fn pre_integration(&mut self, current_sim_nanos: u64, dt_seconds: f64) {
        for effector in self.dynamic_effectors.iter_mut().flatten() {
            effector.pre_integration(current_sim_nanos, dt_seconds);
        }
    }

    /// Hands the stage time to the attached effectors, as the spacecraft does
    /// to its own.
    fn begin_stage(&mut self, stage_nanos: u64) {
        self.stage_nanos = stage_nanos;
        for effector in self.dynamic_effectors.iter_mut().flatten() {
            effector.begin_stage(stage_nanos);
        }
    }

    fn update_contributions(
        &self,
        effector_state: &[f64],
        hub_state: &SpacecraftStateMsg,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        let kinematics = self.kinematics(effector_state);
        let attached_loads_b = self.attached_loads(&kinematics, hub_state);
        let terms = self.back_sub_terms(
            &kinematics,
            hub_state.omega_radps,
            gravity_body_mps2,
            &attached_loads_b,
        );
        back_sub.matrix_a += terms.matrix_a;
        back_sub.matrix_b += terms.matrix_b;
        back_sub.matrix_c += terms.matrix_c;
        back_sub.matrix_d += terms.matrix_d;
        back_sub.vec_trans += terms.vec_trans;
        back_sub.vec_rot += terms.vec_rot;
        self.back_sub.replace(terms);
    }

    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        assert_eq!(
            effector_state.len(),
            self.state_len(),
            "N hinged rigid body state length mismatch"
        );
        let back_sub = self.back_sub.borrow();
        let theta_ddot = &back_sub.theta_ddot_accel * body_trans_accel_mps2
            + &back_sub.theta_ddot_omega_dot * body_omega_dot_radps2
            + &back_sub.theta_ddot_bias;
        let n = self.panels.len();
        effector_state[n..]
            .iter()
            .copied()
            .chain(theta_ddot.iter().copied())
            .collect()
    }

    /// Upstream `updateEffectorMassProps`, with the parallel-axis terms
    /// expanded (`r̃ r̃ᵀ = |r|² I − r rᵀ`, `r̃' r̃ + r̃ r̃' = r r'ᵀ + r' rᵀ − 2 (r·r') I`):
    /// the spacecraft evaluates them several times per integrator stage.
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        let kinematics = self.kinematics(effector_state);
        let mut props = StateEffectorMassProps::default();
        let mut first_moment = Vector3::zeros();
        let mut first_moment_prime = Vector3::zeros();
        let identity = Matrix3::identity();
        for (panel, kin) in self.panels.iter().zip(&kinematics) {
            let inertia = &panel.inertia_about_panel_com_panel_kg_m2;
            let (r, r_prime) = (kin.r_sb_b, kin.r_prime_sb_b);
            props.mass_kg += panel.mass_kg;
            props.inertia_about_point_b_body_kg_m2 += kin.dcm_sb.transpose() * inertia * kin.dcm_sb
                + panel.mass_kg * (r.norm_squared() * identity - r * r.transpose());
            first_moment += panel.mass_kg * r;
            first_moment_prime += panel.mass_kg * r_prime;
            // Upstream `ISPrimePntS_B`.
            let inertia_prime_about_com_body = kin.sum_theta_dot_radps
                * (inertia[(2, 2)] - inertia[(0, 0)])
                * (kin.s1 * kin.s3.transpose() + kin.s3 * kin.s1.transpose());
            props.inertia_about_point_b_body_prime_kg_m2ps += inertia_prime_about_com_body
                - panel.mass_kg
                    * (r * r_prime.transpose() + r_prime * r.transpose()
                        - 2.0 * r.dot(&r_prime) * identity);
        }
        props.center_of_mass_body_m = first_moment / props.mass_kg;
        props.center_of_mass_prime_body_mps = first_moment_prime / props.mass_kg;
        props
    }

    /// Upstream `updateEnergyMomContributions` (angular momentum about B).
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        let kinematics = self.kinematics(effector_state);
        self.panel_motion(&kinematics, body_omega_radps)
            .into_iter()
            .zip(self.panels.iter().zip(&kinematics))
            .map(|((inertia, omega_sn, r_dot), (panel, kin))| {
                inertia * omega_sn + panel.mass_kg * kin.r_sb_b.cross(&r_dot)
            })
            .sum()
    }

    /// Upstream `updateEnergyMomContributions`: kinetic energy relative to B
    /// plus the hinge spring energy.
    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        let kinematics = self.kinematics(effector_state);
        self.panel_motion(&kinematics, body_omega_radps)
            .into_iter()
            .zip(self.panels.iter().zip(&kinematics))
            .map(|((inertia, omega_sn, r_dot), (panel, kin))| {
                let deflection_rad = kin.theta_rad - panel.theta_rest_rad;
                0.5 * omega_sn.dot(&(inertia * omega_sn))
                    + 0.5 * panel.mass_kg * r_dot.dot(&r_dot)
                    + 0.5 * panel.spring_constant_nm_per_rad * deflection_rad * deflection_rad
            })
            .sum()
    }

    fn write_outputs(&mut self, _current_sim_nanos: u64, hub_state: &SpacecraftStateMsg) {
        self.write_hinge_states();
        for (output, state) in self
            .panel_state_out
            .iter()
            .zip(self.panel_inertial_states(hub_state))
        {
            output.write(state);
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// Back-substitution contributions and the panel-acceleration terms
/// `θ̈ = [E F] r̈_B + [E G] ω̇ + E v` cached for `compute_derivatives`.
#[derive(Clone, Debug)]
struct NHingedBackSubTerms {
    matrix_a: Matrix3<f64>,
    matrix_b: Matrix3<f64>,
    matrix_c: Matrix3<f64>,
    matrix_d: Matrix3<f64>,
    vec_trans: Vector3<f64>,
    vec_rot: Vector3<f64>,
    /// `E F` (upstream `matrixEDHRB * matrixFDHRB`).
    theta_ddot_accel: MatrixXx3<f64>,
    /// `E G` (upstream `matrixEDHRB * matrixGDHRB`).
    theta_ddot_omega_dot: MatrixXx3<f64>,
    /// `E v` (upstream `matrixEDHRB * vectorVDHRB`).
    theta_ddot_bias: DVector<f64>,
}

impl Default for NHingedBackSubTerms {
    fn default() -> Self {
        Self {
            matrix_a: Matrix3::zeros(),
            matrix_b: Matrix3::zeros(),
            matrix_c: Matrix3::zeros(),
            matrix_d: Matrix3::zeros(),
            vec_trans: Vector3::zeros(),
            vec_rot: Vector3::zeros(),
            theta_ddot_accel: MatrixXx3::zeros(0),
            theta_ddot_omega_dot: MatrixXx3::zeros(0),
            theta_ddot_bias: DVector::zeros(0),
        }
    }
}

/// One panel's geometry for a given chain state (upstream `HingedPanel`'s
/// derived members, filled by `updateEffectorMassProps`). The hinge position
/// and velocity (`r_HB_B`, `rPrime_HB_B`), which only attached dynamic
/// effectors need, follow from `r_sb_b` and `r_prime_sb_b`.
#[derive(Clone, Debug)]
struct PanelKinematics {
    theta_rad: f64,
    theta_dot_radps: f64,
    /// Upstream `dcm_SB`.
    dcm_sb: Matrix3<f64>,
    /// Upstream `sHat1_B`.
    s1: Vector3<f64>,
    /// Upstream `sHat2_B`.
    s2: Vector3<f64>,
    /// Upstream `sHat3_B`.
    s3: Vector3<f64>,
    /// Upstream `r_SB_B`: this panel's center of mass.
    r_sb_b: Vector3<f64>,
    /// Upstream `rPrime_SB_B`.
    r_prime_sb_b: Vector3<f64>,
    /// θ̇1 + … + θ̇i \[rad/s\]: this panel's rate relative to the hub.
    sum_theta_dot_radps: f64,
    /// Upstream `omega_SB_B`.
    omega_sb_b: Vector3<f64>,
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_nHingedRigidBodyStateEffector.py` and
    //! `test_nHingedRigidBodyDefaultConfig.py`.
    //!
    //! `test_nHingedRigidBodyDynamicEffectorSegmentBounds` is
    //! `dynamic_effectors_only_attach_to_existing_panels` and its
    //! `..._reject_a_missing_panel`; the attached-load terms are also checked
    //! against a single hinged panel carrying the same effector.

    use super::{HingedPanel, NHingedRigidBodyConfig, NHingedRigidBodyStateEffector};
    use crate::dynamics::dual_hinged_rigid_body_state_effector::{
        DualHingedPanelConfig, DualHingedRigidBodyConfig, DualHingedRigidBodyStateEffector,
    };
    use crate::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
    use crate::dynamics::state_effector_test_support::{
        HubSetup, add_point_mass_earth, assert_conserved, run_logging_every_step,
        spacecraft_with_effectors,
    };
    use crate::messages::SpacecraftStateMsg;
    use crate::spacecraft::StateEffector;
    use nalgebra::{Matrix3, Vector3};
    use std::f64::consts::PI;

    fn upstream_panel() -> HingedPanel {
        HingedPanel {
            mass_kg: 50.0,
            center_of_mass_offset_m: 0.75,
            spring_constant_nm_per_rad: 500.0,
            damping_nm_s_per_rad: 0.0,
            inertia_about_panel_com_panel_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                50.0, 25.0, 25.0,
            )),
            theta_init_rad: 0.0,
            theta_dot_init_radps: 0.0,
            theta_rest_rad: 0.0,
        }
    }

    /// Upstream `nHingedRigidBody`: a 4-panel and a 3-panel chain on a tumbling
    /// hub for 1 s at a 0.1 ms step.
    fn n_hinged_rigid_body_test(gravity: bool) {
        let mut config1 = NHingedRigidBodyConfig::new("effector1");
        config1.hinge_position_body_m = Vector3::new(0.5, 0.0, 1.0);
        config1.body_to_hinge_dcm = Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0));
        let mut effector1 = NHingedRigidBodyStateEffector::new(config1);
        let mut config2 = NHingedRigidBodyConfig::new("effector2");
        config2.hinge_position_body_m = Vector3::new(-0.5, 0.0, 1.0);
        let mut effector2 = NHingedRigidBodyStateEffector::new(config2);

        let mut panel = upstream_panel();
        panel.theta_init_rad = 5.0 * PI / 180.0;
        effector1.add_hinged_panel(panel.clone());
        panel.theta_init_rad = 0.0;
        for _ in 0..3 {
            effector1.add_hinged_panel(panel.clone());
            effector2.add_hinged_panel(panel.clone());
        }

        let mut hub = HubSetup::hinged_body_hub();
        hub.r_cn_n_m = Vector3::new(0.1, -0.4, 0.3);
        hub.v_cn_n_mps = Vector3::new(-0.2, 0.5, 0.1);
        hub.omega_bn_b_radps = Vector3::new(0.1, -0.1, 0.1);
        if gravity {
            hub = hub.in_gravity_case_orbit();
        }
        let mut spacecraft = spacecraft_with_effectors(
            &hub,
            100_000,
            vec![Box::new(effector1), Box::new(effector2)],
        );
        if gravity {
            add_point_mass_earth(&mut spacecraft);
        }

        let log = run_logging_every_step(&mut spacecraft, 10_000, || {});

        assert_conserved(&log[0], &log[log.len() - 1], 1e-10, true);
    }

    #[test]
    fn n_hinged_rigid_body_conserves_energy_and_momentum_without_gravity() {
        n_hinged_rigid_body_test(false);
    }

    #[test]
    fn n_hinged_rigid_body_conserves_energy_and_momentum_with_gravity() {
        n_hinged_rigid_body_test(true);
    }

    /// Upstream `test_nHingedRigidBodyOutputMessagesMatchDual`: a two-panel
    /// chain and the equivalent dual-hinged effector on one spacecraft agree on
    /// every output over two 10 ms steps.
    #[test]
    fn n_hinged_rigid_body_output_messages_match_dual() {
        let panel_inertia = Matrix3::from_diagonal(&Vector3::new(50.0, 25.0, 25.0));
        let theta_init_rad = [0.1, -0.2];
        let theta_dot_init_radps = [-0.01, 0.02];
        let hinge_position_body_m = Vector3::new(0.5, -1.5, -0.5);

        let mut n_config = NHingedRigidBodyConfig::new("nHinged");
        n_config.hinge_position_body_m = hinge_position_body_m;
        let mut n_hinged = NHingedRigidBodyStateEffector::new(n_config);
        let mut dual_config = DualHingedRigidBodyConfig::new("dualHinged");
        let mut dual_panels = [
            DualHingedPanelConfig::default(),
            DualHingedPanelConfig::default(),
        ];
        for index in 0..2 {
            n_hinged.add_hinged_panel(HingedPanel {
                mass_kg: 50.0,
                center_of_mass_offset_m: 0.75,
                spring_constant_nm_per_rad: 100.0,
                damping_nm_s_per_rad: 0.0,
                inertia_about_panel_com_panel_kg_m2: panel_inertia,
                theta_init_rad: theta_init_rad[index],
                theta_dot_init_radps: theta_dot_init_radps[index],
                theta_rest_rad: 0.0,
            });
            dual_panels[index] = DualHingedPanelConfig {
                mass_kg: 50.0,
                center_of_mass_offset_m: 0.75,
                spring_constant_nm_per_rad: 100.0,
                damping_nm_s_per_rad: 0.0,
                inertia_about_panel_com_panel_kg_m2: panel_inertia,
                theta_init_rad: theta_init_rad[index],
                theta_dot_init_radps: theta_dot_init_radps[index],
            };
        }
        let [panel1, panel2] = dual_panels;
        dual_config.panel1 = panel1;
        dual_config.panel2 = panel2;
        dual_config.hinge_separation_m = 1.5;
        dual_config.hinge_position_body_m = hinge_position_body_m;
        let dual_hinged = DualHingedRigidBodyStateEffector::new(dual_config);

        let n_hinges = n_hinged.hinged_rigid_body_out.clone();
        let n_states = n_hinged.panel_state_out.clone();
        let dual_hinges = dual_hinged.hinged_rigid_body_out.clone();
        let dual_states = dual_hinged.panel_state_out.clone();

        let mut hub = HubSetup::hinged_body_hub();
        hub.r_cn_n_m = Vector3::new(1000.0, -2000.0, 3000.0);
        hub.v_cn_n_mps = Vector3::new(-2.0, 3.0, 1.0);
        hub.sigma_bn = Vector3::new(0.1, -0.2, 0.05);
        hub.omega_bn_b_radps = Vector3::new(0.1, -0.1, 0.2);
        let mut spacecraft = spacecraft_with_effectors(
            &hub,
            10_000_000,
            vec![Box::new(n_hinged), Box::new(dual_hinged)],
        );

        // Per panel and instant: (θ, θ̇, panel state) of the N-hinged and the
        // dual-hinged effector.
        type Sample = (f64, f64, SpacecraftStateMsg);
        let mut samples: Vec<[(Sample, Sample); 2]> = Vec::new();
        run_logging_every_step(&mut spacecraft, 2, || {
            let sample = |index: usize| {
                let n_hinge = n_hinges[index].read();
                let dual_hinge = dual_hinges[index].read();
                (
                    (
                        n_hinge.theta_rad,
                        n_hinge.theta_dot_radps,
                        n_states[index].read(),
                    ),
                    (
                        dual_hinge.theta_rad,
                        dual_hinge.theta_dot_radps,
                        dual_states[index].read(),
                    ),
                )
            };
            samples.push([sample(0), sample(1)]);
        });

        let accuracy = 1e-13;
        let close = |a: &[f64], b: &[f64], what: &str| {
            for (a, b) in a.iter().zip(b) {
                assert!((a - b).abs() <= accuracy, "{what}: {a:.17e} vs {b:.17e}");
            }
        };
        for panels in &samples {
            for ((n_theta, n_theta_dot, n_state), (d_theta, d_theta_dot, d_state)) in panels {
                close(&[*n_theta], &[*d_theta], "theta");
                close(&[*n_theta_dot], &[*d_theta_dot], "thetaDot");
                close(
                    n_state.position_m.as_slice(),
                    d_state.position_m.as_slice(),
                    "r_BN_N",
                );
                close(
                    n_state.velocity_mps.as_slice(),
                    d_state.velocity_mps.as_slice(),
                    "v_BN_N",
                );
                close(
                    n_state.sigma_bn.as_slice(),
                    d_state.sigma_bn.as_slice(),
                    "sigma_BN",
                );
                close(
                    n_state.omega_radps.as_slice(),
                    d_state.omega_radps.as_slice(),
                    "omega_BN_B",
                );
            }
        }
    }

    fn validation_chain(chain: &str) -> NHingedRigidBodyStateEffector {
        let mut effector = NHingedRigidBodyStateEffector::new(NHingedRigidBodyConfig::new(chain));
        if chain != "NoPanels" {
            for factor in [1.0, 2.0] {
                let mass_scale = match chain {
                    "MasslessPanels" => 0.0,
                    "UnequalMass" => factor,
                    _ => 1.0,
                };
                let distance_scale = if chain == "UnequalDistance" {
                    factor
                } else {
                    1.0
                };
                let inertia_scale = if chain == "UnequalInertia" {
                    factor
                } else {
                    1.0
                };
                effector.add_hinged_panel(HingedPanel {
                    mass_kg: 50.0 * mass_scale,
                    center_of_mass_offset_m: 0.75 * distance_scale,
                    spring_constant_nm_per_rad: 500.0,
                    damping_nm_s_per_rad: 0.0,
                    inertia_about_panel_com_panel_kg_m2: inertia_scale
                        * Matrix3::from_diagonal(&Vector3::new(50.0, 25.0, 25.0)),
                    ..HingedPanel::default()
                });
            }
        }
        effector
    }

    /// Upstream `test_nHingedRigidBodyPanelValidation`.
    #[test]
    fn n_hinged_rigid_body_panel_validation() {
        let cases = [
            ("UnequalMass", Some("same mass and the same hinge")),
            ("UnequalDistance", Some("same mass and the same hinge")),
            ("NoPanels", Some("at least one hinged panel")),
            (
                "MasslessPanels",
                Some("panel mass must be finite and greater than 0"),
            ),
            ("UnequalInertia", None),
        ];
        for (chain, expected_error) in cases {
            let result = validation_chain(chain).validate();
            match expected_error {
                Some(expected) => assert!(
                    result
                        .as_ref()
                        .is_err_and(|message| message.contains(expected)),
                    "{chain}: expected an error containing '{expected}', got {result:?}"
                ),
                None => assert_eq!(result, Ok(()), "{chain}"),
            }
        }
    }

    #[test]
    #[should_panic(expected = "same mass and the same hinge")]
    fn spacecraft_initialization_rejects_uneven_chain() {
        let _ = validation_chain("UnequalMass").initial_state();
    }

    /// Upstream `test_nHingedRigidBodyDefaultConfig.py::test_defaultDcmIsIdentity`.
    #[test]
    fn default_dcm_is_identity() {
        let effector = NHingedRigidBodyStateEffector::new(NHingedRigidBodyConfig::new("chain"));
        assert_eq!(effector.config.body_to_hinge_dcm, Matrix3::identity());
    }

    #[test]
    fn straight_chain_mass_properties() {
        let mut effector = NHingedRigidBodyStateEffector::new(NHingedRigidBodyConfig::new("chain"));
        for _ in 0..3 {
            effector.add_hinged_panel(upstream_panel());
        }

        let props = effector.mass_properties(&[0.0; 6]);

        // Panels along −x at 0.75, 2.25 and 3.75 m.
        assert!((props.mass_kg - 150.0).abs() < 1e-12);
        assert!((props.center_of_mass_body_m - Vector3::new(-2.25, 0.0, 0.0)).norm() < 1e-12);
        let sum_x_squared = 0.75_f64.powi(2) + 2.25_f64.powi(2) + 3.75_f64.powi(2);
        let expected: Matrix3<f64> = 3.0 * Matrix3::from_diagonal(&Vector3::new(50.0, 25.0, 25.0))
            + 50.0 * Matrix3::from_diagonal(&Vector3::new(0.0, sum_x_squared, sum_x_squared));
        assert!((props.inertia_about_point_b_body_kg_m2 - expected).norm() < 1e-10);
        assert_eq!(effector.state_len(), 6);
        assert_eq!(effector.hinged_rigid_body_out.len(), 3);
        assert_eq!(effector.panel_state_out.len(), 3);
    }

    /// A three-panel chain with distinct inertias, springs and dampers, and
    /// the equivalent three-body spinning chain: each body turning about its
    /// frame's y axis, its center of mass `d` along -x, the next joint `2d`
    /// along -x.
    fn panel_and_spinner_chains() -> (
        NHingedRigidBodyStateEffector,
        crate::dynamics::spinning_body_n_dof_state_effector::SpinningBodyNDofStateEffector,
    ) {
        use crate::dynamics::spinning_body_n_dof_state_effector::{
            SpinningBody, SpinningBodyNDofStateEffector,
        };
        let mut config = NHingedRigidBodyConfig::new("panels");
        config.hinge_position_body_m = Vector3::new(0.5, -1.5, -0.5);
        config.body_to_hinge_dcm = Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
        let mut panels = NHingedRigidBodyStateEffector::new(config.clone());
        let mut spinners = SpinningBodyNDofStateEffector::new("spinners");
        let d = 0.75;
        for index in 0..3 {
            let inertia = Matrix3::from_diagonal(&Vector3::new(
                100.0 + 10.0 * index as f64,
                50.0 - 5.0 * index as f64,
                60.0 + 3.0 * index as f64,
            ));
            let panel = HingedPanel {
                mass_kg: 100.0,
                center_of_mass_offset_m: d,
                spring_constant_nm_per_rad: 100.0 + 20.0 * index as f64,
                damping_nm_s_per_rad: 3.0 - index as f64,
                inertia_about_panel_com_panel_kg_m2: inertia,
                ..HingedPanel::default()
            };
            spinners.add_spinning_body(SpinningBody {
                mass_kg: panel.mass_kg,
                inertia_about_com_spinner_kg_m2: inertia,
                parent_to_spinner0_dcm: if index == 0 {
                    config.body_to_hinge_dcm
                } else {
                    Matrix3::identity()
                },
                center_of_mass_spinner_m: Vector3::new(-d, 0.0, 0.0),
                origin_parent_m: if index == 0 {
                    config.hinge_position_body_m
                } else {
                    Vector3::new(-2.0 * d, 0.0, 0.0)
                },
                spin_axis_spinner: Vector3::y(),
                spring_constant_nm_per_rad: panel.spring_constant_nm_per_rad,
                damping_nm_s_per_rad: panel.damping_nm_s_per_rad,
                ..SpinningBody::default()
            });
            panels.add_hinged_panel(panel);
        }
        (panels, spinners)
    }

    fn ext_force_torque(
        force_inertial_n: Vector3<f64>,
        force_body_n: Vector3<f64>,
        torque_body_nm: Vector3<f64>,
    ) -> ExtForceTorque {
        ExtForceTorque::new(ExtForceTorqueConfig {
            name: "extFT".to_string(),
            ext_force_inertial_n: force_inertial_n,
            ext_force_body_n: force_body_n,
            ext_torque_body_nm: torque_body_nm,
        })
    }

    /// Upstream `test_nHingedRigidBodyDynamicEffectorSegmentBounds` (with
    /// zero-based panel indices): the first and last panels take an
    /// effector.
    #[test]
    fn dynamic_effectors_only_attach_to_existing_panels() {
        let (mut panels, _) = panel_and_spinner_chains();
        panels.add_dynamic_effector(
            0,
            ext_force_torque(Vector3::zeros(), Vector3::x(), Vector3::zeros()),
        );
        panels.add_dynamic_effector(
            2,
            ext_force_torque(Vector3::zeros(), Vector3::x(), Vector3::zeros()),
        );
        assert_eq!(
            (0..3)
                .map(|index| panels.dynamic_effectors(index).len())
                .collect::<Vec<_>>(),
            [1, 0, 1]
        );
    }

    /// The same check's out-of-range side.
    #[test]
    #[should_panic(expected = "non-existent panel")]
    fn dynamic_effectors_reject_a_missing_panel() {
        let (mut panels, _) = panel_and_spinner_chains();
        panels.add_dynamic_effector(
            3,
            ext_force_torque(Vector3::zeros(), Vector3::x(), Vector3::zeros()),
        );
    }

    /// An `extForceTorque` attached to each panel (a force and a torque in
    /// panel axes at the hinge, and an inertial force) enters the hinge and
    /// hub equations as the same effector attached to each body of the
    /// equivalent spinning-body chain does; without effectors the two chains
    /// agree too.
    #[test]
    fn attached_effectors_match_the_equivalent_spinning_chain() {
        let (mut panels, mut spinners) = panel_and_spinner_chains();
        let state = [0.08, -0.05, 0.03, -0.02, 0.01, 0.04];
        let hub_state = SpacecraftStateMsg {
            sigma_bn: Vector3::new(0.1, -0.2, 0.3),
            omega_radps: Vector3::new(0.05, -0.02, 0.03),
            ..Default::default()
        };
        let gravity = Vector3::new(0.2, -0.1, 0.15);
        let accel = Vector3::new(0.3, 0.1, -0.2);
        let omega_dot = Vector3::new(-0.01, 0.02, 0.005);
        let compare = |panels: &NHingedRigidBodyStateEffector,
                       spinners: &crate::dynamics::spinning_body_n_dof_state_effector::SpinningBodyNDofStateEffector,
                       what: &str| {
            let mut panel_terms = Default::default();
            let mut spinner_terms = Default::default();
            panels.update_contributions(&state, &hub_state, gravity, &mut panel_terms);
            spinners.update_contributions(&state, &hub_state, gravity, &mut spinner_terms);
            let panel_dot = panels.compute_derivatives(&state, accel, omega_dot);
            let spinner_dot = spinners.compute_derivatives(&state, accel, omega_dot);
            for index in 3..6 {
                let scale = spinner_dot[index].abs().max(1e-3);
                assert!(
                    (panel_dot[index] - spinner_dot[index]).abs() < 1e-9 * scale,
                    "{what}: theta ddot {} vs {}",
                    panel_dot[index],
                    spinner_dot[index]
                );
            }
            for (a, b, name) in [
                (panel_terms.matrix_a, spinner_terms.matrix_a, "A"),
                (panel_terms.matrix_b, spinner_terms.matrix_b, "B"),
                (panel_terms.matrix_c, spinner_terms.matrix_c, "C"),
                (panel_terms.matrix_d, spinner_terms.matrix_d, "D"),
            ] {
                assert!((a - b).norm() <= 1e-9 * b.norm(), "{what} {name}: {a} vs {b}");
            }
            for (a, b, name) in [
                (panel_terms.vec_trans, spinner_terms.vec_trans, "vecTrans"),
                (panel_terms.vec_rot, spinner_terms.vec_rot, "vecRot"),
            ] {
                assert!((a - b).norm() <= 1e-9 * b.norm().max(1.0), "{what} {name}: {a} vs {b}");
            }
            panel_terms.vec_rot
        };
        let unloaded = compare(&panels, &spinners, "unloaded");

        let loads = [
            (
                Vector3::new(-4.0, 6.0, 2.0),
                Vector3::new(10.0, -5.0, 3.0),
                Vector3::new(2.0, -1.0, 4.0),
            ),
            (
                Vector3::zeros(),
                Vector3::new(-2.0, 4.0, 1.0),
                Vector3::new(0.5, 3.0, -1.5),
            ),
            (
                Vector3::new(1.0, -2.0, 0.5),
                Vector3::new(1.0, 1.0, -6.0),
                Vector3::new(-2.0, 0.7, 0.4),
            ),
        ];
        for (index, (force_n, force_s, torque_s)) in loads.into_iter().enumerate() {
            panels.add_dynamic_effector(index, ext_force_torque(force_n, force_s, torque_s));
            spinners.add_dynamic_effector(index, ext_force_torque(force_n, force_s, torque_s));
        }
        let loaded = compare(&panels, &spinners, "loaded");
        assert!((loaded - unloaded).norm() > 1.0);
    }
}
