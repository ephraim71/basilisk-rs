//! Two hinged rigid panels in series: port of upstream Basilisk
//! `simulation/dynamics/dualHingedRigidBodies` (`DualHingedRigidBodyStateEffector`).
//!
//! Panel 1 hinges on the hub at H1; panel 2 hinges on the far end of panel 1
//! at H2. Each hinge is a torsional spring–damper with an optional motor
//! torque, and the four states `[θ1, θ̇1, θ2, θ̇2]` are coupled to the hub by
//! back-substitution, as in Allard, Diaz and Schaub's flexible/slosh
//! formulation.
//!
//! Frames, following upstream:
//! - H1 is fixed in the body frame B by `body_to_hinge_dcm` (`dcm_H1B`) at
//!   `hinge_position_body_m` (`r_H1B_B`).
//! - Panel 1's frame S1 is H1 rotated by θ1 about the hinge axis ŝ12
//!   (`[S1 H1] = M2(θ1)`). Its center of mass lies `d1` along −ŝ11 from H1, and
//!   H2 lies `l1` along −ŝ11.
//! - H2 is S1 rotated by the fixed offset `thetaH2S1` about ŝ12, and panel 2's
//!   frame S2 is H2 rotated by θ2 (`[S2 H2] = M2(θ2)`), so θ2 is the angle of
//!   panel 2 relative to panel 1. Its center of mass lies `d2` along −ŝ21 from
//!   H2.
//!
//! Spring torques are `−k1 θ1` and `−k2 θ2` (rest angles are zero). The motor
//! torque `u1` acts between hub and panel 1, `u2` between the panels.
//!
//! Differences from upstream:
//! - Upstream's `addDynamicEffector` (dynamic effectors attached to a panel) is
//!   not ported, as in [`super::hinged_rigid_body_state_effector`]: there is no
//!   panel-level effector attachment in the spacecraft model yet, so the
//!   attached-load terms are zero.
//! - The motor torques are read in `pre_integration`, before each integration
//!   step, whenever `motor_torque_in` is connected (upstream reads them in
//!   `UpdateState` when the message is linked).

use nalgebra::{Matrix2, Matrix2x3, Matrix3, Vector2, Vector3};
use std::any::Any;
use std::cell::RefCell;

use crate::kinematics::is_rotation_matrix;
use crate::messages::{ArrayMotorTorqueMsg, HingedRigidBodyMsg, Input, Output, SpacecraftStateMsg};
use crate::spacecraft::{BackSubMatrices, StateEffector, StateEffectorMassProps};
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, euler2, mrp_to_dcm, tilde};

/// Mass, joint and initial-state parameters of one panel of a
/// [`DualHingedRigidBodyStateEffector`]. Upstream keeps them as `mass1`,
/// `d1`, ... and `mass2`, `d2`, ...
#[derive(Clone, Debug)]
pub struct DualHingedPanelConfig {
    /// Upstream `mass1`/`mass2` \[kg\] (default 0).
    pub mass_kg: f64,
    /// Upstream `d1`/`d2` \[m\]: distance from the panel's hinge to its center
    /// of mass, along −ŝ1 (default 1).
    pub center_of_mass_offset_m: f64,
    /// Upstream `k1`/`k2` \[N m/rad\]: torsional spring constant of the
    /// panel's hinge (default 1).
    pub spring_constant_nm_per_rad: f64,
    /// Upstream `c1`/`c2` \[N m s/rad\]: rotational damping coefficient of the
    /// panel's hinge (default 0).
    pub damping_nm_s_per_rad: f64,
    /// Upstream `IPntS1_S1`/`IPntS2_S2` \[kg m^2\]: inertia about the panel
    /// center of mass in panel-frame components (default identity).
    pub inertia_about_panel_com_panel_kg_m2: Matrix3<f64>,
    /// Upstream `theta1Init`/`theta2Init` \[rad\] (default 0).
    pub theta_init_rad: f64,
    /// Upstream `theta1DotInit`/`theta2DotInit` \[rad/s\] (default 0).
    pub theta_dot_init_radps: f64,
}

impl Default for DualHingedPanelConfig {
    fn default() -> Self {
        Self {
            mass_kg: 0.0,
            center_of_mass_offset_m: 1.0,
            spring_constant_nm_per_rad: 1.0,
            damping_nm_s_per_rad: 0.0,
            inertia_about_panel_com_panel_kg_m2: Matrix3::identity(),
            theta_init_rad: 0.0,
            theta_dot_init_radps: 0.0,
        }
    }
}

/// User-settable parameters of [`DualHingedRigidBodyStateEffector`].
#[derive(Clone, Debug)]
pub struct DualHingedRigidBodyConfig {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// The panel hinged on the hub.
    pub panel1: DualHingedPanelConfig,
    /// The panel hinged on the far end of panel 1.
    pub panel2: DualHingedPanelConfig,
    /// Upstream `l1` \[m\]: distance from hinge H1 to hinge H2 (default 0).
    pub hinge_separation_m: f64,
    /// Upstream `r_H1B_B` \[m\]: hinge H1 relative to the body origin
    /// (default zero).
    pub hinge_position_body_m: Vector3<f64>,
    /// Upstream `dcm_H1B`: DCM from the body frame to the hinge-1 frame
    /// (default identity).
    pub body_to_hinge_dcm: Matrix3<f64>,
    /// Upstream `thetaH2S1` \[rad\]: fixed rotation of the hinge-2 frame about
    /// ŝ12 relative to panel 1 (default 0).
    pub hinge2_offset_rad: f64,
}

impl DualHingedRigidBodyConfig {
    /// A configuration with upstream's default values.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            panel1: DualHingedPanelConfig::default(),
            panel2: DualHingedPanelConfig::default(),
            hinge_separation_m: 0.0,
            hinge_position_body_m: Vector3::zeros(),
            body_to_hinge_dcm: Matrix3::identity(),
            hinge2_offset_rad: 0.0,
        }
    }

    /// Upstream `validateConfiguration`: panel masses finite and non-negative,
    /// their sum finite and positive, and `body_to_hinge_dcm` a rotation matrix.
    pub fn validate(&self) -> Result<(), String> {
        for (label, mass_kg) in [
            ("panel1.mass_kg", self.panel1.mass_kg),
            ("panel2.mass_kg", self.panel2.mass_kg),
        ] {
            if !mass_kg.is_finite() || mass_kg < 0.0 {
                return Err(format!("{label} must be finite and non-negative"));
            }
        }
        let total_mass_kg = self.panel1.mass_kg + self.panel2.mass_kg;
        if !total_mass_kg.is_finite() || total_mass_kg <= 0.0 {
            return Err(
                "panel1.mass_kg + panel2.mass_kg must be finite and greater than zero".to_string(),
            );
        }
        if !is_rotation_matrix(&self.body_to_hinge_dcm) {
            return Err(
                "body_to_hinge_dcm must be a finite, orthogonal, right-handed rotation matrix"
                    .to_string(),
            );
        }
        Ok(())
    }
}

/// Two hinged rigid panels in series (upstream `DualHingedRigidBodyStateEffector`).
///
/// State layout: `[θ1, θ̇1, θ2, θ̇2]`.
#[derive(Debug)]
pub struct DualHingedRigidBodyStateEffector {
    pub config: DualHingedRigidBodyConfig,
    /// Upstream `motorTorqueInMsg` (optional): entry 0 is the hub-to-panel-1
    /// torque `u1`, entry 1 the panel-1-to-panel-2 torque `u2`.
    pub motor_torque_in: Input<ArrayMotorTorqueMsg>,
    /// Upstream `dualHingedRigidBodyOutMsgs`: angle and rate of each hinge.
    pub hinged_rigid_body_out: [Output<HingedRigidBodyMsg>; 2],
    /// Upstream `dualHingedRigidBodyConfigLogOutMsgs`: inertial state of each
    /// panel, with the panel frame Si as body frame and its center of mass as
    /// position.
    pub panel_state_out: [Output<SpacecraftStateMsg>; 2],
    /// Upstream `theta1`, `theta2` \[rad\] of the last loaded state.
    pub theta_rad: [f64; 2],
    /// Upstream `theta1Dot`, `theta2Dot` \[rad/s\] of the last loaded state.
    pub theta_dot_radps: [f64; 2],
    /// Upstream `u1`, `u2` \[N m\].
    pub motor_torque_nm: [f64; 2],
    back_sub: RefCell<DualHingedBackSubTerms>,
}

impl DualHingedRigidBodyStateEffector {
    pub fn new(config: DualHingedRigidBodyConfig) -> Self {
        let theta_rad = [config.panel1.theta_init_rad, config.panel2.theta_init_rad];
        let theta_dot_radps = [
            config.panel1.theta_dot_init_radps,
            config.panel2.theta_dot_init_radps,
        ];
        Self {
            config,
            motor_torque_in: Input::default(),
            hinged_rigid_body_out: Default::default(),
            panel_state_out: Default::default(),
            theta_rad,
            theta_dot_radps,
            motor_torque_nm: [0.0; 2],
            back_sub: RefCell::new(DualHingedBackSubTerms::default()),
        }
    }

    /// Inertial states of the two panels (upstream `computePanelInertialStates`)
    /// for the last loaded panel state and the given hub state.
    pub fn panel_inertial_states(&self, hub_state: &SpacecraftStateMsg) -> [SpacecraftStateMsg; 2] {
        let kinematics = self.kinematics(&self.current_state());
        let dcm_bn = mrp_to_dcm(hub_state.sigma_bn);
        let dcm_nb = dcm_bn.transpose();
        let omega_bn_b = hub_state.omega_radps;
        let omega_s1n_b = omega_bn_b + kinematics.theta_dot[0] * kinematics.s12;
        let omega_s2n_b = omega_s1n_b + kinematics.theta_dot[1] * kinematics.s22;
        let r_dot_h1b_b = omega_bn_b.cross(&self.config.hinge_position_body_m);
        let r_dot_h2b_b =
            r_dot_h1b_b + omega_s1n_b.cross(&(-self.config.hinge_separation_m * kinematics.s11));
        let d1 = self.config.panel1.center_of_mass_offset_m;
        let d2 = self.config.panel2.center_of_mass_offset_m;
        let v_s1b_b = r_dot_h1b_b + omega_s1n_b.cross(&(-d1 * kinematics.s11));
        let v_s2b_b = r_dot_h2b_b + omega_s2n_b.cross(&(-d2 * kinematics.s21));

        [
            SpacecraftStateMsg {
                position_m: hub_state.position_m + dcm_nb * kinematics.r_s1b_b,
                velocity_mps: hub_state.velocity_mps + dcm_nb * v_s1b_b,
                sigma_bn: dcm_to_mrp(kinematics.dcm_s1b * dcm_bn),
                omega_radps: kinematics.dcm_s1b * omega_s1n_b,
                ..Default::default()
            },
            SpacecraftStateMsg {
                position_m: hub_state.position_m + dcm_nb * kinematics.r_s2b_b,
                velocity_mps: hub_state.velocity_mps + dcm_nb * v_s2b_b,
                sigma_bn: dcm_to_mrp(kinematics.dcm_s2b * dcm_bn),
                omega_radps: kinematics.dcm_s2b * omega_s2n_b,
                ..Default::default()
            },
        ]
    }

    fn current_state(&self) -> [f64; 4] {
        [
            self.theta_rad[0],
            self.theta_dot_radps[0],
            self.theta_rad[1],
            self.theta_dot_radps[1],
        ]
    }

    fn kinematics(&self, effector_state: &[f64]) -> DualHingedKinematics {
        assert_eq!(
            effector_state.len(),
            4,
            "dual hinged rigid body state length mismatch"
        );
        DualHingedKinematics::new(&self.config, effector_state)
    }

    fn write_hinge_states(&self) {
        for (output, (theta_rad, theta_dot_radps)) in self
            .hinged_rigid_body_out
            .iter()
            .zip(self.theta_rad.into_iter().zip(self.theta_dot_radps))
        {
            output.write(HingedRigidBodyMsg {
                theta_rad,
                theta_dot_radps,
            });
        }
    }

    /// Upstream `updateContributions`.
    fn back_sub_terms(
        &self,
        k: &DualHingedKinematics,
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
    ) -> DualHingedBackSubTerms {
        let p1 = &self.config.panel1;
        let p2 = &self.config.panel2;
        let (m1, m2) = (p1.mass_kg, p2.mass_kg);
        let (d1, d2) = (p1.center_of_mass_offset_m, p2.center_of_mass_offset_m);
        let l1 = self.config.hinge_separation_m;
        let i1 = p1.inertia_about_panel_com_panel_kg_m2;
        let i2 = p2.inertia_about_panel_com_panel_kg_m2;
        let [theta1, theta2] = k.theta;
        let [theta1_dot, theta2_dot] = k.theta_dot;
        let theta12_dot = theta1_dot + theta2_dot;
        let [u1, u2] = self.motor_torque_nm;
        let omega = body_omega_radps;

        let grav_torque_pan1_pnt_h1 = -d1 * k.s11.cross(&(m1 * gravity_body_mps2));
        let grav_force_pan2 = m2 * gravity_body_mps2;
        let grav_torque_pan2_pnt_h2 = -d2 * k.s21.cross(&(m2 * gravity_body_mps2));

        let omega_s1 = k.dcm_s1b * omega;
        let omega_s2 = k.dcm_s2b * omega;

        let coupling = m2 * l1 * d2 * k.s13.dot(&k.s23);
        let matrix_a = Matrix2::new(
            i1[(1, 1)] + m1 * d1 * d1 + m2 * l1 * l1 + coupling,
            coupling,
            i2[(1, 1)] + m2 * d2 * d2 + m2 * l1 * d2 * k.s23.dot(&k.s13),
            i2[(1, 1)] + m2 * d2 * d2,
        );
        let matrix_e = matrix_a
            .try_inverse()
            .expect("dual hinged rigid body panel mass matrix must be invertible");
        let matrix_f = Matrix2x3::from_rows(&[
            (-(m2 * l1 + m1 * d1) * k.s13).transpose(),
            (-m2 * d2 * k.s23).transpose(),
        ]);
        let matrix_g = Matrix2x3::from_rows(&[
            (-(i1[(1, 1)] * k.s12
                - m1 * d1 * k.s13.cross(&k.r_s1b_b)
                - m2 * l1 * k.s13.cross(&k.r_s2b_b)))
            .transpose(),
            (-(i2[(1, 1)] * k.s22 - m2 * d2 * k.s23.cross(&k.r_s2b_b))).transpose(),
        ]);

        let accel_s1 = 2.0 * omega.cross(&k.r_prime_s1b_b) + omega.cross(&omega.cross(&k.r_s1b_b));
        let accel_s2 = 2.0 * omega.cross(&k.r_prime_s2b_b) + omega.cross(&omega.cross(&k.r_s2b_b));
        let vector_v = Vector2::new(
            -(i1[(0, 0)] - i1[(2, 2)]) * omega_s1.z * omega_s1.x + u1
                - u2
                - p1.spring_constant_nm_per_rad * theta1
                - p1.damping_nm_s_per_rad * theta1_dot
                + p2.spring_constant_nm_per_rad * theta2
                + p2.damping_nm_s_per_rad * theta2_dot
                + k.s12.dot(&grav_torque_pan1_pnt_h1)
                + l1 * k.s13.dot(&grav_force_pan2)
                - m1 * d1 * k.s13.dot(&accel_s1)
                - m2 * l1
                    * k.s13.dot(
                        &(accel_s2
                            + l1 * theta1_dot * theta1_dot * k.s11
                            + d2 * theta12_dot * theta12_dot * k.s21),
                    ),
            -(i2[(0, 0)] - i2[(2, 2)]) * omega_s2.z * omega_s2.x + u2
                - p2.spring_constant_nm_per_rad * theta2
                - p2.damping_nm_s_per_rad * theta2_dot
                + k.s22.dot(&grav_torque_pan2_pnt_h2)
                - m2 * d2
                    * k.s23
                        .dot(&(accel_s2 + l1 * theta1_dot * theta1_dot * k.s11)),
        );

        let theta_ddot_accel = matrix_e * matrix_f;
        let theta_ddot_omega_dot = matrix_e * matrix_g;
        let theta_ddot_bias = matrix_e * vector_v;

        let trans_factor0 = m1 * d1 * k.s13 + m2 * l1 * k.s13 + m2 * d2 * k.s23;
        let trans_factor1 = m2 * d2 * k.s23;
        let rot_factor0 = i1[(1, 1)] * k.s12
            + m1 * d1 * k.r_s1b_b.cross(&k.s13)
            + i2[(1, 1)] * k.s22
            + m2 * l1 * k.r_s2b_b.cross(&k.s13)
            + m2 * d2 * k.r_s2b_b.cross(&k.s23);
        let rot_factor1 = i2[(1, 1)] * k.s22 + m2 * d2 * k.r_s2b_b.cross(&k.s23);
        let tip_accel_s2 =
            l1 * theta1_dot * theta1_dot * k.s11 + d2 * theta12_dot * theta12_dot * k.s21;

        DualHingedBackSubTerms {
            matrix_a: trans_factor0 * theta_ddot_accel.row(0)
                + trans_factor1 * theta_ddot_accel.row(1),
            matrix_b: trans_factor0 * theta_ddot_omega_dot.row(0)
                + trans_factor1 * theta_ddot_omega_dot.row(1),
            matrix_c: rot_factor0 * theta_ddot_accel.row(0) + rot_factor1 * theta_ddot_accel.row(1),
            matrix_d: rot_factor0 * theta_ddot_omega_dot.row(0)
                + rot_factor1 * theta_ddot_omega_dot.row(1),
            vec_trans: -(m1 * d1 * theta1_dot * theta1_dot * k.s11
                + m2 * tip_accel_s2
                + trans_factor0 * theta_ddot_bias[0]
                + trans_factor1 * theta_ddot_bias[1]),
            vec_rot: -(theta1_dot * i1[(1, 1)] * omega.cross(&k.s12)
                + m1 * omega.cross(&k.r_s1b_b.cross(&k.r_prime_s1b_b))
                + m1 * d1 * theta1_dot * theta1_dot * k.r_s1b_b.cross(&k.s11)
                + theta12_dot * i2[(1, 1)] * omega.cross(&k.s22)
                + m2 * omega.cross(&k.r_s2b_b.cross(&k.r_prime_s2b_b))
                + m2 * k.r_s2b_b.cross(&tip_accel_s2)
                + rot_factor0 * theta_ddot_bias[0]
                + rot_factor1 * theta_ddot_bias[1]),
            theta_ddot_accel,
            theta_ddot_omega_dot,
            theta_ddot_bias,
        }
    }

    /// Panel inertias about their centers of mass and inertial angular
    /// velocities and center-of-mass velocities, in body-frame components.
    fn panel_motion(
        &self,
        k: &DualHingedKinematics,
        body_omega_radps: Vector3<f64>,
    ) -> [(Matrix3<f64>, Vector3<f64>, Vector3<f64>); 2] {
        let i1 = self.config.panel1.inertia_about_panel_com_panel_kg_m2;
        let i2 = self.config.panel2.inertia_about_panel_com_panel_kg_m2;
        [
            (
                k.dcm_s1b.transpose() * i1 * k.dcm_s1b,
                k.theta_dot[0] * k.s12 + body_omega_radps,
                k.r_prime_s1b_b + body_omega_radps.cross(&k.r_s1b_b),
            ),
            (
                k.dcm_s2b.transpose() * i2 * k.dcm_s2b,
                (k.theta_dot[0] + k.theta_dot[1]) * k.s22 + body_omega_radps,
                k.r_prime_s2b_b + body_omega_radps.cross(&k.r_s2b_b),
            ),
        ]
    }
}

impl StateEffector for DualHingedRigidBodyStateEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn state_len(&self) -> usize {
        4
    }

    /// Validates the configuration (upstream `registerStates`) and returns the
    /// initial `[θ1, θ̇1, θ2, θ̇2]`.
    ///
    /// # Panics
    ///
    /// If [`DualHingedRigidBodyConfig::validate`] fails.
    fn initial_state(&self) -> Vec<f64> {
        if let Err(message) = self.config.validate() {
            panic!(
                "invalid DualHingedRigidBodyConfig '{}': {message}",
                self.config.name
            );
        }
        vec![
            self.config.panel1.theta_init_rad,
            self.config.panel1.theta_dot_init_radps,
            self.config.panel2.theta_init_rad,
            self.config.panel2.theta_dot_init_radps,
        ]
    }

    fn load_state(&mut self, state: &[f64]) {
        assert_eq!(
            state.len(),
            self.state_len(),
            "dual hinged rigid body state length mismatch"
        );
        self.theta_rad = [state[0], state[2]];
        self.theta_dot_radps = [state[1], state[3]];
        self.write_hinge_states();
    }

    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {
        if self.motor_torque_in.is_connected() {
            let torques = self.motor_torque_in.read().motor_torque_nm;
            self.motor_torque_nm = [torques[0], torques[1]];
        }
    }

    fn update_contributions(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        let kinematics = self.kinematics(effector_state);
        let terms = self.back_sub_terms(&kinematics, body_omega_radps, gravity_body_mps2);
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
            "dual hinged rigid body state length mismatch"
        );
        let back_sub = self.back_sub.borrow();
        let theta_ddot = back_sub.theta_ddot_accel * body_trans_accel_mps2
            + back_sub.theta_ddot_omega_dot * body_omega_dot_radps2
            + back_sub.theta_ddot_bias;
        vec![
            effector_state[1],
            theta_ddot[0],
            effector_state[3],
            theta_ddot[1],
        ]
    }

    /// Upstream `updateEffectorMassProps`.
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        let k = self.kinematics(effector_state);
        let p1 = &self.config.panel1;
        let p2 = &self.config.panel2;
        let (m1, m2) = (p1.mass_kg, p2.mass_kg);
        let mass_kg = m1 + m2;
        let r_tilde_s1 = tilde(k.r_s1b_b);
        let r_tilde_s2 = tilde(k.r_s2b_b);
        let r_prime_tilde_s1 = tilde(k.r_prime_s1b_b);
        let r_prime_tilde_s2 = tilde(k.r_prime_s2b_b);
        let i1 = p1.inertia_about_panel_com_panel_kg_m2;
        let i2 = p2.inertia_about_panel_com_panel_kg_m2;
        let i1_prime = k.theta_dot[0]
            * (i1[(2, 2)] - i1[(0, 0)])
            * (k.s11 * k.s13.transpose() + k.s13 * k.s11.transpose());
        let i2_prime = (k.theta_dot[0] + k.theta_dot[1])
            * (i2[(2, 2)] - i2[(0, 0)])
            * (k.s21 * k.s23.transpose() + k.s23 * k.s21.transpose());

        StateEffectorMassProps {
            mass_kg,
            mass_dot_kgps: 0.0,
            center_of_mass_body_m: (m1 * k.r_s1b_b + m2 * k.r_s2b_b) / mass_kg,
            center_of_mass_prime_body_mps: (m1 * k.r_prime_s1b_b + m2 * k.r_prime_s2b_b) / mass_kg,
            inertia_about_point_b_body_kg_m2: k.dcm_s1b.transpose() * i1 * k.dcm_s1b
                + m1 * r_tilde_s1 * r_tilde_s1.transpose()
                + k.dcm_s2b.transpose() * i2 * k.dcm_s2b
                + m2 * r_tilde_s2 * r_tilde_s2.transpose(),
            inertia_about_point_b_body_prime_kg_m2ps: i1_prime
                - m1 * (r_prime_tilde_s1 * r_tilde_s1 + r_tilde_s1 * r_prime_tilde_s1)
                + i2_prime
                - m2 * (r_prime_tilde_s2 * r_tilde_s2 + r_tilde_s2 * r_prime_tilde_s2),
        }
    }

    /// Upstream `updateEnergyMomContributions` (angular momentum about B).
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        let k = self.kinematics(effector_state);
        let masses = [self.config.panel1.mass_kg, self.config.panel2.mass_kg];
        let positions = [k.r_s1b_b, k.r_s2b_b];
        self.panel_motion(&k, body_omega_radps)
            .into_iter()
            .zip(masses.into_iter().zip(positions))
            .map(|((inertia, omega_sn, r_dot), (mass_kg, r_sb))| {
                inertia * omega_sn + mass_kg * r_sb.cross(&r_dot)
            })
            .sum()
    }

    /// Upstream `updateEnergyMomContributions`: kinetic energy relative to B
    /// plus the hinge spring energy.
    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        let k = self.kinematics(effector_state);
        let panels = [&self.config.panel1, &self.config.panel2];
        self.panel_motion(&k, body_omega_radps)
            .into_iter()
            .zip(panels.into_iter().zip(k.theta))
            .map(|((inertia, omega_sn, r_dot), (panel, theta_rad))| {
                0.5 * omega_sn.dot(&(inertia * omega_sn))
                    + 0.5 * panel.mass_kg * r_dot.dot(&r_dot)
                    + 0.5 * panel.spring_constant_nm_per_rad * theta_rad * theta_rad
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
#[derive(Clone, Debug, Default)]
struct DualHingedBackSubTerms {
    matrix_a: Matrix3<f64>,
    matrix_b: Matrix3<f64>,
    matrix_c: Matrix3<f64>,
    matrix_d: Matrix3<f64>,
    vec_trans: Vector3<f64>,
    vec_rot: Vector3<f64>,
    /// `E F` (upstream `matrixEDHRB * matrixFDHRB`).
    theta_ddot_accel: Matrix2x3<f64>,
    /// `E G` (upstream `matrixEDHRB * matrixGDHRB`).
    theta_ddot_omega_dot: Matrix2x3<f64>,
    /// `E v` (upstream `matrixEDHRB * vectorVDHRB`).
    theta_ddot_bias: Vector2<f64>,
}

/// Panel geometry for one state (upstream `updateEffectorMassProps`). Hub
/// quantities carry upstream's P (primary body) frame, which is B here.
#[derive(Clone, Debug)]
struct DualHingedKinematics {
    theta: [f64; 2],
    theta_dot: [f64; 2],
    /// Upstream `dcm_S1P`.
    dcm_s1b: Matrix3<f64>,
    /// Upstream `dcm_S2P`.
    dcm_s2b: Matrix3<f64>,
    s11: Vector3<f64>,
    s12: Vector3<f64>,
    s13: Vector3<f64>,
    s21: Vector3<f64>,
    s22: Vector3<f64>,
    s23: Vector3<f64>,
    /// Upstream `r_S1P_P`.
    r_s1b_b: Vector3<f64>,
    /// Upstream `r_S2P_P`.
    r_s2b_b: Vector3<f64>,
    /// Upstream `rPrimeS1P_P`.
    r_prime_s1b_b: Vector3<f64>,
    /// Upstream `rPrimeS2P_P`.
    r_prime_s2b_b: Vector3<f64>,
}

impl DualHingedKinematics {
    fn new(config: &DualHingedRigidBodyConfig, state: &[f64]) -> Self {
        let theta = [state[0], state[2]];
        let theta_dot = [state[1], state[3]];
        let dcm_s1b = euler2(theta[0]) * config.body_to_hinge_dcm;
        let dcm_h2b = euler2(config.hinge2_offset_rad) * dcm_s1b;
        let dcm_s2b = euler2(theta[1]) * dcm_h2b;
        let s11: Vector3<f64> = dcm_s1b.row(0).transpose();
        let s12: Vector3<f64> = dcm_s1b.row(1).transpose();
        let s13: Vector3<f64> = dcm_s1b.row(2).transpose();
        let s21: Vector3<f64> = dcm_s2b.row(0).transpose();
        let s22: Vector3<f64> = dcm_s2b.row(1).transpose();
        let s23: Vector3<f64> = dcm_s2b.row(2).transpose();
        let d1 = config.panel1.center_of_mass_offset_m;
        let d2 = config.panel2.center_of_mass_offset_m;
        let l1 = config.hinge_separation_m;
        let r_h1b_b = config.hinge_position_body_m;

        Self {
            theta,
            theta_dot,
            dcm_s1b,
            dcm_s2b,
            s11,
            s12,
            s13,
            s21,
            s22,
            s23,
            r_s1b_b: r_h1b_b - d1 * s11,
            r_s2b_b: r_h1b_b - l1 * s11 - d2 * s21,
            r_prime_s1b_b: d1 * theta_dot[0] * s13,
            r_prime_s2b_b: l1 * theta_dot[0] * s13 + d2 * (theta_dot[0] + theta_dot[1]) * s23,
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_dualhingedRigidBodyStateEffector.py`.

    use super::{
        DualHingedPanelConfig, DualHingedRigidBodyConfig, DualHingedRigidBodyStateEffector,
    };
    use crate::dynamics::state_effector_test_support::{
        HubSetup, add_point_mass_earth, assert_absolute_equal, assert_conserved,
        assert_relative_equal, run_logging_every_step, spacecraft_with_effectors,
    };
    use crate::messages::{ArrayMotorTorqueMsg, Output, SpacecraftDiagnosticsMsg};
    use crate::spacecraft::StateEffector;
    use nalgebra::{Matrix3, Vector3};
    use std::f64::consts::PI;

    fn upstream_panel(spring_constant_nm_per_rad: f64) -> DualHingedPanelConfig {
        DualHingedPanelConfig {
            mass_kg: 50.0,
            center_of_mass_offset_m: 0.75,
            spring_constant_nm_per_rad,
            damping_nm_s_per_rad: 0.0,
            inertia_about_panel_com_panel_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                50.0, 25.0, 25.0,
            )),
            theta_init_rad: 0.0,
            theta_dot_init_radps: 0.0,
        }
    }

    /// Upstream `panel1` / `panel2` of the tests, with spring constants `k1`,
    /// `k2` and initial angle `theta1Init`.
    fn upstream_effector(
        name: &str,
        hinge_x_m: f64,
        flipped: bool,
        k1: f64,
        k2: f64,
        theta1_init_rad: f64,
    ) -> DualHingedRigidBodyConfig {
        let mut config = DualHingedRigidBodyConfig::new(name);
        config.panel1 = upstream_panel(k1);
        config.panel1.theta_init_rad = theta1_init_rad;
        config.panel2 = upstream_panel(k2);
        config.hinge_separation_m = 1.5;
        config.hinge_position_body_m = Vector3::new(hinge_x_m, 0.0, 1.0);
        if flipped {
            config.body_to_hinge_dcm = Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0));
        }
        config
    }

    /// Upstream `dualHingedRigidBodyTest`: two sprung dual panels on a tumbling
    /// hub for 1 s at a 0.1 ms step. Upstream compares the rotational energy at
    /// sample `len/2 + 1` rather than the first sample; kept as is.
    fn dual_hinged_rigid_body_test(gravity: bool) {
        let panel1 = upstream_effector("panel1", 0.5, true, 100.0, 100.0, 5.0 * PI / 180.0);
        let panel2 = upstream_effector("panel2", -0.5, false, 100.0, 100.0, 5.0 * PI / 180.0);
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
            vec![
                Box::new(DualHingedRigidBodyStateEffector::new(panel1)),
                Box::new(DualHingedRigidBodyStateEffector::new(panel2)),
            ],
        );
        if gravity {
            add_point_mass_earth(&mut spacecraft);
        }

        let log = run_logging_every_step(&mut spacecraft, 10_000, || {});

        let accuracy = 1e-10;
        let last = &log[log.len() - 1];
        assert_conserved(&log[0], last, accuracy, false);
        assert_relative_equal(
            &[last.rotational_energy_j],
            &[log[log.len() / 2 + 1].rotational_energy_j],
            accuracy,
            "rotational energy",
        );
    }

    #[test]
    fn dual_hinged_rigid_body_conserves_energy_and_momentum_without_gravity() {
        dual_hinged_rigid_body_test(false);
    }

    #[test]
    fn dual_hinged_rigid_body_conserves_energy_and_momentum_with_gravity() {
        dual_hinged_rigid_body_test(true);
    }

    /// Upstream `dualHingedRigidBodyMotorTorque`: motor torques `[2, 4]` N m on
    /// the first effector of a hub at rest, 10 s at a 10 ms step.
    #[test]
    fn dual_hinged_rigid_body_motor_torque() {
        let mut panel1 = DualHingedRigidBodyStateEffector::new(upstream_effector(
            "panel1", 0.5, true, 0.0, 100.0, 0.0,
        ));
        let panel2 = DualHingedRigidBodyStateEffector::new(upstream_effector(
            "panel2", -0.5, false, 0.0, 0.0, 0.0,
        ));
        let motor_torque_nm = [2.0, 4.0];
        let motor = Output::new(ArrayMotorTorqueMsg::from_active(&motor_torque_nm));
        panel1.motor_torque_in.connect(motor.slot());
        let panel1_hinges = panel1.hinged_rigid_body_out.clone();
        let panel1_states = panel1.panel_state_out.clone();
        let panel2_states = panel2.panel_state_out.clone();

        let mut hub = HubSetup::hinged_body_hub();
        hub.center_of_mass_body_m = Vector3::new(0.0, 0.0, 1.0);
        let mut spacecraft =
            spacecraft_with_effectors(&hub, 10_000_000, vec![Box::new(panel1), Box::new(panel2)]);
        let state = spacecraft.state_out.clone();
        let mass_props = spacecraft.mass_props_out.clone();

        // First samples of the config log messages and every sample of the
        // panel-1 hinge rates, as the upstream recorders keep them.
        let mut first_panel_states = None;
        let mut hinge_rates = Vec::new();
        let log = run_logging_every_step(&mut spacecraft, 1_000, || {
            first_panel_states
                .get_or_insert_with(|| (panel1_states[0].read(), panel2_states[1].read()));
            hinge_rates.push([
                panel1_hinges[0].read().theta_dot_radps,
                panel1_hinges[1].read().theta_dot_radps,
            ]);
        });

        let accuracy = 1e-10;
        // Upstream checks r_CN_N; the state message carries r_BN_N.
        let final_state = state.read();
        let final_mass_props = mass_props.read();
        let r_cn_n = final_state.position_m
            + final_state
                .body_to_inertial()
                .transform_vector(&final_mass_props.center_of_mass_body_m);
        assert_absolute_equal(r_cn_n.as_slice(), &[0.0, 0.0, 0.0], accuracy, "r_CN_N");

        // Motor work by the trapezoid rule must match the mechanical energy change.
        let step_s = 0.01;
        let motor_power_w: Vec<f64> = hinge_rates
            .iter()
            .map(|rates| motor_torque_nm[0] * rates[0] + motor_torque_nm[1] * rates[1])
            .collect();
        let motor_work_j: f64 = motor_power_w
            .windows(2)
            .map(|pair| 0.5 * (pair[0] + pair[1]) * step_s)
            .sum();
        let total_energy_j = |sample: &SpacecraftDiagnosticsMsg| {
            sample.orbital_energy_j + sample.rotational_energy_j
        };
        let delta_energy_j = total_energy_j(&log[log.len() - 1]) - total_energy_j(&log[0]);
        assert!(
            (delta_energy_j - motor_work_j).abs() <= 1e-5 * motor_work_j.abs(),
            "dE = {delta_energy_j:.6} J but the motors did {motor_work_j:.6} J of work"
        );

        assert_absolute_equal(
            log[log.len() - 1]
                .rotational_angular_momentum_inertial_kg_m2ps
                .as_slice(),
            log[0]
                .rotational_angular_momentum_inertial_kg_m2ps
                .as_slice(),
            accuracy,
            "rotational angular momentum",
        );

        let (s1, s2) = first_panel_states.expect("sampled at t = 0");
        assert_absolute_equal(
            s1.position_m.as_slice(),
            &[1.25, 0.0, 0.0],
            accuracy,
            "r_S1N_N",
        );
        assert_absolute_equal(
            s1.velocity_mps.as_slice(),
            &[0.0, 0.0, 0.0],
            accuracy,
            "v_S1N_N",
        );
        assert_absolute_equal(
            s1.sigma_bn.as_slice(),
            &[0.0, 0.0, 1.0],
            accuracy,
            "sigma_S1N",
        );
        assert_absolute_equal(
            s1.omega_radps.as_slice(),
            &[0.0, 0.0, 0.0],
            accuracy,
            "omega_S1N_S1",
        );
        assert_absolute_equal(
            s2.position_m.as_slice(),
            &[-2.75, 0.0, 0.0],
            accuracy,
            "r_S2N_N",
        );
        assert_absolute_equal(
            s2.velocity_mps.as_slice(),
            &[0.0, 0.0, 0.0],
            accuracy,
            "v_S2N_N",
        );
        assert_absolute_equal(
            s2.sigma_bn.as_slice(),
            &[0.0, 0.0, 0.0],
            accuracy,
            "sigma_S2N",
        );
        assert_absolute_equal(
            s2.omega_radps.as_slice(),
            &[0.0, 0.0, 0.0],
            accuracy,
            "omega_S2N_S2",
        );
    }

    #[test]
    fn mass_properties_place_both_panels_along_the_hinge_line() {
        let config = upstream_effector("panel", 0.5, true, 0.0, 0.0, 0.0);
        let effector = DualHingedRigidBodyStateEffector::new(config);

        let props = effector.mass_properties(&[0.0, 0.0, 0.0, 0.0]);

        // Panel centers of mass at x = 1.25 m and x = 2.75 m, both at z = 1 m.
        assert!((props.mass_kg - 100.0).abs() < 1e-12);
        assert!((props.center_of_mass_body_m - Vector3::new(2.0, 0.0, 1.0)).norm() < 1e-12);
        let expected_inertia: Matrix3<f64> = 2.0
            * Matrix3::from_diagonal(&Vector3::new(50.0, 25.0, 25.0))
            + 50.0 * Matrix3::new(1.0, 0.0, -1.25, 0.0, 1.0 + 1.5625, 0.0, -1.25, 0.0, 1.5625)
            + 50.0 * Matrix3::new(1.0, 0.0, -2.75, 0.0, 1.0 + 7.5625, 0.0, -2.75, 0.0, 7.5625);
        assert!(
            (props.inertia_about_point_b_body_kg_m2 - expected_inertia).norm() < 1e-10,
            "{:?}",
            props.inertia_about_point_b_body_kg_m2
        );
        assert!(props.center_of_mass_prime_body_mps.norm() == 0.0);
    }

    #[test]
    fn reads_both_motor_torques_when_connected() {
        let mut effector =
            DualHingedRigidBodyStateEffector::new(DualHingedRigidBodyConfig::new("p"));
        effector.pre_integration(0, 0.1);
        assert_eq!(effector.motor_torque_nm, [0.0, 0.0]);

        let motor = Output::new(ArrayMotorTorqueMsg::from_active(&[0.5, -1.5, 9.0]));
        effector.motor_torque_in.connect(motor.slot());
        effector.pre_integration(0, 0.1);
        assert_eq!(effector.motor_torque_nm, [0.5, -1.5]);
    }

    #[test]
    fn validation_rejects_invalid_masses_and_hinge_frames() {
        let mut config = upstream_effector("panel", 0.5, true, 0.0, 0.0, 0.0);
        assert_eq!(config.validate(), Ok(()));

        config.panel1.mass_kg = 0.0;
        assert_eq!(config.validate(), Ok(()), "one massless panel is allowed");
        config.panel2.mass_kg = 0.0;
        assert!(config.validate().unwrap_err().contains("greater than zero"));
        config.panel2.mass_kg = -1.0;
        assert!(config.validate().unwrap_err().contains("panel2.mass_kg"));
        config.panel2.mass_kg = f64::NAN;
        assert!(config.validate().unwrap_err().contains("panel2.mass_kg"));
        config.panel2.mass_kg = f64::MAX;
        config.panel1.mass_kg = f64::MAX;
        assert!(config.validate().unwrap_err().contains("greater than zero"));

        let mut config = upstream_effector("panel", 0.5, true, 0.0, 0.0, 0.0);
        config.body_to_hinge_dcm = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, -1.0));
        assert!(config.validate().unwrap_err().contains("rotation matrix"));
    }

    #[test]
    #[should_panic(expected = "greater than zero")]
    fn spacecraft_initialization_rejects_massless_effector() {
        let effector = DualHingedRigidBodyStateEffector::new(DualHingedRigidBodyConfig::new("p"));
        let _ = effector.initial_state();
    }
}
