use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use std::any::Any;
use std::cell::RefCell;

use crate::messages::{ArrayMotorTorqueMsg, HingedRigidBodyMsg, Input, Output, SpacecraftStateMsg};
use crate::spacecraft::{BackSubMatrices, StateEffector, StateEffectorMassProps};
use crate::{Module, SimulationContext};

const NANO2SEC: f64 = 1.0e-9;

#[derive(Clone, Debug)]
pub struct HingedBodyLinearProfiler {
    pub name: String,
    pub start_time_nanos: u64,
    pub end_time_nanos: u64,
    pub start_theta_rad: f64,
    pub end_theta_rad: f64,
    pub hinged_rigid_body_reference_out: Output<HingedRigidBodyMsg>,
    deployment_slope_radps: f64,
}

impl HingedBodyLinearProfiler {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            start_time_nanos: 0,
            end_time_nanos: 0,
            start_theta_rad: 0.0,
            end_theta_rad: 0.0,
            hinged_rigid_body_reference_out: Output::default(),
            deployment_slope_radps: 0.0,
        }
    }

    pub fn with_profile(
        name: impl Into<String>,
        start_time_nanos: u64,
        end_time_nanos: u64,
        start_theta_rad: f64,
        end_theta_rad: f64,
    ) -> Self {
        let mut profiler = Self::new(name);
        profiler.start_time_nanos = start_time_nanos;
        profiler.end_time_nanos = end_time_nanos;
        profiler.start_theta_rad = start_theta_rad;
        profiler.end_theta_rad = end_theta_rad;
        profiler
    }

    pub fn reference_at(&self, current_sim_nanos: u64) -> HingedRigidBodyMsg {
        if current_sim_nanos < self.start_time_nanos {
            HingedRigidBodyMsg {
                theta_rad: self.start_theta_rad,
                theta_dot_radps: 0.0,
            }
        } else if current_sim_nanos <= self.end_time_nanos {
            let elapsed_seconds = (current_sim_nanos - self.start_time_nanos) as f64 * NANO2SEC;
            HingedRigidBodyMsg {
                theta_rad: self.start_theta_rad + elapsed_seconds * self.deployment_slope_radps,
                theta_dot_radps: self.deployment_slope_radps,
            }
        } else {
            HingedRigidBodyMsg {
                theta_rad: self.end_theta_rad,
                theta_dot_radps: 0.0,
            }
        }
    }
}

impl Module for HingedBodyLinearProfiler {
    fn init(&mut self) {
        assert!(
            self.end_time_nanos > self.start_time_nanos,
            "hinged body profiler '{}' requires end_time_nanos > start_time_nanos",
            self.name
        );
        self.deployment_slope_radps = (self.end_theta_rad - self.start_theta_rad)
            / ((self.end_time_nanos - self.start_time_nanos) as f64 * NANO2SEC);
        self.hinged_rigid_body_reference_out
            .write(self.reference_at(0));
    }

    fn update(&mut self, context: &SimulationContext) {
        self.hinged_rigid_body_reference_out
            .write(self.reference_at(context.current_sim_nanos));
    }
}

#[derive(Clone, Debug)]
pub struct HingedRigidBodyConfig {
    pub name: String,
    pub mass_kg: f64,
    pub center_of_mass_offset_m: f64,
    pub spring_constant_nm_per_rad: f64,
    pub damping_nm_s_per_rad: f64,
    pub inertia_about_panel_com_panel_kg_m2: Matrix3<f64>,
    pub hinge_position_body_m: Vector3<f64>,
    pub body_to_hinge_dcm: Matrix3<f64>,
    pub theta_init_rad: f64,
    pub theta_dot_init_radps: f64,
    pub theta_ref_rad: f64,
    pub theta_dot_ref_radps: f64,
}

impl HingedRigidBodyConfig {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            mass_kg: 0.0,
            center_of_mass_offset_m: 1.0,
            spring_constant_nm_per_rad: 1.0,
            damping_nm_s_per_rad: 0.0,
            inertia_about_panel_com_panel_kg_m2: Matrix3::identity(),
            hinge_position_body_m: Vector3::zeros(),
            body_to_hinge_dcm: Matrix3::identity(),
            theta_init_rad: 0.0,
            theta_dot_init_radps: 0.0,
            theta_ref_rad: 0.0,
            theta_dot_ref_radps: 0.0,
        }
    }
}

#[derive(Debug)]
pub struct HingedRigidBodyStateEffector {
    pub config: HingedRigidBodyConfig,
    pub motor_torque_in: Input<ArrayMotorTorqueMsg>,
    pub hinged_rigid_body_ref_in: Input<HingedRigidBodyMsg>,
    pub hinged_rigid_body_out: Output<HingedRigidBodyMsg>,
    pub panel_state_out: Output<SpacecraftStateMsg>,
    pub theta_rad: f64,
    pub theta_dot_radps: f64,
    pub motor_torque_nm: f64,
    pub theta_ref_rad: f64,
    pub theta_dot_ref_radps: f64,
    kinematic: HingedRigidBodyKinematics,
    back_sub: RefCell<HingedRigidBodyBackSubTerms>,
}

impl HingedRigidBodyStateEffector {
    pub fn new(config: HingedRigidBodyConfig) -> Self {
        let theta_rad = config.theta_init_rad;
        let theta_dot_radps = config.theta_dot_init_radps;
        let theta_ref_rad = config.theta_ref_rad;
        let theta_dot_ref_radps = config.theta_dot_ref_radps;
        let mut effector = Self {
            config,
            motor_torque_in: Input::default(),
            hinged_rigid_body_ref_in: Input::default(),
            hinged_rigid_body_out: Output::default(),
            panel_state_out: Output::default(),
            theta_rad,
            theta_dot_radps,
            motor_torque_nm: 0.0,
            theta_ref_rad,
            theta_dot_ref_radps,
            kinematic: HingedRigidBodyKinematics::default(),
            back_sub: RefCell::new(HingedRigidBodyBackSubTerms::default()),
        };
        effector.update_kinematics(theta_rad, theta_dot_radps);
        effector
    }

    pub fn panel_inertial_state(&self, hub_state: &SpacecraftStateMsg) -> SpacecraftStateMsg {
        let body_to_inertial = hub_state.body_to_inertial();
        let panel_to_body = self.kinematic.dcm_panel_body.transpose();
        let panel_to_inertial = body_to_inertial * UnitQuaternion::from_matrix(&panel_to_body);
        let omega_sn_body =
            hub_state.omega_radps + self.theta_dot_radps * self.kinematic.s_hat2_body;
        let position_m = hub_state.position_m
            + body_to_inertial.transform_vector(&self.kinematic.panel_com_position_body_m);
        let velocity_body_mps = self.kinematic.panel_com_prime_body_mps
            + hub_state
                .omega_radps
                .cross(&self.kinematic.panel_com_position_body_m);

        SpacecraftStateMsg {
            position_m,
            velocity_mps: hub_state.velocity_mps
                + body_to_inertial.transform_vector(&velocity_body_mps),
            sigma_bn: quaternion_to_mrp(panel_to_inertial),
            omega_radps: self.kinematic.dcm_panel_body * omega_sn_body,
        }
    }

    fn update_kinematics(&mut self, theta_rad: f64, theta_dot_radps: f64) {
        self.theta_rad = theta_rad;
        self.theta_dot_radps = theta_dot_radps;
        self.kinematic =
            HingedRigidBodyKinematics::from_config(&self.config, theta_rad, theta_dot_radps);
    }

    fn update_commanded_values(&mut self) {
        if self.motor_torque_in.is_connected() {
            self.motor_torque_nm = self.motor_torque_in.read().first_torque_nm();
        }
        if self.hinged_rigid_body_ref_in.is_connected() {
            let reference = self.hinged_rigid_body_ref_in.read();
            self.theta_ref_rad = reference.theta_rad;
            self.theta_dot_ref_radps = reference.theta_dot_radps;
        }
    }
}

impl StateEffector for HingedRigidBodyStateEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn state_len(&self) -> usize {
        2
    }

    fn initial_state(&self) -> Vec<f64> {
        vec![self.config.theta_init_rad, self.config.theta_dot_init_radps]
    }

    fn load_state(&mut self, state: &[f64]) {
        assert_eq!(
            state.len(),
            self.state_len(),
            "hinged rigid body state length mismatch"
        );
        self.update_kinematics(state[0], state[1]);
        self.hinged_rigid_body_out.write(HingedRigidBodyMsg {
            theta_rad: self.theta_rad,
            theta_dot_radps: self.theta_dot_radps,
        });
    }

    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {
        self.update_commanded_values();
    }

    fn update_contributions(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        assert_eq!(
            effector_state.len(),
            self.state_len(),
            "hinged rigid body state length mismatch"
        );
        let kinematics = HingedRigidBodyKinematics::from_config(
            &self.config,
            effector_state[0],
            effector_state[1],
        );
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
            "hinged rigid body state length mismatch"
        );
        let back_sub = self.back_sub.borrow();
        let theta_ddot = back_sub.a_theta.dot(&body_trans_accel_mps2)
            + back_sub.b_theta.dot(&body_omega_dot_radps2)
            + back_sub.c_theta;
        vec![effector_state[1], theta_ddot]
    }

    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        assert_eq!(
            effector_state.len(),
            self.state_len(),
            "hinged rigid body state length mismatch"
        );
        let kinematics = HingedRigidBodyKinematics::from_config(
            &self.config,
            effector_state[0],
            effector_state[1],
        );
        let r_tilde = tilde(kinematics.panel_com_position_body_m);
        StateEffectorMassProps {
            mass_kg: self.config.mass_kg,
            center_of_mass_body_m: kinematics.panel_com_position_body_m,
            inertia_about_point_b_body_kg_m2: kinematics.dcm_panel_body.transpose()
                * self.config.inertia_about_panel_com_panel_kg_m2
                * kinematics.dcm_panel_body
                + self.config.mass_kg * r_tilde * r_tilde.transpose(),
        }
    }

    fn write_outputs(&mut self, _current_sim_nanos: u64, hub_state: &SpacecraftStateMsg) {
        self.hinged_rigid_body_out.write(HingedRigidBodyMsg {
            theta_rad: self.theta_rad,
            theta_dot_radps: self.theta_dot_radps,
        });
        self.panel_state_out
            .write(self.panel_inertial_state(hub_state));
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl HingedRigidBodyStateEffector {
    fn back_sub_terms(
        &self,
        kinematics: &HingedRigidBodyKinematics,
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
    ) -> HingedRigidBodyBackSubTerms {
        let mass = self.config.mass_kg;
        let d = self.config.center_of_mass_offset_m;
        let inertia = self.config.inertia_about_panel_com_panel_kg_m2;
        let panel_iyy = inertia[(1, 1)];
        let panel_ixx = inertia[(0, 0)];
        let panel_izz = inertia[(2, 2)];
        let denom = panel_iyy + mass * d * d;
        assert!(
            denom > 0.0,
            "hinged rigid body '{}' requires positive Iyy + m*d^2",
            self.config.name
        );

        let omega_panel = kinematics.dcm_panel_body * body_omega_radps;
        let omega_tilde = tilde(body_omega_radps);
        let hinge_tilde = tilde(kinematics.hinge_position_body_m);
        let panel_com_tilde = tilde(kinematics.panel_com_position_body_m);
        let a_theta = -mass * d / denom * kinematics.s_hat3_body;
        let b_theta = -((denom * kinematics.s_hat2_body)
            + mass * d * hinge_tilde * kinematics.s_hat3_body)
            / denom;
        let gravity_torque_about_hinge =
            -d * kinematics.s_hat1_body.cross(&(mass * gravity_body_mps2));
        let c_theta = (self.motor_torque_nm
            - self.config.spring_constant_nm_per_rad * (kinematics.theta_rad - self.theta_ref_rad)
            - self.config.damping_nm_s_per_rad
                * (kinematics.theta_dot_radps - self.theta_dot_ref_radps)
            + kinematics.s_hat2_body.dot(&gravity_torque_about_hinge)
            + (panel_izz - panel_ixx + mass * d * d) * omega_panel.z * omega_panel.x
            - mass
                * d
                * kinematics
                    .s_hat3_body
                    .dot(&(omega_tilde * omega_tilde * kinematics.hinge_position_body_m)))
            / denom;

        let theta_dot = kinematics.theta_dot_radps;
        let hinge_axis_inertia = panel_iyy * kinematics.s_hat2_body
            + mass * d * panel_com_tilde * kinematics.s_hat3_body;
        let vec_trans = -(mass * d * theta_dot * theta_dot * kinematics.s_hat1_body
            + mass * d * c_theta * kinematics.s_hat3_body);
        let identity = Matrix3::identity();
        let vec_rot = -((theta_dot * omega_tilde + c_theta * identity) * hinge_axis_inertia
            + mass * d * theta_dot * theta_dot * panel_com_tilde * kinematics.s_hat1_body);

        HingedRigidBodyBackSubTerms {
            matrix_a: mass * d * kinematics.s_hat3_body * a_theta.transpose(),
            matrix_b: mass * d * kinematics.s_hat3_body * b_theta.transpose(),
            matrix_c: hinge_axis_inertia * a_theta.transpose(),
            matrix_d: hinge_axis_inertia * b_theta.transpose(),
            vec_trans,
            vec_rot,
            a_theta,
            b_theta,
            c_theta,
        }
    }
}

#[derive(Clone, Debug, Default)]
struct HingedRigidBodyBackSubTerms {
    matrix_a: Matrix3<f64>,
    matrix_b: Matrix3<f64>,
    matrix_c: Matrix3<f64>,
    matrix_d: Matrix3<f64>,
    vec_trans: Vector3<f64>,
    vec_rot: Vector3<f64>,
    a_theta: Vector3<f64>,
    b_theta: Vector3<f64>,
    c_theta: f64,
}

#[derive(Clone, Debug, Default)]
struct HingedRigidBodyKinematics {
    theta_rad: f64,
    theta_dot_radps: f64,
    hinge_position_body_m: Vector3<f64>,
    panel_com_position_body_m: Vector3<f64>,
    panel_com_prime_body_mps: Vector3<f64>,
    s_hat1_body: Vector3<f64>,
    s_hat2_body: Vector3<f64>,
    s_hat3_body: Vector3<f64>,
    dcm_panel_body: Matrix3<f64>,
}

impl HingedRigidBodyKinematics {
    fn from_config(config: &HingedRigidBodyConfig, theta_rad: f64, theta_dot_radps: f64) -> Self {
        let dcm_panel_hinge = Matrix3::new(
            theta_rad.cos(),
            0.0,
            theta_rad.sin(),
            0.0,
            1.0,
            0.0,
            -theta_rad.sin(),
            0.0,
            theta_rad.cos(),
        );
        let dcm_panel_body = dcm_panel_hinge * config.body_to_hinge_dcm;
        let s_hat1_body = dcm_panel_body.row(0).transpose();
        let s_hat2_body = dcm_panel_body.row(1).transpose();
        let s_hat3_body = dcm_panel_body.row(2).transpose();
        let hinge_position_body_m = config.hinge_position_body_m;
        let d = config.center_of_mass_offset_m;
        let panel_com_position_body_m = hinge_position_body_m - d * s_hat1_body;
        let panel_com_prime_body_mps = d * theta_dot_radps * s_hat3_body;

        Self {
            theta_rad,
            theta_dot_radps,
            hinge_position_body_m,
            panel_com_position_body_m,
            panel_com_prime_body_mps,
            s_hat1_body,
            s_hat2_body,
            s_hat3_body,
            dcm_panel_body,
        }
    }
}

fn tilde(vector: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -vector.z, vector.y, vector.z, 0.0, -vector.x, -vector.y, vector.x, 0.0,
    )
}

fn quaternion_to_mrp(quaternion: UnitQuaternion<f64>) -> Vector3<f64> {
    let q = quaternion.quaternion();
    let denom = 1.0 + q.w;
    if denom.abs() > 1.0e-12 {
        Vector3::new(q.i, q.j, q.k) / denom
    } else {
        -Vector3::new(q.i, q.j, q.k) / (1.0 - q.w)
    }
}

#[cfg(test)]
mod tests {
    use super::{HingedBodyLinearProfiler, HingedRigidBodyConfig, HingedRigidBodyStateEffector};
    use crate::Module;
    use crate::messages::{ArrayMotorTorqueMsg, HingedRigidBodyMsg, Output};
    use crate::spacecraft::{BackSubMatrices, StateEffector};
    use crate::{
        simulation::Simulation,
        spacecraft::{Spacecraft, SpacecraftConfig},
    };
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    #[test]
    fn linear_profiler_matches_basilisk_reference_points() {
        let mut profiler = HingedBodyLinearProfiler::with_profile(
            "profiler",
            1_000_000_000,
            2_000_000_000,
            0.0,
            std::f64::consts::PI / 180.0,
        );
        profiler.init();

        let cases = [
            (0, 0.0, 0.0),
            (1_000_000_000, 0.0, std::f64::consts::PI / 180.0),
            (
                1_500_000_000,
                std::f64::consts::PI / 360.0,
                std::f64::consts::PI / 180.0,
            ),
            (
                2_000_000_000,
                std::f64::consts::PI / 180.0,
                std::f64::consts::PI / 180.0,
            ),
            (2_500_000_000, std::f64::consts::PI / 180.0, 0.0),
        ];

        for (time_nanos, expected_theta, expected_theta_dot) in cases {
            let reference = profiler.reference_at(time_nanos);
            assert!((reference.theta_rad - expected_theta).abs() < 1.0e-12);
            assert!((reference.theta_dot_radps - expected_theta_dot).abs() < 1.0e-12);
        }
    }

    #[test]
    fn hinged_body_reads_motor_and_reference_messages() {
        let mut config = HingedRigidBodyConfig::new("panel");
        config.mass_kg = 10.0;
        config.inertia_about_panel_com_panel_kg_m2 =
            Matrix3::new(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0);
        let mut panel = HingedRigidBodyStateEffector::new(config);
        let motor = Output::new(ArrayMotorTorqueMsg {
            motor_torque_nm: vec![0.25],
        });
        let reference = Output::new(HingedRigidBodyMsg {
            theta_rad: 0.1,
            theta_dot_radps: 0.2,
        });
        panel.motor_torque_in.connect(motor.slot());
        panel.hinged_rigid_body_ref_in.connect(reference.slot());

        panel.pre_integration(0, 0.1);

        assert!((panel.motor_torque_nm - 0.25).abs() < 1.0e-12);
        assert!((panel.theta_ref_rad - 0.1).abs() < 1.0e-12);
        assert!((panel.theta_dot_ref_radps - 0.2).abs() < 1.0e-12);
    }

    #[test]
    fn hinged_body_back_substitution_has_expected_stationary_terms() {
        let mut config = HingedRigidBodyConfig::new("panel");
        config.mass_kg = 100.0;
        config.center_of_mass_offset_m = 1.5;
        config.spring_constant_nm_per_rad = 0.0;
        config.damping_nm_s_per_rad = 0.0;
        config.inertia_about_panel_com_panel_kg_m2 =
            Matrix3::new(100.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 50.0);
        config.hinge_position_body_m = Vector3::new(0.5, 0.0, 1.0);
        config.body_to_hinge_dcm = Matrix3::new(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
        let panel = HingedRigidBodyStateEffector::new(config);
        let mut back_sub = BackSubMatrices::default();

        panel.update_contributions(
            &[0.0, 0.0],
            Vector3::zeros(),
            Vector3::zeros(),
            &mut back_sub,
        );

        assert!(back_sub.vec_trans.norm() < 1.0e-12);
        assert!(back_sub.vec_rot.norm() < 1.0e-12);
        assert!(back_sub.matrix_a.norm() > 0.0);
        assert!(back_sub.matrix_d.norm() > 0.0);
    }

    #[test]
    fn hinged_body_mass_properties_use_supplied_state() {
        let mut config = HingedRigidBodyConfig::new("panel");
        config.mass_kg = 4.0;
        config.center_of_mass_offset_m = 2.0;
        config.inertia_about_panel_com_panel_kg_m2 = Matrix3::identity();
        let panel = HingedRigidBodyStateEffector::new(config);

        let mass_props = panel.mass_properties(&[std::f64::consts::FRAC_PI_2, 0.0]);

        assert_vector_close(
            mass_props.center_of_mass_body_m,
            Vector3::new(0.0, 0.0, -2.0),
            1.0e-12,
        );
        assert!(
            (mass_props.inertia_about_point_b_body_kg_m2
                - Matrix3::new(17.0, 0.0, 0.0, 0.0, 17.0, 0.0, 0.0, 0.0, 1.0))
            .norm()
                < 1.0e-12
        );
    }

    #[test]
    fn hinged_rigid_body_motor_torque_initial_config_matches_basilisk_geometry() {
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 750.0,
            hub_center_of_mass_body_m: Vector3::new(0.0, 0.0, 1.0),
            inertia_kg_m2: Matrix3::new(900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0),
            integration_step_nanos: 10_000_000,
            initial_position_m: Vector3::zeros(),
            initial_velocity_mps: Vector3::zeros(),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
        });

        let panel1 = HingedRigidBodyStateEffector::new(basilisk_motor_torque_panel_1());
        let panel1_state = panel1.panel_state_out.clone();
        let panel2 = HingedRigidBodyStateEffector::new(basilisk_motor_torque_panel_2());
        let panel2_state = panel2.panel_state_out.clone();

        spacecraft.add_state_effector(panel1);
        spacecraft.add_state_effector(panel2);

        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("spacecraft", &mut spacecraft, 10_000_000, 0);
            sim.run_for(0);
        }

        let panel1_state = panel1_state.read();
        let panel2_state = panel2_state.read();
        assert_vector_close(
            panel1_state.position_m,
            Vector3::new(2.0, 0.0, 1.0),
            1.0e-12,
        );
        assert_vector_close(panel1_state.velocity_mps, Vector3::zeros(), 1.0e-12);
        assert_vector_close(panel1_state.sigma_bn, Vector3::new(0.0, 0.0, 1.0), 1.0e-12);
        assert_vector_close(panel1_state.omega_radps, Vector3::zeros(), 1.0e-12);
        assert_vector_close(
            panel2_state.position_m,
            Vector3::new(-2.0, 0.0, 1.0),
            1.0e-12,
        );
        assert_vector_close(panel2_state.velocity_mps, Vector3::zeros(), 1.0e-12);
        assert_vector_close(panel2_state.sigma_bn, Vector3::zeros(), 1.0e-12);
        assert_vector_close(panel2_state.omega_radps, Vector3::zeros(), 1.0e-12);

        let mass_props = spacecraft.mass_props_out.read();
        assert!((mass_props.mass_kg - 950.0).abs() < 1.0e-12);
        assert_vector_close(
            mass_props.center_of_mass_body_m,
            Vector3::new(0.0, 0.0, 1.0),
            1.0e-12,
        );
        assert_matrix_close(
            mass_props.inertia_about_point_b_body_kg_m2,
            Matrix3::new(2050.0, 0.0, 0.0, 0.0, 2650.0, 0.0, 0.0, 0.0, 1500.0),
            1.0e-12,
        );
        assert_vector_close(
            panel1_state.position_m - mass_props.center_of_mass_body_m,
            Vector3::new(2.0, 0.0, 0.0),
            1.0e-12,
        );
        assert_vector_close(
            panel2_state.position_m - mass_props.center_of_mass_body_m,
            Vector3::new(-2.0, 0.0, 0.0),
            1.0e-12,
        );
    }

    #[test]
    fn hinged_rigid_body_motor_torque_integrated_state_matches_basilisk_test_shape() {
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 750.0,
            hub_center_of_mass_body_m: Vector3::new(0.0, 0.0, 1.0),
            inertia_kg_m2: Matrix3::new(900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0),
            integration_step_nanos: 10_000_000,
            initial_position_m: Vector3::zeros(),
            initial_velocity_mps: Vector3::zeros(),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
        });
        let mut panel1 = HingedRigidBodyStateEffector::new(basilisk_motor_torque_panel_1());
        let panel1_hinge = panel1.hinged_rigid_body_out.clone();
        let panel2 = HingedRigidBodyStateEffector::new(basilisk_motor_torque_panel_2());
        let panel2_hinge = panel2.hinged_rigid_body_out.clone();
        let motor = Output::new(ArrayMotorTorqueMsg {
            motor_torque_nm: vec![2.0],
        });

        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.connect(&motor, &mut panel1.motor_torque_in);
            spacecraft.add_state_effector(panel1);
            spacecraft.add_state_effector(panel2);
            sim.add_module("spacecraft", &mut spacecraft, 10_000_000, 0);
            sim.run_for(10_000_000_000);
        }

        let final_mass_props = spacecraft.mass_props_out.read();
        assert!((final_mass_props.mass_kg - 950.0).abs() < 1.0e-10);
        assert!(
            final_mass_props
                .center_of_mass_body_m
                .iter()
                .all(|value| value.is_finite())
        );
        assert!(
            final_mass_props
                .inertia_about_point_b_body_kg_m2
                .iter()
                .all(|value| value.is_finite())
        );
        assert!(panel1_hinge.read().theta_rad.is_finite());
        assert!(panel2_hinge.read().theta_rad.is_finite());
    }

    fn basilisk_motor_torque_panel_1() -> HingedRigidBodyConfig {
        let mut config = HingedRigidBodyConfig::new("panel1");
        config.mass_kg = 100.0;
        config.inertia_about_panel_com_panel_kg_m2 =
            Matrix3::new(100.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 50.0);
        config.center_of_mass_offset_m = 1.5;
        config.spring_constant_nm_per_rad = 0.0;
        config.damping_nm_s_per_rad = 0.0;
        config.hinge_position_body_m = Vector3::new(0.5, 0.0, 1.0);
        config.body_to_hinge_dcm = Matrix3::new(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
        config
    }

    fn basilisk_motor_torque_panel_2() -> HingedRigidBodyConfig {
        let mut config = HingedRigidBodyConfig::new("panel2");
        config.mass_kg = 100.0;
        config.inertia_about_panel_com_panel_kg_m2 =
            Matrix3::new(100.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 50.0);
        config.center_of_mass_offset_m = 1.5;
        config.spring_constant_nm_per_rad = 0.0;
        config.damping_nm_s_per_rad = 0.0;
        config.hinge_position_body_m = Vector3::new(-0.5, 0.0, 1.0);
        config.body_to_hinge_dcm = Matrix3::identity();
        config
    }

    fn assert_vector_close(actual: Vector3<f64>, expected: Vector3<f64>, tolerance: f64) {
        assert!(
            (actual - expected).norm() < tolerance,
            "expected vector {expected:?}, got {actual:?}"
        );
    }

    fn assert_matrix_close(actual: Matrix3<f64>, expected: Matrix3<f64>, tolerance: f64) {
        assert!(
            (actual - expected).norm() < tolerance,
            "expected matrix {expected:?}, got {actual:?}"
        );
    }
}
