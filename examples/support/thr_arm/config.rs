//! Upstream `examples/mujoco/thrArmControlSupport/regulationConfig.py` and
//! `trackingConfig.py`: the spacecraft, actuator and controller settings of
//! the two thruster-arm scenarios, as one [`ThrArmConfig`] with a constructor
//! per scenario.
//!
//! Upstream's optional-attribute conventions map as follows: an unset
//! (`None`) number is an `Option`; `jointProfileType` with its cutoff, filter
//! step and duration is one [`JointProfile`]; `controlWeight` (scalar or six
//! weights) is the diagonal of the allocator's `Wc`; `kTheta`, `pTheta`,
//! `kTrans` and `pTrans` (flattened row-major upstream) are matrices; and the
//! allocator geometry is the [`ThrusterArmConfigMsg`] upstream's
//! `SetThrArmConfig` writes (its flat vectors split into one entry per joint,
//! body or thruster).

use std::f64::consts::PI;
use std::path::PathBuf;

use basilisk_rs::device_interface::joint_array_ref_profiler::JointProfile;
use basilisk_rs::messages::ThrusterArmConfigMsg;
use nalgebra::{DMatrix, Matrix3, Vector3};

/// Upstream `controlObjective`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlObjective {
    /// Hold a fixed position and inertial attitude (`"regulation"`).
    Regulation,
    /// Follow the arc-line-hold path while pointing at the target
    /// (`"tracking"`).
    Tracking,
}

/// Upstream `RegulationConfig` / `TrackingConfig`.
#[derive(Clone, Debug)]
pub struct ThrArmConfig {
    pub control_objective: ControlObjective,
    /// Upstream `xmlPath`: the MuJoCo model, under `examples/data/mujoco/`.
    pub xml_path: PathBuf,
    /// Upstream `integratorRelativeTolerance` / `integratorAbsoluteTolerance`:
    /// `None` keeps `svIntegratorRKF45`'s defaults.
    pub integrator_relative_tolerance: Option<f64>,
    pub integrator_absolute_tolerance: Option<f64>,

    pub num_joints: usize,
    pub num_thrusters: usize,
    /// Upstream `hubMass` \[kg\].
    pub hub_mass_kg: f64,
    /// Upstream `vehicleMass` (regulation) or `scMass` (tracking) \[kg\]: the
    /// vehicle mass of the FSW vehicle configuration.
    pub vehicle_mass_kg: f64,
    /// Upstream `hubHalfLengths` \[m\].
    pub hub_half_lengths_m: [f64; 3],

    /// Upstream `fswTimeStep`, `dynTimeStep`, `outerLoopPeriod` \[s\].
    pub fsw_time_step_s: f64,
    pub dyn_time_step_s: f64,
    pub outer_loop_period_s: f64,

    /// Regulation references: upstream `transRefPosition`, `transRefVelocity`,
    /// `transRefAcceleration` and `attRefSigma_R0N`.
    pub trans_ref_position_m: Vector3<f64>,
    pub trans_ref_velocity_mps: Vector3<f64>,
    pub trans_ref_acceleration_mps2: Vector3<f64>,
    pub att_ref_sigma_r0n: Vector3<f64>,

    /// Tracking trajectory: upstream `transRadius`, `finalPosition`,
    /// `blendEndPosition`, `tCircle`, `tTransition`, `tLinear`.
    pub trans_radius_m: f64,
    pub final_position_m: Vector3<f64>,
    pub blend_end_position_m: Vector3<f64>,
    pub t_circle_s: f64,
    pub t_transition_s: f64,
    pub t_linear_s: f64,
    /// Tracking pointing: upstream `pHat_B`, `scTargetPosition`,
    /// `pointingSmallAngle` \[rad\] and `useBoresightRateDamping`.
    pub p_hat_b: Vector3<f64>,
    pub sc_target_position_m: Vector3<f64>,
    pub pointing_small_angle_rad: Option<f64>,
    pub use_boresight_rate_damping: bool,

    /// Event tolerances: upstream `jointPositionTolerance` \[rad\],
    /// `jointRateTolerance` \[rad/s\], `thrusterZeroTolerance` \[N\].
    pub joint_position_tolerance_rad: f64,
    pub joint_rate_tolerance_radps: f64,
    pub thruster_zero_tolerance_n: f64,

    /// Upstream `pulseResolution` and `minimumOnTime` \[s\].
    pub pulse_resolution_s: f64,
    pub minimum_on_time_s: Option<f64>,
    /// Upstream `thrForces` \[N\], `maxJointTorques` \[N m\] and
    /// `maxPreTorques` \[N m\].
    pub thr_forces_n: Vec<f64>,
    pub max_joint_torques_nm: Vec<f64>,
    pub max_pre_torques_nm: Option<Vec<f64>>,
    /// Upstream `controlWeight` (the diagonal of `Wc`) and `thrustWeight`.
    pub control_weight: [f64; 6],
    pub thrust_weight: f64,
    /// Upstream `thetaWeight` and `useThetaAllocPenalty`.
    pub theta_weight: Option<f64>,
    pub use_theta_alloc_penalty: bool,
    /// Upstream `allocationMaxIterations` and `allocationFunctionTolerance`.
    pub allocation_max_iterations: Option<usize>,
    pub allocation_function_tolerance: Option<f64>,
    pub use_joint_motion_compensation: bool,
    /// Upstream `useJointProfiler`, and `jointProfileType` with its
    /// parameters.
    pub use_joint_profiler: bool,
    pub joint_profile: Option<JointProfile>,
    pub joint_profile_use_shortest_path: bool,
    pub joint_profile_wrap_start_rad: Option<f64>,

    /// MuJoCo names: upstream `hubName`, `jointRefs` (body, joint),
    /// `jointActNames`, `thrActNames`, `bodyActNames`.
    pub hub_name: String,
    pub joint_refs: Vec<(String, String)>,
    pub joint_act_names: Vec<String>,
    pub thr_act_names: Vec<String>,
    pub body_act_names: Vec<String>,

    /// Upstream `kRot` \[N m\], `pRot` \[N m s\], `kiRot`.
    pub k_rot: f64,
    pub p_rot: f64,
    pub ki_rot: f64,
    /// Upstream `kTrans` \[N/m\] and `pTrans` \[N s/m\].
    pub k_trans: Matrix3<f64>,
    pub p_trans: Matrix3<f64>,
    /// Upstream `kTheta` and `pTheta`.
    pub k_theta: DMatrix<f64>,
    pub p_theta: DMatrix<f64>,

    /// The allocator geometry (upstream `r_BcB_B` ... `dcm_C0P`), with
    /// [`hub_mass_kg`](Self::hub_mass_kg) as its hub mass.
    pub arm_config: ThrusterArmConfigMsg,
}

fn names(prefix: &str, suffixes: &[&str]) -> Vec<String> {
    suffixes
        .iter()
        .map(|suffix| format!("{prefix}{suffix}"))
        .collect()
}

fn data_path(file: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("examples/data/mujoco")
        .join(file)
}

/// The arm geometry both configurations share (upstream's `r_BcB_B` through
/// `dcm_C0P`): two four-joint arms, a boom and a tip body each, one
/// thruster per tip.
fn arm_config(hub_mass_kg: f64) -> ThrusterArmConfigMsg {
    let x = |value: f64| Vector3::new(value, 0.0, 0.0);
    let (ex, ey, ez) = (Vector3::x(), Vector3::y(), Vector3::z());
    let arm2_base = Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0));
    let identity = Matrix3::identity();
    ThrusterArmConfigMsg {
        hub_mass_kg,
        r_bcb_b_m: Vector3::zeros(),
        body_arm_idx: vec![0, 0, 1, 1],
        body_joint_idx: vec![1, 3, 1, 3],
        body_mass_kg: vec![8.0, 2.0, 8.0, 2.0],
        r_lcp_p_m: vec![x(0.5), x(0.05), x(0.5), x(0.05)],
        arm_joint_count: vec![4, 4],
        r_cp_p_m: vec![
            x(0.79),
            x(0.0),
            x(1.0),
            x(0.0),
            x(-0.79),
            x(0.0),
            x(1.0),
            x(0.0),
        ],
        s_hat_p: vec![ex, ey, ez, ey, ex, ey, ez, ey],
        // Upstream flattens these column-major; both are symmetric.
        dcm_c0p: vec![
            identity, identity, identity, identity, arm2_base, identity, identity, identity,
        ],
        arm_tree_idx: vec![0, 0],
        thr_arm_idx: vec![0, 1],
        thr_arm_joint_idx: vec![3, 3],
        r_tp_p_m: vec![x(0.1), x(0.1)],
        f_hat_p: vec![x(-1.0), x(-1.0)],
    }
}

impl ThrArmConfig {
    /// The settings both scenarios share.
    fn common(
        control_objective: ControlObjective,
        xml: &str,
        hub_mass_kg: f64,
        vehicle_mass_kg: f64,
    ) -> Self {
        let num_joints = 8;
        let num_thrusters = 2;
        let arm_joints = ["joint_1", "joint_2", "joint_3", "joint_4"];
        let joint_refs = ["arm_1", "arm_2"]
            .iter()
            .flat_map(|arm| {
                arm_joints.iter().enumerate().map(move |(index, joint)| {
                    let body = if index < 2 { "base" } else { "tip" };
                    (format!("{arm}_{body}"), format!("{arm}_{joint}"))
                })
            })
            .collect();
        let mut joint_act_names = names("u_arm_1_", &arm_joints);
        joint_act_names.extend(names("u_arm_2_", &arm_joints));
        Self {
            control_objective,
            xml_path: data_path(xml),
            integrator_relative_tolerance: None,
            integrator_absolute_tolerance: None,
            num_joints,
            num_thrusters,
            hub_mass_kg,
            vehicle_mass_kg,
            hub_half_lengths_m: [0.79, 0.69, 0.52],
            fsw_time_step_s: 0.01,
            dyn_time_step_s: 0.01,
            outer_loop_period_s: 10.0,
            trans_ref_position_m: Vector3::zeros(),
            trans_ref_velocity_mps: Vector3::zeros(),
            trans_ref_acceleration_mps2: Vector3::zeros(),
            att_ref_sigma_r0n: Vector3::zeros(),
            trans_radius_m: 0.0,
            final_position_m: Vector3::zeros(),
            blend_end_position_m: Vector3::zeros(),
            t_circle_s: 0.0,
            t_transition_s: 0.0,
            t_linear_s: 0.0,
            p_hat_b: Vector3::zeros(),
            sc_target_position_m: Vector3::zeros(),
            pointing_small_angle_rad: None,
            use_boresight_rate_damping: false,
            joint_position_tolerance_rad: 1e-3,
            joint_rate_tolerance_radps: 1e-3,
            thruster_zero_tolerance_n: 1e-12,
            pulse_resolution_s: 0.01,
            minimum_on_time_s: Some(0.01),
            thr_forces_n: Vec::new(),
            max_joint_torques_nm: Vec::new(),
            max_pre_torques_nm: None,
            control_weight: [1.0; 6],
            thrust_weight: 0.0,
            theta_weight: None,
            use_theta_alloc_penalty: false,
            allocation_max_iterations: None,
            allocation_function_tolerance: None,
            use_joint_motion_compensation: true,
            use_joint_profiler: false,
            joint_profile: None,
            joint_profile_use_shortest_path: true,
            joint_profile_wrap_start_rad: Some(-PI),
            hub_name: "hub".to_string(),
            joint_refs,
            joint_act_names,
            thr_act_names: names("F_thruster_", &["1", "2"]),
            body_act_names: names("tau_prescribed_", &["1", "2", "3"]),
            k_rot: 0.0,
            p_rot: 0.0,
            ki_rot: -1.0,
            k_trans: Matrix3::zeros(),
            p_trans: Matrix3::zeros(),
            k_theta: DMatrix::zeros(num_joints, num_joints),
            p_theta: DMatrix::zeros(num_joints, num_joints),
            arm_config: arm_config(hub_mass_kg),
        }
    }

    /// Upstream `RegulationConfig()`.
    pub fn regulation() -> Self {
        let mut config = Self::common(
            ControlObjective::Regulation,
            "sat_w_thruster_arms_regulation.xml",
            330.0,
            350.0,
        );
        config.thr_forces_n = vec![2.5; config.num_thrusters];
        config.max_joint_torques_nm = vec![5.0; config.num_joints];
        config.control_weight = [1.0; 6];
        config.thrust_weight = 1e-6;
        let [half_x, half_y, _] = config.hub_half_lengths_m;
        let hub_inertia_z = config.hub_mass_kg * (half_x.powi(2) + half_y.powi(2)) / 3.0;
        let translation_time_constant = 25.0; // [s]
        let attitude_time_constant = 15.0; // [s]
        let p_trans = 2.0 * config.vehicle_mass_kg / translation_time_constant;
        config.p_trans = Matrix3::identity() * p_trans;
        config.k_trans = Matrix3::identity() * (p_trans.powi(2) / (4.0 * config.vehicle_mass_kg));
        config.p_rot = 2.0 * hub_inertia_z / attitude_time_constant;
        config.k_rot = config.p_rot.powi(2) / (4.0 * hub_inertia_z);
        config.ki_rot = -1.0;
        let n = config.num_joints;
        config.k_theta = DMatrix::identity(n, n) * PI.powi(2);
        config.p_theta = DMatrix::identity(n, n) * (2.0 * 1.5 * PI);
        config
    }

    /// Upstream `TrackingConfig()`.
    pub fn tracking() -> Self {
        let mut config = Self::common(
            ControlObjective::Tracking,
            "sat_w_thruster_arms_tracking.xml",
            980.0,
            1000.0,
        );
        config.trans_radius_m = 20.0;
        config.final_position_m = Vector3::new(0.0, -5.0, 0.0);
        config.blend_end_position_m = Vector3::new(0.0, -16.0, 0.0);
        config.t_circle_s = 5.0 * 3600.0;
        config.t_transition_s = 1.0 * 3600.0;
        config.t_linear_s = 110.0 * 60.0;
        config.p_hat_b = Vector3::x();
        config.sc_target_position_m = Vector3::zeros();
        config.pointing_small_angle_rad = None;
        config.use_boresight_rate_damping = true;
        config.thr_forces_n = vec![0.25; config.num_thrusters];
        config.max_joint_torques_nm = vec![2.5; config.num_joints];
        config.max_pre_torques_nm = None;
        config.control_weight = [1.0; 6];
        config.thrust_weight = 1e-4;
        config.k_rot = 1.75;
        config.p_rot = 15.0;
        config.ki_rot = -1.0;
        config.k_trans = Matrix3::identity() * (0.00002 * config.vehicle_mass_kg);
        config.p_trans = Matrix3::identity() * (0.05 * config.vehicle_mass_kg);
        let n = config.num_joints;
        config.k_theta = DMatrix::identity(n, n) * 22.0;
        config.p_theta = DMatrix::identity(n, n) * 14.0;
        config
    }

    /// The vehicle inertia upstream's `SetVehicleConfiguration` gives the
    /// attitude controller: the hub box about its centre (the instantaneous
    /// system inertia is not available to the FSW).
    pub fn hub_box_inertia_kg_m2(&self) -> Matrix3<f64> {
        let [x, y, z] = self.hub_half_lengths_m;
        let m = self.hub_mass_kg / 3.0;
        Matrix3::from_diagonal(&Vector3::new(
            m * (y * y + z * z),
            m * (x * x + z * z),
            m * (x * x + y * y),
        ))
    }
}
