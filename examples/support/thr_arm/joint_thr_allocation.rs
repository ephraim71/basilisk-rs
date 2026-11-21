//! Thruster force and joint angle allocation for thrusters on arms: port of
//! upstream's pure-Python
//! `fswAlgorithms/effectorInterfaces/jointThrAllocation/jointThrAllocation.py`.
//!
//! Each update turns the commanded inertial force `F_N` (rotated into the
//! hub frame, `F_B = [BN] F_N`) and body torque `L_B` into the arm joint
//! angles `θ` and thruster forces `f` that minimize
//!
//! ```text
//! J(θ, f) = (w - [A(θ)] f)ᵀ [W_c] (w - [A(θ)] f) + W_fᵀ f
//!           [+ wrap(θ - θ_now)ᵀ [W_θ] wrap(θ - θ_now)]
//! ```
//!
//! for the wanted wrench `w = (F_B, L_B)`, subject to `-π ≤ θ ≤ π` and
//! `0 ≤ f ≤ f_max`. The map `[A(θ)]` stacks each thruster's force direction
//! and its torque about the system centre of mass, both from the arm
//! kinematics of the [`ThrusterArmConfigMsg`] at the angles `θ`. The
//! optional third term penalizes joint motion away from the measured angles.
//! The problem is solved from five seeds (all angles 0, ±π/4, ±π/2, every
//! thrust 1 N) and the successful solution of least cost is published; when
//! none succeeds, zero forces and zero (or, with the motion penalty, the
//! measured) angles are.
//!
//! As in upstream, this is an example-oriented module: one kinematic tree,
//! arms packed in arm order, one thruster per arm.
//!
//! Differences from upstream:
//!
//! - SciPy's SLSQP is replaced by [`bounded_sqp::minimize`](super::bounded_sqp::minimize);
//!   see there.
//! - The desired joint states carry no write-time header. The stand-in is a
//!   time stamp written by the producer:
//!   [`des_joint_angles_time_out_msg`](JointThrAllocation::des_joint_angles_time_out_msg)
//!   receives the simulation time of every write of the desired states (0
//!   at reset, as upstream's `write(payload)` there).
//! - Upstream raises `BasiliskError` or `ValueError` on a bad configuration
//!   at `Reset`; here [`JointThrAllocation::validate`] reports it and `init`
//!   panics with it.

#![allow(dead_code)]

use std::f64::consts::PI;

use basilisk_rs::messages::{
    BodyTorqueCommandMsg, InertialForceCommandMsg, Input, JointArrayStateMsg, Output,
    ScalarJointStateMsg, SpacecraftStateMsg, ThrusterArmConfigMsg, ThrusterArrayForceCommandMsg,
};
use basilisk_rs::utilities::rigid_body_kinematics::{mrp_to_dcm, prv_to_dcm};
use basilisk_rs::{Module, SimulationContext};
use nalgebra::{DMatrix, DVector, Matrix3, Matrix6, Vector3, Vector6};
use serde::{Deserialize, Serialize};

use super::bounded_sqp::{MinimizeOptions, minimize};

/// The time a message was written \[ns\]: the stand-in for upstream's
/// `timeWritten()` header of the desired joint states, published next to
/// them by their producer.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct WriteTimeMsg {
    pub time_written_nanos: u64,
}

/// Upstream `wrapAngle`: `(θ + π) mod 2π - π`, in `[-π, π)`.
pub fn wrap_angle(angle_rad: f64) -> f64 {
    (angle_rad + PI).rem_euclid(2.0 * PI) - PI
}

/// Upstream `mapMatrix`: the 6 × n map from thruster force magnitudes to the
/// stacked force and torque about `r_com_b` of thrusters at `r_b` along
/// `f_hat_b`.
pub fn map_matrix(
    r_b: &[Vector3<f64>],
    f_hat_b: &[Vector3<f64>],
    r_com_b: Vector3<f64>,
) -> DMatrix<f64> {
    let mut map = DMatrix::zeros(6, r_b.len());
    for (column, (position, direction)) in r_b.iter().zip(f_hat_b).enumerate() {
        let torque = (position - r_com_b).cross(direction);
        for axis in 0..3 {
            map[(axis, column)] = direction[axis];
            map[(axis + 3, column)] = torque[axis];
        }
    }
    map
}

/// A weight or limit given for all thrusters at once or per thruster
/// (upstream's scalar-or-vector setters).
#[derive(Clone, Debug, PartialEq)]
pub enum PerThruster {
    All(f64),
    Each(Vec<f64>),
}

impl PerThruster {
    /// Upstream `resolveWf` / `resolveThrForceMax`.
    pub fn resolve(&self, count: usize, name: &str) -> Result<DVector<f64>, String> {
        match self {
            Self::All(value) => Ok(DVector::from_element(count, *value)),
            Self::Each(values) if values.len() == 1 => Ok(DVector::from_element(count, values[0])),
            Self::Each(values) if values.len() == count => Ok(DVector::from_column_slice(values)),
            Self::Each(values) => Err(format!(
                "{name} vector length {} does not match nThr {count}.",
                values.len()
            )),
        }
    }
}

/// Upstream `setWtheta`: a scalar, the diagonal, or the full joint-motion
/// weighting matrix.
#[derive(Clone, Debug, PartialEq)]
pub enum MotionWeight {
    Scalar(f64),
    Diagonal(Vec<f64>),
    Matrix(DMatrix<f64>),
}

impl MotionWeight {
    /// Upstream `resolveWtheta`: validates the weight and returns the
    /// `n` × `n` matrix. Matrix symmetry and eigenvalue checks use a
    /// tolerance of `10 n ε` relative to the largest entry; roundoff
    /// asymmetry is averaged away and roundoff-negative eigenvalues are
    /// projected to zero.
    pub fn resolve(&self, n: usize) -> Result<DMatrix<f64>, String> {
        fn finite(mut values: impl Iterator<Item = f64>) -> bool {
            values.all(f64::is_finite)
        }
        match self {
            Self::Scalar(value) => {
                if !value.is_finite() {
                    return Err("Wtheta weights must be finite.".into());
                }
                if *value < 0.0 {
                    return Err("Wtheta scalar weight must be nonnegative.".into());
                }
                Ok(DMatrix::identity(n, n) * *value)
            }
            Self::Diagonal(values) => {
                if !finite(values.iter().copied()) {
                    return Err("Wtheta weights must be finite.".into());
                }
                if values.len() != n {
                    return Err(
                        "setWtheta expects scalar, length-nJoint vector, or nJoint x nJoint matrix."
                            .into(),
                    );
                }
                if values.iter().any(|value| *value < 0.0) {
                    return Err("Wtheta vector weights must be nonnegative.".into());
                }
                Ok(DMatrix::from_diagonal(&DVector::from_column_slice(values)))
            }
            Self::Matrix(matrix) => {
                if !finite(matrix.iter().copied()) {
                    return Err("Wtheta weights must be finite.".into());
                }
                if matrix.nrows() != n || matrix.ncols() != n {
                    return Err(
                        "setWtheta expects scalar, length-nJoint vector, or nJoint x nJoint matrix."
                            .into(),
                    );
                }
                if matrix.diagonal().iter().any(|value| *value < 0.0) {
                    return Err(
                        "Wtheta matrix must be positive semidefinite: diagonal entries must be \
                         nonnegative."
                            .into(),
                    );
                }
                let mut resolved = matrix.clone();
                let scale = matrix
                    .iter()
                    .fold(0.0_f64, |max, value| max.max(value.abs()));
                if scale > 0.0 {
                    let scaled = matrix / scale;
                    let tolerance = 10.0 * n as f64 * f64::EPSILON;
                    if (&scaled - scaled.transpose())
                        .iter()
                        .any(|value| value.abs() > tolerance)
                    {
                        return Err("Wtheta matrix must be symmetric.".into());
                    }
                    for row in 0..n {
                        for column in 0..n {
                            let (a, b) = (matrix[(row, column)], matrix[(column, row)]);
                            if a != b {
                                resolved[(row, column)] = 0.5 * a + 0.5 * b;
                            }
                        }
                    }
                    let eigen = (&resolved / scale).symmetric_eigen();
                    if eigen.eigenvalues.iter().any(|value| *value < -tolerance) {
                        return Err("Wtheta matrix must be positive semidefinite.".into());
                    }
                    if eigen.eigenvalues.iter().any(|value| *value < 0.0) {
                        let clamped = eigen.eigenvalues.map(|value| value.max(0.0));
                        let vectors = &eigen.eigenvectors;
                        let mut projected =
                            vectors * DMatrix::from_diagonal(&clamped) * vectors.transpose();
                        projected = (&projected + projected.transpose()) * 0.5;
                        let peak = projected
                            .iter()
                            .fold(1.0_f64, |max, value| max.max(value.abs()));
                        resolved = projected / peak * scale;
                    }
                }
                Ok(resolved)
            }
        }
    }
}

/// User-settable parameters of [`JointThrAllocation`].
#[derive(Clone, Debug)]
pub struct JointThrAllocationConfig {
    /// Upstream `ModelTag`.
    pub name: String,
    /// Upstream `Wc` (`setWc`): the wrench weighting matrix; default the
    /// identity.
    pub wc: Matrix6<f64>,
    /// Upstream `Wf` (`setWf`): the thrust weight; default `1e-6` for every
    /// thruster.
    pub wf: PerThruster,
    /// Upstream `Wtheta` (`setWtheta`, which also sets `useThetaPenalty`):
    /// the optional joint-motion weight.
    pub w_theta: Option<MotionWeight>,
    /// Upstream `maxiter` and `ftol`.
    pub max_iterations: usize,
    pub ftol: f64,
    /// Upstream `thrForceMax` (`setThrForceMax`) \[N\]; default 2.5 N.
    pub thr_force_max_n: PerThruster,
}

impl Default for JointThrAllocationConfig {
    fn default() -> Self {
        Self {
            name: "jointThrAllocation".to_string(),
            wc: Matrix6::identity(),
            wf: PerThruster::All(1e-6),
            w_theta: None,
            max_iterations: 3000,
            ftol: 1e-10,
            thr_force_max_n: PerThruster::All(2.5),
        }
    }
}

impl JointThrAllocationConfig {
    /// Upstream `setWc` with a scalar weight.
    pub fn set_wc_scalar(&mut self, weight: f64) {
        self.wc = Matrix6::identity() * weight;
    }

    /// Upstream `setWc` with six weights.
    pub fn set_wc_diagonal(&mut self, weights: [f64; 6]) {
        self.wc = Matrix6::from_diagonal(&Vector6::from_column_slice(&weights));
    }
}

/// The arm geometry of a [`ThrusterArmConfigMsg`] (upstream
/// `parseArmConfig`), with the arm kinematics upstream's allocation methods
/// compute from it.
#[derive(Clone, Debug, Default)]
pub struct ArmGeometry {
    pub n_arms: usize,
    pub n_thr: usize,
    pub n_joint: usize,
    pub arm_joint_count: Vec<usize>,
    pub arm_joint_start: Vec<usize>,
    pub thr_arm_idx: Vec<usize>,
    pub thr_arm_joint_idx: Vec<usize>,
    pub r_cp_p: Vec<Vector3<f64>>,
    pub r_tp_p: Vec<Vector3<f64>>,
    pub s_hat_p: Vec<Vector3<f64>>,
    pub f_hat_p: Vec<Vector3<f64>>,
    pub dcm_c0p: Vec<Matrix3<f64>>,
    pub hub_mass: f64,
    pub r_bcb_b: Vector3<f64>,
    pub body_arm_idx: Vec<usize>,
    pub body_joint_idx: Vec<usize>,
    pub body_mass: Vec<f64>,
    pub r_lcp_p: Vec<Vector3<f64>>,
}

impl ArmGeometry {
    /// Upstream `parseArmConfig`.
    pub fn from_config(config: &ThrusterArmConfigMsg) -> Self {
        let mut arm_joint_start = Vec::with_capacity(config.arm_joint_count.len());
        let mut start = 0;
        for count in &config.arm_joint_count {
            arm_joint_start.push(start);
            start += count;
        }
        Self {
            n_arms: config.arm_joint_count.len(),
            n_thr: config.thr_arm_idx.len(),
            n_joint: start,
            arm_joint_count: config.arm_joint_count.clone(),
            arm_joint_start,
            thr_arm_idx: config.thr_arm_idx.clone(),
            thr_arm_joint_idx: config.thr_arm_joint_idx.clone(),
            r_cp_p: config.r_cp_p_m.clone(),
            r_tp_p: config.r_tp_p_m.clone(),
            s_hat_p: config.s_hat_p.clone(),
            f_hat_p: config.f_hat_p.clone(),
            dcm_c0p: config.dcm_c0p.clone(),
            hub_mass: config.hub_mass_kg,
            r_bcb_b: config.r_bcb_b_m,
            body_arm_idx: config.body_arm_idx.clone(),
            body_joint_idx: config.body_joint_idx.clone(),
            body_mass: config.body_mass_kg.clone(),
            r_lcp_p: config.r_lcp_p_m.clone(),
        }
    }

    /// Upstream `jointPoseFromTheta`: each joint frame's DCM `[CB]` and
    /// origin `r_CB_B` in the hub frame.
    ///
    /// # Panics
    ///
    /// On a joint axis of near-zero norm (upstream `ValueError`).
    pub fn joint_pose_from_theta(&self, theta: &[f64]) -> (Vec<Matrix3<f64>>, Vec<Vector3<f64>>) {
        let mut dcm_cb = vec![Matrix3::identity(); self.n_joint];
        let mut r_cb_b = vec![Vector3::zeros(); self.n_joint];
        for arm in 0..self.n_arms {
            let start = self.arm_joint_start[arm];
            for local in 0..self.arm_joint_count[arm] {
                let joint = start + local;
                let (dcm_pb, r_pb_b) = if local == 0 {
                    (Matrix3::identity(), Vector3::zeros())
                } else {
                    (dcm_cb[joint - 1], r_cb_b[joint - 1])
                };
                let axis = self.s_hat_p[joint];
                let norm = axis.norm();
                assert!(norm > 1e-12, "Joint axis has near-zero norm.");
                let dcm_cc0 = prv_to_dcm(axis / norm * theta[joint]);
                let dcm_cp = dcm_cc0 * self.dcm_c0p[joint];
                r_cb_b[joint] = r_pb_b + dcm_pb.transpose() * self.r_cp_p[joint];
                dcm_cb[joint] = dcm_cp * dcm_pb;
            }
        }
        (dcm_cb, r_cb_b)
    }

    /// Upstream `computeComFromTheta`: the system centre of mass relative to
    /// the hub frame origin.
    ///
    /// # Panics
    ///
    /// If the total mass is not positive (upstream `ValueError`).
    pub fn com_from_theta(&self, dcm_cb: &[Matrix3<f64>], r_cb_b: &[Vector3<f64>]) -> Vector3<f64> {
        let total = self.hub_mass + self.body_mass.iter().sum::<f64>();
        assert!(total > 0.0, "Total spacecraft mass must be positive.");
        let mut numerator = self.r_bcb_b * self.hub_mass;
        for body in 0..self.body_mass.len() {
            let joint = self.arm_joint_start[self.body_arm_idx[body]] + self.body_joint_idx[body];
            numerator += (r_cb_b[joint] + dcm_cb[joint].transpose() * self.r_lcp_p[body])
                * self.body_mass[body];
        }
        numerator / total
    }

    /// Upstream `mapping`: the force-to-wrench map at the angles `theta`.
    pub fn mapping(&self, theta: &[f64]) -> DMatrix<f64> {
        let (dcm_cb, r_cb_b) = self.joint_pose_from_theta(theta);
        let r_com_b = self.com_from_theta(&dcm_cb, &r_cb_b);
        let mut r_tb_b = Vec::with_capacity(self.n_thr);
        let mut f_hat_b = Vec::with_capacity(self.n_thr);
        for thruster in 0..self.n_thr {
            let joint =
                self.arm_joint_start[self.thr_arm_idx[thruster]] + self.thr_arm_joint_idx[thruster];
            let to_body = dcm_cb[joint].transpose();
            r_tb_b.push(r_cb_b[joint] + to_body * self.r_tp_p[thruster]);
            f_hat_b.push(to_body * self.f_hat_p[thruster]);
        }
        map_matrix(&r_tb_b, &f_hat_b, r_com_b)
    }
}

/// Thruster force and joint angle allocation: upstream `JointThrAllocation`.
#[derive(Debug)]
pub struct JointThrAllocation {
    pub config: JointThrAllocationConfig,
    /// Upstream `armConfigInMsg`.
    pub arm_config_in_msg: Input<ThrusterArmConfigMsg>,
    /// Upstream `hubStatesInMsg`: the hub attitude `σ_BN`.
    pub hub_states_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `transForceInMsg`.
    pub trans_force_in_msg: Input<InertialForceCommandMsg>,
    /// Upstream `rotTorqueInMsg`.
    pub rot_torque_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `jointStatesInMsgs`, made by
    /// [`add_hinged_joint`](Self::add_hinged_joint), read with the motion
    /// penalty.
    pub joint_states_in_msgs: Vec<Input<ScalarJointStateMsg>>,
    /// Upstream `thrForceOutMsg` \[N\].
    pub thr_force_out_msg: Output<ThrusterArrayForceCommandMsg>,
    /// Upstream `desJointAnglesOutMsg`: the angles, zero rates and zero
    /// accelerations.
    pub des_joint_angles_out_msg: Output<JointArrayStateMsg>,
    /// The write-time stand-in of `desJointAnglesOutMsg` (see the module
    /// docs).
    pub des_joint_angles_time_out_msg: Output<WriteTimeMsg>,
    /// Upstream `solutionFound`.
    pub solution_found: bool,
    /// Upstream `bestErrInf`: the largest wrench error of the solution.
    pub best_err_inf: f64,
    /// Upstream `wrenchError`.
    pub wrench_error: Vector6<f64>,
    /// Upstream `costVal`.
    pub cost_val: f64,
    geometry: ArmGeometry,
    wf: DVector<f64>,
    w_theta: Option<DMatrix<f64>>,
    lower: DVector<f64>,
    upper: DVector<f64>,
    seeds: Vec<DVector<f64>>,
}

impl JointThrAllocation {
    pub fn new(config: JointThrAllocationConfig) -> Self {
        Self {
            config,
            arm_config_in_msg: Input::default(),
            hub_states_in_msg: Input::default(),
            trans_force_in_msg: Input::default(),
            rot_torque_in_msg: Input::default(),
            joint_states_in_msgs: Vec::new(),
            thr_force_out_msg: Output::default(),
            des_joint_angles_out_msg: Output::default(),
            des_joint_angles_time_out_msg: Output::default(),
            solution_found: false,
            best_err_inf: f64::NAN,
            wrench_error: Vector6::repeat(f64::NAN),
            cost_val: f64::NAN,
            geometry: ArmGeometry::default(),
            wf: DVector::zeros(0),
            w_theta: None,
            lower: DVector::zeros(0),
            upper: DVector::zeros(0),
            seeds: Vec::new(),
        }
    }

    /// Upstream `addHingedJoint`: one more joint-state input for the motion
    /// penalty.
    pub fn add_hinged_joint(&mut self) -> &mut Input<ScalarJointStateMsg> {
        self.joint_states_in_msgs.push(Input::default());
        self.joint_states_in_msgs.last_mut().expect("just added")
    }

    /// Upstream `useThetaPenalty`.
    pub fn use_theta_penalty(&self) -> bool {
        self.config.w_theta.is_some()
    }

    /// The parsed arm geometry (upstream's `nJoint`, `nThr`, ... attributes).
    pub fn geometry(&self) -> &ArmGeometry {
        &self.geometry
    }

    /// Upstream `validateInputMessages` and the configuration checks of
    /// `Reset`.
    pub fn validate(&self) -> Result<(), String> {
        for (connected, name) in [
            (self.arm_config_in_msg.is_connected(), "armConfigInMsg"),
            (self.hub_states_in_msg.is_connected(), "hubStatesInMsg"),
            (self.trans_force_in_msg.is_connected(), "transForceInMsg"),
            (self.rot_torque_in_msg.is_connected(), "rotTorqueInMsg"),
        ] {
            if !connected {
                return Err(format!("JointThrAllocation.{name} was not linked."));
            }
        }
        Ok(())
    }

    /// Upstream `Reset` without the message checks: parses the geometry,
    /// resolves the weights, bounds and seeds.
    fn configure(&mut self) -> Result<(), String> {
        self.geometry = ArmGeometry::from_config(&self.arm_config_in_msg.read());
        let (n_joint, n_thr) = (self.geometry.n_joint, self.geometry.n_thr);
        self.wf = self.config.wf.resolve(n_thr, "Wf")?;
        self.w_theta = match &self.config.w_theta {
            Some(weight) => Some(weight.resolve(n_joint)?),
            None => None,
        };
        let thrust_max = self.config.thr_force_max_n.resolve(n_thr, "thrForceMax")?;
        // Upstream `bounds`.
        self.lower = DVector::from_fn(
            n_joint + n_thr,
            |index, _| if index < n_joint { -PI } else { 0.0 },
        );
        self.upper = DVector::from_fn(n_joint + n_thr, |index, _| {
            if index < n_joint {
                PI
            } else {
                thrust_max[index - n_joint]
            }
        });
        // Upstream `initialGuesses`.
        self.seeds = [0.0, PI / 4.0, -PI / 4.0, PI / 2.0, -PI / 2.0]
            .iter()
            .map(|angle| {
                DVector::from_fn(
                    n_joint + n_thr,
                    |index, _| if index < n_joint { *angle } else { 1.0 },
                )
            })
            .collect();
        Ok(())
    }

    /// Upstream `cost`.
    pub fn cost(
        &self,
        decision: &DVector<f64>,
        desired_wrench_b: &Vector6<f64>,
        current_angles: Option<&[f64]>,
    ) -> f64 {
        let n_joint = self.geometry.n_joint;
        let theta = &decision.as_slice()[..n_joint];
        let forces = decision.rows(n_joint, self.geometry.n_thr);
        let map = self.geometry.mapping(theta);
        let error = DVector::from_column_slice(desired_wrench_b.as_slice()) - map * forces;
        let wc = DMatrix::from_column_slice(6, 6, self.config.wc.as_slice());
        let mut cost = (error.transpose() * wc * &error)[(0, 0)] + self.wf.dot(&forces);
        if let Some(w_theta) = &self.w_theta {
            let current =
                current_angles.expect("currentJointAngles is required when using Wtheta.");
            let delta = DVector::from_iterator(
                n_joint,
                theta
                    .iter()
                    .zip(current)
                    .map(|(angle, now)| wrap_angle(angle - now)),
            );
            let mut motion = (delta.transpose() * w_theta * &delta)[(0, 0)];
            if motion.is_infinite() {
                // Either sign of overflow must rule out the candidate.
                motion = f64::INFINITY;
            }
            // NaN still propagates; roundoff-negative residuals are zero.
            cost += if motion.is_nan() {
                f64::NAN
            } else {
                motion.max(0.0)
            };
        }
        cost
    }

    /// The desired wrench `(F_B, L_B)` from the inputs.
    fn desired_wrench(&self) -> Vector6<f64> {
        let dcm_bn = mrp_to_dcm(self.hub_states_in_msg.read().sigma_bn);
        let force_b = dcm_bn * self.trans_force_in_msg.read().force_request_inertial_n;
        let torque_b = self.rot_torque_in_msg.read().torque_request_body_nm;
        Vector6::new(
            force_b.x, force_b.y, force_b.z, torque_b.x, torque_b.y, torque_b.z,
        )
    }

    fn write_outputs(&self, decision: &DVector<f64>, nanos: u64) {
        let n_joint = self.geometry.n_joint;
        self.thr_force_out_msg
            .write(ThrusterArrayForceCommandMsg::from_active(
                &decision.as_slice()[n_joint..],
            ));
        self.des_joint_angles_out_msg.write(JointArrayStateMsg {
            states: decision.as_slice()[..n_joint].to_vec(),
            state_dots: vec![0.0; n_joint],
            state_ddots: vec![0.0; n_joint],
        });
        self.des_joint_angles_time_out_msg.write(WriteTimeMsg {
            time_written_nanos: nanos,
        });
    }
}

impl Module for JointThrAllocation {
    fn init(&mut self) {
        if let Err(error) = self.validate().and_then(|()| self.configure()) {
            panic!("{}: {error}", self.config.name);
        }
        self.solution_found = false;
        self.best_err_inf = f64::NAN;
        self.wrench_error = Vector6::repeat(f64::NAN);
        self.cost_val = f64::NAN;
        self.des_joint_angles_out_msg
            .write(JointArrayStateMsg::default());
        self.des_joint_angles_time_out_msg
            .write(WriteTimeMsg::default());
        self.thr_force_out_msg
            .write(ThrusterArrayForceCommandMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let desired = self.desired_wrench();
        let n_joint = self.geometry.n_joint;
        let current_angles = if self.w_theta.is_some() {
            assert_eq!(
                self.joint_states_in_msgs.len(),
                n_joint,
                "The number of joint-state inputs must match the number of joints."
            );
            let angles: Vec<f64> = self
                .joint_states_in_msgs
                .iter()
                .map(|input| input.read().state)
                .collect();
            if angles.iter().any(|angle| !angle.is_finite()) {
                log::error!("Joint-state measurements must be finite when using Wtheta.");
            }
            Some(angles)
        } else {
            None
        };
        let options = MinimizeOptions {
            max_iterations: self.config.max_iterations,
            ftol: self.config.ftol,
        };
        let mut best: Option<(DVector<f64>, f64)> = None;
        for seed in &self.seeds {
            let minimum = minimize(
                |decision| self.cost(decision, &desired, current_angles.as_deref()),
                seed,
                &self.lower,
                &self.upper,
                options,
            );
            if !minimum.success {
                continue;
            }
            let cost = self.cost(&minimum.x, &desired, current_angles.as_deref());
            if best.as_ref().is_none_or(|(_, best_cost)| cost < *best_cost) {
                best = Some((minimum.x, cost));
            }
        }
        let decision = match best {
            Some((decision, cost)) => {
                let map = self.geometry.mapping(&decision.as_slice()[..n_joint]);
                let delivered = map * decision.rows(n_joint, self.geometry.n_thr);
                self.wrench_error = desired - Vector6::from_column_slice(delivered.as_slice());
                self.best_err_inf = self.wrench_error.amax();
                self.solution_found = true;
                self.cost_val = cost;
                decision
            }
            None => {
                self.solution_found = false;
                self.best_err_inf = f64::INFINITY;
                self.wrench_error = desired;
                self.cost_val = f64::NAN;
                let mut decision = DVector::zeros(n_joint + self.geometry.n_thr);
                if let Some(angles) = &current_angles {
                    decision.rows_mut(0, n_joint).copy_from_slice(angles);
                }
                decision
            }
        };
        self.write_outputs(&decision, context.current_sim_nanos);
    }
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `jointThrAllocation/_UnitTest/test_jointThrAllocation.py`:
    //! `test_map_matrix`, `test_spacecraft_layout` (all three angle sets),
    //! `test_spacecraft_layout_includes_hub_com_offset` and
    //! `test_spacecraft_layout_asymmetric_body_masses_shift_com` (one test),
    //! `test_allocation_configuration_helpers`,
    //! `test_joint_motion_penalty_configuration`,
    //! `test_reset_rejects_invalid_motion_weights` and
    //! `test_reset_accepts_semidefinite_motion_weights` (every case),
    //! `test_motion_penalty_is_nonnegative`,
    //! `test_motion_penalty_preserves_nan`,
    //! `test_motion_penalty_does_not_hide_overflow`,
    //! `test_cost_wraps_joint_motion` (every case) and
    //! `test_reset_rejects_missing_input_message` (the first missing input).
    //!
    //! Not ported: the tests that replace SciPy's `minimize` with a stub
    //! returning prescribed decisions and success flags
    //! (`test_update_selects_minimum_total_cost`,
    //! `test_update_uses_current_joint_states`,
    //! `test_update_rejects_nonfinite_joint_states`,
    //! `test_update_optimizer_failure`,
    //! `test_reset_clears_solution_diagnostics`): the optimizer here is not
    //! injectable. [`update_reaches_the_analytic_optimum`] runs the real
    //! optimizer on the same fixture and checks the selected solution, the
    //! diagnostics and the outputs instead.

    use super::*;
    use hifitime::Epoch;

    const NAN: f64 = f64::NAN;

    fn context(nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn close(actual: &DMatrix<f64>, expected: &DMatrix<f64>, tolerance: f64) {
        assert_eq!(actual.shape(), expected.shape());
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!(
                (a - e).abs() <= tolerance + 1e-7 * e.abs(),
                "{actual} vs {expected}"
            );
        }
    }

    /// Upstream `_configured_allocation`: the scenarios' two four-joint arms
    /// with a 980 kg hub.
    fn configured_geometry() -> ArmGeometry {
        ArmGeometry::from_config(&super::super::config::ThrArmConfig::tracking().arm_config)
    }

    /// Upstream `_expected_four_joint_layout`.
    fn expected_layout(theta: &[f64]) -> (Vec<Matrix3<f64>>, Vec<Vector3<f64>>, Vector3<f64>) {
        let rot = |axis: Vector3<f64>, angle: f64| prv_to_dcm(axis * angle);
        let (ex, ey, ez) = (Vector3::x(), Vector3::y(), Vector3::z());
        let arm2 = Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0));
        let mut dcm = vec![Matrix3::identity(); 8];
        let mut r = vec![Vector3::zeros(); 8];
        dcm[0] = rot(ex, theta[0]);
        r[0] = Vector3::new(0.79, 0.0, 0.0);
        dcm[1] = rot(ey, theta[1]) * dcm[0];
        r[1] = r[0];
        dcm[2] = rot(ez, theta[2]) * dcm[1];
        r[2] = r[1] + dcm[1].transpose() * ex;
        dcm[3] = rot(ey, theta[3]) * dcm[2];
        r[3] = r[2];
        dcm[4] = rot(ex, theta[4]) * arm2;
        r[4] = Vector3::new(-0.79, 0.0, 0.0);
        dcm[5] = rot(ey, theta[5]) * dcm[4];
        r[5] = r[4];
        dcm[6] = rot(ez, theta[6]) * dcm[5];
        r[6] = r[5] + dcm[5].transpose() * ex;
        dcm[7] = rot(ey, theta[7]) * dcm[6];
        r[7] = r[6];
        let boom = Vector3::new(0.5, 0.0, 0.0);
        let tip = Vector3::new(0.05, 0.0, 0.0);
        let numerator = (r[1] + dcm[1].transpose() * boom) * 8.0
            + (r[3] + dcm[3].transpose() * tip) * 2.0
            + (r[5] + dcm[5].transpose() * boom) * 8.0
            + (r[7] + dcm[7].transpose() * tip) * 2.0;
        (dcm, r, numerator / (980.0 + 2.0 * (8.0 + 2.0)))
    }

    #[test]
    fn map_matrix_stacks_force_and_torque() {
        let map = map_matrix(
            &[Vector3::x(), Vector3::y()],
            &[Vector3::y(), Vector3::z()],
            Vector3::zeros(),
        );
        let expected = DMatrix::from_row_slice(
            6,
            2,
            &[0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0],
        );
        close(&map, &expected, 0.0);
    }

    #[test]
    fn spacecraft_layout() {
        for theta in [
            [0.0; 8],
            [0.0, PI / 2.0, 0.0, 0.0, 0.0, -PI / 2.0, 0.0, 0.0],
            [
                PI / 6.0,
                -PI / 4.0,
                PI / 3.0,
                -PI / 6.0,
                -PI / 5.0,
                PI / 7.0,
                -PI / 8.0,
                PI / 9.0,
            ],
        ] {
            let geometry = configured_geometry();
            let (dcm, r) = geometry.joint_pose_from_theta(&theta);
            let com = geometry.com_from_theta(&dcm, &r);
            let (expected_dcm, expected_r, expected_com) = expected_layout(&theta);
            for (actual, expected) in dcm.iter().zip(&expected_dcm) {
                assert!((actual - expected).abs().max() < 1e-12);
            }
            for (actual, expected) in r.iter().zip(&expected_r) {
                assert!((actual - expected).abs().max() < 1e-12);
            }
            assert!((com - expected_com).abs().max() < 1e-12);
        }
    }

    /// Upstream's hub offset and asymmetric-mass layout tests: the centre of
    /// mass is the mass-weighted sum of the hub and the arm bodies.
    #[test]
    fn com_includes_hub_offset_and_body_masses() {
        for (offset, masses) in [
            (Vector3::new(0.12, -0.04, 0.03), vec![8.0, 2.0, 8.0, 2.0]),
            (Vector3::zeros(), vec![12.0, 3.0, 4.0, 1.0]),
        ] {
            let mut geometry = configured_geometry();
            geometry.r_bcb_b = offset;
            geometry.body_mass = masses.clone();
            let (dcm, r) = geometry.joint_pose_from_theta(&[0.0; 8]);
            let com = geometry.com_from_theta(&dcm, &r);
            let total = geometry.hub_mass + masses.iter().sum::<f64>();
            let mut expected = offset * geometry.hub_mass;
            for body in 0..4 {
                let joint = geometry.arm_joint_start[geometry.body_arm_idx[body]]
                    + geometry.body_joint_idx[body];
                expected +=
                    (r[joint] + dcm[joint].transpose() * geometry.r_lcp_p[body]) * masses[body];
            }
            assert!((com - expected / total).norm() < 1e-14);
        }
    }

    #[test]
    fn configuration_helpers_resolve_scalars() {
        assert_eq!(
            PerThruster::All(4.0).resolve(2, "thrForceMax").unwrap(),
            DVector::from_column_slice(&[4.0, 4.0])
        );
        assert_eq!(
            PerThruster::All(1e-5).resolve(2, "Wf").unwrap(),
            DVector::from_column_slice(&[1e-5, 1e-5])
        );
        assert!(
            PerThruster::Each(vec![1.0, 2.0, 3.0])
                .resolve(2, "Wf")
                .is_err()
        );
        let mut allocation = allocation_fixture();
        allocation.config.thr_force_max_n = PerThruster::All(4.0);
        allocation.configure().unwrap();
        assert_eq!(allocation.seeds.len(), 5);
        assert_eq!(allocation.seeds[0].len(), 3);
        assert_eq!((allocation.lower[0], allocation.upper[0]), (-PI, PI));
        assert_eq!((allocation.lower[2], allocation.upper[2]), (0.0, 4.0));
    }

    #[test]
    fn joint_motion_penalty_configuration() {
        assert_eq!(
            MotionWeight::Scalar(2.0).resolve(2).unwrap(),
            DMatrix::identity(2, 2) * 2.0
        );
        assert_eq!(
            MotionWeight::Diagonal(vec![1.0, 3.0]).resolve(2).unwrap(),
            DMatrix::from_diagonal(&DVector::from_column_slice(&[1.0, 3.0]))
        );
        let matrix = DMatrix::from_row_slice(2, 2, &[1.0, 0.5, 0.5, 2.0]);
        assert_eq!(
            MotionWeight::Matrix(matrix.clone()).resolve(2).unwrap(),
            matrix
        );
        let mut allocation = allocation_fixture();
        allocation.config.w_theta = Some(MotionWeight::Scalar(1.0));
        assert!(allocation.use_theta_penalty());
        assert_eq!(allocation.joint_states_in_msgs.len(), 2);
        assert!(
            allocation
                .joint_states_in_msgs
                .iter()
                .all(Input::is_connected)
        );
    }

    /// Upstream `test_reset_rejects_invalid_motion_weights`.
    #[test]
    fn invalid_motion_weights_are_rejected() {
        let m = |values: &[f64]| MotionWeight::Matrix(DMatrix::from_row_slice(2, 2, values));
        let cases = [
            (MotionWeight::Scalar(-1.0), "nonnegative"),
            (MotionWeight::Scalar(-1.0e-20), "nonnegative"),
            (MotionWeight::Scalar(NAN), "finite"),
            (MotionWeight::Scalar(f64::INFINITY), "finite"),
            (MotionWeight::Scalar(f64::NEG_INFINITY), "finite"),
            (MotionWeight::Diagonal(vec![1.0, -1.0]), "nonnegative"),
            (MotionWeight::Diagonal(vec![1.0, NAN]), "finite"),
            (MotionWeight::Diagonal(vec![1.0, f64::INFINITY]), "finite"),
            (m(&[1.0, NAN, NAN, 1.0]), "finite"),
            (m(&[1.0, f64::INFINITY, f64::INFINITY, 1.0]), "finite"),
            (m(&[1.0, 1.0, 0.0, 1.0]), "symmetric"),
            (m(&[1.0, 2.0, 2.0, 1.0]), "positive semidefinite"),
            (
                m(&[1.0e-20, 2.0e-20, 2.0e-20, 1.0e-20]),
                "positive semidefinite",
            ),
            (m(&[-1.0e-20, 0.0, 0.0, -1.0e-20]), "positive semidefinite"),
            (m(&[1.0e15, 0.0, 0.0, -1.0]), "nonnegative"),
            (m(&[1.0, 0.0, 0.0, -f64::from_bits(1)]), "nonnegative"),
            (MotionWeight::Diagonal(vec![1.0]), "length-nJoint"),
            (
                MotionWeight::Matrix(DMatrix::from_element(2, 3, 1.0)),
                "nJoint x nJoint",
            ),
        ];
        for (weight, message) in cases {
            let error = weight.resolve(2).expect_err(message);
            assert!(error.contains(message), "{error} should mention {message}");
        }
    }

    /// Upstream `test_reset_accepts_semidefinite_motion_weights`.
    #[test]
    fn semidefinite_motion_weights_are_accepted() {
        let tiny = f64::from_bits(1);
        let m = |values: &[f64]| DMatrix::from_row_slice(2, 2, values);
        let gram = DMatrix::from_row_slice(2, 1, &[0.1, 0.3]);
        let gram = &gram * gram.transpose();
        let eps = f64::EPSILON;
        let cases = [
            (MotionWeight::Scalar(0.0), DMatrix::zeros(2, 2)),
            (MotionWeight::Scalar(1e-20), DMatrix::identity(2, 2) * 1e-20),
            (
                MotionWeight::Diagonal(vec![0.0, 3.0]),
                m(&[0.0, 0.0, 0.0, 3.0]),
            ),
            (
                MotionWeight::Matrix(DMatrix::zeros(2, 2)),
                DMatrix::zeros(2, 2),
            ),
            (
                MotionWeight::Matrix(DMatrix::identity(2, 2) * tiny),
                DMatrix::identity(2, 2) * tiny,
            ),
            (
                MotionWeight::Matrix(DMatrix::from_element(2, 2, tiny)),
                DMatrix::from_element(2, 2, tiny),
            ),
            (
                MotionWeight::Matrix(m(&[1.0, -1.0, -1.0, 1.0])),
                m(&[1.0, -1.0, -1.0, 1.0]),
            ),
            (MotionWeight::Matrix(gram.clone()), gram),
            (
                MotionWeight::Matrix(m(&[1.0, -1.0, -1.0, 1.0]) * 1e-20),
                m(&[1.0, -1.0, -1.0, 1.0]) * 1e-20,
            ),
            (
                MotionWeight::Matrix(m(&[1.0, -0.5, -0.5, 1.0]) * 1e308),
                m(&[1.0, -0.5, -0.5, 1.0]) * 1e308,
            ),
            (
                MotionWeight::Matrix(m(&[1.0, 0.5 + eps, 0.5, 1.0])),
                m(&[1.0, 0.5 + eps / 2.0, 0.5 + eps / 2.0, 1.0]),
            ),
        ];
        for (weight, expected) in cases {
            let resolved = weight.resolve(2).unwrap();
            assert!(resolved.iter().all(|value| value.is_finite()));
            assert_eq!(resolved, resolved.transpose());
            let scale = expected
                .iter()
                .fold(0.0_f64, |max, value| max.max(value.abs()));
            for (actual, wanted) in resolved.iter().zip(expected.iter()) {
                assert!(
                    (actual - wanted).abs() <= 4.0 * eps * scale,
                    "{resolved} vs {expected}"
                );
            }
        }
    }

    /// Upstream `single_arm_allocation`: two x-axis hinges carrying one
    /// x-axis thruster at the centre of mass, asked for 1 N along x.
    struct Fixture {
        _messages: (
            Output<ThrusterArmConfigMsg>,
            Output<SpacecraftStateMsg>,
            Output<InertialForceCommandMsg>,
            Output<BodyTorqueCommandMsg>,
        ),
        joints: Vec<Output<ScalarJointStateMsg>>,
    }

    fn allocation_with_fixture() -> (JointThrAllocation, Fixture) {
        let mut allocation = JointThrAllocation::new(JointThrAllocationConfig::default());
        let config = ThrusterArmConfigMsg {
            hub_mass_kg: 1.0,
            arm_joint_count: vec![2],
            thr_arm_idx: vec![0],
            thr_arm_joint_idx: vec![1],
            r_cp_p_m: vec![Vector3::zeros(); 2],
            r_tp_p_m: vec![Vector3::zeros()],
            s_hat_p: vec![Vector3::x(); 2],
            f_hat_p: vec![Vector3::x()],
            dcm_c0p: vec![Matrix3::identity(); 2],
            ..ThrusterArmConfigMsg::default()
        };
        let messages = (
            Output::new(config),
            Output::new(SpacecraftStateMsg::default()),
            Output::new(InertialForceCommandMsg {
                force_request_inertial_n: Vector3::x(),
            }),
            Output::new(BodyTorqueCommandMsg::default()),
        );
        messages.0.connect_to(&mut allocation.arm_config_in_msg);
        messages.1.connect_to(&mut allocation.hub_states_in_msg);
        messages.2.connect_to(&mut allocation.trans_force_in_msg);
        messages.3.connect_to(&mut allocation.rot_torque_in_msg);
        let joints: Vec<_> = (0..2)
            .map(|_| Output::new(ScalarJointStateMsg::default()))
            .collect();
        for joint in &joints {
            joint.connect_to(allocation.add_hinged_joint());
        }
        (
            allocation,
            Fixture {
                _messages: messages,
                joints,
            },
        )
    }

    fn allocation_fixture() -> JointThrAllocation {
        let (allocation, fixture) = allocation_with_fixture();
        // The outputs outlive the fixture: inputs hold their slots.
        drop(fixture);
        allocation
    }

    /// Upstream `test_cost_wraps_joint_motion`: a 0.5 N candidate for a 1 N
    /// request costs 0.25 + 0.5, plus the penalty of the wrapped angle
    /// errors 0.1 and -0.2 rad, however many turns apart the angles are.
    #[test]
    fn cost_wraps_joint_motion() {
        let m = |values: &[f64]| MotionWeight::Matrix(DMatrix::from_row_slice(2, 2, values));
        let cases = [
            (None, 0.0),
            (Some(MotionWeight::Scalar(2.0)), 0.10),
            (Some(MotionWeight::Diagonal(vec![1.0, 3.0])), 0.13),
            (Some(m(&[1.0, 0.5, 0.5, 2.0])), 0.07),
        ];
        for extra_turns in [0.0, 3.0] {
            for (weight, expected_penalty) in cases.clone() {
                let (mut allocation, _fixture) = allocation_with_fixture();
                allocation.config.wf = PerThruster::All(1.0);
                let penalized = weight.is_some();
                allocation.config.w_theta = weight;
                allocation.init();
                let decision = DVector::from_column_slice(&[-PI + 0.05, PI - 0.1, 0.5]);
                let current = [
                    PI - 0.05 + extra_turns * 2.0 * PI,
                    -PI + 0.1 - extra_turns * 2.0 * PI,
                ];
                let desired = Vector6::new(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                let cost = allocation.cost(&decision, &desired, penalized.then_some(&current[..]));
                let expected = 0.75 + expected_penalty;
                assert!(
                    (cost - expected).abs() <= 1e-6 * expected,
                    "{cost} vs {expected}"
                );
            }
        }
    }

    /// Upstream `test_motion_penalty_is_nonnegative`: near-null directions of
    /// nearly semidefinite weights cost nothing, never a negative amount.
    #[test]
    fn motion_penalty_is_nonnegative() {
        let eps = f64::EPSILON;
        for (weight, angle_error) in [
            ([1.0e15, 1.0e8, 1.0e8, 9.0], [-1.0e-7, 1.0]),
            ([1.0, 1.0 + eps, 1.0 + eps, 1.0], [1.0, -1.0]),
            ([1.0e16, 1.0e8, 1.0e8, 1.0], [-1.0e-8, 1.0]),
        ] {
            let (mut allocation, _fixture) = allocation_with_fixture();
            allocation.config.w_theta =
                Some(MotionWeight::Matrix(DMatrix::from_row_slice(2, 2, &weight)));
            allocation.init();
            let resolved = allocation.w_theta.clone().unwrap();
            assert!(resolved.symmetric_eigenvalues().min() >= -1e-12);
            let decision = DVector::from_column_slice(&[angle_error[0], angle_error[1], 0.0]);
            let cost = allocation.cost(&decision, &Vector6::zeros(), Some(&[0.0, 0.0]));
            assert!(cost.is_finite() && (0.0..=1e-12).contains(&cost), "{cost}");
        }
    }

    /// Upstream `test_motion_penalty_preserves_nan`.
    #[test]
    fn motion_penalty_preserves_nan() {
        let (mut allocation, _fixture) = allocation_with_fixture();
        allocation.config.w_theta = Some(MotionWeight::Scalar(1.0));
        allocation.init();
        let cost = allocation.cost(&DVector::zeros(3), &Vector6::zeros(), Some(&[NAN, 0.0]));
        assert!(cost.is_nan());
    }

    /// Upstream `test_motion_penalty_does_not_hide_overflow`.
    #[test]
    fn motion_penalty_does_not_hide_overflow() {
        let (mut allocation, _fixture) = allocation_with_fixture();
        allocation.config.w_theta = Some(MotionWeight::Matrix(DMatrix::from_row_slice(
            2,
            2,
            &[
                6.227709133842637e307,
                -7.891547020745441e307,
                -7.891547020745441e307,
                1.0e308,
            ],
        )));
        allocation.init();
        let decision = DVector::from_column_slice(&[-2.7992763360330044, -1.7941584448303567, 0.0]);
        let cost = allocation.cost(&decision, &Vector6::zeros(), Some(&[0.0, 0.0]));
        assert!(cost.is_nan() || cost > 0.0, "{cost}");
    }

    /// Upstream `test_reset_rejects_missing_input_message`.
    #[test]
    fn missing_inputs_are_reported() {
        let allocation = JointThrAllocation::new(JointThrAllocationConfig::default());
        let error = allocation.validate().unwrap_err();
        assert!(error.contains("JointThrAllocation.armConfigInMsg"));
    }

    /// The single-arm fixture has an analytic optimum: 1 N along x at any
    /// angles, which leaves only the thrust weight `W_f f` above zero. The
    /// selected solution, its diagnostics and the published outputs match.
    #[test]
    fn update_reaches_the_analytic_optimum() {
        for penalty in [false, true] {
            let (mut allocation, fixture) = allocation_with_fixture();
            if penalty {
                allocation.config.w_theta = Some(MotionWeight::Scalar(1e-3));
                for (joint, angle) in fixture.joints.iter().zip([0.2, -0.3]) {
                    joint.write(ScalarJointStateMsg { state: angle });
                }
            }
            allocation.init();
            assert!(!allocation.solution_found);
            allocation.update(&context(7));
            assert!(allocation.solution_found);
            let forces = allocation.thr_force_out_msg.read().thrust_force_n;
            // The optimum trades the wrench error against the thrust
            // weight: f = 1 - W_f / 2.
            assert!(
                (forces[0] - (1.0 - 1e-6 / 2.0)).abs() < 1e-6,
                "{}",
                forces[0]
            );
            assert!(allocation.best_err_inf < 1e-5);
            assert!(allocation.cost_val.is_finite() && allocation.cost_val < 2e-6);
            let angles = allocation.des_joint_angles_out_msg.read();
            assert_eq!(angles.states.len(), 2);
            assert_eq!(angles.state_dots, vec![0.0, 0.0]);
            if penalty {
                // The motion penalty keeps the angles at the measured ones.
                assert!(
                    (angles.states[0] - 0.2).abs() < 1e-3 && (angles.states[1] + 0.3).abs() < 1e-3
                );
            }
            assert_eq!(
                allocation
                    .des_joint_angles_time_out_msg
                    .read()
                    .time_written_nanos,
                7
            );
        }
    }
}
