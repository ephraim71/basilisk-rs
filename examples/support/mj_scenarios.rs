//! Pieces shared by the ports of upstream's `examples/mujoco/` scenarios
//! (`mujoco` feature):
//!
//! | upstream | here |
//! |---|---|
//! | `mujoco.MJScene.fromFile(f"{CURRENT_FOLDER}/<file>.xml")` | [`load_scene`], from `examples/data/mujoco/` |
//! | `svIntegrators.svIntegratorRKF45(scene)` with `setRelativeTolerance` / `setAbsoluteTolerance` | [`rkf45`] |
//! | `scSim.InitializeSimulation()`, then `body.setPosition(...)` and the other state setters, then `ExecuteSimulation()` | a [`Shared`](super::shared::Shared) scene, [`Simulation::initialize`](basilisk_rs::simulation::Simulation::initialize), the setters on the locked scene, then `run_for` |
//! | the site `stateOutMsg.recorder()` columns | [`site_state_columns`] |
//! | `generateProfiles` (`scenarioDeployPanels`, `scenarioBranchingPanels`, `scenarioMJSceneVizard`) | [`trapezoid_profile`] |
//! | the scripts' Python `PIDController(StatefulSysModel)` | [`PidController`] |
//! | `gravFactory.addBodiesTo(scene)` | [`add_gravity_to_all_bodies`] |
//! | `arrayMotorTorqueToSingleActuators`' `SingleActuatorMsg` outputs | the port's [`MotorTorqueMsg`] outputs through [`MotorTorqueToSingleActuators`] |
//!
//! Upstream's RKF45 accepts a step when every state's error ratio
//! `|high - low| / (|high| relTol + absTol)` is at most one; basilisk-rs's
//! [`Rkf45`] takes the RMS of those ratios over the scene's states (with an
//! absolute-only tolerance on free-joint translations, as upstream), so the
//! two choose different steps for the same tolerances.

#![allow(dead_code)]

use std::path::PathBuf;

use basilisk_rs::dynamics::gravity::PointMassGravityModel;
use basilisk_rs::integrators::{AdaptiveController, Rkf45};
use basilisk_rs::messages::{
    Input, MotorTorqueMsg, Output, PlanetStateMsg, ScalarJointStateMsg, SingleActuatorMsg,
};
use basilisk_rs::mujoco_dynamics::n_body_gravity::NBodyGravity;
use basilisk_rs::mujoco_dynamics::{MjScene, ModelStateSpec, SceneModel, TaskContext, TaskModelId};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{Module, SimulationContext};
use hifitime::Epoch;

use super::common::vector_columns;

/// Where the upstream XML models are kept.
pub const MODEL_DIR: &str = "examples/data/mujoco";

/// Upstream `svIntegratorAdaptiveRungeKutta::relTol` default.
pub const RKF45_DEFAULT_RTOL: f64 = 1.0e-4;
/// Upstream `svIntegratorAdaptiveRungeKutta::absTol` default.
pub const RKF45_DEFAULT_ATOL: f64 = 1.0e-8;

/// The simulation epoch of the scenarios that do not set one.
pub fn epoch() -> Epoch {
    Epoch::from_gregorian_utc_at_midnight(2025, 11, 15)
}

/// `examples/data/mujoco/<file>`.
pub fn model_path(file: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join(MODEL_DIR)
        .join(file)
}

/// Where [`scenario_csv`](super::recording::scenario_csv) writes
/// `<stem>.csv`, for printing (unlike `scenario_output_path`, this does not
/// remove the file).
pub fn written_csv(stem: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("examples/output/scenarios")
        .join(format!("{stem}.csv"))
}

/// Upstream `mujoco.MJScene.fromFile(path)` of a model in
/// `examples/data/mujoco/`.
pub fn load_scene(file: &str) -> MjScene {
    MjScene::from_file(model_path(file))
        .unwrap_or_else(|error| panic!("cannot load MuJoCo model {file}: {error}"))
}

/// Upstream `svIntegratorRKF45(scene)` with `setRelativeTolerance(rtol)`
/// and `setAbsoluteTolerance(atol)`.
pub fn rkf45(rtol: f64, atol: f64) -> Rkf45 {
    Rkf45::new(AdaptiveController {
        rtol,
        atol,
        ..AdaptiveController::default()
    })
}

/// The columns of a site's `stateOutMsg` (a `SpacecraftStateMsg`), prefixed
/// with `prefix` when it is not empty: `r_BN_N_m_i`, `v_BN_N_m_per_s_i`,
/// `sigma_BN_i` and `omega_BN_B_rad_per_s_i`.
pub fn site_state_columns(prefix: &str) -> CsvSourceConfig {
    let name = |column: &str| {
        if prefix.is_empty() {
            column.to_string()
        } else {
            format!("{prefix}_{column}")
        }
    };
    CsvSourceConfig::columns(
        vector_columns("position_m", &name("r_BN_N_m"))
            .into_iter()
            .chain(vector_columns("velocity_mps", &name("v_BN_N_m_per_s")))
            .chain(vector_columns("sigma_bn", &name("sigma_BN")))
            .chain(vector_columns("omega_radps", &name("omega_BN_B_rad_per_s"))),
    )
}

/// One scalar column of a message, `(source path, column)`.
pub fn scalar_column(source: &str, column: &str) -> CsvSourceConfig {
    CsvSourceConfig::columns([(source.to_string(), column.to_string())])
}

/// `(time [ns], value)` points of an interpolator profile.
pub type Profile = Vec<(f64, f64)>;

/// Upstream `generateProfiles(initialPoint, finalPoint, vMax, aMax,
/// timeOffset)`: the trapezoidal (or, when `vMax` is not reached,
/// triangular) point-to-point profile, sampled at 100 evenly spaced times
/// from `time_offset_nanos`. Returns the `(time [ns], value)` points of the
/// position and of the velocity, which upstream fits with degree-1
/// `ScalarJointStateInterpolator`s.
pub fn trapezoid_profile(
    initial: f64,
    target: f64,
    v_max: f64,
    a_max: f64,
    time_offset_nanos: f64,
) -> (Profile, Profile) {
    let distance = (target - initial).abs();
    let mut t_accel = v_max / a_max;
    let d_accel = 0.5 * a_max * t_accel * t_accel;
    let t_const = if 2.0 * d_accel > distance {
        t_accel = (distance / a_max).sqrt();
        0.0
    } else {
        (distance - 2.0 * d_accel) / v_max
    };
    let t_total = 2.0 * t_accel + t_const;
    let sign = if target > initial { 1.0 } else { -1.0 };
    let count = 100;
    let mut positions = Vec::with_capacity(count);
    let mut velocities = Vec::with_capacity(count);
    for index in 0..count {
        // `np.linspace(0, tTotal, 100)`.
        let t = if index == count - 1 {
            t_total
        } else {
            t_total * index as f64 / (count - 1) as f64
        };
        let (position, velocity) = if t < t_accel {
            (0.5 * a_max * t * t, a_max * t)
        } else if t < t_accel + t_const {
            (d_accel + v_max * (t - t_accel), v_max)
        } else {
            let t_decel = t - t_accel - t_const;
            (
                d_accel + v_max * t_const + (v_max * t_decel - 0.5 * a_max * t_decel * t_decel),
                v_max - a_max * t_decel,
            )
        };
        let time = time_offset_nanos + t * 1e9;
        positions.push((time, initial + sign * position));
        velocities.push((time, sign * velocity));
    }
    (positions, velocities)
}

/// The Python `PIDController(StatefulSysModel)` of `scenarioDeployPanels`
/// and `scenarioMJSceneVizard`: an analog PID on a joint,
///
/// ```text
/// u = Kp (θ_desired - θ) + Kd (θ̇_desired - θ̇) + Ki e_I,    ė_I = θ_desired - θ,
/// ```
///
/// with the integral error a scene state (`integralError`, zero at reset),
/// optionally clamped to `±output_limit` (`scenarioMJSceneVizard` clips to
/// ±10 N m). Unlike [`JointPidController`](basilisk_rs::mujoco_dynamics::joint_pid_controller::JointPidController),
/// upstream's C++ model, it integrates the error.
#[derive(Debug)]
pub struct PidController {
    pub name: String,
    pub k_p: f64,
    pub k_d: f64,
    pub k_i: f64,
    /// Clamp of the output, `None` for none.
    pub output_limit: Option<f64>,
    pub measured_in_msg: Input<ScalarJointStateMsg>,
    pub desired_in_msg: Input<ScalarJointStateMsg>,
    pub measured_dot_in_msg: Input<ScalarJointStateMsg>,
    pub desired_dot_in_msg: Input<ScalarJointStateMsg>,
    pub output_out_msg: Output<SingleActuatorMsg>,
}

impl PidController {
    /// `scenarioDeployPanels`' gains: `K_p = 0.1`, `K_d = 0.002`,
    /// `K_i = 0.0001`, no clamp.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            k_p: 0.1,
            k_d: 0.002,
            k_i: 0.0001,
            output_limit: None,
            measured_in_msg: Input::default(),
            desired_in_msg: Input::default(),
            measured_dot_in_msg: Input::default(),
            desired_dot_in_msg: Input::default(),
            output_out_msg: Output::default(),
        }
    }
}

impl SceneModel for PidController {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn state_specs(&self) -> Vec<ModelStateSpec> {
        vec![ModelStateSpec::new("integralError", vec![0.0])]
    }

    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        let error = self.desired_in_msg.read().state - self.measured_in_msg.read().state;
        let error_dot =
            self.desired_dot_in_msg.read().state - self.measured_dot_in_msg.read().state;
        let integral = context.state(0)[0];
        let mut output = self.k_p * error + self.k_d * error_dot + self.k_i * integral;
        if let Some(limit) = self.output_limit {
            output = output.clamp(-limit, limit);
        }
        context.set_derivative(0, &[error]);
        self.output_out_msg
            .write(SingleActuatorMsg { input: output });
    }
}

/// One body of upstream's `gravBodyFactory`, as `addBodiesTo(scene)` hands
/// it to `NBodyGravity`: a point mass with its ephemeris output.
pub struct GravitySourceSpec<'a> {
    /// Upstream's factory key (`"earth"`, `"moon"`, `"sun"`).
    pub name: &'a str,
    pub mu_m3ps2: f64,
    pub is_central_body: bool,
    /// The body's `planetBodyInMsg` source, if it has one.
    pub ephemeris: Option<&'a Output<PlanetStateMsg>>,
}

/// Upstream `gravFactory.addBodiesTo(scene)`: an `NBodyGravity` named
/// `<scene>_gravity` with every factory body as a source and every scene
/// body as a target at its center of mass, in the scene's dynamics task at
/// upstream's default priority -1.
pub fn add_gravity_to_all_bodies(
    scene: &mut MjScene,
    sources: &[GravitySourceSpec<'_>],
) -> TaskModelId<NBodyGravity> {
    let tag = if scene.name.is_empty() {
        "mujocoScene".to_string()
    } else {
        scene.name.clone()
    };
    let mut gravity = NBodyGravity::new(format!("{tag}_gravity"));
    for spec in sources {
        let model = PointMassGravityModel::new(spec.mu_m3ps2).expect("positive mu");
        let source = gravity.add_gravity_source(spec.name, model, spec.is_central_body);
        if let Some(ephemeris) = spec.ephemeris {
            ephemeris.connect_to(&mut source.state_in_msg);
        }
    }
    gravity.add_gravity_targets_for_all_bodies(scene);
    scene.add_model_to_dynamics_task(gravity, -1)
}

/// The commands of basilisk-rs's
/// [`ArrayMotorTorqueToSingleActuators`](basilisk_rs::device_interface::array_motor_torque_to_single_actuators::ArrayMotorTorqueToSingleActuators)
/// as the `SingleActuatorMsg`s upstream's module publishes: the port
/// publishes [`MotorTorqueMsg`]s, which its reaction-wheel effectors read,
/// while the MuJoCo joint motors (and upstream's `saturationSingleActuator`
/// after the splitter) read [`SingleActuatorMsg`]. Copies
/// `motor_torque_nm` of input `i` to `input` of output `i` every update;
/// schedule it right after the splitter.
#[derive(Debug)]
pub struct MotorTorqueToSingleActuators {
    pub motor_torque_in_msgs: Vec<Input<MotorTorqueMsg>>,
    pub actuator_out_msgs: Vec<Output<SingleActuatorMsg>>,
}

impl MotorTorqueToSingleActuators {
    pub fn new(count: usize) -> Self {
        Self {
            motor_torque_in_msgs: (0..count).map(|_| Input::default()).collect(),
            actuator_out_msgs: (0..count).map(|_| Output::default()).collect(),
        }
    }

    fn publish(&self) {
        for (input, output) in self
            .motor_torque_in_msgs
            .iter()
            .zip(&self.actuator_out_msgs)
        {
            output.write(SingleActuatorMsg {
                input: input.read().motor_torque_nm,
            });
        }
    }
}

impl Module for MotorTorqueToSingleActuators {
    fn init(&mut self) {
        self.publish();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.publish();
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.publish();
    }
}

#[cfg(test)]
mod tests {
    use super::trapezoid_profile;

    #[test]
    fn trapezoid_reaches_the_target_at_rest() {
        let (vmax, amax) = (0.05_f64.to_radians(), 0.0001_f64.to_radians());
        let (positions, velocities) = trapezoid_profile(std::f64::consts::PI, 0.0, vmax, amax, 0.0);
        assert_eq!(positions.len(), 100);
        let (t_end, end) = *positions.last().unwrap();
        assert!(end.abs() < 1e-9, "{end}");
        assert!(velocities.last().unwrap().1.abs() < 1e-12);
        // 500 s ramps, then cruise at vmax to cover pi.
        let expected = (2.0 * 500.0 + (std::f64::consts::PI - vmax * 500.0) / vmax) * 1e9;
        assert!((t_end - expected).abs() < 1.0, "{t_end} vs {expected}");
        assert!(
            velocities
                .iter()
                .all(|(_, v)| *v <= 0.0 && *v >= -vmax - 1e-15)
        );
    }
}
