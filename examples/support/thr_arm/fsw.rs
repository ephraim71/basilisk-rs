//! Upstream `examples/mujoco/mujocoModels/BSK_MujocoFSW.py`: the flight
//! software of the thruster-arm scenarios, its tasks, gateway messages and
//! the three events that sequence joint motion and thruster firing.
//!
//! Four FSW tasks ([`TaskSwitch`]es):
//!
//! | task | period | modules (priority) |
//! |---|---|---|
//! | `outerLoopTask` | `outerLoopPeriod` | regulation: `inertial3D` (50), `stateMerge`, `navConverter`, `attError`, `mrpFeedback`, `inertialCartFeedback` (45); tracking: `stateMerge` (50), `navConverter`, `attRef`, `mrpFeedback`, `arcLineHoldRef`, `inertialCartFeedback` (45); then `jointThrAllocation` (44), `thrFiringRound` (43) |
//! | `jointMotionTask` | `fswRate` | [`jointProfiler` (51)], `hingedJointArrayMotor` (50), [`jointMotionCompensator` (49)] |
//! | `thrFiringTask` | `fswRate` | `thrOnTimeToForce` (50), `thrJointCompensation` (49) |
//! | `recorderTask` | `fswRate` | the scenario's recorders |
//!
//! The actuator commands go through gateway messages that several modules
//! write and the MuJoCo actuators read: one joint torque per joint (written
//! by `hingedJointArrayMotor` and `thrJointCompensation`), one force per
//! thruster (`thrOnTimeToForce`), three hub torques (`jointMotionCompensator`)
//! and the thruster on-time commands (the events). Upstream redirects the
//! module outputs to the gateway messages; here the outputs are replaced by
//! clones of the gateway [`Output`]s, which alias them.
//!
//! The events, checked every `fswRate`:
//!
//! - `jointMotion` (active): when a new allocation is published, caches the
//!   desired joint states and the on-times, then either starts joint motion
//!   (a pulse is pending), keeps joint motion going (no pulse, joints still
//!   moving), or cancels any firing and starts coasting;
//! - `thrFiring`: once every joint is within the angle and rate tolerances
//!   of its commanded state, stops joint motion and starts the pending
//!   pulse;
//! - `coast`: once every thruster force is zero, stops firing until the next
//!   allocation.
//!
//! Every action disables all FSW tasks, zeroes the gateways and enables the
//! outer-loop and recorder tasks before enabling its phase's task.
//!
//! Stand-ins for upstream's message headers: the desired joint states'
//! `timeWritten()` is [`WriteTimeMsg`] from the allocation (see
//! [`joint_thr_allocation`](super::joint_thr_allocation)); the on-time
//! gateway is one [`ThrusterCommandMsg`] per thruster whose
//! `command_time_nanos` is the time upstream passes to `write` (0 for the
//! zeroing writes, as upstream's `write(payload)`), which is what
//! [`ThrOnTimeToForce`] latches on.

use std::sync::{Arc, Mutex};

use basilisk_rs::device_interface::joint_array_ref_profiler::{
    JointArrayRefProfiler, JointArrayRefProfilerConfig,
};
use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::fsw_algorithms::att_guidance::location_pointing::{
    LocationPointing, LocationPointingConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::hinged_joint_array_motor::{
    HingedJointArrayMotor, HingedJointArrayMotorConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::joint_motion_compensator::{
    JointMotionCompensator, JointMotionCompensatorConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::thr_joint_compensation::{
    ThrJointCompensation, ThrJointCompensationConfig,
};
use basilisk_rs::fsw_algorithms::formation_flying::inertial_cart_feedback::{
    InertialCartFeedback, InertialCartFeedbackConfig,
};
use basilisk_rs::messages::{
    Input, JointArrayStateMsg, NavigationTranslationMsg, Output, ScalarJointStateMsg,
    SingleActuatorMsg, ThrusterArmConfigMsg, ThrusterCommandMsg, TranslationReferenceMsg,
    VehicleConfigMsg,
};
use basilisk_rs::mujoco_dynamics::thr_on_time_to_force::{
    ThrOnTimeToForce, ThrOnTimeToForceConfig,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;

use super::arc_line_hold_ref::{ArcLineHoldRef, ArcLineHoldRefConfig};
use super::config::{ControlObjective, ThrArmConfig};
use super::dynamics::MujocoDynamics;
use super::events::{Check, EventActivity, Events, InTasks, TaskSwitch};
use super::joint_thr_allocation::{
    JointThrAllocation, JointThrAllocationConfig, MotionWeight, PerThruster, WriteTimeMsg,
    wrap_angle,
};
use super::state_merge::StateMerge;
use super::thr_firing_round::{ThrFiringRound, ThrFiringRoundConfig};
use super::{JOINT_MOTION_PRIORITY, OUTER_LOOP_PRIORITY, THR_FIRING_PRIORITY, sec2nano};

/// Upstream's four FSW tasks.
#[derive(Clone, Debug)]
pub struct FswTasks {
    pub outer_loop: TaskSwitch,
    pub joint_motion: TaskSwitch,
    pub thr_firing: TaskSwitch,
    pub recorder: TaskSwitch,
}

impl FswTasks {
    fn new() -> Self {
        Self {
            outer_loop: TaskSwitch::new(false),
            joint_motion: TaskSwitch::new(false),
            thr_firing: TaskSwitch::new(false),
            recorder: TaskSwitch::new(false),
        }
    }

    /// Upstream `fswProc.disableAllTasks()`.
    fn disable_all(&self) {
        for task in [
            &self.outer_loop,
            &self.joint_motion,
            &self.thr_firing,
            &self.recorder,
        ] {
            task.disable();
        }
    }
}

/// Upstream's gateway messages.
#[derive(Clone, Debug)]
pub struct Gateways {
    /// Upstream `jointTorqueGateways` \[N m\].
    pub joint_torque: Vec<Output<SingleActuatorMsg>>,
    /// Upstream `thrForceGateways` \[N\].
    pub thr_force: Vec<Output<SingleActuatorMsg>>,
    /// Upstream `bodyTorqueGateway` \[N m\].
    pub body_torque: Vec<Output<SingleActuatorMsg>>,
    /// Upstream `thrFiringTimeGateway`, one command per thruster.
    pub thr_firing_time: Vec<Output<ThrusterCommandMsg>>,
}

impl Gateways {
    fn new(joints: usize, thrusters: usize) -> Self {
        fn outputs<T: Clone + Default>(count: usize) -> Vec<Output<T>> {
            (0..count).map(|_| Output::default()).collect()
        }
        Self {
            joint_torque: outputs(joints),
            thr_force: outputs(thrusters),
            body_torque: outputs(3),
            thr_firing_time: outputs(thrusters),
        }
    }

    /// Upstream `zeroGatewayMsgs`.
    pub fn zero(&self) {
        for output in self
            .joint_torque
            .iter()
            .chain(&self.thr_force)
            .chain(&self.body_torque)
        {
            output.write(SingleActuatorMsg { input: 0.0 });
        }
        for output in &self.thr_firing_time {
            output.write(ThrusterCommandMsg::default());
        }
    }

    /// Upstream `thrFiringTimeGateway.write(payload, CurrentSimNanos)`.
    fn write_on_times(&self, on_times_s: &[f64], nanos: u64) {
        for (output, on_time_s) in self.thr_firing_time.iter().zip(on_times_s) {
            output.write(ThrusterCommandMsg {
                on_time_s: *on_time_s,
                command_time_nanos: nanos,
            });
        }
    }
}

/// Upstream's event bookkeeping (`lastDesAngles` ... `thrustJustActivated`).
#[derive(Debug)]
struct EventState {
    last_des_angles: Vec<f64>,
    last_des_angle_rates: Vec<f64>,
    /// Upstream `lastDesCmdWriteTime` (-1 initially).
    last_des_cmd_write_nanos: Option<u64>,
    pending_on_times_s: Vec<f64>,
    pending_firing: bool,
    firing_cancellation_pending: bool,
    joint_motion_phase_active: bool,
    thrust_just_activated: bool,
}

/// The upstream readers the events use (`desAngleReader`, `thrTimeReader`,
/// `anglesReaders`, `angleRatesReaders`, `thrForceReaders`).
#[derive(Clone, Debug)]
struct Readers {
    des_angles: Input<JointArrayStateMsg>,
    des_angles_time: Input<WriteTimeMsg>,
    on_times: Vec<Input<ThrusterCommandMsg>>,
    angles: Vec<Input<ScalarJointStateMsg>>,
    angle_rates: Vec<Input<ScalarJointStateMsg>>,
    thr_forces: Vec<Input<SingleActuatorMsg>>,
}

/// Upstream `BSKMujocoFSWModels`.
pub struct MujocoFsw {
    pub config: ThrArmConfig,
    pub tasks: FswTasks,
    pub gateways: Gateways,
    /// Upstream `vcMsg`, `thrArmConfigMsg`, `transRefMsg` (regulation) and
    /// `scTargetMsg` (tracking).
    pub vc_msg: Output<VehicleConfigMsg>,
    pub thr_arm_config_msg: Output<ThrusterArmConfigMsg>,
    pub trans_ref_msg: Output<TranslationReferenceMsg>,
    pub sc_target_msg: Output<NavigationTranslationMsg>,
    /// Upstream `desAngleReader` (with its write-time stand-in): what the
    /// `jointMotion` event reads. Clones alias the event's reader, so
    /// reconnecting these reconnects it.
    pub des_angle_reader: Input<JointArrayStateMsg>,
    pub des_angle_time_reader: Input<WriteTimeMsg>,
    /// Outer loop.
    pub inertial_3d: Option<InTasks<Inertial3D>>,
    pub att_error: Option<InTasks<AttTrackingError>>,
    pub trans_ref: Option<InTasks<ArcLineHoldRef>>,
    pub att_ref: Option<InTasks<LocationPointing>>,
    pub state_merge: InTasks<StateMerge>,
    pub nav_converter: InTasks<SimpleNavigation>,
    pub mrp_feedback: InTasks<MrpFeedback>,
    pub inertial_cart_feedback: InTasks<InertialCartFeedback>,
    pub joint_thr_allocation: InTasks<JointThrAllocation>,
    pub thr_firing_round: InTasks<ThrFiringRound>,
    /// Joint motion.
    pub joint_profiler: Option<InTasks<JointArrayRefProfiler>>,
    pub hinged_joint_array_motor: InTasks<HingedJointArrayMotor>,
    pub joint_motion_compensator: Option<InTasks<JointMotionCompensator>>,
    /// Thruster firing.
    pub thr_on_time_to_force: InTasks<ThrOnTimeToForce>,
    pub thr_joint_compensation: InTasks<ThrJointCompensation>,
    outer_period_nanos: u64,
    inner_period_nanos: u64,
}

impl MujocoFsw {
    /// Upstream `BSKMujocoFSWModels(SimBase, fswRate, config)`: builds and
    /// connects the modules, the gateways and the events (added to
    /// `events`).
    pub fn new(
        fsw_rate_s: f64,
        config: &ThrArmConfig,
        dynamics: &mut MujocoDynamics,
        events: &mut Events,
    ) -> Self {
        let config = config.clone();
        let tracking = config.control_objective == ControlObjective::Tracking;
        let inner_period_nanos = sec2nano(fsw_rate_s);
        let outer_period_nanos = sec2nano(config.outer_loop_period_s);
        let (n_joint, n_thr) = (config.num_joints, config.num_thrusters);
        assert!(
            !config.use_theta_alloc_penalty || config.theta_weight.is_some(),
            "A theta allocation penalty requires a configured thetaWeight."
        );
        assert!(
            !config.use_joint_profiler || config.joint_profile.is_some(),
            "A joint motion profile requires a configured jointProfileType."
        );
        let tasks = FswTasks::new();
        let outer = &tasks.outer_loop;

        // Upstream `setupGatewayMsgs`: the gateways drive the actuators.
        let gateways = Gateways::new(n_joint, n_thr);
        gateways.zero();
        let mut scene = dynamics.scene.lock();
        for (names, outputs) in [
            (&config.joint_act_names, &gateways.joint_torque),
            (&config.thr_act_names, &gateways.thr_force),
            (&config.body_act_names, &gateways.body_torque),
        ] {
            for (name, output) in names.iter().zip(outputs) {
                output.connect_to(&mut scene.single_actuator_mut(name).actuator_in_msg);
            }
        }
        let hub_origin = scene.body(&config.hub_name).origin().state_out_msg.clone();
        let joints: Vec<(Output<ScalarJointStateMsg>, Output<ScalarJointStateMsg>)> = config
            .joint_refs
            .iter()
            .map(|(body, joint)| {
                let joint = scene.body(body).scalar_joint(joint).clone();
                (joint.state_out_msg, joint.state_dot_out_msg)
            })
            .collect();
        drop(scene);
        let com_states = dynamics.system_com.com_states_out_msg.clone();
        let mass_matrix = dynamics.system_mass_matrix.mass_matrix_out_msg.clone();
        let reaction_forces = dynamics
            .joint_reaction_forces
            .reaction_forces_out_msg
            .clone();

        // Upstream `SetVehicleConfiguration`, `SetThrArmConfig`, `SetTransRef`.
        let vc_msg = Output::new(VehicleConfigMsg {
            mass_kg: config.vehicle_mass_kg,
            inertia_about_point_b_kg_m2: config.hub_box_inertia_kg_m2(),
            ..VehicleConfigMsg::default()
        });
        let thr_arm_config_msg = Output::new(config.arm_config.clone());
        let trans_ref_msg = Output::new(TranslationReferenceMsg {
            position_m: config.trans_ref_position_m,
            velocity_mps: config.trans_ref_velocity_mps,
            acceleration_mps2: config.trans_ref_acceleration_mps2,
        });
        let sc_target_msg = Output::new(NavigationTranslationMsg {
            position_bn_n_m: config.sc_target_position_m,
            ..NavigationTranslationMsg::default()
        });

        // Upstream `SetStateMerge`, `SetNavConverter`.
        let mut state_merge = StateMerge::default();
        hub_origin.connect_to(&mut state_merge.att_state_in_msg);
        com_states.connect_to(&mut state_merge.trans_state_in_msg);
        let mut nav_converter = SimpleNavigation::new("navConverter");
        state_merge
            .state_out_msg
            .connect_to(&mut nav_converter.spacecraft_state_in_msg);

        // Upstream `SetAttRef` / `SetInertial3D`, `SetAttError`, `SetTransRef`.
        let (mut inertial_3d, mut att_error, mut trans_ref, mut att_ref) = (None, None, None, None);
        let guidance = if tracking {
            let mut pointing = LocationPointing::new(LocationPointingConfig {
                name: "attRef".to_string(),
                p_hat_b: config.p_hat_b,
                small_angle_rad: config.pointing_small_angle_rad.unwrap_or(0.0),
                use_boresight_rate_damping: config.use_boresight_rate_damping,
                ..LocationPointingConfig::default()
            });
            nav_converter
                .attitude_out_msg
                .connect_to(&mut pointing.sc_att_in_msg);
            nav_converter
                .translation_out_msg
                .connect_to(&mut pointing.sc_trans_in_msg);
            sc_target_msg.connect_to(&mut pointing.sc_target_in_msg);
            let guidance = pointing.att_guid_out_msg.clone();
            att_ref = Some(InTasks::new(pointing, &[outer]));
            let reference = ArcLineHoldRef::new(ArcLineHoldRefConfig {
                name: "arcLineHoldRef".to_string(),
                radius_m: config.trans_radius_m,
                final_position_m: config.final_position_m,
                blend_end_position_m: config.blend_end_position_m,
                t_circle_s: config.t_circle_s,
                t_transition_s: config.t_transition_s,
                t_linear_s: config.t_linear_s,
            });
            trans_ref = Some(InTasks::new(reference, &[outer]));
            guidance
        } else {
            let reference = Inertial3D::new(Inertial3DConfig {
                name: "inertial3D".to_string(),
                sigma_r0n: config.att_ref_sigma_r0n,
            });
            let mut error = AttTrackingError::new(AttTrackingErrorConfig {
                name: "attError".to_string(),
                sigma_r0r: nalgebra::Vector3::zeros(),
            });
            reference
                .att_ref_out_msg
                .connect_to(&mut error.att_ref_in_msg);
            nav_converter
                .attitude_out_msg
                .connect_to(&mut error.att_nav_in_msg);
            let guidance = error.att_guid_out_msg.clone();
            inertial_3d = Some(InTasks::new(reference, &[outer]));
            att_error = Some(InTasks::new(error, &[outer]));
            guidance
        };

        // Upstream `SetMrpFeedback`.
        let mut mrp_feedback = MrpFeedback::new(MrpFeedbackConfig {
            name: "mrpFeedback".to_string(),
            k: config.k_rot,
            ki: config.ki_rot,
            p: config.p_rot,
            integral_limit: 0.0,
            known_torque_body_nm: nalgebra::Vector3::zeros(),
            control_law_type: 0,
        });
        guidance.connect_to(&mut mrp_feedback.guid_in_msg);
        vc_msg.connect_to(&mut mrp_feedback.veh_config_in_msg);

        // Upstream `SetInertialCartFeedback`.
        let mut cart_feedback = InertialCartFeedback::new(InertialCartFeedbackConfig {
            name: "inertialCartFeedback".to_string(),
            k: config.k_trans,
            p: config.p_trans,
            ..InertialCartFeedbackConfig::default()
        });
        nav_converter
            .translation_out_msg
            .connect_to(&mut cart_feedback.deputy_nav_in_msg);
        match &trans_ref {
            Some(reference) => reference
                .module
                .trans_ref_out_msg
                .connect_to(&mut cart_feedback.deputy_ref_in_msg),
            None => trans_ref_msg.connect_to(&mut cart_feedback.deputy_ref_in_msg),
        }
        vc_msg.connect_to(&mut cart_feedback.deputy_vehicle_config_in_msg);

        // Upstream `SetJointThrAllocation`.
        let mut allocation_config = JointThrAllocationConfig {
            name: "jointThrAllocation".to_string(),
            wf: PerThruster::All(config.thrust_weight),
            thr_force_max_n: PerThruster::Each(config.thr_forces_n.clone()),
            ..JointThrAllocationConfig::default()
        };
        allocation_config.set_wc_diagonal(config.control_weight);
        if let Some(iterations) = config.allocation_max_iterations {
            allocation_config.max_iterations = iterations;
        }
        if let Some(ftol) = config.allocation_function_tolerance {
            allocation_config.ftol = ftol;
        }
        let mut allocation = JointThrAllocation::new(allocation_config);
        thr_arm_config_msg.connect_to(&mut allocation.arm_config_in_msg);
        hub_origin.connect_to(&mut allocation.hub_states_in_msg);
        cart_feedback
            .force_out_msg
            .connect_to(&mut allocation.trans_force_in_msg);
        mrp_feedback
            .cmd_torque_out_msg
            .connect_to(&mut allocation.rot_torque_in_msg);
        if config.use_theta_alloc_penalty {
            allocation.config.w_theta = config.theta_weight.map(MotionWeight::Scalar);
            for (state, _) in &joints {
                state.connect_to(allocation.add_hinged_joint());
            }
        }
        let mut des_angle_reader = Input::default();
        let mut des_angle_time_reader = Input::default();
        allocation
            .des_joint_angles_out_msg
            .connect_to(&mut des_angle_reader);
        allocation
            .des_joint_angles_time_out_msg
            .connect_to(&mut des_angle_time_reader);

        // Upstream `SetThrFiringRound`.
        let mut firing_round = ThrFiringRound::new(ThrFiringRoundConfig {
            name: "thrFiringRound".to_string(),
            control_period_s: outer_period_nanos as f64 * 1e-9,
            on_time_resolution_s: config.pulse_resolution_s,
            thr_min_fire_time_s: config.minimum_on_time_s.unwrap_or(0.0),
            thr_force_max_n: config.thr_forces_n.clone(),
        });
        allocation
            .thr_force_out_msg
            .connect_to(&mut firing_round.thr_force_in_msg);
        let mut on_time_readers = Vec::new();
        for _ in 0..n_thr {
            let mut reader = Input::default();
            firing_round
                .add_thruster_command_output()
                .connect_to(&mut reader);
            on_time_readers.push(reader);
        }

        // Upstream `SetJointProfiler`.
        let joint_profiler = config
            .joint_profile
            .filter(|_| config.use_joint_profiler)
            .map(|profile| {
                let mut profiler = JointArrayRefProfiler::new(JointArrayRefProfilerConfig {
                    name: "jointProfiler".to_string(),
                    num_hinged_joints: n_joint,
                    profile,
                    joint_angle_wrap_start_rad: config.joint_profile_wrap_start_rad,
                    use_shortest_path: config.joint_profile_use_shortest_path,
                });
                for (index, (state, rate)) in joints.iter().enumerate() {
                    state.connect_to(&mut profiler.joint_states_in_msgs[index]);
                    rate.connect_to(&mut profiler.joint_state_dots_in_msgs[index]);
                }
                allocation
                    .des_joint_angles_out_msg
                    .connect_to(&mut profiler.des_joint_states_in_msg);
                profiler
            });

        // Upstream `SetHingedJointArrayMotor`.
        let mut motor = HingedJointArrayMotor::new(HingedJointArrayMotorConfig {
            name: "hingedJointArrayMotor".to_string(),
            num_hinged_joints: n_joint,
            k_theta: config.k_theta.clone(),
            p_theta: config.p_theta.clone(),
            u_max_nm: Some(config.max_joint_torques_nm.clone()),
        });
        mass_matrix.connect_to(&mut motor.mass_matrix_in_msg);
        reaction_forces.connect_to(&mut motor.reaction_forces_in_msg);
        match &joint_profiler {
            Some(profiler) => profiler
                .des_joint_states_out_msg
                .connect_to(&mut motor.des_joint_states_in_msg),
            None => allocation
                .des_joint_angles_out_msg
                .connect_to(&mut motor.des_joint_states_in_msg),
        }
        let mut angle_readers = Vec::new();
        let mut angle_rate_readers = Vec::new();
        for (index, (state, rate)) in joints.iter().enumerate() {
            state.connect_to(&mut motor.joint_states_in_msgs[index]);
            rate.connect_to(&mut motor.joint_state_dots_in_msgs[index]);
            motor.add_motor_torque_output();
            motor.motor_torques_out_msgs[index] = gateways.joint_torque[index].clone();
            let (mut angle, mut angle_rate) = (Input::default(), Input::default());
            state.connect_to(&mut angle);
            rate.connect_to(&mut angle_rate);
            angle_readers.push(angle);
            angle_rate_readers.push(angle_rate);
        }

        // Upstream `SetJointMotionCompensator`.
        let joint_motion_compensator = config.use_joint_motion_compensation.then(|| {
            let mut compensator = JointMotionCompensator::new(JointMotionCompensatorConfig {
                name: "jointMotionCompensator".to_string(),
                num_spacecraft: 1,
                num_hinged_joints: n_joint,
                u_max_nm: config.max_pre_torques_nm.clone(),
            });
            mass_matrix.connect_to(&mut compensator.mass_matrix_in_msg);
            reaction_forces.connect_to(&mut compensator.reaction_forces_in_msg);
            for (index, torque) in gateways.joint_torque.iter().enumerate() {
                torque.connect_to(&mut compensator.joint_torque_in_msgs[index]);
            }
            for axis in 0..3 {
                compensator.add_hub_torque_output();
                compensator.hub_torque_out_msgs[axis] = gateways.body_torque[axis].clone();
            }
            InTasks::new(compensator, &[&tasks.joint_motion])
        });

        // Upstream `SetThrOnTimeToForce`.
        let mut on_time_to_force = ThrOnTimeToForce::new(ThrOnTimeToForceConfig {
            name: "thrOnTimeToForce".to_string(),
            thr_mag_n: config.thr_forces_n.clone(),
        });
        for index in 0..n_thr {
            gateways.thr_firing_time[index]
                .connect_to(&mut on_time_to_force.on_time_in_msgs[index]);
            on_time_to_force.thruster_force_out_msgs[index] = gateways.thr_force[index].clone();
        }

        // Upstream `SetThrJointCompensation`.
        let mut thr_compensation = ThrJointCompensation::new(ThrJointCompensationConfig {
            name: "thrJointCompensation".to_string(),
            num_hinged_joints: n_joint,
            num_thrusters: n_thr,
            u_max_nm: Some(config.max_joint_torques_nm.clone()),
        });
        thr_arm_config_msg.connect_to(&mut thr_compensation.arm_config_in_msg);
        mass_matrix.connect_to(&mut thr_compensation.mass_matrix_in_msg);
        reaction_forces.connect_to(&mut thr_compensation.reaction_forces_in_msg);
        for (index, force) in gateways.thr_force.iter().enumerate() {
            force.connect_to(&mut thr_compensation.thr_forces_in_msgs[index]);
        }
        for (index, (state, _)) in joints.iter().enumerate() {
            state.connect_to(&mut thr_compensation.joint_states_in_msgs[index]);
            thr_compensation.add_motor_torque_output();
            thr_compensation.motor_torques_out_msgs[index] = gateways.joint_torque[index].clone();
        }

        let thr_force_readers = gateways
            .thr_force
            .iter()
            .map(|force| {
                let mut reader = Input::default();
                force.connect_to(&mut reader);
                reader
            })
            .collect();
        let readers = Readers {
            des_angles: des_angle_reader.clone(),
            des_angles_time: des_angle_time_reader.clone(),
            on_times: on_time_readers,
            angles: angle_readers,
            angle_rates: angle_rate_readers,
            thr_forces: thr_force_readers,
        };

        // Upstream: disable all FSW tasks, zero the gateways, enable the
        // outer loop and the recorders, and create the events.
        tasks.disable_all();
        gateways.zero();
        tasks.outer_loop.enable();
        tasks.recorder.enable();
        add_events(
            events,
            inner_period_nanos,
            &config,
            &tasks,
            &gateways,
            readers,
        );

        let joint_motion = &tasks.joint_motion;
        let thr_firing = &tasks.thr_firing;
        Self {
            state_merge: InTasks::new(state_merge, &[outer]),
            nav_converter: InTasks::new(nav_converter, &[outer]),
            mrp_feedback: InTasks::new(mrp_feedback, &[outer]),
            inertial_cart_feedback: InTasks::new(cart_feedback, &[outer]),
            joint_thr_allocation: InTasks::new(allocation, &[outer]),
            thr_firing_round: InTasks::new(firing_round, &[outer]),
            joint_profiler: joint_profiler.map(|profiler| InTasks::new(profiler, &[joint_motion])),
            hinged_joint_array_motor: InTasks::new(motor, &[joint_motion]),
            joint_motion_compensator,
            thr_on_time_to_force: InTasks::new(on_time_to_force, &[thr_firing]),
            thr_joint_compensation: InTasks::new(thr_compensation, &[thr_firing]),
            inertial_3d,
            att_error,
            trans_ref,
            att_ref,
            tasks: tasks.clone(),
            gateways,
            vc_msg,
            thr_arm_config_msg,
            trans_ref_msg,
            sc_target_msg,
            des_angle_reader,
            des_angle_time_reader,
            config,
            outer_period_nanos,
            inner_period_nanos,
        }
    }

    /// Upstream `outerLoopTimeStep` \[ns\].
    pub fn outer_period_nanos(&self) -> u64 {
        self.outer_period_nanos
    }

    /// Upstream `innerLoopTimeStep` \[ns\].
    pub fn inner_period_nanos(&self) -> u64 {
        self.inner_period_nanos
    }

    /// Upstream's `AddModelToTask` calls of the FSW tasks.
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let (outer, inner) = (self.outer_period_nanos, self.inner_period_nanos);
        let tracking = self.config.control_objective == ControlObjective::Tracking;
        let o = OUTER_LOOP_PRIORITY;
        if tracking {
            simulation.add_module("stateMerge", &mut self.state_merge, outer, o + 50);
            simulation.add_module("navConverter", &mut self.nav_converter, outer, o + 49);
            if let Some(att_ref) = self.att_ref.as_mut() {
                simulation.add_module("attRef", att_ref, outer, o + 48);
            }
            simulation.add_module("mrpFeedback", &mut self.mrp_feedback, outer, o + 47);
            if let Some(trans_ref) = self.trans_ref.as_mut() {
                simulation.add_module("arcLineHoldRef", trans_ref, outer, o + 46);
            }
        } else {
            if let Some(inertial_3d) = self.inertial_3d.as_mut() {
                simulation.add_module("inertial3D", inertial_3d, outer, o + 50);
            }
            simulation.add_module("stateMerge", &mut self.state_merge, outer, o + 49);
            simulation.add_module("navConverter", &mut self.nav_converter, outer, o + 48);
            if let Some(att_error) = self.att_error.as_mut() {
                simulation.add_module("attError", att_error, outer, o + 47);
            }
            simulation.add_module("mrpFeedback", &mut self.mrp_feedback, outer, o + 46);
        }
        simulation.add_module(
            "inertialCartFeedback",
            &mut self.inertial_cart_feedback,
            outer,
            o + 45,
        );
        simulation.add_module(
            "jointThrAllocation",
            &mut self.joint_thr_allocation,
            outer,
            o + 44,
        );
        simulation.add_module("thrFiringRound", &mut self.thr_firing_round, outer, o + 43);

        let j = JOINT_MOTION_PRIORITY;
        if let Some(profiler) = self.joint_profiler.as_mut() {
            simulation.add_module("jointProfiler", profiler, inner, j + 51);
        }
        simulation.add_module(
            "hingedJointArrayMotor",
            &mut self.hinged_joint_array_motor,
            inner,
            j + 50,
        );
        if let Some(compensator) = self.joint_motion_compensator.as_mut() {
            simulation.add_module("jointMotionCompensator", compensator, inner, j + 49);
        }

        let t = THR_FIRING_PRIORITY;
        simulation.add_module(
            "thrOnTimeToForce",
            &mut self.thr_on_time_to_force,
            inner,
            t + 50,
        );
        simulation.add_module(
            "thrJointCompensation",
            &mut self.thr_joint_compensation,
            inner,
            t + 49,
        );
    }
}

/// Upstream `wrapAngle` of the event condition.
fn largest_wrapped_error(desired: &[f64], current: &[f64]) -> f64 {
    desired
        .iter()
        .zip(current)
        .map(|(want, have)| wrap_angle(want - have).abs())
        .fold(0.0, f64::max)
}

/// Upstream `createNewEvent("jointMotion" | "thrFiring" | "coast", ...)`.
fn add_events(
    events: &mut Events,
    period_nanos: u64,
    config: &ThrArmConfig,
    tasks: &FswTasks,
    gateways: &Gateways,
    readers: Readers,
) {
    let num_joints = config.num_joints;
    let state = Arc::new(Mutex::new(EventState {
        last_des_angles: vec![0.0; num_joints],
        last_des_angle_rates: vec![0.0; num_joints],
        last_des_cmd_write_nanos: None,
        pending_on_times_s: vec![0.0; config.num_thrusters],
        pending_firing: false,
        firing_cancellation_pending: false,
        joint_motion_phase_active: false,
        thrust_just_activated: false,
    }));
    // Every action starts with: disable all, zero the gateways, enable the
    // outer loop and the recorders.
    let restart = {
        let (tasks, gateways) = (tasks.clone(), gateways.clone());
        move || {
            tasks.disable_all();
            gateways.zero();
            tasks.outer_loop.enable();
            tasks.recorder.enable();
        }
    };

    // jointMotion: a new allocation was published.
    let condition = {
        let (state, readers) = (Arc::clone(&state), readers.clone());
        move |_now: u64| {
            let desired = readers.des_angles.read();
            let write_time = readers.des_angles_time.read().time_written_nanos;
            if desired.states.len() != num_joints || desired.state_dots.len() != num_joints {
                return false;
            }
            let mut state = state
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            if state.last_des_cmd_write_nanos == Some(write_time) {
                return false;
            }
            state.last_des_angles = desired.states;
            state.last_des_angle_rates = desired.state_dots;
            state.last_des_cmd_write_nanos = Some(write_time);
            state.pending_on_times_s = readers
                .on_times
                .iter()
                .map(|input| input.read().on_time_s)
                .collect();
            state.pending_firing = state
                .pending_on_times_s
                .iter()
                .any(|on_time| *on_time > 0.0);
            true
        }
    };
    let action = {
        let (state, tasks, gateways, restart) = (
            Arc::clone(&state),
            tasks.clone(),
            gateways.clone(),
            restart.clone(),
        );
        move |now: u64, activity: &mut EventActivity| {
            restart();
            let mut state = state
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            if !state.pending_firing {
                if state.joint_motion_phase_active {
                    tasks.joint_motion.enable();
                    activity.set("jointMotion", true);
                    activity.set("thrFiring", true);
                    activity.set("coast", false);
                    return;
                }
                state.joint_motion_phase_active = false;
                // Upstream `startFiringCancellation`: latch a zero on-time.
                gateways.write_on_times(&vec![0.0; gateways.thr_firing_time.len()], now);
                state.firing_cancellation_pending = true;
                tasks.thr_firing.enable();
                activity.set("jointMotion", true);
                activity.set("thrFiring", false);
                activity.set("coast", true);
                return;
            }
            state.joint_motion_phase_active = true;
            tasks.joint_motion.enable();
            activity.set("jointMotion", true);
            activity.set("thrFiring", true);
            activity.set("coast", false);
        }
    };
    events.add(
        "jointMotion",
        Check::Every(period_nanos),
        true,
        condition,
        action,
    );

    // thrFiring: the joints reached their commanded states.
    let (position_tolerance, rate_tolerance) = (
        config.joint_position_tolerance_rad,
        config.joint_rate_tolerance_radps,
    );
    let condition = {
        let (state, readers) = (Arc::clone(&state), readers.clone());
        move |_now: u64| {
            let state = state
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            let angles: Vec<f64> = readers
                .angles
                .iter()
                .map(|input| input.read().state)
                .collect();
            let rates: Vec<f64> = readers
                .angle_rates
                .iter()
                .map(|input| input.read().state)
                .collect();
            let angle_error = largest_wrapped_error(&state.last_des_angles, &angles);
            let rate_error = state
                .last_des_angle_rates
                .iter()
                .zip(&rates)
                .map(|(want, have)| (want - have).abs())
                .fold(0.0, f64::max);
            angle_error < position_tolerance && rate_error < rate_tolerance
        }
    };
    let action = {
        let (state, tasks, gateways, restart) = (
            Arc::clone(&state),
            tasks.clone(),
            gateways.clone(),
            restart.clone(),
        );
        move |now: u64, activity: &mut EventActivity| {
            let mut state = state
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            state.thrust_just_activated = true;
            state.joint_motion_phase_active = false;
            restart();
            tasks.thr_firing.enable();
            gateways.write_on_times(&state.pending_on_times_s, now);
            activity.set("jointMotion", true);
            activity.set("thrFiring", false);
            activity.set("coast", true);
        }
    };
    events.add(
        "thrFiring",
        Check::Every(period_nanos),
        false,
        condition,
        action,
    );

    // coast: the thrusters finished firing.
    let zero_tolerance = config.thruster_zero_tolerance_n;
    let condition = {
        let (state, readers) = (Arc::clone(&state), readers);
        move |_now: u64| {
            let largest = readers
                .thr_forces
                .iter()
                .map(|input| input.read().input.abs())
                .fold(0.0, f64::max);
            let mut state = state
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            if state.firing_cancellation_pending {
                state.firing_cancellation_pending = false;
                return false;
            }
            if state.thrust_just_activated {
                state.thrust_just_activated = false;
                return false;
            }
            largest < zero_tolerance
        }
    };
    let action = {
        let state = Arc::clone(&state);
        move |_now: u64, activity: &mut EventActivity| {
            state
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner())
                .joint_motion_phase_active = false;
            restart();
            activity.set("jointMotion", true);
            activity.set("thrFiring", false);
            activity.set("coast", false);
        }
    };
    events.add(
        "coast",
        Check::Every(period_nanos),
        false,
        condition,
        action,
    );
}
