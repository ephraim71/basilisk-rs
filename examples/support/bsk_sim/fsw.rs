//! Upstream `BskSim/models/BSK_Fsw.py`: the flight software of the
//! single-spacecraft BskSim scenarios, its tasks, gateway messages and mode
//! events.
//!
//! | upstream task (priority) | modules (priority) | here |
//! |---|---|---|
//! | `inertial3DPointTask` (20) | `inertial3D` (10), `trackingError` (9) | [`FswTask::Inertial3DPoint`] |
//! | `hillPointTask` (20) | `hillPoint` (10), `trackingError` (9) | [`FswTask::HillPoint`] |
//! | `sunSafePointTask` (20) | `cssWlsEst` (10), `sunSafePoint` (9) | [`FswTask::SunSafePoint`] |
//! | `velocityPointTask` (20) | `velocityPoint` (10), `trackingError` (9) | [`FswTask::VelocityPoint`] |
//! | `lambertGuidanceFirstDV` (20) | `LambertPlanner`, `LambertSolver`, `LambertValidator` | [`FswTask::LambertGuidanceFirstDv`] |
//! | `lambertGuidanceSecondDV` (20) | `LambertSurfaceRelativeVelocity`, `LambertSecondDV` | [`FswTask::LambertGuidanceSecondDv`] |
//! | `mrpFeedbackTask` (10) | `mrpFeedbackControl` (10) | [`FswTask::MrpFeedback`] |
//! | `mrpSteeringRWsTask` (10) | `MRP_Steering` (10), `rate_servo` (9), `rwMotorTorque` (8) | [`FswTask::MrpSteeringRws`] |
//! | `mrpFeedbackRWsTask` (10) | `mrpFeedbackRWs` (9), `rwMotorTorque` (8) | [`FswTask::MrpFeedbackRws`] |
//!
//! Upstream runs the tasks by priority, equal priorities in creation order,
//! and a module listed in several tasks runs in each enabled one. Here each
//! module is scheduled once, in that order, in an [`InTasks`] of the tasks
//! listing it: `trackingError` after `velocityPoint`, `rwMotorTorque` after
//! `mrpFeedbackRWs`. No mode enables two tasks of one module, and only
//! disabled modules lie between a module's upstream positions, so the order
//! of what runs is upstream's in every mode.
//!
//! # Gateway messages
//!
//! Upstream's modes write the same stand-alone C messages from different
//! modules (`AttRefMsg_C_addAuthor`, ...): `attRefMsg`, `attGuidMsg`,
//! `cmdTorqueMsg`, `cmdTorqueDirectMsg` and `cmdRwMotorMsg`. Here they are the
//! [`FswGateway`] [`Output`]s, handed to every authoring module as its output
//! port (an `Output` clone is the same message), and zeroed by the mode
//! events as `zeroGateWayMsgs` does. Upstream assigns its `dvBurnCmdMsg`
//! *copy* to the two Lambert modules' outputs, which leaves each writing its
//! own message; here too.
//!
//! # Modes
//!
//! [`FswMode`] is upstream's `modeRequest` string, and [`BskFsw::new`] adds
//! upstream's `initiate*` events, checked at the FSW rate: when the request
//! names its mode, the event disables every task, zeroes the gateway, enables
//! the mode's tasks and re-activates every other event. Upstream's
//! `initiateStandby` action is a tuple of *strings* (not calls), so it does
//! nothing: requesting `standby` leaves the current tasks running. That is
//! kept, see [`FswMode::Standby`].
//!
//! # Differences from upstream
//!
//! * `cssWlsEst` reads the eight per-device CSS messages (basilisk-rs's CSS
//!   ports) with the normals of `cssConfigMsg`; its `CBias = 1` is the
//!   identity.
//! * `mrpFeedbackRWs` and `rwMotorTorque` are [`Shared`], so faults can change
//!   their parameters mid-run as upstream's change `FSWModels` attributes.

#![allow(dead_code)]

use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_control::mrp_steering::{MrpSteering, MrpSteeringConfig};
use basilisk_rs::fsw_algorithms::att_control::rate_servo_full_nonlinear::{
    RateServoFullNonlinear, RateServoFullNonlinearConfig,
};
use basilisk_rs::fsw_algorithms::att_determination::css_wls_est::{CssWlsEst, CssWlsEstConfig};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::hill_point::{HillPoint, HillPointConfig};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::fsw_algorithms::att_guidance::sun_safe_point::{SunSafePoint, SunSafePointConfig};
use basilisk_rs::fsw_algorithms::att_guidance::velocity_point::{
    VelocityPoint, VelocityPointConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::rw_motor_torque::{
    RwMotorTorque, RwMotorTorqueConfig,
};
use basilisk_rs::fsw_algorithms::orbit_control::lambert_planner::{
    LambertPlanner, LambertPlannerConfig,
};
use basilisk_rs::fsw_algorithms::orbit_control::lambert_second_dv::{
    LambertSecondDv, LambertSecondDvConfig,
};
use basilisk_rs::fsw_algorithms::orbit_control::lambert_solver::{
    LambertSolver, LambertSolverConfig,
};
use basilisk_rs::fsw_algorithms::orbit_control::lambert_surface_relative_velocity::{
    LambertSurfaceRelativeVelocity, LambertSurfaceRelativeVelocityConfig,
};
use basilisk_rs::fsw_algorithms::orbit_control::lambert_validator::{
    LambertValidator, LambertValidatorConfig,
};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, AttitudeReferenceMsg, BodyTorqueCommandMsg,
    DvBurnCommandMsg, Output, RwArrayConfigMsg, VehicleConfigMsg,
};
use basilisk_rs::overrides::TargetKind;
use basilisk_rs::simulation::Simulation;
use nalgebra::Vector3;

use super::BskSim;
use super::dynamics::{BskDynamics, CSS_NORMALS_B, fsw_rw_config, hub_inertia_kg_m2};
use super::events::{Check, InTasks, ModeRequest, TaskSwitch};
use super::shared::Shared;

/// The tasks of upstream's FSW process, in their run order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FswTask {
    Inertial3DPoint,
    HillPoint,
    SunSafePoint,
    VelocityPoint,
    LambertGuidanceFirstDv,
    LambertGuidanceSecondDv,
    MrpFeedback,
    MrpSteeringRws,
    MrpFeedbackRws,
}

impl FswTask {
    pub const ALL: [Self; 9] = [
        Self::Inertial3DPoint,
        Self::HillPoint,
        Self::SunSafePoint,
        Self::VelocityPoint,
        Self::LambertGuidanceFirstDv,
        Self::LambertGuidanceSecondDv,
        Self::MrpFeedback,
        Self::MrpSteeringRws,
        Self::MrpFeedbackRws,
    ];

    /// Upstream's task name.
    pub fn name(self) -> &'static str {
        match self {
            Self::Inertial3DPoint => "inertial3DPointTask",
            Self::HillPoint => "hillPointTask",
            Self::SunSafePoint => "sunSafePointTask",
            Self::VelocityPoint => "velocityPointTask",
            Self::LambertGuidanceFirstDv => "lambertGuidanceFirstDV",
            Self::LambertGuidanceSecondDv => "lambertGuidanceSecondDV",
            Self::MrpFeedback => "mrpFeedbackTask",
            Self::MrpSteeringRws => "mrpSteeringRWsTask",
            Self::MrpFeedbackRws => "mrpFeedbackRWsTask",
        }
    }
}

/// The FSW tasks' on/off switches; all start disabled, as after upstream's
/// `fswProc.disableAllTasks()`.
#[derive(Clone, Debug)]
pub struct FswTasks {
    switches: Vec<TaskSwitch>,
}

impl Default for FswTasks {
    fn default() -> Self {
        Self {
            switches: FswTask::ALL
                .iter()
                .map(|_| TaskSwitch::new(false))
                .collect(),
        }
    }
}

impl FswTasks {
    pub fn get(&self, task: FswTask) -> &TaskSwitch {
        let index = FswTask::ALL
            .iter()
            .position(|known| *known == task)
            .expect("known task");
        &self.switches[index]
    }

    /// Upstream `fswProc.disableAllTasks()`.
    pub fn disable_all(&self) {
        self.switches.iter().for_each(TaskSwitch::disable);
    }

    /// Upstream `enableTask(name)`.
    pub fn enable(&self, task: FswTask) {
        self.get(task).enable();
    }

    /// The enabled tasks.
    pub fn enabled(&self) -> Vec<FswTask> {
        FswTask::ALL
            .into_iter()
            .filter(|task| self.get(*task).is_enabled())
            .collect()
    }
}

/// Upstream `modeRequest` values of `BSK_Fsw`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FswMode {
    /// Upstream `"standby"`. Upstream's action does nothing (see the module
    /// docs): the tasks of the previous mode keep running.
    Standby,
    /// `"inertial3D"`: inertial pointing with the wheels.
    Inertial3D,
    /// `"directInertial3D"`: inertial pointing with the external torque.
    DirectInertial3D,
    /// `"hillPoint"`.
    HillPoint,
    /// `"sunSafePoint"`.
    SunSafePoint,
    /// `"velocityPoint"`.
    VelocityPoint,
    /// `"steeringRW"`: Hill pointing with the MRP steering law.
    SteeringRw,
    /// `"lambertFirstDV"`.
    LambertFirstDv,
    /// `"lambertSecondDV"`.
    LambertSecondDv,
}

impl FswMode {
    pub const ALL: [Self; 9] = [
        Self::Standby,
        Self::Inertial3D,
        Self::DirectInertial3D,
        Self::HillPoint,
        Self::SunSafePoint,
        Self::VelocityPoint,
        Self::SteeringRw,
        Self::LambertFirstDv,
        Self::LambertSecondDv,
    ];

    /// Upstream's event name.
    pub fn event_name(self) -> &'static str {
        match self {
            Self::Standby => "initiateStandby",
            Self::Inertial3D => "initiateAttitudeGuidance",
            Self::DirectInertial3D => "initiateAttitudeGuidanceDirect",
            Self::HillPoint => "initiateHillPoint",
            Self::SunSafePoint => "initiateSunSafePoint",
            Self::VelocityPoint => "initiateVelocityPoint",
            Self::SteeringRw => "initiateSteeringRW",
            Self::LambertFirstDv => "initiateLambertGuidanceFirstDV",
            Self::LambertSecondDv => "initiateLambertGuidanceSecondDV",
        }
    }

    /// The tasks the mode's event enables, or `None` for an event whose
    /// action does nothing.
    pub fn tasks(self) -> Option<&'static [FswTask]> {
        use FswTask::*;
        Some(match self {
            Self::Standby => return None,
            Self::Inertial3D => &[Inertial3DPoint, MrpFeedbackRws],
            Self::DirectInertial3D => &[Inertial3DPoint, MrpFeedback],
            Self::HillPoint => &[HillPoint, MrpFeedbackRws],
            Self::SunSafePoint => &[SunSafePoint, MrpSteeringRws],
            Self::VelocityPoint => &[VelocityPoint, MrpFeedbackRws],
            Self::SteeringRw => &[HillPoint, MrpSteeringRws],
            Self::LambertFirstDv => &[HillPoint, MrpSteeringRws, LambertGuidanceFirstDv],
            Self::LambertSecondDv => &[HillPoint, MrpSteeringRws, LambertGuidanceSecondDv],
        })
    }
}

/// Upstream `setupGatewayMsgs`: the messages several modules write.
#[derive(Clone, Debug, Default)]
pub struct FswGateway {
    pub cmd_torque: Output<BodyTorqueCommandMsg>,
    pub cmd_torque_direct: Output<BodyTorqueCommandMsg>,
    pub att_ref: Output<AttitudeReferenceMsg>,
    pub att_guid: Output<AttitudeGuidanceMsg>,
    pub cmd_rw_motor: Output<ArrayMotorTorqueMsg>,
    pub dv_burn_cmd: Output<DvBurnCommandMsg>,
}

impl FswGateway {
    /// Upstream `zeroGateWayMsgs`.
    pub fn zero(&self) {
        self.cmd_torque.write(BodyTorqueCommandMsg::default());
        self.cmd_torque_direct
            .write(BodyTorqueCommandMsg::default());
        self.att_ref.write(AttitudeReferenceMsg::default());
        self.att_guid.write(AttitudeGuidanceMsg::default());
        self.cmd_rw_motor.write(ArrayMotorTorqueMsg::default());
        self.dv_burn_cmd.write(DvBurnCommandMsg::default());
    }
}

/// Upstream `BSKFswModels`.
pub struct BskFsw {
    /// Upstream `processTasksTimeStep` \[ns\].
    pub task_period_nanos: u64,
    pub tasks: FswTasks,
    pub gateway: FswGateway,
    mode: ModeRequest<FswMode>,
    /// Upstream `vcMsg`.
    pub vehicle_config_msg: Output<VehicleConfigMsg>,
    /// Upstream `fswRwConfigMsg`.
    pub rw_config_msg: Output<RwArrayConfigMsg>,
    pub inertial_3d: InTasks<Inertial3D>,
    pub hill_point: InTasks<HillPoint>,
    pub css_wls_est: InTasks<CssWlsEst>,
    pub sun_safe_point: InTasks<SunSafePoint>,
    pub velocity_point: InTasks<VelocityPoint>,
    pub tracking_error: InTasks<AttTrackingError>,
    pub lambert_planner: InTasks<LambertPlanner>,
    pub lambert_solver: InTasks<LambertSolver>,
    pub lambert_validator: InTasks<LambertValidator>,
    pub lambert_surface_relative_velocity: InTasks<LambertSurfaceRelativeVelocity>,
    pub lambert_second_dv: InTasks<LambertSecondDv>,
    pub mrp_feedback_control: InTasks<MrpFeedback>,
    pub mrp_steering: InTasks<MrpSteering>,
    pub rate_servo: InTasks<RateServoFullNonlinear>,
    pub mrp_feedback_rws: InTasks<Shared<MrpFeedback>>,
    pub rw_motor_torque: InTasks<Shared<RwMotorTorque>>,
}

/// Upstream `mrpFeedback` gains `K = 3.5`, `Ki = -1` (off), `P = 30`,
/// `integralLimit = 2 / Ki * 0.1`.
fn mrp_feedback(name: &str, k: f64, ki: f64, p: f64) -> MrpFeedback {
    MrpFeedback::new(MrpFeedbackConfig {
        name: name.to_string(),
        k,
        ki,
        p,
        integral_limit: 2.0 / ki * 0.1,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 0,
    })
}

impl BskFsw {
    /// Upstream `BSKFswModels(SimBase, fswRate)`: builds the modules, wires
    /// them to `dynamics` and to the gateway, and adds the mode events.
    pub fn new(base: &mut BskSim, dynamics: &mut BskDynamics) -> Self {
        let tasks = FswTasks::default();
        let task = |task: FswTask| tasks.get(task).clone();
        let gateway = FswGateway::default();

        // `SetVehicleConfiguration`, `SetRWConfigMsg`.
        let vehicle_config_msg = Output::new(VehicleConfigMsg {
            inertia_about_point_b_kg_m2: hub_inertia_kg_m2(),
            ..VehicleConfigMsg::default()
        });
        let rw_config_msg = Output::new(fsw_rw_config());
        let nav_attitude = dynamics.simple_nav.attitude_out_msg.clone();
        let nav_translation = dynamics.simple_nav.translation_out_msg.clone();
        let earth_ephemeris = dynamics.earth_ephemeris.ephem_out_msgs[0].clone();

        // `SetInertial3DPointGuidance`.
        let mut inertial_3d = Inertial3D::new(Inertial3DConfig {
            name: "inertial3D".to_string(),
            sigma_r0n: Vector3::new(0.2, 0.4, 0.6),
        });
        inertial_3d.att_ref_out_msg = gateway.att_ref.clone();

        // `SetHillPointGuidance`.
        let mut hill_point = HillPoint::new(HillPointConfig::default());
        nav_translation.connect_to(&mut hill_point.trans_nav_in_msg);
        earth_ephemeris.connect_to(&mut hill_point.cel_body_in_msg);
        hill_point.att_ref_out_msg = gateway.att_ref.clone();

        // `SetCSSWlsEst`.
        let mut css_wls_est = CssWlsEst::new(CssWlsEstConfig {
            name: "cssWlsEst".to_string(),
            sensor_normals_body: CSS_NORMALS_B
                .iter()
                .map(|normal| Vector3::from(*normal))
                .collect(),
            sensor_use_thresh: 0.0,
            use_weights: false,
        });
        for (output, input) in dynamics
            .css_out_msgs()
            .iter()
            .zip(&mut css_wls_est.css_data_in_msgs)
        {
            output.connect_to(input);
        }

        // `SetSunSafePointGuidance`.
        let mut sun_safe_point = SunSafePoint::new(SunSafePointConfig {
            name: "sunSafePoint".to_string(),
            s_hat_bdy_cmd: Vector3::new(0.0, 0.0, 1.0),
            min_unit_mag: 0.0,
            small_angle: 0.0,
            sun_axis_spin_rate_radps: 0.0,
            omega_rn_b_search_radps: Vector3::zeros(),
        });
        nav_attitude.connect_to(&mut sun_safe_point.imu_in_msg);
        css_wls_est
            .nav_state_out_msg
            .connect_to(&mut sun_safe_point.sun_direction_in_msg);
        sun_safe_point.att_guidance_out_msg = gateway.att_guid.clone();

        // `SetVelocityPointGuidance`.
        let mut velocity_point = VelocityPoint::new(VelocityPointConfig {
            name: "velocityPoint".to_string(),
            mu_m3ps2: dynamics.mu_earth_m3ps2,
        });
        nav_translation.connect_to(&mut velocity_point.trans_nav_in_msg);
        earth_ephemeris.connect_to(&mut velocity_point.cel_body_in_msg);
        velocity_point.att_ref_out_msg = gateway.att_ref.clone();

        // `SetAttitudeTrackingError`.
        let mut tracking_error = AttTrackingError::new(AttTrackingErrorConfig {
            name: "trackingError".to_string(),
            sigma_r0r: Vector3::zeros(),
        });
        nav_attitude.connect_to(&mut tracking_error.att_nav_in_msg);
        gateway
            .att_ref
            .connect_to(&mut tracking_error.att_ref_in_msg);
        tracking_error.att_guid_out_msg = gateway.att_guid.clone();

        // `SetMRPFeedbackControl`.
        let mut mrp_feedback_control = mrp_feedback("mrpFeedbackControl", 3.5, -1.0, 30.0);
        gateway
            .att_guid
            .connect_to(&mut mrp_feedback_control.guid_in_msg);
        vehicle_config_msg.connect_to(&mut mrp_feedback_control.veh_config_in_msg);
        mrp_feedback_control.cmd_torque_out_msg = gateway.cmd_torque_direct.clone();

        // `SetMRPFeedbackRWA`.
        let mut mrp_feedback_rws = mrp_feedback("mrpFeedbackRWs", 3.5, -1.0, 30.0);
        vehicle_config_msg.connect_to(&mut mrp_feedback_rws.veh_config_in_msg);
        dynamics
            .rw_speed_out_msg
            .connect_to(&mut mrp_feedback_rws.rw_speeds_in_msg);
        rw_config_msg.connect_to(&mut mrp_feedback_rws.rw_params_in_msg);
        gateway
            .att_guid
            .connect_to(&mut mrp_feedback_rws.guid_in_msg);
        mrp_feedback_rws.cmd_torque_out_msg = gateway.cmd_torque.clone();

        // `SetMRPSteering`.
        let mut mrp_steering = MrpSteering::new(MrpSteeringConfig {
            name: "MRP_Steering".to_string(),
            k1: 0.05,
            k3: 0.75,
            omega_max_radps: 1.0_f64.to_radians(),
            ignore_outer_loop_feedforward: false,
        });
        gateway.att_guid.connect_to(&mut mrp_steering.guid_in_msg);

        // `SetRateServo`.
        let mut rate_servo = RateServoFullNonlinear::new(RateServoFullNonlinearConfig {
            name: "rate_servo".to_string(),
            p: 150.0,
            ki: 5.0,
            known_torque_body_nm: Vector3::zeros(),
            integral_limit: 2.0 / 5.0 * 0.1,
        });
        gateway.att_guid.connect_to(&mut rate_servo.guid_in_msg);
        vehicle_config_msg.connect_to(&mut rate_servo.veh_config_in_msg);
        rw_config_msg.connect_to(&mut rate_servo.rw_params_in_msg);
        dynamics
            .rw_speed_out_msg
            .connect_to(&mut rate_servo.rw_speeds_in_msg);
        mrp_steering
            .rate_cmd_out_msg
            .connect_to(&mut rate_servo.rate_steering_in_msg);
        rate_servo.cmd_torque_out_msg = gateway.cmd_torque.clone();

        // `SetRWMotorTorque`.
        let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
            name: "rwMotorTorque".to_string(),
            control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
        });
        gateway
            .cmd_torque
            .connect_to(&mut rw_motor_torque.veh_control_in_msg);
        rw_motor_torque.rw_motor_torque_out_msg = gateway.cmd_rw_motor.clone();
        rw_config_msg.connect_to(&mut rw_motor_torque.rw_params_in_msg);
        base.registry
            .register(
                "rwMotorTorque.rw_params_in_msg",
                &rw_motor_torque.rw_params_in_msg,
                TargetKind::Input,
            )
            .expect("unique target");

        // The Lambert modules; the scenario sets their parameters.
        let mut lambert_planner = LambertPlanner::new(LambertPlannerConfig {
            name: "LambertPlanner".to_string(),
            ..LambertPlannerConfig::default()
        });
        nav_translation.connect_to(&mut lambert_planner.nav_trans_in_msg);
        let mut lambert_solver = LambertSolver::new(LambertSolverConfig {
            name: "LambertSolver".to_string(),
        });
        lambert_planner
            .lambert_problem_out_msg
            .connect_to(&mut lambert_solver.lambert_problem_in_msg);
        let mut lambert_validator = LambertValidator::new(LambertValidatorConfig {
            name: "LambertValidator".to_string(),
            ..LambertValidatorConfig::default()
        });
        nav_translation.connect_to(&mut lambert_validator.nav_trans_in_msg);
        lambert_planner
            .lambert_problem_out_msg
            .connect_to(&mut lambert_validator.lambert_problem_in_msg);
        lambert_solver
            .lambert_performance_out_msg
            .connect_to(&mut lambert_validator.lambert_performance_in_msg);
        lambert_solver
            .lambert_solution_out_msg
            .connect_to(&mut lambert_validator.lambert_solution_in_msg);
        let mut lambert_surface_relative_velocity =
            LambertSurfaceRelativeVelocity::new(LambertSurfaceRelativeVelocityConfig {
                name: "LambertSurfaceRelativeVelocity".to_string(),
                ..LambertSurfaceRelativeVelocityConfig::default()
            });
        lambert_planner
            .lambert_problem_out_msg
            .connect_to(&mut lambert_surface_relative_velocity.lambert_problem_in_msg);
        earth_ephemeris.connect_to(&mut lambert_surface_relative_velocity.ephemeris_in_msg);
        let mut lambert_second_dv = LambertSecondDv::new(LambertSecondDvConfig {
            name: "LambertSecondDV".to_string(),
            ..LambertSecondDvConfig::default()
        });
        lambert_solver
            .lambert_solution_out_msg
            .connect_to(&mut lambert_second_dv.lambert_solution_in_msg);
        lambert_surface_relative_velocity
            .desired_velocity_out_msg
            .connect_to(&mut lambert_second_dv.desired_velocity_in_msg);

        // Upstream `setupGatewayMsgs`: the dynamics read the FSW commands.
        gateway
            .cmd_torque_direct
            .connect_to(&mut dynamics.ext_torque_cmd_in);
        gateway
            .cmd_rw_motor
            .connect_to(&mut dynamics.rw_commands.torque_in_msg);

        let fsw = Self {
            task_period_nanos: base.fsw_rate_nanos,
            gateway,
            mode: ModeRequest::new(FswMode::Standby),
            vehicle_config_msg,
            rw_config_msg,
            inertial_3d: InTasks::new(inertial_3d, &[&task(FswTask::Inertial3DPoint)]),
            hill_point: InTasks::new(hill_point, &[&task(FswTask::HillPoint)]),
            css_wls_est: InTasks::new(css_wls_est, &[&task(FswTask::SunSafePoint)]),
            sun_safe_point: InTasks::new(sun_safe_point, &[&task(FswTask::SunSafePoint)]),
            velocity_point: InTasks::new(velocity_point, &[&task(FswTask::VelocityPoint)]),
            tracking_error: InTasks::new(
                tracking_error,
                &[
                    &task(FswTask::Inertial3DPoint),
                    &task(FswTask::HillPoint),
                    &task(FswTask::VelocityPoint),
                ],
            ),
            lambert_planner: InTasks::new(
                lambert_planner,
                &[&task(FswTask::LambertGuidanceFirstDv)],
            ),
            lambert_solver: InTasks::new(lambert_solver, &[&task(FswTask::LambertGuidanceFirstDv)]),
            lambert_validator: InTasks::new(
                lambert_validator,
                &[&task(FswTask::LambertGuidanceFirstDv)],
            ),
            lambert_surface_relative_velocity: InTasks::new(
                lambert_surface_relative_velocity,
                &[&task(FswTask::LambertGuidanceSecondDv)],
            ),
            lambert_second_dv: InTasks::new(
                lambert_second_dv,
                &[&task(FswTask::LambertGuidanceSecondDv)],
            ),
            mrp_feedback_control: InTasks::new(
                mrp_feedback_control,
                &[&task(FswTask::MrpFeedback)],
            ),
            mrp_steering: InTasks::new(mrp_steering, &[&task(FswTask::MrpSteeringRws)]),
            rate_servo: InTasks::new(rate_servo, &[&task(FswTask::MrpSteeringRws)]),
            mrp_feedback_rws: InTasks::new(
                Shared::new(mrp_feedback_rws),
                &[&task(FswTask::MrpFeedbackRws)],
            ),
            rw_motor_torque: InTasks::new(
                Shared::new(rw_motor_torque),
                &[
                    &task(FswTask::MrpSteeringRws),
                    &task(FswTask::MrpFeedbackRws),
                ],
            ),
            tasks,
        };
        fsw.add_mode_events(base);
        fsw
    }

    /// Upstream's `initiate*` events.
    fn add_mode_events(&self, base: &mut BskSim) {
        for mode in FswMode::ALL {
            let request = self.mode.clone();
            let (tasks, gateway) = (self.tasks.clone(), self.gateway.clone());
            base.events.add(
                mode.event_name(),
                Check::Every(self.task_period_nanos),
                true,
                move |_| request.is(mode),
                move |_, activity| {
                    let Some(enabled) = mode.tasks() else {
                        return;
                    };
                    tasks.disable_all();
                    gateway.zero();
                    for task in enabled {
                        tasks.enable(*task);
                    }
                    activity.set_all_but(mode.event_name(), true);
                },
            );
        }
    }

    /// The handle of upstream's `modeRequest`; take it before scheduling.
    pub fn mode_request(&self) -> ModeRequest<FswMode> {
        self.mode.clone()
    }

    /// Upstream's FSW process, in task and module order.
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let period = self.task_period_nanos;
        basilisk_rs::schedule! { *simulation,
            "inertial3D" => &mut self.inertial_3d, period = period, priority = 3_000;
            "hillPoint" => &mut self.hill_point, period = period, priority = 2_990;
            "cssWlsEst" => &mut self.css_wls_est, period = period, priority = 2_980;
            "sunSafePoint" => &mut self.sun_safe_point, period = period, priority = 2_970;
            "velocityPoint" => &mut self.velocity_point, period = period, priority = 2_960;
            "trackingError" => &mut self.tracking_error, period = period, priority = 2_950;
            "LambertPlanner" => &mut self.lambert_planner, period = period, priority = 2_940;
            "LambertSolver" => &mut self.lambert_solver, period = period, priority = 2_930;
            "LambertValidator" => &mut self.lambert_validator, period = period, priority = 2_920;
            "LambertSurfaceRelativeVelocity" => &mut self.lambert_surface_relative_velocity, period = period, priority = 2_910;
            "LambertSecondDV" => &mut self.lambert_second_dv, period = period, priority = 2_900;
            "mrpFeedbackControl" => &mut self.mrp_feedback_control, period = period, priority = 2_800;
            "MRP_Steering" => &mut self.mrp_steering, period = period, priority = 2_790;
            "rate_servo" => &mut self.rate_servo, period = period, priority = 2_780;
            "mrpFeedbackRWs" => &mut self.mrp_feedback_rws, period = period, priority = 2_770;
            "rwMotorTorque" => &mut self.rw_motor_torque, period = period, priority = 2_760;
        }
    }
}
