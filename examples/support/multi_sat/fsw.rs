//! Upstream `modelsMultiSat/BSK_MultiSatFsw.py`: the flight-software process
//! of one MultiSat spacecraft, its tasks, gateway messages and mode events.
//!
//! | upstream task (priority) | modules (priority) | here |
//! |---|---|---|
//! | `inertialPointTask<i>` (20) | `inertial3D` (10), `sigma_R0N = [0.1, 0.2, -0.3]` | [`FswTasks::inertial_point`] |
//! | `sunPointTask<i>` (20) | `sunPoint` (`locationPointing` at the Sun, `pHat_B` = panel axis) (10) | [`FswTasks::sun_point`] |
//! | `locPointTask<i>` (20) | `locPoint` (`locationPointing` at the ground station, `pHat_B = [1, 0, 0]`) (10) | [`FswTasks::loc_point`] |
//! | `spacecraftReconfigTask<i>` (15) | `spacecraftReconfig` (10) | [`FswTasks::spacecraft_reconfig`] |
//! | `trackingErrorTask<i>` (10) | `attTrackingError` (9) | [`FswTasks::tracking_error`] |
//! | `mrpFeedbackRWsTask<i>` (5) | `mrpFeedback` (7), `rwMotorTorque` (6) | [`FswTasks::mrp_feedback_rws`] |
//!
//! # Gateway messages
//!
//! Upstream's pointing modules and `spacecraftReconfig` all write the
//! stand-alone `attRefMsg` (`AttRefMsg_C_addAuthor`), which
//! `spacecraftReconfig` and `attTrackingError` read; `attTrackingError`
//! writes `attGuidMsg`. Here they are the [`Gateway`] [`Output`]s, handed to
//! every authoring module as its output port (an `Output` clone is the same
//! message), and zeroed with the actuator commands by the mode events, as
//! upstream's `zeroGateWayMsgs`.
//!
//! # Modes
//!
//! [`FswMode`] is upstream's `modeRequest` and [`StationKeeping`] its
//! `stationKeeping`, both [`Request`]s a scenario sets between runs.
//! [`Fsw::new`] creates upstream's six events: each mode event, when its mode
//! is requested, disables every task, zeroes the gateway, enables the mode's
//! tasks and re-activates the spacecraft's other events; station keeping
//! switches `spacecraftReconfigTask` on or off. All tasks start disabled.
//!
//! `spacecraftReconfig` subscribes to a blank chief message until the
//! scenario connects a chief; `mrpFeedback` runs without integral feedback
//! (`Ki = -1`), with `P = 2 max(I) / 50 s` and `K = (P / 0.9)² / max(I)`.

use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::fsw_algorithms::att_guidance::location_pointing::{
    LocationPointing, LocationPointingConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::rw_motor_torque::{
    RwMotorTorque, RwMotorTorqueConfig,
};
use basilisk_rs::fsw_algorithms::formation_flying::spacecraft_reconfig::{
    SpacecraftReconfig, SpacecraftReconfigConfig,
};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, AttitudeReferenceMsg, NavigationTranslationMsg,
    Output, RwArrayConfigMsg, ThrusterArrayConfigMsg, ThrusterCommandMsg,
};
use basilisk_rs::simulation::Simulation;
use nalgebra::Vector3;

use super::dynamics::Dynamics;
use super::environment::Environment;
use super::events::{Events, Request};
use super::task_switch::{Switched, TaskSwitch};

/// Upstream `modeRequest`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FswMode {
    /// `"standby"`: no attitude control (the default).
    Standby,
    /// `"inertialPointing"`.
    InertialPointing,
    /// `"sunPointing"`: the solar panel axis at the Sun.
    SunPointing,
    /// `"locationPointing"`: body x at the ground station.
    LocationPointing,
}

/// Upstream `stationKeeping`: `"OFF"` (the default) or `"ON"`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StationKeeping {
    Off,
    On,
}

/// The tasks of one spacecraft's FSW process.
#[derive(Clone, Debug)]
pub struct FswTasks {
    pub inertial_point: TaskSwitch,
    pub sun_point: TaskSwitch,
    pub loc_point: TaskSwitch,
    pub spacecraft_reconfig: TaskSwitch,
    pub tracking_error: TaskSwitch,
    pub mrp_feedback_rws: TaskSwitch,
}

impl FswTasks {
    /// Every task disabled (upstream `fswProc.disableAllTasks()`).
    fn disabled() -> Self {
        Self {
            inertial_point: TaskSwitch::new(false),
            sun_point: TaskSwitch::new(false),
            loc_point: TaskSwitch::new(false),
            spacecraft_reconfig: TaskSwitch::new(false),
            tracking_error: TaskSwitch::new(false),
            mrp_feedback_rws: TaskSwitch::new(false),
        }
    }

    /// Upstream `fswProc[i].disableAllTasks()`.
    pub fn disable_all(&self) {
        for task in [
            &self.inertial_point,
            &self.sun_point,
            &self.loc_point,
            &self.spacecraft_reconfig,
            &self.tracking_error,
            &self.mrp_feedback_rws,
        ] {
            task.disable();
        }
    }
}

/// Upstream's gateway messages and the actuator commands
/// `zeroGateWayMsgs` zeroes.
#[derive(Clone, Debug)]
pub struct Gateway {
    /// Upstream `attRefMsg`.
    pub att_ref: Output<AttitudeReferenceMsg>,
    /// Upstream `attGuidMsg`.
    pub att_guid: Output<AttitudeGuidanceMsg>,
    /// Upstream `rwMotorTorque.rwMotorTorqueOutMsg`.
    pub rw_motor_torque: Output<ArrayMotorTorqueMsg>,
    /// Upstream `spacecraftReconfig.onTimeOutMsg`, per thruster.
    pub thruster_on_time: Vec<Output<ThrusterCommandMsg>>,
}

impl Gateway {
    /// Upstream `zeroGateWayMsgs`.
    pub fn zero(&self) {
        self.att_ref.write(AttitudeReferenceMsg::default());
        self.att_guid.write(AttitudeGuidanceMsg::default());
        self.rw_motor_torque.write(ArrayMotorTorqueMsg::default());
        for command in &self.thruster_on_time {
            command.write(ThrusterCommandMsg::default());
        }
    }
}

/// The flight software of one spacecraft (upstream `BSKFswModels`).
pub struct Fsw {
    /// Upstream `spacecraftIndex`.
    pub index: usize,
    /// Upstream `processTasksTimeStep` \[ns\].
    pub task_period_nanos: u64,
    /// Upstream `modeRequest`.
    pub mode_request: Request<FswMode>,
    /// Upstream `stationKeeping`.
    pub station_keeping: Request<StationKeeping>,
    pub tasks: FswTasks,
    pub gateway: Gateway,
    /// Upstream's mode events of this spacecraft.
    pub events: Events,
    /// Upstream `decayTime` \[s\] and `xi` of the attitude control gains.
    pub decay_time_s: f64,
    pub xi: f64,
    pub inertial_3d_point: Switched<Inertial3D>,
    pub sun_point: Switched<LocationPointing>,
    pub loc_point: Switched<LocationPointing>,
    pub spacecraft_reconfig: Switched<SpacecraftReconfig>,
    pub tracking_error: Switched<AttTrackingError>,
    pub mrp_feedback_rws: Switched<MrpFeedback>,
    pub rw_motor_torque: Switched<RwMotorTorque>,
    /// Upstream `fswRwConfigMsg`.
    pub rw_config_msg: Output<RwArrayConfigMsg>,
    /// Upstream `fswThrusterConfigMsg`.
    pub thruster_config_msg: Output<ThrusterArrayConfigMsg>,
    /// Upstream's blank chief message of `spacecraftReconfig`.
    pub blank_chief_msg: Output<NavigationTranslationMsg>,
}

impl Fsw {
    /// Upstream `BSKFswModels(SimBase, fswRate, spacecraftIndex)`: builds and
    /// connects the modules, connects their commands to `dynamics`, and
    /// creates the mode events.
    pub fn new(environment: &Environment, dynamics: &mut Dynamics, task_period_nanos: u64) -> Self {
        let index = dynamics.index;
        let tasks = FswTasks::disabled();
        let att_ref = Output::<AttitudeReferenceMsg>::default();
        let att_guid = Output::<AttitudeGuidanceMsg>::default();
        let nav_att = dynamics.simple_nav.attitude_out_msg.clone();
        let nav_trans = dynamics.simple_nav.translation_out_msg.clone();
        let vehicle_config = dynamics.simple_mass_props.vehicle_config_out_msg.clone();
        let rw_config_msg = Output::new(dynamics.rw_config_message());
        let thruster_config_msg = Output::new(dynamics.thruster_config_message());

        // SetInertial3DPointGuidance
        let mut inertial_3d_point = Inertial3D::new(Inertial3DConfig {
            name: "inertial3D".to_string(),
            sigma_r0n: Vector3::new(0.1, 0.2, -0.3),
        });
        inertial_3d_point.att_ref_out_msg = att_ref.clone();

        // SetSunPointGuidance
        let mut sun_point = LocationPointing::new(LocationPointingConfig {
            name: "sunPoint".to_string(),
            p_hat_b: dynamics.solar_panel_axis,
            ..Default::default()
        });
        nav_att.connect_to(&mut sun_point.sc_att_in_msg);
        nav_trans.connect_to(&mut sun_point.sc_trans_in_msg);
        environment
            .sun_ephemeris_out_msg()
            .connect_to(&mut sun_point.cel_body_in_msg);
        sun_point.att_ref_out_msg = att_ref.clone();

        // SetLocationPointGuidance
        let mut loc_point = LocationPointing::new(LocationPointingConfig {
            name: "locPoint".to_string(),
            p_hat_b: Vector3::x(),
            ..Default::default()
        });
        nav_att.connect_to(&mut loc_point.sc_att_in_msg);
        nav_trans.connect_to(&mut loc_point.sc_trans_in_msg);
        environment
            .ground_state_out_msg()
            .connect_to(&mut loc_point.location_in_msg);
        loc_point.att_ref_out_msg = att_ref.clone();

        // SetAttitudeTrackingError
        let mut tracking_error = AttTrackingError::new(AttTrackingErrorConfig {
            name: "trackingError".to_string(),
            sigma_r0r: Vector3::zeros(),
        });
        nav_att.connect_to(&mut tracking_error.att_nav_in_msg);
        att_ref.connect_to(&mut tracking_error.att_ref_in_msg);
        tracking_error.att_guid_out_msg = att_guid.clone();

        // SetMRPFeedbackRWA
        let decay_time_s = 50.0;
        let xi = 0.9;
        let max_inertia = dynamics.inertia_kg_m2.max();
        let p = 2.0 * max_inertia / decay_time_s;
        let ki = -1.0;
        let mut mrp_feedback_rws = MrpFeedback::new(MrpFeedbackConfig {
            name: "mrpFeedbackRWs".to_string(),
            k: (p / xi) * (p / xi) / max_inertia,
            ki,
            p,
            integral_limit: 2.0 / ki * 0.1,
            known_torque_body_nm: Vector3::zeros(),
            control_law_type: 0,
        });
        vehicle_config.connect_to(&mut mrp_feedback_rws.veh_config_in_msg);
        dynamics
            .rw_speed_out_msg
            .connect_to(&mut mrp_feedback_rws.rw_speeds_in_msg);
        rw_config_msg.connect_to(&mut mrp_feedback_rws.rw_params_in_msg);
        att_guid.connect_to(&mut mrp_feedback_rws.guid_in_msg);

        // SetSpacecraftOrbitReconfig
        let mut spacecraft_reconfig = SpacecraftReconfig::new(SpacecraftReconfigConfig {
            name: "spacecraftReconfig".to_string(),
            mu_m3ps2: environment.mu_m3ps2,
            att_control_time_s: 400.0,
            ..Default::default()
        });
        nav_trans.connect_to(&mut spacecraft_reconfig.deputy_trans_in_msg);
        att_ref.connect_to(&mut spacecraft_reconfig.att_ref_in_msg);
        thruster_config_msg.connect_to(&mut spacecraft_reconfig.thrust_config_in_msg);
        vehicle_config.connect_to(&mut spacecraft_reconfig.vehicle_config_in_msg);
        spacecraft_reconfig.att_ref_out_msg = att_ref.clone();
        let blank_chief_msg = Output::new(NavigationTranslationMsg::default());
        blank_chief_msg.connect_to(&mut spacecraft_reconfig.chief_trans_in_msg);

        // SetRWMotorTorque
        let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
            name: "rwMotorTorque".to_string(),
            control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
        });
        mrp_feedback_rws
            .cmd_torque_out_msg
            .connect_to(&mut rw_motor_torque.veh_control_in_msg);
        rw_config_msg.connect_to(&mut rw_motor_torque.rw_params_in_msg);

        // setupGatewayMsgs: the actuator commands to the dynamics.
        rw_motor_torque
            .rw_motor_torque_out_msg
            .connect_to(dynamics.rw_motor_cmd_in_msg());
        let thruster_on_time: Vec<Output<ThrusterCommandMsg>> = dynamics
            .thruster_cmd_in_msgs()
            .map(|input| {
                let command = spacecraft_reconfig.add_thruster_command_output().clone();
                command.connect_to(input);
                command
            })
            .collect();
        let gateway = Gateway {
            att_ref,
            att_guid,
            rw_motor_torque: rw_motor_torque.rw_motor_torque_out_msg.clone(),
            thruster_on_time,
        };

        let mode_request = Request::new(FswMode::Standby);
        let station_keeping = Request::new(StationKeeping::Off);
        let events = mode_events(index, &tasks, &gateway, &mode_request, &station_keeping);

        Self {
            index,
            task_period_nanos,
            mode_request,
            station_keeping,
            decay_time_s,
            xi,
            inertial_3d_point: Switched::new(inertial_3d_point, &tasks.inertial_point),
            sun_point: Switched::new(sun_point, &tasks.sun_point),
            loc_point: Switched::new(loc_point, &tasks.loc_point),
            spacecraft_reconfig: Switched::new(spacecraft_reconfig, &tasks.spacecraft_reconfig),
            tracking_error: Switched::new(tracking_error, &tasks.tracking_error),
            mrp_feedback_rws: Switched::new(mrp_feedback_rws, &tasks.mrp_feedback_rws),
            rw_motor_torque: Switched::new(rw_motor_torque, &tasks.mrp_feedback_rws),
            tasks,
            gateway,
            events,
            rw_config_msg,
            thruster_config_msg,
            blank_chief_msg,
        }
    }

    /// Upstream `spacecraftReconfig.chiefTransInMsg.subscribeTo(chief)`.
    pub fn set_chief(&mut self, chief: &Output<NavigationTranslationMsg>) {
        chief.connect_to(&mut self.spacecraft_reconfig.module.chief_trans_in_msg);
    }

    /// Upstream `spacecraftReconfig.targetClassicOED`.
    pub fn set_target_classic_oed(&mut self, target: [f64; 6]) {
        self.spacecraft_reconfig.module.config.target_classic_oed = target.into();
    }

    /// Adds the mode events to the events process and the modules to the
    /// spacecraft's FSW process, by task and module priority.
    pub fn schedule<'a>(
        &'a mut self,
        events: &mut Simulation<'a>,
        simulation: &mut Simulation<'a>,
    ) {
        let period = self.task_period_nanos;
        events.add_module(
            format!("events_{}", self.index),
            &mut self.events,
            period,
            0,
        );
        // Upstream task priority * 100 + module priority.
        simulation.add_module("inertial3D", &mut self.inertial_3d_point, period, 2010);
        simulation.add_module("sunPoint", &mut self.sun_point, period, 2010);
        simulation.add_module("locPoint", &mut self.loc_point, period, 2010);
        simulation.add_module(
            "spacecraftReconfig",
            &mut self.spacecraft_reconfig,
            period,
            1510,
        );
        simulation.add_module("trackingError", &mut self.tracking_error, period, 1009);
        simulation.add_module("mrpFeedbackRWs", &mut self.mrp_feedback_rws, period, 507);
        simulation.add_module("rwMotorTorque", &mut self.rw_motor_torque, period, 506);
    }
}

/// Upstream's `createNewEvent` calls of `BSKFswModels.__init__`.
fn mode_events(
    index: usize,
    tasks: &FswTasks,
    gateway: &Gateway,
    mode_request: &Request<FswMode>,
    station_keeping: &Request<StationKeeping>,
) -> Events {
    let mut events = Events::default();
    let modes: [(&str, FswMode, Vec<&TaskSwitch>); 4] = [
        ("initiateStandby", FswMode::Standby, vec![]),
        (
            "initiateInertialPointing",
            FswMode::InertialPointing,
            vec![
                &tasks.inertial_point,
                &tasks.tracking_error,
                &tasks.mrp_feedback_rws,
            ],
        ),
        (
            "initiateSunPointing",
            FswMode::SunPointing,
            vec![
                &tasks.sun_point,
                &tasks.tracking_error,
                &tasks.mrp_feedback_rws,
            ],
        ),
        (
            "initiateLocationPointing",
            FswMode::LocationPointing,
            vec![
                &tasks.loc_point,
                &tasks.tracking_error,
                &tasks.mrp_feedback_rws,
            ],
        ),
    ];
    for (event, mode, enabled) in modes {
        let name = format!("{event}_{index}");
        let request = mode_request.clone();
        let all = tasks.clone();
        let enabled: Vec<TaskSwitch> = enabled.into_iter().cloned().collect();
        let gateway = gateway.clone();
        let current = name.clone();
        events.add(
            name,
            true,
            move || request.is(mode),
            move |activity| {
                all.disable_all();
                gateway.zero();
                for task in &enabled {
                    task.enable();
                }
                activity.set_all_but(&current, true);
            },
        );
    }

    let start = format!("initiateStationKeeping_{index}");
    let stop = format!("stopStationKeeping_{index}");
    let (request, task, other) = (
        station_keeping.clone(),
        tasks.spacecraft_reconfig.clone(),
        stop.clone(),
    );
    events.add(
        start.clone(),
        true,
        move || request.is(StationKeeping::On),
        move |activity| {
            task.enable();
            activity.set(&other, true);
        },
    );
    let (request, task) = (station_keeping.clone(), tasks.spacecraft_reconfig.clone());
    events.add(
        stop,
        true,
        move || request.is(StationKeeping::Off),
        move |activity| {
            task.disable();
            activity.set(&start, true);
        },
    );
    events
}
