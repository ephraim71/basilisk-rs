//! Upstream `BskSim/models/BSK_FormationFsw.py`: inertial pointing of both
//! spacecraft with their wheels, or the deputy pointing at the chief with an
//! external torque.
//!
//! | upstream task (priority) | modules (priority) | here |
//! |---|---|---|
//! | `inertial3DPointTask` (20) | `inertial3D` (10), `trackingError` (9) | [`FormationTask::Inertial3DPoint`] |
//! | `inertial3DPointTask2` (20) | `inertial3D2` (10), `trackingError2` (9) | [`FormationTask::Inertial3DPoint2`] |
//! | `mrpFeedbackRWsTask` (10) | `mrpFeedbackRWs` (9), `rwMotorTorque` (8) | [`FormationTask::MrpFeedbackRws`] |
//! | `mrpFeedbackRWsTask2` (10) | `mrpFeedbackRWs2` (9), `rwMotorTorque2` (8) | [`FormationTask::MrpFeedbackRws2`] |
//! | `mrpFeedbackTask` (10) | `mrpFeedbackControl` (10) | [`FormationTask::MrpFeedback`] |
//! | `spacecraftPointingTask` (-1) | `spacecraftPointing` (8), `trackingError2` (7) | [`FormationTask::SpacecraftPointing`] |
//!
//! `trackingError2` sits in two tasks that run at different places in the
//! tick (before the wheel control, and after `mrpFeedbackControl`, which
//! therefore reads the deputy's guidance of the previous tick). It has no
//! state, so it is two identically configured instances here, one per task,
//! both writing `attGuid2Msg`. The gateway messages are shared [`Output`]s,
//! as in [`super::fsw`].
//!
//! Upstream's formation mode events neither re-activate the others, unlike
//! `BSK_Fsw`'s: each fires once. Its standby action does disable the tasks
//! and zero the gateway.

#![allow(dead_code)]

use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::fsw_algorithms::effector_interfaces::rw_motor_torque::{
    RwMotorTorque, RwMotorTorqueConfig,
};
use basilisk_rs::fsw_algorithms::formation_flying::spacecraft_pointing::{
    SpacecraftPointing, SpacecraftPointingConfig,
};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, AttitudeReferenceMsg, BodyTorqueCommandMsg,
    NavigationAttitudeMsg, Output, RwArrayConfigMsg, RwSpeedMsg, VehicleConfigMsg,
};
use basilisk_rs::simulation::Simulation;
use nalgebra::Vector3;

use super::BskSim;
use super::dynamics::{fsw_rw_config, hub_inertia_kg_m2};
use super::events::{Check, InTasks, ModeRequest, TaskSwitch};
use super::formation_dynamics::FormationDynamics;

/// The tasks of upstream's formation FSW process, in their run order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FormationTask {
    Inertial3DPoint,
    Inertial3DPoint2,
    MrpFeedbackRws,
    MrpFeedbackRws2,
    MrpFeedback,
    SpacecraftPointing,
}

impl FormationTask {
    pub const ALL: [Self; 6] = [
        Self::Inertial3DPoint,
        Self::Inertial3DPoint2,
        Self::MrpFeedbackRws,
        Self::MrpFeedbackRws2,
        Self::MrpFeedback,
        Self::SpacecraftPointing,
    ];
}

/// Upstream `modeRequest` values of `BSK_FormationFsw`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FormationMode {
    /// `"standby"`: every task off.
    Standby,
    /// `"inertial3D"`: both spacecraft point inertially with their wheels.
    Inertial3D,
    /// `"spacecraftPointing"`: the deputy points at the chief.
    SpacecraftPointing,
}

impl FormationMode {
    const ALL: [Self; 3] = [Self::Standby, Self::Inertial3D, Self::SpacecraftPointing];

    fn event_name(self) -> &'static str {
        match self {
            Self::Standby => "initiateStandby",
            Self::Inertial3D => "initiateAttitudeGuidance",
            Self::SpacecraftPointing => "initiateSpacecraftPointing",
        }
    }

    fn tasks(self) -> &'static [FormationTask] {
        use FormationTask::*;
        match self {
            Self::Standby => &[],
            Self::Inertial3D => &[
                Inertial3DPoint,
                MrpFeedbackRws,
                Inertial3DPoint2,
                MrpFeedbackRws2,
            ],
            Self::SpacecraftPointing => &[SpacecraftPointing, MrpFeedback],
        }
    }
}

/// The formation gateway messages (upstream `setupGatewayMsgs`).
#[derive(Clone, Debug, Default)]
pub struct FormationGateway {
    pub cmd_torque: Output<BodyTorqueCommandMsg>,
    pub cmd_torque2: Output<BodyTorqueCommandMsg>,
    pub cmd_torque_direct: Output<BodyTorqueCommandMsg>,
    pub att_ref: Output<AttitudeReferenceMsg>,
    pub att_ref2: Output<AttitudeReferenceMsg>,
    pub att_guid: Output<AttitudeGuidanceMsg>,
    pub att_guid2: Output<AttitudeGuidanceMsg>,
    pub cmd_rw_motor: Output<ArrayMotorTorqueMsg>,
    pub cmd_rw_motor2: Output<ArrayMotorTorqueMsg>,
}

impl FormationGateway {
    /// Upstream `zeroGateWayMsgs`.
    pub fn zero(&self) {
        for torque in [&self.cmd_torque, &self.cmd_torque2, &self.cmd_torque_direct] {
            torque.write(BodyTorqueCommandMsg::default());
        }
        self.att_ref.write(AttitudeReferenceMsg::default());
        self.att_ref2.write(AttitudeReferenceMsg::default());
        self.att_guid.write(AttitudeGuidanceMsg::default());
        self.att_guid2.write(AttitudeGuidanceMsg::default());
        self.cmd_rw_motor.write(ArrayMotorTorqueMsg::default());
        self.cmd_rw_motor2.write(ArrayMotorTorqueMsg::default());
    }
}

/// Upstream `BSKFswModels` of the formation.
pub struct FormationFsw {
    pub task_period_nanos: u64,
    tasks: Vec<TaskSwitch>,
    pub gateway: FormationGateway,
    mode: ModeRequest<FormationMode>,
    pub vehicle_config_msg: Output<VehicleConfigMsg>,
    pub rw_config_msg: Output<RwArrayConfigMsg>,
    pub inertial_3d: InTasks<Inertial3D>,
    pub tracking_error: InTasks<AttTrackingError>,
    pub inertial_3d_2: InTasks<Inertial3D>,
    pub tracking_error_2: InTasks<AttTrackingError>,
    pub mrp_feedback_rws: InTasks<MrpFeedback>,
    pub rw_motor_torque: InTasks<RwMotorTorque>,
    pub mrp_feedback_rws_2: InTasks<MrpFeedback>,
    pub rw_motor_torque_2: InTasks<RwMotorTorque>,
    pub mrp_feedback_control: InTasks<MrpFeedback>,
    pub spacecraft_pointing: InTasks<SpacecraftPointing>,
    /// `trackingError2` as listed in `spacecraftPointingTask`.
    pub tracking_error_2_pointing: InTasks<AttTrackingError>,
}

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

fn tracking_error(
    name: &str,
    nav: &Output<NavigationAttitudeMsg>,
    reference: &Output<AttitudeReferenceMsg>,
    guidance: &Output<AttitudeGuidanceMsg>,
) -> AttTrackingError {
    let mut module = AttTrackingError::new(AttTrackingErrorConfig {
        name: name.to_string(),
        sigma_r0r: Vector3::zeros(),
    });
    nav.connect_to(&mut module.att_nav_in_msg);
    reference.connect_to(&mut module.att_ref_in_msg);
    module.att_guid_out_msg = guidance.clone();
    module
}

/// Upstream `SetMRPFeedbackRWA` and `SetRWMotorTorque` of one spacecraft.
fn wheel_control(
    names: (&str, &str),
    guidance: &Output<AttitudeGuidanceMsg>,
    torque: &Output<BodyTorqueCommandMsg>,
    motor: &Output<ArrayMotorTorqueMsg>,
    speeds: &Output<RwSpeedMsg>,
    vehicle_config: &Output<VehicleConfigMsg>,
    rw_config: &Output<RwArrayConfigMsg>,
) -> (MrpFeedback, RwMotorTorque) {
    let mut control = mrp_feedback(names.0, 3.5, -1.0, 30.0);
    vehicle_config.connect_to(&mut control.veh_config_in_msg);
    speeds.connect_to(&mut control.rw_speeds_in_msg);
    rw_config.connect_to(&mut control.rw_params_in_msg);
    guidance.connect_to(&mut control.guid_in_msg);
    control.cmd_torque_out_msg = torque.clone();
    let mut motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: names.1.to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    torque.connect_to(&mut motor_torque.veh_control_in_msg);
    motor_torque.rw_motor_torque_out_msg = motor.clone();
    rw_config.connect_to(&mut motor_torque.rw_params_in_msg);
    (control, motor_torque)
}

impl FormationFsw {
    pub fn new(base: &mut BskSim, dynamics: &mut FormationDynamics) -> Self {
        let tasks: Vec<TaskSwitch> = FormationTask::ALL
            .iter()
            .map(|_| TaskSwitch::new(false))
            .collect();
        let task = |task: FormationTask| {
            tasks[FormationTask::ALL
                .iter()
                .position(|known| *known == task)
                .expect("task")]
            .clone()
        };
        let gateway = FormationGateway::default();
        let vehicle_config_msg = Output::new(VehicleConfigMsg {
            inertia_about_point_b_kg_m2: hub_inertia_kg_m2(),
            ..VehicleConfigMsg::default()
        });
        // The same wheel configuration for both spacecraft.
        let rw_config_msg = Output::new(fsw_rw_config());
        let chief_nav = &dynamics.chief.simple_nav;
        let deputy_nav = &dynamics.deputy.simple_nav;

        let mut inertial_3d = Inertial3D::new(Inertial3DConfig {
            name: "inertial3D".to_string(),
            sigma_r0n: Vector3::new(0.2, 0.4, 0.6),
        });
        inertial_3d.att_ref_out_msg = gateway.att_ref.clone();
        let mut inertial_3d_2 = Inertial3D::new(Inertial3DConfig {
            name: "inertial3D2".to_string(),
            sigma_r0n: Vector3::new(0.2, 0.4, 0.6),
        });
        inertial_3d_2.att_ref_out_msg = gateway.att_ref2.clone();
        let tracking = tracking_error(
            "trackingError",
            &chief_nav.attitude_out_msg,
            &gateway.att_ref,
            &gateway.att_guid,
        );
        let tracking_2 = tracking_error(
            "trackingError2",
            &deputy_nav.attitude_out_msg,
            &gateway.att_ref2,
            &gateway.att_guid2,
        );
        let tracking_2_pointing = tracking_error(
            "trackingError2",
            &deputy_nav.attitude_out_msg,
            &gateway.att_ref2,
            &gateway.att_guid2,
        );
        let (control, motor_torque) = wheel_control(
            ("mrpFeedbackRWs", "rwMotorTorque"),
            &gateway.att_guid,
            &gateway.cmd_torque,
            &gateway.cmd_rw_motor,
            &dynamics.chief.rw_speed_out_msg,
            &vehicle_config_msg,
            &rw_config_msg,
        );
        let (control_2, motor_torque_2) = wheel_control(
            ("mrpFeedbackRWs2", "rwMotorTorque2"),
            &gateway.att_guid2,
            &gateway.cmd_torque2,
            &gateway.cmd_rw_motor2,
            &dynamics.deputy.rw_speed_out_msg,
            &vehicle_config_msg,
            &rw_config_msg,
        );

        // `SetSpacecraftPointing`.
        let mut pointing = SpacecraftPointing::new(SpacecraftPointingConfig {
            name: "spacecraftPointing".to_string(),
            alignment_vector_b: Vector3::new(1.0, 2.0, 3.0),
        });
        chief_nav
            .translation_out_msg
            .connect_to(&mut pointing.chief_position_in_msg);
        deputy_nav
            .translation_out_msg
            .connect_to(&mut pointing.deputy_position_in_msg);
        pointing.att_reference_out_msg = gateway.att_ref2.clone();

        // `SetMRPFeedbackControl`.
        let mut direct = mrp_feedback("mrpFeedbackControl", 10.0, 0.0001, 30.0);
        gateway.att_guid2.connect_to(&mut direct.guid_in_msg);
        vehicle_config_msg.connect_to(&mut direct.veh_config_in_msg);
        direct.cmd_torque_out_msg = gateway.cmd_torque_direct.clone();

        // Upstream `setupGatewayMsgs`: the dynamics read the FSW commands.
        gateway
            .cmd_torque_direct
            .connect_to(&mut dynamics.deputy_ext_torque_cmd_in);
        gateway
            .cmd_rw_motor
            .connect_to(&mut dynamics.chief.rw_commands.torque_in_msg);
        gateway
            .cmd_rw_motor2
            .connect_to(&mut dynamics.deputy.rw_commands.torque_in_msg);

        let fsw = Self {
            task_period_nanos: base.fsw_rate_nanos,
            gateway,
            mode: ModeRequest::new(FormationMode::Standby),
            vehicle_config_msg,
            rw_config_msg,
            inertial_3d: InTasks::new(inertial_3d, &[&task(FormationTask::Inertial3DPoint)]),
            tracking_error: InTasks::new(tracking, &[&task(FormationTask::Inertial3DPoint)]),
            inertial_3d_2: InTasks::new(inertial_3d_2, &[&task(FormationTask::Inertial3DPoint2)]),
            tracking_error_2: InTasks::new(tracking_2, &[&task(FormationTask::Inertial3DPoint2)]),
            mrp_feedback_rws: InTasks::new(control, &[&task(FormationTask::MrpFeedbackRws)]),
            rw_motor_torque: InTasks::new(motor_torque, &[&task(FormationTask::MrpFeedbackRws)]),
            mrp_feedback_rws_2: InTasks::new(control_2, &[&task(FormationTask::MrpFeedbackRws2)]),
            rw_motor_torque_2: InTasks::new(
                motor_torque_2,
                &[&task(FormationTask::MrpFeedbackRws2)],
            ),
            mrp_feedback_control: InTasks::new(direct, &[&task(FormationTask::MrpFeedback)]),
            spacecraft_pointing: InTasks::new(
                pointing,
                &[&task(FormationTask::SpacecraftPointing)],
            ),
            tracking_error_2_pointing: InTasks::new(
                tracking_2_pointing,
                &[&task(FormationTask::SpacecraftPointing)],
            ),
            tasks,
        };
        fsw.add_mode_events(base);
        fsw
    }

    fn add_mode_events(&self, base: &mut BskSim) {
        for mode in FormationMode::ALL {
            let request = self.mode.clone();
            let (tasks, gateway) = (self.tasks.clone(), self.gateway.clone());
            base.events.add(
                mode.event_name(),
                Check::Every(self.task_period_nanos),
                true,
                move |_| request.is(mode),
                move |_, _| {
                    tasks.iter().for_each(TaskSwitch::disable);
                    gateway.zero();
                    for task in mode.tasks() {
                        let index = FormationTask::ALL
                            .iter()
                            .position(|known| known == task)
                            .expect("task");
                        tasks[index].enable();
                    }
                },
            );
        }
    }

    /// The handle of upstream's `modeRequest`; take it before scheduling.
    pub fn mode_request(&self) -> ModeRequest<FormationMode> {
        self.mode.clone()
    }

    /// Upstream's FSW process, in task and module order.
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let period = self.task_period_nanos;
        basilisk_rs::schedule! { *simulation,
            "inertial3D" => &mut self.inertial_3d, period = period, priority = 3_000;
            "trackingError" => &mut self.tracking_error, period = period, priority = 2_990;
            "inertial3D2" => &mut self.inertial_3d_2, period = period, priority = 2_980;
            "trackingError2" => &mut self.tracking_error_2, period = period, priority = 2_970;
            "mrpFeedbackRWs" => &mut self.mrp_feedback_rws, period = period, priority = 2_900;
            "rwMotorTorque" => &mut self.rw_motor_torque, period = period, priority = 2_890;
            "mrpFeedbackRWs2" => &mut self.mrp_feedback_rws_2, period = period, priority = 2_880;
            "rwMotorTorque2" => &mut self.rw_motor_torque_2, period = period, priority = 2_870;
            "mrpFeedbackControl" => &mut self.mrp_feedback_control, period = period, priority = 2_860;
            "spacecraftPointing" => &mut self.spacecraft_pointing, period = period, priority = 2_000;
            "trackingError2Pointing" => &mut self.tracking_error_2_pointing, period = period, priority = 1_990;
        }
    }
}
