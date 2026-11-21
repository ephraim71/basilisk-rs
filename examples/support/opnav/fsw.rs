//! Upstream `OpNavScenarios/modelsOpNav/BSK_OpNavFsw.py`: the flight
//! software of the OpNav scenarios, its tasks, gateway messages and mode
//! events.
//!
//! | upstream task (priority) | modules (priority) | here |
//! |---|---|---|
//! | `opNavPointTask` (20) | `houghCircles` (15), `pixelLine` (12), `opNavPoint` (9) | [`FswTask::OpNavPoint`] |
//! | `headingPointTask` (20) | `houghCircles` (15), `pixelLine` (12), `headingUKF` (10), `opNavPoint` (9) | [`FswTask::HeadingPoint`] |
//! | `opNavPointLimbTask` (20) | `limbFind` (25), `limbNav` (12), `opNavPoint` (10) | [`FswTask::OpNavPointLimb`] |
//! | `opNavAttODLimbTask` (20) | `limbFind` (25), `limbNav` (12), `opNavPoint` (10), `relativeOD` (9) | [`FswTask::OpNavAttOdLimb`] |
//! | `opNavPointTaskCheat` (20) | `hillPoint` (10), `trackingErrorCam` (9) | [`FswTask::OpNavPointCheat`] |
//! | `mrpFeedbackRWsTask` (15) | `mrpFeedbackRWs` (9), `rwMotorTorque` (8) | [`FswTask::MrpFeedbackRws`] |
//! | `opNavODTaskLimb` (15) | `limbFind` (25), `limbNav` (22), `relativeOD` (20) | [`FswTask::OpNavOdLimb`] |
//! | `imageProcTask` (9) | `houghCircles` (15) | [`FswTask::ImageProc`] |
//! | `opNavODTaskB` (9) | `houghCircles` (15), `pixelLineFilter` (13) | [`FswTask::OpNavOdB`] |
//! | `opNavAttODTask` (9) | `houghCircles` (15), `pixelLine` (14), `opNavPoint` (10), `relativeOD` (9) | [`FswTask::OpNavAttOd`] |
//! | `cnnAttODTask` (9) | `opNavCNN` (15), `pixelLine` (14), `opNavPoint` (10), `relativeOD` (9) | [`FswTask::CnnAttOd`] |
//! | `opNavFaultDet` (9) | `limbFind` (25), `limbNav` (20), `houghCircles` (18), `pixelLine` (16), `OpNav_Fault` (14), `relativeOD` (9) | [`FswTask::OpNavFaultDet`] |
//! | `attODFaultDet` (9) | as `opNavFaultDet`, with `opNavPoint` (10) | [`FswTask::AttOdFaultDet`] |
//! | `cnnFaultDet` (9) | `opNavCNN` (25), `pixelLine` (20), `houghCircles` (18), `pixelLine` (16), `OpNav_Fault` (14), `opNavPoint` (10), `relativeOD` (9) | [`FswTask::CnnFaultDet`] |
//! | `opNavODTask` (5) | `houghCircles` (15), `pixelLine` (14), `relativeOD` (13) | [`FswTask::OpNavOd`] |
//!
//! # The FSW process
//!
//! Unlike BskSim's, these tasks share modules at different positions:
//! `opNavPoint` runs before the attitude control in `pointOpNav` (task
//! priority 20) and after it in `OpNavAttOD` (priority 9), and `cnnFaultDet`
//! runs `pixelLine` twice. So the whole FSW process is one module,
//! [`OpNavFsw`], scheduled at the FSW rate: at each tick it runs the enabled
//! tasks in upstream's order (priority, then creation order, the order of
//! [`FswTask::RUN_ORDER`]), and each task's modules in their priority order,
//! a module listed in several enabled tasks once per task, as upstream.
//!
//! # Gateway messages and modes
//!
//! Upstream's `attGuidMsg`, `opnavMsg`, `opnavPrimaryMsg`,
//! `opnavSecondaryMsg` and `opnavCirclesMsg`, written by several modules, are
//! the [`OpNavGateway`] outputs handed to each authoring module as its output
//! port. [`FswMode`] is upstream's `modeRequest` string; [`OpNavFsw::new`]
//! adds one event per mode, checked at the FSW rate: when the request names
//! its mode, the event disables every task, zeroes the gateway
//! (`zeroGateWayMsgs`) and enables the mode's tasks. Unlike BskSim's, these
//! events do not re-activate each other, so each mode can start once.
//!
//! # Differences from upstream
//!
//! * `pixelLineFilter.gamma = 0.9` has no counterpart: the port of
//!   `pixelLineBiasUKF` has no `gamma` parameter (upstream never reads it).
//! * `opNavCNN` reads its network from `assets/opencv/center_radius_cnn/
//!   CAD.onnx` (`just fetch-opencv-assets`).

use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_determination::heading_sukf::{
    HeadingSukf, HeadingSukfConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::hill_point::{HillPoint, HillPointConfig};
use basilisk_rs::fsw_algorithms::att_guidance::op_nav_point::{OpNavPoint, OpNavPointConfig};
use basilisk_rs::fsw_algorithms::effector_interfaces::rw_motor_torque::{
    RwMotorTorque, RwMotorTorqueConfig,
};
use basilisk_rs::fsw_algorithms::image_processing::center_radius_cnn::{
    CenterRadiusCnn, CenterRadiusCnnConfig,
};
use basilisk_rs::fsw_algorithms::image_processing::horizon_op_nav::{
    HorizonOpNav, HorizonOpNavConfig,
};
use basilisk_rs::fsw_algorithms::image_processing::hough_circles::{
    HoughCircles, HoughCirclesConfig,
};
use basilisk_rs::fsw_algorithms::image_processing::limb_finding::{LimbFinding, LimbFindingConfig};
use basilisk_rs::fsw_algorithms::image_processing::pixel_line_converter::{
    PixelLineConverter, PixelLineConverterConfig,
};
use basilisk_rs::fsw_algorithms::optical_navigation::fault_detection::{
    FaultDetection, FaultDetectionConfig, FaultMode,
};
use basilisk_rs::fsw_algorithms::optical_navigation::pixel_line_bias_ukf::{
    PixelLineBiasUkf, PixelLineBiasUkfConfig,
};
use basilisk_rs::fsw_algorithms::optical_navigation::relative_od_ukf::{
    RelativeOdUkf, RelativeOdUkfConfig,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, OpNavCirclesMsg, OpNavMsg, Output, RwArrayConfigMsg, VehicleConfigMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::utilities::orbital_motion::elements_to_state;
use basilisk_rs::utilities::rigid_body_kinematics::{Axis, dcm_to_mrp, mi, mrp_to_dcm};
use basilisk_rs::{Module, SimulationContext};
use nalgebra::{SMatrix, SVector, Vector3};

use super::bsk_sim::dynamics::{fsw_rw_config, hub_inertia_kg_m2};
use super::bsk_sim::events::{Check, ModeRequest, TaskSwitch};
use super::dynamics::{CAMERA_MRP_CB, OpNavDynamics};
use super::{OpNavSim, elements};

/// Priority of the FSW process: after the dynamics, the cameras and the
/// recorders of the dynamics task.
pub const FSW_PRIORITY: i32 = 3_000;

/// The network of `opNavCNN`.
pub const CNN_NETWORK: &str = "assets/opencv/center_radius_cnn/CAD.onnx";

/// Upstream's `relativeOD` and `pixelLineFilter` default initial orbit
/// (`a = 10 000 km`, `e = 0.2`, `i = 10 deg`, `Omega = 25 deg`, `omega =
/// 10 deg`, `f = 40 deg`) about `mu = 42828.314e9 m^3/s^2`: position \[m\]
/// and velocity \[m/s\].
pub fn filter_initial_orbit() -> (Vector3<f64>, Vector3<f64>) {
    elements_to_state(
        42_828.314e9,
        &elements(10_000e3, 0.2, 10.0, 25.0, 10.0, 40.0),
    )
}

/// The modules of the FSW process.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unit {
    HillPoint,
    OpNavPoint,
    TrackingErrorCam,
    MrpFeedbackRws,
    RwMotorTorque,
    ImageProcessing,
    OpNavCnn,
    PixelLine,
    OpNavFault,
    LimbFinding,
    HorizonNav,
    RelativeOd,
    PixelLineFilter,
    HeadingUkf,
}

impl Unit {
    pub const ALL: [Self; 14] = [
        Self::HillPoint,
        Self::OpNavPoint,
        Self::TrackingErrorCam,
        Self::MrpFeedbackRws,
        Self::RwMotorTorque,
        Self::ImageProcessing,
        Self::OpNavCnn,
        Self::PixelLine,
        Self::OpNavFault,
        Self::LimbFinding,
        Self::HorizonNav,
        Self::RelativeOd,
        Self::PixelLineFilter,
        Self::HeadingUkf,
    ];
}

/// The tasks of upstream's FSW process.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FswTask {
    OpNavPoint,
    HeadingPoint,
    OpNavPointLimb,
    OpNavAttOdLimb,
    OpNavPointCheat,
    MrpFeedbackRws,
    OpNavOd,
    ImageProc,
    OpNavOdLimb,
    OpNavOdB,
    OpNavAttOd,
    CnnAttOd,
    OpNavFaultDet,
    AttOdFaultDet,
    CnnFaultDet,
}

impl FswTask {
    /// Upstream's creation order.
    pub const ALL: [Self; 15] = [
        Self::OpNavPoint,
        Self::HeadingPoint,
        Self::OpNavPointLimb,
        Self::OpNavAttOdLimb,
        Self::OpNavPointCheat,
        Self::MrpFeedbackRws,
        Self::OpNavOd,
        Self::ImageProc,
        Self::OpNavOdLimb,
        Self::OpNavOdB,
        Self::OpNavAttOd,
        Self::CnnAttOd,
        Self::OpNavFaultDet,
        Self::AttOdFaultDet,
        Self::CnnFaultDet,
    ];

    /// The order the process runs its tasks in: by priority, equal
    /// priorities in creation order.
    pub const RUN_ORDER: [Self; 15] = [
        Self::OpNavPoint,
        Self::HeadingPoint,
        Self::OpNavPointLimb,
        Self::OpNavAttOdLimb,
        Self::OpNavPointCheat,
        Self::MrpFeedbackRws,
        Self::OpNavOdLimb,
        Self::ImageProc,
        Self::OpNavOdB,
        Self::OpNavAttOd,
        Self::CnnAttOd,
        Self::OpNavFaultDet,
        Self::AttOdFaultDet,
        Self::CnnFaultDet,
        Self::OpNavOd,
    ];

    /// Upstream's task name.
    pub fn name(self) -> &'static str {
        match self {
            Self::OpNavPoint => "opNavPointTask",
            Self::HeadingPoint => "headingPointTask",
            Self::OpNavPointLimb => "opNavPointLimbTask",
            Self::OpNavAttOdLimb => "opNavAttODLimbTask",
            Self::OpNavPointCheat => "opNavPointTaskCheat",
            Self::MrpFeedbackRws => "mrpFeedbackRWsTask",
            Self::OpNavOd => "opNavODTask",
            Self::ImageProc => "imageProcTask",
            Self::OpNavOdLimb => "opNavODTaskLimb",
            Self::OpNavOdB => "opNavODTaskB",
            Self::OpNavAttOd => "opNavAttODTask",
            Self::CnnAttOd => "cnnAttODTask",
            Self::OpNavFaultDet => "opNavFaultDet",
            Self::AttOdFaultDet => "attODFaultDet",
            Self::CnnFaultDet => "cnnFaultDet",
        }
    }

    /// The task's modules, in their run (priority) order.
    pub fn units(self) -> &'static [Unit] {
        use Unit::*;
        match self {
            Self::OpNavPoint => &[ImageProcessing, PixelLine, OpNavPoint],
            Self::HeadingPoint => &[ImageProcessing, PixelLine, HeadingUkf, OpNavPoint],
            Self::OpNavPointLimb => &[LimbFinding, HorizonNav, OpNavPoint],
            Self::OpNavAttOdLimb => &[LimbFinding, HorizonNav, OpNavPoint, RelativeOd],
            Self::OpNavPointCheat => &[HillPoint, TrackingErrorCam],
            Self::MrpFeedbackRws => &[MrpFeedbackRws, RwMotorTorque],
            Self::OpNavOd => &[ImageProcessing, PixelLine, RelativeOd],
            Self::ImageProc => &[ImageProcessing],
            Self::OpNavOdLimb => &[LimbFinding, HorizonNav, RelativeOd],
            Self::OpNavOdB => &[ImageProcessing, PixelLineFilter],
            Self::OpNavAttOd => &[ImageProcessing, PixelLine, OpNavPoint, RelativeOd],
            Self::CnnAttOd => &[OpNavCnn, PixelLine, OpNavPoint, RelativeOd],
            Self::OpNavFaultDet => &[
                LimbFinding,
                HorizonNav,
                ImageProcessing,
                PixelLine,
                OpNavFault,
                RelativeOd,
            ],
            Self::AttOdFaultDet => &[
                LimbFinding,
                HorizonNav,
                ImageProcessing,
                PixelLine,
                OpNavFault,
                OpNavPoint,
                RelativeOd,
            ],
            Self::CnnFaultDet => &[
                OpNavCnn,
                PixelLine,
                ImageProcessing,
                PixelLine,
                OpNavFault,
                OpNavPoint,
                RelativeOd,
            ],
        }
    }

    fn index(self) -> usize {
        Self::ALL
            .iter()
            .position(|known| *known == self)
            .expect("known task")
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
        &self.switches[task.index()]
    }

    /// Upstream `fswProc.disableAllTasks()`.
    pub fn disable_all(&self) {
        self.switches.iter().for_each(TaskSwitch::disable);
    }

    /// Upstream `enableTask(name)`.
    pub fn enable(&self, task: FswTask) {
        self.get(task).enable();
    }

    /// The enabled tasks, in run order.
    pub fn enabled(&self) -> Vec<FswTask> {
        FswTask::RUN_ORDER
            .into_iter()
            .filter(|task| self.get(*task).is_enabled())
            .collect()
    }
}

/// Upstream `modeRequest` values of `BSK_OpNavFsw`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FswMode {
    /// `"standby"`: every task off.
    Standby,
    /// `"prepOpNav"`: point the camera at the planet with the Hill frame
    /// (`hillPoint`, from the navigation truth).
    PrepOpNav,
    /// `"imageGen"`: as `prepOpNav`, with Hough circles on each image.
    ImageGen,
    /// `"pointOpNav"`: point at the planet found by Hough circles.
    PointOpNav,
    /// `"pointHead"`: as `pointOpNav` through the heading filter.
    PointHead,
    /// `"pointLimb"`: point at the planet found from its limb.
    PointLimb,
    /// `"OpNavOD"`: Hill pointing, orbit determination from Hough circles.
    OpNavOd,
    /// `"OpNavODLimb"`: Hill pointing, orbit determination from the limb.
    OpNavOdLimb,
    /// `"OpNavODB"`: Hill pointing, the pixel-line bias filter.
    OpNavOdB,
    /// `"OpNavAttOD"`: OpNav pointing and orbit determination from Hough
    /// circles.
    OpNavAttOd,
    /// `"OpNavAttODLimb"`: the same from the limb.
    OpNavAttOdLimb,
    /// `"CNNAttOD"`: the same from the CNN.
    CnnAttOd,
    /// `"FaultDet"`: OpNav pointing and orbit determination from both image
    /// processing methods, with fault detection.
    FaultDet,
    /// `"ODFaultDet"`: the same with Hill pointing.
    OdFaultDet,
    /// `"FaultDetCNN"`: the same with the CNN and OpNav pointing.
    FaultDetCnn,
}

impl FswMode {
    pub const ALL: [Self; 15] = [
        Self::Standby,
        Self::PrepOpNav,
        Self::ImageGen,
        Self::PointOpNav,
        Self::PointHead,
        Self::PointLimb,
        Self::OpNavOd,
        Self::OpNavOdLimb,
        Self::OpNavOdB,
        Self::OpNavAttOd,
        Self::OpNavAttOdLimb,
        Self::CnnAttOd,
        Self::FaultDet,
        Self::OdFaultDet,
        Self::FaultDetCnn,
    ];

    /// Upstream's event name.
    pub fn event_name(self) -> &'static str {
        match self {
            Self::Standby => "initiateStandby",
            Self::PrepOpNav => "prepOpNav",
            Self::ImageGen => "imageGen",
            Self::PointOpNav => "pointOpNav",
            Self::PointHead => "pointHead",
            Self::PointLimb => "pointLimb",
            Self::OpNavOd => "OpNavOD",
            Self::OpNavOdLimb => "OpNavODLimb",
            Self::OpNavOdB => "OpNavODB",
            Self::OpNavAttOd => "OpNavAttOD",
            Self::OpNavAttOdLimb => "OpNavAttODLimb",
            Self::CnnAttOd => "CNNAttOD",
            Self::FaultDet => "FaultDet",
            Self::OdFaultDet => "ODFaultDet",
            Self::FaultDetCnn => "FaultDetCNN",
        }
    }

    /// The tasks the mode's event enables.
    pub fn tasks(self) -> &'static [FswTask] {
        use FswTask::*;
        match self {
            Self::Standby => &[],
            Self::PrepOpNav => &[OpNavPointCheat, MrpFeedbackRws],
            Self::ImageGen => &[ImageProc, OpNavPointCheat, MrpFeedbackRws],
            Self::PointOpNav => &[OpNavPoint, MrpFeedbackRws],
            Self::PointHead => &[HeadingPoint, MrpFeedbackRws],
            Self::PointLimb => &[OpNavPointLimb, MrpFeedbackRws],
            Self::OpNavOd => &[OpNavPointCheat, MrpFeedbackRws, OpNavOd],
            Self::OpNavOdLimb => &[OpNavPointCheat, MrpFeedbackRws, OpNavOdLimb],
            Self::OpNavOdB => &[OpNavPointCheat, MrpFeedbackRws, OpNavOdB],
            Self::OpNavAttOd => &[OpNavAttOd, MrpFeedbackRws],
            Self::OpNavAttOdLimb => &[OpNavAttOdLimb, MrpFeedbackRws],
            Self::CnnAttOd => &[CnnAttOd, MrpFeedbackRws],
            Self::FaultDet => &[AttOdFaultDet, MrpFeedbackRws],
            Self::OdFaultDet => &[OpNavPointCheat, MrpFeedbackRws, OpNavFaultDet],
            Self::FaultDetCnn => &[CnnFaultDet, MrpFeedbackRws],
        }
    }
}

/// Upstream `setupGatewayMsgs`: the messages several modules write.
#[derive(Clone, Debug, Default)]
pub struct OpNavGateway {
    pub att_guid: Output<AttitudeGuidanceMsg>,
    pub opnav: Output<OpNavMsg>,
    pub opnav_primary: Output<OpNavMsg>,
    pub opnav_secondary: Output<OpNavMsg>,
    pub opnav_circles: Output<OpNavCirclesMsg>,
}

impl OpNavGateway {
    /// Upstream `zeroGateWayMsgs`.
    pub fn zero(&self) {
        self.att_guid.write(AttitudeGuidanceMsg::default());
        self.opnav.write(OpNavMsg::default());
        self.opnav_primary.write(OpNavMsg::default());
        self.opnav_secondary.write(OpNavMsg::default());
        self.opnav_circles.write(OpNavCirclesMsg::default());
    }
}

/// Upstream `BSKFswModels` of the OpNav scenarios: the FSW process (see the
/// module docs). Its modules are public so that a scenario can change them
/// before scheduling, as upstream's `configure_initial_conditions` does.
pub struct OpNavFsw {
    /// Upstream `processTasksTimeStep` \[ns\].
    pub task_period_nanos: u64,
    pub tasks: FswTasks,
    pub gateway: OpNavGateway,
    mode: ModeRequest<FswMode>,
    /// Upstream `vcMsg`.
    pub vehicle_config_msg: Output<VehicleConfigMsg>,
    /// Upstream `fswRwConfigMsg`.
    pub rw_config_msg: Output<RwArrayConfigMsg>,
    pub hill_point: HillPoint,
    pub op_nav_point: OpNavPoint,
    pub tracking_error_cam: AttTrackingError,
    pub mrp_feedback_rws: MrpFeedback,
    pub rw_motor_torque: RwMotorTorque,
    /// Upstream `imageProcessing` (`houghCircles`).
    pub image_processing: HoughCircles,
    pub op_nav_cnn: CenterRadiusCnn,
    pub pixel_line: PixelLineConverter,
    pub op_nav_fault: FaultDetection,
    pub limb_finding: LimbFinding,
    pub horizon_nav: HorizonOpNav,
    pub relative_od: RelativeOdUkf,
    pub pixel_line_filter: PixelLineBiasUkf,
    pub heading_ukf: HeadingSukf,
}

impl OpNavFsw {
    /// Upstream `BSKFswModels(SimBase, fswRate)`: builds the modules, wires
    /// them to `dynamics` and to the gateway, and adds the mode events.
    pub fn new(base: &mut OpNavSim, dynamics: &mut OpNavDynamics) -> Self {
        let gateway = OpNavGateway::default();
        let nav_attitude = dynamics.simple_nav.attitude_out_msg.clone();
        let nav_translation = dynamics.simple_nav.translation_out_msg.clone();
        let camera_config = dynamics.camera.camera_config_out_msg.clone();
        let image = dynamics.camera.image_out_msg.clone();

        // `SetVehicleConfiguration`, `SetRWConfigMsg`.
        let vehicle_config_msg = Output::new(VehicleConfigMsg {
            inertia_about_point_b_kg_m2: hub_inertia_kg_m2(),
            ..VehicleConfigMsg::default()
        });
        let rw_config_msg = Output::new(fsw_rw_config());

        // `SetHillPointGuidance`.
        let mut hill_point = HillPoint::new(HillPointConfig {
            name: "hillPoint".to_string(),
        });
        nav_translation.connect_to(&mut hill_point.trans_nav_in_msg);
        dynamics.ephem_object.ephem_out_msgs[0].connect_to(&mut hill_point.cel_body_in_msg);

        // `SetMRPFeedbackRWA`.
        let mut mrp_feedback_rws = MrpFeedback::new(MrpFeedbackConfig {
            name: "mrpFeedbackRWs".to_string(),
            k: 3.5,
            ki: -1.0,
            p: 30.0,
            integral_limit: 2.0 / -1.0 * 0.1,
            known_torque_body_nm: Vector3::zeros(),
            control_law_type: 0,
        });
        vehicle_config_msg.connect_to(&mut mrp_feedback_rws.veh_config_in_msg);
        dynamics
            .rw_speed_out_msg
            .connect_to(&mut mrp_feedback_rws.rw_speeds_in_msg);
        rw_config_msg.connect_to(&mut mrp_feedback_rws.rw_params_in_msg);
        gateway
            .att_guid
            .connect_to(&mut mrp_feedback_rws.guid_in_msg);

        // `SetRWMotorTorque`.
        let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
            name: "rwMotorTorque".to_string(),
            control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
        });
        mrp_feedback_rws
            .cmd_torque_out_msg
            .connect_to(&mut rw_motor_torque.veh_control_in_msg);
        rw_motor_torque
            .rw_motor_torque_out_msg
            .connect_to(&mut dynamics.rw_commands.torque_in_msg);
        rw_config_msg.connect_to(&mut rw_motor_torque.rw_params_in_msg);

        // `SetAttTrackingErrorCam`.
        let mut tracking_error_cam = AttTrackingError::new(AttTrackingErrorConfig {
            name: "trackingErrorCam".to_string(),
            sigma_r0r: camera_pointing_sigma_r0r(),
        });
        hill_point
            .att_ref_out_msg
            .connect_to(&mut tracking_error_cam.att_ref_in_msg);
        nav_attitude.connect_to(&mut tracking_error_cam.att_nav_in_msg);
        tracking_error_cam.att_guid_out_msg = gateway.att_guid.clone();

        // `SetImageProcessing`.
        let mut image_processing = HoughCircles::new(HoughCirclesConfig {
            name: "houghCircles".to_string(),
            save_dir: "Test".to_string(),
            save_images: false,
            expected_circles: 1,
            canny_thresh: 200,
            vote_thresh: 25,
            hough_min_dist_px: 50,
            hough_min_radius_px: 20,
            blur_size_px: 9,
            noise_sf: 1.0,
            dp_value: 1,
            hough_max_radius_px: 0,
            ..HoughCirclesConfig::default()
        });
        image.connect_to(&mut image_processing.image_in_msg);
        image_processing.opnav_circles_out_msg = gateway.opnav_circles.clone();

        // `SetPixelLineConversion`.
        let mut pixel_line = PixelLineConverter::new(PixelLineConverterConfig {
            name: "pixelLine".to_string(),
            planet_target: 2,
        });
        gateway
            .opnav_circles
            .connect_to(&mut pixel_line.circles_in_msg);
        camera_config.connect_to(&mut pixel_line.camera_config_in_msg);
        nav_attitude.connect_to(&mut pixel_line.att_in_msg);
        pixel_line.op_nav_out_msg = gateway.opnav.clone();

        // `SetCNNOpNav`.
        let mut op_nav_cnn = CenterRadiusCnn::new(CenterRadiusCnnConfig {
            name: "opNavCNN".to_string(),
            path_to_network: std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(CNN_NETWORK),
            pixel_noise: Vector3::new(5.0, 5.0, 5.0),
            ..CenterRadiusCnnConfig::default()
        });
        image.connect_to(&mut op_nav_cnn.image_in_msg);
        op_nav_cnn.opnav_circles_out_msg = gateway.opnav_circles.clone();

        // `SetRelativeODFilter`.
        let (r_init, v_init) = filter_initial_orbit();
        let mut relative_od = RelativeOdUkf::new(RelativeOdUkfConfig {
            name: "relativeOD".to_string(),
            planet_id_init: 2,
            alpha: 0.02,
            beta: 2.0,
            kappa: 0.0,
            noise_sf: 7.5,
            state_init: SVector::from_iterator(r_init.iter().chain(v_init.iter()).copied()),
            covar_init: SMatrix::from_diagonal(&SVector::from([
                1e6, 1e6, 1e6, 0.02e6, 0.02e6, 0.02e6,
            ])),
            q_noise: relative_od_q_noise(),
        });
        gateway.opnav.connect_to(&mut relative_od.op_nav_in_msg);

        // `SetFaultDetection`.
        let mut op_nav_fault = FaultDetection::new(FaultDetectionConfig {
            name: "OpNav_Fault".to_string(),
            sigma_fault: 0.3,
            fault_mode: FaultMode::Merge,
            ..FaultDetectionConfig::default()
        });
        gateway
            .opnav_primary
            .connect_to(&mut op_nav_fault.nav_meas_primary_in_msg);
        gateway
            .opnav_secondary
            .connect_to(&mut op_nav_fault.nav_meas_secondary_in_msg);
        camera_config.connect_to(&mut op_nav_fault.camera_config_in_msg);
        nav_attitude.connect_to(&mut op_nav_fault.att_in_msg);
        op_nav_fault.op_nav_out_msg = gateway.opnav.clone();

        // `SetLimbFinding`.
        let mut limb_finding = LimbFinding::new(LimbFindingConfig {
            name: "limbFind".to_string(),
            save_images: false,
            canny_thresh_low: 50,
            canny_thresh_high: 100,
            blur_size_px: 5,
            limb_num_thresh: 0,
            ..LimbFindingConfig::default()
        });
        image.connect_to(&mut limb_finding.image_in_msg);

        // `SetHorizonNav`.
        let mut horizon_nav = HorizonOpNav::new(HorizonOpNavConfig {
            name: "limbNav".to_string(),
            planet_target: 2,
            noise_sf: 1.0,
        });
        limb_finding
            .opnav_limb_out_msg
            .connect_to(&mut horizon_nav.limb_in_msg);
        camera_config.connect_to(&mut horizon_nav.camera_config_in_msg);
        nav_attitude.connect_to(&mut horizon_nav.att_in_msg);
        horizon_nav.op_nav_out_msg = gateway.opnav.clone();

        // `SetOpNavPointGuidance`.
        let mut op_nav_point = OpNavPoint::new(OpNavPointConfig {
            name: "opNavPoint".to_string(),
            small_angle_rad: 0.001_f64.to_radians(),
            time_out_s: 1000.0,
            omega_rn_b_radps: Vector3::new(0.001, 0.0, -0.001),
            align_axis_c: Vector3::new(0.0, 0.0, 1.0),
            ..OpNavPointConfig::default()
        });
        op_nav_point.att_guidance_out_msg = gateway.att_guid.clone();
        nav_attitude.connect_to(&mut op_nav_point.imu_in_msg);
        camera_config.connect_to(&mut op_nav_point.camera_config_in_msg);
        gateway
            .opnav
            .connect_to(&mut op_nav_point.opnav_data_in_msg);

        // `SetHeadingUKF`.
        let mut heading_ukf = HeadingSukf::new(HeadingSukfConfig {
            name: "headingUKF".to_string(),
            alpha: 0.02,
            beta: 2.0,
            kappa: 0.0,
            q_obs_val: 0.001,
            state_init: SVector::from([0.0, 0.0, 1.0, 0.0, 0.0]),
            covar_init: SMatrix::from_diagonal(&SVector::from([0.2, 0.2, 0.2, 0.005, 0.005])),
            q_noise: SMatrix::from_diagonal_element(1e-6 * 1e-6),
            ..HeadingSukfConfig::default()
        });
        gateway.opnav.connect_to(&mut heading_ukf.opnav_data_in_msg);
        camera_config.connect_to(&mut heading_ukf.camera_config_in_msg);

        // `SetPixelLineFilter`.
        let mut pixel_line_filter = PixelLineBiasUkf::new(PixelLineBiasUkfConfig {
            name: "pixelLineFilter".to_string(),
            planet_id_init: 2,
            alpha: 0.02,
            beta: 2.0,
            kappa: 0.0,
            state_init: SVector::from_iterator(
                r_init
                    .iter()
                    .chain(v_init.iter())
                    .copied()
                    .chain([1.0, 1.0, 2.0]),
            ),
            covar_init: SMatrix::from_diagonal(&SVector::from([
                10e6, 10e6, 10e6, 0.01e6, 0.01e6, 0.01e6, 1.0, 1.0, 1.0,
            ])),
            q_noise: SMatrix::from_diagonal(&SVector::from([
                1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8, 1e-16, 1e-16, 1e-16,
            ])),
        });
        gateway
            .opnav_circles
            .connect_to(&mut pixel_line_filter.circles_in_msg);
        camera_config.connect_to(&mut pixel_line_filter.camera_config_in_msg);
        nav_attitude.connect_to(&mut pixel_line_filter.att_in_msg);

        gateway.zero();
        let fsw = Self {
            task_period_nanos: base.fsw_rate_nanos,
            tasks: FswTasks::default(),
            gateway,
            mode: ModeRequest::new(FswMode::Standby),
            vehicle_config_msg,
            rw_config_msg,
            hill_point,
            op_nav_point,
            tracking_error_cam,
            mrp_feedback_rws,
            rw_motor_torque,
            image_processing,
            op_nav_cnn,
            pixel_line,
            op_nav_fault,
            limb_finding,
            horizon_nav,
            relative_od,
            pixel_line_filter,
            heading_ukf,
        };
        fsw.add_mode_events(base);
        fsw
    }

    /// Upstream's mode events.
    fn add_mode_events(&self, base: &mut OpNavSim) {
        for mode in FswMode::ALL {
            let request = self.mode.clone();
            let (tasks, gateway) = (self.tasks.clone(), self.gateway.clone());
            base.events.add(
                mode.event_name(),
                Check::Every(self.task_period_nanos),
                true,
                move |_| request.is(mode),
                move |_, _| {
                    tasks.disable_all();
                    gateway.zero();
                    for task in mode.tasks() {
                        tasks.enable(*task);
                    }
                },
            );
        }
    }

    /// The handle of upstream's `modeRequest`; take it before scheduling.
    pub fn mode_request(&self) -> ModeRequest<FswMode> {
        self.mode.clone()
    }

    fn unit_mut(&mut self, unit: Unit) -> &mut dyn Module {
        match unit {
            Unit::HillPoint => &mut self.hill_point,
            Unit::OpNavPoint => &mut self.op_nav_point,
            Unit::TrackingErrorCam => &mut self.tracking_error_cam,
            Unit::MrpFeedbackRws => &mut self.mrp_feedback_rws,
            Unit::RwMotorTorque => &mut self.rw_motor_torque,
            Unit::ImageProcessing => &mut self.image_processing,
            Unit::OpNavCnn => &mut self.op_nav_cnn,
            Unit::PixelLine => &mut self.pixel_line,
            Unit::OpNavFault => &mut self.op_nav_fault,
            Unit::LimbFinding => &mut self.limb_finding,
            Unit::HorizonNav => &mut self.horizon_nav,
            Unit::RelativeOd => &mut self.relative_od,
            Unit::PixelLineFilter => &mut self.pixel_line_filter,
            Unit::HeadingUkf => &mut self.heading_ukf,
        }
    }

    /// Upstream's FSW process.
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let period = self.task_period_nanos;
        simulation.add_module("FSWProcess", self, period, FSW_PRIORITY);
    }
}

/// Upstream `SetAttTrackingErrorCam`'s `sigma_R0R`: the Hill frame rotated
/// (`euler1(90 deg) euler2(90 deg) [CB]`) so that the camera, not the body,
/// points at the planet.
pub fn camera_pointing_sigma_r0r() -> Vector3<f64> {
    let m2 = mi(90.0_f64.to_radians(), Axis::Y);
    let m3 = mi(90.0_f64.to_radians(), Axis::X);
    let m_cam = mrp_to_dcm(Vector3::from(CAMERA_MRP_CB));
    dcm_to_mrp(m3 * m2 * m_cam)
}

/// Upstream `relativeOD.qNoise`: `1e-6` on the positions, `1e-8` on the
/// velocities.
pub fn relative_od_q_noise() -> SMatrix<f64, 6, 6> {
    SMatrix::from_diagonal(&SVector::from([1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8]))
}

impl Module for OpNavFsw {
    /// Upstream's `InitializeSimulation` resets every module of the process,
    /// enabled or not.
    fn init(&mut self) {
        for unit in Unit::ALL {
            self.unit_mut(unit).init();
        }
    }

    fn reset(&mut self, context: &SimulationContext) {
        for unit in Unit::ALL {
            self.unit_mut(unit).reset(context);
        }
    }

    fn update(&mut self, context: &SimulationContext) {
        for task in self.tasks.enabled() {
            for unit in task.units() {
                self.unit_mut(*unit).update(context);
            }
        }
    }
}
