//! Vizard scene description: port of upstream Basilisk
//! `simulation/vizard/_GeneralModuleFiles/vizStructures.h`.
//!
//! These are the settings and per-spacecraft devices that
//! [`VizInterface`](super::viz_interface::VizInterface) packs into Vizard's
//! protobuffer messages, and the thruster grouping
//! [`DataFileToViz`](super::data_file_to_viz::DataFileToViz) shares with it.
//! Across the structures:
//!
//! * Colors are [`Rgba`], four channels from 0 to 255. Upstream's "use
//!   Vizard's default" color sentinels (`int color[4] = {-1}`, or an empty
//!   `std::vector<int>`) are `None`.
//! * Upstream's `int` switches whose values are -1 (off), 0 (Vizard's
//!   default) and 1 (on) are [`VizSetting`]. Switches with more states keep
//!   upstream's integer codes.
//! * Angles are radians, as upstream; the protobuffer mapping converts them
//!   to the degrees Vizard reads.
//! * Numeric settings for which Vizard reads a sentinel as "use the default"
//!   (`ambient = -1`, `keyboardZoomRate = -1`, ...) keep the sentinel as
//!   their default and are sent as they are.
//! * Upstream `MsgCurrStatus`, the per-message freshness record built from
//!   message headers, is not ported; see [`VizInterface`]'s header stand-in.
//!
//! The per-spacecraft containers hold their input ports: one port per
//! device, registered through the `add_*` methods of [`VizSpacecraft`].
//!
//! [`VizInterface`]: super::viz_interface::VizInterface

use nalgebra::Vector3;

use crate::dynamics::reaction_wheel_state_effector::ReactionWheelStateEffectorConfig;
use crate::messages::{
    ChargeMsmMsg, ColorMsg, DataNodeUsageMsg, DataStorageStatusMsg, DeviceCommandMsg, FuelTankMsg,
    Input, PlanetStateMsg, PowerStorageStatusMsg, ReactionWheelStateMsg, SpacecraftStateMsg,
    SunSensorMsg, ThrusterOutputMsg,
};
use crate::sensors::coarse_sun_sensor::CoarseSunSensorConfig;

pub use crate::messages::VizSetting;

/// An RGBA color, each channel from 0 to 255 (upstream `int[4]`).
pub type Rgba = [u8; 4];

/// A named thruster group: upstream `ThrClusterMap`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ThrClusterMap {
    /// Upstream `thrTag`: name of the group (the thruster effector's
    /// `ModelTag` upstream).
    pub thr_tag: String,
    /// Upstream `color`: plume color of every thruster in the group; `None`
    /// is upstream's `{-1}`, Vizard's default.
    pub color: Option<Rgba>,
    /// Upstream `thrOffset` \[m\]: offset subtracted from each thruster's
    /// position, used when the thrusters sit on an effector branch.
    pub thr_offset_m: Vector3<f64>,
}

/// A line from one scene object towards another: upstream `PointLine`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PointLine {
    /// Upstream `fromBodyName`: body the line starts at.
    pub from_body_name: String,
    /// Upstream `toBodyName`: body the line points to.
    pub to_body_name: String,
    /// Upstream `lineColor`.
    pub line_color: Rgba,
}

/// A body-fixed cone that changes opacity when another body enters or leaves
/// it: upstream `KeepOutInCone`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct KeepOutInCone {
    /// Upstream `isKeepIn`: a keep-in cone, else keep-out.
    pub is_keep_in: bool,
    /// Upstream `position_B` \[m\]: cone apex in the from-body frame.
    pub position_b_m: Vector3<f64>,
    /// Upstream `normalVector_B` [-]: cone axis in the from-body frame.
    pub normal_vector_b: Vector3<f64>,
    /// Upstream `incidenceAngle` \[rad\]: cone half angle.
    pub incidence_angle_rad: f64,
    /// Upstream `coneHeight` \[m\]: drawn height (display only).
    pub cone_height_m: f64,
    /// Upstream `fromBodyName`: body the cone is attached to.
    pub from_body_name: String,
    /// Upstream `toBodyName`: body whose presence in the cone is detected.
    pub to_body_name: String,
    /// Upstream `coneColor`.
    pub cone_color: Rgba,
    /// Upstream `coneName`: empty lets Vizard name the cone.
    pub cone_name: String,
}

/// How a standard camera is aimed: upstream `StdCameraSettings.setMode`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum StdCameraMode {
    /// Point at a body; upstream `0`.
    BodyTarget = 0,
    /// Point along a body-fixed vector; upstream `1`, the default.
    #[default]
    PointingVector = 1,
}

/// Body-targeting view of a standard camera: upstream
/// `StdCameraSettings.setView`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum StdCameraView {
    /// Upstream `0`, the default.
    #[default]
    Nadir = 0,
    /// Upstream `1`.
    OrbitNormal = 1,
    /// Upstream `2`.
    AlongTrack = 2,
}

/// A standard Vizard camera on a spacecraft: upstream `StdCameraSettings`.
#[derive(Clone, Debug, PartialEq)]
pub struct StdCameraSettings {
    /// Upstream `spacecraftName`: spacecraft carrying the camera.
    pub spacecraft_name: String,
    /// Upstream `setMode`.
    pub set_mode: StdCameraMode,
    /// Upstream `showHUDElementsInImage`.
    pub show_hud_elements_in_image: VizSetting,
    /// Upstream `fieldOfView` \[rad\]: edge-to-edge field of view; `None` is
    /// upstream's `-1`, Vizard's default.
    pub field_of_view_rad: Option<f64>,
    /// Upstream `bodyTarget`: body to point at in body-targeting mode.
    pub body_target: String,
    /// Upstream `setView`: view in body-targeting mode.
    pub set_view: StdCameraView,
    /// Upstream `pointingVector_B` [-]: pointing direction in
    /// pointing-vector mode; +x by default, as `vizSupport` sets it.
    pub pointing_vector_b: Vector3<f64>,
    /// Upstream `position_B` \[m\]: camera position; a zero vector lets
    /// Vizard place the camera outside the spacecraft along the pointing
    /// vector.
    pub position_b_m: Vector3<f64>,
    /// Upstream `displayName`: optional panel name.
    pub display_name: String,
}

impl Default for StdCameraSettings {
    fn default() -> Self {
        Self {
            spacecraft_name: String::new(),
            set_mode: StdCameraMode::PointingVector,
            show_hud_elements_in_image: VizSetting::UseVizDefault,
            field_of_view_rad: None,
            body_target: String::new(),
            set_view: StdCameraView::Nadir,
            pointing_vector_b: Vector3::x(),
            position_b_m: Vector3::zeros(),
            display_name: String::new(),
        }
    }
}

/// Which actuator panels and heads-up displays Vizard shows for a
/// spacecraft: upstream `ActuatorGuiSettings`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ActuatorGuiSettings {
    /// Upstream `spacecraftName`.
    pub spacecraft_name: String,
    /// Upstream `viewThrusterPanel`.
    pub view_thruster_panel: VizSetting,
    /// Upstream `viewThrusterHUD`.
    pub view_thruster_hud: VizSetting,
    /// Upstream `viewRWPanel`.
    pub view_rw_panel: VizSetting,
    /// Upstream `viewRWHUD`.
    pub view_rw_hud: VizSetting,
    /// Upstream `showThrusterLabels`.
    pub show_thruster_labels: VizSetting,
    /// Upstream `showRWLabels`.
    pub show_rw_labels: VizSetting,
}

/// Which instrument displays Vizard shows for a spacecraft: upstream
/// `InstrumentGuiSettings`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct InstrumentGuiSettings {
    /// Upstream `spacecraftName`.
    pub spacecraft_name: String,
    /// Upstream `viewCSSPanel`.
    pub view_css_panel: VizSetting,
    /// Upstream `viewCSSBoresight`.
    pub view_css_boresight: VizSetting,
    /// Upstream `viewCSSCoverage`.
    pub view_css_coverage: VizSetting,
    /// Upstream `showCSSLabels`.
    pub show_css_labels: VizSetting,
    /// Upstream `showGenericSensorLabels`.
    pub show_generic_sensor_labels: VizSetting,
    /// Upstream `showTransceiverLabels`.
    pub show_transceiver_labels: VizSetting,
    /// Upstream `showTransceiverFrustum`.
    pub show_transceiver_frustum: VizSetting,
    /// Upstream `showGenericStoragePanel`.
    pub show_generic_storage_panel: VizSetting,
    /// Upstream `showMultiShapeLabels`.
    pub show_multi_shape_labels: VizSetting,
}

/// A CAD model or primitive shape drawn in place of scene objects: upstream
/// `CustomModel`.
#[derive(Clone, Debug, PartialEq)]
pub struct CustomModel {
    /// Upstream `modelPath`: path to an `.obj`, or `CUBE`, `CYLINDER` or
    /// `SPHERE` for a primitive.
    pub model_path: String,
    /// Upstream `simBodiesToModify`: bodies to replace; `ALL_SPACECRAFT`
    /// replaces every spacecraft.
    pub sim_bodies_to_modify: Vec<String>,
    /// Upstream `offset` \[m\]: model offset.
    pub offset_m: Vector3<f64>,
    /// Upstream `rotation` \[rad\]: 3-2-1 Euler angles of the model.
    pub rotation_rad: Vector3<f64>,
    /// Upstream `scale` [-]: scale factors along x, y and z; ones by default,
    /// as `vizSupport` sets them.
    pub scale: Vector3<f64>,
    /// Upstream `customTexturePath`: optional texture.
    pub custom_texture_path: String,
    /// Upstream `normalMapPath`: optional normal map for the texture.
    pub normal_map_path: String,
    /// Upstream `shader`: -1 Vizard's default, 0 Unity specular standard, 1
    /// Unity standard.
    pub shader: i32,
    /// Upstream `color`: albedo color, `None` for Vizard's grey.
    pub color: Option<Rgba>,
}

impl Default for CustomModel {
    fn default() -> Self {
        Self {
            model_path: String::new(),
            sim_bodies_to_modify: Vec::new(),
            offset_m: Vector3::zeros(),
            rotation_rad: Vector3::zeros(),
            scale: Vector3::repeat(1.0),
            custom_texture_path: String::new(),
            normal_map_path: String::new(),
            shader: -1,
            color: None,
        }
    }
}

/// A ground or spacecraft location: upstream `LocationPbMsg`.
#[derive(Clone, Debug, PartialEq)]
pub struct Location {
    /// Upstream `stationName`: label of the location.
    pub station_name: String,
    /// Upstream `parentBodyName`: planet or spacecraft P the location sits on.
    pub parent_body_name: String,
    /// Upstream `r_GP_P` \[m\]: position relative to P, P-frame components.
    pub r_gp_p_m: Vector3<f64>,
    /// Upstream `gHat_P` [-]: location normal, P-frame components.
    pub g_hat_p: Vector3<f64>,
    /// Upstream `fieldOfView` \[rad\]: edge-to-edge field of view; -1, the
    /// default, is Vizard's default.
    pub field_of_view_rad: f64,
    /// Upstream `color`; `None` is upstream's `{-1}`, Vizard's default.
    pub color: Option<Rgba>,
    /// Upstream `range` \[m\]: 0 is Vizard's default.
    pub range_m: f64,
    /// Upstream `markerScale` [-]: marker size factor; 0 is Vizard's default.
    pub marker_scale: f64,
    /// Upstream `isHidden`.
    pub is_hidden: bool,
    /// Upstream `label`: shown instead of the station name; `NOLABEL` hides
    /// the label.
    pub label: String,
}

impl Default for Location {
    fn default() -> Self {
        Self {
            station_name: String::new(),
            parent_body_name: String::new(),
            r_gp_p_m: Vector3::zeros(),
            g_hat_p: Vector3::zeros(),
            field_of_view_rad: -1.0,
            color: None,
            range_m: 0.0,
            marker_scale: 0.0,
            is_hidden: false,
            label: String::new(),
        }
    }
}

/// A generic sensor's field-of-view display: upstream `GenericSensor`.
#[derive(Clone, Debug, Default)]
pub struct GenericSensor {
    /// Upstream `r_SB_B` \[m\]: sensor position, body components.
    pub r_sb_b_m: Vector3<f64>,
    /// Upstream `fieldOfView` \[rad\]: one edge-to-edge angle for a conical
    /// sensor, two for a rectangular one.
    pub field_of_view_rad: Vec<f64>,
    /// Upstream `normalVector` [-]: boresight, body components.
    pub normal_vector: Vector3<f64>,
    /// Upstream `isHidden` (`int`): hide the sensor display.
    pub is_hidden: bool,
    /// Upstream `size` \[m\]: display size; 0 is Vizard's default.
    pub size_m: f64,
    /// Upstream `color`: one color per sensor mode (modes 0 and 1 use the
    /// first); empty for Vizard's default.
    pub color: Vec<Rgba>,
    /// Upstream `label`.
    pub label: String,
    /// Upstream `genericSensorCmdInMsg` (optional): sets
    /// `generic_sensor_cmd` while connected.
    pub generic_sensor_cmd_in_msg: Input<DeviceCommandMsg>,
    /// Upstream `genericSensorCmd` [-]: the sensor mode Vizard shows.
    pub generic_sensor_cmd: u64,
}

/// An ellipsoid drawn around a spacecraft: upstream `Ellipsoid`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Ellipsoid {
    /// Upstream `isOn`.
    pub is_on: VizSetting,
    /// Upstream `useBodyFrame`: body frame, else the Hill frame.
    pub use_body_frame: VizSetting,
    /// Upstream `position` \[m\]: center, in the body or Hill frame.
    pub position_m: Vector3<f64>,
    /// Upstream `semiMajorAxes` \[m\]: semi-axes, in the body or Hill frame.
    pub semi_major_axes_m: Vector3<f64>,
    /// Upstream `color`; `None` for Vizard's translucent gold.
    pub color: Option<Rgba>,
    /// Upstream `showGridLines`.
    pub show_grid_lines: VizSetting,
}

/// A region drawn on a planet or spacecraft: upstream `QuadMap`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct QuadMap {
    /// Upstream `ID`: identifies the region for later updates.
    pub id: i32,
    /// Upstream `parentBodyName`: body P the region is drawn on.
    pub parent_body_name: String,
    /// Upstream `vertices` \[m\]: four corners per quad, clockwise, in the P
    /// frame (upstream flattens them to `x, y, z` triples).
    pub vertices_m: Vec<Vector3<f64>>,
    /// Upstream `color`.
    pub color: Option<Rgba>,
    /// Upstream `isHidden`.
    pub is_hidden: bool,
    /// Upstream `label`: shown in the region; `NOLABEL` removes it.
    pub label: String,
}

/// A light on a spacecraft: upstream `Light`.
#[derive(Clone, Debug)]
pub struct Light {
    /// Upstream `label`.
    pub label: String,
    /// Upstream `position` \[m\]: body components.
    pub position_m: Vector3<f64>,
    /// Upstream `fieldOfView` \[rad\]: edge-to-edge beam angle.
    pub field_of_view_rad: f64,
    /// Upstream `normalVector` [-]: beam direction, body components.
    pub normal_vector: Vector3<f64>,
    /// Upstream `lightOn`: set from `on_off_cmd_in_msg` while connected.
    pub light_on: VizSetting,
    /// Upstream `range` \[m\]: distance the light acts over.
    pub range_m: f64,
    /// Upstream `intensity` [-]: -1, the default, is Vizard's 1.0.
    pub intensity: f64,
    /// Upstream `showLightMarker`.
    pub show_light_marker: VizSetting,
    /// Upstream `markerDiameter` \[m\]: -1, the default, is Vizard's 0.01 m.
    pub marker_diameter_m: f64,
    /// Upstream `color`; `None` for Vizard's white.
    pub color: Option<Rgba>,
    /// Upstream `gammaSaturation` [-]: -1, the default, is Vizard's 0.8.
    pub gamma_saturation: f64,
    /// Upstream `showLensFlare`.
    pub show_lens_flare: VizSetting,
    /// Upstream `lensFlareBrightness` [-]: -1, the default, is Vizard's 0.3.
    pub lens_flare_brightness: f64,
    /// Upstream `lensFlareFadeSpeed` [-]: -1, the default, is Vizard's.
    pub lens_flare_fade_speed: f64,
    /// Upstream `onOffCmdInMsg` (optional): its command, cast to `int` and
    /// clamped to -1, 0 or 1, sets `light_on`.
    pub on_off_cmd_in_msg: Input<DeviceCommandMsg>,
}

impl Default for Light {
    fn default() -> Self {
        Self {
            label: String::new(),
            position_m: Vector3::zeros(),
            field_of_view_rad: 0.0,
            normal_vector: Vector3::zeros(),
            light_on: VizSetting::UseVizDefault,
            range_m: 0.0,
            intensity: -1.0,
            show_light_marker: VizSetting::UseVizDefault,
            marker_diameter_m: -1.0,
            color: None,
            gamma_saturation: -1.0,
            show_lens_flare: VizSetting::UseVizDefault,
            lens_flare_brightness: -1.0,
            lens_flare_fade_speed: -1.0,
            on_off_cmd_in_msg: Input::default(),
        }
    }
}

/// One multi-sphere-model (MSM) shape: upstream `MultiShape`.
#[derive(Clone, Debug, PartialEq)]
pub struct MultiShape {
    /// Upstream `isOn`.
    pub is_on: VizSetting,
    /// Upstream `position` \[m\]: center, body components.
    pub position_m: Vector3<f64>,
    /// Upstream `radius` \[m\].
    pub radius_m: f64,
    /// Upstream `currentValue` \[C\]: charge; set from the MSM charge message
    /// while it is connected.
    pub current_value_c: f64,
    /// Upstream `maxValue` \[C\]: largest charge, for scaling the display.
    pub max_value_c: f64,
    /// Upstream `positiveColor`; `None` for Vizard's green.
    pub positive_color: Option<Rgba>,
    /// Upstream `negativeColor`; `None` for Vizard's red.
    pub negative_color: Option<Rgba>,
    /// Upstream `neutralOpacity` [-]: 0 to 255; -1, the default, is Vizard's.
    pub neutral_opacity: i32,
    /// Upstream `shape`: `CUBE`, `CYLINDER` or `SPHERE` (Vizard's default).
    pub shape: String,
    /// Upstream `dimensions` \[m\]: size along x, y and z (z is a cylinder's
    /// height).
    pub dimensions_m: Vector3<f64>,
    /// Upstream `rotation` [-]: MRP attitude of the shape in the body frame.
    pub rotation: Vector3<f64>,
}

impl Default for MultiShape {
    fn default() -> Self {
        Self {
            is_on: VizSetting::UseVizDefault,
            position_m: Vector3::zeros(),
            radius_m: 0.0,
            current_value_c: 0.0,
            max_value_c: 0.0,
            positive_color: None,
            negative_color: None,
            neutral_opacity: -1,
            shape: String::new(),
            dimensions_m: Vector3::zeros(),
            rotation: Vector3::zeros(),
        }
    }
}

/// A spacecraft's MSM shapes and the charges driving them: upstream
/// `MultiShapeInfo`.
#[derive(Clone, Debug, Default)]
pub struct MultiShapeInfo {
    /// Upstream `msmList`.
    pub msm_list: Vec<MultiShape>,
    /// Upstream `msmChargeInMsg` (optional): one charge per shape, which
    /// sets each shape's `current_value_c`.
    pub msm_charge_in_msg: Input<ChargeMsmMsg>,
}

/// A storage device's fill bar: upstream `GenericStorage`.
#[derive(Clone, Debug, Default)]
pub struct GenericStorage {
    /// Upstream `label`: name of the device.
    pub label: String,
    /// Upstream `type`: kind of device ("Battery", "Propellant Tank", ...).
    pub storage_type: String,
    /// Upstream `currentValue`: amount stored; set from a connected state
    /// message.
    pub current_value: f64,
    /// Upstream `maxValue`: capacity; set from a connected state message.
    pub max_value: f64,
    /// Upstream `units`: units of the stored quantity.
    pub units: String,
    /// Upstream `color`: bar colors between the thresholds, one more than
    /// there are thresholds.
    pub color: Vec<Rgba>,
    /// Upstream `thresholds` \[%\]: fractions of `max_value` at which the bar
    /// changes color.
    pub thresholds: Vec<i32>,
    /// Upstream `batteryStateInMsg` (optional; connect one state message).
    pub battery_state_in_msg: Input<PowerStorageStatusMsg>,
    /// Upstream `dataStorageStateInMsg` (optional; connect one state
    /// message).
    pub data_storage_state_in_msg: Input<DataStorageStatusMsg>,
    /// Upstream `fuelTankStateInMsg` (optional; connect one state message).
    pub fuel_tank_state_in_msg: Input<FuelTankMsg>,
}

/// An antenna's access cone and transmission display: upstream
/// `Transceiver`.
#[derive(Clone, Debug, Default)]
pub struct Transceiver {
    /// Upstream `r_SB_B` \[m\]: position, body components.
    pub r_sb_b_m: Vector3<f64>,
    /// Upstream `fieldOfView` \[rad\]: edge-to-edge access cone.
    pub field_of_view_rad: f64,
    /// Upstream `normalVector` [-]: boresight, body components.
    pub normal_vector: Vector3<f64>,
    /// Upstream `isHidden` (`int`).
    pub is_hidden: bool,
    /// Upstream `color`: transmission animation color.
    pub color: Option<Rgba>,
    /// Upstream `label`.
    pub label: String,
    /// Upstream `animationSpeed`: 1 (slowest) to 10; 0 is Vizard's default.
    pub animation_speed: i32,
    /// Upstream `transceiverStateInMsgs` (optional): the data nodes the
    /// antenna serves; see [`Self::add_transceiver_state_input`].
    pub transceiver_state_in_msgs: Vec<Input<DataNodeUsageMsg>>,
    /// Upstream `transceiverState`: 0 off, 1 sending, 2 receiving, 3 both.
    /// While any state input is registered, set from them each update: a
    /// connected node with a negative baud rate is sending, one with a
    /// positive baud rate receiving.
    pub transceiver_state: i32,
}

impl Transceiver {
    /// Adds an entry of upstream `transceiverStateInMsgs` and returns it to
    /// connect.
    pub fn add_transceiver_state_input(&mut self) -> &mut Input<DataNodeUsageMsg> {
        self.transceiver_state_in_msgs.push(Input::default());
        self.transceiver_state_in_msgs
            .last_mut()
            .expect("just pushed")
    }
}

/// One reaction wheel's inputs.
///
/// Upstream reads a single `RWConfigLogMsg` per wheel. The reaction wheel
/// effector here publishes the same data as two ports, its state
/// ([`ReactionWheelStateMsg`]) and its configuration
/// ([`ReactionWheelStateEffectorConfig`]), so Vizard reads both.
#[derive(Clone, Debug, Default)]
pub struct VizReactionWheel {
    /// Wheel speed `Omega` and motor torque `u_current`: connect a wheel's
    /// `state_out`. A wheel is drawn only while this is connected.
    pub rw_state_in_msg: Input<ReactionWheelStateMsg>,
    /// Position `rWB_B`, spin axis `gsHat_B` and limits `Omega_max` and
    /// `u_max`: connect a wheel's `config`.
    pub rw_config_in_msg: Input<ReactionWheelStateEffectorConfig>,
}

/// One thruster's input and its group: an entry of upstream `thrInMsgs` with
/// the matching entry of `thrInfo`.
#[derive(Clone, Debug, Default)]
pub struct VizThruster {
    /// The group this thruster belongs to.
    pub thr_info: ThrClusterMap,
    /// Upstream `thrInMsgs[i]`.
    pub thr_in_msg: Input<ThrusterOutputMsg>,
}

/// One coarse sun sensor's display data: upstream `CSSConfigLogMsgPayload`.
///
/// The sensor here publishes only its signal ([`SunSensorMsg`]); its
/// geometry and limits are configuration, copied in through
/// [`Self::from_sensor_config`] or set directly.
#[derive(Clone, Debug, Default)]
pub struct VizCoarseSunSensor {
    /// Upstream `r_B` \[m\]: sensor position, body components.
    pub r_b_m: Vector3<f64>,
    /// Upstream `nHat_B` [-]: boresight unit vector, body components.
    pub n_hat_b: Vector3<f64>,
    /// Upstream `fov` \[rad\]: boresight-to-edge field of view.
    pub fov_rad: f64,
    /// Upstream `maxSignal` [-]: -1 when not set.
    pub max_signal: f64,
    /// Upstream `minSignal` [-]: -1 when not set.
    pub min_signal: f64,
    /// Upstream `CSSGroupID`: cluster the sensor belongs to.
    pub css_group_id: i32,
    /// Upstream `signal`: connect a sensor's `output_sun_sensor_msg`; its
    /// sensed value is drawn. A sensor is drawn only while this is connected.
    pub css_signal_in_msg: Input<SunSensorMsg>,
}

impl VizCoarseSunSensor {
    /// The display data upstream `coarseSunSensor` logs for a sensor with
    /// this configuration, in group 0.
    pub fn from_sensor_config(config: &CoarseSunSensorConfig) -> Self {
        Self {
            r_b_m: config.position_m,
            // The sensor frame's +z axis, as `CoarseSunSensor` measures along.
            n_hat_b: config
                .body_to_sensor_quaternion
                .inverse()
                .transform_vector(&Vector3::z()),
            fov_rad: config.fov_half_angle_rad,
            max_signal: config.max_output,
            min_signal: config.min_output,
            css_group_id: 0,
            css_signal_in_msg: Input::default(),
        }
    }
}

/// One spacecraft (or spacecraft component) and its devices: upstream
/// `VizSpacecraftData`.
#[derive(Clone, Debug)]
pub struct VizSpacecraft {
    /// Upstream `spacecraftName`; `"bsk-Sat"` by default.
    pub spacecraft_name: String,
    /// Upstream `parentSpacecraftName`: the spacecraft this object is a part
    /// of; empty for a whole spacecraft.
    pub parent_spacecraft_name: String,
    /// Upstream `scStateInMsg`: the spacecraft is drawn only while this is
    /// connected.
    pub sc_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `rwInMsgs`, one entry per wheel; see
    /// [`Self::add_reaction_wheel`].
    pub reaction_wheels: Vec<VizReactionWheel>,
    /// Upstream `thrInMsgs` with `thrInfo`; see [`Self::add_thruster`].
    pub thrusters: Vec<VizThruster>,
    /// Upstream `cssInMsgs`; see [`Self::add_coarse_sun_sensor`].
    pub coarse_sun_sensors: Vec<VizCoarseSunSensor>,
    /// Upstream `genericSensorList`.
    pub generic_sensors: Vec<GenericSensor>,
    /// Upstream `transceiverList`.
    pub transceivers: Vec<Transceiver>,
    /// Upstream `genericStorageList`.
    pub generic_storage: Vec<GenericStorage>,
    /// Upstream `lightList`.
    pub lights: Vec<Light>,
    /// Upstream `spacecraftSprite`: sprite shape (`CIRCLE`, `SQUARE`,
    /// `STAR`, `TRIANGLE` or `bskSat`) with an optional RGB color.
    pub spacecraft_sprite: String,
    /// Upstream `modelDictionaryKey`: Vizard model to use instead of the one
    /// the name selects.
    pub model_dictionary_key: String,
    /// Upstream `logoTexture`: path to a texture for the built-in models.
    pub logo_texture: String,
    /// Upstream `oscOrbitLineColor`.
    pub osc_orbit_line_color: Option<Rgba>,
    /// Upstream `trueTrajectoryLineColor`; replaced by
    /// `true_trajectory_line_color_in_msg` while that is connected.
    pub true_trajectory_line_color: Option<Rgba>,
    /// Upstream `trueTrajectoryLineColorInMsg` (optional).
    pub true_trajectory_line_color_in_msg: Input<ColorMsg>,
    /// Upstream `groundTrackLineColor`.
    pub ground_track_line_color: Option<Rgba>,
    /// Upstream `groundTrackBodyName`: body to draw the ground track on;
    /// empty for the dominant gravity body.
    pub ground_track_body_name: String,
    /// Upstream `msmInfo`.
    pub msm_info: MultiShapeInfo,
    /// Upstream `ellipsoidList`.
    pub ellipsoids: Vec<Ellipsoid>,
}

impl Default for VizSpacecraft {
    fn default() -> Self {
        Self::new("bsk-Sat")
    }
}

impl VizSpacecraft {
    /// A spacecraft with no devices and an unconnected state input.
    pub fn new(spacecraft_name: impl Into<String>) -> Self {
        Self {
            spacecraft_name: spacecraft_name.into(),
            parent_spacecraft_name: String::new(),
            sc_state_in_msg: Input::default(),
            reaction_wheels: Vec::new(),
            thrusters: Vec::new(),
            coarse_sun_sensors: Vec::new(),
            generic_sensors: Vec::new(),
            transceivers: Vec::new(),
            generic_storage: Vec::new(),
            lights: Vec::new(),
            spacecraft_sprite: String::new(),
            model_dictionary_key: String::new(),
            logo_texture: String::new(),
            osc_orbit_line_color: None,
            true_trajectory_line_color: None,
            true_trajectory_line_color_in_msg: Input::default(),
            ground_track_line_color: None,
            ground_track_body_name: String::new(),
            msm_info: MultiShapeInfo::default(),
            ellipsoids: Vec::new(),
        }
    }

    /// Adds a reaction wheel (an entry of upstream `rwInMsgs`) and returns
    /// its inputs to connect.
    pub fn add_reaction_wheel(&mut self) -> &mut VizReactionWheel {
        self.reaction_wheels.push(VizReactionWheel::default());
        self.reaction_wheels.last_mut().expect("just pushed")
    }

    /// Adds a thruster of group `thr_info` (an entry of upstream `thrInMsgs`
    /// and `thrInfo`) and returns its input to connect.
    pub fn add_thruster(&mut self, thr_info: ThrClusterMap) -> &mut Input<ThrusterOutputMsg> {
        self.thrusters.push(VizThruster {
            thr_info,
            thr_in_msg: Input::default(),
        });
        &mut self.thrusters.last_mut().expect("just pushed").thr_in_msg
    }

    /// Adds a coarse sun sensor (an entry of upstream `cssInMsgs`) and
    /// returns its signal input to connect.
    pub fn add_coarse_sun_sensor(
        &mut self,
        sensor: VizCoarseSunSensor,
    ) -> &mut Input<SunSensorMsg> {
        self.coarse_sun_sensors.push(sensor);
        &mut self
            .coarse_sun_sensors
            .last_mut()
            .expect("just pushed")
            .css_signal_in_msg
    }
}

/// Settings Vizard reads from the first message: upstream `VizSettings`.
///
/// The defaults are upstream's, which for most fields mean "use Vizard's
/// default".
#[derive(Clone, Debug, PartialEq)]
pub struct VizSettings {
    /// Upstream `ambient` [-]: ambient light from 0 to 8; -1 is Vizard's
    /// default.
    pub ambient: f64,
    /// Upstream `orbitLinesOn`: osculating orbit lines; -1 off, 0 Vizard's
    /// default, 1 relative to the parent body, 2 relative to the chief.
    pub orbit_lines_on: i32,
    /// Upstream `trueTrajectoryLinesOn`: true-path lines; -1 off, 0 Vizard's
    /// default, 1 inertial, 2 relative to the chief, 3 relative to a body, 4
    /// in the rotating frame of two bodies, 5 in a body's fixed frame.
    pub true_trajectory_lines_on: i32,
    /// Upstream `spacecraftCSon`: spacecraft coordinate axes.
    pub spacecraft_cs_on: VizSetting,
    /// Upstream `planetCSon`: planet coordinate axes.
    pub planet_cs_on: VizSetting,
    /// Upstream `pointLineList`.
    pub point_line_list: Vec<PointLine>,
    /// Upstream `coneList`.
    pub cone_list: Vec<KeepOutInCone>,
    /// Upstream `stdCameraList`.
    pub std_camera_list: Vec<StdCameraSettings>,
    /// Upstream `customModelList`.
    pub custom_model_list: Vec<CustomModel>,
    /// Upstream `actuatorGuiSettingsList`.
    pub actuator_gui_settings_list: Vec<ActuatorGuiSettings>,
    /// Upstream `instrumentGuiSettingsList`.
    pub instrument_gui_settings_list: Vec<InstrumentGuiSettings>,
    /// Upstream `skyBox`: empty for the NASA SVS star map, `ESO`, `black`,
    /// or a path to a custom background.
    pub sky_box: String,
    /// Upstream `viewCameraBoresightHUD`.
    pub view_camera_boresight_hud: VizSetting,
    /// Upstream `viewCameraFrustumHUD`.
    pub view_camera_frustum_hud: VizSetting,
    /// Upstream `viewCameraViewHUD`.
    pub view_camera_view_hud: VizSetting,
    /// Upstream `showCSLabels`.
    pub show_cs_labels: VizSetting,
    /// Upstream `showCelestialBodyLabels`.
    pub show_celestial_body_labels: VizSetting,
    /// Upstream `showSpacecraftLabels`.
    pub show_spacecraft_labels: VizSetting,
    /// Upstream `showCameraLabels`.
    pub show_camera_labels: VizSetting,
    /// Upstream `customGUIReferenceHeight` \[px\]: GUI scaling; -1 or 0 is
    /// Vizard's default, otherwise at least 300.
    pub custom_gui_reference_height_px: f64,
    /// Upstream `defaultSpacecraftSprite`: sprite for every spacecraft.
    pub default_spacecraft_sprite: String,
    /// Upstream `showSpacecraftAsSprites`.
    pub show_spacecraft_as_sprites: VizSetting,
    /// Upstream `showCelestialBodiesAsSprites`.
    pub show_celestial_bodies_as_sprites: VizSetting,
    /// Upstream `show24hrClock`.
    pub show_24hr_clock: VizSetting,
    /// Upstream `showDataRateDisplay`.
    pub show_data_rate_display: VizSetting,
    /// Upstream `keyboardAngularRate` \[rad/s\]: camera rotation per key
    /// press; zero or less is Vizard's default.
    pub keyboard_angular_rate_radps: f64,
    /// Upstream `keyboardZoomRate` [-]: zero or less is Vizard's default.
    pub keyboard_zoom_rate: f64,
    /// Upstream `defaultThrusterColor`; `None` is upstream's `{-1}`.
    pub default_thruster_color: Option<Rgba>,
    /// Upstream `defaultThrusterPlumeLifeScalar` [-]: plume length factor; 1
    /// or 0 is Vizard's default.
    pub default_thruster_plume_life_scalar: f64,
    /// Upstream `orbitLineSegments`: 0 is Vizard's default, otherwise at
    /// least 4.
    pub orbit_line_segments: i32,
    /// Upstream `relativeOrbitRange` \[deg\]: 0 is Vizard's default.
    pub relative_orbit_range_deg: i32,
    /// Upstream `mainCameraTarget`: body the main camera starts on.
    pub main_camera_target: String,
    /// Upstream `showHillFrame`.
    pub show_hill_frame: VizSetting,
    /// Upstream `showVelocityFrame`.
    pub show_velocity_frame: VizSetting,
    /// Upstream `relativeOrbitFrame`: 0 or 1 the Hill frame, 2 the velocity
    /// frame, 3 inertial.
    pub relative_orbit_frame: i32,
    /// Upstream `spacecraftShadowBrightness` [-]: 0 to 1; negative is
    /// Vizard's default.
    pub spacecraft_shadow_brightness: f64,
    /// Upstream `spacecraftSizeMultiplier` [-]: negative is Vizard's
    /// default.
    pub spacecraft_size_multiplier: f64,
    /// Upstream `spacecraftHelioViewSizeMultiplier` [-]: negative is
    /// Vizard's default.
    pub spacecraft_helio_view_size_multiplier: f64,
    /// Upstream `forceStartAtSpacecraftLocalView`; off by default.
    pub force_start_at_spacecraft_local_view: VizSetting,
    /// Upstream `useSimpleLocationMarkers`: Vizard's default simplifies them
    /// past 100 locations; off forces full markers, on simple ones.
    pub use_simple_location_markers: VizSetting,
    /// Upstream `showLocationCommLines`.
    pub show_location_comm_lines: VizSetting,
    /// Upstream `showLocationCones`.
    pub show_location_cones: VizSetting,
    /// Upstream `showLocationLabels`.
    pub show_location_labels: VizSetting,
    /// Upstream `atmospheresOff`.
    pub atmospheres_off: VizSetting,
    /// Upstream `scViewToPlanetViewBoundaryMultiplier` [-]: 1 to 10, times
    /// 1000 m; 0 is Vizard's default.
    pub sc_view_to_planet_view_boundary_multiplier: i32,
    /// Upstream `planetViewToHelioViewBoundaryMultiplier` [-]: 1 to 10; 0 is
    /// Vizard's default.
    pub planet_view_to_helio_view_boundary_multiplier: i32,
    /// Upstream `sunIntensity` [-]: 0 is Vizard's default.
    pub sun_intensity: f64,
    /// Upstream `attenuateSunLightWithDistance`.
    pub attenuate_sun_light_with_distance: VizSetting,
    /// Upstream `showLightLabels`.
    pub show_light_labels: VizSetting,
    /// Upstream `celestialBodyHelioViewSizeMultiplier` [-]: negative is
    /// Vizard's default.
    pub celestial_body_helio_view_size_multiplier: f64,
    /// Upstream `showMissionTime`.
    pub show_mission_time: VizSetting,
    /// Upstream `keyboardLiveInput`: keys Vizard reports back in live
    /// streaming mode.
    pub keyboard_live_input: String,
    /// Upstream `messageBufferSize` \[bytes\]: -1 loads the whole file; 0 is
    /// Vizard's default.
    pub message_buffer_size_bytes: i64,
    /// Upstream `truePathRelativeBody`.
    pub true_path_relative_body: String,
    /// Upstream `truePathRotatingFrame`: two body names separated by a space.
    pub true_path_rotating_frame: String,
    /// Upstream `truePathFixedFrame`.
    pub true_path_fixed_frame: String,
    /// Upstream `showQuadMapLabels`.
    pub show_quad_map_labels: VizSetting,
    /// Upstream `spacecraftOrbitLineWidth` [-]: 0 is Vizard's default.
    pub spacecraft_orbit_line_width: f64,
    /// Upstream `celestialBodyOrbitLineWidth` [-]: 0 is Vizard's default.
    pub celestial_body_orbit_line_width: f64,
    /// Upstream `linesAndFramesLineWidth` [-]: 0 is Vizard's default.
    pub lines_and_frames_line_width: f64,
    /// Upstream `useLineRenderersForTargetLinesAndFrames`.
    pub use_line_renderers_for_target_lines_and_frames: VizSetting,
    /// Upstream `osculatingOrbitLineRange` \[rad\]: lowest and highest true
    /// anomaly to draw relative to the current one; `None` (upstream's
    /// empty vector) is Vizard's default.
    pub osculating_orbit_line_range_rad: Option<[f64; 2]>,
    /// Upstream `osculatingGroundTrackRange` \[rad\]: as above, for the
    /// osculating ground track.
    pub osculating_ground_track_range_rad: Option<[f64; 2]>,
    /// Upstream `showOsculatingGroundTrackLines`.
    pub show_osculating_ground_track_lines: VizSetting,
    /// Upstream `showTruePathGroundTrackLines`.
    pub show_true_path_ground_track_lines: VizSetting,
}

impl Default for VizSettings {
    fn default() -> Self {
        Self {
            ambient: -1.0,
            orbit_lines_on: 0,
            true_trajectory_lines_on: 0,
            spacecraft_cs_on: VizSetting::UseVizDefault,
            planet_cs_on: VizSetting::UseVizDefault,
            point_line_list: Vec::new(),
            cone_list: Vec::new(),
            std_camera_list: Vec::new(),
            custom_model_list: Vec::new(),
            actuator_gui_settings_list: Vec::new(),
            instrument_gui_settings_list: Vec::new(),
            sky_box: String::new(),
            view_camera_boresight_hud: VizSetting::UseVizDefault,
            view_camera_frustum_hud: VizSetting::UseVizDefault,
            view_camera_view_hud: VizSetting::UseVizDefault,
            show_cs_labels: VizSetting::UseVizDefault,
            show_celestial_body_labels: VizSetting::UseVizDefault,
            show_spacecraft_labels: VizSetting::UseVizDefault,
            show_camera_labels: VizSetting::UseVizDefault,
            custom_gui_reference_height_px: -1.0,
            default_spacecraft_sprite: String::new(),
            show_spacecraft_as_sprites: VizSetting::UseVizDefault,
            show_celestial_bodies_as_sprites: VizSetting::UseVizDefault,
            show_24hr_clock: VizSetting::UseVizDefault,
            show_data_rate_display: VizSetting::UseVizDefault,
            keyboard_angular_rate_radps: -1.0,
            keyboard_zoom_rate: -1.0,
            default_thruster_color: None,
            default_thruster_plume_life_scalar: 1.0,
            orbit_line_segments: 0,
            relative_orbit_range_deg: 0,
            main_camera_target: String::new(),
            show_hill_frame: VizSetting::UseVizDefault,
            show_velocity_frame: VizSetting::UseVizDefault,
            relative_orbit_frame: 0,
            spacecraft_shadow_brightness: -1.0,
            spacecraft_size_multiplier: -1.0,
            spacecraft_helio_view_size_multiplier: -1.0,
            force_start_at_spacecraft_local_view: VizSetting::Disabled,
            use_simple_location_markers: VizSetting::UseVizDefault,
            show_location_comm_lines: VizSetting::UseVizDefault,
            show_location_cones: VizSetting::UseVizDefault,
            show_location_labels: VizSetting::UseVizDefault,
            atmospheres_off: VizSetting::UseVizDefault,
            sc_view_to_planet_view_boundary_multiplier: 0,
            planet_view_to_helio_view_boundary_multiplier: 0,
            sun_intensity: 0.0,
            attenuate_sun_light_with_distance: VizSetting::UseVizDefault,
            show_light_labels: VizSetting::UseVizDefault,
            celestial_body_helio_view_size_multiplier: -1.0,
            show_mission_time: VizSetting::UseVizDefault,
            keyboard_live_input: String::new(),
            message_buffer_size_bytes: 0,
            true_path_relative_body: String::new(),
            true_path_rotating_frame: String::new(),
            true_path_fixed_frame: String::new(),
            show_quad_map_labels: VizSetting::UseVizDefault,
            spacecraft_orbit_line_width: 0.0,
            celestial_body_orbit_line_width: 0.0,
            lines_and_frames_line_width: 0.0,
            use_line_renderers_for_target_lines_and_frames: VizSetting::UseVizDefault,
            osculating_orbit_line_range_rad: None,
            osculating_ground_track_range_rad: None,
            show_osculating_ground_track_lines: VizSetting::UseVizDefault,
            show_true_path_ground_track_lines: VizSetting::UseVizDefault,
        }
    }
}

/// Settings Vizard reads from every message: upstream `LiveVizSettings`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LiveVizSettings {
    /// Upstream `targetLineList`: lines redrawn every message.
    pub target_line_list: Vec<PointLine>,
    /// Upstream `relativeOrbitChief`: chief spacecraft of the relative orbit
    /// display; empty or `AUTO` follows the camera target.
    pub relative_orbit_chief: String,
    /// Upstream `terminateVizard`: shut Vizard down. The message carrying it
    /// is the last one sent and is not saved.
    pub terminate_vizard: bool,
    /// Upstream `playbackPaused`.
    pub playback_paused: bool,
    /// Upstream `playbackInRealTime`: 1 real time, -1 frame rate, 0 Vizard's
    /// default.
    pub playback_in_real_time: i32,
    /// Upstream `playbackMultiplier` [-]: playback at `2^multiplier` speed.
    pub playback_multiplier: i32,
}

/// A dialog box shown to the Vizard user: upstream `VizEventDialog`.
#[derive(Clone, Debug, PartialEq)]
pub struct VizEventDialog {
    /// Upstream `eventHandlerID`: returned with the user's replies.
    pub event_handler_id: String,
    /// Upstream `displayString`: the information or question shown.
    pub display_string: String,
    /// Upstream `userOptions`: the choices offered; empty for an
    /// informational dialog.
    pub user_options: Vec<String>,
    /// Upstream `durationOfDisplay` \[s\]: -1, the default, shows the dialog
    /// until the user closes it.
    pub duration_of_display_s: f64,
    /// Upstream `useSimElapsedTimeForDuration`: time the display in
    /// simulation rather than wall-clock time.
    pub use_sim_elapsed_time_for_duration: bool,
    /// Upstream `useConfirmationPanel`: ask the user to confirm a choice.
    pub use_confirmation_panel: VizSetting,
    /// Upstream `hideOnSelection`: -1 keep showing, 0 Vizard's default, 1
    /// hide, 2 destroy the dialog once the user chooses.
    pub hide_on_selection: i32,
    /// Upstream `dialogFormat`: `WARNING`, `CAUTION`, or `none` (the
    /// default) for Vizard's format.
    pub dialog_format: String,
}

impl Default for VizEventDialog {
    fn default() -> Self {
        Self {
            event_handler_id: String::new(),
            display_string: String::new(),
            user_options: Vec::new(),
            duration_of_display_s: -1.0,
            use_sim_elapsed_time_for_duration: false,
            use_confirmation_panel: VizSetting::UseVizDefault,
            hide_on_selection: 0,
            dialog_format: "none".to_string(),
        }
    }
}

/// A celestial body Vizard draws: upstream `GravBodyInfo`.
#[derive(Clone, Debug, PartialEq)]
pub struct GravBodyInfo {
    /// Upstream `bodyName`.
    pub body_name: String,
    /// Upstream `mu` \[m^3/s^2\]: gravitational parameter.
    pub mu_m3ps2: f64,
    /// Upstream `radEquator` \[m\]: equatorial radius.
    pub rad_equator_m: f64,
    /// Upstream `radiusRatio` [-]: polar over equatorial radius.
    pub radius_ratio: f64,
    /// Upstream `modelDictionaryKey`: Vizard model to use instead of the one
    /// the name selects.
    pub model_dictionary_key: String,
}

impl Default for GravBodyInfo {
    fn default() -> Self {
        Self {
            body_name: String::new(),
            mu_m3ps2: 0.0,
            rad_equator_m: 0.0,
            radius_ratio: 1.0,
            model_dictionary_key: String::new(),
        }
    }
}

/// A celestial body and its ephemeris input: an entry of upstream
/// `gravBodyInformation` with the matching entry of `spiceInMsgs`.
#[derive(Clone, Debug, Default)]
pub struct VizCelestialBody {
    /// Upstream `gravBodyInformation[i]`.
    pub grav_body_info: GravBodyInfo,
    /// Upstream `spiceInMsgs[i]` (optional): unconnected, or without an
    /// orientation, the body sits at the origin with an identity
    /// orientation, as upstream's default SPICE payload.
    pub planet_state_in_msg: Input<PlanetStateMsg>,
}
