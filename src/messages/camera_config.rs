//! Camera configuration (upstream `CameraConfigMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

/// What the camera renders: upstream `renderMode`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum CameraRenderMode {
    /// Visual image; upstream `0`, the zeroed default.
    #[default]
    Visual = 0,
    /// Depth buffer rendered to the image; upstream `1`.
    DepthMap = 1,
}

/// An optional Vizard switch with a "use Vizard's default" state: upstream
/// `postProcessingOn` and `showHUDElementsInImage`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(i8)]
pub enum VizSetting {
    /// Leave it to Vizard; upstream `0`, the zeroed (protobuffer) default.
    #[default]
    UseVizDefault = 0,
    /// Forced off; upstream `-1`.
    Disabled = -1,
    /// Forced on; upstream `1`.
    Enabled = 1,
}

/// Camera intrinsics, mounting and Vizard rendering options: upstream
/// `CameraConfigMsg`.
///
/// A pure configuration input, so it has no telemetry. Upstream's fixed
/// `char[MAX_STRING_LENGTH]` strings are `String`s here.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CameraConfigMsg {
    /// Upstream `cameraID` [-]: ID of the camera.
    pub camera_id: i64,
    /// Upstream `isOn` (`int`): the camera takes images at the rendering rate.
    pub is_on: bool,
    /// Upstream `parentName`: name of the body the camera is attached to.
    pub parent_name: String,
    /// Upstream `fieldOfView` \[rad\]: edge-to-edge field of view along the
    /// camera y axis.
    pub field_of_view_rad: f64,
    /// Upstream `resolution` \[pixels\]: image width and height.
    pub resolution_px: [i32; 2],
    /// Upstream `renderRate` \[ns\]: interval between captured frames.
    pub render_rate_nanos: u64,
    /// Upstream `cameraPos_B` \[m\]: camera position, body components.
    pub camera_pos_b_m: Vector3<f64>,
    /// Upstream `sigma_CB` [-]: MRP attitude of the camera frame relative to
    /// the body frame.
    pub sigma_cb: Vector3<f64>,
    /// Upstream `skyBox`: star-field preference.
    pub sky_box: String,
    /// Upstream `postProcessingOn` (`int` -1 / 0 / 1): image post-processing.
    pub post_processing_on: VizSetting,
    /// Upstream `ppFocusDistance` \[m\]: distance to the point of focus (minimum
    /// 0.1); 0 turns the parameter off.
    pub pp_focus_distance_m: f64,
    /// Upstream `ppAperture` [-]: aperture ratio (f-number, 0.05 to 32); 0
    /// turns the parameter off.
    pub pp_aperture: f64,
    /// Upstream `ppFocalLength` \[m\]: focal length (0.001 to 0.3); 0 turns the
    /// parameter off.
    pub pp_focal_length_m: f64,
    /// Upstream `ppMaxBlurSize` [-]: bokeh kernel size — 1 small, 2 medium,
    /// 3 large, 4 extra large; 0 turns the parameter off.
    pub pp_max_blur_size: i32,
    /// Upstream `updateCameraParameters` (`int`): command the instrument
    /// camera to adopt this message's parameters.
    pub update_camera_parameters: bool,
    /// Upstream `renderMode` (`int` 0 / 1).
    pub render_mode: CameraRenderMode,
    /// Upstream `depthMapClippingPlanes` \[m\]: near and far clipping planes in
    /// depth-map mode (Vizard defaults 0.1 and 100).
    pub depth_map_clipping_planes_m: [f64; 2],
    /// Upstream `showHUDElementsInImage` (`int` -1 / 0 / 1).
    pub show_hud_elements_in_image: VizSetting,
}
