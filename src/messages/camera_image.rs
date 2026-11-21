//! An image taken by a camera (upstream `CameraImageMsgPayload`).

use serde::{Deserialize, Serialize};

use crate::telemetry::{TelemetryField, TelemetryMessage};

/// One camera image: upstream `CameraImageMsg`, which the Vizard interface
/// (`vizard::viz_interface::VizInterface`, behind the `vizard` feature)
/// publishes for each instrument camera image Vizard renders.
///
/// Upstream carries a `void*` to a buffer its producer keeps alive, plus the
/// buffer length; this message owns the bytes instead, so `image.len()` is
/// upstream's `imageBufferLength`.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CameraImageMsg {
    /// Upstream `timeTag` \[ns\]: simulation time the image was taken at.
    pub time_tag_nanos: u64,
    /// Upstream `valid` (`int`, 1 / 0): an image is present.
    pub valid: bool,
    /// Upstream `cameraID` [-]: ID of the camera that took the image.
    pub camera_id: i64,
    /// Upstream `imagePointer` and `imageBufferLength`: the encoded image,
    /// byte for byte as the renderer sent it.
    pub image: Vec<u8>,
    /// Upstream `imageType` [-]: channels per pixel, 3 for RGB and 4 for
    /// RGBA.
    pub image_type: i8,
}

impl TelemetryMessage for CameraImageMsg {
    /// The scalars and the image length; the bytes are not columns.
    fn flatten(&self) -> Vec<TelemetryField> {
        [
            ("time_tag_nanos", self.time_tag_nanos as f64),
            ("valid", if self.valid { 1.0 } else { 0.0 }),
            ("camera_id", self.camera_id as f64),
            ("image_buffer_length", self.image.len() as f64),
            ("image_type", f64::from(self.image_type)),
        ]
        .into_iter()
        .map(|(path, value)| TelemetryField {
            path: path.to_string(),
            value,
        })
        .collect()
    }
}
