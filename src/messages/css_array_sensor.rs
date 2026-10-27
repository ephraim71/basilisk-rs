//! Coarse-sun-sensor constellation readings (upstream
//! `CSSArraySensorMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;
use super::{MAX_NUM_CSS_SENSORS, filled_prefix};

/// Raw readings of the whole CSS constellation: upstream `CSSArraySensorMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CssArraySensorMsg {
    /// Upstream `timeTag` \[s\]: vehicle time tag of the measurements.
    pub time_tag_s: f64,
    /// Upstream `CosValue` [-]: measured value per sensor, nominally the cosine
    /// of the sun incidence angle, in sensor order.
    pub cos_values: [f64; MAX_NUM_CSS_SENSORS],
}

impl CssArraySensorMsg {
    /// Builds the message from the readings of the sensors in use; the
    /// remaining entries and the time tag stay zero.
    ///
    /// # Panics
    ///
    /// If more than [`MAX_NUM_CSS_SENSORS`] values are given.
    pub fn from_active(cos_values: &[f64]) -> Self {
        Self {
            time_tag_s: 0.0,
            cos_values: filled_prefix(cos_values, "CSS readings"),
        }
    }
}

impl_telemetry!(CssArraySensorMsg {
    time_tag_s,
    cos_values,
});
