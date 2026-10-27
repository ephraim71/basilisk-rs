//! Coarse-sun-sensor constellation configuration (upstream
//! `CSSConfigMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;
use super::{CssUnitConfigMsg, MAX_NUM_CSS_SENSORS, filled_prefix};

/// Configuration of the whole CSS constellation: upstream `CSSConfigMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CssConfigMsg {
    /// Upstream `nCSS` [-]: number of sensors in use, the leading entries of
    /// `css_units`.
    pub num_css: usize,
    /// Upstream `cssVals` [-]: per-sensor configuration; entries past
    /// `num_css` are unused and zero.
    pub css_units: [CssUnitConfigMsg; MAX_NUM_CSS_SENSORS],
}

impl CssConfigMsg {
    /// Builds the message from the sensors in use, setting `num_css`.
    ///
    /// # Panics
    ///
    /// If more than [`MAX_NUM_CSS_SENSORS`] sensors are given.
    pub fn from_active(css_units: &[CssUnitConfigMsg]) -> Self {
        Self {
            num_css: css_units.len(),
            css_units: filled_prefix(css_units, "CSS configurations"),
        }
    }

    /// The first `num_css` entries (at most [`MAX_NUM_CSS_SENSORS`]).
    pub fn active_units(&self) -> &[CssUnitConfigMsg] {
        &self.css_units[..self.num_css.min(MAX_NUM_CSS_SENSORS)]
    }
}

impl_telemetry!(CssConfigMsg { num_css, css_units });
