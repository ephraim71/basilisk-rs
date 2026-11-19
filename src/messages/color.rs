//! An RGBA color (upstream `ColorMsgPayload`).

use serde::{Deserialize, Serialize};

use crate::telemetry::{TelemetryField, TelemetryMessage};

/// A display color: upstream `ColorMsg`, which Vizard reads for a
/// spacecraft's true-trajectory line.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ColorMsg {
    /// Upstream `colorRGBA` (`int[4]`, 0 to 255): red, green, blue and
    /// alpha.
    pub color_rgba: [u8; 4],
}

impl TelemetryMessage for ColorMsg {
    /// One column per channel, `color_rgba.0` to `color_rgba.3`.
    fn flatten(&self) -> Vec<TelemetryField> {
        self.color_rgba
            .iter()
            .enumerate()
            .map(|(index, channel)| TelemetryField {
                path: format!("color_rgba.{index}"),
                value: f64::from(*channel),
            })
            .collect()
    }
}
