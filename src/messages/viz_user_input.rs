//! What a Vizard user did since the last update (upstream
//! `VizUserInputMsgPayload`).

use serde::{Deserialize, Serialize};

use crate::telemetry::{TelemetryField, TelemetryMessage};

/// A user's answer to a Vizard event dialog: upstream `VizEventReply`.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct VizEventReply {
    /// Upstream `eventHandlerID`: name of the dialog's event handler.
    pub event_handler_id: String,
    /// Upstream `reply`: the option the user selected.
    pub reply: String,
    /// Upstream `eventHandlerDestroyed`: the dialog was closed and destroyed.
    pub event_handler_destroyed: bool,
}

/// Keyboard input and dialog replies Vizard recorded since the previous
/// request: upstream `VizUserInputMsg`, which the Vizard interface
/// (`vizard::viz_interface::VizInterface`, behind the `vizard` feature)
/// publishes in live-streaming mode.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct VizUserInputMsg {
    /// Upstream `frameNumber` [-]: Vizard frame number.
    pub frame_number: i32,
    /// Upstream `keyboardInput`: the keys pressed, in order, among those
    /// Vizard was asked to listen for.
    pub keyboard_input: String,
    /// Upstream `vizEventReplies`: the dialog replies.
    pub viz_event_replies: Vec<VizEventReply>,
}

impl TelemetryMessage for VizUserInputMsg {
    /// The frame number; the keys and replies are not columns.
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![TelemetryField {
            path: "frame_number".to_string(),
            value: f64::from(self.frame_number),
        }]
    }
}
