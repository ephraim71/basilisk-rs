use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct ReactionWheelStateMsg {
    pub omega_radps: f64,
    pub theta_rad: f64,
}

impl TelemetryMessage for ReactionWheelStateMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "omega_radps".to_string(),
                value: self.omega_radps,
            },
            TelemetryField {
                path: "theta_rad".to_string(),
                value: self.theta_rad,
            },
        ]
    }
}
