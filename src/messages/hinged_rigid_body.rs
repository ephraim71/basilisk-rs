use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct HingedRigidBodyMsg {
    pub theta_rad: f64,
    pub theta_dot_radps: f64,
}

impl TelemetryMessage for HingedRigidBodyMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "theta_rad".to_string(),
                value: self.theta_rad,
            },
            TelemetryField {
                path: "theta_dot_radps".to_string(),
                value: self.theta_dot_radps,
            },
        ]
    }
}
