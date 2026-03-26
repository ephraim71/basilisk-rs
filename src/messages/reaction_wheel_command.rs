use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct ReactionWheelCommandMsg {
    pub motor_torque_nm: f64,
}

impl TelemetryMessage for ReactionWheelCommandMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![TelemetryField {
            path: "motor_torque_nm".to_string(),
            value: self.motor_torque_nm,
        }]
    }
}
