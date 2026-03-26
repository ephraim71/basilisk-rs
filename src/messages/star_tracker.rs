use nalgebra::UnitQuaternion;

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug)]
pub struct StarTrackerMsg {
    pub attitude_inertial_to_sensor: UnitQuaternion<f64>,
}

impl Default for StarTrackerMsg {
    fn default() -> Self {
        Self {
            attitude_inertial_to_sensor: UnitQuaternion::identity(),
        }
    }
}

impl TelemetryMessage for StarTrackerMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        let q = self.attitude_inertial_to_sensor.quaternion();
        vec![
            TelemetryField {
                path: "attitude_inertial_to_sensor.x".to_string(),
                value: q.i,
            },
            TelemetryField {
                path: "attitude_inertial_to_sensor.y".to_string(),
                value: q.j,
            },
            TelemetryField {
                path: "attitude_inertial_to_sensor.z".to_string(),
                value: q.k,
            },
            TelemetryField {
                path: "attitude_inertial_to_sensor.w".to_string(),
                value: q.w,
            },
        ]
    }
}
