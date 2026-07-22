use nalgebra::Vector3;

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct NavigationAttitudeMsg {
    pub time_tag_s: f64,
    pub sigma_bn: Vector3<f64>,
    pub omega_bn_b_radps: Vector3<f64>,
    pub vehicle_sun_point_body: Vector3<f64>,
}

impl TelemetryMessage for NavigationAttitudeMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        let mut fields = vec![TelemetryField {
            path: "time_tag_s".to_string(),
            value: self.time_tag_s,
        }];
        for (name, vector) in [
            ("sigma_bn", self.sigma_bn),
            ("omega_bn_b_radps", self.omega_bn_b_radps),
            ("vehicle_sun_point_body", self.vehicle_sun_point_body),
        ] {
            for (axis, value) in [("x", vector.x), ("y", vector.y), ("z", vector.z)] {
                fields.push(TelemetryField {
                    path: format!("{name}.{axis}"),
                    value,
                });
            }
        }
        fields
    }
}
