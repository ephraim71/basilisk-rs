use nalgebra::{Matrix3, Vector3};

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug)]
pub struct SpacecraftMassPropsMsg {
    pub mass_kg: f64,
    pub center_of_mass_body_m: Vector3<f64>,
    pub inertia_about_point_b_body_kg_m2: Matrix3<f64>,
}

impl Default for SpacecraftMassPropsMsg {
    fn default() -> Self {
        Self {
            mass_kg: 0.0,
            center_of_mass_body_m: Vector3::zeros(),
            inertia_about_point_b_body_kg_m2: Matrix3::zeros(),
        }
    }
}

impl TelemetryMessage for SpacecraftMassPropsMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "mass_kg".to_string(),
                value: self.mass_kg,
            },
            TelemetryField {
                path: "center_of_mass_body_m.x".to_string(),
                value: self.center_of_mass_body_m.x,
            },
            TelemetryField {
                path: "center_of_mass_body_m.y".to_string(),
                value: self.center_of_mass_body_m.y,
            },
            TelemetryField {
                path: "center_of_mass_body_m.z".to_string(),
                value: self.center_of_mass_body_m.z,
            },
            TelemetryField {
                path: "inertia_about_point_b_body_kg_m2.m11".to_string(),
                value: self.inertia_about_point_b_body_kg_m2[(0, 0)],
            },
            TelemetryField {
                path: "inertia_about_point_b_body_kg_m2.m12".to_string(),
                value: self.inertia_about_point_b_body_kg_m2[(0, 1)],
            },
            TelemetryField {
                path: "inertia_about_point_b_body_kg_m2.m13".to_string(),
                value: self.inertia_about_point_b_body_kg_m2[(0, 2)],
            },
            TelemetryField {
                path: "inertia_about_point_b_body_kg_m2.m21".to_string(),
                value: self.inertia_about_point_b_body_kg_m2[(1, 0)],
            },
            TelemetryField {
                path: "inertia_about_point_b_body_kg_m2.m22".to_string(),
                value: self.inertia_about_point_b_body_kg_m2[(1, 1)],
            },
            TelemetryField {
                path: "inertia_about_point_b_body_kg_m2.m23".to_string(),
                value: self.inertia_about_point_b_body_kg_m2[(1, 2)],
            },
            TelemetryField {
                path: "inertia_about_point_b_body_kg_m2.m31".to_string(),
                value: self.inertia_about_point_b_body_kg_m2[(2, 0)],
            },
            TelemetryField {
                path: "inertia_about_point_b_body_kg_m2.m32".to_string(),
                value: self.inertia_about_point_b_body_kg_m2[(2, 1)],
            },
            TelemetryField {
                path: "inertia_about_point_b_body_kg_m2.m33".to_string(),
                value: self.inertia_about_point_b_body_kg_m2[(2, 2)],
            },
        ]
    }
}
