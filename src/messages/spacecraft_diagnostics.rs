use nalgebra::Vector3;

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug)]
pub struct SpacecraftDiagnosticsMsg {
    pub omega_dot_radps2: Vector3<f64>,
    pub non_conservative_accel_body_mps2: Vector3<f64>,
    pub drag_force_body_n: Vector3<f64>,
    pub drag_torque_body_nm: Vector3<f64>,
    pub srp_force_body_n: Vector3<f64>,
    pub srp_force_inertial_n: Vector3<f64>,
    pub srp_torque_body_nm: Vector3<f64>,
    pub orbital_kinetic_energy_j: f64,
    pub rotational_energy_j: f64,
    pub orbital_angular_momentum_inertial_kg_m2ps: Vector3<f64>,
    pub rotational_angular_momentum_inertial_kg_m2ps: Vector3<f64>,
}

impl Default for SpacecraftDiagnosticsMsg {
    fn default() -> Self {
        Self {
            omega_dot_radps2: Vector3::zeros(),
            non_conservative_accel_body_mps2: Vector3::zeros(),
            drag_force_body_n: Vector3::zeros(),
            drag_torque_body_nm: Vector3::zeros(),
            srp_force_body_n: Vector3::zeros(),
            srp_force_inertial_n: Vector3::zeros(),
            srp_torque_body_nm: Vector3::zeros(),
            orbital_kinetic_energy_j: 0.0,
            rotational_energy_j: 0.0,
            orbital_angular_momentum_inertial_kg_m2ps: Vector3::zeros(),
            rotational_angular_momentum_inertial_kg_m2ps: Vector3::zeros(),
        }
    }
}

impl TelemetryMessage for SpacecraftDiagnosticsMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "omega_dot_radps2.x".to_string(),
                value: self.omega_dot_radps2.x,
            },
            TelemetryField {
                path: "omega_dot_radps2.y".to_string(),
                value: self.omega_dot_radps2.y,
            },
            TelemetryField {
                path: "omega_dot_radps2.z".to_string(),
                value: self.omega_dot_radps2.z,
            },
            TelemetryField {
                path: "non_conservative_accel_body_mps2.x".to_string(),
                value: self.non_conservative_accel_body_mps2.x,
            },
            TelemetryField {
                path: "non_conservative_accel_body_mps2.y".to_string(),
                value: self.non_conservative_accel_body_mps2.y,
            },
            TelemetryField {
                path: "non_conservative_accel_body_mps2.z".to_string(),
                value: self.non_conservative_accel_body_mps2.z,
            },
            TelemetryField {
                path: "drag_force_body_n.x".to_string(),
                value: self.drag_force_body_n.x,
            },
            TelemetryField {
                path: "drag_force_body_n.y".to_string(),
                value: self.drag_force_body_n.y,
            },
            TelemetryField {
                path: "drag_force_body_n.z".to_string(),
                value: self.drag_force_body_n.z,
            },
            TelemetryField {
                path: "drag_torque_body_nm.x".to_string(),
                value: self.drag_torque_body_nm.x,
            },
            TelemetryField {
                path: "drag_torque_body_nm.y".to_string(),
                value: self.drag_torque_body_nm.y,
            },
            TelemetryField {
                path: "drag_torque_body_nm.z".to_string(),
                value: self.drag_torque_body_nm.z,
            },
            TelemetryField {
                path: "srp_force_body_n.x".to_string(),
                value: self.srp_force_body_n.x,
            },
            TelemetryField {
                path: "srp_force_body_n.y".to_string(),
                value: self.srp_force_body_n.y,
            },
            TelemetryField {
                path: "srp_force_body_n.z".to_string(),
                value: self.srp_force_body_n.z,
            },
            TelemetryField {
                path: "srp_force_inertial_n.x".to_string(),
                value: self.srp_force_inertial_n.x,
            },
            TelemetryField {
                path: "srp_force_inertial_n.y".to_string(),
                value: self.srp_force_inertial_n.y,
            },
            TelemetryField {
                path: "srp_force_inertial_n.z".to_string(),
                value: self.srp_force_inertial_n.z,
            },
            TelemetryField {
                path: "srp_torque_body_nm.x".to_string(),
                value: self.srp_torque_body_nm.x,
            },
            TelemetryField {
                path: "srp_torque_body_nm.y".to_string(),
                value: self.srp_torque_body_nm.y,
            },
            TelemetryField {
                path: "srp_torque_body_nm.z".to_string(),
                value: self.srp_torque_body_nm.z,
            },
            TelemetryField {
                path: "orbital_kinetic_energy_j".to_string(),
                value: self.orbital_kinetic_energy_j,
            },
            TelemetryField {
                path: "rotational_energy_j".to_string(),
                value: self.rotational_energy_j,
            },
            TelemetryField {
                path: "orbital_angular_momentum_inertial_kg_m2ps.x".to_string(),
                value: self.orbital_angular_momentum_inertial_kg_m2ps.x,
            },
            TelemetryField {
                path: "orbital_angular_momentum_inertial_kg_m2ps.y".to_string(),
                value: self.orbital_angular_momentum_inertial_kg_m2ps.y,
            },
            TelemetryField {
                path: "orbital_angular_momentum_inertial_kg_m2ps.z".to_string(),
                value: self.orbital_angular_momentum_inertial_kg_m2ps.z,
            },
            TelemetryField {
                path: "rotational_angular_momentum_inertial_kg_m2ps.x".to_string(),
                value: self.rotational_angular_momentum_inertial_kg_m2ps.x,
            },
            TelemetryField {
                path: "rotational_angular_momentum_inertial_kg_m2ps.y".to_string(),
                value: self.rotational_angular_momentum_inertial_kg_m2ps.y,
            },
            TelemetryField {
                path: "rotational_angular_momentum_inertial_kg_m2ps.z".to_string(),
                value: self.rotational_angular_momentum_inertial_kg_m2ps.z,
            },
        ]
    }
}
