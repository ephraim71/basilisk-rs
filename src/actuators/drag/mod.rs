use nalgebra::Vector3;
use std::any::Any;

use crate::messages::{AtmosphereMsg, Input, SpacecraftStateMsg};
use crate::spacecraft::{DynamicEffector, EffectorOutput};

#[derive(Clone, Debug)]
pub struct DragConfig {
    pub name: String,
    pub projected_area_m2: f64,
    pub drag_coeff: f64,
    pub com_offset_m: Vector3<f64>,
    pub planet_rotation_rate_radps: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct Drag {
    pub config: DragConfig,
    pub input_atmosphere_msg: Input<AtmosphereMsg>,
}

impl Drag {
    pub fn new(config: DragConfig) -> Self {
        Self {
            config,
            input_atmosphere_msg: Input::default(),
        }
    }

    pub fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let atmosphere = self.input_atmosphere_msg.read();
        if atmosphere.neutral_density_kgpm3 <= 0.0 {
            return EffectorOutput::default();
        }

        let position_inertial = state.position_m;
        let velocity_inertial = state.velocity_mps;
        let relative_velocity_inertial = velocity_inertial
            - self
                .config
                .planet_rotation_rate_radps
                .cross(&position_inertial);
        let relative_speed = relative_velocity_inertial.norm();
        if relative_speed == 0.0 {
            return EffectorOutput::default();
        }

        let body_to_inertial = state.body_to_inertial();
        let inertial_to_body = body_to_inertial.inverse();
        let relative_velocity_body = inertial_to_body.transform_vector(&relative_velocity_inertial);
        let force_body = -0.5
            * self.config.drag_coeff
            * self.config.projected_area_m2
            * atmosphere.neutral_density_kgpm3
            * relative_speed
            * relative_speed
            * relative_velocity_body.normalize();

        EffectorOutput {
            force_inertial_n: body_to_inertial.transform_vector(&force_body),
            torque_body_nm: self.config.com_offset_m.cross(&force_body),
        }
    }
}

impl DynamicEffector for Drag {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        Drag::compute_output(self, state)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Quaternion, UnitQuaternion, Vector3};

    use crate::messages::{AtmosphereMsg, Output, SpacecraftStateMsg};

    use super::{Drag, DragConfig};

    fn make_drag(
        density: f64,
        cd: f64,
        area: f64,
        com_offset: Vector3<f64>,
    ) -> (Drag, Output<AtmosphereMsg>) {
        let atmo_out = Output::new(AtmosphereMsg {
            neutral_density_kgpm3: density,
            local_temp_k: 0.0,
        });
        let mut drag = Drag::new(DragConfig {
            name: "drag".to_string(),
            projected_area_m2: area,
            drag_coeff: cd,
            com_offset_m: com_offset,
            planet_rotation_rate_radps: Vector3::zeros(),
        });
        drag.input_atmosphere_msg.connect(atmo_out.slot());
        (drag, atmo_out)
    }

    fn make_state(sigma_bn: Vector3<f64>, velocity_mps: Vector3<f64>) -> SpacecraftStateMsg {
        SpacecraftStateMsg {
            position_m: Vector3::zeros(),
            velocity_mps,
            sigma_bn,
            omega_radps: Vector3::zeros(),
        }
    }

    #[test]
    fn zero_density_yields_zero_output() {
        let (drag, _atmo) = make_drag(0.0, 1.5, 2.75, Vector3::new(1.0, 0.0, 0.0));
        let state = make_state(Vector3::zeros(), Vector3::new(0.0, 0.5, 0.0));
        let out = drag.compute_output(&state);
        assert_eq!(out.force_inertial_n, Vector3::zeros());
        assert_eq!(out.torque_body_nm, Vector3::zeros());
    }

    /// density=2 kg/m³, Cd=1.5, A=2.75 m², no com offset, no planet rotation.
    /// Force = ½ρv²CdA opposing velocity; no torque.
    #[test]
    fn force_opposes_velocity_identity_attitude() {
        let density = 2.0_f64;
        let cd = 1.5_f64;
        let area = 2.75_f64;
        let v_inertial = Vector3::new(0.0, 0.5, 0.0);

        let (drag, _atmo) = make_drag(density, cd, area, Vector3::zeros());
        let state = make_state(Vector3::zeros(), v_inertial);
        let out = drag.compute_output(&state);

        let v_mag = v_inertial.norm();
        let f_mag = 0.5 * density * v_mag * v_mag * cd * area;
        let expected_force = -f_mag * v_inertial.normalize();
        assert!(
            (out.force_inertial_n - expected_force).norm() < 1e-12,
            "force: expected {expected_force:?}, got {:?}",
            out.force_inertial_n
        );
        assert!(
            out.torque_body_nm.norm() < 1e-12,
            "torque should be zero with no com offset"
        );
    }

    /// v=[0,0.5,0] m/s, density=2, Cd=1.5, A=2.75, r_CP=[1,0,0], no planet rotation.
    /// Expected force (inertial) and torque (body) computed analytically.
    #[test]
    fn force_and_torque_with_mrp_attitude() {
        let density = 2.0_f64;
        let cd = 1.5_f64;
        let area = 2.75_f64;
        let com_offset = Vector3::new(1.0_f64, 0.0, 0.0);
        let v_inertial = Vector3::new(0.0, 0.5, 0.0);

        // MRP sigma_BN = [0.1, 0.2, 0.3] → quaternion q_BN (maps N → B frame).
        // q = [(1-|σ|²)/(1+|σ|²), 2σ/(1+|σ|²)] — Shuster 1993 convention.
        let sigma = Vector3::new(0.1_f64, 0.2, 0.3);
        let s2 = sigma.norm_squared(); // 0.14
        let d = 1.0 + s2;
        let q_bn = UnitQuaternion::new_normalize(Quaternion::new(
            (1.0 - s2) / d,
            2.0 * sigma.x / d,
            2.0 * sigma.y / d,
            2.0 * sigma.z / d,
        ));
        // attitude_b_to_i = q_NB = q_BN⁻¹
        let attitude_b_to_i = q_bn.inverse();

        let (drag, _atmo) = make_drag(density, cd, area, com_offset);
        let state = make_state(sigma, v_inertial);
        let out = drag.compute_output(&state);

        // Reference: same math as Basilisk test_cannonballDrag
        let v_body = q_bn.transform_vector(&v_inertial);
        let v_mag = v_inertial.norm(); // frame-invariant
        let f_mag = 0.5 * density * v_mag * v_mag * cd * area;
        let f_body = -f_mag * v_body.normalize();
        let f_inertial = attitude_b_to_i.transform_vector(&f_body);
        let torque_body = com_offset.cross(&f_body);

        assert!(
            (out.force_inertial_n - f_inertial).norm() < 1e-12,
            "force: expected {f_inertial:?}, got {:?}",
            out.force_inertial_n
        );
        assert!(
            (out.torque_body_nm - torque_body).norm() < 1e-12,
            "torque: expected {torque_body:?}, got {:?}",
            out.torque_body_nm
        );
    }
}
