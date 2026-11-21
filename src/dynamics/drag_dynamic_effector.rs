use nalgebra::Vector3;
use std::any::Any;

use crate::messages::{AtmosphereMsg, Input, SpacecraftStateMsg, WindMsg};
use crate::spacecraft::{DynamicEffector, EffectorOutput};

#[derive(Clone, Debug)]
pub struct DragDynamicEffectorConfig {
    pub name: String,
    pub projected_area_m2: f64,
    pub drag_coeff: f64,
    pub com_offset_m: Vector3<f64>,
    /// Planet spin rate used to model a rigidly co-rotating atmosphere via
    /// `v_air = omega x r`. NOTE: assumes `state.position_m` is planet-centered
    /// (origin at the planet's rotation center). In a non-planet-centered frame
    /// this term is taken about the wrong point. Ignored while
    /// `input_wind_msg` is connected, which supplies the air velocity instead
    /// (for example from a wind model in `environment::wind`). Set to zero for
    /// a non-rotating (inertial-velocity) atmosphere.
    pub planet_rotation_rate_radps: Vector3<f64>,
}

impl DragDynamicEffectorConfig {
    /// Check physical and numeric invariants. Returns a description of the first
    /// violation, or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        if !self.projected_area_m2.is_finite() || self.projected_area_m2 < 0.0 {
            return Err(format!(
                "projected_area_m2 must be finite and >= 0, got {}",
                self.projected_area_m2
            ));
        }
        if !self.drag_coeff.is_finite() || self.drag_coeff < 0.0 {
            return Err(format!(
                "drag_coeff must be finite and >= 0, got {}",
                self.drag_coeff
            ));
        }
        if !self.com_offset_m.iter().all(|v| v.is_finite()) {
            return Err(format!(
                "com_offset_m must be finite, got {:?}",
                self.com_offset_m
            ));
        }
        if !self
            .planet_rotation_rate_radps
            .iter()
            .all(|v| v.is_finite())
        {
            return Err(format!(
                "planet_rotation_rate_radps must be finite, got {:?}",
                self.planet_rotation_rate_radps
            ));
        }
        Ok(())
    }
}

/// The air velocity drag acts against, inertial components \[m/s\]: the
/// wind model's `air_velocity_inertial_mps` when `wind` is connected
/// (upstream `windVelInMsg`), else a rigidly co-rotating atmosphere
/// `planet_rotation_rate_radps x position`.
pub(crate) fn air_velocity_inertial(
    wind: &Input<WindMsg>,
    planet_rotation_rate_radps: Vector3<f64>,
    position_inertial_m: Vector3<f64>,
) -> Vector3<f64> {
    if wind.is_connected() {
        wind.read().air_velocity_inertial_mps
    } else {
        planet_rotation_rate_radps.cross(&position_inertial_m)
    }
}

#[derive(Clone, Debug)]
pub struct DragDynamicEffector {
    pub config: DragDynamicEffectorConfig,
    pub input_atmosphere_msg: Input<AtmosphereMsg>,
    /// Upstream `windVelInMsg` (optional): the local air velocity. When
    /// connected, drag acts against the velocity relative to
    /// `air_velocity_inertial_mps` instead of the co-rotation term of
    /// `planet_rotation_rate_radps`.
    pub input_wind_msg: Input<WindMsg>,
}

impl DragDynamicEffector {
    pub fn new(config: DragDynamicEffectorConfig) -> Self {
        if let Err(msg) = config.validate() {
            panic!("invalid DragDynamicEffectorConfig: {msg}");
        }
        Self {
            config,
            input_atmosphere_msg: Input::default(),
            input_wind_msg: Input::default(),
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
            - air_velocity_inertial(
                &self.input_wind_msg,
                self.config.planet_rotation_rate_radps,
                position_inertial,
            );
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

impl DynamicEffector for DragDynamicEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        DragDynamicEffector::compute_output(self, state)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Quaternion, UnitQuaternion, Vector3};

    use crate::messages::{AtmosphereMsg, Output, SpacecraftStateMsg};

    use super::{DragDynamicEffector, DragDynamicEffectorConfig};

    fn make_drag(
        density: f64,
        cd: f64,
        area: f64,
        com_offset: Vector3<f64>,
    ) -> (DragDynamicEffector, Output<AtmosphereMsg>) {
        let atmo_out = Output::new(AtmosphereMsg {
            neutral_density_kgpm3: density,
            local_temp_k: 0.0,
        });
        let mut drag = DragDynamicEffector::new(DragDynamicEffectorConfig {
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
            ..Default::default()
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

        // MRP sigma_BN = [0.1, 0.2, 0.3] -> quaternion q_NB (maps B -> N frame).
        // q = [(1-|σ|²)/(1+|σ|²), 2σ/(1+|σ|²)] — Shuster 1993 convention.
        let sigma = Vector3::new(0.1_f64, 0.2, 0.3);
        let s2 = sigma.norm_squared(); // 0.14
        let d = 1.0 + s2;
        let q_nb = UnitQuaternion::new_normalize(Quaternion::new(
            (1.0 - s2) / d,
            2.0 * sigma.x / d,
            2.0 * sigma.y / d,
            2.0 * sigma.z / d,
        ));

        let (drag, _atmo) = make_drag(density, cd, area, com_offset);
        let state = make_state(sigma, v_inertial);
        let out = drag.compute_output(&state);

        // Cannonball drag: F = -0.5 * rho * Cd * A * |v| * v, in the body frame
        let v_body = q_nb.inverse().transform_vector(&v_inertial);
        let v_mag = v_inertial.norm(); // frame-invariant
        let f_mag = 0.5 * density * v_mag * v_mag * cd * area;
        let f_body = -f_mag * v_body.normalize();
        let f_inertial = q_nb.transform_vector(&f_body);
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

    /// With a rotating planet the drag opposes the *atmosphere-relative* velocity
    /// v_rel = v - (omega x r), not the inertial velocity.
    #[test]
    fn planet_rotation_produces_atmosphere_relative_drag() {
        let density = 2.0_f64;
        let cd = 1.5_f64;
        let area = 2.75_f64;
        let omega = Vector3::new(0.0, 0.0, 7.29e-5); // planet spin [rad/s]
        let position = Vector3::new(7.0e6, 0.0, 0.0);
        let velocity = Vector3::new(0.0, 7600.0, 0.0);

        let atmo_out = Output::new(AtmosphereMsg {
            neutral_density_kgpm3: density,
            local_temp_k: 0.0,
        });
        let mut drag = DragDynamicEffector::new(DragDynamicEffectorConfig {
            name: "drag".to_string(),
            projected_area_m2: area,
            drag_coeff: cd,
            com_offset_m: Vector3::zeros(),
            planet_rotation_rate_radps: omega,
        });
        drag.input_atmosphere_msg.connect(atmo_out.slot());

        // Identity attitude, so body frame == inertial frame.
        let state = make_state(Vector3::zeros(), velocity);
        let state = SpacecraftStateMsg {
            position_m: position,
            ..state
        };
        let out = drag.compute_output(&state);

        let v_rel = velocity - omega.cross(&position);
        let f_mag = 0.5 * density * v_rel.norm() * v_rel.norm() * cd * area;
        let expected = -f_mag * v_rel.normalize();

        // Sanity: co-rotation actually changed the relative velocity.
        assert!(
            (v_rel - velocity).norm() > 1.0,
            "omega x r should be non-trivial"
        );

        let rel_err = (out.force_inertial_n - expected).norm() / expected.norm();
        assert!(
            rel_err < 1e-12,
            "force: expected {expected:?}, got {:?}",
            out.force_inertial_n
        );
    }

    /// Upstream `test_drag_wind_velocity_with_wind_message`: with a wind
    /// model connected, drag opposes the velocity relative to its air
    /// velocity, and the co-rotation rate of the config is ignored.
    #[test]
    fn connected_wind_sets_the_air_velocity() {
        let density = 1.217 * (-(7000e3 - 6371e3) / 8500.0_f64).exp();
        let (cd, area) = (2.2, 1.0);
        let position = Vector3::new(7000e3, 0.0, 0.0);
        let velocity = Vector3::new(0.0, 7600.0, 0.0);
        let wind = zero_wind_at(position);

        let atmo_out = Output::new(AtmosphereMsg {
            neutral_density_kgpm3: density,
            local_temp_k: 0.0,
        });
        let mut drag = DragDynamicEffector::new(DragDynamicEffectorConfig {
            name: "drag".to_string(),
            projected_area_m2: area,
            drag_coeff: cd,
            com_offset_m: Vector3::zeros(),
            planet_rotation_rate_radps: Vector3::new(0.0, 0.0, 1.0),
        });
        drag.input_atmosphere_msg.connect(atmo_out.slot());
        drag.input_wind_msg.connect(wind.output_wind_msg.slot());

        let state = SpacecraftStateMsg {
            position_m: position,
            ..make_state(Vector3::zeros(), velocity)
        };
        let out = drag.compute_output(&state);

        let v_rel = velocity - wind.output_wind_msg.read().air_velocity_inertial_mps;
        assert!((v_rel - velocity).norm() > 1.0, "the wind should matter");
        let expected = -0.5 * cd * v_rel.norm().powi(2) * area * density * v_rel.normalize();
        assert!(
            (out.force_inertial_n - expected).norm() < 1e-10,
            "force: expected {expected:?}, got {:?}",
            out.force_inertial_n
        );
    }

    /// The zero-wind model's air velocity at `position` with Earth's rotation
    /// rate (upstream's wind tests link a `ZeroWindModel`).
    fn zero_wind_at(
        position: Vector3<f64>,
    ) -> crate::environment::wind::zero_wind_model::ZeroWindModel {
        use crate::environment::wind::zero_wind_model::ZeroWindModel;
        use crate::messages::PlanetStateMsg;
        use crate::utilities::astro_constants::OMEGA_EARTH;
        use crate::{Module, SimulationContext};

        let spacecraft = Output::new(SpacecraftStateMsg {
            position_m: position,
            ..Default::default()
        });
        let planet = Output::new(PlanetStateMsg::default());
        let mut wind = ZeroWindModel::named("wind");
        wind.set_planet_omega_n(Vector3::new(0.0, 0.0, OMEGA_EARTH));
        wind.input_state_msg.connect(spacecraft.slot());
        wind.input_planet_msg.connect(planet.slot());
        wind.init();
        wind.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: hifitime::Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });
        wind
    }

    #[test]
    #[should_panic(expected = "drag_coeff")]
    fn rejects_negative_drag_coeff() {
        let _ = make_drag(1.0, -0.5, 2.75, Vector3::zeros());
    }
}
