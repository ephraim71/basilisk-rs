//! Cannonball aerodynamic drag at a site: port of upstream Basilisk
//! `simulation/forceTorque/cannonballDrag`.
//!
//! With the site frame `S` velocity `v_S` relative to the air (inertial
//! velocity, less the air velocity when the wind input is connected), in
//! `S` components,
//!
//! ```text
//! F_S = -½ C_D A |v_S| ρ v_S,    τ_S = r_CP/S × F_S,
//! ```
//!
//! with the drag coefficient, projected area and center-of-pressure offset
//! from the drag geometry input and the density from the atmosphere input.
//! The force and torque drive a force-torque actuator at the site;
//! [`CannonballDrag::apply_to_body`], [`apply_to_site`](CannonballDrag::apply_to_site)
//! and [`apply_to_actuator`](CannonballDrag::apply_to_actuator) wire it
//! (upstream's Python `applyTo`). It is a [`Module`] and, to act at every
//! integrator stage, a [`SceneModel`].
//!
//! Upstream also publishes C-wrapped copies of its outputs; ports serve
//! every consumer, so there is one output each.

use super::actuator::MjForceTorqueActuator;
use super::scene::MjScene;
use super::task::{SceneModel, SceneView, TaskContext};
use crate::messages::{
    AtmosphereMsg, BodyForceCommandMsg, BodyTorqueCommandMsg, DragGeometryMsg, Input, Output,
    SpacecraftStateMsg, WindMsg,
};
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Upstream `CannonballDrag`.
#[derive(Debug)]
pub struct CannonballDrag {
    /// Upstream `ModelTag`.
    pub name: String,
    /// Upstream `dragGeometryInMsg`.
    pub drag_geometry_in_msg: Input<DragGeometryMsg>,
    /// Upstream `atmoDensInMsg`: the neutral density.
    pub atmo_dens_in_msg: Input<AtmosphereMsg>,
    /// Upstream `windVelInMsg`: the air velocity, optional.
    pub wind_vel_in_msg: Input<WindMsg>,
    /// Upstream `referenceFrameStateInMsg`: the state of the site frame `S`.
    pub reference_frame_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `forceOutMsg` (`force_S` \[N\]).
    pub force_out_msg: Output<BodyForceCommandMsg>,
    /// Upstream `torqueOutMsg` (`torque_S` \[N m\]).
    pub torque_out_msg: Output<BodyTorqueCommandMsg>,
}

impl Default for CannonballDrag {
    fn default() -> Self {
        Self {
            name: "cannonballDrag".to_string(),
            drag_geometry_in_msg: Input::default(),
            atmo_dens_in_msg: Input::default(),
            wind_vel_in_msg: Input::default(),
            reference_frame_state_in_msg: Input::default(),
            force_out_msg: Output::default(),
            torque_out_msg: Output::default(),
        }
    }
}

impl CannonballDrag {
    /// Upstream `applyTo(MJBody)`: the drag acts at the body's center of
    /// mass site.
    pub fn apply_to_body(&mut self, scene: &mut MjScene, body: &str) {
        let site = scene.body(body).center_of_mass().name().to_string();
        self.apply_to_site(scene, &site);
    }

    /// Upstream `applyTo(MJSite)`: reads the site state and drives a new
    /// force-torque actuator at the site, `<name>_cannonballDrag_<site>`.
    pub fn apply_to_site(&mut self, scene: &mut MjScene, site: &str) {
        scene
            .site(site)
            .state_out_msg
            .connect_to(&mut self.reference_frame_state_in_msg);
        let actuator =
            scene.add_force_torque_actuator(&format!("{}_cannonballDrag_{site}", self.name), site);
        self.apply_to_actuator(actuator);
    }

    /// Upstream `applyTo(MJForceTorqueActuator)`: drives an existing
    /// actuator (the reference-frame input is the caller's to connect).
    pub fn apply_to_actuator(&self, actuator: &mut MjForceTorqueActuator) {
        self.force_out_msg.connect_to(&mut actuator.force_in_msg);
        self.torque_out_msg.connect_to(&mut actuator.torque_in_msg);
    }

    /// Upstream `Reset`.
    ///
    /// # Panics
    ///
    /// If the atmosphere, geometry or reference-frame input is unconnected
    /// (upstream raises `BasiliskError`).
    fn check_inputs(&self) {
        assert!(
            self.atmo_dens_in_msg.is_connected(),
            "CannonballDrag: atmoDensInMsg is not linked."
        );
        assert!(
            self.drag_geometry_in_msg.is_connected(),
            "CannonballDrag: dragGeometryInMsg is not linked."
        );
        assert!(
            self.reference_frame_state_in_msg.is_connected(),
            "CannonballDrag: referenceFrameStateInMsg is not linked."
        );
    }

    /// Upstream `UpdateState`.
    fn compute(&self) {
        let density = self.atmo_dens_in_msg.read().neutral_density_kgpm3;
        let geometry = self.drag_geometry_in_msg.read();
        let site = self.reference_frame_state_in_msg.read();
        let dcm_sn = mrp_to_dcm(site.sigma_bn);
        let velocity_n = if self.wind_vel_in_msg.is_connected() {
            site.velocity_mps - self.wind_vel_in_msg.read().air_velocity_inertial_mps
        } else {
            site.velocity_mps
        };
        let velocity_s = dcm_sn * velocity_n;
        let force_s = -0.5
            * geometry.drag_coeff
            * geometry.projected_area_m2
            * velocity_s.norm()
            * density
            * velocity_s;
        self.force_out_msg.write(BodyForceCommandMsg {
            force_request_body_n: force_s,
        });
        self.torque_out_msg.write(BodyTorqueCommandMsg {
            torque_request_body_nm: geometry.r_cp_s_m.cross(&force_s),
        });
    }
}

impl Module for CannonballDrag {
    fn init(&mut self) {
        self.check_inputs();
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.compute();
    }
}

impl SceneModel for CannonballDrag {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn scene_reset(&mut self, _scene: &SceneView<'_>, _context: &SimulationContext) {
        self.check_inputs();
    }

    fn scene_update(&mut self, _context: &mut TaskContext<'_>) {
        self.compute();
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_cannonballDrag.py`.

    use std::f64::consts::PI;

    use nalgebra::Vector3;

    use super::*;
    use crate::dynamics::gravity::PointMassGravityModel;
    use crate::environment::atmosphere::exponential_atmosphere::{
        ExponentialAtmosphere, ExponentialAtmosphereConfig,
    };
    use crate::mujoco_dynamics::n_body_gravity::NBodyGravity;
    use crate::mujoco_dynamics::tests::{context, run, sec2nano};
    use crate::utilities::astro_constants::{MU_EARTH, MU_MARS, REQ_EARTH, REQ_MARS};
    use crate::utilities::orbital_motion::{ClassicElements, elements_to_state};

    fn assert_close(actual: Vector3<f64>, expected: Vector3<f64>) {
        assert!(
            (actual - expected).abs().max() <= 1e-12,
            "{actual} vs {expected}"
        );
    }

    fn drag_with(
        density: f64,
        geometry: DragGeometryMsg,
        site: SpacecraftStateMsg,
    ) -> (CannonballDrag, [Box<dyn std::any::Any>; 3]) {
        let mut drag = CannonballDrag::default();
        let atmosphere = Output::new(AtmosphereMsg {
            neutral_density_kgpm3: density,
            ..AtmosphereMsg::default()
        });
        let geometry = Output::new(geometry);
        let site = Output::new(site);
        atmosphere.connect_to(&mut drag.atmo_dens_in_msg);
        geometry.connect_to(&mut drag.drag_geometry_in_msg);
        site.connect_to(&mut drag.reference_frame_state_in_msg);
        (
            drag,
            [Box::new(atmosphere), Box::new(geometry), Box::new(site)],
        )
    }

    /// Upstream `test_cannonballDrag`: the force is `½ ρ v² C_D A` against
    /// the flow in the site frame, and the torque `r_CP × F`.
    #[test]
    fn force_and_torque_match_the_cannonball_formula() {
        let (density, cd, area) = (2.0, 1.5, 2.75);
        let r_cp_s = Vector3::new(1.0, 0.0, 0.0);
        let velocity = Vector3::new(0.0, 0.5, 0.0);
        let sigma = Vector3::new(0.1, 0.2, 0.3);
        let (mut drag, _keep) = drag_with(
            density,
            DragGeometryMsg {
                projected_area_m2: area,
                drag_coeff: cd,
                r_cp_s_m: r_cp_s,
            },
            SpacecraftStateMsg {
                sigma_bn: sigma,
                velocity_mps: velocity,
                ..SpacecraftStateMsg::default()
            },
        );
        Module::init(&mut drag);
        Module::update(&mut drag, &context(sec2nano(1.0)));

        let v_s = mrp_to_dcm(sigma) * velocity;
        let force = -0.5 * density * velocity.norm_squared() * cd * area * v_s.normalize();
        assert_close(drag.force_out_msg.read().force_request_body_n, force);
        assert_close(
            drag.torque_out_msg.read().torque_request_body_nm,
            r_cp_s.cross(&force),
        );
    }

    /// Upstream `test_cannonballDragAtmosphereRelativeVelocityWhenWindLinked`.
    #[test]
    fn uses_the_air_relative_velocity_with_wind() {
        let (density, cd, area) = (1.2, 2.2, 1.0);
        let velocity = Vector3::new(0.0, 7600.0, 0.0);
        let air = Vector3::new(0.0, 510.44, 0.0);
        let (mut drag, _keep) = drag_with(
            density,
            DragGeometryMsg {
                projected_area_m2: area,
                drag_coeff: cd,
                r_cp_s_m: Vector3::zeros(),
            },
            SpacecraftStateMsg {
                velocity_mps: velocity,
                ..SpacecraftStateMsg::default()
            },
        );
        let wind = Output::new(WindMsg {
            air_velocity_inertial_mps: air,
            ..WindMsg::default()
        });
        wind.connect_to(&mut drag.wind_vel_in_msg);
        Module::init(&mut drag);
        Module::update(&mut drag, &context(sec2nano(1.0)));
        let relative = velocity - air;
        let expected = -0.5 * density * relative.norm_squared() * cd * area * relative.normalize();
        assert_close(drag.force_out_msg.read().force_request_body_n, expected);
    }

    /// Upstream `test_orbit`: over one orbit of a point-mass body (three
    /// slide joints) about Earth and Mars, circular (LPO) and elliptic
    /// (LTO), with an exponential atmosphere, the drag force on the
    /// actuator matches the cannonball formula of the recorded state at
    /// every step.
    #[test]
    fn orbit_drag_matches_the_reference_at_every_step() {
        for planet in ["earth", "mars"] {
            for orbit in ["LPO", "LTO"] {
                orbit_case(planet, orbit);
            }
        }
    }

    fn orbit_case(planet: &str, orbit: &str) {
        let (mu, radius, base_density, scale_height) = match planet {
            "earth" => (MU_EARTH * 1e9, REQ_EARTH * 1e3, 1.217, 8500.0),
            _ => (MU_MARS * 1e9, REQ_MARS * 1e3, 0.020, 11100.0),
        };
        let mut scene = MjScene::new(
            r#"
            <mujoco>
                <worldbody>
                    <body name="satellite">
                        <joint name="chief_x" axis="1 0 0" type="slide"/>
                        <joint name="chief_y" axis="0 1 0" type="slide"/>
                        <joint name="chief_z" axis="0 0 1" type="slide"/>
                        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("scene");
        scene.extra_eom_call = true;

        let mut gravity = NBodyGravity::new("gravity");
        gravity.add_gravity_source(planet, PointMassGravityModel::new(mu).expect("mu"), true);
        gravity.add_gravity_targets_for_all_bodies(&mut scene);
        scene.add_model_to_dynamics_task(gravity, -1);

        let mut atmosphere = ExponentialAtmosphere::new(ExponentialAtmosphereConfig {
            name: "ExpAtmo".to_string(),
            planet_radius_m: radius,
            reference_altitude_m: 0.0,
            reference_density_kgpm3: base_density,
            scale_height_m: scale_height,
        });
        let com_state = scene
            .body("satellite")
            .center_of_mass()
            .state_out_msg
            .clone();
        com_state.connect_to(&mut atmosphere.input_state_msg);
        let density_out = atmosphere.output_atmosphere_msg.clone();
        scene.add_module_to_dynamics_task("ExpAtmo", atmosphere, -1);

        let (projected_area, drag_coeff) = (10.0, 2.2);
        let geometry = Output::new(DragGeometryMsg {
            projected_area_m2: projected_area,
            drag_coeff,
            r_cp_s_m: Vector3::new(1.0, 0.0, 0.0),
        });
        let mut drag = CannonballDrag {
            name: "drag".to_string(),
            ..CannonballDrag::default()
        };
        drag.apply_to_body(&mut scene, "satellite");
        density_out.connect_to(&mut drag.atmo_dens_in_msg);
        geometry.connect_to(&mut drag.drag_geometry_in_msg);
        let force_out = drag.force_out_msg.clone();
        scene.add_model_to_dynamics_task(drag, -1);

        let (altitude_min, altitude_max) = match orbit {
            "LPO" => (300e3, 300e3),
            _ => (300e3, 800e3),
        };
        let (r_min, r_max) = (radius + altitude_min, radius + altitude_max);
        let a = (r_min + r_max) / 2.0;
        let elements = ClassicElements {
            semi_major_axis: a,
            eccentricity: 1.0 - r_min / a,
            ..ClassicElements::default()
        };
        let period = 2.0 * PI / (mu / a.powi(3)).sqrt();
        scene.init();
        let (r, v) = elements_to_state(mu, &elements);
        scene.body_mut("satellite").set_position(r);
        scene.body_mut("satellite").set_velocity(v);

        let mut steps = 0;
        run(&mut scene, sec2nano(1.0), sec2nano(period), |_, _| {
            let state = com_state.read();
            let density = density_out.read().neutral_density_kgpm3;
            let speed = state.velocity_mps.norm();
            let reference = if speed > 0.0 {
                0.5 * drag_coeff
                    * density
                    * projected_area
                    * speed
                    * speed
                    * (mrp_to_dcm(state.sigma_bn) * (-state.velocity_mps / speed))
            } else {
                Vector3::zeros()
            };
            assert_close(force_out.read().force_request_body_n, reference);
            steps += 1;
        });
        assert!(steps > 5000, "{planet} {orbit}: {steps} steps");
    }
}
