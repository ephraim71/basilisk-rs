//! Gravity of several bodies on the bodies of a scene: port of upstream
//! Basilisk `simulation/mujocoDynamics/NBodyGravity`.
//!
//! Each **source** is a gravity model (any basilisk-rs
//! [`GravityModel`]: point mass, spherical harmonics, polyhedron) with an
//! optional ephemeris input giving its position and orientation. Each
//! **target** reads a state (normally a scene body's center-of-mass site)
//! and a mass, and publishes the gravity force on it in its own frame,
//! `F_B = m [BN] g_N`, which drives a force actuator at that site:
//!
//! ```text
//! g_N = Σ_s [s→N] g_s([N→s](r - r_s))  -  Σ_{s non-central} [s→N] g_s([N→s](r_c - r_s))
//! ```
//!
//! The inertial frame is centered on the central source when there is one
//! (the second sum is the central body's own acceleration by the other
//! sources), and target positions are relative to it. A source without an
//! ephemeris sits at the origin with the inertial orientation, as does one
//! whose ephemeris carries no orientation (upstream's zero `J20002Pfix`).
//!
//! The model runs in a scene's dynamics task, so gravity follows every
//! integrator stage; an ephemeris that must follow the stages too goes in
//! the task ahead of it. Sources and targets are evaluated in name order,
//! as upstream's `std::map`.

use std::collections::BTreeMap;

use nalgebra::{Matrix3, Vector3};

use super::scene::MjScene;
use super::task::{SceneModel, SceneView, TaskContext};
use crate::dynamics::gravity::GravityModel;
use crate::messages::{
    BodyForceCommandMsg, Input, Output, PlanetStateMsg, SpacecraftMassPropsMsg, SpacecraftStateMsg,
};
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// A source of gravity (upstream `GravitySource`).
#[derive(Debug)]
pub struct GravitySource {
    model: Box<dyn GravityModel>,
    /// Upstream `isCentralBody`.
    pub is_central_body: bool,
    /// Upstream `stateInMsg`: the source's ephemeris. Unconnected, the
    /// source sits at the origin, inertially aligned.
    pub state_in_msg: Input<PlanetStateMsg>,
}

/// A body the sources pull on (upstream `GravityTarget`).
#[derive(Debug, Default)]
pub struct GravityTarget {
    /// Upstream `massPropertiesInMsg`: the target mass (`mass_kg`).
    pub mass_properties_in_msg: Input<SpacecraftMassPropsMsg>,
    /// Upstream `centerOfMassStateInMsg`: the target's position and
    /// attitude.
    pub center_of_mass_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `massFixedForceOutMsg` (`force_S` \[N\]): the gravity force in
    /// the target frame.
    pub mass_fixed_force_out_msg: Output<BodyForceCommandMsg>,
}

/// Upstream `NBodyGravity`.
#[derive(Debug)]
pub struct NBodyGravity {
    /// Upstream `ModelTag`, which also names the force actuators it adds.
    pub name: String,
    sources: BTreeMap<String, GravitySource>,
    targets: BTreeMap<String, GravityTarget>,
}

impl NBodyGravity {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            sources: BTreeMap::new(),
            targets: BTreeMap::new(),
        }
    }

    /// Upstream `addGravitySource(name, model, isCentralBody)`.
    ///
    /// # Panics
    ///
    /// If a source with that name exists.
    pub fn add_gravity_source(
        &mut self,
        name: &str,
        model: impl GravityModel + 'static,
        is_central_body: bool,
    ) -> &mut GravitySource {
        assert!(
            !self.sources.contains_key(name),
            "Cannot use repeated source name: {name}"
        );
        self.sources
            .entry(name.to_string())
            .or_insert(GravitySource {
                model: Box::new(model),
                is_central_body,
                state_in_msg: Input::default(),
            })
    }

    /// Upstream `addGravityTarget(name)`: a target whose inputs the caller
    /// connects.
    ///
    /// # Panics
    ///
    /// If a target with that name exists.
    pub fn add_gravity_target(&mut self, name: &str) -> &mut GravityTarget {
        assert!(
            !self.targets.contains_key(name),
            "Cannot use repeated target name: {name}"
        );
        self.targets.entry(name.to_string()).or_default()
    }

    /// Upstream `addGravityTarget(name, site)`: a target at `site` of
    /// `scene`, reading the site state and driving a new force actuator
    /// there (`<name>_gravity_target_at_<target>`). The mass input is the
    /// caller's to connect.
    pub fn add_gravity_target_at_site(
        &mut self,
        scene: &mut MjScene,
        name: &str,
        site: &str,
    ) -> &mut GravityTarget {
        let actuator = format!("{}_gravity_target_at_{name}", self.name);
        let site_state = scene.site(site).state_out_msg.clone();
        let target = self.add_gravity_target(name);
        site_state.connect_to(&mut target.center_of_mass_state_in_msg);
        target
            .mass_fixed_force_out_msg
            .connect_to(&mut scene.add_force_actuator(&actuator, site).force_in_msg);
        target
    }

    /// Upstream `addGravityTarget(name, body)`: a target at the body's
    /// center of mass, with the body's mass.
    pub fn add_gravity_target_at_body(
        &mut self,
        scene: &mut MjScene,
        name: &str,
        body: &str,
    ) -> &mut GravityTarget {
        let (site, mass) = {
            let body = scene.body(body);
            (
                body.center_of_mass().name().to_string(),
                body.mass_properties_out_msg.clone(),
            )
        };
        let target = self.add_gravity_target_at_site(scene, name, &site);
        mass.connect_to(&mut target.mass_properties_in_msg);
        target
    }

    /// Every scene body as a target named after it, as upstream's
    /// `gravBodyFactory.addBodiesTo(scene)` sets them up (gravity at each
    /// body's center of mass keeps the gravity gradient across a
    /// multibody).
    pub fn add_gravity_targets_for_all_bodies(&mut self, scene: &mut MjScene) {
        for body in scene.body_names() {
            self.add_gravity_target_at_body(scene, &body, &body);
        }
    }

    /// Upstream `getGravitySource`.
    pub fn gravity_source(&self, name: &str) -> &GravitySource {
        &self.sources[name]
    }

    /// Mutable form of [`gravity_source`](Self::gravity_source), to connect
    /// its ephemeris.
    pub fn gravity_source_mut(&mut self, name: &str) -> &mut GravitySource {
        self.sources.get_mut(name).expect("unknown gravity source")
    }

    /// Upstream `getGravityTarget`.
    pub fn gravity_target(&self, name: &str) -> &GravityTarget {
        &self.targets[name]
    }

    /// Mutable form of [`gravity_target`](Self::gravity_target).
    pub fn gravity_target_mut(&mut self, name: &str) -> &mut GravityTarget {
        self.targets.get_mut(name).expect("unknown gravity target")
    }

    /// Upstream `Reset`: every target connected, at most one central
    /// source, and an ephemeris for every source when there are several.
    ///
    /// # Panics
    ///
    /// On those violations (upstream raises `BasiliskError`).
    fn check(&self) {
        let prefix = format!("In NBodyGravity '{}': ", self.name);
        for (name, target) in &self.targets {
            assert!(
                target.center_of_mass_state_in_msg.is_connected(),
                "{prefix}Target '{name}' centerOfMassStateInMsg is not linked!"
            );
            assert!(
                target.mass_properties_in_msg.is_connected(),
                "{prefix}Target '{name}' massPropertiesInMsg is not linked!"
            );
        }
        let mut central = false;
        for (name, source) in &self.sources {
            assert!(
                self.sources.len() == 1 || source.state_in_msg.is_connected(),
                "{prefix}Source '{name}' has no ephemeris input but there are multiple gravity \
                 sources!"
            );
            if source.is_central_body {
                assert!(!central, "{prefix}More than one central body!");
                central = true;
            }
        }
    }

    /// The source's ephemeris: position and the inertial-to-fixed rotation.
    fn source_frame(source: &GravitySource) -> (Vector3<f64>, Matrix3<f64>) {
        if !source.state_in_msg.is_connected() {
            return (Vector3::zeros(), Matrix3::identity());
        }
        let state = source.state_in_msg.read();
        let orientation = state
            .orientation
            .map(|orientation| orientation.inertial_to_fixed)
            .filter(|dcm| *dcm != Matrix3::zeros())
            .unwrap_or_else(Matrix3::identity);
        (state.position_inertial_m, orientation)
    }

    /// Upstream `computeAccelerationFromSource` \[m/s^2\], inertial components,
    /// at `position` relative to the ephemeris origin.
    fn acceleration_from_source(
        name: &str,
        source: &mut GravitySource,
        position: Vector3<f64>,
    ) -> Vector3<f64> {
        let (source_position, inertial_to_fixed) = Self::source_frame(source);
        let position_fixed = inertial_to_fixed * (position - source_position);
        let gravity_fixed = source
            .model
            .acceleration_mps2(position_fixed)
            .unwrap_or_else(|error| panic!("NBodyGravity source '{name}': {error}"));
        inertial_to_fixed.transpose() * gravity_fixed
    }

    /// Upstream `computeAccelerationOnTarget` \[m/s^2\], inertial components.
    fn acceleration_on_target(&mut self, position: Vector3<f64>) -> Vector3<f64> {
        let central = self
            .sources
            .values()
            .find(|source| source.is_central_body)
            .map(|source| Self::source_frame(source).0);
        let central_position = central.unwrap_or_else(Vector3::zeros);
        let target_position = position + central_position;
        let mut gravity = Vector3::zeros();
        for (name, source) in &mut self.sources {
            gravity += Self::acceleration_from_source(name, source, target_position);
            if central.is_some() && !source.is_central_body {
                gravity -= Self::acceleration_from_source(name, source, central_position);
            }
        }
        gravity
    }

    /// Upstream `UpdateState`.
    fn apply(&mut self) {
        let states: Vec<(String, SpacecraftStateMsg, f64)> = self
            .targets
            .iter()
            .map(|(name, target)| {
                (
                    name.clone(),
                    target.center_of_mass_state_in_msg.read(),
                    target.mass_properties_in_msg.read().mass_kg,
                )
            })
            .collect();
        for (name, state, mass) in states {
            let gravity_n = self.acceleration_on_target(state.position_m);
            let force_b = mass * (mrp_to_dcm(state.sigma_bn) * gravity_n);
            self.targets[&name]
                .mass_fixed_force_out_msg
                .write(BodyForceCommandMsg {
                    force_request_body_n: force_b,
                });
        }
    }
}

impl SceneModel for NBodyGravity {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn scene_reset(&mut self, _scene: &SceneView<'_>, _context: &SimulationContext) {
        self.check();
    }

    fn scene_update(&mut self, _context: &mut TaskContext<'_>) {
        self.apply();
    }
}

impl Module for NBodyGravity {
    fn init(&mut self) {
        self.check();
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.apply();
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_gravity.py`.
    //!
    //! Not ported: `test_gps`, which propagates against a GNSS ephemeris
    //! (`gnss_221111_221123_v01.bsp`) with SPICE-driven Earth orientation and
    //! third bodies, and the SPICE variants of `test_mujocoVsSpacecraft`;
    //! both need upstream's `spiceInterface` in the dynamics task. The
    //! spherical-harmonics comparison with a spacecraft is kept with a fixed
    //! Earth orientation in both simulations instead. The ownership test of
    //! the Python bindings does not apply.

    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Matrix3;

    use super::*;
    use crate::dynamics::gravity::{
        GravBodyData, PointMassGravityModel, SphericalHarmonicsGravityModel,
    };
    use crate::integrators::AdaptiveController;
    use crate::integrators::{Euler, Rkf78};
    use crate::messages::PlanetOrientation;
    use crate::mujoco_dynamics::tests::{context, run, sec2nano, test_data};
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};
    use crate::utilities::orbital_motion::{ClassicElements, elements_to_state, state_to_elements};
    use crate::utilities::rigid_body_kinematics::euler123_to_mrp;

    const MU: f64 = 0.3986004415e15;

    fn leo() -> ClassicElements {
        ClassicElements {
            semi_major_axis: 7000.0 * 1000.0,
            eccentricity: 0.0001,
            inclination_rad: 33.3_f64.to_radians(),
            raan_rad: 48.2_f64.to_radians(),
            arg_periapsis_rad: 347.8_f64.to_radians(),
            true_anomaly_rad: 85.3_f64.to_radians(),
            ..ClassicElements::default()
        }
    }

    /// RKF78 at upstream's tolerances. At orbital distances MuJoCo's
    /// center-of-mass arithmetic leaves roundoff of about 1e-9 m between a
    /// body origin and its subtree center of mass, which shows up as angular
    /// acceleration noise near 1e-8 rad/s^2 that the error control resolves
    /// with many small sub-steps; the sub-step cap is raised accordingly.
    fn rkf78(atol: f64, rtol: f64) -> Rkf78 {
        Rkf78::new(AdaptiveController {
            atol,
            rtol,
            max_substeps: 1_000_000,
            ..AdaptiveController::default()
        })
    }

    /// Upstream `test_pointMass`: two orbits under point-mass gravity keep
    /// the orbital elements.
    #[test]
    fn point_mass_gravity_keeps_the_orbit() {
        let (r, v) = elements_to_state(MU, &leo());
        let oe = state_to_elements(MU, r, v);
        let period = 2.0 * PI * (oe.semi_major_axis.powi(3) / MU).sqrt();
        let tf = 2.0 * period;

        let mut scene = MjScene::from_file(test_data("nbody_test_ball.xml")).expect("scene");
        scene.set_integrator(rkf78(1e-12, 1e-10));
        let mut gravity = NBodyGravity::new("gravity");
        gravity.add_gravity_source("earth", PointMassGravityModel::new(MU).expect("mu"), true);
        gravity.add_gravity_target_at_body(&mut scene, "ball", "ball");
        scene.add_model_to_dynamics_task(gravity, -1);
        let com = scene.body("ball").center_of_mass().state_out_msg.clone();
        scene.init();
        {
            let mut body = scene.body_mut("ball");
            body.set_position(r);
            body.set_velocity(v);
            body.set_attitude(euler123_to_mrp(Vector3::new(PI / 2.0, PI / 6.0, PI / 4.0)));
            body.set_attitude_rate(Vector3::new(0.3, 0.1, 0.2));
        }
        run(&mut scene, sec2nano(tf), sec2nano(tf), |_, _| {});

        let state = com.read();
        let post = state_to_elements(MU, state.position_m, state.velocity_mps);
        for (after, before) in [
            (post.semi_major_axis, oe.semi_major_axis),
            (post.eccentricity, oe.eccentricity),
            (post.inclination_rad, oe.inclination_rad),
            (post.raan_rad, oe.raan_rad),
            (post.arg_periapsis_rad, oe.arg_periapsis_rad),
        ] {
            assert!(
                (after - before).abs() <= 1e-4 * before.abs(),
                "{after} vs {before}"
            );
        }
    }

    /// Upstream `test_factory_source_without_attitude`: a central source
    /// whose ephemeris has a position but no orientation pulls like an
    /// inertially aligned point mass.
    #[test]
    fn source_without_attitude_is_inertially_aligned() {
        let radius = 7000.0 * 1000.0;
        let mut scene = MjScene::new(
            r#"
            <mujoco>
                <worldbody>
                    <body name="ball">
                        <freejoint/>
                        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("scene");
        scene.extra_eom_call = true;
        let ephemeris = Output::new(PlanetStateMsg {
            position_inertial_m: Vector3::new(1000.0 * 1000.0, 0.0, 0.0),
            ..PlanetStateMsg::default()
        });
        let mut gravity = NBodyGravity::new("mujocoScene_gravity");
        ephemeris.connect_to(
            &mut gravity
                .add_gravity_source("earth", PointMassGravityModel::new(MU).expect("mu"), true)
                .state_in_msg,
        );
        gravity.add_gravity_targets_for_all_bodies(&mut scene);
        let force = gravity
            .gravity_target("ball")
            .mass_fixed_force_out_msg
            .clone();
        scene.add_model_to_dynamics_task(gravity, -1);
        scene.init();
        scene
            .body_mut("ball")
            .set_position(Vector3::new(radius, 0.0, 0.0));
        scene.update(&context(0));

        let actual = force.read().force_request_body_n;
        let expected = Vector3::new(-MU / (radius * radius), 0.0, 0.0);
        assert!(actual.iter().all(|value| value.is_finite()));
        assert!((actual - expected).norm() <= 1e-12 * expected.norm());
    }

    /// Upstream `test_dumbbell`: two 10 t point masses 200 m apart on a
    /// massless center body. The center keeps its orbit; with an initial
    /// rate of one turn per orbit, gravity gradient keeps the dumbbell
    /// pointing at Earth (constant rate); without it, the inner mass stays
    /// inside.
    #[test]
    fn dumbbell_feels_the_gravity_gradient() {
        for initial_rate in [false, true] {
            let elements = ClassicElements {
                semi_major_axis: 7000.0 * 1000.0,
                ..ClassicElements::default()
            };
            let (r, v) = elements_to_state(MU, &elements);
            let oe = state_to_elements(MU, r, v);
            let period = 2.0 * PI * (oe.semi_major_axis.powi(3) / MU).sqrt();
            let dt = period / 50.0;

            let mut scene =
                MjScene::from_file(test_data("nbody_test_dumbbell.xml")).expect("scene");
            scene.name = "mujocoScene".to_string();
            scene.extra_eom_call = true;
            scene.set_integrator(rkf78(1e-14, 1e-12));
            let mut gravity = NBodyGravity::new("mujocoScene_gravity");
            gravity.add_gravity_source("earth", PointMassGravityModel::new(MU).expect("mu"), true);
            gravity.add_gravity_targets_for_all_bodies(&mut scene);
            scene.add_model_to_dynamics_task(gravity, -1);
            let states: Vec<_> = ["left", "right", "center"]
                .iter()
                .map(|body| scene.body(body).center_of_mass().state_out_msg.clone())
                .collect();
            scene.init();
            {
                let mut center = scene.body_mut("center");
                center.set_position(r);
                center.set_velocity(v);
                center.set_attitude(euler123_to_mrp(Vector3::zeros()));
                center.set_attitude_rate(Vector3::new(
                    0.0,
                    0.0,
                    if initial_rate { 2.0 * PI / period } else { 0.0 },
                ));
            }
            let mut samples = Vec::new();
            run(&mut scene, sec2nano(dt), sec2nano(period), |_, _| {
                samples.push(states.iter().map(|state| state.read()).collect::<Vec<_>>());
            });

            let last = &samples.last().expect("samples")[2];
            let post = state_to_elements(MU, last.position_m, last.velocity_mps);
            assert!(
                (post.semi_major_axis - oe.semi_major_axis).abs()
                    <= 1e-8 + 1e-4 * oe.semi_major_axis
            );
            assert!((post.eccentricity - oe.eccentricity).abs() <= 1e-8 + 1e-4 * oe.eccentricity);
            assert!((post.inclination_rad - oe.inclination_rad).abs() <= 1e-8);
            if initial_rate {
                let rate = 2.0 * PI / period;
                for sample in &samples {
                    assert!((sample[2].omega_radps.z - rate).abs() <= 1e-6 * rate);
                }
            } else {
                for sample in &samples {
                    let (left, right, center) = (
                        sample[0].position_m.norm(),
                        sample[1].position_m.norm(),
                        sample[2].position_m.norm(),
                    );
                    assert!(left < center && center < right);
                }
            }
        }
    }

    /// Upstream `test_mujocoVsSpacecraft` (its no-SPICE case, and its
    /// spherical-harmonics case with a fixed Earth orientation in place of
    /// SPICE): an Euler-integrated scene body under `NBodyGravity` and an
    /// Euler-integrated spacecraft under its gravity effector follow the
    /// same orbit for 10 s.
    #[test]
    fn scene_matches_spacecraft_gravity() {
        for spherical_harmonics in [false, true] {
            let (r, v) = elements_to_state(MU, &leo());
            let (dt, tf) = (1.0, 10.0);
            let ggm03s = "assets/gravity/GGM03S.txt";

            let mut spacecraft = Spacecraft::new(SpacecraftConfig {
                mass_kg: 100.0,
                hub_center_of_mass_body_m: Vector3::zeros(),
                inertia_kg_m2: Matrix3::identity(),
                integration_step_nanos: sec2nano(dt),
                initial_position_m: r,
                initial_velocity_mps: v,
                initial_sigma_bn: Vector3::zeros(),
                initial_omega_radps: Vector3::zeros(),
                integrator: Some(Box::new(Euler)),
            });
            let earth = if spherical_harmonics {
                GravBodyData::new(
                    "earth",
                    SphericalHarmonicsGravityModel::from_file(ggm03s, 4).expect("GGM03S"),
                )
                .expect("earth")
                .central()
                .with_static_orientation(PlanetOrientation::identity())
            } else {
                GravBodyData::point_mass("earth", MU, true, Vector3::zeros(), Vector3::zeros())
                    .expect("earth")
            };
            spacecraft.add_grav_body(earth).expect("earth");
            spacecraft.init();
            let epoch = Epoch::from_gregorian_utc_at_midnight(2022, 11, 14);
            let mut now = 0;
            while now < sec2nano(tf) {
                now += sec2nano(dt);
                spacecraft.update(&SimulationContext {
                    current_sim_nanos: now,
                    current_epoch: epoch,
                });
            }
            let expected = spacecraft.state_out.read().position_m;

            let mut scene = MjScene::from_file(test_data("nbody_test_ball.xml")).expect("scene");
            scene.name = "mujocoScene".to_string();
            scene.set_integrator(Euler);
            let mut gravity = NBodyGravity::new("mujocoScene_gravity");
            let ephemeris = Output::new(PlanetStateMsg {
                orientation: Some(PlanetOrientation::identity()),
                ..PlanetStateMsg::default()
            });
            if spherical_harmonics {
                ephemeris.connect_to(
                    &mut gravity
                        .add_gravity_source(
                            "earth",
                            SphericalHarmonicsGravityModel::from_file(ggm03s, 4).expect("GGM03S"),
                            true,
                        )
                        .state_in_msg,
                );
            } else {
                gravity.add_gravity_source(
                    "earth",
                    PointMassGravityModel::new(MU).expect("mu"),
                    true,
                );
            }
            gravity.add_gravity_targets_for_all_bodies(&mut scene);
            scene.add_model_to_dynamics_task(gravity, -1);
            let com = scene.body("ball").center_of_mass().state_out_msg.clone();
            scene.init();
            scene.body_mut("ball").set_position(r);
            scene.body_mut("ball").set_velocity(v);
            run(&mut scene, sec2nano(dt), sec2nano(tf), |_, _| {});
            let actual = com.read().position_m;

            for axis in 0..3 {
                assert!(
                    (actual[axis] - expected[axis]).abs() <= 1e-10 * expected[axis].abs(),
                    "harmonics {spherical_harmonics}: {actual} vs {expected}"
                );
            }
        }
    }

    #[test]
    #[should_panic(expected = "More than one central body")]
    fn rejects_two_central_bodies() {
        let mut gravity = NBodyGravity::new("gravity");
        let ephemeris = Output::new(PlanetStateMsg::default());
        ephemeris.connect_to(
            &mut gravity
                .add_gravity_source("earth", PointMassGravityModel::new(MU).expect("mu"), true)
                .state_in_msg,
        );
        ephemeris.connect_to(
            &mut gravity
                .add_gravity_source("moon", PointMassGravityModel::new(MU).expect("mu"), true)
                .state_in_msg,
        );
        Module::init(&mut gravity);
    }
}
