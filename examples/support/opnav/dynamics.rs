//! Upstream `OpNavScenarios/modelsOpNav/BSK_OpNavDynamics.py`: the BskSim
//! spacecraft on orbit about Mars, with an instrument camera.
//!
//! | upstream (`DynamicsTask` priority) | here |
//! |---|---|
//! | `gravFactory.createBodies(["sun", "earth", "mars barycenter", "jupiter barycenter"])`, Mars central with `GGM2BData` to degree 2, `createSpiceInterface("2019 DECEMBER 12 18:00:00.0")`, `zeroBase = "mars barycenter"` (200) | [`OpNavDynamics::bodies`]: ANISE ephemerides (DE440s, `pck11.pca`), Mars-centred, at [`epoch`] |
//! | `eclipseObject` (204) | [`OpNavDynamics::eclipse`] |
//! | `scObject` `bskSat` (201) with `rwStateEffector` (301), `extForceTorqueObject` (300) and `thrustersDynamicEffector` | [`OpNavDynamics::spacecraft`] |
//! | `ephemObject` (199) of Mars | [`OpNavDynamics::ephem_object`] |
//! | `SimpleNavObject` (109) | [`OpNavDynamics::simple_nav`] |
//! | `CSSConstellationObject` (299) | [`OpNavDynamics::css`] |
//! | `vizInterface` (100) and the Vizard application | [`OpNavDynamics::renderer`], [`OpNavDynamics::renderer2`]: [`PlanetRenderer`] stand-ins (see below) |
//! | `CameraTask` (60 s): `cameraMod` `instrument`, `cameraMod2` `cam2` (99) | [`OpNavDynamics::camera`], [`OpNavDynamics::camera2`] |
//!
//! # The images: a stand-in for Vizard
//!
//! Upstream's cameras publish their configuration to `vizInterface`, Vizard
//! renders each camera's view every `renderRate` (60 s) and sends the image
//! back, and `vizInterface` publishes it on `opnavImageOutMsgs`, which the
//! camera modules corrupt (box blur of 3 pixels, the scenarios' cosmic rays)
//! for the image processing. Here each camera's image comes from a
//! [`PlanetRenderer`]: a ray-traced, Sun-lit, textureless Mars ellipsoid
//! through the camera's pinhole model (see `planet_renderer` for the model and
//! what it leaves out). It runs where `vizInterface` runs, at the end of the
//! dynamics task, and reads the camera configuration the camera module
//! published at its last update, as `vizInterface` does.
//!
//! # Differences from upstream
//!
//! As in [`bsk_sim::dynamics`](super::bsk_sim::dynamics), which this model
//! shares its spacecraft with:
//!
//! * The planet states come from ANISE (DE440s) instead of CSPICE (DE430),
//!   and the ephemerides run first in the tick (the gravity effector takes
//!   a planet message as written at the current tick).
//! * The wheels take their commands from [`OpNavDynamics::rw_commands`],
//!   which splits the FSW wheel-torque array at the start of the tick, where
//!   upstream's wheel effector reads it.
//! * basilisk-rs's [`Eclipse`] has one occulting body at the origin, Mars;
//!   the Earth's and Jupiter's shadows upstream adds never reach the orbits.
//! * The CSS publish one message each ([`CssConstellation`]); nothing in the
//!   OpNav flight software reads them, as upstream.
//! * The eight ACS thrusters are added without commands, as upstream: they
//!   never fire.
//! * `simpleNav` is [`NoisySimpleNav`] with upstream's `PMatrix` (as 1-sigma
//!   steps, see there), `walkBounds` and `crossTrans`.
//! * The camera's cosmic rays are seeded with [`COSMIC_RAY_SEED`] instead of
//!   the wall clock, so runs repeat.

use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::reaction_wheel_state_effector::ReactionWheelStateEffector;
use basilisk_rs::dynamics::thruster_dynamic_effector::ThrusterDynamicEffector;
use basilisk_rs::environment::anise_planet_ephemeris::AnisePlanetEphemeris;
use basilisk_rs::environment::eclipse::{Eclipse, EclipseConfig};
use basilisk_rs::environment::ephemeris_converter::{EphemerisConverter, EphemerisConverterConfig};
use basilisk_rs::messages::{Output, PlanetStateMsg, ReactionWheelStateMsg, RwSpeedMsg};
use basilisk_rs::sensors::camera::{Camera, CameraConfig};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{HubStateHandle, Spacecraft, SpacecraftConfig};
use hifitime::Epoch;
use nalgebra::Vector3;

use super::bsk_sim::dynamics::{
    CSS_NORMALS_B, CssConstellation, HUB_MASS_KG, THRUSTER_DIRECTION, THRUSTER_POSITION_M,
    WHEEL_COUNT, css, hub_inertia_kg_m2, rw_factory,
};
use super::bsk_sim::simple_nav::{NavSigmas, NoisySimpleNav, grouped};
use super::grav_factory::{Body, GGM2B};
use super::multi_sat_environment::SunEphemerisFromPlanet;
use super::planet_renderer::{PlanetRenderer, PlanetRendererConfig};
use super::sec2nano;
use super::staged::UpdatedAtInit;
use super::thruster_factory::{ThrusterFactory, ThrusterModel};
use super::{HubInit, OpNavSim};

/// Upstream `timeInitString = "2019 DECEMBER 12 18:00:00.0"` (UTC).
pub fn epoch() -> Epoch {
    Epoch::from_gregorian_utc(2019, 12, 12, 18, 0, 0, 0)
}

/// Upstream `gravFactory.createBodies([...])`, in the order of
/// `spiceObject.planetStateOutMsgs` (upstream's `sun = 0`, `earth = 1`,
/// `mars = 2`, `jupiter = 3`).
pub const BODIES: [Body; 4] = [
    Body::Sun,
    Body::Earth,
    Body::MarsBarycenter,
    Body::JupiterBarycenter,
];
pub const SUN: usize = 0;
pub const MARS: usize = 2;

/// Upstream `cameraRate` \[s\]: the camera task period and `renderRate`.
pub const CAMERA_RATE_S: f64 = 60.0;
/// Upstream `cameraRez` \[pixels\].
pub const CAMERA_RESOLUTION_PX: [i32; 2] = [512, 512];
/// Upstream `cameraMod.fieldOfView` \[deg\].
pub const CAMERA_FOV_DEG: f64 = 55.0;
/// Upstream `cameraMRP_CB`.
pub const CAMERA_MRP_CB: [f64; 3] = [0.0, 0.0, 0.0];
/// Upstream `cameraPos_B` \[m\].
pub const CAMERA_POS_B_M: [f64; 3] = [0.0, 0.2, 2.2];
/// Upstream `cameraMod.blurParam` \[pixels\].
pub const CAMERA_BLUR_PX: f64 = 3.0;
/// Stands in for the wall-clock seconds of upstream's cosmic-ray seed.
pub const COSMIC_RAY_SEED: u64 = 0;

/// Upstream `cameraSize` \[m\]: the sensor, 10 mm on a side.
pub fn camera_size_m() -> [f64; 2] {
    let [width, height] = CAMERA_RESOLUTION_PX;
    [10e-3, f64::from(height) / f64::from(width) * 10e-3]
}

/// Upstream `cameraFocal` \[m\].
pub fn camera_focal_m() -> f64 {
    camera_size_m()[1] / 2.0 / (CAMERA_FOV_DEG.to_radians() / 2.0).tan()
}

/// Upstream `SetSimpleNavObject`'s `PMatrix` diagonal.
pub fn simple_nav_sigmas() -> NavSigmas {
    NavSigmas {
        position: 10.0,
        velocity: 0.001,
        attitude: (1.0 / 36000.0_f64).to_radians(),
        rate: 0.00005_f64.to_radians(),
        sun: 0.1_f64.to_radians(),
        delta_v: 0.005,
    }
}

/// Upstream `SetSimpleNavObject`'s `walkBounds`.
pub fn simple_nav_walk_bounds() -> [f64; 6] {
    [
        100_000.0,
        0.1,
        1e-18_f64.to_radians(),
        1e-18_f64.to_radians(),
        5.0_f64.to_radians(),
        0.5,
    ]
}

/// The navigation sensor's error seed when none is set (upstream's
/// `RNGSeed` default): a hash of its name.
pub fn simple_nav_default_seed() -> u64 {
    NoisySimpleNav::new(
        "SimpleNavigation",
        simple_nav_sigmas().p_matrix(),
        grouped(simple_nav_walk_bounds()),
    )
    .seed
}

/// Upstream `SetCamera` (`instrument`).
pub fn instrument_camera() -> CameraConfig {
    CameraConfig {
        name: "instrument".to_string(),
        parent_name: "bskSat".to_string(),
        camera_is_on: true,
        camera_id: 1,
        resolution_px: CAMERA_RESOLUTION_PX,
        render_rate_nanos: sec2nano(CAMERA_RATE_S),
        field_of_view_rad: CAMERA_FOV_DEG.to_radians(),
        camera_pos_b_m: Vector3::from(CAMERA_POS_B_M),
        sigma_cb: Vector3::from(CAMERA_MRP_CB),
        sky_box: "black".to_string(),
        blur_param: CAMERA_BLUR_PX,
        cosmic_ray_seed: Some(COSMIC_RAY_SEED),
        ..CameraConfig::default()
    }
}

/// Upstream `SetCamera2` (`cam2`): set up, but not used by the flight
/// software.
pub fn second_camera() -> CameraConfig {
    CameraConfig {
        name: "cam2".to_string(),
        camera_id: 3,
        sigma_cb: Vector3::new(0.0, 0.5, 0.0),
        ..instrument_camera()
    }
}

/// Upstream `BSKDynamicModels` of the OpNav scenarios.
pub struct OpNavDynamics {
    /// Upstream `processTasksTimeStep` \[ns\].
    pub task_period_nanos: u64,
    /// Upstream `CameraTask` period \[ns\].
    pub camera_period_nanos: u64,
    /// Upstream `spiceObject.planetStateOutMsgs`, in [`BODIES`] order,
    /// relative to Mars.
    pub bodies: Vec<UpdatedAtInit<AnisePlanetEphemeris>>,
    /// The Sun as the eclipse and the CSS read it.
    pub sun: SunEphemerisFromPlanet,
    /// The wheel commands of upstream `rwMotorCmdInMsg`.
    pub rw_commands: ArrayMotorTorqueToSingleActuators,
    pub css: CssConstellation,
    pub eclipse: Eclipse,
    pub spacecraft: Spacecraft,
    /// Upstream `ephemObject`: Mars's ephemeris.
    pub ephem_object: EphemerisConverter,
    pub simple_nav: NoisySimpleNav,
    /// The stand-in for Vizard rendering `cameraMod`'s images.
    pub renderer: PlanetRenderer,
    /// The stand-in for Vizard rendering `cameraMod2`'s images.
    pub renderer2: PlanetRenderer,
    /// Upstream `cameraMod`.
    pub camera: Camera,
    /// Upstream `cameraMod2`.
    pub camera2: Camera,
    /// Upstream `rwStateEffector.rwSpeedOutMsg`.
    pub rw_speed_out_msg: Output<RwSpeedMsg>,
    /// Upstream `rwStateEffector.rwOutMsgs`.
    pub rw_out_msgs: Vec<Output<ReactionWheelStateMsg>>,
    /// Upstream `scObject.dynManager` hub states.
    pub hub: HubStateHandle,
    /// Upstream `gravFactory.gravBodies['mars barycenter'].mu` \[m^3/s^2\].
    pub mu_mars_m3ps2: f64,
}

impl OpNavDynamics {
    /// Upstream `BSKDynamicModels(SimBase, dynRate)` with the cameras
    /// `instrument` and `cam2` (the scenarios change their parameters before
    /// building), and `configure_initial_conditions`' hub state.
    pub fn new(
        base: &OpNavSim,
        initial: &HubInit,
        camera_config: CameraConfig,
        camera2_config: CameraConfig,
    ) -> Self {
        let period = base.dyn_rate_nanos;
        let start = epoch();

        // `SetSpacecraftHub`.
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: HUB_MASS_KG,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: hub_inertia_kg_m2(),
            integration_step_nanos: period,
            initial_position_m: initial.position_m,
            initial_velocity_mps: initial.velocity_mps,
            initial_sigma_bn: initial.sigma_bn,
            initial_omega_radps: initial.omega_bn_b_radps,
            integrator: None,
        });

        // `SetGravityEffector`: Mars central with its degree-2 field, the
        // others point masses, every state relative to Mars.
        let bodies: Vec<_> = BODIES
            .iter()
            .map(|body| UpdatedAtInit::new(body.ephemeris(Body::MarsBarycenter), start))
            .collect();
        for (body, ephemeris) in BODIES.iter().zip(&bodies) {
            let central = *body == Body::MarsBarycenter;
            let grav_body = if central {
                body.spherical_harmonics(GGM2B, 2, true)
            } else {
                body.point_mass(false)
            };
            spacecraft
                .add_grav_body(grav_body)
                .expect("unique gravity body");
            ephemeris.module.output_planet_msg.connect_to(
                spacecraft
                    .grav_body_input_mut(body.name())
                    .expect("added gravity body"),
            );
        }
        let mut sun = SunEphemerisFromPlanet::default();
        bodies[SUN]
            .module
            .output_planet_msg
            .connect_to(&mut sun.sun_in_msg);

        // `SetEclipseObject`.
        let mut eclipse = Eclipse::new(EclipseConfig {
            name: "eclipseObject".to_string(),
            occulting_body_radius_m: Body::MarsBarycenter.radius_equator_m(),
        });
        spacecraft
            .state_out
            .connect_to(&mut eclipse.input_state_msg);
        sun.sun_out_msg.connect_to(&mut eclipse.input_sun_msg);

        // `SetExternalForceTorqueObject`: no command is ever connected.
        spacecraft.add_dynamic_effector(ExtForceTorque::new(ExtForceTorqueConfig {
            name: "externalDisturbance".to_string(),
            ..ExtForceTorqueConfig::default()
        }));

        // `SetSimpleNavObject`.
        let mut simple_nav = NoisySimpleNav::new(
            "SimpleNavigation",
            simple_nav_sigmas().p_matrix(),
            grouped(simple_nav_walk_bounds()),
        );
        simple_nav.cross_trans = true;
        simple_nav.cross_att = false;
        spacecraft
            .state_out
            .connect_to(&mut simple_nav.spacecraft_state_in_msg);

        // `SetReactionWheelDynEffector`.
        let mut wheels = ReactionWheelStateEffector::new("RWStateEffector");
        for config in &rw_factory().wheels {
            wheels.add_reaction_wheel(config.clone());
        }
        let rw_commands =
            ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
                name: "rwMotorCmd".to_string(),
                num_actuators: WHEEL_COUNT,
            });
        for (command, wheel) in rw_commands
            .actuator_out_msgs
            .iter()
            .zip(wheels.wheels_mut())
        {
            command.connect_to(&mut wheel.motor_torque_in_msg);
        }
        let rw_speed_out_msg = wheels.rw_speed_out_msg.clone();
        let rw_out_msgs = wheels
            .wheels()
            .iter()
            .map(|wheel| wheel.state_out.clone())
            .collect();
        spacecraft.add_state_effector(wheels);

        // `SetACSThrusterStateEffector`: eight `MOOG_Monarc_1`, never fired.
        let mut thrusters = ThrusterFactory::default();
        for (position, direction) in THRUSTER_POSITION_M.iter().zip(THRUSTER_DIRECTION) {
            thrusters.create(
                ThrusterModel::MoogMonarc1,
                Vector3::from(*position),
                Vector3::from(direction),
                None,
            );
        }
        for thruster in &thrusters.thrusters {
            spacecraft.add_dynamic_effector(ThrusterDynamicEffector::new(
                thruster.dynamic_effector_config(),
            ));
        }

        // `SetCSSConstellation`.
        let mut css = CssConstellation {
            sensors: CSS_NORMALS_B
                .iter()
                .enumerate()
                .map(|(index, normal)| css(&format!("CSS{}", index + 1), Vector3::from(*normal)))
                .collect(),
        };
        for sensor in &mut css.sensors {
            sun.sun_out_msg.connect_to(&mut sensor.input_sun_msg);
            spacecraft.state_out.connect_to(&mut sensor.input_state_msg);
            eclipse
                .output_eclipse_msg
                .connect_to(&mut sensor.input_eclipse_msg);
        }

        // `SetEphemConvert`.
        let mut ephem_object = EphemerisConverter::new(EphemerisConverterConfig {
            name: "EphemData".to_string(),
            body_names: vec![Body::MarsBarycenter.name().to_string()],
        });
        bodies[MARS]
            .module
            .output_planet_msg
            .connect_to(&mut ephem_object.planet_in_msgs[0]);

        // `SetCamera`, `SetCamera2` and `SetVizInterface`: each camera's
        // images come from its renderer.
        let mut camera = Camera::new(camera_config);
        let mut camera2 = Camera::new(camera2_config);
        let mut renderer = PlanetRenderer::new(PlanetRendererConfig::mars("vizInterface"));
        let mut renderer2 = PlanetRenderer::new(PlanetRendererConfig::mars("vizInterface2"));
        for (renderer, camera) in [(&mut renderer, &mut camera), (&mut renderer2, &mut camera2)] {
            camera
                .camera_config_out_msg
                .connect_to(&mut renderer.camera_config_in_msg);
            spacecraft
                .state_out
                .connect_to(&mut renderer.spacecraft_state_in_msg);
            bodies[MARS]
                .module
                .output_planet_msg
                .connect_to(&mut renderer.planet_state_in_msg);
            bodies[SUN]
                .module
                .output_planet_msg
                .connect_to(&mut renderer.sun_state_in_msg);
            renderer.image_out_msg.connect_to(&mut camera.image_in_msg);
        }

        let hub = spacecraft.hub_state_handle();
        Self {
            task_period_nanos: period,
            camera_period_nanos: sec2nano(CAMERA_RATE_S),
            bodies,
            sun,
            rw_commands,
            css,
            eclipse,
            spacecraft,
            ephem_object,
            simple_nav,
            renderer,
            renderer2,
            camera,
            camera2,
            rw_speed_out_msg,
            rw_out_msgs,
            hub,
            mu_mars_m3ps2: Body::MarsBarycenter.mu_m3ps2(),
        }
    }

    /// Mars's state relative to itself, with its `IAU_MARS` orientation.
    pub fn mars_state_out_msg(&self) -> &Output<PlanetStateMsg> {
        &self.bodies[MARS].module.output_planet_msg
    }

    /// The Sun's state relative to Mars.
    pub fn sun_state_out_msg(&self) -> &Output<PlanetStateMsg> {
        &self.bodies[SUN].module.output_planet_msg
    }

    /// Upstream's `DynamicsTask` and `CameraTask`, in upstream's order with
    /// the ephemerides moved first.
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let period = self.task_period_nanos;
        let camera_period = self.camera_period_nanos;
        for (body, ephemeris) in BODIES.iter().zip(&mut self.bodies) {
            simulation.add_module(format!("{}_spice", body.name()), ephemeris, period, 9_500);
        }
        basilisk_rs::schedule! { *simulation,
            "sun" => &mut self.sun, period = period, priority = 9_450;
            "rwMotorCmd" => &mut self.rw_commands, period = period, priority = 9_400;
            "cssConstellation" => &mut self.css, period = period, priority = 9_300;
            "eclipseObject" => &mut self.eclipse, period = period, priority = 9_200;
            "bskSat" => &mut self.spacecraft, period = period, priority = 9_100;
            "EphemData" => &mut self.ephem_object, period = period, priority = 9_000;
            "SimpleNavigation" => &mut self.simple_nav, period = period, priority = 8_900;
            "vizInterface" => &mut self.renderer, period = period, priority = 8_800;
            "vizInterface2" => &mut self.renderer2, period = period, priority = 8_800;
            "instrument" => &mut self.camera, period = camera_period, priority = 8_000;
            "cam2" => &mut self.camera2, period = camera_period, priority = 8_000;
        }
    }
}
