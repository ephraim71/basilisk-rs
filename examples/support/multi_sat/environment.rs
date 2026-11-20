//! Upstream `modelsMultiSat/BSK_EnvironmentEarth.py` and
//! `BSK_EnvironmentMercury.py`: the environment process every spacecraft of
//! a MultiSat simulation shares.
//!
//! | upstream | here |
//! |---|---|
//! | `BSKEnvironmentModel(SimBase, envRate)` | [`Environment::new`] with a [`Planet`] |
//! | `gravFactory.createBodies([...])`, `createSpiceInterface(time, epochInMsg=True)`, `zeroBase` | [`Environment::bodies`]: one ANISE ephemeris per body, relative to the planet |
//! | `gravFactory.addBodiesTo(scObject)` | [`Environment::add_bodies_to`] |
//! | `ephemObject` (`ephemerisConverter` of every body) | [`Environment::ephemeris_converter`] |
//! | `eclipseObject` with `addSpacecraftToModel` | [`Environment::eclipses`], one per spacecraft, [`Environment::add_spacecraft`] |
//! | `groundStation` (Boulder) | [`Environment::ground_station`] |
//!
//! The bodies are point masses with upstream's `astroConstants`: the Sun,
//! the Earth and the Moon about the central Earth from
//! `2021 MAY 04 07:47:48.965 (UTC)`, or the Sun and Mercury about the
//! central Mercury from `2012 MAY 1 00:28:30.0`. Upstream's SPICE kernels
//! (DE430) become ANISE's DE440s with `pck11.pca` (`just
//! fetch-anise-assets`), each body relative to the central planet, which is
//! upstream's `zeroBase`.
//!
//! # Differences from upstream
//!
//! * basilisk-rs's `eclipse` handles one spacecraft and one occulting body,
//!   the central planet at the inertial origin, so the environment holds an
//!   eclipse module per spacecraft, and the Moon's shadow (upstream adds the
//!   Moon to the Earth environment's eclipse) is not modelled.
//! * The eclipse and the solar panels take the Sun as a `SunEphemerisMsg`;
//!   [`SunEphemerisFromPlanet`] turns the Sun's planet state into one.
//! * As upstream, the ground station's planet input is not connected: the
//!   station sits on a non-rotating planet fixed in the inertial frame.

use basilisk_rs::environment::anise_planet_ephemeris::AnisePlanetEphemeris;
use basilisk_rs::environment::eclipse::{Eclipse, EclipseConfig};
use basilisk_rs::environment::ephemeris_converter::{EphemerisConverter, EphemerisConverterConfig};
use basilisk_rs::environment::ground_location::{
    GroundLocation, GroundLocationConfig, SiteLocation,
};
use basilisk_rs::messages::{
    EclipseMsg, EphemerisMsg, GroundStateMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg,
    SunEphemerisMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::Spacecraft;
use basilisk_rs::utilities::geodetic_conversion::Lla;
use basilisk_rs::{Module, SimulationContext};
use hifitime::Epoch;

use super::grav_factory::Body;
use super::staged::UpdatedAtInit;

/// Upstream's environment files.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Planet {
    /// `BSK_EnvironmentEarth`.
    Earth,
    /// `BSK_EnvironmentMercury`.
    Mercury,
}

impl Planet {
    /// Upstream's `environment` argument.
    pub fn name(self) -> &'static str {
        match self {
            Self::Earth => "Earth",
            Self::Mercury => "Mercury",
        }
    }

    /// The central body.
    pub fn body(self) -> Body {
        match self {
            Self::Earth => Body::Earth,
            Self::Mercury => Body::Mercury,
        }
    }

    /// Upstream `createBodies([...])`, in its order (the Sun first).
    pub fn bodies(self) -> &'static [Body] {
        match self {
            Self::Earth => &[Body::Sun, Body::Earth, Body::Moon],
            Self::Mercury => &[Body::Sun, Body::Mercury],
        }
    }

    /// Upstream `timeInitString` (UTC).
    pub fn epoch(self) -> Epoch {
        match self {
            // "2021 MAY 04 07:47:48.965 (UTC)"
            Self::Earth => Epoch::from_gregorian_utc(2021, 5, 4, 7, 47, 48, 965_000_000),
            // "2012 MAY 1 00:28:30.0"
            Self::Mercury => Epoch::from_gregorian_utc(2012, 5, 1, 0, 28, 30, 0),
        }
    }

    /// Upstream ground station `ModelTag`.
    fn ground_station_name(self) -> &'static str {
        match self {
            Self::Earth => "BoulderGroundStation",
            Self::Mercury => "GroundStation",
        }
    }
}

impl std::str::FromStr for Planet {
    type Err = String;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        match text {
            "Earth" | "earth" => Ok(Self::Earth),
            "Mercury" | "mercury" => Ok(Self::Mercury),
            _ => Err(format!("unknown environment {text:?} (Earth or Mercury)")),
        }
    }
}

/// The Sun's planet state as the `SunEphemerisMsg` the eclipse and solar
/// panel modules read.
#[derive(Default)]
pub struct SunEphemerisFromPlanet {
    pub sun_in_msg: Input<PlanetStateMsg>,
    pub sun_out_msg: Output<SunEphemerisMsg>,
}

impl SunEphemerisFromPlanet {
    fn convert(&self) {
        let sun = self.sun_in_msg.read();
        self.sun_out_msg.write(SunEphemerisMsg {
            sun_position_inertial_m: sun.position_inertial_m,
            sun_velocity_inertial_mps: sun.velocity_inertial_mps,
        });
    }
}

impl Module for SunEphemerisFromPlanet {
    fn init(&mut self) {
        // The Sun's ephemeris has published its first state at its own init.
        self.convert();
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.convert();
    }
}

/// One gravity body: its upstream data and its ephemeris module.
pub struct GravityBody {
    pub body: Body,
    pub central: bool,
    /// Upstream `spiceObject.planetStateOutMsgs[i]`, written at
    /// initialization too (upstream's SPICE `Reset` updates).
    pub ephemeris: UpdatedAtInit<AnisePlanetEphemeris>,
}

impl GravityBody {
    pub fn planet_state_out_msg(&self) -> &Output<PlanetStateMsg> {
        &self.ephemeris.module.output_planet_msg
    }
}

/// The environment process (upstream `BSKEnvironmentModel`).
pub struct Environment {
    pub planet: Planet,
    /// Upstream `envTaskName` period \[ns\].
    pub task_period_nanos: u64,
    /// Upstream `mu` \[m³/s²\] and `planetRadius` \[m\] of the central body.
    pub mu_m3ps2: f64,
    pub planet_radius_m: f64,
    /// Upstream `gravFactory` bodies with their SPICE states, the Sun first
    /// (upstream's `sun = 0`).
    pub bodies: Vec<GravityBody>,
    /// The Sun for the eclipse and the solar panels.
    pub sun: SunEphemerisFromPlanet,
    /// Upstream `ephemObject`: one ephemeris per body, in [`Self::bodies`]
    /// order.
    pub ephemeris_converter: EphemerisConverter,
    /// Upstream `eclipseObject.eclipseOutMsgs`: one module per spacecraft.
    pub eclipses: Vec<Eclipse>,
    /// Upstream `groundStation`.
    pub ground_station: GroundLocation,
}

/// The index of the Sun in [`Environment::bodies`] (upstream `sun = 0`).
pub const SUN: usize = 0;

impl Environment {
    /// Upstream `BSKEnvironmentModel(SimBase, envRate)` of `planet`.
    pub fn new(planet: Planet, task_period_nanos: u64) -> Self {
        let central = planet.body();
        let bodies: Vec<GravityBody> = planet
            .bodies()
            .iter()
            .map(|&body| GravityBody {
                body,
                central: body == central,
                ephemeris: UpdatedAtInit::new(body.ephemeris(central), planet.epoch()),
            })
            .collect();
        let mut ephemeris_converter = EphemerisConverter::new(EphemerisConverterConfig {
            name: "EphemData".to_string(),
            body_names: bodies
                .iter()
                .map(|body| body.body.name().to_string())
                .collect(),
        });
        for (input, body) in ephemeris_converter.planet_in_msgs.iter_mut().zip(&bodies) {
            body.planet_state_out_msg().connect_to(input);
        }
        let mut sun = SunEphemerisFromPlanet::default();
        bodies[SUN]
            .planet_state_out_msg()
            .connect_to(&mut sun.sun_in_msg);

        let planet_radius_m = central.radius_equator_m();
        let ground_station = GroundLocation::new(GroundLocationConfig {
            name: planet.ground_station_name().to_string(),
            planet_radius_m,
            minimum_elevation_rad: 10.0_f64.to_radians(),
            maximum_range_m: Some(1e9),
            location: SiteLocation::Geodetic(Lla::new(
                40.009971_f64.to_radians(),
                (-105.243895_f64).to_radians(),
                1624.0,
            )),
        });
        Self {
            planet,
            task_period_nanos,
            mu_m3ps2: central.mu_m3ps2(),
            planet_radius_m,
            bodies,
            sun,
            ephemeris_converter,
            eclipses: Vec::new(),
            ground_station,
        }
    }

    /// The simulation start epoch.
    pub fn epoch(&self) -> Epoch {
        self.planet.epoch()
    }

    /// Upstream `gravFactory.addBodiesTo(scObject)`: every body as a point
    /// mass, its position from its ephemeris.
    pub fn add_bodies_to(&self, spacecraft: &mut Spacecraft) {
        for body in &self.bodies {
            spacecraft
                .add_grav_body(body.body.point_mass(body.central))
                .expect("one central body");
            body.planet_state_out_msg().connect_to(
                spacecraft
                    .grav_body_input_mut(body.body.name())
                    .expect("the body was just added"),
            );
        }
    }

    /// Upstream `eclipseObject.addSpacecraftToModel(state)` and
    /// `groundStation.addSpacecraftToModel(state)`: returns the spacecraft's
    /// index in [`Self::eclipses`].
    pub fn add_spacecraft(&mut self, state: &Output<SpacecraftStateMsg>) -> usize {
        let index = self.ground_station.add_spacecraft_to_model();
        state.connect_to(&mut self.ground_station.sc_state_in_msgs[index]);

        let mut eclipse = Eclipse::new(EclipseConfig {
            name: "eclipseObject".to_string(),
            occulting_body_radius_m: self.planet_radius_m,
        });
        state.connect_to(&mut eclipse.input_state_msg);
        self.sun.sun_out_msg.connect_to(&mut eclipse.input_sun_msg);
        self.eclipses.push(eclipse);
        self.eclipses.len() - 1
    }

    /// Upstream `eclipseObject.eclipseOutMsgs[index]`.
    pub fn eclipse_out_msg(&self, index: usize) -> &Output<EclipseMsg> {
        &self.eclipses[index].output_eclipse_msg
    }

    /// Upstream `ephemObject.ephemOutMsgs[sun]`.
    pub fn sun_ephemeris_out_msg(&self) -> &Output<EphemerisMsg> {
        &self.ephemeris_converter.ephem_out_msgs[SUN]
    }

    /// Upstream `gravFactory.spiceObject.planetStateOutMsgs[sun]`.
    pub fn sun_planet_state_out_msg(&self) -> &Output<PlanetStateMsg> {
        self.bodies[SUN].planet_state_out_msg()
    }

    /// Upstream `groundStation.currentGroundStateOutMsg`.
    pub fn ground_state_out_msg(&self) -> &Output<GroundStateMsg> {
        &self.ground_station.current_ground_state_out_msg
    }

    /// Adds the modules to the environment process in upstream's task order:
    /// the SPICE states, the ephemerides, the eclipses, the ground station.
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let period = self.task_period_nanos;
        for body in &mut self.bodies {
            simulation.add_module(
                format!("{}_spice", body.body.name()),
                &mut body.ephemeris,
                period,
                200,
            );
        }
        simulation.add_module("sun", &mut self.sun, period, 200);
        simulation.add_module("EphemData", &mut self.ephemeris_converter, period, 200);
        for eclipse in &mut self.eclipses {
            simulation.add_module("eclipseObject", eclipse, period, 200);
        }
        simulation.add_module(
            self.planet.ground_station_name(),
            &mut self.ground_station,
            period,
            200,
        );
    }
}
