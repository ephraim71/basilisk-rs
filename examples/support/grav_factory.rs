//! Gravity bodies for the orbit scenarios: the part of upstream
//! `simIncludeGravBody.gravBodyFactory` they use.
//!
//! | upstream | here |
//! |---|---|
//! | `createEarth()`, `createSun()`, ... `BODY_DATA` | [`Body`] and [`Body::data`] |
//! | `planet.mu`, `planet.radEquator` | [`BodyData::mu_m3ps2`], [`BodyData::radius_equator_m`] |
//! | `createCustomGravObject(label, mu)` | [`custom_point_mass`] |
//! | `useSphericalHarmonicsGravityModel(file, maxDeg)` | [`Body::spherical_harmonics`] |
//! | `createSpiceInterface(...)` | [`Body::ephemeris`] (ANISE, see below) |
//!
//! Upstream's gravity parameters and radii are the `astroConstants` values,
//! here [`basilisk_rs::utilities::astro_constants`] (km based, scaled to SI).
//!
//! `createSpiceInterface` publishes each body's state relative to a
//! `zeroBase` and its `IAU_*` orientation from CSPICE with DE430 and
//! `pck00010.tpc`. The scenarios use basilisk-rs's ANISE ephemeris instead,
//! [`AnisePlanetEphemeris`] with DE440s and `pck11.pca` (the same IAU rotation
//! models), one module per body, connected to that body's gravity input.
//! Schedule the ephemeris modules before the spacecraft: the gravity
//! effector takes a connected planet message as written at the current
//! update (it has no write-time header to Euler-step from, as upstream does).

#![allow(dead_code)]

use std::path::PathBuf;

use anise::constants::frames::{
    EARTH_J2000, IAU_EARTH_FRAME, IAU_JUPITER_FRAME, IAU_MARS_FRAME, IAU_MERCURY_FRAME,
    IAU_MOON_FRAME, IAU_VENUS_FRAME, JUPITER_BARYCENTER_J2000, MARS_BARYCENTER_J2000, MARS_J2000,
    MOON_J2000, SSB_J2000, SUN_J2000,
};
use anise::prelude::Frame;
use basilisk_rs::dynamics::gravity::{GravBodyData, SphericalHarmonicsGravityModel};
use basilisk_rs::environment::anise_planet_ephemeris::{
    AnisePlanetEphemeris, AnisePlanetEphemerisConfig,
};
use basilisk_rs::messages::PlanetOrientation;
use basilisk_rs::utilities::astro_constants::{
    MU_EARTH, MU_JUPITER, MU_MARS, MU_MERCURY, MU_MOON, MU_SUN, MU_VENUS, REQ_EARTH, REQ_JUPITER,
    REQ_MARS, REQ_MERCURY, REQ_MOON, REQ_SUN, REQ_VENUS,
};
use nalgebra::Vector3;

/// The DE440s planetary SPK (`just fetch-anise-assets`).
pub const PLANETARY_SPK: &str = "assets/anise/de440s.bsp";
/// The IAU rotation models (`just fetch-anise-assets`).
pub const PLANETARY_CONSTANTS: &str = "assets/anise/pck11.pca";
/// Upstream `GGM03S.txt` (`just fetch-gravity`).
pub const GGM03S: &str = "assets/gravity/GGM03S.txt";
/// Upstream `GGM03S-J2-only.txt` (`just fetch-orbit-scenario-data`).
pub const GGM03S_J2_ONLY: &str = "assets/gravity/GGM03S-J2-only.txt";
/// Upstream `GGM2B-J2-only.txt` (`just fetch-orbit-scenario-data`).
pub const GGM2B_J2_ONLY: &str = "assets/gravity/GGM2B-J2-only.txt";
/// Upstream `GGM2BData.txt` (`just fetch-orbit-scenario-data`).
pub const GGM2B: &str = "assets/gravity/GGM2BData.txt";

/// A body of upstream's `gravBodyFactory` table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Body {
    Sun,
    Earth,
    Moon,
    /// The planet (NAIF 199; Mercury has no moons, so its barycenter is the
    /// same point).
    Mercury,
    /// The planet (NAIF 299; Venus has no moons, so its barycenter is the
    /// same point).
    Venus,
    Mars,
    MarsBarycenter,
    JupiterBarycenter,
}

/// One row of upstream's `BODY_DATA`, with the ANISE frames that stand in for
/// the SPICE body and `spicePlanetFrame` names.
#[derive(Clone, Copy, Debug)]
pub struct BodyData {
    /// Upstream identifier, also the SPICE name (`"earth"`, `"mars barycenter"`).
    pub name: &'static str,
    /// Gravitational parameter (m^3/s^2).
    pub mu_m3ps2: f64,
    /// Equatorial radius (m).
    pub radius_equator_m: f64,
    /// The body's J2000 frame.
    pub frame: Frame,
    /// The body-fixed `IAU_*` frame (upstream `spicePlanetFrame`); `None`
    /// for the Sun, whose `IAU_SUN` the ANISE constants do not carry (no
    /// scenario here needs the Sun's orientation).
    pub fixed_frame: Option<Frame>,
}

impl Body {
    /// Upstream `BODY_DATA[identifier]`.
    pub fn data(self) -> BodyData {
        let (name, mu_km3ps2, radius_km, frame, fixed_frame) = match self {
            Self::Sun => ("sun", MU_SUN, REQ_SUN, SUN_J2000, None),
            Self::Earth => (
                "earth",
                MU_EARTH,
                REQ_EARTH,
                EARTH_J2000,
                Some(IAU_EARTH_FRAME),
            ),
            Self::Moon => ("moon", MU_MOON, REQ_MOON, MOON_J2000, Some(IAU_MOON_FRAME)),
            Self::Mercury => (
                "mercury",
                MU_MERCURY,
                REQ_MERCURY,
                Frame::new(199, anise::constants::orientations::J2000),
                Some(IAU_MERCURY_FRAME),
            ),
            Self::Venus => (
                "venus",
                MU_VENUS,
                REQ_VENUS,
                Frame::new(299, anise::constants::orientations::J2000),
                Some(IAU_VENUS_FRAME),
            ),
            Self::Mars => ("mars", MU_MARS, REQ_MARS, MARS_J2000, Some(IAU_MARS_FRAME)),
            Self::MarsBarycenter => (
                "mars barycenter",
                MU_MARS,
                REQ_MARS,
                MARS_BARYCENTER_J2000,
                Some(IAU_MARS_FRAME),
            ),
            Self::JupiterBarycenter => (
                "jupiter barycenter",
                MU_JUPITER,
                REQ_JUPITER,
                JUPITER_BARYCENTER_J2000,
                Some(IAU_JUPITER_FRAME),
            ),
        };
        BodyData {
            name,
            mu_m3ps2: mu_km3ps2 * 1.0e9,
            radius_equator_m: radius_km * 1.0e3,
            frame,
            fixed_frame,
        }
    }

    pub fn name(self) -> &'static str {
        self.data().name
    }

    pub fn mu_m3ps2(self) -> f64 {
        self.data().mu_m3ps2
    }

    pub fn radius_equator_m(self) -> f64 {
        self.data().radius_equator_m
    }

    /// Point-mass gravity body at the origin: upstream `create<Body>()`,
    /// with `isCentralBody` set to `central`.
    pub fn point_mass(self, central: bool) -> GravBodyData {
        custom_point_mass(self.name(), self.mu_m3ps2(), central)
    }

    /// Spherical-harmonic gravity body: upstream
    /// `useSphericalHarmonicsGravityModel(file, max_degree)`. The field keeps
    /// the file's reference radius and takes the body's `mu`, as upstream.
    /// Without an ephemeris input the body does not rotate: the identity
    /// orientation upstream falls back to.
    pub fn spherical_harmonics(self, file: &str, max_degree: usize, central: bool) -> GravBodyData {
        let model = SphericalHarmonicsGravityModel::from_file(file, max_degree)
            .unwrap_or_else(|error| panic!("cannot load {file}: {error}"))
            .with_mu_m3ps2(self.mu_m3ps2())
            .expect("positive body mu");
        GravBodyData::new(self.name(), model)
            .expect("named gravity body")
            .central_if(central)
            .with_static_orientation(PlanetOrientation::identity())
    }

    /// The body's state relative to `zero_base` in J2000, and its `IAU_*`
    /// orientation, from ANISE: the stand-in for this body's
    /// `spiceObject.planetStateOutMsgs[...]` (upstream `zeroBase`).
    pub fn ephemeris(self, zero_base: Body) -> AnisePlanetEphemeris {
        self.ephemeris_relative_to(zero_base.data().frame)
    }

    /// As [`Self::ephemeris`], relative to the solar-system barycenter
    /// (upstream's default `zeroBase = "SSB"`).
    pub fn ephemeris_from_ssb(self) -> AnisePlanetEphemeris {
        self.ephemeris_relative_to(SSB_J2000)
    }

    fn ephemeris_relative_to(self, observer: Frame) -> AnisePlanetEphemeris {
        let data = self.data();
        AnisePlanetEphemeris::new(AnisePlanetEphemerisConfig {
            name: format!("{}_ephemeris", data.name),
            spk_path: PathBuf::from(PLANETARY_SPK),
            additional_kernel_paths: vec![PathBuf::from(PLANETARY_CONSTANTS)],
            source_frame: data.frame,
            observer_frame: observer,
            fixed_frame: data.fixed_frame,
        })
    }
}

/// A point-mass body at the origin with the given name and `mu`: upstream
/// `createCustomGravObject(label, mu)`.
pub fn custom_point_mass(name: &str, mu_m3ps2: f64, central: bool) -> GravBodyData {
    GravBodyData::point_mass(name, mu_m3ps2, central, Vector3::zeros(), Vector3::zeros())
        .expect("valid point-mass gravity body")
}

/// Upstream `spkRead(target, time, "J2000", observer)` (scaled to m and m/s):
/// the geometric state of `target` relative to `observer` at `epoch`, from
/// DE440s and any `extra_kernels` (a spacecraft SPK, for instance).
pub fn spk_state(
    target: Frame,
    observer: Frame,
    epoch: hifitime::Epoch,
    extra_kernels: &[&str],
) -> (Vector3<f64>, Vector3<f64>) {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut almanac = anise::prelude::Almanac::new(&root.join(PLANETARY_SPK).to_string_lossy())
        .expect("load DE440s (just fetch-anise-assets)");
    for kernel in extra_kernels {
        almanac = almanac
            .load(&root.join(kernel).to_string_lossy())
            .unwrap_or_else(|error| panic!("cannot load {kernel}: {error}"));
    }
    let state = almanac
        .translate_geometric(target, observer, epoch)
        .expect("ephemeris covers the requested epoch");
    (
        Vector3::new(state.radius_km.x, state.radius_km.y, state.radius_km.z) * 1.0e3,
        Vector3::new(
            state.velocity_km_s.x,
            state.velocity_km_s.y,
            state.velocity_km_s.z,
        ) * 1.0e3,
    )
}

/// An ephemeris module publishing `target`'s state relative to `observer`
/// from DE440s and `extra_kernels`, without orientation: for a spacecraft
/// SPK, the stand-in for upstream `spiceObject` spacecraft states.
pub fn spk_ephemeris(
    name: &str,
    target: Frame,
    observer: Frame,
    extra_kernels: &[&str],
) -> AnisePlanetEphemeris {
    AnisePlanetEphemeris::new(AnisePlanetEphemerisConfig {
        name: name.to_string(),
        spk_path: PathBuf::from(PLANETARY_SPK),
        additional_kernel_paths: extra_kernels.iter().map(PathBuf::from).collect(),
        source_frame: target,
        observer_frame: observer,
        fixed_frame: None,
    })
}
