//! The spacecraft of upstream's two resource tutorials,
//! `examples/scenarioDataDemo.py` and `examples/scenarioPowerDemo.py`.
//!
//! The hub at upstream's defaults (1 kg, identity inertia) on a 400 km
//! circular equatorial Earth orbit from `f = 75 deg`, with
//! `sigma_BN = [0.1, 0.2, -0.3]` and `omega_BN_B = [0.001, -0.001, 0.001]`
//! rad/s, under the point-mass Earth (central) and Sun, from
//! `2021 MAY 04 07:47:48.965 (UTC)`. One 1 s task for one orbit period.
//!
//! Upstream's `spiceInterface` (Earth and Sun relative to the solar-system
//! barycenter) is [`AniseSunEphemeris`], the Sun relative to Earth from
//! DE440s, with the Earth at the origin of the inertial frame. Schedule it
//! before the spacecraft, which takes the Sun's gravity from it; it also
//! updates at initialization, as upstream's `Reset` does ([`UpdatedAtInit`]).
//!
//! Include it next to `common.rs`, `orbit_tools.rs` and `staged.rs`.

#![allow(dead_code)]

use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::environment::anise_sun_ephemeris::{AniseSunEphemeris, AniseSunEphemerisConfig};
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::astro_constants::{D2R, MU_EARTH, MU_SUN, REQ_EARTH};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};

use super::orbit_tools::{orbit_period_s, sec2nano};
use super::staged::UpdatedAtInit;

/// Upstream `testProcessRate = macros.sec2nano(1.0)`.
pub const TASK_PERIOD_NANOS: u64 = 1_000_000_000;
/// The DE440s planetary SPK (`just fetch-anise-assets`).
pub const PLANETARY_SPK: &str = "assets/anise/de440s.bsp";

/// Upstream `timeInitString = '2021 MAY 04 07:47:48.965 (UTC)'`.
pub fn epoch() -> Epoch {
    Epoch::from_gregorian_utc(2021, 5, 4, 7, 47, 48, 965_000_000)
}

/// The tutorial spacecraft and its Sun ephemeris.
pub struct ResourceDemo {
    /// Upstream `scObject`, with the Earth and Sun gravity bodies; the Sun's
    /// input is connected to [`Self::sun`].
    pub spacecraft: Spacecraft,
    /// Upstream `spiceObject` (the Sun; the Earth is the inertial origin).
    pub sun: UpdatedAtInit<AniseSunEphemeris>,
    /// Upstream `P` \[s\]: the orbit period.
    pub period_s: f64,
}

impl ResourceDemo {
    pub fn new() -> Self {
        let mu = MU_EARTH * 1e9;
        let elements = ClassicElements {
            semi_major_axis: REQ_EARTH * 1e3 + 400e3,
            eccentricity: 0.0,
            inclination_rad: 0.0,
            raan_rad: 0.0,
            arg_periapsis_rad: 0.0,
            true_anomaly_rad: 75.0 * D2R,
            ..Default::default()
        };
        let (initial_position_m, initial_velocity_mps) = elements_to_state(mu, &elements);
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 1.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::identity(),
            integration_step_nanos: TASK_PERIOD_NANOS,
            initial_position_m,
            initial_velocity_mps,
            initial_sigma_bn: Vector3::new(0.1, 0.2, -0.3),
            initial_omega_radps: Vector3::new(0.001, -0.001, 0.001),
            integrator: None,
        });
        for (name, mu, central) in [("earth", mu, true), ("sun", MU_SUN * 1e9, false)] {
            spacecraft
                .add_grav_body(
                    GravBodyData::point_mass(name, mu, central, Vector3::zeros(), Vector3::zeros())
                        .expect("valid point-mass gravity body"),
                )
                .expect("unique gravity body");
        }
        let sun = UpdatedAtInit::new(
            AniseSunEphemeris::new(AniseSunEphemerisConfig {
                name: "spiceObject".to_string(),
                spk_path: PLANETARY_SPK.into(),
            }),
            epoch(),
        );
        sun.module.output_planet_msg.connect_to(
            spacecraft
                .grav_body_input_mut("sun")
                .expect("the Sun is a gravity body"),
        );
        Self {
            spacecraft,
            sun,
            period_s: orbit_period_s(mu, elements.semi_major_axis),
        }
    }

    /// Upstream `ConfigureStopTime(macros.sec2nano(P))` \[ns\].
    pub fn duration_nanos(&self) -> u64 {
        sec2nano(self.period_s)
    }
}
