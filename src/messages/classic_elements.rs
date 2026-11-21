//! Classical orbit elements (upstream `ClassicElementsMsgPayload`).
//!
//! Upstream declares the payload and the `classicElements` type of its orbital
//! mechanics utilities as one struct (`typedef ClassicElementsMsgPayload
//! classicElements`), so here the message is the utilities' type,
//! [`ClassicElements`](crate::utilities::orbital_motion::ClassicElements),
//! under its message name. Distances are in metres on the wire.

pub use crate::utilities::orbital_motion::ClassicElements as ClassicElementsMsg;

use super::flatten::impl_telemetry;

impl_telemetry!(ClassicElementsMsg {
    semi_major_axis,
    eccentricity,
    inclination_rad,
    raan_rad,
    arg_periapsis_rad,
    true_anomaly_rad,
    orbit_radius,
    inverse_semi_major_axis,
    periapsis_radius,
    apoapsis_radius,
});
