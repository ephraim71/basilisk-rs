//! SPICE time sampling (upstream `SpiceTimeMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// The current time in several time scales: upstream `SpiceTimeMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SpiceTimeMsg {
    /// Upstream `J2000Current` \[s\]: ephemeris time (TDB) elapsed since
    /// J2000.
    pub j2000_current_s: f64,
    /// Upstream `JulianDateCurrent` \[d\]: UTC Julian date, held at the
    /// following midnight during an inserted leap second (the SPICE JDUTC
    /// convention).
    pub julian_date_current_days: f64,
    /// Upstream `GPSSeconds` \[s\]: GPS seconds of week.
    pub gps_seconds_of_week: f64,
    /// Upstream `GPSWeek` (`uint16_t`): GPS week, modulo 1024.
    pub gps_week: u32,
    /// Upstream `GPSRollovers`: number of 1024-week GPS rollovers.
    pub gps_rollovers: u64,
}

impl_telemetry!(SpiceTimeMsg {
    j2000_current_s,
    julian_date_current_days,
    gps_seconds_of_week,
    gps_week,
    gps_rollovers,
});
