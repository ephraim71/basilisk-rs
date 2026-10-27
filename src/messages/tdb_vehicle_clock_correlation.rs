//! Vehicle-clock to TDB correlation (upstream
//! `TDBVehicleClockCorrelationMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// One correlated pair of vehicle time and ephemeris time (TDB): upstream
/// `TDBVehicleClockCorrelationMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct TdbVehicleClockCorrelationMsg {
    /// Upstream `ephemerisTime` \[s\]: ephemeris time (TDB) at `vehicle_clock_time_s`.
    pub ephemeris_time_s: f64,
    /// Upstream `vehicleClockTime` \[s\]: vehicle clock time, in seconds.
    pub vehicle_clock_time_s: f64,
}

impl_telemetry!(TdbVehicleClockCorrelationMsg {
    ephemeris_time_s,
    vehicle_clock_time_s,
});
