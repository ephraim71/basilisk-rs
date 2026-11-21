//! Radio link between two antennas (upstream `LinkBudgetMsgPayload`).

use serde::{Deserialize, Serialize};

use super::AntennaState;
use super::flatten::impl_telemetry;

/// Carrier-to-noise ratio at each end of a two-antenna link, with the link
/// geometry and band: upstream `LinkBudgetMsg`.
///
/// Upstream's `char antennaName1[20]` and `antennaName2[20]` are `String`s
/// here.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LinkBudgetMsg {
    /// Upstream `antennaName1`: name of antenna 1.
    pub antenna_name_1: String,
    /// Upstream `antennaName2`: name of antenna 2.
    pub antenna_name_2: String,
    /// Upstream `antennaState1` (`uint32_t`): mode of antenna 1.
    pub antenna_state_1: AntennaState,
    /// Upstream `antennaState2` (`uint32_t`): mode of antenna 2.
    pub antenna_state_2: AntennaState,
    /// Upstream `CNR1` \[-\]: linear carrier-to-noise ratio received at
    /// antenna 1; zero when antenna 1 does not receive from antenna 2.
    pub cnr_1: f64,
    /// Upstream `CNR2` \[-\]: linear carrier-to-noise ratio received at
    /// antenna 2; zero when antenna 2 does not receive from antenna 1.
    pub cnr_2: f64,
    /// Upstream `distance` \[m\]: distance between the antennas.
    pub distance_m: f64,
    /// Upstream `bandwidth` \[Hz\]: bandwidth both antennas share.
    pub bandwidth_hz: f64,
    /// Upstream `frequency` \[Hz\]: centre frequency of the shared band.
    pub frequency_hz: f64,
}

impl_telemetry!(LinkBudgetMsg {
    antenna_state_1,
    antenna_state_2,
    cnr_1,
    cnr_2,
    distance_m,
    bandwidth_hz,
    frequency_hz,
});
