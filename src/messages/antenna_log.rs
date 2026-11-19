//! Antenna state (upstream `AntennaLogMsgPayload`, with the state codes of
//! upstream `simulation/communication/_GeneralModuleFiles/AntennaDefinitions.h`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Where an antenna operates: upstream `AntennaTypes::EnvironmentType`,
/// carried in the payload's `uint32_t environment`.
///
/// Upstream's `_ENVIRONMENT_UNKNOWN = -1` marks an antenna whose environment
/// is not yet determined; upstream `simpleAntenna` rejects it at `Reset`, so
/// it is never published and has no variant here.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u32)]
pub enum AntennaEnvironment {
    /// On a spacecraft; upstream `ENVIRONMENT_SPACE = 0`, the zeroed default.
    #[default]
    Space = 0,
    /// At a ground station on Earth; upstream `ENVIRONMENT_EARTH = 1`.
    Earth = 1,
}

/// What an antenna is doing: upstream `AntennaTypes::AntennaStateEnum`,
/// carried in the payload's `uint32_t antennaState`.
///
/// Upstream's `_ANTENNA_STATE_UNKNOWN = -1` likewise marks an unconfigured
/// antenna, which upstream `simpleAntenna` rejects at `Reset`; it has no
/// variant here.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u32)]
pub enum AntennaState {
    /// Off; upstream `ANTENNA_OFF = 0`, the zeroed default.
    #[default]
    Off = 0,
    /// Receiving; upstream `ANTENNA_RX = 1`.
    Rx = 1,
    /// Transmitting; upstream `ANTENNA_TX = 2`.
    Tx = 2,
    /// Receiving and transmitting; upstream `ANTENNA_RXTX = 3`.
    RxTx = 3,
}

/// State, link properties and pose of one antenna: upstream `AntennaLogMsg`,
/// published by upstream `simpleAntenna` (`antennaOutMsg`) and read by
/// `linkBudget` and `antennaPower`.
///
/// Upstream's `char antennaName[20]` is a `String` here, and is not part of
/// the telemetry.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AntennaLogMsg {
    /// Upstream `antennaName`: antenna name.
    pub antenna_name: String,
    /// Upstream `environment`: space or ground antenna.
    pub environment: AntennaEnvironment,
    /// Upstream `antennaState`: off, receiving, transmitting or both.
    pub antenna_state: AntennaState,
    /// Upstream `frequency` \[Hz\]: operating frequency.
    pub frequency_hz: f64,
    /// Upstream `B` \[Hz\]: frequency bandwidth.
    pub bandwidth_hz: f64,
    /// Upstream `HPBW_az` \[rad\]: half-power beamwidth in azimuth.
    pub hpbw_az_rad: f64,
    /// Upstream `HPBW_el` \[rad\]: half-power beamwidth in elevation.
    pub hpbw_el_rad: f64,
    /// Upstream `P_Tx` \[W\]: transmit power.
    pub p_tx_w: f64,
    /// Upstream `P_Rx` \[W\]: receive power.
    pub p_rx_w: f64,
    /// Upstream `DdB` \[dB\]: antenna directivity.
    pub directivity_db: f64,
    /// Upstream `eta_r` [-]: antenna radiation efficiency.
    pub eta_r: f64,
    /// Upstream `P_N` \[W\]: noise power at the receiver.
    pub p_n_w: f64,
    /// Upstream `P_eirp_dB` \[dBW\]: equivalent isotropically radiated power
    /// (the payload comment says W; `simpleAntenna` computes it in dB).
    pub p_eirp_db: f64,
    /// Upstream `G_TN` \[dB/K\]: antenna gain over system noise temperature.
    pub g_tn_db_per_k: f64,
    /// Upstream `T_Ambient` \[K\]: ambient temperature around the antenna.
    pub t_ambient_k: f64,
    /// Upstream `r_AN_N` \[m\]: antenna position relative to the inertial
    /// origin, inertial components.
    pub r_an_n_m: Vector3<f64>,
    /// Upstream `sigma_AN` [-]: MRP attitude of the antenna frame relative to
    /// the inertial frame.
    pub sigma_an: Vector3<f64>,
    /// Upstream `v_AN_N` \[m/s\]: antenna velocity, inertial components.
    pub v_an_n_mps: Vector3<f64>,
    /// Upstream `r_AP_N` \[m\]: antenna position relative to the celestial
    /// body frame `P`, inertial components (ground antennas only).
    pub r_ap_n_m: Vector3<f64>,
    /// Upstream `r_AP_P` \[m\]: the same position in body-fixed `P`
    /// components (ground antennas only).
    pub r_ap_p_m: Vector3<f64>,
    /// Upstream `nHat_LP_N` [-]: surface normal at the ground location,
    /// inertial components (ground antennas only).
    pub n_hat_lp_n: Vector3<f64>,
}

impl_telemetry!(enum AntennaEnvironment as u32);
impl_telemetry!(enum AntennaState as u32);
impl_telemetry!(AntennaLogMsg {
    environment,
    antenna_state,
    frequency_hz,
    bandwidth_hz,
    hpbw_az_rad,
    hpbw_el_rad,
    p_tx_w,
    p_rx_w,
    directivity_db,
    eta_r,
    p_n_w,
    p_eirp_db,
    g_tn_db_per_k,
    t_ambient_k,
    r_an_n_m,
    sigma_an,
    v_an_n_mps,
    r_ap_n_m,
    r_ap_p_m,
    n_hat_lp_n,
});
