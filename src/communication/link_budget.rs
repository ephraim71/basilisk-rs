//! Radio link budget between two antennas: port of upstream Basilisk
//! `simulation/communication/linkBudget`.
//!
//! The module reads two [`AntennaLogMsg`]s (typically from
//! [`super::simple_antenna`]) and publishes, in a [`LinkBudgetMsg`], the
//! carrier-to-noise ratio received at each antenna from the other:
//!
//! * the shared band of the two antennas, and the frequency-offset loss of a
//!   partial overlap (no overlap is no link);
//! * the free-space path loss at the shared band's centre;
//! * for a space-to-ground link with [`LinkBudget::atmos_att`], the gaseous
//!   attenuation of ITU-R P.676-13 integrated along the ray through an
//!   ITU-R P.835 atmosphere, refracted by Snell's law for a spherically
//!   stratified atmosphere, from the ground antenna to 100 km;
//! * the Gaussian-beam pointing loss of both antennas (of the spacecraft
//!   antenna only on a space-to-ground link, the ground antenna being
//!   assumed to track it);
//! * the received power of each antenna that receives while the other
//!   transmits, over its noise power.
//!
//! The link is set up on the first update at which both inputs are written:
//! the antenna placement (space-to-space or space-to-ground; ground-to-ground
//! is not supported) and, with atmospheric attenuation, the layer table of
//! the ground antenna's altitude. Until then, and while a ground antenna has
//! not published its position, the output is zero. Init and reset clear the
//! set-up and publish a zero output.
//!
//! # Differences from upstream
//!
//! * No message headers. An input counts as written once it carries a
//!   positive frequency: an unwritten [`AntennaLogMsg`] is zeroed, and a
//!   producer always publishes one (`SimpleAntenna` requires 30 MHz to
//!   60 GHz).
//! * The ITU-R P.676-13 line tables (upstream's `oxygen.json` and
//!   `waterVapour.json`) are built in, so no data file is needed; upstream's
//!   file-path setters still load replacement JSON tables, parsed as JSON
//!   rather than line by line.
//! * Upstream's run-time `bskError`s (a ground-to-ground link, a zero
//!   distance, a missing gas table) are logged; a ground-to-ground link then
//!   publishes a zero output.

use std::path::PathBuf;

use nalgebra::Vector3;

use super::spectroscopic_data::{OXYGEN_LINES, SpectralLine, WATER_VAPOUR_LINES};
use crate::messages::{
    ANTENNA_NAME_SIZE, AntennaEnvironment, AntennaLogMsg, Input, LinkBudgetMsg, Output,
};
use crate::utilities::astro_constants::{REQ_EARTH, SPEED_LIGHT};
use crate::utilities::itu_ref_atmosphere;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Upstream `h_Toa` \[m\]: top of the atmosphere of the attenuation
/// integration, above the spherical Earth.
const TOP_OF_ATMOSPHERE_M: f64 = 100.0e3;
/// Upstream `MIN_SIN_ELEVATION` \[-\]: sine of the 5 degree elevation the
/// per-layer path is clamped to.
const MIN_SIN_ELEVATION: f64 = 0.0872;
/// ITU-R P.676-13 eq. 14: thickness of the first, sea-level layer \[m\].
const FIRST_LAYER_THICKNESS_M: f64 = 0.1;
/// ITU-R P.676-13 eq. 14: layers per e-fold increase in thickness.
const LAYER_SCALE: f64 = 100.0;

/// Upstream `LinkBudgetTypes::AntennaPlacement`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AntennaPlacement {
    /// Upstream `SPACE_SPACE`: both antennas in space.
    SpaceSpace,
    /// Upstream `SPACE_EARTH`: one antenna on the ground.
    SpaceEarth,
    /// Upstream `_UNKNOWN_BLOCKED`: not set up, or not supported.
    UnknownBlocked,
}

/// Which input carries an antenna.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Slot {
    First,
    Second,
}

/// Upstream `LinkBudgetTypes::GasType`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum GasType {
    Oxygen,
    WaterVapour,
}

/// Upstream `AttenuationLookupTable`: the atmosphere sampled at the midpoint
/// of each layer.
#[derive(Clone, Debug, Default, PartialEq)]
struct AttenuationLookupTable {
    /// Upstream `layer_i` \[m\]: sample altitudes.
    layer_i: Vec<f64>,
    /// Upstream `gamma_i` \[dB/km\]: specific attenuation.
    gamma_i: Vec<f64>,
    /// Upstream `delta_h_layer_i` \[m\]: layer thicknesses.
    delta_h_layer_i: Vec<f64>,
    /// Upstream `n_i` \[-\]: refractive index.
    n_i: Vec<f64>,
}

impl AttenuationLookupTable {
    fn clear(&mut self) {
        self.layer_i.clear();
        self.gamma_i.clear();
        self.delta_h_layer_i.clear();
        self.n_i.clear();
    }
}

/// Link budget of two antennas: upstream `LinkBudget`.
#[derive(Clone, Debug)]
pub struct LinkBudget {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `atmosAtt`: include atmospheric attenuation on a
    /// space-to-ground link. Off by default; set-up clears it for a
    /// space-to-space link.
    pub atmos_att: bool,
    /// Upstream `pointingLoss`: include the pointing loss (default on).
    pub pointing_loss: bool,
    /// Upstream `freqLoss`: include the frequency-offset loss (default on).
    /// Off, the shared band is the smaller of the two bandwidths, at the mean
    /// frequency.
    pub freq_loss: bool,
    /// Upstream `antennaInPayload_1`: antenna 1.
    pub antenna_in_msg_1: Input<AntennaLogMsg>,
    /// Upstream `antennaInPayload_2`: antenna 2.
    pub antenna_in_msg_2: Input<AntennaLogMsg>,
    /// Upstream `linkBudgetOutPayload`: the link budget.
    pub link_budget_out_msg: Output<LinkBudgetMsg>,
    /// Upstream `oxygenLookupFilePath`: replacement oxygen table, if any.
    oxygen_lookup_file_path: Option<PathBuf>,
    /// Upstream `waterVaporLookupFilePath`: replacement water-vapour table,
    /// if any.
    water_vapour_lookup_file_path: Option<PathBuf>,
    /// Upstream `initialized`.
    initialized: bool,
    /// Upstream `waitingWarningIssued`.
    waiting_warning_issued: bool,
    /// Upstream `antennaPlacement`.
    antenna_placement: AntennaPlacement,
    /// Upstream `gndAntPnt`: the input carrying the ground antenna.
    ground_slot: Option<Slot>,
    /// Upstream `antennaIn_1`.
    antenna_in_1: AntennaLogMsg,
    /// Upstream `antennaIn_2`.
    antenna_in_2: AntennaLogMsg,
    /// Upstream `distance` \[m\].
    distance_m: f64,
    /// Upstream `L_FSPL` \[dB\].
    l_fspl_db: f64,
    /// Upstream `L_atm` \[dB\].
    l_atm_db: f64,
    /// Upstream `L_freq` \[dB\].
    l_freq_db: f64,
    /// Upstream `L_point` \[dB\].
    l_point_db: f64,
    /// Upstream `P_Rx1` \[dBW\].
    p_rx_1_dbw: f64,
    /// Upstream `P_Rx2` \[dBW\].
    p_rx_2_dbw: f64,
    /// Upstream `CNR1` \[-\].
    cnr_1: f64,
    /// Upstream `CNR2` \[-\].
    cnr_2: f64,
    /// Upstream `B_overlap` \[Hz\].
    b_overlap_hz: f64,
    /// Upstream `centerfreq` \[Hz\].
    center_frequency_hz: f64,
    /// Upstream `oxygenLookup`.
    oxygen_lookup: Vec<SpectralLine>,
    /// Upstream `waterVaporLookup`.
    water_vapour_lookup: Vec<SpectralLine>,
    /// Upstream `attenuationLookup`.
    attenuation_lookup: AttenuationLookupTable,
    /// Upstream `h_0` \[m\]: altitude of the ground antenna.
    h_0_m: f64,
    /// Upstream `n_0` \[-\]: refractive index at the ground antenna.
    n_0: f64,
    /// Upstream `linkValid`: the antennas share a band.
    link_valid: bool,
}

impl LinkBudget {
    /// A link budget with pointing and frequency-offset losses, without
    /// atmospheric attenuation, and with unconnected ports (upstream's
    /// constructor).
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            atmos_att: false,
            pointing_loss: true,
            freq_loss: true,
            antenna_in_msg_1: Input::default(),
            antenna_in_msg_2: Input::default(),
            link_budget_out_msg: Output::default(),
            oxygen_lookup_file_path: None,
            water_vapour_lookup_file_path: None,
            initialized: false,
            waiting_warning_issued: false,
            antenna_placement: AntennaPlacement::UnknownBlocked,
            ground_slot: None,
            antenna_in_1: AntennaLogMsg::default(),
            antenna_in_2: AntennaLogMsg::default(),
            distance_m: 0.0,
            l_fspl_db: 0.0,
            l_atm_db: 0.0,
            l_freq_db: 0.0,
            l_point_db: 0.0,
            p_rx_1_dbw: 0.0,
            p_rx_2_dbw: 0.0,
            cnr_1: -1.0,
            cnr_2: -1.0,
            b_overlap_hz: 0.0,
            center_frequency_hz: 0.0,
            oxygen_lookup: Vec::new(),
            water_vapour_lookup: Vec::new(),
            attenuation_lookup: AttenuationLookupTable::default(),
            h_0_m: -1.0,
            n_0: -1.0,
            link_valid: false,
        }
    }

    /// Upstream `getL_FSPL` \[dB\]: free-space path loss.
    pub fn l_fspl_db(&self) -> f64 {
        self.l_fspl_db
    }

    /// Upstream `getL_atm` \[dB\]: atmospheric attenuation.
    pub fn l_atm_db(&self) -> f64 {
        self.l_atm_db
    }

    /// Upstream `getL_freq` \[dB\]: frequency-offset loss.
    pub fn l_freq_db(&self) -> f64 {
        self.l_freq_db
    }

    /// Upstream `getL_point` \[dB\]: pointing loss.
    pub fn l_point_db(&self) -> f64 {
        self.l_point_db
    }

    /// Upstream `getCNR1` \[-\]: carrier-to-noise ratio at antenna 1.
    pub fn cnr_1(&self) -> f64 {
        self.cnr_1
    }

    /// Upstream `getCNR2` \[-\]: carrier-to-noise ratio at antenna 2.
    pub fn cnr_2(&self) -> f64 {
        self.cnr_2
    }

    /// The antenna placement found at set-up.
    pub fn antenna_placement(&self) -> AntennaPlacement {
        self.antenna_placement
    }

    /// Upstream `setOxygenLookupFilePath`: a JSON table of the oxygen lines
    /// (`{"data": [{"f0": ..., "a1": ..., ..., "a6": ...}, ...]}`, as
    /// upstream's `oxygen.json`) to use instead of the built-in ITU-R P.676-13
    /// Table 1, loaded at set-up.
    pub fn set_oxygen_lookup_file_path(&mut self, path: impl Into<PathBuf>) {
        self.oxygen_lookup_file_path = Some(path.into());
    }

    /// Upstream `setWaterVaporLookupFilePath`: as
    /// [`Self::set_oxygen_lookup_file_path`], for the water-vapour lines
    /// (`b1` ... `b6`, as upstream's `waterVapour.json`).
    pub fn set_water_vapor_lookup_file_path(&mut self, path: impl Into<PathBuf>) {
        self.water_vapour_lookup_file_path = Some(path.into());
    }

    /// The antenna on input `slot`.
    fn antenna(&self, slot: Slot) -> &AntennaLogMsg {
        match slot {
            Slot::First => &self.antenna_in_1,
            Slot::Second => &self.antenna_in_2,
        }
    }

    /// Upstream `gndAntPnt` and `scAntPnt`: the ground and spacecraft
    /// antennas of a space-to-ground link.
    fn ground_and_spacecraft(&self) -> Option<(&AntennaLogMsg, &AntennaLogMsg)> {
        let ground = self.ground_slot?;
        let spacecraft = match ground {
            Slot::First => Slot::Second,
            Slot::Second => Slot::First,
        };
        Some((self.antenna(ground), self.antenna(spacecraft)))
    }

    /// Upstream `Reset`.
    fn reset_state(&mut self) {
        if !self.antenna_in_msg_1.is_connected() {
            log::error!(
                "{}: LinkBudget.antennaInPayload_1 was not linked.",
                self.name
            );
        }
        if !self.antenna_in_msg_2.is_connected() {
            log::error!(
                "{}: LinkBudget.antennaInPayload_2 was not linked.",
                self.name
            );
        }
        // Producers need not publish at reset, and a written input can still
        // hold the previous run's geometry until its producer updates.
        self.initialized = false;
        self.waiting_warning_issued = false;
        self.ground_slot = None;
        self.link_valid = false;
        self.l_fspl_db = 0.0;
        self.l_atm_db = 0.0;
        self.l_freq_db = 0.0;
        self.l_point_db = 0.0;
        self.p_rx_1_dbw = 0.0;
        self.p_rx_2_dbw = 0.0;
        self.cnr_1 = 0.0;
        self.cnr_2 = 0.0;
        self.distance_m = 0.0;
        self.b_overlap_hz = 0.0;
        self.center_frequency_hz = 0.0;
        self.link_budget_out_msg.write(LinkBudgetMsg::default());
    }

    /// Upstream `initialization`: the antenna placement and, with
    /// atmospheric attenuation, the ground antenna's atmosphere.
    fn initialization(&mut self) {
        use AntennaEnvironment::{Earth, Space};
        let environments = (self.antenna_in_1.environment, self.antenna_in_2.environment);
        (self.antenna_placement, self.ground_slot) = match environments {
            (Earth, Earth) => {
                log::error!(
                    "{}: This module does currently not support link budget calculations between \
                     two ground stations.",
                    self.name
                );
                (AntennaPlacement::UnknownBlocked, None)
            }
            (Earth, Space) => (AntennaPlacement::SpaceEarth, Some(Slot::First)),
            (Space, Earth) => (AntennaPlacement::SpaceEarth, Some(Slot::Second)),
            (Space, Space) => {
                // No atmosphere between two spacecraft.
                self.atmos_att = false;
                (AntennaPlacement::SpaceSpace, None)
            }
        };

        if self.atmos_att
            && let Some((ground, _)) = self.ground_and_spacecraft()
        {
            // Guard against a slightly negative altitude from the Earth radius
            // constant and round-off.
            let h0 = (ground.r_ap_n_m.norm() - REQ_EARTH * 1e3).max(0.0);
            let frequency_hz = ground.frequency_hz;
            self.h_0_m = h0;
            let t_0 = itu_ref_atmosphere::temperature_k(h0);
            let p_0 = itu_ref_atmosphere::pressure_hpa(h0);
            let rho_0 = itu_ref_atmosphere::water_vapour_density_g_per_m3(h0);
            // ITU-R P.453-14 eqs. 10, 1 and 6.
            let e_0 = (rho_0 * t_0) / 216.7;
            self.n_0 = 1.0
                + (77.6 * (p_0 / t_0) - 5.6 * (e_0 / t_0) + 3.75e5 * (e_0 / (t_0 * t_0))) * 1e-6;

            self.oxygen_lookup = self.generate_lookup_table(GasType::Oxygen);
            self.water_vapour_lookup = self.generate_lookup_table(GasType::WaterVapour);
            self.precompute_atmospheric_attenuation_at_layers(frequency_hz);
        }
    }

    /// Upstream `generateLookupTable`: the configured JSON table, or the
    /// built-in one.
    fn generate_lookup_table(&self, gas: GasType) -> Vec<SpectralLine> {
        let (path, built_in) = match gas {
            GasType::Oxygen => (&self.oxygen_lookup_file_path, &OXYGEN_LINES[..]),
            GasType::WaterVapour => (&self.water_vapour_lookup_file_path, &WATER_VAPOUR_LINES[..]),
        };
        let Some(path) = path else {
            return built_in.to_vec();
        };
        let lines = std::fs::read_to_string(path)
            .map_err(|error| {
                format!(
                    "Unable to open lookup table file {}: {error}",
                    path.display()
                )
            })
            .and_then(|text| parse_lookup_table(&text, gas));
        match lines {
            Ok(lines) => {
                if lines.is_empty() {
                    log::warn!(
                        "{}: LinkBudget: No data loaded from {}",
                        self.name,
                        path.display()
                    );
                }
                lines
            }
            Err(message) => {
                log::error!("{}: LinkBudget: {message}", self.name);
                Vec::new()
            }
        }
    }

    /// Upstream `precomputeAtmosphericAttenuationAtLayers`: the specific
    /// attenuation and refractive index at the midpoint of each layer from
    /// the ground antenna to the top of the atmosphere, for `frequency_hz`.
    ///
    /// The layers thicken exponentially with altitude (ITU-R P.676-13 eqs.
    /// 14-16, normalized to the column), and are split at every branch of
    /// the piecewise atmosphere so that no sample straddles one.
    fn precompute_atmospheric_attenuation_at_layers(&mut self, frequency_hz: f64) {
        // Repeated set-ups rebuild a consistent table.
        self.attenuation_lookup.clear();
        let h_0 = self.h_0_m;
        if h_0 >= TOP_OF_ATMOSPHERE_M {
            return;
        }

        let layer_growth = (1.0 / LAYER_SCALE).exp_m1();
        let lower_index = (LAYER_SCALE * (h_0 * layer_growth / FIRST_LAYER_THICKNESS_M).ln_1p()
            + 1.0)
            .floor() as i64;
        let upper_index = (LAYER_SCALE
            * (TOP_OF_ATMOSPHERE_M * layer_growth / FIRST_LAYER_THICKNESS_M).ln_1p()
            + 1.0)
            .ceil() as i64;
        let height_scale = (TOP_OF_ATMOSPHERE_M - h_0)
            / ((upper_index - lower_index) as f64 / LAYER_SCALE).exp_m1();

        let mut layer_tops: Vec<f64> = (lower_index..upper_index)
            .map(|index| {
                if index + 1 == upper_index {
                    TOP_OF_ATMOSPHERE_M
                } else {
                    h_0 + height_scale * ((index + 1 - lower_index) as f64 / LAYER_SCALE).exp_m1()
                }
            })
            .collect();
        layer_tops.extend(
            itu_ref_atmosphere::layer_boundaries_m()
                .into_iter()
                .filter(|&boundary| boundary > h_0 && boundary < TOP_OF_ATMOSPHERE_M),
        );
        layer_tops.sort_by(f64::total_cmp);
        layer_tops.dedup();

        let f_ghz = frequency_hz / 1.0e9;
        let mut layer_bottom = h_0;
        for layer_top in layer_tops {
            let delta_h = layer_top - layer_bottom;
            let h_i = layer_bottom + 0.5 * delta_h;
            let t_i = itu_ref_atmosphere::temperature_k(h_i);
            let p_i = itu_ref_atmosphere::pressure_hpa(h_i);
            let rho_i = itu_ref_atmosphere::water_vapour_density_g_per_m3(h_i);
            // ITU-R P.676-13 eqs. 3 and 4.
            let theta = 300.0 / t_i;
            let e_i = (rho_i * t_i) / 216.7;

            let mut n_oxygen = 0.0;
            for line in &self.oxygen_lookup {
                let [a1, a2, a3, a4, a5, a6] = line.c;
                let f_j = line.f0_ghz;
                // Line strength (eq. 3), width (eqs. 6a, 6b) and correction
                // (eq. 7), then the line shape (eq. 5).
                let strength = a1 * 1.0e-7 * p_i * theta.powf(3.0) * (a2 * (1.0 - theta)).exp();
                let width = a3 * 1.0e-4 * (p_i * theta.powf(0.8 - a4) + 1.1 * e_i * theta);
                let width = (width * width + 2.25e-6).sqrt();
                let correction = (a5 + a6 * theta) * 1.0e-4 * (p_i + e_i) * theta.powf(0.8);
                let den_1 = (f_j - f_ghz) * (f_j - f_ghz) + width * width;
                let den_2 = (f_j + f_ghz) * (f_j + f_ghz) + width * width;
                let shape = f_ghz / f_j
                    * ((width - correction * (f_j - f_ghz)) / den_1
                        + (width - correction * (f_j + f_ghz)) / den_2);
                n_oxygen += strength * shape;
            }
            let mut n_water = 0.0;
            for line in &self.water_vapour_lookup {
                let [b1, b2, b3, b4, b5, b6] = line.c;
                let f_j = line.f0_ghz;
                let strength = b1 * 1.0e-1 * e_i * theta.powf(3.5) * (b2 * (1.0 - theta)).exp();
                let width = b3 * 1.0e-4 * (p_i * theta.powf(b4) + b5 * e_i * theta.powf(b6));
                let width = 0.535 * width
                    + (0.217 * width * width + (2.1316e-12 * f_j * f_j) / theta).sqrt();
                let den = (f_j - f_ghz) * (f_j - f_ghz) + width * width;
                let shape = f_ghz / f_j * 2.0 * (width / den);
                n_water += strength * shape;
            }
            // Debye spectrum width (eq. 9) and dry-air continuum (eq. 8).
            let d = 5.6e-4 * (p_i + e_i) * theta.powf(0.8);
            let n_dry = f_ghz
                * p_i
                * theta
                * theta
                * ((6.14e-5) / (d * (1.0 + ((f_ghz * f_ghz) / (d * d))))
                    + (1.4e-12 * p_i * theta.powf(1.5)) / (1.0 + 1.9e-5 * f_ghz.powf(1.5)));
            // Specific attenuation (eq. 1) and refractive index (ITU-R
            // P.453-12 eqs. 1 and 6).
            let gamma = 0.1820 * f_ghz * (n_oxygen + n_water + n_dry);
            let n_i = 1.0
                + (77.6 * (p_i / t_i) - 5.6 * (e_i / t_i) + 3.75e5 * (e_i / (t_i * t_i))) * 1e-6;

            self.attenuation_lookup.layer_i.push(h_i);
            self.attenuation_lookup.gamma_i.push(gamma);
            self.attenuation_lookup.delta_h_layer_i.push(delta_h);
            self.attenuation_lookup.n_i.push(n_i);
            layer_bottom = layer_top;
        }
    }

    /// Upstream `atmosphereInputsReady`: atmosphere is off, or every ground
    /// antenna has published a position.
    fn atmosphere_inputs_ready(&self) -> bool {
        !self.atmos_att
            || [&self.antenna_in_1, &self.antenna_in_2]
                .iter()
                .all(|antenna| {
                    antenna.environment != AntennaEnvironment::Earth
                        || antenna.r_ap_n_m != Vector3::zeros()
                })
    }

    /// Upstream `calculateLinkBudget`.
    fn calculate_link_budget(&mut self) {
        if self.freq_loss {
            self.calculate_frequency_offset_loss();
        } else {
            // Full overlap of the narrower band, at the mean frequency.
            self.b_overlap_hz = self
                .antenna_in_1
                .bandwidth_hz
                .min(self.antenna_in_2.bandwidth_hz);
            self.center_frequency_hz =
                (self.antenna_in_1.frequency_hz + self.antenna_in_2.frequency_hz) / 2.0;
            if self.b_overlap_hz > 0.0 {
                self.link_valid = true;
            }
        }
        self.calculate_fspl();
        if self.atmos_att {
            self.calculate_atmospheric_loss();
        }
        if self.pointing_loss {
            self.calculate_pointing_loss();
        }
    }

    /// Upstream `calculateFSPL`.
    fn calculate_fspl(&mut self) {
        self.distance_m = (self.antenna_in_1.r_an_n_m - self.antenna_in_2.r_an_n_m).norm();
        if self.distance_m <= 0.0 {
            log::error!(
                "{}: LinkBudget: Distance between antennas is zero or negative.",
                self.name
            );
        }
        if self.distance_m <= 10.0 * 1e3 {
            log::warn!(
                "{}: LinkBudget: You are calculating a linkBudget for a distance below 10 km. The \
                 gaussian-beam assumption might be inaccurate at this range.",
                self.name
            );
        }
        let wavelength_m = SPEED_LIGHT / self.center_frequency_hz;
        self.l_fspl_db =
            20.0 * ((4.0 * std::f64::consts::PI * self.distance_m) / wavelength_m).log10();
    }

    /// Upstream `calculateAtmosphericLoss`: the layer attenuations along the
    /// refracted ray, each over its slant path (ITU-R P.676-13 eqs. 11-12).
    fn calculate_atmospheric_loss(&mut self) {
        let Some((ground, spacecraft)) = self.ground_and_spacecraft() else {
            return;
        };
        let cos_slant_0 =
            calc_cos_slant_ground(ground.r_an_n_m, ground.n_hat_lp_n, spacecraft.r_an_n_m)
                .clamp(-1.0, 1.0);
        if (1.0 - cos_slant_0 * cos_slant_0).max(0.0).sqrt() < MIN_SIN_ELEVATION {
            log::warn!(
                "{}: LinkBudget: Elevation angle below 5 deg, clamping for atmospheric \
                 calculation.",
                self.name
            );
        }

        let radius_m = REQ_EARTH * 1e3;
        let table = &self.attenuation_lookup;
        let mut attenuation_db = 0.0;
        for i in 0..table.layer_i.len() {
            // Every layer refers to the ground ray invariant (eq. 12).
            let cos_slant = (((radius_m + self.h_0_m) * self.n_0)
                / ((radius_m + table.layer_i[i]) * table.n_i[i])
                * cos_slant_0)
                .clamp(-1.0, 1.0);
            let sin_slant = (1.0 - cos_slant * cos_slant)
                .max(0.0)
                .sqrt()
                .max(MIN_SIN_ELEVATION);
            attenuation_db += table.gamma_i[i] * table.delta_h_layer_i[i] * 1e-3 / sin_slant;
        }
        self.l_atm_db = attenuation_db;
    }

    /// Upstream `calculatePointingLoss`: Gaussian-beam loss of each antenna
    /// off its boresight; a ground antenna is assumed to track the
    /// spacecraft.
    fn calculate_pointing_loss(&mut self) {
        let gaussian_loss = |antenna: &AntennaLogMsg, errors: [f64; 2]| {
            10.0 * std::f64::consts::E.log10()
                * 4.0
                * 2f64.ln()
                * ((errors[0] * errors[0]) / (antenna.hpbw_az_rad * antenna.hpbw_az_rad)
                    + (errors[1] * errors[1]) / (antenna.hpbw_el_rad * antenna.hpbw_el_rad))
        };
        let loss = match self.ground_and_spacecraft() {
            None => {
                let (first, second) = (&self.antenna_in_1, &self.antenna_in_2);
                let errors_1 = self.pointing_error(first.r_an_n_m, second.r_an_n_m, first.sigma_an);
                let errors_2 =
                    self.pointing_error(second.r_an_n_m, first.r_an_n_m, second.sigma_an);
                gaussian_loss(first, errors_1) + gaussian_loss(second, errors_2)
            }
            Some((ground, spacecraft)) => {
                let errors =
                    self.pointing_error(spacecraft.r_an_n_m, ground.r_an_n_m, spacecraft.sigma_an);
                gaussian_loss(spacecraft, errors)
            }
        };
        self.l_point_db = loss;
    }

    /// Upstream `getPointingError` \[rad\]: azimuth and elevation of antenna
    /// 2 off the boresight (+z) of antenna 1, whose attitude is `sigma_an_1`.
    fn pointing_error(
        &self,
        r_a1n_n_m: Vector3<f64>,
        r_a2n_n_m: Vector3<f64>,
        sigma_an_1: Vector3<f64>,
    ) -> [f64; 2] {
        let r_a2a1_n = r_a2n_n_m - r_a1n_n_m;
        let distance = r_a2a1_n.norm();
        if distance <= f64::EPSILON {
            log::warn!(
                "{}: LinkBudget.getPointingError: antenna positions are coincident; returning \
                 zero pointing errors.",
                self.name
            );
            return [0.0, 0.0];
        }
        let n_a2a1_a1 = mrp_to_dcm(sigma_an_1) * (r_a2a1_n / distance);
        [
            n_a2a1_a1[0].atan2(n_a2a1_a1[2]),
            n_a2a1_a1[1].atan2(n_a2a1_a1[2]),
        ]
    }

    /// Upstream `calculateFrequencyOffsetLoss`: the shared band, and the loss
    /// of sharing less than the narrower band.
    fn calculate_frequency_offset_loss(&mut self) {
        let (first, second) = (&self.antenna_in_1, &self.antenna_in_2);
        let f_low = (first.frequency_hz - first.bandwidth_hz / 2.0)
            .max(second.frequency_hz - second.bandwidth_hz / 2.0);
        let f_high = (first.frequency_hz + first.bandwidth_hz / 2.0)
            .min(second.frequency_hz + second.bandwidth_hz / 2.0);
        let narrower_bandwidth = first.bandwidth_hz.min(second.bandwidth_hz);
        self.b_overlap_hz = f_high - f_low;
        self.center_frequency_hz = (f_high + f_low) / 2.0;

        if self.b_overlap_hz < narrower_bandwidth && self.b_overlap_hz > 0.0 {
            self.l_freq_db = 10.0 * (narrower_bandwidth / self.b_overlap_hz).log10();
            self.link_valid = true;
        } else if self.b_overlap_hz <= 0.0 {
            log::warn!(
                "{}: LinkBudget: No overlapping bandwidth between the two antennas.",
                self.name
            );
            self.link_valid = false;
        } else {
            self.l_freq_db = 0.0;
            self.link_valid = true;
        }
    }

    /// Upstream `calculateCNR`: at each antenna that receives while the
    /// other transmits, the received power over the noise power; zero
    /// otherwise.
    fn calculate_cnr(&mut self) {
        if !self.link_valid {
            self.cnr_1 = 0.0;
            self.cnr_2 = 0.0;
            return;
        }
        let (l_fspl, l_atm, l_point, l_freq) = (
            self.l_fspl_db,
            self.l_atm_db,
            self.l_point_db,
            self.l_freq_db,
        );
        // Received power [dBW] of `receiver` from `transmitter`, and its CNR.
        let received = |receiver: &AntennaLogMsg, transmitter: &AntennaLogMsg| {
            let gain_db = receiver.directivity_db + 10.0 * receiver.eta_r.log10();
            let p_rx_dbw = transmitter.p_eirp_db + gain_db - l_fspl - l_atm - l_point - l_freq;
            let noise_dbw = 10.0 * receiver.p_n_w.log10();
            (p_rx_dbw, 10f64.powf((p_rx_dbw - noise_dbw) / 10.0))
        };
        let (first, second) = (&self.antenna_in_1, &self.antenna_in_2);
        let at_first = (first.antenna_state.receives() && second.antenna_state.transmits())
            .then(|| received(first, second));
        let at_second = (second.antenna_state.receives() && first.antenna_state.transmits())
            .then(|| received(second, first));
        match at_first {
            Some((p_rx_dbw, cnr)) => (self.p_rx_1_dbw, self.cnr_1) = (p_rx_dbw, cnr),
            None => self.cnr_1 = 0.0,
        }
        match at_second {
            Some((p_rx_dbw, cnr)) => (self.p_rx_2_dbw, self.cnr_2) = (p_rx_dbw, cnr),
            None => self.cnr_2 = 0.0,
        }
    }

    /// Upstream `writeOutputMessages`.
    ///
    /// # Panics
    ///
    /// If an antenna name has [`ANTENNA_NAME_SIZE`] characters or more
    /// (upstream `bskError`).
    fn output_message(&self) -> LinkBudgetMsg {
        for (index, antenna) in [&self.antenna_in_1, &self.antenna_in_2].iter().enumerate() {
            assert!(
                antenna.antenna_name.len() < ANTENNA_NAME_SIZE,
                "LinkBudget: antennaName{} is not null-terminated within {ANTENNA_NAME_SIZE} \
                 characters.",
                index + 1
            );
        }
        LinkBudgetMsg {
            antenna_name_1: self.antenna_in_1.antenna_name.clone(),
            antenna_name_2: self.antenna_in_2.antenna_name.clone(),
            antenna_state_1: self.antenna_in_1.antenna_state,
            antenna_state_2: self.antenna_in_2.antenna_state,
            cnr_1: self.cnr_1,
            cnr_2: self.cnr_2,
            distance_m: self.distance_m,
            bandwidth_hz: self.b_overlap_hz,
            frequency_hz: self.center_frequency_hz,
        }
    }
}

/// Upstream `calcCosSlantGround`: cosine of the elevation of the spacecraft
/// antenna seen from the ground antenna at `r_agn_n_m` with surface normal
/// `n_ag_n` (ITU-R P.676-13 eq. 12).
fn calc_cos_slant_ground(
    r_agn_n_m: Vector3<f64>,
    n_ag_n: Vector3<f64>,
    r_asn_n_m: Vector3<f64>,
) -> f64 {
    let line_of_sight = (r_asn_n_m - r_agn_n_m).normalize();
    let v_n = line_of_sight.dot(&n_ag_n.normalize());
    let v_t = (1.0 - v_n * v_n).sqrt();
    v_n.atan2(v_t).cos()
}

/// Upstream `generateLookupTable`'s file parsing: the `data` rows of an
/// upstream gas table, `f0` and `a1` ... `a6` (oxygen) or `b1` ... `b6`
/// (water vapour).
fn parse_lookup_table(text: &str, gas: GasType) -> Result<Vec<SpectralLine>, String> {
    let (prefix, gas_name) = match gas {
        GasType::Oxygen => ('a', "oxygen"),
        GasType::WaterVapour => ('b', "water vapor"),
    };
    let table: serde_json::Value = serde_json::from_str(text)
        .map_err(|error| format!("Error parsing the {gas_name} lookup table: {error}"))?;
    let rows = table
        .get("data")
        .and_then(serde_json::Value::as_array)
        .ok_or_else(|| format!("The {gas_name} lookup table has no data array."))?;
    rows.iter()
        .map(|row| {
            let value = |key: &str| {
                row.get(key)
                    .and_then(serde_json::Value::as_f64)
                    .ok_or_else(|| format!("Error parsing {key} value in {gas_name} lookup table."))
            };
            let mut c = [0.0; 6];
            for (index, coefficient) in c.iter_mut().enumerate() {
                *coefficient = value(&format!("{prefix}{}", index + 1))?;
            }
            Ok(SpectralLine {
                f0_ghz: value("f0")?,
                c,
            })
        })
        .collect()
}

impl Module for LinkBudget {
    fn init(&mut self) {
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    /// Upstream `UpdateState`: wait for written inputs (and a ground
    /// position), set the link up once, then publish the budget.
    fn update(&mut self, _context: &SimulationContext) {
        let antenna_1 = self.antenna_in_msg_1.read();
        let antenna_2 = self.antenna_in_msg_2.read();
        // The stand-in for upstream's `isWritten`: a written antenna log
        // carries a positive frequency.
        let inputs_written = self.antenna_in_msg_1.is_connected()
            && self.antenna_in_msg_2.is_connected()
            && antenna_1.frequency_hz > 0.0
            && antenna_2.frequency_hz > 0.0;
        if inputs_written {
            self.antenna_in_1 = antenna_1;
            self.antenna_in_2 = antenna_2;
        }
        if !inputs_written || (!self.initialized && !self.atmosphere_inputs_ready()) {
            if !self.waiting_warning_issued {
                log::warn!(
                    "{}: LinkBudget: Waiting for written antenna inputs and a ground position. \
                     Check the order of the ground, antenna, and link-budget models if this link \
                     never starts.",
                    self.name
                );
                self.waiting_warning_issued = true;
            }
            self.link_budget_out_msg.write(LinkBudgetMsg::default());
            return;
        }

        if !self.initialized {
            self.initialization();
            self.initialized = true;
        }
        if self.antenna_placement == AntennaPlacement::UnknownBlocked {
            self.link_budget_out_msg.write(LinkBudgetMsg::default());
            return;
        }
        self.calculate_link_budget();
        self.calculate_cnr();
        self.link_budget_out_msg.write(self.output_message());
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `linkBudget/_UnitTest/test_linkBudget.py` with its
    //! expected values and tolerances, and the native gtest suites
    //! `test_linkBudgetAtmosphere.cpp` (its `ExposesAtmosphericBoundaries`
    //! case is in `utilities::itu_ref_atmosphere`) and
    //! `test_antennaProducerChain.cpp`. The upstream tests write the antenna
    //! payloads directly, as these do; the producer chain runs
    //! `GroundLocation` and `SimpleAntenna`.
    //!
    //! Not ported: `test_atmosphere_reference_checker_rejects_nonfinite`
    //! tests upstream's Python reference generator
    //! (`Support/generate_atmosphere_references.py`), not the module.
    //! `test_antennaProducerScheduling.py` is in `downlink_handling`, which it
    //! also covers.

    use nalgebra::{Matrix3, Rotation3, Vector3};

    use super::{AntennaPlacement, AttenuationLookupTable, LinkBudget, Slot, parse_lookup_table};
    use super::{GasType, TOP_OF_ATMOSPHERE_M};
    use crate::Module;
    use crate::communication::simple_antenna::{SimpleAntenna, SimpleAntennaConfig};
    use crate::environment::ground_location::{GroundLocation, GroundLocationConfig, SiteLocation};
    use crate::messages::{
        AntennaEnvironment, AntennaLogMsg, AntennaState, LinkBudgetMsg, Output, PlanetOrientation,
        PlanetStateMsg, SpacecraftStateMsg,
    };
    use crate::onboard_data_handling::test_support::{TaskClock, context};
    use crate::utilities::astro_constants::{D2R, K_BOLTZMANN, REQ_EARTH, SPEED_LIGHT};
    use crate::utilities::itu_ref_atmosphere::layer_boundaries_m;
    use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, euler1, euler2, mrp_to_dcm};

    use AntennaEnvironment::{Earth, Space};
    use AntennaState::{Off, Rx, RxTx, Tx};

    /// Upstream test's `REQ_EARTH` \[m\].
    const REQ_EARTH_M: f64 = 6378.1366e3;
    const ACCURACY: f64 = 1e-6;
    /// \[dB\] tolerance of the dB comparisons.
    const ACCURACY_DB: f64 = 0.1;
    /// Relative tolerance of the CNR comparisons.
    const ACCURACY_CNR_REL: f64 = 0.05;
    /// CNR of an invalid link or a non-receiving antenna.
    const CNR_INVALID: f64 = 0.0;
    /// Upstream `testProcessRate`, `macros.sec2nano(0.5)`.
    const STEP_NANOS: u64 = 500_000_000;

    // ---------------------------------------------------------------------
    // Upstream's analytical helpers
    // ---------------------------------------------------------------------

    fn compute_spacecraft_position_from_central_angle(
        re_m: f64,
        alt_m: f64,
        central_angle_rad: f64,
    ) -> Vector3<f64> {
        let r = re_m + alt_m;
        Vector3::new(
            r * central_angle_rad.cos(),
            r * central_angle_rad.sin(),
            0.0,
        )
    }

    fn compute_expected_overlap(f1: f64, b1: f64, f2: f64, b2: f64) -> (f64, f64) {
        let lo = (f1 - 0.5 * b1).max(f2 - 0.5 * b2);
        let hi = (f1 + 0.5 * b1).min(f2 + 0.5 * b2);
        let b_ov = hi - lo;
        (b_ov, if b_ov > 0.0 { 0.5 * (hi + lo) } else { 0.0 })
    }

    fn compute_fspl(distance_m: f64, frequency_hz: f64) -> f64 {
        let wavelength = SPEED_LIGHT / frequency_hz;
        20.0 * ((4.0 * std::f64::consts::PI * distance_m) / wavelength).log10()
    }

    fn compute_pointing_loss(theta_az: f64, theta_el: f64, hpbw_az: f64, hpbw_el: f64) -> f64 {
        let coeff = 10.0 * std::f64::consts::E.log10() * 4.0 * 2f64.ln();
        coeff
            * ((theta_az * theta_az) / (hpbw_az * hpbw_az)
                + (theta_el * theta_el) / (hpbw_el * hpbw_el))
    }

    fn compute_cnr(
        p_eirp_db: f64,
        g_rx_db: f64,
        l_fspl: f64,
        l_atm: f64,
        l_point: f64,
        l_freq: f64,
        p_n_watts: f64,
    ) -> f64 {
        let p_rx_dbw = p_eirp_db + g_rx_db - l_fspl - l_atm - l_point - l_freq;
        let cnr_db = p_rx_dbw - 10.0 * p_n_watts.log10();
        10f64.powf(cnr_db / 10.0)
    }

    /// Upstream `compute_antenna_derived_params`.
    struct Derived {
        hpbw_az: f64,
        hpbw_el: f64,
        p_eirp_db: f64,
        g_rx_db: f64,
        p_n: f64,
        t_s: f64,
    }

    #[allow(clippy::too_many_arguments)]
    fn compute_antenna_derived_params(
        directivity_db: f64,
        k: f64,
        p_tx: f64,
        eta_r: f64,
        t_e: f64,
        t_ambient: f64,
        t_sky: f64,
        bandwidth: f64,
    ) -> Derived {
        let dir_lin = 10f64.powf(directivity_db / 10.0);
        let hpbw_el = ((2f64.ln() * 16.0) / (dir_lin * k)).sqrt();
        let t_s = t_e + (1.0 - eta_r) * t_ambient + eta_r * t_sky;
        Derived {
            hpbw_az: k * hpbw_el,
            hpbw_el,
            p_eirp_db: 10.0 * p_tx.log10() + 10.0 * eta_r.log10() + directivity_db,
            g_rx_db: directivity_db + 10.0 * eta_r.log10(),
            p_n: K_BOLTZMANN * t_s * bandwidth,
            t_s,
        }
    }

    /// Upstream `compute_pointing_sigma`: MRP turning the boresight (+z)
    /// from `from_pos` toward `to_pos`.
    fn compute_pointing_sigma(from_pos: Vector3<f64>, to_pos: Vector3<f64>) -> Vector3<f64> {
        let direction = to_pos - from_pos;
        let dist = direction.norm();
        if dist < 1e-6 {
            return Vector3::zeros();
        }
        let d_hat = direction / dist;
        let z_hat = Vector3::z();
        let dot = z_hat.dot(&d_hat);
        if dot > 0.99999 {
            Vector3::zeros()
        } else if dot < -0.99999 {
            Vector3::x()
        } else {
            let axis = z_hat.cross(&d_hat);
            let axis = axis / axis.norm();
            let angle = dot.clamp(-1.0, 1.0).acos();
            (angle / 4.0).tan() * axis
        }
    }

    /// Upstream `apply_pointing_error`.
    fn apply_pointing_error(
        base_sigma: Vector3<f64>,
        error_az: f64,
        error_el: f64,
    ) -> Vector3<f64> {
        let dcm_error = euler2(error_az) * euler1(error_el);
        dcm_to_mrp(mrp_to_dcm(base_sigma) * dcm_error)
    }

    /// Upstream `create_antenna_msg_payload`'s arguments, with its defaults.
    #[derive(Clone)]
    struct Spec {
        name: &'static str,
        environment: AntennaEnvironment,
        state: AntennaState,
        frequency: f64,
        bandwidth: f64,
        directivity_db: f64,
        k: f64,
        p_tx: f64,
        eta_r: f64,
        t_e: f64,
        t_ambient: f64,
        t_sky: f64,
        position: Vector3<f64>,
        sigma: Option<Vector3<f64>>,
        r_ap_n: Vector3<f64>,
        n_hat_lp_n: Vector3<f64>,
        target_position: Option<Vector3<f64>>,
        pointing_error_az_deg: f64,
        pointing_error_el_deg: f64,
    }

    impl Default for Spec {
        fn default() -> Self {
            Self {
                name: "Antenna",
                environment: Space,
                state: Off,
                frequency: 2.2e9,
                bandwidth: 5e6,
                directivity_db: 20.0,
                k: 1.0,
                p_tx: 100.0,
                eta_r: 0.6,
                t_e: 50.0,
                t_ambient: 150.0,
                t_sky: 5.0,
                position: Vector3::zeros(),
                sigma: None,
                r_ap_n: Vector3::zeros(),
                n_hat_lp_n: Vector3::z(),
                target_position: None,
                pointing_error_az_deg: 0.0,
                pointing_error_el_deg: 0.0,
            }
        }
    }

    impl Spec {
        /// Upstream `create_antenna_msg_payload`.
        fn payload(&self) -> AntennaLogMsg {
            let params = compute_antenna_derived_params(
                self.directivity_db,
                self.k,
                self.p_tx,
                self.eta_r,
                self.t_e,
                self.t_ambient,
                self.t_sky,
                self.bandwidth,
            );
            let mut sigma = match (self.sigma, self.target_position) {
                (Some(sigma), _) => sigma,
                (None, Some(target)) => compute_pointing_sigma(self.position, target),
                (None, None) => Vector3::zeros(),
            };
            if self.pointing_error_az_deg != 0.0 || self.pointing_error_el_deg != 0.0 {
                sigma = apply_pointing_error(
                    sigma,
                    self.pointing_error_az_deg.to_radians(),
                    self.pointing_error_el_deg.to_radians(),
                );
            }
            AntennaLogMsg {
                antenna_name: self.name.to_string(),
                environment: self.environment,
                antenna_state: self.state,
                frequency_hz: self.frequency,
                bandwidth_hz: self.bandwidth,
                hpbw_az_rad: params.hpbw_az,
                hpbw_el_rad: params.hpbw_el,
                p_tx_w: self.p_tx,
                p_rx_w: 0.0,
                directivity_db: self.directivity_db,
                eta_r: self.eta_r,
                p_n_w: params.p_n,
                p_eirp_db: params.p_eirp_db,
                g_tn_db_per_k: self.directivity_db + 10.0 * self.eta_r.log10()
                    - 10.0 * params.t_s.log10(),
                t_ambient_k: self.t_ambient,
                r_an_n_m: self.position,
                sigma_an: sigma,
                v_an_n_mps: Vector3::zeros(),
                r_ap_n_m: self.r_ap_n,
                r_ap_p_m: self.r_ap_n,
                n_hat_lp_n: self.n_hat_lp_n,
            }
        }
    }

    /// Upstream `setup_link_budget_sim` and `run_simulation`: the module on
    /// two written antenna messages, run to `duration_nanos`; returns it with
    /// its recorded output.
    fn run_link(
        ant_1: AntennaLogMsg,
        ant_2: AntennaLogMsg,
        enable_atm: bool,
        duration_nanos: u64,
    ) -> (LinkBudget, Vec<LinkBudgetMsg>) {
        let (output_1, output_2) = (Output::new(ant_1), Output::new(ant_2));
        let mut module = LinkBudget::new("linkBudget");
        module.atmos_att = enable_atm;
        output_1.connect_to(&mut module.antenna_in_msg_1);
        output_2.connect_to(&mut module.antenna_in_msg_2);
        module.init();
        let mut log = Vec::new();
        for tick in TaskClock::new(STEP_NANOS).run_until(duration_nanos) {
            module.update(&context(tick));
            log.push(module.link_budget_out_msg.read());
        }
        (module, log)
    }

    /// [`run_link`] for upstream's default one second.
    fn run_link_1s(
        ant_1: &Spec,
        ant_2: &Spec,
        enable_atm: bool,
    ) -> (LinkBudget, Vec<LinkBudgetMsg>) {
        run_link(ant_1.payload(), ant_2.payload(), enable_atm, 1_000_000_000)
    }

    /// A transmitter at the origin and a receiver `distance_m` up +z, each
    /// pointing at the other, at `frequency_hz`.
    fn tx_rx_pair(distance_m: f64, frequency_hz: f64) -> (Spec, Spec) {
        let (pos_1, pos_2) = (Vector3::zeros(), Vector3::new(0.0, 0.0, distance_m));
        (
            Spec {
                name: "Ant1",
                state: Tx,
                frequency: frequency_hz,
                position: pos_1,
                target_position: Some(pos_2),
                ..Spec::default()
            },
            Spec {
                name: "Ant2",
                state: Rx,
                frequency: frequency_hz,
                position: pos_2,
                target_position: Some(pos_1),
                ..Spec::default()
            },
        )
    }

    /// A spacecraft transmitter at `pos_sc` and a ground receiver at
    /// `pos_ground` with a radial normal, each pointing at the other.
    fn space_ground_pair(
        pos_sc: Vector3<f64>,
        pos_ground: Vector3<f64>,
        frequency_hz: f64,
    ) -> (Spec, Spec) {
        (
            Spec {
                name: "SC_Ant",
                state: Tx,
                environment: Space,
                frequency: frequency_hz,
                position: pos_sc,
                target_position: Some(pos_ground),
                ..Spec::default()
            },
            Spec {
                name: "GND_Ant",
                state: Rx,
                environment: Earth,
                frequency: frequency_hz,
                position: pos_ground,
                target_position: Some(pos_sc),
                r_ap_n: pos_ground,
                n_hat_lp_n: Vector3::x(),
                ..Spec::default()
            },
        )
    }

    // ---------------------------------------------------------------------
    // test_linkBudget.py
    // ---------------------------------------------------------------------

    /// Upstream `test_linkBudget` (through `linkBudgetTestFunction`).
    #[test]
    fn link_budget() {
        let cases = [
            (Tx, Rx, false, 1000.0, 0.0, 0.0),
            (Rx, Tx, false, 1000.0, 0.0, 0.0),
            (RxTx, RxTx, false, 1000.0, 0.0, 0.0),
            (Tx, Rx, false, 100.0, 0.0, 0.0),
            (Tx, Rx, false, 10000.0, 0.0, 0.0),
            (Tx, Rx, false, 36000.0, 0.0, 0.0),
            (Tx, Rx, false, 1000.0, 0.0, 5.0),
            (Tx, Rx, false, 1000.0, 0.0, 10.0),
            (Tx, Rx, true, 400.0, 0.0, 0.0),
            (Off, Off, false, 1000.0, 0.0, 0.0),
            (Tx, Tx, false, 1000.0, 0.0, 0.0),
            (Rx, Rx, false, 1000.0, 0.0, 0.0),
        ];
        for (state_1, state_2, space_ground, distance_km, freq_offset_hz, pointing_error_deg) in
            cases
        {
            let frequency = 2.2e9;
            let params =
                compute_antenna_derived_params(20.0, 1.0, 100.0, 0.6, 50.0, 150.0, 5.0, 5e6);
            let distance_m = distance_km * 1000.0;
            let (pos_1, pos_2) = (Vector3::zeros(), Vector3::new(0.0, 0.0, distance_m));
            let ant_1 = Spec {
                name: "Antenna1",
                state: state_1,
                frequency,
                position: pos_1,
                target_position: Some(pos_2),
                pointing_error_az_deg: pointing_error_deg,
                ..Spec::default()
            };
            let (environment, r_ap_n, n_hat, t_ambient, t_sky) = if space_ground {
                (
                    Earth,
                    Vector3::new(REQ_EARTH_M, 0.0, 0.0),
                    Vector3::x(),
                    288.0,
                    200.0,
                )
            } else {
                (Space, Vector3::zeros(), Vector3::z(), 150.0, 5.0)
            };
            let ant_2 = Spec {
                name: "Antenna2",
                environment,
                state: state_2,
                frequency: frequency + freq_offset_hz,
                t_ambient,
                t_sky,
                position: pos_2,
                target_position: Some(pos_1),
                r_ap_n,
                n_hat_lp_n: n_hat,
                ..Spec::default()
            };
            let (module, log) = run_link_1s(&ant_1, &ant_2, false);
            let last = log.last().unwrap();

            assert!((last.distance_m - distance_m).abs() <= ACCURACY);
            assert!(
                (module.l_fspl_db() - compute_fspl(distance_m, frequency)).abs() <= ACCURACY_DB
            );
            if state_1.receives() && state_2.transmits() {
                assert!(last.cnr_1 > 0.0, "CNR1 should be positive");
            } else if !state_1.receives() {
                assert_eq!(last.cnr_1, CNR_INVALID);
            }
            if state_2.receives() && state_1.transmits() {
                assert!(last.cnr_2 > 0.0, "CNR2 should be positive");
            } else if !state_2.receives() {
                assert_eq!(last.cnr_2, CNR_INVALID);
            }
            if pointing_error_deg > 0.0 && !space_ground {
                let truth = compute_pointing_loss(
                    pointing_error_deg.to_radians(),
                    0.0,
                    params.hpbw_az,
                    params.hpbw_el,
                );
                assert!((module.l_point_db() - truth).abs() <= 2.0 * ACCURACY_DB);
            }
        }
    }

    /// Upstream `test_linkBudget_fspl_analytical`.
    #[test]
    fn fspl_analytical() {
        let cases = [
            (1000e3, 2.2e9),
            (400e3, 2.2e9),
            (36000e3, 2.2e9),
            (1000e3, 8.0e9),
            (1000e3, 400e6),
            (100e3, 2.2e9),
            (1000e3, 26e9),
        ];
        for (distance_m, frequency_hz) in cases {
            let (ant_1, ant_2) = tx_rx_pair(distance_m, frequency_hz);
            let (module, _) = run_link_1s(&ant_1, &ant_2, false);
            let truth = compute_fspl(distance_m, frequency_hz);
            assert!((module.l_fspl_db() - truth).abs() <= ACCURACY_DB);
        }
    }

    /// Upstream `test_linkBudget_cnr_calculation`.
    #[test]
    fn cnr_calculation() {
        let (distance_m, frequency_hz) = (1000e3, 2.2e9);
        let params = compute_antenna_derived_params(20.0, 1.0, 100.0, 0.6, 50.0, 150.0, 5.0, 5e6);
        let truth = compute_cnr(
            params.p_eirp_db,
            params.g_rx_db,
            compute_fspl(distance_m, frequency_hz),
            0.0,
            0.0,
            0.0,
            params.p_n,
        );
        let (ant_1, ant_2) = tx_rx_pair(distance_m, frequency_hz);
        let (_, log) = run_link_1s(&ant_1, &ant_2, false);
        let cnr = log.last().unwrap().cnr_2;
        assert!(
            (cnr - truth).abs() / truth <= ACCURACY_CNR_REL,
            "{cnr} vs {truth}"
        );
    }

    /// Upstream `test_linkBudget_cnr_with_pointing_loss`.
    #[test]
    fn cnr_with_pointing_loss() {
        let (distance_m, frequency_hz, bandwidth) = (1200e3, 8.4e9, 1e6);
        let (pos_tx, pos_rx) = (Vector3::zeros(), Vector3::new(0.0, 0.0, distance_m));
        let sigma_tx = apply_pointing_error(
            compute_pointing_sigma(pos_tx, pos_rx),
            5f64.to_radians(),
            0.0,
        );
        let ant_tx = Spec {
            name: "TX_Ant",
            state: Tx,
            frequency: frequency_hz,
            bandwidth,
            position: pos_tx,
            target_position: Some(pos_rx),
            sigma: Some(sigma_tx),
            ..Spec::default()
        };
        let ant_rx = Spec {
            name: "RX_Ant",
            state: Rx,
            frequency: frequency_hz,
            bandwidth,
            position: pos_rx,
            target_position: Some(pos_tx),
            ..Spec::default()
        };
        let (module, log) = run_link_1s(&ant_tx, &ant_rx, false);
        let derived =
            compute_antenna_derived_params(20.0, 1.0, 100.0, 0.6, 50.0, 150.0, 5.0, bandwidth);
        let truth = compute_cnr(
            derived.p_eirp_db,
            derived.g_rx_db,
            compute_fspl(distance_m, frequency_hz),
            module.l_atm_db(),
            module.l_point_db(),
            module.l_freq_db(),
            derived.p_n,
        );
        let cnr = log.last().unwrap().cnr_2;
        assert!(truth > 0.0 && cnr > 0.0);
        assert!((cnr - truth).abs() / truth <= 0.05);
    }

    /// Upstream `test_linkBudget_cnr_with_frequency_loss_partial_overlap`.
    #[test]
    fn cnr_with_frequency_loss_partial_overlap() {
        let distance_m = 900e3;
        let (pos_tx, pos_rx) = (Vector3::zeros(), Vector3::new(0.0, 0.0, distance_m));
        let (b1, b2) = (10e6, 10e6);
        let f1 = 10.000e9;
        let f2 = f1 + 3.0e6;
        let ant_tx = Spec {
            name: "TX_Ant",
            state: Tx,
            frequency: f1,
            bandwidth: b1,
            position: pos_tx,
            target_position: Some(pos_rx),
            ..Spec::default()
        };
        let ant_rx = Spec {
            name: "RX_Ant",
            state: Rx,
            frequency: f2,
            bandwidth: b2,
            position: pos_rx,
            target_position: Some(pos_tx),
            ..Spec::default()
        };
        let (module, log) = run_link_1s(&ant_tx, &ant_rx, false);
        let (b_ov_truth, f_center_truth) = compute_expected_overlap(f1, b1, f2, b2);
        let last = log.last().unwrap();
        assert!((last.bandwidth_hz - b_ov_truth).abs() <= 1e-6);
        let l_freq_truth = 10.0 * (b1.min(b2) / b_ov_truth).log10();
        assert!((module.l_freq_db() - l_freq_truth).abs() <= ACCURACY_DB);

        let derived = compute_antenna_derived_params(20.0, 1.0, 100.0, 0.6, 50.0, 150.0, 5.0, b1);
        // Upstream passes the losses in a different order; their sum is the
        // same.
        let truth = compute_cnr(
            derived.p_eirp_db,
            derived.g_rx_db,
            compute_fspl(distance_m, f_center_truth),
            module.l_point_db(),
            l_freq_truth,
            module.l_atm_db(),
            derived.p_n,
        );
        assert!(truth > 0.0 && last.cnr_2 > 0.0);
        assert!((last.cnr_2 - truth).abs() / truth <= 0.05);
    }

    /// Upstream `test_linkBudget_cnr_bookkeeping_with_atmos_pointing_and_freq_loss`.
    #[test]
    fn cnr_bookkeeping_with_atmos_pointing_and_freq_loss() {
        let pos_ground = Vector3::new(REQ_EARTH_M, 0.0, 0.0);
        let pos_sc =
            compute_spacecraft_position_from_central_angle(REQ_EARTH_M, 400e3, 5f64.to_radians());
        let (b1, b2) = (5e6, 5e6);
        let f1 = 10.000e9;
        let f2 = f1 + 1.0e6;
        let sigma_sc = apply_pointing_error(
            compute_pointing_sigma(pos_sc, pos_ground),
            3f64.to_radians(),
            2f64.to_radians(),
        );
        let (mut ant_sc, mut ant_gnd) = space_ground_pair(pos_sc, pos_ground, f1);
        ant_sc.bandwidth = b1;
        ant_sc.sigma = Some(sigma_sc);
        ant_gnd.frequency = f2;
        ant_gnd.bandwidth = b2;
        let (module, log) = run_link_1s(&ant_sc, &ant_gnd, true);

        let (b_ov_truth, f_center_truth) = compute_expected_overlap(f1, b1, f2, b2);
        assert!(b_ov_truth > 0.0);
        let distance_truth = (pos_sc - pos_ground).norm();
        let derived = compute_antenna_derived_params(20.0, 1.0, 100.0, 0.6, 50.0, 150.0, 5.0, b1);
        let truth = compute_cnr(
            derived.p_eirp_db,
            derived.g_rx_db,
            compute_fspl(distance_truth, f_center_truth),
            module.l_atm_db(),
            module.l_point_db(),
            module.l_freq_db(),
            derived.p_n,
        );
        let cnr = log.last().unwrap().cnr_2;
        assert!(truth > 0.0 && cnr > 0.0);
        assert!((cnr - truth).abs() / truth <= 0.05);
    }

    /// Upstream `test_linkBudget_pointing_loss`.
    #[test]
    fn pointing_loss() {
        let params = compute_antenna_derived_params(20.0, 1.0, 100.0, 0.6, 50.0, 150.0, 5.0, 5e6);
        for error_deg in [0.0, 2.0, 5.0, 8.0, 10.0] {
            let (mut ant_1, ant_2) = tx_rx_pair(1000e3, 2.2e9);
            ant_1.pointing_error_az_deg = error_deg;
            let (module, _) = run_link_1s(&ant_1, &ant_2, false);
            let truth = compute_pointing_loss(
                f64::to_radians(error_deg),
                0.0,
                params.hpbw_az,
                params.hpbw_el,
            );
            assert!(
                (module.l_point_db() - truth).abs() <= ACCURACY_DB,
                "{error_deg} deg"
            );
        }
    }

    /// Upstream `test_linkBudget_pointing_loss_nontrivial_attitude_zero_error`.
    #[test]
    fn pointing_loss_nontrivial_attitude_zero_error() {
        let (pos_1, pos_2) = (Vector3::zeros(), Vector3::new(1000e3, 400e3, 700e3));
        let ant_1 = Spec {
            name: "Ant1",
            state: Tx,
            position: pos_1,
            target_position: Some(pos_2),
            ..Spec::default()
        };
        let ant_2 = Spec {
            name: "Ant2",
            state: Rx,
            position: pos_2,
            target_position: Some(pos_1),
            ..Spec::default()
        };
        let (module, _) = run_link_1s(&ant_1, &ant_2, false);
        assert!(module.l_point_db().abs() < 1e-10, "{}", module.l_point_db());
    }

    /// Upstream `test_linkBudget_no_bandwidth_overlap`.
    #[test]
    fn no_bandwidth_overlap() {
        let (mut ant_1, mut ant_2) = tx_rx_pair(1000e3, 2.2e9);
        (ant_1.state, ant_2.state) = (RxTx, RxTx);
        ant_2.frequency = 2.3e9;
        let (_, log) = run_link_1s(&ant_1, &ant_2, false);
        let last = log.last().unwrap();
        assert!(last.bandwidth_hz <= 0.0);
        assert_eq!(last.cnr_1, CNR_INVALID);
        assert_eq!(last.cnr_2, CNR_INVALID);
    }

    /// Upstream `test_linkBudget_partial_bandwidth_overlap`.
    #[test]
    fn partial_bandwidth_overlap() {
        let (mut ant_1, mut ant_2) = tx_rx_pair(1000e3, 2.200e9);
        ant_1.bandwidth = 10e6;
        ant_2.bandwidth = 10e6;
        ant_2.frequency = 2.203e9;
        let (module, log) = run_link_1s(&ant_1, &ant_2, false);
        let expected_overlap = 7e6;
        assert!((log.last().unwrap().bandwidth_hz - expected_overlap).abs() <= 1e3);
        let expected_l_freq = 10.0 * (10e6 / expected_overlap).log10();
        assert!((module.l_freq_db() - expected_l_freq).abs() <= ACCURACY_DB);
    }

    /// Upstream `test_linkBudget_distance_calculation`.
    #[test]
    fn distance_calculation() {
        let cases = [
            (Vector3::zeros(), Vector3::new(1000e3, 0.0, 0.0), 1000e3),
            (Vector3::zeros(), Vector3::new(0.0, 1000e3, 0.0), 1000e3),
            (Vector3::zeros(), Vector3::new(0.0, 0.0, 1000e3), 1000e3),
            (
                Vector3::new(1e6, 2e6, 3e6),
                Vector3::new(2e6, 2e6, 3e6),
                1e6,
            ),
            (
                Vector3::zeros(),
                Vector3::new(1e6, 1e6, 1e6),
                3f64.sqrt() * 1e6,
            ),
            (
                Vector3::new(100e3, 200e3, 300e3),
                Vector3::new(400e3, 600e3, 800e3),
                (300e3f64.powi(2) + 400e3f64.powi(2) + 500e3f64.powi(2)).sqrt(),
            ),
        ];
        for (pos_1, pos_2, expected) in cases {
            let ant_1 = Spec {
                name: "Ant1",
                state: Tx,
                position: pos_1,
                target_position: Some(pos_2),
                ..Spec::default()
            };
            let ant_2 = Spec {
                name: "Ant2",
                state: Rx,
                position: pos_2,
                target_position: Some(pos_1),
                ..Spec::default()
            };
            let (_, log) = run_link_1s(&ant_1, &ant_2, false);
            assert!((log.last().unwrap().distance_m - expected).abs() <= 1.0);
        }
    }

    /// Upstream `test_linkBudget_symmetric_bidirectional`.
    #[test]
    fn symmetric_bidirectional() {
        let (mut ant_1, mut ant_2) = tx_rx_pair(1000e3, 2.2e9);
        (ant_1.state, ant_2.state) = (RxTx, RxTx);
        let (_, log) = run_link_1s(&ant_1, &ant_2, false);
        let last = log.last().unwrap();
        assert!(last.cnr_1 > 0.0 && last.cnr_2 > 0.0);
        assert!((last.cnr_1 - last.cnr_2).abs() / last.cnr_1.max(last.cnr_2) <= 0.01);
    }

    /// Upstream `test_linkBudget_fspl_vs_distance`: +6 dB per doubling.
    #[test]
    fn fspl_vs_distance() {
        let fspl = |distance_m| {
            let (ant_1, ant_2) = tx_rx_pair(distance_m, 2.2e9);
            run_link_1s(&ant_1, &ant_2, false).0.l_fspl_db()
        };
        let increase = fspl(2000e3) - fspl(1000e3);
        assert!((increase - 20.0 * 2f64.log10()).abs() <= 0.05);
    }

    /// Upstream `test_linkBudget_fspl_vs_frequency`: +6 dB per doubling.
    #[test]
    fn fspl_vs_frequency() {
        let fspl = |frequency_hz| {
            let (ant_1, ant_2) = tx_rx_pair(1000e3, frequency_hz);
            run_link_1s(&ant_1, &ant_2, false).0.l_fspl_db()
        };
        let increase = fspl(4.4e9) - fspl(2.2e9);
        assert!((increase - 20.0 * 2f64.log10()).abs() <= 0.05);
    }

    /// Upstream `test_linkBudget_antenna_state_transitions`.
    #[test]
    fn antenna_state_transitions() {
        let matrix = [
            (Off, Off, false, false),
            (Off, Rx, false, false),
            (Off, Tx, false, false),
            (Off, RxTx, false, false),
            (Rx, Off, false, false),
            (Rx, Rx, false, false),
            (Rx, Tx, true, false),
            (Rx, RxTx, true, false),
            (Tx, Off, false, false),
            (Tx, Rx, false, true),
            (Tx, Tx, false, false),
            (Tx, RxTx, false, true),
            (RxTx, Off, false, false),
            (RxTx, Rx, false, true),
            (RxTx, Tx, true, false),
            (RxTx, RxTx, true, true),
        ];
        for (state_1, state_2, expect_cnr_1, expect_cnr_2) in matrix {
            let (mut ant_1, mut ant_2) = tx_rx_pair(1000e3, 2.2e9);
            (ant_1.state, ant_2.state) = (state_1, state_2);
            let (_, log) = run_link_1s(&ant_1, &ant_2, false);
            let last = log.last().unwrap();
            for (cnr, expect) in [(last.cnr_1, expect_cnr_1), (last.cnr_2, expect_cnr_2)] {
                if expect {
                    assert!(cnr > 0.0, "{state_1:?} / {state_2:?}");
                } else {
                    assert_eq!(cnr, CNR_INVALID, "{state_1:?} / {state_2:?}");
                }
            }
        }
    }

    /// Upstream `test_linkBudget_atmospheric_attenuation_space_ground`.
    #[test]
    fn atmospheric_attenuation_space_ground() {
        let pos_sc = Vector3::new(REQ_EARTH_M + 400e3, 0.0, 0.0);
        let pos_ground = Vector3::new(REQ_EARTH_M, 0.0, 0.0);
        let (ant_sc, ant_gnd) = space_ground_pair(pos_sc, pos_ground, 10e9);
        let (with_atm, log_with) = run_link_1s(&ant_sc, &ant_gnd, true);
        let (without_atm, log_without) = run_link_1s(&ant_sc, &ant_gnd, false);
        assert!(with_atm.l_atm_db() > 0.0);
        assert_eq!(without_atm.l_atm_db(), 0.0);
        assert!(log_with.last().unwrap().cnr_2 < log_without.last().unwrap().cnr_2);
    }

    /// Upstream `test_linkBudget_atmospheric_attenuation_elevation_dependence`.
    #[test]
    fn atmospheric_attenuation_elevation_dependence() {
        let pos_ground = Vector3::new(REQ_EARTH_M, 0.0, 0.0);
        let mut losses = Vec::new();
        let mut cnrs = Vec::new();
        for angle_deg in [0.0, 5.0, 17.0] {
            let pos_sc = compute_spacecraft_position_from_central_angle(
                REQ_EARTH_M,
                400e3,
                f64::to_radians(angle_deg),
            );
            let (ant_sc, ant_gnd) = space_ground_pair(pos_sc, pos_ground, 10e9);
            let (module, log) = run_link_1s(&ant_sc, &ant_gnd, true);
            let (loss, cnr) = (module.l_atm_db(), log.last().unwrap().cnr_2);
            assert!(loss.is_finite() && loss >= 0.0 && cnr >= 0.0);
            losses.push(loss);
            cnrs.push(cnr);
        }
        assert!(
            losses[0] <= losses[1] && losses[1] <= losses[2],
            "{losses:?}"
        );
        assert!(cnrs[0] >= cnrs[1] && cnrs[1] >= cnrs[2], "{cnrs:?}");
    }

    /// Upstream `ATMOSPHERE_ABSOLUTE_CASES`: ground altitude \[m\],
    /// frequency \[Hz\], elevation \[deg\], loss \[dB\], absolute tolerance
    /// \[dB\]. Upstream's references integrate the same model with 0.5 m
    /// cells and the real gas tables.
    const ATMOSPHERE_ABSOLUTE_CASES: [(f64, f64, f64, f64, f64); 48] = [
        (0.0, 10.0e9, 2.0, 0.73913091955873, 0.0001),
        (0.0, 10.0e9, 4.9, 0.695863611338296, 0.0001),
        (0.0, 10.0e9, 5.0, 0.686015971785629, 0.0001),
        (0.0, 10.0e9, 5.1, 0.674405627870, 1e-4),
        (0.0, 10.0e9, 10.0, 0.365788633708, 1e-4),
        (0.0, 10.0e9, 90.0, 0.0651973107843, 1e-4),
        (1500.0, 10.0e9, 2.0, 0.530626295795731, 0.0001),
        (1500.0, 10.0e9, 4.9, 0.494780705281705, 0.0001),
        (1500.0, 10.0e9, 5.0, 0.487639189337712, 0.0001),
        (1500.0, 10.0e9, 5.1, 0.479550654765, 1e-4),
        (1500.0, 10.0e9, 10.0, 0.262189337581, 1e-4),
        (1500.0, 10.0e9, 90.0, 0.0469195476589, 1e-4),
        (0.0, 60.0e9, 2.0, 2417.86624090375, 0.05),
        (0.0, 60.0e9, 4.9, 2148.54549142479, 0.05),
        (0.0, 60.0e9, 5.0, 2118.41165918122, 0.05),
        (0.0, 60.0e9, 5.1, 2086.84987158436, 0.05),
        (0.0, 60.0e9, 10.0, 1187.36152377926, 0.05),
        (0.0, 60.0e9, 90.0, 216.734190702406, 0.05),
        (1500.0, 60.0e9, 2.0, 2181.73496740041, 0.05),
        (1500.0, 60.0e9, 4.9, 1935.05649363071, 0.05),
        (1500.0, 60.0e9, 5.0, 1907.8226152706, 0.05),
        (1500.0, 60.0e9, 5.1, 1879.43704686702, 0.05),
        (1500.0, 60.0e9, 10.0, 1069.87294242870, 0.05),
        (1500.0, 60.0e9, 90.0, 195.325379591458, 0.05),
        (0.0, 118.750334e9, 2.0, 1170.13249602723, 0.02),
        (0.0, 118.750334e9, 4.9, 975.850387897437, 0.02),
        (0.0, 118.750334e9, 5.0, 965.099284676145, 0.02),
        (0.0, 118.750334e9, 5.1, 954.234001602237, 0.02),
        (0.0, 118.750334e9, 10.0, 597.776772083896, 0.02),
        (0.0, 118.750334e9, 90.0, 115.789794121404, 0.02),
        (1500.0, 118.750334e9, 2.0, 1146.79554135026, 0.02),
        (1500.0, 118.750334e9, 4.9, 955.916545174959, 0.02),
        (1500.0, 118.750334e9, 5.0, 945.269417354956, 0.02),
        (1500.0, 118.750334e9, 5.1, 934.570016278601, 0.02),
        (1500.0, 118.750334e9, 10.0, 584.233509932637, 0.02),
        (1500.0, 118.750334e9, 90.0, 112.967089115958, 0.02),
        (0.0, 183.310091e9, 2.0, 1772.51330373514, 0.02),
        (0.0, 183.310091e9, 4.9, 1723.90198331452, 0.02),
        (0.0, 183.310091e9, 5.0, 1699.66523659699, 0.02),
        (0.0, 183.310091e9, 5.1, 1668.96647637460, 0.02),
        (0.0, 183.310091e9, 10.0, 880.711914717179, 0.02),
        (0.0, 183.310091e9, 90.0, 154.784577120573, 0.02),
        (1500.0, 183.310091e9, 2.0, 1037.36082890744, 0.02),
        (1500.0, 183.310091e9, 4.9, 1006.12352730235, 0.02),
        (1500.0, 183.310091e9, 5.0, 991.737401072793, 0.02),
        (1500.0, 183.310091e9, 5.1, 973.934564759451, 0.02),
        (1500.0, 183.310091e9, 10.0, 515.251071114997, 0.02),
        (1500.0, 183.310091e9, 90.0, 90.6712643381242, 0.02),
    ];

    /// Ground and spacecraft payloads of upstream's slant cases: the ground
    /// antenna at `ground_altitude` on the +x axis, the spacecraft 2000 km
    /// away at `elevation_deg`.
    fn slant_pair(
        ground_altitude: f64,
        frequency: f64,
        elevation_deg: f64,
    ) -> (AntennaLogMsg, AntennaLogMsg) {
        let elevation = elevation_deg.to_radians();
        let ground_position = Vector3::new(REQ_EARTH_M + ground_altitude, 0.0, 0.0);
        let spacecraft_position =
            ground_position + 2000.0e3 * Vector3::new(elevation.sin(), elevation.cos(), 0.0);
        let ground = Spec {
            name: "Ground",
            environment: Earth,
            state: Rx,
            frequency,
            position: ground_position,
            r_ap_n: ground_position,
            n_hat_lp_n: Vector3::x(),
            target_position: Some(spacecraft_position),
            ..Spec::default()
        };
        let spacecraft = Spec {
            name: "Spacecraft",
            environment: Space,
            state: Tx,
            frequency,
            position: spacecraft_position,
            target_position: Some(ground_position),
            ..Spec::default()
        };
        (ground.payload(), spacecraft.payload())
    }

    /// Upstream `test_atmospheric_attenuation_absolute`, both antenna
    /// orders, with the built-in gas tables (upstream's JSON files).
    #[test]
    fn atmospheric_attenuation_absolute() {
        for (ground_altitude, frequency, elevation_deg, expected, tolerance) in
            ATMOSPHERE_ABSOLUTE_CASES
        {
            for ground_first in [false, true] {
                let (ground, spacecraft) = slant_pair(ground_altitude, frequency, elevation_deg);
                let (ant_1, ant_2) = if ground_first {
                    (ground, spacecraft)
                } else {
                    (spacecraft, ground)
                };
                let (module, _) = run_link(ant_1, ant_2, true, 0);
                let loss = module.l_atm_db();
                assert!(
                    (loss - expected).abs() <= tolerance,
                    "{ground_altitude} m, {frequency} Hz, {elevation_deg} deg: {loss} vs \
                     {expected}"
                );
            }
        }
    }

    /// Upstream `test_atmospheric_attenuation_slant_ratio`: the slant loss
    /// scales as the cosecant of the elevation within one percent, and a
    /// reset rebuilds the same profile.
    #[test]
    fn atmospheric_attenuation_slant_ratio() {
        for ground_altitude in [0.0, 1500.0] {
            for elevation_deg in [30.0, 60.0] {
                for ground_first in [false, true] {
                    let mut losses = Vec::new();
                    for angle_deg in [90.0, elevation_deg] {
                        let (ground, spacecraft) = slant_pair(ground_altitude, 10.0e9, angle_deg);
                        let (ant_1, ant_2) = if ground_first {
                            (ground, spacecraft)
                        } else {
                            (spacecraft, ground)
                        };
                        let (output_1, output_2) = (Output::new(ant_1), Output::new(ant_2));
                        let mut module = LinkBudget::new("linkBudget");
                        module.atmos_att = true;
                        output_1.connect_to(&mut module.antenna_in_msg_1);
                        output_2.connect_to(&mut module.antenna_in_msg_2);
                        module.init();
                        module.update(&context(0));
                        let loss = module.l_atm_db();
                        assert!(loss.is_finite() && loss > 0.0);
                        // Reset must rebuild the same profile without
                        // duplicating cached layers.
                        module.reset(&context(0));
                        module.update(&context(0));
                        assert!((module.l_atm_db() - loss).abs() <= 1.0e-12 * loss);
                        losses.push(loss);
                    }
                    let expected_ratio = 1.0 / f64::to_radians(elevation_deg).sin();
                    let ratio = losses[1] / losses[0];
                    assert!((ratio - expected_ratio).abs() <= 0.01 * expected_ratio);
                }
            }
        }
    }

    /// Upstream `test_linkBudget_atmospheric_attenuation_low_elevation_stability`.
    #[test]
    fn atmospheric_attenuation_low_elevation_stability() {
        let pos_ground = Vector3::new(REQ_EARTH_M, 0.0, 0.0);
        let pos_sc =
            compute_spacecraft_position_from_central_angle(REQ_EARTH_M, 400e3, 18f64.to_radians());
        let (ant_sc, ant_gnd) = space_ground_pair(pos_sc, pos_ground, 10e9);
        let (module, log) = run_link_1s(&ant_sc, &ant_gnd, true);
        let loss = module.l_atm_db();
        assert!(loss.is_finite() && (0.0..=200.0).contains(&loss));
        assert!(log.last().unwrap().cnr_2 >= 0.0);
    }

    /// Upstream `test_linkBudget_atmospheric_attenuation_space_space`.
    #[test]
    fn atmospheric_attenuation_space_space() {
        let (ant_1, ant_2) = tx_rx_pair(1000e3, 2.2e9);
        let (module, _) = run_link_1s(&ant_1, &ant_2, true);
        assert_eq!(module.l_atm_db(), 0.0);
        assert_eq!(module.antenna_placement(), AntennaPlacement::SpaceSpace);
        assert!(!module.atmos_att);
    }

    /// Upstream `test_linkBudget_atmospheric_attenuation_frequency_dependence`.
    #[test]
    fn atmospheric_attenuation_frequency_dependence() {
        let pos_sc = Vector3::new(REQ_EARTH_M + 400e3, 0.0, 0.0);
        let pos_ground = Vector3::new(REQ_EARTH_M, 0.0, 0.0);
        let losses: Vec<f64> = [2e9, 10e9, 20e9]
            .into_iter()
            .map(|frequency| {
                let (ant_sc, ant_gnd) = space_ground_pair(pos_sc, pos_ground, frequency);
                run_link_1s(&ant_sc, &ant_gnd, true).0.l_atm_db()
            })
            .collect();
        assert!(losses[0] < losses[2], "{losses:?}");
    }

    /// Upstream `test_linkBudget_very_short_distance`.
    #[test]
    fn very_short_distance() {
        let (ant_1, ant_2) = tx_rx_pair(100.0, 2.2e9);
        let (module, log) = run_link_1s(&ant_1, &ant_2, false);
        assert!((module.l_fspl_db() - compute_fspl(100.0, 2.2e9)).abs() <= ACCURACY_DB);
        assert!(log.last().unwrap().cnr_2 > 0.0);
    }

    /// Upstream `test_linkBudget_very_long_distance`.
    #[test]
    fn very_long_distance() {
        let distance_m = 384400e3;
        let (mut ant_1, mut ant_2) = tx_rx_pair(distance_m, 8.4e9);
        (ant_1.directivity_db, ant_1.p_tx) = (45.0, 400.0);
        ant_2.directivity_db = 45.0;
        let (module, log) = run_link_1s(&ant_1, &ant_2, false);
        let distance = log.last().unwrap().distance_m;
        assert!((distance - distance_m).abs() / distance_m <= 1e-6);
        assert!((module.l_fspl_db() - compute_fspl(distance_m, 8.4e9)).abs() <= ACCURACY_DB);
    }

    /// Upstream `test_linkBudget_high_frequency`.
    #[test]
    fn high_frequency() {
        let (ant_1, ant_2) = tx_rx_pair(1000e3, 100e9);
        let (module, log) = run_link_1s(&ant_1, &ant_2, false);
        assert!((module.l_fspl_db() - compute_fspl(1000e3, 100e9)).abs() <= ACCURACY_DB);
        assert!(log.last().unwrap().cnr_2 > 0.0);
    }

    /// Upstream `test_linkBudget_low_frequency`.
    #[test]
    fn low_frequency() {
        let (ant_1, ant_2) = tx_rx_pair(1000e3, 1e9);
        let (module, _) = run_link_1s(&ant_1, &ant_2, false);
        assert!((module.l_fspl_db() - compute_fspl(1000e3, 1e9)).abs() <= ACCURACY_DB);
    }

    /// Upstream `test_linkBudget_different_bandwidths`.
    #[test]
    fn different_bandwidths() {
        let (mut ant_1, mut ant_2) = tx_rx_pair(1000e3, 2.2e9);
        (ant_1.bandwidth, ant_2.bandwidth) = (20e6, 5e6);
        let (module, log) = run_link_1s(&ant_1, &ant_2, false);
        assert!((log.last().unwrap().bandwidth_hz - 5e6).abs() <= 1e3);
        assert!(module.l_freq_db().abs() <= ACCURACY_DB);
    }

    /// Upstream `test_linkBudget_combined_pointing_errors`.
    #[test]
    fn combined_pointing_errors() {
        let params = compute_antenna_derived_params(20.0, 1.0, 100.0, 0.6, 50.0, 150.0, 5.0, 5e6);
        let (mut ant_1, ant_2) = tx_rx_pair(1000e3, 2.2e9);
        (ant_1.pointing_error_az_deg, ant_1.pointing_error_el_deg) = (3.0, 4.0);
        let (module, _) = run_link_1s(&ant_1, &ant_2, false);
        let truth = compute_pointing_loss(
            3f64.to_radians(),
            4f64.to_radians(),
            params.hpbw_az,
            params.hpbw_el,
        );
        assert!((module.l_point_db() - truth).abs() <= ACCURACY_DB);
    }

    /// Upstream `test_linkBudget_output_message_structure`.
    #[test]
    fn output_message_structure() {
        let (mut ant_1, mut ant_2) = tx_rx_pair(1000e3, 2.2e9);
        (ant_1.state, ant_2.state) = (RxTx, RxTx);
        let (_, log) = run_link_1s(&ant_1, &ant_2, false);
        let last = log.last().unwrap();
        for (actual, expected) in [
            (last.distance_m, 1000e3),
            (last.bandwidth_hz, 5e6),
            (last.frequency_hz, 2.2e9),
        ] {
            assert!((actual - expected).abs() / expected <= 1e-6);
        }
        assert!(last.cnr_1.is_finite() && last.cnr_2.is_finite());
        assert!(last.cnr_1 > 0.0 && last.cnr_2 > 0.0);
    }

    /// Upstream `test_linkBudget_output_message_consistency`.
    #[test]
    fn output_message_consistency() {
        let distance_m = 700e3;
        let b = 10e6;
        let f1 = 10.000e9;
        let f2 = f1 + 3.0e6;
        let (mut ant_tx, mut ant_rx) = tx_rx_pair(distance_m, f1);
        (ant_tx.name, ant_rx.name) = ("TX_Ant", "RX_Ant");
        (ant_tx.bandwidth, ant_rx.bandwidth, ant_rx.frequency) = (b, b, f2);
        let (module, log) = run_link_1s(&ant_tx, &ant_rx, false);
        let logged = log.last().unwrap();
        let out = module.link_budget_out_msg.read();
        assert!((logged.distance_m - out.distance_m).abs() <= 1e-6);
        assert!((logged.bandwidth_hz - out.bandwidth_hz).abs() <= 1e-6);
        assert!((logged.frequency_hz - out.frequency_hz).abs() <= 1e-3);
        assert!((logged.cnr_1 - out.cnr_1).abs() <= 1e-12);
        assert!((logged.cnr_2 - out.cnr_2).abs() <= 1e-12);
        assert!((logged.distance_m - distance_m).abs() / distance_m <= 1e-9);
        let (b_ov_truth, f_center_truth) = compute_expected_overlap(f1, b, f2, b);
        assert!((logged.bandwidth_hz - b_ov_truth).abs() <= 1e-6);
        assert!((logged.frequency_hz - f_center_truth).abs() <= 1e-3);
        assert!(
            [&out.antenna_name_1, &out.antenna_name_2]
                .iter()
                .any(|name| name.contains("TX_Ant"))
        );
        assert!(
            [&out.antenna_name_1, &out.antenna_name_2]
                .iter()
                .any(|name| name.contains("RX_Ant"))
        );
    }

    // ---------------------------------------------------------------------
    // test_linkBudgetAtmosphere.cpp
    // ---------------------------------------------------------------------

    const TOP_ALTITUDE_M: f64 = 100.0e3;
    const ALTITUDE_TOLERANCE_M: f64 = 1.0e-8;

    /// Upstream `LinkBudgetTestAccess::buildLayers`: the layer grid at
    /// `altitude` for the dry-air continuum (no gas tables loaded).
    fn build_layers(model: &mut LinkBudget, altitude: f64) -> AttenuationLookupTable {
        model.h_0_m = altitude;
        model.precompute_atmospheric_attenuation_at_layers(10.0e9);
        model.attenuation_lookup.clone()
    }

    /// Upstream `LinkBudgetTestAccess::syntheticLoss`: the loss of a uniform
    /// 0.01 dB/km atmosphere of 4000 layers, refractive index constant or
    /// proportional to `1/r` (`constant_elevation`).
    fn synthetic_loss(
        model: &mut LinkBudget,
        altitude: f64,
        elevation: f64,
        constant_elevation: bool,
    ) -> f64 {
        model.h_0_m = altitude;
        model.n_0 = 1.02;
        model.antenna_in_1 = AntennaLogMsg::default();
        model.antenna_in_2 = AntennaLogMsg::default();
        model.ground_slot = Some(Slot::First);
        let radius = REQ_EARTH * 1e3 + altitude;
        let distance = 2000.0e3;
        model.antenna_in_1.r_an_n_m.x = radius;
        model.antenna_in_1.n_hat_lp_n.x = 1.0;
        model.antenna_in_2.r_an_n_m.x = radius + distance * elevation.sin();
        model.antenna_in_2.r_an_n_m.y = distance * elevation.cos();

        let layer_count = 4000;
        let thickness = (TOP_OF_ATMOSPHERE_M - altitude) / layer_count as f64;
        model.attenuation_lookup = AttenuationLookupTable::default();
        for i in 0..layer_count {
            let height = altitude + (i as f64 + 0.5) * thickness;
            let table = &mut model.attenuation_lookup;
            table.layer_i.push(height);
            table.delta_h_layer_i.push(thickness);
            table.gamma_i.push(0.01);
            // With n proportional to 1/r, Snell's invariant keeps the
            // elevation constant.
            table.n_i.push(if constant_elevation {
                model.n_0 * radius / (REQ_EARTH * 1e3 + height)
            } else {
                model.n_0
            });
        }
        model.calculate_atmospheric_loss();
        model.l_atm_db()
    }

    /// `ground_altitudes()`: below, at and above every boundary.
    fn ground_altitudes() -> Vec<f64> {
        let mut altitudes = vec![0.0, 1500.0, 25000.0, 80000.0, 99999.0];
        for boundary in layer_boundaries_m() {
            altitudes.extend([boundary - 1.0, boundary, boundary + 1.0]);
        }
        altitudes
    }

    /// `LinkBudgetLayerGrid.CoversEntireAtmosphere`: finite midpoint cells
    /// covering the column, split at every atmospheric boundary.
    #[test]
    fn layer_grid_covers_entire_atmosphere() {
        let boundaries = layer_boundaries_m();
        for ground_altitude in ground_altitudes() {
            let layers = build_layers(&mut LinkBudget::new("model"), ground_altitude);
            assert!(!layers.layer_i.is_empty());
            assert!(layers.layer_i.len() <= 931);
            assert_eq!(layers.layer_i.len(), layers.delta_h_layer_i.len());
            assert_eq!(layers.layer_i.len(), layers.gamma_i.len());
            assert_eq!(layers.layer_i.len(), layers.n_i.len());
            if ground_altitude == 0.0 {
                assert_eq!(layers.layer_i.len(), 931);
            }
            let mut previous_top = ground_altitude;
            let mut total_thickness = 0.0;
            for i in 0..layers.layer_i.len() {
                let (height, thickness) = (layers.layer_i[i], layers.delta_h_layer_i[i]);
                assert!(height.is_finite() && thickness.is_finite());
                assert!(layers.gamma_i[i].is_finite() && layers.n_i[i].is_finite());
                assert!(thickness > 0.0);
                assert!(layers.gamma_i[i] >= 0.0);
                assert!(layers.n_i[i] >= 1.0);
                assert!((height - 0.5 * thickness - previous_top).abs() <= ALTITUDE_TOLERANCE_M);
                for &boundary in &boundaries {
                    assert!(
                        !(height - 0.5 * thickness < boundary - ALTITUDE_TOLERANCE_M
                            && height + 0.5 * thickness > boundary + ALTITUDE_TOLERANCE_M),
                        "cell {i} at {ground_altitude} m straddles {boundary} m"
                    );
                }
                previous_top = height + 0.5 * thickness;
                total_thickness += thickness;
                if i > 0 {
                    assert!(height > layers.layer_i[i - 1]);
                }
            }
            assert!((previous_top - TOP_ALTITUDE_M).abs() <= ALTITUDE_TOLERANCE_M);
            assert!(
                (total_thickness - (TOP_ALTITUDE_M - ground_altitude)).abs()
                    <= ALTITUDE_TOLERANCE_M
            );
        }
    }

    /// `LinkBudgetAtmosphere.RebuildingReplacesLayers`.
    #[test]
    fn rebuilding_replaces_layers() {
        let mut model = LinkBudget::new("model");
        let original = build_layers(&mut model, 0.0);
        let repeated = build_layers(&mut model, 0.0);
        assert_eq!(original, repeated);
        for altitude in [100000.0, 110000.0] {
            build_layers(&mut model, 0.0);
            let empty = build_layers(&mut model, altitude);
            assert_eq!(empty, AttenuationLookupTable::default());
        }
    }

    fn ray_path_cases() -> Vec<(f64, f64)> {
        let mut cases = Vec::new();
        for altitude in [0.0, 1500.0] {
            for elevation_deg in [10.0, 30.0, 60.0, 90.0] {
                cases.push((altitude, elevation_deg));
            }
        }
        cases
    }

    /// `LinkBudgetRayPath.MatchesSphericalShellChord`: a uniform refractive
    /// index gives the straight chord through the spherical shell.
    #[test]
    fn ray_path_matches_spherical_shell_chord() {
        for (altitude, elevation_deg) in ray_path_cases() {
            let elevation = elevation_deg * D2R;
            let actual = synthetic_loss(&mut LinkBudget::new("model"), altitude, elevation, false);
            let radius = REQ_EARTH * 1e3 + altitude;
            let height = TOP_ALTITUDE_M - altitude;
            let vertical_projection = radius * elevation.sin();
            let radius_difference_squared = height * (2.0 * radius + height);
            let path_length = radius_difference_squared
                / ((vertical_projection * vertical_projection + radius_difference_squared).sqrt()
                    + vertical_projection);
            let expected = 0.01 * path_length * 1e-3;
            assert!(
                (actual - expected).abs() <= expected * 1.0e-7,
                "{actual} vs {expected}"
            );
        }
    }

    /// `LinkBudgetRayPath.AccountsForRefractiveIndex`: an inverse-radius
    /// refractive index keeps the elevation, so the path is the exact
    /// cosecant one.
    #[test]
    fn ray_path_accounts_for_refractive_index() {
        for (altitude, elevation_deg) in ray_path_cases() {
            let elevation = elevation_deg * D2R;
            let actual = synthetic_loss(&mut LinkBudget::new("model"), altitude, elevation, true);
            let expected = 0.01 * (TOP_ALTITUDE_M - altitude) * 1e-3 / elevation.sin();
            assert!(
                (actual - expected).abs() <= expected * 1.0e-11,
                "{actual} vs {expected}"
            );
        }
    }

    /// `LinkBudgetElevationClamp.PreservesRoundedSineThreshold`: the clamp at
    /// a sine of 0.0872 and the unclamped cosecant path either side of it.
    #[test]
    fn elevation_clamp_preserves_rounded_sine_threshold() {
        let clamp_elevation = 0.0872f64.asin();
        let offset = 1.0e-4 * D2R;
        let elevations = [
            0.0,
            2.0 * D2R,
            4.9 * D2R,
            5.0 * D2R,
            clamp_elevation - offset,
            clamp_elevation,
            clamp_elevation + offset,
            5.1 * D2R,
        ];
        for altitude in [0.0, 1500.0] {
            for elevation in elevations {
                let actual =
                    synthetic_loss(&mut LinkBudget::new("model"), altitude, elevation, true);
                let path_length_km =
                    (TOP_ALTITUDE_M - altitude) * 1e-3 / elevation.sin().max(0.0872);
                let expected = 0.01 * path_length_km;
                assert!(
                    (actual - expected).abs() <= 1.0e-9,
                    "{actual} vs {expected}"
                );
            }
        }
    }

    // ---------------------------------------------------------------------
    // test_antennaProducerChain.cpp
    // ---------------------------------------------------------------------

    const CHAIN_FREQUENCY_HZ: f64 = 20.0e9;
    const CHAIN_BANDWIDTH_HZ: f64 = 8.0e6;
    const CHAIN_DIRECTIVITY_DB: f64 = 30.0;
    const CHAIN_EFFICIENCY: f64 = 0.65;
    const CHAIN_TRANSMIT_POWER_W: f64 = 5.0;
    const CHAIN_RECEIVER_TEMPERATURE_K: f64 = 120.0;
    const CHAIN_AMBIENT_TEMPERATURE_K: f64 = 150.0;
    const CHAIN_DISTANCE_M: f64 = 2000.0e3;

    /// Upstream's `configureAntenna`, without sky-map or ephemeris inputs.
    fn chain_antenna(name: &str) -> SimpleAntenna {
        SimpleAntenna::new(SimpleAntennaConfig {
            name: name.to_string(),
            frequency_hz: CHAIN_FREQUENCY_HZ,
            bandwidth_hz: CHAIN_BANDWIDTH_HZ,
            directivity_db: CHAIN_DIRECTIVITY_DB,
            hpbw_ratio: 1.0,
            p_tx_w: CHAIN_TRANSMIT_POWER_W,
            p_rx_w: 5.0,
            equivalent_noise_temperature_k: CHAIN_RECEIVER_TEMPERATURE_K,
            radiation_efficiency: CHAIN_EFFICIENCY,
            ambient_temperature_k: Some(CHAIN_AMBIENT_TEMPERATURE_K),
            r_ab_b_m: Vector3::zeros(),
            sigma_ab: Vector3::zeros(),
        })
    }

    /// Upstream `AntennaProducerChain`: ground location on a tilted planet,
    /// a space and a ground antenna, and the link budget.
    struct ProducerChain {
        ground_first: bool,
        pointing_offset: f64,
        planet: Output<PlanetStateMsg>,
        spacecraft: Output<SpacecraftStateMsg>,
        ground: GroundLocation,
        transmitter: SimpleAntenna,
        receiver: SimpleAntenna,
        budget: LinkBudget,
        expected_normal_n: Vector3<f64>,
        expected_ground_position_n: Vector3<f64>,
    }

    impl ProducerChain {
        /// Upstream `SetUp`.
        fn new(altitude: f64, ground_first: bool, pointing_offset: f64) -> Self {
            let mut chain = Self {
                ground_first,
                pointing_offset,
                planet: Output::default(),
                spacecraft: Output::default(),
                ground: GroundLocation::new(GroundLocationConfig {
                    planet_radius_m: REQ_EARTH * 1.0e3,
                    ..GroundLocationConfig::default()
                }),
                transmitter: chain_antenna("space"),
                receiver: chain_antenna("ground"),
                budget: LinkBudget::new("linkBudget"),
                expected_normal_n: Vector3::zeros(),
                expected_ground_position_n: Vector3::zeros(),
            };
            let index = chain.ground.add_spacecraft_to_model();
            chain
                .spacecraft
                .connect_to(&mut chain.ground.sc_state_in_msgs[index]);
            chain.planet.connect_to(&mut chain.ground.planet_in_msg);
            chain
                .spacecraft
                .connect_to(&mut chain.transmitter.sc_state_in_msg);
            chain
                .ground
                .current_ground_state_out_msg
                .connect_to(&mut chain.receiver.ground_state_in_msg);
            chain.transmitter.set_antenna_state(Tx);
            chain.receiver.set_antenna_state(Rx);
            let (first, second) = if ground_first {
                (&chain.receiver, &chain.transmitter)
            } else {
                (&chain.transmitter, &chain.receiver)
            };
            first
                .antenna_out_msg
                .connect_to(&mut chain.budget.antenna_in_msg_1);
            second
                .antenna_out_msg
                .connect_to(&mut chain.budget.antenna_in_msg_2);
            chain.budget.atmos_att = true;
            chain.set_geometry(altitude);
            chain
        }

        /// Upstream `setGeometry`: the station altitude, and the external
        /// planet and spacecraft states.
        fn set_geometry(&mut self, altitude: f64) {
            let (latitude, longitude, tilt, elevation) = (0.35f64, -0.6f64, 0.4, 0.7f64);
            self.ground.config.location =
                SiteLocation::Geodetic(crate::utilities::geodetic_conversion::Lla {
                    latitude_rad: latitude,
                    longitude_rad: longitude,
                    altitude_m: altitude,
                });
            let dcm_pn: Matrix3<f64> =
                Rotation3::from_axis_angle(&Vector3::y_axis(), tilt).into_inner();
            self.planet.write(PlanetStateMsg {
                orientation: Some(PlanetOrientation {
                    inertial_to_fixed: dcm_pn,
                    inertial_to_fixed_dot: Matrix3::zeros(),
                }),
                ..PlanetStateMsg::default()
            });
            let normal_p = Vector3::new(
                latitude.cos() * longitude.cos(),
                latitude.cos() * longitude.sin(),
                latitude.sin(),
            );
            self.expected_normal_n = dcm_pn.transpose() * normal_p;
            self.expected_ground_position_n =
                (REQ_EARTH * 1.0e3 + altitude) * self.expected_normal_n;
            let east_p = Vector3::new(-longitude.sin(), longitude.cos(), 0.0);
            let ray_n = elevation.sin() * self.expected_normal_n
                + elevation.cos() * dcm_pn.transpose() * east_p;
            let aligned_nb = Rotation3::rotation_between(&Vector3::z(), &-ray_n)
                .expect("not antiparallel")
                .into_inner();
            let dcm_nb = aligned_nb
                * Rotation3::from_axis_angle(&Vector3::y_axis(), self.pointing_offset).into_inner();
            self.spacecraft.write(SpacecraftStateMsg {
                position_m: self.expected_ground_position_n + CHAIN_DISTANCE_M * ray_n,
                sigma_bn: dcm_to_mrp(dcm_nb.transpose()),
                ..SpacecraftStateMsg::default()
            });
        }

        /// Upstream `resetChain`: producer order; the first one is an init.
        fn reset_chain(&mut self, first: bool) {
            let modules: [&mut dyn Module; 4] = [
                &mut self.ground,
                &mut self.transmitter,
                &mut self.receiver,
                &mut self.budget,
            ];
            for module in modules {
                if first {
                    module.init();
                } else {
                    module.reset(&context(0));
                }
            }
        }

        /// Upstream `updateChain`: one producer-before-consumer tick.
        fn update_chain(&mut self) {
            self.ground.update(&context(0));
            self.transmitter.update(&context(0));
            self.receiver.update(&context(0));
            self.budget.update(&context(0));
        }

        /// Upstream `checkLink`: the producers' outputs against independent
        /// geometry, and the link against a written-payload reference and the
        /// closed-form budget.
        fn check_link(&self) {
            let ground_output = self.ground.current_ground_state_out_msg.read();
            let receive_output = self.receiver.antenna_out_msg.read();
            let transmit_output = self.transmitter.antenna_out_msg.read();
            assert!((ground_output.r_hat_lp_n.dot(&self.expected_normal_n) - 1.0).abs() <= 1.0e-14);
            assert!((receive_output.n_hat_lp_n - self.expected_normal_n).norm() <= 1.0e-14);
            assert!((receive_output.r_ap_n_m - self.expected_ground_position_n).norm() <= 1.0e-8);
            assert_eq!(receive_output.environment, Earth);
            assert_eq!(transmit_output.environment, Space);
            assert!(self.budget.atmos_att);

            // The reference gets the normal and altitude independently of
            // the antenna output.
            let mut reference_ground = receive_output;
            reference_ground.n_hat_lp_n = self.expected_normal_n;
            reference_ground.r_ap_n_m = self.expected_ground_position_n;
            let ground_message = Output::new(reference_ground);
            let space_message = Output::new(transmit_output);
            let mut reference = LinkBudget::new("reference");
            reference.atmos_att = true;
            let (first, second) = if self.ground_first {
                (&ground_message, &space_message)
            } else {
                (&space_message, &ground_message)
            };
            first.connect_to(&mut reference.antenna_in_msg_1);
            second.connect_to(&mut reference.antenna_in_msg_2);
            reference.init();
            reference.update(&context(0));
            assert!(reference.l_atm_db() > 0.0);
            assert!((self.budget.l_atm_db() - reference.l_atm_db()).abs() <= 1.0e-12);

            let beamwidth = (16.0 * 2f64.ln() / 10f64.powf(CHAIN_DIRECTIVITY_DB / 10.0)).sqrt();
            let pointing_loss =
                40.0 * 2f64.ln() / 10f64.ln() * (self.pointing_offset / beamwidth).powi(2);
            let fspl = 20.0
                * (4.0 * std::f64::consts::PI * CHAIN_DISTANCE_M * CHAIN_FREQUENCY_HZ
                    / SPEED_LIGHT)
                    .log10();
            // The ground sky is 200 K.
            let noise = K_BOLTZMANN
                * (CHAIN_RECEIVER_TEMPERATURE_K
                    + (1.0 - CHAIN_EFFICIENCY) * CHAIN_AMBIENT_TEMPERATURE_K
                    + CHAIN_EFFICIENCY * 200.0)
                * CHAIN_BANDWIDTH_HZ;
            let eirp =
                10.0 * (CHAIN_TRANSMIT_POWER_W * CHAIN_EFFICIENCY).log10() + CHAIN_DIRECTIVITY_DB;
            let received = eirp + CHAIN_DIRECTIVITY_DB + 10.0 * CHAIN_EFFICIENCY.log10()
                - fspl
                - pointing_loss
                - reference.l_atm_db();
            let cnr = 10f64.powf(received / 10.0) / noise;
            assert!((self.budget.l_fspl_db() - fspl).abs() <= 1.0e-10);
            assert!((self.budget.l_point_db() - pointing_loss).abs() <= 1.0e-10);
            let output = self.budget.link_budget_out_msg.read();
            let (used, unused) = if self.ground_first {
                (output.cnr_1, output.cnr_2)
            } else {
                (output.cnr_2, output.cnr_1)
            };
            assert!((used - cnr).abs() <= cnr * 1.0e-10, "{used} vs {cnr}");
            assert_eq!(unused, 0.0);
        }
    }

    /// The `GroundGeometry` parametrization: altitude, input order and
    /// pointing offset.
    fn chain_parameters() -> Vec<(f64, bool, f64)> {
        let mut parameters = Vec::new();
        for altitude in [0.0, 1624.0] {
            for ground_first in [false, true] {
                for offset in [0.0, 0.02] {
                    parameters.push((altitude, ground_first, offset));
                }
            }
        }
        parameters
    }

    /// `AntennaProducerChain.InitializesFromFirstProducerTick`.
    #[test]
    fn producer_chain_initializes_from_first_producer_tick() {
        for (altitude, ground_first, offset) in chain_parameters() {
            let mut chain = ProducerChain::new(altitude, ground_first, offset);
            chain.reset_chain(true);
            chain.update_chain();
            chain.check_link();
        }
    }

    /// `AntennaProducerChain.WaitsForBothAntennaMessages`.
    #[test]
    fn producer_chain_waits_for_both_antenna_messages() {
        for (altitude, ground_first, offset) in chain_parameters() {
            let mut chain = ProducerChain::new(altitude, ground_first, offset);
            chain.reset_chain(true);
            chain.budget.update(&context(0));
            let output = chain.budget.link_budget_out_msg.read();
            assert_eq!(
                (output.cnr_1, output.cnr_2, output.bandwidth_hz),
                (0.0, 0.0, 0.0)
            );
            chain.ground.update(&context(0));
            chain.transmitter.update(&context(0));
            chain.budget.update(&context(0));
            assert!(chain.budget.atmos_att);
            assert_eq!((chain.budget.cnr_1(), chain.budget.cnr_2()), (0.0, 0.0));
            assert_eq!(chain.budget.link_budget_out_msg.read().bandwidth_hz, 0.0);
            chain.receiver.update(&context(0));
            chain.budget.update(&context(0));
            chain.check_link();
        }
    }

    /// `AntennaProducerChain.WaitsForGroundPositionWhenAntennaRunsFirst`.
    #[test]
    fn producer_chain_waits_for_ground_position_when_antenna_runs_first() {
        for (altitude, ground_first, offset) in chain_parameters() {
            let mut chain = ProducerChain::new(altitude, ground_first, offset);
            chain.reset_chain(true);
            chain.transmitter.update(&context(0));
            chain.receiver.update(&context(0));
            chain.ground.update(&context(0));
            chain.budget.update(&context(0));
            assert!(chain.budget.atmos_att);
            assert_eq!(chain.budget.l_atm_db(), 0.0);
            let output = chain.budget.link_budget_out_msg.read();
            assert_eq!(
                (output.cnr_1, output.cnr_2, output.bandwidth_hz),
                (0.0, 0.0, 0.0)
            );
            chain.transmitter.update(&context(0));
            chain.receiver.update(&context(0));
            chain.ground.update(&context(0));
            chain.budget.update(&context(0));
            chain.check_link();
        }
    }

    /// `AntennaProducerChain.RebuildsAfterStationAltitudeChanges`.
    #[test]
    fn producer_chain_rebuilds_after_station_altitude_changes() {
        for (altitude, ground_first, offset) in chain_parameters() {
            let mut chain = ProducerChain::new(altitude, ground_first, offset);
            chain.reset_chain(true);
            chain.update_chain();
            chain.check_link();
            let first_loss = chain.budget.l_atm_db();
            chain.set_geometry(altitude + 2000.0);
            chain.reset_chain(false);
            let output = chain.budget.link_budget_out_msg.read();
            assert_eq!((output.cnr_1, output.cnr_2), (0.0, 0.0));
            assert_eq!(chain.budget.l_atm_db(), 0.0);
            chain.update_chain();
            chain.check_link();
            assert!(chain.budget.l_atm_db() < first_loss);
        }
    }

    /// Upstream's hand-written antenna payload of the producer-input tests.
    fn written_antenna(environment: AntennaEnvironment, x_m: f64) -> AntennaLogMsg {
        AntennaLogMsg {
            environment,
            antenna_state: RxTx,
            frequency_hz: CHAIN_FREQUENCY_HZ,
            bandwidth_hz: CHAIN_BANDWIDTH_HZ,
            eta_r: CHAIN_EFFICIENCY,
            p_n_w: 1.0e-14,
            p_eirp_db: 20.0,
            directivity_db: CHAIN_DIRECTIVITY_DB,
            r_an_n_m: Vector3::new(x_m, 0.0, 0.0),
            ..AntennaLogMsg::default()
        }
    }

    /// `LinkBudgetProducerInputs.InitializesSpaceToSpaceLink`: written
    /// space-to-space inputs disable the atmosphere and give a valid link.
    #[test]
    fn producer_inputs_initialize_space_to_space_link() {
        let earth_radius = REQ_EARTH * 1.0e3;
        let first = Output::new(written_antenna(Space, earth_radius + 800.0e3));
        let second = Output::new(written_antenna(
            Space,
            earth_radius + 800.0e3 + CHAIN_DISTANCE_M,
        ));
        let mut model = LinkBudget::new("model");
        model.atmos_att = true;
        model.pointing_loss = false;
        first.connect_to(&mut model.antenna_in_msg_1);
        second.connect_to(&mut model.antenna_in_msg_2);
        model.init();
        model.update(&context(0));
        assert!(!model.atmos_att);
        assert_eq!(model.l_atm_db(), 0.0);
        assert!(model.cnr_1() > 0.0 && model.cnr_2() > 0.0);
    }

    /// `LinkBudgetProducerInputs.GroundPositionOnlyGatesAtmosphere`: without
    /// atmosphere a ground antenna need not have published a position.
    #[test]
    fn producer_inputs_ground_position_only_gates_atmosphere() {
        let earth_radius = REQ_EARTH * 1.0e3;
        let space = Output::new(written_antenna(Space, earth_radius + CHAIN_DISTANCE_M));
        let ground = Output::new(written_antenna(Earth, earth_radius));
        let mut model = LinkBudget::new("model");
        model.pointing_loss = false;
        space.connect_to(&mut model.antenna_in_msg_1);
        ground.connect_to(&mut model.antenna_in_msg_2);
        model.init();
        model.update(&context(0));
        assert!(model.cnr_1() > 0.0 && model.cnr_2() > 0.0);
    }

    // ---------------------------------------------------------------------
    // Port-specific checks
    // ---------------------------------------------------------------------

    /// Upstream's JSON tables parse to the built-in ones.
    #[test]
    fn json_gas_tables_parse() {
        let oxygen = r#"{"title": "t", "data": [
            {"f0": 50.474214, "a1": 0.975, "a2": 9.651, "a3": 6.690, "a4": 0.0, "a5": 2.566, "a6": 6.850}
        ]}"#;
        let lines = parse_lookup_table(oxygen, GasType::Oxygen).unwrap();
        assert_eq!(lines, [super::OXYGEN_LINES[0]]);
        assert!(parse_lookup_table(oxygen, GasType::WaterVapour).is_err());
        assert!(parse_lookup_table("{}", GasType::Oxygen).is_err());
    }

    /// A ground-to-ground link is not supported and publishes nothing.
    #[test]
    fn ground_to_ground_link_publishes_zero() {
        let first = written_antenna(Earth, REQ_EARTH * 1.0e3);
        let mut second = first.clone();
        second.r_an_n_m.y = 1.0e5;
        let (model, log) = run_link(first, second, false, 0);
        assert_eq!(model.antenna_placement(), AntennaPlacement::UnknownBlocked);
        assert_eq!(log[0], LinkBudgetMsg::default());
    }
}
