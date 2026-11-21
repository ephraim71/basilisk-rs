//! Gaussian-beam antenna: port of upstream Basilisk
//! `simulation/communication/simpleAntenna`.
//!
//! An antenna `A` is mounted on a spacecraft `B` (connect
//! [`SimpleAntenna::sc_state_in_msg`]) or at a ground location `L` on Earth
//! (connect [`SimpleAntenna::ground_state_in_msg`]); which one is connected
//! sets its [`AntennaEnvironment`] at init. From its directivity and
//! beamwidth ratio it derives the half-power beamwidths of an elliptical
//! Gaussian beam and its EIRP, and each update it publishes, in an
//! [`AntennaLogMsg`], its pose and its noise:
//!
//! * the sky temperature in the beam: for a space antenna, the Sun, Earth
//!   and Moon in proportion to how much of the beam they cover (the Sun only
//!   while not eclipsed, if an eclipse input is connected), and the
//!   galactic background plus the CMB in the rest; for a ground antenna a
//!   fixed 200 K;
//! * the ambient temperature: the configured one, or 150 K in space and the
//!   ITU-R P.835 troposphere at the station altitude on the ground;
//! * the antenna and system noise temperatures, the noise power over the
//!   bandwidth and the gain over noise temperature.
//!
//! The commanded mode comes from [`SimpleAntenna::antenna_set_state_in_msg`]
//! when connected, and from [`SimpleAntenna::set_antenna_state`] otherwise.
//!
//! # Differences from upstream
//!
//! * The radio parameters are a [`SimpleAntennaConfig`] checked by
//!   [`SimpleAntenna::new`] instead of setters with `bskError`s, so they
//!   cannot be left unset; an empty name is rejected there too.
//! * Upstream's planet inputs are SPICE messages identified by
//!   `PlanetName`; here each planet input is added with its
//!   [`AntennaPlanet`], and the Sun input is a [`SunEphemerisMsg`].
//! * The Haslam 408 MHz sky map (`useHaslamMap`, `configureBrightnessFile`)
//!   needs CFITSIO and the Haslam FITS file and is not ported; the galactic
//!   background is always the ITU-R P.372 model.
//! * An environment that cannot be determined (both or neither state input
//!   connected; upstream's `_ENVIRONMENT_UNKNOWN`) is logged at init, as
//!   upstream's reset `bskError`, and the antenna then publishes nothing.
//! * A brightness temperature outside its frequency range (an upstream
//!   `bskError`) is logged, and that source contributes nothing.
//! * Upstream reads no message header, so no write-time stand-in is needed.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    ANTENNA_NAME_SIZE, AntennaEnvironment, AntennaLogMsg, AntennaState, AntennaStateMsg,
    EclipseMsg, GroundStateMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg, SunEphemerisMsg,
};
use crate::utilities::astro_constants::{
    CMB_TEMPERATURE, K_BOLTZMANN, REQ_EARTH, REQ_JUPITER, REQ_MARS, REQ_MERCURY, REQ_MOON,
    REQ_NEPTUNE, REQ_SATURN, REQ_SUN, REQ_URANUS, REQ_VENUS,
};
use crate::utilities::brightness_temperature_solar_system::{
    RadioSource, brightness_temperature_k,
};
use crate::utilities::linear_algebra::{safe_acos, safe_asin, safe_sqrt};
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_dcm};
use crate::{Module, SimulationContext};

/// Upstream `minThreshold` \[-\]: smallest bandwidth, transmit power and
/// temperatures accepted.
const MIN_THRESHOLD: f64 = 1e-6;
/// Sky temperature of a ground antenna \[K\] (upstream's fixed value).
const GROUND_SKY_TEMPERATURE_K: f64 = 200.0;
/// Ambient temperature of a space antenna \[K\] when none is configured.
const SPACE_AMBIENT_TEMPERATURE_K: f64 = 150.0;
/// ITU-R P.835-6 sea-level temperature \[K\] of the ground ambient model.
const GROUND_TEMPERATURE_K: f64 = 288.15;
/// ITU-R P.835-6 tropospheric lapse rate \[K/m\] of the ground ambient
/// model.
const LAPSE_RATE_K_PER_M: f64 = 0.0065;

/// A body whose position an antenna reads for the sky temperature (upstream
/// `getPlanetEquatorialRadius`'s SPICE names). Only the Earth and the Moon
/// have a brightness temperature, as upstream; the others are ignored.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AntennaPlanet {
    /// Upstream "mercury".
    Mercury,
    /// Upstream "venus".
    Venus,
    /// Upstream "earth".
    Earth,
    /// Upstream "moon".
    Moon,
    /// Upstream "mars" and "mars barycenter".
    Mars,
    /// Upstream "jupiter barycenter".
    Jupiter,
    /// Upstream "saturn".
    Saturn,
    /// Upstream "uranus".
    Uranus,
    /// Upstream "neptune".
    Neptune,
    /// Upstream "custom", whose radius is upstream's `rEqCustom`.
    Custom {
        /// Equatorial radius \[m\]; must be positive.
        equatorial_radius_m: f64,
    },
}

impl AntennaPlanet {
    /// Upstream `getPlanetEquatorialRadius` \[m\].
    pub fn equatorial_radius_m(self) -> f64 {
        let radius_km = match self {
            Self::Mercury => REQ_MERCURY,
            Self::Venus => REQ_VENUS,
            Self::Earth => REQ_EARTH,
            Self::Moon => REQ_MOON,
            Self::Mars => REQ_MARS,
            Self::Jupiter => REQ_JUPITER,
            Self::Saturn => REQ_SATURN,
            Self::Uranus => REQ_URANUS,
            Self::Neptune => REQ_NEPTUNE,
            Self::Custom {
                equatorial_radius_m,
            } => return equatorial_radius_m,
        };
        radius_km * 1000.0
    }

    /// The brightness model of the body, if it has one.
    fn radio_source(self) -> Option<RadioSource> {
        match self {
            Self::Earth => Some(RadioSource::Earth),
            Self::Moon => Some(RadioSource::Moon),
            _ => None,
        }
    }
}

/// Radio parameters and mounting of a [`SimpleAntenna`]: upstream's setters.
#[derive(Clone, Debug)]
pub struct SimpleAntennaConfig {
    /// Upstream `antennaName`: name of the antenna, also its instance name;
    /// non-empty and shorter than [`ANTENNA_NAME_SIZE`] characters.
    pub name: String,
    /// Upstream `frequency` \[Hz\]: centre frequency, within 30 MHz to
    /// 60 GHz (exclusive).
    pub frequency_hz: f64,
    /// Upstream `B` \[Hz\]: bandwidth, above 1e-6 Hz.
    pub bandwidth_hz: f64,
    /// Upstream `DdB` \[dB\]: directivity, above 9 dB for the Gaussian beam
    /// assumption.
    pub directivity_db: f64,
    /// Upstream `k` \[-\]: beamwidth ratio `HPBW_az / HPBW_el`, within 0.5
    /// to 5 (exclusive); 1 for a symmetric beam.
    pub hpbw_ratio: f64,
    /// Upstream `P_Tx` \[W\]: transmit power, above 1e-6 W.
    pub p_tx_w: f64,
    /// Upstream `P_Rx` \[W\]: receive power. A negative value is ignored with
    /// a warning, as upstream's setter does, and 0 W is used.
    pub p_rx_w: f64,
    /// Upstream `T_E` \[K\]: equivalent noise temperature of the receiver
    /// chain, at least 1e-6 K.
    pub equivalent_noise_temperature_k: f64,
    /// Upstream `eta_r` \[-\]: radiation efficiency, within 0 to 1.
    pub radiation_efficiency: f64,
    /// Upstream `T_AmbientSet` \[K\]: ambient temperature, at least 1e-6 K.
    /// `None` is upstream's unset `-1`: computed from the environment.
    pub ambient_temperature_k: Option<f64>,
    /// Upstream `r_AB_B` \[m\]: antenna position relative to the spacecraft
    /// hub, body components (space antennas).
    pub r_ab_b_m: Vector3<f64>,
    /// Upstream `sigma_AB` \[-\]: MRP attitude of the antenna frame relative
    /// to the body frame (space antennas); the boresight is the antenna +z
    /// axis.
    pub sigma_ab: Vector3<f64>,
}

impl SimpleAntennaConfig {
    /// Upstream's setter and reset checks, each a `bskError` there.
    pub fn validate(&self) -> Result<(), String> {
        // Each test fails for NaN, as upstream's checks pass it.
        let in_range = |value: f64, low: f64, high: f64| value > low && value < high;
        let above = |value: f64, low: f64| value > low;
        let at_least = |value: f64, low: f64| value >= low;
        if self.name.is_empty() {
            return Err("SimpleAntenna antennaName has not been set!".to_string());
        }
        if self.name.len() >= ANTENNA_NAME_SIZE {
            return Err(format!(
                "SimpleAntenna: Antenna name length exceeds {ANTENNA_NAME_SIZE} characters."
            ));
        }
        if !in_range(self.frequency_hz, 3e7, 6e10) {
            return Err(format!(
                "SimpleAntenna operating frequency {} Hz is outside the valid range of 30 MHz to \
                 60 GHz.",
                self.frequency_hz
            ));
        }
        if !above(self.bandwidth_hz, MIN_THRESHOLD) {
            return Err(format!(
                "SimpleAntenna bandwidth B {} Hz must be larger than 1e-6 Hz.",
                self.bandwidth_hz
            ));
        }
        if !above(self.directivity_db, 9.0) {
            return Err(format!(
                "SimpleAntenna directivity {} dB must be > 9 dB for the Gaussian beam assumption.",
                self.directivity_db
            ));
        }
        if !in_range(self.hpbw_ratio, 0.5, 5.0) {
            return Err(format!(
                "SimpleAntenna HPBW ratio {} must be between 0.5 and 5.",
                self.hpbw_ratio
            ));
        }
        if !above(self.p_tx_w, MIN_THRESHOLD) {
            return Err(format!(
                "SimpleAntenna transmit power P_Tx {} W must be larger than 1e-6 W.",
                self.p_tx_w
            ));
        }
        if !at_least(self.equivalent_noise_temperature_k, MIN_THRESHOLD) {
            return Err(format!(
                "SimpleAntenna equivalent noise temperature T_E {} K must be at least 1e-6 K.",
                self.equivalent_noise_temperature_k
            ));
        }
        if !(0.0..=1.0).contains(&self.radiation_efficiency) {
            return Err(format!(
                "SimpleAntenna radiation efficiency eta_r {} must be between 0 and 1.",
                self.radiation_efficiency
            ));
        }
        if let Some(temperature) = self.ambient_temperature_k
            && !at_least(temperature, MIN_THRESHOLD)
        {
            return Err(format!(
                "SimpleAntenna ambient temperature {temperature} K must be at least 1e-6 K."
            ));
        }
        Ok(())
    }
}

/// Antenna with a Gaussian beam and a sky-noise model: upstream
/// `SimpleAntenna`.
#[derive(Clone, Debug)]
pub struct SimpleAntenna {
    /// Radio parameters and mounting. Checked by [`Self::new`]; the derived
    /// beamwidths and EIRP follow a change at the next init or reset.
    pub config: SimpleAntennaConfig,
    /// Upstream `scStateInMsg`: state of the spacecraft carrying the antenna
    /// (connect this or `ground_state_in_msg`).
    pub sc_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `groundStateInMsg`: state of the ground location carrying the
    /// antenna (connect this or `sc_state_in_msg`).
    pub ground_state_in_msg: Input<GroundStateMsg>,
    /// Upstream `antennaSetStateInMsg` (optional): commanded mode, read every
    /// update.
    pub antenna_set_state_in_msg: Input<AntennaStateMsg>,
    /// Upstream `sunInMsg` (optional): Sun position for the sky temperature
    /// of a space antenna; required when planets are added.
    pub sun_in_msg: Input<SunEphemerisMsg>,
    /// Upstream `planetInMsgs`: one input per planet, added by
    /// [`Self::add_planet_to_model`].
    pub planet_in_msgs: Vec<Input<PlanetStateMsg>>,
    /// Upstream `sunEclipseInMsg` (optional): Sun visibility at the
    /// spacecraft; a total eclipse removes the Sun from the sky.
    pub sun_eclipse_in_msg: Input<EclipseMsg>,
    /// Upstream `antennaOutMsg`: radio properties, noise and pose.
    pub antenna_out_msg: Output<AntennaLogMsg>,
    /// Upstream `environment`, set at init; `None` for upstream's
    /// `_ENVIRONMENT_UNKNOWN`.
    environment: Option<AntennaEnvironment>,
    /// Upstream `antennaState`.
    antenna_state: AntennaState,
    /// The body of each planet input.
    planets: Vec<AntennaPlanet>,
    /// Upstream `planetBuffer`: the planet states read this update.
    planet_buffer: Vec<PlanetStateMsg>,
    /// Upstream `scState`.
    sc_state: SpacecraftStateMsg,
    /// Upstream `groundState`.
    ground_state: GroundStateMsg,
    /// Upstream `sunData`.
    sun: SunEphemerisMsg,
    /// Upstream `illuminationFactor` \[-\].
    illumination_factor: f64,
    /// Upstream `HPBW` \[rad\]: half-power beamwidths, azimuth then
    /// elevation.
    hpbw_rad: [f64; 2],
    /// Upstream `P_eirp_dB` \[dBW\].
    eirp_dbw: f64,
    /// Upstream `P_N` \[W\].
    noise_power_w: f64,
    /// Upstream `G_TN` \[dB/K\].
    gain_over_noise_temperature_db_per_k: f64,
    /// Upstream `T_Ambient` \[K\].
    ambient_temperature_k: f64,
    /// Upstream `r_AN_N` \[m\].
    r_an_n_m: Vector3<f64>,
    /// Upstream `sigma_AN` \[-\].
    sigma_an: Vector3<f64>,
    /// Upstream `v_AN_N` \[m/s\].
    v_an_n_mps: Vector3<f64>,
    /// Upstream `r_AP_N` \[m\].
    r_ap_n_m: Vector3<f64>,
    /// Upstream `r_BN_N` \[m\]: spacecraft position.
    r_bn_n_m: Vector3<f64>,
}

impl SimpleAntenna {
    /// An antenna with `config`, off, with unconnected ports and no planets.
    ///
    /// # Panics
    ///
    /// If `config` fails [`SimpleAntennaConfig::validate`].
    pub fn new(mut config: SimpleAntennaConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid SimpleAntennaConfig: {message}");
        }
        if config.p_rx_w < 0.0 {
            log::warn!(
                "{}: SimpleAntenna receive power P_Rx {} W is negative; P_Rx = 0.0 W is used.",
                config.name,
                config.p_rx_w
            );
            config.p_rx_w = 0.0;
        }
        Self {
            config,
            sc_state_in_msg: Input::default(),
            ground_state_in_msg: Input::default(),
            antenna_set_state_in_msg: Input::default(),
            sun_in_msg: Input::default(),
            planet_in_msgs: Vec::new(),
            sun_eclipse_in_msg: Input::default(),
            antenna_out_msg: Output::default(),
            environment: None,
            antenna_state: AntennaState::Off,
            planets: Vec::new(),
            planet_buffer: Vec::new(),
            sc_state: SpacecraftStateMsg::default(),
            ground_state: GroundStateMsg::default(),
            sun: SunEphemerisMsg::default(),
            illumination_factor: 0.0,
            hpbw_rad: [-1.0, -1.0],
            eirp_dbw: 0.0,
            noise_power_w: 0.0,
            gain_over_noise_temperature_db_per_k: 0.0,
            ambient_temperature_k: -1.0,
            r_an_n_m: Vector3::zeros(),
            sigma_an: Vector3::zeros(),
            v_an_n_mps: Vector3::zeros(),
            r_ap_n_m: Vector3::zeros(),
            r_bn_n_m: Vector3::zeros(),
        }
    }

    /// Upstream `addPlanetToModel`: adds an input for `planet`'s state and
    /// returns it for connecting.
    ///
    /// # Panics
    ///
    /// If a custom planet's radius is not positive (upstream `bskError`).
    pub fn add_planet_to_model(&mut self, planet: AntennaPlanet) -> &mut Input<PlanetStateMsg> {
        if let AntennaPlanet::Custom {
            equatorial_radius_m,
        } = planet
        {
            assert!(
                equatorial_radius_m > 0.0,
                "SimpleAntenna: Invalid rEqCustom set."
            );
        }
        self.planets.push(planet);
        self.planet_buffer.push(PlanetStateMsg::default());
        self.planet_in_msgs.push(Input::default());
        self.planet_in_msgs
            .last_mut()
            .expect("just pushed a planet input")
    }

    /// Upstream `setAntennaState`: the mode until the next commanded state.
    pub fn set_antenna_state(&mut self, state: AntennaState) {
        self.antenna_state = state;
    }

    /// Upstream `getAntennaState`.
    pub fn antenna_state(&self) -> AntennaState {
        self.antenna_state
    }

    /// Upstream `getAntennaEnvironment`, set at init.
    pub fn environment(&self) -> Option<AntennaEnvironment> {
        self.environment
    }

    /// Upstream `getAntennaHPBW` \[rad\]: half-power beamwidths in azimuth
    /// and elevation, set at init.
    pub fn hpbw_rad(&self) -> [f64; 2] {
        self.hpbw_rad
    }

    /// Upstream `getAntennaNoisePower` \[W\].
    pub fn noise_power_w(&self) -> f64 {
        self.noise_power_w
    }

    /// Upstream `getP_eirp_dB` \[dBW\], set at init.
    pub fn eirp_dbw(&self) -> f64 {
        self.eirp_dbw
    }

    /// Upstream `getG_TN` \[dB/K\].
    pub fn gain_over_noise_temperature_db_per_k(&self) -> f64 {
        self.gain_over_noise_temperature_db_per_k
    }

    /// Upstream `getAntennaEnvTemperature` \[K\].
    pub fn ambient_temperature_k(&self) -> f64 {
        self.ambient_temperature_k
    }

    /// Upstream `Reset`'s connection checks and `initializeAntenna`.
    fn reset_state(&mut self) {
        let name = &self.config.name;
        let on_spacecraft = self.sc_state_in_msg.is_connected();
        let on_ground = self.ground_state_in_msg.is_connected();
        self.environment = match (on_spacecraft, on_ground) {
            (true, false) => Some(AntennaEnvironment::Space),
            (false, true) => Some(AntennaEnvironment::Earth),
            (true, true) => {
                log::error!(
                    "{name}: Both SimpleAntenna.scStateInMsg and SimpleAntenna.groundState were \
                     linked. Please link only one to define the environment."
                );
                None
            }
            (false, false) => {
                log::error!(
                    "{name}: Neither SimpleAntenna.scStateInMsg nor SimpleAntenna.groundState was \
                     linked."
                );
                None
            }
        };
        if !self.sun_in_msg.is_connected() && !self.planet_in_msgs.is_empty() {
            log::error!(
                "{name}: SimpleAntenna.sunInMsg was not linked while planetInMsgs is not empty. \
                 Sun data is required to calculate sky noise."
            );
        }
        if !self.antenna_set_state_in_msg.is_connected() {
            log::info!(
                "{name}: SimpleAntenna.antennaSetStateInMsg was not linked; the antenna state can \
                 only be changed through set_antenna_state."
            );
        }

        // Elliptical Gaussian beam of the configured directivity.
        let config = &self.config;
        let directivity = 10f64.powf(config.directivity_db / 10.0);
        let hpbw_el = ((2f64.ln() * 16.0) / (directivity * config.hpbw_ratio)).sqrt();
        let hpbw_az = config.hpbw_ratio * hpbw_el;
        self.hpbw_rad = [hpbw_az, hpbw_el];
        self.eirp_dbw = 10.0 * config.p_tx_w.log10()
            + 10.0 * config.radiation_efficiency.log10()
            + config.directivity_db;
    }

    /// Upstream `readInputMessages`.
    fn read_input_messages(&mut self) {
        match self.environment {
            Some(AntennaEnvironment::Space) => self.sc_state = self.sc_state_in_msg.read(),
            Some(AntennaEnvironment::Earth) => self.ground_state = self.ground_state_in_msg.read(),
            None => {}
        }
        if self.antenna_set_state_in_msg.is_connected() {
            self.antenna_state = self.antenna_set_state_in_msg.read().antenna_state;
        }
        if self.sun_in_msg.is_connected() {
            self.sun = self.sun_in_msg.read();
        }
        for (buffer, input) in self.planet_buffer.iter_mut().zip(&self.planet_in_msgs) {
            *buffer = input.read();
        }
        if self.sun_eclipse_in_msg.is_connected() {
            self.illumination_factor = self.sun_eclipse_in_msg.read().illumination_factor;
        }
    }

    /// Upstream `calculateAntennaPositionAndOrientation`.
    fn calculate_antenna_position_and_orientation(&mut self) {
        match self.environment {
            Some(AntennaEnvironment::Space) => {
                let dcm_nb = mrp_to_dcm(self.sc_state.sigma_bn).transpose();
                self.r_bn_n_m = self.sc_state.position_m;
                self.r_an_n_m = self.r_bn_n_m + dcm_nb * self.config.r_ab_b_m;
                let dcm_na = dcm_nb * mrp_to_dcm(self.config.sigma_ab).transpose();
                self.sigma_an = dcm_to_mrp(dcm_na.transpose());
                // The antenna moves with the spacecraft.
                self.v_an_n_mps = self.sc_state.velocity_mps;
            }
            Some(AntennaEnvironment::Earth) => {
                self.r_an_n_m = self.ground_state.position_ln_n_m;
                self.v_an_n_mps = Vector3::zeros();
                self.r_ap_n_m = self.ground_state.position_lp_n_m;
            }
            None => {}
        }
    }

    /// Brightness temperature of `source` at the antenna frequency, or zero
    /// (logged) outside the model's range.
    fn brightness_k(&self, source: RadioSource) -> f64 {
        brightness_temperature_k(source, self.config.frequency_hz).unwrap_or_else(|error| {
            log::error!(
                "{}: brightnessTemperatureSolarSystem: {error}",
                self.config.name
            );
            0.0
        })
    }

    /// Upstream `calculateTsky` \[K\]: sky brightness temperature in the
    /// beam.
    pub fn calculate_t_sky(&self) -> f64 {
        match self.environment {
            Some(AntennaEnvironment::Space) => {
                let (az_half, el_half) = (self.hpbw_rad[0] / 2.0, self.hpbw_rad[1] / 2.0);
                let mut galaxy_coverage = 1.0;
                let (mut t_earth, mut t_moon, mut t_sun, mut t_galaxy) = (0.0, 0.0, 0.0, 0.0);
                if self.sun_in_msg.is_connected() {
                    let mut sun_coverage = self.calculate_planet_coverage(
                        REQ_SUN * 1000.0,
                        self.sun.sun_position_inertial_m,
                        az_half,
                        el_half,
                    );
                    if !self.planet_in_msgs.is_empty() {
                        for (planet, state) in self.planets.iter().zip(&self.planet_buffer) {
                            let coverage = self.calculate_planet_coverage(
                                planet.equatorial_radius_m(),
                                state.position_inertial_m,
                                az_half,
                                el_half,
                            );
                            match planet.radio_source() {
                                Some(RadioSource::Earth) => {
                                    t_earth = self.brightness_k(RadioSource::Earth) * coverage;
                                    galaxy_coverage -= coverage;
                                }
                                Some(RadioSource::Moon) => {
                                    t_moon = self.brightness_k(RadioSource::Moon) * coverage;
                                    galaxy_coverage -= coverage;
                                }
                                _ => {}
                            }
                        }
                        if self.sun_eclipse_in_msg.is_connected() && self.illumination_factor == 0.0
                        {
                            sun_coverage = 0.0;
                        }
                    }
                    t_sun = self.brightness_k(RadioSource::Sun) * sun_coverage;
                    galaxy_coverage -= sun_coverage;
                    t_galaxy = self.brightness_k(RadioSource::Galaxy) + CMB_TEMPERATURE;
                }
                t_earth + t_moon + t_sun + t_galaxy * galaxy_coverage
            }
            Some(AntennaEnvironment::Earth) => GROUND_SKY_TEMPERATURE_K,
            None => 0.0,
        }
    }

    /// Upstream `calculatePlanetCoverage` \[-\]: fraction of the beam, of
    /// half-angles `az_half_rad` and `el_half_rad`, covered by a sphere of
    /// `radius_m` at `r_pn_n_m`. The beam and the body's disk are mapped to
    /// a unit circle and a circle, and the result is their overlap area.
    fn calculate_planet_coverage(
        &self,
        radius_m: f64,
        r_pn_n_m: Vector3<f64>,
        az_half_rad: f64,
        el_half_rad: f64,
    ) -> f64 {
        let r_pb_n = r_pn_n_m - self.r_bn_n_m;
        let distance_m = r_pb_n.norm();
        let p_hat_n = if distance_m > 0.0 {
            r_pb_n / distance_m
        } else {
            r_pb_n
        };
        let dcm_an: Matrix3<f64> = mrp_to_dcm(self.sigma_an);
        let p_hat_a = dcm_an * p_hat_n;

        let azimuth = p_hat_a.y.atan2(p_hat_a.z);
        let elevation = p_hat_a.x.atan2(p_hat_a.z);
        let angular_radius = safe_asin(radius_m / distance_m);

        let (y, x) = (azimuth / az_half_rad, elevation / el_half_rad);
        let d = safe_sqrt(x * x + y * y);
        let r = angular_radius / safe_sqrt(az_half_rad * el_half_rad);
        let beam_radius = 1.0;

        if d >= beam_radius + r {
            return 0.0;
        }
        if d <= (beam_radius - r).abs() {
            return (r * r / (beam_radius * beam_radius)).min(1.0);
        }
        let (big, small) = (beam_radius * beam_radius, r * r);
        let alpha = safe_acos((d * d + big - small) / (2.0 * d * beam_radius));
        let beta = safe_acos((d * d + small - big) / (2.0 * d * r));
        let overlap = big * alpha + small * beta
            - 0.5
                * safe_sqrt(
                    (-d + beam_radius + r)
                        * (d + beam_radius - r)
                        * (d - beam_radius + r)
                        * (d + beam_radius + r),
                );
        (overlap / (std::f64::consts::PI * big)).clamp(0.0, 1.0)
    }

    /// Upstream `calculateTambient` \[K\].
    fn calculate_t_ambient(&self) -> f64 {
        match self.environment {
            Some(AntennaEnvironment::Space) => SPACE_AMBIENT_TEMPERATURE_K,
            Some(AntennaEnvironment::Earth) => {
                // ITU-R P.835-6 eq. 1; the station is assumed below 11 km.
                let altitude_m = self.ground_state.position_lp_n_m.norm() - REQ_EARTH * 1000.0;
                GROUND_TEMPERATURE_K - LAPSE_RATE_K_PER_M * altitude_m
            }
            None => f64::NAN,
        }
    }

    /// Upstream `calculateAntennaNoise`.
    fn calculate_antenna_noise(&mut self) {
        let t_sky = self.calculate_t_sky();
        self.ambient_temperature_k = self
            .config
            .ambient_temperature_k
            .unwrap_or_else(|| self.calculate_t_ambient());
        let eta = self.config.radiation_efficiency;
        let t_antenna = (1.0 - eta) * self.ambient_temperature_k + eta * t_sky;
        let t_system = self.config.equivalent_noise_temperature_k + t_antenna;
        self.noise_power_w = K_BOLTZMANN * t_system * self.config.bandwidth_hz;
        self.gain_over_noise_temperature_db_per_k =
            self.config.directivity_db + 10.0 * eta.log10() - 10.0 * t_system.log10();
    }

    /// Upstream `populateOutputMsg`.
    fn output_message(&self) -> AntennaLogMsg {
        let config = &self.config;
        let on_ground = self.environment == Some(AntennaEnvironment::Earth);
        AntennaLogMsg {
            antenna_name: config.name.clone(),
            environment: self.environment.unwrap_or_default(),
            antenna_state: self.antenna_state,
            frequency_hz: config.frequency_hz,
            bandwidth_hz: config.bandwidth_hz,
            hpbw_az_rad: self.hpbw_rad[0],
            hpbw_el_rad: self.hpbw_rad[1],
            p_tx_w: config.p_tx_w,
            p_rx_w: config.p_rx_w,
            directivity_db: config.directivity_db,
            eta_r: config.radiation_efficiency,
            p_n_w: self.noise_power_w,
            p_eirp_db: self.eirp_dbw,
            g_tn_db_per_k: self.gain_over_noise_temperature_db_per_k,
            t_ambient_k: self.ambient_temperature_k,
            r_an_n_m: self.r_an_n_m,
            sigma_an: self.sigma_an,
            v_an_n_mps: self.v_an_n_mps,
            r_ap_n_m: if on_ground {
                self.r_ap_n_m
            } else {
                Vector3::zeros()
            },
            r_ap_p_m: Vector3::zeros(),
            n_hat_lp_n: if on_ground {
                self.ground_state.r_hat_lp_n
            } else {
                Vector3::zeros()
            },
        }
    }
}

impl Module for SimpleAntenna {
    fn init(&mut self) {
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    /// Upstream `UpdateState`.
    fn update(&mut self, _context: &SimulationContext) {
        if self.environment.is_none() {
            return;
        }
        self.read_input_messages();
        self.calculate_antenna_position_and_orientation();
        self.calculate_antenna_noise();
        self.antenna_out_msg.write(self.output_message());
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `simpleAntenna/_UnitTest/test_simpleAntenna.py`: every
    //! case of `test_simpleAntenna` except the one with `useHaslamMap`, with
    //! its truth values and accuracy.
    //!
    //! Not ported: the `useHaslamMap = True` case and
    //! `test_simpleAntenna_haslamMap_three_runs_and_pointing_difference`,
    //! which need the Haslam sky map (CFITSIO and the Haslam FITS file).

    use nalgebra::{Matrix3, Vector3};

    use super::{AntennaPlanet, SimpleAntenna, SimpleAntennaConfig};
    use crate::Module;
    use crate::messages::{
        AntennaEnvironment, AntennaLogMsg, AntennaState, AntennaStateMsg, EclipseMsg,
        GroundStateMsg, Output, PlanetStateMsg, SpacecraftStateMsg, SunEphemerisMsg,
    };
    use crate::onboard_data_handling::test_support::{TaskClock, context};
    use crate::utilities::astro_constants::{AU, K_BOLTZMANN, REQ_EARTH, SMA_MOON};
    use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_dcm};

    /// Upstream `ACCURACY`, compared as `unitTestSupport.isDoubleEqual`.
    const ACCURACY: f64 = 1e-6;
    /// Upstream `testProcessRate`, `macros.sec2nano(0.5)`.
    const STEP_NANOS: u64 = 500_000_000;
    const FREQUENCY_HZ: f64 = 2.2e9;
    const BANDWIDTH_HZ: f64 = 5e6;
    const DIRECTIVITY_DB: f64 = 20.0;
    const HPBW_RATIO: f64 = 1.0;
    const P_TX_W: f64 = 100.0;
    const P_RX_W: f64 = 10.0;
    const T_E_K: f64 = 50.0;
    const ETA_R: f64 = 0.6;

    /// Upstream `earth_horizon_sigma`: boresight on the Earth's limb.
    fn earth_horizon_sigma() -> Vector3<f64> {
        let theta = ((REQ_EARTH * 1e3) / ((800.0 + REQ_EARTH) * 1e3)).asin();
        dcm_to_mrp(Matrix3::new(
            0.0,
            0.0,
            1.0,
            theta.sin(),
            theta.cos(),
            0.0,
            -theta.cos(),
            theta.sin(),
            0.0,
        ))
    }

    /// One parametrization of upstream `test_simpleAntenna`.
    struct Case {
        state: AntennaState,
        on_ground: bool,
        ambient_set_k: Option<f64>,
        t_sky_true_k: f64,
        eclipse: f64,
        sigma_ab: Vector3<f64>,
        sc_pos_unit: Vector3<f64>,
        eclipse_linked: bool,
    }

    fn cases() -> Vec<Case> {
        use AntennaState::{Off, Rx, RxTx, Tx};
        let x = Vector3::x();
        let identity = Vector3::zeros();
        let toward_minus_x = dcm_to_mrp(Matrix3::new(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0));
        let toward_plus_y = dcm_to_mrp(Matrix3::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0));
        let toward_plus_x = dcm_to_mrp(Matrix3::new(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0));
        let case =
            |state, on_ground, ambient_set_k, t_sky_true_k, eclipse, sigma_ab, unit, linked| Case {
                state,
                on_ground,
                ambient_set_k,
                t_sky_true_k,
                eclipse,
                sigma_ab,
                sc_pos_unit: unit,
                eclipse_linked: linked,
            };
        vec![
            case(Off, false, Some(150.0), 5.45, 1.0, identity, x, true),
            case(Rx, false, Some(150.0), 5.45, 1.0, identity, x, true),
            case(Tx, false, Some(150.0), 5.45, 1.0, identity, x, true),
            case(RxTx, false, Some(150.0), 5.45, 1.0, identity, x, true),
            case(RxTx, true, Some(150.0), 200.0, 1.0, identity, x, true),
            case(RxTx, false, None, 5.45, 1.0, identity, x, true),
            case(RxTx, true, None, 200.0, 1.0, identity, x, true),
            // Pointing at the Earth with the Sun eclipsed.
            case(
                RxTx,
                false,
                Some(150.0),
                300.0,
                0.0,
                toward_minus_x,
                x,
                true,
            ),
            // Pointing at the Earth with the Sun visible but outside the beam.
            case(
                RxTx,
                false,
                Some(150.0),
                300.0,
                1.0,
                toward_plus_y,
                Vector3::y(),
                true,
            ),
            // Pointing at the Moon.
            case(
                RxTx,
                false,
                Some(150.0),
                5.660060787285246,
                1.0,
                toward_plus_x,
                x,
                true,
            ),
            // Pointing at the Sun, inside the beam.
            case(
                RxTx,
                false,
                Some(150.0),
                37.82128898104384,
                1.0,
                toward_minus_x,
                -x,
                true,
            ),
            // Pointing at the Earth's limb: partial coverage.
            case(
                RxTx,
                false,
                Some(150.0),
                147.9662263230431,
                0.0,
                earth_horizon_sigma(),
                x,
                true,
            ),
            // Space without the eclipse input.
            case(RxTx, false, Some(150.0), 5.45, 1.0, identity, x, false),
            // Ground without the eclipse input.
            case(RxTx, true, Some(150.0), 200.0, 1.0, identity, x, false),
        ]
    }

    fn config(case: &Case) -> SimpleAntennaConfig {
        SimpleAntennaConfig {
            name: "simpleAntenna".to_string(),
            frequency_hz: FREQUENCY_HZ,
            bandwidth_hz: BANDWIDTH_HZ,
            directivity_db: DIRECTIVITY_DB,
            hpbw_ratio: HPBW_RATIO,
            p_tx_w: P_TX_W,
            p_rx_w: P_RX_W,
            equivalent_noise_temperature_k: T_E_K,
            radiation_efficiency: ETA_R,
            ambient_temperature_k: case.ambient_set_k,
            r_ab_b_m: Vector3::new(0.0, 0.0, 1.0),
            sigma_ab: case.sigma_ab,
        }
    }

    #[track_caller]
    fn assert_double_equal(actual: f64, truth: f64, name: &str) {
        assert!(
            (actual - truth).abs() <= ACCURACY,
            "{name}: got {actual}, expected {truth}"
        );
    }

    /// Upstream `test_simpleAntenna` (through `simpleAntennaTestFunction`).
    #[test]
    fn simple_antenna() {
        for case in cases() {
            let mut module = SimpleAntenna::new(config(&case));
            let sun = Output::new(SunEphemerisMsg::default());
            let earth = Output::new(PlanetStateMsg {
                position_inertial_m: Vector3::new(AU * 1000.0, 0.0, 0.0),
                ..PlanetStateMsg::default()
            });
            let moon = Output::new(PlanetStateMsg {
                position_inertial_m: Vector3::new((AU + SMA_MOON) * 1000.0, 0.0, 0.0),
                ..PlanetStateMsg::default()
            });
            let eclipse = Output::new(EclipseMsg {
                illumination_factor: case.eclipse,
            });
            let command = Output::new(AntennaStateMsg {
                antenna_state: case.state,
            });
            let sc_pos = case.sc_pos_unit * (800.0 * 1000.0 + REQ_EARTH * 1e3);
            let r_bn_n = Vector3::new(AU * 1000.0, 0.0, 0.0) + sc_pos;
            let spacecraft = Output::new(SpacecraftStateMsg {
                position_m: r_bn_n,
                ..SpacecraftStateMsg::default()
            });
            let ground = Output::new(GroundStateMsg {
                position_ln_n_m: Vector3::new(AU * 1000.0, 0.0, 0.0),
                position_lp_n_m: Vector3::new(REQ_EARTH * 1000.0, 0.0, 0.0),
                ..GroundStateMsg::default()
            });
            if case.on_ground {
                ground.connect_to(&mut module.ground_state_in_msg);
            } else {
                spacecraft.connect_to(&mut module.sc_state_in_msg);
            }
            command.connect_to(&mut module.antenna_set_state_in_msg);
            sun.connect_to(&mut module.sun_in_msg);
            earth.connect_to(module.add_planet_to_model(AntennaPlanet::Earth));
            moon.connect_to(module.add_planet_to_model(AntennaPlanet::Moon));
            if case.eclipse_linked {
                eclipse.connect_to(&mut module.sun_eclipse_in_msg);
            }

            module.init();
            let mut log: Vec<AntennaLogMsg> = Vec::new();
            for tick in TaskClock::new(STEP_NANOS).run_until(1_000_000_000) {
                module.update(&context(tick));
                log.push(module.antenna_out_msg.read());
            }

            // Truth values, as upstream computes them.
            let t_ambient =
                case.ambient_set_k
                    .unwrap_or(if case.on_ground { 288.15 } else { 150.0 });
            let dir_lin = 10f64.powf(DIRECTIVITY_DB / 10.0);
            let hpbw_el = ((2f64.ln() * 16.0) / (dir_lin * HPBW_RATIO)).sqrt();
            let hpbw_az = HPBW_RATIO * hpbw_el;
            let t_ant = (1.0 - ETA_R) * t_ambient + ETA_R * case.t_sky_true_k;
            let t_s = T_E_K + t_ant;
            let g_tn = DIRECTIVITY_DB + 10.0 * ETA_R.log10() - 10.0 * t_s.log10();
            let p_n = K_BOLTZMANN * t_s * BANDWIDTH_HZ;
            let p_eirp = 10.0 * P_TX_W.log10() + 10.0 * ETA_R.log10() + DIRECTIVITY_DB;
            let (r_an_n, r_ap_n, sigma_an) = if case.on_ground {
                (
                    Vector3::new(AU * 1000.0, 0.0, 0.0),
                    Vector3::new(REQ_EARTH * 1000.0, 0.0, 0.0),
                    Vector3::zeros(),
                )
            } else {
                let dcm_bn = mrp_to_dcm(Vector3::zeros());
                let r_ab_n = dcm_bn.transpose() * Vector3::new(0.0, 0.0, 1.0);
                let dcm_an = mrp_to_dcm(case.sigma_ab) * dcm_bn;
                (r_bn_n + r_ab_n, Vector3::zeros(), dcm_to_mrp(dcm_an))
            };
            let environment = if case.on_ground {
                AntennaEnvironment::Earth
            } else {
                AntennaEnvironment::Space
            };

            assert_eq!(log.len(), 3);
            for sample in &log {
                assert_eq!(sample.environment, environment);
                assert_eq!(sample.antenna_state, case.state);
                assert_double_equal(sample.frequency_hz, FREQUENCY_HZ, "frequency");
                assert_double_equal(sample.bandwidth_hz, BANDWIDTH_HZ, "bandwidth");
                assert_double_equal(sample.hpbw_el_rad, hpbw_el, "HPBW_el");
                assert_double_equal(sample.hpbw_az_rad, hpbw_az, "HPBW_az");
                assert_double_equal(sample.p_tx_w, P_TX_W, "P_Tx");
                assert_double_equal(sample.p_rx_w, P_RX_W, "P_Rx");
                assert_double_equal(sample.directivity_db, DIRECTIVITY_DB, "DdB");
                assert_double_equal(sample.g_tn_db_per_k, g_tn, "G_TN");
                assert_double_equal(sample.p_n_w, p_n, "P_N");
                assert_double_equal(sample.p_eirp_db, p_eirp, "P_eirp_dB");
                assert_double_equal(sample.t_ambient_k, t_ambient, "T_Ambient");
                for k in 0..3 {
                    assert_double_equal(sample.r_an_n_m[k], r_an_n[k], "r_AN_N");
                    assert_double_equal(sample.sigma_an[k], sigma_an[k], "sigma_AN");
                    assert_double_equal(sample.r_ap_n_m[k], r_ap_n[k], "r_AP_N");
                }
            }
            // The sky temperature the noise power implies, as upstream infers
            // it. Upstream does not assert it; its G_TN check bounds it to
            // about 4e-5 K, and the limb case is 3e-6 K from the truth.
            let t_s_sim = log[2].p_n_w / (K_BOLTZMANN * BANDWIDTH_HZ);
            let t_sky_sim = (t_s_sim - T_E_K - (1.0 - ETA_R) * t_ambient) / ETA_R;
            assert!(
                (t_sky_sim - case.t_sky_true_k).abs() < 1e-5,
                "T_sky: got {t_sky_sim}, expected {}",
                case.t_sky_true_k
            );
            assert!((module.calculate_t_sky() - t_sky_sim).abs() < 1e-9);
        }
    }

    fn valid_config() -> SimpleAntennaConfig {
        SimpleAntennaConfig {
            name: "antenna".to_string(),
            frequency_hz: FREQUENCY_HZ,
            bandwidth_hz: BANDWIDTH_HZ,
            directivity_db: DIRECTIVITY_DB,
            hpbw_ratio: HPBW_RATIO,
            p_tx_w: P_TX_W,
            p_rx_w: P_RX_W,
            equivalent_noise_temperature_k: T_E_K,
            radiation_efficiency: ETA_R,
            ambient_temperature_k: None,
            r_ab_b_m: Vector3::zeros(),
            sigma_ab: Vector3::zeros(),
        }
    }

    /// Upstream's setter `bskError`s, one per parameter.
    #[test]
    fn config_rejects_upstream_setter_errors() {
        let invalid: [fn(&mut SimpleAntennaConfig); 11] = [
            |c| c.name = String::new(),
            |c| c.name = "x".repeat(20),
            |c| c.frequency_hz = 3e7,
            |c| c.frequency_hz = 6e10,
            |c| c.bandwidth_hz = 1e-6,
            |c| c.directivity_db = 9.0,
            |c| c.hpbw_ratio = 0.5,
            |c| c.p_tx_w = 1e-6,
            |c| c.equivalent_noise_temperature_k = 0.0,
            |c| c.radiation_efficiency = 1.5,
            |c| c.ambient_temperature_k = Some(0.0),
        ];
        assert!(valid_config().validate().is_ok());
        for change in invalid {
            let mut config = valid_config();
            change(&mut config);
            assert!(config.validate().is_err(), "{config:?}");
        }
    }

    /// Upstream's `setAntennaP_Rx` warns and ignores a negative power, which
    /// leaves the unset value that reset turns into 0 W.
    #[test]
    fn negative_receive_power_becomes_zero() {
        let mut config = valid_config();
        config.p_rx_w = -1.0;
        assert_eq!(SimpleAntenna::new(config).config.p_rx_w, 0.0);
    }

    /// Upstream's reset `bskError`: the environment needs exactly one state
    /// input; without one the antenna publishes nothing.
    #[test]
    fn undetermined_environment_publishes_nothing() {
        let mut antenna = SimpleAntenna::new(valid_config());
        antenna.init();
        antenna.update(&context(0));
        assert_eq!(antenna.environment(), None);
        assert_eq!(antenna.antenna_out_msg.read(), AntennaLogMsg::default());
    }

    #[test]
    fn state_setter_holds_until_commanded() {
        let spacecraft = Output::new(SpacecraftStateMsg::default());
        let mut antenna = SimpleAntenna::new(valid_config());
        spacecraft.connect_to(&mut antenna.sc_state_in_msg);
        antenna.set_antenna_state(AntennaState::Tx);
        antenna.init();
        antenna.update(&context(0));
        assert_eq!(
            antenna.antenna_out_msg.read().antenna_state,
            AntennaState::Tx
        );
        let command = Output::new(AntennaStateMsg {
            antenna_state: AntennaState::Rx,
        });
        command.connect_to(&mut antenna.antenna_set_state_in_msg);
        antenna.update(&context(STEP_NANOS));
        assert_eq!(antenna.antenna_state(), AntennaState::Rx);
    }

    #[test]
    #[should_panic(expected = "Invalid rEqCustom set")]
    fn custom_planet_needs_a_positive_radius() {
        SimpleAntenna::new(valid_config()).add_planet_to_model(AntennaPlanet::Custom {
            equatorial_radius_m: 0.0,
        });
    }
}
