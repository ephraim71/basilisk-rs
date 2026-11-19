//! Planetary albedo at instruments: port of upstream Basilisk
//! `simulation/environment/albedo`.
//!
//! Each registered planet is a [`PlanetGrid`] of Lambertian patches; the
//! module sums the sunlight they reflect towards each instrument. Per
//! instrument it publishes an [`AlbedoMsg`] holding the albedo from every
//! visible patch ("max") and from the patches inside the instrument's field
//! of view, cosine weighted by its normal, both as ratios of the solar flux
//! at the planet and as fluxes at the instrument.
//!
//! A planet uses either a uniform average albedo or a CSV albedo map
//! (upstream `supportData/AlbedoData`, fetched by `just
//! fetch-environment-data`). Optionally each patch is shadowed by the
//! planet's penumbra model ([`AlbedoConfig::eclipse_case`]).
//!
//! # Differences from upstream
//!
//! * Upstream reads a planet's name from its `SpicePlanetStateMsg`; the
//!   planet messages here carry none, so each planet is registered with an
//!   [`AlbedoPlanet`].
//! * Upstream builds the planet grids at `Reset`; here they are built when the
//!   planet is added, with the grid defaults then in
//!   [`AlbedoConfig`], and an unreadable albedo map is an error of
//!   [`Albedo::add_planet_data_model`].
//! * Upstream's single-planet `planetInMsg` of the radiation base is unused
//!   by albedo and not ported.

use std::fmt;
use std::path::PathBuf;

use nalgebra::{Matrix3, Vector3};

use super::planet_radiation::{
    AlbedoDataFile, PlanetGrid, PlanetGridConfig, PlanetRadiationError, authalic_radius,
    compute_illumination_at_da, planet_inertial_to_fixed, solar_flux_at,
};
use crate::messages::{
    AlbedoMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg, SunEphemerisMsg,
};
use crate::utilities::astro_constants::{
    D2R, REQ_EARTH, REQ_JUPITER, REQ_MARS, REQ_MERCURY, REQ_MOON, REQ_NEPTUNE, REQ_SATURN,
    REQ_URANUS, REQ_VENUS, RP_EARTH, RP_MARS,
};
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// The planets albedo knows radii and average albedos for (upstream's
/// planet-name look-ups `lookupPlanetRadius` and `getAlbedoAverage`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlbedoPlanet {
    Mercury,
    Venus,
    Earth,
    Moon,
    /// Mars, also upstream's "mars barycenter".
    Mars,
    Jupiter,
    Saturn,
    Uranus,
    Neptune,
}

impl AlbedoPlanet {
    /// The planet with upstream's SPICE name `name` ("earth", "mars
    /// barycenter", ...), if albedo knows it.
    pub fn from_spice_name(name: &str) -> Option<Self> {
        Some(match name {
            "mercury" => Self::Mercury,
            "venus" => Self::Venus,
            "earth" => Self::Earth,
            "moon" => Self::Moon,
            "mars" | "mars barycenter" => Self::Mars,
            "jupiter" => Self::Jupiter,
            "saturn" => Self::Saturn,
            "uranus" => Self::Uranus,
            "neptune" => Self::Neptune,
            _ => return None,
        })
    }

    /// Upstream `lookupPlanetRadius`: equatorial and, for Earth and Mars,
    /// polar radius \[m\].
    pub fn radii_m(self) -> (f64, Option<f64>) {
        match self {
            Self::Mercury => (REQ_MERCURY * 1000.0, None),
            Self::Venus => (REQ_VENUS * 1000.0, None),
            Self::Earth => (REQ_EARTH * 1000.0, Some(RP_EARTH * 1000.0)),
            Self::Moon => (REQ_MOON * 1000.0, None),
            Self::Mars => (REQ_MARS * 1000.0, Some(RP_MARS * 1000.0)),
            Self::Jupiter => (REQ_JUPITER * 1000.0, None),
            Self::Saturn => (REQ_SATURN * 1000.0, None),
            Self::Uranus => (REQ_URANUS * 1000.0, None),
            Self::Neptune => (REQ_NEPTUNE * 1000.0, None),
        }
    }

    /// Upstream `getAlbedoAverage` \[-\]: the planet's average Bond albedo.
    pub fn average_albedo(self) -> f64 {
        match self {
            Self::Mercury => 0.119,
            Self::Venus => 0.75,
            Self::Earth => 0.29,
            Self::Moon => 0.123,
            Self::Mars => 0.16,
            Self::Jupiter | Self::Saturn => 0.34,
            Self::Uranus => 0.29,
            Self::Neptune => 0.31,
        }
    }
}

/// User-settable parameters of [`Albedo`].
#[derive(Clone, Debug)]
pub struct AlbedoConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `nHat_B_default` \[-\]: instrument normal used when an
    /// instrument is added with a zero normal.
    pub nhat_b_default: Vector3<f64>,
    /// Upstream `fov_default` \[rad\]: field-of-view half angle used when an
    /// instrument is added with a negative one.
    pub fov_default_rad: f64,
    /// Upstream `illuminationFactorAtdA` \[-\]: constant illumination of every
    /// patch when `eclipse_case` is off.
    pub illumination_factor_at_da: f64,
    /// Upstream `altitudeRateLimit` \[-\]: a planet contributes nothing to an
    /// instrument whose altitude exceeds this multiple of the planet radius;
    /// negative disables the limit.
    pub altitude_rate_limit: f64,
    /// Upstream `eclipseCase`: shadow each patch with the penumbra model.
    pub eclipse_case: bool,
    /// Upstream `defaultNumLat`: latitude bands of an average-albedo grid
    /// added without an explicit size.
    pub default_num_lat: usize,
    /// Upstream `defaultNumLon`: longitude zones of an average-albedo grid
    /// added without an explicit size.
    pub default_num_lon: usize,
}

impl Default for AlbedoConfig {
    fn default() -> Self {
        Self {
            name: "albedo".to_string(),
            nhat_b_default: Vector3::new(1.0, 0.0, 0.0),
            fov_default_rad: 90.0 * D2R,
            illumination_factor_at_da: 1.0,
            altitude_rate_limit: -1.0,
            eclipse_case: false,
            default_num_lat: 180,
            default_num_lon: 360,
        }
    }
}

/// One instrument (upstream `instConfig_t`). The defaults are upstream's
/// unset values, which [`Albedo::add_instrument_config`] replaces by the
/// module defaults.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AlbedoInstrumentConfig {
    /// Upstream `fov` \[rad\]: field-of-view half angle; negative selects
    /// [`AlbedoConfig::fov_default_rad`].
    pub fov_rad: f64,
    /// Upstream `nHat_B` \[-\]: instrument normal, body components (need not
    /// be unit); zero selects [`AlbedoConfig::nhat_b_default`].
    pub nhat_b: Vector3<f64>,
    /// Upstream `r_IB_B` \[m\]: instrument position relative to the body
    /// origin, body components.
    pub r_ib_b_m: Vector3<f64>,
}

impl Default for AlbedoInstrumentConfig {
    fn default() -> Self {
        Self {
            fov_rad: -1.0,
            nhat_b: Vector3::zeros(),
            r_ib_b_m: Vector3::zeros(),
        }
    }
}

/// A configuration problem upstream's `Reset` reports with `bskError`; each
/// is logged at [`Module::init`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlbedoConfigError {
    /// No planet was added.
    NoPlanets,
    /// No instrument was added.
    NoInstruments,
    /// A required input is not connected.
    InputNotConnected(&'static str),
}

impl fmt::Display for AlbedoConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoPlanets => write!(
                f,
                "no planets registered; call add_planet_average_model or add_planet_data_model"
            ),
            Self::NoInstruments => {
                write!(f, "no instruments configured; call add_instrument_config")
            }
            Self::InputNotConnected(port) => write!(f, "{port} is not connected"),
        }
    }
}

#[derive(Clone, Debug)]
struct Instrument {
    fov_rad: f64,
    nhat_b: Vector3<f64>,
    r_ib_b_m: Vector3<f64>,
}

#[derive(Clone, Debug)]
struct PlanetEntry {
    planet: AlbedoPlanet,
    grid: PlanetGrid,
    /// \[m\] authalic radius.
    r_planet_m: f64,
}

/// Per-instrument sums over the planets of one update.
#[derive(Clone, Copy, Debug, Default)]
struct InstrumentAlbedo {
    albedo_max: f64,
    albedo: f64,
    flux_max_w_per_m2: f64,
    flux_w_per_m2: f64,
}

/// Albedo seen by a spacecraft's instruments (upstream `Albedo`).
#[derive(Clone, Debug)]
pub struct Albedo {
    /// Module configuration.
    pub config: AlbedoConfig,
    /// Upstream `spacecraftStateInMsg`: spacecraft position and attitude.
    pub spacecraft_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `sunPositionInMsg`: Sun position.
    pub sun_position_in_msg: Input<SunEphemerisMsg>,
    /// One planet state input per added planet, in the order added.
    pub planet_state_in_msgs: Vec<Input<PlanetStateMsg>>,
    /// Upstream `albOutMsgs`: one output per added instrument.
    pub albedo_out_msgs: Vec<Output<AlbedoMsg>>,
    instruments: Vec<Instrument>,
    planets: Vec<PlanetEntry>,
}

impl Albedo {
    pub fn new(config: AlbedoConfig) -> Self {
        Self {
            config,
            spacecraft_state_in_msg: Input::default(),
            sun_position_in_msg: Input::default(),
            planet_state_in_msgs: Vec::new(),
            albedo_out_msgs: Vec::new(),
            instruments: Vec::new(),
            planets: Vec::new(),
        }
    }

    /// Upstream `addInstrumentConfig`: adds an instrument and returns its
    /// output. A negative field of view or a zero normal is replaced by the
    /// module default (with a warning, as upstream); the normal is
    /// normalized.
    pub fn add_instrument_config(&mut self, config: AlbedoInstrumentConfig) -> &Output<AlbedoMsg> {
        let index = self.albedo_out_msgs.len();
        let fov_rad = if config.fov_rad < 0.0 {
            log::warn!(
                "{}: instrument {index}'s half field of view angle is negative; the default is \
                 used",
                self.config.name
            );
            self.config.fov_default_rad
        } else {
            config.fov_rad
        };
        let nhat_b = if config.nhat_b == Vector3::zeros() {
            log::warn!(
                "{}: instrument {index}'s unit normal is zero; the default is used",
                self.config.name
            );
            self.config.nhat_b_default
        } else {
            config.nhat_b / config.nhat_b.norm()
        };
        self.instruments.push(Instrument {
            fov_rad,
            nhat_b,
            r_ib_b_m: config.r_ib_b_m,
        });
        self.albedo_out_msgs.push(Output::default());
        &self.albedo_out_msgs[index]
    }

    /// Upstream `addPlanetandAlbedoAverageModel(msg)`: adds `planet` with its
    /// average albedo on the default grid, and returns its state input.
    pub fn add_planet_average_model(
        &mut self,
        planet: AlbedoPlanet,
    ) -> Result<&mut Input<PlanetStateMsg>, PlanetRadiationError> {
        let (num_lat, num_lon) = (self.config.default_num_lat, self.config.default_num_lon);
        self.add_planet(planet, planet.average_albedo(), num_lat, num_lon, None)
    }

    /// Upstream `addPlanetandAlbedoAverageModel(msg, ALB_avg, numLat,
    /// numLon)`: adds `planet` with a uniform albedo on a
    /// `num_lat` x `num_lon` grid. As upstream, a negative albedo selects the
    /// planet's average and a zero size the default one.
    pub fn add_planet_average_model_explicit(
        &mut self,
        planet: AlbedoPlanet,
        albedo_avg: f64,
        num_lat: usize,
        num_lon: usize,
    ) -> Result<&mut Input<PlanetStateMsg>, PlanetRadiationError> {
        let albedo_avg = if albedo_avg < 0.0 {
            planet.average_albedo()
        } else {
            albedo_avg
        };
        let num_lat = if num_lat == 0 {
            self.config.default_num_lat
        } else {
            num_lat
        };
        let num_lon = if num_lon == 0 {
            self.config.default_num_lon
        } else {
            num_lon
        };
        self.add_planet(planet, albedo_avg, num_lat, num_lon, None)
    }

    /// Upstream `addPlanetandAlbedoDataModel`: adds `planet` with the albedo
    /// map `data_path/file_name`, whose shape sets the grid.
    pub fn add_planet_data_model(
        &mut self,
        planet: AlbedoPlanet,
        data_path: impl Into<PathBuf>,
        file_name: impl Into<String>,
    ) -> Result<&mut Input<PlanetStateMsg>, PlanetRadiationError> {
        let data = AlbedoDataFile {
            data_path: data_path.into(),
            file_name: file_name.into(),
        };
        let (num_lat, num_lon) = (self.config.default_num_lat, self.config.default_num_lon);
        self.add_planet(
            planet,
            planet.average_albedo(),
            num_lat,
            num_lon,
            Some(data),
        )
    }

    fn add_planet(
        &mut self,
        planet: AlbedoPlanet,
        albedo_avg: f64,
        num_lat: usize,
        num_lon: usize,
        albedo_data: Option<AlbedoDataFile>,
    ) -> Result<&mut Input<PlanetStateMsg>, PlanetRadiationError> {
        let (req_m, rp_m) = planet.radii_m();
        let grid = PlanetGrid::new(&PlanetGridConfig {
            num_lat,
            num_lon,
            albedo_avg,
            albedo_data,
            req_m,
            rp_m,
            ..Default::default()
        })?;
        self.planets.push(PlanetEntry {
            planet,
            grid,
            r_planet_m: authalic_radius(req_m, rp_m),
        });
        self.planet_state_in_msgs.push(Input::default());
        Ok(self
            .planet_state_in_msgs
            .last_mut()
            .expect("planet input was just added"))
    }

    /// Upstream `customReset`'s and the radiation base `Reset`'s checks.
    pub fn configuration_errors(&self) -> Vec<AlbedoConfigError> {
        let mut errors = Vec::new();
        if self.planets.is_empty() {
            errors.push(AlbedoConfigError::NoPlanets);
        }
        if self.instruments.is_empty() {
            errors.push(AlbedoConfigError::NoInstruments);
        }
        if !self.spacecraft_state_in_msg.is_connected() {
            errors.push(AlbedoConfigError::InputNotConnected(
                "spacecraft_state_in_msg",
            ));
        }
        if !self.sun_position_in_msg.is_connected() {
            errors.push(AlbedoConfigError::InputNotConnected("sun_position_in_msg"));
        }
        errors
    }

    /// Upstream `evaluatePlanet`: adds one planet's albedo to every
    /// instrument's sums.
    #[allow(clippy::too_many_arguments)]
    fn evaluate_planet(
        &self,
        entry: &PlanetEntry,
        planet: &PlanetStateMsg,
        r_bn_n_m: Vector3<f64>,
        dcm_bn: &Matrix3<f64>,
        r_sn_n_m: Vector3<f64>,
        s_sun_w_per_m2: f64,
        sums: &mut [InstrumentAlbedo],
    ) {
        let r_pn_n_m = planet.position_inertial_m;
        let r_sp_n_m = r_sn_n_m - r_pn_n_m;
        let inertial_to_fixed = planet_inertial_to_fixed(planet);

        for (instrument, sum) in self.instruments.iter().zip(sums.iter_mut()) {
            let r_ib_n_m = dcm_bn.transpose() * instrument.r_ib_b_m;
            let r_ip_n_m = r_ib_n_m + r_bn_n_m - r_pn_n_m;

            if self.config.altitude_rate_limit >= 0.0 {
                let altitude_m = r_ip_n_m.norm() - entry.r_planet_m;
                if altitude_m / entry.r_planet_m > self.config.altitude_rate_limit {
                    log::warn!(
                        "{}: the altitude to planet radius limit is exceeded for {:?}; its \
                         albedo is set to zero",
                        self.config.name,
                        entry.planet
                    );
                    continue;
                }
            }

            // Solar flux at the instrument, corrected for its distance.
            let r_si_n_m = r_sp_n_m - r_ip_n_m;
            let solar_flux_at_instrument = solar_flux_at(r_si_n_m.norm());

            let patches = entry.grid.compute_patches(
                r_ib_n_m + r_bn_n_m,
                r_sn_n_m,
                r_pn_n_m,
                &inertial_to_fixed,
                entry.r_planet_m,
                s_sun_w_per_m2,
            );
            let nhat_n = dcm_bn.transpose() * instrument.nhat_b;
            let cos_fov = instrument.fov_rad.cos();

            let mut albedo = 0.0;
            let mut albedo_max = 0.0;
            for patch in patches {
                if patch.albedo_flux_w_per_m2 <= 0.0 {
                    continue;
                }
                let ratio = patch.albedo_flux_w_per_m2 / s_sun_w_per_m2;
                let illumination = if self.config.eclipse_case {
                    compute_illumination_at_da(entry.r_planet_m, patch.r_dap_n_m, r_sp_n_m)
                } else {
                    self.config.illumination_factor_at_da
                };
                albedo_max += ratio * illumination;

                // Cosine between the instrument normal and the patch direction.
                let f3 = nhat_n.dot(&(-patch.r_hat_ida_n));
                if f3 >= cos_fov {
                    albedo += ratio * f3 * illumination;
                }
            }

            sum.albedo += albedo;
            sum.albedo_max += albedo_max;
            sum.flux_max_w_per_m2 += solar_flux_at_instrument * albedo_max;
            sum.flux_w_per_m2 += solar_flux_at_instrument * albedo;
        }
    }
}

impl Module for Albedo {
    fn init(&mut self) {
        for error in self.configuration_errors() {
            log::error!("{}: {error}", self.config.name);
        }
        for output in &self.albedo_out_msgs {
            output.write(AlbedoMsg::default());
        }
    }

    fn update(&mut self, _context: &SimulationContext) {
        let spacecraft = self.spacecraft_state_in_msg.read();
        let r_bn_n_m = spacecraft.position_m;
        let dcm_bn = mrp_to_dcm(spacecraft.sigma_bn);
        let r_sn_n_m = self.sun_position_in_msg.read().sun_position_inertial_m;

        let mut sums = vec![InstrumentAlbedo::default(); self.instruments.len()];
        for (entry, input) in self.planets.iter().zip(&self.planet_state_in_msgs) {
            let planet = input.read();
            let sun_distance_m = (r_sn_n_m - planet.position_inertial_m).norm();
            if sun_distance_m <= 0.0 {
                log::error!(
                    "{}: Sun-planet ({:?}) distance <= 0 m",
                    self.config.name,
                    entry.planet
                );
                continue;
            }
            let s_sun_w_per_m2 = solar_flux_at(sun_distance_m);
            self.evaluate_planet(
                entry,
                &planet,
                r_bn_n_m,
                &dcm_bn,
                r_sn_n_m,
                s_sun_w_per_m2,
                &mut sums,
            );
        }

        for (output, sum) in self.albedo_out_msgs.iter().zip(sums) {
            output.write(AlbedoMsg {
                albedo_at_instrument_max: sum.albedo_max,
                albedo_flux_at_instrument_max_w_per_m2: sum.flux_max_w_per_m2,
                albedo_at_instrument: sum.albedo,
                albedo_flux_at_instrument_w_per_m2: sum.flux_w_per_m2,
            });
        }
    }
}

/// Ports of upstream `test_albedo.py` (the `PlanetGrid` tests are in
/// [`super::planet_radiation`]).
///
/// `test_eclipseCase_cutoff_notice.py` has no counterpart: it reminds
/// upstream to retire its deprecated public `eclipseCase` member, which this
/// port does not have.
#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{Albedo, AlbedoConfig, AlbedoConfigError, AlbedoInstrumentConfig, AlbedoPlanet};
    use crate::environment::planet_radiation::PlanetRadiationError;
    use crate::environment::test_assets::asset_or_skip;
    use crate::messages::{
        AlbedoMsg, Output, PlanetOrientation, PlanetStateMsg, SpacecraftStateMsg, SunEphemerisMsg,
    };
    use crate::utilities::astro_constants::{AU, D2R, REQ_EARTH, REQ_MARS};
    use crate::{Module, SimulationContext};

    fn context() -> SimulationContext {
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn planet_at(position_m: Vector3<f64>) -> PlanetStateMsg {
        PlanetStateMsg {
            position_inertial_m: position_m,
            velocity_inertial_mps: Vector3::zeros(),
            orientation: Some(PlanetOrientation::identity()),
        }
    }

    /// Wires the inputs, runs one update and returns the first instrument's
    /// output. `planets` are in the order the module added them.
    fn run_once(
        albedo: &mut Albedo,
        sun_position_m: Vector3<f64>,
        planets: &[PlanetStateMsg],
        spacecraft_position_m: Vector3<f64>,
    ) -> AlbedoMsg {
        let sun = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: sun_position_m,
            ..Default::default()
        });
        let spacecraft = Output::new(SpacecraftStateMsg {
            position_m: spacecraft_position_m,
            ..Default::default()
        });
        let planet_outputs: Vec<_> = planets.iter().cloned().map(Output::new).collect();
        albedo.sun_position_in_msg.connect(sun.slot());
        albedo.spacecraft_state_in_msg.connect(spacecraft.slot());
        for (input, output) in albedo.planet_state_in_msgs.iter_mut().zip(&planet_outputs) {
            input.connect(output.slot());
        }
        albedo.init();
        albedo.update(&context());
        albedo.albedo_out_msgs[0].read()
    }

    fn standard_instrument(alpha: f64) -> AlbedoInstrumentConfig {
        AlbedoInstrumentConfig {
            fov_rad: 80.0 * D2R,
            nhat_b: Vector3::new(-alpha.cos(), -alpha.sin(), 0.0),
            r_ib_b_m: Vector3::zeros(),
        }
    }

    #[derive(Clone, Copy, Debug)]
    enum ModelType {
        AverageImplicit,
        AverageExplicit,
        Data,
    }

    /// Upstream `test_unitAlbedo`: one instrument looking at the planet from
    /// 6000 km altitude, 71 deg from the Sun line.
    fn unit_albedo(planet: AlbedoPlanet, model: ModelType, use_eclipse: bool) -> Option<f64> {
        let mut albedo = Albedo::new(AlbedoConfig {
            name: "Albedo_0".to_string(),
            eclipse_case: use_eclipse,
            ..Default::default()
        });
        match model {
            ModelType::Data => {
                let file = match planet {
                    AlbedoPlanet::Earth => "albedo/Earth_ALB_2018_CERES_All_10x10.csv",
                    _ => "albedo/Mars_ALB_TES_10x10.csv",
                };
                let path = asset_or_skip(file)?;
                albedo
                    .add_planet_data_model(
                        planet,
                        path.parent().unwrap(),
                        path.file_name().unwrap().to_str().unwrap(),
                    )
                    .expect("albedo map");
            }
            ModelType::AverageExplicit => {
                albedo
                    .add_planet_average_model_explicit(planet, 0.5, 200, 400)
                    .expect("valid grid");
            }
            ModelType::AverageImplicit => {
                albedo.add_planet_average_model(planet).expect("valid grid");
            }
        }

        let (sun_position, req_m) = match planet {
            AlbedoPlanet::Earth => (Vector3::new(-AU * 1000.0, 0.0, 0.0), REQ_EARTH * 1000.0),
            _ => (
                Vector3::new(-1.5 * AU * 1000.0, 0.0, 0.0),
                REQ_MARS * 1000.0,
            ),
        };
        let r_sc = req_m + 6000.0 * 1000.0;
        let alpha = 71.0 * D2R;
        albedo.add_instrument_config(standard_instrument(alpha));

        let out = run_once(
            &mut albedo,
            sun_position,
            &[planet_at(Vector3::zeros())],
            r_sc * Vector3::new(alpha.cos(), alpha.sin(), 0.0),
        );
        Some(out.albedo_at_instrument)
    }

    #[test]
    fn albedo_at_instrument_matches_upstream_truth() {
        use AlbedoPlanet::{Earth, Mars};
        use ModelType::{AverageExplicit, AverageImplicit, Data};
        let cases = [
            (Earth, Data, true, 0.0022055492477917),
            (Earth, Data, false, 0.0022055492477917),
            (Earth, AverageExplicit, true, 0.0041742091531996),
            (Earth, AverageExplicit, false, 0.004174209177079),
            (Earth, AverageImplicit, true, 0.002421222716229847),
            (Earth, AverageImplicit, false, 0.002421222716229847),
            (Mars, Data, true, 0.0014001432717662),
            (Mars, Data, false, 0.0014001432717662),
            (Mars, AverageExplicit, true, 0.0035681407388827),
            (Mars, AverageExplicit, false, 0.0035681407390035),
            (Mars, AverageImplicit, true, 0.0011418311186365906),
            (Mars, AverageImplicit, false, 0.0011418311186365906),
        ];
        for (planet, model, use_eclipse, truth) in cases {
            let Some(albedo) = unit_albedo(planet, model, use_eclipse) else {
                continue;
            };
            assert!(
                (albedo - truth).abs() < 1e-12,
                "{planet:?} {model:?} eclipse={use_eclipse}: got {albedo:.16}, expected {truth}"
            );
        }
    }

    /// Upstream `test_albedo_invalid_file`.
    #[test]
    fn a_missing_albedo_map_is_an_error() {
        let mut albedo = Albedo::new(AlbedoConfig::default());
        let result = albedo.add_planet_data_model(
            AlbedoPlanet::Earth,
            std::env::temp_dir(),
            "does_not_exist.file",
        );
        assert!(matches!(
            result,
            Err(PlanetRadiationError::AlbedoData { .. })
        ));
        assert!(albedo.planet_state_in_msgs.is_empty());
    }

    #[test]
    fn missing_planets_instruments_and_inputs_are_configuration_errors() {
        assert_eq!(
            Albedo::new(AlbedoConfig::default()).configuration_errors(),
            vec![
                AlbedoConfigError::NoPlanets,
                AlbedoConfigError::NoInstruments,
                AlbedoConfigError::InputNotConnected("spacecraft_state_in_msg"),
                AlbedoConfigError::InputNotConnected("sun_position_in_msg"),
            ]
        );
    }

    /// Upstream `_run_albedo_at_planet_offset`: the explicit 200 x 400 Earth
    /// with eclipses, the whole scene shifted by `offset`.
    fn albedo_at_planet_offset(offset: Vector3<f64>) -> f64 {
        let mut albedo = Albedo::new(AlbedoConfig {
            eclipse_case: true,
            ..Default::default()
        });
        albedo
            .add_planet_average_model_explicit(AlbedoPlanet::Earth, 0.5, 200, 400)
            .unwrap();
        let alpha = 71.0 * D2R;
        albedo.add_instrument_config(standard_instrument(alpha));
        let r_sc = REQ_EARTH * 1e3 + 6000e3;
        run_once(
            &mut albedo,
            offset + Vector3::new(-AU * 1000.0, 0.0, 0.0),
            &[planet_at(offset)],
            offset + r_sc * Vector3::new(alpha.cos(), alpha.sin(), 0.0),
        )
        .albedo_at_instrument
    }

    /// Upstream `test_albedo_eclipse_uses_planet_relative_sun_vector`.
    #[test]
    fn eclipse_output_is_translation_invariant() {
        let at_origin = albedo_at_planet_offset(Vector3::zeros());
        let offset = albedo_at_planet_offset(Vector3::new(1e12, 0.0, 0.0));
        assert!(
            at_origin > 0.0,
            "reference albedo at origin must be positive"
        );
        let rel_err = (at_origin - offset).abs() / at_origin;
        assert!(
            rel_err < 1e-10,
            "origin={at_origin:.15e}, offset={offset:.15e}, rel_err={rel_err:.2e}"
        );
    }

    /// Upstream `_albedo_run_earth`: the explicit 200 x 400 Earth, no
    /// eclipses, one instrument.
    fn albedo_run_earth(config: AlbedoConfig, instrument: AlbedoInstrumentConfig) -> AlbedoMsg {
        let mut albedo = Albedo::new(config);
        albedo
            .add_planet_average_model_explicit(AlbedoPlanet::Earth, 0.5, 200, 400)
            .unwrap();
        albedo.add_instrument_config(instrument);
        let r_sc = REQ_EARTH * 1000.0 + 6000.0 * 1000.0;
        let alpha = 71.0 * D2R;
        run_once(
            &mut albedo,
            Vector3::new(-AU * 1000.0, 0.0, 0.0),
            &[planet_at(Vector3::zeros())],
            r_sc * Vector3::new(alpha.cos(), alpha.sin(), 0.0),
        )
    }

    fn assert_relative(actual: f64, expected: f64, rel: f64, what: &str) {
        assert!(
            (actual - expected).abs() <= rel * expected.abs(),
            "{what}: {actual:.15e} vs {expected:.15e}"
        );
    }

    #[test]
    fn altitude_rate_limit_suppresses_output() {
        let instrument = standard_instrument(71.0 * D2R);
        let free = albedo_run_earth(AlbedoConfig::default(), instrument);
        let limited = albedo_run_earth(
            AlbedoConfig {
                altitude_rate_limit: 0.0,
                ..Default::default()
            },
            instrument,
        );
        assert!(
            free.albedo_at_instrument > 0.0,
            "baseline albedo must be positive"
        );
        assert_eq!(limited.albedo_at_instrument, 0.0);
    }

    #[test]
    fn unset_instrument_values_fall_back_to_the_module_defaults() {
        let alpha = 71.0 * D2R;
        let explicit = albedo_run_earth(AlbedoConfig::default(), standard_instrument(alpha));
        assert!(explicit.albedo_at_instrument > 0.0);

        let defaults = albedo_run_earth(
            AlbedoConfig {
                nhat_b_default: Vector3::new(-alpha.cos(), -alpha.sin(), 0.0),
                fov_default_rad: 80.0 * D2R,
                ..Default::default()
            },
            AlbedoInstrumentConfig::default(),
        );
        assert_relative(
            defaults.albedo_at_instrument,
            explicit.albedo_at_instrument,
            1e-10,
            "default instrument",
        );
    }

    #[test]
    fn instrument_normal_is_normalized() {
        let instrument = standard_instrument(71.0 * D2R);
        let unit = albedo_run_earth(AlbedoConfig::default(), instrument);
        let scaled = albedo_run_earth(
            AlbedoConfig::default(),
            AlbedoInstrumentConfig {
                nhat_b: 5.0 * instrument.nhat_b,
                ..instrument
            },
        );
        assert!(unit.albedo_at_instrument > 0.0);
        assert_relative(
            scaled.albedo_at_instrument,
            unit.albedo_at_instrument,
            1e-10,
            "scaled normal",
        );
    }

    #[test]
    fn illumination_factor_scales_albedo_linearly() {
        let instrument = standard_instrument(71.0 * D2R);
        let full = albedo_run_earth(AlbedoConfig::default(), instrument);
        let half = albedo_run_earth(
            AlbedoConfig {
                illumination_factor_at_da: 0.5,
                ..Default::default()
            },
            instrument,
        );
        assert!(full.albedo_at_instrument > 0.0);
        assert_relative(
            half.albedo_at_instrument,
            0.5 * full.albedo_at_instrument,
            1e-10,
            "half illumination",
        );
        assert_relative(
            half.albedo_flux_at_instrument_w_per_m2,
            0.5 * full.albedo_flux_at_instrument_w_per_m2,
            1e-10,
            "half illumination flux",
        );
    }

    #[test]
    fn albedo_flux_is_positive_and_bounded_by_the_unfiltered_flux() {
        let out = albedo_run_earth(AlbedoConfig::default(), standard_instrument(71.0 * D2R));
        assert!(out.albedo_flux_at_instrument_w_per_m2 > 0.0);
        assert!(out.albedo_flux_at_instrument_max_w_per_m2 > 0.0);
        assert!(
            out.albedo_flux_at_instrument_max_w_per_m2
                >= out.albedo_flux_at_instrument_w_per_m2 - 1e-12
        );
    }

    /// Upstream `_run_albedo_multi_planet_scene`: 120 x 240 "earths" at
    /// `planet_positions`, one wide instrument along `+x`.
    fn multi_planet_scene(planet_positions: &[Vector3<f64>]) -> AlbedoMsg {
        let mut albedo = Albedo::new(AlbedoConfig::default());
        albedo.add_instrument_config(AlbedoInstrumentConfig {
            fov_rad: 120.0 * D2R,
            nhat_b: Vector3::new(1.0, 0.0, 0.0),
            r_ib_b_m: Vector3::zeros(),
        });
        for _ in planet_positions {
            albedo
                .add_planet_average_model_explicit(AlbedoPlanet::Earth, 0.5, 120, 240)
                .unwrap();
        }
        let planets: Vec<_> = planet_positions.iter().copied().map(planet_at).collect();
        run_once(
            &mut albedo,
            Vector3::new(-1.0e8, 0.0, 0.0),
            &planets,
            Vector3::new(0.0, 8.0e7, 0.0),
        )
    }

    #[test]
    fn two_planets_superpose() {
        let p_a = Vector3::new(-2.0e7, 0.0, 0.0);
        let p_b = Vector3::new(2.0e7, 0.0, 0.0);
        let a = multi_planet_scene(&[p_a]).albedo_at_instrument_max;
        let b = multi_planet_scene(&[p_b]).albedo_at_instrument_max;
        let ab = multi_planet_scene(&[p_a, p_b]).albedo_at_instrument_max;
        assert!(a > 0.0 && b > 0.0 && ab > 0.0);
        assert_relative(ab, a + b, 5e-3, "superposition");
    }

    #[test]
    fn planet_order_does_not_matter() {
        let p_a = Vector3::new(-2.0e7, 0.0, 0.0);
        let p_b = Vector3::new(2.0e7, 0.0, 0.0);
        let ab = multi_planet_scene(&[p_a, p_b]);
        let ba = multi_planet_scene(&[p_b, p_a]);
        assert_relative(
            ab.albedo_at_instrument_max,
            ba.albedo_at_instrument_max,
            1e-10,
            "ratio",
        );
        assert_relative(
            ab.albedo_flux_at_instrument_max_w_per_m2,
            ba.albedo_flux_at_instrument_max_w_per_m2,
            1e-10,
            "flux",
        );
    }

    /// The instrument offset and the attitude move the instrument and turn
    /// its normal: a body rotated 180 deg about `z` with the normal along
    /// `+x` looks where the unrotated `-x` normal does.
    #[test]
    fn instrument_normal_follows_the_attitude() {
        let alpha = 71.0 * D2R;
        let reference = albedo_run_earth(AlbedoConfig::default(), standard_instrument(alpha));

        let mut albedo = Albedo::new(AlbedoConfig::default());
        albedo
            .add_planet_average_model_explicit(AlbedoPlanet::Earth, 0.5, 200, 400)
            .unwrap();
        let normal = standard_instrument(alpha).nhat_b;
        albedo.add_instrument_config(AlbedoInstrumentConfig {
            fov_rad: 80.0 * D2R,
            nhat_b: -normal,
            r_ib_b_m: Vector3::zeros(),
        });
        let sun = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::new(-AU * 1000.0, 0.0, 0.0),
            ..Default::default()
        });
        let r_sc = REQ_EARTH * 1000.0 + 6000.0 * 1000.0;
        // MRP of a 180 deg rotation about z.
        let spacecraft = Output::new(SpacecraftStateMsg {
            position_m: r_sc * Vector3::new(alpha.cos(), alpha.sin(), 0.0),
            sigma_bn: Vector3::new(0.0, 0.0, 1.0),
            ..Default::default()
        });
        let planet = Output::new(PlanetStateMsg {
            orientation: Some(PlanetOrientation {
                inertial_to_fixed: Matrix3::zeros(),
                inertial_to_fixed_dot: Matrix3::zeros(),
            }),
            ..Default::default()
        });
        albedo.sun_position_in_msg.connect(sun.slot());
        albedo.spacecraft_state_in_msg.connect(spacecraft.slot());
        albedo.planet_state_in_msgs[0].connect(planet.slot());
        albedo.init();
        albedo.update(&context());

        // A singular planet orientation is replaced by the identity.
        assert_relative(
            albedo.albedo_out_msgs[0].read().albedo_at_instrument,
            reference.albedo_at_instrument,
            1e-12,
            "rotated body",
        );
    }

    #[test]
    fn planet_names_and_constants() {
        assert_eq!(
            AlbedoPlanet::from_spice_name("mars barycenter"),
            Some(AlbedoPlanet::Mars)
        );
        assert_eq!(AlbedoPlanet::from_spice_name("pluto"), None);
        assert_eq!(AlbedoPlanet::Earth.average_albedo(), 0.29);
        assert_eq!(AlbedoPlanet::Venus.radii_m(), (6_051_800.0, None));
    }
}
