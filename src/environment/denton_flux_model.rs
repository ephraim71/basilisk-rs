//! Geostationary plasma fluxes: port of upstream Basilisk
//! `simulation/environment/dentonFluxModel`.
//!
//! Denton et al. averaged ten years of electron and ion flux measurements in
//! the GEO belt by Kp index, energy and local time. The module interpolates
//! those tables (bilinearly in local time and log energy) at the spacecraft's
//! local time, for a fixed Kp index, on an output energy grid that starts at
//! 100 eV; lower energies are contaminated by secondary and photoelectrons in
//! the data. The tables are upstream's `supportData/DentonGEO` files, fetched
//! by `just fetch-environment-data`: `model_e_array_*.txt` (electrons) and
//! `model_i_array_*.txt` (ions), each an `all`, `low`, `mid` or `high`
//! activity average.
//!
//! # Differences from upstream
//!
//! * Upstream checks the configuration and reads the tables at `Reset`; here
//!   [`DentonFluxModel::new`] does, and returns the error.
//! * The Sun and Earth inputs are a [`SunEphemerisMsg`] and a
//!   [`PlanetStateMsg`] instead of upstream's `SpicePlanetStateMsg`s.
//! * At a local time between 23 h and 24 h upstream indexes local time 24,
//!   past the end of its 24-hour table; here that index wraps to 0 h, the
//!   same hour of the day.

use std::f64::consts::PI;
use std::fmt;
use std::path::{Path, PathBuf};

use nalgebra::{Vector2, Vector3};

use crate::messages::{
    Input, MAX_PLASMA_FLUX_SIZE, Output, PlanetStateMsg, PlasmaFluxMsg, SpacecraftStateMsg,
    SunEphemerisMsg,
};
use crate::{Module, SimulationContext};

/// Upstream `MAX_NUM_KPS`: Kp indices in the tables, `0o` to `9o`.
const MAX_NUM_KPS: usize = 28;
/// Upstream `MAX_NUM_ENERGIES`: energy bins in the tables.
const MAX_NUM_ENERGIES: usize = 40;
/// Upstream `MAX_NUM_LOCAL_TIMES`: hourly local-time bins in the tables.
const MAX_NUM_LOCAL_TIMES: usize = 24;
/// Upstream `MAX_NUM_VALUE_TYPES`: statistics per Kp index in the files
/// (mean, standard deviation, median and percentiles); only the mean is used.
const MAX_NUM_VALUE_TYPES: usize = 7;

/// Upstream `enElec` \[eV\]: center energies of the electron bins.
const ELECTRON_ENERGIES_EV: [f64; MAX_NUM_ENERGIES] = [
    1.034126,
    1.346516,
    1.817463,
    2.399564,
    3.161048,
    4.153217,
    5.539430,
    7.464148,
    9.836741,
    12.543499,
    16.062061,
    20.876962,
    27.183572,
    35.843437,
    47.179073,
    61.424732,
    80.120170,
    104.563461,
    136.914871,
    179.740982,
    235.406829,
    309.020721,
    405.806213,
    532.664123,
    699.243896,
    917.146484,
    1205.174438,
    1582.510986,
    2069.619628,
    2703.301269,
    3540.124511,
    4639.775390,
    6069.347656,
    7957.457519,
    10436.841796,
    13677.195312,
    17923.560546,
    23488.560546,
    30782.000000,
    40326.937500,
];

/// Upstream `enProt` \[eV\]: center energies of the ion (proton) bins.
const PROTON_ENERGIES_EV: [f64; MAX_NUM_ENERGIES] = [
    1.816424,
    2.284231,
    2.904752,
    3.639589,
    4.483188,
    5.671049,
    7.343667,
    9.450922,
    11.934194,
    15.105951,
    19.372854,
    24.943658,
    32.053474,
    41.142940,
    53.239536,
    68.940170,
    89.082473,
    115.585487,
    150.529022,
    196.249755,
    256.610107,
    335.709136,
    439.549621,
    574.766357,
    749.907531,
    982.261108,
    1278.967041,
    1662.856079,
    2170.886474,
    2829.989013,
    3691.509765,
    4822.499023,
    6300.260742,
    8217.569335,
    10726.390625,
    14001.280273,
    18276.244140,
    23856.085937,
    31140.962890,
    40649.562500,
];

/// User-settable parameters of [`DentonFluxModel`].
#[derive(Clone, Debug)]
pub struct DentonFluxModelConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `kpIndex`: the Kp index as two characters, `0o` to `9o`
    /// (`1-`, `3o`, `4+`, ...).
    pub kp_index: String,
    /// Upstream `numOutputEnergies`: energy bins in the output, 1 to
    /// [`MAX_PLASMA_FLUX_SIZE`].
    pub num_output_energies: usize,
    /// Upstream `eDataFullPath`: the electron table.
    pub electron_data_path: PathBuf,
    /// Upstream `iDataFullPath`: the ion table.
    pub ion_data_path: PathBuf,
}

/// Why a [`DentonFluxModel`] could not be built (each a `bskError` upstream).
#[derive(Debug)]
pub enum DentonFluxError {
    /// `num_output_energies` is zero or above [`MAX_PLASMA_FLUX_SIZE`].
    InvalidNumOutputEnergies(usize),
    /// `kp_index` is not one of `0o` ... `9o`.
    InvalidKpIndex(String),
    /// A table could not be read.
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    /// A table holds fewer numbers than the 28 x 7 x 40 x 24 expected, or a
    /// token that is not a number.
    BadTable { path: PathBuf, message: String },
}

impl fmt::Display for DentonFluxError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidNumOutputEnergies(count) => write!(
                f,
                "DentonFluxModel: numOutputEnergies must be between 1 and {MAX_PLASMA_FLUX_SIZE}, \
                 got {count}"
            ),
            Self::InvalidKpIndex(kp) => write!(
                f,
                "DentonFluxModel: Kp index '{kp}' must be two characters between 0o and 9o, such \
                 as '1-', '3o', '4+'"
            ),
            Self::Io { path, source } => {
                write!(
                    f,
                    "DentonFluxModel: could not open {}: {source}",
                    path.display()
                )
            }
            Self::BadTable { path, message } => {
                write!(
                    f,
                    "DentonFluxModel: bad table {}: {message}",
                    path.display()
                )
            }
        }
    }
}

impl std::error::Error for DentonFluxError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            _ => None,
        }
    }
}

/// Mean log10 fluxes of one Kp index, by energy then local time.
type MeanFluxes = Vec<[[f64; MAX_NUM_LOCAL_TIMES]; MAX_NUM_ENERGIES]>;

/// GEO electron and ion fluxes for a Kp index (upstream `DentonFluxModel`).
#[derive(Clone, Debug)]
pub struct DentonFluxModel {
    /// Module configuration.
    pub config: DentonFluxModelConfig,
    /// Upstream `scStateInMsg`: spacecraft position.
    pub spacecraft_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `earthStateInMsg`: Earth position.
    pub earth_state_in_msg: Input<PlanetStateMsg>,
    /// Upstream `sunStateInMsg`: Sun position.
    pub sun_state_in_msg: Input<SunEphemerisMsg>,
    /// Upstream `fluxOutMsg`: the fluxes on the output energy grid.
    pub flux_out_msg: Output<PlasmaFluxMsg>,
    /// Upstream `kpIndexCounter`: the table row of the Kp index, 0 to 27.
    kp_index_counter: usize,
    /// Upstream `inputEnergies` \[eV\]: the output energy grid.
    input_energies_ev: Vec<f64>,
    /// Upstream `logEnElec` and `logEnProt`: natural logs of the bin energies.
    log_electron_energies: [f64; MAX_NUM_ENERGIES],
    log_proton_energies: [f64; MAX_NUM_ENERGIES],
    /// Upstream `mean_e_flux` and `mean_i_flux` \[log10(cm^-2 s^-1 sr^-1
    /// eV^-1)\], for every Kp index.
    mean_electron_flux: MeanFluxes,
    mean_ion_flux: MeanFluxes,
}

impl DentonFluxModel {
    /// Upstream `Reset`: checks the configuration and reads both tables.
    pub fn new(config: DentonFluxModelConfig) -> Result<Self, DentonFluxError> {
        if config.num_output_energies == 0 || config.num_output_energies > MAX_PLASMA_FLUX_SIZE {
            return Err(DentonFluxError::InvalidNumOutputEnergies(
                config.num_output_energies,
            ));
        }
        let kp_index_counter = kp_index_counter(&config.kp_index)
            .ok_or_else(|| DentonFluxError::InvalidKpIndex(config.kp_index.clone()))?;

        // Upstream's integer division: `(40000 - 1) / numOutputEnergies`.
        let step_ev = (39_999 / config.num_output_energies) as f64;
        let mut input_energies_ev = vec![100.0];
        for i in 1..config.num_output_energies {
            input_energies_ev.push(input_energies_ev[i - 1] + step_ev);
        }

        let mean_electron_flux = read_denton_data_file(&config.electron_data_path)?;
        let mean_ion_flux = read_denton_data_file(&config.ion_data_path)?;
        Ok(Self {
            config,
            spacecraft_state_in_msg: Input::default(),
            earth_state_in_msg: Input::default(),
            sun_state_in_msg: Input::default(),
            flux_out_msg: Output::default(),
            kp_index_counter,
            input_energies_ev,
            log_electron_energies: ELECTRON_ENERGIES_EV.map(f64::ln),
            log_proton_energies: PROTON_ENERGIES_EV.map(f64::ln),
            mean_electron_flux,
            mean_ion_flux,
        })
    }

    /// The unconnected inputs upstream's `Reset` reports with `bskError`.
    pub fn unconnected_inputs(&self) -> Vec<&'static str> {
        [
            (
                self.spacecraft_state_in_msg.is_connected(),
                "spacecraft_state_in_msg",
            ),
            (self.earth_state_in_msg.is_connected(), "earth_state_in_msg"),
            (self.sun_state_in_msg.is_connected(), "sun_state_in_msg"),
        ]
        .into_iter()
        .filter(|(connected, _)| !connected)
        .map(|(_, port)| port)
        .collect()
    }

    /// Upstream `UpdateState`'s flux evaluation.
    fn fluxes(&self) -> PlasmaFluxMsg {
        let r_bn_n_m = self.spacecraft_state_in_msg.read().position_m;
        let r_sn_n_m = self.sun_state_in_msg.read().sun_position_inertial_m;
        let r_en_n_m = self.earth_state_in_msg.read().position_inertial_m;
        let r_be_n_m = r_bn_n_m - r_en_n_m;
        let r_se_n_m = r_sn_n_m - r_en_n_m;

        // The Denton data only holds in the GEO belt (upstream measures the
        // radius in the equatorial plane).
        let r_geo_m = 42000e3;
        let tol_m = 4000e3;
        let equatorial_radius_m = r_be_n_m.xy().norm();
        if equatorial_radius_m < r_geo_m - tol_m
            || equatorial_radius_m > r_geo_m + tol_m
            || r_be_n_m.z.abs() > tol_m
        {
            log::warn!(
                "{}: spacecraft not in GEO regime; the Denton model is not valid outside of GEO",
                self.config.name
            );
        }

        let local_time = local_time_h(r_be_n_m, r_se_n_m);
        let lt_floor = local_time.floor();
        let lt_ceil = local_time.ceil();
        let lt_floor_index = lt_floor as usize % MAX_NUM_LOCAL_TIMES;
        let lt_ceil_index = lt_ceil as usize % MAX_NUM_LOCAL_TIMES;

        let mut out = PlasmaFluxMsg::default();
        for (i, &energy_ev) in self.input_energies_ev.iter().enumerate() {
            let log_energy = energy_ev.ln();
            let flux = |table: &MeanFluxes, log_energies: &[f64; MAX_NUM_ENERGIES]| {
                let (lower, higher) = bracketing_bins(log_energies, log_energy);
                let mean = &table[self.kp_index_counter];
                let log10_flux = bilinear(
                    lt_floor,
                    lt_ceil,
                    log_energies[lower],
                    log_energies[higher],
                    log_energy,
                    local_time,
                    [
                        mean[lower][lt_floor_index],
                        mean[higher][lt_floor_index],
                        mean[lower][lt_ceil_index],
                        mean[higher][lt_ceil_index],
                    ],
                );
                // [cm^-2 s^-1 sr^-1 eV^-1] to [m^-2 s^-1 sr^-1 eV^-1].
                10.0_f64.powf(log10_flux) * 1e4
            };
            out.mean_electron_flux[i] = flux(&self.mean_electron_flux, &self.log_electron_energies);
            out.mean_ion_flux[i] = flux(&self.mean_ion_flux, &self.log_proton_energies);
            out.energies_ev[i] = energy_ev;
        }
        out
    }
}

impl Module for DentonFluxModel {
    fn init(&mut self) {
        for port in self.unconnected_inputs() {
            log::error!("{}: {port} is not connected", self.config.name);
        }
        self.flux_out_msg.write(PlasmaFluxMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let fluxes = self.fluxes();
        self.flux_out_msg.write(fluxes);
    }
}

/// Upstream `Reset`'s Kp conversion: `'3-'` is 8, `'3o'` 9, `'3+'` 10;
/// `None` outside `0o` ... `9o`.
fn kp_index_counter(kp_index: &str) -> Option<usize> {
    let &[main, sub] = kp_index.as_bytes() else {
        return None;
    };
    if !main.is_ascii_digit() {
        return None;
    }
    let main = i64::from(main - b'0');
    let counter = match sub {
        b'-' => 3 * main - 1,
        b'o' => 3 * main,
        b'+' => 3 * main + 1,
        _ => return None,
    };
    usize::try_from(counter)
        .ok()
        .filter(|&counter| counter < MAX_NUM_KPS)
}

/// Upstream `calcLocalTime` \[h\]: the spacecraft's local time in the
/// equatorial plane, 12 h towards the Sun and 0 h opposite.
fn local_time_h(r_be_n_m: Vector3<f64>, r_se_n_m: Vector3<f64>) -> f64 {
    let normalize = |v: Vector2<f64>| v / v.norm();
    let spacecraft_hat = normalize(r_be_n_m.xy());
    let sun_hat = normalize(r_se_n_m.xy());
    let x = spacecraft_hat.dot(&sun_hat);
    let y = sun_hat.x * spacecraft_hat.y - sun_hat.y * spacecraft_hat.x;
    if x <= -1.0 {
        // The tables run from 0 h to 23 h, so 24 h is 0 h.
        0.0
    } else {
        12.00 + (y.atan2(x) / (2. * PI)) * 24.0
    }
}

/// Upstream's neighbor search: the bins around `log_energy`, the first bin
/// above it and the one below (bins 0 and 1 below the first bin). `(0, 0)`
/// above the last bin, which the 100 eV to 40 keV output grid never reaches.
fn bracketing_bins(log_energies: &[f64; MAX_NUM_ENERGIES], log_energy: f64) -> (usize, usize) {
    log_energies
        .iter()
        .position(|&bin| bin > log_energy)
        .map_or((0, 0), |above| {
            let higher = above.max(1);
            (higher - 1, higher)
        })
}

/// Upstream `bilinear`: interpolates `f = [f11, f12, f13, f14]` (lower and
/// higher energy at the floor local time, then at the ceiling local time) at
/// local time `x` and log energy `y`.
///
/// At a whole-hour local time (`x1 == x2`) upstream interpolates in energy
/// between `f11` and `f13`, which are then both the lower energy bin; this
/// port keeps that.
fn bilinear(x1: f64, x2: f64, y1: f64, y2: f64, y: f64, x: f64, f: [f64; 4]) -> f64 {
    let [f11, f12, f13, f14] = f;
    if x1 != x2 {
        let r1 = ((x2 - x) / (x2 - x1)) * f11 + ((x - x1) / (x2 - x1)) * f13;
        let r2 = ((x2 - x) / (x2 - x1)) * f12 + ((x - x1) / (x2 - x1)) * f14;
        ((y2 - y) / (y2 - y1)) * r1 + ((y - y1) / (y2 - y1)) * r2
    } else {
        ((y2 - y) / (y2 - y1)) * f11 + ((y - y1) / (y2 - y1)) * f13
    }
}

/// Upstream `readDentonDataFile`: the mean values of a Denton table, which
/// stores, per Kp index, seven statistics of 40 x 24 values each.
fn read_denton_data_file(path: &Path) -> Result<MeanFluxes, DentonFluxError> {
    let text = std::fs::read_to_string(path).map_err(|source| DentonFluxError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let mut values = text.split_whitespace();
    let mut means = vec![[[0.0; MAX_NUM_LOCAL_TIMES]; MAX_NUM_ENERGIES]; MAX_NUM_KPS];
    // Per Kp index, the seven statistics follow one another; the mean is the
    // first.
    for kp_means in means.iter_mut() {
        for value_type in 0..MAX_NUM_VALUE_TYPES {
            for energy_row in kp_means.iter_mut() {
                for mean in energy_row.iter_mut() {
                    let token = values.next().ok_or_else(|| DentonFluxError::BadTable {
                        path: path.to_path_buf(),
                        message: "too few values".to_string(),
                    })?;
                    if value_type == 0 {
                        *mean = token.parse().map_err(|_| DentonFluxError::BadTable {
                            path: path.to_path_buf(),
                            message: format!("'{token}' is not a number"),
                        })?;
                    }
                }
            }
        }
    }
    Ok(means)
}

/// Port of upstream `test_dentonFluxModel.py`.
#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{DentonFluxError, DentonFluxModel, DentonFluxModelConfig, kp_index_counter};
    use crate::environment::test_assets::asset_or_skip;
    use crate::messages::{Output, PlanetStateMsg, SpacecraftStateMsg, SunEphemerisMsg};
    use crate::{Module, SimulationContext};

    /// Upstream's `Support/FluxData_<Kp counter + 1>_<LT>.txt`: the energies
    /// \[eV\], then the electron and ion log10 fluxes \[cm^-2 s^-1 sr^-1
    /// eV^-1\], from Denton's own code.
    fn support_data(kp_counter_plus_one: usize, local_time: f64) -> [[f64; 6]; 3] {
        const ENERGIES: [f64; 6] = [100.0, 6766.0, 13432.0, 20098.0, 26764.0, 33430.0];
        match (kp_counter_plus_one, format!("{local_time:.2}").as_str()) {
            (1, "0.00") => [
                ENERGIES,
                [4.366478, 2.215579, 1.939717, 1.625582, 1.426048, 1.225854],
                [2.885043, 2.183075, 1.754352, 1.576696, 1.493846, 1.433498],
            ],
            (1, "14.73") => [
                ENERGIES,
                [4.167433, 2.216255, 1.692307, 1.411868, 1.243912, 1.109868],
                [2.937672, 1.914759, 1.792578, 1.474009, 1.374561, 1.330405],
            ],
            (12, "0.00") => [
                ENERGIES,
                [4.446945, 3.680005, 3.275302, 2.673625, 2.256708, 1.792481],
                [2.872504, 2.245852, 2.264615, 2.247567, 2.161325, 2.022533],
            ],
            (12, "14.73") => [
                ENERGIES,
                [3.674995, 2.207153, 1.937881, 1.775273, 1.684263, 1.614265],
                [2.939248, 2.107121, 2.108966, 2.103565, 2.071506, 2.010507],
            ],
            (17, "0.00") => [
                ENERGIES,
                [4.403919, 3.887839, 3.637876, 3.233217, 2.876934, 2.372828],
                [3.075782, 2.217317, 2.219227, 2.202308, 2.185595, 2.158604],
            ],
            (17, "14.73") => [
                ENERGIES,
                [3.908586, 1.882125, 1.792568, 1.718928, 1.672045, 1.620677],
                [3.089667, 2.180461, 2.205745, 2.177123, 2.123957, 2.060799],
            ],
            other => panic!("no support data for {other:?}"),
        }
    }

    fn config(kp_index: &str, electron: PathBuf, ion: PathBuf) -> DentonFluxModelConfig {
        DentonFluxModelConfig {
            name: "dentonFluxModule".to_string(),
            kp_index: kp_index.to_string(),
            num_output_energies: 6,
            electron_data_path: electron,
            ion_data_path: ion,
        }
    }

    #[test]
    fn fluxes_match_denton_for_kp_and_local_time() {
        let Some(electron) = asset_or_skip("denton/model_e_array_all.txt") else {
            return;
        };
        let Some(ion) = asset_or_skip("denton/model_i_array_all.txt") else {
            return;
        };
        let accuracy = 1e2;
        let kps = ["0o", "4-", "5+"];
        let local_times = [0.00, 14.73];
        let z_offsets = [0.0, 3500e3];
        let r_en_ns = [Vector3::zeros(), Vector3::new(400e3, 300e3, -200e3)];
        let cases = [
            (kps[0], local_times[0], z_offsets[0], r_en_ns[0]),
            (kps[1], local_times[1], z_offsets[1], r_en_ns[0]),
            (kps[1], local_times[1], z_offsets[0], r_en_ns[1]),
            (kps[1], local_times[1], z_offsets[1], r_en_ns[1]),
            (kps[1], local_times[0], z_offsets[1], r_en_ns[1]),
            (kps[2], local_times[1], z_offsets[1], r_en_ns[1]),
            (kps[2], local_times[0], z_offsets[1], r_en_ns[1]),
        ];

        let mut models = std::collections::HashMap::new();
        for (kp, local_time, z, r_en_n) in cases {
            let module = models.entry(kp).or_insert_with(|| {
                DentonFluxModel::new(config(kp, electron.clone(), ion.clone()))
                    .expect("valid Denton model")
            });

            let angle =
                local_time * 360. / 24. * std::f64::consts::PI / 180.0 - std::f64::consts::PI;
            let orbit_radius = 42000.0 * 1e3;
            let r_be_n = Vector3::new(orbit_radius * angle.cos(), orbit_radius * angle.sin(), z);
            let r_se_n = Vector3::new(149000000000.0, 0., -2.73 * z);
            let spacecraft = Output::new(SpacecraftStateMsg {
                position_m: r_be_n + r_en_n,
                ..Default::default()
            });
            let sun = Output::new(SunEphemerisMsg {
                sun_position_inertial_m: r_se_n + r_en_n,
                ..Default::default()
            });
            let earth = Output::new(PlanetStateMsg {
                position_inertial_m: r_en_n,
                ..Default::default()
            });
            module.spacecraft_state_in_msg.connect(spacecraft.slot());
            module.sun_state_in_msg.connect(sun.slot());
            module.earth_state_in_msg.connect(earth.slot());
            module.init();
            module.update(&SimulationContext {
                current_sim_nanos: 0,
                current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
            });
            let out = module.flux_out_msg.read();

            let kp_counter = kp_index_counter(kp).unwrap();
            let [energies, electron_log10, ion_log10] = support_data(kp_counter + 1, local_time);
            for i in 0..out.energies_ev.len() {
                let (energy, electron_flux, ion_flux) = if i < 6 {
                    (
                        energies[i],
                        10f64.powf(electron_log10[i]) * 1e4,
                        10f64.powf(ion_log10[i]) * 1e4,
                    )
                } else {
                    (0.0, 0.0, 0.0)
                };
                let case = format!("Kp {kp}, LT {local_time}, z {z}, r_EN {r_en_n:?}, bin {i}");
                assert!(
                    (out.energies_ev[i] - energy).abs() <= accuracy,
                    "energy, {case}"
                );
                assert!(
                    (out.mean_electron_flux[i] - electron_flux).abs() <= accuracy,
                    "electron flux {} vs {electron_flux}, {case}",
                    out.mean_electron_flux[i]
                );
                assert!(
                    (out.mean_ion_flux[i] - ion_flux).abs() <= accuracy,
                    "ion flux {} vs {ion_flux}, {case}",
                    out.mean_ion_flux[i]
                );
            }
        }
    }

    /// Upstream `test_dentonFluxModel_missing_file_format_string`: the
    /// missing path is reported as is, format specifiers included.
    #[test]
    fn a_missing_table_reports_its_path() {
        let missing = std::env::temp_dir().join("missing_%s_%n_flux.txt");
        let error =
            DentonFluxModel::new(config("1o", missing.clone(), missing.clone())).unwrap_err();
        assert!(matches!(error, DentonFluxError::Io { .. }));
        assert!(
            error.to_string().contains(&missing.display().to_string()),
            "{error}"
        );
    }

    #[test]
    fn kp_index_converts_like_upstream() {
        assert_eq!(kp_index_counter("0o"), Some(0));
        assert_eq!(kp_index_counter("1-"), Some(2));
        assert_eq!(kp_index_counter("4-"), Some(11));
        assert_eq!(kp_index_counter("5+"), Some(16));
        assert_eq!(kp_index_counter("9o"), Some(27));
        for invalid in ["0-", "9+", "a+", "3", "3x", "10", ""] {
            assert_eq!(kp_index_counter(invalid), None, "{invalid}");
        }
    }

    #[test]
    fn invalid_configurations_are_rejected() {
        let path = PathBuf::from("unused");
        for count in [0, 51] {
            let mut config = config("1o", path.clone(), path.clone());
            config.num_output_energies = count;
            assert!(matches!(
                DentonFluxModel::new(config),
                Err(DentonFluxError::InvalidNumOutputEnergies(c)) if c == count
            ));
        }
        assert!(matches!(
            DentonFluxModel::new(config("9+", path.clone(), path)),
            Err(DentonFluxError::InvalidKpIndex(_))
        ));
    }

    #[test]
    fn local_time_is_noon_towards_the_sun() {
        let sun = Vector3::new(1.5e11, 0.0, 0.0);
        let at = |angle_deg: f64| {
            let angle = angle_deg.to_radians();
            super::local_time_h(Vector3::new(angle.cos(), angle.sin(), 0.3) * 4.2e7, sun)
        };
        assert!((at(0.0) - 12.0).abs() < 1e-12);
        assert!((at(90.0) - 18.0).abs() < 1e-12);
        assert!((at(-90.0) - 6.0).abs() < 1e-12);
        assert_eq!(at(180.0), 0.0);
    }
}
