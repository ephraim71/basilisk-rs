//! Tabulated atmosphere: port of upstream Basilisk
//! `simulation/environment/TabularAtmosphere`, and of the table reader
//! upstream ships with it (`utilities/readAtmTable.py`).
//!
//! The density and temperature are interpolated linearly in altitude from a
//! user-supplied table, sorted by ascending altitude. Outside the table, or
//! outside the reach limits, both are zero (a nonphysical temperature that
//! upstream documents).
//!
//! The altitude is the distance from the planet center less the planet
//! radius (upstream's spherical planet). The planet center is the origin, or
//! the position in `input_planet_msg` when that is connected.
//!
//! # Differences from upstream
//!
//! * As the other atmosphere models here, an instance serves one spacecraft;
//!   upstream's `addSpacecraftToModel` serves several.
//! * The oblate-planet altitude of upstream's atmosphere base
//!   (`setPlanetPolarRadius`) and its state extrapolation to the step midpoint
//!   are not ported, as for the other atmosphere models.
//! * Upstream checks the table at `Reset`; here [`TabularAtmosphere::new`]
//!   does.

use std::fmt;
use std::path::{Path, PathBuf};

use crate::messages::{AtmosphereMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`TabularAtmosphere`].
#[derive(Clone, Debug)]
pub struct TabularAtmosphereConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `planetRadius` \[m\]: radius the altitude is measured from.
    pub planet_radius_m: f64,
    /// Upstream `altList` \[m\]: table altitudes, ascending.
    pub altitudes_m: Vec<f64>,
    /// Upstream `rhoList` \[kg/m^3\]: density at each altitude.
    pub densities_kgpm3: Vec<f64>,
    /// Upstream `tempList` \[K\]: temperature at each altitude.
    pub temperatures_k: Vec<f64>,
    /// Upstream `envMinReach` \[m\]: the atmosphere is zero at or below this
    /// altitude. Upstream's default, -1, keeps everything above -1 m.
    pub env_min_reach_m: f64,
    /// Upstream `envMaxReach` \[m\]: the atmosphere is zero at or above this
    /// altitude; a negative value (upstream's default, -1) turns it off.
    pub env_max_reach_m: f64,
}

impl TabularAtmosphereConfig {
    /// A configuration for `table`, with upstream's default reach limits.
    pub fn from_table(name: impl Into<String>, planet_radius_m: f64, table: AtmTable) -> Self {
        Self {
            name: name.into(),
            planet_radius_m,
            altitudes_m: table.altitudes_m,
            densities_kgpm3: table.densities_kgpm3,
            temperatures_k: table.temperatures_k,
            env_min_reach_m: -1.0,
            env_max_reach_m: -1.0,
        }
    }

    /// Upstream `customReset`'s table checks. Returns a description of the
    /// first violation, or `Ok(())` when the table is usable.
    pub fn validate(&self) -> Result<(), String> {
        let altitudes = self.altitudes_m.len();
        if altitudes != self.densities_kgpm3.len() || altitudes != self.temperatures_k.len() {
            return Err(format!(
                "input arrays not of equal length: {altitudes} altitudes, {} densities, {} \
                 temperatures",
                self.densities_kgpm3.len(),
                self.temperatures_k.len()
            ));
        }
        if altitudes == 0 {
            return Err("no data in altitude list".to_string());
        }
        Ok(())
    }
}

/// Atmosphere interpolated from a table (upstream `TabularAtmosphere`).
#[derive(Clone, Debug)]
pub struct TabularAtmosphere {
    /// Module configuration.
    pub config: TabularAtmosphereConfig,
    /// Upstream `scStateInMsgs`: the spacecraft state (one spacecraft).
    pub input_state_msg: Input<SpacecraftStateMsg>,
    /// Upstream `planetPosInMsg` (optional): the planet position; the planet
    /// is at the origin when unconnected.
    pub input_planet_msg: Input<PlanetStateMsg>,
    /// Upstream `envOutMsgs`: density and temperature at the spacecraft.
    pub output_atmosphere_msg: Output<AtmosphereMsg>,
}

impl TabularAtmosphere {
    /// # Panics
    ///
    /// If the table is empty or its columns differ in length (upstream
    /// `bskError` at reset).
    pub fn new(config: TabularAtmosphereConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid TabularAtmosphereConfig: {message}");
        }
        Self {
            config,
            input_state_msg: Input::default(),
            input_planet_msg: Input::default(),
            output_atmosphere_msg: Output::default(),
        }
    }

    /// Upstream `evaluateAtmosphereModel`: the table value at `altitude_m`.
    ///
    /// The search is upstream's: the first table altitude strictly above
    /// `altitude_m` closes the interval. An altitude equal to the last table
    /// entry therefore finds no interval and stays zero, as upstream.
    fn interpolate(&self, altitude_m: f64) -> AtmosphereMsg {
        let altitudes = &self.config.altitudes_m;
        let mut atmosphere = AtmosphereMsg::default();
        if altitude_m < altitudes[0] || altitude_m > altitudes[altitudes.len() - 1] {
            return atmosphere;
        }
        if let Some(upper) = altitudes.iter().position(|&altitude| altitude > altitude_m) {
            let lower = upper - 1;
            let offset_m = altitude_m - altitudes[lower];
            let span_m = altitudes[upper] - altitudes[lower];
            let rho = &self.config.densities_kgpm3;
            let temp = &self.config.temperatures_k;
            atmosphere.neutral_density_kgpm3 =
                rho[lower] + offset_m * (rho[upper] - rho[lower]) / span_m;
            atmosphere.local_temp_k = temp[lower] + offset_m * (temp[upper] - temp[lower]) / span_m;
        }
        atmosphere
    }

    /// Upstream `updateLocalAtmosphere` for the one spacecraft: zero outside
    /// the reach limits.
    fn local_atmosphere(&self) -> AtmosphereMsg {
        let planet_position_m = if self.input_planet_msg.is_connected() {
            self.input_planet_msg.read().position_inertial_m
        } else {
            nalgebra::Vector3::zeros()
        };
        let r_bp_n_m = self.input_state_msg.read().position_m - planet_position_m;
        let altitude_m = r_bp_n_m.norm() - self.config.planet_radius_m;

        let above_min = altitude_m > self.config.env_min_reach_m;
        let below_max =
            altitude_m < self.config.env_max_reach_m || self.config.env_max_reach_m < 0.0;
        if above_min && below_max {
            self.interpolate(altitude_m)
        } else {
            AtmosphereMsg::default()
        }
    }
}

impl Module for TabularAtmosphere {
    fn init(&mut self) {
        self.output_atmosphere_msg.write(AtmosphereMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.output_atmosphere_msg.write(self.local_atmosphere());
    }
}

/// An atmosphere table: the three columns [`TabularAtmosphere`] takes, sorted
/// by ascending altitude.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AtmTable {
    /// \[m\] altitudes, ascending.
    pub altitudes_m: Vec<f64>,
    /// \[kg/m^3\] density at each altitude.
    pub densities_kgpm3: Vec<f64>,
    /// \[K\] temperature at each altitude.
    pub temperatures_k: Vec<f64>,
}

/// The table layouts upstream's `readAtmTable` reads, named by its `typename`
/// argument.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AtmTableFormat {
    /// `earthgram`: EarthGRAM output, whitespace separated, columns `Hgtkm`
    /// \[km\], `DensMean`, `Tmean` (`supportData/AtmosphereData/EarthGRAMNominal.txt`).
    EarthGram,
    /// `stdatm76`: comma separated with a units row, columns `Altitude` \[m\],
    /// `Density`, `Temperature` (`USStandardAtmosphere1976.csv`).
    StdAtm76,
    /// `msis`: NRLMSISE-00 web output; 29 lines of preamble, then height
    /// \[km\], density \[g/cm^3\], temperature (`NRLMSISE00Nominal.txt`).
    Msis,
    /// `marsgram`: MarsGRAM output, whitespace separated, columns `HgtMOLA`
    /// \[km\], `Denkgm3`, `Temp` (`MarsGRAMNominal.txt`).
    MarsGram,
    /// `venusgram`: comma separated, columns `Height_km`, `Density_kgm3`,
    /// `Temperature_K` (`VenusGRAMNominal.csv`).
    VenusGram,
    /// `uranusgram`: as [`Self::VenusGram`] (`UranusGRAMNominal.csv`).
    UranusGram,
    /// `titangram`: as [`Self::VenusGram`] (`TitanGRAMNominal.csv`).
    TitanGram,
    /// `jupitergram`: as [`Self::VenusGram`] (`JupiterGRAMNominal.csv`).
    JupiterGram,
}

/// Why an atmosphere table could not be read.
#[derive(Debug)]
pub enum AtmTableError {
    /// The file could not be read.
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    /// The header lacks a column the format needs.
    MissingColumn { path: PathBuf, column: String },
    /// A row is too short or holds a value that is not a number.
    BadRow { path: PathBuf, line: usize },
}

impl fmt::Display for AtmTableError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { path, source } => {
                write!(
                    f,
                    "cannot read atmosphere table {}: {source}",
                    path.display()
                )
            }
            Self::MissingColumn { path, column } => write!(
                f,
                "atmosphere table {} has no column '{column}'",
                path.display()
            ),
            Self::BadRow { path, line } => write!(
                f,
                "atmosphere table {} has a malformed row at line {line}",
                path.display()
            ),
        }
    }
}

impl std::error::Error for AtmTableError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            _ => None,
        }
    }
}

/// How the columns of a table are found and scaled to SI.
struct Layout {
    comma_separated: bool,
    /// Lines dropped before the header.
    preamble_lines: usize,
    /// Upstream's `skiprows=[1]`: drop the line right after the header (a
    /// units row in the standard atmosphere; in the GRAM CSV files it is the
    /// first data row, which upstream drops as well).
    skip_line_after_header: bool,
    /// Altitude, density and temperature columns: by header name, or by
    /// position when the header is just column numbers (MSIS).
    columns: Columns,
    altitude_scale: f64,
    density_scale: f64,
}

enum Columns {
    Named([&'static str; 3]),
    Positional,
}

impl AtmTableFormat {
    fn layout(self) -> Layout {
        let named = |altitude, density, temperature, comma_separated, altitude_scale| Layout {
            comma_separated,
            preamble_lines: 0,
            skip_line_after_header: comma_separated,
            columns: Columns::Named([altitude, density, temperature]),
            altitude_scale,
            density_scale: 1.0,
        };
        match self {
            Self::EarthGram => named("Hgtkm", "DensMean", "Tmean", false, 1000.0),
            Self::MarsGram => named("HgtMOLA", "Denkgm3", "Temp", false, 1000.0),
            Self::StdAtm76 => named("Altitude", "Density", "Temperature", true, 1.0),
            Self::VenusGram | Self::UranusGram | Self::TitanGram | Self::JupiterGram => {
                named("Height_km", "Density_kgm3", "Temperature_K", true, 1000.0)
            }
            Self::Msis => Layout {
                comma_separated: false,
                preamble_lines: 29,
                skip_line_after_header: false,
                columns: Columns::Positional,
                altitude_scale: 1000.0,
                // g/cm^3 to kg/m^3, as upstream: `rho / 1000 * 100**3`.
                density_scale: 1.0 / 1000.0 * 100.0_f64.powi(3),
            },
        }
    }
}

/// Upstream `readAtmTable`: reads a tabulated atmosphere and returns it in SI
/// units, sorted by ascending altitude.
pub fn read_atm_table(
    path: impl AsRef<Path>,
    format: AtmTableFormat,
) -> Result<AtmTable, AtmTableError> {
    let path = path.as_ref();
    let text = std::fs::read_to_string(path).map_err(|source| AtmTableError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    parse_atm_table(&text, format, path)
}

fn parse_atm_table(
    text: &str,
    format: AtmTableFormat,
    path: &Path,
) -> Result<AtmTable, AtmTableError> {
    let layout = format.layout();
    let split = |line: &str| -> Vec<String> {
        if layout.comma_separated {
            line.split(',')
                .map(|field| field.trim().to_string())
                .collect()
        } else {
            line.split_whitespace().map(str::to_string).collect()
        }
    };

    // Blank lines are skipped, as pandas does after the preamble.
    let mut lines = text
        .lines()
        .enumerate()
        .skip(layout.preamble_lines)
        .filter(|(_, line)| !line.trim().is_empty());
    let header = lines
        .next()
        .map(|(_, line)| split(line))
        .unwrap_or_default();
    if layout.skip_line_after_header {
        lines.next();
    }

    let indices = match layout.columns {
        Columns::Positional => [0, 1, 2],
        Columns::Named(names) => {
            let mut indices = [0; 3];
            for (index, name) in indices.iter_mut().zip(names) {
                *index = header
                    .iter()
                    .position(|column| column == name)
                    .ok_or_else(|| AtmTableError::MissingColumn {
                        path: path.to_path_buf(),
                        column: name.to_string(),
                    })?;
            }
            indices
        }
    };

    let mut rows = Vec::new();
    for (line_index, line) in lines {
        let fields = split(line);
        let value = |column: usize| -> Option<f64> { fields.get(column)?.parse::<f64>().ok() };
        match (value(indices[0]), value(indices[1]), value(indices[2])) {
            (Some(altitude), Some(density), Some(temperature)) => rows.push((
                altitude * layout.altitude_scale,
                density * layout.density_scale,
                temperature,
            )),
            _ => {
                return Err(AtmTableError::BadRow {
                    path: path.to_path_buf(),
                    line: line_index + 1,
                });
            }
        }
    }

    rows.sort_by(|a, b| a.0.total_cmp(&b.0));
    Ok(AtmTable {
        altitudes_m: rows.iter().map(|row| row.0).collect(),
        densities_kgpm3: rows.iter().map(|row| row.1).collect(),
        temperatures_k: rows.iter().map(|row| row.2).collect(),
    })
}

/// Port of upstream `test_unitTestTabularAtmosphere.py`.
#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{AtmTableFormat, TabularAtmosphere, TabularAtmosphereConfig, read_atm_table};
    use crate::environment::test_assets::asset_or_skip;
    use crate::messages::{Output, PlanetStateMsg, SpacecraftStateMsg};
    use crate::utilities::astro_constants::D2R;
    use crate::utilities::orbital_motion::{ClassicElements, elements_to_state};
    use crate::{Module, SimulationContext};

    const R_EQ_M: f64 = 6_378_136.6;

    fn context(nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The test's truth, `tabAtmoComp`: zero below the table or at/below the
    /// minimum reach, zero above the table or at/above the maximum reach,
    /// otherwise interpolated in the interval closed by the first table
    /// altitude at or above `value`.
    fn tab_atmo_comp(value: f64, x: &[f64], y: &[f64], min_reach: f64, max_reach: f64) -> f64 {
        if value < x[0] || value <= min_reach {
            return 0.0;
        }
        if value > x[x.len() - 1] || value >= max_reach {
            return 0.0;
        }
        for (i, &xi) in x.iter().enumerate() {
            if xi >= value {
                let (x0, y0, y1) = (x[i - 1], y[i - 1], y[i]);
                return y0 + (value - x0) * ((y1 - y0) / (xi - x0));
            }
        }
        unreachable!("value is inside the table")
    }

    fn assert_matches(actual: f64, truth: f64, accuracy: f64, what: &str) {
        let ok = if truth != 0.0 {
            ((actual - truth) / truth).abs() < accuracy
        } else {
            (actual - truth).abs() < accuracy
        };
        assert!(ok, "{what}: got {actual:.12e}, expected {truth:.12e}");
    }

    /// Altitudes exactly on, between, above and below the EarthGRAM table,
    /// with and without each reach limit.
    #[test]
    fn interpolates_the_earthgram_table() {
        let Some(path) = asset_or_skip("atmosphere/EarthGRAMNominal.txt") else {
            return;
        };
        let table = read_atm_table(path, AtmTableFormat::EarthGram).expect("EarthGRAM table");
        let accuracy = 1e-12;

        for altitude_km in [42.0, 33.33333, 10000.0, -10.0] {
            for use_min_reach in [true, false] {
                for use_max_reach in [true, false] {
                    let mut config = TabularAtmosphereConfig::from_table(
                        "tabularAtmosphere",
                        R_EQ_M,
                        table.clone(),
                    );
                    let min_reach = if use_min_reach {
                        config.env_min_reach_m = 50.0 * 1000.0;
                        50.0 * 1000.0
                    } else {
                        -1000.0
                    };
                    let max_reach = if use_max_reach {
                        config.env_max_reach_m = 20.0 * 1000.0;
                        20.0 * 1000.0
                    } else {
                        5000.0 * 1000.0
                    };
                    let mut module = TabularAtmosphere::new(config);

                    let elements = ClassicElements {
                        semi_major_axis: R_EQ_M + altitude_km * 1000.0,
                        eccentricity: 0.0,
                        inclination_rad: 45.0 * D2R,
                        raan_rad: 30.0 * D2R,
                        arg_periapsis_rad: 120.0 * D2R,
                        true_anomaly_rad: 0.0,
                        ..Default::default()
                    };
                    let (r0_n, _) = elements_to_state(0.3986004415e15, &elements);
                    let state = Output::new(SpacecraftStateMsg {
                        position_m: r0_n,
                        ..Default::default()
                    });
                    module.input_state_msg.connect(state.slot());

                    module.init();
                    module.update(&context(0));
                    let out = module.output_atmosphere_msg.read();

                    let case = format!(
                        "altitude {altitude_km} km, min reach {use_min_reach}, max reach \
                         {use_max_reach}"
                    );
                    let altitude_m = altitude_km * 1000.0;
                    let true_density = tab_atmo_comp(
                        altitude_m,
                        &table.altitudes_m,
                        &table.densities_kgpm3,
                        min_reach,
                        max_reach,
                    );
                    let true_temp = tab_atmo_comp(
                        altitude_m,
                        &table.altitudes_m,
                        &table.temperatures_k,
                        min_reach,
                        max_reach,
                    );
                    assert_matches(
                        out.neutral_density_kgpm3,
                        true_density,
                        accuracy,
                        &format!("density, {case}"),
                    );
                    assert_matches(
                        out.local_temp_k,
                        true_temp,
                        accuracy,
                        &format!("temperature, {case}"),
                    );
                }
            }
        }
    }

    fn small_table(name: &str) -> TabularAtmosphereConfig {
        TabularAtmosphereConfig {
            name: name.to_string(),
            planet_radius_m: 1000.0,
            altitudes_m: vec![0.0, 100.0, 200.0],
            densities_kgpm3: vec![1.0, 0.5, 0.1],
            temperatures_k: vec![300.0, 250.0, 200.0],
            env_min_reach_m: -1.0,
            env_max_reach_m: -1.0,
        }
    }

    fn evaluate(module: &mut TabularAtmosphere, position_m: Vector3<f64>) -> (f64, f64) {
        let state = Output::new(SpacecraftStateMsg {
            position_m,
            ..Default::default()
        });
        module.input_state_msg.connect(state.slot());
        module.update(&context(0));
        let out = module.output_atmosphere_msg.read();
        (out.neutral_density_kgpm3, out.local_temp_k)
    }

    #[test]
    fn interpolates_and_zeroes_at_the_table_edges() {
        let mut module = TabularAtmosphere::new(small_table("edges"));
        module.init();

        let (rho, temp) = evaluate(&mut module, Vector3::new(1050.0, 0.0, 0.0));
        assert!((rho - 0.75).abs() < 1e-15 && (temp - 275.0).abs() < 1e-12);
        // The bottom entry is found exactly.
        assert_eq!(
            evaluate(&mut module, Vector3::new(1000.0, 0.0, 0.0)),
            (1.0, 300.0)
        );
        // Upstream's search finds no interval at the top entry: zero.
        assert_eq!(
            evaluate(&mut module, Vector3::new(1200.0, 0.0, 0.0)),
            (0.0, 0.0)
        );
        assert_eq!(
            evaluate(&mut module, Vector3::new(1200.5, 0.0, 0.0)),
            (0.0, 0.0)
        );
        assert_eq!(
            evaluate(&mut module, Vector3::new(999.0, 0.0, 0.0)),
            (0.0, 0.0)
        );
    }

    #[test]
    fn altitude_is_measured_from_the_connected_planet() {
        let mut module = TabularAtmosphere::new(small_table("planet"));
        let planet_position = Vector3::new(5.0e6, -2.0e6, 1.0e6);
        let planet = Output::new(PlanetStateMsg {
            position_inertial_m: planet_position,
            ..Default::default()
        });
        module.input_planet_msg.connect(planet.slot());
        module.init();

        let (rho, _) = evaluate(
            &mut module,
            planet_position + Vector3::new(0.0, 1050.0, 0.0),
        );
        assert!((rho - 0.75).abs() < 1e-12, "got {rho}");
    }

    #[test]
    #[should_panic(expected = "input arrays not of equal length")]
    fn rejects_columns_of_different_lengths() {
        let mut config = small_table("bad");
        config.temperatures_k.pop();
        TabularAtmosphere::new(config);
    }

    #[test]
    #[should_panic(expected = "no data in altitude list")]
    fn rejects_an_empty_table() {
        let mut config = small_table("empty");
        config.altitudes_m.clear();
        config.densities_kgpm3.clear();
        config.temperatures_k.clear();
        TabularAtmosphere::new(config);
    }

    #[test]
    fn reads_every_table_layout() {
        let path = std::path::Path::new("inline");

        let earth = "  Time Hgtkm DensMean Tmean\n 0.0 1.0 0.5 280\n 0.0 0.0 1.2 290\n";
        let table = super::parse_atm_table(earth, AtmTableFormat::EarthGram, path).unwrap();
        assert_eq!(table.altitudes_m, vec![0.0, 1000.0]);
        assert_eq!(table.densities_kgpm3, vec![1.2, 0.5]);
        assert_eq!(table.temperatures_k, vec![290.0, 280.0]);

        let std76 = "Altitude,Temperature,Density\n[m],[Kelvin],[kg/m3]\n-5000,320.65,1.93\n0,288.15,1.225\n";
        let table = super::parse_atm_table(std76, AtmTableFormat::StdAtm76, path).unwrap();
        assert_eq!(table.altitudes_m, vec![-5000.0, 0.0]);
        assert_eq!(table.densities_kgpm3, vec![1.93, 1.225]);

        // Upstream drops the line after the header in the GRAM CSV files too.
        let venus = "ElapsedTime_s,Height_km,Temperature_K,Density_kgm3\n0.0,0.0,735.3,64.79\n0.0,0.5,731.5,63.15\n0.0,1.0,727.6,61.5\n";
        let table = super::parse_atm_table(venus, AtmTableFormat::VenusGram, path).unwrap();
        assert_eq!(table.altitudes_m, vec![500.0, 1000.0]);
        assert_eq!(table.temperatures_k, vec![731.5, 727.6]);

        let mut msis = "preamble\n".repeat(29);
        msis.push_str(
            "\n1         2         3\n    0.0 1.233E-03  286.4\n    0.5 1.168E-03  284.8\n",
        );
        let table = super::parse_atm_table(&msis, AtmTableFormat::Msis, path).unwrap();
        assert_eq!(table.altitudes_m, vec![0.0, 500.0]);
        assert!((table.densities_kgpm3[0] - 1.233).abs() < 1e-12);

        let missing = "Time Hgt DensMean Tmean\n0 0 1 2\n";
        let error = super::parse_atm_table(missing, AtmTableFormat::EarthGram, path).unwrap_err();
        assert!(error.to_string().contains("no column 'Hgtkm'"), "{error}");
    }
}
