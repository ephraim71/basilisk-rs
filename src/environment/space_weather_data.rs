//! Space-weather indices for NRLMSISE-00: port of upstream Basilisk
//! `simulation/environment/spaceWeatherData`.
//!
//! The module reads a CelesTrak-style space-weather table
//! (`https://celestrak.org/SpaceData/SW-Last5Years.csv` or `SW-All.csv`; it
//! is not part of upstream `supportData`) and publishes the 23 indices the
//! NRLMSISE-00 / NRLMSIS 2.0 models take, one [`SwDataMsg`] each:
//!
//! | output | index | MSIS channel |
//! |---|---|---|
//! | 0 | daily Ap average (`AP_AVG`) of the current day | `ap_24_0` |
//! | 1 ..= 20 | the 3-hour Ap of the current bin, then stepping back 3 h at a time | `ap_3_0` ... `ap_3_-57` |
//! | 21 | centered 81-day observed F10.7 (`F10.7_OBS_CENTER81`) of the current day | `f107_1944_0` |
//! | 22 | observed F10.7 (`F10.7_OBS`) of the previous day | `f107_24_-24` |
//!
//! # Differences from upstream
//!
//! * The current UTC date and time come from the simulation context's epoch
//!   instead of an epoch message plus the simulation time.
//! * Upstream's `Reset` checks that the table covers the start epoch; `init`
//!   has no epoch here, so the check is in [`Module::reset`] and
//!   [`SpaceWeatherData::check_coverage`].
//! * A failed load returns an error and, as upstream, leaves any table
//!   loaded before in place.

use std::fmt;
use std::path::{Path, PathBuf};

use hifitime::Epoch;

use crate::messages::{Output, SwDataMsg};
use crate::{Module, SimulationContext};

/// Upstream `numSwMessages`: outputs, one per MSIS space-weather channel.
pub const NUM_SW_MESSAGES: usize = 23;
/// Upstream `numApPerDay`: three-hour Ap entries per day.
const NUM_AP_PER_DAY: usize = 8;
/// The columns a table must have.
const REQUIRED_COLUMNS: [&str; 12] = [
    "DATE",
    "AP1",
    "AP2",
    "AP3",
    "AP4",
    "AP5",
    "AP6",
    "AP7",
    "AP8",
    "AP_AVG",
    "F10.7_OBS",
    "F10.7_OBS_CENTER81",
];

/// User-settable parameters of [`SpaceWeatherData`].
#[derive(Clone, Debug)]
pub struct SpaceWeatherDataConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
}

/// One day of the table (upstream `DailySpaceWeather`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct DailySpaceWeather {
    /// Days since 1970-01-01.
    day_number: i64,
    /// `AP1` ... `AP8` \[-\]: the 3-hour Ap values.
    ap_3hr: [f64; NUM_AP_PER_DAY],
    /// `AP_AVG` \[-\]: daily Ap average.
    ap_avg: f64,
    /// `F10.7_OBS` \[sfu\]: observed daily F10.7.
    f107_obs: f64,
    /// `F10.7_OBS_CENTER81` \[sfu\]: centered 81-day F10.7 average.
    f107_obs_center81: f64,
}

/// Why a space-weather table could not be loaded or used (each a `bskError`
/// upstream).
#[derive(Debug)]
pub enum SpaceWeatherError {
    /// The file could not be read.
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    /// The file has no header line.
    EmptyFile(PathBuf),
    /// The header lacks a required column.
    MissingColumn { path: PathBuf, column: &'static str },
    /// No row before the first invalid one could be parsed.
    NoValidRows(PathBuf),
    /// Two rows have the same date.
    DuplicateDate(PathBuf),
    /// The dates are not ascending.
    UnsortedDates(PathBuf),
    /// No table is loaded.
    NoTable,
    /// The table lacks a day the look-up at the start epoch needs.
    StartNotCovered,
}

impl fmt::Display for SpaceWeatherError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { path, source } => write!(
                f,
                "Could not open space-weather file: {} ({source})",
                path.display()
            ),
            Self::EmptyFile(path) => write!(f, "Space-weather file is empty: {}", path.display()),
            Self::MissingColumn { path, column } => write!(
                f,
                "Space-weather file missing required column '{column}': {}",
                path.display()
            ),
            Self::NoValidRows(path) => {
                write!(
                    f,
                    "No valid weather rows were loaded from: {}",
                    path.display()
                )
            }
            Self::DuplicateDate(path) => write!(
                f,
                "Duplicate DATE row found in space-weather table '{}'.",
                path.display()
            ),
            Self::UnsortedDates(path) => write!(
                f,
                "Space-weather DATE rows must be sorted in ascending order: {}",
                path.display()
            ),
            Self::NoTable => write!(
                f,
                "SpaceWeatherData space-weather table is empty. Call load_space_weather_file()."
            ),
            Self::StartNotCovered => write!(
                f,
                "SpaceWeatherData: simulation start date is not covered by the loaded table. \
                 Check the epoch and the date range of the loaded file."
            ),
        }
    }
}

impl std::error::Error for SpaceWeatherError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            _ => None,
        }
    }
}

/// CelesTrak space-weather publisher for MSIS (upstream `SpaceWeatherData`).
#[derive(Clone, Debug)]
pub struct SpaceWeatherData {
    /// Module configuration.
    pub config: SpaceWeatherDataConfig,
    /// Upstream `swDataOutMsgs`: the 23 MSIS space-weather channels, in the
    /// order of the module documentation.
    pub sw_data_out_msgs: [Output<SwDataMsg>; NUM_SW_MESSAGES],
    weather_data: Vec<DailySpaceWeather>,
    loaded_file_name: Option<PathBuf>,
}

impl SpaceWeatherData {
    /// A module with no table; load one with
    /// [`Self::load_space_weather_file`].
    pub fn new(config: SpaceWeatherDataConfig) -> Self {
        Self {
            config,
            sw_data_out_msgs: std::array::from_fn(|_| Output::default()),
            weather_data: Vec::new(),
            loaded_file_name: None,
        }
    }

    /// The table file loaded last.
    pub fn loaded_file_name(&self) -> Option<&Path> {
        self.loaded_file_name.as_deref()
    }

    /// Upstream `loadSpaceWeatherFile`: reads and validates a CelesTrak-style
    /// table.
    ///
    /// Reading stops at the first row that does not parse (such as the
    /// monthly predictions at the end of CelesTrak files, which have no Ap
    /// values). The dates must then be strictly ascending. On any error the
    /// table loaded before, if any, is kept.
    pub fn load_space_weather_file(
        &mut self,
        file_name: impl AsRef<Path>,
    ) -> Result<(), SpaceWeatherError> {
        let path = file_name.as_ref();
        let text = std::fs::read_to_string(path).map_err(|source| SpaceWeatherError::Io {
            path: path.to_path_buf(),
            source,
        })?;
        let mut lines = text.lines();
        let header_columns = split_csv_line(
            lines
                .next()
                .ok_or_else(|| SpaceWeatherError::EmptyFile(path.to_path_buf()))?,
        );
        for column in REQUIRED_COLUMNS {
            if !header_columns.iter().any(|name| name == column) {
                return Err(SpaceWeatherError::MissingColumn {
                    path: path.to_path_buf(),
                    column,
                });
            }
        }

        let mut loaded = Vec::new();
        for line in lines.filter(|line| !line.is_empty()) {
            match parse_weather_line(line, &header_columns) {
                Some(row) => loaded.push(row),
                None => {
                    log::warn!("Stopped reading space-weather table at first invalid row: {line}");
                    break;
                }
            }
        }
        if loaded.is_empty() {
            return Err(SpaceWeatherError::NoValidRows(path.to_path_buf()));
        }
        for pair in loaded.windows(2) {
            if pair[0].day_number == pair[1].day_number {
                return Err(SpaceWeatherError::DuplicateDate(path.to_path_buf()));
            }
            if pair[0].day_number > pair[1].day_number {
                return Err(SpaceWeatherError::UnsortedDates(path.to_path_buf()));
            }
        }

        self.weather_data = loaded;
        self.loaded_file_name = Some(path.to_path_buf());
        Ok(())
    }

    /// Upstream `Reset`'s probe: whether the loaded table holds every day the
    /// look-up at `epoch` (UTC) needs.
    pub fn check_coverage(&self, epoch: Epoch) -> Result<(), SpaceWeatherError> {
        if self.weather_data.is_empty() {
            return Err(SpaceWeatherError::NoTable);
        }
        self.compute_sw_state(epoch)
            .map(|_| ())
            .ok_or(SpaceWeatherError::StartNotCovered)
    }

    /// The table row of `day_number`.
    fn day_entry(&self, day_number: i64) -> Option<&DailySpaceWeather> {
        self.weather_data
            .binary_search_by_key(&day_number, |row| row.day_number)
            .ok()
            .map(|index| &self.weather_data[index])
    }

    /// Upstream `computeSwState`: the 23 channels at `epoch`, or `None` when a
    /// day they need is missing from the table.
    fn compute_sw_state(&self, epoch: Epoch) -> Option<[f64; NUM_SW_MESSAGES]> {
        let (current_day, seconds_of_day) = utc_day_and_seconds(epoch);
        let current_ap_bin = (seconds_of_day / 3600.0 / 3.0).floor() as i64;

        let day0 = self.day_entry(current_day)?;
        let day_minus1 = self.day_entry(current_day - 1)?;
        let day_minus2 = self.day_entry(current_day - 2)?;
        // The 20-bin look-back only reaches a third day back before 09:00.
        let day_minus3 = if current_ap_bin < 3 {
            *self.day_entry(current_day - 3)?
        } else {
            DailySpaceWeather::default()
        };
        let ap_by_day = [
            day0.ap_3hr,
            day_minus1.ap_3hr,
            day_minus2.ap_3hr,
            day_minus3.ap_3hr,
        ];

        let mut state = [0.0; NUM_SW_MESSAGES];
        state[0] = day0.ap_avg;
        for ap_index in 0..20 {
            let mut bin = current_ap_bin - ap_index;
            let mut day_offset = 0;
            while bin < 0 {
                bin += NUM_AP_PER_DAY as i64;
                day_offset += 1;
            }
            if day_offset > 3 || bin >= NUM_AP_PER_DAY as i64 {
                return None;
            }
            state[1 + ap_index as usize] = ap_by_day[day_offset][bin as usize];
        }
        state[21] = day0.f107_obs_center81;
        state[22] = day_minus1.f107_obs;
        Some(state)
    }

    fn publish(&self, values: [f64; NUM_SW_MESSAGES]) {
        for (output, data_value) in self.sw_data_out_msgs.iter().zip(values) {
            output.write(SwDataMsg { data_value });
        }
    }
}

impl Module for SpaceWeatherData {
    fn init(&mut self) {
        if self.weather_data.is_empty() {
            log::error!("{}: {}", self.config.name, SpaceWeatherError::NoTable);
        }
        // Upstream `publishZeroState`.
        self.publish([0.0; NUM_SW_MESSAGES]);
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.init();
        if let Err(error) = self.check_coverage(context.current_epoch) {
            log::error!("{}: {error}", self.config.name);
        }
    }

    fn update(&mut self, context: &SimulationContext) {
        match self.compute_sw_state(context.current_epoch) {
            Some(state) => self.publish(state),
            // Upstream keeps the last outputs.
            None => log::warn!(
                "{}: failed to retrieve a state; publishing the last available data. This can \
                 happen if the date is not in the file or if the file is empty.",
                self.config.name
            ),
        }
    }
}

/// The UTC day number (days since 1970-01-01) and seconds of that day.
fn utc_day_and_seconds(epoch: Epoch) -> (i64, f64) {
    let (year, month, day, hours, minutes, seconds, nanos) = epoch.to_gregorian_utc();
    let day_number = civil_to_unix_day_number(year, u32::from(month), u32::from(day));
    let seconds_of_day = f64::from(hours) * 3600.0
        + f64::from(minutes) * 60.0
        + f64::from(seconds)
        + nanos as f64 * 1e-9;
    (day_number, seconds_of_day)
}

/// Upstream `civilToUnixDayNumber`: days from 1970-01-01 to a proleptic
/// Gregorian date (negative before 1970).
fn civil_to_unix_day_number(year: i32, month: u32, day: u32) -> i64 {
    let year = i64::from(year) - i64::from(month <= 2);
    let era = if year >= 0 { year } else { year - 399 } / 400;
    let year_of_era = year - era * 400;
    let month = i64::from(month);
    let day_of_year = (153 * (month + if month > 2 { -3 } else { 9 }) + 2) / 5 + i64::from(day) - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
}

fn is_leap_year(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || year % 400 == 0
}

fn days_in_month(year: i32, month: u32) -> u32 {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if is_leap_year(year) => 29,
        2 => 28,
        _ => 0,
    }
}

/// Upstream `splitCsvLine`: comma-separated fields with spaces, tabs and
/// carriage returns trimmed.
fn split_csv_line(line: &str) -> Vec<String> {
    line.split(',')
        .map(|token| token.trim_matches([' ', '\t', '\r']).to_string())
        .collect()
}

/// Upstream `parseDouble`: a finite number with nothing after it.
fn parse_double(token: &str) -> Option<f64> {
    token.parse::<f64>().ok().filter(|value| value.is_finite())
}

/// Upstream `parseWeatherLine`: one table row, or `None` if a field is
/// missing, empty or not a number, or the date is not a calendar date.
fn parse_weather_line(line: &str, header_columns: &[String]) -> Option<DailySpaceWeather> {
    let values = split_csv_line(line);
    let field = |name: &str| -> Option<&str> {
        let index = header_columns.iter().position(|column| column == name)?;
        values.get(index).map(String::as_str)
    };

    let mut date = field("DATE")?.split('-');
    let year: i32 = date.next()?.parse().ok()?;
    let month: u32 = date.next()?.parse().ok()?;
    let day: u32 = date.next()?.parse().ok()?;
    if date.next().is_some() || !(1..=12).contains(&month) {
        return None;
    }
    if day < 1 || day > days_in_month(year, month) {
        return None;
    }

    let mut ap_3hr = [0.0; NUM_AP_PER_DAY];
    for (index, ap) in ap_3hr.iter_mut().enumerate() {
        *ap = parse_double(field(&format!("AP{}", index + 1))?)?;
    }
    Some(DailySpaceWeather {
        day_number: civil_to_unix_day_number(year, month, day),
        ap_3hr,
        ap_avg: parse_double(field("AP_AVG")?)?,
        f107_obs: parse_double(field("F10.7_OBS")?)?,
        f107_obs_center81: parse_double(field("F10.7_OBS_CENTER81")?)?,
    })
}

/// Port of upstream `test_spaceWeatherData.py`.
#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use hifitime::{Duration, Epoch};

    use super::{NUM_SW_MESSAGES, SpaceWeatherData, SpaceWeatherDataConfig, SpaceWeatherError};
    use crate::{Module, SimulationContext};

    const CELESTRAK_CSV: &str = "\
DATE,BSRN,ND,KP1,KP2,KP3,KP4,KP5,KP6,KP7,KP8,KP_SUM,AP1,AP2,AP3,AP4,AP5,AP6,AP7,AP8,AP_AVG,CP,C9,ISN,F10.7_OBS,F10.7_ADJ,F10.7_DATA_TYPE,F10.7_OBS_CENTER81,F10.7_OBS_LAST81,F10.7_ADJ_CENTER81,F10.7_ADJ_LAST81
2026-02-28,2626,4,20,27,13,13,20,17,20,23,153,7,12,5,5,7,6,7,9,7,0.4,2,58,140.7,138.1,OBS,141.7,144.2,139.2,139.9
2026-03-01,2626,5,27,27,23,20,13,7,17,13,147,12,12,9,7,5,3,6,5,7,0.4,2,80,147.0,144.4,OBS,141.1,143.9,138.6,139.7
2026-03-02,2626,6,13,7,10,3,17,10,7,13,80,5,3,4,2,6,4,3,5,4,0.1,0,81,147.6,145.0,OBS,140.3,143.9,138.0,139.7
2026-03-03,2626,7,7,27,20,23,20,33,33,50,213,3,12,7,9,7,18,18,48,15,0.9,4,83,143.5,141.1,OBS,139.5,144.0,137.2,139.8
2026-03-04,2626,8,23,30,17,20,7,7,13,10,127,9,15,6,7,3,3,5,4,6,0.3,1,68,141.2,138.8,OBS,138.5,144.3,136.3,140.1";

    const EXPECTED: [f64; NUM_SW_MESSAGES] = [
        6.0, 4.0, 5.0, 3.0, 3.0, 7.0, 6.0, 15.0, 9.0, 48.0, 18.0, 18.0, 7.0, 9.0, 7.0, 12.0, 3.0,
        5.0, 3.0, 4.0, 6.0, 138.5, 143.5,
    ];

    const MINIMAL_HEADER: &str =
        "DATE,AP1,AP2,AP3,AP4,AP5,AP6,AP7,AP8,AP_AVG,F10.7_OBS,F10.7_OBS_CENTER81";

    /// A table file in the temporary directory, removed when dropped (the
    /// tests' `tempfile.TemporaryDirectory`).
    struct TempTable(PathBuf);

    impl TempTable {
        fn new(name: &str, contents: &str) -> Self {
            let path = std::env::temp_dir().join(format!(
                "bsk_rs_space_weather_{}_{name}.csv",
                std::process::id()
            ));
            std::fs::write(&path, contents).expect("temporary table");
            Self(path)
        }
    }

    impl Drop for TempTable {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }

    fn module() -> SpaceWeatherData {
        SpaceWeatherData::new(SpaceWeatherDataConfig {
            name: "spaceWeatherData".to_string(),
        })
    }

    fn load(name: &str, contents: &str) -> Result<SpaceWeatherData, SpaceWeatherError> {
        let table = TempTable::new(name, contents);
        let mut module = module();
        module.load_space_weather_file(&table.0)?;
        Ok(module)
    }

    fn epoch(year: i32, month: u8, day: u8, hour: u8, minute: u8) -> Epoch {
        Epoch::from_gregorian_utc(year, month, day, hour, minute, 0, 0)
    }

    fn context(start: Epoch, sim_seconds: f64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: (sim_seconds * 1e9) as u64,
            current_epoch: start + Duration::from_seconds(sim_seconds),
        }
    }

    fn outputs(module: &SpaceWeatherData) -> Vec<f64> {
        module
            .sw_data_out_msgs
            .iter()
            .map(|output| output.read().data_value)
            .collect()
    }

    fn assert_outputs(module: &SpaceWeatherData, expected: &[f64; NUM_SW_MESSAGES]) {
        for (index, (actual, expected)) in outputs(module).iter().zip(expected).enumerate() {
            assert!(
                (actual - expected).abs() < 1e-12,
                "channel {index}: got {actual}, expected {expected}"
            );
        }
    }

    /// Initializes at `start` and runs the first update there, which upstream
    /// records as `dataValue[0]`.
    fn first_update(module: &mut SpaceWeatherData, start: Epoch) {
        let context = context(start, 0.0);
        module.reset(&context);
        module.update(&context);
    }

    fn expect_error(name: &str, contents: &str, message: &str) {
        let error = load(name, contents).expect_err("table should be rejected");
        assert!(error.to_string().contains(message), "{error}");
    }

    #[test]
    fn celestrak_example_columns() {
        let mut module = load("celestrak", CELESTRAK_CSV).unwrap();
        first_update(&mut module, epoch(2026, 3, 4, 21, 48));
        assert_outputs(&module, &EXPECTED);
    }

    #[test]
    fn reading_stops_at_the_first_invalid_row() {
        let csv = format!(
            "{CELESTRAK_CSV}\n\
             2038-11-01,2797,16,,,,,,,,,,,,,,,,,,,,,64,95.7,94.3,PRM,96.0,95.4,94.7,96.1\n\
             2038-12-01,2798,19,,,,,,,,,,,,,,,,,,,,,62,95.9,93.3,PRM,96.2,95.9,93.8,95.1"
        );
        let mut module = load("prm", &csv).unwrap();
        first_update(&mut module, epoch(2026, 3, 4, 21, 48));
        assert_outputs(&module, &EXPECTED);
    }

    #[test]
    fn a_missing_required_column_is_rejected() {
        expect_error(
            "missing_column",
            "DATE,AP1,AP2,AP3,AP4,AP5,AP6,AP7,AP8,AP_AVG,F10.7_OBS\n\
             2025-01-04,401,402,403,404,405,406,407,408,450,73\n\
             2025-01-05,301,302,303,304,305,306,307,308,350,72\n\
             2025-01-06,201,202,203,204,205,206,207,208,250,71\n\
             2025-01-07,101,102,103,104,105,106,107,108,150,70",
            "Space-weather file missing required column",
        );
    }

    #[test]
    fn duplicate_dates_are_rejected() {
        expect_error(
            "duplicate",
            &format!(
                "{MINIMAL_HEADER}\n\
                 2025-01-04,401,402,403,404,405,406,407,408,450,73,83\n\
                 2025-01-04,301,302,303,304,305,306,307,308,350,72,82\n\
                 2025-01-06,201,202,203,204,205,206,207,208,250,71,81\n\
                 2025-01-07,101,102,103,104,105,106,107,108,150,70,80"
            ),
            "Duplicate DATE row found in space-weather table",
        );
    }

    #[test]
    fn unsorted_dates_are_rejected() {
        expect_error(
            "unsorted",
            &format!(
                "{MINIMAL_HEADER}\n\
                 2025-01-05,301,302,303,304,305,306,307,308,350,72,82\n\
                 2025-01-04,401,402,403,404,405,406,407,408,450,73,83\n\
                 2025-01-06,201,202,203,204,205,206,207,208,250,71,81\n\
                 2025-01-07,101,102,103,104,105,106,107,108,150,70,80"
            ),
            "Space-weather DATE rows must be sorted in ascending order",
        );
    }

    /// Upstream `test_reset_error_when_epoch_before_table`.
    #[test]
    fn a_start_before_the_table_is_not_covered() {
        let module = load("before_table", CELESTRAK_CSV).unwrap();
        let error = module.check_coverage(epoch(2026, 2, 27, 0, 0)).unwrap_err();
        assert!(
            error
                .to_string()
                .contains("simulation start date is not covered"),
            "{error}"
        );
        assert!(module.check_coverage(epoch(2026, 3, 4, 21, 48)).is_ok());
        assert!(matches!(
            super::SpaceWeatherData::new(super::SpaceWeatherDataConfig {
                name: "empty".to_string()
            })
            .check_coverage(epoch(2026, 3, 4, 0, 0)),
            Err(SpaceWeatherError::NoTable)
        ));
    }

    /// Upstream `test_stale_output_retained_on_missing_day`.
    #[test]
    fn outputs_are_kept_when_a_day_is_missing() {
        let mut module = load("stale", CELESTRAK_CSV).unwrap();
        let start = epoch(2026, 3, 4, 0, 0);
        first_update(&mut module, start);
        module.update(&context(start, 1.0));
        assert!((outputs(&module)[0] - 6.0).abs() < 1e-12);
        assert!((outputs(&module)[21] - 138.5).abs() < 1e-12);

        // 2026-03-05 is not in the table.
        module.update(&context(start, 25.0 * 3600.0 + 1.0));
        assert!((outputs(&module)[0] - 6.0).abs() < 1e-12);
        assert!((outputs(&module)[21] - 138.5).abs() < 1e-12);
    }

    /// Upstream `test_space_weather_data_epoch_update`: hourly updates from
    /// 2026-03-03 18:48 for a day and 3 hours.
    #[test]
    fn the_epoch_advances_with_the_simulation() {
        let mut module = load("epoch_update", CELESTRAK_CSV).unwrap();
        let start = epoch(2026, 3, 3, 18, 48);
        first_update(&mut module, start);
        for hour in 1..=27 {
            module.update(&context(start, f64::from(hour) * 3600.0));
        }
        assert_outputs(&module, &EXPECTED);
    }

    #[test]
    fn a_date_with_trailing_characters_is_rejected() {
        expect_error(
            "date_junk",
            &format!("{MINIMAL_HEADER}\n2026-03-04junk,9,15,6,7,3,3,5,4,6,141.2,138.5"),
            "No valid weather rows were loaded",
        );
    }

    #[test]
    fn a_number_with_trailing_characters_is_rejected() {
        expect_error(
            "number_junk",
            &format!("{MINIMAL_HEADER}\n2026-03-04,9abc,15,6,7,3,3,5,4,6,141.2,138.5"),
            "No valid weather rows were loaded",
        );
    }

    /// Upstream's calendar-date and non-finite value checks, each a table
    /// whose only row must be rejected.
    #[test]
    fn invalid_single_rows_are_rejected() {
        for (name, row) in [
            ("april_31", "2026-04-31,9,15,6,7,3,3,5,4,6,141.2,138.5"),
            ("november_31", "2026-11-31,9,15,6,7,3,3,5,4,6,141.2,138.5"),
            ("feb_30", "2024-02-30,9,15,6,7,3,3,5,4,6,141.2,138.5"),
            (
                "feb_29_non_leap",
                "2025-02-29,9,15,6,7,3,3,5,4,6,141.2,138.5",
            ),
            (
                "feb_29_century",
                "2100-02-29,9,15,6,7,3,3,5,4,6,141.2,138.5",
            ),
            ("day_zero", "2026-03-00,9,15,6,7,3,3,5,4,6,141.2,138.5"),
            ("month_zero", "2026-00-15,9,15,6,7,3,3,5,4,6,141.2,138.5"),
            ("month_13", "2026-13-01,9,15,6,7,3,3,5,4,6,141.2,138.5"),
            ("nan_ap", "2026-03-04,nan,15,6,7,3,3,5,4,6,141.2,138.5"),
            ("inf_f107", "2026-03-04,9,15,6,7,3,3,5,4,6,inf,138.5"),
            (
                "neg_inf_ap_avg",
                "2026-03-04,9,15,6,7,3,3,5,4,-inf,141.2,138.5",
            ),
        ] {
            let error = load(name, &format!("{MINIMAL_HEADER}\n{row}"))
                .expect_err(&format!("{name} should be rejected"));
            assert!(
                error
                    .to_string()
                    .contains("No valid weather rows were loaded"),
                "{name}: {error}"
            );
        }
    }

    #[test]
    fn a_leap_day_is_accepted() {
        let csv = format!(
            "{MINIMAL_HEADER}\n\
             2024-02-28,1,2,3,4,5,6,7,8,4,130.0,128.0\n\
             2024-02-29,9,10,11,12,13,14,15,16,12,131.0,129.0\n\
             2024-03-01,17,18,19,20,21,22,23,24,20,132.0,130.0\n\
             2024-03-02,25,26,27,28,29,30,31,32,28,133.0,131.0"
        );
        let mut module = load("leap", &csv).unwrap();
        first_update(&mut module, epoch(2024, 3, 2, 12, 0));
        assert!((outputs(&module)[0] - 28.0).abs() < 1e-12);

        let quad_century = format!(
            "{MINIMAL_HEADER}\n\
             2000-02-27,1,2,3,4,5,6,7,8,4,130.0,128.0\n\
             2000-02-28,9,10,11,12,13,14,15,16,12,131.0,129.0\n\
             2000-02-29,17,18,19,20,21,22,23,24,20,132.0,130.0\n\
             2000-03-01,25,26,27,28,29,30,31,32,28,133.0,131.0"
        );
        load("quad_century", &quad_century).expect("2000-02-29 is a date");
    }

    /// Upstream `test_failed_reload_preserves_previous_table`.
    #[test]
    fn a_failed_reload_keeps_the_previous_table() {
        let good = TempTable::new("good", CELESTRAK_CSV);
        let bad = TempTable::new(
            "bad",
            "DATE,AP1,AP2,AP3,AP4,AP5,AP6,AP7,AP8,AP_AVG,F10.7_OBS\n\
             2026-03-04,9,15,6,7,3,3,5,4,6,141.2",
        );
        let mut module = module();
        module.load_space_weather_file(&good.0).unwrap();
        assert!(module.load_space_weather_file(&bad.0).is_err());
        assert_eq!(module.loaded_file_name(), Some(good.0.as_path()));

        first_update(&mut module, epoch(2026, 3, 4, 21, 48));
        assert!((outputs(&module)[0] - 6.0).abs() < 1e-12);
    }

    #[test]
    fn day_numbers_match_the_unix_calendar() {
        assert_eq!(super::civil_to_unix_day_number(1970, 1, 1), 0);
        assert_eq!(super::civil_to_unix_day_number(2000, 3, 1), 11_017);
        assert_eq!(super::civil_to_unix_day_number(1969, 12, 31), -1);
        assert_eq!(super::civil_to_unix_day_number(1600, 2, 29), -135_081);
    }

    /// Before 09:00 the look-back reaches a third day; a table one day short
    /// of it cannot serve that time but can serve later ones.
    #[test]
    fn early_bins_need_a_third_previous_day() {
        let csv = format!(
            "{MINIMAL_HEADER}\n\
             2026-03-02,1,2,3,4,5,6,7,8,4,130.0,128.0\n\
             2026-03-03,9,10,11,12,13,14,15,16,12,131.0,129.0\n\
             2026-03-04,17,18,19,20,21,22,23,24,20,132.0,130.0"
        );
        let module = load("third_day", &csv).unwrap();
        assert!(module.check_coverage(epoch(2026, 3, 4, 8, 59)).is_err());
        let state = module
            .compute_sw_state(epoch(2026, 3, 4, 9, 0))
            .expect("bin 3 needs two previous days");
        // Bin 3 of 2026-03-04, then back through 2026-03-03 and 2026-03-02.
        assert_eq!(&state[1..5], &[20.0, 19.0, 18.0, 17.0]);
        assert_eq!(
            &state[5..13],
            &[16.0, 15.0, 14.0, 13.0, 12.0, 11.0, 10.0, 9.0]
        );
        assert_eq!(&state[13..21], &[8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0]);
    }
}
