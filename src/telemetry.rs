use std::any::Any;
use std::collections::{HashMap, HashSet};
use std::fmt;
use std::fs::{File, OpenOptions};
use std::io::{self, BufWriter, Write};
use std::marker::PhantomData;
use std::path::PathBuf;

use serde::Serialize;

use crate::messages::Input;
use crate::{Module, SimulationContext};

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct TelemetryField {
    pub path: String,
    pub value: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct RecordedSample {
    pub sim_time_nanos: u64,
    pub topic: String,
    pub fields: Vec<TelemetryField>,
}

pub trait TelemetryMessage {
    fn flatten(&self) -> Vec<TelemetryField>;
}

#[derive(Clone, Debug)]
pub struct RecorderConfig {
    pub topic: String,
    pub output_path: PathBuf,
}

#[derive(Debug)]
pub struct Recorder<T> {
    pub config: RecorderConfig,
    pub input_msg: Input<T>,
    file_handler: Option<BufWriter<File>>,
    message_type: PhantomData<T>,
}

impl<T> Recorder<T> {
    pub fn new(config: RecorderConfig) -> Self {
        Self {
            config,
            input_msg: Input::default(),
            file_handler: None,
            message_type: PhantomData,
        }
    }
}

#[derive(Clone, Debug)]
pub struct CsvRecorderConfig {
    pub topic: String,
    pub output_path: PathBuf,
}

/// Output format written by [`CsvRecorder`].
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum CsvFormat {
    /// The original recorder format: nanoseconds, floating-point seconds, and
    /// fixed-point values.
    #[default]
    Default,
    /// Scenario format: one integer `time_ns` column and
    /// 18-digit scientific values.
    Scenario,
}

/// Selects and optionally renames the fields recorded from one message source.
///
/// A string converts to an all-fields configuration, which keeps source
/// registration concise:
///
/// ```
/// use basilisk_rs::telemetry::CsvSourceConfig;
///
/// let source = CsvSourceConfig::all("spacecraft");
/// ```
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct CsvSourceConfig {
    name: String,
    columns: Option<Vec<CsvColumn>>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct CsvColumn {
    source_path: String,
    output_column: String,
}

impl CsvSourceConfig {
    /// Records every flattened field and prefixes it with `name` when nonempty.
    pub fn all(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            columns: None,
        }
    }

    /// Records selected flattened fields in the supplied order.
    ///
    /// Each pair is `(source_path, output_column)`. The output columns are
    /// prefixed by [`Self::with_name`] when a nonempty source name is set.
    pub fn columns<I, S, C>(columns: I) -> Self
    where
        I: IntoIterator<Item = (S, C)>,
        S: Into<String>,
        C: Into<String>,
    {
        Self {
            name: String::new(),
            columns: Some(
                columns
                    .into_iter()
                    .map(|(source_path, output_column)| CsvColumn {
                        source_path: source_path.into(),
                        output_column: output_column.into(),
                    })
                    .collect(),
            ),
        }
    }

    /// Sets the namespace prepended to this source's output columns.
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = name.into();
        self
    }
}

impl From<&str> for CsvSourceConfig {
    fn from(name: &str) -> Self {
        Self::all(name)
    }
}

impl From<String> for CsvSourceConfig {
    fn from(name: String) -> Self {
        Self::all(name)
    }
}

/// A single CSV backend that samples any number of differently typed messages.
///
/// Sources retain typed [`Input`] ports internally and are erased only after
/// registration, so wiring remains compile-time type checked. Source
/// registration order, followed by each source's field order, defines the CSV
/// column order on the first sample. Later samples may return those fields in a
/// different order, but may not add, remove, or rename fields.
/// Duplicate, reserved, or CSV-unsafe column names are rejected. Source
/// registration and format selection are frozen after the first sample. As
/// with any [`Input`], an unconnected source reads `T::default()`; a recorder
/// with no sources writes timestamp-only rows.
///
/// ```no_run
/// use std::path::PathBuf;
///
/// use basilisk_rs::messages::{ArrayMotorTorqueMsg, AtmosphereMsg, Output};
/// use basilisk_rs::simulation::Simulation;
/// use basilisk_rs::telemetry::{CsvRecorder, CsvRecorderConfig};
/// use basilisk_rs::{connect, schedule};
/// use hifitime::Epoch;
///
/// let torque = Output::new(ArrayMotorTorqueMsg::from_active(&[0.25]));
/// let atmosphere = Output::new(AtmosphereMsg::default());
/// let mut recorder = CsvRecorder::new(CsvRecorderConfig {
///     topic: "vehicle".into(),
///     output_path: PathBuf::from("vehicle.csv"),
/// });
/// let mut simulation =
///     Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
/// connect!(&simulation,
///     &torque => recorder.add_source::<ArrayMotorTorqueMsg>("rw"),
///     &atmosphere => recorder.add_source::<AtmosphereMsg>("environment"),
/// );
/// schedule! { simulation,
///     "csv" => &mut recorder, 1_000_000_000, 0;
/// }
/// simulation.run_for(0);
/// ```
pub struct CsvRecorder {
    pub config: CsvRecorderConfig,
    format: CsvFormat,
    sources: Vec<Box<dyn CsvSource>>,
    header_paths: Vec<String>,
    header_written: bool,
    file_handler: Option<BufWriter<File>>,
}

impl fmt::Debug for CsvRecorder {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("CsvRecorder")
            .field("config", &self.config)
            .field("format", &self.format)
            .field("source_count", &self.sources.len())
            .field("header_paths", &self.header_paths)
            .field("header_written", &self.header_written)
            .finish_non_exhaustive()
    }
}

impl CsvRecorder {
    pub fn new(config: CsvRecorderConfig) -> Self {
        Self {
            config,
            format: CsvFormat::default(),
            sources: Vec::new(),
            header_paths: Vec::new(),
            header_written: false,
            file_handler: None,
        }
    }

    /// Select an output format without changing the backwards-compatible
    /// [`CsvRecorderConfig`] struct or default behavior.
    pub fn with_format(mut self, format: CsvFormat) -> Self {
        assert!(
            !self.header_written,
            "cannot change CSV format after recording has started"
        );
        self.format = format;
        self
    }

    /// Adds a typed message source to this recorder and returns its input port.
    ///
    /// The returned port can be passed directly to [`crate::connect!`] or
    /// [`crate::simulation::Simulation::connect`]. Multiple calls may use
    /// unrelated message types; the recorder is still scheduled only once.
    pub fn add_source<T>(&mut self, config: impl Into<CsvSourceConfig>) -> &mut Input<T>
    where
        T: Clone + Default + TelemetryMessage + Send + Sync + 'static,
    {
        assert!(
            !self.header_written,
            "cannot add a CSV source after recording has started"
        );

        self.sources.push(Box::new(MessageCsvSource::<T> {
            config: config.into(),
            input: Input::default(),
        }));
        self.sources
            .last_mut()
            .and_then(|source| source.as_any_mut().downcast_mut::<MessageCsvSource<T>>())
            .map(|source| &mut source.input)
            .expect("CSV source type should match inserted source")
    }

    /// Flushes buffered rows to disk without consuming the recorder.
    pub fn flush(&mut self) -> io::Result<()> {
        if let Some(file_handler) = &mut self.file_handler {
            file_handler.flush()
        } else {
            Ok(())
        }
    }

    fn collect_fields(&self) -> Vec<TelemetryField> {
        let mut fields = Vec::new();
        for source in &self.sources {
            fields.extend(source.collect_fields());
        }
        fields
    }
}

trait CsvSource: Any + Send + Sync {
    fn as_any_mut(&mut self) -> &mut dyn Any;
    fn collect_fields(&self) -> Vec<TelemetryField>;
}

struct MessageCsvSource<T> {
    config: CsvSourceConfig,
    input: Input<T>,
}

impl<T> CsvSource for MessageCsvSource<T>
where
    T: Clone + Default + TelemetryMessage + Send + Sync + 'static,
{
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn collect_fields(&self) -> Vec<TelemetryField> {
        let source_fields = self.input.read().flatten();
        let mut source_paths = HashSet::with_capacity(source_fields.len());
        for field in &source_fields {
            assert!(
                source_paths.insert(field.path.as_str()),
                "CSV source '{}' emitted duplicate field path '{}'",
                self.config.name,
                field.path
            );
        }

        let Some(columns) = &self.config.columns else {
            return source_fields
                .into_iter()
                .map(|field| TelemetryField {
                    path: qualified_field_name(&self.config.name, &field.path),
                    value: field.value,
                })
                .collect();
        };

        let fields_by_path: HashMap<&str, f64> = source_fields
            .iter()
            .map(|field| (field.path.as_str(), field.value))
            .collect();
        columns
            .iter()
            .map(|column| TelemetryField {
                path: qualified_field_name(&self.config.name, &column.output_column),
                value: fields_by_path
                    .get(column.source_path.as_str())
                    .copied()
                    .unwrap_or_else(|| {
                        panic!(
                            "CSV source '{}' has no flattened field '{}'",
                            self.config.name, column.source_path
                        )
                    }),
            })
            .collect()
    }
}

fn qualified_field_name(source: &str, path: &str) -> String {
    if source.is_empty() {
        path.to_string()
    } else {
        format!("{source}_{path}")
    }
}

fn validate_csv_schema(topic: &str, format: CsvFormat, paths: &[String]) {
    let reserved_columns: &[&str] = match format {
        CsvFormat::Default => &["sim_time_nanos", "sim_time_s"],
        CsvFormat::Scenario => &["time_ns"],
    };
    let mut unique_paths = HashSet::with_capacity(paths.len());
    for path in paths {
        assert!(
            !path.is_empty(),
            "CSV topic '{topic}' has an empty column name"
        );
        assert!(
            !path
                .bytes()
                .any(|byte| matches!(byte, b',' | b'\"' | b'\r' | b'\n')),
            "CSV topic '{topic}' has an unsafe column name '{path}'"
        );
        assert!(
            !reserved_columns.contains(&path.as_str()),
            "CSV topic '{topic}' uses reserved column name '{path}'"
        );
        assert!(
            unique_paths.insert(path.as_str()),
            "CSV topic '{topic}' has duplicate column name '{path}'"
        );
    }
}

impl<T> Module for Recorder<T>
where
    T: Clone + Default + TelemetryMessage + Send + Sync,
{
    fn init(&mut self) {
        if let Some(parent) = self.config.output_path.parent() {
            std::fs::create_dir_all(parent).expect("failed to create telemetry output directory");
        }

        if self.file_handler.is_none() {
            let file = OpenOptions::new()
                .create(true)
                .append(true)
                .open(&self.config.output_path)
                .expect("failed to open telemetry output file");

            self.file_handler = Some(BufWriter::with_capacity(1024 * 1024, file));
        }
    }

    fn update(&mut self, context: &SimulationContext) {
        let sample = RecordedSample {
            sim_time_nanos: context.current_sim_nanos,
            topic: self.config.topic.clone(),
            fields: self.input_msg.read().flatten(),
        };

        if let Some(file_handler) = &mut self.file_handler {
            serde_json::to_writer(&mut *file_handler, &sample)
                .expect("failed to serialize telemetry sample");
            writeln!(file_handler).expect("failed to append telemetry newline");
        }
    }
}

impl Module for CsvRecorder {
    fn init(&mut self) {
        if let Some(parent) = self.config.output_path.parent() {
            std::fs::create_dir_all(parent).expect("failed to create CSV output directory");
        }

        if self.file_handler.is_none() {
            let file = OpenOptions::new()
                .create(true)
                .write(true)
                .truncate(true)
                .open(&self.config.output_path)
                .expect("failed to open CSV output file");

            self.file_handler = Some(BufWriter::with_capacity(1024 * 1024, file));
        }
    }

    fn update(&mut self, context: &SimulationContext) {
        assert!(
            self.file_handler.is_some(),
            "CSV recorder must be initialized before update"
        );

        let fields = self.collect_fields();
        let paths: Vec<String> = fields.iter().map(|field| field.path.clone()).collect();
        let write_header = !self.header_written;
        validate_csv_schema(&self.config.topic, self.format, &paths);

        if write_header {
            self.header_paths = paths;
        } else {
            let expected_paths: HashSet<&str> =
                self.header_paths.iter().map(String::as_str).collect();
            let actual_paths: HashSet<&str> = paths.iter().map(String::as_str).collect();
            assert!(
                actual_paths == expected_paths,
                "CSV schema changed while recording topic '{}': expected {:?}, got {:?}",
                self.config.topic,
                self.header_paths,
                paths
            );
        }

        let values_by_path: HashMap<&str, f64> = fields
            .iter()
            .map(|field| (field.path.as_str(), field.value))
            .collect();

        if let Some(file_handler) = &mut self.file_handler {
            if write_header {
                match self.format {
                    CsvFormat::Default => {
                        write!(file_handler, "sim_time_nanos,sim_time_s")
                            .expect("failed to write CSV header prefix");
                    }
                    CsvFormat::Scenario => {
                        write!(file_handler, "time_ns").expect("failed to write CSV header prefix");
                    }
                }
                for path in &self.header_paths {
                    write!(&mut *file_handler, ",{path}")
                        .expect("failed to write CSV header field");
                }
                writeln!(&mut *file_handler).expect("failed to finish CSV header");
                self.header_written = true;
            }

            match self.format {
                CsvFormat::Default => {
                    write!(
                        &mut *file_handler,
                        "{},{:.9}",
                        context.current_sim_nanos,
                        context.current_sim_nanos as f64 * 1.0e-9
                    )
                    .expect("failed to write CSV timestamp");
                }
                CsvFormat::Scenario => {
                    write!(&mut *file_handler, "{}", context.current_sim_nanos)
                        .expect("failed to write CSV timestamp");
                }
            }
            for path in &self.header_paths {
                let value = values_by_path[path.as_str()];
                match self.format {
                    CsvFormat::Default => {
                        write!(&mut *file_handler, ",{value:.12}")
                            .expect("failed to write CSV field value");
                    }
                    CsvFormat::Scenario => {
                        write!(&mut *file_handler, ",{value:.18e}")
                            .expect("failed to write CSV field value");
                    }
                }
            }
            writeln!(&mut *file_handler).expect("failed to finish CSV row");
        }
    }
}

#[cfg(test)]
mod tests {
    use std::panic::{AssertUnwindSafe, catch_unwind};
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    use hifitime::Epoch;

    use crate::messages::{ArrayMotorTorqueMsg, AtmosphereMsg, Input, Output, SunSensorMsg};
    use crate::{Module, SimulationContext};

    use super::{
        CsvFormat, CsvRecorder, CsvRecorderConfig, CsvSource, CsvSourceConfig, MessageCsvSource,
        TelemetryField, TelemetryMessage, validate_csv_schema,
    };

    #[derive(Clone, Debug, Default)]
    struct ReorderedMessage {
        first: f64,
        second: f64,
        reverse: bool,
    }

    impl TelemetryMessage for ReorderedMessage {
        fn flatten(&self) -> Vec<TelemetryField> {
            let first = TelemetryField {
                path: "first".to_string(),
                value: self.first,
            };
            let second = TelemetryField {
                path: "second".to_string(),
                value: self.second,
            };
            if self.reverse {
                vec![second, first]
            } else {
                vec![first, second]
            }
        }
    }

    fn unique_csv_path(label: &str) -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system clock after Unix epoch")
            .as_nanos();
        std::env::temp_dir().join(format!(
            "basilisk_rs_{label}_{}_{}.csv",
            std::process::id(),
            nonce
        ))
    }

    fn context() -> SimulationContext {
        SimulationContext {
            current_sim_nanos: 1_250_000_000,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn record_one_sample(output_path: PathBuf, format: Option<CsvFormat>) -> String {
        let message_out = Output::new(ArrayMotorTorqueMsg::from_active(&[0.25]));
        let recorder = CsvRecorder::new(CsvRecorderConfig {
            topic: "torque".to_string(),
            output_path: output_path.clone(),
        });
        let mut recorder = if let Some(format) = format {
            recorder.with_format(format)
        } else {
            recorder
        };
        recorder
            .add_source::<ArrayMotorTorqueMsg>("")
            .connect(message_out.slot());
        recorder.init();
        recorder.update(&context());
        recorder.flush().expect("flush temporary CSV");
        drop(recorder);

        let csv = std::fs::read_to_string(&output_path).expect("read temporary CSV");
        std::fs::remove_file(output_path).expect("remove temporary CSV");
        csv
    }

    #[test]
    fn default_time_columns_remain_backwards_compatible() {
        let csv = record_one_sample(unique_csv_path("default_time"), None);
        let mut lines = csv.lines();
        assert!(
            lines
                .next()
                .expect("CSV header")
                .starts_with("sim_time_nanos,sim_time_s,")
        );
        assert!(
            lines
                .next()
                .expect("CSV row")
                .starts_with("1250000000,1.250000000,")
        );
    }

    #[test]
    fn scenario_time_mode_writes_one_time_ns_column() {
        let csv = record_one_sample(unique_csv_path("scenario_time"), Some(CsvFormat::Scenario));
        let mut lines = csv.lines();
        assert!(
            lines
                .next()
                .expect("CSV header")
                .starts_with("time_ns,motor_torque_nm.0,")
        );
        assert!(
            lines
                .next()
                .expect("CSV row")
                .starts_with("1250000000,2.500000000000000000e-1,")
        );
    }

    #[test]
    fn a_new_recorder_replaces_an_existing_file() {
        let output_path = unique_csv_path("truncate");
        std::fs::write(&output_path, "stale contents\n").expect("seed stale CSV");

        let csv = record_one_sample(output_path, None);

        assert!(!csv.contains("stale contents"));
        assert_eq!(csv.matches("sim_time_nanos,sim_time_s").count(), 1);
    }

    #[test]
    fn heterogeneous_sources_share_one_ordered_row() {
        let output_path = unique_csv_path("heterogeneous");
        let torque_out = Output::new(ArrayMotorTorqueMsg::from_active(&[0.25]));
        let atmosphere_out = Output::new(AtmosphereMsg {
            neutral_density_kgpm3: 1.5e-12,
            local_temp_k: 900.0,
        });
        let mut recorder = CsvRecorder::new(CsvRecorderConfig {
            topic: "combined".to_string(),
            output_path: output_path.clone(),
        });
        recorder
            .add_source::<ArrayMotorTorqueMsg>(
                CsvSourceConfig::columns([("motor_torque_nm.0", "command")]).with_name("rw"),
            )
            .connect(torque_out.slot());
        recorder
            .add_source::<AtmosphereMsg>(CsvSourceConfig::columns([
                ("neutral_density_kgpm3", "density"),
                ("local_temp_k", "temperature"),
            ]))
            .connect(atmosphere_out.slot());

        recorder.init();
        recorder.update(&context());
        recorder.flush().expect("flush combined CSV");
        drop(recorder);

        let csv = std::fs::read_to_string(&output_path).expect("read combined CSV");
        std::fs::remove_file(output_path).expect("remove combined CSV");
        let mut lines = csv.lines();
        assert_eq!(
            lines.next(),
            Some("sim_time_nanos,sim_time_s,rw_command,density,temperature")
        );
        assert_eq!(
            lines.next(),
            Some("1250000000,1.250000000,0.250000000000,0.000000000002,900.000000000000")
        );
    }

    #[test]
    fn same_message_type_can_be_registered_more_than_once() {
        let output_path = unique_csv_path("same_type");
        let first_out = Output::new(SunSensorMsg {
            sensed_value: 0.25,
            ..Default::default()
        });
        let second_out = Output::new(SunSensorMsg {
            sensed_value: 0.75,
            ..Default::default()
        });
        let mut recorder = CsvRecorder::new(CsvRecorderConfig {
            topic: "sensors".to_string(),
            output_path: output_path.clone(),
        });
        recorder
            .add_source::<SunSensorMsg>(
                CsvSourceConfig::columns([("sensed_value", "signal")]).with_name("css1"),
            )
            .connect(first_out.slot());
        recorder
            .add_source::<SunSensorMsg>(
                CsvSourceConfig::columns([("sensed_value", "signal")]).with_name("css2"),
            )
            .connect(second_out.slot());

        recorder.init();
        recorder.update(&context());
        recorder.flush().expect("flush same-type CSV");
        drop(recorder);

        let csv = std::fs::read_to_string(&output_path).expect("read same-type CSV");
        std::fs::remove_file(output_path).expect("remove same-type CSV");
        let mut lines = csv.lines();
        assert_eq!(
            lines.next(),
            Some("sim_time_nanos,sim_time_s,css1_signal,css2_signal")
        );
        assert_eq!(
            lines.next(),
            Some("1250000000,1.250000000,0.250000000000,0.750000000000")
        );
    }

    #[test]
    fn later_field_reordering_preserves_the_first_header_order() {
        let output_path = unique_csv_path("field_reordering");
        let message_out = Output::new(ReorderedMessage {
            first: 1.0,
            second: 2.0,
            reverse: false,
        });
        let mut recorder = CsvRecorder::new(CsvRecorderConfig {
            topic: "reordered".to_string(),
            output_path: output_path.clone(),
        });
        recorder
            .add_source::<ReorderedMessage>("")
            .connect(message_out.slot());
        recorder.init();
        recorder.update(&context());

        message_out.write(ReorderedMessage {
            first: 3.0,
            second: 4.0,
            reverse: true,
        });
        recorder.update(&context());
        recorder.flush().expect("flush reordered CSV");
        drop(recorder);

        let csv = std::fs::read_to_string(&output_path).expect("read reordered CSV");
        std::fs::remove_file(output_path).expect("remove reordered CSV");
        let lines: Vec<&str> = csv.lines().collect();
        assert_eq!(lines[0], "sim_time_nanos,sim_time_s,first,second");
        assert!(lines[1].ends_with(",1.000000000000,2.000000000000"));
        assert!(lines[2].ends_with(",3.000000000000,4.000000000000"));
    }

    #[test]
    fn update_before_init_does_not_freeze_the_schema() {
        let output_path = unique_csv_path("update_before_init");
        let message_out = Output::new(AtmosphereMsg::default());
        let mut recorder = CsvRecorder::new(CsvRecorderConfig {
            topic: "lifecycle".to_string(),
            output_path: output_path.clone(),
        });
        recorder
            .add_source::<AtmosphereMsg>("")
            .connect(message_out.slot());

        let update_result = catch_unwind(AssertUnwindSafe(|| recorder.update(&context())));
        assert!(update_result.is_err());

        recorder.init();
        recorder.update(&context());
        recorder.flush().expect("flush lifecycle CSV");
        drop(recorder);

        let csv = std::fs::read_to_string(&output_path).expect("read lifecycle CSV");
        std::fs::remove_file(output_path).expect("remove lifecycle CSV");
        assert!(csv.starts_with("sim_time_nanos,sim_time_s,neutral_density_kgpm3,local_temp_k\n"));
    }

    #[test]
    fn sources_cannot_be_added_after_the_first_sample() {
        let output_path = unique_csv_path("late_source");
        let mut recorder = CsvRecorder::new(CsvRecorderConfig {
            topic: "late".to_string(),
            output_path: output_path.clone(),
        });
        recorder.add_source::<AtmosphereMsg>("");
        recorder.init();
        recorder.update(&context());

        let add_result = catch_unwind(AssertUnwindSafe(|| {
            recorder.add_source::<SunSensorMsg>("css");
        }));
        assert!(add_result.is_err());
        drop(recorder);
        std::fs::remove_file(output_path).expect("remove late-source CSV");
    }

    #[test]
    #[should_panic(expected = "duplicate column name 'signal'")]
    fn duplicate_output_columns_are_rejected() {
        validate_csv_schema(
            "duplicates",
            CsvFormat::Default,
            &["signal".to_string(), "signal".to_string()],
        );
    }

    #[test]
    #[should_panic(expected = "has no flattened field 'missing'")]
    fn missing_selected_field_is_rejected() {
        let source = MessageCsvSource::<AtmosphereMsg> {
            config: CsvSourceConfig::columns([("missing", "value")]),
            input: Input::default(),
        };
        source.collect_fields();
    }
}
