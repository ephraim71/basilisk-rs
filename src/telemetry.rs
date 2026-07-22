use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
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
    /// Reference scenario format: one integer `time_ns` column and
    /// 18-digit scientific values.
    BasiliskReference,
}

#[derive(Debug)]
pub struct CsvRecorder<T> {
    pub config: CsvRecorderConfig,
    pub input_msg: Input<T>,
    format: CsvFormat,
    header_paths: Vec<String>,
    header_written: bool,
    file_handler: Option<BufWriter<File>>,
    message_type: PhantomData<T>,
}

impl<T> CsvRecorder<T> {
    pub fn new(config: CsvRecorderConfig) -> Self {
        Self {
            config,
            input_msg: Input::default(),
            format: CsvFormat::default(),
            header_paths: Vec::new(),
            header_written: false,
            file_handler: None,
            message_type: PhantomData,
        }
    }

    /// Select an output format without changing the backwards-compatible
    /// [`CsvRecorderConfig`] struct or default behavior.
    pub fn with_format(mut self, format: CsvFormat) -> Self {
        self.format = format;
        self
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
            serde_json::to_writer(&mut *file_handler, &sample).expect("failed to serialize telemetry sample");
            writeln!(file_handler).expect("failed to append telemetry newline");
        }
    }
}

impl<T> Module for CsvRecorder<T>
where
    T: Clone + Default + TelemetryMessage + Send + Sync,
{
    fn init(&mut self) {
        if let Some(parent) = self.config.output_path.parent() {
            std::fs::create_dir_all(parent).expect("failed to create CSV output directory");
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
        let fields = self.input_msg.read().flatten();

        if let Some(file_handler) = &mut self.file_handler {
            if !self.header_written {
                self.header_paths = fields.iter().map(|field| field.path.clone()).collect();
    
                match self.format {
                    CsvFormat::Default => {
                        write!(file_handler, "sim_time_nanos,sim_time_s")
                            .expect("failed to write CSV header prefix");
                    }
                    CsvFormat::BasiliskReference => {
                        write!(file_handler, "time_ns").expect("failed to write CSV header prefix");
                    }
                }
                for path in &self.header_paths {
                    write!(&mut *file_handler, ",{path}").expect("failed to write CSV header field");
                }
                writeln!(&mut *file_handler).expect("failed to finish CSV row");
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
                CsvFormat::BasiliskReference => {
                    write!(&mut *file_handler, "{}", context.current_sim_nanos)
                        .expect("failed to write CSV timestamp");
                }
            }
            for path in &self.header_paths {
                let value = fields
                    .iter()
                    .find(|field| field.path == *path)
                    .map(|field| field.value)
                    .unwrap_or(0.0);
                match self.format {
                    CsvFormat::Default => {
                        write!(&mut *file_handler, ",{value:.12}").expect("failed to write CSV field value");
                    }
                    CsvFormat::BasiliskReference => {
                        write!(&mut *file_handler, ",{value:.18e}").expect("failed to write CSV field value");
                    }
                }
            }
            writeln!(&mut *file_handler).expect("failed to finish CSV row");
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    use hifitime::Epoch;

    use crate::messages::{ArrayMotorTorqueMsg, Output};
    use crate::{Module, SimulationContext};

    use super::{CsvFormat, CsvRecorder, CsvRecorderConfig};

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
        recorder.input_msg.connect(message_out.slot());
        recorder.init();
        recorder.update(&context());
        // Drop the recorder so its BufWriter flushes to disk before we read the
        // file back; a single sample is far below the 1 MiB buffer capacity.
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
    fn reference_time_mode_writes_one_time_ns_column() {
        let csv = record_one_sample(
            unique_csv_path("reference_time"),
            Some(CsvFormat::BasiliskReference),
        );
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
}
