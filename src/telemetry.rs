use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::marker::PhantomData;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{sync_channel, SyncSender};
use std::thread::JoinHandle;

use serde::Serialize;

use crate::messages::Input;
use crate::{Module, SimulationContext};

/// Samples buffered on the simulation thread before a batch is handed off to the
/// writer thread. Batching amortizes the cross-thread wakeup cost over many
/// samples instead of paying it per sample.
const TELEMETRY_BATCH_SIZE: usize = 1024;
/// Number of batches that may be queued to the writer thread. A full channel
/// applies backpressure (the simulation blocks on `send`) rather than growing
/// memory without bound when the disk can't keep up. A capacity of ≥ 2 also
/// gives the double-buffering effect: the sim fills batch *N+1* while the writer
/// drains batch *N*.
const TELEMETRY_CHANNEL_CAPACITY: usize = 8;

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

/// Owns a writer thread and buffers samples until a full batch can be handed off
/// to it. Serialization and file I/O happen on the writer thread, keeping them
/// off the simulation hot path.
///
/// On drop the final partial batch is shipped, the sender is closed (which ends
/// the writer loop and flushes the `BufWriter`), and the thread is joined, so all
/// buffered data is on disk before the recorder is gone.
#[derive(Debug)]
struct BatchWriter<S> {
    batch: Vec<S>,
    sender: Option<SyncSender<Vec<S>>>,
    handle: Option<JoinHandle<()>>,
}

impl<S> BatchWriter<S> {
    /// Spawn a writer thread that owns `writer` and applies `write_sample` to
    /// every sample of every batch it receives, flushing when the channel closes.
    fn spawn<W>(mut writer: BufWriter<File>, mut write_sample: W) -> Self
    where
        S: Send + 'static,
        W: FnMut(&mut BufWriter<File>, &S) + Send + 'static,
    {
        let (sender, receiver) = sync_channel::<Vec<S>>(TELEMETRY_CHANNEL_CAPACITY);
        let handle = std::thread::spawn(move || {
            while let Ok(batch) = receiver.recv() {
                for sample in &batch {
                    write_sample(&mut writer, sample);
                }
            }
            writer.flush().expect("failed to flush telemetry writer");
        });
        Self {
            batch: Vec::with_capacity(TELEMETRY_BATCH_SIZE),
            sender: Some(sender),
            handle: Some(handle),
        }
    }

    /// Buffer a sample, handing the whole batch off once it is full.
    fn push(&mut self, sample: S) {
        self.batch.push(sample);
        if self.batch.len() >= TELEMETRY_BATCH_SIZE {
            self.send_batch();
        }
    }

    /// Hand the current batch to the writer thread, leaving a fresh buffer in its
    /// place. A no-op when nothing is buffered.
    fn send_batch(&mut self) {
        if self.batch.is_empty() {
            return;
        }
        let batch = std::mem::replace(&mut self.batch, Vec::with_capacity(TELEMETRY_BATCH_SIZE));
        if let Some(sender) = &self.sender {
            sender
                .send(batch)
                .expect("telemetry writer thread disconnected");
        }
    }
}

impl<S> Drop for BatchWriter<S> {
    fn drop(&mut self) {
        self.send_batch();
        // Dropping the sender ends the writer loop, which flushes the BufWriter.
        self.sender = None;
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
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
    writer: Option<BatchWriter<RecordedSample>>,
    message_type: PhantomData<T>,
}

impl<T> Recorder<T> {
    pub fn new(config: RecorderConfig) -> Self {
        Self {
            config,
            input_msg: Input::default(),
            writer: None,
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
    Reference,
}

/// A single CSV sample handed off to the writer thread. The topic is not needed
/// because each recorder owns its own file.
#[derive(Debug)]
struct CsvSample {
    sim_time_nanos: u64,
    fields: Vec<TelemetryField>,
}

#[derive(Debug)]
pub struct CsvRecorder<T> {
    pub config: CsvRecorderConfig,
    pub input_msg: Input<T>,
    format: CsvFormat,
    writer: Option<BatchWriter<CsvSample>>,
    message_type: PhantomData<T>,
}

impl<T> CsvRecorder<T> {
    pub fn new(config: CsvRecorderConfig) -> Self {
        Self {
            config,
            input_msg: Input::default(),
            format: CsvFormat::default(),
            writer: None,
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
        if self.writer.is_some() {
            return;
        }
        let file = open_appending_writer(&self.config.output_path);
        self.writer = Some(BatchWriter::spawn(file, |writer, sample: &RecordedSample| {
            serde_json::to_writer(&mut *writer, sample)
                .expect("failed to serialize telemetry sample");
            writeln!(writer).expect("failed to append telemetry newline");
        }));
    }

    fn update(&mut self, context: &SimulationContext) {
        // The only sim-thread work: read the live input and buffer the sample.
        if let Some(writer) = &mut self.writer {
            writer.push(RecordedSample {
                sim_time_nanos: context.current_sim_nanos,
                topic: self.config.topic.clone(),
                fields: self.input_msg.read().flatten(),
            });
        }
    }
}

impl<T> Module for CsvRecorder<T>
where
    T: Clone + Default + TelemetryMessage + Send + Sync,
{
    fn init(&mut self) {
        if self.writer.is_some() {
            return;
        }
        let file = open_appending_writer(&self.config.output_path);
        let format = self.format;
        // Header/column-ordering state lives in the writer-thread closure, where
        // it is naturally sequential.
        let mut header_paths: Vec<String> = Vec::new();
        let mut header_written = false;
        self.writer = Some(BatchWriter::spawn(file, move |writer, sample: &CsvSample| {
            if !header_written {
                header_paths = sample.fields.iter().map(|field| field.path.clone()).collect();
                write_csv_header(writer, format, &header_paths);
                header_written = true;
            }
            write_csv_row(writer, format, sample.sim_time_nanos, &header_paths, &sample.fields);
        }));
    }

    fn update(&mut self, context: &SimulationContext) {
        // The only sim-thread work: read the live input and buffer the sample.
        if let Some(writer) = &mut self.writer {
            writer.push(CsvSample {
                sim_time_nanos: context.current_sim_nanos,
                fields: self.input_msg.read().flatten(),
            });
        }
    }
}

/// Write the CSV header row: the format-specific time columns followed by one
/// column per telemetry path.
fn write_csv_header(writer: &mut BufWriter<File>, format: CsvFormat, header_paths: &[String]) {
    match format {
        CsvFormat::Default => {
            write!(writer, "sim_time_nanos,sim_time_s").expect("failed to write CSV header prefix");
        }
        CsvFormat::Reference => {
            write!(writer, "time_ns").expect("failed to write CSV header prefix");
        }
    }
    for path in header_paths {
        write!(writer, ",{path}").expect("failed to write CSV header field");
    }
    writeln!(writer).expect("failed to finish CSV row");
}

/// Write one CSV data row, emitting fields in `header_paths` order and filling
/// missing paths with `0.0`.
fn write_csv_row(
    writer: &mut BufWriter<File>,
    format: CsvFormat,
    sim_time_nanos: u64,
    header_paths: &[String],
    fields: &[TelemetryField],
) {
    match format {
        CsvFormat::Default => {
            write!(
                writer,
                "{},{:.9}",
                sim_time_nanos,
                sim_time_nanos as f64 * 1.0e-9
            )
            .expect("failed to write CSV timestamp");
        }
        CsvFormat::Reference => {
            write!(writer, "{sim_time_nanos}").expect("failed to write CSV timestamp");
        }
    }
    for path in header_paths {
        let value = fields
            .iter()
            .find(|field| field.path == *path)
            .map(|field| field.value)
            .unwrap_or(0.0);
        match format {
            CsvFormat::Default => {
                write!(writer, ",{value:.12}").expect("failed to write CSV field value");
            }
            CsvFormat::Reference => {
                write!(writer, ",{value:.18e}").expect("failed to write CSV field value");
            }
        }
    }
    writeln!(writer).expect("failed to finish CSV row");
}

/// Open `path` for appending, creating parent directories as needed, wrapped in
/// a large buffered writer.
fn open_appending_writer(path: &Path) -> BufWriter<File> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).expect("failed to create telemetry output directory");
    }
    let file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
        .expect("failed to open telemetry output file");
    BufWriter::with_capacity(1024 * 1024, file)
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
        // Drop the recorder so its writer thread ships the final partial batch,
        // flushes the BufWriter, and is joined before we read the file back.
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
            Some(CsvFormat::Reference),
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
