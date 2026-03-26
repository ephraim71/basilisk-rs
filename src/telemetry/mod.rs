use std::fs::OpenOptions;
use std::io::Write;
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

#[derive(Clone, Debug)]
pub struct Recorder<T> {
    pub config: RecorderConfig,
    pub input_msg: Input<T>,
    message_type: PhantomData<T>,
}

impl<T> Recorder<T> {
    pub fn new(config: RecorderConfig) -> Self {
        Self {
            config,
            input_msg: Input::default(),
            message_type: PhantomData,
        }
    }
}

#[derive(Clone, Debug)]
pub struct CsvRecorderConfig {
    pub topic: String,
    pub output_path: PathBuf,
}

#[derive(Clone, Debug)]
pub struct CsvRecorder<T> {
    pub config: CsvRecorderConfig,
    pub input_msg: Input<T>,
    header_paths: Vec<String>,
    header_written: bool,
    message_type: PhantomData<T>,
}

impl<T> CsvRecorder<T> {
    pub fn new(config: CsvRecorderConfig) -> Self {
        Self {
            config,
            input_msg: Input::default(),
            header_paths: Vec::new(),
            header_written: false,
            message_type: PhantomData,
        }
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
    }

    fn update(&mut self, context: &SimulationContext) {
        let sample = RecordedSample {
            sim_time_nanos: context.current_sim_nanos,
            topic: self.config.topic.clone(),
            fields: self.input_msg.read().flatten(),
        };

        let mut file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.config.output_path)
            .expect("failed to open telemetry output file");

        serde_json::to_writer(&mut file, &sample).expect("failed to serialize telemetry sample");
        writeln!(file).expect("failed to append telemetry newline");
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
    }

    fn update(&mut self, context: &SimulationContext) {
        let fields = self.input_msg.read().flatten();

        let mut file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.config.output_path)
            .expect("failed to open CSV output file");

        if !self.header_written {
            self.header_paths = fields.iter().map(|field| field.path.clone()).collect();

            write!(file, "sim_time_nanos,sim_time_s").expect("failed to write CSV header prefix");
            for path in &self.header_paths {
                write!(file, ",{path}").expect("failed to write CSV header field");
            }
            writeln!(file).expect("failed to finish CSV header");

            self.header_written = true;
        }

        write!(
            file,
            "{},{:.9}",
            context.current_sim_nanos,
            context.current_sim_nanos as f64 * 1.0e-9
        )
        .expect("failed to write CSV timestamp");
        for path in &self.header_paths {
            let value = fields
                .iter()
                .find(|field| field.path == *path)
                .map(|field| field.value)
                .unwrap_or(0.0);
            write!(file, ",{value:.12}").expect("failed to write CSV field value");
        }
        writeln!(file).expect("failed to finish CSV row");
    }
}
