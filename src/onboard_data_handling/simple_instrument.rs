//! Constant-rate data generator: port of upstream Basilisk
//! `simulation/onboardDataHandling/instrument/simpleInstrument`.
//!
//! While on, the instrument publishes [`DataNodeBase::node_baud_rate_bps`]
//! under [`DataNodeBase::node_data_name`]; a device command (for instance
//! from the FSW `simpleInstrumentController`) switches it off and on. See
//! [`super::data_node_base`] for the shared behaviour.

use super::data_node_base::{DataNodeBase, DataNodeFields, DataNodeModel};
use crate::messages::{DATA_NAME_SIZE, DataNodeUsageMsg};

/// The model of [`SimpleInstrument`]: upstream `SimpleInstrument`'s
/// `evaluateDataModel`.
#[derive(Clone, Copy, Debug, Default)]
pub struct SimpleInstrumentModel;

/// A data node that generates data at a fixed rate: upstream
/// `SimpleInstrument`.
pub type SimpleInstrument = DataNodeBase<SimpleInstrumentModel>;

impl SimpleInstrument {
    /// An instrument that is on, with a zero baud rate, no data name and
    /// unconnected ports; set [`DataNodeBase::node_baud_rate_bps`] and
    /// [`DataNodeBase::node_data_name`] before running it.
    pub fn new(name: impl Into<String>) -> Self {
        Self::with_model(name, SimpleInstrumentModel)
    }
}

impl DataNodeModel for SimpleInstrumentModel {
    /// # Panics
    ///
    /// If the data name has [`DATA_NAME_SIZE`] characters or more (upstream
    /// `bskError`).
    fn evaluate_data_model(
        &mut self,
        node: DataNodeFields<'_>,
        usage: &mut DataNodeUsageMsg,
        _current_time_s: f64,
    ) {
        assert!(
            node.node_data_name.len() < DATA_NAME_SIZE,
            "SimpleInstrument: nodeDataName is not null-terminated within {DATA_NAME_SIZE} characters."
        );
        usage.baud_rate_bps = node.node_baud_rate_bps;
        usage.data_name.clone_from(node.node_data_name);
    }
}

#[cfg(test)]
mod tests {
    //! Ports both cases of upstream
    //! `simpleInstrument/_UnitTest/test_simpleInstrument.py` with their
    //! tolerance.

    use hifitime::Epoch;

    use super::SimpleInstrument;
    use crate::messages::{DataNodeUsageMsg, DeviceCommandMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;

    /// Upstream `testProcessRate`, `macros.sec2nano(0.5)`.
    const STEP_NANOS: u64 = 500_000_000;
    /// Upstream `accuracy`.
    const ACCURACY: f64 = 1e-16;

    /// Runs `instrument` to 1 s and returns the recorded baud rates.
    fn run(instrument: &mut SimpleInstrument) -> Vec<f64> {
        let mut log = MessageRecorder::<DataNodeUsageMsg>::default();
        instrument.node_data_out_msg.connect_to(&mut log.input_msg);
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("instrument1", instrument, STEP_NANOS, 1);
            sim.add_module("dataLog", &mut log, STEP_NANOS, 0);
            sim.run_for(1_000_000_000);
        }
        log.samples
            .iter()
            .map(|sample| sample.baud_rate_bps)
            .collect()
    }

    /// Upstream `checkDefault`: the instrument publishes its baud rate
    /// while on.
    #[test]
    fn check_default() {
        let mut instrument = SimpleInstrument::new("instrument1");
        instrument.node_baud_rate_bps = 9600.0;
        let baud = run(&mut instrument);
        assert_eq!(baud.len(), 3);
        for value in baud {
            assert!((value - 9600.0).abs() <= ACCURACY);
        }
    }

    /// Upstream `checkStatus`: a zero device command switches it off.
    #[test]
    fn check_status() {
        let mut instrument = SimpleInstrument::new("instrument1");
        instrument.node_baud_rate_bps = 9600.0;
        let command = Output::new(DeviceCommandMsg { device_cmd: 0 });
        command.connect_to(&mut instrument.node_status_in_msg);
        let baud = run(&mut instrument);
        assert_eq!(baud.len(), 3);
        for value in baud {
            assert!(value.abs() <= ACCURACY);
        }
    }

    #[test]
    fn publishes_the_data_name_while_on() {
        let mut instrument = SimpleInstrument::new("instrument1");
        instrument.node_baud_rate_bps = 1200.0;
        instrument.node_data_name = "Instrument 1".to_string();
        run(&mut instrument);
        let usage = instrument.node_data_out_msg.read();
        assert_eq!(usage.data_name, "Instrument 1");
        assert_eq!(usage.baud_rate_bps, 1200.0);
    }

    #[test]
    #[should_panic(expected = "nodeDataName is not null-terminated")]
    fn rejects_a_data_name_that_does_not_fit() {
        let mut instrument = SimpleInstrument::new("instrument1");
        instrument.node_data_name = "x".repeat(128);
        run(&mut instrument);
    }
}
