//! Per-point mapping data generator: port of upstream Basilisk
//! `simulation/onboardDataHandling/instrument/mappingInstrument`.
//!
//! Each mapping point pairs an [`AccessMsg`] input (typically from a
//! `GroundMapping` or `GroundLocation` point) with a [`DataNodeUsageMsg`]
//! output named after the point. While the point has access the output
//! carries [`MappingInstrument::node_baud_rate_bps`], otherwise zero.
//!
//! Unlike the other instruments this is not a data node: it has no device
//! command, as upstream.

use crate::messages::{AccessMsg, DATA_NAME_SIZE, DataNodeUsageMsg, Input, Output};
use crate::{Module, SimulationContext};

/// Mapping instrument with one data output per accessible point: upstream
/// `MappingInstrument`.
#[derive(Clone, Debug)]
pub struct MappingInstrument {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `nodeBaudRate` \[bit/s\]: data rate of a point with access.
    /// Starts at upstream's `-1`, "unset", which init reports.
    pub node_baud_rate_bps: f64,
    /// Upstream `accessInMsgs`: access of each mapping point, added by
    /// [`Self::add_mapping_point`].
    pub access_in_msgs: Vec<Input<AccessMsg>>,
    /// Upstream `dataNodeOutMsgs`: data of each mapping point, in the order
    /// of `access_in_msgs`.
    pub data_node_out_msgs: Vec<Output<DataNodeUsageMsg>>,
    /// Upstream `mappingPoints`: data name of each point.
    mapping_points: Vec<String>,
}

impl MappingInstrument {
    /// An instrument with no mapping points and the baud rate unset.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            node_baud_rate_bps: -1.0,
            access_in_msgs: Vec::new(),
            data_node_out_msgs: Vec::new(),
            mapping_points: Vec::new(),
        }
    }

    /// Upstream `addMappingPoint`: adds an access input and a data output
    /// named `data_name`, and returns their index in `access_in_msgs` and
    /// `data_node_out_msgs`.
    ///
    /// # Panics
    ///
    /// If `data_name` has [`DATA_NAME_SIZE`] characters or more (upstream
    /// `bskError`).
    pub fn add_mapping_point(&mut self, data_name: impl Into<String>) -> usize {
        let data_name = data_name.into();
        assert!(
            data_name.len() < DATA_NAME_SIZE,
            "MappingInstrument: dataName is {} characters, but DataNodeUsageMsgPayload.dataName \
             supports at most {} characters.",
            data_name.len(),
            DATA_NAME_SIZE - 1
        );
        self.mapping_points.push(data_name);
        self.access_in_msgs.push(Input::default());
        self.data_node_out_msgs.push(Output::default());
        self.data_node_out_msgs.len() - 1
    }

    /// Upstream `Reset`.
    fn reset_state(&self) {
        if self.node_baud_rate_bps < 0.0 {
            log::error!(
                "{}: MappingInstrument.nodeBaudRate is not set to a positive value.",
                self.name
            );
        }
    }
}

impl Module for MappingInstrument {
    fn init(&mut self) {
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    /// Upstream `UpdateState`.
    fn update(&mut self, _context: &SimulationContext) {
        for ((access, output), data_name) in self
            .access_in_msgs
            .iter()
            .zip(&self.data_node_out_msgs)
            .zip(&self.mapping_points)
        {
            output.write(DataNodeUsageMsg {
                data_name: data_name.clone(),
                baud_rate_bps: if access.read().has_access {
                    self.node_baud_rate_bps
                } else {
                    0.0
                },
            });
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports both tests of upstream
    //! `mappingInstrument/_UnitTest/test_mappingInstrument.py`.

    use hifitime::Epoch;

    use super::MappingInstrument;
    use crate::messages::{AccessMsg, DataNodeUsageMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;

    /// Upstream `testProcessRate`, `macros.sec2nano(0.5)`.
    const STEP_NANOS: u64 = 500_000_000;

    /// Upstream `test_mappingInstrument`: an accessible point named "1"
    /// publishes the baud rate, an inaccessible one named "data2" zero.
    #[test]
    fn mapping_instrument() {
        let mut module = MappingInstrument::new("mappingInstrumentTag");
        module.node_baud_rate_bps = 1.0;
        let access_1 = Output::new(AccessMsg {
            has_access: true,
            ..AccessMsg::default()
        });
        let access_2 = Output::new(AccessMsg {
            has_access: false,
            ..AccessMsg::default()
        });
        let point_1 = module.add_mapping_point("1");
        let point_2 = module.add_mapping_point("data2");
        access_1.connect_to(&mut module.access_in_msgs[point_1]);
        access_2.connect_to(&mut module.access_in_msgs[point_2]);

        let mut logs = [
            MessageRecorder::<DataNodeUsageMsg>::default(),
            MessageRecorder::<DataNodeUsageMsg>::default(),
        ];
        for (output, log) in module.data_node_out_msgs.iter().zip(logs.iter_mut()) {
            output.connect_to(&mut log.input_msg);
        }
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("mappingInstrumentTag", &mut module, STEP_NANOS, 1);
            for (index, log) in logs.iter_mut().enumerate() {
                sim.add_module(format!("dataLog{index}"), log, STEP_NANOS, 0);
            }
            sim.run_for(1_000_000_000);
        }

        let baud = |log: &MessageRecorder<DataNodeUsageMsg>| -> Vec<f64> {
            log.samples
                .iter()
                .map(|sample| sample.baud_rate_bps)
                .collect()
        };
        let names = |log: &MessageRecorder<DataNodeUsageMsg>| -> Vec<String> {
            log.samples
                .iter()
                .map(|sample| sample.data_name.clone())
                .collect()
        };
        assert_eq!(baud(&logs[0]), [1.0, 1.0, 1.0]);
        assert_eq!(baud(&logs[1]), [0.0, 0.0, 0.0]);
        assert_eq!(names(&logs[0]), ["1", "1", "1"]);
        assert_eq!(names(&logs[1]), ["data2", "data2", "data2"]);
    }

    /// Upstream `test_mappingInstrument_rejects_long_data_name`.
    #[test]
    #[should_panic(expected = "dataName is 142 characters")]
    fn mapping_instrument_rejects_long_data_name() {
        let mut module = MappingInstrument::new("mappingInstrument");
        module.add_mapping_point(format!("mapping-point-{}", "x".repeat(128)));
    }
}
