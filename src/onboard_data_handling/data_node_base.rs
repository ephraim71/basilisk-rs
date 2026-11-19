//! Data-node plumbing shared by every data producer and sink: port of
//! upstream Basilisk
//! `simulation/onboardDataHandling/_GeneralModuleFiles/dataNodeBase`.
//!
//! [`DataNodeBase`] is the module. Each update it reads the optional device
//! command, which sets [`DataNodeBase::data_status`]; while the status is on
//! (above zero) it asks its [`DataNodeModel`] for the data name and baud rate
//! to publish, and while it is off it publishes a zeroed
//! [`DataNodeUsageMsg`]. A new data node is a type implementing
//! [`DataNodeModel`], used as `DataNodeBase<NewModel>`; see
//! [`super::simple_instrument`].
//!
//! The usage message is kept between updates, as upstream's `nodeDataMsg`
//! is, so a model that leaves the data name alone republishes the previous
//! one.
//!
//! # Differences from upstream
//!
//! * No message headers: the device command is used whenever it is
//!   connected ("connected ⇒ written at the current update"). Upstream's
//!   read can then never fail, and none of the ported models' custom reads
//!   does, so the branch that zeroes the output on a failed read is kept but
//!   unreachable in practice.
//! * The base-class members a model reads or changes reach it as a
//!   [`DataNodeFields`] view rather than through inheritance.

use crate::messages::{DataNodeUsageMsg, DeviceCommandMsg, Input, Output};
use crate::{Module, SimulationContext};

/// The base-class members a [`DataNodeModel`] hook may read or change.
#[derive(Debug)]
pub struct DataNodeFields<'a> {
    /// Upstream `nodeBaudRate` \[bit/s\].
    pub node_baud_rate_bps: f64,
    /// Upstream `nodeDataName`.
    pub node_data_name: &'a mut String,
    /// Upstream `dataStatus`.
    pub data_status: u64,
}

/// What a concrete data node adds to [`DataNodeBase`]: upstream's pure
/// virtual `evaluateDataModel` and the optional `custom*` hooks.
pub trait DataNodeModel: Send {
    /// Upstream `evaluateDataModel`: fills `usage` with the data name and baud
    /// rate to publish at `current_time_s` \[s\]. Called only while the node
    /// is on; `usage` holds what was published last.
    fn evaluate_data_model(
        &mut self,
        node: DataNodeFields<'_>,
        usage: &mut DataNodeUsageMsg,
        current_time_s: f64,
    );

    /// Upstream `customReset`, at init (time zero) and at every reset.
    fn custom_reset(&mut self, _node: DataNodeFields<'_>, _current_sim_nanos: u64) {}

    /// Upstream `customReadMessages`: reads the model's own inputs, and
    /// returns whether they could be read.
    fn custom_read_messages(&mut self) -> bool {
        true
    }

    /// Upstream `customWriteMessages`: writes the model's own outputs after
    /// the usage message.
    fn custom_write_messages(&mut self, _node: DataNodeFields<'_>, _current_sim_nanos: u64) {}
}

/// A data producer or sink (upstream `DataNodeBase`), generic over the model
/// that decides what it publishes.
#[derive(Clone, Debug)]
pub struct DataNodeBase<M> {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `nodeDataOutMsg`: the data this node produces (positive baud
    /// rate) or consumes (negative).
    pub node_data_out_msg: Output<DataNodeUsageMsg>,
    /// Upstream `nodeStatusInMsg` (optional): device command whose
    /// `device_cmd` sets [`Self::data_status`] every update.
    pub node_status_in_msg: Input<DeviceCommandMsg>,
    /// Upstream `nodeBaudRate` \[bit/s\]: data rate of the node, positive for
    /// a generator and negative for a sink.
    pub node_baud_rate_bps: f64,
    /// Upstream `nodeDataName`: name of the data the node produces or
    /// consumes.
    pub node_data_name: String,
    /// Upstream `dataStatus`: device mode; 0 is off, 1 (the default) is on,
    /// and other values are further modes, all on.
    pub data_status: u64,
    /// The model that decides the published data.
    pub model: M,
    /// Upstream `nodeDataMsg`: the usage message, kept between updates.
    node_data_msg: DataNodeUsageMsg,
}

impl<M: DataNodeModel> DataNodeBase<M> {
    /// A data node running `model`, on, with a zero baud rate, no data name
    /// and unconnected ports (upstream's constructor).
    pub fn with_model(name: impl Into<String>, model: M) -> Self {
        Self {
            name: name.into(),
            node_data_out_msg: Output::default(),
            node_status_in_msg: Input::default(),
            node_baud_rate_bps: 0.0,
            node_data_name: String::new(),
            data_status: 1,
            model,
            node_data_msg: DataNodeUsageMsg::default(),
        }
    }

    /// The base-class members, for the model's hooks.
    fn fields(&mut self) -> (DataNodeFields<'_>, &mut M, &mut DataNodeUsageMsg) {
        (
            DataNodeFields {
                node_baud_rate_bps: self.node_baud_rate_bps,
                node_data_name: &mut self.node_data_name,
                data_status: self.data_status,
            },
            &mut self.model,
            &mut self.node_data_msg,
        )
    }

    /// Upstream `Reset`.
    fn reset_node(&mut self, current_sim_nanos: u64) {
        let (node, model, _) = self.fields();
        model.custom_reset(node, current_sim_nanos);
    }

    /// Upstream `readMessages`: the device command, then the model's inputs.
    fn read_messages(&mut self) -> bool {
        if self.node_status_in_msg.is_connected() {
            self.data_status = self.node_status_in_msg.read().device_cmd;
        }
        self.model.custom_read_messages()
    }

    /// Upstream `computeDataStatus`: evaluate the model while on, publish a
    /// zeroed message while off.
    fn compute_data_status(&mut self, current_time_s: f64) {
        if self.data_status > 0 {
            let (node, model, usage) = self.fields();
            model.evaluate_data_model(node, usage, current_time_s);
        } else {
            self.node_data_msg = DataNodeUsageMsg::default();
        }
    }
}

impl<M: DataNodeModel> Module for DataNodeBase<M> {
    fn init(&mut self) {
        self.reset_node(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.reset_node(context.current_sim_nanos);
    }

    /// Upstream `UpdateState`: read, evaluate (or zero the message when a
    /// read failed), then write.
    fn update(&mut self, context: &SimulationContext) {
        if self.read_messages() {
            self.compute_data_status(context.current_sim_nanos as f64 * 1.0e-9);
        } else {
            self.node_data_msg = DataNodeUsageMsg::default();
        }
        self.node_data_out_msg.write(self.node_data_msg.clone());
        let (node, model, _) = self.fields();
        model.custom_write_messages(node, context.current_sim_nanos);
    }
}
