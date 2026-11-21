//! Onboard data handling: ports of upstream Basilisk
//! `simulation/onboardDataHandling`.
//!
//! Instruments generate data and transmitters sink it, each a data node
//! ([`data_node_base`]) publishing one [`crate::messages::DataNodeUsageMsg`]
//! per device; storage units ([`data_storage_unit_base`]) take one such input
//! per node and publish their partitions in a
//! [`crate::messages::DataStorageStatusMsg`], which the transmitters read.

pub mod data_node_base;
pub mod data_storage_unit_base;
pub mod mapping_instrument;
pub mod partitioned_storage_unit;
pub mod simple_instrument;
pub mod simple_storage_unit;
pub mod simple_transmitter;
pub mod space_to_ground_transmitter;
#[cfg(test)]
pub(crate) mod test_support;
