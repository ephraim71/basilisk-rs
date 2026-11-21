//! Radio communication: ports of upstream Basilisk
//! `simulation/communication`.
//!
//! [`simple_antenna`] publishes each antenna's radio properties, noise and
//! pose; [`link_budget`] combines two of them into the carrier-to-noise ratio
//! at each end; [`downlink_handling`] turns that ratio into reliable
//! throughput and drains a storage unit as a data node.

pub mod downlink_handling;
pub mod link_budget;
pub mod simple_antenna;
mod spectroscopic_data;
