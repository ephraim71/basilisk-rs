//! Vizard visualization: upstream Basilisk's `simulation/vizard` family,
//! behind the `vizard` cargo feature.
//!
//! * [`viz_interface::VizInterface`] packs the simulation into Vizard
//!   messages, saved to a file or streamed live over ZeroMQ.
//! * [`data_file_to_viz::DataFileToViz`] replays spacecraft states from a
//!   text file as the messages `VizInterface` reads.
//! * [`viz_structures`] holds the scene description both share (upstream
//!   `_GeneralModuleFiles/vizStructures.h`).
//! * [`proto`] is Vizard's wire format, generated at build time from
//!   upstream's `vizMessage.proto` (kept here as `src/vizard/vizMessage.proto`).
//!
//! Building with the feature needs `protoc` (on `PATH`, or named by the
//! `PROTOC` environment variable) and libzmq:
//!
//! ```text
//! cargo test --features vizard
//! ```

pub mod data_file_to_viz;
pub mod viz_interface;
pub mod viz_structures;

/// Vizard's protobuffer messages (upstream package `vizProtobufferMessage`),
/// as prost generates them from `vizMessage.proto`. [`proto::VizMessage`] is
/// one frame; [`proto::VizInput`] is what a live Vizard sends back.
pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/viz_protobuffer_message.rs"));
}
