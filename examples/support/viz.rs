//! The part of upstream `vizSupport` the misc scenarios use, for the
//! `vizard` feature: a [`VizInterface`] saving a Vizard file under
//! `examples/output/vizard/` (upstream `enableUnityVisualization(...,
//! saveFile=...)`), and a gravity body's Vizard description.
//!
//! Include it, with `common.rs` and `grav_factory.rs`, only with the
//! feature:
//!
//! ```ignore
//! #[cfg(feature = "vizard")]
//! #[path = "support/viz.rs"]
//! mod viz;
//! ```

#![allow(dead_code)]

use std::path::PathBuf;

use basilisk_rs::vizard::viz_interface::{VizInterface, VizInterfaceConfig};
use basilisk_rs::vizard::viz_structures::GravBodyInfo;

use super::common::output_path;
use super::grav_factory::Body;

/// A fresh `examples/output/vizard/<stem>.bin`.
pub fn vizard_file(stem: &str) -> PathBuf {
    output_path("vizard", &format!("{stem}.bin"))
}

/// Upstream `vizSupport.enableUnityVisualization(scSim, task, scList,
/// saveFile=...)`: an interface that saves its messages to `save_file`.
/// Add the spacecraft and bodies, then schedule it after the modules it
/// reads.
pub fn saving_interface(save_file: PathBuf) -> VizInterface {
    VizInterface::new(VizInterfaceConfig {
        name: "vizInterface".to_string(),
        save_file: Some(save_file),
        ..VizInterfaceConfig::default()
    })
}

/// Upstream's `gravBodyInformation` entry of a factory body.
pub fn grav_body_info(body: Body) -> GravBodyInfo {
    GravBodyInfo {
        body_name: body.name().to_string(),
        mu_m3ps2: body.mu_m3ps2(),
        rad_equator_m: body.radius_equator_m(),
        radius_ratio: 1.0,
        ..GravBodyInfo::default()
    }
}
