//! ANISE stand-ins for the parts of upstream's `spiceInterface` that
//! [`super::grav_factory`] does not cover:
//!
//! * [`SpkSpacecraft`]: a spacecraft's state from its own SPK file, upstream
//!   `spiceObject.addSpacecraftNames([...])` and
//!   `transRefStateOutMsgs[i]`, which drives the spacecraft's
//!   `transRefInMsg`;
//! * [`almanac`] and [`spk_read`]: upstream `spkRead`.
//!
//! An ephemeris whose first state other modules read at their reset is
//! scheduled in `staged::UpdatedAtInit`.
//!
//! Include it next to `common.rs` and `grav_factory.rs`:
//!
//! ```ignore
//! #[path = "support/ephemeris.rs"]
//! mod ephemeris;
//! ```

#![allow(dead_code)]

use std::path::Path;

use anise::constants::orientations::J2000;
use anise::prelude::{Almanac, Frame};
use basilisk_rs::messages::{Output, TranslationReferenceMsg};
use basilisk_rs::{Module, SimulationContext};
use hifitime::Epoch;
use nalgebra::Vector3;

use super::grav_factory::PLANETARY_SPK;

/// Upstream `examples/dataForExamples/Spice/spacecraft_21T01.bsp`, the
/// heliocentric trajectory of spacecraft `-60000` (`just
/// fetch-misc-scenario-data`).
pub const SPACECRAFT_21T01_SPK: &str = "assets/spice/spacecraft_21T01.bsp";
/// Upstream `supportData/EphemerisData/hst_edited.bsp`, the Hubble Space
/// Telescope (`just fetch-spice-kernels`).
pub const HST_SPK: &str = "assets/spice/hst_edited.bsp";
/// NAIF ID of upstream's `"-60000"` spacecraft.
pub const SPACECRAFT_21T01_ID: i32 = -60000;
/// NAIF ID of `"HUBBLE SPACE TELESCOPE"`.
pub const HUBBLE_ID: i32 = -48;

/// The J2000 frame centred on the body or spacecraft of NAIF ID `id`.
pub fn j2000_frame(id: i32) -> Frame {
    Frame::new(id, J2000)
}

/// DE440s and `extra_kernels` (repository-relative paths).
pub fn almanac(extra_kernels: &[&str]) -> Almanac {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut almanac = Almanac::new(&root.join(PLANETARY_SPK).to_string_lossy())
        .expect("load DE440s (just fetch-anise-assets)");
    for kernel in extra_kernels {
        almanac = almanac
            .load(&root.join(kernel).to_string_lossy())
            .unwrap_or_else(|error| panic!("cannot load {kernel}: {error}"));
    }
    almanac
}

/// Upstream `spkRead(target, time, "J2000", observer)`: the geometric state
/// of `target` relative to `observer` at `epoch` [m, m/s].
pub fn spk_read(
    almanac: &Almanac,
    target: Frame,
    observer: Frame,
    epoch: Epoch,
) -> (Vector3<f64>, Vector3<f64>) {
    let state = almanac
        .translate_geometric(target, observer, epoch)
        .unwrap_or_else(|error| panic!("no ephemeris of {target} at {epoch}: {error}"));
    (
        Vector3::new(state.radius_km.x, state.radius_km.y, state.radius_km.z) * 1.0e3,
        Vector3::new(
            state.velocity_km_s.x,
            state.velocity_km_s.y,
            state.velocity_km_s.z,
        ) * 1.0e3,
    )
}

/// A spacecraft's position and velocity from an SPK file, relative to
/// `observer` in J2000: the stand-in for upstream's spacecraft outputs of
/// `spiceInterface` (`transRefStateOutMsgs`, zero acceleration).
pub struct SpkSpacecraft {
    pub name: String,
    pub target: Frame,
    pub observer: Frame,
    /// Upstream `transRefStateOutMsgs[i]`.
    pub trans_ref_out_msg: Output<TranslationReferenceMsg>,
    almanac: Almanac,
}

impl SpkSpacecraft {
    /// The spacecraft of NAIF ID `id` in `spk` (loaded with DE440s).
    pub fn new(name: &str, id: i32, spk: &str, observer: Frame) -> Self {
        Self {
            name: name.to_string(),
            target: j2000_frame(id),
            observer,
            trans_ref_out_msg: Output::default(),
            almanac: almanac(&[spk]),
        }
    }
}

impl Module for SpkSpacecraft {
    fn init(&mut self) {
        self.trans_ref_out_msg
            .write(TranslationReferenceMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let (position_m, velocity_mps) = spk_read(
            &self.almanac,
            self.target,
            self.observer,
            context.current_epoch,
        );
        self.trans_ref_out_msg.write(TranslationReferenceMsg {
            position_m,
            velocity_mps,
            acceleration_mps2: Vector3::zeros(),
        });
    }
}
