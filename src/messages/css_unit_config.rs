//! One coarse sun sensor's configuration (upstream
//! `CSSUnitConfigMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Configuration of a single coarse sun sensor: upstream `CSSUnitConfigMsg`,
/// also the element type of [`super::CssConfigMsg`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CssUnitConfigMsg {
    /// Upstream `nHat_B` [-]: sensor unit normal, body components.
    pub n_hat_b: Vector3<f64>,
    /// Upstream `CBias` [-]: calibration gain of this sensor relative to the
    /// others — 1.0 nominally, 1.1 for a sensor reading 10 % strong. (Upstream
    /// labels it \[W\], but it multiplies a dimensionless cosine.)
    pub c_bias: f64,
}

impl_telemetry!(CssUnitConfigMsg { n_hat_b, c_bias });
