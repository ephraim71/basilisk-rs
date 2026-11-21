//! Multi-sphere-model charges of one space object (upstream
//! `ChargeMsmMsgPayload`).

use serde::{Deserialize, Serialize};

use crate::telemetry::{TelemetryField, TelemetryMessage};

/// Charge of each multi-sphere-model sphere of one space object: upstream
/// `ChargeMsmMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ChargeMsmMsg {
    /// Upstream `q` \[C\]: one charge per sphere, in the order the object's
    /// spheres were configured. Upstream's `Eigen::VectorXd` has no fixed
    /// size, so this is a `Vec`.
    pub charges_c: Vec<f64>,
}

impl TelemetryMessage for ChargeMsmMsg {
    /// One column per sphere, `charges_c.0` onwards.
    fn flatten(&self) -> Vec<TelemetryField> {
        self.charges_c
            .iter()
            .enumerate()
            .map(|(index, charge_c)| TelemetryField {
                path: format!("charges_c.{index}"),
                value: *charge_c,
            })
            .collect()
    }
}
