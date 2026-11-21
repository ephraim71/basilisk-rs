//! Electron gun settings (upstream `ElectronBeamMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// The electron beam a servicer fires at a target: upstream `ElectronBeamMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ElectronBeamMsg {
    /// Upstream `energyEB` \[eV\]: energy of the primary beam electrons.
    pub energy_ev: f64,
    /// Upstream `currentEB` \[A\]: primary beam current, positive for
    /// electrons leaving the servicer.
    pub current_a: f64,
    /// Upstream `alphaEB` \[-\]: fraction of the beam current that reaches
    /// the target, in \[0, 1\] (beam divergence).
    pub alpha: f64,
}

impl_telemetry!(ElectronBeamMsg {
    energy_ev,
    current_a,
    alpha,
});
