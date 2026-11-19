//! An area of a spacecraft (upstream `ProjectedAreaMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// An area of a spacecraft: upstream `ProjectedAreaMsg`. It also stands for
/// upstream `ScSunlitFacetAreaMsg`, whose payload is the same single `area`
/// \[m^2\] (the total facet area exposed to the Sun).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ProjectedAreaMsg {
    /// Upstream `area` \[m^2\].
    pub area_m2: f64,
}

impl_telemetry!(ProjectedAreaMsg { area_m2 });
