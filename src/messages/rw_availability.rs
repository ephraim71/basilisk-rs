use super::MAX_EFF_COUNT;
use serde::{Deserialize, Serialize};

/// Flight-software availability state for one reaction wheel.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum RwAvailability {
    /// Upstream deliberately assigns available the zero/default value.
    #[default]
    Available = 0,
    Unavailable = 1,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RwAvailabilityMsg {
    #[serde(with = "super::big_array")]
    pub wheel_availability: [RwAvailability; MAX_EFF_COUNT],
}

impl RwAvailabilityMsg {
    pub fn is_available(&self, index: usize) -> bool {
        self.wheel_availability[index] == RwAvailability::Available
    }
}

impl Default for RwAvailabilityMsg {
    fn default() -> Self {
        Self {
            wheel_availability: [RwAvailability::Available; MAX_EFF_COUNT],
        }
    }
}
