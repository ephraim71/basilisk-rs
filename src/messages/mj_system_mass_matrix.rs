//! Mass matrix of a multibody scene (upstream C++
//! `MJSysMassMatrixMsgPayload`).

use nalgebra::DMatrix;
use serde::{Deserialize, Serialize};

use super::MjJointType;

/// Joint-space mass matrix of the whole scene: upstream
/// `MJSysMassMatrixMsg`.
///
/// Upstream's payload holds `std::vector`s, so the message has no telemetry
/// columns.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MjSystemMassMatrixMsg {
    /// Upstream `nSC` [-]: number of spacecraft in the scene.
    pub num_spacecraft: usize,
    /// Upstream `scStartIdx` [-]: first joint of each spacecraft.
    pub sc_start_idx: Vec<usize>,
    /// Upstream `jointTypes` [-]: kind of each joint.
    pub joint_types: Vec<MjJointType>,
    /// Upstream `massMatrix` [-]: square mass matrix, one row and column per
    /// degree of freedom. Upstream flattens it row-major; build it from that
    /// vector with [`DMatrix::from_row_slice`].
    pub mass_matrix: DMatrix<f64>,
}

impl Default for MjSystemMassMatrixMsg {
    fn default() -> Self {
        Self {
            num_spacecraft: 0,
            sc_start_idx: Vec::new(),
            joint_types: Vec::new(),
            mass_matrix: DMatrix::zeros(0, 0),
        }
    }
}
