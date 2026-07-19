use nalgebra::Vector3;

use super::MAX_EFF_COUNT;

/// Magnetic-torque-bar array geometry and limits.
#[derive(Clone, Debug, PartialEq)]
pub struct MtbArrayConfigMsg {
    pub num_mtb: usize,
    /// Columns of the upstream 3-by-numMTB `GtMatrix_B` matrix.
    pub dipole_axes_body: [Vector3<f64>; MAX_EFF_COUNT],
    pub max_dipoles_am2: [f64; MAX_EFF_COUNT],
}

impl Default for MtbArrayConfigMsg {
    fn default() -> Self {
        Self {
            num_mtb: 0,
            dipole_axes_body: [Vector3::zeros(); MAX_EFF_COUNT],
            max_dipoles_am2: [0.0; MAX_EFF_COUNT],
        }
    }
}
