use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::MAX_EFF_COUNT;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MtbArrayConfigMsg {
    pub num_mtb: usize,
    /// Columns of the 3-by-numMTB `GtMatrix_B` matrix.
    #[serde(with = "super::big_array")]
    pub dipole_axes_body: [Vector3<f64>; MAX_EFF_COUNT],
    #[serde(with = "super::big_array")]
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

impl MtbArrayConfigMsg {
    pub fn from_active(axes_body: &[Vector3<f64>], max_dipoles_am2: &[f64]) -> Self {
        assert_eq!(
            axes_body.len(),
            max_dipoles_am2.len(),
            "each MTB axis must have a corresponding dipole limit"
        );
        assert!(
            axes_body.len() <= MAX_EFF_COUNT,
            "at most {MAX_EFF_COUNT} MTBs are supported"
        );
        let mut message = Self {
            num_mtb: axes_body.len(),
            ..Self::default()
        };
        message.dipole_axes_body[..axes_body.len()].copy_from_slice(axes_body);
        message.max_dipoles_am2[..max_dipoles_am2.len()].copy_from_slice(max_dipoles_am2);
        message
    }
}
