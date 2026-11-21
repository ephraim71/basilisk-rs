//! Per-group settings of upstream `simpleNav`'s error model, for the
//! scenarios that write `PMatrix` and `walkBounds` group by group.
//!
//! The error model itself is the library's
//! [`basilisk_rs::sensors::navigation::SimpleNavigation`]: 18 Gauss-Markov
//! error states (position, velocity, attitude MRP, rate, Sun direction MRP,
//! accumulated ΔV, three each), with `p_matrix_sqrt` the 1-sigma error step.
//! These helpers build its diagonal `PMatrix` and its `walkBounds` from one
//! value per three-state group.

#![allow(dead_code)]

use basilisk_rs::sensors::navigation::{SimpleNavMatrix, SimpleNavVector};

/// The per-group standard deviations of upstream's diagonal `PMatrix`
/// \[m, m/s, -, rad/s, -, m/s\]: position, velocity, attitude, rate, Sun
/// direction and ΔV, three states each.
#[derive(Clone, Copy, Debug, Default)]
pub struct NavSigmas {
    pub position: f64,
    pub velocity: f64,
    pub attitude: f64,
    pub rate: f64,
    pub sun: f64,
    pub delta_v: f64,
}

impl NavSigmas {
    /// The diagonal `PMatrix` the scenarios write out entry by entry.
    pub fn p_matrix(&self) -> SimpleNavMatrix {
        SimpleNavMatrix::from_diagonal(&grouped([
            self.position,
            self.velocity,
            self.attitude,
            self.rate,
            self.sun,
            self.delta_v,
        ]))
    }
}

/// A vector over the error states from one value per three-state group, in
/// the order of [`NavSigmas`] (upstream `walkBounds`).
pub fn grouped(values: [f64; 6]) -> SimpleNavVector {
    SimpleNavVector::from_fn(|index, _| values[index / 3])
}
