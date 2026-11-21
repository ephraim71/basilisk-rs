//! Upstream `simpleNav` with its error model, for the small-body scenarios.
//!
//! basilisk-rs's [`basilisk_rs::sensors::navigation::SimpleNavigation`]
//! publishes the truth: it has no `PMatrix`/`walkBounds` error model. The
//! small-body scenarios feed their filters and controllers with noisy
//! `simpleNav` "measurements", so [`NoisySimpleNav`] adds upstream's 18
//! Gauss-Markov error states (position, velocity, attitude MRP, rate, Sun
//! direction MRP, accumulated ΔV) to that truth, the way upstream's
//! `SimpleNav::computeErrors` and `applyErrors` do: position, velocity, rate
//! and ΔV errors add, the attitude error is composed with `addMRP`, and the
//! Sun direction is rotated by the DCM of its MRP error. The propagation
//! matrix is the identity, with the `crossTrans`/`crossAtt` couplings scaled
//! by the time since the last update.
//!
//! The error process is the crate's [`GaussMarkov`], as in
//! [`basilisk_rs::sensors::planet_nav::PlanetNav`], the crate's port of the
//! same upstream structure for a body ephemeris: `p_matrix_sqrt` is the
//! 1-sigma error step and each state is clamped at its `walk_bounds` entry,
//! as upstream's `GaussMarkov` does. The
//! seed is a hash of the module name (upstream `RNGSeed`), so a rerun repeats
//! its stream; it is not upstream's `std::minstd_rand` stream.
//!
//! The model belongs in `SimpleNavigation` itself; it lives here until that
//! module gains it.

#![allow(dead_code)]

use basilisk_rs::gauss_markov::GaussMarkov;
use basilisk_rs::messages::{
    Input, NavigationAttitudeMsg, NavigationTranslationMsg, Output, PlanetStateMsg,
    SpacecraftStateMsg,
};
use basilisk_rs::utilities::rigid_body_kinematics::{add_mrp, mrp_to_dcm};
use basilisk_rs::{Module, SimulationContext};
use nalgebra::{SMatrix, SVector, Vector3};
use rand::SeedableRng;
use rand::rngs::StdRng;

/// Upstream `numStates`.
pub const SIMPLE_NAV_STATES: usize = 18;
/// A matrix over the error states.
pub type SimpleNavMatrix = SMatrix<f64, SIMPLE_NAV_STATES, SIMPLE_NAV_STATES>;
/// A vector over the error states.
pub type SimpleNavVector = SVector<f64, SIMPLE_NAV_STATES>;

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
        let groups = [
            self.position,
            self.velocity,
            self.attitude,
            self.rate,
            self.sun,
            self.delta_v,
        ];
        SimpleNavMatrix::from_diagonal(&SimpleNavVector::from_fn(|index, _| groups[index / 3]))
    }
}

/// A vector over the error states from one value per three-state group, in
/// the order of [`NavSigmas`] (upstream `walkBounds`).
pub fn grouped(values: [f64; 6]) -> SimpleNavVector {
    SimpleNavVector::from_fn(|index, _| values[index / 3])
}

/// Upstream `simpleNav.SimpleNav()` with its error model.
#[derive(Debug)]
pub struct NoisySimpleNav {
    /// Upstream `ModelTag`; seeds the error stream.
    pub name: String,
    /// Upstream `scStateInMsg`.
    pub spacecraft_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `sunStateInMsg` (optional).
    pub sun_state_in_msg: Input<PlanetStateMsg>,
    /// Upstream `attOutMsg`.
    pub attitude_out_msg: Output<NavigationAttitudeMsg>,
    /// Upstream `transOutMsg`.
    pub translation_out_msg: Output<NavigationTranslationMsg>,
    /// Upstream `PMatrix` (1-sigma, see the module docs).
    pub p_matrix_sqrt: SimpleNavMatrix,
    /// Upstream `walkBounds`.
    pub walk_bounds: SimpleNavVector,
    /// Upstream `crossTrans`.
    pub cross_trans: bool,
    /// Upstream `crossAtt`.
    pub cross_att: bool,
    /// Upstream `RNGSeed`: seeds the error stream at every reset; by default
    /// a hash of the name.
    pub seed: u64,
    error_model: GaussMarkov<SIMPLE_NAV_STATES>,
    rng: StdRng,
    prev_time_nanos: u64,
}

impl NoisySimpleNav {
    pub fn new(
        name: impl Into<String>,
        p_matrix_sqrt: SimpleNavMatrix,
        walk_bounds: SimpleNavVector,
    ) -> Self {
        let name = name.into();
        let seed = seed_from_name(&name);
        Self {
            rng: StdRng::seed_from_u64(seed),
            seed,
            name,
            spacecraft_state_in_msg: Input::default(),
            sun_state_in_msg: Input::default(),
            attitude_out_msg: Output::default(),
            translation_out_msg: Output::default(),
            p_matrix_sqrt,
            walk_bounds,
            cross_trans: false,
            cross_att: false,
            error_model: GaussMarkov::new(SimpleNavMatrix::identity(), p_matrix_sqrt, walk_bounds),
            prev_time_nanos: 0,
        }
    }

    /// Upstream `navErrors`: the errors applied at the last update.
    pub fn nav_errors(&self) -> SimpleNavVector {
        self.error_model.current_state()
    }

    /// Upstream `computeErrors`.
    fn compute_errors(&mut self, current_sim_nanos: u64) -> SimpleNavVector {
        let dt_s = current_sim_nanos.saturating_sub(self.prev_time_nanos) as f64 * 1.0e-9;
        let mut propagation = SimpleNavMatrix::identity();
        for axis in 0..3 {
            if self.cross_trans {
                propagation[(axis, axis + 3)] = dt_s;
            }
            if self.cross_att {
                propagation[(axis + 6, axis + 9)] = dt_s;
            }
        }
        self.error_model.prop_matrix = propagation;
        self.error_model.noise_matrix_sqrt = self.p_matrix_sqrt;
        self.error_model.state_bounds = self.walk_bounds;
        self.error_model.compute_next_state(&mut self.rng)
    }
}

impl Module for NoisySimpleNav {
    /// Upstream `Reset`: restarts the error stream from the seed.
    fn init(&mut self) {
        if !self.spacecraft_state_in_msg.is_connected() {
            log::error!("{}.spacecraft_state_in_msg was not linked", self.name);
        }
        self.error_model.reset();
        self.rng = StdRng::seed_from_u64(self.seed);
        self.prev_time_nanos = 0;
        self.attitude_out_msg
            .write(NavigationAttitudeMsg::default());
        self.translation_out_msg
            .write(NavigationTranslationMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let now = context.current_sim_nanos;
        let state = self.spacecraft_state_in_msg.read();
        let errors = self.compute_errors(now);
        let block =
            |start: usize| Vector3::new(errors[start], errors[start + 1], errors[start + 2]);
        let time_tag_s = now as f64 * 1.0e-9;

        // Upstream `applyErrors`.
        let vehicle_sun_point_body = if self.sun_state_in_msg.is_connected() {
            let to_sun_n = self.sun_state_in_msg.read().position_inertial_m - state.position_m;
            let true_sun_point = state
                .inertial_to_body()
                .transform_vector(&to_sun_n.normalize());
            mrp_to_dcm(block(12)) * true_sun_point
        } else {
            Vector3::zeros()
        };
        self.attitude_out_msg.write(NavigationAttitudeMsg {
            time_tag_s,
            sigma_bn: add_mrp(state.sigma_bn, block(6)),
            omega_bn_b_radps: state.omega_radps + block(9),
            vehicle_sun_point_body,
        });
        self.translation_out_msg.write(NavigationTranslationMsg {
            time_tag_s,
            position_bn_n_m: state.position_m + block(0),
            velocity_bn_n_mps: state.velocity_mps + block(3),
            accumulated_dv_b_mps: state.accumulated_dv_cn_b_mps + block(15),
        });
        self.prev_time_nanos = now;
    }
}

/// The crate sensors' default seed: a hash of the name.
fn seed_from_name(name: &str) -> u64 {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    let mut hasher = DefaultHasher::new();
    name.hash(&mut hasher);
    hasher.finish()
}
