//! Noisy celestial-body ephemeris: port of upstream Basilisk `planetNav`.
//!
//! Reads a body's ephemeris (upstream `ephemerisInMsg`, an [`EphemerisMsg`])
//! and republishes it with Gauss-Markov errors on its position, velocity,
//! attitude and angular rate, upstream `ephemerisOutMsg`. It does for a body,
//! typically a small body with a poorly known ephemeris, what `simpleNav` does
//! for the spacecraft: the noisy state stands in for an estimator, or feeds
//! one as a measurement. With the default configuration (zero noise) the
//! output is the truth with a fresh time tag.
//!
//! The twelve error states are, in order: position \[m\] (0–2), velocity
//! \[m/s\] (3–5), attitude MRP (6–8) and angular rate \[rad/s\] (9–11). The
//! position, velocity and rate errors add to the truth; the attitude error is
//! an MRP composed onto the true attitude (upstream `addMRP`). The error
//! propagation is the identity, plus, when `cross_trans` is set, the velocity
//! error integrated into the position error over the time since the previous
//! update, and likewise for the rate error into the attitude error when
//! `cross_att` is set.
//!
//! The error model is the crate's [`GaussMarkov`], upstream's: `p_matrix_sqrt`
//! is the 1-sigma error step and each state is clamped at its `walk_bounds`
//! entry, see [`crate::gauss_markov`].
//!
//! As upstream, a reset restarts the random stream from the module's seed and
//! zeroes the error state. The seed comes from the module name, like the
//! other sensors in this crate, and [`PlanetNav::set_rng_seed`] replaces it
//! (upstream `RNGSeed`). The stream is not upstream's `std::minstd_rand`, so
//! outputs match upstream statistically, not sample for sample.
//!
//! Deviations: the configuration, including the two cross flags upstream
//! reads only on reset, is read on every update, so an override applies at
//! once; and a reset also clears the previous-update time, which upstream
//! keeps across a re-initialization.

use nalgebra::{SMatrix, SVector, Vector3};
use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::gauss_markov::GaussMarkov;
use crate::messages::{EphemerisMsg, Input, Output};
use crate::sensors::seed_from_name;
use crate::time::diff_nanos_to_seconds;
use crate::utilities::rigid_body_kinematics::add_mrp;
use crate::{Module, SimulationContext};

/// Upstream `numStates`: position, velocity, attitude and rate, three each.
pub const PLANET_NAV_STATES: usize = 12;

/// Square matrix over the [`PLANET_NAV_STATES`] error states.
pub type PlanetNavMatrix = SMatrix<f64, PLANET_NAV_STATES, PLANET_NAV_STATES>;
/// Vector over the [`PLANET_NAV_STATES`] error states.
pub type PlanetNavVector = SVector<f64, PLANET_NAV_STATES>;

/// Error parameters of [`PlanetNav`].
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PlanetNavConfig {
    /// Module name (upstream `ModelTag`); seeds the noise stream.
    pub name: String,
    /// Upstream `PMatrix`: square root of the error covariance (1-sigma),
    /// over the twelve error states. Zero disables the noise.
    pub p_matrix_sqrt: PlanetNavMatrix,
    /// Upstream `walkBounds`: bound each error state is clamped at; a
    /// non-positive bound leaves that state unbounded.
    pub walk_bounds: PlanetNavVector,
    /// Upstream `crossTrans`: the position error integrates the velocity error.
    pub cross_trans: bool,
    /// Upstream `crossAtt`: the attitude error integrates the rate error.
    pub cross_att: bool,
}

impl Default for PlanetNavConfig {
    /// Upstream's constructor: no noise, no bounds, no cross terms.
    fn default() -> Self {
        Self {
            name: String::new(),
            p_matrix_sqrt: PlanetNavMatrix::zeros(),
            walk_bounds: PlanetNavVector::zeros(),
            cross_trans: false,
            cross_att: false,
        }
    }
}

impl PlanetNavConfig {
    /// Check numeric invariants. Returns a description of the first violation,
    /// or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        if !self.p_matrix_sqrt.iter().all(|value| value.is_finite()) {
            return Err("p_matrix_sqrt must be finite".to_string());
        }
        if !self.walk_bounds.iter().all(|value| value.is_finite()) {
            return Err(format!(
                "walk_bounds must be finite, got {:?}",
                self.walk_bounds
            ));
        }
        Ok(())
    }

    /// Upstream `AMatrix` with its cross terms scaled by `dt_s`: the identity
    /// plus `dt` coupling each velocity (rate) error into the position
    /// (attitude) error when the cross flag is set.
    fn propagation_matrix(&self, dt_s: f64) -> PlanetNavMatrix {
        let mut matrix = PlanetNavMatrix::identity();
        for axis in 0..3 {
            if self.cross_trans {
                matrix[(axis, axis + 3)] = dt_s;
            }
            if self.cross_att {
                matrix[(axis + 6, axis + 9)] = dt_s;
            }
        }
        matrix
    }

    fn error_model(&self) -> GaussMarkov<PLANET_NAV_STATES> {
        GaussMarkov::new(
            self.propagation_matrix(0.0),
            self.p_matrix_sqrt,
            self.walk_bounds,
        )
    }
}

/// Body ephemeris with Gauss-Markov errors: upstream `PlanetNav`.
#[derive(Clone, Debug)]
pub struct PlanetNav {
    /// Error parameters, read on every update so an override applies at once.
    pub config: Output<PlanetNavConfig>,
    /// Upstream `ephemerisInMsg`: the true ephemeris.
    pub ephemeris_in_msg: Input<EphemerisMsg>,
    /// Upstream `ephemerisOutMsg`: the ephemeris with errors.
    pub ephemeris_out_msg: Output<EphemerisMsg>,
    error_model: GaussMarkov<PLANET_NAV_STATES>,
    seed: u64,
    rng: StdRng,
    prev_time_nanos: u64,
}

impl PlanetNav {
    /// # Panics
    ///
    /// If `config` fails [`PlanetNavConfig::validate`].
    pub fn new(config: PlanetNavConfig) -> Self {
        if let Err(msg) = config.validate() {
            panic!("invalid PlanetNavConfig: {msg}");
        }
        let seed = seed_from_name(&config.name);
        Self {
            error_model: config.error_model(),
            seed,
            rng: StdRng::seed_from_u64(seed),
            config: Output::new(config),
            ephemeris_in_msg: Input::default(),
            ephemeris_out_msg: Output::default(),
            prev_time_nanos: 0,
        }
    }

    /// Replace the seed (upstream `RNGSeed`) and restart the stream from it
    /// with a zero error state.
    pub fn set_rng_seed(&mut self, seed: u64) {
        self.seed = seed;
        self.restart_error_model();
    }

    /// Upstream `navErrors`: the error states applied at the last update.
    pub fn nav_errors(&self) -> PlanetNavVector {
        self.error_model.current_state()
    }

    fn restart_error_model(&mut self) {
        self.error_model = self.config.read().error_model();
        self.rng = StdRng::seed_from_u64(self.seed);
    }

    /// Upstream `computeErrors`.
    fn compute_errors(&mut self, current_sim_nanos: u64) -> PlanetNavVector {
        let config = self.config.read();
        let dt_s = diff_nanos_to_seconds(current_sim_nanos, self.prev_time_nanos);
        // Assigned rather than rebuilt: the accumulated walk is private state
        // a rebuild would discard.
        self.error_model.prop_matrix = config.propagation_matrix(dt_s);
        self.error_model.noise_matrix_sqrt = config.p_matrix_sqrt;
        self.error_model.state_bounds = config.walk_bounds;
        self.error_model.compute_next_state(&mut self.rng)
    }
}

/// Upstream `applyErrors`: the errors on the truth, time-tagged `time_tag_s`.
fn apply_errors(truth: &EphemerisMsg, errors: &PlanetNavVector, time_tag_s: f64) -> EphemerisMsg {
    let block = |start: usize| Vector3::new(errors[start], errors[start + 1], errors[start + 2]);
    EphemerisMsg {
        position_bdy_zero_n_m: truth.position_bdy_zero_n_m + block(0),
        velocity_bdy_zero_n_mps: truth.velocity_bdy_zero_n_mps + block(3),
        sigma_bn: add_mrp(truth.sigma_bn, block(6)),
        omega_bn_b_radps: truth.omega_bn_b_radps + block(9),
        time_tag_s,
    }
}

impl Module for PlanetNav {
    fn init(&mut self) {
        if !self.ephemeris_in_msg.is_connected() {
            log::error!(
                "PlanetNav '{}' ephemeris_in_msg was not linked",
                self.config.read().name
            );
        }
        self.restart_error_model();
        self.prev_time_nanos = 0;
        self.ephemeris_out_msg.write(EphemerisMsg::default());
    }

    fn reset(&mut self, _context: &SimulationContext) {
        if !self.ephemeris_in_msg.is_connected() {
            log::error!(
                "PlanetNav '{}' ephemeris_in_msg was not linked",
                self.config.read().name
            );
        }
        self.restart_error_model();
        self.prev_time_nanos = 0;
    }

    fn update(&mut self, context: &SimulationContext) {
        let truth = self.ephemeris_in_msg.read();
        let errors = self.compute_errors(context.current_sim_nanos);
        // Upstream `(double)CurrentSimNanos * NANO2SEC`.
        let time_tag_s = context.current_sim_nanos as f64 * 1.0e-9;
        self.ephemeris_out_msg
            .write(apply_errors(&truth, &errors, time_tag_s));
        self.prev_time_nanos = context.current_sim_nanos;
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `planetNav/_UnitTest/test_planetNav.py`: both tests,
    //! with their noise levels, bounds, run length and thresholds. Upstream's
    //! "corner case" (an all-zero noise matrix and bounds) must run without
    //! failing; here it also leaves the output equal to the truth.
    //!
    //! The statistical tests are seeded with upstream's default `RNGSeed`
    //! (`0x1badcad1`) rather than the name hash, so they do not depend on the
    //! standard library's hasher. Their thresholds were written for
    //! upstream's unit-variance, hard-clamped `GaussMarkov`, which this
    //! crate's model is.

    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{EphemerisMsg, Output};
    use crate::simulation::Simulation;
    use crate::{Module, SimulationContext};

    use super::{PlanetNav, PlanetNavConfig, PlanetNavMatrix, PlanetNavVector, apply_errors};

    const STEP_NANOS: u64 = 100_000_000;
    /// Upstream's default `SysModel::RNGSeed`.
    const UPSTREAM_RNG_SEED: u64 = 0x1bad_cad1;
    /// 144 minutes.
    const RUN_NANOS: u64 = 8_640_000_000_000;

    fn epoch() -> Epoch {
        Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
    }

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: epoch(),
        }
    }

    fn truth() -> EphemerisMsg {
        EphemerisMsg {
            position_bdy_zero_n_m: Vector3::new(10_000.0, 0.0, 0.0),
            ..Default::default()
        }
    }

    /// Upstream's sigmas on the diagonal of `PMatrix`, and its bounds.
    fn upstream_config(name: &str, cross_trans: bool) -> PlanetNavConfig {
        let pos_sigma = 5.0;
        let vel_sigma = 0.035;
        let att_sigma = 1.0 / 360.0 * PI / 180.0;
        let rate_sigma = 0.05 * PI / 180.0;
        let mut diagonal = PlanetNavVector::zeros();
        let mut walk_bounds = PlanetNavVector::zeros();
        for axis in 0..3 {
            diagonal[axis] = pos_sigma;
            diagonal[axis + 3] = vel_sigma;
            diagonal[axis + 6] = att_sigma;
            diagonal[axis + 9] = rate_sigma;
            walk_bounds[axis] = 1000.0;
            walk_bounds[axis + 3] = 1.0;
            walk_bounds[axis + 6] = 0.005;
            walk_bounds[axis + 9] = 0.02;
        }
        PlanetNavConfig {
            name: name.to_string(),
            p_matrix_sqrt: PlanetNavMatrix::from_diagonal(&diagonal),
            walk_bounds,
            cross_trans,
            cross_att: false,
        }
    }

    /// Runs `module`, seeded with upstream's default seed, on [`truth`] every
    /// 0.1 s for 144 min and returns every output.
    fn record(module: &mut PlanetNav) -> Vec<EphemerisMsg> {
        module.set_rng_seed(UPSTREAM_RNG_SEED);
        let truth = Output::new(truth());
        truth.connect_to(&mut module.ephemeris_in_msg);
        module.init();
        let mut outputs = Vec::new();
        let mut nanos = 0;
        while nanos <= RUN_NANOS {
            module.update(&context(nanos));
            outputs.push(module.ephemeris_out_msg.read());
            nanos += STEP_NANOS;
        }
        outputs
    }

    /// The four error blocks of an output, relative to [`truth`].
    fn differences(output: &EphemerisMsg) -> [Vector3<f64>; 4] {
        let truth = truth();
        [
            output.position_bdy_zero_n_m - truth.position_bdy_zero_n_m,
            output.velocity_bdy_zero_n_mps - truth.velocity_bdy_zero_n_mps,
            output.sigma_bn - truth.sigma_bn,
            output.omega_bn_b_radps - truth.omega_bn_b_radps,
        ]
    }

    /// Upstream `test_planetNav`, with `crossTrans` on: per error block, the
    /// components exceed their bound at most 0.3 % of the sample count in
    /// total (attitude with upstream's 1e-14 round-off allowance), and some
    /// component exceeds 80 % of its bound at least once. Then the corner
    /// case.
    #[test]
    fn error_bounds_are_enforced_and_used() {
        let mut module = PlanetNav::new(upstream_config("PlanetNavigation", true));
        let outputs = record(&mut module);
        assert_eq!(outputs.len(), 86_401);

        let bounds = [1000.0, 1.0, 5.0e-3, 0.02];
        let allowances = [1.0, 1.0, 1.0 + 1.0e-14, 1.0];
        let count_allow = outputs.len() as f64 * 0.3 / 100.0;
        let labels = ["position", "velocity", "attitude", "rate"];
        for block in 0..4 {
            let mut over = 0usize;
            let mut near = 0usize;
            for output in &outputs {
                for value in differences(output)[block].iter() {
                    if value.abs() > bounds[block] * allowances[block] {
                        over += 1;
                    }
                    if value.abs() > bounds[block] * 0.8 {
                        near += 1;
                    }
                }
            }
            assert!(
                over as f64 <= count_allow,
                "{}: too many error counts: {over} > {count_allow}",
                labels[block]
            );
            assert!(near >= 1, "{}: too few error counts: {near}", labels[block]);
        }

        // Corner case: all-zero noise and bounds run and add no error.
        module.config.write(PlanetNavConfig {
            name: "PlanetNavigation".to_string(),
            ..Default::default()
        });
        let truth_out = Output::new(truth());
        {
            let mut simulation = Simulation::new(epoch(), false);
            simulation.connect(&truth_out, &mut module.ephemeris_in_msg);
            simulation.add_module("planetNav", &mut module, STEP_NANOS, 0);
            simulation.run_for(STEP_NANOS);
        }
        let output = module.ephemeris_out_msg.read();
        assert_eq!(output.position_bdy_zero_n_m, truth().position_bdy_zero_n_m);
        assert_eq!(output.sigma_bn, Vector3::zeros());
        assert!((output.time_tag_s - 0.1).abs() < 1.0e-15);
    }

    /// Upstream `test_gauss_markov_properties`, without the cross terms:
    /// samples with any position component past its bound are at most 0.3 %
    /// of all samples, and at least one sample has a component past 80 %.
    #[test]
    fn position_errors_follow_the_gauss_markov_statistics() {
        let mut module = PlanetNav::new(upstream_config("planetNav", false));
        let outputs = record(&mut module);

        let count_allow = outputs.len() as f64 * 0.3 / 100.0;
        let position_bound = 1000.0;
        let violations = outputs
            .iter()
            .filter(|output| differences(output)[0].amax() > position_bound)
            .count();
        assert!(
            violations as f64 <= count_allow,
            "too many position errors ({violations} > {count_allow})"
        );
        let large = outputs
            .iter()
            .filter(|output| differences(output)[0].amax() > position_bound * 0.8)
            .count();
        assert!(large >= 1, "position errors too small");
    }

    /// With noise on the velocity error only and `cross_trans` set, the
    /// position error integrates the velocity error over each update
    /// interval; without it the position error stays zero.
    #[test]
    fn cross_trans_integrates_the_velocity_error() {
        for cross_trans in [true, false] {
            let mut p_matrix_sqrt = PlanetNavMatrix::zeros();
            for axis in 3..6 {
                p_matrix_sqrt[(axis, axis)] = 0.03;
            }
            let mut module = PlanetNav::new(PlanetNavConfig {
                name: "cross".to_string(),
                p_matrix_sqrt,
                cross_trans,
                ..Default::default()
            });
            let truth_out = Output::new(truth());
            truth_out.connect_to(&mut module.ephemeris_in_msg);
            module.init();

            let mut expected_position_error = Vector3::zeros();
            let mut previous_velocity_error = Vector3::zeros();
            let mut previous_nanos = 0;
            for nanos in [0, 250_000_000, 500_000_000, 1_500_000_000, 1_600_000_000] {
                module.update(&context(nanos));
                let [position_error, velocity_error, _, _] =
                    differences(&module.ephemeris_out_msg.read());
                if cross_trans {
                    let dt_s = (nanos - previous_nanos) as f64 * 1.0e-9;
                    expected_position_error += dt_s * previous_velocity_error;
                }
                assert!(
                    (position_error - expected_position_error).amax() < 1.0e-12,
                    "cross_trans {cross_trans} at {nanos} ns"
                );
                previous_velocity_error = velocity_error;
                previous_nanos = nanos;
            }
            assert!(previous_velocity_error.amax() > 0.0);
        }
    }

    /// The attitude error composes onto the true attitude as an MRP; the
    /// other blocks add.
    #[test]
    fn attitude_errors_compose_onto_the_truth() {
        use crate::utilities::rigid_body_kinematics::mrp_to_dcm;

        let truth = EphemerisMsg {
            position_bdy_zero_n_m: Vector3::new(1.0, 2.0, 3.0),
            velocity_bdy_zero_n_mps: Vector3::new(-1.0, 0.5, 0.25),
            sigma_bn: Vector3::new(0.2, -0.1, 0.3),
            omega_bn_b_radps: Vector3::new(0.01, 0.02, -0.03),
            time_tag_s: 99.0,
        };
        let errors = PlanetNavVector::from_column_slice(&[
            0.1, 0.2, 0.3, 0.01, 0.02, 0.03, 0.01, -0.02, 0.005, 1e-3, 2e-3, 3e-3,
        ]);
        let noisy = apply_errors(&truth, &errors, 4.5);
        assert_eq!(noisy.time_tag_s, 4.5);
        assert_eq!(
            noisy.position_bdy_zero_n_m,
            truth.position_bdy_zero_n_m + Vector3::new(0.1, 0.2, 0.3)
        );
        assert_eq!(
            noisy.omega_bn_b_radps,
            truth.omega_bn_b_radps + Vector3::new(1e-3, 2e-3, 3e-3)
        );
        // [C(noisy)] = [C(error)] [C(truth)].
        let expected = mrp_to_dcm(Vector3::new(0.01, -0.02, 0.005)) * mrp_to_dcm(truth.sigma_bn);
        assert!((mrp_to_dcm(noisy.sigma_bn) - expected).amax() < 1.0e-14);
    }

    /// A reset restarts the stream: the realization repeats.
    #[test]
    fn reset_repeats_the_realization() {
        let mut module = PlanetNav::new(upstream_config("repeat", true));
        let truth_out = Output::new(truth());
        truth_out.connect_to(&mut module.ephemeris_in_msg);
        let run = |module: &mut PlanetNav| -> Vec<EphemerisMsg> {
            (0..10)
                .map(|step| {
                    module.update(&context(step * STEP_NANOS));
                    module.ephemeris_out_msg.read()
                })
                .collect()
        };
        module.init();
        let first = run(&mut module);
        Module::reset(&mut module, &context(0));
        assert_eq!(run(&mut module), first);
        module.set_rng_seed(5);
        assert_ne!(run(&mut module), first);
    }
}
