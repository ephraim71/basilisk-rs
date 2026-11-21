//! Navigation solution from spacecraft truth: port of upstream Basilisk
//! `simpleNav`, with its Gauss-Markov error model.
//!
//! Republishes spacecraft truth, with errors, in the navigation messages the
//! flight-software modules read: upstream `attOutMsg`
//! ([`NavigationAttitudeMsg`]) and `transOutMsg`
//! ([`NavigationTranslationMsg`]), time-tagged with the update time.
//!
//! The truth (upstream `trueAttState` / `trueTransState`): the spacecraft
//! position, velocity, attitude and rate; the accumulated ΔV, copied from
//! the spacecraft state's `TotalAccumDVBdy`
//! ([`SpacecraftStateMsg::accumulated_dv_cn_b_mps`]) as upstream does; and
//! the Sun-point vector (`vehSunPntBdy`), the unit spacecraft-to-Sun
//! direction in body components when the optional
//! [`SimpleNavigation::sun_state_in_msg`] is connected, and zero otherwise.
//!
//! # Errors
//!
//! Upstream's 18 error states, three each, in order: position \[m\] (0–2),
//! velocity \[m/s\] (3–5), attitude MRP (6–8), body rate \[rad/s\] (9–11),
//! Sun-heading MRP (12–14) and accumulated ΔV \[m/s\] (15–17). The position,
//! velocity, rate and ΔV errors add to the truth; the attitude error is
//! composed onto the true attitude (upstream `addMRP`); the Sun heading is
//! rotated by the DCM of its MRP error, and stays zero without a Sun input.
//! The error propagation is the identity, plus, when
//! [`SimpleNavigation::cross_trans`] is set, the velocity error integrated
//! into the position error over the time since the previous update, and
//! likewise the rate error into the attitude error with
//! [`SimpleNavigation::cross_att`] (upstream `AMatrix` and `computeErrors`).
//!
//! The error model is the crate's [`GaussMarkov`], upstream's:
//! [`SimpleNavigation::p_matrix_sqrt`] (upstream `PMatrix`) is the 1-sigma
//! error step and each state is clamped at its
//! [`SimpleNavigation::walk_bounds`] entry, see [`crate::gauss_markov`]. Both
//! default to zero, upstream's constructor, which leaves the errors at zero.
//!
//! As [`crate::sensors::planet_nav::PlanetNav`] does, a reset restarts the
//! random stream from the module's seed and zeroes the error state. The seed
//! comes from the module name, like the other sensors in this crate, and
//! [`SimpleNavigation::set_rng_seed`] replaces it (upstream `RNGSeed`). The
//! stream is not upstream's `std::minstd_rand`, so outputs match upstream
//! statistically, not sample for sample.
//!
//! Deviations: the error parameters, including the two cross flags upstream
//! reads only on reset, are read on every update, so a change applies at
//! once; a reset also clears the previous-update time, which upstream keeps
//! across a re-initialization; `PMatrix`/`walkBounds` are sized at compile
//! time, so upstream's wrong-size errors cannot occur; and a zero attitude or
//! Sun-heading error leaves that truth exact, where upstream's MRP
//! composition rounds it in the last digits.

use nalgebra::{SMatrix, SVector, Vector3};
use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::gauss_markov::GaussMarkov;
use crate::messages::{
    Input, NavigationAttitudeMsg, NavigationTranslationMsg, Output, PlanetStateMsg,
    SpacecraftStateMsg,
};
use crate::sensors::seed_from_name;
use crate::time::diff_nanos_to_seconds;
use crate::utilities::rigid_body_kinematics::{add_mrp, mrp_to_dcm};
use crate::{Module, SimulationContext};

/// Upstream `numStates`: position, velocity, attitude, rate, Sun heading and
/// accumulated ΔV, three each.
pub const SIMPLE_NAV_STATES: usize = 18;

/// Square matrix over the [`SIMPLE_NAV_STATES`] error states.
pub type SimpleNavMatrix = SMatrix<f64, SIMPLE_NAV_STATES, SIMPLE_NAV_STATES>;
/// Vector over the [`SIMPLE_NAV_STATES`] error states.
pub type SimpleNavVector = SVector<f64, SIMPLE_NAV_STATES>;

/// Navigation solution with Gauss-Markov errors: upstream `simpleNav`.
#[derive(Clone, Debug)]
pub struct SimpleNavigation {
    /// Module name (upstream `ModelTag`); seeds the error stream.
    pub name: String,
    /// Upstream `scStateInMsg`: spacecraft truth state.
    pub spacecraft_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `sunStateInMsg` (optional): the Sun's state, for the
    /// Sun-point vector.
    pub sun_state_in_msg: Input<PlanetStateMsg>,
    /// Upstream `attOutMsg`: attitude and body rate.
    pub attitude_out_msg: Output<NavigationAttitudeMsg>,
    /// Upstream `transOutMsg`: inertial position and velocity.
    pub translation_out_msg: Output<NavigationTranslationMsg>,
    /// Upstream `PMatrix`: square root of the error covariance (1-sigma),
    /// over the eighteen error states. Zero (the default) disables the noise.
    pub p_matrix_sqrt: SimpleNavMatrix,
    /// Upstream `walkBounds`: the bound each error state is clamped at; a
    /// non-positive bound leaves that state unbounded.
    pub walk_bounds: SimpleNavVector,
    /// Upstream `crossTrans`: the position error integrates the velocity
    /// error.
    pub cross_trans: bool,
    /// Upstream `crossAtt`: the attitude error integrates the rate error.
    pub cross_att: bool,
    error_model: GaussMarkov<SIMPLE_NAV_STATES>,
    seed: u64,
    rng: StdRng,
    /// Upstream `prevTime`.
    prev_time_nanos: u64,
}

impl SimpleNavigation {
    /// The module with upstream's defaults: no errors.
    pub fn new(name: impl Into<String>) -> Self {
        Self::with_errors(name, SimpleNavMatrix::zeros(), SimpleNavVector::zeros())
    }

    /// The module with upstream's `PMatrix` (1-sigma) and `walkBounds`.
    pub fn with_errors(
        name: impl Into<String>,
        p_matrix_sqrt: SimpleNavMatrix,
        walk_bounds: SimpleNavVector,
    ) -> Self {
        let name = name.into();
        let seed = seed_from_name(&name);
        Self {
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
            seed,
            rng: StdRng::seed_from_u64(seed),
            prev_time_nanos: 0,
        }
    }

    /// The seed of the error stream (upstream `RNGSeed`): by default a hash
    /// of the name.
    pub fn rng_seed(&self) -> u64 {
        self.seed
    }

    /// Replace the seed (upstream `RNGSeed`) and restart the stream from it
    /// with a zero error state.
    pub fn set_rng_seed(&mut self, seed: u64) {
        self.seed = seed;
        self.restart_error_model();
    }

    /// Upstream `navErrors`: the error states applied at the last update.
    pub fn nav_errors(&self) -> SimpleNavVector {
        self.error_model.current_state()
    }

    fn restart_error_model(&mut self) {
        self.error_model = GaussMarkov::new(
            SimpleNavMatrix::identity(),
            self.p_matrix_sqrt,
            self.walk_bounds,
        );
        self.rng = StdRng::seed_from_u64(self.seed);
    }

    /// Upstream `AMatrix` with its cross terms scaled by `dt_s`.
    fn propagation_matrix(&self, dt_s: f64) -> SimpleNavMatrix {
        let mut matrix = SimpleNavMatrix::identity();
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

    /// Upstream `computeErrors`.
    fn compute_errors(&mut self, current_sim_nanos: u64) -> SimpleNavVector {
        let dt_s = diff_nanos_to_seconds(current_sim_nanos, self.prev_time_nanos);
        // Assigned rather than rebuilt: the accumulated walk is private state
        // a rebuild would discard.
        self.error_model.prop_matrix = self.propagation_matrix(dt_s);
        self.error_model.noise_matrix_sqrt = self.p_matrix_sqrt;
        self.error_model.state_bounds = self.walk_bounds;
        self.error_model.compute_next_state(&mut self.rng)
    }

    fn check_inputs(&self) {
        if !self.spacecraft_state_in_msg.is_connected() {
            log::error!(
                "simple navigation '{}' spacecraft_state_in_msg is not connected",
                self.name
            );
        }
    }
}

/// Error state block `start..start + 3`.
fn block(errors: &SimpleNavVector, start: usize) -> Vector3<f64> {
    Vector3::new(errors[start], errors[start + 1], errors[start + 2])
}

/// `truth` with the MRP rotation `error` applied by `apply`. A zero error
/// leaves the truth exact, where upstream's MRP composition rounds it.
fn rotated(
    truth: Vector3<f64>,
    error: Vector3<f64>,
    apply: impl FnOnce(Vector3<f64>) -> Vector3<f64>,
) -> Vector3<f64> {
    if error == Vector3::zeros() {
        truth
    } else {
        apply(error)
    }
}

impl Module for SimpleNavigation {
    fn init(&mut self) {
        self.check_inputs();
        self.restart_error_model();
        self.prev_time_nanos = 0;
        self.attitude_out_msg
            .write(NavigationAttitudeMsg::default());
        self.translation_out_msg
            .write(NavigationTranslationMsg::default());
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_inputs();
        self.restart_error_model();
        self.prev_time_nanos = 0;
    }

    fn update(&mut self, context: &SimulationContext) {
        let state = self.spacecraft_state_in_msg.read();
        // Upstream `(double)Clock * NANO2SEC` on both outputs.
        let time_tag_s = context.current_sim_nanos as f64 * 1.0e-9;

        // Upstream `computeTrueOutput`: the spacecraft-to-Sun direction in
        // body components, only with the Sun input linked.
        let true_sun_point_body = self.sun_state_in_msg.is_connected().then(|| {
            let to_sun_n = self.sun_state_in_msg.read().position_inertial_m - state.position_m;
            state
                .inertial_to_body()
                .transform_vector(&to_sun_n.normalize())
        });
        let errors = self.compute_errors(context.current_sim_nanos);

        // Upstream `applyErrors`.
        let vehicle_sun_point_body = true_sun_point_body.map_or_else(Vector3::zeros, |sun| {
            rotated(sun, block(&errors, 12), |error| mrp_to_dcm(error) * sun)
        });
        let sigma_bn = rotated(state.sigma_bn, block(&errors, 6), |error| {
            add_mrp(state.sigma_bn, error)
        });
        self.attitude_out_msg.write(NavigationAttitudeMsg {
            time_tag_s,
            sigma_bn,
            omega_bn_b_radps: state.omega_radps + block(&errors, 9),
            vehicle_sun_point_body,
        });
        self.translation_out_msg.write(NavigationTranslationMsg {
            time_tag_s,
            position_bn_n_m: state.position_m + block(&errors, 0),
            velocity_bn_n_mps: state.velocity_mps + block(&errors, 3),
            // Upstream `v3Copy(inertialState.TotalAccumDVBdy, trueTransState.vehAccumDV)`
            // plus the ΔV error.
            accumulated_dv_b_mps: state.accumulated_dv_cn_b_mps + block(&errors, 15),
        });
        self.prev_time_nanos = context.current_sim_nanos;
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use crate::dynamics::gravity::GravBodyData;
    use crate::fsw_algorithms::att_guidance::hill_point::{
        HillPoint, HillPointConfig, compute_hill_pointing_reference,
    };
    use crate::fsw_algorithms::att_guidance::velocity_point::{VelocityPoint, VelocityPointConfig};
    use crate::messages::{AttitudeReferenceMsg, Output, SpacecraftStateMsg};
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};
    use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
    use crate::{Module, SimulationContext};

    use super::{SimpleNavMatrix, SimpleNavVector, SimpleNavigation};

    // Ports of upstream `simpleNav/_UnitTest/test_simpleNav.py`: both tests,
    // with their noise levels, bounds, run lengths and thresholds. Upstream's
    // corner case (a 12-state `PMatrix` and `walkBounds` raising an error)
    // cannot be written here: the matrices are 18-state by type. The noise
    // stream is seeded with upstream's default `RNGSeed` (`0x1badcad1`) so
    // the statistics do not depend on the standard library's hasher.

    /// Upstream's 1-sigma levels and bounds, per three-state group.
    const SIGMAS: [f64; 6] = [
        5.0,
        0.035,
        1.0 / 360.0 * std::f64::consts::PI / 180.0,
        0.05 * std::f64::consts::PI / 180.0,
        std::f64::consts::PI / 180.0,
        0.1 * std::f64::consts::PI / 180.0,
    ];
    const BOUNDS: [f64; 6] = [
        1000.0,
        1.0,
        0.005,
        0.02,
        5.0 * std::f64::consts::PI / 180.0,
        0.053,
    ];
    /// Upstream's task rate (0.1 s) and run length (144 min).
    const STEP_NANOS: u64 = 100_000_000;
    const RUN_NANOS: u64 = 60 * 144 * 1_000_000_000;

    fn grouped(values: [f64; 6], scale: f64) -> SimpleNavVector {
        SimpleNavVector::from_fn(|index, _| scale * values[index / 3])
    }

    /// Upstream's module: position `[10000, 0, 0] m`, the Sun at `sun_m`,
    /// `crossTrans` on, `crossAtt` off, seeded with upstream's default seed.
    fn upstream_module(sun_m: Vector3<f64>) -> SimpleNavigation {
        use crate::messages::PlanetStateMsg;

        let state = Output::new(SpacecraftStateMsg {
            position_m: Vector3::new(10_000.0, 0.0, 0.0),
            ..SpacecraftStateMsg::default()
        });
        let sun = Output::new(PlanetStateMsg {
            position_inertial_m: sun_m,
            ..PlanetStateMsg::default()
        });
        let mut navigation = SimpleNavigation::with_errors(
            "SimpleNavigation",
            SimpleNavMatrix::from_diagonal(&grouped(SIGMAS, 1.0)),
            grouped(BOUNDS, 1.0),
        );
        navigation.cross_trans = true;
        navigation.cross_att = false;
        navigation.spacecraft_state_in_msg.connect(state.slot());
        navigation.sun_state_in_msg.connect(sun.slot());
        navigation.set_rng_seed(0x1bad_cad1);
        navigation.init();
        navigation
    }

    /// One sample of the navigation outputs' deviation from the truth:
    /// position, velocity, attitude, rate and ΔV errors per axis, and the
    /// Sun heading's angle from the true one.
    struct Deviation {
        time_nanos: u64,
        groups: [Vector3<f64>; 5],
        sun_angle_rad: f64,
    }

    fn run(
        navigation: &mut SimpleNavigation,
        times: impl Iterator<Item = u64>,
        true_sun: Vector3<f64>,
        samples: &mut Vec<Deviation>,
    ) {
        for time_nanos in times {
            navigation.update(&SimulationContext {
                current_sim_nanos: time_nanos,
                current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
            });
            let attitude = navigation.attitude_out_msg.read();
            let translation = navigation.translation_out_msg.read();
            samples.push(Deviation {
                time_nanos,
                groups: [
                    translation.position_bn_n_m - Vector3::new(10_000.0, 0.0, 0.0),
                    translation.velocity_bn_n_mps,
                    attitude.sigma_bn,
                    attitude.omega_bn_b_radps,
                    translation.accumulated_dv_b_mps,
                ],
                sun_angle_rad: attitude
                    .vehicle_sun_point_body
                    .dot(&true_sun)
                    .clamp(-1.0, 1.0)
                    .acos(),
            });
        }
    }

    /// Upstream `test_unitSimpleNav`: 144 min at the upstream noise, then
    /// 144 min at three times the noise and bounds. At most 0.3 % of the
    /// samples per state group pass the bounds (with the MRP composition's
    /// round-off at a clamped attitude bound, and `4 √3` times the bound for
    /// the Sun-heading angle), and every group passes 80 % of them at least
    /// once.
    #[test]
    fn errors_stay_within_and_use_their_bounds_like_upstream() {
        let sun_m = Vector3::new(10_000.0, 1_000.0, 0.0);
        let true_sun = (sun_m - Vector3::new(10_000.0, 0.0, 0.0)).normalize();
        let mut navigation = upstream_module(sun_m);
        let mut samples = Vec::new();
        run(
            &mut navigation,
            (0..=RUN_NANOS).step_by(STEP_NANOS as usize),
            true_sun,
            &mut samples,
        );
        // Upstream raises the noise and bounds between the two runs.
        navigation.p_matrix_sqrt = SimpleNavMatrix::from_diagonal(&grouped(SIGMAS, 3.0));
        navigation.walk_bounds = grouped(BOUNDS, 3.0);
        run(
            &mut navigation,
            (RUN_NANOS + STEP_NANOS..=2 * RUN_NANOS).step_by(STEP_NANOS as usize),
            true_sun,
            &mut samples,
        );

        let count_allowed = samples.len() as f64 * 0.3 / 100.0;
        for threshold in [1.0, 0.8] {
            let mut counts = [0usize; 6];
            for sample in &samples {
                let scale = if sample.time_nanos < RUN_NANOS {
                    1.0
                } else {
                    3.0
                };
                for (group, deviation) in sample.groups.iter().enumerate() {
                    // Upstream group order: position, velocity, attitude,
                    // rate, then ΔV (bound index 5).
                    let bound_index = if group == 4 { 5 } else { group };
                    let mut bound = scale * BOUNDS[bound_index] * threshold;
                    if group == 2 && threshold == 1.0 {
                        bound *= 1.0 + 1.0e-14;
                    }
                    counts[group] += deviation.iter().filter(|value| value.abs() > bound).count();
                }
                let sun_bound = 4.0 * 3.0_f64.sqrt() * scale * BOUNDS[4] * threshold;
                if sample.sun_angle_rad.abs() > sun_bound {
                    counts[5] += 1;
                }
            }
            for (group, count) in counts.iter().enumerate() {
                if threshold == 1.0 {
                    assert!(
                        *count as f64 <= count_allowed,
                        "group {group}: {count} samples out of bounds (> {count_allowed})"
                    );
                } else {
                    assert!(*count >= 1, "group {group} never reached 80 % of its bound");
                }
            }
        }
    }

    /// Upstream `test_gauss_markov_properties`: 144 min of the upstream
    /// noise with the Sun at `[10000, 10000, 0] m`; the position errors pass
    /// their bounds in at most 0.3 % of the samples and pass 80 % of them at
    /// least once.
    #[test]
    fn position_errors_follow_the_gauss_markov_model_like_upstream() {
        let sun_m = Vector3::new(10_000.0, 10_000.0, 0.0);
        let true_sun = (sun_m - Vector3::new(10_000.0, 0.0, 0.0)).normalize();
        let mut navigation = upstream_module(sun_m);
        let mut samples = Vec::new();
        run(
            &mut navigation,
            (0..=RUN_NANOS).step_by(STEP_NANOS as usize),
            true_sun,
            &mut samples,
        );
        let over = |threshold: f64| {
            samples
                .iter()
                .map(|sample| {
                    sample.groups[0]
                        .iter()
                        .filter(|value| value.abs() > BOUNDS[0] * threshold)
                        .count()
                })
                .sum::<usize>()
        };
        assert!(over(1.0) as f64 <= samples.len() as f64 * 0.3 / 100.0);
        assert!(over(0.8) >= 1);
    }

    /// Upstream `computeErrors` with `crossTrans`: the position error grows
    /// by the velocity error times the time since the previous update, on
    /// top of its own noise; with no position noise the growth is exact.
    #[test]
    fn cross_trans_integrates_the_velocity_error_into_the_position() {
        let mut p_matrix = SimpleNavMatrix::zeros();
        for axis in 3..6 {
            p_matrix[(axis, axis)] = 0.01;
        }
        let state = Output::new(SpacecraftStateMsg::default());
        let mut navigation =
            SimpleNavigation::with_errors("nav", p_matrix, SimpleNavVector::zeros());
        navigation.cross_trans = true;
        navigation.spacecraft_state_in_msg.connect(state.slot());
        navigation.init();

        let mut previous = SimpleNavVector::zeros();
        for step in 1..=20_u64 {
            navigation.update(&SimulationContext {
                current_sim_nanos: step * 500_000_000,
                current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
            });
            let errors = navigation.nav_errors();
            for axis in 0..3 {
                let expected = previous[axis] + 0.5 * previous[axis + 3];
                assert!((errors[axis] - expected).abs() < 1.0e-15);
            }
            assert_eq!(
                navigation.translation_out_msg.read().position_bn_n_m,
                errors.fixed_rows::<3>(0).into_owned()
            );
            previous = errors;
        }
        assert!(previous.fixed_rows::<3>(0).norm() > 0.0);
        // Rates, attitude, Sun heading and ΔV carry no noise.
        assert!(
            previous
                .fixed_rows::<12>(6)
                .iter()
                .all(|value| *value == 0.0)
        );
    }

    /// A reset restarts the error stream from the seed with a zero error
    /// state, so a rerun repeats its errors.
    #[test]
    fn a_reset_repeats_the_error_stream() {
        let state = Output::new(SpacecraftStateMsg::default());
        let mut navigation = SimpleNavigation::with_errors(
            "nav",
            SimpleNavMatrix::from_diagonal(&grouped(SIGMAS, 1.0)),
            grouped(BOUNDS, 1.0),
        );
        navigation.spacecraft_state_in_msg.connect(state.slot());
        navigation.init();
        let stream = |navigation: &mut SimpleNavigation| {
            (1..=5_u64)
                .map(|step| {
                    let context = SimulationContext {
                        current_sim_nanos: step * STEP_NANOS,
                        current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
                    };
                    navigation.update(&context);
                    navigation.nav_errors()
                })
                .collect::<Vec<_>>()
        };
        let first = stream(&mut navigation);
        Module::reset(
            &mut navigation,
            &SimulationContext {
                current_sim_nanos: 0,
                current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
            },
        );
        assert_eq!(navigation.nav_errors(), SimpleNavVector::zeros());
        assert_eq!(stream(&mut navigation), first);
        assert!(first[4].norm() > 0.0);
    }

    fn assert_same_reference(actual: &AttitudeReferenceMsg, expected: &AttitudeReferenceMsg) {
        assert_eq!(actual.sigma_rn, expected.sigma_rn);
        assert_eq!(actual.omega_rn_n_radps, expected.omega_rn_n_radps);
        assert_eq!(actual.domega_rn_n_radps2, expected.domega_rn_n_radps2);
    }

    /// Upstream `simpleNav` with `sunStateInMsg` linked: `vehSunPntBdy` is
    /// `[BN] (r_Sun - r_BN) / |r_Sun - r_BN|`.
    #[test]
    fn the_sun_input_gives_the_body_sun_heading() {
        use crate::messages::PlanetStateMsg;

        let state = SpacecraftStateMsg {
            position_m: Vector3::new(7_000_000.0, -1_000.0, 2_000.0),
            sigma_bn: Vector3::new(0.1, -0.2, 0.3),
            ..SpacecraftStateMsg::default()
        };
        let sun_position = Vector3::new(-1.0e11, 5.0e10, 2.0e10);
        let state_out = Output::new(state.clone());
        let sun_out = Output::new(PlanetStateMsg {
            position_inertial_m: sun_position,
            ..PlanetStateMsg::default()
        });
        let mut navigation = SimpleNavigation::new("simple_navigation");
        navigation.spacecraft_state_in_msg.connect(state_out.slot());
        navigation.sun_state_in_msg.connect(sun_out.slot());
        navigation.init();
        navigation.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });

        let expected = mrp_to_dcm(state.sigma_bn) * (sun_position - state.position_m).normalize();
        let actual = navigation.attitude_out_msg.read().vehicle_sun_point_body;
        assert!(
            (actual - expected).norm() < 1.0e-15,
            "{actual} vs {expected}"
        );
    }

    #[test]
    fn copies_spacecraft_truth_exactly() {
        let state = SpacecraftStateMsg {
            position_m: Vector3::new(7_000_000.0, -1_000.0, 2_000.0),
            velocity_mps: Vector3::new(10.0, 7_500.0, -20.0),
            sigma_bn: Vector3::new(0.1, -0.2, 0.3),
            omega_radps: Vector3::new(0.01, 0.02, -0.03),
            accumulated_dv_cn_b_mps: Vector3::new(0.5, -1.5, 2.5),
            accumulated_dv_bn_b_mps: Vector3::new(9.0, 9.0, 9.0),
            accumulated_dv_cn_n_mps: Vector3::new(-9.0, -9.0, -9.0),
        };
        let state_out = Output::new(state.clone());
        let mut navigation = SimpleNavigation::new("simple_navigation");
        navigation.spacecraft_state_in_msg.connect(state_out.slot());

        navigation.init();
        navigation.update(&SimulationContext {
            current_sim_nanos: 12_345_678_900,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });

        let attitude = navigation.attitude_out_msg.read();
        assert!((attitude.time_tag_s - 12.345_678_9).abs() < 1.0e-14);
        assert_eq!(attitude.sigma_bn, state.sigma_bn);
        assert_eq!(attitude.omega_bn_b_radps, state.omega_radps);
        assert_eq!(attitude.vehicle_sun_point_body, Vector3::zeros());

        let translation = navigation.translation_out_msg.read();
        assert_eq!(translation.time_tag_s, attitude.time_tag_s);
        assert_eq!(translation.position_bn_n_m, state.position_m);
        assert_eq!(translation.velocity_bn_n_mps, state.velocity_mps);
        assert_eq!(
            translation.accumulated_dv_b_mps, state.accumulated_dv_cn_b_mps,
            "upstream copies TotalAccumDVBdy"
        );
    }

    /// A propagated spacecraft feeds `hillPoint` and `velocityPoint` through
    /// the translation output: each reference is the one computed from the
    /// spacecraft truth, and for the circular orbit the Hill frame turns at the
    /// mean motion with its first axis along the position.
    #[test]
    fn translation_output_drives_orbit_pointing_guidance() {
        const MU_EARTH_M3PS2: f64 = 3.986_004_418e14;
        const STEP_NANOS: u64 = 1_000_000_000;
        let radius_m = 7_000_000.0;
        let inclination = 30.0_f64.to_radians();
        let speed_mps = (MU_EARTH_M3PS2 / radius_m).sqrt();
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 100.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::identity(),
            integration_step_nanos: STEP_NANOS,
            initial_position_m: Vector3::new(radius_m, 0.0, 0.0),
            initial_velocity_mps: speed_mps
                * Vector3::new(0.0, inclination.cos(), inclination.sin()),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: None,
        });
        spacecraft
            .add_grav_body(
                GravBodyData::point_mass(
                    "earth",
                    MU_EARTH_M3PS2,
                    true,
                    Vector3::zeros(),
                    Vector3::zeros(),
                )
                .expect("valid Earth gravity body"),
            )
            .expect("unique central gravity body");
        let mut navigation = SimpleNavigation::new("simple_navigation");
        let mut hill_point = HillPoint::new(HillPointConfig::default());
        let mut velocity_point = VelocityPoint::new(VelocityPointConfig {
            mu_m3ps2: MU_EARTH_M3PS2,
            ..Default::default()
        });

        {
            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            crate::connect!(&simulation,
                &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
                &navigation.translation_out_msg => &mut hill_point.trans_nav_in_msg,
                &navigation.translation_out_msg => &mut velocity_point.trans_nav_in_msg,
            );
            crate::schedule! { simulation,
                "spacecraft" => &mut spacecraft, period = STEP_NANOS, priority = 30;
                "navigation" => &mut navigation, period = STEP_NANOS, priority = 20;
                "hill_point" => &mut hill_point, period = STEP_NANOS, priority = 10;
                "velocity_point" => &mut velocity_point, period = STEP_NANOS, priority = 10;
            }
            simulation.run_for(600 * STEP_NANOS);
        }

        let truth = spacecraft.state_out.read();
        let translation = navigation.translation_out_msg.read();
        assert_eq!(translation.time_tag_s, 600.0);
        assert_eq!(translation.position_bn_n_m, truth.position_m);
        assert_eq!(translation.velocity_bn_n_mps, truth.velocity_mps);
        assert!(
            (truth.position_m - Vector3::new(radius_m, 0.0, 0.0)).norm() > 1.0e6,
            "the spacecraft has moved along its orbit"
        );

        let zero = Vector3::zeros();
        let hill = hill_point.att_ref_out_msg.read();
        assert_same_reference(
            &hill,
            &compute_hill_pointing_reference(truth.position_m, truth.velocity_mps, zero, zero),
        );
        assert_same_reference(
            &velocity_point.att_ref_out_msg.read(),
            &velocity_point.compute_velocity_pointing_reference(
                truth.position_m,
                truth.velocity_mps,
                zero,
                zero,
            ),
        );

        let dcm_rn = mrp_to_dcm(hill.sigma_rn);
        let radial = truth.position_m.normalize();
        let orbit_normal = truth.position_m.cross(&truth.velocity_mps).normalize();
        let mean_motion_radps = (MU_EARTH_M3PS2 / radius_m.powi(3)).sqrt();
        assert!((dcm_rn.row(0).transpose() - radial).norm() < 1.0e-12);
        assert!((hill.omega_rn_n_radps - mean_motion_radps * orbit_normal).norm() < 1.0e-9);
    }

    /// A body force on the spacecraft accumulates ΔV in its state message,
    /// which the navigation forwards, so `dvExecuteGuidance` cuts the burn
    /// once the ΔV since its start time exceeds the command: 0.1 m/s² from
    /// 2 s exceeds 0.955 m/s at 11.6 s.
    #[test]
    fn accumulated_dv_ends_a_dv_execute_guidance_burn() {
        use std::any::Any;

        use crate::fsw_algorithms::dv_guidance::dv_execute_guidance::{
            BurnState, DvExecuteGuidance, DvExecuteGuidanceConfig,
        };
        use crate::messages::DvBurnCommandMsg;
        use crate::spacecraft::{DynamicEffector, EffectorOutput};

        struct BodyForce;
        impl DynamicEffector for BodyForce {
            fn name(&self) -> &str {
                "body_force"
            }
            fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
                EffectorOutput {
                    force_inertial_n: state
                        .body_to_inertial()
                        .transform_vector(&Vector3::new(10.0, 0.0, 0.0)),
                    torque_body_nm: Vector3::zeros(),
                }
            }
            fn as_any(&self) -> &dyn Any {
                self
            }
        }

        const STEP_NANOS: u64 = 100_000_000;
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 100.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::identity(),
            integration_step_nanos: STEP_NANOS,
            initial_position_m: Vector3::zeros(),
            initial_velocity_mps: Vector3::zeros(),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: None,
        });
        spacecraft.add_dynamic_effector(BodyForce);
        let mut navigation = SimpleNavigation::new("simple_navigation");
        let mut guidance = DvExecuteGuidance::new(DvExecuteGuidanceConfig::default());
        let burn = Output::new(DvBurnCommandMsg {
            dv_inertial_cmd_mps: Vector3::new(0.955, 0.0, 0.0),
            burn_start_time_nanos: 2_000_000_000,
            ..Default::default()
        });
        guidance.burn_data_in_msg.connect(burn.slot());

        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        {
            let mut simulation = Simulation::new(epoch, false);
            crate::connect!(&simulation,
                &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
                &navigation.translation_out_msg => &mut guidance.nav_data_in_msg,
            );
            crate::schedule! { simulation,
                "spacecraft" => &mut spacecraft, period = STEP_NANOS, priority = 30;
                "navigation" => &mut navigation, period = STEP_NANOS, priority = 20;
                "guidance" => &mut guidance, period = STEP_NANOS, priority = 10;
            }
            simulation.run_for(11_500_000_000);
        }
        assert_eq!(guidance.burn_state, BurnState::Executing);
        assert!((guidance.dv_init_mps.x - 0.2).abs() < 1.0e-12);
        let translation = navigation.translation_out_msg.read();
        assert!((translation.accumulated_dv_b_mps.x - 1.15).abs() < 1.0e-12);
        assert_eq!(
            translation.accumulated_dv_b_mps,
            spacecraft.state_out.read().accumulated_dv_cn_b_mps
        );

        // One more step: 0.96 m/s since the start exceeds the 0.955 m/s command.
        let next = SimulationContext {
            current_sim_nanos: 11_600_000_000,
            current_epoch: epoch,
        };
        spacecraft.update(&next);
        navigation.update(&next);
        guidance.update(&next);
        assert_eq!(guidance.burn_state, BurnState::Complete);
    }
}
