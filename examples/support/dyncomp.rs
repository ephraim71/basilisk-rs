//! Pieces shared by the dynamics-engine comparison examples, the ports of
//! upstream `examples/dynamicsComparison` (`scenario_compare_*`, `sweep_*`):
//!
//! * the history checks of upstream `_comparisonValidation.py`
//!   ([`validate_history`], [`validate_matching_histories`],
//!   [`validate_task_horizon`], [`recorder_sample_interval`],
//!   [`aligned_horizon`], [`require_equal_length`], [`snap_step_to_horizon`]),
//!   with upstream's error messages;
//! * the propagation timing of upstream `_runtimeTable.py`
//!   ([`interleaved_propagation_stats`], [`paired_propagation_times`],
//!   [`median_propagation_time`]) and its runtime CSV
//!   ([`save_runtime_table`]);
//! * the principal angle of the relative attitude every comparison reports,
//!   `4 atan |σ_rel|` ([`principal_angle`]);
//! * the JSON summaries upstream writes to `results/`, here
//!   `examples/output/scenarios/<name>.json` ([`write_json`]);
//! * with the `mujoco` feature, upstream's `gravFactory.addBodiesTo(scene)`
//!   ([`add_earth_gravity_to_scene`]) and a scene whose initial state is set
//!   after its initialization ([`PreparedScene`]).
//!
//! The upstream engines and their basilisk-rs counterparts:
//!
//! | upstream | here |
//! |---|---|
//! | `spacecraft.Spacecraft` (the Backsubstitution Method, "BSM") | [`basilisk_rs::spacecraft::Spacecraft`] |
//! | `mujoco.MJScene` | [`basilisk_rs::mujoco_dynamics::MjScene`] (`mujoco` feature) |
//! | `svIntegratorRK4`, `svIntegratorEuler`, `svIntegratorRK2` | `Rk4`, `Euler`, `Heun` |
//! | `svIntegratorRKF45`, `svIntegratorRKF78` (`relTol`, `absTol`) | `Rkf45`, `Rkf78` with an `AdaptiveController` |
//!
//! ```ignore
//! #[path = "support/dyncomp.rs"]
//! mod dyncomp;
//! ```

#![allow(dead_code)]

use std::path::PathBuf;
use std::time::Instant;

use basilisk_rs::simulation::Simulation;
use basilisk_rs::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_dcm};
use nalgebra::Vector3;

use super::common::scenario_output_path;

/// Upstream `macros.NANO2SEC`.
pub const NANO2SEC: f64 = 1.0e-9;

/// Upstream `macros.sec2nano`: `int(t * 1e9 + 0.5)` for a non-negative time.
pub fn sec2nano(seconds: f64) -> i64 {
    (seconds * 1.0e9 + 0.5).floor() as i64
}

/// [`sec2nano`] for a scheduling period or stop time.
pub fn sec2nano_u64(seconds: f64) -> u64 {
    u64::try_from(sec2nano(seconds)).expect("non-negative time")
}

/// Sample times \[s\] of a recorder's `times()` \[ns\].
pub fn seconds(times_nanos: &[u64]) -> Vec<f64> {
    times_nanos
        .iter()
        .map(|nanos| *nanos as f64 * NANO2SEC)
        .collect()
}

// ---------------------------------------------------------------------------
// `_comparisonValidation.py`
// ---------------------------------------------------------------------------

fn gcd(a: i64, b: i64) -> i64 {
    if b == 0 { a } else { gcd(b, a % b) }
}

/// Upstream `recorderSampleInterval`: the first integer multiple of the task
/// step not shorter than the requested recorder interval \[s\].
pub fn recorder_sample_interval(task_step_s: f64, requested_s: f64) -> Result<f64, String> {
    let task = sec2nano(task_step_s);
    let requested = sec2nano(requested_s);
    if task <= 0 || requested <= 0 {
        return Err("Recorder task and requested intervals must be positive.".to_string());
    }
    let multiple = (requested + task - 1) / task;
    Ok((multiple * task) as f64 * NANO2SEC)
}

/// [`recorder_sample_interval`] as a scheduling period \[ns\].
pub fn recorder_period_nanos(task_step_s: f64, requested_s: f64) -> u64 {
    sec2nano_u64(recorder_sample_interval(task_step_s, requested_s).expect("positive intervals"))
}

/// Upstream `alignedHorizon`: the largest horizon, no longer than the
/// request, that every interval divides in integer nanoseconds \[s\].
pub fn aligned_horizon(requested_s: f64, intervals_s: &[f64]) -> Result<f64, String> {
    let horizon = sec2nano(requested_s);
    let intervals: Vec<i64> = intervals_s
        .iter()
        .map(|interval| sec2nano(*interval))
        .collect();
    if horizon <= 0 || intervals.is_empty() {
        return Err(
            "The requested horizon and at least one interval must be positive.".to_string(),
        );
    }
    if intervals.iter().any(|interval| *interval <= 0) {
        return Err("Every alignment interval must be positive.".to_string());
    }
    let quantum = intervals
        .iter()
        .fold(1_i64, |lcm, interval| lcm / gcd(lcm, *interval) * interval);
    let aligned = horizon / quantum * quantum;
    if aligned == 0 {
        return Err(format!(
            "The requested {requested_s} s horizon is shorter than the {} s common scheduling \
             quantum.",
            quantum as f64 * NANO2SEC
        ));
    }
    Ok(aligned as f64 * NANO2SEC)
}

/// Upstream `validateTaskHorizon`: the requested stop epoch must lie on the
/// periodic task schedule.
pub fn validate_task_horizon(name: &str, final_s: f64, task_s: f64) -> Result<f64, String> {
    let final_nanos = sec2nano(final_s);
    let task = sec2nano(task_s);
    if final_nanos <= 0 || task <= 0 {
        return Err(format!(
            "{name} final time and task interval must both be positive."
        ));
    }
    if final_nanos % task != 0 {
        let reached = final_nanos / task * task;
        return Err(format!(
            "{name} requested final time {} s does not lie on the {} s task schedule; the last \
             executable epoch is {} s.",
            final_nanos as f64 * NANO2SEC,
            task as f64 * NANO2SEC,
            reached as f64 * NANO2SEC
        ));
    }
    Ok(final_nanos as f64 * NANO2SEC)
}

/// One named history checked against the timestamps: its sample count and
/// whether every value is finite (upstream's `**histories` arrays).
#[derive(Clone, Debug)]
pub struct Series {
    pub name: String,
    pub len: usize,
    pub finite: bool,
}

impl Series {
    pub fn scalars(name: &str, values: &[f64]) -> Self {
        Self {
            name: name.to_string(),
            len: values.len(),
            finite: values.iter().all(|value| value.is_finite()),
        }
    }

    pub fn vectors(name: &str, values: &[Vector3<f64>]) -> Self {
        Self {
            name: name.to_string(),
            len: values.len(),
            finite: values
                .iter()
                .all(|value| value.iter().all(|entry| entry.is_finite())),
        }
    }

    pub fn rows(name: &str, values: &[Vec<f64>]) -> Self {
        Self {
            name: name.to_string(),
            len: values.len(),
            finite: values
                .iter()
                .all(|row| row.iter().all(|entry| entry.is_finite())),
        }
    }
}

/// Upstream `validateHistory`: strictly increasing finite timestamps \[s\],
/// histories of matching length with finite values, and, with
/// `expected_final_s`, the complete recorder schedule up to the horizon
/// (`sample_interval_s > 0`) or a last sample exactly at it.
pub fn validate_history(
    name: &str,
    times_s: &[f64],
    expected_final_s: Option<f64>,
    sample_interval_s: f64,
    require_final_sample: bool,
    histories: &[Series],
) -> Result<Vec<f64>, String> {
    if times_s.is_empty() {
        return Err(format!(
            "{name} must contain a nonempty one-dimensional timestamp history."
        ));
    }
    if !times_s.iter().all(|time| time.is_finite()) {
        return Err(format!("{name} contains non-finite timestamps."));
    }
    if times_s.windows(2).any(|pair| pair[1] - pair[0] <= 0.0) {
        return Err(format!("{name} timestamps must be strictly increasing."));
    }
    for series in histories {
        if series.len != times_s.len() {
            return Err(format!(
                "{name} {} has {} samples; expected {}.",
                series.name,
                series.len,
                times_s.len()
            ));
        }
        if !series.finite {
            return Err(format!(
                "{name} {} contains non-finite values.",
                series.name
            ));
        }
    }
    if let Some(expected_final_s) = expected_final_s {
        let final_nanos = sec2nano(expected_final_s);
        let interval = sec2nano(sample_interval_s);
        let actual: Vec<i64> = times_s
            .iter()
            .map(|time| (time / NANO2SEC).round() as i64)
            .collect();
        if interval > 0 {
            if require_final_sample && final_nanos % interval != 0 {
                return Err(format!(
                    "{name} requested final time {} s is not divisible by the {} s sample \
                     interval.",
                    final_nanos as f64 * NANO2SEC,
                    interval as f64 * NANO2SEC
                ));
            }
            let expected_count = final_nanos / interval + 1;
            let complete = actual.len() as i64 == expected_count
                && actual
                    .iter()
                    .enumerate()
                    .all(|(index, nanos)| *nanos == index as i64 * interval);
            if !complete {
                return Err(format!(
                    "{name} ended at {} s with {} samples; expected {} s and {expected_count} \
                     samples on a {} s schedule.",
                    times_s[times_s.len() - 1],
                    actual.len(),
                    ((expected_count - 1) * interval) as f64 * NANO2SEC,
                    interval as f64 * NANO2SEC
                ));
            }
        } else if actual[actual.len() - 1] != final_nanos {
            return Err(format!(
                "{name} ended at {} s, not the requested {} s.",
                times_s[times_s.len() - 1],
                final_nanos as f64 * NANO2SEC
            ));
        }
    }
    Ok(times_s.to_vec())
}

/// Upstream `validateMatchingHistories`: two complete, identical timestamp
/// histories.
pub fn validate_matching_histories(
    name: &str,
    first_s: &[f64],
    second_s: &[f64],
    expected_final_s: Option<f64>,
    sample_interval_s: f64,
    require_final_sample: bool,
) -> Result<Vec<f64>, String> {
    let first = validate_history(
        &format!("{name} first"),
        first_s,
        expected_final_s,
        sample_interval_s,
        require_final_sample,
        &[],
    )?;
    let second = validate_history(
        &format!("{name} second"),
        second_s,
        expected_final_s,
        sample_interval_s,
        require_final_sample,
        &[],
    )?;
    if first != second {
        return Err(format!(
            "{name} timestamp histories differ: {} versus {} samples.",
            first.len(),
            second.len()
        ));
    }
    Ok(first)
}

/// Upstream `requireEqualLength`: the common sample count.
pub fn require_equal_length(name: &str, lengths: &[usize]) -> Result<usize, String> {
    let Some(first) = lengths.first() else {
        return Err(format!("{name} requires at least one history."));
    };
    if lengths.iter().any(|length| length != first) {
        return Err(format!("{name} history lengths differ: {lengths:?}."));
    }
    Ok(*first)
}

/// Upstream `snapStepToHorizon`: the integer-nanosecond divisor of the
/// horizon nearest the requested step (the smaller on a tie) \[s\].
pub fn snap_step_to_horizon(dt_s: f64, horizon_s: f64) -> Result<f64, String> {
    let step = sec2nano(dt_s);
    let horizon = sec2nano(horizon_s);
    if step <= 0 || horizon <= 0 {
        return Err(format!(
            "step {dt_s} s and horizon {horizon_s} s must both be positive."
        ));
    }
    if step > horizon {
        return Err(format!(
            "step {dt_s} s exceeds the {horizon_s} s comparison horizon."
        ));
    }
    if horizon % step == 0 {
        return Ok(dt_s);
    }
    let mut best: Option<i64> = None;
    let mut candidate = 1_i64;
    while candidate * candidate <= horizon {
        if horizon % candidate == 0 {
            for divisor in [candidate, horizon / candidate] {
                let better = match best {
                    None => true,
                    Some(current) => {
                        let (d, c) = ((divisor - step).abs(), (current - step).abs());
                        d < c || (d == c && divisor < current)
                    }
                };
                if better {
                    best = Some(divisor);
                }
            }
        }
        candidate += 1;
    }
    Ok(best.expect("the horizon divides itself") as f64 * NANO2SEC)
}

// ---------------------------------------------------------------------------
// `_runtimeTable.py`
// ---------------------------------------------------------------------------

/// Upstream `DEFAULT_TIMING_REPEATS`: measured trials after one discarded
/// warm-up.
pub const DEFAULT_TIMING_REPEATS: usize = 5;

/// A propagation to time: builds a fresh simulation and returns the seconds
/// its propagation alone took ([`timed_run`]), upstream's `build` callable
/// followed by the timed `ExecuteSimulation()`.
pub type Propagation<'a> = Box<dyn FnMut() -> f64 + 'a>;

/// Initializes `simulation` (model setup, outside the timing) and returns
/// the wall-clock seconds of running it to `stop_nanos`.
pub fn timed_run(simulation: &mut Simulation<'_>, stop_nanos: u64) -> f64 {
    simulation.initialize();
    let start = Instant::now();
    let now = simulation.current_sim_nanos();
    simulation.run_for(stop_nanos.saturating_sub(now));
    start.elapsed().as_secs_f64()
}

/// Median, minimum and population standard deviation of measured
/// propagations \[s\].
#[derive(Clone, Copy, Debug, serde::Serialize)]
pub struct TimingStats {
    pub median: f64,
    pub min: f64,
    pub std: f64,
}

impl TimingStats {
    fn of(samples: &[f64]) -> Self {
        let mut sorted = samples.to_vec();
        sorted.sort_by(f64::total_cmp);
        let n = sorted.len();
        let median = if n % 2 == 1 {
            sorted[n / 2]
        } else {
            0.5 * (sorted[n / 2 - 1] + sorted[n / 2])
        };
        let mean = sorted.iter().sum::<f64>() / n as f64;
        let variance = sorted.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64;
        Self {
            median,
            min: sorted[0],
            std: variance.sqrt(),
        }
    }
}

/// Upstream `medianPropagationTime`: one discarded warm-up, then the median
/// of `repeats` propagations \[s\].
pub fn median_propagation_time(mut build: impl FnMut() -> f64, repeats: usize) -> f64 {
    build();
    let samples: Vec<f64> = (0..repeats.max(1)).map(|_| build()).collect();
    TimingStats::of(&samples).median
}

/// Upstream `interleavedPropagationStats`: one discarded warm-up per
/// builder (`warmups` when given, a shorter horizon), then `repeats` rounds
/// alternating forward and reverse builder order.
pub fn interleaved_propagation_stats(
    builders: &mut [Propagation<'_>],
    repeats: usize,
    warmups: Option<&mut [Propagation<'_>]>,
) -> Vec<TimingStats> {
    if builders.is_empty() {
        return Vec::new();
    }
    match warmups {
        Some(warmups) => {
            assert_eq!(
                warmups.len(),
                builders.len(),
                "warmupBuilders must match builders in length."
            );
            for warmup in warmups.iter_mut() {
                warmup();
            }
        }
        None => {
            for build in builders.iter_mut() {
                build();
            }
        }
    }
    let count = builders.len();
    let mut samples = vec![Vec::new(); count];
    for trial in 0..repeats.max(1) {
        let order: Vec<usize> = if trial % 2 == 0 {
            (0..count).collect()
        } else {
            (0..count).rev().collect()
        };
        for index in order {
            samples[index].push(builders[index]());
        }
    }
    samples
        .iter()
        .map(|values| TimingStats::of(values))
        .collect()
}

/// Upstream `pairedPropagationTimes`: interleaved median BSM and MuJoCo
/// propagation times \[s\].
pub fn paired_propagation_times(
    bsm: Propagation<'_>,
    mujoco: Propagation<'_>,
    repeats: usize,
) -> (f64, f64) {
    let mut builders = [bsm, mujoco];
    let stats = interleaved_propagation_stats(&mut builders, repeats, None);
    (stats[0].median, stats[1].median)
}

/// One row of a runtime table: the case, the BSM seconds and, when MuJoCo
/// ran, its seconds.
pub type RuntimeRow = (String, f64, Option<f64>);

fn csv_field(text: &str) -> String {
    if text.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", text.replace('"', "\"\""))
    } else {
        text.to_string()
    }
}

/// Upstream `saveRuntimeTable`: `examples/output/scenarios/<scenario>_runtime.csv`
/// with the case, BSM and MuJoCo seconds and the BSM/MuJoCo speedup (`n/a`
/// without MuJoCo).
pub fn save_runtime_table(scenario: &str, rows: &[RuntimeRow], case_header: &str) -> PathBuf {
    let mut text = format!(
        "{},BSM [s],MuJoCo [s],Speedup (BSM/MuJoCo)\n",
        csv_field(case_header)
    );
    for (case, bsm, mujoco) in rows {
        let (mujoco_cell, speedup) = match mujoco {
            Some(mujoco) if *mujoco > 0.0 => {
                (format!("{mujoco:.4}"), format!("{:.2}", bsm / mujoco))
            }
            _ => ("n/a".to_string(), "n/a".to_string()),
        };
        text.push_str(&format!(
            "{},{bsm:.4},{mujoco_cell},{speedup}\n",
            csv_field(case)
        ));
    }
    let path = scenario_output_path(&format!("{scenario}_runtime.csv"));
    std::fs::write(&path, text)
        .unwrap_or_else(|error| panic!("cannot write {}: {error}", path.display()));
    path
}

// ---------------------------------------------------------------------------
// Attitude comparison, results
// ---------------------------------------------------------------------------

/// The principal rotation angle between two MRP attitudes, from the relative
/// DCM `[A][B]^T` as `4 atan |σ_rel|` \[rad\]: upstream's
/// `relativePrincipalAngle` for one sample, well conditioned down to machine
/// precision, unlike `arccos((tr - 1)/2)`.
pub fn principal_angle(sigma_a: Vector3<f64>, sigma_b: Vector3<f64>) -> f64 {
    let relative = mrp_to_dcm(sigma_a) * mrp_to_dcm(sigma_b).transpose();
    4.0 * dcm_to_mrp(relative).norm().atan()
}

/// Upstream `relativePrincipalAngle`: [`principal_angle`] per sample of two
/// equally long attitude histories \[rad\].
pub fn relative_principal_angles(
    name: &str,
    sigma_a: &[Vector3<f64>],
    sigma_b: &[Vector3<f64>],
) -> Vec<f64> {
    require_equal_length(name, &[sigma_a.len(), sigma_b.len()]).unwrap_or_else(|e| panic!("{e}"));
    sigma_a
        .iter()
        .zip(sigma_b)
        .map(|(a, b)| principal_angle(*a, *b))
        .collect()
}

/// The largest value \[same unit\].
pub fn max_of(values: impl IntoIterator<Item = f64>) -> f64 {
    values.into_iter().fold(f64::NEG_INFINITY, f64::max)
}

/// Per-sample `|a - b|` of two equally long vector histories.
pub fn vector_differences(a: &[Vector3<f64>], b: &[Vector3<f64>]) -> Vec<f64> {
    assert_eq!(a.len(), b.len(), "history lengths differ");
    a.iter().zip(b).map(|(a, b)| (a - b).norm()).collect()
}

/// Writes a pretty-printed (indent 2) JSON summary to
/// `examples/output/scenarios/<stem>.json`, upstream's
/// `results/<stem>.json`.
pub fn write_json(stem: &str, value: &serde_json::Value) -> PathBuf {
    let path = scenario_output_path(&format!("{stem}.json"));
    let text = serde_json::to_string_pretty(value).expect("serializable summary");
    std::fs::write(&path, text + "\n")
        .unwrap_or_else(|error| panic!("cannot write {}: {error}", path.display()));
    path
}

/// The columns `<prefix>_0`, `<prefix>_1`, `<prefix>_2` of a table CSV.
pub fn vector_headers(prefix: &str) -> [String; 3] {
    [0, 1, 2].map(|index| format!("{prefix}_{index}"))
}

// ---------------------------------------------------------------------------
// Integrators
// ---------------------------------------------------------------------------

/// Upstream's `svIntegrators` integrator classes the comparisons sweep.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum IntegratorName {
    Euler,
    Rk2,
    Rk4,
    Rkf45,
    Rkf78,
}

impl IntegratorName {
    /// The upstream class name, which the JSON rows record.
    pub fn upstream(self) -> &'static str {
        match self {
            Self::Euler => "svIntegratorEuler",
            Self::Rk2 => "svIntegratorRK2",
            Self::Rk4 => "svIntegratorRK4",
            Self::Rkf45 => "svIntegratorRKF45",
            Self::Rkf78 => "svIntegratorRKF78",
        }
    }

    pub fn from_upstream(name: &str) -> Option<Self> {
        [Self::Euler, Self::Rk2, Self::Rk4, Self::Rkf45, Self::Rkf78]
            .into_iter()
            .find(|candidate| candidate.upstream() == name)
    }

    pub fn is_adaptive(self) -> bool {
        matches!(self, Self::Rkf45 | Self::Rkf78)
    }
}

/// An integrator and, for the adaptive ones, upstream's `relTol = absTol =
/// tol` (upstream `makeIntegrator`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IntegratorChoice {
    pub name: IntegratorName,
    pub tol: Option<f64>,
}

impl IntegratorChoice {
    pub const RK4: Self = Self {
        name: IntegratorName::Rk4,
        tol: None,
    };

    /// The adaptive step-size controller at `tol` (both tolerances), with a
    /// sub-step cap high enough for the tightest reference tolerances.
    pub fn controller(self) -> basilisk_rs::integrators::AdaptiveController {
        let tol = self.tol.unwrap_or(1.0e-9);
        basilisk_rs::integrators::AdaptiveController {
            atol: tol,
            rtol: tol,
            max_substeps: 10_000_000,
            ..Default::default()
        }
    }

    /// The integrator of a [`DynamicObject`](basilisk_rs::integrators::DynamicObject)
    /// (a spacecraft or a scene).
    pub fn integrator<D>(self) -> Box<dyn basilisk_rs::integrators::Integrator<D>>
    where
        D: basilisk_rs::integrators::DynamicObject + 'static,
    {
        use basilisk_rs::integrators::{Euler, Heun, Rk4, Rkf45, Rkf78};
        match self.name {
            IntegratorName::Euler => Box::new(Euler),
            IntegratorName::Rk2 => Box::new(Heun),
            IntegratorName::Rk4 => Box::new(Rk4),
            IntegratorName::Rkf45 => Box::new(Rkf45::new(self.controller())),
            IntegratorName::Rkf78 => Box::new(Rkf78::new(self.controller())),
        }
    }
}

// ---------------------------------------------------------------------------
// MuJoCo side
// ---------------------------------------------------------------------------

#[cfg(feature = "mujoco")]
pub use scene::*;

#[cfg(feature = "mujoco")]
mod scene {
    use basilisk_rs::dynamics::gravity::PointMassGravityModel;
    use basilisk_rs::mujoco_dynamics::MjScene;
    use basilisk_rs::mujoco_dynamics::n_body_gravity::NBodyGravity;
    use basilisk_rs::{Module, SimulationContext};

    /// Upstream `gravFactory.createEarth()` with `isCentralBody = True`, then
    /// `gravFactory.addBodiesTo(scene)`: an `NBodyGravity` with the point-mass
    /// Earth as its central source and every scene body as a target, in the
    /// scene's dynamics task.
    pub fn add_earth_gravity_to_scene(scene: &mut MjScene, mu_m3ps2: f64) {
        let mut gravity = NBodyGravity::new(format!("{}_gravity", scene.name));
        gravity.add_gravity_source(
            "earth",
            PointMassGravityModel::new(mu_m3ps2).expect("positive mu"),
            true,
        );
        gravity.add_gravity_targets_for_all_bodies(scene);
        scene.add_model_to_dynamics_task(gravity, -1);
    }

    type Setup = Box<dyn FnOnce(&mut MjScene) + Send>;

    /// A scene whose free-body and joint states are set right after its
    /// initialization: upstream's `InitializeSimulation()` followed by
    /// `body.setPosition(...)`, `setAttitudeRate(...)`,
    /// `getScalarJoint(...).setPosition(...)`.
    pub struct PreparedScene {
        pub scene: MjScene,
        setup: Option<Setup>,
    }

    impl PreparedScene {
        pub fn new(scene: MjScene, setup: impl FnOnce(&mut MjScene) + Send + 'static) -> Self {
            Self {
                scene,
                setup: Some(Box::new(setup)),
            }
        }
    }

    impl Module for PreparedScene {
        fn init(&mut self) {
            self.scene.init();
            if let Some(setup) = self.setup.take() {
                setup(&mut self.scene);
            }
        }

        fn reset(&mut self, context: &SimulationContext) {
            self.scene.reset(context);
        }

        fn update(&mut self, context: &SimulationContext) {
            self.scene.update(context);
        }
    }
}

#[cfg(test)]
mod tests {
    //! `test_scenarioDynamicsComparison.py`'s checks of
    //! `_comparisonValidation`.

    use super::*;

    fn err(result: Result<Vec<f64>, String>) -> String {
        result.expect_err("validation should fail")
    }

    /// `test_history_validation_rejects_mismatch_and_short_run`.
    #[test]
    fn history_validation_rejects_mismatch_and_short_run() {
        assert!(
            err(validate_matching_histories(
                "test",
                &[0.0, 0.4, 1.0],
                &[0.0, 0.5, 1.0],
                Some(1.0),
                0.0,
                false
            ))
            .contains("timestamp histories differ")
        );
        assert!(
            err(validate_history(
                "test",
                &[0.0, 0.5],
                Some(1.0),
                0.5,
                false,
                &[]
            ))
            .contains("ended at")
        );
        // `values=np.zeros((1, 3))`: one sample for two timestamps.
        assert!(
            err(validate_history(
                "test",
                &[0.0, 1.0],
                None,
                0.0,
                false,
                &[Series::vectors("values", &[Vector3::zeros()])]
            ))
            .contains("samples")
        );
        assert!(
            err(validate_history(
                "test",
                &[0.0, 1.0],
                None,
                0.0,
                false,
                &[Series::scalars("values", &[0.0, f64::NAN])]
            ))
            .contains("non-finite values")
        );
    }

    /// `test_history_validation_rejects_invalid_timestamps`, every case but
    /// the two-dimensional `[[0.0, 1.0]]`, which a slice cannot express.
    #[test]
    fn history_validation_rejects_invalid_timestamps() {
        for (times, message) in [
            (vec![], "nonempty one-dimensional"),
            (vec![0.0, f64::NAN], "non-finite timestamps"),
            (vec![0.0, 0.5, 0.5], "strictly increasing"),
            (vec![0.0, 0.6, 0.5], "strictly increasing"),
        ] {
            let error = err(validate_history("test", &times, None, 0.0, false, &[]));
            assert!(error.contains(message), "{times:?}: {error}");
        }
    }

    /// `test_history_validation_accepts_complete_schedule_and_equal_lengths`.
    #[test]
    fn history_validation_accepts_complete_schedule_and_equal_lengths() {
        assert!((validate_task_horizon("test", 1.0, 0.2).unwrap() - 1.0).abs() < 1e-12);
        assert!(
            validate_task_horizon("test", 0.25, 0.2)
                .unwrap_err()
                .contains("does not lie on")
        );
        assert!((recorder_sample_interval(0.2, 0.1).unwrap() - 0.2).abs() < 1e-12);
        assert!((recorder_sample_interval(40.0, 60.0).unwrap() - 80.0).abs() < 1e-9);
        assert!((aligned_horizon(11.7, &[0.4, 0.6]).unwrap() - 10.8).abs() < 1e-9);
        let times = validate_history(
            "test",
            &[0.0, 0.5, 1.0],
            Some(1.0),
            0.5,
            false,
            &[Series::rows(
                "values",
                &[vec![0.0; 2], vec![0.0; 2], vec![0.0; 2]],
            )],
        )
        .unwrap();
        assert_eq!(times, vec![0.0, 0.5, 1.0]);
        assert_eq!(require_equal_length("test", &[2, 2]).unwrap(), 2);
        assert!(
            require_equal_length("test", &[])
                .unwrap_err()
                .contains("at least one history")
        );
        assert!(
            require_equal_length("test", &[1, 2])
                .unwrap_err()
                .contains("history lengths differ")
        );
    }

    /// `test_pareto_rw_requires_common_comparison_horizon`'s `snappedStep`:
    /// 0.8 s snaps to 0.75 s on a 30 s horizon.
    #[test]
    fn step_snaps_to_the_nearest_horizon_divisor() {
        assert!((snap_step_to_horizon(0.8, 30.0).unwrap() - 0.75).abs() < 1e-12);
        assert_eq!(snap_step_to_horizon(0.5, 30.0).unwrap(), 0.5);
        assert!(
            snap_step_to_horizon(40.0, 30.0)
                .unwrap_err()
                .contains("exceeds")
        );
    }

    #[test]
    fn principal_angle_is_the_rotation_between_attitudes() {
        let angle = 0.3_f64;
        let sigma = Vector3::new(0.0, 0.0, (angle / 4.0).tan());
        assert!((principal_angle(sigma, Vector3::zeros()) - angle).abs() < 1e-14);
        assert_eq!(principal_angle(sigma, sigma), 0.0);
    }

    #[test]
    fn timing_statistics_match_numpy() {
        let stats = TimingStats::of(&[3.0, 1.0, 2.0, 4.0]);
        assert_eq!(stats.median, 2.5);
        assert_eq!(stats.min, 1.0);
        assert!((stats.std - 1.25_f64.sqrt()).abs() < 1e-15);
    }
}
