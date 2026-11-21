//! The accuracy-versus-runtime (work-precision, Pareto) study shared by
//! upstream `scenarioCompareParetoRwPanels.py` and
//! `scenarioCompareParetoFlexPanels.py`: for each engine, every integrator
//! configuration's final hub attitude and position are compared with the
//! engine's own tight reference ([`sweep_engine`]), each configuration is
//! timed with both engines interleaved ([`add_interleaved_timings`]), and
//! the non-dominated configurations form the frontier ([`pareto_frontier`]).
//!
//! A configuration whose propagation diverges (non-finite final state, a
//! panic in the dynamics, or a run that stops short of the horizon) is left
//! off the front, as upstream drops `NaN` runs.

#![allow(dead_code)]

use std::panic::{AssertUnwindSafe, catch_unwind};

use nalgebra::Vector3;
use serde_json::{Value, json};

use super::dyncomp::{
    IntegratorChoice, IntegratorName, Propagation, interleaved_propagation_stats, principal_angle,
    sec2nano, snap_step_to_horizon,
};

/// Upstream `TIMING_TRIALS`, `TIMING_WARMUP_SEC` \[s\].
pub const TIMING_TRIALS: usize = 5;
pub const TIMING_WARMUP_S: f64 = 1.0;

/// One sweep entry, upstream's `(integrator, dt, tol)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SweepConfig {
    pub integrator: IntegratorName,
    /// Task (macro) step \[s\].
    pub dt: f64,
    pub tol: Option<f64>,
}

impl SweepConfig {
    pub const fn new(integrator: IntegratorName, dt: f64, tol: Option<f64>) -> Self {
        Self {
            integrator,
            dt,
            tol,
        }
    }

    pub fn choice(&self) -> IntegratorChoice {
        IntegratorChoice {
            name: self.integrator,
            tol: self.tol,
        }
    }

    fn key(&self) -> (IntegratorName, u64, Option<u64>) {
        (
            self.integrator,
            self.dt.to_bits(),
            self.tol.map(f64::to_bits),
        )
    }

    /// The configuration as upstream's JSON records it.
    pub fn record(&self, requested_dt: Option<f64>) -> Value {
        let mut record = json!({
            "integrator": self.integrator.upstream(),
            "dt": self.dt,
            "tol": self.tol,
        });
        if let Some(requested) = requested_dt {
            record["requestedDt"] = json!(requested);
        }
        record
    }
}

/// The final hub attitude (MRP) and inertial position \[m\] of a run.
#[derive(Clone, Copy, Debug)]
pub struct FinalState {
    pub sigma_bn: Vector3<f64>,
    pub r_bn_n: Vector3<f64>,
}

/// One engine propagated with an integrator, a task step and horizon \[s\]
/// and a recorder flag: the recorded `(times_ns, final state)` (with the
/// recorder) and the propagation seconds.
pub type Engine = fn(IntegratorChoice, f64, f64, bool) -> (Option<(u64, FinalState)>, f64);

/// Upstream `finalState`: the final state at the horizon, `Ok(None)` when
/// the run diverged; an error when `dt` does not divide the horizon.
pub fn final_state(
    engine: Engine,
    config: &SweepConfig,
    horizon_s: f64,
) -> Result<Option<FinalState>, String> {
    let stop = sec2nano(horizon_s);
    let step = sec2nano(config.dt);
    if stop % step != 0 {
        return Err(format!(
            "dt={} s does not divide the {} s comparison horizon",
            config.dt, horizon_s
        ));
    }
    let run = catch_unwind(AssertUnwindSafe(|| {
        engine(config.choice(), config.dt, horizon_s, true).0
    }));
    let Ok(Some((last_nanos, state))) = run else {
        return Ok(None);
    };
    let finite = state
        .sigma_bn
        .iter()
        .chain(state.r_bn_n.iter())
        .all(|v| v.is_finite());
    if last_nanos as i64 != stop || !finite {
        return Ok(None);
    }
    Ok(Some(state))
}

/// Upstream `snappedStep`: the horizon divisor nearest `dt`, reported when
/// it differs.
pub fn snapped_step(dt: f64, horizon_s: f64) -> f64 {
    let snapped = snap_step_to_horizon(dt, horizon_s).unwrap_or_else(|error| panic!("{error}"));
    if snapped != dt {
        println!("dt={dt} s does not divide the {horizon_s} s horizon; using dt={snapped} s");
    }
    snapped
}

/// Upstream `normalizedSweepConfigurations`: the executed configurations
/// (steps snapped onto the horizon) and their JSON records, refusing two
/// requests that map onto one executed configuration.
pub fn normalized_sweep_configurations(
    configurations: &[SweepConfig],
    horizon_s: f64,
) -> Result<(Vec<SweepConfig>, Vec<Value>), String> {
    let mut execution: Vec<SweepConfig> = Vec::new();
    let mut requested: Vec<f64> = Vec::new();
    let mut records = Vec::new();
    for config in configurations {
        let executed = SweepConfig {
            dt: snapped_step(config.dt, horizon_s),
            ..*config
        };
        if let Some(index) = execution.iter().position(|e| e.key() == executed.key()) {
            return Err(format!(
                "requested steps {} s and {} s both map to the same executed {} configuration \
                 at dt={} s and tol={:?}.",
                requested[index],
                config.dt,
                config.integrator.upstream(),
                executed.dt,
                config.tol
            ));
        }
        records.push(executed.record((executed.dt != config.dt).then_some(config.dt)));
        execution.push(executed);
        requested.push(config.dt);
    }
    Ok((execution, records))
}

/// One successful configuration of one engine.
#[derive(Clone, Debug)]
pub struct ParetoRow {
    pub config: SweepConfig,
    /// Upstream `error`: final attitude principal angle to the reference \[rad\].
    pub error: f64,
    /// Upstream `positionError` \[m\].
    pub position_error: f64,
    pub wall: f64,
    pub wall_min: f64,
    pub wall_std: f64,
}

impl ParetoRow {
    pub fn to_json(&self, requested_dt: Option<f64>) -> Value {
        let mut row = self.config.record(requested_dt);
        row["error"] = json!(self.error);
        row["positionError"] = json!(self.position_error);
        row["wall"] = json!(self.wall);
        row["wallMin"] = json!(self.wall_min);
        row["wallStd"] = json!(self.wall_std);
        row
    }
}

/// Upstream `sweepEngine`: the reference, its self-consistency check
/// against `reference_check` (attitude \[rad\], position \[m\]) and every
/// configuration's errors.
pub fn sweep_engine(
    engine: Engine,
    reference: &SweepConfig,
    reference_check: &SweepConfig,
    configurations: &[SweepConfig],
    horizon_s: f64,
) -> (Vec<ParetoRow>, f64, f64) {
    let solve = |config: &SweepConfig| {
        final_state(engine, config, horizon_s).unwrap_or_else(|error| panic!("{error}"))
    };
    let (Some(reference_state), Some(check_state)) = (solve(reference), solve(reference_check))
    else {
        panic!("Pareto reference or reference-check propagation failed.");
    };
    let attitude_floor = principal_angle(check_state.sigma_bn, reference_state.sigma_bn);
    let position_floor = (check_state.r_bn_n - reference_state.r_bn_n).norm();
    let rows = configurations
        .iter()
        .filter_map(|config| {
            let state = solve(config)?;
            Some(ParetoRow {
                config: *config,
                error: principal_angle(state.sigma_bn, reference_state.sigma_bn),
                position_error: (state.r_bn_n - reference_state.r_bn_n).norm(),
                wall: f64::NAN,
                wall_min: f64::NAN,
                wall_std: f64::NAN,
            })
        })
        .collect();
    (rows, attitude_floor, position_floor)
}

/// Upstream `addInterleavedTimings`: each configuration, recorder-free, for
/// both engines in alternating order, after a warm-up of
/// [`TIMING_WARMUP_S`].
pub fn add_interleaved_timings(
    engines: &[Engine],
    rows: &mut [&mut Vec<ParetoRow>],
    horizon_s: f64,
) {
    let mut keys: Vec<SweepConfig> = Vec::new();
    for engine_rows in rows.iter() {
        for row in engine_rows.iter() {
            if !keys.iter().any(|key| key.key() == row.config.key()) {
                keys.push(row.config);
            }
        }
    }
    for key in keys {
        let mut measured: Vec<Propagation<'static>> = Vec::new();
        let mut warmups: Vec<Propagation<'static>> = Vec::new();
        let mut destinations = Vec::new();
        for (index, engine) in engines.iter().enumerate() {
            let Some(row) = rows[index]
                .iter()
                .position(|row| row.config.key() == key.key())
            else {
                continue;
            };
            let engine = *engine;
            let warmup = horizon_s.min(TIMING_WARMUP_S);
            measured.push(Box::new(move || {
                engine(key.choice(), key.dt, horizon_s, false).1
            }));
            warmups.push(Box::new(move || {
                engine(key.choice(), key.dt, warmup, false).1
            }));
            destinations.push((index, row));
        }
        let stats = interleaved_propagation_stats(&mut measured, TIMING_TRIALS, Some(&mut warmups));
        for ((engine, row), stats) in destinations.into_iter().zip(stats) {
            let row = &mut rows[engine][row];
            row.wall = stats.median;
            row.wall_min = stats.min;
            row.wall_std = stats.std;
        }
    }
}

/// Upstream `paretoFrontier`: the configurations no other beats in both
/// wall-clock and error, by increasing wall-clock.
pub fn pareto_frontier(rows: &[ParetoRow], error: fn(&ParetoRow) -> f64) -> Vec<ParetoRow> {
    let mut ordered = rows.to_vec();
    ordered.sort_by(|a, b| {
        a.wall
            .total_cmp(&b.wall)
            .then(error(a).total_cmp(&error(b)))
    });
    let mut best = f64::INFINITY;
    ordered
        .into_iter()
        .filter(|row| {
            let keep = error(row) < best;
            if keep {
                best = error(row);
            }
            keep
        })
        .collect()
}

/// The rows of both engines as a table CSV: one line per configuration and
/// engine (`engine` 0 = BSM, 1 = MuJoCo), the integrator as its index in
/// upstream's `Euler, RK2, RK4, RKF45, RKF78` order, `tol` NaN for the
/// fixed-step ones.
pub fn table(bsm: &[ParetoRow], mujoco: &[ParetoRow]) -> (Vec<String>, Vec<Vec<f64>>) {
    let headers = [
        "engine",
        "integrator",
        "dt_s",
        "tol",
        "error_rad",
        "position_error_m",
        "wall_s",
        "wall_min_s",
        "wall_std_s",
    ]
    .map(String::from)
    .to_vec();
    let index = |name: IntegratorName| match name {
        IntegratorName::Euler => 0.0,
        IntegratorName::Rk2 => 1.0,
        IntegratorName::Rk4 => 2.0,
        IntegratorName::Rkf45 => 3.0,
        IntegratorName::Rkf78 => 4.0,
    };
    let rows = bsm
        .iter()
        .map(|row| (0.0, row))
        .chain(mujoco.iter().map(|row| (1.0, row)))
        .map(|(engine, row)| {
            vec![
                engine,
                index(row.config.integrator),
                row.config.dt,
                row.config.tol.unwrap_or(f64::NAN),
                row.error,
                row.position_error,
                row.wall,
                row.wall_min,
                row.wall_std,
            ]
        })
        .collect();
    (headers, rows)
}

/// Parses a `--configs` list, `RK4:0.1,Euler:0.02,RKF45:0.5:1e-7`.
pub fn parse_configs(text: &str) -> Result<Vec<SweepConfig>, String> {
    text.split(',')
        .map(|entry| {
            let fields: Vec<&str> = entry.trim().split(':').collect();
            let integrator = match fields.first().copied() {
                Some("Euler") => IntegratorName::Euler,
                Some("RK2") => IntegratorName::Rk2,
                Some("RK4") => IntegratorName::Rk4,
                Some("RKF45") => IntegratorName::Rkf45,
                Some("RKF78") => IntegratorName::Rkf78,
                other => return Err(format!("unknown integrator {other:?}")),
            };
            let number = |field: Option<&&str>| -> Result<f64, String> {
                field
                    .ok_or_else(|| format!("{entry:?} needs a step"))?
                    .parse()
                    .map_err(|_| format!("invalid number in {entry:?}"))
            };
            let dt = number(fields.get(1))?;
            let tol = match fields.get(2) {
                Some(_) => Some(number(fields.get(2))?),
                None => None,
            };
            Ok(SweepConfig::new(integrator, dt, tol))
        })
        .collect()
}
