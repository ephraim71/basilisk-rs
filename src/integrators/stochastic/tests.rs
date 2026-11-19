//! Tests of the stochastic integrators.
//!
//! Ports of upstream `test_stochasticIntegrators.py`,
//! `test_stochasticIntegratorsJulia.py` and `test_stochasticIntegratorsPaper.py`
//! (the committed reference trajectories are under `reference/`), and of the
//! cases of `test_flatStochastic*.cpp` and `test_flatWeakItoFamilies.cpp` that
//! concern the numerical methods rather than upstream's flat buffers.
//!
//! The toy SDEs upstream drives through a MuJoCo scene and a Python state
//! model are [`Sde`] objects here. Upstream shares every noise source across
//! the scalar states, so source `k` drives each state through the `k`-th
//! diffusion column; an [`Sde`] column is that vector. [`Sde`] sums its
//! weighted derivatives before scaling, in upstream's order, so it reproduces
//! upstream's arithmetic and the one-step snapshots keep their four-ULP
//! tolerance.
//!
//! Not ported, with the reason:
//! - allocation, flat-buffer, registration-order, shared-slot bookkeeping,
//!   generator-subclass dispatch and Python-binding tests: those mechanisms do
//!   not exist here;
//! - rollback after a failed step, and isolation of diffusion side effects:
//!   a step reads `&State` and returns a new one, so neither a panic nor a
//!   diffusion evaluation can change the accepted state;
//! - `DRI1ModeIsFrozenAtBinding`: the non-mixing variant is its own type,
//!   [`Dri1Nm`], so there is no mode to change after first use;
//! - the fixed-seed bit snapshots of `FinalizedTopologyAndFixedSeedBits`: they
//!   pin the Mersenne Twister sequence of the C++ standard library;
//! - `test_legacy_endpoint_map_api_is_removed`: a Python API check.
//!
//! The Monte Carlo tests draw from [`RandomGaussianNoiseGenerator`] (or a
//! seeded `StdRng` for the prescribed Example 1 ensemble) instead of NumPy, so
//! their fixed ensembles differ from upstream's while the statistics and
//! tolerances are upstream's.

use std::sync::{Arc, Mutex};

use hifitime::{Duration, Epoch};
use nalgebra::Matrix2;
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, StandardNormal};
use serde_json::Value;

use super::*;
use crate::integrators::Integrator;

/// A drift or diffusion column `(t, x) -> vector`.
type Field = Box<dyn Fn(f64, &[f64]) -> Vec<f64> + Send>;

/// `dx = f(t, x) dt + Σ_k g_k(t, x) dW_k` over a vector of scalar states.
struct Sde {
    drift: Field,
    diffusion: Vec<Field>,
}

impl Sde {
    fn new(drift: Field, diffusion: Vec<Field>) -> Self {
        Self { drift, diffusion }
    }
}

/// Upstream `FlatStochasticWorkspace::writeDrift` arithmetic: the first term
/// always, later terms only for non-zero weights.
fn upstream_combination<'a>(mut terms: impl Iterator<Item = (&'a [f64], f64)>) -> Option<Vec<f64>> {
    let (first, weight) = terms.next()?;
    let mut combined: Vec<f64> = first.iter().map(|value| value * weight).collect();
    for (term, weight) in terms {
        if weight == 0.0 {
            continue;
        }
        for (sum, value) in combined.iter_mut().zip(term) {
            *sum += value * weight;
        }
    }
    Some(combined)
}

/// `state + combined · step`, upstream's Euclidean candidate update.
fn add_scaled(state: &[f64], combined: Option<Vec<f64>>, step: f64) -> Vec<f64> {
    match combined {
        Some(combined) => state
            .iter()
            .zip(&combined)
            .map(|(value, increment)| value + increment * step)
            .collect(),
        None => state.to_vec(),
    }
}

/// Seconds since the start, as upstream's state model reads them
/// (`NANO2SEC * CurrentSimNanos`).
fn seconds(nanos: u64) -> f64 {
    nanos as f64 * 1.0e-9
}

impl DynamicObject for Sde {
    type State = Vec<f64>;
    type Derivative = Vec<f64>;
    type StepOutput = ();

    fn equations_of_motion(&mut self, state: &Vec<f64>, nanos: u64, _: Epoch) -> Vec<f64> {
        (self.drift)(seconds(nanos), state)
    }

    fn state_with_derivative(&self, state: &Vec<f64>, derivative: &Vec<f64>, dt: f64) -> Vec<f64> {
        add_scaled(state, Some(derivative.clone()), dt)
    }

    fn state_with_derivatives(
        &self,
        state: &Vec<f64>,
        weighted: &[(&Vec<f64>, f64)],
        step: f64,
    ) -> Vec<f64> {
        let combined = upstream_combination(
            weighted
                .iter()
                .map(|&(derivative, weight)| (derivative.as_slice(), weight)),
        );
        add_scaled(state, combined, step)
    }

    fn combine(
        &self,
        state: &Vec<f64>,
        weighted: Vec<(Vec<f64>, f64)>,
        step: f64,
    ) -> (Vec<f64>, ()) {
        let combined = upstream_combination(
            weighted
                .iter()
                .map(|(derivative, weight)| (derivative.as_slice(), *weight)),
        );
        (add_scaled(state, combined, step), ())
    }

    fn merge_step_outputs(&self, _: (), _: ()) {}

    fn noise_source_count(&self) -> usize {
        self.diffusion.len()
    }

    fn equations_of_motion_diffusion(
        &mut self,
        state: &Vec<f64>,
        nanos: u64,
        _: Epoch,
    ) -> Vec<Vec<f64>> {
        self.diffusion
            .iter()
            .map(|column| column(seconds(nanos), state))
            .collect()
    }
}

fn start_epoch() -> Epoch {
    Epoch::from_gregorian_utc_at_midnight(2026, 1, 1)
}

/// Every concrete stochastic integrator, under its upstream short name
/// (upstream `ALL_INTEGRATORS`).
const ALL_METHODS: [&str; 22] = [
    "EulerMaruyama",
    "SRIW1",
    "SOSRI",
    "SRA1",
    "SOSRA",
    "EulerHeun",
    "RKMil",
    "W2Ito1",
    "W2Ito2",
    "DRI1",
    "DRI1NM",
    "RI1",
    "RI3",
    "RI5",
    "RI6",
    "RS1",
    "RS2",
    "SIEA",
    "SMEA",
    "SIEB",
    "SMEB",
    "RDI1WM",
];

/// The integrator named `method`, drawing from `generator`.
fn integrator<D: DynamicObject + 'static>(
    method: &str,
    generator: Arc<Mutex<dyn GaussianNoiseGenerator>>,
) -> Box<dyn Integrator<D>> {
    macro_rules! build {
        ($method:ty) => {
            Box::new(<$method>::new().with_noise_generator(generator))
        };
    }
    match method {
        "EulerMaruyama" => build!(EulerMaruyama),
        "SRIW1" => build!(Sriw1),
        "SOSRI" => build!(Sosri),
        "SRA1" => build!(Sra1),
        "SOSRA" => build!(Sosra),
        "EulerHeun" => build!(EulerHeun),
        "RKMil" => build!(RkMil),
        "W2Ito1" => build!(W2Ito1),
        "W2Ito2" => build!(W2Ito2),
        "DRI1" => build!(Dri1),
        "DRI1NM" => build!(Dri1Nm),
        "RI1" => build!(Ri1),
        "RI3" => build!(Ri3),
        "RI5" => build!(Ri5),
        "RI6" => build!(Ri6),
        "RS1" => build!(Rs1),
        "RS2" => build!(Rs2),
        "SIEA" => build!(Siea),
        "SMEA" => build!(Smea),
        "SIEB" => build!(Sieb),
        "SMEB" => build!(Smeb),
        "RDI1WM" => build!(Rdi1Wm),
        other => panic!("unknown stochastic integrator {other}"),
    }
}

/// A replay queue holding `steps`, and the same queue as a generator handle.
fn prescribed(
    steps: &[(Vec<f64>, Vec<f64>)],
) -> (
    Arc<Mutex<PrescribedGaussianNoiseGenerator>>,
    Arc<Mutex<dyn GaussianNoiseGenerator>>,
) {
    let mut generator = PrescribedGaussianNoiseGenerator::new();
    for (dw, dz) in steps {
        generator.push_step(dw, dz);
    }
    let shared = Arc::new(Mutex::new(generator));
    let dynamic: Arc<Mutex<dyn GaussianNoiseGenerator>> = shared.clone();
    (shared, dynamic)
}

/// Integrates `steps` steps of `dt` from `x0`, like upstream's task loop, and
/// returns every state including the initial one (upstream's logger).
fn trajectory<D: DynamicObject + 'static>(
    integrator: &dyn Integrator<D>,
    object: &mut D,
    x0: D::State,
    dt: f64,
    steps: usize,
) -> Vec<D::State> {
    let dt_nanos = (dt * 1.0e9).round() as u64;
    let mut states = vec![x0.clone()];
    let mut state = x0;
    for step in 0..steps as u64 {
        let nanos = step * dt_nanos;
        let epoch = start_epoch() + Duration::from_total_nanoseconds(nanos as i128);
        state = integrator.propagate(object, &state, nanos, epoch, dt).0;
        states.push(state.clone());
    }
    states
}

// ---------------------------------------------------------------------------
// Reference trajectories (test_stochasticIntegratorsJulia.py and
// test_stochasticIntegratorsPaper.py).
// ---------------------------------------------------------------------------

fn reference_cases(json: &str) -> Vec<Value> {
    let data: Value = serde_json::from_str(json).expect("reference JSON parses");
    data["cases"]
        .as_array()
        .expect("reference JSON has cases")
        .clone()
}

fn vector(value: &Value) -> Vec<f64> {
    value
        .as_array()
        .expect("JSON array")
        .iter()
        .map(|entry| entry.as_f64().expect("JSON number"))
        .collect()
}

fn matrix(value: &Value) -> Vec<Vec<f64>> {
    value
        .as_array()
        .expect("JSON array")
        .iter()
        .map(vector)
        .collect()
}

/// `A · x`.
fn apply(matrix: &[Vec<f64>], x: &[f64]) -> Vec<f64> {
    matrix
        .iter()
        .map(|row| row.iter().zip(x).map(|(a, x)| a * x).sum())
        .collect()
}

/// Upstream `referenceSDE._timeFactor`.
fn time_factor(name: &str) -> fn(f64) -> f64 {
    match name {
        "one" => |_| 1.0,
        "1+t" => |t| 1.0 + t,
        "1+0.5t" => |t| 1.0 + 0.5 * t,
        "0.05_over_sqrt_1pt" => |t| 0.1 * 0.05 / (1.0 + t).sqrt(),
        other => panic!("unknown time factor {other}"),
    }
}

/// Upstream `referenceSDE.makeDrift`.
fn julia_drift(case: &Value) -> Field {
    let spec = &case["drift"];
    match spec["kind"].as_str().expect("drift kind") {
        "linear" => {
            let a = matrix(&spec["matrix"]);
            let factor = time_factor(spec["timeFactor"].as_str().expect("time factor"));
            Box::new(move |t, x| {
                apply(&a, x)
                    .into_iter()
                    .map(|value| factor(t) * value)
                    .collect()
            })
        }
        "ou_additive_td" => {
            Box::new(|t, x| vec![0.05 / (1.0 + t).sqrt() - x[0] / (2.0 * (1.0 + t))])
        }
        other => panic!("unknown drift kind {other}"),
    }
}

/// Upstream `referenceSDE.makeDiffusionColumns`.
fn julia_diffusion(case: &Value) -> Vec<Field> {
    let spec = &case["diffusion"];
    let factor = time_factor(spec["timeFactor"].as_str().expect("time factor"));
    let columns = spec["columns"].as_array().expect("diffusion columns");
    match spec["kind"].as_str().expect("diffusion kind") {
        "state_linear" => columns
            .iter()
            .map(|b| {
                let b = matrix(b);
                Box::new(move |t: f64, x: &[f64]| {
                    apply(&b, x)
                        .into_iter()
                        .map(|value| factor(t) * value)
                        .collect()
                }) as Field
            })
            .collect(),
        "additive" => columns
            .iter()
            .map(|c| {
                let c = vector(c);
                Box::new(move |t: f64, _: &[f64]| c.iter().map(|value| factor(t) * value).collect())
                    as Field
            })
            .collect(),
        other => panic!("unknown diffusion kind {other}"),
    }
}

/// Upstream `test_stochasticIntegratorsPaper._drift`.
fn paper_drift(problem: &str) -> Field {
    if problem == "noncommutative2d" {
        return Box::new(|_, x| vec![1.25 * x[1] - 1.25 * x[0], 0.25 * x[0] - 0.25 * x[1]]);
    }
    if problem == "commutative2d" {
        let av = 299.0 / 200.0;
        return Box::new(move |_, x| vec![av * x[0], av * x[1]]);
    }
    if problem.ends_with("_noncommutative2d") {
        let a = vec![vec![-0.5, 0.2], vec![0.1, -0.4]];
        return Box::new(move |_, x| apply(&a, x));
    }
    if problem.ends_with("_diag2d") {
        return Box::new(|_, x| vec![1.01 * x[0], -0.5 * x[1]]);
    }
    if problem.ends_with("_scalar") {
        return Box::new(|_, x| vec![1.01 * x[0]]);
    }
    panic!("unknown paper problem {problem}")
}

/// Upstream `test_stochasticIntegratorsPaper._diffusionColumns`.
fn paper_diffusion(problem: &str) -> Vec<Field> {
    if problem == "noncommutative2d" {
        return vec![
            Box::new(|_, x| vec![3.0_f64.sqrt() / 2.0 * (x[0] - x[1]), 0.0]),
            Box::new(|_, x| vec![0.5 * (x[0] + x[1]), x[0]]),
        ];
    }
    if problem == "commutative2d" {
        let bv = 0.1;
        return vec![
            Box::new(move |_, x| vec![bv * x[0], 0.0]),
            Box::new(move |_, x| vec![0.0, bv * x[1]]),
        ];
    }
    if problem.ends_with("_noncommutative2d") {
        let b0 = vec![vec![0.3, 0.0], vec![0.0, 0.1]];
        let b1 = vec![vec![0.0, 0.15], vec![0.25, 0.0]];
        return vec![
            Box::new(move |_, x| apply(&b0, x)),
            Box::new(move |_, x| apply(&b1, x)),
        ];
    }
    if problem.ends_with("_diag2d") {
        return vec![
            Box::new(|_, x| vec![0.3 * x[0], 0.0]),
            Box::new(|_, x| vec![0.0, 0.2 * x[1]]),
        ];
    }
    if problem.ends_with("_scalar") {
        return vec![Box::new(|_, x| vec![0.87 * x[0]])];
    }
    panic!("unknown paper problem {problem}")
}

/// Replays one reference case through its integrator and requires the
/// trajectory to match to `atol = 1e-11` (upstream `buildPrescribedNoiseSim`
/// plus the equivalence assertion).
fn assert_reference_case(case: &Value, mut sde: Sde) {
    let method = case["method"].as_str().expect("method");
    let problem = case["problem"].as_str().expect("problem");
    let steps = case["n_steps"].as_u64().expect("n_steps") as usize;
    let noise_steps: Vec<_> = (0..steps)
        .map(|step| (vector(&case["dW"][step]), vector(&case["dZ"][step])))
        .collect();
    let (generator, dynamic) = prescribed(&noise_steps);
    let integrator = integrator::<Sde>(method, dynamic);
    let actual = trajectory(
        integrator.as_ref(),
        &mut sde,
        vector(&case["u0"]),
        case["dt"].as_f64().expect("dt"),
        steps,
    );
    let reference = matrix(&case["trajectory"]);
    assert_eq!(
        actual.len(),
        reference.len(),
        "{method}-{problem}: sample count"
    );
    for (step, (actual, expected)) in actual.iter().zip(&reference).enumerate() {
        for (actual, expected) in actual.iter().zip(expected) {
            assert!(
                (actual - expected).abs() <= 1.0e-11,
                "{method}-{problem}: step {step} gives {actual}, reference {expected}"
            );
        }
    }
    assert_eq!(
        generator.lock().unwrap().remaining(),
        0,
        "{method}-{problem}"
    );
}

/// Upstream `test_juliaEquivalence`: every method reproduces the
/// StochasticDiffEq.jl trajectory driven by the same increments.
#[test]
fn julia_equivalence() {
    let cases = reference_cases(include_str!("reference/julia_reference_trajectories.json"));
    assert_eq!(cases.len(), 47);
    for case in &cases {
        assert_reference_case(case, Sde::new(julia_drift(case), julia_diffusion(case)));
    }
}

/// Upstream `test_paperEquivalence`: RS1/RS2, DRI1/RI and W2Ito reproduce the
/// paper-faithful multi-noise references, including non-commutative noise.
#[test]
fn paper_equivalence() {
    let mut count = 0;
    for json in [
        include_str!("reference/rs_reference_trajectories.json"),
        include_str!("reference/dri1_reference_trajectories.json"),
        include_str!("reference/w2ito_reference_trajectories.json"),
    ] {
        for case in reference_cases(json) {
            let problem = case["problem"].as_str().expect("problem");
            assert_reference_case(
                &case,
                Sde::new(paper_drift(problem), paper_diffusion(problem)),
            );
            count += 1;
        }
    }
    assert_eq!(count, 15);
}

// ---------------------------------------------------------------------------
// test_stochasticIntegrators.py
// ---------------------------------------------------------------------------

/// Upstream `eulerMayuramaIntegrate`: the Python Euler-Maruyama reference,
/// drawing from the same seeded generator as the integrator. Returns the
/// state at each step.
fn euler_maruyama_reference(
    f: &dyn Fn(f64, &[f64]) -> Vec<f64>,
    g: &[Field],
    x0: &[f64],
    dt: f64,
    tf: f64,
    seed: u64,
) -> Vec<Vec<f64>> {
    let steps = (tf / dt).floor() as usize;
    let mut generator = RandomGaussianNoiseGenerator::new(seed);
    let mut x = vec![x0.to_vec()];
    for step in 0..steps {
        let t = dt * step as f64;
        let dw = generator.generate_wiener(g.len(), dt);
        let current = x[step].clone();
        let mut next: Vec<f64> = current
            .iter()
            .zip(f(t, &current))
            .map(|(value, derivative)| value + derivative * dt)
            .collect();
        for (column, dw) in g.iter().zip(&dw) {
            for (value, diffusion) in next.iter_mut().zip(column(t, &current)) {
                *value += diffusion * dw;
            }
        }
        x.push(next);
    }
    x
}

/// Euler-Maruyama trajectory of `sde` with the default generator reseeded.
fn euler_maruyama_run(sde: &mut Sde, x0: &[f64], dt: f64, tf: f64, seed: u64) -> Vec<Vec<f64>> {
    let steps = (tf / dt).round() as usize;
    trajectory(
        &EulerMaruyama::new().with_seed(seed),
        sde,
        x0.to_vec(),
        dt,
        steps,
    )
}

/// Upstream `OrnsteinUhlenbeckSystem` (`dx = θ(μ - x) dt + σ dW`).
struct OrnsteinUhlenbeck {
    x0: f64,
    mu: f64,
    theta: f64,
    sigma: f64,
}

impl Default for OrnsteinUhlenbeck {
    fn default() -> Self {
        Self {
            x0: 0.1,
            mu: 0.0,
            theta: 0.1,
            sigma: 0.01,
        }
    }
}

impl OrnsteinUhlenbeck {
    fn drift(&self) -> Field {
        let (mu, theta) = (self.mu, self.theta);
        Box::new(move |_, x| vec![theta * (mu - x[0])])
    }

    fn diffusion(&self) -> Vec<Field> {
        let sigma = self.sigma;
        vec![Box::new(move |_, _| vec![sigma])]
    }

    fn mean(&self, t: f64) -> f64 {
        self.x0 * (-self.theta * t).exp() + self.mu * (1.0 - (-self.theta * t).exp())
    }

    fn variance(&self, t: f64) -> f64 {
        self.sigma.powi(2) / (2.0 * self.theta) * (1.0 - (-2.0 * self.theta * t).exp())
    }
}

/// Upstream `ComplexOrnsteinUhlenbeckSystem`: two states, two sources.
fn complex_ornstein_uhlenbeck() -> (Field, Vec<Field>, Vec<f64>) {
    let (theta_x, theta_y) = (0.1, 0.073);
    let (sigma_x1, sigma_x2, sigma_y1, sigma_y2) = (0.015, 0.011, 0.0, 0.029);
    (
        Box::new(move |_, x| vec![-theta_x * x[0], -theta_y * x[1]]),
        vec![
            Box::new(move |_, _| vec![sigma_x1, sigma_y1]),
            Box::new(move |_, _| vec![sigma_x2, sigma_y2]),
        ],
        vec![0.1, -0.1],
    )
}

/// Upstream `Example1System` (Tang & Xiao 2017, Example 1).
fn example1() -> (Field, Vec<Field>, Vec<f64>) {
    (
        Box::new(|_, x| {
            vec![
                -273.0 / 512.0 * x[0],
                -1.0 / 160.0 * x[0] - (785.0 / 512.0 - 2.0_f64.sqrt() / 8.0) * x[1],
            ]
        }),
        vec![
            Box::new(|_, x| vec![1.0 / 4.0 * x[0], (1.0 - 2.0 * 2.0_f64.sqrt()) / 4.0 * x[1]]),
            Box::new(|_, x| vec![1.0 / 16.0 * x[0], 1.0 / 10.0 * x[0] + 1.0 / 16.0 * x[1]]),
        ],
        vec![1.0, 1.0],
    )
}

fn assert_final_states_match(actual: &[Vec<f64>], expected: &[Vec<f64>]) {
    let (actual, expected) = (actual.last().unwrap(), expected.last().unwrap());
    for (actual, expected) in actual.iter().zip(expected) {
        assert!(
            (actual - expected).abs() <= 1.0e-12,
            "Euler-Maruyama gives {actual}, reference {expected}"
        );
    }
}

/// Upstream `test_deterministic`: `dx/dt = x t` without noise matches the
/// Python Euler-Maruyama and the analytic `exp(t²/2)` within `0.05`.
#[test]
fn euler_maruyama_deterministic_exponential() {
    let (dt, tf, seed) = (0.01, 1.0, 10);
    let drift = |t: f64, x: &[f64]| vec![t * x[0]];
    let mut sde = Sde::new(Box::new(drift), Vec::new());
    let actual = euler_maruyama_run(&mut sde, &[1.0], dt, tf, seed);
    let expected = euler_maruyama_reference(&drift, &[], &[1.0], dt, tf, seed);
    assert_final_states_match(&actual, &expected);
    let exact = (tf * tf / 2.0).exp();
    assert!((actual.last().unwrap()[0] - exact).abs() <= 0.05);
}

/// Upstream `test_ou`: one Ornstein-Uhlenbeck realization matches the Python
/// Euler-Maruyama with the same seed.
#[test]
fn euler_maruyama_ornstein_uhlenbeck_matches_reference() {
    let system = OrnsteinUhlenbeck::default();
    let mut sde = Sde::new(system.drift(), system.diffusion());
    let actual = euler_maruyama_run(&mut sde, &[system.x0], 1.0, 10.0, 10);
    let expected = euler_maruyama_reference(
        &*system.drift(),
        &system.diffusion(),
        &[system.x0],
        1.0,
        10.0,
        10,
    );
    assert_final_states_match(&actual, &expected);
}

/// Upstream `test_ouComplex`: the coupled two-source process.
#[test]
fn euler_maruyama_complex_ornstein_uhlenbeck_matches_reference() {
    let (drift, diffusion, x0) = complex_ornstein_uhlenbeck();
    let expected = euler_maruyama_reference(&*drift, &diffusion, &x0, 1.0, 10.0, 10);
    let mut sde = Sde::new(drift, diffusion);
    let actual = euler_maruyama_run(&mut sde, &x0, 1.0, 10.0, 10);
    assert_final_states_match(&actual, &expected);
}

/// Upstream `test_example1`: Tang & Xiao Example 1.
#[test]
fn euler_maruyama_example1_matches_reference() {
    let (drift, diffusion, x0) = example1();
    let expected = euler_maruyama_reference(&*drift, &diffusion, &x0, 1.0, 10.0, 10);
    let mut sde = Sde::new(drift, diffusion);
    let actual = euler_maruyama_run(&mut sde, &x0, 1.0, 10.0, 10);
    assert_final_states_match(&actual, &expected);
}

/// Upstream `test_deterministicLimitAllIntegrators`: on `dx/dt = -x` without
/// noise every method is an ODE solver of its drift order. Forward-Euler
/// drift (Euler-Maruyama, RKMil) is within `2e-2` at `h = 1/32`, every other
/// drift tableau within `2e-3`.
#[test]
fn deterministic_limit_of_every_integrator() {
    let (dt, tf, x0): (f64, f64, f64) = (1.0 / 32.0, 1.0, 2.0);
    let expected = x0 * (-tf).exp();
    for method in ALL_METHODS {
        let mut sde = Sde::new(Box::new(|_, x| vec![-x[0]]), Vec::new());
        let generator: Arc<Mutex<dyn GaussianNoiseGenerator>> =
            Arc::new(Mutex::new(RandomGaussianNoiseGenerator::default()));
        let integrator = integrator::<Sde>(method, generator);
        let states = trajectory(integrator.as_ref(), &mut sde, vec![x0], dt, 32);
        let tolerance = if matches!(method, "EulerMaruyama" | "RKMil") {
            2.0e-2
        } else {
            2.0e-3
        };
        let actual = states.last().unwrap()[0];
        assert!(
            (actual - expected).abs() <= tolerance,
            "{method}: deterministic-limit solution {actual}, exact {expected}"
        );
    }
}

/// Upstream `estimateErrorAndEmpiricalVariance` (Tang & Xiao, Section 4):
/// the batch-mean error `μ̂` of `G` and the variance of the batch means.
fn error_and_empirical_variance(
    mut trajectory_end: impl FnMut() -> Vec<f64>,
    g: impl Fn(&[f64]) -> f64,
    expected: f64,
    batches: usize,
    per_batch: usize,
) -> (f64, f64) {
    let mut batch_means = vec![0.0; batches];
    for mean in batch_means.iter_mut() {
        for _ in 0..per_batch {
            *mean += (g(&trajectory_end()) - expected) / per_batch as f64;
        }
    }
    let average = batch_means.iter().sum::<f64>() / batches as f64;
    let variance = batch_means
        .iter()
        .map(|mean| (mean - average).powi(2) / (batches as f64 - 1.0))
        .sum();
    (average, variance)
}

/// Upstream `test_validateOu`: the Euler-Maruyama mean and second moment of
/// the Ornstein-Uhlenbeck process are unbiased within two estimated standard
/// deviations, over 10 batches of 100 seeded realizations.
#[test]
fn euler_maruyama_ornstein_uhlenbeck_weak_moments() {
    let (dt, tf) = (0.05, 0.1);
    let system = OrnsteinUhlenbeck::default();
    for (name, g, expected) in [
        (
            "mean",
            (|x: &[f64]| x[0]) as fn(&[f64]) -> f64,
            system.mean(tf),
        ),
        (
            "variance",
            |x: &[f64]| x[0] * x[0],
            system.variance(tf) + system.mean(tf).powi(2),
        ),
    ] {
        let mut seed = 0;
        let (error, variance) = error_and_empirical_variance(
            || {
                seed += 1;
                let mut sde = Sde::new(system.drift(), system.diffusion());
                euler_maruyama_run(&mut sde, &[system.x0], dt, tf, seed)
                    .pop()
                    .unwrap()
            },
            g,
            expected,
            10,
            100,
        );
        let two_sigma = 2.0 * variance.sqrt();
        assert!(
            error.abs() <= two_sigma,
            "{name}: error {error} exceeds two sigma {two_sigma}"
        );
    }
}

/// Upstream `test_example1_second_moment_equations`: Ito's formula for the
/// squared components gives the closed moment equations behind the Example 1
/// reference.
#[test]
fn example1_second_moment_equations() {
    let (drift, diffusion, _) = example1();
    let (first_decay, second_decay, coupling) = (1.0, 2.5, 0.01);
    for state in [[1.0, 0.0], [0.0, 1.0], [1.0, 1.0]] {
        let f = drift(0.0, &state);
        let mut rates = [2.0 * state[0] * f[0], 2.0 * state[1] * f[1]];
        for column in &diffusion {
            let g = column(0.0, &state);
            rates[0] += g[0] * g[0];
            rates[1] += g[1] * g[1];
        }
        let expected = [
            -first_decay * state[0] * state[0],
            coupling * state[0] * state[0] - second_decay * state[1] * state[1],
        ];
        for (rate, expected) in rates.iter().zip(expected) {
            assert!((rate - expected).abs() <= 1.0e-14 + 1.0e-14 * expected.abs());
        }
    }
}

/// Upstream `_example1_euler_second_moment`: the exact Euler-Maruyama second
/// moment `M_{n+1} = (I+hA) M_n (I+hA)ᵀ + h Σ_k B_k M_n B_kᵀ` of the second
/// component.
fn example1_euler_second_moment(dt: f64, steps: usize) -> f64 {
    let (drift, diffusion, x0) = example1();
    let matrix_of = |field: &dyn Fn(f64, &[f64]) -> Vec<f64>| {
        let first = field(0.0, &[1.0, 0.0]);
        let second = field(0.0, &[0.0, 1.0]);
        Matrix2::new(first[0], second[0], first[1], second[1])
    };
    let transition = Matrix2::identity() + dt * matrix_of(&*drift);
    let columns: Vec<Matrix2<f64>> = diffusion
        .iter()
        .map(|column| matrix_of(&**column))
        .collect();
    let initial = nalgebra::Vector2::new(x0[0], x0[1]);
    let mut moment = initial * initial.transpose();
    for _ in 0..steps {
        let mut next = transition * moment * transition.transpose();
        for column in &columns {
            next += dt * column * moment * column.transpose();
        }
        moment = next;
    }
    moment[(1, 1)]
}

/// Upstream `test_validateExample1`: 1000 Euler-Maruyama realizations of
/// Example 1 over 5 s, replaying prescribed increments, give the second
/// moment of the second component within two standard errors once the exact
/// discretization bias is removed.
#[test]
fn euler_maruyama_example1_second_moment() {
    let dt = 2.0_f64.powi(-3);
    let tf = 5.0;
    let steps = (tf / dt).round() as usize;
    let batches = 10;
    let mut trajectory_seed = 0;
    let (error, variance) = error_and_empirical_variance(
        || {
            let mut rng = StdRng::seed_from_u64(trajectory_seed);
            trajectory_seed += 1;
            let noise: Vec<_> = (0..steps)
                .map(|_| {
                    let dw: Vec<f64> = (0..2)
                        .map(|_| {
                            let draw: f64 = StandardNormal.sample(&mut rng);
                            dt.sqrt() * draw
                        })
                        .collect();
                    (dw, Vec::new())
                })
                .collect();
            let (generator, dynamic) = prescribed(&noise);
            let (drift, diffusion, x0) = example1();
            let mut sde = Sde::new(drift, diffusion);
            let integrator = EulerMaruyama::new().with_noise_generator(dynamic);
            let end = trajectory(&integrator, &mut sde, x0, dt, steps)
                .pop()
                .unwrap();
            assert_eq!(generator.lock().unwrap().remaining(), 0);
            end
        },
        |x| x[1] * x[1],
        149.0 / 150.0 * (-5.0 / 2.0 * tf).exp() + 1.0 / 150.0 * (-tf).exp(),
        batches,
        100,
    );
    let expected_bias = example1_euler_second_moment(dt, steps)
        - (149.0 / 150.0 * (-5.0 / 2.0 * tf).exp() + 1.0 / 150.0 * (-tf).exp());
    let standard_error = (variance / batches as f64).sqrt();
    assert!(error.is_finite() && standard_error.is_finite() && standard_error > 0.0);
    assert!(
        (error - expected_bias).abs() <= 2.0 * standard_error,
        "second-moment error {error}, discretization bias {expected_bias}, standard error \
         {standard_error}"
    );
}

// ---------------------------------------------------------------------------
// test_flatStochastic*.cpp and test_flatWeakItoFamilies.cpp
// ---------------------------------------------------------------------------

/// Upstream `WeakItoDynamics`: two coupled scalar states. With
/// `mixed_topology`, source 0 drives only the first state and source 1 both
/// (upstream's shared `{first: 1, second: 0}` slot after the unshared one).
fn weak_ito_dynamics(noise_count: usize, mixed_topology: bool) -> Sde {
    let drift: Field = Box::new(|_, x| vec![-0.5 * x[0] + 0.2 * x[1], 0.1 * x[0] - 0.4 * x[1]]);
    let diffusion: Vec<Field> = if mixed_topology {
        vec![
            Box::new(|_, x| vec![0.3 * x[0], 0.0]),
            Box::new(|_, x| vec![0.15 * x[1], 0.25 * x[0]]),
        ]
    } else {
        let columns: [Field; 2] = [
            Box::new(|_, x| vec![0.3 * x[0], 0.1 * x[1]]),
            Box::new(|_, x| vec![0.15 * x[1], 0.25 * x[0]]),
        ];
        columns.into_iter().take(noise_count).collect()
    };
    Sde::new(drift, diffusion)
}

const WEAK_ITO_INITIAL: [f64; 2] = [0.5, 1.0];

/// Upstream `EXPECT_DOUBLE_EQ`: equal within four units in the last place.
fn assert_double_eq(actual: f64, expected: f64, context: &str) {
    let ulps = (actual.to_bits() as i64 - expected.to_bits() as i64).unsigned_abs();
    assert!(
        actual.is_sign_negative() == expected.is_sign_negative() && ulps <= 4,
        "{context}: {actual} differs from {expected} by {ulps} ULP"
    );
}

/// One step of `h = 0.125` from `t = 0` with one prescribed sample.
fn single_step<D: DynamicObject + 'static>(
    method: &str,
    object: &mut D,
    x0: D::State,
    dw: &[f64],
    dz: &[f64],
) -> D::State {
    let (_, dynamic) = prescribed(&[(dw.to_vec(), dz.to_vec())]);
    let integrator = integrator::<D>(method, dynamic);
    integrator.propagate(object, &x0, 0, start_epoch(), 0.125).0
}

fn assert_weak_ito_snapshot(method: &str, dw: [f64; 2], dz: [f64; 2], expected: [f64; 2]) {
    let mut sde = weak_ito_dynamics(2, false);
    let state = single_step(method, &mut sde, WEAK_ITO_INITIAL.to_vec(), &dw, &dz);
    assert_double_eq(state[0], expected[0], method);
    assert_double_eq(state[1], expected[1], method);
}

/// Upstream `W2ItoPaperReferenceBits`.
#[test]
fn w2ito_paper_reference_bits() {
    assert_weak_ito_snapshot(
        "W2Ito1",
        [0.017225447734423038, 0.1831984663246682],
        [0.19143508986887522, -0.1501571687945868],
        [0.4886843990979673, 0.9537920887170881],
    );
    assert_weak_ito_snapshot(
        "W2Ito2",
        [0.08714682860793878, 0.07301273674452739],
        [-0.37346576044084245, -0.5112353004834385],
        [0.48945470733642576, 0.9543821243286132],
    );
}

/// Upstream `DRI1PaperReferenceBits`.
#[test]
fn dri1_paper_reference_bits() {
    assert_weak_ito_snapshot(
        "DRI1",
        [-0.13976011088567203, 0.09330800242534518],
        [0.21044072177715215, -0.20696984404638655],
        [0.4875825195312499, 0.9558728841145833],
    );
}

/// Upstream `RIFamilyPaperReferenceBits`.
#[test]
fn ri_family_paper_reference_bits() {
    assert_weak_ito_snapshot(
        "RI1",
        [0.3214313724988835, 0.26565882991430884],
        [-0.07854918074676295, 0.06465323920073543],
        [0.49130517578125005, 0.9527543945312499],
    );
    assert_weak_ito_snapshot(
        "RI3",
        [0.14084505263670893, -0.19898809924898814],
        [-0.4066940192839459, 0.059843829693812356],
        [0.49130517578125005, 0.9527543945312499],
    );
    assert_weak_ito_snapshot(
        "RI5",
        [-0.014115498539039673, -0.44467708215560936],
        [0.09780314160098916, -0.12406419972642377],
        [0.5789940869001351, 1.0376344341650983],
    );
    assert_weak_ito_snapshot(
        "RI6",
        [-0.4024065724993792, 0.2726672203670614],
        [-0.2912841140397067, 0.22757946014224292],
        [0.41054982655179995, 0.8957075905642796],
    );
}

/// Upstream `MixedSharedAndUnsharedNoiseTopology`.
#[test]
fn mixed_shared_and_unshared_noise_topology() {
    for (method, expected) in [
        ("W2Ito1", [0.4865037494250876, 0.8880694041707285]),
        ("W2Ito2", [0.48530347669697266, 0.8876774357988961]),
        ("DRI1", [0.4908012585585223, 0.8831252218343276]),
        ("DRI1NM", [0.5018871960585223, 0.8877931905843275]),
        ("RI1", [0.4906655385427433, 0.8830773481001063]),
        ("RI3", [0.49059808921507964, 0.88314479742777]),
        ("RI5", [0.4896668452149973, 1.031428158767153]),
        ("RI6", [0.4905956034653872, 0.8831502128649623]),
    ] {
        let mut sde = weak_ito_dynamics(2, true);
        let state = single_step(
            method,
            &mut sde,
            WEAK_ITO_INITIAL.to_vec(),
            &[0.5, -0.45],
            &[0.3, -0.2],
        );
        assert_double_eq(state[0], expected[0], method);
        assert_double_eq(state[1], expected[1], method);
    }
}

/// Upstream `OrderedWeakItoDynamics` with its `OrderedPolicy`: one scalar
/// state with two noise sources whose update is not an ordinary sum, so the
/// order and grouping of the increments show in the result. A drift
/// candidate is `x + c h + c x / 8` and a noise increment `x (1 + p) + c`,
/// where `c` is the combined drift or tangent and `p` the pseudo-step.
struct OrderedPolicy;

impl OrderedPolicy {
    fn combined(weighted: &[(&f64, f64)]) -> Option<f64> {
        let values: Vec<[f64; 1]> = weighted.iter().map(|&(value, _)| [*value]).collect();
        upstream_combination(
            values
                .iter()
                .zip(weighted)
                .map(|(value, &(_, weight))| (value.as_slice(), weight)),
        )
        .map(|combined| combined[0])
    }

    fn drift_candidate(base: f64, combined: Option<f64>, step: f64) -> f64 {
        let drift = combined.unwrap_or(0.0);
        base + drift * step + 0.125 * base * drift
    }
}

impl DynamicObject for OrderedPolicy {
    type State = f64;
    type Derivative = f64;
    type StepOutput = ();

    fn equations_of_motion(&mut self, state: &f64, _: u64, _: Epoch) -> f64 {
        0.25 + 0.125 * state
    }

    fn state_with_derivative(&self, state: &f64, derivative: &f64, dt: f64) -> f64 {
        Self::drift_candidate(*state, Some(*derivative), dt)
    }

    fn state_with_derivatives(&self, state: &f64, weighted: &[(&f64, f64)], step: f64) -> f64 {
        Self::drift_candidate(*state, Self::combined(weighted), step)
    }

    fn combine(&self, state: &f64, weighted: Vec<(f64, f64)>, step: f64) -> (f64, ()) {
        let borrowed: Vec<(&f64, f64)> = weighted
            .iter()
            .map(|(value, weight)| (value, *weight))
            .collect();
        (self.state_with_derivatives(state, &borrowed, step), ())
    }

    fn merge_step_outputs(&self, _: (), _: ()) {}

    fn noise_source_count(&self) -> usize {
        2
    }

    fn equations_of_motion_diffusion(&mut self, state: &f64, _: u64, _: Epoch) -> Vec<f64> {
        vec![2.0 + 0.5 * state, 3.0 - 0.25 * state]
    }

    fn state_with_diffusion(&self, state: &f64, diffusion: &[DiffusionIncrement<'_, f64>]) -> f64 {
        diffusion.iter().fold(*state, |state, increment| {
            let tangent = Self::combined(&increment.tangents).unwrap_or(0.0);
            state * (1.0 + increment.pseudo_step) + tangent
        })
    }
}

/// Upstream `NoncommutativeSpecialPolicyPreservesLocalNoiseOrder`.
#[test]
fn noncommutative_update_preserves_source_order() {
    for (method, expected) in [
        ("W2Ito1", 7.655313086497318),
        ("W2Ito2", 7.232975832932708),
        ("DRI1", 15.263479648254595),
        ("DRI1NM", 8.987209328934712),
    ] {
        let state = single_step(method, &mut OrderedPolicy, 1.0, &[0.5, -0.45], &[0.3, -0.2]);
        assert_double_eq(state, expected, method);
    }
}

/// Upstream `AllConcreteVariantsHandleZeroAndScalarNoise`, extended to every
/// method: zero and one noise source both give finite states.
#[test]
fn every_method_handles_zero_and_scalar_noise() {
    for method in ALL_METHODS {
        for noise_count in [0, 1] {
            let mut sde = weak_ito_dynamics(noise_count, false);
            let state = single_step(
                method,
                &mut sde,
                WEAK_ITO_INITIAL.to_vec(),
                &vec![0.125; noise_count],
                &vec![-0.25; noise_count],
            );
            assert!(
                state.iter().all(|value| value.is_finite()),
                "{method} with {noise_count} sources gave {state:?}"
            );
        }
    }
}

/// Upstream `ZeroDurationDoesNotSample` / `ZeroDurationAndRollback...`: a zero
/// step draws nothing and returns the state bit for bit; the next step
/// consumes exactly one sample.
#[test]
fn zero_duration_does_not_sample() {
    for method in ALL_METHODS {
        let (generator, dynamic) = prescribed(&[
            (vec![0.125, -0.25], vec![-0.375, 0.5]),
            (vec![0.5, 0.75], vec![0.25, -0.125]),
        ]);
        let integrator = integrator::<Sde>(method, dynamic);
        let mut sde = weak_ito_dynamics(2, false);
        let x0 = vec![0.75, -1.25];
        let unchanged = integrator.propagate(&mut sde, &x0, 0, start_epoch(), 0.0).0;
        assert_eq!(generator.lock().unwrap().remaining(), 2, "{method}");
        assert_eq!(unchanged[0].to_bits(), 0.75_f64.to_bits(), "{method}");
        assert_eq!(unchanged[1].to_bits(), (-1.25_f64).to_bits(), "{method}");
        integrator.propagate(&mut sde, &x0, 0, start_epoch(), 0.125);
        assert_eq!(generator.lock().unwrap().remaining(), 1, "{method}");
    }
}

/// Upstream `InvalidTimeStepsAreRejectedBeforeSampling`: a negative or
/// infinite step panics before any noise is drawn.
#[test]
fn invalid_time_steps_are_rejected_before_sampling() {
    for dt in [-1.0, f64::INFINITY, f64::NAN] {
        let (generator, dynamic) = prescribed(&[(vec![1.0], Vec::new())]);
        let integrator = EulerMaruyama::new().with_noise_generator(dynamic);
        let mut sde = Sde::new(
            Box::new(|_, x| vec![-x[0]]),
            vec![Box::new(|_, _| vec![1.0])],
        );
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            integrator.propagate(&mut sde, &vec![0.0], 0, start_epoch(), dt)
        }));
        assert!(result.is_err(), "dt = {dt} should be rejected");
        assert_eq!(generator.lock().unwrap().remaining(), 1, "dt = {dt}");
    }
}

/// Records the counts each step requests (upstream `AuxiliaryCountNoiseGenerator`).
#[derive(Debug, Default)]
struct CountingGenerator {
    requests: Vec<(usize, usize)>,
}

impl GaussianNoiseGenerator for CountingGenerator {
    fn set_seed(&mut self, _: u64) {}

    fn generate_with_auxiliary(
        &mut self,
        noise_count: usize,
        auxiliary_count: usize,
        _: f64,
    ) -> GaussianNoiseSample {
        self.requests.push((noise_count, auxiliary_count));
        GaussianNoiseSample {
            dw: vec![0.0625; noise_count],
            dz: vec![-0.03125; noise_count],
        }
    }
}

/// Upstream `ConcreteWeakMethodsRequestOnlyUsedAuxiliaryDraws`, extended to
/// every method: with four sources each step makes one request, for the
/// auxiliary increments its recurrence uses.
#[test]
fn methods_request_only_the_auxiliary_draws_they_use() {
    for (method, auxiliary) in [
        ("EulerMaruyama", 0),
        ("EulerHeun", 0),
        ("RKMil", 0),
        ("SRA1", 4),
        ("SOSRA", 4),
        ("SRIW1", 4),
        ("SOSRI", 4),
        ("W2Ito1", 2),
        ("W2Ito2", 2),
        ("DRI1", 3),
        ("DRI1NM", 0),
        ("RI1", 3),
        ("RI3", 3),
        ("RI5", 3),
        ("RI6", 3),
        ("RS1", 3),
        ("RS2", 3),
        ("SIEA", 0),
        ("SMEA", 0),
        ("SIEB", 0),
        ("SMEB", 0),
        ("RDI1WM", 0),
    ] {
        let generator = Arc::new(Mutex::new(CountingGenerator::default()));
        let dynamic: Arc<Mutex<dyn GaussianNoiseGenerator>> = generator.clone();
        let integrator = integrator::<Sde>(method, dynamic);
        let columns: Vec<Field> = (0..4)
            .map(|k| {
                Box::new(move |_: f64, x: &[f64]| {
                    let scale = 0.05 * (k as f64 + 1.0);
                    vec![scale * x[0], scale * x[1] + 0.01]
                }) as Field
            })
            .collect();
        let mut sde = Sde::new(
            Box::new(|_, x| vec![-0.5 * x[0], 0.1 * x[0] - 0.4 * x[1]]),
            columns,
        );
        let state = integrator
            .propagate(&mut sde, &vec![0.5, 1.0], 0, start_epoch(), 0.125)
            .0;
        assert!(state.iter().all(|value| value.is_finite()), "{method}");
        assert_eq!(
            generator.lock().unwrap().requests,
            [(4, auxiliary)],
            "{method}"
        );
    }
}

/// `set_rng_seed` restarts the default generator, so a reseeded integrator
/// repeats its realization (upstream `setRNGSeed`).
#[test]
fn reseeding_repeats_the_realization() {
    let integrator = Sosri::new().with_seed(3);
    let run = |integrator: &Sosri| {
        let system = OrnsteinUhlenbeck::default();
        let mut sde = Sde::new(system.drift(), system.diffusion());
        trajectory(integrator, &mut sde, vec![system.x0], 0.5, 6)
    };
    let first = run(&integrator);
    integrator.set_rng_seed(3);
    assert_eq!(run(&integrator), first);
    integrator.set_rng_seed(4);
    assert_ne!(run(&integrator), first);
}

/// The generic SRA/SRI integrators reject what upstream's templates reject
/// and, with a shipped tableau, step exactly like the named method.
#[test]
fn custom_tableau_integrators_validate_and_match_named_methods() {
    let mut malformed = SRA1;
    malformed.a0[0][1] = 0.5;
    assert!(StrongSra::new(malformed).is_err());
    let mut malformed = SRIW1;
    malformed.beta4[3] = f64::NAN;
    assert!(StrongSri::new(malformed).is_err());

    let system = OrnsteinUhlenbeck::default();
    let run = |integrator: &dyn Integrator<Sde>| {
        let mut sde = Sde::new(system.drift(), system.diffusion());
        trajectory(integrator, &mut sde, vec![system.x0], 0.25, 8)
    };
    assert_eq!(
        run(&StrongSra::new(SOSRA).unwrap().with_seed(5)),
        run(&Sosra::new().with_seed(5))
    );
    assert_eq!(
        run(&StrongSri::new(SRIW1).unwrap().with_seed(5)),
        run(&Sriw1::new().with_seed(5))
    );
}
