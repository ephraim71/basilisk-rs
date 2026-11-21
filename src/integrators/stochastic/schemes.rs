//! One step of each stochastic Runge-Kutta family: the `integrateImpl`
//! recurrences of upstream `svStochasticIntegrator*.cpp` and the SRA/SRI
//! templates.
//!
//! Notation: `y` is the step-start state, `h` the step, `m` the number of
//! noise sources, `f_j` the drift stages and `g[j][k]` diffusion stage `j` of
//! source `k`. Stage nodes are fractions of `h` past the step start, as in
//! upstream; the weight lists keep upstream's stage order, so an object that
//! sums its weighted derivatives before scaling reproduces upstream's
//! arithmetic.

use hifitime::Epoch;

use super::noise::{three_point, two_point};
use super::tableaus::{
    Dri1Coefficients, RsCoefficients, SieSmeCoefficients, SraTableau, SriTableau, W2ItoTableau,
};
use super::{
    StochasticNoise, StochasticStep, column_increments, source_increments, source_terms, weighted,
    zero_step,
};
use crate::integrators::traits::{DiffusionIncrement, DynamicObject};

/// Euler-Maruyama, strong order 0.5, weak order 1 (upstream
/// `svStochasticIntegratorMayurama`):
/// `x' = x + f(t, x) h + Σ_k g_k(t, x) ΔW_k`.
pub(super) fn euler_maruyama<D: DynamicObject>(
    noise: &StochasticNoise,
    object: &mut D,
    state: &D::State,
    nanos: u64,
    epoch: Epoch,
    h: f64,
) -> (D::State, D::StepOutput) {
    if let Some(unchanged) = zero_step(object, state, h) {
        return unchanged;
    }
    let mut step = StochasticStep::new(object, state, nanos, epoch, h);
    let dw = noise.wiener(step.noise_count, h);
    let f0 = step.drift(state, 0.0);
    let g0 = step.diffusions(state, 0.0);
    step.finish(vec![(f0, 1.0)], &column_increments(&g0, 1.0, &dw))
}

/// Euler-Heun, the Stratonovich analogue of Euler-Maruyama, strong order 0.5
/// (upstream `svStochasticIntegratorEulerHeun`): an Euler-Maruyama predictor
/// `x̄`, then the trapezoidal average of drift and diffusion at `x` and `x̄`.
pub(super) fn euler_heun<D: DynamicObject>(
    noise: &StochasticNoise,
    object: &mut D,
    state: &D::State,
    nanos: u64,
    epoch: Epoch,
    h: f64,
) -> (D::State, D::StepOutput) {
    if let Some(unchanged) = zero_step(object, state, h) {
        return unchanged;
    }
    let mut step = StochasticStep::new(object, state, nanos, epoch, h);
    let dw = noise.wiener(step.noise_count, h);
    let f0 = step.drift(state, 0.0);
    let g0 = step.diffusions(state, 0.0);
    let predictor = step.candidate(&[(&f0, 1.0)], &column_increments(&g0, 1.0, &dw));
    let f1 = step.drift(&predictor, 1.0);
    let g1 = step.diffusions(&predictor, 1.0);
    let increments: Vec<_> = g0
        .iter()
        .zip(&g1)
        .zip(&dw)
        .map(|((first, second), &dw)| DiffusionIncrement {
            tangents: vec![(first, 0.5), (second, 0.5)],
            pseudo_step: dw,
        })
        .collect();
    step.finish(vec![(f0, 0.5), (f1, 0.5)], &increments)
}

/// Derivative-free Runge-Kutta Milstein, strong order 1 for diagonal or scalar
/// Ito noise (upstream `svStochasticIntegratorRKMil`):
///
/// ```text
/// K = x + f h,  x̃ = K + Σ_k g_k √h,  (gg')_k = (g_k(x̃) - g_k(x)) / √h
/// x' = K + Σ_k g_k ΔW_k + Σ_k (gg')_k (ΔW_k² - h) / 2
/// ```
pub(super) fn rk_milstein<D: DynamicObject>(
    noise: &StochasticNoise,
    object: &mut D,
    state: &D::State,
    nanos: u64,
    epoch: Epoch,
    h: f64,
) -> (D::State, D::StepOutput) {
    if let Some(unchanged) = zero_step(object, state, h) {
        return unchanged;
    }
    let mut step = StochasticStep::new(object, state, nanos, epoch, h);
    let m = step.noise_count;
    let dw = noise.wiener(m, h);
    let sqrt_h = h.sqrt();
    let f0 = step.drift(state, 0.0);
    let g0 = step.diffusions(state, 0.0);
    let k_state = step.candidate(&[(&f0, 1.0)], &[]);
    let support = step.with_diffusion(&k_state, &column_increments(&g0, 1.0, &vec![sqrt_h; m]));
    // Upstream evaluates the support diffusion at the step-start time.
    let g_support = step.diffusions(&support, 0.0);
    let inverse_sqrt_h = 1.0 / sqrt_h;
    let mut increments = column_increments(&g0, 1.0, &dw);
    increments.extend(
        g_support
            .iter()
            .zip(&g0)
            .zip(&dw)
            .map(|((support, first), &dw)| DiffusionIncrement {
                tangents: vec![(support, inverse_sqrt_h), (first, -inverse_sqrt_h)],
                pseudo_step: (dw * dw - h) / 2.0,
            }),
    );
    step.finish(vec![(f0, 1.0)], &increments)
}

/// Roessler SRA step for additive noise (upstream
/// `svIntegratorStrongStochasticRungeKuttaSRA::integrateImpl`), with
/// `χ2 = (ΔW + ΔZ/√3)/2`:
///
/// ```text
/// H_i = y + h Σ_j A0_ij f_j + Σ_k Σ_j B0_ij g[j][k] χ2_k
/// y'  = y + h Σ_i α_i f_i + Σ_k Σ_i (β1_i ΔW_k + β2_i χ2_k) g[i][k]
/// ```
pub(super) fn sra<D: DynamicObject, const S: usize>(
    tableau: &SraTableau<S>,
    noise: &StochasticNoise,
    object: &mut D,
    state: &D::State,
    nanos: u64,
    epoch: Epoch,
    h: f64,
) -> (D::State, D::StepOutput) {
    if let Some(unchanged) = zero_step(object, state, h) {
        return unchanged;
    }
    let mut step = StochasticStep::new(object, state, nanos, epoch, h);
    let m = step.noise_count;
    let sample = noise.sample(m, m, h);
    let sqrt_3 = 3.0_f64.sqrt();
    let chi2: Vec<f64> = (0..m)
        .map(|k| (sample.dw[k] + sample.dz[k] / sqrt_3) / 2.0)
        .collect();

    let mut f = vec![step.drift(state, tableau.c0[0])];
    let mut g = vec![step.diffusions(state, tableau.c1[0])];
    for i in 1..S {
        let stage = step.candidate(
            &weighted(&f, &tableau.a0[i][..i]),
            &source_increments(&g, &tableau.b0[i][..i], &chi2),
        );
        f.push(step.drift(&stage, tableau.c0[i]));
        g.push(step.diffusions(&stage, tableau.c1[i]));
    }

    let mut increments = source_increments(&g, &tableau.beta1, &sample.dw);
    increments.extend(source_increments(&g, &tableau.beta2, &chi2));
    step.finish(f.into_iter().zip(tableau.alpha).collect(), &increments)
}

/// Roessler SRI step for diagonal or scalar noise, strong order 1.5
/// (upstream `svIntegratorStrongStochasticRungeKuttaSRI::integrateImpl`).
/// With `χ1 = (ΔW² - h)/(2√h)`, `χ2 = (ΔW + ΔZ/√3)/2` and
/// `χ3 = (ΔW³ - 3ΔW h)/(6h)`:
///
/// ```text
/// H0_i  = y + h Σ_j A0_ij f_j + Σ_k Σ_j B0_ij g[j][k] χ2_k
/// H1_ik = y + h Σ_j A1_ij f_j + √h Σ_j B1_ij g[j][k]         (source k only)
/// y'    = y + h Σ_i α_i f_i
///           + Σ_k Σ_i (β1_i ΔW_k + β2_i χ1_k + β3_i χ2_k + β4_i χ3_k) g[i][k]
/// ```
///
/// The drift stages are shared by every source; diffusion stage `i` of
/// source `k` is evaluated at its own `H1_ik`.
pub(super) fn sri<D: DynamicObject, const S: usize>(
    tableau: &SriTableau<S>,
    noise: &StochasticNoise,
    object: &mut D,
    state: &D::State,
    nanos: u64,
    epoch: Epoch,
    h: f64,
) -> (D::State, D::StepOutput) {
    if let Some(unchanged) = zero_step(object, state, h) {
        return unchanged;
    }
    let mut step = StochasticStep::new(object, state, nanos, epoch, h);
    let m = step.noise_count;
    let sample = noise.sample(m, m, h);
    let sqrt_h = h.sqrt();
    let sqrt_3 = 3.0_f64.sqrt();
    let mut chi1 = Vec::with_capacity(m);
    let mut chi2 = Vec::with_capacity(m);
    let mut chi3 = Vec::with_capacity(m);
    for (&dw, &dz) in sample.dw.iter().zip(&sample.dz) {
        chi1.push((dw * dw - h) / (2.0 * sqrt_h));
        chi2.push((dw + dz / sqrt_3) / 2.0);
        chi3.push((dw * dw * dw - 3.0 * dw * h) / (6.0 * h));
    }

    let mut f = vec![step.drift(state, 0.0)];
    let mut g = vec![step.diffusions(state, 0.0)];
    for i in 1..S {
        let drift_stage = step.candidate(
            &weighted(&f, &tableau.a0[i][..i]),
            &source_increments(&g, &tableau.b0[i][..i], &chi2),
        );
        f.push(step.drift(&drift_stage, tableau.c0[i]));
        let mut stage_columns = Vec::with_capacity(m);
        for k in 0..m {
            let diffusion_stage = step.candidate(
                &weighted(&f[..i], &tableau.a1[i][..i]),
                &[DiffusionIncrement {
                    tangents: source_terms(&g, k, &tableau.b1[i][..i]),
                    pseudo_step: sqrt_h,
                }],
            );
            stage_columns.push(step.diffusion(&diffusion_stage, tableau.c1[i], k));
        }
        g.push(stage_columns);
    }

    let mut increments = source_increments(&g, &tableau.beta1, &sample.dw);
    increments.extend(source_increments(&g, &tableau.beta2, &chi1));
    increments.extend(source_increments(&g, &tableau.beta3, &chi2));
    increments.extend(source_increments(&g, &tableau.beta4, &chi3));
    step.finish(f.into_iter().zip(tableau.alpha).collect(), &increments)
}

/// Tang & Xiao weak-order-2 step for Ito SDEs of any noise structure
/// (upstream `svStochasticIntegratorW2Ito::integrateImpl`). The random
/// variables are a three-point `Î_k` per source (from `ΔW_k`), the scalar
/// `ξ = η1 √h` and the sign `η2` (two-point, from `ΔZ_0` and `ΔZ_1`), and
/// the iterated integrals `Î_kk = (Î_k²/ξ - ξ)/2` and
/// `Î_kl = (Î_l ∓ η2 Î_l)/2` for `k < l` / `k > l`:
///
/// ```text
/// H0_i  = y + h Σ_j A0_ij f_j + Σ_l Σ_j B0_ij g[j][l] Î_l
/// Hk_i  = y + h Σ_j A1_ij f_j + Σ_j B1_ij g[j][k] ξ + Σ_{l≠k} Σ_j B2_ij g[j][l] Î_kl
/// y'    = y + h Σ_i α_i f_i + Σ_k Σ_i (β0_i Î_k + β1_i Î_kk) g[i][k]
/// ```
pub(super) fn w2ito<D: DynamicObject, const S: usize>(
    tableau: &W2ItoTableau<S>,
    noise: &StochasticNoise,
    object: &mut D,
    state: &D::State,
    nanos: u64,
    epoch: Epoch,
    h: f64,
) -> (D::State, D::StepOutput) {
    if let Some(unchanged) = zero_step(object, state, h) {
        return unchanged;
    }
    let mut step = StochasticStep::new(object, state, nanos, epoch, h);
    let m = step.noise_count;
    let sample = noise.sample(m, m.min(2), h);
    let sqrt_h = h.sqrt();
    // Without noise sources there is no `dz` to draw `ξ` and `η2` from; the
    // step then reduces to the deterministic tableau (upstream guard).
    let eta1 = if m > 0 {
        two_point(sample.dz[0], 1.0)
    } else {
        1.0
    };
    let eta2 = if m > 1 {
        two_point(sample.dz[1], 1.0)
    } else {
        0.0
    };
    let xi = sqrt_h * eta1;
    let weak_dw: Vec<f64> = sample.dw.iter().map(|&dw| three_point(dw, h)).collect();
    let diagonal_integral: Vec<f64> = weak_dw
        .iter()
        .map(|&weak| (weak * weak / xi - xi) / 2.0)
        .collect();
    let mixed_integral = |k: usize, l: usize| {
        if k < l {
            0.5 * (weak_dw[l] - eta2 * weak_dw[l])
        } else {
            0.5 * (weak_dw[l] + eta2 * weak_dw[l])
        }
    };

    let mut f = vec![step.drift(state, 0.0)];
    let mut g = vec![step.diffusions(state, 0.0)];
    for i in 1..S {
        let drift_stage = step.candidate(
            &weighted(&f, &tableau.a0[i][..i]),
            &source_increments(&g, &tableau.b0[i][..i], &weak_dw),
        );
        f.push(step.drift(&drift_stage, tableau.c0(i)));
        let mut stage_columns = Vec::with_capacity(m);
        for k in 0..m {
            let increments: Vec<_> = (0..m)
                .map(|l| {
                    if l == k {
                        DiffusionIncrement {
                            tangents: source_terms(&g, l, &tableau.b1[i][..i]),
                            pseudo_step: xi,
                        }
                    } else {
                        DiffusionIncrement {
                            tangents: source_terms(&g, l, &tableau.b2[i][..i]),
                            pseudo_step: mixed_integral(k, l),
                        }
                    }
                })
                .collect();
            let diffusion_stage =
                step.candidate(&weighted(&f[..i], &tableau.a1[i][..i]), &increments);
            stage_columns.push(step.diffusion(&diffusion_stage, tableau.c1(i), k));
        }
        g.push(stage_columns);
    }

    let mut increments = source_increments(&g, &tableau.beta0, &weak_dw);
    increments.extend(source_increments(&g, &tableau.beta1, &diagonal_integral));
    step.finish(f.into_iter().zip(tableau.alpha).collect(), &increments)
}

/// A DRI1-family method: its coefficients and whether it drops the
/// cross-source terms (upstream `setNonMixing`, the DRI1NM variant).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Dri1Scheme {
    pub(super) coefficients: Dri1Coefficients,
    pub(super) non_mixing: bool,
}

/// Debrabant-Roessler DRI1 step, weak order 2 for Ito SDEs (upstream
/// `svStochasticIntegratorDRI1::integrateImpl`), shared by the RI methods.
/// The non-mixing variant drops the cross-source terms, which is exact for
/// diagonal noise. Random variables: three-point `Î_k`, its diagonal
/// `χ1_k = (Î_k² - h)/2`, and for the cross terms two-point `Ẑ_k = ±√h`
/// (one fewer than the sources) in
/// `Î_(k,l) = (Î_k Î_l ∓ √h Ẑ)/2`.
pub(super) fn dri1<D: DynamicObject>(
    scheme: &Dri1Scheme,
    noise: &StochasticNoise,
    object: &mut D,
    state: &D::State,
    nanos: u64,
    epoch: Epoch,
    h: f64,
) -> (D::State, D::StepOutput) {
    if let Some(unchanged) = zero_step(object, state, h) {
        return unchanged;
    }
    let c = &scheme.coefficients;
    let mut step = StochasticStep::new(object, state, nanos, epoch, h);
    let m = step.noise_count;
    let cross_noise = m > 1 && !scheme.non_mixing;
    let auxiliary_count = if cross_noise { m - 1 } else { 0 };
    let sample = noise.sample(m, auxiliary_count, h);
    let sqrt_h = h.sqrt();
    let weak_dw: Vec<f64> = sample.dw.iter().map(|&dw| three_point(dw, h)).collect();
    let diagonal_integral: Vec<f64> = weak_dw
        .iter()
        .map(|&weak| (weak * weak - h) / 2.0)
        .collect();
    let weak_dz: Vec<f64> = (0..m)
        .map(|k| {
            if k < auxiliary_count {
                two_point(sample.dz[k], sqrt_h)
            } else {
                0.0
            }
        })
        .collect();

    // Drift stages H0_2 and H0_3.
    let f0 = step.drift(state, 0.0);
    let g0 = step.diffusions(state, 0.0);
    let stage = step.candidate(&[(&f0, c.a021)], &column_increments(&g0, c.b021, &weak_dw));
    let f1 = step.drift(&stage, c.c02);
    let stage = step.candidate(
        &[(&f0, c.a031), (&f1, c.a032)],
        &column_increments(&g0, c.b031, &weak_dw),
    );
    let f2 = step.drift(&stage, c.c03);

    // Diffusion stages H_2^(k) and H_3^(k), one source at a time.
    let mut g1 = Vec::with_capacity(m);
    let mut g2 = Vec::with_capacity(m);
    for (k, column) in g0.iter().enumerate() {
        let stage = step.candidate(
            &[(&f0, c.a121)],
            &[DiffusionIncrement {
                tangents: vec![(column, 1.0)],
                pseudo_step: c.b121 * sqrt_h,
            }],
        );
        g1.push(step.diffusion(&stage, c.c12, k));
        let stage = step.candidate(
            &[(&f0, c.a131)],
            &[DiffusionIncrement {
                tangents: vec![(column, 1.0)],
                pseudo_step: c.b131 * sqrt_h,
            }],
        );
        g2.push(step.diffusion(&stage, c.c13, k));
    }

    // Mixed-integral support stages Ĥ_2^(l) and Ĥ_3^(l): every source's
    // diffusion at each.
    let mut first_hat = Vec::new();
    let mut second_hat = Vec::new();
    if cross_noise {
        // Upstream reuses the H0_3 drift combination over a zero step here.
        let stale_drift = [(&f0, c.a031), (&f1, c.a032)];
        for l in 0..m {
            let stage = step.candidate_over(
                &stale_drift,
                0.0,
                &[DiffusionIncrement {
                    tangents: vec![(&g0[l], c.b221), (&g1[l], c.b222), (&g2[l], c.b223)],
                    pseudo_step: sqrt_h,
                }],
            );
            first_hat.push(step.diffusions(&stage, 0.0));
            let stage = step.candidate_over(
                &stale_drift,
                0.0,
                &[DiffusionIncrement {
                    tangents: vec![(&g0[l], c.b231), (&g1[l], c.b232), (&g2[l], c.b233)],
                    pseudo_step: sqrt_h,
                }],
            );
            second_hat.push(step.diffusions(&stage, 0.0));
        }
    }

    let self_31 = if cross_noise {
        (m - 1) as f64 * c.beta31
    } else {
        0.0
    };
    let mut increments = column_increments(&g0, c.beta11 + self_31, &weak_dw);
    for (k, column) in g1.iter().enumerate() {
        increments.push(DiffusionIncrement {
            tangents: vec![(column, 1.0)],
            pseudo_step: weak_dw[k] * c.beta12 + diagonal_integral[k] * c.beta22 / sqrt_h,
        });
    }
    for (k, column) in g2.iter().enumerate() {
        increments.push(DiffusionIncrement {
            tangents: vec![(column, 1.0)],
            pseudo_step: weak_dw[k] * c.beta13 + diagonal_integral[k] * c.beta23 / sqrt_h,
        });
    }
    if cross_noise {
        let mixed_integral = |k: usize, l: usize| {
            if k < l {
                (weak_dw[k] * weak_dw[l] - sqrt_h * weak_dz[k]) / 2.0
            } else {
                (weak_dw[k] * weak_dw[l] + sqrt_h * weak_dz[l]) / 2.0
            }
        };
        for l in 0..m {
            for k in (0..m).filter(|&k| k != l) {
                let mixed = mixed_integral(k, l);
                increments.push(DiffusionIncrement {
                    tangents: vec![(&first_hat[l][k], 1.0)],
                    pseudo_step: weak_dw[k] * c.beta32 + mixed * c.beta42 / sqrt_h,
                });
                increments.push(DiffusionIncrement {
                    tangents: vec![(&second_hat[l][k], 1.0)],
                    pseudo_step: weak_dw[k] * c.beta33 + mixed * c.beta43 / sqrt_h,
                });
            }
        }
    }
    step.finish(
        vec![(f0, c.alpha1), (f1, c.alpha2), (f2, c.alpha3)],
        &increments,
    )
}

/// Upstream `RDI1WM` coefficients (anonymous namespace of
/// `svStochasticIntegratorRDI1WM.cpp`).
const RDI1WM_A021: f64 = 2.0 / 3.0;
const RDI1WM_B021: f64 = 2.0 / 3.0;
const RDI1WM_ALPHA1: f64 = 1.0 / 4.0;
const RDI1WM_ALPHA2: f64 = 3.0 / 4.0;
const RDI1WM_C02: f64 = 2.0 / 3.0;
const RDI1WM_BETA11: f64 = 1.0;

/// Roessler RDI1WM step, weak order 1 (deterministic order 2) for diagonal or
/// scalar noise (upstream `svStochasticIntegratorRDI1WM`), with the
/// three-point increment `Î`:
///
/// ```text
/// H = y + a021 f(y) h + b021 g(y) Î
/// y' = y + (α1 f(y) + α2 f(t + c02 h, H)) h + β11 g(y) Î
/// ```
pub(super) fn rdi1wm<D: DynamicObject>(
    noise: &StochasticNoise,
    object: &mut D,
    state: &D::State,
    nanos: u64,
    epoch: Epoch,
    h: f64,
) -> (D::State, D::StepOutput) {
    if let Some(unchanged) = zero_step(object, state, h) {
        return unchanged;
    }
    let mut step = StochasticStep::new(object, state, nanos, epoch, h);
    let weak_dw: Vec<f64> = noise
        .wiener(step.noise_count, h)
        .iter()
        .map(|&dw| three_point(dw, h))
        .collect();
    let f0 = step.drift(state, 0.0);
    let g0 = step.diffusions(state, 0.0);
    let stage = step.candidate(
        &[(&f0, RDI1WM_A021)],
        &column_increments(&g0, RDI1WM_B021, &weak_dw),
    );
    let f1 = step.drift(&stage, RDI1WM_C02);
    step.finish(
        vec![(f0, RDI1WM_ALPHA1), (f1, RDI1WM_ALPHA2)],
        &column_increments(&g0, RDI1WM_BETA11, &weak_dw),
    )
}

/// Roessler RS1/RS2 step, weak order 2 for Stratonovich SDEs of any noise
/// structure (upstream `svStochasticIntegratorRS::integrateImpl`, Roessler
/// 2007 eq. 5.1-5.2). Random variables: three-point `Î_k` and two-point
/// `Ĩ_k = ±√h` (one fewer than the sources), combined into
/// `Î_(k,l) = Î_k Ĩ_l` for `l < k` and `-Î_l Ĩ_k` for `k < l`. Four drift and
/// diffusion stages, plus, for several sources, the cross-source stages
/// `Ĥ_i^(k)` at the step-start time.
pub(super) fn rs<D: DynamicObject>(
    c: &RsCoefficients,
    noise: &StochasticNoise,
    object: &mut D,
    state: &D::State,
    nanos: u64,
    epoch: Epoch,
    h: f64,
) -> (D::State, D::StepOutput) {
    if let Some(unchanged) = zero_step(object, state, h) {
        return unchanged;
    }
    let mut step = StochasticStep::new(object, state, nanos, epoch, h);
    let m = step.noise_count;
    let sample = noise.sample(m, m.saturating_sub(1), h);
    let sqrt_h = h.sqrt();
    let i_hat: Vec<f64> = sample.dw.iter().map(|&dw| three_point(dw, h)).collect();
    let i_tilde: Vec<f64> = (0..m)
        .map(|k| {
            if k + 1 < m {
                two_point(sample.dz[k], sqrt_h)
            } else {
                0.0
            }
        })
        .collect();
    let mixed_integral = |k: usize, l: usize| {
        if l < k {
            i_hat[k] * i_tilde[l]
        } else {
            -i_hat[l] * i_tilde[k]
        }
    };

    let zero = [0.0; 4];
    let a0 = [
        zero,
        [c.a021, 0.0, 0.0, 0.0],
        [c.a031, c.a032, 0.0, 0.0],
        zero,
    ];
    let a1 = [zero, zero, [c.a131, 0.0, 0.0, 0.0], [c.a141, 0.0, 0.0, 0.0]];
    let b0 = [zero, zero, [c.b031, c.b032, 0.0, 0.0], zero];
    let b1 = [
        zero,
        [c.b121, 0.0, 0.0, 0.0],
        [c.b131, c.b132, 0.0, 0.0],
        [c.b141, c.b142, c.b143, 0.0],
    ];
    let b2 = [zero, [c.b221, 0.0, 0.0, 0.0], [c.b231, 0.0, 0.0, 0.0], zero];
    let b3 = [
        zero,
        zero,
        [c.b331, c.b332, 0.0, 0.0],
        [c.b341, c.b342, 0.0, 0.0],
    ];
    let drift_nodes = [0.0, c.c02, c.c03, 0.0];
    let diffusion_nodes = [0.0, 0.0, c.c13, c.c14];

    let mut f = vec![step.drift(state, 0.0)];
    let mut g = vec![step.diffusions(state, 0.0)];
    for i in 1..4 {
        let drift_stage = step.candidate(
            &weighted(&f, &a0[i][..i]),
            &source_increments(&g, &b0[i][..i], &i_hat),
        );
        f.push(step.drift(&drift_stage, drift_nodes[i]));
        let mut stage_columns = Vec::with_capacity(m);
        for k in 0..m {
            let increments: Vec<_> = (0..m)
                .map(|l| DiffusionIncrement {
                    tangents: source_terms(&g, l, if l == k { &b1[i][..i] } else { &b3[i][..i] }),
                    pseudo_step: i_hat[l],
                })
                .collect();
            let diffusion_stage = step.candidate(&weighted(&f[..i], &a1[i][..i]), &increments);
            stage_columns.push(step.diffusion(&diffusion_stage, diffusion_nodes[i], k));
        }
        g.push(stage_columns);
    }

    // Cross-source stages: hats[k][i - 1] is source k at Ĥ_i^(k); stage 0 is
    // the step-start column g[0][k].
    let mut hats: Vec<Vec<D::Derivative>> = Vec::new();
    if m > 1 {
        // Upstream reuses the last A1 drift combination over a zero step here.
        let stale_drift = weighted(&f[..3], &a1[3][..3]);
        for k in 0..m {
            let mut source_hats = Vec::with_capacity(3);
            for row in &b2[1..] {
                let i = source_hats.len() + 1;
                let increments: Vec<_> = (0..m)
                    .filter(|&l| l != k)
                    .map(|l| DiffusionIncrement {
                        tangents: source_terms(&g, l, &row[..i]),
                        pseudo_step: mixed_integral(k, l) / sqrt_h,
                    })
                    .collect();
                let hat_stage = step.candidate_over(&stale_drift, 0.0, &increments);
                source_hats.push(step.diffusion(&hat_stage, 0.0, k));
            }
            hats.push(source_hats);
        }
    }

    let beta1 = [c.beta11, c.beta12, c.beta13, c.beta14];
    let mut increments = source_increments(&g, &beta1, &i_hat);
    for (k, source_hats) in hats.iter().enumerate() {
        increments.push(DiffusionIncrement {
            tangents: vec![
                (&g[0][k], 0.0),
                (&source_hats[0], c.beta22),
                (&source_hats[1], c.beta23),
                (&source_hats[2], 0.0),
            ],
            pseudo_step: sqrt_h,
        });
    }
    let alpha = [c.alpha1, c.alpha2, c.alpha3, c.alpha4];
    step.finish(f.into_iter().zip(alpha).collect(), &increments)
}

/// Tocino & Vigo-Aguiar SIE/SME step, weak order 2 for diagonal or scalar
/// Ito noise (upstream `svStochasticIntegratorSIESME::integrateImpl`), driven
/// by the Gaussian increment's moments `W2 = ΔW²/√h` and `W3 = ν2 ΔW³/h`:
///
/// ```text
/// k1 = f(t + μ0 h,  y + λ0 k0 h + ν1 g0 ΔW + g0 W3)
/// g1 = g(t + μ̄0 h, y + λ̄0 k0 h + β2 g0 √h + β3 g0 W2)
/// g2 = g(t + μ̄0 h, y + λ̄0 k0 h + δ2 g0 √h + δ3 g0 W2)
/// y' = y + (α1 k0 + α2 k1) h + γ1 g0 ΔW
///        + (λ1 ΔW + λ2 √h + λ3 W2) g1 + (μ1 ΔW + μ2 √h + μ3 W2) g2
/// ```
pub(super) fn sie_sme<D: DynamicObject>(
    c: &SieSmeCoefficients,
    noise: &StochasticNoise,
    object: &mut D,
    state: &D::State,
    nanos: u64,
    epoch: Epoch,
    h: f64,
) -> (D::State, D::StepOutput) {
    if let Some(unchanged) = zero_step(object, state, h) {
        return unchanged;
    }
    let mut step = StochasticStep::new(object, state, nanos, epoch, h);
    let dw = noise.wiener(step.noise_count, h);
    let sqrt_h = h.sqrt();
    let w2: Vec<f64> = dw.iter().map(|&dw| dw * dw / sqrt_h).collect();
    let w3: Vec<f64> = dw.iter().map(|&dw| c.nu2 * dw * dw * dw / h).collect();
    let per_source = |pseudo_step: &dyn Fn(usize) -> f64| -> Vec<f64> {
        (0..dw.len()).map(pseudo_step).collect()
    };

    let f0 = step.drift(state, 0.0);
    let g0 = step.diffusions(state, 0.0);
    let stage = step.candidate(
        &[(&f0, c.lambda0)],
        &column_increments(&g0, 1.0, &per_source(&|k| c.nu1 * dw[k] + w3[k])),
    );
    let f1 = step.drift(&stage, c.mu0);
    let stage = step.candidate(
        &[(&f0, c.lambdabar0)],
        &column_increments(
            &g0,
            1.0,
            &per_source(&|k| c.beta2 * sqrt_h + c.beta3 * w2[k]),
        ),
    );
    let g1 = step.diffusions(&stage, c.mubar0);
    let stage = step.candidate(
        &[(&f0, c.lambdabar0)],
        &column_increments(
            &g0,
            1.0,
            &per_source(&|k| c.delta2 * sqrt_h + c.delta3 * w2[k]),
        ),
    );
    let g2 = step.diffusions(&stage, c.mubar0);

    let mut increments = column_increments(&g0, 1.0, &per_source(&|k| c.gamma1 * dw[k]));
    increments.extend(column_increments(
        &g1,
        1.0,
        &per_source(&|k| c.lambda1 * dw[k] + c.lambda2 * sqrt_h + c.lambda3 * w2[k]),
    ));
    increments.extend(column_increments(
        &g2,
        1.0,
        &per_source(&|k| c.mu1 * dw[k] + c.mu2 * sqrt_h + c.mu3 * w2[k]),
    ));
    step.finish(vec![(f0, c.alpha1), (f1, c.alpha2)], &increments)
}
