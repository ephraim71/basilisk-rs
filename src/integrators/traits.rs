use hifitime::Epoch;

/// One noise source's contribution to a stochastic update,
/// `pseudo_step · Σ weightᵢ · tangentᵢ`.
///
/// Each tangent is a diffusion column `g_k(t, x)` in derivative form (see
/// [`DynamicObject::equations_of_motion_diffusion`]); the weights combine the
/// stages of a stochastic Runge-Kutta method and `pseudo_step` is the random
/// variable multiplying them, usually a Wiener increment (upstream
/// `pseudoStep`).
pub struct DiffusionIncrement<'a, T> {
    /// Diffusion tangents and their stage weights.
    pub tangents: Vec<(&'a T, f64)>,
    /// Multiplier of the weighted tangent sum.
    pub pseudo_step: f64,
}

pub trait DynamicObject {
    type State: Clone;
    type StepOutput;
    type Derivative;

    /// Accepts `&mut self` since it is needed to cache the intermediate state into effectors.
    ///
    /// It is upto the caller (eg: within an integrator) to re-sync the committed state afterwards.
    fn equations_of_motion(
        &mut self,
        state: &Self::State,
        current_sim_nanos: u64,
        current_epoch: Epoch,
    ) -> Self::Derivative;

    fn state_with_derivative(
        &self,
        state: &Self::State,
        derivative: &Self::Derivative,
        dt: f64,
    ) -> Self::State;

    fn combine(
        &self,
        state: &Self::State,
        weighted: Vec<(Self::Derivative, f64)>,
        step: f64,
    ) -> (Self::State, Self::StepOutput);

    /// Reduces two per-step outputs into one, used by adaptive integrators to
    /// combine the [`StepOutput`](Self::StepOutput) of each internal sub-step
    /// into a single output for the whole outer step.
    ///
    /// The right implementation depends on what the output *means*:
    /// - **extensive / accumulated** quantities (impulse, Δv, work, propellant)
    ///   sum: `accumulated + next`;
    /// - **snapshot** quantities (a value sampled at the step's end) keep the
    ///   latest: `next`;
    /// - **absent** outputs (`StepOutput = ()`) merge trivially: `()`.
    ///
    /// Fixed-step integrators call [`combine`](Self::combine) exactly once and
    /// never invoke this, so a single fixed step is unaffected by the choice.
    fn merge_step_outputs(
        &self,
        accumulated: Self::StepOutput,
        next: Self::StepOutput,
    ) -> Self::StepOutput;

    /// Forms the stage state `state + step * Σ weightᵢ · derivativeᵢ`.
    ///
    /// General explicit Runge-Kutta methods build each stage from a *linear
    /// combination* of several previously evaluated derivatives (the `a` row of
    /// the Butcher tableau). Because [`state_with_derivative`](Self::state_with_derivative)
    /// is linear and returns a fresh state, the combination can be assembled by
    /// chaining it, so implementors get this for free. Terms with a zero weight
    /// are skipped (adding `0.0` is exact and only wastes an allocation).
    ///
    /// An empty `weighted` slice yields `state.clone()`, i.e. the first RK stage.
    fn state_with_derivatives(
        &self,
        state: &Self::State,
        weighted: &[(&Self::Derivative, f64)],
        step: f64,
    ) -> Self::State {
        let mut trial = state.clone();
        for &(derivative, weight) in weighted {
            if weight == 0.0 {
                continue;
            }
            trial = self.state_with_derivative(&trial, derivative, step * weight);
        }
        trial
    }

    /// Weighted RMS norm of the difference between two candidate next-states,
    /// used by adaptive integrators for step-size control.
    ///
    /// Each scalar component `i` is scaled by `atol + rtol * max(|highᵢ|, |lowᵢ|)`
    /// and the returned value is the root-mean-square over all components, so a
    /// result `<= 1.0` means the step meets the requested tolerance.
    ///
    /// The default returns `None`, signalling that the object does not support
    /// adaptive integration; adaptive drivers then fall back to a single fixed
    /// step. Implement it to enable error-controlled stepping.
    fn error_norm(
        &self,
        _high: &Self::State,
        _low: &Self::State,
        _atol: f64,
        _rtol: f64,
    ) -> Option<f64> {
        None
    }

    /// Upstream `svIntegratorAdaptiveRungeKutta::computeFlatMaxRelativeError`:
    /// the largest error-to-tolerance ratio over the object's states of the
    /// two embedded candidates `state + step · Σ wᵢ kᵢ`, `low` the `b` weights
    /// (upstream `bArray`) and `high` the `b_embedded` ones (`bStarArray`).
    /// A result `<= 1.0` accepts the trial; used by
    /// [`StepControl::Upstream`](super::StepControl::Upstream).
    ///
    /// Upstream compares each registered state as a whole
    /// (`ErrorControlMode::WholeState`): the norm of `high - low` over
    /// `‖high‖ rel_tol + abs_tol`, see
    /// [`relative_error_ratio`](super::relative_error_ratio); a state with
    /// `ErrorControlMode::PerComponent` compares each scalar instead. The
    /// candidates are given as weights rather than states so an object can
    /// include quantities it integrates outside its state, as upstream
    /// integrates them as states.
    ///
    /// The default returns `None`: the object does not support upstream's
    /// error control, and the driver takes the whole step at once.
    #[allow(clippy::too_many_arguments)]
    fn max_relative_error(
        &self,
        _state: &Self::State,
        _low: &[(&Self::Derivative, f64)],
        _high: &[(&Self::Derivative, f64)],
        _step: f64,
        _abs_tol: f64,
        _rel_tol: f64,
    ) -> Option<f64> {
        None
    }

    /// Number of independent Wiener processes `W_k` in
    /// `dx = f(t, x) dt + Σ_k g_k(t, x) dW_k` (upstream: the global noise
    /// sources of the registered states).
    ///
    /// The default is `0`: a deterministic object. Stochastic integrators then
    /// reduce to their drift tableau, and deterministic integrators ignore any
    /// diffusion an object declares.
    fn noise_source_count(&self) -> usize {
        0
    }

    /// Evaluates the diffusion columns `g_k(t, x)` at `state`, one per noise
    /// source and in source order (upstream `equationsOfMotionDiffusion`).
    ///
    /// Each column is expressed as a [`Derivative`](Self::Derivative): the
    /// state change per unit Wiener increment, so stochastic integrators build
    /// stage states with [`state_with_derivatives`](Self::state_with_derivatives)
    /// exactly as they apply drift. Components without noise are zero. Takes
    /// `&mut self` for the same reason as
    /// [`equations_of_motion`](Self::equations_of_motion).
    ///
    /// The default returns no columns, matching the default
    /// [`noise_source_count`](Self::noise_source_count) of `0`; an object
    /// overriding one must override both.
    fn equations_of_motion_diffusion(
        &mut self,
        _state: &Self::State,
        _current_sim_nanos: u64,
        _current_epoch: Epoch,
    ) -> Vec<Self::Derivative> {
        Vec::new()
    }

    /// Applies the diffusion increments to `state` in order (upstream
    /// `applyStochasticNoiseInLocalOrder`): each one adds
    /// `pseudo_step · Σ weightᵢ · tangentᵢ` through
    /// [`state_with_derivatives`](Self::state_with_derivatives). An empty
    /// slice returns `state.clone()`.
    fn state_with_diffusion(
        &self,
        state: &Self::State,
        diffusion: &[DiffusionIncrement<'_, Self::Derivative>],
    ) -> Self::State {
        let mut next = state.clone();
        for increment in diffusion {
            next = self.state_with_derivatives(&next, &increment.tangents, increment.pseudo_step);
        }
        next
    }

    /// The final update of a stochastic step: [`combine`](Self::combine) the
    /// weighted drift over `step`, then add the diffusion increments with
    /// [`state_with_diffusion`](Self::state_with_diffusion). The step output
    /// is the drift's, as diffusion carries no step output.
    ///
    /// Upstream also forms the drift candidate first and then adds the noise
    /// terms. Override this when the state needs the noise inside a
    /// non-linear projection that `combine` applies.
    fn combine_with_diffusion(
        &self,
        state: &Self::State,
        drift: Vec<(Self::Derivative, f64)>,
        step: f64,
        diffusion: &[DiffusionIncrement<'_, Self::Derivative>],
    ) -> (Self::State, Self::StepOutput) {
        let (drifted, output) = self.combine(state, drift, step);
        (self.state_with_diffusion(&drifted, diffusion), output)
    }
}
