use hifitime::Epoch;

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
}
