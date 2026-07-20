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
}
