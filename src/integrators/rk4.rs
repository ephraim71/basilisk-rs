use super::traits::DynamicObject;
use hifitime::{Duration, Epoch};

pub fn propagate_rk4<D: DynamicObject>(
    object: &mut D,
    state: &D::State,
    current_sim_nanos: u64,
    current_epoch: Epoch,
    dt_seconds: f64,
) -> (D::State, D::StepOutput) {
    // Each k evaluates forces at a trial state. State effector outputs are synced
    // inside equations_of_motion so dynamic effectors see the correct trial values.
    let k1 = object.equations_of_motion(state, current_sim_nanos, current_epoch);
    let k2 = object.equations_of_motion(
        &object.state_with_derivative(state, &k1, dt_seconds * 0.5),
        current_sim_nanos + (dt_seconds * 0.5 * 1.0e9) as u64,
        current_epoch + Duration::from_seconds(dt_seconds * 0.5),
    );
    let k3 = object.equations_of_motion(
        &object.state_with_derivative(state, &k2, dt_seconds * 0.5),
        current_sim_nanos + (dt_seconds * 0.5 * 1.0e9) as u64,
        current_epoch + Duration::from_seconds(dt_seconds * 0.5),
    );
    let k4 = object.equations_of_motion(
        &object.state_with_derivative(state, &k3, dt_seconds),
        current_sim_nanos + (dt_seconds * 1.0e9) as u64,
        current_epoch + Duration::from_seconds(dt_seconds),
    );

    object.combine(
        state,
        vec![(k1, 1.0), (k2, 2.0), (k3, 2.0), (k4, 1.0)],
        dt_seconds / 6.0,
    )
}
