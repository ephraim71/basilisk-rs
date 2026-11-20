//! Read and set the hub's translational state between simulation runs.
//!
//! Upstream scenarios change the hub state between `ExecuteSimulation` calls
//! through the state objects of the dynamics manager:
//!
//! ```python
//! velRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubVelocity)
//! velRef.setState(velRef.getState() + dv)
//! ```
//!
//! A [`crate::simulation::Simulation`] holds its modules mutably borrowed while
//! it exists, so the spacecraft cannot be reached directly between
//! [`run_for`](crate::simulation::Simulation::run_for) calls. A
//! [`HubStateHandle`] (from [`crate::spacecraft::Spacecraft::hub_state_handle`])
//! is the stand-in for the state objects: a shared slot, like a message
//! [`Output`](crate::messages::Output), holding the hub position and velocity
//! the spacecraft last committed. A value set through it is taken at the start
//! of the spacecraft's next update, before that update integrates, which is
//! where upstream's next integration step starts from the changed state.
//!
//! Positions and velocities are those the spacecraft integrates: of the body
//! origin `B`, relative to the central gravity body when there is one (upstream
//! `hubPosition`/`hubVelocity`), in inertial components.

use std::sync::{Arc, Mutex, MutexGuard};

use nalgebra::Vector3;

/// Shared handle on a spacecraft hub's position and velocity: upstream
/// `getStateObject(nameOfHubPosition / nameOfHubVelocity)`. Clones share one
/// slot.
#[derive(Clone, Debug, Default)]
pub struct HubStateHandle {
    shared: Arc<Mutex<HubStateSlot>>,
}

#[derive(Clone, Copy, Debug, Default)]
struct HubStateSlot {
    position_m: Vector3<f64>,
    velocity_mps: Vector3<f64>,
    pending_position: bool,
    pending_velocity: bool,
}

impl HubStateHandle {
    fn slot(&self) -> MutexGuard<'_, HubStateSlot> {
        self.shared
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// Hub position relative to the central body (m): upstream
    /// `posRef.getState()`. After a [`Self::set_position_m`] it is the value
    /// set, until the spacecraft integrates from it.
    pub fn position_m(&self) -> Vector3<f64> {
        self.slot().position_m
    }

    /// Hub velocity relative to the central body (m/s): upstream
    /// `velRef.getState()`.
    pub fn velocity_mps(&self) -> Vector3<f64> {
        self.slot().velocity_mps
    }

    /// Replaces the hub position (m) from the next spacecraft update on:
    /// upstream `posRef.setState(r)`.
    pub fn set_position_m(&self, position_m: Vector3<f64>) {
        let mut slot = self.slot();
        slot.position_m = position_m;
        slot.pending_position = true;
    }

    /// Replaces the hub velocity (m/s) from the next spacecraft update on:
    /// upstream `velRef.setState(v)`.
    pub fn set_velocity_mps(&self, velocity_mps: Vector3<f64>) {
        let mut slot = self.slot();
        slot.velocity_mps = velocity_mps;
        slot.pending_velocity = true;
    }

    /// The position and velocity set and not yet taken, each `None` when not
    /// set; clears them.
    pub(crate) fn take_pending(&self) -> (Option<Vector3<f64>>, Option<Vector3<f64>>) {
        let mut slot = self.slot();
        let position = slot.pending_position.then_some(slot.position_m);
        let velocity = slot.pending_velocity.then_some(slot.velocity_mps);
        slot.pending_position = false;
        slot.pending_velocity = false;
        (position, velocity)
    }

    /// Records the state the spacecraft committed.
    pub(crate) fn publish(&self, position_m: Vector3<f64>, velocity_mps: Vector3<f64>) {
        let mut slot = self.slot();
        if !slot.pending_position {
            slot.position_m = position_m;
        }
        if !slot.pending_velocity {
            slot.velocity_mps = velocity_mps;
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::HubStateHandle;

    #[test]
    fn a_set_value_survives_publish_until_taken() {
        let handle = HubStateHandle::default();
        handle.publish(Vector3::x(), Vector3::y());
        assert_eq!(handle.position_m(), Vector3::x());

        let shared = handle.clone();
        shared.set_velocity_mps(Vector3::z());
        handle.publish(Vector3::x() * 2.0, Vector3::y() * 2.0);
        assert_eq!(handle.position_m(), Vector3::x() * 2.0);
        assert_eq!(handle.velocity_mps(), Vector3::z());

        assert_eq!(handle.take_pending(), (None, Some(Vector3::z())));
        assert_eq!(handle.take_pending(), (None, None));
        handle.publish(Vector3::zeros(), Vector3::zeros());
        assert_eq!(handle.velocity_mps(), Vector3::zeros());
    }
}
