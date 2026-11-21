//! Actuators of a scene: port of upstream Basilisk `MJActuator`
//! (`MJSingleActuator`, `MJForceActuator`, `MJTorqueActuator`,
//! `MJForceTorqueActuator`) from
//! `simulation/mujocoDynamics/_GeneralModuleFiles`.
//!
//! An actuator wraps one or more MuJoCo actuators (upstream
//! `MJActuatorObject`) and, on every dynamics evaluation, writes the controls
//! of those MuJoCo actuators from its input messages, or zero while an input
//! is unconnected. The composite actuators drive one unit-gear MuJoCo
//! actuator per site axis.
//!
//! Upstream reads `ForceAtSiteMsg` (`force_S`) and `TorqueAtSiteMsg`
//! (`torque_S`); basilisk-rs reuses its body-frame command messages for
//! them, the site frame `S` taking the place of the body frame:
//! [`BodyForceCommandMsg`] (`force_request_body_n` is `force_S`) and
//! [`BodyTorqueCommandMsg`] (`torque_request_body_nm` is `torque_S`).

use crate::messages::{BodyForceCommandMsg, BodyTorqueCommandMsg, Input, SingleActuatorMsg};

/// One MuJoCo actuator driven by a scene actuator (upstream
/// `MJActuatorObject`).
#[derive(Clone, Debug)]
pub(crate) struct ActuatorObject {
    pub(crate) name: String,
    pub(crate) id: usize,
}

impl ActuatorObject {
    pub(crate) fn new(name: String) -> Self {
        Self { name, id: 0 }
    }
}

/// An actuator with one control (upstream `MJSingleActuator`): a joint
/// motor, or a force and/or torque at a site along a fixed gear.
#[derive(Clone, Debug)]
pub struct MjSingleActuator {
    pub(crate) name: String,
    pub(crate) object: ActuatorObject,
    /// Upstream `actuatorInMsg`: the control value.
    pub actuator_in_msg: Input<SingleActuatorMsg>,
}

/// A force of arbitrary direction at a site (upstream `MJForceActuator`).
#[derive(Clone, Debug)]
pub struct MjForceActuator {
    pub(crate) name: String,
    pub(crate) objects: [ActuatorObject; 3],
    /// Upstream `forceInMsg` (`force_S`, site-frame components \[N\]).
    pub force_in_msg: Input<BodyForceCommandMsg>,
}

/// A torque of arbitrary direction at a site (upstream `MJTorqueActuator`).
#[derive(Clone, Debug)]
pub struct MjTorqueActuator {
    pub(crate) name: String,
    pub(crate) objects: [ActuatorObject; 3],
    /// Upstream `torqueInMsg` (`torque_S`, site-frame components \[N m\]).
    pub torque_in_msg: Input<BodyTorqueCommandMsg>,
}

/// A force and a torque of arbitrary direction at a site (upstream
/// `MJForceTorqueActuator`).
#[derive(Clone, Debug)]
pub struct MjForceTorqueActuator {
    pub(crate) name: String,
    pub(crate) objects: [ActuatorObject; 6],
    /// Upstream `forceInMsg` (`force_S` \[N\]).
    pub force_in_msg: Input<BodyForceCommandMsg>,
    /// Upstream `torqueInMsg` (`torque_S` \[N m\]).
    pub torque_in_msg: Input<BodyTorqueCommandMsg>,
}

macro_rules! actuator_name {
    ($($kind:ident),*) => {$(
        impl $kind {
            /// Upstream `getName`.
            pub fn name(&self) -> &str {
                &self.name
            }
        }
    )*};
}
actuator_name!(
    MjSingleActuator,
    MjForceActuator,
    MjTorqueActuator,
    MjForceTorqueActuator
);

/// Any scene actuator.
#[derive(Clone, Debug)]
pub(crate) enum MjActuator {
    Single(MjSingleActuator),
    Force(MjForceActuator),
    Torque(MjTorqueActuator),
    ForceTorque(MjForceTorqueActuator),
}

impl MjActuator {
    /// Builds the actuator for `objects` (upstream's constructors check the
    /// sub-actuator count, which the composite kinds fix here).
    pub(crate) fn single(name: String, object: String) -> Self {
        Self::Single(MjSingleActuator {
            name,
            object: ActuatorObject::new(object),
            actuator_in_msg: Input::default(),
        })
    }

    pub(crate) fn name(&self) -> &str {
        match self {
            Self::Single(actuator) => &actuator.name,
            Self::Force(actuator) => &actuator.name,
            Self::Torque(actuator) => &actuator.name,
            Self::ForceTorque(actuator) => &actuator.name,
        }
    }

    pub(crate) fn objects_mut(&mut self) -> &mut [ActuatorObject] {
        match self {
            Self::Single(actuator) => std::slice::from_mut(&mut actuator.object),
            Self::Force(actuator) => &mut actuator.objects,
            Self::Torque(actuator) => &mut actuator.objects,
            Self::ForceTorque(actuator) => &mut actuator.objects,
        }
    }

    /// Upstream `updateCtrl`: the controls of the MuJoCo actuators, from the
    /// inputs, or zero for an unconnected input.
    pub(crate) fn update_ctrl(&self, ctrl: &mut [f64]) {
        match self {
            Self::Single(actuator) => {
                ctrl[actuator.object.id] = if actuator.actuator_in_msg.is_connected() {
                    actuator.actuator_in_msg.read().input
                } else {
                    0.0
                };
            }
            Self::Force(actuator) => {
                let force = actuator.force_in_msg.read().force_request_body_n;
                for (object, value) in actuator.objects.iter().zip(force.iter()) {
                    ctrl[object.id] = *value;
                }
            }
            Self::Torque(actuator) => {
                let torque = actuator.torque_in_msg.read().torque_request_body_nm;
                for (object, value) in actuator.objects.iter().zip(torque.iter()) {
                    ctrl[object.id] = *value;
                }
            }
            Self::ForceTorque(actuator) => {
                let force = actuator.force_in_msg.read().force_request_body_n;
                let torque = actuator.torque_in_msg.read().torque_request_body_nm;
                for (index, value) in force.iter().chain(torque.iter()).enumerate() {
                    ctrl[actuator.objects[index].id] = *value;
                }
            }
        }
    }
}
