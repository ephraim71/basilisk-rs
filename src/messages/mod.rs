use std::sync::{Arc, RwLock};

mod atmosphere;
mod attitude_guidance;
mod attitude_reference;
mod body_torque_command;
mod eclipse;
mod gps;
mod imu;
mod magnetic_field;
mod mtb_command;
mod planet_state;
mod reaction_wheel_command;
mod spacecraft_diagnostics;
mod spacecraft_mass_props;
mod spacecraft_state;
mod star_tracker;
mod sun_ephemeris;
mod sun_line;
mod sun_sensor;
mod tam;
mod thruster_command;
mod translation_reference;

pub use atmosphere::AtmosphereMsg;
pub use attitude_guidance::AttitudeGuidanceMsg;
pub use attitude_reference::AttitudeReferenceMsg;
pub use body_torque_command::BodyTorqueCommandMsg;
pub use eclipse::EclipseMsg;
pub use gps::GpsMsg;
pub use imu::ImuMsg;
pub use magnetic_field::MagneticFieldMsg;
pub use mtb_command::MtbCommandMsg;
pub use planet_state::PlanetStateMsg;
pub use reaction_wheel_command::ReactionWheelCommandMsg;
pub use spacecraft_diagnostics::SpacecraftDiagnosticsMsg;
pub use spacecraft_mass_props::SpacecraftMassPropsMsg;
pub use spacecraft_state::SpacecraftStateMsg;
pub use star_tracker::StarTrackerMsg;
pub use sun_ephemeris::SunEphemerisMsg;
pub use sun_line::SunLineMsg;
pub use sun_sensor::SunSensorMsg;
pub use tam::TamMsg;
pub use thruster_command::ThrusterCommandMsg;
pub use translation_reference::TranslationReferenceMsg;

#[derive(Clone, Debug)]
pub struct Output<T> {
    slot: Arc<RwLock<T>>,
}

impl<T> Output<T> {
    pub fn new(initial: T) -> Self {
        Self {
            slot: Arc::new(RwLock::new(initial)),
        }
    }

    pub fn write(&self, value: T) {
        *self
            .slot
            .write()
            .expect("failed to lock output message for write") = value;
    }

    pub(crate) fn slot(&self) -> Arc<RwLock<T>> {
        Arc::clone(&self.slot)
    }
}

impl<T: Clone> Output<T> {
    pub fn read(&self) -> T {
        self.slot
            .read()
            .expect("failed to lock output message for read")
            .clone()
    }
}

impl<T: Default> Default for Output<T> {
    fn default() -> Self {
        Self::new(T::default())
    }
}

#[derive(Clone, Debug)]
pub struct Input<T> {
    slot: Option<Arc<RwLock<T>>>,
}

impl<T> Default for Input<T> {
    fn default() -> Self {
        Self { slot: None }
    }
}

impl<T> Input<T> {
    pub fn is_connected(&self) -> bool {
        self.slot.is_some()
    }

    pub(crate) fn connect(&mut self, slot: Arc<RwLock<T>>) {
        self.slot = Some(slot);
    }
}

impl<T: Clone + Default> Input<T> {
    pub fn read(&self) -> T {
        self.slot
            .as_ref()
            .map(|slot| {
                slot.read()
                    .expect("failed to lock input message for read")
                    .clone()
            })
            .unwrap_or_default()
    }
}
