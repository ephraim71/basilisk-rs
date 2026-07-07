use serde::{Serialize, de::DeserializeOwned};
use serde_json::Value;
use std::sync::{Arc, RwLock};

mod array_motor_torque;
mod atmosphere;
mod attitude_guidance;
mod attitude_reference;
mod body_torque_command;
mod eclipse;
mod gps;
mod hinged_rigid_body;
mod imu;
mod magnetic_field;
mod mtb_command;
mod planet_state;
mod power_node_usage;
mod power_storage_fault;
mod power_storage_status;
mod reaction_wheel_command;
mod reaction_wheel_state;
mod solar_flux;
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

pub use array_motor_torque::ArrayMotorTorqueMsg;
pub use atmosphere::AtmosphereMsg;
pub use attitude_guidance::AttitudeGuidanceMsg;
pub use attitude_reference::AttitudeReferenceMsg;
pub use body_torque_command::BodyTorqueCommandMsg;
pub use eclipse::EclipseMsg;
pub use gps::GpsMsg;
pub use hinged_rigid_body::HingedRigidBodyMsg;
pub use imu::ImuMsg;
pub use magnetic_field::MagneticFieldMsg;
pub use mtb_command::MtbCommandMsg;
pub use planet_state::PlanetStateMsg;
pub use power_node_usage::PowerNodeUsageMsg;
pub use power_storage_fault::PowerStorageFaultMsg;
pub use power_storage_status::PowerStorageStatusMsg;
pub use reaction_wheel_command::ReactionWheelCommandMsg;
pub use reaction_wheel_state::ReactionWheelStateMsg;
pub use solar_flux::SolarFluxMsg;
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MessageOverrideMode {
    Replace,
    Patch,
    Freeze,
    Default,
}

#[derive(Clone, Debug)]
struct MessageOverride {
    mode: MessageOverrideMode,
    value: Value,
}

pub trait SimulationMessage:
    Clone + Default + Serialize + DeserializeOwned + Send + Sync + 'static
{
}

impl<T> SimulationMessage for T where
    T: Clone + Default + Serialize + DeserializeOwned + Send + Sync + 'static
{
}

#[derive(Clone, Debug)]
pub struct Output<T> {
    slot: Arc<RwLock<T>>,
    raw_slot: Arc<RwLock<T>>,
    override_rule: Arc<RwLock<Option<MessageOverride>>>,
}

impl<T: Clone> Output<T> {
    pub fn new(initial: T) -> Self {
        Self {
            slot: Arc::new(RwLock::new(initial.clone())),
            raw_slot: Arc::new(RwLock::new(initial)),
            override_rule: Arc::new(RwLock::new(None)),
        }
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

impl<T> Output<T>
where
    T: SimulationMessage,
{
    pub fn write(&self, value: T) {
        *self
            .raw_slot
            .write()
            .expect("failed to lock raw output message for write") = value.clone();

        let effective = self.apply_override(value).unwrap_or_else(|err| {
            eprintln!("[message-override] failed to apply override: {err}");
            self.raw_slot
                .read()
                .expect("failed to lock raw output message for read")
                .clone()
        });

        *self
            .slot
            .write()
            .expect("failed to lock output message for write") = effective;
    }

    pub fn set_override(
        &self,
        mode: MessageOverrideMode,
        value: Value,
    ) -> Result<(), serde_json::Error> {
        let mut stored_value = match mode {
            MessageOverrideMode::Freeze => serde_json::to_value(self.read())?,
            _ => value,
        };
        {
            let mut override_rule = self
                .override_rule
                .write()
                .expect("failed to lock output override for write");
            if mode == MessageOverrideMode::Patch
                && let Some(existing) = override_rule.as_ref()
                && existing.mode == MessageOverrideMode::Patch
            {
                let mut merged = existing.value.clone();
                merge_json(&mut merged, stored_value);
                stored_value = merged;
            }
            *override_rule = Some(MessageOverride {
                mode,
                value: stored_value,
            });
        }

        let raw = self
            .raw_slot
            .read()
            .expect("failed to lock raw output message for read")
            .clone();
        self.write(raw);
        Ok(())
    }

    pub fn clear_override(&self) {
        *self
            .override_rule
            .write()
            .expect("failed to lock output override for write") = None;
        let raw = self
            .raw_slot
            .read()
            .expect("failed to lock raw output message for read")
            .clone();
        *self
            .slot
            .write()
            .expect("failed to lock output message for write") = raw;
    }

    fn apply_override(&self, raw: T) -> Result<T, serde_json::Error> {
        let Some(rule) = self
            .override_rule
            .read()
            .expect("failed to lock output override for read")
            .clone()
        else {
            return Ok(raw);
        };

        match rule.mode {
            MessageOverrideMode::Replace | MessageOverrideMode::Freeze => {
                serde_json::from_value(rule.value)
            }
            MessageOverrideMode::Patch => {
                let mut base = serde_json::to_value(raw)?;
                merge_json(&mut base, rule.value);
                serde_json::from_value(base)
            }
            MessageOverrideMode::Default => Ok(T::default()),
        }
    }
}

impl<T: Clone + Default> Default for Output<T> {
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

fn merge_json(base: &mut Value, patch: Value) {
    match (base, patch) {
        (Value::Object(base_map), Value::Object(patch_map)) => {
            for (key, patch_value) in patch_map {
                if let Some(base_value) = base_map.get_mut(&key) {
                    merge_json(base_value, patch_value);
                } else {
                    base_map.insert(key, patch_value);
                }
            }
        }
        (base_value, patch_value) => *base_value = patch_value,
    }
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::{MessageOverrideMode, Output};
    use crate::messages::PowerStorageStatusMsg;

    #[test]
    fn output_passes_raw_value_without_override() {
        let output = Output::new(PowerStorageStatusMsg::default());

        output.write(PowerStorageStatusMsg {
            storage_level_j: 1.0,
            storage_capacity_j: 2.0,
            current_net_power_w: 3.0,
        });

        assert_eq!(output.read().storage_level_j, 1.0);
        assert_eq!(output.read().storage_capacity_j, 2.0);
        assert_eq!(output.read().current_net_power_w, 3.0);
    }

    #[test]
    fn patch_override_updates_only_requested_fields() {
        let output = Output::new(PowerStorageStatusMsg::default());

        output
            .set_override(
                MessageOverrideMode::Patch,
                json!({ "current_net_power_w": 9.0 }),
            )
            .unwrap();
        output.write(PowerStorageStatusMsg {
            storage_level_j: 1.0,
            storage_capacity_j: 2.0,
            current_net_power_w: 3.0,
        });

        assert_eq!(output.read().storage_level_j, 1.0);
        assert_eq!(output.read().storage_capacity_j, 2.0);
        assert_eq!(output.read().current_net_power_w, 9.0);
    }

    #[test]
    fn patch_override_merges_successive_patches() {
        let output = Output::new(PowerStorageStatusMsg::default());

        output
            .set_override(MessageOverrideMode::Patch, json!({ "storage_level_j": 1.0 }))
            .unwrap();
        output
            .set_override(
                MessageOverrideMode::Patch,
                json!({ "current_net_power_w": 3.0 }),
            )
            .unwrap();
        output.write(PowerStorageStatusMsg {
            storage_level_j: 10.0,
            storage_capacity_j: 20.0,
            current_net_power_w: 30.0,
        });

        assert_eq!(output.read().storage_level_j, 1.0);
        assert_eq!(output.read().storage_capacity_j, 20.0);
        assert_eq!(output.read().current_net_power_w, 3.0);
    }

    #[test]
    fn clear_override_restores_latest_raw_value() {
        let output = Output::new(PowerStorageStatusMsg::default());

        output
            .set_override(
                MessageOverrideMode::Patch,
                json!({ "current_net_power_w": 9.0 }),
            )
            .unwrap();
        output.write(PowerStorageStatusMsg {
            storage_level_j: 1.0,
            storage_capacity_j: 2.0,
            current_net_power_w: 3.0,
        });
        output.clear_override();

        assert_eq!(output.read().current_net_power_w, 3.0);
    }

    #[test]
    fn freeze_override_holds_effective_value() {
        let output = Output::new(PowerStorageStatusMsg {
            storage_level_j: 1.0,
            storage_capacity_j: 2.0,
            current_net_power_w: 3.0,
        });

        output
            .set_override(MessageOverrideMode::Freeze, json!({}))
            .unwrap();
        output.write(PowerStorageStatusMsg {
            storage_level_j: 10.0,
            storage_capacity_j: 20.0,
            current_net_power_w: 30.0,
        });

        assert_eq!(output.read().storage_level_j, 1.0);
        assert_eq!(output.read().storage_capacity_j, 2.0);
        assert_eq!(output.read().current_net_power_w, 3.0);
    }
}
