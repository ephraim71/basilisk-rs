use serde::{Serialize, de::DeserializeOwned};
use serde_json::Value;
use std::sync::{Arc, RwLock};

mod array_motor_torque;
mod array_motor_voltage;
mod atmosphere;
mod attitude_guidance;
mod attitude_reference;
mod body_torque_command;
mod eclipse;
mod gps;
mod hinged_rigid_body;
mod imu;
mod magnetic_field;
mod mtb_array_command;
mod mtb_array_config;
mod navigation_attitude;
mod planet_state;
mod power_node_usage;
mod power_storage_fault;
mod power_storage_status;
mod reaction_wheel_state;
mod rw_array_config;
mod rw_availability;
mod rw_speed;
mod solar_flux;
mod spacecraft_diagnostics;
mod spacecraft_mass_props;
mod spacecraft_state;
mod star_tracker;
mod sun_ephemeris;
mod sun_line;
mod sun_sensor;
mod tam_sensor;
mod tam_sensor_body;
mod thruster_command;
mod translation_reference;
mod vehicle_config;

pub use array_motor_torque::ArrayMotorTorqueMsg;
pub use array_motor_voltage::ArrayMotorVoltageMsg;
pub use atmosphere::AtmosphereMsg;
pub use attitude_guidance::AttitudeGuidanceMsg;
pub use attitude_reference::AttitudeReferenceMsg;
pub use body_torque_command::BodyTorqueCommandMsg;
pub use eclipse::EclipseMsg;
pub use gps::GpsMsg;
pub use hinged_rigid_body::HingedRigidBodyMsg;
pub use imu::ImuMsg;
pub use magnetic_field::MagneticFieldMsg;
pub use mtb_array_command::MtbArrayCommandMsg;
pub use mtb_array_config::MtbArrayConfigMsg;
pub use navigation_attitude::NavigationAttitudeMsg;
pub use planet_state::{PlanetOrientation, PlanetStateMsg};
pub use power_node_usage::PowerNodeUsageMsg;
pub use power_storage_fault::PowerStorageFaultMsg;
pub use power_storage_status::PowerStorageStatusMsg;
pub use reaction_wheel_state::ReactionWheelStateMsg;
pub use rw_array_config::RwArrayConfigMsg;
pub use rw_availability::{RwAvailability, RwAvailabilityMsg};
pub use rw_speed::RwSpeedMsg;
pub use solar_flux::SolarFluxMsg;
pub use spacecraft_diagnostics::SpacecraftDiagnosticsMsg;
pub use spacecraft_mass_props::SpacecraftMassPropsMsg;
pub use spacecraft_state::SpacecraftStateMsg;
pub use star_tracker::StarTrackerMsg;
pub use sun_ephemeris::SunEphemerisMsg;
pub use sun_line::SunLineMsg;
pub use sun_sensor::SunSensorMsg;
pub use tam_sensor::TamSensorMsg;
pub use tam_sensor_body::TamSensorBodyMsg;
pub use thruster_command::ThrusterCommandMsg;
pub use translation_reference::TranslationReferenceMsg;
pub use vehicle_config::VehicleConfigMsg;

pub const MAX_EFF_COUNT: usize = 36;

/// Serde support for the `[T; MAX_EFF_COUNT]` effector arrays.
///
/// `serde` only provides array impls up to length 32, and `MAX_EFF_COUNT` is
/// larger, so the arrays are carried over the wire as sequences instead.
pub(crate) mod big_array {
    use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Error as _};

    pub fn serialize<S, T, const N: usize>(value: &[T; N], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
        T: Serialize,
    {
        value.as_slice().serialize(serializer)
    }

    pub fn deserialize<'de, D, T, const N: usize>(deserializer: D) -> Result<[T; N], D::Error>
    where
        D: Deserializer<'de>,
        T: Deserialize<'de>,
    {
        let values = Vec::<T>::deserialize(deserializer)?;
        let len = values.len();
        <[T; N]>::try_from(values).map_err(|_| D::Error::invalid_length(len, &"MAX_EFF_COUNT"))
    }
}

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
        let raw = self
            .raw_slot
            .read()
            .expect("failed to lock raw output message for read")
            .clone();
        let existing = self
            .override_rule
            .read()
            .expect("failed to lock output override for read")
            .clone();
        if mode == MessageOverrideMode::Patch
            && let Some(existing) = existing
            && existing.mode == MessageOverrideMode::Patch
        {
            let mut merged = existing.value;
            merge_json(&mut merged, stored_value);
            stored_value = merged;
        }

        let rule = MessageOverride {
            mode,
            value: stored_value,
        };
        let effective = apply_override_rule(raw, &rule)?;

        *self
            .override_rule
            .write()
            .expect("failed to lock output override for write") = Some(rule);
        *self
            .slot
            .write()
            .expect("failed to lock output message for write") = effective;
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

        apply_override_rule(raw, &rule)
    }
}

fn apply_override_rule<T>(raw: T, rule: &MessageOverride) -> Result<T, serde_json::Error>
where
    T: SimulationMessage,
{
    match rule.mode {
        MessageOverrideMode::Replace | MessageOverrideMode::Freeze => {
            serde_json::from_value(rule.value.clone())
        }
        MessageOverrideMode::Patch => {
            let mut base = serde_json::to_value(raw)?;
            merge_json(&mut base, rule.value.clone());
            serde_json::from_value(base)
        }
        MessageOverrideMode::Default => Ok(T::default()),
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

    use super::{ArrayMotorTorqueMsg, MessageOverrideMode, Output};
    use crate::messages::{MAX_EFF_COUNT, PowerStorageStatusMsg};

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
            .set_override(
                MessageOverrideMode::Patch,
                json!({ "storage_level_j": 1.0 }),
            )
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

    #[test]
    fn replace_override_substitutes_the_complete_message() {
        let output = Output::new(PowerStorageStatusMsg::default());

        output
            .set_override(
                MessageOverrideMode::Replace,
                json!({
                    "storage_level_j": 4.0,
                    "storage_capacity_j": 5.0,
                    "current_net_power_w": 6.0,
                }),
            )
            .unwrap();
        output.write(PowerStorageStatusMsg {
            storage_level_j: 10.0,
            storage_capacity_j: 20.0,
            current_net_power_w: 30.0,
        });

        assert_eq!(output.read().storage_level_j, 4.0);
        assert_eq!(output.read().storage_capacity_j, 5.0);
        assert_eq!(output.read().current_net_power_w, 6.0);
    }

    #[test]
    fn default_override_emits_default_until_cleared() {
        let output = Output::new(PowerStorageStatusMsg {
            storage_level_j: 1.0,
            storage_capacity_j: 2.0,
            current_net_power_w: 3.0,
        });

        output
            .set_override(MessageOverrideMode::Default, json!(null))
            .unwrap();
        output.write(PowerStorageStatusMsg {
            storage_level_j: 10.0,
            storage_capacity_j: 20.0,
            current_net_power_w: 30.0,
        });
        assert_eq!(output.read().storage_level_j, 0.0);
        assert_eq!(output.read().storage_capacity_j, 0.0);
        assert_eq!(output.read().current_net_power_w, 0.0);

        output.clear_override();
        assert_eq!(output.read().storage_level_j, 10.0);
        assert_eq!(output.read().storage_capacity_j, 20.0);
        assert_eq!(output.read().current_net_power_w, 30.0);
    }

    #[test]
    fn malformed_replace_is_rejected_without_installing_the_override() {
        let output = Output::new(PowerStorageStatusMsg::default());

        let result = output.set_override(
            MessageOverrideMode::Replace,
            json!({ "storage_level_j": "not a number" }),
        );
        assert!(result.is_err());

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
    fn malformed_successive_patch_preserves_the_previous_valid_patch() {
        let output = Output::new(PowerStorageStatusMsg::default());

        output
            .set_override(
                MessageOverrideMode::Patch,
                json!({ "current_net_power_w": 9.0 }),
            )
            .unwrap();
        let result = output.set_override(
            MessageOverrideMode::Patch,
            json!({ "storage_capacity_j": "not a number" }),
        );
        assert!(result.is_err());

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
    fn maximum_effector_array_round_trips_and_rejects_wrong_lengths() {
        let mut message = ArrayMotorTorqueMsg::default();
        for (index, value) in message.motor_torque_nm.iter_mut().enumerate() {
            *value = index as f64 + 0.25;
        }

        let serialized = serde_json::to_value(&message).unwrap();
        let round_tripped: ArrayMotorTorqueMsg =
            serde_json::from_value(serialized.clone()).unwrap();
        assert_eq!(round_tripped, message);

        let mut short = serialized;
        short["motor_torque_nm"] = json!(vec![0.0; MAX_EFF_COUNT - 1]);
        assert!(serde_json::from_value::<ArrayMotorTorqueMsg>(short).is_err());
    }
}
