use crate::messages::{ArrayMotorTorqueMsg, ArrayMotorVoltageMsg, Input, MAX_EFF_COUNT, Output};
use crate::{Module, SimulationContext};

/// Configuration for converting reaction-wheel motor voltages into torques.
#[derive(Clone, Debug)]
pub struct MotorVoltageInterfaceConfig {
    pub name: String,
    /// Active wheel gains in N m/V. The vector order defines the aggregate
    /// reaction-wheel order; slots after this vector are inactive and zero.
    pub torque_gains_nm_per_v: Vec<f64>,
}

impl MotorVoltageInterfaceConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.torque_gains_nm_per_v.len() > MAX_EFF_COUNT {
            return Err(format!(
                "at most {MAX_EFF_COUNT} motor voltage gains are supported, got {}",
                self.torque_gains_nm_per_v.len()
            ));
        }
        if let Some((index, gain)) = self
            .torque_gains_nm_per_v
            .iter()
            .enumerate()
            .find(|(_, gain)| !gain.is_finite())
        {
            return Err(format!(
                "torque_gains_nm_per_v[{index}] must be finite, got {gain}"
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct MotorVoltageInterface {
    pub config: MotorVoltageInterfaceConfig,
    pub voltage_in_msg: Input<ArrayMotorVoltageMsg>,
    pub motor_torque_out_msg: Output<ArrayMotorTorqueMsg>,
}

impl MotorVoltageInterface {
    pub fn new(config: MotorVoltageInterfaceConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid MotorVoltageInterfaceConfig: {message}");
        }
        Self {
            config,
            voltage_in_msg: Input::default(),
            motor_torque_out_msg: Output::default(),
        }
    }

    fn map_voltage_to_torque(&self, voltage: &ArrayMotorVoltageMsg) -> ArrayMotorTorqueMsg {
        let mut torque = ArrayMotorTorqueMsg::default();
        for (index, gain_nm_per_v) in self.config.torque_gains_nm_per_v.iter().enumerate() {
            torque.motor_torque_nm[index] = voltage.voltage_v[index] * gain_nm_per_v;
        }
        torque
    }
}

impl Module for MotorVoltageInterface {
    fn init(&mut self) {
        if !self.voltage_in_msg.is_connected() {
            log::error!(
                "motor voltage interface '{}' voltage_in_msg is not connected",
                self.config.name
            );
        }
        self.motor_torque_out_msg
            .write(ArrayMotorTorqueMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let voltage = self.voltage_in_msg.read();
        self.motor_torque_out_msg
            .write(self.map_voltage_to_torque(&voltage));
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;

    use crate::messages::{ArrayMotorVoltageMsg, MAX_EFF_COUNT, Output};
    use crate::{Module, SimulationContext};

    use super::{MotorVoltageInterface, MotorVoltageInterfaceConfig};

    fn context() -> SimulationContext {
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    #[test]
    fn maps_active_voltage_slots_and_zero_fills_inactive_slots() {
        let mut voltage = ArrayMotorVoltageMsg::default();
        voltage.voltage_v[..4].copy_from_slice(&[10.0, -5.0, 2.5, 99.0]);
        let voltage_out = Output::new(voltage);
        let mut interface = MotorVoltageInterface::new(MotorVoltageInterfaceConfig {
            name: "motor_voltage_interface".to_string(),
            torque_gains_nm_per_v: vec![0.02, 0.04, -0.1],
        });
        interface.voltage_in_msg.connect(voltage_out.slot());

        interface.init();
        interface.update(&context());

        let torque = interface.motor_torque_out_msg.read();
        assert_eq!(torque.motor_torque_nm[..3], [0.2, -0.2, -0.25]);
        assert!(
            torque.motor_torque_nm[3..]
                .iter()
                .all(|value| *value == 0.0)
        );
    }

    #[test]
    #[should_panic(expected = "at most 36 motor voltage gains are supported")]
    fn rejects_more_than_maximum_effectors() {
        let _ = MotorVoltageInterface::new(MotorVoltageInterfaceConfig {
            name: "too_many".to_string(),
            torque_gains_nm_per_v: vec![1.0; MAX_EFF_COUNT + 1],
        });
    }

    #[test]
    #[should_panic(expected = "must be finite")]
    fn rejects_non_finite_gain() {
        let _ = MotorVoltageInterface::new(MotorVoltageInterfaceConfig {
            name: "non_finite".to_string(),
            torque_gains_nm_per_v: vec![f64::NAN],
        });
    }
}
