use crate::messages::{
    Input, Output, PowerNodeUsageMsg, PowerStorageFaultMsg, PowerStorageStatusMsg,
};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct SimpleBatteryConfig {
    pub name: String,
    pub storage_capacity_j: f64,
    pub stored_charge_init_j: f64,
}

#[derive(Clone, Debug)]
pub struct SimpleBattery {
    pub config: SimpleBatteryConfig,
    pub power_node_in_msgs: Vec<Input<PowerNodeUsageMsg>>,
    pub battery_fault_in_msg: Input<PowerStorageFaultMsg>,
    pub bat_power_out_msg: Output<PowerStorageStatusMsg>,
    pub stored_charge_j: f64,
    previous_time_s: f64,
}

impl SimpleBattery {
    pub fn new(config: SimpleBatteryConfig) -> Self {
        Self {
            stored_charge_j: config.stored_charge_init_j,
            config,
            power_node_in_msgs: Vec::new(),
            battery_fault_in_msg: Input::default(),
            bat_power_out_msg: Output::default(),
            previous_time_s: 0.0,
        }
    }

    pub fn add_power_node_to_model(&mut self) -> &mut Input<PowerNodeUsageMsg> {
        self.power_node_in_msgs.push(Input::default());
        self.power_node_in_msgs
            .last_mut()
            .expect("just pushed a power node input")
    }

    pub fn current_net_power_w(&self) -> f64 {
        self.power_node_in_msgs
            .iter()
            .map(|input| input.read().net_power_w)
            .sum()
    }

    fn evaluate_battery_model(
        &mut self,
        current_net_power_w: f64,
        dt_s: f64,
    ) -> PowerStorageStatusMsg {
        self.stored_charge_j += current_net_power_w * dt_s;
        self.stored_charge_j = self
            .stored_charge_j
            .clamp(0.0, self.config.storage_capacity_j);

        let fault_capacity_ratio = if self.battery_fault_in_msg.is_connected() {
            self.battery_fault_in_msg.read().fault_capacity_ratio
        } else {
            1.0
        };
        assert!(
            (0.0..=1.0).contains(&fault_capacity_ratio),
            "simple battery '{}' fault_capacity_ratio must be between 0 and 1",
            self.config.name
        );

        self.stored_charge_j = self
            .stored_charge_j
            .min(self.config.storage_capacity_j * fault_capacity_ratio);

        PowerStorageStatusMsg {
            storage_level_j: self.stored_charge_j,
            storage_capacity_j: self.config.storage_capacity_j,
            current_net_power_w,
        }
    }
}

impl Module for SimpleBattery {
    fn init(&mut self) {
        assert!(
            self.config.storage_capacity_j > 0.0,
            "simple battery '{}' storage_capacity_j must be positive",
            self.config.name
        );
        assert!(
            self.config.stored_charge_init_j >= 0.0,
            "simple battery '{}' stored_charge_init_j must be non-negative",
            self.config.name
        );
        self.previous_time_s = 0.0;
        self.stored_charge_j = self
            .config
            .stored_charge_init_j
            .min(self.config.storage_capacity_j);
        self.bat_power_out_msg.write(PowerStorageStatusMsg {
            storage_level_j: self.stored_charge_j,
            storage_capacity_j: self.config.storage_capacity_j,
            current_net_power_w: 0.0,
        });
    }

    fn update(&mut self, context: &SimulationContext) {
        if self.power_node_in_msgs.is_empty() {
            self.bat_power_out_msg
                .write(PowerStorageStatusMsg::default());
            return;
        }

        let current_time_s = context.current_sim_nanos as f64 * 1.0e-9;
        let dt_s = current_time_s - self.previous_time_s;
        let current_net_power_w = self.current_net_power_w();
        let status = self.evaluate_battery_model(current_net_power_w, dt_s);
        self.previous_time_s = current_time_s;
        self.bat_power_out_msg.write(status);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;

    use crate::messages::{Output, PowerNodeUsageMsg, PowerStorageFaultMsg};
    use crate::{Module, SimulationContext};

    use super::{SimpleBattery, SimpleBatteryConfig};

    const STEP_NANOS: u64 = 100_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn battery(stored_charge_init_j: f64, storage_capacity_j: f64) -> SimpleBattery {
        SimpleBattery::new(SimpleBatteryConfig {
            name: "simpleBattery".to_string(),
            storage_capacity_j,
            stored_charge_init_j,
        })
    }

    #[test]
    fn storage_limits_match_basilisk_unit_test() {
        let power_1 = Output::new(PowerNodeUsageMsg { net_power_w: 5.0 });
        let power_2 = Output::new(PowerNodeUsageMsg { net_power_w: 5.0 });
        let mut battery = battery(5.0, 10.0);
        battery.add_power_node_to_model().connect(power_1.slot());
        battery.add_power_node_to_model().connect(power_2.slot());

        battery.init();
        for step in 0..=50 {
            battery.update(&context(step * STEP_NANOS));
            let status = battery.bat_power_out_msg.read();
            assert!((status.current_net_power_w - 10.0).abs() < 1.0e-12);
            assert!(status.storage_level_j <= status.storage_capacity_j);
            assert!(status.storage_level_j >= 0.0);
        }

        let status = battery.bat_power_out_msg.read();
        assert!((status.storage_level_j - 10.0).abs() < 1.0e-8);
        assert!((status.storage_capacity_j - 10.0).abs() < 1.0e-12);
    }

    #[test]
    fn no_power_nodes_writes_zero_status_like_power_storage_base() {
        let mut battery = battery(5.0, 10.0);

        battery.init();
        battery.update(&context(STEP_NANOS));

        let status = battery.bat_power_out_msg.read();
        assert_eq!(status.storage_level_j, 0.0);
        assert_eq!(status.storage_capacity_j, 0.0);
        assert_eq!(status.current_net_power_w, 0.0);
    }

    #[test]
    fn fault_capacity_ratio_limits_stored_charge_like_basilisk_unit_test() {
        let cases = [
            (5.0, 5.0, 5.0, 10.0, 0.3),
            (1.0, 1.0, 5.0, 10.0, 0.3),
            (5.0, 5.0, 5.0, 10.0, 0.0),
            (5.0, 5.0, 5.0, 10.0, 1.0),
            (5.0, 5.0, 5.0, 10.0, 1.0e-3),
            (5.0, -5.0, 5.0, 10.0, 0.5),
        ];

        for (stored_charge_init_j, net_power_1_w, net_power_2_w, capacity_j, fault_ratio) in cases {
            let power_1 = Output::new(PowerNodeUsageMsg {
                net_power_w: net_power_1_w,
            });
            let power_2 = Output::new(PowerNodeUsageMsg {
                net_power_w: net_power_2_w,
            });
            let fault = Output::new(PowerStorageFaultMsg {
                fault_capacity_ratio: fault_ratio,
            });
            let mut battery = battery(stored_charge_init_j, capacity_j);
            battery.add_power_node_to_model().connect(power_1.slot());
            battery.add_power_node_to_model().connect(power_2.slot());
            battery.battery_fault_in_msg.connect(fault.slot());

            battery.init();
            for step in 0..=50 {
                battery.update(&context(step * STEP_NANOS));
                let status = battery.bat_power_out_msg.read();
                assert!((status.storage_capacity_j - capacity_j).abs() < 1.0e-12);
                assert!(status.storage_level_j <= capacity_j * fault_ratio + 1.0e-12);
                assert!(status.storage_level_j >= 0.0);
            }
        }
    }
}
