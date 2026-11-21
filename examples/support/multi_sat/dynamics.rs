//! Upstream `modelsMultiSat/BSK_MultiSatDynamics.py`: the dynamics process
//! of one MultiSat spacecraft.
//!
//! | upstream | here |
//! |---|---|
//! | `BSKDynamicModels(SimBase, dynRate, spacecraftIndex)` | [`Dynamics::new`] |
//! | `scObject` (750 kg, `diag(900, 800, 600)` kg m²) with the environment's bodies | [`Dynamics::spacecraft`] |
//! | `rwStateEffector`: four `Honeywell_HR16` (50 N m s) in a 40° pyramid | inside the spacecraft; [`Dynamics::rw_factory`] |
//! | `thrusterDynamicEffector`: one `MOOG_Monarc_90HT` along `+b3` | inside the spacecraft; [`Dynamics::thruster_factory`] |
//! | `fuelTankStateEffector` (uniform burn, 50 of 75 kg) feeding the thruster | inside the spacecraft |
//! | `simpleNavObject`, `simpleMassPropsObject` | [`Dynamics::simple_nav`], [`Dynamics::simple_mass_props`] |
//! | `solarPanel`, `powerSink` (-2 W), `rwPowerList` (5 W each), `powerMonitor` (120 W h, 60 %) | the power fields |
//! | `rwStateEffector.rwMotorCmdInMsg`, `thrusterDynamicEffector.cmdsInMsg` | [`Dynamics::rw_motor_cmd_in_msg`], [`Dynamics::thruster_cmd_in_msgs`] |
//!
//! basilisk-rs's effectors live inside the [`Spacecraft`] and read one
//! command per device. The flight software connects later, so the
//! spacecraft's command inputs are fed by two relays scheduled at the top of
//! the dynamics task: an `arrayMotorTorqueToSingleActuators` splitting the
//! wheels' array command (upstream's `rwMotorCmdInMsg`), and a [`Latch`] per
//! thruster (upstream's `cmdsInMsg`). They read the flight software's
//! command of the previous tick, as the effectors read it at this tick's
//! integration when connected directly, so the timing is unchanged; and a
//! command the mode events zero reaches the effectors, as upstream.
//!
//! As upstream, every solar panel reads `eclipseOutMsgs[0]` (upstream's
//! comment says "the earth message", but the eclipse outputs are per
//! spacecraft, so it is the first spacecraft's shadow). Upstream's
//! `ModelTag`s are kept as module names.

use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::fuel_tank::{FuelTank, FuelTankConfig, FuelTankModel, FuelTankShape};
use basilisk_rs::dynamics::reaction_wheel_state_effector::ReactionWheelStateEffector;
use basilisk_rs::dynamics::thruster_dynamic_effector::ThrusterDynamicEffector;
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, FuelTankMsg, Input, Output, ReactionWheelStateMsg, RwArrayConfigMsg,
    RwSpeedMsg, SpacecraftStateMsg, ThrusterArrayConfigMsg, ThrusterCommandMsg, ThrusterOutputMsg,
};
use basilisk_rs::power::reaction_wheel_power::{ReactionWheelPower, ReactionWheelPowerModel};
use basilisk_rs::power::simple_battery::{SimpleBattery, SimpleBatteryConfig};
use basilisk_rs::power::simple_power_sink::{SimplePowerSink, SimplePowerSinkModel};
use basilisk_rs::power::simple_solar_panel::{SimpleSolarPanel, SimpleSolarPanelModel};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::sensors::simple_mass_props::SimpleMassProps;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use nalgebra::{Matrix3, Vector3};

use super::environment::Environment;
use super::latch::Latch;
use super::rw_factory::{RwFactory, hr16_pyramid};
use super::thruster_factory::{ThrusterFactory, ThrusterModel};

/// Upstream `scObject.hub.mHub` \[kg\].
pub const HUB_MASS_KG: f64 = 750.0;
/// Upstream `maxRWMomentum` \[N m s\].
const MAX_RW_MOMENTUM_NMS: f64 = 50.0;
/// Upstream `powerMonitor.storageCapacity`: 120 W h \[J\].
pub const BATTERY_CAPACITY_J: f64 = 2.0 * 60.0 * 3600.0;

/// Upstream `I_sc` \[kg m²\].
pub fn hub_inertia_kg_m2() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0))
}

/// Upstream's reaction-wheel pyramid: four `Honeywell_HR16` (50 N m s) at
/// 40° elevation and 45°, 135°, 225°, 315° azimuth.
pub fn rw_factory() -> RwFactory {
    hr16_pyramid(MAX_RW_MOMENTUM_NMS)
}

/// The dynamics of one spacecraft (upstream `BSKDynamicModels`).
pub struct Dynamics {
    /// Upstream `spacecraftIndex`.
    pub index: usize,
    /// Upstream `processTasksTimeStep` \[ns\].
    pub task_period_nanos: u64,
    /// Upstream `I_sc`.
    pub inertia_kg_m2: Matrix3<f64>,
    /// Upstream `solarPanelAxis`.
    pub solar_panel_axis: Vector3<f64>,
    /// Upstream `scObject` ("sat-<i>"), with the wheels, the thruster and its
    /// tank.
    pub spacecraft: Spacecraft,
    pub simple_nav: SimpleNavigation,
    pub simple_mass_props: SimpleMassProps,
    /// The wheels as created (upstream `rwFactory`), for the FSW's
    /// configuration message.
    pub rw_factory: RwFactory,
    /// The thrusters as created (upstream `thrusterFactory`).
    pub thruster_factory: ThrusterFactory,
    /// Upstream `rwStateEffector.rwMotorCmdInMsg`: the wheels' array command.
    pub rw_motor_cmd: ArrayMotorTorqueToSingleActuators,
    /// Upstream `thrusterDynamicEffector.cmdsInMsg`: one per thruster.
    pub thruster_cmds: Vec<Latch<ThrusterCommandMsg>>,
    pub solar_panel: SimpleSolarPanel,
    pub power_sink: SimplePowerSink,
    /// Upstream `powerMonitor`.
    pub battery: SimpleBattery,
    /// Upstream `rwPowerList`.
    pub rw_power: Vec<ReactionWheelPower>,
    /// Upstream `rwStateEffector.rwSpeedOutMsg`.
    pub rw_speed_out_msg: Output<RwSpeedMsg>,
    /// Upstream `rwStateEffector.rwOutMsgs`.
    pub rw_out_msgs: Vec<Output<ReactionWheelStateMsg>>,
    /// Upstream `thrusterDynamicEffector.thrusterOutMsgs`.
    pub thruster_out_msgs: Vec<Output<ThrusterOutputMsg>>,
    /// Upstream `fuelTankStateEffector.fuelTankOutMsg`.
    pub fuel_tank_out_msg: Output<FuelTankMsg>,
}

impl Dynamics {
    /// Upstream `BSKDynamicModels(SimBase, dynRate, spacecraftIndex)`: builds
    /// the spacecraft and adds it to the environment's gravity, eclipse and
    /// ground station. The initial state is zero; set it with
    /// [`Self::set_initial_state`].
    pub fn new(environment: &mut Environment, task_period_nanos: u64, index: usize) -> Self {
        let name = format!("sat-{index}");
        let inertia = hub_inertia_kg_m2();
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: HUB_MASS_KG,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: inertia,
            integration_step_nanos: task_period_nanos,
            initial_position_m: Vector3::zeros(),
            initial_velocity_mps: Vector3::zeros(),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: None,
        });
        environment.add_bodies_to(&mut spacecraft);

        // Reaction wheels, commanded through the splitter.
        let rw_factory = rw_factory();
        let mut wheels = ReactionWheelStateEffector::new(format!("RWArray{index}"));
        for wheel in &rw_factory.wheels {
            wheels.add_reaction_wheel(wheel.clone());
        }
        let wheel_count = rw_factory.wheels.len();
        let rw_motor_cmd =
            ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
                name: format!("rwMotorCmd{index}"),
                num_actuators: wheel_count,
            });
        for (command, wheel) in rw_motor_cmd
            .actuator_out_msgs
            .iter()
            .zip(wheels.wheels_mut())
        {
            command.connect_to(&mut wheel.motor_torque_in_msg);
        }
        let rw_speed_out_msg = wheels.rw_speed_out_msg.clone();
        let rw_out_msgs: Vec<_> = wheels
            .wheels()
            .iter()
            .map(|wheel| wheel.state_out.clone())
            .collect();
        spacecraft.add_state_effector(wheels);

        // The thruster and its tank (upstream `FuelTankModelUniformBurn`).
        let mut thruster_factory = ThrusterFactory::default();
        thruster_factory.create(
            ThrusterModel::MoogMonarc90Ht,
            Vector3::zeros(),
            Vector3::z(),
            None,
        );
        let mut tank_model = FuelTankModel::new(
            FuelTankShape::UniformBurn {
                radius_tank_init_m: 1.0,
                length_tank_m: 1.0,
            },
            50.0,
        );
        tank_model.max_fuel_mass_kg = 75.0;
        let mut fuel_tank = FuelTank::new(FuelTankConfig::new("fuelTank", tank_model));
        let mut thruster_cmds = Vec::new();
        let mut thruster_out_msgs = Vec::new();
        let mut thrusters = Vec::new();
        for spec in &thruster_factory.thrusters {
            let mut thruster = ThrusterDynamicEffector::new(spec.dynamic_effector_config());
            let latch = Latch::<ThrusterCommandMsg>::default();
            latch.output.connect_to(&mut thruster.command_in);
            fuel_tank.add_thruster_set(&mut thruster);
            thruster_out_msgs.push(thruster.thruster_out_msg.clone());
            thruster_cmds.push(latch);
            thrusters.push(thruster);
        }
        for thruster in thrusters {
            spacecraft.add_dynamic_effector(thruster);
        }
        let fuel_tank_out_msg = fuel_tank.fuel_tank_out_msg.clone();
        spacecraft.add_state_effector(fuel_tank);

        let mut simple_nav = SimpleNavigation::new(format!("SimpleNavigation{index}"));
        spacecraft
            .state_out
            .connect_to(&mut simple_nav.spacecraft_state_in_msg);
        let mut simple_mass_props = SimpleMassProps::new(format!("SimpleMassProperties{index}"));
        spacecraft
            .mass_props_out
            .connect_to(&mut simple_mass_props.sc_mass_props_in_msg);

        let eclipse_index = environment.add_spacecraft(&spacecraft.state_out);
        debug_assert_eq!(eclipse_index, index, "spacecraft added in index order");

        // Power: the wheels, the panel, the sink and the battery.
        let rw_power: Vec<ReactionWheelPower> = rw_out_msgs
            .iter()
            .enumerate()
            .map(|(item, wheel_state)| {
                let mut model = ReactionWheelPowerModel::default();
                model.base_power_need_w = 5.0;
                model.mech_to_elec_efficiency = 0.5;
                wheel_state.connect_to(&mut model.rw_state_in_msg);
                ReactionWheelPower::new(format!("{name}RWPower{item}"), model)
            })
            .collect();
        let solar_panel_axis = Vector3::z();
        let mut solar_panel = SimpleSolarPanel::new(
            "solarPanel",
            SimpleSolarPanelModel::new(solar_panel_axis, 0.4 * 0.4 * 2.0 + 0.2 * 0.4 * 2.0, 0.35),
        );
        spacecraft
            .state_out
            .connect_to(&mut solar_panel.model.state_in_msg);
        // Upstream's `eclipseOutMsgs[0]`: the first spacecraft's.
        environment
            .eclipse_out_msg(0)
            .connect_to(&mut solar_panel.model.sun_eclipse_in_msg);
        environment
            .sun
            .sun_out_msg
            .connect_to(&mut solar_panel.model.sun_in_msg);
        let power_sink = SimplePowerSink::new(
            "powerSink",
            SimplePowerSinkModel {
                node_power_out_w: -2.0,
            },
        );
        let mut battery = SimpleBattery::new(SimpleBatteryConfig {
            name: "powerMonitor".to_string(),
            storage_capacity_j: BATTERY_CAPACITY_J,
            stored_charge_init_j: BATTERY_CAPACITY_J * 0.6,
        });
        solar_panel
            .node_power_out_msg
            .connect_to(battery.add_power_node_to_model());
        power_sink
            .node_power_out_msg
            .connect_to(battery.add_power_node_to_model());
        for node in &rw_power {
            node.node_power_out_msg
                .connect_to(battery.add_power_node_to_model());
        }

        Self {
            index,
            task_period_nanos,
            inertia_kg_m2: inertia,
            solar_panel_axis,
            spacecraft,
            simple_nav,
            simple_mass_props,
            rw_factory,
            thruster_factory,
            rw_motor_cmd,
            thruster_cmds,
            solar_panel,
            power_sink,
            battery,
            rw_power,
            rw_speed_out_msg,
            rw_out_msgs,
            thruster_out_msgs,
            fuel_tank_out_msg,
        }
    }

    /// Upstream `hub.r_CN_NInit`, `v_CN_NInit`, `sigma_BNInit` and
    /// `omega_BN_BInit` (the center of mass is the body origin).
    pub fn set_initial_state(
        &mut self,
        position_m: Vector3<f64>,
        velocity_mps: Vector3<f64>,
        sigma_bn: Vector3<f64>,
        omega_bn_b_radps: Vector3<f64>,
    ) {
        let config = &mut self.spacecraft.config;
        config.initial_position_m = position_m;
        config.initial_velocity_mps = velocity_mps;
        config.initial_sigma_bn = sigma_bn;
        config.initial_omega_radps = omega_bn_b_radps;
    }

    /// Upstream `scObject.scStateOutMsg`.
    pub fn state_out_msg(&self) -> &Output<SpacecraftStateMsg> {
        &self.spacecraft.state_out
    }

    /// Upstream `rwFactory.getConfigMessage()`.
    pub fn rw_config_message(&self) -> RwArrayConfigMsg {
        self.rw_factory.config_message()
    }

    /// Upstream `fswSetupThrusters.writeConfigMessage()` of the thrusters.
    pub fn thruster_config_message(&self) -> ThrusterArrayConfigMsg {
        self.thruster_factory.config_message()
    }

    /// Upstream `rwStateEffector.rwMotorCmdInMsg`.
    pub fn rw_motor_cmd_in_msg(&mut self) -> &mut Input<ArrayMotorTorqueMsg> {
        &mut self.rw_motor_cmd.torque_in_msg
    }

    /// Upstream `thrusterDynamicEffector.cmdsInMsg`, per thruster.
    pub fn thruster_cmd_in_msgs(&mut self) -> impl Iterator<Item = &mut Input<ThrusterCommandMsg>> {
        self.thruster_cmds.iter_mut().map(|latch| &mut latch.input)
    }

    /// Adds the modules to the spacecraft's dynamics process in upstream's
    /// task order (module priority 100 in insertion order, then the mass
    /// properties at 99), after the command relays.
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let period = self.task_period_nanos;
        simulation.add_module("rwMotorCmd", &mut self.rw_motor_cmd, period, 110);
        for latch in &mut self.thruster_cmds {
            simulation.add_module("thrusterCmd", latch, period, 110);
        }
        simulation.add_module(
            format!("sat-{}", self.index),
            &mut self.spacecraft,
            period,
            100,
        );
        simulation.add_module("SimpleNavigation", &mut self.simple_nav, period, 100);
        simulation.add_module("solarPanel", &mut self.solar_panel, period, 100);
        simulation.add_module("powerSink", &mut self.power_sink, period, 100);
        simulation.add_module("powerMonitor", &mut self.battery, period, 100);
        for node in &mut self.rw_power {
            simulation.add_module("RWPower", node, period, 100);
        }
        simulation.add_module(
            "SimpleMassProperties",
            &mut self.simple_mass_props,
            period,
            99,
        );
    }
}
