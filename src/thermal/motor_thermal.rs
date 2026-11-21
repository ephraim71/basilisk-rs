//! Reaction wheel motor temperature: port of upstream Basilisk
//! `motorThermal`.
//!
//! A lumped heat balance for one wheel motor, read from that wheel's
//! [`ReactionWheelStateMsg`] (one of
//! [`ReactionWheelStateEffector`](crate::dynamics::reaction_wheel_state_effector::ReactionWheelStateEffector)'s
//! per-wheel `state_out` ports). Over each step `dt` the motor gains the
//! power its inefficiency wastes and the bearing friction heat, and loses
//! heat to its surroundings through a thermal resistance:
//!
//! ```text
//! Q_gen = dt · (|Ω u| / η · (1 - η) + |Ω u_f|)
//! Q_out = dt · (T - T_ambient) / R
//! T    += (Q_gen - Q_out) / C
//! ```
//!
//! with `Ω` the wheel speed, `u` the motor torque, `u_f` the friction torque,
//! `η` the motor efficiency, `R` the ambient thermal resistance and `C` the
//! motor heat capacity. The result is published as a [`TemperatureMsg`].
//!
//! [`MotorThermal::current_temperature_c`] is the model state (upstream
//! `currentTemperature`): initialization starts it at
//! [`MotorThermalConfig::initial_temperature_c`], and, as upstream, a later
//! reset keeps it and only restarts the time step.
//!
//! # Differences from upstream
//!
//! * One wheel's [`ReactionWheelStateMsg`] stands in for upstream's
//!   `RWConfigLogMsgPayload`; `Omega`, `u_current` and `frictionTorque` are
//!   its `omega_radps`, `applied_motor_torque_nm` and `friction_torque_nm`.
//! * The wheel input is required: unconnected, upstream reports a `bskError`
//!   at reset; here it is logged at initialization and reset (see
//!   [`MotorThermal::unconnected_inputs`]) and reads as a wheel at rest.
//! * Upstream's single `currentTemperature` field is both the initial value
//!   and the state; here the initial value is in the configuration.
//!
//! # Panics
//!
//! At initialization or reset, if the configuration fails
//! [`MotorThermalConfig::validate`] or the current temperature is at or below
//! absolute zero (upstream `bskError`s).

use crate::messages::{Input, Output, ReactionWheelStateMsg, TemperatureMsg};
use crate::time::diff_nanos_to_seconds;
use crate::{Module, SimulationContext};

/// Absolute zero \[°C\].
const ABSOLUTE_ZERO_C: f64 = -273.15;

/// Thermal parameters of a [`MotorThermal`].
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MotorThermalConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `currentTemperature` at initialization \[°C\]: must be above
    /// absolute zero, upstream's default, which initialization rejects.
    pub initial_temperature_c: f64,
    /// Upstream `ambientTemperature` \[°C\]: temperature the motor relaxes
    /// to; zero by default.
    pub ambient_temperature_c: f64,
    /// Upstream `ambientThermalResistance` \[°C/W\] (its header says W/°C;
    /// the model divides a temperature difference by it): strictly positive.
    pub ambient_thermal_resistance_c_per_w: f64,
    /// Upstream `motorHeatCapacity` \[J/°C\]: strictly positive.
    pub motor_heat_capacity_j_per_c: f64,
    /// Upstream `efficiency` [-]: fraction of the motor power that becomes
    /// mechanical power, strictly between zero and one (upstream's default,
    /// one, is rejected).
    pub efficiency: f64,
}

impl Default for MotorThermalConfig {
    /// Upstream's constructor, which initialization rejects until the
    /// temperature, resistance, heat capacity and efficiency are set.
    fn default() -> Self {
        Self {
            name: String::new(),
            initial_temperature_c: ABSOLUTE_ZERO_C,
            ambient_temperature_c: 0.0,
            ambient_thermal_resistance_c_per_w: -1.0,
            motor_heat_capacity_j_per_c: -1.0,
            efficiency: 1.0,
        }
    }
}

impl MotorThermalConfig {
    /// Upstream `Reset`'s parameter checks. Returns a description of the
    /// first violation, or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        if self.initial_temperature_c <= ABSOLUTE_ZERO_C {
            return Err(format!(
                "initial_temperature_c is {} °C, at or below absolute zero",
                self.initial_temperature_c
            ));
        }
        if self.efficiency <= 0.0 || self.efficiency >= 1.0 {
            return Err(format!(
                "efficiency is {}, must be greater than 0 and smaller than 1",
                self.efficiency
            ));
        }
        if self.ambient_thermal_resistance_c_per_w <= 0.0 {
            return Err(format!(
                "ambient thermal resistance is {} °C/W, must be positive",
                self.ambient_thermal_resistance_c_per_w
            ));
        }
        if self.motor_heat_capacity_j_per_c <= 0.0 {
            return Err(format!(
                "motor heat capacity is {} J/°C, must be positive",
                self.motor_heat_capacity_j_per_c
            ));
        }
        Ok(())
    }
}

/// Lumped motor temperature: upstream `MotorThermal`.
#[derive(Clone, Debug)]
pub struct MotorThermal {
    /// Thermal parameters, checked at initialization and reset.
    pub config: MotorThermalConfig,
    /// Upstream `rwStateInMsg`: the wheel's speed, motor torque and friction
    /// torque (required).
    pub rw_state_in_msg: Input<ReactionWheelStateMsg>,
    /// Upstream `temperatureOutMsg`: the motor temperature.
    pub temperature_out_msg: Output<TemperatureMsg>,
    /// Upstream `currentTemperature` \[°C\]: the motor temperature, the model
    /// state.
    pub current_temperature_c: f64,
    /// Upstream `prevTime` \[ns\]: time of the last update or reset.
    prev_time_nanos: u64,
}

impl MotorThermal {
    /// A motor at `config.initial_temperature_c`; the configuration is
    /// checked at initialization.
    pub fn new(config: MotorThermalConfig) -> Self {
        Self {
            current_temperature_c: config.initial_temperature_c,
            config,
            rw_state_in_msg: Input::default(),
            temperature_out_msg: Output::default(),
            prev_time_nanos: 0,
        }
    }

    /// The required inputs that are not connected (upstream `Reset`'s "not
    /// linked" error).
    pub fn unconnected_inputs(&self) -> Vec<&'static str> {
        if self.rw_state_in_msg.is_connected() {
            Vec::new()
        } else {
            vec!["rw_state_in_msg"]
        }
    }

    /// Upstream `Reset`'s checks, then the time step restarts at
    /// `current_sim_nanos`.
    fn restart(&mut self, current_sim_nanos: u64) {
        for port in self.unconnected_inputs() {
            log::error!(
                "motor thermal '{}': {port} is not connected",
                self.config.name
            );
        }
        if let Err(msg) = self.config.validate() {
            panic!("invalid MotorThermalConfig '{}': {msg}", self.config.name);
        }
        assert!(
            self.current_temperature_c > ABSOLUTE_ZERO_C,
            "motor thermal '{}': current temperature {} °C is at or below absolute zero",
            self.config.name,
            self.current_temperature_c
        );
        self.prev_time_nanos = current_sim_nanos;
    }

    /// Upstream `computeTemperature`.
    fn compute_temperature(&mut self, rw_state: &ReactionWheelStateMsg, current_sim_nanos: u64) {
        let config = &self.config;
        let time_step_s = diff_nanos_to_seconds(current_sim_nanos, self.prev_time_nanos);

        let wheel_power_w = rw_state.omega_radps * rw_state.applied_motor_torque_nm;
        let friction_heat_w = rw_state.omega_radps * rw_state.friction_torque_nm;
        let heat_generation_j = time_step_s
            * (wheel_power_w.abs() / config.efficiency * (1.0 - config.efficiency)
                + friction_heat_w.abs());
        let heat_dissipation_j = time_step_s
            * (self.current_temperature_c - config.ambient_temperature_c)
            / config.ambient_thermal_resistance_c_per_w;

        self.current_temperature_c +=
            (heat_generation_j - heat_dissipation_j) / config.motor_heat_capacity_j_per_c;
    }
}

impl Module for MotorThermal {
    fn init(&mut self) {
        self.current_temperature_c = self.config.initial_temperature_c;
        self.restart(0);
        self.temperature_out_msg.write(TemperatureMsg::default());
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.restart(context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        let rw_state = self.rw_state_in_msg.read();
        self.compute_temperature(&rw_state, context.current_sim_nanos);
        self.temperature_out_msg.write(TemperatureMsg {
            temperature_c: self.current_temperature_c,
        });
        self.prev_time_nanos = context.current_sim_nanos;
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `motorThermal/_UnitTest/test_motorThermal.py`: the five
    //! analytic scenarios with their 1e-8 accuracy, the integration check
    //! against a live `ReactionWheelStateEffector`, and the reset errors
    //! (the unlinked input is logged here, so that case checks
    //! [`MotorThermal::unconnected_inputs`]).

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use crate::Module;
    use crate::dynamics::reaction_wheel_state_effector::{
        ReactionWheelStateEffector, ReactionWheelStateEffectorConfig,
    };
    use crate::messages::{MotorTorqueMsg, Output, ReactionWheelStateMsg, TemperatureMsg};
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};
    use crate::test_utils::MessageRecorder;

    use super::{MotorThermal, MotorThermalConfig};

    /// Upstream `DT` \[s\]: task update period.
    const DT: f64 = 0.1;
    const DT_NANOS: u64 = 100_000_000;
    /// Upstream `N_STEPS`: update steps simulated per scenario.
    const N_STEPS: u64 = 10;
    /// Upstream `THERMAL_RESISTANCE` \[°C/W\].
    const THERMAL_RESISTANCE: f64 = 10.0;
    /// Upstream `HEAT_CAPACITY` \[J/°C\].
    const HEAT_CAPACITY: f64 = 10.0;
    const ACCURACY: f64 = 1.0e-8;

    #[derive(Clone, Copy, Debug, PartialEq)]
    enum Mechanism {
        Dissipation,
        Inefficiency,
        Friction,
        Combined,
    }

    #[derive(Clone, Copy, Debug, PartialEq)]
    enum Direction {
        Cooling,
        Warming,
        Heating,
    }

    /// One entry of upstream `SCENARIOS`.
    #[derive(Clone, Copy, Debug)]
    struct Scenario {
        name: &'static str,
        omega: f64,
        u_current: f64,
        friction_torque: f64,
        efficiency: f64,
        current_temperature: f64,
        ambient_temperature: f64,
        mechanism: Mechanism,
        direction: Direction,
    }

    const SCENARIOS: [Scenario; 5] = [
        Scenario {
            name: "dissipation_cooling",
            omega: 0.0,
            u_current: 0.0,
            friction_torque: 0.0,
            efficiency: 0.5,
            current_temperature: 20.0,
            ambient_temperature: 0.0,
            mechanism: Mechanism::Dissipation,
            direction: Direction::Cooling,
        },
        Scenario {
            name: "dissipation_warming",
            omega: 0.0,
            u_current: 0.0,
            friction_torque: 0.0,
            efficiency: 0.5,
            current_temperature: 0.0,
            ambient_temperature: 20.0,
            mechanism: Mechanism::Dissipation,
            direction: Direction::Warming,
        },
        Scenario {
            name: "inefficiency_only",
            omega: 100.0,
            u_current: 0.2,
            friction_torque: 0.0,
            efficiency: 0.5,
            current_temperature: 20.0,
            ambient_temperature: 20.0,
            mechanism: Mechanism::Inefficiency,
            direction: Direction::Heating,
        },
        Scenario {
            name: "friction_only",
            omega: 100.0,
            u_current: 0.0,
            friction_torque: 0.01,
            efficiency: 0.5,
            current_temperature: 20.0,
            ambient_temperature: 20.0,
            mechanism: Mechanism::Friction,
            direction: Direction::Heating,
        },
        Scenario {
            name: "combined",
            omega: 100.0,
            u_current: 0.2,
            friction_torque: 0.01,
            efficiency: 0.5,
            current_temperature: 20.0,
            ambient_temperature: 0.0,
            mechanism: Mechanism::Combined,
            direction: Direction::Heating,
        },
    ];

    fn epoch() -> Epoch {
        Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
    }

    /// Upstream `referenceTemperatures`: the module heat balance step by step
    /// at the logged times, starting from `prevTime = 0`.
    fn reference_temperatures(scenario: &Scenario, times_nanos: &[u64]) -> Vec<f64> {
        let mut temperature = scenario.current_temperature;
        let mut previous_nanos = 0;
        let mut expected = Vec::new();
        for &current_nanos in times_nanos {
            let dt = (current_nanos - previous_nanos) as f64 * 1.0e-9;
            let wheel_power = scenario.omega * scenario.u_current;
            let friction_heat = scenario.omega * scenario.friction_torque;
            let heat_generation = dt
                * (wheel_power.abs() / scenario.efficiency * (1.0 - scenario.efficiency)
                    + friction_heat.abs());
            let heat_dissipation =
                dt * (temperature - scenario.ambient_temperature) / THERMAL_RESISTANCE;
            temperature += (heat_generation - heat_dissipation) / HEAT_CAPACITY;
            expected.push(temperature);
            previous_nanos = current_nanos;
        }
        expected
    }

    /// Records the sim time of every update, as upstream's `tempLog.times()`.
    #[derive(Default)]
    struct TimeRecorder {
        times_nanos: Vec<u64>,
    }

    impl Module for TimeRecorder {
        fn init(&mut self) {}

        fn update(&mut self, context: &crate::SimulationContext) {
            self.times_nanos.push(context.current_sim_nanos);
        }
    }

    /// Upstream `runScenario`: the module fed a constant wheel state for
    /// `N_STEPS` steps of `DT`. Returns the logged times and temperatures.
    fn run_scenario(scenario: &Scenario) -> (Vec<u64>, Vec<f64>) {
        let mut thermal_model = MotorThermal::new(MotorThermalConfig {
            name: "rwThermals".to_string(),
            initial_temperature_c: scenario.current_temperature,
            ambient_temperature_c: scenario.ambient_temperature,
            efficiency: scenario.efficiency,
            ambient_thermal_resistance_c_per_w: THERMAL_RESISTANCE,
            motor_heat_capacity_j_per_c: HEAT_CAPACITY,
        });
        let rw_state = Output::new(ReactionWheelStateMsg {
            omega_radps: scenario.omega,
            applied_motor_torque_nm: scenario.u_current,
            friction_torque_nm: scenario.friction_torque,
            ..Default::default()
        });
        let mut temp_log = MessageRecorder::<TemperatureMsg>::default();
        let mut times = TimeRecorder::default();
        {
            let mut sim = Simulation::new(epoch(), false);
            sim.connect(&rw_state, &mut thermal_model.rw_state_in_msg);
            sim.connect(&thermal_model.temperature_out_msg, &mut temp_log.input_msg);
            sim.add_module("rwThermals", &mut thermal_model, DT_NANOS, 2);
            sim.add_module("tempLog", &mut temp_log, DT_NANOS, 1);
            sim.add_module("times", &mut times, DT_NANOS, 0);
            sim.run_for(DT_NANOS * N_STEPS);
        }
        let temperatures = temp_log
            .samples
            .iter()
            .map(|sample| sample.temperature_c)
            .collect();
        (times.times_nanos, temperatures)
    }

    /// Upstream `test_motorThermal`: each scenario's history against the
    /// analytic recurrence, and its first step against the closed-form
    /// contribution of the mechanism it isolates.
    #[test]
    fn temperature_follows_the_heat_balance() {
        for scenario in &SCENARIOS {
            let (times, temperatures) = run_scenario(scenario);
            let expected = reference_temperatures(scenario, &times);
            assert_eq!(temperatures.len(), expected.len());
            for (index, (actual, truth)) in temperatures.iter().zip(&expected).enumerate() {
                assert!(
                    (actual - truth).abs() <= ACCURACY,
                    "{}: temperature[{index}] {actual} != {truth}",
                    scenario.name
                );
            }
            assert!(temperatures.len() > 1);
            assert!((temperatures[0] - scenario.current_temperature).abs() <= ACCURACY);

            let first_step = temperatures[1] - scenario.current_temperature;
            let name = scenario.name;
            match scenario.mechanism {
                Mechanism::Dissipation => {
                    let expected_first_step = -DT
                        * (scenario.current_temperature - scenario.ambient_temperature)
                        / THERMAL_RESISTANCE
                        / HEAT_CAPACITY;
                    assert!(
                        (first_step - expected_first_step).abs() <= ACCURACY,
                        "{name}"
                    );
                    let differences: Vec<f64> =
                        temperatures[1..].windows(2).map(|w| w[1] - w[0]).collect();
                    let after_first = &temperatures[1..];
                    if scenario.direction == Direction::Cooling {
                        assert!(differences.iter().all(|d| *d < 0.0), "{name}");
                        assert!(
                            after_first
                                .iter()
                                .all(|t| *t >= scenario.ambient_temperature),
                            "{name}"
                        );
                    } else {
                        assert!(differences.iter().all(|d| *d > 0.0), "{name}");
                        assert!(
                            after_first
                                .iter()
                                .all(|t| *t <= scenario.ambient_temperature),
                            "{name}"
                        );
                    }
                }
                Mechanism::Inefficiency => {
                    let wheel_power = scenario.omega * scenario.u_current;
                    let expected_first_step = DT * wheel_power.abs() / scenario.efficiency
                        * (1.0 - scenario.efficiency)
                        / HEAT_CAPACITY;
                    assert!(
                        (first_step - expected_first_step).abs() <= ACCURACY,
                        "{name}"
                    );
                    assert!(first_step > 0.0, "{name}");
                }
                Mechanism::Friction => {
                    let friction_heat = scenario.omega * scenario.friction_torque;
                    let expected_first_step = DT * friction_heat.abs() / HEAT_CAPACITY;
                    assert!(
                        (first_step - expected_first_step).abs() <= ACCURACY,
                        "{name}"
                    );
                    assert!(first_step > 0.0, "{name}");
                }
                Mechanism::Combined => {}
            }
        }
    }

    /// Upstream `test_motorThermal_integration`: wired to a live wheel spun
    /// up under a constant 0.2 N·m with friction, from 300 RPM, the motor
    /// starts at ambient and heats up, with finite temperatures throughout.
    ///
    /// Upstream's wheel is `simIncludeRW`'s Honeywell HR16 with its friction
    /// model, which is not ported; this wheel has the HR16's 50 N·m·s,
    /// 6000 RPM and 0.2 N·m limits and a Coulomb plus viscous friction.
    #[test]
    fn a_spinning_loaded_wheel_heats_its_motor() {
        let rpm = 2.0 * std::f64::consts::PI / 60.0;
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 750.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
            integration_step_nanos: DT_NANOS,
            initial_position_m: Vector3::new(7_000_000.0, 0.0, 0.0),
            initial_velocity_mps: Vector3::new(0.0, 7_500.0, 0.0),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: None,
        });
        let mut wheel = ReactionWheelStateEffectorConfig::balanced(
            "rw",
            Vector3::zeros(),
            Vector3::new(1.0, 0.0, 0.0),
            0.2,
            50.0,
        );
        wheel.max_speed_radps = 6000.0 * rpm;
        wheel.js_kg_m2 = 50.0 / wheel.max_speed_radps;
        wheel.initial_omega_radps = 300.0 * rpm;
        wheel.coulomb_friction_nm = 0.0005;
        wheel.viscous_friction_nms_per_rad = 1.0e-5;
        let mut reaction_wheels = ReactionWheelStateEffector::new("ReactionWheel");
        reaction_wheels.add_reaction_wheel(wheel);
        let command = Output::new(MotorTorqueMsg {
            motor_torque_nm: 0.2,
        });

        let mut thermal_model = MotorThermal::new(MotorThermalConfig {
            name: "rwThermals".to_string(),
            initial_temperature_c: 20.0,
            ambient_temperature_c: 20.0,
            efficiency: 0.5,
            ambient_thermal_resistance_c_per_w: THERMAL_RESISTANCE,
            motor_heat_capacity_j_per_c: HEAT_CAPACITY,
        });
        let mut temp_log = MessageRecorder::<TemperatureMsg>::default();
        {
            let mut sim = Simulation::new(epoch(), false);
            sim.connect(
                &command,
                &mut reaction_wheels.wheels_mut()[0].motor_torque_in_msg,
            );
            sim.connect(
                &reaction_wheels.wheels()[0].state_out,
                &mut thermal_model.rw_state_in_msg,
            );
            sim.connect(&thermal_model.temperature_out_msg, &mut temp_log.input_msg);
            spacecraft.add_state_effector(reaction_wheels);
            sim.add_module("spacecraftBody", &mut spacecraft, DT_NANOS, 2);
            sim.add_module("rwThermals", &mut thermal_model, DT_NANOS, 1);
            sim.add_module("tempLog", &mut temp_log, DT_NANOS, 0);
            sim.run_for(DT_NANOS * N_STEPS);
        }

        let temperatures: Vec<f64> = temp_log
            .samples
            .iter()
            .map(|sample| sample.temperature_c)
            .collect();
        assert!(
            temperatures.iter().all(|t| t.is_finite()),
            "{temperatures:?}"
        );
        assert!((temperatures[0] - 20.0).abs() <= ACCURACY);
        assert!(
            temperatures.last().expect("logged") > &temperatures[0],
            "a spinning, loaded wheel should heat the motor: {temperatures:?}"
        );
    }

    fn base_configured_model(link_input: bool) -> (MotorThermal, Output<ReactionWheelStateMsg>) {
        let mut thermal_model = MotorThermal::new(MotorThermalConfig {
            name: "rwThermals".to_string(),
            initial_temperature_c: 20.0,
            efficiency: 0.5,
            ambient_thermal_resistance_c_per_w: THERMAL_RESISTANCE,
            motor_heat_capacity_j_per_c: HEAT_CAPACITY,
            ..Default::default()
        });
        let rw_state = Output::new(ReactionWheelStateMsg::default());
        if link_input {
            rw_state.connect_to(&mut thermal_model.rw_state_in_msg);
        }
        (thermal_model, rw_state)
    }

    /// Upstream `test_motorThermal_resetErrors`, "rwStateInMsg not linked".
    #[test]
    fn an_unlinked_wheel_is_reported() {
        let (mut thermal_model, _rw_state) = base_configured_model(false);
        assert_eq!(thermal_model.unconnected_inputs(), ["rw_state_in_msg"]);
        // Logged, not fatal: the motor reads a wheel at rest and relaxes.
        thermal_model.init();
        let (linked, _rw_state) = base_configured_model(true);
        assert!(linked.unconnected_inputs().is_empty());
    }

    /// Breaks one precondition of a valid configuration.
    type BreakConfig = fn(&mut MotorThermalConfig);

    /// Upstream `test_motorThermal_resetErrors`: each broken precondition is
    /// rejected, naming the offending quantity.
    #[test]
    fn broken_preconditions_are_rejected() {
        let cases: [(&str, BreakConfig); 5] = [
            ("absolute zero", |config| {
                config.initial_temperature_c = -300.0
            }),
            ("efficiency", |config| config.efficiency = 1.0),
            ("efficiency", |config| config.efficiency = 0.0),
            ("thermal resistance", |config| {
                config.ambient_thermal_resistance_c_per_w = 0.0
            }),
            ("motor heat capacity", |config| {
                config.motor_heat_capacity_j_per_c = 0.0
            }),
        ];
        for (expected_message, break_config) in cases {
            let (mut thermal_model, _rw_state) = base_configured_model(true);
            assert_eq!(thermal_model.config.validate(), Ok(()));
            break_config(&mut thermal_model.config);
            let message = thermal_model
                .config
                .validate()
                .expect_err("a broken precondition");
            assert!(
                message.contains(expected_message),
                "{message:?} does not name {expected_message:?}"
            );
        }
    }

    /// A broken precondition fails initialization, as upstream's `Reset`
    /// raises.
    #[test]
    #[should_panic(expected = "thermal resistance")]
    fn initialization_rejects_a_broken_precondition() {
        let (mut thermal_model, _rw_state) = base_configured_model(true);
        thermal_model.config.ambient_thermal_resistance_c_per_w = 0.0;
        thermal_model.init();
    }

    /// Upstream's constructor defaults are rejected until configured.
    #[test]
    fn upstream_defaults_are_rejected() {
        assert!(MotorThermalConfig::default().validate().is_err());
    }

    /// A reset keeps the temperature, as upstream, and rejects one driven to
    /// absolute zero.
    #[test]
    fn a_reset_keeps_the_temperature_and_restarts_the_step() {
        let (mut thermal_model, rw_state) = base_configured_model(true);
        rw_state.write(ReactionWheelStateMsg {
            omega_radps: 100.0,
            applied_motor_torque_nm: 0.2,
            ..Default::default()
        });
        thermal_model.init();
        let context = |nanos| crate::SimulationContext {
            current_sim_nanos: nanos,
            current_epoch: epoch(),
        };
        thermal_model.update(&context(0));
        thermal_model.update(&context(DT_NANOS));
        let heated = thermal_model.current_temperature_c;
        assert!(heated > 20.0);

        Module::reset(&mut thermal_model, &context(5 * DT_NANOS));
        assert_eq!(thermal_model.current_temperature_c, heated);
        // The first step after the reset spans one DT, not six.
        thermal_model.update(&context(6 * DT_NANOS));
        let step = thermal_model.current_temperature_c - heated;
        let expected =
            DT * (20.0 / 0.5 * 0.5 - (heated - 0.0) / THERMAL_RESISTANCE) / HEAT_CAPACITY;
        assert!((step - expected).abs() < 1.0e-12, "{step} != {expected}");
    }
}
