//! Sensor temperature: port of upstream Basilisk `sensorThermal`.
//!
//! A flat, body-fixed sensor plate with an insulated back. It absorbs the
//! sunlight falling on its face and the electrical power it draws while on,
//! radiates from its face, and integrates the balance with an Euler step:
//!
//! ```text
//! Q_in  = illumination · S · A · max(ŝ_B · n̂_B, 0) · α + P_draw · on
//! Q_out = A · ε · σ · (T + 273.15)⁴
//! T    += (Q_in - Q_out) / (c · m) · dt
//! ```
//!
//! with `ŝ_B` the Sun direction in body components, `n̂_B` the sensor
//! normal, `A` its area, `α` its absorptivity, `ε` its emissivity, `m` its
//! mass and `c` its specific heat. The illumination factor is the latest
//! [`EclipseMsg`] when `sun_eclipse_in_msg` is connected, and one otherwise;
//! `on` is [`SensorThermal::sensor_power_status`], which
//! `sensor_status_in_msg` overwrites while it is connected. The temperature
//! is published as a [`TemperatureMsg`].
//!
//! Upstream's constants are kept as they are: the solar flux `S` is 1366
//! W/m² at any distance from the Sun, and `σ` is 5.76051e-8 W/(m²·K⁴)
//! (the Stefan–Boltzmann constant is 5.670374e-8).
//!
//! A reset restarts the temperature from
//! [`SensorThermalConfig::initial_temperature_c`] (upstream `T_0`).
//!
//! # Differences from upstream
//!
//! * The Sun is a [`SunEphemerisMsg`], this crate's stand-in for upstream's
//!   `SpicePlanetStateMsg` of the Sun.
//! * The Sun and spacecraft inputs are required: unconnected, upstream reports
//!   a `bskError` at reset; here they read as defaults and are logged at
//!   initialization and reset (see [`SensorThermal::unconnected_inputs`]).
//! * A reset also restarts the Euler step at the reset time. Upstream keeps
//!   the time of the last update, which is the same time whenever upstream
//!   resets (its clock never runs backwards, while
//!   [`Simulation::reset`](crate::simulation::Simulation::reset) rewinds to
//!   zero).
//!
//! # Panics
//!
//! At initialization or reset, if the configuration fails
//! [`SensorThermalConfig::validate`] (upstream `bskError`s).

use nalgebra::Vector3;

use crate::messages::{
    DeviceState, DeviceStatusMsg, EclipseMsg, Input, Output, SpacecraftStateMsg, SunEphemerisMsg,
    TemperatureMsg,
};
use crate::time::NANO2SEC;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Upstream `S` \[W/m²\]: the solar flux, its 1 AU value.
const SOLAR_FLUX_W_PER_M2: f64 = 1366.0;
/// Upstream `boltzmannConst` \[W/(m²·K⁴)\]: upstream's value of the
/// Stefan–Boltzmann constant.
const BOLTZMANN_W_PER_M2_K4: f64 = 5.76051e-8;
/// Offset from degrees Celsius to kelvin.
const CELSIUS_TO_KELVIN: f64 = 273.15;

/// Geometry and thermal properties of a [`SensorThermal`].
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SensorThermalConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `nHat_B`: sensor normal, body components. Normalized at
    /// reset, as upstream.
    pub n_hat_b: Vector3<f64>,
    /// Upstream `sensorPowerDraw` \[W\]: power the sensor dissipates while
    /// on; zero by default.
    pub sensor_power_draw_w: f64,
    /// Upstream `sensorArea` \[m²\]: strictly positive.
    pub sensor_area_m2: f64,
    /// Upstream `sensorAbsorptivity` [-]: in `(0, 1]`.
    pub sensor_absorptivity: f64,
    /// Upstream `sensorEmissivity` [-]: in `(0, 1]`.
    pub sensor_emissivity: f64,
    /// Upstream `sensorMass` \[kg\]: strictly positive; 1 kg by default.
    pub sensor_mass_kg: f64,
    /// Upstream `sensorSpecificHeat` \[J/(kg·K)\]: strictly positive;
    /// aluminium's 890 by default.
    pub sensor_specific_heat_j_per_kg_k: f64,
    /// Upstream `T_0` \[°C\]: temperature at initialization and reset; 30 °C
    /// by default.
    pub initial_temperature_c: f64,
}

impl Default for SensorThermalConfig {
    /// Upstream's constructor. The normal, area, absorptivity and emissivity,
    /// which upstream leaves unset, are zero, which initialization rejects.
    fn default() -> Self {
        Self {
            name: String::new(),
            n_hat_b: Vector3::zeros(),
            sensor_power_draw_w: 0.0,
            sensor_area_m2: 0.0,
            sensor_absorptivity: 0.0,
            sensor_emissivity: 0.0,
            sensor_mass_kg: 1.0,
            sensor_specific_heat_j_per_kg_k: 890.0,
            initial_temperature_c: 30.0,
        }
    }
}

impl SensorThermalConfig {
    /// Upstream `Reset`'s parameter checks. Returns a description of the
    /// first violation, or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        for (label, value) in [
            ("sensor_area_m2", self.sensor_area_m2),
            ("sensor_mass_kg", self.sensor_mass_kg),
            (
                "sensor_specific_heat_j_per_kg_k",
                self.sensor_specific_heat_j_per_kg_k,
            ),
        ] {
            if value <= 0.0 {
                return Err(format!("{label} must be positive, got {value}"));
            }
        }
        for (label, value) in [
            ("sensor_absorptivity", self.sensor_absorptivity),
            ("sensor_emissivity", self.sensor_emissivity),
        ] {
            if value <= 0.0 || value > 1.0 {
                return Err(format!("{label} must be in (0, 1], got {value}"));
            }
        }
        if self.n_hat_b.norm() <= 0.1 {
            return Err(format!(
                "n_hat_b must be a non-zero vector, got {:?}",
                self.n_hat_b
            ));
        }
        Ok(())
    }
}

/// Radiating sensor plate: upstream `SensorThermal`.
#[derive(Clone, Debug)]
pub struct SensorThermal {
    /// Sensor geometry and thermal properties, checked at initialization and
    /// reset.
    pub config: SensorThermalConfig,
    /// Upstream `sunInMsg`: Sun position (required).
    pub sun_in_msg: Input<SunEphemerisMsg>,
    /// Upstream `sensorStatusInMsg`: optional on/off status of the sensor.
    pub sensor_status_in_msg: Input<DeviceStatusMsg>,
    /// Upstream `stateInMsg`: spacecraft position and attitude (required).
    pub state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `sunEclipseInMsg`: optional Sun illumination factor.
    pub sun_eclipse_in_msg: Input<EclipseMsg>,
    /// Upstream `temperatureOutMsg`: the sensor temperature.
    pub temperature_out_msg: Output<TemperatureMsg>,
    /// Upstream `sensorPowerStatus`: whether the sensor draws its power; on
    /// by default, overwritten by `sensor_status_in_msg` while connected.
    pub sensor_power_status: DeviceState,
    /// Upstream `illuminationFactor` [-]: one at reset, then the latest
    /// eclipse message.
    illumination_factor: f64,
    /// Upstream `sensorTemp` \[°C\].
    sensor_temp_c: f64,
    /// Upstream `CurrentSimSecondsOld` \[s\]: time of the last update.
    previous_sim_seconds: f64,
}

impl SensorThermal {
    /// A sensor with `config`; the configuration is checked at
    /// initialization.
    pub fn new(config: SensorThermalConfig) -> Self {
        Self {
            sensor_temp_c: config.initial_temperature_c,
            config,
            sun_in_msg: Input::default(),
            sensor_status_in_msg: Input::default(),
            state_in_msg: Input::default(),
            sun_eclipse_in_msg: Input::default(),
            temperature_out_msg: Output::default(),
            sensor_power_status: DeviceState::On,
            illumination_factor: 1.0,
            previous_sim_seconds: 0.0,
        }
    }

    /// Upstream `sensorTemp` \[°C\]: the current sensor temperature.
    pub fn sensor_temperature_c(&self) -> f64 {
        self.sensor_temp_c
    }

    /// The required inputs that are not connected (upstream `Reset`'s "was
    /// not linked" errors).
    pub fn unconnected_inputs(&self) -> Vec<&'static str> {
        [
            (self.sun_in_msg.is_connected(), "sun_in_msg"),
            (self.state_in_msg.is_connected(), "state_in_msg"),
        ]
        .into_iter()
        .filter(|(connected, _)| !connected)
        .map(|(_, port)| port)
        .collect()
    }

    /// Upstream `Reset`, restarting the Euler step at `current_sim_seconds`.
    fn restart(&mut self, current_sim_seconds: f64) {
        self.illumination_factor = 1.0;
        if let Err(msg) = self.config.validate() {
            panic!("invalid SensorThermalConfig '{}': {msg}", self.config.name);
        }
        self.config.n_hat_b.normalize_mut();
        for port in self.unconnected_inputs() {
            log::error!(
                "sensor thermal '{}': {port} is not connected",
                self.config.name
            );
        }
        self.sensor_temp_c = self.config.initial_temperature_c;
        self.previous_sim_seconds = current_sim_seconds;
    }

    /// Upstream `readMessages` and `computeSunData` \[m²\]: the sensor area
    /// projected along the Sun direction.
    fn projected_area_m2(&mut self) -> f64 {
        let sun = self.sun_in_msg.read();
        let state = self.state_in_msg.read();
        if self.sun_eclipse_in_msg.is_connected() {
            self.illumination_factor = self.sun_eclipse_in_msg.read().illumination_factor;
        }
        if self.sensor_status_in_msg.is_connected() {
            self.sensor_power_status = self.sensor_status_in_msg.read().device_status;
        }

        let r_sb_n = sun.sun_position_inertial_m - state.position_m;
        // Eigen's `normalized()`: a zero vector stays zero.
        let s_hat_n = r_sb_n.try_normalize(0.0).unwrap_or_else(Vector3::zeros);
        let s_hat_b = mrp_to_dcm(state.sigma_bn) * s_hat_n;
        (self.config.sensor_area_m2 * s_hat_b.dot(&self.config.n_hat_b)).max(0.0)
    }

    /// Upstream `evaluateThermalModel`.
    fn evaluate_thermal_model(&mut self, projected_area_m2: f64, current_sim_seconds: f64) {
        let config = &self.config;
        let power_on = if self.sensor_power_status.is_on() {
            1.0
        } else {
            0.0
        };
        let q_in_w = self.illumination_factor
            * SOLAR_FLUX_W_PER_M2
            * projected_area_m2
            * config.sensor_absorptivity
            + config.sensor_power_draw_w * power_on;
        let q_out_w = config.sensor_area_m2
            * config.sensor_emissivity
            * BOLTZMANN_W_PER_M2_K4
            * (self.sensor_temp_c + CELSIUS_TO_KELVIN).powi(4);

        let temperature_rate =
            (q_in_w - q_out_w) / (config.sensor_specific_heat_j_per_kg_k * config.sensor_mass_kg);
        self.sensor_temp_c += temperature_rate * (current_sim_seconds - self.previous_sim_seconds);
        self.previous_sim_seconds = current_sim_seconds;
    }
}

impl Module for SensorThermal {
    fn init(&mut self) {
        self.restart(0.0);
        self.temperature_out_msg.write(TemperatureMsg::default());
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.restart(context.current_sim_nanos as f64 * NANO2SEC);
    }

    fn update(&mut self, context: &SimulationContext) {
        let projected_area_m2 = self.projected_area_m2();
        self.evaluate_thermal_model(
            projected_area_m2,
            context.current_sim_nanos as f64 * NANO2SEC,
        );
        self.temperature_out_msg.write(TemperatureMsg {
            temperature_c: self.sensor_temp_c,
        });
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `sensorThermal/_UnitTest/test_sensorThermal.py` with its
    //! 1e-6 accuracy. Upstream checks only the first three of its four logged
    //! temperatures (its loop runs over the three runs, not the four
    //! samples); all four are checked here.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{
        DeviceState, DeviceStatusMsg, EclipseMsg, Output, SpacecraftStateMsg, SunEphemerisMsg,
    };
    use crate::utilities::astro_constants::AU;
    use crate::{Module, SimulationContext};

    use super::{
        BOLTZMANN_W_PER_M2_K4, CELSIUS_TO_KELVIN, SOLAR_FLUX_W_PER_M2, SensorThermal,
        SensorThermalConfig,
    };

    const SECOND_NANOS: u64 = 1_000_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream's 2 kg aluminium sensor with kapton tape optics, drawing
    /// 30 W, with its normal along body `+z`.
    fn kapton_config() -> SensorThermalConfig {
        SensorThermalConfig {
            name: "sensorThermalModel".to_string(),
            n_hat_b: Vector3::new(0.0, 0.0, 1.0),
            sensor_area_m2: 1.0,
            sensor_absorptivity: 0.25,
            sensor_emissivity: 0.34,
            sensor_mass_kg: 2.0,
            sensor_specific_heat_j_per_kg_k: 890.0,
            sensor_power_draw_w: 30.0,
            initial_temperature_c: 0.0,
        }
    }

    struct Inputs {
        _sun: Output<SunEphemerisMsg>,
        _state: Output<SpacecraftStateMsg>,
        status: Output<DeviceStatusMsg>,
    }

    /// The spacecraft at the Earth's radius on `+x` and the Sun 1 AU along
    /// `+x`, with the sensor switched on by its status message.
    fn connect_inputs(sensor: &mut SensorThermal) -> Inputs {
        let status = Output::new(DeviceStatusMsg {
            device_status: DeviceState::On,
        });
        let state = Output::new(SpacecraftStateMsg {
            position_m: Vector3::new(6378.0 * 1000.0, 0.0, 0.0),
            sigma_bn: Vector3::zeros(),
            ..Default::default()
        });
        let sun = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::new(AU * 1000.0, 0.0, 0.0),
            ..Default::default()
        });
        sun.connect_to(&mut sensor.sun_in_msg);
        state.connect_to(&mut sensor.state_in_msg);
        status.connect_to(&mut sensor.sensor_status_in_msg);
        Inputs {
            _sun: sun,
            _state: state,
            status,
        }
    }

    fn logged(sensor: &mut SensorThermal, nanos: u64, log: &mut Vec<f64>) {
        sensor.update(&context(nanos));
        log.push(sensor.temperature_out_msg.read().temperature_c);
    }

    /// Upstream `test_sensorThermal`: one second from 0 °C, then a reset to
    /// the converged hot temperature (about 98 °C) and one more second, then
    /// a reset to the converged cold one (about -75 °C) and one more second.
    /// The sensor faces away from the Sun, so only its power draw heats it.
    #[test]
    fn temperature_steps_from_zero_hot_and_cold_starts() {
        let mut sensor = SensorThermal::new(kapton_config());
        let _inputs = connect_inputs(&mut sensor);
        let mut sensor_temp = Vec::new();

        sensor.init();
        logged(&mut sensor, 0, &mut sensor_temp);
        logged(&mut sensor, SECOND_NANOS, &mut sensor_temp);

        sensor.config.initial_temperature_c = 97.961;
        Module::reset(&mut sensor, &context(SECOND_NANOS));
        logged(&mut sensor, 2 * SECOND_NANOS, &mut sensor_temp);

        sensor.config.initial_temperature_c = -74.8505;
        Module::reset(&mut sensor, &context(2 * SECOND_NANOS));
        logged(&mut sensor, 3 * SECOND_NANOS, &mut sensor_temp);

        let true_temp = [0.0, -0.0443986873, 97.7691478, -74.8506601];
        for (index, (actual, truth)) in sensor_temp.iter().zip(true_temp).enumerate() {
            assert!(
                (actual - truth).abs() < 1.0e-6,
                "test {}: {actual} °C != {truth} °C",
                index + 1
            );
        }
    }

    /// Facing the Sun fully lit, the face absorbs `S · A · α`; an eclipse
    /// input scales it, and a switched-off sensor draws no power.
    #[test]
    fn sunlight_eclipse_and_power_status_drive_the_heat_input() {
        let rate_for = |illumination: Option<f64>, status: DeviceState| {
            let mut config = kapton_config();
            config.n_hat_b = Vector3::new(2.0, 0.0, 0.0);
            let mut sensor = SensorThermal::new(config);
            let inputs = connect_inputs(&mut sensor);
            inputs.status.write(DeviceStatusMsg {
                device_status: status,
            });
            let eclipse = illumination.map(|illumination_factor| {
                Output::new(EclipseMsg {
                    illumination_factor,
                })
            });
            if let Some(eclipse) = &eclipse {
                eclipse.connect_to(&mut sensor.sun_eclipse_in_msg);
            }
            sensor.init();
            sensor.update(&context(0));
            sensor.update(&context(SECOND_NANOS));
            assert_eq!(sensor.config.n_hat_b, Vector3::new(1.0, 0.0, 0.0));
            sensor.sensor_temperature_c()
        };
        // Q_out at 0 °C, over c · m = 1780 J/K.
        let q_out = 0.34 * BOLTZMANN_W_PER_M2_K4 * CELSIUS_TO_KELVIN.powi(4);
        let sun_in = SOLAR_FLUX_W_PER_M2 * 0.25;
        let expect = |q_in: f64| (q_in - q_out) / 1780.0;

        assert!((rate_for(None, DeviceState::On) - expect(sun_in + 30.0)).abs() < 1.0e-12);
        assert!(
            (rate_for(Some(0.5), DeviceState::On) - expect(0.5 * sun_in + 30.0)).abs() < 1.0e-12
        );
        assert!((rate_for(Some(0.0), DeviceState::Off) - expect(0.0)).abs() < 1.0e-12);
    }

    #[test]
    fn unconnected_required_inputs_are_reported() {
        let mut sensor = SensorThermal::new(kapton_config());
        assert_eq!(sensor.unconnected_inputs(), ["sun_in_msg", "state_in_msg"]);
        let _inputs = connect_inputs(&mut sensor);
        assert!(sensor.unconnected_inputs().is_empty());
    }

    /// Breaks one precondition of a valid configuration.
    type BreakConfig = fn(&mut SensorThermalConfig);

    /// Upstream `Reset`'s checks, each naming the offending parameter.
    #[test]
    fn invalid_configurations_are_rejected() {
        let cases: [(&str, BreakConfig); 7] = [
            ("sensor_area_m2", |config| config.sensor_area_m2 = 0.0),
            ("sensor_mass_kg", |config| config.sensor_mass_kg = 0.0),
            ("sensor_specific_heat", |config| {
                config.sensor_specific_heat_j_per_kg_k = 0.0
            }),
            ("sensor_absorptivity", |config| {
                config.sensor_absorptivity = 0.0
            }),
            ("sensor_absorptivity", |config| {
                config.sensor_absorptivity = 1.5
            }),
            ("sensor_emissivity", |config| config.sensor_emissivity = 0.0),
            ("n_hat_b", |config| config.n_hat_b = Vector3::zeros()),
        ];
        for (expected, break_config) in cases {
            let mut config = kapton_config();
            assert_eq!(config.validate(), Ok(()));
            break_config(&mut config);
            let message = config.validate().expect_err("an invalid configuration");
            assert!(message.contains(expected), "{message:?} names {expected:?}");
        }
        assert!(SensorThermalConfig::default().validate().is_err());
    }

    #[test]
    #[should_panic(expected = "sensor_emissivity")]
    fn initialization_rejects_an_invalid_configuration() {
        let mut config = kapton_config();
        config.sensor_emissivity = 2.0;
        SensorThermal::new(config).init();
    }
}
