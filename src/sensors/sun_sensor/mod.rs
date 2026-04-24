use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use nalgebra::{UnitQuaternion, Vector3};
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, Normal};

use crate::messages::{
    EclipseMsg, Input, Output, SpacecraftStateMsg, SunEphemerisMsg, SunSensorMsg,
};
use crate::{Module, SimulationContext};

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_700.0;

#[derive(Clone, Debug)]
pub struct SunSensorConfig {
    pub name: String,
    pub position_m: Vector3<f64>,
    pub body_to_sensor_quaternion: UnitQuaternion<f64>,
    pub fov_half_angle_rad: f64,
    pub scale_factor: f64,
    pub kelly_factor: f64,
    pub k_power: f64,
    pub bias: f64,
    pub noise_std: f64,
    pub min_output: f64,
    pub max_output: f64,
}

#[derive(Clone, Debug)]
pub struct SunSensor {
    pub config: SunSensorConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub input_sun_msg: Input<SunEphemerisMsg>,
    pub input_eclipse_msg: Input<EclipseMsg>,
    pub output_sun_sensor_msg: Output<SunSensorMsg>,
    rng: StdRng,
}

impl Module for SunSensor {
    fn init(&mut self) {
        self.output_sun_sensor_msg.write(SunSensorMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.read_state_input_message();
        let sun = self.read_sun_input_message();
        let eclipse = self.read_eclipse_input_message();

        let measurement = self.compute_measurement(&state, &sun, &eclipse);
        self.write_output_message(measurement);
    }
}

impl SunSensor {
    pub fn new(config: SunSensorConfig) -> Self {
        Self {
            rng: StdRng::seed_from_u64(seed_from_name(&config.name)),
            config,
            input_state_msg: Input::default(),
            input_sun_msg: Input::default(),
            input_eclipse_msg: Input::default(),
            output_sun_sensor_msg: Output::default(),
        }
    }

    fn read_state_input_message(&self) -> SpacecraftStateMsg {
        self.input_state_msg.read()
    }

    fn read_sun_input_message(&self) -> SunEphemerisMsg {
        self.input_sun_msg.read()
    }

    fn read_eclipse_input_message(&self) -> EclipseMsg {
        if self.input_eclipse_msg.is_connected() {
            self.input_eclipse_msg.read()
        } else {
            EclipseMsg {
                illumination_factor: 1.0,
            }
        }
    }

    fn compute_measurement(
        &mut self,
        state: &SpacecraftStateMsg,
        sun: &SunEphemerisMsg,
        eclipse: &EclipseMsg,
    ) -> SunSensorMsg {
        let spacecraft_to_sun_inertial_m = sun.sun_position_inertial_m - state.position_m;
        if spacecraft_to_sun_inertial_m.norm_squared() == 0.0 {
            return SunSensorMsg::default();
        }

        let sun_direction_inertial = spacecraft_to_sun_inertial_m.normalize();
        let sun_direction_body = state
            .inertial_to_body()
            .transform_vector(&sun_direction_inertial);
        let raw_signal = self.sensor_normal_body().dot(&sun_direction_body);
        let mut true_value = if raw_signal >= self.config.fov_half_angle_rad.cos() {
            raw_signal
        } else {
            0.0
        };

        if true_value > 0.0 {
            let kelly_fit = if self.config.kelly_factor > 1.0e-10 {
                1.0 - (-(true_value.powf(self.config.k_power)) / self.config.kelly_factor).exp()
            } else {
                1.0
            };
            true_value *= kelly_fit;
            true_value *= (ASTRONOMICAL_UNIT_M * ASTRONOMICAL_UNIT_M)
                / spacecraft_to_sun_inertial_m.norm_squared();
            true_value *= eclipse.illumination_factor;
        }

        let mut sensed_value =
            true_value + self.config.bias + self.sample_gaussian_noise(self.config.noise_std);
        true_value *= self.config.scale_factor;
        sensed_value *= self.config.scale_factor;

        true_value = true_value.clamp(self.config.min_output, self.config.max_output);
        sensed_value = sensed_value.clamp(self.config.min_output, self.config.max_output);

        SunSensorMsg {
            sensed_value,
            true_value,
            valid: sensed_value > 0.0,
        }
    }

    fn write_output_message(&mut self, measurement: SunSensorMsg) {
        self.output_sun_sensor_msg.write(measurement);
    }

    fn sensor_normal_body(&self) -> Vector3<f64> {
        self.config
            .body_to_sensor_quaternion
            .inverse()
            .transform_vector(&Vector3::new(0.0, 0.0, 1.0))
            .normalize()
    }

    fn sample_gaussian_noise(&mut self, std_dev: f64) -> f64 {
        if std_dev == 0.0 {
            return 0.0;
        }

        Normal::new(0.0, std_dev)
            .expect("std_dev must be positive")
            .sample(&mut self.rng)
    }
}

fn seed_from_name(name: &str) -> u64 {
    let mut hasher = DefaultHasher::new();
    name.hash(&mut hasher);
    hasher.finish()
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{UnitQuaternion, Vector3};

    use crate::messages::{EclipseMsg, Output, SpacecraftStateMsg, SunEphemerisMsg};
    use crate::{Module, SimulationContext};

    use super::{ASTRONOMICAL_UNIT_M, SunSensor, SunSensorConfig};

    fn dummy_context() -> SimulationContext {
        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        SimulationContext {
            start_epoch: epoch,
            current_sim_nanos: 0,
            current_epoch: epoch,
        }
    }

    fn aligned_sensor() -> SunSensor {
        SunSensor::new(SunSensorConfig {
            name: "css".to_string(),
            position_m: Vector3::zeros(),
            body_to_sensor_quaternion: UnitQuaternion::identity(),
            fov_half_angle_rad: 80.0_f64.to_radians(),
            scale_factor: 1.0,
            kelly_factor: 0.0,
            k_power: 2.0,
            bias: 0.0,
            noise_std: 0.0,
            min_output: 0.0,
            max_output: 1.0e6,
        })
    }

    fn nominal_state() -> SpacecraftStateMsg {
        SpacecraftStateMsg {
            position_m: Vector3::zeros(),
            velocity_mps: Vector3::zeros(),
            attitude_b_to_i: UnitQuaternion::identity(),
            omega_radps: Vector3::zeros(),
        }
    }

    #[test]
    fn boresight_alignment_returns_unity_signal_at_one_au() {
        let mut sensor = aligned_sensor();
        let state_out = Output::new(nominal_state());
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::new(0.0, 0.0, ASTRONOMICAL_UNIT_M),
            sun_velocity_inertial_mps: Vector3::zeros(),
        });

        sensor.input_state_msg.connect(state_out.slot());
        sensor.input_sun_msg.connect(sun_out.slot());
        sensor.init();
        sensor.update(&dummy_context());

        let msg = sensor.output_sun_sensor_msg.read();
        assert!(
            (msg.true_value - 1.0).abs() < 1.0e-12,
            "expected unity true value, got {msg:?}"
        );
        assert!(
            (msg.sensed_value - 1.0).abs() < 1.0e-12,
            "expected unity sensed value, got {msg:?}"
        );
        assert!(msg.valid);
    }

    #[test]
    fn outside_field_of_view_returns_zero_signal() {
        let mut sensor = aligned_sensor();
        let state_out = Output::new(nominal_state());
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::new(ASTRONOMICAL_UNIT_M, 0.0, 0.0),
            sun_velocity_inertial_mps: Vector3::zeros(),
        });

        sensor.input_state_msg.connect(state_out.slot());
        sensor.input_sun_msg.connect(sun_out.slot());
        sensor.init();
        sensor.update(&dummy_context());

        let msg = sensor.output_sun_sensor_msg.read();
        assert!(
            msg.true_value.abs() < 1.0e-12,
            "expected zero true value, got {msg:?}"
        );
        assert!(
            msg.sensed_value.abs() < 1.0e-12,
            "expected zero sensed value, got {msg:?}"
        );
        assert!(!msg.valid);
    }

    #[test]
    fn eclipse_scales_signal() {
        let mut sensor = aligned_sensor();
        let state_out = Output::new(nominal_state());
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::new(0.0, 0.0, ASTRONOMICAL_UNIT_M),
            sun_velocity_inertial_mps: Vector3::zeros(),
        });
        let eclipse_out = Output::new(EclipseMsg {
            illumination_factor: 0.25,
        });

        sensor.input_state_msg.connect(state_out.slot());
        sensor.input_sun_msg.connect(sun_out.slot());
        sensor.input_eclipse_msg.connect(eclipse_out.slot());
        sensor.init();
        sensor.update(&dummy_context());

        let msg = sensor.output_sun_sensor_msg.read();
        assert!(
            (msg.true_value - 0.25).abs() < 1.0e-12,
            "expected eclipse scaling, got {msg:?}"
        );
        assert!(
            (msg.sensed_value - 0.25).abs() < 1.0e-12,
            "expected eclipse scaling, got {msg:?}"
        );
        assert!(msg.valid);
    }
}
