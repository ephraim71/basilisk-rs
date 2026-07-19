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

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_693.0;

/// Below this Kelly factor the fit is treated as disabled, avoiding division by a
/// near-zero denominator in the Kelly curve.
const KELLY_FACTOR_EPSILON: f64 = 1.0e-10;

#[derive(Clone, Debug)]
pub struct CoarseSunSensorConfig {
    pub name: String,
    pub position_m: Vector3<f64>,
    /// Sensor mounting orientation: rotation from body frame to sensor frame.
    /// The boresight is this rotation applied to the sensor +z axis.
    pub body_to_sensor_quaternion: UnitQuaternion<f64>,
    /// Half-angle of the sensor's conical field of view, in radians (0..=PI).
    pub fov_half_angle_rad: f64,
    pub scale_factor: f64,
    pub kelly_factor: f64,
    pub k_power: f64,
    pub bias: f64,
    pub noise_std: f64,
    pub noise_prop: f64,
    pub walk_bounds: f64,
    pub min_output: f64,
    pub max_output: f64,
}

impl CoarseSunSensorConfig {
    /// Check physical and numeric invariants. Returns a description of the first
    /// violation, or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        use std::f64::consts::PI;

        let scalars = [
            ("scale_factor", self.scale_factor),
            ("kelly_factor", self.kelly_factor),
            ("k_power", self.k_power),
            ("bias", self.bias),
            ("noise_std", self.noise_std),
            ("noise_prop", self.noise_prop),
            ("walk_bounds", self.walk_bounds),
            ("fov_half_angle_rad", self.fov_half_angle_rad),
            ("min_output", self.min_output),
            ("max_output", self.max_output),
        ];
        for (name, value) in scalars {
            if !value.is_finite() {
                return Err(format!("{name} must be finite, got {value}"));
            }
        }
        if !self.position_m.iter().all(|v| v.is_finite()) {
            return Err(format!("position_m must be finite, got {:?}", self.position_m));
        }
        if !(0.0..=PI).contains(&self.fov_half_angle_rad) {
            return Err(format!(
                "fov_half_angle_rad must be within [0, PI], got {}",
                self.fov_half_angle_rad
            ));
        }
        if self.kelly_factor < 0.0 {
            return Err(format!("kelly_factor must be >= 0, got {}", self.kelly_factor));
        }
        if self.k_power <= 0.0 {
            return Err(format!("k_power must be > 0, got {}", self.k_power));
        }
        if self.noise_std < 0.0 {
            return Err(format!("noise_std must be >= 0, got {}", self.noise_std));
        }
        if self.min_output > self.max_output {
            return Err(format!(
                "min_output ({}) must not exceed max_output ({})",
                self.min_output, self.max_output
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct CoarseSunSensor {
    pub config: CoarseSunSensorConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub input_sun_msg: Input<SunEphemerisMsg>,
    pub input_eclipse_msg: Input<EclipseMsg>,
    pub output_sun_sensor_msg: Output<SunSensorMsg>,
    noise_model: GaussMarkov,
}

/// First-order bounded Gauss-Markov random walk for the scalar (1-state) case.
#[derive(Clone, Debug)]
struct GaussMarkov {
    rng: StdRng,
    prop_coefficient: f64,
    noise_std: f64,
    upper_bound: f64,
    state: f64,
}

impl GaussMarkov {
    fn new(seed: u64, prop_coefficient: f64, noise_std: f64, upper_bound: f64) -> Self {
        Self {
            rng: StdRng::seed_from_u64(seed),
            prop_coefficient,
            noise_std,
            upper_bound,
            state: 0.0,
        }
    }

    /// Advance the process one step and return the new error state.
    /// `state = prop * state + noiseStd * N(0,1)`, then clamp to `±upper_bound`
    /// when `upper_bound > 0`
    fn compute_next_state(&mut self) -> f64 {
        let sample = Normal::new(0.0, 1.0)
            .expect("unit normal is valid")
            .sample(&mut self.rng);
        self.state = self.prop_coefficient * self.state + self.noise_std * sample;
        if self.upper_bound > 0.0 && self.state.abs() > self.upper_bound {
            self.state = self.upper_bound.copysign(self.state);
        }
        self.state
    }
}

impl Module for CoarseSunSensor {
    fn init(&mut self) {
        // On reset, (re)configure the noise model from the current config.
        self.noise_model = GaussMarkov::new(
            seed_from_name(&self.config.name),
            self.config.noise_prop,
            self.config.noise_std,
            self.config.walk_bounds,
        );
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

impl CoarseSunSensor {
    pub fn new(config: CoarseSunSensorConfig) -> Self {
        if let Err(msg) = config.validate() {
            panic!("invalid CoarseSunSensorConfig: {msg}");
        }
        let noise_model = GaussMarkov::new(
            seed_from_name(&config.name),
            config.noise_prop,
            config.noise_std,
            config.walk_bounds,
        );
        Self {
            noise_model,
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
            let kelly_fit = if self.config.kelly_factor > KELLY_FACTOR_EPSILON {
                1.0 - (-(true_value.powf(self.config.k_power)) / self.config.kelly_factor).exp()
            } else {
                1.0
            };
            true_value *= kelly_fit;
            true_value *= (ASTRONOMICAL_UNIT_M * ASTRONOMICAL_UNIT_M)
                / spacecraft_to_sun_inertial_m.norm_squared();
            true_value *= eclipse.illumination_factor;
        }

        let mut sensed_value = true_value + self.compute_sensor_error();
        true_value *= self.config.scale_factor;
        sensed_value *= self.config.scale_factor;
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

    /// Additive sensor error applied to the true value: bias alone when noise is
    /// disabled, otherwise bias plus the next Gauss-Markov error step.
    fn compute_sensor_error(&mut self) -> f64 {
        if self.config.noise_std <= 0.0 {
            self.config.bias
        } else {
            self.config.bias + self.noise_model.compute_next_state()
        }
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

    use super::{ASTRONOMICAL_UNIT_M, CoarseSunSensor, CoarseSunSensorConfig};

    fn dummy_context() -> SimulationContext {
        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: epoch,
        }
    }

    fn aligned_sensor() -> CoarseSunSensor {
        CoarseSunSensor::new(CoarseSunSensorConfig {
            name: "css".to_string(),
            position_m: Vector3::zeros(),
            body_to_sensor_quaternion: UnitQuaternion::identity(),
            fov_half_angle_rad: 80.0_f64.to_radians(),
            scale_factor: 1.0,
            kelly_factor: 0.0,
            k_power: 2.0,
            bias: 0.0,
            noise_std: 0.0,
            noise_prop: 1.0,
            walk_bounds: -1.0,
            min_output: 0.0,
            max_output: 1.0e6,
        })
    }

    fn nominal_state() -> SpacecraftStateMsg {
        SpacecraftStateMsg {
            position_m: Vector3::zeros(),
            velocity_mps: Vector3::zeros(),
            sigma_bn: Vector3::zeros(),
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

    /// Drive a configured sensor with the sun at `sun_pos` and return the output.
    fn run_with_sun(mut sensor: CoarseSunSensor, sun_pos: Vector3<f64>) -> super::SunSensorMsg {
        let state_out = Output::new(nominal_state());
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: sun_pos,
            sun_velocity_inertial_mps: Vector3::zeros(),
        });
        sensor.input_state_msg.connect(state_out.slot());
        sensor.input_sun_msg.connect(sun_out.slot());
        sensor.init();
        sensor.update(&dummy_context());
        sensor.output_sun_sensor_msg.read()
    }

    #[test]
    fn kelly_factor_attenuates_boresight_signal() {
        let mut sensor = aligned_sensor();
        sensor.config.kelly_factor = 0.15;
        sensor.config.k_power = 2.0;

        let msg = run_with_sun(sensor, Vector3::new(0.0, 0.0, ASTRONOMICAL_UNIT_M));

        // raw signal = 1.0, kellyFit = 1 - e^(-1^2 / 0.15)
        let expected = 1.0 - (-1.0_f64 / 0.15).exp();
        assert!(
            (msg.true_value - expected).abs() < 1.0e-12,
            "expected kelly-attenuated value {expected}, got {msg:?}"
        );
    }

    #[test]
    fn scale_factor_multiplies_true_and_sensed() {
        let mut sensor = aligned_sensor();
        sensor.config.scale_factor = 2.0;

        let msg = run_with_sun(sensor, Vector3::new(0.0, 0.0, ASTRONOMICAL_UNIT_M));

        assert!((msg.true_value - 2.0).abs() < 1.0e-12, "got {msg:?}");
        assert!((msg.sensed_value - 2.0).abs() < 1.0e-12, "got {msg:?}");
    }

    #[test]
    fn bias_offsets_sensed_but_not_true() {
        let mut sensor = aligned_sensor();
        sensor.config.bias = 0.1;

        let msg = run_with_sun(sensor, Vector3::new(0.0, 0.0, ASTRONOMICAL_UNIT_M));

        assert!((msg.true_value - 1.0).abs() < 1.0e-12, "got {msg:?}");
        assert!((msg.sensed_value - 1.1).abs() < 1.0e-12, "got {msg:?}");
    }

    #[test]
    fn saturation_clamps_sensed_but_leaves_true_unclamped() {
        let mut sensor = aligned_sensor();
        sensor.config.max_output = 0.5;

        let msg = run_with_sun(sensor, Vector3::new(0.0, 0.0, ASTRONOMICAL_UNIT_M));

        // sensed pre-clamp is 1.0 -> saturated to max_output; true value is not saturated.
        assert!(
            (msg.sensed_value - 0.5).abs() < 1.0e-12,
            "expected sensed clamped to 0.5, got {msg:?}"
        );
        assert!(
            (msg.true_value - 1.0).abs() < 1.0e-12,
            "expected true value left unclamped at 1.0, got {msg:?}"
        );
    }

    #[test]
    fn sun_distance_scales_signal_by_inverse_square() {
        let sensor = aligned_sensor();

        // Sun at 2 AU -> distance factor = (1/2)^2 = 0.25.
        let msg = run_with_sun(sensor, Vector3::new(0.0, 0.0, 2.0 * ASTRONOMICAL_UNIT_M));

        assert!(
            (msg.true_value - 0.25).abs() < 1.0e-12,
            "expected inverse-square scaling to 0.25, got {msg:?}"
        );
    }

    /// Connect a sensor to a boresight-aligned sun and return it ready to step.
    fn connect_aligned(
        sensor: &mut CoarseSunSensor,
    ) -> (Output<SpacecraftStateMsg>, Output<SunEphemerisMsg>) {
        let state_out = Output::new(nominal_state());
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::new(0.0, 0.0, ASTRONOMICAL_UNIT_M),
            sun_velocity_inertial_mps: Vector3::zeros(),
        });
        sensor.input_state_msg.connect(state_out.slot());
        sensor.input_sun_msg.connect(sun_out.slot());
        sensor.init();
        (state_out, sun_out)
    }

    #[test]
    fn gauss_markov_iid_noise_matches_configured_std() {
        let mut sensor = aligned_sensor();
        sensor.config.noise_std = 0.05;
        sensor.config.noise_prop = 0.0; // IID white-noise special case
        let _keepalive = connect_aligned(&mut sensor);

        let n = 20_000;
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        for _ in 0..n {
            sensor.update(&dummy_context());
            let msg = sensor.output_sun_sensor_msg.read();
            let noise = msg.sensed_value - msg.true_value;
            sum += noise;
            sum_sq += noise * noise;
        }
        let mean = sum / n as f64;
        let std = (sum_sq / n as f64 - mean * mean).sqrt();

        assert!(mean.abs() < 2.0e-3, "noise mean should be ~0, got {mean}");
        assert!(
            (std - 0.05).abs() < 2.5e-3,
            "noise std should match configured 0.05, got {std}"
        );
    }

    #[test]
    #[should_panic(expected = "fov_half_angle_rad")]
    fn rejects_fov_outside_range() {
        let mut config = aligned_sensor().config;
        config.fov_half_angle_rad = 4.0; // greater than PI
        let _ = CoarseSunSensor::new(config);
    }

    #[test]
    #[should_panic(expected = "min_output")]
    fn rejects_min_output_above_max() {
        let mut config = aligned_sensor().config;
        config.min_output = 2.0;
        config.max_output = 1.0;
        let _ = CoarseSunSensor::new(config);
    }

    #[test]
    fn gauss_markov_walk_respects_bounds() {
        let mut sensor = aligned_sensor();
        sensor.config.noise_std = 1.0; // large kicks
        sensor.config.noise_prop = 1.0; // random walk
        sensor.config.walk_bounds = 0.3; // fenced in
        let _keepalive = connect_aligned(&mut sensor);

        for _ in 0..10_000 {
            sensor.update(&dummy_context());
            let msg = sensor.output_sun_sensor_msg.read();
            let noise = msg.sensed_value - msg.true_value;
            assert!(
                noise.abs() <= 0.3 + 1.0e-12,
                "walk error {noise} exceeded bound 0.3"
            );
        }
    }
}
