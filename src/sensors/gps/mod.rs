use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use hifitime::TimeScale;
use nalgebra::{SMatrix, SVector, Vector3};
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, StandardNormal};

use crate::messages::{GpsMsg, Input, Output, SpacecraftStateMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct GpsConfig {
    pub name: String,
    pub p_matrix_sqrt: SMatrix<f64, 6, 6>,
    pub a_matrix: SMatrix<f64, 6, 6>,
    pub walk_bounds: SVector<f64, 6>,
    pub cross_trans: bool,
}

#[derive(Clone, Debug)]
pub struct Gps {
    pub config: GpsConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub output_gps_msg: Output<GpsMsg>,
    prev_time_nanos: u64,
    error_state: SVector<f64, 6>,
    rng: StdRng,
}

impl Module for Gps {
    fn init(&mut self) {
        self.output_gps_msg.write(GpsMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let state = self.read_input_message();
        let true_output = self.compute_true_output(&state, context);
        let sensed_output = self.apply_sensor_errors(true_output, context.current_sim_nanos);

        self.write_output_message(sensed_output);
        self.prev_time_nanos = context.current_sim_nanos;
    }
}

impl Gps {
    pub fn new(config: GpsConfig) -> Self {
        Self {
            rng: StdRng::seed_from_u64(seed_from_name(&config.name)),
            config,
            input_state_msg: Input::default(),
            output_gps_msg: Output::default(),
            prev_time_nanos: 0,
            error_state: SVector::<f64, 6>::zeros(),
        }
    }

    fn read_input_message(&self) -> SpacecraftStateMsg {
        self.input_state_msg.read()
    }

    fn compute_true_output(
        &self,
        state: &SpacecraftStateMsg,
        context: &SimulationContext,
    ) -> GpsMsg {
        let (gps_week, gps_nanoseconds_of_week) = context
            .current_epoch
            .to_time_scale(TimeScale::GPST)
            .to_time_of_week();

        GpsMsg {
            position_m: state.position_m,
            velocity_mps: state.velocity_mps,
            gps_week,
            gps_seconds_of_week: gps_nanoseconds_of_week as f64 * 1.0e-9,
        }
    }

    fn apply_sensor_errors(&mut self, true_output: GpsMsg, current_sim_nanos: u64) -> GpsMsg {
        let dt_seconds = if self.prev_time_nanos == 0 {
            0.0
        } else {
            (current_sim_nanos - self.prev_time_nanos) as f64 * 1.0e-9
        };
        let mut local_a_matrix = self.config.a_matrix;
        if self.config.cross_trans {
            local_a_matrix[(0, 3)] *= dt_seconds;
            local_a_matrix[(1, 4)] *= dt_seconds;
            local_a_matrix[(2, 5)] *= dt_seconds;
        }

        let random_vector = SVector::<f64, 6>::from_fn(|_, _| StandardNormal.sample(&mut self.rng));
        self.error_state =
            local_a_matrix * self.error_state + self.config.p_matrix_sqrt * random_vector;
        self.error_state = self
            .error_state
            .zip_map(&self.config.walk_bounds, |error, bound| {
                if bound > 0.0 {
                    error.clamp(-bound, bound)
                } else {
                    error
                }
            });

        GpsMsg {
            position_m: true_output.position_m
                + Vector3::new(
                    self.error_state[0],
                    self.error_state[1],
                    self.error_state[2],
                ),
            velocity_mps: true_output.velocity_mps
                + Vector3::new(
                    self.error_state[3],
                    self.error_state[4],
                    self.error_state[5],
                ),
            gps_week: true_output.gps_week,
            gps_seconds_of_week: true_output.gps_seconds_of_week,
        }
    }

    fn write_output_message(&mut self, gps_output: GpsMsg) {
        self.output_gps_msg.write(gps_output);
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
    use nalgebra::{SMatrix, SVector, UnitQuaternion, Vector3};

    use crate::messages::{Output, SpacecraftStateMsg};
    use crate::{Module, SimulationContext};

    use super::{Gps, GpsConfig};

    fn zero_noise_config() -> GpsConfig {
        GpsConfig {
            name: "gps".to_string(),
            p_matrix_sqrt: SMatrix::<f64, 6, 6>::zeros(),
            a_matrix: SMatrix::<f64, 6, 6>::zeros(),
            walk_bounds: SVector::<f64, 6>::zeros(),
            cross_trans: false,
        }
    }

    fn make_context(epoch: Epoch, nanos: u64) -> SimulationContext {
        SimulationContext {
            start_epoch: epoch,
            current_sim_nanos: nanos,
            current_epoch: epoch,
        }
    }

    fn run_gps(
        gps: &mut Gps,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        context: &SimulationContext,
    ) {
        let state_out = Output::new(SpacecraftStateMsg {
            position_m: position,
            velocity_mps: velocity,
            attitude_b_to_i: UnitQuaternion::identity(),
            omega_radps: Vector3::zeros(),
        });
        gps.input_state_msg.connect(state_out.slot());
        gps.init();
        gps.update(context);
    }

    /// Zero noise: output position and velocity equal input exactly.
    #[test]
    fn zero_noise_passes_through_position_and_velocity() {
        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        let ctx = make_context(epoch, 0);
        let pos = Vector3::new(6_778_000.0, 0.0, 0.0);
        let vel = Vector3::new(0.0, 7784.0, 0.0);

        let mut gps = Gps::new(zero_noise_config());
        run_gps(&mut gps, pos, vel, &ctx);
        let out = gps.output_gps_msg.read();

        assert!(
            (out.position_m - pos).norm() < 1e-12,
            "position: expected {pos:?}, got {:?}",
            out.position_m
        );
        assert!(
            (out.velocity_mps - vel).norm() < 1e-12,
            "velocity: expected {vel:?}, got {:?}",
            out.velocity_mps
        );
    }

    /// GPS week is positive and seconds_of_week is in [0, 604800).
    #[test]
    fn gps_time_fields_are_valid() {
        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        let ctx = make_context(epoch, 0);

        let mut gps = Gps::new(zero_noise_config());
        run_gps(&mut gps, Vector3::zeros(), Vector3::zeros(), &ctx);
        let out = gps.output_gps_msg.read();

        assert!(
            out.gps_week > 0,
            "gps_week should be > 0 for 2025, got {}",
            out.gps_week
        );
        assert!(
            out.gps_seconds_of_week >= 0.0 && out.gps_seconds_of_week < 604800.0,
            "gps_seconds_of_week out of range: {}",
            out.gps_seconds_of_week
        );
    }

    /// Two GPS units with different names get different seeds → different noise sequences.
    /// With nonzero noise, their outputs diverge after first update.
    #[test]
    fn different_names_produce_different_noise() {
        use nalgebra::SMatrix;
        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        let ctx = make_context(epoch, 1_000_000_000); // 1 second (non-zero dt)

        let noisy_config = |name: &str| GpsConfig {
            name: name.to_string(),
            p_matrix_sqrt: SMatrix::<f64, 6, 6>::identity() * 1.0,
            a_matrix: SMatrix::<f64, 6, 6>::zeros(),
            walk_bounds: SVector::<f64, 6>::zeros(),
            cross_trans: false,
        };

        let pos = Vector3::new(6_778_000.0, 0.0, 0.0);
        let vel = Vector3::zeros();

        let mut gps_a = Gps::new(noisy_config("gps_a"));
        let mut gps_b = Gps::new(noisy_config("gps_b"));
        run_gps(&mut gps_a, pos, vel, &ctx);
        run_gps(&mut gps_b, pos, vel, &ctx);

        let out_a = gps_a.output_gps_msg.read();
        let out_b = gps_b.output_gps_msg.read();
        assert_ne!(
            out_a.position_m, out_b.position_m,
            "different seeds should produce different noise"
        );
    }
}
