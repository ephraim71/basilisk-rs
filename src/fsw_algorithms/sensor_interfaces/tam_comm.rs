use nalgebra::Matrix3;

use crate::messages::{Input, Output, TamSensorBodyMsg, TamSensorMsg};
use crate::{Module, SimulationContext};

const DCM_DETERMINANT_TOLERANCE: f64 = 1.0e-10;

#[derive(Clone, Debug)]
pub struct TamCommConfig {
    pub name: String,
    /// Direction cosine matrix mapping sensor-frame vectors into the body frame.
    pub dcm_bs: Matrix3<f64>,
}

#[derive(Clone, Debug)]
pub struct TamComm {
    pub config: TamCommConfig,
    pub tam_in_msg: Input<TamSensorMsg>,
    pub tam_out_msg: Output<TamSensorBodyMsg>,
}

impl TamComm {
    pub fn new(config: TamCommConfig) -> Self {
        Self {
            config,
            tam_in_msg: Input::default(),
            tam_out_msg: Output::default(),
        }
    }

    fn validate_configuration(&self) {
        if !self.tam_in_msg.is_connected() {
            log::error!("tamComm.tam_in_msg is not connected");
        }

        let determinant = self.config.dcm_bs.determinant();
        if (determinant - 1.0).abs() > DCM_DETERMINANT_TOLERANCE {
            log::warn!("tamComm.dcm_bs determinant should be 1.0, got {determinant}");
        }
    }
}

impl Module for TamComm {
    fn init(&mut self) {
        self.tam_out_msg.write(TamSensorBodyMsg::default());
        self.validate_configuration();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.validate_configuration();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let magnetic_field_sensor_t = self.tam_in_msg.read().magnetic_field_sensor_t;
        let magnetic_field_body_t = self.config.dcm_bs * magnetic_field_sensor_t;

        self.tam_out_msg.write(TamSensorBodyMsg {
            magnetic_field_body_t,
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use rand_distr::{Distribution, StandardNormal};

    use crate::messages::{Input, Output, TamSensorBodyMsg, TamSensorMsg};
    use crate::simulation::Simulation;
    use crate::{Module, SimulationContext};

    use super::{TamComm, TamCommConfig};

    #[derive(Debug, Default)]
    struct TamBodyRecorder {
        input_msg: Input<TamSensorBodyMsg>,
        samples: Vec<Vector3<f64>>,
    }

    impl Module for TamBodyRecorder {
        fn init(&mut self) {
            self.samples.clear();
        }

        fn update(&mut self, _context: &SimulationContext) {
            self.samples
                .push(self.input_msg.read().magnetic_field_body_t);
        }
    }

    #[test]
    fn matches_upstream_tam_comm_unit_test() {
        let mut rng = StdRng::seed_from_u64(0x0054_414d_434f_4d4d);
        let random_matrix = Matrix3::from_fn(|_, _| StandardNormal.sample(&mut rng));
        let dcm_bs = random_matrix.qr().q();
        let magnetic_field_sensor_t = Vector3::new(-1.0e-5, 2.0e-6, -3.0e-5);
        let expected = dcm_bs * magnetic_field_sensor_t;

        let sensor_output = Output::new(TamSensorMsg {
            magnetic_field_sensor_t,
        });
        let mut module = TamComm::new(TamCommConfig {
            name: "tamComm".to_string(),
            dcm_bs,
        });
        let mut recorder = TamBodyRecorder::default();

        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&sensor_output, &mut module.tam_in_msg);
        simulation.connect(&module.tam_out_msg, &mut recorder.input_msg);
        simulation.add_module("tamComm", &mut module, 500_000_000, 0);
        simulation.add_module("tamRecorder", &mut recorder, 500_000_000, 0);
        simulation.run_for(1_000_000_000);
        drop(simulation);

        assert_eq!(recorder.samples.len(), 3);
        for actual in recorder.samples {
            for index in 0..3 {
                assert!(
                    (actual[index] - expected[index]).abs() <= 1.0e-12,
                    "component {index}: expected {}, got {}",
                    expected[index],
                    actual[index],
                );
            }
        }
    }

    #[test]
    fn rotates_sensor_x_into_body_y_for_quarter_turn_mounting() {
        let dcm_bs = Matrix3::new(
            0.0, -1.0, 0.0, //
            1.0, 0.0, 0.0, //
            0.0, 0.0, 1.0,
        );
        let sensor_output = Output::new(TamSensorMsg {
            magnetic_field_sensor_t: Vector3::x(),
        });
        let mut module = TamComm::new(TamCommConfig {
            name: "tamComm".to_string(),
            dcm_bs,
        });
        module.tam_in_msg.connect(sensor_output.slot());

        module.init();
        module.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });

        assert_eq!(
            module.tam_out_msg.read().magnetic_field_body_t,
            Vector3::y()
        );
    }

    #[test]
    fn connects_to_magnetometer_and_recovers_body_frame_field() {
        use nalgebra::UnitQuaternion;

        use crate::messages::{MagneticFieldMsg, SpacecraftStateMsg};
        use crate::sensors::magnetometer::{Magnetometer, MagnetometerConfig};

        let body_to_sensor_quaternion =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.5 * std::f64::consts::PI);
        let dcm_bs = body_to_sensor_quaternion
            .inverse()
            .to_rotation_matrix()
            .into_inner();
        let expected_body_field_t = Vector3::new(1.0e-5, -2.0e-5, 3.0e-5);
        let state_output = Output::new(SpacecraftStateMsg::default());
        let field_output = Output::new(MagneticFieldMsg {
            magnetic_field_inertial_t: expected_body_field_t,
        });
        let mut magnetometer = Magnetometer::new(MagnetometerConfig {
            name: "tam".to_string(),
            body_to_sensor_quaternion,
            bias_t: Vector3::zeros(),
            p_matrix_sqrt_t: Matrix3::zeros(),
            a_matrix: Matrix3::identity(),
            walk_bounds_t: Vector3::zeros(),
            scale_factor: 1.0,
            min_output_t: -1.0,
            max_output_t: 1.0,
            mtb_residual_field: None,
        });
        let mut module = TamComm::new(TamCommConfig {
            name: "tamComm".to_string(),
            dcm_bs,
        });

        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&state_output, &mut magnetometer.input_state_msg);
        simulation.connect(&field_output, &mut magnetometer.input_magnetic_field_msg);
        simulation.connect(&magnetometer.output_tam_msg, &mut module.tam_in_msg);
        simulation.add_module("magnetometer", &mut magnetometer, 500_000_000, 0);
        simulation.add_module("tamComm", &mut module, 500_000_000, 0);
        simulation.run_for(0);
        drop(simulation);

        let actual_body_field_t = module.tam_out_msg.read().magnetic_field_body_t;
        assert!(
            (actual_body_field_t - expected_body_field_t).norm() <= 1.0e-12,
            "expected {expected_body_field_t:?}, got {actual_body_field_t:?}",
        );
    }

    #[test]
    fn unconnected_input_preserves_upstream_zero_payload_behavior() {
        let mut module = TamComm::new(TamCommConfig {
            name: "tamComm".to_string(),
            dcm_bs: Matrix3::identity(),
        });
        let context = SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        };

        module.init();
        module.update(&context);

        assert_eq!(
            module.tam_out_msg.read().magnetic_field_body_t,
            Vector3::zeros()
        );
    }

    #[test]
    fn non_rotation_matrix_is_warned_about_but_still_applied() {
        let dcm_bs = Matrix3::from_diagonal_element(2.0);
        let sensor_output = Output::new(TamSensorMsg {
            magnetic_field_sensor_t: Vector3::new(1.0, -2.0, 3.0),
        });
        let mut module = TamComm::new(TamCommConfig {
            name: "tamComm".to_string(),
            dcm_bs,
        });
        module.tam_in_msg.connect(sensor_output.slot());

        module.init();
        module.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });

        assert_eq!(
            module.tam_out_msg.read().magnetic_field_body_t,
            Vector3::new(2.0, -4.0, 6.0)
        );
    }

    #[test]
    fn reset_preserves_the_last_body_frame_output() {
        let sensor_output = Output::new(TamSensorMsg {
            magnetic_field_sensor_t: Vector3::new(1.0e-5, -2.0e-5, 3.0e-5),
        });
        let mut module = TamComm::new(TamCommConfig {
            name: "tamComm".to_string(),
            dcm_bs: Matrix3::identity(),
        });
        module.tam_in_msg.connect(sensor_output.slot());
        let context = SimulationContext {
            current_sim_nanos: 500_000_000,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        };
        module.init();
        module.update(&context);
        let before = module.tam_out_msg.read();

        Module::reset(&mut module, &context);

        assert_eq!(
            module.tam_out_msg.read().magnetic_field_body_t,
            before.magnetic_field_body_t
        );
    }
}
