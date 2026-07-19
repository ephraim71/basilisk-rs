use nalgebra::{DMatrix, DVector, Vector3};

use crate::messages::{Input, Output, SunLineMsg, SunSensorMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct CssWlsEstConfig {
    pub name: String,
    pub sensor_normals_body: Vec<Vector3<f64>>,
    pub sensor_use_thresh: f64,
    pub use_weights: bool,
}

#[derive(Clone, Debug)]
pub struct CssWlsEst {
    pub config: CssWlsEstConfig,
    pub css_data_in_msgs: Vec<Input<SunSensorMsg>>,
    pub nav_state_out_msg: Output<SunLineMsg>,
}

impl CssWlsEst {
    pub fn new(config: CssWlsEstConfig) -> Self {
        Self {
            css_data_in_msgs: vec![Input::default(); config.sensor_normals_body.len()],
            config,
            nav_state_out_msg: Output::default(),
        }
    }

    fn estimate_sunline(&self) -> SunLineMsg {
        let mut active_normals = Vec::new();
        let mut active_measurements = Vec::new();

        for (input, normal) in self
            .css_data_in_msgs
            .iter()
            .zip(self.config.sensor_normals_body.iter())
        {
            let measurement = input.read().sensed_value;
            if measurement > self.config.sensor_use_thresh {
                active_normals.push(normalize_or_zero(*normal));
                active_measurements.push(measurement);
            }
        }

        let num_active = active_measurements.len();
        if num_active == 0 {
            return SunLineMsg::default();
        }

        let estimate = if num_active == 1 {
            active_normals[0] * active_measurements[0]
        } else if num_active == 2 {
            let h = DMatrix::from_row_slice(
                2,
                3,
                &[
                    active_normals[0].x,
                    active_normals[0].y,
                    active_normals[0].z,
                    active_normals[1].x,
                    active_normals[1].y,
                    active_normals[1].z,
                ],
            );
            let y = DVector::from_column_slice(&active_measurements);
            let hh_t = &h * h.transpose();
            let Some(hh_t_inv) = hh_t.try_inverse() else {
                return SunLineMsg::default();
            };
            let x = h.transpose() * hh_t_inv * y;
            Vector3::new(x[0], x[1], x[2])
        } else {
            let mut h_data = Vec::with_capacity(num_active * 3);
            for normal in &active_normals {
                h_data.extend_from_slice(&[normal.x, normal.y, normal.z]);
            }
            let h = DMatrix::from_row_slice(num_active, 3, &h_data);
            let y = DVector::from_column_slice(&active_measurements);
            let w = if self.config.use_weights {
                DMatrix::from_diagonal(&y)
            } else {
                DMatrix::identity(num_active, num_active)
            };
            let h_t = h.transpose();
            let h_t_w = &h_t * &w;
            let normal_matrix = &h_t_w * &h;
            let Some(normal_matrix_inv) = normal_matrix.try_inverse() else {
                return SunLineMsg::default();
            };
            let x = normal_matrix_inv * h_t_w * y;
            Vector3::new(x[0], x[1], x[2])
        };

        let norm = estimate.norm();
        if norm <= 1.0e-12 {
            return SunLineMsg::default();
        }

        SunLineMsg {
            sun_vector_body: estimate / norm,
            valid: true,
            num_active_sensors: num_active as u32,
        }
    }
}

impl Module for CssWlsEst {
    fn init(&mut self) {
        assert_eq!(
            self.css_data_in_msgs.len(),
            self.config.sensor_normals_body.len(),
            "cssWlsEst input count must match sensor normal count",
        );
        self.nav_state_out_msg.write(SunLineMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.nav_state_out_msg.write(self.estimate_sunline());
    }
}

fn normalize_or_zero(vector: Vector3<f64>) -> Vector3<f64> {
    if vector.norm_squared() > 0.0 {
        vector.normalize()
    } else {
        Vector3::zeros()
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{Output, SunSensorMsg};
    use crate::{Module, SimulationContext};

    use super::{CssWlsEst, CssWlsEstConfig};

    fn dummy_context() -> SimulationContext {
        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: epoch,
        }
    }

    #[test]
    fn recovers_single_sensor_boresight_direction() {
        let mut estimator = CssWlsEst::new(CssWlsEstConfig {
            name: "css".to_string(),
            sensor_normals_body: vec![
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(-1.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
            ],
            sensor_use_thresh: 0.05,
            use_weights: true,
        });
        let outputs = [
            Output::new(SunSensorMsg {
                sensed_value: 1.0,
                true_value: 1.0,
                valid: true,
            }),
            Output::new(SunSensorMsg::default()),
            Output::new(SunSensorMsg::default()),
        ];
        for (input, output) in estimator.css_data_in_msgs.iter_mut().zip(outputs.iter()) {
            input.connect(output.slot());
        }

        estimator.init();
        estimator.update(&dummy_context());

        let msg = estimator.nav_state_out_msg.read();
        assert!((msg.sun_vector_body - Vector3::new(1.0, 0.0, 0.0)).norm() < 1.0e-12);
        assert!(msg.valid);
        assert_eq!(msg.num_active_sensors, 1);
    }
}
