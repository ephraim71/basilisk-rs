use nalgebra::Vector3;

use crate::messages::{AttitudeGuidanceMsg, ImuMsg, Input, Output, SunLineMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct SunSafePointConfig {
    pub name: String,
    pub s_hat_bdy_cmd: Vector3<f64>,
    pub min_unit_mag: f64,
    pub small_angle: f64,
    pub sun_axis_spin_rate_radps: f64,
    pub omega_rn_b_search_radps: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct SunSafePoint {
    pub config: SunSafePointConfig,
    pub sun_direction_in_msg: Input<SunLineMsg>,
    pub imu_in_msg: Input<ImuMsg>,
    pub att_guidance_out_msg: Output<AttitudeGuidanceMsg>,
    e_hat_180_b: Vector3<f64>,
}

impl SunSafePoint {
    pub fn new(config: SunSafePointConfig) -> Self {
        let mut e_hat_180_b = config.s_hat_bdy_cmd.cross(&Vector3::new(1.0, 0.0, 0.0));
        if e_hat_180_b.norm() < 0.1 {
            e_hat_180_b = config.s_hat_bdy_cmd.cross(&Vector3::new(0.0, 1.0, 0.0));
        }
        Self {
            e_hat_180_b: normalize_or_zero(e_hat_180_b),
            config,
            sun_direction_in_msg: Input::default(),
            imu_in_msg: Input::default(),
            att_guidance_out_msg: Output::default(),
        }
    }

    fn compute_guidance(&self) -> AttitudeGuidanceMsg {
        let sun_direction = self.sun_direction_in_msg.read();
        let imu = self.imu_in_msg.read();
        let omega_bn_b = imu.angular_rate_sensor_radps;
        let s_norm = sun_direction.sun_vector_body.norm();

        if sun_direction.valid && s_norm > self.config.min_unit_mag {
            let s_hat_bdy_cmd = normalize_or_zero(self.config.s_hat_bdy_cmd);
            let mut cosine = s_hat_bdy_cmd.dot(&sun_direction.sun_vector_body) / s_norm;
            cosine = cosine.clamp(-1.0, 1.0);
            let sun_angle_err = cosine.acos();

            let sigma_br = if sun_angle_err < self.config.small_angle {
                Vector3::zeros()
            } else {
                let e_hat = if std::f64::consts::PI - sun_angle_err < self.config.small_angle {
                    self.e_hat_180_b
                } else {
                    normalize_or_zero(s_hat_bdy_cmd.cross(&sun_direction.sun_vector_body))
                };
                let sigma = e_hat * (0.25 * sun_angle_err).tan();
                mrp_switch(sigma)
            };

            let omega_rn_b =
                self.config.sun_axis_spin_rate_radps / s_norm * sun_direction.sun_vector_body;

            AttitudeGuidanceMsg {
                sigma_br,
                omega_br_b_radps: omega_bn_b - omega_rn_b,
                omega_rn_b_radps: omega_rn_b,
                domega_rn_b_radps2: Vector3::zeros(),
            }
        } else {
            AttitudeGuidanceMsg {
                sigma_br: Vector3::zeros(),
                omega_br_b_radps: omega_bn_b - self.config.omega_rn_b_search_radps,
                omega_rn_b_radps: self.config.omega_rn_b_search_radps,
                domega_rn_b_radps2: Vector3::zeros(),
            }
        }
    }
}

impl Module for SunSafePoint {
    fn init(&mut self) {
        self.att_guidance_out_msg
            .write(AttitudeGuidanceMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.att_guidance_out_msg.write(self.compute_guidance());
    }
}

fn normalize_or_zero(vector: Vector3<f64>) -> Vector3<f64> {
    if vector.norm_squared() > 0.0 {
        vector.normalize()
    } else {
        Vector3::zeros()
    }
}

fn mrp_switch(sigma: Vector3<f64>) -> Vector3<f64> {
    let norm_sq = sigma.norm_squared();
    if norm_sq > 1.0 {
        -sigma / norm_sq
    } else {
        sigma
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{ImuMsg, Output, SunLineMsg};
    use crate::{Module, SimulationContext};

    use super::{SunSafePoint, SunSafePointConfig};

    fn dummy_context() -> SimulationContext {
        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: epoch,
        }
    }

    #[test]
    fn aligned_sunline_yields_zero_attitude_error() {
        let mut module = SunSafePoint::new(SunSafePointConfig {
            name: "sunSafePoint".to_string(),
            s_hat_bdy_cmd: Vector3::new(0.0, 0.0, 1.0),
            min_unit_mag: 0.1,
            small_angle: 1.0e-12,
            sun_axis_spin_rate_radps: 0.0,
            omega_rn_b_search_radps: Vector3::zeros(),
        });
        let sun_out = Output::new(SunLineMsg {
            sun_vector_body: Vector3::new(0.0, 0.0, 1.0),
            valid: true,
            num_active_sensors: 1,
        });
        let imu_out = Output::new(ImuMsg {
            angular_rate_sensor_radps: Vector3::zeros(),
        });
        module.sun_direction_in_msg.connect(sun_out.slot());
        module.imu_in_msg.connect(imu_out.slot());

        module.init();
        module.update(&dummy_context());

        let msg = module.att_guidance_out_msg.read();
        assert!(msg.sigma_br.norm() < 1.0e-12);
        assert!(msg.omega_br_b_radps.norm() < 1.0e-12);
    }
}
