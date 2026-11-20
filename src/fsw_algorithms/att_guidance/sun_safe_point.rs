use nalgebra::Vector3;

use crate::messages::{AttitudeGuidanceMsg, Input, NavigationAttitudeMsg, Output, SunLineMsg};
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
    /// Upstream `imuInMsg`, a `NavAttMsg`: the body rate `omega_BN_B`.
    pub imu_in_msg: Input<NavigationAttitudeMsg>,
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
        let omega_bn_b = self.imu_in_msg.read().omega_bn_b_radps;
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
                    // Upstream `v3Cross(vehSunPntBdy, sHatBdyCmd, e_hat)`.
                    normalize_or_zero(sun_direction.sun_vector_body.cross(&s_hat_bdy_cmd))
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
    //! Upstream `_UnitTest/test_sunSafePoint.py`, its seven cases, with its
    //! `1e-12` accuracy. Upstream's Sun direction is a `NavAttMsg`; here a
    //! [`SunLineMsg`], valid with the same vector.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{NavigationAttitudeMsg, Output, SunLineMsg};
    use crate::{Module, SimulationContext};

    use super::{SunSafePoint, SunSafePointConfig};

    const ACCURACY: f64 = 1e-12;

    fn context(nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `sunSafePointTestFunction(case)`: the module's
    /// `(sigma_BR, omega_BR_B, omega_RN_B)` at 0, 0.5 and 1 s, and the truth.
    fn run_case(case: u32) {
        let s_hat_cmd_b = if case == 5 {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            Vector3::new(0.0, 0.0, 1.0)
        };
        let min_unit_mag = 0.1;
        let omega_rn_b_search_config = if case == 2 {
            Vector3::new(0.0, 0.0, 0.1)
        } else {
            Vector3::zeros()
        };
        let sun_axis_spin_rate = if case == 7 { 1.5_f64.to_radians() } else { 0.0 };
        let mut module = SunSafePoint::new(SunSafePointConfig {
            name: "sunSafePoint".to_string(),
            s_hat_bdy_cmd: s_hat_cmd_b,
            min_unit_mag,
            small_angle: 0.01_f64.to_radians(),
            sun_axis_spin_rate_radps: sun_axis_spin_rate,
            omega_rn_b_search_radps: omega_rn_b_search_config,
        });

        let sun_vec_b = match case {
            2 | 6 => Vector3::new(0.0, min_unit_mag / 2.0, 0.0),
            3 => s_hat_cmd_b,
            4 | 5 => -s_hat_cmd_b,
            _ => Vector3::new(1.0, 1.0, 0.0),
        };
        let sun_out = Output::new(SunLineMsg {
            sun_vector_body: sun_vec_b,
            valid: true,
            num_active_sensors: 3,
        });
        let omega_bn_b = Vector3::new(0.01, 0.50, -0.2);
        let imu_out = Output::new(NavigationAttitudeMsg {
            omega_bn_b_radps: omega_bn_b,
            ..NavigationAttitudeMsg::default()
        });
        module.sun_direction_in_msg.connect(sun_out.slot());
        module.imu_in_msg.connect(imu_out.slot());

        let phi = (sun_vec_b.normalize().dot(&s_hat_cmd_b)).acos();
        let sigma_true = match case {
            1 | 7 => sun_vec_b.cross(&s_hat_cmd_b).normalize() * (phi / 4.0).tan(),
            4 => s_hat_cmd_b.cross(&Vector3::x()).normalize() * (phi / 4.0).tan(),
            5 => s_hat_cmd_b.cross(&Vector3::y()).normalize() * (phi / 4.0).tan(),
            _ => Vector3::zeros(),
        };
        let omega_rn_b_search = match case {
            2 => omega_rn_b_search_config,
            7 => sun_vec_b.normalize() * sun_axis_spin_rate,
            _ => Vector3::zeros(),
        };

        module.init();
        for step in 0..3 {
            module.update(&context(step * 500_000_000));
            let output = module.att_guidance_out_msg.read();
            assert!(
                (output.sigma_br - sigma_true).amax() < ACCURACY,
                "case {case}: sigma_BR {:?}, want {:?}",
                output.sigma_br,
                sigma_true
            );
            assert!(
                (output.omega_br_b_radps - (omega_bn_b - omega_rn_b_search)).amax() < ACCURACY,
                "case {case}: omega_BR_B {:?}",
                output.omega_br_b_radps
            );
            assert!(
                (output.omega_rn_b_radps - omega_rn_b_search).amax() < ACCURACY,
                "case {case}: omega_RN_B {:?}",
                output.omega_rn_b_radps
            );
        }
    }

    /// Case 1: Sun visible, vectors not aligned.
    #[test]
    fn sun_visible_and_not_aligned() {
        run_case(1);
    }

    /// Case 2: Sun not visible, with a search rate.
    #[test]
    fn sun_not_visible_with_a_search_rate() {
        run_case(2);
    }

    /// Case 3: Sun visible, vectors aligned.
    #[test]
    fn sun_visible_and_aligned() {
        run_case(3);
    }

    /// Case 4: Sun visible, vectors opposite.
    #[test]
    fn sun_visible_and_opposite() {
        run_case(4);
    }

    /// Case 5: Sun visible, vectors opposite, commanded axis `b1`.
    #[test]
    fn sun_visible_and_opposite_about_b1() {
        run_case(5);
    }

    /// Case 6: Sun not visible, no search rate.
    #[test]
    fn sun_not_visible_without_a_search_rate() {
        run_case(6);
    }

    /// Case 7: Sun visible, spinning about the Sun heading.
    #[test]
    fn sun_visible_with_a_spin_about_the_sun_heading() {
        run_case(7);
    }
}
