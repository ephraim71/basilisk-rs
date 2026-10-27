//! Orbital velocity-frame attitude reference: port of upstream Basilisk
//! `velocityPoint`.
//!
//! The reference frame R is the velocity frame of the spacecraft's orbit about
//! a celestial body: `r̂2` along the relative velocity, `r̂3` along the orbit
//! angular momentum and `r̂1 = r̂2 × r̂3`. The module publishes its attitude,
//! and its inertial angular velocity and acceleration about `r̂3`, the
//! true-anomaly rate minus the flight-path-angle rate, which the module takes
//! from the osculating elements and which holds for any Keplerian orbit.
//!
//! The celestial-body ephemeris input is optional; without it the body sits at
//! the inertial origin with zero velocity.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    AttitudeReferenceMsg, EphemerisMsg, Input, NavigationTranslationMsg, Output,
};
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::orbital_motion::state_to_elements;
use crate::utilities::rigid_body_kinematics::dcm_to_mrp;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`VelocityPoint`]. `Default` is upstream's
/// zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct VelocityPointConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `mu` \[m^3/s^2\]: gravitational parameter of the orbited body,
    /// in the length unit of the navigation input. Upstream's default is zero,
    /// so it must be set.
    pub mu_m3ps2: f64,
}

impl Default for VelocityPointConfig {
    fn default() -> Self {
        Self {
            name: "velocityPoint".to_string(),
            mu_m3ps2: 0.0,
        }
    }
}

/// Velocity-frame pointing reference: upstream `velocityPoint`.
#[derive(Clone, Debug)]
pub struct VelocityPoint {
    /// User-settable parameters.
    pub config: VelocityPointConfig,
    /// Upstream `transNavInMsg`: spacecraft inertial position and velocity.
    pub trans_nav_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `celBodyInMsg` (optional): ephemeris of the orbited body.
    pub cel_body_in_msg: Input<EphemerisMsg>,
    /// Upstream `attRefOutMsg`: velocity-frame attitude reference.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
    /// Upstream `planetMsgIsLinked`, sampled at reset.
    planet_msg_is_linked: bool,
}

impl VelocityPoint {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: VelocityPointConfig) -> Self {
        Self {
            config,
            trans_nav_in_msg: Input::default(),
            cel_body_in_msg: Input::default(),
            att_ref_out_msg: Output::default(),
            planet_msg_is_linked: false,
        }
    }

    /// Upstream `Reset_velocityPoint`.
    fn reset_state(&mut self) {
        if !self.trans_nav_in_msg.is_connected() {
            log::error!("velocityPoint.trans_nav_in_msg is not connected");
        }
        self.planet_msg_is_linked = self.cel_body_in_msg.is_connected();
    }

    /// Velocity-frame reference of a spacecraft at `r_bn_n_m`, `v_bn_n_mps`
    /// orbiting a body at `r_body_n_m`, `v_body_n_mps` (upstream
    /// `computeVelocityPointingReference`). All vectors are inertial
    /// components.
    pub fn compute_velocity_pointing_reference(
        &self,
        r_bn_n_m: Vector3<f64>,
        v_bn_n_mps: Vector3<f64>,
        r_body_n_m: Vector3<f64>,
        v_body_n_mps: Vector3<f64>,
    ) -> AttitudeReferenceMsg {
        let r = r_bn_n_m - r_body_n_m;
        let v = v_bn_n_mps - v_body_n_mps;

        let i_v = normalize_or_zero(v);
        let h = r.cross(&v);
        let i_h = normalize_or_zero(h);
        let i_n = i_v.cross(&i_h);
        let dcm_rn = Matrix3::from_rows(&[i_n.transpose(), i_v.transpose(), i_h.transpose()]);

        let rm = r.norm();
        let hm = h.norm();
        // Robustness check, as upstream: within 1 length unit (m) of the body
        // the rates stay zero.
        let (omega_rn_r, domega_rn_r) = if rm > 1.0 {
            let oe = state_to_elements(self.config.mu_m3ps2, r, v);
            let (e, f) = (oe.eccentricity, oe.true_anomaly_rad);
            let dfdt = hm / (rm * rm);
            let ddfdt2 = -2.0 * (v.dot(&r) / (rm * rm)) * dfdt;
            let denom = 1.0 + e * e + 2.0 * e * f.cos();
            let temp = (1.0 + e * f.cos()) / denom;
            (
                dfdt * temp,
                ddfdt2 * temp - dfdt * dfdt * e * (e * e - 1.0) * f.sin() / (denom * denom),
            )
        } else {
            (0.0, 0.0)
        };

        AttitudeReferenceMsg {
            sigma_rn: dcm_to_mrp(dcm_rn),
            omega_rn_n_radps: dcm_rn.tr_mul(&Vector3::new(0.0, 0.0, omega_rn_r)),
            domega_rn_n_radps2: dcm_rn.tr_mul(&Vector3::new(0.0, 0.0, domega_rn_r)),
        }
    }
}

impl Module for VelocityPoint {
    fn init(&mut self) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, _context: &SimulationContext) {
        // An unlinked body is the zeroed ephemeris, as upstream.
        let planet = if self.planet_msg_is_linked {
            self.cel_body_in_msg.read()
        } else {
            EphemerisMsg::default()
        };
        let navigation = self.trans_nav_in_msg.read();
        let reference = self.compute_velocity_pointing_reference(
            navigation.position_bn_n_m,
            navigation.velocity_bn_n_mps,
            planet.position_bdy_zero_n_m,
            planet.velocity_bdy_zero_n_mps,
        );
        self.att_ref_out_msg.write(reference);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{VelocityPoint, VelocityPointConfig};
    use crate::fsw_algorithms::att_guidance::hill_point::test_orbit::{
        MU_EARTH_M3PS2, SEMI_MAJOR_AXIS_M, TRUE_ANOMALY_RAD, astro_functions_oe2rv,
        circular_orbit_state,
    };
    use crate::messages::{AttitudeReferenceMsg, EphemerisMsg, NavigationTranslationMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::orbital_motion::state_to_elements;
    use crate::utilities::rigid_body_kinematics::dcm_to_mrp;
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(0.5)`.
    const TASK_RATE_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `unitTestSupport.isArrayEqual`: absolute, per component.
    fn assert_vector_close(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64) {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!(
                (a - e).abs() <= accuracy,
                "{actual:?} != {expected:?} within {accuracy}"
            );
        }
    }

    fn navigation(r_bn_n: Vector3<f64>, v_bn_n: Vector3<f64>) -> NavigationTranslationMsg {
        NavigationTranslationMsg {
            position_bn_n_m: r_bn_n,
            velocity_bn_n_mps: v_bn_n,
            ..Default::default()
        }
    }

    fn module_with_mu() -> VelocityPoint {
        VelocityPoint::new(VelocityPointConfig {
            mu_m3ps2: MU_EARTH_M3PS2,
            ..Default::default()
        })
    }

    /// `printResults_VelocityPoint` of upstream
    /// `_UnitTest/Support/results_velocityPoint.py`; its `af.RV2OE` is
    /// replaced by `state_to_elements`.
    fn upstream_truth(
        r_bn_n: Vector3<f64>,
        v_bn_n: Vector3<f64>,
        r_body_n: Vector3<f64>,
        v_body_n: Vector3<f64>,
        mu: f64,
    ) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
        let r = r_bn_n - r_body_n;
        let v = v_bn_n - v_body_n;
        let h = r.cross(&v);
        let i_r = r / r.norm();
        let i_v = v / v.norm();
        let i_h = h / h.norm();
        let i_n = i_v.cross(&i_h);
        let vn = Matrix3::from_rows(&[i_n.transpose(), i_v.transpose(), i_h.transpose()]);
        let (hm, rm) = (h.norm(), r.norm());
        let dfdt = hm / (rm * rm);
        let ddfdt2 = -2.0 * v.dot(&i_r) / rm * dfdt;

        let oe = state_to_elements(mu, r, v);
        let (e, f) = (oe.eccentricity, oe.true_anomaly_rad);
        let den = 1.0 + e * e + 2.0 * e * f.cos();
        let temp = e * (e + f.cos()) / den;
        let d_beta_dt = temp * dfdt;
        let dd_beta_dt2 = temp * ddfdt2 + (e * (e * e - 1.0) * f.sin()) / (den * den) * dfdt * dfdt;
        (
            dcm_to_mrp(vn),
            (-d_beta_dt + dfdt) * i_h,
            (-dd_beta_dt2 + ddfdt2) * i_h,
        )
    }

    /// Upstream `test_velocityPoint.py::test_velocityPoint`: a circular
    /// equatorial orbit about a body at the origin, sampled at 0, 0.5 and
    /// 1 s. Upstream's extra 0.6 s run after the checks asks for a stop time
    /// already passed and checks nothing.
    #[test]
    fn matches_upstream_velocity_point_unit_test() {
        let (r_bn_n, v_bn_n) = circular_orbit_state();
        let nav_output = Output::new(navigation(r_bn_n, v_bn_n));
        let planet_output = Output::new(EphemerisMsg::default());
        let mut module = module_with_mu();
        let mut recorder = MessageRecorder::<AttitudeReferenceMsg>::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&nav_output, &mut module.trans_nav_in_msg);
        simulation.connect(&planet_output, &mut module.cel_body_in_msg);
        simulation.connect(&module.att_ref_out_msg, &mut recorder.input_msg);
        simulation.add_module("velocityPoint", &mut module, TASK_RATE_NANOS, 0);
        simulation.add_module("recorder", &mut recorder, TASK_RATE_NANOS, 0);
        simulation.run_for(1_000_000_000);
        drop(simulation);

        let accuracy = 1e-12;
        let f_dot = (MU_EARTH_M3PS2 / (SEMI_MAJOR_AXIS_M.powi(3))).sqrt();
        // `unitTestSupport.compareArray` also checks the sample count.
        assert_eq!(recorder.samples.len(), 3);
        for sample in &recorder.samples {
            assert_vector_close(
                sample.sigma_rn,
                Vector3::new(0.0, 0.0, 0.267949192431),
                accuracy,
            );
            assert_vector_close(
                sample.omega_rn_n_radps,
                Vector3::new(0.0, 0.0, f_dot),
                accuracy,
            );
            assert_vector_close(sample.domega_rn_n_radps2, Vector3::zeros(), accuracy);
        }
    }

    // Not upstream: the `e = 0.8` orbit of upstream's
    // `results_velocityPoint.py`, here inclined and about a moving body, so
    // the flight-path-angle terms are exercised.
    #[test]
    fn matches_the_upstream_support_script_on_an_eccentric_orbit() {
        let (r_rel, v_rel) = astro_functions_oe2rv(
            MU_EARTH_M3PS2,
            SEMI_MAJOR_AXIS_M,
            0.8,
            0.2,
            0.5,
            0.9,
            TRUE_ANOMALY_RAD,
        );
        let r_body_n = Vector3::new(-5.0e8, 2.0e9, 1.0e7);
        let v_body_n = Vector3::new(3.0e4, -1.0e3, 2.0e2);
        let r_bn_n = r_rel + r_body_n;
        let v_bn_n = v_rel + v_body_n;
        let nav_output = Output::new(navigation(r_bn_n, v_bn_n));
        let planet_output = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: r_body_n,
            velocity_bdy_zero_n_mps: v_body_n,
            ..Default::default()
        });
        let mut module = module_with_mu();
        nav_output.connect_to(&mut module.trans_nav_in_msg);
        planet_output.connect_to(&mut module.cel_body_in_msg);
        module.init();
        module.update(&context(0));

        let (sigma, omega, domega) =
            upstream_truth(r_bn_n, v_bn_n, r_body_n, v_body_n, MU_EARTH_M3PS2);
        let output = module.att_ref_out_msg.read();
        assert_vector_close(output.sigma_rn, sigma, 1e-12);
        assert_vector_close(output.omega_rn_n_radps, omega, 1e-10 * omega.norm());
        assert_vector_close(output.domega_rn_n_radps2, domega, 1e-10 * domega.norm());
        assert!(domega.norm() > 1e-9, "the flight-path angle changes");
    }

    // Not upstream: `mu` set on the config after construction takes effect,
    // as upstream reads `configData->mu` at every update.
    #[test]
    fn reads_mu_from_the_config_at_every_update() {
        let (r_rel, v_rel) =
            astro_functions_oe2rv(MU_EARTH_M3PS2, SEMI_MAJOR_AXIS_M, 0.5, 0.0, 0.0, 0.0, 1.0);
        let nav_output = Output::new(navigation(r_rel, v_rel));
        let mut module = VelocityPoint::new(VelocityPointConfig::default());
        nav_output.connect_to(&mut module.trans_nav_in_msg);
        module.config.mu_m3ps2 = MU_EARTH_M3PS2;
        module.init();
        module.update(&context(0));

        let (_, omega, _) = upstream_truth(
            r_rel,
            v_rel,
            Vector3::zeros(),
            Vector3::zeros(),
            MU_EARTH_M3PS2,
        );
        let output = module.att_ref_out_msg.read();
        assert_vector_close(output.omega_rn_n_radps, omega, 1e-10 * omega.norm());
    }

    // Not upstream: upstream samples whether the body message is linked at
    // reset, so a body connected later is ignored until the next reset.
    #[test]
    fn samples_the_celestial_body_link_at_reset() {
        let (r_rel, v_rel) = circular_orbit_state();
        let r_body_n = Vector3::new(0.0, -6.0e8, 0.0);
        let nav_output = Output::new(navigation(r_rel + r_body_n, v_rel));
        let planet_output = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: r_body_n,
            ..Default::default()
        });
        let mut module = module_with_mu();
        nav_output.connect_to(&mut module.trans_nav_in_msg);
        module.init();
        planet_output.connect_to(&mut module.cel_body_in_msg);

        module.update(&context(0));
        let (unlinked, _, _) = upstream_truth(
            r_rel + r_body_n,
            v_rel,
            Vector3::zeros(),
            Vector3::zeros(),
            MU_EARTH_M3PS2,
        );
        assert_vector_close(module.att_ref_out_msg.read().sigma_rn, unlinked, 1e-12);

        Module::reset(&mut module, &context(TASK_RATE_NANOS));
        module.update(&context(TASK_RATE_NANOS));
        assert_vector_close(
            module.att_ref_out_msg.read().sigma_rn,
            Vector3::new(0.0, 0.0, 0.267949192431),
            1e-12,
        );
    }

    // Not upstream: within 1 m of the body the rates are zeroed, as upstream's
    // robustness check does.
    #[test]
    fn zeroes_the_rates_within_one_metre_of_the_body() {
        let nav_output = Output::new(navigation(
            Vector3::new(0.5, 0.0, 0.0),
            Vector3::new(0.0, 7.0e3, 0.0),
        ));
        let mut module = module_with_mu();
        nav_output.connect_to(&mut module.trans_nav_in_msg);
        module.init();
        module.update(&context(0));

        let output = module.att_ref_out_msg.read();
        assert_eq!(output.sigma_rn, Vector3::zeros());
        assert_eq!(output.omega_rn_n_radps, Vector3::zeros());
        assert_eq!(output.domega_rn_n_radps2, Vector3::zeros());
    }
}
