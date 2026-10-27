//! Orbital Hill-frame attitude reference: port of upstream Basilisk `hillPoint`.
//!
//! The reference frame R is the Hill (orbit) frame of the spacecraft relative
//! to a celestial body: `r̂1` along the relative position, `r̂3` along the orbit
//! angular momentum and `r̂2 = r̂3 × r̂1` completing the triad. The module
//! publishes the attitude of R, its inertial angular velocity (the true-anomaly
//! rate about `r̂3`) and its angular acceleration, which hold for any Keplerian
//! orbit (circular, elliptic or hyperbolic).
//!
//! The celestial-body ephemeris input is optional; without it the body sits at
//! the inertial origin with zero velocity.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    AttitudeReferenceMsg, EphemerisMsg, Input, NavigationTranslationMsg, Output,
};
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::dcm_to_mrp;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`HillPoint`]; upstream has none besides the
/// instance name.
#[derive(Clone, Debug)]
pub struct HillPointConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
}

impl Default for HillPointConfig {
    fn default() -> Self {
        Self {
            name: "hillPoint".to_string(),
        }
    }
}

/// Hill-frame pointing reference: upstream `hillPoint`.
#[derive(Clone, Debug)]
pub struct HillPoint {
    /// User-settable parameters.
    pub config: HillPointConfig,
    /// Upstream `transNavInMsg`: spacecraft inertial position and velocity.
    pub trans_nav_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `celBodyInMsg` (optional): ephemeris of the orbited body.
    pub cel_body_in_msg: Input<EphemerisMsg>,
    /// Upstream `attRefOutMsg`: Hill-frame attitude reference.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
    /// Upstream `planetMsgIsLinked`, sampled at reset.
    planet_msg_is_linked: bool,
}

impl HillPoint {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: HillPointConfig) -> Self {
        Self {
            config,
            trans_nav_in_msg: Input::default(),
            cel_body_in_msg: Input::default(),
            att_ref_out_msg: Output::default(),
            planet_msg_is_linked: false,
        }
    }

    /// Upstream `Reset_hillPoint`.
    fn reset_state(&mut self) {
        if !self.trans_nav_in_msg.is_connected() {
            log::error!("hillPoint.trans_nav_in_msg is not connected");
        }
        self.planet_msg_is_linked = self.cel_body_in_msg.is_connected();
    }
}

/// Hill-frame reference of a spacecraft at `r_bn_n_m`, `v_bn_n_mps` orbiting a
/// body at `r_body_n_m`, `v_body_n_mps` (upstream
/// `computeHillPointingReference`). All vectors are inertial components.
pub fn compute_hill_pointing_reference(
    r_bn_n_m: Vector3<f64>,
    v_bn_n_mps: Vector3<f64>,
    r_body_n_m: Vector3<f64>,
    v_body_n_mps: Vector3<f64>,
) -> AttitudeReferenceMsg {
    let rel_pos = r_bn_n_m - r_body_n_m;
    let rel_vel = v_bn_n_mps - v_body_n_mps;

    let i_r = normalize_or_zero(rel_pos);
    let h = rel_pos.cross(&rel_vel);
    let i_h = normalize_or_zero(h);
    let i_theta = i_h.cross(&i_r);
    let dcm_rn = Matrix3::from_rows(&[i_r.transpose(), i_theta.transpose(), i_h.transpose()]);

    let rm = rel_pos.norm();
    let hm = h.norm();
    // Robustness check, as upstream: its comment speaks of 1 km, but the
    // threshold is 1 in the length unit of the inputs (m).
    let (dfdt, ddfdt2) = if rm > 1.0 {
        let dfdt = hm / (rm * rm);
        (dfdt, -2.0 * rel_vel.dot(&i_r) / rm * dfdt)
    } else {
        (0.0, 0.0)
    };

    AttitudeReferenceMsg {
        sigma_rn: dcm_to_mrp(dcm_rn),
        omega_rn_n_radps: dcm_rn.tr_mul(&Vector3::new(0.0, 0.0, dfdt)),
        domega_rn_n_radps2: dcm_rn.tr_mul(&Vector3::new(0.0, 0.0, ddfdt2)),
    }
}

impl Module for HillPoint {
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
        self.att_ref_out_msg.write(compute_hill_pointing_reference(
            navigation.position_bn_n_m,
            navigation.velocity_bn_n_mps,
            planet.position_bdy_zero_n_m,
            planet.velocity_bdy_zero_n_mps,
        ));
    }
}

/// The orbit shared by the upstream `hillPoint`, `velocityPoint` and
/// `celestialTwoBodyPoint` unit tests.
#[cfg(test)]
pub(crate) mod test_orbit {
    use nalgebra::{Matrix3, Vector3};

    use crate::utilities::astro_constants::{D2R, MU_EARTH, REQ_EARTH};

    /// Upstream tests' `astroConstants.MU_EARTH * 1e9` \[m^3/s^2\].
    pub(crate) const MU_EARTH_M3PS2: f64 = MU_EARTH * 1e9;
    /// Upstream tests' `astroConstants.REQ_EARTH * 2.8 * 1000` \[m\].
    pub(crate) const SEMI_MAJOR_AXIS_M: f64 = REQ_EARTH * 2.8 * 1000.0;
    /// Upstream tests' true anomaly, `60 * D2R` \[rad\].
    pub(crate) const TRUE_ANOMALY_RAD: f64 = 60.0 * D2R;

    /// Upstream pure-Python `astroFunctions.OE2RV(mu, a, e, i, Omega, w, nu)`
    /// (perifocal state rotated by `PQW2IJK`), which the upstream tests use to
    /// build their inputs. Its rounding differs from the C `elem2rv`.
    pub(crate) fn astro_functions_oe2rv(
        mu: f64,
        a: f64,
        e: f64,
        i: f64,
        raan: f64,
        w: f64,
        nu: f64,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let p = a * (1.0 - e * e);
        let (c, s) = (nu.cos(), nu.sin());
        let r_pqw = Vector3::new(p * c / (1.0 + e * c), p * s / (1.0 + e * c), 0.0);
        let v_pqw = Vector3::new(-s * (mu / p).sqrt(), (e + c) * (mu / p).sqrt(), 0.0);
        let (so, co) = (raan.sin(), raan.cos());
        let (si, ci) = (i.sin(), i.cos());
        let (sw, cw) = (w.sin(), w.cos());
        let pqw_to_ijk = Matrix3::new(
            co * cw - so * sw * ci,
            -co * sw - so * cw * ci,
            so * si,
            so * cw + co * sw * ci,
            -so * sw + co * cw * ci,
            -co * si,
            sw * si,
            cw * si,
            ci,
        );
        (pqw_to_ijk * r_pqw, pqw_to_ijk * v_pqw)
    }

    /// Position \[m\] and velocity \[m/s\] of the upstream tests' circular
    /// equatorial orbit (`a = 2.8 REQ_EARTH`, `f = 60 deg`).
    pub(crate) fn circular_orbit_state() -> (Vector3<f64>, Vector3<f64>) {
        astro_functions_oe2rv(
            MU_EARTH_M3PS2,
            SEMI_MAJOR_AXIS_M,
            0.0,
            0.0,
            0.0,
            0.0,
            TRUE_ANOMALY_RAD,
        )
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::test_orbit::{
        MU_EARTH_M3PS2, SEMI_MAJOR_AXIS_M, astro_functions_oe2rv, circular_orbit_state,
    };
    use super::{HillPoint, HillPointConfig};
    use crate::messages::{AttitudeReferenceMsg, EphemerisMsg, NavigationTranslationMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
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

    /// `printResults_HillPoint` of upstream `_UnitTest/Support/results_hillPoint.py`.
    fn upstream_truth(
        r_bn_n: Vector3<f64>,
        v_bn_n: Vector3<f64>,
        r_body_n: Vector3<f64>,
        v_body_n: Vector3<f64>,
    ) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
        let r = r_bn_n - r_body_n;
        let v = v_bn_n - v_body_n;
        let h = r.cross(&v);
        let i_r = r / r.norm();
        let i_h = h / h.norm();
        let i_theta = i_h.cross(&i_r);
        let hn = Matrix3::from_rows(&[i_r.transpose(), i_theta.transpose(), i_h.transpose()]);
        let (hm, rm) = (h.norm(), r.norm());
        let dfdt = hm / (rm * rm);
        let ddfdt2 = -2.0 * v.dot(&i_r) / rm * dfdt;
        (dcm_to_mrp(hn), dfdt * i_h, ddfdt2 * i_h)
    }

    /// Upstream `test_hillPoint.py::test_hillPoint`, both `celMsgSet`
    /// parametrizations: a circular equatorial orbit about a body at the
    /// origin, sampled at 0, 0.5 and 1 s. Upstream's extra 0.6 s run after the
    /// checks asks for a stop time already passed and checks nothing.
    #[test]
    fn matches_upstream_hill_point_unit_test() {
        for cel_msg_set in [true, false] {
            let (r_bn_n, v_bn_n) = circular_orbit_state();
            let nav_output = Output::new(navigation(r_bn_n, v_bn_n));
            let planet_output = Output::new(EphemerisMsg::default());
            let mut module = HillPoint::new(HillPointConfig::default());
            let mut recorder = MessageRecorder::<AttitudeReferenceMsg>::default();
            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            simulation.connect(&nav_output, &mut module.trans_nav_in_msg);
            if cel_msg_set {
                simulation.connect(&planet_output, &mut module.cel_body_in_msg);
            }
            simulation.connect(&module.att_ref_out_msg, &mut recorder.input_msg);
            simulation.add_module("hillPoint", &mut module, TASK_RATE_NANOS, 0);
            simulation.add_module("recorder", &mut recorder, TASK_RATE_NANOS, 0);
            simulation.run_for(1_000_000_000);
            drop(simulation);

            let accuracy = 1e-12;
            let f_dot = (MU_EARTH_M3PS2 / (SEMI_MAJOR_AXIS_M.powi(3))).sqrt();
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
                assert_vector_close(
                    sample.domega_rn_n_radps2,
                    Vector3::new(0.0, 0.0, 1.315647475046e-23),
                    accuracy,
                );
            }
        }
    }

    // Not upstream: an inclined elliptic orbit about a moving body, against
    // the formulas of upstream's `results_hillPoint.py`.
    #[test]
    fn matches_the_upstream_support_script_on_an_elliptic_orbit_about_a_moving_body() {
        let (r_rel, v_rel) =
            astro_functions_oe2rv(MU_EARTH_M3PS2, SEMI_MAJOR_AXIS_M, 0.4, 0.3, 0.7, 1.1, 2.0);
        let r_body_n = Vector3::new(1.0e9, -2.0e9, 3.0e8);
        let v_body_n = Vector3::new(1.0e3, 2.0e4, -3.0e3);
        let r_bn_n = r_rel + r_body_n;
        let v_bn_n = v_rel + v_body_n;
        let nav_output = Output::new(navigation(r_bn_n, v_bn_n));
        let planet_output = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: r_body_n,
            velocity_bdy_zero_n_mps: v_body_n,
            ..Default::default()
        });
        let mut module = HillPoint::new(HillPointConfig::default());
        nav_output.connect_to(&mut module.trans_nav_in_msg);
        planet_output.connect_to(&mut module.cel_body_in_msg);
        module.init();
        module.update(&context(0));

        let (sigma, omega, domega) = upstream_truth(r_bn_n, v_bn_n, r_body_n, v_body_n);
        let output = module.att_ref_out_msg.read();
        assert_vector_close(output.sigma_rn, sigma, 1e-12);
        assert_vector_close(output.omega_rn_n_radps, omega, 1e-12 * omega.norm());
        assert_vector_close(output.domega_rn_n_radps2, domega, 1e-12 * domega.norm());
        assert!(domega.norm() > 1e-9, "the orbit is not circular");
    }

    // Not upstream: upstream samples whether the body message is linked at
    // reset, so a body connected later is ignored until the next reset.
    #[test]
    fn samples_the_celestial_body_link_at_reset() {
        let (r_rel, v_rel) = circular_orbit_state();
        let r_body_n = Vector3::new(4.0e8, 0.0, 0.0);
        let nav_output = Output::new(navigation(r_rel + r_body_n, v_rel));
        let planet_output = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: r_body_n,
            ..Default::default()
        });
        let mut module = HillPoint::new(HillPointConfig::default());
        nav_output.connect_to(&mut module.trans_nav_in_msg);
        module.init();
        planet_output.connect_to(&mut module.cel_body_in_msg);

        module.update(&context(0));
        let (unlinked, _, _) =
            upstream_truth(r_rel + r_body_n, v_rel, Vector3::zeros(), Vector3::zeros());
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
        let mut module = HillPoint::new(HillPointConfig::default());
        nav_output.connect_to(&mut module.trans_nav_in_msg);
        module.init();
        module.update(&context(0));

        let output = module.att_ref_out_msg.read();
        assert_eq!(output.sigma_rn, Vector3::zeros());
        assert_eq!(output.omega_rn_n_radps, Vector3::zeros());
        assert_eq!(output.domega_rn_n_radps2, Vector3::zeros());
    }
}
