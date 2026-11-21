//! Two-body celestial pointing reference: port of upstream Basilisk
//! `celestialTwoBodyPoint`.
//!
//! The reference frame R points its first axis `r̂1` at a primary celestial
//! body and its third axis `r̂3` along the normal of the plane spanned by the
//! primary and a secondary direction, so that `r̂2 = r̂3 × r̂1` lies as close as
//! possible to the secondary direction. For example, a sensor can be aimed at
//! a planet while a solar panel normal stays as close as possible to the Sun.
//! The module publishes the attitude of R and its inertial angular velocity and
//! acceleration, computed from the analytic first and second time derivatives
//! of the basis vectors.
//!
//! The secondary body is optional. Without it, or when its direction is within
//! `singularity_thresh_rad` of parallel or antiparallel to the primary one, the
//! secondary direction is the primary relative position crossed with the
//! primary relative velocity. The accelerations of both bodies relative to the
//! spacecraft are taken as zero, since the inputs carry none.

use std::f64::consts::PI;

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    AttitudeReferenceMsg, EphemerisMsg, Input, NavigationTranslationMsg, Output,
};
use crate::utilities::linear_algebra::{normalize_or_zero, safe_acos};
use crate::utilities::rigid_body_kinematics::dcm_to_mrp;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`CelestialTwoBodyPoint`]. `Default` is
/// upstream's zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct CelestialTwoBodyPointConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `singularityThresh` \[rad\]: angle from parallel or
    /// antiparallel below which the secondary direction is replaced by the
    /// primary orbit normal.
    pub singularity_thresh_rad: f64,
}

impl Default for CelestialTwoBodyPointConfig {
    fn default() -> Self {
        Self {
            name: "celestialTwoBodyPoint".to_string(),
            singularity_thresh_rad: 0.0,
        }
    }
}

/// Primary/secondary pointing reference: upstream `celestialTwoBodyPoint`.
#[derive(Clone, Debug)]
pub struct CelestialTwoBodyPoint {
    /// User-settable parameters.
    pub config: CelestialTwoBodyPointConfig,
    /// Upstream `transNavInMsg`: spacecraft inertial position and velocity.
    pub trans_nav_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `celBodyInMsg`: ephemeris of the primary body.
    pub cel_body_in_msg: Input<EphemerisMsg>,
    /// Upstream `secCelBodyInMsg` (optional): ephemeris of the secondary body.
    pub sec_cel_body_in_msg: Input<EphemerisMsg>,
    /// Upstream `attRefOutMsg`: two-body pointing reference.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
    /// Upstream `secCelBodyIsLinked`, sampled at reset.
    sec_cel_body_is_linked: bool,
}

/// States of the two pointing directions relative to the spacecraft, inertial
/// components (upstream `R_P1B_N`, `v_P1B_N`, `a_P1B_N`, `R_P2B_N`, ...).
#[derive(Clone, Copy, Debug)]
struct RelativeStates {
    r_p1b_n: Vector3<f64>,
    v_p1b_n: Vector3<f64>,
    a_p1b_n: Vector3<f64>,
    r_p2b_n: Vector3<f64>,
    v_p2b_n: Vector3<f64>,
    a_p2b_n: Vector3<f64>,
}

impl CelestialTwoBodyPoint {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: CelestialTwoBodyPointConfig) -> Self {
        Self {
            config,
            trans_nav_in_msg: Input::default(),
            cel_body_in_msg: Input::default(),
            sec_cel_body_in_msg: Input::default(),
            att_ref_out_msg: Output::default(),
            sec_cel_body_is_linked: false,
        }
    }

    /// Upstream `Reset_celestialTwoBodyPoint`.
    fn reset_state(&mut self) {
        self.sec_cel_body_is_linked = self.sec_cel_body_in_msg.is_connected();
        if !self.trans_nav_in_msg.is_connected() {
            log::error!("celestialTwoBodyPoint.trans_nav_in_msg is not connected");
        }
        if !self.cel_body_in_msg.is_connected() {
            log::error!("celestialTwoBodyPoint.cel_body_in_msg is not connected");
        }
    }

    /// Upstream `parseInputMessages`: the relative states of both pointing
    /// directions, with the orbit-normal fallback for the secondary one.
    fn relative_states(&self) -> RelativeStates {
        let navigation = self.trans_nav_in_msg.read();
        let primary = self.cel_body_in_msg.read();
        let r_p1b_n = primary.position_bdy_zero_n_m - navigation.position_bn_n_m;
        let v_p1b_n = primary.velocity_bdy_zero_n_mps - navigation.velocity_bn_n_mps;
        let a_p1b_n = Vector3::zeros();

        let secondary = self.sec_cel_body_is_linked.then(|| {
            let secondary = self.sec_cel_body_in_msg.read();
            let r_p2b_n = secondary.position_bdy_zero_n_m - navigation.position_bn_n_m;
            let v_p2b_n = secondary.velocity_bdy_zero_n_mps - navigation.velocity_bn_n_mps;
            let cos_angle = normalize_or_zero(r_p2b_n).dot(&normalize_or_zero(r_p1b_n));
            (r_p2b_n, v_p2b_n, safe_acos(cos_angle))
        });
        let (r_p2b_n, v_p2b_n, a_p2b_n) = match secondary {
            Some((r_p2b_n, v_p2b_n, plat_ang_diff))
                if !is_singular(plat_ang_diff, self.config.singularity_thresh_rad) =>
            {
                (r_p2b_n, v_p2b_n, Vector3::zeros())
            }
            // Absent or (anti)parallel secondary: the primary orbit normal and
            // its derivatives.
            _ => (
                r_p1b_n.cross(&v_p1b_n),
                r_p1b_n.cross(&a_p1b_n),
                v_p1b_n.cross(&a_p1b_n),
            ),
        };
        RelativeStates {
            r_p1b_n,
            v_p1b_n,
            a_p1b_n,
            r_p2b_n,
            v_p2b_n,
            a_p2b_n,
        }
    }
}

/// Upstream's test for a secondary direction too close to parallel or
/// antiparallel to the primary one (a NaN angle is not singular, as upstream).
fn is_singular(plat_ang_diff_rad: f64, threshold_rad: f64) -> bool {
    plat_ang_diff_rad.abs() < threshold_rad || plat_ang_diff_rad.abs() > PI - threshold_rad
}

/// First and second inertial derivatives of the unit vector `x_hat = x/|x|`,
/// for `x` with derivatives `dx`, `ddx` (upstream Eqs. 11a/b and 13a/b), via
/// the projection `[I] - x̂ x̂ᵀ` orthogonal to `x̂`.
fn unit_vector_derivatives(
    x: Vector3<f64>,
    dx: Vector3<f64>,
    ddx: Vector3<f64>,
    x_hat: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let projection = Matrix3::identity() - x_hat * x_hat.transpose();
    let inverse_norm = 1.0 / x.norm();
    let dx_hat = inverse_norm * (projection * dx);
    let coupling = 2.0 * (dx_hat * x_hat.transpose()) + x_hat * dx_hat.transpose();
    let ddx_hat = inverse_norm * (projection * ddx - coupling * dx);
    (dx_hat, ddx_hat)
}

/// Upstream `computeCelestialTwoBodyPoint`: the reference frame and its
/// angular velocity and acceleration from the relative states.
fn two_body_reference(states: &RelativeStates) -> AttitudeReferenceMsg {
    let RelativeStates {
        r_p1b_n,
        v_p1b_n,
        a_p1b_n,
        r_p2b_n,
        v_p2b_n,
        a_p2b_n,
    } = *states;

    // Plane normal and its first two inertial derivatives (Eqs. 4 and 5).
    let r_n = r_p1b_n.cross(&r_p2b_n);
    let v_n = v_p1b_n.cross(&r_p2b_n) + r_p1b_n.cross(&v_p2b_n);
    let a_n = 2.0 * v_p1b_n.cross(&v_p2b_n) + (a_p1b_n.cross(&r_p2b_n) + r_p1b_n.cross(&a_p2b_n));

    // Reference basis (Eqs. 9a-c); the rows of [RN] are the basis vectors.
    let r1_hat = normalize_or_zero(r_p1b_n);
    let r3_hat = normalize_or_zero(r_n);
    let r2_hat = normalize_or_zero(r3_hat.cross(&r1_hat));
    let dcm_rn = Matrix3::from_rows(&[r1_hat.transpose(), r2_hat.transpose(), r3_hat.transpose()]);

    let (dr1_hat, ddr1_hat) = unit_vector_derivatives(r_p1b_n, v_p1b_n, a_p1b_n, r1_hat);
    let (dr3_hat, ddr3_hat) = unit_vector_derivatives(r_n, v_n, a_n, r3_hat);
    let dr2_hat = dr3_hat.cross(&r1_hat) + r3_hat.cross(&dr1_hat);
    let ddr2_hat =
        2.0 * dr3_hat.cross(&dr1_hat) + (ddr3_hat.cross(&r1_hat) + r3_hat.cross(&ddr1_hat));

    let omega_rn_r = Vector3::new(
        r3_hat.dot(&dr2_hat),
        r1_hat.dot(&dr3_hat),
        r2_hat.dot(&dr1_hat),
    );
    // The omega · dr̂_i terms of differentiating omega_i vanish because
    // dr̂_i = omega × r̂_i.
    let domega_rn_r = Vector3::new(
        dr3_hat.dot(&dr2_hat) + r3_hat.dot(&ddr2_hat),
        dr1_hat.dot(&dr3_hat) + r1_hat.dot(&ddr3_hat),
        dr2_hat.dot(&dr1_hat) + r2_hat.dot(&ddr1_hat),
    );

    AttitudeReferenceMsg {
        sigma_rn: dcm_to_mrp(dcm_rn),
        omega_rn_n_radps: dcm_rn.tr_mul(&omega_rn_r),
        domega_rn_n_radps2: dcm_rn.tr_mul(&domega_rn_r),
    }
}

impl Module for CelestialTwoBodyPoint {
    fn init(&mut self) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let reference = two_body_reference(&self.relative_states());
        self.att_ref_out_msg.write(reference);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{CelestialTwoBodyPoint, CelestialTwoBodyPointConfig};
    use crate::fsw_algorithms::att_guidance::hill_point::test_orbit::circular_orbit_state;
    use crate::messages::{AttitudeReferenceMsg, EphemerisMsg, NavigationTranslationMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::astro_constants::D2R;
    use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_dcm};
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(0.5)`.
    const TASK_RATE_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `unitTestSupport.isArrayEqual` and
    /// `np.testing.assert_allclose(rtol=0)`: absolute, per component.
    fn assert_vector_close(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64) {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!(
                (a - e).abs() <= accuracy,
                "{actual:?} != {expected:?} within {accuracy}"
            );
        }
    }

    fn ephemeris(position: Vector3<f64>, velocity: Vector3<f64>) -> EphemerisMsg {
        EphemerisMsg {
            position_bdy_zero_n_m: position,
            velocity_bdy_zero_n_mps: velocity,
            ..Default::default()
        }
    }

    /// The module with upstream's test threshold of 1 deg, set on the config
    /// after construction as a Python user would.
    fn module_with_threshold() -> CelestialTwoBodyPoint {
        let mut module = CelestialTwoBodyPoint::new(CelestialTwoBodyPointConfig::default());
        module.config.singularity_thresh_rad = 1.0 * D2R;
        module
    }

    /// Upstream test helper `computeCelestialTwoBodyPoint`: the analytic truth
    /// from relative states in inertial components.
    fn upstream_truth(
        r_p1: Vector3<f64>,
        v_p1: Vector3<f64>,
        a_p1: Vector3<f64>,
        r_p2: Vector3<f64>,
        v_p2: Vector3<f64>,
        a_p2: Vector3<f64>,
    ) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
        let r_n = r_p1.cross(&r_p2);
        let v_n = v_p1.cross(&r_p2) + r_p1.cross(&v_p2);
        let a_n = a_p1.cross(&r_p2) + r_p1.cross(&a_p2) + 2.0 * v_p1.cross(&v_p2);

        let r1_hat = r_p1 / r_p1.norm();
        let r3_hat = r_n / r_n.norm();
        let r2_hat = r3_hat.cross(&r1_hat);
        let rn = Matrix3::from_rows(&[r1_hat.transpose(), r2_hat.transpose(), r3_hat.transpose()]);

        let identity = Matrix3::identity();
        let c1 = identity - r1_hat * r1_hat.transpose();
        let dr1_hat = 1.0 / r_p1.norm() * (c1 * v_p1);
        let c3 = identity - r3_hat * r3_hat.transpose();
        let dr3_hat = 1.0 / r_n.norm() * (c3 * v_n);
        let dr2_hat = dr3_hat.cross(&r1_hat) + r3_hat.cross(&dr1_hat);

        let omega_rn_r = Vector3::new(
            r3_hat.dot(&dr2_hat),
            r1_hat.dot(&dr3_hat),
            r2_hat.dot(&dr1_hat),
        );

        let temp33_1 = 2.0 * dr1_hat * r1_hat.transpose() + r1_hat * dr1_hat.transpose();
        let ddr1_hat = 1.0 / r_p1.norm() * (c1 * a_p1 - temp33_1 * v_p1);
        let temp33_3 = 2.0 * dr3_hat * r3_hat.transpose() + r3_hat * dr3_hat.transpose();
        let ddr3_hat = 1.0 / r_n.norm() * (c3 * a_n - temp33_3 * v_n);
        let ddr2_hat =
            ddr3_hat.cross(&r1_hat) + r3_hat.cross(&ddr1_hat) + 2.0 * dr3_hat.cross(&dr1_hat);

        let domega_rn_r = Vector3::new(
            dr3_hat.dot(&dr2_hat) + r3_hat.dot(&ddr2_hat),
            dr1_hat.dot(&dr3_hat) + r1_hat.dot(&ddr3_hat),
            dr2_hat.dot(&dr1_hat) + r2_hat.dot(&ddr1_hat),
        );
        (
            dcm_to_mrp(rn),
            rn.transpose() * omega_rn_r,
            rn.transpose() * domega_rn_r,
        )
    }

    /// Runs the module as the upstream sim tests do: spacecraft at rest at
    /// the origin, primary on the circular test orbit, a 0.5 s task run to
    /// 1 s. Returns the three recorded reference messages.
    fn run_upstream_sim(secondary: Option<EphemerisMsg>) -> Vec<AttitudeReferenceMsg> {
        let (r, v) = circular_orbit_state();
        let nav_output = Output::new(NavigationTranslationMsg::default());
        let primary_output = Output::new(ephemeris(r, v));
        let secondary_output = Output::new(secondary.clone().unwrap_or_default());
        let mut module = module_with_threshold();
        let mut recorder = MessageRecorder::<AttitudeReferenceMsg>::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&nav_output, &mut module.trans_nav_in_msg);
        simulation.connect(&primary_output, &mut module.cel_body_in_msg);
        if secondary.is_some() {
            simulation.connect(&secondary_output, &mut module.sec_cel_body_in_msg);
        }
        simulation.connect(&module.att_ref_out_msg, &mut recorder.input_msg);
        simulation.add_module("celestialTwoBodyPoint", &mut module, TASK_RATE_NANOS, 0);
        simulation.add_module("recorder", &mut recorder, TASK_RATE_NANOS, 0);
        simulation.run_for(1_000_000_000);
        drop(simulation);
        recorder.samples
    }

    /// Upstream `test_celestialTwoBodyPoint.py::
    /// test_celestialTwoBodyPointTestFunction`: no secondary body, so the
    /// secondary direction is the primary orbit normal.
    #[test]
    fn matches_upstream_celestial_two_body_point_unit_test() {
        let samples = run_upstream_sim(None);

        let (r_p1, v_p1) = circular_orbit_state();
        let a_p1 = Vector3::zeros();
        let (sigma_rn, omega_rn_n, domega_rn_n) = upstream_truth(
            r_p1,
            v_p1,
            a_p1,
            r_p1.cross(&v_p1),
            r_p1.cross(&a_p1),
            v_p1.cross(&a_p1),
        );
        let accuracy = 1e-12;
        assert_eq!(samples.len(), 3);
        for sample in &samples {
            assert_vector_close(sample.sigma_rn, sigma_rn, accuracy);
            assert_vector_close(sample.omega_rn_n_radps, omega_rn_n, accuracy);
            assert_vector_close(sample.domega_rn_n_radps2, domega_rn_n, accuracy);
        }
    }

    /// Upstream `test_celestialTwoBodyPoint.py::
    /// test_secBodyCelestialTwoBodyPointTestFunction`: a static secondary body
    /// at (500, 500, 500) m, against upstream's hard-coded truth. Upstream's
    /// extra 0.6 s run after the checks asks for a stop time already passed
    /// and checks nothing.
    #[test]
    fn matches_upstream_sec_body_celestial_two_body_point_unit_test() {
        let samples = run_upstream_sim(Some(ephemeris(
            Vector3::new(500.0, 500.0, 500.0),
            Vector3::zeros(),
        )));

        let accuracy = 1e-10;
        assert_eq!(samples.len(), 3);
        for sample in &samples {
            assert_vector_close(
                sample.sigma_rn,
                Vector3::new(0.474475084038, 0.273938317493, 0.191443718765),
                accuracy,
            );
            assert_vector_close(
                sample.omega_rn_n_radps,
                Vector3::new(1.59336987e-04, 2.75979758e-04, 2.64539877e-04),
                accuracy,
            );
            assert_vector_close(
                sample.domega_rn_n_radps2,
                Vector3::new(-1.2147297051129594e-07, -4.1793434670048644e-08, 0.0),
                accuracy,
            );
        }
    }

    #[derive(Clone, Copy, Debug)]
    enum SecondaryMode {
        Moving,
        Absent,
        Parallel,
        Antiparallel,
    }

    /// Upstream `test_celestialTwoBodyPoint.py::test_reference_derivatives`,
    /// every `secondary_mode` and `sample_time`: bodies on straight lines,
    /// the published rate checked against central differences of the
    /// published attitude, the published acceleration against central
    /// differences of the published rate.
    #[test]
    fn matches_upstream_reference_derivatives_test() {
        let primary_position_n = Vector3::new(7e6, 1e6, -2e6);
        let primary_velocity_n = Vector3::new(-1e3, 7e3, 2e3);
        let secondary_position_n = Vector3::new(-3e6, 8e6, 4e6);
        let secondary_velocity_n = Vector3::new(5e3, 1e3, -3e3);
        let step = 0.1;
        let rate_tolerance = 1e-11;
        let acceleration_tolerance = 1e-13;

        for mode in [
            SecondaryMode::Moving,
            SecondaryMode::Absent,
            SecondaryMode::Parallel,
            SecondaryMode::Antiparallel,
        ] {
            for sample_time in [0.0, 250.0] {
                let nav_output = Output::new(NavigationTranslationMsg::default());
                let primary_output = Output::new(EphemerisMsg::default());
                let secondary_output = Output::new(EphemerisMsg::default());
                let mut module = module_with_threshold();
                nav_output.connect_to(&mut module.trans_nav_in_msg);
                primary_output.connect_to(&mut module.cel_body_in_msg);
                if !matches!(mode, SecondaryMode::Absent) {
                    secondary_output.connect_to(&mut module.sec_cel_body_in_msg);
                }
                module.init();

                let mut evaluate = |time: f64| {
                    let primary = ephemeris(
                        primary_position_n + time * primary_velocity_n,
                        primary_velocity_n,
                    );
                    let secondary = match mode {
                        SecondaryMode::Moving => Some(ephemeris(
                            secondary_position_n + time * secondary_velocity_n,
                            secondary_velocity_n,
                        )),
                        SecondaryMode::Absent => None,
                        SecondaryMode::Parallel => Some(ephemeris(
                            2.0 * primary.position_bdy_zero_n_m,
                            2.0 * primary_velocity_n,
                        )),
                        SecondaryMode::Antiparallel => Some(ephemeris(
                            -2.0 * primary.position_bdy_zero_n_m,
                            -2.0 * primary_velocity_n,
                        )),
                    };
                    primary_output.write(primary);
                    if let Some(secondary) = secondary {
                        secondary_output.write(secondary);
                    }
                    // The algorithm is instantaneous; the sample time is
                    // carried by the input states.
                    module.update(&context(0));
                    let output = module.att_ref_out_msg.read();
                    (
                        mrp_to_dcm(output.sigma_rn),
                        output.omega_rn_n_radps,
                        output.domega_rn_n_radps2,
                    )
                };

                let (dcm_minus, omega_minus_n, _) = evaluate(sample_time - step);
                let (dcm_rn, omega_rn_n, domega_rn_n) = evaluate(sample_time);
                let (dcm_plus, omega_plus_n, _) = evaluate(sample_time + step);
                let dcm_dot = (dcm_plus - dcm_minus) / (2.0 * step);
                // 0.5 * np.sum(np.cross(dcm_RN, dcm_dot), axis=0): row-wise
                // cross products of the basis vectors with their rates.
                let omega_difference_n = 0.5
                    * (0..3)
                        .map(|row| {
                            dcm_rn
                                .row(row)
                                .transpose()
                                .cross(&dcm_dot.row(row).transpose())
                        })
                        .fold(Vector3::zeros(), |sum, term| sum + term);
                let acceleration_difference_n = (omega_plus_n - omega_minus_n) / (2.0 * step);

                assert_vector_close(omega_rn_n, omega_difference_n, rate_tolerance);
                assert_vector_close(
                    domega_rn_n,
                    acceleration_difference_n,
                    acceleration_tolerance,
                );
            }
        }
    }

    // Not upstream: within the singularity threshold the secondary body is
    // ignored, so a (nearly) parallel secondary gives exactly the absent-
    // secondary reference; outside the threshold it is used.
    #[test]
    fn parallel_secondary_within_the_threshold_falls_back_to_the_orbit_normal() {
        let (r, v) = circular_orbit_state();
        let absent = run_upstream_sim(None)[0].clone();
        // 0.5 deg off the primary direction, inside the 1 deg threshold.
        let tilted = r + r.norm() * (0.5 * D2R).tan() * Vector3::z();
        let near_parallel = run_upstream_sim(Some(ephemeris(3.0 * tilted, v)))[0].clone();
        assert_eq!(near_parallel.sigma_rn, absent.sigma_rn);
        assert_eq!(near_parallel.omega_rn_n_radps, absent.omega_rn_n_radps);
        assert_eq!(near_parallel.domega_rn_n_radps2, absent.domega_rn_n_radps2);

        // 2 deg off: outside the threshold, so the secondary sets r̂3.
        let tilted = r + r.norm() * (2.0 * D2R).tan() * Vector3::z();
        let used = run_upstream_sim(Some(ephemeris(3.0 * tilted, v)))[0].clone();
        let r3_hat = mrp_to_dcm(used.sigma_rn).row(2).transpose();
        let expected = r.cross(&tilted).normalize();
        assert_vector_close(r3_hat, expected, 1e-12);
    }

    // Not upstream: upstream samples whether the secondary message is linked
    // at reset, so a secondary connected later is ignored until the next
    // reset.
    #[test]
    fn samples_the_secondary_body_link_at_reset() {
        let (r, v) = circular_orbit_state();
        let nav_output = Output::new(NavigationTranslationMsg::default());
        let primary_output = Output::new(ephemeris(r, v));
        let secondary_output = Output::new(ephemeris(
            Vector3::new(500.0, 500.0, 500.0),
            Vector3::zeros(),
        ));
        let mut module = module_with_threshold();
        nav_output.connect_to(&mut module.trans_nav_in_msg);
        primary_output.connect_to(&mut module.cel_body_in_msg);
        module.init();
        secondary_output.connect_to(&mut module.sec_cel_body_in_msg);

        module.update(&context(0));
        let absent = run_upstream_sim(None)[0].clone();
        assert_eq!(module.att_ref_out_msg.read().sigma_rn, absent.sigma_rn);

        Module::reset(&mut module, &context(TASK_RATE_NANOS));
        module.update(&context(TASK_RATE_NANOS));
        assert_vector_close(
            module.att_ref_out_msg.read().sigma_rn,
            Vector3::new(0.474475084038, 0.273938317493, 0.191443718765),
            1e-10,
        );
    }
}
