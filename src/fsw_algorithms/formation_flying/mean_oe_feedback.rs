//! Lyapunov feedback on mean orbital-element differences for a deputy
//! spacecraft: port of upstream Basilisk `meanOEFeedback`.
//!
//! Every update converts the chief's and the deputy's inertial states to
//! osculating classical elements, maps those to mean elements with Brouwer's
//! first-order J2 theory ([`mean_osculating_map`]), and forms the deputy-minus-
//! chief element difference less the configured target difference, either in
//! classical or in equinoctial elements ([`OrbitalElementSet`]). The control
//! force (Schaub & Junkins, *Analytical Mechanics of Space Systems*, ch. 14)
//!
//! `u = -[B(oe_d)]^T [K] Δoe`
//!
//! uses the Gauss variational-equation control matrix `B` evaluated on the
//! deputy's mean elements. It is computed in the deputy's Hill (LVLH) frame and
//! published in inertial components.

use std::f64::consts::PI;

use nalgebra::{Matrix6, Matrix6x3, Vector3, Vector6};

use crate::messages::{InertialForceCommandMsg, Input, NavigationTranslationMsg, Output};
use crate::utilities::orbital_motion::{
    ClassicElements, EquinoctialElements, MeanOscDirection, classic_to_equinoctial,
    eccentric_to_mean_anomaly, hill_frame, mean_osculating_map, state_to_elements,
    true_to_eccentric_anomaly,
};
use crate::{Module, SimulationContext};

/// Orbital-element set the feedback runs on: upstream `oeType`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum OrbitalElementSet {
    /// `(a, e, i, Ω, ω, M)`; upstream `0`, the default.
    #[default]
    Classic = 0,
    /// `(a, e sin(ω+Ω), e cos(ω+Ω), tan(i/2) sin Ω, tan(i/2) cos Ω, Ω+ω+M)`,
    /// which avoids the circular and equatorial singularities; upstream `1`.
    Equinoctial = 1,
}

/// User-settable parameters of [`MeanOeFeedback`]. `Default` is upstream's
/// zero-initialized configuration, which [`Module::reset`] reports as
/// incomplete until `mu_m3ps2`, `req_m` and `j2` are set.
#[derive(Clone, Debug)]
pub struct MeanOeFeedbackConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `K`: Lyapunov gain matrix. Upstream stores it row-major as
    /// `double[36]`; build it from that array with [`Matrix6::from_row_slice`].
    pub k: Matrix6<f64>,
    /// Upstream `targetDiffOeMean`: target deputy-minus-chief mean element
    /// difference, in the set chosen by `oe_type`. The first entry is the
    /// semi-major axis difference normalized by the chief's, `δa/a`.
    pub target_diff_oe_mean: Vector6<f64>,
    /// Upstream `oeType`: element set of the feedback and of
    /// `target_diff_oe_mean`.
    pub oe_type: OrbitalElementSet,
    /// Upstream `mu` \[m^3/s^2\]: gravitational parameter of the central body.
    pub mu_m3ps2: f64,
    /// Upstream `req` \[m\]: equatorial radius of the central body.
    pub req_m: f64,
    /// Upstream `J2` \[-\]: J2 oblateness coefficient of the central body.
    pub j2: f64,
}

impl Default for MeanOeFeedbackConfig {
    fn default() -> Self {
        Self {
            name: "meanOEFeedback".to_string(),
            k: Matrix6::zeros(),
            target_diff_oe_mean: Vector6::zeros(),
            oe_type: OrbitalElementSet::Classic,
            mu_m3ps2: 0.0,
            req_m: 0.0,
            j2: 0.0,
        }
    }
}

/// Mean orbital-element Lyapunov controller for a deputy spacecraft.
#[derive(Clone, Debug)]
pub struct MeanOeFeedback {
    /// User-settable parameters.
    pub config: MeanOeFeedbackConfig,
    /// Upstream `chiefTransInMsg`: chief's inertial position and velocity.
    pub chief_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `deputyTransInMsg`: deputy's inertial position and velocity.
    pub deputy_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `forceOutMsg`: deputy control force, inertial components.
    pub force_out_msg: Output<InertialForceCommandMsg>,
}

impl MeanOeFeedback {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: MeanOeFeedbackConfig) -> Self {
        Self {
            config,
            chief_trans_in_msg: Input::default(),
            deputy_trans_in_msg: Input::default(),
            force_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_meanOEFeedback`: the module retains no state, so a
    /// reset only reports missing inputs and unset planet parameters.
    fn check_configuration(&self) {
        if !self.chief_trans_in_msg.is_connected() {
            log::error!("meanOEFeedback.chief_trans_in_msg is not connected");
        }
        if !self.deputy_trans_in_msg.is_connected() {
            log::error!("meanOEFeedback.deputy_trans_in_msg is not connected");
        }
        if self.config.mu_m3ps2 <= 0.0 {
            log::error!("meanOEFeedback: mu_m3ps2 must be set to a positive value");
        }
        if self.config.req_m <= 0.0 {
            log::error!("meanOEFeedback: req_m must be set to a positive value");
        }
        if self.config.j2 <= 0.0 {
            log::error!("meanOEFeedback: j2 must be set to a positive value");
        }
    }

    /// Upstream `calc_LyapunovFeedback`: control force \[N\] on the deputy,
    /// inertial components.
    fn control_force(
        &self,
        chief: &NavigationTranslationMsg,
        deputy: &NavigationTranslationMsg,
    ) -> Vector3<f64> {
        let config = &self.config;
        let mean_elements = |nav: &NavigationTranslationMsg| {
            let osculating =
                state_to_elements(config.mu_m3ps2, nav.position_bn_n_m, nav.velocity_bn_n_mps);
            mean_osculating_map(
                config.req_m,
                config.j2,
                &osculating,
                MeanOscDirection::OsculatingToMean,
            )
        };
        let chief_mean = mean_elements(chief);
        let deputy_mean = mean_elements(deputy);

        let target = &config.target_diff_oe_mean;
        let (element_error, control_matrix) = match config.oe_type {
            OrbitalElementSet::Classic => (
                classic_element_error(&chief_mean, &deputy_mean, target),
                classic_control_matrix(config.mu_m3ps2, &deputy_mean),
            ),
            OrbitalElementSet::Equinoctial => {
                let chief_eq = classic_to_equinoctial(&chief_mean);
                let deputy_eq = classic_to_equinoctial(&deputy_mean);
                (
                    equinoctial_element_error(&chief_eq, &deputy_eq, target),
                    equinoctial_control_matrix(config.mu_m3ps2, &deputy_eq),
                )
            }
        };

        let force_hill = -(control_matrix.transpose() * (config.k * element_error));
        // [HN] of the deputy, built exactly as upstream builds its LVLH frame.
        hill_frame(deputy.position_bn_n_m, deputy.velocity_bn_n_mps).transpose() * force_hill
    }
}

/// Classical mean element error `(δa/a, δe, δi, δΩ, δω, δM)` less the target,
/// with the angle errors wrapped to `[-π, π]` after the target is removed.
fn classic_element_error(
    chief: &ClassicElements,
    deputy: &ClassicElements,
    target: &Vector6<f64>,
) -> Vector6<f64> {
    let mean_anomaly = |oe: &ClassicElements| {
        let eccentric_anomaly = true_to_eccentric_anomaly(oe.true_anomaly_rad, oe.eccentricity);
        eccentric_to_mean_anomaly(eccentric_anomaly, oe.eccentricity)
    };
    let wrap = |angle| adjust_range(-PI, PI, angle);
    Vector6::new(
        (deputy.semi_major_axis - chief.semi_major_axis) / chief.semi_major_axis - target[0],
        deputy.eccentricity - chief.eccentricity - target[1],
        wrap(deputy.inclination_rad - chief.inclination_rad - target[2]),
        wrap(deputy.raan_rad - chief.raan_rad - target[3]),
        wrap(deputy.arg_periapsis_rad - chief.arg_periapsis_rad - target[4]),
        wrap(mean_anomaly(deputy) - mean_anomaly(chief) - target[5]),
    )
}

/// Equinoctial mean element error `(δa/a, δP1, δP2, δQ1, δQ2, δl)` less the
/// target; only the mean longitude error is wrapped to `[-π, π]`.
fn equinoctial_element_error(
    chief: &EquinoctialElements,
    deputy: &EquinoctialElements,
    target: &Vector6<f64>,
) -> Vector6<f64> {
    Vector6::new(
        (deputy.semi_major_axis - chief.semi_major_axis) / chief.semi_major_axis - target[0],
        deputy.p1 - chief.p1 - target[1],
        deputy.p2 - chief.p2 - target[2],
        deputy.q1 - chief.q1 - target[3],
        deputy.q2 - chief.q2 - target[4],
        adjust_range(
            -PI,
            PI,
            deputy.mean_longitude_rad - chief.mean_longitude_rad - target[5],
        ),
    )
}

/// Upstream `calc_B_cl`: Gauss control matrix of the classical elements
/// (Schaub & Junkins Eq. 14.213), mapping a Hill-frame acceleration to element
/// rates; the semi-major-axis row is normalized by `a`.
fn classic_control_matrix(mu: f64, oe: &ClassicElements) -> Matrix6x3<f64> {
    let a = oe.semi_major_axis;
    let e = oe.eccentricity;
    let i = oe.inclination_rad;
    let f = oe.true_anomaly_rad;
    let theta = oe.arg_periapsis_rad + f;
    let (sin_f, cos_f) = (f.sin(), f.cos());
    let (sin_theta, cos_theta) = (theta.sin(), theta.cos());
    let eta = (1.0 - e * e).sqrt();
    let b = a * (1.0 - e * e).sqrt();
    // powf calls libm's pow like upstream; powi(3) would round differently.
    let n = (mu / a.powf(3.0)).sqrt();
    let h = n * a * b;
    let p = a * (1.0 - e * e);
    let r = p / (1.0 + e * cos_f);

    #[rustfmt::skip]
    let control_matrix = Matrix6x3::new(
        2.0 * a.powi(2) * e * sin_f / h / a, 2.0 * a.powi(2) * p / (h * r) / a, 0.0,
        p * sin_f / h, ((p + r) * cos_f + r * e) / h, 0.0,
        0.0, 0.0, r * cos_theta / h,
        0.0, 0.0, r * sin_theta / (h * i.sin()),
        -p * cos_f / (h * e), (p + r) * sin_f / (h * e), -r * sin_theta * i.cos() / (h * i.sin()),
        eta * (p * cos_f - 2.0 * r * e) / (h * e), -eta * (p + r) * sin_f / (h * e), 0.0,
    );
    control_matrix
}

/// Upstream `calc_B_eq`: Gauss control matrix of the equinoctial elements
/// (Naasz, Karlgaard & Hall, 2002, adapted to Basilisk's element definitions);
/// the semi-major-axis row is normalized by `a`.
fn equinoctial_control_matrix(mu: f64, oe: &EquinoctialElements) -> Matrix6x3<f64> {
    let a = oe.semi_major_axis;
    let (p1, p2) = (oe.p1, oe.p2);
    let (q1, q2) = (oe.q1, oe.q2);
    let (sin_l, cos_l) = (oe.true_longitude_rad.sin(), oe.true_longitude_rad.cos());
    let b = a * (1.0 - p1 * p1 - p2 * p2).sqrt();
    // powf calls libm's pow like upstream; powi(3) would round differently.
    let n = (mu / a.powf(3.0)).sqrt();
    let h = n * a * b;
    let p_r = 1.0 + p1 * sin_l + p2 * cos_l;
    let r_h = h / (mu * p_r);
    let r = r_h * h;
    let p = p_r * r;
    let node_term = 0.5 * r_h * (1.0 + q1 * q1 + q2 * q2);

    #[rustfmt::skip]
    let control_matrix = Matrix6x3::new(
        2.0 * a.powi(2) / h * (p2 * sin_l - p1 * cos_l) / a, 2.0 * a.powi(2) * p_r / h / a, 0.0,
        -p * cos_l / h, 1.0 / h * (r * p1 + (r + p) * sin_l), r_h * p2 * (q2 * sin_l - q1 * cos_l),
        p * sin_l / h, 1.0 / h * (r * p2 + (r + p) * cos_l), r_h * p1 * (q1 * cos_l - q2 * sin_l),
        0.0, 0.0, node_term * sin_l,
        0.0, 0.0, node_term * cos_l,
        -p * a / (h * (a + b)) * ((p1 * sin_l + p2 * cos_l) + 2.0 * b / a),
        r_h * a * (1.0 + p_r) / (a + b) * (p2 * sin_l - p1 * cos_l),
        r_h * (q2 * sin_l - q1 * cos_l),
    );
    control_matrix
}

/// Upstream `adjust_range` (and `spacecraftReconfig`'s `AdjustRange`): shifts
/// `angle` by whole multiples of `upper - lower` until it lies in
/// `[lower, upper]`.
///
/// Repeated subtraction, as upstream, rather than a remainder: the rounding of
/// the result differs. Every caller passes constant bounds with
/// `lower < upper`, so upstream's "illegal parameters" branch is not ported.
/// NaN passes through unchanged as upstream; an infinite angle, on which
/// upstream loops forever, is returned unchanged.
pub(crate) fn adjust_range(lower: f64, upper: f64, angle: f64) -> f64 {
    if angle.is_infinite() {
        return angle;
    }
    let width = upper - lower;
    let mut adjusted = angle;
    while adjusted > upper {
        adjusted -= width;
    }
    while adjusted < lower {
        adjusted += width;
    }
    adjusted
}

impl Module for MeanOeFeedback {
    fn init(&mut self) {
        self.force_out_msg.write(InertialForceCommandMsg::default());
        self.check_configuration();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_configuration();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let chief = self.chief_trans_in_msg.read();
        let deputy = self.deputy_trans_in_msg.read();
        let force_request_inertial_n = self.control_force(&chief, &deputy);
        self.force_out_msg.write(InertialForceCommandMsg {
            force_request_inertial_n,
        });
    }
}

/// Inputs shared by the upstream `formationFlying` unit tests.
#[cfg(test)]
pub(crate) mod test_orbits {
    use nalgebra::Vector3;

    use crate::messages::NavigationTranslationMsg;
    use crate::utilities::astro_constants::MU_EARTH;
    use crate::utilities::orbital_motion::ClassicElements;

    /// Upstream tests' `orbitalMotion.MU_EARTH * 1e9` \[m^3/s^2\].
    pub(crate) const MU_EARTH_M3PS2: f64 = MU_EARTH * 1e9;

    /// Upstream's pure-Python `orbitalMotion.elem2rv` (elliptic branch), which
    /// the upstream tests use to build their navigation inputs. Its operation
    /// order differs from the C `elem2rv` in the last bits.
    pub(crate) fn python_elem2rv(mu: f64, oe: &ClassicElements) -> (Vector3<f64>, Vector3<f64>) {
        let (e, i, raan, argp, f) = (
            oe.eccentricity,
            oe.inclination_rad,
            oe.raan_rad,
            oe.arg_periapsis_rad,
            oe.true_anomaly_rad,
        );
        let p = oe.semi_major_axis * (1.0 - e * e);
        let r = p / (1.0 + e * f.cos());
        let theta = argp + f;
        let position = Vector3::new(
            r * (theta.cos() * raan.cos() - i.cos() * theta.sin() * raan.sin()),
            r * (theta.cos() * raan.sin() + i.cos() * theta.sin() * raan.cos()),
            r * (theta.sin() * i.sin()),
        );
        let h = (mu * p).sqrt();
        let velocity = Vector3::new(
            -mu / h
                * (raan.cos() * (e * argp.sin() + theta.sin())
                    + i.cos() * (e * argp.cos() + theta.cos()) * raan.sin()),
            -mu / h
                * (raan.sin() * (e * argp.sin() + theta.sin())
                    - i.cos() * (e * argp.cos() + theta.cos()) * raan.cos()),
            mu / h * (e * argp.cos() + theta.cos()) * i.sin(),
        );
        (position, velocity)
    }

    /// Navigation message of a spacecraft on `oe`, built as upstream's tests
    /// build theirs.
    pub(crate) fn navigation(oe: &ClassicElements) -> NavigationTranslationMsg {
        let (position_bn_n_m, velocity_bn_n_mps) = python_elem2rv(MU_EARTH_M3PS2, oe);
        NavigationTranslationMsg {
            position_bn_n_m,
            velocity_bn_n_mps,
            ..Default::default()
        }
    }

    /// The chief of `test_meanOEFeedback.py` and `test_spacecraftReconfig.py`.
    pub(crate) fn chief() -> NavigationTranslationMsg {
        navigation(&ClassicElements {
            semi_major_axis: 20000e3,
            eccentricity: 0.1,
            inclination_rad: 0.2,
            raan_rad: 0.3,
            arg_periapsis_rad: 0.4,
            true_anomaly_rad: 0.5,
            ..Default::default()
        })
    }

    /// The deputy of `test_meanOEFeedback.py` and `test_spacecraftReconfig.py`.
    pub(crate) fn deputy() -> NavigationTranslationMsg {
        navigation(&ClassicElements {
            semi_major_axis: (1.0 + 0.0006) * 7000e3,
            eccentricity: 0.2 + 0.0005,
            inclination_rad: 0.0 + 0.0004,
            raan_rad: 0.0 + 0.0003,
            arg_periapsis_rad: 0.0 + 0.0002,
            true_anomaly_rad: 0.0001,
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::{Matrix6, Vector3, Vector6};

    use super::test_orbits::{MU_EARTH_M3PS2, chief, deputy};
    use super::{MeanOeFeedback, MeanOeFeedbackConfig, OrbitalElementSet, adjust_range};
    use crate::messages::{NavigationTranslationMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::astro_constants::{J2_EARTH, REQ_EARTH};
    use crate::utilities::orbital_motion::{
        MeanOscDirection, eccentric_to_mean_anomaly, mean_osculating_map, state_to_elements,
        true_to_eccentric_anomaly,
    };
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(0.1)`.
    const TASK_RATE_NANOS: u64 = 100_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The configuration `test_meanOEFeedback.py` sets.
    fn upstream_config(oe_type: OrbitalElementSet) -> MeanOeFeedbackConfig {
        MeanOeFeedbackConfig {
            k: Matrix6::identity() * 1e7,
            oe_type,
            mu_m3ps2: MU_EARTH_M3PS2,
            req_m: REQ_EARTH * 1e3,
            j2: J2_EARTH,
            ..Default::default()
        }
    }

    fn module_with_inputs(
        config: MeanOeFeedbackConfig,
        chief_out: &Output<NavigationTranslationMsg>,
        deputy_out: &Output<NavigationTranslationMsg>,
    ) -> MeanOeFeedback {
        let mut module = MeanOeFeedback::new(config);
        chief_out.connect_to(&mut module.chief_trans_in_msg);
        deputy_out.connect_to(&mut module.deputy_trans_in_msg);
        module
    }

    /// Upstream `isArrayEqual`: absolute tolerance per component, NaN fails.
    fn assert_array_equal(actual: Vector3<f64>, truth: [f64; 3], accuracy: f64, what: &str) {
        for (index, (value, expected)) in actual.iter().zip(truth).enumerate() {
            assert!(
                (value - expected).abs() <= accuracy,
                "{what}[{index}] = {value:.17e}, truth {expected:.17e}"
            );
        }
    }

    /// `test_meanOEFeedback`, both `useClassicElem` parametrizations, with
    /// upstream's hard-coded truth and `accuracy = 1e-6`. Upstream runs a
    /// 0.1 s task to t = 0.1 s and checks the first recorded sample.
    #[test]
    fn matches_upstream_mean_oe_feedback_unit_test() {
        let accuracy = 1e-6;
        let cases = [
            (
                OrbitalElementSet::Classic,
                [
                    -849.573_474_065_443_4,
                    1_849.776_412_650_328_4,
                    136.078_177_344_793_18,
                ],
            ),
            (
                OrbitalElementSet::Equinoctial,
                [
                    -1_655.371_882_078_803,
                    1_788.617_763_790_425_2,
                    52.548_142_374_539_38,
                ],
            ),
        ];
        for (oe_type, truth) in cases {
            let chief_out = Output::new(chief());
            let deputy_out = Output::new(deputy());
            let mut module = module_with_inputs(upstream_config(oe_type), &chief_out, &deputy_out);
            let mut recorder = MessageRecorder::default();
            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            simulation.connect(&module.force_out_msg, &mut recorder.input_msg);
            simulation.add_module("meanOEFeedback", &mut module, TASK_RATE_NANOS, 0);
            simulation.add_module("recorder", &mut recorder, TASK_RATE_NANOS, 0);
            simulation.run_for(TASK_RATE_NANOS);

            assert_eq!(recorder.samples.len(), 2, "{oe_type:?}");
            assert_array_equal(
                recorder.samples[0].force_request_inertial_n,
                truth,
                accuracy,
                &format!("{oe_type:?} forceRequestInertial"),
            );
            // The inputs are constant, so the second sample repeats the first.
            assert_eq!(recorder.samples[1], recorder.samples[0], "{oe_type:?}");
        }
    }

    /// Beyond `test_meanOEFeedback`, whose gain is diagonal and whose target is
    /// zero: a full, non-symmetric gain (row-major like upstream `K`) and a
    /// nonzero target difference, for both element sets. Truth from the
    /// upstream C module compiled with gcc on the same inputs; upstream's
    /// `accuracy = 1e-6`.
    #[test]
    fn matches_upstream_c_with_a_full_gain_and_a_target_difference() {
        #[rustfmt::skip]
        let k_row_major = [
            1e7, 2e5, 0.0, 0.0, -3e5, 0.0,
            0.0, 2e7, 1e5, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.5e7, 4e5, 0.0, 0.0,
            0.0, -2e5, 0.0, 1e7, 0.0, 1e5,
            3e5, 0.0, 0.0, 0.0, 5e6, 0.0,
            0.0, 0.0, -1e5, 0.0, 0.0, 1e7,
        ];
        let cases = [
            (
                OrbitalElementSet::Classic,
                [
                    747.082_921_009_238_1,
                    1_553.085_307_900_466_4,
                    409.775_939_327_041_44,
                ],
            ),
            (
                OrbitalElementSet::Equinoctial,
                [
                    -1_879.848_327_049_605_1,
                    1_629.328_426_432_884,
                    37.197_541_617_540_466,
                ],
            ),
        ];
        for (oe_type, truth) in cases {
            let chief_out = Output::new(chief());
            let deputy_out = Output::new(deputy());
            let mut module = module_with_inputs(
                MeanOeFeedbackConfig {
                    k: Matrix6::from_row_slice(&k_row_major),
                    target_diff_oe_mean: Vector6::new(1e-4, -2e-4, 3e-4, -1e-4, 2e-4, 0.1),
                    ..upstream_config(oe_type)
                },
                &chief_out,
                &deputy_out,
            );
            module.init();
            module.update(&context(0));
            assert_array_equal(
                module.force_out_msg.read().force_request_inertial_n,
                truth,
                1e-6,
                &format!("{oe_type:?} forceRequestInertial"),
            );
        }
    }

    /// The feedback is linear in the element error: with the deputy's current
    /// mean element difference as the target, the error and the force vanish.
    #[test]
    fn force_vanishes_when_the_target_difference_is_reached() {
        let mu = MU_EARTH_M3PS2;
        let mean = |nav: &NavigationTranslationMsg| {
            mean_osculating_map(
                REQ_EARTH * 1e3,
                J2_EARTH,
                &state_to_elements(mu, nav.position_bn_n_m, nav.velocity_bn_n_mps),
                MeanOscDirection::OsculatingToMean,
            )
        };
        let (chief_mean, deputy_mean) = (mean(&chief()), mean(&deputy()));
        let mean_anomaly =
            |e: f64, f: f64| eccentric_to_mean_anomaly(true_to_eccentric_anomaly(f, e), e);
        let wrap = |angle| adjust_range(-PI, PI, angle);
        let target = Vector6::new(
            (deputy_mean.semi_major_axis - chief_mean.semi_major_axis) / chief_mean.semi_major_axis,
            deputy_mean.eccentricity - chief_mean.eccentricity,
            wrap(deputy_mean.inclination_rad - chief_mean.inclination_rad),
            wrap(deputy_mean.raan_rad - chief_mean.raan_rad),
            wrap(deputy_mean.arg_periapsis_rad - chief_mean.arg_periapsis_rad),
            wrap(
                mean_anomaly(deputy_mean.eccentricity, deputy_mean.true_anomaly_rad)
                    - mean_anomaly(chief_mean.eccentricity, chief_mean.true_anomaly_rad),
            ),
        );

        let chief_out = Output::new(chief());
        let deputy_out = Output::new(deputy());
        let mut module = module_with_inputs(
            MeanOeFeedbackConfig {
                target_diff_oe_mean: target,
                ..upstream_config(OrbitalElementSet::Classic)
            },
            &chief_out,
            &deputy_out,
        );
        module.init();
        module.update(&context(0));
        // 1e7 gain times a residual element error of a few ULP.
        let force = module.force_out_msg.read().force_request_inertial_n;
        assert!(force.norm() < 1e-6, "force {force:?}");
    }

    /// Doubling the gain doubles the force, and a zero gain gives zero force.
    #[test]
    fn force_scales_with_the_gain_matrix() {
        for oe_type in [OrbitalElementSet::Classic, OrbitalElementSet::Equinoctial] {
            let force_for = |gain: f64| {
                let chief_out = Output::new(chief());
                let deputy_out = Output::new(deputy());
                let mut module = module_with_inputs(
                    MeanOeFeedbackConfig {
                        k: Matrix6::identity() * gain,
                        ..upstream_config(oe_type)
                    },
                    &chief_out,
                    &deputy_out,
                );
                module.init();
                module.update(&context(0));
                module.force_out_msg.read().force_request_inertial_n
            };
            assert_eq!(force_for(0.0), Vector3::zeros());
            assert_eq!(force_for(2e7), force_for(1e7) * 2.0);
        }
    }

    #[test]
    fn init_publishes_a_zero_force() {
        let mut module = MeanOeFeedback::new(MeanOeFeedbackConfig::default());
        module
            .force_out_msg
            .write(crate::messages::InertialForceCommandMsg {
                force_request_inertial_n: Vector3::new(1.0, 2.0, 3.0),
            });
        module.init();
        assert_eq!(
            module.force_out_msg.read().force_request_inertial_n,
            Vector3::zeros()
        );
    }

    #[test]
    fn adjust_range_shifts_by_whole_periods_like_upstream() {
        assert_eq!(adjust_range(-PI, PI, 0.5), 0.5);
        assert_eq!(adjust_range(-PI, PI, PI), PI);
        assert_eq!(adjust_range(-PI, PI, -PI), -PI);
        assert_eq!(adjust_range(-PI, PI, 4.0), 4.0 - 2.0 * PI);
        assert_eq!(adjust_range(-PI, PI, -10.0), -10.0 + 2.0 * PI + 2.0 * PI);
        assert_eq!(adjust_range(0.0, 2.0 * PI, -0.25), -0.25 + 2.0 * PI);
        // Repeated subtraction, not a remainder.
        let mut repeated = 20.0;
        while repeated > PI {
            repeated -= 2.0 * PI;
        }
        assert_eq!(adjust_range(-PI, PI, 20.0).to_bits(), repeated.to_bits());
        assert!(adjust_range(-PI, PI, f64::NAN).is_nan());
        assert_eq!(adjust_range(-PI, PI, f64::INFINITY), f64::INFINITY);
    }
}
