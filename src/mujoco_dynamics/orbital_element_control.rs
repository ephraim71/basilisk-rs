//! Orbital-element feedback control: port of upstream Basilisk
//! `simulation/mujocoDynamics/linearTimeInvariantSystem/orbitalElementControl`.
//!
//! An LTI system on the classical-element error
//! `u = (δa/a, δe, δi, δΩ, δω, δM)` (current minus target, angles wrapped
//! to `[-π, π]`), whose output `y` is mapped to a force through the Gauss
//! variational equations at the current elements,
//!
//! ```text
//! F_LVLH = -Bᵀ(oe) y,    F_N = [HN]ᵀ F_LVLH,
//! ```
//!
//! with `[HN]` the radial / along-track / orbit-normal frame. A
//! proportional gain `K` is the feedthrough `D = K`
//! ([`set_proportional_gain`](OrbitalElementControl::set_proportional_gain));
//! an integral gain integrates the error, `B = I`, `C = K`
//! ([`set_integral_gain`](OrbitalElementControl::set_integral_gain)). The
//! model runs in a scene's dynamics task.
//!
//! Upstream logs singular geometries (zero inclination or eccentricity,
//! zero angular momentum) and a non-symmetric-positive-definite gain with
//! `bskLog` and carries on; here they go to `log::error!` and
//! `log::warn!`.

use nalgebra::{DMatrix, DVector, Matrix6x3, Vector3, Vector6};

use super::linear_time_invariant_system::LinearTimeInvariantSystem;
use super::task::{ModelStateSpec, SceneModel, SceneView, TaskContext};
use crate::SimulationContext;
use crate::fsw_algorithms::formation_flying::mean_oe_feedback::adjust_range;
use crate::messages::{ClassicElementsMsg, InertialForceCommandMsg, Input, Output};
use crate::utilities::orbital_motion::{
    eccentric_to_mean_anomaly, elements_to_state, hill_frame, true_to_eccentric_anomaly,
};

/// Upstream `OrbitalElementControl`.
#[derive(Debug)]
pub struct OrbitalElementControl {
    /// Upstream `ModelTag`.
    pub name: String,
    /// Upstream `mu` \[m^3/s^2\]: gravitational parameter of the central
    /// body.
    pub mu: f64,
    /// The LTI system on the element error (upstream's base class).
    pub system: LinearTimeInvariantSystem,
    /// Upstream `targetOEInMsg`: the target elements.
    pub target_oe_in_msg: Input<ClassicElementsMsg>,
    /// Upstream `currentOEInMsg`: the current (osculating) elements.
    pub current_oe_in_msg: Input<ClassicElementsMsg>,
    /// Upstream `forceOutMsg`: the control force, inertial components.
    pub force_out_msg: Output<InertialForceCommandMsg>,
}

impl Default for OrbitalElementControl {
    fn default() -> Self {
        Self {
            name: "orbitalElementControl".to_string(),
            mu: 0.0,
            system: LinearTimeInvariantSystem::default(),
            target_oe_in_msg: Input::default(),
            current_oe_in_msg: Input::default(),
            force_out_msg: Output::default(),
        }
    }
}

/// Upstream `isMatrixSPD`.
fn is_symmetric_positive_definite(matrix: &DMatrix<f64>) -> bool {
    if (matrix - matrix.transpose()).norm() > 1e-12 {
        return false;
    }
    matrix
        .clone()
        .symmetric_eigen()
        .eigenvalues
        .iter()
        .all(|value| *value > 0.0)
}

fn check_gain(gain: &DMatrix<f64>, setter: &str) {
    if gain.nrows() != 6 || gain.ncols() != 6 {
        log::error!(
            "OrbitalElementControl::{setter}: gain has inconsistent dimensions. Expected size \
             [6x6], but received [{} x {}].",
            gain.nrows(),
            gain.ncols()
        );
    }
    if !is_symmetric_positive_definite(gain) {
        log::warn!("OrbitalElementControl::{setter}: gain is not symmetric positive definite");
    }
}

/// Upstream `calculateClassicalBMatrix`: the Gauss variational equations,
/// rows `(da, de, di, dΩ, dω, dM)`, columns the radial, along-track and
/// normal force per unit mass.
fn classical_b_matrix(mu: f64, oe: &ClassicElementsMsg) -> Matrix6x3<f64> {
    const EPS: f64 = 1e-12;
    let (a, e, i) = (oe.semi_major_axis, oe.eccentricity, oe.inclination_rad);
    let f = oe.true_anomaly_rad;
    let theta = oe.arg_periapsis_rad + f;
    let sin_i = i.sin();
    if sin_i.abs() <= EPS {
        log::error!(
            "OrbitalElementControl::calculateClassicalBMatrix: singular inclination detected \
             (sin(i) near zero)."
        );
    }
    let eta = (1.0 - e * e).sqrt();
    let b = a * eta;
    let n = (mu / a.powi(3)).sqrt();
    let h = n * a * b;
    if h.abs() <= EPS {
        log::error!(
            "OrbitalElementControl::calculateClassicalBMatrix: singular orbit detected (angular \
             momentum h near zero)."
        );
    }
    if e.abs() <= EPS || (h * e).abs() <= EPS {
        log::error!(
            "OrbitalElementControl::calculateClassicalBMatrix: singular eccentricity detected \
             (h*e near zero)."
        );
    }
    let p = a * (1.0 - e * e);
    let denominator = 1.0 + e * f.cos();
    if denominator.abs() <= EPS {
        log::error!(
            "OrbitalElementControl::calculateClassicalBMatrix: invalid geometry (1 + e*cos(f) \
             near zero)."
        );
    }
    let r = p / denominator;

    let mut matrix = Matrix6x3::zeros();
    matrix[(0, 0)] = 2.0 * a.powi(2) * e * f.sin() / h / a;
    matrix[(0, 1)] = 2.0 * a.powi(2) * p / (h * r) / a;
    matrix[(1, 0)] = p * f.sin() / h;
    matrix[(1, 1)] = ((p + r) * f.cos() + r * e) / h;
    matrix[(2, 2)] = r * theta.cos() / h;
    matrix[(3, 2)] = r * theta.sin() / (h * sin_i);
    matrix[(4, 0)] = -p * f.cos() / (h * e);
    matrix[(4, 1)] = (p + r) * f.sin() / (h * e);
    matrix[(4, 2)] = -r * theta.sin() * i.cos() / (h * sin_i);
    matrix[(5, 0)] = eta * (p * f.cos() - 2.0 * r * e) / (h * e);
    matrix[(5, 1)] = -eta * (p + r) * f.sin() / (h * e);
    matrix
}

impl OrbitalElementControl {
    /// Upstream `getInputSize`: the six element errors.
    pub fn input_size(&self) -> usize {
        6
    }

    /// Upstream `getOutputSize`: six.
    pub fn output_size(&self) -> usize {
        6
    }

    /// Upstream `setProportionalGain`: `D = K` (6 × 6, symmetric positive
    /// definite).
    pub fn set_proportional_gain(&mut self, gain: DMatrix<f64>) {
        check_gain(&gain, "setProportionalGain");
        self.system.set_d(gain);
    }

    /// Upstream `setIntegralGain`: `B = I`, `C = K`.
    pub fn set_integral_gain(&mut self, gain: DMatrix<f64>) {
        check_gain(&gain, "setIntegralGain");
        self.system.set_b(DMatrix::identity(6, 6));
        self.system.set_c(gain);
    }

    /// Upstream `readInput`: the element error.
    fn element_error(&self) -> Vector6<f64> {
        let target = self.target_oe_in_msg.read();
        let current = self.current_oe_in_msg.read();
        if target.semi_major_axis == 0.0 {
            log::error!(
                "OrbitalElementControl::readInput: targetOE.a is zero; target message may be \
                 unwritten."
            );
        }
        let wrap = |angle| adjust_range(-std::f64::consts::PI, std::f64::consts::PI, angle);
        let mean_anomaly = |oe: &ClassicElementsMsg| {
            eccentric_to_mean_anomaly(
                true_to_eccentric_anomaly(oe.true_anomaly_rad, oe.eccentricity),
                oe.eccentricity,
            )
        };
        Vector6::new(
            (current.semi_major_axis - target.semi_major_axis) / target.semi_major_axis,
            current.eccentricity - target.eccentricity,
            wrap(current.inclination_rad - target.inclination_rad),
            wrap(current.raan_rad - target.raan_rad),
            wrap(current.arg_periapsis_rad - target.arg_periapsis_rad),
            wrap(mean_anomaly(&current) - mean_anomaly(&target)),
        )
    }

    /// Upstream `writeOutput`: the inertial force of output `y`.
    fn force(&self, y: &DVector<f64>) -> Vector3<f64> {
        let current = self.current_oe_in_msg.read();
        let elements = ClassicElementsMsg {
            semi_major_axis: current.semi_major_axis,
            eccentricity: current.eccentricity,
            inclination_rad: current.inclination_rad,
            raan_rad: current.raan_rad,
            arg_periapsis_rad: current.arg_periapsis_rad,
            true_anomaly_rad: current.true_anomaly_rad,
            ..ClassicElementsMsg::default()
        };
        let control_matrix = classical_b_matrix(self.mu, &elements);
        let y = Vector6::from_column_slice(y.as_slice());
        let force_lvlh = -(control_matrix.transpose() * y);
        let (r_n, v_n) = elements_to_state(self.mu, &elements);
        hill_frame(r_n, v_n).transpose() * force_lvlh
    }
}

impl SceneModel for OrbitalElementControl {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn state_specs(&self) -> Vec<ModelStateSpec> {
        self.system.state_specs()
    }

    fn load_states(&mut self, states: &[&[f64]]) {
        self.system.load_states(states);
    }

    /// Upstream `Reset`: the base dimension checks, a positive `mu` and the
    /// connected inputs, logged.
    fn scene_reset(&mut self, _scene: &SceneView<'_>, _context: &SimulationContext) {
        self.system.check_dimensions(6, 6);
        if self.mu <= 0.0 {
            log::error!("OrbitalElementControl::Reset: mu must be set to a positive value.");
        }
        if !self.target_oe_in_msg.is_connected() {
            log::error!("OrbitalElementControl::Reset: targetOEInMsg was not connected.");
        }
        if !self.current_oe_in_msg.is_connected() {
            log::error!("OrbitalElementControl::Reset: currentOEInMsg was not connected.");
        }
    }

    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        let u = DVector::from_column_slice(self.element_error().as_slice());
        let y = self.system.evaluate(context, &u, 6, 6);
        self.force_out_msg.write(InertialForceCommandMsg {
            force_request_inertial_n: self.force(&y),
        });
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_orbitalElementControl.py`: with a negligible
    //! J2, the controller matches basilisk-rs's `meanOEFeedback` for the
    //! same gains, as proportional and as integral control.

    use nalgebra::Matrix6;

    use super::*;
    use crate::Module;
    use crate::fsw_algorithms::formation_flying::mean_oe_feedback::{
        MeanOeFeedback, MeanOeFeedbackConfig,
    };
    use crate::messages::NavigationTranslationMsg;
    use crate::mujoco_dynamics::MjScene;
    use crate::mujoco_dynamics::tests::{context, run, sec2nano};
    use crate::utilities::astro_constants::{MU_EARTH, REQ_EARTH};

    fn elements(a: f64, e: f64, i: f64, raan: f64, omega: f64, f: f64) -> ClassicElementsMsg {
        ClassicElementsMsg {
            semi_major_axis: a,
            eccentricity: e,
            inclination_rad: i,
            raan_rad: raan,
            arg_periapsis_rad: omega,
            true_anomaly_rad: f,
            ..ClassicElementsMsg::default()
        }
    }

    #[test]
    fn matches_mean_oe_feedback_without_j2() {
        for integral in [true, false] {
            let mu = MU_EARTH * 1e9;
            let chief = elements(20000e3, 0.1, 0.2, 0.3, 0.4, 0.5);
            let deputy = elements(
                (1.0 + 0.0006) * 7000e3,
                0.2 + 0.0005,
                0.0004,
                0.0003,
                0.0002,
                0.0001,
            );
            let nav = |oe: &ClassicElementsMsg| {
                let (r, v) = elements_to_state(mu, oe);
                NavigationTranslationMsg {
                    position_bn_n_m: r,
                    velocity_bn_n_mps: v,
                    ..NavigationTranslationMsg::default()
                }
            };

            let mut truth = MeanOeFeedback::new(MeanOeFeedbackConfig {
                k: Matrix6::identity() * 1e7,
                mu_m3ps2: mu,
                req_m: REQ_EARTH * 1e3,
                j2: 1e-12,
                ..MeanOeFeedbackConfig::default()
            });
            let chief_nav = Output::new(nav(&chief));
            let deputy_nav = Output::new(nav(&deputy));
            chief_nav.connect_to(&mut truth.chief_trans_in_msg);
            deputy_nav.connect_to(&mut truth.deputy_trans_in_msg);
            truth.init();
            truth.update(&context(sec2nano(10.0)));
            let expected = truth.force_out_msg.read().force_request_inertial_n;

            let mut control = OrbitalElementControl {
                mu,
                ..OrbitalElementControl::default()
            };
            let gain = DMatrix::identity(6, 6) * 1e7;
            if integral {
                // The error is constant over 10 s, so K/10 integrated
                // matches K.
                control.set_integral_gain(gain / 10.0);
            } else {
                control.set_proportional_gain(gain);
            }
            let target = Output::new(chief);
            let current = Output::new(deputy);
            target.connect_to(&mut control.target_oe_in_msg);
            current.connect_to(&mut control.current_oe_in_msg);
            let force = control.force_out_msg.clone();

            let mut scene = MjScene::new("<mujoco/>").expect("scene");
            scene.add_model_to_dynamics_task(control, -1);
            scene.init();
            run(&mut scene, sec2nano(0.1), sec2nano(10.0), |_, _| {});
            let actual = force.read().force_request_inertial_n;
            for axis in 0..3 {
                assert!(
                    (actual[axis] - expected[axis]).abs() <= 1e-6 + 1e-6 * expected[axis].abs(),
                    "integral {integral}: {actual} vs {expected}"
                );
            }
        }
    }
}
