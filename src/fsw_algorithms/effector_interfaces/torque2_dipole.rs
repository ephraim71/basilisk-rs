//! Body dipole request for a body torque request: port of upstream Basilisk
//! `torque2Dipole`.
//!
//! A magnetic dipole `μ` in the local field `b` produces the torque `μ × b`.
//! The module publishes the dipole perpendicular to `b` that produces the
//! requested torque `τ`,
//!
//! `μ = (b × τ) / |b|²`,
//!
//! which reproduces the component of `τ` perpendicular to `b` (the part along
//! `b` cannot be produced by torque rods). When `|b|²` is at or below
//! [`DB0_EPS`] the request is zero. Reset zeroes the output so that a disabled
//! module leaves the torque rods idle.

use nalgebra::Vector3;

use crate::messages::{
    BodyTorqueCommandMsg, DipoleRequestBodyMsg, Input, Output, TamSensorBodyMsg,
};
use crate::utilities::linear_algebra::DB0_EPS;
use crate::{Module, SimulationContext};

/// Configuration of [`Torque2Dipole`]; upstream has no parameters.
#[derive(Clone, Debug)]
pub struct Torque2DipoleConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
}

impl Default for Torque2DipoleConfig {
    fn default() -> Self {
        Self {
            name: "torque2Dipole".to_string(),
        }
    }
}

/// Torque-to-dipole conversion: upstream `torque2Dipole`.
#[derive(Clone, Debug)]
pub struct Torque2Dipole {
    /// Module configuration.
    pub config: Torque2DipoleConfig,
    /// Upstream `tamSensorBodyInMsg`: measured magnetic field \[T\], body components.
    pub tam_sensor_body_in_msg: Input<TamSensorBodyMsg>,
    /// Upstream `tauRequestInMsg`: requested torque \[N m\], body components.
    pub tau_request_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `dipoleRequestOutMsg`: requested dipole \[A m^2\], body components.
    pub dipole_request_out_msg: Output<DipoleRequestBodyMsg>,
}

impl Torque2Dipole {
    /// Builds the module with unconnected ports.
    pub fn new(config: Torque2DipoleConfig) -> Self {
        Self {
            config,
            tam_sensor_body_in_msg: Input::default(),
            tau_request_in_msg: Input::default(),
            dipole_request_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_torque2Dipole`: reports missing inputs and zeroes the
    /// dipole request.
    fn reset_state(&mut self) {
        if !self.tam_sensor_body_in_msg.is_connected() {
            log::error!("torque2Dipole.tam_sensor_body_in_msg is not connected");
        }
        if !self.tau_request_in_msg.is_connected() {
            log::error!("torque2Dipole.tau_request_in_msg is not connected");
        }
        self.dipole_request_out_msg
            .write(DipoleRequestBodyMsg::default());
    }
}

/// `(b × τ) / |b|²`, or zero for a field with `|b|² <= DB0_EPS`.
fn dipole_for_torque(field_t: Vector3<f64>, torque_nm: Vector3<f64>) -> Vector3<f64> {
    let field_norm_squared = field_t.dot(&field_t);
    if field_norm_squared > DB0_EPS {
        // Scaled by the reciprocal, as upstream's v3Scale(1 / |b|^2, ...).
        field_t.cross(&torque_nm) * (1.0 / field_norm_squared)
    } else {
        Vector3::zeros()
    }
}

impl Module for Torque2Dipole {
    fn init(&mut self) {
        self.dipole_request_out_msg
            .write(DipoleRequestBodyMsg::default());
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let field_t = self.tam_sensor_body_in_msg.read().magnetic_field_body_t;
        let torque_nm = self.tau_request_in_msg.read().torque_request_body_nm;
        self.dipole_request_out_msg.write(DipoleRequestBodyMsg {
            dipole_b_am2: dipole_for_torque(field_t, torque_nm),
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{BodyTorqueCommandMsg, Output, TamSensorBodyMsg};
    use crate::{Module, SimulationContext};

    use super::{Torque2Dipole, Torque2DipoleConfig};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `unitTestSupport.isVectorEqual`: the norm of the difference is
    /// within `accuracy` and the result holds no NaN.
    fn assert_vector_equal(actual: Vector3<f64>, truth: Vector3<f64>, accuracy: f64, what: &str) {
        assert!(
            !actual.iter().any(|value| value.is_nan()),
            "{what}: NaN in {actual}"
        );
        let error = (actual - truth).norm();
        assert!(
            error <= accuracy,
            "{what}: {actual:?} differs from {truth:?} by {error:e}"
        );
    }

    /// The truth `test_torque2Dipole.py` computes with numpy:
    /// `1 / np.dot(b, b) * np.cross(b, tau)`.
    fn upstream_truth(field_t: Vector3<f64>, torque_nm: Vector3<f64>) -> Vector3<f64> {
        let cross = Vector3::new(
            field_t.y * torque_nm.z - field_t.z * torque_nm.y,
            field_t.z * torque_nm.x - field_t.x * torque_nm.z,
            field_t.x * torque_nm.y - field_t.y * torque_nm.x,
        );
        let inverse_norm_squared =
            1.0 / (field_t.x * field_t.x + field_t.y * field_t.y + field_t.z * field_t.z);
        inverse_norm_squared * cross
    }

    fn connected_module(
        field: &Output<TamSensorBodyMsg>,
        torque: &Output<BodyTorqueCommandMsg>,
    ) -> Torque2Dipole {
        let mut module = Torque2Dipole::new(Torque2DipoleConfig::default());
        field.connect_to(&mut module.tam_sensor_body_in_msg);
        torque.connect_to(&mut module.tau_request_in_msg);
        module
    }

    fn field_msg(magnetic_field_body_t: Vector3<f64>) -> TamSensorBodyMsg {
        TamSensorBodyMsg {
            magnetic_field_body_t,
        }
    }

    fn torque_msg(torque_request_body_nm: Vector3<f64>) -> BodyTorqueCommandMsg {
        BodyTorqueCommandMsg {
            torque_request_body_nm,
        }
    }

    /// `test_torque2Dipole.py::test_torque2Dipole_module`, all four sub-tests,
    /// `accuracy = 1e-12` (1e-3 for the reset check, as upstream). Each
    /// sub-test re-initializes the simulation and runs the single t = 0 tick.
    #[test]
    fn matches_upstream_torque2_dipole_unit_test() {
        let accuracy = 1e-12;
        let field_t = Vector3::new(1e-5, 0.0, 0.0);
        let torque_nm = Vector3::new(10.0 * 1e-3, 20.0 * 1e-3, 30.0 * 1e-3);
        let field = Output::new(field_msg(field_t));
        let torque = Output::new(torque_msg(torque_nm));
        let mut module = connected_module(&field, &torque);

        // TEST 1: a non-zero dipole for a non-zero field and torque.
        module.init();
        module.update(&context(0));
        assert_vector_equal(
            module.dipole_request_out_msg.read().dipole_b_am2,
            upstream_truth(field_t, torque_nm),
            accuracy,
            "dipole_B",
        );

        // TEST 2: zero dipole for a zero field.
        field.write(field_msg(Vector3::zeros()));
        module.init();
        module.update(&context(0));
        assert_vector_equal(
            module.dipole_request_out_msg.read().dipole_b_am2,
            Vector3::zeros(),
            accuracy,
            "dipole_B",
        );

        // TEST 3: zero dipole for a zero torque request.
        field.write(field_msg(field_t));
        torque.write(torque_msg(Vector3::zeros()));
        module.init();
        module.update(&context(0));
        assert_vector_equal(
            module.dipole_request_out_msg.read().dipole_b_am2,
            Vector3::zeros(),
            accuracy,
            "dipole_B",
        );

        // TEST 4: Reset() zeroes the output message.
        torque.write(torque_msg(Vector3::new(1.0, 0.5, 0.7)));
        module.init();
        module.update(&context(0));
        assert_ne!(
            module.dipole_request_out_msg.read().dipole_b_am2,
            Vector3::zeros()
        );
        Module::reset(&mut module, &context(0));
        assert_vector_equal(
            module.dipole_request_out_msg.read().dipole_b_am2,
            Vector3::zeros(),
            1e-3,
            "Reset() zeroed dipole request",
        );
    }

    // Not upstream: the dipole reproduces the torque component perpendicular
    // to the field, `(b × τ) / |b|² × b = τ - (τ·b̂) b̂`. Hand-derived: with
    // b = 2e-5 ẑ and τ = (1, 2, 3) mN m the perpendicular torque is
    // (1, 2, 0) mN m, and μ = (b × τ) / |b|² = (-100, 50, 0) A m^2.
    #[test]
    fn dipole_reproduces_the_torque_perpendicular_to_the_field() {
        let field_t = Vector3::new(0.0, 0.0, 2e-5);
        let torque_nm = Vector3::new(1e-3, 2e-3, 3e-3);
        let field = Output::new(field_msg(field_t));
        let torque = Output::new(torque_msg(torque_nm));
        let mut module = connected_module(&field, &torque);
        module.init();
        module.update(&context(0));

        let dipole = module.dipole_request_out_msg.read().dipole_b_am2;
        assert_vector_equal(dipole, Vector3::new(-100.0, 50.0, 0.0), 1e-9, "dipole_B");
        assert_vector_equal(
            dipole.cross(&field_t),
            Vector3::new(1e-3, 2e-3, 0.0),
            1e-15,
            "produced torque",
        );
    }

    // Not upstream: the field guard is upstream's `|b|^2 > DB0_EPS` (1e-30).
    // |b| = 1e-16 T gives |b|^2 = 1e-32 and a zero request; |b| = 1e-14 T
    // gives 1e-28 and the full (b × τ) / |b|^2.
    #[test]
    fn field_guard_is_the_squared_norm_against_db0_eps() {
        let field = Output::new(field_msg(Vector3::new(1e-16, 0.0, 0.0)));
        let torque = Output::new(torque_msg(Vector3::new(0.0, 0.0, 1.0)));
        let mut module = connected_module(&field, &torque);
        module.init();
        module.update(&context(0));
        assert_eq!(
            module.dipole_request_out_msg.read().dipole_b_am2,
            Vector3::zeros()
        );

        let field_t = Vector3::new(1e-14, 0.0, 0.0);
        field.write(field_msg(field_t));
        module.update(&context(10_000_000));
        let dipole = module.dipole_request_out_msg.read().dipole_b_am2;
        let truth = upstream_truth(field_t, Vector3::new(0.0, 0.0, 1.0));
        assert!((dipole - truth).norm() <= 1e-12 * truth.norm(), "{dipole}");
        assert!((dipole.y + 1e14).abs() <= 1.0, "{dipole}");
    }

    // Not upstream: unconnected inputs read upstream's zeroed payloads, so the
    // request is zero rather than NaN.
    #[test]
    fn unconnected_inputs_give_a_zero_request() {
        let mut module = Torque2Dipole::new(Torque2DipoleConfig::default());
        module.init();
        module.update(&context(0));
        assert_eq!(
            module.dipole_request_out_msg.read().dipole_b_am2,
            Vector3::zeros()
        );
    }
}
