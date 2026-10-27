//! Star-tracker case-to-body conversion: port of upstream Basilisk `stComm`
//! (`STSensorData`).
//!
//! Each update forms the inertial-to-case DCM `[CN]` from the star tracker's
//! Euler parameters, maps it into the body frame with the configured mounting
//! DCM, `[BN] = [BP][CN]` (upstream calls the case frame the platform frame
//! `P`), and publishes `σ_BN` with the sensor's time tag as a
//! [`StarTrackerAttitudeMsg`].

use nalgebra::{Matrix3, Vector4};

use crate::messages::{Input, Output, StarTrackerAttitudeMsg, StarTrackerMsg};
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, ep_to_dcm};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`StComm`].
#[derive(Clone, Debug)]
pub struct StCommConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `dcm_BP`: DCM from the star-tracker platform (case) frame to
    /// the body frame. Upstream stores it row-major as `double[9]`; build it
    /// from that array with [`Matrix3::from_row_slice`]. `Default` keeps
    /// upstream's zero matrix, so it must be set.
    pub dcm_bp: Matrix3<f64>,
}

impl Default for StCommConfig {
    fn default() -> Self {
        Self {
            name: "stComm".to_string(),
            dcm_bp: Matrix3::zeros(),
        }
    }
}

/// Star-tracker attitude mapped into the body frame: upstream `stComm`.
#[derive(Clone, Debug)]
pub struct StComm {
    /// User-settable parameters.
    pub config: StCommConfig,
    /// Upstream `stSensorInMsg`: star-tracker attitude of the case frame.
    /// Unconnected, it reads the identity attitude of
    /// `StarTrackerMsg::default()`, so the output is `σ` of `[BP]`; upstream
    /// reads an all-zero `qInrtl2Case` there and outputs zero.
    pub st_sensor_in_msg: Input<StarTrackerMsg>,
    /// Upstream `stAttOutMsg`: body attitude measured by the star tracker.
    pub st_att_out_msg: Output<StarTrackerAttitudeMsg>,
}

impl StComm {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: StCommConfig) -> Self {
        Self {
            config,
            st_sensor_in_msg: Input::default(),
            st_att_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_stProcessTelem`: the module retains no state.
    fn check_connections(&self) {
        if !self.st_sensor_in_msg.is_connected() {
            log::error!("stComm.st_sensor_in_msg is not connected");
        }
    }

    fn body_attitude(&self, sensor: &StarTrackerMsg) -> StarTrackerAttitudeMsg {
        // Upstream's EP2C on the scalar-first set, not nalgebra's own matrix.
        let dcm_cn = ep_to_dcm(Vector4::from(sensor.q_inrtl2case()));
        StarTrackerAttitudeMsg {
            time_tag_nanos: sensor.time_tag_nanos,
            sigma_bn: dcm_to_mrp(self.config.dcm_bp * dcm_cn),
        }
    }
}

impl Module for StComm {
    fn init(&mut self) {
        self.st_att_out_msg.write(StarTrackerAttitudeMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let sensor = self.st_sensor_in_msg.read();
        self.st_att_out_msg.write(self.body_attitude(&sensor));
    }
}

#[cfg(test)]
mod tests {
    // Upstream `stComm` has no unit test; these follow its documentation, with
    // hand-derived truth: a rotation by `θ` about the unit axis `e` has
    // Euler parameters `(cos θ/2, sin θ/2 e)` and MRP `tan(θ/4) e`.

    use std::f64::consts::{FRAC_PI_2, PI};

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{StComm, StCommConfig};
    use crate::messages::{Output, StarTrackerMsg};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn rotation_ep(angle_rad: f64, axis: Vector3<f64>) -> [f64; 4] {
        let (s, c) = (0.5 * angle_rad).sin_cos();
        [c, s * axis.x, s * axis.y, s * axis.z]
    }

    fn assert_vector_close(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64) {
        assert!(
            (actual - expected).norm() <= accuracy,
            "{actual} != {expected}"
        );
    }

    /// With the case aligned to the body, the output is the case attitude:
    /// `σ_BN = tan(θ/4) e` for a general axis, and the time tag passes
    /// through, every update of a 0.5 s task run to 1 s.
    #[test]
    fn identity_mounting_reports_the_case_attitude() {
        let axis = Vector3::new(1.0, -2.0, 2.0) / 3.0;
        let angle: f64 = 1.2;
        let sensor_output = Output::new(StarTrackerMsg::from_q_inrtl2case(
            1_234_567_890,
            rotation_ep(angle, axis),
        ));
        let mut module = StComm::new(StCommConfig {
            dcm_bp: Matrix3::identity(),
            ..Default::default()
        });
        let mut recorder = MessageRecorder::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&sensor_output, &mut module.st_sensor_in_msg);
        simulation.connect(&module.st_att_out_msg, &mut recorder.input_msg);
        simulation.add_module("stComm", &mut module, 500_000_000, 0);
        simulation.add_module("recorder", &mut recorder, 500_000_000, 0);
        simulation.run_for(1_000_000_000);
        drop(simulation);

        assert_eq!(recorder.samples.len(), 3);
        for sample in &recorder.samples {
            assert_eq!(sample.time_tag_nanos, 1_234_567_890);
            assert_vector_close(sample.sigma_bn, (0.25 * angle).tan() * axis, 1e-14);
        }
    }

    /// A tracker mounted rotated by -90 deg about the third axis (`[BP]` is
    /// `M3(-90 deg)`, row-major as upstream) cancels a +90 deg case attitude
    /// about the same axis; with the case aligned to inertial, the output is
    /// the mounting rotation itself, here +90 deg about the first axis.
    #[test]
    fn mounting_dcm_is_applied_before_the_mrp_conversion() {
        let sensor_output = Output::new(StarTrackerMsg::from_q_inrtl2case(
            0,
            rotation_ep(FRAC_PI_2, Vector3::z()),
        ));
        let mut module = StComm::new(StCommConfig::default());
        // Config set after `new` takes effect.
        module.config.dcm_bp = Matrix3::from_row_slice(&[
            0.0, -1.0, 0.0, //
            1.0, 0.0, 0.0, //
            0.0, 0.0, 1.0,
        ]);
        sensor_output.connect_to(&mut module.st_sensor_in_msg);
        module.init();
        module.update(&context(0));
        assert_vector_close(
            module.st_att_out_msg.read().sigma_bn,
            Vector3::zeros(),
            1e-15,
        );

        sensor_output.write(StarTrackerMsg::from_q_inrtl2case(42, [1.0, 0.0, 0.0, 0.0]));
        module.config.dcm_bp = Matrix3::from_row_slice(&[
            1.0, 0.0, 0.0, //
            0.0, 0.0, 1.0, //
            0.0, -1.0, 0.0,
        ]);
        module.update(&context(500_000_000));
        let output = module.st_att_out_msg.read();
        assert_eq!(output.time_tag_nanos, 42);
        assert_vector_close(output.sigma_bn, (PI / 8.0).tan() * Vector3::x(), 1e-15);
    }

    /// Upstream's EP2C is quadratic in the Euler parameters, so the sign of
    /// the set the message hands back does not matter: a 270 deg rotation
    /// gives the short-way MRP of -90 deg.
    #[test]
    fn long_rotations_give_the_short_way_mrp() {
        let sensor_output = Output::new(StarTrackerMsg::from_q_inrtl2case(
            0,
            rotation_ep(1.5 * PI, Vector3::y()),
        ));
        let mut module = StComm::new(StCommConfig {
            dcm_bp: Matrix3::identity(),
            ..Default::default()
        });
        sensor_output.connect_to(&mut module.st_sensor_in_msg);
        module.init();
        module.update(&context(0));
        assert_vector_close(
            module.st_att_out_msg.read().sigma_bn,
            (-FRAC_PI_2 / 4.0).tan() * Vector3::y(),
            1e-15,
        );
    }

    /// Non-commuting factors fix the product order: `[CN] = M3(90 deg)` and
    /// `[BP] = M1(90 deg)` give the permutation `[BN] = [BP][CN]` with rows
    /// `(0 1 0), (0 0 1), (1 0 0)`, a 120 deg turn about `(1, 1, 1)/√3`, so
    /// `σ_BN = tan(30 deg) (1, 1, 1)/√3 = (1, 1, 1)/3`. The reversed product
    /// `[CN][BP]` would give `(1, -1, 1)/3`.
    #[test]
    fn mounting_dcm_premultiplies_the_case_attitude() {
        let sensor_output = Output::new(StarTrackerMsg::from_q_inrtl2case(
            0,
            rotation_ep(FRAC_PI_2, Vector3::z()),
        ));
        let mut module = StComm::new(StCommConfig {
            dcm_bp: Matrix3::from_row_slice(&[
                1.0, 0.0, 0.0, //
                0.0, 0.0, 1.0, //
                0.0, -1.0, 0.0,
            ]),
            ..Default::default()
        });
        sensor_output.connect_to(&mut module.st_sensor_in_msg);
        module.init();
        module.update(&context(0));
        assert_vector_close(
            module.st_att_out_msg.read().sigma_bn,
            Vector3::repeat(1.0 / 3.0),
            1e-15,
        );
    }

    /// An unconnected input reads `StarTrackerMsg::default()`, the identity
    /// attitude, so the output is the mounting attitude itself: here
    /// `[BP] = M3(-90 deg)` gives `σ = -tan(22.5 deg) z`. Upstream reads an
    /// all-zero `qInrtl2Case` instead, whose EP2C is the zero matrix, and
    /// outputs zero (checked against the compiled upstream C).
    #[test]
    fn unconnected_input_reads_the_identity_attitude() {
        let mut module = StComm::new(StCommConfig {
            dcm_bp: Matrix3::from_row_slice(&[
                0.0, -1.0, 0.0, //
                1.0, 0.0, 0.0, //
                0.0, 0.0, 1.0,
            ]),
            ..Default::default()
        });
        module.init();
        module.update(&context(0));
        let output = module.st_att_out_msg.read();
        assert_eq!(output.time_tag_nanos, 0);
        assert_vector_close(output.sigma_bn, -(PI / 8.0).tan() * Vector3::z(), 1e-15);
    }
}
