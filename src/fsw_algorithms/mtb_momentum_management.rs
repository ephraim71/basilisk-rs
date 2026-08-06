use nalgebra::{DMatrix, DVector, Matrix3, SymmetricEigen, Vector3};

use crate::messages::{
    ArrayMotorTorqueMsg, Input, MAX_EFF_COUNT, MagneticDipoleCommandMsg, MotorTorqueMsg,
    MtbArrayCommandMsg, MtbArrayConfigMsg, Output, RwArrayConfigMsg, RwSpeedMsg, TamSensorBodyMsg,
};
use crate::{Module, SimulationContext};

const SVD_SINGULAR_VALUE_CUTOFF: f64 = 1.0e-11;
const MATRIX_INVERSE_DETERMINANT_CUTOFF: f64 = 1.0e-30;

#[derive(Clone, Debug)]
pub struct MtbMomentumManagementConfig {
    pub name: String,
    /// Desired reaction-wheel spin rates.
    pub wheel_speed_biases_radps: [f64; MAX_EFF_COUNT],
    /// Reaction-wheel momentum feedback gain.
    pub c_gain_per_sec: f64,
}

#[derive(Clone, Debug)]
pub struct MtbMomentumManagement {
    pub config: MtbMomentumManagementConfig,

    pub rw_params_in_msg: Input<RwArrayConfigMsg>,
    pub mtb_params_in_msg: Input<MtbArrayConfigMsg>,
    pub tam_sensor_body_in_msg: Input<TamSensorBodyMsg>,
    pub rw_speeds_in_msg: Input<RwSpeedMsg>,
    pub rw_motor_torque_in_msg: Input<ArrayMotorTorqueMsg>,

    pub mtb_cmd_out_msg: Output<MtbArrayCommandMsg>,
    pub mtb_cmd_out_msgs: Vec<Output<MagneticDipoleCommandMsg>>,
    pub rw_motor_torque_out_msg: Output<ArrayMotorTorqueMsg>,
    pub rw_motor_torque_out_msgs: Vec<Output<MotorTorqueMsg>>,

    /// RW configuration cached during initialization, as in upstream Reset().
    pub rw_config_params: RwArrayConfigMsg,
    /// MTB configuration cached during initialization, as in upstream Reset().
    pub mtb_config_params: MtbArrayConfigMsg,

    pub tau_desired_mtb_body_nm: Vector3<f64>,
    pub tau_desired_rw_body_nm: Vector3<f64>,
    pub h_delta_wheels_wheel_nms: [f64; MAX_EFF_COUNT],
    pub h_delta_wheels_body_nms: Vector3<f64>,
    pub tau_desired_rw_wheel_nm: [f64; MAX_EFF_COUNT],
    pub tau_ideal_rw_wheel_nm: [f64; MAX_EFF_COUNT],
    /// Upstream declares this diagnostic at MAX_EFF_CNT but writes its first three entries.
    pub tau_ideal_rw_body_nm: [f64; MAX_EFF_COUNT],
    pub wheel_speed_error_radps: [f64; MAX_EFF_COUNT],
}

impl MtbMomentumManagement {
    pub fn new(config: MtbMomentumManagementConfig) -> Self {
        Self {
            config,
            rw_params_in_msg: Input::default(),
            mtb_params_in_msg: Input::default(),
            tam_sensor_body_in_msg: Input::default(),
            rw_speeds_in_msg: Input::default(),
            rw_motor_torque_in_msg: Input::default(),
            mtb_cmd_out_msg: Output::default(),
            mtb_cmd_out_msgs: Vec::new(),
            rw_motor_torque_out_msg: Output::default(),
            rw_motor_torque_out_msgs: Vec::new(),
            rw_config_params: RwArrayConfigMsg::default(),
            mtb_config_params: MtbArrayConfigMsg::default(),
            tau_desired_mtb_body_nm: Vector3::zeros(),
            tau_desired_rw_body_nm: Vector3::zeros(),
            h_delta_wheels_wheel_nms: [0.0; MAX_EFF_COUNT],
            h_delta_wheels_body_nms: Vector3::zeros(),
            tau_desired_rw_wheel_nm: [0.0; MAX_EFF_COUNT],
            tau_ideal_rw_wheel_nm: [0.0; MAX_EFF_COUNT],
            tau_ideal_rw_body_nm: [0.0; MAX_EFF_COUNT],
            wheel_speed_error_radps: [0.0; MAX_EFF_COUNT],
        }
    }

    /// Registers one independently connectable magnetic-dipole command output.
    pub fn add_mtb_command_output(&mut self) -> &Output<MagneticDipoleCommandMsg> {
        assert!(
            self.mtb_cmd_out_msgs.len() < MAX_EFF_COUNT,
            "at most {MAX_EFF_COUNT} MTB command outputs are supported"
        );
        self.mtb_cmd_out_msgs.push(Output::default());
        self.mtb_cmd_out_msgs
            .last()
            .expect("registered MTB command output should be available")
    }

    /// Registers one independently connectable motor-torque command output.
    pub fn add_rw_motor_torque_output(&mut self) -> &Output<MotorTorqueMsg> {
        assert!(
            self.rw_motor_torque_out_msgs.len() < MAX_EFF_COUNT,
            "at most {MAX_EFF_COUNT} reaction-wheel command outputs are supported"
        );
        self.rw_motor_torque_out_msgs.push(Output::default());
        self.rw_motor_torque_out_msgs
            .last()
            .expect("registered reaction-wheel command output should be available")
    }

    /// Check connectivity and cache the two static array-configuration messages.
    pub fn reset(&mut self, _call_time_nanos: u64) {
        if !self.rw_params_in_msg.is_connected() {
            log::error!("mtbMomentumManagement.rw_params_in_msg is not connected");
        }
        if !self.mtb_params_in_msg.is_connected() {
            log::error!("mtbMomentumManagement.mtb_params_in_msg is not connected");
        }
        if !self.tam_sensor_body_in_msg.is_connected() {
            log::error!("mtbMomentumManagement.tam_sensor_body_in_msg is not connected");
        }
        if !self.rw_speeds_in_msg.is_connected() {
            log::error!("mtbMomentumManagement.rw_speeds_in_msg is not connected");
        }
        if !self.rw_motor_torque_in_msg.is_connected() {
            log::error!("mtbMomentumManagement.rw_motor_torque_in_msg is not connected");
        }

        self.rw_config_params = self.rw_params_in_msg.read();
        self.mtb_config_params = self.mtb_params_in_msg.read();
        if self.rw_config_params.num_rw > MAX_EFF_COUNT {
            log::error!(
                "mtbMomentumManagement.num_rw exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); truncating"
            );
        }
        if self.mtb_config_params.num_mtb > MAX_EFF_COUNT {
            log::error!(
                "mtbMomentumManagement.num_mtb exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); truncating"
            );
        }
    }

    fn active_rw_count(&self) -> usize {
        self.rw_config_params.num_rw.min(MAX_EFF_COUNT)
    }

    fn active_mtb_count(&self) -> usize {
        self.mtb_config_params.num_mtb.min(MAX_EFF_COUNT)
    }

    fn clear_diagnostics(&mut self, num_rw: usize) {
        self.tau_desired_mtb_body_nm = Vector3::zeros();
        self.tau_desired_rw_body_nm = Vector3::zeros();
        self.h_delta_wheels_wheel_nms[..num_rw].fill(0.0);
        self.h_delta_wheels_body_nms = Vector3::zeros();
        self.tau_desired_rw_wheel_nm[..num_rw].fill(0.0);
        self.tau_ideal_rw_wheel_nm[..num_rw].fill(0.0);
        self.tau_ideal_rw_body_nm[..3].fill(0.0);
        self.wheel_speed_error_radps[..num_rw].fill(0.0);
    }

    fn update_control(&mut self) -> (MtbArrayCommandMsg, ArrayMotorTorqueMsg) {
        let num_rw = self.active_rw_count();
        let num_mtb = self.active_mtb_count();
        self.clear_diagnostics(num_rw);

        let gs = matrix_from_body_axes(&self.rw_config_params.spin_axes_body, num_rw);
        let gt = matrix_from_body_axes(&self.mtb_config_params.dipole_axes_body, num_mtb);

        let tam_sensor_body = self.tam_sensor_body_in_msg.read();
        let rw_speeds = self.rw_speeds_in_msg.read();
        let rw_motor_torque_input = self.rw_motor_torque_in_msg.read();

        let mut wheel_speed_error = DVector::zeros(num_rw);
        let mut h_delta_wheels = DVector::zeros(num_rw);
        for wheel in 0..num_rw {
            wheel_speed_error[wheel] = value_or_zero(&rw_speeds.wheel_speeds_radps, wheel)
                - value_or_zero(&self.config.wheel_speed_biases_radps, wheel);
            h_delta_wheels[wheel] =
                value_or_zero(&self.rw_config_params.spin_axis_inertias_kg_m2, wheel)
                    * wheel_speed_error[wheel];
            self.wheel_speed_error_radps[wheel] = wheel_speed_error[wheel];
            self.h_delta_wheels_wheel_nms[wheel] = h_delta_wheels[wheel];
        }

        let h_delta_wheels_body = &gs * &h_delta_wheels;
        self.h_delta_wheels_body_nms = vector3_from_dynamic(&h_delta_wheels_body);

        let tau_ideal_rw_wheel = -self.config.c_gain_per_sec * h_delta_wheels;
        let tau_ideal_rw_body = &gs * &tau_ideal_rw_wheel;
        self.tau_ideal_rw_body_nm[..3]
            .copy_from_slice(vector3_from_dynamic(&tau_ideal_rw_body).as_slice());
        copy_active(&tau_ideal_rw_wheel, &mut self.tau_ideal_rw_wheel_nm, num_rw);

        let b_tilde = tilde(tam_sensor_body.magnetic_field_body_t);
        let b_gt = dynamic_matrix3(&b_tilde) * gt;
        let mut mtb_dipole_commands = if num_mtb == 0 {
            DVector::zeros(0)
        } else {
            -pseudo_inverse(&b_gt, SVD_SINGULAR_VALUE_CUTOFF) * &tau_ideal_rw_body
        };

        for mtb in 0..num_mtb {
            let maximum = value_or_zero(&self.mtb_config_params.max_dipoles_am2, mtb);
            if mtb_dipole_commands[mtb] > maximum {
                mtb_dipole_commands[mtb] = maximum;
            }
            if mtb_dipole_commands[mtb] < -maximum {
                mtb_dipole_commands[mtb] = -maximum;
            }
        }

        let tau_desired_mtb_body = -(&b_gt * &mtb_dipole_commands);
        self.tau_desired_mtb_body_nm = vector3_from_dynamic(&tau_desired_mtb_body);

        let u_delta_body = &tau_desired_mtb_body - &tau_ideal_rw_body;
        let gs_pseudo_inverse = minimum_norm_inverse(&gs);
        let u_delta_wheel = gs_pseudo_inverse * u_delta_body;
        let tau_desired_rw_wheel = &tau_ideal_rw_wheel + u_delta_wheel;
        let tau_desired_rw_body = -(&gs * &tau_desired_rw_wheel);

        self.tau_desired_rw_body_nm = vector3_from_dynamic(&tau_desired_rw_body);
        copy_active(
            &tau_desired_rw_wheel,
            &mut self.tau_desired_rw_wheel_nm,
            num_rw,
        );

        let mut mtb_output = MtbArrayCommandMsg::default();
        for mtb in 0..num_mtb {
            mtb_output.dipole_cmds_am2[mtb] = mtb_dipole_commands[mtb];
        }

        let mut rw_output = ArrayMotorTorqueMsg::default();
        for wheel in 0..MAX_EFF_COUNT {
            rw_output.motor_torque_nm[wheel] =
                value_or_zero(&rw_motor_torque_input.motor_torque_nm, wheel);
        }
        for wheel in 0..num_rw {
            rw_output.motor_torque_nm[wheel] += tau_desired_rw_wheel[wheel];
        }

        (mtb_output, rw_output)
    }
}

impl Module for MtbMomentumManagement {
    fn init(&mut self) {
        self.mtb_cmd_out_msg.write(MtbArrayCommandMsg::default());
        for output in &self.mtb_cmd_out_msgs {
            output.write(MagneticDipoleCommandMsg::default());
        }
        self.rw_motor_torque_out_msg
            .write(ArrayMotorTorqueMsg::default());
        for output in &self.rw_motor_torque_out_msgs {
            output.write(MotorTorqueMsg::default());
        }
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        MtbMomentumManagement::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, _context: &SimulationContext) {
        let (mtb_output, rw_output) = self.update_control();
        for (index, output) in self.mtb_cmd_out_msgs.iter().enumerate() {
            output.write(MagneticDipoleCommandMsg {
                dipole_moment_am2: mtb_output.dipole_cmds_am2[index],
            });
        }
        for (index, output) in self.rw_motor_torque_out_msgs.iter().enumerate() {
            output.write(MotorTorqueMsg {
                motor_torque_nm: rw_output.motor_torque_nm[index],
            });
        }
        self.mtb_cmd_out_msg.write(mtb_output);
        self.rw_motor_torque_out_msg.write(rw_output);
    }
}

fn matrix_from_body_axes(axes: &[Vector3<f64>], count: usize) -> DMatrix<f64> {
    DMatrix::from_fn(3, count, |row, column| {
        axes.get(column).map_or(0.0, |axis| axis[row])
    })
}

fn dynamic_matrix3(matrix: &Matrix3<f64>) -> DMatrix<f64> {
    DMatrix::from_fn(3, 3, |row, column| matrix[(row, column)])
}

fn vector3_from_dynamic(vector: &DVector<f64>) -> Vector3<f64> {
    Vector3::new(vector[0], vector[1], vector[2])
}

fn value_or_zero(values: &[f64], index: usize) -> f64 {
    values.get(index).copied().unwrap_or(0.0)
}

fn copy_active(source: &DVector<f64>, destination: &mut [f64], count: usize) {
    for index in 0..count {
        destination[index] = source[index];
    }
}

/// Returns the skew-symmetric matrix such that `tilde(v) * x == v.cross(x)`.
fn tilde(vector: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -vector.z, vector.y, //
        vector.z, 0.0, -vector.x, //
        -vector.y, vector.x, 0.0,
    )
}

/// Moore-Penrose pseudo-inverse of a (generally rank-deficient) matrix, computed
/// as `M^T (M M^T)^+`. The symmetric factor `M M^T` is diagonalized with a
/// symmetric eigensolver, which stays accurate for the rank-2 `tilde(B) * Gt`
/// matrices this law inverts. nalgebra's general SVD does not: for some field
/// geometries it returns a decomposition with a Moore-Penrose residual of ~1e-7,
/// injecting a spurious dipole-command spike. Modes whose singular value (the
/// square root of a Gram eigenvalue) is below `cutoff` are dropped.
fn pseudo_inverse(matrix: &DMatrix<f64>, cutoff: f64) -> DMatrix<f64> {
    let transpose = matrix.transpose();
    let gram = matrix * &transpose;
    let dimension = gram.nrows();
    let eigen = SymmetricEigen::new(gram);

    let mut gram_pseudo_inverse = DMatrix::zeros(dimension, dimension);
    for index in 0..dimension {
        let eigenvalue = eigen.eigenvalues[index];
        if eigenvalue > 0.0 && eigenvalue.sqrt() >= cutoff {
            let eigenvector = eigen.eigenvectors.column(index);
            gram_pseudo_inverse += (&eigenvector * eigenvector.transpose()) / eigenvalue;
        }
    }

    transpose * gram_pseudo_inverse
}

fn minimum_norm_inverse(matrix: &DMatrix<f64>) -> DMatrix<f64> {
    let transpose = matrix.transpose();
    let gram = matrix * &transpose;
    let determinant = gram.determinant();

    if determinant.is_nan() || determinant.abs() <= MATRIX_INVERSE_DETERMINANT_CUTOFF {
        log::error!("mtbMomentumManagement cannot invert singular Gs * Gs^T matrix");
        return DMatrix::zeros(matrix.ncols(), matrix.nrows());
    }

    match gram.try_inverse() {
        Some(inverse) => transpose * inverse,
        None => {
            log::error!("mtbMomentumManagement cannot invert Gs * Gs^T matrix");
            DMatrix::zeros(matrix.ncols(), matrix.nrows())
        }
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{DMatrix, Vector3};

    use crate::messages::{
        ArrayMotorTorqueMsg, MAX_EFF_COUNT, MtbArrayConfigMsg, Output, RwArrayConfigMsg,
        RwSpeedMsg, TamSensorBodyMsg,
    };
    use crate::{Module, SimulationContext};

    use super::{
        MtbMomentumManagement, MtbMomentumManagementConfig, SVD_SINGULAR_VALUE_CUTOFF,
        pseudo_inverse,
    };

    struct Fixture {
        module: MtbMomentumManagement,
        rw_params: Output<RwArrayConfigMsg>,
        mtb_params: Output<MtbArrayConfigMsg>,
    }

    impl Fixture {
        fn new(
            config: MtbMomentumManagementConfig,
            rw_params: RwArrayConfigMsg,
            mtb_params: MtbArrayConfigMsg,
            tam_body: TamSensorBodyMsg,
            rw_speeds: RwSpeedMsg,
            rw_motor_torque: ArrayMotorTorqueMsg,
        ) -> Self {
            let rw_params = Output::new(rw_params);
            let mtb_params = Output::new(mtb_params);
            let tam_body = Output::new(tam_body);
            let rw_speeds = Output::new(rw_speeds);
            let rw_motor_torque = Output::new(rw_motor_torque);
            let mut module = MtbMomentumManagement::new(config);
            module.rw_params_in_msg.connect(rw_params.slot());
            module.mtb_params_in_msg.connect(mtb_params.slot());
            module.tam_sensor_body_in_msg.connect(tam_body.slot());
            module.rw_speeds_in_msg.connect(rw_speeds.slot());
            module
                .rw_motor_torque_in_msg
                .connect(rw_motor_torque.slot());

            Self {
                module,
                rw_params,
                mtb_params,
            }
        }
    }

    fn context() -> SimulationContext {
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn upstream_fixture(magnetic_field_body_t: Vector3<f64>) -> Fixture {
        let beta = 45.0_f64.to_radians();
        let cosine = beta.cos();
        let sine = beta.sin();

        let mut rw_params = RwArrayConfigMsg {
            num_rw: 4,
            ..Default::default()
        };
        rw_params.spin_axes_body[..4].copy_from_slice(&[
            Vector3::new(0.0, cosine, sine),
            Vector3::new(0.0, sine, -cosine),
            Vector3::new(cosine, -sine, 0.0),
            Vector3::new(-cosine, -sine, 0.0),
        ]);
        rw_params.spin_axis_inertias_kg_m2[..4].fill(0.002);

        let mut mtb_params = MtbArrayConfigMsg {
            num_mtb: 3,
            ..Default::default()
        };
        mtb_params.dipole_axes_body[..3].copy_from_slice(&[
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
        ]);
        mtb_params.max_dipoles_am2[..3].fill(10.0);

        let mut rw_speeds = RwSpeedMsg::default();
        rw_speeds.wheel_speeds_radps[..4].copy_from_slice(&[100.0, 200.0, 300.0, 400.0]);

        Fixture::new(
            MtbMomentumManagementConfig {
                name: "mtbMomentumManagement".to_string(),
                wheel_speed_biases_radps: [0.0; MAX_EFF_COUNT],
                c_gain_per_sec: 0.005,
            },
            rw_params,
            mtb_params,
            TamSensorBodyMsg {
                magnetic_field_body_t,
            },
            rw_speeds,
            ArrayMotorTorqueMsg::default(),
        )
    }

    #[test]
    fn matches_upstream_nonzero_field_functional_case() {
        let mut fixture =
            upstream_fixture(5.0e3 * Vector3::new(0.037_823_47, 0.491_705_16, -0.869_939_9));
        fixture.module.init();
        fixture.module.update(&context());

        let mtb_output = fixture.module.mtb_cmd_out_msg.read();
        let rw_output = fixture.module.rw_motor_torque_out_msg.read();
        assert!(DMatrix::from_column_slice(3, 1, &mtb_output.dipole_cmds_am2[..3]).norm() > 1.0e-8);
        assert!(DMatrix::from_column_slice(4, 1, &rw_output.motor_torque_nm[..4]).norm() > 1.0e-8);
    }

    #[test]
    fn matches_upstream_zero_field_functional_case() {
        let mut fixture = upstream_fixture(Vector3::zeros());
        fixture.module.init();
        fixture.module.update(&context());

        let mtb_output = fixture.module.mtb_cmd_out_msg.read();
        let rw_output = fixture.module.rw_motor_torque_out_msg.read();
        assert!(
            DMatrix::from_column_slice(3, 1, &mtb_output.dipole_cmds_am2[..3]).norm() <= 1.0e-8
        );

        let axes = &fixture.module.rw_config_params.spin_axes_body;
        let gs = DMatrix::from_row_slice(
            3,
            4,
            &[
                axes[0].x, axes[1].x, axes[2].x, axes[3].x, axes[0].y, axes[1].y, axes[2].y,
                axes[3].y, axes[0].z, axes[1].z, axes[2].z, axes[3].z,
            ],
        );
        let wheel_torque = DMatrix::from_column_slice(4, 1, &rw_output.motor_torque_nm[..4]);
        assert!((gs * wheel_torque).norm() <= 1.0e-8);
    }

    #[test]
    fn reproduces_upstream_fixture_numeric_solution() {
        let mut fixture =
            upstream_fixture(5.0e3 * Vector3::new(0.037_823_47, 0.491_705_16, -0.869_939_9));
        fixture.module.init();
        fixture.module.update(&context());

        let expected_dipoles = Vector3::new(
            5.616_499_276_285_865e-7,
            -1.283_771_257_898_261_6e-7,
            -4.814_142_446_928_183e-8,
        );
        let actual_dipoles =
            Vector3::from_column_slice(&fixture.module.mtb_cmd_out_msg.read().dipole_cmds_am2[..3]);
        assert!((actual_dipoles - expected_dipoles).norm() <= 1.0e-12);

        let expected_wheel_torques = [
            -0.000_645_922_748_433_26,
            -0.002_633_047_206_658_91,
            -0.002_881_974_247_586_58,
            -0.003_839_055_797_321_26,
        ];
        let output = fixture.module.rw_motor_torque_out_msg.read();
        for (actual, expected) in output.motor_torque_nm[..4]
            .iter()
            .zip(expected_wheel_torques)
        {
            assert!((actual - expected).abs() <= 1.0e-12);
        }
    }

    #[test]
    fn saturates_dipoles_and_preserves_inactive_wheel_commands() {
        let mut rw_params = RwArrayConfigMsg {
            num_rw: 3,
            ..Default::default()
        };
        rw_params.spin_axes_body[..3].copy_from_slice(&[Vector3::x(), Vector3::y(), Vector3::z()]);
        rw_params.spin_axis_inertias_kg_m2[..3].fill(1.0);

        let mut mtb_params = MtbArrayConfigMsg {
            num_mtb: 3,
            ..Default::default()
        };
        mtb_params.dipole_axes_body[..3].copy_from_slice(&[
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
        ]);
        mtb_params.max_dipoles_am2[..3].fill(0.25);

        let mut speeds = RwSpeedMsg::default();
        speeds.wheel_speeds_radps[1] = 10.0;
        let mut baseline_torque = ArrayMotorTorqueMsg::default();
        baseline_torque.motor_torque_nm[0] = 1.0;
        baseline_torque.motor_torque_nm[10] = 42.0;
        baseline_torque.motor_torque_nm[35] = 99.0;

        let mut fixture = Fixture::new(
            MtbMomentumManagementConfig {
                name: "mtbMomentumManagement".to_string(),
                wheel_speed_biases_radps: [0.0; MAX_EFF_COUNT],
                c_gain_per_sec: 1.0,
            },
            rw_params,
            mtb_params,
            TamSensorBodyMsg {
                magnetic_field_body_t: Vector3::z(),
            },
            speeds,
            baseline_torque,
        );
        fixture.module.init();
        fixture.module.update(&context());

        let mtb_output = fixture.module.mtb_cmd_out_msg.read();
        let rw_output = fixture.module.rw_motor_torque_out_msg.read();
        assert!((mtb_output.dipole_cmds_am2[0] - 0.25).abs() <= 1.0e-12);
        assert!(mtb_output.dipole_cmds_am2[1].abs() <= 1.0e-12);
        assert!(mtb_output.dipole_cmds_am2[2].abs() <= 1.0e-12);
        assert!(
            mtb_output.dipole_cmds_am2[3..]
                .iter()
                .all(|value| *value == 0.0)
        );
        assert!(
            (rw_output.motor_torque_nm[0] - (1.0 + fixture.module.tau_desired_rw_wheel_nm[0]))
                .abs()
                <= 1.0e-12
        );
        assert_eq!(rw_output.motor_torque_nm[10], 42.0);
        assert_eq!(rw_output.motor_torque_nm[35], 99.0);
        assert_eq!(rw_output.motor_torque_nm.len(), MAX_EFF_COUNT);
        assert_eq!(mtb_output.dipole_cmds_am2.len(), MAX_EFF_COUNT);
    }

    #[test]
    fn configuration_messages_are_cached_during_init() {
        let mut fixture = upstream_fixture(Vector3::zeros());
        fixture.module.init();

        fixture.rw_params.write(RwArrayConfigMsg::default());
        fixture.mtb_params.write(MtbArrayConfigMsg::default());
        fixture.module.update(&context());

        assert_eq!(fixture.module.rw_config_params.num_rw, 4);
        assert_eq!(fixture.module.mtb_config_params.num_mtb, 3);
        assert!(
            fixture.module.tau_ideal_rw_wheel_nm[..4]
                .iter()
                .any(|value| *value != 0.0)
        );

        fixture.module.reset(0);
        assert_eq!(fixture.module.rw_config_params.num_rw, 0);
        assert_eq!(fixture.module.mtb_config_params.num_mtb, 0);
    }

    #[test]
    fn singular_wheel_geometry_uses_upstream_zero_inverse_fallback() {
        let mut rw_params = RwArrayConfigMsg {
            num_rw: 1,
            ..Default::default()
        };
        rw_params.spin_axes_body[0] = Vector3::x();
        rw_params.spin_axis_inertias_kg_m2[0] = 1.0;
        let mut speeds = RwSpeedMsg::default();
        speeds.wheel_speeds_radps[0] = 2.0;

        let mut fixture = Fixture::new(
            MtbMomentumManagementConfig {
                name: "mtbMomentumManagement".to_string(),
                wheel_speed_biases_radps: [0.0; MAX_EFF_COUNT],
                c_gain_per_sec: 0.5,
            },
            rw_params,
            MtbArrayConfigMsg::default(),
            TamSensorBodyMsg::default(),
            speeds,
            ArrayMotorTorqueMsg::default(),
        );
        fixture.module.init();
        fixture.module.update(&context());

        assert_eq!(fixture.module.tau_desired_rw_wheel_nm[0], -1.0);
        assert_eq!(
            fixture
                .module
                .rw_motor_torque_out_msg
                .read()
                .motor_torque_nm[0],
            -1.0
        );
    }

    #[test]
    fn svd_cutoff_is_absolute_and_inclusive_like_upstream() {
        let matrix = DMatrix::from_diagonal(&nalgebra::DVector::from_vec(vec![
            SVD_SINGULAR_VALUE_CUTOFF,
            0.5 * SVD_SINGULAR_VALUE_CUTOFF,
            2.0,
        ]));
        let inverse = pseudo_inverse(&matrix, SVD_SINGULAR_VALUE_CUTOFF);

        assert!((inverse[(0, 0)] - 1.0 / SVD_SINGULAR_VALUE_CUTOFF).abs() <= 1.0e-3);
        assert_eq!(inverse[(1, 1)], 0.0);
        assert!((inverse[(2, 2)] - 0.5).abs() <= f64::EPSILON);
    }

    #[test]
    fn computes_non_square_rank_deficient_svd_pseudo_inverse() {
        let matrix = DMatrix::from_row_slice(
            3,
            4,
            &[1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        );
        let expected = DMatrix::from_row_slice(
            4,
            3,
            &[0.5, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0],
        );
        let actual = pseudo_inverse(&matrix, SVD_SINGULAR_VALUE_CUTOFF);

        assert!((actual - expected).norm() <= 1.0e-12);
    }

    // Regression: `tilde(B) * Gt` is rank-2. For this field geometry (captured
    // mid-run from the MTB momentum-management parity scenario) nalgebra's general
    // SVD returned a pseudo-inverse with a Moore-Penrose residual of ~1.6e-7,
    // producing a one-step dipole-command spike that diverged from the C++
    // reference. The Gram/eigen pseudo-inverse must stay accurate here.
    #[test]
    fn pseudo_inverse_is_accurate_for_rank2_field_geometry() {
        let b_gt = DMatrix::from_row_slice(
            3,
            4,
            &[
                0.0,
                1.06879128500276908e-05,
                -2.72834298179075497e-05,
                7.55749564030370400e-06,
                -1.06879128500276908e-05,
                0.0,
                -3.15802641687453088e-05,
                -7.55749564030370400e-06,
                2.72834298179075497e-05,
                3.15802641687453088e-05,
                0.0,
                4.16229171138074661e-05,
            ],
        );
        let pinv = pseudo_inverse(&b_gt, SVD_SINGULAR_VALUE_CUTOFF);
        // Moore-Penrose condition M P M == M.
        let residual = (&b_gt * &pinv * &b_gt - &b_gt).norm();
        assert!(
            residual <= 1.0e-15,
            "pseudo-inverse Moore-Penrose residual too large: {residual:.3e}"
        );
    }
}
