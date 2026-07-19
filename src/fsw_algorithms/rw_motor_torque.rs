use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    ArrayMotorTorqueMsg, BodyTorqueCommandMsg, Input, Output, RwArrayConfigMsg, RwAvailabilityMsg,
    MAX_EFF_COUNT,
};
use crate::{Module, SimulationContext};

const INVERSE_DETERMINANT_EPSILON: f64 = 1.0e-30;

#[derive(Clone, Debug)]
pub struct RwMotorTorqueConfig {
    pub name: String,
    /// Up to three leading, nonzero body-frame control axes. Axes are used verbatim.
    pub control_axes_body: Vec<Vector3<f64>>,
}

#[derive(Clone, Debug)]
pub struct RwMotorTorque {
    pub config: RwMotorTorqueConfig,
    pub veh_control_in_msg: Input<BodyTorqueCommandMsg>,
    pub rw_params_in_msg: Input<RwArrayConfigMsg>,
    pub rw_avail_in_msg: Input<RwAvailabilityMsg>,
    pub rw_motor_torque_out_msg: Output<ArrayMotorTorqueMsg>,
    pub num_control_axes: usize,
    pub num_available_rw: usize,
    rw_config: RwArrayConfigMsg,
    available_spin_axes_body: Vec<Vector3<f64>>,
}

impl RwMotorTorque {
    pub fn new(config: RwMotorTorqueConfig) -> Self {
        Self {
            config,
            veh_control_in_msg: Input::default(),
            rw_params_in_msg: Input::default(),
            rw_avail_in_msg: Input::default(),
            rw_motor_torque_out_msg: Output::default(),
            num_control_axes: 0,
            num_available_rw: 0,
            rw_config: RwArrayConfigMsg::default(),
            available_spin_axes_body: Vec::new(),
        }
    }

    /// Reproduce the upstream Reset routine without conflating it with output self-initialization.
    pub fn reset(&mut self, _call_time_nanos: u64) {
        self.num_control_axes = self
            .config
            .control_axes_body
            .iter()
            .take(3)
            .take_while(|axis| axis.norm() > 0.0)
            .count();
        if self.num_control_axes == 0 {
            log::info!("rwMotorTorque is not configured to control any axes");
        }
        if !self.rw_params_in_msg.is_connected() {
            log::error!("rwMotorTorque.rw_params_in_msg is not connected");
        }
        self.rw_config = self.rw_params_in_msg.read();
        if self.rw_config.num_rw > MAX_EFF_COUNT {
            log::error!("rwMotorTorque.num_rw exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); truncating");
        }

        if !self.rw_avail_in_msg.is_connected() {
            let num_rw = self.rw_config.num_rw.min(MAX_EFF_COUNT);
            self.available_spin_axes_body = (0..num_rw)
                .map(|index| self.rw_config.spin_axes_body[index])
                .collect();
            self.num_available_rw = num_rw;
        }
    }

    fn compute_wheel_torques(&mut self) -> ArrayMotorTorqueMsg {
        let mut output = ArrayMotorTorqueMsg::default();
        if !self.veh_control_in_msg.is_connected() {
            log::error!("rwMotorTorque.veh_control_in_msg is not connected");
        }
        let requested_body_torque = self.veh_control_in_msg.read().torque_request_body_nm;
        let num_rw = self.rw_config.num_rw.min(MAX_EFF_COUNT);

        let availability = self.rw_avail_in_msg.read();
        if self.rw_avail_in_msg.is_connected() {
            self.available_spin_axes_body.clear();
            for index in 0..num_rw {
                if availability.is_available(index) {
                    self.available_spin_axes_body
                        .push(self.rw_config.spin_axes_body[index]);
                }
            }
            self.num_available_rw = self.available_spin_axes_body.len();
        }

        if self.num_control_axes == 0 || self.num_available_rw < self.num_control_axes {
            return output;
        }

        let control_axes = &self.config.control_axes_body[..self.num_control_axes];
        let wheel_body_torque = -requested_body_torque;
        let mut projected_torque = Vector3::zeros();
        for (row, axis) in control_axes.iter().enumerate() {
            projected_torque[row] = axis.dot(&wheel_body_torque);
        }

        let mut projected_wheel_axes =
            vec![vec![0.0; self.num_available_rw]; self.num_control_axes];
        for (row, control_axis) in control_axes.iter().enumerate() {
            for (column, wheel_axis) in self.available_spin_axes_body.iter().enumerate() {
                projected_wheel_axes[row][column] = control_axis.dot(wheel_axis);
            }
        }

        // Upstream pads inactive control dimensions with identity rows before
        // applying its explicit cofactor-based 3x3 inverse.
        let mut gram = Matrix3::identity();
        for row in 0..self.num_control_axes {
            for column in 0..self.num_control_axes {
                gram[(row, column)] = (0..self.num_available_rw)
                    .map(|wheel| {
                        projected_wheel_axes[row][wheel] * projected_wheel_axes[column][wheel]
                    })
                    .sum();
            }
        }
        let Some(gram_inverse) = upstream_m33_inverse(gram) else {
            return output;
        };
        let inverse_projected_torque = gram_inverse * projected_torque;
        let available_commands: Vec<f64> = (0..self.num_available_rw)
            .map(|wheel| {
                (0..self.num_control_axes)
                    .map(|axis| projected_wheel_axes[axis][wheel] * inverse_projected_torque[axis])
                    .sum()
            })
            .collect();

        let mut available_index = 0;
        for index in 0..num_rw {
            if availability.is_available(index) {
                output.motor_torque_nm[index] = available_commands[available_index];
                available_index += 1;
            }
        }
        output
    }
}

fn upstream_m33_inverse(matrix: Matrix3<f64>) -> Option<Matrix3<f64>> {
    let determinant = matrix[(0, 0)] * matrix[(1, 1)] * matrix[(2, 2)]
        + matrix[(0, 1)] * matrix[(1, 2)] * matrix[(2, 0)]
        + matrix[(0, 2)] * matrix[(1, 0)] * matrix[(2, 1)]
        - matrix[(0, 0)] * matrix[(1, 2)] * matrix[(2, 1)]
        - matrix[(0, 1)] * matrix[(1, 0)] * matrix[(2, 2)]
        - matrix[(0, 2)] * matrix[(1, 1)] * matrix[(2, 0)];
    if determinant.is_nan() || determinant.abs() <= INVERSE_DETERMINANT_EPSILON {
        log::error!("rwMotorTorque encountered a singular 3x3 matrix inverse");
        return None;
    }

    let inverse_determinant = determinant.recip();
    Some(Matrix3::new(
        (matrix[(1, 1)] * matrix[(2, 2)] - matrix[(1, 2)] * matrix[(2, 1)]) * inverse_determinant,
        -(matrix[(0, 1)] * matrix[(2, 2)] - matrix[(0, 2)] * matrix[(2, 1)]) * inverse_determinant,
        (matrix[(0, 1)] * matrix[(1, 2)] - matrix[(0, 2)] * matrix[(1, 1)]) * inverse_determinant,
        -(matrix[(1, 0)] * matrix[(2, 2)] - matrix[(1, 2)] * matrix[(2, 0)]) * inverse_determinant,
        (matrix[(0, 0)] * matrix[(2, 2)] - matrix[(0, 2)] * matrix[(2, 0)]) * inverse_determinant,
        -(matrix[(0, 0)] * matrix[(1, 2)] - matrix[(0, 2)] * matrix[(1, 0)]) * inverse_determinant,
        (matrix[(1, 0)] * matrix[(2, 1)] - matrix[(1, 1)] * matrix[(2, 0)]) * inverse_determinant,
        -(matrix[(0, 0)] * matrix[(2, 1)] - matrix[(0, 1)] * matrix[(2, 0)]) * inverse_determinant,
        (matrix[(0, 0)] * matrix[(1, 1)] - matrix[(0, 1)] * matrix[(1, 0)]) * inverse_determinant,
    ))
}

impl Module for RwMotorTorque {
    fn init(&mut self) {
        self.rw_motor_torque_out_msg
            .write(ArrayMotorTorqueMsg::default());
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        RwMotorTorque::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, _context: &SimulationContext) {
        let command = self.compute_wheel_torques();
        self.rw_motor_torque_out_msg.write(command);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

    use crate::messages::{
        BodyTorqueCommandMsg, Output, RwArrayConfigMsg, RwAvailability, RwAvailabilityMsg,
        MAX_EFF_COUNT,
    };
    use crate::{Module, SimulationContext};

    use super::{upstream_m33_inverse, RwMotorTorque, RwMotorTorqueConfig};

    const WHEEL_AXES_36: [[f64; 3]; 36] = [
        [0.4835867893995201, 0.7025829597277155, 0.5220354411517549],
        [0.6274167231454653, 0.4634123147571517, 0.6257773422303058],
        [0.4927675437195689, 0.3909468277672152, 0.7773935462269635],
        [0.2791305379092009, 0.20278639222840245, 0.9385967301954065],
        [0.1742148051521812, 0.9353106472878886, 0.3079662233682429],
        [0.7408864742367625, 0.30733781515416325, 0.5971856492492805],
        [0.49166240509756476, 0.11024265612126483, 0.863779275153674],
        [0.08522980139648922, 0.5635691254043687, 0.8216603445736381],
        [0.5169183283391889, 0.6482094982986043, 0.5591242153068406],
        [0.5539478507672101, 0.4352935184619988, 0.7096910112262675],
        [0.08177103922211226, 0.7185493168899821, 0.6906521384470449],
        [0.5424303480563135, 0.8034905566669417, 0.24530031156636306],
        [0.6791649825098244, 0.25103926707369056, 0.6897203874901293],
        [0.6662787689368599, 0.6695372377111813, 0.32831766535181106],
        [0.28428078464167594, 0.5440295499812461, 0.7894404880867942],
        [0.8881073966834958, 0.007176386091829566, 0.4595799728433832],
        [0.7043700914244455, 0.20398698108861654, 0.6798912308987893],
        [0.5913513581668906, 0.7154722881784563, 0.3720255045596441],
        [0.5353927164036736, 0.8292977052562882, 0.1600623480977027],
        [0.5626385603464779, 0.5530980227747188, 0.6144269099038059],
        [0.8047402627946283, 0.5179828986694456, 0.2899772855298006],
        [0.6435726414836709, 0.49863310510036174, 0.5806714059015666],
        [0.2533767502100278, 0.8066673674024603, 0.533936307831739],
        [0.051675625147813466, 0.741898369799065, 0.6685180914942186],
        [0.6705007071467579, 0.243658731626882, 0.700756180292173],
        [0.6124322825812726, 0.6044312394389204, 0.5094993386086216],
        [0.5025822950964116, 0.49662160344788164, 0.7076567103083798],
        [0.4875326918964735, 0.8575174427431412, 0.16424283766253403],
        [0.3659744927810267, 0.8415919620749859, 0.39722240622155974],
        [0.6205921515961875, 0.5508152351685801, 0.5580931446303532],
        [0.20125257120061574, 0.7022636474963218, 0.6828785924235018],
        [0.4318909377763495, 0.6786025351852008, 0.5941117883924572],
        [0.6839787443692367, 0.6598940110591041, 0.31098709204629277],
        [0.35743175000357147, 0.8343049491885353, 0.4197353878920623],
        [0.8124751056450826, 0.35669421673672336, 0.46114362020262967],
        [0.04721328350343224, 0.8901899787392832, 0.45313652204714083],
    ];

    #[derive(Clone, Copy)]
    enum AvailabilityMode {
        NoMessage,
        On,
        Off,
        Mixed,
    }

    fn context(nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn standard_axes(num_wheels: usize) -> Vec<Vector3<f64>> {
        if num_wheels == MAX_EFF_COUNT {
            WHEEL_AXES_36
                .iter()
                .map(|axis| Vector3::from_row_slice(axis))
                .collect()
        } else {
            vec![
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                Vector3::from_element(1.0 / 3.0_f64.sqrt()),
            ]
        }
    }

    fn upstream_truth(
        control_axes: &[Vector3<f64>],
        wheel_axes: &[Vector3<f64>],
        requested: Vector3<f64>,
        available: &[bool],
        num_wheels: usize,
    ) -> Vec<f64> {
        let mut output = vec![0.0; MAX_EFF_COUNT];
        let num_axes = control_axes.len();
        let available_indices: Vec<_> = (0..num_wheels).filter(|index| available[*index]).collect();
        if num_axes == 0 || available_indices.len() < num_axes {
            return output;
        }
        let mut cg = DMatrix::zeros(num_axes, available_indices.len());
        for (row, control_axis) in control_axes.iter().enumerate() {
            for (column, index) in available_indices.iter().enumerate() {
                cg[(row, column)] = control_axis.dot(&wheel_axes[*index]);
            }
        }
        let gram = &cg * cg.transpose();
        if gram.determinant().abs() < 1.0e-30 {
            return output;
        }
        let projected = DVector::from_iterator(
            num_axes,
            control_axes.iter().map(|axis| axis.dot(&(-requested))),
        );
        let Some(inverse) = gram.try_inverse() else {
            return output;
        };
        let commands = cg.transpose() * inverse * projected;
        for (command_index, wheel_index) in available_indices.iter().enumerate() {
            output[*wheel_index] = commands[command_index];
        }
        output
    }

    #[test]
    fn matches_upstream_legacy_truth_vector() {
        let requested = Vector3::new(1.0, -0.5, 0.7);
        let axes = standard_axes(4);
        let mut rw_config = RwArrayConfigMsg {
            num_rw: 4,
            ..Default::default()
        };
        rw_config.spin_axes_body[..4].copy_from_slice(&axes);
        let command = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: requested,
        });
        let params = Output::new(rw_config);
        let availability = Output::new(RwAvailabilityMsg::default());
        let mut module = RwMotorTorque::new(RwMotorTorqueConfig {
            name: "rwMotorTorque".to_string(),
            control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
        });
        module.veh_control_in_msg.connect(command.slot());
        module.rw_params_in_msg.connect(params.slot());
        module.rw_avail_in_msg.connect(availability.slot());
        module.init();

        let expected = [
            -0.8,
            0.700_000_000_000_000_1,
            -0.5,
            -0.346_410_161_513_775_5,
        ];
        for time in [0, 500_000_000] {
            module.update(&context(time));
            let actual = module.rw_motor_torque_out_msg.read();
            for (index, expected_torque) in expected.iter().enumerate() {
                assert!((actual.motor_torque_nm[index] - expected_torque).abs() <= 1.0e-12);
            }
        }
    }

    #[test]
    fn matches_all_upstream_parameterized_cases() {
        let requested = Vector3::new(1.0, -0.5, 0.7);
        for num_control_axes in 0..=3 {
            for num_wheels in [2, 4, MAX_EFF_COUNT] {
                for availability_mode in [
                    AvailabilityMode::NoMessage,
                    AvailabilityMode::On,
                    AvailabilityMode::Off,
                    AvailabilityMode::Mixed,
                ] {
                    let control_axes =
                        [Vector3::x(), Vector3::y(), Vector3::z()][..num_control_axes].to_vec();
                    let axes = standard_axes(num_wheels);
                    let mut rw_config = RwArrayConfigMsg {
                        num_rw: num_wheels,
                        ..Default::default()
                    };
                    rw_config.spin_axes_body[..axes.len()].copy_from_slice(&axes);
                    let available: Vec<bool> = (0..num_wheels)
                        .map(|index| match availability_mode {
                            AvailabilityMode::NoMessage | AvailabilityMode::On => true,
                            AvailabilityMode::Off => false,
                            AvailabilityMode::Mixed => index < num_wheels / 2,
                        })
                        .collect();
                    let mut availability = RwAvailabilityMsg::default();
                    for (index, is_available) in available.iter().enumerate() {
                        availability.wheel_availability[index] = if *is_available {
                            RwAvailability::Available
                        } else {
                            RwAvailability::Unavailable
                        };
                    }
                    let expected =
                        upstream_truth(&control_axes, &axes, requested, &available, num_wheels);

                    let command = Output::new(BodyTorqueCommandMsg {
                        torque_request_body_nm: requested,
                    });
                    let params = Output::new(rw_config);
                    let availability_output = Output::new(availability);
                    let mut module = RwMotorTorque::new(RwMotorTorqueConfig {
                        name: "rwMotorTorque".to_string(),
                        control_axes_body: control_axes,
                    });
                    module.veh_control_in_msg.connect(command.slot());
                    module.rw_params_in_msg.connect(params.slot());
                    if !matches!(availability_mode, AvailabilityMode::NoMessage) {
                        module.rw_avail_in_msg.connect(availability_output.slot());
                    }
                    module.init();

                    for time in [0, 500_000_000] {
                        module.update(&context(time));
                        let actual = module.rw_motor_torque_out_msg.read();
                        assert_eq!(actual.motor_torque_nm.len(), MAX_EFF_COUNT);
                        for (index, expected_torque) in expected.iter().enumerate() {
                            assert!(
                                (actual.motor_torque_nm[index] - expected_torque).abs() <= 1.0e-8,
                                "axes={num_control_axes}, wheels={num_wheels}, index={index}: expected {}, got {}",
                                expected_torque,
                                actual.motor_torque_nm[index],
                            );
                        }
                        let num_available = available.iter().filter(|value| **value).count();
                        if num_control_axes > 0 && num_available > num_control_axes {
                            let received_body_torque = -(0..num_wheels)
                                .map(|index| axes[index] * actual.motor_torque_nm[index])
                                .sum::<Vector3<f64>>();
                            for axis in 0..num_control_axes {
                                assert!(
                                    (received_body_torque[axis] - requested[axis]).abs() <= 1.0e-8,
                                    "axes={num_control_axes}, wheels={num_wheels}, axis={axis}: requested {}, received {}",
                                    requested[axis],
                                    received_body_torque[axis],
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn uses_axes_verbatim_without_normalizing_them() {
        let command = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::new(2.0, 0.0, 0.0),
        });
        let mut config = RwArrayConfigMsg {
            num_rw: 1,
            ..Default::default()
        };
        config.spin_axes_body[0] = Vector3::new(2.0, 0.0, 0.0);
        let params = Output::new(config);
        let mut module = RwMotorTorque::new(RwMotorTorqueConfig {
            name: "rwMotorTorque".to_string(),
            control_axes_body: vec![Vector3::new(3.0, 0.0, 0.0)],
        });
        module.veh_control_in_msg.connect(command.slot());
        module.rw_params_in_msg.connect(params.slot());
        module.init();
        module.update(&context(0));

        assert!((module.rw_motor_torque_out_msg.read().motor_torque_nm[0] + 1.0).abs() <= 1.0e-12);
    }

    #[test]
    fn stops_counting_control_axes_at_the_first_zero_row() {
        let command = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::new(1.0, -0.5, 0.7),
        });
        let mut config = RwArrayConfigMsg {
            num_rw: 3,
            ..Default::default()
        };
        config.spin_axes_body[..3].copy_from_slice(&[Vector3::x(), Vector3::y(), Vector3::z()]);
        let params = Output::new(config);
        let mut module = RwMotorTorque::new(RwMotorTorqueConfig {
            name: "rwMotorTorque".to_string(),
            control_axes_body: vec![Vector3::x(), Vector3::zeros(), Vector3::z()],
        });
        module.veh_control_in_msg.connect(command.slot());
        module.rw_params_in_msg.connect(params.slot());
        module.init();
        module.update(&context(0));

        assert_eq!(module.num_control_axes, 1);
        assert_eq!(
            module.rw_motor_torque_out_msg.read().motor_torque_nm[..3],
            [-1.0, 0.0, 0.0]
        );
    }

    #[test]
    fn matches_upstream_inverse_singularity_boundary() {
        assert!(
            upstream_m33_inverse(Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, 1.0e-30)))
                .is_none()
        );
        assert!(
            upstream_m33_inverse(Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, 2.0e-30)))
                .is_some()
        );
        assert!(upstream_m33_inverse(Matrix3::from_element(f64::NAN)).is_none());
    }
}
