//! Reaction-wheel null-motion despin superimposed on the attitude-control
//! wheel torques.
//!
//! Port of upstream Basilisk `rwNullSpace`. With `[Gs]` the 3×N matrix of
//! wheel spin axes, reset builds the null-space projector
//!
//! ```text
//! [τ] = [I_N] − [Gs]^T ([Gs][Gs]^T)^-1 [Gs]
//! ```
//!
//! and every update adds `[τ] d`, with `d = −K (Ω − Ω_desired)`, to the
//! commanded wheel torques. The added torques produce no net body torque, so
//! they drive the wheel speeds towards the desired speeds (zero when the
//! optional desired-speed message is not connected) without disturbing the
//! attitude.

use nalgebra::{DMatrix, DVector, Matrix3, Matrix3xX};

use crate::messages::{
    ArrayMotorTorqueMsg, Input, MAX_EFF_COUNT, Output, RwConstellationMsg, RwSpeedMsg,
};
use crate::utilities::linear_algebra::m33_inverse;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`RwNullSpace`].
#[derive(Clone, Debug)]
pub struct RwNullSpaceConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Gain applied to the wheel-speed error (upstream `OmegaGain`, default 0)
    /// \[N m s/rad\].
    pub omega_gain: f64,
}

impl Default for RwNullSpaceConfig {
    fn default() -> Self {
        Self {
            name: "rwNullSpace".to_string(),
            omega_gain: 0.0,
        }
    }
}

/// Adds null-space wheel despin torques to the control wheel torques.
#[derive(Clone, Debug)]
pub struct RwNullSpace {
    /// Module configuration.
    pub config: RwNullSpaceConfig,
    /// Control wheel torques (upstream `rwMotorTorqueInMsg`).
    pub rw_motor_torque_in_msg: Input<ArrayMotorTorqueMsg>,
    /// Current wheel speeds (upstream `rwSpeedsInMsg`).
    pub rw_speeds_in_msg: Input<RwSpeedMsg>,
    /// Desired wheel speeds, optional (upstream `rwDesiredSpeedsInMsg`).
    pub rw_desired_speeds_in_msg: Input<RwSpeedMsg>,
    /// Wheel constellation, read on reset (upstream `rwConfigInMsg`).
    pub rw_config_in_msg: Input<RwConstellationMsg>,
    /// Control plus null-motion wheel torques (upstream `rwMotorTorqueOutMsg`).
    pub rw_motor_torque_out_msg: Output<ArrayMotorTorqueMsg>,
    num_wheels: usize,
    /// Null-space projector `[τ]`, `num_wheels × num_wheels` (upstream `tau`).
    tau: DMatrix<f64>,
}

impl RwNullSpace {
    /// Creates the module with unconnected ports and no wheels.
    pub fn new(config: RwNullSpaceConfig) -> Self {
        Self {
            config,
            rw_motor_torque_in_msg: Input::default(),
            rw_speeds_in_msg: Input::default(),
            rw_desired_speeds_in_msg: Input::default(),
            rw_config_in_msg: Input::default(),
            rw_motor_torque_out_msg: Output::default(),
            num_wheels: 0,
            tau: DMatrix::zeros(0, 0),
        }
    }

    /// Number of wheels read from the constellation at the last reset.
    pub fn num_wheels(&self) -> usize {
        self.num_wheels
    }

    /// Null-space projector `[τ]` computed at the last reset.
    pub fn null_space_projector(&self) -> &DMatrix<f64> {
        &self.tau
    }

    fn reset_projector(&mut self) {
        if !self.rw_config_in_msg.is_connected() {
            log::error!("rwNullSpace.rw_config_in_msg is not connected");
        }
        if !self.rw_motor_torque_in_msg.is_connected() {
            log::error!("rwNullSpace.rw_motor_torque_in_msg is not connected");
        }
        if !self.rw_speeds_in_msg.is_connected() {
            log::error!("rwNullSpace.rw_speeds_in_msg is not connected");
        }

        let constellation = self.rw_config_in_msg.read();
        if constellation.num_rw > MAX_EFF_COUNT {
            // Upstream logs the same error and then overruns its buffers;
            // here the extra wheels are ignored.
            log::error!("rwNullSpace.num_wheels is larger than MAX_EFF_COUNT ({MAX_EFF_COUNT})");
        }
        let wheels = constellation.active_wheels();
        self.num_wheels = wheels.len();
        self.tau = null_space_projector(&Matrix3xX::from_iterator(
            wheels.len(),
            wheels
                .iter()
                .flat_map(|wheel| wheel.gs_hat_b.iter().copied()),
        ));
    }

    fn compute_wheel_torques(&self) -> ArrayMotorTorqueMsg {
        let control = self.rw_motor_torque_in_msg.read();
        let speeds = self.rw_speeds_in_msg.read();
        let desired = if self.rw_desired_speeds_in_msg.is_connected() {
            self.rw_desired_speeds_in_msg.read()
        } else {
            RwSpeedMsg::default()
        };

        let n = self.num_wheels;
        let speed_error = DVector::from_iterator(
            n,
            (0..n).map(|i| speeds.wheel_speeds_radps[i] - desired.wheel_speeds_radps[i]),
        );
        let despin = -self.config.omega_gain * speed_error;
        let null_torque = &self.tau * despin;

        // Entries past `num_wheels` stay zero, including any control torque
        // commanded there, as upstream.
        let mut output = ArrayMotorTorqueMsg::default();
        for (i, null) in null_torque.iter().enumerate() {
            output.motor_torque_nm[i] = null + control.motor_torque_nm[i];
        }
        output
    }
}

/// `[I_N] − [Gs]^T ([Gs][Gs]^T)^-1 [Gs]` for the 3×N spin-axis matrix `[Gs]`.
///
/// A singular `[Gs][Gs]^T` (fewer than three independent axes) is inverted to
/// zero, as upstream `m33Inverse` does, which leaves `[τ] = [I_N]`.
fn null_space_projector(gs: &Matrix3xX<f64>) -> DMatrix<f64> {
    let gs_t = gs.transpose();
    let gram_inverse = m33_inverse(&(gs * &gs_t)).unwrap_or_else(Matrix3::zeros);
    let range_projector = gs_t * (gram_inverse * gs);
    DMatrix::identity(gs.ncols(), gs.ncols()) - range_projector
}

impl Module for RwNullSpace {
    fn init(&mut self) {
        self.rw_motor_torque_out_msg
            .write(ArrayMotorTorqueMsg::default());
        self.reset_projector();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_projector();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let torques = self.compute_wheel_torques();
        self.rw_motor_torque_out_msg.write(torques);
    }
}

#[cfg(test)]
mod tests {
    // Not ported: the pytest's `writeTeXSnippet` calls, which only generate
    // the module's LaTeX documentation.

    use hifitime::Epoch;
    use nalgebra::{DMatrix, DVector, Vector3};

    use crate::messages::{
        ArrayMotorTorqueMsg, Output, RwConfigElementMsg, RwConstellationMsg, RwSpeedMsg,
    };
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    use super::{RwNullSpace, RwNullSpaceConfig};

    fn speeds(values: &[f64]) -> RwSpeedMsg {
        let mut message = RwSpeedMsg::default();
        message.wheel_speeds_radps[..values.len()].copy_from_slice(values);
        message
    }

    fn constellation(axes: &[Vector3<f64>]) -> RwConstellationMsg {
        let wheels: Vec<_> = axes
            .iter()
            .map(|axis| RwConfigElementMsg {
                gs_hat_b: *axis,
                spin_axis_inertia_kg_m2: 0.08,
                max_motor_torque_nm: 0.2,
            })
            .collect();
        RwConstellationMsg::from_active(&wheels)
    }

    /// Truth of `rwNullSpaceTestFunction`, computed as the pytest does with a
    /// general matrix inverse.
    fn upstream_truth(
        axes: &[Vector3<f64>],
        gain: f64,
        wheel_speeds: &[f64],
        desired: &[f64],
        control: &[f64],
    ) -> Vec<f64> {
        let n = axes.len();
        if n == 3 {
            // No null space: the output equals the control torque.
            return control.to_vec();
        }
        let gs_t = DMatrix::from_fn(n, 3, |row, col| axes[row][col]);
        let gs = gs_t.transpose();
        let inverse = (&gs * &gs_t).try_inverse().expect("Gs Gs^T is invertible");
        let tau = DMatrix::identity(n, n) - &gs_t * inverse * &gs;
        let d = DVector::from_fn(n, |i, _| -gain * (wheel_speeds[i] - desired[i]));
        let null = tau * d;
        (0..n).map(|i| control[i] + null[i]).collect()
    }

    /// Port of `test_rwNullSpace`, every `(numWheels, defaultDesired)` case:
    /// 0.5 s task rate, 2 s run, all five samples checked. Upstream compares
    /// only the first two wheels (`compareArrayND(..., 2, ...)`); every wheel
    /// is checked here.
    #[test]
    fn matches_upstream_rw_null_space_unit_test() {
        let accuracy = 1e-6;
        for (num_wheels, default_desired) in [(3, true), (4, true), (3, false), (4, false)] {
            let gain = 0.5;
            let mut axes = vec![Vector3::x(), Vector3::y(), Vector3::z()];
            let mut wheel_speeds = vec![10.0, 20.0, 30.0];
            let mut control = vec![0.1, 0.2, 0.15];
            if num_wheels == 4 {
                axes.push(Vector3::new(1.0, 1.0, 1.0) / 3.0_f64.sqrt());
                wheel_speeds.push(40.0);
                control.push(-0.2);
            }
            let desired = if default_desired {
                vec![0.0; num_wheels]
            } else {
                vec![5.0; num_wheels]
            };

            let speed_out = Output::new(speeds(&wheel_speeds));
            let config_out = Output::new(constellation(&axes));
            let command_out = Output::new(ArrayMotorTorqueMsg::from_active(&control));
            let desired_out = Output::new(speeds(&desired));
            let mut module = RwNullSpace::new(RwNullSpaceConfig {
                omega_gain: gain,
                ..Default::default()
            });
            let mut recorder = MessageRecorder::default();
            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            simulation.connect(&command_out, &mut module.rw_motor_torque_in_msg);
            simulation.connect(&speed_out, &mut module.rw_speeds_in_msg);
            simulation.connect(&config_out, &mut module.rw_config_in_msg);
            if !default_desired {
                simulation.connect(&desired_out, &mut module.rw_desired_speeds_in_msg);
            }
            simulation.connect(&module.rw_motor_torque_out_msg, &mut recorder.input_msg);
            simulation.add_module("rwNullSpace", &mut module, 500_000_000, 0);
            simulation.add_module("recorder", &mut recorder, 500_000_000, 0);
            simulation.run_for(2_000_000_000);
            drop(simulation);

            let expected = upstream_truth(&axes, gain, &wheel_speeds, &desired, &control);
            assert_eq!(recorder.samples.len(), 5);
            for sample in &recorder.samples {
                for (wheel, expected_torque) in expected.iter().enumerate() {
                    let actual = sample.motor_torque_nm[wheel];
                    assert!(
                        (actual - expected_torque).abs() <= accuracy,
                        "numWheels={num_wheels}, defaultDesired={default_desired}, wheel={wheel}: \
                         expected {expected_torque}, got {actual}"
                    );
                }
            }
        }
    }

    #[test]
    fn null_motion_produces_no_body_torque() {
        let axes = [
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Vector3::new(1.0, 1.0, 1.0) / 3.0_f64.sqrt(),
        ];
        let speed_out = Output::new(speeds(&[10.0, 20.0, 30.0, 40.0]));
        let config_out = Output::new(constellation(&axes));
        let command_out = Output::new(ArrayMotorTorqueMsg::default());
        let mut module = RwNullSpace::new(RwNullSpaceConfig {
            omega_gain: 0.5,
            ..Default::default()
        });
        command_out.connect_to(&mut module.rw_motor_torque_in_msg);
        speed_out.connect_to(&mut module.rw_speeds_in_msg);
        config_out.connect_to(&mut module.rw_config_in_msg);
        module.init();
        module.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });

        let torques = module.rw_motor_torque_out_msg.read().motor_torque_nm;
        let body_torque: Vector3<f64> = axes
            .iter()
            .zip(torques)
            .map(|(axis, torque)| axis * torque)
            .sum();
        assert!(body_torque.amax() <= 1e-12);
        assert!(torques.iter().any(|torque| torque.abs() > 1e-3));
    }

    #[test]
    fn singular_axis_set_leaves_the_identity_projector() {
        // Two wheels cannot span three axes: upstream inverts the singular
        // Gram matrix to zero, so [tau] = [I] and the full despin is applied.
        let config_out = Output::new(constellation(&[Vector3::x(), Vector3::y()]));
        let mut module = RwNullSpace::new(RwNullSpaceConfig::default());
        config_out.connect_to(&mut module.rw_config_in_msg);
        module.init();
        assert_eq!(module.num_wheels(), 2);
        assert_eq!(
            *module.null_space_projector(),
            DMatrix::<f64>::identity(2, 2)
        );
    }
}
