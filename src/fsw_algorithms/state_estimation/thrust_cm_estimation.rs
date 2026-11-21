//! Center-of-mass estimation from thruster torques: port of upstream
//! Basilisk `thrustCMEstimation` (`fswAlgorithms/stateEstimation`).
//!
//! While a thruster fires and the attitude controller holds the spacecraft
//! at its reference, the integral feedback torque `L_int` balances the
//! thruster's torque about the center of mass. With the thrust
//! `T = T_max t̂` applied at `r_TB` and the swirl torque `L_swirl t̂`, that
//! torque is
//!
//! `L = (r_TB - r_CB) × T + L_swirl t̂`, measured as `L = -L_int`,
//!
//! which is linear in the center-of-mass location: `y = H r_CB` with
//! `y = L - r_TB × T - L_swirl t̂` and `H = [T̃]`. The component of `r_CB`
//! along the thrust is unobservable from one thruster direction. A Kalman filter with a
//! static state refines `r_CB` from each such measurement, accepted only
//! when the attitude error `sqrt(|σ_BR|² + |ω_BR|²)` is below
//! `attitude_tol`. Without a measurement the residuals are NaN.
//!
//! The optional vehicle-configuration input supplies the true location, to
//! report the estimate's error (NaN without it). The estimate is published
//! as the center of mass of an otherwise zero vehicle configuration.
//!
//! # Guidance write time
//!
//! Upstream also requires the guidance message to have been written
//! (`isWritten`). The Rust inputs carry no header: a connected guidance
//! input counts as written.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, CmEstimationDataMsg, Input, Output,
    ThrusterConfigMsg, VehicleConfigMsg,
};
use crate::utilities::rigid_body_kinematics::tilde;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`ThrustCmEstimation`]. Upstream leaves them
/// uninitialized; they are zero here.
#[derive(Clone, Debug)]
pub struct ThrustCmEstimationConfig {
    /// Module instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `attitudeTol` [-]: largest attitude error at which a
    /// measurement is accepted.
    pub attitude_tol: f64,
    /// Upstream `r_CB_B` \[m\]: initial center-of-mass estimate, body
    /// components.
    pub r_cb_b_m: Vector3<f64>,
    /// Upstream `P0` \[m²\]: initial state variances (the covariance
    /// diagonal).
    pub p0_m2: Vector3<f64>,
    /// Upstream `R0` [N² m²]: measurement noise variances (the covariance
    /// diagonal).
    pub r0_n2m2: Vector3<f64>,
}

impl Default for ThrustCmEstimationConfig {
    fn default() -> Self {
        Self {
            name: "thrustCMEstimation".to_string(),
            attitude_tol: 0.0,
            r_cb_b_m: Vector3::zeros(),
            p0_m2: Vector3::zeros(),
            r0_n2m2: Vector3::zeros(),
        }
    }
}

/// Center-of-mass estimator (upstream `thrustCMEstimation`).
#[derive(Clone, Debug)]
pub struct ThrustCmEstimation {
    /// User-settable parameters, read at reset.
    pub config: ThrustCmEstimationConfig,
    /// Upstream `thrusterConfigBInMsg`: the firing thruster, body
    /// components.
    pub thruster_config_b_in_msg: Input<ThrusterConfigMsg>,
    /// Upstream `intFeedbackTorqueInMsg`: the controller's integral feedback
    /// torque, body components.
    pub int_feedback_torque_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `attGuidInMsg`: attitude tracking error.
    pub att_guid_in_msg: Input<AttitudeGuidanceMsg>,
    /// Upstream `vehConfigInMsg` (optional): the true center of mass, to
    /// report the estimate's error.
    pub veh_config_in_msg: Input<VehicleConfigMsg>,
    /// Upstream `cmEstDataOutMsg`: estimate, its 1-σ and the residuals.
    pub cm_est_data_out_msg: Output<CmEstimationDataMsg>,
    /// Upstream `vehConfigOutMsg`: the estimated center of mass (every other
    /// field zero).
    pub veh_config_out_msg: Output<VehicleConfigMsg>,
    /// Upstream `P`: state covariance.
    covariance: Matrix3<f64>,
    /// Upstream `R`: measurement noise covariance.
    noise: Matrix3<f64>,
    /// Upstream `r_CB_est` \[m\]: current estimate.
    r_cb_est_m: Vector3<f64>,
    /// Upstream `cmKnowledge`: whether the true center of mass is known.
    cm_knowledge: bool,
}

impl ThrustCmEstimation {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: ThrustCmEstimationConfig) -> Self {
        Self {
            config,
            thruster_config_b_in_msg: Input::default(),
            int_feedback_torque_in_msg: Input::default(),
            att_guid_in_msg: Input::default(),
            veh_config_in_msg: Input::default(),
            cm_est_data_out_msg: Output::default(),
            veh_config_out_msg: Output::default(),
            covariance: Matrix3::zeros(),
            noise: Matrix3::zeros(),
            r_cb_est_m: Vector3::zeros(),
            cm_knowledge: false,
        }
    }

    /// Upstream `Reset`: checks the inputs and restarts the filter from the
    /// configured estimate and covariances.
    pub fn reset(&mut self) {
        let name = &self.config.name;
        for (connected, input) in [
            (
                self.thruster_config_b_in_msg.is_connected(),
                "thruster_config_b_in_msg",
            ),
            (
                self.int_feedback_torque_in_msg.is_connected(),
                "int_feedback_torque_in_msg",
            ),
            (self.att_guid_in_msg.is_connected(), "att_guid_in_msg"),
        ] {
            if !connected {
                log::error!("{name}.{input} is not connected");
            }
        }
        self.cm_knowledge = self.veh_config_in_msg.is_connected();
        self.covariance = Matrix3::from_diagonal(&self.config.p0_m2);
        self.noise = Matrix3::from_diagonal(&self.config.r0_n2m2);
        self.r_cb_est_m = self.config.r_cb_b_m;
    }

    /// Upstream `UpdateState`.
    fn step(&mut self) {
        let thruster = self.thruster_config_b_in_msg.read();
        let r_tb_b = thruster.position_thrust_b_m;
        let thrust_b = thruster.max_thrust_n * thruster.t_hat_thrust_b;
        let swirl_b = thruster.swirl_torque_nm * thruster.t_hat_thrust_b;

        let guidance = self.att_guid_in_msg.read();
        let att_error =
            (guidance.sigma_br.norm_squared() + guidance.omega_br_b_radps.norm_squared()).sqrt();
        let torque_b = -self
            .int_feedback_torque_in_msg
            .read()
            .torque_request_body_nm;

        let mut pre_fit = Vector3::from_element(f64::NAN);
        let mut post_fit = Vector3::from_element(f64::NAN);
        // Upstream also requires `isWritten`; connected stands in for it.
        if self.att_guid_in_msg.is_connected() && att_error < self.config.attitude_tol {
            // The torque about B less the known moment arm and swirl.
            let y = torque_b - r_tb_b.cross(&thrust_b) - swirl_b;
            let h = tilde(thrust_b);
            let s = h * self.covariance * h.transpose() + self.noise;
            let gain = self.covariance
                * h.transpose()
                * s.try_inverse().unwrap_or_else(|| {
                    log::error!("{}: singular innovation covariance", self.config.name);
                    Matrix3::from_element(f64::NAN)
                });

            pre_fit = y - h * self.r_cb_est_m;
            self.r_cb_est_m += gain * pre_fit;
            self.covariance = (Matrix3::identity() - gain * h) * self.covariance;
            post_fit = y - h * self.r_cb_est_m;
        }

        let state_error = if self.cm_knowledge {
            self.r_cb_est_m - self.veh_config_in_msg.read().center_of_mass_body_m
        } else {
            Vector3::from_element(f64::NAN)
        };
        self.cm_est_data_out_msg.write(CmEstimationDataMsg {
            att_error,
            state_m: self.r_cb_est_m,
            state_error_m: state_error,
            covariance_m: self.covariance.diagonal().map(f64::sqrt),
            pre_fit_res_nm: pre_fit,
            post_fit_res_nm: post_fit,
        });
        self.veh_config_out_msg.write(VehicleConfigMsg {
            center_of_mass_body_m: self.r_cb_est_m,
            ..Default::default()
        });
    }
}

impl Module for ThrustCmEstimation {
    fn init(&mut self) {
        self.cm_est_data_out_msg
            .write(CmEstimationDataMsg::default());
        self.veh_config_out_msg.write(VehicleConfigMsg::default());
        ThrustCmEstimation::reset(self);
    }

    fn reset(&mut self, _context: &SimulationContext) {
        ThrustCmEstimation::reset(self);
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.step();
    }
}

#[cfg(test)]
mod tests {
    //! Ports `thrustCMEstimation/_UnitTest/test_thrustCMEstimation.py`. Its
    //! measurement noise comes from an unseeded NumPy generator (the pytest
    //! is marked flaky); a seeded generator draws it here.

    use nalgebra::Vector3;
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use rand_distr::{Distribution, Normal};

    use super::{ThrustCmEstimation, ThrustCmEstimationConfig};
    use crate::messages::{
        AttitudeGuidanceMsg, BodyTorqueCommandMsg, CmEstimationDataMsg, Output, ThrusterConfigMsg,
        VehicleConfigMsg,
    };
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    fn context() -> SimulationContext {
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: hifitime::Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `thrustCMEstimationTestFunction`'s module.
    fn upstream_module() -> ThrustCmEstimation {
        ThrustCmEstimation::new(ThrustCmEstimationConfig {
            name: "cmEstimator".to_string(),
            attitude_tol: 1e-4,
            r_cb_b_m: Vector3::new(0.01, -0.025, 0.04),
            p0_m2: Vector3::from_element(0.0025),
            r0_n2m2: Vector3::from_element(1e-8),
        })
    }

    /// Upstream `test_thrustCMEstimation` (`dT = 10`, `accuracy = 1e-12`):
    /// four thrust vectors at four application points, each processed by one
    /// update. The post-fit residuals are smaller than the pre-fit ones and
    /// within the 3-σ measurement noise, and the state error within the 3-σ
    /// estimate covariance. As upstream, the bounds are one-sided.
    #[test]
    fn matches_upstream_thrust_cm_estimation_unit_test() {
        let accuracy = 1e-12;
        let r_tb_b = [[6., 5., 4.], [5., 4., 6.], [4., 6., 5.], [-6., 5., 4.]].map(Vector3::from);
        let t_b = [[1., 2., 3.], [2., 3., 1.], [3., 1., 2.], [1., -2., 3.]].map(Vector3::from);

        for seed in 0..20 {
            let mut module = upstream_module();
            let veh_config = Output::new(VehicleConfigMsg::default());
            let guidance = Output::new(AttitudeGuidanceMsg::default());
            let thruster = Output::new(ThrusterConfigMsg::default());
            let torque = Output::new(BodyTorqueCommandMsg::default());
            veh_config.connect_to(&mut module.veh_config_in_msg);
            guidance.connect_to(&mut module.att_guid_in_msg);
            thruster.connect_to(&mut module.thruster_config_b_in_msg);
            torque.connect_to(&mut module.int_feedback_torque_in_msg);
            let mut log = MessageRecorder::<CmEstimationDataMsg>::default();
            module.cm_est_data_out_msg.connect_to(&mut log.input_msg);
            module.init();

            let mut rng = StdRng::seed_from_u64(seed);
            let noise = Normal::new(0.0, 1e-8_f64.sqrt()).unwrap();
            for (r, t) in r_tb_b.iter().zip(&t_b) {
                let max_thrust = t.norm();
                thruster.write(ThrusterConfigMsg {
                    position_thrust_b_m: *r,
                    t_hat_thrust_b: t / max_thrust,
                    max_thrust_n: max_thrust,
                    swirl_torque_nm: 0.0,
                });
                let measurement_noise = Vector3::from_fn(|_, _| noise.sample(&mut rng));
                torque.write(BodyTorqueCommandMsg {
                    torque_request_body_nm: -r.cross(t) + measurement_noise,
                    ..Default::default()
                });
                module.update(&context());
                log.update(&context());
            }

            assert_eq!(log.samples.len(), 4);
            let three_sigma_noise = 3.0 * 1e-8_f64.sqrt();
            for sample in &log.samples {
                assert!(
                    sample.post_fit_res_nm.norm() < sample.pre_fit_res_nm.norm() + accuracy,
                    "seed {seed}"
                );
                for j in 0..3 {
                    assert!(
                        sample.post_fit_res_nm[j] < three_sigma_noise + accuracy,
                        "seed {seed}"
                    );
                    assert!(
                        sample.state_error_m[j] < 3.0 * sample.covariance_m[j] + accuracy,
                        "seed {seed}"
                    );
                }
            }
        }
    }

    /// Not upstream: an attitude error at or above the tolerance rejects the
    /// measurement (NaN residuals, estimate and covariance kept), and
    /// without the vehicle configuration the state error is NaN.
    #[test]
    fn rejects_measurements_off_reference_and_without_truth_reports_nan_error() {
        let mut module = upstream_module();
        let guidance = Output::new(AttitudeGuidanceMsg {
            sigma_br: Vector3::new(1e-4, 0.0, 0.0),
            ..Default::default()
        });
        let thruster = Output::new(ThrusterConfigMsg {
            position_thrust_b_m: Vector3::new(6., 5., 4.),
            t_hat_thrust_b: Vector3::new(0.0, 0.0, 1.0),
            max_thrust_n: 2.0,
            swirl_torque_nm: 0.1,
        });
        let torque = Output::new(BodyTorqueCommandMsg::default());
        guidance.connect_to(&mut module.att_guid_in_msg);
        thruster.connect_to(&mut module.thruster_config_b_in_msg);
        torque.connect_to(&mut module.int_feedback_torque_in_msg);
        module.init();
        module.update(&context());

        let output = module.cm_est_data_out_msg.read();
        assert_eq!(output.att_error, 1e-4);
        assert!(output.pre_fit_res_nm.iter().all(|v| v.is_nan()));
        assert!(output.post_fit_res_nm.iter().all(|v| v.is_nan()));
        assert!(output.state_error_m.iter().all(|v| v.is_nan()));
        assert_eq!(output.state_m, Vector3::new(0.01, -0.025, 0.04));
        assert!((output.covariance_m - Vector3::from_element(0.05)).norm() < 1e-16);
        assert_eq!(
            module.veh_config_out_msg.read().center_of_mass_body_m,
            Vector3::new(0.01, -0.025, 0.04)
        );
    }

    /// Not upstream: a noise-free measurement with the swirl torque moves the
    /// estimate by the Kalman update, which the closed form reproduces.
    #[test]
    fn noise_free_update_matches_the_closed_form() {
        let mut module = upstream_module();
        let guidance = Output::new(AttitudeGuidanceMsg::default());
        let r_tb = Vector3::new(1.0, -2.0, 0.5);
        let t_hat = Vector3::new(0.0, 0.6, 0.8);
        let (max_thrust, swirl) = (2.0, 0.05);
        let r_cb_true = Vector3::new(0.02, 0.01, -0.03);
        let thrust = max_thrust * t_hat;
        let thruster = Output::new(ThrusterConfigMsg {
            position_thrust_b_m: r_tb,
            t_hat_thrust_b: t_hat,
            max_thrust_n: max_thrust,
            swirl_torque_nm: swirl,
        });
        let torque = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: -((r_tb - r_cb_true).cross(&thrust) + swirl * t_hat),
            ..Default::default()
        });
        guidance.connect_to(&mut module.att_guid_in_msg);
        thruster.connect_to(&mut module.thruster_config_b_in_msg);
        torque.connect_to(&mut module.int_feedback_torque_in_msg);
        module.init();
        module.update(&context());

        let h = crate::utilities::rigid_body_kinematics::tilde(thrust);
        let p = nalgebra::Matrix3::from_diagonal_element(0.0025);
        let r = nalgebra::Matrix3::from_diagonal_element(1e-8);
        let gain = p * h.transpose() * (h * p * h.transpose() + r).try_inverse().unwrap();
        let r_cb_0 = Vector3::new(0.01, -0.025, 0.04);
        let expected = r_cb_0 + gain * (h * r_cb_true - h * r_cb_0);
        let output = module.cm_est_data_out_msg.read();
        assert!((output.state_m - expected).norm() < 1e-12);
        // Only the components across the thrust are observable.
        assert!(((output.state_m - r_cb_true).cross(&t_hat)).norm() < 1e-6);
        assert!(output.post_fit_res_nm.norm() < output.pre_fit_res_nm.norm());
    }
}
