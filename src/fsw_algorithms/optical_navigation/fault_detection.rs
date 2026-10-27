//! Fault detection between two optical-navigation measurements: port of
//! upstream Basilisk `faultDetection`.
//!
//! Two image-processing chains (a primary and a secondary) each provide an
//! [`OpNavMsg`]. When both are valid, the module flags a fault if the
//! camera-frame positions differ by more than `sigma_fault` times the square
//! root of the summed covariance norms, and then trusts the secondary. Without
//! a fault, the [`FaultMode`] decides between the primary measurement and the
//! covariance-weighted merge of both, and also what is published when only one
//! of them is valid.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{CameraConfigMsg, Input, NavigationAttitudeMsg, OpNavMsg, Output};
use crate::utilities::linear_algebra::m33_inverse;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// How restrictive the module is about which measurements it trusts:
/// upstream `faultMode`.
///
/// Upstream stores a `double` and compares it with 0 and 2, so any value of
/// 2 or more acts as 2 and any value between 0 and 2 as 1. Negative values,
/// which upstream documents nowhere (they publish nothing or a zeroed
/// valid message), cannot be expressed.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum FaultMode {
    /// Upstream 0, the least restrictive: use whichever measurement is valid,
    /// and merge them when both are valid and agree.
    #[default]
    Merge = 0,
    /// Upstream 1: when both are valid and agree use the primary; the
    /// secondary only serves the dissimilarity check and is never used alone.
    PrimaryWithDissimilarCheck = 1,
    /// Upstream 2, the most restrictive: as `PrimaryWithDissimilarCheck`, but
    /// the primary is not used without the secondary either.
    RequireBoth = 2,
}

/// User parameters of [`FaultDetection`]; the defaults are upstream's
/// zero-initialized ones.
#[derive(Clone, Debug)]
pub struct FaultDetectionConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `planetTarget` [-]: targeted planet (0 none, 1 Earth, 2 Mars,
    /// 3 Jupiter). Upstream declares it but never uses it.
    pub planet_target: i32,
    /// Upstream `faultMode`.
    pub fault_mode: FaultMode,
    /// Upstream `sigmaFault` [-]: multiplier on the covariances in the
    /// dissimilarity check.
    pub sigma_fault: f64,
}

impl Default for FaultDetectionConfig {
    fn default() -> Self {
        Self {
            name: "faultDetection".to_string(),
            planet_target: 0,
            fault_mode: FaultMode::default(),
            sigma_fault: 0.0,
        }
    }
}

/// Optical-navigation fault detection: upstream `faultDetection`.
#[derive(Clone, Debug)]
pub struct FaultDetection {
    /// Module configuration.
    pub config: FaultDetectionConfig,
    /// Upstream `navMeasPrimaryInMsg`: primary measurement.
    pub nav_meas_primary_in_msg: Input<OpNavMsg>,
    /// Upstream `navMeasSecondaryInMsg`: secondary measurement.
    pub nav_meas_secondary_in_msg: Input<OpNavMsg>,
    /// Upstream `cameraConfigInMsg`: camera mounting (`sigma_CB`).
    pub camera_config_in_msg: Input<CameraConfigMsg>,
    /// Upstream `attInMsg`: spacecraft attitude.
    pub att_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `opNavOutMsg`: the measurement to use.
    pub op_nav_out_msg: Output<OpNavMsg>,
}

impl FaultDetection {
    /// Builds the module with unconnected inputs.
    pub fn new(config: FaultDetectionConfig) -> Self {
        Self {
            config,
            nav_meas_primary_in_msg: Input::default(),
            nav_meas_secondary_in_msg: Input::default(),
            camera_config_in_msg: Input::default(),
            att_in_msg: Input::default(),
            op_nav_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_faultDetection`: check that every input is connected.
    pub fn reset(&mut self) {
        if !self.nav_meas_primary_in_msg.is_connected() {
            log::error!("faultDetection.nav_meas_primary_in_msg is not connected");
        }
        if !self.nav_meas_secondary_in_msg.is_connected() {
            log::error!("faultDetection.nav_meas_secondary_in_msg is not connected");
        }
        if !self.camera_config_in_msg.is_connected() {
            log::error!("faultDetection.camera_config_in_msg is not connected");
        }
        if !self.att_in_msg.is_connected() {
            log::error!("faultDetection.att_in_msg is not connected");
        }
    }

    /// Upstream `Update_faultDetection`: the measurement to publish.
    fn select_measurement(&self) -> OpNavMsg {
        let primary = self.nav_meas_primary_in_msg.read();
        let secondary = self.nav_meas_secondary_in_msg.read();
        let mode = self.config.fault_mode;
        // A zeroed message is upstream's "no valid measurement".
        let invalid = OpNavMsg::default();

        match (primary.valid, secondary.valid) {
            (false, false) => invalid,
            (true, false) if mode == FaultMode::RequireBoth => invalid,
            (true, false) => primary,
            (false, true) if mode == FaultMode::Merge => secondary,
            (false, true) => invalid,
            (true, true) => {
                let fault_norm = (secondary.position_bn_c_m - primary.position_bn_c_m).norm();
                let threshold = self.config.sigma_fault
                    * (row_major_frobenius_norm(&primary.covar_c_m2)
                        + row_major_frobenius_norm(&secondary.covar_c_m2))
                    .sqrt();
                if fault_norm > threshold {
                    // The measurements disagree: trust the secondary.
                    OpNavMsg {
                        fault_detected: true,
                        ..secondary
                    }
                } else if mode == FaultMode::Merge {
                    self.merge(&primary, &secondary)
                } else {
                    primary
                }
            }
        }
    }

    /// Covariance-weighted merge `P = (P1⁻¹ + P2⁻¹)⁻¹`,
    /// `x = P (P1⁻¹ x1 + P2⁻¹ x2)` in the camera frame, mapped to the body and
    /// inertial frames. A singular covariance inverts to zero, as upstream.
    fn merge(&self, primary: &OpNavMsg, secondary: &OpNavMsg) -> OpNavMsg {
        let camera = self.camera_config_in_msg.read();
        let navigation = self.att_in_msg.read();
        let dcm_cb = mrp_to_dcm(camera.sigma_cb);
        let dcm_nc = (dcm_cb * mrp_to_dcm(navigation.sigma_bn)).transpose();

        let inverse = |m: &Matrix3<f64>| m33_inverse(m).unwrap_or_else(Matrix3::zeros);
        let p1_inv = inverse(&primary.covar_c_m2);
        let p2_inv = inverse(&secondary.covar_c_m2);
        let p_c = inverse(&(p1_inv + p2_inv));
        let x_c: Vector3<f64> =
            p_c * (p1_inv * primary.position_bn_c_m + p2_inv * secondary.position_bn_c_m);

        OpNavMsg {
            time_tag_s: primary.time_tag_s,
            valid: true,
            covar_n_m2: dcm_nc * p_c * dcm_nc.transpose(),
            covar_b_m2: dcm_cb.tr_mul(&p_c) * dcm_cb,
            covar_c_m2: p_c,
            position_bn_n_m: dcm_nc * x_c,
            position_bn_b_m: dcm_cb.tr_mul(&x_c),
            position_bn_c_m: x_c,
            planet_id: primary.planet_id,
            fault_detected: false,
        }
    }
}

impl Module for FaultDetection {
    fn init(&mut self) {
        self.op_nav_out_msg.write(OpNavMsg::default());
        FaultDetection::reset(self);
    }

    fn reset(&mut self, _context: &SimulationContext) {
        FaultDetection::reset(self);
    }

    fn update(&mut self, _context: &SimulationContext) {
        let measurement = self.select_measurement();
        self.op_nav_out_msg.write(measurement);
    }
}

/// Upstream `vNorm(covar, 9)`: square root of the sum of squares taken in
/// upstream's row-major order.
fn row_major_frobenius_norm(m: &Matrix3<f64>) -> f64 {
    m.transpose().iter().fold(0.0, |sum, x| sum + x * x).sqrt()
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use crate::messages::{CameraConfigMsg, NavigationAttitudeMsg, OpNavMsg, Output};
    use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
    use crate::{Module, SimulationContext};

    use super::{FaultDetection, FaultDetectionConfig, FaultMode};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn mode(code: u8) -> FaultMode {
        match code {
            0 => FaultMode::Merge,
            1 => FaultMode::PrimaryWithDissimilarCheck,
            _ => FaultMode::RequireBoth,
        }
    }

    /// The expected `(r_BN_C, covar_C, timeTag, faultDetected)`, computed as
    /// the upstream pytest does.
    fn upstream_truth(
        r_c1: Vector3<f64>,
        r_c2: Vector3<f64>,
        valid1: bool,
        valid2: bool,
        fault_mode: u8,
        sigma_fault: f64,
    ) -> (Vector3<f64>, Matrix3<f64>, f64, bool) {
        let covar = Matrix3::new(0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0);
        let none = (Vector3::zeros(), Matrix3::zeros(), 0.0, false);
        match (valid1, valid2) {
            (false, false) => none,
            (false, true) if fault_mode > 0 => none,
            (false, true) => (r_c2, covar, 12345.0, false),
            (true, false) if fault_mode == 2 => none,
            (true, false) => (r_c1, covar, 12345.0, false),
            (true, true) => {
                let fault_norm = (r_c2 - r_c1).norm();
                // The pytest compares with the norms of the covariance's
                // third columns, not with the module's threshold; both agree
                // on every parametrization.
                let z = covar.column(2).norm();
                if fault_norm > sigma_fault * (z + z) {
                    (r_c2, covar, 12345.0, true)
                } else if fault_mode > 0 {
                    (r_c1, covar, 12345.0, false)
                } else {
                    let inv = covar.try_inverse().expect("diagonal covariance");
                    let merged = (inv + inv).try_inverse().expect("diagonal covariance");
                    (merged * (inv * r_c1 + inv * r_c2), merged, 12345.0, false)
                }
            }
        }
    }

    /// `(r_c1, r_c2, valid1, valid2, faultMode)` of the pytest.
    type Parametrization = ([f64; 3], [f64; 3], bool, bool, u8);

    /// Port of `test_faultDetection.py::test_faultdetection`, all twelve
    /// parametrizations, checked to 1e-10 after one update. Upstream checks
    /// only the first two position components (`outputR[-1, 1:]` has two
    /// entries); all three are checked here.
    #[test]
    fn matches_upstream_fault_detection_unit_test() {
        let parametrizations: [Parametrization; 12] = [
            ([10.0, 10.0, 1.0], [10.0, 10.0, 1.0], true, true, 1),
            ([10.0, 10.0, 1.0], [10.0, 10.0, 1.0], true, false, 1),
            ([10.0, 10.0, 1.0], [10.0, 10.0, 1.0], true, false, 2),
            ([10.0, 10.0, 1.0], [10.0, 10.0, 1.0], false, true, 1),
            ([10.0, 10.0, 1.0], [10.0, 10.0, 1.0], false, true, 2),
            ([10.0, 10.0, 1.0], [10.0, 10.0, 1.0], false, true, 0),
            ([10.0, 10.0, 1.0], [10.0, 10.0, 1.0], false, false, 1),
            ([10.0, 10.0, 1.0], [100.0, 10.0, 1.0], true, true, 1),
            ([10.0, 10.0, 1.0], [100.0, 10.0, 1.0], true, true, 0),
            ([10.0, 10.0, 1.0], [11.0, 9.0, 0.5], true, true, 0),
            ([10.0, 10.0, 1.0], [11.0, 9.0, 0.5], true, true, 2),
            ([10.0, 10.0, 1.0], [11.0, 9.0, 0.5], true, true, 0),
        ];
        let accuracy = 1e-10;
        let sigma_fault = 3.0;
        let covar = Matrix3::new(0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0);
        let sigma_cb = Vector3::new(1.0, 0.3, 0.1);
        let sigma_bn = Vector3::new(0.6, 1.0, 0.1);
        let dcm_nc = mrp_to_dcm(sigma_bn).transpose() * mrp_to_dcm(sigma_cb).transpose();

        for (r_c1, r_c2, valid1, valid2, fault_mode) in parametrizations {
            let r_c1 = Vector3::from(r_c1);
            let r_c2 = Vector3::from(r_c2);
            let measurement = |r_c: Vector3<f64>, valid: bool| OpNavMsg {
                position_bn_c_m: r_c,
                position_bn_n_m: dcm_nc * r_c,
                valid,
                covar_c_m2: covar,
                covar_n_m2: dcm_nc * covar * dcm_nc.transpose(),
                time_tag_s: 12345.0,
                ..Default::default()
            };
            let primary = Output::new(measurement(r_c1, valid1));
            let secondary = Output::new(measurement(r_c2, valid2));
            let camera = Output::new(CameraConfigMsg {
                field_of_view_rad: 2.0 * (10.0e-3 / 2.0 / 1.0e-3_f64).atan(),
                resolution_px: [512, 512],
                sigma_cb,
                ..Default::default()
            });
            let attitude = Output::new(NavigationAttitudeMsg {
                sigma_bn,
                ..Default::default()
            });

            let mut module = FaultDetection::new(FaultDetectionConfig {
                sigma_fault,
                fault_mode: mode(fault_mode),
                ..Default::default()
            });
            primary.connect_to(&mut module.nav_meas_primary_in_msg);
            secondary.connect_to(&mut module.nav_meas_secondary_in_msg);
            camera.connect_to(&mut module.camera_config_in_msg);
            attitude.connect_to(&mut module.att_in_msg);
            module.init();
            module.update(&context(0));
            module.update(&context(500_000_000));

            let output = module.op_nav_out_msg.read();
            let (r_expected, covar_expected, time_expected, fault_expected) =
                upstream_truth(r_c1, r_c2, valid1, valid2, fault_mode, sigma_fault);
            let label = format!("{r_c1:?} {r_c2:?} {valid1} {valid2} {fault_mode}");
            assert!(
                (output.position_bn_c_m - r_expected)
                    .iter()
                    .all(|e| e.abs() <= accuracy),
                "{label}: r_BN_C {:?}",
                output.position_bn_c_m
            );
            assert!(
                (output.covar_c_m2 - covar_expected)
                    .iter()
                    .all(|e| e.abs() <= accuracy),
                "{label}: covar_C {:?}",
                output.covar_c_m2
            );
            assert!(
                (output.time_tag_s - time_expected).abs() <= accuracy,
                "{label}: time tag {}",
                output.time_tag_s
            );
            assert_eq!(output.fault_detected, fault_expected, "{label}");
        }
    }

    // Not upstream: singular covariances invert to zero, as upstream's
    // m33Inverse writes, so two agreeing measurements without covariance
    // (zero threshold, zero difference) merge to the zero position.
    #[test]
    fn merges_singular_covariances_to_zero() {
        let measurement = OpNavMsg {
            valid: true,
            position_bn_c_m: Vector3::new(10.0, 10.0, 1.0),
            time_tag_s: 3.0,
            planet_id: 2,
            ..Default::default()
        };
        let primary = Output::new(measurement.clone());
        let secondary = Output::new(measurement);
        let camera = Output::new(CameraConfigMsg::default());
        let attitude = Output::new(NavigationAttitudeMsg::default());
        let mut module = FaultDetection::new(FaultDetectionConfig {
            sigma_fault: 3.0,
            ..Default::default()
        });
        primary.connect_to(&mut module.nav_meas_primary_in_msg);
        secondary.connect_to(&mut module.nav_meas_secondary_in_msg);
        camera.connect_to(&mut module.camera_config_in_msg);
        attitude.connect_to(&mut module.att_in_msg);
        module.init();
        module.update(&context(0));

        let output = module.op_nav_out_msg.read();
        assert!(output.valid && !output.fault_detected);
        assert_eq!(output.time_tag_s, 3.0);
        assert_eq!(output.planet_id, 2);
        assert_eq!(output.position_bn_c_m, Vector3::zeros());
        assert_eq!(output.position_bn_n_m, Vector3::zeros());
        assert_eq!(output.covar_c_m2, Matrix3::zeros());
        assert_eq!(output.covar_n_m2, Matrix3::zeros());
    }

    #[test]
    fn merged_measurement_is_mapped_to_the_body_and_inertial_frames() {
        let sigma_cb = Vector3::new(0.1, -0.2, 0.3);
        let sigma_bn = Vector3::new(-0.3, 0.2, 0.4);
        let covar_1 = Matrix3::new(2.0, 0.1, 0.0, 0.1, 1.0, 0.2, 0.0, 0.2, 3.0);
        let covar_2 = Matrix3::new(1.0, 0.0, 0.3, 0.0, 2.0, 0.0, 0.3, 0.0, 1.5);
        let r_1 = Vector3::new(100.0, -50.0, 1000.0);
        let r_2 = Vector3::new(101.0, -49.0, 1002.0);
        let primary = Output::new(OpNavMsg {
            valid: true,
            position_bn_c_m: r_1,
            covar_c_m2: covar_1,
            time_tag_s: 7.5,
            planet_id: 2,
            ..Default::default()
        });
        let secondary = Output::new(OpNavMsg {
            valid: true,
            position_bn_c_m: r_2,
            covar_c_m2: covar_2,
            time_tag_s: 8.0,
            planet_id: 3,
            ..Default::default()
        });
        let camera = Output::new(CameraConfigMsg {
            sigma_cb,
            ..Default::default()
        });
        let attitude = Output::new(NavigationAttitudeMsg {
            sigma_bn,
            ..Default::default()
        });
        let mut module = FaultDetection::new(FaultDetectionConfig {
            sigma_fault: 3.0,
            ..Default::default()
        });
        primary.connect_to(&mut module.nav_meas_primary_in_msg);
        secondary.connect_to(&mut module.nav_meas_secondary_in_msg);
        camera.connect_to(&mut module.camera_config_in_msg);
        attitude.connect_to(&mut module.att_in_msg);
        module.init();
        module.update(&context(1_000_000_000));
        let output = module.op_nav_out_msg.read();

        let p1_inv = covar_1.try_inverse().expect("positive definite");
        let p2_inv = covar_2.try_inverse().expect("positive definite");
        let p = (p1_inv + p2_inv).try_inverse().expect("positive definite");
        let x = p * (p1_inv * r_1 + p2_inv * r_2);
        let dcm_cb = mrp_to_dcm(sigma_cb);
        let dcm_cn = dcm_cb * mrp_to_dcm(sigma_bn);
        assert!(output.valid && !output.fault_detected);
        assert_eq!(output.time_tag_s, 7.5);
        assert_eq!(output.planet_id, 2);
        assert!((output.position_bn_c_m - x).norm() < 1e-10);
        assert!((output.position_bn_b_m - dcm_cb.transpose() * x).norm() < 1e-10);
        assert!((output.position_bn_n_m - dcm_cn.transpose() * x).norm() < 1e-10);
        assert!((output.covar_c_m2 - p).norm() < 1e-12);
        assert!((output.covar_b_m2 - dcm_cb.transpose() * p * dcm_cb).norm() < 1e-12);
        assert!((output.covar_n_m2 - dcm_cn.transpose() * p * dcm_cn).norm() < 1e-12);
    }
}
