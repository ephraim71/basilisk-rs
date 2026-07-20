use nalgebra::Vector3;

use crate::kinematics::{add_mrp, mrp_to_dcm, subtract_mrp};
use crate::messages::{
    AttitudeGuidanceMsg, AttitudeReferenceMsg, Input, NavigationAttitudeMsg, Output,
};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct AttTrackingErrorConfig {
    pub name: String,
    /// MRP rotation from corrected reference frame R to original reference R0.
    pub sigma_r0r: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct AttTrackingError {
    pub config: AttTrackingErrorConfig,
    pub att_nav_in_msg: Input<NavigationAttitudeMsg>,
    pub att_ref_in_msg: Input<AttitudeReferenceMsg>,
    pub att_guid_out_msg: Output<AttitudeGuidanceMsg>,
}

impl AttTrackingError {
    pub fn new(config: AttTrackingErrorConfig) -> Self {
        Self {
            config,
            att_nav_in_msg: Input::default(),
            att_ref_in_msg: Input::default(),
            att_guid_out_msg: Output::default(),
        }
    }

    pub fn compute_attitude_error(
        &self,
        navigation: NavigationAttitudeMsg,
        reference: AttitudeReferenceMsg,
    ) -> AttitudeGuidanceMsg {
        let sigma_rr0 = -self.config.sigma_r0r;
        let sigma_rn = add_mrp(reference.sigma_rn, sigma_rr0);
        let sigma_br = subtract_mrp(navigation.sigma_bn, sigma_rn);

        let dcm_bn = mrp_to_dcm(navigation.sigma_bn);
        let omega_rn_b_radps = dcm_bn * reference.omega_rn_n_radps;
        let omega_br_b_radps = navigation.omega_bn_b_radps - omega_rn_b_radps;
        let domega_rn_b_radps2 = dcm_bn * reference.domega_rn_n_radps2;

        AttitudeGuidanceMsg {
            sigma_br,
            omega_br_b_radps,
            omega_rn_b_radps,
            domega_rn_b_radps2,
        }
    }

    fn validate_connections(&self) {
        if !self.att_ref_in_msg.is_connected() {
            log::error!("attTrackingError.att_ref_in_msg is not connected");
        }
        if !self.att_nav_in_msg.is_connected() {
            log::error!("attTrackingError.att_nav_in_msg is not connected");
        }
    }
}

impl Module for AttTrackingError {
    fn init(&mut self) {
        self.att_guid_out_msg.write(AttitudeGuidanceMsg::default());
        self.validate_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.validate_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let reference = self.att_ref_in_msg.read();
        let navigation = self.att_nav_in_msg.read();
        let guidance = self.compute_attitude_error(navigation, reference);
        self.att_guid_out_msg.write(guidance);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{AttitudeReferenceMsg, NavigationAttitudeMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    use super::{AttTrackingError, AttTrackingErrorConfig};

    #[test]
    fn matches_upstream_att_tracking_error_unit_test() {
        let navigation_output = Output::new(NavigationAttitudeMsg {
            sigma_bn: Vector3::new(0.25, -0.45, 0.75),
            omega_bn_b_radps: Vector3::new(-0.015, -0.012, 0.005),
            ..Default::default()
        });
        let reference_output = Output::new(AttitudeReferenceMsg {
            sigma_rn: Vector3::new(0.35, -0.25, 0.15),
            omega_rn_n_radps: Vector3::new(0.018, -0.032, 0.015),
            domega_rn_n_radps2: Vector3::new(0.048, -0.022, 0.025),
        });
        let mut module = AttTrackingError::new(AttTrackingErrorConfig {
            name: "attTrackingError".to_string(),
            sigma_r0r: Vector3::new(0.01, 0.05, -0.55),
        });
        let mut recorder = MessageRecorder::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&navigation_output, &mut module.att_nav_in_msg);
        simulation.connect(&reference_output, &mut module.att_ref_in_msg);
        simulation.connect(&module.att_guid_out_msg, &mut recorder.input_msg);
        simulation.add_module("attTrackingError", &mut module, 500_000_000, 0);
        simulation.add_module("recorder", &mut recorder, 500_000_000, 0);
        simulation.run_for(300_000_000);
        assert_eq!(simulation.current_sim_nanos(), 0);
        simulation.run_for(600_000_000);
        drop(simulation);

        assert_eq!(recorder.samples.len(), 2);
        let actual = &recorder.samples[0];
        let expected_sigma = Vector3::new(
            0.183_684_148_175_340_82,
            -0.097_444_776_941_816_55,
            -0.098_960_695_605_181_55,
        );
        let expected_omega_br = Vector3::new(
            -0.011_812_076_480_132_345,
            -0.008_916_032_420_030_657,
            -0.034_412_260_625_307_604,
        );
        let expected_omega_rn = Vector3::new(
            -0.003_187_923_519_867_654_6,
            -0.003_083_967_579_969_344,
            0.039_412_260_625_307_6,
        );
        let expected_domega = Vector3::new(
            -0.023_886_234_212_451_878,
            -0.028_356_002_777_148_778,
            0.045_148_476_404_528_015,
        );

        assert_components_close(actual.sigma_br, expected_sigma, 1.0e-12);
        assert_components_close(actual.omega_br_b_radps, expected_omega_br, 1.0e-12);
        assert_components_close(actual.omega_rn_b_radps, expected_omega_rn, 1.0e-12);
        assert_components_close(actual.domega_rn_b_radps2, expected_domega, 1.0e-12);
    }

    #[test]
    fn reset_preserves_the_last_guidance_output() {
        let navigation = Output::new(NavigationAttitudeMsg {
            sigma_bn: Vector3::new(0.1, -0.2, 0.3),
            omega_bn_b_radps: Vector3::new(0.01, 0.02, -0.03),
            ..Default::default()
        });
        let reference = Output::new(AttitudeReferenceMsg::default());
        let mut module = AttTrackingError::new(AttTrackingErrorConfig {
            name: "attTrackingError".to_string(),
            sigma_r0r: Vector3::zeros(),
        });
        module.att_nav_in_msg.connect(navigation.slot());
        module.att_ref_in_msg.connect(reference.slot());
        let context = SimulationContext {
            current_sim_nanos: 500_000_000,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        };
        module.init();
        module.update(&context);
        let before = module.att_guid_out_msg.read();

        Module::reset(&mut module, &context);
        let after = module.att_guid_out_msg.read();

        assert_eq!(after.sigma_br, before.sigma_br);
        assert_eq!(after.omega_br_b_radps, before.omega_br_b_radps);
        assert_eq!(after.omega_rn_b_radps, before.omega_rn_b_radps);
        assert_eq!(after.domega_rn_b_radps2, before.domega_rn_b_radps2);
    }

    fn assert_components_close(actual: Vector3<f64>, expected: Vector3<f64>, tolerance: f64) {
        for index in 0..3 {
            assert!((actual[index] - expected[index]).abs() <= tolerance);
        }
    }
}
