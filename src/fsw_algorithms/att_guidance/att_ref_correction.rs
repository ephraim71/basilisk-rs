//! Fixed-rotation correction of an attitude reference: port of upstream
//! Basilisk `attRefCorrection`.
//!
//! A feed-through of the attitude reference message that rotates the
//! reference attitude by a constant offset, so that a corrected body frame
//! `Bc` (rather than the body frame `B` itself) is driven onto the original
//! reference `R`. With `sigma_BcB` the attitude of `Bc` relative to `B`, the
//! published reference is
//!
//! `[RcN] = [BcB]^T [RN]`
//!
//! The angular velocity and acceleration of the reference pass through
//! unchanged, as upstream.

use nalgebra::Vector3;

use crate::messages::{AttitudeReferenceMsg, Input, Output};
use crate::utilities::rigid_body_kinematics::add_mrp;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`AttRefCorrection`]. `Default` is upstream's
/// zero-initialized configuration (no correction).
#[derive(Clone, Debug)]
pub struct AttRefCorrectionConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `sigma_BcB`: MRP set of the corrected body frame `Bc`
    /// relative to the body frame `B`.
    pub sigma_bcb: Vector3<f64>,
}

impl Default for AttRefCorrectionConfig {
    fn default() -> Self {
        Self {
            name: "attRefCorrection".to_string(),
            sigma_bcb: Vector3::zeros(),
        }
    }
}

/// Attitude reference corrected by a fixed body-frame offset: upstream
/// `attRefCorrection`.
#[derive(Clone, Debug)]
pub struct AttRefCorrection {
    /// User-settable parameters, read on every update.
    pub config: AttRefCorrectionConfig,
    /// Upstream `attRefInMsg`: reference to correct.
    pub att_ref_in_msg: Input<AttitudeReferenceMsg>,
    /// Upstream `attRefOutMsg`: corrected reference.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
}

impl AttRefCorrection {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: AttRefCorrectionConfig) -> Self {
        Self {
            config,
            att_ref_in_msg: Input::default(),
            att_ref_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_attRefCorrection`: the module keeps no state, so a
    /// reset only reports a missing input.
    fn check_connections(&self) {
        if !self.att_ref_in_msg.is_connected() {
            log::error!("attRefCorrection.att_ref_in_msg is not connected");
        }
    }

    /// `reference` with its attitude replaced by `[RcN] = [BBc][RN]`.
    fn corrected_reference(&self, reference: AttitudeReferenceMsg) -> AttitudeReferenceMsg {
        let sigma_bbc = -self.config.sigma_bcb;
        AttitudeReferenceMsg {
            sigma_rn: add_mrp(reference.sigma_rn, sigma_bbc),
            ..reference
        }
    }
}

impl Module for AttRefCorrection {
    fn init(&mut self) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let reference = self.att_ref_in_msg.read();
        self.att_ref_out_msg
            .write(self.corrected_reference(reference));
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{AttitudeReferenceMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_dcm};
    use crate::{Module, SimulationContext};

    use super::{AttRefCorrection, AttRefCorrectionConfig};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn assert_vector_close(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64) {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!((a - e).abs() <= accuracy, "{actual} != {expected}");
        }
    }

    /// Upstream `test_attRefCorrection.py` (`accuracy = 1e-12`): a half turn
    /// about x as the correction of a quarter turn about x gives a quarter
    /// turn the other way, at t = 0, 0.5 and 1 s.
    #[test]
    fn matches_upstream_att_ref_correction_unit_test() {
        let accuracy = 1e-12;
        let reference = Output::new(AttitudeReferenceMsg {
            sigma_rn: Vector3::new((PI / 8.0).tan(), 0.0, 0.0),
            ..Default::default()
        });
        let mut module = AttRefCorrection::new(AttRefCorrectionConfig::default());
        module.config.sigma_bcb = Vector3::new((PI / 4.0).tan(), 0.0, 0.0);
        let mut recorder = MessageRecorder::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&reference, &mut module.att_ref_in_msg);
        simulation.connect(&module.att_ref_out_msg, &mut recorder.input_msg);
        simulation.add_module("attRefCorrectionTag", &mut module, 500_000_000, 0);
        simulation.add_module("recorder", &mut recorder, 500_000_000, 0);
        simulation.run_for(1_000_000_000);
        drop(simulation);

        let true_vector = Vector3::new(-(PI / 8.0).tan(), 0.0, 0.0);
        assert_eq!(recorder.samples.len(), 3);
        for sample in &recorder.samples {
            assert_vector_close(sample.sigma_rn, true_vector, accuracy);
        }
    }

    // Not upstream: the reference rates pass through untouched, and a general
    // correction satisfies [RcN] = [BcB]^T [RN].
    #[test]
    fn rotates_the_attitude_and_passes_the_rates_through() {
        let input = AttitudeReferenceMsg {
            sigma_rn: Vector3::new(0.1, -0.3, 0.25),
            omega_rn_n_radps: Vector3::new(0.01, -0.02, 0.03),
            domega_rn_n_radps2: Vector3::new(-1e-4, 2e-4, 3e-4),
        };
        let sigma_bcb = Vector3::new(-0.2, 0.15, 0.4);
        let reference = Output::new(input.clone());
        let mut module = AttRefCorrection::new(AttRefCorrectionConfig {
            sigma_bcb,
            ..Default::default()
        });
        reference.connect_to(&mut module.att_ref_in_msg);
        module.init();
        module.update(&context(0));

        let output = module.att_ref_out_msg.read();
        let expected = dcm_to_mrp(mrp_to_dcm(sigma_bcb).transpose() * mrp_to_dcm(input.sigma_rn));
        assert_vector_close(output.sigma_rn, expected, 1e-14);
        assert_eq!(output.omega_rn_n_radps, input.omega_rn_n_radps);
        assert_eq!(output.domega_rn_n_radps2, input.domega_rn_n_radps2);
    }
}
