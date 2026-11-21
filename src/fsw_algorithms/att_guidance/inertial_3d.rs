use nalgebra::Vector3;

use crate::messages::{AttitudeReferenceMsg, Output};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct Inertial3DConfig {
    pub name: String,
    /// Fixed MRP attitude of reference frame R relative to inertial frame N.
    pub sigma_r0n: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct Inertial3D {
    pub config: Inertial3DConfig,
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
}

impl Inertial3D {
    pub fn new(config: Inertial3DConfig) -> Self {
        Self {
            config,
            att_ref_out_msg: Output::default(),
        }
    }

    pub fn pointing_reference(&self) -> AttitudeReferenceMsg {
        AttitudeReferenceMsg {
            sigma_rn: self.config.sigma_r0n,
            omega_rn_n_radps: Vector3::zeros(),
            domega_rn_n_radps2: Vector3::zeros(),
        }
    }
}

impl Module for Inertial3D {
    fn init(&mut self) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
    }

    fn reset(&mut self, _context: &SimulationContext) {}

    fn update(&mut self, _context: &SimulationContext) {
        self.att_ref_out_msg.write(self.pointing_reference());
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    use super::{Inertial3D, Inertial3DConfig};

    #[test]
    fn matches_upstream_inertial_3d_unit_test() {
        let expected_sigma = Vector3::new(0.1, 0.2, 0.3);
        let mut module = Inertial3D::new(Inertial3DConfig {
            name: "inertial3D".to_string(),
            sigma_r0n: expected_sigma,
        });
        let mut recorder = MessageRecorder::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&module.att_ref_out_msg, &mut recorder.input_msg);
        simulation.add_module("inertial3D", &mut module, 500_000_000, 0);
        simulation.add_module("recorder", &mut recorder, 500_000_000, 0);
        simulation.run_for(1_000_000_000);
        drop(simulation);

        assert_eq!(recorder.samples.len(), 3);
        for sample in recorder.samples {
            for index in 0..3 {
                assert!((sample.sigma_rn[index] - expected_sigma[index]).abs() <= 1.0e-12);
                assert!(sample.omega_rn_n_radps[index].abs() <= 1.0e-12);
                assert!(sample.domega_rn_n_radps2[index].abs() <= 1.0e-12);
            }
        }
    }

    #[test]
    fn reset_preserves_the_last_reference_output() {
        let mut module = Inertial3D::new(Inertial3DConfig {
            name: "inertial3D".to_string(),
            sigma_r0n: Vector3::new(0.1, 0.2, 0.3),
        });
        let context = SimulationContext {
            current_sim_nanos: 500_000_000,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        };
        module.init();
        module.update(&context);
        let before = module.att_ref_out_msg.read();

        Module::reset(&mut module, &context);
        let after = module.att_ref_out_msg.read();

        assert_eq!(after.sigma_rn, before.sigma_rn);
        assert_eq!(after.omega_rn_n_radps, before.omega_rn_n_radps);
        assert_eq!(after.domega_rn_n_radps2, before.domega_rn_n_radps2);
    }
}
