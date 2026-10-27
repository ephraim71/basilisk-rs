//! Deputy state relative to the chief in the chief's Hill frame: port of
//! upstream Basilisk `hillStateConverter`.
//!
//! Every update reads the chief's and the deputy's inertial navigation states
//! and publishes the deputy's position relative to the chief and its time
//! derivative as seen in the rotating Hill frame, both in Hill components
//! ([`state_to_hill`], upstream `rv2hill`). The conversion makes no circular
//! orbit assumption.

use crate::messages::{HillRelativeStateMsg, Input, NavigationTranslationMsg, Output};
use crate::utilities::orbital_motion::state_to_hill;
use crate::{Module, SimulationContext};

/// Configuration of [`HillStateConverter`]; upstream has no parameters.
#[derive(Clone, Debug)]
pub struct HillStateConverterConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
}

impl Default for HillStateConverterConfig {
    fn default() -> Self {
        Self {
            name: "hillStateConverter".to_string(),
        }
    }
}

/// Inertial chief and deputy states to a Hill-frame relative state: upstream
/// `hillStateConverter`.
#[derive(Clone, Debug)]
pub struct HillStateConverter {
    /// Module configuration.
    pub config: HillStateConverterConfig,
    /// Upstream `chiefStateInMsg`: chief inertial state, which defines the
    /// Hill frame.
    pub chief_state_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `depStateInMsg`: deputy inertial state.
    pub dep_state_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `hillStateOutMsg`: deputy state relative to the chief, Hill
    /// components.
    pub hill_state_out_msg: Output<HillRelativeStateMsg>,
}

impl HillStateConverter {
    /// Builds the module with unconnected ports.
    pub fn new(config: HillStateConverterConfig) -> Self {
        Self {
            config,
            chief_state_in_msg: Input::default(),
            dep_state_in_msg: Input::default(),
            hill_state_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_hillStateConverter`: the module keeps no state, so a
    /// reset only reports missing inputs.
    fn check_connections(&self) {
        if !self.chief_state_in_msg.is_connected() {
            log::error!("hillStateConverter.chief_state_in_msg is not connected");
        }
        if !self.dep_state_in_msg.is_connected() {
            log::error!("hillStateConverter.dep_state_in_msg is not connected");
        }
    }
}

impl Module for HillStateConverter {
    fn init(&mut self) {
        self.hill_state_out_msg
            .write(HillRelativeStateMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let chief = self.chief_state_in_msg.read();
        let deputy = self.dep_state_in_msg.read();
        let (position_dc_h_m, velocity_dc_h_mps) = state_to_hill(
            chief.position_bn_n_m,
            chief.velocity_bn_n_mps,
            deputy.position_bn_n_m,
            deputy.velocity_bn_n_mps,
        );
        self.hill_state_out_msg.write(HillRelativeStateMsg {
            position_dc_h_m,
            velocity_dc_h_mps,
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{HillStateConverter, HillStateConverterConfig};
    use crate::messages::{HillRelativeStateMsg, NavigationTranslationMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(1.0)`.
    const TASK_RATE_NANOS: u64 = 1_000_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn navigation(position: [f64; 3], velocity: [f64; 3]) -> NavigationTranslationMsg {
        NavigationTranslationMsg {
            position_bn_n_m: Vector3::from(position),
            velocity_bn_n_mps: Vector3::from(velocity),
            ..Default::default()
        }
    }

    /// `pytest.approx` with its defaults: relative 1e-6, absolute 1e-12.
    fn assert_approx(actual: Vector3<f64>, expected: [f64; 3], what: &str) {
        for (index, (value, truth)) in actual.iter().zip(expected).enumerate() {
            let tolerance = (1e-6 * truth.abs()).max(1e-12);
            assert!(
                (value - truth).abs() <= tolerance,
                "{what}[{index}] = {value:.17e}, truth {truth:.17e}"
            );
        }
    }

    fn connected_module(
        chief_out: &Output<NavigationTranslationMsg>,
        deputy_out: &Output<NavigationTranslationMsg>,
    ) -> HillStateConverter {
        let mut module = HillStateConverter::new(HillStateConverterConfig::default());
        chief_out.connect_to(&mut module.chief_state_in_msg);
        deputy_out.connect_to(&mut module.dep_state_in_msg);
        module
    }

    /// Upstream `test_hillStateConverter.py`: a 1 s task run to t = 1 s with
    /// the deputy 1 m radially out and 0.01 m/s faster along-track; the last
    /// recorded sample is checked against upstream's hard-coded truth with
    /// `pytest.approx`.
    #[test]
    fn matches_upstream_hill_state_converter_unit_test() {
        let chief_out = Output::new(navigation([7100.0, 0.0, 0.0], [0.0, 7.000, 0.0]));
        let deputy_out = Output::new(navigation([7101.0, 0.0, 0.0], [0.0, 7.010, 0.0]));
        let mut module = connected_module(&chief_out, &deputy_out);
        let mut recorder = MessageRecorder::<HillRelativeStateMsg>::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&module.hill_state_out_msg, &mut recorder.input_msg);
        simulation.add_module("hillStateConverter", &mut module, TASK_RATE_NANOS, 0);
        simulation.add_module("recorder", &mut recorder, TASK_RATE_NANOS, 0);
        simulation.run_for(TASK_RATE_NANOS);

        assert_eq!(recorder.samples.len(), 2);
        let last = recorder.samples.last().expect("two samples were recorded");
        assert_approx(last.position_dc_h_m, [1.0, 0.0, 0.0], "r_DC_H");
        assert_approx(last.velocity_dc_h_mps, [0.0, 0.00901408, 0.0], "v_DC_H");
    }

    /// Not upstream: an inclined, eccentric chief, with truth built from the
    /// documented equations (`[HN]` from the radial and orbit-normal unit
    /// vectors, transport theorem with the Hill frame rate `|r x v| / r^2`).
    #[test]
    fn applies_the_transport_theorem_for_a_general_orbit() {
        let rc: Vector3<f64> = Vector3::new(6.8e6, 1.2e6, -0.9e6);
        let vc: Vector3<f64> = Vector3::new(-1.1e3, 7.2e3, 2.3e3);
        let rd = rc + Vector3::new(120.0, -40.0, 75.0);
        let vd = vc + Vector3::new(0.3, -0.1, 0.05);

        let h_r = rc / rc.norm();
        let h_h = rc.cross(&vc) / rc.cross(&vc).norm();
        let h_theta = h_h.cross(&h_r);
        let hn = Matrix3::from_rows(&[h_r.transpose(), h_theta.transpose(), h_h.transpose()]);
        let omega_hn_h = Vector3::new(0.0, 0.0, rc.cross(&vc).norm() / rc.norm_squared());
        let rho_h = hn * (rd - rc);
        let rho_prime_h = hn * (vd - vc) - omega_hn_h.cross(&rho_h);

        let chief_out = Output::new(NavigationTranslationMsg {
            position_bn_n_m: rc,
            velocity_bn_n_mps: vc,
            ..Default::default()
        });
        let deputy_out = Output::new(NavigationTranslationMsg {
            position_bn_n_m: rd,
            velocity_bn_n_mps: vd,
            ..Default::default()
        });
        let mut module = connected_module(&chief_out, &deputy_out);
        module.init();
        module.update(&context(0));

        let output = module.hill_state_out_msg.read();
        assert!((output.position_dc_h_m - rho_h).amax() < 1e-9);
        assert!((output.velocity_dc_h_mps - rho_prime_h).amax() < 1e-12);
        // The relative state is frame-independent in magnitude.
        assert!((output.position_dc_h_m.norm() - (rd - rc).norm()).abs() < 1e-9);
    }

    #[test]
    fn init_publishes_a_zero_relative_state() {
        let mut module = HillStateConverter::new(HillStateConverterConfig::default());
        module.hill_state_out_msg.write(HillRelativeStateMsg {
            position_dc_h_m: Vector3::new(1.0, 2.0, 3.0),
            velocity_dc_h_mps: Vector3::new(4.0, 5.0, 6.0),
        });
        module.init();
        assert_eq!(
            module.hill_state_out_msg.read(),
            HillRelativeStateMsg::default()
        );
    }
}
