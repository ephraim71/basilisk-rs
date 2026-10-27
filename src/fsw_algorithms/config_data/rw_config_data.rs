//! Converts the reaction-wheel constellation description into the array form
//! that flight-software consumers read.
//!
//! Port of upstream Basilisk `rwConfigData`. On every reset the module reads
//! the optional [`RwConstellationMsg`] (zero wheels when it is not connected),
//! copies each wheel's spin axis, spin inertia and torque limit into an
//! [`RwArrayConfigMsg`] and publishes it. Updates do nothing.

use crate::messages::{Input, Output, RwArrayConfigMsg, RwConstellationMsg};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`RwConfigData`] (upstream has none).
#[derive(Clone, Debug)]
pub struct RwConfigDataConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
}

impl Default for RwConfigDataConfig {
    fn default() -> Self {
        Self {
            name: "rwConfigData".to_string(),
        }
    }
}

/// Static publisher of the reaction-wheel array configuration.
#[derive(Clone, Debug)]
pub struct RwConfigData {
    /// Module configuration.
    pub config: RwConfigDataConfig,
    /// Wheel constellation, optional (upstream `rwConstellationInMsg`).
    pub rw_constellation_in_msg: Input<RwConstellationMsg>,
    /// Wheel array configuration (upstream `rwParamsOutMsg`).
    pub rw_params_out_msg: Output<RwArrayConfigMsg>,
    /// Retained across resets: upstream overwrites only `num_rw` and the first
    /// `num_rw` wheels, so entries of a larger earlier constellation persist.
    rw_config_params_out: RwArrayConfigMsg,
}

impl RwConfigData {
    /// Creates the module with an unconnected constellation input.
    pub fn new(config: RwConfigDataConfig) -> Self {
        Self {
            config,
            rw_constellation_in_msg: Input::default(),
            rw_params_out_msg: Output::default(),
            rw_config_params_out: RwArrayConfigMsg::default(),
        }
    }

    fn publish_config(&mut self) {
        let constellation = if self.rw_constellation_in_msg.is_connected() {
            self.rw_constellation_in_msg.read()
        } else {
            RwConstellationMsg::default()
        };

        let output = &mut self.rw_config_params_out;
        output.num_rw = constellation.num_rw;
        // `active_wheels` stops at MAX_EFF_COUNT, where upstream would overrun.
        for (index, wheel) in constellation.active_wheels().iter().enumerate() {
            output.spin_axis_inertias_kg_m2[index] = wheel.spin_axis_inertia_kg_m2;
            output.max_motor_torques_nm[index] = wheel.max_motor_torque_nm;
            output.spin_axes_body[index] = wheel.gs_hat_b;
        }
        self.rw_params_out_msg.write(output.clone());
    }
}

impl Module for RwConfigData {
    fn init(&mut self) {
        self.publish_config();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.publish_config();
    }

    fn update(&mut self, _context: &SimulationContext) {}
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{Output, RwConfigElementMsg, RwConstellationMsg};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    use super::{RwConfigData, RwConfigDataConfig};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn wheel(axis: Vector3<f64>, js: f64, u_max: f64) -> RwConfigElementMsg {
        RwConfigElementMsg {
            gs_hat_b: axis,
            spin_axis_inertia_kg_m2: js,
            max_motor_torque_nm: u_max,
        }
    }

    /// Port of `test_rwConfigData.py`: 0.5 s task rate, stop after one period,
    /// both samples carry the constellation's Js, uMax and spin axes.
    #[test]
    fn matches_upstream_rw_config_data_unit_test() {
        let gs_hat = [Vector3::x(), Vector3::y(), Vector3::z()];
        let js = [0.08, 0.09, 0.07];
        let u_max = [0.2, 0.1, 0.3];
        let wheels: Vec<_> = (0..3).map(|i| wheel(gs_hat[i], js[i], u_max[i])).collect();
        let constellation = Output::new(RwConstellationMsg::from_active(&wheels));

        let mut module = RwConfigData::new(RwConfigDataConfig::default());
        let mut recorder = MessageRecorder::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&constellation, &mut module.rw_constellation_in_msg);
        simulation.connect(&module.rw_params_out_msg, &mut recorder.input_msg);
        simulation.add_module("rwConfigData", &mut module, 500_000_000, 0);
        simulation.add_module("recorder", &mut recorder, 500_000_000, 0);
        simulation.run_for(500_000_000);
        drop(simulation);

        let accuracy = 1e-6;
        assert_eq!(recorder.samples.len(), 2);
        for sample in &recorder.samples {
            assert_eq!(sample.num_rw, 3);
            for i in 0..3 {
                assert!((sample.spin_axis_inertias_kg_m2[i] - js[i]).abs() <= accuracy);
                assert!((sample.max_motor_torques_nm[i] - u_max[i]).abs() <= accuracy);
                assert!((sample.spin_axes_body[i] - gs_hat[i]).amax() <= accuracy);
            }
        }
    }

    #[test]
    fn unconnected_constellation_publishes_zero_wheels() {
        let mut module = RwConfigData::new(RwConfigDataConfig::default());
        module.init();
        module.update(&context(0));
        assert_eq!(module.rw_params_out_msg.read().num_rw, 0);
    }

    #[test]
    fn reset_keeps_entries_beyond_a_smaller_constellation() {
        let constellation = Output::new(RwConstellationMsg::from_active(&[
            wheel(Vector3::x(), 0.1, 0.2),
            wheel(Vector3::y(), 0.3, 0.4),
        ]));
        let mut module = RwConfigData::new(RwConfigDataConfig::default());
        constellation.connect_to(&mut module.rw_constellation_in_msg);
        module.init();

        constellation.write(RwConstellationMsg::from_active(&[wheel(
            Vector3::z(),
            0.5,
            0.6,
        )]));
        Module::reset(&mut module, &context(1_000_000_000));

        // Upstream quirk: only num_rw and wheel 0 are rewritten; wheel 1 stays.
        let output = module.rw_params_out_msg.read();
        assert_eq!(output.num_rw, 1);
        assert_eq!(output.spin_axes_body[0], Vector3::z());
        assert_eq!(output.spin_axis_inertias_kg_m2[..2], [0.5, 0.3]);
        assert_eq!(output.max_motor_torques_nm[..2], [0.6, 0.4]);
        assert_eq!(output.spin_axes_body[1], Vector3::y());
    }
}
