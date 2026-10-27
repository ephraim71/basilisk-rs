//! Sun-safe attitude control with thrusters: port of upstream Basilisk
//! `sunSafeACS` (`fswAlgorithms/effectorInterfaces/errorConversion/sunSafeACS.c`).
//!
//! Maps the commanded body torque of the sun-safe controller onto thruster
//! on-times for a single thruster group, with the same mapping as
//! [`super::dv_att_effect`] (upstream `computeSingleThrustBlock`).
//!
//! Upstream quirk kept: unlike `dvAttEffect`, Reset only checks the input and
//! leaves the last on-time command on the output message.

use crate::messages::{BodyTorqueCommandMsg, Input, Output, ThrusterArrayOnTimeCommandMsg};
use crate::{Module, SimulationContext};

use super::dv_att_effect::ThrustGroupConfig;

/// User-settable parameters of [`SunSafeAcs`].
#[derive(Clone, Debug)]
pub struct SunSafeAcsConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `thrData`: the thruster group.
    pub thr_data: ThrustGroupConfig,
}

impl Default for SunSafeAcsConfig {
    fn default() -> Self {
        Self {
            name: "sunSafeACS".to_string(),
            thr_data: ThrustGroupConfig::default(),
        }
    }
}

/// Sun-safe thruster control (upstream `sunSafeACSConfig`).
#[derive(Clone, Debug)]
pub struct SunSafeAcs {
    /// Module configuration, read at every update.
    pub config: SunSafeAcsConfig,
    /// Upstream `cmdTorqueBodyInMsg`.
    pub cmd_torque_body_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `thrData.thrOnTimeOutMsg`.
    pub thr_on_time_out_msg: Output<ThrusterArrayOnTimeCommandMsg>,
}

impl SunSafeAcs {
    /// Builds the module with an unconnected input.
    pub fn new(config: SunSafeAcsConfig) -> Self {
        Self {
            config,
            cmd_torque_body_in_msg: Input::default(),
            thr_on_time_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_sunSafeACS`: only checks the input.
    pub fn reset(&mut self, _call_time_nanos: u64) {
        if !self.cmd_torque_body_in_msg.is_connected() {
            log::error!("sunSafeACS.cmd_torque_body_in_msg is not connected");
        }
        self.config.thr_data.check_capacity("sunSafeACS");
    }
}

impl Module for SunSafeAcs {
    fn init(&mut self) {
        self.thr_on_time_out_msg
            .write(ThrusterArrayOnTimeCommandMsg::default());
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        SunSafeAcs::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, _context: &SimulationContext) {
        let torque_request_body_nm = self.cmd_torque_body_in_msg.read().torque_request_body_nm;
        self.thr_on_time_out_msg
            .write(self.config.thr_data.on_time_command(torque_request_body_nm));
    }
}

#[cfg(test)]
mod tests {
    //! Upstream's only test (`test_errorConversionImports.py`) checks that the
    //! Python wrapper imports; these tests are written from the algorithm,
    //! with hand-derived truth.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{BodyTorqueCommandMsg, MAX_EFF_COUNT, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    use super::super::dv_att_effect::ThrustGroupConfig;
    use super::{SunSafeAcs, SunSafeAcsConfig};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Six thrusters, a +/- pair per body axis, with a 0.5 s/(N m) gain.
    fn six_thruster_group() -> ThrustGroupConfig {
        let axes = [Vector3::x(), Vector3::y(), Vector3::z()];
        ThrustGroupConfig {
            nom_thrust_on_s: 0.0,
            max_num_cmds: 2,
            min_thrust_request_s: 0.01,
            thr_on_map: axes
                .iter()
                .flat_map(|axis| [0.5 * axis, -0.5 * axis])
                .collect(),
        }
    }

    #[test]
    fn maps_the_sun_safe_torque_onto_the_longest_on_times() {
        let command = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::new(0.2, -0.6, 0.01),
        });
        let mut module = SunSafeAcs::new(SunSafeAcsConfig {
            thr_data: six_thruster_group(),
            ..Default::default()
        });
        let mut recorder = MessageRecorder::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&command, &mut module.cmd_torque_body_in_msg);
        simulation.connect(&module.thr_on_time_out_msg, &mut recorder.input_msg);
        simulation.add_module("sunSafeACS", &mut module, 500_000_000, 0);
        simulation.add_module("recorder", &mut recorder, 500_000_000, 0);
        simulation.run_for(1_000_000_000);

        // On-times 0.1, -0.1 -> 0, -0.3 -> 0, 0.3, 0.005 -> 0 (below the
        // 0.01 minimum), -0.005 -> 0; the two longest are thrusters 3 and 0.
        let mut expected = [0.0; MAX_EFF_COUNT];
        expected[0] = 0.1;
        expected[3] = 0.3;
        assert!(!recorder.samples.is_empty());
        for sample in &recorder.samples {
            assert_eq!(sample.on_time_request_s, expected);
        }
    }

    #[test]
    fn nominal_on_time_is_added_before_the_minimum_check() {
        let command = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::new(0.0, 0.0, 0.01),
        });
        let mut module = SunSafeAcs::new(SunSafeAcsConfig {
            thr_data: ThrustGroupConfig {
                nom_thrust_on_s: 0.008,
                max_num_cmds: 6,
                ..six_thruster_group()
            },
            ..Default::default()
        });
        command.connect_to(&mut module.cmd_torque_body_in_msg);
        module.init();
        module.update(&context(0));

        // 0.008 alone is below 0.01; only thruster 4 (0.005 + 0.008) passes.
        let on_times = module.thr_on_time_out_msg.read().on_time_request_s;
        assert!((on_times[4] - 0.013).abs() <= 1e-15);
        assert!(
            on_times
                .iter()
                .enumerate()
                .all(|(index, on_time)| index == 4 || *on_time == 0.0)
        );
    }

    #[test]
    fn reset_keeps_the_last_command_like_upstream() {
        let command = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::new(0.2, -0.6, 0.01),
        });
        let mut module = SunSafeAcs::new(SunSafeAcsConfig {
            thr_data: six_thruster_group(),
            ..Default::default()
        });
        command.connect_to(&mut module.cmd_torque_body_in_msg);
        module.init();
        module.update(&context(0));
        let before = module.thr_on_time_out_msg.read();
        module.reset(500_000_000);
        assert_eq!(module.thr_on_time_out_msg.read(), before);
    }
}
