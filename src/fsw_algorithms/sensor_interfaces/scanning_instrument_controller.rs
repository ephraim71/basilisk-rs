//! Continuous instrument control: port of upstream Basilisk
//! `scanningInstrumentController`.
//!
//! Every update commands the instrument on (`device_cmd = 1`) while the
//! controller is active, the target is accessible, the attitude error norm
//! `|σ_BR|` is within tolerance and, if enabled, so is the rate error norm
//! `|ω_BR|`; otherwise it commands it off. A typical use is continuous nadir
//! imaging (scanning). An optional device status message overrides the
//! controller status.

use crate::messages::{
    AccessMsg, AttitudeGuidanceMsg, DeviceCommandMsg, DeviceStatusMsg, Input, Output,
};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`ScanningInstrumentController`]. `Default` is
/// upstream's zero-initialized configuration, in which the controller is
/// inactive.
#[derive(Clone, Debug)]
pub struct ScanningInstrumentControllerConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `attErrTolerance` \[-\]: largest attitude error MRP norm
    /// `|σ_BR|` at which the instrument may run.
    pub att_err_tolerance: f64,
    /// Upstream `useRateTolerance`: also require `|ω_BR|` within
    /// `rate_err_tolerance_radps`.
    pub use_rate_tolerance: bool,
    /// Upstream `rateErrTolerance` \[rad/s\]: largest rate error norm at which
    /// the instrument may run, when `use_rate_tolerance` is set.
    pub rate_err_tolerance_radps: f64,
    /// Upstream `controllerStatus`: whether the controller runs. When
    /// `device_status_in_msg` is connected, every update overwrites it with
    /// the device status, as upstream.
    pub controller_status: bool,
}

impl Default for ScanningInstrumentControllerConfig {
    fn default() -> Self {
        Self {
            name: "scanningInstrumentController".to_string(),
            att_err_tolerance: 0.0,
            use_rate_tolerance: false,
            rate_err_tolerance_radps: 0.0,
            controller_status: false,
        }
    }
}

/// Continuous instrument on/off controller: upstream
/// `scanningInstrumentController`.
#[derive(Clone, Debug)]
pub struct ScanningInstrumentController {
    /// User-settable parameters.
    pub config: ScanningInstrumentControllerConfig,
    /// Upstream `accessInMsg`: ground-location access.
    pub access_in_msg: Input<AccessMsg>,
    /// Upstream `attGuidInMsg`: attitude tracking errors.
    pub att_guid_in_msg: Input<AttitudeGuidanceMsg>,
    /// Upstream `deviceStatusInMsg` (optional): device status that, when
    /// connected, sets the controller status.
    pub device_status_in_msg: Input<DeviceStatusMsg>,
    /// Upstream `deviceCmdOutMsg`: instrument command, 1 on and 0 off.
    pub device_cmd_out_msg: Output<DeviceCommandMsg>,
}

impl ScanningInstrumentController {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: ScanningInstrumentControllerConfig) -> Self {
        Self {
            config,
            access_in_msg: Input::default(),
            att_guid_in_msg: Input::default(),
            device_status_in_msg: Input::default(),
            device_cmd_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_scanningInstrumentController`: it only checks the
    /// required inputs and leaves the controller status as it is.
    fn check_connections(&self) {
        if !self.access_in_msg.is_connected() {
            log::error!("scanningInstrumentController.access_in_msg is not connected");
        }
        if !self.att_guid_in_msg.is_connected() {
            log::error!("scanningInstrumentController.att_guid_in_msg is not connected");
        }
    }
}

/// The pointing and access condition shared with `simpleInstrumentController`:
/// `|σ_BR| <= att_err_tolerance`, `|ω_BR| <= rate_err_tolerance_radps` when a
/// rate tolerance is given, and access to the target. Bounds are inclusive.
pub(crate) fn imaging_constraints_met(
    att_err_tolerance: f64,
    rate_err_tolerance_radps: Option<f64>,
    guidance: &AttitudeGuidanceMsg,
    access: &AccessMsg,
) -> bool {
    guidance.sigma_br.norm() <= att_err_tolerance
        && rate_err_tolerance_radps
            .is_none_or(|tolerance| guidance.omega_br_b_radps.norm() <= tolerance)
        && access.has_access
}

impl Module for ScanningInstrumentController {
    fn init(&mut self) {
        self.device_cmd_out_msg.write(DeviceCommandMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let access = self.access_in_msg.read();
        let guidance = self.att_guid_in_msg.read();
        if self.device_status_in_msg.is_connected() {
            self.config.controller_status = self.device_status_in_msg.read().device_status.is_on();
        }

        let config = &self.config;
        let instrument_on = config.controller_status
            && imaging_constraints_met(
                config.att_err_tolerance,
                config
                    .use_rate_tolerance
                    .then_some(config.rate_err_tolerance_radps),
                &guidance,
                &access,
            );
        self.device_cmd_out_msg.write(DeviceCommandMsg {
            device_cmd: u64::from(instrument_on),
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{ScanningInstrumentController, ScanningInstrumentControllerConfig};
    use crate::messages::{AccessMsg, AttitudeGuidanceMsg, DeviceState, DeviceStatusMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(1.0)`.
    const TASK_RATE_NANOS: u64 = 1_000_000_000;

    /// One `test_scanningInstrumentController` parametrization, in the
    /// pytest's order: `att_limit, att_mag, use_rate_limit, rate_limit,
    /// omega_mag, deviceStatus, controlStatus, expected_result`.
    type Case = (f64, f64, u8, f64, f64, Option<u8>, Option<u8>, [u64; 3]);

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn device_state(status: u8) -> DeviceState {
        if status == 0 {
            DeviceState::Off
        } else {
            DeviceState::On
        }
    }

    /// Upstream `scanningInstrumentControllerTestFunction`: a 1 s task run to
    /// 1 s and then to 2 s, recording the command at 0, 1 and 2 s.
    fn run_upstream_case(case: &Case) -> Vec<u64> {
        let &(
            att_limit,
            att_mag,
            use_rate_limit,
            rate_limit,
            omega_mag,
            device_status,
            control_status,
            _,
        ) = case;
        let access_output = Output::new(AccessMsg {
            has_access: true,
            ..Default::default()
        });
        let guidance_output = Output::new(AttitudeGuidanceMsg {
            sigma_br: Vector3::new(att_mag, 0.0, 0.0),
            omega_br_b_radps: Vector3::new(omega_mag, 0.0, 0.0),
            ..Default::default()
        });
        let status_output = device_status.map(|status| {
            Output::new(DeviceStatusMsg {
                device_status: device_state(status),
            })
        });

        let mut module = ScanningInstrumentController::new(ScanningInstrumentControllerConfig {
            att_err_tolerance: att_limit,
            use_rate_tolerance: use_rate_limit != 0,
            rate_err_tolerance_radps: rate_limit,
            ..Default::default()
        });
        if let Some(status) = control_status {
            module.config.controller_status = status != 0;
        }
        let mut recorder = MessageRecorder::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        if let Some(status_output) = &status_output {
            simulation.connect(status_output, &mut module.device_status_in_msg);
        }
        simulation.connect(&access_output, &mut module.access_in_msg);
        simulation.connect(&guidance_output, &mut module.att_guid_in_msg);
        simulation.connect(&module.device_cmd_out_msg, &mut recorder.input_msg);
        simulation.add_module(
            "scanningInstrumentController",
            &mut module,
            TASK_RATE_NANOS,
            0,
        );
        simulation.add_module("recorder", &mut recorder, TASK_RATE_NANOS, 0);
        simulation.run_for(TASK_RATE_NANOS);
        simulation.run_for(TASK_RATE_NANOS);
        drop(simulation);

        recorder
            .samples
            .iter()
            .map(|sample| sample.device_cmd)
            .collect()
    }

    /// `test_scanningInstrumentController`, all ten parametrizations, with
    /// upstream's expected commands (exact, as `assert_array_equal`).
    #[test]
    fn matches_upstream_scanning_instrument_controller_unit_test() {
        let cases: [Case; 10] = [
            // 1: attitude compliant, rate disabled, device status 1, controller status 1
            (0.1, 0.01, 0, 0.0, 0.0, Some(1), Some(1), [1, 1, 1]),
            // 2: attitude noncompliant, rate disabled, device status 1, controller status 1
            (0.1, 0.2, 0, 0.0, 0.0, Some(1), Some(1), [0, 0, 0]),
            // 3: attitude compliant, rate disabled, no device status, no controller status
            (0.1, 0.01, 0, 0.0, 0.0, None, None, [0, 0, 0]),
            // 4: attitude compliant, rate disabled, device status 1, controller status 0
            (0.1, 0.01, 0, 0.0, 0.0, Some(1), Some(0), [1, 1, 1]),
            // 5: attitude compliant, rate disabled, device status 0, controller status 1
            (0.1, 0.01, 0, 0.0, 0.0, Some(0), Some(1), [0, 0, 0]),
            // 6: attitude compliant, rate disabled, no device status, controller status 1
            (0.1, 0.01, 0, 0.0, 0.0, None, Some(1), [1, 1, 1]),
            // 7: rate disabled and noncompliant, no device status, controller status 1
            (0.1, 0.01, 0, 0.01, 0.1, None, Some(1), [1, 1, 1]),
            // 8: rate enabled and noncompliant, no device status, controller status 1
            (0.1, 0.01, 1, 0.01, 0.1, None, Some(1), [0, 0, 0]),
            // 9: rate enabled and compliant, device status 1, no controller status
            (0.1, 0.01, 1, 0.01, 0.001, Some(1), None, [1, 1, 1]),
            // 10: attitude noncompliant, rate enabled and compliant
            (0.1, 0.2, 1, 0.01, 0.001, Some(1), None, [0, 0, 0]),
        ];
        for (index, case) in cases.iter().enumerate() {
            assert_eq!(run_upstream_case(case), case.7, "test case {}", index + 1);
        }
    }

    // Not upstream: the tolerances are inclusive, the command follows the
    // constraints from one update to the next, and the controller status a
    // device status message wrote into the config survives a reset.
    #[test]
    fn command_tracks_the_constraints_and_the_device_status() {
        let access_output = Output::new(AccessMsg {
            has_access: true,
            ..Default::default()
        });
        let guidance_output = Output::new(AttitudeGuidanceMsg {
            sigma_br: Vector3::new(0.0, 0.25, 0.0),
            omega_br_b_radps: Vector3::new(0.0, 0.0, 0.5),
            ..Default::default()
        });
        let status_output = Output::new(DeviceStatusMsg {
            device_status: DeviceState::On,
        });
        let mut module = ScanningInstrumentController::new(ScanningInstrumentControllerConfig {
            att_err_tolerance: 0.25,
            use_rate_tolerance: true,
            rate_err_tolerance_radps: 0.5,
            ..Default::default()
        });
        access_output.connect_to(&mut module.access_in_msg);
        guidance_output.connect_to(&mut module.att_guid_in_msg);
        status_output.connect_to(&mut module.device_status_in_msg);
        module.init();
        let command =
            |module: &ScanningInstrumentController| module.device_cmd_out_msg.read().device_cmd;

        module.update(&context(0));
        assert_eq!(command(&module), 1, "errors on the tolerance bounds");

        access_output.write(AccessMsg::default());
        module.update(&context(1));
        assert_eq!(command(&module), 0, "no access");

        access_output.write(AccessMsg {
            has_access: true,
            ..Default::default()
        });
        module.update(&context(2));
        assert_eq!(command(&module), 1, "access regained");

        status_output.write(DeviceStatusMsg {
            device_status: DeviceState::Off,
        });
        module.update(&context(3));
        assert_eq!(command(&module), 0, "device off");
        assert!(!module.config.controller_status);
        Module::reset(&mut module, &context(4));
        assert!(!module.config.controller_status);
    }
}
