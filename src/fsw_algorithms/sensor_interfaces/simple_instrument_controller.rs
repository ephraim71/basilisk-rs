//! Single-shot imaging command: port of upstream Basilisk
//! `simpleInstrumentController`.
//!
//! While the controller is active and the target has not been imaged yet, the
//! module commands one image (`device_cmd = 1` for a single update) as soon as
//! the target is accessible and the attitude error norm `|σ_BR|` (and, if
//! enabled, the rate error norm `|ω_BR|`) is within tolerance. Two optional
//! refinements restrict the capture:
//!
//! * a time window `[imaging_time - lower, imaging_time + upper]` in which the
//!   capture must happen (a zero tolerance disables that bound);
//! * duration-based imaging, where the constraints must hold continuously for
//!   `acquisition_time_nanos` before the capture, and the attempt fails (and
//!   the controller deactivates) once `allowed_time_nanos` has elapsed.
//!
//! An optional device status message overrides the controller status.

use super::scanning_instrument_controller::imaging_constraints_met;
use crate::messages::{
    AccessMsg, AttitudeGuidanceMsg, DeviceCommandMsg, DeviceStatusMsg, Input, Output,
};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`SimpleInstrumentController`]. `Default` is
/// upstream's zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct SimpleInstrumentControllerConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `attErrTolerance` \[-\]: largest attitude error MRP norm
    /// `|σ_BR|` at which an image may be taken.
    pub att_err_tolerance: f64,
    /// Upstream `useRateTolerance`: also require `|ω_BR|` within
    /// `rate_err_tolerance_radps`.
    pub use_rate_tolerance: bool,
    /// Upstream `rateErrTolerance` \[rad/s\]: largest rate error norm at which
    /// an image may be taken, when `use_rate_tolerance` is set.
    pub rate_err_tolerance_radps: f64,
    /// Upstream `useDurationImaging`: require the constraints to hold for
    /// `acquisition_time_nanos` before imaging.
    pub use_duration_imaging: bool,
    /// Upstream `acquisitionTime` \[ns\] (a `double`): how long the constraints
    /// must hold before a duration-based capture. A negative value is set to
    /// zero, with a warning, at the first duration-based attempt.
    pub acquisition_time_nanos: f64,
    /// Upstream `allowedTime` \[ns\] (a `double`): how long a duration-based
    /// attempt may last before it fails. A negative value is set to zero, with
    /// a warning, at the first duration-based attempt.
    pub allowed_time_nanos: f64,
    /// Upstream `timeToleranceLower` \[ns\]: no capture earlier than
    /// `imaging_time_nanos` minus this; zero disables the bound.
    pub time_tolerance_lower_nanos: u64,
    /// Upstream `timeToleranceUpper` \[ns\]: no capture later than
    /// `imaging_time_nanos` plus this; zero disables the bound.
    pub time_tolerance_upper_nanos: u64,
    /// Upstream `imagingTime` \[ns\]: desired imaging time, which the two
    /// tolerances bound the capture window around.
    pub imaging_time_nanos: u64,
}

impl Default for SimpleInstrumentControllerConfig {
    fn default() -> Self {
        Self {
            name: "simpleInstrumentController".to_string(),
            att_err_tolerance: 0.0,
            use_rate_tolerance: false,
            rate_err_tolerance_radps: 0.0,
            use_duration_imaging: false,
            acquisition_time_nanos: 0.0,
            allowed_time_nanos: 0.0,
            time_tolerance_lower_nanos: 0,
            time_tolerance_upper_nanos: 0,
            imaging_time_nanos: 0,
        }
    }
}

/// Single-shot instrument controller: upstream `simpleInstrumentController`.
#[derive(Clone, Debug)]
pub struct SimpleInstrumentController {
    /// User-settable parameters.
    pub config: SimpleInstrumentControllerConfig,
    /// Upstream `locationAccessInMsg`: ground-location access.
    pub location_access_in_msg: Input<AccessMsg>,
    /// Upstream `attGuidInMsg`: attitude tracking errors.
    pub att_guid_in_msg: Input<AttitudeGuidanceMsg>,
    /// Upstream `deviceStatusInMsg` (optional): device status that, when
    /// connected, sets the controller status every update.
    pub device_status_in_msg: Input<DeviceStatusMsg>,
    /// Upstream `deviceCmdOutMsg`: 1 on the update that captures the image,
    /// 0 otherwise.
    pub device_cmd_out_msg: Output<DeviceCommandMsg>,
    /// Upstream `imaged`: the target has been imaged, so no further command
    /// is sent. Init and reset clear it; clear it by hand to image again.
    pub imaged: bool,
    /// Upstream `controllerStatus`: whether the controller runs. Init sets it,
    /// as upstream `SelfInit`, so set it to `false` after init (or through
    /// `device_status_in_msg`) to deactivate the controller; a failed
    /// duration-based attempt clears it. Reset leaves it as it is.
    pub controller_status: bool,
    /// Upstream `elapsedTime` \[ns\]: duration of the current or last
    /// duration-based attempt; a failed attempt sets it to
    /// `allowed_time_nanos`.
    pub elapsed_time_nanos: f64,
    /// Upstream `constraintStartTime` \[ns\] while `constraintsActive` is set:
    /// when the constraints of the running duration-based attempt were first
    /// met.
    attempt_start_nanos: Option<u64>,
}

impl SimpleInstrumentController {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: SimpleInstrumentControllerConfig) -> Self {
        Self {
            config,
            location_access_in_msg: Input::default(),
            att_guid_in_msg: Input::default(),
            device_status_in_msg: Input::default(),
            device_cmd_out_msg: Output::default(),
            imaged: false,
            controller_status: true,
            elapsed_time_nanos: 0.0,
            attempt_start_nanos: None,
        }
    }

    /// Upstream `Reset_simpleInstrumentController`.
    fn reset_state(&mut self) {
        if !self.location_access_in_msg.is_connected() {
            log::error!("simpleInstrumentController.location_access_in_msg is not connected");
        }
        if !self.att_guid_in_msg.is_connected() {
            log::error!("simpleInstrumentController.att_guid_in_msg is not connected");
        }
        self.imaged = false;
        self.attempt_start_nanos = None;
        self.elapsed_time_nanos = 0.0;
    }

    /// Whether `call_nanos` lies in the configured capture window. The
    /// comparisons come before the unsigned differences, so a lower tolerance
    /// larger than the imaging time does not underflow.
    fn in_capture_window(&self, call_nanos: u64) -> bool {
        let config = &self.config;
        let imaging_nanos = config.imaging_time_nanos;
        let too_early = config.time_tolerance_lower_nanos > 0
            && call_nanos < imaging_nanos
            && imaging_nanos - call_nanos > config.time_tolerance_lower_nanos;
        let too_late = config.time_tolerance_upper_nanos > 0
            && call_nanos > imaging_nanos
            && call_nanos - imaging_nanos > config.time_tolerance_upper_nanos;
        !(too_early || too_late)
    }

    /// One update of a duration-based attempt while the constraints hold;
    /// returns whether to capture now.
    fn duration_imaging_step(&mut self, call_nanos: u64, in_window: bool) -> bool {
        let start_nanos = *self.attempt_start_nanos.get_or_insert(call_nanos);
        self.clamp_negative_durations();

        let acquisition = self.config.acquisition_time_nanos;
        let allowed = self.config.allowed_time_nanos;
        // Both converted before subtracting, as upstream's doubles.
        self.elapsed_time_nanos = call_nanos as f64 - start_nanos as f64;
        let elapsed = self.elapsed_time_nanos;
        // Not `f64::min`: upstream's ternary keeps `acquisition` for NaN.
        let effective_image_time = if acquisition > allowed {
            allowed
        } else {
            acquisition
        };

        // Nested as upstream, so NaN durations take the same branches.
        if elapsed >= effective_image_time {
            if acquisition <= allowed && in_window && elapsed <= allowed {
                self.imaged = true;
                self.attempt_start_nanos = None;
                return true;
            } else if elapsed >= allowed {
                // The acquisition, or the wait for the capture window, took
                // longer than allowed: give up and deactivate.
                self.elapsed_time_nanos = allowed;
                self.imaged = false;
                self.controller_status = false;
                self.attempt_start_nanos = None;
            }
        }
        false
    }

    /// Upstream caps negative durations to zero inside the update, changing
    /// the configuration, so each warning is logged once.
    fn clamp_negative_durations(&mut self) {
        let config = &mut self.config;
        if config.acquisition_time_nanos < 0.0 {
            config.acquisition_time_nanos = 0.0;
            log::warn!(
                "simpleInstrumentController: acquisition_time_nanos is negative and has been set \
                 to zero"
            );
        }
        if config.allowed_time_nanos < 0.0 {
            config.allowed_time_nanos = 0.0;
            log::warn!(
                "simpleInstrumentController: allowed_time_nanos is negative and has been set to \
                 zero"
            );
        }
    }

    fn image_command(&mut self, call_nanos: u64) -> bool {
        let access = self.location_access_in_msg.read();
        let guidance = self.att_guid_in_msg.read();
        if self.device_status_in_msg.is_connected() {
            self.controller_status = self.device_status_in_msg.read().device_status.is_on();
        }
        if !self.controller_status || self.imaged {
            return false;
        }

        let config = &self.config;
        let constraints_met = imaging_constraints_met(
            config.att_err_tolerance,
            config
                .use_rate_tolerance
                .then_some(config.rate_err_tolerance_radps),
            &guidance,
            &access,
        );
        if !constraints_met {
            self.attempt_start_nanos = None;
            return false;
        }

        let in_window = self.in_capture_window(call_nanos);
        if self.config.use_duration_imaging {
            self.duration_imaging_step(call_nanos, in_window)
        } else if in_window {
            self.imaged = true;
            true
        } else {
            false
        }
    }
}

impl Module for SimpleInstrumentController {
    /// Upstream `SelfInit` (zeroed output, an active controller and a cleared
    /// attempt), then the reset.
    fn init(&mut self) {
        self.device_cmd_out_msg.write(DeviceCommandMsg::default());
        self.controller_status = true;
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, context: &SimulationContext) {
        let capture = self.image_command(context.current_sim_nanos);
        self.device_cmd_out_msg.write(DeviceCommandMsg {
            device_cmd: u64::from(capture),
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{SimpleInstrumentController, SimpleInstrumentControllerConfig};
    use crate::messages::{AccessMsg, AttitudeGuidanceMsg, DeviceState, DeviceStatusMsg, Output};
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(1.0)`.
    const TASK_RATE_NANOS: u64 = 1_000_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `macros.sec2nano`: `int(time*1E9+0.5)`, which truncates
    /// toward zero.
    fn sec2nano(time_s: f64) -> i64 {
        (time_s * 1e9 + 0.5) as i64
    }

    /// The keyword arguments of upstream `simpleInstrumentControllerTestFunction`.
    struct Params {
        use_rate_limit: u32,
        rate_limit: f64,
        omega_mag: f64,
        device_status: Option<u8>,
        control_status: Option<u32>,
        use_duration: Option<u32>,
        imaging_times_s: [f64; 2],
        time_tolerance_lower_s: Option<f64>,
        time_tolerance_upper_s: Option<f64>,
        imaging_time_s: Option<f64>,
    }

    impl Default for Params {
        fn default() -> Self {
            Self {
                use_rate_limit: 1,
                rate_limit: 0.01,
                omega_mag: 0.001,
                device_status: None,
                control_status: None,
                use_duration: None,
                imaging_times_s: [0.0; 2],
                time_tolerance_lower_s: None,
                time_tolerance_upper_s: None,
                imaging_time_s: None,
            }
        }
    }

    /// Upstream `simpleInstrumentControllerTestFunction`: a 1 s task run to
    /// 1 s, then to 2 s, then `imaged` is cleared and the run continues to
    /// 4 s. Returns the five recorded commands and the final elapsed time.
    fn run_upstream_test_function(params: &Params) -> (Vec<u64>, f64) {
        let access_output = Output::new(AccessMsg {
            has_access: true,
            ..Default::default()
        });
        let guidance_output = Output::new(AttitudeGuidanceMsg {
            sigma_br: Vector3::new(0.01, 0.01, 0.01),
            omega_br_b_radps: Vector3::new(params.omega_mag, 0.0, 0.0),
            ..Default::default()
        });
        let status_output = params.device_status.map(|status| {
            Output::new(DeviceStatusMsg {
                device_status: if status == 0 {
                    DeviceState::Off
                } else {
                    DeviceState::On
                },
            })
        });

        let mut module = SimpleInstrumentController::new(SimpleInstrumentControllerConfig {
            att_err_tolerance: 0.1,
            rate_err_tolerance_radps: params.rate_limit,
            use_rate_tolerance: params.use_rate_limit != 0,
            ..Default::default()
        });
        if let Some(status_output) = &status_output {
            status_output.connect_to(&mut module.device_status_in_msg);
        }
        // Set before initialization, as upstream; init overrides it.
        if let Some(status) = params.control_status {
            module.controller_status = status != 0;
        }
        if let Some(use_duration) = params.use_duration {
            module.config.use_duration_imaging = use_duration != 0;
            let [acquisition_s, allowed_s] = params.imaging_times_s;
            module.config.acquisition_time_nanos = sec2nano(acquisition_s) as f64;
            module.config.allowed_time_nanos = sec2nano(allowed_s) as f64;
        }
        if let Some(lower_s) = params.time_tolerance_lower_s {
            module.config.time_tolerance_lower_nanos = sec2nano(lower_s) as u64;
        }
        if let Some(upper_s) = params.time_tolerance_upper_s {
            module.config.time_tolerance_upper_nanos = sec2nano(upper_s) as u64;
        }
        if let Some(imaging_s) = params.imaging_time_s {
            module.config.imaging_time_nanos = sec2nano(imaging_s) as u64;
        }
        access_output.connect_to(&mut module.location_access_in_msg);
        guidance_output.connect_to(&mut module.att_guid_in_msg);

        module.init();
        let mut commands = Vec::new();
        for tick in 0..=4 {
            if tick == 3 {
                module.imaged = false;
            }
            module.update(&context(tick * TASK_RATE_NANOS));
            commands.push(module.device_cmd_out_msg.read().device_cmd);
        }
        (commands, module.elapsed_time_nanos)
    }

    /// One `test_simple_instrument_controller` parametrization, in the
    /// pytest's order: `use_rate_limit, rate_limit, omega_mag, deviceStatus,
    /// controlStatus, useDuration, imagingTimes, expected_result`.
    type Case = (
        u32,
        f64,
        f64,
        Option<u8>,
        Option<u32>,
        Option<u32>,
        [f64; 2],
        [u64; 5],
    );

    /// `test_simple_instrument_controller`, all fourteen parametrizations,
    /// compared as upstream with `isArrayEqual(..., 1e-12)` on integer
    /// commands, i.e. exactly. Where upstream passes `imagingTimes = None`
    /// the times are unused.
    #[test]
    fn matches_upstream_simple_instrument_controller_unit_test() {
        #[rustfmt::skip]
        let cases: [Case; 14] = [
            // rate disabled, device status not written, no controller status
            (0, 0.0, 0.0, None, None, None, [0.0, 0.0], [1, 0, 0, 1, 0]),
            // rate disabled, rate noncompliant
            (0, 0.01, 0.1, None, None, None, [0.0, 0.0], [1, 0, 0, 1, 0]),
            // rate enabled, rate compliant
            (1, 0.01, 0.001, None, None, None, [0.0, 0.0], [1, 0, 0, 1, 0]),
            // rate enabled, rate noncompliant
            (1, 0.01, 0.1, None, None, None, [0.0, 0.0], [0, 0, 0, 0, 0]),
            // device status of 1, controller status of 0
            (0, 0.0, 0.0, Some(1), Some(0), None, [0.0, 0.0], [1, 0, 0, 1, 0]),
            // device status of 0, controller status of 1
            (0, 0.0, 0.0, Some(0), Some(1), None, [0.0, 0.0], [0, 0, 0, 0, 0]),
            // device status not written, controller status of 1
            (0, 0.0, 0.0, None, Some(1), None, [0.0, 0.0], [1, 0, 0, 1, 0]),
            // immediate imaging (duration flag set to 0)
            (1, 0.01, 0.001, None, None, Some(0), [1.0, 2.0], [1, 0, 0, 1, 0]),
            // duration imaging with zero acquisition and allowed time
            (1, 0.01, 0.001, None, None, Some(1), [0.0, 0.0], [1, 0, 0, 1, 0]),
            // 1 s acquisition and allowed time
            (1, 0.01, 0.001, Some(1), None, Some(1), [1.0, 1.0], [0, 1, 0, 0, 1]),
            // acquisition time greater than allowed time
            (1, 0.01, 0.001, Some(1), None, Some(1), [1.0, 0.0], [0, 0, 0, 0, 0]),
            // acquisition time less than allowed time
            (1, 0.01, 0.001, Some(1), None, Some(1), [1.0, 2.0], [0, 1, 0, 0, 1]),
            // negative acquisition time, images immediately
            (1, 0.01, 0.001, Some(1), None, Some(1), [-1.0, 2.0], [1, 0, 0, 1, 0]),
            // negative allowed time (taken as zero), images immediately
            (1, 0.01, 0.001, Some(1), None, Some(1), [0.0, -1.0], [1, 0, 0, 1, 0]),
        ];
        for (index, case) in cases.into_iter().enumerate() {
            let (
                use_rate_limit,
                rate_limit,
                omega_mag,
                device_status,
                control_status,
                use_duration,
                imaging_times_s,
                expected,
            ) = case;
            let (commands, _) = run_upstream_test_function(&Params {
                use_rate_limit,
                rate_limit,
                omega_mag,
                device_status,
                control_status,
                use_duration,
                imaging_times_s,
                ..Default::default()
            });
            assert_eq!(commands, expected, "test case {}", index + 1);
        }
    }

    /// `test_simple_instrument_controller_time_window`, all five
    /// parametrizations: `timeToleranceLower`, `timeToleranceUpper`,
    /// `imagingTime` \[s\] and the expected commands.
    #[test]
    fn matches_upstream_time_window_unit_test() {
        let cases: [(f64, f64, f64, [u64; 5]); 5] = [
            // zero tolerances disable the time criteria
            (0.0, 0.0, 2.0, [1, 0, 0, 1, 0]),
            // lower tolerance only: image at or after imagingTime - lower
            (1.0, 0.0, 2.0, [0, 1, 0, 1, 0]),
            // upper tolerance only: do not image after imagingTime + upper
            (0.0, 0.5, 2.0, [1, 0, 0, 0, 0]),
            // two-sided interval: image within [1 s, 3 s]
            (1.0, 1.0, 2.0, [0, 1, 0, 1, 0]),
            // lower tolerance larger than imagingTime must not underflow
            (3.0, 1.0, 2.0, [1, 0, 0, 1, 0]),
        ];
        for (index, (lower_s, upper_s, imaging_s, expected)) in cases.into_iter().enumerate() {
            let (commands, _) = run_upstream_test_function(&Params {
                time_tolerance_lower_s: Some(lower_s),
                time_tolerance_upper_s: Some(upper_s),
                imaging_time_s: Some(imaging_s),
                ..Default::default()
            });
            assert_eq!(commands, expected, "time window case {}", index + 1);
        }
    }

    /// `test_simple_instrument_controller_duration_time_window`, all four
    /// parametrizations: duration imaging in the [1.5 s, 2.5 s] window, with
    /// `acquisitionTime`, `allowedTime` \[s\] and the expected commands.
    #[test]
    fn matches_upstream_duration_time_window_unit_test() {
        let cases: [(f64, f64, [u64; 5]); 4] = [
            // acquisition completes before the capture window opens
            (1.0, 3.0, [0, 0, 1, 0, 0]),
            // acquisition completes inside the capture window
            (2.0, 3.0, [0, 0, 1, 0, 0]),
            // allowed time expires before the capture window opens
            (1.0, 1.5, [0, 0, 0, 0, 0]),
            // acquisition completes after the capture window closes
            (3.0, 4.0, [0, 0, 0, 0, 0]),
        ];
        for (index, (acquisition_s, allowed_s, expected)) in cases.into_iter().enumerate() {
            let (commands, _) = run_upstream_test_function(&Params {
                use_duration: Some(1),
                imaging_times_s: [acquisition_s, allowed_s],
                time_tolerance_lower_s: Some(0.5),
                time_tolerance_upper_s: Some(0.5),
                imaging_time_s: Some(2.0),
                ..Default::default()
            });
            assert_eq!(commands, expected, "duration window case {}", index + 1);
        }
    }

    /// `test_simple_instrument_controller_elapsed_time_access`, all five
    /// parametrizations: `acquisitionTime`, `allowedTime` \[s\], the expected
    /// commands and the final `elapsedTime` \[s\], compared as upstream's
    /// `isDoubleEqual(..., 1e-12)`.
    #[test]
    fn matches_upstream_elapsed_time_access_unit_test() {
        let cases: [(f64, f64, [u64; 5], f64); 5] = [
            // zero acquisition duration images immediately
            (0.0, 2.0, [1, 0, 0, 1, 0], 0.0),
            // successful duration imaging stores the acquisition duration
            (1.0, 3.0, [0, 1, 0, 0, 1], 1.0),
            // the second attempt is still accumulating at the final update
            (2.0, 3.0, [0, 0, 1, 0, 0], 1.0),
            // failed duration imaging stores the elapsed attempt time
            (3.0, 2.0, [0, 0, 0, 0, 0], 2.0),
            // failed imaging clamps task-step overshoot to the allowed time
            (3.0, 1.5, [0, 0, 0, 0, 0], 1.5),
        ];
        for (index, (acquisition_s, allowed_s, expected, elapsed_s)) in
            cases.into_iter().enumerate()
        {
            let (commands, elapsed_nanos) = run_upstream_test_function(&Params {
                use_duration: Some(1),
                imaging_times_s: [acquisition_s, allowed_s],
                ..Default::default()
            });
            assert_eq!(commands, expected, "elapsed time case {}", index + 1);
            let expected_nanos = sec2nano(elapsed_s) as f64;
            assert!(
                (elapsed_nanos - expected_nanos).abs() <= 1e-12,
                "elapsed time case {}: {elapsed_nanos} ns, truth {expected_nanos} ns",
                index + 1
            );
        }
    }

    fn compliant_module() -> (
        SimpleInstrumentController,
        Output<AccessMsg>,
        Output<AttitudeGuidanceMsg>,
    ) {
        let access_output = Output::new(AccessMsg {
            has_access: true,
            ..Default::default()
        });
        let guidance_output = Output::new(AttitudeGuidanceMsg {
            sigma_br: Vector3::new(0.0, 0.0, 0.05),
            ..Default::default()
        });
        let mut module = SimpleInstrumentController::new(SimpleInstrumentControllerConfig {
            att_err_tolerance: 0.1,
            ..Default::default()
        });
        access_output.connect_to(&mut module.location_access_in_msg);
        guidance_output.connect_to(&mut module.att_guid_in_msg);
        (module, access_output, guidance_output)
    }

    // Not upstream: a broken constraint restarts the duration-based attempt,
    // so the acquisition time counts from the last time the constraints
    // were regained.
    #[test]
    fn broken_constraints_restart_the_acquisition() {
        let (mut module, access_output, _guidance_output) = compliant_module();
        module.config.use_duration_imaging = true;
        module.config.acquisition_time_nanos = 2e9;
        module.config.allowed_time_nanos = 5e9;
        module.init();

        let mut commands = Vec::new();
        for tick in 0..=5 {
            access_output.write(AccessMsg {
                has_access: tick != 1,
                ..Default::default()
            });
            module.update(&context(tick * TASK_RATE_NANOS));
            commands.push(module.device_cmd_out_msg.read().device_cmd);
        }
        // The attempt restarts at 2 s and captures at 4 s; then it is done.
        assert_eq!(commands, [0, 0, 0, 0, 1, 0]);
        assert_eq!(module.elapsed_time_nanos, 2e9);
        assert!(module.imaged);
    }

    // Not upstream: setting the controller status before init has no effect
    // (upstream `SelfInit` forces it on), setting it after init deactivates
    // the controller, and a reset clears `imaged` but not the status.
    #[test]
    fn controller_status_is_forced_on_at_init_and_kept_by_reset() {
        let (mut module, _access_output, _guidance_output) = compliant_module();
        module.controller_status = false;
        module.init();
        assert!(module.controller_status);
        module.update(&context(0));
        assert_eq!(module.device_cmd_out_msg.read().device_cmd, 1);
        assert!(module.imaged);

        module.controller_status = false;
        Module::reset(&mut module, &context(TASK_RATE_NANOS));
        assert!(!module.imaged);
        assert!(!module.controller_status);
        module.update(&context(TASK_RATE_NANOS));
        assert_eq!(module.device_cmd_out_msg.read().device_cmd, 0);
        assert!(!module.imaged);
    }

    // Not upstream: the negative-duration clamp writes zero into the
    // configuration, as upstream.
    #[test]
    fn negative_durations_are_clamped_in_the_configuration() {
        let (mut module, _access_output, _guidance_output) = compliant_module();
        module.config.use_duration_imaging = true;
        module.config.acquisition_time_nanos = -5.0;
        module.config.allowed_time_nanos = -1.0;
        module.init();
        assert_eq!(module.config.acquisition_time_nanos, -5.0);
        module.update(&context(0));
        assert_eq!(module.config.acquisition_time_nanos, 0.0);
        assert_eq!(module.config.allowed_time_nanos, 0.0);
        assert_eq!(module.device_cmd_out_msg.read().device_cmd, 1);
    }
}
