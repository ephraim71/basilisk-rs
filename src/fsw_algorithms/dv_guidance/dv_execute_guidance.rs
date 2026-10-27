//! Delta-v burn execution monitor: port of upstream Basilisk
//! `dvExecuteGuidance`.
//!
//! The module starts the burn at the commanded start time, recording the
//! accumulated Δv the navigation reports at that moment, and declares it
//! complete once the Δv accumulated since then is larger than the commanded
//! one. It publishes both flags every update and, once the burn is complete,
//! a zero thruster on-time command every update to stop the thrusters.
//!
//! Upstream's documentation lists the ports of `dvGuidance`; the ports here
//! are those its code uses.

use nalgebra::Vector3;

use crate::messages::{
    DvBurnCommandMsg, DvExecutionDataMsg, Input, NavigationTranslationMsg, Output,
    ThrusterArrayOnTimeCommandMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::{Module, SimulationContext};

/// Configuration of [`DvExecuteGuidance`]; upstream has no parameters.
#[derive(Clone, Debug)]
pub struct DvExecuteGuidanceConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
}

impl Default for DvExecuteGuidanceConfig {
    fn default() -> Self {
        Self {
            name: "dvExecuteGuidance".to_string(),
        }
    }
}

/// Progress of the burn: upstream's `burnExecuting` and `burnComplete`
/// flags. Upstream never sets both after an update, so three states cover it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BurnState {
    /// Waiting for the burn start time (both flags clear).
    #[default]
    Pending,
    /// Burning (`burnExecuting`).
    Executing,
    /// The commanded Δv has been exceeded (`burnComplete`); final.
    Complete,
}

/// Delta-v burn start and cut-off logic: upstream `dvExecuteGuidance`.
#[derive(Clone, Debug)]
pub struct DvExecuteGuidance {
    /// Module configuration.
    pub config: DvExecuteGuidanceConfig,
    /// Upstream `navDataInMsg`: navigation solution with the accumulated Δv.
    pub nav_data_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `burnDataInMsg`: commanded Δv and burn start time.
    pub burn_data_in_msg: Input<DvBurnCommandMsg>,
    /// Upstream `burnExecOutMsg`: burn executing and complete flags.
    pub burn_exec_out_msg: Output<DvExecutionDataMsg>,
    /// Upstream `thrCmdOutMsg`: zero on-time command, written only once the
    /// burn is complete.
    pub thr_cmd_out_msg: Output<ThrusterArrayOnTimeCommandMsg>,
    /// Upstream `burnExecuting`/`burnComplete`. Neither init nor reset
    /// changes it, as upstream; set it back to [`BurnState::Pending`] to arm
    /// another burn.
    pub burn_state: BurnState,
    /// Upstream `dvInit` \[m/s\]: accumulated Δv (body frame, as the
    /// navigation message reports it) when the burn started.
    pub dv_init_mps: Vector3<f64>,
}

impl DvExecuteGuidance {
    /// Creates the module with unconnected ports and a pending burn.
    pub fn new(config: DvExecuteGuidanceConfig) -> Self {
        Self {
            config,
            nav_data_in_msg: Input::default(),
            burn_data_in_msg: Input::default(),
            burn_exec_out_msg: Output::default(),
            thr_cmd_out_msg: Output::default(),
            burn_state: BurnState::Pending,
            dv_init_mps: Vector3::zeros(),
        }
    }

    /// Upstream `Reset_dvExecuteGuidance`: it only checks the inputs and
    /// keeps the burn state.
    fn check_connections(&self) {
        if !self.nav_data_in_msg.is_connected() {
            log::error!("dvExecuteGuidance.nav_data_in_msg is not connected");
        }
        if !self.burn_data_in_msg.is_connected() {
            log::error!("dvExecuteGuidance.burn_data_in_msg is not connected");
        }
    }

    fn advance_burn_state(
        &mut self,
        nav: &NavigationTranslationMsg,
        burn: &DvBurnCommandMsg,
        call_nanos: u64,
    ) {
        let burn_time_s = diff_nanos_to_seconds(call_nanos, burn.burn_start_time_nanos);
        if self.burn_state == BurnState::Pending && burn_time_s >= 0.0 {
            self.burn_state = BurnState::Executing;
            self.dv_init_mps = nav.accumulated_dv_b_mps;
        }

        // Upstream compares a zero executed Δv when no burn is running, which
        // can never exceed the commanded norm, so only a running burn ends.
        if self.burn_state == BurnState::Executing {
            let dv_executed = nav.accumulated_dv_b_mps - self.dv_init_mps;
            if dv_executed.norm() > burn.dv_inertial_cmd_mps.norm() {
                self.burn_state = BurnState::Complete;
            }
        }
    }
}

impl Module for DvExecuteGuidance {
    fn init(&mut self) {
        self.burn_exec_out_msg.write(DvExecutionDataMsg::default());
        self.thr_cmd_out_msg
            .write(ThrusterArrayOnTimeCommandMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, context: &SimulationContext) {
        let nav = self.nav_data_in_msg.read();
        let burn = self.burn_data_in_msg.read();
        self.advance_burn_state(&nav, &burn, context.current_sim_nanos);

        if self.burn_state == BurnState::Complete {
            self.thr_cmd_out_msg
                .write(ThrusterArrayOnTimeCommandMsg::default());
        }
        self.burn_exec_out_msg.write(DvExecutionDataMsg {
            burn_executing: self.burn_state == BurnState::Executing,
            burn_complete: self.burn_state == BurnState::Complete,
        });
    }
}

#[cfg(test)]
mod tests {
    // Upstream `dvExecuteGuidance` has no unit test; these follow its code,
    // with hand-derived truth.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{BurnState, DvExecuteGuidance, DvExecuteGuidanceConfig};
    use crate::messages::{
        DvBurnCommandMsg, DvExecutionDataMsg, MAX_EFF_COUNT, NavigationTranslationMsg, Output,
        ThrusterArrayOnTimeCommandMsg,
    };
    use crate::{Module, SimulationContext};

    const SECOND_NANOS: u64 = 1_000_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn navigation(accumulated_dv_b_mps: Vector3<f64>) -> NavigationTranslationMsg {
        NavigationTranslationMsg {
            accumulated_dv_b_mps,
            ..Default::default()
        }
    }

    struct Fixture {
        module: DvExecuteGuidance,
        nav_output: Output<NavigationTranslationMsg>,
        _burn_output: Output<DvBurnCommandMsg>,
    }

    /// A 2 m/s burn (norm of `(0, 1.2, 1.6)`) starting at 2 s, with 10 m/s
    /// already accumulated along x, and a nonzero sentinel in the thruster
    /// command so that a write is visible.
    fn fixture() -> Fixture {
        let nav_output = Output::new(navigation(Vector3::new(10.0, 0.0, 0.0)));
        let burn_output = Output::new(DvBurnCommandMsg {
            dv_inertial_cmd_mps: Vector3::new(0.0, 1.2, 1.6),
            burn_start_time_nanos: 2 * SECOND_NANOS,
            ..Default::default()
        });
        let mut module = DvExecuteGuidance::new(DvExecuteGuidanceConfig::default());
        nav_output.connect_to(&mut module.nav_data_in_msg);
        burn_output.connect_to(&mut module.burn_data_in_msg);
        module.init();
        module
            .thr_cmd_out_msg
            .write(ThrusterArrayOnTimeCommandMsg::from_active(
                &[0.5; MAX_EFF_COUNT],
            ));
        Fixture {
            module,
            nav_output,
            _burn_output: burn_output,
        }
    }

    fn flags(module: &DvExecuteGuidance) -> (bool, bool) {
        let DvExecutionDataMsg {
            burn_executing,
            burn_complete,
        } = module.burn_exec_out_msg.read();
        (burn_executing, burn_complete)
    }

    fn thruster_command_is_zero(module: &DvExecuteGuidance) -> bool {
        module
            .thr_cmd_out_msg
            .read()
            .on_time_request_s
            .iter()
            .all(|&on_time| on_time == 0.0)
    }

    #[test]
    fn starts_at_the_burn_time_and_stops_past_the_commanded_dv() {
        let Fixture {
            mut module,
            nav_output,
            ..
        } = fixture();

        module.update(&context(SECOND_NANOS));
        assert_eq!(flags(&module), (false, false), "before the start time");
        assert!(!thruster_command_is_zero(&module), "no thruster write yet");

        module.update(&context(2 * SECOND_NANOS));
        assert_eq!(flags(&module), (true, false), "at the start time");
        assert_eq!(module.dv_init_mps, Vector3::new(10.0, 0.0, 0.0));

        // An executed dv equal to the commanded vector, component for
        // component, does not complete the burn: the test is strictly greater.
        nav_output.write(navigation(Vector3::new(10.0, 1.2, 1.6)));
        module.update(&context(3 * SECOND_NANOS));
        assert_eq!(flags(&module), (true, false), "commanded dv reached");
        assert!(!thruster_command_is_zero(&module));

        nav_output.write(navigation(Vector3::new(10.0, 1.2, 1.7)));
        module.update(&context(4 * SECOND_NANOS));
        assert_eq!(flags(&module), (false, true), "commanded dv exceeded");
        assert_eq!(module.burn_state, BurnState::Complete);
        assert!(thruster_command_is_zero(&module), "zero thruster command");

        // Complete is final, and a reset does not re-arm the burn.
        nav_output.write(navigation(Vector3::new(10.0, 0.0, 0.0)));
        Module::reset(&mut module, &context(5 * SECOND_NANOS));
        module.update(&context(5 * SECOND_NANOS));
        assert_eq!(flags(&module), (false, true));
    }

    #[test]
    fn a_burn_running_past_its_start_time_keeps_its_initial_dv() {
        let Fixture {
            mut module,
            nav_output,
            ..
        } = fixture();

        // Starting late still starts; the accumulated dv then is the base.
        nav_output.write(navigation(Vector3::new(10.0, 0.0, 5.0)));
        module.update(&context(7 * SECOND_NANOS));
        assert_eq!(flags(&module), (true, false));
        assert_eq!(module.dv_init_mps, Vector3::new(10.0, 0.0, 5.0));

        nav_output.write(navigation(Vector3::new(11.0, 0.0, 5.0)));
        module.update(&context(8 * SECOND_NANOS));
        assert_eq!(module.dv_init_mps, Vector3::new(10.0, 0.0, 5.0));
        assert_eq!(flags(&module), (true, false), "1 m/s of 2 m/s executed");

        nav_output.write(navigation(Vector3::new(12.5, 0.0, 5.0)));
        module.update(&context(9 * SECOND_NANOS));
        assert_eq!(flags(&module), (false, true), "2.5 m/s of 2 m/s executed");
    }

    #[test]
    fn rearming_the_burn_state_starts_a_new_burn() {
        let Fixture {
            mut module,
            nav_output,
            ..
        } = fixture();
        module.update(&context(2 * SECOND_NANOS));
        nav_output.write(navigation(Vector3::new(13.0, 0.0, 0.0)));
        module.update(&context(3 * SECOND_NANOS));
        assert_eq!(module.burn_state, BurnState::Complete);

        module.burn_state = BurnState::Pending;
        module.update(&context(4 * SECOND_NANOS));
        assert_eq!(flags(&module), (true, false));
        assert_eq!(module.dv_init_mps, Vector3::new(13.0, 0.0, 0.0));
    }

    // With unconnected inputs the commanded dv is zero and the burn starts at
    // t = 0, but a zero executed dv never exceeds it.
    #[test]
    fn unconnected_inputs_start_but_never_complete_a_burn() {
        let mut module = DvExecuteGuidance::new(DvExecuteGuidanceConfig::default());
        module.init();
        for tick in 0..3 {
            module.update(&context(tick * SECOND_NANOS));
            assert_eq!(flags(&module), (true, false));
        }
    }
}
