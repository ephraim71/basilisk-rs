//! Inertial pointing with an MRP PD controller written in the scenario
//! itself: port of upstream `examples/scenarioAttitudePointingPy.py`.
//!
//! [`scenarioAttitudePointing`](scenario_attitude_pointing.rs)'s small-tumble
//! case (deep space, one 0.1 s task, 10 minutes), with `mrpFeedback`
//! replaced by `PythonMRPPD`, a Basilisk module written in Python by
//! subclassing `sysModel.SysModel`. Its Rust counterpart is
//! [`ScenarioMrpPd`], a type defined here that implements [`Module`]:
//!
//! | upstream (`sysModel.SysModel` subclass)        | here (Rust)                          |
//! |------------------------------------------------|--------------------------------------|
//! | `self.K`, `self.P`                              | struct fields                        |
//! | `self.guidInMsg = messaging.AttGuidMsgReader()` | [`Input`] field                      |
//! | `self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()` | [`Output`] field               |
//! | `Reset(CurrentSimNanos)`                        | [`Module::init`]                     |
//! | `UpdateState(CurrentSimNanos)`                  | [`Module::update`]                   |
//! | `self.guidInMsg()` / `cmdTorqueOutMsg.write(payload, CurrentSimNanos, moduleID)` | `read()` / `write(payload)` |
//! | `scSim.AddModelToTask(simTaskName, pyMRPPD)`    | scheduled like any library module    |
//! | `self.bskLogger.info(...)`                      | the `log` crate                      |
//!
//! As upstream notes, such a module behaves like any other: it runs in its
//! task in priority order, here after the guidance it reads.
//!
//! Upstream `run(show_plots)` has no parameters. Writes
//! `examples/output/scenarios/scenarioAttitudePointingPy.csv`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/pointing.rs"]
mod pointing;

use attitude::INITIAL_OMEGA_BN_RADPS;
use basilisk_rs::messages::{AttitudeGuidanceMsg, BodyTorqueCommandMsg, Input, Output};
use basilisk_rs::{Module, SimulationContext};
use cli::Cli;
use nalgebra::Vector3;
use pointing::{ControllerPorts, PointingResults, run_pointing};

/// Upstream `PythonMRPPD`: the proportional-derivative MRP law
/// `L_r = -(K sigma_BR + P omega_BR)`, with no feed-forward terms.
struct ScenarioMrpPd {
    k: f64,
    p: f64,
    guid_in_msg: Input<AttitudeGuidanceMsg>,
    cmd_torque_out_msg: Output<BodyTorqueCommandMsg>,
}

impl ScenarioMrpPd {
    fn new() -> Self {
        Self {
            k: 0.0,
            p: 0.0,
            guid_in_msg: Input::default(),
            cmd_torque_out_msg: Output::default(),
        }
    }
}

impl Module for ScenarioMrpPd {
    /// Upstream `Reset`, which does nothing.
    fn init(&mut self) {}

    fn update(&mut self, context: &SimulationContext) {
        let guidance = self.guid_in_msg.read();
        let torque = -(guidance.sigma_br * self.k + guidance.omega_br_b_radps * self.p);
        self.cmd_torque_out_msg.write(BodyTorqueCommandMsg {
            torque_request_body_nm: torque,
            ..Default::default()
        });
        // Upstream's (disabled) sample of module logging.
        log::debug!(
            "Time: {} s, TorqueRequestBody: {torque:?}, sigma_BR: {:?}, omega_BR_B: {:?}",
            context.current_sim_nanos as f64 * 1.0e-9,
            guidance.sigma_br,
            guidance.omega_br_b_radps,
        );
    }
}

pub(crate) fn run(write_csv: bool) -> PointingResults {
    let mut controller = ScenarioMrpPd::new();
    controller.k = 3.5;
    controller.p = 30.0;
    let ports = ControllerPorts {
        guid_in_msg: controller.guid_in_msg.clone(),
        cmd_torque_out_msg: controller.cmd_torque_out_msg.clone(),
        veh_config_in_msg: None,
    };
    run_pointing(
        &mut controller,
        ports,
        Vector3::from(INITIAL_OMEGA_BN_RADPS),
        "scenarioAttitudePointingPy",
        write_csv,
    )
}

fn main() {
    let mut cli = Cli::from_env("scenario_attitude_pointing_py [--all]");
    // Upstream's test runs the one configuration there is.
    cli.switch("all");
    cli.finish();

    let results = run(true);
    println!(
        "wrote {} (final |sigma_BR| = {:.3e})",
        results.csv_path.expect("CSV written").display(),
        results.guidance.last().sigma_br.norm(),
    );
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttitudePointingPy.py` only checks that the
    //! scenario runs. These tests check the module's control law in the loop
    //! and the resulting stabilization.

    use super::*;

    #[test]
    fn the_scenario_module_applies_the_pd_law_and_stabilizes_the_attitude() {
        let results = run(false);
        assert_eq!(results.guidance.values.len(), 50);
        for (guidance, torque) in results.guidance.values.iter().zip(&results.torque.values) {
            let expected = -(3.5 * guidance.sigma_br + 30.0 * guidance.omega_br_b_radps);
            assert_eq!(torque.torque_request_body_nm, expected);
        }
        let last = results.guidance.last();
        assert!(last.sigma_br.norm() < 1.0e-3, "{}", last.sigma_br);
        assert!(last.omega_br_b_radps.norm() < 1.0e-4);
    }
}
