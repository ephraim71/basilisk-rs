//! Inertial pointing with an MRP PD controller written in the scenario as a
//! compiled module: port of upstream
//! `examples/scenarioAttitudePointingNumba.py`.
//!
//! [`scenarioAttitudePointing`](scenario_attitude_pointing.rs)'s small-tumble
//! case (deep space, one 0.1 s task, 10 minutes), with `mrpFeedback`
//! replaced by `NumbaMRPPD`, a `NumbaModel` subclass whose
//! `UpdateStateImpl` Numba compiles to machine code. Rust modules are
//! compiled anyway, so its counterpart, [`ScenarioMrpPd`], is a plain type
//! implementing [`Module`]:
//!
//! | upstream (`NumbaModel` subclass)                         | here (Rust)                        |
//! |----------------------------------------------------------|------------------------------------|
//! | `self.memory.K`, `self.memory.P`                          | struct fields                      |
//! | `guidInMsg` reader, `cmdTorqueOutMsg` message            | [`Input`] / [`Output`] fields      |
//! | `UpdateStateImpl(guidInMsgPayload, cmdTorqueOutMsgPayload, memory)` | [`Module::update`]: `read()`, compute, `write()` |
//! | JIT compilation on first use                             | ahead-of-time, with the scenario   |
//!
//! Upstream `run(show_plots)` has no parameters; like upstream, the run
//! prints its wall-clock time. Writes
//! `examples/output/scenarios/scenarioAttitudePointingNumba.csv`.

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

/// Upstream `NumbaMRPPD`: per axis, `L_r,i = -(K sigma_BR,i + P omega_BR,i)`.
struct ScenarioMrpPd {
    guid_in_msg: Input<AttitudeGuidanceMsg>,
    cmd_torque_out_msg: Output<BodyTorqueCommandMsg>,
    /// Upstream `memory.K`.
    k: f64,
    /// Upstream `memory.P`.
    p: f64,
}

impl Module for ScenarioMrpPd {
    fn init(&mut self) {
        self.cmd_torque_out_msg
            .write(BodyTorqueCommandMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let guidance = self.guid_in_msg.read();
        let mut command = BodyTorqueCommandMsg::default();
        for axis in 0..3 {
            let lr_cmd =
                guidance.sigma_br[axis] * self.k + guidance.omega_br_b_radps[axis] * self.p;
            command.torque_request_body_nm[axis] = -lr_cmd;
        }
        self.cmd_torque_out_msg.write(command);
    }
}

fn run(write_csv: bool) -> PointingResults {
    let mut controller = ScenarioMrpPd {
        guid_in_msg: Input::default(),
        cmd_torque_out_msg: Output::default(),
        k: 3.5,
        p: 30.0,
    };
    let ports = ControllerPorts {
        guid_in_msg: controller.guid_in_msg.clone(),
        cmd_torque_out_msg: controller.cmd_torque_out_msg.clone(),
        veh_config_in_msg: None,
    };
    run_pointing(
        &mut controller,
        ports,
        Vector3::from(INITIAL_OMEGA_BN_RADPS),
        "scenarioAttitudePointingNumba",
        write_csv,
    )
}

fn main() {
    let mut cli = Cli::from_env("scenario_attitude_pointing_numba [--all]");
    // Upstream's test runs the one configuration there is.
    cli.switch("all");
    cli.finish();

    let started = std::time::Instant::now();
    let results = run(true);
    println!("Run scenario in {} s", started.elapsed().as_secs_f64());
    println!(
        "wrote {} (final |sigma_BR| = {:.3e})",
        results.csv_path.expect("CSV written").display(),
        results.guidance.last().sigma_br.norm(),
    );
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttitudePointingNumba.py` only checks that the
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
