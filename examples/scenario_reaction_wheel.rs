//! A CubeSat with one reaction wheel in an `MJScene`: port of upstream
//! `examples/mujoco/scenarioReactionWheel.py`, the recommended first MuJoCo
//! scenario. Needs the `mujoco` feature.
//!
//! `sat_w_wheel.xml` (in `examples/data/mujoco/`) is a free-floating 2 m
//! cube `hub` with a cylindrical `wheel_1` on a hinge, driven by the joint
//! motor `wheel_1`. A stand-alone actuator message holds 2 N m on the wheel
//! for 60 s, then -2 N m for 60 s, integrated by RKF45 at a 1 s task step:
//! the hub spins up against the wheel, then back down, and by conservation
//! of angular momentum ends at rest.
//!
//! Writes `examples/output/scenarios/scenarioReactionWheel.csv`: the hub
//! origin site's state (upstream `bodyStateRecorder`, every step) and the
//! wheel joint rate.
//!
//! Upstream `run(showPlots)` has no scenario parameters, and
//! `test_scenarioMujoco.py` only checks that it runs; `tests` checks the
//! momentum exchange the scenario describes. Upstream's Vizard call (its
//! `saveFile` commented out) is not ported.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/mj_scenarios.rs"]
mod mj_scenarios;
#[path = "support/recording.rs"]
mod recording;

use basilisk_rs::messages::{Output, ScalarJointStateMsg, SingleActuatorMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::seconds;
use message_log::MessageLog;
use mj_scenarios::{
    RKF45_DEFAULT_ATOL, RKF45_DEFAULT_RTOL, epoch, load_scene, rkf45, scalar_column,
    site_state_columns, written_csv,
};
use recording::scenario_csv;

/// Upstream `dt`.
const STEP_NANOS: u64 = seconds(1);
/// Upstream `tf`: the length of each torque phase.
const PHASE_NANOS: u64 = seconds(60);
/// Upstream `torque` \[N m\].
const TORQUE_NM: f64 = 2.0;
const FILE_STEM: &str = "scenarioReactionWheel";

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// The hub origin site's state at every step.
    hub: Vec<(u64, SpacecraftStateMsg)>,
    /// The wheel joint rate \[rad/s\] at every step.
    wheel_rate: Vec<(u64, ScalarJointStateMsg)>,
}

fn run(write_csv: bool) -> Results {
    let mut scene = load_scene("sat_w_wheel.xml");
    scene.set_integrator(rkf45(RKF45_DEFAULT_RTOL, RKF45_DEFAULT_ATOL));

    // The stand-alone `SingleActuatorMsg` with the wheel motor torque.
    let torque = Output::new(SingleActuatorMsg { input: TORQUE_NM });
    torque.connect_to(&mut scene.single_actuator_mut("wheel_1").actuator_in_msg);

    let hub_state = scene.body("hub").origin().state_out_msg.clone();
    let wheel_rate = scene.scalar_joint("wheel_1").state_dot_out_msg.clone();
    let mut hub_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut wheel_log = MessageLog::<ScalarJointStateMsg>::default();
    let mut csv = write_csv.then(|| scenario_csv(FILE_STEM));
    {
        let mut simulation = Simulation::new(epoch(), false);
        connect!(&simulation,
            &hub_state => &mut hub_log.input,
            &wheel_rate => &mut wheel_log.input,
        );
        schedule! { simulation,
            "scene" => &mut scene, period=STEP_NANOS, priority=10;
            "hubLog" => &mut hub_log, period=STEP_NANOS, priority=0;
            "wheelLog" => &mut wheel_log, period=STEP_NANOS, priority=0;
        }
        if let Some(csv) = csv.as_mut() {
            connect!(&simulation,
                &hub_state => csv.add_source::<SpacecraftStateMsg>(site_state_columns("hub")),
                &wheel_rate => csv.add_source::<ScalarJointStateMsg>(
                    scalar_column("state", "wheel_1_rate_rad_per_s"),
                ),
            );
            simulation.add_module("csv", csv, STEP_NANOS, 0);
        }
        simulation.run_for(PHASE_NANOS);
        torque.write(SingleActuatorMsg { input: -TORQUE_NM });
        simulation.run_for(PHASE_NANOS);
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    Results {
        hub: hub_log.into_samples(),
        wheel_rate: wheel_log.into_samples(),
    }
}

fn main() {
    Cli::from_env("scenario_reaction_wheel").finish();
    let results = run(true);
    let (_, last) = results.hub.last().expect("recorded hub states");
    println!("final hub rate {:.3e} rad/s", last.omega_radps.norm());
    println!("wrote {}", written_csv(FILE_STEM).display());
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` only runs the scenario. These check the
    //! momentum exchange it demonstrates.

    use super::*;

    #[test]
    fn the_hub_spins_up_and_back_to_rest() {
        let results = run(false);
        // Steps 0 to 120 s inclusive.
        assert_eq!(results.hub.len(), 121);
        let rate_at = |nanos: u64| {
            results
                .hub
                .iter()
                .find(|(time, _)| *time == nanos)
                .map(|(_, state)| state.omega_radps)
                .unwrap()
        };
        let wheel_at = |nanos: u64| {
            results
                .wheel_rate
                .iter()
                .find(|(time, _)| *time == nanos)
                .map(|(_, rate)| rate.state)
                .unwrap()
        };
        // The wheel spins about the hub's -y axis: the hub counter-rotates
        // about y only.
        let peak = rate_at(PHASE_NANOS);
        assert!(peak.norm() > 1e-3, "{peak}");
        assert!(peak.x.abs() < 1e-9 * peak.norm() && peak.z.abs() < 1e-9 * peak.norm());
        assert!(wheel_at(PHASE_NANOS).abs() > 10.0 * peak.norm());
        // Equal and opposite torque phases: everything back at rest.
        let last = rate_at(2 * PHASE_NANOS);
        assert!(last.norm() < 1e-6 * peak.norm(), "{last} vs peak {peak}");
        assert!(wheel_at(2 * PHASE_NANOS).abs() < 1e-6 * wheel_at(PHASE_NANOS).abs());
        // The hub's rate grows linearly while the torque is constant.
        let half = rate_at(PHASE_NANOS / 2);
        assert!((half.y / peak.y - 0.5).abs() < 1e-6, "{half} vs {peak}");
    }
}
