//! Two CubeSats docking through a MuJoCo weld equality: port of upstream
//! `examples/mujoco/scenarioSimpleDocking.py`. Needs the `mujoco` feature.
//!
//! `sats_dock.xml` (in `examples/data/mujoco/`) holds two free-floating
//! 1 kg cubes, `hub_1` and `hub_2`, each with one thruster along a body
//! axis, contacts disabled, and an inactive `dock` weld between their
//! `contact_point` sites. Set at `(-1, 0, 0)` and `(0, 0, -2)` m after
//! initialization, they
//!
//! 1. thrust towards each other for 1 s (1 N each),
//! 2. brake for 1 s, which brings the contact points together at rest,
//! 3. coast for 0.5 s,
//! 4. dock: the weld is activated and only `hub_1` thrusts, for 1.5 s,
//!    dragging `hub_2` along,
//! 5. undock: the weld is released and `hub_1` thrusts backwards until 6 s,
//!    so the hubs separate.
//!
//! RKF45 at upstream's default tolerances, 0.1 s task step. Writes
//! `examples/output/scenarios/scenarioSimpleDocking.csv`: both contact-point
//! site states (upstream's two recorders) every step; the plot script
//! draws upstream's figure, their distance.
//!
//! Upstream `run(showPlots)` has no scenario parameters, and
//! `test_scenarioMujoco.py` only checks that it runs; `tests` checks the
//! docking sequence. Upstream sets the states and toggles the equality on
//! the scene between `ExecuteSimulation` calls; here the scene is
//! [`Shared`] and locked between `run_for` calls. Upstream's Vizard call
//! (its `saveFile` commented out) is not ported.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/mj_scenarios.rs"]
mod mj_scenarios;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/shared.rs"]
mod shared;

use basilisk_rs::messages::{Output, SingleActuatorMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use mj_scenarios::{
    RKF45_DEFAULT_ATOL, RKF45_DEFAULT_RTOL, epoch, load_scene, rkf45, site_state_columns,
    written_csv,
};
use nalgebra::Vector3;
use recording::{History, scenario_csv};
use shared::Shared;

/// Upstream task step \[ns\].
const STEP_NANOS: u64 = 100_000_000;
/// Upstream `force` \[N\].
const FORCE_N: f64 = 1.0;
const FILE_STEM: &str = "scenarioSimpleDocking";

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    hub_1_contact: Vec<(u64, SpacecraftStateMsg)>,
    hub_2_contact: Vec<(u64, SpacecraftStateMsg)>,
}

/// Upstream `setThrusterForce`.
fn set_thruster_force(messages: &[&Output<SingleActuatorMsg>; 2], thrust: [f64; 2]) {
    for (message, value) in messages.iter().zip(thrust) {
        message.write(SingleActuatorMsg { input: value });
    }
}

fn run(write_csv: bool) -> Results {
    let mut scene = load_scene("sats_dock.xml");
    scene.set_integrator(rkf45(RKF45_DEFAULT_RTOL, RKF45_DEFAULT_ATOL));

    let hub_1_thruster = Output::new(SingleActuatorMsg::default());
    let hub_2_thruster = Output::new(SingleActuatorMsg::default());
    hub_1_thruster.connect_to(&mut scene.single_actuator_mut("hub_1_thruster").actuator_in_msg);
    hub_2_thruster.connect_to(&mut scene.single_actuator_mut("hub_2_thruster").actuator_in_msg);
    let thrusters = [&hub_1_thruster, &hub_2_thruster];

    let contact_1 = scene.site("hub_1_contact_point").state_out_msg.clone();
    let contact_2 = scene.site("hub_2_contact_point").state_out_msg.clone();
    let mut contact_1_log = History::<SpacecraftStateMsg>::new();
    let mut contact_2_log = History::<SpacecraftStateMsg>::new();
    let mut csv = write_csv.then(|| scenario_csv(FILE_STEM));

    let scene = Shared::new(scene);
    let mut scene_task = scene.handle();
    {
        let mut simulation = Simulation::new(epoch(), false);
        connect!(&simulation,
            &contact_1 => &mut contact_1_log.input,
            &contact_2 => &mut contact_2_log.input,
        );
        schedule! { simulation,
            "scene" => &mut scene_task, period=STEP_NANOS, priority=10;
            "hub1ContactRecorder" => &mut contact_1_log, period=STEP_NANOS, priority=0;
            "hub2ContactRecorder" => &mut contact_2_log, period=STEP_NANOS, priority=0;
        }
        if let Some(csv) = csv.as_mut() {
            connect!(&simulation,
                &contact_1 => csv.add_source::<SpacecraftStateMsg>(site_state_columns("hub_1_contact")),
                &contact_2 => csv.add_source::<SpacecraftStateMsg>(site_state_columns("hub_2_contact")),
            );
            simulation.add_module("csv", csv, STEP_NANOS, 0);
        }
        simulation.initialize();

        // The initial positions of both CubeSats.
        {
            let mut scene = scene.lock();
            scene
                .body_mut("hub_1")
                .set_position(Vector3::new(-1.0, 0.0, 0.0));
            scene
                .body_mut("hub_2")
                .set_position(Vector3::new(0.0, 0.0, -2.0));
        }

        // Thrust so that the spacecraft get close to each other.
        set_thruster_force(&thrusters, [FORCE_N, FORCE_N]);
        simulation.run_for(1_000_000_000);
        // Opposite thrust to slow down.
        set_thruster_force(&thrusters, [-FORCE_N, -FORCE_N]);
        simulation.run_for(1_000_000_000);
        // Together and stationary: coast for 0.5 s.
        set_thruster_force(&thrusters, [0.0, 0.0]);
        simulation.run_for(500_000_000);
        // Dock: the weld aligns the two contact-point frames, and hub_1
        // drags hub_2 along.
        scene.lock().set_equality_active("dock", true);
        set_thruster_force(&thrusters, [FORCE_N, 0.0]);
        simulation.run_for(1_500_000_000);
        // Undock: the hubs separate.
        scene.lock().set_equality_active("dock", false);
        set_thruster_force(&thrusters, [-FORCE_N, 0.0]);
        simulation.run_for(2_000_000_000);
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    Results {
        hub_1_contact: contact_1_log.into_samples(),
        hub_2_contact: contact_2_log.into_samples(),
    }
}

impl Results {
    /// Distance between the contact points at each step \[m\].
    fn distances(&self) -> Vec<(u64, f64)> {
        self.hub_1_contact
            .iter()
            .zip(&self.hub_2_contact)
            .map(|((time, one), (_, two))| (*time, (one.position_m - two.position_m).norm()))
            .collect()
    }
}

fn main() {
    Cli::from_env("scenario_simple_docking").finish();
    let results = run(true);
    let distances = results.distances();
    println!(
        "contact distance: {:.3} m at 0 s, {:.2e} m docked at 4 s, {:.3} m at 6 s",
        distances[0].1, distances[40].1, distances[60].1
    );
    println!("wrote {}", written_csv(FILE_STEM).display());
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` only runs the scenario. These check the
    //! docking sequence it describes.

    use super::*;

    #[test]
    fn hubs_meet_stay_docked_and_separate() {
        let results = run(false);
        let distances = results.distances();
        assert_eq!(distances.len(), 61);
        let at = |tenths: usize| distances[tenths].1;
        // Initially at (-1, 0, -0.5) and (0, 0, -1.5) m.
        assert!((at(0) - 2.0_f64.sqrt()).abs() < 1e-12, "{}", at(0));
        // Thrust and brake bring the contact points together at rest.
        assert!(at(20) < 1e-9, "{}", at(20));
        assert!(at(25) < 1e-9, "{}", at(25));
        let hub_2_velocity = |index: usize| results.hub_2_contact[index].1.velocity_mps;
        assert!(hub_2_velocity(25).norm() < 1e-9);
        // Docked: the weld holds the contact points together while hub_1's
        // 1 N, off the pair's center of mass, drags and turns the 2 kg pair.
        // The welded point is the pair's center of mass, so its speed is
        // that of 1 N on 2 kg for 1.5 s, less the turning of the thrust.
        for index in 26..=40 {
            assert!(at(index) < 1e-4, "docked distance {} at {index}", at(index));
        }
        let pulled = hub_2_velocity(40).norm();
        assert!(pulled > 0.7 && pulled < 0.75, "{pulled}");
        // Undocked: the hubs separate.
        assert!(at(50) > 0.5 && at(60) > 2.0, "{} {}", at(50), at(60));
    }
}
