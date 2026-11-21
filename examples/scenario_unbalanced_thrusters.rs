//! A CubeSat with four fuel-burning thrusters, one of them defective: port
//! of upstream `examples/mujoco/scenarioUnbalancedThrusters.py`. Needs the
//! `mujoco` feature.
//!
//! `sat_w_thrusters.xml` (in `examples/data/mujoco/`) is a free-floating 2 m
//! cube `hub` holding four capsule fuel tanks, `tank_1` to `tank_4`, each
//! with a thruster site (`tank_<i>_thrust_point`) and a single actuator
//! that pushes the hub along +z. Every thruster gets a constant 5 N; each
//! tank's mass derivative (`derivativeMassPropertiesInMsg`) is -1 kg/s,
//! except `tank_4`'s, -2 kg/s. Upstream integrates the body masses with the
//! joint states, so the tanks empty linearly and, once `tank_4` is lighter,
//! the center of mass drifts off the thrust axis and the hub starts to
//! rotate. RKF45 at upstream's default tolerances, 1 s task step, 150 s.
//!
//! Writes `examples/output/scenarios/scenarioUnbalancedThrusters.csv`: the
//! four tank masses (upstream's `massPropertiesOutMsg` recorders) and the
//! hub origin state (`bodyStateRecorder`) every step.
//!
//! Upstream `run(showPlots)` has no scenario parameters, and
//! `test_scenarioMujoco.py` only checks that it runs; `tests` checks the
//! mass histories and the onset of rotation. Upstream's Vizard call and its
//! thruster-plume message writers (`ThrusterVizMessageWriter`, Vizard
//! only, `saveFile` commented out) are not ported.

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

use basilisk_rs::messages::{
    Output, SingleActuatorMsg, SpacecraftMassPropsMsg, SpacecraftStateMsg,
};
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
/// Upstream `tf` (2.5 min).
const DURATION_NANOS: u64 = seconds(150);
/// Upstream `thrust` \[N\].
const THRUST_N: f64 = 5.0;
/// Upstream `mDot` \[kg/s\], doubled for `tank_4`.
const MASS_RATE_KGPS: f64 = -1.0;
const FILE_STEM: &str = "scenarioUnbalancedThrusters";

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Each tank's mass history.
    tank_masses: Vec<Vec<(u64, SpacecraftMassPropsMsg)>>,
    hub: Vec<(u64, SpacecraftStateMsg)>,
}

fn run(write_csv: bool) -> Results {
    let mut scene = load_scene("sat_w_thrusters.xml");
    scene.set_integrator(rkf45(RKF45_DEFAULT_RTOL, RKF45_DEFAULT_ATOL));

    // Stand-alone thrust and mass-rate messages, one per tank.
    let mut messages = Vec::new();
    let mut mass_outputs = Vec::new();
    for tank in 1..=4 {
        let thrust = Output::new(SingleActuatorMsg { input: THRUST_N });
        thrust.connect_to(
            &mut scene
                .single_actuator_mut(&format!("tank_{tank}_thrust"))
                .actuator_in_msg,
        );
        let rate = if tank < 4 {
            MASS_RATE_KGPS
        } else {
            2.0 * MASS_RATE_KGPS
        };
        let mass_rate = Output::new(SpacecraftMassPropsMsg {
            mass_kg: rate,
            ..Default::default()
        });
        let mut body = scene.body_mut(&format!("tank_{tank}"));
        mass_rate.connect_to(&mut body.derivative_mass_properties_in_msg);
        mass_outputs.push(body.mass_properties_out_msg.clone());
        messages.push((thrust, mass_rate));
    }
    let hub_state = scene.body("hub").origin().state_out_msg.clone();

    let mut mass_logs: Vec<MessageLog<SpacecraftMassPropsMsg>> =
        (0..4).map(|_| MessageLog::default()).collect();
    let mut hub_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut csv = write_csv.then(|| scenario_csv(FILE_STEM));
    {
        let mut simulation = Simulation::new(epoch(), false);
        for (output, log) in mass_outputs.iter().zip(mass_logs.iter_mut()) {
            connect!(&simulation, output => &mut log.input);
        }
        connect!(&simulation, &hub_state => &mut hub_log.input);
        simulation.add_module("scene", &mut scene, STEP_NANOS, 10);
        for (index, log) in mass_logs.iter_mut().enumerate() {
            simulation.add_module(format!("massPropRecorder{}", index + 1), log, STEP_NANOS, 0);
        }
        schedule! { simulation,
            "bodyStateRecorder" => &mut hub_log, period=STEP_NANOS, priority=0;
        }
        if let Some(csv) = csv.as_mut() {
            for (index, output) in mass_outputs.iter().enumerate() {
                connect!(&simulation,
                    output => csv.add_source::<SpacecraftMassPropsMsg>(scalar_column(
                        "mass_kg",
                        &format!("tank_{}_mass_kg", index + 1),
                    )),
                );
            }
            connect!(&simulation,
                &hub_state => csv.add_source::<SpacecraftStateMsg>(site_state_columns("hub")),
            );
            simulation.add_module("csv", csv, STEP_NANOS, 0);
        }
        simulation.run_for(DURATION_NANOS);
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    drop(messages);
    Results {
        tank_masses: mass_logs
            .into_iter()
            .map(MessageLog::into_samples)
            .collect(),
        hub: hub_log.into_samples(),
    }
}

fn main() {
    Cli::from_env("scenario_unbalanced_thrusters").finish();
    let results = run(true);
    let masses: Vec<f64> = results
        .tank_masses
        .iter()
        .map(|log| log.last().expect("recorded masses").1.mass_kg)
        .collect();
    println!("final tank masses {masses:.2?} kg");
    println!(
        "final hub sigma_BN {:.4?}",
        results.hub.last().expect("recorded hub").1.sigma_bn
    );
    println!("wrote {}", written_csv(FILE_STEM).display());
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` only runs the scenario. These check what it
    //! demonstrates: linear fuel use, and rotation from the imbalance.

    use super::*;

    #[test]
    fn tanks_empty_linearly_and_the_imbalance_turns_the_hub() {
        let results = run(false);
        for (index, log) in results.tank_masses.iter().enumerate() {
            assert_eq!(log.len(), 151);
            let rate = if index < 3 { -1.0 } else { -2.0 };
            let initial = log[0].1.mass_kg;
            // Capsule of radius 0.3 m, half-length 0.3 m, 2000 kg/m^3.
            let volume =
                std::f64::consts::PI * 0.09 * 0.6 + 4.0 / 3.0 * std::f64::consts::PI * 0.027;
            assert!((initial - 2000.0 * volume).abs() < 1e-6, "{initial}");
            for (time, mass) in log {
                let expected = initial + rate * *time as f64 * 1e-9;
                assert!(
                    (mass.mass_kg - expected).abs() < 1e-6,
                    "tank {} at {time}: {} vs {expected}",
                    index + 1,
                    mass.mass_kg
                );
            }
        }
        // Balanced at first: thrust along +z through the center of mass.
        let early = results.hub[10].1.omega_radps.norm();
        let (_, last) = results.hub.last().unwrap();
        assert!(last.velocity_mps.z > 0.0);
        assert!(last.omega_radps.norm() > 100.0 * early.max(1e-12));
        assert!(last.sigma_bn.norm() > 1e-2, "{}", last.sigma_bn);
    }
}
