//! Two electrostatically charged spacecraft in GEO: port of upstream
//! `examples/scenarioTwoChargedSC.py`.
//!
//! A leader and a follower, both at upstream's hub defaults (1 kg, identity
//! inertia, no rotation), on a geostationary orbit about the point-mass
//! Earth, the follower 50 m behind the leader with the same inertial
//! velocity. Each is modelled by upstream's 80-sphere GOES-R bus
//! (`dataForExamples/GOESR_bus_80_sphs.csv`, copied to
//! `examples/data/GOESR_bus_80_sphs.txt` as `*.csv` files are ignored by
//! git), held at -500 V (leader) and +500 V (follower). `msmForceTorque`
//! solves the sphere charges and the electrostatic force on each body, which
//! an `extForceTorque` effector on each applies (the torques are not used,
//! as upstream). One 25 s task for a tenth of an orbit.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted and runs
//! the same single case.
//!
//! Writes `examples/output/scenarios/scenarioTwoChargedSC.csv` every step
//! with each spacecraft's `<Leader|Follower>_r_BN_N_m_<i>`,
//! `<...>_v_BN_N_m_per_s_<i>` and `<...>_sigma_BN_<i>`, and
//! `scenarioTwoChargedSCSpheres.csv` with each sphere's body-frame position
//! `r_SB_B_m_<i>`, `radius_m`, and its final charge on the leader and the
//! follower (`leaderCharge_C`, `followerCharge_C`: upstream
//! `chargeMsmOutMsgs[i].read().q` after the run).

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;

use std::fmt::Write as _;
use std::path::PathBuf;

use attitude::{add_point_mass_earth, scenario_recorder};
use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::msm_force_torque::{
    MsmForceTorque, MsmForceTorqueConfig, MsmSpaceObject,
};
use basilisk_rs::messages::{
    ChargeMsmMsg, InertialForceCommandMsg, Output, SpacecraftStateMsg, VoltMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::MU_EARTH;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{scenario_output_path, vector_columns};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{orbit_period_s, sec2nano};

/// Upstream `simulationTimeStep = macros.sec2nano(25.0)`.
const TASK_PERIOD_NANOS: u64 = 25_000_000_000;
/// Upstream's sphere model of both spacecraft.
const SPHERE_MODEL: &str = "examples/data/GOESR_bus_80_sphs.txt";
/// Upstream `voltLeaderInMsgData` and `voltFollowerInMsgData` \[V\].
const VOLTAGES_V: [f64; 2] = [-500.0, 500.0];
/// Upstream `r_FS`: the follower relative to the leader \[m\].
const FOLLOWER_OFFSET_M: [f64; 3] = [0.0, -50.0, 0.0];
/// Upstream `ModelTag`s.
const NAMES: [&str; 2] = ["Leader", "Follower"];

/// Upstream `run(show_plots)`: no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// The two spacecraft potentials \[V\]; upstream's are [`VOLTAGES_V`].
    voltages_v: [f64; 2],
    /// Write the CSV files (off in the tests).
    write_csv: bool,
}

/// The single run of upstream's `test_scenarioTest.py` entry.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config {
    voltages_v: VOLTAGES_V,
    write_csv: true,
}];

/// Upstream's reading of `GOESR_bus_80_sphs.csv`: rows of `x, y, z, radius`
/// \[m\].
fn sphere_model() -> MsmSpaceObject {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(SPHERE_MODEL);
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("cannot read {}: {error}", path.display()));
    let mut model = MsmSpaceObject::default();
    for line in text.lines().filter(|line| !line.trim().is_empty()) {
        let values: Vec<f64> = line
            .split(',')
            .map(|value| value.trim().parse().expect("numeric sphere data"))
            .collect();
        assert_eq!(values.len(), 4, "sphere rows are x, y, z, radius");
        model
            .sphere_positions_body_m
            .push(Vector3::new(values[0], values[1], values[2]));
        model.radii_m.push(values[3]);
    }
    model
}

/// The recorded histories, every step.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Upstream `dataRecL` and `dataRecF`.
    states: [MessageLog<SpacecraftStateMsg>; 2],
    /// The electrostatic force on each spacecraft (upstream records none).
    forces: [MessageLog<InertialForceCommandMsg>; 2],
    /// Upstream `chargeMsmOutMsgs[i].read()` after the run.
    charges: [ChargeMsmMsg; 2],
    /// Upstream `P` \[s\].
    period_s: f64,
    csv_paths: Vec<PathBuf>,
}

fn run(config: Config) -> Results {
    let mu = MU_EARTH * 1e9;
    let elements = ClassicElements {
        semi_major_axis: 42164.0 * 1e3,
        eccentricity: 0.0,
        ..Default::default()
    };
    let (r_ln_n, v_ln_n) = elements_to_state(mu, &elements);
    let initial = [
        (r_ln_n, v_ln_n),
        (r_ln_n + Vector3::from(FOLLOWER_OFFSET_M), v_ln_n),
    ];
    let mut spacecraft = initial.map(|(position_m, velocity_mps)| {
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 1.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::identity(),
            integration_step_nanos: TASK_PERIOD_NANOS,
            initial_position_m: position_m,
            initial_velocity_mps: velocity_mps,
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: None,
        });
        add_point_mass_earth(&mut spacecraft);
        spacecraft
    });
    let period_s = orbit_period_s(mu, elements.semi_major_axis);
    let duration_nanos = sec2nano(0.1 * period_s);

    let model = sphere_model();
    let mut msm = MsmForceTorque::new(MsmForceTorqueConfig {
        name: "msmForceTorqueTag".to_string(),
        space_objects: vec![model.clone(), model.clone()],
    });
    let voltages = config
        .voltages_v
        .map(|voltage_v| Output::new(VoltMsg { voltage_v }));

    let mut state_logs: [MessageLog<SpacecraftStateMsg>; 2] = Default::default();
    let mut force_logs: [MessageLog<InertialForceCommandMsg>; 2] = Default::default();
    let (mut recorder, csv_path) = scenario_recorder("scenarioTwoChargedSC", config.write_csv);

    let simulation_epoch = Epoch::from_gregorian_utc_at_midnight(2019, 1, 1);
    let mut simulation = Simulation::new(simulation_epoch, false);
    for (index, spacecraft) in spacecraft.iter_mut().enumerate() {
        let name = NAMES[index];
        let mut force = ExtForceTorque::new(ExtForceTorqueConfig {
            name: format!("eForce{name}"),
            ext_force_inertial_n: Vector3::zeros(),
            ext_force_body_n: Vector3::zeros(),
            ext_torque_body_nm: Vector3::zeros(),
        });
        connect!(&simulation,
            &spacecraft.state_out => &mut msm.sc_state_in_msgs[index],
            &voltages[index] => &mut msm.volt_in_msgs[index],
            &msm.e_force_out_msgs[index] => &mut force.cmd_force_inertial_in_msg,
            &spacecraft.state_out => &mut state_logs[index].input,
            &msm.e_force_out_msgs[index] => &mut force_logs[index].input,
            &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
                CsvSourceConfig::columns(
                    vector_columns("position_m", &format!("{name}_r_BN_N_m"))
                        .into_iter()
                        .chain(vector_columns("velocity_mps", &format!("{name}_v_BN_N_m_per_s")))
                        .chain(vector_columns("sigma_bn", &format!("{name}_sigma_BN"))),
                ),
            ),
        );
        spacecraft.add_dynamic_effector(force);
    }
    let charge_outputs = [
        msm.charge_msm_out_msgs[0].clone(),
        msm.charge_msm_out_msgs[1].clone(),
    ];

    // Upstream's task order (all at the default priority): both spacecraft,
    // the MSM module (the force effectors are part of the spacecraft), then
    // the recorders.
    let [leader, follower] = &mut spacecraft;
    let [leader_log, follower_log] = &mut state_logs;
    let [leader_force_log, follower_force_log] = &mut force_logs;
    schedule! { simulation,
        "Leader" => leader, period=TASK_PERIOD_NANOS, priority=100;
        "Follower" => follower, period=TASK_PERIOD_NANOS, priority=100;
        "msmForceTorqueTag" => &mut msm, period=TASK_PERIOD_NANOS, priority=90;
        "dataRecL" => leader_log, period=TASK_PERIOD_NANOS, priority=0;
        "dataRecF" => follower_log, period=TASK_PERIOD_NANOS, priority=0;
        "forceRecL" => leader_force_log, period=TASK_PERIOD_NANOS, priority=0;
        "forceRecF" => follower_force_log, period=TASK_PERIOD_NANOS, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, TASK_PERIOD_NANOS, 0);
    }
    simulation.run_for(duration_nanos);
    drop(simulation);
    drop(recorder);

    let charges = charge_outputs.map(|output| output.read());
    let mut csv_paths: Vec<PathBuf> = csv_path.into_iter().collect();
    if config.write_csv {
        csv_paths.push(write_spheres_csv(&model, &charges));
    }
    Results {
        states: state_logs,
        forces: force_logs,
        charges,
        period_s,
        csv_paths,
    }
}

/// Writes `scenarioTwoChargedSCSpheres.csv`: the sphere model and the final
/// sphere charges of both spacecraft.
fn write_spheres_csv(model: &MsmSpaceObject, charges: &[ChargeMsmMsg; 2]) -> PathBuf {
    let mut text = String::from(
        "sphere,r_SB_B_m_0,r_SB_B_m_1,r_SB_B_m_2,radius_m,leaderCharge_C,followerCharge_C\n",
    );
    for (index, (position, radius)) in model
        .sphere_positions_body_m
        .iter()
        .zip(&model.radii_m)
        .enumerate()
    {
        writeln!(
            text,
            "{index},{:.18e},{:.18e},{:.18e},{:.18e},{:.18e},{:.18e}",
            position.x,
            position.y,
            position.z,
            radius,
            charges[0].charges_c[index],
            charges[1].charges_c[index],
        )
        .expect("writing to a string");
    }
    let path = scenario_output_path("scenarioTwoChargedSCSpheres.csv");
    std::fs::write(&path, text).expect("write the sphere CSV");
    path
}

fn main() {
    let mut cli = Cli::from_env("scenario_two_charged_sc [--all]");
    cli.switch("all");
    cli.finish();

    for config in TEST_PARAMETRIZATIONS {
        let results = run(config);
        for path in &results.csv_paths {
            println!("wrote {}", path.display());
        }
        let separation = |index: usize| {
            (results.states[0].values[index].position_m
                - results.states[1].values[index].position_m)
                .norm()
        };
        let last = results.states[0].values.len() - 1;
        println!(
            "separation {:.3} m -> {:.3} m; total charges {:.3e} C and {:.3e} C",
            separation(0),
            separation(last),
            results.charges[0].charges_c.iter().sum::<f64>(),
            results.charges[1].charges_c.iter().sum::<f64>(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream runs this scenario in `test_scenarioTest.py`, which only
    //! checks that it runs. These tests check the electrostatics it
    //! demonstrates: opposite charges on the two buses, equal and opposite
    //! forces, and an attraction that pulls the spacecraft closer than the
    //! orbital motion alone brings them.

    use super::*;

    fn quiet(voltages_v: [f64; 2]) -> Results {
        run(Config {
            voltages_v,
            write_csv: false,
        })
    }

    fn separation(results: &Results, index: usize) -> f64 {
        (results.states[0].values[index].position_m - results.states[1].values[index].position_m)
            .norm()
    }

    #[test]
    fn the_buses_carry_opposite_charges() {
        let results = quiet(VOLTAGES_V);
        let [leader, follower] = &results.charges;
        assert_eq!(leader.charges_c.len(), 80);
        assert_eq!(follower.charges_c.len(), 80);
        let leader_total: f64 = leader.charges_c.iter().sum();
        let follower_total: f64 = follower.charges_c.iter().sum();
        // Each bus is a conductor of a few meters: about 1e-7 C at 500 V,
        // shared over its surface spheres.
        assert!(leader.charges_c.iter().all(|charge| *charge < 0.0));
        assert!((-1e-6..-1e-8).contains(&leader_total), "{leader_total} C");
        assert!(follower_total > 0.0);
        // Equal geometry and opposite potentials: nearly opposite charges
        // (the other bus, 40-50 m away, shifts them slightly).
        assert!(((leader_total + follower_total) / leader_total).abs() < 0.05);
    }

    #[test]
    fn the_forces_are_equal_and_opposite() {
        let results = quiet(VOLTAGES_V);
        for (leader, follower) in results.forces[0]
            .values
            .iter()
            .zip(&results.forces[1].values)
        {
            let leader = leader.force_request_inertial_n;
            let follower = follower.force_request_inertial_n;
            assert!((leader + follower).norm() <= 1e-9 * leader.norm());
        }
        // The force pulls the leader towards the follower.
        let first = results.forces[0].values[1].force_request_inertial_n;
        let towards_follower =
            results.states[1].values[1].position_m - results.states[0].values[1].position_m;
        assert!(first.dot(&towards_follower) > 0.0);
        assert!((1e-9..1e-6).contains(&first.norm()), "{} N", first.norm());
    }

    #[test]
    fn the_attraction_closes_the_gap() {
        let charged = quiet(VOLTAGES_V);
        let neutral = quiet([0.0, 0.0]);
        let samples = charged.states[0].values.len();
        assert_eq!(
            samples as u64,
            sec2nano(0.1 * charged.period_s) / TASK_PERIOD_NANOS + 1
        );
        assert!((separation(&charged, 0) - 50.0).abs() < 1e-9);
        let last = samples - 1;
        // The orbital motion alone brings the follower in to about 42 m in
        // a tenth of an orbit; the attraction brings it in further.
        let drift = separation(&neutral, last);
        assert!((35.0..48.0).contains(&drift), "{drift} m");
        let pulled = separation(&charged, last);
        assert!(pulled < drift - 0.01, "{pulled} m vs {drift} m");
        assert!(
            neutral.forces[0]
                .values
                .iter()
                .all(|force| force.force_request_inertial_n == Vector3::zeros())
        );
    }
}
