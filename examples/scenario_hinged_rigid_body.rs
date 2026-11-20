//! A Δv maneuver on a spacecraft with two hinged solar panels: port of
//! upstream `examples/scenarioHingedRigidBody.py`.
//!
//! An 800 kg hub carries two 100 kg panels, each a
//! [`HingedRigidBodyStateEffector`] (`d = 1.5 m`, `k = 1000 N m/rad`, no
//! damping) hinged at `[±0.5, 0, 1] m` and released from 5°. The spacecraft
//! flies a near-circular 7000 km equatorial Earth orbit, coasts for 1 % of
//! the period, then an `extForceTorque` (`maneuverThrust`) pushes it with
//! `[-2050, -1430, -0.00076] N` (inertial) for 935 s, the Δv of
//! `scenarioOrbitManeuver`; the panels swing under the acceleration and the
//! hub rotates with them. RK4, 0.1 s steps.
//!
//! Upstream changes `extFTObject.extForce_N` between the two runs. Here the
//! effector's inertial force command input (upstream `cmdForceInertialInMsg`,
//! summed with `extForce_N`) is connected to a stand-alone message written
//! between the runs, which the effector reads from the next step on, as
//! upstream's changed attribute.
//!
//! The spacecraft state and both panel angles are recorded every
//! `samplingTime(0.01 P, 0.1 s, 100)` = 0.5 s, over both runs, to
//! `examples/output/scenarios/scenarioHingedRigidBody.csv`.
//!
//! `tests` ports upstream `test_scenarioHingedRigidBody.py`: the final
//! inertial speed against its truth value.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/multibody.rs"]
mod multibody;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;

use std::path::PathBuf;

use attitude::{sampling_period_nanos, scenario_recorder};
use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::hinged_rigid_body_state_effector::{
    HingedRigidBodyConfig, HingedRigidBodyStateEffector,
};
use basilisk_rs::messages::{
    HingedRigidBodyMsg, InertialForceCommandMsg, Output, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use multibody::set_center_of_mass_initial_state;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{orbit_period_s, run_until, sec2nano};

/// Upstream `simulationTimeStep = macros.sec2nano(0.1)`.
const STEP_NANOS: u64 = 100_000_000;
/// Upstream `T2 = macros.sec2nano(935.)`: the burn.
const BURN_NANOS: u64 = 935_000_000_000;
/// Upstream's second-run `extFTObject.extForce_N` \[N\].
const MANEUVER_FORCE_N: [f64; 3] = [-2050.0, -1430.0, -0.00076];

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The recorded histories, every 0.5 s.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    states: MessageLog<SpacecraftStateMsg>,
    panel1: MessageLog<HingedRigidBodyMsg>,
    panel2: MessageLog<HingedRigidBodyMsg>,
    /// End of the coast, start of the burn \[ns\].
    coast_nanos: u64,
    csv_path: Option<PathBuf>,
}

/// Upstream's panel: 100 kg, `IPntS_S = diag(100, 50, 50)`, `d = 1.5 m`,
/// `k = 1000 N m/rad`, `c = 0`, released from 5° at rest.
fn panel(
    name: &str,
    hinge_position_body_m: Vector3<f64>,
    dcm_hb: Matrix3<f64>,
) -> HingedRigidBodyStateEffector {
    HingedRigidBodyStateEffector::new(HingedRigidBodyConfig {
        mass_kg: 100.0,
        inertia_about_panel_com_panel_kg_m2: Matrix3::from_diagonal(&Vector3::new(
            100.0, 50.0, 50.0,
        )),
        center_of_mass_offset_m: 1.5,
        spring_constant_nm_per_rad: 1000.0,
        damping_nm_s_per_rad: 0.0,
        hinge_position_body_m,
        body_to_hinge_dcm: dcm_hb,
        theta_init_rad: 5.0_f64.to_radians(),
        theta_dot_init_radps: 0.0,
        ..HingedRigidBodyConfig::new(name)
    })
}

fn run(config: Config) -> Results {
    let earth = Body::Earth;
    let elements = ClassicElements {
        semi_major_axis: 7000.0 * 1000.0,
        eccentricity: 0.0001,
        inclination_rad: 0.0,
        raan_rad: 48.2_f64.to_radians(),
        arg_periapsis_rad: 347.8_f64.to_radians(),
        true_anomaly_rad: 85.3_f64.to_radians(),
        ..Default::default()
    };
    let (r_cn_n, v_cn_n) = elements_to_state(earth.mu_m3ps2(), &elements);
    // Upstream `simulationTime = sec2nano(0.01 P)`.
    let coast_nanos = sec2nano(0.01 * orbit_period_s(earth.mu_m3ps2(), elements.semi_major_axis));

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 800.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: r_cn_n,
        initial_velocity_mps: v_cn_n,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(earth.point_mass(true))
        .expect("unique central gravity body");

    let panel1 = panel(
        "panel1",
        Vector3::new(0.5, 0.0, 1.0),
        Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0)),
    );
    let panel2 = panel("panel2", Vector3::new(-0.5, 0.0, 1.0), Matrix3::identity());
    let mut thrust = ExtForceTorque::new(ExtForceTorqueConfig {
        name: "maneuverThrust".to_string(),
        ext_force_inertial_n: Vector3::zeros(),
        ext_force_body_n: Vector3::zeros(),
        ext_torque_body_nm: Vector3::zeros(),
    });
    let maneuver_force = Output::<InertialForceCommandMsg>::default();

    let sample_period = sampling_period_nanos(coast_nanos, STEP_NANOS, 100);
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut panel1_log = MessageLog::<HingedRigidBodyMsg>::default();
    let mut panel2_log = MessageLog::<HingedRigidBodyMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder("scenarioHingedRigidBody", config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &maneuver_force => &mut thrust.cmd_force_inertial_in_msg,
        &spacecraft.state_out => &mut state_log.input,
        &panel1.hinged_rigid_body_out => &mut panel1_log.input,
        &panel2.hinged_rigid_body_out => &mut panel2_log.input,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
            vector_columns("position_m", "r_BN_N_m")
                .into_iter()
                .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s")),
        )),
        &panel1.hinged_rigid_body_out => recorder.add_source::<HingedRigidBodyMsg>(
            CsvSourceConfig::columns([
                ("theta_rad", "panel1_theta_rad"),
                ("theta_dot_radps", "panel1_thetaDot_rad_per_s"),
            ]),
        ),
        &panel2.hinged_rigid_body_out => recorder.add_source::<HingedRigidBodyMsg>(
            CsvSourceConfig::columns([
                ("theta_rad", "panel2_theta_rad"),
                ("theta_dot_radps", "panel2_thetaDot_rad_per_s"),
            ]),
        ),
    );
    spacecraft.add_state_effector(panel1);
    spacecraft.add_state_effector(panel2);
    spacecraft.add_dynamic_effector(thrust);
    set_center_of_mass_initial_state(&mut spacecraft, r_cn_n, v_cn_n);

    schedule! { simulation,
        "bskSat" => &mut spacecraft, period=STEP_NANOS, priority=100;
        "state_log" => &mut state_log, period=sample_period, priority=0;
        "panel1_log" => &mut panel1_log, period=sample_period, priority=0;
        "panel2_log" => &mut panel2_log, period=sample_period, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    run_until(&mut simulation, coast_nanos);
    maneuver_force.write(InertialForceCommandMsg {
        force_request_inertial_n: Vector3::from(MANEUVER_FORCE_N),
    });
    run_until(&mut simulation, coast_nanos + BURN_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        states: state_log,
        panel1: panel1_log,
        panel2: panel2_log,
        coast_nanos,
        csv_path,
    }
}

fn main() {
    Cli::from_env("scenario_hinged_rigid_body").finish();
    let results = run(Config { write_csv: true });
    println!(
        "wrote {} (final |v_BN_N| = {:.6} m/s)",
        results.csv_path.expect("CSV written").display(),
        results.states.last().velocity_mps.norm(),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Upstream `test_scenarioHingedRigidBody.py`: the speed after the burn,
    /// `|v_BN_N|` of the last recorded state, is `8470.84340921` m/s within
    /// `345.1819`.
    #[test]
    fn post_maneuver_speed_matches_upstream() {
        let results = run(Config { write_csv: false });
        let speed = results.states.last().velocity_mps.norm();
        let truth = 8470.84340921;
        let accuracy = 345.1819;
        assert!(
            (speed - truth).abs() <= accuracy,
            "post-maneuver speed {speed} m/s, truth {truth}"
        );
        // Samples every 0.5 s from 0 to the last tick before 0.01 P + 935 s.
        assert_eq!(results.states.times_nanos[1], 500_000_000);
        assert_eq!(results.states.values.len(), results.panel1.values.len());
    }

    /// What the scenario shows: during the coast the undamped panels swing
    /// within their initial 5°, the hub only rocking with them; the burn's
    /// 2.5 m/s^2 then deflects them well past it.
    #[test]
    fn the_burn_deflects_the_swinging_panels() {
        let results = run(Config { write_csv: false });
        let initial = 5.0_f64.to_radians();
        for log in [&results.panel1, &results.panel2] {
            assert!((log.values[0].theta_rad - initial).abs() < 1.0e-12);
            let peak = |burning: bool| {
                log.times_nanos
                    .iter()
                    .zip(&log.values)
                    .filter(|(time, _)| (**time > results.coast_nanos) == burning)
                    .fold(0.0_f64, |max, (_, panel)| max.max(panel.theta_rad.abs()))
            };
            assert!(peak(false) < initial * 1.01, "coast peak {}", peak(false));
            assert!(peak(true) > 3.0 * initial, "burn peak {}", peak(true));
        }
    }
}
