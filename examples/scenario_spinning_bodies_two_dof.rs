//! A spinning cylindrical spacecraft with a two-degree-of-freedom panel
//! assembly: port of upstream `examples/scenarioSpinningBodiesTwoDOF.py`.
//!
//! A 400 kg cylinder (2 m across, 4 m tall) tumbling at
//! `[0.05, -0.05, 0.05] rad/s` in field-free space carries one
//! [`SpinningBodyTwoDofStateEffector`] on its top rim, both joints released
//! from rest away from their spring equilibrium. Upstream
//! `run(show_plots, numberPanels)` maps to `--panels`:
//!
//! * `1`: a massless lower body turning about `x` carries a 50 kg disk
//!   (radius 2 m) turning about `y`: a single panel on a two-axis gimbal
//!   (`k = 100 N m/rad`, `c = 50 N m s/rad`, both joints from 30°);
//! * `2`: two 20 kg, 4 × 2 m plates in series, the first hinged about `x`
//!   at the rim and the second about `y` at the first's far corner
//!   (`k = 50`, `c = 30`, both joints from 60°).
//!
//! RK4, 0.01 s steps, 1 minute. `--all` runs both, as upstream's
//! `test_scenarioSpinningBodiesTwoDOF.py` does. Both joint angles and
//! rates and the hub velocity and rate are recorded every step to
//! `examples/output/scenarios/scenarioSpinningBodiesTwoDOF<panels>.csv`.
//!
//! Upstream's test only checks that both cases run. `tests` checks what a
//! free multibody spacecraft must do: its center of mass stays where
//! upstream's `r_CN_NInit = 0`, `v_CN_NInit = 0` put it, its angular
//! momentum is conserved, the joint dampers only dissipate energy, and the
//! joints settle.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/multibody.rs"]
mod multibody;

use std::path::PathBuf;

use attitude::scenario_recorder;
use basilisk_rs::dynamics::spinning_body_two_dof_state_effector::{
    SpinningBodyTwoDofBodyConfig, SpinningBodyTwoDofConfig, SpinningBodyTwoDofStateEffector,
};
use basilisk_rs::messages::{
    HingedRigidBodyMsg, SpacecraftDiagnosticsMsg, SpacecraftMassPropsMsg, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use hifitime::Epoch;
use message_log::MessageLog;
use multibody::{box_inertia, set_center_of_mass_initial_state};
use nalgebra::{Matrix3, Vector3};

/// Upstream `simulationTimeStep = macros.sec2nano(.01)`.
const STEP_NANOS: u64 = 10_000_000;
/// Upstream `simulationTime = macros.min2nano(1)`.
const DURATION_NANOS: u64 = 60_000_000_000;
/// Upstream `massSC`, `diameter`, `height`.
const HUB_MASS_KG: f64 = 400.0;
const DIAMETER_M: f64 = 2.0;
const HEIGHT_M: f64 = 4.0;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Upstream `numberPanels` (1 or 2).
    number_panels: u8,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(number_panels: u8) -> Self {
        Self {
            number_panels,
            write_csv: true,
        }
    }
}

/// The parametrizations of upstream `test_scenarioSpinningBodiesTwoDOF.py`.
const TEST_PARAMETRIZATIONS: [Config; 2] = [Config::new(1), Config::new(2)];

/// The recorded histories, every step.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    theta1: MessageLog<HingedRigidBodyMsg>,
    theta2: MessageLog<HingedRigidBodyMsg>,
    states: MessageLog<SpacecraftStateMsg>,
    mass_props: MessageLog<SpacecraftMassPropsMsg>,
    diagnostics: MessageLog<SpacecraftDiagnosticsMsg>,
    initial_theta_rad: f64,
    csv_path: Option<PathBuf>,
}

/// Upstream's `spinningBody` for `numberPanels`.
fn spinning_body(number_panels: u8) -> SpinningBodyTwoDofStateEffector {
    let thickness = 0.1;
    let mut config = SpinningBodyTwoDofConfig {
        spinner1_origin_body_m: Vector3::new(
            0.0,
            DIAMETER_M / 2.0,
            HEIGHT_M / 2.0 - thickness / 2.0,
        ),
        ..SpinningBodyTwoDofConfig::new("SpinningBody")
    };
    let joint = |mass_kg, k, c, theta_deg: f64, inertia, center_of_mass, axis| {
        SpinningBodyTwoDofBodyConfig {
            mass_kg,
            spring_constant_nm_per_rad: k,
            damping_nm_s_per_rad: c,
            theta_init_rad: theta_deg.to_radians(),
            theta_dot_init_radps: 0.0,
            center_of_mass_spinner_m: center_of_mass,
            spin_axis_spinner: axis,
            inertia_about_com_spinner_kg_m2: inertia,
        }
    };
    match number_panels {
        1 => {
            let mass = 50.0;
            let radius = DIAMETER_M;
            let transverse = mass / 12.0 * (3.0 * radius * radius + thickness * thickness);
            config.body1 = joint(
                0.0,
                100.0,
                50.0,
                30.0,
                Matrix3::zeros(),
                Vector3::zeros(),
                Vector3::x(),
            );
            config.body2 = joint(
                mass,
                100.0,
                50.0,
                30.0,
                Matrix3::from_diagonal(&Vector3::new(
                    transverse,
                    transverse,
                    mass / 2.0 * radius * radius,
                )),
                Vector3::new(0.0, radius, 0.0),
                Vector3::y(),
            );
            config.spinner2_origin_spinner1_m = Vector3::zeros();
        }
        2 => {
            let mass = 20.0;
            let length = 2.0 * DIAMETER_M;
            let width = DIAMETER_M;
            // Upstream's plate inertia: `length` along y, `width` along x.
            let inertia = box_inertia(mass, width, length, thickness);
            config.body1 = joint(
                mass,
                50.0,
                30.0,
                60.0,
                inertia,
                Vector3::new(0.0, length / 2.0, 0.0),
                Vector3::x(),
            );
            config.body2 = joint(
                mass,
                50.0,
                30.0,
                60.0,
                inertia,
                Vector3::new(-width / 2.0, 0.0, 0.0),
                Vector3::y(),
            );
            config.spinner2_origin_spinner1_m = Vector3::new(-width / 2.0, length / 2.0, 0.0);
        }
        other => panic!("Cannot simulate {other} panels."),
    }
    SpinningBodyTwoDofStateEffector::new(config)
}

fn run(config: Config) -> Results {
    let transverse =
        HUB_MASS_KG / 16.0 * DIAMETER_M * DIAMETER_M + HUB_MASS_KG / 12.0 * HEIGHT_M * HEIGHT_M;
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(
            transverse,
            transverse,
            HUB_MASS_KG / 8.0 * DIAMETER_M * DIAMETER_M,
        )),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.05, -0.05, 0.05),
        integrator: None,
    });
    let spinner = spinning_body(config.number_panels);
    let initial_theta_rad = spinner.config.body1.theta_init_rad;

    let mut theta1_log = MessageLog::<HingedRigidBodyMsg>::default();
    let mut theta2_log = MessageLog::<HingedRigidBodyMsg>::default();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut mass_props_log = MessageLog::<SpacecraftMassPropsMsg>::default();
    let mut diagnostics_log = MessageLog::<SpacecraftDiagnosticsMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(
        &format!("scenarioSpinningBodiesTwoDOF{}", config.number_panels),
        config.write_csv,
    );

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &spinner.spinning_body_out[0] => &mut theta1_log.input,
        &spinner.spinning_body_out[1] => &mut theta2_log.input,
        &spacecraft.state_out => &mut state_log.input,
        &spacecraft.mass_props_out => &mut mass_props_log.input,
        &spacecraft.diagnostics_out => &mut diagnostics_log.input,
        &spinner.spinning_body_out[0] => recorder.add_source::<HingedRigidBodyMsg>(
            CsvSourceConfig::columns([
                ("theta_rad", "theta1_rad"),
                ("theta_dot_radps", "theta1Dot_rad_per_s"),
            ]),
        ),
        &spinner.spinning_body_out[1] => recorder.add_source::<HingedRigidBodyMsg>(
            CsvSourceConfig::columns([
                ("theta_rad", "theta2_rad"),
                ("theta_dot_radps", "theta2Dot_rad_per_s"),
            ]),
        ),
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
            vector_columns("velocity_mps", "v_BN_N_m_per_s")
                .into_iter()
                .chain(vector_columns("omega_radps", "omega_BN_B_rad_per_s")),
        )),
    );
    spacecraft.add_state_effector(spinner);
    // Upstream `r_CN_NInit = v_CN_NInit = 0`.
    set_center_of_mass_initial_state(&mut spacecraft, Vector3::zeros(), Vector3::zeros());

    schedule! { simulation,
        "scObject" => &mut spacecraft, period=STEP_NANOS, priority=100;
        "theta1_log" => &mut theta1_log, period=STEP_NANOS, priority=0;
        "theta2_log" => &mut theta2_log, period=STEP_NANOS, priority=0;
        "state_log" => &mut state_log, period=STEP_NANOS, priority=0;
        "mass_props_log" => &mut mass_props_log, period=STEP_NANOS, priority=0;
        "diagnostics_log" => &mut diagnostics_log, period=STEP_NANOS, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, STEP_NANOS, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        theta1: theta1_log,
        theta2: theta2_log,
        states: state_log,
        mass_props: mass_props_log,
        diagnostics: diagnostics_log,
        initial_theta_rad,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_spinning_bodies_two_dof [--panels 1|2] [--all]");
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(cli.value("panels", 2))]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!(
            "wrote {} (final angles {:.3} and {:.3} deg)",
            results.csv_path.expect("CSV written").display(),
            results.theta1.last().theta_rad.to_degrees(),
            results.theta2.last().theta_rad.to_degrees(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    /// No external force or torque: the center of mass `r_BN + [NB] c_B`
    /// stays at the origin, the inertial angular momentum is conserved, and
    /// the rotational energy (with the springs') only decreases.
    #[test]
    fn a_free_spacecraft_conserves_momentum_and_dissipates_energy() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            assert_eq!(results.states.values.len(), 6001, "{config:?}");
            for (state, mass_props) in results.states.values.iter().zip(&results.mass_props.values)
            {
                let r_cn = state.position_m
                    + mrp_to_dcm(state.sigma_bn).transpose() * mass_props.center_of_mass_body_m;
                assert!(r_cn.norm() < 1.0e-6, "{config:?}: r_CN = {r_cn}");
            }
            let diagnostics = &results.diagnostics.values;
            let momentum = diagnostics[0].rotational_angular_momentum_inertial_kg_m2ps;
            assert!(momentum.norm() > 10.0);
            for sample in diagnostics {
                let drift = sample.rotational_angular_momentum_inertial_kg_m2ps - momentum;
                assert!(
                    drift.norm() < 1.0e-8 * momentum.norm(),
                    "{config:?}: drift {drift}"
                );
            }
            for pair in diagnostics.windows(2) {
                assert!(
                    pair[1].rotational_energy_j <= pair[0].rotational_energy_j + 1.0e-9,
                    "{config:?}: energy rose from {} to {}",
                    pair[0].rotational_energy_j,
                    pair[1].rotational_energy_j
                );
            }
            assert!(
                diagnostics.last().unwrap().rotational_energy_j
                    < 0.9 * diagnostics[0].rotational_energy_j
            );
        }
    }

    /// Released from their initial angles, the damped joints swing back
    /// toward their springs' zero and settle within a few degrees of it
    /// (the tumbling hub's centrifugal load keeps them slightly deflected).
    #[test]
    fn the_damped_joints_settle() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            for log in [&results.theta1, &results.theta2] {
                assert!((log.values[0].theta_rad - results.initial_theta_rad).abs() < 1.0e-12);
                let last = log.last();
                assert!(
                    last.theta_rad.abs() < 0.2 * results.initial_theta_rad,
                    "{config:?}: final angle {} deg",
                    last.theta_rad.to_degrees()
                );
                assert!(last.theta_dot_radps.abs() < 0.01, "{config:?}: {last:?}");
            }
        }
    }
}
