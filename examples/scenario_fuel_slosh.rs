//! Fuel slosh modelled by spring–mass–damper particles in a tank: port of
//! upstream `examples/scenarioFuelSlosh.py`.
//!
//! A 1500 kg hub tumbling at `[0.1, -0.1, 0.1] rad/s` on a 6700 km,
//! `e = 0.01` Earth orbit carries a 400 kg constant-volume [`FuelTank`]
//! (radius 0.5 m at `[0, 0, 0.1] m`, update-only) and three
//! [`LinearSpringMassDamper`] slosh particles of 1500, 1400 and 1300 kg,
//! each on a `k = 1 N/m` spring along one body axis, released from 0.05,
//! -0.025 and -0.015 m. The spacecraft flies a quarter orbit.
//!
//! Upstream `run(show_plots, damping_parameter, timeStep)` maps to
//! `--damping` (the particles' `c` \[N s/m\]) and `--time-step` \[s\]; `--all`
//! runs upstream's three setups, as `test_scenarioFuelSlosh.py` does:
//! (1) no damping at 0.75 s, (2) no damping at 0.3 s, (3) `c = 15` at
//! 0.75 s.
//!
//! basilisk-rs's tank owns its particles (see
//! [`basilisk_rs::dynamics::fuel_tank`]): only the tank is added to the
//! spacecraft. Upstream logs the particles' displacements by reading their
//! `rho` states from the dynamics manager with a `pythonVariableLogger`;
//! here the tank is scheduled as a [`Hosted`] effector whose observer
//! publishes each particle's `ρ` and `ρ̇` after every step as a
//! [`LinearTranslationRigidBodyMsg`]. Upstream's `scObject.logger` of
//! `totOrbEnergy`, `totOrbAngMomPntN_N`, `totRotAngMomPntC_N` and
//! `totRotEnergy` is the spacecraft's diagnostics message.
//!
//! Every step goes to `examples/output/scenarios/scenarioFuelSlosh<setup>.csv`:
//! the hub position and velocity (upstream's orbit figure plots the center
//! of mass, 0.1 m away), the four conserved quantities and the particle
//! displacements.
//!
//! `tests` ports `test_scenarioFuelSlosh.py`: with damping, each particle
//! has reached its final displacement within 0.05 m at the settling time of
//! its second-order system. It also checks the conservation the figures
//! show.

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

use attitude::scenario_recorder;
use basilisk_rs::dynamics::fuel_tank::{FuelTank, FuelTankConfig, FuelTankModel, FuelTankShape};
use basilisk_rs::dynamics::linear_spring_mass_damper::{
    LinearSpringMassDamper, LinearSpringMassDamperConfig,
};
use basilisk_rs::messages::{
    LinearTranslationRigidBodyMsg, Output, SpacecraftDiagnosticsMsg, SpacecraftStateMsg,
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
use multibody::{Hosted, set_center_of_mass_initial_state};
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{orbit_period_s, run_until, sec2nano};

/// Upstream's particles: `(massInit, r_PB_B, pHat_B, rhoInit)`.
const PARTICLES: [(f64, [f64; 3], [f64; 3], f64); 3] = [
    (1500.0, [0.1, 0.0, -0.1], [1.0, 0.0, 0.0], 0.05),
    (1400.0, [0.0, 0.0, 0.1], [0.0, 1.0, 0.0], -0.025),
    (1300.0, [-0.1, 0.0, 0.1], [0.0, 0.0, 1.0], -0.015),
];
/// Upstream `particle.k` \[N/m\].
const SPRING_NPM: f64 = 1.0;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Upstream `damping_parameter` \[N s/m\].
    damping_ns_per_m: f64,
    /// Upstream `timeStep` \[s\].
    time_step_s: f64,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(damping_ns_per_m: f64, time_step_s: f64) -> Self {
        Self {
            damping_ns_per_m,
            time_step_s,
            write_csv: true,
        }
    }

    /// Upstream `setupNo`: 1, 2, 3 for its standard setups, else none.
    fn setup_number(&self) -> Option<u8> {
        match (self.damping_ns_per_m == 0.0, self.time_step_s) {
            (true, 0.75) => Some(1),
            (true, 0.3) => Some(2),
            (false, 0.75) => Some(3),
            _ => None,
        }
    }

    fn file_stem(&self) -> String {
        match self.setup_number() {
            Some(setup) => format!("scenarioFuelSlosh{setup}"),
            None => format!(
                "scenarioFuelSlosh_c{}_dt{}",
                self.damping_ns_per_m, self.time_step_s
            ),
        }
    }
}

/// The parametrizations of upstream `test_scenarioFuelSlosh.py`.
const TEST_PARAMETRIZATIONS: [Config; 3] = [
    Config::new(0.0, 0.75),
    Config::new(0.0, 0.3),
    Config::new(15.0, 0.75),
];

/// The recorded histories, every step (upstream `scLog` and
/// `damperRhoLog`).
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    states: MessageLog<SpacecraftStateMsg>,
    diagnostics: MessageLog<SpacecraftDiagnosticsMsg>,
    particles: [MessageLog<LinearTranslationRigidBodyMsg>; 3],
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let step_nanos = sec2nano(config.time_step_s);
    let earth = Body::Earth;
    let elements = ClassicElements {
        semi_major_axis: 6700.0 * 1000.0,
        eccentricity: 0.01,
        inclination_rad: 30.0_f64.to_radians(),
        raan_rad: 100.0_f64.to_radians(),
        arg_periapsis_rad: 100.0_f64.to_radians(),
        true_anomaly_rad: 0.0,
        ..Default::default()
    };
    let (r_cn_n, v_cn_n) = elements_to_state(earth.mu_m3ps2(), &elements);
    // Upstream `simulationTime = macros.sec2nano(P / 4)`.
    let duration_nanos = sec2nano(orbit_period_s(earth.mu_m3ps2(), elements.semi_major_axis) / 4.0);

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1500.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
        integration_step_nanos: step_nanos,
        initial_position_m: r_cn_n,
        initial_velocity_mps: v_cn_n,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.1, -0.1, 0.1),
        integrator: None,
    });
    spacecraft
        .add_grav_body(earth.point_mass(true))
        .expect("unique central gravity body");

    let mut tank = FuelTank::new(FuelTankConfig {
        r_tb_b_m: Vector3::new(0.0, 0.0, 0.1),
        update_only: true,
        ..FuelTankConfig::new(
            "tank1",
            FuelTankModel::new(
                FuelTankShape::ConstantVolume {
                    radius_tank_init_m: 0.5,
                },
                400.0,
            ),
        )
    });
    for (index, (mass, position, direction, rho)) in PARTICLES.into_iter().enumerate() {
        tank.push_fuel_slosh_particle(LinearSpringMassDamper::new(LinearSpringMassDamperConfig {
            spring_constant_npm: SPRING_NPM,
            damping_ns_per_m: config.damping_ns_per_m,
            rho_init_m: rho,
            rho_dot_init_mps: 0.0,
            mass_init_kg: mass,
            equilibrium_position_body_m: Vector3::from(position),
            direction_body: Vector3::from(direction),
            ..LinearSpringMassDamperConfig::new(format!("particle{}", index + 1))
        }));
    }
    // Upstream's `dynManager.getStateObject(nameOfRhoState)` reads.
    let particle_outputs: [Output<LinearTranslationRigidBodyMsg>; 3] = Default::default();
    let observed_outputs = particle_outputs.clone();
    let tank = Hosted::new(tank).observe(move |tank: &FuelTank, _, _| {
        for (particle, output) in tank.fuel_slosh_particles().iter().zip(&observed_outputs) {
            let particle = particle
                .as_any()
                .downcast_ref::<LinearSpringMassDamper>()
                .expect("spring-mass-damper particle");
            output.write(LinearTranslationRigidBodyMsg {
                rho_m: particle.rho_m,
                rho_dot_mps: particle.rho_dot_mps,
            });
        }
    });

    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut diagnostics_log = MessageLog::<SpacecraftDiagnosticsMsg>::default();
    let mut particle_logs: [MessageLog<LinearTranslationRigidBodyMsg>; 3] = Default::default();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &spacecraft.state_out => &mut state_log.input,
        &spacecraft.diagnostics_out => &mut diagnostics_log.input,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
            vector_columns("position_m", "r_BN_N_m")
                .into_iter()
                .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s")),
        )),
        &spacecraft.diagnostics_out => recorder.add_source::<SpacecraftDiagnosticsMsg>(
            CsvSourceConfig::columns(
                [("orbital_energy_j".to_string(), "totOrbEnergy_J".to_string())]
                    .into_iter()
                    .chain(vector_columns(
                        "orbital_angular_momentum_inertial_kg_m2ps",
                        "totOrbAngMomPntN_N_kg_m2_per_s",
                    ))
                    .chain(vector_columns(
                        "rotational_angular_momentum_inertial_kg_m2ps",
                        "totRotAngMomPntC_N_kg_m2_per_s",
                    ))
                    .chain([("rotational_energy_j".to_string(), "totRotEnergy_J".to_string())]),
            ),
        ),
    );
    for (index, (output, log)) in particle_outputs
        .iter()
        .zip(particle_logs.iter_mut())
        .enumerate()
    {
        connect!(&simulation,
            output => &mut log.input,
            output => recorder.add_source::<LinearTranslationRigidBodyMsg>(CsvSourceConfig::columns([(
                "rho_m".to_string(),
                format!("damperRho{}_m", index + 1),
            )])),
        );
    }
    spacecraft.add_state_effector(tank);
    set_center_of_mass_initial_state(&mut spacecraft, r_cn_n, v_cn_n);

    schedule! { simulation,
        "bskSat" => &mut spacecraft, period=step_nanos, priority=100;
        "state_log" => &mut state_log, period=step_nanos, priority=0;
        "diagnostics_log" => &mut diagnostics_log, period=step_nanos, priority=0;
    }
    for log in &mut particle_logs {
        simulation.add_module("damperRhoLog", log, step_nanos, 0);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, step_nanos, 0);
    }
    run_until(&mut simulation, duration_nanos);
    drop(simulation);
    drop(recorder);

    Results {
        states: state_log,
        diagnostics: diagnostics_log,
        particles: particle_logs,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_fuel_slosh [--damping C] [--time-step DT] [--all]");
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream `__main__`: `run(True, 0.0, 0.75)`.
        vec![Config::new(
            cli.value("damping", 0.0),
            cli.value("time-step", 0.75),
        )]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        let diagnostics = &results.diagnostics.values;
        let first = &diagnostics[0];
        let last = diagnostics.last().expect("diagnostics");
        println!(
            "wrote {} (relative orbital energy change {:.2e}, rotational energy change {:.2e})",
            results.csv_path.expect("CSV written").display(),
            (last.orbital_energy_j - first.orbital_energy_j) / first.orbital_energy_j,
            (last.rotational_energy_j - first.rotational_energy_j) / first.rotational_energy_j,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    /// Upstream `test_scenarioFuelSlosh.py` for the damped setup: for each
    /// particle of mass `m`, `zeta = c / (2 sqrt(m k))`,
    /// `omega_n = sqrt(k / m)`, the 5 % settling time is
    /// `-ln(0.05 sqrt(1 - zeta^2)) / (zeta omega_n)`, and at the first
    /// sample past it the displacement is within `0.05` m of its final value.
    #[test]
    fn damped_particles_settle_as_second_order_systems() {
        for config in TEST_PARAMETRIZATIONS {
            if config.damping_ns_per_m == 0.0 {
                continue;
            }
            let results = quiet(config);
            let accuracy = 0.05;
            for (log, (mass, _, _, _)) in results.particles.iter().zip(PARTICLES) {
                let zeta = config.damping_ns_per_m / (2.0 * (mass * SPRING_NPM).sqrt());
                let omega_n = (SPRING_NPM / mass).sqrt();
                let settling_time =
                    -1.0 / (zeta * omega_n) * (0.05 * (1.0 - zeta * zeta).sqrt()).ln();
                let times = log.times_s();
                let index = times
                    .iter()
                    .position(|time| *time > settling_time)
                    .unwrap_or(0);
                let difference = (log.values[index].rho_m - log.last().rho_m).abs();
                assert!(
                    difference <= accuracy,
                    "particle of {mass} kg: {difference} m from its final value at {settling_time} s"
                );
            }
        }
    }

    /// The particles start from upstream's `rhoInit`, and every step is
    /// logged (upstream `scLog` and `damperRhoLog` at `simulationTimeStep`).
    #[test]
    fn the_particles_start_displaced() {
        let results = quiet(TEST_PARAMETRIZATIONS[2]);
        for (log, (_, _, _, rho)) in results.particles.iter().zip(PARTICLES) {
            assert_eq!(log.values[0].rho_m, rho);
            assert_eq!(log.times_nanos[1], sec2nano(0.75));
        }
        assert_eq!(
            results.states.values.len(),
            results.diagnostics.values.len()
        );
    }

    /// What upstream's figures show: the slosh is internal, so the orbital
    /// energy and angular momentum and the rotational angular momentum are
    /// conserved in every setup (to the integration error, smaller at the
    /// smaller step), and so is the rotational energy without damping; the
    /// dampers dissipate it.
    #[test]
    fn energy_and_momentum_are_conserved_without_damping() {
        let mut undamped_energy_drift = Vec::new();
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            let first = &results.diagnostics.values[0];
            let relative = |value: f64, initial: f64| ((value - initial) / initial).abs();
            let mut rotational_energy_drift = 0.0_f64;
            for sample in &results.diagnostics.values {
                assert!(
                    relative(sample.orbital_energy_j, first.orbital_energy_j) < 1.0e-9,
                    "{config:?}"
                );
                let orbital = sample.orbital_angular_momentum_inertial_kg_m2ps
                    - first.orbital_angular_momentum_inertial_kg_m2ps;
                assert!(
                    orbital.norm()
                        < 1.0e-9 * first.orbital_angular_momentum_inertial_kg_m2ps.norm(),
                    "{config:?}"
                );
                let rotational = sample.rotational_angular_momentum_inertial_kg_m2ps
                    - first.rotational_angular_momentum_inertial_kg_m2ps;
                assert!(
                    rotational.norm()
                        < 1.0e-5 * first.rotational_angular_momentum_inertial_kg_m2ps.norm(),
                    "{config:?}: {rotational}"
                );
                rotational_energy_drift = rotational_energy_drift.max(relative(
                    sample.rotational_energy_j,
                    first.rotational_energy_j,
                ));
            }
            if config.damping_ns_per_m == 0.0 {
                assert!(
                    rotational_energy_drift < 1.0e-5,
                    "{config:?}: {rotational_energy_drift}"
                );
                undamped_energy_drift.push(rotational_energy_drift);
            } else {
                let last = results.diagnostics.last();
                // The dampers take most of the slosh and tumble energy.
                assert!(
                    last.rotational_energy_j < 0.5 * first.rotational_energy_j,
                    "{config:?}"
                );
            }
        }
        // RK4: 0.3 s steps against 0.75 s ones cut the drift (2.5^4 = 39)-fold.
        assert!(
            undamped_energy_drift[1] < undamped_energy_drift[0] / 10.0,
            "{undamped_energy_drift:?}"
        );
    }
}
