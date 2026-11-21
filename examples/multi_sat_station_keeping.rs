//! Formation station keeping with attitude modes in a formation: port of
//! upstream `examples/MultiSatBskSim/scenariosMultiSat/scenario_StationKeepingMultiSat.py`.
//!
//! Three spacecraft with the MultiSat dynamics and flight software about the
//! Earth, every task at 1 s, for 4 hours. Spacecraft 0 flies
//! `a = 1.4 R`, `e = 0.2`, `i = 45°`, `Ω = 48.2°`, `ω = 347.8°`,
//! `f = 85.3°`; spacecraft 1 and 2 the same with `f` scaled by 1.001 and
//! 0.999; all start at `sigma_BN = [0.1, 0.6, -0.8]`, at rest.
//!
//! Each spacecraft's `spacecraftReconfig` (station keeping, `ON`) plans and
//! fires the burns of its one 116 N thruster that take it to its target
//! element difference from the chief, overriding the pointing mode's
//! attitude around the burns:
//!
//! | spacecraft | target `δ(a, e, i, Ω, ω, M)` | mode to 2 h | mode 2 h to 4 h |
//! |---|---|---|---|
//! | 0 | `[0, -0.005, -0.001, 0, 0, 0]`, only with the barycenter chief | Sun pointing | location pointing |
//! | 1 | `[0, 0.005, 0, 0, 0, -0.003]` | inertial pointing | Sun pointing |
//! | 2 | `[0, 0, 0.001, 0, 0, 0.003]` | location pointing | inertial pointing |
//!
//! Upstream `run(showPlots, numberSpacecraft, relativeNavigation)` maps to
//! `--[no-]relative-navigation`: the chief is the formation's barycenter
//! (`formationBarycenter`, Cartesian average, in its own process) or
//! spacecraft 0, whose station keeping is then off. The `__main__` default
//! is spacecraft 0; `--all` runs both, as
//! `test_scenarioStationKeepingMultiSat.py`. Upstream's Vizard set-up is not
//! ported.
//!
//! Writes `examples/output/scenarios/scenario_StationKeepingMultiSat<True|False>_sat<i>.csv`
//! (`str(relativeNavigation)`; every recorded group, see `multi_sat::logs`)
//! and, with the barycenter, `..._chief.csv`;
//! `plot_multi_sat_station_keeping.py` draws upstream's 14 figures for
//! spacecraft 1, as upstream's `pull_outputs(showPlots, relativeNavigation, 1)`.
//!
//! Upstream's test only checks that the scenario runs. `tests` checks that
//! the formation reaches its targets: the element differences of the
//! deputies end near them, only the spacecraft keeping station burn fuel,
//! and the batteries stay within their capacity.

#[path = "support/multi_sat/mod.rs"]
mod multi_sat;

use std::path::PathBuf;

use basilisk_rs::messages::NavigationTranslationMsg;
use basilisk_rs::utilities::orbital_motion::ClassicElements;
use multi_sat::cli::Cli;
use multi_sat::environment::Planet;
use multi_sat::fsw::{FswMode, StationKeeping};
use multi_sat::logs::{Logged, SpacecraftLogs, write_translation_csv};
use multi_sat::message_log::MessageLog;
use multi_sat::{
    MultiSatSim, RELATIVE_NAVIGATION_PROCESS_NAME, Rates, dynamics_process_name, hour2nano,
    sec2nano,
};
use nalgebra::Vector3;

const USAGE: &str = "\
multi_sat_station_keeping [--[no-]relative-navigation]
multi_sat_station_keeping --all   both chiefs (upstream's test)";

/// Upstream `numberSpacecraft`.
const NUMBER_SPACECRAFT: usize = 3;
/// Upstream `simulationTime` of each of the two runs.
const PHASE_HOURS: f64 = 2.0;
/// Upstream `samplingTime` \[s\].
const SAMPLING_S: f64 = 10.0;

#[derive(Clone, Copy, Debug)]
struct Config {
    relative_navigation: bool,
    write_csv: bool,
}

impl Config {
    const fn new(relative_navigation: bool) -> Self {
        Self {
            relative_navigation,
            write_csv: true,
        }
    }

    /// Upstream's `str(relativeNavigation)`.
    fn stem(&self) -> String {
        let flag = if self.relative_navigation {
            "True"
        } else {
            "False"
        };
        format!("scenario_StationKeepingMultiSat{flag}")
    }
}

/// The parametrizations of upstream's test.
const TEST_PARAMETRIZATIONS: [Config; 2] = [Config::new(true), Config::new(false)];

/// Upstream `targetClassicOED` of each spacecraft (spacecraft 0's only with
/// the barycenter chief).
const TARGETS: [[f64; 6]; NUMBER_SPACECRAFT] = [
    [0.0, -0.005, -0.001, 0.0, 0.0, 0.0],
    [0.0, 0.005, 0.0, 0.0, 0.0, -0.003],
    [0.0, 0.0, 0.001, 0.0, 0.0, 0.003],
];

// The histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    logs: Vec<SpacecraftLogs>,
    /// Upstream `chiefTransLog`, with the barycenter chief.
    chief: Option<MessageLog<NavigationTranslationMsg>>,
    /// Upstream `oe[i]`.
    elements: Vec<ClassicElements>,
    csv_paths: Vec<PathBuf>,
}

/// Upstream `configure_initial_conditions`.
fn initial_elements(planet_radius_m: f64) -> Vec<ClassicElements> {
    let degree = std::f64::consts::PI / 180.0;
    let chief = ClassicElements {
        semi_major_axis: 1.4 * planet_radius_m,
        eccentricity: 0.2,
        inclination_rad: 45.0 * degree,
        raan_rad: 48.2 * degree,
        arg_periapsis_rad: 347.8 * degree,
        true_anomaly_rad: 85.3 * degree,
        ..Default::default()
    };
    [1.0, 1.001, 0.999]
        .into_iter()
        .map(|scale| ClassicElements {
            true_anomaly_rad: chief.true_anomaly_rad * scale,
            ..chief
        })
        .collect()
}

fn run(config: Config) -> Results {
    let mut sim = MultiSatSim::new(
        Planet::Earth,
        NUMBER_SPACECRAFT,
        Rates::uniform_s(1.0),
        config.relative_navigation,
    )
    .with_fsw();
    let elements = initial_elements(sim.environment.planet_radius_m);
    for (index, oe) in elements.iter().enumerate() {
        sim.set_initial_elements(index, oe, Vector3::new(0.1, 0.6, -0.8), Vector3::zeros());
    }
    let sampling = sec2nano(SAMPLING_S);
    let mut logs: Vec<SpacecraftLogs> = (0..NUMBER_SPACECRAFT)
        .map(|index| SpacecraftLogs::new(&sim, index, sampling, Logged::ALL))
        .collect();
    let mut chief = sim.relative_navigation.as_ref().map(|barycenter| {
        let mut log = MessageLog::default();
        barycenter.trans_out_msg.connect_to(&mut log.input);
        log
    });

    // `runScenario`: modes, chief and station keeping.
    let modes: Vec<_> = sim.fsw.iter().map(|fsw| fsw.mode_request.clone()).collect();
    modes[0].set(FswMode::SunPointing);
    modes[1].set(FswMode::InertialPointing);
    modes[2].set(FswMode::LocationPointing);
    if config.relative_navigation {
        sim.use_barycenter_as_chief();
    } else {
        sim.use_spacecraft_as_chief(0);
    }
    for (index, (fsw, target)) in sim.fsw.iter_mut().zip(TARGETS).enumerate() {
        if index > 0 || config.relative_navigation {
            fsw.station_keeping.set(StationKeeping::On);
            fsw.set_target_classic_oed(target);
        }
    }
    {
        let mut processes = sim.processes();
        for log in &mut logs {
            log.schedule(processes.process(&dynamics_process_name(log.index)));
        }
        if let Some(log) = &mut chief {
            processes
                .process(RELATIVE_NAVIGATION_PROCESS_NAME)
                .add_module("chiefTransLog", log, sampling, -1);
        }
        let phase = hour2nano(PHASE_HOURS);
        processes.execute_to(phase);
        modes[0].set(FswMode::LocationPointing);
        modes[1].set(FswMode::SunPointing);
        modes[2].set(FswMode::InertialPointing);
        processes.execute_to(2 * phase);
    }

    let mut csv_paths = Vec::new();
    if config.write_csv {
        let stem = config.stem();
        for log in &logs {
            csv_paths.push(log.write_csv(&format!("{stem}_sat{}", log.index)));
        }
        if let Some(log) = &chief {
            csv_paths.push(write_translation_csv(&format!("{stem}_chief"), log));
        }
    }
    Results {
        logs,
        chief,
        elements,
        csv_paths,
    }
}

fn main() {
    let mut cli = Cli::from_env(USAGE);
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream's `__main__` runs `relativeNavigation=False`.
        vec![Config::new(cli.flag("relative-navigation", false))]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        for path in &results.csv_paths {
            println!("wrote {}", path.display());
        }
        for log in &results.logs {
            let fuel = log.fuel.as_ref().expect("fuel recorded");
            println!(
                "  spacecraft {}: {:.3} kg of fuel used",
                log.index,
                fuel.values[0].fuel_mass_kg - fuel.last().fuel_mass_kg
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use basilisk_rs::utilities::orbital_motion::{
        eccentric_to_mean_anomaly, state_to_elements, true_to_eccentric_anomaly,
    };
    use multi_sat::dynamics::BATTERY_CAPACITY_J;

    /// Upstream `pull_outputs`'s `oed`: the deputy-minus-chief classical
    /// element difference, `δa` normalized, angles wrapped to ±π.
    fn element_difference(
        mu: f64,
        chief: (Vector3<f64>, Vector3<f64>),
        deputy: (Vector3<f64>, Vector3<f64>),
    ) -> [f64; 6] {
        let chief = state_to_elements(mu, chief.0, chief.1);
        let deputy = state_to_elements(mu, deputy.0, deputy.1);
        let mean_anomaly = |oe: &ClassicElements| {
            eccentric_to_mean_anomaly(
                true_to_eccentric_anomaly(oe.true_anomaly_rad, oe.eccentricity),
                oe.eccentricity,
            )
        };
        let wrap = |angle: f64| {
            let pi = std::f64::consts::PI;
            if angle > pi {
                angle - 2.0 * pi
            } else if angle < -pi {
                angle + 2.0 * pi
            } else {
                angle
            }
        };
        [
            (deputy.semi_major_axis - chief.semi_major_axis) / chief.semi_major_axis,
            deputy.eccentricity - chief.eccentricity,
            deputy.inclination_rad - chief.inclination_rad,
            wrap(deputy.raan_rad - chief.raan_rad),
            wrap(deputy.arg_periapsis_rad - chief.arg_periapsis_rad),
            wrap(mean_anomaly(&deputy) - mean_anomaly(&chief)),
        ]
    }

    fn check(relative_navigation: bool) {
        let results = run(Config {
            relative_navigation,
            write_csv: false,
        });
        let mu = Planet::Earth.body().mu_m3ps2();
        let samples = (2 * hour2nano(PHASE_HOURS) / sec2nano(SAMPLING_S) + 1) as usize;
        let chief_state = |sample: usize| match &results.chief {
            Some(log) => (
                log.values[sample].position_bn_n_m,
                log.values[sample].velocity_bn_n_mps,
            ),
            None => (
                results.logs[0].positions()[sample],
                results.logs[0].velocities()[sample],
            ),
        };
        let last = samples - 1;
        for log in &results.logs {
            assert_eq!(log.trans.values.len(), samples);
            let keeping = log.index > 0 || relative_navigation;
            let fuel = log.fuel.as_ref().expect("fuel recorded");
            let used = fuel.values[0].fuel_mass_kg - fuel.last().fuel_mass_kg;
            let fired = log.thrusters[0]
                .values
                .iter()
                .any(|thrust| thrust.thrust_factor > 0.0);
            assert_eq!(fired, keeping, "spacecraft {} thrusters", log.index);
            if keeping {
                assert!(used > 0.01, "spacecraft {} used {used} kg", log.index);
            } else {
                assert_eq!(used, 0.0);
            }
            let battery = log.battery.as_ref().expect("battery recorded");
            assert!(
                battery
                    .values
                    .iter()
                    .all(|status| { (0.0..=BATTERY_CAPACITY_J).contains(&status.storage_level_j) })
            );

            // The spacecraft keeping station end near their targets: within
            // 3e-4 in `e`, 1e-4 rad in `i`, 1e-3 rad in `Ω` and 2e-3 rad in
            // `ω` and `M` (the targets are 1e-3 to 5e-3; `δa` is not
            // targeted).
            if keeping {
                let difference = element_difference(
                    mu,
                    chief_state(last),
                    (log.positions()[last], log.velocities()[last]),
                );
                let target = TARGETS[log.index];
                let tolerances = [f64::INFINITY, 3e-4, 1e-4, 1e-3, 2e-3, 2e-3];
                for (element, ((actual, wanted), tolerance)) in
                    difference.iter().zip(target).zip(tolerances).enumerate()
                {
                    assert!(
                        (actual - wanted).abs() < tolerance,
                        "relative navigation {relative_navigation}, spacecraft {}, element \
                         {element}: {actual:e} instead of {wanted:e}",
                        log.index
                    );
                }
            }
        }
        // The initial states are upstream's elements.
        assert!(
            (results.logs[1].positions()[0].norm()
                - basilisk_rs::utilities::orbital_motion::elements_to_state(
                    mu,
                    &results.elements[1]
                )
                .0
                .norm())
            .abs()
                < 1e-6
        );
    }

    #[test]
    fn spacecraft_chief() {
        check(false);
    }

    #[test]
    fn barycenter_chief() {
        check(true);
    }
}
