//! Attitude guidance modes switched per spacecraft in a formation: port of
//! upstream `examples/MultiSatBskSim/scenariosMultiSat/scenario_AttGuidMultiSat.py`.
//!
//! Three spacecraft on the orbits of
//! [`multi_sat_basic_orbit`](multi_sat_basic_orbit.rs) about the Earth, now
//! each with the MultiSat flight software (`multi_sat::fsw`: inertial, Sun
//! and ground-location pointing, `attTrackingError`, `mrpFeedback` and
//! `rwMotorTorque` on the four wheels). Every task runs at 10 s. The modes,
//! requested per spacecraft and switched by upstream's events:
//!
//! | spacecraft | 0 to 1 h | 1 h to 2 h |
//! |---|---|---|
//! | 0 | Sun pointing (solar panel `+b3` at the Sun) | Sun pointing |
//! | 1 | standby (no control) | Sun pointing |
//! | 2 | location pointing (`+b1` at Boulder) | inertial pointing (`sigma_R0N = [0.1, 0.2, -0.3]`) |
//!
//! Upstream `run(show_plots, numberSpacecraft)` takes 3 spacecraft (the
//! initial conditions are for three); `--all` runs that, as
//! `test_scenarioAttGuidMultiSat.py`. Upstream's Vizard set-up is not
//! ported.
//!
//! Writes `examples/output/scenarios/scenario_AttGuidMultiSat_sat<i>.csv`
//! with the navigation, guidance and wheel histories (columns: see
//! `multi_sat::logs`); `plot_multi_sat_att_guid.py` draws upstream's figures
//! for spacecraft 2, as upstream's `pull_outputs(show_plots, 2)`.
//!
//! Upstream's test only checks that the scenario runs. `tests` checks what
//! the modes do: the attitude errors converge in every controlled mode,
//! standby leaves the attitude and wheels alone, and inertial pointing ends
//! at `sigma_R0N`.

#[path = "support/multi_sat/mod.rs"]
mod multi_sat;

use std::path::PathBuf;

use basilisk_rs::utilities::orbital_motion::ClassicElements;
use multi_sat::cli::Cli;
use multi_sat::environment::Planet;
use multi_sat::fsw::FswMode;
use multi_sat::logs::{Logged, SpacecraftLogs};
use multi_sat::{MultiSatSim, Rates, dynamics_process_name, hour2nano, sec2nano};
use nalgebra::Vector3;

const USAGE: &str = "\
multi_sat_att_guid [--number-spacecraft 3]
multi_sat_att_guid --all   upstream's test case (3 spacecraft)";

/// Upstream `simulationTime` of each of the two runs.
const PHASE_HOURS: f64 = 1.0;

#[derive(Clone, Copy, Debug)]
struct Config {
    number_spacecraft: usize,
    write_csv: bool,
}

// The histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    logs: Vec<SpacecraftLogs>,
    csv_paths: Vec<PathBuf>,
}

/// Upstream `configure_initial_conditions`: three orbits about a planet of
/// radius `planet_radius_m`, all at `sigma_BN = [0.1, 0.2, -0.3]`, at rest.
fn initial_elements(planet_radius_m: f64) -> [ClassicElements; 3] {
    let degree = std::f64::consts::PI / 180.0;
    let first = ClassicElements {
        semi_major_axis: 1.1 * planet_radius_m,
        eccentricity: 0.01,
        inclination_rad: 45.0 * degree,
        raan_rad: 48.2 * degree,
        arg_periapsis_rad: 347.8 * degree,
        true_anomaly_rad: 85.3 * degree,
        ..Default::default()
    };
    [
        first,
        ClassicElements {
            semi_major_axis: 1.2 * planet_radius_m,
            eccentricity: 0.1,
            inclination_rad: 30.0 * degree,
            ..first
        },
        ClassicElements {
            semi_major_axis: 1.3 * planet_radius_m,
            eccentricity: 0.05,
            inclination_rad: 60.0 * degree,
            ..first
        },
    ]
}

fn run(config: Config) -> Results {
    assert_eq!(
        config.number_spacecraft, 3,
        "upstream's initial conditions are for three spacecraft"
    );
    let mut sim = MultiSatSim::new(
        Planet::Earth,
        config.number_spacecraft,
        Rates::uniform_s(10.0),
        false,
    )
    .with_fsw();
    for (index, elements) in initial_elements(sim.environment.planet_radius_m)
        .iter()
        .enumerate()
    {
        sim.set_initial_elements(
            index,
            elements,
            Vector3::new(0.1, 0.2, -0.3),
            Vector3::zeros(),
        );
    }
    let mut logs: Vec<SpacecraftLogs> = (0..config.number_spacecraft)
        .map(|index| SpacecraftLogs::new(&sim, index, sec2nano(1.0), Logged::ATTITUDE))
        .collect();
    let modes: Vec<_> = sim.fsw.iter().map(|fsw| fsw.mode_request.clone()).collect();

    // `runScenario`.
    modes[0].set(FswMode::SunPointing);
    modes[1].set(FswMode::Standby);
    modes[2].set(FswMode::LocationPointing);
    {
        let mut processes = sim.processes();
        for log in &mut logs {
            log.schedule(processes.process(&dynamics_process_name(log.index)));
        }
        let phase = hour2nano(PHASE_HOURS);
        processes.execute_to(phase);
        modes[1].set(FswMode::SunPointing);
        modes[2].set(FswMode::InertialPointing);
        processes.execute_to(2 * phase);
    }

    let csv_paths = if config.write_csv {
        logs.iter()
            .map(|log| log.write_csv(&format!("scenario_AttGuidMultiSat_sat{}", log.index)))
            .collect()
    } else {
        Vec::new()
    };
    Results { logs, csv_paths }
}

fn main() {
    let mut cli = Cli::from_env(USAGE);
    let _all = cli.switch("all");
    let config = Config {
        number_spacecraft: cli.value("number-spacecraft", 3),
        write_csv: true,
    };
    cli.finish();

    let results = run(config);
    for (log, path) in results.logs.iter().zip(&results.csv_paths) {
        let guidance = log.att_guid.as_ref().expect("guidance recorded");
        println!(
            "wrote {} (final |sigma_BR| = {:.3e})",
            path.display(),
            guidance.last().sigma_br.norm()
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use basilisk_rs::utilities::rigid_body_kinematics::sub_mrp;

    fn sigma_br_at(log: &SpacecraftLogs, nanos: u64) -> f64 {
        log.att_guid
            .as_ref()
            .expect("guidance recorded")
            .at(nanos)
            .sigma_br
            .norm()
    }

    #[test]
    fn modes_do_what_they_ask() {
        let results = run(Config {
            number_spacecraft: 3,
            write_csv: false,
        });
        let phase = hour2nano(PHASE_HOURS);
        let end = 2 * phase;
        let [sun, standby, location] = [&results.logs[0], &results.logs[1], &results.logs[2]];
        let initial_sigma = Vector3::new(0.1, 0.2, -0.3);

        // Sun pointing from the start: converged by the end of each run.
        for time in [phase, end] {
            let error = sigma_br_at(sun, time);
            assert!(
                error < 1e-3,
                "spacecraft 0 |sigma_BR| = {error:e} at {time}"
            );
        }

        // Standby for an hour: no control, so the hub and the wheels stay
        // at rest and the zeroed gateway reads zero.
        for (time, (att, speed)) in standby
            .att
            .times_nanos
            .iter()
            .zip(standby.att.values.iter().zip(&standby.rw_speed.values))
        {
            if *time > phase {
                break;
            }
            assert!((att.sigma_bn - initial_sigma).norm() < 1e-12);
            assert!(speed.wheel_speeds_radps.iter().all(|speed| *speed == 0.0));
        }
        assert_eq!(sigma_br_at(standby, phase), 0.0);
        // Then Sun pointing converges within the second hour.
        let error = sigma_br_at(standby, end);
        assert!(error < 1e-3, "spacecraft 1 |sigma_BR| = {error:e}");

        // Location pointing tracks the station; inertial pointing ends at
        // `sigma_R0N`, at rest.
        let error = sigma_br_at(location, phase);
        assert!(error < 1e-2, "spacecraft 2 location |sigma_BR| = {error:e}");
        let last = location.att.last();
        let attitude_error = sub_mrp(last.sigma_bn, initial_sigma).norm();
        assert!(
            attitude_error < 1e-4,
            "spacecraft 2 inertial error {attitude_error:e}"
        );
        assert!(last.omega_bn_b_radps.norm() < 1e-5);
    }
}
