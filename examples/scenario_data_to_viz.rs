//! Spacecraft states from another program's text file, replayed for Vizard:
//! port of upstream `examples/scenarioDataToViz.py`.
//!
//! `dataFileToViz` reads a comma-separated file of two spacecraft states,
//! one line per 10 s update (the step and duration come from the file's time
//! column): a servicer holding a target-frame fixed position next to a
//! slowly rotating target, both with the same attitude, on a geostationary
//! orbit. Its spacecraft state messages feed a Vizard interface, which saves
//! the replay to `examples/output/vizard/scenarioDataToViz<attType>.bin`,
//! with the trajectories relative to the chief, spacecraft labels, a 0.2
//! shadow brightness, the Earth, and upstream's custom CAD models (upstream
//! `examples/dataForExamples/Aura_27.obj` for the target and
//! `Loral-1300Com-main.obj` for the servicer, referenced by upstream's
//! relative paths, not fetched). Upstream only streams to a live Vizard
//! (its `saveFile` is commented out); the file is its offline counterpart.
//!
//! Needs the `vizard` feature, which `dataFileToViz` belongs to:
//!
//! ```text
//! cargo run --release --features vizard --example scenario_data_to_viz -- --all
//! ```
//!
//! Upstream `run(show_plots, attType)` maps to `--att-type 0|1` (0: the file
//! gives MRPs, `scHoldTraj_rotating_MRP.csv`; 1: Euler parameters,
//! `scHoldTraj_rotating_EP.csv`); the script's default is 0 and `--all` runs
//! both cases of upstream's `test_scenarioDataToViz.py`. Reads
//! `assets/data_for_examples/` (`just fetch-misc-scenario-data`); writes
//! `examples/output/scenarios/scenarioDataToViz<attType>.csv` (both
//! positions and attitudes, about 100 samples).

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
#[path = "support/viz.rs"]
mod viz;

use std::path::{Path, PathBuf};

use attitude::{sampling_period_nanos, scenario_recorder};
use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::vizard::data_file_to_viz::{AttitudeType, DataFileToViz, DataFileToVizConfig};
use basilisk_rs::vizard::viz_structures::{CustomModel, VizSetting};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;

/// Upstream `ModelTag`s of the two spacecraft.
const NAMES: [&str; 2] = ["servicer", "target"];

/// Upstream's data file of `attType`.
fn data_file(att_type: u8) -> PathBuf {
    let name = match att_type {
        0 => "scHoldTraj_rotating_MRP.csv",
        1 => "scHoldTraj_rotating_EP.csv",
        _ => panic!("unknown attType variable {att_type}"),
    };
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("assets/data_for_examples")
        .join(name)
}

/// Upstream's `t0`, `t1` and `tN` from the file's first column: the step
/// `t1 - t0` and the duration `tN - t0` [ns].
fn file_timing(path: &Path) -> (u64, u64) {
    let text = std::fs::read_to_string(path).unwrap_or_else(|error| {
        panic!(
            "cannot read {} ({error}); run just fetch-misc-scenario-data",
            path.display()
        )
    });
    let times: Vec<f64> = text
        .lines()
        .skip(1)
        .filter(|line| !line.trim().is_empty())
        .map(|line| {
            line.split(',')
                .next()
                .unwrap_or("")
                .trim()
                .parse()
                .expect("numeric time")
        })
        .collect();
    let to_nanos = |seconds: f64| (seconds * 1e9).round() as u64;
    (
        to_nanos(times[1] - times[0]),
        to_nanos(times[times.len() - 1] - times[0]),
    )
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    states: Vec<MessageLog<SpacecraftStateMsg>>,
    csv_path: Option<PathBuf>,
    vizard_path: PathBuf,
}

fn run(att_type: u8, write_csv: bool) -> Results {
    let path = data_file(att_type);
    let (step_nanos, duration_nanos) = file_timing(&path);
    let mut data_module = DataFileToViz::new(DataFileToVizConfig {
        name: "testModule".to_string(),
        data_file_name: path,
        delimiter: ',',
        attitude_type: if att_type == 0 {
            AttitudeType::Mrp
        } else {
            AttitudeType::Quaternion
        },
        ..DataFileToVizConfig::default()
    });
    for _ in NAMES {
        data_module.add_spacecraft();
    }

    let sample_period = sampling_period_nanos(duration_nanos, step_nanos, 100);
    let mut state_logs: Vec<MessageLog<SpacecraftStateMsg>> =
        NAMES.iter().map(|_| MessageLog::default()).collect();
    let stem = format!("scenarioDataToViz{att_type}");
    let (mut recorder, csv_path) = scenario_recorder(&stem, write_csv);
    let vizard_path = viz::vizard_file(&stem);

    let mut simulation =
        Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 12, 29), false);
    // Upstream `enableUnityVisualization(scSim, simTaskName, scList)` with
    // the states read from the data module.
    let mut vizard = viz::saving_interface(vizard_path.clone());
    vizard.settings.true_trajectory_lines_on = 2;
    vizard.settings.show_spacecraft_labels = VizSetting::Enabled;
    vizard.settings.spacecraft_shadow_brightness = 0.2;
    vizard.add_celestial_body(viz::grav_body_info(Body::Earth));
    vizard.settings.custom_model_list.push(CustomModel {
        model_path: "../dataForExamples/Aura_27.obj".to_string(),
        shader: 1,
        sim_bodies_to_modify: vec![NAMES[1].to_string()],
        rotation_rad: Vector3::new(180.0_f64.to_radians(), 0.0, -90.0_f64.to_radians()),
        ..CustomModel::default()
    });
    vizard.settings.custom_model_list.push(CustomModel {
        model_path: "../dataForExamples/Loral-1300Com-main.obj".to_string(),
        sim_bodies_to_modify: vec![NAMES[0].to_string()],
        rotation_rad: Vector3::new(0.0, -90.0_f64.to_radians(), 0.0),
        scale: Vector3::repeat(0.09),
        ..CustomModel::default()
    });
    for (index, name) in NAMES.iter().enumerate() {
        let output = &data_module.spacecraft[index].sc_state_out_msg;
        let spacecraft = vizard.add_spacecraft(*name);
        simulation.connect(output, &mut spacecraft.sc_state_in_msg);
        connect!(&simulation,
            output => &mut state_logs[index].input,
            output => recorder.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
                vector_columns("position_m", &format!("r_B{}N_N_m", index + 1))
                    .into_iter()
                    .chain(vector_columns("sigma_bn", &format!("sigma_B{}N", index + 1))),
            )),
        );
    }
    schedule! { simulation,
        "testModule" => &mut data_module, period=step_nanos, priority=0;
        "vizInterface" => &mut vizard, period=step_nanos, priority=-10;
    }
    for log in &mut state_logs {
        simulation.add_module("dataLog", log, sample_period, -5);
    }
    if write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, -5);
    }
    simulation.run_for(duration_nanos);
    drop(simulation);
    drop(recorder);
    drop(vizard);

    Results {
        states: state_logs,
        csv_path,
        vizard_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_data_to_viz [--att-type 0|1] [--all]");
    let cases = if cli.switch("all") {
        vec![0, 1]
    } else {
        // Upstream's `__main__` runs `run(True, 0)`.
        vec![cli.value("att-type", 0_u8)]
    };
    cli.finish();
    for att_type in cases {
        let results = run(att_type, true);
        let rho = results.states[1].last().position_m - results.states[0].last().position_m;
        println!(
            "wrote {} and {} (final |rho| = {:.2} m)",
            results.csv_path.expect("CSV written").display(),
            results.vizard_path.display(),
            rho.norm()
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioDataToViz.py` only checks that both cases run.
    //! These tests check the replay: the messages hold the file's states,
    //! the MRP and Euler-parameter files describe the same motion, and the
    //! Vizard file holds every frame of both spacecraft.

    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
    use basilisk_rs::vizard::viz_interface::read_viz_file;

    use super::*;

    #[test]
    fn the_replay_holds_the_file_states() {
        let mrp = run(0, false);
        let ep = run(1, false);
        // The first data line, km to m.
        let first = &mrp.states[1].values[0];
        assert_eq!(
            first.position_m,
            Vector3::new(0.0, 42463.9999999575, 0.0) * 1e3
        );
        assert_eq!(
            mrp.states[0].values[0].sigma_bn,
            Vector3::new(0.0, 0.0, 0.414213562373095)
        );
        for spacecraft in 0..2 {
            assert_eq!(
                mrp.states[spacecraft].values.len(),
                ep.states[spacecraft].values.len()
            );
            for (from_mrp, from_ep) in mrp.states[spacecraft]
                .values
                .iter()
                .zip(&ep.states[spacecraft].values)
            {
                assert!((from_mrp.position_m - from_ep.position_m).norm() < 1e-6);
                let difference = mrp_to_dcm(from_mrp.sigma_bn) - mrp_to_dcm(from_ep.sigma_bn);
                assert!(difference.abs().max() < 1e-6, "{difference}");
            }
        }
        // The servicer holds a fixed distance from the target at the end,
        // with the target's attitude.
        let rho = |sample: usize| {
            (mrp.states[1].values[sample].position_m - mrp.states[0].values[sample].position_m)
                .norm()
        };
        let last = mrp.states[0].values.len() - 1;
        assert!(
            (rho(last) - rho(last - 5)).abs() < 1e-3 * rho(last),
            "{} {}",
            rho(last),
            rho(last - 5)
        );
        let attitude =
            mrp_to_dcm(mrp.states[0].last().sigma_bn) - mrp_to_dcm(mrp.states[1].last().sigma_bn);
        assert!(attitude.abs().max() < 1e-3, "{attitude}");

        let frames = read_viz_file(&mrp.vizard_path).expect("readable Vizard file");
        let (step, duration) = file_timing(&data_file(0));
        // One frame per update after t = 0, as upstream's `UpdateState`.
        assert_eq!(frames.len() as u64, duration / step);
        let names: Vec<&str> = frames
            .last()
            .unwrap()
            .spacecraft
            .iter()
            .map(|spacecraft| spacecraft.spacecraft_name.as_str())
            .collect();
        assert_eq!(names, NAMES);
    }
}
