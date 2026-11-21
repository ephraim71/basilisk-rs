//! Orbit determination from the limb, with the camera pointed from the
//! navigation truth: port of upstream
//! `examples/OpNavScenarios/scenariosOpNav/scenario_OpNavODLimb.py`.
//!
//! The OpNav spacecraft ([`opnav::dynamics`]) on an 18 000 km, `e = 0.2`
//! orbit about Mars (`i = 10 deg`, `Omega = 25 deg`, `omega = 190 deg`,
//! `f = 0 deg`), at rest at `sigma_BN = [0, -0.3, 0]`; `relativeOD` starts on
//! the true state. For 3 minutes `prepOpNav` points the camera at Mars from
//! the Hill frame; then `OpNavODLimb` keeps that pointing and adds the orbit
//! determination: every minute `limbFinding` finds the limb,
//! `horizonOpNav` (`noiseSF = 20`) turns it into a position fix and
//! `relativeOD` filters it. 600 minutes at upstream's 0.5 s rates. The orbit
//! starts at periapsis over the night side, whose limb is too faint for the
//! edge detector: the first limbs come with the sunlit crescent, about 50
//! minutes in.
//!
//! **Images.** Upstream renders the camera images in the Vizard application.
//! Here a synthetic renderer stands in for Vizard
//! ([`opnav::planet_renderer`]): a ray-traced, Sun-lit, textureless Mars
//! ellipsoid through the camera model, so only outcomes (the filter's error
//! bounds) compare with upstream.
//!
//! Writes `examples/output/scenarios/scenario_OpNavODLimb.csv`, upstream's
//! recorders: the spacecraft state (`scRec`, at the dynamics rate, here the
//! same 0.5 s), the filter (`filtRec`, `filter_*`) and the measurement
//! (`opNavRec`, `opnav_*`); and `scenario_OpNavODLimb_limb.csv`, the limbs
//! found (`limbRec`, see `opnav::outputs::write_limb_points`).
//! `plot_scenario_op_nav_od_limb.py` draws upstream's figures.
//!
//! `--sim-time m` sets upstream's `simTime` \[min\] (default 600); upstream's
//! `test_bskTestOpNav.py` runs 10 minutes and only checks completion. The
//! tests here also check the filter's errors.

#[path = "support/opnav/mod.rs"]
mod opnav;

use std::path::PathBuf;

use basilisk_rs::messages::{OpNavFilterMsg, OpNavLimbMsg, OpNavMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use opnav::bsk_sim::{scenario_recorder, spacecraft_state_columns};
use opnav::dynamics;
use opnav::fsw::{FswMode, relative_od_q_noise};
use opnav::message_log::MessageLog;
use opnav::outputs::{
    LIMB_LOG_PRIORITY, od_filter_columns, opnav_columns, relative_nav_errors, write_limb_points,
};
use opnav::{
    Cameras, OpNavModels, RATE_S, RECORDER_PRIORITY, execute_modes, hub_init, mars_orbit,
    sim_time_from_cli,
};

const STEM: &str = "scenario_OpNavODLimb";
/// Upstream's default `simTime` \[min\].
const SIM_TIME_MIN: f64 = 600.0;
/// Upstream's `prepOpNav` phase \[min\].
const PREP_TIME_MIN: f64 = 3.0;

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Upstream `simTime` \[min\].
    sim_time_min: f64,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    spacecraft: MessageLog<SpacecraftStateMsg>,
    opnav: MessageLog<OpNavMsg>,
    filter: MessageLog<OpNavFilterMsg>,
    limbs: MessageLog<OpNavLimbMsg>,
    csv_paths: Vec<PathBuf>,
}

fn run(config: Config) -> Results {
    // `configure_initial_conditions`.
    let (r_n, v_n) = mars_orbit(18_000e3, 0.2, 10.0, 25.0, 190.0, 0.0);
    let mut models = OpNavModels::new(
        RATE_S,
        RATE_S,
        &hub_init(r_n, v_n, [0.0, -0.3, 0.0]),
        Cameras::default(),
    );
    let fsw = &mut models.fsw;
    fsw.relative_od.config.state_init =
        nalgebra::SVector::from_iterator(r_n.iter().chain(v_n.iter()).copied());
    fsw.relative_od.config.q_noise = relative_od_q_noise();
    fsw.horizon_nav.config.noise_sf = 20.0;
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    // `log_outputs`.
    let sampling = models.fsw.task_period_nanos;
    let mut spacecraft = MessageLog::<SpacecraftStateMsg>::default();
    let mut opnav = MessageLog::<OpNavMsg>::default();
    let mut filter = MessageLog::<OpNavFilterMsg>::default();
    let mut limbs = MessageLog::<OpNavLimbMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let state_out = &models.dynamics.spacecraft.state_out;
    let gateway = &models.fsw.gateway;
    let filter_out = &models.fsw.relative_od.filt_data_out_msg;
    state_out.connect_to(&mut spacecraft.input);
    gateway.opnav.connect_to(&mut opnav.input);
    filter_out.connect_to(&mut filter.input);
    models
        .fsw
        .limb_finding
        .opnav_limb_out_msg
        .connect_to(&mut limbs.input);
    state_out.connect_to(recorder.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("")));
    filter_out.connect_to(recorder.add_source::<OpNavFilterMsg>(od_filter_columns("filter")));
    gateway
        .opnav
        .connect_to(recorder.add_source::<OpNavMsg>(opnav_columns("opnav")));

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    let camera_period = models.dynamics.camera_period_nanos;
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "scRec" => &mut spacecraft, period = sampling, priority = RECORDER_PRIORITY;
        "opNavRec" => &mut opnav, period = sampling, priority = RECORDER_PRIORITY;
        "filtRec" => &mut filter, period = sampling, priority = RECORDER_PRIORITY;
        "limbRec" => &mut limbs, period = camera_period, priority = LIMB_LOG_PRIORITY;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_PRIORITY);
    }
    execute_modes(
        &mut simulation,
        &mode,
        &runner,
        &[
            (FswMode::PrepOpNav, PREP_TIME_MIN),
            (FswMode::OpNavOdLimb, config.sim_time_min),
        ],
    );
    drop(simulation);
    let limb_path = config
        .write_csv
        .then(|| write_limb_points(&format!("{STEM}_limb"), &limbs, 30));

    Results {
        spacecraft,
        opnav,
        filter,
        limbs,
        csv_paths: csv_path.into_iter().chain(limb_path).collect(),
    }
}

fn main() {
    let sim_time_min = sim_time_from_cli("scenario_op_nav_od_limb", SIM_TIME_MIN);
    let results = run(Config {
        sim_time_min,
        write_csv: true,
    });
    for path in &results.csv_paths {
        println!("wrote {}", path.display());
    }
    let (position, velocity) = relative_nav_errors(
        results.filter.last().state.as_slice(),
        results.spacecraft.last(),
    );
    println!(
        "final relativeOD errors: position {:.3} %, velocity {:.3} %; {} valid fixes",
        position * 100.0,
        velocity * 100.0,
        results.opnav.values.iter().filter(|fix| fix.valid).count()
    );
}

#[cfg(test)]
mod tests {
    //! Upstream `test_bskTestOpNav.py` runs `run(False, 10)` and checks
    //! only that it completes. The other test checks what the scenario
    //! demonstrates: the filter, started on the truth, stays on it with the
    //! limb's fixes.

    use super::*;
    use opnav::min2nano;

    #[test]
    fn upstream_test_run_completes() {
        let results = run(Config {
            sim_time_min: 10.0,
            write_csv: false,
        });
        assert_eq!(
            *results.spacecraft.times_nanos.last().unwrap(),
            min2nano(10.0)
        );
    }

    #[test]
    fn the_filter_stays_on_the_orbit() {
        let results = run(Config {
            sim_time_min: 300.0,
            write_csv: false,
        });
        // The orbit starts at periapsis over the night side: the night limb
        // is too faint for the edge detector, and the first limbs come with
        // the sunlit crescent, about 50 minutes in; then every image has one.
        let fixes: Vec<u64> = results
            .opnav
            .times_nanos
            .iter()
            .zip(&results.opnav.values)
            .filter(|(_, fix)| fix.valid)
            .map(|(time, _)| *time)
            .collect();
        let first = fixes[0];
        assert!(first < min2nano(60.0), "first fix at {first}");
        assert!(fixes.len() >= 240, "{} fixes", fixes.len());
        let limbs = results
            .limbs
            .values
            .iter()
            .filter(|limb| limb.valid)
            .count();
        // Each limb gives a fix, recorded the step after (the last one falls
        // past the end).
        assert!(
            limbs.abs_diff(fixes.len()) <= 1,
            "{limbs} limbs, {} fixes",
            fixes.len()
        );
        let (mut worst_position, mut worst_velocity) = (0.0_f64, 0.0_f64);
        for ((time, state), filter) in results
            .spacecraft
            .times_nanos
            .iter()
            .zip(&results.spacecraft.values)
            .zip(&results.filter.values)
        {
            // The filter's first output is recorded a step after the mode
            // change.
            if *time > min2nano(PREP_TIME_MIN) + 500_000_000 {
                let (position, velocity) = relative_nav_errors(filter.state.as_slice(), state);
                worst_position = worst_position.max(position);
                worst_velocity = worst_velocity.max(velocity);
            }
        }
        assert!(
            worst_position < 0.01,
            "worst position error {worst_position}"
        );
        assert!(
            worst_velocity < 0.02,
            "worst velocity error {worst_velocity}"
        );
    }
}
