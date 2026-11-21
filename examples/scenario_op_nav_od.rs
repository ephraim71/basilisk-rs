//! Orbit determination from Hough circles, with the camera pointed from the
//! navigation truth: port of upstream
//! `examples/OpNavScenarios/scenariosOpNav/scenario_OpNavOD.py`.
//!
//! The OpNav spacecraft ([`opnav::dynamics`]) on a 10 000 km, `e = 0.7` orbit
//! about Mars (`i = 40 deg`, `Omega = 25 deg`, `omega = 190 deg`, `f = 1
//! deg`), at rest in the inertial attitude. Upstream's `run` sets
//! `filterUse = "relOD"`: `relativeOD` starts on the true state. For 10
//! minutes `prepOpNav` points the camera at Mars from the Hill frame; then
//! `OpNavOD` keeps that pointing and adds the orbit determination: every
//! minute the image goes through `houghCircles` (`noiseSF = 0.5`) and
//! `pixelLineConverter` into `relativeOD`. 600 minutes at upstream's 0.5 s
//! rates.
//!
//! As upstream, this orbit's periapsis (3000 km) lies inside Mars (3396 km):
//! the spacecraft starts below the surface and leaves it within the
//! `prepOpNav` phase, and passes below it again near 506 minutes. The
//! gravity field is evaluated there all the same; the camera sees no planet
//! from inside it. Near periapsis Mars overfills the camera's field of view,
//! so the first circles come about 30 minutes in.
//!
//! **Images.** Upstream renders the camera images in the Vizard application.
//! Here a synthetic renderer stands in for Vizard
//! ([`opnav::planet_renderer`]): a ray-traced, Sun-lit, textureless Mars
//! ellipsoid through the camera model, so only outcomes (the filter's error
//! bounds) compare with upstream.
//!
//! Writes `examples/output/scenarios/scenario_OpNavOD.csv`, upstream's
//! recorders at the FSW rate: the spacecraft state (`scRec`), the filter
//! (`filtRec`, `filter_*`), the measurement (`opNavRec`, `opnav_*`) and the
//! circles (`circlesRec`). `plot_scenario_op_nav_od.py` draws upstream's
//! figures.
//!
//! `--sim-time m` sets upstream's `simTime` \[min\] (default 600); upstream's
//! `test_bskTestOpNav.py` runs 10 minutes (the `prepOpNav` phase only) and
//! only checks completion. The tests here also check the filter's errors.

#[path = "support/opnav/mod.rs"]
mod opnav;

use std::path::PathBuf;

use basilisk_rs::messages::{OpNavCirclesMsg, OpNavFilterMsg, OpNavMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use opnav::bsk_sim::{scenario_recorder, spacecraft_state_columns};
use opnav::dynamics;
use opnav::fsw::{FswMode, relative_od_q_noise};
use opnav::message_log::MessageLog;
use opnav::outputs::{circles_columns, od_filter_columns, opnav_columns, relative_nav_errors};
use opnav::{
    Cameras, OpNavModels, RATE_S, RECORDER_PRIORITY, execute_modes, hub_init, mars_orbit,
    sim_time_from_cli,
};

const STEM: &str = "scenario_OpNavOD";
/// Upstream's default `simTime` \[min\].
const SIM_TIME_MIN: f64 = 600.0;
/// Upstream's `prepOpNav` phase \[min\].
const PREP_TIME_MIN: f64 = 10.0;

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
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    // `configure_initial_conditions` with `filterUse = "relOD"`.
    let (r_n, v_n) = mars_orbit(10_000e3, 0.7, 40.0, 25.0, 190.0, 1.0);
    let mut models = OpNavModels::new(
        RATE_S,
        RATE_S,
        &hub_init(r_n, v_n, [0.0; 3]),
        Cameras::default(),
    );
    let fsw = &mut models.fsw;
    fsw.relative_od.config.state_init =
        nalgebra::SVector::from_iterator(r_n.iter().chain(v_n.iter()).copied());
    fsw.relative_od.config.q_noise = relative_od_q_noise();
    fsw.image_processing.config.noise_sf = 0.5;
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    // `log_outputs`.
    let sampling = models.fsw.task_period_nanos;
    let mut spacecraft = MessageLog::<SpacecraftStateMsg>::default();
    let mut opnav = MessageLog::<OpNavMsg>::default();
    let mut filter = MessageLog::<OpNavFilterMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let state_out = &models.dynamics.spacecraft.state_out;
    let gateway = &models.fsw.gateway;
    let filter_out = &models.fsw.relative_od.filt_data_out_msg;
    state_out.connect_to(&mut spacecraft.input);
    gateway.opnav.connect_to(&mut opnav.input);
    filter_out.connect_to(&mut filter.input);
    state_out.connect_to(recorder.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("")));
    filter_out.connect_to(recorder.add_source::<OpNavFilterMsg>(od_filter_columns("filter")));
    gateway
        .opnav
        .connect_to(recorder.add_source::<OpNavMsg>(opnav_columns("opnav")));
    gateway
        .opnav_circles
        .connect_to(recorder.add_source::<OpNavCirclesMsg>(circles_columns()));

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "scRec" => &mut spacecraft, period = sampling, priority = RECORDER_PRIORITY;
        "opNavRec" => &mut opnav, period = sampling, priority = RECORDER_PRIORITY;
        "filtRec" => &mut filter, period = sampling, priority = RECORDER_PRIORITY;
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
            (FswMode::OpNavOd, config.sim_time_min),
        ],
    );
    drop(simulation);

    Results {
        spacecraft,
        opnav,
        filter,
        csv_path,
    }
}

fn main() {
    let sim_time_min = sim_time_from_cli("scenario_op_nav_od", SIM_TIME_MIN);
    let results = run(Config {
        sim_time_min,
        write_csv: true,
    });
    if let Some(path) = &results.csv_path {
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
    //! circles' fixes.

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
        // Close to periapsis (3000 km, inside Mars) the planet overfills the
        // 55-degree field of view; the circles start about 30 minutes in,
        // then come with every image.
        let fixes: Vec<u64> = results
            .opnav
            .times_nanos
            .iter()
            .zip(&results.opnav.values)
            .filter(|(_, fix)| fix.valid)
            .map(|(time, _)| *time)
            .collect();
        let first = fixes[0];
        assert!(first < min2nano(35.0), "first fix at {first}");
        assert!(fixes.len() >= 260, "{} fixes", fixes.len());
        let (mut worst_position, mut worst_velocity) = (0.0_f64, 0.0_f64);
        for ((time, state), filter) in results
            .spacecraft
            .times_nanos
            .iter()
            .zip(&results.spacecraft.values)
            .zip(&results.filter.values)
        {
            if *time > first {
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
