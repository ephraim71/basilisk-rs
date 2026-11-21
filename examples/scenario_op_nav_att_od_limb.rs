//! Optical-navigation pointing and orbit determination from the limb: port of
//! upstream
//! `examples/OpNavScenarios/scenariosOpNav/scenario_OpNavAttODLimb.py`.
//!
//! `scenario_OpNavAttOD` with the limb instead of Hough circles: the OpNav
//! spacecraft ([`opnav::dynamics`]) on the 18 000 km, `e = 0.6` orbit about
//! Mars (`i = 10 deg`, `Omega = 25 deg`, `omega = 190 deg`, `f = 80 deg`),
//! at rest at `sigma_BN = [0, -0.3, 0]`. For 3 minutes `prepOpNav` points
//! the camera at Mars from the Hill frame; then `OpNavAttODLimb` closes the
//! loop on the images: every minute `limbFinding` (Canny edges) finds the
//! limb in the camera image (a 3-pixel blur; `cosmicRays = 1`, which
//! upstream's camera draws with probability `1 - 1/1²`, so never),
//! `horizonOpNav` (`noiseSF = 70`) turns it into a position fix, which
//! `opNavPoint` points the camera with and `relativeOD` (`noiseSF = 5`)
//! filters, started 10 km and 100 m/s away from the truth. 600 minutes in
//! total, at upstream's 0.5 s rates.
//!
//! **Images.** Upstream renders the camera images in the Vizard application.
//! Here a synthetic renderer stands in for Vizard
//! ([`opnav::planet_renderer`]): a ray-traced, Sun-lit, textureless Mars
//! ellipsoid through the camera model. The edges therefore come from a
//! cleaner limb than upstream's, and the checks compare outcomes
//! (navigation error bounds, convergence), not pixels.
//!
//! Writes `examples/output/scenarios/scenario_OpNavAttODLimb.csv`,
//! upstream's recorders at the FSW rate: the spacecraft state (`scRec`),
//! the OpNav measurement (`opNavRec`, `opnav_*`) and the filter (`filtRec`,
//! `filter_*`); and `scenario_OpNavAttODLimb_limb.csv`, the limbs found
//! (`limbRec`, see `opnav::outputs::write_limb_points`).
//! `plot_scenario_op_nav_att_od_limb.py` draws upstream's figures.
//!
//! `--sim-time m` sets upstream's `run(showPlots, simTime)` stop time in
//! minutes (default 600); upstream's `test_bskTestOpNav.py` runs `simTime =
//! 10` and only checks that the scenario completes. The tests here also
//! check that the filter converges on the images.

#[path = "support/opnav/mod.rs"]
mod opnav;

use std::path::PathBuf;

use basilisk_rs::messages::{OpNavFilterMsg, OpNavLimbMsg, OpNavMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use nalgebra::{SVector, Vector3};
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

const STEM: &str = "scenario_OpNavAttODLimb";

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
    let (r_n, v_n) = mars_orbit(18_000e3, 0.6, 10.0, 25.0, 190.0, 80.0);
    let r_error = Vector3::new(10_000.0, 10_000.0, -10_000.0);
    let v_error = Vector3::new(100.0, -10.0, 10.0);
    let mut cameras = Cameras::default();
    cameras.instrument.camera_is_on = true;
    cameras.instrument.cosmic_rays = 1.0;
    let mut models = OpNavModels::new(
        RATE_S,
        RATE_S,
        &hub_init(r_n, v_n, [0.0, -0.3, 0.0]),
        cameras,
    );
    let fsw = &mut models.fsw;
    fsw.relative_od.config.state_init = SVector::from_iterator(
        (r_n + r_error)
            .iter()
            .chain((v_n + v_error).iter())
            .copied(),
    );
    fsw.relative_od.config.q_noise = relative_od_q_noise();
    fsw.horizon_nav.config.noise_sf = 70.0;
    fsw.relative_od.config.noise_sf = 5.0;
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    // `log_outputs`: sampled at the FSW rate, in the dynamics task.
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
    gateway
        .opnav
        .connect_to(recorder.add_source::<OpNavMsg>(opnav_columns("opnav")));
    filter_out.connect_to(recorder.add_source::<OpNavFilterMsg>(od_filter_columns("filter")));

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
            (FswMode::OpNavAttOdLimb, config.sim_time_min),
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
    let sim_time_min = sim_time_from_cli("scenario_op_nav_att_od_limb", SIM_TIME_MIN);
    let results = run(Config {
        sim_time_min,
        write_csv: true,
    });
    for path in &results.csv_paths {
        println!("wrote {}", path.display());
    }
    let truth = results.spacecraft.last();
    let state = results.filter.last().state;
    let (position_error, velocity_error) = relative_nav_errors(state.as_slice(), truth);
    println!(
        "final relativeOD errors: position {:.3} %, velocity {:.3} %; {} valid OpNav fixes",
        position_error * 100.0,
        velocity_error * 100.0,
        results.opnav.values.iter().filter(|fix| fix.valid).count()
    );
}

#[cfg(test)]
mod tests {
    //! Upstream `test_bskTestOpNav.py` runs `run(False, 10)` and checks
    //! only that it completes; [`upstream_test_run_completes`] does the
    //! same. The other test checks what the scenario demonstrates: the
    //! closed loop keeps the camera on Mars and the filter converges.

    use super::*;
    use opnav::min2nano;
    use opnav::outputs::instrument_truth;

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
    fn images_point_the_camera_and_converge_the_filter() {
        let results = run(Config {
            sim_time_min: 120.0,
            write_csv: false,
        });
        let start = min2nano(60.0);
        let mut fixes = 0;
        for ((time, fix), truth) in results
            .opnav
            .times_nanos
            .iter()
            .zip(&results.opnav.values)
            .zip(&results.spacecraft.values)
        {
            if *time < start || !fix.valid {
                continue;
            }
            fixes += 1;
            let range = truth.position_m.norm();
            let fix_error = (fix.position_bn_n_m - truth.position_m).norm() / range;
            assert!(fix_error < 0.02, "fix error {fix_error} at {time}");
        }
        assert!(fixes >= 55, "{fixes} fixes");
        for ((time, truth), filter) in results
            .spacecraft
            .times_nanos
            .iter()
            .zip(&results.spacecraft.values)
            .zip(&results.filter.values)
        {
            if *time < start {
                continue;
            }
            assert!(instrument_truth(truth).off_pointing_rad < 2.0_f64.to_radians());
            let (position, velocity) = relative_nav_errors(filter.state.as_slice(), truth);
            assert!(position < 0.01, "position error {position} at {time}");
            assert!(velocity < 0.02, "velocity error {velocity} at {time}");
        }
        let limbs = results
            .limbs
            .values
            .iter()
            .filter(|limb| limb.valid)
            .count();
        assert!(limbs >= 115, "{limbs} limbs");
    }
}
