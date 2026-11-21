//! Optical-navigation pointing and orbit determination with a neural network:
//! port of upstream
//! `examples/OpNavScenarios/scenariosOpNav/scenario_CNNAttOD.py`.
//!
//! `scenario_OpNavAttOD` with `centerRadiusCNN` instead of Hough circles:
//! the OpNav spacecraft ([`opnav::dynamics`]) on the 18 000 km, `e = 0.6`
//! orbit about Mars (`i = 10 deg`, `Omega = 25 deg`, `omega = 190 deg`,
//! `f = 80 deg`), at rest in the inertial attitude. For 5 minutes
//! `prepOpNav` points the camera at Mars from the Hill frame; then
//! `CNNAttOD` closes the loop on the images: every minute the pre-trained
//! network (`CAD.onnx`, `pixelNoise = [5, 5, 5]`) estimates Mars's centre
//! and apparent radius, `pixelLineConverter` turns them into a position
//! fix, which `opNavPoint` points the camera with and `relativeOD`
//! (`noiseSF = 5`) filters, started 10 km and 100 m/s away from the truth.
//! 600 minutes in total, at upstream's 0.5 s rates.
//!
//! **Images.** Upstream renders the camera images in the Vizard application,
//! the renderer its network was trained on. Here a synthetic renderer stands
//! in for Vizard ([`opnav::planet_renderer`]): a ray-traced, Sun-lit,
//! textureless Mars ellipsoid through the camera model. The network sees
//! images unlike its training set, so the checks are looser than the Hough
//! scenario's and compare outcomes (navigation error bounds), not pixels.
//!
//! Writes `examples/output/scenarios/scenario_CNNAttOD.csv`, upstream's
//! recorders at the FSW rate: the spacecraft state (`scRec`), the OpNav
//! measurement (`opNavRec`, `opnav_*`), the filter (`filtRec`, `filter_*`) and
//! the network's circles (`circlesRec`). `plot_scenario_cnn_att_od.py` draws
//! upstream's figures. The network is read from
//! `assets/opencv/center_radius_cnn/CAD.onnx` (`just fetch-opencv-assets`);
//! without it the run publishes no fixes and the tests are skipped.
//!
//! `--sim-time m` sets upstream's `run(showPlots, simTime)` stop time in
//! minutes (default 600); upstream's `test_bskTestOpNav.py` runs `simTime =
//! 10` and only checks that the scenario completes. The tests here also
//! check that the filter converges on the images.

#[path = "support/opnav/mod.rs"]
mod opnav;

use std::path::PathBuf;

use basilisk_rs::messages::{OpNavCirclesMsg, OpNavFilterMsg, OpNavMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use nalgebra::{SVector, Vector3};
use opnav::bsk_sim::{scenario_recorder, spacecraft_state_columns};
use opnav::dynamics;
use opnav::fsw::{FswMode, relative_od_q_noise};
use opnav::message_log::MessageLog;
use opnav::outputs::{circles_columns, od_filter_columns, opnav_columns, relative_nav_errors};
use opnav::{
    Cameras, OpNavModels, RATE_S, RECORDER_PRIORITY, execute_modes, hub_init, mars_orbit,
    sim_time_from_cli,
};

const STEM: &str = "scenario_CNNAttOD";

/// Upstream's default `simTime` \[min\].
const SIM_TIME_MIN: f64 = 600.0;
/// Upstream's `prepOpNav` phase \[min\].
const PREP_TIME_MIN: f64 = 5.0;

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
    circles: MessageLog<OpNavCirclesMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    // `configure_initial_conditions`.
    let (r_n, v_n) = mars_orbit(18_000e3, 0.6, 10.0, 25.0, 190.0, 80.0);
    let r_error = Vector3::new(10_000.0, 10_000.0, -10_000.0);
    let v_error = Vector3::new(100.0, -10.0, 10.0);
    let mut models = OpNavModels::new(
        RATE_S,
        RATE_S,
        &hub_init(r_n, v_n, [0.0; 3]),
        Cameras::default(),
    );
    let fsw = &mut models.fsw;
    fsw.relative_od.config.state_init = SVector::from_iterator(
        (r_n + r_error)
            .iter()
            .chain((v_n + v_error).iter())
            .copied(),
    );
    fsw.relative_od.config.q_noise = relative_od_q_noise();
    fsw.image_processing.config.noise_sf = 1.0;
    fsw.relative_od.config.noise_sf = 5.0;
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    // `log_outputs`: sampled at the FSW rate, in the dynamics task.
    let sampling = models.fsw.task_period_nanos;
    let mut spacecraft = MessageLog::<SpacecraftStateMsg>::default();
    let mut opnav = MessageLog::<OpNavMsg>::default();
    let mut filter = MessageLog::<OpNavFilterMsg>::default();
    let mut circles = MessageLog::<OpNavCirclesMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let state_out = &models.dynamics.spacecraft.state_out;
    let gateway = &models.fsw.gateway;
    let filter_out = &models.fsw.relative_od.filt_data_out_msg;
    state_out.connect_to(&mut spacecraft.input);
    gateway.opnav.connect_to(&mut opnav.input);
    filter_out.connect_to(&mut filter.input);
    gateway.opnav_circles.connect_to(&mut circles.input);
    state_out.connect_to(recorder.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("")));
    gateway
        .opnav
        .connect_to(recorder.add_source::<OpNavMsg>(opnav_columns("opnav")));
    filter_out.connect_to(recorder.add_source::<OpNavFilterMsg>(od_filter_columns("filter")));
    gateway
        .opnav_circles
        .connect_to(recorder.add_source::<OpNavCirclesMsg>(circles_columns()));

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "scRec" => &mut spacecraft, period = sampling, priority = RECORDER_PRIORITY;
        "opNavRec" => &mut opnav, period = sampling, priority = RECORDER_PRIORITY;
        "filtRec" => &mut filter, period = sampling, priority = RECORDER_PRIORITY;
        "circlesRec" => &mut circles, period = sampling, priority = RECORDER_PRIORITY;
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
            (FswMode::CnnAttOd, config.sim_time_min),
        ],
    );
    drop(simulation);

    Results {
        spacecraft,
        opnav,
        filter,
        circles,
        csv_path,
    }
}

fn main() {
    let sim_time_min = sim_time_from_cli("scenario_cnn_att_od", SIM_TIME_MIN);
    let results = run(Config {
        sim_time_min,
        write_csv: true,
    });
    if let Some(path) = &results.csv_path {
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
    fn the_network_points_the_camera_and_converges_the_filter() {
        let network =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(opnav::fsw::CNN_NETWORK);
        if !network.is_file() {
            eprintln!(
                "skipped: {} missing (just fetch-opencv-assets)",
                network.display()
            );
            return;
        }
        let results = run(Config {
            sim_time_min: 60.0,
            write_csv: false,
        });
        // Every image of `CNNAttOD` (55 of them) gives a circle and a fix.
        let fixes = results.opnav.values.iter().filter(|fix| fix.valid).count();
        assert!(fixes >= 50, "{fixes} fixes");
        let (mut worst_fix, mut worst_circle) = (0.0_f64, 0.0_f64);
        for (((time, truth), fix), circles) in results
            .spacecraft
            .times_nanos
            .iter()
            .zip(&results.spacecraft.values)
            .zip(&results.opnav.values)
            .zip(&results.circles.values)
        {
            if fix.valid {
                let error =
                    (fix.position_bn_n_m - truth.position_m).norm() / truth.position_m.norm();
                worst_fix = worst_fix.max(error);
            }
            if circles.valid && *time > min2nano(10.0) {
                // The circle was found in the image of the previous step.
                let expected = instrument_truth(truth);
                let center = circles.circles_centers_px[0];
                worst_circle = worst_circle
                    .max((center.x - expected.center_px[0]).abs())
                    .max((center.y - expected.center_px[1]).abs())
                    .max((circles.circles_radii_px[0] - expected.radius_px).abs());
            }
        }
        assert!(worst_fix < 0.05, "worst fix error {worst_fix}");
        assert!(worst_circle < 10.0, "worst circle error {worst_circle} px");
        let (mut worst_position, mut worst_velocity) = (0.0_f64, 0.0_f64);
        for ((time, truth), filter) in results
            .spacecraft
            .times_nanos
            .iter()
            .zip(&results.spacecraft.values)
            .zip(&results.filter.values)
        {
            if *time >= min2nano(30.0) {
                assert!(instrument_truth(truth).off_pointing_rad < 3.0_f64.to_radians());
                let (position, velocity) = relative_nav_errors(filter.state.as_slice(), truth);
                worst_position = worst_position.max(position);
                worst_velocity = worst_velocity.max(velocity);
            }
        }
        assert!(
            worst_position < 0.02,
            "worst position error {worst_position}"
        );
        assert!(
            worst_velocity < 0.05,
            "worst velocity error {worst_velocity}"
        );
    }
}
