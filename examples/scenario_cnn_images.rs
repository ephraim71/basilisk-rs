//! Training images for the planet-finding network: port of upstream
//! `examples/OpNavScenarios/scenariosOpNav/CNN_ImageGen/OpNavMonteCarlo.py`
//! with the scenario it runs, `CNN_ImageGen/scenario_CNNImages.py`.
//!
//! The scenario points the OpNav spacecraft's camera at Mars from the Hill
//! frame (the `imageGen` mode: `hillPoint`, `trackingErrorCam`, the wheels,
//! and `houghCircles` on every image) for 100 minutes and saves every
//! camera image. `OpNavMonteCarlo.py` runs it [`NUMBER_OF_RUNS`] times on
//! [`basilisk_rs::monte_carlo::MonteCarlo`], seeds dispersed, with these
//! dispersions, each bound to a field of [`CnnConfig`]:
//!
//! | upstream path | dispersion |
//! |---|---|
//! | `get_DynModel().scObject.hub.r_CN_NInit`, `v_CN_NInit` | `OrbitalElementDispersion`: `a` normal (14 000 km, 2 500 km), `e` uniform on `[0.2, 0.5]`, `i` uniform on `[40, 90] deg`, `Omega = omega = 0`, `f` uniform on `[0, 359] deg`, about `mu = 4.2828371901284e13` |
//! | `get_FswModel().trackingErrorCam.sigma_R0R` | `MRPDispersionPerAxis` on `±0.051` about `[1/3, 1/3, -1/3]` |
//! | `get_DynModel().cameraMod.gaussian` | `UniformDispersion` on `[0, 5]` |
//! | `get_DynModel().cameraMod.saltPepper` | `UniformDispersion` on `[0, 2.5]` |
//! | `get_DynModel().cameraMod.cosmicRays` | `UniformDispersion` on `[0.5, 4]` |
//! | `get_DynModel().cameraMod.blurParam` | `UniformDispersion` on `[1, 6]` |
//!
//! (Upstream also names `cameraMod.darkCurrent` but never adds its
//! dispersion; neither does this port.) The pointing offset `sigma_R0R`
//! moves Mars around the image, and the noise varies from run to run.
//!
//! **Images.** Upstream renders the images in the Vizard application, the
//! renderer the network is trained for. Here a synthetic renderer stands in
//! for Vizard ([`opnav::planet_renderer`]): a ray-traced, Sun-lit,
//! textureless Mars ellipsoid through the camera model, corrupted by the
//! camera module as upstream's. These images are not Vizard's, so a network
//! trained on them would learn a different renderer.
//!
//! Each run retains `scMsg.r_BN_N`, `scMsg.sigma_BN` and `circlesMsg.valid`
//! at the FSW rate, and its images go to
//! `examples/output/montecarlo/cnn_MC_data/run<i>/<time>.png` (upstream's
//! `CNN_ImageGen/cnn_MC_data/run<i>/`). Upstream's post-processing then
//! writes the training labels `run<i>/data.csv`: `Filename, Valid, X_p,
//! Y_p, rho_p, r_BN_N_1, r_BN_N_2, r_BN_N_3`, Mars's true centre and radius
//! in pixels for the images. Upstream's loop writes the labels of its last
//! run only (the file is opened after the loop); here every run gets its
//! own. Upstream names the files `<time>.jpg` while its camera saves
//! `<time>.png`; here the names are the saved files'.
//!
//! `--runs n` and `--sim-time m` shorten the campaign (upstream: 10 runs of
//! 100 minutes). Upstream's `scenario_CNNImages.py` `__main__` runs the
//! scenario without an orbit (its `r_CN_NInit` lines are commented out, so
//! the spacecraft sits at Mars's centre); only the campaign sets one, so
//! only the campaign is ported. Upstream has no test of these scripts; the
//! tests here run a short campaign and check its images and labels.

#[path = "support/opnav/mod.rs"]
mod opnav;

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use anyhow::Context;
use basilisk_rs::messages::{OpNavCirclesMsg, SpacecraftStateMsg};
use basilisk_rs::monte_carlo::{
    CampaignOutcome, ElementDistribution, MonteCarlo, MrpDispersionPerAxis,
    OrbitalElementDispersion, RunInfo, UniformDispersion,
};
use basilisk_rs::simulation::Simulation;
use nalgebra::Vector3;
use opnav::dynamics::{self, CAMERA_FOV_DEG, CAMERA_RATE_S};
use opnav::fsw::FswMode;
use opnav::message_log::MessageLog;
use opnav::monte_carlo::archive_dir;
use opnav::outputs::instrument_truth;
use opnav::{Cameras, OpNavModels, RATE_S, RECORDER_PRIORITY, execute_modes, hub_init, sec2nano};
use serde::{Deserialize, Serialize};

/// Upstream `NUMBER_OF_RUNS`.
const NUMBER_OF_RUNS: usize = 10;
/// Upstream's stop time \[min\].
const SIM_TIME_MIN: f64 = 100.0;
/// The archive, upstream's `cnn_MC_data`.
const ARCHIVE: &str = "cnn_MC_data";

/// One run's configuration (upstream's creation function and the
/// dispersed values).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct CnnConfig {
    /// Upstream `hub.r_CN_NInit` \[m\] and `v_CN_NInit` \[m/s\]: unset by the
    /// scenario, set by the orbit dispersion.
    r_cn_n_init_m: Vector3<f64>,
    v_cn_n_init_mps: Vector3<f64>,
    /// Upstream `trackingErrorCam.sigma_R0R`.
    sigma_r0r: Vector3<f64>,
    /// Upstream `cameraMod.gaussian`, `saltPepper`, `cosmicRays`,
    /// `blurParam`.
    gaussian: f64,
    salt_pepper: f64,
    cosmic_rays: f64,
    blur_param: f64,
    /// Upstream `SimpleNavObject.RNGSeed`.
    nav_seed: u64,
    /// The stop time \[min\].
    sim_time_min: f64,
}

impl CnnConfig {
    /// Upstream's creation function `scenario_OpNav()`.
    fn nominal(sim_time_min: f64) -> Self {
        let camera = dynamics::instrument_camera();
        Self {
            r_cn_n_init_m: Vector3::zeros(),
            v_cn_n_init_mps: Vector3::zeros(),
            sigma_r0r: opnav::fsw::camera_pointing_sigma_r0r(),
            gaussian: camera.gaussian,
            salt_pepper: camera.salt_pepper,
            cosmic_rays: camera.cosmic_rays,
            blur_param: camera.blur_param,
            nav_seed: dynamics::simple_nav_default_seed(),
            sim_time_min,
        }
    }
}

/// What a run retains: `scMsg.r_BN_N`, `scMsg.sigma_BN`, `circlesMsg.valid`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
struct RetainedImages {
    times_nanos: Vec<u64>,
    r_bn_n_m: Vec<[f64; 3]>,
    sigma_bn: Vec<[f64; 3]>,
    circles_valid: Vec<bool>,
}

/// Upstream's execution function `run(TheScenario, runLog)`: the
/// `imageGen` mode for `sim_time_min`, saving every image in `image_dir`.
fn simulate(config: &CnnConfig, image_dir: Option<&Path>) -> anyhow::Result<RetainedImages> {
    let mut cameras = Cameras::default();
    let camera = &mut cameras.instrument;
    camera.field_of_view_rad = CAMERA_FOV_DEG.to_radians();
    camera.camera_is_on = true;
    camera.gaussian = config.gaussian;
    camera.salt_pepper = config.salt_pepper;
    camera.cosmic_rays = config.cosmic_rays;
    camera.blur_param = config.blur_param;
    if let Some(dir) = image_dir {
        std::fs::create_dir_all(dir).with_context(|| format!("cannot create {}", dir.display()))?;
        camera.save_images = true;
        camera.save_dir = format!("{}/", dir.display());
    }
    // `configure_initial_conditions`: at rest, inertially aligned; the
    // pixel-line filter's state (`filterUse = "bias"`) is not flown.
    let mut models = OpNavModels::new(
        RATE_S,
        RATE_S,
        &hub_init(config.r_cn_n_init_m, config.v_cn_n_init_mps, [0.0; 3]),
        cameras,
    );
    models.dynamics.simple_nav.set_rng_seed(config.nav_seed);
    models.fsw.tracking_error_cam.config.sigma_r0r = config.sigma_r0r;
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    // `log_outputs`.
    let sampling = models.fsw.task_period_nanos;
    let mut spacecraft = MessageLog::<SpacecraftStateMsg>::default();
    let mut circles = MessageLog::<OpNavCirclesMsg>::default();
    models
        .dynamics
        .spacecraft
        .state_out
        .connect_to(&mut spacecraft.input);
    models
        .fsw
        .gateway
        .opnav_circles
        .connect_to(&mut circles.input);
    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "scMsg" => &mut spacecraft, period = sampling, priority = RECORDER_PRIORITY;
        "circlesMsg" => &mut circles, period = sampling, priority = RECORDER_PRIORITY;
    }
    execute_modes(
        &mut simulation,
        &mode,
        &runner,
        &[(FswMode::ImageGen, config.sim_time_min)],
    );
    drop(simulation);

    Ok(RetainedImages {
        times_nanos: spacecraft.times_nanos,
        r_bn_n_m: spacecraft
            .values
            .iter()
            .map(|state| state.position_m.into())
            .collect(),
        sigma_bn: spacecraft
            .values
            .iter()
            .map(|state| state.sigma_bn.into())
            .collect(),
        circles_valid: circles.values.iter().map(|circle| circle.valid).collect(),
    })
}

/// Upstream `OpNavMonteCarlo.run`'s controller.
fn campaign(runs: usize, sim_time_min: f64) -> MonteCarlo<CnnConfig> {
    let orbit = OrbitalElementDispersion {
        mu: opnav::monte_carlo::DISPERSION_MU_M3PS2,
        semi_major_axis: Some(ElementDistribution::Normal {
            mean: 14_000e3,
            std_deviation: 2_500e3,
        }),
        eccentricity: Some(ElementDistribution::Uniform {
            lower: 0.2,
            upper: 0.5,
        }),
        inclination_rad: Some(ElementDistribution::Uniform {
            lower: 40.0_f64.to_radians(),
            upper: 90.0_f64.to_radians(),
        }),
        raan_rad: None,
        arg_periapsis_rad: None,
        true_anomaly_rad: Some(ElementDistribution::Uniform {
            lower: 0.0,
            upper: 359.0_f64.to_radians(),
        }),
    };
    let third = 1.0 / 3.0;
    MonteCarlo::new(CnnConfig::nominal(sim_time_min))
        .execution_count(runs)
        .verbose(true)
        .disperse_seeds(true)
        .disperse_seed(|config: &mut CnnConfig| &mut config.nav_seed)
        .disperse_with(move |config: &mut CnnConfig, rng| {
            let (r, v) = orbit.sample_state(rng);
            config.r_cn_n_init_m = r;
            config.v_cn_n_init_mps = v;
        })
        .disperse(
            |config: &mut CnnConfig| &mut config.sigma_r0r,
            MrpDispersionPerAxis {
                bounds: [
                    (third - 0.051, third + 0.051),
                    (third - 0.051, third + 0.051),
                    (-third - 0.051, -third + 0.051),
                ],
            },
        )
        .disperse(
            |config: &mut CnnConfig| &mut config.gaussian,
            UniformDispersion::new(0.0, 5.0),
        )
        .disperse(
            |config: &mut CnnConfig| &mut config.salt_pepper,
            UniformDispersion::new(0.0, 2.5),
        )
        .disperse(
            |config: &mut CnnConfig| &mut config.cosmic_rays,
            UniformDispersion::new(0.5, 4.0),
        )
        .disperse(
            |config: &mut CnnConfig| &mut config.blur_param,
            UniformDispersion::new(1.0, 6.0),
        )
}

/// Upstream's `POST` block: the labels of a run's images, `data.csv`.
fn labels_csv(data: &RetainedImages) -> String {
    let render_rate_nanos = sec2nano(CAMERA_RATE_S);
    let mut text = "Filename,Valid,X_p,Y_p,rho_p,r_BN_N_1,r_BN_N_2,r_BN_N_3\n".to_string();
    for (index, time) in data.times_nanos.iter().enumerate() {
        let valid = data.circles_valid[index];
        if !(valid || (time.is_multiple_of(render_rate_nanos) && *time > 1)) {
            continue;
        }
        let state = SpacecraftStateMsg {
            position_m: Vector3::from(data.r_bn_n_m[index]),
            sigma_bn: Vector3::from(data.sigma_bn[index]),
            ..SpacecraftStateMsg::default()
        };
        let truth = instrument_truth(&state);
        let [x, y, z] = data.r_bn_n_m[index];
        let _ = writeln!(
            text,
            "{:.6}.png,{},{},{},{},{x},{y},{z}",
            *time as f64 * 1e-9,
            u8::from(valid),
            truth.center_px[0],
            truth.center_px[1],
            truth.radius_px,
        );
    }
    text
}

/// The run directory of run `index`.
fn run_dir(archive: &Path, index: usize) -> PathBuf {
    archive.join(format!("run{index}"))
}

/// Upstream `OpNavMonteCarlo.run`: the campaign into `archive`, then the
/// labels of every run.
fn run(
    archive: &Path,
    runs: usize,
    sim_time_min: f64,
    threads: Option<usize>,
) -> anyhow::Result<CampaignOutcome<RetainedImages>> {
    let mut controller = campaign(runs, sim_time_min)
        .archive_dir(archive)
        .show_progress(false);
    if let Some(threads) = threads {
        controller = controller.threads(threads);
    }
    let outcome = controller.execute(|config: &CnnConfig, info: &RunInfo| {
        simulate(config, Some(&run_dir(archive, info.index)))
    })?;
    for (index, data) in &outcome.retained {
        let path = run_dir(archive, *index).join("data.csv");
        std::fs::write(&path, labels_csv(data))
            .with_context(|| format!("cannot write {}", path.display()))?;
    }
    Ok(outcome)
}

fn main() -> anyhow::Result<()> {
    let mut cli = opnav::cli::Cli::from_env(
        "scenario_cnn_images [--runs n] [--sim-time minutes] [--threads n] [--all]   (upstream OpNavMonteCarlo.py: 10 runs of 100 minutes)",
    );
    let _all = cli.switch("all");
    let runs: usize = cli.value("runs", NUMBER_OF_RUNS);
    let sim_time_min: f64 = cli.value("sim-time", SIM_TIME_MIN);
    let threads: usize = cli.value("threads", 0);
    cli.finish();

    let archive = archive_dir(ARCHIVE);
    let outcome = run(
        &archive,
        runs,
        sim_time_min,
        (threads > 0).then_some(threads),
    )?;
    if !outcome.failures.is_empty() {
        anyhow::bail!("No runs should fail: failed runs {:?}", outcome.failures);
    }
    for (index, data) in &outcome.retained {
        let valid = data.circles_valid.iter().filter(|valid| **valid).count();
        println!(
            "run {index}: images and data.csv in {} ({valid} circles found)",
            run_dir(&archive, *index).display()
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    //! Upstream has no test of the image generation; these check what it
    //! produces: dispersed runs, their images and their labels.

    use super::*;

    /// A temporary archive directory, removed when dropped.
    struct TempDir(PathBuf);

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn a_short_campaign_saves_labelled_images() {
        let dir = TempDir(std::env::temp_dir().join(format!("cnn_MC_data-{}", std::process::id())));
        let outcome = run(&dir.0, 2, 10.0, Some(2)).expect("campaign runs");
        assert!(outcome.failures.is_empty());
        let archive = basilisk_rs::monte_carlo::Archive::open(&dir.0).unwrap();
        let configs: Vec<CnnConfig> = (0..2).map(|i| archive.parameters(i).unwrap()).collect();
        assert_ne!(configs[0].r_cn_n_init_m, configs[1].r_cn_n_init_m);
        assert_ne!(configs[0].sigma_r0r, configs[1].sigma_r0r);
        for config in &configs {
            assert!((1.0..=6.0).contains(&config.blur_param));
            assert!((0.5..=4.0).contains(&config.cosmic_rays));
        }
        for index in 0..2 {
            let run = run_dir(&dir.0, index);
            // An image every minute after the first, 1 to 10.
            let images = (1..=10)
                .filter(|minute| run.join(format!("{}.000000.png", minute * 60)).exists())
                .count();
            assert_eq!(images, 10, "run {index}");
            let labels = std::fs::read_to_string(run.join("data.csv")).unwrap();
            let rows: Vec<&str> = labels.lines().skip(1).collect();
            // A row at each image time (`Valid` 0, recorded before the
            // FSW), and one after each image the circles were found in.
            assert!(rows.len() >= 10, "run {index}: {rows:?}");
            assert!(rows.iter().any(|row| row.starts_with("60.000000.png,0,")));
            for row in rows {
                let fields: Vec<f64> = row.split(',').skip(1).map(|f| f.parse().unwrap()).collect();
                // The true radius of Mars in the image is positive.
                assert!(fields[3] > 0.0, "{row}");
            }
        }
    }
}
