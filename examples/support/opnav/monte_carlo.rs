//! Upstream `OpNavScenarios/scenariosOpNav/OpNavMC`: the Monte Carlo
//! campaign `MonteCarlo.py` and the two scenarios it can run,
//! `scenario_OpNavAttODMC.py` and `scenario_LimbAttOD.py`, on
//! [`basilisk_rs::monte_carlo`].
//!
//! | upstream | here |
//! |---|---|
//! | `scenario_OpNavAttODMC.scenario_OpNav`, `scenario_LimbAttOD.scenario_OpNav` (creation functions) | [`McConfig::nominal`] of a [`McScenario`] |
//! | their `run(TheScenario)` (execution functions) | [`simulate`] |
//! | `MonteCarlo.run`: `Controller`, its dispersions and retention policy | [`campaign`], [`RetainedOpNav`] |
//! | `MonteCarlo.displayPlots` (the data callback) | [`nav_percent_errors`] |
//!
//! # The campaign
//!
//! `MonteCarlo.py` runs [`NUMBER_OF_RUNS`] runs, seeds dispersed (the
//! navigation sensor's `RNGSeed`), with three dispersions, each bound to a
//! field of [`McConfig`]:
//!
//! | upstream path | dispersion |
//! |---|---|
//! | `get_FswModel().relativeOD.noiseSF` | `UniformDispersion` on `[1, 10]` |
//! | `get_DynModel().cameraMod.fieldOfView` | `UniformDispersion` on `40 deg ± 0.001 deg` |
//! | `get_DynModel().scObject.hub.r_CN_NInit`, `v_CN_NInit` | `OrbitalElementDispersion`: `a` normal (22 000 km, 3 000 km), `e` uniform on `[0.2, 0.4]`, `i` uniform on `[0, 20 deg]`, `Omega = omega = 0`, `f` uniform on `[0, 180 deg]`, about `mu = 4.2828371901284e13` |
//!
//! Each run retains upstream's message logs at the FSW rate:
//! `scMsg.r_BN_N`, `.v_BN_N`, `.sigma_BN`; `opnavMsg.r_BN_N`, `.covar_N`,
//! `.r_BN_C`, `.covar_C`, `.valid`; `filtMsg.state`, `.covar`; and, for
//! `scenario_LimbAttOD`, its `limbMsg` (see [`RetainedOpNav::limbs`]).
//!
//! # Upstream's reset of the dispersed values
//!
//! Upstream's execution functions call `configure_initial_conditions`
//! again after the controller has applied the dispersions, which sets the
//! orbit and `relativeOD.noiseSF` back to their nominal values: upstream's
//! campaign effectively disperses only the field of view (and the seed).
//! [`McConfig::reset_to_nominal`] reproduces that; by default the dispersed
//! values are flown, as the dispersion list asks, with `relativeOD`
//! starting `rError`, `vError` away from the dispersed orbit.
//!
//! # Differences from upstream
//!
//! * The images come from the synthetic renderer that stands in for Vizard
//!   (see [`super::dynamics`]); upstream streams them from Vizard with
//!   `-directComm`.
//! * `scenario_LimbAttOD`'s `limbMsg` holds up to 2000 limb points per
//!   sample; upstream retains all of them at every FSW step. Here the
//!   valid limbs are retained, each with its points, and every step's
//!   `valid` and `numLimbPoints`.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use basilisk_rs::messages::{OpNavFilterMsg, OpNavLimbMsg, OpNavMsg, SpacecraftStateMsg};
use basilisk_rs::monte_carlo::{
    CampaignOutcome, ElementDistribution, MonteCarlo, OrbitalElementDispersion, RunInfo,
    UniformDispersion,
};
use basilisk_rs::simulation::Simulation;
use nalgebra::{SVector, Vector3};
use serde::{Deserialize, Serialize};

use super::dynamics::{self, CAMERA_FOV_DEG};
use super::fsw::{FswMode, relative_od_q_noise};
use super::message_log::MessageLog;
use super::outputs::{LIMB_LOG_PRIORITY, write_limbs};
use super::table_csv::write_table;
use super::{Cameras, OpNavModels, RATE_S, RECORDER_PRIORITY, execute_modes, hub_init, mars_orbit};

/// Upstream `MonteCarlo.run`'s `NUMBER_OF_RUNS`.
pub const NUMBER_OF_RUNS: usize = 2;
/// The scenarios' `prepOpNav` phase \[min\].
pub const PREP_TIME_MIN: f64 = 3.0;
/// The scenarios' stop time \[min\].
pub const SIM_TIME_MIN: f64 = 100.0;
/// Upstream `rError` \[m\] and `vError` \[m/s\]: `relativeOD`'s initial error.
pub const R_ERROR_M: [f64; 3] = [10_000.0, 10_000.0, -10_000.0];
pub const V_ERROR_MPS: [f64; 3] = [100.0, -10.0, 10.0];
/// Upstream `dispDict["mu"]` \[m^3/s^2\].
pub const DISPERSION_MU_M3PS2: f64 = 4.282_837_190_128_4e13;

/// The scenario a campaign runs (the module `MonteCarlo.py` imports).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum McScenario {
    /// `scenario_OpNavAttODMC`: OpNav pointing and orbit determination from
    /// Hough circles.
    AttOdMc,
    /// `scenario_LimbAttOD`: the same from the limb.
    LimbAttOd,
}

impl McScenario {
    /// Upstream's file name, the stem of the outputs.
    pub fn name(self) -> &'static str {
        match self {
            Self::AttOdMc => "scenario_OpNavAttODMC",
            Self::LimbAttOd => "scenario_LimbAttOD",
        }
    }

    /// Upstream `sigma_BNInit`.
    fn sigma_bn(self) -> [f64; 3] {
        match self {
            Self::AttOdMc => [0.0; 3],
            Self::LimbAttOd => [0.0, -0.3, 0.0],
        }
    }

    /// The mode after `prepOpNav`.
    fn mode(self) -> FswMode {
        match self {
            Self::AttOdMc => FswMode::OpNavAttOd,
            Self::LimbAttOd => FswMode::OpNavAttOdLimb,
        }
    }

    /// Upstream's `relativeOD.noiseSF` after `configure_initial_conditions`
    /// (`scenario_LimbAttOD` keeps `BSK_OpNavFsw`'s 7.5).
    fn relative_od_noise_sf(self) -> f64 {
        match self {
            Self::AttOdMc => 5.0,
            Self::LimbAttOd => 7.5,
        }
    }
}

/// One run's configuration: what the campaign disperses, and the nominal
/// values upstream's creation function sets.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct McConfig {
    pub scenario: McScenario,
    /// Upstream `get_DynModel().scObject.hub.r_CN_NInit` \[m\].
    pub r_cn_n_init_m: Vector3<f64>,
    /// Upstream `get_DynModel().scObject.hub.v_CN_NInit` \[m/s\].
    pub v_cn_n_init_mps: Vector3<f64>,
    /// Upstream `get_DynModel().cameraMod.fieldOfView` \[rad\].
    pub field_of_view_rad: f64,
    /// Upstream `get_FswModel().relativeOD.noiseSF`.
    pub relative_od_noise_sf: f64,
    /// Upstream `SimpleNavObject.RNGSeed`.
    pub nav_seed: u64,
    /// Fly the nominal orbit and `noiseSF`, as upstream's second
    /// `configure_initial_conditions` does (see the module docs).
    pub reset_to_nominal: bool,
}

/// The scenario's nominal orbit (`a = 18 000 km`, `e = 0.6`, `i = 10 deg`,
/// `Omega = 25 deg`, `omega = 190 deg`, `f = 80 deg`).
pub fn nominal_orbit() -> (Vector3<f64>, Vector3<f64>) {
    mars_orbit(18_000e3, 0.6, 10.0, 25.0, 190.0, 80.0)
}

impl McConfig {
    /// Upstream's creation function: `configure_initial_conditions` on the
    /// BSK_OpNav models.
    pub fn nominal(scenario: McScenario) -> Self {
        let (r_cn_n_init_m, v_cn_n_init_mps) = nominal_orbit();
        Self {
            scenario,
            r_cn_n_init_m,
            v_cn_n_init_mps,
            field_of_view_rad: CAMERA_FOV_DEG.to_radians(),
            relative_od_noise_sf: scenario.relative_od_noise_sf(),
            nav_seed: dynamics::simple_nav_default_seed(),
            reset_to_nominal: false,
        }
    }
}

/// What a run retains (upstream's `RetentionPolicy` message logs), by
/// `"<recorder>.<field>"` (upstream's `data["messages"]` keys): one row of
/// values per FSW step.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RetainedOpNav {
    pub times_nanos: Vec<u64>,
    pub messages: BTreeMap<String, Vec<Vec<f64>>>,
    /// `scenario_LimbAttOD`'s valid limbs: the time \[ns\] and the points
    /// \[pixels\] of each.
    pub limbs: Vec<(u64, Vec<[f64; 2]>)>,
}

impl RetainedOpNav {
    /// The rows of `field`.
    pub fn field(&self, field: &str) -> &[Vec<f64>] {
        self.messages
            .get(field)
            .unwrap_or_else(|| panic!("{field} was not retained"))
    }
}

fn vector(value: &Vector3<f64>) -> Vec<f64> {
    value.iter().copied().collect()
}

fn matrix<const R: usize, const C: usize>(value: &nalgebra::SMatrix<f64, R, C>) -> Vec<f64> {
    // Row-major, as upstream's flat arrays.
    value.transpose().iter().copied().collect()
}

/// Upstream's execution function `run(TheScenario)`: 3 minutes of
/// `prepOpNav`, then the scenario's mode to `sim_time_min`; the retained
/// messages.
pub fn simulate(config: &McConfig, sim_time_min: f64) -> RetainedOpNav {
    let scenario = config.scenario;
    let (r_n, v_n) = if config.reset_to_nominal {
        nominal_orbit()
    } else {
        (config.r_cn_n_init_m, config.v_cn_n_init_mps)
    };
    let noise_sf = if config.reset_to_nominal {
        scenario.relative_od_noise_sf()
    } else {
        config.relative_od_noise_sf
    };
    let mut cameras = Cameras::default();
    cameras.instrument.field_of_view_rad = config.field_of_view_rad;
    let mut models = OpNavModels::new(
        RATE_S,
        RATE_S,
        &hub_init(r_n, v_n, scenario.sigma_bn()),
        cameras,
    );
    models.dynamics.simple_nav.set_rng_seed(config.nav_seed);
    let fsw = &mut models.fsw;
    let r_init = r_n + Vector3::from(R_ERROR_M);
    let v_init = v_n + Vector3::from(V_ERROR_MPS);
    fsw.relative_od.config.state_init =
        SVector::from_iterator(r_init.iter().chain(v_init.iter()).copied());
    fsw.relative_od.config.q_noise = relative_od_q_noise();
    fsw.relative_od.config.noise_sf = noise_sf;
    match scenario {
        McScenario::AttOdMc => fsw.image_processing.config.noise_sf = 1.0,
        McScenario::LimbAttOd => fsw.horizon_nav.config.noise_sf = 20.0,
    }
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    // `log_outputs`.
    let sampling = models.fsw.task_period_nanos;
    let camera_period = models.dynamics.camera_period_nanos;
    let mut spacecraft = MessageLog::<SpacecraftStateMsg>::default();
    let mut opnav = MessageLog::<OpNavMsg>::default();
    let mut filter = MessageLog::<OpNavFilterMsg>::default();
    let mut limbs = MessageLog::<OpNavLimbMsg>::default();
    models
        .dynamics
        .spacecraft
        .state_out
        .connect_to(&mut spacecraft.input);
    models.fsw.gateway.opnav.connect_to(&mut opnav.input);
    models
        .fsw
        .relative_od
        .filt_data_out_msg
        .connect_to(&mut filter.input);
    models
        .fsw
        .limb_finding
        .opnav_limb_out_msg
        .connect_to(&mut limbs.input);

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "scMsg" => &mut spacecraft, period = sampling, priority = RECORDER_PRIORITY;
        "opnavMsg" => &mut opnav, period = sampling, priority = RECORDER_PRIORITY;
        "filtMsg" => &mut filter, period = sampling, priority = RECORDER_PRIORITY;
    }
    if scenario == McScenario::LimbAttOd {
        simulation.add_module("limbMsg", &mut limbs, camera_period, LIMB_LOG_PRIORITY);
    }
    execute_modes(
        &mut simulation,
        &mode,
        &runner,
        &[
            (FswMode::PrepOpNav, PREP_TIME_MIN),
            (scenario.mode(), sim_time_min),
        ],
    );
    drop(simulation);

    let mut messages = BTreeMap::new();
    let mut retain = |name: &str, rows: Vec<Vec<f64>>| {
        messages.insert(name.to_string(), rows);
    };
    retain(
        "scMsg.r_BN_N",
        spacecraft
            .values
            .iter()
            .map(|m| vector(&m.position_m))
            .collect(),
    );
    retain(
        "scMsg.v_BN_N",
        spacecraft
            .values
            .iter()
            .map(|m| vector(&m.velocity_mps))
            .collect(),
    );
    retain(
        "scMsg.sigma_BN",
        spacecraft
            .values
            .iter()
            .map(|m| vector(&m.sigma_bn))
            .collect(),
    );
    retain(
        "opnavMsg.r_BN_N",
        opnav
            .values
            .iter()
            .map(|m| vector(&m.position_bn_n_m))
            .collect(),
    );
    retain(
        "opnavMsg.covar_N",
        opnav.values.iter().map(|m| matrix(&m.covar_n_m2)).collect(),
    );
    retain(
        "opnavMsg.r_BN_C",
        opnav
            .values
            .iter()
            .map(|m| vector(&m.position_bn_c_m))
            .collect(),
    );
    retain(
        "opnavMsg.covar_C",
        opnav.values.iter().map(|m| matrix(&m.covar_c_m2)).collect(),
    );
    retain(
        "opnavMsg.valid",
        opnav
            .values
            .iter()
            .map(|m| vec![f64::from(u8::from(m.valid))])
            .collect(),
    );
    retain(
        "filtMsg.state",
        filter.values.iter().map(|m| matrix(&m.state)).collect(),
    );
    retain(
        "filtMsg.covar",
        filter.values.iter().map(|m| matrix(&m.covar)).collect(),
    );
    let limbs = limbs
        .times_nanos
        .iter()
        .zip(&limbs.values)
        .filter(|(_, limb)| limb.valid)
        .map(|(time, limb)| {
            let points = limb
                .active_limb_points()
                .iter()
                .map(|point| [point.x, point.y])
                .collect();
            (*time, points)
        })
        .collect();
    RetainedOpNav {
        times_nanos: spacecraft.times_nanos,
        messages,
        limbs,
    }
}

/// Upstream `MonteCarlo.run`'s controller over `scenario` (see the module
/// docs), `runs` runs.
pub fn campaign(scenario: McScenario, runs: usize, reset_to_nominal: bool) -> MonteCarlo<McConfig> {
    let mut nominal = McConfig::nominal(scenario);
    nominal.reset_to_nominal = reset_to_nominal;
    let orbit = OrbitalElementDispersion {
        mu: DISPERSION_MU_M3PS2,
        semi_major_axis: Some(ElementDistribution::Normal {
            mean: 22_000e3,
            std_deviation: 3_000e3,
        }),
        eccentricity: Some(ElementDistribution::Uniform {
            lower: 0.2,
            upper: 0.4,
        }),
        inclination_rad: Some(ElementDistribution::Uniform {
            lower: 0.0,
            upper: 20.0_f64.to_radians(),
        }),
        raan_rad: None,
        arg_periapsis_rad: None,
        true_anomaly_rad: Some(ElementDistribution::Uniform {
            lower: 0.0,
            upper: 180.0_f64.to_radians(),
        }),
    };
    MonteCarlo::new(nominal)
        .execution_count(runs)
        .verbose(true)
        .disperse_seeds(true)
        .disperse_seed(|config: &mut McConfig| &mut config.nav_seed)
        .disperse(
            |config: &mut McConfig| &mut config.relative_od_noise_sf,
            UniformDispersion::new(1.0, 10.0),
        )
        .disperse(
            |config: &mut McConfig| &mut config.field_of_view_rad,
            UniformDispersion::new(
                40.0_f64.to_radians() - 0.001_f64.to_radians(),
                40.0_f64.to_radians() + 0.001_f64.to_radians(),
            ),
        )
        .disperse_with(move |config: &mut McConfig, rng| {
            let (r, v) = orbit.sample_state(rng);
            config.r_cn_n_init_m = r;
            config.v_cn_n_init_mps = v;
        })
}

/// One row of upstream's `displayPlots`: at a valid OpNav sample, the
/// filter's position and velocity errors and their 3-sigma bounds, in
/// percent of the true position and velocity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavPercentErrors {
    pub time_nanos: u64,
    pub position: f64,
    pub velocity: f64,
    pub position_covar: f64,
    pub velocity_covar: f64,
}

/// Upstream `displayPlots`' `diffPos`, `diffVel`, `covarPos`, `covarVel`.
pub fn nav_percent_errors(data: &RetainedOpNav) -> Vec<NavPercentErrors> {
    let position = data.field("scMsg.r_BN_N");
    let velocity = data.field("scMsg.v_BN_N");
    let states = data.field("filtMsg.state");
    let covar = data.field("filtMsg.covar");
    let valid = data.field("opnavMsg.valid");
    let norm = |values: &[f64]| values.iter().map(|value| value * value).sum::<f64>().sqrt();
    let difference =
        |a: &[f64], b: &[f64]| norm(&a.iter().zip(b).map(|(a, b)| a - b).collect::<Vec<_>>());
    (0..data.times_nanos.len())
        .filter(|&index| (valid[index][0] - 1.0).abs() < 0.01)
        .map(|index| {
            let sigma = |axis: usize| covar[index][axis * 7].sqrt();
            let position_sigma = norm(&[sigma(0), sigma(1), sigma(2)]);
            let velocity_sigma = norm(&[sigma(3), sigma(4), sigma(5)]);
            let r = norm(&position[index]);
            let v = norm(&velocity[index]);
            NavPercentErrors {
                time_nanos: data.times_nanos[index],
                position: difference(&states[index][0..3], &position[index]) / r * 100.0,
                velocity: difference(&states[index][3..6], &velocity[index]) / v * 100.0,
                position_covar: 3.0 * position_sigma / r * 100.0,
                velocity_covar: 3.0 * velocity_sigma / v * 100.0,
            }
        })
        .collect()
}

/// `examples/output/montecarlo/<name>`, the stand-in for upstream's
/// `OpNavMC/MC_data`.
pub fn archive_dir(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("examples/output/montecarlo")
        .join(name)
}

/// How a campaign runs.
#[derive(Clone, Copy, Debug)]
pub struct CampaignOptions {
    /// Upstream `NUMBER_OF_RUNS`.
    pub runs: usize,
    /// Upstream `PROCESSES`; `None` uses rayon's default.
    pub threads: Option<usize>,
    /// See [`McConfig::reset_to_nominal`].
    pub reset_to_nominal: bool,
    /// The scenario's stop time \[min\].
    pub sim_time_min: f64,
}

impl Default for CampaignOptions {
    fn default() -> Self {
        Self {
            runs: NUMBER_OF_RUNS,
            threads: None,
            reset_to_nominal: false,
            sim_time_min: SIM_TIME_MIN,
        }
    }
}

/// Upstream `MonteCarlo.run(show_plots)` with `scenario`: executes the
/// campaign into `archive`.
pub fn run_campaign(
    scenario: McScenario,
    archive: &Path,
    options: CampaignOptions,
) -> anyhow::Result<CampaignOutcome<RetainedOpNav>> {
    let mut controller = campaign(scenario, options.runs, options.reset_to_nominal)
        .archive_dir(archive)
        .show_progress(false);
    if let Some(threads) = options.threads {
        controller = controller.threads(threads);
    }
    let sim_time_min = options.sim_time_min;
    controller.execute(move |config: &McConfig, _info: &RunInfo| Ok(simulate(config, sim_time_min)))
}

/// Upstream's data callback `displayPlots`: a run's percent errors as
/// `examples/output/scenarios/<stem>.csv`.
pub fn write_percent_errors(stem: &str, data: &RetainedOpNav) -> PathBuf {
    let headers = [
        "time_ns",
        "diffPos_percent",
        "diffVel_percent",
        "covarPos_percent",
        "covarVel_percent",
    ]
    .map(String::from)
    .to_vec();
    let rows: Vec<Vec<f64>> = nav_percent_errors(data)
        .iter()
        .map(|row| {
            vec![
                row.time_nanos as f64,
                row.position,
                row.velocity,
                row.position_covar,
                row.velocity_covar,
            ]
        })
        .collect();
    write_table(stem, &headers, &rows)
}

/// A run's retained messages as `examples/output/scenarios/<stem>.csv`, one
/// column per component (`scMsg.r_BN_N_0`, ...), and its limbs, if any, as
/// `<stem>_limb.csv` (see [`super::outputs::write_limb_points`]).
pub fn write_retained(stem: &str, data: &RetainedOpNav) -> Vec<PathBuf> {
    let mut headers = vec!["time_ns".to_string()];
    for (field, rows) in &data.messages {
        let width = rows.first().map_or(0, Vec::len);
        headers.extend((0..width).map(|index| format!("{field}_{index}")));
    }
    let rows: Vec<Vec<f64>> = (0..data.times_nanos.len())
        .map(|sample| {
            let mut row = vec![data.times_nanos[sample] as f64];
            for rows in data.messages.values() {
                row.extend(&rows[sample]);
            }
            row
        })
        .collect();
    let mut paths = vec![write_table(stem, &headers, &rows)];
    if !data.limbs.is_empty() {
        paths.push(write_limbs(&format!("{stem}_limb"), &data.limbs, 30));
    }
    paths
}

/// The command line of the OpNav Monte Carlo examples: the campaign of
/// `MonteCarlo.py` (default), or the scenario's own `__main__` (`--single`).
pub fn main_for(scenario: McScenario, example: &str) -> anyhow::Result<()> {
    let mut cli = super::cli::Cli::from_env(&format!(
        "{example} [--single] [--runs n] [--threads n] [--upstream-reset] [--all]\n  \
         default: MonteCarlo.py's campaign over {name} ({NUMBER_OF_RUNS} runs);\n  \
         --single: {name}'s __main__, one nominal run;\n  \
         --upstream-reset: fly the nominal orbit and noiseSF, as upstream's second configure_initial_conditions does",
        name = scenario.name()
    ));
    let _all = cli.switch("all");
    let single = cli.switch("single");
    let runs: usize = cli.value("runs", NUMBER_OF_RUNS);
    let threads: usize = cli.value("threads", 0);
    let reset_to_nominal = cli.switch("upstream-reset");
    cli.finish();

    if single {
        let data = simulate(&McConfig::nominal(scenario), SIM_TIME_MIN);
        for path in write_retained(scenario.name(), &data) {
            println!("wrote {}", path.display());
        }
        if let Some(last) = nav_percent_errors(&data).last() {
            println!(
                "final errors: position {:.3} %, velocity {:.3} %",
                last.position, last.velocity
            );
        }
        return Ok(());
    }

    let archive = archive_dir(&format!("{}_MC_data", scenario.name()));
    let outcome = run_campaign(
        scenario,
        &archive,
        CampaignOptions {
            runs,
            threads: (threads > 0).then_some(threads),
            reset_to_nominal,
            ..CampaignOptions::default()
        },
    )?;
    if !outcome.failures.is_empty() {
        anyhow::bail!("No runs should fail: failed runs {:?}", outcome.failures);
    }
    println!("{runs} runs archived in {}", archive.display());
    for (index, data) in &outcome.retained {
        let path = write_percent_errors(&format!("{}_run{index}", scenario.name()), data);
        let last = nav_percent_errors(data).last().copied();
        println!(
            "  run {index}: wrote {}; final errors {:?}",
            path.display(),
            last.map(|row| (row.position, row.velocity))
        );
    }
    Ok(())
}
