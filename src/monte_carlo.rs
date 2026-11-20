//! Monte Carlo campaigns: port of upstream Basilisk `utilities/MonteCarlo`
//! (`Controller`, `Dispersions`, `RetentionPolicy`).
//!
//! A campaign runs one simulation many times, each run from a configuration
//! drawn from statistical dispersions of a nominal one, in parallel on a
//! [`rayon`] pool. Each run's configuration and retained data can be
//! archived, reloaded, and a run repeated from its saved configuration.
//!
//! ```ignore
//! let campaign = MonteCarlo::new(nominal_config)
//!     .execution_count(4)
//!     .archive_dir("examples/output/montecarlo")
//!     .disperse(|config: &mut Config| &mut config.hub_mass_kg, UniformDispersion::new(742.5, 757.5))
//!     .disperse(|config: &mut Config| &mut config.sigma_bn, NormalVectorCartDispersion::new(mean, std))
//!     .disperse_seed(|config: &mut Config| &mut config.navigation_seed);
//! let outcome = campaign.execute(|config, _run| Ok(run_scenario(config)))?;
//! ```
//!
//! # Mapping from upstream
//!
//! | upstream (`Controller`) | here |
//! |---|---|
//! | `setSimulationFunction(create)`, `setExecutionFunction(execute)` | the closure given to [`MonteCarlo::execute`]: builds and runs the simulation from the run's configuration, returns the retained data |
//! | `addDispersion(Dispersion("path.to.var", ...))` | [`MonteCarlo::disperse`]`(\|c\| &mut c.var, dispersion)`: an explicit field of the configuration, not a name looked up at run time |
//! | co-dependent dispersions (`OrbitalElementDispersion`) | [`MonteCarlo::disperse_with`] |
//! | `setShouldDisperseSeeds(True)`, models' `RNGSeed` | [`MonteCarlo::disperse_seeds`] on the seed fields registered with [`MonteCarlo::disperse_seed`] |
//! | `addRetentionPolicy(RetentionPolicy)` with message and variable logs | the run closure's return type `R`, any serializable data |
//! | `setExecutionCount`, `setThreadCount`, `setVerbose`, `setShowProgressBar` | [`MonteCarlo::execution_count`], [`MonteCarlo::threads`], [`MonteCarlo::verbose`], [`MonteCarlo::show_progress`] |
//! | `setArchiveDir(dir)` | [`MonteCarlo::archive_dir`] |
//! | `executeSimulations()` → failed run indices | [`MonteCarlo::execute`] → [`CampaignOutcome`] (failures and the retained data) |
//! | `Controller.load(dir)`, `getParameters(i)`, `getRetainedData(i)` | [`Archive::open`], [`Archive::parameters`], [`Archive::retained_data`] |
//! | `executeCallbacks(runs)` (the policy's data callback) | [`Archive::for_each_retained`] |
//! | `reRunCases(runs)` | [`Archive::rerun_cases`] |
//! | `setICDir`, `setICRunFlag`, `runInitialConditions(runs)` | [`Archive::initial_conditions`] (or any saved configurations) and [`MonteCarlo::run_configurations`] |
//!
//! # Archive layout
//!
//! An archive directory holds `MonteCarlo.json` (the campaign: run count,
//! seed and nominal configuration; upstream pickles the controller into
//! `MonteCarlo.data`), and for run `i` `run<i>.json` (its configuration;
//! upstream's file of the run's modifications) and `run<i>.data` (its
//! retained data as JSON; upstream's gzipped pickle). Failed runs are listed
//! in `failures.txt`. As upstream, executing a campaign first empties the
//! directory.
//!
//! # Differences from upstream
//!
//! * Runs execute on threads of one process, not a pool of processes, so
//!   upstream's worker recycling (`setMaxTasksPerChild`), its data-writer
//!   process and queue (`setRetentionQueueSize`) and its pandas
//!   consolidation of retained variables (`DataWriter`, `setVarCast`) have
//!   no counterpart. A run that panics or returns an error fails; the others
//!   still run and keep their data.
//! * Run `i` draws its dispersions from a generator seeded with
//!   `seed + 10 i` (upstream seeds Python's and NumPy's generators with
//!   `10 i`; the default seed is 0). The run closure gets a separate
//!   per-run seed, [`RunInfo::seed`], for any randomness of its own.
//! * Rerunning a case returns its new retained data (upstream discards it
//!   and leaves the archived data in place), so a rerun can be compared with
//!   the original run; the archive is left unchanged.
//! * Initial-condition runs write their data to the archive directory (when
//!   set) rather than into the initial-condition directory.

pub mod dispersions;

use std::collections::BTreeMap;
use std::fs;
use std::io::{BufReader, BufWriter, Write};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use anyhow::{Context, bail};
use indicatif::{ProgressBar, ProgressStyle};
use rand::{Rng, SeedableRng};
use rayon::prelude::*;
use serde::{Deserialize, Serialize, de::DeserializeOwned};

pub use self::dispersions::{
    Dispersion, ElementDistribution, InertiaTensorDispersion, MrpDispersionPerAxis,
    NormalDispersion, NormalThrusterUnitDirectionVectorDispersion, NormalVectorAngleDispersion,
    NormalVectorCartDispersion, OrbitalElementDispersion, RunRng, UniformDispersion,
    UniformEulerAngleMrpDispersion, UniformVectorAngleDispersion, UniformVectorCartDispersion,
};

/// The campaign file of an archive (upstream `MonteCarlo.data`).
const CAMPAIGN_FILE: &str = "MonteCarlo.json";
/// The list of failed runs in an archive (upstream `failures.txt`).
const FAILURES_FILE: &str = "failures.txt";

type FieldDispersion<C> = Box<dyn Fn(&mut C, &mut RunRng) + Send + Sync>;
type SeedField<C> = Box<dyn Fn(&mut C) -> &mut u64 + Send + Sync>;

/// What a run knows about itself.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RunInfo {
    /// The run's index in the campaign.
    pub index: usize,
    /// A seed for the run's own randomness, fixed by the campaign seed and
    /// the index (and independent of the dispersion draws).
    pub seed: u64,
    /// `<archive>/run<i>`, for files the run writes next to its archived
    /// data (upstream passes this file name to an execution function that
    /// takes two arguments); `None` without an archive.
    pub archive_stem: Option<PathBuf>,
}

/// What a campaign produced: the failed runs and the data the others
/// retained.
#[derive(Debug)]
pub struct CampaignOutcome<R> {
    /// Indices of the runs that panicked or returned an error, ascending
    /// (upstream's return value of `executeSimulations`).
    pub failures: Vec<usize>,
    /// The retained data of each successful run, by index.
    pub retained: BTreeMap<usize, R>,
}

/// The archived description of a campaign (`MonteCarlo.json`).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CampaignRecord<C> {
    pub execution_count: usize,
    pub seed: u64,
    pub nominal: C,
}

/// A Monte Carlo campaign over configurations of type `C` (upstream
/// `Controller`).
pub struct MonteCarlo<C> {
    nominal: C,
    dispersions: Vec<FieldDispersion<C>>,
    seed_fields: Vec<SeedField<C>>,
    disperse_seeds: bool,
    execution_count: usize,
    threads: Option<usize>,
    archive_dir: Option<PathBuf>,
    seed: u64,
    verbose: bool,
    show_progress: bool,
}

impl<C> MonteCarlo<C>
where
    C: Clone + Send + Sync + Serialize + DeserializeOwned,
{
    /// A campaign of zero runs around `nominal`, on rayon's default number
    /// of threads, without an archive.
    pub fn new(nominal: C) -> Self {
        Self {
            nominal,
            dispersions: Vec::new(),
            seed_fields: Vec::new(),
            disperse_seeds: false,
            execution_count: 0,
            threads: None,
            archive_dir: None,
            seed: 0,
            verbose: false,
            show_progress: false,
        }
    }

    /// The nominal configuration.
    pub fn nominal(&self) -> &C {
        &self.nominal
    }

    /// Upstream `setExecutionCount`: the number of runs.
    pub fn execution_count(mut self, count: usize) -> Self {
        self.execution_count = count;
        self
    }

    /// Upstream `setThreadCount`: runs execute on a pool of `threads`
    /// threads; one runs them in the calling thread.
    pub fn threads(mut self, threads: usize) -> Self {
        self.threads = Some(threads.max(1));
        self
    }

    /// Upstream `setArchiveDir`: archive each run's configuration and
    /// retained data in `dir` (emptied when the campaign executes).
    pub fn archive_dir(mut self, dir: impl Into<PathBuf>) -> Self {
        self.archive_dir = Some(dir.into());
        self
    }

    /// The campaign seed; run `i` draws its dispersions from `seed + 10 i`.
    pub fn seed(mut self, seed: u64) -> Self {
        self.seed = seed;
        self
    }

    /// Upstream `setVerbose`: report each run on standard output.
    pub fn verbose(mut self, verbose: bool) -> Self {
        self.verbose = verbose;
        self
    }

    /// Upstream `setShowProgressBar`: a progress bar over the runs.
    pub fn show_progress(mut self, show: bool) -> Self {
        self.show_progress = show;
        self
    }

    /// Upstream `addDispersion`: each run sets `field` of its configuration
    /// to a draw of `dispersion` around the field's nominal value.
    /// Dispersions are drawn in the order they are added, after the seeds.
    pub fn disperse<T, D>(
        mut self,
        field: impl Fn(&mut C) -> &mut T + Send + Sync + 'static,
        dispersion: D,
    ) -> Self
    where
        D: Dispersion<Value = T> + 'static,
    {
        self.dispersions.push(Box::new(move |config, rng| {
            let value = field(config);
            *value = dispersion.sample(value, rng);
        }));
        self
    }

    /// A dispersion that sets several fields from one draw, or any other
    /// change of the configuration drawn from the run's generator (upstream's
    /// co-dependent dispersions, `OrbitalElementDispersion`).
    pub fn disperse_with(
        mut self,
        dispersion: impl Fn(&mut C, &mut RunRng) + Send + Sync + 'static,
    ) -> Self {
        self.dispersions.push(Box::new(dispersion));
        self
    }

    /// Registers a module's random seed (upstream `RNGSeed`), which
    /// [`Self::disperse_seeds`] disperses.
    pub fn disperse_seed(
        mut self,
        field: impl Fn(&mut C) -> &mut u64 + Send + Sync + 'static,
    ) -> Self {
        self.seed_fields.push(Box::new(field));
        self
    }

    /// Upstream `setShouldDisperseSeeds`: each run draws every registered
    /// seed uniformly from `[0, 2^32)` before the other dispersions.
    pub fn disperse_seeds(mut self, disperse: bool) -> Self {
        self.disperse_seeds = disperse;
        self
    }

    /// The dispersed configuration of run `index` (upstream's
    /// modifications applied to the created simulation). The same for every
    /// call.
    pub fn run_configuration(&self, index: usize) -> C {
        let mut rng = RunRng::seed_from_u64(dispersion_seed(self.seed, index));
        let mut config = self.nominal.clone();
        if self.disperse_seeds {
            for field in &self.seed_fields {
                *field(&mut config) = rng.gen_range(0..1_u64 << 32);
            }
        }
        for dispersion in &self.dispersions {
            dispersion(&mut config, &mut rng);
        }
        config
    }

    /// Upstream `executeSimulations`: runs `0..execution_count`, each from
    /// its dispersed configuration, and archives them when an archive
    /// directory is set.
    ///
    /// `run(config, info)` builds and executes the simulation and returns
    /// the data to retain. Returns an error only when the archive cannot be
    /// written; a failing run is reported in [`CampaignOutcome::failures`].
    pub fn execute<R, F>(&self, run: F) -> anyhow::Result<CampaignOutcome<R>>
    where
        R: Serialize + Send,
        F: Fn(&C, &RunInfo) -> anyhow::Result<R> + Sync,
    {
        if self.verbose {
            println!(
                "Beginning simulation with {} runs on {} threads",
                self.execution_count,
                self.threads.unwrap_or_else(rayon::current_num_threads)
            );
        }
        if let Some(dir) = &self.archive_dir {
            if dir.exists() {
                fs::remove_dir_all(dir)
                    .with_context(|| format!("cannot empty archive {}", dir.display()))?;
            }
            fs::create_dir_all(dir)
                .with_context(|| format!("cannot create archive {}", dir.display()))?;
            write_json(
                &dir.join(CAMPAIGN_FILE),
                &CampaignRecord {
                    execution_count: self.execution_count,
                    seed: self.seed,
                    nominal: &self.nominal,
                },
            )?;
        }
        let configurations = (0..self.execution_count)
            .map(|index| (index, self.run_configuration(index)))
            .collect();
        self.run_configurations(configurations, run)
    }

    /// Runs the given `(index, configuration)` pairs without dispersing
    /// them, in parallel, archiving each in the archive directory when one is
    /// set (which is not emptied). With [`Archive::initial_conditions`] this
    /// is upstream `runInitialConditions`.
    pub fn run_configurations<R, F>(
        &self,
        configurations: Vec<(usize, C)>,
        run: F,
    ) -> anyhow::Result<CampaignOutcome<R>>
    where
        R: Serialize + Send,
        F: Fn(&C, &RunInfo) -> anyhow::Result<R> + Sync,
    {
        if let Some(dir) = &self.archive_dir {
            fs::create_dir_all(dir)
                .with_context(|| format!("cannot create archive {}", dir.display()))?;
        }
        let progress = self.show_progress.then(|| {
            let bar = ProgressBar::new(configurations.len() as u64);
            bar.set_style(
                ProgressStyle::with_template(
                    "[{elapsed_precise}] {bar:40.cyan/blue} {pos}/{len} runs",
                )
                .expect("valid progress template")
                .progress_chars("##-"),
            );
            bar
        });
        let archive_error = Mutex::new(None);
        let execute_one = |(index, config): &(usize, C)| -> Option<(usize, Option<R>)> {
            let info = RunInfo {
                index: *index,
                seed: run_seed(self.seed, *index),
                archive_stem: self
                    .archive_dir
                    .as_ref()
                    .map(|dir| dir.join(format!("run{index}"))),
            };
            if let Some(stem) = &info.archive_stem
                && let Err(error) = write_json(&stem.with_extension("json"), config)
            {
                archive_error
                    .lock()
                    .expect("unpoisoned")
                    .get_or_insert(error);
                return None;
            }
            if self.verbose {
                println!("Executing run {index}");
            }
            let outcome = catch_unwind(AssertUnwindSafe(|| run(config, &info)));
            let retained = match outcome {
                Ok(Ok(data)) => Some(data),
                Ok(Err(error)) => {
                    eprintln!("Error in Monte Carlo run {index}: {error:#}");
                    None
                }
                Err(_) => {
                    eprintln!("Monte Carlo run {index} panicked");
                    None
                }
            };
            if let (Some(data), Some(stem)) = (&retained, &info.archive_stem)
                && let Err(error) = write_json(&stem.with_extension("data"), data)
            {
                archive_error
                    .lock()
                    .expect("unpoisoned")
                    .get_or_insert(error);
            }
            if let Some(bar) = &progress {
                bar.inc(1);
            }
            if self.verbose && retained.is_some() {
                println!("Run {index} finished successfully");
            }
            Some((*index, retained))
        };
        let results: Vec<Option<(usize, Option<R>)>> = match self.threads {
            Some(1) => configurations.iter().map(execute_one).collect(),
            Some(threads) => rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .context("cannot build the Monte Carlo thread pool")?
                .install(|| configurations.par_iter().map(execute_one).collect()),
            None => configurations.par_iter().map(execute_one).collect(),
        };
        if let Some(bar) = progress {
            bar.finish();
        }
        if let Some(error) = archive_error.into_inner().expect("unpoisoned") {
            return Err(error);
        }

        let mut outcome = CampaignOutcome {
            failures: Vec::new(),
            retained: BTreeMap::new(),
        };
        for (index, retained) in results.into_iter().flatten() {
            match retained {
                Some(data) => {
                    outcome.retained.insert(index, data);
                }
                None => outcome.failures.push(index),
            }
        }
        outcome.failures.sort_unstable();
        if !outcome.failures.is_empty() {
            if self.verbose {
                println!("Failed {:?}", outcome.failures);
            }
            if let Some(dir) = &self.archive_dir {
                fs::write(dir.join(FAILURES_FILE), format!("{:?}", outcome.failures))
                    .context("cannot write failures.txt")?;
            }
        }
        Ok(outcome)
    }
}

/// The seed of run `index`'s dispersion generator.
fn dispersion_seed(campaign_seed: u64, index: usize) -> u64 {
    campaign_seed.wrapping_add(10 * index as u64)
}

/// [`RunInfo::seed`]: a SplitMix64 step of the dispersion seed, so the run's
/// own generator does not repeat the dispersion draws.
fn run_seed(campaign_seed: u64, index: usize) -> u64 {
    let mut z = dispersion_seed(campaign_seed, index).wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn write_json(path: &Path, value: &impl Serialize) -> anyhow::Result<()> {
    let file =
        fs::File::create(path).with_context(|| format!("cannot create {}", path.display()))?;
    let mut writer = BufWriter::new(file);
    serde_json::to_writer(&mut writer, value)
        .with_context(|| format!("cannot write {}", path.display()))?;
    writer
        .flush()
        .with_context(|| format!("cannot write {}", path.display()))
}

fn read_json<T: DeserializeOwned>(path: &Path) -> anyhow::Result<T> {
    let file = fs::File::open(path).with_context(|| format!("cannot open {}", path.display()))?;
    serde_json::from_reader(BufReader::new(file))
        .with_context(|| format!("cannot read {}", path.display()))
}

/// A campaign archive on disk (upstream `Controller.load(dir)`).
#[derive(Clone, Debug)]
pub struct Archive {
    dir: PathBuf,
}

impl Archive {
    /// Opens the archive in `dir`; it need not hold a campaign file (an
    /// initial-condition directory holds only `run<i>.json` files).
    pub fn open(dir: impl Into<PathBuf>) -> anyhow::Result<Self> {
        let dir = dir.into();
        if !dir.is_dir() {
            bail!("no Monte Carlo archive at {}", dir.display());
        }
        Ok(Self { dir })
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// The archived campaign (`MonteCarlo.json`).
    pub fn campaign<C: DeserializeOwned>(&self) -> anyhow::Result<CampaignRecord<C>> {
        read_json(&self.dir.join(CAMPAIGN_FILE))
    }

    /// Upstream `getParameters(i)`: the configuration run `index` used.
    pub fn parameters<C: DeserializeOwned>(&self, index: usize) -> anyhow::Result<C> {
        read_json(&self.dir.join(format!("run{index}.json")))
    }

    /// Upstream `getRetainedData(i)`: the data run `index` retained.
    pub fn retained_data<R: DeserializeOwned>(&self, index: usize) -> anyhow::Result<R> {
        read_json(&self.dir.join(format!("run{index}.data")))
    }

    /// The failed runs of the campaign (`failures.txt`), empty when none.
    pub fn failures(&self) -> anyhow::Result<Vec<usize>> {
        let path = self.dir.join(FAILURES_FILE);
        if !path.exists() {
            return Ok(Vec::new());
        }
        let text = fs::read_to_string(&path)?;
        Ok(serde_json::from_str(&text)?)
    }

    /// Upstream `executeCallbacks(runs)`: calls `callback(index, data)` with
    /// the retained data of each run in turn.
    pub fn for_each_retained<R: DeserializeOwned>(
        &self,
        runs: impl IntoIterator<Item = usize>,
        mut callback: impl FnMut(usize, R),
    ) -> anyhow::Result<()> {
        for index in runs {
            callback(index, self.retained_data(index)?);
        }
        Ok(())
    }

    /// The saved configurations of `runs` (upstream's initial-condition
    /// directory read by `runInitialConditions`).
    pub fn initial_conditions<C: DeserializeOwned>(
        &self,
        runs: impl IntoIterator<Item = usize>,
    ) -> anyhow::Result<Vec<(usize, C)>> {
        runs.into_iter()
            .map(|index| Ok((index, self.parameters(index)?)))
            .collect()
    }

    /// Upstream `reRunCases(runs)`: runs each case again, one after the
    /// other, from its archived configuration, and returns the new data. The
    /// archive is not changed.
    pub fn rerun_cases<C, R, F>(
        &self,
        runs: impl IntoIterator<Item = usize>,
        run: F,
    ) -> anyhow::Result<CampaignOutcome<R>>
    where
        C: DeserializeOwned,
        F: Fn(&C, &RunInfo) -> anyhow::Result<R>,
    {
        let seed = self
            .campaign::<serde_json::Value>()
            .map(|campaign| campaign.seed)
            .unwrap_or(0);
        let mut outcome = CampaignOutcome {
            failures: Vec::new(),
            retained: BTreeMap::new(),
        };
        for index in runs {
            let config: C = self.parameters(index)?;
            let info = RunInfo {
                index,
                seed: run_seed(seed, index),
                archive_stem: None,
            };
            match catch_unwind(AssertUnwindSafe(|| run(&config, &info))) {
                Ok(Ok(data)) => {
                    outcome.retained.insert(index, data);
                }
                Ok(Err(error)) => {
                    eprintln!("Error re-executing run {index}: {error:#}");
                    outcome.failures.push(index);
                }
                Err(_) => {
                    eprintln!("Error re-executing run {index}: panicked");
                    outcome.failures.push(index);
                }
            }
        }
        Ok(outcome)
    }
}

#[cfg(test)]
mod tests {
    //! Ports the behaviour `utilities/MonteCarlo/_UnitTests/test_controller.py`
    //! checks that has a counterpart here: failures reported by index with
    //! the other runs' data kept, empty campaigns, initial-condition runs
    //! and dispersions applied to the run's parameters. The path-string
    //! parsing, worker-process and data-writer tests have no counterpart.

    use std::sync::atomic::{AtomicUsize, Ordering};

    use nalgebra::Vector3;
    use rand::Rng;

    use super::*;

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    struct Config {
        mass_kg: f64,
        offset_m: Vector3<f64>,
        model_seed: u64,
        other_seed: u64,
    }

    fn nominal() -> Config {
        Config {
            mass_kg: 1.0,
            offset_m: Vector3::zeros(),
            model_seed: 0,
            other_seed: 0,
        }
    }

    /// A temporary directory removed when dropped.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new(name: &str) -> Self {
            let dir = std::env::temp_dir().join(format!(
                "basilisk-rs-monte-carlo-{name}-{}",
                std::process::id()
            ));
            let _ = fs::remove_dir_all(&dir);
            Self(dir)
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    fn campaign() -> MonteCarlo<Config> {
        MonteCarlo::new(nominal())
            .disperse(
                |config: &mut Config| &mut config.mass_kg,
                UniformDispersion::new(24.0, 26.0),
            )
            .disperse(
                |config: &mut Config| &mut config.offset_m,
                NormalVectorCartDispersion::new(
                    Vector3::zeros(),
                    Vector3::new(0.01 / 3.0, 0.01 / 3.0, 0.02 / 3.0),
                ),
            )
            .disperse_seed(|config: &mut Config| &mut config.model_seed)
            .disperse_seeds(true)
    }

    /// The samples a run "retains": 32 draws of its own generator, as the
    /// upstream test's execution function retains `np.random` draws.
    fn samples(info: &RunInfo) -> Vec<f64> {
        let mut rng = RunRng::seed_from_u64(info.seed);
        (0..32).map(|_| rng.r#gen::<f64>()).collect()
    }

    /// Upstream `test_uniform_dispersion_randomizes_nested_sim_parameter`:
    /// the run sees the dispersed value, within bounds and different between
    /// runs; and `test_disperse_seeds_only_records_seeded_models`: only the
    /// registered seed is dispersed.
    #[test]
    fn dispersions_set_the_run_configuration() {
        let campaign = campaign().execution_count(2).threads(2);
        let outcome = campaign
            .execute(|config, _info| Ok(config.clone()))
            .expect("campaign runs");
        assert!(outcome.failures.is_empty());
        let runs: Vec<&Config> = outcome.retained.values().collect();
        assert_eq!(runs.len(), 2);
        for (index, config) in runs.iter().enumerate() {
            assert_eq!(**config, campaign.run_configuration(index));
            assert!((24.0..=26.0).contains(&config.mass_kg));
            assert_ne!(config.offset_m, Vector3::zeros());
            assert!(config.model_seed < 1 << 32);
            assert_eq!(config.other_seed, 0);
        }
        assert_ne!(runs[0].mass_kg, runs[1].mass_kg);
        assert_ne!(runs[0].model_seed, runs[1].model_seed);

        let without_seeds = campaign.disperse_seeds(false);
        assert_eq!(without_seeds.run_configuration(1).model_seed, 0);
    }

    /// Upstream `test_task_failures_report_indices_and_preserve_successes`:
    /// failed runs are reported, sorted, in the outcome and `failures.txt`,
    /// and every other run keeps its data, on one or several threads.
    #[test]
    fn failures_report_indices_and_preserve_successes() {
        for threads in [1, 2, 4] {
            let archive = TempDir::new(&format!("failures-{threads}"));
            let campaign = campaign()
                .execution_count(6)
                .threads(threads)
                .archive_dir(&archive.0);
            let outcome = campaign
                .execute(|_config, info| {
                    match info.index {
                        0 => panic!("simulated run failure"),
                        1 | 3 => bail!("simulated run error"),
                        _ => {}
                    }
                    Ok(samples(info))
                })
                .expect("archive written");
            assert_eq!(outcome.failures, vec![0, 1, 3]);
            assert_eq!(
                outcome.retained.keys().copied().collect::<Vec<_>>(),
                vec![2, 4, 5]
            );
            let archive = Archive::open(&archive.0).expect("archive exists");
            assert_eq!(archive.failures().expect("readable"), vec![0, 1, 3]);
            for (index, data) in &outcome.retained {
                let archived: Vec<f64> = archive.retained_data(*index).expect("archived data");
                assert_eq!(&archived, data);
                assert!(archive.retained_data::<Vec<f64>>(0).is_err());
            }
        }
    }

    /// The same campaign gives the same runs on any number of threads.
    #[test]
    fn runs_do_not_depend_on_the_thread_count() {
        let run = |config: &Config, info: &RunInfo| Ok((config.clone(), samples(info)));
        let serial = campaign()
            .execution_count(5)
            .threads(1)
            .execute(run)
            .unwrap();
        let parallel = campaign()
            .execution_count(5)
            .threads(3)
            .execute(run)
            .unwrap();
        assert_eq!(serial.retained, parallel.retained);
        let other_seed = campaign()
            .execution_count(5)
            .seed(1)
            .threads(3)
            .execute(run)
            .unwrap();
        assert_ne!(serial.retained[&2], other_seed.retained[&2]);
    }

    /// Upstream `test_empty_campaign_and_legacy_controller_settings`: an
    /// empty campaign runs nothing and fails nothing, and an archived
    /// campaign loads back.
    #[test]
    fn empty_campaigns_and_loaded_archives() {
        let archive = TempDir::new("empty");
        let calls = AtomicUsize::new(0);
        let outcome = campaign()
            .archive_dir(&archive.0)
            .execute(|_config, _info| {
                calls.fetch_add(1, Ordering::Relaxed);
                Ok(())
            })
            .unwrap();
        assert!(outcome.failures.is_empty() && outcome.retained.is_empty());
        assert_eq!(calls.load(Ordering::Relaxed), 0);

        let campaign = campaign()
            .execution_count(3)
            .seed(4)
            .archive_dir(&archive.0);
        campaign
            .execute(|config, _info| Ok(config.mass_kg))
            .unwrap();
        let loaded = Archive::open(&archive.0).unwrap();
        let record: CampaignRecord<Config> = loaded.campaign().unwrap();
        assert_eq!(record.execution_count, 3);
        assert_eq!(record.seed, 4);
        assert_eq!(record.nominal, nominal());
        for index in 0..3 {
            let parameters: Config = loaded.parameters(index).unwrap();
            assert_eq!(parameters, campaign.run_configuration(index));
            let mass: f64 = loaded.retained_data(index).unwrap();
            assert_eq!(mass, parameters.mass_kg);
        }
        let mut seen = Vec::new();
        loaded
            .for_each_retained(0..3, |index, mass: f64| seen.push((index, mass)))
            .unwrap();
        assert_eq!(seen.len(), 3);
    }

    /// Upstream `reRunCases`: a case run again from its archived
    /// configuration reproduces its data.
    #[test]
    fn rerun_cases_reproduce_the_archived_runs() {
        let archive = TempDir::new("rerun");
        let run = |config: &Config, info: &RunInfo| Ok((config.mass_kg, samples(info)));
        campaign()
            .execution_count(4)
            .archive_dir(&archive.0)
            .execute(run)
            .unwrap();
        let loaded = Archive::open(&archive.0).unwrap();
        let rerun = loaded.rerun_cases::<Config, _, _>([3, 2], run).unwrap();
        assert!(rerun.failures.is_empty());
        for index in [2, 3] {
            let archived: (f64, Vec<f64>) = loaded.retained_data(index).unwrap();
            assert_eq!(rerun.retained[&index], archived);
        }
        let failed = loaded
            .rerun_cases::<Config, (), _>([1], |_config, _info| bail!("no"))
            .unwrap();
        assert_eq!(failed.failures, vec![1]);
    }

    /// Upstream `test_initial_conditions_with_and_without_writer` and
    /// `test_initial_conditions_accept_numpy_indices`: saved
    /// configurations run again, with and without an archive, including an
    /// empty selection.
    #[test]
    fn initial_conditions_run_from_saved_configurations() {
        let initial = TempDir::new("initial");
        fs::create_dir_all(&initial.0).unwrap();
        let cases = [1, 2, 3, 4, 5];
        for index in cases {
            let config = Config {
                mass_kg: index as f64,
                ..nominal()
            };
            write_json(&initial.0.join(format!("run{index}.json")), &config).unwrap();
        }
        let saved = Archive::open(&initial.0).unwrap();
        for with_archive in [false, true] {
            let output = TempDir::new(&format!("initial-output-{with_archive}"));
            let mut controller = campaign().threads(2);
            if with_archive {
                controller = controller.archive_dir(&output.0);
            }
            let configurations = saved.initial_conditions::<Config>(cases).unwrap();
            let outcome = controller
                .run_configurations(configurations, |config, info| {
                    Ok((config.mass_kg, samples(info)))
                })
                .unwrap();
            assert!(outcome.failures.is_empty());
            for index in cases {
                let (mass, _) = &outcome.retained[&index];
                // Saved configurations are not dispersed again.
                assert_eq!(*mass, index as f64);
                if with_archive {
                    let archived: (f64, Vec<f64>) = Archive::open(&output.0)
                        .unwrap()
                        .retained_data(index)
                        .unwrap();
                    assert_eq!(&archived, &outcome.retained[&index]);
                }
            }
            let failing = controller
                .run_configurations(
                    saved.initial_conditions::<Config>([2, 5]).unwrap(),
                    |_, _| bail!("simulated failure"),
                )
                .map(|outcome: CampaignOutcome<()>| outcome.failures)
                .unwrap();
            assert_eq!(failing, vec![2, 5]);
            let empty = controller
                .run_configurations(Vec::new(), |_, _| Ok(()))
                .unwrap();
            assert!(empty.failures.is_empty() && empty.retained.is_empty());
        }
    }
}
