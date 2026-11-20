//! Port of upstream `examples/scenarioConstrainedDynamicsFrequencyAnalysis.py`:
//! how stiff the docking constraint has to be. A chaser and a target
//! spacecraft, each with a hinged solar panel, are held together by a
//! constraint tuned with `alpha = beta = gain`; next to them a single
//! spacecraft models the docked stack as one rigid hub carrying both panels.
//! The chaser's panel and the truth model's matching panel are kicked to
//! 7 deg/s and the run lasts 10 s at a 1 ms step (RKF45). For each gain the
//! scenario compares the panel angle with the truth model's and looks at
//! the spectrum of the attitude violation.
//!
//! | option | upstream | default |
//! |---|---|---|
//! | `--gains 1e1,1e2,...` | `gain_list` | `1e1,1e2,1e3,1e4,1e5` (upstream's test) |
//! | `--model bskSat\|MEV1\|MEV2` | `sc_model` | `MEV2` (upstream's test) |
//!
//! Each gain writes `scenarioConstrainedDynamicsFrequencyAnalysis_<model>_alpha<gain>.csv`
//! (`alpha1e1`, ...): both hub poses (`r_B1N_N_m_i`, `sigma_B1N_i`, `r_B2N_N_m_i`,
//! `sigma_B2N_i`) and the four panel angles and rates (`theta<k>_rad`,
//! `thetaDot<k>_rad_per_s`; panels 1 and 2 on the chaser and target, 3 and
//! 4 on the truth model) at every step.
//! `scenarioConstrainedDynamicsFrequencyAnalysis_<model>_summary.csv` holds,
//! per gain, upstream's printed "frequency of highest amplitude" of
//! `sigma_B2B1[:, 0]` (from the same one-sided DFT upstream plots), the
//! constraint's fictitious frequency (the spectrum's highest peak outside
//! the panels' band), the largest panel angle error and the wall-clock
//! runtime.
//! `plot_scenario_constrained_dynamics_frequency_analysis.py` draws
//! upstream's `PanelAngleErrors` and `FFT` figures.
//!
//! Upstream's `test_scenarioConstrainedDynamicsFrequencyAnalysis.py` runs the
//! five gains with `MEV2` and checks only that the run completes. `tests`
//! runs the same and checks what upstream's description of the figures
//! says: the constraint's fictitious frequency rises with the gain, and the
//! stiffer the constraint, the closer the docked pair's panel follows the
//! rigid truth model.

#[path = "support/common.rs"]
mod common;
#[path = "support/constrained.rs"]
mod constrained;
#[path = "support/hub_init.rs"]
mod hub_init;
#[path = "support/probed_effector.rs"]
mod probed_effector;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/scenario_args.rs"]
mod scenario_args;
#[path = "support/servicing.rs"]
mod servicing;
#[path = "support/table_csv.rs"]
mod table_csv;

use std::time::Instant;

use basilisk_rs::messages::{HingedRigidBodyMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use constrained::{ScModel, Violations, gain_label, hub_pose_columns};
use hifitime::Epoch;
use nalgebra::Vector3;
use recording::{History, scenario_csv};
use scenario_args::ScenarioArgs;
use servicing::{Components, ServicingConfig, servicing};
use table_csv::write_table;

const USAGE: &str = "\
scenario_constrained_dynamics_frequency_analysis [--gains 1e1,1e2,1e3,1e4,1e5] [--model bskSat|MEV1|MEV2]";

const STEM: &str = "scenarioConstrainedDynamicsFrequencyAnalysis";
/// Upstream `SimBaseClass(dynRate=0.001)`.
const STEP_NANOS: u64 = 1_000_000;
/// Upstream `ConfigureStopTime(macros.sec2nano(10))`.
const DURATION_NANOS: u64 = 10_000_000_000;

/// Frequencies \[Hz\] around the panels' 2 Hz natural frequency (lowered
/// to about 1.7 Hz by the hub motion it drives).
pub const PANEL_BAND_HZ: (f64, f64) = (1.0, 2.5);

/// Upstream's test `gain_list`.
pub const TEST_GAINS: [f64; 5] = [1.0e1, 1.0e2, 1.0e3, 1.0e4, 1.0e5];

/// One gain's run.
pub struct GainRun {
    pub gain: f64,
    pub csv_path: std::path::PathBuf,
    pub violations: Violations,
    /// Upstream `sp1Log.theta` \[rad\]: the chaser's panel.
    pub theta_1_rad: Vec<f64>,
    /// Upstream `sp3Log.theta` \[rad\]: the truth model's matching panel.
    pub theta_3_rad: Vec<f64>,
    /// The one-sided amplitude spectrum of `sigma_B2B1[:, 0]` upstream
    /// plots, as `(frequency [Hz], amplitude)`.
    pub spectrum: Vec<(f64, f64)>,
    /// Upstream's printed frequency of highest amplitude of the spectrum
    /// \[Hz\].
    pub dominant_frequency_hz: f64,
    /// Wall-clock time of the run \[s\].
    pub runtime_s: f64,
}

impl GainRun {
    /// The constraint's fictitious frequency \[Hz\]: the highest local
    /// maximum of the spectrum outside the band of the panels' natural
    /// frequency ([`PANEL_BAND_HZ`]); zero when there is none, the mode
    /// lying below the 0.1 Hz resolution of the 10 s run.
    pub fn fictitious_frequency_hz(&self) -> f64 {
        let spectrum = &self.spectrum;
        (1..spectrum.len().saturating_sub(1))
            .filter(|&bin| {
                spectrum[bin].1 > spectrum[bin - 1].1 && spectrum[bin].1 > spectrum[bin + 1].1
            })
            .map(|bin| spectrum[bin])
            .filter(|(frequency, _)| !(PANEL_BAND_HZ.0..=PANEL_BAND_HZ.1).contains(frequency))
            .fold((0.0, f64::MIN), |best, (frequency, amplitude)| {
                if amplitude > best.1 {
                    (frequency, amplitude)
                } else {
                    best
                }
            })
            .0
    }

    /// The largest `|theta_1 - theta_3|` \[deg\].
    pub fn max_panel_angle_error_deg(&self) -> f64 {
        self.theta_1_rad
            .iter()
            .zip(&self.theta_3_rad)
            .map(|(theta_1, theta_3)| (theta_1 - theta_3).abs().to_degrees())
            .fold(0.0, f64::max)
    }
}

pub struct Results {
    pub runs: Vec<GainRun>,
    pub summary_path: std::path::PathBuf,
}

/// Upstream `plot_fft_panels` for one signal: the one-sided amplitude
/// spectrum `|fft(x)|[:N/2] / (N/2)`, as `(frequency [Hz], amplitude)` at
/// the frequencies `n / T`, `T = N / fs`.
pub fn spectrum(signal: &[f64], sampling_rate_hz: f64) -> Vec<(f64, f64)> {
    let n = signal.len();
    let one_sided = n / 2;
    let period_s = n as f64 / sampling_rate_hz;
    // Twiddles e^{-2 pi i m / N}, indexed by m = k n mod N.
    let twiddles: Vec<(f64, f64)> = (0..n)
        .map(|m| {
            let angle = -2.0 * std::f64::consts::PI * m as f64 / n as f64;
            (angle.cos(), angle.sin())
        })
        .collect();
    (0..one_sided)
        .map(|bin| {
            let (mut real, mut imaginary) = (0.0, 0.0);
            let mut index = 0usize;
            for value in signal {
                let (cos, sin) = twiddles[index];
                real += value * cos;
                imaginary += value * sin;
                index += bin;
                if index >= n {
                    index -= n;
                }
            }
            let amplitude = (real * real + imaginary * imaginary).sqrt() / one_sided as f64;
            (bin as f64 / period_s, amplitude)
        })
        .collect()
}

/// The frequency of the largest amplitude of a [`spectrum`] \[Hz\]:
/// upstream's printed "frequency of highest amplitude".
pub fn peak_frequency_hz(spectrum: &[(f64, f64)]) -> f64 {
    spectrum
        .iter()
        .fold((0.0, f64::MIN), |best, &(frequency, amplitude)| {
            if amplitude > best.1 {
                (frequency, amplitude)
            } else {
                best
            }
        })
        .0
}

fn run_gain(model: ScModel, gain: f64) -> GainRun {
    let started = Instant::now();
    let mut setup = servicing(&ServicingConfig {
        model,
        components: Components {
            panels: true,
            slosh: false,
        },
        r_hat: Vector3::x(),
        gain,
        step_nanos: STEP_NANOS,
    });
    let panel_out = setup.panel_out.clone().expect("panels");

    let file_stem = format!("{STEM}_{}_alpha{}", model.label(), gain_label(gain));
    let mut csv = scenario_csv(&file_stem);
    let csv_path = csv.config.output_path.clone();
    let mut history_1 = History::<SpacecraftStateMsg>::new();
    let mut history_2 = History::<SpacecraftStateMsg>::new();
    let mut panel_1 = History::<HingedRigidBodyMsg>::new();
    let mut panel_3 = History::<HingedRigidBodyMsg>::new();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        let [state_1, state_2] =
            [0, 1].map(|index| setup.group.spacecraft()[index].state_out.clone());
        connect!(&simulation,
            &state_1 => csv.add_source::<SpacecraftStateMsg>(hub_pose_columns("B1")),
            &state_2 => csv.add_source::<SpacecraftStateMsg>(hub_pose_columns("B2")),
            &state_1 => &mut history_1.input,
            &state_2 => &mut history_2.input,
            &panel_out[0] => &mut panel_1.input,
            &panel_out[2] => &mut panel_3.input,
        );
        for (index, output) in panel_out.iter().enumerate() {
            let panel = index + 1;
            connect!(&simulation,
                output => csv.add_source::<HingedRigidBodyMsg>(CsvSourceConfig::columns([
                    ("theta_rad".to_string(), format!("theta{panel}_rad")),
                    ("theta_dot_radps".to_string(), format!("thetaDot{panel}_rad_per_s")),
                ])),
            );
        }
        schedule! { simulation,
            "spacecraftGroup" => &mut setup.group, period=STEP_NANOS, priority=10;
            "spacecraftBodyT" => &mut setup.truth, period=STEP_NANOS, priority=10;
            "csv" => &mut csv, period=STEP_NANOS, priority=0;
            "datLog1" => &mut history_1, period=STEP_NANOS, priority=0;
            "datLog2" => &mut history_2, period=STEP_NANOS, priority=0;
            "sp1Log" => &mut panel_1, period=STEP_NANOS, priority=0;
            "sp3Log" => &mut panel_3, period=STEP_NANOS, priority=0;
        }
        simulation.run_for(DURATION_NANOS);
    }
    csv.flush().expect("flush scenario CSV");
    let runtime_s = started.elapsed().as_secs_f64();

    let violations = Violations::new(&setup.geometry, history_1.samples(), history_2.samples());
    let sigma_first: Vec<f64> = violations
        .attitude_sigma_b2b1
        .iter()
        .map(|sigma| sigma[0])
        .collect();
    let spectrum = spectrum(&sigma_first, 1.0e9 / STEP_NANOS as f64);
    let dominant_frequency_hz = peak_frequency_hz(&spectrum);
    GainRun {
        gain,
        csv_path,
        violations,
        theta_1_rad: panel_1.messages().map(|panel| panel.theta_rad).collect(),
        theta_3_rad: panel_3.messages().map(|panel| panel.theta_rad).collect(),
        spectrum,
        dominant_frequency_hz,
        runtime_s,
    }
}

pub fn run(gains: &[f64], model: ScModel) -> Results {
    let runs: Vec<GainRun> = gains.iter().map(|gain| run_gain(model, *gain)).collect();
    let summary_path = write_table(
        &format!("{STEM}_{}_summary", model.label()),
        &[
            "gain".to_string(),
            "dominant_frequency_Hz".to_string(),
            "fictitious_frequency_Hz".to_string(),
            "max_panel_angle_error_deg".to_string(),
            "runtime_s".to_string(),
        ],
        &runs
            .iter()
            .map(|run| {
                vec![
                    run.gain,
                    run.dominant_frequency_hz,
                    run.fictitious_frequency_hz(),
                    run.max_panel_angle_error_deg(),
                    run.runtime_s,
                ]
            })
            .collect::<Vec<_>>(),
    );
    Results { runs, summary_path }
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let gains: Vec<f64> = match args.value("--gains", String::new()).as_str() {
        "" => TEST_GAINS.to_vec(),
        list => list
            .split(',')
            .map(|gain| {
                gain.parse()
                    .unwrap_or_else(|_| args.fail(&format!("invalid gain {gain:?}")))
            })
            .collect(),
    };
    let model_name = args.value("--model", "MEV2".to_string());
    let model = ScModel::parse(&model_name)
        .unwrap_or_else(|| args.fail(&format!("unknown model {model_name:?}")));
    let results = run(&gains, model);
    for run in &results.runs {
        println!(
            "wrote {}: frequency of highest amplitude for gain={} is {} Hz (constraint mode \
             {} Hz), max panel angle error {:.3e} deg ({:.1} s)",
            run.csv_path.display(),
            run.gain,
            run.dominant_frequency_hz,
            run.fictitious_frequency_hz(),
            run.max_panel_angle_error_deg(),
            run.runtime_s,
        );
    }
    println!("wrote {}", results.summary_path.display());
}

#[cfg(test)]
mod tests {
    use super::{ScModel, TEST_GAINS, peak_frequency_hz, run, spectrum};

    /// The spectrum's peak sits on a pure tone's frequency bin.
    #[test]
    fn spectrum_peaks_at_a_tone() {
        let rate_hz = 1000.0;
        let tone: Vec<f64> = (0..1000)
            .map(|n| (2.0 * std::f64::consts::PI * 37.0 * n as f64 / rate_hz).sin())
            .collect();
        let spectrum = spectrum(&tone, rate_hz);
        assert_eq!(spectrum.len(), 500);
        assert_eq!(peak_frequency_hz(&spectrum), 37.0);
        assert!((spectrum[37].1 - 1.0).abs() < 1.0e-9);
    }

    /// `test_scenarioConstrainedDynamicsFrequencyAnalysis.py`: the five gains
    /// with `MEV2` run to completion. Upstream's description of the figures,
    /// checked here: besides the panels' natural frequency, the attitude
    /// violation rings at a fictitious frequency of the constraint, which
    /// rises with the gain as the arm stiffens (about tenfold per tenfold
    /// gain, as `k = alpha^2`); and once that frequency has moved above the
    /// panels' the panel angle error falls, the stiffest arms following the
    /// rigid truth model to within a hundredth of the softest arm's error.
    #[test]
    fn stiffer_constraints_track_the_rigid_truth() {
        let results = run(&TEST_GAINS, ScModel::Mev2);
        assert_eq!(results.runs.len(), TEST_GAINS.len());
        for run in &results.runs {
            assert_eq!(run.theta_1_rad.len(), 10_001, "gain {}", run.gain);
            assert!(run.violations.max_direction_m().is_finite());
        }
        let frequencies: Vec<f64> = results
            .runs
            .iter()
            .map(|run| run.fictitious_frequency_hz())
            .collect();
        for pair in frequencies[1..].windows(2) {
            let ratio = pair[1] / pair[0];
            assert!(
                (5.0..20.0).contains(&ratio),
                "constraint frequencies {frequencies:?} Hz"
            );
        }
        let errors: Vec<f64> = results
            .runs
            .iter()
            .map(|run| run.max_panel_angle_error_deg())
            .collect();
        for pair in errors[1..].windows(2) {
            assert!(pair[1] < pair[0], "panel angle errors {errors:?} deg");
        }
        for stiff in &errors[3..] {
            assert!(
                *stiff < 1.0e-2 * errors[0],
                "panel angle errors {errors:?} deg"
            );
        }
    }
}
