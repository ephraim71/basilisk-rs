//! Port of upstream `examples/scenarioConstrainedDynamicsComponentAnalysis.py`:
//! does the docking constraint carry the coupling of flexible components?
//! A chaser and a target spacecraft are held together by a constraint tuned
//! with `alpha = beta = 1e4`, and compared with a single spacecraft that
//! models the docked stack as one rigid hub carrying the same components.
//! With `panels`, each spacecraft has a hinged solar panel and the chaser's
//! is kicked to 7 deg/s; with `slosh`, the chaser has a fuel tank with three
//! slosh particles kicked to 0.02, -0.025 and -0.015 m/s. The run lasts 10 s
//! at a 1 ms step (RKF45).
//!
//! | option | upstream | default |
//! |---|---|---|
//! | `--components panels\|slosh\|none` (or `panels,slosh`) | `component_list` | `panels` |
//! | `--model bskSat\|MEV1\|MEV2` | `sc_model` | `MEV2` |
//!
//! `--all` runs upstream's test parametrizations, `panels` and `slosh` with
//! `MEV2`. Each run writes
//! `scenarioConstrainedDynamicsComponentAnalysis_<components>_<model>.csv`
//! (`_panels_MEV2`, `_slosh_MEV2`, `_None_MEV2`): both hub poses
//! (`r_B1N_N_m_i`, `sigma_B1N_i`, `r_B2N_N_m_i`, `sigma_B2N_i`), and the four
//! panel angles and rates (`theta<k>_rad`, `thetaDot<k>_rad_per_s`; panels 1
//! and 2 on the chaser and target, 3 and 4 on the truth model) or the six
//! slosh displacements and rates (`rho<k>_m`, `rhoDot<k>_m_per_s`; particles
//! 1–3 on the chaser, 4–6 on the truth model) at every step.
//! `plot_scenario_constrained_dynamics_component_analysis.py` draws upstream's
//! `DirectionConstraint_*`, `AttitudeConstraint_*`, `PanelAngleErrors` and
//! `SloshMassDisplacementErrors` figures.
//!
//! basilisk-rs's fuel tank owns its slosh particles, so only the tank is
//! added to the spacecraft (upstream adds the tank and each particle); the
//! particle states upstream logs from the state manager are published by a
//! [`multibody::Hosted`] tank. Upstream's Vizard setup is not ported.
//!
//! Upstream's `test_scenarioConstrainedDynamicsComponentAnalysis.py` runs
//! `panels` and `slosh` with `MEV2` and checks only that they complete.
//! `tests` runs the same and checks what upstream's description of the
//! figures says: the constraint holds the pair together, and the error
//! between the docked pair's component and the truth model's stays much
//! smaller than the excitation and damps out.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/constrained.rs"]
mod constrained;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/multibody.rs"]
mod multibody;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/servicing.rs"]
mod servicing;

use basilisk_rs::messages::{
    HingedRigidBodyMsg, LinearTranslationRigidBodyMsg, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use constrained::{ScModel, Violations, hub_pose_columns};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;
use recording::scenario_csv;
use servicing::{Components, ServicingConfig, servicing};

const USAGE: &str = "\
scenario_constrained_dynamics_component_analysis [--components panels|slosh|none|panels,slosh] [--model bskSat|MEV1|MEV2]
scenario_constrained_dynamics_component_analysis --all     run upstream's test parametrizations";

const STEM: &str = "scenarioConstrainedDynamicsComponentAnalysis";
/// Upstream `SimBaseClass(dynRate=0.001)`.
const STEP_NANOS: u64 = 1_000_000;
/// Upstream `ConfigureStopTime(macros.sec2nano(10))`.
const DURATION_NANOS: u64 = 10_000_000_000;
/// Upstream `constraintEffector.setAlpha(1E4)`, and `beta = alpha`.
const GAIN: f64 = 1.0e4;

/// Upstream `run(show_plots, component_list, sc_model)`.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub components: Components,
    pub model: ScModel,
}

impl Config {
    /// Upstream's `component_list` in the file names: `panels`, `slosh`,
    /// `None`, `panels_slosh`.
    pub fn components_label(&self) -> String {
        match (self.components.panels, self.components.slosh) {
            (true, true) => "panels_slosh".to_string(),
            (true, false) => "panels".to_string(),
            (false, true) => "slosh".to_string(),
            (false, false) => "None".to_string(),
        }
    }

    pub fn label(&self) -> String {
        format!("_{}_{}", self.components_label(), self.model.label())
    }
}

/// The parametrizations of `test_scenarioConstrainedDynamicsComponentAnalysis.py`.
pub const TEST_CASES: [Config; 2] = [
    Config {
        components: Components {
            panels: true,
            slosh: false,
        },
        model: ScModel::Mev2,
    },
    Config {
        components: Components {
            panels: false,
            slosh: true,
        },
        model: ScModel::Mev2,
    },
];

/// One component and its counterpart on the truth model, sample by sample.
#[derive(Clone, Debug, Default)]
pub struct ComponentPair {
    /// The docked pair's component: a panel angle \[rad\] or a slosh
    /// displacement \[m\].
    pub docked: Vec<f64>,
    /// The truth model's.
    pub truth: Vec<f64>,
}

impl ComponentPair {
    /// Upstream's plotted error, `docked - truth`.
    pub fn error(&self) -> Vec<f64> {
        self.docked
            .iter()
            .zip(&self.truth)
            .map(|(docked, truth)| docked - truth)
            .collect()
    }
}

pub struct Results {
    pub csv_path: std::path::PathBuf,
    pub times_nanos: Vec<u64>,
    pub violations: Violations,
    /// Upstream's `sp1Log`/`sp3Log` and `sp2Log`/`sp4Log` panel angles, with
    /// `panels`.
    pub panels: Vec<ComponentPair>,
    /// Upstream's `rhoLog`/`rhoTLog` displacements of particles 1/4, 2/5 and
    /// 3/6, with `slosh`.
    pub slosh: Vec<ComponentPair>,
}

pub fn run(config: &Config) -> Results {
    let mut setup = servicing(&ServicingConfig {
        model: config.model,
        components: config.components,
        r_hat: Vector3::repeat(1.0).normalize(),
        gain: GAIN,
        step_nanos: STEP_NANOS,
    });

    let file_stem = format!("{STEM}{}", config.label());
    let mut csv = scenario_csv(&file_stem);
    let csv_path = csv.config.output_path.clone();
    let mut history_1 = MessageLog::<SpacecraftStateMsg>::default();
    let mut history_2 = MessageLog::<SpacecraftStateMsg>::default();
    let mut panel_logs: Vec<MessageLog<HingedRigidBodyMsg>> = Vec::new();
    let mut rho_logs: Vec<MessageLog<LinearTranslationRigidBodyMsg>> = Vec::new();
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
        );
        if let Some(panel_out) = &setup.panel_out {
            for (index, output) in panel_out.iter().enumerate() {
                let panel = index + 1;
                let mut log = MessageLog::default();
                connect!(&simulation,
                    output => csv.add_source::<HingedRigidBodyMsg>(CsvSourceConfig::columns([
                        ("theta_rad".to_string(), format!("theta{panel}_rad")),
                        ("theta_dot_radps".to_string(), format!("thetaDot{panel}_rad_per_s")),
                    ])),
                    output => &mut log.input,
                );
                panel_logs.push(log);
            }
        }
        if let (Some(rho_out), Some(rho_truth_out)) = (&setup.rho_out, &setup.rho_truth_out) {
            for (index, output) in rho_out.iter().chain(rho_truth_out).enumerate() {
                let particle = index + 1;
                let mut log = MessageLog::default();
                connect!(&simulation,
                    output => csv.add_source::<LinearTranslationRigidBodyMsg>(
                        CsvSourceConfig::columns([
                            ("rho_m".to_string(), format!("rho{particle}_m")),
                            ("rho_dot_mps".to_string(), format!("rhoDot{particle}_m_per_s")),
                        ]),
                    ),
                    output => &mut log.input,
                );
                rho_logs.push(log);
            }
        }
        schedule! { simulation,
            "spacecraftGroup" => &mut setup.group, period=STEP_NANOS, priority=10;
            "spacecraftBodyT" => &mut setup.truth, period=STEP_NANOS, priority=10;
            "csv" => &mut csv, period=STEP_NANOS, priority=0;
            "datLog1" => &mut history_1, period=STEP_NANOS, priority=0;
            "datLog2" => &mut history_2, period=STEP_NANOS, priority=0;
        }
        for (index, log) in panel_logs.iter_mut().enumerate() {
            simulation.add_module(format!("sp{}Log", index + 1), log, STEP_NANOS, 0);
        }
        for (index, log) in rho_logs.iter_mut().enumerate() {
            simulation.add_module(format!("rho{}Log", index + 1), log, STEP_NANOS, 0);
        }
        simulation.run_for(DURATION_NANOS);
    }
    csv.flush().expect("flush scenario CSV");

    let times_nanos = history_1.times_nanos.clone();
    let violations = Violations::new(
        &setup.geometry,
        &history_1.into_samples(),
        &history_2.into_samples(),
    );
    let values = |log: &MessageLog<HingedRigidBodyMsg>| -> Vec<f64> {
        log.values.iter().map(|panel| panel.theta_rad).collect()
    };
    let panels = if panel_logs.is_empty() {
        Vec::new()
    } else {
        vec![
            ComponentPair {
                docked: values(&panel_logs[0]),
                truth: values(&panel_logs[2]),
            },
            ComponentPair {
                docked: values(&panel_logs[1]),
                truth: values(&panel_logs[3]),
            },
        ]
    };
    let displacement = |log: &MessageLog<LinearTranslationRigidBodyMsg>| -> Vec<f64> {
        log.values.iter().map(|particle| particle.rho_m).collect()
    };
    let slosh = if rho_logs.is_empty() {
        Vec::new()
    } else {
        (0..3)
            .map(|index| ComponentPair {
                docked: displacement(&rho_logs[index]),
                truth: displacement(&rho_logs[index + 3]),
            })
            .collect()
    };
    Results {
        csv_path,
        times_nanos,
        violations,
        panels,
        slosh,
    }
}

fn main() {
    let mut args = Cli::from_env(USAGE);
    let configs = if args.switch("all") {
        TEST_CASES.to_vec()
    } else {
        let mut components = Components::default();
        for component in args.value("components", "panels".to_string()).split(',') {
            match component {
                "panels" => components.panels = true,
                "slosh" => components.slosh = true,
                "none" | "None" => {}
                other => args.fail(&format!("unknown component {other:?}")),
            }
        }
        let model_name = args.value("model", "MEV2".to_string());
        let model = ScModel::parse(&model_name)
            .unwrap_or_else(|| args.fail(&format!("unknown model {model_name:?}")));
        vec![Config { components, model }]
    };
    for config in configs {
        let results = run(&config);
        println!(
            "wrote {} (max direction violation {:.3e} m, max attitude violation {:.3e} deg)",
            results.csv_path.display(),
            results.violations.max_direction_m(),
            results.violations.max_attitude_deg(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::{ComponentPair, Results, TEST_CASES, run};

    fn max_abs(values: &[f64]) -> f64 {
        values.iter().map(|value| value.abs()).fold(0.0, f64::max)
    }

    /// The largest component errors of the first and the last second of
    /// the run (1 ms samples).
    fn error_decay(pair: &ComponentPair) -> (f64, f64) {
        let error = pair.error();
        (
            max_abs(&error[..1000]),
            max_abs(&error[error.len() - 1000..]),
        )
    }

    /// Each error against the motion the excitation causes (the largest
    /// truth-model component motion), and its decay.
    fn check_components(label: &str, results: &Results, pairs: &[ComponentPair]) {
        assert_eq!(results.times_nanos.len(), 10_001, "{label}");
        let excitation = pairs
            .iter()
            .map(|pair| max_abs(&pair.truth))
            .fold(0.0, f64::max);
        for (index, pair) in pairs.iter().enumerate() {
            let (early, late) = error_decay(pair);
            assert!(
                early < 0.02 * excitation,
                "{label} {index}: error {early:e} against the excited motion {excitation:e}"
            );
            assert!(
                late < 0.5 * early,
                "{label} {index}: error {late:e} in the last second, {early:e} in the first"
            );
        }
    }

    /// `test_scenarioConstrainedDynamicsComponentAnalysis.py`: `panels` and
    /// `slosh` with `MEV2` run to completion. Upstream's description of the
    /// figures, checked here: the constraint enforces the rigid connection
    /// (below 0.1 mm and a thousandth of a degree), and the error between
    /// each component of the docked pair and its truth-model counterpart
    /// stays much smaller than the motion the excitation causes (under 2 %
    /// of the largest) and damps out over the run (the last second's below
    /// half the first's).
    #[test]
    fn docked_components_follow_the_rigid_truth_model() {
        for config in TEST_CASES {
            let label = config.label();
            let results = run(&config);
            let direction_m = results.violations.max_direction_m();
            let attitude_deg = results.violations.max_attitude_deg();
            assert!(direction_m < 1.0e-4, "{label}: |psi| {direction_m:e} m");
            assert!(attitude_deg < 1.0e-3, "{label}: |phi| {attitude_deg:e} deg");
            if config.components.panels {
                assert_eq!(results.panels.len(), 2, "{label}");
                check_components(&label, &results, &results.panels);
            }
            if config.components.slosh {
                assert_eq!(results.slosh.len(), 3, "{label}");
                check_components(&label, &results, &results.slosh);
            }
        }
    }
}
