//! Port of upstream `examples/scenarioConstrainedDynamicsManeuverAnalysis.py`:
//! how the docking constraint's gain trades fidelity against run time while
//! the docked pair maneuvers. A chaser and a target hub are held together by
//! a constraint tuned with `alpha = beta = gain`, coast for 5 min, maneuver
//! for 5 min and coast for another 10 min, at a 0.1 s step (RKF45). The
//! `orbit` maneuver pushes the chaser with 1 N along the arm, toward the
//! target; the `attitude` maneuver slews the chaser by 180° about ẑ with an
//! MRP feedback controller (`K = 1`, `P = 50`, no integral feedback) on the
//! chaser's own inertia, its command torque connected at 5 min.
//!
//! | option | upstream | default |
//! |---|---|---|
//! | `--gains 1e1,1e2,...` | `gain_list` | `1e1,1e2,1e3` (upstream's test) |
//! | `--relpos alongtrackahead\|alongtrackbehind\|radial\|antiradial` | `relpos_config` | `alongtrackahead` |
//! | `--orbit LEO\|"no orbit"` | `orbit_config` | `LEO` |
//! | `--maneuver attitude\|orbit` | `maneuver_config` | `attitude` |
//! | `--model bskSat\|MEV1\|MEV2` | `sc_model` | `MEV2` |
//!
//! `--all` runs upstream's test parametrizations: both maneuvers, the three
//! gains, `alongtrackahead`, `LEO`, `MEV2`. Each gain writes
//! `scenarioConstrainedDynamicsManeuverAnalysis_<maneuver>_<relpos>_<orbit>_<model>_alpha<gain>.csv`
//! (`no orbit` as `noorbit`) with both hub states (`r_B1N_N_m_i`,
//! `v_B1N_N_m_per_s_i`, `sigma_B1N_i`, `omega_B1N_B1_rad_per_s_i`, and the
//! same for `B2`) at every step; `..._summary.csv` holds upstream's
//! gain-performance data: the mean direction and attitude violations and the
//! wall-clock runtime of each gain. `plot_scenario_constrained_dynamics_maneuver_analysis.py`
//! draws upstream's `DirectionConstraint`, `AttitudeConstraint` and
//! `GainPerformance` figures.
//!
//! Between runs upstream sets `extFT.extForce_B` and subscribes
//! `extFT.cmdTorqueInMsg`; here the force comes from a `cmdForceBodyInMsg`
//! the scenario writes between runs, and the torque through a
//! [`latch::Latch`] of the controller's command held until 5 min
//! ([`staged::Staged`]), as upstream's unsubscribed input reads nothing.
//! Upstream's initial center of mass `r_B1C = -(m1 + m2 r_B2B1) / (m1 + m2)`
//! adds the scalar `m1` to each component; it is kept, as it only shifts
//! where the docked pair starts. Upstream's Vizard setup is not ported.
//!
//! Upstream's `test_scenarioConstraintDynamicsManeuverAnalysis.py` runs both
//! maneuvers and checks only that they complete. `tests` runs the same and
//! checks what the scenario demonstrates: the stiffer the constraint, the
//! smaller its violations; the attitude maneuver slews the docked pair to
//! the commanded attitude; the orbit maneuver gives the pair the momentum of
//! the push.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/common.rs"]
mod common;
#[path = "support/constrained.rs"]
mod constrained;
#[path = "support/latch.rs"]
mod latch;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/scenario_args.rs"]
mod scenario_args;
#[path = "support/staged.rs"]
mod staged;
#[path = "support/table_csv.rs"]
mod table_csv;

use std::time::Instant;

use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::messages::{
    AttitudeReferenceMsg, BodyForceCommandMsg, BodyTorqueCommandMsg, Output, SpacecraftStateMsg,
    VehicleConfigMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::SpacecraftGroup;
use basilisk_rs::{connect, schedule};
use constrained::{
    ConstraintGeometry, ScModel, Violations, earth_point_mass, gain_label, hub, hub_columns,
    leo_state, model_inertia_kg_m2, upstream_rkf45,
};
use hifitime::Epoch;
use latch::Latch;
use nalgebra::Vector3;
use recording::{History, scenario_csv};
use scenario_args::ScenarioArgs;
use staged::Staged;
use table_csv::write_table;

const USAGE: &str = "\
scenario_constrained_dynamics_maneuver_analysis [--gains 1e1,1e2,1e3] [--relpos alongtrackahead|alongtrackbehind|radial|antiradial]
    [--orbit LEO|\"no orbit\"] [--maneuver attitude|orbit] [--model bskSat|MEV1|MEV2]
scenario_constrained_dynamics_maneuver_analysis --all     run upstream's test parametrizations";

const STEM: &str = "scenarioConstrainedDynamicsManeuverAnalysis";
/// Upstream `SimBaseClass(dynRate=0.1)`.
const STEP_NANOS: u64 = 100_000_000;
/// Upstream's stop times: coast to 5 min, maneuver to 10 min, coast to
/// 20 min.
const MANEUVER_START_NANOS: u64 = 300_000_000_000;
const MANEUVER_END_NANOS: u64 = 600_000_000_000;
const STOP_NANOS: u64 = 1_200_000_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RelativePosition {
    AlongTrackAhead,
    AlongTrackBehind,
    Radial,
    AntiRadial,
}

impl RelativePosition {
    pub fn label(self) -> &'static str {
        match self {
            Self::AlongTrackAhead => "alongtrackahead",
            Self::AlongTrackBehind => "alongtrackbehind",
            Self::Radial => "radial",
            Self::AntiRadial => "antiradial",
        }
    }

    fn parse(name: &str) -> Option<Self> {
        [
            Self::AlongTrackAhead,
            Self::AlongTrackBehind,
            Self::Radial,
            Self::AntiRadial,
        ]
        .into_iter()
        .find(|case| case.label() == name)
    }

    /// Upstream `rHat`: chaser to target.
    fn direction(self, r_cn: Vector3<f64>, rdot_cn: Vector3<f64>) -> Vector3<f64> {
        match self {
            Self::AlongTrackAhead => rdot_cn.normalize(),
            Self::AlongTrackBehind => -rdot_cn.normalize(),
            Self::Radial => r_cn.normalize(),
            Self::AntiRadial => -r_cn.normalize(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OrbitCase {
    /// Upstream `LEO`.
    Leo,
    /// Upstream `no orbit`: drifting in deep space.
    NoOrbit,
}

impl OrbitCase {
    pub fn label(self) -> &'static str {
        match self {
            Self::Leo => "LEO",
            Self::NoOrbit => "noorbit",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Maneuver {
    Orbit,
    Attitude,
}

impl Maneuver {
    pub fn label(self) -> &'static str {
        match self {
            Self::Orbit => "orbit",
            Self::Attitude => "attitude",
        }
    }
}

/// Upstream `run(show_plots, gain_list, relpos_config, orbit_config,
/// maneuver_config, sc_model)`.
#[derive(Clone, Debug)]
pub struct Config {
    pub gains: Vec<f64>,
    pub relpos: RelativePosition,
    pub orbit: OrbitCase,
    pub maneuver: Maneuver,
    pub model: ScModel,
}

impl Config {
    pub fn label(&self) -> String {
        format!(
            "_{}_{}_{}_{}",
            self.maneuver.label(),
            self.relpos.label(),
            self.orbit.label(),
            self.model.label()
        )
    }
}

/// The parametrizations of `test_scenarioConstraintDynamicsManeuverAnalysis.py`.
pub fn test_cases() -> Vec<Config> {
    [Maneuver::Attitude, Maneuver::Orbit]
        .into_iter()
        .map(|maneuver| Config {
            gains: vec![1.0e1, 1.0e2, 1.0e3],
            relpos: RelativePosition::AlongTrackAhead,
            orbit: OrbitCase::Leo,
            maneuver,
            model: ScModel::Mev2,
        })
        .collect()
}

/// Upstream `sigma_R0N = [0, 0, tan(pi/4)]`: 180° about ẑ.
pub fn reference_sigma_rn() -> Vector3<f64> {
    Vector3::new(0.0, 0.0, std::f64::consts::FRAC_PI_4.tan())
}

/// One gain's run.
pub struct GainRun {
    pub gain: f64,
    pub csv_path: std::path::PathBuf,
    pub geometry: ConstraintGeometry,
    pub states_1: Vec<(u64, SpacecraftStateMsg)>,
    pub states_2: Vec<(u64, SpacecraftStateMsg)>,
    pub violations: Violations,
    /// Upstream `scSim.runtime` \[s\].
    pub runtime_s: f64,
}

pub struct Results {
    pub runs: Vec<GainRun>,
    pub summary_path: std::path::PathBuf,
}

fn run_gain(config: &Config, gain: f64) -> GainRun {
    let started = Instant::now();
    let model = config.model.data();
    let inertia = model_inertia_kg_m2();
    let mut chaser = hub(model.chaser_mass_kg, Vector3::zeros(), inertia, STEP_NANOS);
    let mut target = hub(model.target_mass_kg, Vector3::zeros(), inertia, STEP_NANOS);

    let (r_cn, rdot_cn) = match config.orbit {
        OrbitCase::Leo => {
            for spacecraft in [&mut chaser, &mut target] {
                spacecraft
                    .add_grav_body(earth_point_mass())
                    .expect("one central body");
            }
            leo_state()
        }
        OrbitCase::NoOrbit => (Vector3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
    };
    let r_hat = config.relpos.direction(r_cn, rdot_cn);
    let geometry = ConstraintGeometry::along(r_hat, &model);
    let r_b2b1 = geometry.r_b2b1_m();
    let (m1, m2) = (model.chaser_mass_kg, model.target_mass_kg);
    // Upstream's expression, the scalar `m1` added to each component.
    let r_b1c = -(Vector3::repeat(m1) + m2 * r_b2b1) / (m1 + m2);
    let r_b2c = r_b1c + r_b2b1;
    // The hubs carry nothing, so their centers of mass are at B.
    chaser.config.initial_position_m = r_b1c + r_cn;
    chaser.config.initial_velocity_mps = rdot_cn;
    target.config.initial_position_m = r_b2c + r_cn;
    target.config.initial_velocity_mps = rdot_cn;

    // Upstream `set_up_maneuver`: the external force/torque on the chaser.
    let mut ext_ft = ExtForceTorque::new(ExtForceTorqueConfig {
        name: match config.maneuver {
            Maneuver::Orbit => "appliedForce",
            Maneuver::Attitude => "appliedTorque",
        }
        .to_string(),
        ..ExtForceTorqueConfig::default()
    });
    let force_command = Output::<BodyForceCommandMsg>::default();
    force_command.connect_to(&mut ext_ft.cmd_force_body_in_msg);
    let mut torque_latch =
        Staged::new(Latch::<BodyTorqueCommandMsg>::default()).held_until(MANEUVER_START_NANOS);
    torque_latch
        .module
        .output
        .connect_to(&mut ext_ft.cmd_torque_in_msg);
    chaser.add_dynamic_effector(ext_ft);

    let mut group = SpacecraftGroup::new(vec![chaser, target]);
    group.integrator = Some(Box::new(upstream_rkf45()));
    group.add_coupling_effector(geometry.constraint(gain, gain));

    // The attitude maneuver's flight software.
    let vehicle_config = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: inertia,
        ..Default::default()
    });
    let attitude_reference = Output::new(AttitudeReferenceMsg {
        sigma_rn: reference_sigma_rn(),
        ..Default::default()
    });
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut tracking_error = attitude::att_tracking_error(Vector3::zeros());
    tracking_error.config.name = "scSim.attErrorHillPoint".to_string();
    let mut mrp_control = attitude::mrp_feedback(1.0, -1.0, 50.0, Vector3::zeros());

    let file_stem = format!("{STEM}{}_alpha{}", config.label(), gain_label(gain));
    let mut csv = scenario_csv(&file_stem);
    let csv_path = csv.config.output_path.clone();
    let mut history_1 = History::<SpacecraftStateMsg>::new();
    let mut history_2 = History::<SpacecraftStateMsg>::new();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        let [state_1, state_2] = [0, 1].map(|index| group.spacecraft()[index].state_out.clone());
        connect!(&simulation,
            &state_1 => csv.add_source::<SpacecraftStateMsg>(hub_columns("B1")),
            &state_2 => csv.add_source::<SpacecraftStateMsg>(hub_columns("B2")),
            &state_1 => &mut history_1.input,
            &state_2 => &mut history_2.input,
        );
        schedule! { simulation,
            "spacecraftGroup" => &mut group, period=STEP_NANOS, priority=100;
        }
        if config.maneuver == Maneuver::Attitude {
            connect!(&simulation,
                &state_1 => &mut navigation.spacecraft_state_in_msg,
                &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
                &attitude_reference => &mut tracking_error.att_ref_in_msg,
                &tracking_error.att_guid_out_msg => &mut mrp_control.guid_in_msg,
                &vehicle_config => &mut mrp_control.veh_config_in_msg,
                &mrp_control.cmd_torque_out_msg => &mut torque_latch.module.input,
            );
            schedule! { simulation,
                "SimpleNavigation" => &mut navigation, period=STEP_NANOS, priority=90;
                "attError" => &mut tracking_error, period=STEP_NANOS, priority=80;
                "mrpFeedback" => &mut mrp_control, period=STEP_NANOS, priority=70;
                "appliedTorque" => &mut torque_latch, period=STEP_NANOS, priority=60;
            }
        }
        schedule! { simulation,
            "csv" => &mut csv, period=STEP_NANOS, priority=0;
            "datLog1" => &mut history_1, period=STEP_NANOS, priority=0;
            "datLog2" => &mut history_2, period=STEP_NANOS, priority=0;
        }

        simulation.run_for(MANEUVER_START_NANOS);
        if config.maneuver == Maneuver::Orbit {
            // Low thrust aligned to push the target.
            force_command.write(BodyForceCommandMsg {
                force_request_body_n: -geometry.r_p2p1_b1_init_m.normalize(),
            });
        }
        simulation.run_for(MANEUVER_END_NANOS - MANEUVER_START_NANOS);
        // Turn off the low thrust.
        force_command.write(BodyForceCommandMsg::default());
        simulation.run_for(STOP_NANOS - MANEUVER_END_NANOS);
    }
    csv.flush().expect("flush scenario CSV");
    let runtime_s = started.elapsed().as_secs_f64();

    let states_1 = history_1.into_samples();
    let states_2 = history_2.into_samples();
    let violations = Violations::new(&geometry, &states_1, &states_2);
    GainRun {
        gain,
        csv_path,
        geometry,
        states_1,
        states_2,
        violations,
        runtime_s,
    }
}

pub fn run(config: &Config) -> Results {
    let runs: Vec<GainRun> = config
        .gains
        .iter()
        .map(|gain| run_gain(config, *gain))
        .collect();
    let summary_path = write_table(
        &format!("{STEM}{}_summary", config.label()),
        &[
            "gain".to_string(),
            "mean_direction_violation_m".to_string(),
            "mean_attitude_violation_deg".to_string(),
            "runtime_s".to_string(),
        ],
        &runs
            .iter()
            .map(|run| {
                vec![
                    run.gain,
                    run.violations.mean_direction_m(),
                    run.violations.mean_attitude_deg(),
                    run.runtime_s,
                ]
            })
            .collect::<Vec<_>>(),
    );
    Results { runs, summary_path }
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let configs = if args.flag("--all") {
        test_cases()
    } else {
        let gains: Vec<f64> = match args.value("--gains", String::new()).as_str() {
            "" => vec![1.0e1, 1.0e2, 1.0e3],
            list => list
                .split(',')
                .map(|gain| {
                    gain.parse()
                        .unwrap_or_else(|_| args.fail(&format!("invalid gain {gain:?}")))
                })
                .collect(),
        };
        let relpos_name = args.value("--relpos", "alongtrackahead".to_string());
        let relpos = RelativePosition::parse(&relpos_name)
            .unwrap_or_else(|| args.fail(&format!("unknown relpos {relpos_name:?}")));
        let orbit = match args.value("--orbit", "LEO".to_string()).as_str() {
            "LEO" => OrbitCase::Leo,
            "no orbit" | "noorbit" => OrbitCase::NoOrbit,
            other => args.fail(&format!("unknown orbit {other:?}")),
        };
        let maneuver = match args.value("--maneuver", "attitude".to_string()).as_str() {
            "attitude" => Maneuver::Attitude,
            "orbit" => Maneuver::Orbit,
            other => args.fail(&format!("unknown maneuver {other:?}")),
        };
        let model_name = args.value("--model", "MEV2".to_string());
        let model = ScModel::parse(&model_name)
            .unwrap_or_else(|| args.fail(&format!("unknown model {model_name:?}")));
        vec![Config {
            gains,
            relpos,
            orbit,
            maneuver,
            model,
        }]
    };
    for config in configs {
        let results = run(&config);
        for run in &results.runs {
            println!(
                "wrote {} (mean violations {:.3e} m, {:.3e} deg; {:.1} s)",
                run.csv_path.display(),
                run.violations.mean_direction_m(),
                run.violations.mean_attitude_deg(),
                run.runtime_s,
            );
        }
        println!("wrote {}", results.summary_path.display());
    }
}

#[cfg(test)]
mod tests {
    use super::{
        MANEUVER_END_NANOS, MANEUVER_START_NANOS, Maneuver, ScModel, reference_sigma_rn, run,
        test_cases,
    };
    use basilisk_rs::utilities::rigid_body_kinematics::{mrp_to_dcm, sub_mrp};
    use nalgebra::Vector3;

    /// `test_scenarioConstraintDynamicsManeuverAnalysis.py`: both maneuvers
    /// with gains 1e1, 1e2, 1e3, `alongtrackahead`, `LEO`, `MEV2` run to
    /// completion. What the scenario demonstrates, checked here: the mean
    /// violations fall as the gain rises (upstream's gain-performance
    /// plot); the attitude maneuver brings both hubs to the commanded
    /// 180° attitude; the orbit maneuver's 300 N s push goes into the
    /// docked pair's momentum (the accumulated non-gravitational velocity
    /// change), since the constraint forces cancel.
    #[test]
    fn maneuvers_of_the_docked_pair() {
        let model = ScModel::Mev2.data();
        for config in test_cases() {
            let label = config.label();
            let results = run(&config);
            assert_eq!(results.runs.len(), 3, "{label}");
            for pair in results.runs.windows(2) {
                for (stiff, soft, what) in [
                    (
                        pair[1].violations.mean_direction_m(),
                        pair[0].violations.mean_direction_m(),
                        "direction",
                    ),
                    (
                        pair[1].violations.mean_attitude_deg(),
                        pair[0].violations.mean_attitude_deg(),
                        "attitude",
                    ),
                ] {
                    assert!(
                        stiff < soft,
                        "{label}: mean {what} violation {stiff:e} at gain {} not below {soft:e} \
                         at gain {}",
                        pair[1].gain,
                        pair[0].gain
                    );
                }
            }
            for gain_run in &results.runs {
                assert_eq!(gain_run.states_1.len(), 12_001, "{label}");
                let (_, chaser) = gain_run.states_1.last().expect("samples");
                let (_, target) = gain_run.states_2.last().expect("samples");
                match config.maneuver {
                    Maneuver::Attitude => {
                        for (name, state) in [("chaser", chaser), ("target", target)] {
                            let error = sub_mrp(state.sigma_bn, reference_sigma_rn()).norm();
                            assert!(
                                error < 0.02,
                                "{label} gain {}: {name} attitude error |sigma_BR| {error}",
                                gain_run.gain
                            );
                        }
                    }
                    Maneuver::Orbit => {
                        let momentum = model.chaser_mass_kg * chaser.accumulated_dv_cn_n_mps
                            + model.target_mass_kg * target.accumulated_dv_cn_n_mps;
                        // The 1 N body-frame push, rotated with the chaser:
                        // the docked dumbbell turns slowly under the gravity
                        // gradient. Trapezoidal over each 0.1 s step.
                        let force_b = -gain_run.geometry.r_p2p1_b1_init_m.normalize();
                        let push: Vector3<f64> = gain_run
                            .states_1
                            .windows(2)
                            .filter(|step| {
                                step[1].0 > MANEUVER_START_NANOS && step[1].0 <= MANEUVER_END_NANOS
                            })
                            .map(|step| {
                                let dcm_nb = |sigma| mrp_to_dcm(sigma).transpose();
                                0.5 * (dcm_nb(step[0].1.sigma_bn) + dcm_nb(step[1].1.sigma_bn))
                                    * force_b
                                    * 0.1
                            })
                            .sum();
                        assert!(
                            (momentum - push).norm() < 1.0e-4 * push.norm(),
                            "{label} gain {}: momentum change {momentum:?}, push {push:?}",
                            gain_run.gain
                        );
                    }
                }
            }
        }
    }
}
