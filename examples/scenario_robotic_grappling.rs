//! Port of upstream `examples/scenarioRoboticGrappling.py`: docking and
//! berthing through effector branching. A 750 kg chaser carries a 20 kg arm,
//! a [`LinearTranslationOneDofStateEffector`] extended 1 m along x̂ on a
//! spring–damper (`k = 100` N/m, `c = 50` N s/m). A
//! [`ConstraintDynamicEffector`] grapples the arm tip (the constraint's end 1
//! is attached to the arm, upstream `translatingBody.addDynamicEffector`) to a
//! point on the Marman ring of a 750 kg target turned 90° about ŷ, which is
//! off the target's axis. Both spacecraft are integrated together with RKF45.
//!
//! The run has three phases:
//! 1. **Free-flight approach:** the chaser drifts at 0.02 m/s toward the
//!    target with the constraint off; every 0.05 s the tip-to-grapple
//!    distance is checked, and once within 5 mm the constraint is switched
//!    on (upstream's terminal capture event).
//! 2. **Settle:** 30 s at the arm's initial extension.
//! 3. **Berthing:** the arm's reference is profiled to zero
//!    ([`PrescribedLinearTranslation`], smoothed bang-bang over 30 s) and the
//!    arm draws the target in.
//!
//! | option | upstream `run()` argument | default | `--test` |
//! |---|---|---|---|
//! | `--gain` | `gain` (`alpha = beta`) | `1e4` | `1e4` |
//! | `--dyn-rate` | `dynRateSeconds` \[s\] | `1e-4` | `0.01` |
//! | `--approach-gap` | `approachGapMeters` \[m\] | `1.0` | `0.2` |
//! | `--settle` | `settleDurationSeconds` \[s\] | `30` | `30` |
//! | `--berth` | `berthDurationSeconds` \[s\] | `90` | `45` |
//! | `--damping-scale` | `constraintLinearDampingScale` | `1.0` | `0.25` |
//!
//! The defaults are upstream's journal-paper configuration (a 0.1 ms step,
//! minutes of wall time); `--test` is the coarser configuration of upstream's
//! `test_scenarioRoboticGrappling.py`. The constraint's damping is set for
//! critical damping at the pair's reduced mass (`c_d`, scaled) and the
//! target's smallest inertia (`c_a`), as upstream sets it.
//!
//! `scenarioRoboticGrappling.csv` (`scenarioRoboticGrapplingTest.csv` with
//! `--test`, `...Custom.csv` otherwise) holds both hub states (`r_B1N_N_m_i`,
//! `sigma_B1N_i`, `r_B2N_N_m_i`, `sigma_B2N_i`) and the arm's `rho_m` and
//! `rhoDot_m_per_s` every 0.1 s; `..._events.csv` holds the capture time and
//! distance and the start of the retraction.
//! `plot_scenario_robotic_grappling.py` draws upstream's `HubSeparation`,
//! `ArmExtension`, `PositionConstraint` and `AttitudeConstraint` figures.
//! Upstream's Vizard setup is not ported.
//!
//! Upstream's `test_scenarioRoboticGrappling.py` runs the coarse
//! configuration and checks only that it completes. `tests` runs the same
//! and checks what upstream's description of the figures says: the grapple
//! engages; the hub separation holds near 3.2 m after capture and draws down
//! to about 2.2 m as the arm retracts; the constraint pulls the position
//! violation to near zero; the attitude violation stays at numerical zero.

#[path = "support/common.rs"]
mod common;
#[path = "support/constrained.rs"]
mod constrained;
#[path = "support/hub_init.rs"]
mod hub_init;
#[path = "support/prescribed.rs"]
mod prescribed;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/scenario_args.rs"]
mod scenario_args;
#[path = "support/table_csv.rs"]
mod table_csv;

use basilisk_rs::device_interface::prescribed_linear_translation::PrescribedLinearTranslation;
use basilisk_rs::dynamics::constraint_dynamic_effector::{
    ConstraintDynamicEffector, ConstraintDynamicEffectorConfig,
};
use basilisk_rs::dynamics::linear_translation_one_dof_state_effector::{
    LinearTranslationOneDofConfig, LinearTranslationOneDofStateEffector,
};
use basilisk_rs::messages::{
    DeviceState, DeviceStatusMsg, LinearTranslationRigidBodyMsg, Output, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::SpacecraftGroup;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::rigid_body_kinematics::{mrp_to_dcm, sub_mrp};
use basilisk_rs::{connect, schedule};
use constrained::{
    hub, hub_pose_columns, model_inertia_kg_m2, upstream_rkf45, violation_angles_deg,
};
use hifitime::Epoch;
use hub_init::set_center_of_mass_initial_state;
use nalgebra::Vector3;
use prescribed::translation_profiler;
use recording::{History, scenario_csv};
use scenario_args::ScenarioArgs;
use table_csv::write_table;

const USAGE: &str = "\
scenario_robotic_grappling [--gain 1e4] [--dyn-rate 1e-4] [--approach-gap 1.0] [--settle 30]
    [--berth 90] [--damping-scale 1.0]
scenario_robotic_grappling --test     upstream's test configuration";

const STEM: &str = "scenarioRoboticGrappling";
/// Upstream `approachTimeout` \[s\].
const APPROACH_TIMEOUT_S: f64 = 80.0;
/// Upstream `retractionDuration` \[s\].
const RETRACTION_DURATION_S: f64 = 30.0;
/// Upstream `captureEventRate` \[s\].
const CAPTURE_CHECK_S: f64 = 0.05;
/// Upstream `logData`'s `samplingTime` \[s\].
const SAMPLE_S: f64 = 0.1;
/// Upstream `rArmHub` \[m\]: chaser body origin to the arm root.
const ARM_HUB_M: f64 = 1.0;
/// Upstream `rArmInit` \[m\]: initial arm extension.
const ARM_INIT_M: f64 = 1.0;
/// Upstream `rArmFinal` \[m\]: commanded extension at the end of berthing.
const ARM_FINAL_M: f64 = 0.0;
/// Upstream `rTargetTip` \[m\]: target hub to its Marman-ring face.
const TARGET_TIP_M: f64 = 1.0;
/// Upstream `marmanRingRadius` \[m\]: grapple offset from the target's axis.
const MARMAN_RING_RADIUS_M: f64 = 0.75;
/// Upstream `marmanRingHeight` \[m\].
const MARMAN_RING_HEIGHT_M: f64 = 0.1;
/// Upstream `approachSpeed` \[m/s\].
const APPROACH_SPEED_MPS: f64 = 0.02;
/// Upstream `captureTolerance` \[m\].
const CAPTURE_TOLERANCE_M: f64 = 5.0e-3;
/// Upstream `armMass` \[kg\].
const ARM_MASS_KG: f64 = 20.0;
/// Upstream `retractionSmoothingDuration` \[s\].
const RETRACTION_SMOOTHING_S: f64 = 1.0;
/// Upstream `mHub` of both spacecraft \[kg\].
const HUB_MASS_KG: f64 = 750.0;

/// Upstream `run(show_plots, gain, dynRateSeconds, approachGapMeters,
/// settleDurationSeconds, berthDurationSeconds, constraintLinearDampingScale)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    pub gain: f64,
    pub dyn_rate_s: f64,
    pub approach_gap_m: f64,
    pub settle_s: f64,
    pub berth_s: f64,
    pub damping_scale: f64,
}

impl Config {
    /// Upstream's defaults, the journal-paper configuration.
    pub const UPSTREAM: Self = Self {
        gain: 1.0e4,
        dyn_rate_s: 1.0e-4,
        approach_gap_m: 1.0,
        settle_s: 30.0,
        berth_s: 90.0,
        damping_scale: 1.0,
    };

    /// `test_scenarioRoboticGrappling.py`'s configuration.
    pub const TEST: Self = Self {
        gain: 1.0e4,
        dyn_rate_s: 0.01,
        approach_gap_m: 0.2,
        settle_s: 30.0,
        berth_s: 45.0,
        damping_scale: 0.25,
    };

    fn file_stem(&self) -> String {
        let variant = if *self == Self::UPSTREAM {
            ""
        } else if *self == Self::TEST {
            "Test"
        } else {
            "Custom"
        };
        format!("{STEM}{variant}")
    }
}

/// Upstream `macros.sec2nano`.
fn nanos(seconds: f64) -> u64 {
    (seconds * 1.0e9).round() as u64
}

pub struct Results {
    pub csv_path: std::path::PathBuf,
    pub states_1: Vec<(u64, SpacecraftStateMsg)>,
    pub states_2: Vec<(u64, SpacecraftStateMsg)>,
    /// Upstream `armLog.rho` \[m\].
    pub rho_m: Vec<f64>,
    /// Upstream `captureTimeNanos`.
    pub capture_nanos: u64,
    /// Upstream `captureDistance` \[m\].
    pub capture_distance_m: f64,
    /// Upstream `hubSeparation` \[m\].
    pub hub_separation_m: Vec<f64>,
    /// Upstream `psi_B1` \[m\], from the current arm tip.
    pub position_violation_m: Vec<Vector3<f64>>,
    /// Upstream `sigmaB2B1`: the deviation from the prescribed relative
    /// attitude.
    pub attitude_violation: Vec<Vector3<f64>>,
}

pub fn run(config: &Config) -> Results {
    let step_nanos = nanos(config.dyn_rate_s);
    let inertia = model_inertia_kg_m2();
    let mut chaser = hub(HUB_MASS_KG, Vector3::zeros(), inertia, step_nanos);
    let mut target = hub(HUB_MASS_KG, Vector3::zeros(), inertia, step_nanos);

    // Upstream `defineInitialConditions`: collinear, the grapple
    // `approachGap` from the arm tip.
    let r_hat = Vector3::x();
    let r_f0b_b = ARM_HUB_M * r_hat;
    let f_hat_b = r_hat;
    let sigma_b2n_init = Vector3::new(0.0, (std::f64::consts::PI / 8.0).tan(), 0.0);
    let dcm_nb2_init = mrp_to_dcm(sigma_b2n_init).transpose();
    let r_p2b2_b2 = Vector3::new(
        0.0,
        MARMAN_RING_RADIUS_M,
        -(TARGET_TIP_M + MARMAN_RING_HEIGHT_M),
    );
    let arm_tip_b1_init = r_f0b_b + ARM_INIT_M * f_hat_b;
    let chaser_mass_kg = HUB_MASS_KG + ARM_MASS_KG;
    let target_mass_kg = HUB_MASS_KG;
    let chaser_com_from_hub = ARM_MASS_KG * arm_tip_b1_init / chaser_mass_kg;
    let r_c2c1 = arm_tip_b1_init - dcm_nb2_init * r_p2b2_b2 - chaser_com_from_hub
        + config.approach_gap_m * r_hat;
    let total_mass_kg = chaser_mass_kg + target_mass_kg;
    let r_c1c = -(target_mass_kg * r_c2c1) / total_mass_kg;
    let r_c2c = r_c1c + r_c2c1;

    // Upstream `setUpTranslationEffector`: the arm and its profiler.
    let mut arm = LinearTranslationOneDofStateEffector::new(LinearTranslationOneDofConfig {
        mass_kg: ARM_MASS_KG,
        spring_constant_n_per_m: 100.0,
        damping_n_s_per_m: 50.0,
        rho_init_m: ARM_INIT_M,
        rho_dot_init_mps: 0.0,
        axis_body: f_hat_b,
        center_of_mass_translating_m: Vector3::zeros(),
        origin0_body_m: r_f0b_b,
        inertia_about_com_translating_kg_m2: nalgebra::Matrix3::from_diagonal(&Vector3::new(
            50.0, 80.0, 60.0,
        )),
        ..LinearTranslationOneDofConfig::new("arm")
    });
    let translation_command = Output::new(LinearTranslationRigidBodyMsg {
        rho_m: ARM_INIT_M,
        rho_dot_mps: 0.0,
    });
    let mut profiler: PrescribedLinearTranslation = translation_profiler(
        "armRetractProfiler",
        f_hat_b,
        4.0 * (ARM_FINAL_M - ARM_INIT_M).abs() / (RETRACTION_DURATION_S * RETRACTION_DURATION_S),
        ARM_INIT_M,
        0.0,
        RETRACTION_SMOOTHING_S,
    );
    translation_command.connect_to(&mut profiler.linear_translation_rigid_body_in_msg);
    profiler
        .linear_translation_rigid_body_out_msg
        .connect_to(&mut arm.translating_body_ref_in);
    let arm_out = arm.translating_body_out.clone();
    let arm_tip_out = arm.translating_body_state_out.clone();
    chaser.add_state_effector(arm);

    set_center_of_mass_initial_state(&mut chaser, r_c1c, APPROACH_SPEED_MPS * r_hat);
    target.config.initial_sigma_bn = sigma_b2n_init;
    set_center_of_mass_initial_state(&mut target, r_c2c, Vector3::zeros());

    // Upstream `setUpConstraintEffector`: the grapple on the arm tip,
    // critically damped at the reduced mass and the smallest target inertia.
    let mut constraint = ConstraintDynamicEffector::new(ConstraintDynamicEffectorConfig {
        connection_point_1_m: Vector3::zeros(),
        connection_point_2_m: r_p2b2_b2,
        separation_m: Vector3::zeros(),
        // Lock the relative attitude at its initial 90°.
        sigma_b2b1: sigma_b2n_init,
        parent_effectors: [Some(0), None],
        ..ConstraintDynamicEffectorConfig::new("grapple")
    });
    constraint.set_alpha(config.gain).expect("positive gain");
    constraint.set_beta(config.gain).expect("positive gain");
    let reduced_mass_kg = chaser_mass_kg * target_mass_kg / total_mass_kg;
    let target_inertia_min = inertia.diagonal().min();
    let k = config.gain * config.gain;
    constraint
        .set_c_d(config.damping_scale * 2.0 * (k * reduced_mass_kg).sqrt())
        .expect("positive damping");
    constraint
        .set_c_a(2.0 * (k * target_inertia_min).sqrt())
        .expect("positive damping");
    let constraint_status = Output::new(DeviceStatusMsg {
        device_status: DeviceState::Off,
    });
    constraint_status.connect_to(&mut constraint.effector_status_in);
    constraint.effector_status = DeviceState::Off;

    let mut group = SpacecraftGroup::new(vec![chaser, target]);
    group.integrator = Some(Box::new(upstream_rkf45()));
    group.add_coupling_effector(constraint);

    let file_stem = config.file_stem();
    let mut csv = scenario_csv(&file_stem);
    let csv_path = csv.config.output_path.clone();
    let sample_nanos = nanos(SAMPLE_S);
    let mut history_1 = History::<SpacecraftStateMsg>::new();
    let mut history_2 = History::<SpacecraftStateMsg>::new();
    let mut arm_log = History::<LinearTranslationRigidBodyMsg>::new();
    let [state_1, state_2] = [0, 1].map(|index| group.spacecraft()[index].state_out.clone());
    // Upstream `computeDockingDistance`.
    let docking_distance_m = || {
        let target = state_2.read();
        let grapple_n = target.position_m + mrp_to_dcm(target.sigma_bn).transpose() * r_p2b2_b2;
        (grapple_n - arm_tip_out.read().position_m).norm()
    };
    let capture_nanos;
    let capture_distance_m;
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 11, 14), false);
        connect!(&simulation,
            &state_1 => csv.add_source::<SpacecraftStateMsg>(hub_pose_columns("B1")),
            &state_2 => csv.add_source::<SpacecraftStateMsg>(hub_pose_columns("B2")),
            &arm_out => csv.add_source::<LinearTranslationRigidBodyMsg>(CsvSourceConfig::columns([
                ("rho_m", "rho_m"),
                ("rho_dot_mps", "rhoDot_m_per_s"),
            ])),
            &state_1 => &mut history_1.input,
            &state_2 => &mut history_2.input,
            &arm_out => &mut arm_log.input,
        );
        schedule! { simulation,
            "spacecraftGroup" => &mut group, period=step_nanos, priority=100;
            "armRetractProfiler" => &mut profiler, period=step_nanos, priority=50;
            "csv" => &mut csv, period=sample_nanos, priority=0;
            "datLog1" => &mut history_1, period=sample_nanos, priority=0;
            "datLog2" => &mut history_2, period=sample_nanos, priority=0;
            "armLog" => &mut arm_log, period=sample_nanos, priority=0;
        }

        // Phase 1: free-flight approach, the capture event checked every
        // 0.05 s; it engages the grapple and ends the phase.
        let check_nanos = nanos(CAPTURE_CHECK_S);
        while docking_distance_m() > CAPTURE_TOLERANCE_M {
            assert!(
                simulation.current_sim_nanos() < nanos(APPROACH_TIMEOUT_S),
                "Grapple capture event did not occur before approach timeout."
            );
            simulation.run_for(check_nanos);
        }
        constraint_status.write(DeviceStatusMsg {
            device_status: DeviceState::On,
        });
        capture_nanos = simulation.current_sim_nanos();
        capture_distance_m = docking_distance_m();

        // Phase 2: settle at the initial arm extension.
        let settle_stop_nanos = capture_nanos + nanos(config.settle_s);
        simulation.run_for(settle_stop_nanos - capture_nanos);

        // Phase 3: retract the arm; the constraint draws the target in.
        translation_command.write(LinearTranslationRigidBodyMsg {
            rho_m: ARM_FINAL_M,
            rho_dot_mps: 0.0,
        });
        simulation.run_for(nanos(config.berth_s));
    }
    csv.flush().expect("flush scenario CSV");

    let retraction_start_s = (capture_nanos as f64) * 1.0e-9 + config.settle_s;
    write_table(
        &format!("{file_stem}_events"),
        &[
            "capture_time_s".to_string(),
            "capture_distance_m".to_string(),
            "retraction_start_s".to_string(),
        ],
        &[vec![
            capture_nanos as f64 * 1.0e-9,
            capture_distance_m,
            retraction_start_s,
        ]],
    );

    // Upstream `processData`.
    let states_1 = history_1.into_samples();
    let states_2 = history_2.into_samples();
    let rho_m: Vec<f64> = arm_log.messages().map(|arm| arm.rho_m).collect();
    let mut hub_separation_m = Vec::new();
    let mut position_violation_m = Vec::new();
    let mut attitude_violation = Vec::new();
    for (((_, state_1), (_, state_2)), rho) in states_1.iter().zip(&states_2).zip(&rho_m) {
        hub_separation_m.push((state_2.position_m - state_1.position_m).norm());
        let dcm_b1n = mrp_to_dcm(state_1.sigma_bn);
        let dcm_nb2 = mrp_to_dcm(state_2.sigma_bn).transpose();
        let arm_tip_b1 = r_f0b_b + *rho * f_hat_b;
        position_violation_m.push(
            dcm_b1n * state_1.position_m + arm_tip_b1
                - (dcm_b1n * state_2.position_m + dcm_b1n * dcm_nb2 * r_p2b2_b2),
        );
        attitude_violation.push(sub_mrp(
            sub_mrp(state_2.sigma_bn, state_1.sigma_bn),
            sigma_b2n_init,
        ));
    }
    Results {
        csv_path,
        states_1,
        states_2,
        rho_m,
        capture_nanos,
        capture_distance_m,
        hub_separation_m,
        position_violation_m,
        attitude_violation,
    }
}

impl Results {
    /// The largest attitude violation \[deg\] (upstream's plotted
    /// `|4 arctan(sigmaB2B1)|`).
    pub fn max_attitude_violation_deg(&self) -> f64 {
        self.attitude_violation
            .iter()
            .map(|sigma| violation_angles_deg(*sigma).norm())
            .fold(0.0, f64::max)
    }
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let config = if args.flag("--test") {
        Config::TEST
    } else {
        let defaults = Config::UPSTREAM;
        Config {
            gain: args.value("--gain", defaults.gain),
            dyn_rate_s: args.value("--dyn-rate", defaults.dyn_rate_s),
            approach_gap_m: args.value("--approach-gap", defaults.approach_gap_m),
            settle_s: args.value("--settle", defaults.settle_s),
            berth_s: args.value("--berth", defaults.berth_s),
            damping_scale: args.value("--damping-scale", defaults.damping_scale),
        }
    };
    let results = run(&config);
    println!(
        "wrote {} (capture at {:.2} s, {:.2e} m; final hub separation {:.3} m, arm {:.3} m)",
        results.csv_path.display(),
        results.capture_nanos as f64 * 1.0e-9,
        results.capture_distance_m,
        results.hub_separation_m.last().expect("samples"),
        results.rho_m.last().expect("samples"),
    );
}

#[cfg(test)]
mod tests {
    use super::{CAPTURE_TOLERANCE_M, Config, run};

    /// `test_scenarioRoboticGrappling.py`: the coarse configuration (0.01 s
    /// step, 0.2 m gap, 45 s berth, 25 % linear damping) runs to
    /// completion. Upstream's description of the figures, checked here: the
    /// grapple engages after about 10 s of drift; the hubs hold near 3.2 m
    /// apart after capture and end about 2.2 m apart, the arm retracted;
    /// the position violation is pulled below 0.1 mm once engaged; the
    /// attitude violation stays below 1e-4 deg.
    #[test]
    fn the_arm_berths_the_grappled_target() {
        let config = Config::TEST;
        let results = run(&config);
        let capture_s = results.capture_nanos as f64 * 1.0e-9;
        assert!((5.0..15.0).contains(&capture_s), "capture at {capture_s} s");
        assert!(results.capture_distance_m <= CAPTURE_TOLERANCE_M);

        // Samples every 0.1 s; the last of the settle phase.
        let settled = ((capture_s + config.settle_s) / 0.1).round() as usize;
        let settled_separation = results.hub_separation_m[settled];
        assert!(
            (settled_separation - 3.2).abs() < 0.05,
            "separation after capture {settled_separation} m"
        );
        let final_separation = *results.hub_separation_m.last().expect("samples");
        assert!(
            (final_separation - 2.2).abs() < 0.05,
            "final separation {final_separation} m"
        );
        let final_rho = *results.rho_m.last().expect("samples");
        assert!(final_rho.abs() < 0.02, "final arm extension {final_rho} m");

        let violation = |index: usize| results.position_violation_m[index].norm();
        assert!(violation(settled) < 1.0e-4, "{:e} m", violation(settled));
        let last = results.position_violation_m.len() - 1;
        assert!(violation(last) < 1.0e-4, "{:e} m", violation(last));
        let attitude_deg = results.max_attitude_violation_deg();
        assert!(
            attitude_deg < 1.0e-4,
            "attitude violation {attitude_deg:e} deg"
        );
    }
}
