//! Port of upstream
//! `examples/dynamicsComparison/scenarioCompareVariableMass.py`: a 15-minute
//! prograde orbit-raising burn of a MOOG Monarc-445 that drains a spherical
//! hydrazine tank with sloshing propellant (three spring-mass-damper
//! fixtures and a spherical-pendulum first slosh mode), so the mass, the
//! tank inertia and the system center of mass all change, propagated by
//! the Backsubstitution spacecraft and, with the `mujoco` feature, by an
//! `MjScene` whose bodies deplete through their mass-rate inputs (see
//! `support/dyncomp_variable_mass.rs`). The vehicle starts velocity
//! aligned at the orbital pitch rate and then flies passively. RK4 at
//! `min(0.05 s, slosh period / 150)`.
//!
//! In orbit the engines differ by where gravity acts (BSM once at the
//! system center of mass, MuJoCo at every body); the final semi-major axes
//! differ by about 0.34 % of the ~243 km rise. `--deep-space` (upstream
//! `inOrbit=False`) burns from rest without gravity, isolating the shared
//! instantaneous-property depletion convention; `--near-rigid` collapses
//! the slosh to negligible masses there; `--leak` (`useThruster=False`)
//! drains the tank at the engine's mass flow without thrust; `--duration`
//! sets the burn \[s\]. The MuJoCo run starts from the BSM hub-origin state
//! at `t = 0`.
//!
//! Outputs, in `examples/output/scenarios/`:
//! `scenarioCompareVariableMass[_deepSpace[_rigid]].csv` (the orbit or burn
//! speed, masses, rates, slosh displacements and differences every step,
//! for `plot_scenario_compare_variable_mass.py`), the matching `.json` and,
//! with `--timing`, `scenarioCompareVariableMass_runtime.csv`. Upstream's
//! `saveReference` `.npz` of the BSM histories is the CSV's `bsm_*`
//! columns.
//!
//! `tests` ports upstream's variable-mass checks: `_assert_variable_mass_orbit_metrics`,
//! `test_variable_mass_rejects_nondivisible_horizon`,
//! `test_variable_mass_leak_depletes_without_external_impulse`,
//! `test_variable_mass_thruster_burn_covers_requested_horizon` and, with
//! the `mujoco` feature, `test_variable_mass_deep_space_numerical_agreement`
//! and the two native pendulum-damping tests.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/dyncomp.rs"]
mod dyncomp;
#[path = "support/dyncomp_orbit.rs"]
mod dyncomp_orbit;
#[path = "support/dyncomp_variable_mass.rs"]
mod dyncomp_variable_mass;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/multibody.rs"]
mod multibody;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/table_csv.rs"]
mod table_csv;
#[path = "support/thruster_factory.rs"]
mod thruster_factory;

use cli::Cli;
use dyncomp::{DEFAULT_TIMING_REPEATS, Series, save_runtime_table, seconds, validate_history};
use dyncomp_variable_mass::{
    PEND_BOB_INERTIA, SIM_DURATION_S, VariableMassCase, VariableMassHistory,
    initial_thrust_torque_about_com, nominal_mass_flow, run_bsm, slosh_parameters, thruster_spec,
    time_step,
};
use nalgebra::Vector3;
use serde_json::{Value, json};

const FILE_NAME: &str = "scenarioCompareVariableMass";

const USAGE: &str = "\
scenario_compare_variable_mass [--deep-space] [--leak] [--near-rigid] [--duration S] [--timing]
  --deep-space   burn from rest without gravity (upstream inOrbit=False)
  --leak         drain the tank without thrust (upstream useThruster=False)
  --near-rigid   negligible slosh masses (deep space only)
  --duration     burn and comparison window [s] (default 900)
  --timing       also time both engines and write scenarioCompareVariableMass_runtime.csv";

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub case: VariableMassCase,
    /// Upstream `saveTiming`.
    pub save_timing: bool,
}

impl Config {
    pub fn new(use_thruster: bool, in_orbit: bool, sim_duration_s: f64) -> Self {
        Self {
            case: VariableMassCase::new(use_thruster, in_orbit, sim_duration_s),
            save_timing: false,
        }
    }
}

pub struct Results {
    /// Upstream's JSON metrics, by upstream key.
    pub metrics: Value,
    pub bsm: VariableMassHistory,
    pub mujoco: Option<VariableMassHistory>,
    pub csv_path: std::path::PathBuf,
}

impl Results {
    pub fn metric(&self, key: &str) -> f64 {
        self.metrics[key]
            .as_f64()
            .unwrap_or_else(|| panic!("no metric {key}"))
    }
}

fn validated(
    name: &str,
    history: &VariableMassHistory,
    case: &VariableMassCase,
    dt: f64,
) -> Vec<f64> {
    let rho: Vec<Vec<f64>> = history.rho.iter().map(|r| r.to_vec()).collect();
    validate_history(
        name,
        &seconds(&history.times_nanos),
        Some(case.sim_duration_s),
        dt,
        true,
        &[
            Series::vectors("attitude", &history.sigma_bn),
            Series::vectors("rate", &history.omega_bn_b),
            Series::vectors("position", &history.r_cn_n),
            Series::scalars("mass", &history.total_mass),
            Series::rows("slosh", &rho),
            Series::scalars("pendulumPhi", &history.phi),
            Series::scalars("pendulumTheta", &history.theta),
        ],
    )
    .unwrap_or_else(|error| panic!("{error}"))
}

fn speed_change(history: &VariableMassHistory) -> Vec<f64> {
    let v0 = history.v_cn_n[0].norm();
    history.v_cn_n.iter().map(|v| v.norm() - v0).collect()
}

pub fn run(config: Config) -> Results {
    let case = config.case;
    assert!(
        !(case.near_rigid && case.in_orbit),
        "The near-rigid control is defined only in deep space."
    );
    let dt = time_step();
    let spec = thruster_spec();
    let slosh = slosh_parameters(case.near_rigid);

    let (bsm, _) = run_bsm(&case, dt, true);
    let time_s = validated("variable-mass BSM", &bsm, &case, dt);
    let mut stem = if case.in_orbit {
        FILE_NAME.to_string()
    } else {
        format!("{FILE_NAME}_deepSpace")
    };
    if case.near_rigid {
        stem.push_str("_rigid");
    }
    let torque = if case.use_thruster {
        initial_thrust_torque_about_com(spec.max_thrust_n, case.near_rigid)
    } else {
        Vector3::zeros()
    };
    let last = time_s.len() - 1;
    let mut metrics = json!({
        "scenario": stem,
        "timeStep": dt,
        "simDuration": case.sim_duration_s,
        "useThruster": case.use_thruster,
        "inOrbit": case.in_orbit,
        "nearRigid": case.near_rigid,
        "thruster": "MOOG_Monarc_445",
        "maxThrust": spec.max_thrust_n,
        "steadyIsp": spec.steady_isp_s,
        "nominalMassFlow": nominal_mass_flow(spec.max_thrust_n, spec.steady_isp_s),
        "initialThrustTorqueAboutCoM": [torque.x, torque.y, torque.z],
        "initialThrustTorqueMagnitude": torque.norm(),
        "initialPendulumBobInertia": PEND_BOB_INERTIA,
        "initialPendulumMass": slosh.pend_mass,
        "initialSmdMass": slosh.smd_mass,
        "propellantDepletedFraction": 1.0 - bsm.fuel_mass[last] / bsm.fuel_mass[0],
        "wetMassChangeFraction": 1.0 - bsm.total_mass[last] / bsm.total_mass[0],
        "deltaV": bsm.v_cn_n[last].norm() - bsm.v_cn_n[0].norm(),
    });
    if case.in_orbit {
        metrics["semiMajorAxisRise"] = json!(bsm.semi_major_axis[last] - bsm.semi_major_axis[0]);
    }

    let mut headers = vec!["time_ns".to_string()];
    let mut columns: Vec<Vec<f64>> = vec![bsm.times_nanos.iter().map(|t| *t as f64).collect()];
    let push = |headers: &mut Vec<String>,
                columns: &mut Vec<Vec<f64>>,
                engine: &str,
                h: &VariableMassHistory| {
        if case.in_orbit {
            headers.push(format!("{engine}_sma_rise_m"));
            columns.push(
                h.semi_major_axis
                    .iter()
                    .map(|a| a - h.semi_major_axis[0])
                    .collect(),
            );
        } else {
            headers.push(format!("{engine}_burn_speed_mps"));
            columns.push(speed_change(h));
        }
        headers.push(format!("{engine}_total_mass_kg"));
        columns.push(h.total_mass.clone());
        for axis in 0..3 {
            headers.push(format!("{engine}_omega_BN_B_rad_per_s_{axis}"));
            columns.push(h.omega_bn_b.iter().map(|w| w[axis]).collect());
        }
        for axis in 0..3 {
            headers.push(format!("{engine}_rho_m_{axis}"));
            columns.push(h.rho.iter().map(|r| r[axis]).collect());
        }
    };
    push(&mut headers, &mut columns, "bsm", &bsm);

    #[cfg(feature = "mujoco")]
    let mujoco = {
        let (mujoco, _) = dyncomp_variable_mass::run_mujoco(&case, dt, true, bsm.initial_state());
        let earth_mu = dyncomp_variable_mass::earth_mu;
        let mujoco_s = validated("variable-mass MuJoCo", &mujoco, &case, dt);
        dyncomp::validate_matching_histories(
            "variable-mass BSM/MuJoCo",
            &time_s,
            &mujoco_s,
            Some(case.sim_duration_s),
            dt,
            true,
        )
        .unwrap_or_else(|error| panic!("{error}"));
        let att_error = dyncomp::relative_principal_angles(
            "variable-mass attitude",
            &bsm.sigma_bn,
            &mujoco.sigma_bn,
        );
        let com_error = dyncomp::vector_differences(&bsm.r_cn_n, &mujoco.r_cn_n);
        let wrap = |a: f64, b: f64| (a - b).sin().atan2((a - b).cos()).abs();
        metrics["attitudeErrorMax"] = json!(dyncomp::max_of(att_error.iter().copied()));
        metrics["comPositionErrorMax"] = json!(dyncomp::max_of(com_error.iter().copied()));
        metrics["rateErrorMax"] = json!(dyncomp::max_of(dyncomp::vector_differences(
            &bsm.omega_bn_b,
            &mujoco.omega_bn_b
        )));
        metrics["totalMassErrorMax"] = json!(dyncomp::max_of(
            bsm.total_mass
                .iter()
                .zip(&mujoco.total_mass)
                .map(|(a, b)| (a - b).abs())
        ));
        metrics["sloshDisplacementErrorMax"] = json!(dyncomp::max_of(
            bsm.rho
                .iter()
                .zip(&mujoco.rho)
                .flat_map(|(a, b)| (0..3).map(move |i| (a[i] - b[i]).abs()))
        ));
        metrics["pendulumAngleErrorMax"] =
            json!(dyncomp::max_of((0..bsm.phi.len()).flat_map(|i| [
                wrap(bsm.phi[i], mujoco.phi[i]),
                wrap(bsm.theta[i], mujoco.theta[i])
            ])));
        if case.in_orbit {
            let rise = bsm.semi_major_axis[last] - bsm.semi_major_axis[0];
            let difference = (bsm.semi_major_axis[last] - mujoco.semi_major_axis[last]).abs();
            metrics["gravityGradientRateEstimate"] = json!(
                dyncomp_variable_mass::gravity_gradient_rate_estimate(earth_mu())
            );
            metrics["semiMajorAxisRiseMujoco"] =
                json!(mujoco.semi_major_axis[last] - mujoco.semi_major_axis[0]);
            metrics["semiMajorAxisDifferenceFinal"] = json!(difference);
            metrics["semiMajorAxisDifferenceAsFractionOfRise"] = json!(difference / rise.abs());
        }
        push(&mut headers, &mut columns, "mujoco", &mujoco);
        headers.extend(["att_error_rad".to_string(), "com_error_m".to_string()]);
        columns.extend([att_error, com_error]);
        Some(mujoco)
    };
    #[cfg(not(feature = "mujoco"))]
    let mujoco: Option<VariableMassHistory> = None;

    if config.save_timing {
        let build_bsm = || run_bsm(&case, dt, false).1;
        #[cfg(feature = "mujoco")]
        let (bsm_s, mujoco_s) = {
            let initial = bsm.initial_state();
            let build_mujoco =
                move || dyncomp_variable_mass::run_mujoco(&case, dt, false, initial).1;
            let (bsm, mujoco) = dyncomp::paired_propagation_times(
                Box::new(build_bsm),
                Box::new(build_mujoco),
                DEFAULT_TIMING_REPEATS,
            );
            (bsm, Some(mujoco))
        };
        #[cfg(not(feature = "mujoco"))]
        let (bsm_s, mujoco_s) = (
            dyncomp::median_propagation_time(build_bsm, DEFAULT_TIMING_REPEATS),
            None,
        );
        let regime = if case.in_orbit { "orbit" } else { "deep space" };
        let label = format!(
            "Variable-mass hub + slosh ({} s, RK4 dt={dt} s), {regime}",
            case.sim_duration_s
        );
        let path = save_runtime_table(FILE_NAME, &[(label, bsm_s, mujoco_s)], "Case");
        println!("wrote {}", path.display());
    }

    let rows: Vec<Vec<f64>> = (0..time_s.len())
        .map(|index| columns.iter().map(|column| column[index]).collect())
        .collect();
    let csv_path = table_csv::write_table(&stem, &headers, &rows);
    dyncomp::write_json(&stem, &metrics);
    Results {
        metrics,
        bsm,
        mujoco,
        csv_path,
    }
}

fn main() {
    let mut args = Cli::from_env(USAGE);
    let mut config = Config::new(
        !args.switch("leak"),
        !args.switch("deep-space"),
        args.value("duration", SIM_DURATION_S),
    );
    config.case.near_rigid = args.switch("near-rigid");
    if config.case.near_rigid && config.case.in_orbit {
        args.fail("--near-rigid needs --deep-space");
    }
    config.save_timing = args.switch("timing");
    let results = run(config);
    println!(
        "wrote {}: {}",
        results.csv_path.display(),
        serde_json::to_string_pretty(&results.metrics).expect("metrics")
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn between(results: &Results, key: &str, low: f64, high: f64) {
        let value = results.metric(key);
        assert!(
            low < value && value < high,
            "{key} = {value} not in ({low}, {high})"
        );
    }

    #[cfg_attr(not(feature = "mujoco"), allow(dead_code))]
    fn below(results: &Results, key: &str, bound: f64) {
        let value = results.metric(key);
        assert!(value < bound, "{key} = {value} not below {bound}");
    }

    /// `_assert_variable_mass_orbit_metrics`: the default in-orbit burn.
    #[test]
    fn variable_mass_orbit_metrics_match_upstream() {
        let results = run(Config::new(true, true, SIM_DURATION_S));
        between(&results, "propellantDepletedFraction", 0.11, 0.12);
        let torque = results.metric("initialThrustTorqueMagnitude");
        assert!((torque - 0.1049).abs() <= 0.01 * 0.1049, "torque {torque}");
        between(&results, "deltaV", 75.0, 85.0);
        between(&results, "semiMajorAxisRise", 2.3e5, 2.6e5);
        #[cfg(feature = "mujoco")]
        {
            between(&results, "attitudeErrorMax", 3.0e-2, 5.0e-2);
            between(&results, "comPositionErrorMax", 200.0, 300.0);
            between(&results, "rateErrorMax", 1.0e-4, 1.5e-4);
            below(&results, "totalMassErrorMax", 2.0e-3);
            below(&results, "sloshDisplacementErrorMax", 2.0e-5);
            below(&results, "pendulumAngleErrorMax", 3.0e-5);
            between(&results, "semiMajorAxisRiseMujoco", 2.3e5, 2.6e5);
            between(&results, "semiMajorAxisDifferenceFinal", 700.0, 950.0);
            between(
                &results,
                "semiMajorAxisDifferenceAsFractionOfRise",
                2.5e-3,
                4.5e-3,
            );
        }
    }

    /// `test_variable_mass_rejects_nondivisible_horizon`.
    #[test]
    #[should_panic(expected = "not divisible")]
    fn variable_mass_rejects_nondivisible_horizon() {
        run(Config::new(false, false, 1.03));
    }

    /// `test_variable_mass_leak_depletes_without_external_impulse`.
    #[test]
    fn leak_depletes_without_external_impulse() {
        let results = run(Config::new(false, false, 1.0));
        assert!(results.metric("propellantDepletedFraction") > 1.0e-4);
        assert!(results.metric("deltaV").abs() < 1.0e-5);
        assert_eq!(results.metric("initialThrustTorqueMagnitude"), 0.0);
    }

    /// `test_variable_mass_thruster_burn_covers_requested_horizon`: the
    /// engine keeps accelerating past the former 1800 s command limit.
    #[test]
    fn thruster_burn_covers_requested_horizon() {
        let case = VariableMassCase::new(true, false, 1801.0);
        let (history, _) = run_bsm(&case, time_step(), true);
        let time = seconds(&history.times_nanos);
        let speed: Vec<f64> = history.v_cn_n.iter().map(|v| v.norm()).collect();
        let limit = time.partition_point(|t| *t < 1800.0);
        let preceding = speed[limit] - speed[limit - 20];
        let following = speed[speed.len() - 1] - speed[limit];
        assert!(following > 0.8 * preceding, "{following} vs {preceding}");
    }

    /// `test_variable_mass_deep_space_numerical_agreement`.
    #[cfg(feature = "mujoco")]
    #[test]
    fn deep_space_numerical_agreement() {
        let full = run(Config::new(true, false, 900.0));
        between(&full, "propellantDepletedFraction", 0.11, 0.12);
        between(&full, "deltaV", 130.0, 140.0);
        below(&full, "attitudeErrorMax", 5.0e-7);
        below(&full, "rateErrorMax", 2.0e-7);
        below(&full, "comPositionErrorMax", 5.0e-2);
        below(&full, "totalMassErrorMax", 2.0e-3);
        below(&full, "sloshDisplacementErrorMax", 1.0e-3);
        below(&full, "pendulumAngleErrorMax", 1.0e-3);

        let mut config = Config::new(true, false, 120.0);
        config.case.near_rigid = true;
        let rigid = run(config);
        assert_eq!(rigid.metrics["nearRigid"], json!(true));
        between(&rigid, "propellantDepletedFraction", 0.015, 0.016);
        between(&rigid, "wetMassChangeFraction", 0.0075, 0.008);
        between(&rigid, "deltaV", 17.0, 19.0);
        assert!((rigid.metric("initialPendulumMass") - 1.0e-6).abs() < 1e-12);
        assert!((rigid.metric("initialSmdMass") - 1.0e-9).abs() < 1e-15);
        below(&rigid, "attitudeErrorMax", 1.0e-10);
        below(&rigid, "rateErrorMax", 1.0e-11);
        below(&rigid, "comPositionErrorMax", 1.0e-5);
        below(&rigid, "totalMassErrorMax", 1.0e-8);
    }

    #[cfg(feature = "mujoco")]
    mod native_damping {
        //! `test_variable_mass_native_pendulum_damping_conserves_momentum`
        //! and `test_variable_mass_native_damping_in_rotated_anisotropic_system`:
        //! a free parent and a concentric ball-jointed child whose relative
        //! rate two zero-command velocity servos damp about the child x and
        //! y axes, the convention the variable-mass pendulum damping relies
        //! on.

        use basilisk_rs::messages::SpacecraftStateMsg;
        use basilisk_rs::mujoco_dynamics::MjScene;
        use basilisk_rs::simulation::Simulation;
        use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
        use nalgebra::{Matrix3, Vector3};

        use crate::dyncomp::{PreparedScene, sec2nano_u64};
        use crate::message_log::MessageLog;

        fn scene_xml(
            parent_quat: &str,
            parent_inertia: Vector3<f64>,
            child_quat: &str,
            child_inertia: Vector3<f64>,
            damping: f64,
        ) -> String {
            format!(
                r#"
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="parent" quat="{parent_quat}">
      <freejoint/>
      <inertial pos="0 0 0" mass="1" diaginertia="{} {} {}"/>
      <body name="child" quat="{child_quat}">
        <joint name="ball" type="ball"/>
        <inertial pos="0 0 0" mass="1" diaginertia="{} {} {}"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <velocity name="dampX" joint="ball" gear="1 0 0" kv="{damping}"/>
    <velocity name="dampY" joint="ball" gear="0 1 0" kv="{damping}"/>
  </actuator>
</mujoco>
"#,
                parent_inertia.x,
                parent_inertia.y,
                parent_inertia.z,
                child_inertia.x,
                child_inertia.y,
                child_inertia.z
            )
        }

        /// The parent and child histories after starting the ball joint at
        /// `relative_rate`.
        fn run(
            xml: &str,
            relative_rate: Vector3<f64>,
            dt: f64,
            tf: f64,
        ) -> (Vec<SpacecraftStateMsg>, Vec<SpacecraftStateMsg>) {
            let scene = MjScene::new(xml).expect("damping scene");
            let parent = scene.body("parent").origin().state_out_msg.clone();
            let child = scene.body("child").origin().state_out_msg.clone();
            let mut scene = PreparedScene::new(scene, move |scene| {
                scene.body_mut("child").set_ball_joint_rate(relative_rate);
            });
            let (mut parent_log, mut child_log) = (
                MessageLog::<SpacecraftStateMsg>::default(),
                MessageLog::<SpacecraftStateMsg>::default(),
            );
            let step = sec2nano_u64(dt);
            {
                let mut simulation = Simulation::new(crate::dyncomp_orbit::epoch(), false);
                simulation.connect(&parent, &mut parent_log.input);
                simulation.connect(&child, &mut child_log.input);
                simulation.add_module("scene", &mut scene, step, 1);
                simulation.add_module("parent", &mut parent_log, step, 0);
                simulation.add_module("child", &mut child_log, step, 0);
                simulation.run_for(sec2nano_u64(tf));
            }
            (parent_log.values, child_log.values)
        }

        fn inertial_momentum(
            states: &[SpacecraftStateMsg],
            inertia: Vector3<f64>,
        ) -> Vec<Vector3<f64>> {
            states
                .iter()
                .map(|s| {
                    mrp_to_dcm(s.sigma_bn).transpose()
                        * Matrix3::from_diagonal(&inertia)
                        * s.omega_radps
                })
                .collect()
        }

        #[test]
        fn pendulum_damping_conserves_momentum() {
            let (dt, tf, damping, rate0) = (0.001, 0.5, 0.2, 0.4);
            let (parent_inertia, child_inertia) = (2.0, 1.0);
            let quats = [
                "0.9659258263 0.2588190451 0 0",
                "0.9659258263 0 0.2588190451 0",
                "0.9659258263 0 0 0.2588190451",
            ];
            let child_quats = [
                "0.9848077530 0.1736481777 0 0",
                "0.9848077530 0 0.1736481777 0",
                "0.9848077530 0 0 0.1736481777",
            ];
            for axis in 0..3 {
                let xml = scene_xml(
                    quats[axis],
                    Vector3::repeat(parent_inertia),
                    child_quats[axis],
                    Vector3::repeat(child_inertia),
                    damping,
                );
                let mut rate = Vector3::zeros();
                rate[axis] = rate0;
                let (parent, child) = run(&xml, rate, dt, tf);
                let p = inertial_momentum(&parent, Vector3::repeat(parent_inertia));
                let c = inertial_momentum(&child, Vector3::repeat(child_inertia));
                let momentum: Vec<Vector3<f64>> = p.iter().zip(&c).map(|(a, b)| a + b).collect();
                for h in &momentum {
                    assert!(
                        (h - momentum[0]).amax() <= 2.0e-10,
                        "axis {axis}: {h} vs {}",
                        momentum[0]
                    );
                }
                let axis_n = mrp_to_dcm(child[0].sigma_bn).transpose() * Vector3::ith(axis, 1.0);
                let rate_n =
                    |s: &SpacecraftStateMsg| mrp_to_dcm(s.sigma_bn).transpose() * s.omega_radps;
                let last = child.len() - 1;
                let relative = (rate_n(&child[last]) - rate_n(&parent[last])).dot(&axis_n);
                if axis < 2 {
                    let decay = damping * (1.0 / parent_inertia + 1.0 / child_inertia);
                    let expected = rate0 * (-decay * tf).exp();
                    assert!(
                        (relative - expected).abs() <= 2.0e-3 * expected,
                        "axis {axis}: {relative} vs {expected}"
                    );
                } else {
                    assert!(
                        (relative - rate0).abs() <= 2.0e-12,
                        "axis {axis}: {relative}"
                    );
                }
            }
        }

        #[test]
        fn damping_in_rotated_anisotropic_system() {
            let (dt, tf, damping) = (1.0e-4, 2.0e-2, 0.2);
            let parent_inertia = Vector3::new(2.0, 2.5, 3.0);
            let child_inertia = Vector3::new(1.0, 1.2, 1.4);
            let relative_rate = Vector3::new(0.4, -0.3, 0.2);
            let xml = scene_xml(
                "0.9238795325 0.2209423827 0.2209423827 0.2209423827",
                parent_inertia,
                "0.9659258263 -0.1830127019 0.1830127019 0",
                child_inertia,
                damping,
            );
            let (parent, child) = run(&xml, relative_rate, dt, tf);
            let p = inertial_momentum(&parent, parent_inertia);
            let c = inertial_momentum(&child, child_inertia);
            let momentum: Vec<Vector3<f64>> = p.iter().zip(&c).map(|(a, b)| a + b).collect();
            for h in &momentum {
                assert!((h - momentum[0]).amax() <= 2.0e-10);
            }
            let energy: Vec<f64> = parent
                .iter()
                .zip(&child)
                .map(|(p, c)| {
                    0.5 * p
                        .omega_radps
                        .dot(&parent_inertia.component_mul(&p.omega_radps))
                        + 0.5
                            * c.omega_radps
                                .dot(&child_inertia.component_mul(&c.omega_radps))
                })
                .collect();
            let expected = -damping * (relative_rate.x.powi(2) + relative_rate.y.powi(2));
            let measured = (energy[1] - energy[0]) / dt;
            assert!(
                (measured - expected).abs() <= 2.0e-3 * expected.abs(),
                "{measured} vs {expected}"
            );
            assert!(energy[energy.len() - 1] < energy[0]);
        }
    }
}
