//! Port of upstream `examples/dynamicsComparison/sweepTorqueArtifact.py`:
//! two sweeps of the velocity-regime round-off of `scenario_compare_torque`.
//! Needs the `mujoco` feature.
//!
//! * **Velocity sweep**: the torqued body without gravity, drifting at a
//!   constant inertial speed (default 0, 750, 2500, 7500 m/s along
//!   `[2, -3, 6]/7`), against the same body at rest; translation cannot
//!   torque it, so the attitude change is each engine's velocity-dependent
//!   error.
//! * **dt sweep**: the scenario's orbiting-versus-rest discriminator over
//!   RK4 steps (default 0.4 s to 0.0125 s), showing the artifact does not
//!   converge with the step (round-off, not truncation), while the at-rest
//!   cross-engine difference falls as `dt⁴`.
//!
//! Outputs, in `examples/output/scenarios/`: `sweepTorqueArtifact.json` and
//! `sweepTorqueArtifact_velocity.csv`, `sweepTorqueArtifact_dt.csv` (for
//! `plot_dyncomp_paper_figures.py`'s `sweepTorqueArtifact` figure).
//!
//! `tests` ports upstream's `torque-artifact` reduced check and
//! `test_torque_sweep_rejects_unreachable_requested_horizon`.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/dyncomp.rs"]
mod dyncomp;
#[path = "support/dyncomp_orbit.rs"]
mod dyncomp_orbit;
#[path = "support/dyncomp_torque.rs"]
mod dyncomp_torque;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/table_csv.rs"]
mod table_csv;

use cli::Cli;
use dyncomp::{
    Series, max_of, recorder_sample_interval, relative_principal_angles, seconds, validate_history,
    validate_matching_histories, validate_task_horizon, write_json,
};
use dyncomp_torque::{AttitudeHistory, OMEGA0_B, TORQUE_B, TorqueCase, run_bsm, run_mujoco};
use grav_factory::Body;
use nalgebra::Vector3;
use serde_json::json;

const FILE_NAME: &str = "sweepTorqueArtifact";

/// Upstream `DRIFT_DIR`: a generic direction, along which the `m (v × v)`
/// round-off products do not cancel exactly.
pub fn drift_direction() -> Vector3<f64> {
    Vector3::new(2.0, -3.0, 6.0) / 7.0
}

#[derive(Clone, Debug)]
pub struct Config {
    /// Upstream `speeds` \[m/s\].
    pub speeds: Vec<f64>,
    /// Upstream `dts` \[s\].
    pub dts: Vec<f64>,
    pub tf_s: f64,
    pub record_dt_s: f64,
    pub velocity_sweep_dt_s: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            speeds: vec![0.0, 750.0, 2500.0, 7500.0],
            dts: vec![0.4, 0.2, 0.1, 0.05, 0.025, 0.0125],
            tf_s: 600.0,
            record_dt_s: 1.0,
            velocity_sweep_dt_s: 0.1,
        }
    }
}

#[derive(Clone, Copy, Debug, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VelocityRow {
    pub speed: f64,
    pub bsm_drift_att_max: f64,
    pub mujoco_drift_att_max: f64,
}

#[derive(Clone, Copy, Debug, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DtRow {
    pub dt: f64,
    pub bsm_motion_att_max: f64,
    pub mujoco_motion_att_max: f64,
    pub cross_engine_rest_att_max: f64,
}

fn check(result: Result<Vec<f64>, String>) {
    result.unwrap_or_else(|error| panic!("{error}"));
}

fn validated(name: &str, history: &AttitudeHistory, tf: f64, interval: f64) -> Vec<f64> {
    let times = seconds(&history.times_nanos);
    check(validate_history(
        name,
        &times,
        Some(tf),
        interval,
        false,
        &[Series::vectors("attitude", &history.sigma_bn)],
    ));
    times
}

fn drift(speed: f64) -> TorqueCase {
    TorqueCase::drifting(speed * drift_direction(), TORQUE_B, OMEGA0_B)
}

/// Upstream `velocitySweep`.
pub fn velocity_sweep(
    speeds: &[f64],
    dt: f64,
    tf: f64,
    record_dt: f64,
) -> Result<Vec<VelocityRow>, String> {
    validate_task_horizon("torque velocity sweep", tf, dt)?;
    let interval = recorder_sample_interval(dt, record_dt)?;
    let bsm_rest = run_bsm(&drift(0.0), dt, tf, record_dt, true).0;
    let mujoco_rest = run_mujoco(&drift(0.0), dt, tf, record_dt, true, false).history;
    let bsm_rest_s = validated("torque velocity-sweep BSM rest", &bsm_rest, tf, interval);
    let mujoco_rest_s = validated(
        "torque velocity-sweep MuJoCo rest",
        &mujoco_rest,
        tf,
        interval,
    );
    check(validate_matching_histories(
        "torque velocity-sweep rest BSM/MuJoCo",
        &bsm_rest_s,
        &mujoco_rest_s,
        Some(tf),
        interval,
        false,
    ));
    let mut rows = Vec::new();
    for speed in speeds {
        let bsm = run_bsm(&drift(*speed), dt, tf, record_dt, true).0;
        let mujoco = run_mujoco(&drift(*speed), dt, tf, record_dt, true, false).history;
        let bsm_s = validated("torque velocity-sweep BSM", &bsm, tf, interval);
        let mujoco_s = validated("torque velocity-sweep MuJoCo", &mujoco, tf, interval);
        check(validate_matching_histories(
            "torque velocity-sweep BSM drift/rest",
            &bsm_s,
            &bsm_rest_s,
            Some(tf),
            interval,
            false,
        ));
        check(validate_matching_histories(
            "torque velocity-sweep MuJoCo drift/rest",
            &mujoco_s,
            &mujoco_rest_s,
            Some(tf),
            interval,
            false,
        ));
        let row = VelocityRow {
            speed: *speed,
            bsm_drift_att_max: max_of(relative_principal_angles(
                "torque attitude",
                &bsm.sigma_bn,
                &bsm_rest.sigma_bn,
            )),
            mujoco_drift_att_max: max_of(relative_principal_angles(
                "torque attitude",
                &mujoco.sigma_bn,
                &mujoco_rest.sigma_bn,
            )),
        };
        println!(
            "velocity {:8.1} m/s: BSM {:.3e} rad, MuJoCo {:.3e} rad",
            speed, row.bsm_drift_att_max, row.mujoco_drift_att_max
        );
        rows.push(row);
    }
    Ok(rows)
}

/// Upstream `dtSweep`.
pub fn dt_sweep(dts: &[f64], tf: f64, record_dt: f64, mu: f64) -> Vec<DtRow> {
    let mut rows = Vec::new();
    for dt in dts {
        validate_task_horizon("torque time-step sweep", tf, *dt)
            .unwrap_or_else(|error| panic!("{error}"));
        let interval = recorder_sample_interval(*dt, record_dt).expect("interval");
        let orbiting = TorqueCase::orbiting(mu, TORQUE_B, OMEGA0_B);
        let at_rest = TorqueCase::at_rest(TORQUE_B, OMEGA0_B);
        let bsm_orbit = run_bsm(&orbiting, *dt, tf, record_dt, true).0;
        let bsm_rest = run_bsm(&at_rest, *dt, tf, record_dt, true).0;
        let mujoco_orbit = run_mujoco(&orbiting, *dt, tf, record_dt, true, false).history;
        let mujoco_rest = run_mujoco(&at_rest, *dt, tf, record_dt, true, false).history;
        let times: Vec<Vec<f64>> = [
            ("BSM orbit", &bsm_orbit),
            ("BSM rest", &bsm_rest),
            ("MuJoCo orbit", &mujoco_orbit),
            ("MuJoCo rest", &mujoco_rest),
        ]
        .iter()
        .map(|(name, history)| validated(&format!("torque dt-sweep {name}"), history, tf, interval))
        .collect();
        for (name, a, b) in [
            ("torque dt-sweep BSM orbit/rest", 0, 1),
            ("torque dt-sweep MuJoCo orbit/rest", 2, 3),
            ("torque dt-sweep BSM/MuJoCo orbit", 0, 2),
        ] {
            check(validate_matching_histories(
                name,
                &times[a],
                &times[b],
                Some(tf),
                interval,
                false,
            ));
        }
        let angle = |a: &AttitudeHistory, b: &AttitudeHistory| {
            max_of(relative_principal_angles(
                "torque attitude",
                &a.sigma_bn,
                &b.sigma_bn,
            ))
        };
        let row = DtRow {
            dt: *dt,
            bsm_motion_att_max: angle(&bsm_orbit, &bsm_rest),
            mujoco_motion_att_max: angle(&mujoco_orbit, &mujoco_rest),
            cross_engine_rest_att_max: angle(&bsm_rest, &mujoco_rest),
        };
        println!(
            "dt {:7.4} s: MuJoCo artifact {:.3e} rad, BSM artifact {:.3e} rad, at-rest \
             cross-engine {:.3e} rad",
            dt, row.mujoco_motion_att_max, row.bsm_motion_att_max, row.cross_engine_rest_att_max
        );
        rows.push(row);
    }
    rows
}

pub fn run(config: &Config) -> (Vec<VelocityRow>, Vec<DtRow>) {
    let mu = Body::Earth.mu_m3ps2();
    println!("=== velocity sweep (no gravity, inertial drift) ===");
    let velocity = velocity_sweep(
        &config.speeds,
        config.velocity_sweep_dt_s,
        config.tf_s,
        config.record_dt_s,
    )
    .unwrap_or_else(|error| panic!("{error}"));
    println!("=== dt sweep (orbiting vs rest discriminator) ===");
    let dt = dt_sweep(&config.dts, config.tf_s, config.record_dt_s, mu);
    let direction = drift_direction();
    write_json(
        FILE_NAME,
        &json!({
            "scenario": FILE_NAME,
            "configuration": {
                "tf": config.tf_s,
                "recordDt": config.record_dt_s,
                "velocitySweepDt": config.velocity_sweep_dt_s,
                "body": "same rigid body, torque, and initial rate as scenarioCompareTorque",
                "velocityCase": "no gravity, constant inertial drift along velocityDirection",
                "velocityDirection": [direction.x, direction.y, direction.z],
                "dtCase": "orbiting (a=7000 km Kepler) vs at-rest discriminator",
            },
            "velocitySweep": velocity,
            "dtSweep": dt,
        }),
    );
    table_csv::write_table(
        &format!("{FILE_NAME}_velocity"),
        &[
            "speed_mps",
            "bsm_drift_att_max_rad",
            "mujoco_drift_att_max_rad",
        ]
        .map(String::from),
        &velocity
            .iter()
            .map(|r| vec![r.speed, r.bsm_drift_att_max, r.mujoco_drift_att_max])
            .collect::<Vec<_>>(),
    );
    table_csv::write_table(
        &format!("{FILE_NAME}_dt"),
        &[
            "dt_s",
            "bsm_motion_att_max_rad",
            "mujoco_motion_att_max_rad",
            "cross_engine_rest_att_max_rad",
        ]
        .map(String::from),
        &dt.iter()
            .map(|r| {
                vec![
                    r.dt,
                    r.bsm_motion_att_max,
                    r.mujoco_motion_att_max,
                    r.cross_engine_rest_att_max,
                ]
            })
            .collect::<Vec<_>>(),
    );
    (velocity, dt)
}

const USAGE: &str = "\
sweep_torque_artifact [--speeds 0,750,2500,7500] [--dts 0.4,0.2,...] [--duration S]
                      [--record-dt S] [--velocity-dt S]";

fn list(args: &mut Cli, name: &str, default: Vec<f64>) -> Vec<f64> {
    let text: String = args.value(name, String::new());
    if text.is_empty() {
        return default;
    }
    text.split(',')
        .map(|v| {
            v.trim()
                .parse()
                .unwrap_or_else(|_| args.fail(&format!("invalid --{name} {v:?}")))
        })
        .collect()
}

fn main() {
    let mut args = Cli::from_env(USAGE);
    let defaults = Config::default();
    let config = Config {
        speeds: list(&mut args, "speeds", defaults.speeds),
        dts: list(&mut args, "dts", defaults.dts),
        tf_s: args.value("duration", defaults.tf_s),
        record_dt_s: args.value("record-dt", defaults.record_dt_s),
        velocity_sweep_dt_s: args.value("velocity-dt", defaults.velocity_sweep_dt_s),
    };
    run(&config);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `test_research_sweeps_capture_expected_physical_trends[torque-artifact]`.
    #[test]
    fn artifact_grows_with_speed_and_rest_difference_converges() {
        let (velocity, dt) = run(&Config {
            speeds: vec![0.0, 7500.0],
            dts: vec![0.2, 0.1],
            tf_s: 0.2,
            record_dt_s: 0.1,
            velocity_sweep_dt_s: 0.1,
        });
        assert!(
            velocity[1].mujoco_drift_att_max > 1.0e6 * velocity[0].mujoco_drift_att_max,
            "{velocity:?}"
        );
        assert!(
            dt[1].cross_engine_rest_att_max < 0.1 * dt[0].cross_engine_rest_att_max,
            "{dt:?}"
        );
    }

    /// `test_torque_sweep_rejects_unreachable_requested_horizon`.
    #[test]
    fn rejects_unreachable_requested_horizon() {
        let error = velocity_sweep(&[0.0], 0.2, 0.25, 0.1).unwrap_err();
        assert!(error.contains("does not lie on"), "{error}");
    }
}
