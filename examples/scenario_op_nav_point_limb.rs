//! Pointing a camera at Mars from its limb: port of upstream
//! `examples/OpNavScenarios/scenariosOpNav/scenario_OpNavPointLimb.py`.
//!
//! The OpNav spacecraft ([`opnav::dynamics`]) on the circular 18 000 km orbit
//! of `scenario_OpNavPoint` (`i = 20 deg`, `Omega = 25 deg`, `omega = 190
//! deg`, `f = 100 deg`), at rest in the inertial attitude. For 5 minutes
//! `prepOpNav` points the camera at Mars from the Hill frame; then
//! `pointLimb` closes the loop on the images: every minute `limbFinding`
//! (Canny edges) finds the limb, `horizonOpNav` (J. Christian's horizon
//! navigation) turns it into a position fix, and `opNavPoint` keeps the
//! camera on the planet. 200 minutes at upstream's 0.5 s rates.
//!
//! **Images.** Upstream renders the camera images in the Vizard application.
//! Here a synthetic renderer stands in for Vizard
//! ([`opnav::planet_renderer`]): a ray-traced, Sun-lit, textureless Mars
//! ellipsoid through the camera model, so only outcomes (the camera stays on
//! Mars) compare with upstream.
//!
//! Writes `examples/output/scenarios/scenario_OpNavPointLimb.csv`,
//! upstream's recorders at the FSW rate: the attitude guidance
//! (`attGuidRec`) and the wheel torques (`rwMotorRec`, `rwLogs`), plus the
//! spacecraft state; and `scenario_OpNavPointLimb_limb.csv`, the limbs
//! found (see `opnav::outputs::write_limb_points`).
//! `plot_scenario_op_nav_point_limb.py` draws upstream's wheel-torque,
//! attitude-error and rate-error figures.
//!
//! `--sim-time m` sets upstream's `simTime` \[min\] (default 200); upstream's
//! `test_bskTestOpNav.py` runs 10 minutes and only checks completion. The
//! tests here also check that the limb keeps the camera on Mars.

#[path = "support/opnav/mod.rs"]
mod opnav;

use std::path::PathBuf;

use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, OpNavLimbMsg, ReactionWheelStateMsg,
    SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use opnav::bsk_sim::{
    guidance_columns, motor_torque_columns, scenario_recorder, spacecraft_state_columns,
};
use opnav::dynamics;
use opnav::fsw::FswMode;
use opnav::message_log::MessageLog;
use opnav::outputs::{instrument_truth, rw_current_columns, write_limb_points};
use opnav::{
    Cameras, OpNavModels, RATE_S, RECORDER_PRIORITY, execute_modes, hub_init, mars_orbit,
    sim_time_from_cli,
};

const STEM: &str = "scenario_OpNavPointLimb";
/// Upstream's default `simTime` \[min\].
const SIM_TIME_MIN: f64 = 200.0;
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
    guidance: MessageLog<AttitudeGuidanceMsg>,
    limbs: MessageLog<OpNavLimbMsg>,
    csv_paths: Vec<PathBuf>,
}

fn run(config: Config) -> Results {
    // `configure_initial_conditions` (`filterUse = "bias"`: the unused
    // pixel-line filter's initial state).
    let (r_n, v_n) = mars_orbit(18_000e3, 0.0, 20.0, 25.0, 190.0, 100.0);
    let mut models = OpNavModels::new(
        RATE_S,
        RATE_S,
        &hub_init(r_n, v_n, [0.0; 3]),
        Cameras::default(),
    );
    models.fsw.pixel_line_filter.config.state_init = nalgebra::SVector::from_iterator(
        r_n.iter()
            .chain(v_n.iter())
            .copied()
            .chain([0.0, 0.0, -2.0]),
    );
    models.fsw.op_nav_point.config.omega_rn_b_radps = nalgebra::Vector3::new(0.001, 0.0, -0.001);
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    // `log_outputs`.
    let sampling = models.fsw.task_period_nanos;
    let mut spacecraft = MessageLog::<SpacecraftStateMsg>::default();
    let mut guidance = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut limbs = MessageLog::<OpNavLimbMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let state_out = &models.dynamics.spacecraft.state_out;
    let gateway = &models.fsw.gateway;
    let limb_out = &models.fsw.limb_finding.opnav_limb_out_msg;
    state_out.connect_to(&mut spacecraft.input);
    gateway.att_guid.connect_to(&mut guidance.input);
    limb_out.connect_to(&mut limbs.input);
    state_out.connect_to(recorder.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("")));
    gateway
        .att_guid
        .connect_to(recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns("")));
    models
        .fsw
        .rw_motor_torque
        .rw_motor_torque_out_msg
        .connect_to(recorder.add_source::<ArrayMotorTorqueMsg>(motor_torque_columns()));
    for (index, wheel) in models.dynamics.rw_out_msgs.iter().enumerate() {
        wheel.connect_to(recorder.add_source::<ReactionWheelStateMsg>(rw_current_columns(index)));
    }

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    let camera_period = models.dynamics.camera_period_nanos;
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "scRec" => &mut spacecraft, period = sampling, priority = RECORDER_PRIORITY;
        "attGuidRec" => &mut guidance, period = sampling, priority = RECORDER_PRIORITY;
        "limbRec" => &mut limbs, period = camera_period, priority = opnav::outputs::LIMB_LOG_PRIORITY;
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
            (FswMode::PointLimb, config.sim_time_min),
        ],
    );
    drop(simulation);
    let limb_path = config
        .write_csv
        .then(|| write_limb_points(&format!("{STEM}_limb"), &limbs, 30));

    Results {
        spacecraft,
        guidance,
        limbs,
        csv_paths: csv_path.into_iter().chain(limb_path).collect(),
    }
}

fn main() {
    let sim_time_min = sim_time_from_cli("scenario_op_nav_point_limb", SIM_TIME_MIN);
    let results = run(Config {
        sim_time_min,
        write_csv: true,
    });
    for path in &results.csv_paths {
        println!("wrote {}", path.display());
    }
    let valid = results
        .limbs
        .values
        .iter()
        .filter(|limb| limb.valid)
        .count();
    println!(
        "{valid} limbs found; final |sigma_BR| {:.3e}, off-pointing {:.3} deg",
        results.guidance.last().sigma_br.norm(),
        instrument_truth(results.spacecraft.last())
            .off_pointing_rad
            .to_degrees()
    );
}

#[cfg(test)]
mod tests {
    //! Upstream `test_bskTestOpNav.py` runs `run(False, 10)` and checks
    //! only that it completes. The other test checks what the scenario
    //! demonstrates: each image's limb keeps the camera on Mars.

    use super::*;
    use opnav::min2nano;

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
    fn the_limb_keeps_the_camera_on_mars() {
        let results = run(Config {
            sim_time_min: 100.0,
            write_csv: false,
        });
        let images = results
            .limbs
            .times_nanos
            .iter()
            .zip(&results.limbs.values)
            .filter(|(time, limb)| **time > min2nano(PREP_TIME_MIN) && limb.valid)
            .count();
        // One limb per image from the first image of `pointLimb`.
        assert!(images >= 94, "{images} limbs");
        for (time, state) in results
            .spacecraft
            .times_nanos
            .iter()
            .zip(&results.spacecraft.values)
        {
            if *time >= min2nano(30.0) {
                let off_pointing = instrument_truth(state).off_pointing_rad;
                assert!(
                    off_pointing < 2.0_f64.to_radians(),
                    "{off_pointing} at {time}"
                );
            }
        }
        assert!(results.guidance.last().sigma_br.norm() < 0.01);
    }
}
