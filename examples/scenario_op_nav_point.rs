//! Pointing a camera at Mars from Hough circles: port of upstream
//! `examples/OpNavScenarios/scenariosOpNav/scenario_OpNavPoint.py`.
//!
//! The OpNav spacecraft ([`opnav::dynamics`]) on a circular 18 000 km orbit
//! about Mars (`i = 20 deg`, `Omega = 25 deg`, `omega = 190 deg`, `f = 100
//! deg`), at rest in the inertial attitude, in the `pointOpNav` mode from
//! the start: every minute the camera image goes through `houghCircles` and
//! `pixelLineConverter`, and `opNavPoint` turns the camera to the planet
//! found, commanding its search rate `omega_RN_B = [0.001, 0, -0.001] rad/s`
//! while no planet has been seen. `mrpFeedbackRWs` and `rwMotorTorque` fly it
//! on the four wheels. 200 minutes at upstream's 0.5 s rates.
//!
//! **Images.** Upstream renders the camera images in the Vizard application.
//! Here a synthetic renderer stands in for Vizard
//! ([`opnav::planet_renderer`]): a ray-traced, Sun-lit, textureless Mars
//! ellipsoid through the camera model, so only outcomes (the camera finds
//! and holds Mars, the circles match the truth) compare with upstream.
//!
//! Writes `examples/output/scenarios/scenario_OpNavPoint.csv`, upstream's
//! recorders at the FSW rate: the spacecraft state (`scRec`), the OpNav
//! measurement (`opNavRec`, `opnav_*`), the attitude guidance
//! (`attGuidRec`), the wheel torques (`rwMotorRec`, `rwLogs`) and the
//! circles (`circlesRec`). `plot_scenario_op_nav_point.py` draws upstream's
//! wheel-torque and image-processing figures.
//!
//! `--sim-time m` sets upstream's `simTime` \[min\] (default 200); upstream's
//! `test_bskTestOpNav.py` runs 10 minutes and only checks completion. The
//! tests here also check that the camera finds Mars and holds it.

#[path = "support/opnav/mod.rs"]
mod opnav;

use std::path::PathBuf;

use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, OpNavCirclesMsg, OpNavMsg, ReactionWheelStateMsg,
    SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use opnav::bsk_sim::{
    guidance_columns, motor_torque_columns, scenario_recorder, spacecraft_state_columns,
};
use opnav::dynamics;
use opnav::fsw::FswMode;
use opnav::message_log::MessageLog;
use opnav::outputs::{circles_columns, instrument_truth, opnav_columns, rw_current_columns};
use opnav::{
    Cameras, OpNavModels, RATE_S, RECORDER_PRIORITY, execute_modes, hub_init, mars_orbit,
    sim_time_from_cli,
};

const STEM: &str = "scenario_OpNavPoint";
/// Upstream's default `simTime` \[min\].
const SIM_TIME_MIN: f64 = 200.0;

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
    circles: MessageLog<OpNavCirclesMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    // `configure_initial_conditions`: the pixel-line filter's initial state
    // (`filterUse = "bias"`) is set, though no task of this scenario runs it.
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
    let mut circles = MessageLog::<OpNavCirclesMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let state_out = &models.dynamics.spacecraft.state_out;
    let gateway = &models.fsw.gateway;
    state_out.connect_to(&mut spacecraft.input);
    gateway.opnav_circles.connect_to(&mut circles.input);
    state_out.connect_to(recorder.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("")));
    gateway
        .opnav
        .connect_to(recorder.add_source::<OpNavMsg>(opnav_columns("opnav")));
    gateway
        .att_guid
        .connect_to(recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns("")));
    models
        .fsw
        .rw_motor_torque
        .rw_motor_torque_out_msg
        .connect_to(recorder.add_source::<ArrayMotorTorqueMsg>(motor_torque_columns()));
    gateway
        .opnav_circles
        .connect_to(recorder.add_source::<OpNavCirclesMsg>(circles_columns()));
    for (index, wheel) in models.dynamics.rw_out_msgs.iter().enumerate() {
        wheel.connect_to(recorder.add_source::<ReactionWheelStateMsg>(rw_current_columns(index)));
    }

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "scRec" => &mut spacecraft, period = sampling, priority = RECORDER_PRIORITY;
        "circlesRec" => &mut circles, period = sampling, priority = RECORDER_PRIORITY;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_PRIORITY);
    }
    execute_modes(
        &mut simulation,
        &mode,
        &runner,
        &[(FswMode::PointOpNav, config.sim_time_min)],
    );
    drop(simulation);

    Results {
        spacecraft,
        circles,
        csv_path,
    }
}

fn main() {
    let sim_time_min = sim_time_from_cli("scenario_op_nav_point", SIM_TIME_MIN);
    let results = run(Config {
        sim_time_min,
        write_csv: true,
    });
    if let Some(path) = &results.csv_path {
        println!("wrote {}", path.display());
    }
    let first = results
        .circles
        .times_nanos
        .iter()
        .zip(&results.circles.values)
        .find(|(_, circles)| circles.valid)
        .map(|(time, _)| *time as f64 / 60e9);
    println!(
        "first circle at {first:?} min; final off-pointing {:.3} deg",
        instrument_truth(results.spacecraft.last())
            .off_pointing_rad
            .to_degrees()
    );
}

#[cfg(test)]
mod tests {
    //! Upstream `test_bskTestOpNav.py` runs `run(False, 10)` and checks
    //! only that it completes. The other test checks what the scenario
    //! demonstrates: the search finds Mars and the camera then holds it.

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
    fn the_camera_finds_mars_and_holds_it() {
        let results = run(Config {
            sim_time_min: SIM_TIME_MIN,
            write_csv: false,
        });
        let hold_from = min2nano(150.0);
        for ((time, state), circles) in results
            .spacecraft
            .times_nanos
            .iter()
            .zip(&results.spacecraft.values)
            .zip(&results.circles.values)
        {
            if *time < hold_from {
                continue;
            }
            let truth = instrument_truth(state);
            assert!(
                truth.off_pointing_rad < 2.0_f64.to_radians(),
                "off-pointing {} deg at {time}",
                truth.off_pointing_rad.to_degrees()
            );
            if circles.valid {
                let center = circles.circles_centers_px[0];
                assert!((center.x - truth.center_px[0]).abs() < 3.0);
                assert!((center.y - truth.center_px[1]).abs() < 3.0);
                assert!((circles.circles_radii_px[0] - truth.radius_px).abs() < 3.0);
            }
        }
    }
}
