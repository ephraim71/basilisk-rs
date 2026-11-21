//! Pointing a camera at Mars through a heading filter: port of upstream
//! `examples/OpNavScenarios/scenariosOpNav/scenario_OpNavHeading.py`.
//!
//! The OpNav spacecraft ([`opnav::dynamics`]) on the circular 18 000 km orbit
//! of `scenario_OpNavPoint` (`i = 20 deg`, `Omega = 25 deg`, `omega = 190
//! deg`, `f = 100 deg`), at rest in the inertial attitude, in the
//! `pointHead` mode from the start: every minute the camera image goes
//! through `houghCircles` (`noiseSF = 0.5`) and `pixelLineConverter`, the
//! switch heading filter `headingSuKF` (`noiseSF = 1.001`) estimates the
//! planet heading and the body rates perpendicular to it, and `opNavPoint`
//! points the camera along the *filtered* heading (its input is
//! `headingUKF.opnavDataOutMsg`). 200 minutes at upstream's 0.5 s rates.
//!
//! **Images.** Upstream renders the camera images in the Vizard application.
//! Here a synthetic renderer stands in for Vizard
//! ([`opnav::planet_renderer`]): a ray-traced, Sun-lit, textureless Mars
//! ellipsoid through the camera model, so only outcomes (the filtered
//! heading tracks the planet) compare with upstream.
//!
//! Writes `examples/output/scenarios/scenario_OpNavHeading.csv`, upstream's
//! recorders at the FSW rate: the spacecraft state (`scRec`), the
//! measurement (`opNavRec`, `opnav_*`), the guidance (`attGuidRec`), the
//! wheel torques (`rwMotorRec`, `rwLogs`), the circles (`circlesRec`), the
//! heading filter (`filtRec`, `heading_*`) and its OpNav output
//! (`opNavFiltRec`, `headingOpnav_*`). Upstream also logs the filter's
//! internal `bVec_B`, which only its commented-out figures use.
//! `plot_scenario_op_nav_heading.py` draws upstream's `vecTrack` figures.
//!
//! `--sim-time m` sets upstream's `simTime` \[min\] (default 200); upstream's
//! `test_bskTestOpNav.py` runs 10 minutes and only checks completion. The
//! tests here also check the filtered heading against the truth.

#[path = "support/opnav/mod.rs"]
mod opnav;

use std::path::PathBuf;

use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, HeadingFilterMsg, OpNavCirclesMsg, OpNavMsg,
    ReactionWheelStateMsg, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use opnav::bsk_sim::{
    guidance_columns, motor_torque_columns, scenario_recorder, spacecraft_state_columns,
};
use opnav::dynamics;
use opnav::fsw::FswMode;
use opnav::message_log::MessageLog;
use opnav::outputs::{
    circles_columns, heading_filter_columns, instrument_truth, opnav_columns, rw_current_columns,
};
use opnav::{
    Cameras, OpNavModels, RATE_S, RECORDER_PRIORITY, execute_modes, hub_init, mars_orbit,
    sim_time_from_cli,
};

const STEM: &str = "scenario_OpNavHeading";
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
    heading: MessageLog<HeadingFilterMsg>,
    heading_opnav: MessageLog<OpNavMsg>,
    csv_path: Option<PathBuf>,
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
    let fsw = &mut models.fsw;
    fsw.pixel_line_filter.config.state_init = nalgebra::SVector::from_iterator(
        r_n.iter()
            .chain(v_n.iter())
            .copied()
            .chain([0.0, 0.0, -2.0]),
    );
    fsw.op_nav_point.config.omega_rn_b_radps = nalgebra::Vector3::new(0.001, 0.0, -0.001);
    fsw.image_processing.config.noise_sf = 0.5;
    fsw.heading_ukf.config.noise_sf = 1.001;
    fsw.heading_ukf
        .opnav_data_out_msg
        .connect_to(&mut fsw.op_nav_point.opnav_data_in_msg);
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    // `log_outputs`.
    let sampling = models.fsw.task_period_nanos;
    let mut spacecraft = MessageLog::<SpacecraftStateMsg>::default();
    let mut heading = MessageLog::<HeadingFilterMsg>::default();
    let mut heading_opnav = MessageLog::<OpNavMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let state_out = &models.dynamics.spacecraft.state_out;
    let gateway = &models.fsw.gateway;
    let filter_out = &models.fsw.heading_ukf.filt_data_out_msg;
    let filter_opnav_out = &models.fsw.heading_ukf.opnav_data_out_msg;
    state_out.connect_to(&mut spacecraft.input);
    filter_out.connect_to(&mut heading.input);
    filter_opnav_out.connect_to(&mut heading_opnav.input);
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
    filter_out
        .connect_to(recorder.add_source::<HeadingFilterMsg>(heading_filter_columns("heading")));
    filter_opnav_out.connect_to(recorder.add_source::<OpNavMsg>(opnav_columns("headingOpnav")));

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "scRec" => &mut spacecraft, period = sampling, priority = RECORDER_PRIORITY;
        "filtRec" => &mut heading, period = sampling, priority = RECORDER_PRIORITY;
        "opNavFiltRec" => &mut heading_opnav, period = sampling, priority = RECORDER_PRIORITY;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_PRIORITY);
    }
    execute_modes(
        &mut simulation,
        &mode,
        &runner,
        &[(FswMode::PointHead, config.sim_time_min)],
    );
    drop(simulation);

    Results {
        spacecraft,
        heading,
        heading_opnav,
        csv_path,
    }
}

/// The angle \[rad\] between the filter's body-frame heading (its first
/// three states) and the true direction to Mars.
fn heading_error_rad(filter: &HeadingFilterMsg, state: &SpacecraftStateMsg) -> f64 {
    let estimate = nalgebra::Vector3::new(filter.state[0], filter.state[1], filter.state[2]);
    let truth_b = -(basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm(state.sigma_bn)
        * state.position_m);
    basilisk_rs::utilities::linear_algebra::safe_acos(
        estimate.normalize().dot(&truth_b.normalize()),
    )
}

fn main() {
    let sim_time_min = sim_time_from_cli("scenario_op_nav_heading", SIM_TIME_MIN);
    let results = run(Config {
        sim_time_min,
        write_csv: true,
    });
    if let Some(path) = &results.csv_path {
        println!("wrote {}", path.display());
    }
    println!(
        "final heading error {:.3} deg, off-pointing {:.3} deg",
        heading_error_rad(results.heading.last(), results.spacecraft.last()).to_degrees(),
        instrument_truth(results.spacecraft.last())
            .off_pointing_rad
            .to_degrees()
    );
}

#[cfg(test)]
mod tests {
    //! Upstream `test_bskTestOpNav.py` runs `run(False, 10)` and checks
    //! only that it completes. The other test checks what the scenario
    //! demonstrates: once Mars is found, the filtered heading tracks it and
    //! keeps the camera on it.

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
    fn the_filtered_heading_tracks_mars() {
        let results = run(Config {
            sim_time_min: SIM_TIME_MIN,
            write_csv: false,
        });
        let track_from = min2nano(150.0);
        let mut outputs = 0;
        for (((time, state), filter), opnav) in results
            .spacecraft
            .times_nanos
            .iter()
            .zip(&results.spacecraft.values)
            .zip(&results.heading.values)
            .zip(&results.heading_opnav.values)
        {
            if *time < track_from {
                continue;
            }
            let error = heading_error_rad(filter, state);
            assert!(
                error < 1.0_f64.to_radians(),
                "heading error {error} at {time}"
            );
            let truth = instrument_truth(state);
            assert!(
                truth.off_pointing_rad < 2.0_f64.to_radians(),
                "{truth:?} at {time}"
            );
            // The filter's OpNav output, valid at each image: the heading
            // scaled to the measured range, along the true camera-frame
            // direction.
            if opnav.valid {
                outputs += 1;
                let angle = basilisk_rs::utilities::linear_algebra::safe_acos(
                    opnav
                        .position_bn_c_m
                        .normalize()
                        .dot(&truth.r_bn_c_m.normalize()),
                );
                assert!(angle < 1.0_f64.to_radians(), "{angle} at {time}");
            }
        }
        assert!(outputs >= 49, "{outputs} filtered fixes");
    }
}
