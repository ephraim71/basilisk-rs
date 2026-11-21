//! A spacecraft whose orbit comes from an SPK file: port of upstream
//! `examples/scenarioSpiceSpacecraft.py`.
//!
//! The Hubble Space Telescope's trajectory, from upstream's `hst_edited.bsp`
//! relative to the Earth from 2015 February 10, 00:00:00 TDB, prescribes the
//! translation of a 750 kg spacecraft through its `transRefInMsg`, while its
//! attitude stays dynamic: from `sigma_BN = [0.1, 0.2, -0.3]`,
//! `omega_BN = [0.001, -0.01, 0.03] rad/s`, `inertial3D`,
//! `attTrackingError` and `mrpFeedback` (`K = 3.5`, `P = 30`) point it at the
//! inertial frame through an `extForceTorque` effector, as in
//! `scenarioAttitudeFeedback`. One 0.1 s task, 10 minutes, histories sampled
//! every 6 s.
//!
//! Needs the `spice` feature: `hst_edited.bsp` is a type-10 (two-line
//! element) SPK, which CSPICE reads and ANISE does not. As upstream,
//! [`SpiceInterface`] (upstream's kernels, `just fetch-spice-kernels`, plus
//! the Hubble SPK, `zeroBase = "Earth"`) runs before the spacecraft, which
//! takes the position and velocity at initialization and after every
//! integration step, and also publishes the Earth's state (the origin) to
//! its point-mass gravity.
//!
//! ```text
//! cargo run --release --features spice --example scenario_spice_spacecraft
//! ```
//!
//! Upstream `run(show_plots)` has no parameters (it runs in
//! `test_scenarioTest.py`). Writes
//! `examples/output/scenarios/scenarioSpiceSpacecraft.csv`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;

use std::path::PathBuf;

use attitude::{
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, HubState, att_tracking_error, external_disturbance,
    hub_inertia_kg_m2, inertial_3d, mrp_feedback, sampling_period_nanos, scenario_recorder,
    spacecraft, vehicle_config,
};
use basilisk_rs::environment::spice_interface::{SpiceInterface, SpiceInterfaceConfig};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, Output, SpacecraftStateMsg, TranslationReferenceMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::Body;
use hifitime::{Epoch, TimeScale};
use message_log::MessageLog;
use nalgebra::Vector3;

/// Upstream `timeInitString`.
const TIME_INIT: &str = "2015 February 10, 00:00:00.0 TDB";

/// [`TIME_INIT`].
fn start_epoch() -> Epoch {
    Epoch::from_gregorian(2015, 2, 10, 0, 0, 0, 0, TimeScale::TDB)
}

/// Upstream `gravFactory.createSpiceInterface(time, epochInMsg=True)` with
/// `zeroBase = "Earth"`, the Earth, and the Hubble SPK loaded.
fn spice_interface() -> SpiceInterface {
    let kernel_paths = [
        "assets/spice/de430.bsp",
        "assets/spice/naif0012.tls",
        "assets/spice/de-403-masses.tpc",
        "assets/spice/pck00010.tpc",
        "assets/spice/hst_edited.bsp",
    ]
    .into_iter()
    .map(PathBuf::from)
    .collect();
    SpiceInterface::new(SpiceInterfaceConfig {
        name: "spiceObject".to_string(),
        kernel_paths,
        zero_base: "Earth".to_string(),
        utc_cal_init: Some(TIME_INIT.to_string()),
        planet_names: vec!["earth".to_string()],
        planet_frames: vec!["IAU_earth".to_string()],
        spacecraft_names: vec!["HUBBLE SPACE TELESCOPE".to_string()],
        ..SpiceInterfaceConfig::default()
    })
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    state: MessageLog<SpacecraftStateMsg>,
    reference: MessageLog<TranslationReferenceMsg>,
    guidance: MessageLog<AttitudeGuidanceMsg>,
    torque: MessageLog<BodyTorqueCommandMsg>,
    csv_path: Option<PathBuf>,
}

fn run(write_csv: bool) -> Results {
    let inertia = hub_inertia_kg_m2();
    let mut initial = HubState::in_deep_space();
    initial.sigma_bn = Vector3::new(0.1, 0.2, -0.3);
    initial.omega_bn_b_radps = Vector3::new(0.001, -0.01, 0.03);
    let mut spacecraft = spacecraft(inertia, &initial);
    spacecraft
        .add_grav_body(Body::Earth.point_mass(true))
        .expect("one central body");
    let mut spice = spice_interface();
    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut inertial_reference = inertial_3d();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let mut controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 100);
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut reference_log = MessageLog::<TranslationReferenceMsg>::default();
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut torque_log = MessageLog::<BodyTorqueCommandMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder("scenarioSpiceSpacecraft", write_csv);

    let mut simulation = Simulation::new(start_epoch(), false);
    connect!(&simulation, &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg);
    spacecraft.add_dynamic_effector(external_torque);
    simulation.connect(
        &spice.planet_state_out_msgs[0],
        spacecraft
            .grav_body_input_mut("earth")
            .expect("Earth gravity"),
    );
    connect!(&simulation,
        &spice.trans_ref_state_out_msgs[0] => &mut spacecraft.trans_ref_in,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &spacecraft.state_out => &mut state_log.input,
        &spice.trans_ref_state_out_msgs[0] => &mut reference_log.input,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &controller.cmd_torque_out_msg => &mut torque_log.input,
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
        &controller.cmd_torque_out_msg => recorder.add_source::<BodyTorqueCommandMsg>(
            CsvSourceConfig::columns(vector_columns("torque_request_body_nm", "torqueRequestBody_Nm")),
        ),
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(vector_columns("position_m", "r_BN_N_m")),
        ),
    );
    // Upstream's task order: spiceInterface first, as the spacecraft reads
    // its translational reference.
    schedule! { simulation,
        "spiceInterface" => &mut spice, period=DYNAMICS_PERIOD_NANOS, priority=110;
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        "inertial3D" => &mut inertial_reference, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "attTrackingError" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "mrpFeedback" => &mut controller, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "state_log" => &mut state_log, period=sample_period, priority=0;
        "reference_log" => &mut reference_log, period=sample_period, priority=0;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "torque_log" => &mut torque_log, period=sample_period, priority=0;
    }
    if write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        state: state_log,
        reference: reference_log,
        guidance: guidance_log,
        torque: torque_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_spice_spacecraft [--all]");
    // Upstream's test runs the one configuration there is.
    cli.switch("all");
    cli.finish();
    let results = run(true);
    println!(
        "wrote {} (final |sigma_BR| = {:.3e}, |r_BN| = {:.1} km)",
        results.csv_path.expect("CSV written").display(),
        results.guidance.last().sigma_br.norm(),
        results.state.last().position_m.norm() * 1e-3,
    );
}

#[cfg(test)]
mod tests {
    //! Upstream runs the scenario in `test_scenarioTest.py`, which only checks
    //! that it runs. These tests check what it demonstrates: the orbit is
    //! Hubble's from the SPK, and the attitude reaches the inertial frame.

    use super::*;

    #[test]
    fn the_spacecraft_flies_the_hubble_trajectory() {
        let results = run(false);
        assert_eq!(results.state.values.len(), 101);
        for (state, reference) in results.state.values.iter().zip(&results.reference.values) {
            // The spacecraft takes the reference written in the same tick.
            assert_eq!(state.position_m, reference.position_m);
            assert_eq!(state.velocity_mps, reference.velocity_mps);
            // Hubble's low Earth orbit, about 540 km up.
            let altitude_km = state.position_m.norm() * 1e-3 - 6378.1366;
            assert!((450.0..650.0).contains(&altitude_km), "{altitude_km}");
        }
    }

    #[test]
    fn the_attitude_reaches_the_inertial_frame() {
        let results = run(false);
        let initial = results.guidance.values[0].sigma_br.norm();
        assert!((initial - Vector3::<f64>::new(0.1, 0.2, -0.3).norm()).abs() < 1e-12);
        assert!(results.guidance.last().sigma_br.norm() < 1e-3);
        assert!(results.guidance.last().omega_br_b_radps.norm() < 1e-4);
        assert!(results.torque.values[0].torque_request_body_nm.norm() > 0.5);
    }
}
