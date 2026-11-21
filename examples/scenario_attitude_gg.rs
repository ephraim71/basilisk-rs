//! Hill-frame-offset pointing under gravity-gradient torque: port of upstream
//! `examples/scenarioAttitudeGG.py`.
//!
//! [`scenarioAttitudeGuidance`](scenario_attitude_guidance.rs)'s spacecraft
//! and orbit (10 000 km, `e = 0.1`) with a `GravityGradientEffector` for the
//! Earth added. The reference is the Hill frame (`hillPoint`, no ephemeris
//! message) rotated about the second body axis (`attError.sigma_R0R =
//! [0, 0.2, 0]`). With a diagonal inertia this is not a gravity-gradient
//! equilibrium, so the gravity gradient keeps disturbing the `mrpFeedback`
//! loop (`K = 3.5`, `P = 30`, no integral feedback). One 0.1 s task,
//! 10 minutes, histories sampled every 6 s.
//!
//! The effector takes the spacecraft's mass properties from its
//! `mass_props_out` message, and the Earth, which has no ephemeris message,
//! sits at the origin.
//!
//! Upstream `run(show_plots)` has no parameters. Writes
//! `examples/output/scenarios/scenarioAttitudeGG.csv`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;

use std::path::PathBuf;

use attitude::{
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, HubState, add_point_mass_earth, att_tracking_error,
    external_disturbance, hill_point, hub_inertia_kg_m2, mrp_feedback, sampling_period_nanos,
    scenario_recorder, spacecraft, vehicle_config,
};
use basilisk_rs::dynamics::gravity_gradient_effector::{
    GravityGradientEffector, GravityGradientEffectorConfig, GravityGradientPlanet,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, GravityGradientMsg, Output, SpacecraftStateMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{MU_EARTH_M3PS2, vector_columns};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;

/// The recorded histories.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    gravity_gradient: MessageLog<GravityGradientMsg>,
    state: MessageLog<SpacecraftStateMsg>,
    csv_path: Option<PathBuf>,
}

fn run(write_csv: bool) -> Results {
    let inertia = hub_inertia_kg_m2();
    let mut spacecraft = spacecraft(inertia, &HubState::on_orbit(0.1));
    add_point_mass_earth(&mut spacecraft);
    let mut gravity_gradient = GravityGradientEffector::new(GravityGradientEffectorConfig {
        name: "spacecraftBody".to_string(),
        planets: vec![GravityGradientPlanet {
            name: "earth_planet_data".to_string(),
            mu_m3ps2: MU_EARTH_M3PS2,
        }],
    });
    let gravity_gradient_msg = gravity_gradient.gravity_gradient_out_msg.clone();
    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut guidance = hill_point();
    let mut tracking_error = att_tracking_error(Vector3::new(0.0, 0.2, 0.0));
    let mut controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 100);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut gravity_gradient_log = MessageLog::<GravityGradientMsg>::default();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder("scenarioAttitudeGG", write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &spacecraft.mass_props_out => &mut gravity_gradient.mass_props_in_msg,
        &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg,
    );
    spacecraft.add_dynamic_effector(gravity_gradient);
    spacecraft.add_dynamic_effector(external_torque);
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.translation_out_msg => &mut guidance.trans_nav_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &guidance.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &gravity_gradient_msg => &mut gravity_gradient_log.input,
        &spacecraft.state_out => &mut state_log.input,
        &controller.cmd_torque_out_msg => recorder.add_source::<BodyTorqueCommandMsg>(
            CsvSourceConfig::columns(vector_columns(
                "torque_request_body_nm",
                "torqueRequestBody_Nm",
            )),
        ),
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
        &gravity_gradient_msg => recorder.add_source::<GravityGradientMsg>(
            CsvSourceConfig::columns(vector_columns(
                "gravity_gradient_torque_body_nm",
                "gravityGradientTorque_B_Nm",
            )),
        ),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        "hill_point" => &mut guidance, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "tracking_error" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "mrp_feedback" => &mut controller, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "gravity_gradient_log" => &mut gravity_gradient_log, period=sample_period, priority=0;
        "state_log" => &mut state_log, period=sample_period, priority=0;
    }
    if write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        gravity_gradient: gravity_gradient_log,
        state: state_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_attitude_gg [--all]");
    // Upstream's test runs the one configuration there is.
    cli.switch("all");
    cli.finish();

    let results = run(true);
    println!(
        "wrote {} (final |sigma_BR| = {:.3e})",
        results.csv_path.expect("CSV written").display(),
        results.guidance.last().sigma_br.norm(),
    );
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttitudeGG.py` only checks that the scenario
    //! runs. These tests check the gravity-gradient torque against its
    //! closed form and the bounded attitude error the scenario documents.

    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;

    use super::*;

    /// `L_G = 3 mu / |R|^5 (R_B x [I] R_B)`, `R_B = [BN] r_BN`, for the
    /// recorded spacecraft states. The effector's message holds the torque of
    /// the last integrator stage, so the comparison is to a relative 1e-6.
    #[test]
    fn the_torque_matches_the_gravity_gradient_closed_form() {
        let results = run(false);
        let inertia = hub_inertia_kg_m2();
        for (state, recorded) in results
            .state
            .values
            .iter()
            .zip(&results.gravity_gradient.values)
            .skip(1)
        {
            let r_b = mrp_to_dcm(state.sigma_bn) * state.position_m;
            let expected = 3.0 * MU_EARTH_M3PS2 / r_b.norm().powi(5) * r_b.cross(&(inertia * r_b));
            let torque = recorded.gravity_gradient_torque_body_nm;
            assert!(
                (torque - expected).norm() <= 1.0e-6 * expected.norm(),
                "{torque} vs {expected}"
            );
        }
    }

    /// "Where in scenarioAttitudeGuidance the attitude error asymptotically
    /// converged to zero, with the gravity gradient torque the closed loop
    /// dynamics is now only Lagrange stable or bounded."
    #[test]
    fn the_attitude_error_stays_bounded_without_converging() {
        let results = run(false);
        assert_eq!(results.guidance.values.len(), 101);
        let initial = results.guidance.values[0].sigma_br.norm();
        for guidance in results.guidance.since(300_000_000_000) {
            let error = guidance.sigma_br.norm();
            assert!(error < 0.1 * initial, "{error}");
            assert!(error > 1.0e-5, "{error}");
        }
        // Like an unmodeled torque, the gravity gradient leaves an attitude
        // offset of about |L_G| / K.
        let gravity = results
            .gravity_gradient
            .last()
            .gravity_gradient_torque_body_nm;
        let offset = results.guidance.last().sigma_br.norm();
        let expected = gravity.norm() / 3.5;
        assert!(
            (offset - expected).abs() < 0.25 * expected,
            "{offset} vs {expected}"
        );
    }
}
