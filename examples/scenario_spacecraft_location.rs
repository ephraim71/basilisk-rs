//! Spacecraft-to-spacecraft access through a body-fixed antenna cone: port of
//! upstream `examples/scenarioSpacecraftLocation.py`.
//!
//! A 750 kg servicer (inertia `diag(900, 800, 600)` kg m^2) on a circular
//! 10 000 km, 33.3 deg Earth orbit holds the Hill frame with `hillPoint`,
//! `attTrackingError`, `mrpFeedback` (`K = 3.5`, `P = 30`, no integral
//! feedback) and an `extForceTorque` effector. A 350 kg debris object
//! (inertia `diag(600, 650, 450)` kg m^2) tumbles on nearly the same orbit,
//! 40 m ahead with a slightly larger eccentricity (`e = 1e-6`), so it
//! circles about 10 m around that point. `spacecraftLocation` reports access
//! from the servicer's body +y axis (the along-track direction once the
//! Hill frame is held) when the debris is within 55 m and 10 deg of that
//! axis, with the Earth (`rPolar = 0.98 rEquator`) blocking the line of
//! sight. One 1 s task, 140 minutes.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted and runs
//! the same single case.
//!
//! Writes `examples/output/scenarios/scenarioSpacecraftLocation.csv` every
//! step with the access message's `hasAccess`, `slantRange_m` and
//! `elevation_rad` (the angle from the cone edge, `pi/2` minus the angle
//! from the antenna axis), and the servicer's `sigma_BR_<i>`.

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
    HUB_MASS_KG, add_point_mass_earth, att_tracking_error, external_disturbance, hill_point,
    hub_inertia_kg_m2, mrp_feedback, scenario_recorder, vehicle_config,
};
use basilisk_rs::environment::spacecraft_location::{
    Boresight, SpacecraftLocation, SpacecraftLocationConfig,
};
use basilisk_rs::messages::{AccessMsg, AttitudeGuidanceMsg, Output, SpacecraftStateMsg};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{D2R, MU_EARTH, REQ_EARTH};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};

/// Upstream `simulationTimeStep = macros.sec2nano(1.0)`.
const TASK_PERIOD_NANOS: u64 = 1_000_000_000;
/// Upstream `simulationTime = macros.min2nano(140.)`.
const DURATION_NANOS: u64 = 140 * 60 * 1_000_000_000;
/// Upstream `scLocation.maximumRange` \[m\].
const MAXIMUM_RANGE_M: f64 = 55.0;
/// Upstream `scLocation.theta` \[rad\]: the antenna cone half angle.
const CONE_HALF_ANGLE_RAD: f64 = 10.0 * D2R;

/// Upstream `run(show_plots)`: no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The single run of upstream's `test_scenarioSpacecraftLocation.py`.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config { write_csv: true }];

/// The recorded histories, every step.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Upstream `accessRec`.
    access: MessageLog<AccessMsg>,
    servicer: MessageLog<SpacecraftStateMsg>,
    debris: MessageLog<SpacecraftStateMsg>,
    guidance: MessageLog<AttitudeGuidanceMsg>,
    csv_path: Option<PathBuf>,
}

/// A hub with `mass_kg` and `inertia_kg_m2` on the 1 s task.
fn hub(
    mass_kg: f64,
    inertia_kg_m2: Matrix3<f64>,
    elements: &ClassicElements,
    sigma_bn: Vector3<f64>,
    omega_bn_b_radps: Vector3<f64>,
) -> Spacecraft {
    let (initial_position_m, initial_velocity_mps) = elements_to_state(MU_EARTH * 1e9, elements);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2,
        integration_step_nanos: TASK_PERIOD_NANOS,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: sigma_bn,
        initial_omega_radps: omega_bn_b_radps,
        integrator: None,
    });
    add_point_mass_earth(&mut spacecraft);
    spacecraft
}

fn run(config: Config) -> Results {
    let servicer_elements = ClassicElements {
        semi_major_axis: 10_000_000.0,
        eccentricity: 0.0,
        inclination_rad: 33.3 * D2R,
        raan_rad: 48.2 * D2R,
        arg_periapsis_rad: 90.0 * D2R,
        true_anomaly_rad: 0.0,
        ..Default::default()
    };
    // Upstream `oe2.e += 0.000001` and `oe2.f += 40./oe2.a`.
    let debris_elements = ClassicElements {
        eccentricity: servicer_elements.eccentricity + 0.000001,
        true_anomaly_rad: servicer_elements.true_anomaly_rad
            + 40.0 / servicer_elements.semi_major_axis,
        ..servicer_elements
    };
    let inertia = hub_inertia_kg_m2();
    let mut servicer = hub(
        HUB_MASS_KG,
        inertia,
        &servicer_elements,
        Vector3::new(0.1, 0.2, -0.3),
        Vector3::new(0.0, 0.02, 0.01),
    );
    let mut debris = hub(
        350.0,
        Matrix3::from_diagonal(&Vector3::new(600.0, 650.0, 450.0)),
        &debris_elements,
        Vector3::new(0.3, 0.1, 0.2),
        Vector3::new(0.001, -0.01, 0.03),
    );
    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");

    let r_equator_m = REQ_EARTH * 1e3;
    let mut location = SpacecraftLocation::new(SpacecraftLocationConfig {
        name: "scAccess".to_string(),
        r_polar_m: Some(r_equator_m * 0.98),
        maximum_range_m: Some(MAXIMUM_RANGE_M),
        boresight: Some(Boresight {
            a_hat_b: Vector3::y(),
            theta_rad: CONE_HALF_ANGLE_RAD,
        }),
        ..SpacecraftLocationConfig::new(r_equator_m)
    });
    let debris_index = location.add_spacecraft_to_model();

    let mut guidance = hill_point();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let mut controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let mut access_log = MessageLog::<AccessMsg>::default();
    let mut servicer_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut debris_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let (mut recorder, csv_path) =
        scenario_recorder("scenarioSpacecraftLocation", config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    let access = location.access_out_msgs[debris_index].clone();
    connect!(&simulation,
        &servicer.state_out => &mut navigation.spacecraft_state_in_msg,
        &servicer.state_out => &mut location.primary_sc_state_in_msg,
        &debris.state_out => &mut location.sc_state_in_msgs[debris_index],
        &navigation.translation_out_msg => &mut guidance.trans_nav_in_msg,
        &guidance.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg,
        &access => &mut access_log.input,
        &servicer.state_out => &mut servicer_log.input,
        &debris.state_out => &mut debris_log.input,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &access => recorder.add_source::<AccessMsg>(CsvSourceConfig::columns([
            ("has_access", "hasAccess"),
            ("slant_range_m", "slantRange_m"),
            ("elevation_rad", "elevation_rad"),
        ])),
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_br", "sigma_BR")),
        ),
    );
    servicer.add_dynamic_effector(external_torque);

    // Upstream's task order (all at the default priority): both spacecraft,
    // navigation, the location, the flight software, then the recorder.
    schedule! { simulation,
        "Servicer" => &mut servicer, period=TASK_PERIOD_NANOS, priority=100;
        "Debris" => &mut debris, period=TASK_PERIOD_NANOS, priority=100;
        "SimpleNavigation" => &mut navigation, period=TASK_PERIOD_NANOS, priority=90;
        "scAccess" => &mut location, period=TASK_PERIOD_NANOS, priority=80;
        "hillPoint" => &mut guidance, period=TASK_PERIOD_NANOS, priority=70;
        "attError" => &mut tracking_error, period=TASK_PERIOD_NANOS, priority=60;
        "mrpFeedback" => &mut controller, period=TASK_PERIOD_NANOS, priority=50;
        "accessRec" => &mut access_log, period=TASK_PERIOD_NANOS, priority=0;
        "servicerLog" => &mut servicer_log, period=TASK_PERIOD_NANOS, priority=0;
        "debrisLog" => &mut debris_log, period=TASK_PERIOD_NANOS, priority=0;
        "guidanceLog" => &mut guidance_log, period=TASK_PERIOD_NANOS, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, TASK_PERIOD_NANOS, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        access: access_log,
        servicer: servicer_log,
        debris: debris_log,
        guidance: guidance_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_spacecraft_location [--all]");
    cli.switch("all");
    cli.finish();

    for config in TEST_PARAMETRIZATIONS {
        let results = run(config);
        let access_s = results
            .access
            .values
            .iter()
            .filter(|access| access.has_access)
            .count();
        println!(
            "wrote {} ({access_s} s of servicer-debris access in 140 min)",
            results.csv_path.as_ref().expect("CSV written").display(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioSpacecraftLocation.py` only checks that the
    //! scenario runs. These tests check the access it reports against the
    //! geometry of the two recorded spacecraft, and the relative motion the
    //! scenario sets up.

    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;

    use super::*;

    #[test]
    fn access_follows_the_antenna_cone_and_range() {
        let results = quiet();
        assert_eq!(
            results.access.values.len() as u64,
            DURATION_NANOS / TASK_PERIOD_NANOS + 1
        );
        for ((access, servicer), debris) in results
            .access
            .values
            .iter()
            .zip(&results.servicer.values)
            .zip(&results.debris.values)
        {
            // The debris relative to the servicer, in its body frame.
            let r_sl_n = debris.position_m - servicer.position_m;
            let r_sl_b = mrp_to_dcm(servicer.sigma_bn) * r_sl_n;
            let range = r_sl_b.norm();
            let off_axis = (r_sl_b.y / range).clamp(-1.0, 1.0).acos();
            assert!((access.slant_range_m - range).abs() < 1e-6);
            assert!((access.elevation_rad - (std::f64::consts::FRAC_PI_2 - off_axis)).abs() < 1e-9);
            let expected = range <= MAXIMUM_RANGE_M && off_axis <= CONE_HALF_ANGLE_RAD;
            assert_eq!(
                access.has_access, expected,
                "range {range}, angle {off_axis}"
            );
        }
    }

    #[test]
    fn the_debris_circles_ahead_of_the_hill_pointing_servicer() {
        let results = quiet();
        // The servicer settles on the Hill frame within 10 minutes.
        let settled = results
            .guidance
            .since(10 * 60 * TASK_PERIOD_NANOS)
            .map(|guidance| guidance.sigma_br.norm())
            .fold(0.0, f64::max);
        assert!(settled < 1e-3, "|sigma_BR| up to {settled}");
        // A 1e-6 eccentricity difference at 10 000 km: the debris stays
        // within 40 +- 2 x 10 m along track and 10 m radially.
        for (servicer, debris) in results.servicer.values.iter().zip(&results.debris.values) {
            let range = (debris.position_m - servicer.position_m).norm();
            assert!((15.0..65.0).contains(&range), "range {range}");
        }
        // So, once the servicer points +y along track, the debris drifts in
        // and out of the 10 deg cone: access comes and goes.
        let late: Vec<bool> = results
            .access
            .since(10 * 60 * TASK_PERIOD_NANOS)
            .map(|access| access.has_access)
            .collect();
        let switches = late.windows(2).filter(|pair| pair[0] != pair[1]).count();
        assert!(late.iter().any(|access| *access));
        assert!(switches >= 2, "{switches} access changes");
    }

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }
}
