//! Pointing a body axis at an Earth-fixed location: port of upstream
//! `examples/scenarioAttLocPoint.py`.
//!
//! A 750 kg rigid spacecraft on a 600 km altitude, `e = 0.1` Earth orbit
//! (`a = 6978 km`, `i = 63.3°`, `Ω = 88.2°`, `ω = 347.8°`, `f = 135.3°`)
//! starts at `sigma_BN = [0.1, 0.2, -0.3]`,
//! `omega_BN = [0.001, -0.01, 0.03] rad/s`. `groundLocation` publishes the
//! inertial position of a station in Boulder (40.009971° N, 105.243895° W,
//! 1624 m, on a spherical Earth with no planet message, so non-rotating at
//! the origin), `locationPointing` turns it into the 2D pointing error of the
//! third body axis (with boresight rate damping), and `mrpFeedback`
//! (`K = 5.5`, `P = 30`, no integral feedback) commands the torque an
//! `extForceTorque` effector applies. One 1 s task, 20 minutes, histories
//! sampled every `samplingTime(20 min, 1 s, 100)` = 12 s.
//!
//! Upstream's `run(show_plots)` has no scenario parameters. Writes
//! `examples/output/scenarios/scenarioAttLocPoint.csv`.

#[path = "support/att_scenario.rs"]
mod att_scenario;
#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;

use std::path::PathBuf;

use att_scenario::{
    attitude_columns, guidance_columns, rigid_spacecraft, torque_columns, translation_columns,
};
use attitude::{
    HubState, INITIAL_OMEGA_BN_RADPS, INITIAL_SIGMA_BN, add_point_mass_earth, external_disturbance,
    hub_inertia_kg_m2, mrp_feedback, sampling_period_nanos, scenario_recorder, vehicle_config,
};
use basilisk_rs::environment::ground_location::{
    GroundLocation, GroundLocationConfig, SiteLocation,
};
use basilisk_rs::fsw_algorithms::att_guidance::location_pointing::{
    LocationPointing, LocationPointingConfig,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, GroundStateMsg, NavigationAttitudeMsg,
    NavigationTranslationMsg, Output,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::REQ_EARTH;
use basilisk_rs::utilities::geodetic_conversion::Lla;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{MU_EARTH_M3PS2, seconds, vector_columns};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;

/// Upstream `simulationTimeStep = macros.sec2nano(1.0)`.
const STEP_NANOS: u64 = seconds(1);
/// Upstream `simulationTime = macros.min2nano(20.)`.
const DURATION_NANOS: u64 = seconds(20 * 60);
/// CSV file stem, upstream's figure-name prefix.
const FILE_STEM: &str = "scenarioAttLocPoint";

/// Upstream's orbit, `oe` of the script.
fn orbit() -> ClassicElements {
    ClassicElements {
        semi_major_axis: (6378.0 + 600.0) * 1000.0,
        eccentricity: 0.1,
        inclination_rad: 63.3_f64.to_radians(),
        raan_rad: 88.2_f64.to_radians(),
        arg_periapsis_rad: 347.8_f64.to_radians(),
        true_anomaly_rad: 135.3_f64.to_radians(),
        ..Default::default()
    }
}

/// Upstream `groundStation` ("BoulderGroundStation").
fn boulder_ground_station() -> GroundLocation {
    let mut station = GroundLocation::new(GroundLocationConfig {
        name: "BoulderGroundStation".to_string(),
        planet_radius_m: REQ_EARTH * 1.0e3,
        minimum_elevation_rad: 10.0_f64.to_radians(),
        maximum_range_m: Some(1.0e9),
        location: SiteLocation::Geodetic(Lla::new(
            40.009971_f64.to_radians(),
            (-105.243895_f64).to_radians(),
            1624.0,
        )),
    });
    station.add_spacecraft_to_model();
    station
}

/// The recorded histories.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    torque: MessageLog<BodyTorqueCommandMsg>,
    attitude: MessageLog<NavigationAttitudeMsg>,
    translation: MessageLog<NavigationTranslationMsg>,
    station: MessageLog<GroundStateMsg>,
    csv_path: Option<PathBuf>,
}

impl Results {
    /// Angle \[rad\] between the third body axis and the direction from the
    /// spacecraft to the station at sample `index`.
    fn boresight_error_rad(&self, index: usize) -> f64 {
        let to_station = self.station.values[index].position_ln_n_m
            - self.translation.values[index].position_bn_n_m;
        let b3_n = mrp_to_dcm(self.attitude.values[index].sigma_bn).transpose()
            * Vector3::new(0.0, 0.0, 1.0);
        b3_n.angle(&to_station)
    }
}

fn run(write_csv: bool) -> Results {
    let inertia = hub_inertia_kg_m2();
    let (position_m, velocity_mps) = elements_to_state(MU_EARTH_M3PS2, &orbit());
    let initial = HubState {
        position_m,
        velocity_mps,
        sigma_bn: Vector3::from(INITIAL_SIGMA_BN),
        omega_bn_b_radps: Vector3::from(INITIAL_OMEGA_BN_RADPS),
    };
    let mut spacecraft = rigid_spacecraft(inertia, &initial, STEP_NANOS);
    add_point_mass_earth(&mut spacecraft);
    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut station = boulder_ground_station();
    let mut pointing = LocationPointing::new(LocationPointingConfig {
        name: "locPoint".to_string(),
        p_hat_b: Vector3::new(0.0, 0.0, 1.0),
        use_boresight_rate_damping: true,
        ..Default::default()
    });
    let mut controller = mrp_feedback(5.5, -1.0, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let sample_period = sampling_period_nanos(DURATION_NANOS, STEP_NANOS, 100);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut torque_log = MessageLog::<BodyTorqueCommandMsg>::default();
    let mut attitude_log = MessageLog::<NavigationAttitudeMsg>::default();
    let mut translation_log = MessageLog::<NavigationTranslationMsg>::default();
    let mut station_log = MessageLog::<GroundStateMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(FILE_STEM, write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg,
    );
    spacecraft.add_dynamic_effector(external_torque);
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &spacecraft.state_out => &mut station.sc_state_in_msgs[0],
        &navigation.attitude_out_msg => &mut pointing.sc_att_in_msg,
        &navigation.translation_out_msg => &mut pointing.sc_trans_in_msg,
        &station.current_ground_state_out_msg => &mut pointing.location_in_msg,
        &pointing.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &pointing.att_guid_out_msg => &mut guidance_log.input,
        &controller.cmd_torque_out_msg => &mut torque_log.input,
        &navigation.attitude_out_msg => &mut attitude_log.input,
        &navigation.translation_out_msg => &mut translation_log.input,
        &station.current_ground_state_out_msg => &mut station_log.input,
        &controller.cmd_torque_out_msg =>
            recorder.add_source::<BodyTorqueCommandMsg>(torque_columns()),
        &pointing.att_guid_out_msg =>
            recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns()),
        &navigation.attitude_out_msg =>
            recorder.add_source::<NavigationAttitudeMsg>(attitude_columns()),
        &navigation.translation_out_msg =>
            recorder.add_source::<NavigationTranslationMsg>(translation_columns()),
        &station.current_ground_state_out_msg => recorder.add_source::<GroundStateMsg>(
            CsvSourceConfig::columns(vector_columns("position_ln_n_m", "r_LN_N_m")),
        ),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=STEP_NANOS, priority=100;
        "navigation" => &mut navigation, period=STEP_NANOS, priority=90;
        "ground_station" => &mut station, period=STEP_NANOS, priority=85;
        "location_pointing" => &mut pointing, period=STEP_NANOS, priority=80;
        "mrp_feedback" => &mut controller, period=STEP_NANOS, priority=60;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "torque_log" => &mut torque_log, period=sample_period, priority=0;
        "attitude_log" => &mut attitude_log, period=sample_period, priority=0;
        "translation_log" => &mut translation_log, period=sample_period, priority=0;
        "station_log" => &mut station_log, period=sample_period, priority=0;
    }
    if write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        torque: torque_log,
        attitude: attitude_log,
        translation: translation_log,
        station: station_log,
        csv_path,
    }
}

fn main() {
    Cli::from_env("scenario_att_loc_point").finish();
    let results = run(true);
    let last = results.guidance.values.len() - 1;
    println!(
        "wrote {} (final |sigma_BR| = {:.3e}, boresight error {:.3e} deg)",
        results.csv_path.as_ref().expect("CSV written").display(),
        results.guidance.last().sigma_br.norm(),
        results.boresight_error_rad(last).to_degrees(),
    );
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttLocPoint.py` only checks that the scenario
    //! runs. These tests check the pointing it demonstrates.

    use super::*;

    /// The station sits on the 1624 m altitude sphere about the origin.
    #[test]
    fn the_station_is_on_the_spherical_earth() {
        let results = run(false);
        let station = results.station.values[0].position_ln_n_m;
        assert!((station.norm() - (REQ_EARTH * 1.0e3 + 1624.0)).abs() < 1.0e-6);
        // No planet message: the Earth does not rotate, the station is fixed.
        assert_eq!(results.station.last().position_ln_n_m, station);
    }

    /// The third body axis starts far off Boulder and ends pointing at it:
    /// the 2D pointing error (`|sigma_BR| = tan(phi / 4)`) agrees with the
    /// angle between `b3` and the line of sight, and both decay to the small
    /// lag of tracking the moving target.
    #[test]
    fn the_third_body_axis_points_at_boulder() {
        let results = run(false);
        assert_eq!(results.guidance.values.len(), 101);
        assert!(results.boresight_error_rad(0) > 80.0_f64.to_radians());
        for index in [0, 50, 100] {
            let from_guidance = 4.0 * results.guidance.values[index].sigma_br.norm().atan();
            let geometric = results.boresight_error_rad(index);
            assert!(
                (from_guidance - geometric).abs() < 1.0e-9,
                "{from_guidance} vs {geometric}"
            );
        }
        let last = results.guidance.last();
        assert!(last.sigma_br.norm() < 2.0e-4, "{}", last.sigma_br.norm());
        assert!(last.omega_br_b_radps.norm() < 1.0e-5);
        let error = results.boresight_error_rad(100);
        assert!(error < 0.05_f64.to_radians(), "{} deg", error.to_degrees());
        // A large initial slew command; tracking the target needs little.
        assert!(results.torque.values[0].torque_request_body_nm.norm() > 1.0);
        assert!(results.torque.last().torque_request_body_nm.norm() < 1.0e-2);
        // Still converging over the last third.
        assert!(error < results.boresight_error_rad(70));
    }
}
