//! A servicer, a tumbling debris object with free-spinning wheels and an inert
//! debris satellite: port of upstream `examples/scenarioFormationBasic.py`.
//!
//! * Servicer: 750 kg, `I = diag(900, 800, 600) kg m^2`, three Honeywell HR16
//!   wheels (50 N m s, along the body axes, at 100, 200 and 300 RPM, the third
//!   at `[0.5, 0.5, 0.5] m`), on a circular 10 000 km orbit (`i = 33.3°`,
//!   `Ω = 48.2°`, `ω = 90°`, `f = 0`) from `sigma_BN = [0.1, 0.2, -0.3]` at
//!   rest. `hillPoint` (no celestial body message) and `attTrackingError`
//!   with `sigma_R0R = [0.414214, 0, 0]` (third body axis along-track) form
//!   the error, `mrpFeedback` (`K = 3.5`, `P = 30`, no integral feedback,
//!   wheel momentum included) the torque and `rwMotorTorque` the wheel
//!   commands. The wheels saturate at 0.2 N m, and the attitude still
//!   settles.
//! * Debris: 350 kg, `I = diag(600, 650, 450) kg m^2`, two HR16 wheels spinning
//!   freely at +1000 and -1000 RPM about `b1` and `b2`, on the servicer's
//!   orbit with `e = 1e-6`: a 2:1 relative ellipse about the servicer.
//!   Integrated together with the servicer, upstream's
//!   `scObject.syncDynamicsIntegration(scObject2)`, here a
//!   [`SpacecraftGroup`] (RK4, upstream's explicitly set `svIntegratorRK4`).
//! * Debris satellite: 350 kg, the debris inertia, no effectors, 40 m ahead
//!   of the servicer (`f + 40/a`).
//!
//! One 0.5 s task, 40 minutes, histories sampled every 24 s. As in the other
//! reaction-wheel scenarios the wheels take one command each:
//! `arrayMotorTorqueToSingleActuators`, run right after `rwMotorTorque`,
//! stands in for upstream's direct subscription of the wheel effector to the
//! torque array. Upstream's Vizard light and camera are display only and not
//! ported.
//!
//! Upstream `run(show_plots)` has no parameters. Writes
//! `examples/output/scenarios/scenarioFormationBasic.csv`;
//! `plot_scenario_formation_basic.py` redraws upstream's figures from it.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/formation.rs"]
mod formation;
#[path = "support/formation_servicer.rs"]
mod formation_servicer;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/rw_factory.rs"]
mod rw_factory;

use std::path::PathBuf;

use attitude::{att_tracking_error, hill_point, scenario_recorder, vehicle_config};
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffector,
};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, MAX_EFF_COUNT, NavigationTranslationMsg, Output,
    ReactionWheelStateMsg, RwSpeedMsg, SpacecraftDiagnosticsMsg, SpacecraftStateMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::SpacecraftGroup;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{array_columns, vector_columns};
use formation::{InitialState, hub_inertia_kg_m2, rigid_spacecraft, spacecraft_state_columns};
use formation_servicer::{SERVICER_MASS_KG, SERVICER_WHEELS, servicer_wheels, wheel_control};
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::min2nano;
use recording::sampling_time;
use rw_factory::RwFactory;

const USAGE: &str = "scenario_formation_basic   (upstream `run(show_plots)` has no parameters)";

/// Upstream `simulationTimeStep = macros.sec2nano(.5)`.
const STEP_NANOS: u64 = 500_000_000;
/// Upstream `numDataPoints`.
const NUM_DATA_POINTS: u64 = 100;
/// Upstream debris `mHub` (both debris objects) \[kg\].
const DEBRIS_MASS_KG: f64 = 350.0;
/// The debris's two free-spinning wheels.
const DEBRIS_WHEELS: usize = 2;

/// Upstream `I2 = I3 = [600., 0., 0., 0., 650., 0., 0., 0, 450.]` \[kg m^2\].
fn debris_inertia_kg_m2() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(600.0, 650.0, 450.0))
}

/// The recorded histories.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Upstream `attErrLog`.
    guidance: MessageLog<AttitudeGuidanceMsg>,
    /// Upstream `rwCmdLog`.
    #[allow(dead_code)]
    requested_torque: MessageLog<ArrayMotorTorqueMsg>,
    /// Upstream `rwSc1Log`: the servicer's wheels.
    servicer_wheels: Vec<MessageLog<ReactionWheelStateMsg>>,
    /// Upstream `rwSc2Log`: the debris's wheels.
    debris_wheels: Vec<MessageLog<ReactionWheelStateMsg>>,
    /// The states of the servicer, the debris and the debris satellite.
    states: [MessageLog<SpacecraftStateMsg>; 3],
    /// The debris's and the debris satellite's diagnostics (angular momentum).
    debris_diagnostics: [MessageLog<SpacecraftDiagnosticsMsg>; 2],
    csv_path: Option<PathBuf>,
}

fn run(write_csv: bool) -> Results {
    let degrees = std::f64::consts::PI / 180.0;
    let mu = Body::Earth.mu_m3ps2();
    let servicer_oe = ClassicElements {
        semi_major_axis: 10_000_000.0,
        eccentricity: 0.0,
        inclination_rad: 33.3 * degrees,
        raan_rad: 48.2 * degrees,
        arg_periapsis_rad: 90.0 * degrees,
        true_anomaly_rad: 0.0,
        ..Default::default()
    };
    let debris_oe = ClassicElements {
        eccentricity: servicer_oe.eccentricity + 0.000001,
        ..servicer_oe
    };
    let debris_sat_oe = ClassicElements {
        true_anomaly_rad: servicer_oe.true_anomaly_rad + 40.0 / servicer_oe.semi_major_axis,
        ..servicer_oe
    };

    let inertia = hub_inertia_kg_m2();
    let mut servicer = rigid_spacecraft(
        SERVICER_MASS_KG,
        inertia,
        STEP_NANOS,
        &InitialState::at_rest(elements_to_state(mu, &servicer_oe))
            .with_attitude([0.1, 0.2, -0.3], [0.0, 0.0, 0.0]),
    );
    let mut debris = rigid_spacecraft(
        DEBRIS_MASS_KG,
        debris_inertia_kg_m2(),
        STEP_NANOS,
        &InitialState::at_rest(elements_to_state(mu, &debris_oe))
            .with_attitude([0.3, 0.1, 0.2], [0.001, -0.01, 0.03]),
    );
    let mut debris_sat = rigid_spacecraft(
        DEBRIS_MASS_KG,
        debris_inertia_kg_m2(),
        STEP_NANOS,
        &InitialState::at_rest(elements_to_state(mu, &debris_sat_oe))
            .with_attitude([0.0, -0.1, 0.2], [0.01, -0.03, -0.03]),
    );
    for spacecraft in [&mut servicer, &mut debris, &mut debris_sat] {
        spacecraft
            .add_grav_body(Body::Earth.point_mass(true))
            .expect("one central body");
    }

    let servicer_wheels = servicer_wheels();
    let mut servicer_wheel_effector = servicer_wheels.effector;
    // Upstream `rwFactory2`: two free-spinning wheels on the debris.
    let model = ReactionWheelModel::BalancedWheels;
    let mut debris_factory = RwFactory::default();
    debris_factory.create_honeywell_hr16(Vector3::x(), 50.0, 1000.0, Vector3::zeros(), model);
    debris_factory.create_honeywell_hr16(Vector3::y(), 50.0, -1000.0, Vector3::zeros(), model);
    let mut debris_wheels = ReactionWheelStateEffector::new("debrisRW");
    for wheel in debris_factory.wheels {
        debris_wheels.add_reaction_wheel(wheel);
    }
    let rw_config_msg = servicer_wheels.config_msg;
    let wheel_speed_msg = servicer_wheels.speed_msg;
    let servicer_wheel_msgs = servicer_wheels.state_msgs;
    let debris_wheel_speed_msg = debris_wheels.rw_speed_out_msg.clone();
    let debris_wheel_msgs: Vec<Output<ReactionWheelStateMsg>> = debris_wheels
        .wheels()
        .iter()
        .map(|wheel| wheel.state_out.clone())
        .collect();

    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut guidance = hill_point();
    let mut tracking_error = att_tracking_error(Vector3::new(0.414214, 0.0, 0.0));
    let vehicle_config_msg = Output::new(vehicle_config(inertia));
    let (mut controller, mut rw_motor_torque, mut torque_splitter) = wheel_control();

    let duration_nanos = min2nano(40.0);
    let sample_nanos = sampling_time(duration_nanos, STEP_NANOS, NUM_DATA_POINTS);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut requested_torque_log = MessageLog::<ArrayMotorTorqueMsg>::default();
    let mut servicer_wheel_logs: Vec<MessageLog<ReactionWheelStateMsg>> = (0..SERVICER_WHEELS)
        .map(|_| MessageLog::default())
        .collect();
    let mut debris_wheel_logs: Vec<MessageLog<ReactionWheelStateMsg>> =
        (0..DEBRIS_WHEELS).map(|_| MessageLog::default()).collect();
    let mut state_logs: [MessageLog<SpacecraftStateMsg>; 3] = Default::default();
    let mut diagnostics_logs: [MessageLog<SpacecraftDiagnosticsMsg>; 2] = Default::default();
    let (mut csv, csv_path) = scenario_recorder("scenarioFormationBasic", write_csv);

    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        for (command, wheel) in torque_splitter
            .actuator_out_msgs
            .iter()
            .zip(servicer_wheel_effector.wheels_mut())
        {
            simulation.connect(command, &mut wheel.motor_torque_in_msg);
        }
        servicer.add_state_effector(servicer_wheel_effector);
        debris.add_state_effector(debris_wheels);

        connect!(&simulation,
            &servicer.state_out => &mut navigation.spacecraft_state_in_msg,
            &navigation.translation_out_msg => &mut guidance.trans_nav_in_msg,
            &guidance.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
            &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
            &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
            &vehicle_config_msg => &mut controller.veh_config_in_msg,
            &rw_config_msg => &mut controller.rw_params_in_msg,
            &wheel_speed_msg => &mut controller.rw_speeds_in_msg,
            &controller.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
            &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
            &rw_motor_torque.rw_motor_torque_out_msg => &mut torque_splitter.torque_in_msg,
            &tracking_error.att_guid_out_msg => &mut guidance_log.input,
            &rw_motor_torque.rw_motor_torque_out_msg => &mut requested_torque_log.input,
            &servicer.state_out => &mut state_logs[0].input,
            &debris.state_out => &mut state_logs[1].input,
            &debris_sat.state_out => &mut state_logs[2].input,
            &debris.diagnostics_out => &mut diagnostics_logs[0].input,
            &debris_sat.diagnostics_out => &mut diagnostics_logs[1].input,
            &rw_motor_torque.rw_motor_torque_out_msg => csv.add_source::<ArrayMotorTorqueMsg>(
                CsvSourceConfig::columns(array_columns(
                    "motor_torque_nm",
                    "motorTorque_Nm",
                    MAX_EFF_COUNT,
                )),
            ),
            &tracking_error.att_guid_out_msg => csv.add_source::<AttitudeGuidanceMsg>(
                CsvSourceConfig::columns(
                    vector_columns("sigma_br", "sigma_BR")
                        .into_iter()
                        .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
                ),
            ),
            &navigation.translation_out_msg => csv.add_source::<NavigationTranslationMsg>(
                CsvSourceConfig::columns(vector_columns("position_bn_n_m", "r_BN_N_m")),
            ),
            &wheel_speed_msg => csv.add_source::<RwSpeedMsg>(
                CsvSourceConfig::columns(array_columns(
                    "wheel_speeds_radps",
                    "wheelSpeeds_rad_per_s",
                    MAX_EFF_COUNT,
                )),
            ),
            &debris_wheel_speed_msg => csv.add_source::<RwSpeedMsg>(
                CsvSourceConfig::columns(array_columns(
                    "wheel_speeds_radps",
                    "debrisWheelSpeeds_rad_per_s",
                    DEBRIS_WHEELS,
                )),
            ),
            &servicer.state_out => csv.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("servicer")),
            &debris.state_out => csv.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("debris")),
            &debris_sat.state_out => csv.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("debrisSat")),
        );
        for (index, (state, log)) in servicer_wheel_msgs
            .iter()
            .zip(&mut servicer_wheel_logs)
            .enumerate()
        {
            connect!(&simulation,
                state => &mut log.input,
                state => csv.add_source::<ReactionWheelStateMsg>(CsvSourceConfig::columns([(
                    "applied_motor_torque_nm".to_string(),
                    format!("rwTorque_Nm_{index}"),
                )])),
            );
        }
        for (state, log) in debris_wheel_msgs.iter().zip(&mut debris_wheel_logs) {
            simulation.connect(state, &mut log.input);
        }

        // Upstream's synced pair: the servicer integrates the debris.
        let mut servicer_and_debris = SpacecraftGroup::new(vec![servicer, debris]);
        // Upstream's task order: wheels and spacecraft, then navigation and
        // flight software, then the recorders.
        schedule! { simulation,
            "Servicer+Debris" => &mut servicer_and_debris, period=STEP_NANOS, priority=100;
            "DebrisSat" => &mut debris_sat, period=STEP_NANOS, priority=100;
            "SimpleNavigation" => &mut navigation, period=STEP_NANOS, priority=90;
            "hillPoint" => &mut guidance, period=STEP_NANOS, priority=80;
            "attErrorInertial3D" => &mut tracking_error, period=STEP_NANOS, priority=70;
            "mrpFeedback" => &mut controller, period=STEP_NANOS, priority=60;
            "rwMotorTorque" => &mut rw_motor_torque, period=STEP_NANOS, priority=50;
            "rwMotorTorqueSplitter" => &mut torque_splitter, period=STEP_NANOS, priority=45;
            "attErrLog" => &mut guidance_log, period=sample_nanos, priority=0;
            "rwCmdLog" => &mut requested_torque_log, period=sample_nanos, priority=0;
        }
        for log in servicer_wheel_logs.iter_mut().chain(&mut debris_wheel_logs) {
            simulation.add_module("rwLog", log, sample_nanos, 0);
        }
        for log in &mut state_logs {
            simulation.add_module("stateLog", log, sample_nanos, 0);
        }
        for log in &mut diagnostics_logs {
            simulation.add_module("diagnosticsLog", log, sample_nanos, 0);
        }
        if write_csv {
            simulation.add_module("csv", &mut csv, sample_nanos, 0);
        }
        simulation.run_for(duration_nanos);
    }
    if write_csv {
        csv.flush().expect("flush scenario CSV");
    }

    Results {
        guidance: guidance_log,
        requested_torque: requested_torque_log,
        servicer_wheels: servicer_wheel_logs,
        debris_wheels: debris_wheel_logs,
        states: state_logs,
        debris_diagnostics: diagnostics_logs,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env(USAGE);
    cli.finish();
    let results = run(true);
    println!(
        "wrote {} (final |sigma_BR| = {:.3e}, debris wheels at {:.1} and {:.1} RPM)",
        results.csv_path.as_ref().expect("CSV written").display(),
        results.guidance.last().sigma_br.norm(),
        results.debris_wheels[0].last().omega_radps * 60.0 / std::f64::consts::TAU,
        results.debris_wheels[1].last().omega_radps * 60.0 / std::f64::consts::TAU,
    );
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioFormationBasic.py` only checks that the
    //! scenario runs. These tests check what it demonstrates.

    use super::*;

    #[test]
    fn scenario_behaves_as_upstream_describes() {
        let results = run(false);
        assert_eq!(results.guidance.values.len(), 101);

        // "with this maneuver the RW devices are being saturated, and the
        // attitude still eventually stabilizes."
        let saturated = results.servicer_wheels.iter().any(|wheel| {
            wheel
                .values
                .iter()
                .any(|state| (state.applied_motor_torque_nm.abs() - 0.2).abs() < 1e-12)
        });
        assert!(saturated, "no servicer wheel reached its 0.2 N m limit");
        let final_guidance = results.guidance.last();
        assert!(
            final_guidance.sigma_br.norm() < 1e-3,
            "sigma_BR = {}",
            final_guidance.sigma_br
        );
        assert!(final_guidance.omega_br_b_radps.norm() < 1e-4);

        // The debris's wheels spin freely: no motor torque, so each keeps its
        // inertial spin `Omega + g_s . omega_BN` while the tumbling body's
        // rate about its axis changes.
        for (index, (wheel, rpm)) in results
            .debris_wheels
            .iter()
            .zip([1000.0, -1000.0])
            .enumerate()
        {
            let axis = Vector3::ith(index, 1.0);
            let initial =
                rpm * std::f64::consts::TAU / 60.0 + axis.dot(&Vector3::new(0.001, -0.01, 0.03));
            for (state, hub) in wheel.values.iter().zip(&results.states[1].values) {
                assert_eq!(state.applied_motor_torque_nm, 0.0);
                let spin = state.omega_radps + axis.dot(&hub.omega_radps);
                assert!(
                    (spin - initial).abs() < 1e-9 * initial.abs(),
                    "debris wheel {index} spins at {spin} rad/s, initially {initial}"
                );
            }
        }

        // No torque acts on either debris object: their angular momentum
        // about their center of mass is conserved, to the RK4 truncation
        // error (the debris's includes its wheels, integrated in step with
        // the servicer).
        for log in &results.debris_diagnostics {
            let initial = log.values[0].rotational_angular_momentum_inertial_kg_m2ps;
            for diagnostics in &log.values {
                let momentum = diagnostics.rotational_angular_momentum_inertial_kg_m2ps;
                assert!(
                    (momentum - initial).norm() < 1e-8 * initial.norm(),
                    "H = {momentum}, initially {initial}"
                );
            }
        }

        // The debris flies the 2:1 relative ellipse of Clohessy-Wiltshire,
        // `a e (-cos nt, 2 sin nt)` (`a e = 10 m`), about the servicer, and the
        // debris satellite stays 40 m ahead.
        let [servicer, debris, debris_sat] = &results.states;
        let a_e = 10_000_000.0 * 1e-6;
        let mean_motion = (Body::Earth.mu_m3ps2() / 1.0e7_f64.powi(3)).sqrt();
        for ((time_s, chief), (deputy, follower)) in servicer
            .times_s()
            .into_iter()
            .zip(&servicer.values)
            .zip(debris.values.iter().zip(&debris_sat.values))
        {
            let phase = mean_motion * time_s;
            let expected = a_e * (phase.cos().powi(2) + 4.0 * phase.sin().powi(2)).sqrt();
            let ellipse = (deputy.position_m - chief.position_m).norm();
            assert!(
                (ellipse - expected).abs() < 0.05,
                "debris {ellipse} m from the servicer at {time_s} s, expected {expected} m"
            );
            let lead = (follower.position_m - chief.position_m).norm();
            assert!((lead - 40.0).abs() < 0.01, "lead-follower at {lead} m");
        }
    }
}
