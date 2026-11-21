//! Hill-frame tracking with an MRP steering outer loop and a rate-servo inner
//! loop on reaction wheels: port of upstream
//! `examples/scenarioAttitudeSteering.py`.
//!
//! A 750 kg spacecraft (inertia `diag(500, 300, 200) kg m^2`) on the
//! 10 000 km, `e = 0.01` Earth orbit carries three Honeywell HR16 wheels
//! (50 N m s, body axes, 100/200/300 RPM). `hillPoint` makes the Hill frame
//! the reference; `mrpSteering` (`K3 = 0.75`, `omega_max = 1 deg/s`) turns the
//! attitude error into a commanded rate that `rateServoFullNonlinear`
//! (`P = 150`) tracks, and `rwMotorTorque` maps the servo torque onto the
//! wheels. One 0.1 s task, 10 minutes, histories every 3 s
//! (`samplingTime(T, 0.1 s, 200)`).
//!
//! Upstream `run(show_plots, simCase)` maps to `--sim-case <0-3>`; `--all`
//! runs the four cases of upstream's `test_scenarioAttitudeSteering.py`:
//!
//! * 0: large initial error (`sigma_BN = [0.5, 0.6, -0.3]`,
//!   `omega_BN = [0.01, -0.01, -0.01] rad/s`), an unknown
//!   `[0.01, -0.01, 0.005] N m` disturbance, `K1 = 0.05`, servo integral
//!   feedback `Ki = 5`;
//! * 1: case 0 without the integral feedback (`Ki = -1`);
//! * 2: no disturbance, `K1 = 2.2` (beyond the sub-servo separation), a start
//!   near the Hill frame with the Hill-frame rate, `Ki = 5`;
//! * 3: case 2 with the outer-loop feed-forward `omega'_B*/R` ignored.
//!
//! The start of cases 2 and 3 is upstream's: `BH = BN * HN` with numpy arrays
//! is an element-by-element product, not a matrix product, so the initial
//! attitude is the MRP (`C2MRP`) of that element-wise product of
//! `MRP2C([0.001, 0.002, -0.003])` and `[HN] = euler3132C([Omega, i,
//! omega + f])`. It is reproduced here as written; it starts these cases with
//! a sizeable attitude error rather than the "small initial attitude" the
//! upstream text describes.
//!
//! As in the other reaction-wheel scenarios, `arrayMotorTorqueToSingleActuators`
//! carries `rwMotorTorque`'s array command to the per-wheel motor inputs.
//!
//! Writes `examples/output/scenarios/scenarioAttitudeSteering<simCase>.csv`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/rw_factory.rs"]
mod rw_factory;

use std::path::PathBuf;

use attitude::{
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, HubState, ScenarioOrbit, add_point_mass_earth,
    att_tracking_error, external_disturbance, hill_point, sampling_period_nanos, scenario_recorder,
    spacecraft, vehicle_config,
};
use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffector,
};
use basilisk_rs::fsw_algorithms::att_control::mrp_steering::{MrpSteering, MrpSteeringConfig};
use basilisk_rs::fsw_algorithms::att_control::rate_servo_full_nonlinear::{
    RateServoFullNonlinear, RateServoFullNonlinearConfig,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, NavigationTranslationMsg, Output, RateCommandMsg,
    ReactionWheelStateMsg, RwSpeedMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::rigid_body_kinematics::{dcm_to_mrp, euler313_to_dcm, mrp_to_dcm};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{array_columns, vector_columns};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use rw_factory::RwFactory;

const WHEEL_COUNT: usize = 3;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    sim_case: u8,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(sim_case: u8) -> Self {
        Self {
            sim_case,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!("scenarioAttitudeSteering{}", self.sim_case)
    }
}

/// The parametrizations of upstream `test_scenarioAttitudeSteering.py`.
const TEST_PARAMETRIZATIONS: [Config; 4] = [
    Config::new(0),
    Config::new(1),
    Config::new(2),
    Config::new(3),
];

/// The recorded histories.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    requested_torque: MessageLog<ArrayMotorTorqueMsg>,
    wheels: Vec<MessageLog<ReactionWheelStateMsg>>,
    csv_path: Option<PathBuf>,
}

/// The initial hub state of `sim_case`.
fn initial_state(sim_case: u8) -> HubState {
    let orbit = ScenarioOrbit::new(0.01);
    let mut state = HubState::on_orbit(0.01);
    if sim_case < 2 {
        state.sigma_bn = Vector3::new(0.5, 0.6, -0.3);
        state.omega_bn_b_radps = Vector3::new(0.01, -0.01, -0.01);
    } else {
        let dcm_hn = euler313_to_dcm(Vector3::new(
            orbit.raan_rad,
            orbit.inclination_rad,
            orbit.argument_of_periapsis_rad + orbit.true_anomaly_rad,
        ));
        let dcm_bn = mrp_to_dcm(Vector3::new(0.001, 0.002, -0.003));
        // Upstream's `BH = BN * HN` on numpy arrays: element by element.
        let dcm_bh: Matrix3<f64> = dcm_bn.component_mul(&dcm_hn);
        state.sigma_bn = dcm_to_mrp(dcm_bh);
        state.omega_bn_b_radps = orbit.mean_motion_radps() * dcm_hn.row(2).transpose();
    }
    state
}

fn run(config: Config) -> Results {
    let sim_case = config.sim_case;
    assert!(sim_case <= 3, "simCase must be 0, 1, 2 or 3");
    let inertia = Matrix3::from_diagonal(&Vector3::new(500.0, 300.0, 200.0));

    let model = ReactionWheelModel::BalancedWheels;
    let mut rw_factory = RwFactory::default();
    for (axis, rpm) in [
        (Vector3::x(), 100.0),
        (Vector3::y(), 200.0),
        (Vector3::z(), 300.0),
    ] {
        rw_factory.create_honeywell_hr16(axis, 50.0, rpm, Vector3::zeros(), model);
    }
    // Upstream `fswSetupRW.create(gsHat_B, Js, 0.2)` for each wheel.
    let rw_config_msg = Output::new(rw_factory.config_message());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let mut reaction_wheels = ReactionWheelStateEffector::new("spacecraftBody");
    for wheel in rw_factory.wheels {
        reaction_wheels.add_reaction_wheel(wheel);
    }
    let wheel_speed_msg = reaction_wheels.rw_speed_out_msg.clone();
    let wheel_state_msgs: [Output<ReactionWheelStateMsg>; WHEEL_COUNT] =
        std::array::from_fn(|index| reaction_wheels.wheels()[index].state_out.clone());

    let mut spacecraft = spacecraft(inertia, &initial_state(sim_case));
    add_point_mass_earth(&mut spacecraft);
    if sim_case < 2 {
        spacecraft.add_dynamic_effector(external_disturbance(Vector3::new(0.01, -0.01, 0.005)));
    }
    let mut navigation = SimpleNavigation::new("SimpleNavigation");

    let mut guidance = hill_point();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let mut steering = MrpSteering::new(MrpSteeringConfig {
        name: "MRP_Steering".to_string(),
        k1: if sim_case < 2 { 0.05 } else { 2.2 },
        k3: 0.75,
        omega_max_radps: 1.0_f64.to_radians(),
        ignore_outer_loop_feedforward: sim_case == 3,
    });
    let servo_ki = if sim_case == 1 { -1.0 } else { 5.0 };
    let mut servo = RateServoFullNonlinear::new(RateServoFullNonlinearConfig {
        name: "rate_servo".to_string(),
        p: 150.0,
        ki: servo_ki,
        known_torque_body_nm: Vector3::zeros(),
        integral_limit: 2.0 / servo_ki * 0.1,
    });
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    let mut torque_splitter =
        ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "rwMotorTorqueSplitter".to_string(),
            num_actuators: WHEEL_COUNT,
        });

    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 200);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut requested_torque_log = MessageLog::<ArrayMotorTorqueMsg>::default();
    let mut wheel_logs: Vec<MessageLog<ReactionWheelStateMsg>> =
        (0..WHEEL_COUNT).map(|_| MessageLog::default()).collect();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    for (command, wheel) in torque_splitter
        .actuator_out_msgs
        .iter()
        .zip(reaction_wheels.wheels_mut())
    {
        simulation.connect(command, &mut wheel.motor_torque_in_msg);
    }
    spacecraft.add_state_effector(reaction_wheels);

    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.translation_out_msg => &mut guidance.trans_nav_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &guidance.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut steering.guid_in_msg,
        &tracking_error.att_guid_out_msg => &mut servo.guid_in_msg,
        &vehicle_config_msg => &mut servo.veh_config_in_msg,
        &rw_config_msg => &mut servo.rw_params_in_msg,
        &wheel_speed_msg => &mut servo.rw_speeds_in_msg,
        &steering.rate_cmd_out_msg => &mut servo.rate_steering_in_msg,
        &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
        &servo.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut torque_splitter.torque_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut requested_torque_log.input,
        &rw_motor_torque.rw_motor_torque_out_msg => recorder.add_source::<ArrayMotorTorqueMsg>(
            CsvSourceConfig::columns(array_columns(
                "motor_torque_nm",
                "motorTorque_Nm",
                WHEEL_COUNT,
            )),
        ),
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
        &navigation.translation_out_msg => recorder.add_source::<NavigationTranslationMsg>(
            CsvSourceConfig::columns(vector_columns("position_bn_n_m", "r_BN_N_m")),
        ),
        &wheel_speed_msg => recorder.add_source::<RwSpeedMsg>(
            CsvSourceConfig::columns(array_columns(
                "wheel_speeds_radps",
                "wheelSpeeds_rad_per_s",
                WHEEL_COUNT,
            )),
        ),
        &steering.rate_cmd_out_msg => recorder.add_source::<RateCommandMsg>(
            CsvSourceConfig::columns(vector_columns(
                "omega_bast_r_b_radps",
                "omega_BastR_B_rad_per_s",
            )),
        ),
    );
    for (index, (state, log)) in wheel_state_msgs.iter().zip(&mut wheel_logs).enumerate() {
        connect!(&simulation,
            state => &mut log.input,
            state => recorder.add_source::<ReactionWheelStateMsg>(CsvSourceConfig::columns([(
                "applied_motor_torque_nm".to_string(),
                format!("rwTorque_Nm_{index}"),
            )])),
        );
    }
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        "hill_point" => &mut guidance, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "tracking_error" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "mrp_steering" => &mut steering, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "rate_servo" => &mut servo, period=DYNAMICS_PERIOD_NANOS, priority=50;
        "rw_motor_torque" => &mut rw_motor_torque, period=DYNAMICS_PERIOD_NANOS, priority=40;
        "rw_motor_torque_splitter" => &mut torque_splitter, period=DYNAMICS_PERIOD_NANOS, priority=35;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "requested_torque_log" => &mut requested_torque_log, period=sample_period, priority=0;
    }
    for log in &mut wheel_logs {
        simulation.add_module("wheel_log", log, sample_period, 0);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        requested_torque: requested_torque_log,
        wheels: wheel_logs,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_attitude_steering [--sim-case <0|1|2|3>] [--all]");
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream's `__main__` runs `run(True, 0)`.
        vec![Config::new(cli.value("sim-case", 0))]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!(
            "wrote {} (|sigma_BR| {:.3e} -> {:.3e})",
            results.csv_path.expect("CSV written").display(),
            results.guidance.values[0].sigma_br.norm(),
            results.guidance.last().sigma_br.norm(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttitudeSteering.py` only checks that each case
    //! runs. These tests pin the upstream initial attitude of cases 2 and 3
    //! and check the behavior the scenario documents for each case.

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    fn error_norms(results: &Results, since_nanos: u64) -> Vec<f64> {
        results
            .guidance
            .since(since_nanos)
            .map(|guidance| guidance.sigma_br.norm())
            .collect()
    }

    /// `C2MRP(MRP2C([0.001, 0.002, -0.003]) * euler3132C([Omega, i, omega +
    /// f]))` as upstream's numpy evaluates it (an element-wise product).
    #[test]
    fn cases_2_and_3_start_from_upstreams_element_wise_attitude() {
        let expected = Vector3::new(
            -0.000_284_518_364_959_472_46,
            0.000_632_111_224_045_289,
            0.834_864_983_086_920_9,
        );
        let sigma_bn = initial_state(2).sigma_bn;
        assert!((sigma_bn - expected).norm() < 1.0e-12, "{sigma_bn}");
    }

    /// "with this maneuver the RW devices are being saturated, and the
    /// attitude still eventually stabilizes. ... the unknown external torque
    /// is automatically compensated for to yield exponential convergence."
    #[test]
    fn case_0_saturates_the_wheels_and_still_converges() {
        let results = quiet(TEST_PARAMETRIZATIONS[0]);
        // samplingTime(10 min, 0.1 s, 200) = 3 s: 201 samples.
        assert_eq!(results.guidance.values.len(), 201);
        let requested_peak = results
            .requested_torque
            .values
            .iter()
            .flat_map(|command| command.motor_torque_nm[..WHEEL_COUNT].to_vec())
            .fold(0.0_f64, |peak, torque| peak.max(torque.abs()));
        assert!(requested_peak > 0.2, "{requested_peak}");
        for wheel in &results.wheels {
            assert!(
                wheel
                    .values
                    .iter()
                    .all(|state| state.applied_motor_torque_nm.abs() <= 0.2 + 1.0e-12)
            );
        }
        let norms = error_norms(&results, 0);
        assert!(norms[0] > 0.9);
        assert!(*norms.last().unwrap() < 1.0e-3);
        // Exponential: the error halves at least every two minutes.
        let at = |seconds: u64| results.guidance.at(seconds * 1_000_000_000).sigma_br.norm();
        assert!(at(600) < 0.5 * at(480) && at(480) < 0.5 * at(360));
    }

    /// Without the servo's integral feedback the disturbance leaves an offset
    /// that case 0's integral feedback removes.
    #[test]
    fn case_1_keeps_an_offset_without_integral_feedback() {
        let with_integral = quiet(TEST_PARAMETRIZATIONS[0]);
        let without = quiet(TEST_PARAMETRIZATIONS[1]);
        let offset = without.guidance.last().sigma_br.norm();
        assert!(offset > 2.0 * with_integral.guidance.last().sigma_br.norm());
        // Settled: the last two minutes stay within 20 % of the offset.
        for norm in error_norms(&without, 480_000_000_000) {
            assert!((norm - offset).abs() < 0.2 * offset, "{norm} vs {offset}");
        }
    }

    /// Case 2's outer-loop gain violates the sub-servo separation: the error
    /// stops converging ("the local motion is now unstable"). Dropping the
    /// outer-loop feed-forward (case 3) makes the motion locally stable
    /// again.
    #[test]
    fn the_outer_loop_feed_forward_decides_local_stability() {
        let unstable = quiet(TEST_PARAMETRIZATIONS[2]);
        let stable = quiet(TEST_PARAMETRIZATIONS[3]);
        assert_eq!(
            unstable.guidance.values[0].sigma_br,
            stable.guidance.values[0].sigma_br
        );
        for norm in error_norms(&unstable, 300_000_000_000) {
            assert!(norm > 1.0e-3, "{norm}");
        }
        assert!(stable.guidance.last().sigma_br.norm() < 1.0e-8);
    }
}
