//! Port of upstream `examples/scenarioOrbitManeuverTH.py`: the LEO-to-GEO
//! Hohmann transfer of `scenario_orbit_maneuver`, flown with finite thruster
//! burns while the attitude control keeps the thrust axis along the velocity.
//!
//! A 750 kg spacecraft with two thrusters along body +z
//! ([`ThrusterStateEffector`], 0.1 rad/s cutoff, Isp 318 s): 3000 N for the
//! transfer burn and 420 N for the circularization. `velocityPoint`
//! guidance, `attTrackingError` with the 90 degree offset `sigma_R0R =
//! [tan(pi/8), 0, 0]` that points body +z along the velocity, and
//! `mrpFeedback` driving an external torque, on the navigation solution;
//! adaptive RKF45 integration with upstream's tolerances; a 2 s task.
//!
//! The burns are on-time commands written between runs (upstream
//! `thrOnTimeMsg.write(..., time=...)`): one [`ThrusterCommandMsg`] per
//! thruster, stamped with the write time (`command_time_nanos`, the
//! stand-in for the message header). Burn durations follow from the impulsive
//! Hohmann delta-v at full thrust, as upstream computes them.
//!
//! Writes `scenarioOrbitManeuverTH.csv` (state, torque command, attitude
//! error and thruster forces). Upstream has no test for this scenario;
//! `tests` checks that the burns fire as commanded and the spacecraft ends
//! close to GEO.

#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;

use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::thruster_state_effector::{
    ThrusterStateEffector, ThrusterStateEffectorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::velocity_point::{
    VelocityPoint, VelocityPointConfig,
};
use basilisk_rs::fsw_algorithms::att_tracking_error::{AttTrackingError, AttTrackingErrorConfig};
use basilisk_rs::fsw_algorithms::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::integrators::{AdaptiveController, Rkf45};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, Output, SpacecraftStateMsg, ThrusterCommandMsg,
    ThrusterOutputMsg, VehicleConfigMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use basilisk_rs::{connect, schedule};
use common::vector_columns;
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{orbit_period_s, run_until, sec2nano};
use recording::{sampling_time, scenario_csv, state_columns};

const STEP_NANOS: u64 = 2_000_000_000;
const HUB_MASS_KG: f64 = 750.0;
/// Upstream `maxThrustTH1`, `maxThrustTH2` (N).
const MAX_THRUST_TH1_N: f64 = 3000.0;
const MAX_THRUST_TH2_N: f64 = 420.0;

pub struct Results {
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    /// Thrust histories of the two thrusters, every task step.
    pub thrust: [Vec<(u64, ThrusterOutputMsg)>; 2],
    /// Upstream `t0Burn` and `t1Burn` (s), and the times they were commanded (ns).
    pub burns_s: [f64; 2],
    pub burn_starts_nanos: [u64; 2],
    pub geo_radius_m: f64,
    pub final_position_m: Vector3<f64>,
    pub final_velocity_mps: Vector3<f64>,
}

fn thruster(name: &str, max_thrust_n: f64) -> ThrusterStateEffectorConfig {
    ThrusterStateEffectorConfig {
        min_on_time_s: 0.020,
        cutoff_frequency_radps: 0.1,
        area_nozzle_m2: 0.046759,
        ..ThrusterStateEffectorConfig::new(name, Vector3::z(), Vector3::z(), max_thrust_n, 318.0)
    }
}

pub fn run() -> Results {
    let degrees = std::f64::consts::PI / 180.0;
    let inertia = Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0));
    let mu = Body::Earth.mu_m3ps2();
    let r_geo = (mu / (2.0 * std::f64::consts::PI / (24.0 * 3600.0)).powi(2)).cbrt();
    let elements = ClassicElements {
        semi_major_axis: 7_000.0 * 1_000.0,
        eccentricity: 0.0001,
        inclination_rad: 0.0,
        raan_rad: 48.2 * degrees,
        arg_periapsis_rad: 347.8 * degrees,
        true_anomaly_rad: 85.3 * degrees,
        ..Default::default()
    };
    let (position_m, velocity_mps) = elements_to_state(mu, &elements);
    let period_s = orbit_period_s(mu, elements.semi_major_axis);
    let first_stop_nanos = sec2nano(0.25 * period_s);
    let sample_nanos = sampling_time(first_stop_nanos, STEP_NANOS, 100);

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: inertia,
        integration_step_nanos: STEP_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.0, 0.03, 0.01),
        // Upstream `svIntegratorRKF45` with its default tolerances.
        integrator: Some(Box::new(Rkf45::new(AdaptiveController {
            atol: 1.0e-8,
            rtol: 1.0e-4,
            safety: 0.9,
            min_scale: 0.1,
            max_scale: 4.0,
            ..AdaptiveController::default()
        }))),
    });
    spacecraft
        .add_grav_body(Body::Earth.point_mass(true))
        .expect("one central body");
    let hub = spacecraft.hub_state_handle();

    let mut ext_torque = ExtForceTorque::new(ExtForceTorqueConfig {
        name: "externalTorque".to_string(),
        ..ExtForceTorqueConfig::default()
    });
    let mut thrusters = ThrusterStateEffector::new("GTOThrusterDynamics");
    thrusters.add_thruster(thruster("TH1", MAX_THRUST_TH1_N));
    thrusters.add_thruster(thruster("TH2", MAX_THRUST_TH2_N));
    let commands = [
        Output::new(ThrusterCommandMsg::default()),
        Output::new(ThrusterCommandMsg::default()),
    ];
    let thrust_outputs: [Output<ThrusterOutputMsg>; 2] =
        std::array::from_fn(|index| thrusters.thrusters()[index].thruster_out_msg.clone());

    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut guidance = VelocityPoint::new(VelocityPointConfig {
        name: "velocityPoint".to_string(),
        mu_m3ps2: mu,
    });
    let mut tracking_error = AttTrackingError::new(AttTrackingErrorConfig {
        name: "attErrorVelocityPoint".to_string(),
        sigma_r0r: Vector3::new((std::f64::consts::PI / 8.0).tan(), 0.0, 0.0),
    });
    let mut control = MrpFeedback::new(MrpFeedbackConfig {
        name: "mrpFeedback".to_string(),
        k: 3.5,
        ki: -1.0,
        p: 30.0,
        integral_limit: 2.0 / -1.0 * 0.1,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 0,
    });
    let vehicle_config = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: inertia,
        ..VehicleConfigMsg::default()
    });

    let mut csv = scenario_csv("scenarioOrbitManeuverTH");
    let mut history = MessageLog::<SpacecraftStateMsg>::default();
    let mut thrust_histories = [
        MessageLog::<ThrusterOutputMsg>::default(),
        MessageLog::default(),
    ];
    let mut burns_s = [0.0; 2];
    let mut burn_starts_nanos = [0; 2];
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        for (command, thruster) in commands.iter().zip(thrusters.thrusters_mut()) {
            simulation.connect(command, &mut thruster.command_in_msg);
        }
        connect!(&simulation,
            &control.cmd_torque_out_msg => &mut ext_torque.cmd_torque_in_msg,
            &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
            &navigation.translation_out_msg => &mut guidance.trans_nav_in_msg,
            &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
            &guidance.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
            &tracking_error.att_guid_out_msg => &mut control.guid_in_msg,
            &vehicle_config => &mut control.veh_config_in_msg,
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &control.cmd_torque_out_msg => csv.add_source::<BodyTorqueCommandMsg>(
                CsvSourceConfig::columns(vector_columns("torque_request_body_nm", "torqueRequestBody_Nm")),
            ),
            &tracking_error.att_guid_out_msg => csv.add_source::<AttitudeGuidanceMsg>(
                CsvSourceConfig::columns(vector_columns("sigma_br", "sigma_BR")),
            ),
            &thrust_outputs[0] => csv.add_source::<ThrusterOutputMsg>(
                CsvSourceConfig::columns([("thrust_force_n", "thrustForce_N_0")]),
            ),
            &thrust_outputs[1] => csv.add_source::<ThrusterOutputMsg>(
                CsvSourceConfig::columns([("thrust_force_n", "thrustForce_N_1")]),
            ),
            &spacecraft.state_out => &mut history.input,
            &thrust_outputs[0] => &mut thrust_histories[0].input,
            &thrust_outputs[1] => &mut thrust_histories[1].input,
        );
        spacecraft.add_dynamic_effector(ext_torque);
        spacecraft.add_state_effector(thrusters);
        let [thrust_history_0, thrust_history_1] = &mut thrust_histories;
        schedule! { simulation,
            "bsk-Sat" => &mut spacecraft, period=STEP_NANOS, priority=100;
            "SimpleNavigation" => &mut navigation, period=STEP_NANOS, priority=90;
            "velocityPoint" => &mut guidance, period=STEP_NANOS, priority=80;
            "attErrorVelocityPoint" => &mut tracking_error, period=STEP_NANOS, priority=70;
            "mrpFeedback" => &mut control, period=STEP_NANOS, priority=60;
            "csv" => &mut csv, period=sample_nanos, priority=0;
            "dataRec" => &mut history, period=sample_nanos, priority=0;
            "thrCmdRec0" => thrust_history_0, period=STEP_NANOS, priority=0;
            "thrCmdRec1" => thrust_history_1, period=STEP_NANOS, priority=0;
        }
        run_until(&mut simulation, first_stop_nanos);

        // Transfer burn with TH1: upstream `t0Burn = dv0 / (F / m)`.
        let (r0, v0) = (hub.position_m().norm(), hub.velocity_mps().norm());
        let transfer_a = (r0 + r_geo) * 0.5;
        let dv0 = (mu / transfer_a * r_geo / r0).sqrt() - v0;
        burns_s[0] = dv0 / (MAX_THRUST_TH1_N / HUB_MASS_KG);
        burn_starts_nanos[0] = first_stop_nanos;
        commands[0].write(ThrusterCommandMsg {
            on_time_s: burns_s[0],
            command_time_nanos: first_stop_nanos,
        });
        let t2_nanos = sec2nano(burns_s[0]);
        run_until(&mut simulation, first_stop_nanos + t2_nanos);

        // Coast to apoapsis: upstream `T3 = pi / n1 - T3P`.
        let oe1 = state_to_elements(mu, hub.position_m(), hub.velocity_mps());
        let n1 = (mu / oe1.semi_major_axis.powi(3)).sqrt();
        let e1 = 2.0
            * (((1.0 - oe1.eccentricity) / (1.0 + oe1.eccentricity)).sqrt()
                * (oe1.true_anomaly_rad / 2.0).tan())
            .atan();
        let t3p = (oe1.semi_major_axis.powi(3) / mu).sqrt() * (e1 - oe1.eccentricity * e1.sin());
        let t3_nanos = sec2nano(std::f64::consts::PI / n1 - t3p);
        run_until(&mut simulation, first_stop_nanos + t2_nanos + t3_nanos);

        // Circularization burn with TH2.
        let (r1, v1) = (hub.position_m().norm(), hub.velocity_mps().norm());
        let dv1 = (mu / r1).sqrt() - v1;
        let n2 = (mu / r1.powi(3)).sqrt();
        burns_s[1] = dv1 / (MAX_THRUST_TH2_N / HUB_MASS_KG);
        let current_nanos = first_stop_nanos + t2_nanos + t3_nanos;
        burn_starts_nanos[1] = current_nanos;
        for (command, on_time_s) in commands.iter().zip([0.0, burns_s[1]]) {
            command.write(ThrusterCommandMsg {
                on_time_s,
                command_time_nanos: current_nanos,
            });
        }
        let t4_nanos = sec2nano(burns_s[1]);
        let t5_nanos = sec2nano(0.25 * std::f64::consts::PI / n2);
        run_until(&mut simulation, current_nanos + t4_nanos + t5_nanos);
    }
    csv.flush().expect("flush scenario CSV");

    let [thrust_0, thrust_1] = thrust_histories;
    Results {
        states: history.into_samples(),
        thrust: [thrust_0.into_samples(), thrust_1.into_samples()],
        burns_s,
        burn_starts_nanos,
        geo_radius_m: r_geo,
        final_position_m: hub.position_m(),
        final_velocity_mps: hub.velocity_mps(),
    }
}

fn main() {
    let results = run();
    let elements = state_to_elements(
        Body::Earth.mu_m3ps2(),
        results.final_position_m,
        results.final_velocity_mps,
    );
    println!(
        "wrote scenarioOrbitManeuverTH.csv: burns {:.1} s and {:.1} s; final radius {:.1} km (GEO {:.1} km), a = {:.1} km, e = {:.4}",
        results.burns_s[0],
        results.burns_s[1],
        results.final_position_m.norm() / 1e3,
        results.geo_radius_m / 1e3,
        elements.semi_major_axis / 1e3,
        elements.eccentricity
    );
}

#[cfg(test)]
mod tests {
    use basilisk_rs::utilities::orbital_motion::state_to_elements;

    use super::{MAX_THRUST_TH1_N, MAX_THRUST_TH2_N, run};
    use crate::grav_factory::Body;
    use crate::orbit_tools::nano2sec;

    /// No upstream test. Each thruster delivers its commanded impulse, full
    /// thrust for the on-time (the 0.1 rad/s first-order lag shifts the thrust
    /// but keeps the impulse), and fires only after its command; the finite
    /// burns leave a near-GEO orbit.
    #[test]
    fn burns_fire_as_commanded_and_raise_the_orbit_to_geo() {
        let results = run();
        for (index, max_thrust_n) in [MAX_THRUST_TH1_N, MAX_THRUST_TH2_N].into_iter().enumerate() {
            let samples = &results.thrust[index];
            let impulse_ns: f64 = samples
                .windows(2)
                .map(|pair| pair[1].1.thrust_force_n * nano2sec(pair[1].0 - pair[0].0))
                .sum();
            let commanded_ns = max_thrust_n * results.burns_s[index];
            assert!(
                (impulse_ns / commanded_ns - 1.0).abs() < 0.02,
                "thruster {index}: impulse {impulse_ns} N s, commanded {commanded_ns} N s"
            );
            assert!(
                samples
                    .iter()
                    .filter(|(time, _)| *time <= results.burn_starts_nanos[index])
                    .all(|(_, output)| output.thrust_force_n == 0.0),
                "thruster {index} fired before its command"
            );
        }
        let elements = state_to_elements(
            Body::Earth.mu_m3ps2(),
            results.final_position_m,
            results.final_velocity_mps,
        );
        let radius_error = results.final_position_m.norm() / results.geo_radius_m - 1.0;
        assert!(
            radius_error.abs() < 0.01,
            "final radius off GEO by {radius_error}"
        );
        assert!(
            (elements.semi_major_axis / results.geo_radius_m - 1.0).abs() < 0.05,
            "a = {} m",
            elements.semi_major_axis
        );
        assert!(
            elements.eccentricity < 0.05,
            "e = {}",
            elements.eccentricity
        );
    }
}
