//! A servicer approaching, circling and parking next to a debris object: port
//! of upstream `examples/scenarioRendezVous.py`.
//!
//! The 750 kg reaction-wheel servicer of `scenarioFormationBasic` (three
//! HR16 wheels, `mrpFeedback` and `rwMotorTorque`) starts 10 m below and
//! 40 m behind a 350 kg tumbling debris object on a circular 10 000 km orbit
//! (`i = 33.3°`, `Ω = 48.2°`, `ω = 90°`, `f = 0`). Both orbit a point-mass
//! Earth perturbed by the Sun, whose state relative to the Earth comes from
//! the ANISE ephemeris (upstream `createSpiceInterface` with
//! `zeroBase = 'Earth'`, from 2022 May 1 00:28:30 UTC; see
//! `support/grav_factory.rs`) through `ephemerisConverter`. One 1 s task.
//!
//! Three phases, run back to back as upstream's `runSim` calls, each starting
//! with an impulsive change of the servicer's velocity (upstream's
//! `servicerVel.setState`, here the [`HubStateHandle`](basilisk_rs::spacecraft::HubStateHandle))
//! and a switch of the attitude reference `attTrackingError` follows
//! (upstream re-subscribes `attRefInMsg`; here the switch is staged on the
//! module, see `support/staged.rs`):
//! 1. 1/8 orbit: along-track drift (`relOrbDrift`), Hill-frame pointing
//!    (`hillPoint`, `sigma_R0R = [0, 0, -0.414214]`);
//! 2. 2 orbits: a drifting circumnavigation (`relativeEllipse(10, -1, 10)`),
//!    the servicer's `b1` aimed at the debris (`locationPointing` on the
//!    debris's navigation message);
//! 3. 1/2 orbit: a 2:1 bounded ellipse (`relativeEllipse(10, 0, 0)`), the
//!    solar panels' `b3` aimed at the Sun (`locationPointing` on the Sun's
//!    ephemeris).
//!
//! The ephemeris modules run before the spacecraft (upstream runs its SPICE
//! module after them and Euler-steps the Sun's position by the message age,
//! which basilisk-rs's gravity bodies have no header for: a 1 s difference in
//! the Sun's position, about 30 km of 1 AU). The wheels take one command
//! each, through `arrayMotorTorqueToSingleActuators`. Upstream's Vizard light
//! is display only.
//!
//! Upstream `run(show_plots)` has no parameters. Writes
//! `examples/output/scenarios/scenarioRendezVous.csv`, every 1 s as upstream
//! records; `plot_scenario_rendez_vous.py` redraws upstream's figures from
//! it.

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
#[path = "support/rw_factory.rs"]
mod rw_factory;
#[path = "support/staged.rs"]
mod staged;

use std::path::PathBuf;

use attitude::{att_tracking_error, hill_point, scenario_recorder, vehicle_config};
use basilisk_rs::environment::ephemeris_converter::{EphemerisConverter, EphemerisConverterConfig};
use basilisk_rs::fsw_algorithms::att_guidance::location_pointing::{
    LocationPointing, LocationPointingConfig,
};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, MAX_EFF_COUNT, PlanetStateMsg, ReactionWheelStateMsg,
    RwSpeedMsg, SpacecraftStateMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, hill_to_state, state_to_hill,
};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{array_columns, vector_columns};
use formation::{InitialState, hub_inertia_kg_m2, rigid_spacecraft, spacecraft_state_columns};
use formation_servicer::{SERVICER_MASS_KG, SERVICER_WHEELS, servicer_wheels, wheel_control};
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{run_until, sec2nano};
use staged::Staged;

const USAGE: &str = "scenario_rendez_vous   (upstream `run(show_plots)` has no parameters)";

/// Upstream `simulationTimeStep = macros.sec2nano(1)`.
const STEP_NANOS: u64 = 1_000_000_000;
/// Upstream debris `mHub` \[kg\].
const DEBRIS_MASS_KG: f64 = 350.0;

/// The recorded histories, every task step (upstream's
/// `samplingTime(0, ...)`, computed before the run time is known, is 1 ns).
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Upstream `attErrLog`.
    guidance: MessageLog<AttitudeGuidanceMsg>,
    /// Upstream `rwSc1Log`.
    wheels: Vec<MessageLog<ReactionWheelStateMsg>>,
    servicer: MessageLog<SpacecraftStateMsg>,
    debris: MessageLog<SpacecraftStateMsg>,
    /// The Sun's position relative to the Earth.
    sun: MessageLog<PlanetStateMsg>,
    /// The ends of the three phases \[ns\].
    phase_ends_nanos: [u64; 3],
    /// Debris mean motion \[rad/s\].
    mean_motion: f64,
    csv_path: Option<PathBuf>,
}

/// Upstream `relOrbDrift()`: keep the radial offset, null the radial rate and
/// set the along-track rate of a drift, `-1.5 n x`. Returns the servicer's
/// new inertial velocity.
fn rel_orb_drift(
    mean_motion: f64,
    (rc, vc): (Vector3<f64>, Vector3<f64>),
    (rd, vd): (Vector3<f64>, Vector3<f64>),
) -> Vector3<f64> {
    let (rho_h, mut rho_prime_h) = state_to_hill(rc, vc, rd, vd);
    rho_prime_h.x = 0.0;
    rho_prime_h.y = -1.5 * mean_motion * rho_h.x;
    hill_to_state(rc, vc, rho_h, rho_prime_h).1
}

/// Upstream `relativeEllipse(A0, xOff, B0)`: the relative rates of an
/// in-plane ellipse of semi-minor axis `A0` about the radial offset `xOff`
/// (drifting along-track unless `xOff = 0`) through the current position, and,
/// for `B0 >= 0`, of an out-of-plane oscillation of amplitude `B0`. Returns
/// the servicer's new inertial velocity.
fn relative_ellipse(
    mean_motion: f64,
    (rc, vc): (Vector3<f64>, Vector3<f64>),
    (rd, vd): (Vector3<f64>, Vector3<f64>),
    a0: f64,
    x_off: f64,
    b0: f64,
) -> Vector3<f64> {
    let n = mean_motion;
    let (rho_h, mut rho_prime_h) = state_to_hill(rc, vc, rd, vd);
    let alpha = ((rho_h.x - x_off) / a0).acos();
    // Upstream leaves `beta` unset for `0 < B0 <= 0.01` and negative `B0`;
    // it is only used for `B0 >= 0`.
    let beta = if b0 > 0.01 {
        (rho_h.z / b0).acos()
    } else {
        0.0
    };
    rho_prime_h.x = -a0 * n * alpha.sin();
    rho_prime_h.y = -2.0 * a0 * n * alpha.cos() - 1.5 * n * x_off;
    if b0 >= 0.0 {
        rho_prime_h.z = -b0 * n * beta.sin();
    }
    hill_to_state(rc, vc, rho_h, rho_prime_h).1
}

fn run(write_csv: bool) -> Results {
    let degrees = std::f64::consts::PI / 180.0;
    let mu = Body::Earth.mu_m3ps2();
    let debris_oe = ClassicElements {
        semi_major_axis: 10_000_000.0,
        eccentricity: 0.0,
        inclination_rad: 33.3 * degrees,
        raan_rad: 48.2 * degrees,
        arg_periapsis_rad: 90.0 * degrees,
        true_anomaly_rad: 0.0,
        ..Default::default()
    };
    let (r2n, v2n) = elements_to_state(mu, &debris_oe);
    let n = (mu / debris_oe.semi_major_axis.powi(3)).sqrt();
    let orbit_period = 2.0 * std::f64::consts::PI / n;
    let rho_h = Vector3::new(-10.0, -40.0, 0.0);
    let rho_prime_h = Vector3::new(0.0, -1.5 * n * rho_h.x, 0.0);
    let (rn, vn) = hill_to_state(r2n, v2n, rho_h, rho_prime_h);

    let inertia = hub_inertia_kg_m2();
    let mut servicer = rigid_spacecraft(
        SERVICER_MASS_KG,
        inertia,
        STEP_NANOS,
        &InitialState::at_rest((rn, vn)).with_attitude([0.1, 0.2, -0.3], [0.0, 0.0, 0.0]),
    );
    let mut debris = rigid_spacecraft(
        DEBRIS_MASS_KG,
        Matrix3::from_diagonal(&Vector3::new(600.0, 650.0, 450.0)),
        STEP_NANOS,
        &InitialState::at_rest((r2n, v2n)).with_attitude([0.3, 0.1, 0.2], [0.001, -0.01, 0.03]),
    );
    for spacecraft in [&mut servicer, &mut debris] {
        spacecraft
            .add_grav_body(Body::Sun.point_mass(false))
            .expect("sun gravity body");
        spacecraft
            .add_grav_body(Body::Earth.point_mass(true))
            .expect("one central body");
    }
    let servicer_hub = servicer.hub_state_handle();
    let debris_hub = debris.hub_state_handle();
    let mut sun_ephemeris = Body::Sun.ephemeris(Body::Earth);
    let mut earth_ephemeris = Body::Earth.ephemeris(Body::Earth);
    let mut ephemeris_converter = EphemerisConverter::new(EphemerisConverterConfig {
        name: "EphemData".to_string(),
        body_names: vec!["sun".to_string(), "earth".to_string()],
    });

    let wheels = servicer_wheels();
    let mut wheel_effector = wheels.effector;
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut debris_navigation = SimpleNavigation::new("SimpleNavigation2");
    let mut hill_pointing = hill_point();
    let pointing = |name: &str, p_hat_b: Vector3<f64>| {
        LocationPointing::new(LocationPointingConfig {
            name: name.to_string(),
            p_hat_b,
            use_boresight_rate_damping: true,
            ..LocationPointingConfig::default()
        })
    };
    let mut debris_pointing = pointing("scPointing", Vector3::x());
    let mut sun_pointing = pointing("scPointing", Vector3::z());
    let vehicle_config_msg = basilisk_rs::messages::Output::new(vehicle_config(inertia));
    let (mut controller, mut rw_motor_torque, mut torque_splitter) = wheel_control();

    // Upstream's three `runSim` stop times.
    let phase_ends_nanos = {
        let first = sec2nano(orbit_period / 8.0);
        let second = first + sec2nano(orbit_period * 2.0);
        [first, second, second + sec2nano(orbit_period / 2.0)]
    };
    // `fswHillPointing()` before the first run, `fswTargetPointing()` and
    // `fswSunPointing()` at the first two stops.
    let debris_reference = debris_pointing.att_ref_out_msg.clone();
    let sun_reference = sun_pointing.att_ref_out_msg.clone();
    let mut tracking_error = Staged::new(att_tracking_error(Vector3::new(0.0, 0.0, -0.414214)))
        .after(phase_ends_nanos[0], move |module, _| {
            debris_reference.connect_to(&mut module.att_ref_in_msg);
            module.config.sigma_r0r = Vector3::zeros();
        })
        .after(phase_ends_nanos[1], move |module, _| {
            sun_reference.connect_to(&mut module.att_ref_in_msg);
            module.config.sigma_r0r = Vector3::zeros();
        });

    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut wheel_logs: Vec<MessageLog<ReactionWheelStateMsg>> = (0..SERVICER_WHEELS)
        .map(|_| MessageLog::default())
        .collect();
    let mut servicer_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut debris_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut sun_log = MessageLog::<PlanetStateMsg>::default();
    let (mut csv, csv_path) = scenario_recorder("scenarioRendezVous", write_csv);

    {
        let start = Epoch::from_gregorian_utc(2022, 5, 1, 0, 28, 30, 0);
        let mut simulation = Simulation::new(start, false);
        for (command, wheel) in torque_splitter
            .actuator_out_msgs
            .iter()
            .zip(wheel_effector.wheels_mut())
        {
            simulation.connect(command, &mut wheel.motor_torque_in_msg);
        }
        servicer.add_state_effector(wheel_effector);
        connect!(&simulation,
            &sun_ephemeris.output_planet_msg => servicer.grav_body_input_mut("sun").expect("sun"),
            &sun_ephemeris.output_planet_msg => debris.grav_body_input_mut("sun").expect("sun"),
            &earth_ephemeris.output_planet_msg => servicer.grav_body_input_mut("earth").expect("earth"),
            &earth_ephemeris.output_planet_msg => debris.grav_body_input_mut("earth").expect("earth"),
            &sun_ephemeris.output_planet_msg => &mut ephemeris_converter.planet_in_msgs[0],
            &earth_ephemeris.output_planet_msg => &mut ephemeris_converter.planet_in_msgs[1],
            &servicer.state_out => &mut navigation.spacecraft_state_in_msg,
            &debris.state_out => &mut debris_navigation.spacecraft_state_in_msg,
            &navigation.translation_out_msg => &mut hill_pointing.trans_nav_in_msg,
            &ephemeris_converter.ephem_out_msgs[1] => &mut hill_pointing.cel_body_in_msg,
            &debris_navigation.translation_out_msg => &mut debris_pointing.sc_target_in_msg,
            &navigation.translation_out_msg => &mut debris_pointing.sc_trans_in_msg,
            &navigation.attitude_out_msg => &mut debris_pointing.sc_att_in_msg,
            &ephemeris_converter.ephem_out_msgs[0] => &mut sun_pointing.cel_body_in_msg,
            &navigation.translation_out_msg => &mut sun_pointing.sc_trans_in_msg,
            &navigation.attitude_out_msg => &mut sun_pointing.sc_att_in_msg,
            &hill_pointing.att_ref_out_msg => &mut tracking_error.module.att_ref_in_msg,
            &navigation.attitude_out_msg => &mut tracking_error.module.att_nav_in_msg,
            &tracking_error.module.att_guid_out_msg => &mut controller.guid_in_msg,
            &vehicle_config_msg => &mut controller.veh_config_in_msg,
            &wheels.config_msg => &mut controller.rw_params_in_msg,
            &wheels.speed_msg => &mut controller.rw_speeds_in_msg,
            &controller.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
            &wheels.config_msg => &mut rw_motor_torque.rw_params_in_msg,
            &rw_motor_torque.rw_motor_torque_out_msg => &mut torque_splitter.torque_in_msg,
            &tracking_error.module.att_guid_out_msg => &mut guidance_log.input,
            &servicer.state_out => &mut servicer_log.input,
            &debris.state_out => &mut debris_log.input,
            &sun_ephemeris.output_planet_msg => &mut sun_log.input,
            &rw_motor_torque.rw_motor_torque_out_msg => csv.add_source::<ArrayMotorTorqueMsg>(
                CsvSourceConfig::columns(array_columns(
                    "motor_torque_nm",
                    "motorTorque_Nm",
                    MAX_EFF_COUNT,
                )),
            ),
            &tracking_error.module.att_guid_out_msg => csv.add_source::<AttitudeGuidanceMsg>(
                CsvSourceConfig::columns(
                    vector_columns("sigma_br", "sigma_BR")
                        .into_iter()
                        .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
                ),
            ),
            &wheels.speed_msg => csv.add_source::<RwSpeedMsg>(
                CsvSourceConfig::columns(array_columns(
                    "wheel_speeds_radps",
                    "wheelSpeeds_rad_per_s",
                    MAX_EFF_COUNT,
                )),
            ),
            &servicer.state_out => csv.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("servicer")),
            &debris.state_out => csv.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("debris")),
        );
        for (index, (state, log)) in wheels.state_msgs.iter().zip(&mut wheel_logs).enumerate() {
            connect!(&simulation,
                state => &mut log.input,
                state => csv.add_source::<ReactionWheelStateMsg>(CsvSourceConfig::columns([(
                    "applied_motor_torque_nm".to_string(),
                    format!("rwTorque_Nm_{index}"),
                )])),
            );
        }
        schedule! { simulation,
            "sunEphemeris" => &mut sun_ephemeris, period=STEP_NANOS, priority=110;
            "earthEphemeris" => &mut earth_ephemeris, period=STEP_NANOS, priority=110;
            "debris" => &mut debris, period=STEP_NANOS, priority=100;
            "servicer" => &mut servicer, period=STEP_NANOS, priority=99;
            "SimpleNavigation" => &mut navigation, period=STEP_NANOS, priority=90;
            "SimpleNavigation2" => &mut debris_navigation, period=STEP_NANOS, priority=90;
            "EphemData" => &mut ephemeris_converter, period=STEP_NANOS, priority=85;
            "hillPoint" => &mut hill_pointing, period=STEP_NANOS, priority=80;
            "scPointing" => &mut debris_pointing, period=STEP_NANOS, priority=80;
            "sunPointing" => &mut sun_pointing, period=STEP_NANOS, priority=80;
            "attErrorInertial3D" => &mut tracking_error, period=STEP_NANOS, priority=70;
            "mrpFeedback" => &mut controller, period=STEP_NANOS, priority=60;
            "rwMotorTorque" => &mut rw_motor_torque, period=STEP_NANOS, priority=50;
            "rwMotorTorqueSplitter" => &mut torque_splitter, period=STEP_NANOS, priority=45;
            "attErrLog" => &mut guidance_log, period=STEP_NANOS, priority=0;
            "servicerLog" => &mut servicer_log, period=STEP_NANOS, priority=0;
            "debrisLog" => &mut debris_log, period=STEP_NANOS, priority=0;
            "sunLog" => &mut sun_log, period=STEP_NANOS, priority=0;
        }
        for log in &mut wheel_logs {
            simulation.add_module("rwLog", log, STEP_NANOS, 0);
        }
        if write_csv {
            simulation.add_module("csv", &mut csv, STEP_NANOS, 0);
        }
        // Upstream reads the hub states after `InitializeSimulation`.
        simulation.initialize();
        let states = || {
            (
                (debris_hub.position_m(), debris_hub.velocity_mps()),
                (servicer_hub.position_m(), servicer_hub.velocity_mps()),
            )
        };

        let (chief, deputy) = states();
        servicer_hub.set_velocity_mps(rel_orb_drift(n, chief, deputy));
        run_until(&mut simulation, phase_ends_nanos[0]);

        let (chief, deputy) = states();
        servicer_hub.set_velocity_mps(relative_ellipse(n, chief, deputy, 10.0, -1.0, 10.0));
        run_until(&mut simulation, phase_ends_nanos[1]);

        let (chief, deputy) = states();
        servicer_hub.set_velocity_mps(relative_ellipse(n, chief, deputy, 10.0, 0.0, 0.0));
        run_until(&mut simulation, phase_ends_nanos[2]);
    }
    if write_csv {
        csv.flush().expect("flush scenario CSV");
    }

    Results {
        guidance: guidance_log,
        wheels: wheel_logs,
        servicer: servicer_log,
        debris: debris_log,
        sun: sun_log,
        phase_ends_nanos,
        mean_motion: n,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env(USAGE);
    cli.finish();
    let results = run(true);
    let (chief, deputy) = (results.debris.last(), results.servicer.last());
    let (rho_h, _) = state_to_hill(
        chief.position_m,
        chief.velocity_mps,
        deputy.position_m,
        deputy.velocity_mps,
    );
    println!(
        "wrote {} (final |sigma_BR| = {:.3e}, servicer at {:.2?} m in the debris's Hill frame)",
        results.csv_path.as_ref().expect("CSV written").display(),
        results.guidance.last().sigma_br.norm(),
        rho_h.as_slice(),
    );
}

#[cfg(test)]
mod tests {
    //! Upstream runs this scenario in `test_scenarioTest.py` without checks.
    //! These tests check what it demonstrates: the relative orbits each
    //! maneuver sets up and the three pointing modes.

    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;

    use super::*;

    /// The Clohessy-Wiltshire solution `t` after the Hill-frame state
    /// `(rho, rho')` about a circular chief of mean motion `n`.
    fn clohessy_wiltshire(
        n: f64,
        rho: Vector3<f64>,
        rho_prime: Vector3<f64>,
        t: f64,
    ) -> Vector3<f64> {
        let (s, c) = (n * t).sin_cos();
        Vector3::new(
            (4.0 - 3.0 * c) * rho.x + s / n * rho_prime.x + 2.0 / n * (1.0 - c) * rho_prime.y,
            6.0 * (s - n * t) * rho.x + rho.y - 2.0 / n * (1.0 - c) * rho_prime.x
                + (4.0 * s - 3.0 * n * t) / n * rho_prime.y,
            c * rho.z + s / n * rho_prime.z,
        )
    }

    fn hill_state(results: &Results, index: usize) -> (Vector3<f64>, Vector3<f64>) {
        let (chief, deputy) = (
            &results.debris.values[index],
            &results.servicer.values[index],
        );
        state_to_hill(
            chief.position_m,
            chief.velocity_mps,
            deputy.position_m,
            deputy.velocity_mps,
        )
    }

    #[test]
    fn scenario_behaves_as_upstream_describes() {
        let results = run(false);
        let n = results.mean_motion;
        let ends: Vec<usize> = results
            .phase_ends_nanos
            .iter()
            .map(|end| (*end / STEP_NANOS) as usize)
            .collect();
        assert_eq!(results.servicer.values.len(), ends[2] + 1);

        // After each maneuver the servicer follows the linear relative orbit
        // the maneuver aims for, to the Sun's perturbation and the model's
        // nonlinearity (millimetres at 10 m from the debris).
        // (A maneuver at a stop time takes effect with the next step, so the
        // states logged from then on are on the new relative orbit.)
        let starts = [0, ends[0] + 1, ends[1] + 1];
        for (phase, (&start, &end)) in starts.iter().zip(&ends).enumerate() {
            let (rho0, rho_prime0) = hill_state(&results, start);
            for index in (start..=end).step_by(60) {
                let t = (index - start) as f64;
                let expected = clohessy_wiltshire(n, rho0, rho_prime0, t);
                let (rho, _) = hill_state(&results, index);
                assert!(
                    (rho - expected).norm() < 0.02,
                    "phase {} at {t} s: rho = {rho}, CW {expected}",
                    phase + 1
                );
            }
        }
        // 1: an along-track drift at -10 m radially; 3: the bounded 2:1
        // ellipse, 10 m radially, 20 m along-track, about the debris's track.
        let (rho, rho_prime) = hill_state(&results, ends[0]);
        assert!(
            (rho.x + 10.0).abs() < 1e-2 && rho_prime.x.abs() < 1e-4,
            "{rho}"
        );
        let (rho, rho_prime) = hill_state(&results, ends[1] + 1);
        let radial_amplitude = (rho.x.powi(2) + (rho_prime.x / n).powi(2)).sqrt();
        assert!((radial_amplitude - 10.0).abs() < 1e-2, "{radial_amplitude}");
        assert!(
            (rho_prime.y + 2.0 * n * rho.x).abs() < 1e-6,
            "drift {rho_prime}"
        );

        // The wheels saturate during the first slews.
        let saturated = results.wheels.iter().any(|wheel| {
            wheel
                .values
                .iter()
                .any(|state| (state.applied_motor_torque_nm.abs() - 0.2).abs() < 1e-12)
        });
        assert!(saturated, "no wheel reached its 0.2 N m limit");

        // The end of phase 2: `b1` aims at the debris. The end of phase 3:
        // `b3` aims at the Sun.
        let pointing_error = |index: usize, axis: Vector3<f64>, target: Vector3<f64>| {
            let servicer = &results.servicer.values[index];
            let axis_n = mrp_to_dcm(servicer.sigma_bn).transpose() * axis;
            axis_n.angle(&(target - servicer.position_m)).to_degrees()
        };
        let debris_error = pointing_error(
            ends[1],
            Vector3::x(),
            results.debris.values[ends[1]].position_m,
        );
        assert!(
            debris_error < 0.1,
            "b1 is {debris_error} deg off the debris"
        );
        let sun_error = pointing_error(
            ends[2],
            Vector3::z(),
            results.sun.values[ends[2]].position_inertial_m,
        );
        assert!(sun_error < 1e-3, "b3 is {sun_error} deg off the Sun");
    }
}
