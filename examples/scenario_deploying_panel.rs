//! Two hinged solar panels deployed by motors along linear profiles: port of
//! upstream `examples/scenarioDeployingPanel.py`.
//!
//! A 750 kg hub on a 7000 × 42000 km Earth transfer orbit carries two
//! stowed 100 kg panels, [`HingedRigidBodyStateEffector`]s at `θ = -π`
//! (`d = 1.5 m`, `k = 200 N m/rad`, `c = 20 N m s/rad`). Each panel's
//! [`HingedBodyLinearProfiler`] ramps its reference angle (panel 1: from
//! `-π` to `-π/2` between 5 s and 85 s; panel 2: to `-π/1.75`, not fully
//! open, between 100 s and 164 s), and a [`HingedRigidBodyMotor`]
//! (`K = 20`, `P = 10`) drives the panel along it; the hinge spring also
//! acts about the reference. A [`SimpleSolarPanel`] on each (2 m^2, 90 %,
//! normal `ŝ3` of the panel frame) reads the panel's inertial state. The
//! Earth and the Sun attract the spacecraft; RK4, 0.1 s steps, 240 s.
//!
//! Upstream's `createSpiceInterface` (zero base Earth, epoch
//! 2012 MAY 1 00:28:30) is [`AniseSunEphemeris`], the Sun relative to Earth
//! from DE440s (`just fetch-anise-assets`), scheduled before the spacecraft;
//! the Earth stays at the origin, as the zero base.
//!
//! Upstream's panels read their motor torque and reference angle in their
//! own `UpdateState`, after the spacecraft has integrated and before the
//! motors and profilers of the same task run, and use them in the next
//! integration step. basilisk-rs's effectors read their inputs as the step
//! starts; a [`Latch`] per input, scheduled where upstream's panel runs,
//! restores upstream's timing (the first step sees the unwritten zero
//! messages, as upstream's).
//!
//! The hub attitude and rate, both panel angles and rates, and both panels'
//! power are recorded every `samplingTime(240 s, 0.1 s, 1000)` = 0.2 s to
//! `examples/output/scenarios/scenarioDeployingPanel.csv`.
//!
//! Upstream only runs this scenario (`test_scenarioTest.py`); `tests`
//! checks what the figures show: each panel follows its profile to its
//! final angle, the second only after the first, and the deployment, a
//! motion internal to the spacecraft, conserves its angular momentum.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/latch.rs"]
mod latch;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/multibody.rs"]
mod multibody;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;

use std::f64::consts::PI;
use std::path::PathBuf;

use attitude::{sampling_period_nanos, scenario_recorder};
use basilisk_rs::device_interface::hinged_body_linear_profiler::HingedBodyLinearProfiler;
use basilisk_rs::dynamics::hinged_rigid_body_motor::{
    HingedRigidBodyMotor, HingedRigidBodyMotorConfig,
};
use basilisk_rs::dynamics::hinged_rigid_body_state_effector::{
    HingedRigidBodyConfig, HingedRigidBodyStateEffector,
};
use basilisk_rs::environment::anise_sun_ephemeris::{AniseSunEphemeris, AniseSunEphemerisConfig};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, HingedRigidBodyMsg, PowerNodeUsageMsg, SpacecraftDiagnosticsMsg,
    SpacecraftStateMsg,
};
use basilisk_rs::power::simple_solar_panel::{SimpleSolarPanel, SimpleSolarPanelModel};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::{Body, PLANETARY_SPK};
use hifitime::Epoch;
use latch::Latch;
use message_log::MessageLog;
use multibody::set_center_of_mass_initial_state;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{run_until, sec2nano};

/// Upstream `simulationTimeStep = macros.sec2nano(.1)`.
const STEP_NANOS: u64 = 100_000_000;
/// Upstream `simulationTime = macros.sec2nano(240)`.
const DURATION_NANOS: u64 = 240_000_000_000;
/// The profiles: start and end times \[s\] and the final angles \[rad\].
const PANEL1_PROFILE: (f64, f64, f64) = (5.0, 85.0, -PI / 2.0);
const PANEL2_PROFILE: (f64, f64, f64) = (100.0, 164.0, -PI / 1.75);

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The recorded histories, every 0.2 s.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    states: MessageLog<SpacecraftStateMsg>,
    panels: [MessageLog<HingedRigidBodyMsg>; 2],
    references: [MessageLog<HingedRigidBodyMsg>; 2],
    power: [MessageLog<PowerNodeUsageMsg>; 2],
    diagnostics: MessageLog<SpacecraftDiagnosticsMsg>,
    csv_path: Option<PathBuf>,
}

/// One deployable panel with its profiler, motor and solar panel, and the
/// latches standing in for its own input reads.
struct Deployable {
    panel: HingedRigidBodyStateEffector,
    profiler: HingedBodyLinearProfiler,
    motor: HingedRigidBodyMotor,
    solar_panel: SimpleSolarPanel,
    torque_latch: Latch<ArrayMotorTorqueMsg>,
    reference_latch: Latch<HingedRigidBodyMsg>,
}

fn deployable(
    index: usize,
    hinge_position_body_m: Vector3<f64>,
    dcm_hb: Matrix3<f64>,
    profile: (f64, f64, f64),
) -> Deployable {
    let (start_s, end_s, end_theta_rad) = profile;
    let panel = HingedRigidBodyStateEffector::new(HingedRigidBodyConfig {
        mass_kg: 100.0,
        inertia_about_panel_com_panel_kg_m2: Matrix3::from_diagonal(&Vector3::new(
            100.0, 50.0, 50.0,
        )),
        center_of_mass_offset_m: 1.5,
        spring_constant_nm_per_rad: 200.0,
        damping_nm_s_per_rad: 20.0,
        hinge_position_body_m,
        body_to_hinge_dcm: dcm_hb,
        theta_init_rad: -PI,
        theta_dot_init_radps: 0.0,
        ..HingedRigidBodyConfig::new(format!("panel{index}"))
    });
    let profiler = HingedBodyLinearProfiler::with_profile(
        if index == 1 {
            "deploymentProfiler"
        } else {
            "deploymentProfiler2"
        },
        sec2nano(start_s),
        sec2nano(end_s),
        -PI,
        end_theta_rad,
    );
    let motor = HingedRigidBodyMotor::new(HingedRigidBodyMotorConfig {
        k_nm_per_rad: 20.0,
        p_nm_s_per_rad: 10.0,
        ..HingedRigidBodyMotorConfig::new(if index == 1 {
            "hingedRigidBodyMotor".to_string()
        } else {
            format!("hingedRigidBodyMotor{index}")
        })
    });
    let solar_panel = SimpleSolarPanel::new(
        format!("pwr{index}"),
        SimpleSolarPanelModel::new(Vector3::z(), 2.0, 0.9),
    );
    Deployable {
        panel,
        profiler,
        motor,
        solar_panel,
        torque_latch: Latch::default(),
        reference_latch: Latch::default(),
    }
}

fn run(config: Config) -> Results {
    let earth = Body::Earth;
    let r_leo = 7000.0 * 1000.0;
    let r_geo = 42000.0 * 1000.0;
    let semi_major_axis = (r_leo + r_geo) / 2.0;
    let (r_cn_n, v_cn_n) = elements_to_state(
        earth.mu_m3ps2(),
        &ClassicElements {
            semi_major_axis,
            eccentricity: 1.0 - r_leo / semi_major_axis,
            inclination_rad: 0.0,
            raan_rad: 48.2_f64.to_radians(),
            arg_periapsis_rad: 347.8_f64.to_radians(),
            true_anomaly_rad: 85.3_f64.to_radians(),
            ..Default::default()
        },
    );

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 750.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: r_cn_n,
        initial_velocity_mps: v_cn_n,
        initial_sigma_bn: Vector3::new((-90.0_f64 / 4.0).to_radians().tan(), 0.0, 0.0),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(earth.point_mass(true))
        .expect("unique central gravity body");
    spacecraft
        .add_grav_body(Body::Sun.point_mass(false))
        .expect("unique gravity body");
    let mut sun_ephemeris = AniseSunEphemeris::new(AniseSunEphemerisConfig {
        name: "spiceObject".to_string(),
        spk_path: PLANETARY_SPK.into(),
    });

    let mut one = deployable(
        1,
        Vector3::new(-0.5, 0.0, -1.0),
        Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0)),
        PANEL1_PROFILE,
    );
    let mut two = deployable(
        2,
        Vector3::new(0.5, 0.0, -1.0),
        Matrix3::identity(),
        PANEL2_PROFILE,
    );

    let sample_period = sampling_period_nanos(DURATION_NANOS, STEP_NANOS, 1000);
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut diagnostics_log = MessageLog::<SpacecraftDiagnosticsMsg>::default();
    let mut panel_logs: [MessageLog<HingedRigidBodyMsg>; 2] = Default::default();
    let mut reference_logs: [MessageLog<HingedRigidBodyMsg>; 2] = Default::default();
    let mut power_logs: [MessageLog<PowerNodeUsageMsg>; 2] = Default::default();
    let (mut recorder, csv_path) = scenario_recorder("scenarioDeployingPanel", config.write_csv);

    // Upstream `timeInitString = "2012 MAY 1 00:28:30.0"`.
    let epoch = Epoch::from_gregorian_utc(2012, 5, 1, 0, 28, 30, 0);
    let mut simulation = Simulation::new(epoch, false);
    connect!(&simulation,
        &sun_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("sun").expect("sun"),
        &spacecraft.state_out => &mut state_log.input,
        &spacecraft.diagnostics_out => &mut diagnostics_log.input,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
            vector_columns("sigma_bn", "sigma_BN")
                .into_iter()
                .chain(vector_columns("omega_radps", "omega_BN_B_rad_per_s")),
        )),
    );
    for (index, (((deployable, panel_log), reference_log), power_log)) in [&mut one, &mut two]
        .into_iter()
        .zip(panel_logs.iter_mut())
        .zip(reference_logs.iter_mut())
        .zip(power_logs.iter_mut())
        .enumerate()
    {
        let prefix = format!("panel{}", index + 1);
        let Deployable {
            panel,
            profiler,
            motor,
            solar_panel,
            torque_latch,
            reference_latch,
        } = deployable;
        connect!(&simulation,
            &profiler.hinged_rigid_body_reference_out => &mut reference_latch.input,
            &reference_latch.output => &mut panel.hinged_rigid_body_ref_in,
            &panel.hinged_rigid_body_out => &mut motor.hinged_body_state_sensed_in_msg,
            &profiler.hinged_rigid_body_reference_out => &mut motor.hinged_body_state_reference_in_msg,
            &motor.motor_torque_out_msg => &mut torque_latch.input,
            &torque_latch.output => &mut panel.motor_torque_in,
            &panel.panel_state_out => &mut solar_panel.model.state_in_msg,
            &sun_ephemeris.output_sun_msg => &mut solar_panel.model.sun_in_msg,
            &panel.hinged_rigid_body_out => &mut panel_log.input,
            &profiler.hinged_rigid_body_reference_out => &mut reference_log.input,
            &solar_panel.node_power_out_msg => &mut power_log.input,
            &panel.hinged_rigid_body_out => recorder.add_source::<HingedRigidBodyMsg>(
                CsvSourceConfig::columns([
                    ("theta_rad".to_string(), format!("{prefix}_theta_rad")),
                    ("theta_dot_radps".to_string(), format!("{prefix}_thetaDot_rad_per_s")),
                ]),
            ),
            &solar_panel.node_power_out_msg => recorder.add_source::<PowerNodeUsageMsg>(
                CsvSourceConfig::columns([(
                    "net_power_w".to_string(),
                    format!("{prefix}_netPower_W"),
                )]),
            ),
        );
    }
    let Deployable {
        panel: panel1,
        profiler: mut profiler1,
        motor: mut motor1,
        solar_panel: mut solar_panel1,
        torque_latch: mut torque_latch1,
        reference_latch: mut reference_latch1,
    } = one;
    let Deployable {
        panel: panel2,
        profiler: mut profiler2,
        motor: mut motor2,
        solar_panel: mut solar_panel2,
        torque_latch: mut torque_latch2,
        reference_latch: mut reference_latch2,
    } = two;
    spacecraft.add_state_effector(panel1);
    spacecraft.add_state_effector(panel2);
    set_center_of_mass_initial_state(&mut spacecraft, r_cn_n, v_cn_n);

    // Upstream task order: scObject, panel1, profiler1, motor1, panel2,
    // profiler2, motor2, solarPanel1, solarPanel2 (the latches are the
    // panels' own input reads).
    schedule! { simulation,
        "spiceObject" => &mut sun_ephemeris, period=STEP_NANOS, priority=110;
        "bskSat" => &mut spacecraft, period=STEP_NANOS, priority=100;
        "panel1_inputs_torque" => &mut torque_latch1, period=STEP_NANOS, priority=99;
        "panel1_inputs_reference" => &mut reference_latch1, period=STEP_NANOS, priority=99;
        "deploymentProfiler" => &mut profiler1, period=STEP_NANOS, priority=98;
        "hingedRigidBodyMotor" => &mut motor1, period=STEP_NANOS, priority=97;
        "panel2_inputs_torque" => &mut torque_latch2, period=STEP_NANOS, priority=96;
        "panel2_inputs_reference" => &mut reference_latch2, period=STEP_NANOS, priority=96;
        "deploymentProfiler2" => &mut profiler2, period=STEP_NANOS, priority=95;
        "hingedRigidBodyMotor2" => &mut motor2, period=STEP_NANOS, priority=94;
        "pwr1" => &mut solar_panel1, period=STEP_NANOS, priority=93;
        "pwr2" => &mut solar_panel2, period=STEP_NANOS, priority=92;
        "state_log" => &mut state_log, period=sample_period, priority=0;
        "diagnostics_log" => &mut diagnostics_log, period=sample_period, priority=0;
    }
    for log in &mut panel_logs {
        simulation.add_module("panel_log", log, sample_period, 0);
    }
    for log in &mut reference_logs {
        simulation.add_module("reference_log", log, sample_period, 0);
    }
    for log in &mut power_logs {
        simulation.add_module("power_log", log, sample_period, 0);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    run_until(&mut simulation, DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        states: state_log,
        panels: panel_logs,
        references: reference_logs,
        power: power_logs,
        diagnostics: diagnostics_log,
        csv_path,
    }
}

fn main() {
    Cli::from_env("scenario_deploying_panel").finish();
    let results = run(Config { write_csv: true });
    println!(
        "wrote {} (final panel angles {:.2} and {:.2} deg)",
        results.csv_path.expect("CSV written").display(),
        results.panels[0].last().theta_rad.to_degrees(),
        results.panels[1].last().theta_rad.to_degrees(),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(log: &MessageLog<HingedRigidBodyMsg>, seconds: f64) -> &HingedRigidBodyMsg {
        log.at(sec2nano(seconds))
    }

    /// The profilers hold `-π` before their start, ramp linearly and hold
    /// the final angle after their end.
    #[test]
    fn the_references_follow_the_linear_profiles() {
        let results = run(Config { write_csv: false });
        for (log, (start_s, end_s, end_theta)) in results
            .references
            .iter()
            .zip([PANEL1_PROFILE, PANEL2_PROFILE])
        {
            assert!((at(log, start_s).theta_rad + PI).abs() < 1.0e-12);
            let middle = at(log, 0.5 * (start_s + end_s));
            assert!((middle.theta_rad - 0.5 * (end_theta - PI)).abs() < 1.0e-9);
            assert!((middle.theta_dot_radps - (end_theta + PI) / (end_s - start_s)).abs() < 1.0e-9);
            assert!((at(log, 240.0).theta_rad - end_theta).abs() < 1.0e-12);
        }
    }

    /// Each panel deploys along its profile: panel 2 stays stowed while
    /// panel 1 opens, and both settle on their final angles by the end.
    #[test]
    fn the_panels_deploy_to_their_final_angles() {
        let results = run(Config { write_csv: false });
        let [panel1, panel2] = &results.panels;
        assert!((at(panel1, 0.0).theta_rad + PI).abs() < 1.0e-12);
        assert!((at(panel1, 240.0).theta_rad - PANEL1_PROFILE.2).abs() < 0.5_f64.to_radians());
        assert!(at(panel1, 240.0).theta_dot_radps.abs() < 1.0e-3);
        // Panel 1's deployment only rocks panel 2 by a fraction of a degree.
        for seconds in [5.0, 50.0, 85.0, 99.8] {
            assert!((at(panel2, seconds).theta_rad + PI).abs() < 1.0_f64.to_radians());
        }
        assert!((at(panel2, 240.0).theta_rad - PANEL2_PROFILE.2).abs() < 0.5_f64.to_radians());
        // Halfway through each ramp the panel tracks its moving reference.
        for (panel, reference, (start_s, end_s, _)) in [
            (panel1, &results.references[0], PANEL1_PROFILE),
            (panel2, &results.references[1], PANEL2_PROFILE),
        ] {
            let seconds = 0.5 * (start_s + end_s);
            let lag = (at(panel, seconds).theta_rad - at(reference, seconds).theta_rad).abs();
            assert!(
                lag < 5.0_f64.to_radians(),
                "tracking error {} deg",
                lag.to_degrees()
            );
        }
    }

    /// The hinge motors and springs act inside the spacecraft: the
    /// deployment turns the hub but conserves the total angular momentum
    /// (zero, to the integration error) and leaves the hub at rest once the
    /// panels settle.
    #[test]
    fn the_deployment_conserves_angular_momentum() {
        let results = run(Config { write_csv: false });
        let peak_rate = results
            .states
            .values
            .iter()
            .fold(0.0_f64, |max, state| max.max(state.omega_radps.norm()));
        assert!(peak_rate > 1.0e-3, "peak hub rate {peak_rate}");
        // Against the hub's own peak momentum, about 900 kg m^2 times its rate.
        let scale = 900.0 * peak_rate;
        for diagnostics in &results.diagnostics.values {
            let momentum = diagnostics
                .rotational_angular_momentum_inertial_kg_m2ps
                .norm();
            assert!(
                momentum < 1.0e-6 * scale,
                "angular momentum {momentum}, scale {scale}"
            );
        }
        assert!(results.states.last().omega_radps.norm() < 1.0e-2 * peak_rate);
    }

    /// Each panel's power is its projected area times the solar flux and
    /// efficiency: never negative or above the full 2 m^2 output. Stowed,
    /// both panels face away from the Sun; opening turns panel 1's cells
    /// to it, while panel 2, hinged the other way, stays dark.
    #[test]
    fn the_panel_power_follows_the_panel_attitude() {
        let results = run(Config { write_csv: false });
        let full = basilisk_rs::utilities::astro_constants::SOLAR_FLUX_EARTH * 2.0 * 0.9 * 1.05;
        for log in &results.power {
            assert!(
                log.values
                    .iter()
                    .all(|power| (0.0..full).contains(&power.net_power_w))
            );
            assert_eq!(log.values[0].net_power_w, 0.0);
        }
        assert!(results.power[0].last().net_power_w > 0.5 * full / 1.05);
        assert!(
            results.power[1]
                .values
                .iter()
                .all(|power| power.net_power_w == 0.0)
        );
    }
}
