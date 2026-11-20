//! A solar panel and two coarse Sun sensors on a rotating hinged panel: port
//! of upstream `examples/scenarioRotatingPanel.py`.
//!
//! A 750 kg hub on a 7000 × 42000 km Earth transfer orbit carries a 100 kg
//! panel, a [`HingedRigidBodyStateEffector`] hinged at `[-2, 0, 1] m` with
//! its center of mass on the hinge (`d = 0`), no spring and no damper,
//! spinning at 1°/s. A [`SimpleSolarPanel`] (2 m^2, 90 % efficient) and two
//! [`CoarseSunSensor`]s (45° field of view, normals `[1, 0, 0]` and
//! `[0, 0, 1]`) read the panel's inertial state (upstream
//! `hingedRigidBodyConfigLogOutMsg`, here
//! [`HingedRigidBodyStateEffector::panel_state_out`]) instead of the hub's,
//! so their normals are in the rotating panel frame S. The Sun is a fixed
//! stand-alone message at `[0, 1 AU, 0]`. RK4, 1 s steps, for 5 % of the
//! orbit period.
//!
//! The hub attitude, the panel angle, the panel power and both sensor
//! signals are recorded every `samplingTime(0.05 P, 1 s, 200)` = 9 s to
//! `examples/output/scenarios/scenarioRotatingPanel.csv`.
//!
//! Upstream `test_scenarioRotatingPanel.py` only checks that the scenario
//! runs. `tests` checks what the figures show: the panel only makes power
//! while it faces the Sun, its peak is the panel's full output, the sensor
//! on the panel normal reads the same projection, and the hub turns back
//! against the panel, conserving angular momentum.

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
#[path = "support/multibody.rs"]
mod multibody;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;

use std::path::PathBuf;

use attitude::{sampling_period_nanos, scenario_recorder};
use basilisk_rs::dynamics::hinged_rigid_body_state_effector::{
    HingedRigidBodyConfig, HingedRigidBodyStateEffector,
};
use basilisk_rs::messages::{
    HingedRigidBodyMsg, Output, PowerNodeUsageMsg, SpacecraftDiagnosticsMsg, SpacecraftStateMsg,
    SunEphemerisMsg, SunSensorMsg,
};
use basilisk_rs::power::simple_solar_panel::{SimpleSolarPanel, SimpleSolarPanelModel};
use basilisk_rs::sensors::coarse_sun_sensor::{CoarseSunSensor, CoarseSunSensorConfig};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::AU2M;
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use basilisk_rs::utilities::rigid_body_kinematics::euler3;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use multibody::set_center_of_mass_initial_state;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use orbit_tools::{orbit_period_s, run_until, sec2nano};

/// Upstream `simulationTimeStep = macros.sec2nano(1.)`.
const STEP_NANOS: u64 = 1_000_000_000;
/// Upstream `solarPanel.panelArea` \[m^2\] and `panelEfficiency`.
const PANEL_AREA_M2: f64 = 2.0;
const PANEL_EFFICIENCY: f64 = 0.9;

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The recorded histories, every 9 s.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    states: MessageLog<SpacecraftStateMsg>,
    panel: MessageLog<HingedRigidBodyMsg>,
    panel_states: MessageLog<SpacecraftStateMsg>,
    power: MessageLog<PowerNodeUsageMsg>,
    css1: MessageLog<SunSensorMsg>,
    css2: MessageLog<SunSensorMsg>,
    diagnostics: MessageLog<SpacecraftDiagnosticsMsg>,
    sun_position_m: Vector3<f64>,
    csv_path: Option<PathBuf>,
}

/// Upstream `coarseSunSensor.CoarseSunSensor()` with `fov = 45°`,
/// `scaleFactor = 1` and the given normal, other parameters at upstream's
/// defaults (no noise, bias or Kelly fit, `kPower = 2`, outputs within
/// `[0, 1e6]`).
fn coarse_sun_sensor(name: &str, normal: Vector3<f64>) -> CoarseSunSensor {
    CoarseSunSensor::new(CoarseSunSensorConfig {
        name: name.to_string(),
        position_m: Vector3::zeros(),
        body_to_sensor_quaternion: UnitQuaternion::rotation_between(&normal, &Vector3::z())
            .unwrap_or_else(UnitQuaternion::identity),
        fov_half_angle_rad: 45.0_f64.to_radians(),
        scale_factor: 1.0,
        kelly_factor: 0.0,
        k_power: 2.0,
        bias: 0.0,
        noise_std: 0.0,
        noise_prop: 0.0,
        walk_bounds: 0.0,
        min_output: 0.0,
        max_output: 1.0e6,
    })
}

fn run(config: Config) -> Results {
    let earth = Body::Earth;
    let mu = earth.mu_m3ps2();
    let r_leo = 7000.0 * 1000.0;
    let r_geo = 42000.0 * 1000.0;
    let semi_major_axis = (r_leo + r_geo) / 2.0;
    let (r_cn_n, v_cn_n) = elements_to_state(
        mu,
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
    // Upstream `oe = rv2elem(mu, rN, vN)`, whose period sets the run time.
    let elements = state_to_elements(mu, r_cn_n, v_cn_n);
    let duration_nanos = sec2nano(0.05 * orbit_period_s(mu, elements.semi_major_axis));

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

    let panel = HingedRigidBodyStateEffector::new(HingedRigidBodyConfig {
        mass_kg: 100.0,
        inertia_about_panel_com_panel_kg_m2: Matrix3::from_diagonal(&Vector3::new(
            100.0, 50.0, 50.0,
        )),
        // The S and H points coincide.
        center_of_mass_offset_m: 0.0,
        spring_constant_nm_per_rad: 0.0,
        damping_nm_s_per_rad: 0.0,
        hinge_position_body_m: Vector3::new(-2.0, 0.0, 1.0),
        body_to_hinge_dcm: euler3(90.0_f64.to_radians()),
        theta_init_rad: 0.0,
        theta_dot_init_radps: 1.0_f64.to_radians(),
        ..HingedRigidBodyConfig::new("panel1")
    });

    // Upstream `SpicePlanetStateMsgPayload(PositionVector=[0, AU*1000, 0])`.
    let sun_position_m = Vector3::new(0.0, AU2M, 0.0);
    let sun = Output::new(SunEphemerisMsg {
        sun_position_inertial_m: sun_position_m,
        sun_velocity_inertial_mps: Vector3::zeros(),
    });
    let mut solar_panel = SimpleSolarPanel::new(
        "solarPanel",
        // The normal is in the rotating panel frame S.
        SimpleSolarPanelModel::new(Vector3::z(), PANEL_AREA_M2, PANEL_EFFICIENCY),
    );
    let mut css1 = coarse_sun_sensor("CSS1_sensor", Vector3::x());
    let mut css2 = coarse_sun_sensor("CSS2_sensor", Vector3::z());

    let sample_period = sampling_period_nanos(duration_nanos, STEP_NANOS, 200);
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut panel_log = MessageLog::<HingedRigidBodyMsg>::default();
    let mut panel_state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut power_log = MessageLog::<PowerNodeUsageMsg>::default();
    let mut css1_log = MessageLog::<SunSensorMsg>::default();
    let mut css2_log = MessageLog::<SunSensorMsg>::default();
    let mut diagnostics_log = MessageLog::<SpacecraftDiagnosticsMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder("scenarioRotatingPanel", config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &panel.panel_state_out => &mut solar_panel.model.state_in_msg,
        &sun => &mut solar_panel.model.sun_in_msg,
        &panel.panel_state_out => &mut css1.input_state_msg,
        &sun => &mut css1.input_sun_msg,
        &panel.panel_state_out => &mut css2.input_state_msg,
        &sun => &mut css2.input_sun_msg,
        &spacecraft.state_out => &mut state_log.input,
        &panel.hinged_rigid_body_out => &mut panel_log.input,
        &panel.panel_state_out => &mut panel_state_log.input,
        &solar_panel.node_power_out_msg => &mut power_log.input,
        &css1.output_sun_sensor_msg => &mut css1_log.input,
        &css2.output_sun_sensor_msg => &mut css2_log.input,
        &spacecraft.diagnostics_out => &mut diagnostics_log.input,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
            vector_columns("sigma_bn", "sigma_BN"),
        )),
        &panel.hinged_rigid_body_out => recorder.add_source::<HingedRigidBodyMsg>(
            CsvSourceConfig::columns([
                ("theta_rad", "panel1_theta_rad"),
                ("theta_dot_radps", "panel1_thetaDot_rad_per_s"),
            ]),
        ),
        &solar_panel.node_power_out_msg => recorder.add_source::<PowerNodeUsageMsg>(
            CsvSourceConfig::columns([("net_power_w", "netPower_W")]),
        ),
        &css1.output_sun_sensor_msg => recorder.add_source::<SunSensorMsg>(
            CsvSourceConfig::columns([("sensed_value", "CSS1_OutputData")]),
        ),
        &css2.output_sun_sensor_msg => recorder.add_source::<SunSensorMsg>(
            CsvSourceConfig::columns([("sensed_value", "CSS2_OutputData")]),
        ),
    );
    spacecraft.add_state_effector(panel);
    set_center_of_mass_initial_state(&mut spacecraft, r_cn_n, v_cn_n);

    schedule! { simulation,
        "bskSat" => &mut spacecraft, period=STEP_NANOS, priority=100;
        "solarPanel" => &mut solar_panel, period=STEP_NANOS, priority=90;
        "CSS1_sensor" => &mut css1, period=STEP_NANOS, priority=80;
        "CSS2_sensor" => &mut css2, period=STEP_NANOS, priority=70;
        "state_log" => &mut state_log, period=sample_period, priority=0;
        "panel_log" => &mut panel_log, period=sample_period, priority=0;
        "panel_state_log" => &mut panel_state_log, period=sample_period, priority=0;
        "power_log" => &mut power_log, period=sample_period, priority=0;
        "css1_log" => &mut css1_log, period=sample_period, priority=0;
        "css2_log" => &mut css2_log, period=sample_period, priority=0;
        "diagnostics_log" => &mut diagnostics_log, period=sample_period, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    run_until(&mut simulation, duration_nanos);
    drop(simulation);
    drop(recorder);

    Results {
        states: state_log,
        panel: panel_log,
        panel_states: panel_state_log,
        power: power_log,
        css1: css1_log,
        css2: css2_log,
        diagnostics: diagnostics_log,
        sun_position_m,
        csv_path,
    }
}

fn main() {
    Cli::from_env("scenario_rotating_panel").finish();
    let results = run(Config { write_csv: true });
    println!(
        "wrote {} (final panel angle {:.1} deg, peak power {:.1} W)",
        results.csv_path.expect("CSV written").display(),
        results.panel.last().theta_rad.to_degrees(),
        results
            .power
            .values
            .iter()
            .fold(0.0_f64, |max, power| max.max(power.net_power_w)),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use basilisk_rs::utilities::astro_constants::SOLAR_FLUX_EARTH;
    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;

    /// `cos` of the angle between the panel normal `ŝ3` and the Sun, and the
    /// inverse-square distance factor, from a recorded panel state.
    fn sun_geometry(results: &Results, panel: &SpacecraftStateMsg) -> (f64, f64) {
        let r_sun = results.sun_position_m - panel.position_m;
        let sun_s = mrp_to_dcm(panel.sigma_bn) * r_sun.normalize();
        (sun_s.z, (AU2M / r_sun.norm()).powi(2))
    }

    /// "the solar panel only generates power while it faces the Sun": the
    /// power is the projected area times flux and efficiency, peaks at the
    /// full 2 m^2 output while the normal sweeps past the Sun, and is zero
    /// facing away.
    #[test]
    fn the_panel_makes_power_only_facing_the_sun() {
        let results = run(Config { write_csv: false });
        let mut peak_ratio = 0.0_f64;
        let mut dark = 0;
        for (state, power) in results
            .panel_states
            .values
            .iter()
            .zip(&results.power.values)
        {
            let (cos_sun, distance_factor) = sun_geometry(&results, state);
            let full = SOLAR_FLUX_EARTH * distance_factor * PANEL_AREA_M2 * PANEL_EFFICIENCY;
            let expected = full * cos_sun.max(0.0);
            assert!(
                (power.net_power_w - expected).abs() < 1.0e-9 * full,
                "{power:?}"
            );
            peak_ratio = peak_ratio.max(power.net_power_w / full);
            dark += usize::from(power.net_power_w == 0.0);
        }
        // Samples every 9 s at 1 deg/s pass within 4.5 deg of the Sun.
        assert!(peak_ratio > 4.5_f64.to_radians().cos(), "peak {peak_ratio}");
        assert!(dark > results.power.values.len() / 4, "{dark} dark samples");
    }

    /// The panel-normal sensor `CSS2` reads `cos` of the Sun angle (scaled
    /// by distance) inside its 45° field of view, and zero outside; the
    /// edge-on `CSS1` sees the Sun in other parts of the turn.
    #[test]
    fn the_sensors_read_the_rotating_panel_frame() {
        let results = run(Config { write_csv: false });
        let mut css1_lit = 0;
        let mut css2_lit = 0;
        for ((state, css1), css2) in results
            .panel_states
            .values
            .iter()
            .zip(&results.css1.values)
            .zip(&results.css2.values)
        {
            let (cos_sun, distance_factor) = sun_geometry(&results, state);
            let expected = if cos_sun >= 45.0_f64.to_radians().cos() {
                cos_sun * distance_factor
            } else {
                0.0
            };
            assert!(
                (css2.sensed_value - expected).abs() < 1.0e-9,
                "{css2:?} vs {expected}"
            );
            css1_lit += usize::from(css1.sensed_value > 0.0);
            css2_lit += usize::from(css2.sensed_value > 0.0);
            // Normals 90 deg apart, each with a 45 deg half-angle: never both.
            assert!(css1.sensed_value == 0.0 || css2.sensed_value == 0.0);
        }
        assert!(
            css1_lit > 0 && css2_lit > 0,
            "lit samples {css1_lit}, {css2_lit}"
        );
    }

    /// With no hinge spring or damper, the panel turning about a principal
    /// axis through its center of mass exerts no torque on the resting hub:
    /// the panel keeps its 1°/s, the hub its attitude, and the system's
    /// angular momentum is conserved.
    #[test]
    fn the_panel_turns_freely_and_momentum_is_conserved() {
        let results = run(Config { write_csv: false });
        let duration_s = *results.panel.times_nanos.last().unwrap() as f64 * 1.0e-9;
        let mean_rate = results.panel.last().theta_rad / duration_s;
        assert!(
            (mean_rate - 1.0_f64.to_radians()).abs() < 1.0e-12,
            "{mean_rate}"
        );
        let sigma_0 = results.states.values[0].sigma_bn;
        assert!((sigma_0.x - (-22.5_f64).to_radians().tan()).abs() < 1.0e-15);
        for state in &results.states.values {
            assert!((state.sigma_bn - sigma_0).norm() < 1.0e-9);
            assert!(state.omega_radps.norm() < 1.0e-12);
        }
        let initial = results.diagnostics.values[0].rotational_angular_momentum_inertial_kg_m2ps;
        for diagnostics in &results.diagnostics.values {
            let drift = diagnostics.rotational_angular_momentum_inertial_kg_m2ps - initial;
            assert!(drift.norm() < 1.0e-6 * initial.norm(), "drift {drift}");
        }
    }
}
