//! Coarse Sun sensor configuration `0010` from `scenarioCSS.py`.
//!
//! Three independent coarse Sun sensors, no platform/constellation, a fixed
//! half-eclipse, and no Kelly distortion. Writes the sensor signals to CSV.

#[path = "support/common.rs"]
mod common;

use basilisk_rs::messages::{EclipseMsg, Output, SunEphemerisMsg, SunSensorMsg};
use basilisk_rs::sensors::coarse_sun_sensor::{CoarseSunSensor, CoarseSunSensorConfig};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::{CsvFormat, CsvRecorder, CsvRecorderConfig, CsvSourceConfig};
use basilisk_rs::{connect, schedule};
use common::{scenario_output_path, seconds};
use hifitime::Epoch;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_700.0;
const TASK_PERIOD_NANOS: u64 = seconds(1);
const DURATION_NANOS: u64 = seconds(300);

fn sensor(
    name: &str,
    position_m: Vector3<f64>,
    normal_body: Vector3<f64>,
    fov_deg: f64,
) -> CoarseSunSensor {
    let body_to_sensor_quaternion =
        UnitQuaternion::rotation_between(&normal_body.normalize(), &Vector3::z())
            .expect("non-opposite CSS mounting vectors");
    CoarseSunSensor::new(CoarseSunSensorConfig {
        name: name.to_string(),
        position_m,
        body_to_sensor_quaternion,
        fov_half_angle_rad: fov_deg.to_radians(),
        scale_factor: 2.0,
        kelly_factor: 0.0,
        k_power: 1.0,
        bias: 0.0,
        noise_std: 0.0,
        noise_prop: 0.0,
        walk_bounds: 0.0,
        min_output: 0.5,
        max_output: 2.0,
    })
}

fn main() {
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 750.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
        integration_step_nanos: TASK_PERIOD_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.0, 0.0, 1.0_f64.to_radians()),
        integrator: None,
    });
    let mut css1 = sensor(
        "CSS1_sensor",
        Vector3::new(2.00131, 2.36638, 1.0),
        Vector3::x(),
        80.0,
    );
    let mut css2 = sensor(
        "CSS2_sensor",
        Vector3::new(-3.05, 0.55, 1.0),
        Vector3::y(),
        80.0,
    );
    let mut css3 = sensor(
        "CSS3_sensor",
        Vector3::new(-3.05, 0.55, 1.0),
        -Vector3::x(),
        45.0,
    );

    let sun = Output::new(SunEphemerisMsg {
        sun_position_inertial_m: Vector3::new(0.0, ASTRONOMICAL_UNIT_M, 0.0),
        sun_velocity_inertial_mps: Vector3::zeros(),
    });
    let eclipse = Output::new(EclipseMsg {
        illumination_factor: 0.5,
    });
    let output_path = scenario_output_path("scenarioCSS0010.csv");
    let mut recorder = CsvRecorder::new(CsvRecorderConfig {
        topic: "scenarioCSS0010".to_string(),
        output_path: output_path.clone(),
    })
    .with_format(CsvFormat::Scenario);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    for sensor in [&mut css1, &mut css2, &mut css3] {
        connect!(&simulation,
            &spacecraft.state_out => &mut sensor.input_state_msg,
            &sun => &mut sensor.input_sun_msg,
            &eclipse => &mut sensor.input_eclipse_msg,
        );
    }
    connect!(&simulation,
        &css1.output_sun_sensor_msg => recorder.add_source::<SunSensorMsg>(
            CsvSourceConfig::columns([("sensed_value", "css1Signal")]),
        ),
        &css2.output_sun_sensor_msg => recorder.add_source::<SunSensorMsg>(
            CsvSourceConfig::columns([("sensed_value", "css2Signal")]),
        ),
        &css3.output_sun_sensor_msg => recorder.add_source::<SunSensorMsg>(
            CsvSourceConfig::columns([("sensed_value", "css3Signal")]),
        ),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, TASK_PERIOD_NANOS, 30;
        "css1" => &mut css1, TASK_PERIOD_NANOS, 20;
        "css2" => &mut css2, TASK_PERIOD_NANOS, 20;
        "css3" => &mut css3, TASK_PERIOD_NANOS, 20;
        "recorder" => &mut recorder, TASK_PERIOD_NANOS, 0;
    }
    simulation.run_for(DURATION_NANOS);

    println!("wrote {}", output_path.display());
}
