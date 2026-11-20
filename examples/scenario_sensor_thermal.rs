//! Temperature of a body-mounted sensor plate as it is pointed at and away
//! from the Sun: port of upstream `examples/scenarioSensorThermal.py`.
//!
//! A 750 kg spacecraft on a 600 km, 63.3 deg Earth orbit (`e = 0.01`) from
//! `2021 MAY 04 07:47:48.965 (UTC)`, with the Sun and Earth as gravity
//! bodies. `locationPointing` turns the body +z axis to the Sun
//! (`mrpFeedback`, `K = 5.5`, `P = 30`, applied by an `extForceTorque`
//! effector) for one orbit, then the -z axis for a second one. A
//! `sensorThermal` plate on +z (1 m^2, absorptivity 0.25, emissivity 0.34,
//! 2 kg, 890 J/(kg K), 30 W of electronics, 0 °C at the start) heats in the
//! sunlight and cools when turned away. One 1 s task, the temperature
//! recorded every step.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted and
//! runs the same single case.
//!
//! Mapping to basilisk-rs:
//!
//! * upstream's `spiceInterface` (Sun and Earth relative to the solar-system
//!   barycenter) is [`AniseSunEphemeris`], the Sun relative to Earth from
//!   DE440s, with the Earth at the origin of the inertial frame. It runs
//!   before the spacecraft, which takes the Sun's gravity from it, and, as
//!   upstream's `Reset` does, also at initialization ([`UpdatedAtInit`]);
//! * upstream's change of `locPoint.pHat_B` between the two
//!   `ExecuteSimulation` calls is a [`Staged`] change after the first orbit
//!   (`int(P)` seconds).
//!
//! # The second orbit diverges
//!
//! The flip asks for a 180 deg slew, where `locationPointing`'s eigen axis
//! is undefined and its finite-difference rate error jumps (from zero to
//! about 2 rad/s in one 1 s step). With the boresight spin the first orbit
//! leaves (about 1 deg/s, which pointing does not constrain),
//! `mrpFeedback`'s gyroscopic feed-forward terms then turn the eigen axis
//! from step to step and the attitude loop spins the spacecraft up: 4 s
//! after the flip its rate exceeds 1 rad/s and, about 10 s after it, the
//! state is no longer finite. The same happens with a `Reset` of
//! `locationPointing` at the flip, a 0.01 rad `smallAngle`, boresight rate
//! damping or a 0.1 s task, so it is not an artifact of the stand-ins above;
//! both modules implement upstream's equations. Upstream's documentation
//! shows the plate cooling over the second orbit; upstream's test only
//! checks that the script raises no `OSError`, which non-finite states do
//! not. Here the run stops at the first step whose body rate exceeds
//! [`DIVERGED_RATE_RADPS`] and reports it; the CSV ends there.
//!
//! Writes `examples/output/scenarios/scenarioSensorThermal.csv` with
//! `temperature_C`, `sigma_BR_*` and `r_BN_N_m_*`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/staged.rs"]
mod staged;

use std::path::PathBuf;

use attitude::{
    HUB_MASS_KG, external_disturbance, hub_inertia_kg_m2, mrp_feedback, scenario_recorder,
    vehicle_config,
};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::environment::anise_sun_ephemeris::{AniseSunEphemeris, AniseSunEphemerisConfig};
use basilisk_rs::environment::ephemeris_converter::{EphemerisConverter, EphemerisConverterConfig};
use basilisk_rs::fsw_algorithms::att_guidance::location_pointing::{
    LocationPointing, LocationPointingConfig,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, Output, PlanetStateMsg, SpacecraftStateMsg, TemperatureMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::thermal::sensor_thermal::{SensorThermal, SensorThermalConfig};
use basilisk_rs::utilities::astro_constants::{D2R, MU_EARTH, MU_SUN};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;
use staged::{Staged, UpdatedAtInit};

/// Upstream `simulationTimeStep = macros.sec2nano(1.0)`.
const TASK_PERIOD_NANOS: u64 = 1_000_000_000;
/// The DE440s planetary SPK (`just fetch-anise-assets`).
const PLANETARY_SPK: &str = "assets/anise/de440s.bsp";
/// \[rad/s\] body rate above which the attitude loop has diverged and the
/// run stops (see the module docs).
const DIVERGED_RATE_RADPS: f64 = 1.0;

/// Upstream `run(show_plots)`: no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The single run of upstream's `test_scenarioTest.py` entry.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config { write_csv: true }];

/// The recorded histories, every step.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    temperature: MessageLog<TemperatureMsg>,
    guidance: MessageLog<AttitudeGuidanceMsg>,
    /// Upstream `int(P)` \[s\]: when the pointed axis flips.
    flip_time_s: u64,
    /// Upstream `int(2 P)` \[s\]: the stop time.
    stop_time_s: u64,
    /// When the run stopped on a diverged attitude \[s\], if it did.
    diverged_at_s: Option<u64>,
    csv_path: Option<PathBuf>,
}

/// Upstream `timeInitString = "2021 MAY 04 07:47:48.965 (UTC)"`.
fn epoch() -> Epoch {
    Epoch::from_gregorian_utc(2021, 5, 4, 7, 47, 48, 965_000_000)
}

fn run(config: Config) -> Results {
    let mu = MU_EARTH * 1e9;
    let elements = ClassicElements {
        semi_major_axis: (6378.0 + 600.0) * 1000.0,
        eccentricity: 0.01,
        inclination_rad: 63.3 * D2R,
        raan_rad: 88.2 * D2R,
        arg_periapsis_rad: 347.8 * D2R,
        true_anomaly_rad: 135.3 * D2R,
        ..Default::default()
    };
    let (initial_position_m, initial_velocity_mps) = elements_to_state(mu, &elements);
    let period_s = 2.0 * std::f64::consts::PI / (mu / elements.semi_major_axis.powi(3)).sqrt();
    // Upstream `macros.sec2nano(int(P))` and `int(2 * P)`.
    let flip_time_s = period_s as u64;
    let stop_time_s = (2.0 * period_s) as u64;

    let inertia = hub_inertia_kg_m2();
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: inertia,
        integration_step_nanos: TASK_PERIOD_NANOS,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: Vector3::new(0.1, 0.2, -0.3),
        initial_omega_radps: Vector3::new(0.001, -0.01, 0.03),
        integrator: None,
    });
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass("earth", mu, true, Vector3::zeros(), Vector3::zeros())
                .expect("valid point-mass Earth"),
        )
        .expect("unique central gravity body");
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass(
                "sun",
                MU_SUN * 1e9,
                false,
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .expect("valid point-mass Sun"),
        )
        .expect("unique gravity body");
    let mut sun_ephemeris = UpdatedAtInit::new(
        AniseSunEphemeris::new(AniseSunEphemerisConfig {
            name: "spiceObject".to_string(),
            spk_path: PLANETARY_SPK.into(),
        }),
        epoch(),
    );
    let earth_state = Output::new(PlanetStateMsg::default());
    let mut ephemeris_converter = EphemerisConverter::new(EphemerisConverterConfig {
        name: "ephemConverter".to_string(),
        body_names: vec!["sun".to_string(), "earth".to_string()],
    });

    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut pointing = Staged::new(LocationPointing::new(LocationPointingConfig {
        name: "locPoint".to_string(),
        p_hat_b: Vector3::z(),
        ..LocationPointingConfig::default()
    }))
    .after(flip_time_s * 1_000_000_000, |pointing, _| {
        pointing.config.p_hat_b = -Vector3::z();
    });
    let mut controller = mrp_feedback(5.5, -1.0, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));
    let mut thermal_sensor = SensorThermal::new(SensorThermalConfig {
        name: "thermalSensor".to_string(),
        n_hat_b: Vector3::z(),
        sensor_power_draw_w: 30.0,
        sensor_area_m2: 1.0,
        sensor_absorptivity: 0.25,
        sensor_emissivity: 0.34,
        sensor_mass_kg: 2.0,
        sensor_specific_heat_j_per_kg_k: 890.0,
        initial_temperature_c: 0.0,
    });

    let mut temperature_log = MessageLog::<TemperatureMsg>::default();
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder("scenarioSensorThermal", config.write_csv);

    let spacecraft_state = spacecraft.state_out.clone();
    let mut simulation = Simulation::new(epoch(), false);
    connect!(&simulation,
        &sun_ephemeris.module.output_planet_msg => spacecraft
            .grav_body_input_mut("sun")
            .expect("the Sun is a gravity body"),
        &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg,
    );
    spacecraft.add_dynamic_effector(external_torque);
    let ephemeris_inputs = &mut ephemeris_converter.planet_in_msgs;
    connect!(&simulation,
        &sun_ephemeris.module.output_planet_msg => &mut ephemeris_inputs[0],
        &earth_state => &mut ephemeris_inputs[1],
    );
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut pointing.module.sc_att_in_msg,
        &navigation.translation_out_msg => &mut pointing.module.sc_trans_in_msg,
        &ephemeris_converter.ephem_out_msgs[0] => &mut pointing.module.cel_body_in_msg,
        &pointing.module.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &sun_ephemeris.module.output_sun_msg => &mut thermal_sensor.sun_in_msg,
        &spacecraft.state_out => &mut thermal_sensor.state_in_msg,
        &thermal_sensor.temperature_out_msg => &mut temperature_log.input,
        &pointing.module.att_guid_out_msg => &mut guidance_log.input,
        &thermal_sensor.temperature_out_msg => recorder.add_source::<TemperatureMsg>(
            CsvSourceConfig::columns([("temperature_c", "temperature_C")]),
        ),
        &pointing.module.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_br", "sigma_BR")),
        ),
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(vector_columns("position_m", "r_BN_N_m")),
        ),
    );
    // Upstream's priorities: navigation (101), pointing (99), control (98)
    // and the torque effector (97) ahead of the spacecraft and the rest.
    schedule! { simulation,
        "spice" => &mut sun_ephemeris, period=TASK_PERIOD_NANOS, priority=110;
        "navigation" => &mut navigation, period=TASK_PERIOD_NANOS, priority=101;
        "loc_point" => &mut pointing, period=TASK_PERIOD_NANOS, priority=99;
        "mrp_feedback" => &mut controller, period=TASK_PERIOD_NANOS, priority=98;
        "spacecraft" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=50;
        "ephem_converter" => &mut ephemeris_converter, period=TASK_PERIOD_NANOS, priority=40;
        "thermal_sensor" => &mut thermal_sensor, period=TASK_PERIOD_NANOS, priority=30;
        "temperature_log" => &mut temperature_log, period=TASK_PERIOD_NANOS, priority=0;
        "guidance_log" => &mut guidance_log, period=TASK_PERIOD_NANOS, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, TASK_PERIOD_NANOS, 0);
    }
    simulation.run_for(flip_time_s * 1_000_000_000);
    let completed = simulation.run_for_chunked_while(
        (stop_time_s - flip_time_s) * 1_000_000_000,
        TASK_PERIOD_NANOS,
        || spacecraft_state.read().omega_radps.norm() < DIVERGED_RATE_RADPS,
    );
    let diverged_at_s = (!completed).then(|| simulation.current_sim_nanos() / 1_000_000_000);
    drop(simulation);
    drop(recorder);

    Results {
        temperature: temperature_log,
        guidance: guidance_log,
        flip_time_s,
        stop_time_s,
        diverged_at_s,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_sensor_thermal [--all]");
    cli.switch("all");
    cli.finish();

    for config in TEST_PARAMETRIZATIONS {
        let results = run(config);
        let flip = results.temperature.at(results.flip_time_s * 1_000_000_000);
        println!(
            "wrote {} (sensor {:.1} °C at the flip, {:.1} °C at the end)",
            results.csv_path.as_ref().expect("CSV written").display(),
            flip.temperature_c,
            results.temperature.last().temperature_c,
        );
        if let Some(time) = results.diverged_at_s {
            println!(
                "warning: the attitude diverged after the flip at {} s; stopped at {time} s of \
                 {} s (see the scenario docs)",
                results.flip_time_s, results.stop_time_s,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    //! Upstream only runs this scenario (an entry of `test_scenarioTest.py`).
    //! These tests check the first orbit, which demonstrates the heating:
    //! the plate, facing the Sun once the pointing has settled, warms from
    //! 0 °C to near the radiative equilibrium of the absorbed sunlight and
    //! the 30 W it dissipates (`εσT⁴ = αS + P` for the 1 m^2 plate). The
    //! second orbit diverges (see the module docs), so it is not checked.

    use super::*;

    const SOLAR_FLUX_W_PER_M2: f64 = 1361.0;
    const BOLTZMANN_W_PER_M2_K4: f64 = 5.76051e-8;

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    #[test]
    fn the_sensor_warms_while_it_faces_the_sun() {
        let results = quiet();
        let flip = results.flip_time_s as usize;
        assert_eq!(results.temperature.values[0].temperature_c, 0.0);
        // The pointing settles within a quarter orbit.
        for guidance in &results.guidance.values[flip / 4..=flip] {
            assert!(guidance.sigma_br.norm() < 1.0e-3);
        }
        let absorbed_w = 0.25 * SOLAR_FLUX_W_PER_M2 + 30.0;
        let equilibrium_c = (absorbed_w / (0.34 * BOLTZMANN_W_PER_M2_K4)).powf(0.25) - 273.15;
        let at_flip = results.temperature.values[flip].temperature_c;
        assert!(
            (at_flip - equilibrium_c).abs() < 5.0,
            "{at_flip} vs {equilibrium_c}"
        );
        // Warming throughout the sunlit orbit, once sun facing.
        let warming = results.temperature.values[flip / 4..=flip]
            .windows(2)
            .all(|pair| pair[1].temperature_c >= pair[0].temperature_c - 1.0e-9);
        assert!(warming);
    }
}
