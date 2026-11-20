//! A planetary flyby from a custom SPK trajectory, with attitude pointing
//! modes: port of upstream `examples/scenarioFlybySpice.py`.
//!
//! The trajectory of spacecraft `-60000` in upstream's
//! `spacecraft_21T01.bsp` passes Venus, the Earth and Mars. Relative to the
//! Earth (upstream's `zeroBase = "Earth"`) it prescribes the translation of
//! a 750 kg spacecraft through its `transRefInMsg`; the Earth (central),
//! Sun, Moon, Venus and Mars barycenter are point-mass gravity bodies. An
//! `ephemerisConverter` turns the planet states into the ephemerides the
//! guidance modules read:
//!
//! * `velocityPoint` about the flyby planet (Venus, Earth or Mars, with
//!   that body's `mu`), with `sigma_R0R = [tan(90°/4), 0, 0]`;
//! * `locationPointing` of the body `+z` axis at the Earth (the antenna) and
//!   at the Sun (the panels), with boresight rate damping;
//! * `hillPoint` about the flyby planet, with `sigma_R0R = [-1/3, 1/3, -1/3]`
//!   (the camera at `[0, 1.5, 0] m` on the planet).
//!
//! `attTrackingError` follows one of them at a time and `mrpFeedback`
//! (`P = 2 · 900 / 180`, `K = P² / 900`, no integral feedback) commands an
//! `extForceTorque` effector. One 10 s task, from the flyby planet's start
//! epoch: velocity pointing for 4 h, antenna Earth pointing for 4 h, science
//! pointing for 12 h, antenna pointing for 4 h, Sun pointing for 4 h.
//! Upstream switches the tracking error's reference and `sigma_R0R` between
//! `ExecuteSimulation` calls; here the change is staged for the first update
//! after each stop time ([`Staged`]), the update upstream's changed module
//! makes next.
//!
//! Upstream's `spiceInterface` reads the SPK with CSPICE; the SPK is of type
//! 13, which ANISE reads, so the trajectory and planets come from ANISE with
//! DE440s and the default build runs the scenario. Upstream's Vizard
//! transceiver and camera are not ported; the attitude and trajectory are
//! written to CSV.
//!
//! Upstream `run(planetCase)` maps to `--planet venus|earth|mars` (the
//! script's default `mars`); `--all` runs the three cases of upstream's
//! `test_scenarioFlybySpice.py`. Reads `assets/spice/spacecraft_21T01.bsp`
//! (`just fetch-misc-scenario-data`); writes
//! `examples/output/scenarios/scenarioFlybySpice_<planet>.csv`.

#[path = "support/att_scenario.rs"]
mod att_scenario;
#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/ephemeris.rs"]
mod ephemeris;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/staged.rs"]
mod staged;

use std::path::PathBuf;
use std::str::FromStr;

use att_scenario::{attitude_columns, guidance_columns, rigid_spacecraft, torque_columns};
use attitude::{
    HubState, att_tracking_error, external_disturbance, hub_inertia_kg_m2, mrp_feedback,
    sampling_period_nanos, scenario_recorder, vehicle_config,
};
use basilisk_rs::environment::ephemeris_converter::{EphemerisConverter, EphemerisConverterConfig};
use basilisk_rs::fsw_algorithms::att_guidance::hill_point::{HillPoint, HillPointConfig};
use basilisk_rs::fsw_algorithms::att_guidance::location_pointing::{
    LocationPointing, LocationPointingConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::velocity_point::{
    VelocityPoint, VelocityPointConfig,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, AttitudeReferenceMsg, BodyTorqueCommandMsg, NavigationAttitudeMsg, Output,
    PlanetStateMsg, SpacecraftStateMsg, TranslationReferenceMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use ephemeris::{SPACECRAFT_21T01_ID, SPACECRAFT_21T01_SPK, SpkSpacecraft};
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;
use staged::Staged;

/// Upstream `simulationTimeStep = macros.sec2nano(10.)`.
const STEP_NANOS: u64 = 10_000_000_000;
const HOUR_NANOS: u64 = 3_600_000_000_000;
/// Upstream `gravFactory.createBodies("earth", "sun", "moon", "venus",
/// "mars barycenter")`, in the order of `planetStateOutMsgs`.
const BODIES: [Body; 5] = [
    Body::Earth,
    Body::Sun,
    Body::Moon,
    Body::Venus,
    Body::MarsBarycenter,
];
const EARTH: usize = 0;
const SUN: usize = 1;
const VENUS: usize = 3;
const MARS: usize = 4;

/// Upstream `planetCase`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Planet {
    Venus,
    Earth,
    Mars,
}

impl Planet {
    const ALL: [Self; 3] = [Self::Venus, Self::Earth, Self::Mars];

    fn name(self) -> &'static str {
        match self {
            Self::Venus => "venus",
            Self::Earth => "earth",
            Self::Mars => "mars",
        }
    }

    /// Upstream `timeInitString` (UTC).
    fn start_epoch(self) -> Epoch {
        match self {
            Self::Venus => Epoch::from_gregorian_utc(2028, 8, 13, 0, 30, 30, 0),
            Self::Earth => Epoch::from_gregorian_utc(2029, 6, 20, 5, 30, 30, 0),
            Self::Mars => Epoch::from_gregorian_utc(2031, 10, 3, 20, 0, 0, 0),
        }
    }

    /// Upstream `planetIdx`.
    fn index(self) -> usize {
        match self {
            Self::Venus => VENUS,
            Self::Earth => EARTH,
            Self::Mars => MARS,
        }
    }
}

impl FromStr for Planet {
    type Err = String;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        Self::ALL
            .into_iter()
            .find(|planet| planet.name() == text)
            .ok_or_else(|| format!("flyby target {text} not implemented"))
    }
}

/// The flight modes, upstream's `run*Pointing` functions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    /// `runVelocityPointing`.
    Velocity,
    /// `runAntennaEarthPointing`.
    AntennaEarth,
    /// `runSensorSciencePointing`.
    Science,
    /// `runPanelSunPointing`.
    PanelSun,
}

impl Mode {
    /// Upstream's `attError.sigma_R0R` of the mode.
    fn sigma_r0r(self) -> Vector3<f64> {
        match self {
            Self::Velocity => Vector3::new((90.0_f64.to_radians() / 4.0).tan(), 0.0, 0.0),
            Self::AntennaEarth | Self::PanelSun => Vector3::zeros(),
            Self::Science => Vector3::new(-1.0 / 3.0, 1.0 / 3.0, -1.0 / 3.0),
        }
    }
}

/// Upstream's mode sequence and durations [h].
const SCHEDULE: [(Mode, u64); 5] = [
    (Mode::Velocity, 4),
    (Mode::AntennaEarth, 4),
    (Mode::Science, 12),
    (Mode::AntennaEarth, 4),
    (Mode::PanelSun, 4),
];

/// The end time of each mode [ns].
fn mode_ends_nanos() -> Vec<u64> {
    SCHEDULE
        .iter()
        .scan(0, |end, (_, hours)| {
            *end += hours * HOUR_NANOS;
            Some(*end)
        })
        .collect()
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    state: MessageLog<SpacecraftStateMsg>,
    reference: MessageLog<TranslationReferenceMsg>,
    guidance: MessageLog<AttitudeGuidanceMsg>,
    planet: MessageLog<PlanetStateMsg>,
    csv_path: Option<PathBuf>,
}

fn run(planet: Planet, write_csv: bool) -> Results {
    let inertia = hub_inertia_kg_m2();
    let initial = HubState {
        position_m: Vector3::zeros(),
        velocity_mps: Vector3::zeros(),
        sigma_bn: Vector3::new(0.1, 0.2, -0.3),
        omega_bn_b_radps: Vector3::zeros(),
    };
    let mut spacecraft = rigid_spacecraft(inertia, &initial, STEP_NANOS);
    let mut ephemerides = Vec::new();
    for body in BODIES {
        spacecraft
            .add_grav_body(body.point_mass(body == Body::Earth))
            .expect("unique gravity bodies");
        ephemerides.push(body.ephemeris(Body::Earth));
    }
    let mut trajectory = SpkSpacecraft::new(
        "-60000",
        SPACECRAFT_21T01_ID,
        SPACECRAFT_21T01_SPK,
        Body::Earth.data().frame,
    );
    let mut converter = EphemerisConverter::new(EphemerisConverterConfig {
        name: "EphemData".to_string(),
        body_names: BODIES.iter().map(|body| body.name().to_string()).collect(),
    });
    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");

    let velocity_point = |name: &str, body: Body| {
        VelocityPoint::new(VelocityPointConfig {
            name: name.to_string(),
            mu_m3ps2: body.mu_m3ps2(),
        })
    };
    let mut velocity_venus = velocity_point("velocityPointVenus", Body::Venus);
    let mut velocity_earth = velocity_point("velocityPointEarth", Body::Earth);
    let mut velocity_mars = velocity_point("velocityPointMars", Body::MarsBarycenter);
    let location_pointing = |name: &str| {
        LocationPointing::new(LocationPointingConfig {
            name: name.to_string(),
            p_hat_b: Vector3::z(),
            use_boresight_rate_damping: true,
            ..LocationPointingConfig::default()
        })
    };
    let mut earth_point = location_pointing("antennaEarthPoint");
    let mut sun_point = location_pointing("panelSunPoint");
    let mut science_point = HillPoint::new(HillPointConfig {
        name: "sciencePointAsteroid".to_string(),
    });

    // Upstream's `P = 2 II / (3 * 60)`, `K = P^2 / II` with `II = 900`.
    let p = 2.0 * 900.0 / (3.0 * 60.0);
    let mut controller = mrp_feedback(p * p / 900.0, -1.0, p, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let flyby_velocity = match planet {
        Planet::Venus => velocity_venus.att_ref_out_msg.clone(),
        Planet::Earth => velocity_earth.att_ref_out_msg.clone(),
        Planet::Mars => velocity_mars.att_ref_out_msg.clone(),
    };
    let mode_reference = |mode: Mode| -> Output<AttitudeReferenceMsg> {
        match mode {
            Mode::Velocity => flyby_velocity.clone(),
            Mode::AntennaEarth => earth_point.att_ref_out_msg.clone(),
            Mode::Science => science_point.att_ref_out_msg.clone(),
            Mode::PanelSun => sun_point.att_ref_out_msg.clone(),
        }
    };
    let ends = mode_ends_nanos();
    let mut tracking_error = Staged::new(att_tracking_error(SCHEDULE[0].0.sigma_r0r()));
    for (index, (mode, _)) in SCHEDULE.iter().enumerate().skip(1) {
        let reference = mode_reference(*mode);
        let sigma_r0r = mode.sigma_r0r();
        tracking_error = tracking_error.after(ends[index - 1], move |error, _context| {
            reference.connect_to(&mut error.att_ref_in_msg);
            error.config.sigma_r0r = sigma_r0r;
        });
    }
    let duration_nanos = *ends.last().expect("modes");

    let sample_period = sampling_period_nanos(duration_nanos, STEP_NANOS, 1000);
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut reference_log = MessageLog::<TranslationReferenceMsg>::default();
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut planet_log = MessageLog::<PlanetStateMsg>::default();
    let stem = format!("scenarioFlybySpice_{}", planet.name());
    let (mut recorder, csv_path) = scenario_recorder(&stem, write_csv);

    let mut simulation = Simulation::new(planet.start_epoch(), false);
    for ((body, ephemeris), converter_input) in BODIES
        .iter()
        .zip(&ephemerides)
        .zip(&mut converter.planet_in_msgs)
    {
        simulation.connect(
            &ephemeris.output_planet_msg,
            spacecraft
                .grav_body_input_mut(body.name())
                .expect("gravity body"),
        );
        simulation.connect(&ephemeris.output_planet_msg, converter_input);
    }
    connect!(&simulation, &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg);
    spacecraft.add_dynamic_effector(external_torque);
    let ephemeris_out = &converter.ephem_out_msgs;
    connect!(&simulation,
        &trajectory.trans_ref_out_msg => &mut spacecraft.trans_ref_in,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.translation_out_msg => &mut velocity_venus.trans_nav_in_msg,
        &ephemeris_out[VENUS] => &mut velocity_venus.cel_body_in_msg,
        &navigation.translation_out_msg => &mut velocity_earth.trans_nav_in_msg,
        &ephemeris_out[EARTH] => &mut velocity_earth.cel_body_in_msg,
        &navigation.translation_out_msg => &mut velocity_mars.trans_nav_in_msg,
        &ephemeris_out[MARS] => &mut velocity_mars.cel_body_in_msg,
        &navigation.translation_out_msg => &mut earth_point.sc_trans_in_msg,
        &navigation.attitude_out_msg => &mut earth_point.sc_att_in_msg,
        &ephemeris_out[EARTH] => &mut earth_point.cel_body_in_msg,
        &navigation.translation_out_msg => &mut sun_point.sc_trans_in_msg,
        &navigation.attitude_out_msg => &mut sun_point.sc_att_in_msg,
        &ephemeris_out[SUN] => &mut sun_point.cel_body_in_msg,
        &navigation.translation_out_msg => &mut science_point.trans_nav_in_msg,
        &ephemeris_out[planet.index()] => &mut science_point.cel_body_in_msg,
        &flyby_velocity => &mut tracking_error.module.att_ref_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.module.att_nav_in_msg,
        &tracking_error.module.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &spacecraft.state_out => &mut state_log.input,
        &trajectory.trans_ref_out_msg => &mut reference_log.input,
        &tracking_error.module.att_guid_out_msg => &mut guidance_log.input,
        &ephemerides[planet.index()].output_planet_msg => &mut planet_log.input,
        &tracking_error.module.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns()),
        &controller.cmd_torque_out_msg => recorder.add_source::<BodyTorqueCommandMsg>(torque_columns()),
        &navigation.attitude_out_msg => recorder.add_source::<NavigationAttitudeMsg>(attitude_columns()),
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(
                vector_columns("position_m", "r_BN_N_m")
                    .into_iter()
                    .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s")),
            ),
        ),
        &ephemerides[planet.index()].output_planet_msg => recorder.add_source::<PlanetStateMsg>(
            CsvSourceConfig::columns(vector_columns("position_inertial_m", "planet_r_N_m")),
        ),
    );
    for (body, ephemeris) in BODIES.iter().zip(&mut ephemerides) {
        simulation.add_module(
            format!("{}_ephemeris", body.name()),
            ephemeris,
            STEP_NANOS,
            120,
        );
    }
    // Upstream's task order.
    schedule! { simulation,
        "spiceInterface" => &mut trajectory, period=STEP_NANOS, priority=110;
        "spacecraft" => &mut spacecraft, period=STEP_NANOS, priority=100;
        "EphemData" => &mut converter, period=STEP_NANOS, priority=95;
        "SimpleNavigation" => &mut navigation, period=STEP_NANOS, priority=90;
        "velocityPointVenus" => &mut velocity_venus, period=STEP_NANOS, priority=85;
        "velocityPointEarth" => &mut velocity_earth, period=STEP_NANOS, priority=84;
        "velocityPointMars" => &mut velocity_mars, period=STEP_NANOS, priority=83;
        "antennaEarthPoint" => &mut earth_point, period=STEP_NANOS, priority=82;
        "panelSunPoint" => &mut sun_point, period=STEP_NANOS, priority=81;
        "sciencePointAsteroid" => &mut science_point, period=STEP_NANOS, priority=80;
        "attErrorInertial3D" => &mut tracking_error, period=STEP_NANOS, priority=70;
        "mrpFeedback" => &mut controller, period=STEP_NANOS, priority=60;
        "state_log" => &mut state_log, period=sample_period, priority=0;
        "reference_log" => &mut reference_log, period=sample_period, priority=0;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "planet_log" => &mut planet_log, period=sample_period, priority=0;
    }
    if write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    // Upstream's `run*Pointing` calls, each to the end of its mode.
    let mut now = 0;
    for end in ends {
        simulation.run_for(end - now);
        now = end;
    }
    drop(simulation);
    drop(recorder);

    Results {
        state: state_log,
        reference: reference_log,
        guidance: guidance_log,
        planet: planet_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_flyby_spice [--planet venus|earth|mars] [--all]");
    let planets = if cli.switch("all") {
        Planet::ALL.to_vec()
    } else {
        // Upstream's `__main__` runs `run("mars")`.
        vec![cli.value("planet", Planet::Mars)]
    };
    cli.finish();
    for planet in planets {
        let results = run(planet, true);
        let closest_km = results
            .state
            .values
            .iter()
            .zip(&results.planet.values)
            .map(|(state, body)| (state.position_m - body.position_inertial_m).norm() * 1e-3)
            .fold(f64::INFINITY, f64::min);
        println!(
            "wrote {} (closest approach {closest_km:.0} km)",
            results.csv_path.expect("CSV written").display()
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioFlybySpice.py` only checks that each case
    //! runs. These tests check what the scenario demonstrates: the
    //! spacecraft flies past the planet on the SPK trajectory, and the
    //! attitude follows each pointing mode.

    use super::*;

    #[test]
    fn each_flyby_follows_the_trajectory_and_the_pointing_modes() {
        for planet in Planet::ALL {
            let results = run(planet, false);
            for (state, reference) in results.state.values.iter().zip(&results.reference.values) {
                assert_eq!(state.position_m, reference.position_m);
                assert_eq!(state.velocity_mps, reference.velocity_mps);
            }
            // The flyby: the planet is passed within the run.
            let distances_km: Vec<f64> = results
                .state
                .values
                .iter()
                .zip(&results.planet.values)
                .map(|(state, body)| (state.position_m - body.position_inertial_m).norm() * 1e-3)
                .collect();
            let (closest, closest_km) = distances_km.iter().copied().enumerate().fold(
                (0, f64::INFINITY),
                |best, (index, distance)| {
                    if distance < best.1 {
                        (index, distance)
                    } else {
                        best
                    }
                },
            );
            assert!(
                closest > 0 && closest < distances_km.len() - 1,
                "{planet:?}"
            );
            assert!(closest_km < 50_000.0, "{planet:?}: {closest_km} km");
            // Each mode's reference is tracked by the end of the mode.
            for end in mode_ends_nanos() {
                let guidance = results.guidance.at(end);
                assert!(
                    guidance.sigma_br.norm() < 1e-3,
                    "{planet:?} at {end}: {}",
                    guidance.sigma_br
                );
            }
        }
    }
}
