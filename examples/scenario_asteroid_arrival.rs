//! Arrival at asteroid Bennu, capture into an 800 m-altitude circular orbit
//! and a series of Hohmann transfers down to a 200 m-periapsis ellipse,
//! flying four attitude modes: port of upstream
//! `examples/scenarioAsteroidArrival.py`.
//!
//! Bennu (`mu = G m = 6.67408e-11 · 7.329e10 m^3/s^2`, 490 m across, the
//! central body) follows a two-body heliocentric `planetEphemeris` orbit
//! (`a = 1.1264 AU`, `e = 0.20375`) and spins about its pole (RA 85.65 deg,
//! dec -60.17 deg, a 4.296057 h period); the Earth and the Sun are point
//! masses from the ephemeris (upstream `createSpiceInterface` from
//! `2011 January 1 0:00:00.0`, relative to the solar-system barycenter). A
//! 750 kg spacecraft (`I = diag(900, 800, 600) kg m^2`) arrives on a
//! `rP = 1045 m`, `rA = 3 rP` polar orbit 45 deg before periapsis.
//! `simpleNav` feeds four guidance modules, `locationPointing` to the Sun
//! (body `+z`, the panel normal), to Bennu (body `+y`, the science camera)
//! and to the Earth (body `+z`, the antenna), and `velocityPoint` about
//! Bennu; the selected one drives `attTrackingError`, `mrpFeedback`
//! (`P = 2 · 900 / 1200`, `K = P^2 / 900`, no integral feedback) and an
//! `extForceTorque` effector. One 20 s task.
//!
//! The flight plan is upstream's: the `runDvBurn`, `runSensorSciencePointing`,
//! `runPanelSunPointing` and `runAntennaEarthPointing` legs ([`Step::Fly`],
//! [`FlightMode`]), and between them the velocity changes upstream sets
//! through `velRef.setState` ([`Step::Impulse`]): capture at periapsis into
//! the `r0` circle, then Hohmann transfers `r0 → r1`, `r1 → r2` and the first
//! burn of `r2 → r3`. The burns are impulsive; the `runDvBurn` legs only
//! turn the spacecraft (velocity pointing rotated -90 deg about its first
//! axis), and their thruster plume is Vizard decoration.
//!
//! Mapping to basilisk-rs:
//!
//! * upstream's `attError.attRefInMsg.subscribeTo(...)` and
//!   `attError.sigma_R0R = ...` at each leg are a [`Staged`] change of the
//!   tracking-error module at the leg's start time;
//! * `velRef.setState(v)` is
//!   [`HubStateHandle::set_velocity_mps`](basilisk_rs::spacecraft::HubStateHandle),
//!   with `v` computed, as upstream, from the last spacecraft and Bennu states
//!   (read here through inputs connected to their outputs);
//! * the Earth and Sun states come from ANISE (DE440s) instead of CSPICE
//!   (`grav_factory`);
//! * upstream adds `extForceTorque` to the task, where it reads the torque
//!   command once per step before `mrpFeedback` writes the next one; here the
//!   effector reads the command when the spacecraft integrates, as in the
//!   other attitude scenario ports;
//! * the Vizard transceiver, sensor, camera and thruster decorations are not
//!   ported.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted and runs
//! the same single case (upstream's `test_scenarioTest.py` entry).
//!
//! Writes `examples/output/scenarios/scenarioAsteroidArrival.csv` every step:
//! `r_BN_N_m_*`, `v_BN_N_m_per_s_*`, `sigma_BN_*`, Bennu's
//! `bennu_r_N_m_*`, `bennu_v_N_m_per_s_*`, and `sigma_BR_*`.

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
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/staged.rs"]
mod staged;

use std::f64::consts::PI;
use std::path::PathBuf;

use attitude::{
    HUB_MASS_KG, att_tracking_error, external_disturbance, hub_inertia_kg_m2, mrp_feedback,
    scenario_recorder, vehicle_config,
};
use basilisk_rs::environment::ephemeris_converter::{EphemerisConverter, EphemerisConverterConfig};
use basilisk_rs::environment::planet_ephemeris::{
    PlanetEphemeris, PlanetEphemerisBody, PlanetEphemerisConfig, PlanetRotation,
};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::AttTrackingError;
use basilisk_rs::fsw_algorithms::att_guidance::location_pointing::{
    LocationPointing, LocationPointingConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::velocity_point::{
    VelocityPoint, VelocityPointConfig,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, AttitudeReferenceMsg, EphemerisMsg, Input, Output, PlanetStateMsg,
    SpacecraftStateMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{AU, D2R};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::{Body, custom_point_mass};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;
use orbit_tools::{run_until, sec2nano};
use recording::planet_columns;
use staged::Staged;

/// Upstream `simulationTimeStep = macros.sec2nano(20.0)`.
const TASK_PERIOD_NANOS: u64 = 20_000_000_000;
/// Upstream `burnTime = 200 * 60` \[s\].
const BURN_TIME_S: f64 = 200.0 * 60.0;
/// Upstream `runDvBurn`'s `minTime = 40 * 60` \[s\]: the first stop of a
/// burn leg.
const MIN_BURN_LEG_S: f64 = 40.0 * 60.0;
/// Upstream `diam = 2 * 245.03` \[m\]: Bennu's diameter.
const DIAMETER_M: f64 = 2.0 * 245.03;
/// Upstream `cameraLocation`, the science-pointing axis.
const CAMERA_AXIS_B: [f64; 3] = [0.0, 1.5, 0.0];

/// Upstream `mu = G * massBennu` \[m^3/s^2\].
fn bennu_mu_m3ps2() -> f64 {
    6.67408e-11 * 7.329e10
}

/// Upstream's attitude modes: the guidance `attError` follows and its
/// `sigma_R0R`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlightMode {
    /// `runPanelSunPointing`: body `+z` to the Sun.
    PanelSunPointing,
    /// `runSensorSciencePointing`: body `+y` to Bennu.
    SensorSciencePointing,
    /// `runAntennaEarthPointing`: body `+z` to the Earth.
    AntennaEarthPointing,
    /// `runDvBurn(..., burnSign = -1, velAsteroidGuidance.attRefOutMsg)`:
    /// Bennu-relative velocity pointing with `sigma_R0R = [-tan(pi/8), 0,
    /// 0]`, a -90 deg turn about the first axis. Every upstream call passes
    /// `burnSign = -1`.
    DvBurn,
}

impl FlightMode {
    fn sigma_r0r(self) -> Vector3<f64> {
        match self {
            Self::DvBurn => Vector3::new(-(PI / 2.0 / 4.0).tan(), 0.0, 0.0),
            _ => Vector3::zeros(),
        }
    }
}

/// The velocity changes upstream applies between legs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Impulse {
    /// Into the `r0` circle at the arrival periapsis: `Delta_V_Parking_Orbit`
    /// along the Bennu-relative velocity.
    Capture,
    /// First Hohmann burn from the current radius to `target_radius_m`.
    TransferTo { target_radius_m: f64 },
    /// Second Hohmann burn: the circular speed at the current radius.
    Circularize,
}

/// One step of upstream's flight plan.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Step {
    /// Fly `duration_s` in a mode.
    Fly(FlightMode, f64),
    /// Change the Bennu-relative velocity.
    Impulse(Impulse),
}

/// Upstream's orbit radii and periods \[m, s\].
#[derive(Clone, Copy, Debug)]
pub struct Geometry {
    pub mu_m3ps2: f64,
    /// Upstream `r0`: the capture circle.
    pub r0_m: f64,
    /// Upstream `r1`: the intermediate circle.
    pub r1_m: f64,
    /// Upstream `r2`: the science circle.
    pub r2_m: f64,
    /// Upstream `r3`: the final periapsis.
    pub r3_m: f64,
}

impl Geometry {
    fn new() -> Self {
        Self {
            mu_m3ps2: bennu_mu_m3ps2(),
            r0_m: DIAMETER_M / 2.0 + 800.0,
            r1_m: DIAMETER_M / 2.0 + 600.0,
            r2_m: DIAMETER_M / 2.0 + 400.0,
            r3_m: DIAMETER_M / 2.0 + 200.0,
        }
    }

    /// The period of an orbit of semi-major axis `a` \[s\].
    fn period_s(&self, a: f64) -> f64 {
        PI * 2.0 / (self.mu_m3ps2 / a.powi(3)).sqrt()
    }

    /// Upstream `oe`: the arrival orbit, `rP = r0`, `rA = 3 rP`, 45 deg
    /// before periapsis.
    fn arrival_elements(&self) -> ClassicElements {
        let (rp, ra) = (self.r0_m, 3.0 * self.r0_m);
        let a = (rp + ra) / 2.0;
        ClassicElements {
            semi_major_axis: a,
            eccentricity: 1.0 - rp / a,
            inclination_rad: 90.0 * D2R,
            raan_rad: 180.0 * D2R,
            arg_periapsis_rad: 347.8 * D2R,
            true_anomaly_rad: -45.0 * D2R,
            ..Default::default()
        }
    }

    /// Upstream `T1 = M / n` \[s\]: the time to the arrival periapsis.
    fn time_to_periapsis_s(&self) -> f64 {
        let oe = self.arrival_elements();
        let e = oe.eccentricity;
        let ecc =
            ((-oe.true_anomaly_rad / 2.0).tan() * ((1.0 - e) / (1.0 + e)).sqrt()).atan() * 2.0;
        let mean_anomaly = ecc - e * ecc.sin();
        mean_anomaly / (self.mu_m3ps2 / oe.semi_major_axis.powi(3)).sqrt()
    }

    /// Upstream `Delta_V_Parking_Orbit = sqrt(mu / rP) - h / rP` \[m/s\].
    fn capture_dv_mps(&self) -> f64 {
        let oe = self.arrival_elements();
        let h = (self.mu_m3ps2 * oe.semi_major_axis * (1.0 - oe.eccentricity.powi(2))).sqrt();
        (self.mu_m3ps2 / self.r0_m).sqrt() - h / self.r0_m
    }

    /// Upstream's sequence of legs and velocity changes.
    fn flight_plan(&self) -> Vec<Step> {
        use FlightMode::{AntennaEarthPointing, DvBurn, PanelSunPointing, SensorSciencePointing};
        use Step::{Fly, Impulse as Burn};
        let burn = BURN_TIME_S;
        let p0 = self.period_s(self.r0_m);
        let p01 = self.period_s((self.r0_m + self.r1_m) / 2.0);
        let p1 = self.period_s(self.r1_m);
        let p12 = self.period_s((self.r1_m + self.r2_m) / 2.0);
        let p2 = self.period_s(self.r2_m);
        let p23 = self.period_s((self.r2_m + self.r3_m) / 2.0);
        vec![
            Fly(DvBurn, self.time_to_periapsis_s()),
            Burn(Impulse::Capture),
            // The circular capture orbit at r0.
            Fly(DvBurn, burn),
            Fly(SensorSciencePointing, p0 / 3.0 - burn),
            Fly(PanelSunPointing, p0 / 3.0),
            Fly(AntennaEarthPointing, p0 / 3.0 - burn),
            Fly(DvBurn, burn),
            Burn(Impulse::TransferTo {
                target_radius_m: self.r1_m,
            }),
            Fly(DvBurn, burn),
            Fly(PanelSunPointing, p01 / 2.0 - burn * 2.0),
            Fly(DvBurn, burn),
            Burn(Impulse::Circularize),
            Fly(DvBurn, burn),
            Fly(SensorSciencePointing, p1 / 4.0 - burn),
            Fly(AntennaEarthPointing, p1 / 4.0 - burn),
            Fly(DvBurn, burn),
            Burn(Impulse::TransferTo {
                target_radius_m: self.r2_m,
            }),
            Fly(DvBurn, burn),
            Fly(SensorSciencePointing, p12 / 2.0 - burn * 2.0),
            Fly(DvBurn, burn),
            Burn(Impulse::Circularize),
            Fly(DvBurn, burn),
            Fly(SensorSciencePointing, p2 - burn),
            Burn(Impulse::TransferTo {
                target_radius_m: self.r3_m,
            }),
            Fly(DvBurn, burn),
            Fly(SensorSciencePointing, 3.0 * p23 - burn),
        ]
    }
}

/// Upstream's accumulation of `simulationTime` over a leg \[ns\]:
/// `runDvBurn` stops after `minTime` and again at the end.
fn leg_nanos(mode: FlightMode, duration_s: f64) -> u64 {
    if mode == FlightMode::DvBurn {
        assert!(
            duration_s >= MIN_BURN_LEG_S,
            "runDvBurn must have simTime larger than {MIN_BURN_LEG_S} s"
        );
        sec2nano(MIN_BURN_LEG_S) + sec2nano(duration_s - MIN_BURN_LEG_S)
    } else {
        sec2nano(duration_s)
    }
}

/// A flown leg.
#[derive(Clone, Copy, Debug)]
pub struct Leg {
    pub mode: FlightMode,
    pub start_nanos: u64,
    pub stop_nanos: u64,
}

/// An applied velocity change.
#[derive(Clone, Copy, Debug)]
pub struct AppliedImpulse {
    pub impulse: Impulse,
    /// The stop time it follows \[ns\].
    pub time_nanos: u64,
    /// Bennu-relative velocity before and after \[m/s\].
    pub velocity_before_mps: Vector3<f64>,
    pub velocity_after_mps: Vector3<f64>,
}

/// Upstream `run(show_plots)`: no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The single run of upstream's `test_scenarioTest.py` entry.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config { write_csv: true }];

/// The recorded histories, every step.
// Some fields are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    geometry: Geometry,
    /// Upstream `scRec`.
    spacecraft: MessageLog<SpacecraftStateMsg>,
    /// Upstream `astRec`.
    asteroid: MessageLog<PlanetStateMsg>,
    /// `attError.attGuidOutMsg`.
    guidance: MessageLog<AttitudeGuidanceMsg>,
    /// The Earth and Sun ephemerides the pointing modes aim at.
    earth: MessageLog<EphemerisMsg>,
    sun: MessageLog<EphemerisMsg>,
    legs: Vec<Leg>,
    impulses: Vec<AppliedImpulse>,
    csv_path: Option<PathBuf>,
}

impl Results {
    /// Upstream `posData2 = scRec.r_BN_N - astRec.PositionVector` and the
    /// matching relative velocity.
    fn relative_state(&self, index: usize) -> (Vector3<f64>, Vector3<f64>) {
        let spacecraft = &self.spacecraft.values[index];
        let asteroid = &self.asteroid.values[index];
        (
            spacecraft.position_m - asteroid.position_inertial_m,
            spacecraft.velocity_mps - asteroid.velocity_inertial_mps,
        )
    }

    /// Sample indices in `(start, stop]` of time.
    #[cfg_attr(not(test), allow(dead_code))]
    fn samples_in(&self, start_nanos: u64, stop_nanos: u64) -> impl Iterator<Item = usize> + '_ {
        self.spacecraft
            .times_nanos
            .iter()
            .enumerate()
            .filter(move |(_, time)| **time > start_nanos && **time <= stop_nanos)
            .map(|(index, _)| index)
    }
}

/// The new Bennu-relative velocity of `impulse`, upstream's arithmetic on
/// the last recorded states.
fn impulse_velocity(
    impulse: Impulse,
    geometry: &Geometry,
    relative_position_m: Vector3<f64>,
    relative_velocity_mps: Vector3<f64>,
) -> Vector3<f64> {
    let mu = geometry.mu_m3ps2;
    let r = relative_position_m.norm();
    let v = relative_velocity_mps.norm();
    let v_hat = relative_velocity_mps / v;
    match impulse {
        Impulse::Capture => relative_velocity_mps + geometry.capture_dv_mps() * v_hat,
        Impulse::TransferTo { target_radius_m } => {
            let at = (r + target_radius_m) * 0.5;
            let vp = ((2.0 * mu / r) - (mu / at)).sqrt();
            relative_velocity_mps + v_hat * (vp - v)
        }
        Impulse::Circularize => relative_velocity_mps + v_hat * ((mu / r).sqrt() - v),
    }
}

fn run(config: Config) -> Results {
    let geometry = Geometry::new();
    let mu = geometry.mu_m3ps2;

    let mut bennu_ephemeris = PlanetEphemeris::new(PlanetEphemerisConfig {
        name: "planetEphemeris".to_string(),
        bodies: vec![PlanetEphemerisBody {
            name: "bennu".to_string(),
            elements: ClassicElements {
                semi_major_axis: 1.1264 * AU * 1000.0,
                eccentricity: 0.20375,
                inclination_rad: 6.0349 * D2R,
                raan_rad: 2.0609 * D2R,
                arg_periapsis_rad: 66.2231 * D2R,
                true_anomaly_rad: 0.0,
                ..Default::default()
            },
            rotation: Some(PlanetRotation {
                right_ascension_rad: 85.65 * D2R,
                declination_rad: -60.17 * D2R,
                lst0_rad: 0.0,
                rotation_rate_radps: 360.0 * D2R / (4.296057 * 3600.0),
            }),
        }],
        zero_base: None,
    });
    let mut earth_ephemeris = Body::Earth.ephemeris_from_ssb();
    let mut sun_ephemeris = Body::Sun.ephemeris_from_ssb();
    let mut ephemeris_converter = EphemerisConverter::new(EphemerisConverterConfig {
        name: "EphemData".to_string(),
        body_names: vec!["earth".to_string(), "sun".to_string(), "bennu".to_string()],
    });

    let (position_m, velocity_mps) = elements_to_state(mu, &geometry.arrival_elements());
    let inertia = hub_inertia_kg_m2();
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: inertia,
        integration_step_nanos: TASK_PERIOD_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::new(0.1, 0.2, -0.3),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    for body in [
        Body::Earth.point_mass(false),
        Body::Sun.point_mass(false),
        custom_point_mass("bennu", mu, true),
    ] {
        spacecraft
            .add_grav_body(body)
            .expect("unique gravity bodies");
    }
    let hub = spacecraft.hub_state_handle();

    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let location_pointing = |name: &str, p_hat_b: Vector3<f64>| {
        LocationPointing::new(LocationPointingConfig {
            name: name.to_string(),
            p_hat_b,
            use_boresight_rate_damping: true,
            ..LocationPointingConfig::default()
        })
    };
    let mut sun_point = location_pointing("panelSunPoint", Vector3::z());
    let mut velocity_point = VelocityPoint::new(VelocityPointConfig {
        name: "velocityPointAsteroid".to_string(),
        mu_m3ps2: mu,
    });
    let mut science_point = location_pointing("sciencePointAsteroid", Vector3::from(CAMERA_AXIS_B));
    let mut earth_point = location_pointing("antennaEarthPoint", Vector3::z());
    let mut controller = {
        let ii = 900.0;
        let p = 2.0 * ii / (20.0 * 60.0);
        mrp_feedback(p * p / ii, -1.0, p, Vector3::zeros())
    };
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    // The legs, and the tracking-error changes at their starts.
    let plan = geometry.flight_plan();
    let reference = |mode: FlightMode| -> Output<AttitudeReferenceMsg> {
        match mode {
            FlightMode::PanelSunPointing => sun_point.att_ref_out_msg.clone(),
            FlightMode::SensorSciencePointing => science_point.att_ref_out_msg.clone(),
            FlightMode::AntennaEarthPointing => earth_point.att_ref_out_msg.clone(),
            FlightMode::DvBurn => velocity_point.att_ref_out_msg.clone(),
        }
    };
    let mut legs = Vec::new();
    let mut stop_nanos = 0;
    for step in &plan {
        if let Step::Fly(mode, duration_s) = *step {
            let start_nanos = stop_nanos;
            stop_nanos += leg_nanos(mode, duration_s);
            legs.push(Leg {
                mode,
                start_nanos,
                stop_nanos,
            });
        }
    }
    let first_mode = legs[0].mode;
    let mut tracking_error = Staged::new(att_tracking_error(first_mode.sigma_r0r()));
    reference(first_mode).connect_to(&mut tracking_error.module.att_ref_in_msg);
    for leg in &legs[1..] {
        let (output, sigma_r0r) = (reference(leg.mode), leg.mode.sigma_r0r());
        tracking_error =
            tracking_error.after(leg.start_nanos, move |error: &mut AttTrackingError, _| {
                output.connect_to(&mut error.att_ref_in_msg);
                error.config.sigma_r0r = sigma_r0r;
            });
    }

    let mut spacecraft_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut asteroid_log = MessageLog::<PlanetStateMsg>::default();
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut earth_log = MessageLog::<EphemerisMsg>::default();
    let mut sun_log = MessageLog::<EphemerisMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder("scenarioAsteroidArrival", config.write_csv);
    // The last states, read between legs (upstream `scRec.v_BN_N[-1]`,
    // `astRec.VelocityVector[-1]`).
    let mut spacecraft_now = Input::<SpacecraftStateMsg>::default();
    let mut asteroid_now = Input::<PlanetStateMsg>::default();

    let mut impulses = Vec::new();
    {
        let start = Epoch::from_gregorian_utc_at_midnight(2011, 1, 1);
        let mut simulation = Simulation::new(start, false);
        connect!(&simulation,
            &bennu_ephemeris.planet_out_msgs[0] => spacecraft
                .grav_body_input_mut("bennu")
                .expect("Bennu is a gravity body"),
            &earth_ephemeris.output_planet_msg => spacecraft
                .grav_body_input_mut("earth")
                .expect("the Earth is a gravity body"),
            &sun_ephemeris.output_planet_msg => spacecraft
                .grav_body_input_mut("sun")
                .expect("the Sun is a gravity body"),
            &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg,
        );
        spacecraft.add_dynamic_effector(external_torque);
        let converter_inputs = &mut ephemeris_converter.planet_in_msgs;
        connect!(&simulation,
            &earth_ephemeris.output_planet_msg => &mut converter_inputs[0],
            &sun_ephemeris.output_planet_msg => &mut converter_inputs[1],
            &bennu_ephemeris.planet_out_msgs[0] => &mut converter_inputs[2],
        );
        let ephemerides = &ephemeris_converter.ephem_out_msgs;
        connect!(&simulation,
            &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
            &ephemerides[1] => &mut sun_point.cel_body_in_msg,
            &navigation.translation_out_msg => &mut sun_point.sc_trans_in_msg,
            &navigation.attitude_out_msg => &mut sun_point.sc_att_in_msg,
            &navigation.translation_out_msg => &mut velocity_point.trans_nav_in_msg,
            &ephemerides[2] => &mut velocity_point.cel_body_in_msg,
            &ephemerides[2] => &mut science_point.cel_body_in_msg,
            &navigation.translation_out_msg => &mut science_point.sc_trans_in_msg,
            &navigation.attitude_out_msg => &mut science_point.sc_att_in_msg,
            &ephemerides[0] => &mut earth_point.cel_body_in_msg,
            &navigation.translation_out_msg => &mut earth_point.sc_trans_in_msg,
            &navigation.attitude_out_msg => &mut earth_point.sc_att_in_msg,
            &navigation.attitude_out_msg => &mut tracking_error.module.att_nav_in_msg,
            &tracking_error.module.att_guid_out_msg => &mut controller.guid_in_msg,
            &vehicle_config_msg => &mut controller.veh_config_in_msg,
            &spacecraft.state_out => &mut spacecraft_log.input,
            &bennu_ephemeris.planet_out_msgs[0] => &mut asteroid_log.input,
            &tracking_error.module.att_guid_out_msg => &mut guidance_log.input,
            &ephemerides[0] => &mut earth_log.input,
            &ephemerides[1] => &mut sun_log.input,
            &spacecraft.state_out => &mut spacecraft_now,
            &bennu_ephemeris.planet_out_msgs[0] => &mut asteroid_now,
            &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
                CsvSourceConfig::columns(
                    state_columns_with_attitude(),
                ),
            ),
            &bennu_ephemeris.planet_out_msgs[0] => recorder
                .add_source::<PlanetStateMsg>(planet_columns("bennu")),
            &tracking_error.module.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
                CsvSourceConfig::columns(vector_columns("sigma_br", "sigma_BR")),
            ),
        );
        // Upstream's task order (no priorities given): the ephemerides, the
        // spacecraft, the converter, then the flight software; the
        // recorders last.
        schedule! { simulation,
            "planetEphemeris" => &mut bennu_ephemeris, period=TASK_PERIOD_NANOS, priority=100;
            "spiceEarth" => &mut earth_ephemeris, period=TASK_PERIOD_NANOS, priority=99;
            "spiceSun" => &mut sun_ephemeris, period=TASK_PERIOD_NANOS, priority=99;
            "bskSat" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=98;
            "EphemData" => &mut ephemeris_converter, period=TASK_PERIOD_NANOS, priority=97;
            "SimpleNavigation" => &mut navigation, period=TASK_PERIOD_NANOS, priority=96;
            "panelSunPoint" => &mut sun_point, period=TASK_PERIOD_NANOS, priority=95;
            "velocityPointAsteroid" => &mut velocity_point, period=TASK_PERIOD_NANOS, priority=94;
            "sciencePointAsteroid" => &mut science_point, period=TASK_PERIOD_NANOS, priority=93;
            "antennaEarthPoint" => &mut earth_point, period=TASK_PERIOD_NANOS, priority=92;
            "attErrorInertial3D" => &mut tracking_error, period=TASK_PERIOD_NANOS, priority=91;
            "mrpFeedback" => &mut controller, period=TASK_PERIOD_NANOS, priority=90;
            "scRec" => &mut spacecraft_log, period=TASK_PERIOD_NANOS, priority=0;
            "astRec" => &mut asteroid_log, period=TASK_PERIOD_NANOS, priority=0;
            "guidanceRec" => &mut guidance_log, period=TASK_PERIOD_NANOS, priority=0;
            "earthRec" => &mut earth_log, period=TASK_PERIOD_NANOS, priority=0;
            "sunRec" => &mut sun_log, period=TASK_PERIOD_NANOS, priority=0;
        }
        if config.write_csv {
            simulation.add_module("recorder", &mut recorder, TASK_PERIOD_NANOS, 0);
        }

        let mut legs_flown = legs.iter();
        for step in &plan {
            match *step {
                Step::Fly(..) => {
                    let leg = legs_flown.next().expect("one leg per Fly step");
                    run_until(&mut simulation, leg.stop_nanos);
                }
                Step::Impulse(impulse) => {
                    let spacecraft_state = spacecraft_now.read();
                    let asteroid_state = asteroid_now.read();
                    let relative_position =
                        spacecraft_state.position_m - asteroid_state.position_inertial_m;
                    let relative_velocity =
                        spacecraft_state.velocity_mps - asteroid_state.velocity_inertial_mps;
                    let new_velocity =
                        impulse_velocity(impulse, &geometry, relative_position, relative_velocity);
                    hub.set_velocity_mps(new_velocity);
                    impulses.push(AppliedImpulse {
                        impulse,
                        time_nanos: simulation.current_sim_nanos(),
                        velocity_before_mps: relative_velocity,
                        velocity_after_mps: new_velocity,
                    });
                }
            }
        }
    }
    drop(recorder);

    Results {
        geometry,
        spacecraft: spacecraft_log,
        asteroid: asteroid_log,
        guidance: guidance_log,
        earth: earth_log,
        sun: sun_log,
        legs,
        impulses,
        csv_path,
    }
}

/// `r_BN_N_m_*`, `v_BN_N_m_per_s_*` and `sigma_BN_*`.
fn state_columns_with_attitude() -> Vec<(String, String)> {
    vector_columns("position_m", "r_BN_N_m")
        .into_iter()
        .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s"))
        .chain(vector_columns("sigma_bn", "sigma_BN"))
        .collect()
}

fn main() {
    let mut cli = Cli::from_env("scenario_asteroid_arrival [--all]");
    cli.switch("all");
    cli.finish();

    for config in TEST_PARAMETRIZATIONS {
        let results = run(config);
        let last = results.spacecraft.values.len() - 1;
        let (position, velocity) = results.relative_state(last);
        let elements = basilisk_rs::utilities::orbital_motion::state_to_elements(
            results.geometry.mu_m3ps2,
            position,
            velocity,
        );
        println!(
            "wrote {} ({} legs, {} burns; final Bennu-relative orbit: periapsis {:.1} m, \
             apoapsis {:.1} m after {:.2} days)",
            results.csv_path.as_ref().expect("CSV written").display(),
            results.legs.len(),
            results.impulses.len(),
            elements.periapsis_radius,
            elements.apoapsis_radius,
            *results.spacecraft.times_nanos.last().expect("samples") as f64 * 1e-9 / 86_400.0,
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream only runs this scenario (an entry of `test_scenarioTest.py`).
    //! These tests check what it demonstrates: the capture into the `r0`
    //! circle, the Hohmann transfers ending on the `r3 x r2` ellipse, and the
    //! attitude modes reaching their targets.

    use basilisk_rs::utilities::orbital_motion::state_to_elements;
    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;

    use super::*;

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    /// The angle \[rad\] between body `axis_b` and the direction from the
    /// spacecraft to `target_n` at sample `index`.
    fn pointing_error_rad(
        results: &Results,
        index: usize,
        axis_b: Vector3<f64>,
        target_n: Vector3<f64>,
    ) -> f64 {
        let spacecraft = &results.spacecraft.values[index];
        let direction_b = mrp_to_dcm(spacecraft.sigma_bn) * (target_n - spacecraft.position_m);
        direction_b.angle(&axis_b)
    }

    #[test]
    fn the_spacecraft_is_captured_and_transferred_down_to_the_final_ellipse() {
        let results = quiet();
        let geometry = results.geometry;
        assert_eq!(results.legs.len(), 20);
        assert_eq!(results.impulses.len(), 6);
        // One sample per 20 s step, through the last stop time.
        let last_stop = results.legs.last().expect("legs").stop_nanos;
        assert_eq!(
            results.spacecraft.values.len() as u64,
            last_stop / TASK_PERIOD_NANOS + 1
        );

        // The arrival periapsis is the capture radius: the capture burn
        // (within one 20 s step of periapsis) slows the spacecraft down to
        // the circular speed there.
        let capture = results.impulses[0];
        let capture_dv = capture.velocity_after_mps - capture.velocity_before_mps;
        assert!(geometry.capture_dv_mps() < 0.0);
        assert!((capture_dv.norm() + geometry.capture_dv_mps()).abs() < 1e-12);
        assert!(capture_dv.angle(&-capture.velocity_before_mps) < 1e-6);

        // Captured: the radius stays within 0.2 % of r0 until the first
        // transfer burn (the burns fall within one 20 s step of the apsides,
        // and the solar tide is some 1e-5 of Bennu's pull).
        let transfer = results.impulses[1].time_nanos;
        for index in results.samples_in(capture.time_nanos, transfer) {
            let radius = results.relative_state(index).0.norm();
            assert!(
                (radius / geometry.r0_m - 1.0).abs() < 2e-3,
                "radius {radius} m at {} s",
                results.spacecraft.times_nanos[index] as f64 * 1e-9
            );
        }

        // The intermediate and science circles.
        for (circle, radius_m) in [(2, geometry.r1_m), (4, geometry.r2_m)] {
            let start = results.impulses[circle].time_nanos;
            let stop = results.impulses[circle + 1].time_nanos;
            for index in results.samples_in(start, stop) {
                let radius = results.relative_state(index).0.norm();
                assert!((radius / radius_m - 1.0).abs() < 2e-3, "radius {radius} m");
            }
        }

        // The final orbit: periapsis at r3, apoapsis at r2.
        let last = results.spacecraft.values.len() - 1;
        let (position, velocity) = results.relative_state(last);
        let elements = state_to_elements(geometry.mu_m3ps2, position, velocity);
        assert!(
            (elements.periapsis_radius / geometry.r3_m - 1.0).abs() < 2e-3,
            "periapsis {} m",
            elements.periapsis_radius
        );
        assert!(
            (elements.apoapsis_radius / geometry.r2_m - 1.0).abs() < 2e-3,
            "apoapsis {} m",
            elements.apoapsis_radius
        );
    }

    #[test]
    fn each_mode_points_its_axis_at_its_target() {
        let results = quiet();
        // At the end of each leg of 10 000 s or more (with `K = P^2 / I` and
        // `sigma ~ theta / 4` the attitude loop is critically damped and
        // settles in about 8 000 s) the tracking error has settled and the
        // pointed axis is on its target:
        // within 0.5 deg, the science camera lagging most as it tracks Bennu
        // around the final ellipse.
        for leg in &results.legs {
            if leg.stop_nanos - leg.start_nanos < 10_000 * 1_000_000_000 {
                continue;
            }
            let index = results
                .spacecraft
                .times_nanos
                .iter()
                .rposition(|time| *time <= leg.stop_nanos)
                .expect("a sample in the leg");
            let guidance = &results.guidance.values[index];
            assert!(
                guidance.sigma_br.norm() < 2e-3,
                "{:?} leg ending at {} s: |sigma_BR| = {}",
                leg.mode,
                leg.stop_nanos as f64 * 1e-9,
                guidance.sigma_br.norm()
            );
            let asteroid = results.asteroid.values[index].position_inertial_m;
            let error = match leg.mode {
                FlightMode::SensorSciencePointing => Some(pointing_error_rad(
                    &results,
                    index,
                    Vector3::from(CAMERA_AXIS_B),
                    asteroid,
                )),
                FlightMode::PanelSunPointing => Some(pointing_error_rad(
                    &results,
                    index,
                    Vector3::z(),
                    results.sun.values[index].position_bdy_zero_n_m,
                )),
                FlightMode::AntennaEarthPointing => Some(pointing_error_rad(
                    &results,
                    index,
                    Vector3::z(),
                    results.earth.values[index].position_bdy_zero_n_m,
                )),
                _ => None,
            };
            if let Some(error) = error {
                assert!(
                    error < 0.5_f64.to_radians(),
                    "{:?} leg ending at {} s: {} deg off target",
                    leg.mode,
                    leg.stop_nanos as f64 * 1e-9,
                    error.to_degrees()
                );
            }
        }
    }
}
