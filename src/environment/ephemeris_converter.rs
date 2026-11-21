//! Planet state to flight-software ephemeris conversion: port of upstream
//! Basilisk `ephemerisConverter`.
//!
//! Turns the [`PlanetStateMsg`] (upstream `SpicePlanetStateMsg`) that the
//! simulation ephemeris models publish
//! ([`super::anise_planet_ephemeris::AnisePlanetEphemeris`],
//! [`super::anise_sun_ephemeris::AniseSunEphemeris`],
//! [`super::planet_ephemeris::PlanetEphemeris`]) into the [`EphemerisMsg`]
//! that flight software reads (`celestialTwoBodyPoint`, `hillPoint`,
//! `velocityPoint`, `sunlineEphem`, ...). Position and velocity are copied;
//! the body attitude `sigma_BN` is the MRP of the planet's inertial-to-fixed
//! DCM `[BN]`, and its angular velocity the vector of
//! `[ω̃_BN_B] = -[BN]' [BN]ᵀ`. A body without an orientation has upstream's
//! identity attitude and zero rate.
//!
//! Each configured body has one input on `planet_in_msgs` and one output on
//! `ephem_out_msgs`, at the same index.
//!
//! Upstream stamps `timeTag` with the input header's `timeWritten`. Stand-in:
//! connected ⇒ written at the current update (as `css_inputs.rs`), so the time
//! tag is the current simulation time; an unconnected input, like upstream's
//! never-written message, gives zero.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{EphemerisMsg, Input, Output, PlanetStateMsg};
use crate::utilities::rigid_body_kinematics::dcm_to_mrp;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`EphemerisConverter`].
#[derive(Clone, Debug, Default)]
pub struct EphemerisConverterConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// One entry per converted body, upstream's `addSpiceInputMsg` calls;
    /// the names only label the ports, fixed at construction.
    pub body_names: Vec<String>,
}

impl EphemerisConverterConfig {
    /// Checks the configuration; upstream's `Reset` raises on no bodies.
    pub fn validate(&self) -> Result<(), String> {
        if self.body_names.is_empty() {
            return Err("body_names is empty".to_string());
        }
        Ok(())
    }
}

/// Planet states to ephemerides: upstream `EphemerisConverter`.
#[derive(Clone, Debug)]
pub struct EphemerisConverter {
    /// User-settable parameters.
    pub config: EphemerisConverterConfig,
    /// Upstream `spiceInMsgs`: one planet state per body, in the order of
    /// `config.body_names`.
    pub planet_in_msgs: Vec<Input<PlanetStateMsg>>,
    /// Upstream `ephemOutMsgs`: the ephemeris of each body, at the index of
    /// its input.
    pub ephem_out_msgs: Vec<Output<EphemerisMsg>>,
}

impl EphemerisConverter {
    /// Builds the module with one unconnected input and one output per body.
    ///
    /// # Panics
    ///
    /// If `config` fails [`EphemerisConverterConfig::validate`].
    pub fn new(config: EphemerisConverterConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid EphemerisConverterConfig: {message}");
        }
        let body_count = config.body_names.len();
        Self {
            config,
            planet_in_msgs: (0..body_count).map(|_| Input::default()).collect(),
            ephem_out_msgs: (0..body_count).map(|_| Output::default()).collect(),
        }
    }

    /// Index of the ports of the body called `name`.
    pub fn body_index(&self, name: &str) -> Option<usize> {
        self.config.body_names.iter().position(|body| body == name)
    }
}

/// Upstream `convertEphemData` for one body, with time tag `time_tag_s`.
fn convert(planet: &PlanetStateMsg, time_tag_s: f64) -> EphemerisMsg {
    let (dcm_bn, dcm_bn_dot) = planet.orientation.as_ref().map_or(
        (Matrix3::identity(), Matrix3::zeros()),
        |orientation| {
            (
                orientation.inertial_to_fixed,
                orientation.inertial_to_fixed_dot,
            )
        },
    );
    let omega_tilde_bn_b = -dcm_bn_dot * dcm_bn.transpose();
    EphemerisMsg {
        position_bdy_zero_n_m: planet.position_inertial_m,
        velocity_bdy_zero_n_mps: planet.velocity_inertial_mps,
        sigma_bn: dcm_to_mrp(dcm_bn),
        omega_bn_b_radps: Vector3::new(
            omega_tilde_bn_b[(2, 1)],
            omega_tilde_bn_b[(0, 2)],
            omega_tilde_bn_b[(1, 0)],
        ),
        time_tag_s,
    }
}

impl Module for EphemerisConverter {
    fn init(&mut self) {
        for output in &self.ephem_out_msgs {
            output.write(EphemerisMsg::default());
        }
    }

    fn reset(&mut self, _context: &SimulationContext) {}

    fn update(&mut self, context: &SimulationContext) {
        let now_s = context.current_sim_nanos as f64 * 1.0e-9;
        for (input, output) in self.planet_in_msgs.iter().zip(&self.ephem_out_msgs) {
            let time_tag_s = if input.is_connected() { now_s } else { 0.0 };
            output.write(convert(&input.read(), time_tag_s));
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `ephemerisConverter/_UnitTest/test_ephemconvert.py`
    //! with its tolerances, with the ANISE planet ephemeris in place of
    //! Basilisk's SPICE interface: Earth and the Sun carry their IAU
    //! orientations, and Mars' barycenter, which has none (SPICE's identity),
    //! carries no orientation. An end-to-end case feeds the converter from
    //! the ANISE models into a flight-software module.

    use std::path::PathBuf;

    use anise::constants::frames::{
        EARTH_J2000, IAU_EARTH_FRAME, MARS_BARYCENTER_J2000, MOON_J2000, SSB_J2000, SUN_J2000,
    };
    use anise::frames::Frame;
    use hifitime::{Epoch, TimeScale};

    use super::*;
    use crate::environment::anise_planet_ephemeris::{
        AnisePlanetEphemeris, AnisePlanetEphemerisConfig,
    };
    use crate::environment::anise_sun_ephemeris::{AniseSunEphemeris, AniseSunEphemerisConfig};
    use crate::fsw_algorithms::att_guidance::celestial_two_body_point::{
        CelestialTwoBodyPoint, CelestialTwoBodyPointConfig,
    };
    use crate::messages::{AttitudeReferenceMsg, NavigationTranslationMsg, PlanetOrientation};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::rigid_body_kinematics::{euler3, mrp_to_dcm, tilde};

    /// The IAU Sun body-fixed frame (NAIF 10), for which ANISE has no
    /// constant.
    const IAU_SUN_FRAME: Frame = Frame::new(10, 10);

    fn repo_path(relative: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(relative)
    }

    fn anise_body(
        name: &str,
        source: Frame,
        observer: Frame,
        fixed: Option<Frame>,
    ) -> AnisePlanetEphemeris {
        AnisePlanetEphemeris::new(AnisePlanetEphemerisConfig {
            name: name.to_string(),
            spk_path: repo_path("assets/anise/de440s.bsp"),
            additional_kernel_paths: vec![repo_path("assets/anise/pck11.pca")],
            source_frame: source,
            observer_frame: observer,
            fixed_frame: fixed,
        })
    }

    fn converter(names: &[&str]) -> EphemerisConverter {
        EphemerisConverter::new(EphemerisConverterConfig {
            name: "EphemData".to_string(),
            body_names: names.iter().map(|name| name.to_string()).collect(),
        })
    }

    /// `unitTestSupport.isArrayEqualRelative`: relative per component, and
    /// exact where the truth is zero.
    #[track_caller]
    fn assert_relative(actual: &Vector3<f64>, truth: &Vector3<f64>, accuracy: f64, what: &str) {
        for (a, t) in actual.iter().zip(truth.iter()) {
            if *t == 0.0 {
                assert_eq!(*a, 0.0, "{what}: {actual:?} vs {truth:?}");
            } else {
                assert!(
                    ((a - t) / t).abs() <= accuracy,
                    "{what}: {actual:?} vs {truth:?}"
                );
            }
        }
    }

    /// Upstream `test_ephemConvert`: positions and velocities are copied,
    /// and the attitude and rate follow from the planet DCM and its rate.
    #[test]
    fn converts_planet_states_to_ephemerides() {
        let mut bodies = [
            anise_body("earth", EARTH_J2000, SSB_J2000, Some(IAU_EARTH_FRAME)),
            anise_body("mars barycenter", MARS_BARYCENTER_J2000, SSB_J2000, None),
            anise_body("sun", SUN_J2000, SSB_J2000, Some(IAU_SUN_FRAME)),
        ];
        let mut ephem = converter(&["earth", "mars barycenter", "sun"]);
        let mut planet_logs: Vec<MessageRecorder<PlanetStateMsg>> =
            (0..3).map(|_| MessageRecorder::default()).collect();
        let mut ephem_logs: Vec<MessageRecorder<EphemerisMsg>> =
            (0..3).map(|_| MessageRecorder::default()).collect();

        // Upstream: 30 s sampled at 600 points.
        let simulation_nanos = 30_000_000_000_u64;
        let sampling_nanos = simulation_nanos / 599;
        {
            let epoch = Epoch::from_gregorian(2015, 2, 10, 0, 0, 0, 0, TimeScale::TDB);
            let mut sim = Simulation::new(epoch, false);
            for (c, body) in bodies.iter().enumerate() {
                sim.connect(&body.output_planet_msg, &mut ephem.planet_in_msgs[c]);
                sim.connect(&body.output_planet_msg, &mut planet_logs[c].input_msg);
                sim.connect(&ephem.ephem_out_msgs[c], &mut ephem_logs[c].input_msg);
            }
            for (c, body) in bodies.iter_mut().enumerate() {
                sim.add_module(format!("planet{c}"), body, sampling_nanos, 20);
            }
            sim.add_module("EphemData", &mut ephem, sampling_nanos, 10);
            for (c, (planet_log, ephem_log)) in
                planet_logs.iter_mut().zip(&mut ephem_logs).enumerate()
            {
                sim.add_module(format!("planetLog{c}"), planet_log, sampling_nanos, 0);
                sim.add_module(format!("ephemLog{c}"), ephem_log, sampling_nanos, 0);
            }
            sim.run_for(simulation_nanos);
        }

        let accuracy = 1e-12;
        for (planet_log, ephem_log) in planet_logs.iter().zip(&ephem_logs) {
            assert_eq!(ephem_log.samples.len(), 600);
            for (step, (planet, ephemeris)) in planet_log
                .samples
                .iter()
                .zip(&ephem_log.samples)
                .enumerate()
            {
                let orientation = planet
                    .orientation
                    .clone()
                    .unwrap_or_else(PlanetOrientation::identity);
                let dcm_pn = orientation.inertial_to_fixed;
                let omega_tilde = -orientation.inertial_to_fixed_dot * dcm_pn.transpose();
                let omega_truth = Vector3::new(
                    omega_tilde[(2, 1)],
                    omega_tilde[(0, 2)],
                    omega_tilde[(1, 0)],
                );
                assert_relative(
                    &ephemeris.position_bdy_zero_n_m,
                    &planet.position_inertial_m,
                    accuracy,
                    "position",
                );
                assert_relative(
                    &ephemeris.velocity_bdy_zero_n_mps,
                    &planet.velocity_inertial_mps,
                    accuracy,
                    "velocity",
                );
                assert_relative(
                    &ephemeris.sigma_bn,
                    &dcm_to_mrp(dcm_pn),
                    accuracy,
                    "attitude",
                );
                assert!((ephemeris.omega_bn_b_radps - omega_truth).abs().max() <= accuracy);
                assert_eq!(
                    ephemeris.time_tag_s,
                    (step as u64 * sampling_nanos) as f64 * 1.0e-9
                );
            }
        }
        // Earth and the Sun spin at their IAU sidereal rates about their
        // poles; Mars' barycenter, without an orientation, does not rotate.
        let earth_rate = ephem_logs[0].samples[0].omega_bn_b_radps;
        assert!(
            (earth_rate.norm() - 7.292_115e-5).abs() < 1e-9,
            "{earth_rate:?}"
        );
        assert!(earth_rate.z / earth_rate.norm() > 0.999_999);
        assert_eq!(ephem_logs[1].samples[0].sigma_bn, Vector3::zeros());
        let sun_rate = ephem_logs[2].samples[0].omega_bn_b_radps;
        assert!((sun_rate.norm() - 2.865e-6).abs() < 1e-9, "{sun_rate:?}");
    }

    /// Not an upstream case: a known spinning orientation converts to its
    /// MRP and body-frame rate, and an unconnected input to a zero message
    /// with a zero time tag.
    #[test]
    fn converts_a_known_spin_and_leaves_unconnected_bodies_zero() {
        let angle = 0.4;
        let rate = 1.0e-3;
        let dcm_pn = euler3(angle);
        let planet = Output::new(PlanetStateMsg {
            position_inertial_m: Vector3::new(1.0, 2.0, 3.0),
            velocity_inertial_mps: Vector3::new(-4.0, 5.0, -6.0),
            orientation: Some(PlanetOrientation {
                inertial_to_fixed: dcm_pn,
                inertial_to_fixed_dot: -tilde(Vector3::new(0.0, 0.0, rate)) * dcm_pn,
            }),
        });
        let mut ephem = converter(&["spinning", "unconnected"]);
        planet.connect_to(&mut ephem.planet_in_msgs[0]);
        ephem.init();
        ephem.update(&SimulationContext {
            current_sim_nanos: 2_500_000_000,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });

        let spinning = ephem.ephem_out_msgs[0].read();
        assert_eq!(spinning.position_bdy_zero_n_m, Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(
            spinning.velocity_bdy_zero_n_mps,
            Vector3::new(-4.0, 5.0, -6.0)
        );
        assert!((spinning.sigma_bn - Vector3::new(0.0, 0.0, (angle / 4.0).tan())).norm() < 1e-15);
        assert!((spinning.omega_bn_b_radps - Vector3::new(0.0, 0.0, rate)).norm() < 1e-18);
        assert_eq!(spinning.time_tag_s, 2.5);
        assert_eq!(ephem.body_index("unconnected"), Some(1));
        assert_eq!(ephem.ephem_out_msgs[1].read(), EphemerisMsg::default());
    }

    #[test]
    #[should_panic(expected = "body_names is empty")]
    fn rejects_an_empty_body_list() {
        converter(&[]);
    }

    /// End to end: the ANISE Moon and Sun ephemerides feed the converter,
    /// which feeds `celestialTwoBodyPoint`; the reference points its first
    /// axis at the Moon, with the Sun in its 1-2 plane.
    #[test]
    fn anise_ephemerides_drive_celestial_two_body_pointing() {
        let mut moon = anise_body("moon", MOON_J2000, EARTH_J2000, None);
        let mut sun = AniseSunEphemeris::new(AniseSunEphemerisConfig {
            name: "sun".to_string(),
            spk_path: repo_path("assets/anise/de440s.bsp"),
        });
        let mut ephem = converter(&["moon", "sun"]);
        let nav = Output::new(NavigationTranslationMsg {
            position_bn_n_m: Vector3::new(7.0e6, 0.0, 0.0),
            velocity_bn_n_mps: Vector3::new(0.0, 7.5e3, 0.0),
            ..NavigationTranslationMsg::default()
        });
        let mut pointing = CelestialTwoBodyPoint::new(CelestialTwoBodyPointConfig::default());
        let mut reference_log = MessageRecorder::<AttitudeReferenceMsg>::default();
        let (moon_index, sun_index) = (
            ephem.body_index("moon").unwrap(),
            ephem.body_index("sun").unwrap(),
        );
        let second = 1_000_000_000;
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.connect(
                &moon.output_planet_msg,
                &mut ephem.planet_in_msgs[moon_index],
            );
            sim.connect(&sun.output_planet_msg, &mut ephem.planet_in_msgs[sun_index]);
            sim.connect(
                &ephem.ephem_out_msgs[moon_index],
                &mut pointing.cel_body_in_msg,
            );
            sim.connect(
                &ephem.ephem_out_msgs[sun_index],
                &mut pointing.sec_cel_body_in_msg,
            );
            sim.connect(&nav, &mut pointing.trans_nav_in_msg);
            sim.connect(&pointing.att_ref_out_msg, &mut reference_log.input_msg);
            sim.add_module("moon", &mut moon, second, 30);
            sim.add_module("sun", &mut sun, second, 30);
            sim.add_module("ephemerisConverter", &mut ephem, second, 20);
            sim.add_module("celestialTwoBodyPoint", &mut pointing, second, 10);
            sim.add_module("log", &mut reference_log, second, 0);
            sim.run_for(10 * second);
        }

        let moon_state = moon.output_planet_msg.read();
        let sun_state = sun.output_planet_msg.read();
        let moon_ephemeris = ephem.ephem_out_msgs[moon_index].read();
        assert_eq!(
            moon_ephemeris.position_bdy_zero_n_m,
            moon_state.position_inertial_m
        );
        assert_eq!(
            moon_ephemeris.velocity_bdy_zero_n_mps,
            moon_state.velocity_inertial_mps
        );
        assert_eq!(moon_ephemeris.time_tag_s, 10.0);
        assert!(moon_state.position_inertial_m.norm() > 3.0e8);

        assert_eq!(reference_log.samples.len(), 11);
        let reference = reference_log.samples.last().unwrap();
        let dcm_rn = mrp_to_dcm(reference.sigma_rn);
        let r_sc = nav.read().position_bn_n_m;
        let to_moon = (moon_state.position_inertial_m - r_sc).normalize();
        let to_sun = (sun_state.position_inertial_m - r_sc).normalize();
        assert!((dcm_rn.row(0).transpose() - to_moon).norm() < 1e-12);
        let plane_normal = to_moon.cross(&to_sun).normalize();
        assert!((dcm_rn.row(2).transpose() - plane_normal).norm() < 1e-12);
        assert!(dcm_rn.row(1).transpose().dot(&to_sun) > 0.0);
    }
}
