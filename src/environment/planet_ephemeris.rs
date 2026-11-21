//! Keplerian planet ephemeris: port of upstream Basilisk `planetEphemeris`.
//!
//! Each configured body follows a two-body heliocentric orbit from its
//! classical elements at the reset epoch: the mean anomaly advances at the
//! mean motion `sqrt(μ_sun / a³)`, and the position and velocity follow from
//! the advanced elements ([`crate::utilities::orbital_motion`]). A body with a
//! [`PlanetRotation`] also publishes its orientation: the 3-2-3 Euler angles
//! `(α, π/2 - δ, λ)` of its pole right ascension `α`, declination `δ` and
//! local sidereal angle `λ = λ0 + ω t`, with the rate `[PN]' = [ω̃_NP] [PN]`
//! of a steady spin `ω` about the pole. A body without one publishes no
//! orientation, which consumers read as upstream's identity.
//!
//! Every body publishes one [`PlanetStateMsg`] on `planet_out_msgs`, in the
//! order of `config.bodies`. With `zero_base` set, every state is translated
//! (positions and velocities, not orientations) so that body sits at the
//! origin.
//!
//! Upstream's `PlanetName` and `J2000Current` payload fields have no
//! counterpart in [`PlanetStateMsg`]: the name is the body's `name`, and the
//! state is that of the update that wrote it.

use std::f64::consts::FRAC_PI_2;

use nalgebra::Vector3;

use crate::messages::{Output, PlanetOrientation, PlanetStateMsg};
use crate::utilities::astro_constants::MU_SUN;
use crate::utilities::orbital_motion::{
    ClassicElements, eccentric_to_mean_anomaly, eccentric_to_true_anomaly, elements_to_state,
    mean_to_eccentric_anomaly, true_to_eccentric_anomaly,
};
use crate::utilities::rigid_body_kinematics::{euler323_to_dcm, tilde};
use crate::{Module, SimulationContext};

/// Spin state of a body about its pole, upstream's per-body entries of
/// `rightAscension`, `declination`, `lst0` and `rotRate`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PlanetRotation {
    /// Upstream `rightAscension` \[rad\]: right ascension of the north pole
    /// (positive 3-axis rotation).
    pub right_ascension_rad: f64,
    /// Upstream `declination` \[rad\]: declination of the north pole
    /// (negative 2-axis rotation).
    pub declination_rad: f64,
    /// Upstream `lst0` \[rad\]: local sidereal time angle at the reset epoch
    /// (positive 3-axis rotation).
    pub lst0_rad: f64,
    /// Upstream `rotRate` [rad/s]: rotation rate about the pole.
    pub rotation_rate_radps: f64,
}

/// One propagated body.
#[derive(Clone, Debug)]
pub struct PlanetEphemerisBody {
    /// Upstream `planetNames` entry. Upstream's limit of the SPICE payload
    /// name length does not apply.
    pub name: String,
    /// Upstream `planetElements` entry: heliocentric classical elements at
    /// the reset epoch, semi-major axis in metres. The orbit must be
    /// elliptic.
    pub elements: ClassicElements,
    /// Upstream orientation entries (optional).
    pub rotation: Option<PlanetRotation>,
}

/// User-settable parameters of [`PlanetEphemeris`].
#[derive(Clone, Debug, Default)]
pub struct PlanetEphemerisConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// The propagated bodies; one output each, fixed at construction.
    pub bodies: Vec<PlanetEphemerisBody>,
    /// Upstream `zeroBase`: name of the body, matched without regard to
    /// case, that the published states are translated to. `None` (upstream's
    /// empty string) keeps them heliocentric.
    pub zero_base: Option<String>,
}

impl PlanetEphemerisConfig {
    /// Checks the configuration; upstream's `Reset` raises the `zeroBase`
    /// error.
    pub fn validate(&self) -> Result<(), String> {
        for body in &self.bodies {
            let elements = &body.elements;
            if !(elements.semi_major_axis > 0.0 && (0.0..1.0).contains(&elements.eccentricity)) {
                return Err(format!(
                    "{}: elements must describe an elliptic orbit (a > 0, 0 <= e < 1), got a = {}, e = {}",
                    body.name, elements.semi_major_axis, elements.eccentricity
                ));
            }
        }
        if let Some(zero_base) = &self.zero_base
            && self.zero_base_index().is_none()
        {
            return Err(format!(
                "zero_base '{zero_base}' is not one of the bodies configured in this module instance"
            ));
        }
        Ok(())
    }

    /// Index of the `zero_base` body, if set and configured.
    fn zero_base_index(&self) -> Option<usize> {
        let zero_base = self.zero_base.as_ref()?;
        self.bodies
            .iter()
            .position(|body| body.name.eq_ignore_ascii_case(zero_base))
    }
}

/// Two-body ephemeris of a set of planets: upstream `PlanetEphemeris`.
#[derive(Clone, Debug)]
pub struct PlanetEphemeris {
    /// User-settable parameters.
    pub config: PlanetEphemerisConfig,
    /// Upstream `planetOutMsgs`: one state per body, in the order of
    /// `config.bodies`.
    pub planet_out_msgs: Vec<Output<PlanetStateMsg>>,
    /// Upstream `epochTime` \[s\]: simulation time of the element epoch.
    epoch_time_s: f64,
    /// Upstream `zeroBaseIndex`.
    zero_base_index: Option<usize>,
}

impl PlanetEphemeris {
    /// Builds the module with one output per configured body.
    ///
    /// # Panics
    ///
    /// If `config` fails [`PlanetEphemerisConfig::validate`].
    pub fn new(config: PlanetEphemerisConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid PlanetEphemerisConfig: {message}");
        }
        let planet_out_msgs = config.bodies.iter().map(|_| Output::default()).collect();
        let zero_base_index = config.zero_base_index();
        Self {
            config,
            planet_out_msgs,
            epoch_time_s: 0.0,
            zero_base_index,
        }
    }

    /// Upstream `Reset`: the element epoch is the reset time.
    fn reset_state(&mut self, current_sim_nanos: u64) {
        self.epoch_time_s = current_sim_nanos as f64 * 1.0e-9;
        if let Err(message) = self.config.validate() {
            log::error!("{}: {message}", self.config.name);
        }
        self.zero_base_index = self.config.zero_base_index();
    }

    /// The state of `body` at `time_s`, before any zero-base translation.
    fn body_state(&self, body: &PlanetEphemerisBody, time_s: f64) -> PlanetStateMsg {
        let mu_m3ps2 = MU_SUN * 1000.0_f64.powi(3);
        let elements = &body.elements;
        let e = elements.eccentricity;
        let mean_anomaly_0 =
            eccentric_to_mean_anomaly(true_to_eccentric_anomaly(elements.true_anomaly_rad, e), e);
        let mean_anomaly = mean_anomaly_0
            + (mu_m3ps2 / elements.semi_major_axis.powf(3.0)).sqrt() * (time_s - self.epoch_time_s);
        let current = ClassicElements {
            true_anomaly_rad: eccentric_to_true_anomaly(
                mean_to_eccentric_anomaly(mean_anomaly, e),
                e,
            ),
            ..*elements
        };
        let (position_inertial_m, velocity_inertial_mps) = elements_to_state(mu_m3ps2, &current);

        let orientation = body.rotation.map(|rotation| {
            let lst =
                rotation.lst0_rad + rotation.rotation_rate_radps * (time_s - self.epoch_time_s);
            let dcm_pn = euler323_to_dcm(Vector3::new(
                rotation.right_ascension_rad,
                FRAC_PI_2 - rotation.declination_rad,
                lst,
            ));
            let omega_np_p = Vector3::new(0.0, 0.0, -rotation.rotation_rate_radps);
            PlanetOrientation {
                inertial_to_fixed: dcm_pn,
                inertial_to_fixed_dot: tilde(omega_np_p) * dcm_pn,
            }
        });
        PlanetStateMsg {
            position_inertial_m,
            velocity_inertial_mps,
            orientation,
        }
    }
}

impl Module for PlanetEphemeris {
    fn init(&mut self) {
        self.reset_state(0);
        for output in &self.planet_out_msgs {
            output.write(PlanetStateMsg::default());
        }
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.reset_state(context.current_sim_nanos);
    }

    /// Upstream `UpdateState`: every body's state at the current time,
    /// translated to the zero base once all are known.
    fn update(&mut self, context: &SimulationContext) {
        let time_s = context.current_sim_nanos as f64 * 1.0e-9;
        let mut states: Vec<PlanetStateMsg> = self
            .config
            .bodies
            .iter()
            .map(|body| self.body_state(body, time_s))
            .collect();
        if let Some(base) = self.zero_base_index.and_then(|index| states.get(index)) {
            let (base_position, base_velocity) =
                (base.position_inertial_m, base.velocity_inertial_mps);
            for state in &mut states {
                state.position_inertial_m -= base_position;
                state.velocity_inertial_mps -= base_velocity;
            }
        }
        for (output, state) in self.planet_out_msgs.iter().zip(states) {
            output.write(state);
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `planetEphemeris/_UnitTest/test_planetEphemeris.py`
    //! with its tolerances, reimplementing its Python truth.
    //!
    //! `test_module`'s fourteen parametrizations that set only some of the
    //! orientation vectors expect upstream's length-mismatch `BasiliskError`;
    //! a [`PlanetRotation`] holds all four values of a body, so that
    //! configuration cannot be expressed. The two remaining cases (all set,
    //! none set) are ported. Upstream's `PlanetName` and `J2000Current`
    //! checks have no payload field to check (see the module docs); the
    //! sample times are those of the 0.5 s task.

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::*;
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::astro_constants::{D2R, OMEGA_EARTH, OMEGA_VENUS, SMA_EARTH, SMA_VENUS};

    const HALF_SECOND_NANOS: u64 = 500_000_000;

    fn elements(
        a_m: f64,
        e: f64,
        i_deg: f64,
        raan_deg: f64,
        argp_deg: f64,
        f_deg: f64,
    ) -> ClassicElements {
        ClassicElements {
            semi_major_axis: a_m,
            eccentricity: e,
            inclination_rad: i_deg * D2R,
            raan_rad: raan_deg * D2R,
            arg_periapsis_rad: argp_deg * D2R,
            true_anomaly_rad: f_deg * D2R,
            ..ClassicElements::default()
        }
    }

    /// Runs `module` on upstream's 0.5 s task for 1 s and returns each
    /// body's three recorded states.
    fn run(module: &mut PlanetEphemeris) -> Vec<Vec<PlanetStateMsg>> {
        let mut recorders: Vec<MessageRecorder<PlanetStateMsg>> = module
            .planet_out_msgs
            .iter()
            .map(|_| MessageRecorder::default())
            .collect();
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            for (output, recorder) in module.planet_out_msgs.iter().zip(&mut recorders) {
                sim.connect(output, &mut recorder.input_msg);
            }
            sim.add_module("planetEphemeris", module, HALF_SECOND_NANOS, 10);
            for (index, recorder) in recorders.iter_mut().enumerate() {
                sim.add_module(format!("log{index}"), recorder, HALF_SECOND_NANOS, 0);
            }
            sim.run_for(2 * HALF_SECOND_NANOS);
        }
        recorders
            .into_iter()
            .map(|recorder| recorder.samples)
            .collect()
    }

    /// Upstream `isArrayEqual`: absolute, per component.
    #[track_caller]
    fn assert_matrix_near(actual: &Matrix3<f64>, expected: &Matrix3<f64>, accuracy: f64) {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!((a - e).abs() <= accuracy, "{actual} vs {expected}");
        }
    }

    #[track_caller]
    fn assert_vector_near(actual: &Vector3<f64>, expected: &Vector3<f64>, accuracy: f64) {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!((a - e).abs() <= accuracy, "{actual:?} vs {expected:?}");
        }
    }

    /// Upstream `test_module` with every orientation vector set
    /// (`evalAttitude = 1`) or none (`evalAttitude = 0`).
    #[test]
    fn propagates_positions_velocities_and_orientations() {
        let mu = MU_SUN * 1000.0 * 1000.0 * 1000.0;
        let oe_earth = elements(SMA_EARTH * 1000.0, 0.001, 10.0, 30.0, 20.0, 90.0);
        let oe_venus = elements(SMA_VENUS * 1000.0, 0.001, 5.0, 110.0, 220.0, 180.0);
        let ran = [0.0 * D2R, 272.76 * D2R];
        let dec = [90.0 * D2R, 67.16 * D2R];
        let lst0 = [10.0 * D2R, 30.0 * D2R];
        let rate = [OMEGA_EARTH, OMEGA_VENUS];

        for eval_attitude in [true, false] {
            let bodies = ["earth", "venus"]
                .into_iter()
                .zip([oe_earth, oe_venus])
                .enumerate()
                .map(|(c, (name, elements))| PlanetEphemerisBody {
                    name: name.to_string(),
                    elements,
                    rotation: eval_attitude.then_some(PlanetRotation {
                        right_ascension_rad: ran[c],
                        declination_rad: dec[c],
                        lst0_rad: lst0[c],
                        rotation_rate_radps: rate[c],
                    }),
                })
                .collect();
            let mut module = PlanetEphemeris::new(PlanetEphemerisConfig {
                name: "planetEphemeris".to_string(),
                bodies,
                zero_base: None,
            });
            let logs = run(&mut module);

            let accuracy = 1e-3;
            let time_true = [0.0, 0.5, 1.0];
            for (c, (log, oe)) in logs.iter().zip([oe_earth, oe_venus]).enumerate() {
                assert_eq!(log.len(), time_true.len());
                let m0 = eccentric_to_mean_anomaly(
                    true_to_eccentric_anomaly(oe.true_anomaly_rad, oe.eccentricity),
                    oe.eccentricity,
                );
                for (sample, time) in log.iter().zip(time_true) {
                    let mt = m0 + (mu / oe.semi_major_axis.powi(3)).sqrt() * time;
                    let et = mean_to_eccentric_anomaly(mt, oe.eccentricity);
                    let oe_t = ClassicElements {
                        true_anomaly_rad: eccentric_to_true_anomaly(et, oe.eccentricity),
                        ..oe
                    };
                    let (r_true, v_true) = elements_to_state(mu, &oe_t);
                    assert_vector_near(&sample.position_inertial_m, &r_true, accuracy);
                    assert_vector_near(&sample.velocity_inertial_mps, &v_true, accuracy);

                    if eval_attitude {
                        let lst = lst0[c] + rate[c] * time;
                        let dcm = euler323_to_dcm(Vector3::new(ran[c], FRAC_PI_2 - dec[c], lst));
                        let dcm_rate = tilde(Vector3::new(0.0, 0.0, -rate[c])) * dcm;
                        let orientation =
                            sample.orientation.as_ref().expect("orientation computed");
                        assert_matrix_near(&orientation.inertial_to_fixed, &dcm, accuracy);
                        assert_matrix_near(&orientation.inertial_to_fixed_dot, &dcm_rate, 1e-10);
                    } else {
                        // Upstream's identity `J20002Pfix`, zero rate and
                        // `computeOrient = 0`.
                        assert!(sample.orientation.is_none());
                    }
                }
            }
        }
    }

    /// Upstream `_configure_zero_base_test_module`.
    fn zero_base_config(zero_base: Option<&str>) -> PlanetEphemerisConfig {
        let sma_m = SMA_EARTH * 1000.0;
        PlanetEphemerisConfig {
            name: "planetEphemeris".to_string(),
            bodies: vec![
                PlanetEphemerisBody {
                    name: "earth".to_string(),
                    elements: elements(sma_m, 0.01, 10.0, 30.0, 20.0, 90.0),
                    rotation: Some(PlanetRotation {
                        right_ascension_rad: 0.0,
                        declination_rad: 90.0 * D2R,
                        lst0_rad: 0.0,
                        rotation_rate_radps: 1.0e-5,
                    }),
                },
                PlanetEphemerisBody {
                    name: "moon".to_string(),
                    elements: elements(1.01 * sma_m, 0.02, 5.0, 110.0, 220.0, 180.0),
                    rotation: Some(PlanetRotation {
                        right_ascension_rad: 10.0 * D2R,
                        declination_rad: 80.0 * D2R,
                        lst0_rad: 30.0 * D2R,
                        rotation_rate_radps: 2.0e-5,
                    }),
                },
            ],
            zero_base: zero_base.map(str::to_string),
        }
    }

    /// Upstream `test_zero_base_translation`: the zero base translates
    /// positions and velocities only; the name matches without regard to
    /// case.
    #[test]
    fn zero_base_translates_positions_and_velocities_only() {
        assert!(zero_base_config(None).zero_base.is_none());
        let heliocentric = run(&mut PlanetEphemeris::new(zero_base_config(None)));
        let relative = run(&mut PlanetEphemeris::new(zero_base_config(Some("EaRtH"))));

        for (body_helio, body_relative) in heliocentric.iter().zip(&relative) {
            for ((helio, rel), base) in body_helio.iter().zip(body_relative).zip(&heliocentric[0]) {
                assert_vector_near(
                    &rel.position_inertial_m,
                    &(helio.position_inertial_m - base.position_inertial_m),
                    1.0e-6,
                );
                assert_vector_near(
                    &rel.velocity_inertial_mps,
                    &(helio.velocity_inertial_mps - base.velocity_inertial_mps),
                    1.0e-10,
                );
                let (helio_o, rel_o) = (
                    helio.orientation.as_ref().expect("orientation"),
                    rel.orientation.as_ref().expect("orientation"),
                );
                assert_matrix_near(
                    &rel_o.inertial_to_fixed,
                    &helio_o.inertial_to_fixed,
                    1.0e-14,
                );
                assert_matrix_near(
                    &rel_o.inertial_to_fixed_dot,
                    &helio_o.inertial_to_fixed_dot,
                    1.0e-14,
                );
            }
        }
        // The base body itself sits at the origin.
        assert_eq!(relative[0][2].position_inertial_m, Vector3::zeros());
    }

    /// Upstream `test_zero_base_requires_local_body`.
    #[test]
    #[should_panic(expected = "not one of the bodies configured")]
    fn zero_base_must_be_a_configured_body() {
        PlanetEphemeris::new(zero_base_config(Some("mars")));
    }

    /// Not an upstream case: the elements are taken at the reset epoch, so a
    /// reset restarts the propagation.
    #[test]
    fn reset_moves_the_element_epoch() {
        let mut module = PlanetEphemeris::new(zero_base_config(None));
        module.init();
        let at = |nanos| SimulationContext {
            current_sim_nanos: nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        };
        module.update(&at(0));
        let at_epoch = module.planet_out_msgs[0].read();
        module.reset(&at(3_600_000_000_000));
        module.update(&at(3_600_000_000_000));
        let after_reset = module.planet_out_msgs[0].read();
        assert_eq!(
            after_reset.position_inertial_m,
            at_epoch.position_inertial_m
        );
        assert_eq!(
            after_reset.orientation.unwrap().inertial_to_fixed,
            at_epoch.orientation.unwrap().inertial_to_fixed
        );
    }

    #[test]
    #[should_panic(expected = "elliptic")]
    fn rejects_a_non_elliptic_orbit() {
        let mut config = zero_base_config(None);
        config.bodies[0].elements.eccentricity = 1.2;
        PlanetEphemeris::new(config);
    }
}
