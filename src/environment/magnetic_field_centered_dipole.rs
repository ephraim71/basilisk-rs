//! Centered dipole magnetic field: port of upstream Basilisk
//! `magneticFieldCenteredDipole` and the parts of `magneticFieldBase` it
//! uses.
//!
//! The field of a dipole at the planet centre, from the first three IGRF
//! Gauss coefficients (Markley & Crassidis, p. 405):
//! `B_P = (R/r)^3 (3 r̂ (r̂·d) - d)` with `d = [g11, h11, g10]` in
//! planet-fixed components, rotated into inertial components. Outside the
//! optional reach limits `env_min_reach_m < r < env_max_reach_m` the field is
//! zero.
//!
//! It follows the [`super::wmm_field::WmmField`] and
//! [`super::igrf_field::IgrfField`] ports rather than upstream's
//! multi-spacecraft base class: one spacecraft per instance, with
//! `input_state_msg`, the optional `input_planet_msg` and
//! `output_magnetic_field_msg`. With no planet message, or one whose
//! `orientation` is `None`, the planet sits at the inertial origin with an
//! identity orientation. A spacecraft at the planet centre gets a zero field
//! instead of upstream's non-finite one, as `WmmField` does.
//!
//! Not ported: `magneticFieldBase`'s opt-in extrapolation of the spacecraft
//! state to the step midpoint, which needs message write times that
//! basilisk-rs ports do not carry, and its epoch input, which a time-invariant
//! dipole does not read.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{Input, MagneticFieldMsg, Output, PlanetStateMsg, SpacecraftStateMsg};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`MagneticFieldCenteredDipole`]. `Default` is
/// upstream's constructor state, a zero field.
#[derive(Clone, Debug)]
pub struct MagneticFieldCenteredDipoleConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `g10` \[T\]: IGRF coefficient `g_1^0`.
    pub g10_t: f64,
    /// Upstream `g11` \[T\]: IGRF coefficient `g_1^1`.
    pub g11_t: f64,
    /// Upstream `h11` \[T\]: IGRF coefficient `h_1^1`.
    pub h11_t: f64,
    /// Upstream `planetRadius` \[m\]: reference radius of the coefficients.
    pub planet_radius_m: f64,
    /// Upstream `envMinReach` \[m\]: the field is zero at or below this
    /// orbit radius. `None` is upstream's `-1`, off.
    pub env_min_reach_m: Option<f64>,
    /// Upstream `envMaxReach` \[m\]: the field is zero at or beyond this
    /// orbit radius. `None` is upstream's `-1`, off.
    pub env_max_reach_m: Option<f64>,
}

impl Default for MagneticFieldCenteredDipoleConfig {
    fn default() -> Self {
        Self {
            name: "magneticFieldCenteredDipole".to_string(),
            g10_t: 0.0,
            g11_t: 0.0,
            h11_t: 0.0,
            planet_radius_m: 0.0,
            env_min_reach_m: None,
            env_max_reach_m: None,
        }
    }
}

/// Centered dipole field at one spacecraft, published in inertial
/// components, in tesla: upstream `MagneticFieldCenteredDipole`.
#[derive(Clone, Debug)]
pub struct MagneticFieldCenteredDipole {
    /// User-settable parameters.
    pub config: MagneticFieldCenteredDipoleConfig,
    /// Upstream `scStateInMsgs` entry: the spacecraft state.
    pub input_state_msg: Input<SpacecraftStateMsg>,
    /// Upstream `planetPosInMsg` (optional): planet state and orientation.
    pub input_planet_msg: Input<PlanetStateMsg>,
    /// Upstream `envOutMsgs` entry: the field, inertial components.
    pub output_magnetic_field_msg: Output<MagneticFieldMsg>,
}

impl MagneticFieldCenteredDipole {
    pub fn new(config: MagneticFieldCenteredDipoleConfig) -> Self {
        Self {
            config,
            input_state_msg: Input::default(),
            input_planet_msg: Input::default(),
            output_magnetic_field_msg: Output::default(),
        }
    }

    /// Upstream `updateRelativePos` and `evaluateMagneticFieldModel`.
    fn compute_magnetic_field(&self, state: &SpacecraftStateMsg) -> Vector3<f64> {
        let planet = self.input_planet_msg.read();
        let dcm_pn = planet
            .orientation
            .as_ref()
            .map_or_else(Matrix3::identity, |orientation| {
                orientation.inertial_to_fixed
            });
        let r_bp_n = state.position_m - planet.position_inertial_m;
        let r_bp_p = dcm_pn * r_bp_n;
        let orbit_radius_m = r_bp_n.norm();

        let above_min = self
            .config
            .env_min_reach_m
            .is_none_or(|reach| orbit_radius_m > reach);
        let below_max = self
            .config
            .env_max_reach_m
            .is_none_or(|reach| orbit_radius_m < reach);
        let evaluated = above_min && below_max && orbit_radius_m > f64::EPSILON;
        if !evaluated {
            return Vector3::zeros();
        }

        let r_hat_p = r_bp_p.normalize();
        let dipole = Vector3::new(self.config.g11_t, self.config.h11_t, self.config.g10_t);
        let field_p = (self.config.planet_radius_m / orbit_radius_m).powi(3)
            * (3.0 * r_hat_p * r_hat_p.dot(&dipole) - dipole);
        dcm_pn.transpose() * field_p
    }
}

impl Module for MagneticFieldCenteredDipole {
    fn init(&mut self) {
        self.output_magnetic_field_msg
            .write(MagneticFieldMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.input_state_msg.read();
        self.output_magnetic_field_msg.write(MagneticFieldMsg {
            magnetic_field_inertial_t: self.compute_magnetic_field(&state),
        });
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream
    //! `magneticFieldCenteredDipole/_UnitTest/test_magneticFieldCenteredDipole.py`
    //! with its tolerance, reimplementing its Python `centeredDipole` truth.
    //! Upstream's two spacecraft on one module are two module instances here.
    //!
    //! Its "earth" case reads the coefficients from
    //! `simSetPlanetEnvironment.centeredDipoleMagField`, a Python helper
    //! outside the sparse upstream checkout; the IGRF-13 2020 dipole terms
    //! stand in, and the truth reads them back from the configuration as
    //! upstream's does. `test_magneticFieldStepLag.py` is not ported: it
    //! tests the step-midpoint extrapolation, which is not ported (see the
    //! module docs).

    use hifitime::Epoch;

    use super::*;
    use crate::messages::PlanetOrientation;
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::astro_constants::{D2R, REQ_EARTH};
    use crate::utilities::orbital_motion::{ClassicElements, elements_to_state};

    /// IGRF-13 2020 dipole coefficients \[T\] and reference radius \[m\].
    fn earth_dipole() -> MagneticFieldCenteredDipoleConfig {
        MagneticFieldCenteredDipoleConfig {
            name: "CenteredDipole".to_string(),
            g10_t: -29404.8e-9,
            g11_t: -1450.9e-9,
            h11_t: 4652.5e-9,
            planet_radius_m: 6371.2e3,
            ..MagneticFieldCenteredDipoleConfig::default()
        }
    }

    /// Upstream `centeredDipole`: the truth field at `pos_n` relative to the
    /// planet.
    fn truth(
        pos_n: Vector3<f64>,
        dipole: Vector3<f64>,
        planet_radius: f64,
        dcm_pn: Matrix3<f64>,
        min_reach: f64,
        max_reach: f64,
    ) -> Vector3<f64> {
        let radius = pos_n.norm();
        let r_hat_p = dcm_pn * pos_n / radius;
        let field_p =
            (planet_radius / radius).powi(3) * (3.0 * r_hat_p * r_hat_p.dot(&dipole) - dipole);
        if radius < min_reach || (radius > max_reach && max_reach > 0.0) {
            return Vector3::zeros();
        }
        dcm_pn.transpose() * field_p
    }

    /// `unitTestSupport.compareArrayRelative`: each component within
    /// `accuracy` of the truth, relative to the truth's magnitude.
    #[track_caller]
    fn assert_relative(actual: &Vector3<f64>, expected: &Vector3<f64>, accuracy: f64) {
        for (a, e) in actual.iter().zip(expected.iter()) {
            let error = if *e == 0.0 {
                (a - e).abs()
            } else {
                ((a - e) / e).abs()
            };
            assert!(error <= accuracy, "{actual:?} vs {expected:?}");
        }
    }

    /// Upstream `test_module` over every `useDefault`, `useMinReach`,
    /// `useMaxReach` and `usePlanetEphemeris` combination.
    #[test]
    fn matches_upstream_centered_dipole_truth() {
        let mu = 0.3986004415e15;
        let mut elements = ClassicElements {
            semi_major_axis: 6571.0 * 1000.0,
            eccentricity: 0.0,
            inclination_rad: 45.0 * D2R,
            raan_rad: 30.0 * D2R,
            arg_periapsis_rad: 120.0 * D2R,
            true_anomaly_rad: 0.0,
            ..ClassicElements::default()
        };
        let (r0_n, _) = elements_to_state(mu, &elements);
        elements.semi_major_axis = 6600.0 * 1000.0;
        let (r1_n, _) = elements_to_state(mu, &elements);

        for use_default in [true, false] {
            for use_min_reach in [true, false] {
                for use_max_reach in [true, false] {
                    for use_planet_ephemeris in [true, false] {
                        let mut config = if use_default {
                            MagneticFieldCenteredDipoleConfig::default()
                        } else {
                            earth_dipole()
                        };
                        let mut min_reach = -1.0;
                        if use_min_reach {
                            min_reach = (REQ_EARTH + 300.0) * 1000.0;
                            config.env_min_reach_m = Some(min_reach);
                        }
                        let mut max_reach = -1.0;
                        if use_max_reach {
                            // Upstream's value, in metres as written.
                            max_reach = REQ_EARTH + 100.0;
                            config.env_max_reach_m = Some(max_reach);
                        }
                        let mut planet_position = Vector3::zeros();
                        let mut dcm_pn = Matrix3::identity();
                        let planet_output = Output::new(PlanetStateMsg::default());
                        if use_planet_ephemeris {
                            planet_position = Vector3::new(1000.0, 2000.0, -1000.0);
                            dcm_pn = Matrix3::new(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
                            planet_output.write(PlanetStateMsg {
                                position_inertial_m: planet_position,
                                orientation: Some(PlanetOrientation {
                                    inertial_to_fixed: dcm_pn,
                                    inertial_to_fixed_dot: Matrix3::zeros(),
                                }),
                                ..PlanetStateMsg::default()
                            });
                        }

                        let dipole = Vector3::new(config.g11_t, config.h11_t, config.g10_t);
                        let planet_radius = config.planet_radius_m;
                        for r_n in [r0_n, r1_n] {
                            let state_output = Output::new(SpacecraftStateMsg {
                                position_m: r_n + planet_position,
                                ..SpacecraftStateMsg::default()
                            });
                            let mut field = MagneticFieldCenteredDipole::new(config.clone());
                            let mut recorder = MessageRecorder::<MagneticFieldMsg>::default();
                            {
                                let mut sim = Simulation::new(
                                    Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
                                    false,
                                );
                                sim.connect(&state_output, &mut field.input_state_msg);
                                if use_planet_ephemeris {
                                    sim.connect(&planet_output, &mut field.input_planet_msg);
                                }
                                sim.connect(
                                    &field.output_magnetic_field_msg,
                                    &mut recorder.input_msg,
                                );
                                sim.add_module("dipole", &mut field, 500_000_000, 0);
                                sim.add_module("log", &mut recorder, 500_000_000, 0);
                                sim.run_for(1_000_000_000);
                            }

                            let expected =
                                truth(r_n, dipole, planet_radius, dcm_pn, min_reach, max_reach);
                            assert_eq!(recorder.samples.len(), 3);
                            for sample in &recorder.samples {
                                assert_relative(&sample.magnetic_field_inertial_t, &expected, 1e-5);
                            }
                        }
                    }
                }
            }
        }
    }

    /// The Earth case is a nonzero field of the expected magnitude: about
    /// 30 µT at the magnetic equator on the surface, falling with `r^-3`.
    #[test]
    fn earth_dipole_has_the_expected_magnitude() {
        let mut field = MagneticFieldCenteredDipole::new(earth_dipole());
        let state = Output::new(SpacecraftStateMsg {
            position_m: Vector3::new(2.0 * 6371.2e3, 0.0, 0.0),
            ..SpacecraftStateMsg::default()
        });
        state.connect_to(&mut field.input_state_msg);
        field.init();
        field.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });
        let magnitude = field
            .output_magnetic_field_msg
            .read()
            .magnetic_field_inertial_t
            .norm();
        assert!((3.0e-6..5.0e-6).contains(&magnitude), "{magnitude}");
    }

    /// As `WmmField`: an unconnected state (the spacecraft at the planet
    /// centre) gives a finite zero field.
    #[test]
    fn zero_radius_writes_a_finite_zero_field() {
        let mut field = MagneticFieldCenteredDipole::new(earth_dipole());
        field.init();
        field.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });
        assert_eq!(
            field
                .output_magnetic_field_msg
                .read()
                .magnetic_field_inertial_t,
            Vector3::zeros()
        );
    }
}
