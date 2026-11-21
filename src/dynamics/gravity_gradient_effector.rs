//! Gravity-gradient torque from one or more planets: port of upstream Basilisk
//! `simulation/dynamics/GravityGradientEffector`.
//!
//! The first-order gravity-gradient torque of a planet `P` on a spacecraft with
//! center of mass `C` is (Schaub & Junkins, *Analytical Mechanics of Space
//! Systems*, ch. 4)
//!
//! `L_G = 3 μ / |R|⁵ · R × [I_C] R`, with `R = R_C/P = R_C/N − R_P/N`,
//!
//! where `[I_C]` is the spacecraft inertia about `C`. The effector sums this
//! over every configured planet, so one effector keeps applying the right
//! torque as a spacecraft travels from one planet's neighborhood to another's.
//! [`gravity_gradient_torque_body`] is the single implementation of the
//! per-planet torque; [`Spacecraft::enable_gravity_gradient`], the
//! single-central-body switch, uses it too.
//!
//! # Inputs and stand-ins
//!
//! * Upstream reads the spacecraft mass, center of mass and inertia about `B`
//!   (`m_SC`, `centerOfMassSC`, `inertiaSC`) from the state engine. Here they
//!   come from `mass_props_in_msg`, which must be connected to the spacecraft's
//!   `mass_props_out`. The spacecraft publishes them at the end of each update,
//!   so every evaluation within a step uses the mass properties at the step's
//!   start: exact while the mass distribution is fixed, lagging by up to one
//!   step for flexing or deploying parts.
//! * Upstream reads each planet's position `r_PN_N` from the gravity effector.
//!   Here each planet has a `planet_state_in_msgs` entry, to be connected to
//!   the ephemeris output that drives the matching spacecraft gravity body; an
//!   unconnected entry is a planet fixed at the origin. As for the
//!   spacecraft's gravity bodies, the state read during a spacecraft update is
//!   taken to hold at that update's time, the end of the integration step
//!   `pre_integration` announces, and is Euler-stepped to a time within the
//!   step passed to [`DynamicEffector::compute_output_at`]. Outside a step
//!   (the spacecraft's initial evaluation), and through
//!   [`DynamicEffector::compute_output`], which carries no time, it is used as
//!   read.
//! * Upstream adds the planet positions to the hub position *state*, which is
//!   relative to the central body when there is one, so the two agree only when
//!   the central body sits at the origin (as in upstream's tests, which use
//!   SPICE `zeroBase`). Here the spacecraft's inertial position is used, which
//!   is consistent in any ephemeris frame.
//! * Upstream writes `gravityGradientOutMsg` from `UpdateState`, holding the
//!   torque of the dynamics' last evaluation. Here every evaluation publishes
//!   its torque on `gravity_gradient_out_msg`, so after a spacecraft update the
//!   message likewise holds the last evaluation's torque.
//!
//! [`Spacecraft::enable_gravity_gradient`]: crate::spacecraft::Spacecraft::enable_gravity_gradient

use std::any::Any;

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    GravityGradientMsg, Input, Output, PlanetStateMsg, SpacecraftMassPropsMsg, SpacecraftStateMsg,
};
use crate::spacecraft::mrp::body_to_inertial_dcm_from_sigma_bn;
use crate::spacecraft::{DynamicEffector, EffectorOutput};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::rigid_body_kinematics::tilde;

/// Below this distance \[m\] from a planet's center the torque is taken as
/// zero rather than evaluated at the singular point; upstream has no guard.
const MIN_PLANET_DISTANCE_M: f64 = 1.0;

/// First-order gravity-gradient torque of one planet \[N m\], body components:
/// one term of upstream `GravityGradientEffector::computeForceTorque`.
///
/// `body_origin_position_m` is the body frame origin `B` relative to the
/// planet center, inertial components; `body_to_inertial_dcm` is `[NB]`. The
/// mass properties are those of the whole spacecraft, with the inertia about
/// `B`; the torque uses the inertia about the center of mass,
/// `[I_C] = [I_B] − m [c̃][c̃]ᵀ`. Within 1 m of the planet center, where the
/// torque is singular, it is taken as zero.
pub fn gravity_gradient_torque_body(
    body_origin_position_m: Vector3<f64>,
    body_to_inertial_dcm: Matrix3<f64>,
    mass_kg: f64,
    center_of_mass_body_m: Vector3<f64>,
    inertia_about_point_b_body_kg_m2: Matrix3<f64>,
    mu_m3ps2: f64,
) -> Vector3<f64> {
    let com_position_inertial_m =
        body_origin_position_m + body_to_inertial_dcm * center_of_mass_body_m;
    let radius_m = com_position_inertial_m.norm();
    if radius_m < MIN_PLANET_DISTANCE_M {
        return Vector3::zeros();
    }
    let c_tilde = tilde(center_of_mass_body_m);
    let inertia_about_com_body_kg_m2 =
        inertia_about_point_b_body_kg_m2 - mass_kg * c_tilde * c_tilde.transpose();
    let r_hat_body = body_to_inertial_dcm.transpose() * (com_position_inertial_m / radius_m);
    r_hat_body.cross(
        &(3.0 * mu_m3ps2 / radius_m / radius_m / radius_m
            * (inertia_about_com_body_kg_m2 * r_hat_body)),
    )
}

/// One gravity-gradient source planet (upstream `addPlanetName`).
#[derive(Clone, Debug)]
pub struct GravityGradientPlanet {
    /// Planet name, labelling its port (upstream `planetName`).
    pub name: String,
    /// Upstream `<planetName>.mu` \[m^3/s^2\]: gravitational parameter.
    pub mu_m3ps2: f64,
}

/// User-settable parameters of [`GravityGradientEffector`].
#[derive(Clone, Debug)]
pub struct GravityGradientEffectorConfig {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `planetPropertyNames`, one entry per `addPlanetName` call:
    /// the planets whose torques are summed. Fixed at construction.
    pub planets: Vec<GravityGradientPlanet>,
}

impl GravityGradientEffectorConfig {
    /// Checks the configuration; upstream's `Reset` and `linkInStates` raise
    /// on an empty planet list.
    pub fn validate(&self) -> Result<(), String> {
        if self.planets.is_empty() {
            return Err(
                "planets (upstream planetPropertyNames) is empty: specify at least one planet"
                    .to_string(),
            );
        }
        Ok(())
    }
}

/// Gravity-gradient torque summed over planets: upstream
/// `GravityGradientEffector`.
#[derive(Clone, Debug)]
pub struct GravityGradientEffector {
    /// User-settable parameters.
    pub config: GravityGradientEffectorConfig,
    /// Inertial state of each planet, at the index of its
    /// `config.planets` entry (upstream `<planetName>.r_PN_N`).
    pub planet_state_in_msgs: Vec<Input<PlanetStateMsg>>,
    /// The spacecraft's mass properties (upstream `m_SC`, `centerOfMassSC`,
    /// `inertiaSC`); connect it to the spacecraft's `mass_props_out`.
    pub mass_props_in_msg: Input<SpacecraftMassPropsMsg>,
    /// Upstream `gravityGradientOutMsg`: the torque of the last evaluation.
    pub gravity_gradient_out_msg: Output<GravityGradientMsg>,
    /// Start and end \[ns\] of the current integration step; the planet
    /// states read during it hold at its end.
    current_step_nanos: Option<(u64, u64)>,
}

impl GravityGradientEffector {
    /// Builds the effector with one unconnected planet input per planet.
    ///
    /// # Panics
    ///
    /// If `config` fails [`GravityGradientEffectorConfig::validate`].
    pub fn new(config: GravityGradientEffectorConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid GravityGradientEffectorConfig: {message}");
        }
        let planet_count = config.planets.len();
        Self {
            config,
            planet_state_in_msgs: (0..planet_count).map(|_| Input::default()).collect(),
            mass_props_in_msg: Input::default(),
            gravity_gradient_out_msg: Output::default(),
            current_step_nanos: None,
        }
    }

    /// Index of the port of the planet called `name`.
    pub fn planet_index(&self, name: &str) -> Option<usize> {
        self.config
            .planets
            .iter()
            .position(|planet| planet.name == name)
    }

    /// Upstream `computeForceTorque`: the torque \[N m\] about `B`, body
    /// components, for the spacecraft `state` at `current_sim_nanos`.
    ///
    /// # Panics
    ///
    /// If `mass_props_in_msg` is not connected.
    pub fn torque_body_at(
        &self,
        state: &SpacecraftStateMsg,
        current_sim_nanos: u64,
    ) -> Vector3<f64> {
        let planet_step_s = match self.current_step_nanos {
            Some((start, end)) if (start..=end).contains(&current_sim_nanos) => {
                diff_nanos_to_seconds(current_sim_nanos, end)
            }
            _ => 0.0,
        };
        self.torque_body(state, planet_step_s)
    }

    /// The torque with every planet state Euler-stepped by `planet_step_s`.
    fn torque_body(&self, state: &SpacecraftStateMsg, planet_step_s: f64) -> Vector3<f64> {
        assert!(
            self.mass_props_in_msg.is_connected(),
            "GravityGradientEffector '{}': mass_props_in_msg is not connected; connect it to \
             the spacecraft's mass_props_out",
            self.config.name
        );
        let mass_props = self.mass_props_in_msg.read();
        let body_to_inertial_dcm = body_to_inertial_dcm_from_sigma_bn(state.sigma_bn);
        self.config
            .planets
            .iter()
            .zip(&self.planet_state_in_msgs)
            .map(|(planet, planet_state_in)| {
                let planet_state = planet_state_in.read();
                let planet_position_m = planet_state.position_inertial_m
                    + planet_state.velocity_inertial_mps * planet_step_s;
                gravity_gradient_torque_body(
                    state.position_m - planet_position_m,
                    body_to_inertial_dcm,
                    mass_props.mass_kg,
                    mass_props.center_of_mass_body_m,
                    mass_props.inertia_about_point_b_body_kg_m2,
                    planet.mu_m3ps2,
                )
            })
            .sum()
    }

    /// Publishes `torque_body_nm` and returns it as the effector's load.
    fn publish(&self, torque_body_nm: Vector3<f64>) -> EffectorOutput {
        self.gravity_gradient_out_msg.write(GravityGradientMsg {
            gravity_gradient_torque_body_nm: torque_body_nm,
        });
        EffectorOutput {
            force_inertial_n: Vector3::zeros(),
            torque_body_nm,
        }
    }
}

impl DynamicEffector for GravityGradientEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    /// Records the step, whose end is the time the planet states read during
    /// this update hold.
    fn pre_integration(&mut self, current_sim_nanos: u64, dt_seconds: f64) {
        let step_nanos = (dt_seconds * 1.0e9).round() as u64;
        self.current_step_nanos = Some((current_sim_nanos, current_sim_nanos + step_nanos));
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        self.publish(self.torque_body(state, 0.0))
    }

    fn compute_output_at(
        &self,
        state: &SpacecraftStateMsg,
        current_sim_nanos: u64,
    ) -> EffectorOutput {
        self.publish(self.torque_body_at(state, current_sim_nanos))
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `GravityGradientEffector/_UnitTest/test_gravityGradient.py`
    //! and `test_gravityGradientValidation.py` with their tolerances. Upstream's
    //! SPICE planets (with `zeroBase` on the central body) are the ANISE
    //! ephemeris observed from the central body, which sits at the origin.

    use std::path::PathBuf;

    use anise::constants::frames::{EARTH_J2000, VENUS_J2000};
    use anise::frames::Frame;
    use hifitime::Epoch;

    use super::*;
    use crate::dynamics::gravity::GravBodyData;
    use crate::environment::anise_planet_ephemeris::{
        AnisePlanetEphemeris, AnisePlanetEphemerisConfig,
    };
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};
    use crate::test_utils::MessageRecorder;
    use crate::utilities::orbital_motion::{ClassicElements, elements_to_state};
    use crate::utilities::rigid_body_kinematics::mrp_to_dcm;

    /// Upstream `simIncludeGravBody` `createEarth().mu` \[m^3/s^2\].
    const MU_EARTH_M3PS2: f64 = 0.398_600_441_5e15;
    /// Upstream `simIncludeGravBody` `createVenus().mu` \[m^3/s^2\].
    const MU_VENUS_M3PS2: f64 = 3.248_585_99e14;

    fn repo_path(relative: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(relative)
    }

    fn anise_body(name: &str, source: Frame, observer: Frame) -> AnisePlanetEphemeris {
        AnisePlanetEphemeris::new(AnisePlanetEphemerisConfig {
            name: name.to_string(),
            spk_path: repo_path("assets/anise/de440s.bsp"),
            additional_kernel_paths: Vec::new(),
            source_frame: source,
            observer_frame: observer,
            fixed_frame: None,
        })
    }

    fn planet(name: &str, mu_m3ps2: f64) -> GravityGradientPlanet {
        GravityGradientPlanet {
            name: name.to_string(),
            mu_m3ps2,
        }
    }

    fn spacecraft_config(
        mu_m3ps2: f64,
        center_of_mass_body_m: Vector3<f64>,
        inertia_about_com_kg_m2: Matrix3<f64>,
        step_nanos: u64,
    ) -> SpacecraftConfig {
        let (position_m, velocity_mps) = elements_to_state(
            mu_m3ps2,
            &ClassicElements {
                semi_major_axis: 7000.0 * 1000.0,
                eccentricity: 0.0001,
                inclination_rad: 33.3_f64.to_radians(),
                raan_rad: 48.2_f64.to_radians(),
                arg_periapsis_rad: 347.8_f64.to_radians(),
                true_anomaly_rad: 85.3_f64.to_radians(),
                ..ClassicElements::default()
            },
        );
        SpacecraftConfig {
            mass_kg: 100.0,
            hub_center_of_mass_body_m: center_of_mass_body_m,
            inertia_kg_m2: inertia_about_com_kg_m2,
            integration_step_nanos: step_nanos,
            initial_position_m: position_m,
            initial_velocity_mps: velocity_mps,
            initial_sigma_bn: Vector3::new(0.1, 0.2, -0.3),
            initial_omega_radps: Vector3::zeros(),
            integrator: None,
        }
    }

    /// Upstream `truthGravityGradient`: the torque of the central body alone,
    /// from the body frame origin's position and the hub inertia about its
    /// center of mass.
    fn truth_gravity_gradient(
        mu_m3ps2: f64,
        position_m: Vector3<f64>,
        sigma_bn: Vector3<f64>,
        inertia_about_com_kg_m2: Matrix3<f64>,
    ) -> Vector3<f64> {
        let r = position_m.norm();
        let r_hat_b = mrp_to_dcm(sigma_bn) * position_m / r;
        3.0 * mu_m3ps2 / r / r / r * r_hat_b.cross(&(inertia_about_com_kg_m2 * r_hat_b))
    }

    /// Upstream `run`: `planet_case` 0 has Earth alone at the origin; 1 adds
    /// Venus from the ephemeris with only Earth's torque; 2 includes Venus'
    /// torque; 3 orbits Venus, with both torques.
    fn run(center_of_mass_body_m: Vector3<f64>, planet_case: u32) {
        let step_nanos = 1_000_000_000;
        let simulation_nanos = 2_000_000_000;
        let inertia = Matrix3::from_diagonal(&Vector3::new(500.0, 800.0, 350.0));
        let orbits_venus = planet_case == 3;
        let (central_name, central_mu, other_name, other_mu, other_frame, central_frame) =
            if orbits_venus {
                (
                    "venus",
                    MU_VENUS_M3PS2,
                    "earth",
                    MU_EARTH_M3PS2,
                    EARTH_J2000,
                    VENUS_J2000,
                )
            } else {
                (
                    "earth",
                    MU_EARTH_M3PS2,
                    "venus",
                    MU_VENUS_M3PS2,
                    VENUS_J2000,
                    EARTH_J2000,
                )
            };

        let mut spacecraft = Spacecraft::new(spacecraft_config(
            central_mu,
            center_of_mass_body_m,
            inertia,
            step_nanos,
        ));
        spacecraft
            .add_grav_body(
                GravBodyData::point_mass(
                    central_name,
                    central_mu,
                    true,
                    Vector3::zeros(),
                    Vector3::zeros(),
                )
                .unwrap(),
            )
            .unwrap();
        let mut other_ephemeris = anise_body(other_name, other_frame, central_frame);
        if planet_case > 0 {
            spacecraft
                .add_grav_body(
                    GravBodyData::point_mass(
                        other_name,
                        other_mu,
                        false,
                        Vector3::zeros(),
                        Vector3::zeros(),
                    )
                    .unwrap(),
                )
                .unwrap();
            other_ephemeris
                .output_planet_msg
                .connect_to(spacecraft.grav_body_input_mut(other_name).unwrap());
        }

        let mut planets = vec![planet("earth", MU_EARTH_M3PS2)];
        if planet_case >= 2 {
            planets.push(planet("venus", MU_VENUS_M3PS2));
        }
        let mut gravity_gradient = GravityGradientEffector::new(GravityGradientEffectorConfig {
            name: "bskTestSat".to_string(),
            planets,
        });
        if let Some(index) = gravity_gradient.planet_index(other_name) {
            other_ephemeris
                .output_planet_msg
                .connect_to(&mut gravity_gradient.planet_state_in_msgs[index]);
        }
        spacecraft
            .mass_props_out
            .connect_to(&mut gravity_gradient.mass_props_in_msg);
        let mut state_log = MessageRecorder::<SpacecraftStateMsg>::default();
        let mut torque_log = MessageRecorder::<GravityGradientMsg>::default();
        spacecraft.state_out.connect_to(&mut state_log.input_msg);
        gravity_gradient
            .gravity_gradient_out_msg
            .connect_to(&mut torque_log.input_msg);
        spacecraft.add_dynamic_effector(gravity_gradient);

        {
            let epoch = Epoch::from_gregorian_utc(2012, 5, 1, 0, 28, 30, 0);
            let mut sim = Simulation::new(epoch, false);
            if planet_case > 0 {
                sim.add_module("ephemeris", &mut other_ephemeris, step_nanos, 20);
            }
            sim.add_module("spacecraft", &mut spacecraft, step_nanos, 10);
            sim.add_module("stateLog", &mut state_log, step_nanos, 0);
            sim.add_module("torqueLog", &mut torque_log, step_nanos, 0);
            sim.run_for(simulation_nanos);
        }

        assert_eq!(torque_log.samples.len(), 3);
        let accuracy = 1e-10;
        for (state, torque) in state_log.samples.iter().zip(&torque_log.samples) {
            let truth =
                truth_gravity_gradient(central_mu, state.position_m, state.sigma_bn, inertia);
            let error = (torque.gravity_gradient_torque_body_nm - truth).abs().max();
            assert!(
                error <= accuracy,
                "case {planet_case}, c = {center_of_mass_body_m:?}: {:?} vs {truth:?}",
                torque.gravity_gradient_torque_body_nm
            );
            assert!(
                truth.norm() > 1e-4,
                "the torque under test should be significant"
            );
        }
    }

    /// Upstream `test_gravityGradientModule`, over its `planetCase` and
    /// `cmOffset` parameters.
    #[test]
    fn torque_matches_the_central_body_truth_in_every_planet_case() {
        for center_of_mass_body_m in [Vector3::new(0.1, 0.15, -0.1), Vector3::zeros()] {
            for planet_case in 0..4 {
                run(center_of_mass_body_m, planet_case);
            }
        }
    }

    /// Upstream `test_gravityGradient_validation`: attachment and `Reset`
    /// reject an empty planet list; here construction does.
    #[test]
    #[should_panic(expected = "planetPropertyNames")]
    fn rejects_an_empty_planet_list() {
        GravityGradientEffector::new(GravityGradientEffectorConfig {
            name: "gg".to_string(),
            planets: Vec::new(),
        });
    }

    #[test]
    #[should_panic(expected = "mass_props_in_msg is not connected")]
    fn requires_the_mass_properties() {
        let effector = GravityGradientEffector::new(GravityGradientEffectorConfig {
            name: "gg".to_string(),
            planets: vec![planet("earth", MU_EARTH_M3PS2)],
        });
        effector.compute_output(&SpacecraftStateMsg::default());
    }

    /// Each planet contributes its own term at its Euler-stepped position, and
    /// the published message carries the summed torque.
    #[test]
    fn sums_planets_at_their_euler_stepped_positions() {
        let mut effector = GravityGradientEffector::new(GravityGradientEffectorConfig {
            name: "gg".to_string(),
            planets: vec![planet("earth", MU_EARTH_M3PS2), planet("moon", 4.9028e12)],
        });
        let mass_props = Output::new(SpacecraftMassPropsMsg {
            mass_kg: 50.0,
            center_of_mass_body_m: Vector3::new(0.02, -0.01, 0.03),
            inertia_about_point_b_body_kg_m2: Matrix3::new(
                30.0, 1.0, -2.0, 1.0, 40.0, 0.5, -2.0, 0.5, 20.0,
            ),
        });
        mass_props.connect_to(&mut effector.mass_props_in_msg);
        let moon = Output::new(PlanetStateMsg {
            position_inertial_m: Vector3::new(3.0e8, 2.0e8, -1.0e7),
            velocity_inertial_mps: Vector3::new(-500.0, 800.0, 10.0),
            orientation: None,
        });
        let moon_index = effector.planet_index("moon").unwrap();
        moon.connect_to(&mut effector.planet_state_in_msgs[moon_index]);
        // A 10 s step ending at t = 30 s, evaluated at its midpoint.
        effector.pre_integration(20_000_000_000, 10.0);
        let state = SpacecraftStateMsg {
            position_m: Vector3::new(4.0e6, -5.0e6, 2.0e6),
            sigma_bn: Vector3::new(0.2, -0.1, 0.4),
            ..Default::default()
        };

        let output = effector.compute_output_at(&state, 25_000_000_000);

        let mass = mass_props.read();
        let dcm_nb = body_to_inertial_dcm_from_sigma_bn(state.sigma_bn);
        let moon_position_m =
            moon.read().position_inertial_m + moon.read().velocity_inertial_mps * -5.0;
        let expected = [
            (state.position_m, MU_EARTH_M3PS2),
            (state.position_m - moon_position_m, 4.9028e12),
        ]
        .iter()
        .map(|(position_m, mu)| {
            gravity_gradient_torque_body(
                *position_m,
                dcm_nb,
                mass.mass_kg,
                mass.center_of_mass_body_m,
                mass.inertia_about_point_b_body_kg_m2,
                *mu,
            )
        })
        .sum::<Vector3<f64>>();
        assert_eq!(output.torque_body_nm, expected);
        assert_eq!(output.force_inertial_n, Vector3::zeros());
        assert_eq!(
            effector
                .gravity_gradient_out_msg
                .read()
                .gravity_gradient_torque_body_nm,
            expected
        );
        // At the step end, without a time and outside the step, the planets
        // hold their states as read.
        let unstepped = effector.compute_output(&state).torque_body_nm;
        assert_ne!(unstepped, expected);
        for nanos in [30_000_000_000, 0, 40_000_000_000] {
            assert_eq!(
                effector.compute_output_at(&state, nanos).torque_body_nm,
                unstepped
            );
        }
    }

    /// The torque about `C` uses the inertia about `C`, whatever the offset
    /// of `C` from `B`, and vanishes at the guarded singular point.
    #[test]
    fn torque_uses_the_inertia_about_the_center_of_mass() {
        let inertia_c = Matrix3::new(10.0, 0.3, 0.1, 0.3, 20.0, -0.2, 0.1, -0.2, 30.0);
        let mass_kg = 12.0;
        let c = Vector3::new(0.5, -0.25, 0.75);
        let inertia_b = inertia_c + mass_kg * tilde(c) * tilde(c).transpose();
        let dcm_nb = mrp_to_dcm(Vector3::new(0.1, 0.3, -0.2)).transpose();
        let r_c = Vector3::new(6.8e6, 1.0e6, -2.0e6);
        let r_b = r_c - dcm_nb * c;

        let torque =
            gravity_gradient_torque_body(r_b, dcm_nb, mass_kg, c, inertia_b, MU_EARTH_M3PS2);
        let centered = gravity_gradient_torque_body(
            r_c,
            dcm_nb,
            mass_kg,
            Vector3::zeros(),
            inertia_c,
            MU_EARTH_M3PS2,
        );
        assert!((torque - centered).norm() <= 1e-15 * centered.norm());
        let r_hat_b = dcm_nb.transpose() * r_c.normalize();
        let expected =
            3.0 * MU_EARTH_M3PS2 / r_c.norm().powi(3) * r_hat_b.cross(&(inertia_c * r_hat_b));
        assert!((torque - expected).norm() <= 1e-14 * expected.norm());

        assert_eq!(
            gravity_gradient_torque_body(
                Vector3::new(0.5, 0.0, 0.0),
                Matrix3::identity(),
                1.0,
                Vector3::zeros(),
                inertia_c,
                MU_EARTH_M3PS2
            ),
            Vector3::zeros()
        );
    }

    /// The spacecraft's single-body switch and a one-planet effector share
    /// the torque, so they propagate a rigid spacecraft identically.
    #[test]
    fn matches_the_spacecraft_gravity_gradient_switch() {
        let step_nanos = 100_000_000;
        let inertia = Matrix3::new(500.0, 5.0, -3.0, 5.0, 800.0, 2.0, -3.0, 2.0, 350.0);
        let center_of_mass = Vector3::new(0.1, 0.15, -0.1);
        let earth = || {
            GravBodyData::point_mass(
                "earth",
                MU_EARTH_M3PS2,
                true,
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .unwrap()
        };

        let mut with_switch = Spacecraft::new(spacecraft_config(
            MU_EARTH_M3PS2,
            center_of_mass,
            inertia,
            step_nanos,
        ));
        with_switch.add_grav_body(earth()).unwrap();
        with_switch.enable_gravity_gradient(MU_EARTH_M3PS2);

        let mut with_effector = Spacecraft::new(spacecraft_config(
            MU_EARTH_M3PS2,
            center_of_mass,
            inertia,
            step_nanos,
        ));
        with_effector.add_grav_body(earth()).unwrap();
        let mut effector = GravityGradientEffector::new(GravityGradientEffectorConfig {
            name: "gg".to_string(),
            planets: vec![planet("earth", MU_EARTH_M3PS2)],
        });
        with_effector
            .mass_props_out
            .connect_to(&mut effector.mass_props_in_msg);
        with_effector.add_dynamic_effector(effector);

        let switch_state = with_switch.state_out.clone();
        let effector_state = with_effector.state_out.clone();
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("switch", &mut with_switch, step_nanos, 0);
            sim.add_module("effector", &mut with_effector, step_nanos, 0);
            sim.run_for(60_000_000_000);
        }
        let (switch_state, effector_state) = (switch_state.read(), effector_state.read());
        assert_ne!(switch_state.omega_radps, Vector3::zeros());
        assert_eq!(switch_state.omega_radps, effector_state.omega_radps);
        assert_eq!(switch_state.sigma_bn, effector_state.sigma_bn);
        assert_eq!(switch_state.position_m, effector_state.position_m);
    }
}
