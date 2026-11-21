//! Ground location access: port of upstream Basilisk `groundLocation`.
//!
//! A location `L` fixed on the surface of a spherical planet `P` publishes its
//! inertial position and surface normal ([`GroundStateMsg`]) and, for each
//! tracked spacecraft `B`, an [`AccessMsg`] with the slant range, the
//! elevation above the local horizon, the azimuth, their rates, and the
//! spacecraft position and velocity relative to the location in its
//! south-east-zenith (SEZ) frame. A spacecraft has access when its elevation
//! exceeds `minimum_elevation_rad` and its slant range is within
//! `maximum_range_m`. Illumination and glare are not modelled; those
//! [`AccessMsg`] fields stay zero.
//!
//! The planet input is optional. Unconnected, or without an orientation, the
//! planet is the identity-oriented, non-rotating body at the inertial origin
//! that upstream assumes when `planetInMsg` is not linked. The rates are
//! taken relative to the rotating planet, with its angular velocity
//! recovered from the orientation and its rate.
//!
//! Upstream reads the `isWritten` flag of every input but never uses the
//! result, so no write-time stand-in is needed.

use std::f64::consts::FRAC_PI_2;

use nalgebra::{Matrix3, Vector3};

use super::access_geometry::{PlanetFrame, azimuth_and_rates, overhead_tolerance_m};
use crate::messages::{
    AccessMsg, GroundStateMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg,
};
use crate::utilities::astro_constants::{D2R, REQ_EARTH};
use crate::utilities::geodetic_conversion::{Lla, dcm_pcpf_to_sez, lla_to_pcpf, pcpf_to_lla};
use crate::utilities::linear_algebra::safe_acos;
use crate::{Module, SimulationContext};

/// Where a surface location sits on its planet.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SiteLocation {
    /// Upstream `specifyLocation(lat, longitude, alt)`: planet-centred
    /// latitude, longitude and altitude above the spherical planet.
    Geodetic(Lla),
    /// Upstream `specifyLocationPCPF(r_LP_P_Loc)` \[m\]: planet-fixed position.
    PlanetFixed(Vector3<f64>),
}

impl SiteLocation {
    /// Planet-fixed position `r_LP_P` \[m\] and planet-fixed-to-SEZ DCM `[LP]`
    /// of the site on a sphere of `planet_radius_m`.
    pub(crate) fn resolve(&self, planet_radius_m: f64) -> (Vector3<f64>, Matrix3<f64>) {
        match *self {
            Self::Geodetic(lla) => (
                lla_to_pcpf(lla, planet_radius_m, None),
                dcm_pcpf_to_sez(lla.latitude_rad, lla.longitude_rad),
            ),
            Self::PlanetFixed(r_lp_p_m) => {
                let lla = pcpf_to_lla(r_lp_p_m, planet_radius_m, None);
                (
                    r_lp_p_m,
                    dcm_pcpf_to_sez(lla.latitude_rad, lla.longitude_rad),
                )
            }
        }
    }
}

/// User-settable parameters of [`GroundLocation`]. `Default` is upstream's
/// constructor state, with the site at zero latitude, longitude and altitude.
#[derive(Clone, Debug)]
pub struct GroundLocationConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `planetRadius` \[m\]: radius of the spherical planet; Earth's
    /// equatorial radius by default.
    pub planet_radius_m: f64,
    /// Upstream `minimumElevation` \[rad\]: elevation above the local horizon
    /// a spacecraft must exceed to have access; 10 degrees by default.
    pub minimum_elevation_rad: f64,
    /// Upstream `maximumRange` \[m\]: largest slant range with access. `None`
    /// is upstream's `-1`, no limit.
    pub maximum_range_m: Option<f64>,
    /// Upstream `r_LP_P_Init`, set through `specifyLocation` or
    /// `specifyLocationPCPF`: the site, resolved against `planet_radius_m`
    /// at reset.
    pub location: SiteLocation,
}

impl Default for GroundLocationConfig {
    fn default() -> Self {
        Self {
            name: "groundLocation".to_string(),
            planet_radius_m: REQ_EARTH * 1.0e3,
            minimum_elevation_rad: 10.0 * D2R,
            maximum_range_m: None,
            location: SiteLocation::Geodetic(Lla::default()),
        }
    }
}

impl GroundLocationConfig {
    /// Checks the configuration; upstream's `Reset` raises the radius error.
    pub fn validate(&self) -> Result<(), String> {
        if !(self.planet_radius_m >= 0.0 && self.planet_radius_m.is_finite()) {
            return Err(format!(
                "planet_radius_m must be finite and non-negative, got {}",
                self.planet_radius_m
            ));
        }
        if let Some(range) = self.maximum_range_m
            && range.is_nan()
        {
            return Err("maximum_range_m must not be NaN".to_string());
        }
        Ok(())
    }
}

/// Surface location visibility of spacecraft: upstream `GroundLocation`.
#[derive(Clone, Debug)]
pub struct GroundLocation {
    /// User-settable parameters.
    pub config: GroundLocationConfig,
    /// Upstream `planetInMsg` (optional): planet state and orientation.
    pub planet_in_msg: Input<PlanetStateMsg>,
    /// Upstream `scStateInMsgs`: one per tracked spacecraft, added by
    /// [`Self::add_spacecraft_to_model`].
    pub sc_state_in_msgs: Vec<Input<SpacecraftStateMsg>>,
    /// Upstream `currentGroundStateOutMsg`: inertial state of the location.
    pub current_ground_state_out_msg: Output<GroundStateMsg>,
    /// Upstream `accessOutMsgs`: access to each tracked spacecraft, in the
    /// order of `sc_state_in_msgs`.
    pub access_out_msgs: Vec<Output<AccessMsg>>,
    /// Upstream `r_LP_P` \[m\]: planet-fixed position of the location.
    r_lp_p_m: Vector3<f64>,
    /// Upstream `dcm_LP`: planet-fixed to SEZ DCM of the location.
    dcm_lp: Matrix3<f64>,
}

impl GroundLocation {
    /// Builds the module with unconnected ports and no spacecraft.
    ///
    /// # Panics
    ///
    /// If `config` fails [`GroundLocationConfig::validate`].
    pub fn new(config: GroundLocationConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid GroundLocationConfig: {message}");
        }
        let (r_lp_p_m, dcm_lp) = config.location.resolve(config.planet_radius_m);
        Self {
            config,
            planet_in_msg: Input::default(),
            sc_state_in_msgs: Vec::new(),
            current_ground_state_out_msg: Output::default(),
            access_out_msgs: Vec::new(),
            r_lp_p_m,
            dcm_lp,
        }
    }

    /// Upstream `addSpacecraftToModel`: adds a spacecraft state input and its
    /// access output, and returns their index in `sc_state_in_msgs` and
    /// `access_out_msgs`.
    pub fn add_spacecraft_to_model(&mut self) -> usize {
        self.sc_state_in_msgs.push(Input::default());
        self.access_out_msgs.push(Output::default());
        self.access_out_msgs.len() - 1
    }

    /// Upstream `Reset`: places the location from the configuration.
    fn reset_state(&mut self) {
        if let Err(message) = self.config.validate() {
            log::error!("{}: {message}", self.config.name);
        }
        (self.r_lp_p_m, self.dcm_lp) = self.config.location.resolve(self.config.planet_radius_m);
        if self.sc_state_in_msgs.is_empty() {
            log::error!(
                "{}: ground location has no spacecraft to track",
                self.config.name
            );
        }
    }

    /// Upstream `computeAccess` for one spacecraft.
    fn access(
        &self,
        planet: &PlanetFrame,
        r_lp_n: &Vector3<f64>,
        r_hat_lp_n: &Vector3<f64>,
        omega_pn_n: &Vector3<f64>,
        spacecraft: &SpacecraftStateMsg,
    ) -> AccessMsg {
        let r_bp_n = spacecraft.position_m - planet.r_pn_n;
        let r_bl_n = r_bp_n - r_lp_n;
        let slant_range_m = r_bl_n.norm();
        let relative_heading_n = r_bl_n / slant_range_m;
        let elevation_rad = FRAC_PI_2 - safe_acos(r_hat_lp_n.dot(&relative_heading_n));

        let dcm_ln = self.dcm_lp * planet.dcm_pn;
        let r_bl_l = dcm_ln * r_bl_n;
        // Velocity seen from the location, relative to the rotating planet.
        let v_bp_n = spacecraft.velocity_mps - planet.v_pn_n;
        let v_bl_l = dcm_ln * (v_bp_n - omega_pn_n.cross(&r_bp_n));
        let rates = azimuth_and_rates(
            &r_bl_l,
            &v_bl_l,
            overhead_tolerance_m(&planet.r_pn_n, &r_bp_n, r_lp_n),
        );

        let in_range = self
            .config
            .maximum_range_m
            .is_none_or(|range| slant_range_m <= range);
        AccessMsg {
            has_access: elevation_rad > self.config.minimum_elevation_rad && in_range,
            slant_range_m,
            elevation_rad,
            azimuth_rad: rates.azimuth_rad,
            range_dot_mps: v_bl_l.dot(&r_bl_l) / slant_range_m,
            elevation_dot_radps: rates.elevation_dot_radps,
            azimuth_dot_radps: rates.azimuth_dot_radps,
            position_bl_l_m: r_bl_l,
            velocity_bl_l_mps: v_bl_l,
            ..AccessMsg::default()
        }
    }
}

impl Module for GroundLocation {
    fn init(&mut self) {
        self.reset_state();
        self.current_ground_state_out_msg
            .write(GroundStateMsg::default());
        for output in &self.access_out_msgs {
            output.write(AccessMsg::default());
        }
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    /// Upstream `UpdateState`: `updateInertialPositions`, then
    /// `computeAccess` for every spacecraft.
    fn update(&mut self, _context: &SimulationContext) {
        let planet = PlanetFrame::from_planet_state(&self.planet_in_msg.read());
        let r_lp_n = planet.dcm_pn.transpose() * self.r_lp_p_m;
        let r_hat_lp_n = r_lp_n / r_lp_n.norm();
        let omega_pn_n = planet.omega_pn_n();
        self.current_ground_state_out_msg.write(GroundStateMsg {
            position_ln_n_m: planet.r_pn_n + r_lp_n,
            position_lp_n_m: r_lp_n,
            r_hat_lp_n,
        });

        for (input, output) in self.sc_state_in_msgs.iter().zip(&self.access_out_msgs) {
            output.write(self.access(&planet, &r_lp_n, &r_hat_lp_n, &omega_pn_n, &input.read()));
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports every case of upstream
    //! `groundLocation/_UnitTest/test_unitGroundLocation.py` with its
    //! tolerances. `test_AzElR_rates` replaces Basilisk's SPICE Earth and
    //! spacecraft with the ANISE planet ephemeris (IAU Earth orientation) and
    //! the Rust spacecraft; its truth is the module's own one-step Euler
    //! prediction, as upstream.

    use std::path::PathBuf;

    use anise::constants::frames::{EARTH_J2000, IAU_EARTH_FRAME};
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::*;
    use crate::dynamics::gravity::GravBodyData;
    use crate::environment::anise_planet_ephemeris::{
        AnisePlanetEphemeris, AnisePlanetEphemerisConfig,
    };
    use crate::messages::PlanetOrientation;
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};
    use crate::test_utils::MessageRecorder;
    use crate::utilities::orbital_motion::{ClassicElements, elements_to_state};
    use crate::utilities::rigid_body_kinematics::{euler3, tilde};

    const SECOND_NANOS: u64 = 1_000_000_000;
    /// Upstream `radius` of the single-step cases \[m\].
    const RADIUS_M: f64 = 6378136.3;
    /// Upstream `omega` of the single-step cases [rad/s].
    const SPIN_RATE_RADPS: f64 = 7.2921159e-5;

    fn context() -> SimulationContext {
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// `pytest.approx(expected, rel=rel, abs=abs)`.
    #[track_caller]
    fn assert_approx(actual: f64, expected: f64, rel: f64, abs: f64) {
        let tolerance = (rel * expected.abs()).max(abs);
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} vs {expected}, tolerance {tolerance}"
        );
    }

    /// `np.testing.assert_allclose(actual, expected, rtol=rtol, atol=atol)`.
    #[track_caller]
    fn assert_allclose(actual: &Vector3<f64>, expected: &Vector3<f64>, rtol: f64, atol: f64) {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!(
                (a - e).abs() <= atol + rtol * e.abs(),
                "{actual:?} vs {expected:?}"
            );
        }
    }

    fn spacecraft_at(position_m: Vector3<f64>) -> Output<SpacecraftStateMsg> {
        Output::new(SpacecraftStateMsg {
            position_m,
            ..SpacecraftStateMsg::default()
        })
    }

    /// Upstream's planet tilted by `tilt_deg` about the inertial 2-axis and
    /// spinning about its own 3-axis: `[PN]`, `w_PN_P` and the planet state.
    fn tilted_spinning_planet(
        tilt_deg: f64,
        r_pn_n: Vector3<f64>,
        v_pn_n: Vector3<f64>,
    ) -> (Matrix3<f64>, Vector3<f64>, PlanetStateMsg) {
        let (s, c) = tilt_deg.to_radians().sin_cos();
        let dcm_pn = Matrix3::new(c, 0.0, -s, 0.0, 1.0, 0.0, s, 0.0, c);
        let omega_pn_p = Vector3::new(0.0, 0.0, SPIN_RATE_RADPS);
        let planet = PlanetStateMsg {
            position_inertial_m: r_pn_n,
            velocity_inertial_mps: v_pn_n,
            orientation: Some(PlanetOrientation {
                inertial_to_fixed: dcm_pn,
                inertial_to_fixed_dot: -tilde(omega_pn_p) * dcm_pn,
            }),
        };
        (dcm_pn, omega_pn_p, planet)
    }

    /// One update at time zero, as upstream's `SingleStepProcesses`.
    fn single_step(
        config: GroundLocationConfig,
        planet: PlanetStateMsg,
        spacecraft: SpacecraftStateMsg,
    ) -> (AccessMsg, GroundStateMsg) {
        let planet_output = Output::new(planet);
        let spacecraft_output = Output::new(spacecraft);
        let mut station = GroundLocation::new(config);
        let index = station.add_spacecraft_to_model();
        planet_output.connect_to(&mut station.planet_in_msg);
        spacecraft_output.connect_to(&mut station.sc_state_in_msgs[index]);
        station.init();
        station.update(&context());
        (
            station.access_out_msgs[index].read(),
            station.current_ground_state_out_msg.read(),
        )
    }

    fn single_step_config(lla: Lla) -> GroundLocationConfig {
        GroundLocationConfig {
            planet_radius_m: RADIUS_M,
            minimum_elevation_rad: -FRAC_PI_2,
            location: SiteLocation::Geodetic(lla),
            ..GroundLocationConfig::default()
        }
    }

    const TILTS_DEG: [f64; 3] = [0.0, 30.0, 60.0];
    const PLANET_VELOCITIES_MPS: [[f64; 3]; 2] = [[0.0, 0.0, 0.0], [3.0e4, -1.0e3, 2.0e3]];

    /// Upstream `test_range`: slant range, elevation and the range limit for
    /// three spacecraft tracked in parallel; illumination and glare stay zero.
    #[test]
    fn range_elevation_and_range_limit_for_parallel_spacecraft() {
        let req_m = REQ_EARTH * 1.0e3;
        let mut station = GroundLocation::new(GroundLocationConfig {
            name: "groundTarget".to_string(),
            planet_radius_m: req_m,
            maximum_range_m: Some(100e3),
            minimum_elevation_rad: 80.0_f64.to_radians(),
            location: SiteLocation::Geodetic(Lla::new(0.0, 0.0, 0.0)),
        });
        let spacecraft = [
            spacecraft_at(Vector3::new(req_m + 100e3, 0.0, 0.0)),
            spacecraft_at(Vector3::new(req_m + 101e3, 0.0, 0.0)),
            spacecraft_at(
                euler3(11.0_f64.to_radians()) * Vector3::new(100e3, 0.0, 0.0)
                    + Vector3::new(req_m, 0.0, 0.0),
            ),
        ];
        let mut recorders: Vec<MessageRecorder<AccessMsg>> =
            (0..3).map(|_| MessageRecorder::default()).collect();
        for (output, recorder) in spacecraft.iter().zip(&mut recorders) {
            let index = station.add_spacecraft_to_model();
            output.connect_to(&mut station.sc_state_in_msgs[index]);
            station.access_out_msgs[index].connect_to(&mut recorder.input_msg);
        }
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("groundTarget", &mut station, SECOND_NANOS, 0);
            for (index, recorder) in recorders.iter_mut().enumerate() {
                // Upstream samples twice over 10 s: at 0 s and 10 s.
                sim.add_module(format!("log{index}"), recorder, 10 * SECOND_NANOS, 0);
            }
            sim.run_for(10 * SECOND_NANOS);
        }

        for recorder in &recorders {
            assert_eq!(recorder.samples.len(), 2);
            for sample in &recorder.samples {
                assert!(!sample.has_illumination && !sample.has_glare);
                assert_eq!(sample.sun_incidence_angle_rad, 0.0);
                assert_eq!(sample.sc_view_angle_rad, 0.0);
                assert_eq!(sample.glare_factor, 0.0);
            }
        }
        let accuracy = 1e-8;
        let reference = [
            (100e3, 90.0_f64, true),
            (101e3, 90.0, false),
            (100e3, 79.0, false),
        ];
        for (recorder, (range, elevation_deg, access)) in recorders.iter().zip(reference) {
            let sample = &recorder.samples[1];
            assert_approx(sample.slant_range_m, range, accuracy, 1e-12);
            assert_approx(
                sample.elevation_rad,
                elevation_deg.to_radians(),
                accuracy,
                1e-12,
            );
            assert_eq!(sample.has_access, access);
        }
    }

    /// Upstream `test_rotation`: a location at 10 degrees longitude on a
    /// planet turned by -10 degrees sits under a spacecraft on the inertial
    /// 1-axis.
    #[test]
    fn planet_rotation_moves_the_location() {
        let req_m = REQ_EARTH * 1.0e3;
        let planet = PlanetStateMsg {
            orientation: Some(PlanetOrientation {
                inertial_to_fixed: euler3((-10.0_f64).to_radians()),
                inertial_to_fixed_dot: Matrix3::zeros(),
            }),
            ..PlanetStateMsg::default()
        };
        let (access, _) = single_step(
            GroundLocationConfig {
                planet_radius_m: req_m,
                maximum_range_m: Some(200e3),
                minimum_elevation_rad: 10.0_f64.to_radians(),
                location: SiteLocation::Geodetic(Lla::new(0.0, 10.0_f64.to_radians(), 0.0)),
                ..GroundLocationConfig::default()
            },
            planet,
            SpacecraftStateMsg {
                position_m: Vector3::new(req_m + 90e3, 0.0, 0.0),
                ..SpacecraftStateMsg::default()
            },
        );
        assert_approx(access.slant_range_m, 90e3, 1e-8, 1e-12);
        assert_approx(access.elevation_rad, FRAC_PI_2, 1e-8, 1e-12);
        assert!(access.has_access);
    }

    /// Upstream `test_AzElR_rates`: the range, elevation and azimuth one step
    /// later match a one-step Euler integration of their rates.
    #[test]
    fn range_and_angle_rates_predict_the_next_step() {
        let repo = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let mut earth = AnisePlanetEphemeris::new(AnisePlanetEphemerisConfig {
            name: "earth".to_string(),
            spk_path: repo.join("assets/anise/de440s.bsp"),
            additional_kernel_paths: vec![repo.join("assets/anise/pck11.pca")],
            source_frame: EARTH_J2000,
            observer_frame: EARTH_J2000,
            fixed_frame: Some(IAU_EARTH_FRAME),
        });

        // Upstream `simIncludeGravBody.createEarth()`.
        let mu_m3ps2 = 0.3986004415e15;
        let rad_equator_m = REQ_EARTH * 1.0e3;
        let (r_n, v_n) = elements_to_state(
            mu_m3ps2,
            &ClassicElements {
                semi_major_axis: rad_equator_m + 100.0e3,
                eccentricity: 0.00001,
                inclination_rad: 53.0 * D2R,
                raan_rad: 115.0 * D2R,
                arg_periapsis_rad: 5.0 * D2R,
                true_anomaly_rad: 75.0 * D2R,
                ..ClassicElements::default()
            },
        );
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 100.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::identity(),
            integration_step_nanos: SECOND_NANOS,
            initial_position_m: r_n,
            initial_velocity_mps: v_n,
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: None,
        });
        spacecraft
            .add_grav_body(
                GravBodyData::point_mass(
                    "earth",
                    mu_m3ps2,
                    true,
                    Vector3::zeros(),
                    Vector3::zeros(),
                )
                .expect("valid Earth gravity body"),
            )
            .expect("unique central gravity body");

        let mut station = GroundLocation::new(GroundLocationConfig {
            planet_radius_m: rad_equator_m,
            minimum_elevation_rad: 60.0_f64.to_radians(),
            location: SiteLocation::Geodetic(Lla::new(
                40.009971_f64.to_radians(),
                (-105.243895_f64).to_radians(),
                1624.0,
            )),
            ..GroundLocationConfig::default()
        });
        let index = station.add_spacecraft_to_model();
        let mut recorder = MessageRecorder::<AccessMsg>::default();
        let dt_s = 1.0;
        {
            let epoch = Epoch::from_gregorian_utc(2021, 5, 4, 6, 47, 48, 965_000_000);
            let mut sim = Simulation::new(epoch, false);
            sim.connect(&earth.output_planet_msg, &mut station.planet_in_msg);
            sim.connect(&spacecraft.state_out, &mut station.sc_state_in_msgs[index]);
            sim.connect(&station.access_out_msgs[index], &mut recorder.input_msg);
            sim.add_module("earth", &mut earth, SECOND_NANOS, 0);
            sim.add_module("spacecraft", &mut spacecraft, SECOND_NANOS, 0);
            sim.add_module("groundStation", &mut station, SECOND_NANOS, 0);
            sim.add_module("log", &mut recorder, SECOND_NANOS, 0);
            sim.run_for(SECOND_NANOS);
        }

        let [first, second] = &recorder.samples[..] else {
            panic!("expected two access samples");
        };
        assert_approx(
            second.slant_range_m,
            first.slant_range_m + first.range_dot_mps * dt_s,
            1e-5,
            1e-12,
        );
        assert_approx(
            second.elevation_rad,
            first.elevation_rad + first.elevation_dot_radps * dt_s,
            1e-5,
            1e-12,
        );
        assert_approx(
            second.azimuth_rad,
            first.azimuth_rad + first.azimuth_dot_radps * dt_s,
            1e-5,
            1e-12,
        );
    }

    /// Upstream `test_planetFixedSpacecraftZeroRates`: a spacecraft fixed in
    /// a tilted, spinning, translating planet has no velocity or angle rates
    /// relative to the location.
    #[test]
    fn planet_fixed_spacecraft_has_zero_rates() {
        for tilt_deg in TILTS_DEG {
            for v_pn_n in PLANET_VELOCITIES_MPS.map(Vector3::from) {
                let (dcm_pn, omega_pn_p, planet) =
                    tilted_spinning_planet(tilt_deg, Vector3::zeros(), v_pn_n);
                let r_bn_n = Vector3::new(RADIUS_M + 600e3, 0.0, 1.0e6);
                let omega_pn_n = dcm_pn.transpose() * omega_pn_p;
                let (access, _) = single_step(
                    single_step_config(Lla::new(10.0_f64.to_radians(), 20.0_f64.to_radians(), 0.0)),
                    planet,
                    SpacecraftStateMsg {
                        position_m: r_bn_n,
                        velocity_mps: v_pn_n + omega_pn_n.cross(&r_bn_n),
                        ..SpacecraftStateMsg::default()
                    },
                );
                assert_allclose(&access.velocity_bl_l_mps, &Vector3::zeros(), 0.0, 1e-6);
                assert_approx(access.range_dot_mps, 0.0, 1e-6, 1e-6);
                assert_approx(access.azimuth_dot_radps, 0.0, 1e-6, 1e-11);
                assert_approx(access.elevation_dot_radps, 0.0, 1e-6, 1e-11);
            }
        }
    }

    /// Upstream `test_overheadSpacecraft`: a spacecraft exactly overhead, with
    /// a distant tilted planet and a nonzero relative velocity, gets zero
    /// azimuth and angle rates despite the round-off in its SEZ position.
    #[test]
    fn overhead_spacecraft_has_zero_azimuth_and_angle_rates() {
        let altitude_m = 600e3;
        let r_pn_n = Vector3::new(1.5e11, -2.0e10, 5.0e9);
        let (lat, lon) = (35.0_f64.to_radians(), (-70.0_f64).to_radians());
        for tilt_deg in TILTS_DEG {
            for v_pn_n in PLANET_VELOCITIES_MPS.map(Vector3::from) {
                let (dcm_pn, omega_pn_p, planet) = tilted_spinning_planet(tilt_deg, r_pn_n, v_pn_n);
                let r_hat_p = Vector3::new(lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin());
                let r_bp_n = dcm_pn.transpose() * ((RADIUS_M + altitude_m) * r_hat_p);
                let v_bn_n = v_pn_n
                    + (dcm_pn.transpose() * omega_pn_p).cross(&r_bp_n)
                    + dcm_pn.transpose() * Vector3::new(1.0e3, -2.0e3, 5.0e2);
                let (access, _) = single_step(
                    single_step_config(Lla::new(lat, lon, 0.0)),
                    planet,
                    SpacecraftStateMsg {
                        position_m: r_pn_n + r_bp_n,
                        velocity_mps: v_bn_n,
                        ..SpacecraftStateMsg::default()
                    },
                );
                assert!(access.velocity_bl_l_mps.iter().all(|v| v.is_finite()));
                assert!(access.position_bl_l_m.x.hypot(access.position_bl_l_m.y) < 1e-3);
                assert_eq!(access.azimuth_rad, 0.0);
                assert_eq!(access.azimuth_dot_radps, 0.0);
                assert_eq!(access.elevation_dot_radps, 0.0);
                assert_approx(access.slant_range_m, altitude_m, 1e-6, 1e-12);
            }
        }
    }

    /// Upstream `test_nonzeroRelativeStateOutputs`: SEZ position and
    /// velocity, range rate and the published ground state for a fixed
    /// planet and a moving spacecraft.
    #[test]
    fn publishes_relative_state_and_ground_state() {
        let planet = PlanetStateMsg {
            orientation: Some(PlanetOrientation::identity()),
            ..PlanetStateMsg::default()
        };
        let (access, ground) = single_step(
            single_step_config(Lla::new(0.0, 0.0, 0.0)),
            planet,
            SpacecraftStateMsg {
                position_m: Vector3::new(RADIUS_M + 600e3, 1.0e5, 2.0e5),
                velocity_mps: Vector3::new(10.0, 20.0, 30.0),
                ..SpacecraftStateMsg::default()
            },
        );
        let r_bl_l_expected = Vector3::new(-2.0e5, 1.0e5, 6.0e5);
        let v_bl_l_expected = Vector3::new(-30.0, 20.0, 10.0);
        assert_allclose(&access.position_bl_l_m, &r_bl_l_expected, 0.0, 1e-6);
        assert_allclose(&access.velocity_bl_l_mps, &v_bl_l_expected, 0.0, 1e-9);
        assert_approx(
            access.range_dot_mps,
            v_bl_l_expected.dot(&r_bl_l_expected) / r_bl_l_expected.norm(),
            1e-9,
            1e-12,
        );
        assert_allclose(&ground.r_hat_lp_n, &Vector3::x(), 0.0, 1e-12);
        assert_allclose(
            &ground.position_lp_n_m,
            &Vector3::new(RADIUS_M, 0.0, 0.0),
            1e-12,
            0.0,
        );
    }

    /// Not an upstream case: a site given in planet-fixed coordinates
    /// (upstream `specifyLocationPCPF`) behaves as the same site given by
    /// latitude and longitude.
    #[test]
    fn planet_fixed_and_geodetic_sites_agree() {
        let lla = Lla::new(0.3, -1.1, 250.0);
        let spacecraft = SpacecraftStateMsg {
            position_m: Vector3::new(4.0e6, -5.5e6, 2.5e6),
            velocity_mps: Vector3::new(-3.0e3, 1.0e3, 6.0e3),
            ..SpacecraftStateMsg::default()
        };
        let (_, _, planet) = tilted_spinning_planet(30.0, Vector3::zeros(), Vector3::zeros());
        let (geodetic, _) =
            single_step(single_step_config(lla), planet.clone(), spacecraft.clone());
        let r_lp_p = lla_to_pcpf(lla, RADIUS_M, None);
        let (planet_fixed, _) = single_step(
            GroundLocationConfig {
                location: SiteLocation::PlanetFixed(r_lp_p),
                ..single_step_config(lla)
            },
            planet,
            spacecraft,
        );
        assert_allclose(
            &planet_fixed.position_bl_l_m,
            &geodetic.position_bl_l_m,
            1e-12,
            1e-6,
        );
        assert_approx(planet_fixed.azimuth_rad, geodetic.azimuth_rad, 1e-12, 1e-12);
    }

    #[test]
    #[should_panic(expected = "planet_radius_m")]
    fn rejects_a_negative_planet_radius() {
        GroundLocation::new(GroundLocationConfig {
            planet_radius_m: -1.0,
            ..GroundLocationConfig::default()
        });
    }
}
