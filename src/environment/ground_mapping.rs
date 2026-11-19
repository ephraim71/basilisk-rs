//! Ground mapping coverage: port of upstream Basilisk `groundMapping`.
//!
//! A set of mapping points `L`, fixed on the surface of a spherical planet
//! `P`, is checked against one spacecraft `B` and its body-fixed instrument.
//! Each point publishes its inertial state ([`GroundStateMsg`]) and an
//! [`AccessMsg`] with the same range, angle and rate geometry as
//! [`super::ground_location::GroundLocation`]. A point is mapped (has access)
//! when the spacecraft is above `minimum_elevation_rad` as seen from the point,
//! within `maximum_range_m`, and the point lies inside the instrument's
//! conical field of view: in front of the instrument, within
//! `maximum_range_m` along its boresight, and closer to the boresight than
//! the cone radius at that depth. Illumination and glare are not modelled.
//!
//! The planet input is optional, with the defaults of `groundLocation`. The
//! spacecraft input is required; upstream's `Reset` raises when it is not
//! linked, which is logged here.

use std::f64::consts::FRAC_PI_2;

use nalgebra::{Matrix3, Vector3};

use super::access_geometry::{PlanetFrame, azimuth_and_rates, overhead_tolerance_m};
use crate::messages::{
    AccessMsg, GroundStateMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg,
};
use crate::utilities::astro_constants::D2R;
use crate::utilities::geodetic_conversion::{dcm_pcpf_to_sez, pcpf_to_lla};
use crate::utilities::linear_algebra::safe_acos;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Below this magnitude every component of `n_hat_b` counts as zero
/// (Eigen's `isZero()` default precision, as upstream's `Reset` check).
const ZERO_DIRECTION_PRECISION: f64 = 1e-12;

/// User-settable parameters of [`GroundMapping`]. `Default` is upstream's
/// constructor state; `n_hat_b` must be set before use.
#[derive(Clone, Debug)]
pub struct GroundMappingConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `minimumElevation` \[rad\]: elevation above a point's horizon
    /// the spacecraft must exceed; 0 by default.
    pub minimum_elevation_rad: f64,
    /// Upstream `maximumRange` \[m\]: largest slant range, and largest depth
    /// along the boresight, with access. `None` is upstream's `-1`, no limit.
    pub maximum_range_m: Option<f64>,
    /// Upstream `cameraPos_B` \[m\]: instrument position, body components.
    pub camera_pos_b_m: Vector3<f64>,
    /// Upstream `halfFieldOfView` \[rad\]: instrument cone half-angle; 10
    /// degrees by default.
    pub half_field_of_view_rad: f64,
    /// Upstream `nHat_B` [-]: instrument boresight, body components. Used as
    /// given, as upstream, so it should be a unit vector.
    pub n_hat_b: Vector3<f64>,
}

impl Default for GroundMappingConfig {
    fn default() -> Self {
        Self {
            name: "groundMapping".to_string(),
            minimum_elevation_rad: 0.0,
            maximum_range_m: None,
            camera_pos_b_m: Vector3::zeros(),
            half_field_of_view_rad: 10.0 * D2R,
            n_hat_b: Vector3::zeros(),
        }
    }
}

impl GroundMappingConfig {
    /// Checks the configuration; upstream's `Reset` raises the boresight
    /// error.
    pub fn validate(&self) -> Result<(), String> {
        if self
            .n_hat_b
            .iter()
            .all(|component| component.abs() <= ZERO_DIRECTION_PRECISION)
        {
            return Err("n_hat_b vector not set".to_string());
        }
        if let Some(range) = self.maximum_range_m
            && range.is_nan()
        {
            return Err("maximum_range_m must not be NaN".to_string());
        }
        Ok(())
    }
}

/// Mapping-point visibility from a spacecraft instrument: upstream
/// `GroundMapping`.
#[derive(Clone, Debug)]
pub struct GroundMapping {
    /// User-settable parameters.
    pub config: GroundMappingConfig,
    /// Upstream `planetInMsg` (optional): planet state and orientation.
    pub planet_in_msg: Input<PlanetStateMsg>,
    /// Upstream `scStateInMsg`: the mapping spacecraft.
    pub sc_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `currentGroundStateOutMsgs`: inertial state of each mapping
    /// point, added by [`Self::add_point_to_model`].
    pub current_ground_state_out_msgs: Vec<Output<GroundStateMsg>>,
    /// Upstream `accessOutMsgs`: access to each mapping point, in the order
    /// of `current_ground_state_out_msgs`.
    pub access_out_msgs: Vec<Output<AccessMsg>>,
    /// Upstream `mappingPoints` \[m\]: planet-fixed point positions.
    mapping_points_p_m: Vec<Vector3<f64>>,
    /// Upstream `dcm_LP`: planet-fixed to SEZ DCM of each point.
    dcm_lp: Vec<Matrix3<f64>>,
}

/// Spacecraft geometry shared by every point of one update.
struct SpacecraftGeometry {
    /// Upstream `r_BP_N` \[m\].
    r_bp_n: Vector3<f64>,
    /// Upstream `v_BP_N` [m/s].
    v_bp_n: Vector3<f64>,
    /// Upstream `dcm_NB`.
    dcm_nb: Matrix3<f64>,
}

impl GroundMapping {
    /// Builds the module with unconnected ports and no mapping points.
    ///
    /// # Panics
    ///
    /// If `config` fails [`GroundMappingConfig::validate`].
    pub fn new(config: GroundMappingConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid GroundMappingConfig: {message}");
        }
        Self {
            config,
            planet_in_msg: Input::default(),
            sc_state_in_msg: Input::default(),
            current_ground_state_out_msgs: Vec::new(),
            access_out_msgs: Vec::new(),
            mapping_points_p_m: Vec::new(),
            dcm_lp: Vec::new(),
        }
    }

    /// Upstream `addPointToModel`: adds the planet-fixed mapping point
    /// `r_lp_p_init_m` \[m\] with its ground-state and access outputs, and
    /// returns their index.
    ///
    /// The SEZ frame of the point follows from its direction alone, since
    /// the planet is spherical.
    pub fn add_point_to_model(&mut self, r_lp_p_init_m: Vector3<f64>) -> usize {
        let lla = pcpf_to_lla(r_lp_p_init_m, 0.0, None);
        self.mapping_points_p_m.push(r_lp_p_init_m);
        self.dcm_lp
            .push(dcm_pcpf_to_sez(lla.latitude_rad, lla.longitude_rad));
        self.access_out_msgs.push(Output::default());
        self.current_ground_state_out_msgs.push(Output::default());
        self.access_out_msgs.len() - 1
    }

    /// Upstream `Reset`.
    fn reset_state(&self) {
        if !self.sc_state_in_msg.is_connected() {
            log::error!("{}.sc_state_in_msg was not linked", self.config.name);
        }
        if let Err(message) = self.config.validate() {
            log::error!("{}: {message}", self.config.name);
        }
    }

    /// Upstream `checkInstrumentFOV`: whether the point at `r_lp_n` lies in
    /// the instrument cone.
    fn in_instrument_fov(&self, spacecraft: &SpacecraftGeometry, r_lp_n: &Vector3<f64>) -> bool {
        let camera_to_point_n =
            r_lp_n - (spacecraft.r_bp_n + spacecraft.dcm_nb * self.config.camera_pos_b_m);
        let boresight_n = spacecraft.dcm_nb * self.config.n_hat_b;
        let boresight_normal_projection = camera_to_point_n.dot(&boresight_n);
        let within_depth = self
            .config
            .maximum_range_m
            .is_none_or(|range| boresight_normal_projection <= range);
        if !(boresight_normal_projection >= 0.0 && within_depth) {
            return false;
        }
        let cone_radius = boresight_normal_projection * self.config.half_field_of_view_rad.tan();
        let orthogonal_distance = (camera_to_point_n
            - boresight_normal_projection * spacecraft.dcm_nb * self.config.n_hat_b)
            .norm();
        orthogonal_distance < cone_radius
    }

    /// Upstream `computeAccess` for the point with index `point`.
    fn point_access(
        &self,
        point: usize,
        planet: &PlanetFrame,
        omega_pn_n: &Vector3<f64>,
        spacecraft: &SpacecraftGeometry,
    ) -> (GroundStateMsg, AccessMsg) {
        let r_lp_n = planet.dcm_pn.transpose() * self.mapping_points_p_m[point];
        let r_hat_lp_n = r_lp_n / r_lp_n.norm();
        let ground_state = GroundStateMsg {
            position_ln_n_m: planet.r_pn_n + r_lp_n,
            position_lp_n_m: r_lp_n,
            r_hat_lp_n,
        };

        let r_bl_n = spacecraft.r_bp_n - r_lp_n;
        let slant_range_m = r_bl_n.norm();
        let relative_heading_n = r_bl_n / slant_range_m;
        let elevation_rad = FRAC_PI_2 - safe_acos(r_hat_lp_n.dot(&relative_heading_n));

        let dcm_ln = self.dcm_lp[point] * planet.dcm_pn;
        let r_bl_l = dcm_ln * r_bl_n;
        let v_bl_l = dcm_ln * (spacecraft.v_bp_n - omega_pn_n.cross(&spacecraft.r_bp_n));
        let rates = azimuth_and_rates(
            &r_bl_l,
            &v_bl_l,
            overhead_tolerance_m(&planet.r_pn_n, &spacecraft.r_bp_n, &r_lp_n),
        );

        let in_range = self
            .config
            .maximum_range_m
            .is_none_or(|range| slant_range_m <= range);
        let access = AccessMsg {
            has_access: elevation_rad > self.config.minimum_elevation_rad
                && in_range
                && self.in_instrument_fov(spacecraft, &r_lp_n),
            slant_range_m,
            elevation_rad,
            azimuth_rad: rates.azimuth_rad,
            range_dot_mps: v_bl_l.dot(&r_bl_l) / slant_range_m,
            elevation_dot_radps: rates.elevation_dot_radps,
            azimuth_dot_radps: rates.azimuth_dot_radps,
            position_bl_l_m: r_bl_l,
            velocity_bl_l_mps: v_bl_l,
            ..AccessMsg::default()
        };
        (ground_state, access)
    }
}

impl Module for GroundMapping {
    fn init(&mut self) {
        self.reset_state();
        for (state, access) in self
            .current_ground_state_out_msgs
            .iter()
            .zip(&self.access_out_msgs)
        {
            state.write(GroundStateMsg::default());
            access.write(AccessMsg::default());
        }
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    /// Upstream `UpdateState`: `updateInertialPositions`, then
    /// `computeAccess` for every mapping point.
    fn update(&mut self, _context: &SimulationContext) {
        let planet = PlanetFrame::from_planet_state(&self.planet_in_msg.read());
        let state = self.sc_state_in_msg.read();
        let spacecraft = SpacecraftGeometry {
            r_bp_n: state.position_m - planet.r_pn_n,
            v_bp_n: state.velocity_mps - planet.v_pn_n,
            dcm_nb: mrp_to_dcm(state.sigma_bn).transpose(),
        };
        let omega_pn_n = planet.omega_pn_n();

        for point in 0..self.mapping_points_p_m.len() {
            let (ground_state, access) =
                self.point_access(point, &planet, &omega_pn_n, &spacecraft);
            self.current_ground_state_out_msgs[point].write(ground_state);
            self.access_out_msgs[point].write(access);
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports every case of upstream
    //! `groundMapping/_UnitTest/test_groundMapping.py` with its tolerances,
    //! reimplementing its Python truth (`_dcm_LP`, `_azEl` and the central
    //! differences).

    use hifitime::Epoch;

    use super::*;
    use crate::messages::PlanetOrientation;
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::rigid_body_kinematics::tilde;

    /// Upstream `radius` \[m\].
    const RADIUS_M: f64 = 6378136.3;
    /// Upstream `omega` [rad/s].
    const SPIN_RATE_RADPS: f64 = 7.2921159e-5;
    /// Upstream `POINTS_LAT_LON` \[deg\].
    const POINTS_LAT_LON_DEG: [[f64; 2]; 3] = [[10.0, 20.0], [-35.0, 140.0], [60.0, -75.0]];
    const TILTS_DEG: [f64; 3] = [0.0, 30.0, 60.0];
    const PLANET_VELOCITIES_MPS: [[f64; 3]; 2] = [[0.0, 0.0, 0.0], [3.0e4, -1.0e3, 2.0e3]];

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

    /// Upstream `_dcm_LP`: planet-fixed to SEZ DCM of a latitude and
    /// longitude.
    fn truth_dcm_lp(lat: f64, lon: f64) -> Matrix3<f64> {
        Matrix3::new(
            lat.sin() * lon.cos(),
            lat.sin() * lon.sin(),
            -lat.cos(),
            -lon.sin(),
            lon.cos(),
            0.0,
            lat.cos() * lon.cos(),
            lat.cos() * lon.sin(),
            lat.sin(),
        )
    }

    /// Upstream `_azEl`: azimuth and elevation of an SEZ position.
    fn truth_az_el(r_l: &Vector3<f64>) -> (f64, f64) {
        (r_l.y.atan2(-r_l.x), r_l.z.atan2(r_l.x.hypot(r_l.y)))
    }

    fn unit_direction(lat: f64, lon: f64) -> Vector3<f64> {
        Vector3::new(lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin())
    }

    fn boresight_config() -> GroundMappingConfig {
        GroundMappingConfig {
            minimum_elevation_rad: -FRAC_PI_2,
            n_hat_b: Vector3::new(0.0, 1.0, 0.0),
            half_field_of_view_rad: 22.5_f64.to_radians(),
            ..GroundMappingConfig::default()
        }
    }

    /// A planet tilted by `tilt_deg` about the inertial 2-axis, spinning about
    /// its own 3-axis: `[PN]`, `w_PN_P` and its state.
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

    /// One update with `points`, as upstream's `SingleStepProcesses`.
    fn single_step(
        config: GroundMappingConfig,
        planet: PlanetStateMsg,
        spacecraft: SpacecraftStateMsg,
        points: &[Vector3<f64>],
    ) -> Vec<(AccessMsg, GroundStateMsg)> {
        let planet_output = Output::new(planet);
        let spacecraft_output = Output::new(spacecraft);
        let mut mapping = GroundMapping::new(config);
        for point in points {
            mapping.add_point_to_model(*point);
        }
        planet_output.connect_to(&mut mapping.planet_in_msg);
        spacecraft_output.connect_to(&mut mapping.sc_state_in_msg);
        mapping.init();
        mapping.update(&context());
        mapping
            .access_out_msgs
            .iter()
            .zip(&mapping.current_ground_state_out_msgs)
            .map(|(access, state)| (access.read(), state.read()))
            .collect()
    }

    /// Expected SEZ position and velocity of the spacecraft from each point,
    /// with the expected ground state, as upstream `_runMappingPoints`.
    struct MappingRun {
        outputs: Vec<(AccessMsg, GroundStateMsg)>,
        expected_r_bl_l: Vec<Vector3<f64>>,
        expected_v_bl_l: Vec<Vector3<f64>>,
        expected_state: Vec<(Vector3<f64>, Vector3<f64>, Vector3<f64>)>,
    }

    /// Upstream `_runMappingPoints`.
    fn run_mapping_points(
        tilt_deg: f64,
        v_pn_n: Vector3<f64>,
        r_pn_n: Vector3<f64>,
        v_rel_p: Vector3<f64>,
    ) -> MappingRun {
        let (dcm_pn, omega_pn_p, planet) = tilted_spinning_planet(tilt_deg, r_pn_n, v_pn_n);
        let omega_pn_n = dcm_pn.transpose() * omega_pn_p;
        let r_bp_p = Vector3::new(RADIUS_M + 600e3, 3.0e5, 1.0e6);
        let r_bp_n = dcm_pn.transpose() * r_bp_p;
        let v_bn_n = v_pn_n + omega_pn_n.cross(&r_bp_n) + dcm_pn.transpose() * v_rel_p;

        let mut points = Vec::new();
        let mut expected_r_bl_l = Vec::new();
        let mut expected_v_bl_l = Vec::new();
        let mut expected_state = Vec::new();
        for [lat_deg, lon_deg] in POINTS_LAT_LON_DEG {
            let (lat, lon) = (lat_deg.to_radians(), lon_deg.to_radians());
            let r_hat_p = unit_direction(lat, lon);
            let r_lp_p = RADIUS_M * r_hat_p;
            points.push(r_lp_p);
            let dcm_lp = truth_dcm_lp(lat, lon);
            expected_r_bl_l.push(dcm_lp * (r_bp_p - r_lp_p));
            // The rotation correction evaluated in P, independent of the
            // module's inertial route.
            let v_bl_p = dcm_pn * (v_bn_n - v_pn_n) - omega_pn_p.cross(&r_bp_p);
            expected_v_bl_l.push(dcm_lp * v_bl_p);
            let r_lp_n = dcm_pn.transpose() * (RADIUS_M * r_hat_p);
            expected_state.push((dcm_pn.transpose() * r_hat_p, r_lp_n, r_pn_n + r_lp_n));
        }
        let outputs = single_step(
            boresight_config(),
            planet,
            SpacecraftStateMsg {
                position_m: r_pn_n + r_bp_n,
                velocity_mps: v_bn_n,
                ..SpacecraftStateMsg::default()
            },
            &points,
        );
        MappingRun {
            outputs,
            expected_r_bl_l,
            expected_v_bl_l,
            expected_state,
        }
    }

    /// Upstream `_assertGroundState`.
    fn assert_ground_state(run: &MappingRun) {
        for ((_, state), (r_hat, r_lp_n, r_ln_n)) in run.outputs.iter().zip(&run.expected_state) {
            assert_allclose(&state.r_hat_lp_n, r_hat, 0.0, 1e-12);
            assert_allclose(&state.position_lp_n_m, r_lp_n, 1e-12, 0.0);
            assert_allclose(&state.position_ln_n_m, r_ln_n, 1e-12, 0.0);
        }
    }

    /// Upstream `test_groundMapping`: with aligned frames and the spacecraft
    /// on the inertial -2 axis, the point on the line to the origin is mapped
    /// and the point on the 3-axis is not, unless the range limit excludes
    /// the first.
    #[test]
    fn maps_only_the_point_in_view() {
        for max_range in [1e-12, 0.001, -1.0] {
            let planet_output = Output::new(PlanetStateMsg {
                orientation: Some(PlanetOrientation::identity()),
                ..PlanetStateMsg::default()
            });
            let spacecraft_output = Output::new(SpacecraftStateMsg {
                position_m: Vector3::new(0.0, -1.0, 0.0),
                ..SpacecraftStateMsg::default()
            });
            let mut mapping = GroundMapping::new(GroundMappingConfig {
                minimum_elevation_rad: 45.0_f64.to_radians(),
                maximum_range_m: (max_range > 0.0).then_some(max_range),
                camera_pos_b_m: Vector3::zeros(),
                n_hat_b: Vector3::new(0.0, 1.0, 0.0),
                half_field_of_view_rad: 22.5_f64.to_radians(),
                ..GroundMappingConfig::default()
            });
            mapping.add_point_to_model(Vector3::new(0.0, -0.1, 0.0));
            mapping.add_point_to_model(Vector3::new(0.0, 0.0, 22.5_f64.to_radians().tan() + 0.1));
            planet_output.connect_to(&mut mapping.planet_in_msg);
            spacecraft_output.connect_to(&mut mapping.sc_state_in_msg);
            let mut logs = [
                MessageRecorder::<AccessMsg>::default(),
                MessageRecorder::default(),
            ];
            for (output, log) in mapping.access_out_msgs.iter().zip(&mut logs) {
                output.connect_to(&mut log.input_msg);
            }
            {
                let mut sim =
                    Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
                sim.add_module("groundMapping", &mut mapping, 500_000_000, 0);
                let [first, second] = &mut logs;
                sim.add_module("log0", first, 500_000_000, 0);
                sim.add_module("log1", second, 500_000_000, 0);
                sim.run_for(1_000_000_000);
            }

            let mapped = logs.map(|log| log.samples.iter().any(|sample| sample.has_access));
            if max_range > 1.0 || max_range < 0.0 {
                assert!(mapped[0], "first point not mapped, max range {max_range}");
            }
            assert!(!mapped[1], "second point mapped, max range {max_range}");
        }
    }

    /// Upstream `test_planetFixedSpacecraftZeroRates`.
    #[test]
    fn planet_fixed_spacecraft_has_zero_rates() {
        for tilt_deg in TILTS_DEG {
            for v_pn_n in PLANET_VELOCITIES_MPS.map(Vector3::from) {
                let run = run_mapping_points(
                    tilt_deg,
                    v_pn_n,
                    Vector3::new(1.5e11, -2.0e10, 5.0e9),
                    Vector3::zeros(),
                );
                for ((access, _), r_exp) in run.outputs.iter().zip(&run.expected_r_bl_l) {
                    assert_allclose(&access.position_bl_l_m, r_exp, 1e-9, 1e-3);
                    assert_allclose(&access.velocity_bl_l_mps, &Vector3::zeros(), 0.0, 1e-6);
                    assert_approx(access.range_dot_mps, 0.0, 1e-6, 1e-6);
                    assert_approx(access.azimuth_dot_radps, 0.0, 1e-6, 1e-11);
                    assert_approx(access.elevation_dot_radps, 0.0, 1e-6, 1e-11);
                }
                assert_ground_state(&run);
            }
        }
    }

    /// Upstream `test_nonzeroRelativeVelocity`: SEZ velocity evaluated in P,
    /// and angle rates against central differences of the SEZ geometry.
    #[test]
    fn moving_spacecraft_rates_match_finite_differences() {
        let v_rel_p = Vector3::new(-1.2e3, 6.5e3, 3.0e3);
        let dt = 1e-3;
        for tilt_deg in TILTS_DEG {
            for v_pn_n in PLANET_VELOCITIES_MPS.map(Vector3::from) {
                let run = run_mapping_points(
                    tilt_deg,
                    v_pn_n,
                    Vector3::new(1.5e11, -2.0e10, 5.0e9),
                    v_rel_p,
                );
                for (((access, _), r_exp), v_exp) in run
                    .outputs
                    .iter()
                    .zip(&run.expected_r_bl_l)
                    .zip(&run.expected_v_bl_l)
                {
                    assert_allclose(&access.position_bl_l_m, r_exp, 1e-9, 1e-3);
                    assert_allclose(&access.velocity_bl_l_mps, v_exp, 1e-9, 1e-6);
                    assert!(v_exp.norm() > 1e3);

                    let (r_plus, r_minus) = (r_exp + v_exp * dt, r_exp - v_exp * dt);
                    let (az_p, el_p) = truth_az_el(&r_plus);
                    let (az_m, el_m) = truth_az_el(&r_minus);
                    assert_approx(
                        access.range_dot_mps,
                        (r_plus.norm() - r_minus.norm()) / (2.0 * dt),
                        1e-6,
                        1e-12,
                    );
                    assert_approx(
                        access.azimuth_dot_radps,
                        (az_p - az_m) / (2.0 * dt),
                        1e-5,
                        1e-10,
                    );
                    assert_approx(
                        access.elevation_dot_radps,
                        (el_p - el_m) / (2.0 * dt),
                        1e-5,
                        1e-10,
                    );
                }
                assert_ground_state(&run);
            }
        }
    }

    /// Upstream `test_overheadSpacecraft`.
    #[test]
    fn overhead_spacecraft_has_zero_azimuth_and_angle_rates() {
        let altitude_m = 600e3;
        let r_pn_n = Vector3::new(1.5e11, -2.0e10, 5.0e9);
        let (lat, lon) = (35.0_f64.to_radians(), (-70.0_f64).to_radians());
        for tilt_deg in TILTS_DEG {
            for v_pn_n in PLANET_VELOCITIES_MPS.map(Vector3::from) {
                let (dcm_pn, omega_pn_p, planet) = tilted_spinning_planet(tilt_deg, r_pn_n, v_pn_n);
                let r_hat_p = unit_direction(lat, lon);
                let r_bp_n = dcm_pn.transpose() * ((RADIUS_M + altitude_m) * r_hat_p);
                let v_bn_n = v_pn_n
                    + (dcm_pn.transpose() * omega_pn_p).cross(&r_bp_n)
                    + dcm_pn.transpose() * Vector3::new(1.0e3, -2.0e3, 5.0e2);
                let outputs = single_step(
                    boresight_config(),
                    planet,
                    SpacecraftStateMsg {
                        position_m: r_pn_n + r_bp_n,
                        velocity_mps: v_bn_n,
                        ..SpacecraftStateMsg::default()
                    },
                    &[RADIUS_M * r_hat_p],
                );
                let (access, _) = &outputs[0];
                assert!(access.velocity_bl_l_mps.iter().all(|v| v.is_finite()));
                assert!(access.position_bl_l_m.x.hypot(access.position_bl_l_m.y) < 1e-3);
                assert_eq!(access.azimuth_rad, 0.0);
                assert_eq!(access.azimuth_dot_radps, 0.0);
                assert_eq!(access.elevation_dot_radps, 0.0);
                assert_approx(access.slant_range_m, altitude_m, 1e-6, 1e-12);
            }
        }
    }

    /// Upstream `Reset`: the boresight must be set.
    #[test]
    #[should_panic(expected = "n_hat_b")]
    fn rejects_an_unset_boresight() {
        GroundMapping::new(GroundMappingConfig::default());
    }
}
