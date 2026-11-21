//! Imaging-strip target access: port of upstream Basilisk `stripLocation`.
//!
//! A target `L` moves at a constant acquisition speed along the great-circle
//! arc (the central line) from a strip's start to its end on a spherical
//! planet `P`, and stops at the end. The module publishes the target's
//! inertial position and velocity ([`StripStateMsg`]) and, for each tracked
//! spacecraft `B`, an [`AccessMsg`] with the slant range, elevation, azimuth,
//! their rates, and the spacecraft position and velocity relative to the
//! target in its south-east-zenith (SEZ) frame. Because the target moves, its
//! SEZ frame rotates relative to the planet, and the relative velocity and
//! rates account for that rotation.
//!
//! A pre-imaging time extends the strip backwards along the same great
//! circle, so that the target reaches the requested start after that time;
//! access is denied until it has elapsed. Otherwise a spacecraft has access
//! when its elevation exceeds `minimum_elevation_rad` and its slant range is
//! within `maximum_range_m`. Illumination and glare are not modelled.
//!
//! The planet input is optional, with the defaults of
//! [`super::ground_location::GroundLocation`]. Upstream reads the `isWritten`
//! flag of every input but never uses the result, so no write-time stand-in
//! is needed.

use std::f64::consts::FRAC_PI_2;

use nalgebra::{Matrix3, Vector3};

use super::access_geometry::{PlanetFrame, azimuth_and_rates, overhead_tolerance_m};
use super::ground_location::SiteLocation;
use crate::messages::{
    AccessMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg, StripStateMsg,
};
use crate::utilities::astro_constants::{D2R, REQ_EARTH};
use crate::utilities::geodetic_conversion::{Lla, dcm_pcpf_to_sez, pcpf_to_lla};
use crate::utilities::linear_algebra::safe_acos;
use crate::{Module, SimulationContext};

/// Upstream's default and fallback `smallAngle` \[rad\].
const DEFAULT_SMALL_ANGLE_RAD: f64 = 1e-12;
/// Below this slant range \[m\] the spacecraft coincides with the target and
/// upstream publishes a fixed no-access message.
const COINCIDENT_RANGE_M: f64 = 1e-12;

/// User-settable parameters of [`StripLocation`]. `Default` is upstream's
/// constructor state, with both strip ends at zero latitude and longitude.
#[derive(Clone, Debug)]
pub struct StripLocationConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `minimumElevation` \[rad\]: elevation above the target's
    /// horizon a spacecraft must exceed to have access; 10 degrees by default.
    pub minimum_elevation_rad: f64,
    /// Upstream `maximumRange` \[m\]: largest slant range with access. `None`
    /// is upstream's `-1`, no limit.
    pub maximum_range_m: Option<f64>,
    /// Upstream `smallAngle` \[rad\]: below this central angle the strip is a
    /// single point. A non-positive value is replaced by 1e-12 at reset, with
    /// a warning, as upstream.
    pub small_angle_rad: f64,
    /// Upstream `planetRadius` \[m\]: radius of the spherical planet; Earth's
    /// equatorial radius by default.
    pub planet_radius_m: f64,
    /// Upstream `r_LP_P_Start`, set through `specifyLocationStart`: start of
    /// the strip, resolved against `planet_radius_m`.
    pub start: SiteLocation,
    /// Upstream `r_LP_P_End`, set through `specifyLocationEnd`: end of the
    /// strip.
    pub end: SiteLocation,
    /// Upstream `acquisitionSpeed` \[m/ns\]: speed of the target along the
    /// central line, in upstream's unit; 3e-6 m/ns (3 km/s) by default. Zero
    /// or negative holds the target at the (updated) start.
    pub acquisition_speed_m_per_ns: f64,
    /// Upstream `preImagingTime` \[ns\]: time added before the target reaches
    /// the start, during which access is denied.
    pub pre_imaging_time_nanos: u64,
}

impl Default for StripLocationConfig {
    fn default() -> Self {
        Self {
            name: "stripLocation".to_string(),
            minimum_elevation_rad: 10.0 * D2R,
            maximum_range_m: None,
            small_angle_rad: DEFAULT_SMALL_ANGLE_RAD,
            planet_radius_m: REQ_EARTH * 1.0e3,
            start: SiteLocation::Geodetic(Lla::default()),
            end: SiteLocation::Geodetic(Lla::default()),
            acquisition_speed_m_per_ns: 3.0e-6,
            pre_imaging_time_nanos: 0,
        }
    }
}

impl StripLocationConfig {
    /// Checks the configuration; upstream's `Reset` raises the radius error.
    pub fn validate(&self) -> Result<(), String> {
        if !(self.planet_radius_m >= 0.0 && self.planet_radius_m.is_finite()) {
            return Err(format!(
                "planet_radius_m must be finite and non-negative, got {}",
                self.planet_radius_m
            ));
        }
        if !self.acquisition_speed_m_per_ns.is_finite() {
            return Err(format!(
                "acquisition_speed_m_per_ns must be finite, got {}",
                self.acquisition_speed_m_per_ns
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

/// Target motion along the strip, upstream's runtime members.
#[derive(Clone, Debug)]
struct StripTrack {
    /// Upstream `pStart` \[m\]: strip start on the planet surface.
    p_start: Vector3<f64>,
    /// Upstream `pEnd` \[m\]: strip end on the planet surface.
    p_end: Vector3<f64>,
    /// Upstream `pStartUpdated` \[m\]: start moved back by the pre-imaging
    /// time.
    p_start_updated: Vector3<f64>,
    /// Upstream `isStartPositionUpdated`.
    is_start_position_updated: bool,
    /// Upstream `durationStripImaging` \[ns\]: time spent on this strip.
    duration_strip_imaging_nanos: u64,
    /// Upstream `OldSimNanos` \[ns\]: time of the previous update.
    old_sim_nanos: u64,
    /// Upstream `r_LP_P` \[m\]: target position, planet-fixed components.
    r_lp_p: Vector3<f64>,
    /// Upstream `v_LP_P` [m/s]: target velocity relative to the planet,
    /// planet-fixed components.
    v_lp_p: Vector3<f64>,
    /// Upstream `dcm_LP`: planet-fixed to SEZ DCM at the target.
    dcm_lp: Matrix3<f64>,
}

impl Default for StripTrack {
    fn default() -> Self {
        Self {
            p_start: Vector3::zeros(),
            p_end: Vector3::zeros(),
            p_start_updated: Vector3::zeros(),
            is_start_position_updated: false,
            duration_strip_imaging_nanos: 0,
            old_sim_nanos: 0,
            r_lp_p: Vector3::zeros(),
            v_lp_p: Vector3::zeros(),
            dcm_lp: Matrix3::identity(),
        }
    }
}

/// Moving strip target visibility of spacecraft: upstream `StripLocation`.
#[derive(Clone, Debug)]
pub struct StripLocation {
    /// User-settable parameters.
    pub config: StripLocationConfig,
    /// Upstream `planetInMsg` (optional): planet state and orientation.
    pub planet_in_msg: Input<PlanetStateMsg>,
    /// Upstream `scStateInMsgs`: one per tracked spacecraft, added by
    /// [`Self::add_spacecraft_to_model`].
    pub sc_state_in_msgs: Vec<Input<SpacecraftStateMsg>>,
    /// Upstream `currentStripStateOutMsg`: inertial state of the target.
    pub current_strip_state_out_msg: Output<StripStateMsg>,
    /// Upstream `accessOutMsgs`: access to each tracked spacecraft, in the
    /// order of `sc_state_in_msgs`.
    pub access_out_msgs: Vec<Output<AccessMsg>>,
    track: StripTrack,
}

/// `std::max(lo, std::min(hi, value))` as upstream writes its clamps: unlike
/// `f64::clamp`, a NaN `value` gives `hi`.
fn std_clamp(value: f64, lo: f64, hi: f64) -> f64 {
    let upper = if value < hi { value } else { hi };
    if lo < upper { upper } else { lo }
}

impl StripLocation {
    /// Builds the module with unconnected ports and no spacecraft.
    ///
    /// # Panics
    ///
    /// If `config` fails [`StripLocationConfig::validate`].
    pub fn new(config: StripLocationConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid StripLocationConfig: {message}");
        }
        Self {
            config,
            planet_in_msg: Input::default(),
            sc_state_in_msgs: Vec::new(),
            current_strip_state_out_msg: Output::default(),
            access_out_msgs: Vec::new(),
            track: StripTrack::default(),
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

    /// Restarts imaging from the configured start: what upstream users do by
    /// zeroing `durationStripImaging` and clearing `isStartPositionUpdated`
    /// before imaging another strip.
    pub fn begin_new_strip(&mut self) {
        self.track.duration_strip_imaging_nanos = 0;
        self.track.is_start_position_updated = false;
    }

    /// Upstream `durationStripImaging` \[ns\]: time spent imaging the current
    /// strip, pre-imaging included.
    pub fn duration_strip_imaging_nanos(&self) -> u64 {
        self.track.duration_strip_imaging_nanos
    }

    /// Upstream `Reset`.
    fn reset_state(&mut self, current_sim_nanos: u64) {
        self.track.is_start_position_updated = false;
        self.track.duration_strip_imaging_nanos = 0;
        self.track.old_sim_nanos = current_sim_nanos;
        if self.config.small_angle_rad <= 0.0 {
            self.config.small_angle_rad = DEFAULT_SMALL_ANGLE_RAD;
            log::warn!(
                "{}: small_angle_rad must be positive; resetting to 1e-12 rad",
                self.config.name
            );
        }
        if let Err(message) = self.config.validate() {
            log::error!("{}: {message}", self.config.name);
        }
    }

    /// Upstream `PositionCentralLine`: the point a fraction `t` along the arc
    /// of central angle `theta` from `p_start` to `p_end` (spherical linear
    /// interpolation), on the planet surface.
    fn position_central_line(
        &self,
        t: f64,
        theta: f64,
        p_start: &Vector3<f64>,
        p_end: &Vector3<f64>,
    ) -> Vector3<f64> {
        if theta.abs() < self.config.small_angle_rad {
            return *p_start;
        }
        let sin_theta = theta.sin();
        let coeff1 = ((1.0 - t) * theta).sin() / sin_theta;
        let coeff2 = (t * theta).sin() / sin_theta;
        (coeff1 * p_start + coeff2 * p_end).normalize() * self.config.planet_radius_m
    }

    /// Upstream `VelocityCentralLine` [m/s]: the arc tangent at fraction `t`,
    /// scaled to the acquisition speed.
    fn velocity_central_line(
        &self,
        t: f64,
        theta: f64,
        p_start: &Vector3<f64>,
        p_end: &Vector3<f64>,
    ) -> Vector3<f64> {
        if theta.abs() < self.config.small_angle_rad {
            return Vector3::zeros();
        }
        let sin_theta = theta.sin();
        let coeff1 = ((1.0 - t) * theta).cos() * (-theta) / sin_theta;
        let coeff2 = (t * theta).cos() * theta / sin_theta;
        let velocity = coeff1 * p_start + coeff2 * p_end;
        velocity / velocity.norm() * (self.config.acquisition_speed_m_per_ns * 1e9)
    }

    /// Upstream `newpstart`: places the strip ends on the surface and moves
    /// the start back along the arc by the pre-imaging time.
    fn new_p_start(&mut self) {
        let radius = self.config.planet_radius_m;
        let r_lp_p_start = self.config.start.resolve(radius).0;
        let r_lp_p_end = self.config.end.resolve(radius).0;
        let p_start = r_lp_p_start.normalize() * radius;
        let p_end = r_lp_p_end.normalize() * radius;
        let theta = std_clamp(p_start.dot(&p_end) / (radius * radius), -1.0, 1.0).acos();
        self.track.p_start = p_start;
        self.track.p_end = p_end;
        self.track.is_start_position_updated = true;

        let speed = self.config.acquisition_speed_m_per_ns;
        if speed <= 0.0 || self.config.pre_imaging_time_nanos == 0 {
            self.track.p_start_updated = p_start;
            return;
        }
        let line_speed_ratio = theta * radius / speed;
        let t = -(self.config.pre_imaging_time_nanos as f64) / line_speed_ratio;
        self.track.p_start_updated = self.position_central_line(t, theta, &p_start, &p_end);
    }

    /// Upstream `updateTargetPositionPCPF`: advances the target along the
    /// central line to `current_sim_nanos`.
    fn update_target_position_pcpf(&mut self, current_sim_nanos: u64) {
        if !self.track.is_start_position_updated {
            self.new_p_start();
        }
        let radius = self.config.planet_radius_m;
        let p_start_updated = self.track.p_start_updated;
        let p_end = self.track.p_end;
        let theta_updated =
            std_clamp(p_start_updated.dot(&p_end) / (radius * radius), -1.0, 1.0).acos();
        let length_central_line_updated = theta_updated * radius;
        self.track.duration_strip_imaging_nanos += current_sim_nanos
            .checked_sub(self.track.old_sim_nanos)
            .unwrap_or_else(|| {
                panic!(
                    "{}: update at {current_sim_nanos} ns precedes the previous one",
                    self.config.name
                )
            });

        let speed = self.config.acquisition_speed_m_per_ns;
        let (r_lp_p, v_lp_p) = if speed <= 0.0 {
            (p_start_updated, Vector3::zeros())
        } else {
            let line_speed_ratio = length_central_line_updated / speed;
            let tau = self.track.duration_strip_imaging_nanos as f64 / line_speed_ratio;
            let tau_clamped = std_clamp(tau, 0.0, 1.0);
            let r_lp_p =
                self.position_central_line(tau_clamped, theta_updated, &p_start_updated, &p_end);
            // At the end the target stays fixed on the planet.
            let v_lp_p = if tau_clamped >= 1.0 {
                Vector3::zeros()
            } else {
                self.velocity_central_line(tau_clamped, theta_updated, &p_start_updated, &p_end)
            };
            (r_lp_p, v_lp_p)
        };
        let lla = pcpf_to_lla(r_lp_p, radius, None);
        self.track.r_lp_p = r_lp_p;
        self.track.v_lp_p = v_lp_p;
        self.track.dcm_lp = dcm_pcpf_to_sez(lla.latitude_rad, lla.longitude_rad);
    }

    /// Upstream `computeAccess` for one spacecraft, given the target's
    /// inertial position `r_lp_n` and surface normal, and the rotation rate
    /// `omega_lp_l` of its SEZ frame relative to the planet.
    fn access(
        &self,
        planet: &PlanetFrame,
        r_lp_n: &Vector3<f64>,
        r_hat_lp_n: &Vector3<f64>,
        omega_lp_l: &Vector3<f64>,
        spacecraft: &SpacecraftStateMsg,
    ) -> AccessMsg {
        let r_bp_n = spacecraft.position_m - planet.r_pn_n;
        let r_bl_n = r_bp_n - r_lp_n;
        let slant_range_m = r_bl_n.norm();
        if slant_range_m < COINCIDENT_RANGE_M {
            return AccessMsg {
                elevation_rad: -FRAC_PI_2,
                ..AccessMsg::default()
            };
        }
        let relative_heading_n = r_bl_n / slant_range_m;
        let elevation_rad = FRAC_PI_2 - safe_acos(r_hat_lp_n.dot(&relative_heading_n));

        let dcm_ln = self.track.dcm_lp * planet.dcm_pn;
        let r_bl_l = dcm_ln * r_bl_n;
        // Planet rotation acts on the planet-fixed components of r_BP.
        let r_bp_p = planet.dcm_pn * r_bp_n;
        let v_bp_p = planet.dcm_pn * (spacecraft.velocity_mps - planet.v_pn_n)
            - planet.omega_pn_p.cross(&r_bp_p);
        let v_bl_l = self.track.dcm_lp * (v_bp_p - self.track.v_lp_p) - omega_lp_l.cross(&r_bl_l);
        let rates = azimuth_and_rates(
            &r_bl_l,
            &v_bl_l,
            overhead_tolerance_m(&planet.r_pn_n, &r_bp_n, r_lp_n),
        );

        let in_range = self
            .config
            .maximum_range_m
            .is_none_or(|range| slant_range_m <= range);
        let pre_imaging_done = self.track.duration_strip_imaging_nanos as f64
            >= self.config.pre_imaging_time_nanos as f64;
        AccessMsg {
            has_access: elevation_rad > self.config.minimum_elevation_rad
                && in_range
                && pre_imaging_done,
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

impl Module for StripLocation {
    fn init(&mut self) {
        self.reset_state(0);
        self.current_strip_state_out_msg
            .write(StripStateMsg::default());
        for output in &self.access_out_msgs {
            output.write(AccessMsg::default());
        }
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.reset_state(context.current_sim_nanos);
    }

    /// Upstream `UpdateState`: `updateTargetPositionPCPF`, then
    /// `updateInertialPosition` and `computeAccess` for every spacecraft.
    fn update(&mut self, context: &SimulationContext) {
        if self.sc_state_in_msgs.is_empty() {
            log::error!(
                "{}: strip location has no spacecraft to track",
                self.config.name
            );
        }
        let planet = PlanetFrame::from_planet_state(&self.planet_in_msg.read());
        self.update_target_position_pcpf(context.current_sim_nanos);

        // Upstream `updateInertialPosition`: target state, with the planet
        // rotation in its inertial velocity.
        let r_lp_p = self.track.r_lp_p;
        let v_lp_p = self.track.v_lp_p;
        let r_lp_n = planet.dcm_pn.transpose() * r_lp_p;
        let r_hat_lp_n = r_lp_n / r_lp_n.norm();
        let v_lp_n = planet.dcm_pn.transpose() * (v_lp_p + planet.omega_pn_p.cross(&r_lp_p));
        self.current_strip_state_out_msg.write(StripStateMsg {
            position_ln_n_m: planet.r_pn_n + r_lp_n,
            position_lp_n_m: r_lp_n,
            velocity_ln_n_mps: planet.v_pn_n + v_lp_n,
            velocity_lp_n_mps: v_lp_n,
        });

        // The target's motion rotates its SEZ axes relative to the planet.
        let target_radius = r_lp_p.norm();
        let horizontal_radius = r_lp_p.x.hypot(r_lp_p.y);
        let v_lp_l = self.track.dcm_lp * v_lp_p;
        let mut omega_lp_l = Vector3::new(-v_lp_l.y / target_radius, v_lp_l.x / target_radius, 0.0);
        // Longitude is undefined at a pole: keep the tilt rate, no zenith spin.
        if horizontal_radius > f64::EPSILON * target_radius {
            omega_lp_l.z = v_lp_l.y * r_lp_p.z / (target_radius * horizontal_radius);
        }

        for (input, output) in self.sc_state_in_msgs.iter().zip(&self.access_out_msgs) {
            output.write(self.access(&planet, &r_lp_n, &r_hat_lp_n, &omega_lp_l, &input.read()));
        }
        self.track.old_sim_nanos = context.current_sim_nanos;
    }
}

#[cfg(test)]
mod tests {
    //! Ports every case of upstream
    //! `stripLocation/_UnitTest/test_unitStripLocation.py` with its
    //! tolerances. `test_AzElR_rates` replaces Basilisk's SPICE Earth and
    //! spacecraft with the ANISE planet ephemeris (IAU Earth orientation) and
    //! the Rust spacecraft, as in the `groundLocation` port.

    use std::f64::consts::PI;
    use std::path::PathBuf;

    use anise::constants::frames::{EARTH_J2000, IAU_EARTH_FRAME};
    use hifitime::Epoch;

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
    /// Upstream `omega`/`spin_rate` [rad/s].
    const SPIN_RATE_RADPS: f64 = 7.2921159e-5;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
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

    fn geodetic_deg(lat_deg: f64, lon_deg: f64, alt_m: f64) -> SiteLocation {
        SiteLocation::Geodetic(Lla::new(lat_deg.to_radians(), lon_deg.to_radians(), alt_m))
    }

    fn fixed_spacecraft(position_m: Vector3<f64>) -> Output<SpacecraftStateMsg> {
        Output::new(SpacecraftStateMsg {
            position_m,
            ..SpacecraftStateMsg::default()
        })
    }

    /// Runs `strip` tracking one spacecraft for `duration_nanos`, recording
    /// the strip state and access every `sample_nanos`.
    fn run_recorded(
        strip: &mut StripLocation,
        spacecraft: &Output<SpacecraftStateMsg>,
        duration_nanos: u64,
        sample_nanos: u64,
    ) -> (Vec<StripStateMsg>, Vec<AccessMsg>) {
        let index = strip.add_spacecraft_to_model();
        spacecraft.connect_to(&mut strip.sc_state_in_msgs[index]);
        let mut state_log = MessageRecorder::<StripStateMsg>::default();
        let mut access_log = MessageRecorder::<AccessMsg>::default();
        strip
            .current_strip_state_out_msg
            .connect_to(&mut state_log.input_msg);
        strip.access_out_msgs[index].connect_to(&mut access_log.input_msg);
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("strip", strip, SECOND_NANOS, 0);
            sim.add_module("stateLog", &mut state_log, sample_nanos, 0);
            sim.add_module("accessLog", &mut access_log, sample_nanos, 0);
            sim.run_for(duration_nanos);
        }
        (state_log.samples, access_log.samples)
    }

    /// The planet of upstream's rate cases: tilted about the inertial 2-axis
    /// and spun by `spin_rate * time` about its own 3-axis, with
    /// `J20002Pfix_dot = -[ω̃] [PN]`.
    fn spun_planet(
        tilt_rad: f64,
        spin_rate: f64,
        time_s: f64,
        r_pn_n: Vector3<f64>,
        v_pn_n: Vector3<f64>,
    ) -> (Matrix3<f64>, PlanetStateMsg) {
        let (s, c) = tilt_rad.sin_cos();
        let tilt_dcm = Matrix3::new(c, 0.0, -s, 0.0, 1.0, 0.0, s, 0.0, c);
        let (sa, ca) = (spin_rate * time_s).sin_cos();
        let spin_dcm = Matrix3::new(ca, sa, 0.0, -sa, ca, 0.0, 0.0, 0.0, 1.0);
        let dcm_pn = spin_dcm * tilt_dcm;
        let omega_tilde = Matrix3::new(0.0, -spin_rate, 0.0, spin_rate, 0.0, 0.0, 0.0, 0.0, 0.0);
        let planet = PlanetStateMsg {
            position_inertial_m: r_pn_n,
            velocity_inertial_mps: v_pn_n,
            orientation: Some(PlanetOrientation {
                inertial_to_fixed: dcm_pn,
                inertial_to_fixed_dot: -omega_tilde * dcm_pn,
            }),
        };
        (dcm_pn, planet)
    }

    /// Upstream `test_range_degenerate_strip`: a strip with equal ends acts
    /// as a single ground location.
    #[test]
    fn degenerate_strip_range_elevation_and_range_limit() {
        let req_m = REQ_EARTH * 1.0e3;
        let mut strip = StripLocation::new(StripLocationConfig {
            planet_radius_m: req_m,
            maximum_range_m: Some(100e3),
            minimum_elevation_rad: 80.0_f64.to_radians(),
            start: geodetic_deg(0.0, 0.0, 0.0),
            end: geodetic_deg(0.0, 0.0, 0.0),
            acquisition_speed_m_per_ns: 0.0,
            ..StripLocationConfig::default()
        });
        let spacecraft = [
            fixed_spacecraft(Vector3::new(req_m + 100e3, 0.0, 0.0)),
            fixed_spacecraft(Vector3::new(req_m + 101e3, 0.0, 0.0)),
            fixed_spacecraft(
                euler3(11.0_f64.to_radians()) * Vector3::new(100e3, 0.0, 0.0)
                    + Vector3::new(req_m, 0.0, 0.0),
            ),
        ];
        let mut recorders: Vec<MessageRecorder<AccessMsg>> =
            (0..3).map(|_| MessageRecorder::default()).collect();
        for (output, recorder) in spacecraft.iter().zip(&mut recorders) {
            let index = strip.add_spacecraft_to_model();
            output.connect_to(&mut strip.sc_state_in_msgs[index]);
            strip.access_out_msgs[index].connect_to(&mut recorder.input_msg);
        }
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("stripTarget", &mut strip, SECOND_NANOS, 0);
            for (index, recorder) in recorders.iter_mut().enumerate() {
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
        let reference = [
            (100e3, 90.0_f64, true),
            (101e3, 90.0, false),
            (100e3, 79.0, false),
        ];
        for (recorder, (range, elevation_deg, access)) in recorders.iter().zip(reference) {
            let sample = &recorder.samples[1];
            assert_approx(sample.slant_range_m, range, 1e-8, 1e-12);
            assert_approx(
                sample.elevation_rad,
                elevation_deg.to_radians(),
                1e-8,
                1e-12,
            );
            assert_eq!(sample.has_access, access);
        }
    }

    /// Upstream `test_rotation_degenerate_strip`.
    #[test]
    fn degenerate_strip_follows_planet_rotation() {
        let req_m = REQ_EARTH * 1.0e3;
        let mut strip = StripLocation::new(StripLocationConfig {
            planet_radius_m: req_m,
            maximum_range_m: Some(200e3),
            minimum_elevation_rad: 10.0_f64.to_radians(),
            start: geodetic_deg(0.0, 10.0, 0.0),
            end: geodetic_deg(0.0, 10.0, 0.0),
            acquisition_speed_m_per_ns: 0.0,
            ..StripLocationConfig::default()
        });
        let planet = Output::new(PlanetStateMsg {
            orientation: Some(PlanetOrientation {
                inertial_to_fixed: euler3((-10.0_f64).to_radians()),
                inertial_to_fixed_dot: Matrix3::zeros(),
            }),
            ..PlanetStateMsg::default()
        });
        planet.connect_to(&mut strip.planet_in_msg);
        let spacecraft = fixed_spacecraft(Vector3::new(req_m + 90e3, 0.0, 0.0));
        let (_, access) = run_recorded(&mut strip, &spacecraft, SECOND_NANOS, SECOND_NANOS);
        let sample = &access[1];
        assert_approx(sample.slant_range_m, 90e3, 1e-8, 1e-12);
        assert_approx(sample.elevation_rad, FRAC_PI_2, 1e-8, 1e-12);
        assert!(sample.has_access);
    }

    /// Upstream `test_strip_target_motion`: the target follows the
    /// great-circle arc from start to end, passing under a spacecraft at the
    /// midpoint halfway through.
    #[test]
    fn target_moves_along_the_central_line() {
        let radius = REQ_EARTH * 1.0e3;
        let total_s = 10.0;
        let strip_length = (PI / 2.0) * radius;
        let mut strip = StripLocation::new(StripLocationConfig {
            planet_radius_m: radius,
            maximum_range_m: None,
            minimum_elevation_rad: 10.0_f64.to_radians(),
            start: geodetic_deg(0.0, 0.0, 0.0),
            end: geodetic_deg(0.0, 90.0, 0.0),
            acquisition_speed_m_per_ns: strip_length / (total_s * 1e9),
            pre_imaging_time_nanos: 0,
            ..StripLocationConfig::default()
        });
        let midpoint_dir = Vector3::new(
            45.0_f64.to_radians().cos(),
            45.0_f64.to_radians().sin(),
            0.0,
        );
        let spacecraft = fixed_spacecraft((radius + 200e3) * midpoint_dir);
        let (states, access) =
            run_recorded(&mut strip, &spacecraft, 10 * SECOND_NANOS, SECOND_NANOS);

        let position_accuracy = 1.0;
        assert!(
            (states[0].position_lp_n_m - Vector3::new(radius, 0.0, 0.0)).norm() < position_accuracy
        );
        assert!((states[5].position_lp_n_m - radius * midpoint_dir).norm() < position_accuracy);
        assert!(
            (states[10].position_lp_n_m - Vector3::new(0.0, radius, 0.0)).norm()
                < position_accuracy
        );
        assert_approx(access[5].elevation_rad, FRAC_PI_2, 1e-6, 0.01);
        assert!(access[5].has_access);
    }

    /// Upstream `test_pre_imaging_blocks_access`.
    #[test]
    fn pre_imaging_blocks_access() {
        let radius = REQ_EARTH * 1.0e3;
        let mut strip = StripLocation::new(StripLocationConfig {
            planet_radius_m: radius,
            maximum_range_m: None,
            minimum_elevation_rad: 10.0_f64.to_radians(),
            start: geodetic_deg(0.0, 0.0, 0.0),
            end: geodetic_deg(0.0, 0.0, 0.0),
            acquisition_speed_m_per_ns: 0.0,
            pre_imaging_time_nanos: 5 * SECOND_NANOS,
            ..StripLocationConfig::default()
        });
        let spacecraft = fixed_spacecraft(Vector3::new(radius + 100e3, 0.0, 0.0));
        let (_, access) = run_recorded(&mut strip, &spacecraft, 10 * SECOND_NANOS, SECOND_NANOS);
        for (second, sample) in access.iter().enumerate() {
            assert_eq!(sample.has_access, second >= 5, "t = {second} s");
        }
        assert_eq!(access.len(), 11);
    }

    /// Upstream `test_strip_velocity_output`: the target moves at the
    /// acquisition speed, tangent to the sphere, in the strip's great-circle
    /// plane, from start toward end.
    #[test]
    fn target_velocity_is_the_acquisition_speed_along_the_arc() {
        let radius = REQ_EARTH * 1.0e3;
        let speed = (PI / 2.0) * radius / 10e9;
        let mut strip = StripLocation::new(StripLocationConfig {
            planet_radius_m: radius,
            maximum_range_m: None,
            minimum_elevation_rad: 10.0_f64.to_radians(),
            start: geodetic_deg(0.0, 0.0, 0.0),
            end: geodetic_deg(0.0, 90.0, 0.0),
            acquisition_speed_m_per_ns: speed,
            pre_imaging_time_nanos: 0,
            ..StripLocationConfig::default()
        });
        let spacecraft = fixed_spacecraft(Vector3::new(radius + 200e3, 0.0, 0.0));
        let (states, _) = run_recorded(&mut strip, &spacecraft, 10 * SECOND_NANOS, SECOND_NANOS);

        let orbit_normal = Vector3::new(radius, 0.0, 0.0)
            .cross(&Vector3::new(0.0, radius, 0.0))
            .normalize();
        for index in [1, 3, 5, 7, 9] {
            let v = states[index].velocity_lp_n_mps;
            let r = states[index].position_lp_n_m;
            assert_approx(v.norm(), speed * 1e9, 1e-6, 1e-12);
            assert!((r.dot(&v) / (r.norm() * v.norm())).abs() < 1e-6);
            assert!((v.dot(&orbit_normal) / v.norm()).abs() < 1e-6);
            assert!(r.cross(&v).normalize().dot(&orbit_normal) > 0.99);
        }
    }

    /// One update at time zero, as upstream's `SingleStepProcesses`.
    fn single_step(
        config: StripLocationConfig,
        planet: PlanetStateMsg,
        spacecraft: SpacecraftStateMsg,
    ) -> AccessMsg {
        let planet_output = Output::new(planet);
        let spacecraft_output = Output::new(spacecraft);
        let mut strip = StripLocation::new(config);
        let index = strip.add_spacecraft_to_model();
        planet_output.connect_to(&mut strip.planet_in_msg);
        spacecraft_output.connect_to(&mut strip.sc_state_in_msgs[index]);
        strip.init();
        strip.update(&context(0));
        strip.access_out_msgs[index].read()
    }

    fn stationary_config(start: SiteLocation) -> StripLocationConfig {
        StripLocationConfig {
            planet_radius_m: RADIUS_M,
            minimum_elevation_rad: -FRAC_PI_2,
            maximum_range_m: None,
            acquisition_speed_m_per_ns: 0.0,
            pre_imaging_time_nanos: 0,
            start,
            end: start,
            ..StripLocationConfig::default()
        }
    }

    /// Upstream `test_planetFixedSpacecraftZeroRates`.
    #[test]
    fn planet_fixed_spacecraft_has_zero_rates() {
        for tilt_deg in [0.0_f64, 30.0, 60.0] {
            let (s, c) = tilt_deg.to_radians().sin_cos();
            let dcm_pn = Matrix3::new(c, 0.0, -s, 0.0, 1.0, 0.0, s, 0.0, c);
            let omega_pn_p = Vector3::new(0.0, 0.0, SPIN_RATE_RADPS);
            let omega_pn_n = dcm_pn.transpose() * omega_pn_p;
            let planet = PlanetStateMsg {
                orientation: Some(PlanetOrientation {
                    inertial_to_fixed: dcm_pn,
                    inertial_to_fixed_dot: -tilde(omega_pn_p) * dcm_pn,
                }),
                ..PlanetStateMsg::default()
            };
            let r_bn_n = Vector3::new(RADIUS_M + 600e3, 0.0, 1.0e6);
            let access = single_step(
                stationary_config(geodetic_deg(10.0, 20.0, 0.0)),
                planet,
                SpacecraftStateMsg {
                    position_m: r_bn_n,
                    velocity_mps: omega_pn_n.cross(&r_bn_n),
                    ..SpacecraftStateMsg::default()
                },
            );
            assert_allclose(&access.velocity_bl_l_mps, &Vector3::zeros(), 0.0, 1e-6);
            assert_approx(access.range_dot_mps, 0.0, 1e-6, 1e-6);
            assert_approx(access.azimuth_dot_radps, 0.0, 1e-6, 1e-11);
            assert_approx(access.elevation_dot_radps, 0.0, 1e-6, 1e-11);
        }
    }

    /// Upstream `test_nonzeroRelativeVelocityOutput`.
    #[test]
    fn publishes_relative_velocity_consistent_with_range_rate() {
        let access = single_step(
            stationary_config(geodetic_deg(0.0, 0.0, 0.0)),
            PlanetStateMsg {
                orientation: Some(PlanetOrientation::identity()),
                ..PlanetStateMsg::default()
            },
            SpacecraftStateMsg {
                position_m: Vector3::new(RADIUS_M + 600e3, 1.0e5, 2.0e5),
                velocity_mps: Vector3::new(10.0, 20.0, 30.0),
                ..SpacecraftStateMsg::default()
            },
        );
        let v_bl_l_expected = Vector3::new(-30.0, 20.0, 10.0);
        let r_bl_l = access.position_bl_l_m;
        assert_allclose(&access.velocity_bl_l_mps, &v_bl_l_expected, 0.0, 1e-9);
        assert_approx(
            access.range_dot_mps,
            v_bl_l_expected.dot(&r_bl_l) / r_bl_l.norm(),
            1e-9,
            1e-12,
        );
        assert_approx(access.range_dot_mps, 21.8643266644, 1e-8, 1e-12);
    }

    /// Upstream `test_movingStripCoordinateRates`: on a moving strip, the
    /// published SEZ velocity and coordinate rates match central differences
    /// of the published geometry.
    #[test]
    fn moving_strip_rates_match_central_differences() {
        let cases = [
            ([0.0, 0.0], [0.0, 90.0]),
            ([20.0, 10.0], [55.0, 75.0]),
            ([10.0, 20.0], [55.0, 20.0]),
        ];
        let sample_nanos = [9_990_000_000_u64, 10_000_000_000, 10_010_000_000];
        let sample_s = [9.99, 10.0, 10.01];
        let time_step = 0.01;
        for (start_deg, end_deg) in cases {
            for moving_planet in [false, true] {
                let spin_rate = if moving_planet { SPIN_RATE_RADPS } else { 0.0 };
                let tilt = if moving_planet {
                    30.0_f64.to_radians()
                } else {
                    0.0
                };
                let r_pn_n_initial = if moving_planet {
                    Vector3::new(1.5e8, -2.0e7, 5.0e6)
                } else {
                    Vector3::zeros()
                };
                let v_pn_n = if moving_planet {
                    Vector3::new(3.0e4, -1.0e3, 2.0e3)
                } else {
                    Vector3::zeros()
                };
                let r_bp_p_initial = Vector3::new(RADIUS_M + 600e3, 1.0e5, 2.0e5);
                let v_bp_p = if moving_planet {
                    Vector3::new(500.0, -1000.0, 250.0)
                } else {
                    Vector3::zeros()
                };
                let omega_pn_p = Vector3::new(0.0, 0.0, spin_rate);

                let planet_output = Output::new(PlanetStateMsg::default());
                let spacecraft_output = Output::new(SpacecraftStateMsg::default());
                let mut strip = StripLocation::new(StripLocationConfig {
                    planet_radius_m: RADIUS_M,
                    acquisition_speed_m_per_ns: 3.0e-6,
                    start: geodetic_deg(start_deg[0], start_deg[1], 0.0),
                    end: geodetic_deg(end_deg[0], end_deg[1], 0.0),
                    ..StripLocationConfig::default()
                });
                planet_output.connect_to(&mut strip.planet_in_msg);
                let index = strip.add_spacecraft_to_model();
                spacecraft_output.connect_to(&mut strip.sc_state_in_msgs[index]);
                strip.init();
                strip.reset(&context(0));

                let mut positions = Vec::new();
                let mut coordinates = Vec::new();
                let mut velocities = Vec::new();
                let mut coordinate_rates = Vec::new();
                for (time_s, time_nanos) in sample_s.into_iter().zip(sample_nanos) {
                    let r_pn_n = r_pn_n_initial + v_pn_n * time_s;
                    let (dcm_pn, planet) = spun_planet(tilt, spin_rate, time_s, r_pn_n, v_pn_n);
                    planet_output.write(planet);
                    let r_bp_p = r_bp_p_initial + v_bp_p * time_s;
                    spacecraft_output.write(SpacecraftStateMsg {
                        position_m: r_pn_n + dcm_pn.transpose() * r_bp_p,
                        velocity_mps: v_pn_n
                            + dcm_pn.transpose() * (v_bp_p + omega_pn_p.cross(&r_bp_p)),
                        ..SpacecraftStateMsg::default()
                    });
                    strip.update(&context(time_nanos));
                    let output = strip.access_out_msgs[index].read();
                    positions.push(output.position_bl_l_m);
                    coordinates.push([
                        output.slant_range_m,
                        output.azimuth_rad,
                        output.elevation_rad,
                    ]);
                    velocities.push(output.velocity_bl_l_mps);
                    coordinate_rates.push([
                        output.range_dot_mps,
                        output.azimuth_dot_radps,
                        output.elevation_dot_radps,
                    ]);
                }

                // `np.unwrap` of the azimuth.
                for k in 1..3 {
                    let jump = coordinates[k][1] - coordinates[k - 1][1];
                    coordinates[k][1] -= (2.0 * PI) * (jump / (2.0 * PI)).round();
                }
                let expected_velocity = (positions[2] - positions[0]) / (2.0 * time_step);
                let expected_rates: Vec<f64> = (0..3)
                    .map(|k| (coordinates[2][k] - coordinates[0][k]) / (2.0 * time_step))
                    .collect();
                assert_allclose(&velocities[1], &expected_velocity, 1e-7, 5e-5);
                assert_approx(coordinate_rates[1][0], expected_rates[0], 1e-7, 5e-5);
                for k in 1..3 {
                    let (actual, expected) = (coordinate_rates[1][k], expected_rates[k]);
                    assert!(
                        (actual - expected).abs() <= 1e-9 + 1e-7 * expected.abs(),
                        "{start_deg:?}->{end_deg:?}, moving {moving_planet}: rate {k} {actual} vs {expected}"
                    );
                }
            }
        }
    }

    /// Upstream `test_completedStripStops`: once complete, the target stays
    /// at the strip end, fixed on the planet, with zero relative velocity and
    /// coordinate rates for a planet-fixed spacecraft.
    #[test]
    fn completed_strip_stops_at_the_end() {
        let traversal_s = 10.0;
        for pre_imaging_s in [0.0_f64, 2.0] {
            for moving_planet in [false, true] {
                let completion_s = traversal_s + pre_imaging_s;
                let spin_rate = if moving_planet { SPIN_RATE_RADPS } else { 0.0 };
                let tilt = if moving_planet {
                    30.0_f64.to_radians()
                } else {
                    0.0
                };
                let r_pn_n_initial = if moving_planet {
                    Vector3::new(1.5e8, -2.0e7, 5.0e6)
                } else {
                    Vector3::zeros()
                };
                let v_pn_n = if moving_planet {
                    Vector3::new(3.0e4, -1.0e3, 2.0e3)
                } else {
                    Vector3::zeros()
                };
                let r_bp_p = Vector3::new(RADIUS_M + 600e3, 1.0e5, 2.0e5);
                let r_end_p = Vector3::new(0.0, RADIUS_M, 0.0);
                let omega_pn_p = Vector3::new(0.0, 0.0, spin_rate);

                let planet_output = Output::new(PlanetStateMsg::default());
                let spacecraft_output = Output::new(SpacecraftStateMsg::default());
                let mut strip = StripLocation::new(StripLocationConfig {
                    planet_radius_m: RADIUS_M,
                    acquisition_speed_m_per_ns: RADIUS_M * PI / 2.0 / (traversal_s * 1e9),
                    pre_imaging_time_nanos: (pre_imaging_s * 1e9) as u64,
                    start: SiteLocation::Geodetic(Lla::new(0.0, 0.0, 0.0)),
                    end: SiteLocation::Geodetic(Lla::new(0.0, PI / 2.0, 0.0)),
                    ..StripLocationConfig::default()
                });
                planet_output.connect_to(&mut strip.planet_in_msg);
                let index = strip.add_spacecraft_to_model();
                spacecraft_output.connect_to(&mut strip.sc_state_in_msgs[index]);
                strip.init();
                strip.reset(&context(0));

                let first_stopped_s = if pre_imaging_s == 0.0 {
                    completion_s
                } else {
                    completion_s + 1.0
                };
                let mut stopped_positions = Vec::new();
                let mut stopped_coordinates = Vec::new();
                for time_s in [completion_s - 1.0, first_stopped_s, completion_s + 2.0] {
                    let r_pn_n = r_pn_n_initial + v_pn_n * time_s;
                    let (dcm_pn, planet) = spun_planet(tilt, spin_rate, time_s, r_pn_n, v_pn_n);
                    planet_output.write(planet);
                    spacecraft_output.write(SpacecraftStateMsg {
                        position_m: r_pn_n + dcm_pn.transpose() * r_bp_p,
                        velocity_mps: v_pn_n + dcm_pn.transpose() * omega_pn_p.cross(&r_bp_p),
                        ..SpacecraftStateMsg::default()
                    });
                    strip.update(&context((time_s * 1e9) as u64));

                    let state = strip.current_strip_state_out_msg.read();
                    let r_lp_p = dcm_pn * state.position_lp_n_m;
                    if time_s < completion_s {
                        let v_lp_p = dcm_pn * state.velocity_lp_n_mps - omega_pn_p.cross(&r_lp_p);
                        assert_approx(
                            v_lp_p.norm(),
                            strip.config.acquisition_speed_m_per_ns * 1e9,
                            1e-12,
                            1e-12,
                        );
                        assert!((r_lp_p - r_end_p).norm() > 1.0);
                        continue;
                    }

                    let expected_v_lp_n = dcm_pn.transpose() * omega_pn_p.cross(&r_end_p);
                    assert_allclose(&r_lp_p, &r_end_p, 0.0, 1e-7);
                    assert_allclose(&state.velocity_lp_n_mps, &expected_v_lp_n, 0.0, 1e-8);
                    assert_allclose(
                        &state.velocity_ln_n_mps,
                        &(v_pn_n + expected_v_lp_n),
                        0.0,
                        1e-8,
                    );
                    let access = strip.access_out_msgs[index].read();
                    assert_allclose(&access.velocity_bl_l_mps, &Vector3::zeros(), 0.0, 1e-8);
                    assert_approx(access.range_dot_mps, 0.0, 1e-6, 1e-8);
                    assert!(access.azimuth_dot_radps.abs() <= 1e-12);
                    assert!(access.elevation_dot_radps.abs() <= 1e-12);
                    stopped_positions.push(access.position_bl_l_m);
                    stopped_coordinates.push([
                        access.slant_range_m,
                        access.azimuth_rad,
                        access.elevation_rad,
                    ]);
                }

                assert_allclose(&stopped_positions[1], &stopped_positions[0], 0.0, 1e-7);
                assert!((stopped_coordinates[1][0] - stopped_coordinates[0][0]).abs() <= 1e-7);
                for (later, first) in stopped_coordinates[1][1..]
                    .iter()
                    .zip(&stopped_coordinates[0][1..])
                {
                    assert!((later - first).abs() <= 1e-12);
                }
            }
        }
    }

    /// Upstream `test_new_start_pre_imaging`: the start moves back along the
    /// arc so the target reaches the original start after the pre-imaging
    /// time, and the original end at the end of the extended strip.
    #[test]
    fn pre_imaging_extends_the_strip_backwards() {
        let radius = REQ_EARTH * 1.0e3;
        let theta_orig = 90.0_f64.to_radians();
        let speed = theta_orig * radius / 10e9;
        let pre_imaging_nanos = 2 * SECOND_NANOS;
        let angle_behind = speed * pre_imaging_nanos as f64 / radius;
        let expected_new_start =
            radius * Vector3::new((-angle_behind).cos(), (-angle_behind).sin(), 0.0);
        let total_updated_s = (theta_orig + angle_behind) * radius / (speed * 1e9);
        let end_index = total_updated_s.ceil() as usize;

        let mut strip = StripLocation::new(StripLocationConfig {
            name: "stripTargetNewStart".to_string(),
            planet_radius_m: radius,
            maximum_range_m: None,
            minimum_elevation_rad: 10.0_f64.to_radians(),
            start: geodetic_deg(0.0, 0.0, 0.0),
            end: geodetic_deg(0.0, 90.0, 0.0),
            acquisition_speed_m_per_ns: speed,
            pre_imaging_time_nanos: pre_imaging_nanos,
            ..StripLocationConfig::default()
        });
        let spacecraft = fixed_spacecraft(Vector3::new(radius + 200e3, 0.0, 0.0));
        let (states, _) = run_recorded(
            &mut strip,
            &spacecraft,
            end_index as u64 * SECOND_NANOS,
            SECOND_NANOS,
        );

        let original_start = Vector3::new(radius, 0.0, 0.0);
        assert!((states[0].position_lp_n_m - expected_new_start).norm() < 1.0);
        assert!((states[2].position_lp_n_m - original_start).norm() < 1.0);
        assert!((states[end_index].position_lp_n_m - Vector3::new(0.0, radius, 0.0)).norm() < 1.0);
        let new_start = states[0].position_lp_n_m;
        let angle = (new_start.dot(&original_start) / (new_start.norm() * original_start.norm()))
            .clamp(-1.0, 1.0)
            .acos();
        assert!((angle - angle_behind).abs() <= 1e-6);
        assert_eq!(
            strip.duration_strip_imaging_nanos(),
            end_index as u64 * SECOND_NANOS
        );
    }

    /// Upstream `test_AzElR_rates` for a stationary strip at Boulder.
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

        let boulder = geodetic_deg(40.009971, -105.243895, 1624.0);
        let mut strip = StripLocation::new(StripLocationConfig {
            planet_radius_m: rad_equator_m,
            start: boulder,
            end: boulder,
            acquisition_speed_m_per_ns: 0.0,
            minimum_elevation_rad: 60.0_f64.to_radians(),
            ..StripLocationConfig::default()
        });
        let index = strip.add_spacecraft_to_model();
        let mut recorder = MessageRecorder::<AccessMsg>::default();
        {
            let epoch = Epoch::from_gregorian_utc(2021, 5, 4, 6, 47, 48, 965_000_000);
            let mut sim = Simulation::new(epoch, false);
            sim.connect(&earth.output_planet_msg, &mut strip.planet_in_msg);
            sim.connect(&spacecraft.state_out, &mut strip.sc_state_in_msgs[index]);
            sim.connect(&strip.access_out_msgs[index], &mut recorder.input_msg);
            sim.add_module("earth", &mut earth, SECOND_NANOS, 0);
            sim.add_module("spacecraft", &mut spacecraft, SECOND_NANOS, 0);
            sim.add_module("stripStation", &mut strip, SECOND_NANOS, 0);
            sim.add_module("log", &mut recorder, SECOND_NANOS, 0);
            sim.run_for(SECOND_NANOS);
        }
        let [first, second] = &recorder.samples[..] else {
            panic!("expected two access samples");
        };
        let dt_s = 1.0;
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

    /// Not an upstream case: a spacecraft at the target gets upstream's
    /// fixed no-access message.
    #[test]
    fn coincident_spacecraft_has_no_access() {
        let access = single_step(
            stationary_config(geodetic_deg(0.0, 0.0, 0.0)),
            PlanetStateMsg::default(),
            SpacecraftStateMsg {
                position_m: Vector3::new(RADIUS_M, 0.0, 0.0),
                ..SpacecraftStateMsg::default()
            },
        );
        assert_eq!(
            access,
            AccessMsg {
                elevation_rad: -FRAC_PI_2,
                ..AccessMsg::default()
            }
        );
    }

    /// Not an upstream case: upstream `Reset` replaces a non-positive
    /// `smallAngle` with 1e-12 rad, and a new strip restarts from its start.
    #[test]
    fn reset_restores_small_angle_and_a_new_strip_restarts() {
        let mut strip = StripLocation::new(StripLocationConfig {
            small_angle_rad: 0.0,
            ..StripLocationConfig::default()
        });
        strip.add_spacecraft_to_model();
        strip.init();
        assert_eq!(strip.config.small_angle_rad, 1e-12);
        strip.update(&context(0));
        strip.update(&context(4 * SECOND_NANOS));
        assert_eq!(strip.duration_strip_imaging_nanos(), 4 * SECOND_NANOS);
        strip.begin_new_strip();
        assert_eq!(strip.duration_strip_imaging_nanos(), 0);
        strip.update(&context(5 * SECOND_NANOS));
        assert_eq!(strip.duration_strip_imaging_nanos(), SECOND_NANOS);
    }
}
