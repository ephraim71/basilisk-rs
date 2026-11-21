//! Landmark pixels from a pinhole camera: port of upstream Basilisk
//! `pinholeCamera`.
//!
//! Pure geometry, no image: each landmark on a planet's surface (a position
//! and a surface normal, both in the planet-fixed frame `P`) is projected
//! through an ideal pinhole camera mounted on the spacecraft, and published on
//! its own output, upstream `landmarkOutMsgs` ([`LandmarkMsg`]): whether it is
//! visible and, if so, its pixel. Landmarks are registered with
//! [`PinholeCamera::add_landmark`] (upstream `addLandmark`), which creates
//! their output.
//!
//! Inputs are the planet ephemeris (upstream `ephemerisInMsg`,
//! [`EphemerisMsg`], whose attitude is the planet-fixed frame `[PN]`) and the
//! spacecraft state (upstream `scStateInMsg`, [`SpacecraftStateMsg`]). The Sun
//! is taken to sit at the inertial origin, so the planet-to-Sun direction is
//! `-r_PN`.
//!
//! The camera's focal direction is its `+z` axis. With `Δr_C = [CB][BP](r_LP
//! − r_BP)`, a landmark's pixel is
//! `(f Δr_x / (Δr_z w), f Δr_y / (Δr_z w))`, rounded away from zero to whole
//! pixels; a coordinate of exactly zero maps to pixel `1` (upstream's
//! convention). A landmark is visible when all of these hold:
//!
//! * it is in front of the camera and its normal faces the camera (both
//!   angles to the focal plane strictly positive);
//! * its pixel lies within the image, `|2 p_x| <= nx` and `|2 p_y| <= ny`;
//! * the Sun's elevation over its surface, `π/2 − ∠(n, ŝ)`, exceeds
//!   `mask_sun_rad`. Upstream's default `-π/2` lights every landmark whose
//!   normal is not exactly anti-sunward.
//!
//! An invisible landmark publishes pixel `(0, 0)`.
//!
//! [`PinholeCamera::process_batch`] (upstream `processBatch`) runs the same
//! test on a batch of spacecraft states given in the planet frame, without
//! messages; upstream's progress printing is not ported.

use std::f64::consts::FRAC_PI_2;

use nalgebra::{Matrix3, Vector3};

use crate::messages::{EphemerisMsg, Input, LandmarkMsg, Output, SpacecraftStateMsg};
use crate::utilities::linear_algebra::safe_acos;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Optics and mounting of [`PinholeCamera`].
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PinholeCameraConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `f` \[m\]: focal length.
    pub focal_length_m: f64,
    /// Upstream `nxPixel` [-]: number of horizontal pixels.
    pub nx_pixel: f64,
    /// Upstream `nyPixel` [-]: number of vertical pixels.
    pub ny_pixel: f64,
    /// Upstream `wPixel` \[m\]: pixel width.
    pub pixel_width_m: f64,
    /// Upstream `dcm_CB` [-]: DCM from the body frame to the camera frame.
    pub dcm_cb: Matrix3<f64>,
    /// Upstream `maskSun` \[rad\]: minimum Sun elevation over a landmark's
    /// surface for it to count as lit.
    pub mask_sun_rad: f64,
}

impl Default for PinholeCameraConfig {
    /// Upstream's constructor sets only the Sun mask (`-π/2`, no lighting
    /// constraint); the optics here are zero and must be set before use.
    fn default() -> Self {
        Self {
            name: String::new(),
            focal_length_m: 0.0,
            nx_pixel: 0.0,
            ny_pixel: 0.0,
            pixel_width_m: 0.0,
            dcm_cb: Matrix3::identity(),
            mask_sun_rad: -FRAC_PI_2,
        }
    }
}

impl PinholeCameraConfig {
    /// Check numeric invariants. Returns a description of the first violation,
    /// or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        for (label, value) in [
            ("focal_length_m", self.focal_length_m),
            ("nx_pixel", self.nx_pixel),
            ("ny_pixel", self.ny_pixel),
            ("pixel_width_m", self.pixel_width_m),
        ] {
            if !(value.is_finite() && value > 0.0) {
                return Err(format!("{label} must be positive and finite, got {value}"));
            }
        }
        if !self.mask_sun_rad.is_finite() {
            return Err(format!(
                "mask_sun_rad must be finite, got {}",
                self.mask_sun_rad
            ));
        }
        if !self.dcm_cb.iter().all(|value| value.is_finite()) {
            return Err(format!("dcm_cb must be finite, got {:?}", self.dcm_cb));
        }
        Ok(())
    }

    /// Upstream `FOVx` \[rad\]: horizontal field of view.
    pub fn fov_x_rad(&self) -> f64 {
        2.0 * (self.nx_pixel * self.pixel_width_m).atan2(2.0 * self.focal_length_m)
    }

    /// Upstream `FOVy` \[rad\]: vertical field of view.
    pub fn fov_y_rad(&self) -> f64 {
        2.0 * (self.ny_pixel * self.pixel_width_m).atan2(2.0 * self.focal_length_m)
    }
}

/// A surface landmark, planet-fixed frame `P`.
#[derive(Clone, Debug, PartialEq)]
pub struct Landmark {
    /// Upstream `r_LP_P` \[m\]: position relative to the planet center.
    pub position_p_m: Vector3<f64>,
    /// Upstream `nL_P` [-]: surface normal (need not be unit length).
    pub normal_p: Vector3<f64>,
}

/// Spacecraft, camera and Sun geometry in the planet frame for one sample.
struct CameraView {
    /// Upstream `r_BP_P` \[m\]: spacecraft position relative to the planet.
    r_bp_p: Vector3<f64>,
    /// Upstream `dcm_BP`: planet frame to body frame.
    dcm_bp: Matrix3<f64>,
    /// Upstream `e_SP_P`: planet-to-Sun direction.
    e_sp_p: Vector3<f64>,
    /// Upstream `eC_P`: camera focal direction.
    ec_p: Vector3<f64>,
}

impl CameraView {
    fn new(
        config: &PinholeCameraConfig,
        r_bp_p: Vector3<f64>,
        dcm_bp: Matrix3<f64>,
        e_sp_p: Vector3<f64>,
    ) -> Self {
        // Upstream `eC_C`: the focal direction is the camera +z axis.
        let ec_p = (config.dcm_cb * dcm_bp).transpose() * Vector3::z();
        Self {
            r_bp_p,
            dcm_bp,
            e_sp_p,
            ec_p,
        }
    }

    /// Upstream `processInputs`.
    fn from_messages(
        config: &PinholeCameraConfig,
        planet: &EphemerisMsg,
        spacecraft: &SpacecraftStateMsg,
    ) -> Self {
        let dcm_pn = mrp_to_dcm(planet.sigma_bn);
        let r_pn_n = planet.position_bdy_zero_n_m;
        let e_sp_p = dcm_pn * (-r_pn_n / r_pn_n.norm());
        let r_bp_p = dcm_pn * (spacecraft.position_m - r_pn_n);
        let dcm_bp = mrp_to_dcm(spacecraft.sigma_bn) * dcm_pn.transpose();
        Self::new(config, r_bp_p, dcm_bp, e_sp_p)
    }

    /// Upstream `computePixel`.
    fn pixel(&self, config: &PinholeCameraConfig, r_lmk: Vector3<f64>) -> [i32; 2] {
        let dr = config.dcm_cb * self.dcm_bp * (r_lmk - self.r_bp_p);
        let u = config.focal_length_m * dr.x / (dr.z * config.pixel_width_m);
        let v = config.focal_length_m * dr.y / (dr.z * config.pixel_width_m);
        [pixelize(u), pixelize(v)]
    }

    /// Upstream `checkFOV`: in front, facing the camera, and inside the image.
    fn in_field_of_view(
        &self,
        config: &PinholeCameraConfig,
        pixel: [i32; 2],
        landmark: &Landmark,
    ) -> bool {
        let dr = self.r_bp_p - landmark.position_p_m;
        let e_lmk_p = dr / dr.norm();
        // The focal direction is negated so that positive angles are true.
        let in_front =
            FRAC_PI_2 - safe_acos(-self.ec_p.dot(&e_lmk_p) / (self.ec_p.norm() * e_lmk_p.norm()));
        let facing = FRAC_PI_2
            - safe_acos(
                -self.ec_p.dot(&landmark.normal_p) / (self.ec_p.norm() * landmark.normal_p.norm()),
            );
        // Widened so `2 p` cannot overflow for a pixel saturated at the
        // `i32` range.
        in_front > 0.0
            && facing > 0.0
            && (2 * i64::from(pixel[0])).abs() as f64 <= config.nx_pixel
            && (2 * i64::from(pixel[1])).abs() as f64 <= config.ny_pixel
    }

    /// Upstream `checkLighting`.
    fn is_lit(&self, config: &PinholeCameraConfig, normal_p: Vector3<f64>) -> bool {
        let sun_elevation = FRAC_PI_2
            - safe_acos(normal_p.dot(&self.e_sp_p) / (self.e_sp_p.norm() * normal_p.norm()));
        sun_elevation > config.mask_sun_rad
    }

    /// Upstream `loopLandmarks` for one landmark.
    fn observe(&self, config: &PinholeCameraConfig, landmark: &Landmark) -> LandmarkMsg {
        let pixel = self.pixel(config, landmark.position_p_m);
        if self.in_field_of_view(config, pixel, landmark) && self.is_lit(config, landmark.normal_p)
        {
            LandmarkMsg {
                is_visible: true,
                pixel,
            }
        } else {
            LandmarkMsg::default()
        }
    }
}

/// Upstream's pixelization: round away from zero; exactly zero (or NaN) is
/// pixel `1`.
fn pixelize(coordinate: f64) -> i32 {
    if coordinate > 0.0 {
        coordinate.ceil() as i32
    } else if coordinate < 0.0 {
        coordinate.floor() as i32
    } else {
        1
    }
}

/// Landmark visibility and pixels: upstream `PinholeCamera`.
#[derive(Clone, Debug)]
pub struct PinholeCamera {
    /// Optics and mounting, read on every update so an override applies at
    /// once.
    pub config: Output<PinholeCameraConfig>,
    /// Upstream `ephemerisInMsg`: planet ephemeris.
    pub ephemeris_in_msg: Input<EphemerisMsg>,
    /// Upstream `scStateInMsg`: spacecraft state.
    pub sc_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `landmarkOutMsgs`: one output per landmark, in the order of
    /// [`Self::add_landmark`] calls.
    pub landmark_out_msgs: Vec<Output<LandmarkMsg>>,
    landmarks: Vec<Landmark>,
}

impl PinholeCamera {
    /// # Panics
    ///
    /// If `config` fails [`PinholeCameraConfig::validate`].
    pub fn new(config: PinholeCameraConfig) -> Self {
        if let Err(msg) = config.validate() {
            panic!("invalid PinholeCameraConfig: {msg}");
        }
        Self {
            config: Output::new(config),
            ephemeris_in_msg: Input::default(),
            sc_state_in_msg: Input::default(),
            landmark_out_msgs: Vec::new(),
            landmarks: Vec::new(),
        }
    }

    /// Registers a landmark (upstream `addLandmark`) at `position_p_m` with
    /// surface normal `normal_p`, both planet-fixed, and returns its output.
    pub fn add_landmark(
        &mut self,
        position_p_m: Vector3<f64>,
        normal_p: Vector3<f64>,
    ) -> &Output<LandmarkMsg> {
        self.landmarks.push(Landmark {
            position_p_m,
            normal_p,
        });
        self.landmark_out_msgs.push(Output::default());
        self.landmark_out_msgs
            .last()
            .expect("registered landmark output should be available")
    }

    /// The registered landmarks, at the index of their outputs.
    pub fn landmarks(&self) -> &[Landmark] {
        &self.landmarks
    }

    /// Upstream `processBatch`: visibility and pixel of every landmark for
    /// each sample `i` of spacecraft position `r_bp_p_m[i]` and attitude
    /// `sigma_bp[i]` (MRP of `[BP]`), both relative to the planet frame, and
    /// planet-to-Sun direction `e_sp_p[i]`. Row `i` of the result holds one
    /// entry per landmark; upstream returns the same data as a pixel matrix
    /// and a visibility matrix.
    ///
    /// # Panics
    ///
    /// If the three slices differ in length.
    pub fn process_batch(
        &self,
        r_bp_p_m: &[Vector3<f64>],
        sigma_bp: &[Vector3<f64>],
        e_sp_p: &[Vector3<f64>],
    ) -> Vec<Vec<LandmarkMsg>> {
        assert!(
            r_bp_p_m.len() == sigma_bp.len() && r_bp_p_m.len() == e_sp_p.len(),
            "process_batch needs one attitude and one Sun direction per position"
        );
        let config = self.config.read();
        r_bp_p_m
            .iter()
            .zip(sigma_bp)
            .zip(e_sp_p)
            .map(|((r_bp_p, sigma_bp), e_sp_p)| {
                let view = CameraView::new(&config, *r_bp_p, mrp_to_dcm(*sigma_bp), *e_sp_p);
                self.landmarks
                    .iter()
                    .map(|landmark| view.observe(&config, landmark))
                    .collect()
            })
            .collect()
    }

    fn check_connections(&self) {
        let name = self.config.read().name;
        if !self.sc_state_in_msg.is_connected() {
            log::error!("PinholeCamera '{name}' sc_state_in_msg was not linked");
        }
        if !self.ephemeris_in_msg.is_connected() {
            log::error!("PinholeCamera '{name}' ephemeris_in_msg was not linked");
        }
    }
}

impl Module for PinholeCamera {
    fn init(&mut self) {
        self.check_connections();
        for output in &self.landmark_out_msgs {
            output.write(LandmarkMsg::default());
        }
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let config = self.config.read();
        let planet = self.ephemeris_in_msg.read();
        let spacecraft = self.sc_state_in_msg.read();
        let view = CameraView::from_messages(&config, &planet, &spacecraft);
        for (landmark, output) in self.landmarks.iter().zip(&self.landmark_out_msgs) {
            output.write(view.observe(&config, landmark));
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `pinholeCamera/_UnitTest/test_unitPinholeCamera.py`
    //! with its camera, geometry and expected pixels, and adds the branches
    //! that test does not reach: the Sun mask, batch processing, and the
    //! pixel convention at zero.

    use std::f64::consts::{FRAC_PI_2, PI};

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use crate::messages::{EphemerisMsg, LandmarkMsg, Output, SpacecraftStateMsg};
    use crate::simulation::Simulation;
    use crate::utilities::rigid_body_kinematics::dcm_to_mrp;
    use crate::{Module, SimulationContext};

    use super::{PinholeCamera, PinholeCameraConfig, pixelize};

    /// Upstream `orbitalMotion.MU_SUN` \[km^3/s^2\].
    const MU_SUN_KM3PS2: f64 = 132_712_440_023.310;

    /// Upstream's camera: 25 mm focal length, a 2048 × 1536 sensor 17.3 mm
    /// wide, boresight along body `+x`.
    fn upstream_camera() -> PinholeCameraConfig {
        PinholeCameraConfig {
            name: "pinholeCamera".to_string(),
            focal_length_m: 25.0 * 1e-3,
            nx_pixel: 2048.0,
            ny_pixel: 1536.0,
            pixel_width_m: (17.3 * 1e-3) / 2048.0,
            dcm_cb: Matrix3::new(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0),
            ..Default::default()
        }
    }

    /// Upstream's mock planet: the identity planet frame at 1.4583 AU along
    /// `+x` (meters, with `MU_SUN` in km³/s² as upstream writes it).
    fn upstream_planet() -> EphemerisMsg {
        let a_p = 1.4583 * 149_597_870.7 * 1e3;
        EphemerisMsg {
            position_bdy_zero_n_m: Vector3::new(a_p, 0.0, 0.0),
            velocity_bdy_zero_n_mps: Vector3::new(0.0, (MU_SUN_KM3PS2 / a_p).sqrt(), 0.0),
            sigma_bn: dcm_to_mrp(Matrix3::identity()),
            ..Default::default()
        }
    }

    /// Upstream's mock spacecraft, 34 km from the planet on `-x`. Its `[BP]`
    /// is built column by column from the orbit frame and is a reflection
    /// (determinant `-1`), for which `C2MRP` returns the zero MRP, so the
    /// body frame is the planet frame and the camera looks along `+x` at the
    /// planet.
    fn upstream_spacecraft(planet: &EphemerisMsg) -> SpacecraftStateMsg {
        let a: f64 = 34.0 * 1e3;
        let mu: f64 = 4.4631 * 1e5;
        let r_bp_p = Vector3::new(-a, 0.0, 0.0);
        let v_bp_p = Vector3::new(0.0, -(mu / a).sqrt(), 0.0);
        let ir = r_bp_p / r_bp_p.norm();
        let ih = ir.cross(&(v_bp_p / v_bp_p.norm()));
        let it = ir.cross(&ih);
        let dcm_bp = Matrix3::from_columns(&[-ir, ih, it]);
        let dcm_bn = dcm_bp * Matrix3::identity();
        SpacecraftStateMsg {
            position_m: r_bp_p + planet.position_bdy_zero_n_m,
            velocity_mps: v_bp_p + planet.velocity_bdy_zero_n_mps,
            sigma_bn: dcm_to_mrp(dcm_bn),
            ..Default::default()
        }
    }

    /// The upstream landmarks: the image center and its four edge midpoints
    /// (facing the camera), then five that are not in view.
    fn upstream_landmarks() -> Vec<(Vector3<f64>, Vector3<f64>)> {
        let fov_x = 38.17124015837933 * PI / 180.0;
        let fov_y = 29.094758030219015 * PI / 180.0;
        let half_y = 18.0 * 1e3 * (fov_y / 2.0).tan();
        let half_x = 18.0 * 1e3 * (fov_x / 2.0).tan();
        let positions = [
            [-16.0 * 1e3, 0.0, 0.0],
            [-16.0 * 1e3, half_y, 0.0],
            [-16.0 * 1e3, -18.0 * 1e3 * (fov_y / 2.0).tan(), 0.0],
            [-16.0 * 1e3, 0.0, half_x],
            [-16.0 * 1e3, 0.0, -18.0 * 1e3 * (fov_x / 2.0).tan()],
            [16.0 * 1e3, 0.0, 0.0],
            [0.0, 16.0 * 1e3, 0.0],
            [0.0, -16.0 * 1e3, 0.0],
            [0.0, 0.0, 16.0 * 1e3],
            [0.0, 0.0, -16.0 * 1e3],
        ];
        let normals = [
            [-1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
        ];
        positions
            .iter()
            .zip(&normals)
            .map(|(position, normal)| (Vector3::from(*position), Vector3::from(*normal)))
            .collect()
    }

    fn upstream_module() -> PinholeCamera {
        let mut camera = PinholeCamera::new(upstream_camera());
        for (position, normal) in upstream_landmarks() {
            camera.add_landmark(position, normal);
        }
        camera
    }

    /// Upstream `test_visibility`: the center and edge landmarks are visible
    /// at the center pixel `(1, 1)` and at the image half-widths; the other
    /// five are not visible and report pixel `(0, 0)`.
    #[test]
    fn center_and_edge_landmarks_are_visible_and_the_rest_are_not() {
        let planet = Output::new(upstream_planet());
        let spacecraft = Output::new(upstream_spacecraft(&planet.read()));
        let mut camera = upstream_module();
        let mut probes: Vec<_> = (0..camera.landmark_out_msgs.len())
            .map(|_| crate::messages::Input::default())
            .collect();
        for (output, probe) in camera.landmark_out_msgs.iter().zip(&mut probes) {
            output.connect_to(probe);
        }

        let mut first_samples = Vec::new();
        {
            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            simulation.connect(&planet, &mut camera.ephemeris_in_msg);
            simulation.connect(&spacecraft, &mut camera.sc_state_in_msg);
            simulation.add_module("pinholeCamera", &mut camera, 1_000_000_000, 0);
            simulation.run_for(0);
            first_samples.extend(probes.iter().map(|probe| probe.read()));
            simulation.run_for(1_000_000_000);
        }

        let ref_pixel = [
            [1, 1],
            [1, 1536 / 2],
            [1, -1536 / 2],
            [-2048 / 2, 1],
            [2048 / 2, 1],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
        ];
        let ref_is_visible = [
            true, true, true, true, true, false, false, false, false, false,
        ];
        for (index, sample) in first_samples.iter().enumerate() {
            assert_eq!(sample.pixel, ref_pixel[index], "landmark {index} pixel");
            assert_eq!(
                sample.is_visible, ref_is_visible[index],
                "landmark {index} visibility"
            );
        }
        // The scene is static: the second sample repeats the first.
        for (output, first) in camera.landmark_out_msgs.iter().zip(&first_samples) {
            assert_eq!(&output.read(), first);
        }
    }

    /// The fields of view upstream's test quotes ("check camera.FOVx").
    #[test]
    fn fields_of_view_follow_from_the_optics() {
        let config = upstream_camera();
        assert!((config.fov_x_rad() - 38.17124015837933 * PI / 180.0).abs() < 1.0e-14);
        assert!((config.fov_y_rad() - 29.094758030219015 * PI / 180.0).abs() < 1.0e-14);
    }

    /// The batch path agrees with the message path for the same geometry, and
    /// a batch sample looking away from the planet sees nothing.
    #[test]
    fn batch_processing_matches_the_update() {
        let planet_msg = upstream_planet();
        let spacecraft_msg = upstream_spacecraft(&planet_msg);
        let planet = Output::new(planet_msg.clone());
        let spacecraft = Output::new(spacecraft_msg.clone());
        let mut camera = upstream_module();
        planet.connect_to(&mut camera.ephemeris_in_msg);
        spacecraft.connect_to(&mut camera.sc_state_in_msg);
        camera.init();
        camera.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });
        let from_messages: Vec<LandmarkMsg> = camera
            .landmark_out_msgs
            .iter()
            .map(|output| output.read())
            .collect();

        let r_bp_p = spacecraft_msg.position_m - planet_msg.position_bdy_zero_n_m;
        let e_sp_p = -planet_msg.position_bdy_zero_n_m.normalize();
        // A 180° yaw about body z points the camera away from the planet.
        let away = Vector3::new(0.0, 0.0, 1.0);
        let batch = camera.process_batch(
            &[r_bp_p, r_bp_p],
            &[spacecraft_msg.sigma_bn, away],
            &[e_sp_p, e_sp_p],
        );
        assert_eq!(batch.len(), 2);
        assert_eq!(batch[0], from_messages);
        assert!(batch[1].iter().all(|landmark| !landmark.is_visible));
    }

    /// The Sun mask: with the Sun along the image-center landmark's surface
    /// (elevation zero), it is lit for upstream's default mask and dark for
    /// a zero mask; an exactly anti-sunward normal is dark even by default.
    #[test]
    fn sun_mask_gates_visibility() {
        let config = upstream_camera();
        let r_bp_p = Vector3::new(-34.0e3, 0.0, 0.0);
        let center = (
            Vector3::new(-16.0e3, 0.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
        );
        let visible = |mask_sun_rad: f64, e_sp_p: Vector3<f64>| {
            let mut camera = PinholeCamera::new(PinholeCameraConfig {
                mask_sun_rad,
                ..config.clone()
            });
            camera.add_landmark(center.0, center.1);
            camera.process_batch(&[r_bp_p], &[Vector3::zeros()], &[e_sp_p])[0][0].is_visible
        };
        let grazing = Vector3::new(0.0, 1.0, 0.0);
        assert!(visible(-FRAC_PI_2, grazing));
        assert!(!visible(0.0, grazing));
        assert!(visible(0.0, Vector3::new(-1.0, 0.0, 0.0)));
        assert!(!visible(-FRAC_PI_2, Vector3::new(1.0, 0.0, 0.0)));
    }

    /// Upstream's pixelization: away from zero, and `1` at exactly zero.
    #[test]
    fn pixels_round_away_from_zero_with_one_at_zero() {
        assert_eq!(pixelize(0.2), 1);
        assert_eq!(pixelize(767.0001), 768);
        assert_eq!(pixelize(768.0), 768);
        assert_eq!(pixelize(-0.2), -1);
        assert_eq!(pixelize(-1024.0), -1024);
        assert_eq!(pixelize(0.0), 1);
        assert_eq!(pixelize(-0.0), 1);
    }

    #[test]
    #[should_panic(expected = "focal_length_m")]
    fn rejects_a_camera_without_optics() {
        let _ = PinholeCamera::new(PinholeCameraConfig::default());
    }
}
