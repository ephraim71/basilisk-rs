//! Synthetic camera images of a sunlit planet: a small stand-in for the
//! images the Vizard application renders for upstream's optical-navigation
//! scenarios (feature `opencv`).
//!
//! Upstream's `vizInterface` sends the spacecraft, planet and camera states
//! to Vizard, a Unity application, which renders each instrument camera's
//! view (Mars with its texture, Unity lighting and post-processing) and sends
//! the encoded image back on `opnavImageOutMsgs`. Vizard is not available
//! here, so [`PlanetRenderer`] produces that message itself: it ray-traces a
//! textureless ellipsoid through the camera's pinhole model and encodes the
//! image as a PNG [`CameraImageMsg`], which the `camera` module then corrupts
//! and the image-processing modules (`houghCircles`, `limbFinding`,
//! `centerRadiusCNN`) read as they read Vizard's images.
//!
//! # The model
//!
//! * **Camera.** The pinhole geometry of [`CameraConfigMsg`], the convention
//!   `pixelLineConverter` (and upstream's truth in the scenarios'
//!   `pull_outputs`) inverts: the camera `+z` axis is the boresight, `+x`
//!   runs along the image columns and `+y` along the rows, and the centre of
//!   pixel `(u, v)` (OpenCV pixel coordinates) looks along
//!   `[(u - W/2 + 0.5) / d_x, (v - H/2 + 0.5) / d_y, 1]` with
//!   `d_x = W / (2 tan(fov W / (2 H)))`, `d_y = H / (2 tan(fov / 2))`; `fov`
//!   is the field of view along the camera `y` axis. The camera sits at
//!   `cameraPos_B` on the spacecraft, rotated by `sigma_CB`.
//! * **Body.** An ellipsoid with semi-axes [`PlanetRendererConfig::radii_m`]
//!   in the planet-fixed frame of the planet's ephemeris message (its
//!   `J20002Pfix`; inertially aligned without one). No texture, no
//!   atmosphere, no other bodies, no stars (upstream's `skyBox = "black"`).
//! * **Light.** A Lambertian surface of colour
//!   [`PlanetRendererConfig::albedo_bgr`] lit by the Sun (a point at the Sun
//!   message's position) plus a uniform ambient term:
//!   `colour · (ambient + (1 - ambient) max(0, n · s))`. The ambient term
//!   stands in for Vizard's ambient light (upstream sets
//!   `settings.ambient = 0.5`), which leaves the night side faintly visible
//!   in Vizard's images.
//! * **Edges.** Pixels whose centre ray passes within
//!   [`LIMB_SUPERSAMPLING_BAND_PX`] pixels of the limb are averaged over an
//!   `n x n` grid of rays ([`PlanetRendererConfig::limb_supersampling`]), so
//!   the limb is anti-aliased to a fraction of a pixel, as a rendered image
//!   is.
//!
//! The colour, ambient level and anti-aliasing are choices of this stand-in,
//! set to look like the Vizard Mars images of upstream's unit tests
//! (`houghCircles/_UnitTest/mars.png`, `limbFinding/_UnitTest/MarsDark.jpg`);
//! they are not calibrated to Vizard. The image processing therefore sees a
//! cleaner planet than Vizard's (no surface texture, no craters on the limb),
//! so only outcomes such as navigation error bounds and convergence compare
//! with upstream's runs, not pixels.
//!
//! # Timing
//!
//! Vizard renders an instrument camera every `renderRate` of its camera
//! message while the camera is on. [`PlanetRenderer`] does the same: at every
//! update whose time is a multiple of the camera's `render_rate_nanos` it
//! renders and publishes the image, time-tagged with the current time;
//! otherwise it leaves its output alone, so the camera module (which only
//! takes an image time-tagged at the current time) publishes no image.

#![allow(dead_code)]

use basilisk_rs::messages::{
    CameraConfigMsg, CameraImageMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg,
};
use basilisk_rs::utilities::astro_constants::{REQ_MARS, RP_MARS};
use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
use basilisk_rs::{Module, SimulationContext};
use nalgebra::{Matrix3, Vector3};
use opencv::core::{CV_8UC3, Mat, Scalar, Vec3b, Vector};
use opencv::imgcodecs;
use opencv::prelude::*;

/// Half-width \[pixels\] of the band around the limb whose pixels are
/// supersampled.
pub const LIMB_SUPERSAMPLING_BAND_PX: f64 = 1.5;

/// Parameters of [`PlanetRenderer`].
#[derive(Clone, Debug, PartialEq)]
pub struct PlanetRendererConfig {
    /// Module name.
    pub name: String,
    /// Semi-axes of the body \[m\] along the planet-fixed `x`, `y`, `z` axes.
    pub radii_m: Vector3<f64>,
    /// Surface colour at normal solar incidence, blue, green, red \[0, 255\].
    pub albedo_bgr: [f64; 3],
    /// Uniform ambient light, as a fraction of normal solar incidence [-].
    pub ambient: f64,
    /// Rays per pixel side near the limb (1 turns the anti-aliasing off).
    pub limb_supersampling: u32,
}

impl PlanetRendererConfig {
    /// Mars: upstream's `REQ_MARS` and `RP_MARS` radii, a Vizard-like
    /// red-brown surface and a faint night side.
    pub fn mars(name: &str) -> Self {
        Self {
            name: name.to_string(),
            radii_m: Vector3::new(REQ_MARS, REQ_MARS, RP_MARS) * 1e3,
            albedo_bgr: [80.0, 110.0, 175.0],
            ambient: 0.18,
            limb_supersampling: 4,
        }
    }
}

/// What the camera sees: the camera pose and the planet and Sun positions,
/// all relative to the inertial frame `N`.
#[derive(Clone, Debug)]
pub struct Scene {
    /// Camera position \[m\].
    pub r_cn_n_m: Vector3<f64>,
    /// `[CN]`: inertial to camera frame.
    pub dcm_cn: Matrix3<f64>,
    /// Planet centre \[m\].
    pub r_pn_n_m: Vector3<f64>,
    /// `[PN]`: inertial to planet-fixed frame.
    pub dcm_pn: Matrix3<f64>,
    /// Sun position \[m\].
    pub r_sn_n_m: Vector3<f64>,
}

impl Scene {
    /// The scene of `camera` on the spacecraft `state`, with the planet and
    /// Sun ephemerides.
    pub fn new(
        camera: &CameraConfigMsg,
        state: &SpacecraftStateMsg,
        planet: &PlanetStateMsg,
        sun: &PlanetStateMsg,
    ) -> Self {
        let dcm_bn = mrp_to_dcm(state.sigma_bn);
        let dcm_cb = mrp_to_dcm(camera.sigma_cb);
        Self {
            r_cn_n_m: state.position_m + dcm_bn.transpose() * camera.camera_pos_b_m,
            dcm_cn: dcm_cb * dcm_bn,
            r_pn_n_m: planet.position_inertial_m,
            dcm_pn: planet
                .orientation
                .as_ref()
                .map_or_else(Matrix3::identity, |orientation| {
                    orientation.inertial_to_fixed
                }),
            r_sn_n_m: sun.position_inertial_m,
        }
    }
}

/// The pinhole camera of a [`CameraConfigMsg`].
#[derive(Clone, Copy, Debug)]
pub struct Pinhole {
    pub width_px: i32,
    pub height_px: i32,
    /// Pixels per unit focal-plane coordinate along `x` and `y`.
    pub d_x: f64,
    pub d_y: f64,
}

impl Pinhole {
    pub fn new(camera: &CameraConfigMsg) -> Self {
        let [width_px, height_px] = camera.resolution_px;
        let (width, height) = (f64::from(width_px), f64::from(height_px));
        let p_x = 2.0 * (camera.field_of_view_rad * width / height / 2.0).tan();
        let p_y = 2.0 * (camera.field_of_view_rad / 2.0).tan();
        Self {
            width_px,
            height_px,
            d_x: width / p_x,
            d_y: height / p_y,
        }
    }

    /// The camera-frame ray `[x, y, 1]` through the OpenCV pixel coordinates
    /// `(u, v)`.
    pub fn ray_c(&self, u: f64, v: f64) -> Vector3<f64> {
        Vector3::new(
            (u - f64::from(self.width_px) / 2.0 + 0.5) / self.d_x,
            (v - f64::from(self.height_px) / 2.0 + 0.5) / self.d_y,
            1.0,
        )
    }

    /// The OpenCV pixel coordinates of the camera-frame direction `r_c`
    /// (in front of the camera).
    pub fn pixel(&self, r_c: &Vector3<f64>) -> (f64, f64) {
        (
            r_c.x / r_c.z * self.d_x + f64::from(self.width_px) / 2.0 - 0.5,
            r_c.y / r_c.z * self.d_y + f64::from(self.height_px) / 2.0 - 0.5,
        )
    }
}

/// The ellipsoid in its own frame, scaled to the unit sphere.
struct ScaledBody {
    inverse_radii: Vector3<f64>,
    /// Camera position relative to the planet, planet frame, scaled.
    origin: Vector3<f64>,
    /// `[PC]` with the scaling: camera-frame ray to scaled planet frame.
    to_scaled: Matrix3<f64>,
    /// `[PC]`: camera to planet frame.
    dcm_pc: Matrix3<f64>,
    /// Camera position relative to the planet, planet frame \[m\].
    r_cp_p_m: Vector3<f64>,
    /// Sun position relative to the planet, planet frame \[m\].
    r_sp_p_m: Vector3<f64>,
}

impl ScaledBody {
    fn new(radii_m: &Vector3<f64>, scene: &Scene) -> Self {
        let inverse_radii = radii_m.map(|radius| 1.0 / radius);
        let dcm_pc = scene.dcm_pn * scene.dcm_cn.transpose();
        let r_cp_p_m = scene.dcm_pn * (scene.r_cn_n_m - scene.r_pn_n_m);
        Self {
            origin: r_cp_p_m.component_mul(&inverse_radii),
            to_scaled: Matrix3::from_diagonal(&inverse_radii) * dcm_pc,
            dcm_pc,
            r_cp_p_m,
            r_sp_p_m: scene.dcm_pn * (scene.r_sn_n_m - scene.r_pn_n_m),
            inverse_radii,
        }
    }

    /// The closest approach of the camera-frame ray `ray_c` to the scaled
    /// centre (1 at the limb), and the nearest hit distance along the ray in
    /// units of `ray_c`, if it hits.
    fn trace(&self, ray_c: &Vector3<f64>) -> (f64, Option<f64>) {
        let direction = self.to_scaled * ray_c;
        let a = direction.norm_squared();
        let b = 2.0 * self.origin.dot(&direction);
        let c = self.origin.norm_squared() - 1.0;
        let along = -b / (2.0 * a);
        let miss = (self.origin + direction * along).norm();
        let discriminant = b * b - 4.0 * a * c;
        if discriminant < 0.0 {
            return (miss, None);
        }
        let t = (-b - discriminant.sqrt()) / (2.0 * a);
        (miss, (t > 0.0).then_some(t))
    }

    /// The brightness [0, 1] of the surface hit at `t` along `ray_c`.
    fn shade(&self, ray_c: &Vector3<f64>, t: f64, ambient: f64) -> f64 {
        let point = self.r_cp_p_m + self.dcm_pc * ray_c * t;
        let normal = point
            .component_mul(&self.inverse_radii)
            .component_mul(&self.inverse_radii)
            .normalize();
        let to_sun = (self.r_sp_p_m - point).normalize();
        ambient + (1.0 - ambient) * normal.dot(&to_sun).max(0.0)
    }
}

/// Renders the scene to an 8-bit BGR image.
pub fn render(config: &PlanetRendererConfig, camera: &CameraConfigMsg, scene: &Scene) -> Mat {
    let pinhole = Pinhole::new(camera);
    let mut image = Mat::new_rows_cols_with_default(
        pinhole.height_px,
        pinhole.width_px,
        CV_8UC3,
        Scalar::all(0.0),
    )
    .expect("allocate the image");
    let body = ScaledBody::new(&config.radii_m, scene);
    // The scaled-sphere width of a pixel at the body: the band the limb
    // supersampling covers.
    let pixel_angle = 1.0 / pinhole.d_x.min(pinhole.d_y);
    let band = LIMB_SUPERSAMPLING_BAND_PX * pixel_angle * body.origin.norm();
    let samples = config.limb_supersampling.max(1);
    let width = pinhole.width_px as usize;
    let pixels = image.data_typed_mut::<Vec3b>().expect("BGR image");
    for (index, pixel) in pixels.iter_mut().enumerate() {
        let (u, v) = ((index % width) as f64, (index / width) as f64);
        let centre = pinhole.ray_c(u, v);
        let (miss, hit) = body.trace(&centre);
        let brightness = if samples > 1 && (miss - 1.0).abs() < band {
            let mut sum = 0.0;
            for i in 0..samples {
                for j in 0..samples {
                    let offset = |k: u32| (f64::from(k) + 0.5) / f64::from(samples) - 0.5;
                    let ray = pinhole.ray_c(u + offset(i), v + offset(j));
                    if let (_, Some(t)) = body.trace(&ray) {
                        sum += body.shade(&ray, t, config.ambient);
                    }
                }
            }
            sum / f64::from(samples * samples)
        } else {
            hit.map_or(0.0, |t| body.shade(&centre, t, config.ambient))
        };
        if brightness > 0.0 {
            *pixel = Vec3b::from(
                config
                    .albedo_bgr
                    .map(|channel| (channel * brightness).round().clamp(0.0, 255.0) as u8),
            );
        }
    }
    image
}

/// Encodes `image` as an uncompressed PNG, the format the camera module
/// publishes too.
pub fn encode_png(image: &Mat) -> Vec<u8> {
    let mut buffer = Vector::<u8>::new();
    let no_compression = Vector::from_slice(&[imgcodecs::IMWRITE_PNG_COMPRESSION, 0]);
    let encoded = imgcodecs::imencode(".png", image, &mut buffer, &no_compression)
        .expect("encode the rendered image");
    assert!(encoded, "PNG encoding failed");
    buffer.to_vec()
}

/// The stand-in for Vizard rendering one instrument camera (see the module
/// docs).
pub struct PlanetRenderer {
    pub config: PlanetRendererConfig,
    /// The camera's configuration (upstream `vizInterface.addCamMsgToModule`).
    pub camera_config_in_msg: Input<CameraConfigMsg>,
    /// The spacecraft carrying the camera.
    pub spacecraft_state_in_msg: Input<SpacecraftStateMsg>,
    /// The planet in view, with its planet-fixed orientation.
    pub planet_state_in_msg: Input<PlanetStateMsg>,
    /// The Sun.
    pub sun_state_in_msg: Input<PlanetStateMsg>,
    /// Upstream `vizInterface.opnavImageOutMsgs[i]`.
    pub image_out_msg: Output<CameraImageMsg>,
    /// Images rendered since initialization.
    pub frames_rendered: u64,
}

impl PlanetRenderer {
    pub fn new(config: PlanetRendererConfig) -> Self {
        Self {
            config,
            camera_config_in_msg: Input::default(),
            spacecraft_state_in_msg: Input::default(),
            planet_state_in_msg: Input::default(),
            sun_state_in_msg: Input::default(),
            image_out_msg: Output::default(),
            frames_rendered: 0,
        }
    }
}

impl Module for PlanetRenderer {
    fn init(&mut self) {
        self.frames_rendered = 0;
        self.image_out_msg.write(CameraImageMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let camera = self.camera_config_in_msg.read();
        let [width, height] = camera.resolution_px;
        let due = camera.render_rate_nanos > 0
            && context
                .current_sim_nanos
                .is_multiple_of(camera.render_rate_nanos);
        if !camera.is_on || width <= 0 || height <= 0 || !due {
            return;
        }
        let scene = Scene::new(
            &camera,
            &self.spacecraft_state_in_msg.read(),
            &self.planet_state_in_msg.read(),
            &self.sun_state_in_msg.read(),
        );
        let image = render(&self.config, &camera, &scene);
        self.image_out_msg.write(CameraImageMsg {
            time_tag_nanos: context.current_sim_nanos,
            valid: true,
            camera_id: camera.camera_id,
            image: encode_png(&image),
            image_type: 3,
        });
        self.frames_rendered += 1;
    }
}

#[cfg(test)]
mod tests {
    //! The stand-in's geometry against the pinhole model, and what the
    //! image processing makes of it.

    use basilisk_rs::fsw_algorithms::image_processing::hough_circles::{
        HoughCircles, HoughCirclesConfig,
    };
    use hifitime::Epoch;
    use opencv::imgcodecs;

    use super::*;

    const RANGE_M: f64 = 18_000e3;

    fn camera() -> CameraConfigMsg {
        CameraConfigMsg {
            camera_id: 1,
            is_on: true,
            field_of_view_rad: 55.0_f64.to_radians(),
            resolution_px: [512, 512],
            render_rate_nanos: 60_000_000_000,
            ..CameraConfigMsg::default()
        }
    }

    /// A spherical Mars at the origin, the camera `RANGE_M` away along
    /// `-z` looking along `+z` (`[CN]` rotated by `tilt_rad` about `y`), the
    /// Sun far along `sun_dir`.
    fn scene(tilt_rad: f64, sun_dir: Vector3<f64>) -> Scene {
        Scene {
            r_cn_n_m: Vector3::new(0.0, 0.0, -RANGE_M),
            dcm_cn: basilisk_rs::utilities::rigid_body_kinematics::mi(
                tilt_rad,
                basilisk_rs::utilities::rigid_body_kinematics::Axis::Y,
            ),
            r_pn_n_m: Vector3::zeros(),
            dcm_pn: Matrix3::identity(),
            r_sn_n_m: sun_dir.normalize() * 2.0e11,
        }
    }

    fn sphere() -> PlanetRendererConfig {
        let mut config = PlanetRendererConfig::mars("renderer");
        config.radii_m = Vector3::repeat(REQ_MARS * 1e3);
        config
    }

    /// The centroid of the body's image, each pixel weighted by its red
    /// level (with ambient light only, the fraction of it the body covers).
    fn centroid(image: &Mat) -> (f64, f64) {
        let pixels = image.data_typed::<Vec3b>().unwrap();
        let (mut sum, mut x, mut y) = (0.0, 0.0, 0.0);
        for (index, pixel) in pixels.iter().enumerate() {
            let weight = f64::from(pixel[2]);
            sum += weight;
            x += weight * (index % 512) as f64;
            y += weight * (index / 512) as f64;
        }
        (x / sum, y / sum)
    }

    /// [`sphere`] lit by ambient light only.
    fn flat_sphere() -> PlanetRendererConfig {
        let mut config = sphere();
        config.ambient = 1.0;
        config
    }

    #[test]
    fn a_uniformly_lit_sphere_is_the_pinhole_disc() {
        let camera = camera();
        // Ambient light only: every pixel holds the surface colour times
        // the fraction of it the disc covers.
        let config = flat_sphere();
        let image = render(&config, &camera, &scene(0.0, Vector3::new(0.0, 0.0, -1.0)));
        let (x, y) = centroid(&image);
        assert!(
            (x - 255.5).abs() < 0.05 && (y - 255.5).abs() < 0.05,
            "({x}, {y})"
        );
        let pinhole = Pinhole::new(&camera);
        let radius_px = (REQ_MARS * 1e3 / RANGE_M).asin().tan() * pinhole.d_x;
        let pixels = image.data_typed::<Vec3b>().unwrap();
        let coverage: f64 = pixels
            .iter()
            .map(|pixel| f64::from(pixel[2]) / config.albedo_bgr[2])
            .sum();
        let disc = std::f64::consts::PI * radius_px * radius_px;
        assert!((coverage / disc - 1.0).abs() < 2e-3, "{coverage} vs {disc}");
    }

    #[test]
    fn an_off_axis_planet_is_the_projected_ellipse() {
        let camera = camera();
        let tilt_rad = 10.0_f64.to_radians();
        let scene = scene(tilt_rad, Vector3::new(0.0, 0.0, -1.0));
        let image = render(&flat_sphere(), &camera, &scene);
        let (x, y) = centroid(&image);
        // The limb cone (half angle `alpha` about the planet direction,
        // `tilt` off the boresight) cuts the image plane in an ellipse
        // centred between `tan(tilt - alpha)` and `tan(tilt + alpha)`, a
        // little beyond the planet centre's own pixel.
        let pinhole = Pinhole::new(&camera);
        let alpha = (REQ_MARS * 1e3 / RANGE_M).asin();
        let offset = ((tilt_rad + alpha).tan() + (tilt_rad - alpha).tan()) / 2.0 * pinhole.d_x;
        let (u, v) = pinhole.pixel(&(scene.dcm_cn * (scene.r_pn_n_m - scene.r_cn_n_m)));
        assert!(
            (x - (255.5 - offset)).abs() < 0.05,
            "{x} vs {}",
            255.5 - offset
        );
        assert!((y - v).abs() < 0.05, "{y} vs {v}");
        assert!(
            x < u - 3.0,
            "the ellipse centre lies beyond the planet's pixel {u}"
        );
    }

    #[test]
    fn the_sunlit_side_is_brighter_and_the_night_side_faint() {
        let camera = camera();
        // The Sun along +x: the +u half of the disc lit.
        let image = render(&sphere(), &camera, &scene(0.0, Vector3::x()));
        let pixel = |u: i32, v: i32| *image.at_2d::<Vec3b>(v, u).unwrap();
        let config = sphere();
        let lit = pixel(330, 256);
        let night = pixel(180, 256);
        assert!(lit[2] > 100, "{lit:?}");
        let ambient = (config.albedo_bgr[2] * config.ambient).round() as u8;
        assert_eq!(night[2], ambient);
        assert_eq!(pixel(5, 5)[2], 0, "black sky");
    }

    #[test]
    fn hough_circles_find_the_rendered_planet() {
        let camera = camera();
        let image = render(
            &sphere(),
            &camera,
            &scene(0.0, Vector3::new(0.3, 0.2, -1.0)),
        );
        let pinhole = Pinhole::new(&camera);
        let radius_px = (REQ_MARS * 1e3 / RANGE_M).asin().tan() * pinhole.d_x;
        // `BSK_OpNavFsw.SetImageProcessing`.
        let mut hough = HoughCircles::new(HoughCirclesConfig {
            expected_circles: 1,
            canny_thresh: 200,
            vote_thresh: 25,
            hough_min_dist_px: 50,
            hough_min_radius_px: 20,
            blur_size_px: 9,
            noise_sf: 1.0,
            ..HoughCirclesConfig::default()
        });
        let image_out = Output::new(CameraImageMsg {
            time_tag_nanos: 0,
            valid: true,
            camera_id: 1,
            image: encode_png(&image),
            image_type: 3,
        });
        image_out.connect_to(&mut hough.image_in_msg);
        hough.init();
        hough.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });
        let circles = hough.opnav_circles_out_msg.read();
        assert!(circles.valid);
        let center = circles.circles_centers_px[0];
        assert!(
            (center.x - 255.5).abs() <= 1.0 && (center.y - 255.5).abs() <= 1.0,
            "{center}"
        );
        assert!((circles.circles_radii_px[0] - radius_px).abs() <= 1.5);
        // The PNG decodes to the rendered pixels.
        let decoded = imgcodecs::imdecode(
            &Vector::<u8>::from_slice(&encode_png(&image)),
            imgcodecs::IMREAD_COLOR,
        )
        .unwrap();
        assert_eq!(
            decoded.data_typed::<Vec3b>().unwrap(),
            image.data_typed::<Vec3b>().unwrap()
        );
    }

    #[test]
    fn renders_at_the_render_rate_while_the_camera_is_on() {
        let mut renderer = PlanetRenderer::new(sphere());
        let config = Output::new(camera());
        let state = Output::new(SpacecraftStateMsg {
            position_m: Vector3::new(0.0, 0.0, -RANGE_M),
            ..SpacecraftStateMsg::default()
        });
        let planet = Output::new(PlanetStateMsg::default());
        let sun = Output::new(PlanetStateMsg {
            position_inertial_m: Vector3::new(0.0, 0.0, -2.0e11),
            ..PlanetStateMsg::default()
        });
        config.connect_to(&mut renderer.camera_config_in_msg);
        state.connect_to(&mut renderer.spacecraft_state_in_msg);
        planet.connect_to(&mut renderer.planet_state_in_msg);
        sun.connect_to(&mut renderer.sun_state_in_msg);
        renderer.init();
        let context = |seconds: u64| SimulationContext {
            current_sim_nanos: seconds * 1_000_000_000,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        };
        for seconds in [0, 30, 60, 90] {
            renderer.update(&context(seconds));
        }
        assert_eq!(renderer.frames_rendered, 2);
        let image = renderer.image_out_msg.read();
        assert!(image.valid);
        assert_eq!(image.time_tag_nanos, 60_000_000_000);
        assert_eq!(image.camera_id, 1);
        config.write(CameraConfigMsg {
            is_on: false,
            ..camera()
        });
        renderer.update(&context(120));
        assert_eq!(renderer.frames_rendered, 2);
    }
}
