//! A camera that corrupts rendered images with sensor effects: port of
//! upstream Basilisk `camera` (feature `opencv`).
//!
//! Every update publishes the camera's parameters on
//! [`Camera::camera_config_out_msg`] (upstream `cameraConfigOutMsg`, the
//! message Vizard renders instrument images from), then takes this update's
//! image (see [`acquire_image`]: a configured file, else the input message's
//! encoded image, normally Vizard's), corrupts it and publishes it re-encoded
//! as an uncompressed PNG on [`Camera::image_out_msg`]. An image from a file
//! is stamped with the current time, the configured camera ID and its channel
//! count; an image from the input keeps the input's time tag, camera ID and
//! image type. Without an image the output is zeroed. An unreadable image is
//! upstream's `BasiliskError`: logged, and the image output is not written.
//!
//! The corruptions run in upstream's fixed order, each only when its
//! parameter is nonzero:
//!
//! 1. Gaussian noise of standard deviation `2 gaussian`, added in 16-bit and
//!    saturated back to 8-bit, then a to-zero threshold at `6 gaussian` that
//!    keeps the sky dark;
//! 2. a box blur of `blur_param` pixels, rounded and made odd;
//! 3. dark current: a constant `15 dark_current` added the same way (upstream
//!    models it as Gaussian noise of zero standard deviation);
//! 4. an HSV adjustment: the hue rotated by `hsv[0]` radians (in whole
//!    degrees, wrapping), saturation and value scaled by `1 + hsv[k] / 100`
//!    and clamped to `[0, 255]`;
//! 5. a BGR adjustment: each channel scaled by `1 + bgr_percent[k] / 100` and
//!    clamped;
//! 6. dead (black) and hot (white) pixels, each with probability
//!    `2e-5 salt_pepper`. The generator is reseeded every frame, so the same
//!    pixels are stuck in every image;
//! 7. `round(cosmic_rays)` cosmic-ray attempts, each a one-pixel white line
//!    of up to 50 pixels drawn with probability `1 - 1 / round(cosmic_rays)²`.
//!
//! Random numbers come from OpenCV's `cv::RNG`, as upstream. Where upstream's
//! Gaussian noise draws from OpenCV's thread-wide generator (`cv::randn`),
//! each camera here owns one, started from the same default state
//! (`0xFFFFFFFF`) at every reset, so a camera's noise does not depend on what
//! else ran on its thread. Cosmic rays are seeded, as upstream, from the
//! simulation time, the attempt number and the wall-clock time in seconds;
//! [`CameraConfig::cosmic_ray_seed`] replaces the wall clock for reproducible
//! runs. A dead or hot pixel's row is drawn before its column (upstream
//! leaves that order to the compiler).
//!
//! Upstream's `sensorTimeTag` (the input's write time) is stored but never
//! used there, so it is not ported.

use std::fmt;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use nalgebra::Vector3;
use opencv::core::{self as cv_core, Mat, Point, RNG, Scalar, Size, Vec3b, Vector};
use opencv::imgcodecs;
use opencv::imgproc;
use opencv::prelude::*;

use crate::messages::{CameraConfigMsg, CameraImageMsg, Input, Output, VizSetting};
use crate::utilities::opencv_image::{
    ImageError, ImageOrigin, acquire_image, save_debug_image, time_stamped_path,
};
use crate::{Module, SimulationContext};

/// `cv::RNG`'s default state, which a thread's `cv::theRNG()` also starts
/// from.
const OPENCV_DEFAULT_RNG_STATE: u64 = 0xFFFF_FFFF;

/// Standard deviation of the Gaussian noise per unit of `gaussian`.
const GAUSSIAN_NOISE_SCALE: f64 = 2.0;
/// To-zero threshold per unit of `gaussian`: three noise sigmas.
const GAUSSIAN_THRESHOLD_SCALE: f64 = 6.0;
/// Dark-current offset per unit of `dark_current`.
const DARK_CURRENT_SCALE: f64 = 15.0;
/// Dead- and hot-pixel probability per unit of `salt_pepper`.
const SALT_PEPPER_SCALE: f64 = 0.00002;
/// Longest cosmic ray \[pixels\].
const COSMIC_RAY_MAX_SIZE_PX: i32 = 50;

/// Parameters of [`Camera`]; the defaults are upstream's constructor values,
/// with every corruption off.
#[derive(Clone, Debug, PartialEq)]
pub struct CameraConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `filename`: read the image from this file instead of the
    /// input message (upstream's empty string is `None`).
    pub filename: Option<PathBuf>,
    /// Upstream `saveDir`: prefix of the saved image path, which continues
    /// with the time in seconds and `.png`.
    pub save_dir: String,
    /// Upstream `saveImages` (`int`): save every corrupted image.
    pub save_images: bool,

    /// Upstream `parentName`: name of the body the camera is attached to.
    pub parent_name: String,
    /// Upstream `cameraIsOn` (`int`): the camera takes images.
    pub camera_is_on: bool,
    /// Upstream `cameraID` [-].
    pub camera_id: i64,
    /// Upstream `resolution` \[pixels\]: image width and height.
    pub resolution_px: [i32; 2],
    /// Upstream `renderRate` \[ns\]: interval between captured frames.
    pub render_rate_nanos: u64,
    /// Upstream `fieldOfView` \[rad\]: edge-to-edge field of view along the
    /// camera y axis.
    pub field_of_view_rad: f64,
    /// Upstream `cameraPos_B` \[m\]: camera position, body components.
    pub camera_pos_b_m: Vector3<f64>,
    /// Upstream `sigma_CB` [-]: MRP attitude of the camera frame relative to
    /// the body frame.
    pub sigma_cb: Vector3<f64>,
    /// Upstream `skyBox`: star-field preference.
    pub sky_box: String,
    /// Upstream `postProcessingOn` (`int` -1 / 0 / 1).
    pub post_processing_on: VizSetting,
    /// Upstream `ppFocusDistance` \[m\]; 0 turns the parameter off.
    pub pp_focus_distance_m: f64,
    /// Upstream `ppAperture` [-]; 0 turns the parameter off.
    pub pp_aperture: f64,
    /// Upstream `ppFocalLength` \[m\]; 0 turns the parameter off.
    pub pp_focal_length_m: f64,
    /// Upstream `ppMaxBlurSize` [-]; 0 turns the parameter off.
    pub pp_max_blur_size: i32,

    /// Upstream `gaussian` [-]: Gaussian noise level.
    pub gaussian: f64,
    /// Upstream `darkCurrent` [-]: dark current intensity.
    pub dark_current: f64,
    /// Upstream `saltPepper` [-]: dead and hot pixel level.
    pub salt_pepper: f64,
    /// Upstream `cosmicRays` [-]: number of cosmic-ray attempts per image.
    pub cosmic_rays: f64,
    /// Upstream `blurParam` \[pixels\]: box blur size.
    pub blur_param: f64,
    /// Upstream `hsv`: hue shift \[rad\], then saturation and value changes
    /// \[percent\].
    pub hsv: Vector3<f64>,
    /// Upstream `bgrPercent` \[percent\]: blue, green and red changes.
    pub bgr_percent: Vector3<f64>,
    /// Not upstream: stands in for the wall-clock seconds in the cosmic-ray
    /// seed, so that runs repeat. `None` uses the wall clock, as upstream.
    pub cosmic_ray_seed: Option<u64>,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            name: "camera".to_string(),
            filename: None,
            save_dir: String::new(),
            save_images: false,
            parent_name: String::new(),
            camera_is_on: false,
            camera_id: 1,
            resolution_px: [512, 512],
            render_rate_nanos: 60_000_000_000,
            field_of_view_rad: 0.7,
            camera_pos_b_m: Vector3::zeros(),
            sigma_cb: Vector3::zeros(),
            sky_box: "black".to_string(),
            post_processing_on: VizSetting::UseVizDefault,
            pp_focus_distance_m: 0.0,
            pp_aperture: 0.0,
            pp_focal_length_m: 0.0,
            pp_max_blur_size: 0,
            gaussian: 0.0,
            dark_current: 0.0,
            salt_pepper: 0.0,
            cosmic_rays: 0.0,
            blur_param: 0.0,
            hsv: Vector3::zeros(),
            bgr_percent: Vector3::zeros(),
            cosmic_ray_seed: None,
        }
    }
}

/// Why an update publishes no image.
#[derive(Debug)]
enum CameraError {
    /// No image could be read (upstream `BasiliskError`).
    Image(ImageError),
    /// The corrupted image could not be encoded (upstream `BasiliskError`).
    Encode,
    /// A corruption failed in OpenCV.
    OpenCv(opencv::Error),
}

impl fmt::Display for CameraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Image(error) => write!(f, "{error}"),
            Self::Encode => write!(f, "failed to encode image output buffer"),
            Self::OpenCv(error) => write!(f, "OpenCV: {error}"),
        }
    }
}

impl From<ImageError> for CameraError {
    fn from(error: ImageError) -> Self {
        Self::Image(error)
    }
}

impl From<opencv::Error> for CameraError {
    fn from(error: opencv::Error) -> Self {
        Self::OpenCv(error)
    }
}

/// Image-corrupting camera: upstream `Camera`.
#[derive(Clone, Debug)]
pub struct Camera {
    /// Module parameters.
    pub config: CameraConfig,
    /// Upstream `imageInMsg`: the rendered image.
    pub image_in_msg: Input<CameraImageMsg>,
    /// Upstream `imageOutMsg`: the corrupted image.
    pub image_out_msg: Output<CameraImageMsg>,
    /// Upstream `cameraConfigOutMsg`: the camera parameters.
    pub camera_config_out_msg: Output<CameraConfigMsg>,
    /// State of the Gaussian-noise `cv::RNG`.
    noise_rng_state: u64,
}

impl Camera {
    /// Builds the camera with an unconnected input.
    pub fn new(config: CameraConfig) -> Self {
        Self {
            config,
            image_in_msg: Input::default(),
            image_out_msg: Output::default(),
            camera_config_out_msg: Output::default(),
            noise_rng_state: OPENCV_DEFAULT_RNG_STATE,
        }
    }

    /// The camera parameters as upstream publishes them; the message fields
    /// upstream's camera has no parameter for stay zero.
    pub fn camera_config_message(&self) -> CameraConfigMsg {
        let config = &self.config;
        CameraConfigMsg {
            camera_id: config.camera_id,
            is_on: config.camera_is_on,
            parent_name: config.parent_name.clone(),
            field_of_view_rad: config.field_of_view_rad,
            resolution_px: config.resolution_px,
            render_rate_nanos: config.render_rate_nanos,
            camera_pos_b_m: config.camera_pos_b_m,
            sigma_cb: config.sigma_cb,
            sky_box: config.sky_box.clone(),
            post_processing_on: config.post_processing_on,
            pp_focus_distance_m: config.pp_focus_distance_m,
            pp_aperture: config.pp_aperture,
            pp_focal_length_m: config.pp_focal_length_m,
            pp_max_blur_size: config.pp_max_blur_size,
            ..CameraConfigMsg::default()
        }
    }

    /// Upstream `UpdateState` after the configuration message: the image to
    /// publish.
    fn capture(&mut self, current_sim_nanos: u64) -> Result<CameraImageMsg, CameraError> {
        let image_in = if self.image_in_msg.is_connected() {
            self.image_in_msg.read()
        } else {
            CameraImageMsg::default()
        };
        let Some((origin, image)) = acquire_image(
            self.config.filename.as_deref(),
            &image_in,
            current_sim_nanos,
        )?
        else {
            return Ok(CameraImageMsg::default());
        };
        let corrupted = self.apply_filters(&image, current_sim_nanos)?;
        if self.config.save_images {
            if self.config.save_dir.is_empty() {
                log::warn!(
                    "{}: save_images is set without a save_dir",
                    self.config.name
                );
            } else {
                let path = time_stamped_path(&self.config.save_dir, current_sim_nanos, ".png");
                save_debug_image(&self.config.name, &path, &corrupted);
            }
        }

        let mut buffer = Vector::<u8>::new();
        let no_compression = Vector::from_slice(&[imgcodecs::IMWRITE_PNG_COMPRESSION, 0]);
        if !imgcodecs::imencode(".png", &corrupted, &mut buffer, &no_compression)?
            || buffer.is_empty()
        {
            return Err(CameraError::Encode);
        }
        let (time_tag_nanos, camera_id, image_type) = match origin {
            ImageOrigin::File => (
                current_sim_nanos,
                self.config.camera_id,
                corrupted.channels() as i8,
            ),
            ImageOrigin::Message => (
                image_in.time_tag_nanos,
                image_in.camera_id,
                image_in.image_type,
            ),
        };
        Ok(CameraImageMsg {
            time_tag_nanos,
            valid: true,
            camera_id,
            image: buffer.to_vec(),
            image_type,
        })
    }

    /// Upstream `applyFilters`: every configured corruption, in order.
    fn apply_filters(&mut self, source: &Mat, current_sim_nanos: u64) -> opencv::Result<Mat> {
        let config = &self.config;
        let rng_state = &mut self.noise_rng_state;
        let mut image = source.try_clone()?;
        if config.gaussian > 0.0 {
            let noisy = add_gaussian_noise(
                rng_state,
                &image,
                0.0,
                config.gaussian * GAUSSIAN_NOISE_SCALE,
            )?;
            imgproc::threshold(
                &noisy,
                &mut image,
                config.gaussian * GAUSSIAN_THRESHOLD_SCALE,
                255.0,
                imgproc::THRESH_TOZERO,
            )?;
        }
        if config.blur_param > 0.0 {
            let mut blur_size = config.blur_param.round() as i32;
            if blur_size % 2 == 0 {
                blur_size += 1;
            }
            let unblurred = image.try_clone()?;
            imgproc::blur(
                &unblurred,
                &mut image,
                Size::new(blur_size, blur_size),
                Point::new(-1, -1),
                cv_core::BORDER_DEFAULT,
            )?;
        }
        if config.dark_current > 0.0 {
            image = add_gaussian_noise(
                rng_state,
                &image,
                config.dark_current * DARK_CURRENT_SCALE,
                0.0,
            )?;
        }
        if config.hsv.abs().sum() > 0.00001 {
            image = hsv_adjust(&image, &config.hsv)?;
        }
        if config.bgr_percent.abs().sum() != 0.0 {
            bgr_adjust_percent(&mut image, &config.bgr_percent)?;
        }
        if config.salt_pepper > 0.0 {
            let probability = (config.salt_pepper * SALT_PEPPER_SCALE) as f32;
            add_salt_pepper(&mut image, probability, probability)?;
        }
        if config.cosmic_rays > 0.0 {
            add_cosmic_ray_burst(
                &mut image,
                config.cosmic_rays.round(),
                current_sim_nanos,
                config.cosmic_ray_seed,
            )?;
        }
        Ok(image)
    }
}

/// Upstream `addGaussianNoise`: normal noise of `mean` and `std_dev` from
/// the generator in `rng_state`, added in signed 16-bit and saturated back to
/// the source type.
fn add_gaussian_noise(
    rng_state: &mut u64,
    source: &Mat,
    mean: f64,
    std_dev: f64,
) -> opencv::Result<Mat> {
    let mut noise =
        Mat::new_size_with_default(source.size()?, cv_core::CV_16SC3, Scalar::all(0.0))?;
    let mut rng = RNG::new(*rng_state)?;
    rng.fill(
        &mut noise,
        cv_core::RNG_NORMAL,
        &Scalar::all(mean),
        &Scalar::all(std_dev),
        false,
    )?;
    *rng_state = rng.state();

    let mut source_16s = Mat::default();
    source.convert_to(&mut source_16s, cv_core::CV_16SC3, 1.0, 0.0)?;
    let mut sum = Mat::default();
    cv_core::add_weighted(&source_16s, 1.0, &noise, 1.0, 0.0, &mut sum, -1)?;
    let mut noisy = Mat::default();
    sum.convert_to(&mut noisy, source.typ(), 1.0, 0.0)?;
    Ok(noisy)
}

/// Upstream `addCosmicRayBurst`: `count` attempts, each drawing a ray with
/// probability `1 - 1 / count²`.
fn add_cosmic_ray_burst(
    image: &mut Mat,
    count: f64,
    current_sim_nanos: u64,
    seed: Option<u64>,
) -> opencv::Result<()> {
    let probability_threshold = (1.0 / count.powi(2)) as f32;
    for attempt in 0..count.round() as i32 {
        add_cosmic_ray(
            image,
            probability_threshold,
            f64::from(attempt + 1),
            COSMIC_RAY_MAX_SIZE_PX,
            current_sim_nanos,
            seed,
        )?;
    }
    Ok(())
}

/// Upstream `addCosmicRay`: with probability `1 - probability_threshold`, a
/// white line from a random pixel to one within `max_size / 2` pixels in each
/// direction. The generator is seeded with the simulation time, the offset
/// and `seed`, or the wall-clock seconds without one.
fn add_cosmic_ray(
    image: &mut Mat,
    probability_threshold: f32,
    random_offset: f64,
    max_size: i32,
    current_sim_nanos: u64,
    seed: Option<u64>,
) -> opencv::Result<()> {
    let clock = seed.unwrap_or_else(wall_clock_s);
    let mut rng = RNG::new(
        current_sim_nanos
            .wrapping_add(clock)
            .wrapping_add(random_offset as u64),
    )?;
    let probability = rng.uniform_f64(0.0, 1.0)? as f32;
    if probability > probability_threshold {
        // Upstream draws x over the rows and y over the columns.
        let x = rng.uniform(0, image.rows())?;
        let y = rng.uniform(0, image.cols())?;
        let delta_x = rng.uniform(-max_size / 2, max_size / 2)?;
        let delta_y = rng.uniform(-max_size / 2, max_size / 2)?;
        imgproc::line(
            image,
            Point::new(x, y),
            Point::new(x + delta_x, y + delta_y),
            Scalar::new(255.0, 255.0, 255.0, 0.0),
            1,
            imgproc::LINE_8,
            0,
        )?;
    }
    Ok(())
}

/// Upstream `hsvAdjust`: rotates the hue by `hsv[0]` radians and scales
/// saturation and value by `1 + hsv[k] / 100`.
fn hsv_adjust(source: &Mat, hsv: &Vector3<f64>) -> opencv::Result<Mat> {
    let mut local_hsv = Mat::default();
    imgproc::cvt_color_def(source, &mut local_hsv, imgproc::COLOR_BGR2HSV)?;
    // OpenCV stores the hue in [0, 180), half degrees; the shift is
    // truncated to whole degrees as upstream.
    let input_degrees = hsv[0].to_degrees() as i32;
    for pixel in local_hsv.data_typed_mut::<Vec3b>()? {
        let mut h_360 = i32::from(pixel[0]) * 2 + input_degrees;
        h_360 = (f64::from(h_360) - 360.0 * (f64::from(h_360) * (1.0 / 360.0)).floor()) as i32;
        h_360 /= 2;
        if h_360 == 180 {
            h_360 = 0;
        }
        pixel[0] = h_360 as u8;
        for k in 1..3 {
            pixel[k] = scale_channel(pixel[k], hsv[k]);
        }
    }
    let mut adjusted = Mat::default();
    imgproc::cvt_color_def(&local_hsv, &mut adjusted, imgproc::COLOR_HSV2BGR)?;
    Ok(adjusted)
}

/// Upstream `bgrAdjustPercent`: scales each channel by
/// `1 + bgr_percent[k] / 100`.
fn bgr_adjust_percent(image: &mut Mat, bgr_percent: &Vector3<f64>) -> opencv::Result<()> {
    if !image.is_continuous() {
        *image = image.try_clone()?;
    }
    for pixel in image.data_typed_mut::<Vec3b>()? {
        for k in 0..3 {
            pixel[k] = scale_channel(pixel[k], bgr_percent[k]);
        }
    }
    Ok(())
}

/// `value · (1 + percent / 100)`, truncated and clamped to `[0, 255]`.
fn scale_channel(value: u8, percent: f64) -> u8 {
    ((f64::from(value) * (percent / 100.0 + 1.0)) as i32).clamp(0, 255) as u8
}

/// Upstream `addSaltPepper`: `rows · cols · dead_probability` black and
/// `rows · cols · hot_probability` white pixels, from a generator in its
/// default state, so the same pixels every frame.
fn add_salt_pepper(
    image: &mut Mat,
    dead_probability: f32,
    hot_probability: f32,
) -> opencv::Result<()> {
    let mut rng = RNG::default()?;
    let (rows, cols) = (image.rows(), image.cols());
    let pixels = rows as f32 * cols as f32;
    let dead = (pixels * dead_probability) as i32;
    let hot = (pixels * hot_probability) as i32;
    for (count, value) in [(dead, 0_u8), (hot, 255_u8)] {
        for _ in 0..count {
            let row = rng.uniform(0, rows)?;
            let col = rng.uniform(0, cols)?;
            *image.at_2d_mut::<Vec3b>(row, col)? = Vec3b::all(value);
        }
    }
    Ok(())
}

/// Wall-clock time \[s\] since the Unix epoch, upstream's `time(nullptr)`.
fn wall_clock_s() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |elapsed| elapsed.as_secs())
}

impl Module for Camera {
    fn init(&mut self) {
        self.noise_rng_state = OPENCV_DEFAULT_RNG_STATE;
        self.image_out_msg.write(CameraImageMsg::default());
        self.camera_config_out_msg.write(CameraConfigMsg::default());
    }

    /// Upstream's `Reset` does nothing; the noise generator restarts here.
    fn reset(&mut self, _context: &SimulationContext) {
        self.noise_rng_state = OPENCV_DEFAULT_RNG_STATE;
    }

    fn update(&mut self, context: &SimulationContext) {
        self.camera_config_out_msg
            .write(self.camera_config_message());
        match self.capture(context.current_sim_nanos) {
            Ok(image) => self.image_out_msg.write(image),
            // Upstream raises `BasiliskError` here, before writing the image.
            Err(error) => log::error!("{}: {error}", self.config.name),
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports of `test_camera.py` and `test_colorAdjust.py`, on their images
    //! (`just fetch-opencv-assets`).

    use std::path::Path;

    use hifitime::Epoch;

    use super::*;
    use crate::utilities::opencv_image::test_support::{asset, assets_present, scratch_dir};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `macros.sec2nano`.
    fn sec2nano(seconds: f64) -> u64 {
        (seconds * 1.0e9 + 0.5) as u64
    }

    fn read_image(path: &Path) -> Mat {
        let image = imgcodecs::imread(path, imgcodecs::IMREAD_COLOR).unwrap();
        assert!(!image.empty(), "cannot read {}", path.display());
        image
    }

    fn decode(message: &CameraImageMsg) -> Mat {
        imgcodecs::imdecode(&message.image.as_slice(), imgcodecs::IMREAD_COLOR).unwrap()
    }

    fn pixels(image: &Mat) -> Vec<Vec3b> {
        image
            .try_clone()
            .unwrap()
            .data_typed::<Vec3b>()
            .unwrap()
            .to_vec()
    }

    /// Runs `camera` from 0 to `stop_nanos` at the pytests' 0.5 s rate.
    fn run(camera: &mut Camera, stop_nanos: u64) {
        camera.init();
        let step = sec2nano(0.5);
        let mut time = 0;
        while time <= stop_nanos {
            camera.update(&context(time));
            time += step;
        }
    }

    /// The input message the pytests connect (and never use: they read a
    /// file).
    fn pytest_input(camera: &mut Camera) -> Output<CameraImageMsg> {
        let input = Output::new(CameraImageMsg {
            time_tag_nanos: sec2nano(1.0),
            camera_id: 1,
            ..CameraImageMsg::default()
        });
        input.connect_to(&mut camera.image_in_msg);
        input
    }

    #[test]
    fn no_image_zero_msg() {
        let mut camera = Camera::new(CameraConfig {
            name: "cameras".to_string(),
            camera_is_on: true,
            ..CameraConfig::default()
        });
        run(&mut camera, sec2nano(0.5));
        let output = camera.image_out_msg.read();
        assert!(!output.valid);
        assert!(output.image.is_empty());
        assert_eq!(output.time_tag_nanos, 0);
        assert_eq!(output.camera_id, 0);
        assert_eq!(output.image_type, 0);
        // The configuration is published without an image.
        assert!(camera.camera_config_out_msg.read().is_on);
    }

    #[test]
    fn filename_metadata() {
        let path = asset("camera/mars.jpg");
        if !assets_present(&[&path]) {
            return;
        }
        let mut camera = Camera::new(CameraConfig {
            name: "cameras".to_string(),
            filename: Some(path),
            camera_id: 42,
            camera_is_on: true,
            ..CameraConfig::default()
        });
        let sim_end_time = sec2nano(0.5);
        run(&mut camera, sim_end_time);
        let output = camera.image_out_msg.read();
        assert!(output.valid);
        assert_eq!(output.camera_id, 42);
        assert_eq!(output.image_type, 3);
        assert!(output.time_tag_nanos.abs_diff(sim_end_time) <= sec2nano(0.1));
        assert!(!output.image.is_empty());
    }

    #[test]
    fn invalid_filename_error() {
        let mut camera = Camera::new(CameraConfig {
            name: "cameras".to_string(),
            filename: Some(PathBuf::from("/nonexistent/path/to/image.png")),
            camera_is_on: true,
            ..CameraConfig::default()
        });
        camera.init();
        assert!(matches!(
            camera.capture(0),
            Err(CameraError::Image(ImageError::Load(_)))
        ));
        // The configuration is still published; the image is not.
        camera.update(&context(0));
        assert!(camera.camera_config_out_msg.read().is_on);
        assert_eq!(camera.image_out_msg.read(), CameraImageMsg::default());
    }

    /// The pytest's `cameraTest`: corrupt `mars.jpg`, read back the saved
    /// image and compare it with the original.
    fn camera_test(gauss: f64, dark_current: f64, salt_pepper: f64, cosmic: f64, blur_size: f64) {
        let image_path = asset("camera/mars.jpg");
        if !assets_present(&[&image_path]) {
            return;
        }
        let input_image = read_image(&image_path);
        let corrupted = gauss > 0.0
            || dark_current > 0.0
            || salt_pepper > 0.0
            || cosmic > 0.0
            || blur_size > 0.0;
        let tag = format!("{gauss}{dark_current}{salt_pepper}{cosmic}{blur_size}");
        let dir = scratch_dir(&format!("camera-{tag}"));
        let save_dir = format!("{}/{tag}", dir.display());

        let mut camera = Camera::new(CameraConfig {
            name: "cameras".to_string(),
            filename: Some(image_path),
            save_images: true,
            save_dir: save_dir.clone(),
            camera_is_on: true,
            sigma_cb: Vector3::new(0.0, 0.0, 1.0),
            gaussian: gauss,
            dark_current,
            salt_pepper,
            cosmic_rays: cosmic,
            blur_param: blur_size,
            ..CameraConfig::default()
        });
        let _input = pytest_input(&mut camera);
        run(&mut camera, sec2nano(0.5));

        let corrupted_path = if corrupted {
            format!("{save_dir}0.000000.png")
        } else {
            format!("{save_dir}0.500000.png")
        };
        let output_image = read_image(Path::new(&corrupted_path));

        // `‖ ‖pixel_in‖ − ‖pixel_out‖ ‖ / ‖ ‖pixel_in‖ ‖` over the image, the
        // per-pixel norms taken over the colour channels.
        let norms = |image: &Mat| -> Vec<f64> {
            pixels(image)
                .iter()
                .map(|p| p.iter().map(|&c| f64::from(c).powi(2)).sum::<f64>().sqrt())
                .collect()
        };
        let (input_norms, output_norms) = (norms(&input_image), norms(&output_image));
        let difference: f64 = input_norms
            .iter()
            .zip(&output_norms)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt();
        let reference = input_norms.iter().map(|a| a * a).sum::<f64>().sqrt();
        let err = difference / reference;
        eprintln!("camera {tag}: relative image change {err}");

        assert!(
            !(err < 1e-2 && corrupted),
            "Image not corrupted and show be: mars.jpg"
        );
        assert!(
            !(err > 1e-2 && !corrupted),
            "Image corrupted and show not be: mars.jpg"
        );
        let config = camera.camera_config_out_msg.read();
        for i in 0..3 {
            assert!((config.sigma_cb[i] - camera.config.sigma_cb[i]).abs() <= 1e-10);
        }
        assert!(config.is_on);
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn module_without_corruption() {
        camera_test(0.0, 0.0, 0.0, 0.0, 0.0);
    }

    #[test]
    fn module_with_corruption() {
        camera_test(2.0, 2.0, 2.0, 1.0, 3.0);
    }

    /// `colorsys.rgb_to_hsv`, scaled as the pytest's `rgb_to_hsv`: hue in
    /// `[0, 180)`, saturation in `[0, 255]`, value as given.
    fn rgb_to_hsv(rgb: [f64; 3]) -> [f64; 3] {
        let [r, g, b] = rgb;
        let max_c = r.max(g).max(b);
        let min_c = r.min(g).min(b);
        let (h, s) = if min_c == max_c {
            (0.0, 0.0)
        } else {
            let range_c = max_c - min_c;
            let (rc, gc, bc) = (
                (max_c - r) / range_c,
                (max_c - g) / range_c,
                (max_c - b) / range_c,
            );
            let h = if r == max_c {
                bc - gc
            } else if g == max_c {
                2.0 + rc - bc
            } else {
                4.0 + gc - rc
            };
            ((h / 6.0).rem_euclid(1.0), range_c / max_c)
        };
        [h * 180.0, s * 255.0, max_c]
    }

    /// `colorsys.hsv_to_rgb`, scaled as the pytest's `hsv_to_rgb`.
    fn hsv_to_rgb(hsv: [i32; 3]) -> [f64; 3] {
        let (h, s, v) = (
            f64::from(hsv[0]) / 180.0,
            f64::from(hsv[1]) / 255.0,
            f64::from(hsv[2]) / 255.0,
        );
        let rgb = if s == 0.0 {
            [v, v, v]
        } else {
            let i = (h * 6.0) as i32;
            let f = h * 6.0 - f64::from(i);
            let p = v * (1.0 - s);
            let q = v * (1.0 - s * f);
            let t = v * (1.0 - s * (1.0 - f));
            match i % 6 {
                0 => [v, t, p],
                1 => [q, v, p],
                2 => [p, v, t],
                3 => [p, q, v],
                4 => [t, p, v],
                _ => [v, p, q],
            }
        };
        rgb.map(|c| c * 255.0)
    }

    /// The pytest's `trueColorAdjust`: the expected RGB of each test point
    /// after the HSV then BGR adjustment, within 3 of the camera's.
    fn true_color_adjust(input: &Mat, output: &Mat, hsv: [f64; 3], bgr: [f64; 3]) -> bool {
        // These points correspond to the included `tv_test.png`, as (x, y).
        let test_points = [
            (100, 300),
            (250, 300),
            (450, 300),
            (600, 300),
            (700, 300),
            (950, 300),
            (1100, 300),
            (300, 800),
            (880, 780),
        ];
        for (px, py) in test_points {
            let bgr_in = input.at_2d::<Vec3b>(py, px).unwrap();
            let rgb_in = [bgr_in[2], bgr_in[1], bgr_in[0]].map(f64::from);

            let pixel_hsv = rgb_to_hsv(rgb_in);
            let mut expected = [0_i32; 3];
            let input_degrees = hsv[0].to_degrees();
            let mut h_360 = pixel_hsv[0] * 2.0 + input_degrees;
            h_360 -= 360.0 * (h_360 * (1.0 / 360.0)).floor();
            let mut h_360 = (h_360 / 2.0) as i32;
            if h_360 == 180 {
                h_360 = 0;
            }
            expected[0] = h_360;
            for i in 0..2 {
                expected[i + 1] =
                    ((pixel_hsv[i + 1] * (hsv[i + 1] / 100.0 + 1.0)) as i32).clamp(0, 255);
            }
            let after_hsv = hsv_to_rgb(expected).map(|c| c as i32);

            for i in 0..3 {
                expected[i] =
                    (((bgr[2 - i] / 100.0 + 1.0) * f64::from(after_hsv[i])) as i32).clamp(0, 255);
            }
            let bgr_out = output.at_2d::<Vec3b>(py, px).unwrap();
            let rgb_out = [bgr_out[2], bgr_out[1], bgr_out[0]];
            for i in 0..3 {
                if (i32::from(rgb_out[i]) - expected[i]).abs() > 3 {
                    eprintln!("Failed HSV at point: px={px} py={py}");
                    return false;
                }
            }
        }
        true
    }

    #[test]
    // The pytest's hue shift of 3.14159 rad, not π.
    #[allow(clippy::approx_constant)]
    fn color_adjust() {
        let image_path = asset("camera/tv_test.png");
        if !assets_present(&[&image_path]) {
            return;
        }
        let input_image = read_image(&image_path);
        let hsv_cases = [
            [0.0, 0.0, 0.0],
            [1.0, 20.0, -30.0],
            [-1.0, 20.0, -30.0],
            [3.14159, 100.0, -100.0],
        ];
        let bgr_cases = [
            [0.0, 0.0, 0.0],
            [10.0, 20.0, 30.0],
            [-10.0, -30.0, 50.0],
            [-100.0, 200.0, 20.0],
        ];
        for hsv in hsv_cases {
            for bgr in bgr_cases {
                let mut camera = Camera::new(CameraConfig {
                    name: "cameras".to_string(),
                    filename: Some(image_path.clone()),
                    camera_is_on: true,
                    sigma_cb: Vector3::new(0.0, 0.0, 1.0),
                    hsv: Vector3::from(hsv),
                    bgr_percent: Vector3::from(bgr),
                    ..CameraConfig::default()
                });
                let _input = pytest_input(&mut camera);
                // The pytest's single `SingleStepProcesses`, at time zero.
                camera.init();
                camera.update(&context(0));
                let output = decode(&camera.image_out_msg.read());
                assert!(
                    true_color_adjust(&input_image, &output, hsv, bgr),
                    "Test failed color adjustment tv_test.png: hsv {hsv:?} bgr {bgr:?}"
                );
            }
        }
    }

    /// A mid-grey image, sent through the input message.
    fn grey_input(camera: &mut Camera, time_tag_nanos: u64) -> Output<CameraImageMsg> {
        let image = Mat::new_rows_cols_with_default(120, 160, cv_core::CV_8UC3, Scalar::all(100.0))
            .unwrap();
        let mut png = Vector::<u8>::new();
        imgcodecs::imencode_def(".png", &image, &mut png).unwrap();
        let input = Output::new(CameraImageMsg {
            time_tag_nanos,
            valid: true,
            camera_id: 5,
            image: png.to_vec(),
            image_type: 4,
        });
        input.connect_to(&mut camera.image_in_msg);
        input
    }

    #[test]
    fn message_image_keeps_its_metadata() {
        let mut camera = Camera::new(CameraConfig {
            camera_id: 42,
            ..CameraConfig::default()
        });
        let _input = grey_input(&mut camera, 3_000);
        camera.init();
        camera.update(&context(3_000));
        let output = camera.image_out_msg.read();
        assert!(output.valid);
        assert_eq!(
            (output.time_tag_nanos, output.camera_id, output.image_type),
            (3_000, 5, 4)
        );
        let image = decode(&output);
        assert_eq!((image.rows(), image.cols()), (120, 160));
        assert!(pixels(&image).iter().all(|p| *p == Vec3b::all(100)));

        // An image older than the current time is not processed.
        camera.update(&context(3_001));
        assert_eq!(camera.image_out_msg.read(), CameraImageMsg::default());
    }

    #[test]
    fn camera_config_message_carries_the_parameters() {
        let camera = Camera::new(CameraConfig {
            parent_name: "bsk-Sat".to_string(),
            camera_is_on: true,
            camera_id: 2,
            resolution_px: [640, 480],
            render_rate_nanos: 5,
            field_of_view_rad: 0.3,
            camera_pos_b_m: Vector3::new(1.0, 2.0, 3.0),
            sigma_cb: Vector3::new(0.1, 0.2, 0.3),
            sky_box: "ESO".to_string(),
            post_processing_on: VizSetting::Enabled,
            pp_focus_distance_m: 4.0,
            pp_aperture: 5.0,
            pp_focal_length_m: 0.1,
            pp_max_blur_size: 3,
            ..CameraConfig::default()
        });
        assert_eq!(
            camera.camera_config_message(),
            CameraConfigMsg {
                camera_id: 2,
                is_on: true,
                parent_name: "bsk-Sat".to_string(),
                field_of_view_rad: 0.3,
                resolution_px: [640, 480],
                render_rate_nanos: 5,
                camera_pos_b_m: Vector3::new(1.0, 2.0, 3.0),
                sigma_cb: Vector3::new(0.1, 0.2, 0.3),
                sky_box: "ESO".to_string(),
                post_processing_on: VizSetting::Enabled,
                pp_focus_distance_m: 4.0,
                pp_aperture: 5.0,
                pp_focal_length_m: 0.1,
                pp_max_blur_size: 3,
                ..CameraConfigMsg::default()
            }
        );
        let defaults = Camera::new(CameraConfig::default()).camera_config_message();
        assert_eq!(
            (
                defaults.camera_id,
                defaults.resolution_px,
                defaults.render_rate_nanos
            ),
            (1, [512, 512], 60_000_000_000)
        );
        assert_eq!(
            (defaults.field_of_view_rad, defaults.sky_box.as_str()),
            (0.7, "black")
        );
    }

    #[test]
    fn dark_current_is_a_constant_offset() {
        let mut camera = Camera::new(CameraConfig {
            dark_current: 2.0,
            ..CameraConfig::default()
        });
        let _input = grey_input(&mut camera, 0);
        camera.init();
        camera.update(&context(0));
        let image = decode(&camera.image_out_msg.read());
        assert!(pixels(&image).iter().all(|p| *p == Vec3b::all(130)));
    }

    #[test]
    fn gaussian_noise_is_thresholded_and_repeats_after_reset() {
        let mut camera = Camera::new(CameraConfig {
            gaussian: 5.0,
            ..CameraConfig::default()
        });
        let _input = grey_input(&mut camera, 0);
        camera.init();
        camera.update(&context(0));
        let first = camera.image_out_msg.read();
        let noisy = pixels(&decode(&first));
        // Noise of sigma 10 about 100, everything at or under 30 zeroed.
        let mean = noisy.iter().map(|p| f64::from(p[0])).sum::<f64>() / noisy.len() as f64;
        assert!((mean - 100.0).abs() < 1.0, "mean {mean}");
        assert!(noisy.iter().any(|p| p[0] != 100));
        assert!(
            noisy
                .iter()
                .flat_map(|p| p.iter())
                .all(|&c| c == 0 || c > 30)
        );

        // The generator moves on between frames, and restarts at reset.
        camera.update(&context(0));
        assert_ne!(camera.image_out_msg.read().image, first.image);
        camera.reset(&context(0));
        camera.update(&context(0));
        assert_eq!(camera.image_out_msg.read().image, first.image);
    }

    #[test]
    fn even_blur_size_is_made_odd() {
        // A single white pixel spreads over a 5 x 5 box for blur 4.
        let mut image =
            Mat::new_rows_cols_with_default(21, 21, cv_core::CV_8UC3, Scalar::all(0.0)).unwrap();
        *image.at_2d_mut::<Vec3b>(10, 10).unwrap() = Vec3b::all(250);
        let mut camera = Camera::new(CameraConfig {
            blur_param: 4.0,
            ..CameraConfig::default()
        });
        let blurred = camera.apply_filters(&image, 0).unwrap();
        let lit: Vec<(i32, i32)> = (0..21)
            .flat_map(|r| (0..21).map(move |c| (r, c)))
            .filter(|&(r, c)| blurred.at_2d::<Vec3b>(r, c).unwrap()[0] > 0)
            .collect();
        assert_eq!(lit.len(), 25);
        assert_eq!(*blurred.at_2d::<Vec3b>(8, 8).unwrap(), Vec3b::all(10));
    }

    #[test]
    fn stuck_pixels_are_the_same_every_frame() {
        let mut camera = Camera::new(CameraConfig {
            salt_pepper: 500.0,
            ..CameraConfig::default()
        });
        let _input = grey_input(&mut camera, 0);
        camera.init();
        camera.update(&context(0));
        let first = camera.image_out_msg.read();
        let image = pixels(&decode(&first));
        // 2e-5 · 500 = 1 % of 120 · 160 pixels, each dead and hot (fewer if
        // a pixel is drawn twice).
        let dead = image.iter().filter(|p| **p == Vec3b::all(0)).count();
        let hot = image.iter().filter(|p| **p == Vec3b::all(255)).count();
        assert!(dead > 150 && dead <= 192, "{dead} dead pixels");
        assert!(hot > 150 && hot <= 192, "{hot} hot pixels");
        assert_eq!(
            dead + hot + image.iter().filter(|p| **p == Vec3b::all(100)).count(),
            image.len()
        );
        camera.update(&context(0));
        assert_eq!(camera.image_out_msg.read().image, first.image);
    }

    #[test]
    fn cosmic_rays_follow_their_seed() {
        let image =
            Mat::new_rows_cols_with_default(200, 200, cv_core::CV_8UC3, Scalar::all(0.0)).unwrap();
        let rays = |seed: u64, count: f64, time: u64| {
            let mut camera = Camera::new(CameraConfig {
                cosmic_rays: count,
                cosmic_ray_seed: Some(seed),
                ..CameraConfig::default()
            });
            pixels(&camera.apply_filters(&image, time).unwrap())
        };
        let lit = |image: &[Vec3b]| image.iter().filter(|p| **p == Vec3b::all(255)).count();

        // Ten attempts, each drawing a ray with probability 0.99.
        let first = rays(7, 10.0, 0);
        assert!(lit(&first) > 0);
        assert!(
            first
                .iter()
                .all(|p| *p == Vec3b::all(0) || *p == Vec3b::all(255))
        );
        assert_eq!(rays(7, 10.0, 0), first);
        assert_ne!(rays(7, 10.0, 1_000), first);
        // A single attempt draws with probability 1 - 1 / 1² = 0.
        assert_eq!(lit(&rays(7, 1.0, 0)), 0);
    }

    #[test]
    fn hsv_hue_rotation_wraps() {
        // Pure blue (hue 240°) rotated by just over π (180 whole degrees):
        // 60°, yellow.
        let image = Mat::new_rows_cols_with_default(
            2,
            2,
            cv_core::CV_8UC3,
            Scalar::new(255.0, 0.0, 0.0, 0.0),
        )
        .unwrap();
        let mut camera = Camera::new(CameraConfig {
            hsv: Vector3::new(std::f64::consts::PI + 1e-3, 0.0, 0.0),
            ..CameraConfig::default()
        });
        let rotated = camera.apply_filters(&image, 0).unwrap();
        assert_eq!(
            *rotated.at_2d::<Vec3b>(0, 0).unwrap(),
            Vec3b::from([0, 255, 255])
        );

        // Saturation and value are clamped to [0, 255]: -100 % greys out.
        camera.config.hsv = Vector3::new(0.0, -100.0, 50.0);
        let grey = camera.apply_filters(&image, 0).unwrap();
        assert_eq!(*grey.at_2d::<Vec3b>(1, 1).unwrap(), Vec3b::all(255));
    }
}
