//! Planet limb points in a camera image from OpenCV's Canny edge detector:
//! port of upstream Basilisk `limbFinding` (feature `opencv`).
//!
//! The image (see [`acquire_image`]: a configured file, else the input
//! message's encoded image) is converted to grey, Gaussian-blurred (`sigma`
//! 1 px) and run through `cv::Canny` with an L2 gradient. If more than
//! `limb_num_thresh` edge pixels are found, the first [`MAX_LIMB_PNTS`] of
//! them, in row-major order, are published as limb points and the message is
//! valid, naming Mars (planet ID 2). Without an image the message is zeroed.
//!
//! Header stand-in: upstream time-tags the limb with the input message's
//! write time (`imageInMsg.timeWritten()`); here it is the producer's payload
//! time stamp, [`CameraImageMsg::time_tag_nanos`], read whenever the input is
//! connected and kept otherwise. As upstream, that nanosecond count is
//! written unconverted into [`OpNavLimbMsg::time_tag_s`] (and passed on so by
//! `horizonOpNav`), as `pixelLineConverter` does with its circle time tag.

use std::path::PathBuf;

use nalgebra::Vector2;
use opencv::core::{self as cv_core, Mat, Point, Size, Vector};
use opencv::imgproc;

use crate::messages::{CameraImageMsg, Input, MAX_LIMB_PNTS, OpNavLimbMsg, Output};
use crate::utilities::opencv_image::{
    ImageError, ImageOrigin, acquire_image, save_debug_image, time_stamped_path,
};
use crate::{Module, SimulationContext};

/// Upstream's planet ID of the limb: Mars.
const LIMB_PLANET_ID: f64 = 2.0;

/// Upstream's Gaussian blur standard deviation \[pixels\].
const BLUR_SIGMA_PX: f64 = 1.0;

/// Upstream's Sobel aperture of the Canny detector.
const CANNY_APERTURE: i32 = 3;

/// User parameters of [`LimbFinding`]; the defaults are upstream's
/// constructor values.
#[derive(Clone, Debug)]
pub struct LimbFindingConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `filename`: read the image from this file instead of the
    /// input message (upstream's empty string is `None`).
    pub filename: Option<PathBuf>,
    /// Upstream `saveDir`: prefix of the debug image path, which continues
    /// with the time in seconds and `.jpg`.
    pub save_dir: String,
    /// Upstream `saveImages` (`int`): save each decoded message image.
    pub save_images: bool,
    /// Upstream `blurrSize` \[pixels\]: side of the (odd) Gaussian kernel.
    pub blur_size_px: i32,
    /// Upstream `cannyThreshHigh` [-]: higher Canny hysteresis threshold.
    pub canny_thresh_high: i32,
    /// Upstream `cannyThreshLow` [-]: lower Canny hysteresis threshold.
    pub canny_thresh_low: i32,
    /// Upstream `limbNumThresh` [-]: edge pixels needed for a limb to count
    /// as detected.
    pub limb_num_thresh: i32,
}

impl Default for LimbFindingConfig {
    fn default() -> Self {
        Self {
            name: "limbFinding".to_string(),
            filename: None,
            save_dir: String::new(),
            save_images: false,
            blur_size_px: 3,
            canny_thresh_high: 200,
            canny_thresh_low: 100,
            limb_num_thresh: 50,
        }
    }
}

/// Canny-edge limb finder: upstream `LimbFinding`.
#[derive(Clone, Debug)]
pub struct LimbFinding {
    /// Module configuration.
    pub config: LimbFindingConfig,
    /// Upstream `imageInMsg`: the camera image.
    pub image_in_msg: Input<CameraImageMsg>,
    /// Upstream `opnavLimbOutMsg`: the limb points found.
    pub opnav_limb_out_msg: Output<OpNavLimbMsg>,
    /// Upstream `sensorTimeTag` \[ns\]: time tag of the last image read.
    sensor_time_tag_nanos: u64,
}

impl LimbFinding {
    /// Builds the module with an unconnected input.
    pub fn new(config: LimbFindingConfig) -> Self {
        Self {
            config,
            image_in_msg: Input::default(),
            opnav_limb_out_msg: Output::default(),
            sensor_time_tag_nanos: 0,
        }
    }

    /// Upstream `Reset`: check that the image input is connected.
    pub fn reset(&mut self) {
        if !self.image_in_msg.is_connected() {
            log::error!("{}.image_in_msg is not connected", self.config.name);
        }
    }

    /// Upstream `UpdateState`: the limb to publish.
    fn find_limb(&mut self, current_sim_nanos: u64) -> Result<OpNavLimbMsg, ImageError> {
        let image_in = if self.image_in_msg.is_connected() {
            let image_in = self.image_in_msg.read();
            self.sensor_time_tag_nanos = image_in.time_tag_nanos;
            image_in
        } else {
            CameraImageMsg::default()
        };
        let Some((origin, image)) = acquire_image(
            self.config.filename.as_deref(),
            &image_in,
            current_sim_nanos,
        )?
        else {
            return Ok(OpNavLimbMsg::default());
        };
        if origin == ImageOrigin::Message && self.config.save_images {
            let path = time_stamped_path(&self.config.save_dir, current_sim_nanos, ".jpg");
            save_debug_image(&self.config.name, &path, &image);
        }

        let mut grey = Mat::default();
        imgproc::cvt_color_def(&image, &mut grey, imgproc::COLOR_BGR2GRAY)?;
        let mut blurred = Mat::default();
        let blur = self.config.blur_size_px;
        imgproc::gaussian_blur_def(&grey, &mut blurred, Size::new(blur, blur), BLUR_SIGMA_PX)?;
        let mut edges = Mat::default();
        imgproc::canny(
            &blurred,
            &mut edges,
            f64::from(self.config.canny_thresh_low),
            f64::from(self.config.canny_thresh_high),
            CANNY_APERTURE,
            true,
        )?;

        let mut message = OpNavLimbMsg {
            // Nanoseconds, as upstream (see the module docs).
            time_tag_s: self.sensor_time_tag_nanos as f64,
            camera_id: image_in.camera_id,
            ..OpNavLimbMsg::default()
        };
        if cv_core::count_non_zero(&edges)? > self.config.limb_num_thresh {
            let mut locations = Vector::<Point>::new();
            cv_core::find_non_zero(&edges, &mut locations)?;
            for (slot, location) in message.limb_points_px.iter_mut().zip(locations.iter()) {
                *slot = Vector2::new(f64::from(location.x), f64::from(location.y));
            }
            message.num_limb_points = locations.len().min(MAX_LIMB_PNTS);
            message.valid = true;
            message.planet_id = LIMB_PLANET_ID;
        }
        Ok(message)
    }
}

impl Module for LimbFinding {
    fn init(&mut self) {
        self.sensor_time_tag_nanos = 0;
        self.opnav_limb_out_msg.write(OpNavLimbMsg::default());
        LimbFinding::reset(self);
    }

    fn reset(&mut self, _context: &SimulationContext) {
        LimbFinding::reset(self);
    }

    fn update(&mut self, context: &SimulationContext) {
        match self.find_limb(context.current_sim_nanos) {
            Ok(limb) => self.opnav_limb_out_msg.write(limb),
            // Upstream's OpenCV call throws here, before writing: the output
            // keeps its previous value.
            Err(error) => log::error!("{}: {error}", self.config.name),
        }
    }
}

#[cfg(test)]
mod tests {
    //! Port of `test_limbFinding.py`, on its three images (`just
    //! fetch-opencv-assets`).

    use hifitime::Epoch;
    use opencv::imgcodecs;
    use opencv::prelude::*;

    use super::*;
    use crate::utilities::opencv_image::test_support::{asset, assets_present};

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

    /// The pytest's `limbFindingTest`: run 2 s at 0.5 s and compare the first
    /// limb point and the point count of the last output to the reference.
    fn limb_finding_test(
        image: &str,
        blur: i32,
        canny_low: i32,
        canny_high: i32,
        reference: [f64; 2],
        ref_points: f64,
    ) {
        let path = asset(&format!("limb_finding/{image}"));
        if !assets_present(&[&path]) {
            return;
        }
        let mut module = LimbFinding::new(LimbFindingConfig {
            name: "limbFind".to_string(),
            filename: Some(path),
            canny_thresh_high: canny_high,
            canny_thresh_low: canny_low,
            blur_size_px: blur,
            ..LimbFindingConfig::default()
        });
        let image_in = Output::new(CameraImageMsg {
            time_tag_nanos: sec2nano(1.0),
            camera_id: 1,
            ..CameraImageMsg::default()
        });
        image_in.connect_to(&mut module.image_in_msg);

        module.init();
        let step = sec2nano(0.5);
        for k in 0..=4 {
            module.update(&context(k * step));
        }
        let output = module.opnav_limb_out_msg.read();

        // The pytest's `points[-1, 1:]` over the first 1000 points: pairs
        // `j < 999` whose x exceeds 1e-2.
        let found: Vec<Vector2<f64>> = output.limb_points_px[..999]
            .iter()
            .copied()
            .filter(|point| point.x > 1e-2)
            .collect();
        eprintln!(
            "{image}: first limb point {:?}, {} points",
            found.first(),
            output.num_limb_points
        );
        assert!(!found.is_empty(), "no limb points in {image}");
        for i in 0..2 {
            assert!(
                ((reference[i] - found[0][i]) / reference[i]).abs() <= 1.0,
                "Limb Test failed processing {image}"
            );
        }
        assert!(output.valid, "Validity test failed processing {image}");
        assert!(
            (output.num_limb_points as f64 - ref_points).abs() <= 10.0,
            "NumPoints test failed processing {image}: {} points",
            output.num_limb_points
        );
        assert_eq!(output.planet_id, 2.0);
        assert_eq!(output.camera_id, 1);
        assert_eq!(output.time_tag_s, sec2nano(1.0) as f64);
    }

    #[test]
    fn mars_bright() {
        limb_finding_test("MarsBright.jpg", 1, 100, 200, [253.0, 110.0], 2.0 * 475.0);
    }

    #[test]
    fn mars_dark() {
        limb_finding_test("MarsDark.jpg", 1, 100, 200, [187.0, 128.0], 2.0 * 192.0);
    }

    #[test]
    fn moons() {
        limb_finding_test("moons.jpg", 3, 200, 300, [213.0, 66.0], 2.0 * 270.0);
    }

    #[test]
    fn no_image_publishes_zeros() {
        let mut module = LimbFinding::new(LimbFindingConfig::default());
        module.init();
        module.update(&context(0));
        assert_eq!(module.opnav_limb_out_msg.read(), OpNavLimbMsg::default());
    }

    /// An encoded image of a lit square, corners `low` and `high`, on a
    /// black sky of `side` pixels: its outline is the only edge.
    fn square_image(side: i32, low: i32, high: i32) -> Vec<u8> {
        let mut image = Mat::new_rows_cols_with_default(
            side,
            side,
            cv_core::CV_8UC3,
            cv_core::Scalar::all(0.0),
        )
        .unwrap();
        imgproc::rectangle_points(
            &mut image,
            Point::new(low, low),
            Point::new(high, high),
            cv_core::Scalar::all(255.0),
            imgproc::FILLED,
            imgproc::LINE_8,
            0,
        )
        .unwrap();
        let mut png = Vector::<u8>::new();
        imgcodecs::imencode_def(".png", &image, &mut png).unwrap();
        png.to_vec()
    }

    #[test]
    fn edge_count_threshold() {
        let image_in = Output::new(CameraImageMsg {
            time_tag_nanos: 4,
            valid: true,
            camera_id: 9,
            image: square_image(200, 50, 149),
            image_type: 3,
        });
        let mut module = LimbFinding::new(LimbFindingConfig::default());
        image_in.connect_to(&mut module.image_in_msg);
        module.init();
        module.update(&context(4));
        let output = module.opnav_limb_out_msg.read();
        assert!(output.valid);
        assert_eq!((output.camera_id, output.time_tag_s), (9, 4.0));
        let points = output.active_limb_points();
        assert!(points.len() > 300 && points.len() < MAX_LIMB_PNTS);
        // Row-major order: the first point is on the top edge.
        assert!((points[0].y - 50.0).abs() <= 2.0);
        assert!(
            points
                .iter()
                .all(|p| (48.0..=151.0).contains(&p.x) && (48.0..=151.0).contains(&p.y))
        );

        // Too few edge pixels: a measured but invalid limb.
        module.config.limb_num_thresh = points.len() as i32;
        module.update(&context(4));
        let output = module.opnav_limb_out_msg.read();
        assert!(!output.valid);
        assert_eq!(output.num_limb_points, 0);
        assert_eq!(output.camera_id, 9);
    }

    #[test]
    fn limb_points_are_capped() {
        // A 600 px square: an outline of about 2400 edge pixels.
        let image_in = Output::new(CameraImageMsg {
            valid: true,
            image: square_image(700, 50, 649),
            ..CameraImageMsg::default()
        });
        let mut module = LimbFinding::new(LimbFindingConfig::default());
        image_in.connect_to(&mut module.image_in_msg);
        module.init();
        module.update(&context(0));
        let output = module.opnav_limb_out_msg.read();
        assert!(output.valid);
        assert_eq!(output.num_limb_points, MAX_LIMB_PNTS);
        assert!(output.limb_points_px[MAX_LIMB_PNTS - 1].x > 0.0);
    }
}
