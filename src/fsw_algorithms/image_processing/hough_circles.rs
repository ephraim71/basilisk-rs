//! Planet circles in a camera image from OpenCV's Hough circle transform:
//! port of upstream Basilisk `houghCircles` (feature `opencv`).
//!
//! The image (see [`acquire_image`]: a configured file, else the input
//! message's encoded image) is converted to grey, thresholded at 15 into a
//! binary inverse image (dark sky white, lit body black), box-blurred, and
//! searched with `cv::HoughCircles` (`HOUGH_GRADIENT`). The first
//! `expected_circles` circles (at most [`MAX_CIRCLE_NUM`]) are published as
//! pixel centres and radii. Each circle's accumulator votes set the diagonal
//! of the `(x, y, rho)` uncertainty to `noise_sf · votes / vote_thresh`; as
//! upstream, every circle overwrites it, so the last one published sets it.
//! A message with at least one circle is valid and names Mars (planet ID 2)
//! as the first body. Without an image the message is zeroed.
//!
//! Header stand-in: upstream time-tags the circles with the input message's
//! write time (`imageInMsg.timeWritten()`); here it is the producer's payload
//! time stamp, [`CameraImageMsg::time_tag_nanos`], read whenever the input is
//! connected and kept otherwise.
//!
//! Upstream writes its debug image (`saveImages`) to `saveDir` itself; here it
//! goes to the time-stamped path `limbFinding` uses, `save_dir` followed by the
//! time in seconds and `.jpg`.

use std::path::PathBuf;

use nalgebra::{Matrix3, Vector2};
use opencv::core::{Mat, Size, Vec4f, Vector};
use opencv::imgproc;

use crate::messages::{CameraImageMsg, Input, MAX_CIRCLE_NUM, OpNavCirclesMsg, Output};
use crate::utilities::opencv_image::{
    ImageError, ImageOrigin, acquire_image, save_debug_image, time_stamped_path,
};
use crate::{Module, SimulationContext};

/// Upstream's planet ID of the first circle: Mars.
const CIRCLE_PLANET_ID: f64 = 2.0;

/// Grey level below which a pixel counts as sky (upstream's hard-coded
/// threshold).
const SKY_THRESHOLD: f64 = 15.0;

/// User parameters of [`HoughCircles`]; the defaults are upstream's
/// constructor values.
#[derive(Clone, Debug)]
pub struct HoughCirclesConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `filename`: read the image from this file instead of the
    /// input message (upstream's empty string is `None`).
    pub filename: Option<PathBuf>,
    /// Upstream `saveDir`: prefix of the debug image path.
    pub save_dir: String,
    /// Upstream `saveImages` (`int`): save each decoded message image.
    pub save_images: bool,
    /// Upstream `blurrSize` \[pixels\]: side of the blurring box.
    pub blur_size_px: i32,
    /// Upstream `cannyThresh` [-]: higher Canny threshold of the transform
    /// (OpenCV `param1`).
    pub canny_thresh: i32,
    /// Upstream `voteThresh` [-]: accumulator votes for a circle to count as
    /// detected (OpenCV `param2`).
    pub vote_thresh: i32,
    /// Upstream `houghMinDist` \[pixels\]: minimum distance between two
    /// circle centres.
    pub hough_min_dist_px: i32,
    /// Upstream `houghMinRadius` \[pixels\]: minimum circle radius.
    pub hough_min_radius_px: i32,
    /// Upstream `houghMaxRadius` \[pixels\]: maximum circle radius; zero uses
    /// the largest image dimension, a negative value finds centres without
    /// radii.
    pub hough_max_radius_px: i32,
    /// Upstream `dpValue` [-]: inverse accumulator resolution; 1 searches the
    /// full image.
    pub dp_value: i32,
    /// Upstream `noiseSF` [-]: scale factor of the uncertainty.
    pub noise_sf: f64,
    /// Upstream `expectedCircles` [-]: most circles to publish.
    pub expected_circles: usize,
}

impl Default for HoughCirclesConfig {
    fn default() -> Self {
        Self {
            name: "houghCircles".to_string(),
            filename: None,
            save_dir: String::new(),
            save_images: false,
            blur_size_px: 5,
            canny_thresh: 200,
            vote_thresh: 20,
            hough_min_dist_px: 50,
            hough_min_radius_px: 0,
            hough_max_radius_px: 0,
            dp_value: 1,
            noise_sf: 4.0,
            expected_circles: MAX_CIRCLE_NUM,
        }
    }
}

/// Hough-transform circle finder: upstream `HoughCircles`.
#[derive(Clone, Debug)]
pub struct HoughCircles {
    /// Module configuration.
    pub config: HoughCirclesConfig,
    /// Upstream `imageInMsg`: the camera image.
    pub image_in_msg: Input<CameraImageMsg>,
    /// Upstream `opnavCirclesOutMsg`: the circles found.
    pub opnav_circles_out_msg: Output<OpNavCirclesMsg>,
    /// Upstream `sensorTimeTag` \[ns\]: time tag of the last image read.
    sensor_time_tag_nanos: u64,
}

impl HoughCircles {
    /// Builds the module with an unconnected input.
    pub fn new(config: HoughCirclesConfig) -> Self {
        Self {
            config,
            image_in_msg: Input::default(),
            opnav_circles_out_msg: Output::default(),
            sensor_time_tag_nanos: 0,
        }
    }

    /// Upstream `Reset`: check that the image input is connected.
    pub fn reset(&mut self) {
        if !self.image_in_msg.is_connected() {
            log::error!("{}.image_in_msg is not connected", self.config.name);
        }
    }

    /// Upstream `UpdateState`: the circles to publish.
    fn find_circles(&mut self, current_sim_nanos: u64) -> Result<OpNavCirclesMsg, ImageError> {
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
            return Ok(OpNavCirclesMsg::default());
        };
        if origin == ImageOrigin::Message && self.config.save_images {
            let path = time_stamped_path(&self.config.save_dir, current_sim_nanos, ".jpg");
            save_debug_image(&self.config.name, &path, &image);
        }

        let mut grey = Mat::default();
        imgproc::cvt_color_def(&image, &mut grey, imgproc::COLOR_BGR2GRAY)?;
        let mut binary = Mat::default();
        imgproc::threshold(
            &grey,
            &mut binary,
            SKY_THRESHOLD,
            255.0,
            imgproc::THRESH_BINARY_INV,
        )?;
        let mut blurred = Mat::default();
        let blur = self.config.blur_size_px;
        imgproc::blur_def(&binary, &mut blurred, Size::new(blur, blur))?;

        // `(x, y, radius, votes)` per circle, strongest first.
        let mut circles = Vector::<Vec4f>::new();
        imgproc::hough_circles(
            &blurred,
            &mut circles,
            imgproc::HOUGH_GRADIENT,
            f64::from(self.config.dp_value),
            f64::from(self.config.hough_min_dist_px),
            f64::from(self.config.canny_thresh),
            f64::from(self.config.vote_thresh),
            self.config.hough_min_radius_px,
            self.config.hough_max_radius_px,
        )?;

        let mut message = OpNavCirclesMsg {
            time_tag_nanos: self.sensor_time_tag_nanos,
            camera_id: image_in.camera_id,
            ..OpNavCirclesMsg::default()
        };
        let published = self.config.expected_circles.min(MAX_CIRCLE_NUM);
        let mut circles_found = 0;
        for (i, circle) in circles.iter().take(published).enumerate() {
            message.circles_centers_px[i] =
                Vector2::new(f64::from(circle[0]), f64::from(circle[1]));
            message.circles_radii_px[i] = f64::from(circle[2]);
            message.uncertainty = Matrix3::from_diagonal_element(
                self.config.noise_sf * f64::from(circle[3]) / f64::from(self.config.vote_thresh),
            );
            circles_found += 1;
        }
        if circles_found > 0 {
            message.valid = true;
            message.planet_ids[0] = CIRCLE_PLANET_ID;
        }
        Ok(message)
    }
}

impl Module for HoughCircles {
    fn init(&mut self) {
        self.sensor_time_tag_nanos = 0;
        self.opnav_circles_out_msg.write(OpNavCirclesMsg::default());
        HoughCircles::reset(self);
    }

    fn reset(&mut self, _context: &SimulationContext) {
        HoughCircles::reset(self);
    }

    fn update(&mut self, context: &SimulationContext) {
        match self.find_circles(context.current_sim_nanos) {
            Ok(circles) => self.opnav_circles_out_msg.write(circles),
            // Upstream's OpenCV call throws here, before writing: the output
            // keeps its previous value.
            Err(error) => log::error!("{}: {error}", self.config.name),
        }
    }
}

#[cfg(test)]
mod tests {
    //! Port of `test_houghCirlces.py`, on its two images (`just
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

    struct Case {
        image: &'static str,
        blur: i32,
        max_circles: usize,
        min_dist: i32,
        min_rad: i32,
        canny_low: i32,
        canny_high: i32,
        dp: i32,
        /// The pytest's reference `(x, y, radius)` circles.
        reference: &'static [(f64, f64, f64)],
    }

    /// The pytest's `houghCirclesTest`: run 2 s at 0.5 s and compare the last
    /// output to the reference circles.
    fn hough_circles_test(case: &Case) {
        let path = asset(&format!("hough_circles/{}", case.image));
        if !assets_present(&[&path]) {
            return;
        }
        let input_image = imgcodecs::imread(&path, imgcodecs::IMREAD_COLOR).unwrap();

        let mut module = HoughCircles::new(HoughCirclesConfig {
            name: "houghCircles".to_string(),
            filename: Some(path.clone()),
            expected_circles: case.max_circles,
            canny_thresh: case.canny_high,
            vote_thresh: case.canny_low,
            hough_min_dist_px: case.min_dist,
            hough_min_radius_px: case.min_rad,
            blur_size_px: case.blur,
            dp_value: case.dp,
            // `int(input_image.size[0]/1.25)`: the image width.
            hough_max_radius_px: (f64::from(input_image.cols()) / 1.25) as i32,
            ..HoughCirclesConfig::default()
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
        let output = module.opnav_circles_out_msg.read();

        // As the pytest, `radii[-1, 1:]` sets the count: the last of the
        // MAX_CIRCLE_NUM slots is never compared.
        let found: Vec<(f64, f64, f64)> = (0..MAX_CIRCLE_NUM - 1)
            .filter(|&j| output.circles_radii_px[j] > 0.0)
            .map(|j| {
                let center = output.circles_centers_px[j];
                (center.x, center.y, output.circles_radii_px[j])
            })
            .collect();
        eprintln!("{}: circles found {found:?}", case.image);
        // Not in the pytest, whose comparison passes vacuously without any.
        assert!(!found.is_empty(), "no circles found in {}", case.image);
        assert!(output.valid);
        assert_eq!(output.planet_ids[0], 2.0);
        assert_eq!(output.camera_id, 1);
        assert_eq!(output.time_tag_nanos, sec2nano(1.0));

        for (test_circle, ref_circle) in found.iter().zip(case.reference) {
            let test = [test_circle.0, test_circle.1, test_circle.2];
            let reference = [ref_circle.0, ref_circle.1, ref_circle.2];
            for i in 0..3 {
                assert!(
                    ((test[i] - reference[i]) / reference[i]).abs() <= 1.0,
                    "Test failed processing {}: {test_circle:?} vs {ref_circle:?}",
                    case.image
                );
            }
        }
    }

    #[test]
    fn mars_crescent() {
        hough_circles_test(&Case {
            image: "mars.png",
            blur: 5,
            max_circles: 1,
            min_dist: 50,
            min_rad: 20,
            canny_low: 20,
            canny_high: 200,
            dp: 1,
            reference: &[(250.0, 260.0, 110.0)],
        });
    }

    #[test]
    fn moon_crescents() {
        hough_circles_test(&Case {
            image: "moons.png",
            blur: 5,
            max_circles: 10,
            min_dist: 25,
            min_rad: 10,
            canny_low: 20,
            canny_high: 200,
            dp: 1,
            reference: &[
                (205.0, 155.0, 48.900001525878906),
                (590.0, 313.0, 46.29999923706055),
                (590.0, 165.0, 46.29999923706055),
                (400.0, 313.0, 43.79999923706055),
                (400.0, 151.5, 45.0),
                (210.0, 313.0, 45.0),
            ],
        });
    }

    #[test]
    fn no_image_publishes_zeros() {
        let mut module = HoughCircles::new(HoughCirclesConfig::default());
        let image_in = Output::new(CameraImageMsg::default());
        image_in.connect_to(&mut module.image_in_msg);
        module.init();
        module.update(&context(0));
        assert_eq!(
            module.opnav_circles_out_msg.read(),
            OpNavCirclesMsg::default()
        );
    }

    #[test]
    fn unreadable_image_is_an_error() {
        let mut module = HoughCircles::new(HoughCirclesConfig {
            filename: Some(PathBuf::from("/nonexistent/path/to/image.png")),
            ..HoughCirclesConfig::default()
        });
        assert!(matches!(module.find_circles(0), Err(ImageError::Load(_))));
    }

    #[test]
    fn message_image_is_used_without_a_file() {
        // A black sky with a lit disc of radius 60 px at (150, 110).
        let mut image = Mat::new_rows_cols_with_default(
            240,
            320,
            opencv::core::CV_8UC3,
            opencv::core::Scalar::all(0.0),
        )
        .unwrap();
        imgproc::circle(
            &mut image,
            opencv::core::Point::new(150, 110),
            60,
            opencv::core::Scalar::all(200.0),
            imgproc::FILLED,
            imgproc::LINE_8,
            0,
        )
        .unwrap();
        let mut png = Vector::<u8>::new();
        imgcodecs::imencode_def(".png", &image, &mut png).unwrap();

        let mut module = HoughCircles::new(HoughCirclesConfig {
            expected_circles: 1,
            hough_min_radius_px: 20,
            hough_max_radius_px: 200,
            ..HoughCirclesConfig::default()
        });
        let image_in = Output::new(CameraImageMsg {
            time_tag_nanos: 7,
            valid: true,
            camera_id: 3,
            image: png.to_vec(),
            image_type: 3,
        });
        image_in.connect_to(&mut module.image_in_msg);
        module.init();
        module.update(&context(7));
        let output = module.opnav_circles_out_msg.read();
        assert!(output.valid);
        assert_eq!((output.camera_id, output.time_tag_nanos), (3, 7));
        let center = output.circles_centers_px[0];
        assert!((center.x - 150.0).abs() < 3.0 && (center.y - 110.0).abs() < 3.0);
        assert!((output.circles_radii_px[0] - 60.0).abs() < 5.0);
        assert!(output.uncertainty[(0, 0)] > 0.0);
        assert_eq!(output.uncertainty[(0, 0)], output.uncertainty[(2, 2)]);
        assert_eq!(output.circles_radii_px[1], 0.0);

        // A stale image (time tag before now) is not processed.
        module.update(&context(8));
        assert_eq!(
            module.opnav_circles_out_msg.read(),
            OpNavCirclesMsg::default()
        );
    }
}
