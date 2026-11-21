//! Planet centre and apparent radius in a camera image from a trained
//! convolutional network: port of upstream Basilisk `centerRadiusCNN`
//! (feature `opencv`).
//!
//! The network is an ONNX file read with `cv::dnn::readNetFromONNX` at every
//! reset and evaluated on the CPU. Upstream's network is `CAD.onnx`, kept next
//! to the module's sources (`just fetch-opencv-assets` puts it in
//! `assets/opencv/center_radius_cnn/`).
//!
//! The image (see [`acquire_image`]: a configured file, else the input
//! message's encoded image) is turned into a blob scaled to `[0, 1]`, with red
//! and blue swapped, of size `(rows, cols)` (upstream passes the image's rows
//! as the blob width; the two agree for the square images the network takes).
//! Its output is `(x, y, radius)` in pixels. Unless that output is the
//! network's "nothing found" answer, all three coordinates differing from
//! `(rows / 2, cols / 2, cols / 4)`, the first circle of the message is set,
//! with `pixel_noise` on the uncertainty diagonal, and the message is valid,
//! naming Mars (planet ID 2). Without an image the message is zeroed.
//!
//! As upstream, a valid message always carries camera ID 1 and no time tag:
//! upstream reads the input's write time (`sensorTimeTag`) but never
//! publishes it, so no header stand-in is needed.

use std::fmt;
use std::path::PathBuf;

use nalgebra::{Matrix3, Vector2, Vector3};
use opencv::core::{self as cv_core, Scalar, Size};
use opencv::dnn::{self, Net};
use opencv::prelude::*;

use crate::messages::{CameraImageMsg, Input, OpNavCirclesMsg, Output};
use crate::utilities::opencv_image::{
    ImageError, ImageOrigin, acquire_image, save_debug_image, time_stamped_path,
};
use crate::{Module, SimulationContext};

/// Upstream's planet ID of the circle: Mars.
const CIRCLE_PLANET_ID: f64 = 2.0;

/// Upstream's hard-coded camera ID of a valid message.
const CNN_CAMERA_ID: i64 = 1;

/// User parameters of [`CenterRadiusCnn`]; the defaults are upstream's
/// constructor values.
#[derive(Clone, Debug)]
pub struct CenterRadiusCnnConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `filename`: read the image from this file instead of the
    /// input message (upstream's empty string is `None`).
    pub filename: Option<PathBuf>,
    /// Upstream `pathToNetwork`: the trained network, in ONNX.
    pub path_to_network: PathBuf,
    /// Upstream `saveImages` (`int`): save each decoded message image, to
    /// `PreprocessedImage_<time in s>.jpg` in the working directory.
    pub save_images: bool,
    /// Upstream `pixelNoise` \[pixels\]: standard deviation of `(x, y, rho)`,
    /// published on the uncertainty diagonal.
    pub pixel_noise: Vector3<f64>,
}

impl Default for CenterRadiusCnnConfig {
    fn default() -> Self {
        Self {
            name: "centerRadiusCNN".to_string(),
            filename: None,
            path_to_network: PathBuf::from("./position_net2_trained_11-14.onnx"),
            save_images: false,
            pixel_noise: Vector3::repeat(5.0),
        }
    }
}

/// Why an update publishes nothing. Upstream's OpenCV calls throw in these
/// cases, which aborts the update before the output is written.
#[derive(Debug)]
enum CenterRadiusCnnError {
    /// No image could be read.
    Image(ImageError),
    /// The network was not loaded at reset.
    NoNetwork,
    /// The network's evaluation failed.
    OpenCv(opencv::Error),
}

impl fmt::Display for CenterRadiusCnnError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Image(error) => write!(f, "{error}"),
            Self::NoNetwork => write!(f, "no CNN is loaded"),
            Self::OpenCv(error) => write!(f, "CNN evaluation failed: {error}"),
        }
    }
}

impl From<ImageError> for CenterRadiusCnnError {
    fn from(error: ImageError) -> Self {
        Self::Image(error)
    }
}

impl From<opencv::Error> for CenterRadiusCnnError {
    fn from(error: opencv::Error) -> Self {
        Self::OpenCv(error)
    }
}

/// CNN-based planet centre and radius finder: upstream `CenterRadiusCNN`.
///
/// Not `Clone`: a cloned `cv::dnn::Net` shares its state with the original.
#[derive(Debug)]
pub struct CenterRadiusCnn {
    /// Module configuration.
    pub config: CenterRadiusCnnConfig,
    /// Upstream `imageInMsg`: the camera image.
    pub image_in_msg: Input<CameraImageMsg>,
    /// Upstream `opnavCirclesOutMsg`: the circle found.
    pub opnav_circles_out_msg: Output<OpNavCirclesMsg>,
    /// Upstream `positionNet2`: the network, once loaded.
    network: Option<Net>,
}

impl CenterRadiusCnn {
    /// Builds the module with an unconnected input; the network is read at
    /// reset.
    pub fn new(config: CenterRadiusCnnConfig) -> Self {
        Self {
            config,
            image_in_msg: Input::default(),
            opnav_circles_out_msg: Output::default(),
            network: None,
        }
    }

    /// Upstream `Reset`: check that the image input is connected, and read
    /// the network.
    pub fn reset(&mut self) {
        if !self.image_in_msg.is_connected() {
            log::error!("{}.image_in_msg is not connected", self.config.name);
        }
        self.network = match self.load_network() {
            Ok(network) => Some(network),
            Err(error) => {
                log::error!(
                    "{}: the CNN {} could not be read: {error}",
                    self.config.name,
                    self.config.path_to_network.display()
                );
                None
            }
        };
    }

    /// Whether the network was read at the last reset.
    pub fn network_loaded(&self) -> bool {
        self.network.is_some()
    }

    fn load_network(&self) -> opencv::Result<Net> {
        let mut network = dnn::read_net_from_onnx_def(&self.config.path_to_network)?;
        network.set_preferable_backend(dnn::DNN_BACKEND_DEFAULT)?;
        network.set_preferable_target(dnn::DNN_TARGET_CPU)?;
        Ok(network)
    }

    /// Upstream `UpdateState`: the circle to publish.
    fn find_circle(
        &mut self,
        current_sim_nanos: u64,
    ) -> Result<OpNavCirclesMsg, CenterRadiusCnnError> {
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
            return Ok(OpNavCirclesMsg::default());
        };
        if origin == ImageOrigin::Message && self.config.save_images {
            let path = time_stamped_path("PreprocessedImage_", current_sim_nanos, ".jpg");
            save_debug_image(&self.config.name, &path, &image);
        }
        let network = self
            .network
            .as_mut()
            .ok_or(CenterRadiusCnnError::NoNetwork)?;

        let (rows, cols) = (image.rows(), image.cols());
        let blob = dnn::blob_from_image(
            &image,
            1.0 / 255.0,
            Size::new(rows, cols),
            Scalar::all(0.0),
            true,
            false,
            cv_core::CV_32F,
        )?;
        network.set_input_def(&blob)?;
        let output = network.forward_single_def()?;
        let prediction = output.data_typed::<f32>()?;
        let &[x_pred, y_pred, rad_pred, ..] = prediction else {
            return Err(CenterRadiusCnnError::OpenCv(opencv::Error::new(
                cv_core::StsUnmatchedSizes,
                format!("expected 3 network outputs, got {}", prediction.len()),
            )));
        };

        let mut message = OpNavCirclesMsg::default();
        if x_pred != rows as f32 / 2.0
            && y_pred != cols as f32 / 2.0
            && rad_pred != cols as f32 / 4.0
        {
            message.valid = true;
            message.camera_id = CNN_CAMERA_ID;
            message.planet_ids[0] = CIRCLE_PLANET_ID;
            message.circles_centers_px[0] = Vector2::new(f64::from(x_pred), f64::from(y_pred));
            message.circles_radii_px[0] = f64::from(rad_pred);
            message.uncertainty = Matrix3::from_diagonal(&self.config.pixel_noise);
        }
        Ok(message)
    }
}

impl Module for CenterRadiusCnn {
    fn init(&mut self) {
        self.opnav_circles_out_msg.write(OpNavCirclesMsg::default());
        CenterRadiusCnn::reset(self);
    }

    fn reset(&mut self, _context: &SimulationContext) {
        CenterRadiusCnn::reset(self);
    }

    fn update(&mut self, context: &SimulationContext) {
        match self.find_circle(context.current_sim_nanos) {
            Ok(circle) => self.opnav_circles_out_msg.write(circle),
            // Upstream's OpenCV call throws here, before writing: the output
            // keeps its previous value.
            Err(error) => log::error!("{}: {error}", self.config.name),
        }
    }
}

#[cfg(test)]
mod tests {
    //! Port of `test_centerRadiusCNN.py`, on its three images and upstream's
    //! `CAD.onnx` (`just fetch-opencv-assets`).
    //!
    //! The pytest's reference circles are stale: with its 100 % tolerance it
    //! passes, but `CAD.onnx` finds the disc of `mars.jpg` at about
    //! `(231, 239)` with radius 175, not `(255, 263)` with 121, as the image
    //! itself shows. Each test therefore also checks the circle upstream's
    //! own test drew into `_UnitTest/result_<image>.jpg`, measured with an
    //! algebraic circle fit of its red outline pixels, to within 2 pixels.

    use hifitime::Epoch;

    use super::*;
    use crate::messages::MAX_CIRCLE_NUM;
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

    fn network_path() -> PathBuf {
        asset("center_radius_cnn/CAD.onnx")
    }

    /// The pytest's `cnnTest`: run 2 s at 0.5 s and compare the last output
    /// to the reference circles, then to the circle of upstream's result
    /// image, `drawn`.
    fn cnn_test(image: &str, reference: &[(f64, f64, f64)], drawn: (f64, f64, f64)) {
        let path = asset(&format!("center_radius_cnn/{image}"));
        let network = network_path();
        if !assets_present(&[&path, &network]) {
            return;
        }
        let mut module = CenterRadiusCnn::new(CenterRadiusCnnConfig {
            name: "cnn".to_string(),
            filename: Some(path),
            path_to_network: network,
            pixel_noise: Vector3::new(5.0, 5.0, 5.0),
            ..CenterRadiusCnnConfig::default()
        });
        let image_in = Output::new(CameraImageMsg::default());
        image_in.connect_to(&mut module.image_in_msg);

        module.init();
        assert!(module.network_loaded());
        let step = sec2nano(0.5);
        for k in 0..=4 {
            module.update(&context(k * step));
        }
        let output = module.opnav_circles_out_msg.read();

        let found: Vec<(f64, f64, f64)> = (0..MAX_CIRCLE_NUM)
            .filter(|&j| output.circles_radii_px[j] > 0.0)
            .map(|j| {
                let center = output.circles_centers_px[j];
                (center.x, center.y, output.circles_radii_px[j])
            })
            .collect();
        eprintln!("{image}: circles found {found:?}");
        // Not in the pytest, whose comparison passes vacuously without any.
        assert_eq!(found.len(), 1, "expected one circle in {image}");
        assert!(output.valid);
        assert_eq!(output.camera_id, 1);
        assert_eq!(output.planet_ids[0], 2.0);
        assert_eq!(output.uncertainty, Matrix3::from_diagonal_element(5.0));

        for (test_circle, ref_circle) in found.iter().zip(reference) {
            let test = [test_circle.0, test_circle.1, test_circle.2];
            let reference = [ref_circle.0, ref_circle.1, ref_circle.2];
            for i in 0..3 {
                assert!(
                    ((test[i] - reference[i]) / reference[i]).abs() <= 1.0,
                    "Test failed processing {image}: {test_circle:?} vs {ref_circle:?}"
                );
            }
        }
        let (x, y, radius) = found[0];
        assert!(
            (x - drawn.0).abs() <= 2.0
                && (y - drawn.1).abs() <= 2.0
                && (radius - drawn.2).abs() <= 2.0,
            "{image}: {:?} is not upstream's result {drawn:?}",
            found[0]
        );
    }

    #[test]
    fn mars() {
        cnn_test(
            "mars.jpg",
            &[(254.81655883789062, 263.2418518066406, 120.9570541381836)],
            (230.86, 238.84, 175.23),
        );
    }

    #[test]
    fn mars2() {
        cnn_test(
            "mars2.jpg",
            &[(269.21127319, 231.63162231, 144.85394287)],
            (230.97, 241.11, 126.00),
        );
    }

    #[test]
    fn mars3() {
        // The pytest has no reference circle for this image.
        cnn_test("mars3.jpg", &[], (232.32, 296.24, 128.17));
    }

    #[test]
    fn missing_network_publishes_nothing() {
        let path = asset("center_radius_cnn/mars.jpg");
        if !assets_present(&[&path]) {
            return;
        }
        let mut module = CenterRadiusCnn::new(CenterRadiusCnnConfig {
            filename: Some(path),
            path_to_network: PathBuf::from("/nonexistent/network.onnx"),
            ..CenterRadiusCnnConfig::default()
        });
        module.init();
        assert!(!module.network_loaded());
        assert!(matches!(
            module.find_circle(0),
            Err(CenterRadiusCnnError::NoNetwork)
        ));
    }

    #[test]
    fn no_image_publishes_zeros() {
        let mut module = CenterRadiusCnn::new(CenterRadiusCnnConfig {
            path_to_network: PathBuf::from("/nonexistent/network.onnx"),
            ..CenterRadiusCnnConfig::default()
        });
        module.init();
        module.update(&context(0));
        assert_eq!(
            module.opnav_circles_out_msg.read(),
            OpNavCirclesMsg::default()
        );
    }
}
