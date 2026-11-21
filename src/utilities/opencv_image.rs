//! Image intake shared by the OpenCV modules (feature `opencv`): upstream's
//! `houghCircles`, `limbFinding`, `centerRadiusCNN` and `camera` all start
//! their update the same way.
//!
//! A configured file wins (upstream `filename`, "added for debugging
//! purposes", read with `cv::imread`). Otherwise the image of the input
//! message is decoded with `cv::imdecode`, if the message is valid and its time
//! tag is not older than the current simulation time. Either way the image is
//! 8-bit, three-channel BGR (`cv::IMREAD_COLOR`). Without either, there is no
//! image and the module publishes a zeroed message.

use std::fmt;
use std::path::{Path, PathBuf};

use opencv::core::Mat;
use opencv::imgcodecs;
use opencv::prelude::*;

use crate::messages::CameraImageMsg;

/// Where [`acquire_image`] found this update's image.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImageOrigin {
    /// Read from the configured file.
    File,
    /// Decoded from the input message.
    Message,
}

/// Why no image could be read. Upstream either raises `BasiliskError`
/// (`camera`) or lets the following OpenCV call throw on the empty matrix (the
/// FSW modules); both abort the update before the output is written.
#[derive(Debug)]
pub enum ImageError {
    /// `cv::imread` found no readable image at this path.
    Load(PathBuf),
    /// `cv::imdecode` could not decode the message's bytes.
    Decode,
    /// An OpenCV call failed.
    OpenCv(opencv::Error),
}

impl fmt::Display for ImageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Load(path) => write!(f, "failed to load image from {}", path.display()),
            Self::Decode => write!(f, "failed to decode image from input buffer"),
            Self::OpenCv(error) => write!(f, "OpenCV: {error}"),
        }
    }
}

impl std::error::Error for ImageError {}

impl From<opencv::Error> for ImageError {
    fn from(error: opencv::Error) -> Self {
        Self::OpenCv(error)
    }
}

/// This update's image, or `None` when there is none to process.
///
/// `image_in` is the input message as read this update (the default message
/// when the input is unconnected).
pub fn acquire_image(
    filename: Option<&Path>,
    image_in: &CameraImageMsg,
    current_sim_nanos: u64,
) -> Result<Option<(ImageOrigin, Mat)>, ImageError> {
    if let Some(path) = filename {
        let image = imgcodecs::imread(path, imgcodecs::IMREAD_COLOR)?;
        if image.empty() {
            return Err(ImageError::Load(path.to_path_buf()));
        }
        return Ok(Some((ImageOrigin::File, image)));
    }
    if image_in.valid && image_in.time_tag_nanos >= current_sim_nanos {
        // `cv::imdecode` asserts on an empty buffer; it is a decode failure.
        if image_in.image.is_empty() {
            return Err(ImageError::Decode);
        }
        let image = imgcodecs::imdecode(&image_in.image.as_slice(), imgcodecs::IMREAD_COLOR)?;
        if image.empty() {
            return Err(ImageError::Decode);
        }
        return Ok(Some((ImageOrigin::Message, image)));
    }
    Ok(None)
}

/// Upstream's debug-image path: `prefix` followed by the simulation time in
/// seconds as C++ `std::to_string(double)` prints it (six decimals) and
/// `extension`, e.g. `out/run1` + `0.500000` + `.png`. The prefix is joined by
/// plain concatenation, as upstream.
pub fn time_stamped_path(prefix: &str, current_sim_nanos: u64, extension: &str) -> PathBuf {
    PathBuf::from(format!(
        "{prefix}{:.6}{extension}",
        current_sim_nanos as f64 * 1.0e-9
    ))
}

/// Writes `image` to `path`, logging (upstream's `BSK_WARNING`) rather than
/// failing when it cannot.
pub fn save_debug_image(module: &str, path: &Path, image: &Mat) {
    let written = imgcodecs::imwrite_def(path, image);
    if !matches!(written, Ok(true)) {
        log::warn!("{module}: wasn't able to save image to {}", path.display());
    }
}

#[cfg(test)]
pub(crate) mod test_support {
    //! Test assets fetched by `just fetch-opencv-assets`.

    use std::path::{Path, PathBuf};

    /// `assets/opencv/<relative>` in the repository.
    pub(crate) fn asset(relative: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("assets/opencv")
            .join(relative)
    }

    /// Whether every asset is present; prints why the test is skipped if not.
    pub(crate) fn assets_present(paths: &[&Path]) -> bool {
        match paths.iter().find(|path| !path.is_file()) {
            Some(missing) => {
                eprintln!(
                    "skipping: {} is missing; run `just fetch-opencv-assets` to fetch it",
                    missing.display()
                );
                false
            }
            None => true,
        }
    }

    /// A fresh, empty directory for one test's output files.
    pub(crate) fn scratch_dir(test: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("basilisk-rs-{test}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("create a scratch directory");
        dir
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_stamped_path_prints_seconds_like_std_to_string() {
        assert_eq!(
            time_stamped_path("dir/00000", 0, ".png"),
            PathBuf::from("dir/000000.000000.png")
        );
        assert_eq!(
            time_stamped_path("", 500_000_000, ".jpg"),
            PathBuf::from("0.500000.jpg")
        );
    }

    #[test]
    fn no_file_and_no_valid_message_means_no_image() {
        let message = CameraImageMsg::default();
        assert!(acquire_image(None, &message, 0).unwrap().is_none());

        // A valid image older than the current time is stale.
        let stale = CameraImageMsg {
            valid: true,
            time_tag_nanos: 1,
            image: vec![1, 2, 3],
            ..CameraImageMsg::default()
        };
        assert!(acquire_image(None, &stale, 2).unwrap().is_none());
    }

    #[test]
    fn bad_file_and_bad_bytes_are_errors() {
        let missing = Path::new("/nonexistent/path/to/image.png");
        assert!(matches!(
            acquire_image(Some(missing), &CameraImageMsg::default(), 0),
            Err(ImageError::Load(_))
        ));
        for image in [Vec::new(), vec![0_u8; 16]] {
            let message = CameraImageMsg {
                valid: true,
                image,
                ..CameraImageMsg::default()
            };
            assert!(matches!(
                acquire_image(None, &message, 0),
                Err(ImageError::Decode)
            ));
        }
    }

    #[test]
    fn message_image_round_trips_through_png() {
        let image = Mat::new_rows_cols_with_default(
            4,
            6,
            opencv::core::CV_8UC3,
            opencv::core::Scalar::new(10.0, 20.0, 30.0, 0.0),
        )
        .unwrap();
        let mut png = opencv::core::Vector::<u8>::new();
        assert!(imgcodecs::imencode_def(".png", &image, &mut png).unwrap());
        let message = CameraImageMsg {
            valid: true,
            time_tag_nanos: 5,
            image: png.to_vec(),
            ..CameraImageMsg::default()
        };
        let (origin, decoded) = acquire_image(None, &message, 5).unwrap().unwrap();
        assert_eq!(origin, ImageOrigin::Message);
        assert_eq!((decoded.rows(), decoded.cols()), (4, 6));
        assert_eq!(
            *decoded.at_2d::<opencv::core::Vec3b>(3, 5).unwrap(),
            opencv::core::Vec3b::from([10, 20, 30])
        );
    }
}
