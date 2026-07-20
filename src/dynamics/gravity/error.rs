use std::error::Error;
use std::fmt;
use std::path::PathBuf;

#[derive(Debug)]
pub enum GravityError {
    EmptyBodyName,
    DuplicateBodyName(String),
    MultipleCentralBodies { existing: String, attempted: String },
    BodyNotFound(String),
    InvalidParameter { parameter: &'static str, value: f64 },
    InvalidDegree { requested: usize, available: usize },
    MissingOrientation(String),
    SingularPosition,
    InvalidGravityFile { path: PathBuf, reason: String },
    InvalidPolyhedron(String),
}

impl fmt::Display for GravityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyBodyName => write!(f, "gravity body name must not be empty"),
            Self::DuplicateBodyName(name) => {
                write!(f, "gravity body '{name}' is already configured")
            }
            Self::MultipleCentralBodies {
                existing,
                attempted,
            } => write!(
                f,
                "gravity body '{attempted}' cannot be central because '{existing}' is already central"
            ),
            Self::BodyNotFound(name) => write!(f, "gravity body '{name}' was not found"),
            Self::InvalidParameter { parameter, value } => {
                write!(
                    f,
                    "gravity parameter '{parameter}' must be finite and positive, got {value}"
                )
            }
            Self::InvalidDegree {
                requested,
                available,
            } => write!(
                f,
                "requested spherical-harmonic degree {requested}, but only degree {available} is available"
            ),
            Self::MissingOrientation(name) => write!(
                f,
                "gravity body '{name}' uses a body-fixed model but has no orientation input or static orientation"
            ),
            Self::SingularPosition => {
                write!(f, "gravity is undefined at the body's center")
            }
            Self::InvalidGravityFile { path, reason } => {
                write!(f, "invalid gravity file '{}': {reason}", path.display())
            }
            Self::InvalidPolyhedron(reason) => write!(f, "invalid polyhedron: {reason}"),
        }
    }
}

impl Error for GravityError {}
