//! Attitude reference from a file of time-tagged waypoints: port of upstream
//! Basilisk `waypointReference`.
//!
//! Each data line holds a time \[s\], an attitude, the angular rate and the
//! angular acceleration, separated by a one-character delimiter. The
//! attitude is an MRP set (mapped to the short rotation when `|σ| > 1`) or a
//! quaternion with the scalar first or last (normalized, then converted to
//! MRPs). The rates are in inertial components, or in reference-frame
//! components when `use_reference_frame` is set, and are then rotated to
//! inertial.
//!
//! Between two waypoints the reference attitude, rate and acceleration are
//! interpolated linearly in time; when the two MRP sets are more than 1
//! apart the second is replaced by its shadow set first. Before the first
//! waypoint the module holds the first attitude, and after the last the last
//! attitude, both with zero rate and acceleration.
//!
//! The file is read by [`load_waypoints`], a thin layer over
//! [`parse_waypoints`]; the module loads it at reset. Unlike upstream, which
//! streams the file one line ahead of the simulation, the whole file is read
//! and checked at reset, so a malformed line is reported then, and a reset
//! starts the file over (upstream keeps its end-of-file flag). Blank lines
//! are skipped; upstream fails on them. When the file cannot be loaded the
//! error is logged and, as upstream without an open file, nothing is
//! published.

use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};

use nalgebra::{Vector3, Vector4};

use crate::messages::{AttitudeReferenceMsg, Output};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::{ep_to_mrp, mrp_shadow, mrp_to_dcm};
use crate::{Module, SimulationContext};

/// Upstream `SEC2NANO`.
const SEC2NANO: f64 = 1.0e9;

/// The attitude coordinates of a waypoint line (upstream `attitudeType`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum WaypointAttitudeType {
    /// Upstream 0: MRPs `σ1, σ2, σ3`.
    #[default]
    Mrp = 0,
    /// Upstream 1: Euler parameters with the scalar first, `q0, q1, q2, q3`.
    QuaternionScalarFirst = 1,
    /// Upstream 2: Euler parameters with the scalar last, `q1, q2, q3, qs`.
    QuaternionScalarLast = 2,
}

/// How the waypoint file is laid out.
#[derive(Clone, Debug, PartialEq)]
pub struct WaypointFormat {
    /// Upstream `delimiter`: the character between values (upstream uses the
    /// first character of a string). Default `,`.
    pub delimiter: char,
    /// Upstream `headerLines`: lines skipped at the top of the file. Default
    /// 0.
    pub header_lines: usize,
    /// Upstream `attitudeType`. Default MRPs.
    pub attitude_type: WaypointAttitudeType,
    /// Upstream `useReferenceFrame`: the rates and accelerations are in
    /// reference-frame instead of inertial components. Default false.
    pub use_reference_frame: bool,
}

impl Default for WaypointFormat {
    fn default() -> Self {
        Self {
            delimiter: ',',
            header_lines: 0,
            attitude_type: WaypointAttitudeType::Mrp,
            use_reference_frame: false,
        }
    }
}

/// One time-tagged attitude reference.
#[derive(Clone, Debug, Default)]
pub struct Waypoint {
    /// Time of the waypoint \[ns\], truncated from the file's seconds as
    /// upstream.
    pub time_nanos: u64,
    /// The attitude (short-rotation MRPs), rate and acceleration, inertial
    /// components.
    pub reference: AttitudeReferenceMsg,
}

/// Why a waypoint file could not be loaded.
#[derive(Clone, Debug, PartialEq)]
pub enum WaypointFileError {
    /// The file could not be read.
    Io {
        /// The file.
        path: PathBuf,
        /// The reason.
        message: String,
    },
    /// A data line is malformed.
    Parse {
        /// One-based line number in the file.
        line: usize,
        /// The reason.
        message: String,
    },
}

impl fmt::Display for WaypointFileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { path, message } => {
                write!(f, "cannot read waypoint file {}: {message}", path.display())
            }
            Self::Parse { line, message } => write!(f, "waypoint file line {line}: {message}"),
        }
    }
}

impl std::error::Error for WaypointFileError {}

/// Reads and parses the waypoint file at `path` (upstream `Reset`'s file
/// handling).
pub fn load_waypoints(
    path: impl AsRef<Path>,
    format: &WaypointFormat,
) -> Result<Vec<Waypoint>, WaypointFileError> {
    let path = path.as_ref();
    let text = fs::read_to_string(path).map_err(|error| WaypointFileError::Io {
        path: path.to_path_buf(),
        message: error.to_string(),
    })?;
    parse_waypoints(&text, format)
}

/// Parses waypoint lines (upstream `pullDataLine`, for every line after the
/// header). Values beyond the ten (or eleven) a line needs are ignored, as
/// upstream.
pub fn parse_waypoints(
    text: &str,
    format: &WaypointFormat,
) -> Result<Vec<Waypoint>, WaypointFileError> {
    text.lines()
        .enumerate()
        .skip(format.header_lines)
        .filter(|(_, line)| !line.trim().is_empty())
        .map(|(index, line)| {
            parse_line(line, format).map_err(|message| WaypointFileError::Parse {
                line: index + 1,
                message,
            })
        })
        .collect()
}

/// The values of one data line, read in order.
struct Fields<'a> {
    items: std::str::Split<'a, char>,
}

impl Fields<'_> {
    /// Upstream `pullScalar`. Like upstream's `stod`, leading white space is
    /// skipped; trailing white space (a `\r`) is too.
    fn scalar(&mut self) -> Result<f64, String> {
        let item = self.items.next().ok_or("too few values")?.trim();
        item.parse::<f64>()
            .map_err(|error| format!("cannot read {item:?} as a number: {error}"))
    }

    /// Upstream `pullVector`.
    fn vector3(&mut self) -> Result<Vector3<f64>, String> {
        Ok(Vector3::new(self.scalar()?, self.scalar()?, self.scalar()?))
    }

    /// Upstream `pullVector4`.
    fn vector4(&mut self) -> Result<Vector4<f64>, String> {
        Ok(Vector4::new(
            self.scalar()?,
            self.scalar()?,
            self.scalar()?,
            self.scalar()?,
        ))
    }
}

/// One waypoint from one data line.
fn parse_line(line: &str, format: &WaypointFormat) -> Result<Waypoint, String> {
    let mut fields = Fields {
        items: line.split(format.delimiter),
    };
    let time_s = fields.scalar()?;
    let sigma_rn = match format.attitude_type {
        WaypointAttitudeType::Mrp => {
            let sigma = fields.vector3()?;
            if sigma.norm() <= 1.0 {
                sigma
            } else {
                mrp_shadow(sigma)
            }
        }
        WaypointAttitudeType::QuaternionScalarFirst => {
            ep_to_mrp(normalize_or_zero(fields.vector4()?))
        }
        WaypointAttitudeType::QuaternionScalarLast => {
            let q = fields.vector4()?;
            ep_to_mrp(normalize_or_zero(Vector4::new(q[3], q[0], q[1], q[2])))
        }
    };
    let omega = fields.vector3()?;
    let omega_dot = fields.vector3()?;
    let (omega_rn_n, domega_rn_n) = if format.use_reference_frame {
        // `v3tMultM33(v, [RN])` is `[RN]ᵀ v`.
        let dcm_nr = mrp_to_dcm(sigma_rn).transpose();
        (dcm_nr * omega, dcm_nr * omega_dot)
    } else {
        (omega, omega_dot)
    };

    Ok(Waypoint {
        time_nanos: (time_s * SEC2NANO) as u64,
        reference: AttitudeReferenceMsg {
            sigma_rn,
            omega_rn_n_radps: omega_rn_n,
            domega_rn_n_radps2: domega_rn_n,
        },
    })
}

/// User-settable parameters of [`WaypointReference`].
#[derive(Clone, Debug)]
pub struct WaypointReferenceConfig {
    /// Module instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `dataFileName`: the waypoint file, read at reset. Default
    /// empty, which reset reports as an error.
    pub data_file_name: PathBuf,
    /// Upstream `delimiter`, `headerLines`, `attitudeType` and
    /// `useReferenceFrame`.
    pub format: WaypointFormat,
}

impl Default for WaypointReferenceConfig {
    fn default() -> Self {
        Self {
            name: "waypointReference".to_string(),
            data_file_name: PathBuf::new(),
            format: WaypointFormat::default(),
        }
    }
}

/// Attitude reference interpolated between file waypoints: upstream
/// `waypointReference`.
#[derive(Clone, Debug)]
pub struct WaypointReference {
    /// User-settable parameters, read at reset.
    pub config: WaypointReferenceConfig,
    /// Upstream `attRefOutMsg`: the reference attitude, rate and
    /// acceleration.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
    /// The loaded waypoints, in file order.
    waypoints: Vec<Waypoint>,
    /// Upstream's read position: the index of the next waypoint to pull.
    cursor: usize,
    /// Upstream `t_a` and `attRefMsg_a`: the waypoint at or before the
    /// current time.
    a: Waypoint,
    /// Upstream `t_b` and `attRefMsg_b`: the waypoint after it.
    b: Waypoint,
    /// Upstream `endOfFile`.
    end_of_file: bool,
}

impl WaypointReference {
    /// Creates the module with `config`; the file is loaded at reset.
    pub fn new(config: WaypointReferenceConfig) -> Self {
        Self {
            config,
            att_ref_out_msg: Output::default(),
            waypoints: Vec::new(),
            cursor: 0,
            a: Waypoint::default(),
            b: Waypoint::default(),
            end_of_file: false,
        }
    }

    /// Upstream `Reset`: loads the file and pulls the first two waypoints.
    pub fn reset(&mut self) {
        let loaded = if self.config.data_file_name.as_os_str().is_empty() {
            Err("dataFileName must be a non-empty path".to_string())
        } else {
            load_waypoints(&self.config.data_file_name, &self.config.format)
                .map_err(|error| error.to_string())
        };
        let waypoints = loaded.unwrap_or_else(|error| {
            log::error!("{}: {error}", self.config.name);
            Vec::new()
        });
        self.start(waypoints);
    }

    /// Restarts the interpolation on `waypoints`. A single waypoint is held
    /// throughout (upstream would then read an uninitialized second one).
    fn start(&mut self, waypoints: Vec<Waypoint>) {
        self.waypoints = waypoints;
        self.cursor = 0;
        self.end_of_file = false;
        self.a = self.pull().unwrap_or_default();
        self.b = self.pull().unwrap_or_else(|| self.a.clone());
    }

    /// Upstream `pullDataLine`: the next waypoint, or `None` (setting the
    /// end-of-file flag) when none is left.
    fn pull(&mut self) -> Option<Waypoint> {
        let waypoint = self.waypoints.get(self.cursor).cloned();
        match waypoint {
            Some(_) => self.cursor += 1,
            None => self.end_of_file = true,
        }
        waypoint
    }

    /// Upstream `UpdateState`'s reference at `t_nanos`.
    fn reference_at(&mut self, t_nanos: u64) -> AttitudeReferenceMsg {
        let mut reference = AttitudeReferenceMsg::default();
        if t_nanos < self.a.time_nanos {
            // Before the first waypoint: hold its attitude at rest.
            reference.sigma_rn = self.a.reference.sigma_rn;
        } else {
            while t_nanos > self.b.time_nanos && !self.end_of_file {
                if let Some(next) = self.pull() {
                    self.a = std::mem::replace(&mut self.b, next);
                } else {
                    self.a = self.b.clone();
                }
            }
            if t_nanos >= self.a.time_nanos && t_nanos <= self.b.time_nanos {
                let (a, b) = (&self.a.reference, &self.b.reference);
                // A zero-length interval (one waypoint, or repeated time
                // tags) takes the first end; upstream divides by zero.
                let span_s = diff_nanos_to_seconds(self.b.time_nanos, self.a.time_nanos);
                let fraction = if span_s > 0.0 {
                    diff_nanos_to_seconds(t_nanos, self.a.time_nanos) / span_s
                } else {
                    0.0
                };
                let lerp = |v_a: Vector3<f64>, v_b: Vector3<f64>| v_a + (v_b - v_a) * fraction;
                // Interpolate towards the closer of b's two MRP sets.
                let sigma_b = if (b.sigma_rn - a.sigma_rn).norm() <= 1.0 {
                    b.sigma_rn
                } else {
                    mrp_shadow(b.sigma_rn)
                };
                reference.sigma_rn = lerp(a.sigma_rn, sigma_b);
                reference.omega_rn_n_radps = lerp(a.omega_rn_n_radps, b.omega_rn_n_radps);
                reference.domega_rn_n_radps2 = lerp(a.domega_rn_n_radps2, b.domega_rn_n_radps2);
            }
        }
        if t_nanos > self.b.time_nanos && self.end_of_file {
            // After the last waypoint: hold its attitude at rest.
            reference = AttitudeReferenceMsg {
                sigma_rn: self.b.reference.sigma_rn,
                ..Default::default()
            };
        }
        reference
    }
}

impl Module for WaypointReference {
    fn init(&mut self) {
        WaypointReference::reset(self);
    }

    fn reset(&mut self, _context: &SimulationContext) {
        WaypointReference::reset(self);
    }

    fn update(&mut self, context: &SimulationContext) {
        // Upstream publishes only while its file is open.
        if self.waypoints.is_empty() {
            return;
        }
        let reference = self.reference_at(context.current_sim_nanos);
        self.att_ref_out_msg.write(reference);
    }
}

#[cfg(test)]
mod tests {
    //! Ports `waypointReference/_UnitTest/test_WaypointReference.py`, all
    //! twelve parametrizations. The pytest writes its data file with
    //! Python's `str` of the values; Rust's `{:?}` of an `f64` is the same
    //! shortest round-trip text.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{
        WaypointAttitudeType, WaypointFileError, WaypointFormat, WaypointReference,
        WaypointReferenceConfig, load_waypoints, parse_waypoints,
    };
    use crate::messages::AttitudeReferenceMsg;
    use crate::utilities::rigid_body_kinematics::{mrp_to_dcm, mrp_to_ep};
    use crate::{Module, SimulationContext};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Python's `str(list)[1:-1]`: `repr` of each value, joined by `", "`.
    fn python_list(values: &[f64]) -> String {
        values
            .iter()
            .map(|value| format!("{value:?}"))
            .collect::<Vec<_>>()
            .join(", ")
    }

    /// The pytest's truth: the time tags, attitudes (short rotation) and
    /// inertial rates and accelerations, and the data file it writes.
    struct Truth {
        t: Vec<f64>,
        att_rn: Vec<Vector3<f64>>,
        omega_rn_n: Vec<Vector3<f64>>,
        omega_dot_rn_n: Vec<Vector3<f64>>,
        file: String,
    }

    fn truth(
        att_type: WaypointAttitudeType,
        mrp_switching: bool,
        use_reference_frame: bool,
    ) -> Truth {
        let delimiter = ",";
        let mut truth = Truth {
            t: Vec::new(),
            att_rn: Vec::new(),
            omega_rn_n: Vec::new(),
            omega_dot_rn_n: Vec::new(),
            file: String::new(),
        };
        for i in 0..5 {
            let step = 0.05 * i as f64;
            truth.t.push((i + 1) as f64);
            let s = if mrp_switching { 0.8 } else { 0.1 };
            let att_real_mrp = Vector3::new(s + step, 0.2 + step, 0.3 + step);
            let att = if att_real_mrp.norm() <= 1.0 {
                att_real_mrp
            } else {
                -att_real_mrp / att_real_mrp.norm().powi(2)
            };
            truth.att_rn.push(att);
            let omega = Vector3::new(0.4 + step, 0.5 + step, 0.6 + step);
            let omega_dot = Vector3::new(0.7 + step, 0.8 + step, 0.9 + step);
            truth.omega_rn_n.push(omega);
            truth.omega_dot_rn_n.push(omega_dot);

            // `str(t[-1])` of an int.
            let mut line = format!("{}{delimiter}", i + 1);
            let q = mrp_to_ep(att);
            line += &match att_type {
                WaypointAttitudeType::Mrp => python_list(att.as_slice()),
                WaypointAttitudeType::QuaternionScalarFirst => python_list(q.as_slice()),
                WaypointAttitudeType::QuaternionScalarLast => {
                    python_list(&[q[1], q[2], q[3], q[0]])
                }
            };
            line += delimiter;
            let (omega_file, omega_dot_file) = if use_reference_frame {
                let rn = mrp_to_dcm(att);
                (rn * omega, rn * omega_dot)
            } else {
                (omega, omega_dot)
            };
            line += &format!(
                "{}{delimiter}{}\n",
                python_list(omega_file.as_slice()),
                python_list(omega_dot_file.as_slice())
            );
            truth.file += &line;
        }
        truth
    }

    fn assert_vector_close(
        actual: Vector3<f64>,
        expected: Vector3<f64>,
        accuracy: f64,
        what: &str,
    ) {
        assert!(
            (actual - expected).iter().all(|d| d.abs() <= accuracy),
            "{what}: {actual} != {expected}"
        );
    }

    /// Upstream `test_waypointReference` for every `attType`,
    /// `MRPswitching` and `useReferenceFrame` (`accuracy = 1e-12`): a 6 s
    /// run at 0.25 s through waypoints at 1 … 5 s. As upstream, the last
    /// logged sample is not checked.
    #[test]
    fn matches_upstream_waypoint_reference_unit_test() {
        let accuracy = 1e-12;
        let directory =
            std::env::temp_dir().join(format!("waypoint_reference_{}", std::process::id()));
        std::fs::create_dir_all(&directory).unwrap();
        for att_type in [
            WaypointAttitudeType::Mrp,
            WaypointAttitudeType::QuaternionScalarFirst,
            WaypointAttitudeType::QuaternionScalarLast,
        ] {
            for mrp_switching in [true, false] {
                for use_reference_frame in [true, false] {
                    let truth = truth(att_type, mrp_switching, use_reference_frame);
                    let name = format!(
                        "data{}{}{}.txt",
                        att_type as i32,
                        i32::from(mrp_switching),
                        if use_reference_frame { "R" } else { "N" }
                    );
                    let path = directory.join(name);
                    std::fs::write(&path, &truth.file).unwrap();

                    let mut module = WaypointReference::new(WaypointReferenceConfig {
                        name: "testModule".to_string(),
                        data_file_name: path.clone(),
                        format: WaypointFormat {
                            delimiter: ',',
                            header_lines: 0,
                            attitude_type: att_type,
                            use_reference_frame,
                        },
                    });
                    module.init();
                    let mut log = Vec::new();
                    for tick in 0..=24 {
                        let now = tick * 250_000_000;
                        module.update(&context(now));
                        log.push((now as f64 * 1e-9, module.att_ref_out_msg.read()));
                    }
                    std::fs::remove_file(&path).unwrap();

                    let case = format!("{att_type:?} {mrp_switching} {use_reference_frame}");
                    check_log(&truth, &log[..log.len() - 1], accuracy, &case);
                }
            }
        }
        let _ = std::fs::remove_dir(&directory);
    }

    /// The pytest's checks of each logged sample.
    fn check_log(truth: &Truth, log: &[(f64, AttitudeReferenceMsg)], accuracy: f64, case: &str) {
        let t = &truth.t;
        let mut j = 0;
        for (time, sample) in log {
            let time = *time;
            let what = format!("{case} at t = {time}");
            if time < t[0] {
                assert_vector_close(sample.sigma_rn, truth.att_rn[0], accuracy, &what);
                assert_vector_close(sample.omega_rn_n_radps, Vector3::zeros(), accuracy, &what);
                assert_vector_close(sample.domega_rn_n_radps2, Vector3::zeros(), accuracy, &what);
            } else if time <= t[4] {
                while !(time >= t[j] && time <= t[j + 1]) {
                    j += 1;
                }
                let fraction = (time - t[j]) / (t[j + 1] - t[j]);
                let (att_a, att_b) = (truth.att_rn[j], truth.att_rn[j + 1]);
                let sigma = if (att_b - att_a).norm() <= 1.0 {
                    att_a + (att_b - att_a) * fraction
                } else {
                    let shadow = -att_b / att_b.norm().powi(2);
                    att_a + (shadow - att_a) * fraction
                };
                let lerp = |v: &[Vector3<f64>]| v[j] + (v[j + 1] - v[j]) * fraction;
                assert_vector_close(sample.sigma_rn, sigma, accuracy, &what);
                assert_vector_close(
                    sample.omega_rn_n_radps,
                    lerp(&truth.omega_rn_n),
                    accuracy,
                    &what,
                );
                assert_vector_close(
                    sample.domega_rn_n_radps2,
                    lerp(&truth.omega_dot_rn_n),
                    accuracy,
                    &what,
                );
            } else {
                assert_vector_close(sample.sigma_rn, truth.att_rn[4], accuracy, &what);
                assert_vector_close(sample.omega_rn_n_radps, Vector3::zeros(), accuracy, &what);
                assert_vector_close(sample.domega_rn_n_radps2, Vector3::zeros(), accuracy, &what);
            }
        }
    }

    /// Not upstream: header lines and blank lines are skipped, another
    /// delimiter works, and a malformed line or a missing file is an error
    /// naming it.
    #[test]
    fn parser_handles_headers_delimiters_and_errors() {
        let format = WaypointFormat {
            delimiter: ';',
            header_lines: 2,
            ..Default::default()
        };
        let text = "time;sigma\nunits\n1.5; 0.1;0.2;0.3; 1;2;3; 4;5;6\r\n\n2;0.9;0.5;0.2;0;0;0;0;0;0;extra\n";
        let waypoints = parse_waypoints(text, &format).unwrap();
        assert_eq!(waypoints.len(), 2);
        assert_eq!(waypoints[0].time_nanos, 1_500_000_000);
        assert_eq!(waypoints[0].reference.sigma_rn, Vector3::new(0.1, 0.2, 0.3));
        assert_eq!(
            waypoints[0].reference.domega_rn_n_radps2,
            Vector3::new(4.0, 5.0, 6.0)
        );
        // |σ| > 1 is stored as the shadow set.
        let sigma = Vector3::new(0.9, 0.5, 0.2);
        assert_eq!(
            waypoints[1].reference.sigma_rn,
            -sigma / sigma.norm_squared()
        );

        let error = parse_waypoints("h\nh\n1;2;x", &format).unwrap_err();
        assert!(
            matches!(error, WaypointFileError::Parse { line: 3, .. }),
            "{error}"
        );
        let error = parse_waypoints(
            "1;0;0;0;0;0",
            &WaypointFormat {
                delimiter: ';',
                ..Default::default()
            },
        )
        .unwrap_err();
        assert!(error.to_string().contains("too few values"), "{error}");
        assert!(matches!(
            load_waypoints("/nonexistent/waypoints.txt", &format),
            Err(WaypointFileError::Io { .. })
        ));
    }

    /// Not upstream: without a loadable file nothing is published; a single
    /// waypoint is held at rest; a reset restarts the file.
    #[test]
    fn missing_file_single_waypoint_and_reset() {
        let mut module = WaypointReference::new(WaypointReferenceConfig {
            data_file_name: "/nonexistent/waypoints.txt".into(),
            ..Default::default()
        });
        module.init();
        module.update(&context(0));
        assert_eq!(module.att_ref_out_msg.read().sigma_rn, Vector3::zeros());
        assert!(module.waypoints.is_empty());

        let format = WaypointFormat::default();
        module.start(parse_waypoints("1,0.1,0.2,0.3,1,1,1,1,1,1", &format).unwrap());
        for now in [0, 1_000_000_000, 5_000_000_000] {
            module.update(&context(now));
            let output = module.att_ref_out_msg.read();
            assert_eq!(output.sigma_rn, Vector3::new(0.1, 0.2, 0.3));
            if now != 1_000_000_000 {
                assert_eq!(output.omega_rn_n_radps, Vector3::zeros());
            }
        }

        let two = "1,0.1,0,0,0,0,0,0,0,0\n2,0.2,0,0,0,0,0,0,0,0";
        module.start(parse_waypoints(two, &format).unwrap());
        module.update(&context(3_000_000_000));
        assert_eq!(
            module.att_ref_out_msg.read().sigma_rn,
            Vector3::new(0.2, 0.0, 0.0)
        );
        module.start(parse_waypoints(two, &format).unwrap());
        module.update(&context(1_500_000_000));
        assert!(
            (module.att_ref_out_msg.read().sigma_rn - Vector3::new(0.15, 0.0, 0.0)).norm() < 1e-15
        );
    }
}
