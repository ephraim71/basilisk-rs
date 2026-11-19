//! Planet, spacecraft and time states from the NAIF CSPICE toolkit: port of
//! upstream Basilisk `simulation/environment/spiceInterface`. Built with the
//! `spice` cargo feature.
//!
//! Each update the module advances the ephemeris time (TDB) from its initial
//! epoch by the simulation time and publishes
//!
//! * the time in several scales, one [`SpiceTimeMsg`]: TDB seconds past
//!   J2000, the UTC Julian date (held at the following midnight during an
//!   inserted leap second, as SPICE's JDUTC), and the GPS week, seconds of
//!   week and 1024-week rollovers (GPS time is TAI - 19 s since the GPS
//!   epoch, held at zero before it);
//! * for each planet in [`SpiceInterfaceConfig::planet_names`], a
//!   [`PlanetStateMsg`]: `spkezr_c` position and velocity relative to
//!   `zero_base` in `reference_base` without aberration correction, and,
//!   when the body has a frame, the `sxform_c` DCM from `reference_base` to
//!   `IAU_<name>` (or the frame named in `planet_frames`) and its rate;
//! * for each spacecraft in [`SpiceInterfaceConfig::spacecraft_names`], the
//!   same state as a [`SpacecraftStateMsg`], an [`AttitudeReferenceMsg`] and
//!   a [`TranslationReferenceMsg`].
//!
//! These are the messages the ANISE ephemeris models publish
//! ([`super::anise_planet_ephemeris`], [`super::anise_sun_ephemeris`]), so
//! gravity, `ephemerisConverter`, eclipse and the other consumers take either.
//! [`SpiceInterface::sun_out_msg`] repeats the planet named `sun` as the
//! [`SunEphemerisMsg`] those consumers read.
//!
//! A planet's position and orientation can each be reconstructed from a
//! coarse grid of cached SPICE knots instead of queried at every update, or
//! turned off; see [`ReconstructionSettings`] and
//! [`SpiceInterface::set_planet_position_reconstruction`].
//!
//! Kernels are shared by every interface in the process: a kernel requested
//! twice is furnished once, and unloaded when the last interface that holds
//! it drops ([`count_kernels_loaded`], [`is_kernel_loaded`]). CSPICE is one
//! global state, so all calls into it are serialized behind one lock.
//!
//! # Kernels
//!
//! Upstream's default kernels are `naif0012.tls`, `pck00010.tpc`,
//! `de-403-masses.tpc` and `de430.bsp` from `supportData/EphemerisData`.
//! [`default_kernel_paths`], the default of
//! [`SpiceInterfaceConfig::kernel_paths`], takes the first three from
//! `assets/spice/` (`just fetch-spice-kernels`) and DE440s from
//! `assets/anise/de440s.bsp` (`just fetch-anise-assets`), the ephemeris the
//! ANISE models read. [`SpiceInterfaceConfig::spice_data_path`] loads
//! upstream's four file names from one directory instead; `just
//! fetch-spice-kernels` puts DE430 in `assets/spice/` for that. DE430 and
//! DE440 place the solar-system barycenter about 117 km apart, so states
//! relative to it differ by that much between the two.
//!
//! # Differences from upstream
//!
//! * The initial epoch is the simulation's start epoch (the context's epoch
//!   less the simulation time), the stand-in for upstream's `epochInMsg`, and
//!   is formatted as upstream formats that message. Setting
//!   [`SpiceInterfaceConfig::utc_cal_init`] (upstream `UTCCalInit`) instead
//!   parses that string; the module's epoch then differs from the context's.
//! * As upstream, the ephemeris time is the initial one plus the simulation
//!   time, so simulation seconds are TDB seconds. The context epoch (and the
//!   ANISE models reading it) advances in TAI instead; the two drift apart
//!   by the change of the periodic TDB - TT term, at most about 28 µs per
//!   day and 3.3 ms in all.
//! * Upstream's `Reset` loads the kernels, sets the initial epoch, finds the
//!   planet frames and runs one update. Here [`Module::init`] loads the
//!   kernels, the first update (or [`Module::reset`]) does the rest, and
//!   [`Module::reset`] also runs the update.
//! * Body names and output ports are fixed at construction (upstream's
//!   `addPlanetNames`/`addSpacecraftNames`); names of
//!   [`MAX_BODY_NAME_LENGTH`] characters or more are rejected there.
//! * Planet outputs carry no `J2000Current` or `PlanetName`; a body without a
//!   frame (`computeOrient` off) has no orientation instead of an identity
//!   one. [`SpacecraftStateMsg`] has no `r_CN_N`/`v_CN_N`; upstream copies
//!   `r_BN_N`/`v_BN_N` there.
//! * A failing exact SPICE query panics with CSPICE's message, where
//!   upstream's default `ABORT` error action ends the process. A kernel that
//!   fails to load is logged, as upstream continues past it.
//! * The deprecated `clearKeeper` (`kclear_c`), due for removal upstream on
//!   2026-11-20, is not ported.

mod reconstruction;
mod toolkit;

pub use self::reconstruction::{DEFAULT_KNOT_STEP_NANOS, ReconstructionSettings};

use std::collections::HashMap;
use std::fmt;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use hifitime::{Duration, Epoch, Unit};
use nalgebra::{Matrix3, Vector3};

use self::reconstruction::{Channel, ChannelMode, KnotSource, Reconstruction, normalize_body_name};
use self::toolkit::{SpiceKernel, Toolkit, canonical_path};
use crate::messages::{
    AttitudeReferenceMsg, Output, PlanetOrientation, PlanetStateMsg, SpacecraftStateMsg,
    SpiceTimeMsg, SunEphemerisMsg, TranslationReferenceMsg,
};
use crate::time::NANO2SEC;
use crate::utilities::rigid_body_kinematics::dcm_to_mrp;
use crate::{Module, SimulationContext};

/// Upstream `julianDateJ2000` \[d\]: Julian date of the J2000 epoch.
const JULIAN_DATE_J2000: f64 = 2451545.0;
/// Upstream `secondsPerDay` \[s\].
const SECONDS_PER_DAY: f64 = 86400.0;
/// Upstream `secondsPerWeek` \[s\].
const SECONDS_PER_WEEK: f64 = 604800.0;
/// Upstream `weeksPerRollover`.
const WEEKS_PER_ROLLOVER: u64 = 1024;
/// Upstream `leapSecondThreshold` \[s\]: an ET - UTC jump this large is a
/// leap second, not the roundoff of the periodic ET - TAI term.
const LEAP_SECOND_THRESHOLD_S: f64 = 0.5;
/// Upstream `GPSEpochTime`.
const GPS_EPOCH_TIME: &str = "1980 January 6, 00:00:00.0";

/// Upstream `MAX_BODY_NAME_LENGTH`: body names must be shorter.
pub const MAX_BODY_NAME_LENGTH: usize = 64;

/// Upstream's default `timeOutPicture`.
pub const DEFAULT_TIME_OUT_PICTURE: &str = "MON DD,YYYY  HR:MN:SC.#### (UTC) ::UTC";

/// The kernel file names upstream's `Reset` loads from `SPICEDataPath`.
pub const DEFAULT_KERNEL_NAMES: [&str; 4] = [
    "naif0012.tls",
    "pck00010.tpc",
    "de-403-masses.tpc",
    "de430.bsp",
];

/// Upstream's default kernel set with DE440s in place of DE430, from the
/// repository's `assets/` (relative paths, taken from the repository root).
pub fn default_kernel_paths() -> Vec<PathBuf> {
    [
        "assets/spice/naif0012.tls",
        "assets/spice/pck00010.tpc",
        "assets/spice/de-403-masses.tpc",
        "assets/anise/de440s.bsp",
    ]
    .into_iter()
    .map(PathBuf::from)
    .collect()
}

/// Upstream `countKernelsLoaded`: kernels loaded in CSPICE, by any interface.
pub fn count_kernels_loaded() -> usize {
    toolkit::lock().kernel_count()
}

/// Upstream `isKernelLoaded`: whether the kernel at `path` (canonicalized as
/// kernels are when loaded) is loaded in CSPICE.
pub fn is_kernel_loaded(path: &Path) -> bool {
    toolkit::lock().is_loaded(&canonical_path(path))
}

/// A configuration or usage error upstream reports with `bskError`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SpiceInterfaceError {
    /// A planet or spacecraft name of [`MAX_BODY_NAME_LENGTH`] characters or
    /// more.
    NameTooLong(String),
    /// Upstream's "Unable to load SPICE kernel": the path and CSPICE's
    /// message.
    KernelLoad { path: String, message: String },
    /// Upstream's "The output format string is not long enough"; the picture
    /// must have at least 7 characters.
    TimePictureTooShort(String),
    /// The output capacity of the picture does not fit a `SpiceInt`.
    TimePictureTooLong,
    /// Upstream's "knotStep must be positive", for the named planet.
    ZeroKnotStep(String),
    /// A CSPICE call failed; its message.
    Toolkit(String),
}

impl fmt::Display for SpiceInterfaceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NameTooLong(name) => write!(
                f,
                "spiceInterface: body name is {} characters, but the SPICE payload name supports \
                 at most {} characters: {name}",
                name.len(),
                MAX_BODY_NAME_LENGTH - 1
            ),
            Self::KernelLoad { path, message } => {
                write!(f, "Unable to load SPICE kernel: {path}: {message}")
            }
            Self::TimePictureTooShort(picture) => write!(
                f,
                "The output format string is not long enough. It must contain at least 7 \
                 characters. It is currently: {picture}"
            ),
            Self::TimePictureTooLong => {
                write!(f, "The output format string is too long for SPICE.")
            }
            Self::ZeroKnotStep(planet) => {
                write!(
                    f,
                    "spiceInterface: knotStep must be positive for planet '{planet}'."
                )
            }
            Self::Toolkit(message) => write!(f, "CSPICE: {message}"),
        }
    }
}

impl std::error::Error for SpiceInterfaceError {}

/// User-settable parameters of [`SpiceInterface`].
#[derive(Clone, Debug)]
pub struct SpiceInterfaceConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `addKernelPath(s)`: kernels to load, relative paths taken
    /// from the repository root. Default [`default_kernel_paths`] (upstream's
    /// Python default, `auto_configure_kernels`); when empty,
    /// [`Self::spice_data_path`] is used.
    pub kernel_paths: Vec<PathBuf>,
    /// Upstream `SPICEDataPath`: a directory holding [`DEFAULT_KERNEL_NAMES`],
    /// loaded when [`Self::kernel_paths`] is empty.
    pub spice_data_path: Option<PathBuf>,
    /// Upstream `referenceBase`: inertial frame of the states. Default
    /// `"j2000"`.
    pub reference_base: String,
    /// Upstream `zeroBase`: body the positions are relative to. Default
    /// `"SSB"`.
    pub zero_base: String,
    /// Upstream `timeOutPicture`: `timout_c` picture of
    /// [`SpiceInterface::current_time_string`].
    pub time_out_picture: String,
    /// Upstream `UTCCalInit`: a time string for the initial epoch. `None`
    /// takes the simulation's start epoch (upstream's `epochInMsg`).
    pub utc_cal_init: Option<String>,
    /// Upstream `addPlanetNames`: one planet state output per name.
    pub planet_names: Vec<String>,
    /// Upstream `planetFrames`: per planet (by index), a fixed frame in place
    /// of the default `IAU_<name>`; an empty or missing entry keeps the
    /// default. As upstream, spacecraft index this list too.
    pub planet_frames: Vec<String>,
    /// Upstream `addSpacecraftNames`: one set of spacecraft outputs per name.
    pub spacecraft_names: Vec<String>,
}

impl Default for SpiceInterfaceConfig {
    fn default() -> Self {
        Self {
            name: "spice_interface".to_string(),
            kernel_paths: default_kernel_paths(),
            spice_data_path: None,
            reference_base: "j2000".to_string(),
            zero_base: "SSB".to_string(),
            time_out_picture: DEFAULT_TIME_OUT_PICTURE.to_string(),
            utc_cal_init: None,
            planet_names: Vec::new(),
            planet_frames: Vec::new(),
            spacecraft_names: Vec::new(),
        }
    }
}

impl SpiceInterfaceConfig {
    /// Checks the body names (upstream's `addPlanetNames`/`addSpacecraftNames`
    /// `bskError`).
    pub fn validate(&self) -> Result<(), SpiceInterfaceError> {
        match self
            .planet_names
            .iter()
            .chain(&self.spacecraft_names)
            .find(|name| name.len() >= MAX_BODY_NAME_LENGTH)
        {
            Some(name) => Err(SpiceInterfaceError::NameTooLong(name.clone())),
            None => Ok(()),
        }
    }
}

/// One body's state (upstream `SpicePlanetStateMsgPayload`).
#[derive(Clone, Debug)]
struct BodyState {
    /// Upstream `PlanetName`.
    name: String,
    /// Upstream `PositionVector` \[m\].
    position_m: Vector3<f64>,
    /// Upstream `VelocityVector` \[m/s\].
    velocity_mps: Vector3<f64>,
    /// Upstream `J20002Pfix`: inertial-to-fixed DCM.
    inertial_to_fixed: Matrix3<f64>,
    /// Upstream `J20002Pfix_dot`: its time derivative \[1/s\].
    inertial_to_fixed_dot: Matrix3<f64>,
    /// Upstream `computeOrient`.
    compute_orient: bool,
}

impl BodyState {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            position_m: Vector3::zeros(),
            velocity_mps: Vector3::zeros(),
            inertial_to_fixed: Matrix3::identity(),
            inertial_to_fixed_dot: Matrix3::zeros(),
            compute_orient: false,
        }
    }

    fn planet_state_msg(&self) -> PlanetStateMsg {
        PlanetStateMsg {
            position_inertial_m: self.position_m,
            velocity_inertial_mps: self.velocity_mps,
            orientation: self.compute_orient.then_some(PlanetOrientation {
                inertial_to_fixed: self.inertial_to_fixed,
                inertial_to_fixed_dot: self.inertial_to_fixed_dot,
            }),
        }
    }
}

/// Planet, spacecraft and time states from CSPICE: upstream `SpiceInterface`.
pub struct SpiceInterface {
    /// User-settable parameters; the body lists are fixed at construction.
    pub config: SpiceInterfaceConfig,
    /// Upstream `spiceTimeOutMsg`: the current time in several scales.
    pub spice_time_out_msg: Output<SpiceTimeMsg>,
    /// Upstream `planetStateOutMsgs`: one state per planet, in the order of
    /// `config.planet_names`.
    pub planet_state_out_msgs: Vec<Output<PlanetStateMsg>>,
    /// The state of the planet named `sun` (any case), as the ANISE Sun
    /// model publishes it; left at its default without such a planet.
    pub sun_out_msg: Output<SunEphemerisMsg>,
    /// Upstream `scStateOutMsgs`: one state per spacecraft, the attitude the
    /// MRP of its DCM.
    pub sc_state_out_msgs: Vec<Output<SpacecraftStateMsg>>,
    /// Upstream `attRefStateOutMsgs`: each spacecraft attitude as a
    /// reference, zero rates.
    pub att_ref_state_out_msgs: Vec<Output<AttitudeReferenceMsg>>,
    /// Upstream `transRefStateOutMsgs`: each spacecraft position and velocity
    /// as a reference, zero acceleration.
    pub trans_ref_state_out_msgs: Vec<Output<TranslationReferenceMsg>>,
    /// Upstream `planetData`.
    planets: Vec<BodyState>,
    /// Upstream `scData`.
    spacecraft: Vec<BodyState>,
    /// Upstream `SPICELoaded`: the configured kernels have been loaded.
    spice_loaded: bool,
    /// Upstream `loadedKernels`: the kernels this interface holds, by
    /// canonical path.
    loaded_kernels: HashMap<String, Arc<SpiceKernel>>,
    /// The initial epoch string `str2et_c` parsed.
    utc_cal_init: String,
    /// Upstream `timeDataInit`.
    time_data_init: bool,
    /// Upstream `J2000ETInit` \[s\]: ephemeris time at simulation time zero.
    j2000_et_init: f64,
    /// Upstream `J2000Current` \[s\]: current ephemeris time.
    j2000_current: f64,
    /// Upstream `julianDateCurrent` \[d\]: current UTC Julian date.
    julian_date_current: f64,
    /// Upstream `GPSSeconds` \[s\].
    gps_seconds: f64,
    /// Upstream `GPSWeek`.
    gps_week: u16,
    /// Upstream `GPSRollovers`.
    gps_rollovers: u64,
    /// Upstream `TAIGPSEpoch` \[s\]: TAI of the GPS epoch, seconds past J2000.
    tai_gps_epoch: f64,
    /// Upstream `recon`: created by the first reconstruction setter.
    reconstruction: Option<Reconstruction>,
    /// Upstream `spiceQueryCounts`: {position, orientation} SPICE queries per
    /// uppercase body name since the last reset.
    spice_query_counts: HashMap<String, (i64, i64)>,
}

impl SpiceInterface {
    /// Builds the module with one output per planet and three per
    /// spacecraft, and looks up whether each spacecraft has a frame (upstream
    /// `addSpacecraftNames`, so only kernels already loaded count).
    ///
    /// # Panics
    ///
    /// If `config` fails [`SpiceInterfaceConfig::validate`].
    pub fn new(config: SpiceInterfaceConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid SpiceInterfaceConfig: {error}");
        }
        let planets: Vec<_> = config
            .planet_names
            .iter()
            .map(|n| BodyState::new(n))
            .collect();
        let spacecraft: Vec<_> = {
            let mut toolkit = toolkit::lock();
            config
                .spacecraft_names
                .iter()
                .map(|name| BodyState {
                    compute_orient: spice_ok(&config.name, "cnmfrm_c", toolkit.has_frame(name)),
                    ..BodyState::new(name)
                })
                .collect()
        };
        let spacecraft_count = spacecraft.len();
        Self {
            spice_time_out_msg: Output::default(),
            planet_state_out_msgs: planets.iter().map(|_| Output::default()).collect(),
            sun_out_msg: Output::default(),
            sc_state_out_msgs: (0..spacecraft_count).map(|_| Output::default()).collect(),
            att_ref_state_out_msgs: (0..spacecraft_count).map(|_| Output::default()).collect(),
            trans_ref_state_out_msgs: (0..spacecraft_count).map(|_| Output::default()).collect(),
            config,
            planets,
            spacecraft,
            spice_loaded: false,
            loaded_kernels: HashMap::new(),
            utc_cal_init: String::new(),
            time_data_init: false,
            j2000_et_init: 0.0,
            j2000_current: 0.0,
            julian_date_current: 0.0,
            gps_seconds: 0.0,
            gps_week: 0,
            gps_rollovers: 0,
            tai_gps_epoch: 0.0,
            reconstruction: None,
            spice_query_counts: HashMap::new(),
        }
    }

    /// Upstream `UTCCalInit` as used: the initial epoch string, empty until
    /// the time is initialized.
    pub fn utc_cal_init(&self) -> &str {
        &self.utc_cal_init
    }

    /// Upstream `timeDataInit`: whether the initial epoch has been set.
    pub fn time_data_init(&self) -> bool {
        self.time_data_init
    }

    /// Upstream `J2000ETInit` \[s\]: ephemeris time at simulation time zero.
    pub fn j2000_et_init(&self) -> f64 {
        self.j2000_et_init
    }

    /// Upstream `J2000Current` \[s\]: current ephemeris time past J2000.
    pub fn j2000_current(&self) -> f64 {
        self.j2000_current
    }

    /// Upstream `julianDateCurrent` \[d\]: current UTC Julian date.
    pub fn julian_date_current(&self) -> f64 {
        self.julian_date_current
    }

    /// Upstream `GPSSeconds` \[s\]: current GPS seconds of week.
    pub fn gps_seconds(&self) -> f64 {
        self.gps_seconds
    }

    /// Upstream `GPSWeek`: current GPS week, modulo 1024.
    pub fn gps_week(&self) -> u16 {
        self.gps_week
    }

    /// Upstream `GPSRollovers`: number of 1024-week GPS rollovers.
    pub fn gps_rollovers(&self) -> u64 {
        self.gps_rollovers
    }

    /// Upstream `loadSpiceKernel`: loads the kernel `name` in `directory` and
    /// holds it until [`Self::unload_spice_kernel`] or this interface drops.
    pub fn load_spice_kernel(
        &mut self,
        name: &str,
        directory: &Path,
    ) -> Result<(), SpiceInterfaceError> {
        let path = directory.join(name);
        let kernel =
            SpiceKernel::request(&path).map_err(|message| SpiceInterfaceError::KernelLoad {
                path: path.display().to_string(),
                message,
            })?;
        self.loaded_kernels
            .insert(kernel.path().to_string(), kernel);
        Ok(())
    }

    /// Upstream `unloadSpiceKernel`: releases this interface's hold on the
    /// kernel `name` in `directory`; CSPICE unloads it once no interface
    /// holds it.
    pub fn unload_spice_kernel(&mut self, name: &str, directory: &Path) {
        self.loaded_kernels
            .remove(&canonical_path(&directory.join(name)));
    }

    /// Upstream `getCurrentTimeString`: the current time through
    /// `config.time_out_picture`, at most the picture's length less six
    /// characters.
    pub fn current_time_string(&self) -> Result<String, SpiceInterfaceError> {
        let picture = &self.config.time_out_picture;
        if picture.len() < 7 {
            return Err(SpiceInterfaceError::TimePictureTooShort(picture.clone()));
        }
        let capacity = i32::try_from(picture.len() - 5)
            .map_err(|_| SpiceInterfaceError::TimePictureTooLong)?;
        toolkit::lock()
            .timout(self.j2000_current, picture, capacity)
            .map_err(SpiceInterfaceError::Toolkit)
    }

    /// Upstream `setPlanetPositionReconstruction`: reconstructs the planet's
    /// position (and velocity) from a knot grid. Applies from the next
    /// update; the planet must be in `config.planet_names` by the next reset.
    pub fn set_planet_position_reconstruction(
        &mut self,
        planet: &str,
        settings: ReconstructionSettings,
    ) -> Result<(), SpiceInterfaceError> {
        self.configure_reconstruction(planet, Channel::Position, settings)
    }

    /// Upstream `setPlanetOrientationReconstruction`: reconstructs the
    /// planet's orientation (and its rate) from a knot grid.
    pub fn set_planet_orientation_reconstruction(
        &mut self,
        planet: &str,
        settings: ReconstructionSettings,
    ) -> Result<(), SpiceInterfaceError> {
        self.configure_reconstruction(planet, Channel::Orientation, settings)
    }

    /// Upstream `setPlanetPositionDisabled`: the planet's position and
    /// velocity are zero, without a SPICE query.
    pub fn set_planet_position_disabled(&mut self, planet: &str) {
        self.reconstruction_state()
            .set_mode(planet, Channel::Position, ChannelMode::Disabled);
    }

    /// Upstream `setPlanetOrientationDisabled`: the planet's orientation is
    /// the identity with zero rate, without a SPICE query.
    pub fn set_planet_orientation_disabled(&mut self, planet: &str) {
        self.reconstruction_state()
            .set_mode(planet, Channel::Orientation, ChannelMode::Disabled);
    }

    /// Upstream `setPlanetExactSpice`: both channels of the planet back on
    /// the exact per-update SPICE query, the default.
    pub fn set_planet_exact_spice(&mut self, planet: &str) {
        let reconstruction = self.reconstruction_state();
        reconstruction.set_mode(planet, Channel::Position, ChannelMode::Exact);
        reconstruction.set_mode(planet, Channel::Orientation, ChannelMode::Exact);
    }

    /// Upstream `getPlanetSpiceQueryCount`: the {position, orientation} SPICE
    /// queries spent on the body since the last reset, exact path and knot
    /// fills alike; {0, 0} for a planet never queried, {-1, -1} for a name
    /// never added.
    pub fn planet_spice_query_count(&self, planet: &str) -> (i64, i64) {
        self.spice_query_counts
            .get(&normalize_body_name(planet))
            .copied()
            .unwrap_or((-1, -1))
    }

    fn configure_reconstruction(
        &mut self,
        planet: &str,
        channel: Channel,
        settings: ReconstructionSettings,
    ) -> Result<(), SpiceInterfaceError> {
        let reconstruction = self.reconstruction_state();
        if settings.knot_step_nanos == 0 {
            return Err(SpiceInterfaceError::ZeroKnotStep(planet.to_string()));
        }
        reconstruction.configure(planet, channel, settings);
        Ok(())
    }

    /// Upstream `reconState`: the reconstruction state, created on first use
    /// and anchored at the initial epoch if that is already set.
    fn reconstruction_state(&mut self) -> &mut Reconstruction {
        let et0 = if self.time_data_init {
            self.j2000_et_init
        } else {
            0.0
        };
        self.reconstruction
            .get_or_insert_with(|| Reconstruction::new(et0))
    }

    /// The kernel part of upstream's `Reset`: loads the configured kernels
    /// once, logging each failure.
    fn load_configured_kernels(&mut self) {
        let paths: Vec<PathBuf> = if !self.config.kernel_paths.is_empty() {
            self.config.kernel_paths.clone()
        } else if let Some(directory) = &self.config.spice_data_path {
            DEFAULT_KERNEL_NAMES
                .iter()
                .map(|name| directory.join(name))
                .collect()
        } else {
            log::error!(
                "{}: SPICE data path was not set.  No SPICE.",
                self.config.name
            );
            Vec::new()
        };
        for path in paths {
            if path.as_os_str().is_empty() {
                log::warn!("{}: ignoring empty kernel path", self.config.name);
                continue;
            }
            match SpiceKernel::request(&path) {
                Ok(kernel) => {
                    self.loaded_kernels
                        .insert(kernel.path().to_string(), kernel);
                }
                Err(message) => log::error!(
                    "{}: {}",
                    self.config.name,
                    SpiceInterfaceError::KernelLoad {
                        path: path.display().to_string(),
                        message,
                    }
                ),
            }
        }
        self.spice_loaded = true;
    }

    /// Upstream `Reset` without its closing update: loads the kernels if
    /// needed, sets the initial epoch, finds which planets have a frame,
    /// zeroes the query counts and resets the reconstruction caches.
    fn reset_state(&mut self, context: &SimulationContext) {
        if !self.spice_loaded {
            self.load_configured_kernels();
        }
        let mut toolkit = toolkit::lock();
        self.init_time_data(&mut toolkit, context);
        self.j2000_current = self.j2000_et_init;
        self.time_data_init = true;

        for (index, planet) in self.planets.iter_mut().enumerate() {
            planet.compute_orient = match self.config.planet_frames.get(index) {
                Some(frame) if !frame.is_empty() => true,
                _ => spice_ok(
                    &self.config.name,
                    "cnmfrm_c",
                    toolkit.has_frame(&planet.name),
                ),
            };
        }

        self.spice_query_counts = self
            .planets
            .iter()
            .map(|planet| (normalize_body_name(&planet.name), (0, 0)))
            .collect();

        if let Some(reconstruction) = &mut self.reconstruction {
            let planets: Vec<_> = self
                .planets
                .iter()
                .map(|planet| (planet.name.as_str(), planet.compute_orient))
                .collect();
            reconstruction.reset(self.j2000_et_init, &planets);
        }
    }

    /// Upstream `initTimeData`: the initial ephemeris time and the TAI of the
    /// GPS epoch.
    fn init_time_data(&mut self, toolkit: &mut Toolkit, context: &SimulationContext) {
        let name = &self.config.name;
        self.utc_cal_init = match &self.config.utc_cal_init {
            Some(utc_cal_init) => utc_cal_init.clone(),
            None => epoch_message_string(
                context.current_epoch
                    - Duration::from_total_nanoseconds(i128::from(context.current_sim_nanos)),
            ),
        };
        let et_gps_epoch = spice_ok(name, "str2et_c", toolkit.str2et(GPS_EPOCH_TIME));
        self.tai_gps_epoch = spice_ok(name, "unitim_c", toolkit.unitim(et_gps_epoch, "TDB", "TAI"));
        self.j2000_et_init = spice_ok(name, "str2et_c", toolkit.str2et(&self.utc_cal_init));
        if self.j2000_et_init < et_gps_epoch {
            log::warn!(
                "{name}: The initial time is before the GPS epoch.  The GPS time outputs are held \
                 at zero until the GPS epoch."
            );
        }
    }

    /// Upstream `computeGPSData`: GPS week, seconds of week and rollovers
    /// from the TAI elapsed since the GPS epoch, held at zero before it.
    fn compute_gps_data(&mut self, toolkit: &mut Toolkit) {
        let tai = spice_ok(
            &self.config.name,
            "unitim_c",
            toolkit.unitim(self.j2000_current, "TDB", "TAI"),
        );
        let gps_elapsed = (tai - self.tai_gps_epoch).max(0.0);
        let total_weeks = (gps_elapsed / SECONDS_PER_WEEK) as u64;
        self.gps_seconds = gps_elapsed - total_weeks as f64 * SECONDS_PER_WEEK;
        self.gps_rollovers = total_weeks / WEEKS_PER_ROLLOVER;
        self.gps_week = (total_weeks % WEEKS_PER_ROLLOVER) as u16;
    }

    /// Upstream `UpdateState` at simulation time `current_sim_nanos`.
    fn update_state(&mut self, toolkit: &mut Toolkit, current_sim_nanos: u64) {
        self.j2000_current = self.j2000_et_init + current_sim_nanos as f64 * NANO2SEC;

        // UTC Julian date from the UTC seconds past J2000. During an inserted
        // second those fall in the first second of the next day, whose
        // ET - UTC is larger; SPICE's JDUTC holds at that midnight.
        let name = &self.config.name;
        let delta_et = spice_ok(name, "deltet_c", toolkit.deltet(self.j2000_current, "ET"));
        let mut utc_seconds = self.j2000_current - delta_et;
        let utc_delta_et = spice_ok(name, "deltet_c", toolkit.deltet(utc_seconds, "UTC"));
        if utc_delta_et - delta_et > LEAP_SECOND_THRESHOLD_S {
            utc_seconds = utc_seconds.floor();
        }
        self.julian_date_current = JULIAN_DATE_J2000 + utc_seconds / SECONDS_PER_DAY;

        self.compute_gps_data(toolkit);
        let mut query = SpiceQuery {
            toolkit,
            reference_base: &self.config.reference_base,
            zero_base: &self.config.zero_base,
            counts: &mut self.spice_query_counts,
        };
        // Reconstruction is for planets only; spacecraft stay exact.
        pull_spice_data(
            &self.config.name,
            &mut self.planets,
            &self.config.planet_frames,
            self.reconstruction.as_mut(),
            &mut query,
            self.j2000_current,
            current_sim_nanos,
        );
        pull_spice_data(
            &self.config.name,
            &mut self.spacecraft,
            &self.config.planet_frames,
            None,
            &mut query,
            self.j2000_current,
            current_sim_nanos,
        );
        self.write_output_messages();
    }

    /// Upstream `writeOutputMessages`.
    fn write_output_messages(&self) {
        self.spice_time_out_msg.write(SpiceTimeMsg {
            j2000_current_s: self.j2000_current,
            julian_date_current_days: self.julian_date_current,
            gps_seconds_of_week: self.gps_seconds,
            gps_week: u32::from(self.gps_week),
            gps_rollovers: self.gps_rollovers,
        });
        for (output, planet) in self.planet_state_out_msgs.iter().zip(&self.planets) {
            output.write(planet.planet_state_msg());
        }
        if let Some(sun) = self
            .planets
            .iter()
            .find(|planet| planet.name.trim().eq_ignore_ascii_case("sun"))
        {
            self.sun_out_msg.write(SunEphemerisMsg {
                sun_position_inertial_m: sun.position_m,
                sun_velocity_inertial_mps: sun.velocity_mps,
            });
        }
        for (index, spacecraft) in self.spacecraft.iter().enumerate() {
            let sigma = dcm_to_mrp(spacecraft.inertial_to_fixed);
            self.sc_state_out_msgs[index].write(SpacecraftStateMsg {
                position_m: spacecraft.position_m,
                velocity_mps: spacecraft.velocity_mps,
                sigma_bn: sigma,
                ..SpacecraftStateMsg::default()
            });
            self.att_ref_state_out_msgs[index].write(AttitudeReferenceMsg {
                sigma_rn: sigma,
                ..AttitudeReferenceMsg::default()
            });
            self.trans_ref_state_out_msgs[index].write(TranslationReferenceMsg {
                position_m: spacecraft.position_m,
                velocity_mps: spacecraft.velocity_mps,
                ..TranslationReferenceMsg::default()
            });
        }
    }

    /// Default outputs, before the first update.
    fn write_default_outputs(&self) {
        self.spice_time_out_msg.write(SpiceTimeMsg::default());
        for output in &self.planet_state_out_msgs {
            output.write(PlanetStateMsg::default());
        }
        self.sun_out_msg.write(SunEphemerisMsg::default());
        for output in &self.sc_state_out_msgs {
            output.write(SpacecraftStateMsg::default());
        }
        for output in &self.att_ref_state_out_msgs {
            output.write(AttitudeReferenceMsg::default());
        }
        for output in &self.trans_ref_state_out_msgs {
            output.write(TranslationReferenceMsg::default());
        }
    }
}

impl Module for SpiceInterface {
    /// Loads the kernels and publishes default outputs; the initial epoch is
    /// set at the first update, when the simulation epoch is known.
    fn init(&mut self) {
        if !self.spice_loaded {
            self.load_configured_kernels();
        }
        self.time_data_init = false;
        self.write_default_outputs();
    }

    /// Upstream `Reset`: initial epoch, planet frames, query counts and
    /// reconstruction caches, then one update.
    fn reset(&mut self, context: &SimulationContext) {
        self.reset_state(context);
        self.update_state(&mut toolkit::lock(), context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        if !self.time_data_init {
            self.reset_state(context);
        }
        self.update_state(&mut toolkit::lock(), context.current_sim_nanos);
    }
}

/// The SPICE queries of one update, counted per body (upstream
/// `countSpiceQuery`).
struct SpiceQuery<'a> {
    toolkit: &'a mut Toolkit,
    reference_base: &'a str,
    zero_base: &'a str,
    counts: &'a mut HashMap<String, (i64, i64)>,
}

impl SpiceQuery<'_> {
    fn count(&mut self, body: &str, channel: Channel) {
        let counts = self.counts.entry(normalize_body_name(body)).or_default();
        match channel {
            Channel::Position => counts.0 += 1,
            Channel::Orientation => counts.1 += 1,
        }
    }
}

impl KnotSource for SpiceQuery<'_> {
    fn position(&mut self, planet: &str, et: f64) -> Result<(Vector3<f64>, Vector3<f64>), String> {
        let state = self
            .toolkit
            .state(planet, et, self.reference_base, self.zero_base);
        self.count(planet, Channel::Position);
        state
    }

    fn orientation(
        &mut self,
        planet: &str,
        frame: &str,
        et: f64,
    ) -> Result<(Matrix3<f64>, Matrix3<f64>), String> {
        let orientation = self.toolkit.orientation(self.reference_base, frame, et);
        self.count(planet, Channel::Orientation);
        orientation
    }
}

/// Upstream `pullSpiceData`: each body's state at ephemeris time `et`
/// (simulation time `t_nanos`), from `reconstruction` where it handles the
/// channel and from the exact SPICE query otherwise.
fn pull_spice_data(
    module_name: &str,
    bodies: &mut [BodyState],
    frames: &[String],
    mut reconstruction: Option<&mut Reconstruction>,
    query: &mut SpiceQuery<'_>,
    et: f64,
    t_nanos: u64,
) {
    for (index, body) in bodies.iter_mut().enumerate() {
        let frame = match frames.get(index) {
            Some(frame) if !frame.is_empty() => frame.clone(),
            _ => format!("IAU_{}", body.name),
        };

        let reconstructed = reconstruction
            .as_deref_mut()
            .and_then(|reconstruction| reconstruction.position(&body.name, t_nanos, query));
        (body.position_m, body.velocity_mps) = match reconstructed {
            Some(state) => state,
            None => spice_ok(module_name, "spkezr_c", query.position(&body.name, et)),
        };

        // A body without a frame keeps the identity attitude.
        if body.compute_orient {
            let reconstructed = reconstruction.as_deref_mut().and_then(|reconstruction| {
                reconstruction.orientation(&body.name, &frame, t_nanos, query)
            });
            (body.inertial_to_fixed, body.inertial_to_fixed_dot) = match reconstructed {
                Some(orientation) => orientation,
                None => spice_ok(
                    module_name,
                    "sxform_c",
                    query.orientation(&body.name, &frame, et),
                ),
            };
        }
    }
}

/// The value of a CSPICE call the module cannot continue without.
///
/// # Panics
///
/// With CSPICE's message if the call failed, where upstream's default error
/// action aborts.
fn spice_ok<T>(module_name: &str, routine: &str, result: Result<T, String>) -> T {
    result.unwrap_or_else(|message| panic!("{module_name}: {routine} failed: {message}"))
}

/// Upstream's epoch-message format, `"%4d/%02d/%02d, %02d:%02d:%04.6f (UTC)"`,
/// of `epoch` rounded to the microsecond.
fn epoch_message_string(epoch: Epoch) -> String {
    let (year, month, day, hours, minutes, seconds, nanos) =
        epoch.round(Unit::Microsecond * 1).to_gregorian_utc();
    let seconds = f64::from(seconds) + f64::from(nanos) * NANO2SEC;
    format!("{year:4}/{month:02}/{day:02}, {hours:02}:{minutes:02}:{seconds:04.6} (UTC)")
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `spiceInterface/_UnitTest`, plus a cross-check
    //! against the ANISE ephemeris models on the same kernels.
    //!
    //! CSPICE and its loaded-kernel count are process-wide, so every test
    //! here holds [`serial`]. A test whose kernels are missing prints why and
    //! passes; `just fetch-spice-kernels` and `just fetch-anise-assets` fetch
    //! them.
    //!
    //! Not ported:
    //! * `test_spiceBufferOwnership.py::test_scratch_storage_is_private`
    //!   checks that Python cannot reach the private `charBufferSize`; Rust
    //!   privacy is checked at compile time.
    //! * `test_unitSpice.py`'s missing-directory and long-name runs, which
    //!   upstream leaves commented out; the long-name rejection is
    //!   [`a_body_name_of_max_body_name_length_is_rejected`].

    use std::path::{Path, PathBuf};
    use std::sync::{Mutex, MutexGuard, PoisonError};

    use hifitime::{Duration, Epoch};
    use nalgebra::{Matrix3, Vector3};

    use super::*;
    use crate::environment::anise_planet_ephemeris::{
        AnisePlanetEphemeris, AnisePlanetEphemerisConfig,
    };
    use crate::environment::anise_sun_ephemeris::{AniseSunEphemeris, AniseSunEphemerisConfig};
    use crate::simulation::Simulation;

    static SERIAL: Mutex<()> = Mutex::new(());

    /// Exclusive use of CSPICE for one test.
    fn serial() -> MutexGuard<'static, ()> {
        SERIAL.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn repo_path(relative: &Path) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(relative)
    }

    /// Whether every kernel is present; prints why the test is skipped if
    /// not.
    fn kernels_present(paths: &[PathBuf]) -> bool {
        match paths.iter().find(|path| !repo_path(path).is_file()) {
            Some(missing) => {
                eprintln!(
                    "skipping: {} is missing; run `just fetch-spice-kernels` and `just \
                     fetch-anise-assets` to fetch it",
                    repo_path(missing).display()
                );
                false
            }
            None => true,
        }
    }

    fn default_kernels_present() -> bool {
        kernels_present(&default_kernel_paths())
    }

    /// Upstream `macros.sec2nano`.
    fn sec2nano(seconds: f64) -> u64 {
        (seconds * 1.0e9 + 0.5) as u64
    }

    fn context_at(start: Epoch, nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: nanos,
            current_epoch: start + Duration::from_total_nanoseconds(i128::from(nanos)),
        }
    }

    fn config_with_planets(planets: &[&str]) -> SpiceInterfaceConfig {
        SpiceInterfaceConfig {
            planet_names: planets.iter().map(|name| name.to_string()).collect(),
            ..SpiceInterfaceConfig::default()
        }
    }

    /// A kernel-less interface, upstream's `auto_configure_kernels=False`.
    fn config_without_kernels() -> SpiceInterfaceConfig {
        SpiceInterfaceConfig {
            kernel_paths: Vec::new(),
            ..SpiceInterfaceConfig::default()
        }
    }

    fn dcm(msg: &PlanetStateMsg) -> Matrix3<f64> {
        msg.orientation
            .as_ref()
            .map_or_else(Matrix3::identity, |o| o.inertial_to_fixed)
    }

    fn dcm_dot(msg: &PlanetStateMsg) -> Matrix3<f64> {
        msg.orientation
            .as_ref()
            .map_or_else(Matrix3::zeros, |o| o.inertial_to_fixed_dot)
    }

    /// Days since 1970-01-01 of a proleptic Gregorian date.
    fn days_from_civil(year: i64, month: i64, day: i64) -> i64 {
        let y = if month <= 2 { year - 1 } else { year };
        let era = y.div_euclid(400);
        let year_of_era = y - era * 400;
        let day_of_year = (153 * ((month + 9) % 12) + 2) / 5 + day - 1;
        let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
        era * 146_097 + day_of_era - 719_468
    }

    /// Python `isoweekday` of a day number (1 is Monday, 7 Sunday).
    fn iso_weekday(days: i64) -> i64 {
        (days + 3).rem_euclid(7) + 1
    }

    const MONTH_NAMES: [&str; 12] = [
        "January",
        "February",
        "March",
        "April",
        "May",
        "June",
        "July",
        "August",
        "September",
        "October",
        "November",
        "December",
    ];

    /// The Python `datetime` values of the upstream tests.
    #[derive(Clone, Copy, Debug)]
    struct DateTime {
        year: i64,
        month: i64,
        day: i64,
        hour: i64,
        minute: i64,
        second: i64,
        microsecond: i64,
    }

    impl DateTime {
        const fn new(year: i64, month: i64, day: i64, hour: i64, minute: i64, second: i64) -> Self {
            Self {
                year,
                month,
                day,
                hour,
                minute,
                second,
                microsecond: 0,
            }
        }

        fn total_microseconds(&self) -> i64 {
            let seconds = days_from_civil(self.year, self.month, self.day) * 86_400
                + self.hour * 3600
                + self.minute * 60
                + self.second;
            seconds * 1_000_000 + self.microsecond
        }

        /// `(self - other).total_seconds()`.
        fn seconds_since(&self, other: &Self) -> f64 {
            (self.total_microseconds() - other.total_microseconds()) as f64 / 1.0e6
        }

        fn plus_one_second(self) -> Self {
            let seconds = self.total_microseconds() / 1_000_000 + 1;
            let days = seconds.div_euclid(86_400);
            let second_of_day = seconds.rem_euclid(86_400);
            // Back to civil from the day number (inverse of days_from_civil).
            let z = days + 719_468;
            let era = z.div_euclid(146_097);
            let day_of_era = z - era * 146_097;
            let year_of_era =
                (day_of_era - day_of_era / 1460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
            let day_of_year =
                day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
            let mp = (5 * day_of_year + 2) / 153;
            let day = day_of_year - (153 * mp + 2) / 5 + 1;
            let month = if mp < 10 { mp + 3 } else { mp - 9 };
            let year = year_of_era + era * 400 + i64::from(month <= 2);
            Self {
                year,
                month,
                day,
                hour: second_of_day / 3600,
                minute: second_of_day % 3600 / 60,
                second: second_of_day % 60,
                microsecond: self.microsecond,
            }
        }

        /// `strftime("%Y %B %d, %H:%M:%S.%f")`.
        fn long_format(&self) -> String {
            format!(
                "{} {} {:02}, {:02}:{:02}:{:02}.{:06}",
                self.year,
                MONTH_NAMES[(self.month - 1) as usize],
                self.day,
                self.hour,
                self.minute,
                self.second,
                self.microsecond
            )
        }

        /// `strftime("%Y-%m-%d %H:%M:%S")`.
        fn iso_format(&self) -> String {
            format!(
                "{}-{:02}-{:02} {:02}:{:02}:{:02}",
                self.year, self.month, self.day, self.hour, self.minute, self.second
            )
        }
    }

    const GPS_EPOCH: DateTime = DateTime::new(1980, 1, 6, 0, 0, 0);
    const UNIX_EPOCH: DateTime = DateTime::new(1970, 1, 1, 0, 0, 0);
    const UNIX_EPOCH_JULIAN_DATE: f64 = 2440587.5;
    const WEEKS_PER_ROLLOVER: u64 = 1024;

    /// Upstream `simHelpers.timeStringToGregorianUTCMsg`: the UTC epoch of a
    /// SPICE time string, through `str2et_c` and a microsecond `et2utc_c`.
    fn time_string_to_utc_epoch(time: &str) -> Epoch {
        let leap_seconds = SpiceKernel::request(Path::new("assets/spice/naif0012.tls"))
            .expect("leap-seconds kernel loads");
        let utc = {
            let mut toolkit = toolkit::lock();
            let et = toolkit.str2et(time).expect("valid SPICE time string");
            toolkit.et2utc(et, "C", 6).expect("ET converts to UTC")
        };
        drop(leap_seconds);
        // "YYYY MON DD HR:MN:SC.ffffff"
        let fields: Vec<&str> = utc.split_whitespace().collect();
        let month = [
            "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC",
        ]
        .iter()
        .position(|name| *name == fields[1])
        .expect("month abbreviation") as u8
            + 1;
        let clock: Vec<&str> = fields[3].split(':').collect();
        let (whole, fraction) = clock[2].split_once('.').expect("fractional seconds");
        Epoch::from_gregorian_utc(
            fields[0].parse().expect("year"),
            month,
            fields[2].parse().expect("day"),
            clock[0].parse().expect("hour"),
            clock[1].parse().expect("minute"),
            whole.parse().expect("second"),
            fraction.parse::<u32>().expect("microseconds") * 1000,
        )
    }

    // ------------------------------------------------------------------
    // test_unitSpice.py
    // ------------------------------------------------------------------

    type Truth = [f64; 3];

    /// Upstream's parametrization: SPICE date, plot date, then the Mars
    /// barycenter, Earth and Sun positions \[km\] (JPL Horizons) and whether
    /// the epoch comes from the epoch message. The digits are upstream's.
    #[rustfmt::skip]
    #[allow(clippy::excessive_precision)]
    const UNIT_SPICE_CASES: [(&str, &str, Truth, Truth, Truth, bool); 24] = [
        ("2015 February 10, 00:00:00.0 TDB", "02/10/15", [2.049283795042291E+08, 4.654550957513031E+07, 1.580778617009296E+07], [-1.137790671899544E+08, 8.569008401822130E+07, 3.712507705247846E+07], [4.480338216752146E+05, -7.947764237588293E+04, -5.745748832696378E+04], false),
        ("2015 February 20, 00:00:00.0 TDB", "02/20/15", [1.997780190793433E+08, 6.636509140613769E+07, 2.503734390917690E+07], [-1.286636391244711E+08, 6.611156946652703E+07, 2.863736192793162E+07], [4.535258225415821E+05, -7.180260790174162E+04, -5.428151563919459E+04], false),
        ("2015 April 10, 00:00:00.0 TDB", "04/10/15", [1.468463828343225E+08, 1.502909016357645E+08, 6.495986693265321E+07], [-1.406808744055921E+08, -4.614996219219251E+07, -2.003047222725225E+07], [4.786772771370058E+05, -3.283487082146838E+04, -3.809690999538498E+04], false),
        ("2015 April 20, 00:00:00.0 TDB", "04/20/15", [1.313859463489376E+08, 1.637422864185919E+08, 7.154683454681394E+07], [-1.304372112275012E+08, -6.769916175223185E+07, -2.937179538983412E+07], [4.834188130324496E+05, -2.461799304268214E+04, -3.467083541217462E+04], false),
        ("2015 June 10, 00:00:00.0 TDB", "06/10/15", [3.777078276008123E+07, 2.080046702252371E+08, 9.437503998071109E+07], [-2.946784585692780E+07, -1.365779215199542E+08, -5.923299652516938E+07], [5.052070933145228E+05, 1.837038638578682E+04, -1.665575509449521E+04], false),
        ("2015 June 20, 00:00:00.0 TDB", "06/20/15", [1.778745850655047E+07, 2.116712756672253E+08, 9.659608128589308E+07], [-4.338053580692466E+06, -1.393743714818930E+08, -6.044500073550620E+07], [5.090137386469169E+05, 2.694272566092012E+04, -1.304862690440150E+04], false),
        ("2015 August 10, 00:00:00.0 TDB", "08/10/15", [-8.302866300591002E+07, 2.053384243312354E+08, 9.641192179982041E+07], [1.111973130587749E+08, -9.507060674145325E+07, -4.123957889640842E+07], [5.263951240980130E+05, 7.113788303899391E+04, 5.601415938789949E+03], false),
        ("2015 August 20, 00:00:00.0 TDB", "08/20/15", [-1.015602120938274E+08, 1.994877113707506E+08, 9.422840996376510E+07], [1.267319535735967E+08, -7.664279872369213E+07, -3.325170492059625E+07], [5.294368386862668E+05, 7.989635630604146E+04, 9.307380417148834E+03], false),
        ("2015 October 10, 00:00:00.0 TDB", "10/10/15", [-1.828944464171771E+08, 1.498117608528323E+08, 7.363786404040858E+07], [1.440636517249971E+08, 3.827074461651713E+07, 1.656440704503509E+07], [5.433268959250135E+05, 1.251717566539142E+05, 2.849999378507032E+04], false),
        ("2015 October 20, 00:00:00.0 TDB", "10/20/15", [-1.955685835661002E+08, 1.367530082668284E+08, 6.799006120628108E+07], [1.343849818314204E+08, 6.019977403127116E+07, 2.607024615553362E+07], [5.457339294611338E+05, 1.342148834831663E+05, 3.234185290692726E+04], false),
        ("2015 December 10, 00:00:00.0 TDB", "12/10/15", [-2.392022492203017E+08, 5.847873056287902E+07, 3.326431285934674E+07], [3.277145427626925E+07, 1.320956053465003E+08, 5.723804900679157E+07], [5.559607335969181E+05, 1.813263443761486E+05, 5.242991145066972E+04], false),
        ("2015 December 20, 00:00:00.0 TDB", "12/20/15", [-2.432532635321551E+08, 4.156075241419497E+07, 2.561357000250468E+07], [6.866134677340530E+06, 1.351080593806486E+08, 5.854460725992832E+07], [5.575179227151767E+05, 1.907337298309725E+05, 5.645553733620559E+04], false),
        ("2016 February 10, 00:00:00.0 TDB", "02/10/16", [-2.385499316808322E+08, -4.818711325555268E+07, -1.567983931044450E+07], [-1.132504603146183E+08, 8.647183658089657E+07, 3.746046979137553E+07], [5.630027736619673E+05, 2.402590276126245E+05, 7.772804647512530E+04], false),
        ("2016 February 20, 00:00:00.0 TDB", "02/20/16", [-2.326189626865872E+08, -6.493600278878165E+07, -2.352250994262638E+07], [-1.282197228963156E+08, 6.695033443521750E+07, 2.899822684259482E+07], [5.635403728358555E+05, 2.498445036007692E+05, 8.185973180152237E+04], false),
        ("2016 April 10, 00:00:00.0 TDB", "04/10/16", [-1.795928090776869E+08, -1.395102817786797E+08, -5.916067235603344E+07], [-1.399773606371217E+08, -4.749126225182984E+07, -2.061331784067504E+07], [5.639699901756521E+05, 2.977755140828988E+05, 1.025609223621660E+05], false),
        ("2016 April 20, 00:00:00.0 TDB", "04/20/16", [-1.646488222290798E+08, -1.517361128528306E+08, -6.517204439842616E+07], [-1.294310199316289E+08, -6.890085351639988E+07, -2.989515576444890E+07], [5.636348418836893E+05, 3.073681895822543E+05, 1.067117713233756E+05], false),
        ("2016 June 10, 00:00:00.0 TDB", "06/10/16", [-7.085675304355654E+07, -1.933788289169757E+08, -8.680583098365544E+07], [-2.756178030784301E+07, -1.365892814324490E+08, -5.923888961370525E+07], [5.597493293268962E+05, 3.563312003229167E+05, 1.279448896916322E+05], false),
        ("2016 June 20, 00:00:00.0 TDB", "06/20/16", [-4.994076874358515E+07, -1.967336617729592E+08, -8.890950206156498E+07], [-2.413995783152328E+06, -1.390828611356292E+08, -6.032062925759617E+07], [5.585605124166313E+05, 3.659402953599973E+05, 1.321213212542782E+05], false),
        ("2016 August 10, 00:00:00.0 TDB", "08/10/16", [5.965895856067932E+07, -1.851251207911916E+08, -8.654489416392270E+07], [1.124873641861596E+08, -9.344410235679612E+07, -4.053621796412993E+07], [5.501477436262188E+05, 4.149525682073312E+05, 1.534983234437988E+05], false),
        ("2016 August 20, 00:00:00.0 TDB", "08/20/16", [8.021899957229958E+07, -1.770523551156136E+08, -8.339737954004113E+07], [1.277516713236801E+08, -7.484361731649198E+07, -3.247232896320245E+07], [5.480148786547601E+05, 4.245199573757614E+05, 1.576874582857746E+05], false),
        ("2016 October 10, 00:00:00.0 TDB", "10/10/16", [1.674991232418756E+08, -1.090427183510760E+08, -5.456038490399034E+07], [1.434719792031415E+08, 4.029387436854774E+07, 1.744057129058395E+07], [5.348794087050703E+05, 4.727986075510736E+05, 1.788947974297997E+05], false),
        ("2016 October 20, 00:00:00.0 TDB", "10/20/16", [1.797014954219412E+08, -9.133803506487390E+07, -4.676932170648168E+07], [1.334883617893556E+08, 6.209917895944476E+07, 2.689423661839254E+07], [5.318931290166953E+05, 4.821605725626923E+05, 1.830201949404063E+05], false),
        ("2016 December 10, 00:00:00.0 TDB", "12/10/16", [2.087370835407280E+08, 1.035903131428964E+07, -9.084001492689754E+05], [3.082308903715757E+07, 1.328026978502711E+08, 5.754559376449955E+07], [5.147792796720199E+05, 5.293951386498961E+05, 2.038816823549219E+05], false),
        ("2016 December 20, 00:00:00.0 TDB", "12/20/16", [2.075411186038260E+08, 3.092173198610598E+07, 8.555251204561792E+06], [4.885421269793877E+06, 1.355125217382336E+08, 5.872015978003684E+07], [5.110736843893399E+05, 5.385860060393942E+05, 2.079481168492821E+05], true),
    ];

    /// Upstream `unitSpice` for one parametrization; the failure messages.
    fn unit_spice(
        date_spice: &str,
        date_plot: &str,
        mars_truth_km: Truth,
        earth_truth_km: Truth,
        sun_truth_km: Truth,
        use_msg: bool,
    ) -> Vec<String> {
        let mut messages = Vec::new();
        let planet_names = ["earth", "mars barycenter", "sun"];
        // Upstream's own kernel set from its `SPICEDataPath`: the Horizons
        // truth is DE430's, whose barycenter is ~117 km from DE440's.
        let mut config = SpiceInterfaceConfig {
            name: "SpiceInterfaceData".to_string(),
            kernel_paths: Vec::new(),
            spice_data_path: Some(repo_path(Path::new("assets/spice"))),
            utc_cal_init: Some(date_spice.to_string()),
            ..config_with_planets(&planet_names)
        };
        if use_msg {
            // The planet frame names can be set, and an empty one keeps the
            // default IAU frame.
            config.planet_frames = vec!["IAU_earth".into(), String::new(), "IAU_sun".into()];
            // The simulation epoch is the epoch message; upstream also sets a
            // wrong `UTCCalInit` to check the message overrides it, which here
            // `utc_cal_init = None` is.
            config.utc_cal_init = None;
        }
        let start = time_string_to_utc_epoch(date_spice);
        let mut spice = SpiceInterface::new(config);
        let time_out = spice.spice_time_out_msg.clone();
        let planet_out = spice.planet_state_out_msgs.clone();

        // (time [ns], GPS seconds, Julian date) at every update.
        let step = sec2nano(0.1);
        let stop = 60 * 1_000_000_000;
        let mut log = Vec::new();
        {
            let mut sim = Simulation::new(start, false);
            sim.add_module("spice", &mut spice, step, 0);
            sim.run_for(0);
            loop {
                let time = time_out.read();
                log.push((
                    sim.current_sim_nanos(),
                    time.gps_seconds_of_week,
                    time.julian_date_current_days,
                ));
                if sim.current_sim_nanos() >= stop {
                    break;
                }
                sim.run_for(step);
            }
        }

        let year: i64 = format!("20{}", &date_plot[6..8]).parse().unwrap();
        let month: i64 = date_plot[0..2].parse().unwrap();
        let day: i64 = date_plot[3..5].parse().unwrap();
        let date_days = days_from_civil(year, month, day);
        let sunday = iso_weekday(date_days) == 7;
        let gps_time_seconds = ((date_days - days_from_civil(1980, 1, 6)) * 86_400) as f64;
        let julian_start_date = UNIX_EPOCH_JULIAN_DATE + date_days as f64;

        // The GPS seconds advance with the simulation time (a Sunday ends a
        // GPS week, so is skipped).
        let initial_difference = log[0].1 - log[0].0 as f64 * 1.0e-9;
        let mut gps_row = log[0];
        for &row in &log[1..] {
            if sunday {
                continue;
            }
            gps_row = row;
            let difference = row.1 - row.0 as f64 * 1.0e-9;
            if (difference - initial_difference).abs() > 1.0e-6 {
                messages.push(format!(
                    "{date_spice}: time delta check failed with difference of {}",
                    difference - initial_difference
                ));
            }
        }

        // Absolute GPS time, with the leap second of 2015 June 30 and less
        // the periodic TDB - TT term of the leap-seconds kernel.
        let after_leap = (year, month, day) > (2015, 6, 30);
        let mut gps_end_time = if after_leap {
            gps_time_seconds + 17.0 + 60.0 - 68.184
        } else {
            gps_time_seconds + 16.0 + 60.0 - 67.184
        };
        let et_end = ((date_days - days_from_civil(2000, 1, 1)) * 86_400 - 43_200) as f64 + 60.0;
        let mean_anomaly = 6.239996 + 1.99096871e-7 * et_end;
        let eccentric_anomaly = mean_anomaly + 1.671e-2 * mean_anomaly.sin();
        gps_end_time -= 1.657e-3 * eccentric_anomaly.sin();
        let gps_week = (gps_end_time / (86_400.0 * 7.0)) as i64;
        let gps_second_assumed = gps_end_time - (gps_week * 86_400 * 7) as f64;
        let gps_second_difference = (gps_row.1 - gps_second_assumed).abs();
        let gps_tolerance = if use_msg { 2.0e-2 } else { 1.0e-4 };
        if !sunday && gps_second_difference > gps_tolerance {
            messages.push(format!(
                "{date_spice}: absolute GPS time check failed with difference of \
                 {gps_second_difference}"
            ));
        }

        // Absolute UTC Julian date.
        let jd_end_time =
            julian_start_date + 0.0006944440 - if after_leap { 68.184 } else { 67.184 } / 86_400.0;
        let jd_end_sim = log.last().unwrap().2;
        if (jd_end_sim - jd_end_time).abs() > 0.1 / (24.0 * 3600.0) {
            messages.push(format!(
                "{date_spice}: absolute Julian date check failed with difference of {}",
                (jd_end_sim - jd_end_time).abs()
            ));
        }

        // Planet positions against JPL Horizons.
        let position_tolerance_m = if use_msg { 1000.0 } else { 250.0 };
        for (index, truth_km, body) in [
            (1, mars_truth_km, "Mars"),
            (0, earth_truth_km, "Earth"),
            (2, sun_truth_km, "Sun"),
        ] {
            let position = planet_out[index].read().position_inertial_m;
            let error_m = (position - Vector3::from(truth_km) * 1000.0).norm();
            if error_m > position_tolerance_m {
                messages.push(format!(
                    "{date_spice}: {body} position check failed with difference of {error_m}"
                ));
            }
        }
        messages
    }

    #[test]
    fn planet_states_and_time_match_jpl_horizons() {
        let _serial = serial();
        let upstream_kernels: Vec<_> = DEFAULT_KERNEL_NAMES
            .iter()
            .map(|name| Path::new("assets/spice").join(name))
            .collect();
        if !kernels_present(&upstream_kernels) {
            return;
        }
        let failures: Vec<String> = UNIT_SPICE_CASES
            .iter()
            .flat_map(|&(date_spice, date_plot, mars, earth, sun, use_msg)| {
                unit_spice(date_spice, date_plot, mars, earth, sun, use_msg)
            })
            .collect();
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    /// Upstream `test_59_999999_second_epoch` (issue #690): an epoch a
    /// microsecond short of a minute is not rounded up.
    #[test]
    fn an_epoch_just_before_a_minute_keeps_its_seconds() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let start = time_string_to_utc_epoch("2015 February 10, 00:00:59.999999 UTC");
        let mut spice = SpiceInterface::new(SpiceInterfaceConfig {
            name: "SpiceInterfaceData".to_string(),
            ..SpiceInterfaceConfig::default()
        });
        {
            let mut sim = Simulation::new(start, false);
            sim.add_module("spice", &mut spice, sec2nano(0.1), 0);
            sim.run_for(60 * 1_000_000_000);
        }
        let utc_cal_init = spice.utc_cal_init();
        assert!(
            utc_cal_init.ends_with("59.999999 (UTC)"),
            "initial epoch string {utc_cal_init}"
        );
    }

    #[test]
    fn a_body_name_of_max_body_name_length_is_rejected() {
        let long_name = "x".repeat(MAX_BODY_NAME_LENGTH);
        let config = SpiceInterfaceConfig {
            planet_names: vec!["earth".into(), long_name.clone()],
            ..config_without_kernels()
        };
        assert_eq!(
            config.validate(),
            Err(SpiceInterfaceError::NameTooLong(long_name))
        );
        let longest_accepted = SpiceInterfaceConfig {
            spacecraft_names: vec!["y".repeat(MAX_BODY_NAME_LENGTH - 1)],
            ..config_without_kernels()
        };
        assert_eq!(longest_accepted.validate(), Ok(()));
    }

    // ------------------------------------------------------------------
    // test_unitSpiceSpacecraft.py
    // ------------------------------------------------------------------

    #[test]
    fn hubble_state_matches_jpl_horizons() {
        let _serial = serial();
        let hubble = PathBuf::from("assets/spice/hst_edited.bsp");
        if !default_kernels_present() || !kernels_present(std::slice::from_ref(&hubble)) {
            return;
        }
        let mut spice = SpiceInterface::new(SpiceInterfaceConfig {
            name: "SpiceInterfaceData".to_string(),
            spacecraft_names: vec!["HUBBLE SPACE TELESCOPE".to_string()],
            utc_cal_init: Some("2015 February 10, 00:00:00.0 TDB".to_string()),
            zero_base: "earth".to_string(),
            ..SpiceInterfaceConfig::default()
        });
        let kernel_dir = repo_path(hubble.parent().unwrap());
        spice
            .load_spice_kernel("hst_edited.bsp", &kernel_dir)
            .expect("Hubble kernel loads");
        {
            let mut sim =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2015, 2, 10), false);
            sim.add_module("spice", &mut spice, sec2nano(0.1), 0);
            sim.run_for(sec2nano(0.1));
        }
        spice.unload_spice_kernel("hst_edited.bsp", &kernel_dir);

        let truth_position =
            Vector3::new(-5855529.540348052, 1986110.860522791, -3116764.7117067943);
        let truth_velocity =
            Vector3::new(-1848.9038338503085, -7268.515626753905, -1155.3578832725618);
        let accuracy = 0.01;
        let sc_state = spice.sc_state_out_msgs[0].read();
        assert!(
            (sc_state.position_m - truth_position).norm() <= accuracy,
            "r_BN_N {}",
            sc_state.position_m
        );
        assert!(
            (sc_state.velocity_mps - truth_velocity).norm() <= accuracy,
            "v_BN_N {}",
            sc_state.velocity_mps
        );
        assert!(
            sc_state.sigma_bn.norm() <= accuracy,
            "sigma_BN {}",
            sc_state.sigma_bn
        );
        let att_ref = spice.att_ref_state_out_msgs[0].read();
        assert!(att_ref.sigma_rn.norm() <= accuracy);
        assert!(att_ref.omega_rn_n_radps.norm() <= accuracy);
        assert!(att_ref.domega_rn_n_radps2.norm() <= accuracy);
        let trans_ref = spice.trans_ref_state_out_msgs[0].read();
        assert!((trans_ref.position_m - truth_position).norm() <= accuracy);
        assert!((trans_ref.velocity_mps - truth_velocity).norm() <= accuracy);
        assert!(trans_ref.acceleration_mps2.norm() <= accuracy);
    }

    // ------------------------------------------------------------------
    // test_spiceTimeOutput.py
    // ------------------------------------------------------------------

    /// Upstream `run_spice_time_output`: one update at `epoch`.
    fn run_spice_time_output(epoch: DateTime) -> SpiceTimeMsg {
        let mut spice = SpiceInterface::new(SpiceInterfaceConfig {
            utc_cal_init: Some(format!("{} (UTC)", epoch.long_format())),
            ..SpiceInterfaceConfig::default()
        });
        let time_out = spice.spice_time_out_msg.clone();
        let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2000, 1, 1), false);
        sim.add_module("spice", &mut spice, sec2nano(1.0), 0);
        sim.run_for(0);
        time_out.read()
    }

    #[test]
    fn gps_time_and_julian_date_follow_their_definitions() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let mut fractional = DateTime::new(2026, 3, 20, 12, 0, 0);
        fractional.microsecond = 123_000;
        for (epoch, gps_utc_offset) in [
            // GPS week 1024 starts at this UTC time
            (DateTime::new(1999, 8, 21, 23, 59, 47), 13.0),
            // just after the leap second of 2015 June 30
            (DateTime::new(2015, 7, 1, 0, 0, 0), 17.0),
            // TDB - TT near its maximum of about +1.7 ms
            (DateTime::new(2024, 4, 4, 0, 0, 0), 18.0),
            // fractional UTC seconds
            (fractional, 18.0),
            // past GPS week 3551, where week*604800 s overflows a 32-bit int
            (DateTime::new(2048, 6, 1, 0, 0, 0), 18.0),
        ] {
            let time = run_spice_time_output(epoch);
            let gps_time = epoch.seconds_since(&GPS_EPOCH) + gps_utc_offset;
            let julian_date =
                UNIX_EPOCH_JULIAN_DATE + epoch.seconds_since(&UNIX_EPOCH) / SECONDS_PER_DAY;
            let gps_week = u64::from(time.gps_week) + WEEKS_PER_ROLLOVER * time.gps_rollovers;
            assert_eq!(
                gps_week,
                (gps_time / SECONDS_PER_WEEK).floor() as u64,
                "{epoch:?}"
            );
            assert!(
                (time.gps_seconds_of_week - gps_time.rem_euclid(SECONDS_PER_WEEK)).abs() <= 1.0e-6,
                "{epoch:?}: GPS seconds {}",
                time.gps_seconds_of_week
            );
            assert!(
                (time.julian_date_current_days - julian_date).abs() <= 1.0e-9,
                "{epoch:?}: Julian date {}",
                time.julian_date_current_days
            );
        }
    }

    #[test]
    fn gps_time_is_held_at_zero_before_the_gps_epoch() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let time = run_spice_time_output(DateTime::new(1979, 6, 1, 0, 0, 0));
        assert_eq!(time.gps_week, 0);
        assert_eq!(time.gps_rollovers, 0);
        assert_eq!(time.gps_seconds_of_week, 0.0);
    }

    /// Upstream `test_time_output_across_midnight`: the Julian date holds at
    /// midnight through an inserted leap second while GPS time advances.
    #[test]
    fn julian_date_holds_through_a_leap_second() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        for (epoch, gps_utc_offset, has_leap_second) in [
            (DateTime::new(1998, 12, 31, 23, 59, 59), 12.0, true), // before J2000
            (DateTime::new(2015, 6, 30, 23, 59, 59), 16.0, true),  // June leap second
            (DateTime::new(2016, 12, 31, 23, 59, 59), 17.0, true), // December leap second
            (DateTime::new(2026, 3, 20, 23, 59, 59), 18.0, false), // ordinary midnight
        ] {
            let mut spice = SpiceInterface::new(SpiceInterfaceConfig {
                utc_cal_init: Some(format!("{} UTC", epoch.iso_format())),
                ..SpiceInterfaceConfig::default()
            });
            let start = Epoch::from_gregorian_utc_at_midnight(2000, 1, 1);
            spice.init();
            spice.reset(&context_at(start, 0));

            let midnight = epoch.plus_one_second();
            let midnight_julian_date =
                UNIX_EPOCH_JULIAN_DATE + midnight.seconds_since(&UNIX_EPOCH) / SECONDS_PER_DAY;
            let initial_gps_time = epoch.seconds_since(&GPS_EPOCH) + gps_utc_offset;
            let mut previous_julian_date = None;
            for elapsed in [
                0.0, 0.5, 0.999, 1.0, 1.001, 1.5, 1.999, 2.0, 2.001, 2.5, 3.0,
            ] {
                spice.update(&context_at(start, sec2nano(elapsed)));
                let time = spice.spice_time_out_msg.read();
                let seconds_from_midnight: f64 = if has_leap_second {
                    (elapsed - 1.0f64).min(0.0) + (elapsed - 2.0f64).max(0.0)
                } else {
                    elapsed - 1.0
                };
                let expected_julian_date =
                    midnight_julian_date + seconds_from_midnight / SECONDS_PER_DAY;
                let expected_gps_time = initial_gps_time + elapsed;
                let gps_week = u64::from(time.gps_week) + WEEKS_PER_ROLLOVER * time.gps_rollovers;

                let label = format!("{epoch:?} + {elapsed} s");
                assert!(
                    (time.julian_date_current_days - expected_julian_date).abs() <= 1.0e-9,
                    "{label}: Julian date {}",
                    time.julian_date_current_days
                );
                if has_leap_second && (1.0..=2.0).contains(&elapsed) {
                    assert_eq!(
                        time.julian_date_current_days, midnight_julian_date,
                        "{label}"
                    );
                }
                if let Some(previous) = previous_julian_date {
                    assert!(time.julian_date_current_days >= previous, "{label}");
                }
                previous_julian_date = Some(time.julian_date_current_days);
                assert_eq!(
                    gps_week,
                    (expected_gps_time / SECONDS_PER_WEEK).floor() as u64,
                    "{label}"
                );
                assert!(
                    (time.gps_seconds_of_week - expected_gps_time.rem_euclid(SECONDS_PER_WEEK))
                        .abs()
                        <= 1.0e-6,
                    "{label}: GPS seconds {}",
                    time.gps_seconds_of_week
                );
            }
        }
    }

    #[test]
    fn gps_time_starts_when_the_simulation_crosses_the_gps_epoch() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let mut spice = SpiceInterface::new(SpiceInterfaceConfig {
            utc_cal_init: Some("1980 JAN 05 23:59:59 UTC".to_string()),
            ..SpiceInterfaceConfig::default()
        });
        let start = Epoch::from_gregorian_utc_at_midnight(2000, 1, 1);
        spice.init();
        spice.reset(&context_at(start, 0));
        for elapsed in [0.0, 0.5, 1.0, 1.5, 2.0] {
            spice.update(&context_at(start, sec2nano(elapsed)));
            let time = spice.spice_time_out_msg.read();
            let expected_seconds = (elapsed - 1.0f64).max(0.0);
            assert_eq!(time.gps_week, 0);
            assert_eq!(time.gps_rollovers, 0);
            assert!((time.gps_seconds_of_week - expected_seconds).abs() <= 1.0e-6);
        }
    }

    // ------------------------------------------------------------------
    // test_spiceBufferOwnership.py
    // ------------------------------------------------------------------

    #[test]
    fn time_conversion_survives_repeated_resets() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        for (picture, expected) in [
            (None, "JAN 01,2000  12:00:01.0000 (UTC)"),
            (
                Some("YYYY-MM-DD HR:MN:SC.### ::UTC"),
                "2000-01-01 12:00:01.000",
            ),
        ] {
            let mut config = SpiceInterfaceConfig {
                utc_cal_init: Some("2000 JAN 01 12:00:00 UTC".to_string()),
                ..SpiceInterfaceConfig::default()
            };
            if let Some(picture) = picture {
                config.time_out_picture = picture.to_string();
            }
            let mut spice = SpiceInterface::new(config);
            let start = Epoch::from_gregorian_utc_at_midnight(2000, 1, 1);
            spice.init();
            let elapsed_seconds = 1.0;
            let initial_julian_date = 2451545.0;
            for _ in 0..3 {
                spice.reset(&context_at(start, 0));
                spice.update(&context_at(start, sec2nano(elapsed_seconds)));
                assert_eq!(spice.current_time_string().unwrap(), expected);
                assert!(
                    (spice.julian_date_current()
                        - (initial_julian_date + elapsed_seconds / SECONDS_PER_DAY))
                        .abs()
                        <= 1.0e-9
                );
            }
        }
    }

    #[test]
    fn a_time_picture_shorter_than_seven_characters_is_rejected() {
        for picture_length in 0..7 {
            let spice = SpiceInterface::new(SpiceInterfaceConfig {
                time_out_picture: "X".repeat(picture_length),
                ..config_without_kernels()
            });
            let error = spice.current_time_string().unwrap_err();
            assert!(
                matches!(error, SpiceInterfaceError::TimePictureTooShort(_)),
                "{error}"
            );
            assert!(
                error
                    .to_string()
                    .contains("output format string is not long enough")
            );
        }
    }

    // ------------------------------------------------------------------
    // test_multipleInterfaces.py, test_spiceThreadSafety.py
    // ------------------------------------------------------------------

    /// Upstream `createOneSim`: an interface run for two 1 ns ticks, long
    /// enough to furnish its kernels.
    fn create_one_sim() -> SpiceInterface {
        let mut spice = SpiceInterface::new(SpiceInterfaceConfig::default());
        let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        sim.add_module("spice", &mut spice, 1, 0);
        sim.run_for(2);
        drop(sim);
        spice
    }

    /// Kernels are furnished once however many interfaces request them, and
    /// unloaded when the last one drops.
    #[test]
    fn kernels_are_shared_and_released_with_the_last_interface() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        assert_eq!(count_kernels_loaded(), 0);
        {
            let first = create_one_sim();
            let kernels_with_one_sim = count_kernels_loaded();
            assert!(kernels_with_one_sim > 0);
            for path in default_kernel_paths() {
                assert!(is_kernel_loaded(&path), "{}", path.display());
            }
            let more: Vec<_> = (0..20).map(|_| create_one_sim()).collect();
            assert_eq!(count_kernels_loaded(), kernels_with_one_sim);
            drop(first);
            assert_eq!(count_kernels_loaded(), kernels_with_one_sim);
            drop(more);
        }
        assert_eq!(count_kernels_loaded(), 0);
    }

    #[test]
    fn a_kernel_held_twice_is_unloaded_only_after_both_release_it() {
        let _serial = serial();
        let directory = repo_path(Path::new("assets/spice"));
        if !kernels_present(&[PathBuf::from("assets/spice/naif0012.tls")]) {
            return;
        }
        let mut first = SpiceInterface::new(config_without_kernels());
        let mut second = SpiceInterface::new(config_without_kernels());
        first.load_spice_kernel("naif0012.tls", &directory).unwrap();
        second
            .load_spice_kernel("naif0012.tls", &directory.join("..").join("spice"))
            .unwrap();
        assert_eq!(count_kernels_loaded(), 1);
        first.unload_spice_kernel("naif0012.tls", &directory);
        assert!(is_kernel_loaded(&directory.join("naif0012.tls")));
        second.unload_spice_kernel("naif0012.tls", &directory);
        assert_eq!(count_kernels_loaded(), 0);

        let missing = first.load_spice_kernel("no_such_kernel.tls", &directory);
        assert!(matches!(
            missing,
            Err(SpiceInterfaceError::KernelLoad { .. })
        ));
        assert_eq!(count_kernels_loaded(), 0);
    }

    /// Upstream `testSpiceThreadSafety` (issue #220), with threads in place
    /// of processes: CSPICE state is shared by threads, the harder case.
    #[test]
    fn interfaces_created_and_dropped_in_parallel_do_not_interfere() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let (workers, iterations) = (10, 3);
        let start = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        let successes: usize = std::thread::scope(|scope| {
            let handles: Vec<_> = (0..workers)
                .map(|_| {
                    scope.spawn(move || {
                        for _ in 0..iterations {
                            let mut spice =
                                SpiceInterface::new(config_with_planets(&["earth", "sun"]));
                            spice.init();
                            spice.reset(&context_at(start, 0));
                            std::thread::sleep(std::time::Duration::from_millis(1));
                            assert!(
                                spice.planet_state_out_msgs[0]
                                    .read()
                                    .position_inertial_m
                                    .norm()
                                    > 0.0
                            );
                        }
                        iterations
                    })
                })
                .collect();
            handles
                .into_iter()
                .map(|handle| handle.join().unwrap_or(0))
                .sum()
        });
        assert_eq!(successes, workers * iterations);
        assert_eq!(count_kernels_loaded(), 0);
    }

    // ------------------------------------------------------------------
    // test_spiceInterfaceReconstruction.py
    // ------------------------------------------------------------------

    /// Names the default kernels resolve; Earth has both a position and a
    /// constant-rate IAU_EARTH orientation.
    const RECON_PLANETS: [&str; 3] = ["earth", "mars barycenter", "sun"];

    /// Upstream `DATE`, "2021 MARCH 1 00:00:00.0 (UTC)".
    fn recon_epoch() -> Epoch {
        Epoch::from_gregorian_utc_at_midnight(2021, 3, 1)
    }

    fn seconds_grid(
        knot_step_s: f64,
        n_knots: usize,
        use_velocity: bool,
        interpolate: bool,
    ) -> ReconstructionSettings {
        ReconstructionSettings {
            knot_step_nanos: sec2nano(knot_step_s),
            n_knots,
            use_velocity,
            interpolate,
        }
    }

    /// Upstream `_runToStop`: a fresh interface configured by `configure`,
    /// run to `tf` on a `dt` task; the final planet states and the interface.
    fn run_to_stop(
        configure: impl FnOnce(&mut SpiceInterface),
        dt: f64,
        tf: f64,
    ) -> (Vec<PlanetStateMsg>, SpiceInterface) {
        let mut spice = SpiceInterface::new(config_with_planets(&RECON_PLANETS));
        configure(&mut spice);
        let outputs = spice.planet_state_out_msgs.clone();
        {
            let mut sim = Simulation::new(recon_epoch(), false);
            sim.add_module("spice", &mut spice, sec2nano(dt), 0);
            sim.run_for(sec2nano(tf));
        }
        (outputs.iter().map(Output::read).collect(), spice)
    }

    /// The interface's updates at `dt` steps from `from_s` to `to_s`
    /// inclusive, as a scheduler would run them.
    fn step(spice: &mut SpiceInterface, dt: f64, from_s: f64, to_s: f64) {
        let steps = ((to_s - from_s) / dt).round() as u64;
        for index in 0..=steps {
            spice.update(&context_at(
                recon_epoch(),
                sec2nano(from_s + index as f64 * dt),
            ));
        }
    }

    fn assert_identical(lhs: &PlanetStateMsg, rhs: &PlanetStateMsg, label: &str) {
        assert_eq!(lhs.position_inertial_m, rhs.position_inertial_m, "{label}");
        assert_eq!(
            lhs.velocity_inertial_mps, rhs.velocity_inertial_mps,
            "{label}"
        );
        assert_eq!(dcm(lhs), dcm(rhs), "{label}");
        assert_eq!(dcm_dot(lhs), dcm_dot(rhs), "{label}");
    }

    #[test]
    fn unconfigured_planets_are_bit_identical_to_the_exact_path() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let (dt, tf) = (10.0, 300.0);
        let (base, _) = run_to_stop(|_| {}, dt, tf);
        let (with_interp, _) = run_to_stop(
            |spice| {
                spice
                    .set_planet_position_reconstruction("earth", seconds_grid(60.0, 2, true, true))
                    .unwrap();
            },
            dt,
            tf,
        );
        // Mars and the Sun are untouched by Earth's reconstruction.
        for index in 1..3 {
            assert_identical(&base[index], &with_interp[index], RECON_PLANETS[index]);
        }
        // Earth keeps its orientation flag.
        assert_eq!(
            base[0].orientation.is_some(),
            with_interp[0].orientation.is_some()
        );

        let (base2, _) = run_to_stop(|_| {}, dt, tf);
        for index in 0..3 {
            assert_identical(&base[index], &base2[index], RECON_PLANETS[index]);
        }
    }

    #[test]
    fn position_reconstruction_is_accurate_and_cheaper() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let (dt, tf) = (10.0, 600.0);
        let (exact, _) = run_to_stop(|_| {}, dt, tf);
        let (recon, spice) = run_to_stop(
            |spice| {
                spice
                    .set_planet_position_reconstruction("earth", seconds_grid(120.0, 2, true, true))
                    .unwrap();
            },
            dt,
            tf,
        );
        let error_m = (recon[0].position_inertial_m - exact[0].position_inertial_m).norm();
        assert!(
            error_m < 1.0,
            "position reconstruction error too large: {error_m} m"
        );
        let (n_position, n_orientation) = spice.planet_spice_query_count("earth");
        assert!(
            n_position <= 12,
            "expected ~horizon/knot position queries, got {n_position}"
        );
        assert!(
            n_orientation > n_position,
            "exact orientation channel should query every step, got {n_orientation}"
        );
    }

    #[test]
    fn single_knot_orientation_extrapolation_is_exact_for_a_constant_spinner() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let (dt, tf) = (10.0, 600.0);
        let (exact, _) = run_to_stop(|_| {}, dt, tf);
        let (recon, spice) = run_to_stop(
            |spice| {
                spice
                    .set_planet_orientation_reconstruction(
                        "earth",
                        seconds_grid(100_000.0, 1, true, false),
                    )
                    .unwrap();
            },
            dt,
            tf,
        );
        let dcm_error = (dcm(&recon[0]) - dcm(&exact[0])).abs().max();
        assert!(dcm_error < 1.0e-9, "max|dDCM| = {dcm_error}");
        let (n_position, n_orientation) = spice.planet_spice_query_count("earth");
        assert_eq!(n_orientation, 1);
        assert!(n_position > n_orientation);
    }

    #[test]
    fn disabled_channels_emit_the_trivial_default() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let (recon, spice) = run_to_stop(
            |spice| {
                spice.set_planet_position_disabled("earth");
                spice.set_planet_orientation_disabled("earth");
            },
            10.0,
            100.0,
        );
        assert_eq!(recon[0].position_inertial_m, Vector3::zeros());
        assert_eq!(recon[0].velocity_inertial_mps, Vector3::zeros());
        assert!(recon[0].orientation.is_some(), "Earth still has a frame");
        assert_eq!(dcm(&recon[0]), Matrix3::identity());
        assert_eq!(dcm_dot(&recon[0]), Matrix3::zeros());
        assert_eq!(spice.planet_spice_query_count("earth"), (0, 0));
        assert!(spice.j2000_current() > 0.0);
    }

    #[test]
    fn velocity_is_the_derivative_of_the_reconstructed_position() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let dt = 1.0;
        let mut spice = SpiceInterface::new(config_with_planets(&RECON_PLANETS));
        spice
            .set_planet_position_reconstruction("earth", seconds_grid(90.0, 2, true, true))
            .unwrap();
        let earth = spice.planet_state_out_msgs[0].clone();
        let mut samples = Vec::new();
        {
            let mut sim = Simulation::new(recon_epoch(), false);
            sim.add_module("spice", &mut spice, sec2nano(dt), 0);
            sim.run_for(0);
            samples.push(earth.read());
            while sim.current_sim_nanos() < sec2nano(300.0) {
                sim.run_for(sec2nano(dt));
                samples.push(earth.read());
            }
        }
        let max_relative_error = samples
            .windows(3)
            .map(|window| {
                let finite_difference =
                    (window[2].position_inertial_m - window[0].position_inertial_m) / (2.0 * dt);
                let emitted = window[1].velocity_inertial_mps;
                (finite_difference - emitted).norm() / emitted.norm().max(1.0e-9)
            })
            .fold(0.0, f64::max);
        assert!(
            max_relative_error < 1.0e-4,
            "emitted velocity inconsistent with dP/dt: max relErr = {max_relative_error}"
        );
    }

    /// Upstream `test_integratorAgnosticQueryCountOnDynamicsTask` runs the
    /// interface on an MJScene dynamics task (MuJoCo, behind a feature not
    /// built here) under RK4 and RKF45. Here the interface is updated
    /// directly at the stage times of 20 s RK4 and RKF45 steps: the knot
    /// cache serves every stage, and both need the same queries.
    #[test]
    fn query_count_does_not_depend_on_the_integrator_stages() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let (dt, tf) = (20.0, 1200.0);
        let query_count = |stage_fractions: &[f64]| {
            let mut spice = SpiceInterface::new(config_with_planets(&RECON_PLANETS));
            spice
                .set_planet_position_reconstruction("earth", seconds_grid(120.0, 2, true, true))
                .unwrap();
            spice.init();
            for step in 0..(tf / dt) as u64 {
                for fraction in stage_fractions {
                    let t = sec2nano((step as f64 + fraction) * dt);
                    spice.update(&context_at(recon_epoch(), t));
                }
            }
            spice.planet_spice_query_count("earth").0
        };
        let rk4 = query_count(&[0.0, 0.5, 0.5, 1.0]);
        let rkf45 = query_count(&[0.0, 0.25, 0.375, 12.0 / 13.0, 1.0, 0.5]);
        assert!(
            rk4 <= 15,
            "expected ~horizon/knot queries on dynamics task, got {rk4}"
        );
        assert_eq!(rk4, rkf45, "query count must be integrator-agnostic");
    }

    #[test]
    fn exact_spice_restores_the_default_path() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let (dt, tf) = (10.0, 200.0);
        let (exact, _) = run_to_stop(|_| {}, dt, tf);
        let (reverted, spice) = run_to_stop(
            |spice| {
                spice
                    .set_planet_position_reconstruction("earth", seconds_grid(60.0, 2, true, true))
                    .unwrap();
                spice
                    .set_planet_orientation_reconstruction(
                        "earth",
                        seconds_grid(60.0, 2, true, true),
                    )
                    .unwrap();
                spice.set_planet_exact_spice("earth");
            },
            dt,
            tf,
        );
        assert_eq!(
            exact[0].position_inertial_m,
            reverted[0].position_inertial_m
        );
        assert_eq!(dcm(&exact[0]), dcm(&reverted[0]));
        let (n_position, n_orientation) = spice.planet_spice_query_count("earth");
        assert!(
            n_position == n_orientation && n_position as f64 > tf / dt,
            "exact-reverted channels should query every step on both channels, got \
             {n_position}, {n_orientation}"
        );
    }

    #[test]
    fn a_single_knot_is_exact_whatever_the_interpolate_flag() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let (dt, tf) = (10.0, 600.0);
        let (exact, _) = run_to_stop(|_| {}, dt, tf);
        for interpolate in [true, false] {
            let (recon, spice) = run_to_stop(
                |spice| {
                    spice
                        .set_planet_orientation_reconstruction(
                            "earth",
                            seconds_grid(100_000.0, 1, true, interpolate),
                        )
                        .unwrap();
                },
                dt,
                tf,
            );
            let dcm_error = (dcm(&recon[0]) - dcm(&exact[0])).abs().max();
            assert!(
                dcm_error < 1.0e-9,
                "interpolate={interpolate}: max|dDCM| = {dcm_error}"
            );
            assert_eq!(
                spice.planet_spice_query_count("earth").1,
                1,
                "interpolate={interpolate}"
            );
        }
    }

    #[test]
    fn an_odd_knot_stencil_is_centred() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let (dt, tf) = (10.0, 600.0);
        let (exact, _) = run_to_stop(|_| {}, dt, tf);
        let error_with = |n_knots| {
            let (recon, _) = run_to_stop(
                |spice| {
                    spice
                        .set_planet_position_reconstruction(
                            "earth",
                            seconds_grid(150.0, n_knots, true, true),
                        )
                        .unwrap();
                },
                dt,
                tf,
            );
            (recon[0].position_inertial_m - exact[0].position_inertial_m).norm()
        };
        let (error2, error3) = (error_with(2), error_with(3));
        assert!(
            error3 <= (error2 * 2.0).max(1.0e-3),
            "3-knot stencil not centred: err2={error2} err3={error3}"
        );
    }

    #[test]
    fn reconfiguring_the_knot_step_clears_the_stale_cache() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let (dt, tf) = (10.0, 600.0);
        let mut spice = SpiceInterface::new(config_with_planets(&RECON_PLANETS));
        spice
            .set_planet_position_reconstruction("earth", seconds_grid(300.0, 2, true, true))
            .unwrap();
        spice.init();
        step(&mut spice, dt, 0.0, tf);
        spice
            .set_planet_position_reconstruction("earth", seconds_grid(120.0, 2, true, true))
            .unwrap();
        step(&mut spice, dt, tf + dt, 2.0 * tf);

        let (exact, _) = run_to_stop(|_| {}, dt, 2.0 * tf);
        let error_m = (spice.planet_state_out_msgs[0].read().position_inertial_m
            - exact[0].position_inertial_m)
            .norm();
        assert!(
            error_m < 1.0,
            "reconfigured knotStep reused stale cache: {error_m} m"
        );
    }

    #[test]
    fn reconstruction_enabled_after_reset_is_anchored_at_the_epoch() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let (dt, tf) = (10.0, 300.0);
        let mut spice = SpiceInterface::new(config_with_planets(&RECON_PLANETS));
        spice.init();
        step(&mut spice, dt, 0.0, tf);
        spice
            .set_planet_position_reconstruction("earth", seconds_grid(120.0, 2, true, true))
            .unwrap();
        step(&mut spice, dt, tf + dt, 2.0 * tf);

        let (exact, _) = run_to_stop(|_| {}, dt, 2.0 * tf);
        let error_m = (spice.planet_state_out_msgs[0].read().position_inertial_m
            - exact[0].position_inertial_m)
            .norm();
        assert!(
            error_m < 1.0,
            "reconstruction enabled after reset anchored at wrong epoch: {error_m} m"
        );
    }

    #[test]
    fn query_counts_track_the_exact_path() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let (dt, tf) = (10.0, 300.0);
        let (_, spice) = run_to_stop(|_| {}, dt, tf);
        let steps = (tf / dt).round() as i64;
        let (n_position, n_orientation) = spice.planet_spice_query_count("earth");
        assert!(
            (steps..=steps + 2).contains(&n_position),
            "exact-path position queries ~one per step: got {n_position}"
        );
        assert_eq!(n_orientation, n_position);
        let (mars_position, mars_orientation) = spice.planet_spice_query_count("mars barycenter");
        assert_eq!(mars_position, n_position);
        assert!(mars_orientation == 0 || mars_orientation == n_position);
        assert_eq!(spice.planet_spice_query_count("jupiter"), (-1, -1));
    }

    #[test]
    fn a_misconfigured_planet_warns_instead_of_failing() {
        let _serial = serial();
        if !default_kernels_present() {
            return;
        }
        let (_, spice) = run_to_stop(
            |spice| {
                spice
                    .set_planet_position_reconstruction(
                        "jupiter",
                        seconds_grid(60.0, 2, true, true),
                    )
                    .unwrap();
            },
            10.0,
            100.0,
        );
        assert_eq!(spice.planet_spice_query_count("jupiter"), (-1, -1));
        run_to_stop(|spice| spice.set_planet_exact_spice("venus"), 10.0, 100.0);
    }

    #[test]
    fn a_zero_knot_step_is_rejected() {
        let mut spice = SpiceInterface::new(config_without_kernels());
        let zero = ReconstructionSettings {
            knot_step_nanos: 0,
            ..ReconstructionSettings::default()
        };
        assert_eq!(
            spice.set_planet_position_reconstruction("earth", zero),
            Err(SpiceInterfaceError::ZeroKnotStep("earth".to_string()))
        );
        assert_eq!(
            spice.set_planet_orientation_reconstruction("earth", zero),
            Err(SpiceInterfaceError::ZeroKnotStep("earth".to_string()))
        );
    }

    // ------------------------------------------------------------------
    // basilisk-rs: SPICE against ANISE
    // ------------------------------------------------------------------

    /// On the same kernels (DE440s, and the Earth high-precision BPC for
    /// ITRF93), the CSPICE interface and the ANISE models publish the same
    /// Earth, Moon and Sun states and the same Earth orientation.
    #[test]
    fn spice_and_anise_agree_on_the_same_kernels() {
        use anise::constants::frames::{EARTH_ITRF93, EARTH_J2000, MOON_J2000, SSB_J2000};

        let _serial = serial();
        let de440s = PathBuf::from("assets/anise/de440s.bsp");
        let earth_bpc = PathBuf::from("assets/anise/earth_latest_high_prec.bpc");
        // pck00010 gives the Moon and the Sun their IAU frames, which the
        // ANISE models here do not compute.
        let kernels = vec![
            PathBuf::from("assets/spice/naif0012.tls"),
            PathBuf::from("assets/spice/pck00010.tpc"),
            de440s.clone(),
            earth_bpc.clone(),
        ];
        if !kernels_present(&kernels) {
            return;
        }

        let mut worst = [0.0f64; 6];
        for start in [
            Epoch::from_gregorian_utc_at_midnight(2015, 2, 10),
            Epoch::from_gregorian_utc(2020, 7, 4, 12, 30, 15, 0),
            Epoch::from_gregorian_utc(2025, 11, 20, 6, 0, 0, 0),
        ] {
            let mut barycentric = SpiceInterface::new(SpiceInterfaceConfig {
                kernel_paths: kernels.clone(),
                planet_names: vec!["earth".to_string()],
                planet_frames: vec!["ITRF93".to_string()],
                ..SpiceInterfaceConfig::default()
            });
            let mut geocentric = SpiceInterface::new(SpiceInterfaceConfig {
                kernel_paths: kernels.clone(),
                zero_base: "EARTH".to_string(),
                planet_names: vec!["moon".to_string(), "sun".to_string()],
                ..SpiceInterfaceConfig::default()
            });
            let anise = |name: &str, source, observer, fixed_frame| {
                AnisePlanetEphemeris::new(AnisePlanetEphemerisConfig {
                    name: name.to_string(),
                    spk_path: de440s.clone(),
                    additional_kernel_paths: vec![earth_bpc.clone()],
                    source_frame: source,
                    observer_frame: observer,
                    fixed_frame,
                })
            };
            let mut anise_earth = anise("earth", EARTH_J2000, SSB_J2000, Some(EARTH_ITRF93));
            let mut anise_moon = anise("moon", MOON_J2000, EARTH_J2000, None);
            let mut anise_sun = AniseSunEphemeris::new(AniseSunEphemerisConfig {
                name: "sun".to_string(),
                spk_path: de440s.clone(),
            });

            barycentric.init();
            geocentric.init();
            anise_earth.init();
            anise_moon.init();
            anise_sun.init();
            for hour in 0..=24u64 {
                let t = hour * 3600 * 1_000_000_000;
                barycentric.update(&context_at(start, t));
                geocentric.update(&context_at(start, t));
                // ANISE at the ephemeris time the interface used: upstream
                // advances ET by the simulation time, while the context
                // epoch advances in TAI.
                let anise_context = SimulationContext {
                    current_sim_nanos: t,
                    current_epoch: Epoch::from_et_seconds(barycentric.j2000_current()),
                };
                anise_earth.update(&anise_context);
                anise_moon.update(&anise_context);
                anise_sun.update(&anise_context);

                for (spice, anise) in [
                    (
                        &barycentric.planet_state_out_msgs[0],
                        &anise_earth.output_planet_msg,
                    ),
                    (
                        &geocentric.planet_state_out_msgs[0],
                        &anise_moon.output_planet_msg,
                    ),
                    (
                        &geocentric.planet_state_out_msgs[1],
                        &anise_sun.output_planet_msg,
                    ),
                ] {
                    let (spice, anise) = (spice.read(), anise.read());
                    worst[0] = worst[0]
                        .max((spice.position_inertial_m - anise.position_inertial_m).norm());
                    worst[1] = worst[1]
                        .max((spice.velocity_inertial_mps - anise.velocity_inertial_mps).norm());
                    if anise.orientation.is_some() {
                        assert!(spice.orientation.is_some());
                        worst[2] = worst[2].max((dcm(&spice) - dcm(&anise)).abs().max());
                        worst[3] = worst[3].max((dcm_dot(&spice) - dcm_dot(&anise)).abs().max());
                    }
                }
                let (spice_sun, anise_sun) = (
                    geocentric.sun_out_msg.read(),
                    anise_sun.output_sun_msg.read(),
                );
                worst[4] = worst[4].max(
                    (spice_sun.sun_position_inertial_m - anise_sun.sun_position_inertial_m).norm(),
                );
                worst[5] = worst[5].max(
                    (spice_sun.sun_velocity_inertial_mps - anise_sun.sun_velocity_inertial_mps)
                        .norm(),
                );
            }
        }
        let [
            position_m,
            velocity_mps,
            dcm_error,
            dcm_dot_error,
            sun_position_m,
            sun_velocity_mps,
        ] = worst;
        assert!(position_m < 1.0e-6, "position {position_m} m");
        assert!(velocity_mps < 1.0e-9, "velocity {velocity_mps} m/s");
        assert!(dcm_error < 1.0e-12, "DCM {dcm_error}");
        assert!(dcm_dot_error < 1.0e-16, "DCM rate {dcm_dot_error} 1/s");
        assert!(sun_position_m < 1.0e-6, "Sun position {sun_position_m} m");
        assert!(
            sun_velocity_mps < 1.0e-9,
            "Sun velocity {sun_velocity_mps} m/s"
        );
    }

    #[test]
    fn the_epoch_is_formatted_as_upstream_formats_the_epoch_message() {
        assert_eq!(
            epoch_message_string(Epoch::from_gregorian_utc(
                2015,
                2,
                10,
                0,
                0,
                59,
                999_999_000
            )),
            "2015/02/10, 00:00:59.999999 (UTC)"
        );
        assert_eq!(
            epoch_message_string(Epoch::from_gregorian_utc(2019, 1, 1, 0, 0, 5, 500_000_000)),
            "2019/01/01, 00:00:5.500000 (UTC)"
        );
        // Sub-microsecond parts round, carrying into the minute.
        assert_eq!(
            epoch_message_string(Epoch::from_gregorian_utc(2019, 1, 1, 0, 0, 59, 999_999_700)),
            "2019/01/01, 00:01:0.000000 (UTC)"
        );
    }
}
