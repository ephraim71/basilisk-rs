//! Data file to Vizard: port of upstream Basilisk
//! `simulation/vizard/dataFileToViz`.
//!
//! Replays spacecraft states from a delimited text file, one line per
//! update, as the messages [`VizInterface`](super::viz_interface::VizInterface)
//! reads: a [`SpacecraftStateMsg`] per spacecraft, a [`ThrusterOutputMsg`]
//! per thruster and a [`ReactionWheelStateMsg`] per reaction wheel. Each
//! line holds the time (read and ignored), then for each spacecraft in the
//! order they were added:
//!
//! 1. the inertial position and velocity, in the file's units, scaled by
//!    `convert_pos_to_meters` (km to m by default);
//! 2. the attitude, as an MRP set, Euler parameters `(q0, q1, q2, q3)` or
//!    3-2-1 Euler angles \[rad\] ([`AttitudeType`]);
//! 3. the body rate \[rad/s\];
//! 4. the thrust \[N\] of each of its thrusters, cluster by cluster;
//! 5. the speed \[rad/s\] and motor torque \[N m\] of each of its wheels.
//!
//! A thruster's position, direction and maximum thrust, and a wheel's
//! position, spin axis and limits, are configuration given when the device
//! is added.
//!
//! # Differences from upstream
//!
//! * Upstream keeps the thruster and wheel properties in flat lists across
//!   all spacecraft but indexes them with a counter that restarts at each
//!   spacecraft, so every spacecraft after the first reuses the first
//!   spacecraft's thruster and wheel properties. Here each device carries
//!   its own.
//! * A wheel is published as the two ports a reaction wheel effector has,
//!   its state and its configuration ([`ReactionWheelStateEffectorConfig`],
//!   holding the position, spin axis and limits), where upstream writes one
//!   `RWConfigLogMsg`.
//! * A line with a missing or non-numeric value is logged and none of it is
//!   published; upstream's `stod` throws partway through the line. The end
//!   of the file is logged once rather than at every later update.

use std::fs::File;
use std::io::{BufRead, BufReader, Lines};
use std::path::PathBuf;

use nalgebra::{Vector3, Vector4};

use super::viz_structures::ThrClusterMap;
use crate::dynamics::reaction_wheel_state_effector::ReactionWheelStateEffectorConfig;
use crate::messages::{Output, ReactionWheelStateMsg, SpacecraftStateMsg, ThrusterOutputMsg};
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::{ep_to_mrp, euler321_to_mrp};
use crate::{Module, SimulationContext};

/// How the file gives each attitude: upstream `attitudeType`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum AttitudeType {
    /// Modified Rodrigues parameters; upstream `0`, the default.
    #[default]
    Mrp = 0,
    /// Euler parameters `(q0, q1, q2, q3)`, scalar first; upstream `1`.
    Quaternion = 1,
    /// 3-2-1 Euler angles \[rad\]; upstream `2`.
    Euler321 = 2,
}

/// User-settable parameters of [`DataFileToViz`]. `Default` is upstream's
/// constructor state.
#[derive(Clone, Debug, PartialEq)]
pub struct DataFileToVizConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `dataFileName`: the file to replay; must be set.
    pub data_file_name: PathBuf,
    /// Upstream `delimiter`: the separator between values (upstream uses
    /// the first character of its string); a space by default.
    pub delimiter: char,
    /// Upstream `convertPosToMeters` [-]: factor from the file's position
    /// and velocity units to meters; 1000 (km) by default.
    pub convert_pos_to_meters: f64,
    /// Upstream `headerLine`: the first line is a header, skipped.
    pub header_line: bool,
    /// Upstream `attitudeType`.
    pub attitude_type: AttitudeType,
}

impl Default for DataFileToVizConfig {
    fn default() -> Self {
        Self {
            name: "dataFileToViz".to_string(),
            data_file_name: PathBuf::new(),
            delimiter: ' ',
            convert_pos_to_meters: 1000.0,
            header_line: true,
            attitude_type: AttitudeType::Mrp,
        }
    }
}

/// One thruster of a replayed spacecraft.
#[derive(Clone, Debug, Default)]
pub struct DataFileThruster {
    /// Upstream `thrPosList` entry \[m\]: position, body components.
    pub position_b_m: Vector3<f64>,
    /// Upstream `thrDirList` entry [-]: unit thrust direction, body
    /// components.
    pub direction_b: Vector3<f64>,
    /// Upstream `thrForceMaxList` entry \[N\]: maximum thrust.
    pub max_thrust_n: f64,
    /// Upstream `thrScOutMsgs[sc][i]`.
    pub thruster_out_msg: Output<ThrusterOutputMsg>,
}

/// A named group of thrusters: an entry of upstream `thrMsgDataSC[sc]` with
/// its `numThrPerCluster` thrusters.
#[derive(Clone, Debug, Default)]
pub struct DataFileThrusterCluster {
    /// The group's name and display options.
    pub thr_info: ThrClusterMap,
    /// The group's thrusters, in file order.
    pub thrusters: Vec<DataFileThruster>,
}

impl DataFileThrusterCluster {
    /// Upstream `appendThrPos`, `appendThrDir` and `appendThrForceMax` for
    /// one thruster: adds it and returns its output. The direction is
    /// normalized, as upstream does.
    pub fn add_thruster(
        &mut self,
        position_b_m: Vector3<f64>,
        direction_b: Vector3<f64>,
        max_thrust_n: f64,
    ) -> &Output<ThrusterOutputMsg> {
        self.thrusters.push(DataFileThruster {
            position_b_m,
            direction_b: normalize_or_zero(direction_b),
            max_thrust_n,
            thruster_out_msg: Output::default(),
        });
        &self.thrusters.last().expect("just pushed").thruster_out_msg
    }
}

/// One reaction wheel of a replayed spacecraft.
#[derive(Clone, Debug, Default)]
pub struct DataFileReactionWheel {
    /// Upstream `rwScOutMsgs[sc][i]`'s speed `Omega` and torque
    /// `u_current`.
    pub rw_state_out_msg: Output<ReactionWheelStateMsg>,
    /// Upstream `rwScOutMsgs[sc][i]`'s position `rWB_B`, spin axis
    /// `gsHat_B` and limits `Omega_max` (`max_speed_radps`) and `u_max`
    /// (`max_torque_nm`); the other fields are zero.
    pub rw_config_out_msg: Output<ReactionWheelStateEffectorConfig>,
}

/// One replayed spacecraft: an entry of upstream `scStateOutMsgs` with its
/// thrusters and wheels.
#[derive(Clone, Debug, Default)]
pub struct DataFileSpacecraft {
    /// Upstream `scStateOutMsgs[sc]`: position, velocity, attitude and rate.
    pub sc_state_out_msg: Output<SpacecraftStateMsg>,
    /// Upstream `thrMsgDataSC[sc]` with `thrScOutMsgs[sc]`.
    pub thruster_clusters: Vec<DataFileThrusterCluster>,
    /// Upstream `rwScOutMsgs[sc]`.
    pub reaction_wheels: Vec<DataFileReactionWheel>,
}

impl DataFileSpacecraft {
    /// One cluster of upstream `appendThrClusterMap`: adds a thruster group,
    /// for [`DataFileThrusterCluster::add_thruster`].
    pub fn add_thruster_cluster(
        &mut self,
        thr_info: ThrClusterMap,
    ) -> &mut DataFileThrusterCluster {
        self.thruster_clusters.push(DataFileThrusterCluster {
            thr_info,
            thrusters: Vec::new(),
        });
        self.thruster_clusters.last_mut().expect("just pushed")
    }

    /// One wheel of upstream `appendNumOfRWs`, with `appendRwPos`,
    /// `appendRwDir`, `appendOmegaMax` and `appendUMax`: adds a wheel at
    /// `position_b_m` spinning about `spin_axis_b` (normalized, as upstream
    /// does) and returns its outputs.
    pub fn add_reaction_wheel(
        &mut self,
        position_b_m: Vector3<f64>,
        spin_axis_b: Vector3<f64>,
        omega_max_radps: f64,
        u_max_nm: f64,
    ) -> &DataFileReactionWheel {
        let config = ReactionWheelStateEffectorConfig {
            position_m: position_b_m,
            spin_axis_body: normalize_or_zero(spin_axis_b),
            max_speed_radps: omega_max_radps,
            max_torque_nm: u_max_nm,
            ..ReactionWheelStateEffectorConfig::default()
        };
        self.reaction_wheels.push(DataFileReactionWheel {
            rw_state_out_msg: Output::default(),
            rw_config_out_msg: Output::new(config),
        });
        self.reaction_wheels.last().expect("just pushed")
    }

    /// The thrusters, cluster by cluster, each with its group: the order
    /// upstream's `thrInfo` lists them in for `vizInterface`.
    pub fn thrusters(&self) -> impl Iterator<Item = (&ThrClusterMap, &DataFileThruster)> {
        self.thruster_clusters.iter().flat_map(|cluster| {
            cluster
                .thrusters
                .iter()
                .map(move |thruster| (&cluster.thr_info, thruster))
        })
    }
}

/// Replays spacecraft states from a text file: upstream `DataFileToViz`.
#[derive(Debug)]
pub struct DataFileToViz {
    /// User-settable parameters.
    pub config: DataFileToVizConfig,
    /// The replayed spacecraft; see [`Self::add_spacecraft`].
    pub spacecraft: Vec<DataFileSpacecraft>,
    /// Upstream `fileHandle`: the remaining lines of the open file.
    lines: Option<Lines<BufReader<File>>>,
    /// The end of the file has been logged.
    end_reported: bool,
}

/// The values of one line, read in order (upstream `pullScalar`).
struct LineValues<'a> {
    tokens: std::str::Split<'a, char>,
}

impl LineValues<'_> {
    fn scalar(&mut self) -> Result<f64, String> {
        let token = self.tokens.next().ok_or("the line ends early")?;
        token
            .trim()
            .parse()
            .map_err(|_| format!("`{token}` is not a number"))
    }

    fn vector3(&mut self) -> Result<Vector3<f64>, String> {
        Ok(Vector3::new(self.scalar()?, self.scalar()?, self.scalar()?))
    }

    fn vector4(&mut self) -> Result<Vector4<f64>, String> {
        Ok(Vector4::new(
            self.scalar()?,
            self.scalar()?,
            self.scalar()?,
            self.scalar()?,
        ))
    }
}

/// The messages one line gives one spacecraft, in device order.
struct SpacecraftLine {
    state: SpacecraftStateMsg,
    thrust_n: Vec<f64>,
    wheels: Vec<ReactionWheelStateMsg>,
}

impl DataFileToViz {
    /// Builds the module with no spacecraft.
    pub fn new(config: DataFileToVizConfig) -> Self {
        Self {
            config,
            spacecraft: Vec::new(),
            lines: None,
            end_reported: false,
        }
    }

    /// One spacecraft of upstream `setNumOfSatellites`: adds a spacecraft
    /// and returns it, to add its devices.
    pub fn add_spacecraft(&mut self) -> &mut DataFileSpacecraft {
        self.spacecraft.push(DataFileSpacecraft::default());
        self.spacecraft.last_mut().expect("just pushed")
    }

    /// Upstream `Reset`: (re)opens the file and skips the header line.
    fn reset_state(&mut self) {
        let name = &self.config.name;
        if self.config.data_file_name.as_os_str().is_empty() {
            log::error!("{name}: dataFileName must be an non-empty string.");
        }
        if self.spacecraft.is_empty() {
            log::error!("{name}: spacecraft list must have at least one element in it.");
        }

        self.lines = None;
        self.end_reported = false;
        let path = &self.config.data_file_name;
        match File::open(path) {
            Ok(file) => {
                let mut lines = BufReader::new(file).lines();
                if self.config.header_line {
                    lines.next();
                }
                self.lines = Some(lines);
                log::info!("{name}: loaded the file: {}.", path.display());
            }
            Err(error) => log::error!(
                "{name}: was not able to load the file {}: {error}",
                path.display()
            ),
        }
    }

    /// Reads one line into per-spacecraft messages (upstream `UpdateState`'s
    /// parsing).
    fn parse_line(&self, line: &str) -> Result<Vec<SpacecraftLine>, String> {
        let mut values = LineValues {
            tokens: line.split(self.config.delimiter),
        };
        // The time column is not used.
        values.scalar()?;
        let scale = self.config.convert_pos_to_meters;
        self.spacecraft
            .iter()
            .map(|spacecraft| {
                let position_m = values.vector3()? * scale;
                let velocity_mps = values.vector3()? * scale;
                let sigma_bn = match self.config.attitude_type {
                    AttitudeType::Mrp => values.vector3()?,
                    AttitudeType::Quaternion => ep_to_mrp(values.vector4()?),
                    AttitudeType::Euler321 => euler321_to_mrp(values.vector3()?),
                };
                let omega_radps = values.vector3()?;
                let thrust_n = spacecraft
                    .thrusters()
                    .map(|_| values.scalar())
                    .collect::<Result<_, _>>()?;
                let wheels = spacecraft
                    .reaction_wheels
                    .iter()
                    .map(|_| {
                        Ok(ReactionWheelStateMsg {
                            omega_radps: values.scalar()?,
                            applied_motor_torque_nm: values.scalar()?,
                            ..ReactionWheelStateMsg::default()
                        })
                    })
                    .collect::<Result<_, String>>()?;
                Ok(SpacecraftLine {
                    state: SpacecraftStateMsg {
                        position_m,
                        velocity_mps,
                        sigma_bn,
                        omega_radps,
                        ..SpacecraftStateMsg::default()
                    },
                    thrust_n,
                    wheels,
                })
            })
            .collect()
    }

    /// Upstream `UpdateState`'s message writes.
    fn publish(&self, lines: Vec<SpacecraftLine>) {
        for (spacecraft, line) in self.spacecraft.iter().zip(lines) {
            spacecraft.sc_state_out_msg.write(line.state);
            for ((_, thruster), thrust_n) in spacecraft.thrusters().zip(line.thrust_n) {
                thruster.thruster_out_msg.write(ThrusterOutputMsg {
                    max_thrust_n: thruster.max_thrust_n,
                    thrust_force_n: thrust_n,
                    thruster_location_m: thruster.position_b_m,
                    thruster_direction: thruster.direction_b,
                    ..ThrusterOutputMsg::default()
                });
            }
            for (wheel, state) in spacecraft.reaction_wheels.iter().zip(line.wheels) {
                wheel.rw_state_out_msg.write(state);
            }
        }
    }
}

impl Module for DataFileToViz {
    fn init(&mut self) {
        self.reset_state();
        for spacecraft in &self.spacecraft {
            spacecraft
                .sc_state_out_msg
                .write(SpacecraftStateMsg::default());
            for (_, thruster) in spacecraft.thrusters() {
                thruster
                    .thruster_out_msg
                    .write(ThrusterOutputMsg::default());
            }
            for wheel in &spacecraft.reaction_wheels {
                wheel
                    .rw_state_out_msg
                    .write(ReactionWheelStateMsg::default());
            }
        }
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    /// Upstream `UpdateState`: publishes the next line of the file.
    fn update(&mut self, _context: &SimulationContext) {
        let Some(lines) = &mut self.lines else {
            return;
        };
        let line = match lines.next() {
            Some(Ok(line)) => line,
            Some(Err(error)) => {
                log::error!("{}: could not read the file: {error}", self.config.name);
                return;
            }
            None => {
                if !self.end_reported {
                    log::info!("{}: reached end of file.", self.config.name);
                    self.end_reported = true;
                }
                return;
            }
        };
        match self.parse_line(&line) {
            Ok(lines) => self.publish(lines),
            Err(problem) => log::error!(
                "{}: skipping the line `{line}`: {problem}",
                self.config.name
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `_UnitTest/test_dataFileToViz.py`, with the module's
    //! outputs also run through [`VizInterface`] into a file and read back
    //! (upstream wires them to `vizInterface` without saving).
    //!
    //! Upstream's `test_data_file_to_viz_viz_interface_import_order` checks
    //! that the SWIG wrappers of the two modules share one `ThrClusterMap`
    //! type. Both modules here use the one Rust type, which
    //! `replays_spacecraft_states_through_viz_interface` passes from one to
    //! the other.

    use std::f64::consts::PI;
    use std::path::{Path, PathBuf};
    use std::time::{SystemTime, UNIX_EPOCH};

    use hifitime::Epoch;
    use nalgebra::{Vector3, Vector4};

    use super::{AttitudeType, DataFileToViz, DataFileToVizConfig};
    use crate::messages::{ReactionWheelStateMsg, SpacecraftStateMsg, ThrusterOutputMsg};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::astro_constants::RPM;
    use crate::vizard::viz_interface::{VizInterface, VizInterfaceConfig, read_viz_file};
    use crate::vizard::viz_structures::{GravBodyInfo, ThrClusterMap};
    use crate::{Module, SimulationContext};

    const R_B1N: [f64; 3] = [6761.48, 1569.01, 905.867];
    const V_B1N: [f64; 3] = [-1.95306, 6.3124, 3.64446];
    const BETA_B1N: [f64; 4] = [0.182574, 0.365148, 0.547723, 0.730297];
    const SIGMA_B1N: [f64; 3] = [0.1, 0.2, 0.3];
    const OMEGA: [f64; 3] = [0.0, 0.0, 0.0];
    const R_B2N: [f64; 3] = [6761.48, 1569.02, 905.874];
    const V_B2N: [f64; 3] = [-1.95308, 6.31239, 3.64446];
    const BETA_B2N: [f64; 4] = [-0.182574, 0.365148, 0.547723, 0.730297];
    const SIGMA_B2N: [f64; 3] = [-0.1, 0.1, 0.3];

    const TH1_ACS: f64 = 1.0;
    const TH1_DV: f64 = 100.0;
    const TH2_ACS: f64 = 0.001;
    const TH2_DV: f64 = 200.0;
    const U1_SC1: f64 = 0.1;
    const U2_SC1: f64 = -0.1;
    const U1_SC2: f64 = 0.3;

    const RED: [u8; 4] = [255, 0, 0, 255];
    const BLUE: [u8; 4] = [0, 0, 255, 255];
    const YELLOW: [u8; 4] = [255, 255, 0, 255];

    fn omega1_sc1() -> f64 {
        100.0 * RPM
    }
    fn omega2_sc1() -> f64 {
        500.0 * RPM
    }
    fn omega1_sc2() -> f64 {
        1000.0 * RPM
    }

    fn unique_path(label: &str, extension: &str) -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system clock after Unix epoch")
            .as_nanos();
        std::env::temp_dir().join(format!(
            "basilisk_rs_data_file_to_viz_{label}_{}_{nonce}.{extension}",
            std::process::id()
        ))
    }

    /// Python's `str(list)[1:-1]` of the test's values.
    fn list(values: &[f64]) -> String {
        values
            .iter()
            .map(|value| format!("{value:?}"))
            .collect::<Vec<_>>()
            .join(", ")
    }

    /// The test's data file, line for line as upstream writes it.
    fn write_data_file(path: &Path, quaternions: bool, thrusters: bool, wheels: bool) {
        let d = ",";
        let (att1, att2) = if quaternions {
            (list(&BETA_B1N), list(&BETA_B2N))
        } else {
            (list(&SIGMA_B1N), list(&SIGMA_B2N))
        };
        let dt_s = 0.1;
        let sim_time_s = 2.0;
        let mut text = String::new();
        for i in 0..((sim_time_s / dt_s) as usize + 2) {
            let t = (i as f64 * dt_s * 1e4).round() / 1e4;
            let mut line = format!(
                "{t:?}{d}{}{d}{}{d}{att1}{d}{}{d}",
                list(&R_B1N),
                list(&V_B1N),
                list(&OMEGA)
            );
            if thrusters {
                line += &format!("{TH1_ACS:?}{d}{TH1_DV:?}{d}");
            }
            if wheels {
                line += &format!(
                    "{:?}{d}{U1_SC1:?}{d}{:?}{d}{U2_SC1:?}{d}",
                    omega1_sc1(),
                    omega2_sc1()
                );
            }
            line += &format!(
                "{}{d}{}{d}{att2}{d}{}",
                list(&R_B2N),
                list(&V_B2N),
                list(&OMEGA)
            );
            if thrusters {
                line += &format!("{d}{TH2_ACS:?}{d}{TH2_DV:?}{d}{TH2_DV:?}");
            }
            if wheels {
                line += &format!("{d}{:?}{d}{U1_SC2:?}", omega1_sc2());
            }
            text += &line;
            text.push('\n');
        }
        std::fs::write(path, text).expect("writable data file");
    }

    /// Upstream `RigidBodyKinematics.EP2MRP`.
    fn ep_to_mrp_truth(q: [f64; 4]) -> [f64; 3] {
        if q[0] >= 0.0 {
            [
                q[1] / (1.0 + q[0]),
                q[2] / (1.0 + q[0]),
                q[3] / (1.0 + q[0]),
            ]
        } else {
            [
                -q[1] / (1.0 - q[0]),
                -q[2] / (1.0 - q[0]),
                -q[3] / (1.0 - q[0]),
            ]
        }
    }

    /// Upstream `RigidBodyKinematics.euler3212MRP`, through `euler3212EP`.
    fn euler321_to_mrp_truth(angles: [f64; 3]) -> [f64; 3] {
        let (s1, c1) = (angles[0] / 2.0).sin_cos();
        let (s2, c2) = (angles[1] / 2.0).sin_cos();
        let (s3, c3) = (angles[2] / 2.0).sin_cos();
        ep_to_mrp_truth([
            c1 * c2 * c3 + s1 * s2 * s3,
            c1 * c2 * s3 - s1 * s2 * c3,
            c1 * s2 * c3 + s1 * c2 * s3,
            s1 * c2 * c3 - c1 * s2 * s3,
        ])
    }

    /// Upstream `unitTestSupport.isVectorEqual`.
    fn is_vector_equal(result: &Vector3<f64>, truth: [f64; 3], accuracy: f64) -> bool {
        result
            .iter()
            .zip(truth)
            .all(|(r, t)| (r - t).abs() <= accuracy)
    }

    /// Upstream `unitTestSupport.isDoubleEqualRelative`.
    fn is_double_equal_relative(result: f64, truth: f64, accuracy: f64) -> bool {
        (result - truth).abs() / truth.abs() <= accuracy
    }

    /// Upstream `run`: `convert_pos_units` -1 keeps the default factor,
    /// `att_type` -1 the default attitude type.
    fn run(convert_pos_units: f64, att_type: i32, check_thruster: bool, check_rw: bool) {
        let case = format!(
            "convert {convert_pos_units}, att {att_type}, thr {check_thruster}, rw {check_rw}"
        );
        let data_file = unique_path("data", "txt");
        let viz_file = unique_path("viz", "bin");
        write_data_file(&data_file, att_type == 1, check_thruster, check_rw);

        let mut config = DataFileToVizConfig {
            name: "testModule".to_string(),
            data_file_name: data_file.clone(),
            delimiter: ',',
            ..DataFileToVizConfig::default()
        };
        let convert_pos_units = if convert_pos_units > 0.0 {
            config.convert_pos_to_meters = convert_pos_units;
            convert_pos_units
        } else {
            1000.0
        };
        match att_type {
            0 => config.attitude_type = AttitudeType::Mrp,
            1 => config.attitude_type = AttitudeType::Quaternion,
            2 => config.attitude_type = AttitudeType::Euler321,
            _ => {}
        }
        let mut module = DataFileToViz::new(config);
        module.add_spacecraft();
        module.add_spacecraft();
        let thr_num_list = [vec![1, 1], vec![1, 2]];

        if check_thruster {
            let sc1 = &mut module.spacecraft[0];
            sc1.add_thruster_cluster(ThrClusterMap {
                thr_tag: "adcs".to_string(),
                color: Some(RED),
                ..ThrClusterMap::default()
            })
            .add_thruster(
                Vector3::new(0.0, 0.0, 3.0),
                Vector3::new(0.0, 0.0, -1.0),
                TH1_ACS,
            );
            sc1.add_thruster_cluster(ThrClusterMap {
                thr_tag: "dv".to_string(),
                color: Some(BLUE),
                ..ThrClusterMap::default()
            })
            .add_thruster(
                Vector3::new(0.0, 0.0, -3.0),
                Vector3::new(0.0, 0.0, 1.0),
                TH1_DV,
            );

            let sc2 = &mut module.spacecraft[1];
            sc2.add_thruster_cluster(ThrClusterMap {
                thr_tag: "adcs".to_string(),
                ..ThrClusterMap::default()
            })
            .add_thruster(
                Vector3::new(0.0, 0.0, 3.0),
                Vector3::new(0.0, 0.0, -1.0),
                TH2_ACS,
            );
            let dv2 = sc2.add_thruster_cluster(ThrClusterMap {
                thr_tag: "dv".to_string(),
                ..ThrClusterMap::default()
            });
            dv2.add_thruster(
                Vector3::new(0.0, 0.0, -3.0),
                Vector3::new(0.0, 0.0, 1.0),
                TH2_DV,
            );
            dv2.add_thruster(
                Vector3::new(0.0, 2.0, -3.0),
                Vector3::new(0.0, 0.0, 1.0),
                TH2_DV,
            );
            for (spacecraft, per_cluster) in module.spacecraft.iter().zip(&thr_num_list) {
                let counts: Vec<usize> = spacecraft
                    .thruster_clusters
                    .iter()
                    .map(|cluster| cluster.thrusters.len())
                    .collect();
                assert_eq!(&counts, per_cluster);
            }
        }
        if check_rw {
            let omega_max = 3000.0 * RPM;
            let sc1 = &mut module.spacecraft[0];
            sc1.add_reaction_wheel(Vector3::zeros(), Vector3::x(), omega_max, 0.5);
            sc1.add_reaction_wheel(Vector3::zeros(), Vector3::y(), omega_max, 0.5);
            module.spacecraft[1].add_reaction_wheel(Vector3::zeros(), Vector3::y(), omega_max, 0.5);
        }

        // The Vizard interface reads the replayed spacecraft; only the first
        // carries the gravity body.
        let mut viz = VizInterface::new(VizInterfaceConfig {
            save_file: Some(viz_file.clone()),
            ..VizInterfaceConfig::default()
        });
        viz.add_celestial_body(GravBodyInfo {
            body_name: "earth".to_string(),
            ..GravBodyInfo::default()
        });
        if check_thruster {
            viz.settings.default_thruster_color = Some(YELLOW);
        }
        let mut sc_logs: Vec<MessageRecorder<SpacecraftStateMsg>> =
            (0..2).map(|_| MessageRecorder::default()).collect();
        let mut thr_logs: Vec<MessageRecorder<ThrusterOutputMsg>> = Vec::new();
        let mut rw_logs: Vec<MessageRecorder<ReactionWheelStateMsg>> = Vec::new();
        for (index, spacecraft) in module.spacecraft.iter().enumerate() {
            let viz_sc = viz.add_spacecraft(format!("test{}", index + 1));
            spacecraft
                .sc_state_out_msg
                .connect_to(&mut viz_sc.sc_state_in_msg);
            spacecraft
                .sc_state_out_msg
                .connect_to(&mut sc_logs[index].input_msg);
            for (thr_info, thruster) in spacecraft.thrusters() {
                thruster
                    .thruster_out_msg
                    .connect_to(viz_sc.add_thruster(thr_info.clone()));
                let mut log = MessageRecorder::default();
                thruster.thruster_out_msg.connect_to(&mut log.input_msg);
                thr_logs.push(log);
            }
            for wheel in &spacecraft.reaction_wheels {
                let viz_wheel = viz_sc.add_reaction_wheel();
                wheel
                    .rw_state_out_msg
                    .connect_to(&mut viz_wheel.rw_state_in_msg);
                wheel
                    .rw_config_out_msg
                    .connect_to(&mut viz_wheel.rw_config_in_msg);
                let mut log = MessageRecorder::default();
                wheel.rw_state_out_msg.connect_to(&mut log.input_msg);
                rw_logs.push(log);
            }
        }

        let rate_nanos = 100_000_000;
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("testModule", &mut module, rate_nanos, 10);
            sim.add_module("viz", &mut viz, rate_nanos, 0);
            for log in &mut sc_logs {
                sim.add_module("scLog", log, rate_nanos, 0);
            }
            for log in &mut thr_logs {
                sim.add_module("thrLog", log, rate_nanos, 0);
            }
            for log in &mut rw_logs {
                sim.add_module("rwLog", log, rate_nanos, 0);
            }
            sim.run_for(20 * rate_nanos);
        }
        drop(viz);
        std::fs::remove_file(&data_file).expect("removable data file");

        // Upstream's checks, on the first recorded sample.
        let (att1_in, att2_in) = match att_type {
            1 => (ep_to_mrp_truth(BETA_B1N), ep_to_mrp_truth(BETA_B2N)),
            2 => (
                euler321_to_mrp_truth(SIGMA_B1N),
                euler321_to_mrp_truth(SIGMA_B2N),
            ),
            _ => (SIGMA_B1N, SIGMA_B2N),
        };
        let scaled = |r: [f64; 3]| r.map(|x| x * convert_pos_units);
        let first1 = &sc_logs[0].samples[0];
        let first2 = &sc_logs[1].samples[0];
        assert!(
            is_vector_equal(&first1.position_m, scaled(R_B1N), 0.1),
            "pos1 {case}"
        );
        assert!(
            is_vector_equal(&first2.position_m, scaled(R_B2N), 0.1),
            "pos2 {case}"
        );
        assert!(
            is_vector_equal(&first1.sigma_bn, att1_in, 0.1),
            "att1 {case}"
        );
        assert!(
            is_vector_equal(&first2.sigma_bn, att2_in, 0.1),
            "att2 {case}"
        );
        if check_thruster {
            let thrust: Vec<f64> = thr_logs
                .iter()
                .map(|log| log.samples[0].thrust_force_n)
                .collect();
            for (index, truth) in [TH1_ACS, TH1_DV, TH2_ACS, TH2_DV, TH2_DV]
                .iter()
                .enumerate()
            {
                assert!(
                    is_double_equal_relative(thrust[index], *truth, 0.001),
                    "thruster {index} {case}"
                );
            }
        }
        if check_rw {
            let first = |index: usize| &rw_logs[index].samples[0];
            for (index, (omega, u)) in [
                (omega1_sc1(), U1_SC1),
                (omega2_sc1(), U2_SC1),
                (omega1_sc2(), U1_SC2),
            ]
            .into_iter()
            .enumerate()
            {
                assert!(
                    is_double_equal_relative(first(index).omega_radps, omega, 0.001),
                    "Omega of wheel {index} {case}"
                );
                assert!(
                    is_double_equal_relative(first(index).applied_motor_torque_nm, u, 0.001),
                    "u_current of wheel {index} {case}"
                );
            }
        }

        // What Vizard was given.
        let messages = read_viz_file(&viz_file).expect("readable Vizard file");
        std::fs::remove_file(&viz_file).expect("removable Vizard file");
        assert_eq!(messages.len(), 20, "{case}");
        let last = messages.last().expect("frames");
        assert_eq!(last.spacecraft.len(), 2, "{case}");
        assert_eq!(last.celestial_bodies.len(), 1, "{case}");
        let sc1 = &last.spacecraft[0];
        let sc2 = &last.spacecraft[1];
        assert_eq!(
            (sc1.spacecraft_name.as_str(), sc2.spacecraft_name.as_str()),
            ("test1", "test2")
        );
        assert!(
            (sc1.position[0] - R_B1N[0] * convert_pos_units).abs() <= 0.1,
            "{case}"
        );
        if check_thruster {
            let tags: Vec<&str> = sc2
                .thrusters
                .iter()
                .map(|t| t.thruster_tag.as_str())
                .collect();
            assert_eq!(tags, vec!["adcs", "dv", "dv"], "{case}");
            assert_eq!(sc1.thrusters[0].color, vec![255, 0, 0, 255], "{case}");
            assert!(sc2.thrusters[0].color.is_empty(), "{case}");
            assert_eq!(sc2.thrusters[2].position, vec![0.0, 2.0, -3.0], "{case}");
            assert_eq!(sc2.thrusters[2].current_thrust, TH2_DV, "{case}");
            let settings = messages[0].settings.as_ref().expect("settings");
            assert_eq!(
                settings.default_thruster_color,
                vec![255, 255, 0, 255],
                "{case}"
            );
        } else {
            assert!(
                sc1.thrusters.is_empty() && sc2.thrusters.is_empty(),
                "{case}"
            );
        }
        if check_rw {
            assert_eq!(
                (sc1.reaction_wheels.len(), sc2.reaction_wheels.len()),
                (2, 1),
                "{case}"
            );
            let wheel = &sc2.reaction_wheels[0];
            assert_eq!(wheel.spin_axis_vector, vec![0.0, 1.0, 0.0], "{case}");
            assert!(
                is_double_equal_relative(wheel.wheel_speed, omega1_sc2(), 0.001),
                "{case}"
            );
            assert!(
                is_double_equal_relative(wheel.max_speed, 3000.0 * RPM, 1e-12),
                "{case}"
            );
            assert_eq!(wheel.max_torque, 0.5, "{case}");
        } else {
            assert!(sc1.reaction_wheels.is_empty(), "{case}");
        }
    }

    /// Upstream `test_module`, over its parameter grid.
    #[test]
    fn replays_spacecraft_states_through_viz_interface() {
        for convert_pos_units in [-1.0, 1000.0] {
            for att_type in [-1, 0, 1, 2] {
                for check_thruster in [false, true] {
                    for check_rw in [false, true] {
                        run(convert_pos_units, att_type, check_thruster, check_rw);
                    }
                }
            }
        }
    }

    fn context_at(sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn module_reading(text: &str, header_line: bool) -> (DataFileToViz, PathBuf) {
        let path = unique_path("edge", "txt");
        std::fs::write(&path, text).expect("writable data file");
        let mut module = DataFileToViz::new(DataFileToVizConfig {
            data_file_name: path.clone(),
            header_line,
            convert_pos_to_meters: 1.0,
            ..DataFileToVizConfig::default()
        });
        module.add_spacecraft();
        (module, path)
    }

    /// One spacecraft line with space delimiters, upstream's default.
    fn line(t: f64, x: f64) -> String {
        format!("{t} {x} 0 0 0 0 0 0 0 0 0 0 0\n")
    }

    /// Upstream indexes its flat property lists with a counter restarting
    /// at each spacecraft; each device keeps its own properties here.
    #[test]
    fn each_spacecraft_keeps_its_own_device_properties() {
        let path = unique_path("devices", "txt");
        std::fs::write(
            &path,
            "0 1 0 0 0 0 0 0 0 0 0 0 0 5 7 0.5 1 0 0 0 0 0 0 0 0 0 0 0 6 8 0.25\n",
        )
        .expect("writable data file");
        let mut module = DataFileToViz::new(DataFileToVizConfig {
            data_file_name: path.clone(),
            header_line: false,
            ..DataFileToVizConfig::default()
        });
        for (max_thrust_n, z_m, omega_max) in [(1.0, 3.0, 10.0), (2.0, -3.0, 20.0)] {
            let sc = module.add_spacecraft();
            sc.add_thruster_cluster(ThrClusterMap::default())
                .add_thruster(
                    Vector3::new(0.0, 0.0, z_m),
                    Vector3::new(0.0, 0.0, -2.0 * z_m),
                    max_thrust_n,
                );
            sc.add_reaction_wheel(
                Vector3::new(z_m, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 4.0),
                omega_max,
                0.1,
            );
        }
        module.init();
        module.update(&context_at(0));
        std::fs::remove_file(&path).expect("removable data file");

        let thruster = module.spacecraft[1].thrusters().next().expect("thruster").1;
        let output = thruster.thruster_out_msg.read();
        assert_eq!(output.max_thrust_n, 2.0);
        assert_eq!(output.thrust_force_n, 6.0);
        assert_eq!(output.thruster_location_m, Vector3::new(0.0, 0.0, -3.0));
        assert_eq!(
            output.thruster_direction,
            Vector3::new(0.0, 0.0, 1.0),
            "normalized"
        );
        let wheel = &module.spacecraft[1].reaction_wheels[0];
        let config = wheel.rw_config_out_msg.read();
        assert_eq!(config.max_speed_radps, 20.0);
        assert_eq!(config.position_m, Vector3::new(-3.0, 0.0, 0.0));
        assert_eq!(config.spin_axis_body, Vector3::z());
        let state = wheel.rw_state_out_msg.read();
        assert_eq!(
            (state.omega_radps, state.applied_motor_torque_nm),
            (8.0, 0.25)
        );
        assert_eq!(
            module.spacecraft[0].sc_state_out_msg.read().position_m.x,
            1000.0
        );
    }

    /// The header line is skipped by default; a line that does not parse is
    /// skipped whole; past the end of the file the last values stay.
    #[test]
    fn header_bad_lines_and_end_of_file() {
        let text = format!(
            "time x y z\n{}0.1 2 0 0 0 0 0 0 0 0 0 0 oops\n0.2 3 0 0\n{}",
            line(0.0, 1.0),
            line(0.3, 4.0)
        );
        let (mut module, path) = module_reading(&text, true);
        module.init();
        let x = |module: &DataFileToViz| module.spacecraft[0].sc_state_out_msg.read().position_m.x;
        module.update(&context_at(0));
        assert_eq!(x(&module), 1.0);
        module.update(&context_at(1));
        assert_eq!(x(&module), 1.0, "non-numeric value");
        module.update(&context_at(2));
        assert_eq!(x(&module), 1.0, "short line");
        module.update(&context_at(3));
        assert_eq!(x(&module), 4.0);
        module.update(&context_at(4));
        assert_eq!(x(&module), 4.0, "end of file");

        // A reset reopens the file at the top.
        module.reset(&context_at(0));
        module.update(&context_at(0));
        assert_eq!(x(&module), 1.0);
        std::fs::remove_file(&path).expect("removable data file");
    }

    #[test]
    fn without_header_the_first_line_is_data() {
        let (mut module, path) = module_reading(&line(0.0, 9.0), false);
        module.init();
        module.update(&context_at(0));
        std::fs::remove_file(&path).expect("removable data file");
        assert_eq!(
            module.spacecraft[0].sc_state_out_msg.read().position_m.x,
            9.0
        );
    }

    #[test]
    fn missing_file_publishes_nothing() {
        let mut module = DataFileToViz::new(DataFileToVizConfig {
            data_file_name: unique_path("missing", "txt"),
            ..DataFileToVizConfig::default()
        });
        module.add_spacecraft();
        module.init();
        module.update(&context_at(0));
        assert_eq!(
            module.spacecraft[0].sc_state_out_msg.read().position_m,
            Vector3::zeros()
        );
    }

    /// Euler parameters with a negative scalar map to the short MRP set, and
    /// 3-2-1 angles agree with upstream's conversion.
    #[test]
    fn attitude_types_convert_like_upstream() {
        let beta = [-0.5, 0.5, 0.5, 0.5];
        let angles = [0.3, -PI / 5.0, 1.1];
        assert_eq!(
            crate::utilities::rigid_body_kinematics::ep_to_mrp(Vector4::from(beta)),
            Vector3::from(ep_to_mrp_truth(beta))
        );
        let rust = crate::utilities::rigid_body_kinematics::euler321_to_mrp(Vector3::from(angles));
        assert!(is_vector_equal(&rust, euler321_to_mrp_truth(angles), 1e-15));
    }
}
