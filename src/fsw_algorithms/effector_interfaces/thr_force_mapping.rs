//! Attitude-control torque to thruster force mapping: port of upstream
//! Basilisk `fswAlgorithms/effectorInterfaces/thrForceMapping`.
//!
//! Each thruster `i` produces the body torque `d_i F_i` with the torque arm
//! `d_i = (r_i - r_COM) × ĝ_i`, so a cluster produces `τ = [D] F` with
//! `[D] = [d_1 … d_N]`. The commanded torque is projected onto up to three
//! control axes `[C]` (rows `ĉ_j`), and the minimum-norm forces
//!
//! `F = ([C][D])ᵀ ([C][D] ([C][D])ᵀ)⁻¹ [C] L_r`
//!
//! implement it. The Gram matrix is padded with identity rows for the
//! uncontrolled axes so that it stays invertible.
//!
//! * On-pulsing thrusters (RCS, [`ThrForceSign::Positive`]) must produce
//!   non-negative forces: the most negative force is subtracted from all of
//!   them, which assumes all thrusters firing together produce no torque.
//! * Off-pulsing thrusters (DV, [`ThrForceSign::Negative`]) nominally fire at
//!   their maximum thrust; the torque of that nominal firing is added to the
//!   request and the module solves for the negative force corrections.
//! * Off-pulsing always, and on-pulsing with `use_2nd_loop`, repeats the
//!   minimum-norm solution using only the thrusters whose first-pass force has
//!   the right sign; all others are commanded zero.
//! * When the angle between the requested and the saturated torque exceeds
//!   `ang_err_thresh_rad` and a thruster is over its maximum, all forces are
//!   scaled down uniformly so that none is saturated.

use std::fmt;

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    BodyTorqueCommandMsg, Input, MAX_EFF_COUNT, Output, ThrusterArrayConfigMsg,
    ThrusterArrayForceCommandMsg, VehicleConfigMsg,
};
use crate::utilities::linear_algebra::{
    m33_determinant, m33_inverse, normalize_or_zero, safe_acos,
};
use crate::{Module, SimulationContext};

/// Sign of the thruster forces the mapping solves for: upstream
/// `thrForceSign`, with the upstream integer codes as discriminants.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub enum ThrForceSign {
    /// Upstream `0`, the zero-initialized default. Reset reports it as a
    /// configuration error; the mapping then neither shifts the forces nor
    /// selects any thruster in the second loop.
    #[default]
    Unset = 0,
    /// On-pulsing (RCS) thrusters: non-negative forces; upstream `+1`.
    Positive = 1,
    /// Off-pulsing (DV) thrusters: non-positive corrections to the maximum
    /// thrust; upstream `-1`.
    Negative = -1,
}

impl ThrForceSign {
    /// The upstream integer code as the `f64` the mapping multiplies by.
    fn factor(self) -> f64 {
        f64::from(self as i32)
    }
}

/// User-settable parameters of [`ThrForceMapping`]. `Default` is upstream's
/// zero-initialized configuration, which reset reports as incomplete until
/// the control axes and `thr_force_sign` are set.
#[derive(Clone, Debug)]
pub struct ThrForceMappingConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `controlAxes_B` \[-\]: body-frame control axes. The leading
    /// axes whose norm exceeds `epsilon` are controlled; the first one that
    /// does not ends the list. As upstream, reset normalizes the controlled
    /// axes in place.
    pub control_axes_b: [Vector3<f64>; 3],
    /// Upstream `thrForceSign`: on-pulsing (`+1`) or off-pulsing (`-1`).
    pub thr_force_sign: ThrForceSign,
    /// Upstream `angErrThresh` \[rad\]: torque direction error above which
    /// saturated forces are scaled down (default 0, so any nonzero error
    /// scales). Upstream's comment says a negative value bypasses the
    /// scaling, but its code, followed here, then always scales.
    pub ang_err_thresh_rad: f64,
    /// Upstream `epsilon` \[-\]: smallness threshold for control axis norms,
    /// the Gram determinant and the requested torque norm (default 0).
    pub epsilon: f64,
    /// Upstream `use2ndLoop`: repeat the on-pulsing solution with only the
    /// thrusters of positive first-pass force (default off). Off-pulsing
    /// always runs the second loop.
    pub use_2nd_loop: bool,
}

impl Default for ThrForceMappingConfig {
    fn default() -> Self {
        Self {
            name: "thrForceMapping".to_string(),
            control_axes_b: [Vector3::zeros(); 3],
            thr_force_sign: ThrForceSign::Unset,
            ang_err_thresh_rad: 0.0,
            epsilon: 0.0,
            use_2nd_loop: false,
        }
    }
}

/// Configuration problem that upstream `Reset_thrForceMapping` reports with
/// `BSK_ERROR`. Upstream raises on the first one and abandons the reset; here
/// each is logged with `log::error!` and the reset completes, as everywhere in
/// the crate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ResetError {
    NoControlAxes,
    ThrForceSignUnset,
    InputNotConnected(&'static str),
    NonPositiveMaxThrust { thruster: usize },
}

impl fmt::Display for ResetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoControlAxes => write!(f, "thrForceMapping is not set up to control any axes"),
            Self::ThrForceSignUnset => write!(
                f,
                "thrForceMapping must have thr_force_sign set to either +1 or -1"
            ),
            Self::InputNotConnected(port) => write!(f, "thrForceMapping.{port} is not connected"),
            Self::NonPositiveMaxThrust { thruster } => write!(
                f,
                "thrForceMapping: thruster {thruster} has a non-sensible saturation limit of <= 0 N"
            ),
        }
    }
}

/// Maps a commanded body torque onto thruster forces (upstream
/// `thrForceMapping`).
#[derive(Clone, Debug)]
pub struct ThrForceMapping {
    /// User-settable parameters.
    pub config: ThrForceMappingConfig,
    /// Upstream `cmdTorqueInMsg`: commanded attitude control torque `L_r`.
    pub cmd_torque_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `thrConfigInMsg`: thruster cluster configuration, read at
    /// reset.
    pub thr_config_in_msg: Input<ThrusterArrayConfigMsg>,
    /// Upstream `vehConfigInMsg`: vehicle configuration; its center of mass is
    /// read on every update.
    pub veh_config_in_msg: Input<VehicleConfigMsg>,
    /// Upstream `thrForceCmdOutMsg`: commanded thruster forces.
    pub thr_force_cmd_out_msg: Output<ThrusterArrayForceCommandMsg>,
    /// Upstream `numControlAxes`.
    num_control_axes: usize,
    /// Upstream `rThruster_B` \[m\]: thruster locations, body frame.
    positions_b_m: Vec<Vector3<f64>>,
    /// Upstream `gtThruster_B` \[-\]: thrust unit directions, body frame.
    directions_b: Vec<Vector3<f64>>,
    /// Upstream `thrForcMag` \[N\]: maximum thrusts. As upstream, an entry is
    /// only overwritten by a sensible (positive) limit and is never cleared.
    max_thrust_n: [f64; MAX_EFF_COUNT],
    /// Upstream `outTorqAngErr` \[rad\].
    torque_angle_error_rad: f64,
}

impl ThrForceMapping {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: ThrForceMappingConfig) -> Self {
        Self {
            config,
            cmd_torque_in_msg: Input::default(),
            thr_config_in_msg: Input::default(),
            veh_config_in_msg: Input::default(),
            thr_force_cmd_out_msg: Output::default(),
            num_control_axes: 0,
            positions_b_m: Vec::new(),
            directions_b: Vec::new(),
            max_thrust_n: [0.0; MAX_EFF_COUNT],
            torque_angle_error_rad: 0.0,
        }
    }

    /// Number of controlled axes found at the last reset (upstream
    /// `numControlAxes`).
    pub fn num_control_axes(&self) -> usize {
        self.num_control_axes
    }

    /// Per-thruster maximum thrust \[N\] used by the saturation logic
    /// (upstream `thrForcMag`).
    pub fn thruster_max_thrust_n(&self) -> &[f64; MAX_EFF_COUNT] {
        &self.max_thrust_n
    }

    /// Angle between the projected requested torque and the torque of the
    /// saturated output forces at the last update \[rad\] (upstream
    /// `outTorqAngErr`).
    pub fn torque_angle_error_rad(&self) -> f64 {
        self.torque_angle_error_rad
    }

    /// Upstream `Reset_thrForceMapping`: counts and normalizes the control
    /// axes and reads the thruster configuration, logging every configuration
    /// error upstream reports.
    pub fn reset(&mut self, _call_time_nanos: u64) {
        for error in self.configure() {
            log::error!("{error}");
        }
    }

    fn configure(&mut self) -> Vec<ResetError> {
        let mut errors = Vec::new();

        let epsilon = self.config.epsilon;
        self.num_control_axes = 0;
        for axis in &mut self.config.control_axes_b {
            if axis.norm() > epsilon {
                *axis = normalize_or_zero(*axis);
                self.num_control_axes += 1;
            } else {
                break;
            }
        }
        if self.num_control_axes == 0 {
            errors.push(ResetError::NoControlAxes);
        }
        if self.config.thr_force_sign == ThrForceSign::Unset {
            errors.push(ResetError::ThrForceSignUnset);
        }

        for (connected, port) in [
            (self.thr_config_in_msg.is_connected(), "thr_config_in_msg"),
            (self.veh_config_in_msg.is_connected(), "veh_config_in_msg"),
            (self.cmd_torque_in_msg.is_connected(), "cmd_torque_in_msg"),
        ] {
            if !connected {
                errors.push(ResetError::InputNotConnected(port));
            }
        }

        let cluster = self.thr_config_in_msg.read();
        let thrusters = cluster.active_thrusters();
        self.positions_b_m = thrusters.iter().map(|t| t.position_thrust_b_m).collect();
        self.directions_b = thrusters.iter().map(|t| t.t_hat_thrust_b).collect();
        for (thruster, (config, max_thrust_n)) in
            thrusters.iter().zip(&mut self.max_thrust_n).enumerate()
        {
            if config.max_thrust_n <= 0.0 {
                errors.push(ResetError::NonPositiveMaxThrust { thruster });
            } else {
                *max_thrust_n = config.max_thrust_n;
            }
        }
        errors
    }

    /// Upstream `Update_thrForceMapping`.
    fn compute_thruster_forces(&mut self) -> ThrusterArrayForceCommandMsg {
        let sign = self.config.thr_force_sign;
        let epsilon = self.config.epsilon;
        let center_of_mass_b_m = self.veh_config_in_msg.read().center_of_mass_body_m;
        let max_thrust_n = &self.max_thrust_n[..self.positions_b_m.len()];

        // Eq. 4 and 6: the torque arm of each thruster, the columns of [D].
        let torque_arms: Vec<Vector3<f64>> = self
            .positions_b_m
            .iter()
            .zip(&self.directions_b)
            .map(|(position, direction)| (position - center_of_mass_b_m).cross(direction))
            .collect();

        // Eq. 5 and 7: off-pulsing thrusters nominally fire at full thrust, so
        // the request is offset by the torque of that nominal firing.
        let mut torque_offset_nm = Vector3::zeros();
        if sign == ThrForceSign::Negative {
            for (arm, &max_thrust) in torque_arms.iter().zip(max_thrust_n) {
                torque_offset_nm -= max_thrust * arm;
            }
        }
        let requested_torque_nm =
            torque_offset_nm + self.cmd_torque_in_msg.read().torque_request_body_nm;

        let axes = self.control_axes();
        // Only the controlled components of the projected request are nonzero.
        let projected_torque_nm = axes.matrix * requested_torque_nm;

        let mut forces_n = axes.minimum_norm_forces(&torque_arms, projected_torque_nm, epsilon);
        if sign == ThrForceSign::Positive {
            subtract_min(&mut forces_n);
        }

        if sign == ThrForceSign::Negative || self.config.use_2nd_loop {
            // Eq. 11 and 12: solve again with the thrusters of the desired sign.
            let used: Vec<bool> = forces_n.iter().map(|f| f * sign.factor() > 0.0).collect();
            let used_arms: Vec<Vector3<f64>> = torque_arms
                .iter()
                .zip(&used)
                .filter_map(|(arm, &is_used)| is_used.then_some(*arm))
                .collect();
            let mut used_forces_n =
                axes.minimum_norm_forces(&used_arms, projected_torque_nm, epsilon);
            if sign == ThrForceSign::Positive {
                subtract_min(&mut used_forces_n);
            }
            let mut used_forces_n = used_forces_n.into_iter();
            forces_n = used
                .iter()
                .map(|&is_used| {
                    if is_used {
                        used_forces_n
                            .next()
                            .expect("one second-loop force per used thruster")
                    } else {
                        0.0
                    }
                })
                .collect();
        }

        // Eq. 16; a negative threshold always scales saturated forces.
        self.torque_angle_error_rad = torque_angle_error(
            &torque_arms,
            projected_torque_nm,
            epsilon,
            &forces_n,
            max_thrust_n,
        );
        if self.torque_angle_error_rad > self.config.ang_err_thresh_rad {
            let max_fraction_used = forces_n
                .iter()
                .zip(max_thrust_n)
                .filter(|&(_, &max_thrust)| max_thrust > 0.0)
                .map(|(force, max_thrust)| force.abs() / max_thrust)
                .fold(0.0, f64::max);
            if max_fraction_used > 1.0 {
                let scale = 1.0 / max_fraction_used;
                forces_n.iter_mut().for_each(|force| *force *= scale);
                self.torque_angle_error_rad = torque_angle_error(
                    &torque_arms,
                    projected_torque_nm,
                    epsilon,
                    &forces_n,
                    max_thrust_n,
                );
            }
        }

        ThrusterArrayForceCommandMsg::from_active(&forces_n)
    }

    fn control_axes(&self) -> ControlAxes {
        let count = self.num_control_axes;
        let axes = &self.config.control_axes_b;
        ControlAxes {
            matrix: Matrix3::from_fn(
                |row, column| {
                    if row < count { axes[row][column] } else { 0.0 }
                },
            ),
            count,
        }
    }
}

/// The control-axis matrix `[C]`: rows are the controlled axes, padded with
/// zero rows.
struct ControlAxes {
    matrix: Matrix3<f64>,
    count: usize,
}

impl ControlAxes {
    /// Upstream `findMinimumNormForce` (Eq. 9 and 15): the minimum-norm forces
    /// of the thrusters with torque arms `torque_arms` that produce the
    /// projected torque, with no sign constraint.
    fn minimum_norm_forces(
        &self,
        torque_arms: &[Vector3<f64>],
        projected_torque_nm: Vector3<f64>,
        epsilon: f64,
    ) -> Vec<f64> {
        // The columns of [C][D].
        let projected_arms: Vec<Vector3<f64>> =
            torque_arms.iter().map(|arm| self.matrix * arm).collect();
        // [C][D]([C][D])^T on the controlled block, identity elsewhere.
        let mut gram = Matrix3::identity();
        for row in 0..self.count {
            for column in 0..self.count {
                gram[(row, column)] = projected_arms
                    .iter()
                    .fold(0.0, |sum, arm| sum + arm[row] * arm[column]);
            }
        }
        let gram_inverse = if m33_determinant(&gram) > epsilon {
            m33_inverse(&gram).unwrap_or_else(Matrix3::zeros)
        } else {
            Matrix3::zeros()
        };
        let weights = gram_inverse * projected_torque_nm;
        // Summed from +0 like upstream's `mtMultV` (not `dot`), so that a
        // force with only zero terms is +0 rather than -0.
        projected_arms
            .iter()
            .map(|arm| {
                arm.iter()
                    .zip(&weights)
                    .fold(0.0, |sum, (arm_j, weight_j)| sum + arm_j * weight_j)
            })
            .collect()
    }
}

/// Upstream `substractMin`: subtracts the most negative force (if any) from
/// all forces, so that the smallest becomes zero.
fn subtract_min(forces_n: &mut [f64]) {
    let min_n = forces_n
        .iter()
        .fold(0.0, |min, &force| if force < min { force } else { min });
    forces_n.iter_mut().for_each(|force| *force -= min_n);
}

/// Upstream `computeTorqueAngErr`: angle \[rad\] between `requested_torque_nm`
/// and the torque the forces produce once each is saturated at its thruster's
/// maximum. Zero when the request norm does not exceed `epsilon`.
fn torque_angle_error(
    torque_arms: &[Vector3<f64>],
    requested_torque_nm: Vector3<f64>,
    epsilon: f64,
    forces_n: &[f64],
    max_thrust_n: &[f64],
) -> f64 {
    if requested_torque_nm.norm() > epsilon {
        let requested_direction = normalize_or_zero(requested_torque_nm);
        let actual_torque_nm = torque_arms.iter().zip(forces_n).zip(max_thrust_n).fold(
            Vector3::zeros(),
            |sum, ((arm, &force), &max_thrust)| {
                let saturated = if force.abs() < max_thrust {
                    force
                } else {
                    max_thrust * force.abs() / force
                };
                sum + saturated * arm
            },
        );
        let cos_angle = requested_direction.dot(&normalize_or_zero(actual_torque_nm));
        if cos_angle < 1.0 {
            return safe_acos(cos_angle);
        }
    }
    0.0
}

impl Module for ThrForceMapping {
    fn init(&mut self) {
        self.thr_force_cmd_out_msg
            .write(ThrusterArrayForceCommandMsg::default());
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        ThrForceMapping::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, _context: &SimulationContext) {
        let forces = self.compute_thruster_forces();
        self.thr_force_cmd_out_msg.write(forces);
    }
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_thrForceMapping.py` and
    //! `test_thrForceMap_configErr.py`, with the truth of
    //! `Support/Results_thrForceMapping.py` re-implemented in
    //! [`upstream_truth`]. Upstream's `BSK_ERROR` raises a Python exception in
    //! the config-error pytest; here the errors reset reports are checked
    //! instead.

    use nalgebra::{DMatrix, Matrix3, Vector3};

    use crate::messages::{
        BodyTorqueCommandMsg, MAX_EFF_COUNT, Output, ThrusterArrayConfigMsg, ThrusterConfigMsg,
        VehicleConfigMsg,
    };
    use crate::{Module, SimulationContext};

    use super::{ResetError, ThrForceMapping, ThrForceMappingConfig, ThrForceSign};

    const DV_POSITIONS_M: [[f64; 3]; 6] = [
        [0.0, 0.413, -0.1671],
        [0.0, -0.413, -0.1671],
        [0.35766849176297305, 0.20650000000000013, -0.1671],
        [0.3576684917629732, -0.20649999999999988, -0.1671],
        [-0.35766849176297333, 0.20649999999999968, -0.1671],
        [-0.35766849176297305, -0.20650000000000018, -0.1671],
    ];
    const DV_DIRECTIONS: [[f64; 3]; 6] = [[0.0, 0.0, 1.0]; 6];
    const RCS_POSITIONS_M: [[f64; 3]; 8] = [
        [-0.86360, -0.82550, 1.79070],
        [-0.82550, -0.86360, 1.79070],
        [0.82550, 0.86360, 1.79070],
        [0.86360, 0.82550, 1.79070],
        [-0.86360, -0.82550, -1.79070],
        [-0.82550, -0.86360, -1.79070],
        [0.82550, 0.86360, -1.79070],
        [0.86360, 0.82550, -1.79070],
    ];
    const RCS_DIRECTIONS: [[f64; 3]; 8] = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ];

    /// One parametrization of the upstream pytests.
    #[derive(Clone, Copy, Debug)]
    struct Scenario {
        use_dv_thruster: bool,
        use_com_offset: bool,
        drop_thruster: usize,
        use_2nd_loop: bool,
        num_control_axis: usize,
        saturate_thrusters: u8,
        misconfig_thruster: bool,
    }

    /// The module wired as the pytest wires it, and the data its truth uses.
    struct Fixture {
        module: ThrForceMapping,
        /// `rcsLocationData` / `rcsDirectionData`: `MAX_EFF_COUNT` rows, the
        /// installed thrusters first and zeros after.
        positions_m: Vec<Vector3<f64>>,
        directions: Vec<Vector3<f64>>,
        num_thrusters: usize,
        /// The pytest's `numControlAxis`.
        num_control_axis: usize,
        requested_torque_nm: Vector3<f64>,
        center_of_mass_m: Vector3<f64>,
    }

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: hifitime::Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn fixture(scenario: Scenario) -> Fixture {
        let center_of_mass_m = if scenario.use_com_offset {
            Vector3::new(0.03, 0.001, 0.02)
        } else {
            Vector3::zeros()
        };
        let requested_torque_nm = if scenario.saturate_thrusters > 0 {
            Vector3::new(10.0, -5.0, 7.0)
        } else {
            Vector3::new(1.0, -0.5, 0.7)
        };
        let mut config = ThrForceMappingConfig {
            use_2nd_loop: scenario.use_2nd_loop,
            epsilon: 0.0005,
            ..Default::default()
        };
        if scenario.saturate_thrusters == 2 {
            config.ang_err_thresh_rad = 10.0_f64.to_radians();
        }
        for axis in 0..scenario.num_control_axis {
            config.control_axes_b[axis][axis] = 1.0;
        }

        let (positions, directions, max_thrust_n): (&[[f64; 3]], &[[f64; 3]], f64) =
            if scenario.use_dv_thruster {
                config.thr_force_sign = ThrForceSign::Negative;
                (&DV_POSITIONS_M, &DV_DIRECTIONS, 10.0)
            } else {
                config.thr_force_sign = ThrForceSign::Positive;
                (&RCS_POSITIONS_M, &RCS_DIRECTIONS, 0.95)
            };
        // The pytest zeroes the last `dropThruster` thrusters and installs the
        // others.
        let num_thrusters = positions.len() - scenario.drop_thruster;
        let padded = |rows: &[[f64; 3]]| -> Vec<Vector3<f64>> {
            (0..MAX_EFF_COUNT)
                .map(|i| {
                    if i < num_thrusters {
                        Vector3::from(rows[i])
                    } else {
                        Vector3::zeros()
                    }
                })
                .collect()
        };
        let positions_m = padded(positions);
        let directions = padded(directions);

        let thrusters: Vec<_> = (0..num_thrusters)
            .map(|i| ThrusterConfigMsg {
                position_thrust_b_m: positions_m[i],
                t_hat_thrust_b: directions[i],
                max_thrust_n: if scenario.misconfig_thruster && i == 0 {
                    0.0
                } else {
                    max_thrust_n
                },
                swirl_torque_nm: 0.0,
            })
            .collect();

        let mut module = ThrForceMapping::new(config);
        Output::new(ThrusterArrayConfigMsg::from_active(&thrusters))
            .connect_to(&mut module.thr_config_in_msg);
        Output::new(VehicleConfigMsg {
            center_of_mass_body_m: center_of_mass_m,
            ..Default::default()
        })
        .connect_to(&mut module.veh_config_in_msg);
        Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: requested_torque_nm,
        })
        .connect_to(&mut module.cmd_torque_in_msg);

        Fixture {
            module,
            positions_m,
            directions,
            num_thrusters,
            num_control_axis: scenario.num_control_axis,
            requested_torque_nm,
            center_of_mass_m,
        }
    }

    /// Upstream `Results_thrForceMapping.results_thrForceMapping`, fed what
    /// the pytest feeds it: the module's control axes and maximum thrusts as
    /// read back after reset.
    fn upstream_truth(fixture: &Fixture, use_2nd_loop: bool) -> Vec<f64> {
        let module = &fixture.module;
        let config = &module.config;
        let sign = f64::from(config.thr_force_sign as i32);
        let thr_force_mag = module.thruster_max_thrust_n();
        let epsilon = config.epsilon;
        let num_thrusters = fixture.num_thrusters;
        // `np.reshape(controlAxes_B, (3, 3))`, rows are axes.
        let c = Matrix3::from_fn(|row, column| config.control_axes_b[row][column]);
        let arm = |i: usize| {
            (fixture.positions_m[i] - fixture.center_of_mass_m).cross(&fixture.directions[i])
        };

        let mut lr_offset = Vector3::zeros();
        let d = DMatrix::from_fn(3, MAX_EFF_COUNT, |row, i| arm(i)[row]);
        if sign < 0.0 {
            for (i, &max_thrust_n) in thr_force_mag.iter().enumerate() {
                lr_offset -= max_thrust_n * arm(i);
            }
        }
        let lr_bar = c * (fixture.requested_torque_nm + lr_offset);
        let mut f = map_to_force(&c, &d, lr_bar, epsilon);
        if sign > 0.0 {
            subtract_min(&mut f, num_thrusters);
        }

        let used: Vec<bool> = f.iter().map(|force| force * sign > epsilon).collect();
        if sign < 0.0 || use_2nd_loop {
            let used_columns: Vec<usize> = (0..f.len()).filter(|&i| used[i]).collect();
            let d_new = DMatrix::from_fn(3, used_columns.len(), |row, k| arm(used_columns[k])[row]);
            let mut f_new = map_to_force(&c, &d_new, lr_bar, epsilon);
            if sign > 0.0 {
                subtract_min(&mut f_new, d_new.ncols());
            }
            let mut f_new = f_new.into_iter();
            for (force, &is_used) in f.iter_mut().zip(&used) {
                *force = if is_used {
                    f_new.next().expect("one force per used thruster")
                } else {
                    0.0
                };
            }
        }

        let angle = angle_error(&d, lr_bar, &f, thr_force_mag, num_thrusters);
        if angle > config.ang_err_thresh_rad {
            let mut max_fract_use = 0.0;
            for i in 0..num_thrusters {
                if thr_force_mag[i] > 0.0 && f[i].abs() / thr_force_mag[i] > max_fract_use {
                    max_fract_use = f[i].abs() / thr_force_mag[i];
                }
            }
            if max_fract_use > 1.0 {
                f.iter_mut().for_each(|force| *force /= max_fract_use);
            }
        }
        f
    }

    /// `Results_thrForceMapping.mapToForce`.
    fn map_to_force(
        c: &Matrix3<f64>,
        d: &DMatrix<f64>,
        lr_bar: Vector3<f64>,
        epsilon: f64,
    ) -> Vec<f64> {
        let nonzero_columns = |m: &DMatrix<f64>| {
            m.column_iter()
                .filter(|column| column.iter().any(|&x| x != 0.0))
                .count()
        };
        let c_dyn = DMatrix::from_fn(3, 3, |row, column| c[(row, column)]);
        let num_control_axes = nonzero_columns(&c_dyn);
        let num_thr = nonzero_columns(d);
        let cd = &c_dyn * d;
        let mut ddt = Matrix3::identity();
        for i in 0..num_control_axes {
            for j in 0..num_control_axes {
                ddt[(i, j)] = (0..num_thr).map(|k| cd[(i, k)] * cd[(j, k)]).sum();
            }
        }
        let ddt_inv = match ddt.try_inverse() {
            Some(inverse) if ddt.determinant() >= epsilon => inverse,
            _ => Matrix3::zeros(),
        };
        let weights = ddt_inv * lr_bar;
        cd.column_iter()
            .map(|column| column.dot(&weights))
            .collect()
    }

    /// `Results_thrForceMapping.subtractMin`.
    fn subtract_min(f: &mut [f64], size: usize) {
        let mut min_value = 0.0;
        for &force in &f[..size] {
            if force < min_value {
                min_value = force;
            }
        }
        f[..size].iter_mut().for_each(|force| *force -= min_value);
    }

    /// `Results_thrForceMapping.results_computeAngErr`.
    fn angle_error(
        d: &DMatrix<f64>,
        lr_bar: Vector3<f64>,
        f: &[f64],
        thr_force_mag: &[f64],
        num_thrusters: usize,
    ) -> f64 {
        if lr_bar.norm() <= 1e-9 {
            return 0.0;
        }
        let mut tau_actual = Vector3::zeros();
        for i in 0..num_thrusters {
            let thr_force = if f[i].abs() < thr_force_mag[i] {
                f[i]
            } else {
                thr_force_mag[i] * f[i].abs() / f[i]
            };
            tau_actual += thr_force * Vector3::new(d[(0, i)], d[(1, i)], d[(2, i)]);
        }
        let cos_angle = (lr_bar / lr_bar.norm()).dot(&(tau_actual / tau_actual.norm()));
        if cos_angle < 1.0 {
            cos_angle.acos()
        } else {
            0.0
        }
    }

    /// Upstream `unitTestSupport.isArrayEqual` over the first `size` entries:
    /// a NaN result fails, otherwise every entry must be within `accuracy`.
    fn assert_array_equal(result: &[f64], truth: &[f64], size: usize, accuracy: f64, what: &str) {
        assert!(
            !result.iter().any(|x| x.is_nan()),
            "{what}: NaN in {result:?}"
        );
        for i in 0..size {
            assert!(
                (result[i] - truth[i]).abs() <= accuracy,
                "{what}[{i}]: expected {}, got {}",
                truth[i],
                result[i]
            );
        }
    }

    /// Upstream `test_thrForceMapping.py::test_module`: every combination of
    /// DV/RCS thrusters, the four (COM offset, dropped thrusters, second loop)
    /// cases, one to three control axes and three saturation levels. The
    /// first recorded sample (t = 0) must match the Python truth within 1e-6,
    /// RCS forces must not be negative, and the produced torque must point
    /// along the request on the control axes. As upstream, DV thrusters with
    /// three control axes are run but not checked.
    #[test]
    fn matches_upstream_thr_force_mapping_unit_test() {
        let accuracy = 1e-6;
        for use_dv_thruster in [true, false] {
            for (use_com_offset, drop_thruster, use_2nd_loop) in [
                (false, 0, false),
                (false, 1, true),
                (false, 2, true),
                (true, 0, false),
            ] {
                for num_control_axis in [1, 2, 3] {
                    for saturate_thrusters in [0, 1, 2] {
                        let scenario = Scenario {
                            use_dv_thruster,
                            use_com_offset,
                            drop_thruster,
                            use_2nd_loop,
                            num_control_axis,
                            saturate_thrusters,
                            misconfig_thruster: false,
                        };
                        let mut fixture = fixture(scenario);
                        fixture.module.init();
                        fixture.module.update(&context(0));
                        let output = fixture.module.thr_force_cmd_out_msg.read().thrust_force_n;
                        if use_dv_thruster && num_control_axis == 3 {
                            continue;
                        }

                        let what = format!("{scenario:?}");
                        let truth = upstream_truth(&fixture, use_2nd_loop);
                        assert_array_equal(&output, &truth, MAX_EFF_COUNT, accuracy, &what);
                        if !use_dv_thruster {
                            assert!(output.iter().all(|&f| f >= 0.0), "{what}: {output:?}");
                            assert!(truth.iter().all(|&f| f >= 0.0), "{what}: {truth:?}");
                        }
                        assert_torque_along_request(&fixture, &output, &what);
                    }
                }
            }
        }
    }

    /// The pytest's "CompareTorques" check: on the control axes, the torque
    /// of the output forces must point along the (offset) request.
    fn assert_torque_along_request(fixture: &Fixture, output: &[f64], what: &str) {
        let module = &fixture.module;
        let n = fixture.num_thrusters;
        let arm = |i: usize| {
            (fixture.positions_m[i] - fixture.center_of_mass_m).cross(&fixture.directions[i])
        };
        let received: Vector3<f64> = (0..MAX_EFF_COUNT).map(|i| output[i] * arm(i)).sum();
        let mut lr_offset = Vector3::zeros();
        if module.config.thr_force_sign == ThrForceSign::Negative {
            for i in 0..n {
                if module.thruster_max_thrust_n()[i] >= 0.0 {
                    lr_offset -= module.thruster_max_thrust_n()[i] * arm(i);
                }
            }
        }
        let lr_b = fixture.requested_torque_nm + lr_offset;
        // `C` here is the pytest's own (numControlAxis x 3) identity rows.
        let project = |v: Vector3<f64>| -> Vector3<f64> {
            (0..fixture.num_control_axis).fold(Vector3::zeros(), |sum, axis| {
                let mut c = Vector3::zeros();
                c[axis] = 1.0;
                sum + c * c.dot(&v)
            })
        };
        let requested_bar = project(lr_b);
        let received_bar = project(received);
        let requested_unit = requested_bar / requested_bar.norm();
        let received_unit = if received_bar.norm() == 0.0 {
            Vector3::zeros()
        } else {
            received_bar / received_bar.norm()
        };
        assert_array_equal(
            received_unit.as_slice(),
            requested_unit.as_slice(),
            3,
            1e-6,
            &format!("{what} torque direction"),
        );
    }

    /// Upstream `test_thrForceMap_configErr.py::test_module`: DV and RCS
    /// clusters with zero to two dropped thrusters, zero or one control axis,
    /// and with or without a thruster of zero maximum thrust. Upstream expects
    /// a `BasiliskError` from reset when there is no control axis or a bad
    /// thruster; otherwise the forces must match the truth within 1e-6.
    #[test]
    fn matches_upstream_thr_force_map_config_err_test() {
        for use_dv_thruster in [true, false] {
            for drop_thruster in [0, 1, 2] {
                for num_control_axis in [0, 1] {
                    for misconfig_thruster in [true, false] {
                        let scenario = Scenario {
                            use_dv_thruster,
                            use_com_offset: false,
                            drop_thruster,
                            use_2nd_loop: false,
                            num_control_axis,
                            saturate_thrusters: 0,
                            misconfig_thruster,
                        };
                        let what = format!("{scenario:?}");
                        let mut fixture = fixture(scenario);
                        // What upstream's InitializeSimulation runs: reset.
                        let errors = fixture.module.configure();
                        assert_eq!(
                            errors.contains(&ResetError::NoControlAxes),
                            num_control_axis == 0,
                            "{what}: {errors:?}"
                        );
                        assert_eq!(
                            errors.contains(&ResetError::NonPositiveMaxThrust { thruster: 0 }),
                            misconfig_thruster,
                            "{what}: {errors:?}"
                        );
                        let expect_error = misconfig_thruster || num_control_axis == 0;
                        assert_eq!(errors.is_empty(), !expect_error, "{what}: {errors:?}");
                        if expect_error {
                            continue;
                        }

                        fixture.module.update(&context(0));
                        let output = fixture.module.thr_force_cmd_out_msg.read().thrust_force_n;
                        let truth = upstream_truth(&fixture, false);
                        assert_array_equal(&output, &truth, fixture.num_thrusters, 1e-6, &what);
                    }
                }
            }
        }
    }

    #[test]
    fn reset_normalizes_the_leading_control_axes_in_place() {
        let mut module = ThrForceMapping::new(ThrForceMappingConfig {
            control_axes_b: [
                Vector3::new(3.0, 0.0, 4.0),
                Vector3::zeros(),
                // Ignored: it follows the first axis that is too short.
                Vector3::new(0.0, 2.0, 0.0),
            ],
            thr_force_sign: ThrForceSign::Positive,
            ..Default::default()
        });
        let errors = module.configure();
        assert_eq!(module.num_control_axes(), 1);
        // Scaled by 1/5 as upstream, so 0.6 is off by one ulp.
        assert!((module.config.control_axes_b[0] - Vector3::new(0.6, 0.0, 0.8)).norm() <= 1e-15);
        assert_eq!(module.config.control_axes_b[2], Vector3::new(0.0, 2.0, 0.0));
        assert_eq!(
            errors,
            [
                ResetError::InputNotConnected("thr_config_in_msg"),
                ResetError::InputNotConnected("veh_config_in_msg"),
                ResetError::InputNotConnected("cmd_torque_in_msg"),
            ]
        );
    }

    /// Upstream quirk: reset only overwrites a thruster's saturation limit
    /// with a positive value, so a limit that drops to zero keeps the old one.
    #[test]
    fn keeps_the_previous_limit_of_a_thruster_whose_limit_becomes_non_positive() {
        let thruster = |max_thrust_n| ThrusterConfigMsg {
            position_thrust_b_m: Vector3::new(0.0, 1.0, 0.0),
            t_hat_thrust_b: Vector3::new(1.0, 0.0, 0.0),
            max_thrust_n,
            swirl_torque_nm: 0.0,
        };
        let cluster = Output::new(ThrusterArrayConfigMsg::from_active(&[thruster(2.0)]));
        let mut module = ThrForceMapping::new(ThrForceMappingConfig {
            control_axes_b: [Vector3::z(), Vector3::zeros(), Vector3::zeros()],
            thr_force_sign: ThrForceSign::Positive,
            ..Default::default()
        });
        cluster.connect_to(&mut module.thr_config_in_msg);
        module.configure();
        assert_eq!(module.thruster_max_thrust_n()[0], 2.0);

        cluster.write(ThrusterArrayConfigMsg::from_active(&[thruster(0.0)]));
        assert_eq!(
            module.configure(),
            [
                ResetError::InputNotConnected("veh_config_in_msg"),
                ResetError::InputNotConnected("cmd_torque_in_msg"),
                ResetError::NonPositiveMaxThrust { thruster: 0 },
            ]
        );
        assert_eq!(module.thruster_max_thrust_n()[0], 2.0);
    }

    /// One thruster at +y firing along +x makes a -z torque, so a +z request
    /// cannot be met by on-pulsing and a -z request is met exactly. The center
    /// of mass is read on every update, not only at reset.
    #[test]
    fn maps_a_single_axis_request_and_follows_the_center_of_mass() {
        let cluster = Output::new(ThrusterArrayConfigMsg::from_active(&[ThrusterConfigMsg {
            position_thrust_b_m: Vector3::new(0.0, 1.0, 0.0),
            t_hat_thrust_b: Vector3::new(1.0, 0.0, 0.0),
            max_thrust_n: 10.0,
            swirl_torque_nm: 0.0,
        }]));
        let vehicle = Output::new(VehicleConfigMsg::default());
        let torque = Output::new(BodyTorqueCommandMsg::default());
        let mut module = ThrForceMapping::new(ThrForceMappingConfig {
            control_axes_b: [Vector3::z(), Vector3::zeros(), Vector3::zeros()],
            thr_force_sign: ThrForceSign::Positive,
            ..Default::default()
        });
        cluster.connect_to(&mut module.thr_config_in_msg);
        vehicle.connect_to(&mut module.veh_config_in_msg);
        torque.connect_to(&mut module.cmd_torque_in_msg);
        module.init();

        let request = |z: f64| BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::new(0.0, 0.0, z),
        };
        torque.write(request(-2.0));
        module.update(&context(0));
        // Arm (0, 1, 0) x (1, 0, 0) = -z: 2 N.
        assert_eq!(module.thr_force_cmd_out_msg.read().thrust_force_n[0], 2.0);

        // With the center of mass at y = 0.5 the arm halves.
        vehicle.write(VehicleConfigMsg {
            center_of_mass_body_m: Vector3::new(0.0, 0.5, 0.0),
            ..Default::default()
        });
        module.update(&context(500_000_000));
        assert_eq!(module.thr_force_cmd_out_msg.read().thrust_force_n[0], 4.0);

        // A +z request asks for -4 N, which the minimum subtraction lifts to 0.
        torque.write(request(2.0));
        module.update(&context(1_000_000_000));
        assert_eq!(module.thr_force_cmd_out_msg.read().thrust_force_n[0], 0.0);
    }
}
