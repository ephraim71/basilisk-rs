//! Body force and torque to thruster force mapping: port of upstream Basilisk
//! `fswAlgorithms/effectorInterfaces/forceTorqueThrForceMapping`.
//!
//! The commanded torque and force are stacked into `[τ; F]` (each optional;
//! an unconnected command is zero). Thruster `i` contributes the column
//! `[d_i; ĝ_i]` with `d_i = (r_i - r_COM) × ĝ_i` to the 6×N matrix `[DG]`.
//! Rows of `[DG]` that are zero (within 1e-7) for every thruster, such as a
//! force axis no thruster points along, are removed, and the forces are the
//! minimum-norm solution `F = [DG]ᵀ([DG][DG]ᵀ)⁻¹ [τ; F]`. The smallest force
//! is then subtracted from all of them so that none is negative. Unlike
//! [`super::thr_force_mapping`] this module only handles on-pulsing thrusters
//! and solves once.
//!
//! Upstream bug, preserved: after zero rows are removed, the minimum-norm
//! inverse is multiplied by the *leading* entries of `[τ; F]` instead of the
//! entries of the rows that were kept. The result is right only when the
//! removed rows are the last ones (for example a missing force-z axis).

use nalgebra::{SMatrix, Vector3, Vector6};

use crate::messages::{
    BodyForceCommandMsg, BodyTorqueCommandMsg, Input, MAX_EFF_COUNT, Output,
    ThrusterArrayConfigMsg, ThrusterArrayForceCommandMsg, VehicleConfigMsg,
};
use crate::utilities::linear_algebra::{is_zero_within, m_minimum_norm_inverse};
use crate::{Module, SimulationContext};

/// Upstream's tolerance for a row of `[DG]` to count as zero.
const ZERO_ROW_TOLERANCE: f64 = 1e-7;

/// User-settable parameters of [`ForceTorqueThrForceMapping`]; upstream has
/// none beyond the instance name.
#[derive(Clone, Debug)]
pub struct ForceTorqueThrForceMappingConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
}

impl Default for ForceTorqueThrForceMappingConfig {
    fn default() -> Self {
        Self {
            name: "forceTorqueThrForceMapping".to_string(),
        }
    }
}

/// Maps a commanded body force and torque onto thruster forces (upstream
/// `forceTorqueThrForceMapping`).
#[derive(Clone, Debug)]
pub struct ForceTorqueThrForceMapping {
    /// User-settable parameters.
    pub config: ForceTorqueThrForceMappingConfig,
    /// Upstream `cmdTorqueInMsg` (optional): commanded body torque.
    pub cmd_torque_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `cmdForceInMsg` (optional): commanded body force.
    pub cmd_force_in_msg: Input<BodyForceCommandMsg>,
    /// Upstream `thrConfigInMsg`: thruster cluster configuration, read at
    /// reset.
    pub thr_config_in_msg: Input<ThrusterArrayConfigMsg>,
    /// Upstream `vehConfigInMsg`: vehicle configuration; its center of mass is
    /// read at reset.
    pub veh_config_in_msg: Input<VehicleConfigMsg>,
    /// Upstream `thrForceCmdOutMsg`: commanded thruster forces.
    pub thr_force_cmd_out_msg: Output<ThrusterArrayForceCommandMsg>,
    /// Upstream `rThruster_B` \[m\]: thruster locations, body frame.
    positions_b_m: Vec<Vector3<f64>>,
    /// Upstream `gtThruster_B` \[-\]: thrust unit directions, body frame.
    directions_b: Vec<Vector3<f64>>,
    /// Upstream `CoM_B` \[m\]: center of mass, body frame.
    center_of_mass_b_m: Vector3<f64>,
}

impl ForceTorqueThrForceMapping {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: ForceTorqueThrForceMappingConfig) -> Self {
        Self {
            config,
            cmd_torque_in_msg: Input::default(),
            cmd_force_in_msg: Input::default(),
            thr_config_in_msg: Input::default(),
            veh_config_in_msg: Input::default(),
            thr_force_cmd_out_msg: Output::default(),
            positions_b_m: Vec::new(),
            directions_b: Vec::new(),
            center_of_mass_b_m: Vector3::zeros(),
        }
    }

    /// Upstream `Reset_forceTorqueThrForceMapping`: reads the thruster cluster
    /// and the center of mass, and publishes zero forces so that a disabled
    /// module leaves the thrusters off.
    pub fn reset(&mut self, _call_time_nanos: u64) {
        if !self.thr_config_in_msg.is_connected() {
            log::error!("forceTorqueThrForceMapping.thr_config_in_msg is not connected");
        }
        if !self.veh_config_in_msg.is_connected() {
            log::error!("forceTorqueThrForceMapping.veh_config_in_msg is not connected");
        }

        let cluster = self.thr_config_in_msg.read();
        self.center_of_mass_b_m = self.veh_config_in_msg.read().center_of_mass_body_m;
        if cluster.num_thrusters > MAX_EFF_COUNT {
            log::error!(
                "forceTorqueThrForceMapping: the thruster configuration has more than \
                 MAX_EFF_COUNT ({MAX_EFF_COUNT}) thrusters"
            );
        }
        let thrusters = cluster.active_thrusters();
        self.positions_b_m = thrusters.iter().map(|t| t.position_thrust_b_m).collect();
        self.directions_b = thrusters.iter().map(|t| t.t_hat_thrust_b).collect();
        for (index, thruster) in thrusters.iter().enumerate() {
            if thruster.max_thrust_n <= 0.0 {
                log::error!(
                    "forceTorqueThrForceMapping: thruster {index} has a non-sensible \
                     saturation limit of <= 0 N"
                );
            }
        }

        self.thr_force_cmd_out_msg
            .write(ThrusterArrayForceCommandMsg::default());
    }

    /// Upstream `Update_forceTorqueThrForceMapping`.
    fn compute_thruster_forces(&self) -> ThrusterArrayForceCommandMsg {
        let torque_nm = if self.cmd_torque_in_msg.is_connected() {
            self.cmd_torque_in_msg.read().torque_request_body_nm
        } else {
            Vector3::zeros()
        };
        let force_n = if self.cmd_force_in_msg.is_connected() {
            self.cmd_force_in_msg.read().force_request_body_n
        } else {
            Vector3::zeros()
        };
        let torque_force = Vector6::new(
            torque_nm.x,
            torque_nm.y,
            torque_nm.z,
            force_n.x,
            force_n.y,
            force_n.z,
        );

        let num_thrusters = self.positions_b_m.len();
        // Columns past the installed thrusters stay zero, as upstream's
        // MAX_EFF_COUNT-wide matrix.
        let mut mapping = SMatrix::<f64, 6, MAX_EFF_COUNT>::zeros();
        for (column, (position, direction)) in self
            .positions_b_m
            .iter()
            .zip(&self.directions_b)
            .enumerate()
        {
            let torque_arm = (position - self.center_of_mass_b_m).cross(direction);
            mapping
                .fixed_view_mut::<3, 1>(0, column)
                .copy_from(&torque_arm);
            mapping
                .fixed_view_mut::<3, 1>(3, column)
                .copy_from(direction);
        }

        let kept_rows: Vec<usize> = (0..6)
            .filter(|&row| {
                !is_zero_within(
                    &mapping.view((row, 0), (1, num_thrusters)),
                    ZERO_ROW_TOLERANCE,
                )
            })
            .collect();
        // MAX_EFF_COUNT x kept_rows.len()
        let inverse = m_minimum_norm_inverse(&mapping.select_rows(&kept_rows));

        // Upstream bug, kept: the leading entries of [τ; F] are used, not the
        // entries of the kept rows.
        let forces_n: Vec<f64> = (0..num_thrusters)
            .map(|thruster| {
                (0..kept_rows.len())
                    .fold(0.0, |sum, k| sum + inverse[(thruster, k)] * torque_force[k])
            })
            .collect();

        // Upstream starts the minimum search at the first force, so the
        // smallest force always becomes zero.
        let min_force_n = forces_n.iter().skip(1).fold(
            forces_n.first().copied().unwrap_or(0.0),
            |min, &force| {
                if force < min { force } else { min }
            },
        );
        let shifted_n: Vec<f64> = forces_n.iter().map(|force| force - min_force_n).collect();
        ThrusterArrayForceCommandMsg::from_active(&shifted_n)
    }
}

impl Module for ForceTorqueThrForceMapping {
    fn init(&mut self) {
        self.thr_force_cmd_out_msg
            .write(ThrusterArrayForceCommandMsg::default());
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        ForceTorqueThrForceMapping::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, _context: &SimulationContext) {
        let forces = self.compute_thruster_forces();
        self.thr_force_cmd_out_msg.write(forces);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Vector3, Vector6};

    use crate::messages::{
        BodyForceCommandMsg, BodyTorqueCommandMsg, MAX_EFF_COUNT, Output, ThrusterArrayConfigMsg,
        ThrusterConfigMsg, VehicleConfigMsg,
    };
    use crate::{Module, SimulationContext};

    use super::{ForceTorqueThrForceMapping, ForceTorqueThrForceMappingConfig};

    const MAX_THRUST_N: f64 = 3.0;
    const TOLERANCE: f64 = 1e-3;

    /// Eight RCS thrusters, none of which points along z.
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

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn torque_msg(torque_nm: [f64; 3]) -> BodyTorqueCommandMsg {
        BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::from(torque_nm),
        }
    }

    fn force_msg(force_n: [f64; 3]) -> BodyForceCommandMsg {
        BodyForceCommandMsg {
            force_request_body_n: Vector3::from(force_n),
        }
    }

    fn cluster(positions_m: &[[f64; 3]], directions: &[[f64; 3]]) -> ThrusterArrayConfigMsg {
        let thrusters: Vec<_> = positions_m
            .iter()
            .zip(directions)
            .map(|(position, direction)| ThrusterConfigMsg {
                position_thrust_b_m: Vector3::from(*position),
                t_hat_thrust_b: Vector3::from(*direction),
                max_thrust_n: MAX_THRUST_N,
                swirl_torque_nm: 0.0,
            })
            .collect();
        ThrusterArrayConfigMsg::from_active(&thrusters)
    }

    /// The pytest's `calculate_force_torque`: `[τ; F]` of the thruster forces.
    fn torque_and_force(
        forces_n: &[f64],
        positions_m: &[[f64; 3]],
        directions: &[[f64; 3]],
        center_of_mass_m: Vector3<f64>,
    ) -> Vector6<f64> {
        let mut total = Vector6::zeros();
        for ((&force, position), direction) in forces_n.iter().zip(positions_m).zip(directions) {
            let force_vector = force * Vector3::from(*direction);
            let torque = (Vector3::from(*position) - center_of_mass_m).cross(&force_vector);
            total += Vector6::new(
                torque.x,
                torque.y,
                torque.z,
                force_vector.x,
                force_vector.y,
                force_vector.z,
            );
        }
        total
    }

    /// Upstream `forceTorqueThrForceMappingTestFunction`: a 0.5 s run with
    /// the given request, after which every force must lie in
    /// `[0, maxThrust]` (within 1e-10) and reproduce the requested torque and
    /// force within 1e-3 (norm of the difference, as `compareVector`). Then a
    /// nonzero request on both inputs is re-initialized and run, and a final
    /// reset must zero the output.
    fn run_upstream_case(
        positions_m: &[[f64; 3]],
        directions: &[[f64; 3]],
        requested_torque_nm: [f64; 3],
        requested_force_n: [f64; 3],
        center_of_mass_m: [f64; 3],
        torque_in_msg_flag: bool,
    ) {
        let center_of_mass_m = Vector3::from(center_of_mass_m);
        let mut module =
            ForceTorqueThrForceMapping::new(ForceTorqueThrForceMappingConfig::default());
        let torque = Output::new(torque_msg(requested_torque_nm));
        let force = Output::new(force_msg(requested_force_n));
        if torque_in_msg_flag {
            torque.connect_to(&mut module.cmd_torque_in_msg);
        }
        force.connect_to(&mut module.cmd_force_in_msg);
        Output::new(cluster(positions_m, directions)).connect_to(&mut module.thr_config_in_msg);
        Output::new(VehicleConfigMsg {
            center_of_mass_body_m: center_of_mass_m,
            ..Default::default()
        })
        .connect_to(&mut module.veh_config_in_msg);

        module.init();
        for nanos in [0, 500_000_000] {
            module.update(&context(nanos));
        }
        let forces_n = &module.thr_force_cmd_out_msg.read().thrust_force_n[..positions_m.len()];
        for (index, &force_n) in forces_n.iter().enumerate() {
            assert!(
                force_n >= -1e-10,
                "thruster {index} has negative force {force_n}"
            );
            assert!(
                force_n <= MAX_THRUST_N + 1e-10,
                "thruster {index} exceeds the max thrust: {force_n}"
            );
        }
        let requested = Vector6::new(
            requested_torque_nm[0],
            requested_torque_nm[1],
            requested_torque_nm[2],
            requested_force_n[0],
            requested_force_n[1],
            requested_force_n[2],
        );
        let actual = torque_and_force(forces_n, positions_m, directions, center_of_mass_m);
        assert!(
            (actual - requested).norm() <= TOLERANCE,
            "requested {requested}, produced {actual}"
        );

        let torque = Output::new(torque_msg([1.0, 0.5, 0.7]));
        torque.connect_to(&mut module.cmd_torque_in_msg);
        let force = Output::new(force_msg([1.0, 0.5, 0.7]));
        force.connect_to(&mut module.cmd_force_in_msg);
        module.init();
        for nanos in [0, 500_000_000] {
            module.update(&context(nanos));
        }
        module.reset(0);
        let output = module.thr_force_cmd_out_msg.read().thrust_force_n;
        assert_eq!(output.len(), MAX_EFF_COUNT);
        assert!(output.iter().map(|f| f * f).sum::<f64>().sqrt() <= TOLERANCE);
    }

    /// Upstream `test_forceTorqueThrForceMapping1`: no thruster points along
    /// z (the force-z row is removed) and the center of mass is offset.
    #[test]
    fn matches_upstream_force_torque_thr_force_mapping_test_1() {
        run_upstream_case(
            &RCS_POSITIONS_M,
            &RCS_DIRECTIONS,
            [0.4, 0.2, 0.4],
            [0.9, 1.1, 0.0],
            [0.1, 0.1, 0.1],
            true,
        );
    }

    /// Upstream `test_forceTorqueThrForceMapping2`: a connected zero torque
    /// request with a nonzero force request.
    #[test]
    fn matches_upstream_force_torque_thr_force_mapping_test_2() {
        run_upstream_case(
            &RCS_POSITIONS_M,
            &RCS_DIRECTIONS,
            [0.0, 0.0, 0.0],
            [0.9, 1.1, 0.0],
            [0.1, 0.1, 0.1],
            true,
        );
    }

    /// Upstream `test_forceTorqueThrForceMapping3`: no torque input message at
    /// all, so the torque request reads as zero.
    #[test]
    fn matches_upstream_force_torque_thr_force_mapping_test_3() {
        run_upstream_case(
            &RCS_POSITIONS_M,
            &RCS_DIRECTIONS,
            [0.0, 0.0, 0.0],
            [0.9, 1.1, 0.0],
            [0.1, 0.1, 0.1],
            false,
        );
    }

    /// Upstream `test_forceTorqueThrForceMapping4`: twelve thrusters that
    /// cover every direction, so no row is removed.
    #[test]
    fn matches_upstream_force_torque_thr_force_mapping_test_4() {
        let positions_m = [
            [-1.0, -1.0, 1.0],
            [-1.0, -1.0, 1.0],
            [-1.0, -1.0, 1.0],
            [1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
            [1.0, 1.0, -1.0],
            [1.0, 1.0, -1.0],
            [1.0, 1.0, -1.0],
            [-1.0, -1.0, -1.0],
            [-1.0, -1.0, -1.0],
            [-1.0, -1.0, -1.0],
        ];
        let directions = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, 0.0, -1.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        run_upstream_case(
            &positions_m,
            &directions,
            [0.0, 0.0, 0.0],
            [0.9, 1.1, 1.0],
            [0.1, 0.1, 0.1],
            true,
        );
    }

    /// Upstream bug, kept: four thrusters in the xy plane leave only the
    /// τ_z, F_x and F_y rows of `[DG]`. Every torque arm is +z, so the kept
    /// rows are `[1 1 1 1; 0 0 -1 1; 1 -1 0 0]` and the minimum-norm inverse
    /// has the columns `[1 1 1 1]/4`, `[0 0 -1 1]/2` and `[1 -1 0 0]/2`. Upstream
    /// multiplies them by `(τ_x, τ_y, τ_z)` instead of `(τ_z, F_x, F_y)`: a
    /// pure yaw request of 1 N·m is mapped as an F_y request,
    /// `[0.5, -0.5, 0, 0]`, shifted by its minimum to `[1, 0, 0.5, 0.5]`.
    #[test]
    fn upstream_bug_pairs_kept_rows_with_the_leading_command_entries() {
        let positions_m = [
            [1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
        ];
        let directions = [
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
        ];
        let mut module =
            ForceTorqueThrForceMapping::new(ForceTorqueThrForceMappingConfig::default());
        let torque = Output::new(torque_msg([0.0, 0.0, 1.0]));
        torque.connect_to(&mut module.cmd_torque_in_msg);
        Output::new(cluster(&positions_m, &directions)).connect_to(&mut module.thr_config_in_msg);
        Output::new(VehicleConfigMsg::default()).connect_to(&mut module.veh_config_in_msg);
        module.init();
        module.update(&context(0));

        let output = module.thr_force_cmd_out_msg.read().thrust_force_n;
        assert_eq!(output[..4], [1.0, 0.0, 0.5, 0.5]);
        assert_eq!(output[4..], [0.0; MAX_EFF_COUNT - 4]);
        // What the thrusters actually produce: F_y = 1 and τ_z = 2.
        let produced = torque_and_force(&output[..4], &positions_m, &directions, Vector3::zeros());
        assert_eq!(produced, Vector6::new(0.0, 0.0, 2.0, 0.0, 1.0, 0.0));
    }

    #[test]
    fn reads_the_center_of_mass_only_at_reset() {
        let center_of_mass_m = Vector3::new(0.1, 0.1, 0.1);
        let vehicle = Output::new(VehicleConfigMsg::default());
        let mut module =
            ForceTorqueThrForceMapping::new(ForceTorqueThrForceMappingConfig::default());
        Output::new(torque_msg([0.4, 0.2, 0.4])).connect_to(&mut module.cmd_torque_in_msg);
        Output::new(force_msg([0.9, 1.1, 0.0])).connect_to(&mut module.cmd_force_in_msg);
        Output::new(cluster(&RCS_POSITIONS_M, &RCS_DIRECTIONS))
            .connect_to(&mut module.thr_config_in_msg);
        vehicle.connect_to(&mut module.veh_config_in_msg);
        module.init();
        module.update(&context(0));
        let centered = module.thr_force_cmd_out_msg.read().thrust_force_n;

        vehicle.write(VehicleConfigMsg {
            center_of_mass_body_m: center_of_mass_m,
            ..Default::default()
        });
        module.update(&context(500_000_000));
        assert_eq!(module.thr_force_cmd_out_msg.read().thrust_force_n, centered);

        module.reset(500_000_000);
        module.update(&context(1_000_000_000));
        let offset = module.thr_force_cmd_out_msg.read().thrust_force_n;
        assert_ne!(offset, centered);
        // The new forces implement the request about the new center of mass.
        let produced = torque_and_force(
            &offset[..8],
            &RCS_POSITIONS_M,
            &RCS_DIRECTIONS,
            center_of_mass_m,
        );
        assert!((produced - Vector6::new(0.4, 0.2, 0.4, 0.9, 1.1, 0.0)).norm() <= 1e-12);
    }
}
