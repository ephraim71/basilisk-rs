//! Thruster-based reaction-wheel desaturation: port of upstream Basilisk
//! `thrustRWDesat` (`fswAlgorithms/effectorInterfaces/thrustRWDesat`).
//!
//! The momentum to remove is the negative sum of the wheel spin vectors
//! (`-Σ Ω_i ĝ_s,i`, in wheel-speed units). Once per firing period the module
//! fires the single thruster whose torque arm best matches the current
//! direction of momentum reduction, for an on-time equal to that projection
//! (capped at `max_firing_s`). Thrusters are penalized by the impulse already
//! fired along their thrust direction, so the firings do not keep pushing the
//! vehicle one way. The direction of momentum reduction is re-chosen (as the
//! torque direction of the thruster best matching the momentum to remove) only
//! once the momentum along it has dropped to `dm_thresh_radps`, and
//! desaturation stops when the momentum to remove is below `dm_thresh_radps`.
//!
//! Upstream quirks kept: the direction of momentum reduction is never reset
//! (Reset keeps the last one), and an update that is inside the firing
//! cool-down or below the threshold writes no output, so the previous command
//! stays on the message.

use nalgebra::Vector3;

use crate::messages::{
    Input, MAX_EFF_COUNT, Output, RwConstellationMsg, RwSpeedMsg, ThrusterArrayConfigMsg,
    ThrusterArrayOnTimeCommandMsg, VehicleConfigMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`ThrustRwDesat`]. Defaults are upstream's zero
/// initialization.
#[derive(Clone, Debug)]
pub struct ThrustRwDesatConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `maxFiring` \[s\]: longest on-time of a single firing.
    pub max_firing_s: f64,
    /// Upstream `thrFiringPeriod` \[s\]: rest time after a firing before the
    /// next one may be commanded.
    pub thr_firing_period_s: f64,
    /// Upstream `DMThresh` \[rad/s\]: summed wheel speed below which
    /// desaturation stops, and below which the direction of momentum
    /// reduction may be re-chosen.
    pub dm_thresh_radps: f64,
}

impl Default for ThrustRwDesatConfig {
    fn default() -> Self {
        Self {
            name: "thrustRWDesat".to_string(),
            max_firing_s: 0.0,
            thr_firing_period_s: 0.0,
            dm_thresh_radps: 0.0,
        }
    }
}

/// Thruster-based wheel desaturation logic (upstream `thrustRWDesatConfig`).
#[derive(Clone, Debug)]
pub struct ThrustRwDesat {
    /// Module configuration, read at every reset and update.
    pub config: ThrustRwDesatConfig,
    /// Upstream `rwSpeedInMsg`.
    pub rw_speed_in_msg: Input<RwSpeedMsg>,
    /// Upstream `rwConfigInMsg`: wheel spin axes, read at reset.
    pub rw_config_in_msg: Input<RwConstellationMsg>,
    /// Upstream `thrConfigInMsg`: thruster cluster, read at reset.
    pub thr_config_in_msg: Input<ThrusterArrayConfigMsg>,
    /// Upstream `vecConfigInMsg`: center of mass, read at reset.
    pub vec_config_in_msg: Input<VehicleConfigMsg>,
    /// Upstream `thrCmdOutMsg`.
    pub thr_cmd_out_msg: Output<ThrusterArrayOnTimeCommandMsg>,
    /// Upstream `currDMDir` [-]: current direction of momentum reduction.
    /// As upstream, this one field is both the user-set initial direction
    /// (zero by default; set it before the first update) and the module
    /// state: Reset leaves it unchanged.
    pub curr_dm_dir: Vector3<f64>,
    /// Upstream `accumulatedImp` \[s\]: sum of the fired on-times times the
    /// thrust directions, body frame.
    pub accumulated_imp: Vector3<f64>,
    /// Upstream `totalAccumFiring` \[s\]: total on-time commanded since reset.
    pub total_accum_firing_s: f64,
    /// Upstream `rwAlignMap`: wheel spin axes, body frame.
    rw_align_map: Vec<Vector3<f64>>,
    thrusters: Vec<DesatThruster>,
    /// Upstream `previousFiring` \[ns\].
    previous_firing_nanos: u64,
}

/// One thruster's geometry cached at reset.
#[derive(Clone, Copy, Debug)]
struct DesatThruster {
    /// Upstream `thrAlignMap` entry: thrust direction, body frame.
    direction_b: Vector3<f64>,
    /// Upstream `thrTorqueMap` entry: torque per unit thrust about the center
    /// of mass \[m\], body frame.
    torque_b_m: Vector3<f64>,
}

impl ThrustRwDesat {
    /// Builds the module with unconnected inputs and a zero direction of
    /// momentum reduction.
    pub fn new(config: ThrustRwDesatConfig) -> Self {
        Self {
            config,
            rw_speed_in_msg: Input::default(),
            rw_config_in_msg: Input::default(),
            thr_config_in_msg: Input::default(),
            vec_config_in_msg: Input::default(),
            thr_cmd_out_msg: Output::default(),
            curr_dm_dir: Vector3::zeros(),
            accumulated_imp: Vector3::zeros(),
            total_accum_firing_s: 0.0,
            rw_align_map: Vec::new(),
            thrusters: Vec::new(),
            previous_firing_nanos: 0,
        }
    }

    /// Upstream `Reset_thrustRWDesat`: cache the wheel and thruster geometry,
    /// clear the firing history and zero the output.
    pub fn reset(&mut self, _call_time_nanos: u64) {
        if !self.rw_config_in_msg.is_connected() {
            log::error!("thrustRWDesat.rw_config_in_msg is not connected");
        }
        if !self.vec_config_in_msg.is_connected() {
            log::error!("thrustRWDesat.vec_config_in_msg is not connected");
        }
        if !self.thr_config_in_msg.is_connected() {
            log::error!("thrustRWDesat.thr_config_in_msg is not connected");
        }
        if !self.rw_speed_in_msg.is_connected() {
            log::error!("thrustRWDesat.rw_speed_in_msg is not connected");
        }

        let rw_config = self.rw_config_in_msg.read();
        let vehicle_config = self.vec_config_in_msg.read();
        let thruster_config = self.thr_config_in_msg.read();
        if rw_config.num_rw > MAX_EFF_COUNT {
            log::error!("thrustRWDesat.num_rw exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); truncating");
        }
        if thruster_config.num_thrusters > MAX_EFF_COUNT {
            log::error!(
                "thrustRWDesat.num_thrusters exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); truncating"
            );
        }

        self.rw_align_map = rw_config
            .active_wheels()
            .iter()
            .map(|wheel| wheel.gs_hat_b)
            .collect();
        self.thrusters = thruster_config
            .active_thrusters()
            .iter()
            .map(|thruster| {
                let moment_arm_m =
                    thruster.position_thrust_b_m - vehicle_config.center_of_mass_body_m;
                DesatThruster {
                    direction_b: thruster.t_hat_thrust_b,
                    torque_b_m: moment_arm_m.cross(&thruster.t_hat_thrust_b),
                }
            })
            .collect();

        self.previous_firing_nanos = 0;
        self.accumulated_imp = Vector3::zeros();
        self.total_accum_firing_s = 0.0;
        self.thr_cmd_out_msg
            .write(ThrusterArrayOnTimeCommandMsg::default());
    }

    /// Upstream `Update_thrustRWDesat`; `None` when upstream returns without
    /// writing the output.
    fn compute_firing(&mut self, call_time_nanos: u64) -> Option<ThrusterArrayOnTimeCommandMsg> {
        if diff_nanos_to_seconds(call_time_nanos, self.previous_firing_nanos)
            < self.config.thr_firing_period_s
        {
            return None;
        }

        let wheel_speeds = self.rw_speed_in_msg.read().wheel_speeds_radps;
        let observed_speed = self
            .rw_align_map
            .iter()
            .zip(wheel_speeds)
            .fold(Vector3::zeros(), |sum, (axis, speed)| sum - speed * axis);
        if observed_speed.norm() < self.config.dm_thresh_radps {
            return None;
        }

        // Re-choose the direction of momentum reduction only once the momentum
        // along the current one has been removed.
        if self.curr_dm_dir.dot(&observed_speed) <= self.config.dm_thresh_radps
            && let Some(index) = self.best_thruster(&observed_speed)
        {
            self.curr_dm_dir = normalize_or_zero(self.thrusters[index].torque_b_m);
        }

        let mut output = ThrusterArrayOnTimeCommandMsg::default();
        if let Some(index) = self.best_thruster(&self.curr_dm_dir) {
            let thruster = self.thrusters[index];
            let mut on_time_s = self.curr_dm_dir.dot(&thruster.torque_b_m);
            if on_time_s > self.config.max_firing_s {
                on_time_s = self.config.max_firing_s;
            }
            output.on_time_request_s[index] = on_time_s;
            self.previous_firing_nanos = call_time_nanos;
            self.total_accum_firing_s += on_time_s;
            self.accumulated_imp += on_time_s * thruster.direction_b;
        }
        Some(output)
    }

    /// The thruster whose torque arm projects most onto `direction`, less the
    /// impulse already fired along its thrust direction. Only a positive
    /// projection and a positive match qualify; ties keep the lowest index.
    fn best_thruster(&self, direction: &Vector3<f64>) -> Option<usize> {
        let mut best: Option<(usize, f64)> = None;
        for (index, thruster) in self.thrusters.iter().enumerate() {
            let fire_value = direction.dot(&thruster.torque_b_m);
            let current_match = self.accumulated_imp.dot(&thruster.direction_b);
            let best_match = best.map_or(0.0, |(_, value)| value);
            if fire_value - current_match > best_match && fire_value > 0.0 {
                best = Some((index, fire_value - current_match));
            }
        }
        best.map(|(index, _)| index)
    }
}

impl Module for ThrustRwDesat {
    fn init(&mut self) {
        self.thr_cmd_out_msg
            .write(ThrusterArrayOnTimeCommandMsg::default());
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        ThrustRwDesat::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        if let Some(output) = self.compute_firing(context.current_sim_nanos) {
            self.thr_cmd_out_msg.write(output);
        }
    }
}

#[cfg(test)]
mod tests {
    //! Port of `test_thrustRWDesat.py`, plus tests of the cool-down, the stop
    //! threshold and the firing cap written from the algorithm.
    //!
    //! Upstream runs the sim to 1 s, swaps in faster wheel speeds and calls
    //! `InitializeSimulation` again, which restarts time at 0, runs SelfInit
    //! and Reset (keeping `currDMDir`) and clears the recorder; only the second
    //! run is compared with the first three truth rows (the last two truth rows
    //! are never reached). Here `init()` stands for `InitializeSimulation`,
    //! and the first run is checked against the same rows as well.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{
        MAX_EFF_COUNT, Output, RwConfigElementMsg, RwConstellationMsg, RwSpeedMsg,
        ThrusterArrayConfigMsg, ThrusterConfigMsg, VehicleConfigMsg,
    };
    use crate::{Module, SimulationContext};

    use super::{ThrustRwDesat, ThrustRwDesatConfig};

    const NUM_THRUSTERS: usize = 8;

    const RCS_LOCATIONS_M: [[f64; 3]; NUM_THRUSTERS] = [
        [-0.86360, -0.82550, 1.79070],
        [-0.82550, -0.86360, 1.79070],
        [0.82550, 0.86360, 1.79070],
        [0.86360, 0.82550, 1.79070],
        [-0.86360, -0.82550, -1.79070],
        [-0.82550, -0.86360, -1.79070],
        [0.82550, 0.86360, -1.79070],
        [0.86360, 0.82550, -1.79070],
    ];
    const RCS_DIRECTIONS: [[f64; 3]; NUM_THRUSTERS] = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
    ];

    /// Upstream `trueVector`.
    const TRUTH: [[f64; NUM_THRUSTERS]; 5] = [
        [0., 0., 0., 0., 0., 0., 0., 0.],
        [0.0, 1.97181559e+00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.28062495e+00, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.28062495e+00, 0.0, 0.0],
        [0.0, 1.97181559e+00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ];

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn wheel_speeds(speeds_radps: [f64; 3]) -> RwSpeedMsg {
        let mut message = RwSpeedMsg::default();
        message.wheel_speeds_radps[..3].copy_from_slice(&speeds_radps);
        message
    }

    /// The module and the wheel-speed output feeding it.
    struct Harness {
        module: ThrustRwDesat,
        speeds: Output<RwSpeedMsg>,
    }

    /// The upstream test setup: three orthogonal wheels, eight RCS thrusters
    /// about a center of mass at the body origin, and `currDMDir = [1, 0, 0]`.
    fn upstream_harness(config: ThrustRwDesatConfig, speeds_radps: [f64; 3]) -> Harness {
        let wheels: Vec<_> = [Vector3::x(), Vector3::y(), Vector3::z()]
            .into_iter()
            .map(|gs_hat_b| RwConfigElementMsg {
                gs_hat_b,
                spin_axis_inertia_kg_m2: 0.08,
                max_motor_torque_nm: 0.2,
            })
            .collect();
        let thrusters: Vec<_> = RCS_LOCATIONS_M
            .iter()
            .zip(RCS_DIRECTIONS)
            .map(|(location, direction)| ThrusterConfigMsg {
                position_thrust_b_m: Vector3::from(*location),
                t_hat_thrust_b: Vector3::from(direction),
                max_thrust_n: 2.0,
                swirl_torque_nm: 0.0,
            })
            .collect();
        let speeds = Output::new(wheel_speeds(speeds_radps));
        let wheels = Output::new(RwConstellationMsg::from_active(&wheels));
        let thrusters = Output::new(ThrusterArrayConfigMsg::from_active(&thrusters));
        let vehicle = Output::new(VehicleConfigMsg::default());

        let mut module = ThrustRwDesat::new(config);
        module.curr_dm_dir = Vector3::new(1.0, 0.0, 0.0);
        speeds.connect_to(&mut module.rw_speed_in_msg);
        wheels.connect_to(&mut module.rw_config_in_msg);
        thrusters.connect_to(&mut module.thr_config_in_msg);
        vehicle.connect_to(&mut module.vec_config_in_msg);
        module.init();
        Harness { module, speeds }
    }

    fn upstream_config() -> ThrustRwDesatConfig {
        ThrustRwDesatConfig {
            thr_firing_period_s: 0.5,
            dm_thresh_radps: 20.0,
            max_firing_s: 5.0,
            ..Default::default()
        }
    }

    fn record(module: &mut ThrustRwDesat, times_nanos: &[u64]) -> Vec<[f64; NUM_THRUSTERS]> {
        times_nanos
            .iter()
            .map(|&time| {
                module.update(&context(time));
                let mut row = [0.0; NUM_THRUSTERS];
                row.copy_from_slice(
                    &module.thr_cmd_out_msg.read().on_time_request_s[..NUM_THRUSTERS],
                );
                row
            })
            .collect()
    }

    fn assert_rows_match(actual: &[[f64; NUM_THRUSTERS]], truth: &[[f64; NUM_THRUSTERS]]) {
        for (row, (actual_row, truth_row)) in actual.iter().zip(truth).enumerate() {
            for (thruster, (actual, truth)) in actual_row.iter().zip(truth_row).enumerate() {
                assert!(
                    (actual - truth).abs() <= 1e-6,
                    "sample {row}, thruster {thruster}: expected {truth}, got {actual}"
                );
            }
        }
    }

    #[test]
    fn matches_upstream_thrust_rw_desat_unit_test() {
        let times = [0, 500_000_000, 1_000_000_000];
        let mut harness = upstream_harness(upstream_config(), [20.0, 10.0, 30.0]);
        let first_run = record(&mut harness.module, &times);
        assert_eq!(first_run.len(), 3);
        assert_rows_match(&first_run, &TRUTH[..3]);

        // Faster wheels, then upstream's second InitializeSimulation.
        harness.speeds.write(wheel_speeds([30.0, 20.0, 40.0]));
        harness.module.init();
        let second_run = record(&mut harness.module, &times);
        assert_eq!(second_run.len(), 3);
        assert_rows_match(&second_run, &TRUTH[..second_run.len()]);

        // Upstream's "Reset() zeroed onTime" check reads the first recorded
        // sample; Reset itself must also zero the output message.
        assert!(second_run[0].iter().all(|on_time| on_time.abs() <= 1e-3));
        harness.module.reset(0);
        assert_eq!(
            harness.module.thr_cmd_out_msg.read().on_time_request_s,
            [0.0; MAX_EFF_COUNT]
        );
        assert_eq!(harness.module.accumulated_imp, Vector3::zeros());
        assert_eq!(harness.module.total_accum_firing_s, 0.0);
    }

    #[test]
    fn reset_keeps_the_direction_of_momentum_reduction() {
        let mut harness = upstream_harness(upstream_config(), [20.0, 10.0, 30.0]);
        record(&mut harness.module, &[0, 500_000_000]);
        let direction = harness.module.curr_dm_dir;
        assert_ne!(direction, Vector3::new(1.0, 0.0, 0.0));
        harness.module.reset(0);
        assert_eq!(harness.module.curr_dm_dir, direction);
    }

    #[test]
    fn holds_the_previous_command_during_the_cool_down() {
        let mut harness = upstream_harness(
            ThrustRwDesatConfig {
                thr_firing_period_s: 1.0,
                ..upstream_config()
            },
            [20.0, 10.0, 30.0],
        );
        let samples = record(
            &mut harness.module,
            &[0, 500_000_000, 1_000_000_000, 1_500_000_000],
        );
        // t = 0 is inside the cool-down from the reset time 0; the firing at
        // 1 s stays on the message through 1.5 s.
        assert_eq!(samples[0], [0.0; NUM_THRUSTERS]);
        assert_eq!(samples[1], [0.0; NUM_THRUSTERS]);
        assert!((samples[2][1] - 1.971_815_59).abs() <= 1e-6);
        assert_eq!(samples[3], samples[2]);
    }

    #[test]
    fn stops_below_the_momentum_threshold() {
        let mut harness = upstream_harness(upstream_config(), [2.0, 1.0, 3.0]);
        let samples = record(&mut harness.module, &[0, 500_000_000, 1_000_000_000]);
        assert!(samples.iter().flatten().all(|on_time| *on_time == 0.0));
        assert_eq!(harness.module.total_accum_firing_s, 0.0);
    }

    #[test]
    fn caps_each_firing_at_the_maximum_on_time() {
        let mut harness = upstream_harness(
            ThrustRwDesatConfig {
                max_firing_s: 0.25,
                ..upstream_config()
            },
            [20.0, 10.0, 30.0],
        );
        let samples = record(&mut harness.module, &[0, 500_000_000]);
        assert_eq!(samples[1][1], 0.25);
        assert_eq!(harness.module.total_accum_firing_s, 0.25);
        assert_eq!(harness.module.accumulated_imp, Vector3::new(0.0, 0.25, 0.0));
    }
}
