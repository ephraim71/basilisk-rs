//! Torque-to-thruster on-time mapping for groups of thrusters: port of
//! upstream Basilisk `dvAttEffect`
//! (`fswAlgorithms/effectorInterfaces/errorConversion/dvAttEffect.c`).
//!
//! Each thruster group maps the commanded body torque linearly onto on-times
//! (`thr_on_map`), adds a nominal on-time, drops requests below a minimum and
//! commands only the `max_num_cmds` longest on-times, publishing each group's
//! on-times on one output per thruster. The per-group mapping is shared with
//! [`super::sun_safe_acs`], as upstream shares `computeSingleThrustBlock`.

use nalgebra::Vector3;

use crate::messages::{BodyTorqueCommandMsg, Input, MAX_EFF_COUNT, Output, ThrusterCommandMsg};
use crate::{Module, SimulationContext};

/// Upstream `MAX_NUM_THR_GROUPS`: number of thruster groups the module holds.
pub const MAX_NUM_THR_GROUPS: usize = 4;

/// One thruster group (upstream `ThrustGroupData`). Defaults are upstream's
/// zero initialization.
#[derive(Clone, Debug, Default)]
pub struct ThrustGroupConfig {
    /// Upstream `nomThrustOn` \[s\]: on-time added to every mapped request.
    pub nom_thrust_on_s: f64,
    /// Upstream `maxNumCmds` [-]: number of (longest) on-times commanded.
    pub max_num_cmds: usize,
    /// Upstream `minThrustRequest` \[s\]: on-times below this are zeroed.
    pub min_thrust_request_s: f64,
    /// Upstream `thrOnMap` \[s/(N m)\]: one row per effector mapping the body
    /// torque onto its on-time. Its length is upstream `numEffectors` (at
    /// most [`MAX_EFF_COUNT`]; further rows are ignored).
    pub thr_on_map: Vec<Vector3<f64>>,
}

/// An effector's requested on-time (upstream `effPairs`).
#[derive(Clone, Copy, Debug)]
struct EffectorPair {
    on_time_s: f64,
    thrust_index: usize,
}

impl ThrustGroupConfig {
    /// Upstream `computeSingleThrustBlock`: the group's on-time command \[s\]
    /// for the requested body torque, one entry per effector in `thr_on_map`
    /// order (zero past the group's effectors).
    pub(crate) fn on_time_command(
        &self,
        torque_request_body_nm: Vector3<f64>,
    ) -> [f64; MAX_EFF_COUNT] {
        let mut pairs: Vec<EffectorPair> = self
            .thr_on_map
            .iter()
            .take(MAX_EFF_COUNT)
            .enumerate()
            .map(|(thrust_index, row)| {
                let mut on_time_s = row.dot(&torque_request_body_nm) + self.nom_thrust_on_s;
                if on_time_s < self.min_thrust_request_s {
                    on_time_s = 0.0;
                }
                EffectorPair {
                    on_time_s,
                    thrust_index,
                }
            })
            .collect();
        sort_by_decreasing_on_time(&mut pairs);

        // Upstream reads past the sorted effectors (uninitialized memory) when
        // `maxNumCmds > numEffectors`; here only existing effectors are
        // commanded.
        let mut command = [0.0; MAX_EFF_COUNT];
        for pair in pairs.iter().take(self.max_num_cmds) {
            command[pair.thrust_index] = pair.on_time_s;
        }
        command
    }

    /// Logs upstream's array-capacity limit, which upstream does not check.
    pub(crate) fn check_capacity(&self, module: &str) {
        if self.thr_on_map.len() > MAX_EFF_COUNT {
            log::error!(
                "{module}: a thruster group has more than MAX_EFF_COUNT ({MAX_EFF_COUNT}) effectors; ignoring the rest"
            );
        }
    }
}

/// Upstream `effectorVSort`: bubble sort into decreasing on-time, at most one
/// pass per element. Ties keep their order, and since NaN never compares
/// less, a NaN on-time is never moved past its neighbours (a library sort
/// could panic on such an inconsistent order).
fn sort_by_decreasing_on_time(pairs: &mut [EffectorPair]) {
    let mut swapped = true;
    for _ in 0..pairs.len() {
        if !swapped {
            break;
        }
        swapped = false;
        for j in 1..pairs.len() {
            if pairs[j - 1].on_time_s < pairs[j].on_time_s {
                pairs.swap(j - 1, j);
                swapped = true;
            }
        }
    }
}

/// User-settable parameters of [`DvAttEffect`].
#[derive(Clone, Debug)]
pub struct DvAttEffectConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `thrGroups`: the thruster groups; their number is upstream
    /// `numThrGroups` (at most [`MAX_NUM_THR_GROUPS`]).
    pub thr_groups: Vec<ThrustGroupConfig>,
}

impl Default for DvAttEffectConfig {
    fn default() -> Self {
        Self {
            name: "dvAttEffect".to_string(),
            thr_groups: Vec::new(),
        }
    }
}

/// Delta-v attitude effector management (upstream `dvAttEffectConfig`).
#[derive(Clone, Debug)]
pub struct DvAttEffect {
    /// Module configuration. The group parameters are read at every update,
    /// but the number of groups is fixed by [`DvAttEffect::new`], which
    /// creates the output list of every group.
    pub config: DvAttEffectConfig,
    /// Upstream `cmdTorqueBodyInMsg`.
    pub cmd_torque_body_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `thrGroups[i].thrOnTimeOutMsg`: per configured group, the
    /// commanded on-time of each of its thrusters, one output per thruster in
    /// `thr_on_map` order, registered with
    /// [`DvAttEffect::add_thruster_command_output`].
    pub thr_on_time_out_msgs: Vec<Vec<Output<ThrusterCommandMsg>>>,
}

impl DvAttEffect {
    /// Builds the module with an empty output list per configured thruster
    /// group (at most [`MAX_NUM_THR_GROUPS`]; further groups are ignored with
    /// an error).
    pub fn new(config: DvAttEffectConfig) -> Self {
        if config.thr_groups.len() > MAX_NUM_THR_GROUPS {
            log::error!(
                "dvAttEffect supports at most MAX_NUM_THR_GROUPS ({MAX_NUM_THR_GROUPS}) thruster groups; ignoring the rest"
            );
        }
        let num_groups = config.thr_groups.len().min(MAX_NUM_THR_GROUPS);
        Self {
            config,
            cmd_torque_body_in_msg: Input::default(),
            thr_on_time_out_msgs: (0..num_groups).map(|_| Vec::new()).collect(),
        }
    }

    /// Registers one independently connectable thruster command output of
    /// group `group_index`. A group's outputs follow its `thr_on_map` order:
    /// the first one registered carries the on-time of its first effector.
    pub fn add_thruster_command_output(
        &mut self,
        group_index: usize,
    ) -> &Output<ThrusterCommandMsg> {
        assert!(
            group_index < self.thr_on_time_out_msgs.len(),
            "dvAttEffect has no thruster group {group_index}"
        );
        let outputs = &mut self.thr_on_time_out_msgs[group_index];
        assert!(
            outputs.len() < MAX_EFF_COUNT,
            "at most {MAX_EFF_COUNT} thruster command outputs per group are supported"
        );
        outputs.push(Output::default());
        outputs
            .last()
            .expect("registered thruster command output should be available")
    }

    /// Upstream `Reset_dvAttEffect`: check the input and zero every group's
    /// output.
    pub fn reset(&mut self, _call_time_nanos: u64) {
        if !self.cmd_torque_body_in_msg.is_connected() {
            log::error!("dvAttEffect.cmd_torque_body_in_msg is not connected");
        }
        for (group, outputs) in self
            .config
            .thr_groups
            .iter()
            .zip(&self.thr_on_time_out_msgs)
        {
            group.check_capacity("dvAttEffect");
            for output in outputs {
                output.write(ThrusterCommandMsg::default());
            }
        }
    }
}

impl Module for DvAttEffect {
    fn init(&mut self) {
        for output in self.thr_on_time_out_msgs.iter().flatten() {
            output.write(ThrusterCommandMsg::default());
        }
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        DvAttEffect::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, _context: &SimulationContext) {
        let torque_request_body_nm = self.cmd_torque_body_in_msg.read().torque_request_body_nm;
        for (group, outputs) in self
            .config
            .thr_groups
            .iter()
            .zip(&self.thr_on_time_out_msgs)
        {
            let on_times = group.on_time_command(torque_request_body_nm);
            for (index, output) in outputs.iter().enumerate() {
                output.write(ThrusterCommandMsg {
                    on_time_s: on_times[index],
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    //! Upstream's only test (`test_errorConversionImports.py`) checks that the
    //! Python wrapper imports; these tests are written from the algorithm,
    //! with hand-derived truth.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{BodyTorqueCommandMsg, MAX_EFF_COUNT, Output, ThrusterCommandMsg};
    use crate::{Module, SimulationContext};

    use super::{
        DvAttEffect, DvAttEffectConfig, EffectorPair, MAX_NUM_THR_GROUPS, ThrustGroupConfig,
        sort_by_decreasing_on_time,
    };

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn expected(entries: &[(usize, f64)]) -> [f64; MAX_EFF_COUNT] {
        let mut on_times = [0.0; MAX_EFF_COUNT];
        for &(index, on_time_s) in entries {
            on_times[index] = on_time_s;
        }
        on_times
    }

    fn assert_on_times(actual: &[f64], expected: &[f64]) {
        assert_eq!(actual.len(), expected.len(), "number of thrusters");
        for (index, (actual, expected)) in actual.iter().zip(expected).enumerate() {
            assert!(
                (actual - expected).abs() <= 1e-12,
                "thruster {index}: expected {expected}, got {actual}"
            );
        }
    }

    /// The on-times on one group's outputs, in thruster order.
    fn published(outputs: &[Output<ThrusterCommandMsg>]) -> Vec<f64> {
        outputs
            .iter()
            .map(|output| output.read().on_time_s)
            .collect()
    }

    /// Registers one command output per effector of every group.
    fn add_outputs(module: &mut DvAttEffect) {
        for group_index in 0..module.thr_on_time_out_msgs.len() {
            for _ in 0..module.config.thr_groups[group_index].thr_on_map.len() {
                module.add_thruster_command_output(group_index);
            }
        }
    }

    /// Four RCS-like thrusters: +x, -x, +y, -y torque.
    fn rcs_group(max_num_cmds: usize) -> ThrustGroupConfig {
        ThrustGroupConfig {
            nom_thrust_on_s: 0.1,
            max_num_cmds,
            min_thrust_request_s: 0.05,
            thr_on_map: vec![
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(-1.0, 0.0, 0.0),
                Vector3::new(0.0, 1.0, 0.0),
                Vector3::new(0.0, -1.0, 0.0),
            ],
        }
    }

    /// Three DV thrusters reacting to z torque with different gains.
    fn dv_group() -> ThrustGroupConfig {
        ThrustGroupConfig {
            nom_thrust_on_s: 0.0,
            max_num_cmds: 3,
            min_thrust_request_s: 0.0,
            thr_on_map: vec![
                Vector3::new(0.0, 0.0, 2.0),
                Vector3::new(0.0, 0.0, -1.0),
                Vector3::new(0.0, 0.0, 4.0),
            ],
        }
    }

    #[test]
    fn maps_the_torque_onto_the_longest_on_times_of_every_group() {
        let command = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::new(0.3, -0.2, 0.25),
        });
        let mut module = DvAttEffect::new(DvAttEffectConfig {
            thr_groups: vec![rcs_group(2), dv_group()],
            ..Default::default()
        });
        assert_eq!(module.thr_on_time_out_msgs.len(), 2);
        command.connect_to(&mut module.cmd_torque_body_in_msg);
        add_outputs(&mut module);
        module.init();
        module.update(&context(500_000_000));

        // RCS: 0.3 + 0.1, -0.3 + 0.1 -> 0, -0.2 + 0.1 -> 0, 0.2 + 0.1; the
        // two longest are thrusters 0 and 3.
        assert_on_times(
            &published(&module.thr_on_time_out_msgs[0]),
            &expected(&[(0, 0.4), (3, 0.30000000000000004)])[..4],
        );
        // DV: 0.5, -0.25 (below the zero minimum, so zeroed), 1.0.
        assert_on_times(
            &published(&module.thr_on_time_out_msgs[1]),
            &expected(&[(0, 0.5), (2, 1.0)])[..3],
        );
    }

    #[test]
    fn commands_only_existing_effectors_when_max_num_cmds_exceeds_them() {
        let group = rcs_group(10);
        let command = group.on_time_command(Vector3::new(0.3, -0.2, 0.0));
        assert_on_times(&command, &expected(&[(0, 0.4), (3, 0.30000000000000004)]));
    }

    #[test]
    fn ties_keep_the_lower_thruster_index() {
        let group = ThrustGroupConfig {
            max_num_cmds: 1,
            thr_on_map: vec![Vector3::z(), Vector3::z(), Vector3::z()],
            ..Default::default()
        };
        let command = group.on_time_command(Vector3::new(0.0, 0.0, 0.7));
        assert_eq!(command, expected(&[(0, 0.7)]));
    }

    #[test]
    fn bubble_sort_matches_upstream_effector_v_sort() {
        let mut pairs: Vec<EffectorPair> = [0.2, 0.9, 0.2, f64::NAN, 1.5, 0.4]
            .into_iter()
            .enumerate()
            .map(|(thrust_index, on_time_s)| EffectorPair {
                on_time_s,
                thrust_index,
            })
            .collect();
        sort_by_decreasing_on_time(&mut pairs);
        // The NaN at index 3 splits the array: each side is sorted on its own.
        let order: Vec<usize> = pairs.iter().map(|pair| pair.thrust_index).collect();
        assert_eq!(order, [1, 0, 2, 3, 4, 5]);

        let mut empty: Vec<EffectorPair> = Vec::new();
        sort_by_decreasing_on_time(&mut empty);
        assert!(empty.is_empty());
    }

    #[test]
    fn reset_zeros_every_group_output() {
        let command = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::new(0.3, -0.2, 0.25),
        });
        let mut module = DvAttEffect::new(DvAttEffectConfig {
            thr_groups: vec![rcs_group(2), dv_group()],
            ..Default::default()
        });
        command.connect_to(&mut module.cmd_torque_body_in_msg);
        add_outputs(&mut module);
        module.init();
        module.update(&context(500_000_000));
        assert!(
            published(&module.thr_on_time_out_msgs[1])
                .iter()
                .any(|on_time| *on_time != 0.0)
        );

        module.reset(1_000_000_000);
        for outputs in &module.thr_on_time_out_msgs {
            assert!(!outputs.is_empty());
            assert!(published(outputs).iter().all(|on_time| *on_time == 0.0));
        }
    }

    #[test]
    fn holds_at_most_four_groups() {
        let module = DvAttEffect::new(DvAttEffectConfig {
            thr_groups: vec![dv_group(); MAX_NUM_THR_GROUPS + 1],
            ..Default::default()
        });
        assert_eq!(module.thr_on_time_out_msgs.len(), MAX_NUM_THR_GROUPS);
    }
}
