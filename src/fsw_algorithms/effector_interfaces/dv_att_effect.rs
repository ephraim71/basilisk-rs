//! Torque-to-thruster on-time mapping for groups of thrusters: port of
//! upstream Basilisk `dvAttEffect`
//! (`fswAlgorithms/effectorInterfaces/errorConversion/dvAttEffect.c`).
//!
//! Each thruster group maps the commanded body torque linearly onto on-times
//! (`thr_on_map`), adds a nominal on-time, drops requests below a minimum and
//! commands only the `max_num_cmds` longest on-times, publishing one on-time
//! message per group. The per-group mapping is shared with
//! [`super::sun_safe_acs`], as upstream shares `computeSingleThrustBlock`.

use nalgebra::Vector3;

use crate::messages::{
    BodyTorqueCommandMsg, Input, MAX_EFF_COUNT, Output, ThrusterArrayOnTimeCommandMsg,
};
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
    /// Upstream `computeSingleThrustBlock`: the group's on-time command for the
    /// requested body torque.
    pub(crate) fn on_time_command(
        &self,
        torque_request_body_nm: Vector3<f64>,
    ) -> ThrusterArrayOnTimeCommandMsg {
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
        let mut command = ThrusterArrayOnTimeCommandMsg::default();
        for pair in pairs.iter().take(self.max_num_cmds) {
            command.on_time_request_s[pair.thrust_index] = pair.on_time_s;
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
    /// creates one output per group.
    pub config: DvAttEffectConfig,
    /// Upstream `cmdTorqueBodyInMsg`.
    pub cmd_torque_body_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `thrGroups[i].thrOnTimeOutMsg`: one on-time command per
    /// configured group, created by [`DvAttEffect::new`].
    pub thr_on_time_out_msgs: Vec<Output<ThrusterArrayOnTimeCommandMsg>>,
}

impl DvAttEffect {
    /// Builds the module with one output per configured thruster group (at most
    /// [`MAX_NUM_THR_GROUPS`]; further groups are ignored with an error).
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
            thr_on_time_out_msgs: (0..num_groups).map(|_| Output::default()).collect(),
        }
    }

    /// Upstream `Reset_dvAttEffect`: check the input and zero every group's
    /// output.
    pub fn reset(&mut self, _call_time_nanos: u64) {
        if !self.cmd_torque_body_in_msg.is_connected() {
            log::error!("dvAttEffect.cmd_torque_body_in_msg is not connected");
        }
        for (group, output) in self
            .config
            .thr_groups
            .iter()
            .zip(&self.thr_on_time_out_msgs)
        {
            group.check_capacity("dvAttEffect");
            output.write(ThrusterArrayOnTimeCommandMsg::default());
        }
    }
}

impl Module for DvAttEffect {
    fn init(&mut self) {
        for output in &self.thr_on_time_out_msgs {
            output.write(ThrusterArrayOnTimeCommandMsg::default());
        }
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        DvAttEffect::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, _context: &SimulationContext) {
        let torque_request_body_nm = self.cmd_torque_body_in_msg.read().torque_request_body_nm;
        for (group, output) in self
            .config
            .thr_groups
            .iter()
            .zip(&self.thr_on_time_out_msgs)
        {
            output.write(group.on_time_command(torque_request_body_nm));
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

    use crate::messages::{BodyTorqueCommandMsg, MAX_EFF_COUNT, Output};
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

    fn assert_on_times(actual: &[f64; MAX_EFF_COUNT], expected: &[f64; MAX_EFF_COUNT]) {
        for (index, (actual, expected)) in actual.iter().zip(expected).enumerate() {
            assert!(
                (actual - expected).abs() <= 1e-12,
                "thruster {index}: expected {expected}, got {actual}"
            );
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
        module.init();
        module.update(&context(500_000_000));

        // RCS: 0.3 + 0.1, -0.3 + 0.1 -> 0, -0.2 + 0.1 -> 0, 0.2 + 0.1; the
        // two longest are thrusters 0 and 3.
        assert_on_times(
            &module.thr_on_time_out_msgs[0].read().on_time_request_s,
            &expected(&[(0, 0.4), (3, 0.30000000000000004)]),
        );
        // DV: 0.5, -0.25 (below the zero minimum, so zeroed), 1.0.
        assert_on_times(
            &module.thr_on_time_out_msgs[1].read().on_time_request_s,
            &expected(&[(0, 0.5), (2, 1.0)]),
        );
    }

    #[test]
    fn commands_only_existing_effectors_when_max_num_cmds_exceeds_them() {
        let group = rcs_group(10);
        let command = group.on_time_command(Vector3::new(0.3, -0.2, 0.0));
        assert_on_times(
            &command.on_time_request_s,
            &expected(&[(0, 0.4), (3, 0.30000000000000004)]),
        );
    }

    #[test]
    fn ties_keep_the_lower_thruster_index() {
        let group = ThrustGroupConfig {
            max_num_cmds: 1,
            thr_on_map: vec![Vector3::z(), Vector3::z(), Vector3::z()],
            ..Default::default()
        };
        let command = group.on_time_command(Vector3::new(0.0, 0.0, 0.7));
        assert_eq!(command.on_time_request_s, expected(&[(0, 0.7)]));
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
        module.init();
        module.update(&context(500_000_000));
        assert!(
            module.thr_on_time_out_msgs[1]
                .read()
                .on_time_request_s
                .iter()
                .any(|on_time| *on_time != 0.0)
        );

        module.reset(1_000_000_000);
        for output in &module.thr_on_time_out_msgs {
            assert_eq!(output.read().on_time_request_s, [0.0; MAX_EFF_COUNT]);
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
