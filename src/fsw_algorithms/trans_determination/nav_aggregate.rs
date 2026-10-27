//! Navigation message aggregation: port of upstream Basilisk `navAggregate`.
//!
//! The module reads up to [`MAX_AGG_NAV_MSG`] attitude and translational
//! navigation messages and assembles one message of each kind, taking every
//! field from the input the configuration selects: for instance the attitude
//! from one star tracker filter and the rate from another. An index may point
//! past the configured message count, at a slot that is never read and so
//! stays zero, as upstream.
//!
//! Reset clamps message counts above [`MAX_AGG_NAV_MSG`] and indices at or
//! above it, with a warning, writing the clamped values back into the
//! configuration as upstream does.

use crate::messages::{Input, NavigationAttitudeMsg, NavigationTranslationMsg, Output};
use crate::{Module, SimulationContext};

/// Upstream `MAX_AGG_NAV_MSG`: number of input slots of each kind.
pub const MAX_AGG_NAV_MSG: usize = 10;

/// User-settable parameters of [`NavAggregate`]. `Default` is upstream's
/// zeroed configuration: no inputs, every field from slot 0.
#[derive(Clone, Debug)]
pub struct NavAggregateConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `attMsgCount`: number of attitude inputs read, from slot 0.
    pub att_msg_count: usize,
    /// Upstream `transMsgCount`: number of translational inputs read, from
    /// slot 0.
    pub trans_msg_count: usize,
    /// Upstream `attTimeIdx`: attitude slot providing the time tag.
    pub att_time_idx: usize,
    /// Upstream `transTimeIdx`: translational slot providing the time tag.
    pub trans_time_idx: usize,
    /// Upstream `attIdx`: attitude slot providing `sigma_bn`.
    pub att_idx: usize,
    /// Upstream `rateIdx`: attitude slot providing `omega_bn_b_radps`.
    pub rate_idx: usize,
    /// Upstream `posIdx`: translational slot providing the position.
    pub pos_idx: usize,
    /// Upstream `velIdx`: translational slot providing the velocity.
    pub vel_idx: usize,
    /// Upstream `dvIdx`: translational slot providing the accumulated
    /// delta-v.
    pub dv_idx: usize,
    /// Upstream `sunIdx`: attitude slot providing the sun heading.
    pub sun_idx: usize,
}

impl Default for NavAggregateConfig {
    fn default() -> Self {
        Self {
            name: "navAggregate".to_string(),
            att_msg_count: 0,
            trans_msg_count: 0,
            att_time_idx: 0,
            trans_time_idx: 0,
            att_idx: 0,
            rate_idx: 0,
            pos_idx: 0,
            vel_idx: 0,
            dv_idx: 0,
            sun_idx: 0,
        }
    }
}

/// Navigation aggregator: upstream `navAggregate`.
#[derive(Clone, Debug)]
pub struct NavAggregate {
    /// User-settable parameters; reset clamps them in place.
    pub config: NavAggregateConfig,
    /// Upstream `attMsgs[i].navAttInMsg`: [`MAX_AGG_NAV_MSG`] attitude input
    /// slots.
    pub nav_att_in_msgs: Vec<Input<NavigationAttitudeMsg>>,
    /// Upstream `transMsgs[i].navTransInMsg`: [`MAX_AGG_NAV_MSG`]
    /// translational input slots.
    pub nav_trans_in_msgs: Vec<Input<NavigationTranslationMsg>>,
    /// Upstream `navAttOutMsg`: aggregated attitude navigation message.
    pub nav_att_out_msg: Output<NavigationAttitudeMsg>,
    /// Upstream `navTransOutMsg`: aggregated translational navigation message.
    pub nav_trans_out_msg: Output<NavigationTranslationMsg>,
    /// Upstream `attMsgs[i].msgStorage`: last payload read from each slot.
    att_msg_storage: [NavigationAttitudeMsg; MAX_AGG_NAV_MSG],
    /// Upstream `transMsgs[i].msgStorage`: last payload read from each slot.
    trans_msg_storage: [NavigationTranslationMsg; MAX_AGG_NAV_MSG],
}

impl NavAggregate {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: NavAggregateConfig) -> Self {
        Self {
            config,
            nav_att_in_msgs: (0..MAX_AGG_NAV_MSG).map(|_| Input::default()).collect(),
            nav_trans_in_msgs: (0..MAX_AGG_NAV_MSG).map(|_| Input::default()).collect(),
            nav_att_out_msg: Output::default(),
            nav_trans_out_msg: Output::default(),
            att_msg_storage: Default::default(),
            trans_msg_storage: Default::default(),
        }
    }

    /// Upstream `Reset_aggregateNav`: clamps the configuration, checks the
    /// inputs in use and clears the stored payloads.
    fn reset_aggregator(&mut self) {
        let config = &mut self.config;
        clamp_count("attitude message count", &mut config.att_msg_count);
        clamp_count("translation message count", &mut config.trans_msg_count);

        for i in 0..config.att_msg_count {
            if !self.nav_att_in_msgs.get(i).is_some_and(Input::is_connected) {
                log::error!(
                    "navAggregate: an attitude input message was not linked. Be sure that att_msg_count is set properly."
                );
            }
        }
        for i in 0..config.trans_msg_count {
            if !self
                .nav_trans_in_msgs
                .get(i)
                .is_some_and(Input::is_connected)
            {
                log::error!(
                    "navAggregate: a translation input message was not linked. Be sure that trans_msg_count is set properly."
                );
            }
        }

        clamp_index("att_time_idx", &mut config.att_time_idx);
        clamp_index("att_idx", &mut config.att_idx);
        clamp_index("rate_idx", &mut config.rate_idx);
        clamp_index("sun_idx", &mut config.sun_idx);
        clamp_index("trans_time_idx", &mut config.trans_time_idx);
        clamp_index("pos_idx", &mut config.pos_idx);
        clamp_index("vel_idx", &mut config.vel_idx);
        clamp_index("dv_idx", &mut config.dv_idx);

        self.att_msg_storage = Default::default();
        self.trans_msg_storage = Default::default();
    }

    /// The attitude message assembled from the selected slots; zero when no
    /// attitude input is configured.
    fn aggregate_attitude(&mut self) -> NavigationAttitudeMsg {
        let count = self.config.att_msg_count;
        if count == 0 {
            return NavigationAttitudeMsg::default();
        }
        for (storage, input) in self
            .att_msg_storage
            .iter_mut()
            .zip(&self.nav_att_in_msgs)
            .take(count)
        {
            *storage = input.read();
        }
        let storage = &self.att_msg_storage;
        let config = &self.config;
        NavigationAttitudeMsg {
            time_tag_s: storage[config.att_time_idx].time_tag_s,
            sigma_bn: storage[config.att_idx].sigma_bn,
            omega_bn_b_radps: storage[config.rate_idx].omega_bn_b_radps,
            vehicle_sun_point_body: storage[config.sun_idx].vehicle_sun_point_body,
        }
    }

    /// The translational message assembled from the selected slots; zero when
    /// no translational input is configured.
    fn aggregate_translation(&mut self) -> NavigationTranslationMsg {
        let count = self.config.trans_msg_count;
        if count == 0 {
            return NavigationTranslationMsg::default();
        }
        for (storage, input) in self
            .trans_msg_storage
            .iter_mut()
            .zip(&self.nav_trans_in_msgs)
            .take(count)
        {
            *storage = input.read();
        }
        let storage = &self.trans_msg_storage;
        let config = &self.config;
        NavigationTranslationMsg {
            time_tag_s: storage[config.trans_time_idx].time_tag_s,
            position_bn_n_m: storage[config.pos_idx].position_bn_n_m,
            velocity_bn_n_mps: storage[config.vel_idx].velocity_bn_n_mps,
            accumulated_dv_b_mps: storage[config.dv_idx].accumulated_dv_b_mps,
        }
    }
}

/// Caps a message count at [`MAX_AGG_NAV_MSG`], with upstream's warning.
fn clamp_count(what: &str, count: &mut usize) {
    if *count > MAX_AGG_NAV_MSG {
        log::warn!(
            "navAggregate: the {what} {count} is larger than allowed ({MAX_AGG_NAV_MSG}). Setting count to max value."
        );
        *count = MAX_AGG_NAV_MSG;
    }
}

/// Caps a slot index at the last slot, with upstream's warning.
fn clamp_index(name: &str, index: &mut usize) {
    if *index >= MAX_AGG_NAV_MSG {
        log::warn!(
            "navAggregate: the {name} variable {index} is too large. Must be less than {MAX_AGG_NAV_MSG}. Setting index to max value."
        );
        *index = MAX_AGG_NAV_MSG - 1;
    }
}

impl Module for NavAggregate {
    fn init(&mut self) {
        self.nav_att_out_msg.write(NavigationAttitudeMsg::default());
        self.nav_trans_out_msg
            .write(NavigationTranslationMsg::default());
        self.reset_aggregator();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_aggregator();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let attitude = self.aggregate_attitude();
        let translation = self.aggregate_translation();
        self.nav_att_out_msg.write(attitude);
        self.nav_trans_out_msg.write(translation);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{MAX_AGG_NAV_MSG, NavAggregate, NavAggregateConfig};
    use crate::messages::{NavigationAttitudeMsg, NavigationTranslationMsg, Output};
    use crate::{Module, SimulationContext};

    const TASK_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn nav_att(scale: f64, time_tag_s: f64) -> NavigationAttitudeMsg {
        NavigationAttitudeMsg {
            time_tag_s,
            sigma_bn: Vector3::new(0.1, 0.01, -0.1) * scale,
            omega_bn_b_radps: Vector3::new(1.0, 1.0, -1.0) * scale,
            vehicle_sun_point_body: Vector3::new(-0.1, 0.1, 0.1) * scale,
        }
    }

    fn nav_trans(scale: f64, time_tag_s: f64) -> NavigationTranslationMsg {
        NavigationTranslationMsg {
            time_tag_s,
            position_bn_n_m: Vector3::new(1000.0, 100.0, -1000.0) * scale,
            velocity_bn_n_mps: Vector3::new(1.0, 1.0, -1.0) * scale,
            accumulated_dv_b_mps: Vector3::new(-10.1, 10.1, 10.1) * scale,
        }
    }

    fn assert_vector_close(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64) {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!((a - e).abs() <= accuracy, "{actual} != {expected}");
        }
    }

    /// Upstream `navAggregateTestFunction(numAttNav, numTransNav)`. Upstream's
    /// four input payloads are message 1 and twice message 1 (time tags 11.11,
    /// 22.22, 11.1 and 22.2).
    fn upstream_nav_aggregate_test(num_att_nav: usize, num_trans_nav: usize) {
        let nav_att1 = nav_att(1.0, 11.11);
        let nav_att2 = nav_att(2.0, 22.22);
        let nav_trans1 = nav_trans(1.0, 11.1);
        let nav_trans2 = nav_trans(2.0, 22.2);
        let nav_att1_out = Output::new(nav_att1.clone());
        let nav_att2_out = Output::new(nav_att2.clone());
        let nav_trans1_out = Output::new(nav_trans1.clone());
        let nav_trans2_out = Output::new(nav_trans2.clone());

        let mut module = NavAggregate::new(NavAggregateConfig::default());
        // Here the index asks to read from an empty (zero) slot.
        module.config.att_msg_count = if num_att_nav == 3 { 2 } else { num_att_nav };
        module.config.trans_msg_count = if num_trans_nav == 3 { 2 } else { num_trans_nav };

        if num_att_nav <= MAX_AGG_NAV_MSG {
            nav_att1_out.connect_to(&mut module.nav_att_in_msgs[0]);
            nav_att2_out.connect_to(&mut module.nav_att_in_msgs[1]);
        } else {
            for input in &mut module.nav_att_in_msgs {
                nav_att1_out.connect_to(input);
            }
        }
        if num_trans_nav <= MAX_AGG_NAV_MSG {
            nav_trans1_out.connect_to(&mut module.nav_trans_in_msgs[0]);
            nav_trans2_out.connect_to(&mut module.nav_trans_in_msgs[1]);
        } else {
            for input in &mut module.nav_trans_in_msgs {
                nav_trans1_out.connect_to(input);
            }
        }

        // Always read from the last message counter.
        if num_att_nav > 1 {
            let config = &mut module.config;
            config.att_time_idx = num_att_nav - 1;
            config.att_idx = num_att_nav - 1;
            config.rate_idx = num_att_nav - 1;
            config.sun_idx = num_att_nav - 1;
        }
        if num_trans_nav > 1 {
            let config = &mut module.config;
            config.trans_time_idx = num_trans_nav - 1;
            config.pos_idx = num_trans_nav - 1;
            config.vel_idx = num_trans_nav - 1;
            config.dv_idx = num_trans_nav - 1;
        }

        module.init();
        let mut att_samples = Vec::new();
        let mut trans_samples = Vec::new();
        for tick in 0..=2 {
            module.update(&context(tick * TASK_NANOS));
            att_samples.push(module.nav_att_out_msg.read());
            trans_samples.push(module.nav_trans_out_msg.read());
        }

        let true_att = match num_att_nav {
            0 | 3 => NavigationAttitudeMsg::default(),
            1 | 11 => nav_att1,
            2 => nav_att2,
            _ => unreachable!("not an upstream parametrization"),
        };
        let true_trans = match num_trans_nav {
            0 | 3 => NavigationTranslationMsg::default(),
            1 | 11 => nav_trans1,
            2 => nav_trans2,
            _ => unreachable!("not an upstream parametrization"),
        };

        let accuracy = 1e-12;
        let case = format!("({num_att_nav}, {num_trans_nav})");
        for attitude in &att_samples {
            assert!(
                (attitude.time_tag_s - true_att.time_tag_s).abs() <= accuracy,
                "{case}"
            );
            assert_vector_close(attitude.sigma_bn, true_att.sigma_bn, accuracy);
            assert_vector_close(
                attitude.omega_bn_b_radps,
                true_att.omega_bn_b_radps,
                accuracy,
            );
            assert_vector_close(
                attitude.vehicle_sun_point_body,
                true_att.vehicle_sun_point_body,
                accuracy,
            );
        }
        for translation in &trans_samples {
            assert!(
                (translation.time_tag_s - true_trans.time_tag_s).abs() <= accuracy,
                "{case}"
            );
            assert_vector_close(
                translation.position_bn_n_m,
                true_trans.position_bn_n_m,
                accuracy,
            );
            assert_vector_close(
                translation.velocity_bn_n_mps,
                true_trans.velocity_bn_n_mps,
                accuracy,
            );
            assert_vector_close(
                translation.accumulated_dv_b_mps,
                true_trans.accumulated_dv_b_mps,
                accuracy,
            );
        }

        let config = &module.config;
        if num_att_nav == 11 {
            assert_eq!(
                config.att_msg_count, MAX_AGG_NAV_MSG,
                "{case}: numAttNav too large"
            );
            assert_eq!(
                config.att_time_idx,
                MAX_AGG_NAV_MSG - 1,
                "{case}: attTimeIdx too large"
            );
            assert_eq!(
                config.att_idx,
                MAX_AGG_NAV_MSG - 1,
                "{case}: attIdx too large"
            );
            assert_eq!(
                config.rate_idx,
                MAX_AGG_NAV_MSG - 1,
                "{case}: rateIdx too large"
            );
            assert_eq!(
                config.sun_idx,
                MAX_AGG_NAV_MSG - 1,
                "{case}: sunIdx too large"
            );
        }
        if num_trans_nav == 11 {
            assert_eq!(
                config.trans_msg_count, MAX_AGG_NAV_MSG,
                "{case}: numTransNav too large"
            );
            assert_eq!(
                config.pos_idx,
                MAX_AGG_NAV_MSG - 1,
                "{case}: posIdx too large"
            );
            assert_eq!(
                config.vel_idx,
                MAX_AGG_NAV_MSG - 1,
                "{case}: velIdx too large"
            );
            assert_eq!(
                config.dv_idx,
                MAX_AGG_NAV_MSG - 1,
                "{case}: dvIdx too large"
            );
        }
    }

    /// Upstream `test_module`, every `(numAttNav, numTransNav)`
    /// parametrization.
    #[test]
    fn matches_upstream_nav_aggregate_unit_test() {
        for (num_att_nav, num_trans_nav) in [
            (0, 0),
            (1, 1),
            (0, 1),
            (1, 0),
            (2, 2),
            (1, 2),
            (0, 2),
            (2, 1),
            (2, 0),
            (3, 3),
            (3, 2),
            (3, 1),
            (3, 0),
            (2, 3),
            (1, 3),
            (0, 3),
            (11, 11),
            (3, 11),
            (2, 11),
            (1, 11),
            (0, 11),
            (11, 3),
            (11, 2),
            (11, 1),
            (11, 0),
        ] {
            upstream_nav_aggregate_test(num_att_nav, num_trans_nav);
        }
    }

    // Not upstream: each field comes from its own slot, and reset clears the
    // stored payloads so a slot dropped from the count reads zero.
    #[test]
    fn takes_each_field_from_its_selected_slot() {
        let first = Output::new(nav_trans(1.0, 1.0));
        let second = Output::new(nav_trans(2.0, 2.0));
        let third = Output::new(nav_trans(3.0, 3.0));
        let mut module = NavAggregate::new(NavAggregateConfig {
            trans_msg_count: 3,
            trans_time_idx: 2,
            pos_idx: 0,
            vel_idx: 1,
            dv_idx: 2,
            ..Default::default()
        });
        first.connect_to(&mut module.nav_trans_in_msgs[0]);
        second.connect_to(&mut module.nav_trans_in_msgs[1]);
        third.connect_to(&mut module.nav_trans_in_msgs[2]);
        module.init();
        module.update(&context(0));
        let output = module.nav_trans_out_msg.read();
        assert_eq!(output.time_tag_s, 3.0);
        assert_eq!(output.position_bn_n_m, nav_trans(1.0, 0.0).position_bn_n_m);
        assert_eq!(
            output.velocity_bn_n_mps,
            nav_trans(2.0, 0.0).velocity_bn_n_mps
        );
        assert_eq!(
            output.accumulated_dv_b_mps,
            nav_trans(3.0, 0.0).accumulated_dv_b_mps
        );

        module.config.trans_msg_count = 2;
        module.reset(&context(TASK_NANOS));
        module.update(&context(TASK_NANOS));
        let output = module.nav_trans_out_msg.read();
        assert_eq!(output.time_tag_s, 0.0);
        assert_eq!(output.accumulated_dv_b_mps, Vector3::zeros());
        assert_eq!(output.position_bn_n_m, nav_trans(1.0, 0.0).position_bn_n_m);
    }
}
