//! Thrusters as upstream `simIncludeThruster.thrusterFactory` creates them,
//! and the two thruster layouts of upstream's
//! `scenarioAttitudeFeedback2T_TH` scenarios.
//!
//! `thrusterFactory.create(type, r_B, tHat_B)` starts from the factory
//! defaults (`MaxThrust = 0.2 N`, `steadyIsp = 100 s`, `areaNozzle = 0.1 m^2`,
//! `MinOnTime = 0.02 s`, `cutoffFrequency = 10 rad/s`, no swirl or
//! dispersion), applies the named model, normalizes the direction and, as
//! `useMinPulseTime` defaults to off, zeroes the minimum on-time.

#![allow(dead_code)]

use basilisk_rs::dynamics::thruster_dynamic_effector::ThrusterDynamicEffectorConfig;
use basilisk_rs::dynamics::thruster_state_effector::ThrusterStateEffectorConfig;
use basilisk_rs::fsw_algorithms::effector_interfaces::thr_firing_remainder::BaseThrustState;
use basilisk_rs::fsw_algorithms::effector_interfaces::thr_firing_schmitt::{
    ThrFiringSchmitt, ThrFiringSchmittConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::thr_force_mapping::{
    ThrForceMapping, ThrForceMappingConfig, ThrForceSign,
};
use basilisk_rs::messages::{ThrusterArrayConfigMsg, ThrusterConfigMsg};
use nalgebra::Vector3;

/// One thruster of a factory (upstream `THRSimConfig`).
#[derive(Clone, Debug)]
pub struct ThrusterSpec {
    /// Upstream `label`, `TH<n>` by default.
    pub label: String,
    /// Upstream `thrLoc_B` \[m\].
    pub position_m: Vector3<f64>,
    /// Upstream `thrDir_B`, normalized.
    pub direction_body: Vector3<f64>,
    /// Upstream `MaxThrust` \[N\].
    pub max_thrust_n: f64,
    /// Upstream `steadyIsp` \[s\].
    pub steady_isp_s: f64,
    /// Upstream `areaNozzle` \[m^2\].
    pub area_nozzle_m2: f64,
    /// Upstream `MinOnTime` \[s\].
    pub min_on_time_s: f64,
    /// Upstream `cutoffFrequency` \[rad/s\] (thruster state effector only).
    pub cutoff_frequency_radps: f64,
}

/// Upstream thruster models: `thrusterFactory.<name>(TH)`.
#[derive(Clone, Copy, Debug)]
pub enum ThrusterModel {
    /// `MOOG_Monarc_1`: 0.9 N, Isp 227.5 s, nozzle 0.000079 m^2.
    MoogMonarc1,
    /// `MOOG_Monarc_5`: 4.5 N, Isp 226.1 s, nozzle 0.0020 m^2.
    MoogMonarc5,
    /// `MOOG_Monarc_22_6`: 22 N, Isp 229.5 s, nozzle 0.0045 m^2.
    MoogMonarc22_6,
}

/// Upstream `thrusterFactory`: the thrusters in creation order.
#[derive(Clone, Debug, Default)]
pub struct ThrusterFactory {
    pub thrusters: Vec<ThrusterSpec>,
}

impl ThrusterFactory {
    /// Upstream `create(type, r_B, tHat_B)`, optionally with
    /// `cutoffFrequency=...`.
    pub fn create(
        &mut self,
        model: ThrusterModel,
        position_m: Vector3<f64>,
        direction_body: Vector3<f64>,
        cutoff_frequency_radps: Option<f64>,
    ) -> &mut ThrusterSpec {
        let (max_thrust_n, steady_isp_s, area_nozzle_m2) = match model {
            ThrusterModel::MoogMonarc1 => (0.9, 227.5, 0.000_079),
            ThrusterModel::MoogMonarc5 => (4.5, 226.1, 0.0020),
            ThrusterModel::MoogMonarc22_6 => (22.0, 229.5, 0.0045),
        };
        let label = format!("TH{}", self.thrusters.len() + 1);
        self.thrusters.push(ThrusterSpec {
            label,
            position_m,
            direction_body: direction_body.normalize(),
            max_thrust_n,
            steady_isp_s,
            area_nozzle_m2,
            // `useMinPulseTime` defaults to off, which zeroes `MinOnTime`.
            min_on_time_s: 0.0,
            cutoff_frequency_radps: cutoff_frequency_radps.unwrap_or(10.0),
        });
        self.thrusters.last_mut().expect("just created")
    }

    /// Upstream `getConfigMessage()`: the FSW view of the cluster.
    pub fn config_message(&self) -> ThrusterArrayConfigMsg {
        let thrusters: Vec<ThrusterConfigMsg> = self
            .thrusters
            .iter()
            .map(|thruster| ThrusterConfigMsg {
                position_thrust_b_m: thruster.position_m,
                t_hat_thrust_b: thruster.direction_body,
                max_thrust_n: thruster.max_thrust_n,
                swirl_torque_nm: 0.0,
            })
            .collect();
        ThrusterArrayConfigMsg::from_active(&thrusters)
    }
}

impl ThrusterSpec {
    /// The [`ThrusterDynamicEffector`](basilisk_rs::dynamics::thruster_dynamic_effector::ThrusterDynamicEffector)
    /// configuration of this thruster.
    pub fn dynamic_effector_config(&self) -> ThrusterDynamicEffectorConfig {
        let mut config = ThrusterDynamicEffectorConfig::generic_pulsed(
            self.label.clone(),
            self.position_m,
            self.direction_body,
            self.max_thrust_n,
            self.min_on_time_s,
            self.steady_isp_s,
        );
        config.area_nozzle_m2 = self.area_nozzle_m2;
        config
    }

    /// The [`ThrusterStateEffector`](basilisk_rs::dynamics::thruster_state_effector::ThrusterStateEffector)
    /// configuration of this thruster.
    pub fn state_effector_config(&self) -> ThrusterStateEffectorConfig {
        let mut config = ThrusterStateEffectorConfig::new(
            self.label.clone(),
            self.position_m,
            self.direction_body,
            self.max_thrust_n,
            self.steady_isp_s,
        );
        config.min_on_time_s = self.min_on_time_s;
        config.cutoff_frequency_radps = self.cutoff_frequency_radps;
        config.area_nozzle_m2 = self.area_nozzle_m2;
        config
    }
}

/// The eight attitude-control thrusters of the `2T_TH` scenarios
/// (`MOOG_Monarc_1`): locations \[m\] and thrust directions.
pub fn acs_thruster_layout() -> Vec<(Vector3<f64>, Vector3<f64>)> {
    let locations = [
        [0.038_749_451_609_022_88, -1.206_182_747_348_013, 0.85245],
        [0.038_749_451_609_022_88, -1.206_182_747_348_013, -0.85245],
        [-0.038_749_451_609_022_656, -1.206_182_747_348_013, 0.85245],
        [-0.038_749_451_609_022_656, -1.206_182_747_348_013, -0.85245],
        [-0.038_749_451_609_022_88, 1.206_182_747_348_013, 0.85245],
        [-0.038_749_451_609_022_88, 1.206_182_747_348_013, -0.85245],
        [0.038_749_451_609_022_656, 1.206_182_747_348_013, 0.85245],
        [0.038_749_451_609_022_656, 1.206_182_747_348_013, -0.85245],
    ];
    #[allow(clippy::approx_constant)]
    let directions = [
        [-0.707_106_781_186_547_6, 0.707_106_781_186_547_5, 0.0],
        [-0.707_106_781_186_547_6, 0.707_106_781_186_547_5, 0.0],
        [0.707_106_781_186_547_5, 0.707_106_781_186_547_6, 0.0],
        [0.707_106_781_186_547_5, 0.707_106_781_186_547_6, 0.0],
        [0.707_106_781_186_547_6, -0.707_106_781_186_547_5, 0.0],
        [0.707_106_781_186_547_6, -0.707_106_781_186_547_5, 0.0],
        [-0.707_106_781_186_547_5, -0.707_106_781_186_547_6, 0.0],
        [-0.707_106_781_186_547_5, -0.707_106_781_186_547_6, 0.0],
    ];
    locations
        .into_iter()
        .zip(directions)
        .map(|(location, direction)| (Vector3::from(location), Vector3::from(direction)))
        .collect()
}

/// The six orbit-control thrusters of the `2T_TH` scenarios
/// (`MOOG_Monarc_22_6`), all firing along `+b3`.
pub fn dv_thruster_layout() -> Vec<(Vector3<f64>, Vector3<f64>)> {
    let locations = [
        [0.0, 0.95, -1.1],
        [0.822_724_133_595_216_6, 0.475_000_000_000_000_3, -1.1],
        [0.822_724_133_595_216_8, -0.474_999_999_999_999_76, -1.1],
        [0.0, -0.95, -1.1],
        [-0.822_724_133_595_216_5, -0.475_000_000_000_000_4, -1.1],
        [-0.822_724_133_595_217, 0.474_999_999_999_999_3, -1.1],
    ];
    locations
        .into_iter()
        .map(|location| (Vector3::from(location), Vector3::z()))
        .collect()
}

/// The thruster set of the `2T_TH` scenarios: the six DV thrusters
/// (`MOOG_Monarc_22_6`) when `use_dv_thrusters`, else the eight ACS
/// thrusters (`MOOG_Monarc_1`), created with `cutoffFrequency` when given.
pub fn thruster_factory(
    use_dv_thrusters: bool,
    cutoff_frequency_radps: Option<f64>,
) -> ThrusterFactory {
    let (model, layout) = if use_dv_thrusters {
        (ThrusterModel::MoogMonarc22_6, dv_thruster_layout())
    } else {
        (ThrusterModel::MoogMonarc1, acs_thruster_layout())
    };
    let mut factory = ThrusterFactory::default();
    for (position_m, direction_body) in layout {
        factory.create(model, position_m, direction_body, cutoff_frequency_radps);
    }
    factory
}

/// Upstream `thrForceMapping` of the `2T_TH` scenarios: all three body axes
/// for the on-pulsing ACS thrusters (`thrForceSign = +1`), the first two for
/// the off-pulsing DV thrusters (`thrForceSign = -1`).
pub fn thr_force_mapping(use_dv_thrusters: bool) -> ThrForceMapping {
    let (control_axes_b, thr_force_sign) = if use_dv_thrusters {
        (
            [Vector3::x(), Vector3::y(), Vector3::zeros()],
            ThrForceSign::Negative,
        )
    } else {
        (
            [Vector3::x(), Vector3::y(), Vector3::z()],
            ThrForceSign::Positive,
        )
    };
    ThrForceMapping::new(ThrForceMappingConfig {
        name: "thrForceMapping".to_string(),
        control_axes_b,
        thr_force_sign,
        ..Default::default()
    })
}

/// Upstream `thrFiringSchmitt` of the `2T_TH` scenarios: 0.002 s minimum
/// fire time, `level_on = 0.75`, `level_off = 0.25`, off-pulsing
/// (`baseThrustState = 1`) for the DV thrusters.
pub fn thr_firing_schmitt(use_dv_thrusters: bool) -> ThrFiringSchmitt {
    ThrFiringSchmitt::new(ThrFiringSchmittConfig {
        name: "thrFiringSchmitt".to_string(),
        level_on: 0.75,
        level_off: 0.25,
        thr_min_fire_time_s: 0.002,
        base_thrust_state: if use_dv_thrusters {
            BaseThrustState::OffPulsing
        } else {
            BaseThrustState::OnPulsing
        },
    })
}
