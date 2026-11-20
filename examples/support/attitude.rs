//! Setup shared by the ports of upstream's attitude-control example scenarios
//! (`scenarioAttitudeFeedback*`, `scenarioAttitudeGuidance`, ...): the
//! 750 kg rigid hub, the 10 000 km orbit, the point-mass Earth and the
//! standard flight-software blocks, configured as those scripts configure
//! them.
//!
//! Include it next to `common.rs`:
//!
//! ```ignore
//! #[path = "support/common.rs"]
//! mod common;
//! #[path = "support/attitude.rs"]
//! mod attitude;
//! ```

#![allow(dead_code)]

use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::hill_point::{HillPoint, HillPointConfig};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::messages::VehicleConfigMsg;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::{CsvFormat, CsvRecorder, CsvRecorderConfig};
use std::path::PathBuf;

use nalgebra::{Matrix3, Vector3};

use super::common::{MU_EARTH_M3PS2, elem2rv, scenario_output_path, seconds};

/// Upstream `simulationTimeStep = macros.sec2nano(.1)`: the dynamics task.
pub const DYNAMICS_PERIOD_NANOS: u64 = 100_000_000;
/// Upstream `simulationTime = macros.min2nano(10.)`.
pub const DURATION_NANOS: u64 = seconds(600);
/// Upstream `scObject.hub.mHub` \[kg\].
pub const HUB_MASS_KG: f64 = 750.0;
/// Upstream `sigma_BNInit` of most attitude scenarios.
pub const INITIAL_SIGMA_BN: [f64; 3] = [0.1, 0.2, -0.3];
/// Upstream `omega_BN_BInit` \[rad/s\] of most attitude scenarios.
pub const INITIAL_OMEGA_BN_RADPS: [f64; 3] = [0.001, -0.01, 0.03];
/// Upstream `extFTObject.extTorquePntB_B` \[N m\], the "unmodeled" torque of
/// the feedback scenarios (also their `knownTorquePntB_B`).
pub const UNMODELED_TORQUE_NM: [f64; 3] = [0.25, -0.25, 0.1];

/// Upstream `I = [900., 0., 0., 0., 800., 0., 0., 0., 600.]` \[kg m^2\].
pub fn hub_inertia_kg_m2() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0))
}

/// Upstream `simHelpers.samplingTime`: the recorder period \[ns\] giving
/// roughly `data_points` samples, a whole number of `base_period_nanos`.
pub fn sampling_period_nanos(duration_nanos: u64, base_period_nanos: u64, data_points: u64) -> u64 {
    (duration_nanos / base_period_nanos / (data_points - 1) * base_period_nanos).max(1)
}

/// The orbit of the attitude scenarios: `a = 10 000 km`, `i = 33.3°`,
/// `Ω = 48.2°`, `ω = 347.8°`, `f = 85.3°`, with the scenario's eccentricity.
pub struct ScenarioOrbit {
    pub semimajor_axis_m: f64,
    pub eccentricity: f64,
    pub inclination_rad: f64,
    pub raan_rad: f64,
    pub argument_of_periapsis_rad: f64,
    pub true_anomaly_rad: f64,
}

impl ScenarioOrbit {
    pub fn new(eccentricity: f64) -> Self {
        Self {
            semimajor_axis_m: 10_000_000.0,
            eccentricity,
            inclination_rad: 33.3_f64.to_radians(),
            raan_rad: 48.2_f64.to_radians(),
            argument_of_periapsis_rad: 347.8_f64.to_radians(),
            true_anomaly_rad: 85.3_f64.to_radians(),
        }
    }

    /// Upstream `orbitalMotion.elem2rv(mu, oe)` about the point-mass Earth.
    pub fn state(&self) -> (Vector3<f64>, Vector3<f64>) {
        elem2rv(
            MU_EARTH_M3PS2,
            self.semimajor_axis_m,
            self.eccentricity,
            self.inclination_rad,
            self.raan_rad,
            self.argument_of_periapsis_rad,
            self.true_anomaly_rad,
        )
    }

    /// Mean motion `n = sqrt(mu / a^3)` \[rad/s\].
    pub fn mean_motion_radps(&self) -> f64 {
        (MU_EARTH_M3PS2 / self.semimajor_axis_m.powi(3)).sqrt()
    }
}

/// The initial hub state of a scenario.
pub struct HubState {
    pub position_m: Vector3<f64>,
    pub velocity_mps: Vector3<f64>,
    pub sigma_bn: Vector3<f64>,
    pub omega_bn_b_radps: Vector3<f64>,
}

impl HubState {
    /// The orbit `ScenarioOrbit::new(eccentricity)` with the standard
    /// initial attitude and rate.
    pub fn on_orbit(eccentricity: f64) -> Self {
        let (position_m, velocity_mps) = ScenarioOrbit::new(eccentricity).state();
        Self {
            position_m,
            velocity_mps,
            sigma_bn: Vector3::from(INITIAL_SIGMA_BN),
            omega_bn_b_radps: Vector3::from(INITIAL_OMEGA_BN_RADPS),
        }
    }

    /// Deep space at the origin, at rest, with the standard initial attitude
    /// and rate (the scenarios that set no orbit).
    pub fn in_deep_space() -> Self {
        Self {
            position_m: Vector3::zeros(),
            velocity_mps: Vector3::zeros(),
            sigma_bn: Vector3::from(INITIAL_SIGMA_BN),
            omega_bn_b_radps: Vector3::from(INITIAL_OMEGA_BN_RADPS),
        }
    }
}

/// Upstream `spacecraft.Spacecraft()` with `mHub = 750`, `r_BcB_B = 0` and
/// hub inertia `inertia_kg_m2`, integrated with RK4 on the dynamics task.
pub fn spacecraft(inertia_kg_m2: Matrix3<f64>, initial: &HubState) -> Spacecraft {
    Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2,
        integration_step_nanos: DYNAMICS_PERIOD_NANOS,
        initial_position_m: initial.position_m,
        initial_velocity_mps: initial.velocity_mps,
        initial_sigma_bn: initial.sigma_bn,
        initial_omega_radps: initial.omega_bn_b_radps,
        integrator: None,
    })
}

/// Upstream `gravFactory.createEarth()` as the central body, a point mass
/// at the origin.
pub fn add_point_mass_earth(spacecraft: &mut Spacecraft) {
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass(
                "earth",
                MU_EARTH_M3PS2,
                true,
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .expect("valid point-mass Earth"),
        )
        .expect("unique central gravity body");
}

/// Upstream `messaging.VehicleConfigMsgPayload(ISCPntB_B=I)`: only the
/// inertia is set.
pub fn vehicle_config(inertia_kg_m2: Matrix3<f64>) -> VehicleConfigMsg {
    VehicleConfigMsg {
        inertia_about_point_b_kg_m2: inertia_kg_m2,
        ..Default::default()
    }
}

/// Upstream `extForceTorque.ExtForceTorque()` named `externalDisturbance`
/// with a constant torque about B \[N m\].
pub fn external_disturbance(torque_body_nm: Vector3<f64>) -> ExtForceTorque {
    ExtForceTorque::new(ExtForceTorqueConfig {
        name: "externalDisturbance".to_string(),
        ext_force_inertial_n: Vector3::zeros(),
        ext_force_body_n: Vector3::zeros(),
        ext_torque_body_nm: torque_body_nm,
    })
}

/// Upstream `inertial3D` with `sigma_R0N = 0`.
pub fn inertial_3d() -> Inertial3D {
    Inertial3D::new(Inertial3DConfig {
        name: "inertial3D".to_string(),
        sigma_r0n: Vector3::zeros(),
    })
}

/// Upstream `hillPoint`.
pub fn hill_point() -> HillPoint {
    HillPoint::new(HillPointConfig {
        name: "hillPoint".to_string(),
    })
}

/// Upstream `attTrackingError` named `attErrorInertial3D`, with the
/// body-frame correction `sigma_R0R`.
pub fn att_tracking_error(sigma_r0r: Vector3<f64>) -> AttTrackingError {
    AttTrackingError::new(AttTrackingErrorConfig {
        name: "attErrorInertial3D".to_string(),
        sigma_r0r,
    })
}

/// Upstream `mrpFeedback` with gains `K`, `Ki` (negative turns the integral
/// feedback off), `P`, the scripts' `integralLimit = 2 / Ki * 0.1`, and the
/// feed-forward `knownTorquePntB_B` \[N m\].
pub fn mrp_feedback(k: f64, ki: f64, p: f64, known_torque_body_nm: Vector3<f64>) -> MrpFeedback {
    MrpFeedback::new(MrpFeedbackConfig {
        name: "mrpFeedback".to_string(),
        k,
        ki,
        p,
        integral_limit: 2.0 / ki * 0.1,
        known_torque_body_nm,
        control_law_type: 0,
    })
}

/// A scenario-format CSV recorder for
/// `examples/output/scenarios/<stem>.csv`, with that path when `write_csv`.
/// The file is only replaced (and created) when `write_csv`; the caller
/// schedules the recorder only then.
pub fn scenario_recorder(stem: &str, write_csv: bool) -> (CsvRecorder, Option<PathBuf>) {
    let output_path = write_csv.then(|| scenario_output_path(&format!("{stem}.csv")));
    let recorder = CsvRecorder::new(CsvRecorderConfig {
        topic: stem.to_string(),
        output_path: output_path.clone().unwrap_or_default(),
    })
    .with_format(CsvFormat::Scenario);
    (recorder, output_path)
}

/// `0` or `1`, as upstream's `str(int(flag))` figure-name suffixes.
pub fn bit(flag: bool) -> char {
    if flag { '1' } else { '0' }
}
