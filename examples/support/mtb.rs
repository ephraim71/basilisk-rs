//! Setup shared by the ports of upstream's magnetic torque-bar momentum
//! management scenarios (`scenarioMtbMomentumManagement`,
//! `scenarioMtbMomentumManagementSimple`): the 10 kg spacecraft on its
//! 400 km orbit, the four BCT RWP015 wheels, the four torque bars, WMM at the
//! scenarios' epoch, the noiseless magnetometer and its `tamComm`, and the
//! attitude controller.

#![allow(dead_code)]

use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::dynamics::reaction_wheel_state_effector::ReactionWheelModel;
use basilisk_rs::environment::wmm_field::{WmmField, WmmFieldConfig};
use basilisk_rs::fsw_algorithms::mrp_feedback::MrpFeedback;
use basilisk_rs::fsw_algorithms::tam_comm::{TamComm, TamCommConfig};
use basilisk_rs::messages::{MtbArrayConfigMsg, VehicleConfigMsg};
use basilisk_rs::sensors::magnetometer::{Magnetometer, MagnetometerConfig};
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use hifitime::Epoch;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};

use super::attitude::mrp_feedback;
use super::common::{MU_EARTH_M3PS2, elem2rv};
use super::rw_factory::RwFactory;

/// Upstream `simulationTime = macros.min2nano(120.)`.
pub const DURATION_NANOS: u64 = 7_200_000_000_000;
/// Upstream `scObject.hub.mHub` \[kg\].
pub const HUB_MASS_KG: f64 = 10.0;
/// Upstream `maxDipole` \[A m^2\] of every torque bar.
pub const MAX_DIPOLE_AM2: f64 = 0.1;

/// Upstream `epochMsg = timeStringToGregorianUTCMsg('2019 June 27,
/// 10:23:0.0 (UTC)')`: the simulation start epoch, which WMM is evaluated at.
pub fn epoch() -> Epoch {
    Epoch::from_gregorian_utc(2019, 6, 27, 10, 23, 0, 0)
}

/// Upstream `I = [0.02 / 3, 0.1256 / 3, 0.1256 / 3]` (diagonal) \[kg m^2\].
pub fn hub_inertia_kg_m2() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(0.02 / 3.0, 0.1256 / 3.0, 0.1256 / 3.0))
}

/// Upstream `VehicleConfigMsgPayload(ISCPntB_B=I)`, with the hub mass.
pub fn vehicle_config() -> VehicleConfigMsg {
    VehicleConfigMsg {
        inertia_about_point_b_kg_m2: hub_inertia_kg_m2(),
        center_of_mass_body_m: Vector3::zeros(),
        mass_kg: HUB_MASS_KG,
        current_adcs_state: 0,
    }
}

/// Upstream `Gs` (`beta = 52 deg`): the four wheel spin axes.
pub fn wheel_axes() -> [Vector3<f64>; 4] {
    let beta = 52.0 * std::f64::consts::PI / 180.0;
    [
        Vector3::new(0.0, beta.cos(), beta.sin()),
        Vector3::new(0.0, beta.sin(), -beta.cos()),
        Vector3::new(beta.cos(), -beta.sin(), 0.0),
        Vector3::new(-beta.cos(), -beta.sin(), 0.0),
    ]
}

/// The four `BCT_RWP015` balanced wheels, `Omega_max = 5000 RPM`.
pub fn rw_factory() -> RwFactory {
    let mut factory = RwFactory::default();
    for axis in wheel_axes() {
        factory.create_bct_rwp015(axis, 5_000.0, ReactionWheelModel::BalancedWheels);
    }
    factory
}

/// Upstream `GtMatrix_B`: the torque-bar axes. The scripts' diagonal axis
/// is the truncated literal `0.70710678`.
#[allow(clippy::approx_constant)]
pub fn mtb_axes() -> [Vector3<f64>; 4] {
    [
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::new(0.707_106_78, 0.707_106_78, 0.0),
    ]
}

/// Upstream `mtbConfigParams`: the four torque bars at 0.1 A m^2.
pub fn mtb_config() -> MtbArrayConfigMsg {
    MtbArrayConfigMsg::from_active(&mtb_axes(), &[MAX_DIPOLE_AM2; 4])
}

/// The spacecraft on upstream's orbit (`a = 6778.14 km`, circular,
/// `i = 45 deg`, `Ω = 60 deg`) about a point-mass Earth, starting at
/// `sigma_BN = [0.1, 0.2, -0.3]`, `omega_BN = [0.001, -0.01, 0.03] rad/s`.
pub fn spacecraft(step_nanos: u64) -> Spacecraft {
    let degrees = std::f64::consts::PI / 180.0;
    let (initial_position_m, initial_velocity_mps) = elem2rv(
        MU_EARTH_M3PS2,
        6_778_140.0,
        0.0,
        45.0 * degrees,
        60.0 * degrees,
        0.0,
        0.0,
    );
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: hub_inertia_kg_m2(),
        integration_step_nanos: step_nanos,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: Vector3::new(0.1, 0.2, -0.3),
        initial_omega_radps: Vector3::new(0.001, -0.01, 0.03),
        integrator: None,
    });
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
    spacecraft
}

/// Upstream `magneticFieldWMM` named `WMM`.
pub fn wmm() -> WmmField {
    WmmField::new(WmmFieldConfig {
        name: "WMM".to_string(),
        ..WmmFieldConfig::default()
    })
}

/// Upstream `mrpFeedback` with `K = 0.0001`, `P = 0.002` and no integral
/// feedback.
pub fn controller() -> MrpFeedback {
    mrp_feedback(0.0001, -1.0, 0.002, Vector3::zeros())
}

/// Upstream `magnetometer.Magnetometer()` with `scaleFactor = 1` and no
/// noise.
pub fn tam() -> Magnetometer {
    Magnetometer::new(MagnetometerConfig {
        name: "TAM_sensor".to_string(),
        body_to_sensor_quaternion: UnitQuaternion::identity(),
        bias_t: Vector3::zeros(),
        p_matrix_sqrt_t: Matrix3::zeros(),
        a_matrix: Matrix3::identity(),
        walk_bounds_t: Vector3::zeros(),
        scale_factor: 1.0,
        min_output_t: -f64::MAX,
        max_output_t: f64::MAX,
        mtb_residual_field: None,
    })
}

/// Upstream `tamComm` with `dcm_BS = I`.
pub fn tam_comm() -> TamComm {
    TamComm::new(TamCommConfig {
        name: "tamComm".to_string(),
        dcm_bs: Matrix3::identity(),
    })
}
