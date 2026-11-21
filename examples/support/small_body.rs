//! Setup shared by the ports of upstream's Bennu proximity-operation
//! scenarios (`scenarioSmallBodyNav`, `scenarioSmallBodyFeedbackControl`)
//! and the Vesta one (`scenarioSmallBodyNavUKF`): the small bodies'
//! `planetEphemeris` orbits, the 330 kg spacecraft with its three HR16
//! wheels, the `planetNav` and diagonal-`PMatrix` noise settings, and the
//! zeroed Sun messages those scripts write.
//!
//! Include it next to `common.rs` and `rw_factory.rs`:
//!
//! ```ignore
//! #[path = "support/common.rs"]
//! mod common;
//! #[path = "support/rw_factory.rs"]
//! mod rw_factory;
//! #[path = "support/small_body.rs"]
//! mod small_body;
//! ```

#![allow(dead_code)]

use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffector,
};
use basilisk_rs::environment::ephemeris_converter::{EphemerisConverter, EphemerisConverterConfig};
use basilisk_rs::environment::planet_ephemeris::{
    PlanetEphemeris, PlanetEphemerisBody, PlanetEphemerisConfig, PlanetRotation,
};
use basilisk_rs::messages::RwArrayConfigMsg;
use basilisk_rs::sensors::planet_nav::{
    PlanetNav, PlanetNavConfig, PlanetNavMatrix, PlanetNavVector,
};
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::astro_constants::{AU, D2R, MU_SUN};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use nalgebra::{Matrix3, Vector3};

use super::rw_factory::RwFactory;

/// Upstream `simulationTimeStep = macros.sec2nano(1.0)` of the Bennu scenarios.
pub const BENNU_STEP_NANOS: u64 = 1_000_000_000;
/// Upstream `mu = 4.892` \[m^3/s^2\]: Bennu's gravitational parameter.
pub const BENNU_MU_M3PS2: f64 = 4.892;
/// Upstream `mass = 330.` \[kg\].
pub const SPACECRAFT_MASS_KG: f64 = 330.0;
/// Upstream `rotRate = 360 deg / (4.297461 h)` \[rad/s\]: Bennu's spin.
pub const BENNU_ROTATION_RATE_RADPS: f64 = 360.0 * D2R / (4.297461 * 3600.0);
/// The Sun's gravitational parameter \[m^3/s^2\] of upstream `elem2rv`.
pub const MU_SUN_M3PS2: f64 = MU_SUN * 1.0e9;

/// Upstream `I = [82.12, 0, 0, 0, 98.40, 0, 0, 0, 121.0]` \[kg m^2\].
pub fn spacecraft_inertia_kg_m2() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(82.12, 98.40, 121.0))
}

/// Upstream `oeAsteroid`: Bennu's heliocentric orbit on December 31st, 2018
/// (JPL Horizons).
pub fn bennu_elements() -> ClassicElements {
    ClassicElements {
        semi_major_axis: 1.1259 * AU * 1000.0,
        eccentricity: 0.20373,
        inclination_rad: 6.0343 * D2R,
        raan_rad: 2.01820 * D2R,
        arg_periapsis_rad: 66.304 * D2R,
        true_anomaly_rad: 346.32 * D2R,
        ..Default::default()
    }
}

/// Upstream `r_ON_N, v_ON_N = orbitalMotion.elem2rv(MU_SUN, oeAsteroid)`:
/// Bennu's initial heliocentric state \[m, m/s\].
pub fn bennu_initial_state() -> (Vector3<f64>, Vector3<f64>) {
    elements_to_state(MU_SUN_M3PS2, &bennu_elements())
}

/// Upstream `planetEphemeris.PlanetEphemeris()` named `planetEphemeris`
/// with the one body `bennu` on [`bennu_elements`], spinning at
/// [`BENNU_ROTATION_RATE_RADPS`] about the pole `(right_ascension_deg,
/// declination_deg)` from `lst0 = 0`.
pub fn bennu_ephemeris(right_ascension_deg: f64, declination_deg: f64) -> PlanetEphemeris {
    PlanetEphemeris::new(PlanetEphemerisConfig {
        name: "planetEphemeris".to_string(),
        bodies: vec![PlanetEphemerisBody {
            name: "bennu".to_string(),
            elements: bennu_elements(),
            rotation: Some(PlanetRotation {
                right_ascension_rad: right_ascension_deg * D2R,
                declination_rad: declination_deg * D2R,
                lst0_rad: 0.0,
                rotation_rate_radps: BENNU_ROTATION_RATE_RADPS,
            }),
        }],
        zero_base: None,
    })
}

/// Upstream `ephemerisConverter.EphemerisConverter()` named
/// `ephemConverter`, with one `addSpiceInputMsg` for `body`.
pub fn ephemeris_converter(body: &str) -> EphemerisConverter {
    EphemerisConverter::new(EphemerisConverterConfig {
        name: "ephemConverter".to_string(),
        body_names: vec![body.to_string()],
    })
}

/// Upstream `planetNav.PlanetNav()` with the diagonal `PMatrix` of
/// per-group standard deviations (position, velocity, attitude, rate) and
/// the per-group `walkBounds`.
pub fn planet_nav(name: &str, sigmas: [f64; 4], walk_bounds: [f64; 4]) -> PlanetNav {
    PlanetNav::new(PlanetNavConfig {
        name: name.to_string(),
        p_matrix_sqrt: PlanetNavMatrix::from_diagonal(&PlanetNavVector::from_fn(|index, _| {
            sigmas[index / 3]
        })),
        walk_bounds: PlanetNavVector::from_fn(|index, _| walk_bounds[index / 3]),
        cross_trans: false,
        cross_att: false,
    })
}

/// Upstream's `bskSat` of the Bennu scenarios: 330 kg, inertia
/// [`spacecraft_inertia_kg_m2`], `sigma_BN = [0.1, 0, 0]`,
/// `omega_BN_B = [0.1, 0.1, 0.1] rad/s`, at `r_BO_N`, `v_BO_N` from Bennu
/// (`r_BN_N = r_BO_N + r_ON_N`), integrated with RK4 at 1 s.
pub fn bennu_spacecraft(r_bo_n_m: Vector3<f64>, v_bo_n_mps: Vector3<f64>) -> Spacecraft {
    let (r_on_n, v_on_n) = bennu_initial_state();
    Spacecraft::new(SpacecraftConfig {
        mass_kg: SPACECRAFT_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: spacecraft_inertia_kg_m2(),
        integration_step_nanos: BENNU_STEP_NANOS,
        initial_position_m: r_bo_n_m + r_on_n,
        initial_velocity_mps: v_bo_n_mps + v_on_n,
        initial_sigma_bn: Vector3::new(0.1, 0.0, 0.0),
        initial_omega_radps: Vector3::new(0.1, 0.1, 0.1),
        integrator: None,
    })
}

/// The three-wheel cluster of the Bennu scenarios: upstream
/// `rwFactory.create('Honeywell_HR16', axis, maxMomentum=max_momentum_nms,
/// Omega=...)` along the body axes at 100, 200 and 300 RPM, balanced wheels
/// at the hub origin, in `RW_cluster`; with the FSW configuration message
/// (`getConfigMessage()`) and the per-wheel split of `rwMotorTorque`'s
/// array command (upstream subscribes the effector to that array).
pub struct WheelCluster {
    pub effector: ReactionWheelStateEffector,
    pub config: RwArrayConfigMsg,
    pub splitter: ArrayMotorTorqueToSingleActuators,
}

pub fn wheel_cluster(max_momentum_nms: f64) -> WheelCluster {
    let mut factory = RwFactory::default();
    for (axis, speed_rpm) in [
        (Vector3::x(), 100.0),
        (Vector3::y(), 200.0),
        (Vector3::z(), 300.0),
    ] {
        factory.create_honeywell_hr16(
            axis,
            max_momentum_nms,
            speed_rpm,
            Vector3::zeros(),
            ReactionWheelModel::BalancedWheels,
        );
    }
    let config = factory.config_message();
    let mut effector = ReactionWheelStateEffector::new("RW_cluster");
    let count = factory.wheels.len();
    for wheel in factory.wheels {
        effector.add_reaction_wheel(wheel);
    }
    WheelCluster {
        effector,
        config,
        splitter: ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "rwMotorTorqueSplitter".to_string(),
            num_actuators: count,
        }),
    }
}
