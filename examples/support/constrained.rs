//! Setup shared by the ports of upstream's constrained-dynamics example
//! scenarios (`scenarioConstrainedDynamics*`, `scenarioRoboticGrappling`):
//! two spacecraft joined by a
//! [`ConstraintDynamicEffector`] and integrated together, the spacecraft
//! models they choose from, and the constraint violations they plot.
//!
//! | upstream | here |
//! |---|---|
//! | `scObject1.syncDynamicsIntegration(scObject2)` | [`SpacecraftGroup`](basilisk_rs::spacecraft::SpacecraftGroup) of both |
//! | `svIntegratorRKF45` on `scObject1` | [`upstream_rkf45`] as the group's integrator |
//! | `scObject1/2.addDynamicEffector(constraintEffector)` | `SpacecraftGroup::add_coupling_effector` |
//! | `psi_B1` in `process_data` | [`ConstraintGeometry::direction_violation_b1`] |
//! | `subMRP(sigma_B2N, sigma_B1N)` | [`attitude_violation`] |
//! | `4 arctan(sigma_B2B1)` in degrees | [`violation_angles_deg`] |
//!
//! basilisk-rs's [`Rkf45`] controls its step with its own error norm (an RMS
//! over every scalar) rather than upstream's largest whole-state ratio; with
//! upstream's tolerances (`relTol = 1e-4`, `absTol = 1e-8`) and step factors
//! the two take comparable sub-steps.

#![allow(dead_code)]

use basilisk_rs::dynamics::constraint_dynamic_effector::{
    ConstraintDynamicEffector, ConstraintDynamicEffectorConfig,
};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::integrators::{AdaptiveController, Rkf45};
use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::utilities::rigid_body_kinematics::{mrp_to_dcm, sub_mrp};
use nalgebra::{Matrix3, Vector3};

use super::common::vector_columns;

/// Upstream `earthGravBody.mu` \[m^3/s^2\] of the constrained scenarios.
pub const MU_EARTH_M3PS2: f64 = 0.3986004415e15;

/// Upstream `svIntegratorRKF45` with its default tolerances and step
/// factors.
pub fn upstream_rkf45() -> Rkf45 {
    Rkf45::new(AdaptiveController {
        atol: 1.0e-8,
        rtol: 1.0e-4,
        safety: 0.9,
        min_scale: 0.1,
        max_scale: 4.0,
        ..AdaptiveController::default()
    })
}

/// Upstream's `gravityEffector.GravBodyData()` named `earth_planet_data`,
/// the central body.
pub fn earth_point_mass() -> GravBodyData {
    GravBodyData::point_mass(
        "earth_planet_data",
        MU_EARTH_M3PS2,
        true,
        Vector3::zeros(),
        Vector3::zeros(),
    )
    .expect("valid point-mass Earth")
}

/// The scenarios' low Earth orbit: `a = radEquator + 7500 km` (upstream
/// leaves `radEquator` of its bare `GravBodyData` at 0), `e = 0.01`,
/// `i = 30°`, `Ω = 60°`, `ω = 15°`, `f = 90°`.
pub fn leo_state() -> (Vector3<f64>, Vector3<f64>) {
    let degrees = std::f64::consts::PI / 180.0;
    elements_to_state(
        MU_EARTH_M3PS2,
        &ClassicElements {
            semi_major_axis: 7500.0e3,
            eccentricity: 0.01,
            inclination_rad: 30.0 * degrees,
            raan_rad: 60.0 * degrees,
            arg_periapsis_rad: 15.0 * degrees,
            true_anomaly_rad: 90.0 * degrees,
            ..ClassicElements::default()
        },
    )
}

/// A rigid hub: upstream `spacecraft.Spacecraft()` with `mHub`, `r_BcB_B`
/// and `IHubPntBc_B`. Its initial state is set by the scenario.
pub fn hub(
    mass_kg: f64,
    hub_center_of_mass_body_m: Vector3<f64>,
    inertia_kg_m2: Matrix3<f64>,
    step_nanos: u64,
) -> Spacecraft {
    Spacecraft::new(SpacecraftConfig {
        mass_kg,
        hub_center_of_mass_body_m,
        inertia_kg_m2,
        integration_step_nanos: step_nanos,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    })
}

/// Upstream `sc_model`: the chaser and target spacecraft of the analysis
/// scenarios.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScModel {
    /// Two identical "bskSats".
    BskSat,
    /// MEV-1 servicing Intelsat-901.
    Mev1,
    /// MEV-2 servicing Intelsat 10-02.
    Mev2,
}

/// The mass properties and arm geometry of an [`ScModel`].
#[derive(Clone, Copy, Debug)]
pub struct ScModelData {
    /// Upstream `scObject1.hub.mHub` \[kg\].
    pub chaser_mass_kg: f64,
    /// Upstream `scObject2.hub.mHub` \[kg\].
    pub target_mass_kg: f64,
    /// Upstream `r_P1B1_mag` \[m\]: chaser center of mass to connection point.
    pub r_p1b1_mag_m: f64,
    /// Upstream `r_P2B2_mag` \[m\]: target center of mass to connection point.
    pub r_p2b2_mag_m: f64,
    /// Upstream `r_P2P1_mag` \[m\]: arm length.
    pub r_p2p1_mag_m: f64,
}

impl ScModel {
    /// Upstream's model names, `bskSat`, `MEV1`, `MEV2`.
    pub fn label(self) -> &'static str {
        match self {
            Self::BskSat => "bskSat",
            Self::Mev1 => "MEV1",
            Self::Mev2 => "MEV2",
        }
    }

    pub fn parse(name: &str) -> Option<Self> {
        match name {
            "bskSat" => Some(Self::BskSat),
            "MEV1" => Some(Self::Mev1),
            "MEV2" => Some(Self::Mev2),
            _ => None,
        }
    }

    pub fn data(self) -> ScModelData {
        let (chaser_mass_kg, target_mass_kg, r_p1b1_mag_m, r_p2b2_mag_m, r_p2p1_mag_m) = match self
        {
            Self::BskSat => (750.0, 750.0, 0.1, 0.1, 0.1),
            Self::Mev1 => (2330.0, 4723.0, 1.0, 0.5, 0.5),
            Self::Mev2 => (2875.0, 3298.0, 1.0, 0.5, 0.5),
        };
        ScModelData {
            chaser_mass_kg,
            target_mass_kg,
            r_p1b1_mag_m,
            r_p2b2_mag_m,
            r_p2p1_mag_m,
        }
    }
}

/// Upstream `IHubPntBc_B = diag(900, 800, 600)` \[kg m^2\], every model's
/// hub inertia.
pub fn model_inertia_kg_m2() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0))
}

/// Where the constraint holds spacecraft 2 relative to spacecraft 1.
#[derive(Clone, Copy, Debug)]
pub struct ConstraintGeometry {
    /// Upstream `r_P1B1_B1` \[m\].
    pub r_p1b1_b1_m: Vector3<f64>,
    /// Upstream `r_P2B2_B2` \[m\].
    pub r_p2b2_b2_m: Vector3<f64>,
    /// Upstream `r_P2P1_B1Init` \[m\].
    pub r_p2p1_b1_init_m: Vector3<f64>,
}

impl ConstraintGeometry {
    /// The analysis scenarios' arms along `r_hat`: `r_P1B1 = |r_P1B1| r̂`,
    /// `r_P2B2 = -|r_P2B2| r̂`, `r_P2P1 = |r_P2P1| r̂`.
    pub fn along(r_hat: Vector3<f64>, model: &ScModelData) -> Self {
        Self {
            r_p1b1_b1_m: model.r_p1b1_mag_m * r_hat,
            r_p2b2_b2_m: -model.r_p2b2_mag_m * r_hat,
            r_p2p1_b1_init_m: model.r_p2p1_mag_m * r_hat,
        }
    }

    /// Upstream `r_B2B1 = r_P1B1_B1 + r_P2P1_B1Init - r_P2B2_B2`, the
    /// satisfied constraint's hub offset with the frames aligned.
    pub fn r_b2b1_m(&self) -> Vector3<f64> {
        self.r_p1b1_b1_m + self.r_p2p1_b1_init_m - self.r_p2b2_b2_m
    }

    /// Upstream's `ConstraintDynamicEffector` with this geometry, named
    /// `constraintEffector`, tuned with `alpha` and `beta`.
    pub fn constraint(&self, alpha: f64, beta: f64) -> ConstraintDynamicEffector {
        let mut constraint = ConstraintDynamicEffector::new(ConstraintDynamicEffectorConfig {
            connection_point_1_m: self.r_p1b1_b1_m,
            connection_point_2_m: self.r_p2b2_b2_m,
            separation_m: self.r_p2p1_b1_init_m,
            ..ConstraintDynamicEffectorConfig::new("constraintEffector")
        });
        constraint.set_alpha(alpha).expect("positive alpha");
        constraint.set_beta(beta).expect("positive beta");
        constraint
    }

    /// Upstream `psi_B1` of `process_data`: the direction violation from the
    /// logged hub states, B1 components.
    pub fn direction_violation_b1(
        &self,
        state_1: &SpacecraftStateMsg,
        state_2: &SpacecraftStateMsg,
    ) -> Vector3<f64> {
        let dcm_b1n = mrp_to_dcm(state_1.sigma_bn);
        let dcm_nb2 = mrp_to_dcm(state_2.sigma_bn).transpose();
        dcm_b1n * state_1.position_m + self.r_p1b1_b1_m + self.r_p2p1_b1_init_m
            - (dcm_b1n * state_2.position_m + dcm_b1n * dcm_nb2 * self.r_p2b2_b2_m)
    }
}

/// Upstream `sigma_B2B1 = subMRP(sigma_B2N, sigma_B1N)`.
pub fn attitude_violation(
    state_1: &SpacecraftStateMsg,
    state_2: &SpacecraftStateMsg,
) -> Vector3<f64> {
    sub_mrp(state_2.sigma_bn, state_1.sigma_bn)
}

/// Upstream's plotted attitude violation `4 arctan(sigma_B2B1) R2D` \[deg\],
/// componentwise.
pub fn violation_angles_deg(sigma_b2b1: Vector3<f64>) -> Vector3<f64> {
    sigma_b2b1.map(|component| (4.0 * component.atan()).to_degrees())
}

/// The constraint violations of a run, sample by sample.
#[derive(Clone, Debug, Default)]
pub struct Violations {
    /// Sample times \[ns\].
    pub times_nanos: Vec<u64>,
    /// Upstream `psi_B1` \[m\].
    pub direction_b1_m: Vec<Vector3<f64>>,
    /// Upstream `sigma_B2B1`.
    pub attitude_sigma_b2b1: Vec<Vector3<f64>>,
}

impl Violations {
    /// The violations of paired samples of both hub states.
    pub fn new(
        geometry: &ConstraintGeometry,
        states_1: &[(u64, SpacecraftStateMsg)],
        states_2: &[(u64, SpacecraftStateMsg)],
    ) -> Self {
        assert_eq!(states_1.len(), states_2.len(), "paired state samples");
        let mut violations = Self::default();
        for ((time_nanos, state_1), (time_2, state_2)) in states_1.iter().zip(states_2) {
            assert_eq!(time_nanos, time_2, "paired state samples");
            violations.times_nanos.push(*time_nanos);
            violations
                .direction_b1_m
                .push(geometry.direction_violation_b1(state_1, state_2));
            violations
                .attitude_sigma_b2b1
                .push(attitude_violation(state_1, state_2));
        }
        violations
    }

    /// The largest `|psi_B1|` \[m\].
    pub fn max_direction_m(&self) -> f64 {
        self.direction_b1_m
            .iter()
            .map(|psi| psi.norm())
            .fold(0.0, f64::max)
    }

    /// The largest `|4 arctan(sigma_B2B1)|` \[deg\].
    pub fn max_attitude_deg(&self) -> f64 {
        self.attitude_sigma_b2b1
            .iter()
            .map(|sigma| violation_angles_deg(*sigma).norm())
            .fold(0.0, f64::max)
    }

    /// The mean `|psi_B1|` \[m\] (upstream's gain-performance plot).
    pub fn mean_direction_m(&self) -> f64 {
        self.direction_b1_m
            .iter()
            .map(|psi| psi.norm())
            .sum::<f64>()
            / self.direction_b1_m.len() as f64
    }

    /// The mean `|4 arctan(sigma_B2B1)|` \[deg\].
    pub fn mean_attitude_deg(&self) -> f64 {
        self.attitude_sigma_b2b1
            .iter()
            .map(|sigma| violation_angles_deg(*sigma).norm())
            .sum::<f64>()
            / self.attitude_sigma_b2b1.len() as f64
    }
}

/// `r_<frame>N_N_m_i` and `sigma_<frame>N_i` of a spacecraft state message:
/// what the violations are computed from.
pub fn hub_pose_columns(frame: &str) -> CsvSourceConfig {
    CsvSourceConfig::columns(
        vector_columns("position_m", &format!("r_{frame}N_N_m"))
            .into_iter()
            .chain(vector_columns("sigma_bn", &format!("sigma_{frame}N"))),
    )
}

/// The `gain` as upstream's figure labels print it, `f'{gain:.0e}'`
/// without the exponent's sign and padding: `1e1`, `1e4`.
pub fn gain_label(gain: f64) -> String {
    format!("{gain:.0e}")
}

/// `r_<frame>N_N_m_i`, `v_<frame>N_N_m_per_s_i`, `sigma_<frame>N_i` and
/// `omega_<frame>N_<frame>_rad_per_s_i` of a spacecraft state message, the
/// columns the constrained scenarios write for each hub (`frame` is `B1`,
/// `B2`, ...).
pub fn hub_columns(frame: &str) -> CsvSourceConfig {
    CsvSourceConfig::columns(
        vector_columns("position_m", &format!("r_{frame}N_N_m"))
            .into_iter()
            .chain(vector_columns(
                "velocity_mps",
                &format!("v_{frame}N_N_m_per_s"),
            ))
            .chain(vector_columns("sigma_bn", &format!("sigma_{frame}N")))
            .chain(vector_columns(
                "omega_radps",
                &format!("omega_{frame}N_{frame}_rad_per_s"),
            )),
    )
}
