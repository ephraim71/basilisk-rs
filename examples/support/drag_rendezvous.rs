//! Upstream `scenarioDragRendezvous.drag_simulator`, shared by
//! `scenario_drag_rendezvous` and `scenario_drag_sensitivity`: two 6 kg
//! spacecraft, the chief "wiggum" and the deputy "lou", each with one 0.06 m²
//! drag plate, on a 300 km circular orbit. The chief's attitude follows its
//! Hill frame; the deputy's follows its Hill frame rotated by the relative
//! MRP `[G] x` of the Hill-frame relative state `x` (`hillToAttRef` with
//! upstream's static LQR gain, saturated at 0.2), so differential drag
//! steers it to the chief.
//!
//! Upstream prescribes both attitudes through `Spacecraft.attRefInMsg`, here
//! [`Spacecraft::att_ref_in`]: after each integration step the hub takes the
//! reference the guidance wrote at the previous step. The modules run in
//! upstream's priority order: atmosphere, spacecraft, navigation, Hill
//! state, references; the drag effectors read the densities during the
//! integration, the same values upstream's drag `UpdateState` reads just
//! before it.
//!
//! The atmosphere is upstream's exponential model with `planetRadius = R +
//! 300 km`, a 8 km scale height and the reach limits `+-300 km` about that
//! radius. basilisk-rs's [`ExponentialAtmosphere`] measures the altitude from
//! `R` and takes the reference altitude 300 km instead, the same density;
//! the reach limits never bind on this orbit. Upstream logs the drag force
//! of each effector's last integrator stage; here the spacecraft diagnostics
//! give it at the committed state.

#![allow(dead_code)]

use basilisk_rs::dynamics::facet_drag_dynamic_effector::{
    FacetDragDynamicEffector, FacetDragDynamicEffectorConfig,
};
use basilisk_rs::environment::atmosphere::exponential_atmosphere::{
    ExponentialAtmosphere, ExponentialAtmosphereConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::hill_point::{HillPoint, HillPointConfig};
use basilisk_rs::fsw_algorithms::formation_flying::hill_state_converter::{
    HillStateConverter, HillStateConverterConfig,
};
use basilisk_rs::fsw_algorithms::formation_flying::hill_to_att_ref::{
    HillToAttRef, HillToAttRefConfig,
};
use basilisk_rs::messages::{
    AtmosphereMsg, AttitudeReferenceMsg, HillRelativeStateMsg, SpacecraftDiagnosticsMsg,
    SpacecraftStateMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use hifitime::Epoch;
use nalgebra::{Matrix3, Matrix3x6, Vector3};

use super::common::vector_columns;
use super::grav_factory::{Body, GGM03S_J2_ONLY};
use super::message_log::MessageLog;
use super::orbit_tools::sec2nano;
use super::recording::scenario_csv;

/// Upstream `dynTimeStep` and `fswTimeStep`.
pub const STEP_NANOS: u64 = 60_000_000_000;

/// Upstream `dataForExamples/static_lqr_controlGain.npz` (`arr_0`, 3 x 6,
/// row-major).
pub const LQR_GAIN: [f64; 18] = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.029377303118632132,
    -0.0003741657386790044,
    0.0,
    1.7309242462202326,
    13.956521395165405,
    0.0,
];

/// Upstream `drag_simulator(altOffset, trueAnomOffset, densMultiplier,
/// ctrlType="lqr", useJ2, useWind=False)`. `ctrlType` only labels upstream's
/// printout (both use the static gain), and the wind case is not ported.
#[derive(Clone, Copy, Debug)]
pub struct RendezvousConfig {
    /// Deputy altitude offset from the chief (m).
    pub alt_offset_m: f64,
    /// Deputy true-anomaly lag behind the chief (deg).
    pub true_anomaly_offset_deg: f64,
    /// Scale factor on the atmosphere's base density.
    pub density_multiplier: f64,
    /// J2 gravity from `GGM03S-J2-only.txt` instead of a point mass.
    pub use_j2: bool,
}

/// Upstream's `results_dict`: every recorded series, one entry per 60 s
/// step.
#[derive(Clone, Debug, Default)]
pub struct RendezvousResults {
    /// The CSV written, if any.
    pub csv_path: Option<std::path::PathBuf>,
    /// `dynTimeData` (ns).
    pub times_nanos: Vec<u64>,
    /// `wiggum.*`: the chief's state.
    pub chief: Vec<SpacecraftStateMsg>,
    /// `lou.*`: the deputy's state.
    pub deputy: Vec<SpacecraftStateMsg>,
    /// `relState.*`: the deputy relative to the chief in the chief's Hill
    /// frame.
    pub hill: Vec<HillRelativeStateMsg>,
    /// `chiefDensity`, `depDensity` (kg/m^3).
    pub chief_density: Vec<f64>,
    pub deputy_density: Vec<f64>,
    /// `chiefDrag_B`, `depDrag_B` (N).
    pub chief_drag_body_n: Vec<Vector3<f64>>,
    pub deputy_drag_body_n: Vec<Vector3<f64>>,
    pub mu_m3ps2: f64,
    /// `dens_mult`.
    pub density_multiplier: f64,
}

/// Upstream `setup_spacecraft_plant`: the 6 kg hub with its drag plate and
/// its navigation.
fn spacecraft_plant(
    position_m: Vector3<f64>,
    velocity_mps: Vector3<f64>,
    name: &str,
    use_j2: bool,
) -> (Spacecraft, FacetDragDynamicEffector, SimpleNavigation) {
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 6.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(10.0, 9.0, 8.0)),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    let earth = if use_j2 {
        Body::Earth.spherical_harmonics(GGM03S_J2_ONLY, 2, true)
    } else {
        Body::Earth.point_mass(true)
    };
    spacecraft
        .add_grav_body(earth)
        .expect("one central gravity body");
    let mut drag = FacetDragDynamicEffector::new(FacetDragDynamicEffectorConfig {
        name: format!("{name}_dragEffector"),
        planet_rotation_rate_radps: Vector3::zeros(),
    });
    // `-euler3(45 deg) [0, -1, 0]`.
    let angle = 45.0_f64.to_radians();
    let normal = Vector3::new(angle.sin(), angle.cos(), 0.0);
    drag.add_facet(0.3 * 0.2, 2.2, normal, Vector3::zeros());
    (
        spacecraft,
        drag,
        SimpleNavigation::new(format!("{name}_navigator")),
    )
}

fn atmosphere(density_multiplier: f64) -> ExponentialAtmosphere {
    ExponentialAtmosphere::new(ExponentialAtmosphereConfig {
        name: "atmosphere".to_string(),
        planet_radius_m: Body::Earth.radius_equator_m(),
        reference_altitude_m: 300e3,
        reference_density_kgpm3: 2.022e-14 * 1000.0 * density_multiplier,
        scale_height_m: 8.0e3,
    })
}

fn state_columns(prefix: &str) -> CsvSourceConfig {
    CsvSourceConfig::columns(
        vector_columns("position_m", &format!("{prefix}_r_BN_N_m"))
            .into_iter()
            .chain(vector_columns(
                "velocity_mps",
                &format!("{prefix}_v_BN_N_m_per_s"),
            ))
            .chain(vector_columns("sigma_bn", &format!("{prefix}_sigma_BN"))),
    )
}

/// Runs upstream's `drag_simulator`, writing the histories to
/// `examples/output/scenarios/<stem>.csv` when `csv_stem` is given.
pub fn drag_simulator(config: RendezvousConfig, csv_stem: Option<&str>) -> RendezvousResults {
    let mu = Body::Earth.mu_m3ps2();
    let radius = Body::Earth.radius_equator_m() + 300e3;
    let elements = |semi_major_axis: f64, true_anomaly_deg: f64| ClassicElements {
        semi_major_axis,
        eccentricity: 0.0,
        inclination_rad: 45.0_f64.to_radians(),
        raan_rad: 20.0_f64.to_radians(),
        arg_periapsis_rad: 30.0_f64.to_radians(),
        true_anomaly_rad: true_anomaly_deg.to_radians(),
        ..Default::default()
    };
    let (chief_r, chief_v) = elements_to_state(mu, &elements(radius, 20.0));
    let (deputy_r, deputy_v) = elements_to_state(
        mu,
        &elements(
            radius + config.alt_offset_m,
            20.0 - config.true_anomaly_offset_deg,
        ),
    );
    let (mut chief, mut chief_drag, mut chief_nav) =
        spacecraft_plant(chief_r, chief_v, "wiggum", config.use_j2);
    let (mut deputy, mut deputy_drag, mut deputy_nav) =
        spacecraft_plant(deputy_r, deputy_v, "lou", config.use_j2);
    let mut chief_atmosphere = atmosphere(config.density_multiplier);
    let mut deputy_atmosphere = atmosphere(config.density_multiplier);

    let mut chief_att_ref = HillPoint::new(HillPointConfig {
        name: "chief_att_ref".to_string(),
    });
    let mut deputy_hill_ref = HillPoint::new(HillPointConfig {
        name: "dep_hill_ref".to_string(),
    });
    let mut hill_state = HillStateConverter::new(HillStateConverterConfig {
        name: "dep_hillStateNav".to_string(),
    });
    let mut deputy_att_ref = HillToAttRef::new(HillToAttRefConfig {
        name: "dep_att_ref".to_string(),
        gain_matrix: Matrix3x6::from_row_slice(&LQR_GAIN),
        rel_mrp_min: -0.2,
        rel_mrp_max: 0.2,
    });

    let duration_nanos = sec2nano(40.0 * 2.0 * std::f64::consts::PI / (mu / radius.powi(3)).sqrt());
    let mut csv = csv_stem.map(scenario_csv);
    let mut chief_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut deputy_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut hill_log = MessageLog::<HillRelativeStateMsg>::default();
    let mut chief_density_log = MessageLog::<AtmosphereMsg>::default();
    let mut deputy_density_log = MessageLog::<AtmosphereMsg>::default();
    let mut chief_drag_log = MessageLog::<SpacecraftDiagnosticsMsg>::default();
    let mut deputy_drag_log = MessageLog::<SpacecraftDiagnosticsMsg>::default();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        connect!(&simulation,
            &deputy.state_out => &mut deputy_atmosphere.input_state_msg,
            &chief.state_out => &mut chief_atmosphere.input_state_msg,
            &deputy_atmosphere.output_atmosphere_msg => &mut deputy_drag.input_atmosphere_msg,
            &chief_atmosphere.output_atmosphere_msg => &mut chief_drag.input_atmosphere_msg,
            &chief.state_out => &mut chief_nav.spacecraft_state_in_msg,
            &deputy.state_out => &mut deputy_nav.spacecraft_state_in_msg,
            &chief_nav.translation_out_msg => &mut chief_att_ref.trans_nav_in_msg,
            &chief_att_ref.att_ref_out_msg => &mut chief.att_ref_in,
            &deputy_nav.translation_out_msg => &mut deputy_hill_ref.trans_nav_in_msg,
            &deputy_nav.translation_out_msg => &mut hill_state.dep_state_in_msg,
            &chief_nav.translation_out_msg => &mut hill_state.chief_state_in_msg,
            &hill_state.hill_state_out_msg => &mut deputy_att_ref.hill_state_in_msg,
            &deputy_hill_ref.att_ref_out_msg => &mut deputy_att_ref.att_ref_in_msg,
            &deputy_att_ref.att_ref_out_msg => &mut deputy.att_ref_in,
            &chief.state_out => &mut chief_log.input,
            &deputy.state_out => &mut deputy_log.input,
            &hill_state.hill_state_out_msg => &mut hill_log.input,
            &chief_atmosphere.output_atmosphere_msg => &mut chief_density_log.input,
            &deputy_atmosphere.output_atmosphere_msg => &mut deputy_density_log.input,
            &chief.diagnostics_out => &mut chief_drag_log.input,
            &deputy.diagnostics_out => &mut deputy_drag_log.input,
        );
        if let Some(csv) = csv.as_mut() {
            connect!(&simulation,
                &chief.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns("wiggum")),
                &deputy.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns("lou")),
                &hill_state.hill_state_out_msg => csv.add_source::<HillRelativeStateMsg>(
                    CsvSourceConfig::columns(
                        vector_columns("position_dc_h_m", "r_DC_H_m")
                            .into_iter()
                            .chain(vector_columns("velocity_dc_h_mps", "v_DC_H_m_per_s")),
                    ),
                ),
                &chief_atmosphere.output_atmosphere_msg => csv.add_source::<AtmosphereMsg>(
                    CsvSourceConfig::columns([(
                        "neutral_density_kgpm3",
                        "chiefDensity_kg_per_m3",
                    )]),
                ),
                &deputy_atmosphere.output_atmosphere_msg => csv.add_source::<AtmosphereMsg>(
                    CsvSourceConfig::columns([(
                        "neutral_density_kgpm3",
                        "depDensity_kg_per_m3",
                    )]),
                ),
                &chief.diagnostics_out => csv.add_source::<SpacecraftDiagnosticsMsg>(
                    CsvSourceConfig::columns(vector_columns("drag_force_body_n", "chiefDrag_B_N")),
                ),
                &deputy.diagnostics_out => csv.add_source::<SpacecraftDiagnosticsMsg>(
                    CsvSourceConfig::columns(vector_columns("drag_force_body_n", "depDrag_B_N")),
                ),
                &deputy_att_ref.att_ref_out_msg => csv.add_source::<AttitudeReferenceMsg>(
                    CsvSourceConfig::columns(vector_columns("sigma_rn", "lou_sigma_RN")),
                ),
            );
        }
        chief.add_dynamic_effector(chief_drag);
        deputy.add_dynamic_effector(deputy_drag);
        // Upstream's model priorities: atmosphere 920, spacecraft 811/810,
        // navigation 801/800, Hill state 790, references 789/710/700,
        // recorders 707..700.
        schedule! { simulation,
            "deputy_atmosphere" => &mut deputy_atmosphere, period=STEP_NANOS, priority=920;
            "chief_atmosphere" => &mut chief_atmosphere, period=STEP_NANOS, priority=920;
            "deputy" => &mut deputy, period=STEP_NANOS, priority=811;
            "chief" => &mut chief, period=STEP_NANOS, priority=810;
            "deputy_nav" => &mut deputy_nav, period=STEP_NANOS, priority=801;
            "chief_nav" => &mut chief_nav, period=STEP_NANOS, priority=800;
            "hill_state" => &mut hill_state, period=STEP_NANOS, priority=790;
            "deputy_hill_ref" => &mut deputy_hill_ref, period=STEP_NANOS, priority=789;
            "chief_att_ref" => &mut chief_att_ref, period=STEP_NANOS, priority=710;
            "deputy_att_ref" => &mut deputy_att_ref, period=STEP_NANOS, priority=700;
            "chief_log" => &mut chief_log, period=STEP_NANOS, priority=600;
            "deputy_log" => &mut deputy_log, period=STEP_NANOS, priority=600;
            "hill_log" => &mut hill_log, period=STEP_NANOS, priority=600;
            "chief_density_log" => &mut chief_density_log, period=STEP_NANOS, priority=600;
            "deputy_density_log" => &mut deputy_density_log, period=STEP_NANOS, priority=600;
            "chief_drag_log" => &mut chief_drag_log, period=STEP_NANOS, priority=600;
            "deputy_drag_log" => &mut deputy_drag_log, period=STEP_NANOS, priority=600;
        }
        if let Some(csv) = csv.as_mut() {
            simulation.add_module("csv", csv, STEP_NANOS, 600);
        }
        simulation.run_for(duration_nanos);
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    RendezvousResults {
        csv_path: csv.map(|csv| csv.config.output_path.clone()),
        times_nanos: chief_log.times_nanos.clone(),
        chief: chief_log.values.iter().cloned().collect(),
        deputy: deputy_log.values.iter().cloned().collect(),
        hill: hill_log.values.iter().cloned().collect(),
        chief_density: chief_density_log
            .values
            .iter()
            .map(|m| m.neutral_density_kgpm3)
            .collect(),
        deputy_density: deputy_density_log
            .values
            .iter()
            .map(|m| m.neutral_density_kgpm3)
            .collect(),
        chief_drag_body_n: chief_drag_log
            .values
            .iter()
            .map(|m| m.drag_force_body_n)
            .collect(),
        deputy_drag_body_n: deputy_drag_log
            .values
            .iter()
            .map(|m| m.drag_force_body_n)
            .collect(),
        mu_m3ps2: mu,
        density_multiplier: config.density_multiplier,
    }
}
