//! Port of upstream `examples/scenarioAerobrake.py`: a 100 kg cube with two
//! spring-damped hinged solar panels aerobrakes through a 135 km periapsis
//! of an equatorial 135 x 11,000 km orbit, in the exponential Earth
//! atmosphere, with facet drag, either holding a velocity-pointing attitude
//! (MRP feedback on an external torque) or tumbling free.
//!
//! Upstream runs four cases, `(useAttitudeControl, dragModel)`:
//!
//! | `--case` | control | drag model | here |
//! |---|---|---|---|
//! | `legacy-ctrl` | velocity pointing | panels baked into the hub's facets | ported |
//! | `legacy-free` | none | panels baked into the hub's facets | ported |
//! | `branched-ctrl` | velocity pointing | a drag effector on each panel | **not ported** |
//! | `branched-free` | none | a drag effector on each panel | **not ported** |
//!
//! The branched cases attach a `facetDragDynamicEffector` to each
//! `hingedRigidBodyStateEffector` (`panel.addDynamicEffector`), so the
//! panel's drag acts at the deflected panel and drives its hinge. basilisk-rs's
//! hinged rigid body has no attached-effector support (its docs list
//! `addDynamicEffector` as not ported, as for every state effector), and the
//! [`basilisk_rs::spacecraft::StateEffector`] interface gives an effector no
//! hub attitude to place an attached load with, so these two cases are
//! missing. The legacy model treats each panel as a rigid extension of the
//! hub for drag: two facets (both faces) at the undeflected panel center.
//!
//! The run lasts `--orbits` periods (default 2) at the `--dyn-rate-s` step
//! (default 0.2 s), recording every step to
//! `scenarioAerobrake_<model>_<ctrl|free>.csv`: the state, both panel
//! angles and the density. Upstream's drag effectors read the density in
//! `UpdateState`, after the spacecraft, and use it in the next integration:
//! a [`Latch`] after the spacecraft reproduces that delay. Without `--case`
//! the legacy cases run (upstream's `__main__` runs all four); `--all` runs
//! the ported case of `test_scenarioAerobrake.py`, `legacy + ctrl` at
//! 0.5 s for 1.2 orbits.
//!
//! The upstream test only runs the scenario; `tests` checks the aerobraking
//! its figures show: the periapsis pass lowers the apoapsis, the attitude
//! control holds the body axis `b2` on the velocity, and the panels deflect
//! in the drag pulse and settle.

#[path = "support/common.rs"]
mod common;
#[path = "support/latch.rs"]
mod latch;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/scenario_args.rs"]
mod scenario_args;

use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::facet_drag_dynamic_effector::{
    FacetDragDynamicEffector, FacetDragDynamicEffectorConfig,
};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::dynamics::hinged_rigid_body_state_effector::{
    HingedRigidBodyConfig, HingedRigidBodyStateEffector,
};
use basilisk_rs::environment::atmosphere::exponential_atmosphere::{
    ExponentialAtmosphere, ExponentialAtmosphereConfig,
};
use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::velocity_point::{
    VelocityPoint, VelocityPointConfig,
};
use basilisk_rs::messages::{
    AtmosphereMsg, EphemerisMsg, HingedRigidBodyMsg, Output, SpacecraftStateMsg, VehicleConfigMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_dcm};
use basilisk_rs::{Module, connect, schedule};
use common::vector_columns;
use hifitime::Epoch;
use latch::Latch;
use nalgebra::{Matrix3, Vector3};
use recording::{History, scenario_csv};
use scenario_args::ScenarioArgs;

const USAGE: &str = "\
scenario_aerobrake [--case legacy-ctrl|legacy-free] [--dyn-rate-s 0.2] [--orbits 2]
scenario_aerobrake --all     run the ported upstream test case (legacy-ctrl, 0.5 s, 1.2 orbits)";

const HUB_MASS_KG: f64 = 100.0;
const HUB_SIDE_M: f64 = 1.0;
const HUB_HALF_M: f64 = HUB_SIDE_M / 2.0;
const HUB_FACE_AREA_M2: f64 = HUB_SIDE_M * HUB_SIDE_M;
const HUB_INERTIA_KG_M2: f64 = HUB_MASS_KG / 6.0 * HUB_SIDE_M * HUB_SIDE_M;
const PANEL_MASS_KG: f64 = 10.0;
const PANEL_LENGTH_M: f64 = 2.0;
const PANEL_WIDTH_M: f64 = 1.0;
const PANEL_AREA_M2: f64 = PANEL_LENGTH_M * PANEL_WIDTH_M;
const PANEL_D_M: f64 = PANEL_LENGTH_M / 2.0;
const PANEL_STIFFNESS: f64 = 5.0;
const PANEL_DAMPING: f64 = 8.0;
const DRAG_COEFF_PANEL: f64 = 2.2;
const DRAG_COEFF_HUB: f64 = 2.2;
/// Upstream `earthRadius` (m), for the altitudes.
pub const EARTH_RADIUS_M: f64 = 6378.1363e3;
const PERIAPSIS_ALTITUDE_M: f64 = 135e3;
const APOAPSIS_ALTITUDE_M: f64 = 11000e3;
const START_TRUE_ANOMALY_DEG: f64 = -60.0;
/// Upstream `earthGrav.mu` (m^3/s^2).
pub const MU_M3PS2: f64 = 0.3986004415e15;
const ATT_CTRL_K: f64 = 5.0;
const ATT_CTRL_P: f64 = 50.0;

/// Upstream `dragModel`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DragModel {
    Legacy,
    Branched,
}

/// Upstream `runOneCase(useAttitudeControl, dragModel, dynRateSeconds,
/// nOrbitsToRun)`.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub attitude_control: bool,
    pub drag_model: DragModel,
    pub dyn_rate_s: f64,
    pub orbits: f64,
}

impl Config {
    /// Upstream's `caseSuffix`.
    pub fn file_stem(&self) -> String {
        let model = match self.drag_model {
            DragModel::Legacy => "legacy",
            DragModel::Branched => "branched",
        };
        let control = if self.attitude_control {
            "ctrl"
        } else {
            "free"
        };
        format!("scenarioAerobrake_{model}_{control}")
    }
}

/// The ported case of `test_scenarioAerobrake.py` (which also runs
/// `branched + ctrl`).
pub const TEST_CASE: Config = Config {
    attitude_control: true,
    drag_model: DragModel::Legacy,
    dyn_rate_s: 0.5,
    orbits: 1.2,
};

/// Upstream `panel1DCM`: the +X panel's hinge frame.
fn panel1_dcm() -> Matrix3<f64> {
    Matrix3::new(-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0)
}

/// Upstream `panel2DCM`: the -X panel, hinge along -Z.
fn panel2_dcm() -> Matrix3<f64> {
    Matrix3::new(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0)
}

/// Upstream `buildPanel`.
fn panel(name: &str, dcm_hb: Matrix3<f64>, r_hb_b: Vector3<f64>) -> HingedRigidBodyStateEffector {
    let mut config = HingedRigidBodyConfig::new(name);
    config.mass_kg = PANEL_MASS_KG;
    config.inertia_about_panel_com_panel_kg_m2 = Matrix3::from_diagonal(&Vector3::new(
        PANEL_MASS_KG * PANEL_WIDTH_M.powi(2) / 12.0,
        PANEL_MASS_KG * PANEL_LENGTH_M.powi(2) / 12.0,
        PANEL_MASS_KG * (PANEL_LENGTH_M.powi(2) + PANEL_WIDTH_M.powi(2)) / 12.0,
    ));
    config.center_of_mass_offset_m = PANEL_D_M;
    config.spring_constant_nm_per_rad = PANEL_STIFFNESS;
    config.damping_nm_s_per_rad = PANEL_DAMPING;
    config.hinge_position_body_m = r_hb_b;
    config.body_to_hinge_dcm = dcm_hb;
    config.theta_init_rad = 0.0;
    config.theta_dot_init_radps = 0.0;
    HingedRigidBodyStateEffector::new(config)
}

/// Upstream `addCubeFacets` and, for the legacy model,
/// `addRigidPanelFacetsToHub`.
fn hub_drag(drag_model: DragModel) -> FacetDragDynamicEffector {
    let mut drag = FacetDragDynamicEffector::new(FacetDragDynamicEffectorConfig {
        name: "hubDrag".to_string(),
        planet_rotation_rate_radps: Vector3::zeros(),
    });
    for axis in 0..3 {
        for sign in [1.0, -1.0] {
            let mut normal = Vector3::zeros();
            normal[axis] = sign;
            drag.add_facet(
                HUB_FACE_AREA_M2,
                DRAG_COEFF_HUB,
                normal,
                HUB_HALF_M * normal,
            );
        }
    }
    if drag_model == DragModel::Legacy {
        for sign in [1.0, -1.0] {
            let location = Vector3::new(sign * (HUB_HALF_M + PANEL_D_M), 0.0, 0.0);
            drag.add_facet(PANEL_AREA_M2, DRAG_COEFF_PANEL, Vector3::y(), location);
            drag.add_facet(PANEL_AREA_M2, DRAG_COEFF_PANEL, -Vector3::y(), location);
        }
    }
    drag
}

pub struct Results {
    pub csv_path: std::path::PathBuf,
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    pub theta1: Vec<f64>,
    pub theta2: Vec<f64>,
    pub density: Vec<f64>,
    /// Upstream `period` (s).
    pub period_s: f64,
}

impl Results {
    /// Upstream `pointingErrDeg`: the angle between `b2` and the velocity.
    pub fn pointing_error_deg(&self) -> Vec<f64> {
        self.states
            .iter()
            .map(|(_, state)| {
                let b2_n = mrp_to_dcm(state.sigma_bn).transpose() * Vector3::y();
                let cos = b2_n.dot(&state.velocity_mps.normalize()).clamp(-1.0, 1.0);
                cos.acos().to_degrees()
            })
            .collect()
    }

    /// Upstream `apoapsisAlt`: `2 a - min |r| - R` with the run's smallest
    /// radius.
    pub fn apoapsis_altitude_m(&self) -> Vec<f64> {
        let min_radius = self
            .states
            .iter()
            .map(|(_, state)| state.position_m.norm())
            .fold(f64::INFINITY, f64::min);
        self.states
            .iter()
            .map(|(_, state)| {
                let r = state.position_m.norm();
                let energy = 0.5 * state.velocity_mps.norm_squared() - MU_M3PS2 / r;
                2.0 * (-MU_M3PS2 / (2.0 * energy)) - min_radius - EARTH_RADIUS_M
            })
            .collect()
    }
}

pub fn run(config: Config) -> Results {
    assert!(
        config.drag_model == DragModel::Legacy,
        "the branched drag model needs dynamic effectors attached to the hinged panels, which \
         basilisk-rs does not support"
    );
    let step_nanos = (config.dyn_rate_s * 1e9 + 0.5) as u64;
    let r_peri = EARTH_RADIUS_M + PERIAPSIS_ALTITUDE_M;
    let r_apo = EARTH_RADIUS_M + APOAPSIS_ALTITUDE_M;
    let elements = ClassicElements {
        semi_major_axis: 0.5 * (r_peri + r_apo),
        eccentricity: (r_apo - r_peri) / (r_apo + r_peri),
        inclination_rad: 0.0,
        raan_rad: 0.0,
        arg_periapsis_rad: 0.0,
        true_anomaly_rad: START_TRUE_ANOMALY_DEG.to_radians(),
        ..Default::default()
    };
    let (r_n, v_n) = elements_to_state(MU_M3PS2, &elements);
    // b2 along the velocity, b3 along the orbit normal.
    let v_hat = v_n.normalize();
    let h_hat = r_n.cross(&v_n).normalize();
    let in_plane = v_hat.cross(&h_hat);
    let dcm_bn = Matrix3::from_rows(&[in_plane.transpose(), v_hat.transpose(), h_hat.transpose()]);
    let orbit_rate = r_n.cross(&v_n).norm() / r_n.dot(&r_n);

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal_element(HUB_INERTIA_KG_M2),
        integration_step_nanos: step_nanos,
        initial_position_m: r_n,
        initial_velocity_mps: v_n,
        initial_sigma_bn: dcm_to_mrp(dcm_bn),
        initial_omega_radps: Vector3::new(0.0, 0.0, orbit_rate),
        integrator: None,
    });
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass(
                "earth_planet_data",
                MU_M3PS2,
                true,
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .expect("valid point-mass Earth"),
        )
        .expect("one central gravity body");
    let panel1 = panel("panel1", panel1_dcm(), Vector3::new(HUB_HALF_M, 0.0, 0.0));
    let panel2 = panel("panel2", panel2_dcm(), Vector3::new(-HUB_HALF_M, 0.0, 0.0));

    // Upstream `simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")`.
    let mut atmosphere = ExponentialAtmosphere::new(ExponentialAtmosphereConfig {
        name: "expAtmo".to_string(),
        planet_radius_m: 6_378_136.6,
        reference_altitude_m: 0.0,
        reference_density_kgpm3: 1.217,
        scale_height_m: 8_500.0,
    });
    let mut density_latch = Latch::<AtmosphereMsg>::default();
    let mut drag = hub_drag(config.drag_model);

    let mut modules: Vec<(&'static str, Box<dyn Module>, i32)> = Vec::new();
    let mut control_torque = ExtForceTorque::new(ExtForceTorqueConfig {
        name: "ctrlTorque".to_string(),
        ..Default::default()
    });
    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    if config.attitude_control {
        let mut navigation = SimpleNavigation::new("sNav");
        let earth_ephemeris = Output::new(EphemerisMsg::default());
        let mut velocity_point = VelocityPoint::new(VelocityPointConfig {
            name: "velPoint".to_string(),
            mu_m3ps2: MU_M3PS2,
        });
        let mut tracking_error = AttTrackingError::new(AttTrackingErrorConfig {
            name: "attErr".to_string(),
            sigma_r0r: Vector3::zeros(),
        });
        let vehicle_config = Output::new(VehicleConfigMsg {
            inertia_about_point_b_kg_m2: Matrix3::from_diagonal_element(HUB_INERTIA_KG_M2),
            ..Default::default()
        });
        let ki = -1.0_f64;
        let mut feedback = MrpFeedback::new(MrpFeedbackConfig {
            name: "mrpFb".to_string(),
            k: ATT_CTRL_K,
            ki,
            p: ATT_CTRL_P,
            integral_limit: 2.0 / ki.abs() * 0.1,
            known_torque_body_nm: Vector3::zeros(),
            control_law_type: 0,
        });
        connect!(&simulation,
            &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
            &navigation.translation_out_msg => &mut velocity_point.trans_nav_in_msg,
            &earth_ephemeris => &mut velocity_point.cel_body_in_msg,
            &velocity_point.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
            &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
            &tracking_error.att_guid_out_msg => &mut feedback.guid_in_msg,
            &vehicle_config => &mut feedback.veh_config_in_msg,
            &feedback.cmd_torque_out_msg => &mut control_torque.cmd_torque_in_msg,
        );
        modules.push(("sNav", Box::new(navigation), 15));
        modules.push(("velPoint", Box::new(velocity_point), 14));
        modules.push(("attErr", Box::new(tracking_error), 13));
        modules.push(("mrpFb", Box::new(feedback), 12));
    }

    let mut csv = scenario_csv(&config.file_stem());
    let csv_path = csv.config.output_path.clone();
    let mut state_log = History::<SpacecraftStateMsg>::new();
    let mut theta1_log = History::<HingedRigidBodyMsg>::new();
    let mut theta2_log = History::<HingedRigidBodyMsg>::new();
    let mut density_log = History::<AtmosphereMsg>::new();
    let period_s =
        2.0 * std::f64::consts::PI * (elements.semi_major_axis.powi(3) / MU_M3PS2).sqrt();
    {
        connect!(&simulation,
            &spacecraft.state_out => &mut atmosphere.input_state_msg,
            &atmosphere.output_atmosphere_msg => &mut density_latch.input,
            &density_latch.output => &mut drag.input_atmosphere_msg,
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(
                CsvSourceConfig::columns(
                    vector_columns("position_m", "r_BN_N_m")
                        .into_iter()
                        .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s"))
                        .chain(vector_columns("sigma_bn", "sigma_BN")),
                ),
            ),
            &panel1.hinged_rigid_body_out => csv.add_source::<HingedRigidBodyMsg>(
                CsvSourceConfig::columns([("theta_rad", "panel1_theta_rad")]),
            ),
            &panel2.hinged_rigid_body_out => csv.add_source::<HingedRigidBodyMsg>(
                CsvSourceConfig::columns([("theta_rad", "panel2_theta_rad")]),
            ),
            &atmosphere.output_atmosphere_msg => csv.add_source::<AtmosphereMsg>(
                CsvSourceConfig::columns([(
                    "neutral_density_kgpm3",
                    "neutralDensity_kg_per_m3",
                )]),
            ),
            &spacecraft.state_out => &mut state_log.input,
            &panel1.hinged_rigid_body_out => &mut theta1_log.input,
            &panel2.hinged_rigid_body_out => &mut theta2_log.input,
            &atmosphere.output_atmosphere_msg => &mut density_log.input,
        );
        spacecraft.add_state_effector(panel1);
        spacecraft.add_state_effector(panel2);
        spacecraft.add_dynamic_effector(drag);
        if config.attitude_control {
            spacecraft.add_dynamic_effector(control_torque);
        }
        schedule! { simulation,
            "atmosphere" => &mut atmosphere, period=step_nanos, priority=30;
            "spacecraft" => &mut spacecraft, period=step_nanos, priority=20;
            "density_latch" => &mut density_latch, period=step_nanos, priority=19;
            "csv" => &mut csv, period=step_nanos, priority=0;
            "state_log" => &mut state_log, period=step_nanos, priority=0;
            "theta1_log" => &mut theta1_log, period=step_nanos, priority=0;
            "theta2_log" => &mut theta2_log, period=step_nanos, priority=0;
            "density_log" => &mut density_log, period=step_nanos, priority=0;
        }
        for (name, module, priority) in modules.iter_mut() {
            simulation.add_module(*name, module.as_mut(), step_nanos, *priority);
        }
        simulation.run_for((config.orbits * period_s * 1e9 + 0.5) as u64);
    }
    drop(simulation);
    csv.flush().expect("flush scenario CSV");
    Results {
        csv_path,
        states: state_log.into_samples(),
        theta1: theta1_log.messages().map(|m| m.theta_rad).collect(),
        theta2: theta2_log.messages().map(|m| m.theta_rad).collect(),
        density: density_log
            .messages()
            .map(|m| m.neutral_density_kgpm3)
            .collect(),
        period_s,
    }
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let configs = if args.flag("--all") {
        vec![TEST_CASE]
    } else {
        let dyn_rate_s = args.value("--dyn-rate-s", 0.2);
        let orbits = args.value("--orbits", 2.0);
        let case = |attitude_control, drag_model| Config {
            attitude_control,
            drag_model,
            dyn_rate_s,
            orbits,
        };
        match args.value("--case", String::new()).as_str() {
            "" => vec![
                case(true, DragModel::Legacy),
                case(false, DragModel::Legacy),
            ],
            "legacy-ctrl" => vec![case(true, DragModel::Legacy)],
            "legacy-free" => vec![case(false, DragModel::Legacy)],
            "branched-ctrl" | "branched-free" => args.fail(
                "the branched drag model is not ported: basilisk-rs's hinged rigid body has no \
                 attached dynamic effectors",
            ),
            other => args.fail(&format!("unknown case {other:?}")),
        }
    };
    for config in configs {
        let results = run(config);
        let apoapsis = results.apoapsis_altitude_m();
        println!(
            "wrote {} (apoapsis altitude {:.1} -> {:.1} km, max |theta1| {:.2} deg)",
            results.csv_path.display(),
            apoapsis[0] / 1000.0,
            apoapsis[apoapsis.len() - 1] / 1000.0,
            results
                .theta1
                .iter()
                .fold(0.0_f64, |m, t| m.max(t.abs()))
                .to_degrees()
        );
    }
}

#[cfg(test)]
mod tests {
    use super::{TEST_CASE, run};

    /// `test_scenarioAerobrake.py` runs `branched + ctrl` (not ported) and
    /// `legacy + ctrl` at 0.5 s for 1.2 orbits without checks. The periapsis
    /// pass must lower the apoapsis, the controller hold `b2` on the
    /// velocity, and the panels deflect in the drag pulse and settle.
    #[test]
    fn periapsis_pass_lowers_the_apoapsis() {
        let results = run(TEST_CASE);
        let apoapsis = results.apoapsis_altitude_m();
        let drop_km = (apoapsis[0] - apoapsis[apoapsis.len() - 1]) / 1000.0;
        assert!(drop_km > 1000.0, "apoapsis drop {drop_km} km");
        let pointing = results.pointing_error_deg();
        let worst = pointing.iter().fold(0.0_f64, |m, e| m.max(*e));
        assert!(worst < 0.5, "pointing error up to {worst} deg");
        for theta in [&results.theta1, &results.theta2] {
            let peak = theta
                .iter()
                .fold(0.0_f64, |m, t| m.max(t.abs()))
                .to_degrees();
            assert!(peak > 10.0, "panel deflection peak {peak} deg");
            let last = theta[theta.len() - 1].abs().to_degrees();
            assert!(last < 0.5, "panel deflection at the end {last} deg");
        }
        assert!(results.density.iter().any(|rho| *rho > 1e-8));
    }
}
