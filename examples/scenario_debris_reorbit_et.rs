//! Port of upstream `examples/scenarioDebrisReorbitET.py`: an electrostatic
//! tractor. A 500 kg servicer charged to +25 kV holds a 2000 kg debris
//! object charged to -25 kV at 30 m on a GEO orbit; the Coulomb attraction
//! ([`MsmForceTorque`], one sphere each) pulls the debris along while the
//! servicer's thrust ([`EtSphericalControl`]) keeps the separation, raising
//! the debris' orbit.
//!
//! Both craft start on the same circular GEO orbit, the debris 50 m behind,
//! and are integrated for one orbit at 300 s steps; their states at every
//! step go to `scenarioDebrisReorbitET.csv` (`servicer_*` and `debris_*`
//! columns).
//!
//! Upstream's `extForceTorque` modules read their commands in `UpdateState`,
//! after the spacecraft in the dynamics task, and apply them during the next
//! integration. basilisk-rs's [`ExtForceTorque`] reads its commands during
//! the integration, which gives the same for the electrostatic forces (the
//! force module runs after both craft, as upstream). The control thrust,
//! though, comes from the flight-software process, after the dynamics: a
//! [`Latch`] where upstream's `extFTObjectServicerControl` runs keeps the
//! command upstream's effector would hold, one control step older.
//!
//! `test_scenarioDebrisReorbitET.py` only runs the scenario (and checks a
//! plotting helper); `tests` checks the tractor at work: the separation is
//! drawn to the 30 m reference and the debris' semi-major axis grows.

#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/latch.rs"]
mod latch;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/scenario_args.rs"]
mod scenario_args;

use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::msm_force_torque::{
    MsmForceTorque, MsmForceTorqueConfig, MsmSpaceObject,
};
use basilisk_rs::fsw_algorithms::formation_flying::et_spherical_control::{
    EtSphericalControl, EtSphericalControlConfig,
};
use basilisk_rs::messages::{
    InertialForceCommandMsg, Output, SpacecraftStateMsg, VehicleConfigMsg, VoltMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use basilisk_rs::{connect, schedule};
use common::vector_columns;
use grav_factory::Body;
use hifitime::Epoch;
use latch::Latch;
use nalgebra::{Matrix3, Vector3};
use recording::{History, scenario_csv};
use scenario_args::ScenarioArgs;

const USAGE: &str = "scenario_debris_reorbit_et     run the electrostatic tractor reorbit";

/// Upstream `simulationTimeStep` and `fswTimeStep`.
const STEP_NANOS: u64 = 300_000_000_000;
/// Upstream `L_r`: the reference separation (m).
pub const REFERENCE_SEPARATION_M: f64 = 30.0;

pub struct Results {
    pub csv_path: std::path::PathBuf,
    pub servicer: Vec<(u64, SpacecraftStateMsg)>,
    pub debris: Vec<(u64, SpacecraftStateMsg)>,
    /// Upstream's `oe`: the initial elements.
    pub initial_elements: ClassicElements,
    pub mu_m3ps2: f64,
}

fn spacecraft(mass_kg: f64, position_m: Vector3<f64>, velocity_mps: Vector3<f64>) -> Spacecraft {
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(Body::Earth.point_mass(true))
        .expect("one central gravity body");
    spacecraft
}

fn state_columns(prefix: &str) -> CsvSourceConfig {
    CsvSourceConfig::columns(
        vector_columns("position_m", &format!("{prefix}_r_BN_N_m"))
            .into_iter()
            .chain(vector_columns(
                "velocity_mps",
                &format!("{prefix}_v_BN_N_m_per_s"),
            )),
    )
}

pub fn run() -> Results {
    let mu = Body::Earth.mu_m3ps2();
    let (servicer_r, servicer_v) = elements_to_state(
        mu,
        &ClassicElements {
            semi_major_axis: 42_164.0 * 1e3,
            ..Default::default()
        },
    );
    let elements = state_to_elements(mu, servicer_r, servicer_v);
    // The debris 50 m behind the servicer, along track.
    let debris_r = servicer_r + Vector3::new(0.0, -50.0, 0.0);
    let period_s = 2.0 * std::f64::consts::PI / (mu / elements.semi_major_axis.powi(3)).sqrt();
    let duration_nanos = (period_s * 1e9 + 0.5) as u64;

    let servicer_mass_kg = 500.0;
    let debris_mass_kg = 2000.0;
    let mut servicer = spacecraft(servicer_mass_kg, servicer_r, servicer_v);
    let mut debris = spacecraft(debris_mass_kg, debris_r, servicer_v);
    let servicer_config = Output::new(VehicleConfigMsg {
        mass_kg: servicer_mass_kg,
        ..Default::default()
    });
    let debris_config = Output::new(VehicleConfigMsg {
        mass_kg: debris_mass_kg,
        ..Default::default()
    });

    let mut msm = MsmForceTorque::new(MsmForceTorqueConfig {
        name: "msmForceTorqueTag".to_string(),
        space_objects: vec![
            MsmSpaceObject {
                radii_m: vec![5.0],
                sphere_positions_body_m: vec![Vector3::zeros()],
            },
            MsmSpaceObject {
                radii_m: vec![4.0],
                sphere_positions_body_m: vec![Vector3::zeros()],
            },
        ],
    });
    let servicer_voltage = Output::new(VoltMsg {
        voltage_v: 25_000.0,
    });
    let debris_voltage = Output::new(VoltMsg {
        voltage_v: -25_000.0,
    });
    let mut servicer_electrostatic = ExtForceTorque::new(ExtForceTorqueConfig {
        name: "eForceServicer".to_string(),
        ..Default::default()
    });
    let mut debris_electrostatic = ExtForceTorque::new(ExtForceTorqueConfig {
        name: "eForceDebris".to_string(),
        ..Default::default()
    });
    let mut servicer_control = ExtForceTorque::new(ExtForceTorqueConfig {
        name: "controlServicer".to_string(),
        ..Default::default()
    });
    let mut control_latch = Latch::<InertialForceCommandMsg>::default();
    let mut servicer_nav = SimpleNavigation::new("SimpleNavigation");
    let mut debris_nav = SimpleNavigation::new("SimpleNavigation3");

    let ki: f64 = 4e-7;
    let pi = 1.85 * ki.sqrt();
    let mut et_control = EtSphericalControl::new(EtSphericalControlConfig {
        name: "ETcontrol".to_string(),
        mu_m3ps2: mu,
        l_r_m: REFERENCE_SEPARATION_M,
        theta_r_rad: 0.0,
        phi_r_rad: 0.0,
        k: Matrix3::from_diagonal_element(ki),
        p: Matrix3::from_diagonal_element(pi),
    });

    let mut csv = scenario_csv("scenarioDebrisReorbitET");
    let csv_path = csv.config.output_path.clone();
    let mut servicer_log = History::<SpacecraftStateMsg>::new();
    let mut debris_log = History::<SpacecraftStateMsg>::new();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        connect!(&simulation,
            &servicer.state_out => &mut msm.sc_state_in_msgs[0],
            &debris.state_out => &mut msm.sc_state_in_msgs[1],
            &servicer_voltage => &mut msm.volt_in_msgs[0],
            &debris_voltage => &mut msm.volt_in_msgs[1],
            &msm.e_force_out_msgs[0] => &mut servicer_electrostatic.cmd_force_inertial_in_msg,
            &msm.e_force_out_msgs[1] => &mut debris_electrostatic.cmd_force_inertial_in_msg,
            &et_control.force_inertial_out_msg => &mut control_latch.input,
            &control_latch.output => &mut servicer_control.cmd_force_inertial_in_msg,
            &servicer.state_out => &mut servicer_nav.spacecraft_state_in_msg,
            &debris.state_out => &mut debris_nav.spacecraft_state_in_msg,
            &servicer_nav.translation_out_msg => &mut et_control.servicer_trans_in_msg,
            &debris_nav.translation_out_msg => &mut et_control.debris_trans_in_msg,
            &servicer_nav.attitude_out_msg => &mut et_control.servicer_att_in_msg,
            &servicer_config => &mut et_control.servicer_vehicle_config_in_msg,
            &debris_config => &mut et_control.debris_vehicle_config_in_msg,
            &msm.e_force_out_msgs[0] => &mut et_control.e_force_in_msg,
            &servicer.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns("servicer")),
            &debris.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns("debris")),
            &servicer.state_out => &mut servicer_log.input,
            &debris.state_out => &mut debris_log.input,
        );
        servicer.add_dynamic_effector(servicer_electrostatic);
        servicer.add_dynamic_effector(servicer_control);
        debris.add_dynamic_effector(debris_electrostatic);
        // Upstream's dynamics task in order, then the flight-software process.
        schedule! { simulation,
            "servicer" => &mut servicer, period=STEP_NANOS, priority=100;
            "debris" => &mut debris, period=STEP_NANOS, priority=99;
            "msm" => &mut msm, period=STEP_NANOS, priority=98;
            "control_latch" => &mut control_latch, period=STEP_NANOS, priority=97;
            "servicer_nav" => &mut servicer_nav, period=STEP_NANOS, priority=96;
            "debris_nav" => &mut debris_nav, period=STEP_NANOS, priority=95;
            "csv" => &mut csv, period=STEP_NANOS, priority=90;
            "servicer_log" => &mut servicer_log, period=STEP_NANOS, priority=90;
            "debris_log" => &mut debris_log, period=STEP_NANOS, priority=90;
            "et_control" => &mut et_control, period=STEP_NANOS, priority=50;
        }
        simulation.run_for(duration_nanos);
    }
    csv.flush().expect("flush scenario CSV");
    Results {
        csv_path,
        servicer: servicer_log.into_samples(),
        debris: debris_log.into_samples(),
        initial_elements: elements,
        mu_m3ps2: mu,
    }
}

fn main() {
    ScenarioArgs::from_env(USAGE);
    let results = run();
    let (_, servicer) = results.servicer.last().unwrap();
    let (_, debris) = results.debris.last().unwrap();
    let debris_elements =
        state_to_elements(results.mu_m3ps2, debris.position_m, debris.velocity_mps);
    println!(
        "wrote {} (final separation {:.2} m, debris semi-major axis +{:.3} km)",
        results.csv_path.display(),
        (servicer.position_m - debris.position_m).norm(),
        (debris_elements.semi_major_axis - results.initial_elements.semi_major_axis) / 1000.0
    );
}

#[cfg(test)]
mod tests {
    use super::{REFERENCE_SEPARATION_M, run};
    use basilisk_rs::utilities::orbital_motion::state_to_elements;

    /// `test_scenarioDebrisReorbitET.py` only runs the scenario; the tractor
    /// must hold the debris near the 30 m reference and raise its orbit.
    #[test]
    fn tractor_holds_the_separation_and_raises_the_debris() {
        let results = run();
        let separation = |index: usize| {
            (results.servicer[index].1.position_m - results.debris[index].1.position_m).norm()
        };
        assert!((separation(0) - 50.0).abs() < 1e-6);
        let last = results.servicer.len() - 1;
        assert!(
            (separation(last) - REFERENCE_SEPARATION_M).abs() < 1.0,
            "final separation {} m",
            separation(last)
        );
        let (_, debris) = results.debris.last().unwrap();
        let elements = state_to_elements(results.mu_m3ps2, debris.position_m, debris.velocity_mps);
        assert!(
            elements.semi_major_axis > results.initial_elements.semi_major_axis,
            "debris semi-major axis {} m",
            elements.semi_major_axis
        );
    }
}
