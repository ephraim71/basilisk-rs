//! MSIS atmosphere, no-wind configuration from `scenarioDragDeorbit.py`.
//!
//! The simulation advances on a 15-second task cadence and stops on the first
//! task tick below 100 km. It writes the resulting state history to CSV.

#[path = "support/common.rs"]
mod common;

use std::path::PathBuf;

use anise::constants::frames::{EARTH_J2000, IAU_EARTH_FRAME};
use basilisk_rs::dynamics::drag_dynamic_effector::{
    DragDynamicEffector, DragDynamicEffectorConfig,
};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::environment::atmosphere::msis_atmosphere::{MsisAtmosphere, MsisAtmosphereConfig};
use basilisk_rs::messages::{
    AtmosphereMsg, Input, Output, PlanetOrientation, PlanetStateMsg, SpacecraftDiagnosticsMsg,
    SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::{CsvFormat, CsvRecorder, CsvRecorderConfig, CsvSourceConfig};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use common::{
    EARTH_EQUATORIAL_RADIUS_M, MU_EARTH_M3PS2, elem2rv, scenario_output_path, seconds,
    vector_columns,
};
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};

const TASK_PERIOD_NANOS: u64 = seconds(15);
const SAMPLE_PERIOD_NANOS: u64 = seconds(45);
const TERMINAL_ALTITUDE_M: f64 = 100_000.0;
const INITIAL_ALTITUDE_M: f64 = 250_000.0;
const PROJECTED_AREA_M2: f64 = 10.0;
const DRAG_COEFFICIENT: f64 = 2.2;

/// Models the separate upstream drag task: it reads the atmosphere after the
/// spacecraft task and the cached density is consumed on the next integration
/// tick. This one-task delay is observable in the recorded trajectory.
#[derive(Clone, Debug, Default)]
struct AtmosphereLatch {
    atmosphere_in_msg: Input<AtmosphereMsg>,
    atmosphere_out_msg: Output<AtmosphereMsg>,
}

impl Module for AtmosphereLatch {
    fn init(&mut self) {
        self.atmosphere_out_msg.write(AtmosphereMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.atmosphere_out_msg.write(self.atmosphere_in_msg.read());
    }
}

fn main() {
    let degrees = std::f64::consts::PI / 180.0;
    let semimajor_axis_m = EARTH_EQUATORIAL_RADIUS_M + INITIAL_ALTITUDE_M;
    let (initial_position_m, initial_velocity_mps) = elem2rv(
        MU_EARTH_M3PS2,
        semimajor_axis_m,
        0.0001,
        33.3 * degrees,
        48.2 * degrees,
        347.8 * degrees,
        85.3 * degrees,
    );

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        // The upstream example intentionally leaves the hub at its 1 kg default.
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: TASK_PERIOD_NANOS,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
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

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut atmosphere = MsisAtmosphere::new(MsisAtmosphereConfig {
        name: "MsisAtmo".to_string(),
        first_kernel_path: repo_root.join("assets/anise/pck11.pca"),
        additional_kernel_paths: vec![repo_root.join("assets/anise/earth_latest_high_prec.bpc")],
        inertial_frame: EARTH_J2000,
        fixed_frame: IAU_EARTH_FRAME,
        ap_array: [8.0; 7],
        f107_daily: 110.0,
        f107_average: 110.0,
    });
    // The source module defaults to an identity planet DCM when no ephemeris
    // message is supplied. Supplying that state explicitly avoids ANISE's
    // rotating-Earth fallback while retaining the shared MSIS module API.
    let planet_state = Output::new(PlanetStateMsg {
        position_inertial_m: Vector3::zeros(),
        velocity_inertial_mps: Vector3::zeros(),
        orientation: Some(PlanetOrientation::identity()),
    });

    let mut drag = DragDynamicEffector::new(DragDynamicEffectorConfig {
        name: "DragEff".to_string(),
        projected_area_m2: PROJECTED_AREA_M2,
        drag_coeff: DRAG_COEFFICIENT,
        com_offset_m: Vector3::zeros(),
        // `useWind=False`: upstream drag uses inertial spacecraft velocity.
        planet_rotation_rate_radps: Vector3::zeros(),
    });
    let mut atmosphere_latch = AtmosphereLatch::default();
    let start_epoch = Epoch::from_gregorian_utc_at_midnight(2019, 1, 1);
    let mut simulation = Simulation::new(start_epoch, false);
    connect!(&simulation,
        &atmosphere_latch.atmosphere_out_msg => &mut drag.input_atmosphere_msg,
    );
    spacecraft.add_dynamic_effector(drag);

    let state_handle = spacecraft.state_out.clone();
    let output_path = scenario_output_path("scenarioDragDeorbitmsis0.csv");
    let mut recorder = CsvRecorder::new(CsvRecorderConfig {
        topic: "scenarioDragDeorbitmsis0".to_string(),
        output_path: output_path.clone(),
    })
    .with_format(CsvFormat::Scenario);

    connect!(&simulation,
        &spacecraft.state_out => &mut atmosphere.input_state_msg,
        &planet_state => &mut atmosphere.input_planet_msg,
        &atmosphere.output_atmosphere_msg => &mut atmosphere_latch.atmosphere_in_msg,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(
                vector_columns("position_m", "r_BN_N_m")
                    .into_iter()
                    .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s")),
            ),
        ),
        &spacecraft.diagnostics_out => recorder.add_source::<SpacecraftDiagnosticsMsg>(
            CsvSourceConfig::columns(vector_columns(
                "drag_force_body_n",
                "forceExternal_B_N",
            )),
        ),
        &atmosphere.output_atmosphere_msg => recorder.add_source::<AtmosphereMsg>(
            CsvSourceConfig::columns([(
                "neutral_density_kgpm3",
                "neutralDensity_kg_per_m3",
            )]),
        ),
    );
    schedule! { simulation,
        "atmosphere" => &mut atmosphere, TASK_PERIOD_NANOS, 30;
        "spacecraft" => &mut spacecraft, TASK_PERIOD_NANOS, 20;
        "atmosphere_latch" => &mut atmosphere_latch, TASK_PERIOD_NANOS, 5;
        "recorder" => &mut recorder, SAMPLE_PERIOD_NANOS, 0;
    }

    let orbit_period_s =
        2.0 * std::f64::consts::PI * (semimajor_axis_m.powi(3) / MU_EARTH_M3PS2).sqrt();
    let maximum_duration_nanos = (100.0 * orbit_period_s * 1.0e9) as u64;
    let terminal_radius_m = EARTH_EQUATORIAL_RADIUS_M + TERMINAL_ALTITUDE_M;

    simulation.run_for(0);
    while simulation.current_sim_nanos() < maximum_duration_nanos
        && state_handle.read().position_m.norm() >= terminal_radius_m
    {
        simulation.run_for(TASK_PERIOD_NANOS);
    }

    let final_altitude_m = state_handle.read().position_m.norm() - EARTH_EQUATORIAL_RADIUS_M;
    assert!(
        final_altitude_m < TERMINAL_ALTITUDE_M,
        "spacecraft did not deorbit within the 100-orbit cap"
    );
    println!(
        "wrote {} (terminal tick {:.0} s, altitude {:.3} km)",
        output_path.display(),
        simulation.current_sim_nanos() as f64 * 1.0e-9,
        final_altitude_m / 1_000.0,
    );
}
