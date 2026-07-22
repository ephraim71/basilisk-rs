//! Parity case for `scenarioBasicOrbit.py`: LEO, point-mass Earth.
//!
//! The binary intentionally performs no plotting. It writes the deterministic
//! simulation result to `examples/output/scenarios/scenarioBasicOrbitLEO0Earth.csv`.

mod common;

use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::messages::{Input, Output, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::{
    CsvFormat, CsvRecorder, CsvRecorderConfig, TelemetryField, TelemetryMessage,
};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use common::{MU_EARTH_M3PS2, elem2rv, scenario_output_path, seconds, vector_fields};
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};

const SPACECRAFT_PERIOD_NANOS: u64 = seconds(10);
const SAMPLE_PERIOD_NANOS: u64 = seconds(40);

#[derive(Clone, Debug, Default)]
struct BasicOrbitTelemetry {
    state: SpacecraftStateMsg,
}

impl TelemetryMessage for BasicOrbitTelemetry {
    fn flatten(&self) -> Vec<TelemetryField> {
        let mut fields = vector_fields("r_BN_N_m", self.state.position_m);
        fields.extend(vector_fields("v_BN_N_m_per_s", self.state.velocity_mps));
        fields
    }
}

#[derive(Clone, Debug, Default)]
struct BasicOrbitCollector {
    state_in_msg: Input<SpacecraftStateMsg>,
    telemetry_out_msg: Output<BasicOrbitTelemetry>,
}

impl Module for BasicOrbitCollector {
    fn init(&mut self) {
        self.telemetry_out_msg.write(BasicOrbitTelemetry::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.telemetry_out_msg.write(BasicOrbitTelemetry {
            state: self.state_in_msg.read(),
        });
    }
}

fn main() {
    let degrees = std::f64::consts::PI / 180.0;
    let semimajor_axis_m = 7_000_000.0;
    let eccentricity = 0.0001;
    let (initial_position_m, initial_velocity_mps) = elem2rv(
        MU_EARTH_M3PS2,
        semimajor_axis_m,
        eccentricity,
        33.3 * degrees,
        48.2 * degrees,
        347.8 * degrees,
        85.3 * degrees,
    );
    let orbit_period_s =
        2.0 * std::f64::consts::PI * (semimajor_axis_m.powi(3) / MU_EARTH_M3PS2).sqrt();
    let duration_nanos = (0.75 * orbit_period_s * 1.0e9) as u64;

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 100.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: SPACECRAFT_PERIOD_NANOS,
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

    let mut collector = BasicOrbitCollector::default();
    let output_path = scenario_output_path("scenarioBasicOrbitLEO0Earth.csv");
    let mut recorder = CsvRecorder::new(CsvRecorderConfig {
        topic: "scenarioBasicOrbitLEO0Earth".to_string(),
        output_path: output_path.clone(),
    })
    .with_format(CsvFormat::Reference);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &spacecraft.state_out => &mut collector.state_in_msg,
        &collector.telemetry_out_msg => &mut recorder.input_msg,
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, SPACECRAFT_PERIOD_NANOS, 20;
        "collector" => &mut collector, SAMPLE_PERIOD_NANOS, 10;
        "recorder" => &mut recorder, SAMPLE_PERIOD_NANOS, 0;
    }
    simulation.run_for(duration_nanos);

    println!("wrote {}", output_path.display());
}
