//! Port of upstream `examples/scenarioOrbitMultiBody.py`: the Hubble Space
//! Telescope or New Horizons propagated from its SPICE state under the Earth
//! (GGM03S to degree and order 100, the central body), Moon, Sun, Mars and
//! Jupiter, and compared with its SPICE trajectory.
//!
//! Needs the `spice` feature: the spacecraft trajectories are upstream's
//! `hst_edited.bsp`, a type-10 (two-line-element) SPK, and
//! `nh_pred_od077.bsp`, a big-endian SPK, which CSPICE reads and ANISE does
//! not. Since CSPICE is needed anyway, the planets come from it too, as
//! upstream: [`SpiceInterface`] with upstream's kernels (`de430.bsp`,
//! `naif0012.tls`, `de-403-masses.tpc`, `pck00010.tpc`, `just
//! fetch-spice-kernels`) plus the spacecraft SPK (`just
//! fetch-orbit-scenario-data` for New Horizons), `zeroBase = "Earth"`, from
//! 2012 May 1 00:28:30 UTC. The initial state is the spacecraft's SPICE state
//! at the start (upstream `spkRead`); the same interface publishes the SPICE
//! spacecraft state the run is compared with.
//!
//! ```text
//! cargo run --release --features spice --example scenario_orbit_multi_body -- --all
//! ```
//!
//! `--spacecraft Hubble|NewHorizons` (default Hubble); `--all` runs both cases
//! of `test_scenarioOrbitMultiBody.py`. Writes
//! `scenarioOrbitMultiBody<case>.csv`: the simulated state and the SPICE
//! state (`spice_r_BN_N_m_i`, `spice_v_BN_N_m_per_s_i`) of the spacecraft.
//!
//! `tests` ports the upstream check: after 2000 s the simulated position is
//! within 300 m of the SPICE position.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;

use std::path::PathBuf;

use basilisk_rs::environment::spice_interface::{SpiceInterface, SpiceInterfaceConfig};
use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::{Body, GGM03S};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::sec2nano;
use recording::{sampling_time, scenario_csv, state_columns};

const USAGE: &str = "\
scenario_orbit_multi_body [--spacecraft Hubble|NewHorizons]
scenario_orbit_multi_body --all     run both upstream test cases";

const STEP_NANOS: u64 = 5_000_000_000;

/// Upstream `scCase`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpacecraftCase {
    Hubble,
    NewHorizons,
}

impl SpacecraftCase {
    pub fn label(self) -> &'static str {
        match self {
            Self::Hubble => "Hubble",
            Self::NewHorizons => "NewHorizons",
        }
    }

    /// Upstream `scSpiceName` and `scEphemerisPath`.
    fn spice(self) -> (&'static str, &'static str) {
        match self {
            Self::Hubble => ("HUBBLE SPACE TELESCOPE", "assets/spice/hst_edited.bsp"),
            Self::NewHorizons => ("NEW HORIZONS", "assets/spice/nh_pred_od077.bsp"),
        }
    }
}

/// Upstream `timeInitString = "2012 MAY 1 00:28:30.0"`.
fn start_epoch() -> Epoch {
    Epoch::from_gregorian_utc(2012, 5, 1, 0, 28, 30, 0)
}

/// Upstream `createBodies("earth", "mars barycenter", "sun", "moon", "jupiter barycenter")`.
const BODIES: [Body; 5] = [
    Body::Earth,
    Body::MarsBarycenter,
    Body::Sun,
    Body::Moon,
    Body::JupiterBarycenter,
];

/// Upstream `createSpiceInterface(time, epochInMsg=True)` with `zeroBase =
/// "Earth"`, the factory's planets and `spicePlanetFrame`s, plus the
/// spacecraft SPK furnished by the scenario.
fn spice_interface(case: SpacecraftCase) -> SpiceInterface {
    let (name, spk) = case.spice();
    let mut kernel_paths: Vec<PathBuf> = [
        "assets/spice/de430.bsp",
        "assets/spice/naif0012.tls",
        "assets/spice/de-403-masses.tpc",
        "assets/spice/pck00010.tpc",
    ]
    .into_iter()
    .map(PathBuf::from)
    .collect();
    kernel_paths.push(PathBuf::from(spk));
    SpiceInterface::new(SpiceInterfaceConfig {
        name: "spiceObject".to_string(),
        kernel_paths,
        zero_base: "Earth".to_string(),
        planet_names: BODIES.iter().map(|body| body.name().to_string()).collect(),
        planet_frames: [
            "IAU_earth",
            "IAU_mars",
            "IAU_sun",
            "IAU_moon",
            "IAU_jupiter",
        ]
        .into_iter()
        .map(String::from)
        .collect(),
        spacecraft_names: vec![name.to_string()],
        ..SpiceInterfaceConfig::default()
    })
}

pub struct Results {
    /// Simulated and SPICE spacecraft states at the samples.
    pub simulated: Vec<(u64, SpacecraftStateMsg)>,
    pub spice: Vec<(u64, SpacecraftStateMsg)>,
    /// Upstream `rBSK` and `rTrue` (m).
    pub final_simulated_m: Vector3<f64>,
    pub final_spice_m: Vector3<f64>,
}

/// Upstream `run(show_plots, scCase)`.
pub fn run(case: SpacecraftCase) -> Results {
    let mut spice = spice_interface(case);
    // Upstream `spkRead(scSpiceName, timeInitString, "J2000", "EARTH")`.
    spice.init();
    spice.update(&SimulationContext {
        current_sim_nanos: 0,
        current_epoch: start_epoch(),
    });
    let initial = spice.sc_state_out_msgs[0].read();

    let duration_nanos = sec2nano(2000.0);
    let sample_nanos = sampling_time(duration_nanos, STEP_NANOS, 50);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: initial.position_m,
        initial_velocity_mps: initial.velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    for body in BODIES {
        let gravity = if body == Body::Earth {
            Body::Earth.spherical_harmonics(GGM03S, 100, true)
        } else {
            body.point_mass(false)
        };
        spacecraft
            .add_grav_body(gravity)
            .expect("unique gravity bodies");
    }

    let mut csv = scenario_csv(&format!("scenarioOrbitMultiBody{}", case.label()));
    let mut simulated = MessageLog::<SpacecraftStateMsg>::default();
    let mut spice_history = MessageLog::<SpacecraftStateMsg>::default();
    {
        let mut simulation = Simulation::new(start_epoch(), false);
        for (body, planet_msg) in BODIES.iter().zip(&spice.planet_state_out_msgs) {
            simulation.connect(
                planet_msg,
                spacecraft
                    .grav_body_input_mut(body.name())
                    .expect("gravity body"),
            );
        }
        connect!(&simulation,
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &spice.sc_state_out_msgs[0] => csv.add_source::<SpacecraftStateMsg>(
                CsvSourceConfig::columns(
                    vector_columns("position_m", "spice_r_BN_N_m")
                        .into_iter()
                        .chain(vector_columns("velocity_mps", "spice_v_BN_N_m_per_s")),
                ),
            ),
            &spacecraft.state_out => &mut simulated.input,
            &spice.sc_state_out_msgs[0] => &mut spice_history.input,
        );
        // The SPICE interface runs before the spacecraft: the gravity effector
        // takes the planet messages as written at the current update.
        schedule! { simulation,
            "spiceObject" => &mut spice, period=STEP_NANOS, priority=2;
            "bsk-Sat" => &mut spacecraft, period=STEP_NANOS, priority=1;
            "csv" => &mut csv, period=sample_nanos, priority=0;
            "simulated" => &mut simulated, period=sample_nanos, priority=0;
            "spice" => &mut spice_history, period=sample_nanos, priority=0;
        }
        simulation.run_for(duration_nanos);
    }
    csv.flush().expect("flush scenario CSV");

    let simulated = simulated.into_samples();
    let spice = spice_history.into_samples();
    Results {
        final_simulated_m: simulated.last().expect("samples").1.position_m,
        final_spice_m: spice.last().expect("samples").1.position_m,
        simulated,
        spice,
    }
}

fn main() {
    let mut args = Cli::from_env(USAGE);
    let cases = if args.switch("all") {
        vec![SpacecraftCase::Hubble, SpacecraftCase::NewHorizons]
    } else {
        match args.value("spacecraft", "Hubble".to_string()).as_str() {
            "Hubble" => vec![SpacecraftCase::Hubble],
            "NewHorizons" => vec![SpacecraftCase::NewHorizons],
            other => args.fail(&format!("unknown spacecraft {other:?}")),
        }
    };
    for case in cases {
        let results = run(case);
        println!(
            "wrote scenarioOrbitMultiBody{}.csv: final position {:.1} m from SPICE",
            case.label(),
            (results.final_simulated_m - results.final_spice_m).norm()
        );
    }
}

#[cfg(test)]
mod tests {
    use super::{SpacecraftCase, run};

    /// `test_scenarioOrbitMultiBody.py`, Hubble and New Horizons:
    /// `compareVector(rTrue, rBSK, 300.0)`, the norm of the difference.
    #[test]
    fn final_position_matches_spice() {
        for case in [SpacecraftCase::Hubble, SpacecraftCase::NewHorizons] {
            let results = run(case);
            let error = (results.final_simulated_m - results.final_spice_m).norm();
            assert!(error <= 300.0, "{}: |r_BN_N| error {error} m", case.label());
            assert_eq!(results.simulated.len(), results.spice.len());
        }
    }
}
