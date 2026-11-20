//! Mean orbital-element feedback of a deputy relative to a chief: port of
//! upstream `examples/scenarioFormationMeanOEFeedback.py`.
//!
//! Two 500 kg rigid spacecraft orbit a GGM03S degree-2 Earth, the chief on
//! `a = 11 000 km`, `e = 0.4`, `i = 10°`, `ω = 70°`, `M = 0`, the deputy on
//! slightly offset elements. `meanOEFeedback` (`K = 1e7 I`, J2 mean elements)
//! drives the deputy's mean element difference to
//! `[0, 0, 0, 0.0003, 0.0002, 0.0001]` through an `extForceTorque` inertial
//! force on the deputy. One 15 s dynamics task and one 15 s FSW task, the
//! dynamics first, as upstream's two process priorities order them.
//!
//! The Earth's orientation feeds the tesseral terms of the field. Upstream
//! takes it from `createSpiceInterface(time="2012 MAY 1 00:28:30.0")` with
//! `zeroBase = "Earth"`; here the ANISE ephemeris (DE440s, `IAU_EARTH` from
//! `pck11.pca`, the same rotation model as `pck00010.tpc` to 1e-11) publishes
//! the same state, see `support/grav_factory.rs`.
//!
//! [`HeldOrientation`] passes that state on to the gravity bodies with a zero
//! orientation rate, so the field keeps the orientation of the current tick
//! through each 15 s step instead of rotating it back to the RK4 stage times.
//! Upstream's truth values need this: with the rate extrapolated the chief
//! drifts 20 m (5e-6 relative) from them within an orbit, equally for the
//! rate and its negative, while with it held both element sets match to
//! 2e-7 (the truth predates the stage-time extrapolation of upstream's
//! current `GravBodyData::computePlanetOrientation`).
//!
//! Upstream `run(show_plots, useClassicElem, numOrbits)` maps to
//! `--[no-]classic-elem` (feedback in classical or equinoctial elements) and
//! `--orbits` (the upstream script's `__main__` runs 40; its test runs 1).
//! `--all` runs both element sets for one orbit, as
//! `test_scenarioFormationMeanOEFeedback.py` does.
//!
//! Writes `examples/output/scenarios/scenarioFormationMeanOEFeedback<classic>.csv`
//! (`<classic>` is `int(useClassicElem)`) with the chief and deputy states;
//! `plot_scenario_formation_mean_oe_feedback.py` redraws upstream's mean
//! element difference figures from it. Reads `assets/gravity/GGM03S.txt`
//! (`just fetch-gravity`) and the ANISE kernels (`just fetch-anise-assets`).

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/formation.rs"]
mod formation;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;

use std::path::PathBuf;

use attitude::scenario_recorder;
use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::fsw_algorithms::formation_flying::mean_oe_feedback::{
    MeanOeFeedback, MeanOeFeedbackConfig, OrbitalElementSet,
};
use basilisk_rs::messages::{Input, Output, PlanetStateMsg, SpacecraftStateMsg};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::utilities::astro_constants::{J2_EARTH, MU_EARTH, REQ_EARTH};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use cli::Cli;
use formation::{
    InitialState, hub_inertia_kg_m2, mean_to_true_anomaly, rigid_spacecraft,
    spacecraft_state_columns,
};
use grav_factory::{Body, GGM03S};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Matrix6, Vector6};
use orbit_tools::{orbit_period_s, sec2nano};
use recording::sampling_time;

const USAGE: &str = "\
scenario_formation_mean_oe_feedback [--[no-]classic-elem] [--orbits N]
scenario_formation_mean_oe_feedback --all   both element sets, one orbit (upstream's test)";

/// Upstream `dynTimeStep` and `fswTimeStep = macros.sec2nano(15.0)`.
const STEP_NANOS: u64 = 15_000_000_000;
/// Upstream `numDataPoints`.
const NUM_DATA_POINTS: u64 = 1000;
/// Upstream `scObject.hub.mHub` (both spacecraft) \[kg\].
const HUB_MASS_KG: f64 = 500.0;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    use_classic_elem: bool,
    num_orbits: f64,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(use_classic_elem: bool, num_orbits: f64) -> Self {
        Self {
            use_classic_elem,
            num_orbits,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!(
            "scenarioFormationMeanOEFeedback{}",
            u8::from(self.use_classic_elem)
        )
    }
}

/// The parametrizations of upstream `test_scenarioFormationMeanOEFeedback.py`.
const TEST_PARAMETRIZATIONS: [Config; 2] = [Config::new(true, 1.0), Config::new(false, 1.0)];

/// Upstream's returned `dataPos, dataVel` (chief) and `dataPos2, dataVel2`
/// (deputy) recorder histories.
struct Results {
    chief: MessageLog<SpacecraftStateMsg>,
    deputy: MessageLog<SpacecraftStateMsg>,
    csv_path: Option<PathBuf>,
}

/// Upstream's chief elements: `a = 11 000 km`, `e = 0.4`, `i = 10°`,
/// `Ω = 0`, `ω = 70°`, `f` from `M = 0`.
fn chief_elements() -> ClassicElements {
    let degrees = std::f64::consts::PI / 180.0;
    let eccentricity = 0.4;
    ClassicElements {
        semi_major_axis: 11_000.0 * 1e3,
        eccentricity,
        inclination_rad: 10.0 * degrees,
        raan_rad: 0.0,
        arg_periapsis_rad: 70.0 * degrees,
        true_anomaly_rad: mean_to_true_anomaly(0.0, eccentricity),
        ..Default::default()
    }
}

/// Upstream's deputy elements: the chief's offset by
/// `[a·1e-4, 2e-4, -3e-4, 4e-4, 5e-4]`, `M + 6e-4` (converted to `f` with the
/// chief's eccentricity, as upstream does).
fn deputy_elements(chief: &ClassicElements) -> ClassicElements {
    ClassicElements {
        semi_major_axis: chief.semi_major_axis * (1.0 + 0.0001),
        eccentricity: chief.eccentricity + 0.0002,
        inclination_rad: chief.inclination_rad - 0.0003,
        raan_rad: chief.raan_rad + 0.0004,
        arg_periapsis_rad: chief.arg_periapsis_rad + 0.0005,
        true_anomaly_rad: mean_to_true_anomaly(0.0 + 0.0006, chief.eccentricity),
        ..Default::default()
    }
}

/// The planet state of `input` with its orientation rate zeroed: the
/// orientation the gravity field holds through each integration step (see
/// the module docs).
#[derive(Debug, Default)]
struct HeldOrientation {
    input: Input<PlanetStateMsg>,
    output: Output<PlanetStateMsg>,
}

impl Module for HeldOrientation {
    fn init(&mut self) {}

    fn update(&mut self, _context: &SimulationContext) {
        let mut state = self.input.read();
        if let Some(orientation) = state.orientation.as_mut() {
            orientation.inertial_to_fixed_dot = Matrix3::zeros();
        }
        self.output.write(state);
    }
}

fn run(config: Config) -> Results {
    let mu = Body::Earth.mu_m3ps2();
    let inertia = hub_inertia_kg_m2();
    let chief_oe = chief_elements();
    let deputy_oe = deputy_elements(&chief_oe);

    let mut chief = rigid_spacecraft(
        HUB_MASS_KG,
        inertia,
        STEP_NANOS,
        &InitialState::at_rest(elements_to_state(mu, &chief_oe)),
    );
    let mut deputy = rigid_spacecraft(
        HUB_MASS_KG,
        inertia,
        STEP_NANOS,
        &InitialState::at_rest(elements_to_state(mu, &deputy_oe)),
    );
    for spacecraft in [&mut chief, &mut deputy] {
        spacecraft
            .add_grav_body(Body::Earth.spherical_harmonics(GGM03S, 2, true))
            .expect("one central body");
    }
    let mut earth_ephemeris = Body::Earth.ephemeris(Body::Earth);
    let mut earth_orientation = HeldOrientation::default();

    let mut external_force = ExtForceTorque::new(ExtForceTorqueConfig {
        name: "externalDisturbance2".to_string(),
        ..Default::default()
    });
    let mut chief_navigation = SimpleNavigation::new("simpleNavObject");
    let mut deputy_navigation = SimpleNavigation::new("simpleNavObject2");

    let mut feedback = MeanOeFeedback::new(MeanOeFeedbackConfig {
        name: "meanOEFeedback".to_string(),
        k: Matrix6::identity() * 1e7,
        target_diff_oe_mean: Vector6::new(0.000, 0.000, 0.000, 0.0003, 0.0002, 0.0001),
        oe_type: if config.use_classic_elem {
            OrbitalElementSet::Classic
        } else {
            OrbitalElementSet::Equinoctial
        },
        mu_m3ps2: MU_EARTH * 1e9,
        req_m: REQ_EARTH * 1e3,
        j2: J2_EARTH,
    });

    let orbit_period = orbit_period_s(mu, chief_oe.semi_major_axis);
    let duration_nanos = sec2nano(orbit_period * config.num_orbits);
    // A recorder period below the task's samples every task step.
    let sample_nanos = sampling_time(duration_nanos, STEP_NANOS, NUM_DATA_POINTS).max(STEP_NANOS);
    let mut chief_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut deputy_log = MessageLog::<SpacecraftStateMsg>::default();
    let (mut csv, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    {
        let start = Epoch::from_gregorian_utc(2012, 5, 1, 0, 28, 30, 0);
        let mut simulation = Simulation::new(start, false);
        connect!(&simulation,
            &feedback.force_out_msg => &mut external_force.cmd_force_inertial_in_msg,
        );
        deputy.add_dynamic_effector(external_force);
        connect!(&simulation,
            &earth_ephemeris.output_planet_msg => &mut earth_orientation.input,
            &earth_orientation.output => chief.grav_body_input_mut("earth").expect("earth"),
            &earth_orientation.output => deputy.grav_body_input_mut("earth").expect("earth"),
            &chief.state_out => &mut chief_navigation.spacecraft_state_in_msg,
            &deputy.state_out => &mut deputy_navigation.spacecraft_state_in_msg,
            &chief_navigation.translation_out_msg => &mut feedback.chief_trans_in_msg,
            &deputy_navigation.translation_out_msg => &mut feedback.deputy_trans_in_msg,
            &chief.state_out => &mut chief_log.input,
            &deputy.state_out => &mut deputy_log.input,
            &chief.state_out => csv.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("chief")),
            &deputy.state_out => csv.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("deputy")),
        );
        // Dynamics process (upstream priority 2), then the FSW process.
        schedule! { simulation,
            "spiceObject" => &mut earth_ephemeris, period=STEP_NANOS, priority=100;
            "heldOrientation" => &mut earth_orientation, period=STEP_NANOS, priority=99;
            "scObject" => &mut chief, period=STEP_NANOS, priority=50;
            "scObject2" => &mut deputy, period=STEP_NANOS, priority=50;
            "simpleNavObject" => &mut chief_navigation, period=STEP_NANOS, priority=40;
            "simpleNavObject2" => &mut deputy_navigation, period=STEP_NANOS, priority=40;
            "dataLog" => &mut chief_log, period=sample_nanos, priority=30;
            "dataLog2" => &mut deputy_log, period=sample_nanos, priority=30;
            "meanOEFeedback" => &mut feedback, period=STEP_NANOS, priority=10;
        }
        if config.write_csv {
            simulation.add_module("csv", &mut csv, sample_nanos, 30);
        }
        simulation.run_for(duration_nanos);
    }
    if config.write_csv {
        csv.flush().expect("flush scenario CSV");
    }

    Results {
        chief: chief_log,
        deputy: deputy_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env(USAGE);
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream's `__main__` runs `run(True, True, 40)`.
        vec![Config::new(
            cli.flag("classic-elem", true),
            cli.value("orbits", 40.0),
        )]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        let relative_m = results.deputy.last().position_m - results.chief.last().position_m;
        println!(
            "wrote {} ({} samples, final deputy-chief separation {:.1} m)",
            results.csv_path.as_ref().expect("CSV written").display(),
            results.chief.values.len(),
            relative_m.norm(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioFormationMeanOEFeedback.py`: the chief and
    //! deputy states at every 200th of the recorded samples (one orbit) to a
    //! relative `1e-6` of upstream's truth.

    use super::*;
    use crate::formation::{compare_array_relative, every_nth};

    const TRUE_POS: [[f64; 3]; 4] = [
        [2.25733295e+06, 6.10774942e+06, 1.07696101e+06],
        [-1.16512871e+07, -5.29348500e+06, -9.38587959e+05],
        [-4.18795722e+06, -1.45287438e+07, -2.56449870e+06],
        [7.22485250e+06, -9.08490705e+06, -1.59808179e+06],
    ];
    const TRUE_VEL: [[f64; 3]; 4] = [
        [-8.64065671e+03, 3.09716311e+03, 5.46113421e+02],
        [2.89330869e+02, -4.99814200e+03, -8.81761232e+02],
        [3.85697370e+03, -8.90502532e+02, -1.55217728e+02],
        [2.70893560e+03, 4.86595115e+03, 8.60082783e+02],
    ];

    fn check(config: Config, true_pos2: [[f64; 3]; 4], true_vel2: [[f64; 3]; 4]) {
        let results = run(Config {
            write_csv: false,
            ..config
        });
        // Upstream `skipValue = int(numDataPoints / numTruthPoints)`.
        let skip = (NUM_DATA_POINTS / 5) as usize;
        let rows = |log: &MessageLog<SpacecraftStateMsg>, position: bool| {
            let values: Vec<_> = log
                .values
                .iter()
                .map(|state| {
                    if position {
                        state.position_m
                    } else {
                        state.velocity_mps
                    }
                })
                .collect();
            every_nth(&values, skip)
        };
        let accuracy = 1e-6;
        let failures: Vec<String> = [
            compare_array_relative(
                &TRUE_POS,
                &rows(&results.chief, true),
                accuracy,
                "chief r_BN_N Vector",
            ),
            compare_array_relative(
                &TRUE_VEL,
                &rows(&results.chief, false),
                accuracy,
                "chief v_BN_N Vector",
            ),
            compare_array_relative(
                &true_pos2,
                &rows(&results.deputy, true),
                accuracy,
                "deputy r_BN_N Vector",
            ),
            compare_array_relative(
                &true_vel2,
                &rows(&results.deputy, false),
                accuracy,
                "deputy v_BN_N Vector",
            ),
        ]
        .concat();
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn classic_element_feedback_matches_upstream_truth() {
        check(
            TEST_PARAMETRIZATIONS[0],
            [
                [2.24178592e+06, 6.11203995e+06, 1.07566911e+06],
                [-1.16485050e+07, -5.31123782e+06, -9.39975349e+05],
                [-4.17870694e+06, -1.45368263e+07, -2.56333852e+06],
                [7.22886213e+06, -9.08758649e+06, -1.59749668e+06],
            ],
            [
                [-8.64850674e+03, 3.08090460e+03, 5.42902521e+02],
                [2.94841385e+02, -4.99616760e+03, -8.80700996e+02],
                [3.85656016e+03, -8.87283980e+02, -1.54781609e+02],
                [2.70685918e+03, 4.86570257e+03, 8.59167262e+02],
            ],
        );
    }

    #[test]
    fn equinoctial_element_feedback_matches_upstream_truth() {
        check(
            TEST_PARAMETRIZATIONS[1],
            [
                [2.24178592e+06, 6.11203995e+06, 1.07566911e+06],
                [-1.16530991e+07, -5.30437873e+06, -9.38904459e+05],
                [-4.18987116e+06, -1.45340931e+07, -2.56591226e+06],
                [7.22079323e+06, -9.09440713e+06, -1.60488770e+06],
            ],
            [
                [-8.64850674e+03, 3.08090460e+03, 5.42902521e+02],
                [2.91749347e+02, -4.99568886e+03, -8.80758393e+02],
                [3.85566147e+03, -8.89923124e+02, -1.56722671e+02],
                [2.71046529e+03, 4.86245502e+03, 8.58723185e+02],
            ],
        );
    }
}
