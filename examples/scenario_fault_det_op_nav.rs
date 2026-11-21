//! Fault detection between two image-processing methods: port of upstream
//! `examples/OpNavScenarios/scenariosOpNav/scenario_faultDetOpNav.py`.
//!
//! The OpNav spacecraft ([`opnav::dynamics`]) on the 18 000 km, `e = 0.6`
//! orbit of `scenario_OpNavAttOD` (`i = 10 deg`, `Omega = 25 deg`, `omega =
//! 190 deg`, `f = 80 deg`), at rest at `sigma_BN = [0, -0.3, 0]`;
//! `relativeOD` (`noiseSF = 5`) starts on the true state. For 3 minutes
//! `prepOpNav` points the camera at Mars from the Hill frame; then
//! `FaultDet` processes every image twice: the limb (`limbFinding`,
//! `horizonOpNav`) is the primary measurement and Hough circles
//! (`houghCircles`, `pixelLineConverter`) the secondary one, and
//! `faultDetection` (`sigmaFault = 1`, `faultMode = 0`) merges them, or
//! flags a fault and keeps the secondary when they disagree. The result
//! points the camera (`opNavPoint`) and feeds `relativeOD`. The camera adds
//! cosmic rays (`cosmicRays = 2`: two attempts per image, each drawn with
//! probability `1 - 1/2² = 0.75`). 600 minutes at upstream's 0.5 s rates.
//!
//! **Images.** Upstream renders the camera images in the Vizard application.
//! Here a synthetic renderer stands in for Vizard
//! ([`opnav::planet_renderer`]): a ray-traced, Sun-lit, textureless Mars
//! ellipsoid through the camera model, so only outcomes (navigation error
//! bounds, which measurements are valid) compare with upstream.
//!
//! Writes `examples/output/scenarios/scenario_faultDetOpNav.csv`, upstream's
//! recorders: the spacecraft state (`scRec`), the filter (`filtRec`,
//! `filter_*`), the merged measurement (`opNavRec`, `opnav_*`) and the
//! primary and secondary ones (`opNavPrimRec`, `primary_*`; `opNavSecRec`,
//! `secondary_*`). `plot_scenario_fault_det_op_nav.py` draws upstream's
//! figures.
//!
//! `--sim-time m` sets upstream's `simTime` \[min\] (default 600); upstream's
//! `test_bskTestOpNav.py` runs 10 minutes and only checks completion. The
//! tests here also check the merged measurement and the filter.

#[path = "support/opnav/mod.rs"]
mod opnav;

use std::path::PathBuf;

use basilisk_rs::messages::{OpNavFilterMsg, OpNavMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use nalgebra::SVector;
use opnav::bsk_sim::{scenario_recorder, spacecraft_state_columns};
use opnav::dynamics;
use opnav::fsw::{FswMode, relative_od_q_noise};
use opnav::message_log::MessageLog;
use opnav::outputs::{od_filter_columns, opnav_columns, relative_nav_errors};
use opnav::{
    Cameras, OpNavModels, RATE_S, RECORDER_PRIORITY, execute_modes, hub_init, mars_orbit,
    sim_time_from_cli,
};

const STEM: &str = "scenario_faultDetOpNav";
/// Upstream's default `simTime` \[min\].
const SIM_TIME_MIN: f64 = 600.0;
/// Upstream's `prepOpNav` phase \[min\].
const PREP_TIME_MIN: f64 = 3.0;

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Upstream `simTime` \[min\].
    sim_time_min: f64,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    spacecraft: MessageLog<SpacecraftStateMsg>,
    filter: MessageLog<OpNavFilterMsg>,
    opnav: MessageLog<OpNavMsg>,
    primary: MessageLog<OpNavMsg>,
    secondary: MessageLog<OpNavMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    // `configure_initial_conditions`.
    let (r_n, v_n) = mars_orbit(18_000e3, 0.6, 10.0, 25.0, 190.0, 80.0);
    let mut cameras = Cameras::default();
    cameras.instrument.cosmic_rays = 2.0;
    let mut models = OpNavModels::new(
        RATE_S,
        RATE_S,
        &hub_init(r_n, v_n, [0.0, -0.3, 0.0]),
        cameras,
    );
    let fsw = &mut models.fsw;
    fsw.relative_od.config.state_init =
        SVector::from_iterator(r_n.iter().chain(v_n.iter()).copied());
    fsw.relative_od.config.q_noise = relative_od_q_noise();
    // `OpNavMsg_C_addAuthor`: the limb is the primary measurement, the
    // circles the secondary one.
    fsw.horizon_nav.op_nav_out_msg = fsw.gateway.opnav_primary.clone();
    fsw.pixel_line.op_nav_out_msg = fsw.gateway.opnav_secondary.clone();
    fsw.relative_od.config.noise_sf = 5.0;
    fsw.op_nav_fault.config.sigma_fault = 1.0;
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    // `log_outputs`.
    let sampling = models.fsw.task_period_nanos;
    let mut spacecraft = MessageLog::<SpacecraftStateMsg>::default();
    let mut filter = MessageLog::<OpNavFilterMsg>::default();
    let mut opnav = MessageLog::<OpNavMsg>::default();
    let mut primary = MessageLog::<OpNavMsg>::default();
    let mut secondary = MessageLog::<OpNavMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let state_out = &models.dynamics.spacecraft.state_out;
    let gateway = &models.fsw.gateway;
    let filter_out = &models.fsw.relative_od.filt_data_out_msg;
    state_out.connect_to(&mut spacecraft.input);
    filter_out.connect_to(&mut filter.input);
    gateway.opnav.connect_to(&mut opnav.input);
    gateway.opnav_primary.connect_to(&mut primary.input);
    gateway.opnav_secondary.connect_to(&mut secondary.input);
    state_out.connect_to(recorder.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("")));
    filter_out.connect_to(recorder.add_source::<OpNavFilterMsg>(od_filter_columns("filter")));
    for (output, prefix) in [
        (&gateway.opnav, "opnav"),
        (&gateway.opnav_primary, "primary"),
        (&gateway.opnav_secondary, "secondary"),
    ] {
        output.connect_to(recorder.add_source::<OpNavMsg>(opnav_columns(prefix)));
    }

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "scRec" => &mut spacecraft, period = sampling, priority = RECORDER_PRIORITY;
        "filtRec" => &mut filter, period = sampling, priority = RECORDER_PRIORITY;
        "opNavRec" => &mut opnav, period = sampling, priority = RECORDER_PRIORITY;
        "opNavPrimRec" => &mut primary, period = sampling, priority = RECORDER_PRIORITY;
        "opNavSecRec" => &mut secondary, period = sampling, priority = RECORDER_PRIORITY;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_PRIORITY);
    }
    execute_modes(
        &mut simulation,
        &mode,
        &runner,
        &[
            (FswMode::PrepOpNav, PREP_TIME_MIN),
            (FswMode::FaultDet, config.sim_time_min),
        ],
    );
    drop(simulation);

    Results {
        spacecraft,
        filter,
        opnav,
        primary,
        secondary,
        csv_path,
    }
}

fn main() {
    let sim_time_min = sim_time_from_cli("scenario_fault_det_op_nav", SIM_TIME_MIN);
    let results = run(Config {
        sim_time_min,
        write_csv: true,
    });
    if let Some(path) = &results.csv_path {
        println!("wrote {}", path.display());
    }
    let count = |log: &MessageLog<OpNavMsg>| log.values.iter().filter(|fix| fix.valid).count();
    let faults = results
        .opnav
        .values
        .iter()
        .filter(|fix| fix.fault_detected)
        .count();
    let (position, velocity) = relative_nav_errors(
        results.filter.last().state.as_slice(),
        results.spacecraft.last(),
    );
    println!(
        "{} limb and {} circle fixes, {} merged, {faults} faults; final relativeOD errors: position {:.3} %, velocity {:.3} %",
        count(&results.primary),
        count(&results.secondary),
        count(&results.opnav),
        position * 100.0,
        velocity * 100.0
    );
}

#[cfg(test)]
mod tests {
    //! Upstream `test_bskTestOpNav.py` runs `run(False, 10)` and checks
    //! only that it completes. The other test checks what the scenario
    //! demonstrates: both methods measure every image, the merged
    //! measurement is close to the truth, and the filter stays on the orbit.

    use super::*;
    use opnav::min2nano;

    #[test]
    fn upstream_test_run_completes() {
        let results = run(Config {
            sim_time_min: 10.0,
            write_csv: false,
        });
        assert_eq!(
            *results.spacecraft.times_nanos.last().unwrap(),
            min2nano(10.0)
        );
    }

    #[test]
    fn merged_measurements_keep_the_filter_on_the_orbit() {
        let results = run(Config {
            sim_time_min: 120.0,
            write_csv: false,
        });
        let count = |log: &MessageLog<OpNavMsg>| log.values.iter().filter(|fix| fix.valid).count();
        // One image a minute from the first one of `FaultDet`, measured by
        // both methods.
        assert!(count(&results.primary) >= 115);
        assert!(count(&results.secondary) >= 115);
        assert!(count(&results.opnav) >= 115);
        let (mut worst_fix, mut worst_position, mut worst_velocity) = (0.0_f64, 0.0_f64, 0.0_f64);
        for (((time, state), fix), filter) in results
            .spacecraft
            .times_nanos
            .iter()
            .zip(&results.spacecraft.values)
            .zip(&results.opnav.values)
            .zip(&results.filter.values)
        {
            // The filter's first output is recorded a step after the mode
            // change.
            if *time <= min2nano(PREP_TIME_MIN) + 500_000_000 {
                continue;
            }
            if fix.valid {
                let error =
                    (fix.position_bn_n_m - state.position_m).norm() / state.position_m.norm();
                worst_fix = worst_fix.max(error);
            }
            let (position, velocity) = relative_nav_errors(filter.state.as_slice(), state);
            worst_position = worst_position.max(position);
            worst_velocity = worst_velocity.max(velocity);
        }
        assert!(worst_fix < 0.04, "worst merged fix error {worst_fix}");
        assert!(
            worst_position < 0.01,
            "worst position error {worst_position}"
        );
        assert!(
            worst_velocity < 0.02,
            "worst velocity error {worst_velocity}"
        );
    }
}
