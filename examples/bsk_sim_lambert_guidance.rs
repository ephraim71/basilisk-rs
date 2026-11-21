//! BskSim Lambert guidance to a geostationary slot: port of upstream
//! `examples/BskSim/scenarios/scenario_LambertGuidance.py`.
//!
//! The BskSim spacecraft ([`bsk_sim::dynamics`]) on a 40 000 km, `e = 0.001`
//! orbit (`i = Omega = omega = 1 deg`, `f = -30 deg`), with upstream's noisy
//! navigation (1 m and 0.1 m/s standard deviations, walk bounds 10 m and
//! 1 m/s), on a 10 s dynamics task and 60 s FSW tasks
//! (`BSKSim(fswRate=60, dynRate=10)`):
//!
//! 1. `lambertFirstDV` until `tm1 = 2460 s`: Hill pointing with the MRP
//!    steering law, while `LambertPlanner`, `LambertSolver` and
//!    `LambertValidator` plan the transfer to `r_TN_N = [42 164 km, 0, 0]`
//!    at `tm2 = 4980 s`. The validated burn (`dvBurnCmdOutMsg`, as last
//!    recorded) is then added to the hub velocity (upstream
//!    `velRef.setState`).
//! 2. `lambertSecondDV` until `tm2`: `LambertSurfaceRelativeVelocity` and
//!    `LambertSecondDV` command the burn to zero velocity relative to the
//!    rotating Earth surface (`vRelativeDesired_S = 0`), added at `tm2`.
//! 3. `standby` until 6000 s. Upstream's standby action does nothing, so the
//!    second-burn mode's tasks keep running (see [`bsk_sim::fsw`]).
//!
//! Writes `examples/output/scenarios/scenario_LambertGuidance.csv` every
//! dynamics step: truth and navigation position and velocity, the two burn
//! commands and the Earth ephemeris attitude. `plot_bsk_sim_lambert_guidance.py`
//! draws upstream's `orbit`, `position`, `velocity` and
//! `surfaceRelativeVelocity` figures.
//!
//! Upstream's `test_bskTestScript.py` only runs the scenario; the tests check
//! that the spacecraft arrives at the target and stays over the same surface
//! point.

#[path = "support/bsk_sim/mod.rs"]
mod bsk_sim;

use std::path::PathBuf;

use basilisk_rs::messages::{
    DvBurnCommandMsg, EphemerisMsg, NavigationTranslationMsg, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use bsk_sim::cli::Cli;
use bsk_sim::common::vector_columns;
use bsk_sim::dynamics;
use bsk_sim::fsw::FswMode;
use bsk_sim::grav_factory::Body;
use bsk_sim::latch::Latch;
use bsk_sim::message_log::MessageLog;
use bsk_sim::simple_nav::{NavSigmas, grouped};
use bsk_sim::{
    BskModels, HubInit, OMEGA_BN_B_INIT, RECORDER_PRIORITY, SIGMA_BN_INIT, elements,
    scenario_recorder, sec2nano, translation_columns,
};
use nalgebra::{Matrix6, Vector3, Vector6};

const STEM: &str = "scenario_LambertGuidance";
/// Upstream `tm1`, `tm2` \[s\].
const TM1_S: f64 = 2460.0;
const TM2_S: f64 = 4980.0;
/// Upstream `r_TN_N` \[m\].
const R_TN_N_M: [f64; 3] = [42164.0 * 1000.0, 0.0, 0.0];
const POS_SIGMA_M: f64 = 1.0;
const VEL_SIGMA_MPS: f64 = 0.1;

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    truth: MessageLog<SpacecraftStateMsg>,
    nav: MessageLog<NavigationTranslationMsg>,
    earth: MessageLog<EphemerisMsg>,
    dv1_mps: Vector3<f64>,
    dv2_mps: Vector3<f64>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let initial = HubInit::from_elements(
        Body::Earth.mu_m3ps2(),
        &elements(40_000.0 * 1000.0, 0.001, 1.0, 1.0, 1.0, -30.0),
        SIGMA_BN_INIT,
        OMEGA_BN_B_INIT,
    );
    let mut models = BskModels::new(60.0, 10.0, &initial);
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    // The scenario's `__init__`: navigation errors and Lambert parameters.
    let nav = &mut models.dynamics.simple_nav;
    nav.p_matrix_sqrt = NavSigmas {
        position: POS_SIGMA_M,
        velocity: VEL_SIGMA_MPS,
        ..NavSigmas::default()
    }
    .p_matrix();
    nav.walk_bounds = grouped([10.0, 1.0, 0.0, 0.0, 0.0, 0.0]);
    let mu = models.dynamics.mu_earth_m3ps2;
    let fsw = &mut models.fsw;
    let planner = &mut fsw.lambert_planner.module.config;
    planner.r_tn_n_m = Vector3::from(R_TN_N_M);
    planner.final_time_s = TM2_S;
    planner.maneuver_time_s = TM1_S;
    planner.mu_m3ps2 = mu;
    planner.num_revolutions = 0;
    let validator = &mut fsw.lambert_validator.module.config;
    validator.final_time_s = TM2_S;
    validator.maneuver_time_s = TM1_S;
    validator.max_distance_target_m = 500.0;
    validator.min_orbit_radius_m = 6378.0 * 1000.0;
    validator.uncertainty_states = Matrix6::from_diagonal(&Vector6::new(
        POS_SIGMA_M,
        POS_SIGMA_M,
        POS_SIGMA_M,
        VEL_SIGMA_MPS,
        VEL_SIGMA_MPS,
        VEL_SIGMA_MPS,
    ));
    validator.uncertainty_dv_mps = 0.1;
    validator.dv_convergence_tolerance_mps = 1.0;
    let surface = &mut fsw.lambert_surface_relative_velocity.module.config;
    surface.v_relative_desired_s_mps = Vector3::zeros();
    surface.time_s = TM2_S;

    // `log_outputs`: `recorder()` with no sampling time, every dynamics step.
    let sampling = models.dynamics.task_period_nanos;
    let mut truth = MessageLog::<SpacecraftStateMsg>::default();
    let mut nav_log = MessageLog::<NavigationTranslationMsg>::default();
    // The recorders' last samples, which the scenario reads between runs.
    let mut dv1 = Latch::<DvBurnCommandMsg>::default();
    let mut dv2 = Latch::<DvBurnCommandMsg>::default();
    let (dv1_last, dv2_last) = (dv1.output.clone(), dv2.output.clone());
    let mut earth = MessageLog::<EphemerisMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let state = &models.dynamics.spacecraft.state_out;
    let nav = &models.dynamics.simple_nav.translation_out_msg;
    let dv1_msg = &models.fsw.lambert_validator.module.dv_burn_cmd_out_msg;
    let dv2_msg = &models.fsw.lambert_second_dv.module.dv_burn_cmd_out_msg;
    let earth_msg = &models.dynamics.earth_ephemeris.ephem_out_msgs[0];
    state.connect_to(&mut truth.input);
    nav.connect_to(&mut nav_log.input);
    dv1_msg.connect_to(&mut dv1.input);
    dv2_msg.connect_to(&mut dv2.input);
    earth_msg.connect_to(&mut earth.input);
    state.connect_to(
        recorder.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
            vector_columns("position_m", "r_BN_N_m")
                .into_iter()
                .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s")),
        )),
    );
    nav.connect_to(recorder.add_source::<NavigationTranslationMsg>(translation_columns("meas")));
    dv1_msg.connect_to(
        recorder.add_source::<DvBurnCommandMsg>(CsvSourceConfig::columns(vector_columns(
            "dv_inertial_cmd_mps",
            "dv1InrtlCmd_m_per_s",
        ))),
    );
    dv2_msg.connect_to(
        recorder.add_source::<DvBurnCommandMsg>(CsvSourceConfig::columns(vector_columns(
            "dv_inertial_cmd_mps",
            "dv2InrtlCmd_m_per_s",
        ))),
    );
    earth_msg.connect_to(
        recorder.add_source::<EphemerisMsg>(CsvSourceConfig::columns(
            vector_columns("sigma_bn", "earth_sigma_BN")
                .into_iter()
                .chain(vector_columns(
                    "omega_bn_b_radps",
                    "earth_omega_BN_B_rad_per_s",
                )),
        )),
    );

    let hub = models.dynamics.hub.clone();
    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "sc_truth_recorder" => &mut truth, period = sampling, priority = RECORDER_PRIORITY;
        "sc_meas_recorder" => &mut nav_log, period = sampling, priority = RECORDER_PRIORITY;
        "dv1Cmd_recorder" => &mut dv1, period = sampling, priority = RECORDER_PRIORITY;
        "dv2Cmd_recorder" => &mut dv2, period = sampling, priority = RECORDER_PRIORITY;
        "ephem_recorder" => &mut earth, period = sampling, priority = RECORDER_PRIORITY;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_PRIORITY);
    }

    // `runScenario`.
    mode.set(FswMode::LambertFirstDv);
    runner.execute(&mut simulation, sec2nano(TM1_S));
    let dv1_mps = dv1_last.read().dv_inertial_cmd_mps;
    hub.set_velocity_mps(hub.velocity_mps() + dv1_mps);
    mode.set(FswMode::LambertSecondDv);
    runner.execute(&mut simulation, sec2nano(TM2_S));
    let dv2_mps = dv2_last.read().dv_inertial_cmd_mps;
    hub.set_velocity_mps(hub.velocity_mps() + dv2_mps);
    mode.set(FswMode::Standby);
    runner.execute(&mut simulation, sec2nano(6000.0));
    drop(simulation);
    drop(recorder);

    Results {
        truth,
        nav: nav_log,
        earth,
        dv1_mps,
        dv2_mps,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env("bsk_sim_lambert_guidance");
    cli.finish();
    let results = run(Config { write_csv: true });
    let arrival = results.truth.at(sec2nano(TM2_S));
    println!(
        "wrote {} (burns {:.2} and {:.2} m/s, {:.1} m from the target at tm2)",
        results.csv_path.expect("CSV written").display(),
        results.dv1_mps.norm(),
        results.dv2_mps.norm(),
        (arrival.position_m - Vector3::from(R_TN_N_M)).norm(),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;

    /// The first burn sends the spacecraft to `r_TN_N` at `tm2`; the
    /// navigation errors at the burn (bounded by the 10 m / 1 m/s walk)
    /// leave it a few kilometres off.
    #[test]
    fn the_transfer_reaches_the_target() {
        let results = run(Config { write_csv: false });
        assert!(results.dv1_mps.norm() > 100.0, "{}", results.dv1_mps.norm());
        let arrival = results.truth.at(sec2nano(TM2_S));
        let miss = (arrival.position_m - Vector3::from(R_TN_N_M)).norm();
        assert!(miss < 10_000.0, "{miss} m");
        // The navigation solution is the truth plus bounded errors.
        for (truth, nav) in results.truth.values.iter().zip(&results.nav.values).skip(1) {
            assert!((truth.position_m - nav.position_bn_n_m).amax() <= 10.0 + 1e-9);
            assert!((truth.velocity_mps - nav.velocity_bn_n_mps).amax() <= 1.0 + 1e-9);
        }
    }

    /// After the second burn the spacecraft moves with the Earth's surface
    /// below it (upstream's `surfaceRelativeVelocity` figure): its velocity
    /// relative to the rotating Earth is small.
    #[test]
    fn the_second_burn_matches_the_surface_velocity() {
        let results = run(Config { write_csv: false });
        assert!(results.dv2_mps.norm() > 10.0);
        let samples = results.truth.times_nanos.iter().zip(&results.truth.values);
        let after: Vec<_> = samples
            .zip(&results.earth.values)
            .filter(|((time, _), _)| **time > sec2nano(TM2_S) + 10_000_000_000)
            .collect();
        assert!(!after.is_empty());
        for ((_, state), earth) in after {
            let dcm_pn = mrp_to_dcm(earth.sigma_bn);
            let omega_pn_n = dcm_pn.transpose() * earth.omega_bn_b_radps;
            let relative = state.velocity_mps - omega_pn_n.cross(&state.position_m);
            assert!(relative.norm() < 5.0, "{}", relative.norm());
        }
    }
}
