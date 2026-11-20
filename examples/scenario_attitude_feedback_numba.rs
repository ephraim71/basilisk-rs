//! Inertial attitude pointing with the navigation, guidance and control
//! modules written in the scenario itself: port of upstream
//! `examples/scenarioAttitudeFeedbackNumba.py`.
//!
//! The simulation is [`scenarioAttitudeFeedback`](scenario_attitude_feedback.rs)'s
//! (one 0.1 s task, 10 minutes, an `extForceTorque` effector applying the
//! control torque), but `simpleNav`, `inertial3D`, `attTrackingError` and
//! `mrpFeedback` are replaced by modules defined in the script. Upstream
//! writes them as Numba-compiled `NumbaModel` subclasses to show
//! user-written modules running at compiled speed. In Rust a user module is
//! simply a type implementing [`Module`], compiled with the rest of the
//! scenario; the four below transcribe the Numba ones:
//!
//! | upstream (Python / Numba)                   | here (Rust)                                  |
//! |---------------------------------------------|----------------------------------------------|
//! | `class X(NumbaModel)`                        | `struct X` + `impl Module for X`             |
//! | reader / message attributes in `__init__`    | [`Input`] / [`Output`] fields                |
//! | `self.memory.<name>`                         | plain fields                                 |
//! | `UpdateStateImpl(<payloads>, memory, ...)`   | [`Module::update`]: `read()`, then `write()` |
//! | `<msg>IsLinked` arguments                    | [`Input::is_connected`]                      |
//! | `Reset(CurrentSimNanos)`                     | [`Module::init`] / [`Module::reset`]         |
//! | the per-module `rng`                         | a seeded [`StdRng`] field                    |
//! | `RigidBodyKinematicsNumba`                   | `basilisk_rs::utilities::rigid_body_kinematics` |
//!
//! Upstream `run(show_plots, useUnmodeledTorque, useIntGain, useKnownTorque)`
//! maps to `--unmodeled-torque`, `--int-gain` and `--known-torque`; `--all`
//! runs the four parametrizations of upstream's
//! `test_scenarioAttitudeFeedbackNumba.py`.
//!
//! Writes `examples/output/scenarios/scenarioAttitudeFeedbackNumba<ut><ig><kt>.csv`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;

use std::path::PathBuf;

use attitude::{
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, HubState, UNMODELED_TORQUE_NM, add_point_mass_earth,
    bit, external_disturbance, hub_inertia_kg_m2, sampling_period_nanos, scenario_recorder,
    spacecraft, vehicle_config,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, AttitudeReferenceMsg, BodyTorqueCommandMsg, Input, MAX_EFF_COUNT,
    NavigationAttitudeMsg, NavigationTranslationMsg, Output, PlanetStateMsg, RwArrayConfigMsg,
    RwAvailabilityMsg, RwSpeedMsg, SpacecraftStateMsg, VehicleConfigMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::rigid_body_kinematics::{add_mrp, mrp_to_dcm, sub_mrp};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use cli::Cli;
use common::vector_columns;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{SMatrix, SVector, Vector3};
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, StandardNormal};

/// Upstream `NumbaSimpleNav`: spacecraft truth plus an 18-state random-walk
/// error (position, velocity, attitude, rate, Sun direction, ΔV), driven by
/// `PMatrix` and clamped to `walkBounds`. The scenario leaves both zero, so
/// the outputs are the truth.
struct ScenarioSimpleNav {
    sc_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `sunStateInMsg` (unconnected in this scenario).
    sun_state_in_msg: Input<PlanetStateMsg>,
    att_out_msg: Output<NavigationAttitudeMsg>,
    trans_out_msg: Output<NavigationTranslationMsg>,
    errors: SVector<f64, 18>,
    p_matrix: SMatrix<f64, 18, 18>,
    walk_bounds: SVector<f64, 18>,
    cross_trans: bool,
    cross_att: bool,
    prev_time_nanos: u64,
    rng: StdRng,
}

impl ScenarioSimpleNav {
    fn new() -> Self {
        Self {
            sc_state_in_msg: Input::default(),
            sun_state_in_msg: Input::default(),
            att_out_msg: Output::default(),
            trans_out_msg: Output::default(),
            errors: SVector::zeros(),
            p_matrix: SMatrix::zeros(),
            walk_bounds: SVector::zeros(),
            cross_trans: false,
            cross_att: false,
            prev_time_nanos: 0,
            rng: StdRng::seed_from_u64(0),
        }
    }
}

impl Module for ScenarioSimpleNav {
    fn init(&mut self) {
        self.att_out_msg.write(NavigationAttitudeMsg::default());
        self.trans_out_msg
            .write(NavigationTranslationMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let now = context.current_sim_nanos;
        let state = self.sc_state_in_msg.read();
        let dt_s = if self.prev_time_nanos == 0 {
            0.0
        } else {
            (now - self.prev_time_nanos) as f64 * 1.0e-9
        };

        let old = self.errors;
        let mut new = old;
        if self.cross_trans {
            let mut position_error = new.fixed_rows_mut::<3>(0);
            position_error += dt_s * old.fixed_rows::<3>(3);
        }
        if self.cross_att {
            let mut attitude_error = new.fixed_rows_mut::<3>(6);
            attitude_error += dt_s * old.fixed_rows::<3>(9);
        }
        let white = SVector::<f64, 18>::from_fn(|_, _| StandardNormal.sample(&mut self.rng));
        new += self.p_matrix * white;
        for (error, bound) in new.iter_mut().zip(self.walk_bounds.iter()) {
            if *bound > 0.0 {
                *error = error.clamp(-bound, *bound);
            }
        }
        self.errors = new;

        let time_tag_s = now as f64 * 1.0e-9;
        let error = |start: usize| Vector3::new(new[start], new[start + 1], new[start + 2]);
        self.trans_out_msg.write(NavigationTranslationMsg {
            time_tag_s,
            position_bn_n_m: state.position_m + error(0),
            velocity_bn_n_mps: state.velocity_mps + error(3),
            accumulated_dv_b_mps: state.accumulated_dv_cn_b_mps + error(15),
        });
        let vehicle_sun_point_body = if self.sun_state_in_msg.is_connected() {
            let sun = self.sun_state_in_msg.read();
            let sc_to_sun = sun.position_inertial_m - state.position_m;
            let norm = sc_to_sun.norm();
            let sc_to_sun = if norm > 0.0 {
                sc_to_sun / norm
            } else {
                sc_to_sun
            };
            mrp_to_dcm(error(12)) * (mrp_to_dcm(state.sigma_bn) * sc_to_sun)
        } else {
            Vector3::zeros()
        };
        self.att_out_msg.write(NavigationAttitudeMsg {
            time_tag_s,
            sigma_bn: add_mrp(state.sigma_bn, error(6)),
            omega_bn_b_radps: state.omega_radps + error(9),
            vehicle_sun_point_body,
        });
        self.prev_time_nanos = now;
    }
}

/// Upstream `NumbaInertial3D`: a fixed inertial reference `sigma_R0N`.
struct ScenarioInertial3D {
    att_ref_out_msg: Output<AttitudeReferenceMsg>,
    sigma_r0n: Vector3<f64>,
}

impl Module for ScenarioInertial3D {
    fn init(&mut self) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg {
            sigma_rn: self.sigma_r0n,
            omega_rn_n_radps: Vector3::zeros(),
            domega_rn_n_radps2: Vector3::zeros(),
        });
    }
}

/// Upstream `NumbaAttTrackingError`: attitude and rate errors of the body
/// relative to the reference corrected by `sigma_R0R`.
struct ScenarioAttTrackingError {
    att_nav_in_msg: Input<NavigationAttitudeMsg>,
    att_ref_in_msg: Input<AttitudeReferenceMsg>,
    att_guid_out_msg: Output<AttitudeGuidanceMsg>,
    sigma_r0r: Vector3<f64>,
}

impl Module for ScenarioAttTrackingError {
    fn init(&mut self) {
        self.att_guid_out_msg.write(AttitudeGuidanceMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let navigation = self.att_nav_in_msg.read();
        let reference = self.att_ref_in_msg.read();
        let sigma_rn = add_mrp(reference.sigma_rn, -self.sigma_r0r);
        let dcm_bn = mrp_to_dcm(navigation.sigma_bn);
        let omega_rn_b = dcm_bn * reference.omega_rn_n_radps;
        self.att_guid_out_msg.write(AttitudeGuidanceMsg {
            sigma_br: sub_mrp(navigation.sigma_bn, sigma_rn),
            omega_br_b_radps: navigation.omega_bn_b_radps - omega_rn_b,
            omega_rn_b_radps: omega_rn_b,
            domega_rn_b_radps2: dcm_bn * reference.domega_rn_n_radps2,
        });
    }
}

/// Upstream `NumbaMrpFeedback`: the MRP feedback law of `mrpFeedback`, with
/// optional integral feedback and reaction-wheel momentum.
struct ScenarioMrpFeedback {
    guid_in_msg: Input<AttitudeGuidanceMsg>,
    veh_config_in_msg: Input<VehicleConfigMsg>,
    rw_speeds_in_msg: Input<RwSpeedMsg>,
    rw_avail_in_msg: Input<RwAvailabilityMsg>,
    rw_params_in_msg: Input<RwArrayConfigMsg>,
    cmd_torque_out_msg: Output<BodyTorqueCommandMsg>,
    k: f64,
    p: f64,
    ki: f64,
    integral_limit: f64,
    known_torque_body_nm: Vector3<f64>,
    control_law_type: i32,
    int_sigma: Vector3<f64>,
    prior_time_nanos: u64,
    rw_config: RwArrayConfigMsg,
}

impl ScenarioMrpFeedback {
    /// Upstream's `memory` defaults: `K = 3.5`, `P = 30`, integral off.
    fn new() -> Self {
        Self {
            guid_in_msg: Input::default(),
            veh_config_in_msg: Input::default(),
            rw_speeds_in_msg: Input::default(),
            rw_avail_in_msg: Input::default(),
            rw_params_in_msg: Input::default(),
            cmd_torque_out_msg: Output::default(),
            k: 3.5,
            p: 30.0,
            ki: -1.0,
            integral_limit: 0.0,
            known_torque_body_nm: Vector3::zeros(),
            control_law_type: 0,
            int_sigma: Vector3::zeros(),
            prior_time_nanos: 0,
            rw_config: RwArrayConfigMsg::default(),
        }
    }
}

impl Module for ScenarioMrpFeedback {
    /// Upstream `Reset`.
    fn init(&mut self) {
        self.cmd_torque_out_msg
            .write(BodyTorqueCommandMsg::default());
        self.int_sigma = Vector3::zeros();
        self.prior_time_nanos = 0;
        self.rw_config = if self.rw_params_in_msg.is_connected() {
            self.rw_params_in_msg.read()
        } else {
            RwArrayConfigMsg::default()
        };
    }

    fn update(&mut self, context: &SimulationContext) {
        let now = context.current_sim_nanos;
        let dt_s = if self.prior_time_nanos == 0 {
            0.0
        } else {
            (now - self.prior_time_nanos) as f64 * 1.0e-9
        };
        self.prior_time_nanos = now;

        let guidance = self.guid_in_msg.read();
        let inertia = self.veh_config_in_msg.read().inertia_about_point_b_kg_m2;
        let sigma_br = guidance.sigma_br;
        let omega_br_b = guidance.omega_br_b_radps;
        let omega_rn_b = guidance.omega_rn_b_radps;
        let omega_bn_b = omega_br_b + omega_rn_b;

        let mut z = Vector3::zeros();
        if self.ki > 0.0 {
            self.int_sigma += self.k * dt_s * sigma_br;
            let limit = self.integral_limit;
            self.int_sigma = self.int_sigma.map(|value| value.clamp(-limit, limit));
            z = self.int_sigma + inertia * omega_br_b;
        }
        let v3_4 = self.ki * z;
        let mut torque = self.k * sigma_br + self.p * (omega_br_b + v3_4);

        let mut momentum = inertia * omega_bn_b;
        if self.rw_speeds_in_msg.is_connected() {
            let speeds = self.rw_speeds_in_msg.read();
            let availability = self.rw_avail_in_msg.read();
            for index in 0..self.rw_config.num_rw.min(MAX_EFF_COUNT) {
                if !self.rw_avail_in_msg.is_connected() || availability.is_available(index) {
                    let gs = self.rw_config.spin_axes_body[index];
                    momentum += self.rw_config.spin_axis_inertias_kg_m2[index]
                        * (omega_bn_b.dot(&gs) + speeds.wheel_speeds_radps[index])
                        * gs;
                }
            }
        }
        let v3_8 = if self.control_law_type == 0 {
            omega_rn_b + v3_4
        } else {
            omega_bn_b
        };
        torque -= v3_8.cross(&momentum);
        torque += inertia * (omega_bn_b.cross(&omega_rn_b) - guidance.domega_rn_b_radps2);
        torque += self.known_torque_body_nm;
        self.cmd_torque_out_msg.write(BodyTorqueCommandMsg {
            torque_request_body_nm: -torque,
        });
    }
}

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    use_unmodeled_torque: bool,
    use_int_gain: bool,
    use_known_torque: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(use_unmodeled_torque: bool, use_int_gain: bool, use_known_torque: bool) -> Self {
        Self {
            use_unmodeled_torque,
            use_int_gain,
            use_known_torque,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!(
            "scenarioAttitudeFeedbackNumba{}{}{}",
            bit(self.use_unmodeled_torque),
            bit(self.use_int_gain),
            bit(self.use_known_torque),
        )
    }
}

/// The parametrizations of upstream `test_scenarioAttitudeFeedbackNumba.py`.
const TEST_PARAMETRIZATIONS: [Config; 4] = [
    Config::new(false, false, false),
    Config::new(true, false, false),
    Config::new(true, true, false),
    Config::new(true, false, true),
];

/// The recorded histories, every `samplingTime(T, 0.1 s, 50)` = 12.2 s.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    torque: MessageLog<BodyTorqueCommandMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let inertia = hub_inertia_kg_m2();
    let mut spacecraft = spacecraft(inertia, &HubState::on_orbit(0.01));
    add_point_mass_earth(&mut spacecraft);
    let disturbance_nm = if config.use_unmodeled_torque {
        Vector3::from(UNMODELED_TORQUE_NM)
    } else {
        Vector3::zeros()
    };
    let mut external_torque = external_disturbance(disturbance_nm);

    let mut navigation = ScenarioSimpleNav::new();
    let mut inertial_reference = ScenarioInertial3D {
        att_ref_out_msg: Output::default(),
        sigma_r0n: Vector3::zeros(),
    };
    let mut tracking_error = ScenarioAttTrackingError {
        att_nav_in_msg: Input::default(),
        att_ref_in_msg: Input::default(),
        att_guid_out_msg: Output::default(),
        sigma_r0r: Vector3::zeros(),
    };
    let mut controller = ScenarioMrpFeedback::new();
    if config.use_int_gain {
        controller.ki = 0.0002;
        controller.integral_limit = 2.0 / controller.ki * 0.1;
    }
    if config.use_known_torque {
        controller.known_torque_body_nm = Vector3::from(UNMODELED_TORQUE_NM);
    }
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 50);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut torque_log = MessageLog::<BodyTorqueCommandMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg,
    );
    spacecraft.add_dynamic_effector(external_torque);
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.sc_state_in_msg,
        &navigation.att_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &controller.cmd_torque_out_msg => &mut torque_log.input,
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
        &controller.cmd_torque_out_msg => recorder.add_source::<BodyTorqueCommandMsg>(
            CsvSourceConfig::columns(vector_columns(
                "torque_request_body_nm",
                "torqueRequestBody_Nm",
            )),
        ),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "simpleNav" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        "inertial3D" => &mut inertial_reference, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "attTrackingError" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "mrpFeedback" => &mut controller, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "torque_log" => &mut torque_log, period=sample_period, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    let started = std::time::Instant::now();
    simulation.run_for(DURATION_NANOS);
    if config.write_csv {
        // Upstream prints the wall-clock time of `ExecuteSimulation`.
        println!("Run scenario in {:.3} s", started.elapsed().as_secs_f64());
    }
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        torque: torque_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_attitude_feedback_numba [--[no-]unmodeled-torque] [--[no-]int-gain] \
         [--[no-]known-torque] [--all]",
    );
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(
            cli.flag("unmodeled-torque", false),
            cli.flag("int-gain", false),
            cli.flag("known-torque", false),
        )]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!(
            "wrote {} (final |sigma_BR| = {:.3e})",
            results.csv_path.expect("CSV written").display(),
            results.guidance.last().sigma_br.norm(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttitudeFeedbackNumba.py` only checks that each
    //! parametrization runs. These tests check that the scenario's own
    //! modules agree with the library modules they transcribe, and that the
    //! closed loop behaves as `scenarioAttitudeFeedback`'s does.

    use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
    use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
        AttTrackingError, AttTrackingErrorConfig,
    };
    use basilisk_rs::sensors::navigation::SimpleNavigation;

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    fn context(nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2019, 1, 1),
        }
    }

    /// A slowly varying input sequence, one value per 0.1 s update.
    fn wobble(step: u64, phase: f64, scale: f64) -> Vector3<f64> {
        let t = step as f64 * 0.1;
        scale
            * Vector3::new(
                (0.3 * t + phase).sin(),
                (0.2 * t).cos(),
                0.5 - (0.1 * t).sin(),
            )
    }

    #[test]
    fn the_mrp_feedback_module_matches_the_library_mrp_feedback() {
        for (ki, known_torque) in [
            (-1.0, Vector3::zeros()),
            (0.0002, Vector3::zeros()),
            (0.0002, Vector3::new(0.25, -0.25, 0.1)),
        ] {
            let guidance = Output::<AttitudeGuidanceMsg>::default();
            let vehicle = Output::new(vehicle_config(hub_inertia_kg_m2()));
            let mut scenario = ScenarioMrpFeedback::new();
            scenario.ki = ki;
            scenario.integral_limit = 2.0 / ki * 0.1;
            scenario.known_torque_body_nm = known_torque;
            let mut library = MrpFeedback::new(MrpFeedbackConfig {
                name: "mrpFeedback".to_string(),
                k: 3.5,
                ki,
                p: 30.0,
                integral_limit: 2.0 / ki * 0.1,
                known_torque_body_nm: known_torque,
                control_law_type: 0,
            });
            guidance.connect_to(&mut scenario.guid_in_msg);
            guidance.connect_to(&mut library.guid_in_msg);
            vehicle.connect_to(&mut scenario.veh_config_in_msg);
            vehicle.connect_to(&mut library.veh_config_in_msg);
            scenario.init();
            library.init();
            for step in 0..200 {
                guidance.write(AttitudeGuidanceMsg {
                    sigma_br: wobble(step, 0.0, 0.3),
                    omega_br_b_radps: wobble(step, 1.0, 0.02),
                    omega_rn_b_radps: wobble(step, 2.0, 0.001),
                    domega_rn_b_radps2: wobble(step, 3.0, 1.0e-5),
                });
                scenario.update(&context(step * DYNAMICS_PERIOD_NANOS));
                library.update(&context(step * DYNAMICS_PERIOD_NANOS));
                let ours = scenario.cmd_torque_out_msg.read().torque_request_body_nm;
                let theirs = library.cmd_torque_out_msg.read().torque_request_body_nm;
                assert!((ours - theirs).norm() <= 1.0e-12 * theirs.norm().max(1.0));
            }
        }
    }

    #[test]
    fn the_guidance_and_navigation_modules_match_the_library_ones() {
        let state = Output::<SpacecraftStateMsg>::default();
        let mut scenario_nav = ScenarioSimpleNav::new();
        let mut library_nav = SimpleNavigation::new("SimpleNavigation");
        state.connect_to(&mut scenario_nav.sc_state_in_msg);
        state.connect_to(&mut library_nav.spacecraft_state_in_msg);

        let reference = Output::<AttitudeReferenceMsg>::default();
        let sigma_r0r = Vector3::new(0.0, 0.2, 0.0);
        let mut scenario_error = ScenarioAttTrackingError {
            att_nav_in_msg: Input::default(),
            att_ref_in_msg: Input::default(),
            att_guid_out_msg: Output::default(),
            sigma_r0r,
        };
        let mut library_error = AttTrackingError::new(AttTrackingErrorConfig {
            name: "attTrackingError".to_string(),
            sigma_r0r,
        });
        library_nav
            .attitude_out_msg
            .connect_to(&mut scenario_error.att_nav_in_msg);
        library_nav
            .attitude_out_msg
            .connect_to(&mut library_error.att_nav_in_msg);
        reference.connect_to(&mut scenario_error.att_ref_in_msg);
        reference.connect_to(&mut library_error.att_ref_in_msg);

        for module in [
            &mut scenario_nav as &mut dyn Module,
            &mut library_nav,
            &mut scenario_error,
            &mut library_error,
        ] {
            module.init();
        }
        for step in 0..100 {
            state.write(SpacecraftStateMsg {
                position_m: 7.0e6 * wobble(step, 0.0, 1.0),
                velocity_mps: 7.0e3 * wobble(step, 0.5, 1.0),
                sigma_bn: wobble(step, 1.0, 0.4),
                omega_radps: wobble(step, 1.5, 0.05),
                ..Default::default()
            });
            reference.write(AttitudeReferenceMsg {
                sigma_rn: wobble(step, 2.0, 0.3),
                omega_rn_n_radps: wobble(step, 2.5, 0.001),
                domega_rn_n_radps2: wobble(step, 3.0, 1.0e-6),
            });
            let now = context(step * DYNAMICS_PERIOD_NANOS);
            scenario_nav.update(&now);
            library_nav.update(&now);
            scenario_error.update(&now);
            library_error.update(&now);

            assert_eq!(
                scenario_nav.trans_out_msg.read(),
                library_nav.translation_out_msg.read()
            );
            let ours = scenario_nav.att_out_msg.read();
            let theirs = library_nav.attitude_out_msg.read();
            assert!((ours.sigma_bn - theirs.sigma_bn).norm() < 1.0e-15);
            assert_eq!(ours.omega_bn_b_radps, theirs.omega_bn_b_radps);

            let ours = scenario_error.att_guid_out_msg.read();
            let theirs = library_error.att_guid_out_msg.read();
            assert!((ours.sigma_br - theirs.sigma_br).norm() < 1.0e-14);
            assert!((ours.omega_br_b_radps - theirs.omega_br_b_radps).norm() < 1.0e-15);
            assert!((ours.omega_rn_b_radps - theirs.omega_rn_b_radps).norm() < 1.0e-15);
        }
    }

    #[test]
    fn the_closed_loop_behaves_as_scenario_attitude_feedback() {
        let disturbance = Vector3::from(UNMODELED_TORQUE_NM);
        let runs: Vec<Results> = TEST_PARAMETRIZATIONS.into_iter().map(quiet).collect();
        for results in &runs {
            // samplingTime(10 min, 0.1 s, 50) = 12.2 s: 50 samples.
            assert_eq!(results.guidance.values.len(), 50);
            assert_eq!(
                results.guidance.values[0].sigma_br,
                Vector3::new(0.1, 0.2, -0.3)
            );
        }
        assert!(runs[0].guidance.last().sigma_br.norm() < 1.0e-4);
        let offset = runs[1].guidance.last().sigma_br;
        assert!((offset - disturbance / 3.5).norm() < 1.0e-4, "{offset}");
        assert!(runs[2].guidance.last().sigma_br.norm() < 0.1 * offset.norm());
        assert!(runs[3].guidance.last().sigma_br.norm() < 1.0e-4);
        let torque = runs[3].torque.last().torque_request_body_nm;
        assert!((torque + disturbance).norm() < 1.0e-4, "{torque}");
    }
}
