//! BskSim fault list: port of upstream
//! `examples/BskSim/scenarios/scenario_FaultList.py`, and of
//! `src/tests/test_bskFaults.py`, which exercises every `BSK_Faults` class on
//! the same spacecraft.
//!
//! The BskSim spacecraft ([`bsk_sim::dynamics`]) on a 6835.5 km,
//! `e = 0.00063`, `i = 98 deg` polar orbit (`Omega = 85.47 deg`, `omega = 0`,
//! `f = 30 deg`), Hill pointing with its wheels for 50 minutes on 1 s tasks
//! (`fswRate = dynRate = 1`), with upstream's `faultList`
//! ([`bsk_sim::faults`]):
//!
//! * `RwPowerFault(5 min, 0.01 W, wheel 1)`: `RW2`'s power limit drops to
//!   0.01 W;
//! * `MagPolarNoise(i min, "NOISE")` for `i = 0 ... 50`: once a minute the
//!   magnetometer's noise and random-walk bounds are blended between the
//!   nominal 4 nT / 15 nT and the polar 120 nT / 600 nT by the geodetic
//!   latitude (a sigmoid centred on 60 deg).
//!
//! Each fault is an override rule on a port the dynamics register; see
//! [`bsk_sim::faults`] for the mapping and for what is not ported
//! (magnetometer spiking).
//!
//! Writes `examples/output/scenarios/scenario_FaultList.csv` every step:
//! wheel speeds and states, guidance, navigation translation, the WMM field,
//! the TAM reading, the spacecraft state and the Earth's `J20002Pfix`.
//! `plot_bsk_sim_fault_list.py` draws upstream's `attitudeErrorNorm`,
//! `rateError`, `RWSpeeds`, `Magnetometer` and `Latitude` figures.

#[path = "support/bsk_sim/mod.rs"]
mod bsk_sim;

use std::path::PathBuf;

use basilisk_rs::messages::{
    AttitudeGuidanceMsg, MagneticFieldMsg, NavigationTranslationMsg, Output, PlanetStateMsg,
    ReactionWheelStateMsg, RwSpeedMsg, SpacecraftStateMsg, TamSensorMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::{CsvSourceConfig, TelemetryField, TelemetryMessage};
use basilisk_rs::{Module, SimulationContext};
use bsk_sim::cli::Cli;
use bsk_sim::common::vector_columns;
use bsk_sim::dynamics;
use bsk_sim::faults::{Fault, FaultTargets, Wheels, configure_faults};
use bsk_sim::fsw::FswMode;
use bsk_sim::grav_factory::Body;
use bsk_sim::message_log::MessageLog;
use bsk_sim::{
    BskModels, HubInit, OMEGA_BN_B_INIT, RECORDER_PRIORITY, SIGMA_BN_INIT, elements,
    guidance_columns, min2nano, scenario_recorder, spacecraft_state_columns, translation_columns,
    wheel_speed_columns,
};
use nalgebra::Matrix3;
use serde::{Deserialize, Serialize};

const STEM: &str = "scenario_FaultList";

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The Earth's `J20002Pfix` from `planetStateOutMsgs[earth]`, whose
/// orientation the CSV columns of [`PlanetStateMsg`] leave out.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct EarthOrientationMsg {
    j2000_to_pfix: Matrix3<f64>,
}

impl TelemetryMessage for EarthOrientationMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        let mut fields = Vec::with_capacity(9);
        for row in 0..3 {
            for column in 0..3 {
                fields.push(TelemetryField {
                    path: format!("J20002Pfix_{row}{column}"),
                    value: self.j2000_to_pfix[(row, column)],
                });
            }
        }
        fields
    }
}

/// Republishes the Earth state's orientation for the recorder.
#[derive(Default)]
struct EarthOrientation {
    input: basilisk_rs::messages::Input<PlanetStateMsg>,
    output: Output<EarthOrientationMsg>,
}

impl Module for EarthOrientation {
    fn init(&mut self) {
        self.output.write(EarthOrientationMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let orientation = self.input.read().orientation;
        self.output.write(EarthOrientationMsg {
            j2000_to_pfix: orientation.map_or_else(Matrix3::identity, |o| o.inertial_to_fixed),
        });
    }
}

/// Upstream `scenario_FaultList`'s `configure_initial_conditions`.
fn initial_state() -> HubInit {
    HubInit::from_elements(
        Body::Earth.mu_m3ps2(),
        &elements(6_835_525.292, 0.00063, 98.0, 85.47, 0.0, 30.0),
        SIGMA_BN_INIT,
        OMEGA_BN_B_INIT,
    )
}

/// Upstream `run`'s `faultList`.
fn fault_list() -> Vec<Fault> {
    let fault_period = min2nano(1.0);
    let stop = min2nano(50.0);
    let count = stop / fault_period + 1;
    let mut faults =
        vec![Fault::rw_power(min2nano(5.0), 0.01, Wheels::One(1)).expect("valid fault")];
    faults.extend(
        (0..count).map(|index| {
            Fault::mag_polar_noise(index * fault_period, "NOISE").expect("valid fault")
        }),
    );
    faults
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    wheels: Vec<MessageLog<ReactionWheelStateMsg>>,
    guidance: MessageLog<AttitudeGuidanceMsg>,
    field: MessageLog<MagneticFieldMsg>,
    tam: MessageLog<TamSensorMsg>,
    states: MessageLog<SpacecraftStateMsg>,
    earth: MessageLog<EarthOrientationMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let mut models = BskModels::new(1.0, 1.0, &initial_state());
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());
    // `runScenario`: the mode, then `configure_faults`.
    mode.set(FswMode::HillPoint);
    let targets = FaultTargets::new(&models.base, &models.dynamics, &models.fsw);
    configure_faults(&mut models.base, &targets, &fault_list());

    let sampling = models.fsw.task_period_nanos;
    let mut wheels: Vec<MessageLog<ReactionWheelStateMsg>> =
        (0..4).map(|_| MessageLog::default()).collect();
    let mut guidance = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut field = MessageLog::<MagneticFieldMsg>::default();
    let mut tam = MessageLog::<TamSensorMsg>::default();
    let mut states = MessageLog::<SpacecraftStateMsg>::default();
    let mut earth = MessageLog::<EarthOrientationMsg>::default();
    let mut earth_orientation = EarthOrientation::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let dyn_models = &models.dynamics;
    let gateway = &models.fsw.gateway;
    dyn_models
        .earth
        .module
        .output_planet_msg
        .connect_to(&mut earth_orientation.input);
    gateway.att_guid.connect_to(&mut guidance.input);
    dyn_models
        .wmm
        .output_magnetic_field_msg
        .connect_to(&mut field.input);
    dyn_models.tam.output_tam_msg.connect_to(&mut tam.input);
    dyn_models
        .spacecraft
        .state_out
        .connect_to(&mut states.input);
    earth_orientation.output.connect_to(&mut earth.input);
    dyn_models
        .rw_speed_out_msg
        .connect_to(recorder.add_source::<RwSpeedMsg>(wheel_speed_columns()));
    gateway
        .att_guid
        .connect_to(recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns("")));
    dyn_models
        .simple_nav
        .translation_out_msg
        .connect_to(recorder.add_source::<NavigationTranslationMsg>(translation_columns("nav")));
    for (index, (state, log)) in dyn_models.rw_out_msgs.iter().zip(&mut wheels).enumerate() {
        state.connect_to(&mut log.input);
        state.connect_to(
            recorder.add_source::<ReactionWheelStateMsg>(CsvSourceConfig::columns([
                (
                    "applied_motor_torque_nm".to_string(),
                    format!("rwTorque_Nm_{index}"),
                ),
                (
                    "omega_radps".to_string(),
                    format!("rwOmega_rad_per_s_{index}"),
                ),
            ])),
        );
    }
    dyn_models
        .wmm
        .output_magnetic_field_msg
        .connect_to(
            recorder.add_source::<MagneticFieldMsg>(CsvSourceConfig::columns(vector_columns(
                "magnetic_field_inertial_t",
                "magField_N_T",
            ))),
        );
    dyn_models
        .tam
        .output_tam_msg
        .connect_to(
            recorder.add_source::<TamSensorMsg>(CsvSourceConfig::columns(vector_columns(
                "magnetic_field_sensor_t",
                "tam_S_T",
            ))),
        );
    dyn_models
        .spacecraft
        .state_out
        .connect_to(recorder.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("")));
    earth_orientation
        .output
        .connect_to(recorder.add_source::<EarthOrientationMsg>(""));

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "earthOrientation" => &mut earth_orientation, period = sampling, priority = RECORDER_PRIORITY;
        "attGuidMsg" => &mut guidance, period = sampling, priority = RECORDER_PRIORITY;
        "magLog" => &mut field, period = sampling, priority = RECORDER_PRIORITY;
        "tamLog" => &mut tam, period = sampling, priority = RECORDER_PRIORITY;
        "scStatesLog" => &mut states, period = sampling, priority = RECORDER_PRIORITY;
        "earthStateLog" => &mut earth, period = sampling, priority = RECORDER_PRIORITY;
    }
    for log in &mut wheels {
        simulation.add_module("rwLogs", log, sampling, RECORDER_PRIORITY);
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_PRIORITY);
    }

    runner.execute(&mut simulation, min2nano(50.0));
    drop(simulation);
    drop(recorder);

    Results {
        wheels,
        guidance,
        field,
        tam,
        states,
        earth,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env("bsk_sim_fault_list");
    cli.finish();
    let results = run(Config { write_csv: true });
    println!(
        "wrote {} (final |sigma_BR| = {:.3e})",
        results.csv_path.expect("CSV written").display(),
        results.guidance.last().sigma_br.norm(),
    );
}

#[cfg(test)]
mod tests {
    //! The scenario's effects, then upstream `test_bskFaults.py`.

    use super::*;
    use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::MrpFeedback;
    use basilisk_rs::overrides::Registry;
    use basilisk_rs::utilities::astro_constants::{REQ_EARTH, RP_EARTH};
    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
    use bsk_sim::faults::{FaultError, fault_type_in_list, faults_like, fixed_frame_to_lla};
    use bsk_sim::shared::Shared;
    use nalgebra::Vector3;
    use serde_json::Value;

    fn latitude_deg(state: &SpacecraftStateMsg, earth: &EarthOrientationMsg) -> f64 {
        let (latitude, _, _) = fixed_frame_to_lla(
            earth.j2000_to_pfix * state.position_m,
            REQ_EARTH * 1000.0,
            RP_EARTH / REQ_EARTH,
        );
        latitude.to_degrees()
    }

    /// After 5 minutes `RW2` delivers at most 0.01 W: its torque is limited
    /// to `P_max / |Omega|`, with the speed at the start of the step.
    #[test]
    fn rw2_is_power_limited_after_five_minutes() {
        let results = run(Config { write_csv: false });
        let rw2 = &results.wheels[1];
        let mut limited = false;
        for index in 1..rw2.values.len() {
            let power = (rw2.values[index].applied_motor_torque_nm
                * rw2.values[index - 1].omega_radps)
                .abs();
            if rw2.times_nanos[index] > min2nano(5.0) + 1_000_000_000 {
                assert!(
                    power <= 0.01 + 1e-12,
                    "{power} W at {}",
                    rw2.times_nanos[index]
                );
                limited |= power > 0.0099;
            }
        }
        assert!(limited, "the limit binds");
        // Before the fault the wheel is only torque-limited.
        let early_peak = rw2
            .values
            .iter()
            .take(300)
            .map(|state| (state.applied_motor_torque_nm * state.omega_radps).abs())
            .fold(0.0_f64, f64::max);
        assert!(early_peak > 0.01, "{early_peak}");
    }

    /// The magnetometer error (reading minus the WMM field in body axes) is
    /// bounded by the nominal walk bounds at low latitude, and grows towards
    /// the polar ones over the poles.
    #[test]
    fn the_magnetometer_is_noisier_over_the_poles() {
        let results = run(Config { write_csv: false });
        let (mut equatorial, mut polar) = (0.0_f64, 0.0_f64);
        let mut polar_samples = 0;
        for index in 1..results.tam.values.len() {
            let state = &results.states.values[index];
            let truth =
                mrp_to_dcm(state.sigma_bn) * results.field.values[index].magnetic_field_inertial_t;
            let error = (results.tam.values[index].magnetic_field_sensor_t - truth).amax();
            let latitude = latitude_deg(state, &results.earth.values[index]).abs();
            if latitude < 30.0 {
                equatorial = equatorial.max(error);
            } else if latitude > 75.0 {
                polar = polar.max(error);
                polar_samples += 1;
            }
        }
        assert!(polar_samples > 60, "the orbit passes over a pole");
        assert!(equatorial < 20e-9, "{equatorial} T");
        assert!(polar > 5.0 * equatorial, "{polar} T vs {equatorial} T");
    }

    // ---------------------------------------------------------------------
    // Upstream `test_bskFaults.py`.
    // ---------------------------------------------------------------------

    /// What a fault run leaves to inspect.
    struct FaultRun {
        registry: Registry,
        mrp_feedback_rws: Shared<MrpFeedback>,
        targets: FaultTargets,
    }

    impl FaultRun {
        fn effective(&self, target: &str) -> Value {
            self.registry.effective(target).expect("registered target")
        }
    }

    /// Upstream `run_faults(faults, stop_sec, initial_tam_a_matrix)`: the
    /// `FaultInjectionTestScenario` (upstream's default 0.1 s rates, the
    /// fault-list orbit, `hillPoint`).
    fn run_faults(
        faults: &[Fault],
        stop_s: f64,
        initial_tam_a_matrix: Option<Matrix3<f64>>,
    ) -> FaultRun {
        let mut models = BskModels::new(0.1, 0.1, &initial_state());
        let (mode, runner) = (models.fsw.mode_request(), models.base.runner());
        mode.set(FswMode::HillPoint);
        let targets = FaultTargets::new(&models.base, &models.dynamics, &models.fsw);
        configure_faults(&mut models.base, &targets, faults);
        if let Some(a_matrix) = initial_tam_a_matrix {
            let mut config = models.dynamics.tam.config.read();
            config.a_matrix = a_matrix;
            models.dynamics.tam.config.write(config);
        }
        let run = FaultRun {
            registry: models.base.registry.clone(),
            mrp_feedback_rws: models.fsw.mrp_feedback_rws.module.handle(),
            targets: targets.clone(),
        };
        let mut simulation = Simulation::new(dynamics::epoch(), false);
        models.schedule(&mut simulation);
        runner.execute(&mut simulation, bsk_sim::sec2nano(stop_s));
        run
    }

    fn number(value: &Value, pointer: &str) -> f64 {
        value
            .pointer(pointer)
            .and_then(Value::as_f64)
            .expect("number")
    }

    /// Upstream `test_fault_helpers`.
    #[test]
    fn fault_helpers() {
        let a = Fault::rw_power(0, 0.5, Wheels::One(0)).unwrap();
        let b = Fault::rw_friction(0, 2.0, 1).unwrap();
        let faults = vec![a.clone(), b.clone()];
        assert!(fault_type_in_list(&faults, &a));
        let found = faults_like(&faults, &b);
        assert_eq!(found.len(), 1);
        assert!(matches!(found[0], Fault::RwFriction { .. }));
    }

    /// Upstream `test_rw_power_fault`, wheels 0 to 3.
    #[test]
    fn rw_power_fault() {
        for wheel in 0..4 {
            let run = run_faults(
                &[Fault::rw_power(0, 0.25, Wheels::One(wheel)).unwrap()],
                1.0,
                None,
            );
            let config = run.effective(&format!("RW{}.config", wheel + 1));
            assert_eq!(number(&config, "/max_power_w"), 0.25);
        }
    }

    /// Upstream `test_rw_power_fault_all_and_invalid`.
    #[test]
    fn rw_power_fault_all_and_invalid() {
        let run = run_faults(&[Fault::rw_power(0, 0.2, Wheels::All).unwrap()], 1.0, None);
        assert_eq!(number(&run.effective("RW1.config"), "/max_power_w"), 0.2);
        assert_eq!(number(&run.effective("RW4.config"), "/max_power_w"), 0.2);
        assert!(matches!(
            Fault::rw_power(0, 0.2, Wheels::One(9)),
            Err(FaultError::Invalid(_))
        ));
        assert!(matches!(
            Fault::rw_power(0, 0.2, Wheels::One(4)),
            Err(FaultError::Invalid(_))
        ));
    }

    /// Upstream `test_rw_friction_fault`, wheels 0 and 2.
    #[test]
    fn rw_friction_fault() {
        let baseline = run_faults(&[], 1.0, None);
        for wheel in [0, 2] {
            let target = format!("RW{}.config", wheel + 1);
            let before = number(&baseline.effective(&target), "/coulomb_friction_nm");
            let run = run_faults(&[Fault::rw_friction(0, 3.0, wheel).unwrap()], 1.0, None);
            let after = number(&run.effective(&target), "/coulomb_friction_nm");
            assert_eq!(after, before * 3.0);
        }
        assert!(matches!(
            Fault::rw_friction(0, 3.0, 4),
            Err(FaultError::Invalid(_))
        ));
    }

    /// Upstream `test_debris_impact_on_off`. A location or direction of the
    /// wrong length cannot be written as `[f64; 3]`; the zero direction is
    /// still refused.
    #[test]
    fn debris_impact_on_off() {
        let run = run_faults(
            &[
                Fault::debris_impact_on(0, [0.0, 0.05, 0.0], [0.0, -1.0, 0.0], 0.1, 1000.0)
                    .unwrap(),
                Fault::debris_impact_off(bsk_sim::sec2nano(0.5)),
            ],
            1.0,
            None,
        );
        let force = run.effective("externalDisturbance.ext_force_body");
        for axis in 0..3 {
            assert_eq!(
                number(&force, &format!("/force_request_body_n/{axis}")),
                0.0
            );
        }
        assert!(matches!(
            Fault::debris_impact_on(0, [0.0; 3], [0.0; 3], 0.1, 8e3),
            Err(FaultError::Invalid(_))
        ));
    }

    /// The impact delivers `m v` of momentum per dynamics step it lasts.
    #[test]
    fn debris_impact_pushes_the_spacecraft() {
        let hit = run_faults(
            &[
                Fault::debris_impact_on(0, [0.0, 0.05, 0.0], [0.0, -1.0, 0.0], 0.1, 1000.0)
                    .unwrap(),
            ],
            0.3,
            None,
        );
        let force = hit.effective("externalDisturbance.ext_force_body");
        // 0.1 kg at 1 km/s over 0.1 s.
        assert!((number(&force, "/force_request_body_n/1") + 1000.0).abs() < 1e-9);
        let torque = hit.effective("externalDisturbance.ext_torque_body");
        // `r x F` with `r` along the force: no torque.
        assert_eq!(number(&torque, "/torque_request_body_nm/2"), 0.0);
    }

    /// Upstream `test_css_signal_fault`: the faulted sensors read zero.
    #[test]
    fn css_signal_fault() {
        let run = run_faults(
            &[Fault::css_signal(0, "CSSFAULT_OFF", &[0, 2, 7]).unwrap()],
            1.0,
            None,
        );
        for css in [1, 3, 8] {
            let output = run.effective(&format!("CSS{css}.output_sun_sensor_msg"));
            assert_eq!(number(&output, "/sensed_value"), 0.0);
        }
        assert!(matches!(
            Fault::css_signal(0, "CSSFAULT_OFF", &[8]),
            Err(FaultError::Invalid(_))
        ));
        assert!(matches!(
            Fault::css_signal(0, "CSSFAULT_STUCK", &[0]),
            Err(FaultError::Invalid(_))
        ));
    }

    /// The other CSS faults: stuck at the maximum, stuck where it was, and
    /// random.
    #[test]
    fn css_stuck_and_random_faults() {
        let run = run_faults(
            &[
                Fault::css_signal(0, "CSSFAULT_STUCK_MAX", &[0]).unwrap(),
                Fault::css_signal(0, "CSSFAULT_RAND", &[1]).unwrap(),
                Fault::css_signal(0, "CSSFAULT_STUCK_RAND", &[2]).unwrap(),
            ],
            1.0,
            None,
        );
        assert_eq!(
            number(
                &run.effective("CSS1.output_sun_sensor_msg"),
                "/sensed_value"
            ),
            2.0
        );
        for css in [2, 3] {
            let value = number(
                &run.effective(&format!("CSS{css}.output_sun_sensor_msg")),
                "/sensed_value",
            );
            assert!((0.0..=4.0).contains(&value), "{value}");
        }
    }

    /// Upstream `test_rw_bit_flip_fault`, wheels 0 and 2: the FSW's
    /// `GsMatrix_B` entry changes and stays nonzero.
    #[test]
    fn rw_bit_flip_fault() {
        for wheel in [0, 2] {
            let run = run_faults(&[Fault::rw_bit_flip(0, 0, wheel, 0).unwrap()], 1.0, None);
            let params = run.effective("rwMotorTorque.rw_params_in_msg");
            let flipped = number(&params, &format!("/spin_axes_body/{wheel}/0"));
            let nominal = bsk_sim::dynamics::rw_spin_axis(wheel)[0];
            assert!(flipped != 0.0);
            assert!(flipped != nominal);
            assert!((flipped - nominal).abs() < 1e-3);
            assert!(run.targets.messages()[0].starts_with("RW Bit Flip Fault"));
        }
        assert!(matches!(
            Fault::rw_bit_flip(0, 0, 4, 0),
            Err(FaultError::Invalid(_))
        ));
        assert!(matches!(
            Fault::rw_bit_flip(0, 16, 0, 0),
            Err(FaultError::Invalid(_))
        ));
        assert!(matches!(
            Fault::rw_bit_flip(0, 0, 0, 3),
            Err(FaultError::Invalid(_))
        ));
    }

    /// Upstream `test_rw_gain_bit_flip_fault`.
    #[test]
    fn rw_gain_bit_flip_fault() {
        let baseline = run_faults(&[], 1.0, None).mrp_feedback_rws.lock().config.k;
        let run = run_faults(&[Fault::rw_gain_bit_flip(0, 0, "K").unwrap()], 1.0, None);
        let flipped = run.mrp_feedback_rws.lock().config.k;
        assert!(flipped != baseline);
        // float16(3.5) is 0x4300; bit 0 makes it 0x4301.
        assert_eq!(flipped, 3.501953125);
        assert!(matches!(
            Fault::rw_gain_bit_flip(0, 16, "K"),
            Err(FaultError::Invalid(_))
        ));
    }

    /// Upstream `test_magnetometer_fault_modes`, its stuck-value half: the
    /// reading's axis holds the value. Spiking is not ported.
    #[test]
    fn magnetometer_fault_modes() {
        let run = run_faults(
            &[Fault::magnetometer(0, "MAG_FAULT_STUCK_VALUE", 0, 1.23).unwrap()],
            1.0,
            None,
        );
        let reading = run.effective("TAM_sensor.output_tam_msg");
        assert_eq!(number(&reading, "/magnetic_field_sensor_t/0"), 1.23);
        assert!(number(&reading, "/magnetic_field_sensor_t/1").abs() < 1e-3);
        assert!(matches!(
            Fault::magnetometer(0, "MAG_FAULT_STUCK_VALUE", 3, 1.0),
            Err(FaultError::Invalid(_))
        ));
        assert!(matches!(
            Fault::magnetometer(0, "MAG_FAULT_STUCK", 0, 1.0),
            Err(FaultError::Invalid(_))
        ));
        assert!(matches!(
            Fault::magnetometer(0, "MAG_FAULT_SPIKING", 1, 0.0),
            Err(FaultError::Unsupported(_))
        ));
    }

    /// A stuck-current axis keeps the value it had.
    #[test]
    fn magnetometer_stuck_current() {
        let run = run_faults(
            &[
                Fault::magnetometer(bsk_sim::sec2nano(0.5), "MAG_FAULT_STUCK_CURRENT", 2, 0.0)
                    .unwrap(),
            ],
            1.0,
            None,
        );
        let rules = run
            .registry
            .targets()
            .into_iter()
            .find(|(name, _)| name == "TAM_sensor.output_tam_msg")
            .expect("target")
            .1
            .rules();
        assert_eq!(rules.len(), 1);
    }

    /// Upstream `test_mag_polar_noise_modes`, its `"NOISE"` half: the TAM's
    /// `A` matrix is set to the identity and its noise and bounds to the
    /// latitude blend. `"SPIKE"` is not ported.
    #[test]
    fn mag_polar_noise_modes() {
        let run = run_faults(
            &[Fault::mag_polar_noise(0, "NOISE").unwrap()],
            1.0,
            Some(Matrix3::zeros()),
        );
        let config = run.effective("TAM_sensor.config");
        let a_matrix: Matrix3<f64> = serde_json::from_value(config["a_matrix"].clone()).unwrap();
        assert_eq!(a_matrix, Matrix3::identity());
        let noise: Matrix3<f64> =
            serde_json::from_value(config["p_matrix_sqrt_t"].clone()).unwrap();
        let bounds: Vector3<f64> = serde_json::from_value(config["walk_bounds_t"].clone()).unwrap();
        for axis in 0..3 {
            assert!((4e-9..=120e-9).contains(&noise[(axis, axis)]));
            assert!((15e-9..=600e-9).contains(&bounds[axis]));
            assert!(bounds[axis] >= 3.1 * noise[(axis, axis)]);
        }
        assert!(matches!(
            Fault::mag_polar_noise(0, "SPIKE"),
            Err(FaultError::Unsupported(_))
        ));
        assert!(matches!(
            Fault::mag_polar_noise(0, "OTHER"),
            Err(FaultError::Invalid(_))
        ));
    }
}
