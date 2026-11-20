//! A Monte Carlo campaign of the reaction-wheel attitude scenario: port of
//! upstream `examples/scenarioMonteCarloAttRW.py`.
//!
//! The simulation is `scenarioAttitudeFeedbackRW` with balanced wheels and
//! the voltage interface (`rwMotorVoltage`, 0-10 V, and a
//! `motorVoltageInterface`), on one 0.1 s task for 10 minutes. A campaign of
//! [`basilisk_rs::monte_carlo::MonteCarlo`] runs it 4 times with these
//! dispersions, each bound to a field of the run's [`Config`]:
//!
//! | parameter | dispersion |
//! |---|---|
//! | `sigma_BN` initial | normal about `[0.02, 0.04, -0.06]`, σ = 0.002 |
//! | `omega_BN_B` initial | normal about `[0.0002, -0.002, 0.006] rad/s`, σ = 0.01/3 °/s |
//! | hub mass | uniform on 750 kg ± 1 % |
//! | `r_BcB_B` | normal about 0, σ = `[0.01, 0.01, 0.02] / 3 m` |
//! | hub inertia | `InertiaTensorDispersion`: diagonal ± up to 1 kg m², turned by three 0.25° (1σ) angles |
//! | wheel spin axes | normal about the body axes, σ = 0.002/3 along and 0.001/3 across |
//! | wheel speeds | uniform on 100, 200, 300 RPM ± 1 % (in rad/s) |
//! | voltage-to-torque gains | uniform on 0.02 N m/V ± 1 % each |
//!
//! As upstream, the flight software keeps the nominal wheel axes and
//! inertia, created before the dispersions apply. The data each run retains
//! ([`Retained`], upstream's `RetentionPolicy` message logs) are archived
//! with its configuration in `examples/output/montecarlo/scenarioMonteCarloAttRW/`.
//!
//! Upstream `run(saveFigures, case, show_plots)` maps to `--case 1|2`:
//!
//! * 1: execute the campaign; load the archive; check the retained data;
//!   rerun the last case and compare its attitude history with the archived
//!   one; check that the runs' parameters and histories differ;
//! * 2: run upstream's three saved initial conditions,
//!   `examples/Support/run_MC_IC/run<i>.json` (copied to
//!   `examples/data/run_MC_IC/`), into
//!   `examples/output/montecarlo/scenarioMonteCarloAttRW_IC/`. Upstream's
//!   archive keys its parameters by attribute path; [`config_from_upstream`]
//!   maps each known path to its field explicitly.
//!
//! The script's default is case 1, and `--all` runs both cases of upstream's
//! `test_scenarioMonteCarloAttRW.py`. As upstream's data callback plots each
//! run, each run's retained data are written to
//! `examples/output/scenarios/scenarioMonteCarloAttRW_run<i>.csv` (case 1)
//! or `scenarioMonteCarloAttRW_IC<i>.csv` (case 2).
//!
//! Differences from upstream:
//!
//! * The histories are sampled every 1.2 s (`samplingTime` of 500 points on
//!   the 0.1 s task, 501 samples); upstream's `simulationTime // 499` is not
//!   a multiple of the task period.
//! * No module of this simulation draws random numbers in basilisk-rs, so
//!   there is no `RNGSeed` to disperse; [`MonteCarlo::disperse_seeds`] is on
//!   as upstream, with no seed registered.
//! * The wheel spin axes are normalized by the wheel model (upstream uses
//!   the dispersed `gsHat_B` as is).
//! * The archive is kept; upstream deletes it (`delete_data`).

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/rw_factory.rs"]
mod rw_factory;
#[path = "support/table_csv.rs"]
mod table_csv;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, bail};
use attitude::{
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, ScenarioOrbit, att_tracking_error, inertial_3d,
    mrp_feedback, sampling_period_nanos, vehicle_config,
};
use basilisk_rs::device_interface::motor_voltage_interface::{
    MotorVoltageInterface, MotorVoltageInterfaceConfig,
};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffector,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::fsw_algorithms::rw_motor_voltage::{RwMotorVoltage, RwMotorVoltageConfig};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, ArrayMotorVoltageMsg, AttitudeGuidanceMsg, NavigationTranslationMsg,
    Output, ReactionWheelStateMsg, RwSpeedMsg,
};
use basilisk_rs::monte_carlo::{
    Archive, CampaignOutcome, InertiaTensorDispersion, MonteCarlo, NormalVectorCartDispersion,
    RunInfo, UniformDispersion,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{MU_EARTH_M3PS2, rpm_to_radps};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use rw_factory::RwFactory;
use serde::{Deserialize, Serialize};

/// Upstream `NUMBER_OF_RUNS`.
const NUMBER_OF_RUNS: usize = 4;
const WHEEL_COUNT: usize = 3;
/// Upstream's nominal wheel speeds [RPM].
const WHEEL_SPEEDS_RPM: [f64; WHEEL_COUNT] = [100.0, 200.0, 300.0];
/// Upstream `voltageToTorqueGain = 0.2 / 10.0` [N m/V].
const VOLTAGE_TO_TORQUE_GAIN: f64 = 0.2 / 10.0;

/// The dispersed parameters of a run (upstream's dispersion targets).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Config {
    /// Upstream `hub.sigma_BNInit`.
    sigma_bn_init: Vector3<f64>,
    /// Upstream `hub.omega_BN_BInit` [rad/s].
    omega_bn_b_init_radps: Vector3<f64>,
    /// Upstream `hub.mHub` [kg].
    hub_mass_kg: f64,
    /// Upstream `hub.r_BcB_B` [m].
    hub_com_offset_m: Vector3<f64>,
    /// Upstream `hubref.IHubPntBc_B` [kg m²].
    hub_inertia_kg_m2: Matrix3<f64>,
    /// Upstream `RW<i>.gsHat_B`.
    wheel_spin_axes: [Vector3<f64>; WHEEL_COUNT],
    /// Upstream `RW<i>.Omega` [rad/s].
    wheel_speeds_radps: [f64; WHEEL_COUNT],
    /// Upstream `rwVoltageIO.voltage2TorqueGain` [N m/V].
    voltage_to_torque_gains: [f64; WHEEL_COUNT],
}

impl Config {
    /// Upstream `createScenarioAttitudeFeedbackRW()`'s values.
    fn nominal() -> Self {
        Self {
            sigma_bn_init: Vector3::new(0.1, 0.2, -0.3),
            omega_bn_b_init_radps: Vector3::new(0.001, -0.01, 0.03),
            hub_mass_kg: 750.0,
            hub_com_offset_m: Vector3::zeros(),
            hub_inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
            wheel_spin_axes: [Vector3::x(), Vector3::y(), Vector3::z()],
            wheel_speeds_radps: WHEEL_SPEEDS_RPM.map(rpm_to_radps),
            voltage_to_torque_gains: [VOLTAGE_TO_TORQUE_GAIN; WHEEL_COUNT],
        }
    }
}

/// Upstream's campaign: the controller and its dispersions.
fn campaign() -> MonteCarlo<Config> {
    let degree = std::f64::consts::PI / 180.0;
    let (minor, major) = (0.001 / 3.0, 0.002 / 3.0);
    let mut campaign = MonteCarlo::new(Config::nominal())
        .execution_count(NUMBER_OF_RUNS)
        .disperse_seeds(true)
        .disperse(
            |config: &mut Config| &mut config.sigma_bn_init,
            NormalVectorCartDispersion::new(
                Vector3::new(0.02, 0.04, -0.06),
                Vector3::repeat(0.002),
            ),
        )
        .disperse(
            |config: &mut Config| &mut config.omega_bn_b_init_radps,
            NormalVectorCartDispersion::new(
                Vector3::new(0.0002, -0.002, 0.006),
                Vector3::repeat(0.01 / 3.0 * degree),
            ),
        )
        .disperse(
            |config: &mut Config| &mut config.hub_mass_kg,
            UniformDispersion::new(750.0 * 0.99, 750.0 * 1.01),
        )
        .disperse(
            |config: &mut Config| &mut config.hub_com_offset_m,
            NormalVectorCartDispersion::new(
                Vector3::zeros(),
                Vector3::new(0.01 / 3.0, 0.01 / 3.0, 0.02 / 3.0),
            ),
        )
        .disperse(
            |config: &mut Config| &mut config.hub_inertia_kg_m2,
            InertiaTensorDispersion::with_std_angle(0.25 * degree),
        );
    let axis_stds = [
        Vector3::new(major, minor, minor),
        Vector3::new(minor, major, minor),
        Vector3::new(minor, minor, major),
    ];
    let axes = [Vector3::x(), Vector3::y(), Vector3::z()];
    for wheel in 0..WHEEL_COUNT {
        campaign = campaign.disperse(
            move |config: &mut Config| &mut config.wheel_spin_axes[wheel],
            NormalVectorCartDispersion::new(axes[wheel], axis_stds[wheel]),
        );
    }
    for (wheel, speed_rpm) in WHEEL_SPEEDS_RPM.into_iter().enumerate() {
        let nominal = rpm_to_radps(speed_rpm);
        campaign = campaign.disperse(
            move |config: &mut Config| &mut config.wheel_speeds_radps[wheel],
            UniformDispersion::new(nominal * 0.99, nominal * 1.01),
        );
    }
    for wheel in 0..WHEEL_COUNT {
        campaign = campaign.disperse(
            move |config: &mut Config| &mut config.voltage_to_torque_gains[wheel],
            UniformDispersion::new(VOLTAGE_TO_TORQUE_GAIN * 0.99, VOLTAGE_TO_TORQUE_GAIN * 1.01),
        );
    }
    campaign
}

/// What each run retains (upstream's `RetentionPolicy` message logs): the
/// sample times and the logged message fields.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Retained {
    times_nanos: Vec<u64>,
    /// `rwMotorTorqueMsg.motorTorque`.
    motor_torque_nm: Vec<[f64; WHEEL_COUNT]>,
    /// `guidMsg.sigma_BR`.
    sigma_br: Vec<Vector3<f64>>,
    /// `guidMsg.omega_BR_B`.
    omega_br_b_radps: Vec<Vector3<f64>>,
    /// `transMsg.r_BN_N`.
    r_bn_n_m: Vec<Vector3<f64>>,
    /// `rwSpeedMsg.wheelSpeeds`.
    wheel_speeds_radps: Vec<[f64; WHEEL_COUNT]>,
    /// `voltMsg.voltage`.
    voltage_v: Vec<[f64; WHEEL_COUNT]>,
    /// `rw<i>Msg.u_current`.
    rw_torque_nm: Vec<[f64; WHEEL_COUNT]>,
}

/// Upstream `createScenarioAttitudeFeedbackRW()` and `executeScenario(sim)`
/// for one configuration.
fn simulate(config: &Config) -> Retained {
    let mut rw_factory = RwFactory::default();
    for (wheel, axis) in [Vector3::x(), Vector3::y(), Vector3::z()]
        .into_iter()
        .enumerate()
    {
        let position = if wheel == 2 {
            Vector3::new(0.5, 0.5, 0.5)
        } else {
            Vector3::zeros()
        };
        rw_factory.create_honeywell_hr16(
            axis,
            50.0,
            WHEEL_SPEEDS_RPM[wheel],
            position,
            ReactionWheelModel::BalancedWheels,
        );
    }
    // The flight software uses the wheels as created, before the dispersions.
    let rw_config_msg = Output::new(rw_factory.config_message());
    let mut reaction_wheels = ReactionWheelStateEffector::new("RW_cluster");
    for (wheel, mut wheel_config) in rw_factory.wheels.into_iter().enumerate() {
        wheel_config.spin_axis_body = config.wheel_spin_axes[wheel];
        wheel_config.initial_omega_radps = config.wheel_speeds_radps[wheel];
        reaction_wheels.add_reaction_wheel(wheel_config);
    }
    let wheel_speed_msg = reaction_wheels.rw_speed_out_msg.clone();
    let wheel_state_msgs: Vec<Output<ReactionWheelStateMsg>> = reaction_wheels
        .wheels()
        .iter()
        .map(|wheel| wheel.state_out.clone())
        .collect();

    let (position_m, velocity_mps) = ScenarioOrbit::new(0.01).state();
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: config.hub_mass_kg,
        hub_center_of_mass_body_m: config.hub_com_offset_m,
        inertia_kg_m2: config.hub_inertia_kg_m2,
        integration_step_nanos: DYNAMICS_PERIOD_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: config.sigma_bn_init,
        initial_omega_radps: config.omega_bn_b_init_radps,
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
        .expect("one central body");
    let nominal_inertia = Config::nominal().hub_inertia_kg_m2;
    let vehicle_config_msg = Output::new(vehicle_config(nominal_inertia));

    let mut voltage_interface = MotorVoltageInterface::new(MotorVoltageInterfaceConfig {
        name: "rwVoltageInterface".to_string(),
        torque_gains_nm_per_v: config.voltage_to_torque_gains.to_vec(),
    });
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut inertial_reference = inertial_3d();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let mut controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    let mut rw_motor_voltage = RwMotorVoltage::new(RwMotorVoltageConfig {
        name: "rwMotorVoltage".to_string(),
        v_min_v: 0.0,
        v_max_v: 10.0,
        k: 0.0,
    });

    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 500);
    let mut torque_log = MessageLog::<ArrayMotorTorqueMsg>::default();
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut translation_log = MessageLog::<NavigationTranslationMsg>::default();
    let mut speed_log = MessageLog::<RwSpeedMsg>::default();
    let mut voltage_log = MessageLog::<ArrayMotorVoltageMsg>::default();
    let mut wheel_logs: Vec<MessageLog<ReactionWheelStateMsg>> =
        (0..WHEEL_COUNT).map(|_| MessageLog::default()).collect();

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    for (command, wheel) in voltage_interface
        .motor_torque_out_msgs
        .iter()
        .zip(reaction_wheels.wheels_mut())
    {
        simulation.connect(command, &mut wheel.motor_torque_in_msg);
    }
    spacecraft.add_state_effector(reaction_wheels);
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &rw_config_msg => &mut controller.rw_params_in_msg,
        &wheel_speed_msg => &mut controller.rw_speeds_in_msg,
        &controller.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut rw_motor_voltage.torque_in_msg,
        &rw_config_msg => &mut rw_motor_voltage.rw_params_in_msg,
        &rw_motor_voltage.voltage_out_msg => &mut voltage_interface.voltage_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut torque_log.input,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &navigation.translation_out_msg => &mut translation_log.input,
        &wheel_speed_msg => &mut speed_log.input,
        &rw_motor_voltage.voltage_out_msg => &mut voltage_log.input,
    );
    for (state, log) in wheel_state_msgs.iter().zip(&mut wheel_logs) {
        simulation.connect(state, &mut log.input);
    }
    // Upstream's task order: the spacecraft (priority 1) and wheels (2),
    // the voltage interface, navigation and flight software, the recorders.
    schedule! { simulation,
        "spacecraftBody" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "rwVoltageInterface" => &mut voltage_interface, period=DYNAMICS_PERIOD_NANOS, priority=90;
        "SimpleNavigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "inertial3D" => &mut inertial_reference, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "attErrorInertial3D" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "mrpFeedback" => &mut controller, period=DYNAMICS_PERIOD_NANOS, priority=50;
        "rwMotorTorque" => &mut rw_motor_torque, period=DYNAMICS_PERIOD_NANOS, priority=40;
        "rwMotorVoltage" => &mut rw_motor_voltage, period=DYNAMICS_PERIOD_NANOS, priority=30;
        "torque_log" => &mut torque_log, period=sample_period, priority=0;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "translation_log" => &mut translation_log, period=sample_period, priority=0;
        "speed_log" => &mut speed_log, period=sample_period, priority=0;
        "voltage_log" => &mut voltage_log, period=sample_period, priority=0;
    }
    for log in &mut wheel_logs {
        simulation.add_module("wheel_log", log, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);

    let first_three = |values: &[f64]| -> [f64; WHEEL_COUNT] { [values[0], values[1], values[2]] };
    Retained {
        times_nanos: guidance_log.times_nanos.clone(),
        motor_torque_nm: torque_log
            .values
            .iter()
            .map(|msg| first_three(&msg.motor_torque_nm))
            .collect(),
        sigma_br: guidance_log.values.iter().map(|msg| msg.sigma_br).collect(),
        omega_br_b_radps: guidance_log
            .values
            .iter()
            .map(|msg| msg.omega_br_b_radps)
            .collect(),
        r_bn_n_m: translation_log
            .values
            .iter()
            .map(|msg| msg.position_bn_n_m)
            .collect(),
        wheel_speeds_radps: speed_log
            .values
            .iter()
            .map(|msg| first_three(&msg.wheel_speeds_radps))
            .collect(),
        voltage_v: voltage_log
            .values
            .iter()
            .map(|msg| first_three(&msg.voltage_v))
            .collect(),
        rw_torque_nm: (0..wheel_logs[0].values.len())
            .map(|sample| {
                std::array::from_fn(|wheel| {
                    wheel_logs[wheel].values[sample].applied_motor_torque_nm
                })
            })
            .collect(),
    }
}

/// The campaign's run function (upstream's creation and execution
/// functions together).
fn run_case(config: &Config, _info: &RunInfo) -> anyhow::Result<Retained> {
    Ok(simulate(config))
}

/// Upstream's data callback `plotSim`: writes a run's retained data as a
/// scenario CSV for `plot_scenario_monte_carlo_att_rw.py`.
fn write_run_csv(stem: &str, data: &Retained) -> PathBuf {
    let mut headers = vec!["time_ns".to_string()];
    let groups = [
        "motorTorque_Nm",
        "rwTorque_Nm",
        "sigma_BR",
        "omega_BR_B_rad_per_s",
        "r_BN_N_m",
        "wheelSpeeds_rad_per_s",
        "rwVoltage_V",
    ];
    for group in groups {
        headers.extend((0..3).map(|index| format!("{group}_{index}")));
    }
    let rows: Vec<Vec<f64>> = (0..data.times_nanos.len())
        .map(|sample| {
            let mut row = vec![data.times_nanos[sample] as f64];
            row.extend(data.motor_torque_nm[sample]);
            row.extend(data.rw_torque_nm[sample]);
            row.extend(data.sigma_br[sample].iter());
            row.extend(data.omega_br_b_radps[sample].iter());
            row.extend(data.r_bn_n_m[sample].iter());
            row.extend(data.wheel_speeds_radps[sample]);
            row.extend(data.voltage_v[sample]);
            row
        })
        .collect();
    table_csv::write_table(stem, &headers, &rows)
}

/// Case 1: execute, load, rerun and compare. Returns the loaded archive.
fn monte_carlo_case(archive_dir: &Path, threads: Option<usize>) -> anyhow::Result<Archive> {
    let mut controller = campaign()
        .archive_dir(archive_dir)
        .show_progress(threads.is_none());
    if let Some(threads) = threads {
        controller = controller.threads(threads);
    }
    let outcome = controller.execute(run_case)?;
    if !outcome.failures.is_empty() {
        bail!("No runs should fail: {:?}", outcome.failures);
    }
    // The data can be loaded from disk, in this or another program.
    let loaded = Archive::open(archive_dir)?;
    let last = NUMBER_OF_RUNS - 1;
    let retained: Retained = loaded
        .retained_data(last)
        .context("Retained data should be available after execution")?;
    // A rerun from the archived parameters reproduces the run.
    let rerun = loaded.rerun_cases::<Config, _, _>([last], run_case)?;
    if !rerun.failures.is_empty() {
        bail!("Should rerun case successfully");
    }
    for (old, new) in retained
        .sigma_br
        .iter()
        .zip(&rerun.retained[&last].sigma_br)
    {
        if (old - new).abs().max() >= 0.001 {
            bail!("Outputs shouldn't change on runs if random seeds are same");
        }
    }
    Ok(loaded)
}

/// Upstream's archived parameter `name` of an initial-condition file, set
/// on `config`. The values are Python literals, which JSON parses.
fn apply_upstream_parameter(config: &mut Config, name: &str, value: &str) -> anyhow::Result<()> {
    let number = || -> anyhow::Result<f64> { Ok(value.trim().parse()?) };
    let vector = || -> anyhow::Result<Vector3<f64>> {
        let values: Vec<f64> = serde_json::from_str(value)?;
        Ok(Vector3::from_column_slice(&values))
    };
    match name {
        "TaskList[0].TaskModels[0].hub.sigma_BNInit" => config.sigma_bn_init = vector()?,
        "TaskList[0].TaskModels[0].hub.omega_BN_BInit" => config.omega_bn_b_init_radps = vector()?,
        "TaskList[0].TaskModels[0].hub.mHub" => config.hub_mass_kg = number()?,
        "TaskList[0].TaskModels[0].hub.r_BcB_B" => config.hub_com_offset_m = vector()?,
        "hubref.IHubPntBc_B" => {
            let rows: Vec<Vec<f64>> = serde_json::from_str(value)?;
            config.hub_inertia_kg_m2 = Matrix3::from_fn(|row, column| rows[row][column]);
        }
        "RW1.gsHat_B" => config.wheel_spin_axes[0] = vector()?,
        "RW2.gsHat_B" => config.wheel_spin_axes[1] = vector()?,
        "RW3.gsHat_B" => config.wheel_spin_axes[2] = vector()?,
        "RW1.Omega" => config.wheel_speeds_radps[0] = number()?,
        "RW2.Omega" => config.wheel_speeds_radps[1] = number()?,
        "RW3.Omega" => config.wheel_speeds_radps[2] = number()?,
        "rwVoltageIO.voltage2TorqueGain[0]" => config.voltage_to_torque_gains[0] = number()?,
        "rwVoltageIO.voltage2TorqueGain[1]" => config.voltage_to_torque_gains[1] = number()?,
        "rwVoltageIO.voltage2TorqueGain[2]" => config.voltage_to_torque_gains[2] = number()?,
        _ => bail!("unknown upstream parameter {name}"),
    }
    Ok(())
}

/// Upstream's saved initial condition `run<index>.json` as a configuration:
/// the nominal one with the saved parameters applied.
fn config_from_upstream(path: &Path) -> anyhow::Result<Config> {
    let text =
        std::fs::read_to_string(path).with_context(|| format!("cannot read {}", path.display()))?;
    let parameters: BTreeMap<String, String> = serde_json::from_str(&text)?;
    let mut config = Config::nominal();
    for (name, value) in &parameters {
        apply_upstream_parameter(&mut config, name, value)
            .with_context(|| format!("{}: {name} = {value}", path.display()))?;
    }
    Ok(config)
}

/// Upstream's initial-condition directory, `examples/Support/run_MC_IC`.
fn initial_condition_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("examples/data/run_MC_IC")
}

/// Case 2: run upstream's three saved initial conditions.
fn initial_conditions_case(
    archive_dir: &Path,
    threads: Option<usize>,
) -> anyhow::Result<CampaignOutcome<Retained>> {
    let configurations = (0..3)
        .map(|index| {
            let path = initial_condition_dir().join(format!("run{index}.json"));
            Ok((index, config_from_upstream(&path)?))
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
    if archive_dir.exists() {
        std::fs::remove_dir_all(archive_dir)?;
    }
    let mut controller = campaign().archive_dir(archive_dir);
    if let Some(threads) = threads {
        controller = controller.threads(threads);
    }
    let outcome = controller.run_configurations(configurations, run_case)?;
    if !outcome.failures.is_empty() {
        bail!("Should run ICs successfully: {:?}", outcome.failures);
    }
    Ok(outcome)
}

fn archive_dir(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("examples/output/montecarlo")
        .join(name)
}

fn main() -> anyhow::Result<()> {
    let mut cli = Cli::from_env("scenario_monte_carlo_att_rw [--case 1|2] [--all]");
    let cases = if cli.switch("all") {
        vec![1, 2]
    } else {
        // Upstream's `__main__` runs case 1.
        vec![cli.value("case", 1_u8)]
    };
    cli.finish();
    for case in cases {
        if case == 1 {
            let dir = archive_dir("scenarioMonteCarloAttRW");
            let loaded = monte_carlo_case(&dir, None)?;
            // Upstream `executeCallbacks()`: the data callback for each run.
            loaded.for_each_retained(0..NUMBER_OF_RUNS, |index, data: Retained| {
                write_run_csv(&format!("scenarioMonteCarloAttRW_run{index}"), &data);
            })?;
            println!(
                "case 1: {NUMBER_OF_RUNS} runs archived in {}; wrote examples/output/scenarios/scenarioMonteCarloAttRW_run<i>.csv",
                dir.display()
            );
        } else {
            let dir = archive_dir("scenarioMonteCarloAttRW_IC");
            initial_conditions_case(&dir, None)?;
            Archive::open(&dir)?.for_each_retained(0..3, |index, data: Retained| {
                write_run_csv(&format!("scenarioMonteCarloAttRW_IC{index}"), &data);
            })?;
            println!(
                "case 2: 3 initial conditions archived in {}; wrote examples/output/scenarios/scenarioMonteCarloAttRW_IC<i>.csv",
                dir.display()
            );
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    //! Ports the assertions upstream's `run` makes for both cases of
    //! `test_scenarioMonteCarloAttRW.py` (no run fails, retained data
    //! available, a rerun reproduces its run to 0.001, the dispersed
    //! parameters and attitude histories differ between runs, the saved
    //! initial conditions run), plus that every run still points the
    //! spacecraft. The `RNGSeed` check has no counterpart (no seeded
    //! module).

    use super::*;

    /// A temporary archive directory, removed when dropped.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new(name: &str) -> Self {
            Self(std::env::temp_dir().join(format!("{name}-{}", std::process::id())))
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn case_1_monte_carlo_runs_archive_and_rerun() {
        let dir = TempDir::new("scenarioMonteCarloAttRW");
        let loaded = monte_carlo_case(&dir.0, Some(2)).expect("case 1 passes upstream's checks");
        let params1: Config = loaded.parameters(NUMBER_OF_RUNS - 1).unwrap();
        let params2: Config = loaded.parameters(NUMBER_OF_RUNS - 2).unwrap();
        // "dispersion should be different in each run"
        assert_ne!(params1.sigma_bn_init, params2.sigma_bn_init);
        assert_ne!(params1.omega_bn_b_init_radps, params2.omega_bn_b_init_radps);
        assert_ne!(params1.hub_mass_kg, params2.hub_mass_kg);
        assert_ne!(params1.hub_com_offset_m, params2.hub_com_offset_m);
        assert_ne!(params1.hub_inertia_kg_m2, params2.hub_inertia_kg_m2);
        assert!((742.5..=757.5).contains(&params1.hub_mass_kg));
        for wheel in 0..WHEEL_COUNT {
            let gain = params1.voltage_to_torque_gains[wheel];
            assert!((0.0198..=0.0202).contains(&gain));
        }
        // "Dispersed runs should produce different attitude tracking histories"
        let run1: Retained = loaded.retained_data(NUMBER_OF_RUNS - 1).unwrap();
        let run2: Retained = loaded.retained_data(NUMBER_OF_RUNS - 2).unwrap();
        assert_ne!(run1.sigma_br, run2.sigma_br);
        assert_eq!(run1.times_nanos.len(), 501);
        // Every dispersed run still reaches the inertial attitude.
        loaded
            .for_each_retained(0..NUMBER_OF_RUNS, |index, data: Retained| {
                let last = data.sigma_br.last().unwrap().norm();
                assert!(last < 1e-3, "run {index}: {last}");
            })
            .unwrap();
    }

    #[test]
    fn case_2_runs_the_saved_initial_conditions() {
        let dir = TempDir::new("scenarioMonteCarloAttRW_IC");
        let outcome = initial_conditions_case(&dir.0, Some(2)).expect("case 2 runs");
        assert_eq!(outcome.retained.len(), 3);
        // The saved parameters, not dispersed again.
        let loaded = Archive::open(&dir.0).unwrap();
        let run0: Config = loaded.parameters(0).unwrap();
        assert_eq!(run0.hub_mass_kg, 777.2725585);
        assert_eq!(run0.wheel_speeds_radps[0], 99.97586904);
        assert_eq!(run0.hub_inertia_kg_m2[(0, 2)], -30.06671637364518);
        assert!(config_from_upstream(&initial_condition_dir().join("missing.json")).is_err());
        let mut unknown = Config::nominal();
        assert!(apply_upstream_parameter(&mut unknown, "RW4.Omega", "1.0").is_err());
    }
}
