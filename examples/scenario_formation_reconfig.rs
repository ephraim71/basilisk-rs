//! Impulsive formation reconfiguration of a deputy relative to a chief: port
//! of upstream `examples/scenarioFormationReconfig.py`.
//!
//! Two 500 kg rigid spacecraft orbit a point-mass Earth, the chief on
//! `a = 11 000 km`, `e = 0.4`, `i = 60°`, `Ω = 90°`, `ω = 60°`, `M = 40°`.
//! `spacecraftReconfig` plans the impulsive burns that take the deputy to the
//! classical element difference `[0, 1e-4, 2e-4, -1e-4, -2e-4, -3e-4]`
//! within one chief orbit, fires them with two 22 N `MOOG_Monarc_22_6`
//! thrusters along `[1, 1, 1]/sqrt(3)`, and publishes the burn attitudes,
//! which `attTrackingError` and `mrpFeedback` (`K = 10`, `Ki = 2e-4`,
//! `P = 50`) track with an `extForceTorque` torque. One 2 s dynamics task
//! and one 2 s FSW task, the dynamics first (two processes of equal priority,
//! in creation order), for 1.1 chief orbits.
//!
//! Upstream builds the deputy's elements on the chief's object (`oe2 = oe`),
//! so its offsets, its `M2E(M2, oe.e)` and the orbit period of the run
//! duration all see the deputy's semi-major axis and eccentricity; so does
//! this port.
//!
//! The thrusters take one command each ([`SpacecraftReconfig::add_thruster_command_output`],
//! upstream's one `onTimeOutMsg` array).
//!
//! Upstream `run(show_plots, useRefAttitude)` maps to `--[no-]ref-attitude`:
//! connect `inertial3D` (`sigma_R0N = [1, 0, 0]`) as the attitude to hold
//! between burns, or hold none. The upstream script's `__main__` runs without;
//! `--all` runs both, as `test_scenarioFormationReconfig.py` does.
//!
//! Writes `examples/output/scenarios/scenarioFormationReconfig<ref>.csv`
//! (`<ref>` is `int(useRefAttitude)`) with both states, the deputy's
//! attitude error and reference and the thruster on-times;
//! `plot_scenario_formation_reconfig.py` redraws upstream's figures from it.

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
#[path = "support/thruster_factory.rs"]
mod thruster_factory;

use std::path::PathBuf;

use attitude::{att_tracking_error, mrp_feedback, scenario_recorder};
use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::thruster_dynamic_effector::ThrusterDynamicEffector;
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::fsw_algorithms::formation_flying::spacecraft_reconfig::{
    SpacecraftReconfig, SpacecraftReconfigConfig,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, AttitudeReferenceMsg, Output, SpacecraftStateMsg, ThrusterCommandMsg,
    VehicleConfigMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::MU_EARTH;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use formation::{
    InitialState, hub_inertia_kg_m2, mean_to_true_anomaly, rigid_spacecraft,
    spacecraft_state_columns,
};
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Vector3, Vector6};
use orbit_tools::{orbit_period_s, sec2nano};
use recording::sampling_time;
use thruster_factory::{ThrusterFactory, ThrusterModel};

const USAGE: &str = "\
scenario_formation_reconfig [--[no-]ref-attitude]
scenario_formation_reconfig --all   with and without the reference attitude (upstream's test)";

/// Upstream `dynTimeStep` and `fswTimeStep = macros.sec2nano(2.0)`.
const STEP_NANOS: u64 = 2_000_000_000;
/// Upstream `numDataPoints`.
const NUM_DATA_POINTS: u64 = 1000;
/// Upstream `scObject.hub.mHub` (both spacecraft) \[kg\].
const HUB_MASS_KG: f64 = 500.0;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    use_ref_attitude: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(use_ref_attitude: bool) -> Self {
        Self {
            use_ref_attitude,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!(
            "scenarioFormationReconfig{}",
            u8::from(self.use_ref_attitude)
        )
    }
}

/// The parametrizations of upstream `test_scenarioFormationReconfig.py`.
const TEST_PARAMETRIZATIONS: [Config; 2] = [Config::new(true), Config::new(false)];

/// Upstream's returned `dataPos, dataVel` (chief), `dataPos2, dataVel2`
/// (deputy) and `dataAttErr` recorder histories.
// The deputy history is only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    chief: MessageLog<SpacecraftStateMsg>,
    deputy: MessageLog<SpacecraftStateMsg>,
    attitude_error: MessageLog<AttitudeGuidanceMsg>,
    csv_path: Option<PathBuf>,
}

/// Upstream's chief and deputy elements, with upstream's aliasing of the two
/// (`oe2 = oe`): the deputy's mean anomaly is converted with the deputy's
/// eccentricity. Returns `(chief, deputy)`.
fn initial_elements() -> (ClassicElements, ClassicElements) {
    let degrees = std::f64::consts::PI / 180.0;
    let mean_anomaly = 40.0 * degrees;
    let chief = ClassicElements {
        semi_major_axis: 11_000.0 * 1e3,
        eccentricity: 0.4,
        inclination_rad: 60.0 * degrees,
        raan_rad: 90.0 * degrees,
        arg_periapsis_rad: 60.0 * degrees,
        true_anomaly_rad: mean_to_true_anomaly(mean_anomaly, 0.4),
        ..Default::default()
    };
    let eccentricity = chief.eccentricity - 0.0002;
    let deputy = ClassicElements {
        semi_major_axis: (1.0 + 0.0003) * chief.semi_major_axis,
        eccentricity,
        inclination_rad: chief.inclination_rad + 0.0001,
        raan_rad: chief.raan_rad + 0.0004,
        arg_periapsis_rad: chief.arg_periapsis_rad - 0.0001,
        true_anomaly_rad: mean_to_true_anomaly(mean_anomaly + 0.0002, eccentricity),
        ..Default::default()
    };
    (chief, deputy)
}

fn run(config: Config) -> Results {
    let mu = Body::Earth.mu_m3ps2();
    let inertia = hub_inertia_kg_m2();
    let (chief_oe, deputy_oe) = initial_elements();

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
            .add_grav_body(Body::Earth.point_mass(true))
            .expect("one central body");
    }

    // Upstream `thFactory2.create('MOOG_Monarc_22_6', [0, 0, 0], [1, 1, 1],
    // useMinPulseTime=False)`, twice.
    let mut factory = ThrusterFactory::default();
    for _ in 0..2 {
        factory.create(
            ThrusterModel::MoogMonarc22_6,
            Vector3::zeros(),
            Vector3::new(1.0, 1.0, 1.0),
            None,
        );
    }
    let thruster_config_msg = Output::new(factory.config_message());
    let mut thrusters: Vec<ThrusterDynamicEffector> = factory
        .thrusters
        .iter()
        .map(|thruster| ThrusterDynamicEffector::new(thruster.dynamic_effector_config()))
        .collect();
    let mut external_torque = ExtForceTorque::new(ExtForceTorqueConfig {
        name: "externalDisturbance2".to_string(),
        ..Default::default()
    });
    let mut chief_navigation = SimpleNavigation::new("simpleNavObject");
    let mut deputy_navigation = SimpleNavigation::new("simpleNavObject2");

    let vehicle_config_msg = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: inertia,
        mass_kg: HUB_MASS_KG,
        ..Default::default()
    });
    let mut inertial_reference = Inertial3D::new(Inertial3DConfig {
        name: "inertial_3D2".to_string(),
        sigma_r0n: Vector3::new(1.0, 0.0, 0.0),
    });
    let mut reconfig = SpacecraftReconfig::new(SpacecraftReconfigConfig {
        name: "spacecraftReconfig".to_string(),
        mu_m3ps2: MU_EARTH * 1e9,
        att_control_time_s: 400.0,
        target_classic_oed: Vector6::new(0.0000, 0.0001, 0.0002, -0.0001, -0.0002, -0.0003),
    });
    let commands: Vec<Output<ThrusterCommandMsg>> = thrusters
        .iter()
        .map(|_| reconfig.add_thruster_command_output().clone())
        .collect();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    tracking_error.config.name = "attError".to_string();
    let mut controller = mrp_feedback(10.0, 0.0002, 50.0, Vector3::zeros());

    // Upstream's deputy `oe.a` after the aliased offsets.
    let orbit_period = orbit_period_s(mu, deputy_oe.semi_major_axis);
    let duration_nanos = sec2nano(orbit_period * 1.1);
    let sample_nanos = sampling_time(duration_nanos, STEP_NANOS, NUM_DATA_POINTS).max(STEP_NANOS);
    let mut chief_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut deputy_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut attitude_error_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let (mut csv, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        for (index, (command, thruster)) in commands.iter().zip(&mut thrusters).enumerate() {
            connect!(&simulation,
                command => &mut thruster.command_in,
                command => csv.add_source::<ThrusterCommandMsg>(CsvSourceConfig::columns([(
                    "on_time_s".to_string(),
                    format!("OnTimeRequest_s_{index}"),
                )])),
            );
        }
        for thruster in thrusters {
            deputy.add_dynamic_effector(thruster);
        }
        connect!(&simulation,
            &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg,
        );
        deputy.add_dynamic_effector(external_torque);
        connect!(&simulation,
            &chief.state_out => &mut chief_navigation.spacecraft_state_in_msg,
            &deputy.state_out => &mut deputy_navigation.spacecraft_state_in_msg,
            &chief_navigation.translation_out_msg => &mut reconfig.chief_trans_in_msg,
            &deputy_navigation.translation_out_msg => &mut reconfig.deputy_trans_in_msg,
            &thruster_config_msg => &mut reconfig.thrust_config_in_msg,
            &vehicle_config_msg => &mut reconfig.vehicle_config_in_msg,
            &reconfig.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
            &deputy_navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
            &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
            &vehicle_config_msg => &mut controller.veh_config_in_msg,
            &chief.state_out => &mut chief_log.input,
            &deputy.state_out => &mut deputy_log.input,
            &tracking_error.att_guid_out_msg => &mut attitude_error_log.input,
            &chief.state_out => csv.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("chief")),
            &deputy.state_out => csv.add_source::<SpacecraftStateMsg>(spacecraft_state_columns("deputy")),
            &tracking_error.att_guid_out_msg => csv.add_source::<AttitudeGuidanceMsg>(
                CsvSourceConfig::columns(vector_columns("sigma_br", "sigma_BR")),
            ),
            &reconfig.att_ref_out_msg => csv.add_source::<AttitudeReferenceMsg>(
                CsvSourceConfig::columns(vector_columns("sigma_rn", "sigma_RN")),
            ),
        );
        if config.use_ref_attitude {
            connect!(&simulation,
                &inertial_reference.att_ref_out_msg => &mut reconfig.att_ref_in_msg,
            );
        }
        // Dynamics process, its recorders (upstream adds them to the dynamics
        // task, so they sample the FSW outputs of the previous tick), then the
        // FSW process.
        schedule! { simulation,
            "scObject" => &mut chief, period=STEP_NANOS, priority=50;
            "scObject2" => &mut deputy, period=STEP_NANOS, priority=50;
            "simpleNavObject" => &mut chief_navigation, period=STEP_NANOS, priority=40;
            "simpleNavObject2" => &mut deputy_navigation, period=STEP_NANOS, priority=40;
            "dataLog" => &mut chief_log, period=sample_nanos, priority=30;
            "dataLog2" => &mut deputy_log, period=sample_nanos, priority=30;
            "attErrLog" => &mut attitude_error_log, period=sample_nanos, priority=30;
            "inertial_3D2" => &mut inertial_reference, period=STEP_NANOS, priority=14;
            "spacecraftReconfig" => &mut reconfig, period=STEP_NANOS, priority=13;
            "attError" => &mut tracking_error, period=STEP_NANOS, priority=12;
            "mrpFeedback" => &mut controller, period=STEP_NANOS, priority=11;
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
        attitude_error: attitude_error_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env(USAGE);
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream's `__main__` runs `run(True, useRefAttitude=False)`.
        vec![Config::new(cli.flag("ref-attitude", false))]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!(
            "wrote {} ({} samples, final |sigma_BR| = {:.3e})",
            results.csv_path.as_ref().expect("CSV written").display(),
            results.chief.values.len(),
            results.attitude_error.last().sigma_br.norm(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioFormationReconfig.py`: the chief and deputy
    //! states (relative `1e-6`) and the deputy's attitude error (absolute
    //! `1e-6`) at every 200th recorded sample against upstream's truth.

    use super::*;
    use crate::formation::{compare_array, compare_array_relative, every_nth};

    const TRUE_POS: [[f64; 3]; 6] = [
        [-2668550.43581724, -6980898.10827464, 4622064.93739554],
        [3239719.11900864, -12361691.80735476, -5611358.11637523],
        [6956886.21097725, -6512028.87881183, -12049680.37988781],
        [6203357.9456249, 3386315.3204033, -10744531.13935835],
        [-904077.16857348, 6986198.37448257, 1565907.58993232],
        [-1497223.74594733, -9688096.70685282, 2593267.5982793],
    ];
    const TRUE_VEL: [[f64; 3]; 6] = [
        [1952.09236395, -6264.36921517, -3381.12315544],
        [2251.85678459, 773.95089899, -3900.33036227],
        [735.27848237, 3673.48086541, -1273.53968917],
        [-1521.51022886, 4060.99888361, 2635.33302062],
        [-3836.04015487, -3920.91042533, 6644.21644811],
        [2480.74592749, -4214.7585866, -4296.77798709],
    ];

    fn check(
        config: Config,
        true_pos2: [[f64; 3]; 6],
        true_vel2: [[f64; 3]; 6],
        true_att_err: [[f64; 3]; 6],
    ) {
        let results = run(Config {
            write_csv: false,
            ..config
        });
        // Upstream `skipValue = int(numDataPoints / numTruthPoints)`.
        let skip = (NUM_DATA_POINTS / 5) as usize;
        let positions = |log: &MessageLog<SpacecraftStateMsg>| {
            let values: Vec<_> = log.values.iter().map(|state| state.position_m).collect();
            every_nth(&values, skip)
        };
        let velocities = |log: &MessageLog<SpacecraftStateMsg>| {
            let values: Vec<_> = log.values.iter().map(|state| state.velocity_mps).collect();
            every_nth(&values, skip)
        };
        let attitude_errors: Vec<_> = results
            .attitude_error
            .values
            .iter()
            .map(|guidance| guidance.sigma_br)
            .collect();
        let accuracy = 1e-6;
        let failures: Vec<String> = [
            compare_array_relative(
                &TRUE_POS,
                &positions(&results.chief),
                accuracy,
                "chief r_BN_N Vector",
            ),
            compare_array_relative(
                &TRUE_VEL,
                &velocities(&results.chief),
                accuracy,
                "chief v_BN_N Vector",
            ),
            compare_array_relative(
                &true_pos2,
                &positions(&results.deputy),
                accuracy,
                "deputy r2_BN_N Vector",
            ),
            compare_array_relative(
                &true_vel2,
                &velocities(&results.deputy),
                accuracy,
                "deputy v2_BN_N Vector",
            ),
            compare_array(
                &true_att_err,
                &every_nth(&attitude_errors, skip),
                accuracy,
                "deputy attitude Error",
            ),
        ]
        .concat();
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn reconfiguration_with_reference_attitude_matches_upstream_truth() {
        check(
            TEST_PARAMETRIZATIONS[0],
            [
                [-2667507.97243685, -6983454.82971912, 4626165.55303272],
                [3241108.94387093, -12365029.53882338, -5606492.86033854],
                [6957594.00740821, -6517924.48451533, -12049169.67128781],
                [6203604.90565029, 3379408.49215067, -10749322.78076205],
                [-899274.80454994, 6988633.22340264, 1559582.60590812],
                [-1499288.94036976, -9685672.95831528, 2596285.54475707],
            ],
            [
                [1953.40294294, -6262.33384039, -3379.83469644],
                [2251.63426593, 772.82266533, -3901.38099724],
                [734.53359595, 3672.29757633, -1275.08087935],
                [-1519.3712049, 4061.85725527, 2633.58633737],
                [-3836.64595382, -3914.06736883, 6647.63278995],
                [2478.79370247, -4217.73758483, -4296.15323074],
            ],
            [
                [0.00000000e+00, 0.00000000e+00, 0.00000000e+00],
                [5.21804822e-15, 0.00000000e+00, 0.00000000e+00],
                [2.73188701e-03, 2.42888055e-03, -1.85264053e-03],
                [1.50892109e-01, 1.31129690e-02, 1.99974845e-02],
                [1.66533454e-15, 2.49085251e-16, 1.43583255e-16],
                [2.01960670e-12, 4.98741819e-12, 7.61317659e-12],
            ],
        );
    }

    #[test]
    fn reconfiguration_without_reference_attitude_matches_upstream_truth() {
        check(
            TEST_PARAMETRIZATIONS[1],
            [
                [-2667507.97243685, -6983454.82971912, 4626165.55303272],
                [3241108.94387093, -12365029.53882338, -5606492.86033854],
                [6957593.3006838, -6517926.49084233, -12049170.68096033],
                [6203601.49094786, 3379393.02430127, -10749332.43260227],
                [-899283.12085206, 6988580.86462193, 1559588.28436277],
                [-1499187.38519734, -9685744.8647062, 2596126.83315495],
            ],
            [
                [1953.40294294, -6262.33384039, -3379.83469644],
                [2251.63426593, 772.82266533, -3901.38099724],
                [734.53150821, 3672.2915335, -1275.08394125],
                [-1519.37151914, 4061.84582195, 2633.58351375],
                [-3836.64758968, -3914.12259276, 6647.64791968],
                [2478.82196216, -4217.65022651, -4296.20016979],
            ],
            [
                [0.00000000e+00, 0.00000000e+00, 0.00000000e+00],
                [1.01291777e-15, -2.81104312e-15, -2.10435630e-15],
                [-1.09830506e-03, 2.86489461e-03, 9.96740835e-04],
                [9.08383514e-02, 6.71564082e-02, -3.21870531e-02],
                [1.35545560e-15, 1.02827666e-15, -6.35661937e-16],
                [-5.65881607e-05, 3.19108458e-04, -1.34177223e-04],
            ],
        );
    }
}
