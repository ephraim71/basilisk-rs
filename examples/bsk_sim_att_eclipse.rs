//! BskSim Sun-safe pointing through an eclipse: port of upstream
//! `examples/BskSim/scenarios/scenario_AttEclipse.py`.
//!
//! The BskSim spacecraft ([`bsk_sim::dynamics`]) on a circular 7000 km orbit
//! (`i = 33.3 deg`, `Omega = 48.2 deg`, `omega = 347.8 deg`, `f = 85.3 deg`),
//! starting at `sigma_BN = [0.1, 0.2, -0.3]`, `omega_BN = [0.001, -0.01,
//! 0.03] rad/s`, in the `sunSafePoint` mode for 60 minutes on 1 s tasks
//! (`BSKSim(fswRate=1, dynRate=1)`): `cssWlsEst` estimates the Sun direction
//! from the eight coarse Sun sensors, `sunSafePoint` points the body `+z`
//! axis at it, and the MRP steering law, `rate_servo` and `rwMotorTorque`
//! drive the four wheels. In the Earth's shadow the sensors read nothing, the
//! estimate is invalid and `sunSafePoint` holds the body rate at its (zero)
//! search rate.
//!
//! Writes `examples/output/scenarios/scenario_AttEclipse.csv` every FSW step:
//! illumination factor, wheel speeds, wheel torque commands, guidance and the
//! estimated Sun direction. `plot_bsk_sim_att_eclipse.py` draws upstream's
//! `attitudeErrorNorm`, `rwMotorTorque`, `rateError`, `rwSpeed`,
//! `illuminationFraction` and `sunDirectionVector` figures.
//!
//! Upstream's `test_bskTestScript.py` only runs the scenario; the tests check
//! the eclipse and the Sun pointing on either side of it.

#[path = "support/bsk_sim/mod.rs"]
mod bsk_sim;

use std::path::PathBuf;

use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, EclipseMsg, RwSpeedMsg, SunLineMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use bsk_sim::cli::Cli;
use bsk_sim::common::vector_columns;
use bsk_sim::dynamics;
use bsk_sim::fsw::FswMode;
use bsk_sim::grav_factory::Body;
use bsk_sim::message_log::MessageLog;
use bsk_sim::{
    BskModels, HubInit, OMEGA_BN_B_INIT, RECORDER_PRIORITY, SIGMA_BN_INIT, elements,
    guidance_columns, min2nano, motor_torque_columns, scenario_recorder, wheel_speed_columns,
};

const STEM: &str = "scenario_AttEclipse";

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    illumination: MessageLog<EclipseMsg>,
    guidance: MessageLog<AttitudeGuidanceMsg>,
    sun_line: MessageLog<SunLineMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let initial = HubInit::from_elements(
        Body::Earth.mu_m3ps2(),
        &elements(7_000_000.0, 0.0, 33.3, 48.2, 347.8, 85.3),
        SIGMA_BN_INIT,
        OMEGA_BN_B_INIT,
    );
    let mut models = BskModels::new(1.0, 1.0, &initial);
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    let sampling = models.fsw.task_period_nanos;
    let mut illumination = MessageLog::<EclipseMsg>::default();
    let mut guidance = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut sun_line = MessageLog::<SunLineMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let eclipse = &models.dynamics.eclipse.output_eclipse_msg;
    let gateway = &models.fsw.gateway;
    let css_estimate = &models.fsw.css_wls_est.module.nav_state_out_msg;
    eclipse.connect_to(&mut illumination.input);
    gateway.att_guid.connect_to(&mut guidance.input);
    css_estimate.connect_to(&mut sun_line.input);
    eclipse.connect_to(
        recorder.add_source::<EclipseMsg>(CsvSourceConfig::columns([(
            "illumination_factor",
            "illuminationFactor",
        )])),
    );
    models
        .dynamics
        .rw_speed_out_msg
        .connect_to(recorder.add_source::<RwSpeedMsg>(wheel_speed_columns()));
    gateway
        .cmd_rw_motor
        .connect_to(recorder.add_source::<ArrayMotorTorqueMsg>(motor_torque_columns()));
    gateway
        .att_guid
        .connect_to(recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns("")));
    css_estimate.connect_to(recorder.add_source::<SunLineMsg>(CsvSourceConfig::columns(
        vector_columns("sun_vector_body", "vehSunPntBdy"),
    )));

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "illuminationRec" => &mut illumination, period = sampling, priority = RECORDER_PRIORITY;
        "sunSafeRec" => &mut guidance, period = sampling, priority = RECORDER_PRIORITY;
        "cssEstRec" => &mut sun_line, period = sampling, priority = RECORDER_PRIORITY;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_PRIORITY);
    }

    mode.set(FswMode::SunSafePoint);
    runner.execute(&mut simulation, min2nano(60.0));
    drop(simulation);
    drop(recorder);

    Results {
        illumination,
        guidance,
        sun_line,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env("bsk_sim_att_eclipse");
    cli.finish();
    let results = run(Config { write_csv: true });
    let shadow = results
        .illumination
        .values
        .iter()
        .filter(|eclipse| eclipse.illumination_factor < 1.0)
        .count();
    println!(
        "wrote {} ({shadow} s in shadow, final |sigma_BR| = {:.3e})",
        results.csv_path.expect("CSV written").display(),
        results.guidance.last().sigma_br.norm(),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The orbit passes through the Earth's shadow, about a third of it.
    /// There the sensors see nothing: the Sun estimate is invalid and the
    /// guidance holds the body still. In sunlight the body `+z` axis is
    /// pointed at the Sun, before the eclipse and again after it.
    #[test]
    fn the_spacecraft_points_at_the_sun_outside_the_eclipse() {
        let results = run(Config { write_csv: false });
        let illumination: Vec<f64> = results
            .illumination
            .values
            .iter()
            .map(|eclipse| eclipse.illumination_factor)
            .collect();
        let shadow = illumination.iter().filter(|factor| **factor == 0.0).count();
        assert!((1800..2400).contains(&shadow), "{shadow} s in full shadow");
        let entry = illumination
            .iter()
            .position(|factor| *factor < 1.0)
            .expect("an eclipse");
        let exit = illumination
            .iter()
            .rposition(|factor| *factor < 1.0)
            .expect("an eclipse");

        // The sensors read the eclipse of the previous step, and the
        // recorded guidance is the previous step's.
        for index in entry + 2..exit {
            if illumination[index - 2..=index]
                .iter()
                .all(|factor| *factor == 0.0)
            {
                let sun_line = &results.sun_line.values[index];
                assert!(!sun_line.valid, "a Sun estimate in the shadow at {index} s");
                let guidance = &results.guidance.values[index];
                assert_eq!(guidance.sigma_br, nalgebra::Vector3::zeros());
            }
        }
        // Sunlit and settled: the two minutes before the eclipse, and the
        // end of the run.
        for index in (entry - 120..entry)
            .chain(results.guidance.values.len() - 60..results.guidance.values.len())
        {
            let sun_line = &results.sun_line.values[index];
            assert!(sun_line.valid);
            assert!(
                sun_line.sun_vector_body.z > 0.9999,
                "{index}: {:?}",
                sun_line.sun_vector_body
            );
            assert!(results.guidance.values[index].sigma_br.norm() < 1e-3);
        }
    }
}
