//! Port of upstream `examples/scenarioOrbitFromOmm.py`: import a catalog of
//! CCSDS Orbit Mean-Elements Messages (OMM) that share one epoch and
//! propagate every satellite for two periods of the slowest orbit about a
//! point-mass Earth.
//!
//! The catalog is upstream's `dataForExamples/ommExample.kvn`, copied to
//! `examples/data/ommExample.kvn`; `--omm <path>` reads another KVN file.
//! Each satellite is its own [`Spacecraft`] (100 kg, 10 kg m^2), all in one
//! 10 s task with a 10 s state recorder, as upstream.
//!
//! # Differences from upstream
//!
//! Upstream's `ommHandling.satOmm2elem` reads XML, JSON, CSV or KVN and turns
//! the SGP4 mean elements into osculating J2000 elements by propagating them
//! with SGP4 to the epoch and rotating the TEME state to GCRF (IAU-76/80).
//! basilisk-rs has neither SGP4 nor the TEME rotation, so this port reads the
//! KVN encoding and takes the mean elements as two-body elements in the
//! inertial frame: `a` from the mean motion and `mu_earth`, the true anomaly
//! from the mean anomaly. The orbits therefore differ from upstream's by the
//! SGP4 short-period terms (kilometres); the propagation that follows, and
//! upstream's checks of it, are unchanged. The validation of the records
//! (bound orbit above the Earth, one common epoch) is upstream's.
//!
//! Writes `scenarioOrbitFromOmm<index>.csv` per satellite. `tests` ports both
//! tests of `test_scenarioOrbitFromOmm.py`.

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

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, eccentric_to_true_anomaly, elements_to_state, mean_to_eccentric_anomaly,
};
use cli::Cli;
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{nano2sec, orbit_period_s, sec2nano};
use recording::{scenario_csv, state_columns};

const USAGE: &str = "scenario_orbit_from_omm [--omm examples/data/ommExample.kvn]";

/// Upstream `step_seconds`.
const STEP_SECONDS: f64 = 10.0;

/// The example catalog.
pub fn default_omm_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/data/ommExample.kvn")
}

/// One OMM record: the part of upstream `OmmData` the scenario uses.
#[derive(Clone, Debug)]
pub struct OmmRecord {
    /// `OBJECT_NAME` (upstream `satName`).
    pub name: String,
    /// `NORAD_CAT_ID` (upstream `noradID`), empty when absent.
    pub norad_id: String,
    /// `EPOCH` as written (upstream `ommEpoch`).
    pub epoch: String,
    /// `MEAN_MOTION` (rev/day).
    pub mean_motion_rev_per_day: f64,
    /// The imported orbit (upstream `oe`); see the module docs.
    pub elements: ClassicElements,
}

/// Parses the KVN encoding: a record per `CCSDS_OMM_VERS` key, `COMMENT`
/// lines skipped, `[unit]` suffixes dropped (upstream `_parseOmmKvn`).
fn parse_kvn(text: &str) -> Vec<HashMap<String, String>> {
    let mut records = Vec::new();
    let mut current: HashMap<String, String> = HashMap::new();
    for line in text.lines().map(str::trim) {
        if line.is_empty() || line.starts_with("COMMENT") {
            continue;
        }
        let Some((key, value)) = line.split_once('=') else {
            continue;
        };
        let key = key.trim().to_uppercase();
        let mut value = value.trim();
        if value.ends_with(']')
            && let Some((number, _units)) = value.split_once('[')
        {
            value = number.trim();
        }
        if key == "CCSDS_OMM_VERS" && !current.is_empty() {
            records.push(std::mem::take(&mut current));
        }
        current.insert(key, value.to_string());
    }
    if !current.is_empty() {
        records.push(current);
    }
    records
}

/// Upstream `satOmm2elem` for a KVN file, with the two-body stand-in for
/// SGP4 described in the module docs.
pub fn read_omm(text: &str) -> Result<Vec<OmmRecord>, String> {
    let mu = Body::Earth.mu_m3ps2();
    parse_kvn(text)
        .into_iter()
        .map(|fields| {
            let number = |key: &str| -> Result<f64, String> {
                fields
                    .get(key)
                    .ok_or_else(|| format!("OMM record is missing {key}"))?
                    .parse()
                    .map_err(|_| format!("OMM field {key} is not a number"))
            };
            for (key, supported) in [
                ("CENTER_NAME", "EARTH"),
                ("REF_FRAME", "TEME"),
                ("TIME_SYSTEM", "UTC"),
                ("MEAN_ELEMENT_THEORY", "SGP4"),
            ] {
                if fields.get(key).map(|value| value.to_uppercase()).as_deref() != Some(supported) {
                    return Err(format!("only {key} = {supported} is supported"));
                }
            }
            let mean_motion_rev_per_day = number("MEAN_MOTION")?;
            let eccentricity = number("ECCENTRICITY")?;
            let mean_motion_radps = mean_motion_rev_per_day * std::f64::consts::TAU / 86_400.0;
            let mean_anomaly_rad = number("MEAN_ANOMALY")?.to_radians();
            let elements = ClassicElements {
                semi_major_axis: (mu / (mean_motion_radps * mean_motion_radps)).cbrt(),
                eccentricity,
                inclination_rad: number("INCLINATION")?.to_radians(),
                raan_rad: number("RA_OF_ASC_NODE")?.to_radians(),
                arg_periapsis_rad: number("ARG_OF_PERICENTER")?.to_radians(),
                true_anomaly_rad: eccentric_to_true_anomaly(
                    mean_to_eccentric_anomaly(mean_anomaly_rad, eccentricity),
                    eccentricity,
                ),
                ..Default::default()
            };
            Ok(OmmRecord {
                name: fields.get("OBJECT_NAME").cloned().unwrap_or_default(),
                norad_id: fields.get("NORAD_CAT_ID").cloned().unwrap_or_default(),
                epoch: fields
                    .get("EPOCH")
                    .cloned()
                    .ok_or("OMM record is missing EPOCH")?,
                mean_motion_rev_per_day,
                elements,
            })
        })
        .collect()
}

/// `YYYY-MM-DDThh:mm:ss[.ffffff]` (UTC) as an [`Epoch`].
fn parse_epoch(text: &str) -> Result<Epoch, String> {
    let invalid = || format!("invalid OMM epoch {text:?}");
    let (date, time) = text.split_once('T').ok_or_else(invalid)?;
    let date: Vec<i32> = date
        .split('-')
        .map(|field| field.parse().map_err(|_| invalid()))
        .collect::<Result<_, _>>()?;
    let time: Vec<f64> = time
        .split(':')
        .map(|field| field.parse().map_err(|_| invalid()))
        .collect::<Result<_, _>>()?;
    if date.len() != 3 || time.len() != 3 {
        return Err(invalid());
    }
    let seconds = time[2].floor();
    Ok(Epoch::from_gregorian_utc(
        date[0],
        date[1] as u8,
        date[2] as u8,
        time[0] as u8,
        time[1] as u8,
        seconds as u8,
        ((time[2] - seconds) * 1.0e9).round() as u32,
    ))
}

/// One propagated satellite (upstream's `satellites` entries).
pub struct Satellite {
    pub record: OmmRecord,
    pub time_s: Vec<f64>,
    pub position_m: Vec<Vector3<f64>>,
    pub velocity_mps: Vec<Vector3<f64>>,
    pub period_s: f64,
}

pub struct Results {
    pub epoch: Epoch,
    pub mu_m3ps2: f64,
    pub earth_radius_m: f64,
    pub satellites: Vec<Satellite>,
}

/// Upstream `run(show_plots, omm_path)`.
pub fn run(omm_path: &Path) -> Result<Results, String> {
    let text = std::fs::read_to_string(omm_path)
        .map_err(|error| format!("cannot read {}: {error}", omm_path.display()))?;
    let records = read_omm(&text)?;
    if records.is_empty() {
        return Err("The OMM file contains no usable records.".to_string());
    }
    let epoch_text = records[0].epoch.clone();
    if records.iter().any(|record| record.epoch != epoch_text) {
        return Err("This example requires all OMM records to share a common epoch.".to_string());
    }
    let epoch = parse_epoch(&epoch_text)?;
    let mu = Body::Earth.mu_m3ps2();
    let earth_radius_m = Body::Earth.radius_equator_m();
    for record in &records {
        let oe = &record.elements;
        if !(oe.semi_major_axis.is_finite() && oe.eccentricity.is_finite())
            || oe.semi_major_axis <= 0.0
            || !(0.0..1.0).contains(&oe.eccentricity)
            || oe.semi_major_axis * (1.0 - oe.eccentricity) <= earth_radius_m
        {
            return Err(format!(
                "{} must have a bound osculating orbit above Earth.",
                record.name
            ));
        }
    }

    let step_nanos = sec2nano(STEP_SECONDS);
    let periods: Vec<f64> = records
        .iter()
        .map(|record| orbit_period_s(mu, record.elements.semi_major_axis))
        .collect();
    let longest = periods.iter().copied().fold(0.0, f64::max);
    let stop_seconds = (2.0 * longest / STEP_SECONDS).ceil() * STEP_SECONDS;

    let mut vehicles: Vec<Spacecraft> = records
        .iter()
        .map(|record| {
            let (position_m, velocity_mps) = elements_to_state(mu, &record.elements);
            let mut vehicle = Spacecraft::new(SpacecraftConfig {
                mass_kg: 100.0,
                hub_center_of_mass_body_m: Vector3::zeros(),
                inertia_kg_m2: Matrix3::from_diagonal_element(10.0),
                integration_step_nanos: step_nanos,
                initial_position_m: position_m,
                initial_velocity_mps: velocity_mps,
                initial_sigma_bn: Vector3::zeros(),
                initial_omega_radps: Vector3::zeros(),
                integrator: None,
            });
            vehicle
                .add_grav_body(Body::Earth.point_mass(true))
                .expect("one central body");
            vehicle
        })
        .collect();
    let mut csvs: Vec<_> = (0..records.len())
        .map(|index| scenario_csv(&format!("scenarioOrbitFromOmm{index}")))
        .collect();
    let mut histories: Vec<MessageLog<SpacecraftStateMsg>> =
        records.iter().map(|_| MessageLog::default()).collect();
    {
        let mut simulation = Simulation::new(epoch, false);
        for ((index, vehicle), (csv, history)) in vehicles
            .iter_mut()
            .enumerate()
            .zip(csvs.iter_mut().zip(histories.iter_mut()))
        {
            simulation.connect(
                &vehicle.state_out,
                csv.add_source::<SpacecraftStateMsg>(state_columns()),
            );
            simulation.connect(&vehicle.state_out, &mut history.input);
            simulation.add_module(format!("ommSatellite{index}"), vehicle, step_nanos, 20);
            simulation.add_module(format!("csv{index}"), csv, step_nanos, 0);
            simulation.add_module(format!("recorder{index}"), history, step_nanos, 0);
        }
        simulation.run_for(sec2nano(stop_seconds));
    }
    for csv in &mut csvs {
        csv.flush().expect("flush scenario CSV");
    }

    let satellites = records
        .into_iter()
        .zip(histories)
        .zip(periods)
        .map(|((record, history), period_s)| Satellite {
            record,
            time_s: history
                .times_nanos
                .clone()
                .into_iter()
                .map(nano2sec)
                .collect(),
            position_m: history
                .values
                .iter()
                .map(|state| state.position_m)
                .collect(),
            velocity_mps: history
                .values
                .iter()
                .map(|state| state.velocity_mps)
                .collect(),
            period_s,
        })
        .collect();
    Ok(Results {
        epoch,
        mu_m3ps2: mu,
        earth_radius_m,
        satellites,
    })
}

fn main() {
    let mut args = Cli::from_env(USAGE);
    let path = args.value("omm", default_omm_path());
    match run(&path) {
        Ok(results) => {
            for (index, satellite) in results.satellites.iter().enumerate() {
                println!(
                    "wrote scenarioOrbitFromOmm{index}.csv: {} (NORAD {}), a = {:.1} km, {:.1} min period",
                    satellite.record.name,
                    satellite.record.norad_id,
                    satellite.record.elements.semi_major_axis / 1e3,
                    satellite.period_s / 60.0
                );
            }
        }
        Err(message) => {
            eprintln!("error: {message}");
            std::process::exit(1);
        }
    }
}

#[cfg(test)]
mod tests {
    use basilisk_rs::utilities::orbital_motion::{
        eccentric_to_mean_anomaly, eccentric_to_true_anomaly, elements_to_state,
        mean_to_eccentric_anomaly, true_to_eccentric_anomaly,
    };

    use super::{default_omm_path, run};

    fn assert_close(actual: nalgebra::Vector3<f64>, expected: nalgebra::Vector3<f64>, atol: f64) {
        for axis in 0..3 {
            assert!(
                (actual[axis] - expected[axis]).abs() <= atol,
                "{actual:?} vs {expected:?} (atol {atol})"
            );
        }
    }

    /// `test_scenario_orbit_from_omm`: import, epoch alignment, initial state
    /// and the final state against the Kepler solution (0.2 m, 2e-4 m/s).
    #[test]
    fn imported_orbits_propagate_as_kepler_orbits() {
        let results = run(&default_omm_path()).expect("example catalog imports");
        let ids: Vec<&str> = results
            .satellites
            .iter()
            .map(|satellite| satellite.record.norad_id.as_str())
            .collect();
        assert_eq!(ids, ["999000001", "999000002"]);
        for satellite in &results.satellites {
            let record = &satellite.record;
            assert_eq!(record.epoch, results.satellites[0].record.epoch);
            assert_eq!(satellite.time_s[0], 0.0);
            assert!(*satellite.time_s.last().unwrap() >= 2.0 * satellite.period_s);
            assert!(
                satellite
                    .position_m
                    .iter()
                    .all(|r| r.iter().all(|x| x.is_finite()))
            );
            assert!(
                satellite
                    .velocity_mps
                    .iter()
                    .all(|v| v.iter().all(|x| x.is_finite()))
            );

            let (initial_r, initial_v) = elements_to_state(results.mu_m3ps2, &record.elements);
            assert_close(satellite.position_m[0], initial_r, 1e-8);
            assert_close(satellite.velocity_mps[0], initial_v, 1e-11);

            // Kepler's equation, independently of the spacecraft integrator.
            let mut elements = record.elements;
            let e = elements.eccentricity;
            let initial_mean = eccentric_to_mean_anomaly(
                true_to_eccentric_anomaly(elements.true_anomaly_rad, e),
                e,
            );
            let mean_motion = (results.mu_m3ps2 / elements.semi_major_axis.powi(3)).sqrt();
            let final_mean = (initial_mean + mean_motion * satellite.time_s.last().unwrap())
                .rem_euclid(std::f64::consts::TAU);
            elements.true_anomaly_rad =
                eccentric_to_true_anomaly(mean_to_eccentric_anomaly(final_mean, e), e);
            let (expected_r, expected_v) = elements_to_state(results.mu_m3ps2, &elements);
            assert_close(*satellite.position_m.last().unwrap(), expected_r, 0.2);
            assert_close(*satellite.velocity_mps.last().unwrap(), expected_v, 2e-4);
        }
    }

    /// `test_scenario_rejects_mixed_epochs`.
    #[test]
    fn rejects_mixed_epochs() {
        let text = std::fs::read_to_string(default_omm_path()).expect("example catalog");
        let text = text.replacen("EPOCH = 2026-09-24", "EPOCH = 2026-09-25", 1);
        let path = std::env::temp_dir().join(format!(
            "basilisk-rs-mixed-epochs-{}.kvn",
            std::process::id()
        ));
        std::fs::write(&path, text).expect("write temporary catalog");
        let result = run(&path);
        std::fs::remove_file(&path).ok();
        let message = result.err().expect("mixed epochs are rejected");
        assert!(message.contains("common epoch"), "{message}");
    }
}
