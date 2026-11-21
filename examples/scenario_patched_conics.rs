//! Port of upstream `examples/scenarioPatchedConics.py`: an Earth-to-Jupiter
//! Hohmann transfer flown as patched conics, switching the central body from
//! the Earth to the Sun to Jupiter.
//!
//! Earth, Sun and Jupiter are point masses held at fixed positions (upstream
//! writes static planet messages): the Sun at the origin, the Earth one AU
//! and Jupiter 5.2 AU away along -y and +y. The spacecraft starts on a
//! 7000 km circular Earth orbit and, as upstream:
//!
//! 1. coasts (5 s steps, Earth central) to the burn angle of the departure
//!    hyperbola and burns tangentially;
//! 2. climbs out for 110 000 s;
//! 3. switches to the Sun as central body (adding the Earth's position and
//!    its 29.7859 km/s orbital velocity) and flies the heliocentric transfer
//!    with one-week steps;
//! 4. switches to 500 s steps half a week before arrival;
//! 5. switches to Jupiter as central body and flies six weeks past arrival.
//!
//! # The Rust counterpart of upstream's mid-run changes
//!
//! Upstream changes the central body (`isCentralBody`), the task period
//! (`updateTaskPeriod`) and the recorder interval (`updateTimeInterval`)
//! between `ExecuteSimulation` calls, and the spacecraft integrates from its
//! last update to the next task time. A basilisk-rs [`Spacecraft`] takes its
//! central body when the gravity body is added and integrates a fixed step,
//! so this port flies the run as legs: each leg is a fresh spacecraft and
//! simulation for one central body and one step size, started from the state
//! the previous leg ended with (re-expressed relative to the new central
//! body, as upstream does with `posRef`/`velRef.setState`). The task times
//! are upstream's: [`next_task_time`] is `SysModelTask::updatePeriod`, which
//! puts the first step after a period change off the new grid's spacing, and
//! that step is a leg of its own. The departure burn changes the velocity
//! between two legs. Upstream's recorder (`samplingTime`, then 20 min) is
//! applied to the tick states afterwards, and the samples are written with a
//! [`CsvRecorder`](basilisk_rs::telemetry::CsvRecorder).
//!
//! Writes `scenarioPatchedConics.csv`. `tests` ports
//! `test_scenarioPatchedConics.py`: the final Jupiter-relative position within
//! 1000 km (per component) of upstream's truth.

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

use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::messages::{Output, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use grav_factory::Body;
use hifitime::{Duration, Epoch};
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::sec2nano;
use recording::{sampling_time, scenario_csv, state_columns};

/// Upstream `rEarth` and `rJupiter` (m).
const R_EARTH_ORBIT_M: f64 = 149_598_023.0 * 1000.0;
const R_JUPITER_ORBIT_M: f64 = 778_298_361.0 * 1000.0;
/// Upstream `v_Earth` and Jupiter's `vel_N_Jup` (m/s).
const V_EARTH_MPS: f64 = 29.7859 * 1000.0;
const V_JUPITER_MPS: f64 = 13.0697 * 1000.0;
const STEP_EARTH_NANOS: u64 = 5_000_000_000;
const STEP_ARRIVAL_NANOS: u64 = 500_000_000_000;
/// Upstream `oneWeek`.
const WEEK_NANOS: u64 = 7 * 24 * 3600 * 1_000_000_000;

/// The central bodies of the legs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Central {
    Earth,
    Sun,
    Jupiter,
}

/// The fixed planet positions (m).
fn planet_position_m(body: Central) -> Vector3<f64> {
    match body {
        Central::Earth => Vector3::new(0.0, -R_EARTH_ORBIT_M, 0.0),
        Central::Sun => Vector3::zeros(),
        Central::Jupiter => Vector3::new(0.0, R_JUPITER_ORBIT_M, 0.0),
    }
}

/// Upstream `SysModelTask::updatePeriod`: the next task time after the period
/// changes from `old_period` to `new_period`, the task's next time having been
/// `next_start` (its first time is zero).
pub fn next_task_time(next_start: u64, old_period: u64, new_period: u64) -> u64 {
    if next_start > old_period {
        let mut start = (next_start / new_period) * new_period;
        if start <= next_start - old_period {
            start += new_period;
        }
        start
    } else {
        0
    }
}

/// The task ticks from `first` every `period` up to `stop` (upstream runs the
/// ticks at or before the stop time).
fn ticks(first: u64, period: u64, stop: u64) -> Vec<u64> {
    (0..)
        .map(|index| first + index * period)
        .take_while(|tick| *tick <= stop)
        .collect()
}

/// A hub state relative to the leg's central body.
#[derive(Clone, Copy, Debug)]
struct HubState {
    position_m: Vector3<f64>,
    velocity_mps: Vector3<f64>,
}

/// Flies one leg: `steps` steps of `step_nanos` about `central`, starting at
/// `start_nanos` from `state`. Returns the final hub state and pushes the
/// state message of every step (absolute times) to `ticks_out`.
fn fly_leg(
    central: Central,
    state: HubState,
    start_nanos: u64,
    step_nanos: u64,
    steps: u64,
    ticks_out: &mut Vec<(u64, SpacecraftStateMsg)>,
) -> HubState {
    if steps == 0 {
        return state;
    }
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: step_nanos,
        initial_position_m: state.position_m,
        initial_velocity_mps: state.velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    // Upstream `createEarth()`, `createJupiter()` (`jupiter barycenter`), `createSun()`.
    for (body, data) in [
        (Central::Earth, Body::Earth),
        (Central::Jupiter, Body::JupiterBarycenter),
        (Central::Sun, Body::Sun),
    ] {
        spacecraft
            .add_grav_body(
                GravBodyData::point_mass(
                    data.name(),
                    data.mu_m3ps2(),
                    body == central,
                    planet_position_m(body),
                    Vector3::zeros(),
                )
                .expect("valid point mass"),
            )
            .expect("unique gravity bodies");
    }
    let hub = spacecraft.hub_state_handle();
    let mut history = MessageLog::<SpacecraftStateMsg>::default();
    {
        let epoch = start_epoch() + Duration::from_total_nanoseconds(start_nanos as i128);
        let mut simulation = Simulation::new(epoch, false);
        connect!(&simulation, &spacecraft.state_out => &mut history.input);
        schedule! { simulation,
            "spacecraftBody" => &mut spacecraft, period=step_nanos, priority=10;
            "dataLog" => &mut history, period=step_nanos, priority=0;
        }
        simulation.run_for(steps * step_nanos);
    }
    // The leg's first sample is the previous leg's last state.
    ticks_out.extend(
        history
            .into_samples()
            .into_iter()
            .skip(1)
            .map(|(time, state)| (start_nanos + time, state)),
    );
    HubState {
        position_m: hub.position_m(),
        velocity_mps: hub.velocity_mps(),
    }
}

fn start_epoch() -> Epoch {
    Epoch::from_gregorian_utc_at_midnight(2019, 1, 1)
}

/// Flies to each of `ticks_in` (absolute times after `last_nanos`, the time
/// of `state`) about `central`: one leg per run of equal steps. Returns the
/// final state and its time.
fn fly_ticks(
    central: Central,
    mut state: HubState,
    mut last_nanos: u64,
    ticks_in: &[u64],
    ticks_out: &mut Vec<(u64, SpacecraftStateMsg)>,
) -> (HubState, u64) {
    let mut index = 0;
    while index < ticks_in.len() {
        let step = ticks_in[index] - last_nanos;
        let mut steps = 1;
        while index + (steps as usize) < ticks_in.len()
            && ticks_in[index + steps as usize] - ticks_in[index + steps as usize - 1] == step
        {
            steps += 1;
        }
        state = fly_leg(central, state, last_nanos, step, steps, ticks_out);
        last_nanos += steps * step;
        index += steps as usize;
    }
    (state, last_nanos)
}

/// Re-expresses `state` from the `from` to the `to` central body (upstream's
/// `posRef.setState(rN + pos)`, `velRef.setState(vN + vel)`), with the
/// velocity offset upstream adds.
fn change_central(
    state: HubState,
    from: Central,
    to: Central,
    velocity_offset_mps: Vector3<f64>,
) -> HubState {
    HubState {
        position_m: state.position_m + planet_position_m(from) - planet_position_m(to),
        velocity_mps: state.velocity_mps + velocity_offset_mps,
    }
}

pub struct Results {
    /// The recorded samples (upstream `dataLog`).
    pub samples: Vec<(u64, SpacecraftStateMsg)>,
    /// Upstream's return value: the final hub position relative to Jupiter (m).
    pub final_position_m: Vector3<f64>,
}

/// Upstream `run(show_plots)`.
pub fn run() -> Results {
    let mu_earth = Body::Earth.mu_m3ps2();
    let mu_sun = Body::Sun.mu_m3ps2();
    let r_leo = 7000.0 * 1000.0;
    let (r_e, v_e) = elements_to_state(
        mu_earth,
        &ClassicElements {
            semi_major_axis: r_leo,
            ..Default::default()
        },
    );
    let oe = state_to_elements(mu_earth, r_e, v_e);

    // Upstream's departure geometry.
    let transfer_a = (R_EARTH_ORBIT_M + R_JUPITER_ORBIT_M) * 0.5;
    let v_perihelion = (2.0 * mu_sun / R_EARTH_ORBIT_M - mu_sun / transfer_a).sqrt();
    let t2_nanos = sec2nano(std::f64::consts::PI / (mu_sun / transfer_a.powi(3)).sqrt());
    let n = (mu_earth / oe.semi_major_axis.powi(3)).sqrt();
    let v_earth_rel = v_perihelion - V_EARTH_MPS;
    let e_hyperbola = r_leo * v_earth_rel * v_earth_rel / mu_earth + 1.0;
    let v0 = (v_earth_rel * v_earth_rel + 2.0 * mu_earth / r_leo).sqrt();
    let delta_v1 = v0 - (mu_earth / r_leo).sqrt();
    let burn_angle = (1.0 / e_hyperbola).acos() + std::f64::consts::PI;
    let simulation_time_nanos = sec2nano(burn_angle / n);
    let sample_nanos = sampling_time(simulation_time_nanos, STEP_EARTH_NANOS, 100);

    let mut tick_states = Vec::new();
    // 1. Coast to the burn, Earth central, 5 s steps.
    let earth_ticks = ticks(STEP_EARTH_NANOS, STEP_EARTH_NANOS, simulation_time_nanos);
    let start = HubState {
        position_m: r_e,
        velocity_mps: v_e,
    };
    let (mut state, mut last) = fly_ticks(Central::Earth, start, 0, &earth_ticks, &mut tick_states);
    // The departure burn, along the Earth-relative velocity.
    state.velocity_mps += state.velocity_mps.normalize() * delta_v1;
    // 2. Climb out for 110 000 s.
    let climb_stop = simulation_time_nanos + sec2nano(110_000.0);
    let climb_ticks = ticks(last + STEP_EARTH_NANOS, STEP_EARTH_NANOS, climb_stop);
    (state, last) = fly_ticks(Central::Earth, state, last, &climb_ticks, &mut tick_states);

    // 3. The Sun becomes the central body; one-week task period.
    state = change_central(
        state,
        Central::Earth,
        Central::Sun,
        Vector3::new(V_EARTH_MPS, 0.0, 0.0),
    );
    let first_week = next_task_time(last + STEP_EARTH_NANOS, STEP_EARTH_NANOS, WEEK_NANOS);
    // Upstream's 20 incremental stops end at `T2 - oneWeek` past the climb.
    let cruise_stop = (climb_stop as f64 + t2_nanos as f64 - WEEK_NANOS as f64) as u64;
    let cruise_ticks = ticks(first_week, WEEK_NANOS, cruise_stop);
    (state, last) = fly_ticks(Central::Sun, state, last, &cruise_ticks, &mut tick_states);
    let recorder_switch_nanos = last;

    // 4. 500 s task period, recorder every 20 min.
    let next_week = cruise_ticks
        .last()
        .map_or(first_week, |tick| tick + WEEK_NANOS);
    let next_tick = next_task_time(next_week, WEEK_NANOS, STEP_ARRIVAL_NANOS);
    let approach_stop = (climb_stop as f64 + t2_nanos as f64 - WEEK_NANOS as f64 * 0.5) as u64;
    let approach_ticks = ticks(next_tick, STEP_ARRIVAL_NANOS, approach_stop);
    (state, last) = fly_ticks(Central::Sun, state, last, &approach_ticks, &mut tick_states);

    // 5. Jupiter becomes the central body.
    state = change_central(
        state,
        Central::Sun,
        Central::Jupiter,
        Vector3::new(V_JUPITER_MPS, 0.0, 0.0),
    );
    let next_tick = approach_ticks
        .last()
        .map_or(next_tick, |tick| tick + STEP_ARRIVAL_NANOS);
    let arrival_stop = climb_stop + t2_nanos + 6 * WEEK_NANOS;
    let arrival_ticks = ticks(next_tick, STEP_ARRIVAL_NANOS, arrival_stop);
    (state, _) = fly_ticks(
        Central::Jupiter,
        state,
        last,
        &arrival_ticks,
        &mut tick_states,
    );

    // Upstream's recorder on the tick states: at t = 0, then at the first
    // tick at least one interval after the last sample.
    let mut samples = Vec::new();
    samples.push((
        0,
        SpacecraftStateMsg {
            position_m: r_e + planet_position_m(Central::Earth),
            velocity_mps: v_e,
            ..SpacecraftStateMsg::default()
        },
    ));
    let mut next_record = sample_nanos;
    for (time, tick_state) in tick_states {
        if time >= next_record {
            let interval = if time > recorder_switch_nanos {
                sec2nano(20.0 * 60.0)
            } else {
                sample_nanos
            };
            samples.push((time, tick_state));
            next_record = time + interval;
        }
    }
    write_csv(&samples);
    Results {
        samples,
        final_position_m: state.position_m,
    }
}

/// Writes `samples` through a scenario CSV recorder, updated by hand at each
/// sample's time.
fn write_csv(samples: &[(u64, SpacecraftStateMsg)]) {
    let state = Output::new(SpacecraftStateMsg::default());
    let mut csv = scenario_csv("scenarioPatchedConics");
    state.connect_to(csv.add_source::<SpacecraftStateMsg>(state_columns()));
    csv.init();
    for (time, sample) in samples {
        state.write(sample.clone());
        csv.update(&SimulationContext {
            current_sim_nanos: *time,
            current_epoch: start_epoch() + Duration::from_total_nanoseconds(*time as i128),
        });
    }
    csv.flush().expect("flush scenario CSV");
}

fn main() {
    let results = run();
    let r = results.final_position_m;
    println!(
        "wrote scenarioPatchedConics.csv ({} samples): final Jupiter-relative position [{:.3}, {:.3}, {:.3}] m",
        results.samples.len(),
        r.x,
        r.y,
        r.z
    );
}

#[cfg(test)]
mod tests {
    use super::{next_task_time, run};

    /// `SysModelTask::updatePeriod`: the next start keeps to the old schedule
    /// and lands on the new period's grid.
    #[test]
    fn task_period_change_matches_upstream() {
        let week = 604_800_000_000_000;
        assert_eq!(next_task_time(120_000_000_000, 5_000_000_000, week), week);
        // 1 209 600 s is not a multiple of 500 s: the grid point below it.
        assert_eq!(
            next_task_time(2 * week, week, 500_000_000_000),
            2 * week - 100_000_000_000
        );
        assert_eq!(next_task_time(5, 5, 7), 0);
    }

    /// `test_scenarioPatchedConics.py`: `compareArray(truePos, dataPos,
    /// 1000000.0)`, each component within 1e6 m.
    #[test]
    fn final_position_matches_upstream() {
        let true_position = [19497456649.927856, 1947075805.9933228, 0.0];
        let results = run();
        for (axis, truth) in true_position.into_iter().enumerate() {
            let error = (results.final_position_m[axis] - truth).abs();
            assert!(
                error <= 1.0e6,
                "r_BN_N[{axis}] = {} m, truth {truth} m",
                results.final_position_m[axis]
            );
        }
    }
}
