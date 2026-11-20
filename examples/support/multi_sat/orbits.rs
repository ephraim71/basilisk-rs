//! The run the three orbit-only MultiSat scenarios share
//! (`scenario_BasicOrbitMultiSat`, its `_MT` variant and
//! `scenario_constellationFromTle`): spacecraft with the MultiSat dynamics
//! and no flight software, each from its orbit elements, their navigation
//! and wheels recorded (upstream's `log_outputs`), on one or more threads.

use basilisk_rs::utilities::orbital_motion::ClassicElements;
use nalgebra::Vector3;

use super::environment::Planet;
use super::logs::{Logged, SpacecraftLogs};
use super::{MultiSatSim, Rates, dynamics_process_name};

/// One spacecraft's initial conditions: upstream's `oe[i]`, `sigma_BNInit`
/// and `omega_BN_BInit`.
#[derive(Clone, Copy, Debug)]
pub struct InitialOrbit {
    pub elements: ClassicElements,
    pub sigma_bn: Vector3<f64>,
    pub omega_bn_b_radps: Vector3<f64>,
}

/// What [`run_orbits`] runs.
#[derive(Clone, Debug)]
pub struct OrbitRun {
    pub planet: Planet,
    pub rates: Rates,
    /// Upstream `samplingTime` \[ns\].
    pub sampling_nanos: u64,
    /// Upstream `simulationTime` \[ns\].
    pub duration_nanos: u64,
    /// Upstream `TotalSim.resetThreads(numThreads)`; 1 runs serially.
    pub threads: usize,
}

/// Builds the simulation, sets the initial conditions, records every
/// spacecraft and runs; returns the histories in spacecraft order.
pub fn run_orbits(run: &OrbitRun, initial: &[InitialOrbit]) -> Vec<SpacecraftLogs> {
    let mut sim = MultiSatSim::new(run.planet, initial.len(), run.rates, false);
    for (index, orbit) in initial.iter().enumerate() {
        sim.set_initial_elements(
            index,
            &orbit.elements,
            orbit.sigma_bn,
            orbit.omega_bn_b_radps,
        );
    }
    let mut logs: Vec<SpacecraftLogs> = (0..initial.len())
        .map(|index| SpacecraftLogs::new(&sim, index, run.sampling_nanos, Logged::ORBIT))
        .collect();
    {
        let mut processes = sim.processes();
        for log in &mut logs {
            log.schedule(processes.process(&dynamics_process_name(log.index)));
        }
        processes.reset_threads(run.threads);
        processes.execute_to(run.duration_nanos);
    }
    logs
}
