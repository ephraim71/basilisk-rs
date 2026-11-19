//! Spacecraft group tests. Upstream checks that each synced object runs its
//! pre- and post-integration once per step (`test_dynamicObject.cpp`); the
//! rest of upstream's synchronization tests are about pointer lifetimes (see
//! `integrators::synchronized`). Upstream's physical check of synchronized
//! spacecraft is the constraint effector's, ported with it.

use std::any::Any;
use std::sync::{Arc, Mutex};

use hifitime::{Duration, Epoch};
use nalgebra::{Matrix3, Vector3};

use super::SpacecraftGroup;
use crate::dynamics::gravity::GravBodyData;
use crate::dynamics::mean_reverting_noise_state_effector::{
    MeanRevertingNoiseStateEffector, MeanRevertingNoiseStateEffectorConfig,
};
use crate::dynamics::reaction_wheel_state_effector::{
    ReactionWheelStateEffector, ReactionWheelStateEffectorConfig,
};
use crate::integrators::stochastic::EulerMaruyama;
use crate::integrators::{AdaptiveController, Integrator, Rk4, Rkf45};
use crate::messages::{SpacecraftDiagnosticsMsg, SpacecraftStateMsg};
use crate::simulation::Simulation;
use crate::spacecraft::{
    BackSubMatrices, CouplingEffector, EffectorOutput, Spacecraft, SpacecraftConfig,
    SpacecraftGroupDynamics, StateEffector,
};
use crate::{Module, SimulationContext};

const MU_EARTH_M3PS2: f64 = 3.986_004_418e14;
const STEP_NANOS: u64 = 10_000_000;

fn start_epoch() -> Epoch {
    Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
}

fn context(tick: u64) -> SimulationContext {
    SimulationContext {
        current_sim_nanos: tick * STEP_NANOS,
        current_epoch: start_epoch()
            + Duration::from_total_nanoseconds((tick * STEP_NANOS) as i128),
    }
}

/// A tumbling spacecraft in LEO with an offset center of mass, a reaction
/// wheel and a mean-reverting noise state, `variant` perturbing its initial
/// state.
fn busy_spacecraft(variant: f64) -> Spacecraft {
    let radius_m = 7.0e6 + 1.0e4 * variant;
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 120.0 + variant,
        hub_center_of_mass_body_m: Vector3::new(0.02, -0.01, 0.05),
        inertia_kg_m2: Matrix3::new(10.0, 0.2, 0.1, 0.2, 12.0, 0.3, 0.1, 0.3, 8.0),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::new(radius_m, 0.0, 0.0),
        initial_velocity_mps: Vector3::new(0.0, (MU_EARTH_M3PS2 / radius_m).sqrt(), 10.0),
        initial_sigma_bn: Vector3::new(0.1, -0.2, 0.05 * variant),
        initial_omega_radps: Vector3::new(0.03, -0.02 * variant, 0.05),
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
            .expect("valid Earth gravity body"),
        )
        .expect("unique central gravity body");
    let mut wheels = ReactionWheelStateEffector::new("wheels");
    wheels.add_reaction_wheel(ReactionWheelStateEffectorConfig::balanced(
        "rw_x",
        Vector3::new(0.1, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        0.02,
        0.4,
    ));
    spacecraft.add_state_effector(wheels);
    spacecraft.add_state_effector(MeanRevertingNoiseStateEffector::new(
        MeanRevertingNoiseStateEffectorConfig {
            stationary_std: 0.3,
            time_constant_s: 0.5,
            ..MeanRevertingNoiseStateEffectorConfig::new("noise")
        },
    ));
    spacecraft
}

fn outputs(spacecraft: &Spacecraft) -> (SpacecraftStateMsg, SpacecraftDiagnosticsMsg) {
    (
        spacecraft.state_out.read(),
        spacecraft.diagnostics_out.read(),
    )
}

fn assert_bitwise_equal(
    (state, diagnostics): &(SpacecraftStateMsg, SpacecraftDiagnosticsMsg),
    (expected_state, expected_diagnostics): &(SpacecraftStateMsg, SpacecraftDiagnosticsMsg),
    context: &str,
) {
    assert_eq!(
        format!("{state:?}"),
        format!("{expected_state:?}"),
        "{context}"
    );
    assert_eq!(
        format!("{diagnostics:?}"),
        format!("{expected_diagnostics:?}"),
        "{context}"
    );
}

/// Runs `steps` updates after `t = 0`, returning the outputs of every update.
fn run_alone(
    mut spacecraft: Spacecraft,
    steps: u64,
) -> Vec<(SpacecraftStateMsg, SpacecraftDiagnosticsMsg)> {
    spacecraft.init();
    (0..=steps)
        .map(|tick| {
            spacecraft.update(&context(tick));
            outputs(&spacecraft)
        })
        .collect()
}

fn run_group(
    group: &mut SpacecraftGroup,
    steps: u64,
) -> Vec<Vec<(SpacecraftStateMsg, SpacecraftDiagnosticsMsg)>> {
    group.init();
    (0..=steps)
        .map(|tick| {
            group.update(&context(tick));
            group.spacecraft().iter().map(outputs).collect()
        })
        .collect()
}

/// The additivity check: a group of one spacecraft publishes bit for bit
/// what the spacecraft publishes on its own, under a fixed-step, an adaptive
/// and a stochastic integrator.
#[test]
fn a_one_member_group_integrates_like_the_spacecraft_alone() {
    type Pair = (
        Box<dyn Integrator<Spacecraft>>,
        Box<dyn Integrator<SpacecraftGroupDynamics>>,
    );
    let controller = AdaptiveController {
        rtol: 1.0e-12,
        atol: 1.0e-12,
        ..AdaptiveController::default()
    };
    type Factory = fn(AdaptiveController) -> Pair;
    let integrators: [(&str, Factory); 3] = [
        ("RK4", |_| (Box::new(Rk4), Box::new(Rk4))),
        ("RKF45", |controller| {
            (
                Box::new(Rkf45::new(controller)),
                Box::new(Rkf45::new(controller)),
            )
        }),
        ("Euler-Maruyama", |_| {
            (
                Box::new(EulerMaruyama::new().with_seed(7)),
                Box::new(EulerMaruyama::new().with_seed(7)),
            )
        }),
    ];
    for (name, factory) in integrators {
        let (alone_integrator, group_integrator) = factory(controller);
        let mut alone = busy_spacecraft(1.0);
        alone.config.integrator = Some(alone_integrator);
        let expected = run_alone(alone, 30);

        let mut group = SpacecraftGroup::new(vec![busy_spacecraft(1.0)]);
        group.integrator = Some(group_integrator);
        let actual = run_group(&mut group, 30);
        for (tick, (actual, expected)) in actual.iter().zip(&expected).enumerate() {
            assert_bitwise_equal(&actual[0], expected, &format!("{name} at tick {tick}"));
        }
    }
}

/// Spacecraft without coupling effectors advance under a fixed-step method
/// exactly as they would alone.
#[test]
fn uncoupled_spacecraft_integrate_as_they_would_alone() {
    let variants = [0.0, 1.0, 2.5];
    let mut group = SpacecraftGroup::new(variants.iter().map(|&v| busy_spacecraft(v)).collect());
    let together = run_group(&mut group, 20);
    for (index, &variant) in variants.iter().enumerate() {
        let alone = run_alone(busy_spacecraft(variant), 20);
        for (tick, (together, alone)) in together.iter().zip(&alone).enumerate() {
            assert_bitwise_equal(
                &together[index],
                alone,
                &format!("spacecraft {index} at tick {tick}"),
            );
        }
    }
}

/// What a [`Probe`] saw.
#[derive(Debug, Default)]
struct ProbeLog {
    init_counts: Vec<usize>,
    pre_integrations: Vec<(u64, f64)>,
    updates: Vec<u64>,
    stages: usize,
}

/// A coupling effector that pushes spacecraft 0 with `force_n` and a body
/// torque and spacecraft 1 back with `-force_n`, and checks at every stage
/// that the two states it sees belong to the same stage: spacecraft `i`
/// coasts from `start[i]` at `velocity[i]`, so both have covered the same
/// fraction of the step.
struct Probe {
    force_n: Vector3<f64>,
    torque_nm: Vector3<f64>,
    log: Arc<Mutex<ProbeLog>>,
}

impl CouplingEffector for Probe {
    fn name(&self) -> &str {
        "probe"
    }

    fn init(&mut self, spacecraft_count: usize) {
        self.log.lock().unwrap().init_counts.push(spacecraft_count);
    }

    fn pre_integration(&mut self, current_sim_nanos: u64, dt_seconds: f64) {
        self.log
            .lock()
            .unwrap()
            .pre_integrations
            .push((current_sim_nanos, dt_seconds));
    }

    fn add_loads(&mut self, states: &[SpacecraftStateMsg], loads: &mut [EffectorOutput]) {
        assert_eq!(states.len(), 2);
        // Spacecraft 1 is pushed opposite to spacecraft 0 and both start at
        // rest, so their displacements stay opposite: a spacecraft evaluated
        // at another stage than its partner breaks the symmetry.
        let displacement_0 = states[0].position_m - Vector3::new(0.0, 0.0, 0.0);
        let displacement_1 = states[1].position_m - Vector3::new(10.0, 0.0, 0.0);
        assert!(
            (displacement_0 + displacement_1).norm() <= 1.0e-13,
            "stage states of different stages: {displacement_0:?} vs {displacement_1:?}"
        );
        loads[0].force_inertial_n += self.force_n;
        loads[0].torque_body_nm += self.torque_nm;
        loads[1].force_inertial_n -= self.force_n;
        self.log.lock().unwrap().stages += 1;
    }

    fn update(&mut self, current_sim_nanos: u64) {
        self.log.lock().unwrap().updates.push(current_sim_nanos);
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

/// Counts the spacecraft's calls of a state effector without states.
struct CallCounter {
    pre_integrations: Arc<Mutex<usize>>,
    writes: Arc<Mutex<usize>>,
}

impl StateEffector for CallCounter {
    fn name(&self) -> &str {
        "counter"
    }
    fn state_len(&self) -> usize {
        0
    }
    fn initial_state(&self) -> Vec<f64> {
        Vec::new()
    }
    fn load_state(&mut self, _: &[f64]) {}
    fn pre_integration(&mut self, _: u64, _: f64) {
        *self.pre_integrations.lock().unwrap() += 1;
    }
    fn update_contributions(
        &self,
        _: &[f64],
        _: Vector3<f64>,
        _: Vector3<f64>,
        _: &mut BackSubMatrices,
    ) {
    }
    fn compute_derivatives(&self, _: &[f64], _: Vector3<f64>, _: Vector3<f64>) -> Vec<f64> {
        Vec::new()
    }
    fn write_outputs(&mut self, _: u64, _: &SpacecraftStateMsg) {
        *self.writes.lock().unwrap() += 1;
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

fn free_hub(position_m: Vector3<f64>) -> Spacecraft {
    Spacecraft::new(SpacecraftConfig {
        mass_kg: 50.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal_element(4.0),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    })
}

/// Two hubs pushed apart by a coupling effector, scheduled by the simulation:
/// the effector sees both stage states of every stage, and its loads reach
/// both spacecraft. Each spacecraft runs its pre-integration and publishes
/// once per step (upstream `ReplacementPreservesSyncedDynamics`), and the
/// effector initializes once, prepares every step and updates after every
/// publication, `t = 0` included.
#[test]
fn coupling_effectors_see_one_stage_and_load_both_spacecraft() {
    let log = Arc::new(Mutex::new(ProbeLog::default()));
    type Counter = Arc<Mutex<usize>>;
    let counts: Vec<(Counter, Counter)> = (0..2)
        .map(|_| (Arc::new(Mutex::new(0)), Arc::new(Mutex::new(0))))
        .collect();
    let mut spacecraft = vec![
        free_hub(Vector3::zeros()),
        free_hub(Vector3::new(10.0, 0.0, 0.0)),
    ];
    for (member, (pre_integrations, writes)) in spacecraft.iter_mut().zip(&counts) {
        member.add_state_effector(CallCounter {
            pre_integrations: pre_integrations.clone(),
            writes: writes.clone(),
        });
    }
    let force_n = Vector3::new(-2.0, 1.0, 0.5);
    let torque_nm = Vector3::new(0.0, 0.0, 0.2);
    let mut group = SpacecraftGroup::new(spacecraft);
    group.add_coupling_effector(Probe {
        force_n,
        torque_nm,
        log: log.clone(),
    });

    let steps = 50;
    {
        let mut simulation = Simulation::new(start_epoch(), false);
        simulation.add_module("pair", &mut group, STEP_NANOS, 0);
        simulation.run_for(steps * STEP_NANOS);
    }

    let duration_s = (steps * STEP_NANOS) as f64 * 1.0e-9;
    let [first, second] = group.spacecraft() else {
        unreachable!()
    };
    let (first, second) = (first.state_out.read(), second.state_out.read());
    let accuracy = 1.0e-12;
    assert!((first.accumulated_dv_cn_n_mps - force_n * duration_s / 50.0).norm() < accuracy);
    assert!((second.accumulated_dv_cn_n_mps + force_n * duration_s / 50.0).norm() < accuracy);
    assert!((first.omega_radps - torque_nm * duration_s / 4.0).norm() < accuracy);
    assert_eq!(second.omega_radps, Vector3::zeros());

    let log = log.lock().unwrap();
    assert_eq!(log.init_counts, vec![2]);
    assert_eq!(log.stages, 4 * steps as usize, "four RK4 stages per step");
    let expected_pre: Vec<(u64, f64)> = (0..steps)
        .map(|step| (step * STEP_NANOS, STEP_NANOS as f64 * 1.0e-9))
        .collect();
    assert_eq!(log.pre_integrations, expected_pre);
    let expected_updates: Vec<u64> = (0..=steps).map(|tick| tick * STEP_NANOS).collect();
    assert_eq!(log.updates, expected_updates);
    for (pre_integrations, writes) in &counts {
        assert_eq!(*pre_integrations.lock().unwrap(), steps as usize);
        // `init` and every update, `t = 0` included.
        assert_eq!(*writes.lock().unwrap(), steps as usize + 2);
    }
}

#[test]
#[should_panic(expected = "different integration step")]
fn group_members_must_share_the_integration_step() {
    let mut other = free_hub(Vector3::zeros());
    other.config.integration_step_nanos = 2 * STEP_NANOS;
    SpacecraftGroup::new(vec![free_hub(Vector3::zeros()), other]).init();
}

/// Upstream rejects an integrator set on a synced secondary.
#[test]
#[should_panic(expected = "set SpacecraftGroup::integrator instead")]
fn group_members_cannot_bring_their_own_integrator() {
    let mut other = free_hub(Vector3::zeros());
    other.config.integrator = Some(Box::new(Rk4));
    SpacecraftGroup::new(vec![free_hub(Vector3::zeros()), other]).init();
}
