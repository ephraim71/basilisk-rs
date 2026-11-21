//! Upstream `examples/mujoco/mujocoModels/BSK_MujocoDynamics.py`: the MuJoCo
//! scene of the thruster-arm scenarios and the analysis modules that read it.
//!
//! The `DynamicsTask` runs, every `dynRate`, the scene (`MJScene.fromFile`
//! with `svIntegratorRKF45` and `extraEoMCall`), then `MJSystemCoM`,
//! `MJSystemMassMatrix` and `MJJointReactionForces`, which read the scene's
//! state after its update. The scene is [`Shared`], so the scenario can set
//! the initial conditions after the simulation is initialized (upstream's
//! `InitializeSimulation()` followed by `configure_initial_conditions()`).

use basilisk_rs::integrators::{AdaptiveController, Rkf45};
use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::mujoco_dynamics::mj_joint_reaction_forces::MjJointReactionForces;
use basilisk_rs::mujoco_dynamics::mj_system_com::MjSystemCom;
use basilisk_rs::mujoco_dynamics::mj_system_mass_matrix::MjSystemMassMatrix;
use basilisk_rs::simulation::Simulation;

use super::config::ThrArmConfig;
use super::shared::Shared;
use super::{DYNAMICS_PRIORITY, sec2nano};

/// Upstream `svIntegratorAdaptiveRungeKutta`'s default `relTol` and
/// `absTol`.
pub const RKF45_RELATIVE_TOLERANCE: f64 = 1e-4;
pub const RKF45_ABSOLUTE_TOLERANCE: f64 = 1e-8;

/// Upstream `BSKMujocoDynamicsModels`.
pub struct MujocoDynamics {
    /// Upstream `processTasksTimeStep` \[ns\].
    pub task_period_nanos: u64,
    /// Upstream `scene` (`ModelTag = "mujocoScene"`).
    pub scene: Shared<MjScene>,
    /// Upstream `systemCoM`.
    pub system_com: MjSystemCom,
    /// Upstream `systemMassMatrix`.
    pub system_mass_matrix: MjSystemMassMatrix,
    /// Upstream `jointReactionForces`.
    pub joint_reaction_forces: MjJointReactionForces,
}

impl MujocoDynamics {
    /// Upstream `BSKMujocoDynamicsModels(SimBase, dynRate, config)`.
    ///
    /// # Panics
    ///
    /// If the model file cannot be loaded (upstream logs "MuJoCo dynamics
    /// configuration must provide a non-empty xmlPath").
    pub fn new(dyn_rate_s: f64, config: &ThrArmConfig) -> Self {
        let mut scene = MjScene::from_file(&config.xml_path)
            .unwrap_or_else(|error| panic!("MuJoCo model {}: {error}", config.xml_path.display()));
        // Upstream `setScene`.
        scene.extra_eom_call = true;
        scene.name = "mujocoScene".to_string();
        scene.set_integrator(Rkf45::new(AdaptiveController {
            rtol: config
                .integrator_relative_tolerance
                .unwrap_or(RKF45_RELATIVE_TOLERANCE),
            atol: config
                .integrator_absolute_tolerance
                .unwrap_or(RKF45_ABSOLUTE_TOLERANCE),
            ..AdaptiveController::default()
        }));
        let mut system_com = MjSystemCom::new(&scene);
        system_com.name = "systemCoM".to_string();
        let mut system_mass_matrix = MjSystemMassMatrix::new(&scene);
        system_mass_matrix.name = "systemMassMatrix".to_string();
        let mut joint_reaction_forces = MjJointReactionForces::new(&scene);
        joint_reaction_forces.name = "jointReactionForces".to_string();
        Self {
            task_period_nanos: sec2nano(dyn_rate_s),
            scene: Shared::new(scene),
            system_com,
            system_mass_matrix,
            joint_reaction_forces,
        }
    }

    /// Upstream's `AddModelToTask(taskName, ..., 200 ... 197)`.
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let period = self.task_period_nanos;
        simulation.add_module(
            "mujocoScene",
            &mut self.scene,
            period,
            DYNAMICS_PRIORITY + 200,
        );
        simulation.add_module(
            "systemCoM",
            &mut self.system_com,
            period,
            DYNAMICS_PRIORITY + 199,
        );
        simulation.add_module(
            "systemMassMatrix",
            &mut self.system_mass_matrix,
            period,
            DYNAMICS_PRIORITY + 198,
        );
        simulation.add_module(
            "jointReactionForces",
            &mut self.joint_reaction_forces,
            period,
            DYNAMICS_PRIORITY + 197,
        );
    }
}
