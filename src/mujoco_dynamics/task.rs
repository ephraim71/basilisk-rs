//! Models of a scene's dynamics tasks: the basilisk-rs form of upstream
//! Basilisk's `SysModelTask dynamicsTask`, `StatefulSysModel` and
//! `DynParamRegisterer` (`simulation/mujocoDynamics/_GeneralModuleFiles`).
//!
//! Upstream runs any `SysModel` in a scene's dynamics task, once per
//! integrator stage, and a `StatefulSysModel` additionally registers
//! continuous states whose derivatives (and diffusions) it sets in
//! `UpdateState`. Here a [`SceneModel`] is what a scene task holds:
//!
//! - [`SceneModel::scene_update`] is upstream `UpdateState`. Its
//!   [`TaskContext`] carries the stage's simulation time, read access to the
//!   scene's MuJoCo data, the model's states at the stage, and setters for
//!   their derivatives and diffusions.
//! - [`SceneModel::state_specs`] is upstream `registerStates`: the model's
//!   states, their initial values and their noise sources.
//! - [`SceneModel::scene_reset`] is upstream `Reset`, at every scene reset.
//! - [`SceneModel::load_states`] hands the committed state values back after
//!   every reset and step, which is what the models' state getters (upstream
//!   `getStateValue`, `getX`, `getIntegralError`) read.
//!
//! Like upstream `StateData`, the derivative and diffusion a model sets
//! persist until it sets them again, and start at zero on every reset.
//!
//! Any [`Module`] runs in a task through [`ModuleTaskModel`], as upstream adds
//! a plain `SysModel` (an interpolator, an ephemeris) to the dynamics task.

use std::any::Any;

use hifitime::Epoch;
use mujoco_rs::prelude::{MjData, MjModel};

use super::body::MjBody;
use crate::{Module, SimulationContext};

/// The MuJoCo model and data of a scene. The data owns the model; read it
/// with [`MjData::model`].
pub type MjSceneData = MjData<Box<MjModel>>;

/// Downcasting support for task models, implemented for every `'static`
/// type.
pub trait AsAny {
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

impl<T: Any> AsAny for T {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

/// One continuous state of a [`SceneModel`] (upstream
/// `DynParamRegisterer::registerState`).
#[derive(Clone, Debug, PartialEq)]
pub struct ModelStateSpec {
    /// State name, local to the model (upstream prefixes it with the model
    /// tag and module id).
    pub name: String,
    /// Initial value; its length is the state size.
    pub initial: Vec<f64>,
    /// Independent Wiener processes driving the state (upstream
    /// `StateSpec::noiseCount`).
    pub noise_sources: usize,
}

impl ModelStateSpec {
    /// A deterministic state.
    pub fn new(name: impl Into<String>, initial: Vec<f64>) -> Self {
        Self {
            name: name.into(),
            initial,
            noise_sources: 0,
        }
    }

    /// The same state driven by `noise_sources` Wiener processes.
    pub fn with_noise_sources(mut self, noise_sources: usize) -> Self {
        self.noise_sources = noise_sources;
        self
    }
}

/// Read access to a scene for its task models: the MuJoCo model and data at
/// the current evaluation, and the bodies.
#[derive(Clone, Copy)]
pub struct SceneView<'a> {
    pub(crate) data: &'a MjSceneData,
    pub(crate) bodies: &'a [MjBody],
}

impl<'a> SceneView<'a> {
    /// The MuJoCo data, after the forward kinematics of the evaluation's
    /// state.
    pub fn data(&self) -> &'a MjSceneData {
        self.data
    }

    /// The compiled MuJoCo model.
    pub fn model(&self) -> &'a MjModel {
        self.data.model()
    }

    /// The scene bodies, in MuJoCo body order (world body excluded).
    pub fn bodies(&self) -> &'a [MjBody] {
        self.bodies
    }
}

/// Where one model state sits in the scene's flat model-state vector and in
/// the diffusion store.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct StateSlot {
    pub(crate) offset: usize,
    pub(crate) len: usize,
    pub(crate) noise_sources: usize,
    pub(crate) diffusion_offset: usize,
}

/// What a task model sees during one dynamics evaluation.
pub struct TaskContext<'a> {
    pub(crate) current_sim_nanos: u64,
    pub(crate) current_epoch: Epoch,
    pub(crate) scene: SceneView<'a>,
    pub(crate) states: &'a [f64],
    pub(crate) derivatives: &'a mut [f64],
    pub(crate) diffusions: &'a mut [f64],
    pub(crate) slots: &'a [StateSlot],
    pub(crate) base_offset: usize,
    pub(crate) base_diffusion_offset: usize,
}

impl TaskContext<'_> {
    /// Simulation time of the evaluation \[ns\] (upstream `CurrentSimNanos`).
    pub fn current_sim_nanos(&self) -> u64 {
        self.current_sim_nanos
    }

    /// Epoch of the evaluation.
    pub fn current_epoch(&self) -> Epoch {
        self.current_epoch
    }

    /// The simulation context of the evaluation, for code shared with a
    /// [`Module`] update.
    pub fn simulation_context(&self) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: self.current_sim_nanos,
            current_epoch: self.current_epoch,
        }
    }

    /// The scene at the evaluation.
    pub fn scene(&self) -> SceneView<'_> {
        self.scene
    }

    fn slot(&self, state: usize) -> &StateSlot {
        self.slots.get(state).unwrap_or_else(|| {
            panic!(
                "task model has {} states; state {state} does not exist",
                self.slots.len()
            )
        })
    }

    /// Value of the model's state `state` (in [`SceneModel::state_specs`]
    /// order) at the evaluation (upstream `stateView`).
    pub fn state(&self, state: usize) -> &[f64] {
        let slot = self.slot(state);
        let start = slot.offset - self.base_offset;
        &self.states[start..start + slot.len]
    }

    /// Upstream `setDerivative`.
    ///
    /// # Panics
    ///
    /// If `value` does not have the state's size.
    pub fn set_derivative(&mut self, state: usize, value: &[f64]) {
        let slot = self.slot(state).clone();
        assert_eq!(
            value.len(),
            slot.len,
            "derivative of task-model state {state} must have {} entries",
            slot.len
        );
        let start = slot.offset - self.base_offset;
        self.derivatives[start..start + slot.len].copy_from_slice(value);
    }

    /// Upstream `setDiffusion(value, source)`: the column of noise source
    /// `source` of the state.
    ///
    /// # Panics
    ///
    /// If the state has no such noise source or `value` does not have the
    /// state's size.
    pub fn set_diffusion(&mut self, state: usize, source: usize, value: &[f64]) {
        let slot = self.slot(state).clone();
        assert!(
            source < slot.noise_sources,
            "task-model state {state} has {} noise sources; source {source} does not exist",
            slot.noise_sources
        );
        assert_eq!(
            value.len(),
            slot.len,
            "diffusion of task-model state {state} must have {} entries",
            slot.len
        );
        let start = slot.diffusion_offset - self.base_diffusion_offset + source * slot.len;
        self.diffusions[start..start + slot.len].copy_from_slice(value);
    }
}

/// A model in a scene's dynamics (or diffusion dynamics) task.
///
/// The scene calls [`scene_update`](Self::scene_update) once per integrator stage, at
/// times that need not be ordered or evenly spaced, so a model's outputs
/// should depend only on its inputs, the time and its states (upstream's
/// "memoryless" warning on `AddModelToDynamicsTask`).
pub trait SceneModel: AsAny + Send + 'static {
    /// Upstream `ModelTag`.
    fn model_tag(&self) -> &str;

    /// Upstream `StatefulSysModel::registerStates`: the model's continuous
    /// states, read on every scene reset. The default is none.
    fn state_specs(&self) -> Vec<ModelStateSpec> {
        Vec::new()
    }

    /// The committed values of the states, in [`state_specs`](Self::state_specs)
    /// order, after every scene reset and step.
    fn load_states(&mut self, _states: &[&[f64]]) {}

    /// Upstream `Reset`, at every scene reset.
    fn scene_reset(&mut self, _scene: &SceneView<'_>, _context: &SimulationContext) {}

    /// Upstream `UpdateState`, at every evaluation of the task.
    fn scene_update(&mut self, context: &mut TaskContext<'_>);
}

/// A [`Module`] run in a scene task: its `init` at the first scene reset,
/// `reset` at later ones, and `update` at every evaluation.
#[derive(Debug)]
pub struct ModuleTaskModel<M> {
    model_tag: String,
    initialized: bool,
    /// The module.
    pub module: M,
}

impl<M> ModuleTaskModel<M> {
    pub fn new(model_tag: impl Into<String>, module: M) -> Self {
        Self {
            model_tag: model_tag.into(),
            initialized: false,
            module,
        }
    }
}

impl<M: Module + 'static> SceneModel for ModuleTaskModel<M> {
    fn model_tag(&self) -> &str {
        &self.model_tag
    }

    fn scene_reset(&mut self, _scene: &SceneView<'_>, context: &SimulationContext) {
        if self.initialized {
            self.module.reset(context);
        } else {
            self.module.init();
            self.initialized = true;
        }
    }

    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        self.module.update(&context.simulation_context());
    }
}
