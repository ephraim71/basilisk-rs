//! A multibody system solved by MuJoCo: port of upstream Basilisk `MJScene`
//! (with `MJSpec`, `MJEquality` and `MJFwdKinematics`) from
//! `simulation/mujocoDynamics/_GeneralModuleFiles`.
//!
//! An [`MjScene`] is a dynamic object, upstream's alternative to a
//! `Spacecraft`: bodies connected by joints, described by a MuJoCo XML
//! model, whose joint positions and velocities, body masses, actuator
//! activations and task-model states are integrated by any basilisk-rs
//! [`Integrator`], stochastic ones included. Each [`Module::update`] advances
//! the scene from the previous update to the current time in one integrator
//! step, so the scene's task period is its integration step.
//!
//! Every evaluation of the equations of motion copies the stage state into
//! MuJoCo, runs the **dynamics task** (models that set actuator inputs or
//! compute forces from the current state, see [`SceneModel`]), writes the
//! actuator controls and joint constraints, and asks MuJoCo
//! (`mj_fwdActuation`, `mj_fwdAcceleration`, `mj_fwdConstraint`) for the joint
//! accelerations. The task starts with a forward-kinematics pass at priority
//! [`FWD_KINEMATICS_PRIORITY`](MjScene::FWD_KINEMATICS_PRIORITY), which also
//! publishes every site's frame state, so later models read the stage's
//! kinematics. A separate **diffusion task** runs for the diffusion
//! evaluations of stochastic integrators.
//!
//! MuJoCo's own gravity is switched off; forces come from task models (for
//! example [`NBodyGravity`](super::n_body_gravity::NBodyGravity)) through
//! actuators.
//!
//! Differences from upstream:
//!
//! - Upstream edits a live `mjSpec`; here the scene keeps the XML and the
//!   edits made to it (new sites, actuators, equality settings) and replays
//!   them on every compilation, because mujoco-rs's specification is not
//!   `Send` (see [`spec`](super::spec)).
//! - The integrated state ([`MjSceneState`]) exists from construction, so
//!   initial joint states, masses and activations can be set before the
//!   first reset, which keeps them, as upstream keeps the states it already
//!   registered across later resets. Task-model states start from their
//!   [`ModelStateSpec`](super::ModelStateSpec) initial values on every reset,
//!   as upstream's `registerStates` sets them.
//! - Upstream holds a constrained scalar joint with a single-joint equality,
//!   `q - qpos0 = eq_data[0]`, writing the commanded value into `eq_data`,
//!   which mujoco-rs only exposes through `unsafe`. Here each scalar joint
//!   gets a hidden fixed tendon of unit coefficient on it,
//!   `_basilisk_constrainedTendon_<joint>`, and the constrained equality
//!   `_basilisk_constrainedEquality_<joint>` is a tendon equality,
//!   `L - tendon_length0 = 0` with `L = q`, the same constraint row (unit
//!   Jacobian, the joint's inverse weight). The commanded value moves the
//!   tendon's reference length, `tendon_length0 = qpos0 + value`, which
//!   nothing else at run time reads.
//! - Upstream evaluates the dynamics task at every stage of a zero-length
//!   step (an update at the reset time); here such an update evaluates the
//!   equations of motion once, which writes the same task outputs.
//! - Task models read the scene but cannot write joint states directly;
//!   upstream's direct `StateData` writes from inside the task have no
//!   counterpart.

use std::marker::PhantomData;
use std::ops::{Deref, DerefMut, Range};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use hifitime::{Duration, Epoch};
use mujoco_rs::prelude::*;
use nalgebra::{DVector, Matrix3, Vector3};

use super::actuator::{
    ActuatorObject, MjActuator, MjForceActuator, MjForceTorqueActuator, MjSingleActuator,
    MjTorqueActuator,
};
use super::body::{
    JointSlot, MjBallJoint, MjBody, MjFreeJoint, MjJointKind, MjScalarJoint, MjSite,
};
use super::spec::{
    ActuatorTarget, CompositeKind, MjSceneError, SceneSpec, SpecEdit, read_contents,
};
use super::state::{MjSceneDerivative, MjSceneState, QposPolicy};
use super::task::{
    MjSceneData, ModelStateSpec, ModuleTaskModel, SceneModel, SceneView, StateSlot, TaskContext,
};
use crate::integrators::{DynamicObject, Integrator, Rk4};
use crate::messages::{
    Input, MjSceneStateMsg, Output, ScalarJointStateMsg, SpacecraftMassPropsMsg, SpacecraftStateMsg,
};
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_ep};
use crate::{Module, SimulationContext};

/// Mass changes below this are ignored (upstream `massEpsilon`).
const MASS_EPSILON: f64 = 10.0 * f64::EPSILON;

/// Shared access to a scene's MuJoCo data, for modules outside its tasks
/// that read it (upstream modules hold an `MJScene*` and call
/// `getMujocoData`), such as [`MjSystemCom`](super::mj_system_com::MjSystemCom).
///
/// The scene locks the data while it updates; a module running in the
/// simulation's own schedule sees the data of the scene's last update. Do not
/// lock it from inside the scene's tasks, which run while the scene holds
/// the lock: models there read the data from their
/// [`TaskContext`](super::TaskContext), which is why the analysis modules
/// are also [`SceneModel`]s, to add with
/// [`add_model_to_dynamics_task`](MjScene::add_model_to_dynamics_task)
/// rather than [`add_module_to_dynamics_task`](MjScene::add_module_to_dynamics_task).
#[derive(Clone)]
pub struct MjSceneHandle {
    data: Arc<Mutex<MjSceneData>>,
}

impl MjSceneHandle {
    /// Locks the MuJoCo data.
    pub fn lock(&self) -> MutexGuard<'_, MjSceneData> {
        self.data.lock().unwrap_or_else(PoisonError::into_inner)
    }
}

impl std::fmt::Debug for MjSceneHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MjSceneHandle").finish_non_exhaustive()
    }
}

/// An equality constraint of the scene (upstream `MJEquality`). Its active
/// flag and solver parameters are set through the scene.
#[derive(Clone, Debug)]
pub struct MjEquality {
    name: String,
    id: usize,
}

impl MjEquality {
    /// Upstream `getName`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// MuJoCo equality id.
    pub fn id(&self) -> usize {
        self.id
    }
}

/// Geometry of one geom attached to a scene body (upstream `MJGeomInfo`).
#[derive(Clone, Debug, PartialEq)]
pub struct MjGeomInfo {
    /// Name of the body the geom is attached to.
    pub body_name: String,
    /// MuJoCo geom type (`mjtGeom`: 2 sphere, 3 capsule, 5 cylinder, 6
    /// box, 7 mesh...).
    pub geom_type: i32,
    /// Geom size parameters \[m\], as MuJoCo's `geom_size`.
    pub size: [f64; 3],
    /// Position relative to the body frame \[m\].
    pub pos: [f64; 3],
    /// Scalar-first orientation relative to the body frame.
    pub quat: [f64; 4],
    /// Color [-]: the geom's RGBA, or its material's when the geom keeps the
    /// default gray (MuJoCo's renderer precedence).
    pub rgba: [f64; 4],
}

/// Typed handle to a model added to a scene task, for reading or adjusting
/// it later through [`MjScene::task_model`] and [`MjScene::task_model_mut`].
pub struct TaskModelId<M> {
    key: usize,
    marker: PhantomData<fn() -> M>,
}

impl<M> Clone for TaskModelId<M> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<M> Copy for TaskModelId<M> {}

impl<M> std::fmt::Debug for TaskModelId<M> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TaskModelId")
            .field("key", &self.key)
            .finish()
    }
}

enum TaskKind {
    /// Upstream `MJFwdKinematics`.
    FwdKinematics,
    Model(Box<dyn SceneModel>),
}

struct TaskEntry {
    key: usize,
    priority: i32,
    kind: TaskKind,
    /// The model's states, set at every reset.
    slots: Vec<StateSlot>,
    range: Range<usize>,
    diffusion_range: Range<usize>,
    /// For a dynamics-task model that also runs in the diffusion task
    /// ([`MjScene::add_model_to_dynamics_and_diffusion_tasks`]), its
    /// priority there.
    diffusion_priority: Option<i32>,
}

/// The state dimensions and joint kinds a recompiled model must keep once
/// the scene has been reset (upstream `MujocoTopology`).
#[derive(Clone, Debug, PartialEq)]
struct Topology {
    nq: usize,
    nv: usize,
    na: usize,
    nbody: usize,
    joints: Vec<MjJointKind>,
}

impl Topology {
    fn capture(model: &MjModel) -> Self {
        Self {
            nq: model.ffi().nq as usize,
            nv: model.ffi().nv as usize,
            na: model.ffi().na as usize,
            nbody: model.ffi().nbody as usize,
            joints: model
                .jnt_type()
                .iter()
                .map(|kind| joint_kind(*kind))
                .collect(),
        }
    }
}

fn joint_kind(kind: MjtJoint) -> MjJointKind {
    match kind {
        MjtJoint::mjJNT_FREE => MjJointKind::Free,
        MjtJoint::mjJNT_BALL => MjJointKind::Ball,
        MjtJoint::mjJNT_SLIDE => MjJointKind::Slide,
        _ => MjJointKind::Hinge,
    }
}

/// A MuJoCo multibody system integrated as a basilisk-rs dynamic object
/// (upstream `MJScene`).
pub struct MjScene {
    /// Upstream `ModelTag`.
    pub name: String,
    /// Upstream `extraEoMCall`: evaluate the equations of motion once more
    /// with the final state of every step, so task-model outputs and the
    /// recorded accelerations match it. It does not change the dynamics.
    pub extra_eom_call: bool,
    /// Upstream `stateOutMsg`: `qpos`, `qvel` and `act` after every update.
    pub state_out_msg: Output<MjSceneStateMsg>,
    spec: SceneSpec,
    needs_recompile: bool,
    handle: MjSceneHandle,
    bodies: Vec<MjBody>,
    actuators: Vec<MjActuator>,
    equalities: Vec<MjEquality>,
    policy: QposPolicy,
    /// The reference lengths last given to the constrained-joint tendons,
    /// `(tendon id, length)`, reapplied after `mj_setConst` recomputes them.
    constrained_targets: Vec<(usize, f64)>,
    topology: Option<Topology>,
    dynamics_task: Vec<TaskEntry>,
    diffusion_task: Vec<TaskEntry>,
    next_task_key: usize,
    integrator: Option<Box<dyn Integrator<MjScene>>>,
    state: MjSceneState,
    model_derivatives: DVector<f64>,
    model_diffusions: Vec<f64>,
    mass_derivative: DVector<f64>,
    model_const_stale: bool,
    kinematics_stale: bool,
    first_dynamics_call: bool,
    time_before_nanos: u64,
}

impl std::fmt::Debug for MjScene {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MjScene")
            .field("name", &self.name)
            .field("bodies", &self.body_names())
            .field("state", &self.state)
            .finish_non_exhaustive()
    }
}

impl MjScene {
    /// Priority of the default forward-kinematics model of the dynamics task
    /// (upstream `FWD_KINEMATICS_PRIORITY`); higher priorities run first.
    pub const FWD_KINEMATICS_PRIORITY: i32 = 10_000;

    /// Upstream `MJScene(xmlString)`: a scene from a MuJoCo XML model.
    pub fn new(xml: &str) -> Result<Self, MjSceneError> {
        Self::with_files(xml, Vec::new())
    }

    /// Upstream `MJScene(xmlString, files)`: a scene whose XML references
    /// asset files (meshes, textures), loaded into MuJoCo's virtual file
    /// system.
    pub fn with_files(xml: &str, files: Vec<PathBuf>) -> Result<Self, MjSceneError> {
        let mut spec = SceneSpec::new(xml, files);
        let contents = read_contents(&spec.build()?);
        let model = spec.compile()?;

        // Bodies in MuJoCo id order, each with its `_com` site at the center
        // of mass and its `_origin` site (upstream `MJBody`'s constructor
        // and `configure`), and one constrained equality per scalar joint.
        let mut bodies = Vec::new();
        let mut equality_names = contents.equalities.clone();
        for id in 1..model.ffi().nbody as usize {
            let name = model
                .id_to_name(MjtObjBase::mjOBJ_BODY, id)
                .expect("every scene body is named")
                .to_string();
            let spec_body = contents
                .bodies
                .iter()
                .find(|body| body.name == name)
                .expect("compiled body is in the specification");
            let mut sites: Vec<MjSite> = spec_body
                .sites
                .iter()
                .map(|site| MjSite::new(site.clone()))
                .collect();
            let com = format!("{name}_com");
            let com_position = model.body_ipos()[id];
            if spec_body.sites.contains(&com) {
                spec.push(SpecEdit::SitePose {
                    site: com,
                    pos: Some(com_position),
                    quat: None,
                });
            } else {
                spec.push(SpecEdit::AddSite {
                    body: name.clone(),
                    name: com.clone(),
                    pos: com_position,
                    quat: [1.0, 0.0, 0.0, 0.0],
                });
                sites.push(MjSite::new(com));
            }
            let origin = format!("{name}_origin");
            if !spec_body.sites.contains(&origin) {
                spec.push(SpecEdit::AddSite {
                    body: name.clone(),
                    name: origin.clone(),
                    pos: [0.0; 3],
                    quat: [1.0, 0.0, 0.0, 0.0],
                });
                sites.push(MjSite::new(origin));
            }

            let mut body = MjBody {
                name: name.clone(),
                id,
                sites,
                scalar_joints: Vec::new(),
                ball_joint: None,
                free_joint: None,
                mass_properties_out_msg: Output::default(),
                derivative_mass_properties_in_msg: Input::default(),
            };
            for (joint, kind) in &spec_body.joints {
                match joint_kind(*kind) {
                    MjJointKind::Free => {
                        body.free_joint = Some(MjFreeJoint {
                            name: joint.clone(),
                            id: 0,
                            qpos_adr: 0,
                            qvel_adr: 0,
                        })
                    }
                    MjJointKind::Ball => {
                        body.ball_joint = Some(MjBallJoint {
                            name: joint.clone(),
                            id: 0,
                            qpos_adr: 0,
                            qvel_adr: 0,
                        })
                    }
                    scalar => {
                        let equality = format!("_basilisk_constrainedEquality_{joint}");
                        let tendon = format!("_basilisk_constrainedTendon_{joint}");
                        spec.push(SpecEdit::AddJointEquality {
                            name: equality.clone(),
                            tendon: tendon.clone(),
                            joint: joint.clone(),
                        });
                        equality_names.push(equality.clone());
                        body.scalar_joints.push(MjScalarJoint {
                            name: joint.clone(),
                            kind: scalar,
                            id: 0,
                            qpos_adr: 0,
                            qvel_adr: 0,
                            axis: [0.0; 3],
                            constrained_equality: equality,
                            constrained_tendon: tendon,
                            constrained_tendon_id: 0,
                            state_out_msg: Output::default(),
                            state_dot_out_msg: Output::default(),
                            constrained_state_in_msg: Input::default(),
                        });
                    }
                }
            }
            bodies.push(body);
        }

        // Upstream `loadActuators`: the composite actuators declared in the
        // custom text reuse existing `<name>_fx`... actuators or create them
        // at the hinted site; every other actuator is a single actuator.
        let mut unused: Vec<String> = contents.actuators.clone();
        let mut actuators = Vec::new();
        for (kind, name, site) in &contents.composite_actuators {
            actuators.push(composite_actuator(
                &mut spec,
                *kind,
                name,
                site,
                &mut unused,
            )?);
        }
        for name in unused {
            actuators.push(MjActuator::single(name.clone(), name));
        }

        let equalities = equality_names
            .into_iter()
            .map(|name| MjEquality { name, id: 0 })
            .collect();

        let model = spec.compile()?;
        let data = MjData::new(Box::new(model));
        let nbody = data.model().ffi().nbody as usize;
        let state = MjSceneState {
            qpos: DVector::from_column_slice(data.qpos()),
            qvel: DVector::from_column_slice(data.qvel()),
            act: DVector::from_column_slice(data.act()),
            mass: DVector::from_column_slice(data.model().body_mass()),
            models: DVector::zeros(0),
        };

        let mut scene = Self {
            name: "MJScene".to_string(),
            extra_eom_call: false,
            state_out_msg: Output::default(),
            spec,
            needs_recompile: false,
            handle: MjSceneHandle {
                data: Arc::new(Mutex::new(data)),
            },
            bodies,
            actuators,
            equalities,
            policy: QposPolicy::default(),
            constrained_targets: Vec::new(),
            topology: None,
            dynamics_task: Vec::new(),
            diffusion_task: Vec::new(),
            next_task_key: 0,
            integrator: None,
            state,
            model_derivatives: DVector::zeros(0),
            model_diffusions: Vec::new(),
            mass_derivative: DVector::zeros(nbody),
            model_const_stale: false,
            kinematics_stale: true,
            first_dynamics_call: true,
            time_before_nanos: 0,
        };
        let handle = scene.handle.clone();
        scene.configure(&handle.lock())?;
        scene.add_fwd_kinematics_to_dynamics_task(Self::FWD_KINEMATICS_PRIORITY);
        Ok(scene)
    }

    /// Upstream `MJScene::fromFile`.
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self, MjSceneError> {
        let path = path.as_ref();
        let xml = std::fs::read_to_string(path)
            .map_err(|error| MjSceneError::File(format!("{}: {error}", path.display())))?;
        Self::new(&xml)
    }

    /// Shared access to the MuJoCo data for modules outside the scene's
    /// tasks.
    pub fn handle(&self) -> MjSceneHandle {
        self.handle.clone()
    }

    /// Upstream `setIntegrator`. The default is [`Rk4`], as upstream.
    pub fn set_integrator(&mut self, integrator: impl Integrator<MjScene> + 'static) {
        self.integrator = Some(Box::new(integrator));
    }

    /// [`set_integrator`](Self::set_integrator) for an integrator chosen at
    /// run time.
    pub fn set_boxed_integrator(&mut self, integrator: Box<dyn Integrator<MjScene>>) {
        self.integrator = Some(integrator);
    }

    /// Upstream `setHighOrderAttitudeIntegration`. By default each attitude
    /// quaternion advances by one exponential map of the stage-averaged body
    /// rate (MuJoCo's own update, second-order accurate on SO(3)); with this
    /// set, quaternions integrate their rate `½ q ⊗ (0, ω)` at every stage and
    /// are normalized, so the attitude has the integrator's full order. The
    /// result then no longer bit-matches MuJoCo's stepper.
    ///
    /// # Panics
    ///
    /// If the mode changes after the first reset (upstream: "attitude
    /// integration mode cannot change" once the state topology is final).
    pub fn set_high_order_attitude_integration(&mut self, enabled: bool) {
        if enabled == self.policy.high_order {
            return;
        }
        assert!(
            self.topology.is_none(),
            "Changing MuJoCo attitude integration mode cannot change finalized state topology \
             (attitude integration mode cannot change after Reset)"
        );
        self.policy.high_order = enabled;
    }

    /// Upstream `getHighOrderAttitudeIntegration`.
    pub fn high_order_attitude_integration(&self) -> bool {
        self.policy.high_order
    }

    // -----------------------------------------------------------------------
    // Objects
    // -----------------------------------------------------------------------

    /// Upstream `getBodyNames`, in MuJoCo body order.
    pub fn body_names(&self) -> Vec<String> {
        self.bodies.iter().map(|body| body.name.clone()).collect()
    }

    /// Upstream `getBodyParentName`: `"world"` for a body attached to the
    /// world body.
    pub fn body_parent_name(&self, body: &str) -> Result<String, MjSceneError> {
        let data = self.handle.lock();
        let model = data.model();
        let id = model
            .name_to_id(MjtObjBase::mjOBJ_BODY, body)
            .ok_or_else(|| MjSceneError::NotFound {
                kind: "body",
                name: body.to_string(),
            })?;
        let parent = model.body_parentid()[id] as usize;
        Ok(if parent == 0 {
            "world".to_string()
        } else {
            model
                .id_to_name(MjtObjBase::mjOBJ_BODY, parent)
                .unwrap_or_default()
                .to_string()
        })
    }

    /// Upstream `getGeomInfos`: the geoms of every body other than the world
    /// body.
    pub fn geom_infos(&self) -> Vec<MjGeomInfo> {
        const DEFAULT_RGBA: [f32; 4] = [0.5, 0.5, 0.5, 1.0];
        let data = self.handle.lock();
        let model = data.model();
        let mut geoms = Vec::new();
        for geom in 0..model.ffi().ngeom as usize {
            let body = model.geom_bodyid()[geom] as usize;
            if body == 0 {
                continue;
            }
            let mut rgba = model.geom_rgba()[geom];
            let material = model.geom_matid()[geom];
            if material >= 0 && rgba == DEFAULT_RGBA {
                rgba = model.mat_rgba()[material as usize];
            }
            geoms.push(MjGeomInfo {
                body_name: model
                    .id_to_name(MjtObjBase::mjOBJ_BODY, body)
                    .unwrap_or_default()
                    .to_string(),
                geom_type: model.geom_type()[geom] as i32,
                size: model.geom_size()[geom],
                pos: model.geom_pos()[geom],
                quat: model.geom_quat()[geom],
                rgba: rgba.map(f64::from),
            });
        }
        geoms
    }

    /// The scene bodies, in MuJoCo body order.
    pub fn bodies(&self) -> &[MjBody] {
        &self.bodies
    }

    fn body_index(&self, name: &str) -> usize {
        self.bodies
            .iter()
            .position(|body| body.name == name)
            .unwrap_or_else(|| panic!("Unknown body '{name}' in MJScene"))
    }

    /// Upstream `getBody`, with read access to the body's states.
    ///
    /// # Panics
    ///
    /// If the scene has no such body (upstream raises `BasiliskError`).
    pub fn body(&self, name: &str) -> BodyRef<'_> {
        BodyRef {
            body: &self.bodies[self.body_index(name)],
            state: &self.state,
        }
    }

    /// Upstream `getBody`, with its state setters and mutable ports.
    ///
    /// # Panics
    ///
    /// If the scene has no such body.
    pub fn body_mut(&mut self, name: &str) -> BodyMut<'_> {
        let index = self.body_index(name);
        BodyMut {
            body: &mut self.bodies[index],
            state: &mut self.state,
            kinematics_stale: &mut self.kinematics_stale,
        }
    }

    /// Upstream `getSite`.
    ///
    /// # Panics
    ///
    /// If no body has such a site.
    pub fn site(&self, name: &str) -> &MjSite {
        self.bodies
            .iter()
            .flat_map(|body| body.sites.iter())
            .find(|site| site.name == name)
            .unwrap_or_else(|| panic!("Unknown site '{name}' in MJScene"))
    }

    fn has_site(&self, name: &str) -> bool {
        self.bodies.iter().any(|body| body.has_site(name))
    }

    /// The scalar joint named `name`, in whichever body.
    ///
    /// # Panics
    ///
    /// If no body has such a scalar joint.
    pub fn scalar_joint(&self, name: &str) -> &MjScalarJoint {
        self.bodies
            .iter()
            .flat_map(|body| body.scalar_joints.iter())
            .find(|joint| joint.name == name)
            .unwrap_or_else(|| panic!("Unknown scalar joint '{name}' in MJScene"))
    }

    /// Mutable form of [`scalar_joint`](Self::scalar_joint), to connect its
    /// constrained-state input.
    pub fn scalar_joint_mut(&mut self, name: &str) -> &mut MjScalarJoint {
        self.bodies
            .iter_mut()
            .flat_map(|body| body.scalar_joints.iter_mut())
            .find(|joint| joint.name == name)
            .unwrap_or_else(|| panic!("Unknown scalar joint '{name}' in MJScene"))
    }

    /// Upstream `getEquality`.
    ///
    /// # Panics
    ///
    /// If the scene has no such equality.
    pub fn equality(&self, name: &str) -> &MjEquality {
        self.equalities
            .iter()
            .find(|equality| equality.name == name)
            .unwrap_or_else(|| panic!("Unknown equality '{name}' in MJScene"))
    }

    /// Upstream `MJEquality::setActive`.
    pub fn set_equality_active(&mut self, name: &str, active: bool) {
        let id = self.equality(name).id;
        self.handle.lock().eq_active_mut()[id] = active;
    }

    /// Upstream `MJEquality::isActive`.
    pub fn is_equality_active(&self, name: &str) -> bool {
        let id = self.equality(name).id;
        self.handle.lock().eq_active()[id]
    }

    /// Upstream `MJEquality::setSolref` (applied at the next compilation).
    pub fn set_equality_solref(&mut self, name: &str, time_constant: f64, damping_ratio: f64) {
        let name = self.equality(name).name.clone();
        self.push_edit(SpecEdit::EqualitySolref {
            name,
            solref: [time_constant, damping_ratio],
        });
    }

    /// Upstream `MJEquality::setSolimp` (applied at the next compilation).
    pub fn set_equality_solimp(
        &mut self,
        name: &str,
        d0: f64,
        dwidth: f64,
        width: f64,
        midpoint: f64,
        power: f64,
    ) {
        let name = self.equality(name).name.clone();
        self.push_edit(SpecEdit::EqualitySolimp {
            name,
            solimp: [d0, dwidth, width, midpoint, power],
        });
    }

    fn actuator(&self, name: &str) -> &MjActuator {
        self.actuators
            .iter()
            .find(|actuator| actuator.name() == name)
            .unwrap_or_else(|| panic!("Tried to get actuator '{name}' but it does not exist."))
    }

    fn actuator_mut(&mut self, name: &str) -> &mut MjActuator {
        self.actuators
            .iter_mut()
            .find(|actuator| actuator.name() == name)
            .unwrap_or_else(|| panic!("Tried to get actuator '{name}' but it does not exist."))
    }

    fn has_actuator(&self, name: &str) -> bool {
        self.actuators
            .iter()
            .any(|actuator| actuator.name() == name)
    }

    /// Names of every scene actuator.
    pub fn actuator_names(&self) -> Vec<String> {
        self.actuators
            .iter()
            .map(|actuator| actuator.name().to_string())
            .collect()
    }
}

/// Upstream's typed actuator getters (`getSingleActuator`...).
macro_rules! actuator_getters {
    ($($get:ident, $get_mut:ident, $variant:ident, $kind:ident;)*) => {
        impl MjScene {
            $(
                #[doc = concat!("Upstream `get", stringify!($kind), "`.\n\n# Panics\n\n",
                    "If the scene has no such actuator, or it is of another kind.")]
                pub fn $get(&self, name: &str) -> &$kind {
                    match self.actuator(name) {
                        MjActuator::$variant(actuator) => actuator,
                        _ => panic!(
                            "Tried to get actuator '{name}', but it is not of the expected type."
                        ),
                    }
                }

                #[doc = concat!("Mutable form of [`", stringify!($get), "`](Self::", stringify!($get), "), to connect its inputs.")]
                pub fn $get_mut(&mut self, name: &str) -> &mut $kind {
                    match self.actuator_mut(name) {
                        MjActuator::$variant(actuator) => actuator,
                        _ => panic!(
                            "Tried to get actuator '{name}', but it is not of the expected type."
                        ),
                    }
                }
            )*
        }
    };
}

actuator_getters! {
    single_actuator, single_actuator_mut, Single, MjSingleActuator;
    force_actuator, force_actuator_mut, Force, MjForceActuator;
    torque_actuator, torque_actuator_mut, Torque, MjTorqueActuator;
    force_torque_actuator, force_torque_actuator_mut, ForceTorque, MjForceTorqueActuator;
}

/// Upstream `MJSpec::createActuator`: the sub-actuators of a composite
/// actuator, reusing the existing ones in `unused` and creating the rest at
/// `site`.
fn composite_actuator(
    spec: &mut SceneSpec,
    kind: CompositeKind,
    name: &str,
    site: &str,
    unused: &mut Vec<String>,
) -> Result<MjActuator, MjSceneError> {
    let mut objects = Vec::new();
    for (suffix, gear_index) in kind.sub_actuators() {
        let sub = format!("{name}{suffix}");
        if let Some(position) = unused.iter().position(|existing| *existing == sub) {
            unused.remove(position);
        } else if site.is_empty() {
            return Err(MjSceneError::NotFound {
                kind: "site",
                name: format!("(none given) to autogenerate mujoco actuator for actuator '{name}'"),
            });
        } else {
            let mut gear = [0.0; 6];
            gear[*gear_index] = 1.0;
            spec.push(SpecEdit::AddActuator {
                name: sub.clone(),
                target: ActuatorTarget::Site(site.to_string()),
                gear,
            });
        }
        objects.push(ActuatorObject::new(sub));
    }
    Ok(match kind {
        CompositeKind::Force => MjActuator::Force(MjForceActuator {
            name: name.to_string(),
            objects: objects.try_into().expect("three sub-actuators"),
            force_in_msg: Input::default(),
        }),
        CompositeKind::Torque => MjActuator::Torque(MjTorqueActuator {
            name: name.to_string(),
            objects: objects.try_into().expect("three sub-actuators"),
            torque_in_msg: Input::default(),
        }),
        CompositeKind::ForceTorque => MjActuator::ForceTorque(MjForceTorqueActuator {
            name: name.to_string(),
            objects: objects.try_into().expect("six sub-actuators"),
            force_in_msg: Input::default(),
            torque_in_msg: Input::default(),
        }),
    })
}

// ---------------------------------------------------------------------------
// Specification edits
// ---------------------------------------------------------------------------

impl MjScene {
    /// Upstream `markAsNeedingToRecompileModel`, with the edit to replay.
    fn push_edit(&mut self, edit: SpecEdit) {
        self.spec.push(edit);
        self.needs_recompile = true;
    }

    /// Upstream `MJBody::addSite`: a site fixed to `body` at `position` \[m\]
    /// with attitude `attitude` (MRP of the site frame relative to the body
    /// frame).
    ///
    /// # Panics
    ///
    /// If the body does not exist or already has such a site.
    pub fn add_site(
        &mut self,
        body: &str,
        name: &str,
        position: Vector3<f64>,
        attitude: Vector3<f64>,
    ) -> &MjSite {
        let index = self.body_index(body);
        assert!(
            !self.has_site(name),
            "Tried to create site '{name}' twice for body '{body}'"
        );
        let quat = mrp_to_ep(attitude);
        self.push_edit(SpecEdit::AddSite {
            body: body.to_string(),
            name: name.to_string(),
            pos: position.into(),
            quat: [quat[0], quat[1], quat[2], quat[3]],
        });
        let sites = &mut self.bodies[index].sites;
        sites.push(MjSite::new(name.to_string()));
        sites.last().expect("site just added")
    }

    /// Upstream `MJSite::setPositionRelativeToBody` \[m\].
    pub fn set_site_position(&mut self, site: &str, position: Vector3<f64>) {
        let site = self.site(site).name.clone();
        self.push_edit(SpecEdit::SitePose {
            site,
            pos: Some(position.into()),
            quat: None,
        });
    }

    /// Upstream `MJSite::setAttitudeRelativeToBody` (MRP of the site frame
    /// relative to the body frame).
    pub fn set_site_attitude(&mut self, site: &str, attitude: Vector3<f64>) {
        let site = self.site(site).name.clone();
        let quat = mrp_to_ep(attitude);
        self.push_edit(SpecEdit::SitePose {
            site,
            pos: None,
            quat: Some([quat[0], quat[1], quat[2], quat[3]]),
        });
    }

    fn check_new_actuator(&self, name: &str) {
        assert!(
            !self.has_actuator(name),
            "Tried to add actuator with name '{name}' but one already exists with that name."
        );
    }

    /// Upstream `addJointSingleActuator`: a motor on `joint` (unit gear).
    ///
    /// # Panics
    ///
    /// If an actuator with that name exists.
    pub fn add_joint_single_actuator(&mut self, name: &str, joint: &str) -> &mut MjSingleActuator {
        self.check_new_actuator(name);
        self.push_edit(SpecEdit::AddActuator {
            name: name.to_string(),
            target: ActuatorTarget::Joint(joint.to_string()),
            gear: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });
        self.actuators
            .push(MjActuator::single(name.to_string(), name.to_string()));
        self.single_actuator_mut(name)
    }

    /// Upstream `addSingleActuator`: one control mapped through `gear` to a
    /// force (first three entries) and torque (last three) in the frame of
    /// `site`.
    ///
    /// # Panics
    ///
    /// If an actuator with that name exists.
    pub fn add_single_actuator(
        &mut self,
        name: &str,
        site: &str,
        gear: [f64; 6],
    ) -> &mut MjSingleActuator {
        self.check_new_actuator(name);
        self.push_edit(SpecEdit::AddActuator {
            name: name.to_string(),
            target: ActuatorTarget::Site(site.to_string()),
            gear,
        });
        self.actuators
            .push(MjActuator::single(name.to_string(), name.to_string()));
        self.single_actuator_mut(name)
    }

    fn add_composite(&mut self, kind: CompositeKind, name: &str, site: &str) {
        self.check_new_actuator(name);
        let mut unused = Vec::new();
        let actuator = composite_actuator(&mut self.spec, kind, name, site, &mut unused)
            .expect("a site is given");
        self.needs_recompile = true;
        self.actuators.push(actuator);
    }

    /// Upstream `addForceActuator`: a force at `site`, in site-frame
    /// components.
    pub fn add_force_actuator(&mut self, name: &str, site: &str) -> &mut MjForceActuator {
        self.add_composite(CompositeKind::Force, name, site);
        self.force_actuator_mut(name)
    }

    /// Upstream `addTorqueActuator`: a torque at `site`, in site-frame
    /// components.
    pub fn add_torque_actuator(&mut self, name: &str, site: &str) -> &mut MjTorqueActuator {
        self.add_composite(CompositeKind::Torque, name, site);
        self.torque_actuator_mut(name)
    }

    /// Upstream `addForceTorqueActuator`.
    pub fn add_force_torque_actuator(
        &mut self,
        name: &str,
        site: &str,
    ) -> &mut MjForceTorqueActuator {
        self.add_composite(CompositeKind::ForceTorque, name, site);
        self.force_torque_actuator_mut(name)
    }

    /// Whether specification edits await compilation (upstream
    /// `hasPendingModelChanges`).
    pub fn has_pending_model_changes(&self) -> bool {
        self.needs_recompile
    }

    /// Upstream `recompileIfNeeded`: compiles the pending edits and swaps in
    /// the new model, carrying the runtime data over. On failure the scene
    /// keeps its current model and the pending edits, so a later fix (such
    /// as adding a missing site) recovers.
    ///
    /// # Errors
    ///
    /// If compilation fails, or after the first reset if the new model would
    /// change the state topology or reorder runtime objects.
    pub fn recompile(&mut self) -> Result<(), MjSceneError> {
        if !self.needs_recompile {
            return Ok(());
        }
        let candidate = self.spec.compile()?;
        let handle = self.handle.clone();
        let mut data = handle.lock();
        let current = data.model();
        let current_topology = Topology::capture(current);
        let candidate_topology = Topology::capture(&candidate);
        if candidate_topology != current_topology {
            return Err(MjSceneError::Topology(
                "MuJoCo recompilation would change finalized state or qpos-policy topology."
                    .to_string(),
            ));
        }
        for (object, count) in [
            (MjtObjBase::mjOBJ_BODY, current.ffi().nbody),
            (MjtObjBase::mjOBJ_JOINT, current.ffi().njnt),
            (MjtObjBase::mjOBJ_ACTUATOR, current.ffi().nu),
            (MjtObjBase::mjOBJ_EQUALITY, current.ffi().neq),
        ] {
            for id in 0..count as usize {
                if current.id_to_name(object, id) != candidate.id_to_name(object, id) {
                    return Err(MjSceneError::Topology(
                        "MuJoCo recompilation would reorder or remove runtime-state objects."
                            .to_string(),
                    ));
                }
            }
        }

        let mut next = MjData::new(Box::new(candidate));
        next.set_time(data.time());
        copy_prefix(next.qpos_mut(), data.qpos());
        copy_prefix(next.qvel_mut(), data.qvel());
        copy_prefix(next.act_mut(), data.act());
        copy_prefix(next.qacc_warmstart_mut(), data.qacc_warmstart());
        copy_prefix(next.qfrc_applied_mut(), data.qfrc_applied());
        copy_prefix(next.ctrl_mut(), data.ctrl());
        copy_prefix(next.eq_active_mut(), data.eq_active());
        copy_prefix(next.xfrc_applied_mut(), data.xfrc_applied());
        copy_prefix(next.mocap_pos_mut(), data.mocap_pos());
        copy_prefix(next.mocap_quat_mut(), data.mocap_quat());
        copy_prefix(next.userdata_mut(), data.userdata());
        *data = next;
        self.configure(&data)?;
        self.needs_recompile = false;
        self.kinematics_stale = true;
        Ok(())
    }

    fn recompile_or_panic(&mut self) {
        if let Err(error) = self.recompile() {
            panic!("MJScene '{}': {error}", self.name);
        }
    }

    /// Upstream `MJSpec::configure`: resolves the MuJoCo ids and addresses of
    /// every object from the compiled model.
    fn configure(&mut self, data: &MjSceneData) -> Result<(), MjSceneError> {
        let model = data.model();
        let id = |object: MjtObjBase, kind: &'static str, name: &str| {
            model
                .name_to_id(object, name)
                .ok_or_else(|| MjSceneError::NotFound {
                    kind,
                    name: name.to_string(),
                })
        };
        for body in &mut self.bodies {
            body.id = id(MjtObjBase::mjOBJ_BODY, "body", &body.name)?;
            for site in &mut body.sites {
                site.id = id(MjtObjBase::mjOBJ_SITE, "site", &site.name)?;
            }
            for joint in &mut body.scalar_joints {
                joint.id = id(MjtObjBase::mjOBJ_JOINT, "joint", &joint.name)?;
                joint.qpos_adr = model.jnt_qposadr()[joint.id] as usize;
                joint.qvel_adr = model.jnt_dofadr()[joint.id] as usize;
                joint.axis = model.jnt_axis()[joint.id];
                joint.constrained_tendon_id = id(
                    MjtObjBase::mjOBJ_TENDON,
                    "tendon",
                    &joint.constrained_tendon,
                )?;
            }
            if let Some(joint) = &mut body.ball_joint {
                joint.id = id(MjtObjBase::mjOBJ_JOINT, "joint", &joint.name)?;
                joint.qpos_adr = model.jnt_qposadr()[joint.id] as usize;
                joint.qvel_adr = model.jnt_dofadr()[joint.id] as usize;
            }
            if let Some(joint) = &mut body.free_joint {
                joint.id = id(MjtObjBase::mjOBJ_JOINT, "joint", &joint.name)?;
                joint.qpos_adr = model.jnt_qposadr()[joint.id] as usize;
                joint.qvel_adr = model.jnt_dofadr()[joint.id] as usize;
            }
        }
        for actuator in &mut self.actuators {
            for object in actuator.objects_mut() {
                object.id = id(MjtObjBase::mjOBJ_ACTUATOR, "actuator", &object.name)?;
            }
        }
        for equality in &mut self.equalities {
            equality.id = id(MjtObjBase::mjOBJ_EQUALITY, "equality", &equality.name)?;
        }
        self.policy.joints = (0..model.ffi().njnt as usize)
            .map(|joint| JointSlot {
                kind: joint_kind(model.jnt_type()[joint]),
                qpos_adr: model.jnt_qposadr()[joint] as usize,
                qvel_adr: model.jnt_dofadr()[joint] as usize,
            })
            .collect();
        self.constrained_targets.clear();
        Ok(())
    }

    /// Upstream `printMujocoModelDebugInfo`: MuJoCo's text dump of the
    /// compiled model (`mj_printModel`).
    ///
    /// # Errors
    ///
    /// If the file cannot be written.
    pub fn print_model_debug_info(&self, path: impl AsRef<Path>) -> Result<(), MjSceneError> {
        let path = path.as_ref();
        self.handle
            .lock()
            .model()
            .print(path)
            .map_err(|error| MjSceneError::Save(format!("'{}': {error}", path.display())))
    }

    /// Upstream `saveToFile`: writes the scene, pending edits and generated
    /// sites included, as MuJoCo XML, or as a binary model when `path` ends in
    /// `.mjb`.
    ///
    /// # Errors
    ///
    /// If the edits do not compile or the file cannot be written.
    pub fn save_to_file(&self, path: impl AsRef<Path>) -> Result<(), MjSceneError> {
        let path = path.as_ref();
        if path.extension().is_some_and(|extension| extension == "mjb") {
            return self
                .handle
                .lock()
                .model()
                .save_to_file(path)
                .map_err(|error| MjSceneError::Save(format!("'{}': {error}", path.display())));
        }
        self.spec.save_xml(path)
    }
}

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------

/// Which scene task.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Task {
    Dynamics,
    Diffusion,
}

impl MjScene {
    fn task(&mut self, task: Task) -> &mut Vec<TaskEntry> {
        match task {
            Task::Dynamics => &mut self.dynamics_task,
            Task::Diffusion => &mut self.diffusion_task,
        }
    }

    /// Inserts by priority, higher first, keeping insertion order among
    /// equal priorities (upstream `SysModelTask::AddNewObject`).
    fn insert_entry(&mut self, task: Task, priority: i32, kind: TaskKind) -> usize {
        if let TaskKind::Model(model) = &kind {
            assert!(
                self.topology.is_none() || model.state_specs().is_empty(),
                "MJScene '{}': a stateful model cannot be added after Reset, once the state \
                 topology is final",
                self.name
            );
        }
        let key = self.next_task_key;
        self.next_task_key += 1;
        let entries = self.task(task);
        let position = entries
            .iter()
            .position(|entry| entry.priority < priority)
            .unwrap_or(entries.len());
        entries.insert(
            position,
            TaskEntry {
                key,
                priority,
                kind,
                slots: Vec::new(),
                range: 0..0,
                diffusion_range: 0..0,
                diffusion_priority: None,
            },
        );
        key
    }

    /// Upstream `AddModelToDynamicsTask(model, priority)`: `model` runs at
    /// every dynamics evaluation, higher priorities first (upstream's default
    /// priority is -1).
    pub fn add_model_to_dynamics_task<M: SceneModel>(
        &mut self,
        model: M,
        priority: i32,
    ) -> TaskModelId<M> {
        let key = self.insert_entry(Task::Dynamics, priority, TaskKind::Model(Box::new(model)));
        TaskModelId {
            key,
            marker: PhantomData,
        }
    }

    /// Upstream `AddModelToDiffusionDynamicsTask(model, priority)`: `model`
    /// runs at every diffusion evaluation of a stochastic integrator.
    pub fn add_model_to_diffusion_dynamics_task<M: SceneModel>(
        &mut self,
        model: M,
        priority: i32,
    ) -> TaskModelId<M> {
        let key = self.insert_entry(Task::Diffusion, priority, TaskKind::Model(Box::new(model)));
        TaskModelId {
            key,
            marker: PhantomData,
        }
    }

    /// Upstream `AddModelToDynamicsTask(model, priority)` followed by
    /// `AddModelToDiffusionDynamicsTask(model, diffusion_priority)` on the
    /// same model, as upstream's stochastic scenarios add a
    /// `StatefulSysModel` whose diffusion depends on its state: the model
    /// runs at every dynamics evaluation and at every diffusion evaluation
    /// (with the diffusion task's models, by priority), and its states are
    /// registered once.
    pub fn add_model_to_dynamics_and_diffusion_tasks<M: SceneModel>(
        &mut self,
        model: M,
        priority: i32,
        diffusion_priority: i32,
    ) -> TaskModelId<M> {
        let id = self.add_model_to_dynamics_task(model, priority);
        self.entry_mut(id.key).diffusion_priority = Some(diffusion_priority);
        id
    }

    /// Runs a [`Module`] in the dynamics task (upstream adds any `SysModel`,
    /// such as an ephemeris, with `AddModelToDynamicsTask`).
    pub fn add_module_to_dynamics_task<M: Module + 'static>(
        &mut self,
        model_tag: &str,
        module: M,
        priority: i32,
    ) -> TaskModelId<ModuleTaskModel<M>> {
        self.add_model_to_dynamics_task(ModuleTaskModel::new(model_tag, module), priority)
    }

    /// Upstream `AddFwdKinematicsToDynamicsTask`: another forward-kinematics
    /// pass at `priority`. The scene's states only change between
    /// evaluations, so an extra pass refreshes nothing until a later
    /// evaluation; it is kept for upstream's task layouts.
    pub fn add_fwd_kinematics_to_dynamics_task(&mut self, priority: i32) {
        self.insert_entry(Task::Dynamics, priority, TaskKind::FwdKinematics);
    }

    /// Upstream `AddFwdKinematicsToDiffusionDynamicsTask`: the diffusion task
    /// has no forward-kinematics pass unless one is added.
    pub fn add_fwd_kinematics_to_diffusion_dynamics_task(&mut self, priority: i32) {
        self.insert_entry(Task::Diffusion, priority, TaskKind::FwdKinematics);
    }

    fn entry(&self, key: usize) -> &TaskEntry {
        self.dynamics_task
            .iter()
            .chain(self.diffusion_task.iter())
            .find(|entry| entry.key == key)
            .expect("task model id belongs to this scene")
    }

    fn entry_mut(&mut self, key: usize) -> &mut TaskEntry {
        self.dynamics_task
            .iter_mut()
            .chain(self.diffusion_task.iter_mut())
            .find(|entry| entry.key == key)
            .expect("task model id belongs to this scene")
    }

    /// The model `id` refers to.
    pub fn task_model<M: SceneModel>(&self, id: TaskModelId<M>) -> &M {
        match &self.entry(id.key).kind {
            TaskKind::Model(model) => model
                .as_any()
                .downcast_ref()
                .expect("task model id has the model's type"),
            TaskKind::FwdKinematics => unreachable!("ids refer to models"),
        }
    }

    /// Mutable form of [`task_model`](Self::task_model).
    pub fn task_model_mut<M: SceneModel>(&mut self, id: TaskModelId<M>) -> &mut M {
        match &mut self.entry_mut(id.key).kind {
            TaskKind::Model(model) => model
                .as_any_mut()
                .downcast_mut()
                .expect("task model id has the model's type"),
            TaskKind::FwdKinematics => unreachable!("ids refer to models"),
        }
    }

    /// The current value of state `state` of the model `id` refers to.
    ///
    /// # Panics
    ///
    /// Before the first reset, which lays out the model states.
    pub fn task_model_state<M: SceneModel>(&self, id: TaskModelId<M>, state: usize) -> &[f64] {
        let slot = &self.entry(id.key).slots[state];
        &self.state.models.as_slice()[slot.offset..slot.offset + slot.len]
    }

    /// Upstream `StateData::setState` on a task-model state, after the
    /// first reset (before it, the model's own initial value applies).
    ///
    /// # Panics
    ///
    /// If `value` does not have the state's size.
    pub fn set_task_model_state<M: SceneModel>(
        &mut self,
        id: TaskModelId<M>,
        state: usize,
        value: &[f64],
    ) {
        let slot = self.entry(id.key).slots[state].clone();
        assert_eq!(
            value.len(),
            slot.len,
            "task-model state {state} has {} entries",
            slot.len
        );
        self.state.models.as_mut_slice()[slot.offset..slot.offset + slot.len]
            .copy_from_slice(value);
        self.load_model_states();
    }

    /// Lays out the task-model states (dynamics task first, as upstream
    /// registers them) and sets their initial values.
    fn register_model_states(&mut self) {
        let mut offset = 0;
        let mut diffusion_offset = 0;
        let mut initial = Vec::new();
        for entry in self
            .dynamics_task
            .iter_mut()
            .chain(self.diffusion_task.iter_mut())
        {
            let specs: Vec<ModelStateSpec> = match &entry.kind {
                TaskKind::Model(model) => model.state_specs(),
                TaskKind::FwdKinematics => Vec::new(),
            };
            let start = offset;
            let diffusion_start = diffusion_offset;
            entry.slots = specs
                .iter()
                .map(|spec| {
                    let slot = StateSlot {
                        offset,
                        len: spec.initial.len(),
                        noise_sources: spec.noise_sources,
                        diffusion_offset,
                    };
                    offset += spec.initial.len();
                    diffusion_offset += spec.initial.len() * spec.noise_sources;
                    initial.extend_from_slice(&spec.initial);
                    slot
                })
                .collect();
            entry.range = start..offset;
            entry.diffusion_range = diffusion_start..diffusion_offset;
        }
        self.state.models = DVector::from_vec(initial);
        self.model_derivatives = DVector::zeros(offset);
        self.model_diffusions = vec![0.0; diffusion_offset];
    }

    /// Hands the committed task-model states to their models.
    fn load_model_states(&mut self) {
        let models = self.state.models.as_slice();
        for entry in self
            .dynamics_task
            .iter_mut()
            .chain(self.diffusion_task.iter_mut())
        {
            if let TaskKind::Model(model) = &mut entry.kind {
                if entry.slots.is_empty() {
                    continue;
                }
                let states: Vec<&[f64]> = entry
                    .slots
                    .iter()
                    .map(|slot| &models[slot.offset..slot.offset + slot.len])
                    .collect();
                model.load_states(&states);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// States
// ---------------------------------------------------------------------------

impl MjScene {
    /// The integrated state.
    pub fn state(&self) -> &MjSceneState {
        &self.state
    }

    /// Upstream `assembleFullQpos`.
    pub fn qpos(&self) -> &DVector<f64> {
        &self.state.qpos
    }

    /// Upstream `assembleFullQvel`.
    pub fn qvel(&self) -> &DVector<f64> {
        &self.state.qvel
    }

    /// Upstream `getActState`: the actuator activations (empty for a model
    /// without activation states).
    pub fn act(&self) -> &DVector<f64> {
        &self.state.act
    }

    /// Upstream `getActState()->setState`.
    pub fn set_act(&mut self, act: &[f64]) {
        assert_eq!(act.len(), self.state.act.len(), "act has na entries");
        self.state.act.copy_from_slice(act);
    }

    /// Upstream `getMassState`: one mass \[kg\] per MuJoCo body, the world
    /// body first.
    pub fn masses(&self) -> &DVector<f64> {
        &self.state.mass
    }

    /// Upstream `getMassState()->setState`. The masses are checked at the
    /// next reset or evaluation.
    pub fn set_masses(&mut self, masses: &[f64]) {
        assert_eq!(
            masses.len(),
            self.state.mass.len(),
            "masses has one entry per body, the world body included"
        );
        self.state.mass.copy_from_slice(masses);
    }
}

/// Upstream `validateMujocoMassStates`.
fn validate_masses(masses: &DVector<f64>, model: &MjModel) {
    assert!(
        masses.len() == model.ffi().nbody as usize && masses[0].is_finite() && masses[0] == 0.0,
        "The MuJoCo world-body mass state must remain finite and zero."
    );
    for id in 1..masses.len() {
        let (old, new) = (model.body_mass()[id], masses[id]);
        assert!(
            new.is_finite() && new >= 0.0,
            "MuJoCo body mass states must be finite and nonnegative."
        );
        assert!(
            !((old - new).abs() > MASS_EPSILON && (old <= MASS_EPSILON || new <= MASS_EPSILON)),
            "Runtime MuJoCo mass updates cannot transition a body to or from zero mass \
             because no reversible inertia scaling exists."
        );
    }
}

impl MjScene {
    /// Upstream `synchronizeMujocoFromStates`: applies the masses of `state`
    /// (rescaling each changed body's inertia with its mass, and refreshing
    /// MuJoCo's constants), then copies its joint states and activations.
    fn synchronize(&mut self, data: &mut MjSceneData, state: &MjSceneState) {
        validate_masses(&state.mass, data.model());

        let changed: Vec<usize> = (1..state.mass.len())
            .filter(|&id| (data.model().body_mass()[id] - state.mass[id]).abs() > MASS_EPSILON)
            .collect();
        if !changed.is_empty() {
            mutate_model(data, |model| {
                for &id in &changed {
                    // Upstream `applyPrevalidatedMass`: the ratio before the
                    // mass is overwritten, inertia linear in mass.
                    let ratio = state.mass[id] / model.body_mass()[id];
                    model.body_mass_mut()[id] = state.mass[id];
                    for value in &mut model.body_inertia_mut()[id] {
                        *value *= ratio;
                    }
                }
            });
            self.model_const_stale = true;
        }
        if self.model_const_stale {
            data.set_const();
            // `mj_setConst` recomputes the tendon reference lengths.
            set_tendon_references(data, &self.constrained_targets);
            self.model_const_stale = false;
        }
        // `mj_setConst` restores the reference pose, so the state goes in
        // after it.
        copy_state_into(data, state);
        self.kinematics_stale = true;
    }

    /// Upstream `MJScalarJoint::updateConstrainedEquality` for every joint:
    /// a joint whose constrained-state input is connected has its equality
    /// active and is held at the commanded value (see the module docs).
    fn update_constrained_joints(&mut self, data: &mut MjSceneData) {
        let mut targets = Vec::new();
        for joint in self
            .bodies
            .iter()
            .flat_map(|body| body.scalar_joints.iter())
        {
            let equality = self
                .equalities
                .iter()
                .find(|equality| equality.name == joint.constrained_equality)
                .expect("every scalar joint has its constrained equality");
            let connected = joint.constrained_state_in_msg.is_connected();
            data.eq_active_mut()[equality.id] = connected;
            let offset = if connected {
                joint.constrained_state_in_msg.read().state
            } else {
                0.0
            };
            targets.push((
                joint.constrained_tendon_id,
                data.model().qpos0()[joint.qpos_adr] + offset,
            ));
        }
        set_tendon_references(data, &targets);
        self.constrained_targets = targets;
    }

    fn write_body_state_outputs(&self, state: &MjSceneState) {
        for body in &self.bodies {
            body.mass_properties_out_msg.write(SpacecraftMassPropsMsg {
                mass_kg: state.mass[body.id],
                ..SpacecraftMassPropsMsg::default()
            });
            for joint in &body.scalar_joints {
                joint.state_out_msg.write(ScalarJointStateMsg {
                    state: state.qpos[joint.qpos_adr],
                });
                joint.state_dot_out_msg.write(ScalarJointStateMsg {
                    state: state.qvel[joint.qvel_adr],
                });
            }
        }
    }

    fn write_state_message(&self) {
        self.state_out_msg.write(MjSceneStateMsg {
            qpos: self.state.qpos.as_slice().to_vec(),
            qvel: self.state.qvel.as_slice().to_vec(),
            act: self.state.act.as_slice().to_vec(),
        });
    }

    /// Upstream `MJFwdKinematics::fwdKinematics`: refreshes the positions and
    /// velocities when the joint states changed, and publishes the sites.
    fn forward_kinematics(&mut self, data: &mut MjSceneData) {
        if !self.kinematics_stale {
            return;
        }
        data.fwd_position();
        data.fwd_velocity();
        self.kinematics_stale = false;
        for site in self.bodies.iter().flat_map(|body| body.sites.iter()) {
            site.state_out_msg.write(site_state(data, site.id));
        }
    }

    /// Runs the dynamics or diffusion task at `nanos` for `state`. The
    /// diffusion task also runs the dynamics-task models added to both
    /// tasks, merged by priority (equal priorities in the order the models
    /// were added).
    fn execute_task(
        &mut self,
        task: Task,
        data: &mut MjSceneData,
        state: &MjSceneState,
        nanos: u64,
        epoch: Epoch,
    ) {
        match task {
            Task::Dynamics => {
                for index in 0..self.dynamics_task.len() {
                    self.execute_entry(Task::Dynamics, index, data, state, nanos, epoch);
                }
            }
            Task::Diffusion => {
                let mut order: Vec<(i32, usize, Task, usize)> =
                    self.diffusion_task
                        .iter()
                        .enumerate()
                        .map(|(index, entry)| (entry.priority, entry.key, Task::Diffusion, index))
                        .chain(self.dynamics_task.iter().enumerate().filter_map(
                            |(index, entry)| {
                                entry
                                    .diffusion_priority
                                    .map(|priority| (priority, entry.key, Task::Dynamics, index))
                            },
                        ))
                        .collect();
                order.sort_by_key(|(priority, key, ..)| (std::cmp::Reverse(*priority), *key));
                for (_, _, task, index) in order {
                    self.execute_entry(task, index, data, state, nanos, epoch);
                }
            }
        }
    }

    /// Runs entry `index` of `task` (a forward-kinematics pass or a model).
    fn execute_entry(
        &mut self,
        task: Task,
        index: usize,
        data: &mut MjSceneData,
        state: &MjSceneState,
        nanos: u64,
        epoch: Epoch,
    ) {
        if matches!(self.task(task)[index].kind, TaskKind::FwdKinematics) {
            self.forward_kinematics(data);
            return;
        }
        let entries = match task {
            Task::Dynamics => &mut self.dynamics_task,
            Task::Diffusion => &mut self.diffusion_task,
        };
        let entry = &mut entries[index];
        let TaskKind::Model(model) = &mut entry.kind else {
            unreachable!("checked above");
        };
        let mut context = TaskContext {
            current_sim_nanos: nanos,
            current_epoch: epoch,
            scene: SceneView {
                data,
                bodies: &self.bodies,
            },
            states: &state.models.as_slice()[entry.range.clone()],
            derivatives: &mut self.model_derivatives.as_mut_slice()[entry.range.clone()],
            diffusions: &mut self.model_diffusions[entry.diffusion_range.clone()],
            slots: &entry.slots,
            base_offset: entry.range.start,
            base_diffusion_offset: entry.diffusion_range.start,
        };
        model.scene_update(&mut context);
    }

    /// Upstream `MJScene::equationsOfMotion`.
    fn evaluate(&mut self, state: &MjSceneState, nanos: u64, epoch: Epoch) -> MjSceneDerivative {
        self.recompile_or_panic();
        let handle = self.handle.clone();
        let mut guard = handle.lock();
        let data = &mut *guard;

        self.synchronize(data, state);
        data.set_time(nanos as f64 * 1e-9);
        // Zero the controls on the first evaluation, so nothing uninitialized
        // drives the system at the start.
        if self.first_dynamics_call {
            data.ctrl_mut().fill(0.0);
            self.first_dynamics_call = false;
        }
        self.write_body_state_outputs(state);

        self.execute_task(Task::Dynamics, data, state, nanos, epoch);
        // Refresh in case nothing in the task ran the kinematics.
        self.forward_kinematics(data);

        for actuator in &self.actuators {
            actuator.update_ctrl(data.ctrl_mut());
        }
        self.update_constrained_joints(data);

        data.fwd_actuation();
        data.fwd_acceleration();
        data.fwd_constraint();

        assert!(
            data.qacc().iter().all(|value| !value.is_nan()),
            "Encountered NaN acceleration at time {}s in MJScene '{}'",
            nanos as f64 * 1e-9,
            self.name
        );

        for body in &self.bodies {
            if body.derivative_mass_properties_in_msg.is_connected() {
                self.mass_derivative[body.id] =
                    body.derivative_mass_properties_in_msg.read().mass_kg;
            }
        }

        MjSceneDerivative {
            qpos: self.policy.rate(&state.qpos, &state.qvel),
            qvel: DVector::from_column_slice(data.qacc()),
            act: DVector::from_column_slice(data.act_dot()),
            mass: self.mass_derivative.clone(),
            models: self.model_derivatives.clone(),
        }
    }

    /// Upstream `MJScene::equationsOfMotionDiffusion`: runs the diffusion
    /// task and returns one column per noise source.
    fn evaluate_diffusion(
        &mut self,
        state: &MjSceneState,
        nanos: u64,
        epoch: Epoch,
    ) -> Vec<MjSceneDerivative> {
        self.recompile_or_panic();
        let handle = self.handle.clone();
        let mut guard = handle.lock();
        let data = &mut *guard;
        self.synchronize(data, state);
        data.set_time(nanos as f64 * 1e-9);
        self.execute_task(Task::Diffusion, data, state, nanos, epoch);
        drop(guard);

        let zero = self.zero_derivative();
        let mut columns = Vec::new();
        for entry in self.dynamics_task.iter().chain(self.diffusion_task.iter()) {
            for slot in &entry.slots {
                for source in 0..slot.noise_sources {
                    let mut column = zero.clone();
                    let start = slot.diffusion_offset + source * slot.len;
                    column.models.as_mut_slice()[slot.offset..slot.offset + slot.len]
                        .copy_from_slice(&self.model_diffusions[start..start + slot.len]);
                    columns.push(column);
                }
            }
        }
        columns
    }

    fn zero_derivative(&self) -> MjSceneDerivative {
        MjSceneDerivative {
            qpos: DVector::zeros(
                self.policy
                    .rate_len(self.state.qpos.len(), self.state.qvel.len()),
            ),
            qvel: DVector::zeros(self.state.qvel.len()),
            act: DVector::zeros(self.state.act.len()),
            mass: DVector::zeros(self.state.mass.len()),
            models: DVector::zeros(self.state.models.len()),
        }
    }

    /// Upstream `MJScene::Reset`.
    fn reset_scene(&mut self, current_sim_nanos: u64) {
        self.recompile_or_panic();
        let handle = self.handle.clone();
        let mut guard = handle.lock();
        let data = &mut *guard;
        if self.topology.is_none() {
            self.topology = Some(Topology::capture(data.model()));
        }
        self.time_before_nanos = current_sim_nanos;
        self.first_dynamics_call = true;
        self.register_model_states();
        self.mass_derivative = DVector::zeros(self.state.mass.len());

        let context = SimulationContext {
            current_sim_nanos,
            // Upstream resets with the time only; the epoch is not known at
            // `Module::init`.
            current_epoch: Epoch::from_tai_seconds(0.0),
        };
        for entry in self
            .dynamics_task
            .iter_mut()
            .chain(self.diffusion_task.iter_mut())
        {
            if let TaskKind::Model(model) = &mut entry.kind {
                model.scene_reset(
                    &SceneView {
                        data,
                        bodies: &self.bodies,
                    },
                    &context,
                );
            }
        }
        assert!(
            !self.needs_recompile,
            "MJScene '{}': configure MuJoCo specification changes before Reset.",
            self.name
        );
        let state = self.state.clone();
        self.synchronize(data, &state);
        data.set_time(current_sim_nanos as f64 * 1e-9);
        drop(guard);
        self.write_state_message();
        self.load_model_states();
    }

    /// Upstream `postIntegration`: publishes the kinematics of the final
    /// state, or evaluates the equations of motion once more with it.
    fn post_integration(&mut self, nanos: u64, epoch: Epoch) {
        self.time_before_nanos = nanos;
        let state = self.state.clone();
        if self.extra_eom_call {
            self.evaluate(&state, nanos, epoch);
            if self.noise_source_count() > 0 || !self.diffusion_task.is_empty() {
                self.evaluate_diffusion(&state, nanos, epoch);
            }
        } else {
            let handle = self.handle.clone();
            let mut guard = handle.lock();
            copy_state_into(&mut guard, &state);
            self.kinematics_stale = true;
            self.forward_kinematics(&mut guard);
        }
    }
}

/// Upstream `MJSite::writeFwdKinematicsMessage`: the site frame state from
/// the MuJoCo data.
fn site_state(data: &MjSceneData, site: usize) -> SpacecraftStateMsg {
    let rotation = Matrix3::from_row_slice(&data.site_xmat()[site]);
    let velocity = data.object_velocity(MjtObjBase::mjOBJ_SITE, site, false);
    SpacecraftStateMsg {
        position_m: Vector3::from(data.site_xpos()[site]),
        velocity_mps: Vector3::new(velocity[3], velocity[4], velocity[5]),
        sigma_bn: dcm_to_mrp(rotation.transpose()),
        omega_radps: rotation.transpose() * Vector3::new(velocity[0], velocity[1], velocity[2]),
        ..SpacecraftStateMsg::default()
    }
}

/// Copies the common prefix of `source` into `destination` (upstream copies
/// `min(old, new)` entries of every runtime array on recompilation).
fn copy_prefix<T: Copy>(destination: &mut [T], source: &[T]) {
    let len = destination.len().min(source.len());
    destination[..len].copy_from_slice(&source[..len]);
}

/// Upstream `copyMujocoArraysFromStates`.
fn copy_state_into(data: &mut MjSceneData, state: &MjSceneState) {
    data.qpos_mut().copy_from_slice(state.qpos.as_slice());
    data.qvel_mut().copy_from_slice(state.qvel.as_slice());
    data.act_mut().copy_from_slice(state.act.as_slice());
}

/// Gives each `(tendon, length)` its reference length `tendon_length0`,
/// changing the model only when one differs.
fn set_tendon_references(data: &mut MjSceneData, targets: &[(usize, f64)]) {
    if targets
        .iter()
        .any(|&(tendon, length)| data.model().tendon_length0()[tendon] != length)
    {
        mutate_model(data, |model| {
            for &(tendon, length) in targets {
                model.tendon_length0_mut()[tendon] = length;
            }
        });
    }
}

/// Changes model parameters in place: mujoco-rs only gives mutable access to
/// a model a data does not own, so a modified copy replaces it (the layouts
/// match, which [`MjData::swap_model`] checks).
fn mutate_model(data: &mut MjSceneData, mutate: impl FnOnce(&mut MjModel)) {
    let mut model = Box::new(data.model().clone());
    mutate(&mut model);
    data.swap_model(model);
}

// ---------------------------------------------------------------------------
// Body views
// ---------------------------------------------------------------------------

/// A body with read access to its states (upstream `MJBody` getters).
pub struct BodyRef<'a> {
    body: &'a MjBody,
    state: &'a MjSceneState,
}

impl Deref for BodyRef<'_> {
    type Target = MjBody;

    fn deref(&self) -> &MjBody {
        self.body
    }
}

impl BodyRef<'_> {
    /// Upstream `getMass` \[kg\].
    pub fn mass(&self) -> f64 {
        self.state.mass[self.body.id]
    }

    /// Position \[rad\] or \[m\] of the scalar joint `name` (upstream
    /// `getPositionState`).
    pub fn scalar_joint_position(&self, name: &str) -> f64 {
        self.state.qpos[self.body.scalar_joint(name).qpos_adr]
    }

    /// Rate \[rad/s\] or \[m/s\] of the scalar joint `name` (upstream
    /// `getVelocityState`).
    pub fn scalar_joint_velocity(&self, name: &str) -> f64 {
        self.state.qvel[self.body.scalar_joint(name).qvel_adr]
    }
}

/// A body with its state setters (upstream `MJBody` and `MJJoint` setters)
/// and mutable ports.
pub struct BodyMut<'a> {
    body: &'a mut MjBody,
    state: &'a mut MjSceneState,
    kinematics_stale: &'a mut bool,
}

impl Deref for BodyMut<'_> {
    type Target = MjBody;

    fn deref(&self) -> &MjBody {
        self.body
    }
}

impl DerefMut for BodyMut<'_> {
    fn deref_mut(&mut self) -> &mut MjBody {
        self.body
    }
}

impl BodyMut<'_> {
    /// Upstream `getMass` \[kg\].
    pub fn mass(&self) -> f64 {
        self.state.mass[self.body.id]
    }

    /// The body's entry of the mass state \[kg\] (upstream
    /// `getMassState()->setState` for one body).
    pub fn set_mass(&mut self, mass: f64) {
        self.state.mass[self.body.id] = mass;
    }

    /// Upstream `MJBody::setPosition` \[m\]: the free-joint position, or the
    /// first three scalar joints when they translate along the body axes.
    ///
    /// # Panics
    ///
    /// If the body has neither.
    pub fn set_position(&mut self, position: Vector3<f64>) {
        if let Some(joint) = &self.body.free_joint {
            let adr = joint.qpos_adr;
            self.state.qpos.rows_mut(adr, 3).copy_from(&position);
        } else if self.body.has_3d_translation() {
            for axis in 0..3 {
                self.state.qpos[self.body.scalar_joints[axis].qpos_adr] = position[axis];
            }
        } else {
            panic!(
                "Tried to set position in a body with no 'free' joint or no 3D translational \
                 joints: {}",
                self.body.name
            );
        }
        *self.kinematics_stale = true;
    }

    /// Upstream `MJBody::setVelocity` \[m/s\], like
    /// [`set_position`](Self::set_position).
    ///
    /// # Panics
    ///
    /// If the body has neither a free joint nor three translational joints.
    pub fn set_velocity(&mut self, velocity: Vector3<f64>) {
        if let Some(joint) = &self.body.free_joint {
            let adr = joint.qvel_adr;
            self.state.qvel.rows_mut(adr, 3).copy_from(&velocity);
        } else if self.body.has_3d_translation() {
            for axis in 0..3 {
                self.state.qvel[self.body.scalar_joints[axis].qvel_adr] = velocity[axis];
            }
        } else {
            panic!(
                "Tried to set velocity in a body with no 'free' joint or no 3D translational \
                 joints: {}",
                self.body.name
            );
        }
        *self.kinematics_stale = true;
    }

    fn free_joint_or_panic(&self, what: &str) -> MjFreeJoint {
        self.body
            .free_joint
            .clone()
            .unwrap_or_else(|| panic!("Tried to set {what} in non-free body: {}", self.body.name))
    }

    /// Upstream `MJBody::setAttitude`: the free-joint attitude from the MRP
    /// `sigma_BN`.
    ///
    /// # Panics
    ///
    /// If the body is not free.
    pub fn set_attitude(&mut self, sigma_bn: Vector3<f64>) {
        let joint = self.free_joint_or_panic("attitude");
        let quat = mrp_to_ep(sigma_bn);
        self.state
            .qpos
            .rows_mut(joint.qpos_adr + 3, 4)
            .copy_from(&quat);
        *self.kinematics_stale = true;
    }

    /// Upstream `MJBody::setAttitudeRate` \[rad/s\], body-frame components.
    ///
    /// # Panics
    ///
    /// If the body is not free.
    pub fn set_attitude_rate(&mut self, omega_bn_b: Vector3<f64>) {
        let joint = self.free_joint_or_panic("attitude rate");
        self.state
            .qvel
            .rows_mut(joint.qvel_adr + 3, 3)
            .copy_from(&omega_bn_b);
        *self.kinematics_stale = true;
    }

    /// Upstream `MJScalarJoint::setPosition` \[rad\] or \[m\].
    pub fn set_scalar_joint_position(&mut self, name: &str, value: f64) {
        let adr = self.body.scalar_joint(name).qpos_adr;
        self.state.qpos[adr] = value;
        *self.kinematics_stale = true;
    }

    /// Upstream `MJScalarJoint::setVelocity` \[rad/s\] or \[m/s\].
    pub fn set_scalar_joint_velocity(&mut self, name: &str, value: f64) {
        let adr = self.body.scalar_joint(name).qvel_adr;
        self.state.qvel[adr] = value;
        *self.kinematics_stale = true;
    }

    /// The ball joint's scalar-first quaternion (upstream
    /// `getPositionState()->setState`).
    pub fn set_ball_joint_quaternion(&mut self, quaternion: [f64; 4]) {
        let adr = self.body.ball_joint().qpos_adr;
        self.state.qpos.as_mut_slice()[adr..adr + 4].copy_from_slice(&quaternion);
        *self.kinematics_stale = true;
    }

    /// The ball joint's body rate \[rad/s\] (upstream
    /// `getVelocityState()->setState`).
    pub fn set_ball_joint_rate(&mut self, omega: Vector3<f64>) {
        let adr = self.body.ball_joint().qvel_adr;
        self.state.qvel.rows_mut(adr, 3).copy_from(&omega);
        *self.kinematics_stale = true;
    }

    /// The free joint's scalar-first quaternion (upstream
    /// `getAttitudeState()->setState`).
    pub fn set_free_joint_quaternion(&mut self, quaternion: [f64; 4]) {
        let joint = self.free_joint_or_panic("attitude");
        let adr = joint.qpos_adr + 3;
        self.state.qpos.as_mut_slice()[adr..adr + 4].copy_from_slice(&quaternion);
        *self.kinematics_stale = true;
    }
}

// ---------------------------------------------------------------------------
// Integration
// ---------------------------------------------------------------------------

impl MjScene {
    fn combined(
        weighted: &[(&MjSceneDerivative, f64)],
        select: impl Fn(&MjSceneDerivative) -> &DVector<f64>,
        len: usize,
    ) -> DVector<f64> {
        let mut sum = DVector::zeros(len);
        for &(derivative, weight) in weighted {
            if weight != 0.0 {
                sum.axpy(weight, select(derivative), 1.0);
            }
        }
        sum
    }
}

impl DynamicObject for MjScene {
    type State = MjSceneState;
    type StepOutput = ();
    type Derivative = MjSceneDerivative;

    fn equations_of_motion(
        &mut self,
        state: &MjSceneState,
        current_sim_nanos: u64,
        current_epoch: Epoch,
    ) -> MjSceneDerivative {
        self.evaluate(state, current_sim_nanos, current_epoch)
    }

    fn state_with_derivative(
        &self,
        state: &MjSceneState,
        derivative: &MjSceneDerivative,
        dt: f64,
    ) -> MjSceneState {
        self.state_with_derivatives(state, &[(derivative, 1.0)], dt)
    }

    /// Forms the weighted rate first and applies it once, as upstream's
    /// `buildDriftCandidate` does: the quaternion update is not additive.
    fn state_with_derivatives(
        &self,
        state: &MjSceneState,
        weighted: &[(&MjSceneDerivative, f64)],
        step: f64,
    ) -> MjSceneState {
        let advance = |base: &DVector<f64>,
                       select: &dyn Fn(&MjSceneDerivative) -> &DVector<f64>| {
            let mut next = base.clone();
            next.axpy(step, &Self::combined(weighted, select, base.len()), 1.0);
            next
        };
        let qpos_rate = Self::combined(
            weighted,
            |derivative| &derivative.qpos,
            self.policy.rate_len(state.qpos.len(), state.qvel.len()),
        );
        MjSceneState {
            qpos: self.policy.advance(&state.qpos, &qpos_rate, step),
            qvel: advance(&state.qvel, &|derivative| &derivative.qvel),
            act: advance(&state.act, &|derivative| &derivative.act),
            mass: advance(&state.mass, &|derivative| &derivative.mass),
            models: advance(&state.models, &|derivative| &derivative.models),
        }
    }

    fn combine(
        &self,
        state: &MjSceneState,
        weighted: Vec<(MjSceneDerivative, f64)>,
        step: f64,
    ) -> (MjSceneState, ()) {
        let borrowed: Vec<(&MjSceneDerivative, f64)> = weighted
            .iter()
            .map(|(derivative, weight)| (derivative, *weight))
            .collect();
        (self.state_with_derivatives(state, &borrowed, step), ())
    }

    fn merge_step_outputs(&self, _accumulated: (), _next: ()) {}

    /// Weighted RMS over every scalar, with a zero relative tolerance for
    /// the free-joint translations and velocities (upstream
    /// `configureAdaptiveStateTolerances`: at orbital scale a relative
    /// tolerance on the position would swamp the attitude-sized errors).
    fn error_norm(
        &self,
        high: &MjSceneState,
        low: &MjSceneState,
        atol: f64,
        rtol: f64,
    ) -> Option<f64> {
        let (free_qpos, free_qvel) = self.policy.free_translation_entries();
        let mut sum = 0.0;
        let mut count = 0usize;
        let mut accumulate = |high: &DVector<f64>, low: &DVector<f64>, absolute_only: &[usize]| {
            for (index, (h, l)) in high.iter().zip(low.iter()).enumerate() {
                let relative = if absolute_only.contains(&index) {
                    0.0
                } else {
                    rtol
                };
                let scale = atol + relative * h.abs().max(l.abs());
                let scaled = (h - l) / scale;
                sum += scaled * scaled;
                count += 1;
            }
        };
        accumulate(&high.qpos, &low.qpos, &free_qpos);
        accumulate(&high.qvel, &low.qvel, &free_qvel);
        accumulate(&high.act, &low.act, &[]);
        accumulate(&high.mass, &low.mass, &[]);
        accumulate(&high.models, &low.models, &[]);
        Some(if count == 0 {
            0.0
        } else {
            (sum / count as f64).sqrt()
        })
    }

    fn noise_source_count(&self) -> usize {
        self.dynamics_task
            .iter()
            .chain(self.diffusion_task.iter())
            .flat_map(|entry| entry.slots.iter())
            .map(|slot| slot.noise_sources)
            .sum()
    }

    fn equations_of_motion_diffusion(
        &mut self,
        state: &MjSceneState,
        current_sim_nanos: u64,
        current_epoch: Epoch,
    ) -> Vec<MjSceneDerivative> {
        self.evaluate_diffusion(state, current_sim_nanos, current_epoch)
    }
}

impl Module for MjScene {
    fn init(&mut self) {
        self.reset_scene(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.reset_scene(context.current_sim_nanos);
    }

    /// Upstream `UpdateState`: integrates from the previous update to now in
    /// one step, then publishes the state.
    fn update(&mut self, context: &SimulationContext) {
        let now = context.current_sim_nanos;
        let start = self.time_before_nanos;
        assert!(
            now >= start,
            "MJScene '{}' updated at {now} ns, before its previous update at {start} ns",
            self.name
        );
        let state = self.state.clone();
        if now > start {
            let elapsed = now - start;
            let start_epoch =
                context.current_epoch - Duration::from_total_nanoseconds(elapsed as i128);
            let integrator = self.integrator.take();
            let default_integrator = Rk4;
            let chosen: &dyn Integrator<MjScene> =
                integrator.as_deref().unwrap_or(&default_integrator);
            let (next, ()) =
                chosen.propagate(self, &state, start, start_epoch, elapsed as f64 * 1e-9);
            self.integrator = integrator;
            self.state = next;
        } else {
            self.evaluate(&state, now, context.current_epoch);
        }
        self.post_integration(now, context.current_epoch);
        self.write_state_message();
        let state = self.state.clone();
        self.write_body_state_outputs(&state);
        self.load_model_states();
    }
}
