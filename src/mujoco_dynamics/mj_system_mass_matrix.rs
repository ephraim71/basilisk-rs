//! Joint-space mass matrix of a scene: port of upstream Basilisk
//! `simulation/mujocoDynamics/MJSystemMassMatrix`.
//!
//! Publishes MuJoCo's dense mass matrix (`mj_fullM`) with the scene's
//! layout: the joint kinds, and one spacecraft per kinematic tree whose root
//! body (a child of the world) has joints, starting at that body's first
//! joint. Every free joint must be the first joint of a tree root.
//!
//! Like [`MjSystemCom`](super::mj_system_com::MjSystemCom), it reads the
//! scene from the simulation's schedule or from inside a scene task.

use nalgebra::DMatrix;

use super::scene::{MjScene, MjSceneHandle};
use super::task::{MjSceneData, SceneModel, SceneView, TaskContext};
use crate::messages::{MjJointType, MjSystemMassMatrixMsg, Output};
use crate::mujoco_dynamics::mujoco_rs::prelude::{MjModel, MjtJoint};
use crate::{Module, SimulationContext};

/// Upstream `jointTypes` code of a MuJoCo joint.
pub(crate) fn joint_type(kind: MjtJoint) -> MjJointType {
    match kind {
        MjtJoint::mjJNT_FREE => MjJointType::Free,
        MjtJoint::mjJNT_BALL => MjJointType::Ball,
        MjtJoint::mjJNT_SLIDE => MjJointType::Slide,
        _ => MjJointType::Hinge,
    }
}

/// Upstream's check that every free joint is the first joint of a
/// kinematic-tree root body.
///
/// # Panics
///
/// If one is not (upstream raises `BasiliskError`).
pub(crate) fn check_free_joints(model: &MjModel, module: &str) {
    for joint in 0..model.ffi().njnt as usize {
        if model.jnt_type()[joint] == MjtJoint::mjJNT_FREE {
            let body = model.jnt_bodyid()[joint] as usize;
            let is_root = model.body_parentid()[body] == 0;
            let is_first = joint as i32 == model.body_jntadr()[body];
            assert!(
                is_root && is_first,
                "{module}: Free joint j={joint} on body b={body} must be the first joint on a \
                 tree root body."
            );
        }
    }
}

/// Upstream `MJSystemMassMatrix`.
#[derive(Debug)]
pub struct MjSystemMassMatrix {
    /// Upstream `ModelTag`.
    pub name: String,
    /// Upstream `massMatrixOutMsg`.
    pub mass_matrix_out_msg: Output<MjSystemMassMatrixMsg>,
    scene: MjSceneHandle,
    sc_start_idx: Vec<usize>,
    joint_types: Vec<MjJointType>,
}

impl MjSystemMassMatrix {
    /// The module for `scene` (upstream `module.scene = scene`).
    pub fn new(scene: &MjScene) -> Self {
        Self {
            name: "MJSystemMassMatrix".to_string(),
            mass_matrix_out_msg: Output::default(),
            scene: scene.handle(),
            sc_start_idx: Vec::new(),
            joint_types: Vec::new(),
        }
    }

    /// Upstream `Reset`: the joint kinds and the spacecraft of the scene.
    fn configure(&mut self, model: &MjModel) {
        self.joint_types = model
            .jnt_type()
            .iter()
            .map(|kind| joint_type(*kind))
            .collect();
        self.sc_start_idx = (1..model.ffi().nbody as usize)
            .filter(|&body| model.body_parentid()[body] == 0 && model.body_jntnum()[body] > 0)
            .map(|body| model.body_jntadr()[body] as usize)
            .collect();
        check_free_joints(model, "MJSystemMassMatrix");
    }

    /// Upstream `UpdateState`.
    fn publish(&self, data: &MjSceneData) {
        let nv = data.model().ffi().nv as usize;
        let mut dense = vec![0.0; nv * nv];
        data.full_m(&mut dense)
            .expect("the mass matrix buffer has nv x nv entries");
        self.mass_matrix_out_msg.write(MjSystemMassMatrixMsg {
            num_spacecraft: self.sc_start_idx.len(),
            sc_start_idx: self.sc_start_idx.clone(),
            joint_types: self.joint_types.clone(),
            mass_matrix: DMatrix::from_row_slice(nv, nv, &dense),
        });
    }
}

impl Module for MjSystemMassMatrix {
    fn init(&mut self) {
        let scene = self.scene.clone();
        self.configure(scene.lock().model());
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.init();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let scene = self.scene.clone();
        self.publish(&scene.lock());
    }
}

impl SceneModel for MjSystemMassMatrix {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn scene_reset(&mut self, scene: &SceneView<'_>, _context: &SimulationContext) {
        self.configure(scene.model());
    }

    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        self.publish(context.scene().data());
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_MJSystemMassMatrix.py`.

    use nalgebra::Matrix3;

    use super::*;
    use crate::mujoco_dynamics::tests::context;

    const HUB_MASS: f64 = 50.0;
    const ARM_MASS: f64 = 10.0;

    fn hub_inertia() -> Matrix3<f64> {
        Matrix3::from_diagonal(&nalgebra::Vector3::new(10.0, 11.0, 12.0))
    }

    fn rod_inertia() -> Matrix3<f64> {
        Matrix3::from_diagonal(&nalgebra::Vector3::new(5.0, 6.0, 7.0))
    }

    const HUB: &str = r#"<inertial pos="0 0 0" quat="1 0 0 0" mass="50" diaginertia="10 11 12"/>
      <geom type="box" size="0.5 0.3 0.2"/>"#;

    fn xml(bodies: &str) -> String {
        format!(
            r#"<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    {bodies}
  </worldbody>
</mujoco>"#
        )
    }

    fn single() -> String {
        xml(&format!(r#"<body name="hub"><freejoint/>{HUB}</body>"#))
    }

    fn one_joint(joint: &str, position: &str, name: &str) -> String {
        xml(&format!(
            r#"<body name="hub"><freejoint/>{HUB}
      <body name="{name}" pos="{position}">
        <joint name="j" {joint}/>
        <inertial pos="0 0 0" quat="1 0 0 0" mass="10" diaginertia="5 6 7"/>
      </body>
    </body>"#
        ))
    }

    fn double() -> String {
        xml(&format!(
            r#"<body name="hub1"><freejoint/>{HUB}</body>
    <body name="hub2"><freejoint/>{HUB}</body>"#
        ))
    }

    fn translation() -> String {
        xml(r#"<body name="hub">
      <joint name="tx" type="slide" axis="1 0 0" limited="false"/>
      <joint name="ty" type="slide" axis="0 1 0" limited="false"/>
      <joint name="tz" type="slide" axis="0 0 1" limited="false"/>
      <inertial pos="0 0 0" mass="50" diaginertia="10 11 12"/>
      <geom type="box" size="0.5 0.3 0.2"/>
    </body>"#)
    }

    /// Upstream `hingedMassMatrix`, hinge angle zero.
    fn hinged_mass_matrix(arm_length: f64) -> DMatrix<f64> {
        let mut m = DMatrix::zeros(7, 7);
        let (hub, rod) = (hub_inertia(), rod_inertia());
        for i in 0..3 {
            m[(i, i)] = HUB_MASS + ARM_MASS;
            for j in 0..3 {
                m[(3 + i, 3 + j)] = hub[(i, j)] + rod[(i, j)];
            }
        }
        m[(4, 4)] += ARM_MASS * arm_length.powi(2);
        m[(5, 5)] += ARM_MASS * arm_length.powi(2);
        m[(2, 4)] = -ARM_MASS * arm_length;
        m[(4, 2)] = -ARM_MASS * arm_length;
        m[(1, 5)] = ARM_MASS * arm_length;
        m[(5, 1)] = ARM_MASS * arm_length;
        m[(6, 6)] = rod[(2, 2)];
        m[(6, 5)] = rod[(2, 2)];
        m[(5, 6)] = rod[(2, 2)];
        m
    }

    /// Upstream `sliderMassMatrix`, slide position zero.
    fn slider_mass_matrix(arm_length: f64) -> DMatrix<f64> {
        let mut m = DMatrix::zeros(7, 7);
        let (hub, rod) = (hub_inertia(), rod_inertia());
        for i in 0..3 {
            m[(i, i)] = HUB_MASS + ARM_MASS;
            for j in 0..3 {
                m[(3 + i, 3 + j)] = hub[(i, j)] + rod[(i, j)];
            }
        }
        m[(4, 4)] += ARM_MASS * arm_length.powi(2);
        m[(5, 5)] += ARM_MASS * arm_length.powi(2);
        m[(2, 4)] = ARM_MASS * arm_length;
        m[(4, 2)] = ARM_MASS * arm_length;
        m[(1, 5)] = -ARM_MASS * arm_length;
        m[(5, 1)] = -ARM_MASS * arm_length;
        m[(6, 6)] = ARM_MASS;
        m[(0, 6)] = ARM_MASS;
        m[(6, 0)] = ARM_MASS;
        m
    }

    fn block_diagonal_hub(count: usize) -> DMatrix<f64> {
        let mut m = DMatrix::zeros(6 * count, 6 * count);
        let hub = hub_inertia();
        for k in 0..count {
            for i in 0..3 {
                m[(6 * k + i, 6 * k + i)] = HUB_MASS;
                for j in 0..3 {
                    m[(6 * k + 3 + i, 6 * k + 3 + j)] = hub[(i, j)];
                }
            }
        }
        m
    }

    /// Upstream `test_MJSystemMassMatrix` over its five models: the
    /// spacecraft count, their first joints, the joint kinds and the mass
    /// matrix at the first sample.
    #[test]
    fn mass_matrix_and_layout_of_the_scene() {
        use MjJointType::{Free, Hinge, Slide};
        /// Name, XML, spacecraft count, first joints, joint kinds, mass matrix.
        type Case = (
            &'static str,
            String,
            usize,
            Vec<usize>,
            Vec<MjJointType>,
            DMatrix<f64>,
        );
        let cases: Vec<Case> = vec![
            (
                "One SC",
                single(),
                1,
                vec![0],
                vec![Free],
                block_diagonal_hub(1),
            ),
            (
                "One Hinge",
                one_joint(r#"type="hinge" axis="0 0 1""#, "0.6 0 0", "link_h"),
                1,
                vec![0],
                vec![Free, Hinge],
                hinged_mass_matrix(0.6),
            ),
            (
                "One Slider",
                one_joint(r#"type="slide" axis="1 0 0""#, "-0.6 0 0", "link_s"),
                1,
                vec![0],
                vec![Free, Slide],
                slider_mass_matrix(0.6),
            ),
            (
                "Two SC",
                double(),
                2,
                vec![0, 1],
                vec![Free, Free],
                block_diagonal_hub(2),
            ),
            (
                "Translation SC Only",
                translation(),
                1,
                vec![0],
                vec![Slide, Slide, Slide],
                DMatrix::identity(3, 3) * HUB_MASS,
            ),
        ];
        for (name, xml, num_spacecraft, sc_start_idx, joint_types, truth) in cases {
            let mut scene = MjScene::new(&xml).expect("scene");
            scene.extra_eom_call = true;
            let mut module = MjSystemMassMatrix::new(&scene);
            scene.init();
            module.init();
            scene.update(&context(0));
            module.update(&context(0));
            let msg = module.mass_matrix_out_msg.read();
            assert_eq!(msg.num_spacecraft, num_spacecraft, "{name}");
            assert_eq!(msg.sc_start_idx, sc_start_idx, "{name}");
            assert_eq!(msg.joint_types, joint_types, "{name}");
            assert_eq!(msg.mass_matrix.shape(), truth.shape(), "{name}");
            for (actual, expected) in msg.mass_matrix.iter().zip(truth.iter()) {
                assert!(
                    (actual - expected).abs() <= 1e-8 * expected.abs().max(1e-300) + 1e-12,
                    "{name}: {} vs {truth}",
                    msg.mass_matrix
                );
            }
        }
    }
}
