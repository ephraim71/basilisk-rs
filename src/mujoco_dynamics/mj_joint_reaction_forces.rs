//! Generalized forces on the joints of a scene: port of upstream Basilisk
//! `simulation/mujocoDynamics/MJJointReactionForces`.
//!
//! Publishes, for every degree of freedom, MuJoCo's bias (Coriolis,
//! centrifugal), passive (springs, dampers), constraint, applied and
//! actuator generalized forces of the last evaluation, with the joint
//! layout: each joint's kinematic tree (numbered in body order of the tree
//! roots), body, kind and first degree of freedom. Every free joint must be
//! the first joint of a tree root.
//!
//! Like [`MjSystemCom`](super::mj_system_com::MjSystemCom), it reads the
//! scene from the simulation's schedule or from inside a scene task.

use super::mj_system_mass_matrix::{check_free_joints, joint_type};
use super::scene::{MjScene, MjSceneHandle};
use super::task::{MjSceneData, SceneModel, SceneView, TaskContext};
use crate::messages::{MjJointReactionsMsg, MjJointType, Output};
use crate::mujoco_dynamics::mujoco_rs::prelude::MjModel;
use crate::{Module, SimulationContext};

/// Upstream `MJJointReactionForces`.
#[derive(Debug)]
pub struct MjJointReactionForces {
    /// Upstream `ModelTag`.
    pub name: String,
    /// Upstream `reactionForcesOutMsg`.
    pub reaction_forces_out_msg: Output<MjJointReactionsMsg>,
    scene: MjSceneHandle,
    joint_tree_idx: Vec<usize>,
    joint_parent_body_idx: Vec<usize>,
    joint_types: Vec<MjJointType>,
    joint_dof_start: Vec<usize>,
}

impl MjJointReactionForces {
    /// The module for `scene` (upstream `module.scene = scene`).
    pub fn new(scene: &MjScene) -> Self {
        Self {
            name: "MJJointReactionForces".to_string(),
            reaction_forces_out_msg: Output::default(),
            scene: scene.handle(),
            joint_tree_idx: Vec::new(),
            joint_parent_body_idx: Vec::new(),
            joint_types: Vec::new(),
            joint_dof_start: Vec::new(),
        }
    }

    /// Upstream `Reset`: the static joint layout.
    fn configure(&mut self, model: &MjModel) {
        let nbody = model.ffi().nbody as usize;
        let parents = model.body_parentid();
        let mut tree_of_body: Vec<Option<usize>> = vec![None; nbody];
        let mut next_tree = 0;
        for body in 1..nbody {
            let mut root = body;
            while root > 0 && parents[root] != 0 {
                root = parents[root] as usize;
            }
            let tree = *tree_of_body[root].get_or_insert_with(|| {
                next_tree += 1;
                next_tree - 1
            });
            tree_of_body[body] = Some(tree);
        }
        let njnt = model.ffi().njnt as usize;
        self.joint_parent_body_idx = (0..njnt)
            .map(|joint| model.jnt_bodyid()[joint] as usize)
            .collect();
        self.joint_tree_idx = self
            .joint_parent_body_idx
            .iter()
            .map(|&body| tree_of_body[body].unwrap_or_default())
            .collect();
        self.joint_types = model
            .jnt_type()
            .iter()
            .map(|kind| joint_type(*kind))
            .collect();
        self.joint_dof_start = (0..njnt)
            .map(|joint| model.jnt_dofadr()[joint] as usize)
            .collect();
        check_free_joints(model, "MJJointReactionForces");
    }

    /// Upstream `UpdateState`.
    fn publish(&self, data: &MjSceneData) {
        let constraint_forces = if data.nefc() > 0 {
            data.qfrc_constraint().to_vec()
        } else {
            vec![0.0; data.qvel().len()]
        };
        self.reaction_forces_out_msg.write(MjJointReactionsMsg {
            joint_tree_idx: self.joint_tree_idx.clone(),
            joint_parent_body_idx: self.joint_parent_body_idx.clone(),
            joint_types: self.joint_types.clone(),
            joint_dof_start: self.joint_dof_start.clone(),
            bias_forces: data.qfrc_bias().to_vec(),
            passive_forces: data.qfrc_passive().to_vec(),
            constraint_forces,
            applied_forces: data.qfrc_applied().to_vec(),
            actuator_forces: data.qfrc_actuator().to_vec(),
        });
    }
}

impl Module for MjJointReactionForces {
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

impl SceneModel for MjJointReactionForces {
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
    //! Port of upstream `test_MJJointReactionForces.py`.

    use std::f64::consts::PI;

    use super::*;
    use crate::messages::SingleActuatorMsg;
    use crate::mujoco_dynamics::tests::{run, sec2nano};

    const MASS2: f64 = 2.0;
    const LINK1_LENGTH: f64 = 0.5;
    const LINK2_COM_OFFSET: f64 = 0.3;
    const DAMPING: f64 = 2.5;
    const OFFSET: f64 = 0.15;

    fn fixed_base() -> String {
        format!(
            r#"<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="base">
      <geom type="box" size="0.2 0.2 0.2" mass="5" contype="0" conaffinity="0"/>
      <body name="link1" pos="0 0 0">
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <geom type="sphere" pos="0.05 0 0" size="0.01" mass="1" contype="0" conaffinity="0"/>
        <geom type="capsule" fromto="0 0 0  {LINK1_LENGTH} 0 0" size="0.03" mass="0.0" contype="0" conaffinity="0"/>
        <body name="link2" pos="{LINK1_LENGTH} 0 0">
          <joint name="j2" type="hinge" axis="0 0 1"/>
          <geom type="sphere" pos="{LINK2_COM_OFFSET} 0 0" size="0.01" mass="{MASS2}" contype="0" conaffinity="0"/>
          <geom type="capsule" fromto="0 0 0  0.2 0 0" size="0.02" mass="0.0" contype="0" conaffinity="0"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>"#
        )
    }

    fn damped_hub(name: &str, link: &str, joint: &str, position: &str) -> String {
        format!(
            r#"<body name="{name}" pos="{position}">
      <freejoint/>
      <geom type="box" size="0.5 0.3 0.2" mass="50" contype="0" conaffinity="0"/>
      <body name="{link}" pos="0 0 0">
        <joint name="{joint}" type="hinge" axis="0 0 1" damping="{DAMPING}"/>
        <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>
        <geom type="capsule" fromto="0 0 0  0.3 0 0" size="0.05" mass="0.0" contype="0" conaffinity="0"/>
      </body>
    </body>"#
        )
    }

    fn scene_xml(bodies: &str) -> String {
        format!(
            r#"<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    {bodies}
  </worldbody>
</mujoco>"#
        )
    }

    fn thruster() -> String {
        format!(
            r#"<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <geom type="box" size="0.5 0.3 0.2" mass="50"/>
      <site name="thruster1" pos="0 {OFFSET} 0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="F_thruster1" site="thruster1" gear="-1 0 0 0 0 0"/>
  </actuator>
</mujoco>"#
        )
    }

    /// `np.testing.assert_allclose(actual, truth, rtol=1e-8)` (absolute
    /// tolerance zero).
    fn assert_allclose(actual: &[f64], truth: &[f64], what: &str) {
        assert_eq!(actual.len(), truth.len(), "{what}");
        for (a, t) in actual.iter().zip(truth) {
            assert!(
                (a - t).abs() <= 1e-8 * t.abs(),
                "{what}: {actual:?} vs {truth:?}"
            );
        }
    }

    /// Upstream `test_MJJointReactionForces` over its four models: the joint
    /// layout and the five generalized-force vectors at the first sample.
    #[test]
    fn joint_layout_and_generalized_forces() {
        use MjJointType::{Free, Hinge};
        let thruster_force = 1.0;
        let joint_rate = 1.2;
        let (angle1, angle2, rate1, rate2) = (0.0, PI / 4.0, 1.0, -0.5);
        let cases = [
            (
                "Fixed Base",
                fixed_base(),
                vec![0, 0],
                vec![2, 3],
                vec![Hinge, Hinge],
                vec![0, 1],
            ),
            (
                "One SC",
                scene_xml(&damped_hub("hub", "link1", "j1", "0 0 0")),
                vec![0, 0],
                vec![1, 2],
                vec![Free, Hinge],
                vec![0, 6],
            ),
            (
                "Two SC",
                scene_xml(&format!(
                    "{}\n{}",
                    damped_hub("hub1", "link1", "j1", "0 0 0"),
                    damped_hub("hub2", "link2", "j2", "2 0 0")
                )),
                vec![0, 0, 1, 1],
                vec![1, 2, 3, 4],
                vec![Free, Hinge, Free, Hinge],
                vec![0, 6, 7, 13],
            ),
            (
                "Thruster",
                thruster(),
                vec![0],
                vec![1],
                vec![Free],
                vec![0],
            ),
        ];
        for (name, xml, tree, parent, types, dof_start) in cases {
            let mut scene = MjScene::new(&xml).expect("scene");
            scene.extra_eom_call = true;
            let command = Output::new(SingleActuatorMsg {
                input: thruster_force,
            });
            if name == "Thruster" {
                command.connect_to(&mut scene.single_actuator_mut("F_thruster1").actuator_in_msg);
            }
            let mut module = MjJointReactionForces::new(&scene);
            scene.init();
            module.init();
            match name {
                "Fixed Base" => {
                    scene
                        .body_mut("link1")
                        .set_scalar_joint_position("j1", angle1);
                    scene
                        .body_mut("link1")
                        .set_scalar_joint_velocity("j1", rate1);
                    scene
                        .body_mut("link2")
                        .set_scalar_joint_position("j2", angle2);
                    scene
                        .body_mut("link2")
                        .set_scalar_joint_velocity("j2", rate2);
                }
                "One SC" => scene
                    .body_mut("link1")
                    .set_scalar_joint_velocity("j1", joint_rate),
                "Two SC" => {
                    scene
                        .body_mut("link1")
                        .set_scalar_joint_velocity("j1", joint_rate);
                    scene
                        .body_mut("link2")
                        .set_scalar_joint_velocity("j2", joint_rate);
                }
                _ => {}
            }
            let mut first = None;
            run(&mut scene, sec2nano(0.01), sec2nano(0.01), |_, _| {
                module.update(&crate::mujoco_dynamics::tests::context(0));
                first.get_or_insert_with(|| module.reaction_forces_out_msg.read());
            });
            let msg = first.expect("samples");
            assert_eq!(msg.joint_tree_idx, tree, "{name}");
            assert_eq!(msg.joint_parent_body_idx, parent, "{name}");
            assert_eq!(msg.joint_types, types, "{name}");
            assert_eq!(msg.joint_dof_start, dof_start, "{name}");

            let nv = msg.bias_forces.len();
            let zeros = vec![0.0; nv];
            let (mut bias, mut passive, mut actuator) =
                (zeros.clone(), zeros.clone(), zeros.clone());
            match name {
                "Fixed Base" => {
                    let h = MASS2 * LINK1_LENGTH * LINK2_COM_OFFSET * angle2.sin();
                    bias = vec![
                        -h * (2.0 * rate1 * rate2 + rate2 * rate2),
                        h * rate1 * rate1,
                    ];
                }
                "One SC" => passive[6] = -DAMPING * joint_rate,
                "Two SC" => {
                    passive[6] = -DAMPING * joint_rate;
                    passive[13] = -DAMPING * joint_rate;
                }
                _ => actuator = vec![-thruster_force, 0.0, 0.0, 0.0, 0.0, OFFSET * thruster_force],
            }
            assert_allclose(&msg.bias_forces, &bias, &format!("{name} bias"));
            assert_allclose(&msg.passive_forces, &passive, &format!("{name} passive"));
            assert_allclose(
                &msg.constraint_forces,
                &zeros,
                &format!("{name} constraint"),
            );
            assert_allclose(&msg.applied_forces, &zeros, &format!("{name} applied"));
            assert_allclose(&msg.actuator_forces, &actuator, &format!("{name} actuator"));
        }
    }
}
