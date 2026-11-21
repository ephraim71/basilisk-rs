//! Center of mass of a whole scene: port of upstream Basilisk
//! `simulation/mujocoDynamics/MJSystemCoM`.
//!
//! The mass-weighted mean of the body centers of mass,
//! `r_C = Σ mᵢ rᵢ / M`, and its time derivative: the mass-weighted body
//! center-of-mass velocities (from MuJoCo's center-of-mass Jacobians),
//! plus, for bodies whose mass changes, the quotient-rule term
//! `Σ ṁᵢ (rᵢ - r_C) / M` with the mass rates of the bodies'
//! [`derivative_mass_properties_in_msg`](super::MjBody::derivative_mass_properties_in_msg).
//!
//! Upstream publishes an `SCStatesMsg` with only `r_CN_N` and `v_CN_N` set;
//! basilisk-rs's [`SpacecraftStateMsg`] has no separate center-of-mass
//! fields, so here the system center of mass is the message's
//! `position_m` and `velocity_mps`, the other fields zero. Upstream's
//! C-wrapped copy of the output is not needed: a port serves every
//! consumer.
//!
//! The module reads the scene either from the simulation's schedule (a
//! [`Module`], through an [`MjSceneHandle`]) or from inside the scene's
//! dynamics task (a [`SceneModel`]), as upstream sets `module.scene` and
//! adds it to either.

use nalgebra::{DVector, Vector3};

use super::scene::{MjScene, MjSceneHandle};
use super::task::{MjSceneData, SceneModel, TaskContext};
use crate::messages::{Input, Output, SpacecraftMassPropsMsg, SpacecraftStateMsg};
use crate::{Module, SimulationContext};

/// Upstream `MJSystemCoM`.
#[derive(Debug)]
pub struct MjSystemCom {
    /// Upstream `ModelTag`.
    pub name: String,
    /// Upstream `comStatesOutMsg`: the system center of mass in
    /// `position_m` (`r_CN_N`) and `velocity_mps` (`v_CN_N`).
    pub com_states_out_msg: Output<SpacecraftStateMsg>,
    scene: MjSceneHandle,
    mass_rates: Vec<(usize, Input<SpacecraftMassPropsMsg>)>,
}

impl MjSystemCom {
    /// The module for `scene` (upstream `module.scene = scene`); the bodies'
    /// mass-rate inputs are shared with it, so later connections count.
    pub fn new(scene: &MjScene) -> Self {
        Self {
            name: "MJSystemCoM".to_string(),
            com_states_out_msg: Output::default(),
            scene: scene.handle(),
            mass_rates: scene
                .bodies()
                .iter()
                .map(|body| (body.id(), body.derivative_mass_properties_in_msg.clone()))
                .collect(),
        }
    }

    /// Upstream `UpdateState`.
    fn publish(&self, data: &MjSceneData) {
        let model = data.model();
        let nbody = model.ffi().nbody as usize;
        let masses = model.body_mass();
        let qvel = DVector::from_column_slice(data.qvel());
        let mut total = 0.0;
        let mut position = Vector3::zeros();
        let mut velocity = Vector3::zeros();
        for (body, &mass) in masses.iter().enumerate().take(nbody).skip(1) {
            if mass <= 0.0 {
                continue;
            }
            total += mass;
            position += mass * Vector3::from(data.xipos()[body]);
        }
        if !qvel.is_empty() && total > 0.0 {
            let nv = qvel.len();
            for (body, &mass) in masses.iter().enumerate().take(nbody).skip(1) {
                if mass <= 0.0 {
                    continue;
                }
                let (jacp, _) = data.jac_body_com(true, false, body);
                let jacobian = nalgebra::DMatrix::from_row_slice(3, nv, &jacp);
                let body_velocity = jacobian * &qvel;
                velocity +=
                    mass * Vector3::new(body_velocity[0], body_velocity[1], body_velocity[2]);
            }
        }
        if total > 0.0 {
            let center = position / total;
            for (body, mass_rate) in &self.mass_rates {
                if masses[*body] <= 0.0 || !mass_rate.is_connected() {
                    continue;
                }
                velocity +=
                    mass_rate.read().mass_kg * (Vector3::from(data.xipos()[*body]) - center);
            }
            position /= total;
            velocity /= total;
        }
        self.com_states_out_msg.write(SpacecraftStateMsg {
            position_m: position,
            velocity_mps: velocity,
            ..SpacecraftStateMsg::default()
        });
    }
}

impl Module for MjSystemCom {
    fn init(&mut self) {}

    fn update(&mut self, _context: &SimulationContext) {
        let scene = self.scene.clone();
        self.publish(&scene.lock());
    }
}

impl SceneModel for MjSystemCom {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        self.publish(context.scene().data());
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_MJSystemCoM.py` (its C-message comparison has
    //! no counterpart).

    use super::*;
    use crate::mujoco_dynamics::tests::{run, sec2nano};

    const HUB_MASS: f64 = 50.0;
    const ARM_MASS: f64 = 10.0;
    const SX: f64 = 0.5;
    const L: f64 = 2.0;
    const THETA_DEG: f64 = 30.0;

    fn xml_single() -> String {
        format!(
            r#"<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" quat="1 0 0 0" mass="{HUB_MASS}" diaginertia="1 1 1"/>
      <geom type="box" size="{SX} 0.3 0.2" rgba="0.8 0.2 0.2 0.4"/>
    </body>
  </worldbody>
</mujoco>"#
        )
    }

    fn xml_straight() -> String {
        let half = L / 2.0;
        format!(
            r#"<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" quat="1 0 0 0" mass="{HUB_MASS}" diaginertia="10 10 10"/>
      <geom type="box" size="{SX} 0.3 0.2"/>
      <body name="arm_plus" pos="{SX} 0 0">
        <inertial pos="{half} 0 0" quat="1 0 0 0" mass="{ARM_MASS}" diaginertia="5 5 5"/>
        <geom type="capsule" fromto="0 0 0 {L} 0 0" size="0.05"/>
      </body>
      <body name="arm_minus" pos="-{SX} 0 0">
        <inertial pos="-{half} 0 0" quat="1 0 0 0" mass="{ARM_MASS}" diaginertia="5 5 5"/>
        <geom type="capsule" fromto="0 0 0 -{L} 0 0" size="0.05"/>
      </body>
    </body>
  </worldbody>
</mujoco>"#
        )
    }

    fn xml_angled() -> String {
        let (c, s) = (THETA_DEG.to_radians().cos(), THETA_DEG.to_radians().sin());
        let (hc, hs) = (L / 2.0 * c, L / 2.0 * s);
        let (lc, ls) = (L * c, L * s);
        format!(
            r#"<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" quat="1 0 0 0" mass="{HUB_MASS}" diaginertia="10 10 10"/>
      <geom type="box" size="{SX} 0.3 0.2"/>
      <body name="arm_plus" pos="{SX} 0 0">
        <inertial pos="{hc} {hs} 0" quat="1 0 0 0" mass="{ARM_MASS}" diaginertia="5 5 5"/>
        <geom type="capsule" fromto="0 0 0 {lc} {ls} 0" size="0.05"/>
      </body>
      <body name="arm_minus" pos="-{SX} 0 0">
        <inertial pos="{} {hs} 0" quat="1 0 0 0" mass="{ARM_MASS}" diaginertia="5 5 5"/>
        <geom type="capsule" fromto="0 0 0 {} {ls} 0" size="0.05"/>
      </body>
    </body>
  </worldbody>
</mujoco>"#,
            -hc, -lc
        )
    }

    fn xml_variable_mass(rotated: bool) -> String {
        let pose = if rotated {
            r#"pos="4 -2 1" quat="0.9238795325112867 0 0 0.3826834323650898""#
        } else {
            ""
        };
        format!(
            r#"<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub" {pose}>
      <freejoint/>
      <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>
      <body name="tank" pos="3 0 0">
        <inertial pos="0 0 0" mass="2" diaginertia="0.1 0.1 0.1"/>
      </body>
    </body>
  </worldbody>
</mujoco>"#
        )
    }

    /// Upstream `_expected_com`.
    fn expected_com(model: &str, r: Vector3<f64>, v: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
        if model != "angled" {
            return (r, v);
        }
        let (c, s) = (THETA_DEG.to_radians().cos(), THETA_DEG.to_radians().sin());
        let r_plus = Vector3::new(SX, 0.0, 0.0) + Vector3::new(L / 2.0 * c, L / 2.0 * s, 0.0);
        let r_minus = Vector3::new(-SX, 0.0, 0.0) + Vector3::new(-L / 2.0 * c, L / 2.0 * s, 0.0);
        let total = HUB_MASS + 2.0 * ARM_MASS;
        (r + (ARM_MASS * r_plus + ARM_MASS * r_minus) / total, v)
    }

    /// Upstream `test_MJSystemCoM`, over the three geometries, moving or
    /// not, displaced or not; the module runs in the dynamics task.
    #[test]
    fn center_of_mass_of_rigid_assemblies() {
        for model in ["single", "straight", "angled"] {
            for moving in [true, false] {
                for displaced in [true, false] {
                    let xml = match model {
                        "single" => xml_single(),
                        "straight" => xml_straight(),
                        _ => xml_angled(),
                    };
                    let r0 = if displaced {
                        Vector3::new(1234.5, -678.9, 42.0)
                    } else {
                        Vector3::zeros()
                    };
                    let v0 = if moving {
                        Vector3::new(0.12, -0.07, 0.03)
                    } else {
                        Vector3::zeros()
                    };
                    let mut scene = MjScene::new(&xml).expect("scene");
                    scene.extra_eom_call = true;
                    let module = MjSystemCom::new(&scene);
                    let out = module.com_states_out_msg.clone();
                    scene.add_model_to_dynamics_task(module, -1);
                    scene.init();
                    scene.body_mut("hub").set_position(r0);
                    scene.body_mut("hub").set_velocity(v0);
                    run(&mut scene, sec2nano(0.1), sec2nano(0.2), |_, _| {});

                    let (r_truth, v_truth) = expected_com(model, r0, v0);
                    let r_truth = r_truth + v_truth * 0.2;
                    let state = out.read();
                    let context = format!("{model} moving {moving} displaced {displaced}");
                    for axis in 0..3 {
                        // `unitTestSupport.compareArrayND` with accuracy 1e-12.
                        assert!(
                            (state.position_m[axis] - r_truth[axis]).abs() <= 1e-12,
                            "{context}: {}",
                            state.position_m
                        );
                        assert!(
                            (state.velocity_mps[axis] - v_truth[axis]).abs() <= 1e-12,
                            "{context}: {}",
                            state.velocity_mps
                        );
                    }
                }
            }
        }
    }

    /// Upstream `test_variable_mass_velocity_is_com_position_derivative`: a
    /// depleting tank adds the quotient-rule term to the velocity, in
    /// inertial components also for a rotated hub.
    #[test]
    fn variable_mass_velocity_is_the_com_position_derivative() {
        for rotated in [false, true] {
            let (dt, tf, mass_rate) = (1.0e-3, 2.0e-3, -0.5);
            let root_position = if rotated {
                Vector3::new(4.0, -2.0, 1.0)
            } else {
                Vector3::zeros()
            };
            let tank_offset = if rotated {
                Vector3::new(3.0 / 2.0_f64.sqrt(), 3.0 / 2.0_f64.sqrt(), 0.0)
            } else {
                Vector3::new(3.0, 0.0, 0.0)
            };
            let root_velocity = Vector3::new(1.0, -0.5, 0.25);

            let mut scene = MjScene::new(&xml_variable_mass(rotated)).expect("scene");
            scene.extra_eom_call = true;
            let rate = Output::new(SpacecraftMassPropsMsg {
                mass_kg: mass_rate,
                ..SpacecraftMassPropsMsg::default()
            });
            rate.connect_to(&mut scene.body_mut("tank").derivative_mass_properties_in_msg);
            let module = MjSystemCom::new(&scene);
            let out = module.com_states_out_msg.clone();
            scene.add_model_to_dynamics_task(module, -1);
            scene.init();
            scene.body_mut("hub").set_velocity(root_velocity);

            let mut samples = Vec::new();
            run(&mut scene, sec2nano(dt), sec2nano(tf), |_, now| {
                samples.push((now as f64 * 1e-9, out.read()));
            });
            for (t, state) in samples {
                let tank_mass = 2.0 + mass_rate * t;
                let expected_position = root_position
                    + t * root_velocity
                    + tank_mass / (10.0 + tank_mass) * tank_offset;
                let expected_velocity =
                    root_velocity + 10.0 * mass_rate / (10.0 + tank_mass).powi(2) * tank_offset;
                assert!(
                    (state.position_m - expected_position).abs().max() <= 2.0e-12,
                    "rotated {rotated} t {t}"
                );
                assert!(
                    (state.velocity_mps - expected_velocity).abs().max() <= 2.0e-12,
                    "rotated {rotated} t {t}"
                );
            }
        }
    }

    /// Run from the simulation's schedule (through the scene handle), the
    /// module reads the scene's last state.
    #[test]
    fn runs_outside_the_dynamics_task() {
        let mut scene = MjScene::new(&xml_angled()).expect("scene");
        let mut module = MjSystemCom::new(&scene);
        scene.init();
        scene
            .body_mut("hub")
            .set_position(Vector3::new(1.0, 2.0, 3.0));
        scene.update(&crate::mujoco_dynamics::tests::context(0));
        module.init();
        module.update(&crate::mujoco_dynamics::tests::context(0));
        let (expected, _) = expected_com("angled", Vector3::new(1.0, 2.0, 3.0), Vector3::zeros());
        assert!((module.com_states_out_msg.read().position_m - expected).norm() < 1e-12);
    }
}
