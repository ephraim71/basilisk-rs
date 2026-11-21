//! The integrated state of a scene and its update rule: port of upstream
//! Basilisk `MJQuaternionStatePolicy` (`MJNativeQuaternionStatePolicy`,
//! `MJHighOrderQuaternionStatePolicy`) and of the joint, mass, activation
//! and task-model state records `MJScene` registers.
//!
//! The joint positions `qpos` hold scalar-first attitude quaternions for the
//! free and ball joints, so a stage state is not `x + h·Σ aⱼ kⱼ` for them.
//! Upstream's two policies, chosen per scene before the first reset:
//!
//! - **native** (default): the `qpos` rate is `qvel` (the joint-space
//!   tangent), and a stage state applies the weighted rate once through
//!   MuJoCo's `mj_integratePos`: linear for translations and scalar joints, an
//!   exponential map (`mju_quatIntegrate`) for each quaternion. This is
//!   MuJoCo's own stepper, second-order accurate on SO(3) whatever the
//!   integrator.
//! - **high order**: each quaternion has the four-component rate
//!   `q̇ = ½ q ⊗ (0, ω)`, a stage adds the weighted rate linearly and then
//!   normalizes the quaternion, so the attitude inherits the integrator's
//!   order.
//!
//! Diffusion columns are expressed as derivatives and applied the same way;
//! only task-model states carry noise, so the joint part of a column is
//! always zero.

use mujoco_rs::wrappers::fun::utility::mju_quat_integrate;
use nalgebra::DVector;

use super::body::{JointSlot, MjJointKind};

/// The integrated state of an [`MjScene`](super::MjScene) (upstream: the
/// joint records, `mujocoMass`, `mujocoAct` and the task-model states of its
/// `DynParamManager`).
#[derive(Clone, Debug, PartialEq)]
pub struct MjSceneState {
    /// Joint positions in MuJoCo `qpos` layout (`nq`).
    pub qpos: DVector<f64>,
    /// Joint velocities in MuJoCo `qvel` layout (`nv`).
    pub qvel: DVector<f64>,
    /// Actuator activations in MuJoCo `act` layout (`na`).
    pub act: DVector<f64>,
    /// Body masses \[kg\], one per MuJoCo body, the world body (always zero)
    /// included.
    pub mass: DVector<f64>,
    /// The task-model states, model by model in task order.
    pub models: DVector<f64>,
}

/// Rate of an [`MjSceneState`].
#[derive(Clone, Debug, PartialEq)]
pub struct MjSceneDerivative {
    /// Rate of `qpos`: the joint-space tangent (`nv`) under the native
    /// policy, the `qpos`-shaped rate (`nq`) under the high-order one.
    pub qpos: DVector<f64>,
    /// Joint accelerations `qacc` (`nv`).
    pub qvel: DVector<f64>,
    /// Activation rates `act_dot` (`na`).
    pub act: DVector<f64>,
    /// Body mass rates \[kg/s\].
    pub mass: DVector<f64>,
    /// Task-model state rates.
    pub models: DVector<f64>,
}

/// How `qpos` advances (upstream: the quaternion state policy of each
/// joint record, the Euclidean policy elsewhere).
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct QposPolicy {
    pub(crate) joints: Vec<JointSlot>,
    pub(crate) high_order: bool,
}

impl QposPolicy {
    /// Length of the `qpos` rate.
    pub(crate) fn rate_len(&self, nq: usize, nv: usize) -> usize {
        if self.high_order { nq } else { nv }
    }

    /// The `qpos` rate of a state with `qvel` (upstream
    /// `setJointPositionDerivativesFromMujoco`, or the bulk `qvel` copy of the
    /// native policy).
    pub(crate) fn rate(&self, qpos: &DVector<f64>, qvel: &DVector<f64>) -> DVector<f64> {
        if !self.high_order {
            return qvel.clone();
        }
        let mut rate = DVector::zeros(qpos.len());
        for joint in &self.joints {
            let (p, v) = (joint.qpos_adr, joint.qvel_adr);
            match joint.kind {
                MjJointKind::Free => {
                    for i in 0..3 {
                        rate[p + i] = qvel[v + i];
                    }
                    let q = quaternion_rate(
                        &qpos.as_slice()[p + 3..p + 7],
                        &qvel.as_slice()[v + 3..v + 6],
                    );
                    rate.as_mut_slice()[p + 3..p + 7].copy_from_slice(&q);
                }
                MjJointKind::Ball => {
                    let q = quaternion_rate(&qpos.as_slice()[p..p + 4], &qvel.as_slice()[v..v + 3]);
                    rate.as_mut_slice()[p..p + 4].copy_from_slice(&q);
                }
                MjJointKind::Slide | MjJointKind::Hinge => rate[p] = qvel[v],
            }
        }
        rate
    }

    /// `qpos` advanced by `step` along `rate` (upstream
    /// `buildDriftCandidate` and `applyNoiseIncrement`).
    pub(crate) fn advance(
        &self,
        qpos: &DVector<f64>,
        rate: &DVector<f64>,
        step: f64,
    ) -> DVector<f64> {
        let mut next = qpos.clone();
        if self.high_order {
            next.axpy(step, rate, 1.0);
            for joint in &self.joints {
                if let Some(offset) = joint.kind.quaternion_offset() {
                    let start = joint.qpos_adr + offset;
                    normalize(&mut next.as_mut_slice()[start..start + 4]);
                }
            }
        } else {
            integrate_pos(&self.joints, next.as_mut_slice(), rate.as_slice(), step);
        }
        next
    }

    /// The `qpos` entries that are free-joint translations, and the `qvel`
    /// entries that are free-joint velocities: the records upstream gives a
    /// zero relative tolerance under adaptive integration.
    pub(crate) fn free_translation_entries(&self) -> (Vec<usize>, Vec<usize>) {
        let mut qpos = Vec::new();
        let mut qvel = Vec::new();
        for joint in &self.joints {
            if joint.kind == MjJointKind::Free {
                qpos.extend(joint.qpos_adr..joint.qpos_adr + 3);
                qvel.extend(joint.qvel_adr..joint.qvel_adr + 3);
            }
        }
        (qpos, qvel)
    }
}

/// MuJoCo's `mj_integratePos`: translations and scalar joints advance
/// linearly, quaternions through `mju_quatIntegrate`.
pub(crate) fn integrate_pos(joints: &[JointSlot], qpos: &mut [f64], qvel: &[f64], dt: f64) {
    for joint in joints {
        let (mut p, mut v) = (joint.qpos_adr, joint.qvel_adr);
        match joint.kind {
            MjJointKind::Free | MjJointKind::Ball => {
                if joint.kind == MjJointKind::Free {
                    for i in 0..3 {
                        qpos[p + i] += dt * qvel[v + i];
                    }
                    p += 3;
                    v += 3;
                }
                let mut quat = [qpos[p], qpos[p + 1], qpos[p + 2], qpos[p + 3]];
                let vel = [qvel[v], qvel[v + 1], qvel[v + 2]];
                mju_quat_integrate(&mut quat, &vel, dt);
                qpos[p..p + 4].copy_from_slice(&quat);
            }
            MjJointKind::Slide | MjJointKind::Hinge => qpos[p] += dt * qvel[v],
        }
    }
}

/// Upstream `quaternionRate`: `½ q ⊗ (0, ω)` for a scalar-first `q` and the
/// body rate `ω`.
fn quaternion_rate(q: &[f64], omega: &[f64]) -> [f64; 4] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let (wx, wy, wz) = (omega[0], omega[1], omega[2]);
    [
        0.5 * (-x * wx - y * wy - z * wz),
        0.5 * (w * wx + y * wz - z * wy),
        0.5 * (w * wy - x * wz + z * wx),
        0.5 * (w * wz + x * wy - y * wx),
    ]
}

/// Upstream `normalizeQuaternion`: divides by the norm when it is positive.
fn normalize(q: &mut [f64]) {
    let norm = q.iter().map(|value| value * value).sum::<f64>().sqrt();
    if norm > 0.0 {
        for value in q {
            *value /= norm;
        }
    }
}
