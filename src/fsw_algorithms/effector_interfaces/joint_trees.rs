//! Multibody-scene bookkeeping shared by the joint controllers
//! [`super::hinged_joint_array_motor`], [`super::joint_motion_compensator`]
//! and [`super::thr_joint_compensation`] (upstream repeats it in each
//! module): the kinematic trees of the scene, the non-actuator generalized
//! forces and the mass matrix of one tree.
//!
//! The scene is a set of kinematic trees, one per spacecraft, each a
//! free-floating hub (a free joint, six degrees of freedom) carrying hinged
//! joints only. The hinges are numbered across the whole scene in joint
//! order; that number is the hinge's index among the module's joint ports.

use std::collections::BTreeMap;
use std::fmt;

use nalgebra::{DMatrix, DVector};

use crate::messages::{MjJointReactionsMsg, MjJointType};

/// One kinematic tree of the scene (upstream `TreeInfo`).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct KinematicTree {
    /// Upstream `freeJointIdx`: scene index of the tree's free joint.
    pub(crate) free_joint: usize,
    /// Upstream `hingeJointIdxs`: scene index of each of the tree's hinges.
    pub(crate) hinge_joints: Vec<usize>,
    /// Upstream `hingeGlobalIdxs`: scene-wide hinge number of each of the
    /// tree's hinges.
    pub(crate) hinge_global: Vec<usize>,
}

/// The kinematic trees of a scene by tree index (upstream `treeMap`).
pub(crate) type KinematicTrees = BTreeMap<usize, KinematicTree>;

/// A scene the joint controllers cannot work with. Upstream raises a
/// `BasiliskError` for the topology errors and reads out of bounds for the
/// shape errors; here each stops the update before the outputs are written.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SceneError {
    /// The joint lists of the reactions message differ in length.
    JointListLengths,
    /// The generalized force vectors of the reactions message differ in
    /// length.
    ForceLengths,
    /// The mass matrix is not square with one row per degree of freedom.
    MassMatrixShape {
        rows: usize,
        columns: usize,
        dofs: usize,
    },
    /// A joint's degrees of freedom run past those of the scene.
    DofOutOfRange { joint: usize },
    /// The first joint of a tree is not a free joint.
    FirstJointNotFree { tree: usize },
    /// A later joint of a tree is not a hinge.
    JointNotHinge { joint: usize, tree: usize },
    /// The scene has a different number of hinges than the module has joint
    /// ports.
    HingeCountMismatch { configured: usize, found: usize },
    /// A mass sub-matrix the controller solves with is singular.
    SingularMassMatrix,
}

impl fmt::Display for SceneError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::JointListLengths => write!(
                f,
                "the joint tree, type and degree-of-freedom lists differ in length"
            ),
            Self::ForceLengths => write!(f, "the generalized force vectors differ in length"),
            Self::MassMatrixShape {
                rows,
                columns,
                dofs,
            } => write!(
                f,
                "the mass matrix is {rows} x {columns}, not {dofs} x {dofs}"
            ),
            Self::DofOutOfRange { joint } => {
                write!(
                    f,
                    "the degrees of freedom of joint {joint} are out of range"
                )
            }
            Self::FirstJointNotFree { tree } => {
                write!(
                    f,
                    "first joint in kinematic tree {tree} is not a free joint"
                )
            }
            Self::JointNotHinge { joint, tree } => {
                write!(
                    f,
                    "joint {joint} in kinematic tree {tree} is not a hinged joint"
                )
            }
            Self::HingeCountMismatch { configured, found } => write!(
                f,
                "num_hinged_joints ({configured}) does not match the number of hinge joints found \
                 in the kinematic trees ({found})"
            ),
            Self::SingularMassMatrix => write!(f, "a mass sub-matrix is singular"),
        }
    }
}

/// The kinematic trees of the scene: each tree's first joint must be free
/// and its later ones hinges, and the scene must hold `num_hinged_joints`
/// hinges in all.
pub(crate) fn kinematic_trees(
    reactions: &MjJointReactionsMsg,
    num_hinged_joints: usize,
) -> Result<KinematicTrees, SceneError> {
    let num_joints = reactions.joint_tree_idx.len();
    if reactions.joint_types.len() != num_joints || reactions.joint_dof_start.len() != num_joints {
        return Err(SceneError::JointListLengths);
    }
    let mut trees = KinematicTrees::new();
    let mut next_hinge = 0;
    for (joint, (&tree, &joint_type)) in reactions
        .joint_tree_idx
        .iter()
        .zip(&reactions.joint_types)
        .enumerate()
    {
        match trees.get_mut(&tree) {
            None => {
                if joint_type != MjJointType::Free {
                    return Err(SceneError::FirstJointNotFree { tree });
                }
                trees.insert(
                    tree,
                    KinematicTree {
                        free_joint: joint,
                        ..Default::default()
                    },
                );
            }
            Some(info) => {
                if joint_type != MjJointType::Hinge {
                    return Err(SceneError::JointNotHinge { joint, tree });
                }
                info.hinge_joints.push(joint);
                info.hinge_global.push(next_hinge);
                next_hinge += 1;
            }
        }
    }
    if next_hinge != num_hinged_joints {
        return Err(SceneError::HingeCountMismatch {
            configured: num_hinged_joints,
            found: next_hinge,
        });
    }
    Ok(trees)
}

/// The non-actuator generalized force on every degree of freedom,
/// `passive + constraint + applied - bias`.
pub(crate) fn non_actuator_forces(
    reactions: &MjJointReactionsMsg,
) -> Result<DVector<f64>, SceneError> {
    let dofs = reactions.bias_forces.len();
    if [
        &reactions.passive_forces,
        &reactions.constraint_forces,
        &reactions.applied_forces,
    ]
    .iter()
    .any(|forces| forces.len() != dofs)
    {
        return Err(SceneError::ForceLengths);
    }
    Ok(DVector::from_iterator(
        dofs,
        (0..dofs).map(|k| {
            reactions.passive_forces[k]
                + reactions.constraint_forces[k]
                + reactions.applied_forces[k]
                - reactions.bias_forces[k]
        }),
    ))
}

/// Checks that the mass matrix has one row and column per degree of
/// freedom.
pub(crate) fn check_mass_matrix(mass_matrix: &DMatrix<f64>, dofs: usize) -> Result<(), SceneError> {
    if mass_matrix.shape() == (dofs, dofs) {
        Ok(())
    } else {
        Err(SceneError::MassMatrixShape {
            rows: mass_matrix.nrows(),
            columns: mass_matrix.ncols(),
            dofs,
        })
    }
}

/// The scene degrees of freedom of a tree: the first `num_base_dofs` of its
/// free joint (3 for the hub translation, 6 with its rotation), then the one
/// of each hinge. Every index is below `dofs`.
pub(crate) fn tree_dofs(
    reactions: &MjJointReactionsMsg,
    tree: &KinematicTree,
    num_base_dofs: usize,
    dofs: usize,
) -> Result<Vec<usize>, SceneError> {
    let free_start = reactions.joint_dof_start[tree.free_joint];
    if free_start + num_base_dofs > dofs {
        return Err(SceneError::DofOutOfRange {
            joint: tree.free_joint,
        });
    }
    let mut indices: Vec<usize> = (free_start..free_start + num_base_dofs).collect();
    for &joint in &tree.hinge_joints {
        let hinge_dof = reactions.joint_dof_start[joint];
        if hinge_dof >= dofs {
            return Err(SceneError::DofOutOfRange { joint });
        }
        indices.push(hinge_dof);
    }
    Ok(indices)
}

/// The rows and columns `indices` of `matrix` (upstream `Mfull`).
pub(crate) fn sub_matrix(matrix: &DMatrix<f64>, indices: &[usize]) -> DMatrix<f64> {
    matrix.select_rows(indices).select_columns(indices)
}

/// The entries `indices` of `vector`.
pub(crate) fn sub_vector(vector: &DVector<f64>, indices: &[usize]) -> DVector<f64> {
    DVector::from_iterator(indices.len(), indices.iter().map(|&i| vector[i]))
}

/// `[A]^-1 [B]` for a symmetric `[A]`, by Cholesky factorization when `[A]`
/// is positive definite (upstream uses Eigen's LDLT), otherwise by LU.
pub(crate) fn solve_symmetric(
    a: &DMatrix<f64>,
    b: &DMatrix<f64>,
) -> Result<DMatrix<f64>, SceneError> {
    match a.clone().cholesky() {
        Some(cholesky) => Ok(cholesky.solve(b)),
        None => a
            .clone()
            .lu()
            .solve(b)
            .ok_or(SceneError::SingularMassMatrix),
    }
}

/// [`solve_symmetric`] for one right-hand side.
pub(crate) fn solve_symmetric_vector(
    a: &DMatrix<f64>,
    b: &DVector<f64>,
) -> Result<DVector<f64>, SceneError> {
    let solution = solve_symmetric(a, &DMatrix::from_column_slice(b.len(), 1, b.as_slice()))?;
    Ok(DVector::from_column_slice(solution.as_slice()))
}

/// Upstream's optional torque limit: clamps each entry of `torques` to
/// `±u_max[i]`.
pub(crate) fn saturate(torques: &mut DVector<f64>, u_max: &[f64]) {
    for (torque, &limit) in torques.iter_mut().zip(u_max) {
        *torque = (-limit).max(limit.min(*torque));
    }
}

#[cfg(test)]
pub(crate) mod test_support {
    //! The scenes of the upstream joint-controller pytests.

    use nalgebra::{DMatrix, Matrix3, Vector3};
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use rand_distr::{Distribution, StandardNormal};

    use crate::messages::{MjJointReactionsMsg, MjJointType};

    /// The pytests' block-diagonal scene mass matrix: per spacecraft,
    /// `A A^T + 1e-3 I` for a standard-normal `A`, with the hub translation
    /// block set to `10 I` and the hub rotation block to `diag(8, 6, 4)`.
    ///
    /// The pytests draw `A` from `np.random.default_rng(12345)`; NumPy's
    /// generator is not reproduced, so the entries differ, but the truth is
    /// recomputed from the same matrix.
    pub(crate) fn scene_mass_matrix(
        num_spacecraft: usize,
        hinges_per_spacecraft: usize,
    ) -> DMatrix<f64> {
        let mut rng = StdRng::seed_from_u64(12345);
        let dofs = 6 + hinges_per_spacecraft;
        let mut mass = DMatrix::zeros(dofs * num_spacecraft, dofs * num_spacecraft);
        for sc in 0..num_spacecraft {
            let a = DMatrix::from_fn(dofs, dofs, |_, _| {
                <StandardNormal as Distribution<f64>>::sample(&StandardNormal, &mut rng)
            });
            let mut spd: DMatrix<f64> = &a * a.transpose() + 1e-3 * DMatrix::identity(dofs, dofs);
            spd.view_mut((0, 0), (3, 3))
                .copy_from(&(10.0 * Matrix3::identity()));
            spd.view_mut((3, 3), (3, 3))
                .copy_from(&Matrix3::from_diagonal(&Vector3::new(8.0, 6.0, 4.0)));
            mass.view_mut((sc * dofs, sc * dofs), (dofs, dofs))
                .copy_from(&spd);
        }
        mass
    }

    /// The pytests' joint reactions: per spacecraft a free joint and
    /// `hinges_per_spacecraft` hinges, in spacecraft order, with the given
    /// bias forces and zero passive, constraint and applied forces.
    pub(crate) fn scene_reactions(
        num_spacecraft: usize,
        hinges_per_spacecraft: usize,
        bias_forces: Vec<f64>,
    ) -> MjJointReactionsMsg {
        let mut reactions = MjJointReactionsMsg::default();
        let mut current_dof = 0;
        for sc in 0..num_spacecraft {
            reactions
                .joint_tree_idx
                .extend(std::iter::repeat_n(sc, hinges_per_spacecraft + 1));
            reactions.joint_types.push(MjJointType::Free);
            reactions.joint_types.extend(std::iter::repeat_n(
                MjJointType::Hinge,
                hinges_per_spacecraft,
            ));
            reactions.joint_dof_start.push(current_dof);
            current_dof += 6;
            for _ in 0..hinges_per_spacecraft {
                reactions.joint_dof_start.push(current_dof);
                current_dof += 1;
            }
        }
        reactions.joint_parent_body_idx = vec![0; reactions.joint_tree_idx.len()];
        let dofs = bias_forces.len();
        reactions.bias_forces = bias_forces;
        reactions.passive_forces = vec![0.0; dofs];
        reactions.constraint_forces = vec![0.0; dofs];
        reactions.applied_forces = vec![0.0; dofs];
        reactions.actuator_forces = vec![0.0; dofs];
        reactions
    }

    /// `np.testing.assert_allclose(actual, expected, rtol, atol)`.
    pub(crate) fn assert_allclose(
        actual: &[f64],
        expected: &[f64],
        rtol: f64,
        atol: f64,
        what: &str,
    ) {
        assert_eq!(actual.len(), expected.len(), "{what}");
        for (index, (a, e)) in actual.iter().zip(expected).enumerate() {
            assert!(
                (a - e).abs() <= atol + rtol * e.abs(),
                "{what}, entry {index}: {a:e} vs {e:e}"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{DMatrix, DVector};

    use super::test_support::scene_reactions;
    use super::{
        SceneError, check_mass_matrix, kinematic_trees, non_actuator_forces, saturate, tree_dofs,
    };
    use crate::messages::MjJointType;

    #[test]
    fn numbers_the_hinges_across_the_scene() {
        let reactions = scene_reactions(2, 3, vec![0.0; 18]);
        let trees = kinematic_trees(&reactions, 6).unwrap();
        assert_eq!(trees.len(), 2);
        assert_eq!(trees[&0].free_joint, 0);
        assert_eq!(trees[&0].hinge_joints, [1, 2, 3]);
        assert_eq!(trees[&1].free_joint, 4);
        assert_eq!(trees[&1].hinge_joints, [5, 6, 7]);
        assert_eq!(trees[&1].hinge_global, [3, 4, 5]);
        assert_eq!(
            tree_dofs(&reactions, &trees[&1], 6, 18).unwrap(),
            [9, 10, 11, 12, 13, 14, 15, 16, 17]
        );
        assert_eq!(
            tree_dofs(&reactions, &trees[&1], 3, 18).unwrap(),
            [9, 10, 11, 15, 16, 17]
        );
        assert_eq!(
            tree_dofs(&reactions, &trees[&1], 6, 17),
            Err(SceneError::DofOutOfRange { joint: 7 })
        );
    }

    #[test]
    fn rejects_scenes_that_are_not_hubs_with_hinges() {
        let mut reactions = scene_reactions(1, 2, vec![0.0; 8]);
        assert_eq!(
            kinematic_trees(&reactions, 3),
            Err(SceneError::HingeCountMismatch {
                configured: 3,
                found: 2
            })
        );
        reactions.joint_types[2] = MjJointType::Slide;
        assert_eq!(
            kinematic_trees(&reactions, 2),
            Err(SceneError::JointNotHinge { joint: 2, tree: 0 })
        );
        reactions.joint_types[0] = MjJointType::Ball;
        assert_eq!(
            kinematic_trees(&reactions, 2),
            Err(SceneError::FirstJointNotFree { tree: 0 })
        );
        reactions.joint_types.pop();
        assert_eq!(
            kinematic_trees(&reactions, 2),
            Err(SceneError::JointListLengths)
        );
    }

    #[test]
    fn adds_the_non_actuator_forces_and_checks_shapes() {
        let mut reactions = scene_reactions(1, 0, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        reactions.passive_forces[0] = 10.0;
        reactions.constraint_forces[1] = 20.0;
        reactions.applied_forces[2] = 30.0;
        assert_eq!(
            non_actuator_forces(&reactions).unwrap(),
            DVector::from_vec(vec![9.0, 18.0, 27.0, -4.0, -5.0, -6.0])
        );
        reactions.applied_forces.pop();
        assert_eq!(
            non_actuator_forces(&reactions),
            Err(SceneError::ForceLengths)
        );
        assert!(check_mass_matrix(&DMatrix::zeros(6, 6), 6).is_ok());
        assert!(check_mass_matrix(&DMatrix::zeros(6, 5), 6).is_err());
    }

    #[test]
    fn saturates_symmetrically() {
        let mut torques = DVector::from_vec(vec![0.5, -0.5, 0.01, f64::NAN]);
        saturate(&mut torques, &[0.03; 4]);
        assert_eq!(torques.as_slice()[..3], [0.03, -0.03, 0.01]);
        // `std::max(-u, std::min(u, NaN))` is `u`, as here.
        assert_eq!(torques[3], 0.03);
    }
}
