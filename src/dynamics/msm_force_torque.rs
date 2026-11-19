//! Electrostatic forces and torques between charged spacecraft by the
//! multi-sphere method (MSM): port of upstream Basilisk
//! `simulation/dynamics/msmForceTorque`.
//!
//! Each space object is modeled as a set of conducting spheres fixed in its
//! body frame, all at the object's electric potential. With `k_c` Coulomb's
//! constant, the elastance matrix `[S]` has `k_c / R_i` on its diagonal (`R_i`
//! the radius of sphere `i`) and `k_c / |r_ij|` off it, and the sphere charges
//! solve `[S] q = V`. The force on sphere `j` of an object is the Coulomb force
//! of every sphere of the *other* objects,
//!
//! `F_j = − Σ_i k_c q_j q_i r_ij / |r_ij|³`, with `r_ij = r_i − r_j`,
//!
//! and each object receives the sum of its spheres' forces (inertial
//! components) and their torques about its body frame origin `B` (body
//! components). A sphere pair closer than the sphere's radius is skipped with a
//! warning, as upstream.
//!
//! This is a standalone module rather than a dynamic effector: per object it
//! publishes the force on an [`InertialForceCommandMsg`] and the torque on a
//! [`BodyTorqueCommandMsg`], which an
//! [`ExtForceTorque`](super::ext_force_torque::ExtForceTorque) on each
//! spacecraft applies, and the charges on a [`ChargeMsmMsg`].
//!
//! Upstream solves `[S] q = V` by Cholesky factorization. `[S]` is symmetric
//! positive definite for separated spheres; where overlapping spheres make it
//! indefinite, Eigen's factorization returns an unspecified result, and here an
//! LU solve is used instead, with a warning.

use nalgebra::{DMatrix, DVector, Vector3};

use crate::messages::{
    BodyTorqueCommandMsg, ChargeMsmMsg, InertialForceCommandMsg, Input, Output, SpacecraftStateMsg,
    VoltMsg,
};
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Upstream `kc` \[N m^2/C^2\]: Coulomb's constant as upstream rounds it.
const COULOMB_CONSTANT_NM2PC2: f64 = 8.99e9;

/// The multi-sphere model of one space object (upstream
/// `addSpacecraftToModel` arguments).
#[derive(Clone, Debug, Default)]
pub struct MsmSpaceObject {
    /// Upstream `radii` \[m\]: radius of each sphere.
    pub radii_m: Vec<f64>,
    /// Upstream `r_SB_B` \[m\]: center of each sphere relative to `B`, body
    /// components, at the index of its radius.
    pub sphere_positions_body_m: Vec<Vector3<f64>>,
}

/// User-settable parameters of [`MsmForceTorque`].
#[derive(Clone, Debug, Default)]
pub struct MsmForceTorqueConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// One entry per `addSpacecraftToModel` call, in that order; fixed at
    /// construction.
    pub space_objects: Vec<MsmSpaceObject>,
}

impl MsmForceTorqueConfig {
    /// Checks the configuration as upstream's `addSpacecraftToModel` and
    /// `Reset` do: matching radii and positions, at least two objects and at
    /// least one sphere.
    pub fn validate(&self) -> Result<(), String> {
        for (index, object) in self.space_objects.iter().enumerate() {
            if object.radii_m.len() != object.sphere_positions_body_m.len() {
                return Err(format!(
                    "space object {index}: the MSM radii and positions must have the same \
                     size, they have sizes {} and {}",
                    object.radii_m.len(),
                    object.sphere_positions_body_m.len()
                ));
            }
        }
        if self.space_objects.len() < 2 {
            return Err(format!(
                "MsmForceTorque must have 2 or more spacecraft components added, got {}",
                self.space_objects.len()
            ));
        }
        if self
            .space_objects
            .iter()
            .all(|object| object.radii_m.is_empty())
        {
            return Err("MsmForceTorque does not have any spheres added".to_string());
        }
        Ok(())
    }
}

/// Electrostatic MSM forces and torques: upstream `MsmForceTorque`.
#[derive(Clone, Debug)]
pub struct MsmForceTorque {
    /// User-settable parameters.
    pub config: MsmForceTorqueConfig,
    /// Upstream `scStateInMsgs`: each object's state, at the index of its
    /// `config.space_objects` entry (required).
    pub sc_state_in_msgs: Vec<Input<SpacecraftStateMsg>>,
    /// Upstream `voltInMsgs`: each object's potential (required).
    pub volt_in_msgs: Vec<Input<VoltMsg>>,
    /// Upstream `eTorqueOutMsgs`: electrostatic torque on each object about
    /// its `B`, body components.
    pub e_torque_out_msgs: Vec<Output<BodyTorqueCommandMsg>>,
    /// Upstream `eForceOutMsgs`: electrostatic force on each object, inertial
    /// components.
    pub e_force_out_msgs: Vec<Output<InertialForceCommandMsg>>,
    /// Upstream `chargeMsmOutMsgs`: the charges of each object's spheres.
    pub charge_msm_out_msgs: Vec<Output<ChargeMsmMsg>>,
}

impl MsmForceTorque {
    /// Builds the module with unconnected inputs and one set of outputs per
    /// space object.
    ///
    /// # Panics
    ///
    /// If `config` fails [`MsmForceTorqueConfig::validate`].
    pub fn new(config: MsmForceTorqueConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid MsmForceTorqueConfig: {message}");
        }
        let count = config.space_objects.len();
        Self {
            config,
            sc_state_in_msgs: (0..count).map(|_| Input::default()).collect(),
            volt_in_msgs: (0..count).map(|_| Input::default()).collect(),
            e_torque_out_msgs: (0..count).map(|_| Output::default()).collect(),
            e_force_out_msgs: (0..count).map(|_| Output::default()).collect(),
            charge_msm_out_msgs: (0..count).map(|_| Output::default()).collect(),
        }
    }

    /// Upstream `UpdateState` without the message traffic: the force
    /// (inertial), torque (body) and sphere charges of every object, for the
    /// given object states and potentials.
    pub fn compute(
        &self,
        states: &[SpacecraftStateMsg],
        voltages_v: &[f64],
    ) -> Vec<(InertialForceCommandMsg, BodyTorqueCommandMsg, ChargeMsmMsg)> {
        let objects = &self.config.space_objects;
        let dcms_nb: Vec<_> = states
            .iter()
            .map(|state| mrp_to_dcm(state.sigma_bn).transpose())
            .collect();
        let mut sphere_positions_m = Vec::new();
        let mut sphere_radii_m = Vec::new();
        let mut sphere_voltages_v = Vec::new();
        for (((object, state), dcm_nb), voltage_v) in
            objects.iter().zip(states).zip(&dcms_nb).zip(voltages_v)
        {
            for (radius_m, r_sb_b) in object.radii_m.iter().zip(&object.sphere_positions_body_m) {
                sphere_positions_m.push(state.position_m + dcm_nb * r_sb_b);
                sphere_radii_m.push(*radius_m);
                sphere_voltages_v.push(*voltage_v);
            }
        }

        let count = sphere_positions_m.len();
        let elastance = DMatrix::from_fn(count, count, |i, j| {
            if i == j {
                COULOMB_CONSTANT_NM2PC2 / sphere_radii_m[i]
            } else {
                COULOMB_CONSTANT_NM2PC2 / (sphere_positions_m[i] - sphere_positions_m[j]).norm()
            }
        });
        let voltages = DVector::from_vec(sphere_voltages_v);
        let charges_c = match elastance.clone().cholesky() {
            Some(cholesky) => cholesky.solve(&voltages),
            None => {
                log::warn!(
                    "MsmForceTorque '{}': the elastance matrix is not positive definite \
                     (overlapping spheres?); solving by LU",
                    self.config.name
                );
                elastance
                    .lu()
                    .solve(&voltages)
                    .unwrap_or_else(|| DVector::from_element(count, f64::NAN))
            }
        };

        let mut first_sphere = 0;
        objects
            .iter()
            .zip(&dcms_nb)
            .enumerate()
            .map(|(object_index, (object, dcm_nb))| {
                let spheres = first_sphere..first_sphere + object.radii_m.len();
                first_sphere = spheres.end;
                let mut force_n = Vector3::zeros();
                let mut torque_b = Vector3::zeros();
                for j in spheres.clone() {
                    let mut sphere_force_n = Vector3::zeros();
                    for i in (0..count).filter(|i| !spheres.contains(i)) {
                        let r_ij = sphere_positions_m[i] - sphere_positions_m[j];
                        let distance_m = r_ij.norm();
                        if distance_m > sphere_radii_m[j] {
                            sphere_force_n -=
                                COULOMB_CONSTANT_NM2PC2 * charges_c[j] * charges_c[i] * r_ij
                                    / distance_m
                                    / distance_m
                                    / distance_m;
                        } else {
                            log::warn!(
                                "MsmForceTorque: spacecraft {object_index}, sphere {} is too \
                                 close to another sphere",
                                j - spheres.start
                            );
                        }
                    }
                    force_n += sphere_force_n;
                    torque_b += object.sphere_positions_body_m[j - spheres.start]
                        .cross(&(dcm_nb.transpose() * sphere_force_n));
                }
                (
                    InertialForceCommandMsg {
                        force_request_inertial_n: force_n,
                    },
                    BodyTorqueCommandMsg {
                        torque_request_body_nm: torque_b,
                    },
                    ChargeMsmMsg {
                        charges_c: charges_c.as_slice()[spheres].to_vec(),
                    },
                )
            })
            .collect()
    }
}

impl Module for MsmForceTorque {
    /// Upstream `Reset`: every state and voltage input must be connected.
    fn init(&mut self) {
        for (index, (state_in, volt_in)) in self
            .sc_state_in_msgs
            .iter()
            .zip(&self.volt_in_msgs)
            .enumerate()
        {
            assert!(
                state_in.is_connected(),
                "MsmForceTorque '{}': sc_state_in_msgs[{index}] was not connected",
                self.config.name
            );
            assert!(
                volt_in.is_connected(),
                "MsmForceTorque '{}': volt_in_msgs[{index}] was not connected",
                self.config.name
            );
        }
    }

    fn update(&mut self, _context: &SimulationContext) {
        let states: Vec<_> = self.sc_state_in_msgs.iter().map(Input::read).collect();
        let voltages_v: Vec<_> = self
            .volt_in_msgs
            .iter()
            .map(|volt_in| volt_in.read().voltage_v)
            .collect();
        let results = self.compute(&states, &voltages_v);
        for (index, (force, torque, charges)) in results.into_iter().enumerate() {
            self.e_force_out_msgs[index].write(force);
            self.e_torque_out_msgs[index].write(torque);
            self.charge_msm_out_msgs[index].write(charges);
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `msmForceTorque/_UnitTest/test_msmForceTorque.py` with
    //! its hand-computed truth and relative accuracy, plus the configuration
    //! checks of its `Reset` and `addSpacecraftToModel`.

    use hifitime::Epoch;

    use super::*;
    use crate::simulation::Simulation;

    /// `unitTestSupport.compareDoubleArrayRelative` / `compareListRelative`.
    #[track_caller]
    fn assert_relative(actual: &[f64], truth: &[f64], accuracy: f64, what: &str) {
        assert_eq!(actual.len(), truth.len(), "{what}");
        for (a, t) in actual.iter().zip(truth) {
            assert!(
                ((a - t) / t).abs() <= accuracy,
                "{what}: {actual:?} vs {truth:?}"
            );
        }
    }

    fn sphere_model(count: usize) -> MsmSpaceObject {
        let positions = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [14.0, 5.0, 6.0]];
        let radii = [1.0, 2.0, 1.5];
        MsmSpaceObject {
            radii_m: radii[..count].to_vec(),
            sphere_positions_body_m: positions[..count]
                .iter()
                .copied()
                .map(Vector3::from)
                .collect(),
        }
    }

    /// Upstream `msmForceTorqueTestFunction`: three objects of two or three
    /// spheres at their own potentials, one update.
    #[test]
    fn forces_torques_and_charges_match_the_hand_computed_truth() {
        let mut module = MsmForceTorque::new(MsmForceTorqueConfig {
            name: "msmForceTorqueTag".to_string(),
            space_objects: vec![sphere_model(2), sphere_model(3), sphere_model(2)],
        });
        let states = [
            ([10.0, 2.0, 3.0], [0.1, 0.2, 0.3]),
            ([-10.0, -2.0, 3.0], [-0.1, 0.2, 0.3]),
            ([1.0, 1.0, 0.0], [0.1, 0.2, -0.3]),
        ]
        .map(|(position, sigma)| {
            Output::new(SpacecraftStateMsg {
                position_m: Vector3::from(position),
                sigma_bn: Vector3::from(sigma),
                ..Default::default()
            })
        });
        let volts =
            [30000.0, -10000.0, 20000.0].map(|voltage_v| Output::new(VoltMsg { voltage_v }));
        for index in 0..3 {
            states[index].connect_to(&mut module.sc_state_in_msgs[index]);
            volts[index].connect_to(&mut module.volt_in_msgs[index]);
        }
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("msmForceTorqueTag", &mut module, 500_000_000, 0);
            sim.run_for(0);
        }

        let force_truth = [
            [6.48179e-05, 0.00147205, 0.000924806],
            [0.00107182, -0.000240543, 0.000110224],
            [-0.00113664, -0.00123151, -0.00103503],
        ];
        let torque_truth = [
            [-0.00268192, 0.00295288, -0.000603687],
            [0.00688387, -0.00209438, -0.00647544],
            [0.00581629, 0.007876, -0.00986612],
        ];
        let charge_truth: [&[f64]; 3] = [
            &[1.99932e-6, 5.73861e-6],
            &[-1.06715e-6, -2.51072e-6, -1.94044e-6],
            &[1.30148e-6, 3.23131e-6],
        ];
        let accuracy = 1e-4;
        for index in 0..3 {
            let force = module.e_force_out_msgs[index]
                .read()
                .force_request_inertial_n;
            assert_relative(force.as_slice(), &force_truth[index], accuracy, "force");
            let torque = module.e_torque_out_msgs[index]
                .read()
                .torque_request_body_nm;
            assert_relative(torque.as_slice(), &torque_truth[index], accuracy, "torque");
            let charges = module.charge_msm_out_msgs[index].read().charges_c;
            assert_relative(&charges, charge_truth[index], accuracy, "charge");
        }
    }

    #[test]
    fn rejects_mismatched_radii_and_positions() {
        let mut object = sphere_model(2);
        object.radii_m.push(1.0);
        let config = MsmForceTorqueConfig {
            name: "msm".to_string(),
            space_objects: vec![object, sphere_model(2)],
        };
        assert!(config.validate().unwrap_err().contains("same size"));
    }

    #[test]
    fn requires_two_objects_and_a_sphere() {
        let one = MsmForceTorqueConfig {
            name: "msm".to_string(),
            space_objects: vec![sphere_model(2)],
        };
        assert!(one.validate().unwrap_err().contains("2 or more"));
        let empty = MsmForceTorqueConfig {
            name: "msm".to_string(),
            space_objects: vec![MsmSpaceObject::default(), MsmSpaceObject::default()],
        };
        assert!(empty.validate().unwrap_err().contains("any spheres"));
    }

    #[test]
    #[should_panic(expected = "volt_in_msgs[1] was not connected")]
    fn requires_every_input_connected() {
        let mut module = MsmForceTorque::new(MsmForceTorqueConfig {
            name: "msm".to_string(),
            space_objects: vec![sphere_model(1), sphere_model(1)],
        });
        let state = Output::<SpacecraftStateMsg>::default();
        let volt = Output::<VoltMsg>::default();
        for index in 0..2 {
            state.connect_to(&mut module.sc_state_in_msgs[index]);
        }
        volt.connect_to(&mut module.volt_in_msgs[0]);
        module.init();
    }

    /// Two single-sphere objects far apart reduce to Coulomb's law between
    /// point charges `q = V R / k_c`, equal and opposite.
    #[test]
    fn distant_spheres_obey_coulombs_law() {
        let object = |radius_m: f64| MsmSpaceObject {
            radii_m: vec![radius_m],
            sphere_positions_body_m: vec![Vector3::zeros()],
        };
        let module = MsmForceTorque::new(MsmForceTorqueConfig {
            name: "msm".to_string(),
            space_objects: vec![object(1.0), object(2.0)],
        });
        let separation_m = 1.0e6;
        let states = [
            SpacecraftStateMsg::default(),
            SpacecraftStateMsg {
                position_m: Vector3::new(separation_m, 0.0, 0.0),
                ..Default::default()
            },
        ];
        let results = module.compute(&states, &[1000.0, 2000.0]);
        let q0 = 1000.0 * 1.0 / COULOMB_CONSTANT_NM2PC2;
        let q1 = 2000.0 * 2.0 / COULOMB_CONSTANT_NM2PC2;
        let coulomb = COULOMB_CONSTANT_NM2PC2 * q0 * q1 / separation_m.powi(2);
        let force0 = results[0].0.force_request_inertial_n;
        let force1 = results[1].0.force_request_inertial_n;
        assert!((force0.x + coulomb).abs() <= 1e-5 * coulomb, "{force0:?}");
        assert!((force0 + force1).norm() <= 1e-12 * coulomb);
        assert_eq!(results[0].1.torque_request_body_nm, Vector3::zeros());
    }
}
