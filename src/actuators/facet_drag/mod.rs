use nalgebra::Vector3;
use std::any::Any;

use crate::messages::{AtmosphereMsg, Input, SpacecraftStateMsg};
use crate::spacecraft::{DynamicEffector, EffectorOutput};

#[derive(Clone, Debug)]
pub struct FacetDragFacet {
    pub area_m2: f64,
    pub drag_coeff: f64,
    pub normal_body: Vector3<f64>,
    pub location_body_m: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct FacetDragConfig {
    pub name: String,
}

#[derive(Clone, Debug)]
pub struct FacetDragDynamicEffector {
    pub config: FacetDragConfig,
    pub atmosphere_in_msg: Input<AtmosphereMsg>,
    pub facets: Vec<FacetDragFacet>,
}

impl FacetDragDynamicEffector {
    pub fn new(config: FacetDragConfig) -> Self {
        Self {
            config,
            atmosphere_in_msg: Input::default(),
            facets: Vec::new(),
        }
    }

    pub fn add_facet(
        &mut self,
        area_m2: f64,
        drag_coeff: f64,
        normal_body: Vector3<f64>,
        location_body_m: Vector3<f64>,
    ) {
        self.facets.push(FacetDragFacet {
            area_m2,
            drag_coeff,
            normal_body,
            location_body_m,
        });
    }

    pub fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let atmosphere = self.atmosphere_in_msg.read();
        if atmosphere.neutral_density_kgpm3 <= 0.0 || state.velocity_mps.norm_squared() == 0.0 {
            return EffectorOutput::default();
        }

        let body_to_inertial = state.body_to_inertial();
        let inertial_to_body = body_to_inertial.inverse();
        let velocity_body = inertial_to_body.transform_vector(&state.velocity_mps);
        let speed = velocity_body.norm();
        if speed == 0.0 {
            return EffectorOutput::default();
        }
        let velocity_hat_body = velocity_body / speed;

        let (force_body, torque_body) = self.facets.iter().fold(
            (Vector3::zeros(), Vector3::zeros()),
            |(force_sum, torque_sum), facet| {
                let projected_area = facet.area_m2 * facet.normal_body.dot(&velocity_hat_body);
                if projected_area <= 0.0 {
                    return (force_sum, torque_sum);
                }

                let facet_force_body = -0.5
                    * speed
                    * speed
                    * facet.drag_coeff
                    * projected_area
                    * atmosphere.neutral_density_kgpm3
                    * velocity_hat_body;
                (
                    force_sum + facet_force_body,
                    torque_sum + facet.location_body_m.cross(&facet_force_body),
                )
            },
        );

        EffectorOutput {
            force_inertial_n: body_to_inertial.transform_vector(&force_body),
            torque_body_nm: torque_body,
        }
    }
}

impl DynamicEffector for FacetDragDynamicEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        FacetDragDynamicEffector::compute_output(self, state)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use crate::messages::{AtmosphereMsg, Output, SpacecraftStateMsg};

    use super::{FacetDragConfig, FacetDragDynamicEffector};

    fn make_drag(density: f64) -> (FacetDragDynamicEffector, Output<AtmosphereMsg>) {
        let atmosphere = Output::new(AtmosphereMsg {
            neutral_density_kgpm3: density,
            local_temp_k: 0.0,
        });
        let mut drag = FacetDragDynamicEffector::new(FacetDragConfig {
            name: "FacetDrag".to_string(),
        });
        drag.atmosphere_in_msg.connect(atmosphere.slot());
        (drag, atmosphere)
    }

    fn make_state(sigma_bn: Vector3<f64>, velocity_mps: Vector3<f64>) -> SpacecraftStateMsg {
        SpacecraftStateMsg {
            position_m: Vector3::zeros(),
            velocity_mps,
            sigma_bn,
            omega_radps: Vector3::zeros(),
        }
    }

    fn expected_force(
        density: f64,
        area: f64,
        coeff: f64,
        normal_body: Vector3<f64>,
        sigma_bn: Vector3<f64>,
        inertial_velocity: Vector3<f64>,
    ) -> Vector3<f64> {
        let state = make_state(sigma_bn, inertial_velocity);
        let velocity_body = state
            .body_to_inertial()
            .inverse()
            .transform_vector(&inertial_velocity);
        let speed = velocity_body.norm();
        let velocity_hat_body = velocity_body / speed;
        let projected_area = area * normal_body.dot(&velocity_hat_body);
        if projected_area > 0.0 {
            -0.5 * density * projected_area * coeff * speed * speed * velocity_hat_body
        } else {
            Vector3::zeros()
        }
    }

    #[test]
    fn facet_drag_calculation_matches_basilisk_unit_test_cases() {
        let cases = [
            (
                vec![1.0, 1.0],
                vec![2.0, 2.0],
                vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)],
                vec![Vector3::new(0.1, 0.0, 0.0), Vector3::new(0.0, 0.1, 0.0)],
            ),
            (
                vec![1.0, 1.0],
                vec![2.0, 2.0],
                vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)],
                vec![Vector3::new(0.3, 0.0, 0.0), Vector3::new(0.0, 0.3, 0.0)],
            ),
            (
                vec![1.0, 2.0],
                vec![2.0, 4.0],
                vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)],
                vec![Vector3::new(0.1, 0.0, 0.0), Vector3::new(0.0, 0.1, 0.0)],
            ),
            (
                vec![1.0, 1.0],
                vec![2.0, 2.0],
                vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)],
                vec![Vector3::new(0.1, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.1)],
            ),
            (
                vec![1.0, 1.0],
                vec![2.0, 2.0],
                vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)],
                vec![Vector3::new(0.1, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.1)],
            ),
            (
                vec![1.0, 1.0],
                vec![2.0, 2.0],
                vec![Vector3::new(0.0, 0.0, -1.0), Vector3::new(0.0, -1.0, 0.0)],
                vec![Vector3::new(0.0, 0.0, 0.1), Vector3::new(0.0, 0.1, 0.0)],
            ),
        ];

        let density = 1.0e-12;
        let sigma_bn = Vector3::zeros();
        let inertial_velocity = Vector3::new(0.0, 7.788e3, 0.0);

        for (areas, coeffs, normals, locations) in cases {
            let (mut drag, _atmo) = make_drag(density);
            for index in 0..areas.len() {
                drag.add_facet(
                    areas[index],
                    coeffs[index],
                    normals[index],
                    locations[index],
                );
            }
            let state = make_state(sigma_bn, inertial_velocity);
            let output = drag.compute_output(&state);

            let expected_force_body = (0..areas.len())
                .map(|index| {
                    expected_force(
                        density,
                        areas[index],
                        coeffs[index],
                        normals[index],
                        sigma_bn,
                        inertial_velocity,
                    )
                })
                .sum::<Vector3<f64>>();
            let expected_torque_body = (0..areas.len())
                .map(|index| {
                    locations[index].cross(&expected_force(
                        density,
                        areas[index],
                        coeffs[index],
                        normals[index],
                        sigma_bn,
                        inertial_velocity,
                    ))
                })
                .sum::<Vector3<f64>>();

            assert!(
                (output.force_inertial_n - expected_force_body).norm() < 1.0e-12,
                "force mismatch: expected {expected_force_body:?}, got {:?}",
                output.force_inertial_n
            );
            assert!(
                (output.torque_body_nm - expected_torque_body).norm() < 1.0e-12,
                "torque mismatch: expected {expected_torque_body:?}, got {:?}",
                output.torque_body_nm
            );
        }
    }

    #[test]
    fn shadowed_facets_produce_zero_force_and_torque_like_basilisk_unit_test() {
        let cases = [
            (
                vec![1.0, 1.0],
                vec![2.0, 2.0],
                vec![Vector3::new(0.0, 0.0, -1.0), Vector3::new(0.0, -1.0, 0.0)],
                vec![Vector3::new(0.0, 0.0, 0.1), Vector3::new(0.0, 0.1, 0.0)],
            ),
            (
                vec![1.0, 1.0],
                vec![2.0, 4.0],
                vec![Vector3::new(0.0, 0.0, -1.0), Vector3::new(0.0, -1.0, 0.0)],
                vec![Vector3::new(0.0, 0.0, 0.1), Vector3::new(0.0, 0.1, 0.0)],
            ),
            (
                vec![1.0, 1.0],
                vec![2.0, 2.0],
                vec![Vector3::new(0.0, 0.0, -1.0), Vector3::new(0.0, -1.0, 0.0)],
                vec![Vector3::new(0.0, 0.0, 0.4), Vector3::new(0.0, 0.4, 0.0)],
            ),
        ];

        for (areas, coeffs, normals, locations) in cases {
            let (mut drag, _atmo) = make_drag(1.0e-12);
            for index in 0..areas.len() {
                drag.add_facet(
                    areas[index],
                    coeffs[index],
                    normals[index],
                    locations[index],
                );
            }

            let output = drag.compute_output(&make_state(
                Vector3::zeros(),
                Vector3::new(0.0, 7.788e3, 0.0),
            ));

            assert!(output.force_inertial_n.norm() < 1.0e-12);
            assert!(output.torque_body_nm.norm() < 1.0e-12);
        }
    }

    #[test]
    fn attitude_rotates_drag_force_between_body_and_inertial_frames() {
        let (mut drag, _atmo) = make_drag(2.0);
        drag.add_facet(
            3.0,
            2.2,
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.2, 0.0),
        );
        let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let q = rotation.quaternion();
        let sigma_bn = Vector3::new(q.i, q.j, q.k) / (1.0 + q.w);
        let state = make_state(sigma_bn, Vector3::new(0.0, 10.0, 0.0));

        let output = drag.compute_output(&state);

        assert!(output.force_inertial_n.norm() > 0.0);
        assert!(output.torque_body_nm.norm() > 0.0);
    }

    #[test]
    fn zero_density_or_velocity_yields_zero_output() {
        let (mut drag, _atmo) = make_drag(0.0);
        drag.add_facet(
            1.0,
            2.0,
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.1, 0.0, 0.0),
        );

        let output = drag.compute_output(&make_state(
            Vector3::zeros(),
            Vector3::new(0.0, 7.788e3, 0.0),
        ));
        assert_eq!(output.force_inertial_n, Vector3::zeros());
        assert_eq!(output.torque_body_nm, Vector3::zeros());

        let (mut drag, _atmo) = make_drag(1.0);
        drag.add_facet(
            1.0,
            2.0,
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.1, 0.0, 0.0),
        );
        let output = drag.compute_output(&make_state(Vector3::zeros(), Vector3::zeros()));
        assert_eq!(output.force_inertial_n, Vector3::zeros());
        assert_eq!(output.torque_body_nm, Vector3::zeros());
    }
}
