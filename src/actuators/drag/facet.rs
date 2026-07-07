use nalgebra::Vector3;
use std::any::Any;

use crate::messages::{AtmosphereMsg, Input, SpacecraftStateMsg};
use crate::spacecraft::{DynamicEffector, EffectorOutput};

#[derive(Clone, Debug)]
pub struct FacetDragConfig {
    pub name: String,
    pub planet_rotation_rate_radps: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct DragFacet {
    pub area_m2: f64,
    pub drag_coeff: f64,
    pub normal_body: Vector3<f64>,
    pub location_body_m: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct FacetDrag {
    pub config: FacetDragConfig,
    pub input_atmosphere_msg: Input<AtmosphereMsg>,
    pub facets: Vec<DragFacet>,
}

impl FacetDrag {
    pub fn new(config: FacetDragConfig) -> Self {
        Self {
            config,
            input_atmosphere_msg: Input::default(),
            facets: Vec::new(),
        }
    }

    pub fn add_facet(
        &mut self,
        area_m2: f64,
        drag_coeff: f64,
        normal_body: Vector3<f64>,
        location_body_m: Vector3<f64>,
    ) -> usize {
        self.facets.push(DragFacet {
            area_m2,
            drag_coeff,
            normal_body,
            location_body_m,
        });
        self.facets.len() - 1
    }

    pub fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let atmosphere = self.input_atmosphere_msg.read();
        if atmosphere.neutral_density_kgpm3 <= 0.0 {
            return EffectorOutput::default();
        }

        let relative_velocity_inertial = state.velocity_mps
            - self
                .config
                .planet_rotation_rate_radps
                .cross(&state.position_m);
        let relative_speed = relative_velocity_inertial.norm();
        if relative_speed == 0.0 {
            return EffectorOutput::default();
        }

        let body_to_inertial = state.body_to_inertial();
        let v_hat_body = body_to_inertial
            .inverse()
            .transform_vector(&relative_velocity_inertial)
            / relative_speed;

        let (force_body, torque_body) = self.facets.iter().fold(
            (Vector3::zeros(), Vector3::zeros()),
            |(f_sum, t_sum), facet| {
                let cos_theta = facet.normal_body.dot(&v_hat_body);
                let projected_area = facet.area_m2 * cos_theta;
                if projected_area <= 0.0 {
                    return (f_sum, t_sum);
                }
                let facet_force = -0.5
                    * atmosphere.neutral_density_kgpm3
                    * facet.drag_coeff
                    * projected_area
                    * relative_speed
                    * relative_speed
                    * v_hat_body;
                (
                    f_sum + facet_force,
                    t_sum + facet.location_body_m.cross(&facet_force),
                )
            },
        );

        EffectorOutput {
            force_inertial_n: body_to_inertial.transform_vector(&force_body),
            torque_body_nm: torque_body,
        }
    }
}

impl DynamicEffector for FacetDrag {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        FacetDrag::compute_output(self, state)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Quaternion, UnitQuaternion, Vector3};

    use crate::messages::{AtmosphereMsg, Output, SpacecraftStateMsg};

    use super::{FacetDrag, FacetDragConfig};

    fn make_drag(density: f64) -> (FacetDrag, Output<AtmosphereMsg>) {
        let atmo_out = Output::new(AtmosphereMsg {
            neutral_density_kgpm3: density,
            local_temp_k: 0.0,
        });
        let mut drag = FacetDrag::new(FacetDragConfig {
            name: "facet_drag".to_string(),
            planet_rotation_rate_radps: Vector3::zeros(),
        });
        drag.input_atmosphere_msg.connect(atmo_out.slot());
        (drag, atmo_out)
    }

    fn make_state(sigma_bn: Vector3<f64>, velocity_mps: Vector3<f64>) -> SpacecraftStateMsg {
        SpacecraftStateMsg {
            position_m: Vector3::zeros(),
            velocity_mps,
            sigma_bn,
            omega_radps: Vector3::zeros(),
        }
    }

    #[test]
    fn zero_density_yields_zero_output() {
        let (mut drag, _atmo) = make_drag(0.0);
        drag.add_facet(
            1.0,
            2.0,
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.1, 0.0, 0.0),
        );
        let out = drag.compute_output(&make_state(
            Vector3::zeros(),
            Vector3::new(0.0, 7788.0, 0.0),
        ));
        assert_eq!(out.force_inertial_n, Vector3::zeros());
        assert_eq!(out.torque_body_nm, Vector3::zeros());
    }

    /// Facets whose normals point away from the flow direction (cos_θ ≤ 0) contribute nothing.
    /// Normals [0,0,-1] and [0,-1,0], v=[0,7788,0]: both perpendicular to or opposing flow.
    /// Both normals are perpendicular or opposing the flow, so total force/torque = zero.
    #[test]
    fn facets_facing_away_from_flow_produce_no_force() {
        let (mut drag, _atmo) = make_drag(1.0);
        drag.add_facet(
            1.0,
            2.0,
            Vector3::new(0.0, 0.0, -1.0),
            Vector3::new(0.0, 0.0, 0.1),
        );
        drag.add_facet(
            1.0,
            2.0,
            Vector3::new(0.0, -1.0, 0.0),
            Vector3::new(0.0, 0.1, 0.0),
        );
        let out = drag.compute_output(&make_state(
            Vector3::zeros(),
            Vector3::new(0.0, 7788.0, 0.0),
        ));
        assert_eq!(out.force_inertial_n, Vector3::zeros());
        assert_eq!(out.torque_body_nm, Vector3::zeros());
    }

    /// Two facets, identity attitude, v=[0,7788,0] m/s, density=1.0 kg/m³.
    /// Facet 1: normal=[1,0,0] → cos_θ=0 → no contribution.
    /// Facet 2: normal=[0,1,0], area=1.0, Cd=2.0, loc=[0.3,0,0] → head-on.
    /// Force and torque computed directly from the facet drag formula.
    #[test]
    fn two_facets_identity_attitude_force_and_torque() {
        let density = 1.0_f64;
        let v_inertial = Vector3::new(0.0_f64, 7788.0, 0.0);

        let (mut drag, _atmo) = make_drag(density);
        drag.add_facet(
            1.0,
            2.0,
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.1, 0.0, 0.0),
        );
        drag.add_facet(
            1.0,
            2.0,
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.3, 0.0, 0.0),
        );

        let out = drag.compute_output(&make_state(Vector3::zeros(), v_inertial));

        // identity attitude: v_hat_B = v_hat_N = [0,1,0]
        let v_mag = v_inertial.norm();
        let v_hat_body = v_inertial / v_mag;

        // facet 1: cos_θ = [1,0,0]·[0,1,0] = 0 → skip
        // facet 2: cos_θ = [0,1,0]·[0,1,0] = 1.0
        let proj2 = 1.0_f64 * 1.0;
        let f2_body = -0.5 * density * 2.0 * proj2 * v_mag * v_mag * v_hat_body;
        let t2_body = Vector3::new(0.3, 0.0, 0.0).cross(&f2_body);

        assert!(
            (out.force_inertial_n - f2_body).norm() < 1e-12,
            "force: expected {f2_body:?}, got {:?}",
            out.force_inertial_n
        );
        assert!(
            (out.torque_body_nm - t2_body).norm() < 1e-12,
            "torque: expected {t2_body:?}, got {:?}",
            out.torque_body_nm
        );
    }

    /// Two facets, MRP attitude σ=[0.1,0.2,0.3], v=[7000,0,0] m/s, density=2.0 kg/m³.
    /// Expected values computed directly from the facet drag formula.
    /// Parametrized cases with MRP attitude rotation.
    #[test]
    fn two_facets_mrp_attitude_force_and_torque() {
        let density = 2.0_f64;
        let sigma = Vector3::new(0.1_f64, 0.2, 0.3);
        let v_inertial = Vector3::new(7000.0_f64, 0.0, 0.0);

        // MRP σ → quaternion q_NB (maps body→inertial): same convention as cannonball test
        let s2 = sigma.norm_squared(); // 0.14
        let d = 1.0 + s2;
        let q_nb = UnitQuaternion::new_normalize(Quaternion::new(
            (1.0 - s2) / d,
            2.0 * sigma.x / d,
            2.0 * sigma.y / d,
            2.0 * sigma.z / d,
        ));

        // v_hat in body frame: R_BN * v_inertial / |v|
        let v_mag = v_inertial.norm();
        let v_hat_body = q_nb.inverse().transform_vector(&v_inertial) / v_mag;

        let facets = [
            (
                1.5_f64,
                2.0_f64,
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(0.0, 0.5, 0.0),
            ),
            (
                1.0_f64,
                1.5_f64,
                Vector3::new(0.0, 1.0, 0.0),
                Vector3::new(0.3, 0.0, 0.0),
            ),
        ];

        // expected: facet drag force summed over facets
        let mut expected_force_body = Vector3::zeros();
        let mut expected_torque_body = Vector3::zeros();
        for (area, cd, normal, loc) in facets {
            let cos_theta = normal.dot(&v_hat_body);
            let proj = area * cos_theta;
            if proj > 0.0 {
                let f = -0.5 * density * cd * proj * v_mag * v_mag * v_hat_body;
                expected_force_body += f;
                expected_torque_body += loc.cross(&f);
            }
        }
        let expected_force_inertial = q_nb.transform_vector(&expected_force_body);

        let (mut drag, _atmo) = make_drag(density);
        for (area, cd, normal, loc) in facets {
            drag.add_facet(area, cd, normal, loc);
        }
        let out = drag.compute_output(&make_state(sigma, v_inertial));

        assert!(
            (out.force_inertial_n - expected_force_inertial).norm() < 1e-12,
            "force: expected {expected_force_inertial:?}, got {:?}",
            out.force_inertial_n
        );
        assert!(
            (out.torque_body_nm - expected_torque_body).norm() < 1e-12,
            "torque: expected {expected_torque_body:?}, got {:?}",
            out.torque_body_nm
        );
    }
}
