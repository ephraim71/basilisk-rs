use nalgebra::{Matrix3, Vector3};
use std::any::Any;

use crate::messages::{EclipseMsg, HingedRigidBodyMsg, Input, SpacecraftStateMsg, SunEphemerisMsg};
use crate::spacecraft::{DynamicEffector, EffectorOutput};

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_693.0;
const SOLAR_FLUX_AT_EARTH_WPM2: f64 = 1361.0;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Clone, Debug)]
pub struct FacetSRPDynamicEffectorConfig {
    pub name: String,
}

#[derive(Clone, Debug)]
pub struct SolarRadiationPressureFacet {
    pub area_m2: f64,
    pub dcm_facet0_body: Matrix3<f64>,
    pub normal_facet: Vector3<f64>,
    pub articulation_axis_facet: Vector3<f64>,
    pub location_body_m: Vector3<f64>,
    pub diffuse_coeff: f64,
    pub specular_coeff: f64,
    pub articulation_angle_in: Input<HingedRigidBodyMsg>,
}

impl SolarRadiationPressureFacet {
    pub fn panel(
        area_m2: f64,
        normal_body: Vector3<f64>,
        location_body_m: Vector3<f64>,
        diffuse_coeff: f64,
        specular_coeff: f64,
    ) -> Self {
        Self {
            area_m2,
            dcm_facet0_body: Matrix3::identity(),
            normal_facet: normal_body,
            articulation_axis_facet: Vector3::zeros(),
            location_body_m,
            diffuse_coeff,
            specular_coeff,
            articulation_angle_in: Input::default(),
        }
    }

    pub fn articulated(
        area_m2: f64,
        dcm_facet0_body: Matrix3<f64>,
        normal_facet: Vector3<f64>,
        articulation_axis_facet: Vector3<f64>,
        location_body_m: Vector3<f64>,
        diffuse_coeff: f64,
        specular_coeff: f64,
    ) -> Self {
        Self {
            area_m2,
            dcm_facet0_body,
            normal_facet,
            articulation_axis_facet,
            location_body_m,
            diffuse_coeff,
            specular_coeff,
            articulation_angle_in: Input::default(),
        }
    }

    fn normal_body(&self) -> Vector3<f64> {
        let articulation_angle_rad = if self.articulation_angle_in.is_connected() {
            self.articulation_angle_in.read().theta_rad
        } else {
            0.0
        };
        let dcm_facet_facet0 = prv_to_dcm(articulation_angle_rad * self.articulation_axis_facet);
        let dcm_facet_body = dcm_facet_facet0 * self.dcm_facet0_body;
        dcm_facet_body.transpose() * self.normal_facet
    }
}

#[derive(Clone, Debug)]
pub struct FacetSRPDynamicEffector {
    pub config: FacetSRPDynamicEffectorConfig,
    pub input_sun_msg: Input<SunEphemerisMsg>,
    pub input_eclipse_msg: Input<EclipseMsg>,
    pub facets: Vec<SolarRadiationPressureFacet>,
}

impl FacetSRPDynamicEffector {
    pub fn new(config: FacetSRPDynamicEffectorConfig) -> Self {
        Self {
            config,
            input_sun_msg: Input::default(),
            input_eclipse_msg: Input::default(),
            facets: Vec::new(),
        }
    }

    pub fn add_panel(
        &mut self,
        area_m2: f64,
        normal_body: Vector3<f64>,
        location_body_m: Vector3<f64>,
        diffuse_coeff: f64,
        specular_coeff: f64,
    ) -> usize {
        self.facets.push(SolarRadiationPressureFacet::panel(
            area_m2,
            normal_body,
            location_body_m,
            diffuse_coeff,
            specular_coeff,
        ));
        self.facets.len() - 1
    }

    pub fn add_facet(
        &mut self,
        area_m2: f64,
        dcm_facet0_body: Matrix3<f64>,
        normal_facet: Vector3<f64>,
        articulation_axis_facet: Vector3<f64>,
        location_body_m: Vector3<f64>,
        diffuse_coeff: f64,
        specular_coeff: f64,
    ) -> usize {
        self.facets.push(SolarRadiationPressureFacet::articulated(
            area_m2,
            dcm_facet0_body,
            normal_facet,
            articulation_axis_facet,
            location_body_m,
            diffuse_coeff,
            specular_coeff,
        ));
        self.facets.len() - 1
    }

    pub fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let sun = self.input_sun_msg.read();
        let sun_vector_inertial = sun.sun_position_inertial_m - state.position_m;
        let sun_distance_m = sun_vector_inertial.norm();
        if sun_distance_m == 0.0 {
            return EffectorOutput::default();
        }

        let body_to_inertial = state.body_to_inertial();
        let sun_vector_body = body_to_inertial
            .inverse()
            .transform_vector(&sun_vector_inertial);
        let sun_hat_body = sun_vector_body / sun_distance_m;
        let srp_pressure = SOLAR_FLUX_AT_EARTH_WPM2 / SPEED_OF_LIGHT_MPS
            * (ASTRONOMICAL_UNIT_M / sun_distance_m).powi(2);
        let illumination_factor = if self.input_eclipse_msg.is_connected() {
            self.input_eclipse_msg.read().illumination_factor
        } else {
            1.0
        };

        let (force_body, torque_body) = self.facets.iter().fold(
            (Vector3::zeros(), Vector3::zeros()),
            |(force_sum, torque_sum), facet| {
                let normal_body = facet.normal_body();
                let cos_theta = normal_body.dot(&sun_hat_body);
                let projected_area_m2 = facet.area_m2 * cos_theta;
                if projected_area_m2 <= 0.0 {
                    return (force_sum, torque_sum);
                }

                let facet_force_body = -srp_pressure
                    * projected_area_m2
                    * ((1.0 - facet.specular_coeff) * sun_hat_body
                        + 2.0
                            * (facet.diffuse_coeff / 3.0 + facet.specular_coeff * cos_theta)
                            * normal_body);
                (
                    force_sum + facet_force_body,
                    torque_sum + facet.location_body_m.cross(&facet_force_body),
                )
            },
        );

        EffectorOutput {
            force_inertial_n: body_to_inertial
                .transform_vector(&(illumination_factor * force_body)),
            torque_body_nm: illumination_factor * torque_body,
        }
    }
}

impl DynamicEffector for FacetSRPDynamicEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        FacetSRPDynamicEffector::compute_output(self, state)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn prv_to_dcm(prv: Vector3<f64>) -> Matrix3<f64> {
    let angle = prv.norm();
    if angle == 0.0 {
        return Matrix3::identity();
    }

    let axis = prv / angle;
    let cp = angle.cos();
    let sp = angle.sin();
    let d1 = 1.0 - cp;

    Matrix3::new(
        axis.x * axis.x * d1 + cp,
        axis.x * axis.y * d1 + axis.z * sp,
        axis.x * axis.z * d1 - axis.y * sp,
        axis.y * axis.x * d1 - axis.z * sp,
        axis.y * axis.y * d1 + cp,
        axis.y * axis.z * d1 + axis.x * sp,
        axis.z * axis.x * d1 + axis.y * sp,
        axis.z * axis.y * d1 - axis.x * sp,
        axis.z * axis.z * d1 + cp,
    )
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, Vector3};

    use crate::messages::{HingedRigidBodyMsg, Output, SpacecraftStateMsg, SunEphemerisMsg};

    use super::{
        ASTRONOMICAL_UNIT_M, FacetSRPDynamicEffector, FacetSRPDynamicEffectorConfig,
        SOLAR_FLUX_AT_EARTH_WPM2, SPEED_OF_LIGHT_MPS, prv_to_dcm,
    };

    fn make_faceted_srp() -> (FacetSRPDynamicEffector, Output<SunEphemerisMsg>) {
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::zeros(),
            sun_velocity_inertial_mps: Vector3::zeros(),
        });
        let mut srp = FacetSRPDynamicEffector::new(FacetSRPDynamicEffectorConfig {
            name: "facet_srp".to_string(),
        });
        srp.input_sun_msg.connect(sun_out.slot());
        (srp, sun_out)
    }

    fn make_state(position_m: Vector3<f64>) -> SpacecraftStateMsg {
        SpacecraftStateMsg {
            position_m,
            ..Default::default()
        }
    }

    #[test]
    fn faceted_srp_uses_projected_panel_area() {
        let (mut srp, _sun_out) = make_faceted_srp();
        let area_m2 = 2.0;
        let normal_body = Vector3::new(-1.0, 0.0, 0.0);
        let location_body_m = Vector3::new(0.0, 1.0, 0.0);
        let diffuse_coeff = 0.1;
        let specular_coeff = 0.2;
        srp.add_panel(
            area_m2,
            normal_body,
            location_body_m,
            diffuse_coeff,
            specular_coeff,
        );

        let state = make_state(Vector3::new(ASTRONOMICAL_UNIT_M, 0.0, 0.0));
        let out = srp.compute_output(&state);
        let sun_hat_body = Vector3::new(-1.0, 0.0, 0.0);
        let cos_theta = 1.0;
        let srp_pressure = SOLAR_FLUX_AT_EARTH_WPM2 / SPEED_OF_LIGHT_MPS;
        let expected_force = -srp_pressure
            * area_m2
            * cos_theta
            * ((1.0 - specular_coeff) * sun_hat_body
                + 2.0 * (diffuse_coeff / 3.0 + specular_coeff * cos_theta) * normal_body);

        assert!((out.force_inertial_n - expected_force).norm() < 1.0e-18);
        assert!((out.torque_body_nm - location_body_m.cross(&expected_force)).norm() < 1.0e-18);
        let truth_force = Vector3::new(1.1500845250305353e-05, 0.0, 0.0);
        let truth_torque = Vector3::new(0.0, 0.0, -1.1500845250305353e-05);
        assert!((out.force_inertial_n - truth_force).norm() < 1.0e-18);
        assert!((out.torque_body_nm - truth_torque).norm() < 1.0e-18);

        let (mut double_area_srp, _sun_out) = make_faceted_srp();
        double_area_srp.add_panel(
            2.0 * area_m2,
            normal_body,
            location_body_m,
            diffuse_coeff,
            specular_coeff,
        );
        let double_area_out = double_area_srp.compute_output(&state);
        assert!((double_area_out.force_inertial_n - 2.0 * out.force_inertial_n).norm() < 1.0e-18);
        assert!((double_area_out.torque_body_nm - 2.0 * out.torque_body_nm).norm() < 1.0e-18);
    }

    #[test]
    fn faceted_srp_ignores_panels_facing_away_from_sun() {
        let (mut srp, _sun_out) = make_faceted_srp();
        srp.add_panel(
            2.0,
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            0.1,
            0.2,
        );

        let out = srp.compute_output(&make_state(Vector3::new(ASTRONOMICAL_UNIT_M, 0.0, 0.0)));
        assert_eq!(out.force_inertial_n, Vector3::zeros());
        assert_eq!(out.torque_body_nm, Vector3::zeros());
    }

    #[test]
    fn faceted_srp_reads_hinged_panel_angle_for_articulated_area() {
        let (mut srp, _sun_out) = make_faceted_srp();
        let facet_index = srp.add_facet(
            2.0,
            Matrix3::identity(),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 1.0, 0.0),
            0.1,
            0.2,
        );
        let hinge_out = Output::new(HingedRigidBodyMsg {
            theta_rad: std::f64::consts::FRAC_PI_2,
            theta_dot_radps: 0.0,
        });
        srp.facets[facet_index]
            .articulation_angle_in
            .connect(hinge_out.slot());

        let out = srp.compute_output(&make_state(Vector3::new(ASTRONOMICAL_UNIT_M, 0.0, 0.0)));

        assert!(out.force_inertial_n.x > 0.0);
        assert!(out.force_inertial_n.y.abs() < 1.0e-18);
        assert!(out.force_inertial_n.z.abs() < 1.0e-18);
        assert!(out.torque_body_nm.z < 0.0);
    }

    fn make_facet_srp_geometry(
        facet_rot_angle1_rad: f64,
        facet_rot_angle2_rad: f64,
    ) -> (FacetSRPDynamicEffector, Output<SunEphemerisMsg>) {
        let (mut srp, sun_out) = make_faceted_srp();
        let facet_rot_angle1 = Output::new(HingedRigidBodyMsg {
            theta_rad: facet_rot_angle1_rad,
            theta_dot_radps: 0.0,
        });
        let facet_rot_angle2 = Output::new(HingedRigidBodyMsg {
            theta_rad: facet_rot_angle2_rad,
            theta_dot_radps: 0.0,
        });

        let area1 = 1.5 * 1.5;
        let area2 = std::f64::consts::PI * (0.5 * 7.5) * (0.5 * 7.5);
        let areas = [
            area1, area1, area1, area1, area1, area1, area2, area2, area2, area2,
        ];
        let dcm_facet0_body = [
            prv_to_dcm((-90.0_f64).to_radians() * Vector3::new(0.0, 0.0, 1.0)),
            prv_to_dcm(0.0_f64.to_radians() * Vector3::new(0.0, 0.0, 1.0)),
            prv_to_dcm(90.0_f64.to_radians() * Vector3::new(0.0, 0.0, 1.0)),
            prv_to_dcm(180.0_f64.to_radians() * Vector3::new(0.0, 0.0, 1.0)),
            prv_to_dcm(90.0_f64.to_radians() * Vector3::new(1.0, 0.0, 0.0)),
            prv_to_dcm((-90.0_f64).to_radians() * Vector3::new(1.0, 0.0, 0.0)),
            prv_to_dcm(0.0_f64.to_radians() * Vector3::new(0.0, 0.0, 1.0)),
            prv_to_dcm(180.0_f64.to_radians() * Vector3::new(0.0, 0.0, 1.0)),
            prv_to_dcm(0.0_f64.to_radians() * Vector3::new(0.0, 0.0, 1.0)),
            prv_to_dcm(180.0_f64.to_radians() * Vector3::new(0.0, 0.0, 1.0)),
        ];
        let normal_facet = Vector3::new(0.0, 1.0, 0.0);
        let articulation_axes = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
        ];
        let locations = [
            Vector3::new(0.75, 0.0, 0.0),
            Vector3::new(0.0, 0.75, 0.0),
            Vector3::new(-0.75, 0.0, 0.0),
            Vector3::new(0.0, -0.75, 0.0),
            Vector3::new(0.0, 0.0, 0.75),
            Vector3::new(0.0, 0.0, -0.75),
            Vector3::new(4.5, 0.0, 0.75),
            Vector3::new(4.5, 0.0, 0.75),
            Vector3::new(-4.5, 0.0, 0.75),
            Vector3::new(-4.5, 0.0, 0.75),
        ];

        for index in 0..10 {
            let facet_index = srp.add_facet(
                areas[index],
                dcm_facet0_body[index],
                normal_facet,
                articulation_axes[index],
                locations[index],
                0.1,
                0.9,
            );
            if index == 6 || index == 7 {
                srp.facets[facet_index]
                    .articulation_angle_in
                    .connect(facet_rot_angle1.slot());
            } else if index == 8 || index == 9 {
                srp.facets[facet_index]
                    .articulation_angle_in
                    .connect(facet_rot_angle2.slot());
            }
        }

        (srp, sun_out)
    }

    #[test]
    fn faceted_srp_matches_expected_angle_matrix() {
        const CASES: &[(f64, f64, [f64; 3], [f64; 3], [f64; 3], [f64; 3])] = &[
            (
                -1.81514242207410276e-01,
                -4.88692190558412287e-01,
                [
                    7.47989353500040436e+10,
                    5.15885995413138007e+05,
                    1.44782682250461940e-09,
                ],
                [
                    -2.01390767547514972e-09,
                    7.32336213861859954e-09,
                    1.29366869264312278e-09,
                ],
                [
                    8.03556507711792803e-05,
                    6.91366863567341228e-10,
                    8.79536574135154611e-11,
                ],
                [
                    -4.83324032035429793e-10,
                    1.75805194766718394e-09,
                    3.07435122879342404e-10,
                ],
            ),
            (
                -1.81514242207410276e-01,
                7.88888821901437054e-01,
                [
                    7.47989353500040436e+10,
                    5.15885995413136319e+05,
                    -4.16400423040528305e-09,
                ],
                [
                    -1.68660413878935974e-09,
                    6.88689864166876038e-10,
                    3.25576096078302785e-09,
                ],
                [
                    8.03555501034925422e-05,
                    5.85858735424897174e-10,
                    -2.49393074568392565e-10,
                ],
                [
                    -4.04233026647802450e-10,
                    1.65155059209643911e-10,
                    7.80594029590135886e-10,
                ],
            ),
            (
                -1.81514242207410276e-01,
                -1.57079632679489656e+00,
                [
                    7.47989353500040436e+10,
                    5.15885995413133118e+05,
                    -1.09148082269435656e-09,
                ],
                [
                    -1.11386981981196533e-09,
                    2.92954772294651977e-09,
                    6.68321887031000345e-09,
                ],
                [
                    8.03551608247815509e-05,
                    4.02214575510718485e-10,
                    -6.45257658531728250e-11,
                ],
                [
                    -2.66569937530560417e-10,
                    7.03525937048451923e-10,
                    1.59941956705916198e-09,
                ],
            ),
            (
                -1.81514242207410276e-01,
                3.14159265358979312e+00,
                [
                    7.47989353500040436e+10,
                    5.15885995413139346e+05,
                    -1.09840971702839266e-09,
                ],
                [
                    -2.26803673043662901e-09,
                    4.65766334458732345e-09,
                    -2.26420950096616011e-10,
                ],
                [
                    8.03557154887762844e-05,
                    7.72761707591355413e-10,
                    -6.53559137267986849e-11,
                ],
                [
                    -5.44339106267705830e-10,
                    1.11772509011227806e-09,
                    -5.49111767664817644e-11,
                ],
            ),
            (
                7.88888821901437054e-01,
                -4.88692190558412287e-01,
                [
                    7.47989353500040436e+10,
                    5.15885995413135272e+05,
                    5.62310698119545861e-09,
                ],
                [
                    -1.47202055465073719e-09,
                    2.15471293779872284e-09,
                    -1.95860482619321602e-09,
                ],
                [
                    8.03554983100795999e-05,
                    5.18778362092877250e-10,
                    3.38161934631246848e-10,
                ],
                [
                    -3.53816204754374274e-10,
                    5.17561159134996882e-10,
                    -4.70368756439870848e-10,
                ],
            ),
            (
                7.88888821901437054e-01,
                7.88888821901437054e-01,
                [
                    7.47989353500040436e+10,
                    5.15885995413133234e+05,
                    1.76311521354685657e-12,
                ],
                [
                    -1.14476468904150477e-09,
                    -4.47990112129300968e-09,
                    -5.39167653504120878e-12,
                ],
                [
                    8.03553978430409944e-05,
                    4.13322614879376844e-10,
                    -1.08738496341512206e-12,
                ],
                [
                    -2.74764302896562408e-10,
                    -1.07528805493712893e-09,
                    -4.31388632280086543e-12,
                ],
            ),
            (
                7.88888821901437054e-01,
                -1.57079632679489656e+00,
                [
                    7.47989353500040436e+10,
                    5.15885995413130091e+05,
                    3.07192459377101824e-09,
                ],
                [
                    -5.71815560249440106e-10,
                    -2.23975384928749747e-09,
                    3.43089332442784760e-09,
                ],
                [
                    8.03550072999863522e-05,
                    2.29448054633865939e-10,
                    1.83307650121758542e-10,
                ],
                [
                    -1.36928594289066939e-10,
                    -5.37486050966583556e-10,
                    8.21571521213131646e-10,
                ],
            ),
            (
                7.88888821901437054e-01,
                3.14159265358979312e+00,
                [
                    7.47989353500040436e+10,
                    5.15885995413136436e+05,
                    3.07640826986138322e-09,
                ],
                [
                    -1.72621819708802864e-09,
                    -5.10966674289278166e-10,
                    -3.48517511193686748e-09,
                ],
                [
                    8.03555630514036910e-05,
                    6.00247684455789496e-10,
                    1.84760128604455189e-10,
                ],
                [
                    -4.14887005230439075e-10,
                    -1.22747885167474144e-10,
                    -8.37900662129750107e-10,
                ],
            ),
            (
                1.57079632679489656e+00,
                -4.88692190558412287e-01,
                [
                    7.47989353500040436e+10,
                    5.15885995413131954e+05,
                    2.55028874779196988e-09,
                ],
                [
                    -8.99951256305644922e-10,
                    4.39584400755117302e-09,
                    -5.39970746464384202e-09,
                ],
                [
                    8.03551103118641072e-05,
                    3.36030769780667161e-10,
                    1.54676443078760568e-10,
                ],
                [
                    -2.16685200472441155e-10,
                    1.05615098983022819e-09,
                    -1.30011111499494518e-09,
                ],
            ),
            (
                1.57079632679489656e+00,
                7.88888821901437054e-01,
                [
                    7.47989353500040436e+10,
                    5.15885995413130091e+05,
                    -3.07621416888859417e-09,
                ],
                [
                    -5.72948900545040920e-10,
                    -2.23879939186332787e-09,
                    -3.43769336604715973e-09,
                ],
                [
                    8.03550099847672166e-05,
                    2.30844147052551066e-10,
                    -1.85605170825326833e-10,
                ],
                [
                    -1.37835322195780119e-10,
                    -5.36723506904595338e-10,
                    -8.27011888486983948e-10,
                ],
            ),
            (
                1.57079632679489656e+00,
                -1.57079632679489656e+00,
                [
                    7.47989353500040436e+10,
                    5.15885995413126424e+05,
                    7.53028639413852331e-25,
                ],
                [
                    -7.30827026811535432e-31,
                    2.11883421343758657e-25,
                    4.84733065321680863e-20,
                ],
                [
                    8.03546180056633104e-05,
                    4.69700636251968956e-11,
                    4.52680364450649033e-26,
                ],
                [
                    -3.51229856963777936e-31,
                    5.09252423935862960e-26,
                    3.87786471691407495e-21,
                ],
            ),
            (
                1.57079632679489656e+00,
                3.14159265358979312e+00,
                [
                    7.47989353500040436e+10,
                    5.15885995413133292e+05,
                    -6.93864228825009723e-13,
                ],
                [
                    -1.15422987934052068e-09,
                    1.72961355622059020e-09,
                    -6.92537924719111394e-09,
                ],
                [
                    8.03551742339976848e-05,
                    4.17586412276757546e-10,
                    4.17140997766987818e-13,
                ],
                [
                    -2.77820759484211376e-10,
                    4.15400094003585153e-10,
                    -1.66692452231041672e-09,
                ],
            ),
            (
                3.14159265358979312e+00,
                -4.88692190558412287e-01,
                [
                    7.47989353500040436e+10,
                    5.15885995413138124e+05,
                    2.54038449633602876e-09,
                ],
                [
                    -2.05154890456843505e-09,
                    6.12385747566317676e-09,
                    1.52113795935785141e-09,
                ],
                [
                    8.03556599371782948e-05,
                    7.03460799314575826e-10,
                    1.53250676342572317e-10,
                ],
                [
                    -4.92399131679724575e-10,
                    1.47026949904281304e-09,
                    3.63184949685269468e-10,
                ],
            ),
            (
                3.14159265358979312e+00,
                7.88888821901437054e-01,
                [
                    7.47989353500040436e+10,
                    5.15885995413136436e+05,
                    -3.07173015129258939e-09,
                ],
                [
                    -1.72422958546389970e-09,
                    -5.10748099970037278e-10,
                    3.48033882532808675e-09,
                ],
                [
                    8.03555593599732560e-05,
                    5.97936401391995970e-10,
                    -1.84152785353073957e-10,
                ],
                [
                    -4.13295864791398279e-10,
                    -1.22572799255965964e-10,
                    8.34030822168270338e-10,
                ],
            ),
            (
                3.14159265358979312e+00,
                -1.57079632679489656e+00,
                [
                    7.47989353500040436e+10,
                    5.15885995413133292e+05,
                    -6.93864258506987613e-13,
                ],
                [
                    -1.15145464075638510e-09,
                    1.72961386413677514e-09,
                    6.90872781588858623e-09,
                ],
                [
                    8.03551695685947466e-05,
                    4.14248785446993986e-10,
                    4.17141027443236129e-13,
                ],
                [
                    -2.75600202759931201e-10,
                    4.15400750103228639e-10,
                    1.65360118224890953e-09,
                ],
            ),
            (
                3.14159265358979312e+00,
                3.14159265358979312e+00,
                [
                    7.47989353500040436e+10,
                    5.15885995413139579e+05,
                    -5.08665782034850009e-12,
                ],
                [
                    -2.30568292386572150e-09,
                    3.45818450910949476e-09,
                    4.86969528856909132e-20,
                ],
                [
                    8.03557246812245871e-05,
                    7.84861106630310512e-10,
                    9.42131409647206314e-14,
                ],
                [
                    -5.53418281961805572e-10,
                    8.29964282124062124e-10,
                    3.87782271164859374e-21,
                ],
            ),
        ];

        for (angle1, angle2, position, sigma_bn, expected_force_body, expected_torque_body) in CASES
        {
            let (srp, _sun_out) = make_facet_srp_geometry(*angle1, *angle2);
            let state = SpacecraftStateMsg {
                position_m: Vector3::from_column_slice(position),
                sigma_bn: Vector3::from_column_slice(sigma_bn),
                ..Default::default()
            };
            let out = srp.compute_output(&state);
            let force_body = state
                .inertial_to_body()
                .transform_vector(&out.force_inertial_n);
            let expected_force_body = Vector3::from_column_slice(expected_force_body);
            let expected_torque_body = Vector3::from_column_slice(expected_torque_body);

            assert!(
                (force_body - expected_force_body).norm() < 1.0e-12,
                "force mismatch for angles ({angle1}, {angle2}): {:?} vs {:?}",
                force_body,
                expected_force_body
            );
            assert!(
                (out.torque_body_nm - expected_torque_body).norm() < 1.0e-12,
                "torque mismatch for angles ({angle1}, {angle2}): {:?} vs {:?}",
                out.torque_body_nm,
                expected_torque_body
            );
        }
    }
}
