use nalgebra::{Matrix3, Vector3};
use std::any::Any;

use crate::messages::{EclipseMsg, HingedRigidBodyMsg, Input, SpacecraftStateMsg, SunEphemerisMsg};
use crate::spacecraft::{DynamicEffector, EffectorOutput};

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_693.0;
const SOLAR_FLUX_AT_EARTH_WPM2: f64 = 1368.0;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Clone, Debug)]
pub struct FacetSolarRadiationPressureConfig {
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

    pub fn basilisk_facet(
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
pub struct FacetSolarRadiationPressure {
    pub config: FacetSolarRadiationPressureConfig,
    pub input_sun_msg: Input<SunEphemerisMsg>,
    pub input_eclipse_msg: Input<EclipseMsg>,
    pub facets: Vec<SolarRadiationPressureFacet>,
}

impl FacetSolarRadiationPressure {
    pub fn new(config: FacetSolarRadiationPressureConfig) -> Self {
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
        self.facets
            .push(SolarRadiationPressureFacet::basilisk_facet(
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

impl DynamicEffector for FacetSolarRadiationPressure {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        FacetSolarRadiationPressure::compute_output(self, state)
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
        ASTRONOMICAL_UNIT_M, SOLAR_FLUX_AT_EARTH_WPM2, SPEED_OF_LIGHT_MPS,
        FacetSolarRadiationPressure, FacetSolarRadiationPressureConfig, prv_to_dcm,
    };

    fn make_faceted_srp() -> (FacetSolarRadiationPressure, Output<SunEphemerisMsg>) {
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::zeros(),
            sun_velocity_inertial_mps: Vector3::zeros(),
        });
        let mut srp = FacetSolarRadiationPressure::new(FacetSolarRadiationPressureConfig {
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
        let truth_force = Vector3::new(1.1559997283187156e-05, 0.0, 0.0);
        let truth_torque = Vector3::new(0.0, 0.0, -1.1559997283187156e-05);
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
    ) -> (FacetSolarRadiationPressure, Output<SunEphemerisMsg>) {
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
                [7.47989353500040436e+10, 5.15885995413138007e+05, 1.45522626190523832e-09],
                [-2.02426521610768207e-09, 7.36103154607472756e-09, 1.30029389651201269e-09],
                [8.07689421493787326e-05, 6.94922003790389547e-10, 8.84087712273876494e-11],
                [-4.85809474172430342e-10, 1.76709678887626562e-09, 3.08993553013402924e-10],
            ),
            (
                -1.81514242207410276e-01,
                7.88888821901437054e-01,
                [7.47989353500040436e+10, 5.15885995413136377e+05, -4.18541185261527595e-09],
                [-1.69527338436825773e-09, 6.92231034796033242e-10, 3.27249808248759240e-09],
                [8.07688409535506347e-05, 5.88865724239834186e-10, -2.50672827755971150e-10],
                [-4.06307769102161628e-10, 1.66003736665319856e-10, 7.84602315775134753e-10],
            ),
            (
                -1.81514242207410276e-01,
                -1.57079632679489656e+00,
                [7.47989353500040436e+10, 5.15885995413133118e+05, -1.09709359577007079e-09],
                [-1.11959151591885651e-09, 2.94461895494971787e-09, 6.71754904645029702e-09],
                [8.07684496757949012e-05, 4.04274616511184557e-10, -6.48525693892832591e-11],
                [-2.67935190627053501e-10, 7.07147392958166552e-10, 1.60761108503976440e-09],
            ),
            (
                -1.81514242207410276e-01,
                3.14159265358979312e+00,
                [7.47989353500040436e+10, 5.15885995413139346e+05, -1.10409394280671546e-09],
                [-2.27970147601766702e-09, 4.68161769408908801e-09, -2.27590687739806715e-10],
                [8.07690071995704063e-05, 7.76735847354714936e-10, -6.56912781036466056e-11],
                [-5.47138478228993783e-10, 1.12347282059827926e-09, -5.51977542317580777e-11],
            ),
            (
                7.88888821901437054e-01,
                -4.88692190558412287e-01,
                [7.47989353500040436e+10, 5.15885995413135272e+05, 5.65203032732392761e-09],
                [-1.47959619970620237e-09, 2.16579917396403347e-09, -1.96868077811172867e-09],
                [8.07687889138205655e-05, 5.21451805470716195e-10, 3.39905206950813926e-10],
                [-3.55639689483822639e-10, 5.20226280659629952e-10, -4.72789837805585363e-10],
            ),
            (
                7.88888821901437054e-01,
                7.88888821901437054e-01,
                [7.47989353500040436e+10, 5.15885995413133292e+05, 1.78129823600510236e-12],
                [-1.15065253634972154e-09, -4.50294251487911448e-09, -5.44728094277561570e-12],
                [8.07686879207334492e-05, 4.15448461023409787e-10, -1.09859917789136915e-12],
                [-2.76177500752750385e-10, -1.08081859424121445e-09, -4.35837548491516689e-12],
            ),
            (
                7.88888821901437054e-01,
                -1.57079632679489656e+00,
                [7.47989353500040436e+10, 5.15885995413130091e+05, 3.08771327408683695e-09],
                [-5.74753638210736047e-10, -2.25127251522850488e-09, 3.44852179181340728e-09],
                [8.07682953655750936e-05, 2.30624565425478565e-10, 1.84244516643907398e-10],
                [-1.37630514935058348e-10, -5.40249687766415927e-10, 8.25783044630392991e-10],
            ),
            (
                7.88888821901437054e-01,
                3.14159265358979312e+00,
                [7.47989353500040436e+10, 5.15885995413136552e+05, 3.09224318925005697e-09],
                [-1.73510175942835194e-09, -5.13595283319066788e-10, -3.50311284198017211e-09],
                [8.07688539880026653e-05, 6.03340907575195750e-10, 1.85711973573094684e-10],
                [-4.17025002880384356e-10, -1.23379665560082699e-10, -8.42220233144499909e-10],
            ),
            (
                1.57079632679489656e+00,
                -4.88692190558412287e-01,
                [7.47989353500040436e+10, 5.15885995413131954e+05, 2.56341420947017663e-09],
                [-9.04586210064130018e-10, 4.41846312080777279e-09, -5.42751718643912807e-09],
                [8.07683989297686914e-05, 3.37766511236772252e-10, 1.55481022132305184e-10],
                [-2.17804682928966701e-10, 1.06159114466377660e-09, -1.30682800882659793e-09],
            ),
            (
                1.57079632679489656e+00,
                7.88888821901437054e-01,
                [7.47989353500040436e+10, 5.15885995413130091e+05, -3.09204708717193147e-09],
                [-5.75898666319843590e-10, -2.25030821788735775e-09, -3.45539196031237487e-09],
                [8.07682980780408436e-05, 2.32035055010870009e-10, -1.86565729262010433e-10],
                [-1.38546593374462460e-10, -5.39479285304440931e-10, -8.31279515097890165e-10],
            ),
            (
                1.57079632679489656e+00,
                -1.57079632679489656e+00,
                [7.47989353500040436e+10, 5.15885995413126424e+05, 7.56902346867034364e-25],
                [-7.34588980452431361e-31, 2.12973872332931873e-25, 4.87226197019158045e-20],
                [8.07679040720313436e-05, 4.72116436776507042e-11, 4.55013493811315474e-26],
                [-3.53040060865677165e-31, 5.11877060499591692e-26, 3.89781075566911339e-21],
            ),
            (
                1.57079632679489656e+00,
                3.14159265358979312e+00,
                [7.47989353500040436e+10, 5.15885995413133292e+05, -7.01020572814995146e-13],
                [-1.16017358532405057e-09, 1.73851213803509806e-09, -6.96104148279447560e-09],
                [8.07684631763070380e-05, 4.19742823089824445e-10, 4.21443631046766960e-13],
                [-2.79255422358217835e-10, 4.17538780960277211e-10, -1.67553249919692433e-09],
            ),
            (
                3.14159265358979312e+00,
                -4.88692190558412287e-01,
                [7.47989353500040436e+10, 5.15885995413138240e+05, 2.55340780936576272e-09],
                [-2.06210041397806429e-09, 6.15535866262845031e-09, 1.52894371144913118e-09],
                [8.07689513629074898e-05, 7.07078500053390931e-10, 1.54040547113582656e-10],
                [-4.94931542354610663e-10, 1.47783511310493831e-09, 3.65038605087516849e-10],
            ),
            (
                3.14159265358979312e+00,
                7.88888821901437054e-01,
                [7.47989353500040436e+10, 5.15885995413136552e+05, -3.08751682520671773e-09],
                [-1.73309263923642361e-09, -5.13374454841041994e-10, 3.49822667870800667e-09],
                [8.07688502585025988e-05, 6.01005788226736771e-10, -1.85098366784817449e-10],
                [-4.15417452994552494e-10, -1.23202773985701883e-10, 8.38310483486241863e-10],
            ),
            (
                3.14159265358979312e+00,
                -1.57079632679489656e+00,
                [7.47989353500040436e+10, 5.15885995413133292e+05, -7.01020603304545268e-13],
                [-1.15736972567802632e-09, 1.73851245222708845e-09, 6.94421832512272436e-09],
                [8.07684584627897553e-05, 4.16370775265363955e-10, 4.21443661934786501e-13],
                [-2.77011964999694743e-10, 4.17539449036736563e-10, 1.66207175533428735e-09],
            ),
            (
                3.14159265358979312e+00,
                3.14159265358979312e+00,
                [7.47989353500040436e+10, 5.15885995413139637e+05, -5.13911656042752609e-12],
                [-2.31754169003927590e-09, 3.47597091028983753e-09, 4.89488989746396909e-20],
                [8.07690164398238797e-05, 7.88897864785735134e-10, 9.51847618705056100e-14],
                [-5.56264665536023685e-10, 8.34233018406953263e-10, 3.89788184150300466e-21],
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
