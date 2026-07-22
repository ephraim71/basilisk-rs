use nalgebra::Vector3;

use super::*;
use crate::messages::{Output, PlanetOrientation, PlanetStateMsg};

#[derive(Debug)]
struct ConstantGravityModel {
    acceleration_mps2: Vector3<f64>,
}

impl GravityModel for ConstantGravityModel {
    fn frame(&self) -> GravityFrame {
        GravityFrame::InertialInvariant
    }

    fn gravitational_parameter_m3ps2(&self) -> f64 {
        1.0
    }

    fn acceleration_mps2(
        &mut self,
        _position_m: Vector3<f64>,
    ) -> Result<Vector3<f64>, GravityError> {
        Ok(self.acceleration_mps2)
    }

    fn specific_potential_jpkg(&mut self, _position_m: Vector3<f64>) -> Result<f64, GravityError> {
        Ok(-1.0)
    }
}

#[derive(Debug)]
struct BodyFixedGravityModel;

impl GravityModel for BodyFixedGravityModel {
    fn frame(&self) -> GravityFrame {
        GravityFrame::BodyFixed
    }

    fn gravitational_parameter_m3ps2(&self) -> f64 {
        1.0
    }

    fn acceleration_mps2(
        &mut self,
        position_m: Vector3<f64>,
    ) -> Result<Vector3<f64>, GravityError> {
        Ok(Vector3::new(
            2.0 * position_m.x,
            3.0 * position_m.y,
            4.0 * position_m.z,
        ))
    }

    fn specific_potential_jpkg(&mut self, position_m: Vector3<f64>) -> Result<f64, GravityError> {
        Ok(-position_m.norm())
    }
}

#[test]
fn point_mass_validates_parameters_and_matches_closed_form() {
    assert!(matches!(
        PointMassGravityModel::new(0.0),
        Err(GravityError::InvalidParameter { .. })
    ));
    let mut model = PointMassGravityModel::new(8.0).expect("positive mu");
    let acceleration = model
        .acceleration_mps2(Vector3::new(2.0, 0.0, 0.0))
        .expect("non-singular position");
    assert!((acceleration - Vector3::new(-2.0, 0.0, 0.0)).norm() < 1.0e-14);
    assert_eq!(
        model
            .specific_potential_jpkg(Vector3::new(2.0, 0.0, 0.0))
            .expect("non-singular position"),
        -4.0
    );
    assert!(matches!(
        model.acceleration_mps2(Vector3::zeros()),
        Err(GravityError::SingularPosition)
    ));
}

#[test]
fn custom_gravity_models_are_extensible_without_editing_an_enum() {
    let mut gravity = GravityEffector::new();
    gravity
        .add_grav_body(
            GravBodyData::new(
                "custom",
                ConstantGravityModel {
                    acceleration_mps2: Vector3::new(1.0, -2.0, 3.0),
                },
            )
            .expect("valid body name"),
        )
        .expect("unique body");
    gravity.update_cache(0).expect("valid static state");
    let acceleration = gravity
        .compute_gravity_field(Vector3::new(1.0, 0.0, 0.0), 0)
        .expect("custom model evaluation");
    assert_eq!(acceleration, Vector3::new(1.0, -2.0, 3.0));
}

#[test]
fn gravity_effector_rejects_duplicate_names_and_central_bodies() {
    let mut gravity = GravityEffector::new();
    gravity
        .add_grav_body(
            GravBodyData::point_mass("earth", 1.0, true, Vector3::zeros(), Vector3::zeros())
                .expect("valid body"),
        )
        .expect("first central body");
    assert!(matches!(
        gravity.add_grav_body(
            GravBodyData::point_mass("earth", 1.0, false, Vector3::zeros(), Vector3::zeros())
                .expect("valid body")
        ),
        Err(GravityError::DuplicateBodyName(name)) if name == "earth"
    ));
    assert!(matches!(
        gravity.add_grav_body(
            GravBodyData::point_mass("moon", 1.0, true, Vector3::zeros(), Vector3::zeros())
                .expect("valid body")
        ),
        Err(GravityError::MultipleCentralBodies { .. })
    ));
}

#[test]
fn body_fixed_models_require_explicit_orientation() {
    let mut gravity = GravityEffector::new();
    gravity
        .add_grav_body(GravBodyData::new("body", BodyFixedGravityModel).expect("valid body"))
        .expect("unique body");
    assert!(matches!(
        gravity.update_cache(0),
        Err(GravityError::MissingOrientation(name)) if name == "body"
    ));
}

#[test]
fn body_fixed_models_use_explicit_static_orientation() {
    let angle = std::f64::consts::FRAC_PI_2;
    let inertial_to_fixed = nalgebra::Matrix3::new(
        angle.cos(),
        angle.sin(),
        0.0,
        -angle.sin(),
        angle.cos(),
        0.0,
        0.0,
        0.0,
        1.0,
    );
    let mut gravity = GravityEffector::new();
    gravity
        .add_grav_body(
            GravBodyData::new("body", BodyFixedGravityModel)
                .expect("valid body")
                .with_static_orientation(PlanetOrientation {
                    inertial_to_fixed,
                    inertial_to_fixed_dot: nalgebra::Matrix3::zeros(),
                }),
        )
        .expect("unique body");
    gravity.update_cache(0).expect("orientation supplied");
    let acceleration = gravity
        .compute_gravity_field(Vector3::new(1.0, 0.0, 0.0), 0)
        .expect("body-fixed evaluation");
    assert!((acceleration - Vector3::new(3.0, 0.0, 0.0)).norm() < 1.0e-14);
}

#[test]
fn body_fixed_models_consume_orientation_from_planet_messages() {
    let source = Output::new(PlanetStateMsg {
        position_inertial_m: Vector3::zeros(),
        velocity_inertial_mps: Vector3::zeros(),
        orientation: Some(PlanetOrientation::identity()),
    });
    let mut body = GravBodyData::new("body", BodyFixedGravityModel).expect("valid body");
    body.planet_body_input_mut().connect(source.slot());
    let mut gravity = GravityEffector::new();
    gravity.add_grav_body(body).expect("unique body");
    gravity.update_cache(0).expect("message has orientation");
    let acceleration = gravity
        .compute_gravity_field(Vector3::new(1.0, 2.0, 3.0), 0)
        .expect("body-fixed evaluation");
    assert_eq!(acceleration, Vector3::new(2.0, 6.0, 12.0));
}

#[test]
fn central_body_output_publishes_the_cached_planet_state() {
    let state = PlanetStateMsg {
        position_inertial_m: Vector3::new(1.0, 2.0, 3.0),
        velocity_inertial_mps: Vector3::new(4.0, 5.0, 6.0),
        orientation: None,
    };
    let mut gravity = GravityEffector::new();
    gravity
        .add_grav_body(
            GravBodyData::new(
                "earth",
                PointMassGravityModel::new(1.0).expect("positive mu"),
            )
            .expect("valid body")
            .central()
            .with_static_ephemeris(state.clone()),
        )
        .expect("unique central body");
    gravity.update_cache(10).expect("valid state");
    let output = gravity.central_body_out.read();
    assert_eq!(output.position_inertial_m, state.position_inertial_m);
    assert_eq!(output.velocity_inertial_mps, state.velocity_inertial_mps);
}

#[test]
fn multi_body_field_includes_central_body_indirect_acceleration() {
    let mut gravity = GravityEffector::new();
    gravity
        .add_grav_body(
            GravBodyData::point_mass("central", 8.0, true, Vector3::zeros(), Vector3::zeros())
                .expect("valid central body"),
        )
        .expect("unique central body");
    gravity
        .add_grav_body(
            GravBodyData::point_mass(
                "perturber",
                1.0,
                false,
                Vector3::new(10.0, 0.0, 0.0),
                Vector3::zeros(),
            )
            .expect("valid perturber"),
        )
        .expect("unique body");
    gravity.update_cache(0).expect("valid states");
    let acceleration = gravity
        .compute_gravity_field(Vector3::new(2.0, 0.0, 0.0), 0)
        .expect("non-singular positions");
    let expected_x = -2.0 + 1.0 / 64.0 - 1.0 / 100.0;
    assert!((acceleration.x - expected_x).abs() < 1.0e-14);
    assert_eq!(acceleration.y, 0.0);
    assert_eq!(acceleration.z, 0.0);
}

#[test]
fn spherical_harmonics_matches_reference_degree_20_result() {
    let mut model = SphericalHarmonicsGravityModel::from_file("assets/gravity/GGM03S.txt", 20)
        .expect("valid GGM03S coefficients");
    let acceleration = model
        .acceleration_at_degree_mps2(Vector3::new(15_000.0, 10_000.0, 6_378_136.3), 20, true)
        .expect("non-singular position");
    let expected = Vector3::new(
        -0.022_749_966_730_128_943,
        -0.015_319_349_272_587_638,
        -9.766_642_361_331_22,
    );
    assert!((acceleration - expected).norm() < 1.0e-12);
}

#[test]
fn polyhedral_model_matches_reference_tetrahedron_result() {
    let vertices = vec![
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(-1.0, -1.0, 1.0),
        Vector3::new(-1.0, 1.0, -1.0),
        Vector3::new(1.0, -1.0, -1.0),
    ];
    let faces = vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
    let mut model = PolyhedralGravityModel::new(10.0, vertices, faces).expect("valid tetrahedron");
    let acceleration = model
        .acceleration_mps2(Vector3::new(4.0, 3.0, 2.0))
        .expect("position outside polyhedron");
    let expected = Vector3::new(
        -0.257_558_193_308_830_73,
        -0.192_843_308_078_523_3,
        -0.128_018_169_556_617_88,
    );
    assert!((acceleration - expected).norm() < 1.0e-13);
}

#[test]
fn polyhedral_model_rejects_open_or_degenerate_meshes() {
    let vertices = vec![
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::zeros(),
    ];
    assert!(matches!(
        PolyhedralGravityModel::new(1.0, vertices, vec![[0, 1, 2], [0, 2, 3]]),
        Err(GravityError::InvalidPolyhedron(_))
    ));
}

#[test]
fn polyhedral_model_loads_txt_meshes_with_kilometer_units() {
    let path =
        std::env::temp_dir().join(format!("basilisk-rs-polyhedron-{}.txt", std::process::id()));
    let contents = "4 4\n\
0.001 0.001 0.001\n\
-0.001 -0.001 0.001\n\
-0.001 0.001 -0.001\n\
0.001 -0.001 -0.001\n\
1 3 2\n\
1 2 4\n\
1 4 3\n\
2 3 4\n";
    std::fs::write(&path, contents).expect("write temporary polyhedron fixture");
    let model = PolyhedralGravityModel::from_mesh_file(&path, 10.0).expect("load text mesh");
    std::fs::remove_file(&path).expect("remove temporary polyhedron fixture");
    assert_eq!(model.vertex_count(), 4);
    assert_eq!(model.face_count(), 4);
    assert!((model.volume_m3() - 8.0 / 3.0).abs() < 1.0e-14);
}

#[test]
fn polyhedral_model_loads_obj_and_gaskell_tab_meshes() {
    let obj = "v 0.001 0.001 0.001\n\
v -0.001 -0.001 0.001\n\
v -0.001 0.001 -0.001\n\
v 0.001 -0.001 -0.001\n\
f 1 3 2\n\
f 1 2 4\n\
f 1 4 3\n\
f 2 3 4\n";
    let tab = "4 4\n\
1 0.001 0.001 0.001\n\
2 -0.001 -0.001 0.001\n\
3 -0.001 0.001 -0.001\n\
4 0.001 -0.001 -0.001\n\
1 1 3 2\n\
2 1 2 4\n\
3 1 4 3\n\
4 2 3 4\n";

    for (extension, contents) in [("obj", obj), ("tab", tab)] {
        let path = std::env::temp_dir().join(format!(
            "basilisk-rs-polyhedron-{}.{}",
            std::process::id(),
            extension
        ));
        std::fs::write(&path, contents).expect("write temporary polyhedron fixture");
        let model = PolyhedralGravityModel::from_mesh_file(&path, 10.0).expect("load mesh");
        std::fs::remove_file(&path).expect("remove temporary polyhedron fixture");
        assert_eq!(model.vertex_count(), 4);
        assert_eq!(model.face_count(), 4);
    }
}
