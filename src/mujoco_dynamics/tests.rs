//! Ports of upstream `simulation/mujocoDynamics/_GeneralModuleFiles/_UnitTest`
//! (`test_integration.py`, `test_objects.py`, `test_mass_update.py`,
//! `test_mass_props_and_com.py`, `test_qpos_policy.py`,
//! `test_site_velocity.py`, `test_stateful_sys_model.py`) for the scene
//! itself; the module tests are with their modules.
//!
//! Not ported, as they test upstream's Python bindings rather than the
//! dynamics: the SWIG proxy ownership and signature tests, the
//! `DynParamManager` state-handle identity and shape checks, the numba and
//! Python-subclass models, and the guards against scene mutation from inside
//! a callback (a task model here only gets read access to the scene).

use std::path::PathBuf;

use hifitime::Epoch;
use nalgebra::{DVector, Vector3, Vector4};

use super::*;
use crate::integrators::{DynamicObject, Euler, Rk4, Rkf78};
use crate::messages::{BodyTorqueCommandMsg, Output, SingleActuatorMsg, SpacecraftMassPropsMsg};
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, ep_to_dcm, mrp_to_ep};
use crate::{Module, SimulationContext};
use mujoco_rs::prelude::MjModel;

pub(crate) fn test_data(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("src/mujoco_dynamics/test_data")
        .join(name)
}

pub(crate) fn epoch() -> Epoch {
    Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
}

pub(crate) fn context(nanos: u64) -> SimulationContext {
    SimulationContext {
        current_sim_nanos: nanos,
        current_epoch: epoch() + hifitime::Duration::from_total_nanoseconds(nanos as i128),
    }
}

/// Seconds to nanoseconds, as upstream `macros.sec2nano`.
pub(crate) fn sec2nano(seconds: f64) -> u64 {
    (seconds * 1e9).round() as u64
}

/// Runs an initialized `scene` from `0` to `stop` in steps of `step`
/// \[ns\], calling `each` after every update (including the one at zero), as
/// a recorder in the same task after the scene would.
pub(crate) fn run(scene: &mut MjScene, step: u64, stop: u64, mut each: impl FnMut(&MjScene, u64)) {
    let mut now = 0;
    loop {
        scene.update(&context(now));
        each(scene, now);
        if now + step > stop {
            break;
        }
        now += step;
    }
}

fn quaternion_of(sigma: Vector3<f64>) -> Vector4<f64> {
    mrp_to_ep(sigma)
}

/// Upstream `_shortRotationQuaternion`: the scalar part made nonnegative.
fn short_rotation(quaternion: &[f64]) -> Vec<f64> {
    if quaternion[0] < 0.0 {
        quaternion.iter().map(|value| -value).collect()
    } else {
        quaternion.to_vec()
    }
}

fn assert_close(actual: &[f64], expected: &[f64], absolute: f64) {
    assert_eq!(actual.len(), expected.len());
    for (index, (a, e)) in actual.iter().zip(expected).enumerate() {
        assert!(
            (a - e).abs() <= absolute,
            "entry {index}: {a} vs {e} (tolerance {absolute})"
        );
    }
}

/// `pytest.approx` with its default relative tolerance of 1e-6 (and
/// absolute 1e-12).
fn assert_approx(actual: &[f64], expected: &[f64]) {
    for (a, e) in actual.iter().zip(expected) {
        assert!(
            (a - e).abs() <= (1e-6 * e.abs()).max(1e-12),
            "{actual:?} vs {expected:?}"
        );
    }
}

// ---------------------------------------------------------------------------
// test_integration.py
// ---------------------------------------------------------------------------

/// Upstream `test_integration`: RK4 integration of `test_sat.xml` (a cube
/// with two hinged panels, free of forces, with an initial body rate and
/// panel rates) matches MuJoCo's own RK4 stepper, whose result is
/// `test_sat_qpos_reference.txt`, to 1e-14.
#[test]
fn rk4_integration_matches_mujoco_native_stepper() {
    let reference: Vec<Vec<f64>> =
        std::fs::read_to_string(test_data("test_sat_qpos_reference.txt"))
            .expect("reference data")
            .lines()
            .map(|line| {
                line.split_whitespace()
                    .map(|value| value.parse().expect("number"))
                    .collect()
            })
            .collect();
    let dt = reference[1][0] - reference[0][0];
    let last = reference.last().expect("rows");
    let tf = last[0];

    let mut scene = MjScene::from_file(test_data("test_sat.xml")).expect("scene");
    scene.init();
    scene
        .body_mut("cube")
        .set_attitude_rate(Vector3::new(0.1, 0.2, 0.3));
    scene
        .body_mut("panel_1")
        .set_scalar_joint_velocity("panel_1_elevation", 0.1);
    scene
        .body_mut("panel_2")
        .set_scalar_joint_velocity("panel_2_elevation", 0.1);

    let mut qpos = Vec::new();
    run(&mut scene, sec2nano(dt), sec2nano(tf), |scene, _| {
        qpos = scene.state_out_msg.read().qpos;
    });

    assert_close(&qpos[0..3], &last[1..4], 1e-14);
    assert_close(
        &short_rotation(&qpos[3..7]),
        &short_rotation(&last[4..8]),
        1e-14,
    );
    assert_close(&qpos[7..], &last[8..], 1e-14);

    // The same from the cube origin site message.
    let cube = scene.body("cube").origin().state_out_msg.read();
    assert_approx(cube.position_m.as_slice(), &last[1..4]);
    let quaternion = quaternion_of(cube.sigma_bn);
    assert_approx(
        &short_rotation(quaternion.as_slice()),
        &short_rotation(&last[4..8]),
    );
}

const SPHERE_XML: &str = r#"
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <geom type="sphere" size="1" mass="10"/>
    </body>
  </worldbody>
</mujoco>
"#;

const FILTERED_ACTUATOR_XML: &str = r#"
<mujoco>
  <worldbody>
    <body name="body">
      <joint name="slide" type="slide"/>
      <geom type="sphere" size="1" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <general
      name="filtered"
      joint="slide"
      dyntype="filter"
      dynprm="2"
      gear="1"
    />
  </actuator>
</mujoco>
"#;

/// Upstream `_runFreeSpin`: the torque-free sphere spun up to `omega`,
/// integrated with RK4 at `dt` to `tf`; returns the final attitude
/// quaternion from the hub origin message.
fn run_free_spin(high_order: bool, dt: f64, tf: f64, omega: Vector3<f64>) -> Vector4<f64> {
    let mut scene = MjScene::new(SPHERE_XML).expect("scene");
    scene.extra_eom_call = true;
    scene.set_high_order_attitude_integration(high_order);
    scene.set_integrator(Rk4);
    scene.init();
    scene.body_mut("hub").set_attitude_rate(omega);
    let mut sigma = Vector3::zeros();
    run(&mut scene, sec2nano(dt), sec2nano(tf), |scene, _| {
        sigma = scene.body("hub").origin().state_out_msg.read().sigma_bn;
    });
    quaternion_of(sigma)
}

/// Principal rotation angle between two attitude quaternions.
fn attitude_error(quaternion: Vector4<f64>, reference: Vector4<f64>) -> f64 {
    let dcm = ep_to_dcm(quaternion) * ep_to_dcm(reference).transpose();
    4.0 * dcm_to_mrp(dcm).norm().atan()
}

/// Upstream `test_highOrderQuaternionConvergesFasterThanDefault`: halving
/// the step cuts the high-order (RK4) attitude error by about 16, the
/// default exponential-map path's by about 4.
#[test]
fn high_order_quaternion_converges_faster_than_default() {
    let omega = Vector3::new(0.4, -0.3, 0.8);
    let tf = 5.0;
    let error = |high_order, dt, reference| {
        attitude_error(run_free_spin(high_order, dt, tf, omega), reference)
    };
    let reference_high = run_free_spin(true, 0.005, tf, omega);
    let reference_low = run_free_spin(false, 0.005, tf, omega);

    let ratio_high = error(true, 0.10, reference_high) / error(true, 0.05, reference_high);
    let ratio_low = error(false, 0.10, reference_low) / error(false, 0.05, reference_low);

    assert!(ratio_high > 8.0, "high-order ratio {ratio_high}");
    assert!(ratio_low < 6.0, "default ratio {ratio_low}");
}

/// Upstream `test_quaternionStaysNormalized` (and test_qpos_policy's
/// `test_qpos_policy_normalizes_quaternions`) in both attitude modes.
#[test]
fn quaternion_stays_normalized() {
    for high_order in [false, true] {
        let q = run_free_spin(high_order, 0.1, 20.0, Vector3::new(0.3, -0.2, 0.5));
        assert!((q.norm() - 1.0).abs() <= 1e-10, "high order {high_order}");
        let q = run_free_spin(high_order, 0.1, 5.0, Vector3::new(0.4, -0.3, 0.8));
        assert!((q.norm() - 1.0).abs() <= 1e-12, "high order {high_order}");
    }
}

/// Upstream test_qpos_policy `test_high_order_policy_improves_rk4_convergence`.
#[test]
fn high_order_policy_improves_rk4_convergence() {
    let omega = Vector3::new(0.4, -0.3, 0.8);
    let reference = run_free_spin(true, 0.0025, 2.0, omega);
    let error = |dt| attitude_error(run_free_spin(true, dt, 2.0, omega), reference);
    assert!(error(0.1) / error(0.05) > 8.0);
}

/// Upstream `test_repeatedResetSucceeds`: `test_sat.xml` has `nq != nv`, and
/// the published state keeps the compiled sizes across resets.
#[test]
fn repeated_reset_succeeds() {
    let mut scene = MjScene::from_file(test_data("test_sat.xml")).expect("scene");
    scene.set_integrator(Rk4);
    scene.reset(&context(0));
    let first = scene.state_out_msg.read();
    scene.reset(&context(0));
    let second = scene.state_out_msg.read();

    assert_eq!((first.qpos.len(), second.qpos.len()), (9, 9));
    assert_eq!((first.qvel.len(), second.qvel.len()), (8, 8));
    assert_eq!((first.act.len(), second.act.len()), (0, 0));
    assert_eq!(first, second);
}

/// Upstream `test_repeated_reset_preserves_mass_and_activation_states`.
#[test]
fn repeated_reset_preserves_mass_and_activation_states() {
    let mut scene = MjScene::new(FILTERED_ACTUATOR_XML).expect("scene");
    scene.reset(&context(0));
    assert_eq!(scene.act().len(), 1);

    scene.set_act(&[0.75]);
    scene.set_masses(&[0.0, 2.5]);
    scene.reset(&context(sec2nano(3.0)));

    assert_eq!(scene.act().as_slice(), &[0.75]);
    assert_eq!(scene.masses().as_slice(), &[0.0, 2.5]);
    assert_eq!(scene.state_out_msg.read().act, vec![0.75]);
}

/// Upstream `test_reset_rejects_invalid_or_irreversible_mass_states`.
#[test]
fn reset_rejects_invalid_or_irreversible_mass_states() {
    for masses in [[f64::NAN, 1.0], [1.0, 1.0], [0.0, 0.0]] {
        let mut scene = MjScene::new(FILTERED_ACTUATOR_XML).expect("scene");
        scene.reset(&context(0));
        scene.set_masses(&masses);
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            scene.reset(&context(0));
        }));
        let message = result.expect_err("reset must reject the masses");
        let message = message
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| message.downcast_ref::<&str>().map(|text| text.to_string()))
            .unwrap_or_default();
        assert!(message.contains("mass"), "{masses:?}: {message}");
    }
}

// ---------------------------------------------------------------------------
// test_objects.py
// ---------------------------------------------------------------------------

const PREEXISTING_BODY_SITES_XML: &str = r#"
<mujoco>
  <worldbody>
    <body name="root">
      <joint name="slide" type="slide"/>
      <geom type="sphere" size="1" mass="1"/>
      <site name="root_com"/>
      <site name="root_origin"/>
    </body>
  </worldbody>
</mujoco>
"#;

const XML_EXPORT_XML: &str = r#"
<mujoco>
  <worldbody>
    <body name="root">
      <freejoint/>
      <geom type="sphere" size="1" pos="0.25 0 0" mass="1"/>
      <site name="site"/>
    </body>
  </worldbody>
</mujoco>
"#;

fn temp_path(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("basilisk-rs-mujoco-{}-{name}", std::process::id()))
}

/// Upstream `test_loading`: the bodies, sites and actuators of
/// `test_sat.xml`, including the composite actuators declared in its custom
/// text, and a site and actuator added after a reset.
#[test]
fn loading_creates_bodies_sites_and_actuators() {
    let mut scene = MjScene::from_file(test_data("test_sat.xml")).expect("scene");
    for (body, sites) in [
        ("cube", &["cube_com", "cube_origin"][..]),
        (
            "panel_1",
            &["panel_1_com", "panel_1_origin", "test_site"][..],
        ),
        ("panel_2", &["panel_2_com", "panel_2_origin"][..]),
    ] {
        let body = scene.body(body);
        for site in sites {
            assert_eq!(body.site(site).name(), *site);
        }
    }
    scene.single_actuator("panel_1_elevation");
    scene.single_actuator("panel_2_elevation");
    scene.force_actuator("test_act_1");
    scene.force_actuator("test_act_2");
    scene.force_torque_actuator("test_act_3");
    scene.force_torque_actuator("test_act_4");

    scene.reset(&context(0));
    scene.add_torque_actuator("test_add_1", "panel_2_origin");
    scene.add_site("cube", "test_add_1", Vector3::zeros(), Vector3::zeros());
    scene.torque_actuator("test_add_1");
    scene.body("cube").site("test_add_1");

    // The next evaluation recompiles the model with the new elements.
    scene.update(&context(1));
    assert!(!scene.has_pending_model_changes());
}

/// Upstream `test_xml_save_recompiles_pending_changes`: the saved XML holds
/// the generated sites and pending edits, and reloads.
#[test]
fn xml_save_recompiles_pending_changes() {
    for phase in ["constructed", "reset", "site_added", "actuator_added"] {
        let mut scene = MjScene::new(XML_EXPORT_XML).expect("scene");
        if phase != "constructed" {
            scene.reset(&context(0));
        }
        match phase {
            "site_added" => {
                scene.add_site("root", "added_site", Vector3::zeros(), Vector3::zeros());
            }
            "actuator_added" => {
                scene.add_single_actuator("added_actuator", "site", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
            }
            _ => {}
        }
        let path = temp_path(&format!("{phase}.xml"));
        scene.save_to_file(&path).expect("save");
        assert!(path.is_file());

        let mut restored = MjScene::from_file(&path).expect("reload");
        restored.reset(&context(0));
        for site in ["root_com", "root_origin", "site"] {
            assert_eq!(restored.body("root").site(site).name(), site);
        }
        if phase == "site_added" {
            assert_eq!(
                restored.body("root").site("added_site").name(),
                "added_site"
            );
        }
        if phase == "actuator_added" {
            assert_eq!(
                restored.single_actuator("added_actuator").name(),
                "added_actuator"
            );
        }
        std::fs::remove_file(&path).ok();
    }
}

/// Upstream `test_xml_save_reports_write_errors`.
#[test]
fn xml_save_reports_write_errors() {
    let mut scene = MjScene::new(XML_EXPORT_XML).expect("scene");
    scene.reset(&context(0));
    let path = temp_path("missing%folder").join("scene.xml");
    let error = scene
        .save_to_file(&path)
        .expect_err("the folder does not exist");
    let message = error.to_string();
    assert!(message.contains("Failed to save MuJoCo XML"), "{message}");
    assert!(message.contains(&path.display().to_string()), "{message}");
    assert!(!path.exists());
}

/// Upstream `test_actuator_addition_alone_triggers_recompile`.
#[test]
fn actuator_addition_alone_triggers_recompile() {
    let mut scene = MjScene::from_file(test_data("test_sat.xml")).expect("scene");
    scene.reset(&context(0));
    assert_eq!(
        scene
            .add_torque_actuator("late_torque", "panel_2_origin")
            .name(),
        "late_torque"
    );
    scene.update(&context(1));
    assert_eq!(scene.torque_actuator("late_torque").name(), "late_torque");
    assert!(!scene.has_pending_model_changes());
}

/// Upstream `test_scalar_joint_equality_recompiles_when_body_sites_already_exist`.
#[test]
fn scalar_joint_equality_exists_when_body_sites_already_exist() {
    let mut scene = MjScene::new(PREEXISTING_BODY_SITES_XML).expect("scene");
    scene.reset(&context(0));
    let equality = scene
        .body("root")
        .scalar_joint("slide")
        .constrained_equality()
        .to_string();
    assert!(!scene.is_equality_active(&equality));
}

/// Upstream `test_adaptive_free_joint_translation_tolerances_are_stage_independent`
/// (and `..._apply_after_integrator_replacement`): under adaptive
/// integration the free-joint translation and velocity records have a zero
/// relative tolerance, the attitude keeps it.
#[test]
fn adaptive_free_joint_translation_tolerances_are_absolute() {
    let mut scene = MjScene::from_file(test_data("test_sat.xml")).expect("scene");
    scene.set_integrator(Rkf78::default());
    scene.reset(&context(0));
    let joint = scene.body("cube").free_joint().clone();
    let base = scene.state().clone();
    let (atol, rtol) = (1e-12, 1.0);

    // A large position offers a large relative scale; with rtol zeroed the
    // difference is judged against atol alone.
    let mut high = base.clone();
    let mut low = base.clone();
    high.qpos[joint.qpos_adr()] = 7.0e6 + 1.0;
    low.qpos[joint.qpos_adr()] = 7.0e6;
    let translation = scene.error_norm(&high, &low, atol, rtol).expect("norm");
    assert!(translation > 1e6, "translation error norm {translation}");

    let mut high = base.clone();
    let mut low = base.clone();
    high.qvel[joint.qvel_adr()] = 1.0e3 + 1.0;
    low.qvel[joint.qvel_adr()] = 1.0e3;
    let velocity = scene.error_norm(&high, &low, atol, rtol).expect("norm");
    assert!(velocity > 1e6, "velocity error norm {velocity}");

    // The attitude quaternion keeps the relative tolerance.
    let mut high = base.clone();
    let mut low = base;
    high.qpos[joint.qpos_adr() + 3] = 1.0;
    low.qpos[joint.qpos_adr() + 3] = 1.0 - 1e-3;
    let attitude = scene.error_norm(&high, &low, atol, rtol).expect("norm");
    assert!(attitude < 1.0, "attitude error norm {attitude}");
}

/// Upstream `test_failed_candidate_compile_preserves_wrappers_and_recovers`:
/// an actuator on a missing site fails to compile, the scene keeps its
/// state, and adding the site recovers.
#[test]
fn failed_candidate_compile_preserves_state_and_recovers() {
    let mut scene = MjScene::from_file(test_data("test_sat.xml")).expect("scene");
    scene.reset(&context(0));

    scene.add_torque_actuator("pending_torque", "pending_site");
    let error = scene.recompile().expect_err("the site does not exist");
    assert!(error.to_string().contains("pending"), "{error}");

    scene
        .body_mut("cube")
        .set_position(Vector3::new(1.0, 2.0, 3.0));
    scene.add_site("cube", "pending_site", Vector3::zeros(), Vector3::zeros());
    scene.reset(&context(0));

    let adr = scene.body("cube").free_joint().qpos_adr();
    assert_eq!(&scene.qpos().as_slice()[adr..adr + 3], &[1.0, 2.0, 3.0]);
    scene.torque_actuator("pending_torque");
}

/// A task model with one state of rate `rate`, for checking integration
/// spans.
struct ConstantRate {
    rate: f64,
    value: f64,
}

impl SceneModel for ConstantRate {
    fn model_tag(&self) -> &str {
        "constantRate"
    }

    fn state_specs(&self) -> Vec<ModelStateSpec> {
        vec![ModelStateSpec::new("x", vec![0.0])]
    }

    fn load_states(&mut self, states: &[&[f64]]) {
        self.value = states[0][0];
    }

    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        context.set_derivative(0, &[self.rate]);
    }
}

/// Upstream `test_nonzero_reset_synchronizes_integration_clocks`: after a
/// reset at 2 s, an update at 2.125 s integrates 0.125 s.
#[test]
fn nonzero_reset_synchronizes_integration_clocks() {
    let mut scene = MjScene::new("<mujoco/>").expect("scene");
    let id = scene.add_model_to_dynamics_task(
        ConstantRate {
            rate: 1.0,
            value: 0.0,
        },
        -1,
    );
    let reset_nanos = 2_000_000_000;
    scene.reset(&context(reset_nanos));
    scene.update(&context(reset_nanos + 125_000_000));
    assert!((scene.task_model(id).value - 0.125).abs() < 1e-12);
}

/// Upstream `test_unnamed_body_introspection`: generated body names are
/// unique, deterministic, and agree with the parent and geometry queries
/// before and after a reset.
#[test]
fn unnamed_body_introspection() {
    for root_name in ["", r#"name="_nameless_1""#] {
        let xml = format!(
            r#"
            <mujoco>
              <worldbody>
                <body {root_name}>
                  <freejoint/>
                  <geom type="box" size="0.5 0.5 0.5"/>
                  <body pos="0 0 2">
                    <joint type="hinge"/>
                    <geom type="sphere" size="0.2"/>
                    <body name="named_tip" pos="0 0 1">
                      <joint type="hinge"/>
                      <geom type="sphere" size="0.1"/>
                    </body>
                  </body>
                </body>
                <body pos="3 0 0">
                  <freejoint/>
                  <geom type="box" size="0.3 0.3 0.3"/>
                </body>
                <body name="_nameless_0" pos="6 0 0">
                  <freejoint/>
                  <geom type="box" size="0.4 0.4 0.4"/>
                </body>
              </worldbody>
            </mujoco>
            "#
        );
        let mut scene = MjScene::new(&xml).expect("scene");
        let names = scene.body_names();
        let unique: std::collections::HashSet<_> = names.iter().collect();
        assert_eq!((names.len(), unique.len()), (5, 5));
        assert!(names.iter().all(|name| !name.is_empty()));
        assert_eq!(names[2], "named_tip");
        assert_eq!(names[4], "_nameless_0");
        if !root_name.is_empty() {
            assert_eq!(names[0], "_nameless_1");
        }
        let parents = [
            "world".to_string(),
            names[0].clone(),
            names[1].clone(),
            "world".to_string(),
            "world".to_string(),
        ];

        let check = |scene: &MjScene| {
            assert_eq!(scene.body_names(), names);
            for (name, parent) in names.iter().zip(&parents) {
                assert_eq!(&scene.body_parent_name(name).expect("body"), parent);
                assert_eq!(scene.body(name).name(), name);
            }
            let geoms: Vec<String> = scene
                .geom_infos()
                .into_iter()
                .map(|geom| geom.body_name)
                .collect();
            assert_eq!(geoms, names);
            let error = scene.body_parent_name("missing").expect_err("unknown body");
            assert!(error.to_string().contains("unknown body 'missing'"));
        };
        check(&scene);
        assert_eq!(MjScene::new(&xml).expect("scene").body_names(), names);
        scene.reset(&context(0));
        check(&scene);
    }
}

/// Upstream `test_geometry_rgba_matches_mujoco_material_precedence`: the
/// material color unless the geom has a non-default RGBA (MuJoCo's renderer
/// precedence).
#[test]
fn geometry_rgba_matches_mujoco_material_precedence() {
    let cases: [(&str, &str, [f64; 4]); 10] = [
        ("", "1 0 0 0.25", [0.5, 0.5, 0.5, 1.0]),
        (r#"rgba="0 0 1 0.5""#, "1 0 0 0.25", [0.0, 0.0, 1.0, 0.5]),
        (r#"material="paint""#, "1 0 0 0.25", [1.0, 0.0, 0.0, 0.25]),
        (r#"material="paint""#, "1 0 0 0", [1.0, 0.0, 0.0, 0.0]),
        (r#"material="paint""#, "1 0 0 1", [1.0, 0.0, 0.0, 1.0]),
        (
            r#"material="paint" rgba="0.5 0.5 0.5 1""#,
            "1 0 0 0.25",
            [1.0, 0.0, 0.0, 0.25],
        ),
        (
            r#"material="paint" rgba="0 1 0 0.5""#,
            "1 0 0 0.25",
            [0.0, 1.0, 0.0, 0.5],
        ),
        (
            r#"material="paint" rgba="0.5 0.5 0.5 0.125""#,
            "1 0 0 0.25",
            [0.5, 0.5, 0.5, 0.125],
        ),
        (
            r#"material="paint" class="tinted""#,
            "1 0 0 0.25",
            [0.0, 0.0, 1.0, 0.75],
        ),
        (r#"class="painted""#, "1 0 0 0.25", [1.0, 0.0, 0.0, 0.25]),
    ];
    for (geom_attributes, material_rgba, expected) in cases {
        let mut scene = MjScene::new(&format!(
            r#"
            <mujoco>
              <default>
                <default class="tinted"><geom rgba="0 0 1 0.75"/></default>
                <default class="painted"><geom material="paint"/></default>
              </default>
              <asset>
                <material name="unused" rgba="0 1 0 1"/>
                <material name="paint" rgba="{material_rgba}"/>
              </asset>
              <worldbody>
                <body name="hub">
                  <freejoint/>
                  <geom type="box" size="0.5 0.5 0.5" {geom_attributes}/>
                </body>
              </worldbody>
            </mujoco>
            "#
        ))
        .expect("scene");
        for _ in 0..2 {
            let geoms = scene.geom_infos();
            assert_eq!(geoms.len(), 1);
            for (actual, expected) in geoms[0].rgba.iter().zip(expected) {
                assert!(
                    (actual - expected).abs() < 1e-6,
                    "{geom_attributes} / {material_rgba}: {:?}",
                    geoms[0].rgba
                );
            }
            scene.reset(&context(0));
        }
    }
}

// ---------------------------------------------------------------------------
// test_mass_update.py, test_mass_props_and_com.py
// ---------------------------------------------------------------------------

/// Upstream `test_continuouslyChangingMass`: a ball pushed by 100 N while
/// losing 100 kg/s follows `v = ln(m0) - ln(m)` and
/// `x = t + m/100 ln(m/m0)`.
#[test]
fn continuously_changing_mass_follows_rocket_equation() {
    let (dt, tf) = (1.0, 25.0);
    let mut scene = MjScene::from_file(test_data("test_ball.xml")).expect("scene");
    scene.extra_eom_call = true;
    let thrust = Output::new(SingleActuatorMsg { input: 100.0 });
    thrust.connect_to(&mut scene.single_actuator_mut("ball").actuator_in_msg);
    let depletion_rate = 100.0;
    let mass_rate = Output::new(SpacecraftMassPropsMsg {
        mass_kg: -depletion_rate,
        ..SpacecraftMassPropsMsg::default()
    });
    mass_rate.connect_to(&mut scene.body_mut("ball").derivative_mass_properties_in_msg);

    scene.init();
    // The initial position does not matter.
    scene
        .body_mut("ball")
        .set_position(Vector3::new(0.0, 1.0, 0.0));

    let mut masses = Vec::new();
    let mut times = Vec::new();
    let mut last = None;
    run(&mut scene, sec2nano(dt), sec2nano(tf), |scene, now| {
        let body = scene.body("ball");
        masses.push(body.mass_properties_out_msg.read().mass_kg);
        times.push(now as f64 * 1e-9);
        last = Some(body.origin().state_out_msg.read());
    });
    let m0 = masses[0];
    let t = *times.last().expect("samples");
    let m = m0 - depletion_rate * t;
    let expected_v = m0.ln() - m.ln();
    let expected_x = t + m / depletion_rate * (m / m0).ln();
    let last = last.expect("samples");

    assert!((last.velocity_mps.x - expected_v).abs() <= 1e-6 * expected_v.abs());
    // The position catches `mj_setConst` leaving qpos at the reference pose.
    assert!((last.position_m.x - expected_x).abs() <= 1e-3 * expected_x.abs());
}

/// Upstream `test_com_site_reports_true_center_of_mass`: the `_com` site sits
/// at the center of mass, offset 2 m from the body origin, oriented like the
/// body frame.
#[test]
fn com_site_reports_true_center_of_mass() {
    let com_offset = 2.0;
    let mut scene = MjScene::new(&format!(
        r#"
        <mujoco>
            <option gravity="0 0 0" integrator="RK4" timestep="0.01"/>
            <worldbody>
                <body name="sat">
                    <freejoint/>
                    <geom type="box" size="1 1 0.5" pos="0 0 {com_offset}" density="100"/>
                </body>
            </worldbody>
        </mujoco>
        "#
    ))
    .expect("scene");
    scene.init();
    let dt = sec2nano(0.01);
    run(&mut scene, dt, dt, |_, _| {});

    let body = scene.body("sat");
    let com = body.center_of_mass().state_out_msg.read();
    let origin = body.origin().state_out_msg.read();
    assert!(((com.position_m - origin.position_m).norm() - com_offset).abs() <= 1e-9);
    assert!((com.sigma_bn - origin.sigma_bn).norm() <= 1e-12);
}

/// Upstream `test_inertia_rescales_with_mass`: doubling the mass halves the
/// angular acceleration under a constant torque.
#[test]
fn inertia_rescales_with_mass() {
    let (dt, tf, torque, mass_dot) = (0.01, 2.0, 5.0, 800.0);
    let mut scene = MjScene::new(
        r#"
        <mujoco>
            <option gravity="0 0 0" integrator="RK4" timestep="0.01"/>
            <worldbody>
                <body name="rotor">
                    <freejoint/>
                    <geom type="box" size="1 1 2" density="100"/>
                </body>
            </worldbody>
        </mujoco>
        "#,
    )
    .expect("scene");
    let torque_msg = Output::new(BodyTorqueCommandMsg {
        torque_request_body_nm: Vector3::new(torque, 0.0, 0.0),
    });
    torque_msg.connect_to(
        &mut scene
            .add_torque_actuator("trq", "rotor_origin")
            .torque_in_msg,
    );
    let mass_rate = Output::new(SpacecraftMassPropsMsg {
        mass_kg: mass_dot,
        ..SpacecraftMassPropsMsg::default()
    });
    mass_rate.connect_to(&mut scene.body_mut("rotor").derivative_mass_properties_in_msg);

    scene.init();
    let initial_mass = scene.body("rotor").mass();
    let mut times = Vec::new();
    let mut omega = Vec::new();
    let mut mass = 0.0;
    run(&mut scene, sec2nano(dt), sec2nano(tf), |scene, now| {
        let body = scene.body("rotor");
        times.push(now as f64 * 1e-9);
        omega.push(body.origin().state_out_msg.read().omega_radps.x);
        mass = body.mass_properties_out_msg.read().mass_kg;
    });
    assert!((mass - 2.0 * initial_mass).abs() <= 1e-3 * 2.0 * initial_mass);

    let slope = |i: usize, j: usize| (omega[j] - omega[i]) / (times[j] - times[i]);
    let n = times.len();
    let ratio = slope(n - 5, n - 1) / slope(1, 5);
    assert!((ratio - 0.5).abs() <= 0.1 * 0.5, "alpha ratio {ratio}");
}

// ---------------------------------------------------------------------------
// test_qpos_policy.py
// ---------------------------------------------------------------------------

const HINGE_XML: &str = r#"
<mujoco>
  <worldbody>
    <body name="link">
      <joint name="hinge" type="hinge"/>
      <geom type="sphere" size="1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"#;

const BALL_XML: &str = r#"
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="rotor">
      <joint name="ball" type="ball"/>
      <geom type="sphere" size="1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"#;

const BALL_THEN_SLIDE_XML: &str = r#"
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="mixed">
      <joint name="ball" type="ball"/>
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"#;

const FREE_THEN_SLIDE_XML: &str = r#"
<mujoco>
  <worldbody>
    <body name="hub">
      <freejoint name="free"/>
      <geom type="sphere" size="1" mass="10"/>
      <body name="link" pos="2 0 0">
        <joint name="slide" type="slide" axis="0 1 0"/>
        <geom type="sphere" size="0.5" mass="1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;

const MIXED_JOINT_XML: &str = r#"
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="free_body">
      <freejoint name="free"/>
      <geom type="sphere" size="0.2" mass="10"/>
      <body name="ball_body" pos="1 0 0">
        <joint name="ball" type="ball"/>
        <geom type="sphere" size="0.15" mass="2"/>
        <body name="slide_body" pos="1 0 0">
          <joint name="slide" type="slide" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;

/// Upstream `test_high_order_mode_is_finalized_policy_topology`.
#[test]
fn high_order_mode_is_fixed_after_reset() {
    let mut scene = MjScene::new(HINGE_XML).expect("scene");
    scene.reset(&context(0));
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        scene.set_high_order_attitude_integration(true);
    }));
    assert!(result.is_err());
    assert!(!scene.high_order_attitude_integration());
    scene.reset(&context(0));
}

/// Upstream `test_runtime_equality_activation_survives_compatible_recompile`.
#[test]
fn runtime_equality_activation_survives_compatible_recompile() {
    let mut scene = MjScene::new(HINGE_XML).expect("scene");
    scene.reset(&context(0));
    let equality = scene
        .body("link")
        .scalar_joint("hinge")
        .constrained_equality()
        .to_string();
    scene.set_equality_active(&equality, true);
    assert!(scene.is_equality_active(&equality));

    scene.add_site("link", "new_site", Vector3::zeros(), Vector3::zeros());
    scene.reset(&context(0));
    assert!(scene.is_equality_active(&equality));
}

/// Upstream `test_scalar_joint_metadata_uses_joint_id_after_free_joint`.
#[test]
fn scalar_joint_metadata_after_free_joint() {
    let mut scene = MjScene::new(FREE_THEN_SLIDE_XML).expect("scene");
    scene.reset(&context(0));
    let joint = scene.body("link").scalar_joint("slide").clone();
    assert!(!joint.is_hinge());
    assert_eq!(joint.axis(), [0.0, 1.0, 0.0]);
}

/// Upstream `test_native_joint_policies_match_mj_integrate_pos_bits`: one
/// Euler step of the native policy is bit for bit MuJoCo's
/// `mj_integratePos`.
#[test]
fn native_joint_policies_match_mj_integrate_pos_bits() {
    let mut scene = MjScene::new(MIXED_JOINT_XML).expect("scene");
    scene.set_integrator(Euler);
    scene.reset(&context(0));
    {
        let mut body = scene.body_mut("free_body");
        body.set_position(Vector3::new(0.25, -0.5, 0.75));
        body.set_free_joint_quaternion([0.9, 0.1, -0.2, 0.3]);
        body.set_velocity(Vector3::new(0.4, -0.2, 0.1));
        body.set_attitude_rate(Vector3::new(0.3, -0.5, 0.7));
    }
    {
        let mut body = scene.body_mut("ball_body");
        body.set_ball_joint_quaternion([0.8, -0.3, 0.4, 0.1]);
        body.set_ball_joint_rate(Vector3::new(-0.6, 0.2, 0.4));
    }
    {
        let mut body = scene.body_mut("slide_body");
        body.set_scalar_joint_position("slide", 0.6);
        body.set_scalar_joint_velocity("slide", -0.35);
    }
    let time_step = 0.0125;
    let mut expected = scene.qpos().as_slice().to_vec();
    let qvel = scene.qvel().as_slice().to_vec();
    let model = MjModel::from_xml_string(MIXED_JOINT_XML).expect("native model");
    model.integrate_pos(&mut expected, &qvel, time_step);

    scene.update(&context(sec2nano(time_step)));
    let actual = scene.qpos().as_slice();
    for (a, e) in actual.iter().zip(&expected) {
        assert_eq!(a.to_bits(), e.to_bits(), "{actual:?} vs {expected:?}");
    }
}

/// Upstream `test_joint_owned_buffer_follows_mujoco_joint_address_order`.
#[test]
fn joint_states_follow_mujoco_address_order() {
    let mut scene = MjScene::new(BALL_THEN_SLIDE_XML).expect("scene");
    scene.reset(&context(0));
    assert_eq!(scene.body("mixed").ball_joint().qpos_adr(), 0);
    assert_eq!(scene.body("mixed").scalar_joint("slide").qpos_adr(), 4);
    {
        let mut body = scene.body_mut("mixed");
        body.set_ball_joint_quaternion([1.0, 0.0, 0.0, 0.0]);
        body.set_scalar_joint_position("slide", 0.75);
        body.set_ball_joint_rate(Vector3::new(0.1, 0.2, 0.3));
        body.set_scalar_joint_velocity("slide", 0.4);
    }
    assert_eq!(scene.qpos().as_slice(), &[1.0, 0.0, 0.0, 0.0, 0.75]);
    assert_eq!(scene.qvel().as_slice(), &[0.1, 0.2, 0.3, 0.4]);
}

/// Upstream `test_ball_joint_owned_state_transfer`, in both modes.
#[test]
fn ball_joint_state_transfer() {
    for high_order in [false, true] {
        let mut scene = MjScene::new(BALL_XML).expect("scene");
        scene.set_high_order_attitude_integration(high_order);
        scene.reset(&context(0));
        {
            let mut body = scene.body_mut("rotor");
            body.set_ball_joint_quaternion([1.0, 0.0, 0.0, 0.0]);
            body.set_ball_joint_rate(Vector3::new(0.2, -0.1, 0.3));
        }
        scene.update(&context(1));
        let quaternion = &scene.qpos().as_slice()[0..4];
        let norm = quaternion.iter().map(|q| q * q).sum::<f64>().sqrt();
        assert!((norm - 1.0).abs() < 1e-12);
        assert_eq!(scene.qvel().len(), 3);
    }
}

// ---------------------------------------------------------------------------
// test_site_velocity.py
// ---------------------------------------------------------------------------

const ROTATED_SITE_XML: &str = r#"
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
      <site name="rotated" quat="0.7071067811865476 0 0 0.7071067811865476"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Upstream `test_siteVelocity`: a force-free hub keeps its initial linear
/// and angular velocity.
#[test]
fn site_velocity_is_conserved_without_forces() {
    let mut scene = MjScene::from_file(test_data("sat_hub_only.xml")).expect("scene");
    let velocity = Vector3::new(5e-3, -5e-3, 5e-3);
    let rate = Vector3::new(0.01, -0.01, 0.005);
    scene.init();
    {
        let mut hub = scene.body_mut("hub");
        hub.set_position(Vector3::new(1.5, -1.0, 0.75));
        hub.set_velocity(velocity);
        hub.set_attitude(Vector3::new(0.15, -0.1, 0.1));
        hub.set_attitude_rate(rate);
    }
    let mut last = None;
    run(&mut scene, sec2nano(1.0), sec2nano(30.0), |scene, _| {
        last = Some(scene.body("hub").origin().state_out_msg.read());
    });
    let last = last.expect("samples");
    for (actual, expected) in last.velocity_mps.iter().zip(velocity.iter()) {
        assert!((actual - expected).abs() <= 1e-8 * expected.abs());
    }
    for (actual, expected) in last.omega_radps.iter().zip(rate.iter()) {
        assert!((actual - expected).abs() <= 1e-8 * expected.abs());
    }
}

/// Upstream `test_rotated_site_angular_velocity`: a site rotated 90° about z
/// reports the body rate in its own frame.
#[test]
fn rotated_site_angular_velocity() {
    let mut scene = MjScene::new(ROTATED_SITE_XML).expect("scene");
    scene.init();
    scene
        .body_mut("hub")
        .set_attitude_rate(Vector3::new(0.1, 0.2, 0.3));
    let dt = sec2nano(0.1);
    run(&mut scene, dt, dt, |_, _| {});
    let state = scene.site("rotated").state_out_msg.read();
    assert_close(state.omega_radps.as_slice(), &[0.2, -0.1, 0.3], 1e-14);
}

/// Records the site position the dynamics task sees at each evaluation.
struct SiteProbe {
    site: Output<crate::messages::SpacecraftStateMsg>,
    samples: Vec<(u64, f64)>,
}

impl SceneModel for SiteProbe {
    fn model_tag(&self) -> &str {
        "siteProbe"
    }

    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        self.samples
            .push((context.current_sim_nanos(), self.site.read().position_m.x));
    }
}

/// Upstream `test_site_messages_publish_stage_and_final_states`: models in
/// the dynamics task read the site states of the stage, and after the step
/// the site holds the final state, with or without the extra evaluation.
#[test]
fn site_messages_publish_stage_and_final_states() {
    for extra_eom_call in [false, true] {
        let dt = 0.1;
        let mut scene = MjScene::new(ROTATED_SITE_XML).expect("scene");
        scene.extra_eom_call = extra_eom_call;
        let site = scene.site("rotated").state_out_msg.clone();
        let probe = scene.add_model_to_dynamics_task(
            SiteProbe {
                site: site.clone(),
                samples: Vec::new(),
            },
            0,
        );
        scene.init();
        scene
            .body_mut("hub")
            .set_position(Vector3::new(2.0, 0.0, 0.0));
        scene
            .body_mut("hub")
            .set_velocity(Vector3::new(1.0, 0.0, 0.0));
        run(&mut scene, sec2nano(dt), sec2nano(dt), |_, _| {});

        // RK4 stages at the step start, midpoint (twice) and end: the stage
        // position is 2 + v (t - 0) at each.
        let samples = &scene.task_model(probe).samples;
        let stages: Vec<_> = samples.iter().filter(|(nanos, _)| *nanos > 0).collect();
        assert!(!stages.is_empty());
        for (nanos, x) in stages {
            let t = *nanos as f64 * 1e-9;
            assert!((x - (2.0 + t)).abs() < 1e-12, "stage at {t}: {x}");
        }
        assert!((site.read().position_m.x - (2.0 + dt)).abs() <= 1e-6 * (2.0 + dt));
    }
}

// ---------------------------------------------------------------------------
// test_stateful_sys_model.py
// ---------------------------------------------------------------------------

/// Upstream `ExponentialStateModel`: `dx/dt = t x`.
struct ExponentialState {
    value: f64,
}

impl SceneModel for ExponentialState {
    fn model_tag(&self) -> &str {
        "testModel"
    }

    fn state_specs(&self) -> Vec<ModelStateSpec> {
        vec![ModelStateSpec::new("x", vec![0.0])]
    }

    fn load_states(&mut self, states: &[&[f64]]) {
        self.value = states[0][0];
    }

    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        let t = context.current_sim_nanos() as f64 * 1e-9;
        let x = context.state(0)[0];
        context.set_derivative(0, &[t * x]);
    }
}

/// Upstream `test_stateful`: with `x(0) = 1`, `x(1) = e^(1/2)`.
#[test]
fn stateful_model_integrates_its_state() {
    let mut scene = MjScene::new("<mujoco/>").expect("scene");
    let id = scene.add_model_to_dynamics_task(ExponentialState { value: 0.0 }, -1);
    scene.init();
    scene.set_task_model_state(id, 0, &[1.0]);
    run(&mut scene, sec2nano(0.01), sec2nano(1.0), |_, _| {});
    let expected = (0.5f64).exp();
    let value = scene.task_model(id).value;
    assert!((value - expected).abs() <= 1e-6 * expected, "{value}");
    assert_eq!(scene.task_model_state(id, 0), &[value]);
}

/// Upstream `test_empty_model_tags_receive_unique_state_namespaces`: two
/// copies of one model keep separate states.
#[test]
fn identical_models_keep_separate_states() {
    let mut scene = MjScene::new("<mujoco/>").expect("scene");
    let first = scene.add_model_to_dynamics_task(
        ConstantRate {
            rate: 1.0,
            value: 0.0,
        },
        -1,
    );
    let second = scene.add_model_to_dynamics_task(
        ConstantRate {
            rate: 2.0,
            value: 0.0,
        },
        -1,
    );
    scene.init();
    scene.update(&context(sec2nano(0.5)));
    assert!((scene.task_model(first).value - 0.5).abs() < 1e-12);
    assert!((scene.task_model(second).value - 1.0).abs() < 1e-12);
}

/// Upstream `test_late_stateful_model_is_rejected_before_task_mutation`.
#[test]
#[should_panic(expected = "cannot be added after Reset")]
fn late_stateful_model_is_rejected() {
    let mut scene = MjScene::new("<mujoco/>").expect("scene");
    scene.reset(&context(0));
    scene.add_model_to_dynamics_task(
        ConstantRate {
            rate: 1.0,
            value: 0.0,
        },
        -1,
    );
}

const SLIDER_XML: &str = r#"
<mujoco>
  <worldbody>
    <body name="slider">
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Upstream `test_diffusion_synchronizes_candidate_state_and_mass_before_callbacks`.
#[test]
fn diffusion_synchronizes_candidate_state_and_mass() {
    let mut scene = MjScene::new(SLIDER_XML).expect("scene");
    scene.add_fwd_kinematics_to_diffusion_dynamics_task(0);
    scene.reset(&context(0));
    scene
        .body_mut("slider")
        .set_scalar_joint_position("slide", 2.0);
    scene.set_masses(&[0.0, 2.0]);

    let state = scene.state().clone();
    scene.equations_of_motion_diffusion(&state, sec2nano(0.25), epoch());
    assert_close(
        scene
            .body("slider")
            .origin()
            .state_out_msg
            .read()
            .position_m
            .as_slice(),
        &[2.0, 0.0, 0.0],
        1e-14,
    );

    let mut state = scene.state().clone();
    state.mass = DVector::from_vec(vec![0.0, -1.0]);
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        scene.equations_of_motion_diffusion(&state, sec2nano(0.25), epoch());
    }));
    assert!(result.is_err(), "a negative mass is rejected");
}

/// Sample statistics of an Ornstein-Uhlenbeck path, as upstream's noise
/// tests estimate them.
pub(crate) struct OuStatistics {
    pub(crate) mean: f64,
    pub(crate) variance: f64,
    pub(crate) phi: f64,
    pub(crate) time_constant: f64,
}

/// Upstream's estimates: the mean, the unbiased variance, the lag-one
/// autocorrelation `φ` and the time constant `-dt / ln φ` it implies (with
/// `φ` clamped to `[1e-6, 0.999999]`).
pub(crate) fn ornstein_uhlenbeck_statistics(y: &[f64], dt: f64) -> OuStatistics {
    let n = y.len() as f64;
    let mean = y.iter().sum::<f64>() / n;
    let variance = y.iter().map(|value| (value - mean).powi(2)).sum::<f64>() / (n - 1.0);
    let (numerator, denominator) = y.windows(2).fold((0.0, 0.0), |(num, den), pair| {
        let (y0, y1) = (pair[0] - mean, pair[1] - mean);
        (num + y0 * y1, den + y0 * y0)
    });
    let phi = numerator / denominator;
    let time_constant = -dt / phi.clamp(1e-6, 0.999_999).ln();
    OuStatistics {
        mean,
        variance,
        phi,
        time_constant,
    }
}

// ---------------------------------------------------------------------------
// End to end: FSW joint control driving a scene
// ---------------------------------------------------------------------------

/// The FSW `hingedJointArrayMotor`, written for MuJoCo scenes, drives a
/// hub's two hinged panels to their desired angles from the scene's mass
/// matrix and joint reactions (`MJSystemMassMatrix`, `MJJointReactionForces`)
/// and joint states, through the panels' motors, all scheduled by a
/// [`Simulation`](crate::simulation::Simulation) at their own rates.
#[test]
fn hinged_joint_array_motor_drives_scene_panels() {
    use crate::fsw_algorithms::effector_interfaces::hinged_joint_array_motor::{
        HingedJointArrayMotor, HingedJointArrayMotorConfig,
    };
    use crate::messages::JointArrayStateMsg;
    use crate::mujoco_dynamics::mj_joint_reaction_forces::MjJointReactionForces;
    use crate::mujoco_dynamics::mj_system_mass_matrix::MjSystemMassMatrix;
    use crate::simulation::Simulation;
    use nalgebra::DMatrix;

    let mut scene = MjScene::new(
        r#"
        <mujoco>
          <worldbody>
            <body name="hub">
              <freejoint/>
              <inertial pos="0 0 0" mass="500" diaginertia="400 400 400"/>
              <body name="panel_1" pos="1 0 0">
                <joint name="hinge_1" type="hinge" axis="0 0 1"/>
                <inertial pos="0.5 0 0" mass="10" diaginertia="1 1 1"/>
              </body>
              <body name="panel_2" pos="-1 0 0">
                <joint name="hinge_2" type="hinge" axis="0 0 1"/>
                <inertial pos="-0.5 0 0" mass="10" diaginertia="1 1 1"/>
              </body>
            </body>
          </worldbody>
        </mujoco>
        "#,
    )
    .expect("scene");
    scene.extra_eom_call = true;

    let desired_angles = [0.5, -0.3];
    let desired = Output::new(JointArrayStateMsg {
        states: desired_angles.to_vec(),
        state_dots: vec![0.0; 2],
        state_ddots: vec![0.0; 2],
    });
    let mut motor = HingedJointArrayMotor::new(HingedJointArrayMotorConfig {
        num_hinged_joints: 2,
        k_theta: DMatrix::identity(2, 2) * 4.0,
        p_theta: DMatrix::identity(2, 2) * 4.0,
        ..HingedJointArrayMotorConfig::default()
    });
    desired.connect_to(&mut motor.des_joint_states_in_msg);
    for (index, joint) in ["hinge_1", "hinge_2"].iter().enumerate() {
        let state = scene.scalar_joint(joint).clone();
        state
            .state_out_msg
            .connect_to(&mut motor.joint_states_in_msgs[index]);
        state
            .state_dot_out_msg
            .connect_to(&mut motor.joint_state_dots_in_msgs[index]);
        let torque = motor.add_motor_torque_output().clone();
        torque.connect_to(
            &mut scene
                .add_joint_single_actuator(&format!("motor_{index}"), joint)
                .actuator_in_msg,
        );
    }
    let mut mass_matrix = MjSystemMassMatrix::new(&scene);
    let mut reactions = MjJointReactionForces::new(&scene);
    mass_matrix
        .mass_matrix_out_msg
        .connect_to(&mut motor.mass_matrix_in_msg);
    reactions
        .reaction_forces_out_msg
        .connect_to(&mut motor.reaction_forces_in_msg);

    let (scene_step, control_step) = (sec2nano(0.01), sec2nano(0.05));
    {
        let mut simulation = Simulation::new(epoch(), false);
        simulation.add_module("scene", &mut scene, scene_step, 30);
        simulation.add_module("massMatrix", &mut mass_matrix, control_step, 20);
        simulation.add_module("reactions", &mut reactions, control_step, 20);
        simulation.add_module("motor", &mut motor, control_step, 10);
        simulation.run_for(sec2nano(8.0));
    }

    for (joint, desired) in ["hinge_1", "hinge_2"].iter().zip(desired_angles) {
        let state = scene.scalar_joint(joint).state_out_msg.read().state;
        let rate = scene.scalar_joint(joint).state_dot_out_msg.read().state;
        assert!(
            (state - desired).abs() < 1e-3,
            "{joint}: {state} vs {desired}"
        );
        assert!(rate.abs() < 1e-2, "{joint} rate {rate}");
    }
}

/// Upstream `MJScalarJoint::updateConstrainedEquality`: while a scalar
/// joint's constrained-state input is connected, its equality is active and
/// holds it at the commanded value, offset from its reference position
/// (`q = qpos0 + value`); a joint without the input stays free.
#[test]
fn constrained_joint_follows_its_commanded_state() {
    let mut scene = MjScene::new(
        r#"
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="a">
              <joint name="slide_a" type="slide" axis="1 0 0"/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body>
            <body name="b" pos="0 2 0">
              <joint name="hinge_b" type="hinge" axis="0 0 1" ref="0.2"/>
              <geom type="sphere" size="0.1" pos="0.5 0 0" mass="1"/>
            </body>
            <body name="c" pos="0 4 0">
              <joint name="slide_c" type="slide" axis="1 0 0"/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body>
          </worldbody>
        </mujoco>
        "#,
    )
    .expect("scene");
    let command_a = Output::new(crate::messages::ScalarJointStateMsg { state: 0.3 });
    let command_b = Output::new(crate::messages::ScalarJointStateMsg { state: -0.4 });
    command_a.connect_to(&mut scene.scalar_joint_mut("slide_a").constrained_state_in_msg);
    command_b.connect_to(&mut scene.scalar_joint_mut("hinge_b").constrained_state_in_msg);
    scene.init();
    scene
        .body_mut("c")
        .set_scalar_joint_velocity("slide_c", 0.1);
    run(&mut scene, sec2nano(0.01), sec2nano(2.0), |_, _| {});

    let a = scene.body("a").scalar_joint_position("slide_a");
    let b = scene.body("b").scalar_joint_position("hinge_b");
    assert!((a - 0.3).abs() < 1e-3, "slide held at {a}");
    assert!((b - (0.2 - 0.4)).abs() < 1e-3, "hinge held at {b}");
    // The bodies move with their joints.
    let origin = scene.body("a").origin().state_out_msg.read().position_m;
    assert!((origin.x - a).abs() < 1e-12, "slide body at {origin}");
    let sigma = scene.body("b").origin().state_out_msg.read().sigma_bn;
    // A hinge rotates its body by `q - ref` from the XML pose.
    assert!(
        (4.0 * sigma.z.atan() - (b - 0.2)).abs() < 1e-9,
        "hinge body at {sigma}"
    );
    let c = scene.body("c").scalar_joint_position("slide_c");
    assert!((c - 0.2).abs() < 1e-9, "free slide at {c}");
    for joint in ["slide_a", "hinge_b"] {
        let equality = scene.scalar_joint(joint).constrained_equality().to_string();
        assert!(scene.is_equality_active(&equality));
    }
    let free = scene
        .scalar_joint("slide_c")
        .constrained_equality()
        .to_string();
    assert!(!scene.is_equality_active(&free));

    // A new command moves the joint.
    command_a.write(crate::messages::ScalarJointStateMsg { state: -0.1 });
    let mut now = sec2nano(2.0);
    for _ in 0..200 {
        now += sec2nano(0.01);
        scene.update(&context(now));
    }
    let a = scene.body("a").scalar_joint_position("slide_a");
    assert!((a + 0.1).abs() < 1e-3, "slide moved to {a}");
}

/// Every basilisk-rs integrator, deterministic and stochastic, advances a
/// scene: a body coasting on a free joint and a slide (exact for any
/// method), and a noise-free mean-reverting task state `x' = -x`, `x(0) =
/// 1`, which reaches `e^-1` to the accuracy of a first-order method at
/// 0.01 s.
#[test]
fn every_integrator_advances_a_scene() {
    use crate::integrators::stochastic::{
        Dri1, Dri1Nm, EulerHeun, EulerMaruyama, Rdi1Wm, Ri1, Ri3, Ri5, Ri6, RkMil, Rs1, Rs2, Siea,
        Sieb, Smea, Smeb, Sosra, Sosri, Sra1, Sriw1, StrongSra, StrongSri, W2Ito1, W2Ito2,
    };
    use crate::integrators::{
        Dopri45, Heun, Integrator, Midpoint, Ralston, Rk4ThreeEighths, Rkf45,
    };
    use crate::mujoco_dynamics::stochastic_atm_density::StochasticAtmDensity;

    let integrators: Vec<(&str, Box<dyn Integrator<MjScene>>)> = vec![
        ("Euler", Box::new(Euler)),
        ("Midpoint", Box::new(Midpoint)),
        ("Heun", Box::new(Heun)),
        ("Ralston", Box::new(Ralston)),
        ("Rk4", Box::new(Rk4)),
        ("Rk4ThreeEighths", Box::new(Rk4ThreeEighths)),
        ("Rkf45", Box::new(Rkf45::default())),
        ("Dopri45", Box::new(Dopri45::default())),
        ("Rkf78", Box::new(Rkf78::default())),
        ("EulerMaruyama", Box::new(EulerMaruyama::new())),
        ("EulerHeun", Box::new(EulerHeun::new())),
        ("RkMil", Box::new(RkMil::new())),
        ("Sra1", Box::new(Sra1::new())),
        ("Sosra", Box::new(Sosra::new())),
        ("Sriw1", Box::new(Sriw1::new())),
        ("Sosri", Box::new(Sosri::new())),
        ("W2Ito1", Box::new(W2Ito1::new())),
        ("W2Ito2", Box::new(W2Ito2::new())),
        ("Dri1", Box::new(Dri1::new())),
        ("Dri1Nm", Box::new(Dri1Nm::new())),
        ("Ri1", Box::new(Ri1::new())),
        ("Ri3", Box::new(Ri3::new())),
        ("Ri5", Box::new(Ri5::new())),
        ("Ri6", Box::new(Ri6::new())),
        ("Rdi1Wm", Box::new(Rdi1Wm::new())),
        ("Rs1", Box::new(Rs1::new())),
        ("Rs2", Box::new(Rs2::new())),
        ("Siea", Box::new(Siea::new())),
        ("Sieb", Box::new(Sieb::new())),
        ("Smea", Box::new(Smea::new())),
        ("Smeb", Box::new(Smeb::new())),
        (
            "StrongSra",
            Box::new(StrongSra::new(crate::integrators::stochastic::SRA1).expect("SRA1")),
        ),
        (
            "StrongSri",
            Box::new(StrongSri::new(crate::integrators::stochastic::SRIW1).expect("SRIW1")),
        ),
    ];
    for (name, integrator) in integrators {
        let mut scene = MjScene::new(
            r#"
            <mujoco>
              <worldbody>
                <body name="hub">
                  <freejoint/>
                  <geom type="sphere" size="0.5" mass="2"/>
                </body>
                <body name="slider" pos="0 3 0">
                  <joint name="slide" type="slide" axis="1 0 0"/>
                  <geom type="sphere" size="0.1" mass="1"/>
                </body>
              </worldbody>
            </mujoco>
            "#,
        )
        .expect("scene");
        scene.set_boxed_integrator(integrator);
        let mut noise = StochasticAtmDensity::default();
        noise.noise.set_state_value(1.0);
        let id = scene.add_model_to_dynamics_task(noise, -1);
        scene.init();
        scene
            .body_mut("hub")
            .set_velocity(Vector3::new(1.0, -2.0, 0.5));
        scene
            .body_mut("slider")
            .set_scalar_joint_velocity("slide", 0.25);
        run(&mut scene, sec2nano(0.01), sec2nano(1.0), |_, _| {});

        let x = scene.task_model(id).noise.state_value();
        assert!((x - (-1.0f64).exp()).abs() < 2.5e-3, "{name}: x = {x}");
        let hub = scene.body("hub").free_joint().qpos_adr();
        let position = &scene.qpos().as_slice()[hub..hub + 3];
        assert_close(position, &[1.0, -2.0, 0.5], 1e-12);
        assert!(
            (scene.body("slider").scalar_joint_position("slide") - 0.25).abs() < 1e-12,
            "{name}"
        );
    }
}
