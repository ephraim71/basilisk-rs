//! Upstream `vizSupport.enableUnityVisualization` for an `MJScene` (its
//! `_handleMJScene`, `_getMJSceneGeomVizInfos`, `_appendMJSceneMultiShapes`
//! and `_createCustomModelsFromMJScene`), for the `mujoco` and `vizard`
//! features:
//!
//! * every scene body becomes a Vizard spacecraft named after it, reading
//!   its origin site's state: first the bodies attached to the world, then
//!   the others, each with its MuJoCo parent as `parentSpacecraftName`;
//! * the first supported geom of a body (box, sphere, cylinder, capsule as a
//!   cylinder, ellipsoid as a sphere) replaces the body's model with a
//!   primitive [`CustomModel`] of the geom's size, pose and color; its other
//!   geoms are drawn as [`MultiShape`]s.
//!
//! ```ignore
//! #[cfg(feature = "vizard")]
//! #[path = "support/mj_viz.rs"]
//! mod mj_viz;
//! ```

#![allow(dead_code)]

use basilisk_rs::mujoco_dynamics::{MjGeomInfo, MjScene};
use basilisk_rs::utilities::rigid_body_kinematics::{ep_to_euler321, ep_to_mrp};
use basilisk_rs::vizard::viz_interface::VizInterface;
use basilisk_rs::vizard::viz_structures::{
    CustomModel, MultiShape, Rgba, VizSetting, VizSpacecraft,
};
use nalgebra::{Vector3, Vector4};

// MuJoCo `mjtGeom` codes with a Vizard primitive.
const MJGEOM_SPHERE: i32 = 2;
const MJGEOM_CAPSULE: i32 = 3;
const MJGEOM_ELLIPSOID: i32 = 4;
const MJGEOM_CYLINDER: i32 = 5;
const MJGEOM_BOX: i32 = 6;

/// Upstream's per-geom Vizard data (`geomToVizInfo`).
#[derive(Clone, Debug, PartialEq)]
pub struct GeomVizInfo {
    pub body_name: String,
    pub model_path: &'static str,
    pub custom_scale: Vector3<f64>,
    pub multi_shape_dimensions: Vector3<f64>,
    pub offset: Vector3<f64>,
    /// 3-2-1 Euler angles of the geom in the body frame.
    pub custom_rotation: Vector3<f64>,
    /// MRP of the geom in the body frame.
    pub multi_shape_rotation: Vector3<f64>,
    pub color: Rgba,
}

/// Upstream `geomToVizInfo`, `None` for a geom type without a Vizard
/// primitive (meshes, planes...).
pub fn geom_viz_info(geom: &MjGeomInfo) -> Option<GeomVizInfo> {
    let [s0, s1, s2] = geom.size;
    let (model_path, scale) = match geom.geom_type {
        MJGEOM_BOX => ("CUBE", Vector3::new(2.0 * s0, 2.0 * s1, 2.0 * s2)),
        MJGEOM_SPHERE => ("SPHERE", Vector3::repeat(2.0 * s0)),
        // Vizard's cylinder takes the half-height along z; a capsule is
        // drawn as a cylinder.
        MJGEOM_CYLINDER | MJGEOM_CAPSULE => ("CYLINDER", Vector3::new(2.0 * s0, 2.0 * s0, s1)),
        MJGEOM_ELLIPSOID => ("SPHERE", Vector3::new(2.0 * s0, 2.0 * s1, 2.0 * s2)),
        _ => return None,
    };
    let quaternion = Vector4::from(geom.quat);
    Some(GeomVizInfo {
        body_name: geom.body_name.clone(),
        model_path,
        custom_scale: scale,
        multi_shape_dimensions: scale,
        offset: Vector3::from(geom.pos),
        custom_rotation: ep_to_euler321(quaternion),
        multi_shape_rotation: ep_to_mrp(quaternion),
        color: geom.rgba.map(|channel| (channel * 255.0) as u8),
    })
}

/// Upstream `_getMJSceneGeomVizInfos`: the supported geoms grouped by body,
/// in order of each body's first geom.
pub fn scene_geom_viz_infos(scene: &MjScene) -> Vec<(String, Vec<GeomVizInfo>)> {
    let mut groups: Vec<(String, Vec<GeomVizInfo>)> = Vec::new();
    for info in scene.geom_infos().iter().filter_map(geom_viz_info) {
        match groups.iter_mut().find(|(body, _)| *body == info.body_name) {
            Some((_, infos)) => infos.push(info),
            None => groups.push((info.body_name.clone(), vec![info])),
        }
    }
    groups
}

/// Upstream `_makeMJSceneMultiShape`.
fn multi_shape(info: &GeomVizInfo) -> MultiShape {
    MultiShape {
        is_on: VizSetting::Enabled,
        position_m: info.offset,
        current_value_c: 1.0,
        max_value_c: 1.0,
        positive_color: Some(info.color),
        negative_color: Some(info.color),
        shape: info.model_path.to_string(),
        dimensions_m: info.multi_shape_dimensions,
        rotation: info.multi_shape_rotation,
        ..MultiShape::default()
    }
}

/// Upstream `_handleMJScene` with `_createCustomModelsFromMJScene`: adds
/// every body of `scene` to `viz` and returns the names of the spacecraft
/// it added, in the order added.
///
/// # Panics
///
/// If the scene has no body attached to the world (upstream raises
/// `ValueError`).
pub fn add_scene(viz: &mut VizInterface, scene: &MjScene) -> Vec<String> {
    let names = scene.body_names();
    let parent = |name: &str| {
        scene
            .body_parent_name(name)
            .expect("scene body has a parent")
    };
    let roots: Vec<&String> = names
        .iter()
        .filter(|name| parent(name) == "world")
        .collect();
    assert!(
        !roots.is_empty(),
        "MJScene has no world-parented body. Cannot determine Vizard spacecraft roots."
    );
    let geoms = scene_geom_viz_infos(scene);
    let extra_geoms = |name: &str| -> Vec<MultiShape> {
        geoms
            .iter()
            .find(|(body, _)| body == name)
            .map(|(_, infos)| infos.iter().skip(1).map(multi_shape).collect())
            .unwrap_or_default()
    };
    let mut added = Vec::new();
    let children = names.iter().filter(|name| parent(name) != "world");
    for (name, is_root) in roots
        .iter()
        .map(|name| (*name, true))
        .chain(children.map(|name| (name, false)))
    {
        let spacecraft = viz.add_spacecraft(name.clone());
        if !is_root {
            spacecraft.parent_spacecraft_name = parent(name);
        }
        scene
            .body(name)
            .origin()
            .state_out_msg
            .connect_to(&mut spacecraft.sc_state_in_msg);
        spacecraft.msm_info.msm_list.extend(extra_geoms(name));
        added.push(name.clone());
    }
    for (body, infos) in &geoms {
        let base = &infos[0];
        viz.settings.custom_model_list.push(CustomModel {
            model_path: base.model_path.to_string(),
            sim_bodies_to_modify: vec![body.clone()],
            offset_m: base.offset,
            rotation_rad: base.custom_rotation,
            scale: base.custom_scale,
            color: Some(base.color),
            ..CustomModel::default()
        });
    }
    added
}

/// The spacecraft entry named `name`.
///
/// # Panics
///
/// If `viz` has no such spacecraft (upstream raises `ValueError`).
pub fn spacecraft_mut<'a>(viz: &'a mut VizInterface, name: &str) -> &'a mut VizSpacecraft {
    viz.spacecraft
        .iter_mut()
        .find(|spacecraft| spacecraft.spacecraft_name == name)
        .unwrap_or_else(|| panic!("Could not find spacecraft '{name}' in Vizard spacecraft data."))
}

/// Upstream `vizSupport.toRGBA255` for the named colors the MuJoCo
/// scenarios use.
pub fn rgba(name: &str) -> Rgba {
    match name {
        "yellow" => [255, 255, 0, 255],
        "cyan" => [0, 255, 255, 255],
        "turquoise" => [64, 224, 208, 255],
        _ => panic!("no color named {name}"),
    }
}
