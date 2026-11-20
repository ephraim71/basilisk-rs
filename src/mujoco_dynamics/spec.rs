//! The MuJoCo model specification behind a scene: port of upstream Basilisk
//! `simulation/mujocoDynamics/_GeneralModuleFiles/MJSpec`.
//!
//! Upstream keeps a live `mjSpec` next to the compiled model and edits it in
//! place (new sites, actuators and joint equalities), recompiling when the
//! model goes stale. mujoco-rs's [`MjSpec`] is not `Send`, and a scene must
//! be, so here the scene keeps the source XML and the list of edits made to
//! it instead, and every compilation parses the XML again and replays the
//! edits. Parsing is deterministic, so the replayed specification is the one
//! upstream would hold.
//!
//! Every parse also applies upstream's normalization: gravity is switched off
//! (forces come from models such as [`NBodyGravity`](super::n_body_gravity::NBodyGravity)),
//! and unnamed bodies get scene-local `_nameless_<k>` names, reserving every
//! explicit name first (upstream `nameUnnamedBodies`). Upstream names the
//! other unnamed objects (joints, sites, actuators, equalities) from a
//! process-wide counter; here they are named per scene with the same
//! reservation rule, so identical XML always yields identical names.

use std::collections::HashSet;
use std::fmt;
use std::path::{Path, PathBuf};

use mujoco_rs::prelude::*;
use mujoco_rs::wrappers::mj_auxiliary::MjVfs;
use mujoco_rs::wrappers::mj_editing::{MjsBody, SpecElement, SpecItem};

/// Name of the in-memory file that holds the scene XML when the scene also
/// has asset files, so MuJoCo can resolve them from one virtual file system.
const SCENE_XML_FILE: &str = "__basilisk_scene__.xml";

/// Errors of scene construction, compilation and export.
#[derive(Clone, Debug, PartialEq)]
pub enum MjSceneError {
    /// The XML could not be parsed (upstream `mj_parseXMLString` error).
    Parse(String),
    /// The specification could not be compiled (upstream `mj_compile`
    /// error), with MuJoCo's message.
    Compile(String),
    /// An asset file could not be loaded.
    File(String),
    /// No object of the given kind and name exists.
    NotFound {
        /// Object kind ("body", "site", "joint", "actuator", "equality").
        kind: &'static str,
        /// The name looked up.
        name: String,
    },
    /// An object with the given name already exists.
    AlreadyExists {
        /// Object kind.
        kind: &'static str,
        /// The duplicate name.
        name: String,
    },
    /// An object exists but is not of the expected kind (upstream "it is not
    /// of the expected type").
    WrongKind {
        /// Object kind.
        kind: &'static str,
        /// The name looked up.
        name: String,
    },
    /// A recompiled model would change the finalized state layout.
    Topology(String),
    /// The model could not be written (upstream "Failed to save MuJoCo
    /// XML").
    Save(String),
}

impl fmt::Display for MjSceneError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Parse(message) => write!(f, "failed to parse the MuJoCo XML: {message}"),
            Self::Compile(message) => write!(f, "failed to compile the MuJoCo model: {message}"),
            Self::File(message) => write!(f, "failed to load a MuJoCo asset file: {message}"),
            Self::NotFound { kind, name } => write!(f, "unknown {kind} '{name}' in MJScene"),
            Self::AlreadyExists { kind, name } => {
                write!(f, "a {kind} named '{name}' already exists in MJScene")
            }
            Self::WrongKind { kind, name } => {
                write!(f, "{kind} '{name}' is not of the expected type")
            }
            Self::Topology(message) => write!(f, "{message}"),
            Self::Save(message) => write!(f, "Failed to save MuJoCo XML to {message}"),
        }
    }
}

impl std::error::Error for MjSceneError {}

/// What a new actuator transmits to (upstream `mjTRN_JOINT` or
/// `mjTRN_SITE`).
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum ActuatorTarget {
    Joint(String),
    Site(String),
}

/// One recorded change to the parsed XML, replayed on every compilation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum SpecEdit {
    /// Upstream `MJBody::addSite`.
    AddSite {
        body: String,
        name: String,
        pos: [f64; 3],
        quat: [f64; 4],
    },
    /// Upstream `MJSite::setPositionRelativeToBody` (and the attitude
    /// setter) on a site the XML or an earlier edit made.
    SitePose {
        site: String,
        pos: Option<[f64; 3]>,
        quat: Option<[f64; 4]>,
    },
    /// Upstream `MJSpec::addJointSingleActuator`, `addSingleActuator` and
    /// the sub-actuators of `createActuator`: a direct-drive actuator
    /// (`dyntype` none, fixed unit gain, no bias).
    AddActuator {
        name: String,
        target: ActuatorTarget,
        gear: [f64; 6],
    },
    /// Upstream `createConstrainedEquality`, which adds an inactive
    /// single-joint equality with zero data. Here the equality holds a fixed
    /// tendon `tendon` of unit coefficient on `joint`, whose reference
    /// length can be moved at run time (see [`MjScene`](super::MjScene)).
    AddJointEquality {
        name: String,
        tendon: String,
        joint: String,
    },
    /// Upstream `MJEquality::setSolref`.
    EqualitySolref { name: String, solref: [f64; 2] },
    /// Upstream `MJEquality::setSolimp`.
    EqualitySolimp { name: String, solimp: [f64; 5] },
}

/// The source of a scene's MuJoCo model: the XML, its asset files and the
/// edits made since.
#[derive(Clone, Debug)]
pub(crate) struct SceneSpec {
    xml: String,
    files: Vec<PathBuf>,
    edits: Vec<SpecEdit>,
}

impl SceneSpec {
    pub(crate) fn new(xml: impl Into<String>, files: Vec<PathBuf>) -> Self {
        Self {
            xml: xml.into(),
            files,
            edits: Vec::new(),
        }
    }

    pub(crate) fn push(&mut self, edit: SpecEdit) {
        self.edits.push(edit);
    }

    /// A virtual file system with the asset files and, when there are any,
    /// the scene XML, or `None` for a self-contained XML.
    fn virtual_file_system(&self) -> Result<Option<MjVfs>, MjSceneError> {
        if self.files.is_empty() {
            return Ok(None);
        }
        let mut vfs = MjVfs::new();
        for file in &self.files {
            vfs.add_file(file).map_err(|error| {
                MjSceneError::File(format!("error loading file {}: {error}", file.display()))
            })?;
        }
        vfs.add_from_buffer(SCENE_XML_FILE, self.xml.as_bytes())
            .map_err(|error| MjSceneError::File(format!("error loading the scene XML: {error}")))?;
        Ok(Some(vfs))
    }

    /// Parses the XML, normalizes it and replays the edits.
    pub(crate) fn build(&self) -> Result<MjSpec, MjSceneError> {
        let vfs = self.virtual_file_system()?;
        let mut spec = match &vfs {
            Some(vfs) => MjSpec::from_xml_vfs(SCENE_XML_FILE, vfs),
            None => MjSpec::from_xml_string(&self.xml),
        }
        .map_err(|error| MjSceneError::Parse(error.to_string()))?;
        spec.option_mut().gravity = [0.0; 3];
        name_unnamed_bodies(&mut spec)?;
        name_unnamed_objects(&mut spec)?;
        for edit in &self.edits {
            apply_edit(&mut spec, edit)?;
        }
        Ok(spec)
    }

    /// Builds and compiles the specification.
    pub(crate) fn compile(&self) -> Result<MjModel, MjSceneError> {
        let mut spec = self.build()?;
        let vfs = self.virtual_file_system()?;
        match &vfs {
            Some(vfs) => spec.compile_with_vfs(vfs),
            None => spec.compile(),
        }
        .map_err(|error| MjSceneError::Compile(error.to_string()))
    }

    /// Builds the specification, compiles it (MuJoCo exports a compiled
    /// specification, generated sites included) and writes it as XML.
    pub(crate) fn save_xml(&self, path: &Path) -> Result<(), MjSceneError> {
        let mut spec = self.build()?;
        let vfs = self.virtual_file_system()?;
        match &vfs {
            Some(vfs) => spec.compile_with_vfs(vfs),
            None => spec.compile(),
        }
        .map_err(|error| MjSceneError::Compile(error.to_string()))?;
        spec.save_xml(path)
            .map_err(|error| MjSceneError::Save(format!("'{}': {error}", path.display())))
    }
}

/// Next `_nameless_<k>` name not in `used`, which it then reserves.
fn next_nameless(used: &mut HashSet<String>, counter: &mut usize) -> String {
    loop {
        let name = format!("_nameless_{counter}");
        *counter += 1;
        if used.insert(name.clone()) {
            return name;
        }
    }
}

/// Upstream `nameUnnamedBodies`: every explicit body name is reserved first,
/// then the unnamed bodies are named in specification (depth-first) order.
fn name_unnamed_bodies(spec: &mut MjSpec) -> Result<(), MjSceneError> {
    let mut used: HashSet<String> = spec
        .body_iter()
        .map(|body| body.name().to_string())
        .filter(|name| !name.is_empty())
        .collect();
    let mut counter = 0;

    fn walk(
        body: &mut MjsBody,
        used: &mut HashSet<String>,
        counter: &mut usize,
    ) -> Result<(), MjSceneError> {
        for child in body.body_iter_mut() {
            if child.name().is_empty() {
                let name = next_nameless(used, counter);
                child
                    .set_name(&name)
                    .map_err(|error| MjSceneError::Parse(format!("naming body {name}: {error}")))?;
            }
            walk(child, used, counter)?;
        }
        Ok(())
    }

    walk(spec.world_body_mut(), &mut used, &mut counter)
}

/// Names the unnamed joints, sites, actuators and equalities, one
/// `_nameless_<k>` sequence per object kind.
fn name_unnamed_objects(spec: &mut MjSpec) -> Result<(), MjSceneError> {
    macro_rules! name_kind {
        ($iter:ident, $iter_mut:ident) => {{
            let mut used: HashSet<String> = spec
                .$iter()
                .map(|item| item.name().to_string())
                .filter(|name| !name.is_empty())
                .collect();
            let mut counter = 0;
            for item in spec.$iter_mut() {
                if item.name().is_empty() {
                    let name = next_nameless(&mut used, &mut counter);
                    item.set_name(&name).map_err(|error| {
                        MjSceneError::Parse(format!("naming object {name}: {error}"))
                    })?;
                }
            }
        }};
    }
    name_kind!(joint_iter, joint_iter_mut);
    name_kind!(site_iter, site_iter_mut);
    name_kind!(actuator_iter, actuator_iter_mut);
    name_kind!(equality_iter, equality_iter_mut);
    Ok(())
}

fn apply_edit(spec: &mut MjSpec, edit: &SpecEdit) -> Result<(), MjSceneError> {
    let not_found = |kind, name: &str| MjSceneError::NotFound {
        kind,
        name: name.to_string(),
    };
    let exists = |kind, name: &str| MjSceneError::AlreadyExists {
        kind,
        name: name.to_string(),
    };
    match edit {
        SpecEdit::AddSite {
            body,
            name,
            pos,
            quat,
        } => {
            let body = spec.body_mut(body).ok_or_else(|| not_found("body", body))?;
            let site = body.add_site();
            site.set_name(name).map_err(|_| exists("site", name))?;
            *site.pos_mut() = *pos;
            *site.quat_mut() = *quat;
        }
        SpecEdit::SitePose { site, pos, quat } => {
            let site_spec = spec.site_mut(site).ok_or_else(|| not_found("site", site))?;
            if let Some(pos) = pos {
                *site_spec.pos_mut() = *pos;
            }
            if let Some(quat) = quat {
                *site_spec.quat_mut() = *quat;
            }
        }
        SpecEdit::AddActuator { name, target, gear } => {
            let actuator = spec.add_actuator();
            actuator
                .set_name(name)
                .map_err(|_| exists("actuator", name))?;
            match target {
                ActuatorTarget::Joint(joint) => {
                    actuator.set_trntype(MjtTrn::mjTRN_JOINT);
                    actuator.set_target(joint);
                }
                ActuatorTarget::Site(site) => {
                    actuator.set_trntype(MjtTrn::mjTRN_SITE);
                    actuator.set_target(site);
                }
            }
            actuator.set_dyntype(MjtDyn::mjDYN_NONE);
            actuator.set_gaintype(MjtGain::mjGAIN_FIXED);
            actuator.set_biastype(MjtBias::mjBIAS_NONE);
            actuator.gainprm_mut()[0] = 1.0;
            *actuator.gear_mut() = *gear;
        }
        SpecEdit::AddJointEquality {
            name,
            tendon,
            joint,
        } => {
            let fixed = spec.add_tendon();
            fixed
                .set_name(tendon)
                .map_err(|_| exists("tendon", tendon))?;
            fixed.wrap_joint(joint, 1.0);
            let equality = spec.add_equality();
            equality
                .set_name(name)
                .map_err(|_| exists("equality", name))?;
            equality.set_type(MjtEq::mjEQ_TENDON);
            equality.set_name1(tendon);
            equality.set_active(false);
            equality.data_mut().fill(0.0);
        }
        SpecEdit::EqualitySolref { name, solref } => {
            let equality = spec
                .equality_mut(name)
                .ok_or_else(|| not_found("equality", name))?;
            *equality.solref_mut() = *solref;
        }
        SpecEdit::EqualitySolimp { name, solimp } => {
            let equality = spec
                .equality_mut(name)
                .ok_or_else(|| not_found("equality", name))?;
            *equality.solimp_mut() = *solimp;
        }
    }
    Ok(())
}

/// The bodies, joints, sites and composite-actuator declarations of a
/// parsed specification, read once at scene construction.
#[derive(Clone, Debug, Default)]
pub(crate) struct SpecContents {
    /// Bodies other than the world body, in specification order, each with
    /// its sites and its joints (with their kinds), in specification order.
    pub(crate) bodies: Vec<SpecBody>,
    /// Every actuator name.
    pub(crate) actuators: Vec<String>,
    /// Every equality name.
    pub(crate) equalities: Vec<String>,
    /// The `<custom><text name="basilisk:...">` entries upstream reads to
    /// build composite actuators: `(kind, name, site hint)`.
    pub(crate) composite_actuators: Vec<(CompositeKind, String, String)>,
}

#[derive(Clone, Debug)]
pub(crate) struct SpecBody {
    pub(crate) name: String,
    pub(crate) sites: Vec<String>,
    pub(crate) joints: Vec<(String, MjtJoint)>,
}

/// The three composite actuator kinds upstream declares through custom text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CompositeKind {
    Force,
    Torque,
    ForceTorque,
}

impl CompositeKind {
    /// Upstream `createActuator`'s sub-actuator suffixes and gear indices.
    pub(crate) fn sub_actuators(self) -> &'static [(&'static str, usize)] {
        match self {
            Self::Force => &[("_fx", 0), ("_fy", 1), ("_fz", 2)],
            Self::Torque => &[("_mx", 3), ("_my", 4), ("_mz", 5)],
            Self::ForceTorque => &[
                ("_fx", 0),
                ("_fy", 1),
                ("_fz", 2),
                ("_mx", 3),
                ("_my", 4),
                ("_mz", 5),
            ],
        }
    }
}

/// Reads the bodies, actuators, equalities and composite-actuator
/// declarations of `spec`.
pub(crate) fn read_contents(spec: &MjSpec) -> SpecContents {
    let bodies = spec
        .body_iter()
        .skip(1)
        .map(|body| SpecBody {
            name: body.name().to_string(),
            sites: body
                .site_iter(false)
                .map(|site| site.name().to_string())
                .collect(),
            joints: body
                .joint_iter(false)
                .map(|joint| (joint.name().to_string(), joint.type_()))
                .collect(),
        })
        .collect();
    let actuators = spec
        .actuator_iter()
        .map(|actuator| actuator.name().to_string())
        .collect();
    let equalities = spec
        .equality_iter()
        .map(|equality| equality.name().to_string())
        .collect();

    // Upstream `readCustomDoubleSplit`: whitespace-separated `name@site`
    // tokens, the site hint optional.
    let mut composite_actuators = Vec::new();
    for (key, kind) in [
        ("basilisk:forceactuator", CompositeKind::Force),
        ("basilisk:torqueactuator", CompositeKind::Torque),
        ("basilisk:forcetorqueactuator", CompositeKind::ForceTorque),
    ] {
        let Some(text) = spec.text_iter().find(|text| text.name() == key) else {
            continue;
        };
        for token in text.data().split_whitespace() {
            let (name, site) = token.split_once('@').unwrap_or((token, ""));
            composite_actuators.push((kind, name.to_string(), site.to_string()));
        }
    }

    SpecContents {
        bodies,
        actuators,
        equalities,
        composite_actuators,
    }
}
