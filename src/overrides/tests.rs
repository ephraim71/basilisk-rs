//! Fixtures here are deliberately generic. They exercise the registry, not any
//! particular instrument, so they name no real device, field or calibration
//! value — an application's hardware vocabulary has no business in this crate.

use serde::{Deserialize, Serialize};
use serde_json::json;

use crate::messages::{Input, Output};

use super::paths::payload_paths;
use super::schema::leaf_fields;
use crate::overrides::{Mode, Registry, TargetKind};

/// A kind this crate does not define, declared the way an application declares
/// one for itself. Several fixtures register a config-shaped type, and routing
/// them through [`TargetKind::Custom`] exercises the extension point directly:
/// the registry stores and reports a name it has never heard of, and such a
/// target accepts exactly what a built-in kind accepts.
const CONFIG: TargetKind = TargetKind::Custom("config");

/// Stands in for a published measurement: a few scalars and one integer.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(default)]
struct ReadingMsg {
    value: f64,
    quality_percent: f64,
    angle_deg: f64,
    rate_dps: f64,
    status: u8,
}

/// Stands in for a device config: a live field, a scale factor, a rate, and one
/// consumed only at construction.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(default)]
struct DeviceConfig {
    bias: f64,
    scale: f64,
    sample_hz: f64,
    rng_seed: u64,
}

fn device_config() -> DeviceConfig {
    DeviceConfig {
        bias: 0.0,
        scale: 4.0,
        sample_hz: 200.0,
        rng_seed: 7,
    }
}

fn reading(angle_deg: f64) -> ReadingMsg {
    ReadingMsg {
        value: 12.5,
        quality_percent: 91.0,
        angle_deg,
        rate_dps: -1.5,
        status: 0,
    }
}

fn registry_with_reading() -> (Registry, Output<ReadingMsg>) {
    let registry = Registry::new();
    let output = Output::new(reading(3.25));
    registry
        .register("SENSOR", "sensor_0.output_msg", &output, TargetKind::Output)
        .unwrap();
    (registry, output)
}

fn registry_with_config() -> (Registry, Output<DeviceConfig>) {
    let registry = Registry::new();
    let config = Output::new(device_config());
    registry
        .register("SENSOR", "sensor_0.config", &config, CONFIG)
        .unwrap();
    (registry, config)
}

fn paths(registry: &Registry, target: &str) -> Vec<String> {
    registry
        .spec(target)
        .unwrap()
        .fields
        .iter()
        .map(|field| field.path.clone())
        .collect()
}

/// The schema and the payload check share one walk, so what counts as a leaf is
/// one decision rather than two that can drift apart. Arrays are leaves because
/// a JSON merge replaces them wholesale; an empty object is a leaf because there
/// is nothing under it to name; the root is not a field.
#[test]
fn the_schema_and_the_payload_check_agree_on_what_a_leaf_is() {
    let value = json!({
        "flat": 1.0,
        "nested": { "inner": 2.0, "deeper": { "leaf": 3.0 } },
        "list": [1.0, 2.0],
        "empty": {},
    });

    let described: Vec<String> = leaf_fields(&value)
        .into_iter()
        .map(|field| field.path)
        .collect();

    assert_eq!(
        described,
        [
            "empty",
            "flat",
            "list",
            "nested.deeper.leaf",
            "nested.inner"
        ]
    );
    assert_eq!(
        payload_paths(&value),
        described,
        "a payload addresses different paths than the schema advertises"
    );

    let kinds: Vec<&str> = leaf_fields(&value).iter().map(|field| field.kind).collect();
    assert_eq!(kinds, ["object", "number", "array", "number", "number"]);
}

#[test]
fn spec_lists_the_leaf_fields_of_the_type_default() {
    let (registry, _output) = registry_with_reading();

    assert_eq!(
        paths(&registry, "sensor_0.output_msg"),
        [
            "angle_deg",
            "quality_percent",
            "rate_dps",
            "status",
            "value"
        ]
    );
}

#[test]
fn spec_reports_the_type_default_not_the_configured_value() {
    let (registry, _config) = registry_with_config();

    let spec = registry.spec("sensor_0.config").unwrap();

    assert_eq!(spec.kind, CONFIG);
    // 4.0 is this instance's configured scale; the spec reports the type's.
    assert_eq!(spec.type_default["scale"], json!(0.0));
}

#[test]
fn a_misspelled_field_is_rejected_rather_than_silently_ignored() {
    let (registry, output) = registry_with_reading();

    let error = registry
        .install(
            "sensor_0.output_msg",
            Mode::Patch,
            json!({ "angle_dge": 10.0 }),
        )
        .unwrap_err()
        .to_string();

    assert!(error.contains("angle_dge"), "unhelpful error: {error}");
    assert!(error.contains("did you mean 'angle_deg'"), "{error}");
    assert!(!output.is_overridden());
}

#[test]
fn accumulated_patches_survive_a_rejected_one() {
    let (registry, output) = registry_with_reading();
    registry
        .install(
            "sensor_0.output_msg",
            Mode::Patch,
            json!({ "angle_deg": 10.0 }),
        )
        .unwrap();

    let _ = registry.install(
        "sensor_0.output_msg",
        Mode::Patch,
        json!({ "rate_dps": "sideways" }),
    );
    output.write(reading(3.25));

    assert_eq!(output.read().angle_deg, 10.0);
    assert_eq!(output.read().rate_dps, -1.5);
}

#[test]
fn upstream_and_effective_diverge_under_an_override() {
    let (registry, output) = registry_with_reading();

    registry
        .install(
            "sensor_0.output_msg",
            Mode::Patch,
            json!({ "angle_deg": 10.0 }),
        )
        .unwrap();
    output.write(reading(3.25));

    let upstream = registry.upstream("sensor_0.output_msg").unwrap();
    let effective = registry.effective("sensor_0.output_msg").unwrap();

    assert_eq!(upstream["angle_deg"], json!(3.25));
    assert_eq!(effective["angle_deg"], json!(10.0));
    assert_eq!(upstream["rate_dps"], effective["rate_dps"]);
}

#[test]
fn registering_the_same_name_twice_is_an_error() {
    let (registry, _output) = registry_with_reading();
    let other = Output::new(reading(0.0));

    let error = registry
        .register("SENSOR", "sensor_0.output_msg", &other, TargetKind::Output)
        .unwrap_err()
        .to_string();

    assert!(error.contains("sensor_0.output_msg"));
}

#[test]
fn an_input_target_overrides_only_that_consumer() {
    let registry = Registry::new();
    let producer = Output::new(reading(3.25));
    let mut faulted: Input<ReadingMsg> = Input::default();
    let mut healthy: Input<ReadingMsg> = Input::default();
    producer.connect_to(&mut faulted);
    producer.connect_to(&mut healthy);
    registry
        .register(
            "SENSOR",
            "consumer_0.input_msg",
            &faulted,
            TargetKind::Input,
        )
        .unwrap();

    registry
        .install(
            "consumer_0.input_msg",
            Mode::Patch,
            json!({ "angle_deg": 10.0 }),
        )
        .unwrap();

    assert_eq!(faulted.read().angle_deg, 10.0);
    assert_eq!(healthy.read().angle_deg, 3.25);
    assert_eq!(producer.read().angle_deg, 3.25);
}

/// An input that is registered before it is wired would report `T::default()` as
/// its upstream, which reads as real data. Refusing the registration is the only
/// point at which that is still detectable.
#[test]
fn registering_an_unconnected_input_is_refused() {
    let registry = Registry::new();
    let orphan: Input<ReadingMsg> = Input::default();

    let error = registry
        .register("SENSOR", "consumer_0.input_msg", &orphan, TargetKind::Input)
        .unwrap_err()
        .to_string();

    assert!(error.contains("consumer_0.input_msg"), "{error}");
    assert!(error.contains("connected"), "{error}");
}

// ---------------------------------------------------------------------------
// `replace` means all of it
// ---------------------------------------------------------------------------

/// The failure this closes: `#[serde(default)]` lets a one-field document
/// deserialise, so the omitted fields come back as `T::default()`. Before this
/// check the command below was accepted, reported as installed, and silently
/// reset `sample_hz` from 200 to 0 — a worse outcome than any fault the operator
/// was trying to inject.
#[test]
fn a_partial_replace_is_refused_rather_than_defaulting_the_fields_it_omits() {
    let (registry, config) = registry_with_config();

    let error = registry
        .install("sensor_0.config", Mode::Replace, json!({ "scale": 1.0 }))
        .expect_err("a partial replace was accepted")
        .to_string();

    assert!(error.contains("bias"), "{error}");
    assert!(error.contains("sample_hz"), "{error}");
    assert!(error.contains("rng_seed"), "{error}");
    assert!(error.contains("patch"), "no way out offered: {error}");

    // The configured baseline is untouched — nothing was installed.
    assert_eq!(config.read().sample_hz, 200.0);
    assert_eq!(config.read().scale, 4.0);
    assert!(!config.is_overridden());
}

#[test]
fn a_replace_naming_every_field_is_accepted() {
    let (registry, config) = registry_with_config();

    registry
        .install(
            "sensor_0.config",
            Mode::Replace,
            json!({ "bias": 1.0, "scale": 2.0, "sample_hz": 50.0, "rng_seed": 3 }),
        )
        .expect("a complete replace was refused");

    assert_eq!(config.read().sample_hz, 50.0);
    assert_eq!(config.read().scale, 2.0);
}

/// The other modes are partial by definition or carry no payload at all, so the
/// completeness rule must not reach them.
#[test]
fn only_replace_has_to_be_complete() {
    let (registry, config) = registry_with_config();

    registry
        .install("sensor_0.config", Mode::Patch, json!({ "scale": 1.0 }))
        .expect("a one-field patch was refused");
    assert_eq!(config.read().scale, 1.0);
    assert_eq!(
        config.read().sample_hz,
        200.0,
        "patch touched a field it did not name"
    );

    for mode in [Mode::Freeze, Mode::Default] {
        registry
            .install("sensor_0.config", mode, json!(null))
            .unwrap_or_else(|err| panic!("{mode:?} was refused as incomplete: {err}"));
    }
}

/// Both checks run against one spec, and the unknown-path check runs first. A
/// misspelling in an otherwise complete replace should read as the typo it is,
/// not as a missing field plus a mystery one.
#[test]
fn a_typo_in_a_complete_replace_reports_the_typo_not_the_shortfall() {
    let (registry, _config) = registry_with_config();

    let error = registry
        .install(
            "sensor_0.config",
            Mode::Replace,
            json!({ "bias": 1.0, "scael": 2.0, "sample_hz": 50.0, "rng_seed": 3 }),
        )
        .expect_err("a misspelled field was accepted")
        .to_string();

    assert!(error.contains("unknown field 'scael'"), "{error}");
    assert!(error.contains("did you mean 'scale'"), "{error}");
    assert!(
        !error.contains("omits"),
        "the shortfall masked the typo: {error}"
    );
}

/// A `replace` hides the patch beneath it while installed; removing the replace
/// uncovers it again, and the patch's own id still names its own layer.
#[test]
fn a_replace_hides_a_patch_rather_than_destroying_it() {
    let (registry, output) = registry_with_reading();
    let patch = registry
        .install(
            "sensor_0.output_msg",
            Mode::Patch,
            json!({ "angle_deg": 10.0 }),
        )
        .unwrap();
    let replace = registry
        .install(
            "sensor_0.output_msg",
            Mode::Replace,
            serde_json::to_value(reading(20.0)).unwrap(),
        )
        .unwrap();

    assert_eq!(output.read().angle_deg, 20.0);

    assert!(registry.clear_rule("sensor_0.output_msg", replace).unwrap());
    assert_eq!(
        output.read().angle_deg,
        10.0,
        "the patch under the replace did not come back"
    );

    assert!(registry.clear_rule("sensor_0.output_msg", patch).unwrap());
    assert_eq!(output.read().angle_deg, 3.25);

    // An id naming no installed layer clears nothing.
    assert!(!registry.clear_rule("sensor_0.output_msg", patch).unwrap());
}

/// An id that cleared nothing and a target that does not exist are different
/// answers. Reporting both as `false` would let a mistyped name read as a fault
/// that had already expired.
#[test]
fn clearing_by_id_on_an_unknown_target_is_an_error_not_a_false() {
    let (registry, _output) = registry_with_reading();
    let id = registry
        .install(
            "sensor_0.output_msg",
            Mode::Patch,
            json!({ "angle_deg": 10.0 }),
        )
        .unwrap();

    let error = registry
        .clear_rule("sensor_0.output_mgs", id)
        .unwrap_err()
        .to_string();

    assert!(error.contains("sensor_0.output_mgs"));
}

#[test]
fn an_unknown_target_is_an_error() {
    let (registry, _output) = registry_with_reading();

    let error = registry
        .install("nope.output_msg", Mode::Patch, json!({}))
        .unwrap_err()
        .to_string();

    assert!(error.contains("nope.output_msg"));
    assert!(
        !error.contains("registered no override targets"),
        "a populated registry reported itself as empty: {error}"
    );
}

/// A simulation that wired nothing up refuses every command, and saying
/// "unknown target" sends an operator to check a spelling that was never wrong.
#[test]
fn an_empty_registry_says_so_rather_than_blaming_the_name() {
    let registry = Registry::new();

    let error = registry
        .install("sensor_0.config", Mode::Patch, json!({}))
        .unwrap_err()
        .to_string();

    assert!(
        error.contains("registered no override targets"),
        "unhelpful error: {error}"
    );
    assert!(error.contains("sensor_0.config"), "{error}");
}

/// The group is a deployment role, not a property of the type: the same module
/// type can be one unit in one role and an array of them in another. So it has
/// to be supplied per registration rather than derived from the type.
#[test]
fn one_module_type_can_be_registered_under_two_groups() {
    let registry = Registry::new();
    let primary = Output::new(reading(1.0));
    let redundant = Output::new(reading(2.0));
    registry
        .register(
            "PRIMARY",
            "sensor_p.output_msg",
            &primary,
            TargetKind::Output,
        )
        .unwrap();
    registry
        .register(
            "ARRAY",
            "sensor_a0.output_msg",
            &redundant,
            TargetKind::Output,
        )
        .unwrap();

    assert_eq!(
        registry.spec("sensor_p.output_msg").unwrap().group,
        "PRIMARY"
    );
    assert_eq!(
        registry.spec("sensor_a0.output_msg").unwrap().group,
        "ARRAY"
    );
}

/// The registry publishes no JSON of its own, so this is the seam an application
/// builds its manifest or status view on. It has to carry the name alongside the
/// target, in a stable order, or a dump built from it cannot be diffed.
#[test]
fn targets_enumerates_every_registration_in_name_order() {
    let registry = Registry::new();
    let reading_out = Output::new(reading(1.0));
    let config_out = Output::new(device_config());
    registry
        .register(
            "SENSOR",
            "sensor_0.output_msg",
            &reading_out,
            TargetKind::Output,
        )
        .unwrap();
    registry
        .register("DEVICE", "device_0.config", &config_out, CONFIG)
        .unwrap();

    let listed = registry.targets();
    let names: Vec<&str> = listed.iter().map(|(name, _)| name.as_str()).collect();
    assert_eq!(names, ["device_0.config", "sensor_0.output_msg"]);

    let groups: Vec<&str> = listed
        .iter()
        .map(|(_, target)| target.spec().unwrap().group)
        .collect();
    assert_eq!(groups, ["DEVICE", "SENSOR"]);
    assert_eq!(names, registry.target_names());
}

// ---------------------------------------------------------------------------
// replaceAt paths
// ---------------------------------------------------------------------------

/// A config with an array, so element paths have something to resolve against.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(default)]
struct VectorConfig {
    bias_xyz: [f64; 3],
    sample_hz: f64,
}

fn registry_with_vector() -> (Registry, Output<VectorConfig>) {
    let registry = Registry::new();
    let output = Output::new(VectorConfig {
        bias_xyz: [0.0, 0.0, 0.0],
        sample_hz: 200.0,
    });
    registry
        .register("DEVICE", "device_0.config", &output, CONFIG)
        .unwrap();
    (registry, output)
}

/// The schema names `bias_xyz`, not `bias_xyz.0`. An index has to resolve
/// through its parent, or every element command would be refused as an unknown
/// field before the pointer was ever tried.
#[test]
fn an_index_into_a_known_array_is_addressable_though_the_schema_lists_the_array() {
    let (registry, output) = registry_with_vector();

    registry
        .install(
            "device_0.config",
            Mode::ReplaceAt,
            json!([{ "op": "replace", "path": "/bias_xyz/2", "value": 0.5 }]),
        )
        .expect("an element of a registered array was refused");

    assert_eq!(output.read().bias_xyz, [0.0, 0.0, 0.5]);
}

#[test]
fn a_replace_at_naming_a_field_the_type_does_not_have_is_refused_with_a_suggestion() {
    let (registry, _output) = registry_with_vector();

    let error = registry
        .install(
            "device_0.config",
            Mode::ReplaceAt,
            json!([{ "op": "replace", "path": "/sample_hs", "value": 1.0 }]),
        )
        .expect_err("a misspelled field was accepted");

    let error = format!("{error:#}");
    assert!(error.contains("unknown field 'sample_hs'"), "{error}");
    assert!(error.contains("did you mean 'sample_hz'"), "{error}");
}

/// An index is only meaningful under an array. Allowing it anywhere would let
/// `sample_hz.0` through the path check and leave serde to explain it.
#[test]
fn an_index_under_a_field_that_is_not_an_array_is_refused() {
    let (registry, _output) = registry_with_vector();

    let error = registry
        .install(
            "device_0.config",
            Mode::ReplaceAt,
            json!([{ "op": "replace", "path": "/sample_hz/0", "value": 1.0 }]),
        )
        .expect_err("an index into a scalar was accepted");

    assert!(format!("{error:#}").contains("unknown field 'sample_hz.0'"));
}

#[test]
fn a_replace_at_document_that_is_not_a_list_of_operations_is_refused() {
    let (registry, _output) = registry_with_vector();

    let error = registry
        .install(
            "device_0.config",
            Mode::ReplaceAt,
            json!({ "bias_xyz": [1.0, 2.0, 3.0] }),
        )
        .expect_err("a merge document was accepted as a patch document");

    assert!(format!("{error:#}").contains("must be an array of operations"));
}

/// The schema and the payload check have to agree about arrays too: `payload_paths`
/// stops at an array, so a whole-array patch still validates as one field.
#[test]
fn a_whole_array_patch_still_validates_against_the_array_itself() {
    let (registry, output) = registry_with_vector();

    registry
        .install(
            "device_0.config",
            Mode::Patch,
            json!({ "bias_xyz": [1.0, 2.0, 3.0] }),
        )
        .expect("a whole-array patch was refused");

    assert_eq!(output.read().bias_xyz, [1.0, 2.0, 3.0]);
}

// ---------------------------------------------------------------------------
// Option fields: the schema is read off `T::default()`, so a field that is
// `None` there has no children to advertise
// ---------------------------------------------------------------------------

/// A `replace` carries a complete message, and for an `Option` field that means
/// the fields *inside* it whenever it is `Some`. Those paths are absent from a
/// schema derived from `T::default()`, where the option is `None` and therefore a
/// single null leaf — so a valid, typed, complete replacement was refused for
/// naming fields that "do not exist".
///
/// Uses this crate's own [`PlanetStateMsg`], whose `orientation` defaults to
/// `None`, so the shape is not hypothetical.
#[test]
fn a_replace_may_name_fields_the_type_default_leaves_unpopulated() {
    use crate::messages::{PlanetOrientation, PlanetStateMsg};
    use nalgebra::Vector3;

    let registry = Registry::new();
    let output = Output::new(PlanetStateMsg::default());
    registry
        .register("PLANET", "earth.output_msg", &output, TargetKind::Output)
        .unwrap();

    let complete = serde_json::to_value(PlanetStateMsg {
        position_inertial_m: Vector3::new(1.0, 2.0, 3.0),
        velocity_inertial_mps: Vector3::new(4.0, 5.0, 6.0),
        orientation: Some(PlanetOrientation::identity()),
    })
    .unwrap();

    registry
        .install("earth.output_msg", Mode::Replace, complete)
        .expect("a complete, typed replacement was refused");

    assert!(
        output.read().orientation.is_some(),
        "the replacement did not take"
    );
}

/// The same defect reached `patch`, and there it needed no unusual payload at
/// all: a module that has populated an option publishes fields the type default
/// does not mention, and changing one of them is an ordinary request.
#[test]
fn a_patch_may_name_a_field_inside_an_option_the_module_populated() {
    use crate::messages::{PlanetOrientation, PlanetStateMsg};
    use nalgebra::Matrix3;

    let registry = Registry::new();
    let output = Output::new(PlanetStateMsg {
        orientation: Some(PlanetOrientation::identity()),
        ..PlanetStateMsg::default()
    });
    registry
        .register("PLANET", "earth.output_msg", &output, TargetKind::Output)
        .unwrap();

    let spun = serde_json::to_value(Matrix3::from_diagonal_element(2.0)).unwrap();
    registry
        .install(
            "earth.output_msg",
            Mode::Patch,
            json!({ "orientation": { "inertial_to_fixed_dot": spun } }),
        )
        .expect("patching a populated option was refused");

    let orientation = output.read().orientation.expect("the option was cleared");
    assert_eq!(
        orientation.inertial_to_fixed_dot,
        Matrix3::from_diagonal_element(2.0)
    );
    // The sibling field inside the option was left alone, which is what makes
    // this a patch rather than a replace of the option.
    assert_eq!(orientation.inertial_to_fixed, Matrix3::identity());
}

/// A misspelling *inside* a populated option still has to be caught. The
/// candidate is what the payload produces, so a name serde dropped is absent
/// from it — the same comparison that accepts the two tests above rejects this.
#[test]
fn a_typo_inside_an_option_is_still_refused() {
    use crate::messages::{PlanetOrientation, PlanetStateMsg};

    let registry = Registry::new();
    let output = Output::new(PlanetStateMsg {
        orientation: Some(PlanetOrientation::identity()),
        ..PlanetStateMsg::default()
    });
    registry
        .register("PLANET", "earth.output_msg", &output, TargetKind::Output)
        .unwrap();

    let error = registry
        .install(
            "earth.output_msg",
            Mode::Patch,
            json!({ "orientation": { "inertial_to_fixed_dto": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] } }),
        )
        .unwrap_err()
        .to_string();

    assert!(
        error.contains("inertial_to_fixed_dto"),
        "unhelpful error: {error}"
    );
    assert!(!output.is_overridden(), "a refused patch was installed");
}

/// A real field carrying an unusable value must be reported as the value error it
/// is. The name check derives its field list from the applied candidate, so when
/// the apply fails there is no candidate to derive from — and answering "unknown
/// field" would deny a field that exists and send the operator looking for a
/// misspelling that is not there.
#[test]
fn a_valid_field_with_an_unusable_value_reports_the_value_not_the_name() {
    use crate::messages::{PlanetOrientation, PlanetStateMsg};

    let registry = Registry::new();
    let output = Output::new(PlanetStateMsg {
        orientation: Some(PlanetOrientation::identity()),
        ..PlanetStateMsg::default()
    });
    registry
        .register("PLANET", "earth.output_msg", &output, TargetKind::Output)
        .unwrap();

    let error = registry
        .install(
            "earth.output_msg",
            Mode::Patch,
            json!({ "orientation": { "inertial_to_fixed": "not a matrix" } }),
        )
        .unwrap_err()
        .to_string();

    assert!(
        !error.contains("unknown field"),
        "a real field was reported as unknown: {error}"
    );
    // The error has to say what was actually wrong with the value, not merely
    // decline to blame the name.
    assert!(
        error.contains("invalid type") || error.contains("expected"),
        "the value error says nothing about the value: {error}"
    );
    assert!(!output.is_overridden(), "a refused patch was installed");
}

/// The same value error, but with the option `None` in both the type default and
/// the port's current value — so neither runtime value can show that the field
/// being named exists. It still must not be reported as a misspelling.
#[test]
fn a_replace_filling_an_empty_option_badly_reports_the_value_not_the_name() {
    use crate::messages::{PlanetOrientation, PlanetStateMsg};

    let registry = Registry::new();
    // `orientation` is None here, and None in PlanetStateMsg::default().
    let output = Output::new(PlanetStateMsg::default());
    registry
        .register("PLANET", "earth.output_msg", &output, TargetKind::Output)
        .unwrap();

    let mut payload = serde_json::to_value(PlanetStateMsg {
        orientation: Some(PlanetOrientation::identity()),
        ..PlanetStateMsg::default()
    })
    .unwrap();
    payload["orientation"]["inertial_to_fixed"] = json!("not a matrix");

    let error = registry
        .install("earth.output_msg", Mode::Replace, payload)
        .unwrap_err()
        .to_string();

    assert!(
        !error.contains("unknown field"),
        "a real field was reported as unknown: {error}"
    );
    assert!(
        error.contains("invalid type") || error.contains("expected"),
        "the value error says nothing about the value: {error}"
    );
}

// ---------------------------------------------------------------------------
// mode names on the wire
// ---------------------------------------------------------------------------

/// The wire spelling is part of the contract with every client, so it is
/// asserted rather than left to `rename_all` to keep right.
#[test]
fn every_mode_serialises_to_the_name_a_client_sends() {
    for (mode, name) in [
        (Mode::Replace, "replace"),
        (Mode::Patch, "patch"),
        (Mode::Freeze, "freeze"),
        (Mode::Default, "default"),
        (Mode::ReplaceAt, "replaceAt"),
    ] {
        assert_eq!(serde_json::to_value(mode).unwrap(), json!(name));
        assert_eq!(
            serde_json::from_value::<Mode>(json!(name)).unwrap(),
            mode,
            "'{name}' did not round-trip"
        );
    }
}

/// This crate carries no deprecated mode spellings, so a name it does not
/// define is refused rather than quietly mapped onto one it does. An
/// application that wants to keep honouring a name it has retired does that in
/// its own parser, where the saved scenarios needing it actually live.
#[test]
fn a_mode_name_this_crate_does_not_define_is_refused() {
    for undefined in ["pointerReplace", "jsonPatch", "replaceat", "replace_at"] {
        assert!(
            serde_json::from_value::<Mode>(json!(undefined)).is_err(),
            "'{undefined}' deserialised into a mode this crate never named"
        );
    }
}
