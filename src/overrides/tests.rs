//! Fixtures here are deliberately generic. They exercise the registry, not any
//! particular instrument, so they name no real device, field or calibration
//! value — an application's hardware vocabulary has no business in this crate.

use serde::{Deserialize, Serialize};
use serde_json::json;

use crate::messages::{Input, Output};

use super::schema::leaf_fields;
use crate::overrides::{
    Case, Fault, FaultEventKind, FaultSchedule, Mode, Registry, Request, TargetKind,
};

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
        .register("sensor_0.output_msg", &output, TargetKind::Output)
        .unwrap();
    (registry, output)
}

fn registry_with_config() -> (Registry, Output<DeviceConfig>) {
    let registry = Registry::new();
    let config = Output::new(device_config());
    registry
        .register("sensor_0.config", &config, CONFIG)
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

/// What the schema calls a leaf, and that a payload can name every one of them.
/// An array is a leaf because listing an entry per element would have a client
/// render one control per component *and* another for the whole vector; an empty
/// object is a leaf because there is nothing under it to name; the root is not a
/// field.
#[test]
fn a_payload_can_name_every_leaf_the_schema_publishes() {
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
            "/empty",
            "/flat",
            "/list",
            "/nested/deeper/leaf",
            "/nested/inner"
        ]
    );
    // Every path the schema publishes is a payload key verbatim. That is the
    // property one path syntax exists for, and the only thing tying the two
    // sides together now that a payload names its pointers outright.
    let payload: serde_json::Map<String, serde_json::Value> = described
        .iter()
        .map(|path| (path.clone(), json!(0)))
        .collect();
    assert_eq!(
        Request::replace(serde_json::Value::Object(payload))
            .unwrap()
            .named_paths(),
        described,
        "a path the schema publishes is not usable as a payload key"
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
            "/angle_deg",
            "/quality_percent",
            "/rate_dps",
            "/status",
            "/value"
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
            Request::replace(json!({ "/angle_dge": 10.0 })).unwrap(),
        )
        .unwrap_err()
        .to_string();

    assert!(error.contains("angle_dge"), "unhelpful error: {error}");
    assert!(error.contains("did you mean '/angle_deg'"), "{error}");
    assert!(!output.is_overridden());
}

#[test]
fn accumulated_patches_survive_a_rejected_one() {
    let (registry, output) = registry_with_reading();
    registry
        .install(
            "sensor_0.output_msg",
            Request::replace(json!({ "/angle_deg": 10.0 })).unwrap(),
        )
        .unwrap();

    let _ = registry.install(
        "sensor_0.output_msg",
        Request::replace(json!({ "/rate_dps": "sideways" })).unwrap(),
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
            Request::replace(json!({ "/angle_deg": 10.0 })).unwrap(),
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
        .register("sensor_0.output_msg", &other, TargetKind::Output)
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
        .register("consumer_0.input_msg", &faulted, TargetKind::Input)
        .unwrap();

    registry
        .install(
            "consumer_0.input_msg",
            Request::replace(json!({ "/angle_deg": 10.0 })).unwrap(),
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
        .register("consumer_0.input_msg", &orphan, TargetKind::Input)
        .unwrap_err()
        .to_string();

    assert!(error.contains("consumer_0.input_msg"), "{error}");
    assert!(error.contains("connected"), "{error}");
}

// ---------------------------------------------------------------------------
// `replace` means all of it
// ---------------------------------------------------------------------------

/// A replace is relative, so a partial document changes what it names and
/// leaves the rest of the message alone.
///
/// This is the behaviour the old completeness guard existed to prevent, and it
/// is now the point. That guard protected a promise `replace` no longer makes:
/// with `patch` folded into it, a document naming one field is a legitimate way
/// to ask for one field to change. What the guard was really defending against —
/// `#[serde(default)]` silently resetting the fields a document omits — cannot
/// happen when the document is merged onto the upstream value rather than
/// deserialised on its own.
#[test]
fn a_partial_replace_changes_only_what_it_names() {
    let (registry, config) = registry_with_config();

    registry
        .install(
            "sensor_0.config",
            Request::replace(json!({ "/scale": 1.0 })).unwrap(),
        )
        .expect("a partial replace was refused");

    assert_eq!(config.read().scale, 1.0, "the named field did not change");
    assert_eq!(
        config.read().sample_hz,
        200.0,
        "an omitted field was reset to the type default"
    );
}

#[test]
fn a_replace_naming_every_field_is_accepted() {
    let (registry, config) = registry_with_config();

    registry
        .install(
            "sensor_0.config",
            Request::replace(
                json!({ "/bias": 1.0, "/scale": 2.0, "/sample_hz": 50.0, "/rng_seed": 3 }),
            )
            .unwrap(),
        )
        .expect("a complete replace was refused");

    assert_eq!(config.read().sample_hz, 50.0);
    assert_eq!(config.read().scale, 2.0);
}

/// A freeze and a default take a selection rather than a message, so an empty
/// payload names the whole message rather than naming nothing.
#[test]
fn a_freeze_and_a_default_accept_an_empty_payload() {
    let (registry, config) = registry_with_config();

    registry
        .install(
            "sensor_0.config",
            Request::replace(json!({ "/scale": 1.0 })).unwrap(),
        )
        .expect("a one-field patch was refused");
    assert_eq!(config.read().scale, 1.0);
    assert_eq!(
        config.read().sample_hz,
        200.0,
        "patch touched a field it did not name"
    );

    let request = Request::freeze(json!(null)).unwrap();
    let mode = request.mode();
    registry
        .install("sensor_0.config", request)
        .unwrap_or_else(|err| panic!("{mode:?} was refused as incomplete: {err}"));
}

/// Serde ignores a field it does not recognise, so a misspelling merges
/// cleanly, reports success and changes nothing. The check names it and offers
/// the field it was probably meant to be.
#[test]
fn a_typo_in_a_replace_is_reported_with_the_field_it_resembles() {
    let (registry, _config) = registry_with_config();

    let error = registry
        .install(
            "sensor_0.config",
            Request::replace(
                json!({ "/bias": 1.0, "/scael": 2.0, "/sample_hz": 50.0, "/rng_seed": 3 }),
            )
            .unwrap(),
        )
        .expect_err("a misspelled field was accepted")
        .to_string();

    assert!(error.contains("unknown field '/scael'"), "{error}");
    assert!(error.contains("did you mean '/scale'"), "{error}");
}

/// A rule naming the same field as the one beneath it wins while installed,
/// because it composes last — not because it displaced anything. Removing it
/// uncovers the earlier value, and each id still names its own layer.
#[test]
fn removing_a_rule_uncovers_the_value_the_one_beneath_it_set() {
    let (registry, output) = registry_with_reading();
    let patch = registry
        .install(
            "sensor_0.output_msg",
            Request::replace(json!({ "/angle_deg": 10.0 })).unwrap(),
        )
        .unwrap();
    let replace = registry
        .install(
            "sensor_0.output_msg",
            Request::replace(json!({ "": serde_json::to_value(reading(20.0)).unwrap() })).unwrap(),
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
            Request::replace(json!({ "/angle_deg": 10.0 })).unwrap(),
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
        .install("nope.output_msg", Request::replace(json!({})).unwrap())
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
        .install("sensor_0.config", Request::replace(json!({})).unwrap())
        .unwrap_err()
        .to_string();

    assert!(
        error.contains("registered no override targets"),
        "unhelpful error: {error}"
    );
    assert!(error.contains("sensor_0.config"), "{error}");
}

/// The same module type is often flown twice — one unit in one role, an array of
/// them in another — so two registrations of one type have to stay independent.
/// A fault on either must not reach the other.
#[test]
fn one_module_type_can_be_registered_under_two_names() {
    let registry = Registry::new();
    let primary = Output::new(reading(1.0));
    let redundant = Output::new(reading(2.0));
    registry
        .register("sensor_p.output_msg", &primary, TargetKind::Output)
        .unwrap();
    registry
        .register("sensor_a0.output_msg", &redundant, TargetKind::Output)
        .unwrap();

    registry
        .install(
            "sensor_p.output_msg",
            Request::replace(json!({ "/angle_deg": 9.0 })).unwrap(),
        )
        .unwrap();

    assert_eq!(primary.read().angle_deg, 9.0);
    assert_eq!(redundant.read().angle_deg, 2.0, "the sibling is untouched");
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
        .register("sensor_0.output_msg", &reading_out, TargetKind::Output)
        .unwrap();
    registry
        .register("device_0.config", &config_out, CONFIG)
        .unwrap();

    let listed = registry.targets();
    let names: Vec<&str> = listed.iter().map(|(name, _)| name.as_str()).collect();
    assert_eq!(names, ["device_0.config", "sensor_0.output_msg"]);

    // Each spec travels with its own target rather than with its position, so the
    // reordering the map does cannot pair a name with someone else's schema.
    let kinds: Vec<&str> = listed
        .iter()
        .map(|(_, target)| target.spec().unwrap().kind.as_str())
        .collect();
    assert_eq!(kinds, ["config", "output"]);
    assert_eq!(names, registry.target_names());
}

// ---------------------------------------------------------------------------
// operation paths
// ---------------------------------------------------------------------------

/// A config with an array, so element paths have something to resolve against.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(default)]
struct VectorConfig {
    bias_xyz: [f64; 3],
    sample_hz: f64,
}

/// A nested object whose fields all have defaults, so a dropped one
/// deserialises cleanly instead of failing — which is what makes a typo inside
/// a container silent rather than self-reporting.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(default)]
struct NestedGain {
    gain: f64,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(default)]
struct NestedConfig {
    sample_hz: f64,
    nested: NestedGain,
}

fn registry_with_nested() -> (Registry, Output<NestedConfig>) {
    let registry = Registry::new();
    let output = Output::new(NestedConfig {
        sample_hz: 200.0,
        nested: NestedGain { gain: 1.5 },
    });
    registry
        .register("device_0.config", &output, CONFIG)
        .unwrap();
    (registry, output)
}

/// An array of objects, which is where a container assignment reaches furthest:
/// one payload supplies every field of every element.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(default)]
struct Stage {
    gain: f64,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(default)]
struct StageConfig {
    stages: Vec<Stage>,
}

fn registry_with_stages() -> (Registry, Output<StageConfig>) {
    let registry = Registry::new();
    let output = Output::new(StageConfig {
        stages: vec![Stage { gain: 1.5 }],
    });
    registry
        .register("device_0.config", &output, CONFIG)
        .unwrap();
    (registry, output)
}

/// A message that is not a struct with fields. `SimulationMessage` is a blanket
/// impl over `Clone + Default + Serialize + DeserializeOwned`, so this is one,
/// and the root is the only path it has.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
struct Level(f64);

fn registry_with_vector() -> (Registry, Output<VectorConfig>) {
    let registry = Registry::new();
    let output = Output::new(VectorConfig {
        bias_xyz: [0.0, 0.0, 0.0],
        sample_hz: 200.0,
    });
    registry
        .register("device_0.config", &output, CONFIG)
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
            Request::replace(json!({ "/bias_xyz/2": 0.5 })).unwrap(),
        )
        .expect("an element of a registered array was refused");

    assert_eq!(output.read().bias_xyz, [0.0, 0.0, 0.5]);
}

/// `spin_axes_body` is one 3-vector per wheel, so wheel 0's y-axis sits two
/// segments below the array the schema advertises.
#[test]
fn an_index_inside_an_array_element_is_addressable_too() {
    use crate::messages::RwArrayConfigMsg;

    let registry = Registry::new();
    let output = Output::new(RwArrayConfigMsg::default());
    registry.register("rw.config", &output, CONFIG).unwrap();

    registry
        .install(
            "rw.config",
            Request::replace(json!({ "/spin_axes_body/0/1": 7.0 })).unwrap(),
        )
        .expect("a component of an array element was refused");

    assert_eq!(output.read().spin_axes_body[0][1], 7.0);

    // A name below an array is checked like any other, and no component of a
    // 3-vector is one edit from `why`, so nothing is suggested. 108 element
    // paths are in reach here — a looser measure offered `/spin_axes_body/10/0`.
    let error = registry
        .install(
            "rw.config",
            Request::replace(json!({ "/spin_axes_body/0/why": 7.0 })).unwrap(),
        )
        .expect_err("a path into nothing was installed")
        .to_string();
    assert!(
        error.contains("unknown field '/spin_axes_body/0/why'"),
        "{error}"
    );
    assert!(!error.contains("did you mean"), "{error}");
}

#[test]
fn an_operation_naming_a_field_the_type_does_not_have_is_refused_with_a_suggestion() {
    let (registry, _output) = registry_with_vector();

    let error = registry
        .install(
            "device_0.config",
            Request::replace(json!({ "/sample_hs": 1.0 })).unwrap(),
        )
        .expect_err("a misspelled field was accepted");

    let error = format!("{error:#}");
    assert!(error.contains("unknown field '/sample_hs'"), "{error}");
    assert!(error.contains("did you mean '/sample_hz'"), "{error}");
}

/// An index is only meaningful under an array. Allowing it anywhere would let
/// `sample_hz.0` through the path check and leave serde to explain it.
#[test]
fn an_index_under_a_field_that_is_not_an_array_is_refused() {
    let (registry, _output) = registry_with_vector();

    let error = registry
        .install(
            "device_0.config",
            Request::replace(json!({ "/sample_hz/0": 1.0 })).unwrap(),
        )
        .expect_err("an index into a scalar was accepted");

    assert!(format!("{error:#}").contains("unknown field '/sample_hz/0'"));
}

/// A whole array and one of its elements are the same kind of request, which
/// is what one payload shape buys: no choosing a format before writing a path.
#[test]
fn a_replace_addresses_a_whole_array_or_one_element_alike() {
    let (registry, output) = registry_with_vector();

    registry
        .install(
            "device_0.config",
            Request::replace(json!({ "/bias_xyz": [1.0, 2.0, 3.0] })).unwrap(),
        )
        .expect("a whole-array assignment was refused");
    assert_eq!(output.read().bias_xyz, [1.0, 2.0, 3.0]);

    registry
        .install(
            "device_0.config",
            Request::replace(json!({ "/bias_xyz/1": 9.0 })).unwrap(),
        )
        .expect("an element assignment was refused");
    assert_eq!(output.read().bias_xyz, [1.0, 9.0, 3.0]);
}

/// The schema stops at an array rather than listing an entry per element, so a
/// whole-array assignment names one field. An element is still addressable —
/// `indexes_a_known_array` checks the index against its parent.
#[test]
fn a_whole_array_patch_still_validates_against_the_array_itself() {
    let (registry, output) = registry_with_vector();

    registry
        .install(
            "device_0.config",
            Request::replace(json!({ "/bias_xyz": [1.0, 2.0, 3.0] })).unwrap(),
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
        .register("earth.output_msg", &output, TargetKind::Output)
        .unwrap();

    let complete = serde_json::to_value(PlanetStateMsg {
        position_inertial_m: Vector3::new(1.0, 2.0, 3.0),
        velocity_inertial_mps: Vector3::new(4.0, 5.0, 6.0),
        orientation: Some(PlanetOrientation::identity()),
    })
    .unwrap();

    registry
        .install(
            "earth.output_msg",
            Request::replace(json!({ "": complete })).unwrap(),
        )
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
        .register("earth.output_msg", &output, TargetKind::Output)
        .unwrap();

    let spun = serde_json::to_value(Matrix3::from_diagonal_element(2.0)).unwrap();
    registry
        .install(
            "earth.output_msg",
            Request::replace(json!({ "/orientation/inertial_to_fixed_dot": spun })).unwrap(),
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
        .register("earth.output_msg", &output, TargetKind::Output)
        .unwrap();

    let error = registry
        .install(
            "earth.output_msg",
            Request::replace(json!({ "/orientation/inertial_to_fixed_dto": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] })).unwrap(),
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
        .register("earth.output_msg", &output, TargetKind::Output)
        .unwrap();

    let error = registry
        .install(
            "earth.output_msg",
            Request::replace(json!({ "/orientation/inertial_to_fixed": "not a matrix" })).unwrap(),
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
        .register("earth.output_msg", &output, TargetKind::Output)
        .unwrap();

    let mut payload = serde_json::to_value(PlanetStateMsg {
        orientation: Some(PlanetOrientation::identity()),
        ..PlanetStateMsg::default()
    })
    .unwrap();
    payload["orientation"]["inertial_to_fixed"] = json!("not a matrix");

    let error = registry
        .install(
            "earth.output_msg",
            Request::replace(json!({ "": payload })).unwrap(),
        )
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
    for (mode, name) in [(Mode::Replace, "replace"), (Mode::Freeze, "freeze")] {
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
    for undefined in [
        "patch",
        "replaceAt",
        "pointerReplace",
        "jsonPatch",
        "replaceat",
        "replace_at",
    ] {
        assert!(
            serde_json::from_value::<Mode>(json!(undefined)).is_err(),
            "'{undefined}' deserialised into a mode this crate never named"
        );
    }
}

/// A field whose name contains a dot is addressed by the pointer `/a.b`, and a
/// dot is not a separator, so `/a.b` and `/a/b` are different places. The name
/// check tells them apart on its own.
#[test]
fn a_dotted_key_is_refused_rather_than_matching_the_path_it_resembles() {
    use crate::messages::{PlanetOrientation, PlanetStateMsg};

    let registry = Registry::new();
    let output = Output::new(PlanetStateMsg {
        orientation: Some(PlanetOrientation::identity()),
        ..PlanetStateMsg::default()
    });
    registry
        .register("earth.output_msg", &output, TargetKind::Output)
        .unwrap();

    let error = registry
        .install(
            "earth.output_msg",
            Request::replace(json!({ "/orientation.inertial_to_fixed": 1.0 })).unwrap(),
        )
        .expect_err("a dotted key was accepted as the path it resembles");

    // No bespoke guard is needed for this any more. Pointer paths spell the two
    // readings differently — `/orientation.inertial_to_fixed` for a member whose
    // name contains a dot, `/orientation/inertial_to_fixed` for the nested
    // field — so the ordinary unknown-field check separates them, and its "did
    // you mean" happens to be exactly the correction the sender needs.
    let error = error.to_string();
    assert!(
        error.contains("unknown field '/orientation.inertial_to_fixed'"),
        "the error did not name the key as written: {error}"
    );
    assert!(
        error.contains("did you mean '/orientation/inertial_to_fixed'"),
        "the error did not offer the path the sender meant: {error}"
    );
    assert!(
        output.installed_overrides().is_empty(),
        "a rule that can never apply was installed anyway"
    );
}

/// A whole-object assignment supplies each field inside it, and serde drops one
/// it does not recognise. Checking only the assignment's own pointer let
/// `{"/nested": {"gaim": 9}}` install, report success, and leave `gain` at its
/// default — the exact failure the name check exists to prevent, one level in.
#[test]
fn a_typo_inside_a_container_assignment_is_refused() {
    let (registry, output) = registry_with_nested();

    let error = registry
        .install(
            "device_0.config",
            Request::replace(json!({ "/nested": { "gaim": 9.0 } })).unwrap(),
        )
        .expect_err("a typo inside a container was installed")
        .to_string();

    assert!(error.contains("'/nested/gaim'"), "{error}");
    assert!(error.contains("did you mean '/nested/gain'"), "{error}");
    assert_eq!(
        output.read().nested.gain,
        1.5,
        "the value was changed anyway"
    );
    assert!(!output.is_overridden());
}

/// The root is a container like any other, and the widest one — a typo there
/// defaults every field the message has.
#[test]
fn a_typo_inside_a_root_assignment_is_refused() {
    let (registry, output) = registry_with_nested();

    let error = registry
        .install(
            "device_0.config",
            Request::replace(json!({ "": { "nestde": { "gain": 9.0 } } })).unwrap(),
        )
        .expect_err("a typo in a whole-message replacement was installed")
        .to_string();

    assert!(error.contains("'/nestde/gain'"), "{error}");
    // Two letters swapped, which is a typo like any other and still one edit.
    assert!(error.contains("did you mean '/nested/gain'"), "{error}");
    assert_eq!(
        output.read().nested.gain,
        1.5,
        "the value was changed anyway"
    );
}

/// The root is the one pointer with no name in it to misspell, but the name
/// check judged it like a field and found the type did not have it. A scalar
/// message could be frozen and defaulted whole and not replaced, so the three
/// modes disagreed about whether such a target held anything at all.
#[test]
fn a_message_that_is_a_scalar_is_replaceable_at_its_root() {
    let registry = Registry::new();
    let output = Output::new(Level(1.0));
    registry
        .register("device_0.level", &output, CONFIG)
        .unwrap();

    registry
        .install(
            "device_0.level",
            Request::replace(json!({ "": 9.0 })).unwrap(),
        )
        .expect("the only path a scalar message has was refused");

    assert_eq!(output.read(), Level(9.0));

    // Exempt from the name check, not from the apply. The root reported "unknown
    // field ''" for a value of the wrong type too, naming the pointer when the
    // value was what was wrong.
    let error = registry
        .install(
            "device_0.level",
            Request::replace(json!({ "": "nope" })).unwrap(),
        )
        .expect_err("a string was installed into an f64")
        .to_string();
    assert!(error.contains("invalid type"), "{error}");
    assert!(!error.contains("unknown field"), "{error}");
}

/// An element is a container too, and every way of reaching one hid the same
/// typo: the walk that finds supplied fields stopped at arrays, and what did get
/// named below an array was waved through unchecked. All three spellings supply
/// `/stages/0/gaim`, and each left `gain` at its default while reporting success.
#[test]
fn a_typo_inside_an_array_element_is_refused_however_it_is_reached() {
    for payload in [
        json!({ "/stages": [{ "gaim": 9.0 }] }),
        json!({ "/stages/0": { "gaim": 9.0 } }),
        json!({ "": { "stages": [{ "gaim": 9.0 }] } }),
    ] {
        let (registry, output) = registry_with_stages();

        let error = registry
            .install(
                "device_0.config",
                Request::replace(payload.clone()).unwrap(),
            )
            .expect_err("a typo inside an array element was installed")
            .to_string();

        assert!(error.contains("'/stages/0/gaim'"), "{payload}: {error}");
        assert!(
            error.contains("did you mean '/stages/0/gain'"),
            "{payload}: {error}"
        );
        assert_eq!(
            output.read().stages[0].gain,
            1.5,
            "the value was changed anyway: {payload}"
        );
    }
}

/// Checking elements must not cost the array its own paths: one that grows,
/// empties, or is written whole is still the operator's to command.
#[test]
fn an_array_may_still_be_grown_emptied_and_written_whole() {
    for (payload, expected) in [
        (
            json!({ "/stages": [{ "gain": 1.0 }, { "gain": 2.0 }] }),
            vec![1.0, 2.0],
        ),
        (json!({ "/stages": [] }), vec![]),
        (json!({ "/stages/0/gain": 4.0 }), vec![4.0]),
    ] {
        let (registry, output) = registry_with_stages();

        registry
            .install(
                "device_0.config",
                Request::replace(payload.clone()).unwrap(),
            )
            .unwrap_or_else(|error| panic!("{payload} was refused: {error:#}"));

        let gains: Vec<f64> = output.read().stages.iter().map(|s| s.gain).collect();
        assert_eq!(gains, expected, "{payload}");
    }
}

/// Past the end of an array there is no element to check a name against, and an
/// index there may be one the payload means to grow into. The apply answers,
/// naming the pointer, rather than the name check guessing "unknown field".
#[test]
fn an_index_past_the_end_of_an_array_is_left_to_the_apply() {
    let (registry, _output) = registry_with_stages();

    let error = registry
        .install(
            "device_0.config",
            Request::replace(json!({ "/stages/7/gain": 1.0 })).unwrap(),
        )
        .expect_err("an index past the end was installed")
        .to_string();

    assert!(error.contains("/stages/7/gain"), "{error}");
    assert!(!error.contains("unknown field"), "{error}");
}

/// The wider walk is the name check's, not the schema's. A client renders one
/// control per advertised field, so listing `/stages` *and* every element inside
/// it would draw the same value twice.
#[test]
fn checking_elements_does_not_add_them_to_the_published_schema() {
    let (registry, _output) = registry_with_stages();

    let paths: Vec<String> = registry
        .spec("device_0.config")
        .unwrap()
        .fields
        .into_iter()
        .map(|field| field.path)
        .collect();

    assert_eq!(paths, ["/stages"]);
}

/// A location may be a whole object, but the schema lists only leaves, so a
/// container path was reported as an unknown field though its pointer resolved.
#[test]
fn an_operation_may_address_a_whole_object_valued_field() {
    use crate::messages::{PlanetOrientation, PlanetStateMsg};
    use nalgebra::Matrix3;

    let registry = Registry::new();
    let output = Output::new(PlanetStateMsg {
        orientation: Some(PlanetOrientation::identity()),
        ..PlanetStateMsg::default()
    });
    registry
        .register("earth.output_msg", &output, TargetKind::Output)
        .unwrap();

    let mut replacement = PlanetOrientation::identity();
    replacement.inertial_to_fixed = Matrix3::from_diagonal_element(3.0);
    let replacement = serde_json::to_value(replacement).unwrap();

    registry
        .install(
            "earth.output_msg",
            Request::replace(json!({ "/orientation": replacement })).unwrap(),
        )
        .expect("replacing a whole object-valued field was refused");

    let orientation = output.read().orientation.expect("the option was cleared");
    assert_eq!(
        orientation.inertial_to_fixed,
        Matrix3::from_diagonal_element(3.0)
    );

    // Accepting containers must not accept a misspelled one: `orientatio` has no
    // leaf beneath it, so it is still a name error.
    let error = registry
        .install(
            "earth.output_msg",
            Request::replace(json!({ "/orientatio": {} })).unwrap(),
        )
        .expect_err("a misspelled container path was accepted");
    assert!(
        error.to_string().contains("unknown field '/orientatio'"),
        "{error}"
    );
}

/// The registry keeps a clone. With the connection held directly, `connect`
/// replaced only the original's, so the registry reported — and froze — a value
/// from the producer the input had been disconnected from.
#[test]
fn a_registered_input_follows_a_later_reconnect() {
    let producer_one = Output::new(ReadingMsg {
        value: 1.0,
        ..ReadingMsg::default()
    });
    let producer_two = Output::new(ReadingMsg {
        value: 2.0,
        ..ReadingMsg::default()
    });

    let mut input: Input<ReadingMsg> = Input::default();
    producer_one.connect_to(&mut input);

    let registry = Registry::new();
    registry
        .register("reading.input_msg", &input, TargetKind::Input)
        .unwrap();

    producer_two.connect_to(&mut input);

    assert_eq!(input.read().value, 2.0, "the input itself did not rewire");
    assert_eq!(
        registry.upstream("reading.input_msg").unwrap()["value"]
            .as_f64()
            .unwrap(),
        2.0,
        "the registry still reads the producer the input was disconnected from"
    );

    // A freeze captures what the target reads now. Against a stale connection it
    // pinned the consumer to a value no current producer ever published.
    registry
        .install("reading.input_msg", Request::freeze(json!({})).unwrap())
        .unwrap();
    assert_eq!(
        input.read().value,
        2.0,
        "the freeze pinned a value from the disconnected producer"
    );
}

/// Cloning a port aliases it, so a collection has to be built one at a time.
/// `vec![Input::default(); n]` shared one rule stack even before it shared a
/// connection, so a fault on one sensor applied to every sibling.
#[test]
fn independently_built_inputs_do_not_share_a_connection_or_a_rule_stack() {
    let producers: Vec<Output<ReadingMsg>> = (0..3)
        .map(|index| {
            Output::new(ReadingMsg {
                value: index as f64,
                ..ReadingMsg::default()
            })
        })
        .collect();

    let mut inputs: Vec<Input<ReadingMsg>> = (0..3).map(|_| Input::default()).collect();
    for (input, producer) in inputs.iter_mut().zip(&producers) {
        producer.connect_to(input);
    }

    // Each input reads its own producer.
    for (index, input) in inputs.iter().enumerate() {
        assert_eq!(
            input.read().value,
            index as f64,
            "input {index} is wired to the wrong producer"
        );
    }

    // A fault on one is invisible to the others.
    let registry = Registry::new();
    registry
        .register("reading.0", &inputs[0], TargetKind::Input)
        .unwrap();
    registry
        .install(
            "reading.0",
            Request::replace(json!({ "/value": 99.0 })).unwrap(),
        )
        .unwrap();

    assert_eq!(inputs[0].read().value, 99.0, "the fault did not apply");
    assert_eq!(inputs[1].read().value, 1.0, "the fault leaked to a sibling");
    assert_eq!(inputs[2].read().value, 2.0, "the fault leaked to a sibling");
}

/// A field renamed to carry a dot is addressable, and is not confused with the
/// nested path that used to flatten to the same string.
#[test]
fn a_field_whose_name_contains_the_separator_is_still_addressable() {
    #[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
    #[serde(default)]
    struct RenamedConfig {
        #[serde(rename = "gain.value")]
        gain_value: f64,
        plain: f64,
    }

    let registry = Registry::new();
    let output = Output::new(RenamedConfig::default());
    registry.register("device.config", &output, CONFIG).unwrap();

    // The schema advertises it in the only form serde accepts.
    let spec = registry.spec("device.config").unwrap();
    assert!(
        spec.fields.iter().any(|field| field.path == "/gain.value"),
        "the renamed field was not advertised: {:?}",
        spec.fields
    );

    registry
        .install(
            "device.config",
            Request::replace(json!({ "/gain.value": 5.0 })).unwrap(),
        )
        .expect("the renamed field was refused as a path");
    assert_eq!(output.read().gain_value, 5.0);
    assert_eq!(output.read().plain, 0.0, "the sibling was disturbed");

    // A dot the type cannot account for is still refused — now by the ordinary
    // name check, because `/plain.value` is a path the schema does not list and
    // is spelled differently from the nested `/plain/value` it resembles.
    let error = registry
        .install(
            "device.config",
            Request::replace(json!({ "/plain.value": 1.0 })).unwrap(),
        )
        .expect_err("an unaccounted dotted key was accepted");
    assert!(
        error.to_string().contains("unknown field '/plain.value'"),
        "{error}"
    );
}

// ---------------------------------------------------------------------------
// Scheduling: when a rule goes in, and when it comes back out
// ---------------------------------------------------------------------------

const SECOND: u64 = 1_000_000_000;

/// The rate this fixture's device publishes, so a test can say what a withdrawn
/// fault uncovers without repeating the number.
const PUBLISHED_RATE_DPS: f64 = -1.5;

fn rate_fault(rate_dps: f64) -> Request {
    Request::replace(json!({ "/rate_dps": rate_dps })).unwrap()
}

/// Due is "at or past", not "exactly at". A schedule advanced in steps a fault
/// falls between must still install it: the alternative is a campaign that
/// quietly injects nothing because the run loop's step did not divide its dates.
#[test]
fn a_fault_waits_for_the_clock_and_lands_on_the_first_advance_past_its_time() {
    let (registry, output) = registry_with_reading();
    let case = Case::new("late").with(
        Fault::new("sensor_0.output_msg", rate_fault(9.0))
            .at(5 * SECOND)
            .labelled("rate runs away"),
    );
    let mut faults = FaultSchedule::new(&registry, case);

    assert!(faults.advance(4 * SECOND).is_empty());
    assert_eq!(output.read().rate_dps, PUBLISHED_RATE_DPS);
    assert_eq!(faults.pending(), 1);

    let events = faults.advance(7 * SECOND);
    assert_eq!(events.len(), 1);
    assert!(matches!(events[0].kind, FaultEventKind::Installed(_)));
    assert_eq!(output.read().rate_dps, 9.0);
    assert_eq!(faults.pending(), 0);
    assert_eq!(faults.in_force(), 1);
}

/// Two faults on one port, one of which expires. The expiry names the rule its
/// own install returned, so the fault installed beside it is still in force
/// afterwards — which is the whole reason a schedule tracks ids rather than
/// clearing the target.
#[test]
fn a_lifetime_withdraws_its_own_rule_and_leaves_the_one_installed_beside_it() {
    let (registry, output) = registry_with_reading();
    let case = Case::new("dropout")
        .with(
            Fault::new("sensor_0.output_msg", rate_fault(0.0))
                .at(SECOND)
                .lasting(2 * SECOND)
                .labelled("rate dropout"),
        )
        .with(
            Fault::new(
                "sensor_0.output_msg",
                Request::replace(json!({ "/status": 3 })).unwrap(),
            )
            .at(SECOND)
            .labelled("stuck status"),
        );
    let mut faults = FaultSchedule::new(&registry, case);

    let installs = faults.advance(SECOND);
    // Two faults dated the same moment go in the order the case wrote them.
    assert_eq!(installs[0].label, "rate dropout");
    assert_eq!(installs[1].label, "stuck status");
    assert_eq!(output.read().rate_dps, 0.0);
    assert_eq!(output.read().status, 3);
    assert_eq!(faults.in_force(), 2);

    let withdrawals = faults.advance(3 * SECOND);
    assert_eq!(withdrawals.len(), 1);
    assert!(matches!(withdrawals[0].kind, FaultEventKind::Withdrawn(_)));
    assert_eq!(
        output.read().rate_dps,
        PUBLISHED_RATE_DPS,
        "the dropout did not lift when its lifetime ran out"
    );
    assert_eq!(
        output.read().status,
        3,
        "the fault installed beside the dropout was withdrawn with it"
    );
    assert_eq!(faults.in_force(), 1);
}

/// A lifetime that runs past the end of the clock. Added to the install time it
/// overflows, and both ways that lands are wrong in the same direction: a debug
/// build panics, and a release build wraps to an expiry in the past and
/// withdraws the fault on the advance that installed it — silently, and the
/// opposite of what the longest lifetime expressible could have asked for.
#[test]
fn a_lifetime_longer_than_the_clock_holds_for_the_rest_of_the_run() {
    let (registry, output) = registry_with_reading();
    let case = Case::new("held").with(
        Fault::new("sensor_0.output_msg", rate_fault(0.0))
            .at(SECOND)
            .lasting(u64::MAX)
            .labelled("held for the rest of the run"),
    );
    let mut faults = FaultSchedule::new(&registry, case);

    let installs = faults.advance(SECOND);
    assert_eq!(installs.len(), 1, "the fault was installed and withdrawn");
    assert_eq!(output.read().rate_dps, 0.0);
    assert_eq!(faults.in_force(), 1);

    assert!(faults.advance(u64::MAX / 2).is_empty());
    assert_eq!(
        output.read().rate_dps,
        0.0,
        "a lifetime past the end of the clock expired inside it"
    );
    assert_eq!(faults.in_force(), 1);
}

/// A fault nobody dated in advance. Whoever notices the condition posts it, and
/// need not be the thread advancing the schedule — which is what makes a
/// condition-triggered fault as ordinary as a clocked one.
#[test]
fn a_fault_posted_while_the_run_is_going_is_installed_on_the_next_advance() {
    let (registry, output) = registry_with_reading();
    let mut faults = FaultSchedule::new(&registry, Case::new("live"));
    let sender = faults.sender();

    std::thread::spawn(move || {
        sender
            .send(Fault::new("sensor_0.output_msg", rate_fault(4.0)).labelled("operator"))
            .expect("the schedule is still running");
    })
    .join()
    .expect("the posting thread finished");

    let events = faults.advance(SECOND);
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].label, "operator");
    assert_eq!(
        events[0].at_nanos, SECOND,
        "an event carries when it landed"
    );
    assert_eq!(output.read().rate_dps, 4.0);
}

/// A refused fault is recorded and the campaign carries on. One unusable entry
/// in a list written by hand must not cancel the faults after it, and it must
/// not pass unremarked either: `is_refusal` is how a run says which of the two
/// happened.
#[test]
fn a_refused_fault_is_recorded_rather_than_taking_the_ones_after_it_down() {
    let (registry, output) = registry_with_reading();
    let case = Case::new("typo")
        .with(
            Fault::new(
                "sensor_0.output_msg",
                Request::replace(json!({ "/rate_dsp": 9.0 })).unwrap(),
            )
            .labelled("misspelled"),
        )
        .with(Fault::new("sensor_0.output_msg", rate_fault(2.0)).labelled("sound"));
    let mut faults = FaultSchedule::new(&registry, case);

    let events = faults.advance(0);
    assert_eq!(events.len(), 2);
    assert!(events[0].is_refusal());
    let FaultEventKind::Refused(reason) = &events[0].kind else {
        panic!("a misspelled field was installed: {:?}", events[0].kind);
    };
    assert!(reason.contains("did you mean '/rate_dps'"), "{reason}");

    assert!(!events[1].is_refusal());
    assert_eq!(output.read().rate_dps, 2.0);
    assert_eq!(faults.in_force(), 1, "a refusal was counted as installed");
}

/// The same check, moved to before the run. A mistyped target otherwise surfaces
/// as a refusal minutes in, on a run that then reports no effect — which reads
/// exactly like a fault that had none.
#[test]
fn preflight_reports_a_fault_that_cannot_be_installed_before_the_run_starts() {
    let (registry, _output) = registry_with_reading();
    let case = Case::new("mistyped_target")
        .with(Fault::new("sensor_1.output_msg", rate_fault(1.0)).labelled("the wrong device"));
    let faults = FaultSchedule::new(&registry, case);

    let error = faults
        .preflight()
        .expect_err("a fault naming no registered target passed preflight");
    let reported = format!("{error:#}");
    assert!(reported.contains("the wrong device"), "{reported}");
    assert!(
        reported.contains("unknown override target 'sensor_1.output_msg'"),
        "{reported}"
    );
    // Checking is not installing.
    assert_eq!(faults.pending(), 1);
}

/// Ending a campaign ends the campaign's faults. A simulation can carry rules
/// this schedule never installed — an operator's, another campaign's — and
/// clearing the target would take those too.
#[test]
fn withdrawing_a_campaign_leaves_a_rule_installed_by_someone_else_alone() {
    let (registry, output) = registry_with_reading();
    registry
        .install(
            "sensor_0.output_msg",
            Request::replace(json!({ "/status": 7 })).unwrap(),
        )
        .expect("an installed rule from outside the campaign");

    let mut faults = FaultSchedule::new(
        &registry,
        Case::new("campaign").with(Fault::new("sensor_0.output_msg", rate_fault(0.0))),
    );
    faults.advance(0);
    assert_eq!(output.read().rate_dps, 0.0);

    faults.withdraw_all();
    assert_eq!(
        output.read().rate_dps,
        PUBLISHED_RATE_DPS,
        "the campaign's own rule survived its withdrawal"
    );
    assert_eq!(
        output.read().status,
        7,
        "a rule this campaign never installed was withdrawn with it"
    );
    assert_eq!(faults.in_force(), 0);
}

/// Lifting a campaign's faults is not the same as calling the campaign off. What
/// is in force comes out; what has not landed yet still lands, because a
/// schedule that forgot its remaining faults could not be told to lift one
/// without being abandoned.
#[test]
fn withdrawing_what_is_in_force_leaves_the_faults_still_to_come() {
    let (registry, output) = registry_with_reading();
    let case = Case::new("two_stage")
        .with(
            Fault::new("sensor_0.output_msg", rate_fault(0.0))
                .at(SECOND)
                .labelled("first"),
        )
        .with(
            Fault::new("sensor_0.output_msg", rate_fault(9.0))
                .at(3 * SECOND)
                .labelled("second"),
        );
    let mut faults = FaultSchedule::new(&registry, case);

    faults.advance(SECOND);
    assert_eq!(output.read().rate_dps, 0.0);

    faults.withdraw_in_force();
    assert_eq!(output.read().rate_dps, PUBLISHED_RATE_DPS);
    assert_eq!(faults.in_force(), 0);
    assert_eq!(faults.pending(), 1, "the fault still to come was dropped");

    faults.advance(3 * SECOND);
    assert_eq!(
        output.read().rate_dps,
        9.0,
        "the campaign stopped at the withdrawal instead of carrying on"
    );
}

/// Calling it off. Nothing waiting ever lands, and every fault dropped is in the
/// report — a run they were cancelled out of reads exactly like a run whose
/// faults had no effect.
#[test]
fn cancelling_stops_a_fault_from_ever_landing_and_records_that_it_did() {
    let (registry, output) = registry_with_reading();
    let case = Case::new("called_off")
        .with(
            Fault::new("sensor_0.output_msg", rate_fault(0.0))
                .at(SECOND)
                .labelled("in force"),
        )
        .with(
            Fault::new("sensor_0.output_msg", rate_fault(9.0))
                .at(3 * SECOND)
                .labelled("called off"),
        );
    let mut faults = FaultSchedule::new(&registry, case);
    faults.advance(SECOND);

    faults.withdraw_all();
    assert_eq!(faults.in_force(), 0);
    assert_eq!(faults.pending(), 0);

    faults.advance(10 * SECOND);
    assert_eq!(
        output.read().rate_dps,
        PUBLISHED_RATE_DPS,
        "a cancelled fault landed anyway"
    );

    let cancelled: Vec<(&str, u64)> = faults
        .events()
        .iter()
        .filter_map(|event| match event.kind {
            FaultEventKind::Cancelled { due_at_nanos } => {
                Some((event.label.as_str(), due_at_nanos))
            }
            _ => None,
        })
        .collect();
    assert_eq!(
        cancelled,
        [("called off", 3 * SECOND)],
        "a fault that never landed went unreported"
    );
}

/// A cancellation reaches what was posted before it, including what is still in
/// the channel. It cannot reach what is posted after: a sender that has been
/// handed out cannot be recalled, and this is the one part of ending a campaign
/// that dropping the senders does rather than the schedule.
#[test]
fn cancelling_drops_what_was_posted_before_it_and_not_what_follows() {
    let (registry, output) = registry_with_reading();
    let mut faults = FaultSchedule::new(&registry, Case::new("live"));
    let sender = faults.sender();

    sender
        .send(Fault::new("sensor_0.output_msg", rate_fault(0.0)).labelled("posted first"))
        .unwrap();
    faults.cancel_pending();
    faults.advance(SECOND);
    assert_eq!(
        output.read().rate_dps,
        PUBLISHED_RATE_DPS,
        "a fault posted before the cancellation was installed after it"
    );

    sender
        .send(Fault::new("sensor_0.output_msg", rate_fault(4.0)).labelled("posted after"))
        .unwrap();
    faults.advance(2 * SECOND);
    assert_eq!(output.read().rate_dps, 4.0);
}

/// The sentinel is internal to the round trip, so the views the REST API reads
/// still answer `null` for a non-finite float.
///
/// bifrost's condition evaluator reads `effective` and treats `null` as "not a
/// number this instant", so a value that started answering `"inf"` here would
/// turn a watched field into a hard error at arm time.
#[test]
fn the_outward_facing_views_still_answer_null_for_a_non_finite() {
    let (registry, output) = registry_with_reading();
    output.write(reading(f64::NEG_INFINITY));

    registry
        .install(
            "sensor_0.output_msg",
            Request::replace(json!({ "/rate_dps": 2.0 })).unwrap(),
        )
        .unwrap();

    for view in [
        registry.upstream("sensor_0.output_msg").unwrap(),
        registry.effective("sensor_0.output_msg").unwrap(),
    ] {
        assert_eq!(
            view["angle_deg"],
            json!(null),
            "a non-finite float reached a caller as something other than null"
        );
    }
}
