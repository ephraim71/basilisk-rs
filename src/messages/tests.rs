use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use serde_json::json;

use crate::overrides::{Mode, Request};

use super::{Input, Output};
use crate::messages::PowerStorageStatusMsg;
use crate::messages::SpacecraftStateMsg;
use crate::messages::{ArrayMotorTorqueMsg, MAX_EFF_COUNT};

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
enum PayloadMessage {
    #[default]
    Unit,
    Newtype(f64),
    Tuple(f64, f64),
    Struct {
        value: f64,
    },
}

#[test]
fn a_replace_naming_every_field_substitutes_the_whole_message() {
    let output = Output::new(PowerStorageStatusMsg::default());

    output
        .set_override(
            Request::replace(json!({ "/storage_level_j": 4.0, "/storage_capacity_j": 5.0, "/current_net_power_w": 6.0 }))
            .unwrap(),
        )
        .unwrap();
    output.write(PowerStorageStatusMsg {
        storage_level_j: 10.0,
        storage_capacity_j: 20.0,
        current_net_power_w: 30.0,
    });

    assert_eq!(output.read().storage_level_j, 4.0);
    assert_eq!(output.read().storage_capacity_j, 5.0);
    assert_eq!(output.read().current_net_power_w, 6.0);
}

/// `MAX_EFF_COUNT` is past serde's built-in array impls, so these arrays
/// travel as sequences and their length is checked on the way back.
#[test]
fn maximum_effector_array_round_trips_and_rejects_wrong_lengths() {
    let mut message = ArrayMotorTorqueMsg::default();
    for (index, value) in message.motor_torque_nm.iter_mut().enumerate() {
        *value = index as f64 + 0.25;
    }

    let serialized = serde_json::to_value(&message).unwrap();
    let round_tripped: ArrayMotorTorqueMsg = serde_json::from_value(serialized.clone()).unwrap();
    assert_eq!(round_tripped, message);

    let mut short = serialized;
    short["motor_torque_nm"] = json!(vec![0.0; MAX_EFF_COUNT - 1]);
    assert!(serde_json::from_value::<ArrayMotorTorqueMsg>(short).is_err());
}

#[test]
fn a_map_message_remains_overrideable() {
    let output = Output::new(BTreeMap::from([("level".to_owned(), 1.0)]));

    output
        .set_override(Request::replace(json!({ "": { "level": 2.0 } })).unwrap())
        .expect("a serde-round-trippable map was refused");

    assert_eq!(output.read()["level"], 2.0);
}

#[test]
fn payload_enum_variants_remain_overrideable() {
    for (upstream, replacement, expected) in [
        (
            PayloadMessage::Newtype(1.0),
            json!({ "Newtype": 2.0 }),
            PayloadMessage::Newtype(2.0),
        ),
        (
            PayloadMessage::Tuple(1.0, 2.0),
            json!({ "Tuple": [3.0, 4.0] }),
            PayloadMessage::Tuple(3.0, 4.0),
        ),
        (
            PayloadMessage::Struct { value: 1.0 },
            json!({ "Struct": { "value": 5.0 } }),
            PayloadMessage::Struct { value: 5.0 },
        ),
    ] {
        let output = Output::new(upstream);
        output
            .set_override(Request::replace(json!({ "": replacement })).unwrap())
            .expect("a serde-round-trippable enum variant was refused");
        assert_eq!(output.read(), expected);
    }
}

#[test]
fn a_replace_cannot_send_an_internal_sentinel_as_a_float() {
    for sentinel in ["inf", "-inf", "NaN"] {
        let output = Output::new(PowerStorageStatusMsg::default());

        let error = output
            .set_override(Request::replace(json!({ "/storage_level_j": sentinel })).unwrap())
            .expect_err("an internal sentinel was accepted from an operator")
            .to_string();

        assert!(error.contains("invalid type: string"), "{error}");
    }

    let text = Output::new(String::new());
    text.set_override(Request::replace(json!({ "": "inf" })).unwrap())
        .expect("ordinary text matching a sentinel was refused");
    assert_eq!(text.read(), "inf");
}

fn status(storage_level_j: f64) -> PowerStorageStatusMsg {
    PowerStorageStatusMsg {
        storage_level_j,
        storage_capacity_j: 0.0,
        current_net_power_w: 0.0,
    }
}

fn connected_input(producer: &Output<PowerStorageStatusMsg>) -> Input<PowerStorageStatusMsg> {
    let mut input = Input::default();
    input.connect(producer.slot());
    input
}

#[test]
fn input_override_is_invisible_to_the_other_consumers() {
    let producer = Output::new(PowerStorageStatusMsg::default());
    let faulted = connected_input(&producer);
    let healthy = connected_input(&producer);
    producer.write(status(5.0));

    faulted
        .set_override(Request::replace(json!({ "/storage_level_j": 99.0 })).unwrap())
        .unwrap();

    assert_eq!(faulted.read().storage_level_j, 99.0);
    assert_eq!(healthy.read().storage_level_j, 5.0);
    assert_eq!(producer.read().storage_level_j, 5.0);
}

#[test]
fn input_upstream_is_the_producers_effective_value_not_its_raw_one() {
    let producer = Output::new(PowerStorageStatusMsg::default());
    let input = connected_input(&producer);
    producer.write(status(5.0));

    producer
        .set_override(Request::replace(json!({ "/storage_level_j": 7.0 })).unwrap())
        .unwrap();
    input
        .set_override(Request::replace(json!({ "/storage_capacity_j": 3.0 })).unwrap())
        .unwrap();

    assert_eq!(producer.read_upstream().storage_level_j, 5.0);
    assert_eq!(input.read_upstream().storage_level_j, 7.0);
    assert_eq!(input.read().storage_level_j, 7.0);
    assert_eq!(input.read().storage_capacity_j, 3.0);
}

#[test]
fn input_reads_upstream_again_once_its_override_is_cleared() {
    let producer = Output::new(PowerStorageStatusMsg::default());
    let input = connected_input(&producer);
    producer.write(status(5.0));
    input
        .set_override(Request::replace(json!({ "/storage_level_j": 99.0 })).unwrap())
        .unwrap();

    input.clear_override();

    assert_eq!(input.read().storage_level_j, 5.0);
}

#[test]
fn input_rejects_a_value_of_the_wrong_shape() {
    let producer = Output::new(PowerStorageStatusMsg::default());
    let input = connected_input(&producer);

    let result = input
        .set_override(Request::replace(json!({ "/storage_level_j": "not a number" })).unwrap());

    assert!(result.is_err(), "expected a type error, got {result:?}");
    assert!(!input.is_overridden());
}

#[test]
fn preview_override_reports_the_result_without_installing_it() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output.write(status(5.0));

    let previewed = output
        .preview_override(Request::replace(json!({ "/storage_level_j": 99.0 })).unwrap())
        .unwrap();

    assert_eq!(previewed.storage_level_j, 99.0);
    assert_eq!(output.read().storage_level_j, 5.0);
    assert!(!output.is_overridden());
}

#[test]
fn preview_override_accounts_for_the_patch_already_installed() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output.write(status(5.0));
    output
        .set_override(Request::replace(json!({ "/storage_level_j": 1.0 })).unwrap())
        .unwrap();

    let previewed = output
        .preview_override(Request::replace(json!({ "/storage_capacity_j": 2.0 })).unwrap())
        .unwrap();

    assert_eq!(previewed.storage_level_j, 1.0, "accumulated patch was lost");
    assert_eq!(previewed.storage_capacity_j, 2.0);
}

#[test]
fn input_preview_override_does_not_install() {
    let producer = Output::new(PowerStorageStatusMsg::default());
    let input = connected_input(&producer);
    producer.write(status(5.0));

    let previewed = input
        .preview_override(Request::replace(json!({ "/storage_level_j": 99.0 })).unwrap())
        .unwrap();

    assert_eq!(previewed.storage_level_j, 99.0);
    assert_eq!(input.read().storage_level_j, 5.0);
    assert!(!input.is_overridden());
}

#[test]
fn a_disconnected_input_still_reads_its_default() {
    let input: Input<PowerStorageStatusMsg> = Input::default();

    assert_eq!(input.read().storage_level_j, 0.0);
}

#[test]
fn output_passes_raw_value_without_override() {
    let output = Output::new(PowerStorageStatusMsg::default());

    output.write(PowerStorageStatusMsg {
        storage_level_j: 1.0,
        storage_capacity_j: 2.0,
        current_net_power_w: 3.0,
    });

    assert_eq!(output.read().storage_level_j, 1.0);
    assert_eq!(output.read().storage_capacity_j, 2.0);
    assert_eq!(output.read().current_net_power_w, 3.0);
}

#[test]
fn patch_override_updates_only_requested_fields() {
    let output = Output::new(PowerStorageStatusMsg::default());

    output
        .set_override(Request::replace(json!({ "/current_net_power_w": 9.0 })).unwrap())
        .unwrap();
    output.write(PowerStorageStatusMsg {
        storage_level_j: 1.0,
        storage_capacity_j: 2.0,
        current_net_power_w: 3.0,
    });

    assert_eq!(output.read().storage_level_j, 1.0);
    assert_eq!(output.read().storage_capacity_j, 2.0);
    assert_eq!(output.read().current_net_power_w, 9.0);
}

#[test]
fn patch_override_merges_successive_patches() {
    let output = Output::new(PowerStorageStatusMsg::default());

    output
        .set_override(Request::replace(json!({ "/storage_level_j": 1.0 })).unwrap())
        .unwrap();
    output
        .set_override(Request::replace(json!({ "/current_net_power_w": 3.0 })).unwrap())
        .unwrap();
    output.write(PowerStorageStatusMsg {
        storage_level_j: 10.0,
        storage_capacity_j: 20.0,
        current_net_power_w: 30.0,
    });

    assert_eq!(output.read().storage_level_j, 1.0);
    assert_eq!(output.read().storage_capacity_j, 20.0);
    assert_eq!(output.read().current_net_power_w, 3.0);
}

#[test]
fn clear_override_restores_latest_raw_value() {
    let output = Output::new(PowerStorageStatusMsg::default());

    output
        .set_override(Request::replace(json!({ "/current_net_power_w": 9.0 })).unwrap())
        .unwrap();
    output.write(PowerStorageStatusMsg {
        storage_level_j: 1.0,
        storage_capacity_j: 2.0,
        current_net_power_w: 3.0,
    });
    output.clear_override();

    assert_eq!(output.read().current_net_power_w, 3.0);
}

#[test]
fn set_override_rejects_a_value_of_the_wrong_shape() {
    let output = Output::new(PowerStorageStatusMsg::default());

    let result = output
        .set_override(Request::replace(json!({ "/storage_level_j": "not a number" })).unwrap());

    assert!(result.is_err(), "expected a type error, got {result:?}");
}

#[test]
fn a_rejected_override_leaves_the_previous_one_working() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output
        .set_override(Request::replace(json!({ "/storage_level_j": 1.0 })).unwrap())
        .unwrap();

    let _ = output
        .set_override(Request::replace(json!({ "/storage_capacity_j": "not a number" })).unwrap());
    output.write(PowerStorageStatusMsg {
        storage_level_j: 10.0,
        storage_capacity_j: 20.0,
        current_net_power_w: 30.0,
    });

    assert_eq!(output.read().storage_level_j, 1.0);
    assert_eq!(output.read().storage_capacity_j, 20.0);
}

#[test]
fn read_upstream_sees_through_an_active_override() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output
        .set_override(Request::replace(json!({ "/storage_level_j": 1.0 })).unwrap())
        .unwrap();

    output.write(PowerStorageStatusMsg {
        storage_level_j: 10.0,
        storage_capacity_j: 20.0,
        current_net_power_w: 30.0,
    });

    assert_eq!(output.read().storage_level_j, 1.0);
    assert_eq!(output.read_upstream().storage_level_j, 10.0);
}

#[test]
fn clearing_by_id_removes_that_layer_and_no_other() {
    let output = Output::new(PowerStorageStatusMsg::default());
    let first = output
        .set_override(Request::replace(json!({ "/storage_level_j": 1.0 })).unwrap())
        .unwrap();
    let second = output
        .set_override(Request::replace(json!({ "/storage_capacity_j": 2.0 })).unwrap())
        .unwrap();

    // An id identifies one layer. Clearing the inner one leaves the outer
    // one installed and working.
    assert!(output.clear_override_by_id(first));
    assert_eq!(output.installed_overrides().len(), 1);
    assert_eq!(output.read().storage_level_j, 0.0);
    assert_eq!(output.read().storage_capacity_j, 2.0);

    // An id that names no installed layer clears nothing.
    assert!(
        !output.clear_override_by_id(first),
        "an id already cleared removed a second layer"
    );
    assert_eq!(output.installed_overrides().len(), 1);

    assert!(output.clear_override_by_id(second));
    assert!(output.installed_overrides().is_empty());
}

#[test]
fn installed_overrides_reports_the_installed_rules() {
    let output = Output::new(PowerStorageStatusMsg::default());
    assert!(output.installed_overrides().is_empty());

    output
        .set_override(Request::replace(json!({ "/storage_level_j": 1.0 })).unwrap())
        .unwrap();

    let state = output.installed_overrides();
    assert_eq!(state.len(), 1);
    assert_eq!(state[0].mode(), Mode::Replace);
    assert_eq!(
        serde_json::to_value(state[0].document()).unwrap(),
        json!({ "/storage_level_j": 1.0 })
    );
}

/// Each patch is its own layer, so each stays removable on its own.
#[test]
fn a_patch_layers_over_the_one_before_it_rather_than_merging() {
    let output = Output::new(PowerStorageStatusMsg::default());
    let first = output
        .set_override(Request::replace(json!({ "/storage_level_j": 1.0 })).unwrap())
        .unwrap();
    output
        .set_override(Request::replace(json!({ "/storage_capacity_j": 2.0 })).unwrap())
        .unwrap();

    assert_eq!(output.installed_overrides().len(), 2);
    assert_eq!(output.read().storage_level_j, 1.0);
    assert_eq!(output.read().storage_capacity_j, 2.0);

    // Removing the inner layer leaves the outer one working.
    assert!(output.clear_override_by_id(first));
    assert_eq!(output.read().storage_level_j, 0.0);
    assert_eq!(output.read().storage_capacity_j, 2.0);
}

/// Removing one layer leaves the others in force. A timed rule must not
/// permanently destroy a fault that was already running when it expires.
#[test]
fn removing_a_rule_leaves_the_one_beneath_it_in_force() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output
        .set_override(Request::replace(json!({ "/storage_level_j": 1.0 })).unwrap())
        .unwrap();
    let replace = output
        .set_override(
            Request::replace(json!({ "/storage_level_j": 5.0, "/storage_capacity_j": 0.0, "/current_net_power_w": 0.0 }))
            .unwrap(),
        )
        .unwrap();

    // While it is installed the replace defines the whole message.
    assert_eq!(output.read().storage_level_j, 5.0);

    assert!(output.clear_override_by_id(replace));
    assert_eq!(
        output.read().storage_level_j,
        1.0,
        "the patch underneath the replace did not come back"
    );
}

/// A refused rule must leave the stack exactly as it found it: the candidate is
/// folded over a copy, and only a fold that succeeds is committed.
#[test]
fn a_rejected_rule_leaves_the_installed_stack_untouched() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output
        .set_override(Request::replace(json!({ "/storage_level_j": 1.0 })).unwrap())
        .unwrap();

    let refused = output
        .set_override(Request::replace(json!({ "/storage_level_j": "not a number" })).unwrap());

    assert!(refused.is_err());
    assert_eq!(output.installed_overrides().len(), 1);
    assert_eq!(
        output.read().storage_level_j,
        1.0,
        "a refused replace destroyed the override already installed"
    );
}

#[test]
fn freeze_override_holds_effective_value() {
    let output = Output::new(PowerStorageStatusMsg {
        storage_level_j: 1.0,
        storage_capacity_j: 2.0,
        current_net_power_w: 3.0,
    });

    output
        .set_override(Request::freeze(json!({})).unwrap())
        .unwrap();
    output.write(PowerStorageStatusMsg {
        storage_level_j: 10.0,
        storage_capacity_j: 20.0,
        current_net_power_w: 30.0,
    });

    assert_eq!(output.read().storage_level_j, 1.0);
    assert_eq!(output.read().storage_capacity_j, 2.0);
    assert_eq!(output.read().current_net_power_w, 3.0);
}

// -----------------------------------------------------------------------
// addressing one element of a vector
// -----------------------------------------------------------------------

fn spinning(rate: [f64; 3]) -> SpacecraftStateMsg {
    SpacecraftStateMsg {
        omega_radps: nalgebra::Vector3::new(rate[0], rate[1], rate[2]),
        ..SpacecraftStateMsg::default()
    }
}

fn assign(path: &str, value: f64) -> serde_json::Value {
    json!({ path: value })
}

#[test]
fn an_operation_document_changes_one_element_and_leaves_its_siblings_upstream() {
    let output = Output::new(SpacecraftStateMsg::default());
    output.write(spinning([1.0, 2.0, 3.0]));

    output
        .set_override(Request::replace(assign("/omega_radps/1", 9.0)).unwrap())
        .unwrap();

    assert_eq!(
        output.read().omega_radps,
        nalgebra::Vector3::new(1.0, 9.0, 3.0)
    );

    // The siblings track upstream rather than being pinned to what they
    // held when the rule was installed. This is the whole point of the mode.
    output.write(spinning([4.0, 5.0, 6.0]));
    assert_eq!(
        output.read().omega_radps,
        nalgebra::Vector3::new(4.0, 9.0, 6.0)
    );
}

/// Every rule is relative, so a document addressing one location composes with
/// the merge beneath it instead of replacing the whole message.
#[test]
fn an_operation_document_composes_with_the_merge_beneath_it() {
    let output = Output::new(SpacecraftStateMsg::default());
    output.write(spinning([1.0, 2.0, 3.0]));

    output
        .set_override(Request::replace(json!({ "/sigma_bn": [7.0, 0.0, 0.0] })).unwrap())
        .unwrap();
    output
        .set_override(Request::replace(assign("/omega_radps/0", 9.0)).unwrap())
        .unwrap();

    let effective = output.read();
    assert_eq!(
        effective.omega_radps.x, 9.0,
        "the pointer replace did not apply"
    );
    assert_eq!(
        effective.sigma_bn.x, 7.0,
        "the element assignment masked the rule below it instead of composing"
    );
}

#[test]
fn a_later_rule_covers_an_earlier_one_and_gives_it_back_when_removed() {
    let output = Output::new(SpacecraftStateMsg::default());
    output.write(spinning([1.0, 2.0, 3.0]));

    output
        .set_override(Request::replace(assign("/omega_radps/1", 9.0)).unwrap())
        .unwrap();
    let covering = output
        .set_override(
            Request::replace(
                json!({ "": serde_json::to_value(spinning([0.0, 0.0, 0.0])).unwrap() }),
            )
            .unwrap(),
        )
        .unwrap();
    assert_eq!(output.read().omega_radps.y, 0.0, "the replace did not take");

    assert!(output.clear_override_by_id(covering));
    assert_eq!(
        output.read().omega_radps.y,
        9.0,
        "removing the later rule lost the one it was covering"
    );
}

/// Which of two overlapping assignments wins must not depend on the order the
/// payload was written in, nor on a feature some other crate turned on.
#[test]
fn a_deeper_assignment_wins_whatever_order_the_payload_arrived_in() {
    for payload in [
        json!({ "/inner/value": 9.0, "/inner": { "value": 4.0 } }),
        json!({ "/inner": { "value": 4.0 }, "/inner/value": 9.0 }),
    ] {
        let output = Output::new(Optional::default());
        output.write(Optional {
            keep: 0.0,
            inner: Some(Inner { value: 0.0 }),
        });
        output
            .set_override(Request::replace(payload.clone()).unwrap())
            .unwrap();

        assert_eq!(
            output.read().inner.unwrap().value,
            9.0,
            "the container overwrote the field inside it: {payload}"
        );
    }
}

/// A key that is not a pointer is answered with the pointer it should have
/// been. This is the mistake a client makes when it carries a field name where
/// a path belongs, and it is refused when the request is *parsed*, so it never
/// reaches a port.
#[test]
fn a_replace_key_that_is_not_a_pointer_is_answered_with_one() {
    let error = Request::replace(json!({ "omega_radps": [1.0, 2.0, 3.0] }))
        .expect_err("a bare field name was accepted as a path")
        .to_string();

    assert!(error.contains("write '/omega_radps' instead"), "{error}");
}

#[test]
fn a_pointer_that_does_not_resolve_is_refused_not_ignored() {
    let output = Output::new(SpacecraftStateMsg::default());
    output.write(spinning([1.0, 2.0, 3.0]));

    let error = output
        .set_override(Request::replace(assign("/omega_radps/7", 9.0)).unwrap())
        .expect_err("an index past the end of a fixed vector was accepted");
    assert!(error.to_string().contains("does not resolve"), "{error}");

    // And nothing was installed by the attempt.
    assert!(!output.is_overridden());
    assert_eq!(output.read().omega_radps.x, 1.0);
}

/// One payload shape, so a whole array and one of its elements are the same
/// kind of request and anything else is refused by naming the shape it wanted.
/// An RFC 6902 operation document is the likely wrong guess, being what an array
/// of `replace` operations looks like elsewhere.
#[test]
fn a_replace_takes_a_pointer_map_and_nothing_else() {
    let output = Output::new(SpacecraftStateMsg::default());

    output
        .set_override(Request::replace(json!({ "/omega_radps": [1.0, 2.0, 3.0] })).unwrap())
        .expect("a whole-array assignment was refused");
    assert_eq!(output.read().omega_radps.z, 3.0);

    output
        .set_override(Request::replace(assign("/omega_radps/0", 9.0)).unwrap())
        .expect("an element assignment was refused");
    assert_eq!(output.read().omega_radps.x, 9.0);

    for wrong in [
        json!([{ "op": "replace", "path": "/omega_radps/0", "value": 1.0 }]),
        json!(42),
        json!("a string"),
        json!(null),
    ] {
        let error = Request::replace(wrong.clone())
            .expect_err("a payload of the wrong shape was accepted")
            .to_string();
        assert!(error.contains("JSON pointers"), "{wrong}: {error}");
    }
}

// ---------------------------------------------------------------------------
// per-field freeze and default
// ---------------------------------------------------------------------------

/// A freeze naming fields holds those and no others. Everything it does not name
/// keeps tracking upstream, which is the difference between pinning one axis of
/// a body rate and pinning the whole vector.
#[test]
fn a_freeze_naming_fields_holds_those_and_lets_the_rest_track() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output.write(status(1.0));

    output
        .set_override(Request::freeze(json!(["/storage_level_j"])).unwrap())
        .unwrap();

    output.write(PowerStorageStatusMsg {
        storage_level_j: 99.0,
        storage_capacity_j: 42.0,
        current_net_power_w: 7.0,
    });

    assert_eq!(output.read().storage_level_j, 1.0, "the named field moved");
    assert_eq!(
        output.read().storage_capacity_j,
        42.0,
        "an unnamed field stopped tracking upstream"
    );
    assert_eq!(output.read().current_net_power_w, 7.0);
}

/// An empty payload still means the whole message, which is how both modes
/// behaved when neither took one.
#[test]
fn an_empty_selection_still_means_the_whole_message() {
    for empty in [json!(null), json!({}), json!([])] {
        let frozen = Output::new(PowerStorageStatusMsg::default());
        frozen.write(status(1.0));
        frozen
            .set_override(Request::freeze(empty.clone()).unwrap())
            .unwrap();
        frozen.write(status(99.0));
        assert_eq!(
            frozen.read().storage_level_j,
            1.0,
            "{empty} did not freeze all"
        );
    }
}

/// Nothing masks any more, so a stack is the composition of all of it — a
/// per-field freeze under a replace of a different field leaves both in force.
#[test]
fn every_mode_composes_because_none_of_them_mask() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output.write(status(1.0));

    output
        .set_override(Request::freeze(json!(["/storage_level_j"])).unwrap())
        .unwrap();
    output
        .set_override(Request::replace(json!({ "/current_net_power_w": 3.0 })).unwrap())
        .unwrap();
    output.write(PowerStorageStatusMsg {
        storage_level_j: 99.0,
        storage_capacity_j: 88.0,
        current_net_power_w: 77.0,
    });

    assert_eq!(output.read().storage_level_j, 1.0, "the freeze was masked");
    assert_eq!(
        output.read().storage_capacity_j,
        88.0,
        "a field no rule names did not reach the reader"
    );
    assert_eq!(
        output.read().current_net_power_w,
        3.0,
        "the replace was masked"
    );
}

/// A sun sensor facing away from the sun publishes NaN for its angles, and JSON
/// has no spelling for one, so the round trip an override performs used to refuse
/// the whole message -- naming a field the operator never touched. The fault the
/// operator asked for is unrelated to the field that could not be written.
#[test]
fn a_replace_lands_on_a_message_whose_other_float_is_not_finite() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output.write(PowerStorageStatusMsg {
        storage_level_j: f64::NAN,
        storage_capacity_j: 6.0,
        current_net_power_w: f64::NAN,
    });

    output
        .set_override(Request::replace(json!({ "/storage_capacity_j": 20.0 })).unwrap())
        .expect("a replace was refused because another field held NaN");

    assert_eq!(
        output.read().storage_capacity_j,
        20.0,
        "the named field did not take the value"
    );
    assert!(
        output.read().storage_level_j.is_nan(),
        "a field no rule names lost its NaN"
    );
    assert!(
        output.read().current_net_power_w.is_nan(),
        "a field no rule names lost its NaN"
    );
}

/// The same message, frozen: a freeze captures the live value, and one of them is
/// a NaN that has to survive being captured and written back.
#[test]
fn a_freeze_can_capture_a_float_that_is_not_finite() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output.write(PowerStorageStatusMsg {
        storage_level_j: f64::NAN,
        storage_capacity_j: 6.0,
        current_net_power_w: 0.0,
    });

    output
        .set_override(Request::freeze(json!(["/storage_level_j"])).unwrap())
        .expect("a freeze of a NaN was refused");
    output.write(status(99.0));

    assert!(
        output.read().storage_level_j.is_nan(),
        "the freeze did not hold the NaN it captured"
    );
}

#[test]
fn a_freeze_keeps_the_sign_of_an_infinity() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output.write(PowerStorageStatusMsg {
        storage_level_j: f64::INFINITY,
        storage_capacity_j: 6.0,
        current_net_power_w: 0.0,
    });

    output
        .set_override(Request::freeze(json!(["/storage_level_j"])).unwrap())
        .expect("a freeze of infinity was refused");
    output.write(status(99.0));

    assert_eq!(output.read().storage_level_j, f64::INFINITY);
}

/// A pointer that cannot resolve is refused when the rule is built, so no layer
/// is installed that would quietly do nothing on every write.
#[test]
fn a_selection_naming_an_unresolvable_pointer_is_refused_at_install() {
    let output = Output::new(SpacecraftStateMsg::default());
    output.write(spinning([1.0, 2.0, 3.0]));

    let error = output
        .set_override(Request::freeze(json!(["/omega_radps/7"])).unwrap())
        .expect_err("an index past the end of a fixed vector was accepted")
        .to_string();
    assert!(error.contains("does not resolve"), "{error}");
    assert!(
        !output.is_overridden(),
        "a rule that cannot apply was installed"
    );
}

/// An array element is a pointer segment like any other, so one axis freezes
/// without pinning its siblings.
#[test]
fn a_freeze_can_name_one_element_of_a_vector() {
    let output = Output::new(SpacecraftStateMsg::default());
    output.write(spinning([1.0, 2.0, 3.0]));

    output
        .set_override(Request::freeze(json!(["/omega_radps/1"])).unwrap())
        .unwrap();
    output.write(spinning([9.0, 9.0, 9.0]));

    assert_eq!(output.read().omega_radps.y, 2.0, "the named axis moved");
    assert_eq!(
        output.read().omega_radps.x,
        9.0,
        "a sibling axis was pinned"
    );
    assert_eq!(
        output.read().omega_radps.z,
        9.0,
        "a sibling axis was pinned"
    );
}

/// A path that is not a pointer is refused with the pointer it should have been,
/// which is the correction anyone carrying a dotted path from the old schema
/// needs.
#[test]
fn a_dotted_path_in_a_selection_is_answered_with_its_pointer() {
    let error = Request::freeze(json!(["omega_radps.1"]))
        .expect_err("a dotted path was accepted as a pointer")
        .to_string();
    assert!(error.contains("/omega_radps/1"), "{error}");
}

// ---------------------------------------------------------------------------
// a stale layer must not cancel the rest
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
struct Inner {
    value: f64,
}

/// An option a module populates and later stops populating, which is what makes
/// a pointer beneath it resolve at install and miss later.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
struct Optional {
    keep: f64,
    inner: Option<Inner>,
}

/// Failing the fold over a rule that has nothing to do this tick discarded every
/// other override and published the raw upstream value — a fault-injection
/// mechanism quietly injecting nothing.
#[test]
fn a_rule_that_stops_resolving_is_skipped_without_cancelling_the_others() {
    let output = Output::new(Optional::default());
    output.write(Optional {
        keep: 1.0,
        inner: Some(Inner { value: 5.0 }),
    });

    output
        .set_override(Request::replace(json!({ "/inner/value": 9.0 })).unwrap())
        .expect("the pointer resolves while the option is populated");
    output
        .set_override(Request::freeze(json!(null)).unwrap())
        .unwrap();

    // the option goes away, so the pointer rule has nothing to address
    output.write(Optional {
        keep: 77.0,
        inner: None,
    });

    assert_eq!(
        output.read().keep,
        1.0,
        "the freeze was discarded along with the rule that could not apply"
    );
    assert_eq!(
        output.installed_overrides().len(),
        2,
        "a rule was removed rather than skipped for this one value"
    );

    // and it comes back on its own once the option is populated again
    output.write(Optional {
        keep: 88.0,
        inner: Some(Inner { value: 1.0 }),
    });
    assert_eq!(
        output.read().inner.unwrap().value,
        9.0,
        "the rule did not resume"
    );
}

/// A candidate that cannot apply is still refused at install: leniency belongs
/// to an installed stack, not to admitting a new rule.
#[test]
fn a_candidate_that_cannot_apply_is_still_refused_at_install() {
    let output = Output::new(Optional::default());
    output.write(Optional {
        keep: 1.0,
        inner: None,
    });

    output
        .set_override(Request::replace(json!({ "/inner/value": 9.0 })).unwrap())
        .expect_err("a rule that cannot apply right now was installed");
    assert!(!output.is_overridden());
}

/// The other half of that leniency. Judging the rules already in force strictly
/// at install let one layer with nothing to do this tick refuse every unrelated
/// fault behind it, which is the same "quietly injects nothing" failure reached
/// from the control path instead of the write path.
#[test]
fn a_stale_layer_does_not_block_a_later_install() {
    let output = Output::new(Optional::default());
    output.write(Optional {
        keep: 1.0,
        inner: Some(Inner { value: 5.0 }),
    });
    output
        .set_override(Request::replace(json!({ "/inner/value": 9.0 })).unwrap())
        .unwrap();

    output.write(Optional {
        keep: 77.0,
        inner: None,
    });

    output
        .preview_override(Request::replace(json!({ "/keep": 8.0 })).unwrap())
        .expect("a preview was refused over a rule it has nothing to do with");
    output
        .set_override(Request::replace(json!({ "/keep": 8.0 })).unwrap())
        .expect("an install was refused over a rule it has nothing to do with");
    assert_eq!(output.read().keep, 8.0);
}

/// The same JSON must mean the same request whichever side built it.
#[test]
fn an_empty_field_list_means_the_whole_message_however_it_was_built() {
    use crate::overrides::Selection;

    let built = Request::Freeze(Selection::fields(vec![]));
    let round_tripped: Request =
        serde_json::from_value(serde_json::to_value(&built).unwrap()).unwrap();
    assert_eq!(
        built, round_tripped,
        "the same JSON produced a different request"
    );

    let output = Output::new(PowerStorageStatusMsg::default());
    output.write(status(1.0));
    output.set_override(built).unwrap();
    output.write(status(99.0));
    assert_eq!(
        output.read().storage_level_j,
        1.0,
        "an empty field list froze nothing instead of the whole message"
    );
}

/// `~` introduces an escape and RFC 6901 defines exactly two, so a stray one is
/// a typo rather than a literal tilde.
#[test]
fn a_pointer_with_an_undefined_escape_is_refused() {
    use crate::overrides::Pointer;

    assert!(Pointer::parse("/a~2b").is_err(), "'~2' was accepted");
    assert!(
        Pointer::parse("/trailing~").is_err(),
        "a trailing '~' was accepted"
    );
    assert!(
        Pointer::parse("/a~0b").is_ok(),
        "'~0' should be a literal '~'"
    );
    assert!(
        Pointer::parse("/a~1b").is_ok(),
        "'~1' should be a literal '/'"
    );
}

/// A field the patch never named keeps the value it had, including the sign of
/// an infinity. Before the serializer went through `non_finite`, every
/// non-finite float came back as NaN because JSON writes all three as `null`.
#[test]
fn a_replace_leaves_an_untouched_infinity_alone() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output.write(PowerStorageStatusMsg {
        storage_level_j: f64::INFINITY,
        storage_capacity_j: 6.0,
        current_net_power_w: f64::NEG_INFINITY,
    });

    output
        .set_override(Request::replace(json!({ "/storage_capacity_j": 20.0 })).unwrap())
        .expect("a replace was refused because another field was not finite");

    let applied = output.read();
    assert_eq!(applied.storage_capacity_j, 20.0, "the fault did not land");
    assert_eq!(
        applied.storage_level_j,
        f64::INFINITY,
        "an untouched infinity was flattened"
    );
    assert_eq!(
        applied.current_net_power_w,
        f64::NEG_INFINITY,
        "an untouched infinity lost its sign"
    );
}

/// The ports of upstream FSW message payloads: each one round-trips through
/// JSON with every array at full length, defaults to upstream's zeroed
/// payload, and flattens to exactly one telemetry column per value it holds.
mod upstream_payloads {
    use std::any::type_name;
    use std::collections::HashSet;
    use std::fmt::Debug;

    use nalgebra::{Matrix3, Quaternion, SMatrix, UnitQuaternion, Vector2, Vector3};
    use serde_json::{Value, json};

    use crate::messages::{
        AccelerometerDataMsg, AccelerometerPacket, AccessMsg, ArrayEffectorLockMsg,
        AttitudeStateMsg, BodyForceCommandMsg, BodyHeadingMsg, CameraConfigMsg, CameraRenderMode,
        CssArraySensorMsg, CssConfigMsg, CssUnitConfigMsg, DeviceCommandMsg, DeviceState,
        DeviceStatusMsg, DipoleRequestBodyMsg, DvBurnCommandMsg, DvExecutionDataMsg, EffectorLock,
        EphemerisMsg, GroundStateMsg, HeadingFilterMsg, HillRelativeStateMsg, ImuMsg,
        ImuSensorBodyMsg, InertialFilterMsg, InertialForceCommandMsg, InertialHeadingMsg,
        MAX_ACC_BUF_PKT, MAX_EFF_COUNT, MAX_LIMB_PNTS, MAX_NUM_CSS_SENSORS,
        NavigationTranslationMsg, OpNavCirclesMsg, OpNavFilterMsg, OpNavLimbMsg, OpNavMsg,
        PixelLineFilterMsg, PrescribedRotationMsg, RateCommandMsg, ReconfigBurnArrayInfoMsg,
        ReconfigBurnInfoMsg, ReconfigBurnStatus, RwConfigElementMsg, RwConstellationMsg,
        SKF_N_STATES, SimulationMessage, SpacecraftStateMsg, StarTrackerAttitudeMsg,
        StarTrackerMsg, StripStateMsg, SunlineFilterMsg, TdbVehicleClockCorrelationMsg,
        ThrusterArrayConfigMsg, ThrusterArrayForceCommandMsg, ThrusterArrayOnTimeCommandMsg,
        ThrusterConfigMsg, VizSetting,
    };
    use crate::telemetry::TelemetryMessage;

    /// Runs `$check::<T>()` for every ported payload that has telemetry, which
    /// is all of them except [`CameraConfigMsg`].
    macro_rules! for_each_telemetry_message {
        ($check:ident) => {
            $check::<AccelerometerDataMsg>();
            $check::<AccelerometerPacket>();
            $check::<AccessMsg>();
            $check::<ArrayEffectorLockMsg>();
            $check::<AttitudeStateMsg>();
            $check::<BodyForceCommandMsg>();
            $check::<BodyHeadingMsg>();
            $check::<CssArraySensorMsg>();
            $check::<CssConfigMsg>();
            $check::<CssUnitConfigMsg>();
            $check::<DeviceCommandMsg>();
            $check::<DeviceStatusMsg>();
            $check::<DipoleRequestBodyMsg>();
            $check::<DvBurnCommandMsg>();
            $check::<DvExecutionDataMsg>();
            $check::<EphemerisMsg>();
            $check::<GroundStateMsg>();
            $check::<HeadingFilterMsg>();
            $check::<HillRelativeStateMsg>();
            $check::<ImuMsg>();
            $check::<ImuSensorBodyMsg>();
            $check::<InertialFilterMsg>();
            $check::<InertialForceCommandMsg>();
            $check::<InertialHeadingMsg>();
            $check::<NavigationTranslationMsg>();
            $check::<OpNavCirclesMsg>();
            $check::<OpNavFilterMsg>();
            $check::<OpNavLimbMsg>();
            $check::<OpNavMsg>();
            $check::<PixelLineFilterMsg>();
            $check::<PrescribedRotationMsg>();
            $check::<RateCommandMsg>();
            $check::<ReconfigBurnArrayInfoMsg>();
            $check::<ReconfigBurnInfoMsg>();
            $check::<RwConfigElementMsg>();
            $check::<RwConstellationMsg>();
            $check::<StarTrackerAttitudeMsg>();
            $check::<StarTrackerMsg>();
            $check::<StripStateMsg>();
            $check::<SunlineFilterMsg>();
            $check::<TdbVehicleClockCorrelationMsg>();
            $check::<ThrusterArrayConfigMsg>();
            $check::<ThrusterArrayForceCommandMsg>();
            $check::<ThrusterArrayOnTimeCommandMsg>();
            $check::<ThrusterConfigMsg>();
        };
    }

    /// Every unit enum the payloads hold, as `(zeroed default, non-default
    /// variant, that variant's upstream code)`. The variant names are unique
    /// across the enums, so a serialised name identifies its enum.
    const VARIANTS: [(&str, &str, f64); 5] = [
        ("Off", "On", 1.0),
        ("Unlocked", "Locked", 1.0),
        ("NotScheduled", "Burned", 2.0),
        ("Visual", "DepthMap", 1.0),
        ("UseVizDefault", "Enabled", 1.0),
    ];

    /// Gives every number in `value` a distinct non-zero value — fractional
    /// where the field is a float, integral where it is an integer — sets every
    /// flag, names every empty string, and moves every enum off its default.
    fn fill(value: &mut Value, counter: &mut u64) {
        match value {
            Value::Number(number) => {
                *counter += 1;
                let filled = if number.is_f64() {
                    json!(*counter as f64 + 0.25)
                } else {
                    json!(*counter)
                };
                *value = filled;
            }
            Value::Bool(flag) => *flag = true,
            Value::String(text) if text.is_empty() => *text = format!("text {counter}"),
            Value::String(variant) => {
                let (_, other, _) = VARIANTS
                    .iter()
                    .find(|(default, _, _)| default == variant)
                    .unwrap_or_else(|| panic!("add the enum holding {variant:?} to VARIANTS"));
                *variant = (*other).to_owned();
            }
            Value::Null => {}
            Value::Array(items) => items.iter_mut().for_each(|item| fill(item, counter)),
            Value::Object(fields) => fields.values_mut().for_each(|field| fill(field, counter)),
        }
    }

    fn filled_json<T: SimulationMessage>() -> Value {
        let mut value = serde_json::to_value(T::default()).unwrap();
        fill(&mut value, &mut 0);
        value
    }

    fn filled<T: SimulationMessage>() -> T {
        serde_json::from_value(filled_json::<T>())
            .unwrap_or_else(|error| panic!("{} rejected its own shape: {error}", type_name::<T>()))
    }

    /// Every leaf of `value`, as the number its telemetry column holds: a
    /// flag is 1 or 0, and an enum variant its upstream code.
    fn leaf_values(value: &Value, leaves: &mut Vec<f64>) {
        match value {
            Value::Number(number) => leaves.push(number.as_f64().unwrap()),
            Value::Bool(flag) => leaves.push(if *flag { 1.0 } else { 0.0 }),
            Value::String(variant) => leaves.push(
                VARIANTS
                    .iter()
                    .find_map(|(default, other, code)| {
                        (default == variant)
                            .then_some(0.0)
                            .or((other == variant).then_some(*code))
                    })
                    .unwrap_or_else(|| panic!("{variant:?} is not a known enum variant")),
            ),
            Value::Null => panic!("a payload serialised a null"),
            Value::Array(items) => items.iter().for_each(|item| leaf_values(item, leaves)),
            Value::Object(fields) => fields.values().for_each(|field| leaf_values(field, leaves)),
        }
    }

    fn round_trips_at_full_size<T: SimulationMessage + PartialEq + Debug>() {
        let json = filled_json::<T>();
        let message = filled::<T>();

        assert_ne!(
            message,
            T::default(),
            "{} kept its default",
            type_name::<T>()
        );
        assert_eq!(
            serde_json::to_value(&message).unwrap(),
            json,
            "{} did not serialise back to what it was read from",
            type_name::<T>()
        );
        let back: T = serde_json::from_value(serde_json::to_value(&message).unwrap()).unwrap();
        assert_eq!(
            back,
            message,
            "{} changed in a round trip",
            type_name::<T>()
        );
    }

    #[test]
    fn every_payload_round_trips_with_full_length_arrays() {
        for_each_telemetry_message!(round_trips_at_full_size);
        round_trips_at_full_size::<CameraConfigMsg>();
    }

    fn assert_zeroed(value: &Value, path: &str, message: &str) {
        match value {
            Value::Number(number) => {
                assert_eq!(number.as_f64(), Some(0.0), "{message}{path} is not zero");
            }
            Value::Bool(flag) => assert!(!flag, "{message}{path} is set"),
            Value::String(text) => assert!(
                text.is_empty() || VARIANTS.iter().any(|(default, _, _)| default == text),
                "{message}{path} is {text:?}"
            ),
            Value::Null => panic!("{message}{path} is null"),
            Value::Array(items) => {
                for (index, item) in items.iter().enumerate() {
                    assert_zeroed(item, &format!("{path}/{index}"), message);
                }
            }
            Value::Object(fields) => {
                for (name, field) in fields {
                    assert_zeroed(field, &format!("{path}/{name}"), message);
                }
            }
        }
    }

    fn defaults_to_the_zeroed_payload<T: SimulationMessage + TelemetryMessage>() {
        let default = T::default();
        assert_zeroed(
            &serde_json::to_value(&default).unwrap(),
            "",
            type_name::<T>(),
        );
        for field in default.flatten() {
            assert_eq!(
                field.value,
                0.0,
                "{}.{} is not zero",
                type_name::<T>(),
                field.path
            );
        }
    }

    /// Upstream zero-initialises every payload, and an unconnected `Input`
    /// reads `T::default()`, so the two must agree for a port to behave like
    /// an unlinked upstream message.
    #[test]
    fn every_payload_defaults_to_upstreams_zeroed_payload() {
        // `StarTrackerMsg` documents an identity attitude instead.
        let star_tracker = StarTrackerMsg::default();
        assert_eq!(star_tracker.time_tag_nanos, 0);
        assert_eq!(
            star_tracker.attitude_inertial_to_sensor,
            UnitQuaternion::identity()
        );

        macro_rules! zeroed {
            ($($message:ty),+ $(,)?) => {
                $(defaults_to_the_zeroed_payload::<$message>();)+
            };
        }
        zeroed!(
            AccelerometerDataMsg,
            AccelerometerPacket,
            AccessMsg,
            ArrayEffectorLockMsg,
            AttitudeStateMsg,
            BodyForceCommandMsg,
            BodyHeadingMsg,
            CssArraySensorMsg,
            CssConfigMsg,
            CssUnitConfigMsg,
            DeviceCommandMsg,
            DeviceStatusMsg,
            DipoleRequestBodyMsg,
            DvBurnCommandMsg,
            DvExecutionDataMsg,
            EphemerisMsg,
            GroundStateMsg,
            HeadingFilterMsg,
            HillRelativeStateMsg,
            ImuMsg,
            ImuSensorBodyMsg,
            InertialFilterMsg,
            InertialForceCommandMsg,
            InertialHeadingMsg,
            NavigationTranslationMsg,
            OpNavCirclesMsg,
            OpNavFilterMsg,
            OpNavLimbMsg,
            OpNavMsg,
            PixelLineFilterMsg,
            PrescribedRotationMsg,
            RateCommandMsg,
            ReconfigBurnArrayInfoMsg,
            ReconfigBurnInfoMsg,
            RwConfigElementMsg,
            RwConstellationMsg,
            StarTrackerAttitudeMsg,
            StripStateMsg,
            SunlineFilterMsg,
            TdbVehicleClockCorrelationMsg,
            ThrusterArrayConfigMsg,
            ThrusterArrayForceCommandMsg,
            ThrusterArrayOnTimeCommandMsg,
            ThrusterConfigMsg,
        );

        let camera = CameraConfigMsg::default();
        assert_zeroed(
            &serde_json::to_value(&camera).unwrap(),
            "",
            "CameraConfigMsg",
        );
        assert_eq!(camera.render_mode as u8, 0);
        assert_eq!(camera.post_processing_on as i8, 0);
        assert_eq!(camera.show_hud_elements_in_image as i8, 0);
    }

    /// Every value a message holds appears in exactly one uniquely named
    /// column — a field left out of `impl_telemetry!` would be missing here.
    fn flattens_one_column_per_value<T: SimulationMessage + TelemetryMessage>() {
        let message = filled::<T>();
        let fields = message.flatten();

        let mut paths = HashSet::new();
        for field in &fields {
            assert!(
                paths.insert(field.path.as_str()),
                "{} repeats the column {}",
                type_name::<T>(),
                field.path
            );
        }

        let mut expected = Vec::new();
        leaf_values(&serde_json::to_value(&message).unwrap(), &mut expected);
        let mut actual: Vec<f64> = fields.iter().map(|field| field.value).collect();
        expected.sort_by(f64::total_cmp);
        actual.sort_by(f64::total_cmp);
        assert_eq!(
            actual,
            expected,
            "{} telemetry does not hold exactly its values",
            type_name::<T>()
        );
    }

    #[test]
    fn telemetry_holds_every_value_once() {
        for_each_telemetry_message!(flattens_one_column_per_value);
    }

    fn paths<T: TelemetryMessage>(message: &T) -> Vec<String> {
        message
            .flatten()
            .into_iter()
            .map(|field| field.path)
            .collect()
    }

    #[test]
    fn telemetry_paths_follow_the_existing_conventions() {
        assert_eq!(
            paths(&NavigationTranslationMsg::default()),
            [
                "time_tag_s",
                "position_bn_n_m.x",
                "position_bn_n_m.y",
                "position_bn_n_m.z",
                "velocity_bn_n_mps.x",
                "velocity_bn_n_mps.y",
                "velocity_bn_n_mps.z",
                "accumulated_dv_b_mps.x",
                "accumulated_dv_b_mps.y",
                "accumulated_dv_b_mps.z",
            ]
        );

        let sunline = paths(&SunlineFilterMsg::default());
        for path in [
            "covar.m11",
            "covar.m66",
            "state.0",
            "state.5",
            "state_error.5",
            "post_fit_res.31",
            "num_obs",
        ] {
            assert!(sunline.iter().any(|p| p == path), "missing {path}");
        }
        assert_eq!(
            sunline[1..7],
            [
                "covar.m11",
                "covar.m12",
                "covar.m13",
                "covar.m14",
                "covar.m15",
                "covar.m16"
            ]
        );

        let thrusters = paths(&ThrusterArrayConfigMsg::default());
        assert_eq!(thrusters[0], "num_thrusters");
        assert_eq!(thrusters[1], "thrusters.0.position_thrust_b_m.x");
        assert_eq!(
            thrusters.last().map(String::as_str),
            Some("thrusters.35.swirl_torque_nm")
        );

        let limb = paths(&OpNavLimbMsg::default());
        assert_eq!(
            limb.last().map(String::as_str),
            Some("limb_points_px.1999.y")
        );

        // The columns the hand-written telemetry used keep their names.
        let imu = paths(&ImuMsg::default());
        assert_eq!(
            imu[9..],
            [
                "angular_rate_sensor_radps.x",
                "angular_rate_sensor_radps.y",
                "angular_rate_sensor_radps.z"
            ]
        );
        assert_eq!(
            paths(&StarTrackerMsg::default()),
            [
                "time_tag_nanos",
                "attitude_inertial_to_sensor.x",
                "attitude_inertial_to_sensor.y",
                "attitude_inertial_to_sensor.z",
                "attitude_inertial_to_sensor.w",
            ]
        );
    }

    #[test]
    fn matrix_telemetry_is_row_major_and_enums_report_their_upstream_code() {
        let op_nav = OpNavMsg {
            covar_n_m2: Matrix3::from_row_slice(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]),
            ..OpNavMsg::default()
        };
        let value = |path: &str| {
            op_nav
                .flatten()
                .into_iter()
                .find(|field| field.path == path)
                .map(|field| field.value)
        };
        assert_eq!(value("covar_n_m2.m12"), Some(2.0));
        assert_eq!(value("covar_n_m2.m21"), Some(4.0));
        assert_eq!(value("covar_n_m2.m33"), Some(9.0));

        let mut lock = ArrayEffectorLockMsg::default();
        lock.effector_lock[1] = EffectorLock::Locked;
        let fields = lock.flatten();
        assert_eq!(fields[0].value, 0.0);
        assert_eq!(fields[1].path, "effector_lock.1");
        assert_eq!(fields[1].value, 1.0);

        let mut burns = ReconfigBurnArrayInfoMsg::default();
        burns.burn_array[2].status = ReconfigBurnStatus::Skipped;
        let skipped = burns
            .flatten()
            .into_iter()
            .find(|field| field.path == "burn_array.2.status")
            .expect("the burn status has a column");
        assert_eq!(skipped.value, 3.0);
    }

    #[test]
    fn discrete_states_keep_their_upstream_codes_and_round_trip() {
        assert_eq!(DeviceState::Off as u8, 0);
        assert_eq!(DeviceState::On as u8, 1);
        assert!(DeviceState::On.is_on() && !DeviceState::Off.is_on());
        assert_eq!(EffectorLock::Unlocked as u8, 0);
        assert_eq!(EffectorLock::Locked as u8, 1);
        assert_eq!(ReconfigBurnStatus::NotScheduled as u8, 0);
        assert_eq!(ReconfigBurnStatus::Pending as u8, 1);
        assert_eq!(ReconfigBurnStatus::Burned as u8, 2);
        assert_eq!(ReconfigBurnStatus::Skipped as u8, 3);
        assert_eq!(CameraRenderMode::Visual as u8, 0);
        assert_eq!(CameraRenderMode::DepthMap as u8, 1);
        assert_eq!(VizSetting::UseVizDefault as i8, 0);
        assert_eq!(VizSetting::Disabled as i8, -1);
        assert_eq!(VizSetting::Enabled as i8, 1);

        let status = DeviceStatusMsg {
            device_status: DeviceState::On,
        };
        let back: DeviceStatusMsg =
            serde_json::from_value(serde_json::to_value(&status).unwrap()).unwrap();
        assert_eq!(back, status);

        let mut lock = ArrayEffectorLockMsg::default();
        lock.effector_lock[MAX_EFF_COUNT - 1] = EffectorLock::Locked;
        let back: ArrayEffectorLockMsg =
            serde_json::from_value(serde_json::to_value(&lock).unwrap()).unwrap();
        assert_eq!(back, lock);

        let mut burns = ReconfigBurnArrayInfoMsg::default();
        for (burn, status) in burns.burn_array.iter_mut().zip([
            ReconfigBurnStatus::Pending,
            ReconfigBurnStatus::Burned,
            ReconfigBurnStatus::Skipped,
        ]) {
            burn.status = status;
        }
        let back: ReconfigBurnArrayInfoMsg =
            serde_json::from_value(serde_json::to_value(&burns).unwrap()).unwrap();
        assert_eq!(back, burns);

        let camera = CameraConfigMsg {
            render_mode: CameraRenderMode::DepthMap,
            post_processing_on: VizSetting::Disabled,
            show_hud_elements_in_image: VizSetting::Enabled,
            ..CameraConfigMsg::default()
        };
        let back: CameraConfigMsg =
            serde_json::from_value(serde_json::to_value(&camera).unwrap()).unwrap();
        assert_eq!(back, camera);
    }

    #[test]
    fn long_arrays_reject_a_wrong_length() {
        let mut limb = serde_json::to_value(OpNavLimbMsg::default()).unwrap();
        limb["limb_points_px"] = json!(vec![[0.0, 0.0]; MAX_LIMB_PNTS - 1]);
        let error = serde_json::from_value::<OpNavLimbMsg>(limb)
            .expect_err("a short limb-point array was accepted")
            .to_string();
        assert!(
            error.contains(&format!("an array of length {MAX_LIMB_PNTS}")),
            "{error}"
        );

        let mut packets = serde_json::to_value(AccelerometerDataMsg::default()).unwrap();
        let extra = packets["accel_packets"][0].clone();
        packets["accel_packets"].as_array_mut().unwrap().push(extra);
        assert_eq!(
            packets["accel_packets"].as_array().unwrap().len(),
            MAX_ACC_BUF_PKT + 1
        );
        assert!(serde_json::from_value::<AccelerometerDataMsg>(packets).is_err());
    }

    fn thruster(index: usize) -> ThrusterConfigMsg {
        let offset = index as f64;
        ThrusterConfigMsg {
            position_thrust_b_m: Vector3::new(offset, 1.0, 2.0),
            t_hat_thrust_b: Vector3::x(),
            max_thrust_n: 1.0 + offset,
            swirl_torque_nm: 0.1 * offset,
        }
    }

    #[test]
    fn from_active_fills_the_leading_entries_and_zeroes_the_rest() {
        let on_times = ThrusterArrayOnTimeCommandMsg::from_active(&[0.5, 0.25]);
        assert_eq!(on_times.on_time_request_s[..2], [0.5, 0.25]);
        assert!(on_times.on_time_request_s[2..].iter().all(|&t| t == 0.0));

        let forces = ThrusterArrayForceCommandMsg::from_active(&[1.0; MAX_EFF_COUNT]);
        assert!(forces.thrust_force_n.iter().all(|&f| f == 1.0));

        let readings = CssArraySensorMsg::from_active(&[0.1, 0.2, 0.3]);
        assert_eq!(readings.time_tag_s, 0.0);
        assert_eq!(readings.cos_values[..3], [0.1, 0.2, 0.3]);
        assert!(readings.cos_values[3..].iter().all(|&c| c == 0.0));

        let thrusters: Vec<_> = (0..3).map(thruster).collect();
        let config = ThrusterArrayConfigMsg::from_active(&thrusters);
        assert_eq!(config.num_thrusters, 3);
        assert_eq!(config.active_thrusters(), thrusters.as_slice());
        assert_eq!(config.thrusters[3], ThrusterConfigMsg::default());

        let units = [
            CssUnitConfigMsg {
                n_hat_b: Vector3::x(),
                c_bias: 1.0,
            },
            CssUnitConfigMsg {
                n_hat_b: Vector3::y(),
                c_bias: 1.1,
            },
        ];
        let css = CssConfigMsg::from_active(&units);
        assert_eq!(css.num_css, 2);
        assert_eq!(css.active_units(), units.as_slice());
        assert_eq!(css.css_units[2], CssUnitConfigMsg::default());

        let wheels = [RwConfigElementMsg {
            gs_hat_b: Vector3::z(),
            spin_axis_inertia_kg_m2: 0.1,
            max_motor_torque_nm: 0.2,
        }];
        let constellation = RwConstellationMsg::from_active(&wheels);
        assert_eq!(constellation.num_rw, 1);
        assert_eq!(constellation.active_wheels(), wheels.as_slice());
    }

    #[test]
    fn active_entries_never_read_past_the_array() {
        let config = ThrusterArrayConfigMsg {
            num_thrusters: MAX_EFF_COUNT + 5,
            ..ThrusterArrayConfigMsg::default()
        };
        assert_eq!(config.active_thrusters().len(), MAX_EFF_COUNT);

        let css = CssConfigMsg {
            num_css: MAX_NUM_CSS_SENSORS + 1,
            ..CssConfigMsg::default()
        };
        assert_eq!(css.active_units().len(), MAX_NUM_CSS_SENSORS);

        let limb = OpNavLimbMsg {
            num_limb_points: 2,
            ..OpNavLimbMsg::default()
        };
        assert_eq!(limb.active_limb_points(), [Vector2::zeros(); 2]);
    }

    #[test]
    #[should_panic(expected = "at most 32 CSS readings are supported")]
    fn from_active_refuses_more_entries_than_upstream_holds() {
        let _ = CssArraySensorMsg::from_active(&[0.0; MAX_NUM_CSS_SENSORS + 1]);
    }

    /// Upstream `EP2C`, written out independently of nalgebra.
    fn ep2c(q: [f64; 4]) -> Matrix3<f64> {
        let [q0, q1, q2, q3] = q;
        Matrix3::new(
            q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3,
            2.0 * (q1 * q2 + q0 * q3),
            2.0 * (q1 * q3 - q0 * q2),
            2.0 * (q1 * q2 - q0 * q3),
            q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3,
            2.0 * (q2 * q3 + q0 * q1),
            2.0 * (q1 * q3 + q0 * q2),
            2.0 * (q2 * q3 - q0 * q1),
            q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3,
        )
    }

    /// Upstream `MRP2C`, written out independently of nalgebra.
    fn mrp2c(sigma: Vector3<f64>) -> Matrix3<f64> {
        let s2 = sigma.norm_squared();
        let tilde = sigma.cross_matrix();
        Matrix3::identity()
            + (8.0 * tilde * tilde - 4.0 * (1.0 - s2) * tilde) / ((1.0 + s2) * (1.0 + s2))
    }

    #[test]
    fn star_tracker_quaternion_matches_upstream_q_inrtl2case() {
        let norm = (0.5_f64.powi(2) + 0.1_f64.powi(2) + 0.2_f64.powi(2) + 0.3_f64.powi(2)).sqrt();
        let beta_cn = [0.5 / norm, 0.1 / norm, -0.2 / norm, 0.3 / norm];
        let message = StarTrackerMsg::from_q_inrtl2case(42, beta_cn);

        assert_eq!(message.time_tag_nanos, 42);
        assert!((message.dcm_cn() - ep2c(beta_cn)).abs().max() < 1e-15);
        let back = message.q_inrtl2case();
        for (actual, expected) in back.iter().zip(beta_cn) {
            assert!((actual - expected).abs() < 1e-15, "{back:?} vs {beta_cn:?}");
        }

        // The same attitude stored with the opposite sign reads back with β0 >= 0.
        let flipped = StarTrackerMsg {
            attitude_inertial_to_sensor: UnitQuaternion::new_unchecked(
                -message.attitude_inertial_to_sensor.into_inner(),
            ),
            ..message.clone()
        };
        assert!(flipped.q_inrtl2case()[0] > 0.0);
        assert!((flipped.dcm_cn() - message.dcm_cn()).abs().max() < 1e-15);
    }

    /// The Rust star tracker composes `body_to_sensor * inertial_to_body`; the
    /// documented mapping holds only if that product's matrix is `[CB][BN]`.
    #[test]
    fn star_tracker_attitude_is_the_inertial_to_case_dcm() {
        let state = SpacecraftStateMsg {
            sigma_bn: Vector3::new(0.1, -0.2, 0.3),
            ..SpacecraftStateMsg::default()
        };
        let dcm_bn = mrp2c(state.sigma_bn);
        assert!(
            (state.inertial_to_body().to_rotation_matrix().into_inner() - dcm_bn)
                .abs()
                .max()
                < 1e-15
        );

        let body_to_sensor = UnitQuaternion::from_quaternion(Quaternion::new(0.9, 0.1, 0.3, -0.2));
        let dcm_cb = body_to_sensor.to_rotation_matrix().into_inner();
        let message = StarTrackerMsg {
            time_tag_nanos: 0,
            attitude_inertial_to_sensor: body_to_sensor * state.inertial_to_body(),
        };
        assert!((message.dcm_cn() - dcm_cb * dcm_bn).abs().max() < 1e-15);
        assert!((ep2c(message.q_inrtl2case()) - dcm_cb * dcm_bn).abs().max() < 1e-15);
    }

    #[test]
    fn a_reduced_covariance_packs_into_the_front_of_the_flat_array() {
        let small = SMatrix::<f64, 3, 3>::from_fn(|row, column| (10 * row + column) as f64 + 1.0);
        let mut message = SunlineFilterMsg::default();
        message.set_packed_covariance(&small);

        // Upstream `memmove`s the nine row-major entries into flat slots 0..9
        // of the 6x6 row-major array.
        let mut flat = [0.0; SKF_N_STATES * SKF_N_STATES];
        for (index, value) in flat.iter_mut().enumerate() {
            *value = message.covar[(index / SKF_N_STATES, index % SKF_N_STATES)];
        }
        assert_eq!(
            flat[..9],
            [1.0, 2.0, 3.0, 11.0, 12.0, 13.0, 21.0, 22.0, 23.0]
        );
        assert!(flat[9..].iter().all(|&value| value == 0.0));
        assert_eq!(message.packed_covariance::<3>(), small);

        let full = SMatrix::<f64, SKF_N_STATES, SKF_N_STATES>::from_fn(|row, column| {
            (row * SKF_N_STATES + column) as f64
        });
        message.set_packed_covariance(&full);
        assert_eq!(message.covar, full);
        assert_eq!(message.packed_covariance::<SKF_N_STATES>(), full);
    }

    #[test]
    #[should_panic(expected = "does not fit")]
    fn a_covariance_larger_than_the_payload_is_refused() {
        SunlineFilterMsg::default().set_packed_covariance(&SMatrix::<f64, 7, 7>::zeros());
    }
}
