use serde_json::json;

use crate::overrides::{Mode, Request};

use super::{Input, Output};
use crate::messages::PowerStorageStatusMsg;
use crate::messages::SpacecraftStateMsg;
use crate::messages::{ArrayMotorTorqueMsg, MAX_EFF_COUNT};

#[test]
fn a_replace_naming_every_field_substitutes_the_whole_message() {
    let output = Output::new(PowerStorageStatusMsg::default());

    output
        .set_override(
            Request::replace(json!({
                "storage_level_j": 4.0,
                "storage_capacity_j": 5.0,
                "current_net_power_w": 6.0,
            }))
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

#[test]
fn default_override_emits_default_until_cleared() {
    let output = Output::new(PowerStorageStatusMsg {
        storage_level_j: 1.0,
        storage_capacity_j: 2.0,
        current_net_power_w: 3.0,
    });

    output
        .set_override(Request::default(json!(null)).unwrap())
        .unwrap();
    output.write(PowerStorageStatusMsg {
        storage_level_j: 10.0,
        storage_capacity_j: 20.0,
        current_net_power_w: 30.0,
    });
    assert_eq!(output.read().storage_level_j, 0.0);
    assert_eq!(output.read().storage_capacity_j, 0.0);
    assert_eq!(output.read().current_net_power_w, 0.0);

    output.clear_override();
    assert_eq!(output.read().storage_level_j, 10.0);
    assert_eq!(output.read().storage_capacity_j, 20.0);
    assert_eq!(output.read().current_net_power_w, 30.0);
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
        .set_override(Request::replace(json!({ "storage_level_j": 99.0 })).unwrap())
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
        .set_override(Request::replace(json!({ "storage_level_j": 7.0 })).unwrap())
        .unwrap();
    input
        .set_override(Request::replace(json!({ "storage_capacity_j": 3.0 })).unwrap())
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
        .set_override(Request::replace(json!({ "storage_level_j": 99.0 })).unwrap())
        .unwrap();

    input.clear_override();

    assert_eq!(input.read().storage_level_j, 5.0);
}

#[test]
fn input_rejects_a_value_of_the_wrong_shape() {
    let producer = Output::new(PowerStorageStatusMsg::default());
    let input = connected_input(&producer);

    let result =
        input.set_override(Request::replace(json!({ "storage_level_j": "not a number" })).unwrap());

    assert!(result.is_err(), "expected a type error, got {result:?}");
    assert!(!input.is_overridden());
}

#[test]
fn preview_override_reports_the_result_without_installing_it() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output.write(status(5.0));

    let previewed = output
        .preview_override(Request::replace(json!({ "storage_level_j": 99.0 })).unwrap())
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
        .set_override(Request::replace(json!({ "storage_level_j": 1.0 })).unwrap())
        .unwrap();

    let previewed = output
        .preview_override(Request::replace(json!({ "storage_capacity_j": 2.0 })).unwrap())
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
        .preview_override(Request::replace(json!({ "storage_level_j": 99.0 })).unwrap())
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
        .set_override(Request::replace(json!({ "current_net_power_w": 9.0 })).unwrap())
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
        .set_override(Request::replace(json!({ "storage_level_j": 1.0 })).unwrap())
        .unwrap();
    output
        .set_override(Request::replace(json!({ "current_net_power_w": 3.0 })).unwrap())
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
        .set_override(Request::replace(json!({ "current_net_power_w": 9.0 })).unwrap())
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
        .set_override(Request::replace(json!({ "storage_level_j": "not a number" })).unwrap());

    assert!(result.is_err(), "expected a type error, got {result:?}");
}

#[test]
fn a_rejected_override_leaves_the_previous_one_working() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output
        .set_override(Request::replace(json!({ "storage_level_j": 1.0 })).unwrap())
        .unwrap();

    let _ = output
        .set_override(Request::replace(json!({ "storage_capacity_j": "not a number" })).unwrap());
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
        .set_override(Request::replace(json!({ "storage_level_j": 1.0 })).unwrap())
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
        .set_override(Request::replace(json!({ "storage_level_j": 1.0 })).unwrap())
        .unwrap();
    let second = output
        .set_override(Request::replace(json!({ "storage_capacity_j": 2.0 })).unwrap())
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
        .set_override(Request::replace(json!({ "storage_level_j": 1.0 })).unwrap())
        .unwrap();

    let state = output.installed_overrides();
    assert_eq!(state.len(), 1);
    assert_eq!(state[0].mode(), Mode::Replace);
    assert_eq!(
        serde_json::to_value(state[0].document()).unwrap(),
        json!({ "storage_level_j": 1.0 })
    );
}

/// Each patch is its own layer, so each stays removable on its own.
#[test]
fn a_patch_layers_over_the_one_before_it_rather_than_merging() {
    let output = Output::new(PowerStorageStatusMsg::default());
    let first = output
        .set_override(Request::replace(json!({ "storage_level_j": 1.0 })).unwrap())
        .unwrap();
    output
        .set_override(Request::replace(json!({ "storage_capacity_j": 2.0 })).unwrap())
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
        .set_override(Request::replace(json!({ "storage_level_j": 1.0 })).unwrap())
        .unwrap();
    let replace = output
        .set_override(
            Request::replace(json!({
                "storage_level_j": 5.0,
                "storage_capacity_j": 0.0,
                "current_net_power_w": 0.0,
            }))
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
        .set_override(Request::replace(json!({ "storage_level_j": 1.0 })).unwrap())
        .unwrap();

    let refused = output
        .set_override(Request::replace(json!({ "storage_level_j": "not a number" })).unwrap());

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
// operation documents: addressing one element of a vector
// -----------------------------------------------------------------------

fn spinning(rate: [f64; 3]) -> SpacecraftStateMsg {
    SpacecraftStateMsg {
        omega_radps: nalgebra::Vector3::new(rate[0], rate[1], rate[2]),
        ..SpacecraftStateMsg::default()
    }
}

fn replace_op(path: &str, value: f64) -> serde_json::Value {
    json!([{ "op": "replace", "path": path, "value": value }])
}

#[test]
fn an_operation_document_changes_one_element_and_leaves_its_siblings_upstream() {
    let output = Output::new(SpacecraftStateMsg::default());
    output.write(spinning([1.0, 2.0, 3.0]));

    output
        .set_override(Request::replace(replace_op("/omega_radps/1", 9.0)).unwrap())
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
        .set_override(Request::replace(json!({ "sigma_bn": [7.0, 0.0, 0.0] })).unwrap())
        .unwrap();
    output
        .set_override(Request::replace(replace_op("/omega_radps/0", 9.0)).unwrap())
        .unwrap();

    let effective = output.read();
    assert_eq!(
        effective.omega_radps.x, 9.0,
        "the pointer replace did not apply"
    );
    assert_eq!(
        effective.sigma_bn.x, 7.0,
        "the operation document masked the merge below it instead of composing"
    );
}

#[test]
fn a_later_rule_covers_an_earlier_one_and_gives_it_back_when_removed() {
    let output = Output::new(SpacecraftStateMsg::default());
    output.write(spinning([1.0, 2.0, 3.0]));

    output
        .set_override(Request::replace(replace_op("/omega_radps/1", 9.0)).unwrap())
        .unwrap();
    let covering = output
        .set_override(
            Request::replace(serde_json::to_value(spinning([0.0, 0.0, 0.0])).unwrap()).unwrap(),
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

/// Refused when the request is *parsed*, not when it is installed: an operation
/// this crate does not implement is not a rule that fails to apply, it is not a
/// rule at all, so it never reaches a port.
#[test]
fn a_document_using_any_operation_but_replace_is_refused() {
    for operation in ["add", "remove", "move", "copy", "test"] {
        let error =
            Request::replace(json!([{ "op": operation, "path": "/omega_radps/0", "value": 1.0 }]))
                .expect_err("{operation} was accepted")
                .to_string();
        assert!(
            error.contains(&format!("'{operation}'")) && error.contains("RFC 6902"),
            "{operation} was refused without saying which operation, or against what: {error}"
        );
    }
}

#[test]
fn a_pointer_that_does_not_resolve_is_refused_not_ignored() {
    let output = Output::new(SpacecraftStateMsg::default());
    output.write(spinning([1.0, 2.0, 3.0]));

    let error = output
        .set_override(Request::replace(replace_op("/omega_radps/7", 9.0)).unwrap())
        .expect_err("an index past the end of a fixed vector was accepted");
    assert!(error.to_string().contains("does not resolve"), "{error}");

    // And nothing was installed by the attempt.
    assert!(!output.is_overridden());
    assert_eq!(output.read().omega_radps.x, 1.0);
}

/// A replace takes either patch format, so both shapes are accepted and only a
/// payload that is neither is refused.
#[test]
fn a_replace_takes_either_patch_format_and_nothing_else() {
    let output = Output::new(SpacecraftStateMsg::default());

    output
        .set_override(Request::replace(json!({ "omega_radps": [1.0, 2.0, 3.0] })).unwrap())
        .expect("a merge document was refused");
    assert_eq!(output.read().omega_radps.z, 3.0);

    output
        .set_override(Request::replace(replace_op("/omega_radps/0", 9.0)).unwrap())
        .expect("an operation document was refused");
    assert_eq!(output.read().omega_radps.x, 9.0);

    for neither in [json!(42), json!("a string"), json!(null)] {
        let error = Request::replace(neither.clone())
            .expect_err("a payload that is neither shape was accepted")
            .to_string();
        assert!(error.contains("neither"), "{neither}: {error}");
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

/// A default naming fields resets those and no others — the mirror of a
/// per-field freeze, drawing from the type default instead of the live value.
#[test]
fn a_default_naming_fields_resets_those_and_leaves_the_rest() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output.write(PowerStorageStatusMsg {
        storage_level_j: 5.0,
        storage_capacity_j: 6.0,
        current_net_power_w: 7.0,
    });

    output
        .set_override(Request::default(json!(["/storage_capacity_j"])).unwrap())
        .unwrap();

    assert_eq!(
        output.read().storage_capacity_j,
        0.0,
        "the named field kept its value"
    );
    assert_eq!(
        output.read().storage_level_j,
        5.0,
        "an unnamed field was reset"
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

        let reset = Output::new(PowerStorageStatusMsg::default());
        reset.write(status(5.0));
        reset
            .set_override(Request::default(empty.clone()).unwrap())
            .unwrap();
        assert_eq!(
            reset.read().storage_level_j,
            0.0,
            "{empty} did not reset all"
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
        .set_override(Request::replace(json!({ "current_net_power_w": 3.0 })).unwrap())
        .unwrap();
    output
        .set_override(Request::default(json!(["/storage_capacity_j"])).unwrap())
        .unwrap();

    output.write(PowerStorageStatusMsg {
        storage_level_j: 99.0,
        storage_capacity_j: 88.0,
        current_net_power_w: 77.0,
    });

    assert_eq!(output.read().storage_level_j, 1.0, "the freeze was masked");
    assert_eq!(
        output.read().storage_capacity_j,
        0.0,
        "the default was masked"
    );
    assert_eq!(
        output.read().current_net_power_w,
        3.0,
        "the replace was masked"
    );
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
        .set_override(
            Request::replace(json!([{ "op": "replace", "path": "/inner/value", "value": 9.0 }]))
                .unwrap(),
        )
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
        .set_override(
            Request::replace(json!([{ "op": "replace", "path": "/inner/value", "value": 9.0 }]))
                .unwrap(),
        )
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
        .set_override(
            Request::replace(json!([{ "op": "replace", "path": "/inner/value", "value": 9.0 }]))
                .unwrap(),
        )
        .unwrap();

    output.write(Optional {
        keep: 77.0,
        inner: None,
    });

    output
        .preview_override(Request::replace(json!({ "keep": 8.0 })).unwrap())
        .expect("a preview was refused over a rule it has nothing to do with");
    output
        .set_override(Request::replace(json!({ "keep": 8.0 })).unwrap())
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
