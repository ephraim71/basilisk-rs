use serde_json::json;

use crate::overrides::Mode;

use super::{Input, Output};
use crate::messages::PowerStorageStatusMsg;
use crate::messages::SpacecraftStateMsg;
use crate::messages::{ArrayMotorTorqueMsg, MAX_EFF_COUNT};

#[test]
fn replace_override_substitutes_the_complete_message() {
    let output = Output::new(PowerStorageStatusMsg::default());

    output
        .set_override(
            Mode::Replace,
            json!({
                "storage_level_j": 4.0,
                "storage_capacity_j": 5.0,
                "current_net_power_w": 6.0,
            }),
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

    output.set_override(Mode::Default, json!(null)).unwrap();
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
        .set_override(Mode::Patch, json!({ "storage_level_j": 99.0 }))
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
        .set_override(Mode::Patch, json!({ "storage_level_j": 7.0 }))
        .unwrap();
    input
        .set_override(Mode::Patch, json!({ "storage_capacity_j": 3.0 }))
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
        .set_override(Mode::Patch, json!({ "storage_level_j": 99.0 }))
        .unwrap();

    input.clear_override();

    assert_eq!(input.read().storage_level_j, 5.0);
}

#[test]
fn input_rejects_a_value_of_the_wrong_shape() {
    let producer = Output::new(PowerStorageStatusMsg::default());
    let input = connected_input(&producer);

    let result = input.set_override(Mode::Patch, json!({ "storage_level_j": "not a number" }));

    assert!(result.is_err(), "expected a type error, got {result:?}");
    assert!(!input.is_overridden());
}

#[test]
fn preview_override_reports_the_result_without_installing_it() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output.write(status(5.0));

    let previewed = output
        .preview_override(Mode::Patch, json!({ "storage_level_j": 99.0 }))
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
        .set_override(Mode::Patch, json!({ "storage_level_j": 1.0 }))
        .unwrap();

    let previewed = output
        .preview_override(Mode::Patch, json!({ "storage_capacity_j": 2.0 }))
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
        .preview_override(Mode::Patch, json!({ "storage_level_j": 99.0 }))
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
        .set_override(Mode::Patch, json!({ "current_net_power_w": 9.0 }))
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
        .set_override(Mode::Patch, json!({ "storage_level_j": 1.0 }))
        .unwrap();
    output
        .set_override(Mode::Patch, json!({ "current_net_power_w": 3.0 }))
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
        .set_override(Mode::Patch, json!({ "current_net_power_w": 9.0 }))
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

    let result = output.set_override(Mode::Patch, json!({ "storage_level_j": "not a number" }));

    assert!(result.is_err(), "expected a type error, got {result:?}");
}

#[test]
fn a_rejected_override_leaves_the_previous_one_working() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output
        .set_override(Mode::Patch, json!({ "storage_level_j": 1.0 }))
        .unwrap();

    let _ = output.set_override(Mode::Patch, json!({ "storage_capacity_j": "not a number" }));
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
        .set_override(Mode::Patch, json!({ "storage_level_j": 1.0 }))
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
        .set_override(Mode::Patch, json!({ "storage_level_j": 1.0 }))
        .unwrap();
    let second = output
        .set_override(Mode::Patch, json!({ "storage_capacity_j": 2.0 }))
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
        .set_override(Mode::Patch, json!({ "storage_level_j": 1.0 }))
        .unwrap();

    let state = output.installed_overrides();
    assert_eq!(state.len(), 1);
    assert_eq!(state[0].mode, Mode::Patch);
    assert_eq!(state[0].value, json!({ "storage_level_j": 1.0 }));
}

/// Each patch is its own layer, so each stays removable on its own.
#[test]
fn a_patch_layers_over_the_one_before_it_rather_than_merging() {
    let output = Output::new(PowerStorageStatusMsg::default());
    let first = output
        .set_override(Mode::Patch, json!({ "storage_level_j": 1.0 }))
        .unwrap();
    output
        .set_override(Mode::Patch, json!({ "storage_capacity_j": 2.0 }))
        .unwrap();

    assert_eq!(output.installed_overrides().len(), 2);
    assert_eq!(output.read().storage_level_j, 1.0);
    assert_eq!(output.read().storage_capacity_j, 2.0);

    // Removing the inner layer leaves the outer one working.
    assert!(output.clear_override_by_id(first));
    assert_eq!(output.read().storage_level_j, 0.0);
    assert_eq!(output.read().storage_capacity_j, 2.0);
}

/// An absolute rule hides the layers beneath it rather than deleting them,
/// so removing it gives the earlier fault back. A timed `replace` must not
/// permanently destroy a fault that was already running.
#[test]
fn removing_a_replace_uncovers_the_patch_it_was_hiding() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output
        .set_override(Mode::Patch, json!({ "storage_level_j": 1.0 }))
        .unwrap();
    let replace = output
        .set_override(
            Mode::Replace,
            json!({
                "storage_level_j": 5.0,
                "storage_capacity_j": 0.0,
                "current_net_power_w": 0.0,
            }),
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

/// A refused rule must leave the stack exactly as it found it. An absolute
/// rule is the dangerous case: it displaces every layer beneath it, so a
/// rollback that only removes the candidate loses them.
#[test]
fn a_rejected_replace_leaves_the_layers_it_would_have_displaced() {
    let output = Output::new(PowerStorageStatusMsg::default());
    output
        .set_override(Mode::Patch, json!({ "storage_level_j": 1.0 }))
        .unwrap();

    let refused = output.set_override(Mode::Replace, json!({ "storage_level_j": "not a number" }));

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

    output.set_override(Mode::Freeze, json!({})).unwrap();
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
// pointerReplace: addressing one element of a vector
// -----------------------------------------------------------------------

fn spinning(rate: [f64; 3]) -> SpacecraftStateMsg {
    SpacecraftStateMsg {
        omega_radps: nalgebra::Vector3::new(rate[0], rate[1], rate[2]),
        ..SpacecraftStateMsg::default()
    }
}

fn replace_at(path: &str, value: f64) -> serde_json::Value {
    json!([{ "op": "replace", "path": path, "value": value }])
}

#[test]
fn a_pointer_replace_changes_one_element_and_leaves_its_siblings_upstream() {
    let output = Output::new(SpacecraftStateMsg::default());
    output.write(spinning([1.0, 2.0, 3.0]));

    output
        .set_override(Mode::PointerReplace, replace_at("/omega_radps/1", 9.0))
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

/// `fold` starts at the outermost **absolute** rule, so a mode that is
/// relative but not recognised as one would mask every layer beneath it.
/// `Patch` and `PointerReplace` must both count as relative.
#[test]
fn a_pointer_replace_layer_does_not_mask_the_patch_beneath_it() {
    let output = Output::new(SpacecraftStateMsg::default());
    output.write(spinning([1.0, 2.0, 3.0]));

    output
        .set_override(Mode::Patch, json!({ "sigma_bn": [7.0, 0.0, 0.0] }))
        .unwrap();
    output
        .set_override(Mode::PointerReplace, replace_at("/omega_radps/0", 9.0))
        .unwrap();

    let effective = output.read();
    assert_eq!(
        effective.omega_radps.x, 9.0,
        "the pointer replace did not apply"
    );
    assert_eq!(
        effective.sigma_bn.x, 7.0,
        "the pointer replace masked the patch below it, so it was folded as an absolute rule"
    );
}

#[test]
fn a_replace_masks_a_pointer_replace_and_gives_it_back_when_it_is_removed() {
    let output = Output::new(SpacecraftStateMsg::default());
    output.write(spinning([1.0, 2.0, 3.0]));

    output
        .set_override(Mode::PointerReplace, replace_at("/omega_radps/1", 9.0))
        .unwrap();
    let absolute = output
        .set_override(
            Mode::Replace,
            serde_json::to_value(spinning([0.0, 0.0, 0.0])).unwrap(),
        )
        .unwrap();
    assert_eq!(output.read().omega_radps.y, 0.0, "the replace did not take");

    assert!(output.clear_override_by_id(absolute));
    assert_eq!(
        output.read().omega_radps.y,
        9.0,
        "removing the replace lost the pointer replace it was covering"
    );
}

#[test]
fn pointer_replace_refuses_every_operation_but_replace() {
    let output = Output::new(SpacecraftStateMsg::default());

    for operation in ["add", "remove", "move", "copy", "test"] {
        let error = output
            .set_override(
                Mode::PointerReplace,
                json!([{ "op": operation, "path": "/omega_radps/0", "value": 1.0 }]),
            )
            .expect_err("{operation} was accepted");
        let error = error.to_string();
        assert!(
            error.contains(&format!("'{operation}'")) && error.contains("RFC 6902"),
            "{operation} was refused without saying which operation, or against what: {error}"
        );
    }
}

#[test]
fn a_pointer_replace_path_that_does_not_resolve_is_refused_not_ignored() {
    let output = Output::new(SpacecraftStateMsg::default());
    output.write(spinning([1.0, 2.0, 3.0]));

    let error = output
        .set_override(Mode::PointerReplace, replace_at("/omega_radps/7", 9.0))
        .expect_err("an index past the end of a fixed vector was accepted");
    assert!(error.to_string().contains("does not resolve"), "{error}");

    // And nothing was installed by the attempt.
    assert!(!output.is_overridden());
    assert_eq!(output.read().omega_radps.x, 1.0);
}

#[test]
fn a_pointer_replace_value_that_is_not_an_operation_list_is_refused() {
    let output = Output::new(SpacecraftStateMsg::default());
    let error = output
        .set_override(
            Mode::PointerReplace,
            json!({ "omega_radps": [1.0, 2.0, 3.0] }),
        )
        .expect_err("a merge document was accepted as a patch document");
    assert!(error.to_string().contains("must be an array"), "{error}");
}
