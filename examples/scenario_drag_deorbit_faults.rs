//! Fault injection over the drag-deorbit scenario.
//!
//! `scenario_drag_deorbit` flown five times: once nominally, then four times
//! with one component made to look broken while every other module runs exactly
//! the code it runs nominally. Nothing opts into a fault mode and no module is
//! told a fault exists. The atmosphere model, the drag effector and the
//! integrator are untouched; what changes is a rule laid over a port between
//! them, installed at a moment mid-run and, in two cases, withdrawn again.
//!
//! | case | what is made to look broken | what it is here to show |
//! |---|---|---|
//! | `nominal` | nothing | the reference deorbit the others are read against |
//! | `atmosphere_frozen` | the atmosphere's published message stops changing | a frozen producer, seen by every consumer at once |
//! | `drag_blind` | the drag effector's own view reads vacuum | the same fault confined to one consumer — the message itself stays correct |
//! | `dropout_and_recovery` | density drops out for 15 minutes, under a second fault that never lifts | withdrawing one rule by id while another on the same port stays in force |
//! | `triggered_below_200_km` | the drag effector goes blind the tick the vehicle crosses 200 km | a fault dated by a condition rather than by the clock |
//!
//! The faults themselves are an [`overrides::Case`](basilisk_rs::overrides::Case):
//! a named list of [`Fault`](basilisk_rs::overrides::Fault)s handed to a
//! [`FaultSchedule`](basilisk_rs::overrides::FaultSchedule), which is scheduled
//! into the simulation like any other module and installs each one on the tick
//! it is dated for. The last case has no date to give: its fault is posted
//! through [`FaultSchedule::sender`](basilisk_rs::overrides::FaultSchedule::sender)
//! by the run loop the moment the altitude crosses, and would be posted the same
//! way by a control socket or an operator's console.
//!
//! Each case writes `examples/output/overrides/<case>.csv`, and three of its
//! columns say what the fault did: `neutralDensity_truth_kg_per_m3` is what the
//! atmosphere model computed, `neutralDensity_published_kg_per_m3` is what its
//! consumers read, and `neutralDensity_drag_kg_per_m3` is what the drag
//! effector consumed. All three agree on the nominal run and diverge exactly
//! where a rule is installed — which is the point: a case is diagnosed by
//! comparing them, not by trusting any one of them.
//!
//! More targets are registered than the cases use. The catalogue printed at
//! startup is the list, and naming one in a `Fault` below is the whole change
//! needed to try it.

#[path = "support/common.rs"]
mod common;

use std::path::PathBuf;

use anise::constants::frames::{EARTH_J2000, IAU_EARTH_FRAME};
use basilisk_rs::dynamics::drag_dynamic_effector::{
    DragDynamicEffector, DragDynamicEffectorConfig,
};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::environment::atmosphere::msis_atmosphere::{MsisAtmosphere, MsisAtmosphereConfig};
use basilisk_rs::messages::{
    AtmosphereMsg, Input, Output, PlanetOrientation, PlanetStateMsg, SpacecraftDiagnosticsMsg,
    SpacecraftStateMsg,
};
use basilisk_rs::overrides::{
    Case, Fault, FaultSchedule, Overridable, Registry, Request, TargetKind,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::{CsvFormat, CsvRecorder, CsvRecorderConfig, CsvSourceConfig};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use common::{
    EARTH_EQUATORIAL_RADIUS_M, MU_EARTH_M3PS2, elem2rv, output_path, seconds, vector_columns,
};
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};
use serde_json::{Value, json};

const TASK_PERIOD_NANOS: u64 = seconds(15);
const SAMPLE_PERIOD_NANOS: u64 = seconds(45);
const TERMINAL_ALTITUDE_M: f64 = 100_000.0;
const INITIAL_ALTITUDE_M: f64 = 250_000.0;
const PROJECTED_AREA_M2: f64 = 10.0;
const DRAG_COEFFICIENT: f64 = 2.2;

/// The nominal run reaches 100 km at about 2190 s. A case whose fault removes
/// the drag that gets it there would otherwise orbit indefinitely, so every case
/// stops here whether or not it deorbited, and the cases stay comparable.
const DURATION_CAP_NANOS: u64 = seconds(7_200);

/// Every dated fault lands here: late enough that the run is established and the
/// atmosphere is publishing a sensible density, early enough to leave most of
/// the nominal descent for it to affect.
const FAULT_ONSET_NANOS: u64 = seconds(600);

/// How long the dropout lasts before it is withdrawn.
const DROPOUT_LIFETIME_NANOS: u64 = seconds(900);

/// Where the condition-triggered case fires. Not a time: no one writing this
/// case knows when the vehicle gets here, which is the reason that fault is
/// posted from the run loop rather than dated in the list with the others.
const TRIGGER_ALTITUDE_M: f64 = 200_000.0;

/// The temperature the stuck-reading fault holds. Nothing in this scenario
/// consumes temperature, which is why it is the field used to show a rule
/// surviving the removal of the one installed beside it: it proves the layering
/// without also perturbing the trajectory being measured.
const STUCK_TEMP_K: f64 = 273.15;

const ATMOSPHERE_DENSITY: &str = "atmosphere.output_atmosphere_msg";
const ATMOSPHERE_STATE_IN: &str = "atmosphere.input_state_msg";
const DRAG_DENSITY_IN: &str = "drag.input_atmosphere_msg";
const SPACECRAFT_STATE: &str = "spacecraft.state_out";

/// Models the separate upstream drag task: it reads the atmosphere after the
/// spacecraft task and the cached density is consumed on the next integration
/// tick. This one-task delay is observable in the recorded trajectory.
#[derive(Clone, Debug, Default)]
struct AtmosphereLatch {
    atmosphere_in_msg: Input<AtmosphereMsg>,
    atmosphere_out_msg: Output<AtmosphereMsg>,
}

impl Module for AtmosphereLatch {
    fn init(&mut self) {
        self.atmosphere_out_msg.write(AtmosphereMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.atmosphere_out_msg.write(self.atmosphere_in_msg.read());
    }
}

/// Written once for the type, so both of the latch's ports reach an operator
/// from the single `register_module` call at the assembly site below — and a
/// port added to the latch later would reach one without that site changing.
impl Overridable for AtmosphereLatch {
    fn register_targets(&self, registry: &Registry, prefix: &str) -> anyhow::Result<()> {
        registry.register(
            format!("{prefix}.atmosphere_in_msg"),
            &self.atmosphere_in_msg,
            TargetKind::Input,
        )?;
        registry.register(
            format!("{prefix}.atmosphere_out_msg"),
            &self.atmosphere_out_msg,
            TargetKind::Output,
        )
    }
}

/// Republishes the densities a fault can make disagree, so that one CSV carries
/// all of them.
///
/// A recorder source reads a port the way any other consumer does, so it can
/// only ever log the effective value — the one with the rules applied. The truth
/// column has to come from somewhere else, and `Output::read_upstream` is it:
/// what the atmosphere module actually computed, whatever is laid over it.
///
/// Scheduled between the atmosphere and the spacecraft, so `consumed_by_drag` is
/// sampled before the latch republishes. That is the density this tick's
/// integration uses, rather than the one the next tick will.
#[derive(Clone, Debug, Default)]
struct DensityWitness {
    /// The atmosphere's own output port, aliased.
    published: Output<AtmosphereMsg>,
    /// The drag effector's own input port, aliased.
    consumed_by_drag: Input<AtmosphereMsg>,
    truth_out: Output<AtmosphereMsg>,
    drag_view_out: Output<AtmosphereMsg>,
}

impl Module for DensityWitness {
    fn init(&mut self) {
        self.truth_out.write(AtmosphereMsg::default());
        self.drag_view_out.write(AtmosphereMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.truth_out.write(self.published.read_upstream());
        self.drag_view_out.write(self.consumed_by_drag.read());
    }
}

// ---------------------------------------------------------------------------
// The cases
// ---------------------------------------------------------------------------

/// A [`Case`], plus the one thing a case cannot carry: a fault with no date.
struct Scenario {
    case: Case,
    trigger: Option<Trigger>,
}

/// A fault posted the first time the true altitude falls below a threshold.
///
/// A `Case` holds faults dated in simulation time, and this one has no time to
/// give: it belongs to a state the vehicle reaches when it reaches it. The run
/// loop watches for that and posts the fault through the schedule's sender,
/// which is the same road a control socket or a console would take.
struct Trigger {
    below_altitude_m: f64,
    fault: Fault,
}

fn scenarios() -> Vec<Scenario> {
    vec![
        Scenario {
            case: Case::new("nominal")
                .with_description("no faults: the reference every other case is read against"),
            trigger: None,
        },
        Scenario {
            case: Case::new("atmosphere_frozen")
                .with_description(
                    "the atmosphere model keeps running; its published message stops changing",
                )
                .with(
                    // An empty selection is the whole message: every field holds
                    // the value it had at the moment the rule was installed. A
                    // freeze reads that from the live port, so the case does not
                    // have to know what the density was at 600 s to hold it
                    // there.
                    Fault::new(ATMOSPHERE_DENSITY, freeze_whole_message())
                        .at(FAULT_ONSET_NANOS)
                        .labelled("frozen atmosphere output"),
                ),
            trigger: None,
        },
        Scenario {
            case: Case::new("drag_blind")
                .with_description(
                    "the message goes on being published correctly; one consumer reads vacuum",
                )
                .with(
                    // The drag effector's own input rather than the atmosphere's
                    // output, so no other consumer is affected — the recorder and
                    // the witness go on logging the true density while the
                    // vehicle stops feeling it.
                    Fault::new(DRAG_DENSITY_IN, vacuum())
                        .at(FAULT_ONSET_NANOS)
                        .labelled("drag reads vacuum"),
                ),
            trigger: None,
        },
        Scenario {
            case: Case::new("dropout_and_recovery")
                .with_description(
                    "density drops out for 15 minutes under a stuck reading that never lifts",
                )
                .with(
                    Fault::new(ATMOSPHERE_DENSITY, vacuum())
                        .at(FAULT_ONSET_NANOS)
                        .lasting(DROPOUT_LIFETIME_NANOS)
                        .labelled("density dropout"),
                )
                .with(
                    // The same target, a different field, its own layer. Every
                    // rule is relative: this one touches the path it names and
                    // leaves density to the rule beneath it, so the two coexist
                    // and the dropout's expiry cannot take this one with it.
                    Fault::new(ATMOSPHERE_DENSITY, stuck_temperature())
                        .at(FAULT_ONSET_NANOS)
                        .labelled("stuck temperature"),
                ),
            trigger: None,
        },
        Scenario {
            case: Case::new("triggered_below_200_km").with_description(
                "dated by a condition rather than a clock: posted as the vehicle crosses 200 km",
            ),
            trigger: Some(Trigger {
                below_altitude_m: TRIGGER_ALTITUDE_M,
                fault: Fault::new(DRAG_DENSITY_IN, vacuum()).labelled("drag blinded on crossing"),
            }),
        },
    ]
}

fn freeze_whole_message() -> Request {
    Request::freeze(json!([])).expect("a whole-message freeze")
}

fn vacuum() -> Request {
    Request::replace(json!({ "/neutral_density_kgpm3": 0.0 })).expect("a one-field replace")
}

fn stuck_temperature() -> Request {
    Request::replace(json!({ "/local_temp_k": STUCK_TEMP_K })).expect("a one-field replace")
}

/// What a run is judged on afterwards.
struct Outcome {
    name: String,
    stopped_at_nanos: u64,
    final_altitude_m: f64,
    deorbited: bool,
    /// What consumers of the atmosphere message read at the last tick.
    published: Value,
    /// What the atmosphere model computed at the last tick, faults or not.
    truth: Value,
    /// The altitude a condition-dated fault was posted at, for a case that has
    /// one. What says the fault followed the vehicle's state rather than a
    /// clock.
    fired_at_altitude_m: Option<f64>,
    csv: PathBuf,
}

fn main() {
    let mut outcomes = Vec::new();
    for (index, scenario) in scenarios().into_iter().enumerate() {
        outcomes.push(run_case(scenario, index == 0));
    }
    report(&outcomes);
    check(&outcomes);
}

fn run_case(scenario: Scenario, describe: bool) -> Outcome {
    let name = scenario.case.name().to_string();
    println!("\n=== {name} ===");
    if let Some(description) = scenario.case.description() {
        println!("{description}\n");
    }

    let degrees = std::f64::consts::PI / 180.0;
    let semimajor_axis_m = EARTH_EQUATORIAL_RADIUS_M + INITIAL_ALTITUDE_M;
    let (initial_position_m, initial_velocity_mps) = elem2rv(
        MU_EARTH_M3PS2,
        semimajor_axis_m,
        0.0001,
        33.3 * degrees,
        48.2 * degrees,
        347.8 * degrees,
        85.3 * degrees,
    );

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        // The upstream example intentionally leaves the hub at its 1 kg default.
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: TASK_PERIOD_NANOS,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass(
                "earth",
                MU_EARTH_M3PS2,
                true,
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .expect("valid point-mass Earth"),
        )
        .expect("unique central gravity body");

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut atmosphere = MsisAtmosphere::new(MsisAtmosphereConfig {
        name: "MsisAtmo".to_string(),
        first_kernel_path: repo_root.join("assets/anise/pck11.pca"),
        additional_kernel_paths: vec![repo_root.join("assets/anise/earth_latest_high_prec.bpc")],
        inertial_frame: EARTH_J2000,
        fixed_frame: IAU_EARTH_FRAME,
        ap_array: [8.0; 7],
        f107_daily: 110.0,
        f107_average: 110.0,
    });
    let planet_state = Output::new(PlanetStateMsg {
        position_inertial_m: Vector3::zeros(),
        velocity_inertial_mps: Vector3::zeros(),
        orientation: Some(PlanetOrientation::identity()),
    });

    let mut drag = DragDynamicEffector::new(DragDynamicEffectorConfig {
        name: "DragEff".to_string(),
        projected_area_m2: PROJECTED_AREA_M2,
        drag_coeff: DRAG_COEFFICIENT,
        com_offset_m: Vector3::zeros(),
        planet_rotation_rate_radps: Vector3::zeros(),
    });
    let mut atmosphere_latch = AtmosphereLatch::default();
    let start_epoch = Epoch::from_gregorian_utc_at_midnight(2019, 1, 1);
    let mut simulation = Simulation::new(start_epoch, false);
    connect!(&simulation,
        &atmosphere_latch.atmosphere_out_msg => &mut drag.input_atmosphere_msg,
    );

    // Cloning a port aliases it rather than copying its value, so this handle,
    // the effector's own and the one the registry keeps are three names for one
    // message and one rule stack. That is what lets a fault installed through
    // the registry be felt by an effector that has already been moved into the
    // spacecraft and can no longer be reached from here.
    let drag_density_in = drag.input_atmosphere_msg.clone();
    spacecraft.add_dynamic_effector(drag);

    let mut witness = DensityWitness {
        published: atmosphere.output_atmosphere_msg.clone(),
        consumed_by_drag: drag_density_in.clone(),
        ..DensityWitness::default()
    };

    let state_handle = spacecraft.state_out.clone();
    let csv_path = output_path("overrides", &format!("{name}.csv"));
    let mut recorder = CsvRecorder::new(CsvRecorderConfig {
        topic: format!("dragDeorbitFaults_{name}"),
        output_path: csv_path.clone(),
    })
    .with_format(CsvFormat::Scenario);

    connect!(&simulation,
        &spacecraft.state_out => &mut atmosphere.input_state_msg,
        &planet_state => &mut atmosphere.input_planet_msg,
        &atmosphere.output_atmosphere_msg => &mut atmosphere_latch.atmosphere_in_msg,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(
                vector_columns("position_m", "r_BN_N_m")
                    .into_iter()
                    .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s")),
            ),
        ),
        &spacecraft.diagnostics_out => recorder.add_source::<SpacecraftDiagnosticsMsg>(
            CsvSourceConfig::columns(vector_columns(
                "drag_force_body_n",
                "forceExternal_B_N",
            )),
        ),
        &atmosphere.output_atmosphere_msg => recorder.add_source::<AtmosphereMsg>(
            CsvSourceConfig::columns([
                ("neutral_density_kgpm3", "neutralDensity_published_kg_per_m3"),
                ("local_temp_k", "localTemp_published_K"),
            ]),
        ),
        &witness.truth_out => recorder.add_source::<AtmosphereMsg>(
            CsvSourceConfig::columns([(
                "neutral_density_kgpm3",
                "neutralDensity_truth_kg_per_m3",
            )]),
        ),
        &witness.drag_view_out => recorder.add_source::<AtmosphereMsg>(
            CsvSourceConfig::columns([(
                "neutral_density_kgpm3",
                "neutralDensity_drag_kg_per_m3",
            )]),
        ),
    );

    // Registration comes after the wiring on purpose: an input that is not
    // connected yet reads the type default, and the registry refuses one rather
    // than let it describe a fault against a value no producer published.
    let registry = Registry::new();
    registry
        .register(
            ATMOSPHERE_DENSITY,
            &atmosphere.output_atmosphere_msg,
            TargetKind::Output,
        )
        .expect("the atmosphere output is registrable");
    registry
        .register(
            ATMOSPHERE_STATE_IN,
            &atmosphere.input_state_msg,
            TargetKind::Input,
        )
        .expect("the atmosphere state input is connected");
    registry
        .register(SPACECRAFT_STATE, &spacecraft.state_out, TargetKind::Output)
        .expect("the spacecraft state output is registrable");
    registry
        .register(DRAG_DENSITY_IN, &drag_density_in, TargetKind::Input)
        .expect("the drag atmosphere input is connected");
    // One call for a module that lists its own ports.
    registry
        .register_module("latch", &atmosphere_latch)
        .expect("the latch is wired");

    if describe {
        describe_targets(&registry);
    }

    let mut faults = FaultSchedule::new(&registry, scenario.case);
    // Every target name and field is checked here, before a tick has run. A
    // mistyped one otherwise surfaces ten minutes into a run, as a refusal in a
    // report that then reads exactly like a fault that had no effect.
    faults
        .preflight()
        .expect("every fault names a registered target and a field it has");
    // Taken before the simulation takes the schedule: from here on the run loop
    // reaches its own faults only through this.
    let sender = faults.sender();

    schedule! { simulation,
        // Above every module that reads a port it touches, so a fault dated for
        // this tick is in force for this tick's readers.
        "faults" => &mut faults, period=TASK_PERIOD_NANOS, priority=100;
        "atmosphere" => &mut atmosphere, period=TASK_PERIOD_NANOS, priority=30;
        "witness" => &mut witness, period=TASK_PERIOD_NANOS, priority=25;
        "spacecraft" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=20;
        "atmosphere_latch" => &mut atmosphere_latch, period=TASK_PERIOD_NANOS, priority=5;
        "recorder" => &mut recorder, period=SAMPLE_PERIOD_NANOS, priority=0;
    }

    // The harness watches what the integrator produced rather than what
    // consumers see, so a fault laid over the state message can neither end a
    // run early nor hold one open, and neither can one fire a trigger. A harness
    // that reads the overridden value is measuring the fault instead of its
    // effect.
    let true_altitude_m =
        || state_handle.read_upstream().position_m.norm() - EARTH_EQUATORIAL_RADIUS_M;
    let mut trigger = scenario.trigger;
    let mut fired_at_altitude_m = None;

    simulation.run_for(0);
    while simulation.current_sim_nanos() < DURATION_CAP_NANOS
        && true_altitude_m() >= TERMINAL_ALTITUDE_M
    {
        let altitude_m = true_altitude_m();
        if trigger
            .as_ref()
            .is_some_and(|trigger| altitude_m < trigger.below_altitude_m)
        {
            let fired = trigger.take().expect("the trigger fired");
            // The schedule belongs to the simulation now, so the fault goes to
            // it rather than through it: posted here, installed by the schedule
            // itself on the next tick.
            sender
                .send(fired.fault)
                .expect("the schedule is still running");
            fired_at_altitude_m = Some(altitude_m);
        }
        simulation.run_for(TASK_PERIOD_NANOS);
    }

    let stopped_at_nanos = simulation.current_sim_nanos();
    let final_altitude_m = true_altitude_m();

    // A run ends when the vehicle deorbits or the cap is reached, which can be
    // before every fault the case listed has come due. Cancelling what is left
    // records those, so the report below says a fault never happened rather than
    // leaving it out — the one way a case can inject less than it lists and look
    // as though it did. The other half of ending a campaign is deliberately not
    // called: withdrawing the rules in force would lift the very faults the
    // outcome is about to be read from.
    faults.cancel_pending();

    for event in faults.events() {
        println!("  {event}");
    }
    assert!(
        !faults.events().iter().any(|event| event.is_refusal()),
        "a fault in '{name}' was refused, so this run measured the wrong thing"
    );
    report_rules_in_force(&registry);

    Outcome {
        name,
        stopped_at_nanos,
        final_altitude_m,
        deorbited: final_altitude_m < TERMINAL_ALTITUDE_M,
        published: registry
            .effective(ATMOSPHERE_DENSITY)
            .expect("a registered target"),
        truth: registry
            .upstream(ATMOSPHERE_DENSITY)
            .expect("a registered target"),
        fired_at_altitude_m,
        csv: csv_path,
    }
}

/// What an operator sees before choosing anything: every target this simulation
/// registered, and the paths each one accepts.
///
/// The schema is read off the message type at runtime rather than written out
/// here, so a field added to `AtmosphereMsg` appears in this listing without
/// anything in this file changing.
fn describe_targets(registry: &Registry) {
    println!("registered override targets");
    for (name, target) in registry.targets() {
        let spec = target.spec().expect("a target describes itself");
        let paths: Vec<&str> = spec
            .fields
            .iter()
            .map(|field| field.path.as_str())
            .collect();
        println!("  {name:<34} {:<7} {}", spec.kind.as_str(), paths.join(" "));
    }

    // Those paths are checked, and the check is why a fault cannot quietly do
    // nothing: serde drops a field it does not recognise, so without it a
    // misspelled path would install, report success and change no value.
    let typo = Request::replace(json!({ "/neutral_density_kgpm2": 0.0 }))
        .expect("a well-formed replace document");
    let refusal = registry
        .validate(ATMOSPHERE_DENSITY, &typo)
        .expect_err("a misspelled field is refused");
    println!("\n  a misspelled path is refused rather than installed:");
    println!("    {refusal}\n");
}

/// What the case left installed, read back from the registry rather than from
/// the schedule: this is what any client interrogating the simulation would see.
fn report_rules_in_force(registry: &Registry) {
    for (name, target) in registry.targets() {
        let modes: Vec<&str> = target
            .rules()
            .iter()
            .map(|rule| rule.mode().as_str())
            .collect();
        if !modes.is_empty() {
            println!("  '{name}' still carries {}", modes.join(" then "));
        }
    }
}

fn report(outcomes: &[Outcome]) {
    println!("\n=== summary ===\n");
    println!(
        "  {:<24} {:>14} {:>12} {:>14} {:>14}",
        "case", "run ended", "altitude", "density read", "density true"
    );
    for outcome in outcomes {
        let ended = if outcome.deorbited {
            format!("{:.0} s deorbit", outcome.stopped_at_nanos as f64 * 1.0e-9)
        } else {
            format!("{:.0} s capped", outcome.stopped_at_nanos as f64 * 1.0e-9)
        };
        println!(
            "  {:<24} {ended:>14} {:>9.1} km {:>14.3e} {:>14.3e}",
            outcome.name,
            outcome.final_altitude_m / 1_000.0,
            density(&outcome.published),
            density(&outcome.truth),
        );
    }
    println!();
    for outcome in outcomes {
        println!("  wrote {}", outcome.csv.display());
    }
}

/// The claims the runs exist to support, as assertions rather than as prose:
/// each one fails loudly if the mechanism stops doing what its case says it
/// does.
fn check(outcomes: &[Outcome]) {
    let nominal = outcome(outcomes, "nominal");
    assert!(
        nominal.deorbited,
        "the reference case must still deorbit inside the cap"
    );
    assert_eq!(
        density(&nominal.published),
        density(&nominal.truth),
        "with nothing installed, consumers read exactly what the model computed"
    );

    let frozen = outcome(outcomes, "atmosphere_frozen");
    assert!(
        density(&frozen.published) < density(&frozen.truth),
        "the model went on computing a denser atmosphere than its consumers could read"
    );
    assert!(
        frozen.stopped_at_nanos > nominal.stopped_at_nanos,
        "a density frozen at its 600 s value cannot deorbit as fast as one that keeps rising"
    );

    let blind = outcome(outcomes, "drag_blind");
    assert!(
        !blind.deorbited,
        "an effector reading vacuum applies no drag, so the orbit holds"
    );
    assert_eq!(
        density(&blind.published),
        density(&blind.truth),
        "the fault is on one consumer's view, so the published message is untouched by it"
    );
    assert!(
        blind.final_altitude_m > frozen.final_altitude_m,
        "no drag at all decays more slowly than drag frozen at a 600 s value"
    );

    let recovered = outcome(outcomes, "dropout_and_recovery");
    assert!(
        recovered.deorbited,
        "withdrawing the dropout restores the descent"
    );
    assert!(
        recovered.stopped_at_nanos > nominal.stopped_at_nanos,
        "15 minutes of missing drag has to cost something"
    );
    assert_eq!(
        density(&recovered.published),
        density(&recovered.truth),
        "the dropout was withdrawn by id, so density reads through again"
    );
    assert_eq!(
        temperature(&recovered.published),
        STUCK_TEMP_K,
        "and the rule installed beside it was not withdrawn with it"
    );

    let triggered = outcome(outcomes, "triggered_below_200_km");
    let fired_at_m = triggered
        .fired_at_altitude_m
        .expect("the trigger fired during the run");
    assert!(
        fired_at_m < TRIGGER_ALTITUDE_M,
        "the fault was posted before the altitude it is dated by was reached"
    );
    assert!(
        // The condition is checked once a tick, so the crossing is caught within
        // one tick's descent of it rather than exactly on it.
        fired_at_m > TRIGGER_ALTITUDE_M - 5_000.0,
        "the crossing was noticed well after it happened: posted at {fired_at_m:.0} m"
    );
    assert!(
        triggered.stopped_at_nanos > nominal.stopped_at_nanos,
        "losing drag partway down cannot get the vehicle down sooner"
    );
    // It still deorbits, and that is the finding rather than a failure: by the
    // time it crosses, the vehicle is on a descending arc that no longer needs
    // the drag it just lost. A fault that stops a component is not the same as
    // one that stops the outcome, which is the distinction a run like this is
    // for.
}

fn outcome<'a>(outcomes: &'a [Outcome], name: &str) -> &'a Outcome {
    outcomes
        .iter()
        .find(|outcome| outcome.name == name)
        .unwrap_or_else(|| panic!("case '{name}' was run"))
}

fn density(message: &Value) -> f64 {
    field(message, "neutral_density_kgpm3")
}

fn temperature(message: &Value) -> f64 {
    field(message, "local_temp_k")
}

fn field(message: &Value, name: &str) -> f64 {
    message
        .get(name)
        .and_then(Value::as_f64)
        .unwrap_or_else(|| panic!("an atmosphere message carries '{name}': {message}"))
}
