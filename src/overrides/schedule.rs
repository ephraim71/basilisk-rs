//! When a rule goes in, and when it comes back out.
//!
//! [`Registry`] answers what can be injected into, and installs one rule when
//! asked. Every caller then wrote the same loop around it: a list of faults in
//! the order they come due, the ids of the ones installed so far, and a check on
//! each tick for which are due and which have expired. That loop is here, so a
//! campaign of faults is a value — a [`Case`] — rather than bookkeeping written
//! again per scenario.
//!
//! Three ways to drive one, and they compose:
//!
//! - **On the simulation's clock.** [`FaultSchedule`] is a [`Module`], so
//!   [`schedule!`](crate::schedule) takes it like any other and every fault
//!   lands on the tick it is dated for. Give it a priority above the modules
//!   that read the ports it touches, and a fault dated `t` is in force for the
//!   readers of tick `t`.
//! - **By hand.** [`FaultSchedule::advance`] takes the current time and returns
//!   what it did, for a run loop that steps the simulation itself.
//! - **While it runs, from somewhere else.** [`FaultSchedule::sender`] hands out
//!   a [`FaultSender`] any thread can post to: a control socket, an operator's
//!   console, or the run loop that has already handed the schedule to the
//!   simulation and cannot reach it any more. A posted fault is installed by the
//!   schedule itself, on the simulation's thread at a defined point in the tick,
//!   so a run stays reproducible. A [`Registry`] clone installs from another
//!   thread just as well and costs exactly that — see
//!   [`Output`](crate::messages::Output) on what a racing write is worth.
//!
//! Nothing here decides whether a fault is reasonable, and nothing re-checks a
//! payload: [`Registry::install`] validates, and a refusal is recorded as a
//! [`FaultEvent`] rather than raised, so one unusable fault cannot take a run
//! down with it. [`FaultSchedule::preflight`] is the same check moved to before
//! the run, which is where a mistyped target name is cheapest to find.

use std::collections::VecDeque;
use std::fmt;
use std::sync::mpsc::{Receiver, Sender, TryRecvError, channel};

use anyhow::{Context, Result, bail};

use crate::{Module, SimulationContext};

use super::{Registry, Request, RuleId};

/// One injection: what to install, where, when, and whether it ever lifts.
///
/// The time is simulation time in nanoseconds, and a fault with no time is due
/// the next time its schedule advances — which is what a fault triggered by a
/// condition rather than by a clock wants, since whoever noticed the condition
/// is already holding the moment it happened.
#[derive(Clone, Debug)]
pub struct Fault {
    label: String,
    target: String,
    request: Request,
    at_nanos: u64,
    lifetime_nanos: Option<u64>,
}

impl Fault {
    /// A fault on `target`, due the next time the schedule advances.
    ///
    /// The label defaults to the mode, since every report carries the target
    /// beside it; [`Self::labelled`] replaces it with what this particular fault
    /// is *for*, which is the thing neither the mode nor the target says.
    pub fn new(target: impl Into<String>, request: Request) -> Self {
        Self {
            label: request.mode().as_str().to_string(),
            target: target.into(),
            request,
            at_nanos: 0,
            lifetime_nanos: None,
        }
    }

    /// Dates the fault: it is installed on the first advance at or past
    /// `at_nanos`.
    ///
    /// A fault dated in the past is due immediately rather than skipped. A run
    /// that starts late, or a schedule advanced in coarse steps, would otherwise
    /// drop faults on the floor, and a fault campaign that quietly injects
    /// nothing is the one outcome worth ruling out.
    pub fn at(mut self, at_nanos: u64) -> Self {
        self.at_nanos = at_nanos;
        self
    }

    /// Withdraws the fault `lifetime_nanos` after it is installed.
    ///
    /// Measured from the install rather than from the date, so a fault posted
    /// mid-run gets the lifetime it was given rather than one counted from a
    /// moment it was never dated with.
    ///
    /// A lifetime that would run past the end of the clock stops there, so the
    /// fault holds for the rest of the run rather than expiring the instant it
    /// lands.
    pub fn lasting(mut self, lifetime_nanos: u64) -> Self {
        self.lifetime_nanos = Some(lifetime_nanos);
        self
    }

    /// What to call this fault in a report.
    pub fn labelled(mut self, label: impl Into<String>) -> Self {
        self.label = label.into();
        self
    }

    pub fn label(&self) -> &str {
        &self.label
    }

    pub fn target(&self) -> &str {
        &self.target
    }

    pub fn request(&self) -> &Request {
        &self.request
    }

    pub fn at_nanos(&self) -> u64 {
        self.at_nanos
    }

    /// How long the fault stays installed, or `None` if it never lifts.
    pub fn lifetime_nanos(&self) -> Option<u64> {
        self.lifetime_nanos
    }
}

/// A named set of faults: one run's worth of them.
///
/// Two runs of the same simulation usually differ in their faults and in
/// nothing else, so the faults are a value that can be named, listed beside
/// other cases and handed to a [`FaultSchedule`] — rather than a branch inside
/// the assembly code, which is the shape that ends with a simulation that knows
/// which experiment it is part of.
///
/// A case with no faults is the nominal run, and worth stating: it is what every
/// other case is read against.
///
/// Deliberately has no `Default`: a case is named where it is written, and an
/// unnamed one reports every event it raises against an empty string.
#[derive(Clone, Debug)]
pub struct Case {
    name: String,
    description: Option<String>,
    faults: Vec<Fault>,
}

impl Case {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            description: None,
            faults: Vec::new(),
        }
    }

    /// What the case is for. The fault list says what is injected; it does not
    /// say what the run is meant to demonstrate.
    pub fn with_description(mut self, description: impl Into<String>) -> Self {
        self.description = Some(description.into());
        self
    }

    pub fn with(mut self, fault: Fault) -> Self {
        self.faults.push(fault);
        self
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn description(&self) -> Option<&str> {
        self.description.as_deref()
    }

    pub fn faults(&self) -> &[Fault] {
        &self.faults
    }
}

/// What a schedule did, and when.
///
/// Recorded rather than printed, because a schedule driven as a [`Module`] has
/// no caller to hand a return value to — its whole report is
/// [`FaultSchedule::events`] afterwards.
#[derive(Clone, Debug)]
pub struct FaultEvent {
    /// The simulation time the schedule had reached, in nanoseconds.
    pub at_nanos: u64,
    pub label: String,
    pub target: String,
    pub kind: FaultEventKind,
}

impl FaultEvent {
    /// Whether the registry did not carry this change out.
    ///
    /// The one event kind a run should not ignore: a fault that was refused is a
    /// fault that is not being injected, and a case whose fault never landed
    /// reports "no effect" just as convincingly as one whose fault had none.
    pub fn is_refusal(&self) -> bool {
        matches!(self.kind, FaultEventKind::Refused(_))
    }
}

impl fmt::Display for FaultEvent {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let seconds = self.at_nanos as f64 * 1.0e-9;
        write!(formatter, "{seconds:.3} s  ")?;
        match &self.kind {
            FaultEventKind::Installed(id) => write!(
                formatter,
                "installed '{}' on '{}' as {id:?}",
                self.label, self.target
            ),
            FaultEventKind::Withdrawn(id) => write!(
                formatter,
                "withdrew '{}' from '{}', {id:?}",
                self.label, self.target
            ),
            FaultEventKind::Cancelled { due_at_nanos } => write!(
                formatter,
                "cancelled '{}' on '{}', which was due at {:.3} s",
                self.label,
                self.target,
                *due_at_nanos as f64 * 1.0e-9
            ),
            FaultEventKind::Refused(reason) => write!(
                formatter,
                "refused '{}' on '{}': {reason}",
                self.label, self.target
            ),
        }
    }
}

#[derive(Clone, Debug)]
pub enum FaultEventKind {
    Installed(RuleId),
    Withdrawn(RuleId),
    /// Dropped before it ever installed, by [`FaultSchedule::cancel_pending`],
    /// with the time it would have landed at.
    ///
    /// Recorded rather than passed over: a run something was cancelled out of
    /// looks exactly like a run whose fault had no effect, and only the report
    /// can tell the two apart.
    Cancelled {
        due_at_nanos: u64,
    },
    /// The registry did not carry the change out, and why: an unknown target, a
    /// field the message type does not have, a payload that cannot produce a
    /// well-formed message, or a rule that something else had already removed.
    Refused(String),
}

/// One rule this schedule installed, and when it comes out.
struct Installed {
    id: RuleId,
    target: String,
    label: String,
    /// `None` for a fault that stays in force until the run ends.
    clear_at_nanos: Option<u64>,
}

/// A [`Case`] against a running simulation: it installs each fault when the
/// clock reaches it and withdraws the ones that expire.
///
/// A withdrawal names the [`RuleId`] its install returned rather than clearing
/// the target, which is the difference between ending one fault and ending every
/// fault that happens to share a port. Two faults on one message — a dropout
/// that lifts and a stuck field that does not — is an ordinary campaign, and the
/// second must survive the first.
///
/// ```
/// use basilisk_rs::messages::Output;
/// use basilisk_rs::overrides::{Case, Fault, FaultSchedule, Registry, Request, TargetKind};
/// use serde_json::json;
///
/// #[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
/// struct GyroMsg {
///     rate_dps: f64,
/// }
///
/// let gyro = Output::new(GyroMsg { rate_dps: 1.0 });
/// let registry = Registry::new();
/// registry.register("gyro_0.output_msg", &gyro, TargetKind::Output)?;
///
/// let case = Case::new("stuck_rate").with(
///     Fault::new("gyro_0.output_msg", Request::freeze(json!([]))?)
///         .at(2_000_000_000)
///         .lasting(1_000_000_000)
///         .labelled("gyro stops updating"),
/// );
///
/// let mut faults = FaultSchedule::new(&registry, case);
/// faults.preflight()?;
///
/// // Nothing is due yet, so the port carries what the module published.
/// faults.advance(1_000_000_000);
/// gyro.write(GyroMsg { rate_dps: 2.0 });
/// assert_eq!(gyro.read().rate_dps, 2.0);
///
/// // Due: the value at the moment of the freeze is what readers now see, and
/// // what the module goes on publishing is visible only upstream of it.
/// faults.advance(2_000_000_000);
/// gyro.write(GyroMsg { rate_dps: 3.0 });
/// assert_eq!(gyro.read().rate_dps, 2.0);
/// assert_eq!(gyro.read_upstream().rate_dps, 3.0);
///
/// // Expired: the fault lifts itself, and the module's own value reappears.
/// faults.advance(3_000_000_000);
/// assert_eq!(gyro.read().rate_dps, 3.0);
/// assert!(!faults.events().iter().any(|event| event.is_refusal()));
/// # Ok::<(), anyhow::Error>(())
/// ```
pub struct FaultSchedule {
    registry: Registry,
    case: String,
    /// Ordered by the time each comes due, so the front is always the next one.
    pending: VecDeque<Fault>,
    installed: Vec<Installed>,
    /// Faults posted by another thread, drained on the next advance.
    incoming: Receiver<Fault>,
    /// Kept so [`Self::sender`] can hand out clones, and so the channel stays
    /// open when a caller drops theirs.
    outgoing: Sender<Fault>,
    events: Vec<FaultEvent>,
    /// The last time this was advanced to. What a fault posted mid-run is dated
    /// against, and what an event records.
    now_nanos: u64,
}

/// Written out rather than derived: a [`Registry`] holds its targets
/// type-erased and cannot print them, and what a reader wants from a schedule is
/// its progress through the case rather than its faults spelled out.
impl fmt::Debug for FaultSchedule {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("FaultSchedule")
            .field("case", &self.case)
            .field("now_nanos", &self.now_nanos)
            .field("pending", &self.pending.len())
            .field("in_force", &self.installed.len())
            .field("events", &self.events.len())
            .finish_non_exhaustive()
    }
}

impl FaultSchedule {
    pub fn new(registry: &Registry, case: Case) -> Self {
        let (outgoing, incoming) = channel();
        let mut schedule = Self {
            registry: registry.clone(),
            case: case.name,
            pending: VecDeque::new(),
            installed: Vec::new(),
            incoming,
            outgoing,
            events: Vec::new(),
            now_nanos: 0,
        };
        for fault in case.faults {
            schedule.queue(fault);
        }
        schedule
    }

    /// The case this schedule is running.
    pub fn case(&self) -> &str {
        &self.case
    }

    /// A handle for posting faults to this schedule from anywhere, including
    /// another thread.
    ///
    /// The handle is what makes a fault triggered by a *condition* as ordinary
    /// as one triggered by a clock: whatever code is watching the condition
    /// posts a [`Fault`] the moment it fires, and the schedule installs it on
    /// the next tick. Nothing has to be dated in advance, and the schedule can
    /// already be owned by the simulation.
    pub fn sender(&self) -> FaultSender {
        FaultSender(self.outgoing.clone())
    }

    /// Installs everything due at `now_nanos` and withdraws everything that has
    /// expired, returning the events that raised.
    ///
    /// Every event is also kept, so a schedule driven as a [`Module`] — where
    /// this return value goes nowhere — still reports what it did through
    /// [`Self::events`].
    pub fn advance(&mut self, now_nanos: u64) -> &[FaultEvent] {
        let raised_from = self.events.len();
        self.now_nanos = now_nanos;
        self.drain_incoming();

        while self
            .pending
            .front()
            .is_some_and(|fault| fault.at_nanos <= now_nanos)
        {
            let fault = self.pending.pop_front().expect("a fault was due");
            self.install(fault);
        }
        self.withdraw_expired();

        &self.events[raised_from..]
    }

    /// Adds `fault` and applies whatever that makes due at the time this
    /// schedule has already reached, which for a fault with no date is the fault
    /// itself.
    ///
    /// For a caller that still owns the schedule. One that has handed it to a
    /// simulation uses [`Self::sender`], which arrives at the same place and
    /// dates what it carries by exactly the same rule.
    pub fn inject(&mut self, fault: Fault) -> &[FaultEvent] {
        self.queue(fault);
        self.advance(self.now_nanos)
    }

    /// Withdraws every rule this schedule installed and still holds, and leaves
    /// the faults that have not come due to come due as planned.
    ///
    /// Lifts a campaign's faults without ending the campaign. Anything installed
    /// through the registry by anyone else — an operator's rule, another
    /// schedule's — is left alone, because this names the ids it installed
    /// rather than clearing the targets they sit on.
    pub fn withdraw_in_force(&mut self) {
        for installed in std::mem::take(&mut self.installed) {
            self.withdraw(installed);
        }
    }

    /// Drops every fault that has not come due, so none of them ever installs,
    /// and leaves the rules already in force exactly as they are.
    ///
    /// Stops a campaign without undoing it: nothing new is injected, and what is
    /// already broken stays broken. Each dropped fault is recorded, because a
    /// run these were cancelled out of reads exactly like a run whose faults had
    /// no effect.
    ///
    /// Everything posted by now goes with them, the channel included. A fault
    /// posted *after* this returns is still queued — a [`FaultSender`] that has
    /// been handed out cannot be recalled, and dropping it is what stops it.
    pub fn cancel_pending(&mut self) {
        self.drain_incoming();
        for fault in std::mem::take(&mut self.pending) {
            let due_at_nanos = fault.at_nanos;
            self.raise(
                fault.label,
                fault.target,
                FaultEventKind::Cancelled { due_at_nanos },
            );
        }
    }

    /// Ends the campaign: [`Self::withdraw_in_force`] and then
    /// [`Self::cancel_pending`], so nothing of this schedule's is left installed
    /// and nothing of its is still to come.
    ///
    /// The two halves are separate because ending a campaign and undoing one are
    /// different things, and a schedule that offered only this would have to be
    /// abandoned rather than told to stop.
    pub fn withdraw_all(&mut self) {
        self.withdraw_in_force();
        self.cancel_pending();
    }

    /// Whether every fault still waiting would be accepted, asked now rather
    /// than when each comes due.
    ///
    /// A campaign is written by hand, so a mistyped target name or field is the
    /// likely mistake, and without this it surfaces as a refusal minutes into a
    /// run that then reports no effect — the same reading a fault with no effect
    /// gives.
    ///
    /// Shape only, and against the values the ports hold *now*. A fault
    /// addressing a field beneath an `Option` no producer has populated yet
    /// cannot be checked here, so passing this is not a promise that every
    /// install will succeed later.
    pub fn preflight(&self) -> Result<()> {
        for fault in &self.pending {
            self.registry
                .validate(&fault.target, &fault.request)
                .with_context(|| format!("fault '{}' cannot be installed", fault.label))?;
        }
        Ok(())
    }

    /// Everything this schedule has done, oldest first.
    pub fn events(&self) -> &[FaultEvent] {
        &self.events
    }

    /// The faults that have not come due yet.
    pub fn pending(&self) -> usize {
        self.pending.len()
    }

    /// The rules this schedule installed and has not withdrawn.
    pub fn in_force(&self) -> usize {
        self.installed.len()
    }

    fn drain_incoming(&mut self) {
        loop {
            match self.incoming.try_recv() {
                Ok(fault) => self.queue(fault),
                // The schedule holds a sender of its own, so the channel cannot
                // disconnect while it is alive; both arms mean nothing is
                // waiting.
                Err(TryRecvError::Empty | TryRecvError::Disconnected) => return,
            }
        }
    }

    /// Files `fault` by the time it comes due, ahead of nothing it is later
    /// than, so a fault posted mid-run takes its place among the ones already
    /// waiting rather than at the back.
    fn queue(&mut self, fault: Fault) {
        let position = self
            .pending
            .iter()
            .position(|waiting| waiting.at_nanos > fault.at_nanos)
            .unwrap_or(self.pending.len());
        self.pending.insert(position, fault);
    }

    fn install(&mut self, fault: Fault) {
        let Fault {
            label,
            target,
            request,
            lifetime_nanos,
            ..
        } = fault;

        match self.registry.install(&target, request) {
            Ok(id) => {
                self.installed.push(Installed {
                    id,
                    target: target.clone(),
                    label: label.clone(),
                    // Saturating, so a lifetime that runs past the end of the
                    // clock lands at the end of it rather than wrapping to an
                    // expiry in the past — which would withdraw the fault on the
                    // advance that installed it, and do it silently in a release
                    // build. The longest lifetime expressible means "for the rest
                    // of the run", and this is that.
                    clear_at_nanos: lifetime_nanos
                        .map(|lifetime| self.now_nanos.saturating_add(lifetime)),
                });
                self.raise(label, target, FaultEventKind::Installed(id));
            }
            Err(error) => {
                let reason = format!("{error:#}");
                log::error!("fault '{label}' on '{target}' was refused: {reason}");
                self.raise(label, target, FaultEventKind::Refused(reason));
            }
        }
    }

    fn withdraw_expired(&mut self) {
        let now_nanos = self.now_nanos;
        let (expired, in_force) =
            std::mem::take(&mut self.installed)
                .into_iter()
                .partition(|installed| {
                    installed
                        .clear_at_nanos
                        .is_some_and(|clear_at| clear_at <= now_nanos)
                });
        self.installed = in_force;

        for installed in expired {
            self.withdraw(installed);
        }
    }

    fn withdraw(&mut self, installed: Installed) {
        let Installed {
            id, target, label, ..
        } = installed;

        let kind = match self.registry.clear_rule(&target, id) {
            Ok(true) => FaultEventKind::Withdrawn(id),
            // The rule was gone before its lifetime ran out, so the fault ended
            // at a moment this schedule cannot report. Said plainly rather than
            // reported as a withdrawal this did not perform.
            Ok(false) => FaultEventKind::Refused(format!(
                "{id:?} was no longer installed when its lifetime expired"
            )),
            Err(error) => FaultEventKind::Refused(format!("{error:#}")),
        };
        self.raise(label, target, kind);
    }

    fn raise(&mut self, label: String, target: String, kind: FaultEventKind) {
        self.events.push(FaultEvent {
            at_nanos: self.now_nanos,
            label,
            target,
            kind,
        });
    }
}

/// Drives the schedule from the simulation's own clock.
///
/// Scheduled like any other module, which is the point: a campaign is added to a
/// simulation the way a sensor is, and the run loop keeps no fault bookkeeping
/// of its own.
///
/// Give it a priority above every module that reads a port it touches. Modules
/// run in priority order within a tick, so a lower one would let this tick's
/// readers see the value the fault was meant to replace.
impl Module for FaultSchedule {
    fn init(&mut self) {}

    fn update(&mut self, context: &SimulationContext) {
        for event in self.advance(context.current_sim_nanos) {
            log::info!("{event}");
        }
    }
}

/// Posts faults to a [`FaultSchedule`] that is already running.
///
/// Cloneable and sendable, so a control socket, a console and a run loop can
/// each hold one. What is posted is installed by the schedule, on the thread
/// that advances it.
#[derive(Clone, Debug)]
pub struct FaultSender(Sender<Fault>);

impl FaultSender {
    /// Queues `fault` for the schedule's next advance.
    ///
    /// A fault with no time, or one dated at or before the schedule's current
    /// time, is installed on that advance; a later one waits its turn among the
    /// faults already pending.
    ///
    /// Fails only if the schedule is gone, which is worth an error rather than a
    /// silent drop: an operator who sends a fault into a finished run should not
    /// be told it landed.
    pub fn send(&self, fault: Fault) -> Result<()> {
        if self.0.send(fault).is_err() {
            bail!("the fault schedule this sender belongs to is no longer running");
        }
        Ok(())
    }
}
