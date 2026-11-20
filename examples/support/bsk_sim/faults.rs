//! Upstream `BskSim/models/BSK_Faults.py`: faults a scenario lists and adds
//! to the simulation (`fault.addFaultToSimulation(sim, idx)`).
//!
//! Each [`Fault`] is due at a simulation time. [`configure_faults`] adds one
//! upstream event per fault (`add<name>Fault_<idx>`, `conditionTime` = the
//! fault time), whose action executes the fault once, prints its message and
//! deactivates the event, as upstream's `FaultObject` does.
//!
//! Upstream's faults change model attributes (`RW1.P_max`, `TAM.senNoiseStd`,
//! `CSSConstellationObject.sensorList[i].faultState`, ...). Here a fault is a
//! rule of basilisk-rs's override system ([`basilisk_rs::overrides`]) laid
//! over a port, installed through the [`Registry`] targets the models
//! register ([`super::dynamics`] lists them), so no model is told a fault
//! exists:
//!
//! | upstream | here |
//! |---|---|
//! | `RwPowerFault`: `RW<i>.P_max = limit` | replace `/max_power_w` of `RW<i>.config` |
//! | `RwFrictionFault`: `RW<i>.fCoulomb *= k` | replace `/coulomb_friction_nm` of `RW<i>.config` |
//! | `DebrisImpactOn`/`Off`: `extForce_B`, `extTorquePntB_B` | replace / clear `externalDisturbance.ext_force_body` and `.ext_torque_body` |
//! | `CssSignalFault` `CSSFAULT_OFF` | replace `/sensed_value` with 0 |
//! | `CSSFAULT_STUCK_MAX` | replace `/sensed_value` with `scaleFactor * 1.0` |
//! | `CSSFAULT_STUCK_CURRENT` | freeze `/sensed_value` |
//! | `CSSFAULT_STUCK_RAND`, `CSSFAULT_RAND` | replace `/sensed_value` with a (re)drawn fault noise value |
//! | `MagnetometerFault` `MAG_FAULT_STUCK_VALUE` / `_STUCK_CURRENT` | replace / freeze `/magnetic_field_sensor_t/<axis>` of `TAM_sensor.output_tam_msg` |
//! | `MagPolarNoise` `"NOISE"` | replace `/a_matrix`, `/p_matrix_sqrt_t`, `/walk_bounds_t` of `TAM_sensor.config` |
//! | `RWBitFlipFault`: `rwMotorTorque.GsMatrix_B[3 i + j]` | replace `/spin_axes_body/<i>/<j>` of `rwMotorTorque.rw_params_in_msg`, then `rwMotorTorque` reset (it reads the array there) |
//! | `RWGainBitFlipFault`: `mrpFeedbackRWs.K` / `P` / `Ki` | the [`Shared`] module's gain |
//!
//! # Not ported
//!
//! `MAG_FAULT_SPIKING` (so `MagnetometerFault` spiking and `MagPolarNoise`
//! `"SPIKE"` / `"BOTH"`): upstream multiplies the axis by `spikeAmount` at
//! random steps, inside the magnetometer. basilisk-rs's [`Magnetometer`] has
//! no fault states, and an override can only replace or freeze a value, not
//! scale the one the sensor publishes in the same tick. Those constructors
//! return [`FaultError::Unsupported`].
//!
//! # Differences from upstream
//!
//! * Upstream's `RWBitFlipFault` also flips `gsHat_B` of `DynModels.RW<i>`,
//!   but that attribute is a Python copy of the wheel's axis, so the wheel
//!   itself is unchanged: only the FSW matrix is faulted, here too.
//! * The CSS fault noise is a bounded random walk (`faultNoiseStd = 0.5`,
//!   bounds 2) drawn by the fault from a seeded generator, starting when the
//!   fault does; upstream's walk runs inside each sensor from the start.
//! * Upstream's checks that raise `ValueError` return a [`FaultError`].
//!   Arguments the Rust types already constrain (3-vectors) have no check.
//!
//! [`Magnetometer`]: basilisk_rs::sensors::magnetometer::Magnetometer

#![allow(dead_code)]

use std::str::FromStr;
use std::sync::{Arc, Mutex};

use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::MrpFeedback;
use basilisk_rs::fsw_algorithms::effector_interfaces::rw_motor_torque::RwMotorTorque;
use basilisk_rs::messages::{Output, PlanetStateMsg, SpacecraftStateMsg};
use basilisk_rs::overrides::{Registry, Request, RuleId};
use basilisk_rs::utilities::astro_constants::{REQ_EARTH, RP_EARTH};
use nalgebra::{Matrix3, Vector3};
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, Normal};
use serde_json::{Value, json};

use super::BskSim;
use super::dynamics::{BskDynamics, CSS_COUNT, TAM_NOISE_STD_T, TAM_WALK_BOUNDS_T, WHEEL_COUNT};
use super::events::Check;
use super::fsw::BskFsw;
use super::shared::Shared;

/// A fault upstream would reject with `ValueError`, or one basilisk-rs
/// cannot express.
#[derive(Clone, Debug, PartialEq)]
pub enum FaultError {
    Invalid(String),
    Unsupported(String),
}

impl std::fmt::Display for FaultError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Invalid(message) | Self::Unsupported(message) => formatter.write_str(message),
        }
    }
}

fn invalid<T>(message: &str) -> Result<T, FaultError> {
    Err(FaultError::Invalid(message.to_string()))
}

/// Upstream `wheelIdx`: one wheel (0-3) or `"all"`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wheels {
    One(usize),
    All,
}

/// Upstream `CssSignalFault.type`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CssFault {
    Off,
    StuckCurrent,
    StuckMax,
    StuckRand,
    Rand,
}

impl FromStr for CssFault {
    type Err = FaultError;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        Ok(match value {
            "CSSFAULT_OFF" => Self::Off,
            "CSSFAULT_STUCK_CURRENT" => Self::StuckCurrent,
            "CSSFAULT_STUCK_MAX" => Self::StuckMax,
            "CSSFAULT_STUCK_RAND" => Self::StuckRand,
            "CSSFAULT_RAND" => Self::Rand,
            _ => {
                return invalid(
                    "Fault parameter 'type' must be one of ['CSSFAULT_OFF', 'CSSFAULT_RAND', \
                     'CSSFAULT_STUCK_CURRENT', 'CSSFAULT_STUCK_MAX', 'CSSFAULT_STUCK_RAND']",
                );
            }
        })
    }
}

/// Upstream `MagnetometerFault.faultType` (spiking is not ported).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MagFault {
    StuckCurrent,
    StuckValue,
}

impl FromStr for MagFault {
    type Err = FaultError;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value {
            "MAG_FAULT_STUCK_CURRENT" => Ok(Self::StuckCurrent),
            "MAG_FAULT_STUCK_VALUE" => Ok(Self::StuckValue),
            "MAG_FAULT_SPIKING" => Err(spiking_unsupported()),
            _ => invalid(
                "Fault parameter 'faultType' must be one of ['MAG_FAULT_SPIKING', \
                 'MAG_FAULT_STUCK_CURRENT', 'MAG_FAULT_STUCK_VALUE']",
            ),
        }
    }
}

fn spiking_unsupported() -> FaultError {
    FaultError::Unsupported(
        "MAG_FAULT_SPIKING: basilisk-rs's magnetometer has no fault states, and an override \
         cannot scale the value it publishes"
            .to_string(),
    )
}

/// Upstream `RWGainBitFlipFault.gainType`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gain {
    K,
    P,
    Ki,
}

impl FromStr for Gain {
    type Err = FaultError;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value {
            "K" => Ok(Self::K),
            "P" => Ok(Self::P),
            "Ki" => Ok(Self::Ki),
            _ => invalid("Fault parameter 'gainType' must be 'K', 'P' or 'Ki'"),
        }
    }
}

/// One fault of upstream's `faultList`.
#[derive(Clone, Debug, PartialEq)]
pub enum Fault {
    /// Upstream `RwPowerFault`.
    RwPower {
        time_nanos: u64,
        reduced_limit_w: f64,
        wheels: Wheels,
    },
    /// Upstream `RwFrictionFault`.
    RwFriction {
        time_nanos: u64,
        friction_multiplier: f64,
        wheel: usize,
    },
    /// Upstream `DebrisImpactOn`.
    DebrisImpactOn {
        time_nanos: u64,
        location_m: Vector3<f64>,
        direction: Vector3<f64>,
        mass_kg: f64,
        velocity_mag_mps: f64,
    },
    /// Upstream `DebrisImpactOff`.
    DebrisImpactOff { time_nanos: u64 },
    /// Upstream `CssSignalFault`.
    CssSignal {
        time_nanos: u64,
        kind: CssFault,
        css: Vec<usize>,
    },
    /// Upstream `RWBitFlipFault`.
    RwBitFlip {
        time_nanos: u64,
        bit: u32,
        wheel: usize,
        element: usize,
    },
    /// Upstream `RWGainBitFlipFault`.
    RwGainBitFlip {
        time_nanos: u64,
        bit: u32,
        gain: Gain,
    },
    /// Upstream `MagnetometerFault`.
    Magnetometer {
        time_nanos: u64,
        kind: MagFault,
        axis: usize,
        stuck_value_t: f64,
    },
    /// Upstream `MagPolarNoise` with `faultType = "NOISE"`.
    MagPolarNoise { time_nanos: u64 },
}

impl Fault {
    /// Upstream `RwPowerFault(time, reducedLimit, wheelIdx)`.
    pub fn rw_power(
        time_nanos: u64,
        reduced_limit_w: f64,
        wheels: Wheels,
    ) -> Result<Self, FaultError> {
        if let Wheels::One(index) = wheels
            && index >= WHEEL_COUNT
        {
            return invalid("Fault parameter 'wheelIdx' must be 0-3 or 'all'");
        }
        Ok(Self::RwPower {
            time_nanos,
            reduced_limit_w,
            wheels,
        })
    }

    /// Upstream `RwFrictionFault(time, frictionMultiplier, wheelIdx)`.
    pub fn rw_friction(
        time_nanos: u64,
        friction_multiplier: f64,
        wheel: usize,
    ) -> Result<Self, FaultError> {
        if wheel >= WHEEL_COUNT {
            return invalid("Fault parameter 'wheelIdx' must be 0-3");
        }
        Ok(Self::RwFriction {
            time_nanos,
            friction_multiplier,
            wheel,
        })
    }

    /// Upstream `DebrisImpactOn(time, debris_location, direction, mass,
    /// velocityMag=8e3)`.
    pub fn debris_impact_on(
        time_nanos: u64,
        location_m: [f64; 3],
        direction: [f64; 3],
        mass_kg: f64,
        velocity_mag_mps: f64,
    ) -> Result<Self, FaultError> {
        let direction = Vector3::from(direction);
        if direction.norm() == 0.0 {
            return invalid("Fault parameter 'direction' must be non-zero");
        }
        Ok(Self::DebrisImpactOn {
            time_nanos,
            location_m: Vector3::from(location_m),
            direction: direction.normalize(),
            mass_kg,
            velocity_mag_mps,
        })
    }

    /// Upstream `DebrisImpactOff(time)`.
    pub fn debris_impact_off(time_nanos: u64) -> Self {
        Self::DebrisImpactOff { time_nanos }
    }

    /// Upstream `CssSignalFault(time, type, cssIdx)`.
    pub fn css_signal(time_nanos: u64, kind: &str, css: &[usize]) -> Result<Self, FaultError> {
        let kind = kind.parse()?;
        if css.iter().any(|index| *index >= CSS_COUNT) {
            return invalid("Fault parameter 'cssIdx' must be in the range 0-7");
        }
        Ok(Self::CssSignal {
            time_nanos,
            kind,
            css: css.to_vec(),
        })
    }

    /// Upstream `RWBitFlipFault(time, bitFlipIdx, RW_Idx, elementIdx)`.
    pub fn rw_bit_flip(
        time_nanos: u64,
        bit: u32,
        wheel: usize,
        element: usize,
    ) -> Result<Self, FaultError> {
        if bit > 15 {
            return invalid("Fault parameter 'bitFlipIdx' must be 0-15");
        }
        if wheel >= WHEEL_COUNT {
            return invalid("Fault parameter 'RW_Idx' must be 0-3");
        }
        if element > 2 {
            return invalid("Fault parameter 'elementIdx' must be 0-2");
        }
        Ok(Self::RwBitFlip {
            time_nanos,
            bit,
            wheel,
            element,
        })
    }

    /// Upstream `RWGainBitFlipFault(time, bitFlipIdx, gainType)`.
    pub fn rw_gain_bit_flip(time_nanos: u64, bit: u32, gain: &str) -> Result<Self, FaultError> {
        if bit > 15 {
            return invalid("Fault parameter 'bitFlipIdx' must be 0-15");
        }
        Ok(Self::RwGainBitFlip {
            time_nanos,
            bit,
            gain: gain.parse()?,
        })
    }

    /// Upstream `MagnetometerFault(time, faultType, faultyAxis,
    /// stuckValue=0.0)`.
    pub fn magnetometer(
        time_nanos: u64,
        kind: &str,
        axis: usize,
        stuck_value_t: f64,
    ) -> Result<Self, FaultError> {
        let kind = kind.parse()?;
        if axis > 2 {
            return invalid("Fault parameter 'faultyAxis' must be 0-2");
        }
        Ok(Self::Magnetometer {
            time_nanos,
            kind,
            axis,
            stuck_value_t,
        })
    }

    /// Upstream `MagPolarNoise(time, faultType)`.
    pub fn mag_polar_noise(time_nanos: u64, kind: &str) -> Result<Self, FaultError> {
        match kind {
            "NOISE" => Ok(Self::MagPolarNoise { time_nanos }),
            "SPIKE" | "BOTH" => Err(spiking_unsupported()),
            _ => invalid("Invalid fault type. Must be 'NOISE', 'SPIKE', or 'BOTH'."),
        }
    }

    /// Upstream `time` \[ns\].
    pub fn time_nanos(&self) -> u64 {
        match self {
            Self::RwPower { time_nanos, .. }
            | Self::RwFriction { time_nanos, .. }
            | Self::DebrisImpactOn { time_nanos, .. }
            | Self::DebrisImpactOff { time_nanos }
            | Self::CssSignal { time_nanos, .. }
            | Self::RwBitFlip { time_nanos, .. }
            | Self::RwGainBitFlip { time_nanos, .. }
            | Self::Magnetometer { time_nanos, .. }
            | Self::MagPolarNoise { time_nanos } => *time_nanos,
        }
    }

    /// Upstream `name`, the default of each class.
    pub fn name(&self) -> &'static str {
        match self {
            Self::RwPower { .. } => "rwPowerLimited",
            Self::RwFriction { .. } => "rwFriction",
            Self::DebrisImpactOn { .. } => "DebrisImpactOn",
            Self::DebrisImpactOff { .. } => "DebrisImpactOff",
            Self::CssSignal { .. } => "cssSignalOff",
            Self::RwBitFlip { .. } => "RWBitFlipFault",
            Self::RwGainBitFlip { .. } => "RWGainBitFlipFault",
            Self::Magnetometer { .. } => "magnetometerFault",
            Self::MagPolarNoise { .. } => "magPolarFault",
        }
    }
}

/// Upstream `faultTypeInList(faultList, type)`, by variant.
pub fn fault_type_in_list(faults: &[Fault], like: &Fault) -> bool {
    faults
        .iter()
        .any(|fault| std::mem::discriminant(fault) == std::mem::discriminant(like))
}

/// Upstream `getFaultFromList(faultList, type)`, by variant.
pub fn faults_like<'a>(faults: &'a [Fault], like: &Fault) -> Vec<&'a Fault> {
    faults
        .iter()
        .filter(|fault| std::mem::discriminant(*fault) == std::mem::discriminant(like))
        .collect()
}

/// What the faults act on: the override targets, and the few handles the
/// override system cannot stand in for.
pub struct FaultTargets {
    registry: Registry,
    dyn_rate_s: f64,
    spacecraft_state: Output<SpacecraftStateMsg>,
    earth_state: Output<PlanetStateMsg>,
    mrp_feedback_rws: Shared<MrpFeedback>,
    rw_motor_torque: Shared<RwMotorTorque>,
    messages: Arc<Mutex<Vec<String>>>,
}

impl Clone for FaultTargets {
    fn clone(&self) -> Self {
        Self {
            registry: self.registry.clone(),
            dyn_rate_s: self.dyn_rate_s,
            spacecraft_state: self.spacecraft_state.clone(),
            earth_state: self.earth_state.clone(),
            mrp_feedback_rws: self.mrp_feedback_rws.handle(),
            rw_motor_torque: self.rw_motor_torque.handle(),
            messages: Arc::clone(&self.messages),
        }
    }
}

impl FaultTargets {
    pub fn new(base: &BskSim, dynamics: &BskDynamics, fsw: &BskFsw) -> Self {
        Self {
            registry: base.registry.clone(),
            dyn_rate_s: base.dyn_rate_nanos as f64 * 1e-9,
            spacecraft_state: dynamics.spacecraft.state_out.clone(),
            earth_state: dynamics.earth.module.output_planet_msg.clone(),
            mrp_feedback_rws: fsw.mrp_feedback_rws.module.handle(),
            rw_motor_torque: fsw.rw_motor_torque.module.handle(),
            messages: Arc::default(),
        }
    }

    /// The messages the executed faults printed, in order.
    pub fn messages(&self) -> Vec<String> {
        self.messages
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .clone()
    }

    fn install(&self, target: &str, request: Value) -> RuleId {
        self.registry
            .install(target, Request::replace(request).expect("valid replace"))
            .unwrap_or_else(|error| panic!("fault on {target}: {error:#}"))
    }

    fn freeze(&self, target: &str, fields: Value) {
        self.registry
            .install(target, Request::freeze(fields).expect("valid freeze"))
            .unwrap_or_else(|error| panic!("fault on {target}: {error:#}"));
    }

    fn effective(&self, target: &str) -> Value {
        self.registry
            .effective(target)
            .unwrap_or_else(|error| panic!("fault target {target}: {error:#}"))
    }

    fn print(&self, message: String) {
        println!("{message}");
        self.messages
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .push(message);
    }
}

fn number(value: &Value, pointer: &str) -> f64 {
    value
        .pointer(pointer)
        .and_then(Value::as_f64)
        .unwrap_or_else(|| panic!("no number at {pointer}"))
}

fn minutes(nanos: u64) -> f64 {
    nanos as f64 * 1e-9 / 60.0
}

/// Upstream `configure_faults(faultList)`: `fault.addFaultToSimulation(self,
/// i)` for each fault. The events are checked at the dynamics rate.
pub fn configure_faults(base: &mut BskSim, targets: &FaultTargets, faults: &[Fault]) {
    for (index, fault) in faults.iter().enumerate() {
        let event_name = format!("add{}Fault_{index}", fault.name());
        let mut state = FaultState {
            fault: fault.clone(),
            targets: targets.clone(),
            executed: false,
            printed: false,
            random: StdRng::seed_from_u64(0x4253_4b46 + index as u64),
            fault_detail: None,
        };
        let name = event_name.clone();
        base.events.add(
            event_name,
            Check::At(fault.time_nanos()),
            true,
            |_| true,
            move |now, activity| {
                state.execute(now);
                state.print();
                activity.set(&name, false);
            },
        );
        // Upstream `CSSFAULT_RAND` redraws the sensor value at every step.
        if let Fault::CssSignal {
            kind: CssFault::Rand,
            css,
            time_nanos,
        } = fault
        {
            let name = format!("add{}Fault_{index}_redraw", fault.name());
            let mut redraw = CssRedraw::new(targets, css, index);
            let start = *time_nanos;
            base.events.add(
                name.clone(),
                Check::Every(base.dyn_rate_nanos),
                true,
                move |now| now > start,
                move |_, activity| {
                    redraw.step();
                    activity.set(&name, true);
                },
            );
        }
    }
}

struct FaultState {
    fault: Fault,
    targets: FaultTargets,
    executed: bool,
    printed: bool,
    random: StdRng,
    /// The values a bit flip changed, for its message.
    fault_detail: Option<String>,
}

/// Upstream `CoarseSunSensor.faultNoiseStd`.
const CSS_FAULT_NOISE_STD: f64 = 0.5;
/// Upstream's fault-noise walk bounds.
const CSS_FAULT_NOISE_BOUND: f64 = 2.0;
/// Upstream BskSim `cssDevice.scaleFactor`.
const CSS_SCALE_FACTOR: f64 = 2.0;

fn css_target(index: usize) -> String {
    format!("CSS{}.output_sun_sensor_msg", index + 1)
}

/// The fault value of a CSS: upstream sets `sensedValue` before the scale
/// factor and the `[0, 1e6]` saturation.
fn css_fault_value(raw: f64) -> f64 {
    (raw * CSS_SCALE_FACTOR).clamp(0.0, 1e6)
}

/// The `CSSFAULT_RAND` walk of the faulted sensors.
struct CssRedraw {
    targets: FaultTargets,
    css: Vec<usize>,
    walk: Vec<f64>,
    rules: Vec<Option<RuleId>>,
    random: StdRng,
}

impl CssRedraw {
    fn new(targets: &FaultTargets, css: &[usize], index: usize) -> Self {
        Self {
            targets: targets.clone(),
            css: css.to_vec(),
            walk: vec![0.0; css.len()],
            rules: vec![None; css.len()],
            random: StdRng::seed_from_u64(0x5241_4e44 + index as u64),
        }
    }

    fn step(&mut self) {
        let normal = Normal::new(0.0, CSS_FAULT_NOISE_STD).expect("valid noise");
        for (slot, css) in self.css.iter().enumerate() {
            self.walk[slot] = (self.walk[slot] + normal.sample(&mut self.random))
                .clamp(-CSS_FAULT_NOISE_BOUND, CSS_FAULT_NOISE_BOUND);
            let target = css_target(*css);
            if let Some(rule) = self.rules[slot].take() {
                self.targets
                    .registry
                    .clear_rule(&target, rule)
                    .expect("registered target");
            }
            let value = css_fault_value(self.walk[slot]);
            self.rules[slot] = Some(self.targets.install(
                &target,
                json!({ "/sensed_value": value, "/valid": value > 0.0 }),
            ));
        }
    }
}

impl FaultState {
    fn execute(&mut self, now_nanos: u64) {
        let targets = self.targets.clone();
        match self.fault.clone() {
            // Upstream's debris faults have no `executed` guard.
            Fault::DebrisImpactOn {
                location_m,
                direction,
                mass_kg,
                velocity_mag_mps,
                ..
            } => {
                let momentum = mass_kg * velocity_mag_mps * direction;
                let force_b = momentum / targets.dyn_rate_s;
                let torque_b = location_m.cross(&force_b);
                targets.install(
                    "externalDisturbance.ext_force_body",
                    json!({ "/force_request_body_n": [force_b.x, force_b.y, force_b.z] }),
                );
                targets.install(
                    "externalDisturbance.ext_torque_body",
                    json!({ "/torque_request_body_nm": [torque_b.x, torque_b.y, torque_b.z] }),
                );
                return;
            }
            Fault::DebrisImpactOff { .. } => {
                for target in [
                    "externalDisturbance.ext_force_body",
                    "externalDisturbance.ext_torque_body",
                ] {
                    targets.registry.clear(target).expect("registered target");
                }
                return;
            }
            _ => {}
        }
        if self.executed {
            return;
        }
        self.executed = true;
        match self.fault.clone() {
            Fault::RwPower {
                reduced_limit_w,
                wheels,
                ..
            } => {
                let indices: Vec<usize> = match wheels {
                    Wheels::All => (0..WHEEL_COUNT).collect(),
                    Wheels::One(index) => vec![index],
                };
                for index in indices {
                    targets.install(
                        &format!("RW{}.config", index + 1),
                        json!({ "/max_power_w": reduced_limit_w }),
                    );
                }
            }
            Fault::RwFriction {
                friction_multiplier,
                wheel,
                ..
            } => {
                let target = format!("RW{}.config", wheel + 1);
                let friction = number(&targets.effective(&target), "/coulomb_friction_nm");
                targets.install(
                    &target,
                    json!({ "/coulomb_friction_nm": friction * friction_multiplier }),
                );
            }
            Fault::CssSignal { kind, css, .. } => {
                for index in css {
                    let target = css_target(index);
                    match kind {
                        CssFault::Off => {
                            targets
                                .install(&target, json!({ "/sensed_value": 0.0, "/valid": false }));
                        }
                        CssFault::StuckMax => {
                            targets.install(
                                &target,
                                json!({ "/sensed_value": css_fault_value(1.0), "/valid": true }),
                            );
                        }
                        CssFault::StuckCurrent => {
                            targets.freeze(&target, json!(["/sensed_value", "/valid"]));
                        }
                        CssFault::StuckRand | CssFault::Rand => {
                            // `CssRedraw` takes over for `CSSFAULT_RAND`.
                            let draw = Normal::new(0.0, CSS_FAULT_NOISE_STD)
                                .expect("valid noise")
                                .sample(&mut self.random)
                                .clamp(-CSS_FAULT_NOISE_BOUND, CSS_FAULT_NOISE_BOUND);
                            let value = css_fault_value(draw);
                            targets.install(
                                &target,
                                json!({ "/sensed_value": value, "/valid": value > 0.0 }),
                            );
                        }
                    }
                }
            }
            Fault::RwBitFlip {
                bit,
                wheel,
                element,
                ..
            } => {
                let target = "rwMotorTorque.rw_params_in_msg";
                let pointer = format!("/spin_axes_body/{wheel}/{element}");
                let original = number(&targets.effective(target), &pointer);
                let flipped = f16_to_f64(f64_to_f16(original) ^ (1 << bit));
                targets.install(target, json!({ pointer: flipped }));
                // Upstream assigns `GsMatrix_B`, which `rwMotorTorque` builds
                // from its wheel input at reset.
                let now = now_nanos;
                targets.rw_motor_torque.lock().reset(now);
                self.fault_detail = Some(format!(
                    "The value changed from {original} (binary: {:016b}) to {flipped} (binary: {:016b})",
                    f64_to_f16(original),
                    f64_to_f16(flipped)
                ));
            }
            Fault::RwGainBitFlip { bit, gain, .. } => {
                let mut module = targets.mrp_feedback_rws.lock();
                let value = match gain {
                    Gain::K => &mut module.config.k,
                    Gain::P => &mut module.config.p,
                    Gain::Ki => &mut module.config.ki,
                };
                let original = *value;
                *value = f16_to_f64(f64_to_f16(original) ^ (1 << bit));
                self.fault_detail = Some(format!(
                    "The value changed from {original} (binary: {:016b}) to {} (binary: {:016b})",
                    f64_to_f16(original),
                    *value,
                    f64_to_f16(*value)
                ));
            }
            Fault::Magnetometer {
                kind,
                axis,
                stuck_value_t,
                ..
            } => {
                let pointer = format!("/magnetic_field_sensor_t/{axis}");
                match kind {
                    MagFault::StuckValue => {
                        targets.install(
                            "TAM_sensor.output_tam_msg",
                            json!({ pointer: stuck_value_t }),
                        );
                    }
                    MagFault::StuckCurrent => {
                        targets.freeze("TAM_sensor.output_tam_msg", json!([pointer]));
                    }
                }
            }
            Fault::MagPolarNoise { .. } => {
                let request = polar_noise(&targets);
                targets.install("TAM_sensor.config", request);
            }
            Fault::DebrisImpactOn { .. } | Fault::DebrisImpactOff { .. } => unreachable!(),
        }
    }

    /// Upstream `print()`, once.
    fn print(&mut self) {
        if self.printed {
            return;
        }
        let time = minutes(self.fault.time_nanos());
        let message = match &self.fault {
            Fault::RwPower {
                reduced_limit_w,
                wheels,
                ..
            } => match wheels {
                Wheels::All => format!(
                    "RW Power Fault: all RW's power limit reduced to {reduced_limit_w} Watts at {time} minutes!"
                ),
                Wheels::One(index) => format!(
                    "RW Power Fault: RW{index}'s power limit reduced to {reduced_limit_w} Watts at {time} minutes!"
                ),
            },
            Fault::RwFriction {
                friction_multiplier,
                wheel,
                ..
            } => format!(
                "RW Friction Fault: RW{wheel}'s friction multiplied by {friction_multiplier}x at {time} minutes!"
            ),
            Fault::DebrisImpactOn {
                mass_kg,
                velocity_mag_mps,
                direction,
                location_m,
                ..
            } => format!(
                "Debris impact case custom with {mass_kg} kg and {:?} m/s of debris hit at {:?} m at {time} minutes!",
                (direction * *velocity_mag_mps).as_slice(),
                location_m.as_slice()
            ),
            Fault::DebrisImpactOff { .. } => "Debris impact done!".to_string(),
            Fault::CssSignal { kind, css, .. } => {
                format!("CSS Signal Fault {kind:?} executed on CSS {css:?} at {time} minutes!")
            }
            Fault::RwBitFlip {
                wheel,
                element,
                bit,
                ..
            } => format!(
                "RW Bit Flip Fault occurred for RW{wheel} of element {element} on bit {bit} at {time} minutes! {}",
                self.fault_detail.clone().unwrap_or_default()
            ),
            Fault::RwGainBitFlip { gain, bit, .. } => format!(
                "RW Gain {gain:?} Bit Flip Fault occurred on bit {bit} at {time} minutes! {}",
                self.fault_detail.clone().unwrap_or_default()
            ),
            Fault::Magnetometer {
                kind,
                axis,
                stuck_value_t,
                ..
            } => match kind {
                MagFault::StuckCurrent => format!(
                    "Magnetometer Fault: Magnetometer{axis} is stuck at current value at {time} minutes!"
                ),
                MagFault::StuckValue => format!(
                    "Magnetometer Fault: Magnetometer{axis} is stuck at value {stuck_value_t} at {time} minutes!"
                ),
            },
            // Upstream's `MagPolarNoise.print` prints nothing.
            Fault::MagPolarNoise { .. } => {
                self.printed = true;
                return;
            }
        };
        self.targets.print(message);
        self.printed = true;
    }
}

/// Upstream `MagPolarNoise.execute` with `"NOISE"`: the TAM noise and walk
/// bounds blended from nominal to polar by the geodetic latitude.
fn polar_noise(targets: &FaultTargets) -> Value {
    let r_bn_n = targets.spacecraft_state.read().position_m;
    let earth = targets.earth_state.read();
    let dcm_pn = earth
        .orientation
        .map_or_else(Matrix3::identity, |orientation| {
            orientation.inertial_to_fixed
        });
    let r_bp_p = dcm_pn * (r_bn_n - earth.position_inertial_m);
    let (latitude_rad, _, _) = fixed_frame_to_lla(r_bp_p, REQ_EARTH * 1000.0, RP_EARTH / REQ_EARTH);
    let latitude_deg = latitude_rad.to_degrees();
    // 60 deg threshold, 5 deg transition.
    let s = 1.0 / (1.0 + (-(latitude_deg.abs() - 60.0) / 5.0).exp());

    let nominal_noise = TAM_NOISE_STD_T;
    let nominal_walk = TAM_WALK_BOUNDS_T;
    let polar_noise = 120e-9;
    let polar_walk = 600e-9;
    let noise = (1.0 - s) * nominal_noise + s * polar_noise;
    let walk = (1.0 - s) * nominal_walk + s * polar_walk;
    // Upstream keeps the bounds above 3.1 sigma of both the new and the
    // previous noise.
    let config = targets.effective("TAM_sensor.config");
    let previous_noise = serde_json::from_value::<Matrix3<f64>>(config["p_matrix_sqrt_t"].clone())
        .expect("TAM noise matrix")
        .diagonal();
    let walk = Vector3::from_fn(|index, _| walk.max(3.1 * noise.max(previous_noise[index])));

    json!({
        "/a_matrix": serde_json::to_value(Matrix3::<f64>::identity()).expect("matrix"),
        "/p_matrix_sqrt_t": serde_json::to_value(Matrix3::from_diagonal_element(noise)).expect("matrix"),
        "/walk_bounds_t": serde_json::to_value(walk).expect("vector"),
    })
}

/// Upstream `vizSupport.fixedframe2lla(r_GP_P, radEquator, radRatio)`:
/// latitude and longitude \[rad\] and altitude \[m\] of a planet-fixed position
/// above the ellipsoid (Bowring's formula).
pub fn fixed_frame_to_lla(
    r_gp_p: Vector3<f64>,
    rad_equator_m: f64,
    rad_ratio: f64,
) -> (f64, f64, f64) {
    let (x, y, z) = (r_gp_p.x, r_gp_p.y, r_gp_p.z);
    let a = rad_equator_m;
    let b = rad_equator_m * rad_ratio;
    let e2 = 1.0 - (b / a).powi(2);
    let ep2 = (a * a - b * b) / (b * b);
    let r = (x * x + y * y).sqrt();
    let longitude = y.atan2(x);
    let theta = (z * a).atan2(r * b);
    let (stheta, ctheta) = theta.sin_cos();
    let latitude = (z + ep2 * b * stheta.powi(3)).atan2(r - e2 * a * ctheta.powi(3));
    let n = a / (1.0 - e2 * latitude.sin().powi(2)).sqrt();
    let altitude = r / latitude.cos() - n;
    (latitude, longitude, altitude)
}

/// numpy `np.float16(value)` bits (`npy_doublebits_to_halfbits`): round to
/// nearest, ties to even.
pub fn f64_to_f16(value: f64) -> u16 {
    let bits = value.to_bits();
    let sign = ((bits & 0x8000_0000_0000_0000) >> 48) as u16;
    let exponent = bits & 0x7ff0_0000_0000_0000;
    let significand = bits & 0x000f_ffff_ffff_ffff;
    // Overflow and NaN.
    if exponent >= 0x40f0_0000_0000_0000 {
        if exponent == 0x7ff0_0000_0000_0000 && significand != 0 {
            let nan = (0x7c00 + (significand >> 42)) as u16;
            return sign + if nan == 0x7c00 { nan + 1 } else { nan };
        }
        return sign + 0x7c00;
    }
    // Underflow: a subnormal half or a signed zero.
    if exponent <= 0x3f00_0000_0000_0000 {
        if exponent < 0x3e60_0000_0000_0000 {
            return sign;
        }
        let mut subnormal = (0x0010_0000_0000_0000 + significand) << ((exponent >> 52) - 998);
        if (subnormal & 0x003f_ffff_ffff_ffff) != 0x0010_0000_0000_0000 {
            subnormal += 0x0010_0000_0000_0000;
        }
        return sign + (subnormal >> 53) as u16;
    }
    let half_exponent = ((exponent - 0x3f00_0000_0000_0000) >> 42) as u16;
    let mut rounded = significand;
    if (rounded & 0x0000_07ff_ffff_ffff) != 0x0000_0200_0000_0000 {
        rounded += 0x0000_0200_0000_0000;
    }
    // A carry out of the significand increments the exponent.
    sign + half_exponent + (rounded >> 42) as u16
}

/// The value of float16 `bits`.
pub fn f16_to_f64(bits: u16) -> f64 {
    let sign = if bits & 0x8000 != 0 { -1.0 } else { 1.0 };
    let exponent = i32::from((bits >> 10) & 0x1f);
    let fraction = f64::from(bits & 0x3ff);
    sign * match exponent {
        0 => fraction * 2f64.powi(-24),
        31 if fraction == 0.0 => f64::INFINITY,
        31 => f64::NAN,
        _ => (1.0 + fraction / 1024.0) * 2f64.powi(exponent - 15),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// numpy: `np.float16(0.5416752)` is `0x3855`, and flipping bit 0 gives
    /// `0x3854`.
    #[test]
    fn float16_matches_numpy() {
        for (value, bits) in [
            (1.0, 0x3c00),
            (-2.0, 0xc000),
            (3.5, 0x4300),
            (0.1, 0x2e66),
            (65504.0, 0x7bff),
            (2f64.powi(-24), 0x0001),
            (0.0, 0x0000),
        ] {
            assert_eq!(f64_to_f16(value), bits, "{value}");
            assert_eq!(
                f16_to_f64(bits),
                if value == 0.1 { 0.0999755859375 } else { value }
            );
        }
        // A tie rounds to the even mantissa: 1 + 2^-11 is halfway to 1 + 2^-10.
        assert_eq!(f64_to_f16(1.0 + 2f64.powi(-11)), 0x3c00);
        assert_eq!(f64_to_f16(1.0 + 3.0 * 2f64.powi(-11)), 0x3c02);
    }

    /// Upstream `test_fixedframe2lla_basic_case`.
    #[test]
    fn fixed_frame_to_lla_on_the_equator() {
        let (latitude, longitude, _) = fixed_frame_to_lla(
            Vector3::new(6378137.0, 0.0, 0.0),
            REQ_EARTH * 1000.0,
            RP_EARTH / REQ_EARTH,
        );
        assert!(latitude.abs() < 1e-6);
        assert!(longitude.abs() < 1e-6);
    }
}
