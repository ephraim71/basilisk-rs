//! Upstream `BskSim/models/BSK_Dynamics.py`: the 750 kg spacecraft of every
//! single-spacecraft BskSim scenario, its environment and its devices.
//!
//! | upstream (`DynamicsTask` priority) | here |
//! |---|---|
//! | `gravFactory.createBodies(['sun', 'earth', 'moon'])`, `createSpiceInterface("2012 MAY 1 00:28:30.0")`, `zeroBase = 'Earth'` (200) | [`BskDynamics::sun`], [`BskDynamics::earth`], [`BskDynamics::moon`]: ANISE ephemerides (DE440s, `pck11.pca`), Earth-centred, at [`epoch`] |
//! | `eclipseObject` (204) | [`BskDynamics::eclipse`] |
//! | `scObject` `bskSat` (201) with `rwStateEffector` (301), `extForceTorqueObject` (300) and `thrustersDynamicEffector` | [`BskDynamics::spacecraft`] |
//! | `magModule` `WMM` (200), `TAM` `TAM_sensor` (200) | [`BskDynamics::wmm`], [`BskDynamics::tam`] |
//! | `EarthEphemObject` (199) | [`BskDynamics::earth_ephemeris`] |
//! | `simpleNavObject` (109) | [`BskDynamics::simple_nav`] |
//! | `CSSConstellationObject` (108) | [`BskDynamics::css`] |
//! | `AddRWFault`, `PeriodicRWFault` and their events | [`RwFaults`] |
//!
//! The order of the module updates is upstream's, with two differences that
//! basilisk-rs's missing message headers ask for: the ephemerides run before
//! the spacecraft (the gravity effector takes a planet message as written at
//! the current tick, see `grav_factory`), and the wheels take their commands
//! from [`BskDynamics::rw_commands`], which splits the FSW wheel-torque array
//! where upstream's wheel effector reads it, at the start of the tick.
//!
//! # Differences from upstream
//!
//! * The planet states come from ANISE (DE440s) instead of CSPICE (DE430),
//!   as in the other ported scenarios.
//! * basilisk-rs's [`Eclipse`] has one occulting body at the origin, here the
//!   Earth: the Moon's shadow upstream adds is not modelled. It never reaches
//!   the BskSim orbits.
//! * Upstream's `CSSConstellation` publishes one array message; basilisk-rs
//!   sensors keep one message per device, so [`CssConstellation`] updates the
//!   eight sensors in one module and publishes eight messages.
//! * The eight ACS thrusters are added to the spacecraft without commands,
//!   as upstream, which never schedules their effector: they never fire.
//! * `simpleNav` is [`NoisySimpleNav`] (upstream's error model), with
//!   upstream's default zero `PMatrix`; `scenario_LambertGuidance` sets it.
//! * The WMM epoch is the simulation epoch ([`WmmField`] evaluates the model
//!   at the simulation clock, upstream reads `epochMsg`, the same instant).
//!   Upstream subscribes no planet message to the WMM module, so the field
//!   is evaluated with the Earth-fixed frame aligned with the inertial one;
//!   [`WmmField`] does the same with its planet input unconnected.
//! * `np.random.uniform()` of `PeriodicRWFault` is a seeded [`StdRng`].
//!
//! # Fault targets
//!
//! [`BskDynamics::new`] registers in the [`Registry`]: `RW1.config` ...
//! `RW4.config` (the wheel parameters upstream's faults change on
//! `DynModels.RW1` ...), `CSS1.output_sun_sensor_msg` ... `CSS8...`,
//! `TAM_sensor.config`, `TAM_sensor.output_tam_msg`, and
//! `externalDisturbance.ext_force_body` / `externalDisturbance.ext_torque_body`:
//! upstream's `extForceTorqueObject.extForce_B` and `extTorquePntB_B` are
//! module parameters, here the commands of a second [`ExtForceTorque`] whose
//! loads add to the first one's.

#![allow(dead_code)]

use std::sync::{Arc, Mutex};

use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffector, ReactionWheelStateEffectorConfig,
};
use basilisk_rs::dynamics::thruster_dynamic_effector::ThrusterDynamicEffector;
use basilisk_rs::environment::anise_planet_ephemeris::AnisePlanetEphemeris;
use basilisk_rs::environment::anise_sun_ephemeris::{AniseSunEphemeris, AniseSunEphemerisConfig};
use basilisk_rs::environment::eclipse::{Eclipse, EclipseConfig};
use basilisk_rs::environment::ephemeris_converter::{EphemerisConverter, EphemerisConverterConfig};
use basilisk_rs::environment::wmm_field::{WmmField, WmmFieldConfig};
use basilisk_rs::messages::{
    BodyForceCommandMsg, BodyTorqueCommandMsg, Input, Output, ReactionWheelStateMsg,
    RwArrayConfigMsg, RwSpeedMsg, SunSensorMsg,
};
use basilisk_rs::overrides::{Registry, Request, TargetKind};
use basilisk_rs::sensors::coarse_sun_sensor::{CoarseSunSensor, CoarseSunSensorConfig};
use basilisk_rs::sensors::magnetometer::{Magnetometer, MagnetometerConfig};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{HubStateHandle, Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::rigid_body_kinematics::{Axis, mi};
use basilisk_rs::{Module, SimulationContext};
use hifitime::Epoch;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use serde_json::json;

use super::events::Check;
use super::grav_factory::{Body, PLANETARY_SPK};
use super::rw_factory::RwFactory;
use super::simple_nav::{NoisySimpleNav, SimpleNavMatrix, SimpleNavVector};
use super::staged::UpdatedAtInit;
use super::thruster_factory::{ThrusterFactory, ThrusterModel};
use super::{BskSim, HubInit};

/// Upstream `timeInitString = "2012 MAY 1 00:28:30.0"` (UTC).
pub fn epoch() -> Epoch {
    Epoch::from_gregorian_utc(2012, 5, 1, 0, 28, 30, 0)
}

/// Upstream `I_sc` \[kg m^2\].
pub fn hub_inertia_kg_m2() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0))
}

/// Upstream `scObject.hub.mHub` \[kg\].
pub const HUB_MASS_KG: f64 = 750.0;
/// Upstream `maxRWMomentum` \[N m s\].
pub const MAX_RW_MOMENTUM_NMS: f64 = 50.0;
/// The number of reaction wheels.
pub const WHEEL_COUNT: usize = 4;
/// The number of coarse Sun sensors.
pub const CSS_COUNT: usize = 8;

/// Upstream `rwElAngle`, `rwAzimuthAngle` \[deg\] and `rwPosVector` \[m\] of the
/// pyramid of four wheels.
const RW_ELEVATION_DEG: [f64; WHEEL_COUNT] = [40.0; WHEEL_COUNT];
const RW_AZIMUTH_DEG: [f64; WHEEL_COUNT] = [45.0, 135.0, 225.0, 315.0];
const RW_POSITION_M: [[f64; 3]; WHEEL_COUNT] = [
    [0.8, 0.8, 1.79070],
    [0.8, -0.8, 1.79070],
    [-0.8, -0.8, 1.79070],
    [-0.8, 0.8, 1.79070],
];

/// Upstream `gsHat = Mi(-azimuth, 3) Mi(elevation, 2) [1, 0, 0]` of wheel
/// `index`, also the FSW's `fswSetupRW` spin axes.
pub fn rw_spin_axis(index: usize) -> Vector3<f64> {
    mi(-RW_AZIMUTH_DEG[index].to_radians(), Axis::Z)
        * mi(RW_ELEVATION_DEG[index].to_radians(), Axis::Y)
        * Vector3::x()
}

/// Upstream's four `rwFactory.create('Honeywell_HR16', gsHat,
/// maxMomentum=50, rWB_B=...)`, at rest.
pub fn rw_factory() -> RwFactory {
    let mut factory = RwFactory::default();
    for (index, position) in RW_POSITION_M.iter().enumerate() {
        factory.create_honeywell_hr16(
            rw_spin_axis(index),
            MAX_RW_MOMENTUM_NMS,
            0.0,
            Vector3::from(*position),
            ReactionWheelModel::BalancedWheels,
        );
    }
    factory
}

/// Upstream `fswSetupRW.create(gsHat, Js = 50 / 6000 RPM, 0.2)` for each
/// wheel: the FSW wheel configuration, `SetRWConfigMsg`.
pub fn fsw_rw_config() -> RwArrayConfigMsg {
    rw_factory().config_message()
}

/// Upstream `nHat_B_List`, the CSS normals, also `SetCSSWlsEst`'s: upstream's
/// six-digit values, not exactly unit vectors.
#[allow(clippy::approx_constant)]
pub const CSS_NORMALS_B: [[f64; 3]; CSS_COUNT] = [
    [0.0, 0.707107, 0.707107],
    [0.707107, 0., 0.707107],
    [0.0, -0.707107, 0.707107],
    [-0.707107, 0., 0.707107],
    [0.0, -0.965926, -0.258819],
    [-0.707107, -0.353553, -0.612372],
    [0., 0.258819, -0.965926],
    [0.707107, -0.353553, -0.612372],
];

/// Upstream ACS thruster locations \[m\] and directions.
pub const THRUSTER_POSITION_M: [[f64; 3]; 8] = [
    [0.8255, 0.8803, 1.7653],
    [0.8255, 0.8803, 0.2604],
    [0.8803, 0.8255, 1.7653],
    [0.8803, 0.8255, 0.2604],
    [-0.8255, -0.8803, 1.7653],
    [-0.8255, -0.8803, 0.2604],
    [-0.8803, -0.8255, 1.7653],
    [-0.8803, -0.8255, 0.2604],
];
pub const THRUSTER_DIRECTION: [[f64; 3]; 8] = [
    [0.0, -1.0, 0.0],
    [0.0, -1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
];

/// Upstream `TAM.senNoiseStd` \[T\].
pub const TAM_NOISE_STD_T: f64 = 4e-9;
/// Upstream `TAM.walkBounds` \[T\].
pub const TAM_WALK_BOUNDS_T: f64 = 15e-9;

/// Upstream `CSSConstellation`: eight coarse Sun sensors updated in one
/// module, in order. Each publishes its own message.
pub struct CssConstellation {
    pub sensors: Vec<CoarseSunSensor>,
}

impl Module for CssConstellation {
    fn init(&mut self) {
        for sensor in &mut self.sensors {
            sensor.init();
        }
    }

    fn update(&mut self, context: &SimulationContext) {
        for sensor in &mut self.sensors {
            sensor.update(context);
        }
    }
}

/// Upstream `setupCSS`: `fov = 80 deg`, `scaleFactor = 2`, normal `n_hat_b`,
/// and the `CoarseSunSensor` defaults (no Kelly factor, noise or bias,
/// outputs in `[0, 1e6]`).
pub fn css(name: &str, n_hat_b: Vector3<f64>) -> CoarseSunSensor {
    let normal = n_hat_b.normalize();
    let body_to_sensor_quaternion = UnitQuaternion::rotation_between(&normal, &Vector3::z())
        .unwrap_or_else(|| {
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI)
        });
    CoarseSunSensor::new(CoarseSunSensorConfig {
        name: name.to_string(),
        position_m: Vector3::zeros(),
        body_to_sensor_quaternion,
        fov_half_angle_rad: 80.0_f64.to_radians(),
        scale_factor: 2.0,
        kelly_factor: 0.0,
        k_power: 2.0,
        bias: 0.0,
        noise_std: 0.0,
        noise_prop: 0.0,
        walk_bounds: 0.0,
        min_output: 0.0,
        max_output: 1e6,
    })
}

/// Upstream `AddRWFault`, `PeriodicRWFault` and the `addOneTimeRWFault`,
/// `addRepeatedRWFault` events, with the `oneTimeRWFaultFlag`,
/// `oneTimeFaultTime` and `repeatRWFaultFlag` the scenario sets on
/// `BSKSim`. Clones share the flags and the log.
#[derive(Clone, Debug, Default)]
pub struct RwFaults {
    state: Arc<Mutex<RwFaultState>>,
}

#[derive(Debug)]
struct RwFaultState {
    one_time_flag: bool,
    one_time_nanos: i64,
    repeat_flag: bool,
    log: Vec<RwFaultRecord>,
}

impl Default for RwFaultState {
    fn default() -> Self {
        Self {
            one_time_flag: false,
            one_time_nanos: -1,
            repeat_flag: false,
            log: Vec::new(),
        }
    }
}

/// One entry of upstream's `RWFaultLog`.
#[derive(Clone, Debug, PartialEq)]
pub struct RwFaultRecord {
    /// Upstream `faultType`, always `"friction"`.
    pub kind: &'static str,
    /// Coulomb friction added \[N m\].
    pub friction_nm: f64,
    /// Upstream `faultRW`, 1 to 4.
    pub wheel: usize,
    /// When \[min\].
    pub time_min: f64,
}

impl RwFaults {
    /// Upstream `oneTimeRWFaultFlag = 1`, `oneTimeFaultTime = time`.
    pub fn one_time_at(&self, nanos: u64) {
        let mut state = self.lock();
        state.one_time_flag = true;
        state.one_time_nanos = nanos as i64;
    }

    /// Upstream `repeatRWFaultFlag = 1`.
    pub fn repeat(&self) {
        self.lock().repeat_flag = true;
    }

    /// Upstream `DynModels.RWFaultLog`.
    pub fn log(&self) -> Vec<RwFaultRecord> {
        self.lock().log.clone()
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, RwFaultState> {
        self.state
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// Upstream `AddRWFault("friction", fault, faultRW, currentTime)`:
    /// `RW<faultRW>.fCoulomb += fault`, as a rule on the wheel's parameters.
    fn add_friction(&self, registry: &Registry, friction_nm: f64, wheel: usize, now_nanos: u64) {
        self.lock().log.push(RwFaultRecord {
            kind: "friction",
            friction_nm,
            wheel,
            time_min: now_nanos as f64 * 1e-9 / 60.0,
        });
        let target = format!("RW{wheel}.config");
        let current = registry
            .effective(&target)
            .expect("registered wheel")
            .get("coulomb_friction_nm")
            .and_then(serde_json::Value::as_f64)
            .expect("wheel friction");
        registry
            .install(
                &target,
                Request::replace(json!({ "/coulomb_friction_nm": current + friction_nm }))
                    .expect("valid replace"),
            )
            .expect("wheel friction fault");
    }
}

/// Upstream `BSKDynamicModels`.
pub struct BskDynamics {
    /// Upstream `processTasksTimeStep` \[ns\].
    pub task_period_nanos: u64,
    pub eclipse: Eclipse,
    /// The Sun, Earth-centred: `planetStateOutMsgs[sun]`, and the Sun message
    /// of the eclipse and the sensors.
    pub sun: UpdatedAtInit<AniseSunEphemeris>,
    /// `planetStateOutMsgs[earth]`, the zero base, with its `IAU_EARTH`
    /// orientation.
    pub earth: UpdatedAtInit<AnisePlanetEphemeris>,
    /// `planetStateOutMsgs[moon]`.
    pub moon: UpdatedAtInit<AnisePlanetEphemeris>,
    /// The wheel commands of upstream `rwMotorCmdInMsg`.
    pub rw_commands: ArrayMotorTorqueToSingleActuators,
    pub spacecraft: Spacecraft,
    pub wmm: WmmField,
    pub tam: Magnetometer,
    pub earth_ephemeris: EphemerisConverter,
    pub simple_nav: NoisySimpleNav,
    pub css: CssConstellation,
    /// Upstream `extForceTorqueObject.cmdTorqueInMsg`, which the FSW connects
    /// to its `cmdTorqueDirectMsg`.
    pub ext_torque_cmd_in: Input<BodyTorqueCommandMsg>,
    /// Upstream `rwStateEffector.rwSpeedOutMsg`.
    pub rw_speed_out_msg: Output<RwSpeedMsg>,
    /// Upstream `rwStateEffector.rwOutMsgs`.
    pub rw_out_msgs: Vec<Output<ReactionWheelStateMsg>>,
    /// Upstream `RW1` ... `RW4`: the wheel parameters.
    pub rw_configs: Vec<Output<ReactionWheelStateEffectorConfig>>,
    /// Upstream `extForce_B` \[N\], `extTorquePntB_B` \[N m\] (see the module docs).
    pub ext_force_body: Output<BodyForceCommandMsg>,
    pub ext_torque_body: Output<BodyTorqueCommandMsg>,
    /// Upstream `scObject.dynManager.getStateObject('hubPosition'/'hubVelocity')`.
    pub hub: HubStateHandle,
    /// Upstream `gravFactory.gravBodies['earth'].mu` \[m^3/s^2\].
    pub mu_earth_m3ps2: f64,
    pub rw_faults: RwFaults,
}

impl BskDynamics {
    /// Upstream `BSKDynamicModels(SimBase, dynRate)` and
    /// `configure_initial_conditions`.
    pub fn new(base: &mut BskSim, initial: &HubInit) -> Self {
        let period = base.dyn_rate_nanos;
        let start = epoch();

        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: HUB_MASS_KG,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: hub_inertia_kg_m2(),
            integration_step_nanos: period,
            initial_position_m: initial.position_m,
            initial_velocity_mps: initial.velocity_mps,
            initial_sigma_bn: initial.sigma_bn,
            initial_omega_radps: initial.omega_bn_b_radps,
            integrator: None,
        });

        // `SetGravityBodies`.
        let sun = AniseSunEphemeris::new(AniseSunEphemerisConfig {
            name: "sun_ephemeris".to_string(),
            spk_path: PLANETARY_SPK.into(),
        });
        let earth = Body::Earth.ephemeris(Body::Earth);
        let moon = Body::Moon.ephemeris(Body::Earth);
        for (body, central, ephemeris) in [
            (Body::Sun, false, &sun.output_planet_msg),
            (Body::Earth, true, &earth.output_planet_msg),
            (Body::Moon, false, &moon.output_planet_msg),
        ] {
            spacecraft
                .add_grav_body(body.point_mass(central))
                .expect("unique gravity body");
            ephemeris.connect_to(
                spacecraft
                    .grav_body_input_mut(body.name())
                    .expect("added gravity body"),
            );
        }
        let mut earth_ephemeris = EphemerisConverter::new(EphemerisConverterConfig {
            name: "EarthEphemObject".to_string(),
            body_names: vec!["earth".to_string()],
        });
        earth
            .output_planet_msg
            .connect_to(&mut earth_ephemeris.planet_in_msgs[0]);

        // `SetExternalForceTorqueObject`.
        let external = ExtForceTorque::new(ExtForceTorqueConfig {
            name: "externalDisturbance".to_string(),
            ..ExtForceTorqueConfig::default()
        });
        let ext_torque_cmd_in = external.cmd_torque_in_msg.clone();
        spacecraft.add_dynamic_effector(external);
        let ext_force_body = Output::<BodyForceCommandMsg>::default();
        let ext_torque_body = Output::<BodyTorqueCommandMsg>::default();
        let mut parameters = ExtForceTorque::new(ExtForceTorqueConfig {
            name: "externalDisturbanceParameters".to_string(),
            ..ExtForceTorqueConfig::default()
        });
        ext_force_body.connect_to(&mut parameters.cmd_force_body_in_msg);
        ext_torque_body.connect_to(&mut parameters.cmd_torque_in_msg);
        spacecraft.add_dynamic_effector(parameters);

        // `SetSimpleNavObject`.
        let mut simple_nav = NoisySimpleNav::new(
            "SimpleNavigation",
            SimpleNavMatrix::zeros(),
            SimpleNavVector::zeros(),
        );
        spacecraft
            .state_out
            .connect_to(&mut simple_nav.spacecraft_state_in_msg);

        // `SetEclipseObject`.
        let mut eclipse = Eclipse::new(EclipseConfig {
            name: "eclipseObject".to_string(),
            occulting_body_radius_m: Body::Earth.radius_equator_m(),
        });
        spacecraft
            .state_out
            .connect_to(&mut eclipse.input_state_msg);
        sun.output_sun_msg.connect_to(&mut eclipse.input_sun_msg);

        // `SetCSSConstellation`.
        let mut css = CssConstellation {
            sensors: CSS_NORMALS_B
                .iter()
                .enumerate()
                .map(|(index, normal)| css(&format!("CSS{}", index + 1), Vector3::from(*normal)))
                .collect(),
        };
        for sensor in &mut css.sensors {
            sun.output_sun_msg.connect_to(&mut sensor.input_sun_msg);
            spacecraft.state_out.connect_to(&mut sensor.input_state_msg);
            eclipse
                .output_eclipse_msg
                .connect_to(&mut sensor.input_eclipse_msg);
        }

        // `SetReactionWheelDynEffector`.
        let factory = rw_factory();
        let mut wheels = ReactionWheelStateEffector::new("RWA");
        for config in &factory.wheels {
            wheels.add_reaction_wheel(config.clone());
        }
        let rw_commands =
            ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
                name: "rwMotorCmd".to_string(),
                num_actuators: WHEEL_COUNT,
            });
        for (command, wheel) in rw_commands
            .actuator_out_msgs
            .iter()
            .zip(wheels.wheels_mut())
        {
            command.connect_to(&mut wheel.motor_torque_in_msg);
        }
        let rw_speed_out_msg = wheels.rw_speed_out_msg.clone();
        let rw_out_msgs = wheels
            .wheels()
            .iter()
            .map(|wheel| wheel.state_out.clone())
            .collect();
        let rw_configs: Vec<_> = wheels
            .wheels()
            .iter()
            .map(|wheel| wheel.config.clone())
            .collect();
        spacecraft.add_state_effector(wheels);

        // `SetThrusterStateEffector`: eight `MOOG_Monarc_1`, never commanded.
        let mut thrusters = ThrusterFactory::default();
        for (position, direction) in THRUSTER_POSITION_M.iter().zip(THRUSTER_DIRECTION) {
            thrusters.create(
                ThrusterModel::MoogMonarc1,
                Vector3::from(*position),
                Vector3::from(direction),
                None,
            );
        }
        for thruster in &thrusters.thrusters {
            spacecraft.add_dynamic_effector(ThrusterDynamicEffector::new(
                thruster.dynamic_effector_config(),
            ));
        }

        // `SetMagneticField`, `SetMagnetometers`.
        let mut wmm = WmmField::new(WmmFieldConfig {
            name: "WMM".to_string(),
            env_min_reach_m: None,
            env_max_reach_m: None,
        });
        spacecraft.state_out.connect_to(&mut wmm.input_state_msg);
        let mut tam = Magnetometer::new(MagnetometerConfig {
            name: "TAM_sensor".to_string(),
            body_to_sensor_quaternion: UnitQuaternion::identity(),
            bias_t: Vector3::zeros(),
            p_matrix_sqrt_t: Matrix3::from_diagonal_element(TAM_NOISE_STD_T),
            a_matrix: Matrix3::identity(),
            walk_bounds_t: Vector3::repeat(TAM_WALK_BOUNDS_T),
            scale_factor: 1.0,
            min_output_t: -1e200,
            max_output_t: 1e200,
            mtb_residual_field: None,
        });
        spacecraft.state_out.connect_to(&mut tam.input_state_msg);
        wmm.output_magnetic_field_msg
            .connect_to(&mut tam.input_magnetic_field_msg);

        let hub = spacecraft.hub_state_handle();
        let dynamics = Self {
            task_period_nanos: period,
            eclipse,
            sun: UpdatedAtInit::new(sun, start),
            earth: UpdatedAtInit::new(earth, start),
            moon: UpdatedAtInit::new(moon, start),
            rw_commands,
            spacecraft,
            wmm,
            tam,
            earth_ephemeris,
            simple_nav,
            css,
            ext_torque_cmd_in,
            rw_speed_out_msg,
            rw_out_msgs,
            rw_configs,
            ext_force_body,
            ext_torque_body,
            hub,
            mu_earth_m3ps2: Body::Earth.mu_m3ps2(),
            rw_faults: RwFaults::default(),
        };
        dynamics.register_targets(&base.registry);
        dynamics.add_rw_fault_events(base);
        dynamics
    }

    fn register_targets(&self, registry: &Registry) {
        for (index, config) in self.rw_configs.iter().enumerate() {
            registry
                .register(
                    format!("RW{}.config", index + 1),
                    config,
                    TargetKind::Output,
                )
                .expect("unique target");
        }
        for sensor in &self.css.sensors {
            registry
                .register(
                    format!("{}.output_sun_sensor_msg", sensor.config.name),
                    &sensor.output_sun_sensor_msg,
                    TargetKind::Output,
                )
                .expect("unique target");
        }
        registry
            .register("TAM_sensor.config", &self.tam.config, TargetKind::Output)
            .expect("unique target");
        registry
            .register(
                "TAM_sensor.output_tam_msg",
                &self.tam.output_tam_msg,
                TargetKind::Output,
            )
            .expect("unique target");
        registry
            .register(
                "externalDisturbance.ext_force_body",
                &self.ext_force_body,
                TargetKind::Output,
            )
            .expect("unique target");
        registry
            .register(
                "externalDisturbance.ext_torque_body",
                &self.ext_torque_body,
                TargetKind::Output,
            )
            .expect("unique target");
    }

    /// Upstream's `addOneTimeRWFault` and `addRepeatedRWFault` events.
    fn add_rw_fault_events(&self, base: &mut BskSim) {
        let rate = Check::Every(self.task_period_nanos);
        let (faults, registry) = (self.rw_faults.clone(), base.registry.clone());
        let condition_faults = faults.clone();
        base.events.add(
            "addOneTimeRWFault",
            rate,
            true,
            move |now| {
                let state = condition_faults.lock();
                now as i64 >= state.one_time_nanos && state.one_time_flag
            },
            move |now, _| {
                faults.add_friction(&registry, 0.05, 1, now);
                faults.lock().one_time_flag = false;
            },
        );

        let (faults, registry) = (self.rw_faults.clone(), base.registry.clone());
        let condition_faults = faults.clone();
        let mut random = StdRng::seed_from_u64(0x0052_5746_4155_4c54);
        base.events.add(
            "addRepeatedRWFault",
            rate,
            true,
            move |_| condition_faults.lock().repeat_flag,
            move |now, activity| {
                // Upstream `PeriodicRWFault(1 / 3000, "friction", 0.005, 1, t)`.
                if random.r#gen::<f64>() < 1.0 / 3000.0 {
                    faults.add_friction(&registry, 0.005, 1, now);
                }
                activity.set("addRepeatedRWFault", true);
            },
        );
    }

    /// Upstream `AddModelToTask("DynamicsTask", ...)` with upstream's
    /// priorities, the ephemerides moved before the spacecraft.
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let period = self.task_period_nanos;
        basilisk_rs::schedule! { *simulation,
            "eclipseObject" => &mut self.eclipse, period = period, priority = 9_400;
            "sun" => &mut self.sun, period = period, priority = 9_350;
            "earth" => &mut self.earth, period = period, priority = 9_340;
            "moon" => &mut self.moon, period = period, priority = 9_330;
            "rwMotorCmd" => &mut self.rw_commands, period = period, priority = 9_300;
            "bskSat" => &mut self.spacecraft, period = period, priority = 9_200;
            "WMM" => &mut self.wmm, period = period, priority = 9_100;
            "TAM_sensor" => &mut self.tam, period = period, priority = 9_090;
            "EarthEphemObject" => &mut self.earth_ephemeris, period = period, priority = 9_080;
            "SimpleNavigation" => &mut self.simple_nav, period = period, priority = 9_050;
            "cssConstellation" => &mut self.css, period = period, priority = 9_000;
        }
    }

    /// The CSS output messages, upstream `constellationOutMsg`.
    pub fn css_out_msgs(&self) -> Vec<Output<SunSensorMsg>> {
        self.css
            .sensors
            .iter()
            .map(|sensor| sensor.output_sun_sensor_msg.clone())
            .collect()
    }
}
