//! A propellant tank whose mass the thrusters it feeds deplete: port of
//! upstream Basilisk `simulation/dynamics/FuelTank` (`FuelTank` and the
//! `FuelTankModel` family).
//!
//! The tank's propellant mass is its integrated state. Its center of mass and
//! inertia follow from the mass through a [`FuelTankModel`] (constant volume,
//! constant density, emptying sphere, uniform or centrifugal burn of a
//! cylinder), given in the tank frame T at `r_TB_B` with orientation
//! `[TB]`. Propellant leaves the tank at the rate the thrusters it feeds
//! consume plus an optional leak:
//!
//! - thrusters: [`FuelTank::add_thruster_set`] (upstream `addThrusterSet`)
//!   connects a thruster effector's [`MassFlowRateMsg`] output to one of the
//!   tank's [`thruster_mass_flow_in_msgs`](FuelTank::thruster_mass_flow_in_msgs);
//!   the thruster effectors write it at every integrator stage;
//! - leak: [`FuelTankConfig::fuel_leak_rate_kgps`], or
//!   [`fuel_leak_rate_in_msg`](FuelTank::fuel_leak_rate_in_msg) when connected.
//!
//! The tank can also drive fuel-slosh particles
//! ([`FuelTank::push_fuel_slosh_particle`], upstream `pushFuelSloshParticle`),
//! which carry part of its propellant: each is drained in proportion to its
//! share of the total, and the tank drains the rest. See [`super::fuel_slosh`].
//!
//! **Update-only depletion.** With [`FuelTankConfig::update_only`] (the
//! default, upstream `updateOnly`) the mass properties follow the depletion
//! but the equations of motion take the mass-flow rates as zero; otherwise
//! the tank adds the depletion torques of its moving center of mass and
//! changing inertia, and the mass rate enters the hub's equations.
//!
//! Differences from upstream:
//! - The tank owns its slosh particles: their states follow the tank's mass
//!   in the tank's state, and the tank alone is added to the spacecraft.
//!   Upstream's checks that a particle is added to the spacecraft, after its
//!   tank (`test_attachedSloshRequires...`), have no counterpart.
//! - The spacecraft has one set of mass-property rates. The tank and its
//!   particles report the rates upstream substitutes into the equations of
//!   motion (`mEffDotDynamics`, `rEffPrime_CB_BDynamics`,
//!   `IEffPrimePntB_BDynamics`), so the hub dynamics match upstream; the
//!   spacecraft's center-of-mass velocity, used for its accumulated ΔV and
//!   its energy and momentum diagnostics, misses the depletion terms upstream
//!   takes from the retained-property rates (`mEffDot`, `rEffPrime_CB_B`)
//!   while the tank is update-only. [`FuelTank::depletion`] reports both.
//! - Upstream clamps a mass state that integrated below zero inside the mass
//!   property update. Here negative masses are read as zero and the states
//!   are clamped once the step is complete
//!   ([`StateEffector::modify_states`]).
//! - A dynamic thruster's blow-down reads [`FuelTankMsg::fuel_mass_kg`], the
//!   tank's own propellant at the start of the step; upstream sets the
//!   tank-plus-particles mass at every stage.

use nalgebra::{Matrix3, Vector3};
use std::any::Any;
use std::cell::Cell;
use std::f64::consts::PI;

use super::fuel_slosh::{DepletionMassProps, FuelSlosh, FuelSloshFlow};
use crate::messages::{FuelTankMsg, Input, MassFlowRateMsg, Output, SpacecraftStateMsg};
use crate::spacecraft::{BackSubMatrices, StateEffector, StateEffectorMassProps};
use crate::utilities::newton_raphson::newton_raphson_solve;

/// Geometry of a [`FuelTankModel`]: upstream's `FuelTankModel` subclasses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FuelTankShape {
    /// Upstream `FuelTankModelConstantVolume`: a sphere of constant radius
    /// whose propellant density drops.
    ConstantVolume {
        /// Upstream `radiusTankInit` \[m\].
        radius_tank_init_m: f64,
    },
    /// Upstream `FuelTankModelConstantDensity`: a sphere that shrinks with
    /// its propellant.
    ConstantDensity {
        /// Upstream `radiusTankInit` \[m\]: radius when full.
        radius_tank_init_m: f64,
    },
    /// Upstream `FuelTankModelEmptying`: a spherical tank emptying along its
    /// `-k̂3` axis, the propellant surface at angle `θ*` from `k̂3`.
    Emptying {
        /// Upstream `radiusTankInit` \[m\].
        radius_tank_init_m: f64,
    },
    /// Upstream `FuelTankModelUniformBurn`: a solid cylinder about `k̂3`
    /// whose density drops.
    UniformBurn {
        /// Upstream `radiusTankInit` \[m\].
        radius_tank_init_m: f64,
        /// Upstream `lengthTank` \[m\].
        length_tank_m: f64,
    },
    /// Upstream `FuelTankModelCentrifugalBurn`: a cylinder about `k̂3` burning
    /// from its axis outwards.
    CentrifugalBurn {
        /// Upstream `radiusTankInit` \[m\]: outer radius.
        radius_tank_init_m: f64,
        /// Upstream `lengthTank` \[m\].
        length_tank_m: f64,
    },
}

/// How a tank's propellant mass sets its center of mass and inertia
/// (upstream `FuelTankModel`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FuelTankModel {
    pub shape: FuelTankShape,
    /// Upstream `propMassInit` \[kg\]: initial propellant mass.
    pub prop_mass_init_kg: f64,
    /// Upstream `maxFuelMass` \[kg\]: capacity, reported only (default 1).
    pub max_fuel_mass_kg: f64,
    /// Upstream `r_TcT_TInit` \[m\]: center of mass relative to the tank
    /// frame origin T when full, tank-frame components (default zero).
    pub r_tct_t_init_m: Vector3<f64>,
}

/// Tank center of mass and inertia for one propellant mass (upstream
/// `computeTankProps` outputs).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FuelTankProperties {
    /// Upstream `r_TcT_T` \[m\]: center of mass relative to T, T components.
    pub r_tct_t_m: Vector3<f64>,
    /// Upstream `ITankPntT_T` \[kg m^2\]: propellant inertia about its center
    /// of mass, T components.
    pub inertia_tank_pnt_t_t_kg_m2: Matrix3<f64>,
}

/// Rates of the tank properties at a propellant flow (upstream
/// `computeTankPropDerivs` outputs).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FuelTankPropertyRates {
    /// Upstream `rPrime_TcT_T` \[m/s\].
    pub r_prime_tct_t_mps: Vector3<f64>,
    /// Upstream `rPPrime_TcT_T` \[m/s^2\].
    pub r_pprime_tct_t_mps2: Vector3<f64>,
    /// Upstream `IPrimeTankPntT_T` \[kg m^2/s\].
    pub inertia_prime_tank_pnt_t_t_kg_m2ps: Matrix3<f64>,
}

impl FuelTankModel {
    /// A model of `shape` initially holding `prop_mass_init_kg`, with
    /// upstream's other defaults.
    pub fn new(shape: FuelTankShape, prop_mass_init_kg: f64) -> Self {
        Self {
            shape,
            prop_mass_init_kg,
            max_fuel_mass_kg: 1.0,
            r_tct_t_init_m: Vector3::zeros(),
        }
    }

    /// Upstream `computeTankProps`: center of mass and inertia holding
    /// `fuel_mass_kg`.
    pub fn properties(&self, fuel_mass_kg: f64) -> FuelTankProperties {
        let m = fuel_mass_kg;
        let r_init = self.r_tct_t_init_m;
        match self.shape {
            FuelTankShape::ConstantVolume { radius_tank_init_m } => FuelTankProperties {
                r_tct_t_m: r_init,
                inertia_tank_pnt_t_t_kg_m2: 2.0 / 5.0
                    * m
                    * radius_tank_init_m
                    * radius_tank_init_m
                    * Matrix3::identity(),
            },
            FuelTankShape::ConstantDensity { radius_tank_init_m } => {
                let radius_tank = self.constant_density_radius(m, radius_tank_init_m);
                FuelTankProperties {
                    r_tct_t_m: r_init,
                    inertia_tank_pnt_t_t_kg_m2: 2.0 / 5.0
                        * m
                        * radius_tank
                        * radius_tank
                        * Matrix3::identity(),
                }
            }
            FuelTankShape::Emptying { radius_tank_init_m } => {
                self.emptying_properties(m, radius_tank_init_m)
            }
            FuelTankShape::UniformBurn {
                radius_tank_init_m: r,
                length_tank_m: l,
            } => {
                let transverse = m * (r * r / 4.0 + l * l / 12.0);
                FuelTankProperties {
                    r_tct_t_m: r_init,
                    inertia_tank_pnt_t_t_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                        transverse,
                        transverse,
                        m * r * r / 2.0,
                    )),
                }
            }
            FuelTankShape::CentrifugalBurn {
                radius_tank_init_m: r,
                length_tank_m: l,
            } => {
                let r_inner = self.centrifugal_inner_radius(m, r, l);
                let transverse = m * ((r * r + r_inner * r_inner) / 4.0 + l * l / 12.0);
                FuelTankProperties {
                    r_tct_t_m: r_init,
                    inertia_tank_pnt_t_t_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                        transverse,
                        transverse,
                        m * (r * r + r_inner * r_inner) / 2.0,
                    )),
                }
            }
        }
    }

    /// Upstream `computeTankPropDerivs`: the rates of the tank properties
    /// holding `fuel_mass_kg` and draining at `fuel_mass_dot_kgps` (upstream
    /// evaluates them after `computeTankProps` at the same mass, whose
    /// intermediate values they reuse).
    pub fn property_rates(
        &self,
        fuel_mass_kg: f64,
        fuel_mass_dot_kgps: f64,
    ) -> FuelTankPropertyRates {
        let m = fuel_mass_kg;
        let m_dot = fuel_mass_dot_kgps;
        let still = |inertia_prime: Matrix3<f64>| FuelTankPropertyRates {
            r_prime_tct_t_mps: Vector3::zeros(),
            r_pprime_tct_t_mps2: Vector3::zeros(),
            inertia_prime_tank_pnt_t_t_kg_m2ps: inertia_prime,
        };
        match self.shape {
            FuelTankShape::ConstantVolume {
                radius_tank_init_m: r,
            } => still(2.0 / 5.0 * m_dot * r * r * Matrix3::identity()),
            FuelTankShape::ConstantDensity { radius_tank_init_m } => {
                let radius_tank = self.constant_density_radius(m, radius_tank_init_m);
                still(2.0 / 3.0 * m_dot * radius_tank * radius_tank * Matrix3::identity())
            }
            FuelTankShape::Emptying { radius_tank_init_m } => {
                self.emptying_property_rates(m, m_dot, radius_tank_init_m)
            }
            FuelTankShape::UniformBurn {
                radius_tank_init_m: r,
                length_tank_m: l,
            } => {
                let transverse = m_dot * (r * r / 4.0 + l * l / 12.0);
                still(Matrix3::from_diagonal(&Vector3::new(
                    transverse,
                    transverse,
                    m_dot * r * r / 2.0,
                )))
            }
            FuelTankShape::CentrifugalBurn {
                radius_tank_init_m: r,
                length_tank_m: l,
            } => {
                let r_inner = self.centrifugal_inner_radius(m, r, l);
                let transverse = m_dot * (r_inner * r_inner / 2.0 + l * l / 12.0);
                still(Matrix3::from_diagonal(&Vector3::new(
                    transverse,
                    transverse,
                    m_dot * r_inner * r_inner,
                )))
            }
        }
    }

    /// Upstream `FuelTankModelConstantDensity::radiusTank`.
    fn constant_density_radius(&self, fuel_mass_kg: f64, radius_tank_init_m: f64) -> f64 {
        (fuel_mass_kg / self.prop_mass_init_kg).powf(1.0 / 3.0) * radius_tank_init_m
    }

    /// Upstream `FuelTankModelCentrifugalBurn::radiusInner`.
    fn centrifugal_inner_radius(&self, fuel_mass_kg: f64, r: f64, l: f64) -> f64 {
        let rho_fuel = self.prop_mass_init_kg / (PI * r * r * l);
        (r * r - fuel_mass_kg / (PI * l * rho_fuel)).max(0.0).sqrt()
    }

    /// Upstream `FuelTankModelEmptying::rhoFuel` \[kg/m^3\].
    fn emptying_density(&self, r: f64) -> f64 {
        self.prop_mass_init_kg / (4.0 / 3.0 * PI * r * r * r)
    }

    /// Upstream `FuelTankModelEmptying::thetaStar` \[rad\]: the propellant
    /// surface angle from `k̂3` holding `fuel_mass_kg`, by Newton–Raphson
    /// from 90°. The expressions keep upstream's operation order.
    fn emptying_theta_star(&self, fuel_mass_kg: f64, r: f64) -> f64 {
        if fuel_mass_kg == self.prop_mass_init_kg {
            return 0.0;
        }
        let rho_fuel = self.emptying_density(r);
        let f = |theta: f64| {
            2.0 / 3.0
                * PI
                * rho_fuel
                * r
                * r
                * r
                * (1.0 + 3.0 / 2.0 * theta.cos() - 1.0 / 2.0 * theta.cos().powf(3.0))
                - fuel_mass_kg
        };
        let f_prime = |theta: f64| {
            2.0 / 3.0
                * PI
                * rho_fuel
                * r
                * r
                * r
                * (-3.0 / 2.0 * theta.sin() + 3.0 / 2.0 * theta.cos().powf(2.0) * theta.sin())
        };
        newton_raphson_solve(PI / 2.0, 1e-20, f, f_prime)
    }

    fn emptying_properties(&self, fuel_mass_kg: f64, r: f64) -> FuelTankProperties {
        let rho_fuel = self.emptying_density(r);
        let theta = self.emptying_theta_star(fuel_mass_kg, r);
        let (sin_theta, cos_theta) = (theta.sin(), theta.cos());
        let volume = 2.0 / 3.0
            * PI
            * r.powf(3.0)
            * (1.0 + 3.0 / 2.0 * cos_theta - 1.0 / 2.0 * cos_theta.powf(3.0));
        let delta_radius_k3 = if volume != 0.0 {
            PI * r.powf(4.0) / (4.0 * volume)
                * (2.0 * cos_theta.powf(2.0) - cos_theta.powf(4.0) - 1.0)
        } else {
            -r
        };
        let scale = 2.0 / 5.0 * PI * rho_fuel * r.powf(5.0);
        let axial = scale
            * (2.0 / 3.0 + 1.0 / 4.0 * cos_theta * sin_theta.powf(4.0)
                - 1.0 / 12.0 * ((3.0 * theta).cos() - 9.0 * cos_theta));
        let transverse = scale
            * (2.0 / 3.0 - 1.0 / 4.0 * cos_theta.powf(5.0)
                + 1.0 / 24.0 * ((3.0 * theta).cos() - 9.0 * cos_theta)
                + 5.0 / 4.0 * cos_theta
                + 1.0 / 8.0 * cos_theta * sin_theta.powf(4.0));
        FuelTankProperties {
            r_tct_t_m: self.r_tct_t_init_m + delta_radius_k3 * Vector3::z(),
            inertia_tank_pnt_t_t_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                transverse, transverse, axial,
            )),
        }
    }

    fn emptying_property_rates(&self, m: f64, m_dot: f64, r: f64) -> FuelTankPropertyRates {
        let rho_fuel = self.emptying_density(r);
        let theta = self.emptying_theta_star(m, r);
        let (sin_theta, cos_theta) = (theta.sin(), theta.cos());
        let (theta_dot, theta_ddot) = if m != self.prop_mass_init_kg {
            let theta_dot = -m_dot / (PI * rho_fuel * r.powf(3.0) * sin_theta.powf(3.0));
            // Assumes a constant mass-flow rate, as upstream.
            (
                theta_dot,
                -3.0 * theta_dot * theta_dot * cos_theta / sin_theta,
            )
        } else {
            (0.0, 0.0)
        };
        let scale = 2.0 / 5.0 * PI * rho_fuel * r.powf(5.0) * theta_dot;
        let axial = scale
            * (cos_theta.powf(2.0) * sin_theta.powf(3.0) - 1.0 / 4.0 * sin_theta.powf(5.0)
                + 1.0 / 4.0 * (3.0 * theta).sin()
                - 3.0 / 4.0 * sin_theta);
        let transverse = scale
            * (5.0 / 4.0 * sin_theta * cos_theta.powf(4.0)
                - 5.0 / 4.0 * sin_theta
                - 1.0 / 8.0 * (3.0 * theta).sin()
                + 3.0 / 8.0 * sin_theta
                + 1.0 / 2.0 * cos_theta.powf(2.0) * sin_theta.powf(3.0)
                - 1.0 / 8.0 * sin_theta.powf(5.0));
        let (r_prime, r_pprime) = if m != 0.0 {
            let q = sin_theta.powf(4.0);
            let q_dot = 4.0 * sin_theta.powf(3.0) * cos_theta * theta_dot;
            let q_ddot = 4.0
                * ((3.0 * sin_theta.powf(2.0) * cos_theta.powf(2.0) - sin_theta.powf(4.0))
                    * theta_dot
                    * theta_dot
                    + sin_theta.powf(3.0) * cos_theta * theta_ddot);
            let factor = -PI * r.powf(4.0) * rho_fuel / 4.0;
            (
                factor * (q_dot / m - q * m_dot / (m * m)) * Vector3::z(),
                factor
                    * (q_ddot / m - 2.0 * q_dot * m_dot / (m * m)
                        + 2.0 * q * m_dot * m_dot / (m * m * m))
                    * Vector3::z(),
            )
        } else {
            (Vector3::zeros(), Vector3::zeros())
        };
        FuelTankPropertyRates {
            r_prime_tct_t_mps: r_prime,
            r_pprime_tct_t_mps2: r_pprime,
            inertia_prime_tank_pnt_t_t_kg_m2ps: Matrix3::from_diagonal(&Vector3::new(
                transverse, transverse, axial,
            )),
        }
    }
}

/// User-settable parameters of [`FuelTank`].
#[derive(Clone, Debug)]
pub struct FuelTankConfig {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `setTankModel`.
    pub model: FuelTankModel,
    /// Upstream `dcm_TB`: DCM from the body frame to the tank frame (default
    /// identity).
    pub dcm_tb: Matrix3<f64>,
    /// Upstream `r_TB_B` \[m\]: tank frame origin relative to the body
    /// origin (default zero).
    pub r_tb_b_m: Vector3<f64>,
    /// Upstream `updateOnly` (default `true`): see the module docs.
    pub update_only: bool,
    /// Upstream `fuelLeakRate` \[kg/s\]: propellant leak, overridden by
    /// [`FuelTank::fuel_leak_rate_in_msg`] when connected (default 0).
    pub fuel_leak_rate_kgps: f64,
}

impl FuelTankConfig {
    /// A tank of `model` with upstream's defaults.
    pub fn new(name: impl Into<String>, model: FuelTankModel) -> Self {
        Self {
            name: name.into(),
            model,
            dcm_tb: Matrix3::identity(),
            r_tb_b_m: Vector3::zeros(),
            update_only: true,
            fuel_leak_rate_kgps: 0.0,
        }
    }
}

/// A thruster effector a [`FuelTank`] can feed (upstream
/// `FuelTank::addThrusterSet`, overloaded for both thruster effectors).
pub trait TankFedThruster {
    /// The propellant mass flow the thruster effector consumes, written at
    /// every integrator stage (upstream `stateDerivContribution(0)`).
    fn mass_flow_out_msg(&self) -> &Output<MassFlowRateMsg>;

    /// Gives the thruster the tank's propellant state (upstream: the tank
    /// sets the thruster's `fuelMass` for its blow-down model). The default
    /// ignores it.
    fn connect_fuel_tank(&mut self, _fuel_tank_out_msg: &Output<FuelTankMsg>) {}
}

/// How a tank state's propellant flow is shared out (upstream
/// `FuelTank::updateEffectorMassProps`).
#[derive(Clone, Debug)]
pub struct FuelTankDepletion {
    /// Upstream `fuelConsumption` \[kg/s\]: the leak plus the thrusters'
    /// consumption, zero once the tank and its particles are empty.
    pub fuel_consumption_kgps: f64,
    /// Upstream `tankFuelConsumption` \[kg/s\]: the tank's own share.
    pub tank_fuel_consumption_kgps: f64,
    /// The tank's own mass properties (upstream `effProps` of the tank).
    pub tank: DepletionMassProps,
    /// The flow assigned to each slosh particle, in push order.
    pub particle_flows: Vec<FuelSloshFlow>,
}

/// Propellant tank state effector (upstream `FuelTank`).
///
/// State layout: `[m]`, the tank's propellant mass \[kg\], followed by the
/// states of its slosh particles in push order.
pub struct FuelTank {
    pub config: FuelTankConfig,
    /// Upstream `fuelLeakRateInMsg`: (optional) leak rate, overriding
    /// [`FuelTankConfig::fuel_leak_rate_kgps`] when connected.
    pub fuel_leak_rate_in_msg: Input<MassFlowRateMsg>,
    /// The propellant consumption of each thruster effector the tank feeds,
    /// registered with [`FuelTank::add_thruster_set`] or
    /// [`FuelTank::add_thruster_mass_flow_input`].
    pub thruster_mass_flow_in_msgs: Vec<Input<MassFlowRateMsg>>,
    /// Upstream `fuelTankOutMsg`, written after every integration step.
    pub fuel_tank_out_msg: Output<FuelTankMsg>,
    /// Upstream `massState` \[kg\] of the last loaded state.
    pub fuel_mass_kg: f64,
    particles: Vec<Box<dyn FuelSlosh>>,
    /// The last loaded state, tank and particles.
    loaded_state: Vec<f64>,
    /// Upstream `emptyTankWarningPrinted`.
    empty_tank_warning_printed: Cell<bool>,
}

impl FuelTank {
    pub fn new(config: FuelTankConfig) -> Self {
        let model = config.model;
        Self {
            fuel_leak_rate_in_msg: Input::default(),
            thruster_mass_flow_in_msgs: Vec::new(),
            fuel_tank_out_msg: Output::new(FuelTankMsg {
                fuel_mass_kg: model.prop_mass_init_kg,
                fuel_mass_dot_kgps: 0.0,
                max_fuel_mass_kg: model.max_fuel_mass_kg,
            }),
            fuel_mass_kg: model.prop_mass_init_kg,
            particles: Vec::new(),
            loaded_state: Vec::new(),
            empty_tank_warning_printed: Cell::new(false),
            config,
        }
    }

    /// Upstream `pushFuelSloshParticle`: the tank takes `particle` over and
    /// drains it with its own propellant. Add the tank, not the particle, to
    /// the spacecraft.
    pub fn push_fuel_slosh_particle(&mut self, particle: impl FuelSlosh + 'static) {
        self.particles.push(Box::new(particle));
    }

    /// The slosh particles in push order; downcast one with
    /// [`StateEffector::as_any`].
    pub fn fuel_slosh_particles(&self) -> &[Box<dyn FuelSlosh>] {
        &self.particles
    }

    /// Upstream `addThrusterSet`: the tank supplies `thruster`'s propellant,
    /// reading its consumption at every integrator stage and giving it the
    /// tank's propellant state.
    pub fn add_thruster_set(&mut self, thruster: &mut impl TankFedThruster) {
        thruster
            .mass_flow_out_msg()
            .connect_to(self.add_thruster_mass_flow_input());
        thruster.connect_fuel_tank(&self.fuel_tank_out_msg);
    }

    /// Registers one more thruster consumption input, connectable to any
    /// [`MassFlowRateMsg`] output.
    pub fn add_thruster_mass_flow_input(&mut self) -> &mut Input<MassFlowRateMsg> {
        self.thruster_mass_flow_in_msgs.push(Input::default());
        self.thruster_mass_flow_in_msgs
            .last_mut()
            .expect("registered thruster mass flow input should be available")
    }

    /// The tank state's slice of each particle, in push order.
    fn particle_states<'a>(&self, effector_state: &'a [f64]) -> Vec<&'a [f64]> {
        let mut offset = 1;
        self.particles
            .iter()
            .map(|particle| {
                let slice = &effector_state[offset..offset + particle.state_len()];
                offset += particle.state_len();
                slice
            })
            .collect()
    }

    fn assert_state_length(&self, effector_state: &[f64]) {
        assert_eq!(
            effector_state.len(),
            self.state_len(),
            "fuel tank state length mismatch"
        );
    }

    /// The leak plus every connected thruster's consumption \[kg/s\].
    fn requested_consumption_kgps(&self) -> f64 {
        let leak = if self.fuel_leak_rate_in_msg.is_connected() {
            self.fuel_leak_rate_in_msg.read().mass_flow_rate_kgps
        } else {
            self.config.fuel_leak_rate_kgps
        };
        leak + self
            .thruster_mass_flow_in_msgs
            .iter()
            .map(|input| input.read().mass_flow_rate_kgps)
            .sum::<f64>()
    }

    /// Upstream `FuelTank::updateEffectorMassProps`: shares the propellant
    /// flow out between the tank and its particles at `effector_state`, and
    /// the tank's own mass properties.
    pub fn depletion(&self, effector_state: &[f64]) -> FuelTankDepletion {
        self.assert_state_length(effector_state);
        let tank_mass = effector_state[0].max(0.0);
        let particle_masses: Vec<f64> = self
            .particles
            .iter()
            .zip(self.particle_states(effector_state))
            .map(|(particle, state)| particle.fuel_mass_kg(state))
            .collect();
        let total_mass = tank_mass + particle_masses.iter().sum::<f64>();
        let update_only = self.config.update_only;
        let mut consumption = self.requested_consumption_kgps();

        if total_mass <= 0.0 {
            if consumption > 0.0 && !self.empty_tank_warning_printed.get() {
                log::warn!(
                    "FuelTank '{}': available propellant has reached zero. Fuel mass depletion is stopped.",
                    self.config.name
                );
                self.empty_tank_warning_printed.set(true);
            }
            consumption = 0.0;
        }
        let share = |mass: f64| {
            if total_mass > 0.0 {
                mass / total_mass * consumption
            } else {
                0.0
            }
        };
        let particle_flows = particle_masses
            .iter()
            .map(|&mass| FuelSloshFlow {
                fuel_mass_dot_kgps: -share(mass),
                omit_mass_rate_dynamics: update_only,
            })
            .collect();
        let tank_fuel_consumption = share(tank_mass);
        FuelTankDepletion {
            fuel_consumption_kgps: consumption,
            tank_fuel_consumption_kgps: tank_fuel_consumption,
            tank: self.tank_mass_props(tank_mass, -tank_fuel_consumption),
            particle_flows,
        }
    }

    /// Tank center of mass `r_TcB_B` \[m\] and the model properties and
    /// rates at `mass` draining at `mass_rate`.
    fn tank_geometry(
        &self,
        mass: f64,
        mass_rate: f64,
    ) -> (Vector3<f64>, FuelTankProperties, FuelTankPropertyRates) {
        let model = &self.config.model;
        let properties = model.properties(mass);
        let rates = model.property_rates(mass, mass_rate);
        let r_tcb_b = self.config.r_tb_b_m + self.config.dcm_tb.transpose() * properties.r_tct_t_m;
        (r_tcb_b, properties, rates)
    }

    /// Upstream `updateEffectorMassProps` and
    /// `updateRetainedMassPropertyDerivatives` for the tank alone.
    fn tank_mass_props(&self, mass: f64, mass_rate: f64) -> DepletionMassProps {
        let dcm_tb = self.config.dcm_tb;
        let (r, properties, rates) = self.tank_geometry(mass, mass_rate);
        let r_prime = dcm_tb.transpose() * rates.r_prime_tct_t_mps;
        let parallel_axis = r.dot(&r) * Matrix3::identity() - r * r.transpose();
        let retained = StateEffectorMassProps {
            mass_kg: mass,
            mass_dot_kgps: mass_rate,
            center_of_mass_body_m: r,
            center_of_mass_prime_body_mps: r_prime,
            inertia_about_point_b_body_kg_m2: dcm_tb.transpose()
                * properties.inertia_tank_pnt_t_t_kg_m2
                * dcm_tb
                + mass * parallel_axis,
            inertia_about_point_b_body_prime_kg_m2ps: dcm_tb.transpose()
                * rates.inertia_prime_tank_pnt_t_t_kg_m2ps
                * dcm_tb
                + mass_rate * parallel_axis
                + mass
                    * (2.0 * r.dot(&r_prime) * Matrix3::identity()
                        - r * r_prime.transpose()
                        - r_prime * r.transpose()),
        };
        // Upstream `hasMassPropertyRateDynamics`: with a flow, the generic
        // rate terms take only the mass rate, and only when not update-only;
        // the tank adds its own depletion terms in `update_contributions`.
        let dynamics = if mass_rate != 0.0 {
            StateEffectorMassProps {
                mass_dot_kgps: if self.config.update_only {
                    0.0
                } else {
                    mass_rate
                },
                center_of_mass_prime_body_mps: Vector3::zeros(),
                inertia_about_point_b_body_prime_kg_m2ps: Matrix3::zeros(),
                ..retained.clone()
            }
        } else {
            retained.clone()
        };
        DepletionMassProps { retained, dynamics }
    }
}

impl StateEffector for FuelTank {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn state_len(&self) -> usize {
        1 + self
            .particles
            .iter()
            .map(|particle| particle.state_len())
            .sum::<usize>()
    }

    /// Upstream `registerStates`: the initial propellant mass, then each
    /// particle's initial state.
    fn initial_state(&self) -> Vec<f64> {
        self.empty_tank_warning_printed.set(false);
        let mut state = vec![self.config.model.prop_mass_init_kg];
        for particle in &self.particles {
            state.extend(particle.initial_state());
        }
        state
    }

    fn load_state(&mut self, state: &[f64]) {
        self.assert_state_length(state);
        self.fuel_mass_kg = state[0];
        self.loaded_state.clear();
        self.loaded_state.extend_from_slice(state);
        let mut offset = 1;
        for particle in &mut self.particles {
            let len = particle.state_len();
            particle.load_state(&state[offset..offset + len]);
            offset += len;
        }
    }

    fn pre_integration(&mut self, current_sim_nanos: u64, dt_seconds: f64) {
        for particle in &mut self.particles {
            particle.pre_integration(current_sim_nanos, dt_seconds);
        }
    }

    fn begin_stage(&mut self, stage_nanos: u64) {
        for particle in &mut self.particles {
            particle.begin_stage(stage_nanos);
        }
    }

    /// Upstream `updateContributions`: with depletion dynamics on, the torque
    /// of the moving propellant center of mass and the changing inertia;
    /// then each particle's contributions.
    fn update_contributions(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        self.assert_state_length(effector_state);
        if !self.config.update_only {
            let depletion = self.depletion(effector_state);
            let mass = effector_state[0].max(0.0);
            let m_dot = -depletion.tank_fuel_consumption_kgps;
            let dcm_tb = self.config.dcm_tb;
            let (r, _, rates) = self.tank_geometry(mass, m_dot);
            let r_prime = dcm_tb.transpose() * rates.r_prime_tct_t_mps;
            let r_pprime = dcm_tb.transpose() * rates.r_pprime_tct_t_mps2;
            let omega = body_omega_radps;
            let inertia_prime = depletion
                .tank
                .retained
                .inertia_about_point_b_body_prime_kg_m2ps;
            back_sub.vec_rot += -mass * r.cross(&r_pprime)
                - mass * omega.cross(&r.cross(&r_prime))
                - m_dot * r.cross(&r_prime)
                - inertia_prime * omega;
        }
        for (particle, state) in self
            .particles
            .iter()
            .zip(self.particle_states(effector_state))
        {
            particle.update_contributions(state, body_omega_radps, gravity_body_mps2, back_sub);
        }
    }

    /// Upstream `computeDerivatives`: the tank mass drains at its share of
    /// the flow (not below empty), each particle at its own.
    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        let depletion = self.depletion(effector_state);
        let mut tank_consumption = depletion.tank_fuel_consumption_kgps;
        if effector_state[0] <= 0.0 && tank_consumption > 0.0 {
            tank_consumption = 0.0;
        }
        let mut derivatives = vec![-tank_consumption];
        for ((particle, state), flow) in self
            .particles
            .iter()
            .zip(self.particle_states(effector_state))
            .zip(depletion.particle_flows)
        {
            derivatives.extend(particle.compute_derivatives_with_flow(
                state,
                body_trans_accel_mps2,
                body_omega_dot_radps2,
                flow,
            ));
        }
        derivatives
    }

    /// The tank's and its particles' mass properties combined, with the
    /// rates the equations of motion use (see the module docs).
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        let depletion = self.depletion(effector_state);
        let mut parts = vec![depletion.tank.dynamics];
        for ((particle, state), flow) in self
            .particles
            .iter()
            .zip(self.particle_states(effector_state))
            .zip(depletion.particle_flows)
        {
            parts.push(particle.mass_properties_with_flow(state, flow).dynamics);
        }
        combine_mass_props(&parts)
    }

    /// Upstream `updateEnergyMomContributions` (angular momentum about B,
    /// the propellant moving with the body), plus the particles'.
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        self.assert_state_length(effector_state);
        let mass = effector_state[0].max(0.0);
        let (r, properties, _) = self.tank_geometry(mass, 0.0);
        let dcm_tb = self.config.dcm_tb;
        let inertia_tank_b = dcm_tb.transpose() * properties.inertia_tank_pnt_t_t_kg_m2 * dcm_tb;
        let r_dot = body_omega_radps.cross(&r);
        let mut momentum = inertia_tank_b * body_omega_radps + mass * r.cross(&r_dot);
        for (particle, state) in self
            .particles
            .iter()
            .zip(self.particle_states(effector_state))
        {
            momentum += particle.rotational_angular_momentum_body(state, body_omega_radps);
        }
        momentum
    }

    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        self.assert_state_length(effector_state);
        let mass = effector_state[0].max(0.0);
        let (r, properties, _) = self.tank_geometry(mass, 0.0);
        let dcm_tb = self.config.dcm_tb;
        let inertia_tank_b = dcm_tb.transpose() * properties.inertia_tank_pnt_t_t_kg_m2 * dcm_tb;
        let r_dot = body_omega_radps.cross(&r);
        let mut energy = 0.5 * body_omega_radps.dot(&(inertia_tank_b * body_omega_radps))
            + 0.5 * mass * r_dot.dot(&r_dot);
        for (particle, state) in self
            .particles
            .iter()
            .zip(self.particle_states(effector_state))
        {
            energy += particle.rotational_energy_j(state, body_omega_radps);
        }
        energy
    }

    /// Upstream `writeOutputMessages`, then the particles' outputs.
    fn write_outputs(&mut self, current_sim_nanos: u64, hub_state: &SpacecraftStateMsg) {
        let depletion = if self.loaded_state.is_empty() {
            self.depletion(&self.initial_state())
        } else {
            self.depletion(&self.loaded_state)
        };
        self.fuel_tank_out_msg.write(FuelTankMsg {
            fuel_mass_kg: depletion.tank.retained.mass_kg,
            fuel_mass_dot_kgps: depletion.tank.retained.mass_dot_kgps,
            max_fuel_mass_kg: self.config.model.max_fuel_mass_kg,
        });
        for particle in &mut self.particles {
            particle.write_outputs(current_sim_nanos, hub_state);
        }
    }

    /// Clamps masses integrated below zero (upstream does it in
    /// `updateEffectorMassProps` and `retrieveMassValue`), then lets each
    /// particle rewrite its state.
    fn modify_states(&mut self, current_sim_nanos: u64, effector_state: &mut [f64]) {
        self.assert_state_length(effector_state);
        effector_state[0] = effector_state[0].max(0.0);
        let mut offset = 1;
        for particle in &mut self.particles {
            let len = particle.state_len();
            let state = &mut effector_state[offset..offset + len];
            let index = particle.fuel_mass_index();
            state[index] = state[index].max(0.0);
            particle.modify_states(current_sim_nanos, state);
            offset += len;
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// One set of mass properties for several bodies: summed masses, rates and
/// inertias, and the mass-weighted center of mass and its rate.
fn combine_mass_props(parts: &[StateEffectorMassProps]) -> StateEffectorMassProps {
    let mass_kg: f64 = parts.iter().map(|part| part.mass_kg).sum();
    let mut combined = StateEffectorMassProps {
        mass_kg,
        ..StateEffectorMassProps::default()
    };
    let mut first_moment = Vector3::zeros();
    let mut first_moment_prime = Vector3::zeros();
    for part in parts {
        combined.mass_dot_kgps += part.mass_dot_kgps;
        first_moment += part.mass_kg * part.center_of_mass_body_m;
        first_moment_prime += part.mass_kg * part.center_of_mass_prime_body_mps;
        combined.inertia_about_point_b_body_kg_m2 += part.inertia_about_point_b_body_kg_m2;
        combined.inertia_about_point_b_body_prime_kg_m2ps +=
            part.inertia_about_point_b_body_prime_kg_m2ps;
    }
    if mass_kg > 0.0 {
        combined.center_of_mass_body_m = first_moment / mass_kg;
        combined.center_of_mass_prime_body_mps = first_moment_prime / mass_kg;
    }
    combined
}

#[cfg(test)]
// The truth values are upstream's, digit for digit.
#[allow(clippy::excessive_precision)]
mod tests {
    //! Ports of upstream `test_tank_models.py`, `test_mass_depletion.py` and
    //! `test_fuelTankConnections.py`.
    //!
    //! Not ported, as they have no counterpart here:
    //! - `test_attachedSloshRequiresTankFirst`,
    //!   `test_attachedSloshRequiresSpacecraftRegistration` and
    //!   `test_repeatedInitializationKeepsAttachedSloshValid`: the tank owns its
    //!   particles, so they cannot be added to the spacecraft separately or in
    //!   the wrong order;
    //! - `test_prescribedMotionRejectsVariableMassTank`: there is no
    //!   prescribed-motion effector;
    //! - `test_deprecatedPublicVariables`, `test_tank_retains_connections`,
    //!   `test_connection_survives_tank`, `test_connection_cycle_is_collectible`,
    //!   `test_null_connection_is_rejected` and
    //!   `test_dynamic_thruster_requires_tank_model`: Python accessor,
    //!   ownership and null-pointer checks; the Rust types make the null
    //!   connection and the missing tank model unrepresentable.
    //!
    //! `test_tankModelOutlivesPythonReference` keeps its depletion checks.
    //!
    //! Upstream's tank writes its message at the start of each tick, before
    //! the spacecraft integrates, so its sample `k` holds the state at tick
    //! `k − 1`; here the tank writes after the step, and sample `k − 1` is
    //! compared.

    use super::{FuelTank, FuelTankConfig, FuelTankModel, FuelTankShape};
    use crate::dynamics::linear_spring_mass_damper::{
        LinearSpringMassDamper, LinearSpringMassDamperConfig,
    };
    use crate::dynamics::spherical_pendulum::{SphericalPendulum, SphericalPendulumConfig};
    use crate::dynamics::state_effector_test_support::{
        GRAVITY_CASE_R_CN_N_M, GRAVITY_CASE_V_CN_N_MPS, HubSetup, add_point_mass_earth,
        run_logging_every_step, spacecraft_with_effectors,
    };
    use crate::dynamics::thruster_dynamic_effector::{
        ThrusterDynamicEffector, ThrusterDynamicEffectorConfig,
    };
    use crate::dynamics::thruster_state_effector::{
        ThrusterStateEffector, ThrusterStateEffectorConfig,
    };
    use crate::integrators::traits::DynamicObject;
    use crate::messages::{FuelTankMsg, MassFlowRateMsg, Output, ThrusterCommandMsg};
    use crate::simulation::Simulation;
    use crate::spacecraft::{IntegratedState, Spacecraft, StateEffector};
    use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    /// Pushes a slosh particle onto a tank.
    type PushParticle = Box<dyn Fn(&mut FuelTank)>;

    /// `numpy.testing.assert_allclose`: `|actual − desired| <= atol + rtol |desired|`.
    fn assert_allclose(actual: &[f64], desired: &[f64], rtol: f64, atol: f64, what: &str) {
        assert_eq!(actual.len(), desired.len(), "{what}: dimension mismatch");
        for (index, (a, d)) in actual.iter().zip(desired).enumerate() {
            assert!(
                (a - d).abs() <= atol + rtol * d.abs(),
                "{what}[{index}]: {a:.17e} vs {d:.17e}"
            );
        }
    }

    /// `pytest.approx` with its defaults: within `max(1e-6 |expected|, 1e-12)`.
    fn assert_approx(actual: f64, expected: f64, what: &str) {
        let tolerance = (1e-6 * expected.abs()).max(1e-12);
        assert!(
            (actual - expected).abs() <= tolerance,
            "{what}: {actual:e} vs {expected:e}"
        );
    }

    /// `numpy.isclose` with its default `atol` of 1e-8.
    fn is_close(actual: f64, desired: f64, rtol: f64) -> bool {
        (actual - desired).abs() <= 1e-8 + rtol * desired.abs()
    }

    fn row_major(matrix: &Matrix3<f64>) -> Vec<f64> {
        matrix.transpose().iter().copied().collect()
    }

    /// Upstream `trials` of `test_tank_models.py`: `(mFuel, mDotFuel)`.
    const TRIALS: [(f64, f64); 3] = [(0.0, 0.0), (10.0, -1.0), (5.0, -1.0)];

    /// One `test_tankModel*` case: for each trial, `computeTankProps` then
    /// `computeTankPropDerivs`, against the truth rows. `inertia_atol`
    /// selects the emptying test's absolute inertia check (rtol 1e-7).
    #[allow(clippy::too_many_arguments)]
    fn check_tank_model(
        shape: FuelTankShape,
        true_inertia: [[f64; 9]; 3],
        true_inertia_prime: [[f64; 9]; 3],
        true_r: [[f64; 3]; 3],
        true_r_prime: [[f64; 3]; 3],
        true_r_pprime: [[f64; 3]; 3],
        inertia_atol: bool,
    ) {
        let mut model = FuelTankModel::new(shape, 10.0);
        model.r_tct_t_init_m = Vector3::new(1.0, 1.0, 1.0);
        let accuracy = 1e-8;
        for (index, (mass, mass_dot)) in TRIALS.into_iter().enumerate() {
            let properties = model.properties(mass);
            let rates = model.property_rates(mass, mass_dot);
            let inertia = row_major(&properties.inertia_tank_pnt_t_t_kg_m2);
            if inertia_atol {
                assert_allclose(
                    &inertia,
                    &true_inertia[index],
                    1e-7,
                    accuracy,
                    "ITankPntT_T",
                );
            } else {
                assert_allclose(&inertia, &true_inertia[index], accuracy, 0.0, "ITankPntT_T");
            }
            assert_allclose(
                &row_major(&rates.inertia_prime_tank_pnt_t_t_kg_m2ps),
                &true_inertia_prime[index],
                accuracy,
                0.0,
                "IPrimeTankPntT_T",
            );
            assert_allclose(
                properties.r_tct_t_m.as_slice(),
                &true_r[index],
                accuracy,
                0.0,
                "r_TcT_T",
            );
            assert_allclose(
                rates.r_prime_tct_t_mps.as_slice(),
                &true_r_prime[index],
                accuracy,
                0.0,
                "rPrime_TcT_T",
            );
            assert_allclose(
                rates.r_pprime_tct_t_mps2.as_slice(),
                &true_r_pprime[index],
                accuracy,
                0.0,
                "rPPrime_TcT_T",
            );
        }
    }

    fn diagonal(a: f64, b: f64, c: f64) -> [f64; 9] {
        [a, 0.0, 0.0, 0.0, b, 0.0, 0.0, 0.0, c]
    }

    const STILL: [[f64; 3]; 3] = [[0.0; 3]; 3];
    const AT_INIT: [[f64; 3]; 3] = [[1.0; 3]; 3];

    #[test]
    fn tank_model_constant_volume() {
        check_tank_model(
            FuelTankShape::ConstantVolume {
                radius_tank_init_m: 5.0,
            },
            [
                diagonal(0.0, 0.0, 0.0),
                diagonal(100.0, 100.0, 100.0),
                diagonal(50.0, 50.0, 50.0),
            ],
            [
                diagonal(0.0, 0.0, 0.0),
                diagonal(-10.0, -10.0, -10.0),
                diagonal(-10.0, -10.0, -10.0),
            ],
            AT_INIT,
            STILL,
            STILL,
            false,
        );
    }

    #[test]
    fn tank_model_constant_density() {
        let i = 31.498026247371826;
        let i_prime = -10.499342082457275;
        check_tank_model(
            FuelTankShape::ConstantDensity {
                radius_tank_init_m: 5.0,
            },
            [
                diagonal(0.0, 0.0, 0.0),
                diagonal(100.0, 100.0, 100.0),
                diagonal(i, i, i),
            ],
            [
                diagonal(0.0, 0.0, 0.0),
                diagonal(
                    -16.666666666666668,
                    -16.666666666666668,
                    -16.666666666666668,
                ),
                diagonal(i_prime, i_prime, i_prime),
            ],
            AT_INIT,
            STILL,
            STILL,
            false,
        );
    }

    #[test]
    fn tank_model_emptying() {
        check_tank_model(
            FuelTankShape::Emptying {
                radius_tank_init_m: 5.0,
            },
            [
                diagonal(0.0, 0.0, 0.0),
                diagonal(100.0, 100.0, 100.0),
                diagonal(50.0, 50.0, 50.0),
            ],
            [
                diagonal(0.0, 0.0, 0.0),
                diagonal(0.0, 0.0, 0.0),
                diagonal(-8.75, -8.75, -12.5),
            ],
            [
                [1.0, 1.0, 1.0 - 5.0],
                [1.0, 1.0, 1.0],
                [1.0, 1.0, 1.0 - 15.0 / 8.0],
            ],
            [[0.0; 3], [0.0; 3], [0.0, 0.0, -3.0 / 8.0]],
            [[0.0; 3], [0.0; 3], [0.0, 0.0, -1.0 / 60.0]],
            true,
        );
    }

    /// Upstream `test_emptying_tank_derivatives_match_finite_differences`.
    #[test]
    fn emptying_tank_derivatives_match_finite_differences() {
        for fuel_mass_fraction in [0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 0.95] {
            let model = FuelTankModel::new(
                FuelTankShape::Emptying {
                    radius_tank_init_m: 0.5,
                },
                200.0,
            );
            let fuel_mass = fuel_mass_fraction * model.prop_mass_init_kg;
            let fuel_mass_rate = -1.7;
            let dt = 1.0e-2;

            let center = model.properties(fuel_mass).r_tct_t_m;
            let rates = model.property_rates(fuel_mass, fuel_mass_rate);
            let plus = model.properties(fuel_mass + fuel_mass_rate * dt);
            let minus = model.properties(fuel_mass - fuel_mass_rate * dt);

            let fd_center_rate = (plus.r_tct_t_m - minus.r_tct_t_m) / (2.0 * dt);
            let fd_center_second = (plus.r_tct_t_m - 2.0 * center + minus.r_tct_t_m) / (dt * dt);
            let fd_inertia_rate =
                (plus.inertia_tank_pnt_t_t_kg_m2 - minus.inertia_tank_pnt_t_t_kg_m2) / (2.0 * dt);
            let what = format!("fraction {fuel_mass_fraction}");
            assert_allclose(
                rates.r_prime_tct_t_mps.as_slice(),
                fd_center_rate.as_slice(),
                1.0e-5,
                0.0,
                &what,
            );
            assert_allclose(
                rates.r_pprime_tct_t_mps2.as_slice(),
                fd_center_second.as_slice(),
                1.0e-5,
                0.0,
                &what,
            );
            assert_allclose(
                rates.inertia_prime_tank_pnt_t_t_kg_m2ps.as_slice(),
                fd_inertia_rate.as_slice(),
                1.0e-5,
                0.0,
                &what,
            );
        }
    }

    #[test]
    fn tank_model_uniform_burn() {
        let t = 83.33333333333334;
        let half = 41.66666666666667;
        let t_prime = -8.3333333333334;
        check_tank_model(
            FuelTankShape::UniformBurn {
                radius_tank_init_m: 5.0,
                length_tank_m: 5.0,
            },
            [
                diagonal(0.0, 0.0, 0.0),
                diagonal(t, t, 125.0),
                diagonal(half, half, 62.5),
            ],
            [
                diagonal(0.0, 0.0, 0.0),
                diagonal(t_prime, t_prime, -12.5),
                diagonal(t_prime, t_prime, -12.5),
            ],
            AT_INIT,
            STILL,
            STILL,
            false,
        );
    }

    #[test]
    fn tank_model_centrifugal_burn() {
        let t = 83.33333333333334;
        let half = 57.291666666666671;
        check_tank_model(
            FuelTankShape::CentrifugalBurn {
                radius_tank_init_m: 5.0,
                length_tank_m: 5.0,
            },
            [
                diagonal(0.0, 0.0, 0.0),
                diagonal(t, t, 125.0),
                diagonal(half, half, 93.75),
            ],
            [
                diagonal(0.0, 0.0, 0.0),
                diagonal(-2.0833333333333335, -2.0833333333333335, 0.0),
                diagonal(
                    -8.3333333333333339,
                    -8.3333333333333339,
                    -12.500000000000002,
                ),
            ],
            AT_INIT,
            STILL,
            STILL,
            false,
        );
    }

    fn constant_volume(prop_mass_init_kg: f64, radius_tank_init_m: f64) -> FuelTankModel {
        FuelTankModel::new(
            FuelTankShape::ConstantVolume { radius_tank_init_m },
            prop_mass_init_kg,
        )
    }

    /// Upstream `_makeTank` of `test_fuelTankConnections.py`.
    fn connection_tank() -> FuelTank {
        FuelTank::new(FuelTankConfig::new("tank", constant_volume(12.0, 0.5)))
    }

    /// Upstream `test_thruster_mass_flow_after_scope_exit`: the tank drains at
    /// the thrust over `g0 Isp` of a thruster at full thrust.
    #[test]
    fn tank_reads_thruster_mass_flow() {
        let thrust = 9.80665;
        let specific_impulse = 10.0;
        let expected_rate = -thrust / (9.80665 * specific_impulse);

        let mut tank = connection_tank();
        let mut config = ThrusterDynamicEffectorConfig::simple(
            "thruster",
            Vector3::zeros(),
            Vector3::x(),
            thrust,
        );
        config.steady_isp_s = specific_impulse;
        let mut thruster = ThrusterDynamicEffector::new(config);
        let command = Output::new(ThrusterCommandMsg {
            on_time_s: 1.0,
            command_time_nanos: 0,
        });
        command.connect_to(&mut thruster.command_in);
        tank.add_thruster_set(&mut thruster);
        thruster.pre_integration(0);
        let depletion = tank.depletion(&tank.initial_state());
        assert_approx(
            depletion.tank.retained.mass_dot_kgps,
            expected_rate,
            "dynamic thruster",
        );

        let mut tank = connection_tank();
        let mut thrusters = ThrusterStateEffector::new("thrusters");
        let mut config = ThrusterStateEffectorConfig::new(
            "thruster",
            Vector3::zeros(),
            Vector3::x(),
            thrust,
            specific_impulse,
        );
        config.kappa_init = 1.0;
        thrusters.add_thruster(config);
        tank.add_thruster_set(&mut thrusters);
        thrusters.load_state(&[1.0]);
        thrusters.begin_stage(0);
        let depletion = tank.depletion(&tank.initial_state());
        assert_approx(
            depletion.tank.retained.mass_dot_kgps,
            expected_rate,
            "state thruster",
        );
    }

    /// Upstream `test_slosh_allocation_after_scope_exit`: a particle drains in
    /// proportion to its share of the propellant.
    #[test]
    fn tank_allocates_flow_to_slosh_particles() {
        let leak_rate = 0.2;
        let particle_mass = 4.0;
        let particles: [PushParticle; 2] = [
            Box::new(move |tank: &mut FuelTank| {
                let mut config = LinearSpringMassDamperConfig::new("particle");
                config.mass_init_kg = particle_mass;
                tank.push_fuel_slosh_particle(LinearSpringMassDamper::new(config));
            }),
            Box::new(move |tank: &mut FuelTank| {
                let mut config = SphericalPendulumConfig::new("pendulum");
                config.pendulum_radius_m = 0.3;
                config.mass_init_kg = particle_mass;
                tank.push_fuel_slosh_particle(SphericalPendulum::new(config));
            }),
        ];
        for push in particles {
            let mut tank = connection_tank();
            tank.config.fuel_leak_rate_kgps = leak_rate;
            push(&mut tank);
            let state = tank.initial_state();
            let depletion = tank.depletion(&state);
            let tank_mass = depletion.tank.retained.mass_kg;
            let expected_rate = -leak_rate * particle_mass / (tank_mass + particle_mass);
            let particle = &tank.fuel_slosh_particles()[0];
            assert_approx(
                particle.fuel_mass_kg(&state[1..]),
                particle_mass,
                "fuelMass",
            );
            let flow = depletion.particle_flows[0].fuel_mass_dot_kgps;
            assert_approx(flow, expected_rate, "fuelMassDot");
            assert_approx(
                depletion.tank.retained.mass_dot_kgps + flow,
                -leak_rate,
                "total rate",
            );
        }
    }

    /// A tank with no thrusters, leak or particles neither drains nor moves.
    #[test]
    fn idle_tank_has_no_mass_rate() {
        let tank = connection_tank();
        let props = tank.mass_properties(&tank.initial_state());
        assert_eq!(props.mass_kg, 12.0);
        assert_eq!(props.mass_dot_kgps, 0.0);
        assert_eq!(
            tank.compute_derivatives(&[12.0], Vector3::zeros(), Vector3::zeros()),
            vec![0.0]
        );
    }

    /// The slosh particle of upstream `_initializeAttachedSlosh`, at `offset`.
    fn attached_slosh(pendulum: bool, offset: Vector3<f64>, mass_kg: f64) -> PushParticle {
        Box::new(move |tank: &mut FuelTank| {
            if pendulum {
                let mut config = SphericalPendulumConfig::new("pendulum");
                config.pendulum_radius_m = 0.5;
                config.tank_center_body_m = offset - Vector3::new(0.5, 0.0, 0.0);
                config.mass_init_kg = mass_kg;
                tank.push_fuel_slosh_particle(SphericalPendulum::new(config));
            } else {
                let mut config = LinearSpringMassDamperConfig::new("particle");
                config.spring_constant_npm = 1.0;
                config.damping_ns_per_m = 0.0;
                config.equilibrium_position_body_m = offset;
                config.direction_body = Vector3::x();
                config.mass_init_kg = mass_kg;
                tank.push_fuel_slosh_particle(LinearSpringMassDamper::new(config));
            }
        })
    }

    /// Upstream `_initializeAttachedSlosh`: a 100 kg hub with a 20 kg tank and
    /// a 5 kg slosh particle at one offset, returning the spacecraft and the
    /// initial state.
    fn initialize_attached_slosh(
        pendulum: bool,
        update_only: bool,
        leak_rate: f64,
    ) -> (Spacecraft, IntegratedState) {
        let offset = Vector3::new(0.6, -0.3, 0.2);
        let mut model = constant_volume(20.0, 0.5);
        model.r_tct_t_init_m = Vector3::zeros();
        let mut config = FuelTankConfig::new("tank", model);
        config.r_tb_b_m = offset;
        config.fuel_leak_rate_kgps = leak_rate;
        config.update_only = update_only;
        let mut tank = FuelTank::new(config);
        attached_slosh(pendulum, offset, 5.0)(&mut tank);
        let effector_state = tank.initial_state();

        let hub = HubSetup {
            mass_kg: 100.0,
            center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(80.0, 90.0, 100.0)),
            r_cn_n_m: Vector3::zeros(),
            v_cn_n_mps: Vector3::zeros(),
            sigma_bn: Vector3::zeros(),
            omega_bn_b_radps: Vector3::new(0.1, -0.2, 0.15),
        };
        let spacecraft = spacecraft_with_effectors(&hub, 100_000_000, vec![Box::new(tank)]);
        let state = IntegratedState {
            position_wrt_central_body_m: spacecraft.config.initial_position_m,
            velocity_wrt_central_body_mps: spacecraft.config.initial_velocity_mps,
            sigma_bn: spacecraft.config.initial_sigma_bn,
            omega_radps: spacecraft.config.initial_omega_radps,
            effector_states: vec![effector_state],
        };
        (spacecraft, state)
    }

    /// Upstream `_hubStateDerivatives`: `[r̈_BN_N, ω̇_BN_B]`.
    fn hub_state_derivatives(spacecraft: &mut Spacecraft, state: &IntegratedState) -> Vec<f64> {
        let derivative = spacecraft.equations_of_motion(
            state,
            0,
            Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        );
        derivative
            .velocity_wrt_central_body_dot_mps2
            .iter()
            .chain(derivative.omega_dot_radps2.iter())
            .copied()
            .collect()
    }

    fn tank_of(spacecraft: &Spacecraft) -> &FuelTank {
        spacecraft.state_effectors[0]
            .as_any()
            .downcast_ref::<FuelTank>()
            .expect("first state effector is the tank")
    }

    /// Upstream `test_attachedSloshReportsComponentRates`.
    fn attached_slosh_reports_component_rates(pendulum: bool, update_only: bool) {
        let (mut spacecraft, state) = initialize_attached_slosh(pendulum, update_only, 1.0);
        let tank = tank_of(&spacecraft);
        let tank_state = &state.effector_states[0];
        let depletion = tank.depletion(tank_state);
        let particle = &tank.fuel_slosh_particles()[0];
        let flow = depletion.particle_flows[0];
        let particle_props = particle.mass_properties_with_flow(&tank_state[1..], flow);

        let tank_mass_rate = -0.8;
        let slosh_mass_rate = -0.2;
        assert!((depletion.tank.retained.mass_dot_kgps - tank_mass_rate).abs() <= 1.0e-15);
        assert!((flow.fuel_mass_dot_kgps - slosh_mass_rate).abs() <= 1.0e-15);

        // Upstream `updateSCMassProps` from the retained rates: `mDot_SC`
        // and `centerOfMassPrimeSC`.
        let hub_mass = 100.0;
        let parts = [&depletion.tank.retained, &particle_props.retained];
        let total_mass = hub_mass + parts.iter().map(|part| part.mass_kg).sum::<f64>();
        let mass_rate: f64 = parts.iter().map(|part| part.mass_dot_kgps).sum();
        assert!((mass_rate - -1.0).abs() <= 1.0e-15);
        let c = parts
            .iter()
            .map(|part| part.mass_kg * part.center_of_mass_body_m)
            .sum::<Vector3<f64>>()
            / total_mass;
        let c_prime = parts
            .iter()
            .map(|part| {
                part.mass_kg * part.center_of_mass_prime_body_mps
                    + part.mass_dot_kgps * part.center_of_mass_body_m
            })
            .sum::<Vector3<f64>>()
            / total_mass
            - mass_rate * c / total_mass;
        let offset = Vector3::new(0.6, -0.3, 0.2);
        let center_of_mass = 25.0 * offset / total_mass;
        let expected_c_prime = -offset / total_mass + center_of_mass / total_mass;
        assert_allclose(
            c_prime.as_slice(),
            expected_c_prime.as_slice(),
            1.0e-14,
            1.0e-16,
            "cPrime_B",
        );

        let offset_tilde = offset.cross_matrix();
        let expected_inertia_rate = -slosh_mass_rate * offset_tilde * offset_tilde;
        assert_allclose(
            particle_props
                .retained
                .inertia_about_point_b_body_prime_kg_m2ps
                .as_slice(),
            expected_inertia_rate.as_slice(),
            1.0e-14,
            1.0e-16,
            "slosh IEffPrimePntB_B",
        );

        let derivatives = hub_state_derivatives(&mut spacecraft, &state);
        if update_only {
            let (mut control, control_state) = initialize_attached_slosh(pendulum, true, 0.0);
            let control_derivatives = hub_state_derivatives(&mut control, &control_state);
            assert_allclose(
                &derivatives,
                &control_derivatives,
                0.0,
                1.0e-13,
                "hub derivatives",
            );
        } else {
            let (velocity, rate) = if pendulum {
                (
                    [
                        0.00527105039614227,
                        0.00102237938234717,
                        -0.00073415691683492,
                    ],
                    [
                        0.00337533072997688,
                        0.00425076546666240,
                        0.00457131641985391,
                    ],
                )
            } else {
                (
                    [
                        0.00440480657192980,
                        0.00124386704106702,
                        -0.00089226309734675,
                    ],
                    [
                        0.00337915307437880,
                        0.00416556431825436,
                        0.00444682568256779,
                    ],
                )
            };
            assert_allclose(
                &derivatives[..3],
                &velocity,
                1.0e-13,
                1.0e-15,
                "velocity derivative",
            );
            assert_allclose(
                &derivatives[3..],
                &rate,
                1.0e-13,
                1.0e-15,
                "rate derivative",
            );
        }
    }

    #[test]
    fn attached_spring_mass_damper_reports_component_rates_update_only() {
        attached_slosh_reports_component_rates(false, true);
    }

    #[test]
    fn attached_spring_mass_damper_reports_component_rates_with_depletion_dynamics() {
        attached_slosh_reports_component_rates(false, false);
    }

    #[test]
    fn attached_pendulum_reports_component_rates_update_only() {
        attached_slosh_reports_component_rates(true, true);
    }

    #[test]
    fn attached_pendulum_reports_component_rates_with_depletion_dynamics() {
        attached_slosh_reports_component_rates(true, false);
    }

    /// Upstream `thrFactory.create('TEST_Thruster', [1, 0, 0], [0, 1, 0])`,
    /// with the factory's defaults and no minimum pulse time.
    fn test_thruster_config() -> ThrusterDynamicEffectorConfig {
        let mut config = ThrusterDynamicEffectorConfig::simple(
            "TH1",
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            0.9,
        );
        config.steady_isp_s = 227.5;
        config.area_nozzle_m2 = 0.07;
        config
    }

    /// Upstream `test_massDepletionTest`: a 40 kg tank drained by one thruster
    /// firing for 9.9 s, over 10 minutes in orbit.
    fn mass_depletion_test(state_thruster: bool) {
        let mut model = constant_volume(40.0, 46.0 / 2.0 / 3.2808399 / 12.0);
        model.r_tct_t_init_m = Vector3::zeros();
        let mut tank = FuelTank::new(FuelTankConfig::new("tank", model));
        let command = Output::new(ThrusterCommandMsg {
            on_time_s: 9.9,
            command_time_nanos: 0,
        });

        let mut effectors: Vec<Box<dyn StateEffector>> = Vec::new();
        let mut dynamic_thruster = None;
        if state_thruster {
            let base = test_thruster_config();
            let mut thrusters = ThrusterStateEffector::new("Thrusters");
            let thruster = thrusters.add_thruster(ThrusterStateEffectorConfig {
                min_on_time_s: 0.0,
                area_nozzle_m2: base.area_nozzle_m2,
                ..ThrusterStateEffectorConfig::new(
                    "TH1",
                    base.position_m,
                    base.direction_body,
                    base.max_thrust_n,
                    base.steady_isp_s,
                )
            });
            command.connect_to(&mut thruster.command_in_msg);
            tank.add_thruster_set(&mut thrusters);
            effectors.push(Box::new(thrusters));
        } else {
            let mut thruster = ThrusterDynamicEffector::new(test_thruster_config());
            command.connect_to(&mut thruster.command_in);
            tank.add_thruster_set(&mut thruster);
            dynamic_thruster = Some(thruster);
        }
        let fuel_out = tank.fuel_tank_out_msg.clone();
        effectors.push(Box::new(tank));

        let hub = HubSetup {
            mass_kg: 750.0,
            center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
            r_cn_n_m: Vector3::from(GRAVITY_CASE_R_CN_N_M),
            v_cn_n_mps: Vector3::from(GRAVITY_CASE_V_CN_N_MPS),
            sigma_bn: Vector3::new(0.1, 0.2, -0.3),
            omega_bn_b_radps: Vector3::new(0.001, -0.01, 0.03),
        };
        let mut spacecraft = spacecraft_with_effectors(&hub, 100_000_000, effectors);
        if let Some(thruster) = dynamic_thruster {
            spacecraft.add_dynamic_effector(thruster);
        }
        add_point_mass_earth(&mut spacecraft);
        let state_out = spacecraft.state_out.clone();

        let mut fuel_log: Vec<FuelTankMsg> = Vec::new();
        run_logging_every_step(&mut spacecraft, 6_000, || fuel_log.push(fuel_out.read()));

        let final_state = state_out.read();
        let (true_pos, true_sigma) = if state_thruster {
            (
                [-6781593.400948599, 4946868.619447934, 5486741.690842073],
                [
                    0.14366625871003397,
                    -0.06488330854626220,
                    0.3032637107362375,
                ],
            )
        } else {
            (
                [
                    -6.7815933935338277e+06,
                    4.9468685979815889e+06,
                    5.4867416696776701e+06,
                ],
                [
                    1.4401781243854264e-01,
                    -6.4168702021364002e-02,
                    3.0166086824900967e-01,
                ],
            )
        };
        assert_allclose(
            final_state.position_m.as_slice(),
            &true_pos,
            1e-6,
            0.0,
            "position",
        );
        assert_allclose(
            final_state.sigma_bn.as_slice(),
            &true_sigma,
            1e-4,
            0.0,
            "attitude",
        );

        // Upstream samples 100 and the last (see the module note on timing).
        assert_allclose(
            &[fuel_log[99].fuel_mass_dot_kgps],
            &[-0.000403404216123],
            1e-3,
            0.0,
            "mass depletion ramped up",
        );
        assert_eq!(
            fuel_log[fuel_log.len() - 2].fuel_mass_dot_kgps,
            0.0,
            "mass depletion ramped down"
        );
    }

    #[test]
    fn mass_depletion_with_dynamic_thruster() {
        mass_depletion_test(false);
    }

    #[test]
    fn mass_depletion_with_thruster_state_effector() {
        mass_depletion_test(true);
    }

    /// A tank of `model` on upstream's default 1 kg hub, leaking at
    /// `leak_rate`, with an optional slosh particle; runs for `steps` steps
    /// of 0.1 s and returns the spacecraft and the tank message after each.
    fn run_leaky_tank(
        model: FuelTankModel,
        leak_rate: f64,
        leak_msg: Option<&Output<MassFlowRateMsg>>,
        add_particle: Option<PushParticle>,
        steps: usize,
    ) -> (Spacecraft, Vec<FuelTankMsg>) {
        let mut config = FuelTankConfig::new("tank", model);
        config.fuel_leak_rate_kgps = leak_rate;
        let mut tank = FuelTank::new(config);
        if let Some(leak) = leak_msg {
            leak.connect_to(&mut tank.fuel_leak_rate_in_msg);
        }
        if let Some(push) = add_particle {
            push(&mut tank);
        }
        let fuel_out = tank.fuel_tank_out_msg.clone();
        let hub = HubSetup {
            mass_kg: 1.0,
            inertia_kg_m2: Matrix3::identity(),
            ..HubSetup::hinged_body_hub()
        };
        let mut spacecraft = spacecraft_with_effectors(&hub, 100_000_000, vec![Box::new(tank)]);
        let mut log = Vec::new();
        run_logging_every_step(&mut spacecraft, steps, || log.push(fuel_out.read()));
        (spacecraft, log)
    }

    /// Upstream `test_leakyTank`: a leaking tank sheds `ṁ t` from the vehicle,
    /// whatever share of it fuel slosh carries.
    #[test]
    fn leaky_tank_sheds_its_leak_whatever_the_slosh_share() {
        for slosh_mass in [0.0, 10.0, 40.0] {
            let initial_fuel_mass = 40.0;
            let leak_rate = 1e-5;
            let stop_time = 1000.0;
            let particle: Option<PushParticle> = (slosh_mass > 0.0).then(|| {
                Box::new(move |tank: &mut FuelTank| {
                    let mut config = LinearSpringMassDamperConfig::new("particle");
                    config.spring_constant_npm = 100.0;
                    config.damping_ns_per_m = 1.0;
                    config.direction_body = Vector3::x();
                    config.mass_init_kg = slosh_mass;
                    tank.push_fuel_slosh_particle(LinearSpringMassDamper::new(config));
                }) as PushParticle
            });
            let total_propellant = initial_fuel_mass + slosh_mass;
            let (spacecraft, log) = run_leaky_tank(
                constant_volume(initial_fuel_mass, 0.0),
                leak_rate,
                None,
                particle,
                10_000,
            );

            let expected_tank_mass_rate = -leak_rate * initial_fuel_mass / total_propellant;
            for sample in &log {
                assert_allclose(
                    &[sample.fuel_mass_dot_kgps],
                    &[expected_tank_mass_rate],
                    1e-6,
                    1e-8,
                    "tank mass rate",
                );
            }
            let expected_tank_mass =
                initial_fuel_mass * (1.0 - leak_rate * stop_time / total_propellant);
            let final_tank_mass = log[log.len() - 1].fuel_mass_kg;
            assert!(
                is_close(final_tank_mass, expected_tank_mass, 1e-6),
                "tank mass"
            );

            let tank = tank_of(&spacecraft);
            let final_slosh_mass = tank
                .fuel_slosh_particles()
                .first()
                .map(|particle| {
                    particle
                        .as_any()
                        .downcast_ref::<LinearSpringMassDamper>()
                        .expect("spring-mass-damper particle")
                        .mass_kg
                })
                .unwrap_or(0.0);
            let shed = total_propellant - (tank.fuel_mass_kg + final_slosh_mass);
            assert!(
                is_close(shed, leak_rate * stop_time, 1e-6),
                "shed propellant {shed:e}"
            );
        }
    }

    /// Upstream `test_leakyTankInputMessageOverridesSetter`.
    #[test]
    fn leak_rate_message_overrides_the_configured_rate() {
        let initial_fuel_mass = 40.0;
        let message_leak_rate = 2.0e-5;
        let leak = Output::new(MassFlowRateMsg {
            mass_flow_rate_kgps: message_leak_rate,
        });
        let (_, log) = run_leaky_tank(
            constant_volume(initial_fuel_mass, 0.0),
            1.0e-5,
            Some(&leak),
            None,
            10_000,
        );
        for sample in &log {
            assert_allclose(
                &[sample.fuel_mass_dot_kgps],
                &[-message_leak_rate],
                1e-6,
                1e-8,
                "mass rate",
            );
        }
        assert!(is_close(
            log[log.len() - 1].fuel_mass_kg,
            initial_fuel_mass - 1000.0 * message_leak_rate,
            1e-6
        ));
    }

    /// The five tank models with upstream `_configureTankGeometry`'s unit
    /// dimensions.
    fn unit_tank_models(prop_mass_init_kg: f64) -> Vec<FuelTankModel> {
        [
            FuelTankShape::ConstantVolume {
                radius_tank_init_m: 1.0,
            },
            FuelTankShape::ConstantDensity {
                radius_tank_init_m: 1.0,
            },
            FuelTankShape::Emptying {
                radius_tank_init_m: 1.0,
            },
            FuelTankShape::UniformBurn {
                radius_tank_init_m: 1.0,
                length_tank_m: 1.0,
            },
            FuelTankShape::CentrifugalBurn {
                radius_tank_init_m: 1.0,
                length_tank_m: 1.0,
            },
        ]
        .into_iter()
        .map(|shape| FuelTankModel::new(shape, prop_mass_init_kg))
        .collect()
    }

    /// Upstream `test_leakyTankRunsOutOfFuel`, and the depletion checks of
    /// `test_tankModelOutlivesPythonReference`: every model empties to
    /// exactly zero and stops depleting.
    #[test]
    fn leaky_tank_runs_out_of_fuel() {
        let leak_rate = 1.0e-5;
        for model in unit_tank_models(1.0e-4) {
            let (_, log) = run_leaky_tank(model, leak_rate, None, None, 200);
            let what = format!("{:?}", model.shape);
            let tolerance = 1e-14;
            for sample in &log {
                assert!(sample.fuel_mass_kg.is_finite(), "{what}");
                assert!(sample.fuel_mass_dot_kgps.is_finite(), "{what}");
                assert!(sample.fuel_mass_kg >= -tolerance, "{what}");
            }
            let last = &log[log.len() - 1];
            assert!(
                last.fuel_mass_kg.abs() <= tolerance,
                "{what}: {}",
                last.fuel_mass_kg
            );
            assert!(last.fuel_mass_dot_kgps.abs() <= tolerance, "{what}");
            assert!(
                log.iter()
                    .any(|sample| is_close(sample.fuel_mass_dot_kgps, -leak_rate, 1e-6)),
                "{what}: never leaked at the configured rate"
            );
        }
    }

    /// Upstream `test_attachedSloshMassClampsAtEmpty`: a depletion step that
    /// crosses zero leaves the tank and its particle exactly empty.
    #[test]
    fn attached_slosh_mass_clamps_at_empty() {
        for pendulum in [false, true] {
            let mut config = FuelTankConfig::new("tank", constant_volume(1.0e-4, 0.5));
            config.fuel_leak_rate_kgps = 1.0e-4;
            let mut tank = FuelTank::new(config);
            attached_slosh(pendulum, Vector3::zeros(), 2.5e-5)(&mut tank);
            let hub = HubSetup {
                mass_kg: 100.0,
                inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(80.0, 90.0, 100.0)),
                ..HubSetup::hinged_body_hub()
            };
            let mut spacecraft =
                spacecraft_with_effectors(&hub, 1_000_000_000, vec![Box::new(tank)]);
            run_logging_every_step(&mut spacecraft, 3, || {});

            let tank = tank_of(&spacecraft);
            let particle = &tank.fuel_slosh_particles()[0];
            let particle_mass = if pendulum {
                particle
                    .as_any()
                    .downcast_ref::<SphericalPendulum>()
                    .expect("pendulum")
                    .mass_kg
            } else {
                particle
                    .as_any()
                    .downcast_ref::<LinearSpringMassDamper>()
                    .expect("spring-mass-damper")
                    .mass_kg
            };
            let tolerance = 1.0e-14;
            assert!(
                tank.fuel_mass_kg.abs() <= tolerance,
                "tank {}",
                tank.fuel_mass_kg
            );
            assert!(particle_mass.abs() <= tolerance, "particle {particle_mass}");
            let final_state = {
                let mut state = vec![tank.fuel_mass_kg];
                state.extend(particle.initial_state());
                let index = 1 + particle.fuel_mass_index();
                state[index] = particle_mass;
                state
            };
            let props = particle.mass_properties(&final_state[1..]);
            assert!(props.mass_kg.abs() <= tolerance);
            let state = IntegratedState {
                position_wrt_central_body_m: Vector3::zeros(),
                velocity_wrt_central_body_mps: Vector3::zeros(),
                sigma_bn: Vector3::zeros(),
                omega_radps: Vector3::zeros(),
                effector_states: vec![final_state],
            };
            let derivatives = hub_state_derivatives(&mut spacecraft, &state);
            assert!(derivatives.iter().all(|value| value.is_finite()));
        }
    }

    /// The offset, rotated, depleting emptying tank of upstream
    /// `test_depletionTorque*`, on a 500 kg hub at rest.
    fn mounted_emptying_tank(prop_mass_init_kg: f64) -> (FuelTank, Matrix3<f64>, Vector3<f64>) {
        let dcm_tb = mrp_to_dcm(Vector3::new(0.15, -0.25, 0.1));
        let r_tb_b = Vector3::new(0.6, 0.0, 0.4);
        let model = FuelTankModel::new(
            FuelTankShape::Emptying {
                radius_tank_init_m: 0.5,
            },
            prop_mass_init_kg,
        );
        let mut config = FuelTankConfig::new("tank", model);
        config.dcm_tb = dcm_tb;
        config.r_tb_b_m = r_tb_b;
        config.update_only = false;
        (FuelTank::new(config), dcm_tb, r_tb_b)
    }

    fn resting_hub() -> HubSetup {
        HubSetup {
            mass_kg: 500.0,
            inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(300.0, 300.0, 300.0)),
            ..HubSetup::hinged_body_hub()
        }
    }

    /// The hub rate must lie along `−(r_TB_B × [TB]ᵀ k̂3)` and have
    /// `magnitude` (rtol 1e-5).
    fn assert_depletion_spin(
        omega: Vector3<f64>,
        dcm_tb: Matrix3<f64>,
        r_tb_b: Vector3<f64>,
        magnitude: f64,
    ) {
        let k3_b = dcm_tb.transpose() * Vector3::z();
        let expected_axis = r_tb_b.cross(&k3_b).normalize();
        assert_allclose(
            (omega / omega.norm()).as_slice(),
            (-expected_axis).as_slice(),
            1e-7,
            1e-6,
            "spin axis",
        );
        assert_allclose(&[omega.norm()], &[magnitude], 1e-5, 0.0, "spin rate");
    }

    /// Upstream `test_depletionTorqueFollowsTankMounting`.
    #[test]
    fn depletion_torque_follows_tank_mounting() {
        let (mut tank, dcm_tb, r_tb_b) = mounted_emptying_tank(200.0);
        tank.config.fuel_leak_rate_kgps = 1.0;
        let mut spacecraft =
            spacecraft_with_effectors(&resting_hub(), 100_000_000, vec![Box::new(tank)]);
        let state_out = spacecraft.state_out.clone();
        run_logging_every_step(&mut spacecraft, 200, || {});
        assert_depletion_spin(
            state_out.read().omega_radps,
            dcm_tb,
            r_tb_b,
            2.6449538196e-04,
        );
    }

    /// Upstream `test_depletionTorqueUsesCurrentMassFlowRate`: a leak ramped
    /// through the input message, `ṁ(t) = 1 + 0.5 t`.
    #[test]
    fn depletion_torque_uses_current_mass_flow_rate() {
        let (mut tank, dcm_tb, r_tb_b) = mounted_emptying_tank(300.0);
        let leak = Output::new(MassFlowRateMsg::default());
        leak.connect_to(&mut tank.fuel_leak_rate_in_msg);
        let mut spacecraft =
            spacecraft_with_effectors(&resting_hub(), 100_000_000, vec![Box::new(tank)]);
        let state_out = spacecraft.state_out.clone();
        let dt = 0.1;
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("spacecraft", &mut spacecraft, 100_000_000, 0);
            sim.run_for(0);
            for step in 0..150 {
                leak.write(MassFlowRateMsg {
                    mass_flow_rate_kgps: 1.0 + 0.5 * step as f64 * dt,
                });
                sim.run_for(100_000_000);
            }
        }
        assert_depletion_spin(
            state_out.read().omega_radps,
            dcm_tb,
            r_tb_b,
            1.5818171569e-03,
        );
    }
}
