//! Atmospheric wind models: port of upstream Basilisk
//! `simulation/environment/_GeneralModuleFiles/windBase`, the shared plumbing
//! of every wind model, and the models built on it.
//!
//! [`WindBase`] is the module. It reads the spacecraft and planet states,
//! forms the spacecraft position relative to the planet `r_BP_N` and the
//! velocity of a rigidly co-rotating atmosphere `omega_planet x r_BP_N`, and
//! hands both to a [`WindModel`], which returns the [`WindMsg`]. A new wind
//! model is a type implementing [`WindModel`], used as `WindBase<NewModel>`;
//! see [`zero_wind_model`].
//!
//! The planet angular velocity is either set by hand
//! ([`WindBase::set_planet_omega_n`], Earth's rate by default) or derived from
//! the planet orientation and its rate in the planet message (the default,
//! upstream's "SPICE omega").
//!
//! # Differences from upstream
//!
//! * Like the atmosphere models here, an instance serves one spacecraft
//!   (`input_state_msg` / `output_wind_msg`). Upstream's
//!   `addSpacecraftToModel` serves several from one instance; create one
//!   instance per spacecraft instead.
//! * No message headers: upstream zeroes the output while a linked input is
//!   unwritten. The stand-in is "connected ⇒ written at the current update",
//!   so the output is zero while an input is unconnected.
//! * Upstream's optional extrapolation of the spacecraft state to the middle
//!   of the next integration step (`setExtrapolateScStateToStepMidpoint`) is
//!   not ported: it is driven by the message write times, which the ports do
//!   not carry. The atmosphere models here do not extrapolate either.
//! * The epoch input (`epochInMsg`) is replaced by the simulation context,
//!   which [`WindModel::evaluate_wind_model`] receives, so a time-dependent
//!   model reads [`SimulationContext::current_epoch`].

pub mod zero_wind_model;

use std::fmt;

use nalgebra::{Matrix3, Vector3};

use crate::messages::{Input, Output, PlanetStateMsg, SpacecraftStateMsg, WindMsg};
use crate::utilities::astro_constants::OMEGA_EARTH;
use crate::{Module, SimulationContext};

/// What a concrete wind model adds to [`WindBase`]: upstream's pure virtual
/// `WindBase::evaluateWindModel`.
pub trait WindModel: Send {
    /// The local air velocity at the spacecraft.
    ///
    /// * `r_bp_n_m` \[m\]: spacecraft position relative to the planet,
    ///   inertial components (upstream `r_BP_N`).
    /// * `v_corotating_air_n_mps` \[m/s\]: velocity of the co-rotating
    ///   atmosphere at that position, inertial components (upstream
    ///   `v_corotatingAir_N`).
    /// * `context`: the current time and epoch (upstream `currentTime` and the
    ///   module epoch).
    fn evaluate_wind_model(
        &mut self,
        r_bp_n_m: Vector3<f64>,
        v_corotating_air_n_mps: Vector3<f64>,
        context: &SimulationContext,
    ) -> WindMsg;
}

/// User-settable parameters of [`WindBase`].
#[derive(Clone, Debug)]
pub struct WindBaseConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
}

/// A configuration problem upstream's `Reset` reports with `bskError`; each
/// is logged at [`Module::init`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WindConfigError {
    /// An input the model needs is not connected: upstream's "Wind model has
    /// no spacecraft added to it" and "planetPosInMsg is not linked".
    InputNotConnected(&'static str),
}

impl fmt::Display for WindConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InputNotConnected(port) => write!(f, "wind model: {port} is not connected"),
        }
    }
}

/// The wind-model module (upstream `WindBase`), generic over the model that
/// supplies the wind on top of the co-rotating atmosphere.
#[derive(Clone, Debug)]
pub struct WindBase<M> {
    /// Module configuration.
    pub config: WindBaseConfig,
    /// The model that turns position and co-rotation into a wind.
    pub model: M,
    /// Upstream `scStateInMsgs`: the spacecraft state (one spacecraft).
    pub input_state_msg: Input<SpacecraftStateMsg>,
    /// Upstream `planetPosInMsg`: the planet position and orientation.
    pub input_planet_msg: Input<PlanetStateMsg>,
    /// Upstream `envOutMsgs`: the air velocity at the spacecraft.
    pub output_wind_msg: Output<WindMsg>,
    /// Upstream `planetOmega_N` \[rad/s\]: the manually set planet angular
    /// velocity, inertial components.
    planet_omega_n_radps: Vector3<f64>,
    /// Upstream `useSpiceOmega`: derive the angular velocity from the planet
    /// message instead of using `planet_omega_n_radps`.
    use_spice_omega: bool,
    /// Upstream `spiceOmega_N` \[rad/s\]: the angular velocity last derived
    /// from the planet message.
    spice_omega_n_radps: Vector3<f64>,
}

impl<M: WindModel> WindBase<M> {
    /// A wind module running `model`, with Earth's rotation rate as the
    /// manual planet angular velocity and the planet-message rate selected.
    pub fn new(config: WindBaseConfig, model: M) -> Self {
        Self {
            config,
            model,
            input_state_msg: Input::default(),
            input_planet_msg: Input::default(),
            output_wind_msg: Output::default(),
            planet_omega_n_radps: Vector3::new(0.0, 0.0, OMEGA_EARTH),
            use_spice_omega: true,
            spice_omega_n_radps: Vector3::zeros(),
        }
    }

    /// Upstream `getPlanetOmega_N` \[rad/s\]: the manually set planet angular
    /// velocity, inertial components.
    pub fn planet_omega_n(&self) -> Vector3<f64> {
        self.planet_omega_n_radps
    }

    /// Upstream `setPlanetOmega_N`: sets the planet angular velocity \[rad/s\]
    /// and selects it over the planet-message rate. Call
    /// [`Self::set_use_spice_omega`] afterwards to select the message rate
    /// again.
    ///
    /// # Panics
    ///
    /// If a component is NaN or infinite (upstream `bskError`).
    pub fn set_planet_omega_n(&mut self, omega_n_radps: Vector3<f64>) {
        assert!(
            omega_n_radps.iter().all(|value| value.is_finite()),
            "planet angular velocity must be finite, got {omega_n_radps:?}"
        );
        self.planet_omega_n_radps = omega_n_radps;
        self.use_spice_omega = false;
    }

    /// Upstream `getUseSpiceOmegaFlag`: whether the planet-message rate is
    /// selected.
    pub fn use_spice_omega(&self) -> bool {
        self.use_spice_omega
    }

    /// Upstream `setUseSpiceOmegaFlag`: select the planet-message rate
    /// (`true`) or the manual one (`false`).
    pub fn set_use_spice_omega(&mut self, use_spice: bool) {
        self.use_spice_omega = use_spice;
    }

    /// Upstream `Reset`'s checks, each a `bskError` upstream.
    pub fn configuration_errors(&self) -> Vec<WindConfigError> {
        let mut errors = Vec::new();
        if !self.input_state_msg.is_connected() {
            errors.push(WindConfigError::InputNotConnected("input_state_msg"));
        }
        if !self.input_planet_msg.is_connected() {
            errors.push(WindConfigError::InputNotConnected("input_planet_msg"));
        }
        errors
    }

    /// Upstream `getSelectedOmega` \[rad/s\]: the planet-message rate when it
    /// is selected and the planet message is written, else the manual one.
    fn selected_omega_n(&self) -> Vector3<f64> {
        if self.use_spice_omega && self.input_planet_msg.is_connected() {
            self.spice_omega_n_radps
        } else {
            self.planet_omega_n_radps
        }
    }

    /// The air velocity at the spacecraft, or `None` while an input is
    /// missing (upstream `readMessages` failing).
    fn local_wind(&mut self, context: &SimulationContext) -> Option<WindMsg> {
        if !self.input_planet_msg.is_connected() {
            return None;
        }
        let planet = self.input_planet_msg.read();
        if self.use_spice_omega {
            self.spice_omega_n_radps = spice_omega_n(&planet);
        }
        if !self.input_state_msg.is_connected() {
            return None;
        }
        let spacecraft = self.input_state_msg.read();

        let omega_planet_n = self.selected_omega_n();
        let r_bp_n_m = spacecraft.position_m - planet.position_inertial_m;
        let v_corotating_air_n_mps = omega_planet_n.cross(&r_bp_n_m);
        Some(
            self.model
                .evaluate_wind_model(r_bp_n_m, v_corotating_air_n_mps, context),
        )
    }
}

/// Upstream `updatePlanetOmegaFromSpice`: the inertial planet angular
/// velocity from the planet orientation `C = [PN]` and its rate.
///
/// The kinematic equation gives `[omega_PN_P x] = -C_dot C^T`, and
/// `omega_N = C^T omega_PN_P`. A planet message without orientation stands
/// for upstream's zeroed payload, whose rate is zero.
fn spice_omega_n(planet: &PlanetStateMsg) -> Vector3<f64> {
    let (c, c_dot) = planet.orientation.as_ref().map_or_else(
        || (Matrix3::identity(), Matrix3::zeros()),
        |orientation| {
            (
                orientation.inertial_to_fixed,
                orientation.inertial_to_fixed_dot,
            )
        },
    );
    let skew_p = -c_dot * c.transpose();
    let omega_p = Vector3::new(skew_p[(2, 1)], skew_p[(0, 2)], skew_p[(1, 0)]);
    c.transpose() * omega_p
}

impl<M: WindModel> Module for WindBase<M> {
    fn init(&mut self) {
        for error in self.configuration_errors() {
            log::error!("{}: {error}", self.config.name);
        }
        self.output_wind_msg.write(WindMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        // Upstream zeroes the output when its inputs cannot be read, so that
        // no stale wind is left behind.
        let wind = self.local_wind(context).unwrap_or_default();
        self.output_wind_msg.write(wind);
    }
}
