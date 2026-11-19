//! Charging of a servicer and a target spacecraft: port of upstream Basilisk
//! `simulation/environment/spacecraftChargingDynamics`.
//!
//! Each spacecraft is a capacitor charged by the ambient plasma electrons
//! and ions, by photoelectrons, and by an electron beam the servicer fires
//! at the target, `dphi/dt = I_total / C`. The two potentials are integrated
//! with RK4 (upstream's `svIntegratorRK4`) over the time since the previous
//! update, with the inputs read at this update held over the step; the
//! currents are then re-evaluated at the new potentials and published.
//!
//! The electron beam input is optional. Without it there is no beam current.
//!
//! # Differences from upstream
//!
//! * No message headers: the required inputs are reported at `init` when
//!   unconnected, and the beam input is read whenever it is connected
//!   ("connected ⇒ written at the current update"). Upstream raises
//!   `bskError` at every update while a required input is unlinked or
//!   unwritten.
//! * The plasma and capacitor parameters are a config struct checked by
//!   [`SpacecraftChargingDynamics::new`] instead of setters with `assert`s.
//! * Upstream's own `DynamicObject` integrator is the crate's [`Rk4`] over a
//!   [`DynamicObject`] of the two potentials.

use std::f64::consts::PI;
use std::fmt;

use hifitime::Epoch;
use nalgebra::Vector2;

use crate::integrators::{DynamicObject, Integrator, Rk4};
use crate::messages::{
    CurrentMsg, ElectronBeamMsg, Input, Output, ProjectedAreaMsg, SpacecraftStateMsg, VoltMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::astro_constants::{MASS_ELECTRON, MASS_PROTON, Q_CHARGE};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`SpacecraftChargingDynamics`] (upstream's
/// setters), with upstream's defaults.
#[derive(Clone, Debug)]
pub struct SpacecraftChargingDynamicsConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `servicerPotentialInit` \[V\].
    pub servicer_potential_init_v: f64,
    /// Upstream `targetPotentialInit` \[V\].
    pub target_potential_init_v: f64,
    /// Upstream `servicerCapacitance` \[F\], positive.
    pub servicer_capacitance_f: f64,
    /// Upstream `targetCapacitance` \[F\], positive.
    pub target_capacitance_f: f64,
    /// Upstream `fluxPhotoelectrons` \[A/m^2\]: photoelectron current density,
    /// positive.
    pub flux_photoelectrons_a_per_m2: f64,
    /// Upstream `tempPhotoelectrons` \[eV\], positive.
    pub temp_photoelectrons_ev: f64,
    /// Upstream `tempElectrons` \[eV\]: plasma electron temperature, positive.
    pub temp_electrons_ev: f64,
    /// Upstream `densityElectrons` \[m^-3\]: plasma electron density,
    /// positive.
    pub density_electrons_per_m3: f64,
    /// Upstream `tempIons` \[eV\]: plasma ion temperature, positive.
    pub temp_ions_ev: f64,
    /// Upstream `densityIons` \[m^-3\]: plasma ion density, positive.
    pub density_ions_per_m3: f64,
    /// Upstream `bulkVelocityIons` \[m/s\]: bulk plasma ion speed, not
    /// negative.
    pub bulk_velocity_ions_mps: f64,
}

impl Default for SpacecraftChargingDynamicsConfig {
    fn default() -> Self {
        Self {
            name: "spacecraftChargingDynamics".to_string(),
            servicer_potential_init_v: 0.0,
            target_potential_init_v: 0.0,
            servicer_capacitance_f: 1e-9,
            target_capacitance_f: 1e-9,
            flux_photoelectrons_a_per_m2: 1e-6,
            temp_photoelectrons_ev: 2.0,
            temp_electrons_ev: 2.0,
            density_electrons_per_m3: 950_000.0,
            temp_ions_ev: 2.0,
            density_ions_per_m3: 950_000.0,
            bulk_velocity_ions_mps: 400_000.0,
        }
    }
}

impl SpacecraftChargingDynamicsConfig {
    /// Upstream's setter `assert`s. Returns a description of the first
    /// violation, or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        let positive = [
            ("servicer_capacitance_f", self.servicer_capacitance_f),
            ("target_capacitance_f", self.target_capacitance_f),
            (
                "flux_photoelectrons_a_per_m2",
                self.flux_photoelectrons_a_per_m2,
            ),
            ("temp_photoelectrons_ev", self.temp_photoelectrons_ev),
            ("temp_electrons_ev", self.temp_electrons_ev),
            ("density_electrons_per_m3", self.density_electrons_per_m3),
            ("temp_ions_ev", self.temp_ions_ev),
            ("density_ions_per_m3", self.density_ions_per_m3),
        ];
        for (name, value) in positive {
            if !(value.is_finite() && value > 0.0) {
                return Err(format!("{name} must be finite and positive, got {value}"));
            }
        }
        if !(self.bulk_velocity_ions_mps.is_finite() && self.bulk_velocity_ions_mps >= 0.0) {
            return Err(format!(
                "bulk_velocity_ions_mps must be finite and >= 0, got {}",
                self.bulk_velocity_ions_mps
            ));
        }
        for (name, value) in [
            ("servicer_potential_init_v", self.servicer_potential_init_v),
            ("target_potential_init_v", self.target_potential_init_v),
        ] {
            if !value.is_finite() {
                return Err(format!("{name} must be finite, got {value}"));
            }
        }
        Ok(())
    }
}

/// The currents on one spacecraft \[A\].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CraftCurrents {
    /// Upstream `*EBCurrent`: electron beam.
    pub electron_beam_a: f64,
    /// Upstream `*PhotoelectricCurrent`.
    pub photoelectric_a: f64,
    /// Upstream `*PlasmaElectronCurrent`.
    pub plasma_electron_a: f64,
    /// Upstream `*PlasmaIonCurrent`.
    pub plasma_ion_a: f64,
}

impl CraftCurrents {
    fn total_a(&self) -> f64 {
        self.plasma_electron_a + self.plasma_ion_a + self.photoelectric_a + self.electron_beam_a
    }
}

/// What one spacecraft contributes to its currents.
#[derive(Clone, Copy, Debug, Default)]
struct Craft {
    /// \[m^2\] surface area exposed to the plasma.
    surface_area_m2: f64,
    /// \[m^2\] sunlit area.
    sunlit_area_m2: f64,
    /// \[m/s\] inertial speed.
    speed_mps: f64,
}

/// The inputs of one integration step and the plasma parameters: the
/// right-hand side of the two charging equations (upstream
/// `equationsOfMotion` and the current methods).
#[derive(Clone, Copy, Debug)]
struct ChargingEnvironment {
    servicer: Craft,
    target: Craft,
    /// \[eV\] electron beam energy, \[A\] current, \[-\] fraction reaching the
    /// target.
    beam_energy_ev: f64,
    beam_current_a: f64,
    beam_alpha: f64,
    servicer_capacitance_f: f64,
    target_capacitance_f: f64,
    flux_photoelectrons_a_per_m2: f64,
    temp_photoelectrons_ev: f64,
    temp_electrons_ev: f64,
    density_electrons_per_m3: f64,
    temp_ions_ev: f64,
    density_ions_per_m3: f64,
    bulk_velocity_ions_mps: f64,
}

impl ChargingEnvironment {
    /// Upstream `computePlasmaElectronCurrent`.
    fn plasma_electron_current(&self, surface_area_m2: f64, potential_v: f64) -> f64 {
        let thermal_velocity =
            ((8.0 * Q_CHARGE * self.temp_electrons_ev) / (MASS_ELECTRON * PI)).sqrt();
        let saturation =
            -0.25 * surface_area_m2 * Q_CHARGE * self.density_electrons_per_m3 * thermal_velocity;
        if potential_v <= 0.0 {
            saturation * (potential_v / self.temp_electrons_ev).exp()
        } else {
            saturation * (1.0 + potential_v / self.temp_electrons_ev)
        }
    }

    /// Upstream `computePlasmaIonCurrent`: thermal collection while the
    /// ions' thermal speed exceeds their speed relative to the spacecraft,
    /// ram collection on the sunlit area otherwise.
    fn plasma_ion_current(&self, craft: &Craft, potential_v: f64) -> f64 {
        let thermal_velocity = ((8.0 * Q_CHARGE * self.temp_ions_ev) / (MASS_PROTON * PI)).sqrt();
        let relative_velocity = (craft.speed_mps - self.bulk_velocity_ions_mps).abs();
        if thermal_velocity >= relative_velocity {
            let saturation = 0.25
                * craft.surface_area_m2
                * Q_CHARGE
                * self.density_ions_per_m3
                * thermal_velocity;
            if potential_v <= 0.0 {
                saturation * (1.0 - potential_v / self.temp_ions_ev)
            } else {
                saturation * (-potential_v / self.temp_ions_ev).exp()
            }
        } else {
            craft.sunlit_area_m2 * Q_CHARGE * self.density_ions_per_m3 * relative_velocity
        }
    }

    /// Upstream `computePhotoelectricCurrent`, for one spacecraft.
    fn photoelectric_current(&self, sunlit_area_m2: f64, potential_v: f64) -> f64 {
        if potential_v <= 0.0 {
            self.flux_photoelectrons_a_per_m2 * sunlit_area_m2
        } else {
            self.flux_photoelectrons_a_per_m2
                * sunlit_area_m2
                * (-potential_v / self.temp_photoelectrons_ev).exp()
        }
    }

    /// Upstream `computeElectronBeamCurrent`: servicer and target beam
    /// currents, zero once the potential difference stops the beam.
    fn electron_beam_currents(&self, servicer_v: f64, target_v: f64) -> (f64, f64) {
        if self.beam_energy_ev > servicer_v - target_v {
            let intermediate = -(self.beam_energy_ev - servicer_v + target_v) / 20.0;
            let turn_on = 1.0 - intermediate.exp();
            (
                self.beam_current_a * turn_on,
                -self.beam_alpha * self.beam_current_a * turn_on,
            )
        } else {
            (0.0, 0.0)
        }
    }

    /// Upstream `computeCurrents`: every current at the potentials
    /// `[servicer, target]`.
    fn currents(&self, potentials_v: &Vector2<f64>) -> [CraftCurrents; 2] {
        let (servicer_v, target_v) = (potentials_v.x, potentials_v.y);
        let (servicer_beam, target_beam) = self.electron_beam_currents(servicer_v, target_v);
        let craft = |craft: &Craft, potential_v: f64, beam_a: f64| CraftCurrents {
            electron_beam_a: beam_a,
            photoelectric_a: self.photoelectric_current(craft.sunlit_area_m2, potential_v),
            plasma_electron_a: self.plasma_electron_current(craft.surface_area_m2, potential_v),
            plasma_ion_a: self.plasma_ion_current(craft, potential_v),
        };
        [
            craft(&self.servicer, servicer_v, servicer_beam),
            craft(&self.target, target_v, target_beam),
        ]
    }
}

impl DynamicObject for ChargingEnvironment {
    type State = Vector2<f64>;
    type StepOutput = ();
    type Derivative = Vector2<f64>;

    fn equations_of_motion(
        &mut self,
        state: &Vector2<f64>,
        _current_sim_nanos: u64,
        _current_epoch: Epoch,
    ) -> Vector2<f64> {
        let [servicer, target] = self.currents(state);
        Vector2::new(
            servicer.total_a() / self.servicer_capacitance_f,
            target.total_a() / self.target_capacitance_f,
        )
    }

    fn state_with_derivative(
        &self,
        state: &Vector2<f64>,
        derivative: &Vector2<f64>,
        dt: f64,
    ) -> Vector2<f64> {
        state + derivative * dt
    }

    fn combine(
        &self,
        state: &Vector2<f64>,
        weighted: Vec<(Vector2<f64>, f64)>,
        step: f64,
    ) -> (Vector2<f64>, ()) {
        let increment: Vector2<f64> = weighted
            .iter()
            .map(|(derivative, weight)| derivative * *weight)
            .sum();
        (state + increment * step, ())
    }

    fn merge_step_outputs(&self, _accumulated: (), _next: ()) {}
}

/// An input upstream reports with `bskError` when unlinked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChargingInputNotConnected(pub &'static str);

impl fmt::Display for ChargingInputNotConnected {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} was not linked", self.0)
    }
}

/// Servicer and target charging (upstream `SpacecraftChargingDynamics`).
#[derive(Clone, Debug)]
pub struct SpacecraftChargingDynamics {
    /// Module configuration.
    pub config: SpacecraftChargingDynamicsConfig,
    /// Upstream `servicerStateInMsg`: servicer inertial velocity.
    pub servicer_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `targetStateInMsg`: target inertial velocity.
    pub target_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `electronBeamInMsg` (optional): the servicer's electron beam.
    pub electron_beam_in_msg: Input<ElectronBeamMsg>,
    /// Upstream `servicerSurfaceAreaInMsg`: servicer area exposed to the
    /// plasma.
    pub servicer_surface_area_in_msg: Input<ProjectedAreaMsg>,
    /// Upstream `targetSurfaceAreaInMsg`.
    pub target_surface_area_in_msg: Input<ProjectedAreaMsg>,
    /// Upstream `servicerSunlitAreaInMsg`: servicer sunlit area.
    pub servicer_sunlit_area_in_msg: Input<ProjectedAreaMsg>,
    /// Upstream `targetSunlitAreaInMsg`.
    pub target_sunlit_area_in_msg: Input<ProjectedAreaMsg>,
    /// Upstream `servicerPotentialOutMsg`.
    pub servicer_potential_out_msg: Output<VoltMsg>,
    /// Upstream `targetPotentialOutMsg`.
    pub target_potential_out_msg: Output<VoltMsg>,
    /// Upstream `servicerEBCurrentOutMsg`.
    pub servicer_eb_current_out_msg: Output<CurrentMsg>,
    /// Upstream `targetEBCurrentOutMsg`.
    pub target_eb_current_out_msg: Output<CurrentMsg>,
    /// Upstream `servicerPhotoelectricCurrentOutMsg`.
    pub servicer_photoelectric_current_out_msg: Output<CurrentMsg>,
    /// Upstream `targetPhotoelectricCurrentOutMsg`.
    pub target_photoelectric_current_out_msg: Output<CurrentMsg>,
    /// Upstream `servicerPlasmaElectronCurrentOutMsg`.
    pub servicer_plasma_electron_current_out_msg: Output<CurrentMsg>,
    /// Upstream `targetPlasmaElectronCurrentOutMsg`.
    pub target_plasma_electron_current_out_msg: Output<CurrentMsg>,
    /// Upstream `servicerPlasmaIonCurrentOutMsg`.
    pub servicer_plasma_ion_current_out_msg: Output<CurrentMsg>,
    /// Upstream `targetPlasmaIonCurrentOutMsg`.
    pub target_plasma_ion_current_out_msg: Output<CurrentMsg>,
    /// \[V\] servicer and target potentials.
    potentials_v: Vector2<f64>,
    /// Upstream `timeBeforeNanos`: the time the potentials are at.
    time_before_nanos: u64,
}

impl SpacecraftChargingDynamics {
    /// # Panics
    ///
    /// If the configuration is invalid (upstream's setter `assert`s).
    pub fn new(config: SpacecraftChargingDynamicsConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid SpacecraftChargingDynamicsConfig: {message}");
        }
        let potentials_v = Vector2::new(
            config.servicer_potential_init_v,
            config.target_potential_init_v,
        );
        Self {
            config,
            servicer_state_in_msg: Input::default(),
            target_state_in_msg: Input::default(),
            electron_beam_in_msg: Input::default(),
            servicer_surface_area_in_msg: Input::default(),
            target_surface_area_in_msg: Input::default(),
            servicer_sunlit_area_in_msg: Input::default(),
            target_sunlit_area_in_msg: Input::default(),
            servicer_potential_out_msg: Output::default(),
            target_potential_out_msg: Output::default(),
            servicer_eb_current_out_msg: Output::default(),
            target_eb_current_out_msg: Output::default(),
            servicer_photoelectric_current_out_msg: Output::default(),
            target_photoelectric_current_out_msg: Output::default(),
            servicer_plasma_electron_current_out_msg: Output::default(),
            target_plasma_electron_current_out_msg: Output::default(),
            servicer_plasma_ion_current_out_msg: Output::default(),
            target_plasma_ion_current_out_msg: Output::default(),
            potentials_v,
            time_before_nanos: 0,
        }
    }

    /// The servicer and target potentials \[V\].
    pub fn potentials_v(&self) -> (f64, f64) {
        (self.potentials_v.x, self.potentials_v.y)
    }

    /// The required inputs that are not connected.
    pub fn configuration_errors(&self) -> Vec<ChargingInputNotConnected> {
        [
            (
                self.servicer_state_in_msg.is_connected(),
                "servicer_state_in_msg",
            ),
            (
                self.target_state_in_msg.is_connected(),
                "target_state_in_msg",
            ),
            (
                self.servicer_surface_area_in_msg.is_connected(),
                "servicer_surface_area_in_msg",
            ),
            (
                self.target_surface_area_in_msg.is_connected(),
                "target_surface_area_in_msg",
            ),
            (
                self.servicer_sunlit_area_in_msg.is_connected(),
                "servicer_sunlit_area_in_msg",
            ),
            (
                self.target_sunlit_area_in_msg.is_connected(),
                "target_sunlit_area_in_msg",
            ),
        ]
        .into_iter()
        .filter(|(connected, _)| !connected)
        .map(|(_, port)| ChargingInputNotConnected(port))
        .collect()
    }

    /// Upstream `UpdateState`'s input reads.
    fn environment(&self) -> ChargingEnvironment {
        let beam = if self.electron_beam_in_msg.is_connected() {
            self.electron_beam_in_msg.read()
        } else {
            ElectronBeamMsg::default()
        };
        let config = &self.config;
        ChargingEnvironment {
            servicer: Craft {
                surface_area_m2: self.servicer_surface_area_in_msg.read().area_m2,
                sunlit_area_m2: self.servicer_sunlit_area_in_msg.read().area_m2,
                speed_mps: self.servicer_state_in_msg.read().velocity_mps.norm(),
            },
            target: Craft {
                surface_area_m2: self.target_surface_area_in_msg.read().area_m2,
                sunlit_area_m2: self.target_sunlit_area_in_msg.read().area_m2,
                speed_mps: self.target_state_in_msg.read().velocity_mps.norm(),
            },
            beam_energy_ev: beam.energy_ev,
            beam_current_a: beam.current_a,
            beam_alpha: beam.alpha,
            servicer_capacitance_f: config.servicer_capacitance_f,
            target_capacitance_f: config.target_capacitance_f,
            flux_photoelectrons_a_per_m2: config.flux_photoelectrons_a_per_m2,
            temp_photoelectrons_ev: config.temp_photoelectrons_ev,
            temp_electrons_ev: config.temp_electrons_ev,
            density_electrons_per_m3: config.density_electrons_per_m3,
            temp_ions_ev: config.temp_ions_ev,
            density_ions_per_m3: config.density_ions_per_m3,
            bulk_velocity_ions_mps: config.bulk_velocity_ions_mps,
        }
    }

    /// Upstream `writeOutputStateMessages`.
    fn write_outputs(&self, currents: [CraftCurrents; 2]) {
        let [servicer, target] = currents;
        let current = |output: &Output<CurrentMsg>, current_a: f64| {
            output.write(CurrentMsg { current_a });
        };
        self.servicer_potential_out_msg.write(VoltMsg {
            voltage_v: self.potentials_v.x,
        });
        self.target_potential_out_msg.write(VoltMsg {
            voltage_v: self.potentials_v.y,
        });
        current(&self.servicer_eb_current_out_msg, servicer.electron_beam_a);
        current(&self.target_eb_current_out_msg, target.electron_beam_a);
        current(
            &self.servicer_photoelectric_current_out_msg,
            servicer.photoelectric_a,
        );
        current(
            &self.target_photoelectric_current_out_msg,
            target.photoelectric_a,
        );
        current(
            &self.servicer_plasma_electron_current_out_msg,
            servicer.plasma_electron_a,
        );
        current(
            &self.target_plasma_electron_current_out_msg,
            target.plasma_electron_a,
        );
        current(
            &self.servicer_plasma_ion_current_out_msg,
            servicer.plasma_ion_a,
        );
        current(&self.target_plasma_ion_current_out_msg, target.plasma_ion_a);
    }

    /// Upstream `Reset`: back to the initial potentials at `now_nanos`. The
    /// currents published are zero, which is what upstream computes before
    /// any input is read.
    fn restart(&mut self, now_nanos: u64) {
        for error in self.configuration_errors() {
            log::error!("{}: {error}", self.config.name);
        }
        self.potentials_v = Vector2::new(
            self.config.servicer_potential_init_v,
            self.config.target_potential_init_v,
        );
        self.time_before_nanos = now_nanos;
        self.write_outputs([CraftCurrents::default(); 2]);
    }
}

impl Module for SpacecraftChargingDynamics {
    fn init(&mut self) {
        self.restart(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.restart(context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        let mut environment = self.environment();
        let dt_s = diff_nanos_to_seconds(context.current_sim_nanos, self.time_before_nanos);
        let (potentials_v, ()) = Rk4.propagate(
            &mut environment,
            &self.potentials_v,
            self.time_before_nanos,
            context.current_epoch,
            dt_s,
        );
        self.potentials_v = potentials_v;
        self.time_before_nanos = context.current_sim_nanos;
        self.write_outputs(environment.currents(&self.potentials_v));
    }
}

/// Port of upstream `test_spacecraftChargingDynamics.py`.
///
/// Upstream propagates two spacecraft hubs to feed the velocities; nothing
/// acts on them, so their velocities are constant and fixed state messages
/// stand in for them here.
#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{
        ChargingInputNotConnected, SpacecraftChargingDynamics, SpacecraftChargingDynamicsConfig,
    };
    use crate::messages::{ElectronBeamMsg, Output, ProjectedAreaMsg, SpacecraftStateMsg};
    use crate::utilities::astro_constants::{MASS_ELECTRON, MASS_PROTON, Q_CHARGE};
    use crate::{Module, SimulationContext};

    const TEMP_ELECTRONS: f64 = 2.0;
    const DENSITY_ELECTRONS: f64 = 950000.0;
    const TEMP_IONS: f64 = 2.0;
    const DENSITY_IONS: f64 = 950000.0;
    const TEMP_PHOTONS: f64 = 2.0;
    const FLUX_PHOTONS: f64 = 1e-6;

    /// The test's truth for one set of beam and plasma parameters.
    struct Truth {
        servicer_surface_area: f64,
        target_surface_area: f64,
        servicer_sunlit_area: f64,
        target_sunlit_area: f64,
        v_sn_n: Vector3<f64>,
        v_tn_n: Vector3<f64>,
        bulk_velocity_ions: f64,
        capacitance: f64,
        electron_beam_current: f64,
        electron_beam_energy: f64,
        electron_beam_alpha: f64,
    }

    fn compute_plasma_electron_current(potential: f64, surface_area: f64) -> f64 {
        let velocity_electrons = ((8.0 * Q_CHARGE * TEMP_ELECTRONS) / (MASS_ELECTRON * PI)).sqrt();
        let base = -0.25 * surface_area * Q_CHARGE * DENSITY_ELECTRONS * velocity_electrons;
        if potential <= 0.0 {
            base * (potential / TEMP_ELECTRONS).exp()
        } else {
            base * (1.0 + (potential / TEMP_ELECTRONS))
        }
    }

    fn compute_plasma_ion_current(
        potential: f64,
        surface_area: f64,
        sunlit_area: f64,
        v_bn_n: Vector3<f64>,
        bulk_velocity_ions: f64,
    ) -> f64 {
        let thermal_velocity_ions = ((8.0 * Q_CHARGE * TEMP_IONS) / (MASS_PROTON * PI)).sqrt();
        let relative_velocity_ions = (v_bn_n.norm() - bulk_velocity_ions).abs();
        if thermal_velocity_ions >= relative_velocity_ions {
            let base = 0.25 * surface_area * Q_CHARGE * DENSITY_IONS * thermal_velocity_ions;
            if potential <= 0.0 {
                base * (1.0 - (potential / TEMP_IONS))
            } else {
                base * (-potential / TEMP_IONS).exp()
            }
        } else {
            sunlit_area * Q_CHARGE * DENSITY_IONS * relative_velocity_ions
        }
    }

    fn compute_photoelectric_current(potential: f64, sunlit_area: f64) -> f64 {
        if potential <= 0.0 {
            FLUX_PHOTONS * sunlit_area
        } else {
            FLUX_PHOTONS * sunlit_area * (-potential / TEMP_PHOTONS).exp()
        }
    }

    fn compute_electron_beam_current(
        current: f64,
        energy: f64,
        alpha: f64,
        servicer_potential: f64,
        target_potential: f64,
    ) -> (f64, f64) {
        if energy > (servicer_potential - target_potential) {
            let intermediate_term = -(energy - servicer_potential + target_potential) / 20.0;
            (
                current * (1.0 - intermediate_term.exp()),
                -alpha * current * (1.0 - intermediate_term.exp()),
            )
        } else {
            (0.0, 0.0)
        }
    }

    impl Truth {
        /// `[servicer, target]` currents: plasma electron, plasma ion,
        /// photoelectric, electron beam.
        fn currents(&self, servicer_potential: f64, target_potential: f64) -> [[f64; 4]; 2] {
            let (servicer_eb, target_eb) = compute_electron_beam_current(
                self.electron_beam_current,
                self.electron_beam_energy,
                self.electron_beam_alpha,
                servicer_potential,
                target_potential,
            );
            [
                [
                    compute_plasma_electron_current(servicer_potential, self.servicer_surface_area),
                    compute_plasma_ion_current(
                        servicer_potential,
                        self.servicer_surface_area,
                        self.servicer_sunlit_area,
                        self.v_sn_n,
                        self.bulk_velocity_ions,
                    ),
                    compute_photoelectric_current(servicer_potential, self.servicer_sunlit_area),
                    servicer_eb,
                ],
                [
                    compute_plasma_electron_current(target_potential, self.target_surface_area),
                    compute_plasma_ion_current(
                        target_potential,
                        self.target_surface_area,
                        self.target_sunlit_area,
                        self.v_tn_n,
                        self.bulk_velocity_ions,
                    ),
                    compute_photoelectric_current(target_potential, self.target_sunlit_area),
                    target_eb,
                ],
            ]
        }

        fn equations_of_motion(
            &self,
            servicer_potential: f64,
            target_potential: f64,
        ) -> (f64, f64) {
            let [servicer, target] = self.currents(servicer_potential, target_potential);
            (
                servicer.iter().sum::<f64>() / self.capacitance,
                target.iter().sum::<f64>() / self.capacitance,
            )
        }

        fn rk4(&self, servicer: f64, target: f64, dt: f64) -> (f64, f64) {
            let (s1, t1) = self.equations_of_motion(servicer, target);
            let (s2, t2) =
                self.equations_of_motion(servicer + 0.5 * (dt * s1), target + 0.5 * (dt * t1));
            let (s3, t3) =
                self.equations_of_motion(servicer + 0.5 * (dt * s2), target + 0.5 * (dt * t2));
            let (s4, t4) = self.equations_of_motion(servicer + dt * s3, target + dt * t3);
            (
                servicer + (dt / 6.0) * (s1 + 2.0 * s2 + 2.0 * s3 + s4),
                target + (dt / 6.0) * (t1 + 2.0 * t2 + 2.0 * t3 + t4),
            )
        }
    }

    fn assert_close(actual: f64, expected: f64, what: &str) {
        assert!(
            (actual - expected).abs() <= 1e-7,
            "{what}: got {actual:e}, expected {expected:e}"
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn charging_dynamics_case(
        servicer_potential_init: f64,
        target_potential_init: f64,
        servicer_radius: f64,
        target_radius: f64,
        bulk_velocity_ions: f64,
        electron_beam_energy: f64,
        electron_beam_current: f64,
        electron_beam_alpha: f64,
    ) {
        let case = format!(
            "phi0=({servicer_potential_init}, {target_potential_init}), radii=({servicer_radius}, \
             {target_radius}), v_bulk={bulk_velocity_ions}, E_EB={electron_beam_energy}, \
             I_EB={electron_beam_current}, alpha={electron_beam_alpha}"
        );
        let step_nanos = 100;
        let test_time_step_sec = 1e-7;
        let v_n = Vector3::new(-5199.77710904224, -3436.681645356935, 1041.576797498721);

        let truth = Truth {
            servicer_surface_area: 4.0 * PI * servicer_radius * servicer_radius,
            target_surface_area: 4.0 * PI * target_radius * target_radius,
            servicer_sunlit_area: PI * servicer_radius * servicer_radius,
            target_sunlit_area: PI * target_radius * target_radius,
            v_sn_n: v_n,
            v_tn_n: v_n,
            bulk_velocity_ions,
            capacitance: 1e-9,
            electron_beam_current,
            electron_beam_energy,
            electron_beam_alpha,
        };

        let servicer_state = Output::new(SpacecraftStateMsg {
            position_m: Vector3::new(10.0, 0.0, 0.0),
            velocity_mps: v_n,
            ..Default::default()
        });
        let target_state = Output::new(SpacecraftStateMsg {
            position_m: Vector3::new(5.0, 0.0, 0.0),
            velocity_mps: v_n,
            ..Default::default()
        });
        let area = |area_m2| Output::new(ProjectedAreaMsg { area_m2 });
        let servicer_surface = area(truth.servicer_surface_area);
        let target_surface = area(truth.target_surface_area);
        let servicer_sunlit = area(truth.servicer_sunlit_area);
        let target_sunlit = area(truth.target_sunlit_area);
        let beam = Output::new(ElectronBeamMsg {
            energy_ev: electron_beam_energy,
            current_a: electron_beam_current,
            alpha: electron_beam_alpha,
        });

        let mut charging = SpacecraftChargingDynamics::new(SpacecraftChargingDynamicsConfig {
            name: "SpacecraftChargingDynamics".to_string(),
            servicer_potential_init_v: servicer_potential_init,
            target_potential_init_v: target_potential_init,
            servicer_capacitance_f: truth.capacitance,
            target_capacitance_f: truth.capacitance,
            flux_photoelectrons_a_per_m2: FLUX_PHOTONS,
            temp_electrons_ev: TEMP_ELECTRONS,
            density_electrons_per_m3: DENSITY_ELECTRONS,
            temp_ions_ev: TEMP_IONS,
            density_ions_per_m3: DENSITY_IONS,
            bulk_velocity_ions_mps: bulk_velocity_ions,
            ..Default::default()
        });
        charging
            .servicer_state_in_msg
            .connect(servicer_state.slot());
        charging.target_state_in_msg.connect(target_state.slot());
        charging.electron_beam_in_msg.connect(beam.slot());
        charging
            .servicer_surface_area_in_msg
            .connect(servicer_surface.slot());
        charging
            .target_surface_area_in_msg
            .connect(target_surface.slot());
        charging
            .servicer_sunlit_area_in_msg
            .connect(servicer_sunlit.slot());
        charging
            .target_sunlit_area_in_msg
            .connect(target_sunlit.slot());

        charging.init();
        let mut previous: Option<(f64, f64)> = None;
        // Upstream records every 100 ns step from 0 to 10 us.
        for step in 0..=100_u64 {
            charging.update(&SimulationContext {
                current_sim_nanos: step * step_nanos,
                current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
            });
            let servicer_v = charging.servicer_potential_out_msg.read().voltage_v;
            let target_v = charging.target_potential_out_msg.read().voltage_v;

            let [servicer_truth, target_truth] = truth.currents(servicer_v, target_v);
            let outputs = [
                (
                    &charging.servicer_plasma_electron_current_out_msg,
                    servicer_truth[0],
                    "servicer plasma electron",
                ),
                (
                    &charging.servicer_plasma_ion_current_out_msg,
                    servicer_truth[1],
                    "servicer plasma ion",
                ),
                (
                    &charging.servicer_photoelectric_current_out_msg,
                    servicer_truth[2],
                    "servicer photoelectric",
                ),
                (
                    &charging.servicer_eb_current_out_msg,
                    servicer_truth[3],
                    "servicer electron beam",
                ),
                (
                    &charging.target_plasma_electron_current_out_msg,
                    target_truth[0],
                    "target plasma electron",
                ),
                (
                    &charging.target_plasma_ion_current_out_msg,
                    target_truth[1],
                    "target plasma ion",
                ),
                (
                    &charging.target_photoelectric_current_out_msg,
                    target_truth[2],
                    "target photoelectric",
                ),
                (
                    &charging.target_eb_current_out_msg,
                    target_truth[3],
                    "target electron beam",
                ),
            ];
            for (output, expected, what) in outputs {
                assert_close(
                    output.read().current_a,
                    expected,
                    &format!("{what}, step {step}, {case}"),
                );
            }

            let (servicer_expected, target_expected) = match previous {
                None => (servicer_potential_init, target_potential_init),
                Some((servicer, target)) => truth.rk4(servicer, target, test_time_step_sec),
            };
            assert_close(
                servicer_v,
                servicer_expected,
                &format!("servicer potential, step {step}, {case}"),
            );
            assert_close(
                target_v,
                target_expected,
                &format!("target potential, step {step}, {case}"),
            );
            previous = Some((servicer_v, target_v));
        }
    }

    /// Upstream's full parameter sweep.
    #[test]
    fn currents_and_potentials_match_the_rk4_truth() {
        for (servicer_init, target_init) in
            [(0.0, 0.0), (-5000.0, -5000.0), (10.0, 10.0), (11000.0, 0.0)]
        {
            for (servicer_radius, target_radius) in [(3.0, 3.0), (2.0, 4.0), (4.0, 2.0)] {
                for bulk_velocity_ions in [0.0, 1.0, 400000.0] {
                    for electron_beam_energy in [0.0, 10000.0] {
                        for electron_beam_current in [0.0, 0.001, 250e-6] {
                            for electron_beam_alpha in [0.0, 0.5, 1.0] {
                                charging_dynamics_case(
                                    servicer_init,
                                    target_init,
                                    servicer_radius,
                                    target_radius,
                                    bulk_velocity_ions,
                                    electron_beam_energy,
                                    electron_beam_current,
                                    electron_beam_alpha,
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn without_a_beam_there_is_no_beam_current() {
        let mut charging = SpacecraftChargingDynamics::new(SpacecraftChargingDynamicsConfig {
            servicer_potential_init_v: -100.0,
            ..Default::default()
        });
        let area = Output::new(ProjectedAreaMsg { area_m2: 10.0 });
        charging.servicer_surface_area_in_msg.connect(area.slot());
        charging.init();
        charging.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });
        assert_eq!(charging.servicer_eb_current_out_msg.read().current_a, 0.0);
        assert_eq!(charging.target_eb_current_out_msg.read().current_a, 0.0);
        assert_eq!(charging.potentials_v(), (-100.0, 0.0));
        assert!(
            charging
                .servicer_plasma_electron_current_out_msg
                .read()
                .current_a
                < 0.0
        );
    }

    #[test]
    fn unconnected_required_inputs_are_reported() {
        let charging = SpacecraftChargingDynamics::new(SpacecraftChargingDynamicsConfig::default());
        let errors = charging.configuration_errors();
        assert_eq!(errors.len(), 6);
        assert_eq!(
            errors[0],
            ChargingInputNotConnected("servicer_state_in_msg")
        );
        assert_eq!(
            errors[5],
            ChargingInputNotConnected("target_sunlit_area_in_msg")
        );
    }

    #[test]
    #[should_panic(expected = "servicer_capacitance_f must be finite and positive")]
    fn rejects_a_non_positive_capacitance() {
        SpacecraftChargingDynamics::new(SpacecraftChargingDynamicsConfig {
            servicer_capacitance_f: 0.0,
            ..Default::default()
        });
    }

    #[test]
    #[should_panic(expected = "bulk_velocity_ions_mps must be finite and >= 0")]
    fn rejects_a_negative_bulk_velocity() {
        SpacecraftChargingDynamics::new(SpacecraftChargingDynamicsConfig {
            bulk_velocity_ions_mps: -1.0,
            ..Default::default()
        });
    }
}
