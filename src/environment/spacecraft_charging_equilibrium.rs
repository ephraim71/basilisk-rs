//! Equilibrium potentials of a servicer and a target spacecraft: port of
//! upstream Basilisk `simulation/environment/spacecraftChargingEquilibrium`.
//!
//! A spacecraft floats at the potential where the currents it collects and
//! emits cancel. The currents are integrals over the ambient plasma spectrum
//! ([`PlasmaFluxMsg`], e.g. from [`super::denton_flux_model`]): collected
//! electrons and ions, the secondary electrons both release, backscattered
//! electrons, and photoelectrons from the sunlit area. The servicer also
//! fires an electron beam at the target, which lands on it and releases
//! secondaries and backscatter there.
//!
//! The servicer's potential is solved first, with the target taken at 0 V,
//! then the target's at that servicer potential, each by bisection. Index 0
//! of every port array is the servicer ([`SERVICER`]), index 1 the target
//! ([`TARGET`]); only the servicer's beam is used.
//!
//! # Differences from upstream
//!
//! * The two spacecraft are fixed port arrays instead of two
//!   `addSpacecraft` calls, so "exactly two spacecraft" holds by
//!   construction; an unconnected state input is reported at `init`.
//! * No message headers: the beam and sunlit-area inputs are used whenever
//!   connected ("connected ⇒ written at the current update").
//! * The sunlit-area inputs are [`ProjectedAreaMsg`]s, which carry the same
//!   single area as upstream's `ScSunlitFacetAreaMsg`.
//! * Upstream raises `bskError` from `UpdateState` for an unusable plasma
//!   grid, short yield tables or a failed root solve; here the update logs
//!   the error and leaves the outputs as they were.
//! * Upstream's debug prints (`setEnableDebugPrints`) are `log::debug!`
//!   records.
//! * Upstream reads the spacecraft positions and attitudes but uses neither;
//!   the state inputs are only checked for a connection.

use std::fmt;

use crate::messages::{
    ElectronBeamMsg, Input, MAX_PLASMA_FLUX_SIZE, Output, PlasmaFluxMsg, ProjectedAreaMsg,
    ScChargingCurrentsMsg, SpacecraftStateMsg, VoltMsg,
};
use crate::utilities::bisection::bisection_solve;
use crate::{Module, SimulationContext};

/// Index of the servicer in the port arrays.
pub const SERVICER: usize = 0;
/// Index of the target in the port arrays.
pub const TARGET: usize = 1;

/// Upstream `elementaryCharge` \[C\] (the module's own value, not
/// `Q_CHARGE`).
const ELEMENTARY_CHARGE: f64 = 1.60217663e-19;
/// Upstream `minIntegrationEnergy` \[eV\]: lower bound of the current
/// integrals.
const MIN_INTEGRATION_ENERGY: f64 = 0.1;
/// Upstream `servicerSolveAccuracy` \[V\].
const SERVICER_SOLVE_ACCURACY: f64 = 1e-6;
/// Upstream `targetSolveAccuracy` \[V\].
const TARGET_SOLVE_ACCURACY: f64 = 1e-8;

/// User-settable parameters of [`SpacecraftChargingEquilibrium`] (upstream's
/// setters).
#[derive(Clone, Debug)]
pub struct SpacecraftChargingEquilibriumConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `yieldSEEelectron` \[-\]: electron-impact secondary emission
    /// yield on the plasma energy grid.
    pub yield_see_electron: Vec<f64>,
    /// Upstream `yieldSEEion` \[-\]: ion-impact secondary emission yield.
    pub yield_see_ion: Vec<f64>,
    /// Upstream `yieldBackscattered` \[-\]: electron backscatter yield.
    pub yield_backscattered: Vec<f64>,
    /// Upstream `setSunlitAreaDefault` \[m^2\]: per-spacecraft sunlit area
    /// used while that spacecraft's sunlit-area input is unconnected.
    pub sunlit_area_defaults_m2: [Option<f64>; 2],
    /// Upstream `sunlitAreaFallback` \[m^2\]: sunlit area without an input or
    /// a default.
    pub sunlit_area_fallback_m2: f64,
    /// Upstream `rootSolveLowerBound` \[V\]: lower bisection bracket.
    pub root_solve_lower_bound_v: f64,
    /// Upstream `rootSolveUpperBound` \[V\]: upper bisection bracket.
    pub root_solve_upper_bound_v: f64,
    /// Upstream `defaultSurfaceArea` \[m^2\]: area exposed to the plasma.
    pub surface_area_m2: f64,
    /// Upstream `photoelectronFlux` \[A/m^2\]: photoelectron current density
    /// at a non-positive potential.
    pub photoelectron_flux_a_per_m2: f64,
    /// Upstream `photoelectronTemperature` \[eV\].
    pub photoelectron_temperature_ev: f64,
    /// Upstream `secondaryElectronTemperature` \[eV\].
    pub secondary_electron_temperature_ev: f64,
    /// Upstream `backscatterElectronTemperature` \[eV\].
    pub backscatter_electron_temperature_ev: f64,
    /// Upstream `beamElectronTemperature` \[eV\]: beam turn-on energy.
    pub beam_electron_temperature_ev: f64,
    /// Upstream `trapzBins`: trapezoids of every current integral.
    pub trapz_bins: usize,
}

impl SpacecraftChargingEquilibriumConfig {
    /// The three yield tables, with upstream's defaults for everything else.
    pub fn new(
        yield_see_electron: Vec<f64>,
        yield_see_ion: Vec<f64>,
        yield_backscattered: Vec<f64>,
    ) -> Self {
        Self {
            name: "spacecraftChargingEquilibrium".to_string(),
            yield_see_electron,
            yield_see_ion,
            yield_backscattered,
            sunlit_area_defaults_m2: [None, None],
            sunlit_area_fallback_m2: 0.0,
            root_solve_lower_bound_v: -400_000.0,
            root_solve_upper_bound_v: 400_000.0,
            surface_area_m2: 50.264,
            photoelectron_flux_a_per_m2: 40e-6,
            photoelectron_temperature_ev: 2.0,
            secondary_electron_temperature_ev: 5.0,
            backscatter_electron_temperature_ev: 5.0,
            beam_electron_temperature_ev: 20.0,
            trapz_bins: 1000,
        }
    }

    /// Upstream's setter checks. Returns a description of the first
    /// violation, or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        for (name, table) in [
            ("yield_see_electron", &self.yield_see_electron),
            ("yield_see_ion", &self.yield_see_ion),
            ("yield_backscattered", &self.yield_backscattered),
        ] {
            if table.is_empty() || !table.iter().all(|value| value.is_finite() && *value >= 0.0) {
                return Err(format!(
                    "{name} must be a non-empty table of finite non-negative values"
                ));
            }
        }
        let non_negative_areas = self
            .sunlit_area_defaults_m2
            .iter()
            .flatten()
            .chain([&self.sunlit_area_fallback_m2]);
        for area in non_negative_areas {
            if !(area.is_finite() && *area >= 0.0) {
                return Err(format!("sunlit areas must be finite and >= 0, got {area}"));
            }
        }
        if !(self.root_solve_lower_bound_v.is_finite()
            && self.root_solve_upper_bound_v.is_finite()
            && self.root_solve_lower_bound_v < self.root_solve_upper_bound_v)
        {
            return Err(format!(
                "root solve bounds must be finite with lower < upper, got [{}, {}]",
                self.root_solve_lower_bound_v, self.root_solve_upper_bound_v
            ));
        }
        if !(self.surface_area_m2.is_finite() && self.surface_area_m2 > 0.0) {
            return Err(format!(
                "surface_area_m2 must be finite and positive, got {}",
                self.surface_area_m2
            ));
        }
        if !(self.photoelectron_flux_a_per_m2.is_finite()
            && self.photoelectron_flux_a_per_m2 >= 0.0)
        {
            return Err(format!(
                "photoelectron_flux_a_per_m2 must be finite and >= 0, got {}",
                self.photoelectron_flux_a_per_m2
            ));
        }
        for (name, temperature) in [
            (
                "photoelectron_temperature_ev",
                self.photoelectron_temperature_ev,
            ),
            (
                "secondary_electron_temperature_ev",
                self.secondary_electron_temperature_ev,
            ),
            (
                "backscatter_electron_temperature_ev",
                self.backscatter_electron_temperature_ev,
            ),
            (
                "beam_electron_temperature_ev",
                self.beam_electron_temperature_ev,
            ),
        ] {
            if !(temperature.is_finite() && temperature > 0.0) {
                return Err(format!(
                    "{name} must be finite and positive, got {temperature}"
                ));
            }
        }
        if self.trapz_bins == 0 {
            return Err("trapz_bins must be strictly positive".to_string());
        }
        Ok(())
    }
}

/// Which side of the beam a current is for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Craft {
    Servicer,
    Target,
}

/// The servicer's electron gun (upstream `ElectronGunConfig`).
#[derive(Clone, Copy, Debug)]
struct ElectronGun {
    alpha: f64,
    current_a: f64,
    energy_ev: f64,
}

impl Default for ElectronGun {
    fn default() -> Self {
        Self {
            alpha: 1.0,
            current_a: 0.0,
            energy_ev: 0.0,
        }
    }
}

/// The plasma spectrum and yields of one update: the current models
/// (upstream's current methods, `interp`, `trapz`, `getFlux` and
/// `getYield`).
struct Currents<'a> {
    config: &'a SpacecraftChargingEquilibriumConfig,
    /// \[eV\] the plasma energy grid.
    energies: Vec<f64>,
    /// \[m^-2 s^-1 sr^-1 eV^-1\] electron and ion fluxes on the grid.
    electron_flux: Vec<f64>,
    ion_flux: Vec<f64>,
}

impl Currents<'_> {
    fn max_energy(&self) -> f64 {
        self.energies[self.energies.len() - 1]
    }

    /// Upstream `interp`: linear interpolation in `y` over the energy grid,
    /// extrapolating from the end intervals.
    fn interp(&self, y: &[f64], x: f64) -> f64 {
        // The first grid energy above `x` closes the interval.
        let above = self.energies.partition_point(|&energy| energy <= x);
        let upper = above.clamp(1, self.energies.len() - 1);
        let lower = upper - 1;
        let (x0, x1) = (self.energies[lower], self.energies[upper]);
        y[lower] + ((y[upper] - y[lower]) / (x1 - x0)) * (x - x0)
    }

    /// Upstream `getFlux`, clipped at zero.
    fn flux(&self, energy: f64, flux: &[f64]) -> f64 {
        self.interp(flux, energy).max(0.0)
    }

    /// Upstream `getYield`, clipped at zero.
    fn yield_at(&self, energy: f64, table: &[f64]) -> f64 {
        self.interp(table, energy).max(0.0)
    }

    /// Upstream `trapz`.
    fn trapz(&self, f: impl Fn(f64) -> f64, a: f64, b: f64) -> f64 {
        let n = self.config.trapz_bins;
        if !a.is_finite() || !b.is_finite() || n == 0 || b == a {
            return 0.0;
        }
        let h = (b - a) / n as f64;
        let sum: f64 = (1..n).map(|i| f(a + i as f64 * h)).sum();
        h * (0.5 * f(a) + sum + 0.5 * f(b))
    }

    /// The bounds of a collection integral over the landing energy: the grid
    /// when the potential repels the species, the grid shifted up by `|phi|`
    /// when it attracts it (every particle then lands with at least `|phi|`).
    fn bounds(&self, phi: f64, repelling: bool) -> (f64, f64) {
        if repelling {
            (MIN_INTEGRATION_ENERGY, self.max_energy())
        } else {
            (
                MIN_INTEGRATION_ENERGY + phi.abs(),
                self.max_energy() + phi.abs(),
            )
        }
    }

    /// The electron collection integral, weighted by `weight(E)`.
    fn electron_integral(&self, phi: f64, weight: impl Fn(f64) -> f64) -> f64 {
        let (a, b) = self.bounds(phi, phi < 0.0);
        self.trapz(
            |e| weight(e) * (e / (e - phi)) * self.flux(e - phi, &self.electron_flux),
            a,
            b,
        )
    }

    /// The ion collection integral, weighted by `weight(E)`.
    fn ion_integral(&self, phi: f64, weight: impl Fn(f64) -> f64) -> f64 {
        let (a, b) = self.bounds(phi, phi > 0.0);
        self.trapz(
            |e| weight(e) * (e / (e + phi)) * self.flux(e + phi, &self.ion_flux),
            a,
            b,
        )
    }

    /// Upstream `electronCurrent` \[A\].
    fn electron_current(&self, phi: f64, area_m2: f64) -> f64 {
        -ELEMENTARY_CHARGE * area_m2 * self.electron_integral(phi, |_| 1.0)
    }

    /// Upstream `ionCurrent` \[A\].
    fn ion_current(&self, phi: f64, area_m2: f64) -> f64 {
        ELEMENTARY_CHARGE * area_m2 * self.ion_integral(phi, |_| 1.0)
    }

    /// Emission at a positive potential is mostly recollected.
    fn recollected(&self, emitted: f64, phi: f64, temperature_ev: f64) -> f64 {
        if phi <= 0.0 {
            emitted
        } else {
            emitted * (-phi / temperature_ev).exp()
        }
    }

    /// Upstream `SEEelectronCurrent` \[A\].
    fn see_electron_current(&self, phi: f64, area_m2: f64) -> f64 {
        let table = &self.config.yield_see_electron;
        let emitted =
            ELEMENTARY_CHARGE * area_m2 * self.electron_integral(phi, |e| self.yield_at(e, table));
        self.recollected(emitted, phi, self.config.secondary_electron_temperature_ev)
    }

    /// Upstream `SEEionCurrent` \[A\].
    fn see_ion_current(&self, phi: f64, area_m2: f64) -> f64 {
        let table = &self.config.yield_see_ion;
        let emitted =
            ELEMENTARY_CHARGE * area_m2 * self.ion_integral(phi, |e| self.yield_at(e, table));
        self.recollected(emitted, phi, self.config.secondary_electron_temperature_ev)
    }

    /// Upstream `backscatteringCurrent` \[A\].
    fn backscattering_current(&self, phi: f64, area_m2: f64) -> f64 {
        let table = &self.config.yield_backscattered;
        let emitted =
            ELEMENTARY_CHARGE * area_m2 * self.electron_integral(phi, |e| self.yield_at(e, table));
        self.recollected(
            emitted,
            phi,
            self.config.backscatter_electron_temperature_ev,
        )
    }

    /// Upstream `photoelectricCurrent` \[A\].
    fn photoelectric_current(&self, phi: f64, sunlit_area_m2: f64) -> f64 {
        let emitted = self.config.photoelectron_flux_a_per_m2 * sunlit_area_m2;
        self.recollected(emitted, phi, self.config.photoelectron_temperature_ev)
    }

    /// The six terms that do not involve the beam, in the message's order.
    fn non_beam(&self, phi: f64, area_m2: f64, sunlit_area_m2: f64) -> [f64; 6] {
        [
            self.electron_current(phi, area_m2),
            self.ion_current(phi, area_m2),
            self.see_electron_current(phi, area_m2),
            self.see_ion_current(phi, area_m2),
            self.backscattering_current(phi, area_m2),
            self.photoelectric_current(phi, sunlit_area_m2),
        ]
    }

    /// Upstream `electronBeamCurrent` \[A\]: emitted by the servicer,
    /// received (negative) by the target, zero once the potential difference
    /// stops the beam.
    fn electron_beam_current(
        &self,
        phi_s: f64,
        phi_t: f64,
        craft: Craft,
        gun: &ElectronGun,
    ) -> f64 {
        if gun.energy_ev > phi_s - phi_t {
            let turn_on = 1.0
                - (-(gun.energy_ev - phi_s + phi_t) / self.config.beam_electron_temperature_ev)
                    .exp();
            match craft {
                Craft::Servicer => gun.current_a * turn_on,
                Craft::Target => -gun.alpha * gun.current_a * turn_on,
            }
        } else {
            0.0
        }
    }

    /// Upstream `SEEelectronBeamCurrent` and `electronBeamBackscattering`
    /// \[A\]: what the landing beam releases from the target.
    fn beam_emission(&self, phi_s: f64, phi_t: f64, gun: &ElectronGun, table: &[f64]) -> f64 {
        let landing_energy = gun.energy_ev - phi_s + phi_t;
        let incoming = (-self.electron_beam_current(phi_s, phi_t, Craft::Target, gun)).max(0.0);
        self.yield_at(landing_energy, table) * incoming
    }
}

/// An input upstream's `Reset` reports with `bskError` when unlinked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EquilibriumInputNotConnected(pub &'static str);

impl fmt::Display for EquilibriumInputNotConnected {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} was not linked", self.0)
    }
}

/// Coupled servicer and target equilibrium potentials (upstream
/// `SpacecraftChargingEquilibrium`).
#[derive(Clone, Debug)]
pub struct SpacecraftChargingEquilibrium {
    /// Module configuration.
    pub config: SpacecraftChargingEquilibriumConfig,
    /// Upstream `scStateInMsgs`: servicer and target states.
    pub spacecraft_state_in_msgs: [Input<SpacecraftStateMsg>; 2],
    /// Upstream `plasmaFluxInMsg`: the ambient plasma spectrum.
    pub plasma_flux_in_msg: Input<PlasmaFluxMsg>,
    /// Upstream `eBeamInMsgs` (optional): the electron beams; only the
    /// servicer's is used.
    pub electron_beam_in_msgs: [Input<ElectronBeamMsg>; 2],
    /// Upstream `scSunlitAreaInMsgs` (optional): sunlit areas.
    pub sunlit_area_in_msgs: [Input<ProjectedAreaMsg>; 2],
    /// Upstream `voltOutMsgs`: the equilibrium potentials.
    pub volt_out_msgs: [Output<VoltMsg>; 2],
    /// Upstream `currentsOutMsgs`: every current at the equilibrium.
    pub currents_out_msgs: [Output<ScChargingCurrentsMsg>; 2],
}

impl SpacecraftChargingEquilibrium {
    /// # Panics
    ///
    /// If the configuration is invalid (upstream's setter `bskError`s).
    pub fn new(config: SpacecraftChargingEquilibriumConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid SpacecraftChargingEquilibriumConfig: {message}");
        }
        Self {
            config,
            spacecraft_state_in_msgs: Default::default(),
            plasma_flux_in_msg: Input::default(),
            electron_beam_in_msgs: Default::default(),
            sunlit_area_in_msgs: Default::default(),
            volt_out_msgs: Default::default(),
            currents_out_msgs: Default::default(),
        }
    }

    /// Upstream `Reset`'s checks: the plasma input and both spacecraft.
    pub fn configuration_errors(&self) -> Vec<EquilibriumInputNotConnected> {
        let mut errors = Vec::new();
        if !self.plasma_flux_in_msg.is_connected() {
            errors.push(EquilibriumInputNotConnected("plasma_flux_in_msg"));
        }
        for (input, port) in self
            .spacecraft_state_in_msgs
            .iter()
            .zip(["spacecraft_state_in_msgs[0]", "spacecraft_state_in_msgs[1]"])
        {
            if !input.is_connected() {
                errors.push(EquilibriumInputNotConnected(port));
            }
        }
        errors
    }

    /// Upstream `readMessages`: the grid of strictly increasing positive
    /// energies at the front of the plasma message, or `None` with fewer than
    /// two.
    fn plasma_currents(&self) -> Option<Currents<'_>> {
        let plasma = self.plasma_flux_in_msg.read();
        let mut count = 0;
        while count < MAX_PLASMA_FLUX_SIZE {
            let energy = plasma.energies_ev[count];
            if !energy.is_finite() || energy <= 0.0 {
                break;
            }
            if count > 0 && energy <= plasma.energies_ev[count - 1] {
                break;
            }
            count += 1;
        }
        (count >= 2).then(|| Currents {
            config: &self.config,
            energies: plasma.energies_ev[..count].to_vec(),
            electron_flux: plasma.mean_electron_flux[..count].to_vec(),
            ion_flux: plasma.mean_ion_flux[..count].to_vec(),
        })
    }

    /// The sunlit area of spacecraft `index`: the input when connected and
    /// valid, else the default, else the fallback.
    fn sunlit_area_m2(&self, index: usize) -> f64 {
        let default = self.config.sunlit_area_defaults_m2[index]
            .unwrap_or(self.config.sunlit_area_fallback_m2);
        let input = &self.sunlit_area_in_msgs[index];
        if !input.is_connected() {
            return default;
        }
        let area = input.read().area_m2;
        if area.is_finite() && area >= 0.0 {
            area
        } else {
            log::warn!(
                "{}: spacecraft {index} sunlit area message was invalid ({area}); using \
                 {default} m^2",
                self.config.name
            );
            default
        }
    }

    /// The servicer's electron gun, with `alpha` clamped to \[0, 1\] (1 when
    /// not finite).
    fn servicer_gun(&self) -> ElectronGun {
        let input = &self.electron_beam_in_msgs[SERVICER];
        if !input.is_connected() {
            return ElectronGun::default();
        }
        let beam = input.read();
        let alpha = if !beam.alpha.is_finite() {
            log::warn!(
                "{}: non-finite beam alpha ({}); using 1.0",
                self.config.name,
                beam.alpha
            );
            1.0
        } else {
            let clamped = beam.alpha.clamp(0.0, 1.0);
            if clamped != beam.alpha {
                log::warn!(
                    "{}: beam alpha {} is outside [0, 1]; clamping to {clamped}",
                    self.config.name,
                    beam.alpha
                );
            }
            clamped
        };
        ElectronGun {
            alpha,
            current_a: beam.current_a,
            energy_ev: beam.energy_ev,
        }
    }

    /// Upstream `UpdateState`'s solve: the potentials and current breakdowns,
    /// or the reason the update cannot be made.
    fn solve(&self) -> Result<[(f64, ScChargingCurrentsMsg); 2], String> {
        let currents = self.plasma_currents().ok_or(
            "plasma_flux_in_msg.energies must contain at least 2 strictly increasing positive bins",
        )?;
        let bins = currents.energies.len();
        if [
            &self.config.yield_see_electron,
            &self.config.yield_see_ion,
            &self.config.yield_backscattered,
        ]
        .iter()
        .any(|table| table.len() < bins)
        {
            return Err(format!(
                "yield tables must have at least {bins} entries to match the plasma energy grid"
            ));
        }

        let area_m2 = self.config.surface_area_m2;
        let sunlit_m2 = [self.sunlit_area_m2(SERVICER), self.sunlit_area_m2(TARGET)];
        let gun = self.servicer_gun();
        let bracket = [
            self.config.root_solve_lower_bound_v,
            self.config.root_solve_upper_bound_v,
        ];
        let target_reference_v = 0.0;

        let phi_s = bisection_solve(bracket, SERVICER_SOLVE_ACCURACY, |phi_s| {
            let total = currents
                .non_beam(phi_s, area_m2, sunlit_m2[SERVICER])
                .iter()
                .sum::<f64>()
                + currents.electron_beam_current(phi_s, target_reference_v, Craft::Servicer, &gun);
            log::debug!("servicer phi {phi_s} V: total current {total} A");
            total
        })
        .map_err(|error| format!("servicer root solve failed: {error}"))?;

        let target_total = |phi_t: f64| {
            currents
                .non_beam(phi_t, area_m2, sunlit_m2[TARGET])
                .iter()
                .sum::<f64>()
                + currents.electron_beam_current(phi_s, phi_t, Craft::Target, &gun)
                + currents.beam_emission(phi_s, phi_t, &gun, &self.config.yield_see_electron)
                + currents.beam_emission(phi_s, phi_t, &gun, &self.config.yield_backscattered)
        };
        let phi_t = bisection_solve(bracket, TARGET_SOLVE_ACCURACY, |phi_t| {
            let total = target_total(phi_t);
            log::debug!("target phi {phi_t} V: total current {total} A");
            total
        })
        .map_err(|error| format!("target root solve failed: {error}"))?;
        log::debug!("equilibrium: servicer {phi_s} V, target {phi_t} V");

        let breakdown = |phi: f64, sunlit: f64, craft: Craft| {
            let [
                electron,
                ion,
                see_electron,
                see_ion,
                backscattering,
                photoelectric,
            ] = currents.non_beam(phi, area_m2, sunlit);
            let non_beam = electron + ion + see_electron + see_ion + backscattering + photoelectric;
            let (beam, beam_see, beam_backscattering) = match craft {
                Craft::Servicer => (
                    currents.electron_beam_current(
                        phi_s,
                        target_reference_v,
                        Craft::Servicer,
                        &gun,
                    ),
                    0.0,
                    0.0,
                ),
                Craft::Target => (
                    currents.electron_beam_current(phi_s, phi_t, Craft::Target, &gun),
                    currents.beam_emission(phi_s, phi_t, &gun, &self.config.yield_see_electron),
                    currents.beam_emission(phi_s, phi_t, &gun, &self.config.yield_backscattered),
                ),
            };
            ScChargingCurrentsMsg {
                electron_current_a: electron,
                ion_current_a: ion,
                see_electron_current_a: see_electron,
                see_ion_current_a: see_ion,
                backscattering_current_a: backscattering,
                photoelectric_current_a: photoelectric,
                non_beam_current_a: non_beam,
                beam_current_a: beam,
                beam_see_current_a: beam_see,
                beam_backscattering_current_a: beam_backscattering,
                total_current_a: non_beam + beam + beam_see + beam_backscattering,
            }
        };
        Ok([
            (
                phi_s,
                breakdown(phi_s, sunlit_m2[SERVICER], Craft::Servicer),
            ),
            (phi_t, breakdown(phi_t, sunlit_m2[TARGET], Craft::Target)),
        ])
    }
}

impl Module for SpacecraftChargingEquilibrium {
    fn init(&mut self) {
        for error in self.configuration_errors() {
            log::error!("{}: {error}", self.config.name);
        }
        for index in [SERVICER, TARGET] {
            if !self.sunlit_area_in_msgs[index].is_connected()
                && self.config.sunlit_area_defaults_m2[index].is_none()
            {
                log::warn!(
                    "{}: sunlit area not provided for spacecraft {index} (no input connected \
                     and no default set); using the fallback {} m^2",
                    self.config.name,
                    self.config.sunlit_area_fallback_m2
                );
            }
        }
        for output in &self.volt_out_msgs {
            output.write(VoltMsg::default());
        }
        for output in &self.currents_out_msgs {
            output.write(ScChargingCurrentsMsg::default());
        }
    }

    fn update(&mut self, _context: &SimulationContext) {
        match self.solve() {
            Ok(solution) => {
                for ((volt, currents), (phi, breakdown)) in self
                    .volt_out_msgs
                    .iter()
                    .zip(&self.currents_out_msgs)
                    .zip(solution)
                {
                    volt.write(VoltMsg { voltage_v: phi });
                    currents.write(breakdown);
                }
            }
            Err(error) => log::error!("{}: {error}", self.config.name),
        }
    }
}

/// Port of upstream `test_spacecraftChargingEquilibrium.py`, with its Python
/// reimplementation of the module (`SpacecraftChargingEquilibriumCppReference`)
/// as the truth.
#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{
        EquilibriumInputNotConnected, SERVICER, SpacecraftChargingEquilibrium,
        SpacecraftChargingEquilibriumConfig, TARGET,
    };
    use crate::messages::{
        ElectronBeamMsg, MAX_PLASMA_FLUX_SIZE, Output, PlasmaFluxMsg, ProjectedAreaMsg,
        ScChargingCurrentsMsg, SpacecraftStateMsg,
    };
    use crate::{Module, SimulationContext};

    // Upstream `Support/*.txt`, comma separated.
    const PARTICLE_ENERGIES: &str = "\
        1,1.32571136559011,1.75751062485479,2.32995181051537,3.08884359647748,\
        4.09491506238043,5.42867543932386,7.19685673001152,9.54095476349994,12.648552168553,\
        16.7683293681101,22.2299648252619,29.4705170255181,39.0693993705462,\
        51.7947467923121,68.66488450043,91.0298177991522,120.679264063933,159.985871960606,\
        212.095088792019,281.176869797423,372.759372031494,494.171336132383,\
        655.128556859551,868.511373751353,1151.39539932645,1526.41796717523,\
        2023.58964772516,2682.69579527972,3556.48030622313,4714.86636345739,\
        6250.55192527398,8286.42772854684,10985.4114198756,14563.4847750124,\
        19306.9772888325,25595.4792269953,33932.2177189533,44984.3266896944,\
        59636.2331659465,79060.432109077,104811.313415469,138949.549437314,184206.996932672,\
        244205.309454865,323745.754281765,429193.426012878,568986.602901829,\
        754312.006335462,1000000";
    const ELECTRON_FLUX: &str = "\
        142538.021826004,188959.146648717,250496.274506758,332069.916899005,\
        440201.023378213,583530.574997596,773507.137901355,1025296.11716565,1358981.2242574,\
        1801150.356706,2386985.87083745,3163014.36819607,4190714.743701,5551235.91751145,\
        7351536.50920867,9732323.74600166,12878228.0972002,17030682.1386727,\
        22503935.2382186,29704452.5986442,39153481.0305026,51511580.8657313,67602041.679657,\
        88426728.1240205,115162200.290635,149114892.319781,191600871.221157,\
        243698701.974369,305807652.300501,376941550.21243,453732201.664613,529261457.357464,\
        592159151.763303,626913970.84883,616825238.034412,550729570.674974,432313603.206208,\
        286108825.053946,151004911.049457,59043198.1775204,15510780.5000492,\
        2405066.89424557,185383.970437304,5657.17373367019,50.540223508356,\
        0.0885974960705026,1.79304806198985e-05,2.07375350403455e-10,5.3958032677695e-17,\
        9.17825444310256e-26";
    const ION_FLUX: &str = "\
        181.857253292203,241.087565726719,319.607847767992,423.699534439274,\
        561.688838688543,744.611882610353,987.095805932138,1308.5256956304,1734.5895492537,\
        2299.32285139829,3047.81276992537,4039.77203589902,5354.25737906018,7095.8905836982,\
        9403.04575051127,12458.5978981802,16503.9878654545,21857.5449725614,\
        28938.2120284789,38296.0101913297,50650.7054220298,66940.078029223,88378.7359098572,\
        116527.164184155,153368.004877825,201381.314933719,263601.044673634,\
        343618.724430703,445474.184180104,573334.134426357,730807.837854701,\
        919695.472275884,1137944.74931567,1376689.91601591,1616624.89719652,\
        1824842.01802494,1954771.00905009,1953496.69935292,1780550.13159026,\
        1436481.58284242,985800.331692948,545930.563771116,227514.199178404,\
        65040.5604963729,11280.9918879578,1008.82078109747,37.4868135511385,\
        0.434824259189975,0.00107753290100978,3.45063088397749e-07";
    const YIELD_SEE_ELECTRON: &str = "\
        0.0185619483542123,0.0245639535380386,0.0324922551779598,0.0429554946248849,\
        0.0567481140257798,0.0749024666419679,0.0987520317395966,0.130005076642659,\
        0.170824667882257,0.223903966205012,0.292512564348373,0.380466295745115,\
        0.491934890026278,0.630947116423562,0.800391334161561,1.0002808653985,\
        1.22515741425904,1.46091973288411,1.68229681411935,1.85362312824905,\
        1.93663665692856,1.90700658246898,1.7725305962027,1.57428664490212,1.36027115486988,\
        1.1580990370266,0.976225323456431,0.816306623472782,0.678005231274042,\
        0.55998214854873,0.460356911962533,0.377013357856168,0.307802429132152,\
        0.250671479655205,0.203739839457937,0.165335580239064,0.134005542290243,\
        0.108508240747331,0.0877969243736941,0.0709979328823207,0.0573877406630616,\
        0.0463707412330848,0.0374588870207738,0.0302536792311926,0.0244306198574293,\
        0.0197260194995043,0.0159259437726193,0.012857036919323,0.0103789560870877,\
        0.00837816565270477";
    const YIELD_SEE_ION: &str = "\
        0.0154318478866309,0.0177681334977816,0.0204581087563757,0.0235553136752343,\
        0.0271213903084352,0.0312273074310733,0.0359547696129978,0.0413978380677476,\
        0.0476647945217221,0.0548802836493134,0.0631877743456504,0.0727523852055212,\
        0.0837641249204219,0.0964416036576083,0.111036276428005,0.12783728328528,\
        0.147176952792104,0.169437032737413,0.195055702763596,0.224535402987466,\
        0.258451474164244,0.29746153826377,0.342315438195397,0.393865379095275,\
        0.453075637790964,0.521030783987241,0.598940719575238,0.68813990158506,\
        0.790076756847741,0.906287396230308,1.03834518681195,1.18777453452805,\
        1.35591362531963,1.54370769453735,1.7514135166265,1.97820077521706,2.22165246357011,\
        2.47720172345053,2.73760212467971,2.99260825362291,3.22911745634141,\
        3.43203426460577,3.58599141186135,3.67775918149205,3.69878252205291,\
        3.64702961091208,3.52745369156195,3.35088235909706,3.13177379390712,\
        2.88564263883983";
    const YIELD_BACKSCATTERED: &str = "\
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.00471177401789873,0.042347070475594,0.079898246154131,\
        0.117325277281387,0.154571663822451,0.191558525200946,0.228177079455233,\
        0.264279452675775,0.299668162370585,0.334085420935377,0.367204857351251,\
        0.380714768585934,0.374975015142897,0.368000076259533,0.359760433607396,\
        0.35038431999662,0.34023039079649,0.329930896350622,0.320349159555133,\
        0.312396271335776,0.306716479996372,0.303387441127111,0.301881721933531,\
        0.30139647675476,0.301295958800736,0.301284239863521,0.301283592654777,\
        0.30128357915357,0.301283579074914,0.301283579074828,0.301283579074828,\
        0.301283579074828,0.301283579074828,0.301283579074828,0.301283579074828,\
        0.301283579074828";

    const Q0: f64 = 1.60217663e-19;
    const TRAPZN: usize = 1000;
    const JPH: f64 = 40e-6;
    const KTPH: f64 = 2.0;
    const TSEE: f64 = 5.0;
    const TEB: f64 = 20.0;
    const DEFAULT_AREA: f64 = 50.264;
    const DEFAULT_SUNLIT_AREA: f64 = 0.0;
    const ROOT_BRACKET: (f64, f64) = (-400000.0, 400000.0);

    fn parse(text: &str) -> Vec<f64> {
        text.trim()
            .split(',')
            .map(|value| value.trim().parse().unwrap())
            .collect()
    }

    struct SupportData {
        energies: Vec<f64>,
        electron_flux: Vec<f64>,
        ion_flux: Vec<f64>,
        yield_see_electron: Vec<f64>,
        yield_see_ion: Vec<f64>,
        yield_backscattered: Vec<f64>,
    }

    fn support_data() -> SupportData {
        SupportData {
            energies: parse(PARTICLE_ENERGIES),
            electron_flux: parse(ELECTRON_FLUX),
            ion_flux: parse(ION_FLUX),
            yield_see_electron: parse(YIELD_SEE_ELECTRON),
            yield_see_ion: parse(YIELD_SEE_ION),
            yield_backscattered: parse(YIELD_BACKSCATTERED),
        }
    }

    #[derive(Clone, Copy, Debug)]
    struct Beam {
        energy_eb: f64,
        current_eb: f64,
        alpha_eb: f64,
    }

    /// The test's `SpacecraftChargingEquilibriumCppReference`.
    struct Reference {
        energies: Vec<f64>,
        electron_flux: Vec<f64>,
        ion_flux: Vec<f64>,
        yield_see_electron: Vec<f64>,
        yield_see_ion: Vec<f64>,
        yield_backscattered: Vec<f64>,
    }

    #[derive(Clone, Copy, PartialEq, Eq)]
    enum Particle {
        Electron,
        Ion,
    }

    #[derive(Clone, Copy, PartialEq, Eq)]
    enum Yield {
        Electron,
        Ion,
        Backscattered,
    }

    impl Reference {
        fn interp(x_vector: &[f64], y_vector: &[f64], x: f64) -> f64 {
            let mut idx1 = x_vector
                .iter()
                .position(|&value| value > x)
                .map_or(-1, |i| i as i64);
            if idx1 == -1 {
                idx1 = x_vector.len() as i64 - 1;
            } else if idx1 == 0 {
                idx1 = 1;
            }
            let (i0, i1) = ((idx1 - 1) as usize, idx1 as usize);
            y_vector[i0]
                + ((y_vector[i1] - y_vector[i0]) / (x_vector[i1] - x_vector[i0]))
                    * (x - x_vector[i0])
        }

        fn trapz(f: impl Fn(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
            if !a.is_finite() || !b.is_finite() || n == 0 || b == a {
                return 0.0;
            }
            let h = (b - a) / n as f64;
            let mut total = 0.0;
            for i in 1..n {
                total += f(a + i as f64 * h);
            }
            h * (0.5 * f(a) + total + 0.5 * f(b))
        }

        fn get_flux(&self, energy: f64, particle: Particle) -> f64 {
            let flux = match particle {
                Particle::Electron => &self.electron_flux,
                Particle::Ion => &self.ion_flux,
            };
            Self::interp(&self.energies, flux, energy).max(0.0)
        }

        fn get_yield(&self, energy: f64, yield_type: Yield) -> f64 {
            let table = match yield_type {
                Yield::Electron => &self.yield_see_electron,
                Yield::Ion => &self.yield_see_ion,
                Yield::Backscattered => &self.yield_backscattered,
            };
            Self::interp(&self.energies, table, energy).max(0.0)
        }

        fn last_energy(&self) -> f64 {
            self.energies[self.energies.len() - 1]
        }

        fn electron_current(&self, phi: f64, area: f64) -> f64 {
            let (lower, upper) = if phi < 0.0 {
                (0.1, self.last_energy())
            } else {
                (0.1 + phi.abs(), self.last_energy() + phi.abs())
            };
            let integral = Self::trapz(
                |e| (e / (e - phi)) * self.get_flux(e - phi, Particle::Electron),
                lower,
                upper,
                TRAPZN,
            );
            -Q0 * area * integral
        }

        fn ion_current(&self, phi: f64, area: f64) -> f64 {
            let (lower, upper) = if phi > 0.0 {
                (0.1, self.last_energy())
            } else {
                (0.1 + phi.abs(), self.last_energy() + phi.abs())
            };
            let integral = Self::trapz(
                |e| (e / (e + phi)) * self.get_flux(e + phi, Particle::Ion),
                lower,
                upper,
                TRAPZN,
            );
            Q0 * area * integral
        }

        fn see_electron_current(&self, phi: f64, area: f64) -> f64 {
            let (lower, upper) = if phi < 0.0 {
                (0.1, self.last_energy())
            } else {
                (0.1 + phi.abs(), self.last_energy() + phi.abs())
            };
            let integral = Self::trapz(
                |e| {
                    self.get_yield(e, Yield::Electron)
                        * (e / (e - phi))
                        * self.get_flux(e - phi, Particle::Electron)
                },
                lower,
                upper,
                TRAPZN,
            );
            let i_see_e = Q0 * area * integral;
            if phi <= 0.0 {
                i_see_e
            } else {
                i_see_e * (-phi / TSEE).exp()
            }
        }

        fn see_ion_current(&self, phi: f64, area: f64) -> f64 {
            let (lower, upper) = if phi > 0.0 {
                (0.1, self.last_energy())
            } else {
                (0.1 + phi.abs(), self.last_energy() + phi.abs())
            };
            let integral = Self::trapz(
                |e| {
                    self.get_yield(e, Yield::Ion)
                        * (e / (e + phi))
                        * self.get_flux(e + phi, Particle::Ion)
                },
                lower,
                upper,
                TRAPZN,
            );
            let i_see_i = Q0 * area * integral;
            if phi <= 0.0 {
                i_see_i
            } else {
                i_see_i * (-phi / TSEE).exp()
            }
        }

        fn backscattering_current(&self, phi: f64, area: f64) -> f64 {
            let (lower, upper) = if phi < 0.0 {
                (0.1, self.last_energy())
            } else {
                (0.1 + phi.abs(), self.last_energy() + phi.abs())
            };
            let integral = Self::trapz(
                |e| {
                    self.get_yield(e, Yield::Backscattered)
                        * (e / (e - phi))
                        * self.get_flux(e - phi, Particle::Electron)
                },
                lower,
                upper,
                TRAPZN,
            );
            let i_bs = Q0 * area * integral;
            if phi <= 0.0 {
                i_bs
            } else {
                i_bs * (-phi / TSEE).exp()
            }
        }

        fn photoelectric_current(phi: f64, area: f64) -> f64 {
            if phi > 0.0 {
                JPH * area * (-phi / KTPH).exp()
            } else {
                JPH * area
            }
        }

        fn electron_beam_current(phi_s: f64, phi_t: f64, servicer: bool, beam: &Beam) -> f64 {
            if beam.energy_eb > phi_s - phi_t {
                let turn_on = 1.0 - (-(beam.energy_eb - phi_s + phi_t) / TEB).exp();
                if servicer {
                    beam.current_eb * turn_on
                } else {
                    -beam.alpha_eb * beam.current_eb * turn_on
                }
            } else {
                0.0
            }
        }

        fn see_electron_beam_current(&self, phi_s: f64, phi_t: f64, beam: &Beam) -> f64 {
            let e_eff = beam.energy_eb - phi_s + phi_t;
            let incoming = (-Self::electron_beam_current(phi_s, phi_t, false, beam)).max(0.0);
            self.get_yield(e_eff, Yield::Electron) * incoming
        }

        fn electron_beam_backscattering(&self, phi_s: f64, phi_t: f64, beam: &Beam) -> f64 {
            let e_eff = beam.energy_eb - phi_s + phi_t;
            let incoming = (-Self::electron_beam_current(phi_s, phi_t, false, beam)).max(0.0);
            self.get_yield(e_eff, Yield::Backscattered) * incoming
        }

        fn non_beam(&self, phi: f64, area: f64, sunlit_area: f64) -> f64 {
            self.electron_current(phi, area)
                + self.ion_current(phi, area)
                + self.see_electron_current(phi, area)
                + self.see_ion_current(phi, area)
                + self.backscattering_current(phi, area)
                + Self::photoelectric_current(phi, sunlit_area)
        }

        fn sum_currents_servicer(
            &self,
            phi_s: f64,
            area: f64,
            sunlit_area: f64,
            beam: &Beam,
        ) -> f64 {
            self.non_beam(phi_s, area, sunlit_area)
                + Self::electron_beam_current(phi_s, 0.0, true, beam)
        }

        fn sum_currents_target(
            &self,
            phi_t: f64,
            phi_s: f64,
            area: f64,
            sunlit_area: f64,
            beam: &Beam,
        ) -> f64 {
            self.non_beam(phi_t, area, sunlit_area)
                + Self::electron_beam_current(phi_s, phi_t, false, beam)
                + self.see_electron_beam_current(phi_s, phi_t, beam)
                + self.electron_beam_backscattering(phi_s, phi_t, beam)
        }

        fn bisection_solve(interval: (f64, f64), accuracy: f64, f: impl Fn(f64) -> f64) -> f64 {
            let (mut left, mut right) = interval;
            let mut current_estimate = (left + right) / 2.0;
            if left > right {
                return f64::NAN;
            }
            let mut f_left = f(left);
            let f_right = f(right);
            if f_left == 0.0 {
                return left;
            }
            if f_right == 0.0 {
                return right;
            }
            if f_left * f_right > 0.0 {
                return f64::NAN;
            }
            while right - left >= accuracy {
                current_estimate = (left + right) / 2.0;
                let f_mid = f(current_estimate);
                if f_mid == 0.0 {
                    break;
                }
                if (f_left < 0.0 && f_mid > 0.0) || (f_left > 0.0 && f_mid < 0.0) {
                    right = current_estimate;
                } else {
                    left = current_estimate;
                    f_left = f_mid;
                }
            }
            current_estimate
        }

        fn solve_coupled_equilibrium(
            &self,
            beam: &Beam,
            servicer_sunlit_area: f64,
            target_sunlit_area: f64,
        ) -> (f64, f64) {
            let phi_s = Self::bisection_solve(ROOT_BRACKET, 1e-6, |phi| {
                self.sum_currents_servicer(phi, DEFAULT_AREA, servicer_sunlit_area, beam)
            });
            let phi_t = Self::bisection_solve(ROOT_BRACKET, 1e-8, |phi| {
                self.sum_currents_target(phi, phi_s, DEFAULT_AREA, target_sunlit_area, beam)
            });
            (phi_s, phi_t)
        }

        /// The test's `_python_target_breakdown`.
        fn target_breakdown(
            &self,
            phi_t: f64,
            phi_s: f64,
            beam: &Beam,
            sunlit_area: f64,
        ) -> ScChargingCurrentsMsg {
            let area = DEFAULT_AREA;
            let i_e = self.electron_current(phi_t, area);
            let i_i = self.ion_current(phi_t, area);
            let i_see_e = self.see_electron_current(phi_t, area);
            let i_see_i = self.see_ion_current(phi_t, area);
            let i_bs = self.backscattering_current(phi_t, area);
            let i_ph = Self::photoelectric_current(phi_t, sunlit_area);
            let env_i = i_e + i_i + i_see_e + i_see_i + i_bs + i_ph;
            let beam_arrival_i = Self::electron_beam_current(phi_s, phi_t, false, beam);
            let beam_see_i = self.see_electron_beam_current(phi_s, phi_t, beam);
            let beam_bs_i = self.electron_beam_backscattering(phi_s, phi_t, beam);
            ScChargingCurrentsMsg {
                electron_current_a: i_e,
                ion_current_a: i_i,
                see_electron_current_a: i_see_e,
                see_ion_current_a: i_see_i,
                backscattering_current_a: i_bs,
                photoelectric_current_a: i_ph,
                non_beam_current_a: env_i,
                beam_current_a: beam_arrival_i,
                beam_see_current_a: beam_see_i,
                beam_backscattering_current_a: beam_bs_i,
                total_current_a: env_i + beam_arrival_i + beam_see_i + beam_bs_i,
            }
        }
    }

    /// Zero-padded copy of a table, as the plasma message holds it.
    fn padded(values: &[f64]) -> [f64; MAX_PLASMA_FLUX_SIZE] {
        let mut out = [0.0; MAX_PLASMA_FLUX_SIZE];
        out[..values.len()].copy_from_slice(values);
        out
    }

    struct ModuleRun {
        servicer_v: f64,
        target_v: f64,
        target_currents: ScChargingCurrentsMsg,
    }

    /// The test's `_run_cpp_module_once`: one update of the module with the
    /// servicer 10 m from the target at GEO noon.
    #[allow(clippy::too_many_arguments)]
    fn run_module_once(
        energies: &[f64],
        electron_flux: &[f64],
        ion_flux: &[f64],
        data: &SupportData,
        beam: Beam,
        servicer_sunlit_area: Option<f64>,
        target_sunlit_area: Option<f64>,
    ) -> ModuleRun {
        let mut module =
            SpacecraftChargingEquilibrium::new(SpacecraftChargingEquilibriumConfig::new(
                data.yield_see_electron.clone(),
                data.yield_see_ion.clone(),
                data.yield_backscattered.clone(),
            ));

        let lt: f64 = 12.0;
        let angle = lt * 360.0 / 24.0 * std::f64::consts::PI / 180.0 - std::f64::consts::PI;
        let orbit_radius = 42000.0e3;
        let r_target = Vector3::new(orbit_radius * angle.cos(), orbit_radius * angle.sin(), 0.0);
        let r_servicer = r_target + Vector3::new(0.0, 10.0, 0.0);
        let states = [r_servicer, r_target].map(|position_m| {
            Output::new(SpacecraftStateMsg {
                position_m,
                ..Default::default()
            })
        });
        let plasma = Output::new(PlasmaFluxMsg {
            mean_electron_flux: padded(electron_flux),
            mean_ion_flux: padded(ion_flux),
            energies_ev: padded(energies),
        });
        let beam_out = Output::new(ElectronBeamMsg {
            energy_ev: beam.energy_eb,
            current_a: beam.current_eb,
            alpha: beam.alpha_eb,
        });
        for (input, state) in module.spacecraft_state_in_msgs.iter_mut().zip(&states) {
            input.connect(state.slot());
        }
        module.plasma_flux_in_msg.connect(plasma.slot());
        module.electron_beam_in_msgs[SERVICER].connect(beam_out.slot());
        let sunlit: Vec<_> = [servicer_sunlit_area, target_sunlit_area]
            .into_iter()
            .map(|area| area.map(|area_m2| Output::new(ProjectedAreaMsg { area_m2 })))
            .collect();
        for (input, output) in module.sunlit_area_in_msgs.iter_mut().zip(&sunlit) {
            if let Some(output) = output {
                input.connect(output.slot());
            }
        }

        module.init();
        module.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });
        ModuleRun {
            servicer_v: module.volt_out_msgs[SERVICER].read().voltage_v,
            target_v: module.volt_out_msgs[TARGET].read().voltage_v,
            target_currents: module.currents_out_msgs[TARGET].read(),
        }
    }

    fn reference(data: &SupportData, scale_electron_flux: f64, scale_ion_flux: f64) -> Reference {
        Reference {
            energies: data.energies.clone(),
            electron_flux: data
                .electron_flux
                .iter()
                .map(|f| f * scale_electron_flux)
                .collect(),
            ion_flux: data.ion_flux.iter().map(|f| f * scale_ion_flux).collect(),
            yield_see_electron: data.yield_see_electron.clone(),
            yield_see_ion: data.yield_see_ion.clone(),
            yield_backscattered: data.yield_backscattered.clone(),
        }
    }

    /// The test's `_compare_python_reference_to_cpp`; returns the module's
    /// and the reference's target potentials.
    fn compare_reference_to_module(
        data: &SupportData,
        beam: Beam,
        scale_electron_flux: f64,
        scale_ion_flux: f64,
        servicer_sunlit_area: f64,
        target_sunlit_area: f64,
    ) -> (f64, f64) {
        let reference = reference(data, scale_electron_flux, scale_ion_flux);
        let run = run_module_once(
            &reference.energies,
            &reference.electron_flux,
            &reference.ion_flux,
            data,
            beam,
            Some(servicer_sunlit_area),
            Some(target_sunlit_area),
        );
        let (phi_s, phi_t) =
            reference.solve_coupled_equilibrium(&beam, servicer_sunlit_area, target_sunlit_area);

        let case = format!("{beam:?}, flux scales ({scale_electron_flux}, {scale_ion_flux})");
        for value in [run.servicer_v, run.target_v, phi_s, phi_t] {
            assert!(value.is_finite(), "{case}");
        }
        assert!(
            (run.servicer_v - phi_s).abs() <= 1e-3,
            "servicer {} vs {phi_s}, {case}",
            run.servicer_v
        );
        assert!(
            (run.target_v - phi_t).abs() <= 1e-3,
            "target {} vs {phi_t}, {case}",
            run.target_v
        );
        (run.target_v, phi_t)
    }

    const NOMINAL_BEAM: Beam = Beam {
        energy_eb: 33000.0,
        current_eb: 4.5e-6,
        alpha_eb: 1.0,
    };

    /// Upstream `test_python_reimplementation_matches_cpp_module`,
    /// `test_cpp_vs_python_plasma_scaling_cases` and
    /// `test_plasma_scaling_trends_target_potential`.
    #[test]
    fn potentials_match_the_reference_across_plasma_scalings() {
        let data = support_data();
        let run = |electron, ion| {
            compare_reference_to_module(
                &data,
                NOMINAL_BEAM,
                electron,
                ion,
                DEFAULT_SUNLIT_AREA,
                DEFAULT_SUNLIT_AREA,
            )
        };
        let base = run(1.0, 1.0);
        let more_electrons = run(2.0, 1.0);
        let fewer_electrons = run(0.5, 1.0);
        let more_ions = run(1.0, 2.0);
        let fewer_ions = run(1.0, 0.5);

        // More ambient electrons make the target more negative, more ions
        // more positive; both for the module (.0) and the reference (.1).
        assert!(more_electrons.0 < base.0 && base.0 < fewer_electrons.0);
        assert!(more_electrons.1 < base.1 && base.1 < fewer_electrons.1);
        assert!(fewer_ions.0 < base.0 && base.0 < more_ions.0);
        assert!(fewer_ions.1 < base.1 && base.1 < more_ions.1);
    }

    /// Upstream `test_cpp_vs_python_zero_padded_plasma_bins`.
    #[test]
    fn trailing_zero_bins_are_ignored() {
        let data = support_data();
        let valid_bins = 30;
        let run = run_module_once(
            &data.energies[..valid_bins],
            &data.electron_flux[..valid_bins],
            &data.ion_flux[..valid_bins],
            &data,
            NOMINAL_BEAM,
            None,
            None,
        );
        let reference = Reference {
            energies: data.energies[..valid_bins].to_vec(),
            electron_flux: data.electron_flux[..valid_bins].to_vec(),
            ion_flux: data.ion_flux[..valid_bins].to_vec(),
            yield_see_electron: data.yield_see_electron[..valid_bins].to_vec(),
            yield_see_ion: data.yield_see_ion[..valid_bins].to_vec(),
            yield_backscattered: data.yield_backscattered[..valid_bins].to_vec(),
        };
        let (phi_s, phi_t) = reference.solve_coupled_equilibrium(
            &NOMINAL_BEAM,
            DEFAULT_SUNLIT_AREA,
            DEFAULT_SUNLIT_AREA,
        );
        assert!(
            (run.servicer_v - phi_s).abs() <= 1e-3,
            "{} vs {phi_s}",
            run.servicer_v
        );
        assert!(
            (run.target_v - phi_t).abs() <= 1e-3,
            "{} vs {phi_t}",
            run.target_v
        );
    }

    /// Upstream `test_cpp_vs_python_beam_parameter_cases`.
    #[test]
    fn potentials_match_the_reference_across_beams() {
        let data = support_data();
        for (energy_eb, current_eb, alpha_eb) in [
            (22000.0, 2.0e-6, 0.8),
            (33000.0, 4.5e-6, 1.0),
            (45000.0, 7.0e-6, 0.6),
            (40000.0, 5.0e-4, 0.95),
            (33000.0, 0.0, 1.0),
        ] {
            let beam = Beam {
                energy_eb,
                current_eb,
                alpha_eb,
            };
            compare_reference_to_module(
                &data,
                beam,
                1.0,
                1.0,
                DEFAULT_SUNLIT_AREA,
                DEFAULT_SUNLIT_AREA,
            );
        }
    }

    /// Upstream `test_cpp_requires_exactly_two_spacecraft`: the two
    /// spacecraft are fixed ports here, so the counterpart is an unconnected
    /// target state.
    #[test]
    fn both_spacecraft_states_are_required() {
        let data = support_data();
        let mut module =
            SpacecraftChargingEquilibrium::new(SpacecraftChargingEquilibriumConfig::new(
                data.yield_see_electron.clone(),
                data.yield_see_ion.clone(),
                data.yield_backscattered.clone(),
            ));
        let servicer = Output::new(SpacecraftStateMsg::default());
        let plasma = Output::new(PlasmaFluxMsg::default());
        module.spacecraft_state_in_msgs[SERVICER].connect(servicer.slot());
        module.plasma_flux_in_msg.connect(plasma.slot());
        assert_eq!(
            module.configuration_errors(),
            vec![EquilibriumInputNotConnected("spacecraft_state_in_msgs[1]")]
        );

        let bare = SpacecraftChargingEquilibrium::new(SpacecraftChargingEquilibriumConfig::new(
            vec![0.0],
            vec![0.0],
            vec![0.0],
        ));
        assert_eq!(bare.configuration_errors().len(), 3);
    }

    /// Upstream `test_cpp_vs_python_sunlit_area_override_cases`.
    #[test]
    fn potentials_match_the_reference_with_sunlit_areas() {
        let data = support_data();
        for (servicer_sunlit_area, target_sunlit_area) in
            [(5.0, 10.0), (15.0, 25.132), (25.132, 40.0)]
        {
            compare_reference_to_module(
                &data,
                NOMINAL_BEAM,
                1.0,
                1.0,
                servicer_sunlit_area,
                target_sunlit_area,
            );
        }
    }

    fn assert_breakdown(
        module: &ScChargingCurrentsMsg,
        reference: &ScChargingCurrentsMsg,
        case: &str,
    ) {
        let environment_terms = [
            (
                "i_e",
                module.electron_current_a,
                reference.electron_current_a,
            ),
            ("i_i", module.ion_current_a, reference.ion_current_a),
            (
                "i_see_e",
                module.see_electron_current_a,
                reference.see_electron_current_a,
            ),
            (
                "i_see_i",
                module.see_ion_current_a,
                reference.see_ion_current_a,
            ),
            (
                "i_bs",
                module.backscattering_current_a,
                reference.backscattering_current_a,
            ),
            (
                "i_ph",
                module.photoelectric_current_a,
                reference.photoelectric_current_a,
            ),
            (
                "env_i",
                module.non_beam_current_a,
                reference.non_beam_current_a,
            ),
            ("total_i", module.total_current_a, reference.total_current_a),
        ];
        for (name, actual, expected) in environment_terms {
            assert!(
                (actual - expected).abs() <= 1e-6,
                "{name}: {actual} vs {expected}, {case}"
            );
        }
        let beam_terms = [
            (
                "beam_arrival_i",
                module.beam_current_a,
                reference.beam_current_a,
            ),
            (
                "beam_see_i",
                module.beam_see_current_a,
                reference.beam_see_current_a,
            ),
            (
                "beam_bs_i",
                module.beam_backscattering_current_a,
                reference.beam_backscattering_current_a,
            ),
        ];
        for (name, actual, expected) in beam_terms {
            assert!(
                (actual - expected).abs() <= 1e-9,
                "{name}: {actual} vs {expected}, {case}"
            );
        }
        // The landing beam is non-positive; what it releases non-negative.
        assert!(module.beam_current_a <= 1e-12, "{case}");
        assert!(module.beam_see_current_a >= -1e-12, "{case}");
        assert!(module.beam_backscattering_current_a >= -1e-12, "{case}");
    }

    /// Upstream `test_target_current_breakdown_cpp_vs_python_equilibrium_beam_cases`.
    #[test]
    fn target_current_breakdown_matches_the_reference_across_beams() {
        let data = support_data();
        let reference = reference(&data, 1.0, 1.0);
        for (energy_eb, current_eb, alpha_eb) in [
            (22000.0, 2.0e-6, 0.8),
            (33000.0, 4.5e-6, 1.0),
            (45000.0, 7.0e-6, 0.6),
        ] {
            let beam = Beam {
                energy_eb,
                current_eb,
                alpha_eb,
            };
            let run = run_module_once(
                &data.energies,
                &data.electron_flux,
                &data.ion_flux,
                &data,
                beam,
                None,
                None,
            );
            let expected = reference.target_breakdown(
                run.target_v,
                run.servicer_v,
                &beam,
                DEFAULT_SUNLIT_AREA,
            );
            assert_breakdown(&run.target_currents, &expected, &format!("{beam:?}"));
        }
    }

    /// Upstream `test_target_current_breakdown_cpp_vs_python_near_equilibrium`.
    #[test]
    fn target_current_breakdown_matches_the_reference_with_sunlit_areas() {
        let data = support_data();
        let reference = reference(&data, 1.0, 1.0);
        for target_sunlit_area in [10.0, 25.132, 40.0] {
            let run = run_module_once(
                &data.energies,
                &data.electron_flux,
                &data.ion_flux,
                &data,
                NOMINAL_BEAM,
                None,
                Some(target_sunlit_area),
            );
            let expected = reference.target_breakdown(
                run.target_v,
                run.servicer_v,
                &NOMINAL_BEAM,
                target_sunlit_area,
            );
            assert_breakdown(
                &run.target_currents,
                &expected,
                &format!("sunlit area {target_sunlit_area}"),
            );
        }
    }

    #[test]
    fn a_degenerate_plasma_grid_leaves_the_outputs_unchanged() {
        let data = support_data();
        let mut module =
            SpacecraftChargingEquilibrium::new(SpacecraftChargingEquilibriumConfig::new(
                data.yield_see_electron.clone(),
                data.yield_see_ion.clone(),
                data.yield_backscattered.clone(),
            ));
        let mut energies = [0.0; MAX_PLASMA_FLUX_SIZE];
        energies[0] = 10.0;
        let plasma = Output::new(PlasmaFluxMsg {
            energies_ev: energies,
            ..Default::default()
        });
        module.plasma_flux_in_msg.connect(plasma.slot());
        module.init();
        assert!(
            module
                .solve()
                .unwrap_err()
                .contains("at least 2 strictly increasing")
        );
        module.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });
        assert_eq!(module.volt_out_msgs[SERVICER].read().voltage_v, 0.0);
    }

    #[test]
    fn sunlit_area_input_overrides_default_and_fallback() {
        let mut config = SpacecraftChargingEquilibriumConfig::new(vec![0.0], vec![0.0], vec![0.0]);
        config.sunlit_area_fallback_m2 = 1.0;
        config.sunlit_area_defaults_m2 = [Some(2.0), None];
        let mut module = SpacecraftChargingEquilibrium::new(config);
        assert_eq!(module.sunlit_area_m2(SERVICER), 2.0);
        assert_eq!(module.sunlit_area_m2(TARGET), 1.0);
        let area = Output::new(ProjectedAreaMsg { area_m2: 3.0 });
        module.sunlit_area_in_msgs[SERVICER].connect(area.slot());
        assert_eq!(module.sunlit_area_m2(SERVICER), 3.0);
        area.write(ProjectedAreaMsg { area_m2: f64::NAN });
        assert_eq!(
            module.sunlit_area_m2(SERVICER),
            2.0,
            "an invalid area falls back"
        );
    }

    #[test]
    #[should_panic(expected = "yield_see_ion must be a non-empty table")]
    fn rejects_an_empty_yield_table() {
        SpacecraftChargingEquilibrium::new(SpacecraftChargingEquilibriumConfig::new(
            vec![0.1],
            vec![],
            vec![0.1],
        ));
    }

    #[test]
    #[should_panic(expected = "root solve bounds")]
    fn rejects_reversed_root_bounds() {
        let mut config = SpacecraftChargingEquilibriumConfig::new(vec![0.1], vec![0.1], vec![0.1]);
        config.root_solve_lower_bound_v = 1.0;
        config.root_solve_upper_bound_v = -1.0;
        SpacecraftChargingEquilibrium::new(config);
    }
}
