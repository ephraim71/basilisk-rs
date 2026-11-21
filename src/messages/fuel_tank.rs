//! Fuel tank propellant state (upstream `FuelTankMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Propellant held by one fuel tank: upstream `FuelTankMsg`, published by
/// [`FuelTank`](crate::dynamics::fuel_tank::FuelTank).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FuelTankMsg {
    /// Upstream `fuelMass` \[kg\]: propellant mass in the tank, not counting
    /// the fuel-slosh particles it drives.
    pub fuel_mass_kg: f64,
    /// Upstream `fuelMassDot` \[kg/s\]: rate of change of `fuel_mass_kg`.
    pub fuel_mass_dot_kgps: f64,
    /// Upstream `maxFuelMass` \[kg\]: propellant capacity.
    pub max_fuel_mass_kg: f64,
}

impl_telemetry!(FuelTankMsg {
    fuel_mass_kg,
    fuel_mass_dot_kgps,
    max_fuel_mass_kg,
});
