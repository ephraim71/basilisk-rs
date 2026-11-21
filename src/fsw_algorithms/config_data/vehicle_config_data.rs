//! Publishes the spacecraft's top-level mass properties for flight software.
//!
//! Port of upstream Basilisk `vehicleConfigData`. The configured inertia,
//! center of mass and mass are written to a [`VehicleConfigMsg`] on every
//! reset; updates do nothing, so the message holds those values until the next
//! reset. `current_adcs_state` is left at zero, as upstream.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{Output, VehicleConfigMsg};
use crate::{Module, SimulationContext};

/// User-settable vehicle properties; upstream zero-initializes all of them.
#[derive(Clone, Debug)]
pub struct VehicleConfigDataConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Spacecraft inertia about point B, body frame (upstream `ISCPntB_B`) \[kg m^2\].
    pub inertia_about_point_b_kg_m2: Matrix3<f64>,
    /// Center of mass in body-frame components (upstream `CoM_B`) \[m\].
    pub center_of_mass_body_m: Vector3<f64>,
    /// Spacecraft mass (upstream `massSC`) \[kg\].
    pub mass_kg: f64,
}

impl Default for VehicleConfigDataConfig {
    fn default() -> Self {
        Self {
            name: "vehicleConfigData".to_string(),
            inertia_about_point_b_kg_m2: Matrix3::zeros(),
            center_of_mass_body_m: Vector3::zeros(),
            mass_kg: 0.0,
        }
    }
}

/// Static publisher of the vehicle configuration message.
#[derive(Clone, Debug)]
pub struct VehicleConfigData {
    /// Module configuration.
    pub config: VehicleConfigDataConfig,
    /// Vehicle properties (upstream `vecConfigOutMsg`).
    pub vec_config_out_msg: Output<VehicleConfigMsg>,
}

impl VehicleConfigData {
    /// Creates the module; the message is published on init and reset.
    pub fn new(config: VehicleConfigDataConfig) -> Self {
        Self {
            config,
            vec_config_out_msg: Output::default(),
        }
    }

    fn publish_config(&self) {
        self.vec_config_out_msg.write(VehicleConfigMsg {
            inertia_about_point_b_kg_m2: self.config.inertia_about_point_b_kg_m2,
            center_of_mass_body_m: self.config.center_of_mass_body_m,
            mass_kg: self.config.mass_kg,
            ..Default::default()
        });
    }
}

impl Module for VehicleConfigData {
    fn init(&mut self) {
        self.publish_config();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.publish_config();
    }

    fn update(&mut self, _context: &SimulationContext) {}
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    use super::{VehicleConfigData, VehicleConfigDataConfig};

    /// Port of `test_vehicleConfigData.py`: 0.5 s task rate, stop at 0.5 s,
    /// both samples hold the configured values. Upstream compares only the
    /// first three inertia entries (`compareArrayND(..., 3, ...)`); all nine
    /// are checked here.
    #[test]
    fn matches_upstream_vehicle_config_data_unit_test() {
        let inertia = Matrix3::from_row_slice(&[1000., 0., 0., 0., 800., 0., 0., 0., 800.]);
        let center_of_mass = Vector3::new(1.0, 1.0, 1.0);
        let mass = 300.0;
        let mut module = VehicleConfigData::new(VehicleConfigDataConfig {
            inertia_about_point_b_kg_m2: inertia,
            center_of_mass_body_m: center_of_mass,
            mass_kg: mass,
            ..Default::default()
        });
        let mut recorder = MessageRecorder::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&module.vec_config_out_msg, &mut recorder.input_msg);
        simulation.add_module("vehicleConfigData", &mut module, 500_000_000, 0);
        simulation.add_module("recorder", &mut recorder, 500_000_000, 0);
        simulation.run_for(500_000_000);
        drop(simulation);

        let accuracy = 1e-6;
        assert_eq!(recorder.samples.len(), 2);
        for sample in &recorder.samples {
            assert!((sample.center_of_mass_body_m - center_of_mass).amax() <= accuracy);
            assert!((sample.inertia_about_point_b_kg_m2 - inertia).amax() <= accuracy);
            assert!((sample.mass_kg - mass).abs() <= accuracy);
            assert_eq!(sample.current_adcs_state, 0);
        }
    }

    #[test]
    fn reset_republishes_the_current_configuration() {
        let mut module = VehicleConfigData::new(VehicleConfigDataConfig {
            mass_kg: 300.0,
            ..Default::default()
        });
        module.init();
        module.config.mass_kg = 250.0;
        module.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });
        assert_eq!(module.vec_config_out_msg.read().mass_kg, 300.0);

        Module::reset(
            &mut module,
            &SimulationContext {
                current_sim_nanos: 1_000_000_000,
                current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
            },
        );
        assert_eq!(module.vec_config_out_msg.read().mass_kg, 250.0);
    }
}
