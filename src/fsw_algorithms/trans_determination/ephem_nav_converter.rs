//! Ephemeris to translational navigation message: port of upstream Basilisk
//! `ephemNavConverter`.
//!
//! Every update copies the time tag, position and velocity of an ephemeris
//! message into a translational navigation message, so an ephemeris source
//! can stand in for a navigation filter. The accumulated delta-v stays zero.

use crate::messages::{EphemerisMsg, Input, NavigationTranslationMsg, Output};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`EphemNavConverter`]; upstream has none.
#[derive(Clone, Debug)]
pub struct EphemNavConverterConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
}

impl Default for EphemNavConverterConfig {
    fn default() -> Self {
        Self {
            name: "ephemNavConverter".to_string(),
        }
    }
}

/// Ephemeris to navigation converter: upstream `ephemNavConverter`.
#[derive(Clone, Debug)]
pub struct EphemNavConverter {
    /// Module configuration.
    pub config: EphemNavConverterConfig,
    /// Upstream `ephInMsg`: ephemeris to convert.
    pub eph_in_msg: Input<EphemerisMsg>,
    /// Upstream `stateOutMsg`: the ephemeris state as a navigation estimate.
    pub state_out_msg: Output<NavigationTranslationMsg>,
}

impl EphemNavConverter {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: EphemNavConverterConfig) -> Self {
        Self {
            config,
            eph_in_msg: Input::default(),
            state_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_ephemNavConverter`: only checks the input.
    fn check_connections(&self) {
        if !self.eph_in_msg.is_connected() {
            log::error!("ephemNavConverter.eph_in_msg is not connected");
        }
    }
}

impl Module for EphemNavConverter {
    fn init(&mut self) {
        self.state_out_msg
            .write(NavigationTranslationMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let ephemeris = self.eph_in_msg.read();
        self.state_out_msg.write(NavigationTranslationMsg {
            time_tag_s: ephemeris.time_tag_s,
            position_bn_n_m: ephemeris.position_bdy_zero_n_m,
            velocity_bn_n_mps: ephemeris.velocity_bdy_zero_n_mps,
            ..Default::default()
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{EphemNavConverter, EphemNavConverterConfig};
    use crate::fsw_algorithms::trans_determination::ephemeris_utilities::test_support::{
        Planet, julian_date, planet_rv,
    };
    use crate::messages::{EphemerisMsg, Output};
    use crate::{Module, SimulationContext};

    const TASK_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn assert_vector_close(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64) {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!((a - e).abs() <= accuracy, "{actual} != {expected}");
        }
    }

    /// Upstream `test_ephemNavConverter`: the Earth's Meeus ephemeris on
    /// 2018-10-16 (upstream `astroFunctions.Earth_RV`), converted at t = 0
    /// and 0.5 s.
    #[test]
    fn matches_upstream_ephem_nav_converter_unit_test() {
        let (position, velocity) = planet_rv(Planet::Earth, julian_date(2018, 10, 16));
        let input = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: position,
            velocity_bdy_zero_n_mps: velocity,
            time_tag_s: 1.0,
            ..Default::default()
        });
        let mut module = EphemNavConverter::new(EphemNavConverterConfig::default());
        input.connect_to(&mut module.eph_in_msg);
        module.init();

        let pos_acc = 1e1;
        let vel_acc = 1e-4;
        for tick in 0..=1 {
            module.update(&context(tick * TASK_NANOS));
            let output = module.state_out_msg.read();
            assert_vector_close(output.position_bn_n_m, position, pos_acc);
            assert_vector_close(output.velocity_bn_n_mps, velocity, vel_acc);
            assert!((output.time_tag_s - 1.0).abs() <= vel_acc);
        }
    }

    // Not upstream: the copy is exact and leaves the accumulated delta-v zero.
    #[test]
    fn copies_the_translational_state_exactly() {
        let input = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: Vector3::new(1.5e11, -2.0e10, 3.0e9),
            velocity_bdy_zero_n_mps: Vector3::new(-4.0e3, 2.9e4, 1.0),
            sigma_bn: Vector3::new(0.1, 0.2, 0.3),
            omega_bn_b_radps: Vector3::new(1e-5, 0.0, 7.3e-5),
            time_tag_s: 42.25,
        });
        let mut module = EphemNavConverter::new(EphemNavConverterConfig::default());
        input.connect_to(&mut module.eph_in_msg);
        module.init();
        module.update(&context(0));
        let output = module.state_out_msg.read();
        assert_eq!(output.time_tag_s, 42.25);
        assert_eq!(output.position_bn_n_m, Vector3::new(1.5e11, -2.0e10, 3.0e9));
        assert_eq!(output.velocity_bn_n_mps, Vector3::new(-4.0e3, 2.9e4, 1.0));
        assert_eq!(output.accumulated_dv_b_mps, Vector3::zeros());
    }
}
