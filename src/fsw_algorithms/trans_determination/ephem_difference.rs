//! Ephemerides relative to a base body: port of upstream Basilisk
//! `ephemDifference`.
//!
//! Up to [`MAX_NUM_CHANGE_BODIES`] body ephemerides are re-expressed relative
//! to a base body: each output is its input with the base body's position and
//! velocity subtracted. The time tag and attitude fields pass through from the
//! body's own message.
//!
//! Reset counts the leading connected body inputs; the first unconnected slot
//! ends the list, and only that many outputs are written.

use crate::messages::{EphemerisMsg, Input, Output};
use crate::{Module, SimulationContext};

/// Upstream `MAX_NUM_CHANGE_BODIES`: number of body slots.
pub const MAX_NUM_CHANGE_BODIES: usize = 10;

/// A body ephemeris input and its converted output: upstream
/// `EphemChangeConfig`.
#[derive(Clone, Debug, Default)]
pub struct EphemChangeConfig {
    /// Upstream `ephInMsg`: ephemeris of the body.
    pub eph_in_msg: Input<EphemerisMsg>,
    /// Upstream `ephOutMsg`: the body's ephemeris relative to the base body.
    pub eph_out_msg: Output<EphemerisMsg>,
}

/// User-settable parameters of [`EphemDifference`]; upstream has none.
#[derive(Clone, Debug)]
pub struct EphemDifferenceConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
}

impl Default for EphemDifferenceConfig {
    fn default() -> Self {
        Self {
            name: "ephemDifference".to_string(),
        }
    }
}

/// Ephemeris differencing: upstream `ephemDifference`.
#[derive(Clone, Debug)]
pub struct EphemDifference {
    /// Module configuration.
    pub config: EphemDifferenceConfig,
    /// Upstream `ephBaseInMsg`: ephemeris of the base body.
    pub eph_base_in_msg: Input<EphemerisMsg>,
    /// Upstream `changeBodies`: [`MAX_NUM_CHANGE_BODIES`] body slots. Connect
    /// the bodies to the leading slots.
    pub change_bodies: Vec<EphemChangeConfig>,
    /// Upstream `ephBdyCount`: number of leading connected body inputs found
    /// at the last reset.
    pub eph_bdy_count: usize,
}

impl EphemDifference {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: EphemDifferenceConfig) -> Self {
        Self {
            config,
            eph_base_in_msg: Input::default(),
            change_bodies: (0..MAX_NUM_CHANGE_BODIES)
                .map(|_| EphemChangeConfig::default())
                .collect(),
            eph_bdy_count: 0,
        }
    }

    /// Upstream `Reset_ephemDifference`: checks the base input and counts the
    /// connected bodies.
    fn count_bodies(&mut self) {
        if !self.eph_base_in_msg.is_connected() {
            log::error!("ephemDifference.eph_base_in_msg is not connected");
        }
        self.eph_bdy_count = self
            .change_bodies
            .iter()
            .take(MAX_NUM_CHANGE_BODIES)
            .take_while(|body| body.eph_in_msg.is_connected())
            .count();
        if self.eph_bdy_count == 0 {
            log::warn!(
                "Your outgoing ephemeris message count is zero. Be sure to specify desired output messages."
            );
        }
    }
}

impl Module for EphemDifference {
    fn init(&mut self) {
        for body in &self.change_bodies {
            body.eph_out_msg.write(EphemerisMsg::default());
        }
        self.count_bodies();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.count_bodies();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let base = self.eph_base_in_msg.read();
        for body in self.change_bodies.iter().take(self.eph_bdy_count) {
            let mut ephemeris = body.eph_in_msg.read();
            ephemeris.position_bdy_zero_n_m -= base.position_bdy_zero_n_m;
            ephemeris.velocity_bdy_zero_n_mps -= base.velocity_bdy_zero_n_mps;
            body.eph_out_msg.write(ephemeris);
        }
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{EphemDifference, EphemDifferenceConfig};
    use crate::fsw_algorithms::trans_determination::ephemeris_utilities::test_support::{
        Planet, julian_date, planet_rv,
    };
    use crate::messages::{EphemerisMsg, Output};
    use crate::{Module, SimulationContext};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn ephemeris(planet: Planet, time_tag_s: f64) -> EphemerisMsg {
        let (position, velocity) = planet_rv(planet, julian_date(2018, 10, 16));
        EphemerisMsg {
            position_bdy_zero_n_m: position,
            velocity_bdy_zero_n_mps: velocity,
            time_tag_s,
            ..Default::default()
        }
    }

    fn assert_vector_close(actual: Vector3<f64>, expected: [f64; 3], accuracy: f64) {
        for (a, e) in actual.iter().zip(expected) {
            assert!((a - e).abs() <= accuracy, "{actual} != {expected:?}");
        }
    }

    /// Upstream `test_ephemDifference(ephBdyCount)` for `ephBdyCount` 3 and 0:
    /// Mars, Jupiter and Saturn relative to the Earth on 2018-10-16, from the
    /// Meeus ephemerides of upstream `astroFunctions` (\[km\], \[km/s\]), over
    /// one update at t = 0.
    #[test]
    fn matches_upstream_ephem_difference_unit_test() {
        let true_r_vector = [
            [69313607.6209608, -75620898.04028425, -5443274.17030424],
            [-5.33462105e+08, -7.56888610e+08, 1.17556184e+07],
            [9.94135029e+07, -1.54721593e+09, 1.65081472e+07],
        ];
        let true_v_vector = [
            [15.04232523, -1.13359121, 0.47668898],
            [23.2531093, -33.17628299, -0.22550391],
            [21.02793499, -25.86425597, -0.38273815],
        ];
        let pos_acc = 1e1;
        let vel_acc = 1e-4;

        for eph_bdy_count in [3, 0] {
            let base = Output::new(ephemeris(Planet::Earth, 1234.0));
            let mut module = EphemDifference::new(EphemDifferenceConfig::default());
            base.connect_to(&mut module.eph_base_in_msg);
            let bodies: Vec<Output<EphemerisMsg>> = [Planet::Mars, Planet::Jupiter, Planet::Saturn]
                .into_iter()
                .take(eph_bdy_count)
                .map(|planet| Output::new(ephemeris(planet, 321.0)))
                .collect();
            for (body, slot) in bodies.iter().zip(&mut module.change_bodies) {
                body.connect_to(&mut slot.eph_in_msg);
            }

            module.init();
            module.update(&context(0));

            for (i, slot) in module.change_bodies.iter().take(eph_bdy_count).enumerate() {
                let output = slot.eph_out_msg.read();
                assert_vector_close(output.position_bdy_zero_n_m, true_r_vector[i], pos_acc);
                assert_vector_close(output.velocity_bdy_zero_n_mps, true_v_vector[i], vel_acc);
                assert_eq!(output.time_tag_s, 321.0, "body {i}");
            }
            assert_eq!(module.eph_bdy_count, eph_bdy_count);
        }
    }

    // Not upstream: counting stops at the first unconnected slot, unused
    // outputs stay zero, and the body's attitude fields pass through.
    #[test]
    fn counts_only_the_leading_connected_bodies() {
        let base = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: Vector3::new(1.0, 2.0, 3.0),
            velocity_bdy_zero_n_mps: Vector3::new(-1.0, 0.5, 0.0),
            sigma_bn: Vector3::new(0.9, 0.9, 0.9),
            time_tag_s: 7.0,
            ..Default::default()
        });
        let body = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: Vector3::new(10.0, 20.0, 30.0),
            velocity_bdy_zero_n_mps: Vector3::new(1.0, 1.0, 1.0),
            sigma_bn: Vector3::new(0.1, 0.2, 0.3),
            omega_bn_b_radps: Vector3::new(0.01, 0.0, -0.01),
            time_tag_s: 3.0,
        });
        let mut module = EphemDifference::new(EphemDifferenceConfig::default());
        base.connect_to(&mut module.eph_base_in_msg);
        for slot in [0, 1, 3] {
            body.connect_to(&mut module.change_bodies[slot].eph_in_msg);
        }
        module.init();
        module.update(&context(0));

        assert_eq!(module.eph_bdy_count, 2);
        let expected = EphemerisMsg {
            position_bdy_zero_n_m: Vector3::new(9.0, 18.0, 27.0),
            velocity_bdy_zero_n_mps: Vector3::new(2.0, 0.5, 1.0),
            sigma_bn: Vector3::new(0.1, 0.2, 0.3),
            omega_bn_b_radps: Vector3::new(0.01, 0.0, -0.01),
            time_tag_s: 3.0,
        };
        assert_eq!(module.change_bodies[0].eph_out_msg.read(), expected);
        assert_eq!(module.change_bodies[1].eph_out_msg.read(), expected);
        assert_eq!(
            module.change_bodies[3].eph_out_msg.read(),
            EphemerisMsg::default()
        );
    }
}
