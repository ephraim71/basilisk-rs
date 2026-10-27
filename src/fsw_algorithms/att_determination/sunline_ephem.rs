//! Ephemeris-based sun heading: port of upstream Basilisk `sunlineEphem`.
//!
//! The module takes the sun position from an ephemeris message and the
//! spacecraft position and attitude from navigation messages, and publishes
//! the unit vector from the spacecraft to the sun in body components as the
//! `vehicle_sun_point_body` of a navigation attitude message. The other fields
//! of that message stay zero, as upstream.

use crate::messages::{
    EphemerisMsg, Input, NavigationAttitudeMsg, NavigationTranslationMsg, Output,
};
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Configuration of [`SunlineEphem`]; upstream has no parameters.
#[derive(Clone, Debug)]
pub struct SunlineEphemConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
}

impl Default for SunlineEphemConfig {
    fn default() -> Self {
        Self {
            name: "sunlineEphem".to_string(),
        }
    }
}

/// Sun heading from ephemerides: upstream `sunlineEphem`.
#[derive(Clone, Debug)]
pub struct SunlineEphem {
    /// Module configuration.
    pub config: SunlineEphemConfig,
    /// Upstream `sunPositionInMsg`: sun ephemeris.
    pub sun_position_in_msg: Input<EphemerisMsg>,
    /// Upstream `scPositionInMsg`: spacecraft inertial position.
    pub sc_position_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `scAttitudeInMsg`: spacecraft attitude.
    pub sc_attitude_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `navStateOutMsg`: sun heading in body components.
    pub nav_state_out_msg: Output<NavigationAttitudeMsg>,
}

impl SunlineEphem {
    /// Builds the module with unconnected inputs.
    pub fn new(config: SunlineEphemConfig) -> Self {
        Self {
            config,
            sun_position_in_msg: Input::default(),
            sc_position_in_msg: Input::default(),
            sc_attitude_in_msg: Input::default(),
            nav_state_out_msg: Output::default(),
        }
    }

    /// Upstream checks the inputs on every update rather than at reset.
    fn check_connections(&self) {
        if !self.sun_position_in_msg.is_connected() {
            log::error!("sunlineEphem.sun_position_in_msg is not connected");
        }
        if !self.sc_position_in_msg.is_connected() {
            log::error!("sunlineEphem.sc_position_in_msg is not connected");
        }
        if !self.sc_attitude_in_msg.is_connected() {
            log::error!("sunlineEphem.sc_attitude_in_msg is not connected");
        }
    }

    fn sun_heading(&self) -> NavigationAttitudeMsg {
        let sun = self.sun_position_in_msg.read();
        let position = self.sc_position_in_msg.read();
        let attitude = self.sc_attitude_in_msg.read();

        let r_hat_sb_n = normalize_or_zero(sun.position_bdy_zero_n_m - position.position_bn_n_m);
        // Normalized again in the body frame, as upstream.
        let r_hat_sb_b = normalize_or_zero(mrp_to_dcm(attitude.sigma_bn) * r_hat_sb_n);
        NavigationAttitudeMsg {
            vehicle_sun_point_body: r_hat_sb_b,
            ..Default::default()
        }
    }
}

impl Module for SunlineEphem {
    fn init(&mut self) {
        self.nav_state_out_msg
            .write(NavigationAttitudeMsg::default());
    }

    /// Upstream `Reset_sunlineEphem` does nothing.
    fn reset(&mut self, _context: &SimulationContext) {}

    fn update(&mut self, _context: &SimulationContext) {
        self.check_connections();
        self.nav_state_out_msg.write(self.sun_heading());
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{EphemerisMsg, NavigationAttitudeMsg, NavigationTranslationMsg, Output};
    use crate::{Module, SimulationContext};

    use super::{SunlineEphem, SunlineEphemConfig};

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

    /// Upstream `test_sunlineEphem.py::test_module`: the sun at the origin,
    /// the spacecraft one unit away along each axis, identity attitude. Each
    /// case re-initializes the simulation, runs it to 1 s and resets the
    /// module, as upstream does.
    #[test]
    fn matches_upstream_sunline_ephem_unit_test() {
        let test_vectors = [
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let true_vectors = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ];
        let accuracy = 1e-12;

        let attitude_output = Output::new(NavigationAttitudeMsg::default());
        let sun_output = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: Vector3::zeros(),
            ..Default::default()
        });
        let position_output = Output::new(NavigationTranslationMsg::default());
        let mut module = SunlineEphem::new(SunlineEphemConfig::default());
        sun_output.connect_to(&mut module.sun_position_in_msg);
        position_output.connect_to(&mut module.sc_position_in_msg);
        attitude_output.connect_to(&mut module.sc_attitude_in_msg);

        for (test_vector, true_vector) in test_vectors.iter().zip(&true_vectors) {
            position_output.write(NavigationTranslationMsg {
                position_bn_n_m: Vector3::from(*test_vector),
                ..Default::default()
            });

            module.init();
            for tick in 0..=2 {
                module.update(&context(tick * TASK_NANOS));
            }
            let estimate = module.nav_state_out_msg.read().vehicle_sun_point_body;
            assert_vector_close(estimate, Vector3::from(*true_vector), accuracy);

            module.reset(&context(1));
        }
    }

    // Not upstream: a rotated attitude and an offset sun, with hand-derived
    // truth. sigma_BN = tan(pi/8) * z is a +90 deg rotation about z, so
    // [BN] maps N-frame x to -y and y to x.
    #[test]
    fn rotates_the_inertial_heading_into_the_body_frame() {
        let attitude_output = Output::new(NavigationAttitudeMsg {
            sigma_bn: Vector3::new(0.0, 0.0, (std::f64::consts::PI / 8.0).tan()),
            ..Default::default()
        });
        let sun_output = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: Vector3::new(3.0e11, 4.0e11, 0.0),
            ..Default::default()
        });
        let position_output = Output::new(NavigationTranslationMsg {
            position_bn_n_m: Vector3::new(7.0e6, 0.0, 0.0),
            ..Default::default()
        });
        let mut module = SunlineEphem::new(SunlineEphemConfig::default());
        sun_output.connect_to(&mut module.sun_position_in_msg);
        position_output.connect_to(&mut module.sc_position_in_msg);
        attitude_output.connect_to(&mut module.sc_attitude_in_msg);
        module.init();
        module.update(&context(0));

        let r_sb_n: Vector3<f64> = Vector3::new(3.0e11 - 7.0e6, 4.0e11, 0.0);
        let r_hat_sb_n = r_sb_n / r_sb_n.norm();
        let expected = Vector3::new(r_hat_sb_n.y, -r_hat_sb_n.x, 0.0);
        let output = module.nav_state_out_msg.read();
        assert_vector_close(output.vehicle_sun_point_body, expected, 1e-12);
        assert_eq!(output.time_tag_s, 0.0);
        assert_eq!(output.sigma_bn, Vector3::zeros());
        assert_eq!(output.omega_bn_b_radps, Vector3::zeros());
    }

    // Not upstream: coincident sun and spacecraft give upstream's zero vector
    // (v3Normalize of a zero vector), not NaN.
    #[test]
    fn coincident_sun_and_spacecraft_give_a_zero_heading() {
        let mut module = SunlineEphem::new(SunlineEphemConfig::default());
        module.init();
        module.update(&context(0));
        assert_eq!(
            module.nav_state_out_msg.read().vehicle_sun_point_body,
            Vector3::zeros()
        );
    }
}
