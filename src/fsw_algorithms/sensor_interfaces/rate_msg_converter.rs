//! IMU rate to navigation attitude message: port of upstream Basilisk
//! `rateMsgConverter`.
//!
//! Each update copies the body angular rate of an [`ImuSensorBodyMsg`] into
//! `omega_bn_b_radps` of an otherwise zeroed [`NavigationAttitudeMsg`], the
//! common attitude input of the FSW modules.

use crate::messages::{ImuSensorBodyMsg, Input, NavigationAttitudeMsg, Output};
use crate::{Module, SimulationContext};

/// Configuration of [`RateMsgConverter`]; upstream has no parameters.
#[derive(Clone, Debug)]
pub struct RateMsgConverterConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
}

impl Default for RateMsgConverterConfig {
    fn default() -> Self {
        Self {
            name: "rateMsgConverter".to_string(),
        }
    }
}

/// IMU body rate repackaged as a navigation attitude: upstream
/// `rateMsgConverter`.
#[derive(Clone, Debug)]
pub struct RateMsgConverter {
    /// Module configuration.
    pub config: RateMsgConverterConfig,
    /// Upstream `imuRateInMsg`: IMU sample in body components.
    pub imu_rate_in_msg: Input<ImuSensorBodyMsg>,
    /// Upstream `navRateOutMsg`: navigation attitude carrying only the rate.
    pub nav_rate_out_msg: Output<NavigationAttitudeMsg>,
}

impl RateMsgConverter {
    /// Creates the module with unconnected ports.
    pub fn new(config: RateMsgConverterConfig) -> Self {
        Self {
            config,
            imu_rate_in_msg: Input::default(),
            nav_rate_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_rateMsgConverter`: the module retains no state.
    fn check_connections(&self) {
        if !self.imu_rate_in_msg.is_connected() {
            log::error!("rateMsgConverter.imu_rate_in_msg is not connected");
        }
    }
}

impl Module for RateMsgConverter {
    fn init(&mut self) {
        self.nav_rate_out_msg
            .write(NavigationAttitudeMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let imu = self.imu_rate_in_msg.read();
        self.nav_rate_out_msg.write(NavigationAttitudeMsg {
            omega_bn_b_radps: imu.angular_rate_body_radps,
            ..Default::default()
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{RateMsgConverter, RateMsgConverterConfig};
    use crate::messages::{ImuSensorBodyMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    fn assert_vector_close(actual: Vector3<f64>, expected: [f64; 3], accuracy: f64, what: &str) {
        for (index, (value, truth)) in actual.iter().zip(expected).enumerate() {
            assert!(
                (value - truth).abs() <= accuracy,
                "{what}[{index}] = {value}, truth {truth}"
            );
        }
    }

    /// `test_rateMsgConverter.py::test_module`: a 0.5 s task run to 1 s
    /// records three samples; the rate is copied and the attitude and sun
    /// heading stay zero, with upstream's `accuracy = 1e-12`.
    #[test]
    fn matches_upstream_rate_msg_converter_unit_test() {
        let accuracy = 1e-12;
        let imu_output = Output::new(ImuSensorBodyMsg {
            angular_rate_body_radps: Vector3::new(-0.1, 0.2, -0.3),
            ..Default::default()
        });
        let mut module = RateMsgConverter::new(RateMsgConverterConfig::default());
        let mut recorder = MessageRecorder::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&imu_output, &mut module.imu_rate_in_msg);
        simulation.connect(&module.nav_rate_out_msg, &mut recorder.input_msg);
        simulation.add_module("rateMsgConverter", &mut module, 500_000_000, 0);
        simulation.add_module("recorder", &mut recorder, 500_000_000, 0);
        simulation.run_for(1_000_000_000);
        drop(simulation);

        assert_eq!(recorder.samples.len(), 3);
        for sample in &recorder.samples {
            assert_vector_close(
                sample.omega_bn_b_radps,
                [-0.1, 0.2, -0.3],
                accuracy,
                "omega_BN_B",
            );
            assert_vector_close(sample.sigma_bn, [0.0; 3], accuracy, "sigma_BN");
            assert_vector_close(
                sample.vehicle_sun_point_body,
                [0.0; 3],
                accuracy,
                "vehSunPntBdy",
            );
            assert_eq!(sample.time_tag_s, 0.0);
        }
    }

    // Not upstream: the other IMU fields are dropped and an unconnected input
    // gives a zero rate.
    #[test]
    fn copies_only_the_body_rate() {
        let context = SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        };
        let imu_output = Output::new(ImuSensorBodyMsg {
            delta_velocity_body_mps: Vector3::new(1.0, 2.0, 3.0),
            accel_body_mps2: Vector3::new(4.0, 5.0, 6.0),
            delta_angle_body_rad: Vector3::new(7.0, 8.0, 9.0),
            angular_rate_body_radps: Vector3::new(0.01, 0.02, 0.03),
        });
        let mut module = RateMsgConverter::new(RateMsgConverterConfig::default());
        imu_output.connect_to(&mut module.imu_rate_in_msg);
        module.init();
        module.update(&context);
        let output = module.nav_rate_out_msg.read();
        assert_eq!(output.omega_bn_b_radps, Vector3::new(0.01, 0.02, 0.03));
        assert_eq!(output.sigma_bn, Vector3::zeros());
        assert_eq!(output.vehicle_sun_point_body, Vector3::zeros());

        let mut unconnected = RateMsgConverter::new(RateMsgConverterConfig::default());
        unconnected.init();
        unconnected.update(&context);
        assert_eq!(
            unconnected.nav_rate_out_msg.read().omega_bn_b_radps,
            Vector3::zeros()
        );
    }
}
