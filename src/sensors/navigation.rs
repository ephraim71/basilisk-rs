use crate::messages::{
    Input, NavigationAttitudeMsg, Output, SpacecraftStateMsg, TranslationReferenceMsg,
};
use crate::{Module, SimulationContext};

/// Perfect navigation adapter that republishes spacecraft truth in the
/// navigation-message types consumed by flight-software modules.
///
/// This is a noiseless navigation solution: position, velocity,
/// attitude, and body rate are copied exactly from the connected spacecraft
/// state. The Sun-point field remains zero because [`SpacecraftStateMsg`] does
/// not carry a Sun direction.
#[derive(Clone, Debug)]
pub struct SimpleNavigation {
    pub name: String,
    pub spacecraft_state_in_msg: Input<SpacecraftStateMsg>,
    pub attitude_out_msg: Output<NavigationAttitudeMsg>,
    pub translation_out_msg: Output<TranslationReferenceMsg>,
}

impl SimpleNavigation {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            spacecraft_state_in_msg: Input::default(),
            attitude_out_msg: Output::default(),
            translation_out_msg: Output::default(),
        }
    }
}

impl Module for SimpleNavigation {
    fn init(&mut self) {
        if !self.spacecraft_state_in_msg.is_connected() {
            log::error!(
                "simple navigation '{}' spacecraft_state_in_msg is not connected",
                self.name
            );
        }
        self.attitude_out_msg
            .write(NavigationAttitudeMsg::default());
        self.translation_out_msg
            .write(TranslationReferenceMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let state = self.spacecraft_state_in_msg.read();

        self.attitude_out_msg.write(NavigationAttitudeMsg {
            time_tag_s: context.current_sim_nanos as f64 * 1.0e-9,
            sigma_bn: state.sigma_bn,
            omega_bn_b_radps: state.omega_radps,
            vehicle_sun_point_body: nalgebra::Vector3::zeros(),
        });
        self.translation_out_msg.write(TranslationReferenceMsg {
            position_m: state.position_m,
            velocity_mps: state.velocity_mps,
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{Output, SpacecraftStateMsg};
    use crate::{Module, SimulationContext};

    use super::SimpleNavigation;

    #[test]
    fn copies_spacecraft_truth_exactly() {
        let state = SpacecraftStateMsg {
            position_m: Vector3::new(7_000_000.0, -1_000.0, 2_000.0),
            velocity_mps: Vector3::new(10.0, 7_500.0, -20.0),
            sigma_bn: Vector3::new(0.1, -0.2, 0.3),
            omega_radps: Vector3::new(0.01, 0.02, -0.03),
        };
        let state_out = Output::new(state.clone());
        let mut navigation = SimpleNavigation::new("simple_navigation");
        navigation.spacecraft_state_in_msg.connect(state_out.slot());

        navigation.init();
        navigation.update(&SimulationContext {
            current_sim_nanos: 12_345_678_900,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });

        let attitude = navigation.attitude_out_msg.read();
        assert!((attitude.time_tag_s - 12.345_678_9).abs() < 1.0e-14);
        assert_eq!(attitude.sigma_bn, state.sigma_bn);
        assert_eq!(attitude.omega_bn_b_radps, state.omega_radps);
        assert_eq!(attitude.vehicle_sun_point_body, Vector3::zeros());

        let translation = navigation.translation_out_msg.read();
        assert_eq!(translation.position_m, state.position_m);
        assert_eq!(translation.velocity_mps, state.velocity_mps);
    }
}
