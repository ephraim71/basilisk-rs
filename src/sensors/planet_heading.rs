//! Body-frame heading to a planet: port of upstream Basilisk `planetHeading`.
//!
//! Reads the planet state (upstream `planetPositionInMsg`, a
//! `SpicePlanetStateMsg`, here [`PlanetStateMsg`]) and the spacecraft state
//! (upstream `spacecraftStateInMsg`, [`SpacecraftStateMsg`]) and publishes the
//! unit vector from the spacecraft to the planet in body components, upstream
//! `planetHeadingOutMsg` ([`BodyHeadingMsg`]):
//!
//! `r̂_PB_B = [BN] (r_PN_N − r_BN_N) / |r_PN_N − r_BN_N|`.
//!
//! Only the heading is modelled, not its rate. A spacecraft at the planet's
//! center has no heading; the output is then zero, as Eigen's `normalized()`
//! returns a zero vector unchanged.

use crate::messages::{BodyHeadingMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg};
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Heading to a planet in the body frame: upstream `PlanetHeading`.
#[derive(Clone, Debug)]
pub struct PlanetHeading {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `planetPositionInMsg`: the planet's inertial state.
    pub planet_position_in_msg: Input<PlanetStateMsg>,
    /// Upstream `spacecraftStateInMsg`: the spacecraft's inertial state.
    pub spacecraft_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `planetHeadingOutMsg`: unit heading to the planet, body frame.
    pub planet_heading_out_msg: Output<BodyHeadingMsg>,
}

impl PlanetHeading {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            planet_position_in_msg: Input::default(),
            spacecraft_state_in_msg: Input::default(),
            planet_heading_out_msg: Output::default(),
        }
    }

    fn check_connections(&self) {
        if !self.planet_position_in_msg.is_connected() {
            log::error!(
                "planetHeading '{}' planet_position_in_msg was not linked",
                self.name
            );
        }
        if !self.spacecraft_state_in_msg.is_connected() {
            log::error!(
                "planetHeading '{}' spacecraft_state_in_msg was not linked",
                self.name
            );
        }
    }
}

impl Module for PlanetHeading {
    fn init(&mut self) {
        self.check_connections();
        self.planet_heading_out_msg.write(BodyHeadingMsg::default());
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let planet = self.planet_position_in_msg.read();
        let spacecraft = self.spacecraft_state_in_msg.read();

        let r_pb_n = planet.position_inertial_m - spacecraft.position_m;
        let r_pb_b = mrp_to_dcm(spacecraft.sigma_bn) * r_pb_n;
        let norm = r_pb_b.norm();
        let r_hat_xb_b = if norm > 0.0 { r_pb_b / norm } else { r_pb_b };

        self.planet_heading_out_msg
            .write(BodyHeadingMsg { r_hat_xb_b });
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `planetHeading/_UnitTest/test_planetHeading.py` with its
    //! relative tolerance, and adds a rotated-attitude case and the
    //! degenerate co-located case.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{Output, PlanetStateMsg, SpacecraftStateMsg};
    use crate::simulation::Simulation;
    use crate::{Module, SimulationContext};

    use super::PlanetHeading;

    /// Upstream `orbitalMotion.AU` \[km\].
    const AU_KM: f64 = 149_597_870.693;

    fn epoch() -> Epoch {
        Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
    }

    fn heading(planet: PlanetStateMsg, spacecraft: SpacecraftStateMsg) -> Vector3<f64> {
        let planet_out = Output::new(planet);
        let spacecraft_out = Output::new(spacecraft);
        let mut module = PlanetHeading::new("planetHeading");
        planet_out.connect_to(&mut module.planet_position_in_msg);
        spacecraft_out.connect_to(&mut module.spacecraft_state_in_msg);
        module.init();
        module.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: epoch(),
        });
        module.planet_heading_out_msg.read().r_hat_xb_b
    }

    /// `pytest.approx(expected, rel)`: relative tolerance, with upstream's
    /// default absolute floor of 1e-12 for zero entries.
    fn assert_approx(actual: Vector3<f64>, expected: Vector3<f64>, rel: f64) {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!(
                (a - e).abs() <= (rel * e.abs()).max(1.0e-12),
                "{actual} vs {expected}"
            );
        }
    }

    /// Upstream `test_planetHeading`: Earth at the origin, a spacecraft with
    /// inertial attitude at 1 AU along +z; one step gives [0, 0, -1].
    #[test]
    fn heading_from_one_au_along_z_points_down_z() {
        let planet = Output::new(PlanetStateMsg::default());
        let spacecraft = Output::new(SpacecraftStateMsg {
            position_m: Vector3::new(0.0, 0.0, AU_KM * 1000.0),
            ..Default::default()
        });
        let mut module = PlanetHeading::new("planetHeading");
        {
            let mut simulation = Simulation::new(epoch(), false);
            simulation.connect(&planet, &mut module.planet_position_in_msg);
            simulation.connect(&spacecraft, &mut module.spacecraft_state_in_msg);
            simulation.add_module("planetHeading", &mut module, 1_000_000_000, 0);
            simulation.run_for(0);
        }
        assert_approx(
            module.planet_heading_out_msg.read().r_hat_xb_b,
            Vector3::new(0.0, 0.0, -1.0),
            1.0e-8,
        );
    }

    /// With a rotated body the heading is the inertial direction mapped by
    /// `[BN]`, here through the quaternion form of the same attitude.
    #[test]
    fn heading_is_rotated_into_the_body_frame() {
        let planet_position = Vector3::new(1.0e8, -2.0e7, 3.0e7);
        let state = SpacecraftStateMsg {
            position_m: Vector3::new(7.0e6, 1.0e6, -2.0e6),
            sigma_bn: Vector3::new(0.1, -0.3, 0.25),
            ..Default::default()
        };
        let actual = heading(
            PlanetStateMsg {
                position_inertial_m: planet_position,
                ..Default::default()
            },
            state.clone(),
        );
        let expected = state
            .inertial_to_body()
            .transform_vector(&(planet_position - state.position_m).normalize());
        assert!(
            (actual - expected).norm() < 1.0e-14,
            "{actual} vs {expected}"
        );
        assert!((actual.norm() - 1.0).abs() < 1.0e-15);
    }

    /// A spacecraft at the planet's center has no heading: zero, not NaN.
    #[test]
    fn co_located_spacecraft_has_a_zero_heading() {
        let position = Vector3::new(1.0, 2.0, 3.0);
        let actual = heading(
            PlanetStateMsg {
                position_inertial_m: position,
                ..Default::default()
            },
            SpacecraftStateMsg {
                position_m: position,
                ..Default::default()
            },
        );
        assert_eq!(actual, Vector3::zeros());
    }
}
