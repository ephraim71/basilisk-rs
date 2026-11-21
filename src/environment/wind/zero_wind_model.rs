//! Zero wind: port of upstream Basilisk
//! `simulation/environment/ZeroWindModel`.
//!
//! The air moves only with the co-rotating atmosphere that [`WindBase`]
//! computes; the wind perturbation is always zero. See [`super`] for the
//! inputs, the planet angular velocity options and the differences from
//! upstream.

use nalgebra::Vector3;

use super::{WindBase, WindBaseConfig, WindModel};
use crate::SimulationContext;
use crate::messages::WindMsg;

/// The zero-wind model (upstream `ZeroWindModel::evaluateWindModel`).
#[derive(Clone, Copy, Debug, Default)]
pub struct ZeroWind;

impl WindModel for ZeroWind {
    fn evaluate_wind_model(
        &mut self,
        _r_bp_n_m: Vector3<f64>,
        v_corotating_air_n_mps: Vector3<f64>,
        _context: &SimulationContext,
    ) -> WindMsg {
        WindMsg {
            air_velocity_inertial_mps: v_corotating_air_n_mps,
            wind_velocity_inertial_mps: Vector3::zeros(),
        }
    }
}

/// Upstream `ZeroWindModel`: a [`WindBase`] running [`ZeroWind`].
pub type ZeroWindModel = WindBase<ZeroWind>;

impl ZeroWindModel {
    /// A zero-wind module named `name`.
    pub fn named(name: impl Into<String>) -> Self {
        WindBase::new(WindBaseConfig { name: name.into() }, ZeroWind)
    }
}

/// Ports of upstream `test_zeroWindModel.py`.
///
/// `test_windStepLag.py` is not ported: it exercises the extrapolation of the
/// spacecraft state by message age, which needs message write times the
/// ports do not carry (see [`super`]).
#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::ZeroWindModel;
    use crate::environment::wind::WindConfigError;
    use crate::messages::{Output, PlanetOrientation, PlanetStateMsg, SpacecraftStateMsg};
    use crate::utilities::astro_constants::OMEGA_EARTH;
    use crate::{Module, SimulationContext};

    fn context(seconds: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: seconds * 1_000_000_000,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn planet_at(position_m: Vector3<f64>) -> PlanetStateMsg {
        PlanetStateMsg {
            position_inertial_m: position_m,
            ..Default::default()
        }
    }

    /// A planet at the origin with an identity orientation rotating about
    /// `+z` at `omega_z` \[rad/s\], as upstream's `J20002Pfix_dot`.
    fn spinning_planet(omega_z: f64) -> PlanetStateMsg {
        PlanetStateMsg {
            position_inertial_m: Vector3::zeros(),
            velocity_inertial_mps: Vector3::zeros(),
            orientation: Some(PlanetOrientation {
                inertial_to_fixed: Matrix3::identity(),
                inertial_to_fixed_dot: Matrix3::new(
                    0.0, omega_z, 0.0, //
                    -omega_z, 0.0, 0.0, //
                    0.0, 0.0, 0.0,
                ),
            }),
        }
    }

    struct Harness {
        wind: ZeroWindModel,
        _spacecraft: Output<SpacecraftStateMsg>,
        _planet: Output<PlanetStateMsg>,
    }

    fn connected(
        mut wind: ZeroWindModel,
        r_bn_n_m: Vector3<f64>,
        planet: PlanetStateMsg,
    ) -> Harness {
        let spacecraft = Output::new(SpacecraftStateMsg {
            position_m: r_bn_n_m,
            ..Default::default()
        });
        let planet = Output::new(planet);
        wind.input_state_msg.connect(spacecraft.slot());
        wind.input_planet_msg.connect(planet.slot());
        Harness {
            wind,
            _spacecraft: spacecraft,
            _planet: planet,
        }
    }

    /// Upstream runs one-second steps to t = 1 s and checks the last record.
    fn run(harness: &mut Harness) {
        harness.wind.init();
        harness.wind.update(&context(0));
        harness.wind.update(&context(1));
    }

    fn assert_close(actual: Vector3<f64>, expected: Vector3<f64>, atol: f64, what: &str) {
        assert!(
            (actual - expected).amax() <= atol,
            "{what}: got {actual:?}, expected {expected:?}"
        );
    }

    #[test]
    fn zero_wind_outputs_the_corotating_atmosphere() {
        let omega_planet = Vector3::new(0.0, 0.0, 1.0e-4);
        let planet_position = Vector3::new(1.0e6, 2.0e6, -5.0e5);
        let r_bn_n = Vector3::new(6_578_000.0, 0.0, 0.0) + planet_position;

        let mut wind = ZeroWindModel::named("zeroWind");
        wind.set_planet_omega_n(omega_planet);
        wind.set_use_spice_omega(false);
        let mut harness = connected(wind, r_bn_n, planet_at(planet_position));
        run(&mut harness);

        let out = harness.wind.output_wind_msg.read();
        let expected_air = omega_planet.cross(&(r_bn_n - planet_position));
        assert_close(
            out.air_velocity_inertial_mps,
            expected_air,
            1.0e-8,
            "v_air_N",
        );
        assert_close(
            out.wind_velocity_inertial_mps,
            Vector3::zeros(),
            1.0e-8,
            "v_wind_N",
        );
        assert_close(
            out.air_velocity_inertial_mps,
            out.wind_velocity_inertial_mps + expected_air,
            1.0e-8,
            "v_air_N = v_corotating + v_wind_N",
        );
    }

    #[test]
    fn spice_rate_comes_from_the_planet_orientation_derivative() {
        let spice_omega = Vector3::new(0.0, 0.0, 1.5e-4);
        let r_bn_n = Vector3::new(6_578_000.0, 0.0, 0.0);

        let mut wind = ZeroWindModel::named("zeroWind");
        wind.set_use_spice_omega(true);
        let mut harness = connected(wind, r_bn_n, spinning_planet(spice_omega.z));
        run(&mut harness);

        assert_close(
            harness
                .wind
                .output_wind_msg
                .read()
                .air_velocity_inertial_mps,
            spice_omega.cross(&r_bn_n),
            1.0e-8,
            "SPICE-derived omega",
        );
    }

    #[test]
    fn zero_orientation_rate_gives_zero_spice_rate() {
        let mut wind = ZeroWindModel::named("zeroWind");
        wind.set_use_spice_omega(true);
        let mut harness = connected(
            wind,
            Vector3::new(6_578_000.0, 0.0, 0.0),
            spinning_planet(0.0),
        );
        run(&mut harness);

        assert_close(
            harness
                .wind
                .output_wind_msg
                .read()
                .air_velocity_inertial_mps,
            Vector3::zeros(),
            1.0e-8,
            "zero C_dot",
        );
    }

    #[test]
    fn manual_mode_defaults_to_earth_rotation() {
        let r_bn_n = Vector3::new(6_578_000.0, 0.0, 0.0);

        let mut wind = ZeroWindModel::named("zeroWind");
        wind.set_use_spice_omega(false);
        let mut harness = connected(wind, r_bn_n, spinning_planet(0.0));
        run(&mut harness);

        assert_close(
            harness
                .wind
                .output_wind_msg
                .read()
                .air_velocity_inertial_mps,
            Vector3::new(0.0, 0.0, OMEGA_EARTH).cross(&r_bn_n),
            1.0e-8,
            "default Earth rotation",
        );
    }

    /// Upstream serves three spacecraft from one module; here each spacecraft
    /// gets its own instance.
    #[test]
    fn one_instance_per_spacecraft() {
        let omega_planet = Vector3::new(0.0, 0.0, 1.0e-4);
        let planet_position = Vector3::new(1.0e6, 0.0, 0.0);
        let spacecraft_positions = [
            Vector3::new(6_578_000.0, 0.0, 0.0) + planet_position,
            Vector3::new(6_578_000.0, 100_000.0, 50_000.0) + planet_position,
            Vector3::new(6_578_000.0, -50_000.0, -25_000.0) + planet_position,
        ];

        for (index, r_bn_n) in spacecraft_positions.into_iter().enumerate() {
            let mut wind = ZeroWindModel::named("zeroWind");
            wind.set_planet_omega_n(omega_planet);
            wind.set_use_spice_omega(false);
            let mut harness = connected(wind, r_bn_n, planet_at(planet_position));
            run(&mut harness);

            let out = harness.wind.output_wind_msg.read();
            assert_close(
                out.air_velocity_inertial_mps,
                omega_planet.cross(&(r_bn_n - planet_position)),
                1.0e-8,
                &format!("spacecraft {index} v_air_N"),
            );
            assert_close(
                out.wind_velocity_inertial_mps,
                Vector3::zeros(),
                1.0e-8,
                &format!("spacecraft {index} v_wind_N"),
            );
        }
    }

    #[test]
    fn an_unconnected_planet_is_a_configuration_error() {
        let mut wind = ZeroWindModel::named("zeroWind");
        let spacecraft = Output::new(SpacecraftStateMsg {
            position_m: Vector3::new(6_578_000.0, 0.0, 0.0),
            ..Default::default()
        });
        wind.input_state_msg.connect(spacecraft.slot());

        assert_eq!(
            wind.configuration_errors(),
            vec![WindConfigError::InputNotConnected("input_planet_msg")]
        );
        assert_eq!(
            ZeroWindModel::named("bare").configuration_errors(),
            vec![
                WindConfigError::InputNotConnected("input_state_msg"),
                WindConfigError::InputNotConnected("input_planet_msg"),
            ]
        );
    }

    #[test]
    fn planet_at_the_origin() {
        let omega_planet = Vector3::new(0.0, 0.0, 1.0e-4);
        let r_bn_n = Vector3::new(6_578_000.0, 0.0, 0.0);

        let mut wind = ZeroWindModel::named("zeroWind");
        wind.set_planet_omega_n(omega_planet);
        wind.set_use_spice_omega(false);
        let mut harness = connected(wind, r_bn_n, planet_at(Vector3::zeros()));
        run(&mut harness);

        assert_close(
            harness
                .wind
                .output_wind_msg
                .read()
                .air_velocity_inertial_mps,
            omega_planet.cross(&r_bn_n),
            1.0e-8,
            "planet at origin",
        );
    }

    #[test]
    fn zero_angular_velocity_gives_still_air() {
        let planet_position = Vector3::new(1.0e6, 0.0, 0.0);

        let mut wind = ZeroWindModel::named("zeroWind");
        wind.set_planet_omega_n(Vector3::zeros());
        wind.set_use_spice_omega(false);
        let mut harness = connected(
            wind,
            Vector3::new(6_578_000.0, 0.0, 0.0) + planet_position,
            planet_at(planet_position),
        );
        run(&mut harness);

        let out = harness.wind.output_wind_msg.read();
        assert_close(
            out.air_velocity_inertial_mps,
            Vector3::zeros(),
            1.0e-8,
            "v_air_N",
        );
        assert_close(
            out.wind_velocity_inertial_mps,
            Vector3::zeros(),
            1.0e-8,
            "v_wind_N",
        );
    }

    #[test]
    fn planet_omega_round_trips() {
        let mut wind = ZeroWindModel::named("zeroWind");
        assert_close(
            wind.planet_omega_n(),
            Vector3::new(0.0, 0.0, OMEGA_EARTH),
            1.0e-8,
            "default omega",
        );
        assert!(wind.use_spice_omega());

        let custom = Vector3::new(1.0e-5, 2.0e-5, 3.0e-5);
        wind.set_planet_omega_n(custom);
        assert_close(wind.planet_omega_n(), custom, 1.0e-8, "round trip");
        assert!(!wind.use_spice_omega(), "setting omega selects it");
    }

    #[test]
    fn switching_between_manual_and_spice_rates() {
        let spice_omega = Vector3::new(0.0, 0.0, 1.0e-4);
        let r_bn_n = Vector3::new(6_578_000.0, 0.0, 0.0);

        let manual_omega = Vector3::new(0.0, 0.0, 2.0e-4);
        let mut manual = ZeroWindModel::named("zeroWind");
        manual.set_planet_omega_n(manual_omega);
        manual.set_use_spice_omega(false);
        let mut harness = connected(manual, r_bn_n, spinning_planet(spice_omega.z));
        run(&mut harness);
        assert_close(
            harness
                .wind
                .output_wind_msg
                .read()
                .air_velocity_inertial_mps,
            manual_omega.cross(&r_bn_n),
            1.0e-8,
            "manual mode",
        );

        // A manual rate set first is ignored once the SPICE rate is selected.
        let mut spice = ZeroWindModel::named("zeroWind2");
        spice.set_planet_omega_n(Vector3::new(0.0, 0.0, 3.0e-4));
        spice.set_use_spice_omega(true);
        let mut harness = connected(spice, r_bn_n, spinning_planet(spice_omega.z));
        run(&mut harness);
        assert_close(
            harness
                .wind
                .output_wind_msg
                .read()
                .air_velocity_inertial_mps,
            spice_omega.cross(&r_bn_n),
            1.0e-8,
            "SPICE mode",
        );
    }

    /// Upstream links inputs that are never written; with "connected ⇒
    /// written" the equivalent is an unconnected input.
    #[test]
    fn missing_inputs_give_zero_wind() {
        let mut wind = ZeroWindModel::named("zeroWind");
        wind.init();
        wind.update(&context(0));
        assert_close(
            wind.output_wind_msg.read().air_velocity_inertial_mps,
            Vector3::zeros(),
            1.0e-12,
            "no inputs",
        );

        // A planet alone is not enough either.
        let planet = Output::new(spinning_planet(1.0e-4));
        let mut wind = ZeroWindModel::named("zeroWind");
        wind.input_planet_msg.connect(planet.slot());
        wind.init();
        wind.update(&context(0));
        assert_close(
            wind.output_wind_msg.read().air_velocity_inertial_mps,
            Vector3::zeros(),
            1.0e-12,
            "no spacecraft",
        );
    }

    #[test]
    #[should_panic(expected = "planet angular velocity must be finite")]
    fn rejects_a_nan_planet_omega() {
        ZeroWindModel::named("testWindNaN").set_planet_omega_n(Vector3::new(0.0, f64::NAN, 0.0));
    }

    #[test]
    #[should_panic(expected = "planet angular velocity must be finite")]
    fn rejects_an_infinite_planet_omega() {
        ZeroWindModel::named("testWindInf").set_planet_omega_n(Vector3::new(
            0.0,
            f64::INFINITY,
            0.0,
        ));
    }
}
