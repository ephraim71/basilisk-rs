//! Flat solar panel: port of upstream Basilisk `simpleSolarPanel`.
//!
//! A [`PowerNode`] that supplies the sunlight a flat, body-fixed panel
//! converts:
//!
//! ```text
//! P = SOLAR_FLUX_EARTH · (1 AU / |r_SB|)² · illumination · A · max(ŝ_B · n̂_B, 0) · η
//! ```
//!
//! where `r_SB` is the Sun position relative to the spacecraft, `ŝ_B` its
//! direction in body components, `n̂_B` the panel normal, `A` the panel area
//! and `η` the conversion efficiency. The illumination factor is the latest
//! [`EclipseMsg`] (such as [`crate::environment::eclipse::Eclipse`] publishes)
//! when `sun_eclipse_in_msg` is connected, and one otherwise. The flux at
//! 1 AU is upstream's `SOLAR_FLUX_EARTH`
//! ([`crate::utilities::astro_constants::SOLAR_FLUX_EARTH`], 1361 W/m²); the
//! panel does not read a [`SolarFluxMsg`](crate::messages::SolarFluxMsg).
//!
//! # Differences from upstream
//!
//! * The Sun is a [`SunEphemerisMsg`], this crate's stand-in for upstream's
//!   `SpicePlanetStateMsg` of the Sun.
//! * Upstream's `setPanelParameters` is [`SimpleSolarPanelModel::new`].
//! * The Sun and spacecraft inputs are required: unconnected, upstream reads
//!   zeroed payloads and reports a `bskError` at reset; here they read as
//!   defaults too and are logged at initialization and reset (see
//!   [`SimpleSolarPanelModel::unconnected_inputs`]).
//!
//! # Panics
//!
//! At initialization or reset, if the panel area or efficiency is negative or
//! the normal is shorter than 0.1 (upstream `bskError`s).

use nalgebra::Vector3;

use super::power_node::{PowerModel, PowerNode};
use crate::messages::{EclipseMsg, Input, SpacecraftStateMsg, SunEphemerisMsg};
use crate::utilities::astro_constants::{AU2M, SOLAR_FLUX_EARTH};
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;

/// Panel geometry, efficiency and inputs of a [`SimpleSolarPanel`]: upstream
/// `SimpleSolarPanel`'s own members.
#[derive(Clone, Debug)]
pub struct SimpleSolarPanelModel {
    /// Upstream `nHat_B`: panel normal, body components. Normalized at reset,
    /// as upstream.
    pub n_hat_b: Vector3<f64>,
    /// Upstream `panelArea` \[m²\].
    pub panel_area_m2: f64,
    /// Upstream `panelEfficiency` [-]: fraction of the incident solar power
    /// converted to electrical power.
    pub panel_efficiency: f64,
    /// Upstream `sunInMsg`: Sun position (required).
    pub sun_in_msg: Input<SunEphemerisMsg>,
    /// Upstream `stateInMsg`: spacecraft position and attitude (required).
    pub state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `sunEclipseInMsg`: optional Sun illumination factor.
    pub sun_eclipse_in_msg: Input<EclipseMsg>,
    /// Upstream `illuminationFactor` [-]: one at reset, then the latest
    /// eclipse message.
    illumination_factor: f64,
    /// Upstream `sunData`: the Sun read this update.
    sun: SunEphemerisMsg,
    /// Upstream `stateCurrent`: the spacecraft state read this update.
    state: SpacecraftStateMsg,
}

impl SimpleSolarPanelModel {
    /// A panel with normal `n_hat_b` (body components), area
    /// `panel_area_m2` \[m²\] and efficiency `panel_efficiency` (upstream
    /// `setPanelParameters`).
    pub fn new(n_hat_b: Vector3<f64>, panel_area_m2: f64, panel_efficiency: f64) -> Self {
        Self {
            n_hat_b,
            panel_area_m2,
            panel_efficiency,
            sun_in_msg: Input::default(),
            state_in_msg: Input::default(),
            sun_eclipse_in_msg: Input::default(),
            illumination_factor: 1.0,
            sun: SunEphemerisMsg::default(),
            state: SpacecraftStateMsg::default(),
        }
    }

    /// The required inputs that are not connected (upstream `Reset`'s
    /// "was not linked" errors).
    pub fn unconnected_inputs(&self) -> Vec<&'static str> {
        [
            (self.sun_in_msg.is_connected(), "sun_in_msg"),
            (self.state_in_msg.is_connected(), "state_in_msg"),
        ]
        .into_iter()
        .filter(|(connected, _)| !connected)
        .map(|(_, port)| port)
        .collect()
    }

    /// Upstream `computeSunData` \[m²\]: the panel area projected along the
    /// Sun direction, and the inverse-square factor `(1 AU / |r_SB|)²`.
    fn compute_sun_data(&self) -> (f64, f64) {
        let r_sb_n = self.sun.sun_position_inertial_m - self.state.position_m;
        // Eigen's `normalized()`: a zero vector stays zero.
        let s_hat_n = r_sb_n.try_normalize(0.0).unwrap_or_else(Vector3::zeros);
        let s_hat_b = mrp_to_dcm(self.state.sigma_bn) * s_hat_n;

        let projected_area_m2 = (self.panel_area_m2 * s_hat_b.dot(&self.n_hat_b)).max(0.0);
        let sun_distance_factor = AU2M.powi(2) / r_sb_n.norm().powi(2);
        (projected_area_m2, sun_distance_factor)
    }
}

impl PowerModel for SimpleSolarPanelModel {
    /// Upstream `customReset`.
    fn reset(&mut self, node_name: &str) {
        self.illumination_factor = 1.0;
        assert!(
            self.panel_area_m2 >= 0.0,
            "simple solar panel '{node_name}': panel_area_m2 must be non-negative, got {}",
            self.panel_area_m2
        );
        assert!(
            self.panel_efficiency >= 0.0,
            "simple solar panel '{node_name}': panel_efficiency must be non-negative, got {}",
            self.panel_efficiency
        );
        assert!(
            self.n_hat_b.norm() > 0.1,
            "simple solar panel '{node_name}': n_hat_b must be a non-zero vector, got {:?}",
            self.n_hat_b
        );
        self.n_hat_b.normalize_mut();
        for port in self.unconnected_inputs() {
            log::error!("simple solar panel '{node_name}': {port} is not connected");
        }
    }

    /// Upstream `customReadMessages`: the Sun and spacecraft (zeroed when
    /// unconnected), and the illumination factor when the eclipse input is
    /// connected. Always succeeds.
    fn read_messages(&mut self) -> bool {
        self.sun = self.sun_in_msg.read();
        self.state = self.state_in_msg.read();
        if self.sun_eclipse_in_msg.is_connected() {
            self.illumination_factor = self.sun_eclipse_in_msg.read().illumination_factor;
        }
        true
    }

    /// Upstream `evaluatePowerModel`.
    fn evaluate_power_model(&mut self) -> f64 {
        let (projected_area_m2, sun_distance_factor) = self.compute_sun_data();
        let sun_power_factor = SOLAR_FLUX_EARTH * sun_distance_factor * self.illumination_factor;
        sun_power_factor * projected_area_m2 * self.panel_efficiency
    }
}

/// Upstream `SimpleSolarPanel`: a [`PowerNode`] running
/// [`SimpleSolarPanelModel`].
pub type SimpleSolarPanel = PowerNode<SimpleSolarPanelModel>;

#[cfg(test)]
mod tests {
    //! Ports upstream `simpleSolarPanel/_UnitTest/test_unitSimpleSolarPanel.py`
    //! with its 1e-7 accuracy over all twelve parameter combinations, and adds
    //! an end-to-end power network: a panel lit through the `Eclipse` module
    //! over an orbit charges a `SimpleBattery` while a `SimplePowerSink`
    //! drains it.

    use std::f64::consts::FRAC_PI_2;

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use crate::dynamics::gravity::GravBodyData;
    use crate::environment::eclipse::{Eclipse, EclipseConfig};
    use crate::environment::solar_flux::{SolarFlux, SolarFluxConfig};
    use crate::messages::{
        EclipseMsg, Output, PowerNodeUsageMsg, SpacecraftStateMsg, SunEphemerisMsg,
    };
    use crate::power::simple_battery::{SimpleBattery, SimpleBatteryConfig};
    use crate::power::simple_power_sink::{SimplePowerSink, SimplePowerSinkModel};
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};
    use crate::test_utils::MessageRecorder;
    use crate::utilities::astro_constants::{AU, MU_EARTH, REQ_EARTH, SOLAR_FLUX_EARTH};
    use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, euler321_to_dcm, mrp_to_dcm};
    use crate::{Module, SimulationContext};

    use super::{SimpleSolarPanel, SimpleSolarPanelModel};

    fn epoch() -> Epoch {
        Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
    }

    /// Upstream `rbk.C2MRP(rbk.euler3212C([0, angle, 0]))`.
    fn pitched_attitude(angle_rad: f64) -> Vector3<f64> {
        dcm_to_mrp(euler321_to_dcm(Vector3::new(0.0, angle_rad, 0.0)))
    }

    /// Upstream `run`: a unit-area, unit-efficiency panel facing body `+x`,
    /// the Sun at the origin and the spacecraft at `-orbit_distance_m` on the
    /// x axis, run for one second at 1 s. Returns the logged power.
    fn run(orbit_distance_m: f64, eclipse_value: f64, sc_attitude: Vector3<f64>) -> Vec<f64> {
        let eclipse = Output::new(EclipseMsg {
            illumination_factor: eclipse_value,
        });
        let sun = Output::new(SunEphemerisMsg::default());
        let state = Output::new(SpacecraftStateMsg {
            position_m: Vector3::new(-orbit_distance_m, 0.0, 0.0),
            sigma_bn: sc_attitude,
            ..Default::default()
        });

        let mut panel = SimpleSolarPanel::new(
            "panel",
            SimpleSolarPanelModel::new(Vector3::new(1.0, 0.0, 0.0), 1.0, 1.0),
        );
        let mut recorder = MessageRecorder::<PowerNodeUsageMsg>::default();
        {
            let mut sim = Simulation::new(epoch(), false);
            sim.connect(&state, &mut panel.model.state_in_msg);
            sim.connect(&eclipse, &mut panel.model.sun_eclipse_in_msg);
            sim.connect(&sun, &mut panel.model.sun_in_msg);
            sim.connect(&panel.node_power_out_msg, &mut recorder.input_msg);
            sim.add_module("panel", &mut panel, 1_000_000_000, 1);
            sim.add_module("recorder", &mut recorder, 1_000_000_000, 0);
            sim.run_for(1_000_000_000);
        }
        recorder
            .samples
            .iter()
            .map(|sample| sample.net_power_w)
            .collect()
    }

    /// Upstream `test_simpleSolarPanel`: shadowing, attitude (sun-facing, 60°
    /// and 90° away) and distance (1 AU and 1.52 AU) all scale the 1361 W of
    /// a face-on unit panel at Earth.
    #[test]
    fn power_scales_with_eclipse_attitude_and_distance() {
        for orbit_distance_m in [1000.0 * AU, 1000.0 * 1.52 * AU] {
            for eclipse_value in [0.0, 1.0] {
                for sc_attitude in [
                    Vector3::zeros(),
                    pitched_attitude(60.0_f64.to_radians()),
                    pitched_attitude(90.0_f64.to_radians()),
                ] {
                    let reference_power = SOLAR_FLUX_EARTH;
                    let sun_distance_mult = (AU * 1000.0).powi(2) / orbit_distance_m.powi(2);
                    // cos of the principal rotation angle: the attitude angle
                    // away from the Sun.
                    let sc_att_mult = (0.5 * (mrp_to_dcm(sc_attitude).trace() - 1.0))
                        .acos()
                        .abs()
                        .cos();
                    let reference_multiplier =
                        1.0 * eclipse_value * sun_distance_mult * sc_att_mult;

                    let power = run(orbit_distance_m, eclipse_value, sc_attitude);
                    let truth = reference_power * reference_multiplier;
                    assert!(
                        (power[1] - truth).abs() < 1.0e-7,
                        "distance {orbit_distance_m} m, eclipse {eclipse_value}, \
                         attitude {sc_attitude:?}: {} W != {truth} W",
                        power[1]
                    );
                }
            }
        }
    }

    fn context() -> SimulationContext {
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: epoch(),
        }
    }

    /// One update of a panel facing `+x`, with the Sun 1 AU along `+x`.
    fn panel_power(
        panel: &mut SimpleSolarPanel,
        sigma_bn: Vector3<f64>,
        illumination: Option<f64>,
    ) -> f64 {
        let sun = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::new(AU * 1000.0, 0.0, 0.0),
            ..Default::default()
        });
        let state = Output::new(SpacecraftStateMsg {
            sigma_bn,
            ..Default::default()
        });
        let eclipse = illumination.map(|illumination_factor| {
            Output::new(EclipseMsg {
                illumination_factor,
            })
        });
        sun.connect_to(&mut panel.model.sun_in_msg);
        state.connect_to(&mut panel.model.state_in_msg);
        if let Some(eclipse) = &eclipse {
            eclipse.connect_to(&mut panel.model.sun_eclipse_in_msg);
        }
        panel.init();
        panel.update(&context());
        panel.node_power_out_msg.read().net_power_w
    }

    /// A panel facing away from the Sun projects no area rather than a
    /// negative one.
    #[test]
    fn a_panel_facing_away_supplies_nothing() {
        let mut panel = SimpleSolarPanel::new(
            "panel",
            SimpleSolarPanelModel::new(Vector3::new(-1.0, 0.0, 0.0), 2.0, 0.3),
        );
        assert_eq!(panel_power(&mut panel, Vector3::zeros(), None), 0.0);
    }

    /// The normal is normalized at reset; without an eclipse input the panel
    /// is fully lit; a partial eclipse scales the power.
    #[test]
    fn the_normal_is_normalized_and_illumination_defaults_to_one() {
        let mut panel = SimpleSolarPanel::new(
            "panel",
            SimpleSolarPanelModel::new(Vector3::new(5.0, 0.0, 0.0), 2.0, 0.3),
        );
        let power = panel_power(&mut panel, Vector3::zeros(), None);
        assert!((power - SOLAR_FLUX_EARTH * 2.0 * 0.3).abs() < 1.0e-9);
        assert_eq!(panel.model.n_hat_b, Vector3::new(1.0, 0.0, 0.0));

        let mut shaded = SimpleSolarPanel::new(
            "shaded",
            SimpleSolarPanelModel::new(Vector3::new(1.0, 0.0, 0.0), 2.0, 0.3),
        );
        let power = panel_power(&mut shaded, Vector3::zeros(), Some(0.25));
        assert!((power - 0.25 * SOLAR_FLUX_EARTH * 2.0 * 0.3).abs() < 1.0e-9);
    }

    /// Rotating the body a quarter turn about `z` turns the `-y` panel to
    /// the Sun.
    #[test]
    fn attitude_rotates_the_sun_into_the_body_frame() {
        let mut panel = SimpleSolarPanel::new(
            "panel",
            SimpleSolarPanelModel::new(Vector3::new(0.0, -1.0, 0.0), 1.0, 1.0),
        );
        // [BN] = rotation by +90° about z: inertial +x is body -y.
        let sigma_bn = dcm_to_mrp(euler321_to_dcm(Vector3::new(FRAC_PI_2, 0.0, 0.0)));
        let power = panel_power(&mut panel, sigma_bn, None);
        assert!((power - SOLAR_FLUX_EARTH).abs() < 1.0e-9, "{power}");
    }

    #[test]
    fn unconnected_required_inputs_are_reported() {
        let mut panel = SimpleSolarPanel::new(
            "panel",
            SimpleSolarPanelModel::new(Vector3::new(1.0, 0.0, 0.0), 1.0, 1.0),
        );
        assert_eq!(
            panel.model.unconnected_inputs(),
            ["sun_in_msg", "state_in_msg"]
        );
        let sun = Output::new(SunEphemerisMsg::default());
        sun.connect_to(&mut panel.model.sun_in_msg);
        assert_eq!(panel.model.unconnected_inputs(), ["state_in_msg"]);
    }

    #[test]
    #[should_panic(expected = "panel_area_m2")]
    fn a_negative_area_is_rejected() {
        let mut panel = SimpleSolarPanel::new(
            "panel",
            SimpleSolarPanelModel::new(Vector3::new(1.0, 0.0, 0.0), -1.0, 1.0),
        );
        panel.init();
    }

    #[test]
    #[should_panic(expected = "panel_efficiency")]
    fn a_negative_efficiency_is_rejected() {
        let mut panel = SimpleSolarPanel::new(
            "panel",
            SimpleSolarPanelModel::new(Vector3::new(1.0, 0.0, 0.0), 1.0, -1.0),
        );
        panel.init();
    }

    #[test]
    #[should_panic(expected = "n_hat_b")]
    fn a_zero_normal_is_rejected() {
        let mut panel = SimpleSolarPanel::new(
            "panel",
            SimpleSolarPanelModel::new(Vector3::zeros(), 1.0, 1.0),
        );
        panel.init();
    }

    /// End to end: a spacecraft in a 7000 km equatorial orbit, with the Sun
    /// along inertial `+x` and a fixed attitude, flies one orbit through
    /// Earth's shadow. A 2 m², 25 % panel facing `+x` and lit through
    /// `Eclipse` charges a `SimpleBattery` while a 300 W `SimplePowerSink`
    /// drains it.
    ///
    /// The battery level must equal its Euler sum of the panel and sink
    /// outputs, rise in sunlight and fall in shadow; the panel must supply
    /// nothing in the umbra; and in sunlight the panel must agree with
    /// `SolarFlux`, which sees the same geometry and eclipse with its own flux
    /// constant.
    #[test]
    fn a_sunlit_panel_charges_a_battery_that_a_sink_drains() {
        const STEP_NANOS: u64 = 10_000_000_000;
        const PANEL_AREA_M2: f64 = 2.0;
        const PANEL_EFFICIENCY: f64 = 0.25;
        const SINK_POWER_W: f64 = -300.0;
        const STORED_CHARGE_INIT_J: f64 = 2.0e6;

        let mu_m3ps2 = MU_EARTH * 1.0e9;
        let radius_m = 7_000_000.0;
        let speed_mps = (mu_m3ps2 / radius_m).sqrt();
        let period_s = 2.0 * std::f64::consts::PI * radius_m / speed_mps;
        let steps = (period_s * 1.0e9 / STEP_NANOS as f64).ceil() as u64;

        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 100.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::from_diagonal_element(10.0),
            // One integration step per task tick.
            integration_step_nanos: STEP_NANOS,
            initial_position_m: Vector3::new(radius_m, 0.0, 0.0),
            initial_velocity_mps: Vector3::new(0.0, speed_mps, 0.0),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: None,
        });
        spacecraft
            .add_grav_body(
                GravBodyData::point_mass(
                    "earth",
                    mu_m3ps2,
                    true,
                    Vector3::zeros(),
                    Vector3::zeros(),
                )
                .expect("valid Earth gravity body"),
            )
            .expect("unique central gravity body");
        let sun = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::new(AU * 1000.0, 0.0, 0.0),
            ..Default::default()
        });
        let mut eclipse = Eclipse::new(EclipseConfig {
            name: "eclipse".to_string(),
            occulting_body_radius_m: REQ_EARTH * 1000.0,
        });
        let mut solar_flux = SolarFlux::new(SolarFluxConfig {
            name: "solarFlux".to_string(),
        });
        let mut panel = SimpleSolarPanel::new(
            "panel",
            SimpleSolarPanelModel::new(
                Vector3::new(1.0, 0.0, 0.0),
                PANEL_AREA_M2,
                PANEL_EFFICIENCY,
            ),
        );
        let mut sink = SimplePowerSink::new(
            "sink",
            SimplePowerSinkModel {
                node_power_out_w: SINK_POWER_W,
            },
        );
        let mut battery = SimpleBattery::new(SimpleBatteryConfig {
            name: "battery".to_string(),
            storage_capacity_j: 1.0e7,
            stored_charge_init_j: STORED_CHARGE_INIT_J,
        });

        let state_out = spacecraft.state_out.clone();
        let panel_out = panel.node_power_out_msg.clone();
        let eclipse_out = eclipse.output_eclipse_msg.clone();
        let flux_out = solar_flux.solar_flux_out_msg.clone();
        let battery_out = battery.bat_power_out_msg.clone();

        let mut sim = Simulation::new(epoch(), false);
        sim.connect(&spacecraft.state_out, &mut eclipse.input_state_msg);
        sim.connect(&sun, &mut eclipse.input_sun_msg);
        sim.connect(
            &spacecraft.state_out,
            &mut solar_flux.spacecraft_state_in_msg,
        );
        sim.connect(&sun, &mut solar_flux.sun_position_in_msg);
        sim.connect(&eclipse.output_eclipse_msg, &mut solar_flux.eclipse_in_msg);
        sim.connect(&spacecraft.state_out, &mut panel.model.state_in_msg);
        sim.connect(&sun, &mut panel.model.sun_in_msg);
        sim.connect(
            &eclipse.output_eclipse_msg,
            &mut panel.model.sun_eclipse_in_msg,
        );
        sim.connect(&panel.node_power_out_msg, battery.add_power_node_to_model());
        sim.connect(&sink.node_power_out_msg, battery.add_power_node_to_model());
        sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 50);
        sim.add_module("eclipse", &mut eclipse, STEP_NANOS, 40);
        sim.add_module("solarFlux", &mut solar_flux, STEP_NANOS, 30);
        sim.add_module("panel", &mut panel, STEP_NANOS, 20);
        sim.add_module("sink", &mut sink, STEP_NANOS, 20);
        sim.add_module("battery", &mut battery, STEP_NANOS, 10);

        // `SolarFlux`'s own flux at 1 AU.
        let solar_flux_1au_w_per_m2 = 1361.0;
        let mut expected_charge_j = STORED_CHARGE_INIT_J;
        let mut previous_charge_j = STORED_CHARGE_INIT_J;
        let (mut sunlit_steps, mut umbra_steps) = (0, 0);
        for step in 0..=steps {
            sim.run_for(if step == 0 { 0 } else { STEP_NANOS });
            let panel_w = panel_out.read().net_power_w;
            let illumination = eclipse_out.read().illumination_factor;
            let charge_j = battery_out.read().storage_level_j;
            if step > 0 {
                // The battery's own Euler step, time stamps and all.
                let dt_s =
                    (step * STEP_NANOS) as f64 * 1.0e-9 - ((step - 1) * STEP_NANOS) as f64 * 1.0e-9;
                expected_charge_j += (panel_w + SINK_POWER_W) * dt_s;
            }
            assert!(
                (charge_j - expected_charge_j).abs() < 1.0e-6,
                "step {step}: battery {charge_j} J != {expected_charge_j} J"
            );

            if illumination == 0.0 {
                umbra_steps += 1;
                assert_eq!(panel_w, 0.0, "step {step}: power in the umbra");
                if step > 0 {
                    assert!(charge_j < previous_charge_j, "step {step}: no drain");
                }
            } else if illumination == 1.0 {
                sunlit_steps += 1;
                assert!(panel_w > -SINK_POWER_W, "step {step}: panel {panel_w} W");
                if step > 0 {
                    assert!(charge_j > previous_charge_j, "step {step}: no charge");
                }
                // Both scale the same distance and eclipse; the panel adds
                // its incidence angle.
                let r_sb_n = sun.read().sun_position_inertial_m - state_out.read().position_m;
                let incidence_cos = r_sb_n.normalize().x;
                let flux = flux_out.read().flux_w_per_m2;
                let panel_flux = panel_w / (PANEL_AREA_M2 * PANEL_EFFICIENCY * incidence_cos);
                assert!(
                    (panel_flux / SOLAR_FLUX_EARTH - flux / solar_flux_1au_w_per_m2).abs()
                        < 1.0e-12,
                    "step {step}: panel {panel_flux} W/m^2, flux {flux} W/m^2"
                );
            }
            previous_charge_j = charge_j;
        }
        drop(sim);

        // A 7000 km orbit spends roughly a third of its period in shadow.
        let total = steps + 1;
        assert!(umbra_steps * 5 > total, "{umbra_steps} of {total} in umbra");
        assert!(sunlit_steps * 2 > total, "{sunlit_steps} of {total} sunlit");
        let final_charge_j = battery.bat_power_out_msg.read().storage_level_j;
        assert!(final_charge_j > STORED_CHARGE_INIT_J);
    }
}
