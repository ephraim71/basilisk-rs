//! Conversion between Cartesian states and classical orbit elements: port of
//! upstream Basilisk `simulation/dynamics/DynOutput/orbElemConvert`.
//!
//! Exactly one input is connected: orbit elements, which are converted to an
//! inertial position and velocity
//! ([`elements_to_state`]), or a spacecraft or planet state, whose position and
//! velocity are converted to elements ([`state_to_elements`]), both about a
//! body of gravitational parameter `mu_m3ps2`. The module then publishes the
//! position and velocity on both state outputs and the elements on the element
//! output.
//!
//! Upstream also requires at least one output to be linked and writes only the
//! linked ones; outputs here do not know their consumers, so all three are
//! written every update. The state outputs carry only the position and
//! velocity, the rest zero, as upstream's zeroed payloads. Upstream converts
//! only once its input has been written. Stand-in: connected ⇒ written.

use nalgebra::Vector3;

use crate::messages::{ClassicElementsMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg};
use crate::utilities::orbital_motion::{ClassicElements, elements_to_state, state_to_elements};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`OrbElemConvert`].
#[derive(Clone, Debug, Default)]
pub struct OrbElemConvertConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `mu` \[m^3/s^2\]: gravitational parameter of the central body.
    pub mu_m3ps2: f64,
}

/// Orbit element / Cartesian state converter: upstream `OrbElemConvert`.
#[derive(Clone, Debug)]
pub struct OrbElemConvert {
    /// User-settable parameters.
    pub config: OrbElemConvertConfig,
    /// Upstream `scStateInMsg`: a spacecraft state to convert to elements.
    pub sc_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `spiceStateInMsg`: a planet state to convert to elements.
    pub spice_state_in_msg: Input<PlanetStateMsg>,
    /// Upstream `elemInMsg`: elements to convert to a state.
    pub elem_in_msg: Input<ClassicElementsMsg>,
    /// Upstream `scStateOutMsg`: the position and velocity.
    pub sc_state_out_msg: Output<SpacecraftStateMsg>,
    /// Upstream `spiceStateOutMsg`: the position and velocity.
    pub spice_state_out_msg: Output<PlanetStateMsg>,
    /// Upstream `elemOutMsg`: the elements.
    pub elem_out_msg: Output<ClassicElementsMsg>,
    position_m: Vector3<f64>,
    velocity_mps: Vector3<f64>,
    current_elements: ClassicElements,
}

impl OrbElemConvert {
    pub fn new(config: OrbElemConvertConfig) -> Self {
        Self {
            config,
            sc_state_in_msg: Input::default(),
            spice_state_in_msg: Input::default(),
            elem_in_msg: Input::default(),
            sc_state_out_msg: Output::default(),
            spice_state_out_msg: Output::default(),
            elem_out_msg: Output::default(),
            position_m: Vector3::zeros(),
            velocity_mps: Vector3::zeros(),
            current_elements: ClassicElements::default(),
        }
    }

    /// Upstream `r_N` \[m\]: the inertial position of the last update.
    pub fn position_m(&self) -> Vector3<f64> {
        self.position_m
    }

    /// Upstream `v_N` \[m/s\]: the inertial velocity of the last update.
    pub fn velocity_mps(&self) -> Vector3<f64> {
        self.velocity_mps
    }

    /// Upstream `CurrentElem`: the elements of the last update.
    pub fn current_elements(&self) -> ClassicElements {
        self.current_elements
    }

    fn connected_input_count(&self) -> usize {
        [
            self.sc_state_in_msg.is_connected(),
            self.spice_state_in_msg.is_connected(),
            self.elem_in_msg.is_connected(),
        ]
        .into_iter()
        .filter(|connected| *connected)
        .count()
    }
}

impl Module for OrbElemConvert {
    /// Upstream `Reset`: exactly one input must be connected.
    ///
    /// # Panics
    ///
    /// If no input, or more than one, is connected.
    fn init(&mut self) {
        match self.connected_input_count() {
            0 => panic!(
                "OrbElemConvert '{}': no input message was connected",
                self.config.name
            ),
            1 => {}
            count => panic!(
                "OrbElemConvert '{}': found {count} input messages, there can be only one",
                self.config.name
            ),
        }
    }

    fn update(&mut self, _context: &SimulationContext) {
        let mu = self.config.mu_m3ps2;
        if self.elem_in_msg.is_connected() {
            self.current_elements = self.elem_in_msg.read();
            (self.position_m, self.velocity_mps) = elements_to_state(mu, &self.current_elements);
        } else {
            (self.position_m, self.velocity_mps) = if self.sc_state_in_msg.is_connected() {
                let state = self.sc_state_in_msg.read();
                (state.position_m, state.velocity_mps)
            } else {
                let state = self.spice_state_in_msg.read();
                (state.position_inertial_m, state.velocity_inertial_mps)
            };
            self.current_elements = state_to_elements(mu, self.position_m, self.velocity_mps);
        }

        self.elem_out_msg.write(self.current_elements);
        self.sc_state_out_msg.write(SpacecraftStateMsg {
            position_m: self.position_m,
            velocity_mps: self.velocity_mps,
            ..Default::default()
        });
        self.spice_state_out_msg.write(PlanetStateMsg {
            position_inertial_m: self.position_m,
            velocity_inertial_mps: self.velocity_mps,
            orientation: None,
        });
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `orbElemConvert/_UnitTest/test_orb_elem_convert.py`
    //! `test_orb_elem_convert` over its full parameter list, with its
    //! tolerances: elements to state against the closed-form `elem2rv` truth,
    //! then the state, as a spacecraft and as a planet message, back to
    //! elements, against the input elements, the published elements and
    //! upstream's Python re-derivation of `rv2elem`.
    //!
    //! `test_singular_state_message` is ported for its exactly singular
    //! inclinations (0 and π). Not ported: its near-singular ones (`1e-8` and
    //! `π − 1e-8`), and `test_degenerate_conic_messages`. They check upstream's
    //! rewritten `rv2elem`/`elem2rv` (inclination by `atan2` near the equator,
    //! radial and rest states with `e = 1` and `rPeriap = 0`, eccentricities
    //! within `1e-12` of one treated as ordinary conics), which
    //! `utilities::orbital_motion` predates; they belong with an update of
    //! that shared utility, not with this module.

    use std::f64::consts::PI;

    use hifitime::Epoch;

    use super::*;
    use crate::simulation::Simulation;

    const MU_M3PS2: f64 = 0.3986004415E+15;

    fn run_once(module: &mut OrbElemConvert) {
        let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        sim.add_module("OrbElemConvertData", module, 1_000_000_000, 0);
        sim.run_for(1_000_000_000);
    }

    #[track_caller]
    fn assert_allclose(actual: Vector3<f64>, truth: Vector3<f64>, rtol: f64, what: &str) {
        for (a, t) in actual.iter().zip(truth.iter()) {
            assert!(
                (a - t).abs() <= rtol * t.abs(),
                "{what}: {actual:?} vs {truth:?}"
            );
        }
    }

    /// `ElemDiff` entries above index 2 are angles, compared modulo 2π.
    fn wrap_two_pi(index: usize, mut diff: f64, eps: f64) -> f64 {
        if index > 2 {
            if (diff - 2.0 * PI).abs() < eps {
                diff -= 2.0 * PI;
            } else if (diff + 2.0 * PI).abs() < eps {
                diff += 2.0 * PI;
            }
        }
        diff
    }

    /// Upstream's closed-form `elem2rv` truth.
    fn elem2rv_truth(
        mut a: f64,
        e: f64,
        i: f64,
        an: f64,
        ap: f64,
        f: f64,
    ) -> (Vector3<f64>, Vector3<f64>, f64) {
        let p = if e == 1.0 && a < 0.0 {
            let rp = -a;
            a = 0.0;
            2.0 * rp
        } else {
            a * (1.0 - e * e)
        };
        let r = p / (1.0 + e * f.cos());
        let theta = ap + f;
        let h = (MU_M3PS2 * p).sqrt();
        let r_truth = Vector3::new(
            r * (an.cos() * theta.cos() - an.sin() * theta.sin() * i.cos()),
            r * (an.sin() * theta.cos() + an.cos() * theta.sin() * i.cos()),
            r * (theta.sin() * i.sin()),
        );
        let v_truth = Vector3::new(
            -MU_M3PS2 / h
                * (an.cos() * (theta.sin() + e * ap.sin())
                    + an.sin() * (theta.cos() + e * ap.cos()) * i.cos()),
            -MU_M3PS2 / h
                * (an.sin() * (theta.sin() + e * ap.sin())
                    - an.cos() * (theta.cos() + e * ap.cos()) * i.cos()),
            -MU_M3PS2 / h * (-(theta.cos() + e * ap.cos()) * i.sin()),
        );
        (r_truth, v_truth, a)
    }

    /// Upstream's Python re-derivation of `rv2elem`: `(a, e, i, Ω, ω)`.
    fn rv2elem_truth(r_vec: Vector3<f64>, v_vec: Vector3<f64>) -> [f64; 5] {
        let eps_conv = 0.0000000001;
        let h_vec = r_vec.cross(&v_vec);
        let h = h_vec.norm();
        let n_vec = Vector3::z().cross(&h_vec);
        let n = n_vec.norm();
        let r = r_vec.norm();
        let v = v_vec.norm();
        let e_vec = (v * v / MU_M3PS2 - 1.0 / r) * r_vec - (r_vec.dot(&v_vec) / MU_M3PS2) * v_vec;
        let e = e_vec.norm();
        let alpha = 2.0 / r - v * v / MU_M3PS2;
        let a = if alpha.abs() > eps_conv {
            1.0 / alpha
        } else {
            0.0
        };
        let i = (h_vec[2] / h).acos();
        let (omega_node, omega_peri) = if e >= 1e-11 && i >= 1e-11 {
            let mut node = (n_vec[0] / n).acos();
            if n_vec[1] < 0.0 {
                node = 2.0 * PI - node;
            }
            let mut peri = (n_vec.dot(&e_vec) / n / e).acos();
            if e_vec[2] < 0.0 {
                peri = 2.0 * PI - peri;
            }
            (node, peri)
        } else if e >= 1e-11 && i < 1e-11 {
            let mut peri = (e_vec[0] / e).acos();
            if e_vec[1] < 0.0 {
                peri = 2.0 * PI - peri;
            }
            (0.0, peri)
        } else if e < 1e-11 && i >= 1e-11 {
            let mut node = (n_vec[0] / n).acos();
            if n_vec[1] < 0.0 {
                node = 2.0 * PI - node;
            }
            (node, 0.0)
        } else {
            (0.0, 0.0)
        };
        [a, e, i, omega_node, omega_peri]
    }

    /// Upstream `orbElem` for one parameter set.
    fn orb_elem(a: f64, e: f64, i: f64, an: f64, ap: f64, f: f64) {
        let eps_diff = 1e-5;
        let mut elements = ClassicElements {
            eccentricity: e,
            inclination_rad: i,
            raan_rad: an,
            arg_periapsis_rad: ap,
            true_anomaly_rad: f,
            ..Default::default()
        };
        if e == 1.0 {
            elements.semi_major_axis = 0.0;
            elements.periapsis_radius = -a;
        } else {
            elements.semi_major_axis = a;
        }
        let elements_out = Output::new(elements);
        let mut forward = OrbElemConvert::new(OrbElemConvertConfig {
            name: "OrbElemConvertData".to_string(),
            mu_m3ps2: MU_M3PS2,
        });
        elements_out.connect_to(&mut forward.elem_in_msg);
        run_once(&mut forward);

        let (r_sim, v_sim) = (forward.position_m(), forward.velocity_mps());
        let planet = forward.spice_state_out_msg.read();
        assert_allclose(
            r_sim,
            planet.position_inertial_m,
            eps_diff,
            "planet position message",
        );
        assert_allclose(
            v_sim,
            planet.velocity_inertial_mps,
            eps_diff,
            "planet velocity message",
        );
        let (r_truth, v_truth, a) = elem2rv_truth(a, e, i, an, ap, f);
        assert_allclose(r_sim, r_truth, eps_diff, "position vector");
        assert_allclose(v_sim, v_truth, eps_diff, "velocity vector");

        for as_planet in [false, true] {
            let mut inverse = OrbElemConvert::new(OrbElemConvertConfig {
                name: "OrbElemConvertData".to_string(),
                mu_m3ps2: MU_M3PS2,
            });
            let sc_out = Output::new(SpacecraftStateMsg {
                position_m: r_sim,
                velocity_mps: v_sim,
                ..Default::default()
            });
            let planet_out = Output::new(PlanetStateMsg {
                position_inertial_m: r_sim,
                velocity_inertial_mps: v_sim,
                orientation: None,
            });
            if as_planet {
                planet_out.connect_to(&mut inverse.spice_state_in_msg);
            } else {
                sc_out.connect_to(&mut inverse.sc_state_in_msg);
            }
            run_once(&mut inverse);
            let out = inverse.current_elements();
            let out_values = [
                out.semi_major_axis,
                out.eccentricity,
                out.inclination_rad,
                out.raan_rad,
                out.arg_periapsis_rad,
                out.true_anomaly_rad,
            ];
            let inputs = [a, e, i, an, ap, f];
            for (index, (input, output)) in inputs.iter().zip(&out_values).enumerate() {
                let diff = wrap_two_pi(index, input - output, eps_diff);
                assert!(
                    diff.abs() <= eps_diff,
                    "sim orbital element {index}: {input} vs {output}"
                );
            }
            assert_eq!(inverse.elem_out_msg.read(), out, "orbital element message");
            let derived = rv2elem_truth(r_truth, v_truth);
            for (index, (truth, output)) in derived.iter().zip(&out_values).enumerate() {
                let diff = wrap_two_pi(index, truth - output, eps_diff);
                assert!(
                    diff.abs() <= eps_diff,
                    "calculated orbital element {index}: {truth} vs {output}"
                );
            }
        }
    }

    /// Upstream `test_orb_elem_convert` over its parameter list: inclined
    /// and equatorial ellipses (in `e` and in `a`), circles, parabolas and
    /// hyperbolas (in `a` and in `e`).
    #[test]
    fn elements_and_states_convert_both_ways_for_every_conic() {
        let d2r = PI / 180.0;
        let (inc, an, ap, f) = (33.3 * d2r, 48.2 * d2r, 347.8 * d2r, 85.3 * d2r);
        let mut cases = Vec::new();
        for (i, an) in [(inc, an), (0.0, 0.0)] {
            for e in [0.01, 0.10, 0.25, 0.50, 0.75] {
                cases.push((10000000.0, e, i, an, ap, f));
            }
            for a in [10000000.0, 100000.0, 10000.0, 1000.0, 100.0, 10.0] {
                cases.push((a, 0.50, i, an, ap, f));
            }
        }
        for (i, an) in [(inc, an), (0.0, 0.0)] {
            for a in [
                10000000.0, 1000000.0, 100000.0, 10000.0, 1000.0, 100.0, 10.0,
            ] {
                cases.push((a, 0.0, i, an, 0.0, f));
            }
        }
        for (i, an) in [(inc, an), (0.0, 0.0)] {
            for a in [-10.0, -100.0, -1000.0, -10000.0, -100000.0] {
                cases.push((a, 1.0, i, an, ap, f));
            }
        }
        for (i, an) in [(inc, an), (0.0, 0.0)] {
            for a in [-10.0, -100.0, -1000.0, -10000.0, -100000.0] {
                cases.push((a, 1.3, i, an, ap, f));
            }
            for e in [1.1, 1.2, 1.3, 1.4, 1.5] {
                cases.push((-100000.0, e, i, an, ap, f));
            }
        }
        assert_eq!(cases.len(), 66);
        for (a, e, i, an, ap, f) in cases {
            orb_elem(a, e, i, an, ap, f);
        }
    }

    /// Upstream `test_singular_state_message` for its exactly equatorial and
    /// retrograde-equatorial inclinations: a periapsis state (or any point of
    /// a circle) keeps its singular-angle conventions through the element
    /// message, and converts back to the same state. The upstream Python
    /// reference elements are their closed forms here.
    #[test]
    fn singular_states_keep_their_conventions() {
        let mu = 398600.436 * 1e9;
        let radius = 6e6;
        let wrapped = |angle: f64| angle - (angle / (2.0 * PI)).round() * 2.0 * PI;
        for eccentricity in [0.0, 0.3] {
            for inclination in [0.0, PI] {
                let cos_i = inclination.cos();
                let position = radius * Vector3::new(0.6, 0.8 * cos_i, 0.0);
                let speed = (mu * (1.0 + eccentricity) / radius).sqrt();
                let velocity = speed * Vector3::new(-0.8, 0.6 * cos_i, 0.0);
                let state_out = Output::new(SpacecraftStateMsg {
                    position_m: position,
                    velocity_mps: velocity,
                    ..Default::default()
                });
                let mut converter = OrbElemConvert::new(OrbElemConvertConfig {
                    name: "converter".to_string(),
                    mu_m3ps2: mu,
                });
                state_out.connect_to(&mut converter.sc_state_in_msg);
                run_once(&mut converter);
                let result = converter.elem_out_msg.read();

                let case = format!("e = {eccentricity}, i = {inclination}: {result:?}");
                assert!(
                    (result.inclination_rad - inclination).abs() <= 2e-15,
                    "{case}"
                );
                assert!(
                    (result.eccentricity - eccentricity).abs() <= 2e-15,
                    "{case}"
                );
                assert!(wrapped(result.raan_rad).abs() < 2e-14, "{case}");
                let (expected_periapsis, expected_anomaly) = if eccentricity == 0.0 {
                    (0.0, 0.8_f64.atan2(0.6))
                } else {
                    (0.8_f64.atan2(0.6), 0.0)
                };
                assert!(
                    wrapped(result.arg_periapsis_rad - expected_periapsis).abs() < 2e-14,
                    "{case}"
                );
                assert!(
                    wrapped(result.true_anomaly_rad - expected_anomaly).abs() < 2e-14,
                    "{case}"
                );
                let semi_major_axis = radius / (1.0 - eccentricity);
                for (value, truth) in [
                    (result.semi_major_axis, semi_major_axis),
                    (result.inverse_semi_major_axis, 1.0 / semi_major_axis),
                    (result.orbit_radius, radius),
                    (result.periapsis_radius, radius),
                    (
                        result.apoapsis_radius,
                        semi_major_axis * (1.0 + eccentricity),
                    ),
                ] {
                    assert!(
                        (value - truth).abs() <= 2e-15 + 2e-14 * truth.abs(),
                        "{case}"
                    );
                }

                let mut inverse = OrbElemConvert::new(OrbElemConvertConfig {
                    name: "inverse".to_string(),
                    mu_m3ps2: mu,
                });
                converter.elem_out_msg.connect_to(&mut inverse.elem_in_msg);
                run_once(&mut inverse);
                assert!(
                    ((inverse.position_m() - position) / radius).abs().max() <= 2e-14,
                    "{case}"
                );
                assert!(
                    ((inverse.velocity_mps() - velocity) / speed).abs().max() <= 2e-14,
                    "{case}"
                );
            }
        }
    }

    #[test]
    #[should_panic(expected = "no input message was connected")]
    fn requires_an_input() {
        OrbElemConvert::new(OrbElemConvertConfig::default()).init();
    }

    #[test]
    #[should_panic(expected = "found 2 input messages")]
    fn accepts_only_one_input() {
        let mut module = OrbElemConvert::new(OrbElemConvertConfig::default());
        Output::<SpacecraftStateMsg>::default().connect_to(&mut module.sc_state_in_msg);
        Output::<ClassicElementsMsg>::default().connect_to(&mut module.elem_in_msg);
        module.init();
    }
}
