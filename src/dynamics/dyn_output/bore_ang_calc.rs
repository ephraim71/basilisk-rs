//! Boresight angles to a celestial body or an inertial heading: port of
//! upstream Basilisk `simulation/dynamics/DynOutput/boreAngCalc`.
//!
//! The boresight `bore_vec_body` is a body-fixed direction (used as given, not
//! normalized). With a celestial body connected, the module builds the
//! pointing frame `Po` of the spacecraft relative to the body: `x̂` along the
//! relative position `r_P/B`, `ẑ` along `r̂_P/B × ĥ`, where `ĥ` is the unit
//! relative angular momentum `r_P/B × v_P/B`, and `ŷ = ẑ × x̂`. The miss angle
//! is the angle between the boresight and `x̂`, and the azimuth
//! `atan2(b_z, b_y)` places the miss about `x̂` (zero when `|b_y| < 1e-10`).
//! Without a celestial body, the miss angle is the angle between the boresight
//! and the inertial heading `inertial_heading_inertial`, and the azimuth is
//! zero.
//!
//! Upstream recomputes only when its inputs have been written, and otherwise
//! republishes the previous angles. Stand-in: connected ⇒ written, so every
//! update recomputes.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{BoreAngleMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg};
use crate::utilities::linear_algebra::{normalize_or_zero, safe_acos};
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Upstream's `eps`: below this `|b_y|` the azimuth is zero rather than an
/// ill-conditioned `atan2`.
const AZIMUTH_EPSILON: f64 = 1e-10;

/// An inertial heading at most this long counts as unset (upstream `1e-8`).
const MIN_INERTIAL_HEADING_NORM: f64 = 1e-8;

/// User-settable parameters of [`BoreAngCalc`].
#[derive(Clone, Debug, Default)]
pub struct BoreAngCalcConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `boreVec_B` \[-\]: the boresight, body components.
    pub bore_vec_body: Vector3<f64>,
    /// Upstream `inertialHeadingVec_N` \[-\]: the target heading, inertial
    /// components, used when no celestial body is connected.
    pub inertial_heading_inertial: Vector3<f64>,
}

/// The target the boresight is measured against, fixed by `init`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Target {
    /// Before `init`.
    #[default]
    Unset,
    /// Upstream `useCelestialHeading`: the connected celestial body.
    CelestialBody,
    /// Upstream `useInertialHeading`: the configured inertial heading.
    InertialHeading,
}

/// Boresight miss and azimuth angles: upstream `BoreAngCalc`.
#[derive(Clone, Debug)]
pub struct BoreAngCalc {
    /// User-settable parameters.
    pub config: BoreAngCalcConfig,
    /// Upstream `scStateInMsg`: the spacecraft state (required).
    pub sc_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `celBodyInMsg`: the celestial body pointed at (optional).
    pub cel_body_in_msg: Input<PlanetStateMsg>,
    /// Upstream `angOutMsg`: the boresight angles.
    pub ang_out_msg: Output<BoreAngleMsg>,
    target: Target,
    bore_vec_point: Vector3<f64>,
}

impl BoreAngCalc {
    pub fn new(config: BoreAngCalcConfig) -> Self {
        Self {
            config,
            sc_state_in_msg: Input::default(),
            cel_body_in_msg: Input::default(),
            ang_out_msg: Output::default(),
            target: Target::Unset,
            bore_vec_point: Vector3::zeros(),
        }
    }

    /// Upstream `boreVec_Po`: the boresight in the pointing frame at the last
    /// celestial-body update (zero before one).
    pub fn bore_vec_point_frame(&self) -> Vector3<f64> {
        self.bore_vec_point
    }

    /// Upstream `computeCelestialAxisPoint`: the boresight in the pointing
    /// frame of `planet` relative to the spacecraft.
    fn celestial_axis_point(
        &self,
        state: &SpacecraftStateMsg,
        planet: &PlanetStateMsg,
    ) -> Vector3<f64> {
        let r_pb_n = planet.position_inertial_m - state.position_m;
        let r_hat_pb_n = normalize_or_zero(r_pb_n);
        let v_pb_n = planet.velocity_inertial_mps - state.velocity_mps;
        let sec_hat_n = normalize_or_zero(r_pb_n.cross(&v_pb_n));
        let z_hat = normalize_or_zero(r_hat_pb_n.cross(&sec_hat_n));
        let y_hat = z_hat.cross(&r_hat_pb_n);
        let dcm_po_n =
            Matrix3::from_rows(&[r_hat_pb_n.transpose(), y_hat.transpose(), z_hat.transpose()]);
        let dcm_b_po = mrp_to_dcm(state.sigma_bn) * dcm_po_n.transpose();
        dcm_b_po.transpose() * self.config.bore_vec_body
    }
}

/// Upstream `computeCelestialOutputData`: the angles of a boresight given in
/// the pointing frame.
fn celestial_angles(bore_vec_point: Vector3<f64>) -> BoreAngleMsg {
    BoreAngleMsg {
        miss_angle_rad: safe_acos(bore_vec_point.x).abs(),
        azimuth_rad: if bore_vec_point.y.abs() < AZIMUTH_EPSILON {
            0.0
        } else {
            bore_vec_point.z.atan2(bore_vec_point.y)
        },
    }
}

impl Module for BoreAngCalc {
    /// Upstream `Reset`: the spacecraft state is required, and the target is
    /// the celestial body if connected, else the inertial heading if set.
    ///
    /// # Panics
    ///
    /// If the spacecraft state is unconnected, or there is neither a
    /// celestial body nor an inertial heading.
    fn init(&mut self) {
        assert!(
            self.sc_state_in_msg.is_connected(),
            "BoreAngCalc '{}': sc_state_in_msg was not connected",
            self.config.name
        );
        self.target = if self.cel_body_in_msg.is_connected() {
            Target::CelestialBody
        } else if self.config.inertial_heading_inertial.norm() > MIN_INERTIAL_HEADING_NORM {
            Target::InertialHeading
        } else {
            panic!(
                "BoreAngCalc '{}': either cel_body_in_msg must be connected or \
                 inertial_heading_inertial set",
                self.config.name
            );
        };
        self.bore_vec_point = Vector3::zeros();
        self.ang_out_msg.write(BoreAngleMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.sc_state_in_msg.read();
        let angles = match self.target {
            Target::CelestialBody => {
                self.bore_vec_point =
                    self.celestial_axis_point(&state, &self.cel_body_in_msg.read());
                celestial_angles(self.bore_vec_point)
            }
            Target::InertialHeading => {
                // Upstream `computeInertialOutputData`.
                let heading_body =
                    mrp_to_dcm(state.sigma_bn) * self.config.inertial_heading_inertial;
                BoreAngleMsg {
                    miss_angle_rad: safe_acos(self.config.bore_vec_body.dot(&heading_body)).abs(),
                    azimuth_rad: 0.0,
                }
            }
            Target::Unset => panic!("BoreAngCalc '{}': update before init", self.config.name),
        };
        self.ang_out_msg.write(angles);
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `boreAngCalc/_UnitTest/test_bore_ang_calc.py` and
    //! `test_bore_ang_calc_inertial_heading.py` with their truth and
    //! tolerances; the miss and azimuth checks are made two-sided.

    use std::f64::consts::PI;

    use hifitime::Epoch;

    use super::*;
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::rigid_body_kinematics::euler321_to_mrp;

    /// Runs the module for one update at `t = 0` and returns its output and
    /// boresight in the pointing frame.
    fn run_once(module: &mut BoreAngCalc) -> (BoreAngleMsg, Vector3<f64>) {
        let mut log = MessageRecorder::<BoreAngleMsg>::default();
        module.ang_out_msg.connect_to(&mut log.input_msg);
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("solarArrayBoresight", module, 1_000_000_000, 10);
            sim.add_module("log", &mut log, 1_000_000_000, 0);
            sim.run_for(0);
        }
        (log.samples[0].clone(), module.bore_vec_point_frame())
    }

    /// Upstream `bore_ang_calc_func`: a spacecraft and the Sun 1 km apart,
    /// with the spacecraft's velocity across the line between them.
    fn celestial_case(bore_vec_body: Vector3<f64>, euler321: Vector3<f64>) {
        let sigma_bn = if euler321.x == 0.0 {
            Vector3::zeros()
        } else {
            euler321_to_mrp(euler321)
        };
        let state = Output::new(SpacecraftStateMsg {
            position_m: Vector3::new(10000.0, 0.0, 0.0),
            velocity_mps: Vector3::new(-365052.0511, 0.0, 0.0),
            sigma_bn,
            ..Default::default()
        });
        let sun = Output::new(PlanetStateMsg {
            position_inertial_m: Vector3::new(10000.0, 1000.0, 0.0),
            ..Default::default()
        });
        let mut module = BoreAngCalc::new(BoreAngCalcConfig {
            name: "solarArrayBoresight".to_string(),
            bore_vec_body,
            ..Default::default()
        });
        state.connect_to(&mut module.sc_state_in_msg);
        sun.connect_to(&mut module.cel_body_in_msg);
        let (angles, bore_vec_point) = run_once(&mut module);

        // Upstream's truth, step by step.
        let state = state.read();
        let sun = sun.read();
        let dcm_bn = mrp_to_dcm(sigma_bn);
        let rel_pos = sun.position_inertial_m - state.position_m;
        let rel_vel = sun.velocity_inertial_mps - state.velocity_mps;
        let sec_point = rel_pos.cross(&rel_vel) / rel_pos.cross(&rel_vel).norm();
        let prim_point = rel_pos / rel_pos.norm();
        let mut dcm_po_n = Matrix3::zeros();
        dcm_po_n[(0, 0)] = prim_point[0];
        dcm_po_n[(0, 1)] = prim_point[1];
        let row_2 = prim_point.cross(&sec_point) / prim_point.cross(&sec_point).norm();
        dcm_po_n.set_row(2, &row_2.transpose());
        dcm_po_n.set_row(1, &row_2.cross(&prim_point).transpose());
        let dcm_b_po = dcm_bn * dcm_po_n.transpose();
        let bore_point_truth = dcm_b_po.transpose() * bore_vec_body;

        let tolerance = 1e-10;
        assert!(
            (bore_vec_point - bore_point_truth).abs().max() <= tolerance,
            "{bore_vec_point:?} vs {bore_point_truth:?}"
        );
        let miss_truth = bore_point_truth.x.acos();
        assert!(
            (angles.miss_angle_rad - miss_truth).abs() <= tolerance,
            "{angles:?}"
        );
        if miss_truth != PI / 2.0 {
            let azimuth_truth = bore_point_truth.z.atan2(bore_point_truth.y);
            assert!(
                (angles.azimuth_rad - azimuth_truth).abs() <= tolerance,
                "{angles:?}"
            );
        }
    }

    /// Upstream `test_bore_ang_calc`.
    #[test]
    fn celestial_angles_match_the_truth() {
        let s = 1.0 / 3.0_f64.sqrt();
        let quarter = PI / 4.0;
        let mut cases = Vec::new();
        for (x, y, z) in [
            (s, s, s),
            (-s, s, s),
            (s, -s, s),
            (-s, -s, s),
            (s, s, -s),
            (-s, s, -s),
            (s, -s, -s),
            (-s, -s, -s),
        ] {
            cases.push((Vector3::new(x, y, z), Vector3::zeros()));
        }
        for pitch in [quarter, -quarter] {
            for yaw in [quarter, 3.0 * quarter, 5.0 * quarter, -quarter] {
                cases.push((Vector3::z(), Vector3::new(yaw, pitch, 0.0)));
            }
        }
        cases.push((Vector3::x(), Vector3::zeros()));
        assert_eq!(cases.len(), 17);
        for (bore_vec_body, euler321) in cases {
            celestial_case(bore_vec_body, euler321);
        }
    }

    /// Upstream `test_bore_ang_calc_inertial_heading`.
    #[test]
    fn inertial_heading_miss_angle_matches_the_truth() {
        let h = 1.0 / 2.0_f64.sqrt();
        let tilted = Vector3::new(PI / 4.0, 0.0, -PI / 4.0);
        let cases = [
            (Vector3::x(), Vector3::zeros()),
            (Vector3::y(), Vector3::zeros()),
            (Vector3::z(), Vector3::zeros()),
            (Vector3::x(), tilted),
            (Vector3::y(), tilted),
            (Vector3::z(), tilted),
            (Vector3::new(h, -h, 0.0), tilted),
            (Vector3::new(0.0, h, h), tilted),
            (Vector3::new(h, 0.0, -h), tilted),
        ];
        for (heading, euler321) in cases {
            let sigma_bn = euler321_to_mrp(euler321);
            let state = Output::new(SpacecraftStateMsg {
                sigma_bn,
                ..Default::default()
            });
            let bore_vec_body = Vector3::x();
            let mut module = BoreAngCalc::new(BoreAngCalcConfig {
                name: "solarArrayBoresight".to_string(),
                bore_vec_body,
                inertial_heading_inertial: heading,
            });
            state.connect_to(&mut module.sc_state_in_msg);
            let (angles, _) = run_once(&mut module);
            let miss_truth = bore_vec_body.dot(&(mrp_to_dcm(sigma_bn) * heading)).acos();
            assert!(
                (angles.miss_angle_rad - miss_truth).abs() <= 1e-10,
                "{heading:?}: {angles:?} vs {miss_truth}"
            );
            assert_eq!(angles.azimuth_rad, 0.0);
        }
    }

    #[test]
    #[should_panic(expected = "sc_state_in_msg was not connected")]
    fn requires_the_spacecraft_state() {
        let mut module = BoreAngCalc::new(BoreAngCalcConfig {
            inertial_heading_inertial: Vector3::x(),
            ..Default::default()
        });
        module.init();
    }

    #[test]
    #[should_panic(expected = "either cel_body_in_msg must be connected")]
    fn requires_a_target() {
        let state = Output::<SpacecraftStateMsg>::default();
        let mut module = BoreAngCalc::new(BoreAngCalcConfig::default());
        state.connect_to(&mut module.sc_state_in_msg);
        module.init();
    }

    /// A connected celestial body takes precedence over an inertial heading.
    #[test]
    fn a_celestial_body_takes_precedence() {
        let state = Output::new(SpacecraftStateMsg {
            velocity_mps: Vector3::new(0.0, 0.0, 1.0),
            ..Default::default()
        });
        let planet = Output::new(PlanetStateMsg {
            position_inertial_m: Vector3::new(0.0, 5.0, 0.0),
            ..Default::default()
        });
        let mut module = BoreAngCalc::new(BoreAngCalcConfig {
            name: "bore".to_string(),
            bore_vec_body: Vector3::y(),
            inertial_heading_inertial: Vector3::x(),
        });
        state.connect_to(&mut module.sc_state_in_msg);
        planet.connect_to(&mut module.cel_body_in_msg);
        let (angles, _) = run_once(&mut module);
        assert!(angles.miss_angle_rad.abs() < 1e-15, "{angles:?}");
    }
}
