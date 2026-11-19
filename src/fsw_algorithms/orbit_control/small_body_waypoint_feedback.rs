//! Lyapunov waypoint feedback about a small body: port of upstream Basilisk
//! `fswAlgorithms/orbitControl/smallBodyWaypointFeedback`.
//!
//! The spacecraft state `x1` (position) and `x2` (velocity) is taken relative
//! to the small body in the body's Hill frame `O` about the Sun, and driven to
//! the waypoint `(x1_ref, x2_ref)` with the force
//!
//! ```text
//! u_O = -(f(x1, x2) - f(x1_ref, x2_ref)) - K1 (x1 - x1_ref) - K2 (x2 - x2_ref)
//! ```
//!
//! where `f` is the relative dynamics: the rotating-frame terms of the body's
//! orbit (true-anomaly rate and acceleration from its heliocentric elements),
//! the body's point-mass gravity, the solar tidal acceleration, and solar
//! radiation pressure along the Hill radial axis. The force is published in
//! spacecraft body components. The body's orbit is assumed known; the Sun's
//! gravitational parameter is upstream's hard-coded `1.327124e20` m^3/s^2, and
//! the Sun is taken at the origin of its own ephemeris message.
//!
//! Upstream's `IHubPntC_B` (spacecraft inertia) is not used by the law and is
//! not ported, nor is its C-wrapped output copy. Upstream leaves the area,
//! mass, gains, waypoint and `mu_ast` uninitialized; here they default to zero
//! and must be set (a zero mass makes the force `NaN`).

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    BodyForceCommandMsg, EphemerisMsg, Input, NavigationAttitudeMsg, NavigationTranslationMsg,
    Output,
};
use crate::utilities::astro_constants::AU;
use crate::utilities::orbital_motion::{hill_frame, state_to_elements, state_to_hill};
use crate::utilities::rigid_body_kinematics::{mrp_to_dcm, tilde};
use crate::{Module, SimulationContext};

/// Upstream `mu_sun` [m^3/s^2].
const MU_SUN_M3PS2: f64 = 1.327124e20;

/// User-settable parameters of [`SmallBodyWaypointFeedback`].
#[derive(Clone, Debug)]
pub struct SmallBodyWaypointFeedbackConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `C_SRP` [-]: SRP scaling coefficient (default 1).
    pub c_srp: f64,
    /// Upstream `P_0` [N/m^2]: solar radiation pressure at 1 AU (default
    /// 4.56e-6).
    pub p_0_npm2: f64,
    /// Upstream `rho` [-]: surface reflectivity (default 0.4).
    pub rho: f64,
    /// Upstream `A_sc` [m^2]: spacecraft area.
    pub a_sc_m2: f64,
    /// Upstream `M_sc` \[kg\]: spacecraft mass.
    pub m_sc_kg: f64,
    /// Upstream `mu_ast` [m^3/s^2]: gravitational parameter of the small body.
    pub mu_ast_m3ps2: f64,
    /// Upstream `x1_ref` \[m\]: waypoint position, Hill-frame components.
    pub x1_ref_m: Vector3<f64>,
    /// Upstream `x2_ref` [m/s]: waypoint velocity, Hill-frame components.
    pub x2_ref_mps: Vector3<f64>,
    /// Upstream `K1` [1/s^2]: position gain.
    pub k1: Matrix3<f64>,
    /// Upstream `K2` [1/s]: velocity gain.
    pub k2: Matrix3<f64>,
}

impl Default for SmallBodyWaypointFeedbackConfig {
    fn default() -> Self {
        Self {
            name: "smallBodyWaypointFeedback".to_string(),
            c_srp: 1.0,
            p_0_npm2: 4.56e-6,
            rho: 0.4,
            a_sc_m2: 0.0,
            m_sc_kg: 0.0,
            mu_ast_m3ps2: 0.0,
            x1_ref_m: Vector3::zeros(),
            x2_ref_mps: Vector3::zeros(),
            k1: Matrix3::zeros(),
            k2: Matrix3::zeros(),
        }
    }
}

/// Waypoint-to-waypoint force control about a small body: upstream
/// `SmallBodyWaypointFeedback`.
#[derive(Clone, Debug)]
pub struct SmallBodyWaypointFeedback {
    /// Module configuration.
    pub config: SmallBodyWaypointFeedbackConfig,
    /// Upstream `navTransInMsg`: spacecraft inertial position and velocity.
    pub nav_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `navAttInMsg`: spacecraft attitude.
    pub nav_att_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `asteroidEphemerisInMsg`: heliocentric state of the small
    /// body.
    pub asteroid_ephemeris_in_msg: Input<EphemerisMsg>,
    /// Upstream `sunEphemerisInMsg`: position of the Sun.
    pub sun_ephemeris_in_msg: Input<EphemerisMsg>,
    /// Upstream `forceOutMsg`: commanded force, spacecraft body components.
    pub force_out_msg: Output<BodyForceCommandMsg>,
}

impl SmallBodyWaypointFeedback {
    /// Builds the module with unconnected ports.
    pub fn new(config: SmallBodyWaypointFeedbackConfig) -> Self {
        Self {
            config,
            nav_trans_in_msg: Input::default(),
            nav_att_in_msg: Input::default(),
            asteroid_ephemeris_in_msg: Input::default(),
            sun_ephemeris_in_msg: Input::default(),
            force_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset`: the module keeps no state, so a reset only reports
    /// missing inputs.
    fn check_connections(&self) {
        for (port, connected) in [
            ("nav_trans_in_msg", self.nav_trans_in_msg.is_connected()),
            ("nav_att_in_msg", self.nav_att_in_msg.is_connected()),
            (
                "asteroid_ephemeris_in_msg",
                self.asteroid_ephemeris_in_msg.is_connected(),
            ),
            (
                "sun_ephemeris_in_msg",
                self.sun_ephemeris_in_msg.is_connected(),
            ),
        ] {
            if !connected {
                log::error!("SmallBodyWaypointFeedback.{port} is not connected");
            }
        }
    }

    /// Upstream `computeControl`: the commanded force, body components.
    fn force_b(&self) -> Vector3<f64> {
        let config = &self.config;
        let navigation = self.nav_trans_in_msg.read();
        let attitude = self.nav_att_in_msg.read();
        let asteroid = self.asteroid_ephemeris_in_msg.read();
        let sun = self.sun_ephemeris_in_msg.read();
        let (r_on_n, v_on_n) = (
            asteroid.position_bdy_zero_n_m,
            asteroid.velocity_bdy_zero_n_mps,
        );

        // True-anomaly rate and acceleration of the body's heliocentric orbit.
        let elements = state_to_elements(MU_SUN_M3PS2, r_on_n, v_on_n);
        let (e, f) = (elements.eccentricity, elements.true_anomaly_rad);
        let mean_motion_p =
            (MU_SUN_M3PS2 / (elements.semi_major_axis * (1.0 - e * e)).powf(3.0)).sqrt();
        let f_dot = mean_motion_p * (1.0 + e * f.cos()).powf(2.0);
        let f_ddot = -2.0 * e * mean_motion_p * f.sin() * (1.0 + e * f.cos()) * f_dot;

        let dcm_on = hill_frame(r_on_n, v_on_n);
        // Sun relative to the body, assuming a heliocentric frame centred on
        // the Sun rather than the solar-system barycentre.
        let r_so_o = dcm_on * (sun.position_bdy_zero_n_m - r_on_n);
        let dcm_ob = dcm_on * mrp_to_dcm(attitude.sigma_bn).transpose();
        let (x1, x2) = state_to_hill(
            r_on_n,
            v_on_n,
            navigation.position_bn_n_m,
            navigation.velocity_bn_n_mps,
        );

        let o_hat_3_tilde = tilde(Vector3::z());
        let sun_distance = r_so_o.norm();
        let sun_hat = r_so_o / sun_distance;
        let tidal = (sun_hat * sun_hat.transpose() * 3.0 - Matrix3::identity())
            * (MU_SUN_M3PS2 / sun_distance.powf(3.0));
        let srp = Vector3::x()
            * (config.c_srp
                * config.p_0_npm2
                * (1.0 + config.rho)
                * (config.a_sc_m2 / config.m_sc_kg)
                * (AU * 1000.0).powf(2.0)
                / sun_distance.powf(2.0));
        let dynamics = |x1: Vector3<f64>, x2: Vector3<f64>| {
            -o_hat_3_tilde * x1 * f_ddot
                - o_hat_3_tilde * x2 * (2.0 * f_dot)
                - o_hat_3_tilde * o_hat_3_tilde * x1 * f_dot.powf(2.0)
                - x1 * (config.mu_ast_m3ps2 / x1.norm().powf(3.0))
                + tidal * x1
                + srp
        };

        let thrust_o = -(dynamics(x1, x2) - dynamics(config.x1_ref_m, config.x2_ref_mps))
            - config.k1 * (x1 - config.x1_ref_m)
            - config.k2 * (x2 - config.x2_ref_mps);
        dcm_ob.transpose() * thrust_o
    }
}

impl Module for SmallBodyWaypointFeedback {
    fn init(&mut self) {
        self.force_out_msg.write(BodyForceCommandMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.force_out_msg.write(BodyForceCommandMsg {
            force_request_body_n: self.force_b(),
        });
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, Vector3};

    use super::{SmallBodyWaypointFeedback, SmallBodyWaypointFeedbackConfig};
    use crate::Module;
    use crate::fsw_algorithms::orbit_control::test_support::{conic_state, context, elements};
    use crate::messages::{EphemerisMsg, NavigationAttitudeMsg, NavigationTranslationMsg, Output};
    use crate::utilities::astro_constants::{AU, D2R, MU_SUN};
    use crate::utilities::orbital_motion::hill_to_state;

    /// The upstream test's module settings.
    fn config() -> SmallBodyWaypointFeedbackConfig {
        SmallBodyWaypointFeedbackConfig {
            a_sc_m2: 1.0,
            m_sc_kg: 300.0,
            mu_ast_m3ps2: 4.892,
            x1_ref_m: Vector3::new(-2000.0, 0.0, 0.0),
            x2_ref_mps: Vector3::zeros(),
            k1: Matrix3::from_diagonal_element(5e-4),
            k2: Matrix3::identity(),
            ..Default::default()
        }
    }

    /// The force for a spacecraft at `(r_bn_n, v_bn_n)` about the upstream
    /// test's asteroid, which is at `(r_on_n, v_on_n)`.
    fn force(
        r_bn_n: Vector3<f64>,
        v_bn_n: Vector3<f64>,
        r_on_n: Vector3<f64>,
        v_on_n: Vector3<f64>,
    ) -> Vector3<f64> {
        let mut module = SmallBodyWaypointFeedback::new(config());
        let navigation = Output::new(NavigationTranslationMsg {
            position_bn_n_m: r_bn_n,
            velocity_bn_n_mps: v_bn_n,
            ..Default::default()
        });
        let attitude = Output::new(NavigationAttitudeMsg {
            sigma_bn: Vector3::new(0.1, 0.0, 0.0),
            ..Default::default()
        });
        let asteroid = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: r_on_n,
            velocity_bdy_zero_n_mps: v_on_n,
            ..Default::default()
        });
        let sun = Output::new(EphemerisMsg::default());
        navigation.connect_to(&mut module.nav_trans_in_msg);
        attitude.connect_to(&mut module.nav_att_in_msg);
        asteroid.connect_to(&mut module.asteroid_ephemeris_in_msg);
        sun.connect_to(&mut module.sun_ephemeris_in_msg);
        module.init();
        module.update(&context(0));
        module.force_out_msg.read().force_request_body_n
    }

    /// The upstream test's asteroid orbit (1.1259 AU, e = 0.20373).
    fn asteroid() -> (Vector3<f64>, Vector3<f64>) {
        conic_state(
            MU_SUN * 1000.0_f64.powi(3),
            &elements(
                1.1259 * AU * 1000.0,
                0.20373,
                6.0343 * D2R,
                2.01820 * D2R,
                66.304 * D2R,
                346.32 * D2R,
            ),
        )
    }

    /// Upstream `test_smallBodyWaypointFeedback`, function 1: far from the
    /// waypoint the force exceeds 1 N.
    #[test]
    fn far_from_the_waypoint_the_force_is_large() {
        let (r_on_n, v_on_n) = asteroid();
        let r_bo_n = Vector3::new(-2000.0, 1500.0, 1000.0);
        let force = force(r_on_n + r_bo_n, v_on_n, r_on_n, v_on_n);
        assert!(force.norm() > 1.0, "{force}");
    }

    /// Upstream `test_smallBodyWaypointFeedback`, function 2: at the waypoint
    /// the force vanishes (below 1e-8 N).
    #[test]
    fn at_the_waypoint_the_force_vanishes() {
        let (r_on_n, v_on_n) = asteroid();
        let (r_bn_n, v_bn_n) = hill_to_state(
            r_on_n,
            v_on_n,
            Vector3::new(-2000.0, 0.0, 0.0),
            Vector3::zeros(),
        );
        let force = force(r_bn_n, v_bn_n, r_on_n, v_on_n);
        assert!(force.norm() < 1e-8, "{force}");
    }

    /// Not upstream: away from the waypoint the force is the gain law plus the
    /// change in the relative dynamics, written out term by term in the Hill
    /// frame (the SRP term cancels), then rotated into the body frame.
    #[test]
    fn force_is_the_gain_law_rotated_into_the_body_frame() {
        use crate::utilities::orbital_motion::{hill_frame, state_to_elements, state_to_hill};
        use crate::utilities::rigid_body_kinematics::{mrp_to_dcm, tilde};

        let (r_on_n, v_on_n) = asteroid();
        let (r_bn_n, v_bn_n) = hill_to_state(
            r_on_n,
            v_on_n,
            Vector3::new(-1900.0, 50.0, -30.0),
            Vector3::new(0.01, -0.02, 0.005),
        );
        let actual = force(r_bn_n, v_bn_n, r_on_n, v_on_n);
        // The module's relative state, roundoff of the large inertial
        // positions included.
        let (x1, x2) = state_to_hill(r_on_n, v_on_n, r_bn_n, v_bn_n);

        let mu_sun = 1.327124e20;
        let elements = state_to_elements(mu_sun, r_on_n, v_on_n);
        let (e, f) = (elements.eccentricity, elements.true_anomaly_rad);
        let p = elements.semi_major_axis * (1.0 - e * e);
        let f_dot = (mu_sun / p.powi(3)).sqrt() * (1.0 + e * f.cos()).powi(2);
        let f_ddot = -2.0 * e * (mu_sun / p.powi(3)).sqrt() * f.sin() * (1.0 + e * f.cos()) * f_dot;
        let r_so = (hill_frame(r_on_n, v_on_n) * -r_on_n).norm();
        let w = tilde(Vector3::z());
        let tidal = |x: Vector3<f64>| {
            let s = hill_frame(r_on_n, v_on_n) * -r_on_n / r_so;
            (s * s.transpose() * 3.0 - Matrix3::identity()) * x * (mu_sun / r_so.powi(3))
        };
        let dynamics = |x: Vector3<f64>, v: Vector3<f64>| {
            -w * x * f_ddot
                - w * v * (2.0 * f_dot)
                - w * w * x * f_dot * f_dot
                - x * (4.892 / x.norm().powi(3))
                + tidal(x)
        };
        let x_ref = Vector3::new(-2000.0, 0.0, 0.0);
        let thrust_o =
            -(dynamics(x1, x2) - dynamics(x_ref, Vector3::zeros())) - (x1 - x_ref) * 5e-4 - x2;
        let dcm_ob =
            hill_frame(r_on_n, v_on_n) * mrp_to_dcm(Vector3::new(0.1, 0.0, 0.0)).transpose();
        let expected = dcm_ob.transpose() * thrust_o;
        assert!(
            (actual - expected).norm() < 1e-12 * expected.norm(),
            "{actual} vs {expected}"
        );
    }
}
