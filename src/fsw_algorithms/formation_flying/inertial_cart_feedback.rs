//! Inertial Cartesian tracking control of a deputy's desired state: port of
//! upstream Basilisk `inertialCartFeedback`.
//!
//! A Lyapunov tracking law (Schaub & Junkins, *Analytical Mechanics of Space
//! Systems*, chapter 14) in inertial components. With `Δr = r - r*` and
//! `Δv = v - v*` the errors of the deputy's state from the reference, the
//! commanded force is
//!
//! ```text
//! F_N = -m a_g(r) + m a* - [K] Δr - [P] Δv,    a_g(r) = -μ r / |r|³
//! ```
//!
//! where `a*` is the reference's total inertial acceleration (natural
//! gravity plus any feedforward for non-natural motion) and `m` the deputy
//! mass. Gravity compensation is skipped when `mu` is not positive.
//!
//! As upstream, `[K]` and `[P]` act on the errors directly (they are forces
//! per metre and per metre per second, not accelerations), and the deputy
//! mass is read at reset. Upstream's C-wrapped copy of the output
//! (`forceOutMsgC`) carries the same payload and is not ported.

use std::fmt;

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    InertialForceCommandMsg, Input, NavigationTranslationMsg, Output, TranslationReferenceMsg,
    VehicleConfigMsg,
};
use crate::{Module, SimulationContext};

/// Below this deputy radius \[m\] upstream refuses to evaluate gravity.
const MIN_GRAVITY_RADIUS_M: f64 = 1e-6;

/// User-settable parameters of [`InertialCartFeedback`]. `Default` leaves
/// `k` and `p` zero, which [`Module::reset`] reports until they are set.
#[derive(Clone, Debug)]
pub struct InertialCartFeedbackConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `mu` \[m^3/s^2\]: gravitational parameter for the gravity
    /// compensation, skipped unless positive; must not be negative. Default
    /// 0 (upstream's unset value is -1, which also skips the compensation).
    pub mu_m3ps2: f64,
    /// Upstream `K` \[N/m\]: symmetric positive definite position gain.
    /// Upstream's `setK` takes it row-major as 9 values; build it from those
    /// with [`Matrix3::from_row_slice`].
    pub k: Matrix3<f64>,
    /// Upstream `P` \[N s/m\]: symmetric positive definite velocity gain,
    /// row-major like `k`.
    pub p: Matrix3<f64>,
}

impl Default for InertialCartFeedbackConfig {
    fn default() -> Self {
        Self {
            name: "inertialCartFeedback".to_string(),
            mu_m3ps2: 0.0,
            k: Matrix3::zeros(),
            p: Matrix3::zeros(),
        }
    }
}

/// A configuration problem upstream's `Reset` or setters report with
/// `bskError`; each is logged at reset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ConfigError {
    InputNotConnected(&'static str),
    NonPositiveMass,
    NegativeMu,
    GainNotPositiveDefinite(&'static str),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InputNotConnected(port) => {
                write!(f, "inertialCartFeedback.{port} is not connected")
            }
            Self::NonPositiveMass => write!(f, "inertialCartFeedback deputy mass must be positive"),
            Self::NegativeMu => write!(f, "inertialCartFeedback: mu must be non-negative"),
            Self::GainNotPositiveDefinite(gain) => write!(
                f,
                "inertialCartFeedback: {gain} must be symmetric positive definite"
            ),
        }
    }
}

/// Why an update publishes nothing: upstream raises a `BasiliskError` when
/// the deputy is too close to the centre to evaluate gravity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct DeputyAtCentre;

impl fmt::Display for DeputyAtCentre {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "inertialCartFeedback deputy position norm too small to evaluate gravity"
        )
    }
}

/// Inertial Cartesian feedback controller: upstream `InertialCartFeedback`.
#[derive(Clone, Debug)]
pub struct InertialCartFeedback {
    /// Module configuration.
    pub config: InertialCartFeedbackConfig,
    /// Upstream `deputyNavInMsg`: deputy inertial position and velocity.
    pub deputy_nav_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `deputyRefInMsg`: desired deputy position, velocity and total
    /// inertial acceleration.
    pub deputy_ref_in_msg: Input<TranslationReferenceMsg>,
    /// Upstream `deputyVehicleConfigInMsg`: deputy mass, read at reset.
    pub deputy_vehicle_config_in_msg: Input<VehicleConfigMsg>,
    /// Upstream `forceOutMsg`: deputy force command, inertial components.
    pub force_out_msg: Output<InertialForceCommandMsg>,
    /// Upstream `deputyMass` \[kg\], read at reset.
    deputy_mass_kg: f64,
}

impl InertialCartFeedback {
    /// Builds the module with unconnected ports.
    pub fn new(config: InertialCartFeedbackConfig) -> Self {
        Self {
            config,
            deputy_nav_in_msg: Input::default(),
            deputy_ref_in_msg: Input::default(),
            deputy_vehicle_config_in_msg: Input::default(),
            force_out_msg: Output::default(),
            deputy_mass_kg: 0.0,
        }
    }

    /// Upstream `Reset` (and the validation of upstream's `setMu`, `setK`
    /// and `setP`, which here run on the configuration at reset): reads the
    /// deputy mass and returns every configuration problem.
    fn configure(&mut self) -> Vec<ConfigError> {
        let mut errors = Vec::new();
        for (connected, port) in [
            (self.deputy_nav_in_msg.is_connected(), "deputy_nav_in_msg"),
            (self.deputy_ref_in_msg.is_connected(), "deputy_ref_in_msg"),
            (
                self.deputy_vehicle_config_in_msg.is_connected(),
                "deputy_vehicle_config_in_msg",
            ),
        ] {
            if !connected {
                errors.push(ConfigError::InputNotConnected(port));
            }
        }
        self.deputy_mass_kg = self.deputy_vehicle_config_in_msg.read().mass_kg;
        if self.deputy_mass_kg <= 0.0 {
            errors.push(ConfigError::NonPositiveMass);
        }
        if self.config.mu_m3ps2 < 0.0 {
            errors.push(ConfigError::NegativeMu);
        }
        for (gain, name) in [(&self.config.k, "k"), (&self.config.p, "p")] {
            if !is_symmetric_positive_definite(gain) {
                errors.push(ConfigError::GainNotPositiveDefinite(name));
            }
        }
        errors
    }

    fn reset_module(&mut self) {
        for error in self.configure() {
            log::error!("{error}");
        }
    }

    /// Upstream `UpdateState`: the deputy force command \[N\], inertial
    /// components.
    fn force_command(&self) -> Result<Vector3<f64>, DeputyAtCentre> {
        let config = &self.config;
        let deputy = self.deputy_nav_in_msg.read();
        let reference = self.deputy_ref_in_msg.read();
        let r_bn_n = deputy.position_bn_n_m;
        let delta_r_n = r_bn_n - reference.position_m;
        let delta_v_n = deputy.velocity_bn_n_mps - reference.velocity_mps;

        let mut a_grav_n = Vector3::zeros();
        if config.mu_m3ps2 > 0.0 {
            let radius = r_bn_n.norm();
            if radius > MIN_GRAVITY_RADIUS_M {
                a_grav_n = -config.mu_m3ps2 * r_bn_n / (radius * radius * radius);
            } else {
                return Err(DeputyAtCentre);
            }
        }
        let mass_kg = self.deputy_mass_kg;
        Ok(-mass_kg * a_grav_n + mass_kg * reference.acceleration_mps2
            - config.k * delta_r_n
            - config.p * delta_v_n)
    }
}

/// Upstream's `setK`/`setP` check: `[G] - [G]^T` has a Frobenius norm of at
/// most 1e-12, and the smallest eigenvalue of the matrix (its lower triangle,
/// as Eigen's `selfadjointView<Lower>` reads it) is positive.
fn is_symmetric_positive_definite(gain: &Matrix3<f64>) -> bool {
    let min_eigenvalue = gain.symmetric_eigenvalues().min();
    (gain - gain.transpose()).norm() <= 1e-12 && min_eigenvalue > 0.0
}

impl Module for InertialCartFeedback {
    fn init(&mut self) {
        self.force_out_msg.write(InertialForceCommandMsg::default());
        self.reset_module();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_module();
    }

    fn update(&mut self, _context: &SimulationContext) {
        match self.force_command() {
            Ok(force_request_inertial_n) => self.force_out_msg.write(InertialForceCommandMsg {
                force_request_inertial_n,
            }),
            // Upstream throws before writing: the output keeps its previous
            // value.
            Err(error) => log::error!("{error}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{ConfigError, DeputyAtCentre, InertialCartFeedback, InertialCartFeedbackConfig};
    use crate::messages::{
        NavigationTranslationMsg, Output, TranslationReferenceMsg, VehicleConfigMsg,
    };
    use crate::{Module, SimulationContext};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The pytest's `gravAccel`.
    fn grav_accel(mu: f64, r: Vector3<f64>) -> Vector3<f64> {
        if mu > 0.0 {
            assert!(
                r.norm() > 1e-6,
                "position norm too small to evaluate gravity"
            );
            -mu * r / r.norm().powi(3)
        } else {
            Vector3::zeros()
        }
    }

    /// The pytest's `computeTruthForce`.
    #[allow(clippy::too_many_arguments)]
    fn upstream_truth(
        mu: f64,
        deputy_mass: f64,
        k: Matrix3<f64>,
        p: Matrix3<f64>,
        r_deputy: Vector3<f64>,
        v_deputy: Vector3<f64>,
        r_deputy_ref: Vector3<f64>,
        v_deputy_ref: Vector3<f64>,
        a_deputy_ref: Vector3<f64>,
    ) -> Vector3<f64> {
        let delta_r = r_deputy - r_deputy_ref;
        let delta_v = v_deputy - v_deputy_ref;
        -deputy_mass * grav_accel(mu, r_deputy) + deputy_mass * a_deputy_ref
            - k * delta_r
            - p * delta_v
    }

    fn upstream_gains() -> (Matrix3<f64>, Matrix3<f64>) {
        (
            Matrix3::from_row_slice(&[2.0e-5, 0.0, 0.0, 0.0, 3.0e-5, 0.0, 0.0, 0.0, 4.0e-5]),
            Matrix3::from_row_slice(&[5.0e-2, 0.0, 0.0, 0.0, 6.0e-2, 0.0, 0.0, 0.0, 7.0e-2]),
        )
    }

    /// Upstream `test_inertialCartFeedback`, all four `(useMu,
    /// nonNaturalMotion)` parametrizations: one update at t = 0, the force
    /// checked against the truth with the absolute accuracy 1e-10. (The
    /// pytest's `getK`/`getP` round trip is the public `config` here.)
    #[test]
    fn matches_upstream_inertial_cart_feedback_unit_test() {
        for use_mu in [true, false] {
            for non_natural_motion in [true, false] {
                let mu = if use_mu { 3.986004418e14 } else { 0.0 };
                let (k, p) = upstream_gains();
                let mut module = InertialCartFeedback::new(InertialCartFeedbackConfig {
                    mu_m3ps2: mu,
                    k,
                    p,
                    ..Default::default()
                });

                let deputy = NavigationTranslationMsg {
                    position_bn_n_m: Vector3::new(7000e3 + 40.0, 100e3 - 15.0, -50e3 + 30.0),
                    velocity_bn_n_mps: Vector3::new(0.05, 7500.08, 49.92),
                    ..Default::default()
                };
                let r_ref = Vector3::new(7000e3 + 10.0, 100e3 - 20.0, -50e3 + 15.0);
                let a_grav_ref = grav_accel(mu, r_ref);
                let reference = TranslationReferenceMsg {
                    position_m: r_ref,
                    velocity_mps: Vector3::new(0.03, 7500.0 - 0.01, 50.0 + 0.02),
                    acceleration_mps2: if non_natural_motion {
                        a_grav_ref + Vector3::new(0.001, -0.002, 0.003)
                    } else {
                        a_grav_ref
                    },
                };
                let deputy_mass = 300.0;
                let deputy_out = Output::new(deputy.clone());
                let reference_out = Output::new(reference.clone());
                let config_out = Output::new(VehicleConfigMsg {
                    mass_kg: deputy_mass,
                    ..Default::default()
                });
                deputy_out.connect_to(&mut module.deputy_nav_in_msg);
                reference_out.connect_to(&mut module.deputy_ref_in_msg);
                config_out.connect_to(&mut module.deputy_vehicle_config_in_msg);
                module.init();
                assert_eq!(module.configure(), []);
                module.update(&context(0));

                let force = module.force_out_msg.read().force_request_inertial_n;
                let expected = upstream_truth(
                    mu,
                    deputy_mass,
                    k,
                    p,
                    deputy.position_bn_n_m,
                    deputy.velocity_bn_n_mps,
                    reference.position_m,
                    reference.velocity_mps,
                    reference.acceleration_mps2,
                );
                for (index, (value, truth)) in force.iter().zip(expected.iter()).enumerate() {
                    assert!(
                        (value - truth).abs() <= 1e-10,
                        "use_mu {use_mu}, non-natural {non_natural_motion}, component {index}: \
                         {value:e} vs {truth:e}"
                    );
                }
            }
        }
    }

    #[test]
    fn reset_reports_missing_inputs_mass_mu_and_gains() {
        let mut module = InertialCartFeedback::new(InertialCartFeedbackConfig {
            mu_m3ps2: -1.0,
            ..Default::default()
        });
        assert_eq!(
            module.configure(),
            [
                ConfigError::InputNotConnected("deputy_nav_in_msg"),
                ConfigError::InputNotConnected("deputy_ref_in_msg"),
                ConfigError::InputNotConnected("deputy_vehicle_config_in_msg"),
                ConfigError::NonPositiveMass,
                ConfigError::NegativeMu,
                ConfigError::GainNotPositiveDefinite("k"),
                ConfigError::GainNotPositiveDefinite("p"),
            ]
        );
        // Asymmetric, and symmetric but indefinite.
        let (k, p) = upstream_gains();
        module.config = InertialCartFeedbackConfig {
            k: Matrix3::from_row_slice(&[1.0, 1e-9, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
            p: -p,
            ..Default::default()
        };
        let errors = module.configure();
        assert!(errors.contains(&ConfigError::GainNotPositiveDefinite("k")));
        assert!(errors.contains(&ConfigError::GainNotPositiveDefinite("p")));
        module.config.k = k;
        module.config.p = p;
        assert!(
            !module
                .configure()
                .iter()
                .any(|error| matches!(error, ConfigError::GainNotPositiveDefinite(_)))
        );
    }

    #[test]
    fn the_deputy_mass_is_read_at_reset() {
        let (k, p) = upstream_gains();
        let mut module = InertialCartFeedback::new(InertialCartFeedbackConfig {
            k,
            p,
            ..Default::default()
        });
        let deputy_out = Output::new(NavigationTranslationMsg::default());
        let reference_out = Output::new(TranslationReferenceMsg {
            acceleration_mps2: Vector3::new(1.0, 0.0, 0.0),
            ..Default::default()
        });
        let config_out = Output::new(VehicleConfigMsg {
            mass_kg: 2.0,
            ..Default::default()
        });
        deputy_out.connect_to(&mut module.deputy_nav_in_msg);
        reference_out.connect_to(&mut module.deputy_ref_in_msg);
        config_out.connect_to(&mut module.deputy_vehicle_config_in_msg);
        module.init();
        config_out.write(VehicleConfigMsg {
            mass_kg: 5.0,
            ..Default::default()
        });
        module.update(&context(0));
        assert_eq!(
            module.force_out_msg.read().force_request_inertial_n,
            Vector3::new(2.0, 0.0, 0.0)
        );
        module.reset(&context(0));
        module.update(&context(0));
        assert_eq!(
            module.force_out_msg.read().force_request_inertial_n,
            Vector3::new(5.0, 0.0, 0.0)
        );
    }

    #[test]
    fn gravity_at_the_centre_stops_the_update() {
        let (k, p) = upstream_gains();
        let mut module = InertialCartFeedback::new(InertialCartFeedbackConfig {
            mu_m3ps2: 3.986004418e14,
            k,
            p,
            ..Default::default()
        });
        let deputy_out = Output::new(NavigationTranslationMsg::default());
        let reference_out = Output::new(TranslationReferenceMsg::default());
        deputy_out.connect_to(&mut module.deputy_nav_in_msg);
        reference_out.connect_to(&mut module.deputy_ref_in_msg);
        assert_eq!(module.force_command(), Err(DeputyAtCentre));
        // Without gravity compensation the same state is fine.
        module.config.mu_m3ps2 = 0.0;
        assert_eq!(module.force_command(), Ok(Vector3::zeros()));
    }
}
