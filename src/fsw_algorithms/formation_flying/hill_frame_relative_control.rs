//! Hill-frame relative position and velocity tracking control: port of
//! upstream Basilisk `hillFrameRelativeControl`.
//!
//! The deputy's position `x` and velocity `x'` relative to the chief, in
//! chief Hill-frame components (as seen in the rotating frame), track the
//! references `x_r`, `x'_r` with the commanded acceleration
//!
//! ```text
//! a_H = -[K] (x - x_r) - [P] (x' - x'_r) - ([A1] x + [A2] x')
//! ```
//!
//! where the feedforward `[A1] x + [A2] x'` cancels the nonlinear relative
//! motion terms of a chief on a general (non-circular) orbit:
//!
//! ```text
//! [A1] x = ((2μ/r³ + θ̇²) x + θ̈ y,  -θ̈ x + (θ̇² - μ/r³) y,  -μ/r³ z)
//! [A2] x' = (2θ̇ y',  -2θ̇ x',  0)
//! ```
//!
//! with `θ̇ = |r × v| / r²`, `ṙ = r·v / r` and `θ̈ = -2 ṙ θ̇ / r` from the chief
//! state. The inertial force command is `F_N = m_d [HN]^T a_H`, `m_d` the
//! deputy mass.
//!
//! The relative state comes either from a Hill-state input (for example
//! [`super::hill_state_converter`]) or is computed here from the deputy's
//! inertial navigation state; exactly one of the two inputs must be
//! connected. Upstream's C-wrapped copy of the output (`forceOutMsgC`)
//! carries the same payload and is not ported.

use std::fmt;

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    HillRelativeStateMsg, InertialForceCommandMsg, Input, NavigationTranslationMsg, Output,
    VehicleConfigMsg,
};
use crate::utilities::linear_algebra::{is_equal_within, m33_eigen_values};
use crate::utilities::orbital_motion::{hill_frame, state_to_hill};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`HillFrameRelativeControl`]. `Default` is
/// upstream's zero-initialized configuration, which [`Module::reset`]
/// reports as incomplete until `mu_m3ps2`, `k` and `p` are set.
#[derive(Clone, Debug)]
pub struct HillFrameRelativeControlConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `mu` \[m^3/s^2\]: gravitational parameter of the central body
    /// used by the feedforward; must be positive.
    pub mu_m3ps2: f64,
    /// Upstream `K` \[1/s^2\]: symmetric positive definite position gain, Hill
    /// frame. Upstream's `setK` takes it row-major as 9 values; build it from
    /// those with [`Matrix3::from_row_slice`].
    pub k: Matrix3<f64>,
    /// Upstream `P` \[1/s\]: symmetric positive definite velocity gain, Hill
    /// frame, row-major like `k`.
    pub p: Matrix3<f64>,
    /// Upstream `rRef_H` \[m\]: reference relative position, Hill components.
    pub r_ref_h_m: Vector3<f64>,
    /// Upstream `vRef_H` \[m/s\]: reference relative velocity as seen in the
    /// Hill frame, Hill components.
    pub v_ref_h_mps: Vector3<f64>,
}

impl Default for HillFrameRelativeControlConfig {
    fn default() -> Self {
        Self {
            name: "hillFrameRelativeControl".to_string(),
            mu_m3ps2: 0.0,
            k: Matrix3::zeros(),
            p: Matrix3::zeros(),
            r_ref_h_m: Vector3::zeros(),
            v_ref_h_mps: Vector3::zeros(),
        }
    }
}

/// A configuration problem upstream's `Reset` or setters report with
/// `bskError`; each is logged at reset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ConfigError {
    InputNotConnected(&'static str),
    /// Both or neither of `hill_state_in_msg` and `deputy_trans_in_msg` are
    /// connected.
    RelativeStateSource,
    NonPositiveMass,
    NonPositiveMu,
    GainNotPositiveDefinite(&'static str),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InputNotConnected(port) => {
                write!(f, "hillFrameRelativeControl.{port} is not connected")
            }
            Self::RelativeStateSource => write!(
                f,
                "hillFrameRelativeControl requires exactly one of hill_state_in_msg or \
                 deputy_trans_in_msg to be connected"
            ),
            Self::NonPositiveMass => write!(
                f,
                "hillFrameRelativeControl.deputy_vehicle_config_in_msg mass must be positive"
            ),
            Self::NonPositiveMu => write!(f, "hillFrameRelativeControl.mu must be positive"),
            Self::GainNotPositiveDefinite(gain) => write!(
                f,
                "hillFrameRelativeControl: {gain} must be symmetric positive definite"
            ),
        }
    }
}

/// Why an update publishes nothing; upstream raises a `BasiliskError` before
/// writing the output.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum UpdateError {
    NonPositiveMu,
    ZeroChiefRadius,
    ZeroChiefAngularMomentum,
}

impl fmt::Display for UpdateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonPositiveMu => write!(f, "hillFrameRelativeControl.mu must be positive"),
            Self::ZeroChiefRadius => {
                write!(f, "hillFrameRelativeControl chief radius norm is zero")
            }
            Self::ZeroChiefAngularMomentum => write!(
                f,
                "hillFrameRelativeControl chief angular momentum norm is zero"
            ),
        }
    }
}

/// Inertial force command from Hill-frame relative-state errors: upstream
/// `HillFrameRelativeControl`.
#[derive(Clone, Debug)]
pub struct HillFrameRelativeControl {
    /// Module configuration.
    pub config: HillFrameRelativeControlConfig,
    /// Upstream `chiefTransInMsg`: chief inertial state, which defines the
    /// Hill frame.
    pub chief_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `hillStateInMsg`: deputy state relative to the chief, Hill
    /// components. Connect this or `deputy_trans_in_msg`, not both.
    pub hill_state_in_msg: Input<HillRelativeStateMsg>,
    /// Upstream `deputyTransInMsg`: deputy inertial state, used when
    /// `hill_state_in_msg` is not connected.
    pub deputy_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `deputyVehicleConfigInMsg`: deputy mass.
    pub deputy_vehicle_config_in_msg: Input<VehicleConfigMsg>,
    /// Upstream `forceOutMsg`: deputy force command, inertial components.
    pub force_out_msg: Output<InertialForceCommandMsg>,
}

impl HillFrameRelativeControl {
    /// Builds the module with unconnected ports.
    pub fn new(config: HillFrameRelativeControlConfig) -> Self {
        Self {
            config,
            chief_trans_in_msg: Input::default(),
            hill_state_in_msg: Input::default(),
            deputy_trans_in_msg: Input::default(),
            deputy_vehicle_config_in_msg: Input::default(),
            force_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset` (and the validation of upstream's `setK`, `setP` and
    /// `setMu`, which here run on the configuration at reset).
    fn configuration_errors(&self) -> Vec<ConfigError> {
        let mut errors = Vec::new();
        if !self.chief_trans_in_msg.is_connected() {
            errors.push(ConfigError::InputNotConnected("chief_trans_in_msg"));
        }
        if self.hill_state_in_msg.is_connected() == self.deputy_trans_in_msg.is_connected() {
            errors.push(ConfigError::RelativeStateSource);
        }
        if !self.deputy_vehicle_config_in_msg.is_connected() {
            errors.push(ConfigError::InputNotConnected(
                "deputy_vehicle_config_in_msg",
            ));
        } else if self.deputy_vehicle_config_in_msg.read().mass_kg <= 0.0 {
            errors.push(ConfigError::NonPositiveMass);
        }
        if self.config.mu_m3ps2 <= 0.0 {
            errors.push(ConfigError::NonPositiveMu);
        }
        if !is_symmetric_positive_definite(&self.config.k) {
            errors.push(ConfigError::GainNotPositiveDefinite("k"));
        }
        if !is_symmetric_positive_definite(&self.config.p) {
            errors.push(ConfigError::GainNotPositiveDefinite("p"));
        }
        errors
    }

    fn check_configuration(&self) {
        for error in self.configuration_errors() {
            log::error!("{error}");
        }
    }

    /// Upstream `computeForceCommand`: the deputy force command \[N\],
    /// inertial components.
    fn force_command(&self) -> Result<Vector3<f64>, UpdateError> {
        let config = &self.config;
        if config.mu_m3ps2 <= 0.0 {
            return Err(UpdateError::NonPositiveMu);
        }
        let chief = self.chief_trans_in_msg.read();
        let mass_kg = self.deputy_vehicle_config_in_msg.read().mass_kg;
        let r_chief_n = chief.position_bn_n_m;
        let v_chief_n = chief.velocity_bn_n_mps;

        let r_chief = r_chief_n.norm();
        if r_chief <= 0.0 {
            return Err(UpdateError::ZeroChiefRadius);
        }
        let h_chief = r_chief_n.cross(&v_chief_n).norm();
        if h_chief <= 0.0 {
            return Err(UpdateError::ZeroChiefAngularMomentum);
        }
        let dcm_hn = hill_frame(r_chief_n, v_chief_n);

        let (r_rel_h, v_rel_h) = if self.hill_state_in_msg.is_connected() {
            let hill = self.hill_state_in_msg.read();
            (hill.position_dc_h_m, hill.velocity_dc_h_mps)
        } else {
            let deputy = self.deputy_trans_in_msg.read();
            state_to_hill(
                r_chief_n,
                v_chief_n,
                deputy.position_bn_n_m,
                deputy.velocity_bn_n_mps,
            )
        };

        // Chief true-latitude rate and acceleration, radial rate and mu/r^3.
        let theta_dot = h_chief / (r_chief * r_chief);
        let r_dot = r_chief_n.dot(&v_chief_n) / r_chief;
        let theta_ddot = -2.0 * r_dot * theta_dot / r_chief;
        let mu_over_r3 = config.mu_m3ps2 / (r_chief * r_chief * r_chief);

        let ff_a1 = Vector3::new(
            (2.0 * mu_over_r3 + theta_dot * theta_dot) * r_rel_h.x + theta_ddot * r_rel_h.y,
            -theta_ddot * r_rel_h.x + (theta_dot * theta_dot - mu_over_r3) * r_rel_h.y,
            -mu_over_r3 * r_rel_h.z,
        );
        let ff_a2 = Vector3::new(
            2.0 * theta_dot * v_rel_h.y,
            -2.0 * theta_dot * v_rel_h.x,
            0.0,
        );

        let k_term = config.k * (r_rel_h - config.r_ref_h_m);
        let p_term = config.p * (v_rel_h - config.v_ref_h_mps);
        let a_cmd_h = -k_term - p_term - (ff_a1 + ff_a2);
        Ok(mass_kg * (dcm_hn.transpose() * a_cmd_h))
    }
}

/// Upstream's `setK`/`setP` check: symmetric to 1e-12 in every element
/// (`m33IsEqual`) and every eigenvalue (`m33EigenValues`) above 1e-12.
fn is_symmetric_positive_definite(gain: &Matrix3<f64>) -> bool {
    is_equal_within(gain, &gain.transpose(), 1e-12)
        && m33_eigen_values(gain).iter().all(|&value| value > 1e-12)
}

impl Module for HillFrameRelativeControl {
    fn init(&mut self) {
        self.force_out_msg.write(InertialForceCommandMsg::default());
        self.check_configuration();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_configuration();
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

    use super::{
        ConfigError, HillFrameRelativeControl, HillFrameRelativeControlConfig, UpdateError,
    };
    use crate::fsw_algorithms::formation_flying::hill_state_converter::{
        HillStateConverter, HillStateConverterConfig,
    };
    use crate::messages::{NavigationTranslationMsg, Output, VehicleConfigMsg};
    use crate::{Module, SimulationContext};

    const MU: f64 = 3.986004418e14;
    /// Upstream test task rate, `macros.sec2nano(0.1)`.
    const TASK_RATE_NANOS: u64 = 100_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn navigation(position: [f64; 3], velocity: [f64; 3]) -> NavigationTranslationMsg {
        NavigationTranslationMsg {
            position_bn_n_m: Vector3::from(position),
            velocity_bn_n_mps: Vector3::from(velocity),
            ..Default::default()
        }
    }

    fn upstream_config() -> HillFrameRelativeControlConfig {
        HillFrameRelativeControlConfig {
            mu_m3ps2: MU,
            k: Matrix3::from_row_slice(&[2.0e-6, 0.0, 0.0, 0.0, 3.0e-6, 0.0, 0.0, 0.0, 4.0e-6]),
            p: Matrix3::from_row_slice(&[1.5e-3, 0.0, 0.0, 0.0, 1.0e-3, 0.0, 0.0, 0.0, 2.5e-3]),
            r_ref_h_m: Vector3::new(100.0, -50.0, 20.0),
            v_ref_h_mps: Vector3::new(0.01, -0.02, 0.005),
            ..Default::default()
        }
    }

    /// The pytest's `_compute_truth_force`, which rebuilds the Hill frame and
    /// the relative state with NumPy.
    #[allow(clippy::too_many_arguments)]
    fn upstream_truth(
        chief_r: Vector3<f64>,
        chief_v: Vector3<f64>,
        deputy_r: Vector3<f64>,
        deputy_v: Vector3<f64>,
        mass: f64,
        k_gain: Matrix3<f64>,
        p_gain: Matrix3<f64>,
        r_ref: Vector3<f64>,
        v_ref: Vector3<f64>,
        mu: f64,
    ) -> Vector3<f64> {
        let r_norm = chief_r.norm();
        let h_vec = chief_r.cross(&chief_v);
        let h_norm = h_vec.norm();
        let i_r = chief_r / r_norm;
        let i_h = h_vec / h_norm;
        let i_t = i_h.cross(&i_r);
        let dcm_hn = Matrix3::from_rows(&[i_r.transpose(), i_t.transpose(), i_h.transpose()]);

        let r_rel_h = dcm_hn * (deputy_r - chief_r);
        let omega_hn_h = Vector3::new(0.0, 0.0, h_norm / (r_norm * r_norm));
        let v_rel_h = dcm_hn * (deputy_v - chief_v) - omega_hn_h.cross(&r_rel_h);

        let r_dot = chief_r.dot(&chief_v) / r_norm;
        let theta_dot = h_norm / (r_norm * r_norm);
        let theta_ddot = -2.0 * r_dot * theta_dot / r_norm;
        let mu_over_r3 = mu / r_norm.powi(3);

        let ff_a1 = Vector3::new(
            (2.0 * mu_over_r3 + theta_dot.powi(2)) * r_rel_h[0] + theta_ddot * r_rel_h[1],
            -theta_ddot * r_rel_h[0] + (theta_dot.powi(2) - mu_over_r3) * r_rel_h[1],
            -mu_over_r3 * r_rel_h[2],
        );
        let ff_a2 = Vector3::new(
            2.0 * theta_dot * v_rel_h[1],
            -2.0 * theta_dot * v_rel_h[0],
            0.0,
        );
        let a_cmd_h = -k_gain * (r_rel_h - r_ref) - p_gain * (v_rel_h - v_ref) - (ff_a1 + ff_a2);
        mass * (dcm_hn.transpose() * a_cmd_h)
    }

    /// Upstream `test_hillFrameRelativeControl`, both `use_hill_state`
    /// parametrizations: the deputy state comes from `hillStateConverter` or
    /// straight from the deputy navigation input. A 0.1 s task runs to 0.1 s
    /// and the last force is checked against the truth with the absolute
    /// accuracy 1e-10 (upstream also checks the identical C-wrapped output).
    #[test]
    fn matches_upstream_hill_frame_relative_control_unit_test() {
        let accuracy = 1e-10;
        let chief_r = [7000e3, 200e3, -100e3];
        let chief_v = [50.0, 7500.0, 100.0];
        let deputy_r = [7000e3 + 120.0, 200e3 - 80.0, -100e3 + 30.0];
        let deputy_v = [50.2, 7499.8, 100.05];
        let mass = 420.0;
        let chief_out = Output::new(navigation(chief_r, chief_v));
        let deputy_out = Output::new(navigation(deputy_r, deputy_v));
        let config_out = Output::new(VehicleConfigMsg {
            mass_kg: mass,
            ..Default::default()
        });
        let config = upstream_config();
        let expected = upstream_truth(
            Vector3::from(chief_r),
            Vector3::from(chief_v),
            Vector3::from(deputy_r),
            Vector3::from(deputy_v),
            mass,
            config.k,
            config.p,
            config.r_ref_h_m,
            config.v_ref_h_mps,
            MU,
        );

        for use_hill_state in [false, true] {
            let mut module = HillFrameRelativeControl::new(config.clone());
            chief_out.connect_to(&mut module.chief_trans_in_msg);
            config_out.connect_to(&mut module.deputy_vehicle_config_in_msg);
            let mut converter = HillStateConverter::new(HillStateConverterConfig::default());
            if use_hill_state {
                chief_out.connect_to(&mut converter.chief_state_in_msg);
                deputy_out.connect_to(&mut converter.dep_state_in_msg);
                converter
                    .hill_state_out_msg
                    .connect_to(&mut module.hill_state_in_msg);
            } else {
                deputy_out.connect_to(&mut module.deputy_trans_in_msg);
            }
            converter.init();
            module.init();
            assert_eq!(module.configuration_errors(), []);
            for nanos in [0, TASK_RATE_NANOS] {
                // The converter is added to the task first, as upstream.
                if use_hill_state {
                    converter.update(&context(nanos));
                }
                module.update(&context(nanos));
            }

            let force = module.force_out_msg.read().force_request_inertial_n;
            for (index, (value, truth)) in force.iter().zip(expected.iter()).enumerate() {
                assert!(
                    (value - truth).abs() <= accuracy,
                    "use_hill_state {use_hill_state}, component {index}: {value:e} vs {truth:e}"
                );
            }
        }
    }

    /// Not upstream: a deputy at the chief with a zero reference needs no
    /// force.
    #[test]
    fn a_deputy_on_the_chief_with_zero_reference_needs_no_force() {
        let chief_out = Output::new(navigation([7000e3, 0.0, 0.0], [0.0, 7546.0, 0.0]));
        let config_out = Output::new(VehicleConfigMsg {
            mass_kg: 100.0,
            ..Default::default()
        });
        let mut module = HillFrameRelativeControl::new(HillFrameRelativeControlConfig {
            r_ref_h_m: Vector3::zeros(),
            v_ref_h_mps: Vector3::zeros(),
            ..upstream_config()
        });
        chief_out.connect_to(&mut module.chief_trans_in_msg);
        chief_out.connect_to(&mut module.deputy_trans_in_msg);
        config_out.connect_to(&mut module.deputy_vehicle_config_in_msg);
        module.init();
        module.update(&context(0));
        assert_eq!(
            module.force_out_msg.read().force_request_inertial_n,
            Vector3::zeros()
        );
    }

    #[test]
    fn reset_reports_missing_inputs_and_invalid_gains() {
        let mut module = HillFrameRelativeControl::new(HillFrameRelativeControlConfig::default());
        assert_eq!(
            module.configuration_errors(),
            [
                ConfigError::InputNotConnected("chief_trans_in_msg"),
                ConfigError::RelativeStateSource,
                ConfigError::InputNotConnected("deputy_vehicle_config_in_msg"),
                ConfigError::NonPositiveMu,
                ConfigError::GainNotPositiveDefinite("k"),
                ConfigError::GainNotPositiveDefinite("p"),
            ]
        );

        // Both relative-state sources, a massless deputy and an asymmetric
        // gain.
        let chief_out = Output::new(navigation([7000e3, 0.0, 0.0], [0.0, 7546.0, 0.0]));
        let config_out = Output::new(VehicleConfigMsg::default());
        let hill_out = Output::new(Default::default());
        chief_out.connect_to(&mut module.chief_trans_in_msg);
        chief_out.connect_to(&mut module.deputy_trans_in_msg);
        hill_out.connect_to(&mut module.hill_state_in_msg);
        config_out.connect_to(&mut module.deputy_vehicle_config_in_msg);
        module.config = upstream_config();
        module.config.k[(0, 1)] = 1e-6;
        assert_eq!(
            module.configuration_errors(),
            [
                ConfigError::RelativeStateSource,
                ConfigError::NonPositiveMass,
                ConfigError::GainNotPositiveDefinite("k"),
            ]
        );
    }

    #[test]
    fn a_degenerate_chief_stops_the_update() {
        let chief_out = Output::new(navigation([7000e3, 0.0, 0.0], [7546.0, 0.0, 0.0]));
        let deputy_out = Output::new(navigation([7000e3, 10.0, 0.0], [7546.0, 0.0, 0.0]));
        let config_out = Output::new(VehicleConfigMsg {
            mass_kg: 100.0,
            ..Default::default()
        });
        let mut module = HillFrameRelativeControl::new(upstream_config());
        chief_out.connect_to(&mut module.chief_trans_in_msg);
        deputy_out.connect_to(&mut module.deputy_trans_in_msg);
        config_out.connect_to(&mut module.deputy_vehicle_config_in_msg);
        module.init();
        // Radial chief velocity: no angular momentum.
        assert_eq!(
            module.force_command(),
            Err(UpdateError::ZeroChiefAngularMomentum)
        );
        chief_out.write(navigation([0.0; 3], [0.0, 7546.0, 0.0]));
        assert_eq!(module.force_command(), Err(UpdateError::ZeroChiefRadius));
        module.config.mu_m3ps2 = 0.0;
        assert_eq!(module.force_command(), Err(UpdateError::NonPositiveMu));
        module.update(&context(0));
        assert_eq!(
            module.force_out_msg.read().force_request_inertial_n,
            Vector3::zeros()
        );
    }
}
