//! Holonomic constraint between two spacecraft: port of upstream Basilisk
//! `simulation/dynamics/constraintEffector` (`ConstraintDynamicEffector`).
//!
//! The constraint holds connection point P2 of spacecraft 2 at the prescribed
//! offset `r_P2P1_B1Init` from connection point P1 of spacecraft 1, and the
//! attitude of B2 relative to B1 at `sigma_B2B1Init`. It does so with
//! Baumgarte stabilization (Vaz Carneiro, Morell and Schaub, "Post-Docking
//! Complex Spacecraft Dynamics Using Baumgarte Stabilization", AAS/AIAA 2023).
//! With the direction violation `ψ` (P2's offset from its prescribed place)
//! and its rate `ψ'` as seen from B1, the force on spacecraft 1 is
//! `Fc = k_d ψ + c_d ψ'` and spacecraft 2 receives `-Fc`. With the attitude
//! violation `φ` (an MRP set) the attitude torque on spacecraft 2 is
//! `-k_a φ - c_a ¼ [B(φ)] ω_B2B1` and spacecraft 1 receives its negative.
//! Each torque also carries the moment of the force about the body origin.
//! Stiff gains make the connection virtually rigid; set them through `alpha`
//! and `beta` (`k_d = k_a = α²`, `c_d = c_a = 2β`) or individually.
//!
//! The constraint is a [`CouplingEffector`] of a
//! [`SpacecraftGroup`](crate::spacecraft::SpacecraftGroup), which plays the
//! part of upstream `syncDynamicsIntegration`: both spacecraft are integrated
//! in one step and the constraint sees both stage states at every stage.
//! Upstream the effector is added to each spacecraft and alternates between
//! them, computing the loads when spacecraft 1 calls it and handing the stored
//! ones to spacecraft 2; here it computes both at once.
//!
//! After each update's integration the group calls
//! [`update`](CouplingEffector::update), which is upstream's `UpdateState`:
//! it reads the optional [`DeviceStatusMsg`] (a disabled constraint applies and
//! reports no loads but still measures the violations; it is active when the
//! input is not connected), low-pass filters the force and torque magnitudes
//! and writes the [`ConstDynEffectorMsg`]. Like upstream, the message carries
//! the loads and violation of the last stage the integrator evaluated, and the
//! status it reads applies from the next step.
//!
//! Deviations from upstream:
//! - The message's force is `Fc`, the force on spacecraft 1. Upstream writes
//!   `forceExternal_N`, which after its alternation holds spacecraft 2's
//!   `-Fc`; its own test checks the field against `Fc`.
//! - Upstream reads the parents' states from their state managers, so their
//!   positions are relative to the central body when one is set. Here the
//!   constraint gets inertial states; the two agree when both spacecraft
//!   share the central body.
//! - Upstream can also attach the constraint to a state effector (a branch)
//!   through its inertial properties. basilisk-rs attaches dynamic effectors
//!   to hubs only, so both ends here are spacecraft hubs.
//! - The setters return a [`ConstraintEffectorError`] where upstream raises a
//!   `BasiliskError`, keeping the previous value as upstream does. A group
//!   panics with that error when it initializes with an invalid constraint.

use std::any::Any;
use std::fmt;

use nalgebra::Vector3;

use crate::messages::{
    ConstDynEffectorMsg, DeviceState, DeviceStatusMsg, Input, Output, SpacecraftStateMsg,
};
use crate::spacecraft::{CouplingEffector, EffectorOutput};
use crate::utilities::rigid_body_kinematics::{b_mat_mrp, dcm_to_mrp, mrp_to_dcm};

/// A configuration the constraint rejects: upstream's `BasiliskError`s.
#[derive(Clone, Debug, PartialEq)]
pub enum ConstraintEffectorError {
    /// A tuning parameter or gain setter got a value that is not finite and
    /// positive.
    InvalidParameter {
        /// The setter's parameter.
        name: &'static str,
        /// The rejected value.
        value: f64,
    },
    /// `alpha` is not set and no gain is set explicitly.
    MissingAlpha,
    /// `beta` is not set and no gain is set explicitly.
    MissingBeta,
    /// `alpha` or `beta` overflows a gain derived from it.
    NonFiniteDerivedGains,
    /// The low-pass filter cut-off frequency is not positive.
    InvalidCutOffFrequency(f64),
}

impl fmt::Display for ConstraintEffectorError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidParameter { name, value } => {
                write!(
                    formatter,
                    "{name} must be finite and greater than 0, got {value}"
                )
            }
            Self::MissingAlpha => formatter
                .write_str("Alpha must be set to a positive nonzero value prior to initialization"),
            Self::MissingBeta => formatter
                .write_str("Beta must be set to a positive nonzero value prior to initialization"),
            Self::NonFiniteDerivedGains => {
                formatter.write_str("alpha and beta must produce finite derived gains")
            }
            Self::InvalidCutOffFrequency(cut_off) => write!(
                formatter,
                "Cut off frequency of low pass filter w_c must be greater than 0, got {cut_off}"
            ),
        }
    }
}

impl std::error::Error for ConstraintEffectorError {}

/// Geometry of a [`ConstraintDynamicEffector`].
#[derive(Clone, Debug, PartialEq)]
pub struct ConstraintDynamicEffectorConfig {
    pub name: String,
    /// Group indices of spacecraft 1, which hosts connection point P1 and
    /// frame B1, and spacecraft 2. Upstream takes them from the order in
    /// which the two spacecraft link the effector. Default `[0, 1]`.
    pub spacecraft: [usize; 2],
    /// Upstream `r_P1B1_B1` \[m\]: connection point P1 relative to the body
    /// origin of spacecraft 1, B1 components.
    pub connection_point_1_m: Vector3<f64>,
    /// Upstream `r_P2B2_B2` \[m\]: connection point P2 relative to the body
    /// origin of spacecraft 2, B2 components.
    pub connection_point_2_m: Vector3<f64>,
    /// Upstream `r_P2P1_B1Init` \[m\]: the prescribed position of P2 relative
    /// to P1, B1 components.
    pub separation_m: Vector3<f64>,
    /// Upstream `sigma_B2B1Init`: the prescribed attitude of B2 relative to
    /// B1 (MRP). Default zero, the frames aligned.
    pub sigma_b2b1: Vector3<f64>,
}

impl ConstraintDynamicEffectorConfig {
    /// A constraint between spacecraft 0 and 1 of the group, with every
    /// vector zero.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            spacecraft: [0, 1],
            connection_point_1_m: Vector3::zeros(),
            connection_point_2_m: Vector3::zeros(),
            separation_m: Vector3::zeros(),
            sigma_b2b1: Vector3::zeros(),
        }
    }
}

/// One gain: the value in use and whether a setter supplied it (upstream
/// `k_dExplicit` and its siblings). A derived gain is refreshed from `alpha`
/// or `beta` at every reset; an explicit one is kept.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Gain {
    value: f64,
    explicit: bool,
}

impl Gain {
    /// The value a reset gives it: its own if explicit, else `derived()`.
    fn reset_value(self, derived: impl FnOnce() -> f64) -> f64 {
        if self.explicit { self.value } else { derived() }
    }
}

/// Coefficients of upstream's second-order low-pass filter, the bilinear
/// discretization set by `setFilter_Data`: `y_t = a y_{t-1} + b y_{t-2} +
/// c u_t + d u_{t-1} + e u_{t-2}`. All zero, the default, filters to zero.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct FilterCoefficients {
    a: f64,
    b: f64,
    c: f64,
    d: f64,
    e: f64,
}

/// One filtered magnitude: its two previous inputs and outputs (upstream
/// `F_mag_tminus1`, `F_mag_tminus2`, `F_filtered_mag_tminus1`,
/// `F_filtered_mag_tminus2` and their torque counterparts).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct MagnitudeFilter {
    previous_inputs: [f64; 2],
    previous_outputs: [f64; 2],
}

impl MagnitudeFilter {
    /// Upstream `computeFilteredForce` / `computeFilteredTorque`: filters the
    /// new magnitude and shifts the history.
    fn step(&mut self, coefficients: &FilterCoefficients, magnitude: f64) -> f64 {
        let FilterCoefficients { a, b, c, d, e } = *coefficients;
        let [input_1, input_2] = self.previous_inputs;
        let [output_1, output_2] = self.previous_outputs;
        let filtered = a * output_1 + b * output_2 + c * magnitude + d * input_1 + e * input_2;
        self.previous_inputs = [magnitude, input_1];
        self.previous_outputs = [filtered, output_1];
        filtered
    }
}

/// Constraint between two spacecraft of a group: upstream
/// `ConstraintDynamicEffector`.
#[derive(Debug)]
pub struct ConstraintDynamicEffector {
    pub config: ConstraintDynamicEffectorConfig,
    /// Upstream `effectorStatusInMsg`: optional on/off command.
    pub effector_status_in: Input<DeviceStatusMsg>,
    /// Upstream `constraintElements`.
    pub constraint_elements_out: Output<ConstDynEffectorMsg>,
    /// Upstream `effectorStatus`: whether the constraint applies its loads.
    /// Every update sets it from `effector_status_in`, or to on when that is
    /// not connected; set it to choose the status of the first step. Default
    /// on.
    pub effector_status: DeviceState,
    alpha: f64,
    beta: f64,
    k_d: Gain,
    c_d: Gain,
    k_a: Gain,
    c_a: Gain,
    filter: FilterCoefficients,
    force_filter: MagnitudeFilter,
    torque_1_filter: MagnitudeFilter,
    torque_2_filter: MagnitudeFilter,
    /// Upstream `psi_N` \[m\], at the last evaluated stage.
    psi_n: Vector3<f64>,
    /// Upstream `Fc_N` \[N\]: force on spacecraft 1 at the last evaluated stage.
    force_n: Vector3<f64>,
    /// Upstream `T_B1` \[N·m\].
    torque_1_b1: Vector3<f64>,
    /// Upstream `T_B2` \[N·m\].
    torque_2_b2: Vector3<f64>,
}

impl ConstraintDynamicEffector {
    /// A constraint with every gain and tuning parameter unset and no
    /// filtering. Set `alpha` and `beta`, or gains, before the group
    /// initializes.
    pub fn new(config: ConstraintDynamicEffectorConfig) -> Self {
        Self {
            config,
            effector_status_in: Input::default(),
            constraint_elements_out: Output::default(),
            effector_status: DeviceState::On,
            alpha: 0.0,
            beta: 0.0,
            k_d: Gain::default(),
            c_d: Gain::default(),
            k_a: Gain::default(),
            c_a: Gain::default(),
            filter: FilterCoefficients::default(),
            force_filter: MagnitudeFilter::default(),
            torque_1_filter: MagnitudeFilter::default(),
            torque_2_filter: MagnitudeFilter::default(),
            psi_n: Vector3::zeros(),
            force_n: Vector3::zeros(),
            torque_1_b1: Vector3::zeros(),
            torque_2_b2: Vector3::zeros(),
        }
    }

    fn positive(name: &'static str, value: f64) -> Result<f64, ConstraintEffectorError> {
        if value.is_finite() && value > 0.0 {
            Ok(value)
        } else {
            Err(ConstraintEffectorError::InvalidParameter { name, value })
        }
    }

    fn set_gain(
        gain: &mut Gain,
        name: &'static str,
        value: f64,
    ) -> Result<(), ConstraintEffectorError> {
        *gain = Gain {
            value: Self::positive(name, value)?,
            explicit: true,
        };
        Ok(())
    }

    /// Upstream `setAlpha`: the proportional tuning parameter \[-\], whose
    /// square is the numerical value of the derived `k_d` \[N/m\] and `k_a`
    /// \[N·m\]. Finite and positive.
    pub fn set_alpha(&mut self, alpha: f64) -> Result<(), ConstraintEffectorError> {
        self.alpha = Self::positive("alpha", alpha)?;
        Ok(())
    }

    /// Upstream `setBeta`: the derivative tuning parameter \[-\], twice which
    /// is the numerical value of the derived `c_d` \[N·s/m\] and `c_a`
    /// \[N·m·s\]. Finite and positive.
    pub fn set_beta(&mut self, beta: f64) -> Result<(), ConstraintEffectorError> {
        self.beta = Self::positive("beta", beta)?;
        Ok(())
    }

    /// Upstream `setK_d`: the direction proportional gain \[N/m\], kept over
    /// `alpha²`. Finite and positive.
    pub fn set_k_d(&mut self, k_d: f64) -> Result<(), ConstraintEffectorError> {
        Self::set_gain(&mut self.k_d, "k_d", k_d)
    }

    /// Upstream `setC_d`: the direction derivative gain \[N·s/m\], kept over
    /// `2β`. Finite and positive.
    pub fn set_c_d(&mut self, c_d: f64) -> Result<(), ConstraintEffectorError> {
        Self::set_gain(&mut self.c_d, "c_d", c_d)
    }

    /// Upstream `setK_a`: the attitude proportional gain \[N·m\] on the
    /// dimensionless MRP error, kept over `alpha²`. Finite and positive.
    pub fn set_k_a(&mut self, k_a: f64) -> Result<(), ConstraintEffectorError> {
        Self::set_gain(&mut self.k_a, "k_a", k_a)
    }

    /// Upstream `setC_a`: the attitude derivative gain \[N·m·s\] on the MRP
    /// error rate, kept over `2β`. Finite and positive.
    pub fn set_c_a(&mut self, c_a: f64) -> Result<(), ConstraintEffectorError> {
        Self::set_gain(&mut self.c_a, "c_a", c_a)
    }

    /// Upstream `getAlpha`.
    pub fn alpha(&self) -> f64 {
        self.alpha
    }

    /// Upstream `getBeta`.
    pub fn beta(&self) -> f64 {
        self.beta
    }

    /// Upstream `getK_d` \[N/m\]: the explicit gain, or the one derived at
    /// the last reset.
    pub fn k_d(&self) -> f64 {
        self.k_d.value
    }

    /// Upstream `getC_d` \[N·s/m\].
    pub fn c_d(&self) -> f64 {
        self.c_d.value
    }

    /// Upstream `getK_a` \[N·m\].
    pub fn k_a(&self) -> f64 {
        self.k_a.value
    }

    /// Upstream `getC_a` \[N·m·s\].
    pub fn c_a(&self) -> f64 {
        self.c_a.value
    }

    /// Upstream `setFilter_Data`: a second-order low-pass filter of the load
    /// magnitudes with cut-off frequency `cut_off_radps`, digital step
    /// `step_s` \[s\] and damping coefficient `damping`. Without it the
    /// filtered magnitudes are zero.
    pub fn set_filter_data(
        &mut self,
        cut_off_radps: f64,
        step_s: f64,
        damping: f64,
    ) -> Result<(), ConstraintEffectorError> {
        // Upstream's `if (wc > 0)` rejects NaN too.
        if cut_off_radps.is_nan() || cut_off_radps <= 0.0 {
            return Err(ConstraintEffectorError::InvalidCutOffFrequency(
                cut_off_radps,
            ));
        }
        let wch2 = (cut_off_radps * step_s).powi(2);
        let numerator = [wch2, 2.0 * wch2, wch2];
        let denominator = [
            -4.0 + 4.0 * damping * step_s - wch2,
            8.0 - 2.0 * wch2,
            4.0 + 4.0 * damping * step_s + wch2,
        ];
        self.filter = FilterCoefficients {
            a: denominator[1] / denominator[2],
            b: denominator[0] / denominator[2],
            c: numerator[2] / denominator[2],
            d: numerator[1] / denominator[2],
            e: numerator[0] / denominator[2],
        };
        Ok(())
    }

    /// Upstream `Reset`: checks that the gains can be set
    /// (`validateConfiguration`), then derives every gain not set explicitly
    /// (`initializeGains`). A failed reset keeps the previous gains.
    pub fn reset(&mut self) -> Result<(), ConstraintEffectorError> {
        // Partial explicit configurations stay valid, zero unspecified gains
        // included (upstream comment).
        let any_explicit = [self.k_d, self.c_d, self.k_a, self.c_a]
            .iter()
            .any(|gain| gain.explicit);
        if self.alpha <= 0.0 && !any_explicit {
            return Err(ConstraintEffectorError::MissingAlpha);
        }
        if self.beta <= 0.0 && !any_explicit {
            return Err(ConstraintEffectorError::MissingBeta);
        }
        let (alpha, beta) = (self.alpha, self.beta);
        let k_d = self.k_d.reset_value(|| alpha * alpha);
        let c_d = self.c_d.reset_value(|| 2.0 * beta);
        let k_a = self.k_a.reset_value(|| alpha * alpha);
        let c_a = self.c_a.reset_value(|| 2.0 * beta);
        if ![k_d, c_d, k_a, c_a].iter().all(|gain| gain.is_finite()) {
            return Err(ConstraintEffectorError::NonFiniteDerivedGains);
        }
        self.k_d.value = k_d;
        self.c_d.value = c_d;
        self.k_a.value = k_a;
        self.c_a.value = c_a;
        Ok(())
    }

    /// Upstream `psi_N` \[m\]: the direction violation at the last evaluated
    /// stage.
    pub fn direction_violation_inertial_m(&self) -> Vector3<f64> {
        self.psi_n
    }

    /// Upstream `readInputMessage`.
    fn read_status(&mut self) {
        self.effector_status = if self.effector_status_in.is_connected() {
            self.effector_status_in.read().device_status
        } else {
            DeviceState::On
        };
    }
}

impl CouplingEffector for ConstraintDynamicEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    /// Upstream `linkInStates` (`validateConfiguration` and
    /// `initializeGains`) for both parents.
    ///
    /// # Panics
    ///
    /// If [`reset`](ConstraintDynamicEffector::reset) fails, or the two
    /// spacecraft indices are equal or outside the group.
    fn init(&mut self, spacecraft_count: usize) {
        if let Err(error) = self.reset() {
            panic!("ConstraintDynamicEffector '{}': {error}", self.config.name);
        }
        let [first, second] = self.config.spacecraft;
        assert!(
            first != second && first < spacecraft_count && second < spacecraft_count,
            "ConstraintDynamicEffector '{}' connects spacecraft {first} and {second} of a group of \
             {spacecraft_count}",
            self.config.name
        );
    }

    /// Upstream `computeForceTorque`: the violations, and while the
    /// constraint is on, the loads on both spacecraft.
    fn add_loads(&mut self, states: &[SpacecraftStateMsg], loads: &mut [EffectorOutput]) {
        let [first, second] = self.config.spacecraft;
        let (body_1, body_2) = (&states[first], &states[second]);
        let r_p1b1_b1 = self.config.connection_point_1_m;
        let r_p2b2_b2 = self.config.connection_point_2_m;
        let omega_b1n_b1 = body_1.omega_radps;
        let omega_b2n_b2 = body_2.omega_radps;

        // Direction constraint and its rate, inertial components.
        let dcm_b1n = mrp_to_dcm(body_1.sigma_bn);
        let dcm_b2n = mrp_to_dcm(body_2.sigma_bn);
        let dcm_nb1 = dcm_b1n.transpose();
        let dcm_nb2 = dcm_b2n.transpose();
        let r_p2p1_n =
            dcm_nb2 * r_p2b2_b2 + body_2.position_m - dcm_nb1 * r_p1b1_b1 - body_1.position_m;
        let rdot_p1n_n = dcm_nb1 * omega_b1n_b1.cross(&r_p1b1_b1) + body_1.velocity_mps;
        let rdot_p2n_n = dcm_nb2 * omega_b2n_b2.cross(&r_p2b2_b2) + body_2.velocity_mps;
        let omega_b1n_n = dcm_nb1 * omega_b1n_b1;
        self.psi_n = r_p2p1_n - dcm_nb1 * self.config.separation_m;
        let psi_prime_n = rdot_p2n_n - rdot_p1n_n - omega_b1n_n.cross(&r_p2p1_n);

        // Attitude constraint and its rate.
        let dcm_b2b1_init = mrp_to_dcm(self.config.sigma_b2b1);
        let phi = dcm_to_mrp(dcm_b2n * dcm_nb1 * dcm_b2b1_init.transpose());
        let omega_b2b1_b2 = omega_b2n_b2 - dcm_b2n * omega_b1n_n;

        if !self.effector_status.is_on() {
            // Disabled: apply and report no loads, but keep measuring the
            // violations (upstream).
            self.force_n = Vector3::zeros();
            self.torque_1_b1 = Vector3::zeros();
            self.torque_2_b2 = Vector3::zeros();
            return;
        }

        self.force_n = self.k_d.value * self.psi_n + self.c_d.value * psi_prime_n;
        let torque_1_length_b1 = r_p1b1_b1.cross(&(dcm_b1n * self.force_n));
        let torque_2_length_b2 = -r_p2b2_b2.cross(&(dcm_b2n * self.force_n));
        let torque_2_attitude_b2 =
            -self.k_a.value * phi - self.c_a.value * 0.25 * b_mat_mrp(phi) * omega_b2b1_b2;
        let torque_1_attitude_b1 = -(dcm_b1n * dcm_nb2) * torque_2_attitude_b2;
        self.torque_1_b1 = torque_1_length_b1 + torque_1_attitude_b1;
        self.torque_2_b2 = torque_2_length_b2 + torque_2_attitude_b2;

        loads[first].force_inertial_n += self.force_n;
        loads[first].torque_body_nm += self.torque_1_b1;
        loads[second].force_inertial_n -= self.force_n;
        loads[second].torque_body_nm += self.torque_2_b2;
    }

    /// Upstream `UpdateState`: reads the status, filters the load magnitudes
    /// and writes the output message.
    fn update(&mut self, _current_sim_nanos: u64) {
        self.read_status();
        let force_filtered = self.force_filter.step(&self.filter, self.force_n.norm());
        let torque_1_filtered = self
            .torque_1_filter
            .step(&self.filter, self.torque_1_b1.norm());
        let torque_2_filtered = self
            .torque_2_filter
            .step(&self.filter, self.torque_2_b2.norm());
        self.constraint_elements_out.write(ConstDynEffectorMsg {
            force_inertial_n: self.force_n,
            torque_1_body_nm: self.torque_1_b1,
            torque_2_body_nm: self.torque_2_b2,
            direction_violation_inertial_m: self.psi_n,
            force_magnitude_filtered_n: force_filtered,
            torque_1_magnitude_filtered_nm: torque_1_filtered,
            torque_2_magnitude_filtered_nm: torque_2_filtered,
        });
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_ConstraintDynamicEffectorUnit.py`,
    //! `test_InOutMessageFiltering.py` and `test_constraintEffectorValidation.py`.
    //!
    //! Upstream integrates both spacecraft with `svIntegratorRKF45` at its
    //! default tolerances (`relTol = 1e-4`, `absTol = 1e-8`); here
    //! [`Rkf45`] with the same tolerances advances the
    //! [`SpacecraftGroup`]. Every check uses numpy's `assert_allclose`, so
    //! besides the stated absolute tolerance it allows `1e-7` of the expected
    //! value. Upstream gives the initial hub center of mass (`r_CN_NInit`,
    //! `v_CN_NInit`); basilisk-rs starts the body origin, offset from it by
    //! `-r_BcB_B`.
    //!
    //! The validation cases that attach the constraint to a spinning-body
    //! branch (`attachToBranch`, path `"branch"`) are not ported: basilisk-rs
    //! attaches dynamic effectors to hubs only and has no
    //! `spinningBodyOneDOFStateEffector`. Upstream's single-spacecraft hub case
    //! runs here with a group of two spacecraft, the smallest a constraint
    //! connects.

    use std::panic::{AssertUnwindSafe, catch_unwind};

    use hifitime::{Duration, Epoch};
    use nalgebra::{Matrix3, Vector3};

    use super::*;
    use crate::dynamics::gravity::GravBodyData;
    use crate::integrators::{AdaptiveController, Rkf45};
    use crate::messages::SpacecraftDiagnosticsMsg;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig, SpacecraftGroup};
    use crate::utilities::astro_constants::D2R;
    use crate::utilities::orbital_motion::{ClassicElements, elements_to_state};
    use crate::utilities::rigid_body_kinematics::sub_mrp;
    use crate::{Module, SimulationContext};

    /// Upstream `testProcessRate` \[ns\].
    const STEP_NANOS: u64 = 1_000_000;
    /// Upstream `stopTime` \[s\], in steps.
    const STEP_COUNT: u64 = 1_000;
    const HUB_MASS_KG: f64 = 750.0;
    /// Upstream `ALPHA` and `BETA`, both `1E3`.
    const TUNING: f64 = 1.0e3;

    fn start_epoch() -> Epoch {
        Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
    }

    fn context(tick: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: tick * STEP_NANOS,
            current_epoch: start_epoch()
                + Duration::from_total_nanoseconds((tick * STEP_NANOS) as i128),
        }
    }

    /// `np.testing.assert_allclose(actual, desired, atol)` with its default
    /// `rtol = 1e-7`.
    fn assert_allclose(actual: f64, desired: f64, atol: f64, context: &str) {
        assert!(
            (actual - desired).abs() <= atol + 1.0e-7 * desired.abs(),
            "{context}: {actual:e} vs {desired:e} (atol {atol:e})"
        );
    }

    fn assert_vector_allclose(
        actual: Vector3<f64>,
        desired: Vector3<f64>,
        atol: f64,
        context: &str,
    ) {
        for axis in 0..3 {
            assert_allclose(actual[axis], desired[axis], atol, context);
        }
    }

    fn hub_center_of_mass_m() -> Vector3<f64> {
        Vector3::new(0.0, 0.0, 1.0)
    }

    /// Upstream `IHubPntBc_B` \[kg m²\].
    fn hub_inertia_kg_m2() -> Matrix3<f64> {
        Matrix3::from_diagonal_element(600.0)
    }

    /// Upstream's hub: 750 kg, `r_BcB_B = [0, 0, 1]` m, `IHubPntBc_B = 600 I`,
    /// starting from the center-of-mass state `r_CN_NInit`, `v_CN_NInit` with
    /// the frames aligned.
    fn hub(
        com_position_m: Vector3<f64>,
        com_velocity_mps: Vector3<f64>,
        omega_radps: Vector3<f64>,
    ) -> Spacecraft {
        let center_of_mass_m = hub_center_of_mass_m();
        Spacecraft::new(SpacecraftConfig {
            mass_kg: HUB_MASS_KG,
            hub_center_of_mass_body_m: center_of_mass_m,
            inertia_kg_m2: hub_inertia_kg_m2(),
            integration_step_nanos: STEP_NANOS,
            initial_position_m: com_position_m - center_of_mass_m,
            initial_velocity_mps: com_velocity_mps - omega_radps.cross(&center_of_mass_m),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: omega_radps,
            integrator: None,
        })
    }

    /// Upstream `svIntegratorRKF45` with its default tolerances.
    fn upstream_rkf45() -> Rkf45 {
        Rkf45::new(AdaptiveController {
            rtol: 1.0e-4,
            atol: 1.0e-8,
            ..AdaptiveController::default()
        })
    }

    /// The constraint geometry of the conservation and filtering tests.
    struct Geometry {
        connection_point_1_m: Vector3<f64>,
        connection_point_2_m: Vector3<f64>,
        separation_m: Vector3<f64>,
    }

    impl Geometry {
        /// Both arms `COMoffset = 0.1` m and the separation `l = 0.1` m along
        /// `direction`.
        fn along(direction: Vector3<f64>) -> Self {
            Self {
                connection_point_1_m: direction * 0.1,
                connection_point_2_m: -direction * 0.1,
                separation_m: direction * 0.1,
            }
        }

        fn constraint(&self) -> ConstraintDynamicEffector {
            ConstraintDynamicEffector::new(ConstraintDynamicEffectorConfig {
                connection_point_1_m: self.connection_point_1_m,
                connection_point_2_m: self.connection_point_2_m,
                separation_m: self.separation_m,
                ..ConstraintDynamicEffectorConfig::new("constraintEffector")
            })
        }
    }

    /// Upstream's two spacecraft spinning together at `[0.01, 0.01, 0.01]`
    /// rad/s about their joint center of mass, spacecraft 2's hub center of
    /// mass at `r_B2N_N_0` moving at `rDot_B2N_N` and the constraint initially
    /// satisfied along `r_B2N_N_0`.
    fn spinning_pair(
        r_b2n_n: Vector3<f64>,
        rdot_b2n_n: Vector3<f64>,
    ) -> (Geometry, [Spacecraft; 2]) {
        let geometry = Geometry::along(r_b2n_n / r_b2n_n.norm());
        let r_b1n_n = r_b2n_n + geometry.connection_point_2_m
            - geometry.separation_m
            - geometry.connection_point_1_m;
        let r_cn_n = (r_b1n_n * HUB_MASS_KG + r_b2n_n * HUB_MASS_KG) / (2.0 * HUB_MASS_KG);
        let omega_cn_n = Vector3::repeat(0.01);
        let rdot_b1n_n_0 = rdot_b2n_n + omega_cn_n.cross(&(r_b1n_n - r_cn_n));
        let rdot_b2n_n_0 = rdot_b2n_n + omega_cn_n.cross(&(r_b2n_n - r_cn_n));
        let pair = [
            hub(r_b1n_n, rdot_b1n_n_0, omega_cn_n),
            hub(r_b2n_n, rdot_b2n_n_0, omega_cn_n),
        ];
        (geometry, pair)
    }

    /// One sample of a run: both spacecraft states and diagnostics, and the
    /// constraint message.
    struct Sample {
        states: [SpacecraftStateMsg; 2],
        diagnostics: [SpacecraftDiagnosticsMsg; 2],
        constraint: ConstDynEffectorMsg,
    }

    /// Groups `pair` with `constraint` and upstream's RKF45, and runs it from
    /// `t = 0` to 1 s like upstream's task, recording every step. `switch`
    /// runs between ticks, after the tick it is given.
    fn run(
        pair: [Spacecraft; 2],
        constraint: ConstraintDynamicEffector,
        mut switch: impl FnMut(u64),
    ) -> Vec<Sample> {
        let constraint_out = constraint.constraint_elements_out.clone();
        let mut group = SpacecraftGroup::new(pair.into());
        group.integrator = Some(Box::new(upstream_rkf45()));
        group.add_coupling_effector(constraint);
        group.init();
        (0..=STEP_COUNT)
            .map(|tick| {
                group.update(&context(tick));
                switch(tick);
                let [first, second] = group.spacecraft() else {
                    unreachable!()
                };
                Sample {
                    states: [first.state_out.read(), second.state_out.read()],
                    diagnostics: [first.diagnostics_out.read(), second.diagnostics_out.read()],
                    constraint: constraint_out.read(),
                }
            })
            .collect()
    }

    fn tuned(mut constraint: ConstraintDynamicEffector) -> ConstraintDynamicEffector {
        constraint.set_alpha(TUNING).expect("valid alpha");
        constraint.set_beta(TUNING).expect("valid beta");
        constraint
    }

    /// Upstream `psi_B1`: the direction violation from the logged states, B1
    /// components.
    fn direction_violation_b1(
        geometry: &Geometry,
        states: &[SpacecraftStateMsg; 2],
    ) -> Vector3<f64> {
        let dcm_b1n = mrp_to_dcm(states[0].sigma_bn);
        let dcm_nb2 = mrp_to_dcm(states[1].sigma_bn).transpose();
        dcm_b1n * states[0].position_m + geometry.connection_point_1_m + geometry.separation_m
            - (dcm_b1n * states[1].position_m + dcm_b1n * dcm_nb2 * geometry.connection_point_2_m)
    }

    /// Upstream `constraintEffectorOrbitalConservation`: the pair in an
    /// eccentric LEO about a point-mass Earth. The direction violation stays
    /// within 1e-7 m and the attitude violation within 1e-4, and the combined
    /// orbital angular momentum and energy are conserved.
    #[test]
    fn orbital_conservation() {
        let mu_m3ps2 = 0.3986004415e15;
        let elements = ClassicElements {
            // Upstream `radEquator + 7500e3`, with `radEquator` left at 0.
            semi_major_axis: 7500.0e3,
            eccentricity: 0.01,
            inclination_rad: 30.0 * D2R,
            raan_rad: 60.0 * D2R,
            arg_periapsis_rad: 15.0 * D2R,
            true_anomaly_rad: 90.0 * D2R,
            ..ClassicElements::default()
        };
        let (r_b2n_n, rdot_b2n_n) = elements_to_state(mu_m3ps2, &elements);
        let (geometry, mut pair) = spinning_pair(r_b2n_n, rdot_b2n_n);
        for spacecraft in &mut pair {
            spacecraft
                .add_grav_body(
                    GravBodyData::point_mass(
                        "earth_planet_data",
                        mu_m3ps2,
                        true,
                        Vector3::zeros(),
                        Vector3::zeros(),
                    )
                    .expect("valid Earth gravity body"),
                )
                .expect("unique central gravity body");
        }
        let samples = run(pair, tuned(geometry.constraint()), |_| {});

        let direction_accuracy_m = 1.0e-7;
        let attitude_accuracy = 1.0e-4;
        let accuracy = 1.0e-12;
        let combined_momentum = |sample: &Sample| {
            sample.diagnostics[0].orbital_angular_momentum_inertial_kg_m2ps
                + sample.diagnostics[1].orbital_angular_momentum_inertial_kg_m2ps
        };
        let combined_energy = |sample: &Sample| {
            sample.diagnostics[0].orbital_energy_j + sample.diagnostics[1].orbital_energy_j
        };
        let (momentum_0, energy_0) = (combined_momentum(&samples[0]), combined_energy(&samples[0]));
        for sample in &samples {
            assert_vector_allclose(
                direction_violation_b1(&geometry, &sample.states),
                Vector3::zeros(),
                direction_accuracy_m,
                "direction constraint component magnitude exceeded in orbital conservation test",
            );
            assert_vector_allclose(
                sub_mrp(sample.states[1].sigma_bn, sample.states[0].sigma_bn),
                Vector3::zeros(),
                attitude_accuracy,
                "attitude constraint component magnitude exceeded in orbital conservation test",
            );
            assert_vector_allclose(
                combined_momentum(sample),
                momentum_0,
                accuracy,
                "orbital angular momentum difference component magnitude exceeded",
            );
            assert_allclose(
                combined_energy(sample),
                energy_0,
                accuracy,
                "orbital energy difference magnitude exceeded",
            );
        }
    }

    /// Upstream `constraintEffectorRotationalConservation`: the pair without
    /// gravity. Both violations stay within 1e-12, and the combined rotational
    /// angular momentum about the joint center of mass and the combined
    /// rotational energy are conserved.
    #[test]
    fn rotational_conservation() {
        let (geometry, pair) = spinning_pair(Vector3::repeat(1.0), Vector3::repeat(1.0));
        let samples = run(pair, tuned(geometry.constraint()), |_| {});

        // Upstream's angular momentum of both hubs about their joint
        // (body-origin) center of mass, from the logged states.
        let inertia = hub_inertia_kg_m2();
        let momenta = |sample: &Sample| {
            let [first, second] = &sample.states;
            let r_cn_n = (first.position_m + second.position_m) / 2.0;
            let rdot_cn_n = (first.velocity_mps + second.velocity_mps) / 2.0;
            [first, second].map(|state| {
                let r_bc_n = state.position_m - r_cn_n;
                let rdot_bc_n = state.velocity_mps - rdot_cn_n;
                r_cn_n.cross(&(HUB_MASS_KG * rdot_bc_n))
                    + mrp_to_dcm(state.sigma_bn).transpose() * (inertia * state.omega_radps)
                    + HUB_MASS_KG * r_bc_n.cross(&rdot_bc_n)
            })
        };
        let combined_energy = |sample: &Sample| {
            sample.diagnostics[0].rotational_energy_j + sample.diagnostics[1].rotational_energy_j
        };
        let [momentum_1_0, momentum_2_0] = momenta(&samples[0]);
        let energy_0 = combined_energy(&samples[0]);
        let accuracy = 1.0e-12;
        for sample in &samples {
            assert_vector_allclose(
                direction_violation_b1(&geometry, &sample.states),
                Vector3::zeros(),
                accuracy,
                "direction constraint component magnitude exceeded in rotational conservation test",
            );
            assert_vector_allclose(
                sample.states[1].sigma_bn - sample.states[0].sigma_bn,
                Vector3::zeros(),
                accuracy,
                "attitude constraint component magnitude exceeded in rotational conservation test",
            );
            let [momentum_1, momentum_2] = momenta(sample);
            assert_vector_allclose(
                momentum_1 + momentum_2,
                momentum_1_0 + momentum_2_0,
                accuracy,
                "rotational angular momentum difference component magnitude exceeded",
            );
            assert_allclose(
                combined_energy(sample),
                energy_0,
                accuracy,
                "rotational energy difference magnitude exceeded",
            );
        }
    }

    /// Upstream's check of the force and torques: `Fc`, `L1_B1` and `L2_B2`
    /// from the logged states, with `k_d` and `c_d` standing for the equal
    /// attitude gains.
    fn expected_loads(
        geometry: &Geometry,
        states: &[SpacecraftStateMsg; 2],
        k_d: f64,
        c_d: f64,
    ) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>, Vector3<f64>) {
        let [first, second] = states;
        let dcm_b1n = mrp_to_dcm(first.sigma_bn);
        let dcm_b2n = mrp_to_dcm(second.sigma_bn);
        let (dcm_nb1, dcm_nb2) = (dcm_b1n.transpose(), dcm_b2n.transpose());
        let r_p2p1_n = dcm_nb2 * geometry.connection_point_2_m + second.position_m
            - dcm_nb1 * geometry.connection_point_1_m
            - first.position_m;
        let sigma_b2b1 = dcm_to_mrp(dcm_b2n * dcm_nb1);
        let rdot_p1n_n =
            dcm_nb1 * first.omega_radps.cross(&geometry.connection_point_1_m) + first.velocity_mps;
        let rdot_p2n_n = dcm_nb2 * second.omega_radps.cross(&geometry.connection_point_2_m)
            + second.velocity_mps;
        let psi_n = r_p2p1_n - dcm_nb1 * geometry.separation_m;
        let omega_b1n_n = dcm_nb1 * first.omega_radps;
        let psi_prime_n = rdot_p2n_n - rdot_p1n_n - omega_b1n_n.cross(&r_p2p1_n);
        let force_n = k_d * psi_n + c_d * psi_prime_n;
        let omega_b2b1_b2 = second.omega_radps - dcm_b2n * omega_b1n_n;
        let torque_1_length = geometry.connection_point_1_m.cross(&(dcm_b1n * force_n));
        let torque_2_length = -geometry.connection_point_2_m.cross(&(dcm_b2n * force_n));
        let torque_2_attitude =
            -k_d * sigma_b2b1 - c_d * 0.25 * b_mat_mrp(sigma_b2b1) * omega_b2b1_b2;
        let torque_1_attitude = -(dcm_b1n * dcm_nb2) * torque_2_attitude;
        (
            psi_n,
            force_n,
            torque_1_length + torque_1_attitude,
            torque_2_length + torque_2_attitude,
        )
    }

    /// Upstream's filter recursion on the logged magnitudes: the first two
    /// filtered values from the log, then
    /// `y_i = a y_{i-1} + b y_{i-2} + c u_i + d u_{i-1} + e u_{i-2}`.
    fn filtered(coefficients: [f64; 5], logged: &[f64], magnitudes: &[f64]) -> f64 {
        let [a, b, c, d, e] = coefficients;
        let mut check = logged[..2].to_vec();
        for i in 2..magnitudes.len() {
            check.push(
                a * check[i - 1]
                    + b * check[i - 2]
                    + c * magnitudes[i]
                    + d * magnitudes[i - 1]
                    + e * magnitudes[i - 2],
            );
        }
        check[magnitudes.len() - 1]
    }

    /// Upstream `test_constraintEffectorAllCases` for a positive cut-off
    /// frequency, with the status input `On`, `Off` or not connected: the
    /// rotational scenario's final message matches the force, torques and
    /// violation computed from the final states and the filter recursion
    /// on the logged magnitudes, all to 1e-8.
    fn check_message_and_filter(status: Option<DeviceState>) {
        let (cut_off_radps, step_s, damping) = (0.1, 1.0, 0.7);
        let (geometry, pair) = spinning_pair(Vector3::repeat(1.0), Vector3::repeat(1.0));
        let mut constraint = tuned(geometry.constraint());
        constraint
            .set_filter_data(cut_off_radps, step_s, damping)
            .expect("positive cut-off frequency");
        if let Some(device_status) = status {
            let status_out = Output::new(DeviceStatusMsg { device_status });
            constraint.effector_status_in.connect(status_out.slot());
        }
        let samples = run(pair, constraint, |_| {});

        let accuracy = 1.0e-8;
        let last = samples.last().expect("logged samples");
        let message = &last.constraint;
        if status != Some(DeviceState::Off) {
            let (k_d, c_d) = (TUNING * TUNING, 2.0 * TUNING);
            let (psi_n, force_n, torque_1, torque_2) =
                expected_loads(&geometry, &last.states, k_d, c_d);
            let norm = |vector: Vector3<f64>| vector.norm();
            assert_allclose(
                norm(message.direction_violation_inertial_m - psi_n),
                0.0,
                accuracy,
                "direction constraint output message norm is incorrect",
            );
            assert_allclose(
                norm(message.force_inertial_n - force_n),
                0.0,
                accuracy,
                "constraint force output message norm is incorrect",
            );
            assert_allclose(
                norm(message.torque_1_body_nm - torque_1),
                0.0,
                accuracy,
                "constraint torque on s/c 1 output message norm is incorrect",
            );
            assert_allclose(
                norm(message.torque_2_body_nm - torque_2),
                0.0,
                accuracy,
                "constraint torque on s/c 2 output message norm is incorrect",
            );

            let wch2 = (cut_off_radps * step_s).powi(2);
            let denominator = [
                -4.0 + 4.0 * damping * step_s - wch2,
                8.0 - 2.0 * wch2,
                4.0 + 4.0 * damping * step_s + wch2,
            ];
            let coefficients = [
                denominator[1] / denominator[2],
                denominator[0] / denominator[2],
                wch2 / denominator[2],
                2.0 * wch2 / denominator[2],
                wch2 / denominator[2],
            ];
            let series = |field: fn(&ConstDynEffectorMsg) -> f64| -> Vec<f64> {
                samples
                    .iter()
                    .map(|sample| field(&sample.constraint))
                    .collect()
            };
            for (name, logged, magnitudes) in [
                (
                    "filtered constraint force",
                    series(|m| m.force_magnitude_filtered_n),
                    series(|m| m.force_inertial_n.norm()),
                ),
                (
                    "filtered constraint torque on s/c 1",
                    series(|m| m.torque_1_magnitude_filtered_nm),
                    series(|m| m.torque_1_body_nm.norm()),
                ),
                (
                    "filtered constraint torque on s/c 2",
                    series(|m| m.torque_2_magnitude_filtered_nm),
                    series(|m| m.torque_2_body_nm.norm()),
                ),
            ] {
                assert_allclose(
                    logged[logged.len() - 1] - filtered(coefficients, &logged, &magnitudes),
                    0.0,
                    accuracy,
                    &format!("{name} output message norm is incorrect"),
                );
            }
        }

        if status == Some(DeviceState::Off) {
            assert_eq!(
                message.force_inertial_n.norm(),
                0.0,
                "deviceStatus 0 test case failed"
            );
        } else {
            assert!(
                message.force_inertial_n.norm() > 0.0,
                "deviceStatus test case failed"
            );
        }
        if status == Some(DeviceState::On) {
            assert!(
                message.force_magnitude_filtered_n > 0.0,
                "positive cut off frequency test case failed"
            );
        }
    }

    /// Upstream `test_constraintEffectorAllCases(0.1, 0)`.
    #[test]
    fn message_and_filter_with_the_constraint_off() {
        check_message_and_filter(Some(DeviceState::Off));
    }

    /// Upstream `test_constraintEffectorAllCases(0.1, 1)`.
    #[test]
    fn message_and_filter_with_the_constraint_on() {
        check_message_and_filter(Some(DeviceState::On));
    }

    /// Upstream `test_constraintEffectorAllCases(0.1, -1)`: without a status
    /// input the constraint is on.
    #[test]
    fn message_and_filter_without_a_status_input() {
        check_message_and_filter(None);
    }

    /// Upstream `test_constraintEffectorAllCases(-1, 1)` and `(0, 1)`: a
    /// cut-off frequency that is not positive is rejected, and the filter
    /// keeps filtering to zero.
    #[test]
    fn non_positive_cut_off_frequency_is_rejected() {
        let mut constraint =
            ConstraintDynamicEffector::new(ConstraintDynamicEffectorConfig::new("constraint"));
        for cut_off_radps in [-1.0, 0.0, f64::NAN] {
            let error = constraint
                .set_filter_data(cut_off_radps, 1.0, 0.7)
                .expect_err("non-positive cut-off frequency");
            assert!(matches!(
                error,
                ConstraintEffectorError::InvalidCutOffFrequency(_)
            ));
        }
        assert_eq!(constraint.filter, FilterCoefficients::default());
    }

    /// The stretched pair of `test_constraintEffectorDeviceStatusGatesLoads`.
    struct DeviceStatusRun {
        dv_max: f64,
        dv_after_switch: f64,
        final_loads: f64,
        psi_n: Vector3<f64>,
        psi_expected_n: Vector3<f64>,
        momentum: f64,
    }

    /// Upstream `run_deviceStatus_case`: both hubs at rest with their
    /// connection points 1 mm further apart than the constraint length, the
    /// status starting at `initial` and switching at `switch_tick` when given.
    fn run_device_status_case(initial: DeviceState, switch_tick: Option<u64>) -> DeviceStatusRun {
        let link_axis = Vector3::x();
        let stretch_m = 1.0e-3;
        let geometry = Geometry::along(link_axis);
        let second_com_m = geometry.connection_point_1_m + geometry.separation_m
            - geometry.connection_point_2_m
            + link_axis * stretch_m;
        let pair = [
            hub(Vector3::zeros(), Vector3::zeros(), Vector3::zeros()),
            hub(second_com_m, Vector3::zeros(), Vector3::zeros()),
        ];
        let mut constraint = tuned(geometry.constraint());
        constraint
            .set_filter_data(0.1, 1.0, 0.7)
            .expect("positive cut-off frequency");
        constraint.effector_status = initial;
        let status_out = Output::new(DeviceStatusMsg {
            device_status: initial,
        });
        constraint.effector_status_in.connect(status_out.slot());
        let switched = if initial.is_on() {
            DeviceState::Off
        } else {
            DeviceState::On
        };
        let samples = run(pair, constraint, |tick| {
            if Some(tick) == switch_tick {
                status_out.write(DeviceStatusMsg {
                    device_status: switched,
                });
            }
        });

        // The effector reads the switched status on its next task step, so
        // two steps are skipped before measuring the growth (upstream).
        let split_tick = switch_tick.unwrap_or(STEP_COUNT / 2) as usize + 2;
        let dv = |index: usize| -> Vec<Vector3<f64>> {
            samples
                .iter()
                .map(|sample| sample.states[index].accumulated_dv_cn_n_mps)
                .collect()
        };
        let (dv_1, dv_2) = (dv(0), dv(1));
        let largest_component = |values: &mut dyn Iterator<Item = Vector3<f64>>| {
            values.map(|value| value.abs().max()).fold(0.0, f64::max)
        };
        let dv_after_switch =
            largest_component(&mut dv_1[split_tick..].iter().map(|dv| dv - dv_1[split_tick])).max(
                largest_component(&mut dv_2[split_tick..].iter().map(|dv| dv - dv_2[split_tick])),
            );
        let dv_max = largest_component(&mut dv_1.iter().copied())
            .max(largest_component(&mut dv_2.iter().copied()));
        let last = samples.last().expect("logged samples");
        let final_loads = last
            .constraint
            .force_inertial_n
            .iter()
            .chain(last.constraint.torque_1_body_nm.iter())
            .chain(last.constraint.torque_2_body_nm.iter())
            .fold(0.0_f64, |largest, value| largest.max(value.abs()));
        let [first, second] = &last.states;
        let psi_expected_n = second.position_m
            + mrp_to_dcm(second.sigma_bn).transpose() * geometry.connection_point_2_m
            - first.position_m
            - mrp_to_dcm(first.sigma_bn).transpose() * geometry.connection_point_1_m
            - mrp_to_dcm(first.sigma_bn).transpose() * geometry.separation_m;
        // Upstream logs `v_CN_N`; a rigid hub's center of mass moves at
        // `v_BN + [NB] (ω × r_BcB)`.
        let com_velocity = |state: &SpacecraftStateMsg| {
            state.velocity_mps
                + mrp_to_dcm(state.sigma_bn).transpose()
                    * state.omega_radps.cross(&hub_center_of_mass_m())
        };
        let momentum = samples
            .iter()
            .map(|sample| {
                (HUB_MASS_KG * com_velocity(&sample.states[0])
                    + HUB_MASS_KG * com_velocity(&sample.states[1]))
                .norm()
            })
            .fold(0.0, f64::max);
        DeviceStatusRun {
            dv_max,
            dv_after_switch,
            final_loads,
            psi_n: last.constraint.direction_violation_inertial_m,
            psi_expected_n,
            momentum,
        }
    }

    /// Upstream `test_constraintEffectorDeviceStatusGatesLoads`: the status
    /// gates the loads themselves, not only the message. Off, on, on then
    /// off and off then on at 0.5 s: no run changes the total momentum; the
    /// disabled run accumulates no ΔV and the enabled ones do; switched off,
    /// the constraint reports no loads and stops accelerating the spacecraft
    /// but still reports the violation the states imply.
    #[test]
    fn device_status_gates_loads() {
        let accuracy = 1.0e-8;
        let momentum_accuracy = 1.0e-6;
        // Bounds the drift between the reported violation and the logged
        // states (upstream).
        let psi_accuracy = 1.0e-5;
        let switch_tick = Some(STEP_COUNT / 2);
        let off = run_device_status_case(DeviceState::Off, None);
        let on = run_device_status_case(DeviceState::On, None);
        let on_off = run_device_status_case(DeviceState::On, switch_tick);
        let off_on = run_device_status_case(DeviceState::Off, switch_tick);

        for (name, run) in [
            ("off", &off),
            ("on", &on),
            ("on then off", &on_off),
            ("off then on", &off_on),
        ] {
            assert!(
                run.momentum < momentum_accuracy,
                "{name} run changed the total linear momentum by {:e} kg m/s",
                run.momentum
            );
        }
        assert!(
            on.dv_max > 1.0e-3,
            "enabled constraint imparted no measurable impulse"
        );
        assert!(
            off_on.dv_max > 1.0e-3,
            "constraint enabled after a disabled initialization imparted no measurable impulse"
        );
        assert!(
            on.dv_after_switch > 1.0e-3,
            "enabled constraint imparted no impulse over the second half of the run"
        );
        assert!(
            on_off.psi_expected_n.norm() > 5.0 * psi_accuracy,
            "released spacecraft barely drifted apart"
        );
        assert_allclose(
            off.dv_max,
            0.0,
            accuracy,
            "disabled constraint still accelerated the spacecraft",
        );
        assert_allclose(
            on_off.final_loads,
            0.0,
            accuracy,
            "constraint switched off still reported loads in its output message",
        );
        assert_allclose(
            on_off.dv_after_switch,
            0.0,
            accuracy,
            "constraint switched off kept accelerating the spacecraft",
        );
        assert_vector_allclose(
            on_off.psi_n,
            on_off.psi_expected_n,
            psi_accuracy,
            "constraint switched off stopped reporting the direction constraint violation",
        );
    }

    /// How the validation tests initialize the constraint: upstream's direct
    /// `Reset(0)`, or the group initializing (upstream `InitializeSimulation`
    /// attaching it to the hubs).
    #[derive(Clone, Copy, Debug, PartialEq)]
    enum Path {
        Reset,
        Hub,
    }

    /// Upstream `_simulationWithConstraint` for a hub: two 100 kg spacecraft
    /// with `IHubPntBc_B = diag(10, 20, 30)` kg m² and a 0.1 s step, and the
    /// constraint, unconfigured, in their group.
    fn group_with_constraint() -> SpacecraftGroup {
        let spacecraft = || {
            Spacecraft::new(SpacecraftConfig {
                mass_kg: 100.0,
                hub_center_of_mass_body_m: Vector3::zeros(),
                inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(10.0, 20.0, 30.0)),
                integration_step_nanos: 100_000_000,
                initial_position_m: Vector3::zeros(),
                initial_velocity_mps: Vector3::zeros(),
                initial_sigma_bn: Vector3::zeros(),
                initial_omega_radps: Vector3::zeros(),
                integrator: None,
            })
        };
        let mut group = SpacecraftGroup::new(vec![spacecraft(), spacecraft()]);
        group.add_coupling_effector(ConstraintDynamicEffector::new(
            ConstraintDynamicEffectorConfig::new("constraint"),
        ));
        group
    }

    fn constraint_mut(group: &mut SpacecraftGroup) -> &mut ConstraintDynamicEffector {
        group.coupling_effectors_mut()[0]
            .as_any_mut()
            .downcast_mut()
            .expect("the group's constraint")
    }

    /// Initializes along `path`; an error carries upstream's message.
    fn initialize(path: Path, group: &mut SpacecraftGroup) -> Result<(), String> {
        match path {
            Path::Reset => constraint_mut(group)
                .reset()
                .map_err(|error| error.to_string()),
            Path::Hub => catch_unwind(AssertUnwindSafe(|| group.init())).map_err(|panic| {
                panic
                    .downcast_ref::<String>()
                    .cloned()
                    .or_else(|| {
                        panic
                            .downcast_ref::<&str>()
                            .map(|message| message.to_string())
                    })
                    .unwrap_or_default()
            }),
        }
    }

    fn gains(constraint: &ConstraintDynamicEffector) -> [f64; 4] {
        [
            constraint.k_d(),
            constraint.c_d(),
            constraint.k_a(),
            constraint.c_a(),
        ]
    }

    /// Upstream `test_constraintEffector_validation`: both paths reject a
    /// missing alpha, and with alpha set a missing beta.
    #[test]
    fn missing_tuning_parameters_are_rejected() {
        for path in [Path::Reset, Path::Hub] {
            for missing in ["Alpha", "Beta"] {
                let mut group = group_with_constraint();
                if missing == "Beta" {
                    constraint_mut(&mut group)
                        .set_alpha(1.0)
                        .expect("valid alpha");
                }
                let error = initialize(path, &mut group).expect_err("missing tuning parameter");
                assert!(error.contains(missing), "{path:?}: {error}");
            }
        }
    }

    /// Upstream `test_constraintEffector_gainInitialization`: both paths
    /// derive the unspecified gains from alpha 3 and beta 2.
    #[test]
    fn gains_are_derived_from_the_tuning_parameters() {
        for path in [Path::Reset, Path::Hub] {
            let mut group = group_with_constraint();
            constraint_mut(&mut group)
                .set_alpha(3.0)
                .expect("valid alpha");
            constraint_mut(&mut group)
                .set_beta(2.0)
                .expect("valid beta");
            initialize(path, &mut group).expect("valid configuration");
            let constraint = constraint_mut(&mut group);
            assert_eq!(gains(constraint), [9.0, 4.0, 9.0, 4.0], "{path:?}");
        }
    }

    /// Upstream `test_retuning_refreshes_only_derived_gains`: gains set
    /// explicitly, before or after initialization and even to the values they
    /// would derive, stay fixed when alpha and beta are retuned; the others
    /// follow at every reset.
    #[test]
    fn retuning_refreshes_only_derived_gains() {
        type Setter =
            fn(&mut ConstraintDynamicEffector, f64) -> Result<(), ConstraintEffectorError>;
        let setters: [Setter; 4] = [
            ConstraintDynamicEffector::set_k_d,
            ConstraintDynamicEffector::set_c_d,
            ConstraintDynamicEffector::set_k_a,
            ConstraintDynamicEffector::set_c_a,
        ];
        let initial = [9.0, 4.0, 9.0, 4.0];
        let retuned = [25.0, 14.0, 25.0, 14.0];
        for path in [Path::Reset, Path::Hub] {
            for explicit_mask in 0..16 {
                for overrides_before in [true, false] {
                    let mut group = group_with_constraint();
                    constraint_mut(&mut group)
                        .set_alpha(3.0)
                        .expect("valid alpha");
                    constraint_mut(&mut group)
                        .set_beta(2.0)
                        .expect("valid beta");
                    let set_overrides = |group: &mut SpacecraftGroup| {
                        for (index, setter) in setters.iter().enumerate() {
                            if explicit_mask & (1 << index) != 0 {
                                setter(constraint_mut(group), initial[index]).expect("valid gain");
                            }
                        }
                    };
                    if overrides_before {
                        set_overrides(&mut group);
                    }
                    initialize(path, &mut group).expect("valid configuration");
                    if !overrides_before {
                        set_overrides(&mut group);
                    }
                    let constraint = constraint_mut(&mut group);
                    constraint.set_alpha(5.0).expect("valid alpha");
                    constraint.set_beta(7.0).expect("valid beta");
                    for _ in 0..3 {
                        constraint.reset().expect("valid configuration");
                        let expected: Vec<f64> = (0..4)
                            .map(|index| {
                                if explicit_mask & (1 << index) != 0 {
                                    initial[index]
                                } else {
                                    retuned[index]
                                }
                            })
                            .collect();
                        assert_eq!(
                            gains(constraint).to_vec(),
                            expected,
                            "{path:?} {explicit_mask}"
                        );
                    }
                }
            }
        }
    }

    /// Upstream `test_invalid_gain_setters_preserve_derived_status`: a
    /// rejected setter changes neither the value nor whether the gain is
    /// derived.
    #[test]
    fn invalid_gain_setters_preserve_derived_status() {
        type Setter =
            fn(&mut ConstraintDynamicEffector, f64) -> Result<(), ConstraintEffectorError>;
        type Getter = fn(&ConstraintDynamicEffector) -> f64;
        let fields: [(&str, Setter, Getter); 6] = [
            (
                "Alpha",
                ConstraintDynamicEffector::set_alpha,
                ConstraintDynamicEffector::alpha,
            ),
            (
                "Beta",
                ConstraintDynamicEffector::set_beta,
                ConstraintDynamicEffector::beta,
            ),
            (
                "K_d",
                ConstraintDynamicEffector::set_k_d,
                ConstraintDynamicEffector::k_d,
            ),
            (
                "C_d",
                ConstraintDynamicEffector::set_c_d,
                ConstraintDynamicEffector::c_d,
            ),
            (
                "K_a",
                ConstraintDynamicEffector::set_k_a,
                ConstraintDynamicEffector::k_a,
            ),
            (
                "C_a",
                ConstraintDynamicEffector::set_c_a,
                ConstraintDynamicEffector::c_a,
            ),
        ];
        for (name, setter, getter) in fields {
            for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -1.0] {
                let mut constraint = ConstraintDynamicEffector::new(
                    ConstraintDynamicEffectorConfig::new("constraint"),
                );
                constraint.set_alpha(3.0).expect("valid alpha");
                constraint.set_beta(2.0).expect("valid beta");
                constraint.reset().expect("valid configuration");
                let prior = getter(&constraint);
                let error = setter(&mut constraint, bad).expect_err("invalid value");
                assert!(
                    matches!(error, ConstraintEffectorError::InvalidParameter { .. }),
                    "{name}"
                );
                assert_eq!(getter(&constraint), prior, "{name} {bad}");
                constraint.set_alpha(5.0).expect("valid alpha");
                constraint.set_beta(7.0).expect("valid beta");
                constraint.reset().expect("valid configuration");
                assert_eq!(gains(&constraint), [25.0, 14.0, 25.0, 14.0], "{name} {bad}");
            }
        }
    }

    /// Upstream `test_derived_gain_overflow_preserves_prior_gains`: retuning
    /// to an alpha or beta whose derived gain overflows fails before any gain
    /// is replaced.
    #[test]
    fn derived_gain_overflow_preserves_prior_gains() {
        for path in [Path::Reset, Path::Hub] {
            for field in ["Alpha", "Beta"] {
                let mut group = group_with_constraint();
                let constraint = constraint_mut(&mut group);
                constraint.set_alpha(3.0).expect("valid alpha");
                constraint.set_beta(2.0).expect("valid beta");
                constraint.reset().expect("valid configuration");
                if field == "Alpha" {
                    constraint.set_alpha(f64::MAX).expect("finite alpha");
                } else {
                    constraint.set_beta(f64::MAX).expect("finite beta");
                }
                let error = initialize(path, &mut group).expect_err("overflowing derived gain");
                assert!(
                    error.contains("finite derived gains"),
                    "{path:?} {field}: {error}"
                );
                assert_eq!(
                    gains(constraint_mut(&mut group)),
                    [9.0, 4.0, 9.0, 4.0],
                    "{path:?} {field}"
                );
            }
        }
    }

    /// Upstream `test_partial_explicit_gains_preserve_zero_unspecified_gains`:
    /// one explicit gain needs no tuning parameters, the others staying zero.
    #[test]
    fn partial_explicit_gains_preserve_zero_unspecified_gains() {
        let mut constraint =
            ConstraintDynamicEffector::new(ConstraintDynamicEffectorConfig::new("constraint"));
        constraint.set_k_d(3.0).expect("valid gain");
        constraint.reset().expect("partial explicit configuration");
        assert_eq!(gains(&constraint), [3.0, 0.0, 0.0, 0.0]);
        constraint.set_alpha(5.0).expect("valid alpha");
        constraint.set_beta(7.0).expect("valid beta");
        constraint.reset().expect("valid configuration");
        assert_eq!(gains(&constraint), [3.0, 14.0, 25.0, 14.0]);
    }

    /// Upstream `test_fully_explicit_gains_do_not_derive_unused_tuning_parameters`.
    #[test]
    fn fully_explicit_gains_do_not_derive_unused_tuning_parameters() {
        let mut constraint =
            ConstraintDynamicEffector::new(ConstraintDynamicEffectorConfig::new("constraint"));
        constraint.set_alpha(f64::MAX).expect("finite alpha");
        constraint.set_beta(f64::MAX).expect("finite beta");
        constraint.set_k_d(3.0).expect("valid gain");
        constraint.set_c_d(4.0).expect("valid gain");
        constraint.set_k_a(5.0).expect("valid gain");
        constraint.set_c_a(6.0).expect("valid gain");
        constraint.reset().expect("valid configuration");
        assert_eq!(gains(&constraint), [3.0, 4.0, 5.0, 6.0]);
    }

    /// A constraint must join two different spacecraft of its group.
    #[test]
    #[should_panic(expected = "connects spacecraft 1 and 2 of a group of 2")]
    fn a_constraint_outside_its_group_is_rejected() {
        let mut constraint = tuned(ConstraintDynamicEffector::new(
            ConstraintDynamicEffectorConfig {
                spacecraft: [1, 2],
                ..ConstraintDynamicEffectorConfig::new("constraint")
            },
        ));
        constraint.init(2);
    }
}
