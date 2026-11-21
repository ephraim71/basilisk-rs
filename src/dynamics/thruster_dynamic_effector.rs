//! A thruster with on and off ramps: port of upstream Basilisk
//! `simulation/dynamics/Thrusters/thrusterDynamicEffector`
//! (`ThrusterDynamicEffector`), one effector per thruster.
//!
//! The thruster reads its [`ThrusterCommandMsg`] before each integration step
//! (see [`ThrusterDynamicEffector::pre_integration`] for when a command
//! re-arms it) and, as upstream `computeForceTorque`, decides at every
//! integrator stage whether it is still firing
//! ([`DynamicEffector::begin_stage`]). It publishes its delivered thrust as a
//! [`ThrusterOutputMsg`] (upstream `thrusterOutMsgs`) before each step, and
//! the propellant it consumes as a [`MassFlowRateMsg`] at every stage, which
//! a [`FuelTank`](super::fuel_tank::FuelTank) it is attached to
//! ([`FuelTank::add_thruster_set`](super::fuel_tank::FuelTank::add_thruster_set),
//! upstream `addThrusterSet`) drains. The tank in turn sets the propellant
//! mass of the blow-down model through
//! [`ThrusterDynamicEffector::fuel_tank_in_msg`].

use nalgebra::Vector3;
use std::any::Any;

use super::fuel_tank::TankFedThruster;
use crate::messages::{
    FuelTankMsg, Input, MassFlowRateMsg, Output, SpacecraftStateMsg, ThrusterCommandMsg,
    ThrusterOutputMsg,
};
use crate::spacecraft::{DynamicEffector, EffectorOutput};
use crate::utilities::astro_constants::EARTH_GRAV;

#[derive(Clone, Debug)]
pub struct ThrusterRampPoint {
    pub time_delta_s: f64,
    pub thrust_factor: f64,
    pub isp_factor: f64,
}

#[derive(Clone, Debug)]
pub struct ThrusterDynamicEffectorConfig {
    pub name: String,
    pub position_m: Vector3<f64>,
    pub direction_body: Vector3<f64>,
    pub area_nozzle_m2: f64,
    pub max_thrust_n: f64,
    pub steady_isp_s: f64,
    pub min_on_time_s: f64,
    pub max_swirl_torque_nm: f64,
    pub thrust_on_ramp: Vec<ThrusterRampPoint>,
    pub thrust_off_ramp: Vec<ThrusterRampPoint>,
    pub thrust_blow_down_coeff: Vec<f64>,
    pub isp_blow_down_coeff: Vec<f64>,
    pub fuel_mass_kg: Option<f64>,
    pub thrust_magnitude_dispersion: f64,
    pub update_only: bool,
}

impl ThrusterDynamicEffectorConfig {
    pub fn simple(
        name: impl Into<String>,
        position_m: Vector3<f64>,
        direction_body: Vector3<f64>,
        max_thrust_n: f64,
    ) -> Self {
        Self {
            name: name.into(),
            position_m,
            direction_body,
            area_nozzle_m2: 0.0,
            max_thrust_n,
            steady_isp_s: 300.0,
            min_on_time_s: 0.0,
            max_swirl_torque_nm: 0.0,
            thrust_on_ramp: Vec::new(),
            thrust_off_ramp: Vec::new(),
            thrust_blow_down_coeff: Vec::new(),
            isp_blow_down_coeff: Vec::new(),
            fuel_mass_kg: None,
            thrust_magnitude_dispersion: 0.0,
            update_only: true,
        }
    }

    pub fn generic_pulsed(
        name: impl Into<String>,
        position_m: Vector3<f64>,
        direction_body: Vector3<f64>,
        max_thrust_n: f64,
        min_on_time_s: f64,
        steady_isp_s: f64,
    ) -> Self {
        let mut config = Self::simple(name, position_m, direction_body, max_thrust_n);
        config.min_on_time_s = min_on_time_s;
        config.steady_isp_s = steady_isp_s;
        config
    }
}

#[derive(Clone, Debug)]
pub struct ThrusterOperation {
    pub fire_counter: u64,
    pub thrust_factor: f64,
    pub isp_factor: f64,
    pub thrust_blow_down_factor: f64,
    pub isp_blow_down_factor: f64,
    pub thrust_on_cmd_s: f64,
    pub total_on_time_s: f64,
    pub thrust_on_ramp_time_s: f64,
    pub thrust_on_steady_time_s: f64,
    pub thrust_off_ramp_time_s: f64,
    pub thruster_start_time_s: f64,
    pub previous_iter_time_s: f64,
}

impl Default for ThrusterOperation {
    fn default() -> Self {
        Self {
            fire_counter: 0,
            thrust_factor: 0.0,
            isp_factor: 1.0,
            thrust_blow_down_factor: 1.0,
            isp_blow_down_factor: 1.0,
            thrust_on_cmd_s: 0.0,
            total_on_time_s: 0.0,
            thrust_on_ramp_time_s: 0.0,
            thrust_on_steady_time_s: 0.0,
            thrust_off_ramp_time_s: 0.0,
            thruster_start_time_s: 0.0,
            previous_iter_time_s: 0.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ThrusterDynamicEffector {
    pub config: ThrusterDynamicEffectorConfig,
    pub command_in: Input<ThrusterCommandMsg>,
    /// Upstream `fuelMass`, set by the `FuelTank` the thruster is attached
    /// to: when connected, the blow-down model reads the tank's
    /// [`FuelTankMsg::fuel_mass_kg`] instead of
    /// [`ThrusterDynamicEffectorConfig::fuel_mass_kg`].
    pub fuel_tank_in_msg: Input<FuelTankMsg>,
    /// Upstream `thrusterOutMsgs` entry: the delivered thrust, written before
    /// each integration step.
    pub thruster_out_msg: Output<ThrusterOutputMsg>,
    /// Upstream `mDotTotal` (`stateDerivContribution(0)`): the propellant the
    /// thruster consumes, written before each step and at every integrator
    /// stage.
    pub mass_flow_out_msg: Output<MassFlowRateMsg>,
    pub operation: ThrusterOperation,
    last_command: ThrusterCommandMsg,
    /// Upstream `prevFireTime` \[s\]: time of the last firing update.
    previous_fire_time_s: f64,
}

impl ThrusterDynamicEffector {
    pub fn new(config: ThrusterDynamicEffectorConfig) -> Self {
        Self {
            config,
            command_in: Input::default(),
            fuel_tank_in_msg: Input::default(),
            thruster_out_msg: Output::default(),
            mass_flow_out_msg: Output::default(),
            operation: ThrusterOperation::default(),
            last_command: ThrusterCommandMsg::default(),
            previous_fire_time_s: 0.0,
        }
    }

    /// Upstream re-arms the thruster on every new write of its command (a
    /// changed header `timeWritten`), so a producer repeating an on-time
    /// restarts the firing each period. Ports carry no header: a command is
    /// new when its `command_time_nanos` or its on-time differs from the last
    /// one taken, so stamped writes re-arm every period and an unstamped,
    /// hand-written command still takes effect when its value changes.
    ///
    /// Then, as upstream `computeForceTorque` at the step's first stage,
    /// updates the blow-down factors and the firing at the step start, and
    /// writes the thruster's outputs.
    pub fn pre_integration(&mut self, current_sim_nanos: u64) {
        let current_time_s = current_sim_nanos as f64 * 1.0e-9;
        let command = self.command_in.read();
        if command.command_time_nanos != self.last_command.command_time_nanos
            || (command.on_time_s - self.last_command.on_time_s).abs() > 1.0e-15
        {
            self.configure_thrust_request(command.on_time_s, current_time_s);
            self.last_command = command;
        }

        self.compute_blowdown_decay();
        self.update_firing(current_time_s);
        self.thruster_out_msg.write(self.thruster_output());
        self.write_mass_flow();
    }

    /// Upstream `computeForceTorque`'s firing update at `time_s`: the
    /// thruster keeps firing while its on-time has not expired, within a
    /// relative 1e-9 of the time since the last update, and otherwise shuts
    /// down along its off ramp.
    fn update_firing(&mut self, time_s: f64) {
        let dt_s = time_s - self.previous_fire_time_s;
        if (self.operation.thrust_on_cmd_s + self.operation.thruster_start_time_s - time_s)
            >= -dt_s * 10e-10
            && self.operation.thrust_on_cmd_s > 0.0
        {
            self.compute_thruster_fire(time_s);
        } else if self.operation.thrust_factor > 0.0 {
            self.compute_thruster_shut(time_s);
        }
        self.previous_fire_time_s = time_s;
    }

    fn write_mass_flow(&self) {
        self.mass_flow_out_msg.write(MassFlowRateMsg {
            mass_flow_rate_kgps: self.mass_flow_rate_kgps(),
        });
    }

    /// Thrust force \[N\] in body components, without the mass-depletion
    /// terms (upstream `SingleThrusterForce`).
    fn thrust_force_body_n(&self) -> Vector3<f64> {
        let thrust_mag_n = self.config.max_thrust_n
            * self.operation.thrust_factor
            * self.operation.thrust_blow_down_factor
            * (1.0 + self.config.thrust_magnitude_dispersion);
        thrust_mag_n * normalized_direction(self.config.direction_body)
    }

    /// Thrust torque about B \[N m\] in body components, swirl included and
    /// the mass-depletion terms not (upstream `SingleThrusterTorque`).
    fn thrust_torque_body_nm(&self, force_body_n: Vector3<f64>) -> Vector3<f64> {
        self.config.position_m.cross(&force_body_n)
            + self.operation.thrust_factor
                * self.operation.thrust_blow_down_factor
                * self.config.max_swirl_torque_nm
                * normalized_direction(self.config.direction_body)
    }

    /// Upstream `writeOutputMessages` for this thruster.
    pub fn thruster_output(&self) -> ThrusterOutputMsg {
        let force_body_n = self.thrust_force_body_n();
        ThrusterOutputMsg {
            max_thrust_n: self.config.max_thrust_n,
            thrust_factor: self.operation.thrust_factor,
            thrust_blow_down_factor: self.operation.thrust_blow_down_factor,
            isp_blow_down_factor: self.operation.isp_blow_down_factor,
            thrust_force_n: force_body_n.norm(),
            thrust_force_b_n: force_body_n,
            thrust_torque_pnt_b_b_nm: self.thrust_torque_body_nm(force_body_n),
            thruster_location_m: self.config.position_m,
            thruster_direction: self.config.direction_body,
        }
    }

    pub fn compute_output(&self, state: &crate::messages::SpacecraftStateMsg) -> EffectorOutput {
        let direction_body = normalized_direction(self.config.direction_body);
        let mut force_body_n = self.thrust_force_body_n();
        let mut torque_body_nm = self.thrust_torque_body_nm(force_body_n);

        if !self.config.update_only {
            let omega_body = state.omega_radps;
            let m_dot_nozzle = self.mass_flow_rate_kgps();
            force_body_n += 2.0 * m_dot_nozzle * omega_body.cross(&self.config.position_m);
            torque_body_nm += m_dot_nozzle
                * nozzle_mass_depletion_torque(
                    direction_body,
                    self.config.area_nozzle_m2,
                    omega_body,
                );
        }

        let attitude_b_to_i = state.body_to_inertial();

        EffectorOutput {
            force_inertial_n: attitude_b_to_i.transform_vector(&force_body_n),
            torque_body_nm,
        }
    }

    pub fn mass_flow_rate_kgps(&self) -> f64 {
        if self.config.steady_isp_s <= 0.0
            || self.operation.isp_factor <= 0.0
            || self.operation.isp_blow_down_factor <= 0.0
        {
            return 0.0;
        }

        self.config.max_thrust_n
            * self.operation.thrust_factor
            * self.operation.thrust_blow_down_factor
            * (1.0 + self.config.thrust_magnitude_dispersion)
            / (EARTH_GRAV
                * self.config.steady_isp_s
                * self.operation.isp_factor
                * self.operation.isp_blow_down_factor)
    }

    fn configure_thrust_request(&mut self, command_on_time_s: f64, current_time_s: f64) {
        if command_on_time_s >= self.config.min_on_time_s {
            self.operation.thrust_on_cmd_s = command_on_time_s;
            if self.operation.thrust_factor <= 0.0 {
                self.operation.fire_counter += 1;
            }
        } else {
            self.operation.thrust_on_cmd_s = if self.operation.thrust_factor > 0.0 {
                command_on_time_s
            } else {
                0.0
            };
        }

        self.operation.thruster_start_time_s = current_time_s;
        self.operation.previous_iter_time_s = current_time_s;
        self.operation.thrust_on_ramp_time_s = 0.0;
        self.operation.thrust_on_steady_time_s = 0.0;
        self.operation.thrust_off_ramp_time_s = 0.0;
    }

    /// Upstream `computeBlowDownDecay`, with the propellant mass of the tank
    /// feeding the thruster when one is attached.
    fn compute_blowdown_decay(&mut self) {
        let fuel_mass_kg = if self.fuel_tank_in_msg.is_connected() {
            Some(self.fuel_tank_in_msg.read().fuel_mass_kg)
        } else {
            self.config.fuel_mass_kg
        };
        if let Some(fuel_mass_kg) = fuel_mass_kg {
            if !self.config.thrust_blow_down_coeff.is_empty() {
                let thrust_blow_down =
                    evaluate_polynomial(&self.config.thrust_blow_down_coeff, fuel_mass_kg);
                self.operation.thrust_blow_down_factor =
                    (thrust_blow_down / self.config.max_thrust_n).clamp(0.0, 1.0);
            }

            if !self.config.isp_blow_down_coeff.is_empty() {
                let isp_blow_down =
                    evaluate_polynomial(&self.config.isp_blow_down_coeff, fuel_mass_kg);
                self.operation.isp_blow_down_factor =
                    (isp_blow_down / self.config.steady_isp_s).clamp(0.0, 1.0);
            }
        }
    }

    fn compute_thruster_fire(&mut self, current_time_s: f64) {
        if self.operation.thrust_on_ramp_time_s == 0.0 && !self.config.thrust_on_ramp.is_empty() {
            self.operation.thrust_on_ramp_time_s =
                self.thr_factor_to_time(&self.config.thrust_on_ramp, self.operation.thrust_factor);
        }
        let local_on_ramp = (current_time_s - self.operation.previous_iter_time_s)
            + self.operation.thrust_on_ramp_time_s;
        if let Some((thrust_factor, isp_factor)) =
            interpolate_ramp(&self.config.thrust_on_ramp, local_on_ramp, 0.0, 0.0)
        {
            self.operation.thrust_factor = thrust_factor;
            self.operation.isp_factor = isp_factor;
            self.operation.thrust_on_ramp_time_s = local_on_ramp.max(0.0);
            self.operation.total_on_time_s += current_time_s - self.operation.previous_iter_time_s;
            self.operation.previous_iter_time_s = current_time_s;
            return;
        }

        self.operation.thrust_on_steady_time_s +=
            current_time_s - self.operation.previous_iter_time_s;
        self.operation.total_on_time_s += current_time_s - self.operation.previous_iter_time_s;
        self.operation.previous_iter_time_s = current_time_s;
        self.operation.thrust_factor = 1.0;
        self.operation.isp_factor = 1.0;
        self.operation.thrust_off_ramp_time_s = 0.0;
    }

    fn compute_thruster_shut(&mut self, current_time_s: f64) {
        if self.operation.thrust_off_ramp_time_s == 0.0 && !self.config.thrust_off_ramp.is_empty() {
            self.operation.thrust_off_ramp_time_s =
                self.thr_factor_to_time(&self.config.thrust_off_ramp, self.operation.thrust_factor);
        }
        let local_off_ramp = (current_time_s - self.operation.previous_iter_time_s)
            + self.operation.thrust_off_ramp_time_s;
        if let Some((thrust_factor, isp_factor)) =
            interpolate_ramp(&self.config.thrust_off_ramp, local_off_ramp, 1.0, 1.0)
        {
            self.operation.thrust_factor = thrust_factor;
            self.operation.isp_factor = isp_factor;
            self.operation.thrust_off_ramp_time_s = local_off_ramp.max(0.0);
            self.operation.previous_iter_time_s = current_time_s;
            return;
        }

        self.operation.thrust_factor = 0.0;
        self.operation.isp_factor = 0.0;
        self.operation.thrust_on_ramp_time_s = 0.0;
        self.operation.previous_iter_time_s = current_time_s;
    }

    fn thr_factor_to_time(&self, ramp: &[ThrusterRampPoint], thrust_factor: f64) -> f64 {
        if ramp.is_empty() {
            return 0.0;
        }

        let ramp_direction =
            (ramp.last().expect("nonempty ramp").thrust_factor - thrust_factor).signum();
        let mut prev_valid_thr_factor = if ramp_direction < 0.0 { 1.0 } else { 0.0 };
        let mut prev_valid_delta = 0.0;

        for point in ramp {
            let point_check = if ramp_direction > 0.0 {
                point.thrust_factor <= thrust_factor
            } else {
                point.thrust_factor >= thrust_factor
            };
            if point_check {
                prev_valid_thr_factor = point.thrust_factor;
                prev_valid_delta = point.time_delta_s;
                continue;
            }

            if (point.thrust_factor - prev_valid_thr_factor).abs() < 1.0e-15 {
                return point.time_delta_s;
            }

            return (point.time_delta_s - prev_valid_delta)
                / (point.thrust_factor - prev_valid_thr_factor)
                * (thrust_factor - prev_valid_thr_factor)
                + prev_valid_delta;
        }

        ramp.last().expect("nonempty ramp").time_delta_s
    }
}

impl DynamicEffector for ThrusterDynamicEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn pre_integration(&mut self, current_sim_nanos: u64, _dt_seconds: f64) {
        ThrusterDynamicEffector::pre_integration(self, current_sim_nanos);
    }

    /// Upstream `computeForceTorque`'s firing update at the stage time, and
    /// the stage's propellant consumption for an attached tank.
    fn begin_stage(&mut self, stage_nanos: u64) {
        self.update_firing(stage_nanos as f64 * 1.0e-9);
        self.write_mass_flow();
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        ThrusterDynamicEffector::compute_output(self, state)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl TankFedThruster for ThrusterDynamicEffector {
    fn mass_flow_out_msg(&self) -> &Output<MassFlowRateMsg> {
        &self.mass_flow_out_msg
    }

    fn connect_fuel_tank(&mut self, fuel_tank_out_msg: &Output<FuelTankMsg>) {
        fuel_tank_out_msg.connect_to(&mut self.fuel_tank_in_msg);
    }
}

fn normalized_direction(direction_body: Vector3<f64>) -> Vector3<f64> {
    if direction_body.norm_squared() > 0.0 {
        direction_body.normalize()
    } else {
        Vector3::zeros()
    }
}

fn interpolate_ramp(
    ramp: &[ThrusterRampPoint],
    local_time_s: f64,
    default_thrust_factor: f64,
    default_isp_factor: f64,
) -> Option<(f64, f64)> {
    if ramp.is_empty() {
        return None;
    }

    let mut prev_valid_thr_factor = default_thrust_factor;
    let mut prev_valid_isp_factor = default_isp_factor;
    let mut prev_valid_delta = 0.0;
    for point in ramp {
        if local_time_s < point.time_delta_s {
            let slope_thrust = (point.thrust_factor - prev_valid_thr_factor)
                / (point.time_delta_s - prev_valid_delta);
            let slope_isp = (point.isp_factor - prev_valid_isp_factor)
                / (point.time_delta_s - prev_valid_delta);
            return Some((
                slope_thrust * (local_time_s - prev_valid_delta) + prev_valid_thr_factor,
                slope_isp * (local_time_s - prev_valid_delta) + prev_valid_isp_factor,
            ));
        }
        prev_valid_thr_factor = point.thrust_factor;
        prev_valid_isp_factor = point.isp_factor;
        prev_valid_delta = point.time_delta_s;
    }

    None
}

fn evaluate_polynomial(coeffs_descending: &[f64], x: f64) -> f64 {
    coeffs_descending
        .iter()
        .fold(0.0, |value, coefficient| value * x + coefficient)
}

/// Torque per unit mass flow of the propellant leaving the nozzle of a
/// thruster along `thrust_direction_body` while the body turns at
/// `omega_body_radps` (upstream's mass-depletion torque of
/// `computeForceTorque` and `calcForceTorqueOnBody`, before the factor
/// `mDotNozzle`).
pub(crate) fn nozzle_mass_depletion_torque(
    thrust_direction_body: Vector3<f64>,
    area_nozzle_m2: f64,
    omega_body_radps: Vector3<f64>,
) -> Vector3<f64> {
    let b_m1 = thrust_direction_body;
    let b_m2 = Vector3::new(-b_m1.y, b_m1.x, b_m1.z);
    let b_m3 = b_m1.cross(&b_m2);
    let b_mj = nalgebra::Matrix3::from_columns(&[b_m1, b_m2, b_m3]);
    let axes_weight_matrix = nalgebra::Matrix3::new(2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    let thrust_tilde = tilde(thrust_direction_body);

    (thrust_tilde * thrust_tilde.transpose()
        + area_nozzle_m2 / (4.0 * std::f64::consts::PI)
            * b_mj
            * axes_weight_matrix
            * b_mj.transpose())
        * omega_body_radps
}

fn tilde(vector: Vector3<f64>) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(
        0.0, -vector.z, vector.y, vector.z, 0.0, -vector.x, -vector.y, vector.x, 0.0,
    )
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use crate::messages::{FuelTankMsg, Output, SpacecraftStateMsg, ThrusterCommandMsg};
    use crate::spacecraft::DynamicEffector;

    use super::{ThrusterDynamicEffector, ThrusterDynamicEffectorConfig};

    /// Fire a simple thruster (no ramp) for 5 s starting at t=0 and return it ready for output.
    /// Config: max_thrust=1 N, Isp=226.7 s, position=[1.125, 0.5, 2.0] m, angles long=30° lat=15°.
    fn fired_thruster() -> ThrusterDynamicEffector {
        let long_rad = 30.0_f64.to_radians();
        let lat_rad = 15.0_f64.to_radians();
        let direction = Vector3::new(
            long_rad.cos() * lat_rad.cos(),
            long_rad.sin() * lat_rad.cos(),
            lat_rad.sin(),
        );
        let mut config = ThrusterDynamicEffectorConfig::simple(
            "thr",
            Vector3::new(1.125, 0.5, 2.0),
            direction,
            1.0,
        );
        config.steady_isp_s = 226.7;

        let mut thr = ThrusterDynamicEffector::new(config);
        let cmd = Output::new(ThrusterCommandMsg {
            on_time_s: 5.0,
            ..Default::default()
        });
        thr.command_in.connect(cmd.slot());
        thr.pre_integration(0); // fires immediately at t=0
        thr
    }

    fn identity_state() -> SpacecraftStateMsg {
        SpacecraftStateMsg {
            position_m: Vector3::zeros(),
            velocity_mps: Vector3::zeros(),
            sigma_bn: Vector3::zeros(),
            omega_radps: Vector3::zeros(),
            ..Default::default()
        }
    }

    #[test]
    fn full_thrust_force() {
        let thr = fired_thruster();
        let long_rad = 30.0_f64.to_radians();
        let lat_rad = 15.0_f64.to_radians();
        let expected_force = Vector3::new(
            long_rad.cos() * lat_rad.cos(),
            long_rad.sin() * lat_rad.cos(),
            lat_rad.sin(),
        ); // 1.0 N * normalised direction, identity attitude → inertial = body

        let out = thr.compute_output(&identity_state());
        assert!(
            (out.force_inertial_n - expected_force).norm() < 1e-3,
            "force mismatch: expected {expected_force:?}, got {:?}",
            out.force_inertial_n
        );
    }

    #[test]
    fn full_thrust_torque_from_moment_arm() {
        let thr = fired_thruster();
        let long_rad = 30.0_f64.to_radians();
        let lat_rad = 15.0_f64.to_radians();
        let force = Vector3::new(
            long_rad.cos() * lat_rad.cos(),
            long_rad.sin() * lat_rad.cos(),
            lat_rad.sin(),
        );
        let position = Vector3::new(1.125, 0.5, 2.0);
        let expected_torque = position.cross(&force);

        let out = thr.compute_output(&identity_state());
        assert!(
            (out.torque_body_nm - expected_torque).norm() < 1e-3,
            "torque mismatch: expected {expected_torque:?}, got {:?}",
            out.torque_body_nm
        );
    }

    /// Steps `thruster` every 0.1 s over `[start_s, stop_s)` and returns the
    /// times at which it was firing.
    fn firing_times(thruster: &mut ThrusterDynamicEffector, start_s: f64, stop_s: f64) -> Vec<f64> {
        let mut firing = Vec::new();
        let mut step = (start_s * 10.0).round() as u64;
        while (step as f64) < stop_s * 10.0 {
            thruster.pre_integration(step * 100_000_000);
            if thruster.operation.thrust_factor > 0.0 {
                firing.push(step as f64 / 10.0);
            }
            step += 1;
        }
        firing
    }

    fn simple_thruster(command: &Output<ThrusterCommandMsg>) -> ThrusterDynamicEffector {
        let mut thruster = ThrusterDynamicEffector::new(ThrusterDynamicEffectorConfig::simple(
            "thr",
            Vector3::zeros(),
            Vector3::x(),
            1.0,
        ));
        command.connect_to(&mut thruster.command_in);
        thruster
    }

    /// Upstream re-arms on every new write, so the same 0.5 s on-time issued
    /// each 1 s period fires 0.5 s in every period, not only the first.
    #[test]
    fn a_repeated_on_time_with_a_new_command_time_fires_again() {
        let command = Output::new(ThrusterCommandMsg::default());
        let mut thruster = simple_thruster(&command);
        for period in 0..3_u64 {
            command.write(ThrusterCommandMsg {
                on_time_s: 0.5,
                command_time_nanos: period * 1_000_000_000,
            });
            let start_s = period as f64;
            let firing = firing_times(&mut thruster, start_s, start_s + 1.0);
            let expected: Vec<f64> = (0..=5).map(|tenth| start_s + tenth as f64 / 10.0).collect();
            assert_eq!(firing.len(), expected.len(), "period {period}: {firing:?}");
            for (actual, expected) in firing.iter().zip(&expected) {
                assert!(
                    (actual - expected).abs() < 1e-12,
                    "period {period}: {firing:?}"
                );
            }
        }
        assert_eq!(thruster.operation.fire_counter, 3);
    }

    #[test]
    fn a_command_seen_before_does_not_re_arm() {
        let command = Output::new(ThrusterCommandMsg {
            on_time_s: 0.5,
            command_time_nanos: 0,
        });
        let mut thruster = simple_thruster(&command);
        assert!(!firing_times(&mut thruster, 0.0, 1.0).is_empty());
        // Nothing new was issued: the expired command stays expired.
        assert!(firing_times(&mut thruster, 1.0, 2.0).is_empty());
        assert_eq!(thruster.operation.fire_counter, 1);
    }

    /// A hand-written command that is never stamped still takes effect when
    /// its on-time changes.
    #[test]
    fn an_unstamped_command_re_arms_when_its_on_time_changes() {
        let command = Output::new(ThrusterCommandMsg::default());
        let mut thruster = simple_thruster(&command);
        assert!(firing_times(&mut thruster, 0.0, 1.0).is_empty());
        command.write(ThrusterCommandMsg {
            on_time_s: 0.3,
            ..Default::default()
        });
        assert_eq!(firing_times(&mut thruster, 1.0, 2.0).len(), 4);
    }

    #[test]
    fn mass_flow_rate_formula() {
        let thr = fired_thruster();
        let expected_mdot = 1.0 / (9.80665 * 226.7);

        let mdot = thr.mass_flow_rate_kgps();
        assert!(
            (mdot - expected_mdot).abs() < 1e-6,
            "mdot mismatch: expected {expected_mdot:.6e}, got {mdot:.6e}"
        );
    }

    /// As upstream `computeForceTorque`, the firing is decided at every
    /// integrator stage: the stage at the end of the on-time still fires
    /// (within upstream's relative 1e-9 of the time since the last update),
    /// a later one shuts the thruster down.
    #[test]
    fn firing_is_decided_at_every_stage() {
        let command = Output::new(ThrusterCommandMsg {
            on_time_s: 0.5,
            command_time_nanos: 0,
        });
        let mut thruster = simple_thruster(&command);
        // Read at t = 0, so the on-time ends at 0.5 s.
        thruster.pre_integration(0);
        thruster.pre_integration(400_000_000);
        for stage in [400_000_000, 450_000_000, 450_000_000, 500_000_000] {
            thruster.begin_stage(stage);
            assert_eq!(thruster.operation.thrust_factor, 1.0, "stage {stage}");
        }
        thruster.pre_integration(500_000_000);
        thruster.begin_stage(500_000_000);
        assert_eq!(thruster.operation.thrust_factor, 1.0);
        thruster.begin_stage(550_000_000);
        assert_eq!(thruster.operation.thrust_factor, 0.0);
        assert_eq!(thruster.mass_flow_out_msg.read().mass_flow_rate_kgps, 0.0);
    }

    /// Upstream `writeOutputMessages` and `computeStateContribution`: the
    /// delivered thrust and the propellant consumption.
    #[test]
    fn thruster_publishes_its_thrust_and_mass_flow() {
        let mut thruster = fired_thruster();
        thruster.config.max_swirl_torque_nm = 0.2;
        thruster.pre_integration(0);
        let output = thruster.thruster_out_msg.read();
        let direction = thruster.config.direction_body.normalize();
        assert_eq!(output.max_thrust_n, 1.0);
        assert_eq!(output.thrust_factor, 1.0);
        assert_eq!(
            (output.thrust_blow_down_factor, output.isp_blow_down_factor),
            (1.0, 1.0)
        );
        assert!((output.thrust_force_n - 1.0).abs() < 1e-15);
        assert!((output.thrust_force_b_n - direction).norm() < 1e-15);
        let expected_torque = thruster.config.position_m.cross(&direction) + 0.2 * direction;
        assert!((output.thrust_torque_pnt_b_b_nm - expected_torque).norm() < 1e-15);
        assert_eq!(output.thruster_location_m, thruster.config.position_m);
        assert_eq!(output.thruster_direction, thruster.config.direction_body);
        let mass_flow = thruster.mass_flow_out_msg.read().mass_flow_rate_kgps;
        assert!((mass_flow - 1.0 / (9.80665 * 226.7)).abs() < 1e-15);
    }

    /// A tank feeding the thruster sets the propellant mass of the blow-down
    /// model.
    #[test]
    fn blow_down_follows_the_fuel_tank() {
        let command = Output::new(ThrusterCommandMsg {
            on_time_s: 1.0,
            command_time_nanos: 0,
        });
        let mut thruster = simple_thruster(&command);
        thruster.config.max_thrust_n = 2.0;
        // Thrust 0.1 N per kg of propellant.
        thruster.config.thrust_blow_down_coeff = vec![0.1, 0.0];
        thruster.config.fuel_mass_kg = Some(15.0);
        let tank = Output::new(FuelTankMsg {
            fuel_mass_kg: 5.0,
            ..FuelTankMsg::default()
        });
        thruster.pre_integration(0);
        assert!((thruster.operation.thrust_blow_down_factor - 0.75).abs() < 1e-15);
        tank.connect_to(&mut thruster.fuel_tank_in_msg);
        thruster.pre_integration(0);
        assert!((thruster.operation.thrust_blow_down_factor - 0.25).abs() < 1e-15);
    }

    /// Without update-only, the propellant leaving the nozzle adds upstream's
    /// force `2 ṁ ω × r` to the thrust.
    #[test]
    fn mass_depletion_force_follows_the_body_rate() {
        let mut thruster = fired_thruster();
        thruster.config.update_only = false;
        let omega = Vector3::new(0.0, 0.0, 0.3);
        let state = SpacecraftStateMsg {
            omega_radps: omega,
            ..identity_state()
        };
        let m_dot = thruster.mass_flow_rate_kgps();
        let expected = thruster.config.direction_body.normalize()
            + 2.0 * m_dot * omega.cross(&thruster.config.position_m);
        let output = thruster.compute_output(&state);
        assert!((output.force_inertial_n - expected).norm() < 1e-15);
    }
}
