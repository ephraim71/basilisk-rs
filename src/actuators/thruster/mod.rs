use nalgebra::Vector3;
use std::any::Any;

use crate::messages::{Input, SpacecraftStateMsg, ThrusterCommandMsg};
use crate::spacecraft::{DynamicEffector, EffectorOutput};

#[derive(Clone, Debug)]
pub struct ThrusterRampPoint {
    pub time_delta_s: f64,
    pub thrust_factor: f64,
    pub isp_factor: f64,
}

#[derive(Clone, Debug)]
pub struct ThrusterConfig {
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

impl ThrusterConfig {
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
pub struct Thruster {
    pub config: ThrusterConfig,
    pub command_in: Input<ThrusterCommandMsg>,
    pub operation: ThrusterOperation,
    last_command_s: f64,
}

impl Thruster {
    pub fn new(config: ThrusterConfig) -> Self {
        Self {
            config,
            command_in: Input::default(),
            operation: ThrusterOperation::default(),
            last_command_s: 0.0,
        }
    }

    pub fn pre_integration(&mut self, current_sim_nanos: u64) {
        let current_time_s = current_sim_nanos as f64 * 1.0e-9;
        let command_on_time_s = self.command_in.read().on_time_s;
        if (command_on_time_s - self.last_command_s).abs() > 1.0e-15 {
            self.configure_thrust_request(command_on_time_s, current_time_s);
            self.last_command_s = command_on_time_s;
        }

        self.compute_blowdown_decay();

        if (self.operation.thrust_on_cmd_s + self.operation.thruster_start_time_s - current_time_s)
            >= 0.0
            && self.operation.thrust_on_cmd_s > 0.0
        {
            self.compute_thruster_fire(current_time_s);
        } else if self.operation.thrust_factor > 0.0 {
            self.compute_thruster_shut(current_time_s);
        }
    }

    pub fn compute_output(&self, state: &crate::messages::SpacecraftStateMsg) -> EffectorOutput {
        let direction_body = normalized_direction(self.config.direction_body);
        let thrust_mag_n = self.config.max_thrust_n
            * self.operation.thrust_factor
            * self.operation.thrust_blow_down_factor
            * (1.0 + self.config.thrust_magnitude_dispersion);
        let force_body_n = thrust_mag_n * direction_body;
        let swirl_torque_body_nm = self.operation.thrust_factor
            * self.operation.thrust_blow_down_factor
            * self.config.max_swirl_torque_nm
            * direction_body;

        let mut torque_body_nm = self.config.position_m.cross(&force_body_n) + swirl_torque_body_nm;

        if !self.config.update_only {
            let omega_body = state.omega_radps;
            let m_dot_nozzle = self.mass_flow_rate_kgps();
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
            / (9.80665
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

    fn compute_blowdown_decay(&mut self) {
        if let Some(fuel_mass_kg) = self.config.fuel_mass_kg {
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

impl DynamicEffector for Thruster {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn pre_integration(&mut self, current_sim_nanos: u64, _dt_seconds: f64) {
        Thruster::pre_integration(self, current_sim_nanos);
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        Thruster::compute_output(self, state)
    }

    fn as_any(&self) -> &dyn Any {
        self
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

fn nozzle_mass_depletion_torque(
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

    use crate::messages::{Output, SpacecraftStateMsg, ThrusterCommandMsg};

    use super::{Thruster, ThrusterConfig};

    /// Fire a simple thruster (no ramp) for 5 s starting at t=0 and return it ready for output.
    /// Config: max_thrust=1 N, Isp=226.7 s, position=[1.125, 0.5, 2.0] m, angles long=30° lat=15°.
    fn fired_thruster() -> Thruster {
        let long_rad = 30.0_f64.to_radians();
        let lat_rad = 15.0_f64.to_radians();
        let direction = Vector3::new(
            long_rad.cos() * lat_rad.cos(),
            long_rad.sin() * lat_rad.cos(),
            lat_rad.sin(),
        );
        let mut config =
            ThrusterConfig::simple("thr", Vector3::new(1.125, 0.5, 2.0), direction, 1.0);
        config.steady_isp_s = 226.7;

        let mut thr = Thruster::new(config);
        let cmd = Output::new(ThrusterCommandMsg { on_time_s: 5.0 });
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
}
