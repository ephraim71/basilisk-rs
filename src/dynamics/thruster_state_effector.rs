//! Thrusters whose thrust follows a first-order filter: port of upstream
//! Basilisk `simulation/dynamics/Thrusters/thrusterStateEffector`
//! (`ThrusterStateEffector`).
//!
//! Each thruster's thrust factor `κ` (upstream `kappa`) is an integrated
//! state with cutoff frequency `ω_c`: `κ̇ = ω_c (1 − κ)` while the thruster is
//! commanded on and `κ̇ = −ω_c κ` otherwise. The thrust is `κ F_max (1 + δ)`
//! along the thrust direction, with dispersion `δ`; the propellant flow is
//! `κ F_max / (g0 Isp)`.
//!
//! Commands arrive per thruster as [`ThrusterCommandMsg`]s, read before each
//! integration step: a command is new, and re-arms the thruster, when its
//! `command_time_nanos` or on-time differs from the last one taken (the
//! stand-in for upstream's header `timeWritten`, as in
//! [`ThrusterDynamicEffector::pre_integration`](super::thruster_dynamic_effector::ThrusterDynamicEffector::pre_integration)).
//! A thruster fires until the read time plus its on-time, checked at every
//! integrator stage ([`StateEffector::begin_stage`]).
//!
//! As upstream, the force and torque on the hub are evaluated once the step
//! is complete (upstream `calcForceTorqueOnBody` in `postIntegration`), from
//! the thrust factor of the step's last integrator stage, and held over the
//! next step; the force and torque before the first step use the initial
//! thrust factors.
//!
//! Outputs: one [`ThrusterOutputMsg`] per thruster (upstream
//! `thrusterOutMsgs`) after every step, and the total propellant flow as a
//! [`MassFlowRateMsg`] at every stage, which a
//! [`FuelTank`](super::fuel_tank::FuelTank) the effector is attached to
//! ([`FuelTank::add_thruster_set`](super::fuel_tank::FuelTank::add_thruster_set),
//! upstream `addThrusterSet`) drains. Upstream the tank reads the flow of the
//! previous stage's thrust factors; here it reads the current stage's.
//!
//! A thruster mounted on another body F (upstream `addThruster(config,
//! bodyStateMsg)`) takes that body's [`SpacecraftStateMsg`] in
//! [`StateThruster::attached_body_in_msg`]; its location and direction are
//! then given in F, and F's pose relative to the hub is refreshed before each
//! step from the body state and the hub state of the previous step.

use nalgebra::{Matrix3, Vector3};
use std::any::Any;

use super::fuel_tank::TankFedThruster;
use super::thruster_dynamic_effector::nozzle_mass_depletion_torque;
use crate::messages::{
    Input, MassFlowRateMsg, Output, SpacecraftStateMsg, ThrusterCommandMsg, ThrusterOutputMsg,
};
use crate::spacecraft::{BackSubMatrices, StateEffector};
use crate::utilities::astro_constants::EARTH_GRAV;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;

/// Configuration of one thruster of a [`ThrusterStateEffector`]: the parts
/// of upstream `THRSimConfig` this model reads, and its `kappaInit` entry.
#[derive(Clone, Debug, PartialEq)]
pub struct ThrusterStateEffectorConfig {
    /// Thruster name (upstream `label`).
    pub name: String,
    /// Upstream `thrLoc_B` \[m\]: thruster location, body components (or
    /// mounting-body components when attached to another body).
    pub position_m: Vector3<f64>,
    /// Upstream `thrDir_B`: unit thrust direction, same frame as
    /// `position_m`.
    pub direction_body: Vector3<f64>,
    /// Upstream `MaxThrust` \[N\]: steady-state thrust.
    pub max_thrust_n: f64,
    /// Upstream `steadyIsp` \[s\]: specific impulse.
    pub steady_isp_s: f64,
    /// Upstream `MinOnTime` \[s\]: shortest on-time that fires the thruster.
    pub min_on_time_s: f64,
    /// Upstream `cutoffFrequency` \[rad/s\]: first-order filter frequency
    /// (default 10, upstream's documented default).
    pub cutoff_frequency_radps: f64,
    /// Upstream `MaxSwirlTorque` \[N m\]: swirl torque at full thrust.
    pub max_swirl_torque_nm: f64,
    /// Upstream `thrusterMagDisp` [-]: fractional thrust dispersion.
    pub thrust_magnitude_dispersion: f64,
    /// Upstream `areaNozzle` \[m^2\]: nozzle exit area, for the
    /// mass-depletion torque.
    pub area_nozzle_m2: f64,
    /// Upstream `updateOnly` (default `true`): leave out the force and torque
    /// of the propellant leaving the nozzle.
    pub update_only: bool,
    /// Upstream `kappaInit` entry [-]: initial thrust factor, in `[0, 1]`
    /// (default 0).
    pub kappa_init: f64,
}

impl ThrusterStateEffectorConfig {
    /// A thruster with upstream's defaults for the remaining parameters.
    pub fn new(
        name: impl Into<String>,
        position_m: Vector3<f64>,
        direction_body: Vector3<f64>,
        max_thrust_n: f64,
        steady_isp_s: f64,
    ) -> Self {
        Self {
            name: name.into(),
            position_m,
            direction_body,
            max_thrust_n,
            steady_isp_s,
            min_on_time_s: 0.0,
            cutoff_frequency_radps: 10.0,
            max_swirl_torque_nm: 0.0,
            thrust_magnitude_dispersion: 0.0,
            area_nozzle_m2: 0.0,
            update_only: true,
            kappa_init: 0.0,
        }
    }

    /// Upstream `validateConfiguration`: the initial thrust factor is finite
    /// and in `[0, 1]`.
    pub fn validate(&self) -> Result<(), String> {
        if !self.kappa_init.is_finite() || !(0.0..=1.0).contains(&self.kappa_init) {
            return Err(format!(
                "kappa_init must be finite and between 0 and 1, got {}",
                self.kappa_init
            ));
        }
        Ok(())
    }
}

/// Operating state of one thruster (the parts of upstream `THROperation`
/// this model uses).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct StateThrusterOperation {
    /// Upstream `fireCounter`: number of firings started.
    pub fire_counter: u64,
    /// Upstream `ThrustFactor`: the thrust factor of the last integrator
    /// stage.
    pub thrust_factor: f64,
    /// Upstream `ThrustOnCmd` \[s\]: on-time of the command in force.
    pub thrust_on_cmd_s: f64,
    /// Upstream `ThrusterEndTime` \[s\]: when the command in force expires.
    pub thruster_end_time_s: f64,
    /// Upstream `opThrustForce_B` \[N\]: thrust force, body components.
    pub thrust_force_body_n: Vector3<f64>,
    /// Upstream `opThrustTorquePntB_B` \[N m\]: thrust torque about B, body
    /// components.
    pub thrust_torque_pnt_b_body_nm: Vector3<f64>,
}

/// Pose of a thruster's mounting body F relative to the hub (upstream
/// `BodyToHubInfo`); the identity for a thruster on the hub.
#[derive(Clone, Debug, PartialEq)]
struct BodyToHub {
    /// Upstream `r_FB_B` \[m\].
    r_fb_b_m: Vector3<f64>,
    /// Upstream `dcm_BF`.
    dcm_bf: Matrix3<f64>,
    /// Upstream `omega_FB_B` \[rad/s\].
    omega_fb_b_radps: Vector3<f64>,
}

impl Default for BodyToHub {
    fn default() -> Self {
        Self {
            r_fb_b_m: Vector3::zeros(),
            dcm_bf: Matrix3::identity(),
            omega_fb_b_radps: Vector3::zeros(),
        }
    }
}

/// One thruster of a [`ThrusterStateEffector`], with its own ports.
#[derive(Clone, Debug)]
pub struct StateThruster {
    pub config: ThrusterStateEffectorConfig,
    /// The thruster's on-time command (upstream `cmdsInMsg` entry).
    pub command_in_msg: Input<ThrusterCommandMsg>,
    /// Upstream `attachedBodyInMsgs` entry: (optional) state of the body the
    /// thruster is mounted on.
    pub attached_body_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `thrusterOutMsgs` entry, written after every step.
    pub thruster_out_msg: Output<ThrusterOutputMsg>,
    pub operation: StateThrusterOperation,
    body_to_hub: BodyToHub,
    last_command: ThrusterCommandMsg,
}

impl StateThruster {
    fn new(config: ThrusterStateEffectorConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!(
                "invalid ThrusterStateEffectorConfig '{}': {message}",
                config.name
            );
        }
        Self {
            operation: StateThrusterOperation {
                thrust_factor: config.kappa_init,
                ..StateThrusterOperation::default()
            },
            config,
            command_in_msg: Input::default(),
            attached_body_in_msg: Input::default(),
            thruster_out_msg: Output::default(),
            body_to_hub: BodyToHub::default(),
            last_command: ThrusterCommandMsg::default(),
        }
    }

    /// Upstream `ConfigureThrustRequests` for an on-time of `on_time_s` read
    /// at `read_time_s`.
    fn configure_thrust_request(&mut self, on_time_s: f64, read_time_s: f64) {
        let operation = &mut self.operation;
        if on_time_s >= self.config.min_on_time_s {
            operation.thrust_on_cmd_s = on_time_s;
            if operation.thrust_factor <= 0.0 {
                operation.fire_counter += 1;
            }
        } else {
            // A short command only shortens a firing in progress.
            operation.thrust_on_cmd_s = if operation.thrust_factor > 1e-5 {
                on_time_s
            } else {
                0.0
            };
        }
        operation.thruster_end_time_s = read_time_s + operation.thrust_on_cmd_s;
    }

    /// Upstream `UpdateThrusterProperties`: the mounting body's pose relative
    /// to the hub at state `hub`.
    fn update_body_to_hub(&mut self, hub: &SpacecraftStateMsg) {
        if !self.attached_body_in_msg.is_connected() {
            return;
        }
        let body = self.attached_body_in_msg.read();
        let dcm_bn = mrp_to_dcm(hub.sigma_bn);
        let dcm_fn = mrp_to_dcm(body.sigma_bn);
        let dcm_bf = dcm_bn * dcm_fn.transpose();
        self.body_to_hub = BodyToHub {
            r_fb_b_m: dcm_bn * (body.position_m - hub.position_m),
            dcm_bf,
            omega_fb_b_radps: dcm_bf * body.omega_radps - hub.omega_radps,
        };
    }

    /// Thrust direction and location in body components.
    fn direction_and_location_body(&self) -> (Vector3<f64>, Vector3<f64>) {
        let dcm_bf = self.body_to_hub.dcm_bf;
        (
            dcm_bf * self.config.direction_body,
            self.body_to_hub.r_fb_b_m + dcm_bf * self.config.position_m,
        )
    }

    /// The propellant flow \[kg/s\] at the current thrust factor (upstream
    /// `updateEffectorMassProps`, without the dispersion).
    fn mass_flow_rate_kgps(&self) -> f64 {
        let factor = self.operation.thrust_factor;
        if self.config.steady_isp_s * factor > 0.0 {
            self.config.max_thrust_n * factor / (EARTH_GRAV * self.config.steady_isp_s)
        } else {
            0.0
        }
    }

    /// Upstream `calcForceTorqueOnBody` for this thruster at hub rate
    /// `omega_bn_b_radps`: records its thrust force and torque and returns
    /// its force and torque on the hub, mass-depletion terms included.
    fn force_torque_on_body(
        &mut self,
        omega_bn_b_radps: Vector3<f64>,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let (direction, location) = self.direction_and_location_body();
        let factor = self.operation.thrust_factor;
        let thrust_n =
            self.config.max_thrust_n * factor * (1.0 + self.config.thrust_magnitude_dispersion);
        let thrust_force = thrust_n * direction;
        let thrust_torque =
            location.cross(&thrust_force) + factor * self.config.max_swirl_torque_nm * direction;
        let mut force = thrust_force;
        let mut torque = thrust_torque;
        if !self.config.update_only {
            let m_dot_nozzle = self.mass_flow_rate_kgps();
            let omega = self.body_to_hub.omega_fb_b_radps + omega_bn_b_radps;
            force += 2.0 * m_dot_nozzle * omega.cross(&location);
            torque += m_dot_nozzle
                * nozzle_mass_depletion_torque(direction, self.config.area_nozzle_m2, omega);
        }
        self.operation.thrust_force_body_n = thrust_force;
        self.operation.thrust_torque_pnt_b_body_nm = thrust_torque;
        (force, torque)
    }

    /// Upstream `writeOutputStateMessages` for this thruster; the blow-down
    /// factors stay zero, as upstream leaves them.
    fn thruster_output(&self) -> ThrusterOutputMsg {
        ThrusterOutputMsg {
            max_thrust_n: self.config.max_thrust_n,
            thrust_factor: self.operation.thrust_factor,
            thrust_force_n: self.operation.thrust_force_body_n.norm(),
            thrust_force_b_n: self.operation.thrust_force_body_n,
            thrust_torque_pnt_b_b_nm: self.operation.thrust_torque_pnt_b_body_nm,
            thruster_location_m: self.config.position_m,
            thruster_direction: self.config.direction_body,
            ..ThrusterOutputMsg::default()
        }
    }
}

/// First-order thruster set (upstream `ThrusterStateEffector`).
///
/// State layout: `[κ_1, …, κ_n]`, one thrust factor per thruster in the
/// order added.
#[derive(Clone, Debug)]
pub struct ThrusterStateEffector {
    pub name: String,
    /// Upstream `mDotTotal`: the propellant the thrusters consume \[kg/s\],
    /// written at every integrator stage.
    pub mass_flow_out_msg: Output<MassFlowRateMsg>,
    thrusters: Vec<StateThruster>,
    /// Upstream `kappaState` of the last loaded state.
    kappa: Vec<f64>,
    /// Integrator stage time \[s\] (upstream `integTime`).
    stage_time_s: f64,
    /// Whether a derivative evaluation ran since the outputs were written.
    stage_evaluated: bool,
    /// Upstream `forceOnBody_B` \[N\].
    force_on_body_b_n: Vector3<f64>,
    /// Upstream `torqueOnBodyPntB_B` \[N m\].
    torque_on_body_pnt_b_b_nm: Vector3<f64>,
    /// The hub state at the last output write.
    hub_state: SpacecraftStateMsg,
}

impl ThrusterStateEffector {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            mass_flow_out_msg: Output::default(),
            thrusters: Vec::new(),
            kappa: Vec::new(),
            stage_time_s: 0.0,
            stage_evaluated: false,
            force_on_body_b_n: Vector3::zeros(),
            torque_on_body_pnt_b_b_nm: Vector3::zeros(),
            hub_state: SpacecraftStateMsg::default(),
        }
    }

    /// Upstream `addThruster`: registers one thruster and returns its ports.
    ///
    /// # Panics
    ///
    /// If [`ThrusterStateEffectorConfig::validate`] fails.
    pub fn add_thruster(&mut self, config: ThrusterStateEffectorConfig) -> &mut StateThruster {
        self.kappa.push(config.kappa_init);
        self.thrusters.push(StateThruster::new(config));
        self.thrusters
            .last_mut()
            .expect("registered thruster should be available")
    }

    /// The thrusters in the order added.
    pub fn thrusters(&self) -> &[StateThruster] {
        &self.thrusters
    }

    /// Mutable thruster access for wiring and configuration.
    pub fn thrusters_mut(&mut self) -> &mut [StateThruster] {
        &mut self.thrusters
    }

    /// Upstream `forceOnBody_B` \[N\]: the force on the hub held over the
    /// current step.
    pub fn force_on_body_b_n(&self) -> Vector3<f64> {
        self.force_on_body_b_n
    }

    /// Upstream `torqueOnBodyPntB_B` \[N m\]: the torque about B held over the
    /// current step.
    pub fn torque_on_body_pnt_b_b_nm(&self) -> Vector3<f64> {
        self.torque_on_body_pnt_b_b_nm
    }

    /// Upstream `mDotTotal` \[kg/s\] at the current thrust factors.
    pub fn mass_flow_rate_kgps(&self) -> f64 {
        self.thrusters
            .iter()
            .map(StateThruster::mass_flow_rate_kgps)
            .sum()
    }

    fn write_mass_flow(&self) {
        self.mass_flow_out_msg.write(MassFlowRateMsg {
            mass_flow_rate_kgps: self.mass_flow_rate_kgps(),
        });
    }

    fn assert_state_length(&self, state: &[f64]) {
        assert_eq!(
            state.len(),
            self.thrusters.len(),
            "thruster state effector state length mismatch"
        );
    }
}

impl StateEffector for ThrusterStateEffector {
    fn name(&self) -> &str {
        &self.name
    }

    fn state_len(&self) -> usize {
        self.thrusters.len()
    }

    /// Upstream `registerStates`: each thruster's `kappa_init`.
    fn initial_state(&self) -> Vec<f64> {
        self.thrusters
            .iter()
            .map(|thruster| thruster.config.kappa_init)
            .collect()
    }

    fn load_state(&mut self, state: &[f64]) {
        self.assert_state_length(state);
        self.kappa.clear();
        self.kappa.extend_from_slice(state);
    }

    /// Upstream `UpdateState`: reads each thruster's command and refreshes the
    /// pose of the bodies thrusters are mounted on.
    fn pre_integration(&mut self, current_sim_nanos: u64, _dt_seconds: f64) {
        let current_time_s = current_sim_nanos as f64 * 1.0e-9;
        self.stage_time_s = current_time_s;
        for thruster in &mut self.thrusters {
            let command = thruster.command_in_msg.read();
            if command.command_time_nanos != thruster.last_command.command_time_nanos
                || (command.on_time_s - thruster.last_command.on_time_s).abs() > 1.0e-15
            {
                thruster.configure_thrust_request(command.on_time_s, current_time_s);
                thruster.last_command = command;
            }
            thruster.update_body_to_hub(&self.hub_state);
        }
    }

    /// The stage time and the stage's thrust factors (upstream sets
    /// `ThrustFactor` in `computeDerivatives`), and the stage's propellant
    /// flow.
    fn begin_stage(&mut self, stage_nanos: u64) {
        self.stage_time_s = stage_nanos as f64 * 1.0e-9;
        for (thruster, kappa) in self.thrusters.iter_mut().zip(&self.kappa) {
            thruster.operation.thrust_factor = *kappa;
        }
        self.stage_evaluated = true;
        self.write_mass_flow();
    }

    /// Upstream `updateContributions`: the held force and torque.
    fn update_contributions(
        &self,
        effector_state: &[f64],
        _body_omega_radps: Vector3<f64>,
        _gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        self.assert_state_length(effector_state);
        back_sub.vec_trans += self.force_on_body_b_n;
        back_sub.vec_rot += self.torque_on_body_pnt_b_b_nm;
    }

    /// Upstream `computeDerivatives`: each thrust factor relaxes towards 1
    /// while its command lasts at the stage time, towards 0 otherwise.
    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        _body_trans_accel_mps2: Vector3<f64>,
        _body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        self.assert_state_length(effector_state);
        self.thrusters
            .iter()
            .zip(effector_state)
            .map(|(thruster, kappa)| {
                let operation = &thruster.operation;
                let frequency = thruster.config.cutoff_frequency_radps;
                if (operation.thruster_end_time_s - self.stage_time_s) >= 0.0
                    && operation.thrust_on_cmd_s > 0.0
                {
                    (1.0 - kappa) * frequency
                } else {
                    -kappa * frequency
                }
            })
            .collect()
    }

    /// Upstream `calcForceTorqueOnBody` (after a step, or for the initial
    /// state before the first) and `writeOutputStateMessages`.
    fn write_outputs(&mut self, _current_sim_nanos: u64, hub_state: &SpacecraftStateMsg) {
        if !self.stage_evaluated {
            for (thruster, kappa) in self.thrusters.iter_mut().zip(&self.kappa) {
                thruster.operation.thrust_factor = *kappa;
            }
            self.write_mass_flow();
        }
        self.stage_evaluated = false;
        let mut force = Vector3::zeros();
        let mut torque = Vector3::zeros();
        for thruster in &mut self.thrusters {
            let (thruster_force, thruster_torque) =
                thruster.force_torque_on_body(hub_state.omega_radps);
            force += thruster_force;
            torque += thruster_torque;
            thruster.thruster_out_msg.write(thruster.thruster_output());
        }
        self.force_on_body_b_n = force;
        self.torque_on_body_pnt_b_b_nm = torque;
        self.hub_state = hub_state.clone();
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl TankFedThruster for ThrusterStateEffector {
    fn mass_flow_out_msg(&self) -> &Output<MassFlowRateMsg> {
        &self.mass_flow_out_msg
    }
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_ThrusterStateEffectorUnit.py` and
    //! `test_thrusterStateDataAccess.py`. The effector's tank feed is tested
    //! with the tank (`fuel_tank::tests`).

    use super::{ThrusterStateEffector, ThrusterStateEffectorConfig};
    use crate::dynamics::state_effector_test_support::{
        HubSetup, run_inspecting_every_step, spacecraft_with_effectors,
    };
    use crate::messages::{Output, SpacecraftStateMsg, ThrusterCommandMsg};
    use crate::spacecraft::{BackSubMatrices, StateEffector};
    use crate::utilities::rigid_body_kinematics::{add_mrp, dcm_to_mrp, mrp_to_dcm};
    use nalgebra::{Matrix3, Vector3};

    /// Upstream `thrDir_B` from longitude and latitude angles \[rad\].
    fn direction(long_rad: f64, lat_rad: f64) -> Vector3<f64> {
        Vector3::new(
            long_rad.cos() * lat_rad.cos(),
            long_rad.sin() * lat_rad.cos(),
            lat_rad.sin(),
        )
    }

    /// Upstream `unitThrusters`: one or two first-order thrusters on the
    /// hub (the second optionally on a body F) commanded at t = 0, run for
    /// 2 s at 10 ms, against the closed-form thrust factor
    /// `κ(t) = 1 + (κ0 − 1) e^(−ω t)` (on) or `κ0 e^(−ω t)` (off).
    #[allow(clippy::too_many_arguments)]
    fn unit_thrusters(
        thrust_number: usize,
        initial_condition: f64,
        duration_s: f64,
        long_angle_deg: f64,
        lat_angle_deg: f64,
        location: [f64; 3],
        swirl_torque_nm: f64,
        attach_body: bool,
    ) {
        let hub = HubSetup {
            mass_kg: 750.0,
            center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
            r_cn_n_m: Vector3::new(-4020338.690396649, 7490566.741852513, 5248299.211589362),
            v_cn_n_mps: Vector3::new(-5199.77710904224, -3436.681645356935, 1041.576797498721),
            sigma_bn: Vector3::new(0.1, 0.2, -0.3),
            omega_bn_b_radps: Vector3::new(0.001, -0.01, 0.03),
        };
        let g = 9.80665;
        let isp = 226.7;
        let long_rad = long_angle_deg.to_radians();
        let lat_rad = lat_angle_deg.to_radians();

        let mut thrusters = ThrusterStateEffector::new("ACSThrusterDynamics");
        let loc1 = Vector3::from(location);
        let dir1 = direction(long_rad, lat_rad);
        let mut config1 = ThrusterStateEffectorConfig::new("thruster1", loc1, dir1, 10.0, isp);
        config1.min_on_time_s = 0.006;
        config1.cutoff_frequency_radps = 5.0;
        config1.max_swirl_torque_nm = swirl_torque_nm;
        config1.kappa_init = initial_condition;
        let command1 = Output::new(ThrusterCommandMsg {
            on_time_s: duration_s,
            command_time_nanos: 0,
        });
        command1.connect_to(&mut thrusters.add_thruster(config1).command_in_msg);

        let dcm_bf = Matrix3::new(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0);
        let r_fb_b = Vector3::new(0.0, 0.0, 1.0);
        let body_out = Output::new(SpacecraftStateMsg::default());
        let mut loc2 = Vector3::new(1.0, 0.0, 0.0);
        let mut dir2 = direction(
            long_rad + std::f64::consts::FRAC_PI_4,
            lat_rad - std::f64::consts::FRAC_PI_4,
        );
        if thrust_number == 2 {
            let mut config2 = ThrusterStateEffectorConfig::new("thruster2", loc2, dir2, 20.0, isp);
            config2.min_on_time_s = 0.006;
            config2.cutoff_frequency_radps = 2.0;
            let thruster2 = thrusters.add_thruster(config2);
            let command2 = Output::new(ThrusterCommandMsg {
                on_time_s: 2.0,
                command_time_nanos: 0,
            });
            command2.connect_to(&mut thruster2.command_in_msg);
            if attach_body {
                body_out.connect_to(&mut thruster2.attached_body_in_msg);
                dir2 = dcm_bf * dir2;
                loc2 = dcm_bf * loc2 + r_fb_b;
            }
        }

        let mut spacecraft = spacecraft_with_effectors(&hub, 10_000_000, vec![Box::new(thrusters)]);
        let state_out = spacecraft.state_out.clone();
        let mut samples: Vec<(Vector3<f64>, Vector3<f64>, f64)> = Vec::new();
        run_inspecting_every_step(&mut spacecraft, 200, |spacecraft| {
            let thrusters = spacecraft.state_effectors[0]
                .as_any()
                .downcast_ref::<ThrusterStateEffector>()
                .expect("thruster state effector");
            samples.push((
                thrusters.force_on_body_b_n(),
                thrusters.torque_on_body_pnt_b_b_nm(),
                thrusters.mass_flow_out_msg.read().mass_flow_rate_kgps,
            ));
            // Upstream `attachedBodyModule`: the body F, fixed to the hub.
            let hub_state = state_out.read();
            let dcm_fb = dcm_bf.transpose();
            let dcm_bn = mrp_to_dcm(hub_state.sigma_bn);
            body_out.write(SpacecraftStateMsg {
                position_m: hub_state.position_m + dcm_bn.transpose() * r_fb_b,
                sigma_bn: add_mrp(hub_state.sigma_bn, dcm_to_mrp(dcm_fb)),
                omega_radps: dcm_fb * hub_state.omega_radps,
                ..SpacecraftStateMsg::default()
            });
        });

        for (index, (force, torque, mass_flow)) in samples.iter().enumerate() {
            let t = index as f64 * 0.01;
            let factor1 = if duration_s == 0.0 {
                initial_condition * (-5.0 * t).exp()
            } else {
                1.0 + (initial_condition - 1.0) * (-5.0 * t).exp()
            };
            let force1 = factor1 * 10.0 * dir1;
            let mut true_force = force1;
            let mut true_torque = loc1.cross(&force1) + factor1 * swirl_torque_nm * dir1;
            let mut true_mass_flow = 10.0 * factor1 / (g * isp);
            if thrust_number == 2 {
                let factor2 = 1.0 - (-2.0 * t).exp();
                let force2 = factor2 * 20.0 * dir2;
                true_force += force2;
                true_torque += loc2.cross(&force2);
                true_mass_flow = (10.0 * factor1 + 20.0 * factor2) / (g * isp);
            }
            // Upstream compares from the second sample on for the force and
            // torque, and the first three samples for the mass flow.
            if index > 0 {
                for axis in 0..3 {
                    assert!(
                        (force[axis] - true_force[axis]).abs() <= 1e-3,
                        "t = {t}: force {force:?} vs {true_force:?}"
                    );
                    assert!(
                        (torque[axis] - true_torque[axis]).abs() <= 1e-3,
                        "t = {t}: torque {torque:?} vs {true_torque:?}"
                    );
                }
            }
            if index < 3 {
                assert!(
                    (mass_flow - true_mass_flow).abs() <= 1e-6,
                    "t = {t}: mass flow {mass_flow} vs {true_mass_flow}"
                );
            }
        }
    }

    const LOCATION_1: [f64; 3] = [1.125, 0.5, 2.0];
    const LOCATION_2: [f64; 3] = [-1.125, 0.5, -2.0];

    #[test]
    fn one_thruster_firing() {
        unit_thrusters(1, 0.0, 2.0, 30.0, 15.0, LOCATION_1, 0.0, false);
    }

    #[test]
    fn one_thruster_shutting_down() {
        unit_thrusters(1, 1.0, 0.0, 30.0, 15.0, LOCATION_1, 0.0, false);
    }

    #[test]
    fn one_thruster_firing_at_another_location() {
        unit_thrusters(1, 0.0, 2.0, 60.0, -15.0, LOCATION_2, 0.0, false);
    }

    #[test]
    fn one_thruster_shutting_down_at_another_location() {
        unit_thrusters(1, 1.0, 0.0, 60.0, -15.0, LOCATION_2, 0.0, false);
    }

    #[test]
    fn two_thrusters_firing() {
        unit_thrusters(2, 0.0, 2.0, 30.0, 15.0, LOCATION_1, 0.0, false);
    }

    #[test]
    fn two_thrusters_one_shutting_down() {
        unit_thrusters(2, 1.0, 0.0, 30.0, 15.0, LOCATION_1, 0.0, false);
    }

    #[test]
    fn two_thrusters_with_swirl_torque() {
        unit_thrusters(2, 0.0, 2.0, 30.0, 15.0, LOCATION_1, 2.0, false);
    }

    #[test]
    fn two_thrusters_one_on_an_attached_body() {
        unit_thrusters(2, 0.0, 2.0, 30.0, 15.0, LOCATION_1, 0.0, true);
    }

    /// Upstream `test_thrusterData_is_iterable_list`: the thrusters keep
    /// their order and their configurations stay editable.
    #[test]
    fn thrusters_are_listed_in_order_and_editable() {
        let mut thrusters = ThrusterStateEffector::new("thrusters");
        let max_thrusts = [10.0, 20.0];
        for max_thrust in max_thrusts {
            thrusters.add_thruster(ThrusterStateEffectorConfig::new(
                "thruster",
                Vector3::x(),
                Vector3::x(),
                max_thrust,
                0.0,
            ));
        }
        let listed: Vec<f64> = thrusters
            .thrusters()
            .iter()
            .map(|thruster| thruster.config.max_thrust_n)
            .collect();
        assert_eq!(listed, max_thrusts);
        thrusters.thrusters_mut()[0].config.max_thrust_n = 99.0;
        assert_eq!(thrusters.thrusters()[0].config.max_thrust_n, 99.0);
        assert_eq!(thrusters.state_len(), 2);
    }

    /// Upstream `validateConfiguration`: `kappaInit` in `[0, 1]`.
    #[test]
    fn initial_thrust_factor_is_validated() {
        let mut config =
            ThrusterStateEffectorConfig::new("thruster", Vector3::x(), Vector3::x(), 1.0, 200.0);
        assert_eq!(config.validate(), Ok(()));
        for kappa in [-0.1, 1.1, f64::NAN] {
            config.kappa_init = kappa;
            assert!(config.validate().is_err(), "kappa {kappa}");
        }
    }

    #[test]
    #[should_panic(expected = "kappa_init must be finite and between 0 and 1")]
    fn adding_a_thruster_with_an_invalid_initial_factor_panics() {
        let mut config =
            ThrusterStateEffectorConfig::new("thruster", Vector3::x(), Vector3::x(), 1.0, 200.0);
        config.kappa_init = 2.0;
        ThrusterStateEffector::new("thrusters").add_thruster(config);
    }

    fn single_thruster(min_on_time_s: f64) -> (ThrusterStateEffector, Output<ThrusterCommandMsg>) {
        let mut thrusters = ThrusterStateEffector::new("thrusters");
        let mut config = ThrusterStateEffectorConfig::new(
            "thruster",
            Vector3::zeros(),
            Vector3::x(),
            1.0,
            200.0,
        );
        config.min_on_time_s = min_on_time_s;
        let command = Output::new(ThrusterCommandMsg::default());
        command.connect_to(&mut thrusters.add_thruster(config).command_in_msg);
        (thrusters, command)
    }

    fn kappa_rate(thrusters: &mut ThrusterStateEffector, stage_nanos: u64, kappa: f64) -> f64 {
        thrusters.load_state(&[kappa]);
        thrusters.begin_stage(stage_nanos);
        thrusters.compute_derivatives(&[kappa], Vector3::zeros(), Vector3::zeros())[0]
    }

    /// A command fires until its read time plus its on-time, checked at the
    /// stage time; a repeated on-time with a new stamp re-arms the thruster.
    #[test]
    fn commands_fire_until_their_end_time_and_re_arm_when_new() {
        let (mut thrusters, command) = single_thruster(0.0);
        command.write(ThrusterCommandMsg {
            on_time_s: 0.5,
            command_time_nanos: 0,
        });
        thrusters.pre_integration(0, 0.1);
        assert_eq!(kappa_rate(&mut thrusters, 500_000_000, 0.0), 10.0);
        assert_eq!(kappa_rate(&mut thrusters, 600_000_000, 1.0), -10.0);
        assert_eq!(thrusters.thrusters()[0].operation.fire_counter, 1);

        // The same command, not re-issued: still expired.
        thrusters.pre_integration(1_000_000_000, 0.1);
        assert_eq!(kappa_rate(&mut thrusters, 1_000_000_000, 0.0), 0.0);

        // Re-issued with a new stamp: fires for another 0.5 s.
        command.write(ThrusterCommandMsg {
            on_time_s: 0.5,
            command_time_nanos: 1_000_000_000,
        });
        thrusters.pre_integration(1_000_000_000, 0.1);
        assert_eq!(kappa_rate(&mut thrusters, 1_400_000_000, 0.0), 10.0);
        assert_eq!(thrusters.thrusters()[0].operation.fire_counter, 2);
    }

    /// Upstream's minimum on-time: a short command starts nothing, but cuts a
    /// firing in progress short.
    #[test]
    fn short_commands_only_shorten_a_firing() {
        let (mut thrusters, command) = single_thruster(0.1);
        command.write(ThrusterCommandMsg {
            on_time_s: 0.05,
            command_time_nanos: 0,
        });
        thrusters.pre_integration(0, 0.01);
        assert_eq!(kappa_rate(&mut thrusters, 10_000_000, 0.0), 0.0);
        assert_eq!(thrusters.thrusters()[0].operation.fire_counter, 0);

        command.write(ThrusterCommandMsg {
            on_time_s: 1.0,
            command_time_nanos: 100_000_000,
        });
        thrusters.pre_integration(100_000_000, 0.01);
        assert!(kappa_rate(&mut thrusters, 200_000_000, 0.5) > 0.0);
        command.write(ThrusterCommandMsg {
            on_time_s: 0.05,
            command_time_nanos: 300_000_000,
        });
        thrusters.pre_integration(300_000_000, 0.01);
        assert!(kappa_rate(&mut thrusters, 340_000_000, 0.5) > 0.0);
        assert!(kappa_rate(&mut thrusters, 360_000_000, 0.5) < 0.0);
    }

    /// The force and torque on the hub are those of the last stage's thrust
    /// factor, held until the next step completes, and enter the
    /// back-substitution unchanged.
    #[test]
    fn held_force_and_torque_enter_the_back_substitution() {
        let mut thrusters = ThrusterStateEffector::new("thrusters");
        let mut config = ThrusterStateEffectorConfig::new(
            "thruster",
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::x(),
            2.0,
            200.0,
        );
        config.max_swirl_torque_nm = 0.5;
        config.kappa_init = 0.25;
        thrusters.add_thruster(config);
        thrusters.load_state(&[0.25]);
        thrusters.write_outputs(0, &SpacecraftStateMsg::default());
        assert_eq!(thrusters.force_on_body_b_n(), Vector3::new(0.5, 0.0, 0.0));
        assert_eq!(
            thrusters.torque_on_body_pnt_b_b_nm(),
            Vector3::new(0.125, 0.0, -0.5)
        );
        let output = thrusters.thrusters()[0].thruster_out_msg.read();
        assert_eq!(output.thrust_factor, 0.25);
        assert_eq!(output.thrust_force_n, 0.5);

        // A stage at a new thrust factor changes neither until the step is
        // complete.
        thrusters.load_state(&[0.75]);
        thrusters.begin_stage(10_000_000);
        let mut back_sub = BackSubMatrices::default();
        thrusters.update_contributions(&[0.75], Vector3::zeros(), Vector3::zeros(), &mut back_sub);
        assert_eq!(back_sub.vec_trans, Vector3::new(0.5, 0.0, 0.0));
        assert_eq!(back_sub.vec_rot, Vector3::new(0.125, 0.0, -0.5));
        assert!((thrusters.mass_flow_rate_kgps() - 1.5 / (9.80665 * 200.0)).abs() < 1e-15);
        thrusters.load_state(&[0.8]);
        thrusters.write_outputs(10_000_000, &SpacecraftStateMsg::default());
        assert_eq!(thrusters.force_on_body_b_n(), Vector3::new(1.5, 0.0, 0.0));
    }

    /// Without update-only, the propellant leaving the nozzle adds upstream's
    /// mass-depletion force `2 ṁ ω × r` and torque.
    #[test]
    fn mass_depletion_terms_follow_the_hub_rate() {
        let mut thrusters = ThrusterStateEffector::new("thrusters");
        let location = Vector3::new(0.0, 1.0, 0.0);
        let mut config =
            ThrusterStateEffectorConfig::new("thruster", location, Vector3::x(), 2.0, 200.0);
        config.update_only = false;
        config.area_nozzle_m2 = 0.1;
        config.kappa_init = 1.0;
        thrusters.add_thruster(config);
        thrusters.load_state(&[1.0]);
        let omega = Vector3::new(0.0, 0.0, 0.2);
        thrusters.write_outputs(
            0,
            &SpacecraftStateMsg {
                omega_radps: omega,
                ..SpacecraftStateMsg::default()
            },
        );
        let m_dot = 2.0 / (9.80665 * 200.0);
        let expected_force = Vector3::new(2.0, 0.0, 0.0) + 2.0 * m_dot * omega.cross(&location);
        assert!((thrusters.force_on_body_b_n() - expected_force).norm() < 1e-15);
        // The thruster's own output leaves the depletion terms out.
        let output = thrusters.thrusters()[0].thruster_out_msg.read();
        assert_eq!(output.thrust_force_b_n, Vector3::new(2.0, 0.0, 0.0));
    }
}
