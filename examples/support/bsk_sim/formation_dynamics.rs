//! Upstream `BskSim/models/BSK_FormationDynamics.py`: a chief and a deputy,
//! each the 750 kg hub with four HR16 wheels, about a point-mass Earth.
//!
//! | upstream task (module priority) | here |
//! |---|---|
//! | `DynamicsTask`: `rwStateEffector` (301), `scObject` `chief` (201), `simpleNavObject` (109) | [`FormationDynamics::chief`] |
//! | `DynamicsTask2`: `rwStateEffector2` (301), `extForceTorqueObject2` (300), `scObject2` `deputy` (201), `simpleNavObject2` (109) | [`FormationDynamics::deputy`] |
//!
//! Upstream runs the first task's modules, then its recorders, then the
//! second task's; [`super::RECORDER_PRIORITY`] sits between the two here, and
//! [`super::RECORDER_TASK2_PRIORITY`] after the second. Each spacecraft's
//! wheels take their commands from an `ArrayMotorTorqueToSingleActuators`
//! where upstream's wheel effector reads the FSW array, as in
//! [`super::dynamics`].

#![allow(dead_code)]

use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::reaction_wheel_state_effector::ReactionWheelStateEffector;
use basilisk_rs::messages::{
    BodyTorqueCommandMsg, Input, Output, ReactionWheelStateMsg, RwSpeedMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use nalgebra::Vector3;

use super::dynamics::{HUB_MASS_KG, WHEEL_COUNT, hub_inertia_kg_m2, rw_factory};
use super::grav_factory::Body;
use super::{BskSim, HubInit};

/// One spacecraft of the formation and its devices.
pub struct FormationSpacecraft {
    /// The wheel commands of upstream `rwMotorCmdInMsg`.
    pub rw_commands: ArrayMotorTorqueToSingleActuators,
    pub spacecraft: Spacecraft,
    pub simple_nav: SimpleNavigation,
    /// Upstream `rwStateEffector.rwSpeedOutMsg`.
    pub rw_speed_out_msg: Output<RwSpeedMsg>,
    /// Upstream `rwStateEffector.rwOutMsgs`.
    pub rw_out_msgs: Vec<Output<ReactionWheelStateMsg>>,
}

impl FormationSpacecraft {
    /// Upstream's `scObject` (`ModelTag` is the schedule name), its wheels
    /// `wheels_name` and navigation `nav_name`.
    fn new(wheels_name: &str, nav_name: &str, initial: &HubInit, period: u64) -> Self {
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: HUB_MASS_KG,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: hub_inertia_kg_m2(),
            integration_step_nanos: period,
            initial_position_m: initial.position_m,
            initial_velocity_mps: initial.velocity_mps,
            initial_sigma_bn: initial.sigma_bn,
            initial_omega_radps: initial.omega_bn_b_radps,
            integrator: None,
        });
        spacecraft
            .add_grav_body(Body::Earth.point_mass(true))
            .expect("one central body");

        let mut simple_nav = SimpleNavigation::new(nav_name);
        spacecraft
            .state_out
            .connect_to(&mut simple_nav.spacecraft_state_in_msg);

        let mut wheels = ReactionWheelStateEffector::new(wheels_name);
        for config in rw_factory().wheels {
            wheels.add_reaction_wheel(config);
        }
        let rw_commands =
            ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
                name: format!("{wheels_name}_cmd"),
                num_actuators: WHEEL_COUNT,
            });
        for (command, wheel) in rw_commands
            .actuator_out_msgs
            .iter()
            .zip(wheels.wheels_mut())
        {
            command.connect_to(&mut wheel.motor_torque_in_msg);
        }
        let rw_speed_out_msg = wheels.rw_speed_out_msg.clone();
        let rw_out_msgs = wheels
            .wheels()
            .iter()
            .map(|wheel| wheel.state_out.clone())
            .collect();
        spacecraft.add_state_effector(wheels);
        Self {
            rw_commands,
            spacecraft,
            simple_nav,
            rw_speed_out_msg,
            rw_out_msgs,
        }
    }
}

/// Upstream `BSKDynamicModels` of the formation.
pub struct FormationDynamics {
    /// Upstream `processTasksTimeStep` \[ns\].
    pub task_period_nanos: u64,
    pub chief: FormationSpacecraft,
    pub deputy: FormationSpacecraft,
    /// Upstream `extForceTorqueObject2.cmdTorqueInMsg`.
    pub deputy_ext_torque_cmd_in: Input<BodyTorqueCommandMsg>,
    /// Upstream `gravFactory.gravBodies['earth'].mu` \[m^3/s^2\].
    pub mu_earth_m3ps2: f64,
}

impl FormationDynamics {
    /// Upstream `BSKDynamicModels(SimBase, dynRate)` and
    /// `configure_initial_conditions`.
    pub fn new(base: &BskSim, chief: &HubInit, deputy: &HubInit) -> Self {
        let period = base.dyn_rate_nanos;
        let chief = FormationSpacecraft::new("RW_chief", "SimpleNavigation_chief", chief, period);
        let mut deputy =
            FormationSpacecraft::new("RW_deputy", "SimpleNavigation_deputy", deputy, period);
        let external = ExtForceTorque::new(ExtForceTorqueConfig {
            name: "externalDisturbance".to_string(),
            ..ExtForceTorqueConfig::default()
        });
        let deputy_ext_torque_cmd_in = external.cmd_torque_in_msg.clone();
        deputy.spacecraft.add_dynamic_effector(external);
        Self {
            task_period_nanos: period,
            chief,
            deputy,
            deputy_ext_torque_cmd_in,
            mu_earth_m3ps2: Body::Earth.mu_m3ps2(),
        }
    }

    /// Upstream `DynamicsTask` and `DynamicsTask2`.
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let period = self.task_period_nanos;
        basilisk_rs::schedule! { *simulation,
            "RW_chief_cmd" => &mut self.chief.rw_commands, period = period, priority = 9_300;
            "chief" => &mut self.chief.spacecraft, period = period, priority = 9_200;
            "SimpleNavigation_chief" => &mut self.chief.simple_nav, period = period, priority = 9_100;
            "RW_deputy_cmd" => &mut self.deputy.rw_commands, period = period, priority = 4_900;
            "deputy" => &mut self.deputy.spacecraft, period = period, priority = 4_800;
            "SimpleNavigation_deputy" => &mut self.deputy.simple_nav, period = period, priority = 4_700;
        }
    }
}
