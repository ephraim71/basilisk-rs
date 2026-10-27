//! MRP proportional-derivative attitude tracking control: port of upstream
//! Basilisk `mrpPD`.
//!
//! The control torque (Schaub and Junkins, *Analytical Mechanics of Space
//! Systems*, section 8.4.1)
//!
//! `L_r = -K σ_B/R - P δω + [I](ω̇_R/N - ω_B/N × ω_R/N) + ω_R/N × [I] ω_B/N - L`
//!
//! asymptotically tracks a general reference motion when there are no unknown
//! dynamics; `L` is a known external torque. It is the external-torque
//! (thruster) counterpart of [`super::mrp_feedback`]: no reaction-wheel
//! momentum and no integral feedback. The inertia is read from the vehicle
//! configuration message at reset only.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{AttitudeGuidanceMsg, BodyTorqueCommandMsg, Input, Output, VehicleConfigMsg};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`MrpPd`]. `Default` is upstream's
/// zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct MrpPdConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `K` (labelled \[rad/s\] upstream): proportional gain on the
    /// MRP error.
    pub k: f64,
    /// Upstream `P` \[N m s\]: rate error feedback gain.
    pub p: f64,
    /// Upstream `knownTorquePntB_B` \[N m\]: known external torque, body
    /// components.
    pub known_torque_body_nm: Vector3<f64>,
}

impl Default for MrpPdConfig {
    fn default() -> Self {
        Self {
            name: "mrpPD".to_string(),
            k: 0.0,
            p: 0.0,
            known_torque_body_nm: Vector3::zeros(),
        }
    }
}

/// MRP PD attitude controller: upstream `mrpPD`.
#[derive(Clone, Debug)]
pub struct MrpPd {
    /// User-settable parameters.
    pub config: MrpPdConfig,
    /// Upstream `guidInMsg`: attitude guidance.
    pub guid_in_msg: Input<AttitudeGuidanceMsg>,
    /// Upstream `vehConfigInMsg`: vehicle configuration (inertia), read at
    /// reset.
    pub veh_config_in_msg: Input<VehicleConfigMsg>,
    /// Upstream `cmdTorqueOutMsg`: commanded external control torque.
    pub cmd_torque_out_msg: Output<BodyTorqueCommandMsg>,
    /// Upstream `ISCPntB_B` \[kg m^2\]: inertia read at reset.
    inertia_kg_m2: Matrix3<f64>,
}

impl MrpPd {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: MrpPdConfig) -> Self {
        Self {
            config,
            guid_in_msg: Input::default(),
            veh_config_in_msg: Input::default(),
            cmd_torque_out_msg: Output::default(),
            inertia_kg_m2: Matrix3::zeros(),
        }
    }

    /// Upstream `Reset_mrpPD`.
    fn reset_controller(&mut self) {
        if !self.guid_in_msg.is_connected() {
            log::error!("mrpPD.guid_in_msg is not connected");
        }
        if !self.veh_config_in_msg.is_connected() {
            log::error!("mrpPD.veh_config_in_msg is not connected");
        }
        self.inertia_kg_m2 = self.veh_config_in_msg.read().inertia_about_point_b_kg_m2;
    }

    /// The control torque \[N m\], in upstream's operation order.
    fn control_torque(&self, guidance: &AttitudeGuidanceMsg) -> Vector3<f64> {
        let config = &self.config;
        let inertia = &self.inertia_kg_m2;
        let omega_rn_b = guidance.omega_rn_b_radps;
        let omega_bn_b = guidance.omega_br_b_radps + omega_rn_b;

        let feedback = guidance.sigma_br * config.k + guidance.omega_br_b_radps * config.p;
        let gyroscopic = omega_rn_b.cross(&(inertia * omega_bn_b));
        let feedforward = inertia * (guidance.domega_rn_b_radps2 - omega_bn_b.cross(&omega_rn_b));
        -(config.known_torque_body_nm + (feedback - gyroscopic - feedforward))
    }
}

impl Module for MrpPd {
    fn init(&mut self) {
        self.cmd_torque_out_msg
            .write(BodyTorqueCommandMsg::default());
        self.reset_controller();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_controller();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let guidance = self.guid_in_msg.read();
        self.cmd_torque_out_msg.write(BodyTorqueCommandMsg {
            torque_request_body_nm: self.control_torque(&guidance),
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{MrpPd, MrpPdConfig};
    use crate::Module;
    use crate::fsw_algorithms::att_control::rate_servo_full_nonlinear::test_support::{
        TASK_NANOS, assert_array_equal, context, guidance, vehicle_config,
    };
    use crate::messages::{AttitudeGuidanceMsg, BodyTorqueCommandMsg, Output, VehicleConfigMsg};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;

    /// Upstream `findTrueTorques` of `test_mrpPD.py`, which builds the inertia
    /// from the diagonal of `ISCPntB_B`.
    fn upstream_true_torque(
        config: &MrpPdConfig,
        guidance: &AttitudeGuidanceMsg,
        vehicle: &VehicleConfigMsg,
    ) -> Vector3<f64> {
        let inertia = Matrix3::from_diagonal(&vehicle.inertia_about_point_b_kg_m2.diagonal());
        let omega_bn_b = guidance.omega_br_b_radps + guidance.omega_rn_b_radps;
        let temp1 = inertia * omega_bn_b;
        let temp2 = guidance.domega_rn_b_radps2 - omega_bn_b.cross(&guidance.omega_rn_b_radps);
        let mut lr = config.k * guidance.sigma_br + config.p * guidance.omega_br_b_radps
            - guidance.omega_rn_b_radps.cross(&temp1)
            - inertia * temp2;
        lr += config.known_torque_body_nm;
        lr * -1.0
    }

    /// `test_mrp_PD_tracking`, both `setExtTorque` parametrizations, with
    /// upstream's Python truth and `accuracy = 1e-12`: a 0.5 s task run to
    /// 1 s, all three samples checked.
    #[test]
    fn matches_upstream_mrp_pd_tracking_unit_test() {
        for set_ext_torque in [false, true] {
            let guidance_out = Output::new(guidance());
            let vehicle_out = Output::new(vehicle_config());
            let mut module = MrpPd::new(MrpPdConfig::default());
            module.config.k = 0.15;
            module.config.p = 150.0;
            if set_ext_torque {
                module.config.known_torque_body_nm = Vector3::new(0.1, 0.2, 0.3);
            }
            vehicle_out.connect_to(&mut module.veh_config_in_msg);
            guidance_out.connect_to(&mut module.guid_in_msg);

            let mut recorder = MessageRecorder::<BodyTorqueCommandMsg>::default();
            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            simulation.connect(&module.cmd_torque_out_msg, &mut recorder.input_msg);
            simulation.add_module("mrpPD", &mut module, TASK_NANOS, 0);
            simulation.add_module("recorder", &mut recorder, TASK_NANOS, 0);
            simulation.run_for(2 * TASK_NANOS);
            drop(simulation);

            let truth = upstream_true_torque(&module.config, &guidance(), &vehicle_config());
            assert_eq!(recorder.samples.len(), 3, "setExtTorque {set_ext_torque}");
            for (index, sample) in recorder.samples.iter().enumerate() {
                assert_array_equal(
                    sample.torque_request_body_nm,
                    truth,
                    1e-12,
                    &format!("setExtTorque {set_ext_torque} torqueRequestBody sample {index}"),
                );
            }
        }
    }

    /// Not upstream: with no rates the torque is `-(K σ + L)`.
    #[test]
    fn a_pure_attitude_error_gives_a_proportional_torque() {
        let sigma_br = Vector3::new(0.1, -0.2, 0.3);
        let guidance_out = Output::new(AttitudeGuidanceMsg {
            sigma_br,
            ..Default::default()
        });
        let vehicle_out = Output::new(vehicle_config());
        let mut module = MrpPd::new(MrpPdConfig {
            k: 2.0,
            p: 50.0,
            known_torque_body_nm: Vector3::new(0.5, 0.0, -0.5),
            ..Default::default()
        });
        guidance_out.connect_to(&mut module.guid_in_msg);
        vehicle_out.connect_to(&mut module.veh_config_in_msg);
        module.init();
        module.update(&context(0));
        assert_array_equal(
            module.cmd_torque_out_msg.read().torque_request_body_nm,
            Vector3::new(-0.7, 0.4, -0.1),
            1e-15,
            "torqueRequestBody",
        );
    }

    /// Not upstream: as upstream, the inertia is read at reset; a later
    /// vehicle configuration applies after the next reset.
    #[test]
    fn reads_the_inertia_only_at_reset() {
        let guidance_out = Output::new(guidance());
        let vehicle_out = Output::new(vehicle_config());
        let mut module = MrpPd::new(MrpPdConfig {
            k: 0.15,
            p: 150.0,
            ..Default::default()
        });
        guidance_out.connect_to(&mut module.guid_in_msg);
        vehicle_out.connect_to(&mut module.veh_config_in_msg);
        module.init();
        module.update(&context(0));
        let before = module.cmd_torque_out_msg.read().torque_request_body_nm;

        let stiffer = VehicleConfigMsg {
            inertia_about_point_b_kg_m2: Matrix3::identity() * 3000.0,
            ..Default::default()
        };
        vehicle_out.write(stiffer.clone());
        module.update(&context(TASK_NANOS));
        assert_eq!(
            module.cmd_torque_out_msg.read().torque_request_body_nm,
            before
        );

        module.reset(&context(TASK_NANOS));
        module.update(&context(2 * TASK_NANOS));
        assert_array_equal(
            module.cmd_torque_out_msg.read().torque_request_body_nm,
            upstream_true_torque(&module.config, &guidance(), &stiffer),
            1e-12,
            "after reset",
        );
    }

    #[test]
    fn init_publishes_a_zero_torque() {
        let mut module = MrpPd::new(MrpPdConfig::default());
        module.cmd_torque_out_msg.write(BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::new(1.0, 2.0, 3.0),
        });
        module.init();
        assert_eq!(
            module.cmd_torque_out_msg.read().torque_request_body_nm,
            Vector3::zeros()
        );
    }
}
