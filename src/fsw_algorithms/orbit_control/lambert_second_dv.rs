//! Arrival delta-v of a Lambert transfer: port of upstream Basilisk
//! `fswAlgorithms/orbitControl/lambertSecondDV`.
//!
//! Every update commands the delta-v from the selected Lambert solution's
//! arrival velocity to the desired velocity, at the desired maneuver time. An
//! invalid Lambert solution commands nothing (a zeroed burn). Upstream's
//! `lambertSolutionSpecifier` values other than 1 or 2, which it rejects with
//! `BasiliskError`, cannot be expressed in [`LambertSolutionSpecifier`].

use super::lambert_validator::lambert_burn_command;
use crate::messages::{
    DesiredVelocityMsg, DvBurnCommandMsg, Input, LambertSolutionMsg, LambertSolutionSpecifier,
    Output,
};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`LambertSecondDv`].
#[derive(Clone, Debug)]
pub struct LambertSecondDvConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `lambertSolutionSpecifier`: which Lambert solution to use
    /// (default the first).
    pub lambert_solution_specifier: LambertSolutionSpecifier,
}

impl Default for LambertSecondDvConfig {
    fn default() -> Self {
        Self {
            name: "lambertSecondDV".to_string(),
            lambert_solution_specifier: LambertSolutionSpecifier::First,
        }
    }
}

/// Commands the burn from the Lambert arrival velocity to the desired one:
/// upstream `LambertSecondDV`.
#[derive(Clone, Debug)]
pub struct LambertSecondDv {
    /// Module configuration.
    pub config: LambertSecondDvConfig,
    /// Upstream `lambertSolutionInMsg`: the solver's velocities.
    pub lambert_solution_in_msg: Input<LambertSolutionMsg>,
    /// Upstream `desiredVelocityInMsg`: the velocity to reach, and when.
    pub desired_velocity_in_msg: Input<DesiredVelocityMsg>,
    /// Upstream `dvBurnCmdOutMsg`: the delta-v command.
    pub dv_burn_cmd_out_msg: Output<DvBurnCommandMsg>,
}

impl LambertSecondDv {
    /// Builds the module with unconnected ports.
    pub fn new(config: LambertSecondDvConfig) -> Self {
        Self {
            config,
            lambert_solution_in_msg: Input::default(),
            desired_velocity_in_msg: Input::default(),
            dv_burn_cmd_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset`: the module keeps no state, so a reset only reports
    /// missing inputs.
    fn check_connections(&self) {
        if !self.lambert_solution_in_msg.is_connected() {
            log::error!("lambertSecondDV.lambert_solution_in_msg is not connected");
        }
        if !self.desired_velocity_in_msg.is_connected() {
            log::error!("lambertSecondDV.desired_velocity_in_msg is not connected");
        }
    }

    fn command(&self) -> DvBurnCommandMsg {
        let solution = *self
            .lambert_solution_in_msg
            .read()
            .solution(self.config.lambert_solution_specifier);
        let desired = self.desired_velocity_in_msg.read();
        if solution.valid {
            lambert_burn_command(
                desired.v_desired_n_mps - solution.v2_n_mps,
                desired.maneuver_time_s,
            )
        } else {
            DvBurnCommandMsg::default()
        }
    }
}

impl Module for LambertSecondDv {
    fn init(&mut self) {
        self.dv_burn_cmd_out_msg.write(DvBurnCommandMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.dv_burn_cmd_out_msg.write(self.command());
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::{LambertSecondDv, LambertSecondDvConfig};
    use crate::Module;
    use crate::fsw_algorithms::orbit_control::test_support::{conic_state, context, elements};
    use crate::messages::{
        DesiredVelocityMsg, DvBurnCommandMsg, LambertSolutionMsg, LambertSolutionSpecifier,
        LambertTransferVelocities, Output,
    };
    use crate::utilities::astro_constants::D2R;

    fn connected(
        config: LambertSecondDvConfig,
        solution: &Output<LambertSolutionMsg>,
        desired: &Output<DesiredVelocityMsg>,
    ) -> LambertSecondDv {
        let mut module = LambertSecondDv::new(config);
        solution.connect_to(&mut module.lambert_solution_in_msg);
        desired.connect_to(&mut module.desired_velocity_in_msg);
        module.init();
        module
    }

    /// Upstream `test_lambertSecondDV`: two delta-vs at two maneuver times
    /// with a valid solution, and one with an invalid solution, whose command
    /// must be zero; the delta-v at 1e-4 m/s and the burn start time.
    #[test]
    fn matches_upstream_lambert_second_dv_test() {
        let accuracy = 1e-4;
        let mu = 3.986004418e14;
        let mut cases = Vec::new();
        for dv in [
            Vector3::new(1.0, 3.0, 4.0),
            Vector3::new(500.0, -100.0, 200.0),
        ] {
            for tm in [1.0e3, 2.0e3] {
                cases.push((dv, tm, true));
            }
        }
        cases.push((Vector3::new(1.0, 3.0, 4.0), 1.0e3, false));

        for (dv, tm, valid) in cases {
            let oe = elements(
                10000.0 * 1000.0,
                0.1,
                10.0 * D2R,
                20.0 * D2R,
                30.0 * D2R,
                40.0 * D2R,
            );
            let (_, v1) = conic_state(mu, &oe);
            let solution = Output::new(LambertSolutionMsg {
                first: LambertTransferVelocities {
                    v2_n_mps: v1,
                    valid,
                    ..Default::default()
                },
                ..Default::default()
            });
            let desired = Output::new(DesiredVelocityMsg {
                v_desired_n_mps: v1 + dv,
                maneuver_time_s: tm,
            });
            let mut module = connected(LambertSecondDvConfig::default(), &solution, &desired);
            module.update(&context(0));
            let command = module.dv_burn_cmd_out_msg.read();
            let (dv_true, burn_start_true) = if valid {
                (dv, (tm * 1e9) as u64)
            } else {
                (Vector3::zeros(), 0)
            };
            assert!(
                (command.dv_inertial_cmd_mps - dv_true).amax() <= accuracy,
                "dv={dv:?}, tm={tm}, valid={valid}: {command:?}"
            );
            assert_eq!(command.burn_start_time_nanos, burn_start_true);
        }
    }

    /// Not upstream: the second solution's arrival velocity and validity are
    /// used when specified, and the unused rotation is the x axis at zero
    /// rate.
    #[test]
    fn second_solution_specifier_uses_the_second_arrival_velocity() {
        let solution = Output::new(LambertSolutionMsg {
            first: LambertTransferVelocities {
                v2_n_mps: Vector3::new(9.0, 9.0, 9.0),
                valid: false,
                ..Default::default()
            },
            second: LambertTransferVelocities {
                v2_n_mps: Vector3::new(1.0, 2.0, 3.0),
                valid: true,
                ..Default::default()
            },
        });
        let desired = Output::new(DesiredVelocityMsg {
            v_desired_n_mps: Vector3::new(2.0, 2.0, 2.0),
            maneuver_time_s: 0.5,
        });
        let mut module = connected(
            LambertSecondDvConfig {
                lambert_solution_specifier: LambertSolutionSpecifier::Second,
                ..Default::default()
            },
            &solution,
            &desired,
        );
        module.update(&context(0));
        assert_eq!(
            module.dv_burn_cmd_out_msg.read(),
            DvBurnCommandMsg {
                dv_inertial_cmd_mps: Vector3::new(1.0, 0.0, -1.0),
                dv_rot_vec_unit: Vector3::x(),
                dv_rot_vec_mag_radps: 0.0,
                burn_start_time_nanos: 500_000_000,
            }
        );

        module.config.lambert_solution_specifier = LambertSolutionSpecifier::First;
        module.update(&context(0));
        assert_eq!(
            module.dv_burn_cmd_out_msg.read(),
            DvBurnCommandMsg::default()
        );
    }
}
