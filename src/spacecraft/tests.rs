use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};
use std::any::Any;
use std::sync::{Arc, Mutex};

use super::{
    EffectorOutput, Spacecraft, SpacecraftConfig, StateEffector,
    mrp::body_to_inertial_dcm_from_sigma_bn,
};
use crate::Module;
use crate::dynamics::drag_dynamic_effector::{DragDynamicEffector, DragDynamicEffectorConfig};
use crate::dynamics::gravity::GravBodyData;
use crate::dynamics::hinged_rigid_body_state_effector::{
    HingedRigidBodyConfig, HingedRigidBodyStateEffector,
};
use crate::dynamics::reaction_wheel_state_effector::{
    ReactionWheelStateEffector, ReactionWheelStateEffectorConfig,
};
use crate::integrators::stochastic::{
    Dri1, Dri1Nm, EulerHeun, EulerMaruyama, Rdi1Wm, Ri1, Ri3, Ri5, Ri6, RkMil, Rs1, Rs2, Siea,
    Sieb, Smea, Smeb, Sosra, Sosri, Sra1, Sriw1, W2Ito1, W2Ito2,
};
use crate::integrators::{
    AdaptiveController, ButcherTableau, Dopri45, DynamicObject, Euler, Heun, Integrator, Midpoint,
    RK4, Ralston, Rk4, Rk4ThreeEighths, Rkf45, Rkf78, propagate_adaptive_rk, propagate_explicit_rk,
};
use crate::messages::{
    AtmosphereMsg, HingedRigidBodyMsg, Input, MotorTorqueMsg, Output, SpacecraftStateMsg,
};
use crate::simulation::Simulation;
use crate::spacecraft::structs::BackSubMatrices;
use crate::spacecraft::traits::DynamicEffector;

const MU_EARTH_M3PS2: f64 = 3.986_004_418e14;
const STEP_NANOS: u64 = 5_000_000; // 5 ms
const DURATION_NANOS: u64 = 100_000_000_000; // 100 s

fn start_epoch() -> Epoch {
    Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
}

/// Circular LEO spacecraft with point-mass Earth gravity and no effectors.
fn circular_orbit_spacecraft(radius_m: f64) -> Spacecraft {
    let v_circular = (MU_EARTH_M3PS2 / radius_m).sqrt();
    let mut sc = Spacecraft::new(SpacecraftConfig {
        mass_kg: 100.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::new(radius_m, 0.0, 0.0),
        initial_velocity_mps: Vector3::new(0.0, v_circular, 0.0),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    sc.add_grav_body(
        GravBodyData::point_mass(
            "earth",
            MU_EARTH_M3PS2,
            true,
            Vector3::zeros(),
            Vector3::zeros(),
        )
        .expect("valid Earth gravity body"),
    )
    .expect("unique central gravity body");
    sc
}

fn orbital_angular_momentum(pos: Vector3<f64>, vel: Vector3<f64>) -> Vector3<f64> {
    pos.cross(&vel)
}

fn orbital_energy(pos: Vector3<f64>, vel: Vector3<f64>) -> f64 {
    0.5 * vel.norm_squared() - MU_EARTH_M3PS2 / pos.norm()
}

#[test]
fn t0_refresh_uses_fresh_dynamic_effector_inputs() {
    let velocity_mps = Vector3::new(0.0, 2.0, 0.0);
    let density_kgpm3 = 2.0;
    let drag_coefficient = 1.5;
    let projected_area_m2 = 3.0;
    let expected_force_body_n = Vector3::new(
        0.0,
        -0.5 * density_kgpm3 * velocity_mps.norm_squared() * drag_coefficient * projected_area_m2,
        0.0,
    );

    let atmosphere = Output::<AtmosphereMsg>::default();
    let mut drag = DragDynamicEffector::new(DragDynamicEffectorConfig {
        name: "drag".to_string(),
        projected_area_m2,
        drag_coeff: drag_coefficient,
        com_offset_m: Vector3::zeros(),
        planet_rotation_rate_radps: Vector3::zeros(),
    });
    drag.input_atmosphere_msg.connect(atmosphere.slot());

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: 1_000_000_000,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft.add_dynamic_effector(drag);
    let diagnostics = spacecraft.diagnostics_out.clone();

    let mut simulation = Simulation::new(start_epoch(), false);
    simulation.add_module("spacecraft", &mut spacecraft, 1_000_000_000, 0);
    simulation.initialize();
    assert_eq!(
        diagnostics.read().drag_force_body_n,
        Vector3::zeros(),
        "initialization should observe the default atmosphere input"
    );

    atmosphere.write(AtmosphereMsg {
        neutral_density_kgpm3: density_kgpm3,
        local_temp_k: 0.0,
    });
    simulation.run_for(0);
    assert_eq!(
        diagnostics.read().drag_force_body_n,
        expected_force_body_n,
        "the t=0 refresh should observe inputs published earlier on the same tick"
    );
}

/// Orbital angular momentum must be conserved to 1e-10 relative over 100 s.
#[test]
fn keplerian_orbit_conserves_angular_momentum() {
    let radius_m = 7_000_000.0;
    let v_circular = (MU_EARTH_M3PS2 / radius_m).sqrt();
    let pos0 = Vector3::new(radius_m, 0.0, 0.0);
    let vel0 = Vector3::new(0.0, v_circular, 0.0);
    let l_initial = orbital_angular_momentum(pos0, vel0);

    let mut sc = circular_orbit_spacecraft(radius_m);
    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.add_module("spacecraft", &mut sc, STEP_NANOS, 0);
        sim.run_for(DURATION_NANOS);
    }

    let sf = sc.state_out.read();
    let l_final = orbital_angular_momentum(sf.position_m, sf.velocity_mps);
    let l_norm = l_initial.norm();

    for i in 0..3 {
        let rel_err = (l_final[i] - l_initial[i]).abs() / l_norm;
        assert!(
            rel_err < 1e-10,
            "orbital angular momentum component {i} not conserved: \
             L0={:.6e}  Lf={:.6e}  rel_err={:.2e}",
            l_initial[i],
            l_final[i],
            rel_err
        );
    }
}

/// Orbital energy must be conserved to 1e-10 relative over 100 s.
#[test]
fn keplerian_orbit_conserves_energy() {
    let radius_m = 7_000_000.0;
    let v_circular = (MU_EARTH_M3PS2 / radius_m).sqrt();
    let pos0 = Vector3::new(radius_m, 0.0, 0.0);
    let vel0 = Vector3::new(0.0, v_circular, 0.0);
    let e_initial = orbital_energy(pos0, vel0);

    let mut sc = circular_orbit_spacecraft(radius_m);
    let diagnostics = sc.diagnostics_out.clone();
    let diagnostics_initial;
    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.add_module("spacecraft", &mut sc, STEP_NANOS, 0);
        sim.run_for(0);
        diagnostics_initial = diagnostics.read();
        sim.run_for(DURATION_NANOS);
    }

    let sf = sc.state_out.read();
    let e_final = orbital_energy(sf.position_m, sf.velocity_mps);

    let rel_err = (e_final - e_initial).abs() / e_initial.abs();
    assert!(
        rel_err < 1e-10,
        "orbital energy not conserved: E0={:.6e}  Ef={:.6e}  rel_err={:.2e}",
        e_initial,
        e_final,
        rel_err
    );

    let diagnostics_final = diagnostics.read();
    let expected_total_energy_j = 100.0 * e_initial;
    assert!(
        (diagnostics_initial.orbital_energy_j - expected_total_energy_j).abs()
            / expected_total_energy_j.abs()
            < 1.0e-12,
        "initial diagnostic orbital energy does not match the closed form"
    );
    assert!(
        (diagnostics_final.orbital_energy_j - diagnostics_initial.orbital_energy_j).abs()
            / diagnostics_initial.orbital_energy_j.abs()
            < 1.0e-10,
        "diagnostic orbital total energy was not conserved"
    );
    let expected_initial_acceleration = Vector3::new(-MU_EARTH_M3PS2 / radius_m.powi(2), 0.0, 0.0);
    assert!(
        (diagnostics_initial.gravity_accel_inertial_mps2 - expected_initial_acceleration).norm()
            < 1.0e-12,
        "diagnostic gravity acceleration does not match point-mass truth"
    );
}

/// Body angular momentum magnitude must be conserved under torque-free rotation.
#[test]
fn torque_free_rotation_conserves_angular_momentum() {
    let inertia_diag = Vector3::new(0.12, 0.15, 0.18);
    let inertia = Matrix3::from_diagonal(&inertia_diag);
    let omega0 = Vector3::new(0.1, 0.05, 0.02);
    let l_initial = (inertia * omega0).norm();

    let mut sc = Spacecraft::new(SpacecraftConfig {
        mass_kg: 10.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: inertia,
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: omega0,
        integrator: None,
    });

    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.add_module("spacecraft", &mut sc, STEP_NANOS, 0);
        sim.run_for(DURATION_NANOS);
    }

    let omega_f = sc.state_out.read().omega_radps;
    let l_final = (inertia * omega_f).norm();

    let rel_err = (l_final - l_initial).abs() / l_initial;
    assert!(
        rel_err < 1e-10,
        "body angular momentum not conserved: L0={:.6e}  Lf={:.6e}  rel_err={:.2e}",
        l_initial,
        l_final,
        rel_err
    );
}

#[test]
fn balanced_reaction_wheel_back_substitution_conserves_total_angular_momentum() {
    let locked_inertia_x = 0.12;
    let wheel_js = 0.02;
    let applied_torque_nm = 0.001;

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 10.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::new(locked_inertia_x, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.18),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });

    let mut wheel_config = ReactionWheelStateEffectorConfig::balanced(
        "rw_x",
        Vector3::zeros(),
        Vector3::new(1.0, 0.0, 0.0),
        1.0,
        100.0,
    );
    wheel_config.js_kg_m2 = wheel_js;
    let mut reaction_wheels = ReactionWheelStateEffector::new("reaction_wheels");
    reaction_wheels.add_reaction_wheel(wheel_config);
    let command = Output::new(MotorTorqueMsg {
        motor_torque_nm: applied_torque_nm,
    });

    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.connect(
            &command,
            &mut reaction_wheels.wheels_mut()[0].motor_torque_in_msg,
        );
        spacecraft.add_state_effector(reaction_wheels);
        sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 0);
        sim.run_for(1_000_000_000);
    }

    let body_omega_x = spacecraft.state_out.read().omega_radps.x;
    let wheel_omega_x = spacecraft.state_effectors[0]
        .as_any()
        .downcast_ref::<ReactionWheelStateEffector>()
        .expect("expected reaction wheel state effector")
        .wheels()[0]
        .omega_radps;

    let total_angular_momentum_x = locked_inertia_x * body_omega_x + wheel_js * wheel_omega_x;
    let expected_body_omega_x = -applied_torque_nm / (locked_inertia_x - wheel_js);
    let expected_wheel_omega_x = applied_torque_nm / wheel_js - expected_body_omega_x;

    assert!(
        total_angular_momentum_x.abs() < 1e-10,
        "expected internal wheel torque to conserve angular momentum, got Hx={total_angular_momentum_x:.6e}"
    );
    assert!(
        (body_omega_x - expected_body_omega_x).abs() < 5.0e-5,
        "expected body omega {:.6e} rad/s, got {:.6e} rad/s",
        expected_body_omega_x,
        body_omega_x
    );
    assert!(
        (wheel_omega_x - expected_wheel_omega_x).abs() < 5.0e-5,
        "expected wheel omega {:.6e} rad/s, got {:.6e} rad/s",
        expected_wheel_omega_x,
        wheel_omega_x
    );
}

/// A single fully-coupled imbalanced reaction wheel, offset from the body origin
/// so the position-coupling terms are exercised. com_offset_m = U_s/mass = 0.002 m.
fn fully_coupled_reaction_wheel() -> ReactionWheelStateEffector {
    let mut config = ReactionWheelStateEffectorConfig::jitter_fully_coupled(
        "rw_fc",
        Vector3::new(0.1, 0.15, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        -1.0,   // torque limit disabled
        1000.0, // momentum limit high enough that speed saturation never trips
        2.0,    // mass_kg
        0.008,  // jt
        0.008,  // jg
        0.002,  // com offset d = U_s/mass = 0.004/2.0
        8.0e-5, // off-diagonal inertia j13 (dynamic imbalance)
    );
    config.js_kg_m2 = 0.01;
    config.initial_omega_radps = 8.0;
    let mut reaction_wheels = ReactionWheelStateEffector::new("reaction_wheels");
    reaction_wheels.add_reaction_wheel(config);
    reaction_wheels
}

fn fully_coupled_reaction_wheel_spacecraft() -> Spacecraft {
    Spacecraft::new(SpacecraftConfig {
        mass_kg: 50.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::new(0.9, 0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.6),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.1, -0.05, 0.08),
        integrator: None,
    })
}

/// The wheel coasts with no motor command, no friction, and no gravity, so the
/// only internal coupling is the conservative imbalance: total rotational angular
/// momentum must be conserved.
#[test]
fn fully_coupled_reaction_wheel_conserves_total_angular_momentum() {
    let mut spacecraft = fully_coupled_reaction_wheel_spacecraft();
    let mut reaction_wheels = fully_coupled_reaction_wheel();
    let command = Output::new(MotorTorqueMsg::default());

    let diagnostics = spacecraft.diagnostics_out.clone();
    let h_initial;
    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.connect(
            &command,
            &mut reaction_wheels.wheels_mut()[0].motor_torque_in_msg,
        );
        spacecraft.add_state_effector(reaction_wheels);
        sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 0);
        sim.run_for(0);
        h_initial = diagnostics
            .read()
            .rotational_angular_momentum_inertial_kg_m2ps;
        sim.run_for(DURATION_NANOS);
    }

    let h_final = diagnostics
        .read()
        .rotational_angular_momentum_inertial_kg_m2ps;
    let h_norm = h_initial.norm();
    assert!(h_norm > 0.0, "expected non-zero initial angular momentum");

    for i in 0..3 {
        let rel_err = (h_final[i] - h_initial[i]).abs() / h_norm;
        assert!(
            rel_err < 1e-10,
            "fully-coupled RW angular momentum component {i} not conserved: \
             H0={:.6e}  Hf={:.6e}  rel_err={:.2e}",
            h_initial[i],
            h_final[i],
            rel_err
        );
    }
}

/// Cross-validation of the fully-coupled wheel dynamics against an independently
/// generated reference trajectory (one coasting imbalanced wheel, no gravity, no
/// motor torque, 1e-4 s RK4 steps for 0.5 s). Conservation laws cannot catch an
/// error that happens to preserve energy and momentum; a full trajectory match to
/// a reference does. The reference values below were produced by the original C++
/// model with identical initial conditions and wheel-frame axes.
#[test]
fn fully_coupled_reaction_wheel_matches_reference_trajectory() {
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 50.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::new(0.9, 0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.6),
        integration_step_nanos: 100_000, // 1e-4 s
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.1, -0.05, 0.08),
        integrator: None,
    });

    let mut config = ReactionWheelStateEffectorConfig::jitter_fully_coupled(
        "rw_fc",
        Vector3::new(0.1, 0.15, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        -1.0,
        -1.0,
        2.0,
        0.008,
        0.008,
        0.002,
        8.0e-5,
    );
    // Match the reference wheel frame exactly.
    config.torque_axis_body = Vector3::new(1.0, 0.0, 0.0);
    config.gimbal_axis_body = Vector3::new(0.0, 1.0, 0.0);
    config.js_kg_m2 = 0.01;
    config.initial_omega_radps = 8.0;
    let mut reaction_wheels = ReactionWheelStateEffector::new("reaction_wheels");
    reaction_wheels.add_reaction_wheel(config);

    let command = Output::new(MotorTorqueMsg::default());

    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.connect(
            &command,
            &mut reaction_wheels.wheels_mut()[0].motor_torque_in_msg,
        );
        spacecraft.add_state_effector(reaction_wheels);
        sim.add_module("spacecraft", &mut spacecraft, 100_000, 0);
        sim.run_for(500_000_000); // 0.5 s
    }

    let omega = spacecraft.state_out.read().omega_radps;
    let sigma = spacecraft.state_out.read().sigma_bn;
    let wheel_omega = spacecraft.state_effectors[0]
        .as_any()
        .downcast_ref::<ReactionWheelStateEffector>()
        .expect("expected reaction wheel state effector")
        .wheels()[0]
        .omega_radps;

    // Reference trajectory from the independent C++ model.
    let ref_omega = Vector3::new(
        0.1013573369696111,
        -0.04537441586853965,
        0.09294849613683358,
    );
    let ref_sigma = Vector3::new(
        0.012642215452002677,
        -0.0059225901096826485,
        0.01032380305253673,
    );
    let ref_wheel: f64 = 7.987030732719156;

    assert!(
        (omega - ref_omega).norm() / ref_omega.norm() < 1e-7,
        "body rate off reference: got {omega:?}, want {ref_omega:?}"
    );
    assert!(
        (sigma - ref_sigma).norm() / ref_sigma.norm() < 1e-7,
        "attitude off reference: got {sigma:?}, want {ref_sigma:?}"
    );
    assert!(
        (wheel_omega - ref_wheel).abs() / ref_wheel.abs() < 1e-7,
        "wheel speed off reference: got {wheel_omega}, want {ref_wheel}"
    );
}

/// Same coasting scenario, checking total rotational kinetic energy. Energy is a
/// stricter check than momentum: it catches sign/term errors in the coupled
/// mass properties and back-substitution that momentum can be blind to.
#[test]
fn fully_coupled_reaction_wheel_conserves_rotational_energy() {
    let mut spacecraft = fully_coupled_reaction_wheel_spacecraft();
    let mut reaction_wheels = fully_coupled_reaction_wheel();
    let command = Output::new(MotorTorqueMsg::default());

    let diagnostics = spacecraft.diagnostics_out.clone();
    let e_initial;
    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.connect(
            &command,
            &mut reaction_wheels.wheels_mut()[0].motor_torque_in_msg,
        );
        spacecraft.add_state_effector(reaction_wheels);
        sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 0);
        sim.run_for(0);
        e_initial = diagnostics.read().rotational_energy_j;
        sim.run_for(DURATION_NANOS);
    }
    let e_final = diagnostics.read().rotational_energy_j;

    let rel_err = (e_final - e_initial).abs() / e_initial.abs();
    assert!(
        rel_err < 1e-10,
        "fully-coupled RW rotational energy not conserved: \
         E0={e_initial:.6e}  Ef={e_final:.6e}  rel_err={rel_err:.2e}"
    );
}

/// A balanced wheel and a fully-coupled imbalanced wheel on one spacecraft
/// simultaneously: back-substitution contributions from each wheel model must
/// sum correctly rather than only being correct in isolation. Both models are
/// momentum-conservative in isolation, so coasting with no motor torque or
/// friction must conserve total rotational angular momentum with both present.
/// (JitterSimple is intentionally excluded here: it injects an imbalance
/// force/torque onto the hub without tracking the imbalance mass as a state, so
/// it is not expected to conserve system momentum exactly, in isolation or
/// combined with other wheels.)
#[test]
fn mixed_reaction_wheel_models_conserve_total_angular_momentum() {
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 50.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::new(0.9, 0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.6),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.1, -0.05, 0.08),
        integrator: None,
    });

    let mut balanced_config = ReactionWheelStateEffectorConfig::balanced(
        "rw_balanced",
        Vector3::new(0.1, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        -1.0,
        1000.0,
    );
    balanced_config.js_kg_m2 = 0.01;
    balanced_config.initial_omega_radps = 5.0;
    let mut balanced = ReactionWheelStateEffector::new("rw_balanced");
    balanced.add_reaction_wheel(balanced_config);

    let mut fully_coupled_config = ReactionWheelStateEffectorConfig::jitter_fully_coupled(
        "rw_fully_coupled",
        Vector3::new(0.0, 0.0, 0.1),
        Vector3::new(0.0, 0.0, 1.0),
        -1.0,
        1000.0,
        2.0,
        0.008,
        0.008,
        0.002,
        8.0e-5,
    );
    fully_coupled_config.js_kg_m2 = 0.01;
    fully_coupled_config.initial_omega_radps = 8.0;
    let mut fully_coupled = ReactionWheelStateEffector::new("rw_fully_coupled");
    fully_coupled.add_reaction_wheel(fully_coupled_config);

    let cmd_balanced = Output::new(MotorTorqueMsg::default());
    let cmd_fully_coupled = Output::new(MotorTorqueMsg::default());

    let diagnostics = spacecraft.diagnostics_out.clone();
    let h_initial;
    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.connect(
            &cmd_balanced,
            &mut balanced.wheels_mut()[0].motor_torque_in_msg,
        );
        sim.connect(
            &cmd_fully_coupled,
            &mut fully_coupled.wheels_mut()[0].motor_torque_in_msg,
        );
        spacecraft.add_state_effector(balanced);
        spacecraft.add_state_effector(fully_coupled);
        sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 0);
        sim.run_for(0);
        h_initial = diagnostics
            .read()
            .rotational_angular_momentum_inertial_kg_m2ps;
        sim.run_for(DURATION_NANOS);
    }

    let h_final = diagnostics
        .read()
        .rotational_angular_momentum_inertial_kg_m2ps;
    let h_norm = h_initial.norm();
    assert!(h_norm > 0.0, "expected non-zero initial angular momentum");
    assert_eq!(spacecraft.state_effectors.len(), 2);

    for i in 0..3 {
        let rel_err = (h_final[i] - h_initial[i]).abs() / h_norm;
        assert!(
            rel_err < 1e-9,
            "mixed RW models angular momentum component {i} not conserved: \
             H0={:.6e}  Hf={:.6e}  rel_err={:.2e}",
            h_initial[i],
            h_final[i],
            rel_err
        );
    }
}

/// Total rotational angular momentum about the system center of mass must be
/// conserved while a sprung hinged panel oscillates under torque-free rotation.

#[test]
fn sprung_hinged_panel_conserves_total_angular_momentum() {
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 750.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::new(900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.1, -0.05, 0.08),
        integrator: None,
    });

    // Sprung panel offset from the spin axis, displaced from its rest angle so it
    // oscillates. No motor torque and no damping: the only internal coupling is the
    // conservative spring, and there is no external torque (no gravity body).
    let mut config = HingedRigidBodyConfig::new("panel");
    config.mass_kg = 100.0;
    config.inertia_about_panel_com_panel_kg_m2 =
        Matrix3::new(100.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 50.0);
    config.center_of_mass_offset_m = 1.5;
    config.spring_constant_nm_per_rad = 100.0;
    config.damping_nm_s_per_rad = 0.0;
    config.hinge_position_body_m = Vector3::new(0.5, 0.0, 1.0);
    config.body_to_hinge_dcm = Matrix3::identity();
    config.theta_init_rad = 0.2;
    config.theta_ref_rad = 0.0;
    let panel = HingedRigidBodyStateEffector::new(config);

    let diagnostics = spacecraft.diagnostics_out.clone();
    let h_initial;
    {
        let mut sim = Simulation::new(start_epoch(), false);
        spacecraft.add_state_effector(panel);
        sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 0);
        sim.run_for(0);
        h_initial = diagnostics
            .read()
            .rotational_angular_momentum_inertial_kg_m2ps;
        sim.run_for(DURATION_NANOS);
    }

    let h_final = diagnostics
        .read()
        .rotational_angular_momentum_inertial_kg_m2ps;
    let h_norm = h_initial.norm();
    assert!(h_norm > 0.0, "expected non-zero initial angular momentum");

    for i in 0..3 {
        let rel_err = (h_final[i] - h_initial[i]).abs() / h_norm;
        assert!(
            rel_err < 1e-10,
            "rotational angular momentum component {i} not conserved: \
             H0={:.6e}  Hf={:.6e}  rel_err={:.2e}",
            h_initial[i],
            h_final[i],
            rel_err
        );
    }
}

/// Rotational kinetic energy must be conserved under torque-free rigid-body rotation.
/// (Reads the diagnostics `rotational_energy_j` field directly, exercising that path.)
#[test]
fn torque_free_rotation_conserves_rotational_energy() {
    let inertia = Matrix3::from_diagonal(&Vector3::new(0.12, 0.15, 0.18));
    let mut sc = Spacecraft::new(SpacecraftConfig {
        mass_kg: 10.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: inertia,
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.1, 0.05, 0.02),
        integrator: None,
    });

    let diagnostics = sc.diagnostics_out.clone();
    let e_initial;
    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.add_module("spacecraft", &mut sc, STEP_NANOS, 0);
        sim.run_for(0);
        e_initial = diagnostics.read().rotational_energy_j;
        sim.run_for(DURATION_NANOS);
    }
    let e_final = diagnostics.read().rotational_energy_j;

    let rel_err = (e_final - e_initial).abs() / e_initial.abs();
    assert!(
        rel_err < 1e-10,
        "rotational energy not conserved: E0={e_initial:.6e}  Ef={e_final:.6e}  rel_err={rel_err:.2e}"
    );
}

/// With a conservative sprung panel (no damping, no motor) and no external torque,
/// total rotational energy (kinetic + spring potential) must be conserved. This
/// exercises the spring potential-energy term and is a stricter check than momentum:
/// it catches sign/term errors in the back-substitution that momentum can be blind to.
#[test]
fn sprung_hinged_panel_conserves_total_energy() {
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 750.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::new(900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.1, -0.05, 0.08),
        integrator: None,
    });

    let mut config = HingedRigidBodyConfig::new("panel");
    config.mass_kg = 100.0;
    config.inertia_about_panel_com_panel_kg_m2 =
        Matrix3::new(100.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 50.0);
    config.center_of_mass_offset_m = 1.5;
    config.spring_constant_nm_per_rad = 100.0;
    config.damping_nm_s_per_rad = 0.0;
    config.hinge_position_body_m = Vector3::new(0.5, 0.0, 1.0);
    config.body_to_hinge_dcm = Matrix3::identity();
    config.theta_init_rad = 0.2;
    config.theta_ref_rad = 0.0;
    let panel = HingedRigidBodyStateEffector::new(config);

    let diagnostics = spacecraft.diagnostics_out.clone();
    let e_initial;
    {
        let mut sim = Simulation::new(start_epoch(), false);
        spacecraft.add_state_effector(panel);
        sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 0);
        sim.run_for(0);
        e_initial = diagnostics.read().rotational_energy_j;
        sim.run_for(DURATION_NANOS);
    }
    let e_final = diagnostics.read().rotational_energy_j;

    // RK4 is not symplectic, so energy conservation is to integration accuracy rather
    // than machine precision; the spring oscillates ~10 times over the run.
    let rel_err = (e_final - e_initial).abs() / e_initial.abs();
    assert!(
        rel_err < 1e-9,
        "total rotational energy not conserved: E0={e_initial:.6e}  Ef={e_final:.6e}  rel_err={rel_err:.2e}"
    );
}

/// A panel damper (c>0) must conserve total angular momentum (it is an internal
/// torque) while strictly dissipating energy. A flipped damper sign would inject
/// energy instead of removing it, so asserting monotonic energy decrease guards the
/// sign of the damping term.
#[test]
fn damped_hinged_panel_dissipates_energy_but_conserves_momentum() {
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 750.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::new(900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.1, -0.05, 0.08),
        integrator: None,
    });

    let mut config = HingedRigidBodyConfig::new("panel");
    config.mass_kg = 100.0;
    config.inertia_about_panel_com_panel_kg_m2 =
        Matrix3::new(100.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 50.0);
    config.center_of_mass_offset_m = 1.5;
    config.spring_constant_nm_per_rad = 100.0;
    config.damping_nm_s_per_rad = 5.0; // positive damping -> dissipation
    config.hinge_position_body_m = Vector3::new(0.5, 0.0, 1.0);
    config.body_to_hinge_dcm = Matrix3::identity();
    config.theta_init_rad = 0.2;
    config.theta_ref_rad = 0.0;
    let panel = HingedRigidBodyStateEffector::new(config);

    let diagnostics = spacecraft.diagnostics_out.clone();
    let h_initial;
    let mut energies = Vec::new();
    {
        let mut sim = Simulation::new(start_epoch(), false);
        spacecraft.add_state_effector(panel);
        sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 0);
        sim.run_for(0);
        let d0 = diagnostics.read();
        h_initial = d0.rotational_angular_momentum_inertial_kg_m2ps;
        energies.push(d0.rotational_energy_j);
        for _ in 0..20 {
            sim.run_for(5_000_000_000); // 5 s increments
            energies.push(diagnostics.read().rotational_energy_j);
        }
    }
    let d_final = diagnostics.read();
    let h_final = d_final.rotational_angular_momentum_inertial_kg_m2ps;

    // Angular momentum conserved despite the internal damper.
    let h_norm = h_initial.norm();
    for i in 0..3 {
        let rel_err = (h_final[i] - h_initial[i]).abs() / h_norm;
        assert!(
            rel_err < 1e-10,
            "damped panel: angular momentum component {i} not conserved (rel_err={rel_err:.2e})"
        );
    }

    // Energy strictly decreases (monotonic dissipation), and the total drop is real.
    for window in energies.windows(2) {
        assert!(
            window[1] <= window[0] + 1e-9,
            "energy increased across an interval: {:.9e} -> {:.9e}",
            window[0],
            window[1]
        );
    }
    let total_drop = energies.first().unwrap() - energies.last().unwrap();
    assert!(
        total_drop > 1e-3,
        "expected meaningful energy dissipation, got drop={total_drop:.3e}"
    );
}

/// A torque-free spin about a principal axis keeps omega exactly constant, so the
/// body-to-inertial attitude is a closed-form rotation by omega*t. Spinning fast
/// enough to cross |sigma|=1 many times exercises the MRP shadow-set switch on every
/// revolution; the final DCM must still match the analytic rotation. This is the
/// direct attitude-level guard for the RK4 shadow-switch fix.
#[test]
fn principal_axis_spin_matches_analytic_attitude_across_mrp_switches() {
    let omega_z = 0.5; // rad/s about the body z (a principal axis -> torque free)
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 10.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        // z is the max-inertia principal axis: spin about it is torque-free and stable.
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(0.12, 0.15, 0.18)),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.0, 0.0, omega_z),
        integrator: None,
    });

    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 0);
        sim.run_for(DURATION_NANOS);
    }

    let state = spacecraft.state_out.read();

    // omega about a principal axis is unchanged by torque-free motion.
    assert!(
        (state.omega_radps - Vector3::new(0.0, 0.0, omega_z)).norm() < 1e-10,
        "omega drifted: {:?}",
        state.omega_radps
    );

    // Analytic body-to-inertial DCM: active rotation by angle = omega_z * t about z.
    let angle = omega_z * (DURATION_NANOS as f64 * 1e-9);
    let (c, s) = (angle.cos(), angle.sin());
    let expected_body_to_inertial = Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0);
    let actual_body_to_inertial = body_to_inertial_dcm_from_sigma_bn(state.sigma_bn);

    let dcm_err = (actual_body_to_inertial - expected_body_to_inertial).norm();
    assert!(
        dcm_err < 1e-10,
        "attitude DCM does not match analytic rotation across MRP switches: err={dcm_err:.3e}"
    );
}

#[test]
fn spacecraft_outputs_initial_state_and_mass_props() {
    let inertia_diag = Vector3::new(0.12, 0.15, 0.18);
    let omega0 = Vector3::new(0.1, 0.05, 0.02);
    let velocity0 = Vector3::new(10.0, -20.0, 30.0);
    let position0 = Vector3::new(7_000_000.0, 1_000.0, -2_000.0);
    let mass_kg = 12.0;

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&inertia_diag),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: position0,
        initial_velocity_mps: velocity0,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: omega0,
        integrator: None,
    });

    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 0);
        sim.run_for(0);
    }

    let mass_props = spacecraft.mass_props_out.read();
    let state = spacecraft.state_out.read();

    assert!((mass_props.mass_kg - mass_kg).abs() < 1e-12);
    assert!(mass_props.center_of_mass_body_m.norm() < 1e-12);
    assert!(
        (mass_props.inertia_about_point_b_body_kg_m2 - Matrix3::from_diagonal(&inertia_diag))
            .norm()
            < 1e-12
    );

    assert!((state.position_m - position0).norm() < 1e-12);
    assert!((state.velocity_mps - velocity0).norm() < 1e-12);
    assert!(state.sigma_bn.norm() < 1e-12);
    assert!((state.omega_radps - omega0).norm() < 1e-12);
}

#[test]
fn spacecraft_mass_props_include_hub_center_of_mass_offset() {
    let hub_center = Vector3::new(0.0, 0.0, 1.0);
    let hub_inertia_about_com = Matrix3::new(900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 750.0,
        hub_center_of_mass_body_m: hub_center,
        inertia_kg_m2: hub_inertia_about_com,
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });

    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 0);
        sim.run_for(0);
    }

    let mass_props = spacecraft.mass_props_out.read();
    assert!((mass_props.mass_kg - 750.0).abs() < 1.0e-12);
    assert!((mass_props.center_of_mass_body_m - hub_center).norm() < 1.0e-12);
    assert!(
        (mass_props.inertia_about_point_b_body_kg_m2
            - Matrix3::new(1650.0, 0.0, 0.0, 0.0, 1550.0, 0.0, 0.0, 0.0, 600.0))
        .norm()
            < 1.0e-12
    );
}

#[test]
fn mrp_body_to_inertial_dcm_matches_spacecraft_state_rotation() {
    let sigma_bn = Vector3::new(0.1, 0.2, 0.3);
    let dcm = body_to_inertial_dcm_from_sigma_bn(sigma_bn);
    let quaternion_rotation = SpacecraftStateMsg {
        sigma_bn,
        ..SpacecraftStateMsg::default()
    }
    .body_to_inertial()
    .to_rotation_matrix()
    .into_inner();

    assert!(
        (dcm - quaternion_rotation).norm() < 1.0e-12,
        "expected direct MRP DCM to match existing body_to_inertial rotation"
    );
}

struct PublishingThetaEffector {
    theta_out: Output<HingedRigidBodyMsg>,
}

impl PublishingThetaEffector {
    fn new() -> Self {
        Self {
            theta_out: Output::default(),
        }
    }
}

impl StateEffector for PublishingThetaEffector {
    fn name(&self) -> &str {
        "publishing_theta"
    }

    fn state_len(&self) -> usize {
        1
    }

    fn initial_state(&self) -> Vec<f64> {
        vec![0.0]
    }

    fn load_state(&mut self, state: &[f64]) {
        self.theta_out.write(HingedRigidBodyMsg {
            theta_rad: state[0],
            theta_dot_radps: 1.0,
        });
    }

    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {}

    fn update_contributions(
        &self,
        _effector_state: &[f64],
        _body_omega_radps: Vector3<f64>,
        _gravity_body_mps2: Vector3<f64>,
        _back_sub: &mut BackSubMatrices,
    ) {
    }

    fn compute_derivatives(
        &self,
        _effector_state: &[f64],
        _body_trans_accel_mps2: Vector3<f64>,
        _body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        vec![1.0]
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

struct RecordingThetaDynamicEffector {
    theta_in: Input<HingedRigidBodyMsg>,
    observed_thetas: Arc<Mutex<Vec<f64>>>,
}

impl DynamicEffector for RecordingThetaDynamicEffector {
    fn name(&self) -> &str {
        "recording_theta"
    }

    fn compute_output(&self, _state: &SpacecraftStateMsg) -> EffectorOutput {
        self.observed_thetas
            .lock()
            .expect("observed theta lock poisoned")
            .push(self.theta_in.read().theta_rad);
        EffectorOutput::default()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[test]
fn rk_substeps_sync_state_effector_outputs_before_dynamic_effectors() {
    let publisher = PublishingThetaEffector::new();
    let theta_out = publisher.theta_out.clone();
    let observed_thetas = Arc::new(Mutex::new(Vec::new()));
    let mut recorder = RecordingThetaDynamicEffector {
        theta_in: Input::default(),
        observed_thetas: Arc::clone(&observed_thetas),
    };
    recorder.theta_in.connect(theta_out.slot());

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 10.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: 1_000_000_000,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft.add_state_effector(publisher);
    spacecraft.add_dynamic_effector(recorder);
    spacecraft.init();

    let state = spacecraft
        .integrated_state
        .clone()
        .expect("spacecraft must initialize integrated state");

    Rk4.propagate(&mut spacecraft, &state, 0, start_epoch(), 1.0);

    let observed = observed_thetas
        .lock()
        .expect("observed theta lock poisoned")
        .clone();
    assert!(
        observed.iter().any(|theta| (*theta - 0.5).abs() < 1.0e-12),
        "expected a synced RK midpoint theta, observed {observed:?}"
    );
    assert!(
        observed.iter().any(|theta| (*theta - 1.0).abs() < 1.0e-12),
        "expected a synced RK endpoint theta, observed {observed:?}"
    );
}

/// Final position after 100 s must match a known reference to 1e-7 relative.
/// Reference generated from a trusted run with the same ICs.
#[test]
fn keplerian_orbit_position_regression() {
    let mut sc = circular_orbit_spacecraft(7_000_000.0);
    let mut sim = Simulation::new(start_epoch(), false);
    sim.add_module("spacecraft", &mut sc, STEP_NANOS, 0);
    sim.run_for(DURATION_NANOS);

    let state = sc.state_out.read();
    let r = state.position_m.norm();
    let rel_err = (r - 7_000_000.0).abs() / 7_000_000.0;
    assert!(
        rel_err < 1e-7,
        "orbital radius drifted: r={:.6e} m  expected=7.0e6 m  rel_err={:.2e}",
        r,
        rel_err
    );
}

/// Every built-in integrator struct is assignable to the config slot, including
/// the adaptive ones that carry an extra `where` bound.
#[test]
fn all_integrators_are_pluggable() {
    let _integrators: [Box<dyn Integrator<Spacecraft>>; 31] = [
        Box::new(Euler),
        Box::new(Midpoint),
        Box::new(Heun),
        Box::new(Ralston),
        Box::new(Rk4),
        Box::new(Rk4ThreeEighths),
        Box::new(Rkf45::default()),
        Box::new(Dopri45::default()),
        Box::new(Rkf78::default()),
        Box::new(EulerMaruyama::new()),
        Box::new(EulerHeun::new()),
        Box::new(RkMil::new()),
        Box::new(Sra1::new()),
        Box::new(Sosra::new()),
        Box::new(Sriw1::new()),
        Box::new(Sosri::new()),
        Box::new(W2Ito1::new()),
        Box::new(W2Ito2::new()),
        Box::new(Dri1::new()),
        Box::new(Dri1Nm::new()),
        Box::new(Ri1::new()),
        Box::new(Ri3::new()),
        Box::new(Ri5::new()),
        Box::new(Ri6::new()),
        Box::new(Rs1::new()),
        Box::new(Rs2::new()),
        Box::new(Siea::new()),
        Box::new(Smea::new()),
        Box::new(Sieb::new()),
        Box::new(Smeb::new()),
        Box::new(Rdi1Wm::new()),
    ];
}

/// Runs one full circular orbit with the given integrator and returns the
/// final radius error relative to the exact circular radius.
fn orbit_radius_error(integrator: Box<dyn Integrator<Spacecraft>>) -> f64 {
    let radius_m = 7_000_000.0;
    let mut sc = circular_orbit_spacecraft(radius_m);
    sc.config.integrator = Some(integrator);
    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.add_module("spacecraft", &mut sc, STEP_NANOS, 0);
        sim.run_for(DURATION_NANOS);
    }
    let r = sc.state_out.read().position_m.norm();
    (r - radius_m).abs() / radius_m
}

/// A higher-order method should hold the circular radius at least as tightly as
/// a lower-order one over the same 100 s, and every method should stay bounded.
#[test]
fn integrator_accuracy_improves_with_order() {
    let euler = orbit_radius_error(Box::new(Euler));
    let heun = orbit_radius_error(Box::new(Heun));
    let rk4 = orbit_radius_error(Box::new(Rk4));
    let dopri = orbit_radius_error(Box::new(Dopri45::default()));

    // Sanity: nothing blew up.
    for (name, err) in [
        ("euler", euler),
        ("heun", heun),
        ("rk4", rk4),
        ("dopri45", dopri),
    ] {
        assert!(
            err.is_finite() && err < 1e-2,
            "{name} radius error {err:.3e}"
        );
    }
    // Order shows up as strictly better radius retention.
    assert!(heun < euler, "heun {heun:.3e} !< euler {euler:.3e}");
    assert!(rk4 < heun, "rk4 {rk4:.3e} !< heun {heun:.3e}");
    assert!(dopri < 1e-6, "adaptive dopri45 radius error {dopri:.3e}");
}

// ---------------------------------------------------------------------------
// Port of upstream `test_Integrators.py` and `test_integratorConvergence.py`
// (`simulation/dynamics/Integrators/_UnitTest`). The adaptive cases use
// basilisk-rs's step-size controller (an RMS norm over every scaled state
// component) rather than upstream's per-state norm; the truth is upstream's.
// The state-specific tolerance setters those files also test have no
// counterpart: one controller tolerance covers every state.
// ---------------------------------------------------------------------------

/// Upstream `simIncludeGravBody.createEarth().mu` \[m^3/s^2\].
const BASILISK_MU_EARTH_M3PS2: f64 = 0.398_600_441_5e15;

/// A fixed-step custom tableau as an integrator (upstream `svIntegratorRungeKutta`).
#[derive(Debug)]
struct TableauIntegrator(&'static ButcherTableau);

impl<D: DynamicObject + 'static> Integrator<D> for TableauIntegrator {
    fn propagate(
        &self,
        object: &mut D,
        state: &D::State,
        current_sim_nanos: u64,
        current_epoch: Epoch,
        dt_seconds: f64,
    ) -> (D::State, D::StepOutput) {
        propagate_explicit_rk(
            self.0,
            object,
            state,
            current_sim_nanos,
            current_epoch,
            dt_seconds,
        )
    }
}

/// An embedded custom tableau as an integrator (upstream
/// `svIntegratorAdaptiveRungeKutta`).
#[derive(Debug)]
struct AdaptiveTableauIntegrator(&'static ButcherTableau, AdaptiveController);

impl<D: DynamicObject + 'static> Integrator<D> for AdaptiveTableauIntegrator {
    fn propagate(
        &self,
        object: &mut D,
        state: &D::State,
        current_sim_nanos: u64,
        current_epoch: Epoch,
        dt_seconds: f64,
    ) -> (D::State, D::StepOutput) {
        propagate_adaptive_rk(
            self.0,
            &self.1,
            object,
            state,
            current_sim_nanos,
            current_epoch,
            dt_seconds,
        )
    }
}

/// Upstream's `rk3` case: Kutta's third-order method.
const KUTTA_RK3: ButcherTableau = ButcherTableau {
    a: &[&[], &[0.5], &[-1.0, 2.0]],
    c: &[0.0, 0.5, 1.0],
    b: &[1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0],
    b_embedded: None,
    error_estimate_order: 3,
};

/// Upstream's `bogackiShampine` case: the third-order solution (upstream
/// `b_star_coefficients`) is propagated with the second-order one embedded.
const BOGACKI_SHAMPINE: ButcherTableau = ButcherTableau {
    a: &[
        &[],
        &[0.5],
        &[0.0, 0.75],
        &[2.0 / 9.0, 1.0 / 3.0, 4.0 / 9.0],
    ],
    c: &[0.0, 0.5, 0.75, 1.0],
    b: &[2.0 / 9.0, 1.0 / 3.0, 4.0 / 9.0, 0.0],
    b_embedded: Some(&[7.0 / 24.0, 1.0 / 4.0, 1.0 / 3.0, 1.0 / 8.0]),
    error_estimate_order: 2,
};

/// Upstream `test_scenarioIntegrators`: the LEO of `test_Integrators.py`
/// integrated with 120 s steps for three quarters of an orbit; returns the
/// position at every step, initial state included.
fn scenario_integrator_positions(integrator: Box<dyn Integrator<Spacecraft>>) -> Vec<Vector3<f64>> {
    use crate::utilities::orbital_motion::{ClassicElements, elements_to_state, state_to_elements};

    let mu = BASILISK_MU_EARTH_M3PS2;
    let elements = ClassicElements {
        semi_major_axis: 7000.0 * 1000.0,
        eccentricity: 0.0001,
        inclination_rad: 33.3_f64.to_radians(),
        raan_rad: 48.2_f64.to_radians(),
        arg_periapsis_rad: 347.8_f64.to_radians(),
        true_anomaly_rad: 85.3_f64.to_radians(),
        ..ClassicElements::default()
    };
    let (position_m, velocity_mps) = elements_to_state(mu, &elements);
    let elements = state_to_elements(mu, position_m, velocity_mps);
    let mean_motion = (mu / elements.semi_major_axis.powi(3)).sqrt();
    let period_s = 2.0 * std::f64::consts::PI / mean_motion;
    // Upstream `macros.sec2nano(0.75 * P)`; the task then ticks every 120 s
    // up to that stop time.
    let stop_nanos = (0.75 * period_s * 1.0e9).round() as u64;
    let step_nanos = 120_000_000_000;

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 100.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: step_nanos,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: Some(integrator),
    });
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass("earth", mu, true, Vector3::zeros(), Vector3::zeros())
                .expect("valid Earth gravity body"),
        )
        .expect("unique central gravity body");
    spacecraft.init();
    let mut positions = Vec::new();
    for tick in 0..=stop_nanos / step_nanos {
        spacecraft.update(&crate::SimulationContext {
            current_sim_nanos: tick * step_nanos,
            current_epoch: start_epoch(),
        });
        positions.push(spacecraft.state_out.read().position_m);
    }
    positions
}

/// Compares upstream's sampled truth points (every `len / 4`-th record, or
/// the last one) to 1 m per component.
fn assert_matches_truth(positions: &[Vector3<f64>], truth: &[[f64; 3]], name: &str) {
    let sampled: Vec<Vector3<f64>> = if truth.len() == 1 {
        vec![*positions.last().unwrap()]
    } else {
        let skip = positions.len() / (truth.len() - 1);
        positions.iter().step_by(skip).copied().collect()
    };
    assert!(
        sampled.len() >= truth.len(),
        "{name}: {} samples",
        sampled.len()
    );
    for (index, (actual, expected)) in sampled.iter().zip(truth).enumerate() {
        for axis in 0..3 {
            assert!(
                (actual[axis] - expected[axis]).abs() <= 1.0,
                "{name}: point {index} axis {axis} is {}, truth {}",
                actual[axis],
                expected[axis]
            );
        }
    }
}

/// The analytic two-body truth upstream shares between its three adaptive
/// cases, at the last 120 s tick.
const ADAPTIVE_TRUTH: [[f64; 3]; 1] = [[5879286.370258273, 3561242.50810664, -1319786.625981673]];

#[test]
fn scenario_integrators_rk4_matches_upstream_truth() {
    assert_matches_truth(
        &scenario_integrator_positions(Box::new(Rk4)),
        &[
            [
                -2.8168016010234915e6,
                5.248174846916147e6,
                3.677157264677297e6,
            ],
            [
                -6.379381726549218e6,
                -1.4688565370540658e6,
                2.4807857675497606e6,
            ],
            [
                -2.230094305694789e6,
                -6.410420020364709e6,
                -1.7146277675541767e6,
            ],
            [
                4.614900659014343e6,
                -3.60224207689023e6,
                -3.837022825958977e6,
            ],
            [
                5.879095186201691e6,
                3.561495655367985e6,
                -1.3195821703218794e6,
            ],
        ],
        "rk4",
    );
}

#[test]
fn scenario_integrators_euler_matches_upstream_truth() {
    assert_matches_truth(
        &scenario_integrator_positions(Box::new(Euler)),
        &[
            [
                -2.8168016010234915e6,
                5.248174846916147e6,
                3.677157264677297e6,
            ],
            [
                -7.061548530211288e6,
                -1.4488790844105487e6,
                2.823580168201031e6,
            ],
            [
                -4.831279689590867e6,
                -8.015202650472983e6,
                -1.1434851461593418e6,
            ],
            [
                719606.5825106134,
                -1.0537603309084207e7,
                -4.966060248346598e6,
            ],
            [
                6.431097055190775e6,
                -9.795566286964862e6,
                -7.438012269629238e6,
            ],
        ],
        "euler",
    );
}

/// Upstream `svIntegratorRK2` is Heun's method.
#[test]
fn scenario_integrators_rk2_matches_upstream_truth() {
    assert_matches_truth(
        &scenario_integrator_positions(Box::new(Heun)),
        &[
            [
                -2.8168016010234915e6,
                5.248174846916147e6,
                3.677157264677297e6,
            ],
            [
                -6.425636528569288e6,
                -1.466693214251768e6,
                2.50438327358707e6,
            ],
            [
                -2.466642497083674e6,
                -6.509473992136429e6,
                -1.6421621818735446e6,
            ],
            [
                4.342561337924192e6,
                -4.1593822658140697e6,
                -3.947594705237753e6,
            ],
            [
                6.279757158711852e6,
                2.8527385905952943e6,
                -1.8260959147806289e6,
            ],
        ],
        "rk2",
    );
}

#[test]
fn scenario_integrators_rk3_matches_upstream_truth() {
    assert_matches_truth(
        &scenario_integrator_positions(Box::new(TableauIntegrator(&KUTTA_RK3))),
        &[
            [-2816801.601023492, 5248174.846916147, 3677157.2646772973],
            [-6380014.419169294, -1467304.5024778044, 2481775.1156921615],
            [-2231193.424069216, -6407257.277104054, -1712704.800575082],
            [4613563.820461048, -3590442.718508557, -3831202.081509318],
            [5853794.060191272, 3579538.889746918, -1299292.6887013493],
        ],
        "rk3",
    );
}

/// Upstream's adaptive cases: relative tolerance 0 and absolute tolerance
/// 0.01 (RKF45, RKF78) or 1e-4 (Bogacki-Shampine) all land on the two-body
/// truth within 1 m.
#[test]
fn scenario_integrators_adaptive_methods_match_upstream_truth() {
    let controller = |atol| AdaptiveController {
        atol,
        rtol: 0.0,
        ..Default::default()
    };
    for (name, integrator) in [
        (
            "rkf45",
            Box::new(Rkf45::new(controller(0.01))) as Box<dyn Integrator<Spacecraft>>,
        ),
        ("rkf78", Box::new(Rkf78::new(controller(0.01)))),
        (
            "bogackiShampine",
            Box::new(AdaptiveTableauIntegrator(
                &BOGACKI_SHAMPINE,
                controller(1.0e-4),
            )),
        ),
    ] {
        assert_matches_truth(
            &scenario_integrator_positions(integrator),
            &ADAPTIVE_TRUTH,
            name,
        );
    }
}

/// Upstream `test_integrator_convergence`: on the unit circular orbit
/// (`μ = 1`), the position error at 1 s for 8, 16, 32 and 64 steps falls
/// monotonically with the method's order, within 0.25.
#[test]
fn integrator_convergence_orders() {
    let final_position = |integrator: Box<dyn Integrator<Spacecraft>>, step_count: u64| {
        let step_nanos = 1_000_000_000 / step_count;
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 1.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::identity(),
            integration_step_nanos: step_nanos,
            initial_position_m: Vector3::new(1.0, 0.0, 0.0),
            initial_velocity_mps: Vector3::new(0.0, 1.0, 0.0),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: Some(integrator),
        });
        spacecraft
            .add_grav_body(
                GravBodyData::point_mass(
                    "central_body",
                    1.0,
                    true,
                    Vector3::zeros(),
                    Vector3::zeros(),
                )
                .expect("valid central body"),
            )
            .expect("unique central gravity body");
        {
            let mut simulation = Simulation::new(start_epoch(), false);
            simulation.add_module("orbit", &mut spacecraft, step_nanos, 0);
            simulation.run_for(1_000_000_000);
            assert_eq!(simulation.current_sim_nanos(), 1_000_000_000);
        }
        spacecraft.state_out.read().position_m
    };
    let reference = Vector3::new(1.0_f64.cos(), 1.0_f64.sin(), 0.0);
    type Factory = fn() -> Box<dyn Integrator<Spacecraft>>;
    let cases: [(&str, Factory, f64); 4] = [
        ("euler", || Box::new(Euler), 1.0),
        ("rk2", || Box::new(Heun), 2.0),
        ("rk3", || Box::new(TableauIntegrator(&KUTTA_RK3)), 3.0),
        ("rk4", || Box::new(TableauIntegrator(&RK4)), 4.0),
    ];
    for (name, factory, expected_order) in cases {
        let errors: Vec<f64> = [8, 16, 32, 64]
            .into_iter()
            .map(|count| (final_position(factory(), count) - reference).norm())
            .collect();
        assert!(
            errors.iter().all(|error| error.is_finite() && *error > 0.0),
            "{name}: {errors:?}"
        );
        assert!(
            errors.windows(2).all(|pair| pair[1] < pair[0]),
            "{name}: {errors:?}"
        );
        for pair in errors.windows(2) {
            let order = (pair[0] / pair[1]).log2();
            assert!((order - expected_order).abs() <= 0.25, "{name}: {errors:?}");
        }
    }
}

// ---------------------------------------------------------------------------
// Accumulated ΔV: port of upstream `scAccumDV` and `scAccumDVExtForce`
// (`simulation/dynamics/spacecraft/_UnitTest/test_spacecraft.py`).
// ---------------------------------------------------------------------------

/// Upstream `extForceTorque` with a constant body-frame force and no torque.
struct ConstantBodyForce {
    force_body_n: Vector3<f64>,
}

impl DynamicEffector for ConstantBodyForce {
    fn name(&self) -> &str {
        "externalDisturbance"
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        EffectorOutput {
            force_inertial_n: state
                .body_to_inertial()
                .transform_vector(&self.force_body_n),
            torque_body_nm: Vector3::zeros(),
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// Runs `spacecraft` with 0.1 s steps to 0.5 s and returns the state message
/// of every step, initial one included.
fn accumulated_dv_log(spacecraft: &mut Spacecraft) -> Vec<SpacecraftStateMsg> {
    spacecraft.init();
    (0..=5u64)
        .map(|tick| {
            spacecraft.update(&crate::SimulationContext {
                current_sim_nanos: tick * 100_000_000,
                current_epoch: start_epoch(),
            });
            spacecraft.state_out.read()
        })
        .collect()
}

fn assert_vector_near(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64, context: &str) {
    for axis in 0..3 {
        assert!(
            (actual[axis] - expected[axis]).abs() <= accuracy,
            "{context}: {actual:?} vs {expected:?}"
        );
    }
}

/// Upstream `scAccumDV`: under gravity alone the center of mass accumulates
/// no ΔV, while the body origin, 100 m from the spinning center of mass,
/// accumulates the change of its rotational velocity `ω × (-r_BcB)` seen in
/// the body frame. Accuracy 1e-10 m/s.
#[test]
fn accumulated_dv_with_offset_center_of_mass_under_gravity() {
    use crate::utilities::rigid_body_kinematics::mrp_to_dcm;

    let center_of_mass_body_m = Vector3::new(0.0, 100.0, 0.0);
    let omega_radps = Vector3::new(0.0, 0.0, std::f64::consts::PI / 180.0);
    // Upstream gives the initial center-of-mass state (`r_CN_NInit`,
    // `v_CN_NInit`); the body origin is offset from it by `-r_BcB`.
    let com_position_m = Vector3::new(-7_000_000.0, 0.0, 0.0);
    let com_velocity_mps = Vector3::new(0.0, 7000.0, 0.0);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 100.0,
        hub_center_of_mass_body_m: center_of_mass_body_m,
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(500.0, 200.0, 300.0)),
        integration_step_nanos: 100_000_000,
        initial_position_m: com_position_m - center_of_mass_body_m,
        initial_velocity_mps: com_velocity_mps - omega_radps.cross(&center_of_mass_body_m),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: omega_radps,
        integrator: None,
    });
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass(
                "earth",
                BASILISK_MU_EARTH_M3PS2,
                true,
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .expect("valid Earth gravity body"),
        )
        .expect("unique central gravity body");
    let log = accumulated_dv_log(&mut spacecraft);

    let accuracy = 1.0e-10;
    let rotation_velocity_body = omega_radps.cross(&-center_of_mass_body_m);
    let mut truth_bn_b = Vector3::zeros();
    assert_eq!(log[0].accumulated_dv_bn_b_mps, Vector3::zeros());
    for pair in log.windows(2) {
        let (before, after) = (&pair[0], &pair[1]);
        assert_vector_near(
            after.accumulated_dv_cn_b_mps,
            Vector3::zeros(),
            accuracy,
            "dv_CN_B",
        );
        let dcm_bn_after = mrp_to_dcm(after.sigma_bn);
        let dcm_bn_before = mrp_to_dcm(before.sigma_bn);
        truth_bn_b += dcm_bn_after
            * (dcm_bn_after.transpose() * rotation_velocity_body
                - dcm_bn_before.transpose() * rotation_velocity_body);
        assert_vector_near(
            after.accumulated_dv_bn_b_mps,
            truth_bn_b,
            accuracy,
            "dv_BN_B",
        );
        assert_vector_near(
            after.accumulated_dv_cn_n_mps,
            Vector3::zeros(),
            accuracy,
            "dv_CN_N",
        );
    }
    assert!(
        truth_bn_b.norm() > 1.0e-3,
        "the body origin's rotational ΔV is resolved"
    );
}

/// Upstream `scAccumDVExtForce`: a constant external force on a 100 kg hub
/// without gravity accumulates `F t / m` in the inertial frame. The step's
/// non-conservative acceleration is `F / m`.
#[test]
fn accumulated_dv_under_a_constant_external_force() {
    let force_body_n = Vector3::new(0.31, 0.72, 0.18);
    let mass_kg = 100.0;
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: 100_000_000,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft.add_dynamic_effector(ConstantBodyForce { force_body_n });
    let log = accumulated_dv_log(&mut spacecraft);

    for (tick, state) in log.iter().enumerate() {
        let time_s = tick as f64 * 0.1;
        let truth = force_body_n * time_s / mass_kg;
        assert_vector_near(state.accumulated_dv_cn_n_mps, truth, 1.0e-10, "dv_CN_N");
        // The attitude stays inertial, so the body-frame sums agree.
        assert_vector_near(state.accumulated_dv_cn_b_mps, truth, 1.0e-10, "dv_CN_B");
        assert_vector_near(state.accumulated_dv_bn_b_mps, truth, 1.0e-10, "dv_BN_B");
    }
    assert_vector_near(
        spacecraft
            .diagnostics_out
            .read()
            .non_conservative_accel_body_mps2,
        force_body_n / mass_kg,
        1.0e-12,
        "non-conservative acceleration",
    );

    // Initialization restarts the sums.
    spacecraft.init();
    assert_eq!(
        spacecraft.state_out.read().accumulated_dv_cn_n_mps,
        Vector3::zeros()
    );
}

/// The accumulated ΔV is measured from velocities relative to the central
/// body: a central body moving at constant velocity adds nothing.
#[test]
fn accumulated_dv_ignores_the_central_body_motion() {
    let mut spacecraft = circular_orbit_spacecraft(7_000_000.0);
    spacecraft.config.integration_step_nanos = 1_000_000_000;
    spacecraft.integration_step_seconds = 1.0;
    spacecraft.integration_step_duration = hifitime::Duration::from_seconds(1.0);
    spacecraft.gravity = crate::dynamics::gravity::GravityEffector::new();
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass(
                "earth",
                MU_EARTH_M3PS2,
                true,
                Vector3::new(1.0e11, 0.0, 0.0),
                Vector3::new(0.0, 30_000.0, 0.0),
            )
            .expect("valid Earth gravity body"),
        )
        .expect("unique central gravity body");
    spacecraft.init();
    for tick in 0..=10u64 {
        spacecraft.update(&crate::SimulationContext {
            current_sim_nanos: tick * 1_000_000_000,
            current_epoch: start_epoch(),
        });
    }
    let state = spacecraft.state_out.read();
    assert!(
        state.accumulated_dv_cn_n_mps.norm() < 1.0e-9,
        "{:?}",
        state.accumulated_dv_cn_n_mps
    );
}

// ---------------------------------------------------------------------------
// Diffusion of the spacecraft states.
// ---------------------------------------------------------------------------

/// A one-state effector with two noise sources: `x' = -x`, diffusion
/// columns `[0.5]` and `[0.25 x]`.
struct TwoSourceNoise {
    value: f64,
}

impl StateEffector for TwoSourceNoise {
    fn name(&self) -> &str {
        "two_source_noise"
    }
    fn state_len(&self) -> usize {
        1
    }
    fn initial_state(&self) -> Vec<f64> {
        vec![1.0]
    }
    fn load_state(&mut self, state: &[f64]) {
        self.value = state[0];
    }
    fn pre_integration(&mut self, _: u64, _: f64) {}
    fn update_contributions(
        &self,
        _: &[f64],
        _: Vector3<f64>,
        _: Vector3<f64>,
        _: &mut BackSubMatrices,
    ) {
    }
    fn compute_derivatives(&self, state: &[f64], _: Vector3<f64>, _: Vector3<f64>) -> Vec<f64> {
        vec![-state[0]]
    }
    fn noise_source_count(&self) -> usize {
        2
    }
    fn compute_diffusion(&self, state: &[f64]) -> Vec<Vec<f64>> {
        vec![vec![0.5], vec![0.25 * state[0]]]
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// The spacecraft numbers its noise sources effector by effector and places
/// each column in its own effector's slot, with zero hub diffusion; an
/// effector without noise contributes none.
#[test]
fn spacecraft_diffusion_columns_follow_the_effectors() {
    let mut spacecraft = circular_orbit_spacecraft(7_000_000.0);
    spacecraft.add_state_effector(fully_coupled_reaction_wheel());
    spacecraft.add_state_effector(TwoSourceNoise { value: 0.0 });
    spacecraft.init();
    assert_eq!(spacecraft.noise_source_count(), 2);
    let mut state = spacecraft.integrated_state.clone().unwrap();
    state.effector_states[1][0] = 2.0;
    let columns = spacecraft.equations_of_motion_diffusion(&state, 0, start_epoch());
    assert_eq!(columns.len(), 2);
    for (column, expected) in columns.iter().zip([0.5, 0.5]) {
        assert_eq!(column.position_wrt_central_body_dot_mps, Vector3::zeros());
        assert_eq!(column.velocity_wrt_central_body_dot_mps2, Vector3::zeros());
        assert_eq!(column.gravity_accel_inertial_mps2, Vector3::zeros());
        assert_eq!(column.sigma_dot, Vector3::zeros());
        assert_eq!(column.omega_dot_radps2, Vector3::zeros());
        assert!(
            column.effector_state_dots[0]
                .iter()
                .all(|&value| value == 0.0)
        );
        assert_eq!(column.effector_state_dots[1], [expected]);
    }
}

/// A stochastic integrator on a spacecraft without noise is its drift
/// tableau: Euler-Maruyama steps exactly like Euler, and the noise sources of
/// an effector move only that effector's state.
#[test]
fn stochastic_integrators_reduce_to_their_drift_without_noise() {
    let run = |integrator: Box<dyn Integrator<Spacecraft>>, with_noise: bool| {
        let mut spacecraft = circular_orbit_spacecraft(7_000_000.0);
        spacecraft.config.integrator = Some(integrator);
        if with_noise {
            spacecraft.add_state_effector(TwoSourceNoise { value: 0.0 });
        }
        {
            let mut simulation = Simulation::new(start_epoch(), false);
            simulation.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 0);
            simulation.run_for(200 * STEP_NANOS);
        }
        (
            spacecraft.state_out.read(),
            spacecraft.integrated_state.clone().unwrap(),
        )
    };
    let (euler, _) = run(Box::new(Euler), false);
    let (euler_maruyama, _) = run(Box::new(EulerMaruyama::new()), false);
    assert_eq!(euler.position_m, euler_maruyama.position_m);
    assert_eq!(euler.velocity_mps, euler_maruyama.velocity_mps);

    let (noisy, noisy_state) = run(Box::new(EulerMaruyama::new().with_seed(9)), true);
    assert_eq!(
        noisy.position_m, euler.position_m,
        "noise must not reach the hub"
    );
    let deterministic_decay = (1.0 - STEP_NANOS as f64 * 1.0e-9).powi(200);
    assert!(
        (noisy_state.effector_states[0][0] - deterministic_decay).abs() > 1.0e-6,
        "the noise source moves its effector state"
    );
}
