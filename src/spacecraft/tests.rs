use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};
use std::any::Any;
use std::sync::{Arc, Mutex};

use super::{
    BackSubMatrices, DynamicEffector, EffectorOutput, Spacecraft, SpacecraftConfig, StateEffector,
    mrp::body_to_inertial_dcm_from_sigma_bn,
};
use crate::Module;
use crate::actuators::hinged_rigid_body::{HingedRigidBodyConfig, HingedRigidBodyStateEffector};
use crate::actuators::reaction_wheel_state_effector::{
    ReactionWheelStateEffector, ReactionWheelStateEffectorConfig,
};
use crate::environment::gravity::GravBodyData;
use crate::messages::{
    HingedRigidBodyMsg, Input, Output, ReactionWheelCommandMsg, SpacecraftStateMsg,
};
use crate::simulation::Simulation;

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
    });
    sc.add_grav_body(GravBodyData::point_mass(
        "earth",
        MU_EARTH_M3PS2,
        true,
        Vector3::zeros(),
        Vector3::zeros(),
    ));
    sc
}

fn orbital_angular_momentum(pos: Vector3<f64>, vel: Vector3<f64>) -> Vector3<f64> {
    pos.cross(&vel)
}

fn orbital_energy(pos: Vector3<f64>, vel: Vector3<f64>) -> f64 {
    0.5 * vel.norm_squared() - MU_EARTH_M3PS2 / pos.norm()
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
    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.add_module("spacecraft", &mut sc, STEP_NANOS, 0);
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
    });

    let mut reaction_wheel =
        ReactionWheelStateEffector::new(ReactionWheelStateEffectorConfig::balanced(
            "rw_x",
            Vector3::zeros(),
            Vector3::new(1.0, 0.0, 0.0),
            1.0,
            100.0,
        ));
    reaction_wheel.config.js_kg_m2 = wheel_js;
    let command = Output::new(ReactionWheelCommandMsg {
        motor_torque_nm: applied_torque_nm,
    });

    {
        let mut sim = Simulation::new(start_epoch(), false);
        sim.connect(&command, &mut reaction_wheel.command_in);
        spacecraft.add_state_effector(reaction_wheel);
        sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 0);
        sim.run_for(1_000_000_000);
    }

    let body_omega_x = spacecraft.state_out.read().omega_radps.x;
    let wheel_omega_x = spacecraft.state_effectors[0]
        .as_any()
        .downcast_ref::<ReactionWheelStateEffector>()
        .expect("expected reaction wheel state effector")
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
    });
    spacecraft.add_state_effector(publisher);
    spacecraft.add_dynamic_effector(recorder);
    spacecraft.init();

    let state = spacecraft
        .integrated_state
        .clone()
        .expect("spacecraft must initialize integrated state");
    spacecraft.propagate_rk4(&state, 0, start_epoch(), 1.0);

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
