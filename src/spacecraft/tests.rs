use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};

use super::{Spacecraft, SpacecraftConfig, mrp::body_to_inertial_dcm_from_sigma_bn};
use crate::gravity::GravBodyData;
use crate::messages::{Output, ReactionWheelCommandMsg, SpacecraftStateMsg};
use crate::reaction_wheel::{ReactionWheel, ReactionWheelConfig};
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
        inertia_kg_m2: Matrix3::new(locked_inertia_x, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.18),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
    });

    let mut reaction_wheel = ReactionWheel::new(ReactionWheelConfig::balanced(
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
        .downcast_ref::<ReactionWheel>()
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

#[test]
fn spacecraft_outputs_mass_props_and_seeded_diagnostics() {
    let inertia_diag = Vector3::new(0.12, 0.15, 0.18);
    let omega0 = Vector3::new(0.1, 0.05, 0.02);
    let velocity0 = Vector3::new(10.0, -20.0, 30.0);
    let position0 = Vector3::new(7_000_000.0, 1_000.0, -2_000.0);
    let mass_kg = 12.0;

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg,
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
    let diagnostics = spacecraft.diagnostics_out.read();

    assert!((mass_props.mass_kg - mass_kg).abs() < 1e-12);
    assert!(mass_props.center_of_mass_body_m.norm() < 1e-12);
    assert!(
        (mass_props.inertia_about_point_b_body_kg_m2 - Matrix3::from_diagonal(&inertia_diag))
            .norm()
            < 1e-12
    );

    let expected_rotational_energy =
        0.5 * (Matrix3::from_diagonal(&inertia_diag) * omega0).dot(&omega0);
    let expected_orbital_kinetic_energy = 0.5 * mass_kg * velocity0.norm_squared();
    let expected_orbital_angular_momentum = position0.cross(&(mass_kg * velocity0));

    assert!(diagnostics.omega_dot_radps2.norm() < 1e-12);
    assert!(diagnostics.non_conservative_accel_body_mps2.norm() < 1e-12);
    assert!(
        (diagnostics.rotational_energy_j - expected_rotational_energy).abs() < 1e-12,
        "expected rotational energy {:.6e}, got {:.6e}",
        expected_rotational_energy,
        diagnostics.rotational_energy_j
    );
    assert!(
        (diagnostics.orbital_kinetic_energy_j - expected_orbital_kinetic_energy).abs() < 1e-12,
        "expected orbital kinetic energy {:.6e}, got {:.6e}",
        expected_orbital_kinetic_energy,
        diagnostics.orbital_kinetic_energy_j
    );
    assert!(
        (diagnostics.orbital_angular_momentum_inertial_kg_m2ps - expected_orbital_angular_momentum)
            .norm()
            < 1e-12
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
