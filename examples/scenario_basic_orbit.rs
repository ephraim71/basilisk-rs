//! Scenario: Basic Orbit
//!
//! Port of Basilisk's `scenarioBasicOrbit.py`.
//!
//! Supports LEO / GTO / GEO orbit cases with point-mass or J2 spherical harmonics gravity,
//! matching Basilisk's `orbitCase` and `useSphericalHarmonics` parameters.
//!
//! Run with:
//!   cargo run --example scenario_basic_orbit                 # LEO, point-mass
//!   cargo run --example scenario_basic_orbit -- gto          # GTO, point-mass
//!   cargo run --example scenario_basic_orbit -- geo          # GEO, point-mass
//!   cargo run --example scenario_basic_orbit -- leo sh       # LEO, J2 spherical harmonics
//!   cargo run --example scenario_basic_orbit -- gto sh       # GTO, J2 spherical harmonics

use basilisk_rs::gravity::GravBodyData;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use hifitime::Epoch;
use nalgebra::{UnitQuaternion, Vector3};

#[derive(Clone, Copy, Debug)]
#[allow(dead_code)]
enum Orbit {
    /// a = 7 000 km, e = 0.0001, i = 33.3°  (default LEO case)
    Leo,
    /// a = 42 000 km, e = 0.00001, i = 0°
    Geo,
    /// a = (R_LEO + R_GEO) / 2, e = 1 − R_LEO/a, i = 0°
    Gto,
}

const MU_EARTH_M3PS2: f64 = 3.986_004_418e14;
const R_LEO_M: f64 = 7_000_000.0;
const R_GEO_M: f64 = 42_000_000.0;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let orbit_case = args.iter().find_map(|a| match a.to_lowercase().as_str() {
        "geo" => Some(Orbit::Geo),
        "gto" => Some(Orbit::Gto),
        _     => None,
    }).unwrap_or(Orbit::Leo);
    let use_sh = args.iter().any(|a| a.eq_ignore_ascii_case("sh"));

    let deg = std::f64::consts::PI / 180.0;

    // Common orbital angles — identical for all orbit cases (matches Basilisk)
    let raan = 48.2 * deg;
    let aop = 347.8 * deg;
    let ta0 = 85.3 * deg;

    let (a, e, inc) = match orbit_case {
        Orbit::Leo => (R_LEO_M, 0.0001, 33.3 * deg),
        Orbit::Geo => (R_GEO_M, 0.00001, 0.0_f64),
        Orbit::Gto => {
            let a = (R_LEO_M + R_GEO_M) / 2.0;
            let e = 1.0 - R_LEO_M / a;
            (a, e, 0.0_f64)
        }
    };

    let (r0, v0) = elem2rv(MU_EARTH_M3PS2, a, e, inc, raan, aop, ta0);
    let n = (MU_EARTH_M3PS2 / a.powi(3)).sqrt();
    let period = 2.0 * std::f64::consts::PI / n;

    let step_nanos: u64 = 10_000_000_000; // 10 s — matches Basilisk simulationTimeStep
    let num_orbits = if use_sh { 3.0 } else { 0.75 };
    let duration_nanos = (num_orbits * period * 1e9) as u64;
    let t_sim = duration_nanos as f64 * 1e-9;

    let gravity_label = if use_sh { "J2 spherical harmonics (GGM03S deg-2)" } else { "point-mass" };
    println!("=== scenario_basic_orbit ===");
    println!("Orbit:   {:?}  gravity: {}", orbit_case, gravity_label);
    println!("         a = {:.1} km  e = {:.5}  i = {:.1}°", a / 1e3, e, inc / deg);
    println!("         Ω = {:.1}°  ω = {:.1}°  f₀ = {:.1}°", 48.2, 347.8, 85.3);
    println!("Period:  {:.2} s  ({:.4} orbits simulated)", period, t_sim / period);
    println!("Steps:   {} × 10 s", duration_nanos / step_nanos);
    println!();
    println!("  t [s]      |r| [km]    r_x [km]    r_y [km]    r_z [km]");
    println!("{}", "-".repeat(65));

    println!(
        "{:10.1}   {:9.3}   {:10.3}   {:10.3}   {:10.3}",
        0.0,
        r0.norm() / 1e3,
        r0.x / 1e3,
        r0.y / 1e3,
        r0.z / 1e3,
    );

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 100.0,
        inertia_kg_m2: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        initial_position_m: r0,
        initial_velocity_mps: v0,
        initial_attitude_b_to_i: UnitQuaternion::identity(),
        initial_omega_radps: Vector3::zeros(),
    });

    if use_sh {
        spacecraft.add_grav_body(GravBodyData::spherical_harmonics_from_file(
            "earth",
            "assets/gravity/GGM03S.txt",
            2, // J2 only — degree 2, matches Basilisk useSphericalHarmonicsGravityModel(..., 2)
            true,
            Vector3::zeros(),
            Vector3::zeros(),
        ));
    } else {
        spacecraft.add_grav_body(GravBodyData::point_mass(
            "earth",
            MU_EARTH_M3PS2,
            true,
            Vector3::zeros(),
            Vector3::zeros(),
        ));
    }

    {
        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        let mut sim = Simulation::new(epoch, false);
        sim.add_module("spacecraft", &mut spacecraft, step_nanos, 0);
        sim.run_for(duration_nanos);
    }

    let r_sim = spacecraft.state_out.read().position_m;
    let v_sim = spacecraft.state_out.read().velocity_mps;

    println!(
        "{:10.1}   {:9.3}   {:10.3}   {:10.3}   {:10.3}",
        t_sim,
        r_sim.norm() / 1e3,
        r_sim.x / 1e3,
        r_sim.y / 1e3,
        r_sim.z / 1e3,
    );

    println!();

    if use_sh {
        // Basilisk SH case: track semi-major axis deviation over 3 orbits.
        // J2 causes RAAN/AoP precession but SMA should stay approximately constant.
        let a_final = sma_from_rv(MU_EARTH_M3PS2, r_sim, v_sim);
        println!("Initial SMA: {:.3} km", a / 1e3);
        println!("Final SMA:   {:.3} km  (Δa = {:.3} km)", a_final / 1e3, (a_final - a) / 1e3);
    } else {
        // Basilisk point-mass case: position deviation from analytical Keplerian solution.
        let r_kep = kepler_position(MU_EARTH_M3PS2, a, e, inc, raan, aop, ta0, t_sim);
        let final_diff = (r_sim - r_kep).norm();
        println!("finalDiff = {:.4e} m  (Basilisk reference: < 1 m)", final_diff);
        assert!(
            final_diff < 1.0,
            "trajectory deviated from Keplerian: finalDiff = {:.3e} m",
            final_diff
        );
    }
}

// --- Orbital mechanics helpers -----------------------------------------------

/// Classical orbital elements → inertial position and velocity.
/// Matches Basilisk `orbitalMotion.elem2rv`.
fn elem2rv(
    mu: f64, a: f64, e: f64, inc: f64, raan: f64, aop: f64, ta: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    let p = a * (1.0 - e * e);
    let r_mag = p / (1.0 + e * ta.cos());

    let (si, ci) = inc.sin_cos();
    let (sr, cr) = raan.sin_cos();
    let (sw, cw) = aop.sin_cos();
    let p_hat = Vector3::new(
        cr * cw - sr * sw * ci,
        sr * cw + cr * sw * ci,
        sw * si,
    );
    let q_hat = Vector3::new(
        -cr * sw - sr * cw * ci,
        -sr * sw + cr * cw * ci,
        cw * si,
    );

    let (sf, cf) = ta.sin_cos();
    let r = r_mag * (cf * p_hat + sf * q_hat);
    let v = (mu / p).sqrt() * (-sf * p_hat + (e + cf) * q_hat);
    (r, v)
}

/// Semi-major axis from inertial position and velocity (vis-viva).
fn sma_from_rv(mu: f64, r: Vector3<f64>, v: Vector3<f64>) -> f64 {
    let energy = v.norm_squared() / 2.0 - mu / r.norm();
    -mu / (2.0 * energy)
}

fn f_to_e(f: f64, e: f64) -> f64 {
    f64::atan2((1.0 - e * e).sqrt() * f.sin(), f.cos() + e)
}

fn e_to_f(ee: f64, e: f64) -> f64 {
    f64::atan2((1.0 - e * e).sqrt() * ee.sin(), ee.cos() - e)
}

fn solve_kepler(m: f64, e: f64) -> f64 {
    let two_pi = 2.0 * std::f64::consts::PI;
    let m = ((m % two_pi) + two_pi) % two_pi;
    let mut ee = m;
    for _ in 0..50 {
        let delta = (m - ee + e * ee.sin()) / (1.0 - e * ee.cos());
        ee += delta;
        if delta.abs() < 1e-14 {
            break;
        }
    }
    ee
}

/// Analytical Keplerian position at elapsed time t_sec.
fn kepler_position(
    mu: f64, a: f64, e: f64, inc: f64, raan: f64, aop: f64, ta0: f64, t_sec: f64,
) -> Vector3<f64> {
    let n = (mu / a.powi(3)).sqrt();
    let e0 = f_to_e(ta0, e);
    let m0 = e0 - e * e0.sin();
    let m = m0 + n * t_sec;
    let et = solve_kepler(m, e);
    let ta = e_to_f(et, e);
    elem2rv(mu, a, e, inc, raan, aop, ta).0
}
