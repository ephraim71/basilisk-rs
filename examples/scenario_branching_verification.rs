//! Verification of effector branching against impulse-momentum truth and the
//! integrator's order: port of upstream
//! `examples/scenarioBranchingVerification.py`.
//!
//! A 750 kg hub tumbling in field-free space carries one multibody host
//! with an `extForceTorque` load branched onto the host's tip body. Upstream
//! runs two studies on every host:
//!
//! * **impulse-momentum**: at a fine fixed step, the spacecraft's angular
//!   momentum about its center of mass (`totRotAngMomPntC_N`) and its
//!   center-of-mass ΔV (`TotalAccumDV_CN_N`) are compared with the
//!   cumulative Simpson integrals of the branched torque about the center
//!   of mass and of the force over the total mass; the residuals must stay
//!   at the floating-point floor;
//! * **step refinement**: the final hub states at successively halved
//!   steps converge at the fitted order of RK4, four.
//!
//! The hosts ported are upstream's `spinningBodyNDOF` (three segments of a
//! massless bending joint and a torsional 33.3 kg sub-panel; tip body 6)
//! and `nHingedRigidBody` (three 100 kg hinged panels; tip panel 3).
//! Upstream's `extForceTorque` carries `extForce_B = [10, -5, 3] N` at the
//! tip body's frame origin and `extTorquePntB_B = [2, -1, 4] N m` about it,
//! both in the tip frame, and `extForce_N = [-4, 6, 2] N`. It is attached
//! as upstream does, an [`ExtForceTorque`] the tip body evaluates at its
//! frame: the tip body's joint frame on the spinning-body chain
//! ([`SpinningBodyNDofStateEffector::add_dynamic_effector`]), the tip panel's
//! hinge frame on the N-hinged chain
//! ([`NHingedRigidBodyStateEffector::add_dynamic_effector`]).
//!
//! Not ported: upstream's third host, `linearTranslationNDOF`: basilisk-rs's
//! [`LinearTranslationNDofStateEffector`](basilisk_rs::dynamics::linear_translation_n_dof_state_effector::LinearTranslationNDofStateEffector)
//! takes no attached effectors or loads yet.
//!
//! Upstream `run(show_plots, dtSweep, convergenceFinalTime, conservationDt,
//! conservationFinalTime)` maps to `--dt-sweep a,b,c`, `--convergence-time`,
//! `--conservation-dt` and `--conservation-time`; the defaults are
//! upstream's (`--test` selects the shortened parameters upstream's
//! `test_scenarioBranchingVerification.py` runs). The residual histories go
//! to `examples/output/scenarios/scenarioBranchingVerificationConservation_<host>.csv`
//! and the refinement residuals to
//! `examples/output/scenarios/scenarioBranchingVerificationConvergence.csv`.
//!
//! `tests` ports `test_scenarioBranchingVerification.py` for the ported
//! hosts: every observable converges at an order within `(3.5, 4.5)`, the
//! angular residual stays below `1e-8 N m s` and the linear one below
//! `1e-12 m/s`.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/multibody.rs"]
mod multibody;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/table_csv.rs"]
mod table_csv;

use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::n_hinged_rigid_body_state_effector::{
    HingedPanel, NHingedRigidBodyConfig, NHingedRigidBodyStateEffector,
};
use basilisk_rs::dynamics::spinning_body_n_dof_state_effector::{
    SpinningBody, SpinningBodyNDofStateEffector,
};
use basilisk_rs::messages::{
    Output, SpacecraftDiagnosticsMsg, SpacecraftMassPropsMsg, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use hifitime::Epoch;
use message_log::MessageLog;
use multibody::{box_inertia, set_center_of_mass_initial_state};
use nalgebra::{Matrix3, Vector3};
use orbit_tools::sec2nano;
use table_csv::write_table;

/// Upstream `CONVERGENCE_DT_SWEEP` \[s\] and `CONVERGENCE_FINAL_TIME`.
const CONVERGENCE_DT_SWEEP: [f64; 5] = [0.05, 0.025, 0.0125, 0.00625, 0.003125];
const CONVERGENCE_FINAL_TIME: f64 = 10.0;
/// Upstream `CONSERVATION_DT` and `CONSERVATION_FINAL_TIME` \[s\].
const CONSERVATION_DT: f64 = 1.0e-4;
const CONSERVATION_FINAL_TIME: f64 = 4.0;
/// Upstream `setupExtFT`: the branched load.
const EXT_FORCE_B: [f64; 3] = [10.0, -5.0, 3.0];
const EXT_FORCE_N: [f64; 3] = [-4.0, 6.0, 2.0];
const EXT_TORQUE_PNT_B_B: [f64; 3] = [2.0, -1.0, 4.0];
/// Upstream `sc.hub.mHub`.
const HUB_MASS_KG: f64 = 750.0;
/// Upstream `OBSERVABLES`.
const OBSERVABLES: [&str; 4] = ["sigma_BN", "omega_BN_B", "r_BN_N", "v_BN_N"];

/// Upstream `HOST_SETUPS` (the ported ones).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Host {
    SpinningBodyNdof,
    NHingedRigidBody,
}

const HOSTS: [Host; 2] = [Host::SpinningBodyNdof, Host::NHingedRigidBody];

impl Host {
    fn label(self) -> &'static str {
        match self {
            Self::SpinningBodyNdof => "spinningBodyNDOF",
            Self::NHingedRigidBody => "nHingedRigidBody",
        }
    }
}

/// Upstream `HostProps`, with the host effector and its tip body's state
/// output (upstream `<logAttr>[segment - 1]`).
struct HostSetup {
    effector: HostEffector,
    total_mass_kg: f64,
    /// Upstream `r_PcP_P`: the tip body's center of mass relative to its
    /// frame origin, in its frame.
    r_pcp_p_m: Vector3<f64>,
    tip_state_out: Output<SpacecraftStateMsg>,
}

enum HostEffector {
    Spinning(SpinningBodyNDofStateEffector),
    Hinged(NHingedRigidBodyStateEffector),
}

/// Upstream `setupExtFT`.
fn setup_ext_ft() -> ExtForceTorque {
    ExtForceTorque::new(ExtForceTorqueConfig {
        name: "extFT".to_string(),
        ext_force_inertial_n: Vector3::from(EXT_FORCE_N),
        ext_force_body_n: Vector3::from(EXT_FORCE_B),
        ext_torque_body_nm: Vector3::from(EXT_TORQUE_PNT_B_B),
    })
}

/// Upstream `setupSpinningBodyNDOF`: three two-DOF segments, tip body 6.
fn setup_spinning_body_ndof() -> HostSetup {
    let segments = 3;
    let mass = 100.0 / segments as f64;
    let length = 18.0 / segments as f64;
    let width = 3.0;
    let thickness = 0.3;
    let mut host = SpinningBodyNDofStateEffector::new("spinningBodyNDOF");
    for index in 0..segments {
        host.add_spinning_body(SpinningBody {
            mass_kg: 0.0,
            inertia_about_com_spinner_kg_m2: Matrix3::zeros(),
            parent_to_spinner0_dcm: Matrix3::identity(),
            center_of_mass_spinner_m: Vector3::new(0.0, length / 2.0, 0.0),
            origin_parent_m: if index == 0 {
                Vector3::new(0.0, 1.5, 1.5 - thickness / 2.0)
            } else {
                Vector3::new(0.0, length, 0.0)
            },
            spin_axis_spinner: Vector3::x(),
            theta_init_rad: 2.0_f64.to_radians(),
            theta_dot_init_radps: -0.5_f64.to_radians(),
            spring_constant_nm_per_rad: 10.0,
            damping_nm_s_per_rad: 8.0,
        });
        host.add_spinning_body(SpinningBody {
            mass_kg: mass,
            // Upstream: `lengthSubPanel` along y, `widthSubPanel` along x.
            inertia_about_com_spinner_kg_m2: box_inertia(mass, width, length, thickness),
            parent_to_spinner0_dcm: Matrix3::identity(),
            center_of_mass_spinner_m: Vector3::new(0.0, length / 2.0, 0.0),
            origin_parent_m: Vector3::zeros(),
            spin_axis_spinner: Vector3::y(),
            theta_init_rad: 2.0_f64.to_radians(),
            theta_dot_init_radps: -0.5_f64.to_radians(),
            spring_constant_nm_per_rad: 1.0,
            damping_nm_s_per_rad: 0.8,
        });
    }
    let tip = 2 * segments - 1;
    host.add_dynamic_effector(tip, setup_ext_ft());
    HostSetup {
        tip_state_out: host.spinning_body_state_out[tip].clone(),
        effector: HostEffector::Spinning(host),
        total_mass_kg: mass * segments as f64,
        r_pcp_p_m: Vector3::new(0.0, length / 2.0, 0.0),
    }
}

/// Upstream `setupNHingedRigidBody`: three identical panels, tip panel 3.
fn setup_n_hinged_rigid_body() -> HostSetup {
    let panel_mass = 100.0;
    let half_length = 0.75;
    let theta_init_deg: [f64; 3] = [5.0, -2.0, 3.0];
    let theta_dot_init_degps: [f64; 3] = [-1.0, 0.0, 0.5];
    let mut config = NHingedRigidBodyConfig::new("nHingedRigidBody");
    config.hinge_position_body_m = Vector3::new(0.5, -1.5, -0.5);
    let mut host = NHingedRigidBodyStateEffector::new(config);
    for (theta, theta_dot) in theta_init_deg.into_iter().zip(theta_dot_init_degps) {
        host.add_hinged_panel(HingedPanel {
            mass_kg: panel_mass,
            center_of_mass_offset_m: half_length,
            spring_constant_nm_per_rad: 100.0,
            damping_nm_s_per_rad: 0.0,
            inertia_about_panel_com_panel_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                100.0, 50.0, 50.0,
            )),
            theta_init_rad: theta.to_radians(),
            theta_dot_init_radps: theta_dot.to_radians(),
            theta_rest_rad: 0.0,
        });
    }
    let tip = 2;
    host.add_dynamic_effector(tip, setup_ext_ft());
    HostSetup {
        tip_state_out: host.panel_state_out[tip].clone(),
        effector: HostEffector::Hinged(host),
        total_mass_kg: panel_mass * 3.0,
        // The panel's center of mass, `d` along -ŝ1 from its hinge.
        r_pcp_p_m: Vector3::new(-half_length, 0.0, 0.0),
    }
}

fn setup(host: Host) -> HostSetup {
    match host {
        Host::SpinningBodyNdof => setup_spinning_body_ndof(),
        Host::NHingedRigidBody => setup_n_hinged_rigid_body(),
    }
}

/// One run's records, every step (upstream `datLog`, `ipLog`, `scLog`).
struct Case {
    states: MessageLog<SpacecraftStateMsg>,
    mass_props: MessageLog<SpacecraftMassPropsMsg>,
    diagnostics: MessageLog<SpacecraftDiagnosticsMsg>,
    tip: MessageLog<SpacecraftStateMsg>,
    total_mass_kg: f64,
    r_pcp_p_m: Vector3<f64>,
    dt_s: f64,
}

/// Upstream `runToFinalState` / `runWithFullLogs`: the hub with `host`
/// branched, from upstream's initial state, for `final_time_s` at `dt_s`.
fn run_case(host: Host, dt_s: f64, final_time_s: f64) -> Case {
    let step_nanos = sec2nano(dt_s);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
        integration_step_nanos: step_nanos,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::new(0.10, 0.05, -0.08),
        initial_omega_radps: Vector3::new(0.05, 0.02, -0.03),
        integrator: None,
    });
    let HostSetup {
        effector,
        total_mass_kg,
        r_pcp_p_m,
        tip_state_out,
    } = setup(host);
    match effector {
        HostEffector::Spinning(effector) => spacecraft.add_state_effector(effector),
        HostEffector::Hinged(effector) => spacecraft.add_state_effector(effector),
    }
    // Upstream `r_CN_NInit = v_CN_NInit = 0`.
    set_center_of_mass_initial_state(&mut spacecraft, Vector3::zeros(), Vector3::zeros());

    let mut states = MessageLog::<SpacecraftStateMsg>::default();
    let mut mass_props = MessageLog::<SpacecraftMassPropsMsg>::default();
    let mut diagnostics = MessageLog::<SpacecraftDiagnosticsMsg>::default();
    let mut tip = MessageLog::<SpacecraftStateMsg>::default();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        connect!(&simulation,
            &spacecraft.state_out => &mut states.input,
            &spacecraft.mass_props_out => &mut mass_props.input,
            &spacecraft.diagnostics_out => &mut diagnostics.input,
            &tip_state_out => &mut tip.input,
        );
        schedule! { simulation,
            "scObject" => &mut spacecraft, period=step_nanos, priority=100;
            "datLog" => &mut states, period=step_nanos, priority=0;
            "massLog" => &mut mass_props, period=step_nanos, priority=0;
            "scLog" => &mut diagnostics, period=step_nanos, priority=0;
            "ipLog" => &mut tip, period=step_nanos, priority=0;
        }
        simulation.run_for(sec2nano(final_time_s));
    }
    Case {
        states,
        mass_props,
        diagnostics,
        tip,
        total_mass_kg,
        r_pcp_p_m,
        dt_s,
    }
}

/// `scipy.integrate.cumulative_simpson(y, dx=dx, initial=0)` for vector
/// samples: each interval integrated over the quadratic through it and its
/// neighbour (the next one for even intervals, the previous one for odd
/// intervals and the last), then summed.
fn cumulative_simpson(samples: &[Vector3<f64>], dx: f64) -> Vec<Vector3<f64>> {
    let n = samples.len();
    let mut result = vec![Vector3::zeros(); n];
    if n < 3 {
        for index in 1..n {
            result[index] = result[index - 1] + 0.5 * dx * (samples[index - 1] + samples[index]);
        }
        return result;
    }
    for interval in 0..n - 1 {
        let y = |offset: usize| samples[offset];
        let integral = if interval % 2 == 0 && interval != n - 2 {
            dx / 12.0 * (5.0 * y(interval) + 8.0 * y(interval + 1) - y(interval + 2))
        } else {
            dx / 12.0 * (-y(interval - 1) + 8.0 * y(interval) + 5.0 * y(interval + 1))
        };
        result[interval + 1] = result[interval] + integral;
    }
    result
}

/// Upstream `computeResiduals`: `H_C(t) - H_C(0) - ∫τ_C` and
/// `ΔV_C(t) - ΔV_C(0) - ∫F/M`, with the force `[NP] F_P + F_N` at the tip
/// frame origin, `r_PcN - [NP] r_PcP_P`.
#[allow(clippy::type_complexity)]
fn compute_residuals(case: &Case) -> (Vec<f64>, Vec<Vector3<f64>>, Vec<Vector3<f64>>) {
    let n = case.tip.values.len();
    assert_eq!(
        case.states.values.len(),
        n,
        "sc log and ip log lengths disagree"
    );
    let force_b = Vector3::from(EXT_FORCE_B);
    let force_inertial_n = Vector3::from(EXT_FORCE_N);
    let torque_b = Vector3::from(EXT_TORQUE_PNT_B_B);
    let total_mass = HUB_MASS_KG + case.total_mass_kg;
    let mut forces_n = Vec::with_capacity(n);
    let mut torques_c = Vec::with_capacity(n);
    for ((tip, state), mass_props) in case
        .tip
        .values
        .iter()
        .zip(&case.states.values)
        .zip(&case.mass_props.values)
    {
        let dcm_np = mrp_to_dcm(tip.sigma_bn).transpose();
        let force_n = dcm_np * force_b + force_inertial_n;
        let r_cn_n = state.position_m
            + mrp_to_dcm(state.sigma_bn).transpose() * mass_props.center_of_mass_body_m;
        let r_p_n = tip.position_m - dcm_np * case.r_pcp_p_m;
        forces_n.push(force_n);
        torques_c.push(dcm_np * torque_b + (r_p_n - r_cn_n).cross(&force_n));
    }
    let impulse = cumulative_simpson(&forces_n, case.dt_s);
    let angular_impulse = cumulative_simpson(&torques_c, case.dt_s);
    let h0 = case.diagnostics.values[0].rotational_angular_momentum_inertial_kg_m2ps;
    let dv0 = case.states.values[0].accumulated_dv_cn_n_mps;
    let res_h = case
        .diagnostics
        .values
        .iter()
        .zip(&angular_impulse)
        .map(|(diagnostics, integral)| {
            diagnostics.rotational_angular_momentum_inertial_kg_m2ps - h0 - integral
        })
        .collect();
    let res_v = case
        .states
        .values
        .iter()
        .zip(&impulse)
        .map(|(state, integral)| state.accumulated_dv_cn_n_mps - dv0 - integral / total_mass)
        .collect();
    (case.tip.times_s(), res_h, res_v)
}

/// Upstream's peak residual norms of one host.
#[derive(Clone, Copy, Debug)]
struct MaxResiduals {
    angular: f64,
    linear: f64,
}

/// Upstream `runConservationStudy`.
fn run_conservation_study(
    dt_s: f64,
    final_time_s: f64,
    write_csv: bool,
) -> Vec<(Host, MaxResiduals)> {
    HOSTS
        .iter()
        .map(|&host| {
            let case = run_case(host, dt_s, final_time_s);
            let (times, res_h, res_v) = compute_residuals(&case);
            let max = MaxResiduals {
                angular: res_h.iter().fold(0.0_f64, |max, residual| max.max(residual.norm())),
                linear: res_v.iter().fold(0.0_f64, |max, residual| max.max(residual.norm())),
            };
            println!(
                "[{}]  dt={dt_s}  T={final_time_s}\n    max ||H_C residual||  = {:.3e}  N m s\n    max ||Δv_C residual|| = {:.3e}  m/s",
                host.label(),
                max.angular,
                max.linear
            );
            if write_csv {
                let headers: Vec<String> = ["time_s".to_string()]
                    .into_iter()
                    .chain((0..3).map(|index| format!("resH_N_m_s_{index}")))
                    .chain((0..3).map(|index| format!("resV_m_per_s_{index}")))
                    .collect();
                let rows: Vec<Vec<f64>> = times
                    .iter()
                    .zip(res_h.iter().zip(&res_v))
                    .map(|(time, (h, v))| vec![*time, h.x, h.y, h.z, v.x, v.y, v.z])
                    .collect();
                let path = write_table(&format!("scenarioBranchingVerificationConservation_{}", host.label()), &headers, &rows);
                println!("wrote {}", path.display());
            }
            (host, max)
        })
        .collect()
}

/// The final value of `observable` in `case`.
fn final_observable(case: &Case, observable: &str) -> Vector3<f64> {
    let state = case.states.last();
    match observable {
        "sigma_BN" => state.sigma_bn,
        "omega_BN_B" => state.omega_radps,
        "r_BN_N" => state.position_m,
        "v_BN_N" => state.velocity_mps,
        other => panic!("unknown observable {other}"),
    }
}

/// Least-squares slope of `log(residual)` against `log(dt)`, upstream's
/// `np.polyfit(np.log(pairDts), np.log(res), 1)[0]`.
fn fitted_order(dts: &[f64], residuals: &[f64]) -> f64 {
    let xs: Vec<f64> = dts.iter().map(|dt| dt.ln()).collect();
    let ys: Vec<f64> = residuals.iter().map(|residual| residual.ln()).collect();
    let n = xs.len() as f64;
    let mean_x = xs.iter().sum::<f64>() / n;
    let mean_y = ys.iter().sum::<f64>() / n;
    let covariance: f64 = xs
        .iter()
        .zip(&ys)
        .map(|(x, y)| (x - mean_x) * (y - mean_y))
        .sum();
    let variance: f64 = xs.iter().map(|x| (x - mean_x).powi(2)).sum();
    covariance / variance
}

/// The fitted order of every host and observable, upstream's `slopes`.
type Slopes = Vec<(Host, Vec<(&'static str, f64)>)>;

/// Upstream `runConvergenceStudy`.
fn run_convergence_study(dt_sweep: &[f64], final_time_s: f64, write_csv: bool) -> Slopes {
    let pair_dts = &dt_sweep[..dt_sweep.len() - 1];
    let mut columns: Vec<(String, Vec<f64>)> = Vec::new();
    let slopes = HOSTS
        .iter()
        .map(|&host| {
            println!("\n[{}]", host.label());
            let runs: Vec<Case> = dt_sweep
                .iter()
                .map(|&dt| {
                    let case = run_case(host, dt, final_time_s);
                    println!(
                        "  dt={dt}  |omega(T)|={:.6e}  |sigma(T)|={:.6e}",
                        case.states.last().omega_radps.norm(),
                        case.states.last().sigma_bn.norm()
                    );
                    case
                })
                .collect();
            let host_slopes = OBSERVABLES
                .iter()
                .map(|&observable| {
                    let residuals: Vec<f64> = runs
                        .windows(2)
                        .map(|pair| {
                            (final_observable(&pair[0], observable)
                                - final_observable(&pair[1], observable))
                            .norm()
                        })
                        .collect();
                    let slope = fitted_order(pair_dts, &residuals);
                    println!(
                        "{:>26}  {:>12}  {:?}    {slope:+.3}",
                        host.label(),
                        observable,
                        residuals
                    );
                    columns.push((format!("{}_{observable}", host.label()), residuals));
                    (observable, slope)
                })
                .collect();
            (host, host_slopes)
        })
        .collect();
    if write_csv {
        let headers: Vec<String> = ["dt_s".to_string()]
            .into_iter()
            .chain(columns.iter().map(|(name, _)| name.clone()))
            .collect();
        let rows: Vec<Vec<f64>> = pair_dts
            .iter()
            .enumerate()
            .map(|(index, dt)| {
                std::iter::once(*dt)
                    .chain(columns.iter().map(|(_, residuals)| residuals[index]))
                    .collect()
            })
            .collect();
        let path = write_table("scenarioBranchingVerificationConvergence", &headers, &rows);
        println!("wrote {}", path.display());
    }
    slopes
}

/// Upstream `run(...)` parameters.
#[derive(Clone, Debug)]
struct Config {
    dt_sweep: Vec<f64>,
    convergence_final_time_s: f64,
    conservation_dt_s: f64,
    conservation_final_time_s: f64,
    /// Write the CSV files (off in the tests).
    write_csv: bool,
}

impl Config {
    /// Upstream's `run(show_plots)` defaults.
    fn upstream_default() -> Self {
        Self {
            dt_sweep: CONVERGENCE_DT_SWEEP.to_vec(),
            convergence_final_time_s: CONVERGENCE_FINAL_TIME,
            conservation_dt_s: CONSERVATION_DT,
            conservation_final_time_s: CONSERVATION_FINAL_TIME,
            write_csv: true,
        }
    }

    /// Upstream `test_scenarioBranchingVerification.py`'s shortened run.
    fn upstream_test() -> Self {
        Self {
            dt_sweep: vec![0.05, 0.025, 0.0125, 0.00625],
            convergence_final_time_s: 4.0,
            conservation_dt_s: 1.0e-3,
            conservation_final_time_s: 1.0,
            write_csv: true,
        }
    }
}

/// Upstream's returned metrics (read by the tests).
#[cfg_attr(not(test), allow(dead_code))]
struct Metrics {
    slopes: Slopes,
    max_residuals: Vec<(Host, MaxResiduals)>,
}

/// Upstream `run`: the residual study, then the refinement study.
fn run(config: &Config) -> Metrics {
    let max_residuals = run_conservation_study(
        config.conservation_dt_s,
        config.conservation_final_time_s,
        config.write_csv,
    );
    let slopes = run_convergence_study(
        &config.dt_sweep,
        config.convergence_final_time_s,
        config.write_csv,
    );
    Metrics {
        slopes,
        max_residuals,
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_branching_verification [--test] [--dt-sweep a,b,...] [--convergence-time T] \
         [--conservation-dt DT] [--conservation-time T]",
    );
    let mut config = if cli.switch("test") {
        Config::upstream_test()
    } else {
        Config::upstream_default()
    };
    let sweep: String = cli.value("dt-sweep", String::new());
    if !sweep.is_empty() {
        config.dt_sweep = sweep
            .split(',')
            .map(|value| {
                value
                    .trim()
                    .parse()
                    .expect("--dt-sweep takes comma-separated seconds")
            })
            .collect();
    }
    config.convergence_final_time_s =
        cli.value("convergence-time", config.convergence_final_time_s);
    config.conservation_dt_s = cli.value("conservation-dt", config.conservation_dt_s);
    config.conservation_final_time_s =
        cli.value("conservation-time", config.conservation_final_time_s);
    cli.finish();
    run(&config);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `cumulative_simpson` is exact for quadratics, as scipy's is.
    #[test]
    fn cumulative_simpson_integrates_quadratics_exactly() {
        for n in [3, 4, 7, 8] {
            let dx = 0.1;
            let samples: Vec<Vector3<f64>> = (0..n)
                .map(|index| {
                    let t = index as f64 * dx;
                    Vector3::new(1.0, 2.0 * t, 3.0 * t * t)
                })
                .collect();
            let integral = cumulative_simpson(&samples, dx);
            for (index, value) in integral.iter().enumerate() {
                let t = index as f64 * dx;
                let truth = Vector3::new(t, t * t, t * t * t);
                assert!(
                    (value - truth).norm() < 1e-14,
                    "n={n}, {index}: {value} vs {truth}"
                );
            }
        }
    }

    /// Upstream `test_scenarioBranchingVerification.py`, for the ported
    /// hosts: RK4's order of 4 for every observable, and impulse-momentum
    /// residuals at the floating-point floor.
    #[test]
    fn branching_holds_order_and_impulse_momentum() {
        let metrics = run(&Config {
            write_csv: false,
            ..Config::upstream_test()
        });
        let hosts: Vec<Host> = metrics.slopes.iter().map(|(host, _)| *host).collect();
        assert_eq!(hosts, HOSTS.to_vec());
        for (host, observables) in &metrics.slopes {
            for (observable, order) in observables {
                assert!(
                    3.5 < *order && *order < 4.5,
                    "{} {observable} converged at order {order:.2}, expected RK4's order of 4",
                    host.label()
                );
            }
        }
        for (host, residuals) in &metrics.max_residuals {
            assert!(
                residuals.angular < 1.0e-8,
                "{} angular impulse residual {:.3e} N m s is too large",
                host.label(),
                residuals.angular
            );
            assert!(
                residuals.linear < 1.0e-12,
                "{} linear impulse residual {:.3e} m/s is too large",
                host.label(),
                residuals.linear
            );
        }
    }
}
