//! Port of upstream `examples/scenarioConstrainedDynamics.py`: two identical
//! spacecraft hubs held together by a
//! [`ConstraintDynamicEffector`](basilisk_rs::dynamics::constraint_dynamic_effector::ConstraintDynamicEffector),
//! tumbling slowly in low Earth orbit or in deep space.
//!
//! | `--env` | CSV |
//! |---|---|
//! | `Gravity` (default, upstream's test) | `scenarioConstrainedDynamicsGravity.csv` |
//! | `NoGravity` | `scenarioConstrainedDynamicsNoGravity.csv` |
//!
//! `--all` runs both. Each 750 kg hub (`r_BcB_B = [0, 0, 1]` m,
//! `IHubPntBc_B = 600 I` kg m²) connects 0.1 m from its body origin through
//! a 0.1 m arm along the line to spacecraft 2's initial position; the pair
//! spins at `[0.01, 0.01, 0.01]` rad/s about its joint center of mass. The
//! constraint is tuned with `alpha = 1e2`, `beta = 1e3`; both spacecraft are
//! integrated together with RKF45 at 0.1 s for one minute. Upstream's
//! `r_CN_NInit` are the centers of mass; [`hub_init`] converts them to the
//! hub states as upstream's `Spacecraft::initializeDynamics` does.
//!
//! The CSV holds both hub states at every step (`r_B1N_N_m_i`,
//! `sigma_B1N_i`, `r_B2N_N_m_i`, ...); `plot_scenario_constrained_dynamics.py`
//! derives upstream's direction and attitude violations from them.
//!
//! Upstream's `test_scenarioConstrainedDynamics.py` only checks that the
//! `Gravity` case runs. `tests` also checks what the figures show: after the
//! initial transient both violations stay small, the direction violation
//! below a millimetre and the attitude violation below a hundredth of a
//! degree, and the pair keeps tumbling together.

#[path = "support/common.rs"]
mod common;
#[path = "support/constrained.rs"]
mod constrained;
#[path = "support/hub_init.rs"]
mod hub_init;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/scenario_args.rs"]
mod scenario_args;

use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::SpacecraftGroup;
use basilisk_rs::{connect, schedule};
use constrained::{
    ConstraintGeometry, Violations, earth_point_mass, hub, hub_columns, leo_state, upstream_rkf45,
};
use hifitime::Epoch;
use hub_init::set_center_of_mass_initial_state;
use nalgebra::{Matrix3, Vector3};
use recording::{History, scenario_csv};
use scenario_args::ScenarioArgs;

const USAGE: &str = "\
scenario_constrained_dynamics [--env Gravity|NoGravity]
scenario_constrained_dynamics --all     run both environments";

/// Upstream `simulationTimeStep = macros.sec2nano(0.1)`.
const STEP_NANOS: u64 = 100_000_000;
/// Upstream `stopTime = macros.min2nano(1)`.
const DURATION_NANOS: u64 = 60_000_000_000;
/// Upstream `scObject.hub.mHub` \[kg\].
const HUB_MASS_KG: f64 = 750.0;
/// Upstream `COMoffset` \[m\]: body origin to connection point.
const COM_OFFSET_M: f64 = 0.1;
/// Upstream `l` \[m\]: arm length.
const ARM_LENGTH_M: f64 = 0.1;

/// Upstream `env`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Environment {
    Gravity,
    NoGravity,
}

impl Environment {
    pub fn label(self) -> &'static str {
        match self {
            Self::Gravity => "Gravity",
            Self::NoGravity => "NoGravity",
        }
    }
}

pub const ALL: [Environment; 2] = [Environment::Gravity, Environment::NoGravity];

pub struct Results {
    pub csv_path: std::path::PathBuf,
    pub states_1: Vec<(u64, SpacecraftStateMsg)>,
    pub states_2: Vec<(u64, SpacecraftStateMsg)>,
    pub violations: Violations,
}

pub fn run(environment: Environment) -> Results {
    let hub_center_of_mass_m = Vector3::new(0.0, 0.0, 1.0);
    let inertia = Matrix3::from_diagonal_element(600.0);
    let mut spacecraft_1 = hub(HUB_MASS_KG, hub_center_of_mass_m, inertia, STEP_NANOS);
    let mut spacecraft_2 = hub(HUB_MASS_KG, hub_center_of_mass_m, inertia, STEP_NANOS);

    let (r_b2n_n_0, rdot_b2n_n) = match environment {
        Environment::Gravity => {
            spacecraft_1
                .add_grav_body(earth_point_mass())
                .expect("one central body");
            spacecraft_2
                .add_grav_body(earth_point_mass())
                .expect("one central body");
            leo_state()
        }
        Environment::NoGravity => (Vector3::repeat(1.0), Vector3::repeat(1.0)),
    };

    // With the B1, B2 and N frames initially aligned.
    let direction = r_b2n_n_0 / r_b2n_n_0.norm();
    let geometry = ConstraintGeometry {
        r_p1b1_b1_m: direction * COM_OFFSET_M,
        r_p2b2_b2_m: -direction * COM_OFFSET_M,
        r_p2p1_b1_init_m: direction * ARM_LENGTH_M,
    };
    let r_b1n_n_0 =
        r_b2n_n_0 + geometry.r_p2b2_b2_m - geometry.r_p2p1_b1_init_m - geometry.r_p1b1_b1_m;
    let rdot_b1n_n = rdot_b2n_n;

    // The pair spins about its joint center of mass C.
    let r_cn_n = (r_b1n_n_0 + r_b2n_n_0) / 2.0;
    let omega_cn_n = Vector3::repeat(0.01);
    let rdot_b1n_n_0 = rdot_b1n_n + omega_cn_n.cross(&(r_b1n_n_0 - r_cn_n));
    let rdot_b2n_n_0 = rdot_b2n_n + omega_cn_n.cross(&(r_b2n_n_0 - r_cn_n));
    for (spacecraft, position_m, velocity_mps) in [
        (&mut spacecraft_1, r_b1n_n_0, rdot_b1n_n_0),
        (&mut spacecraft_2, r_b2n_n_0, rdot_b2n_n_0),
    ] {
        spacecraft.config.initial_omega_radps = omega_cn_n;
        set_center_of_mass_initial_state(spacecraft, position_m, velocity_mps);
    }

    let mut group = SpacecraftGroup::new(vec![spacecraft_1, spacecraft_2]);
    group.integrator = Some(Box::new(upstream_rkf45()));
    group.add_coupling_effector(geometry.constraint(1.0e2, 1.0e3));

    let file_stem = format!("scenarioConstrainedDynamics{}", environment.label());
    let mut csv = scenario_csv(&file_stem);
    let csv_path = csv.config.output_path.clone();
    let mut history_1 = History::<SpacecraftStateMsg>::new();
    let mut history_2 = History::<SpacecraftStateMsg>::new();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2024, 5, 14), false);
        let [state_1, state_2] = [0, 1].map(|index| group.spacecraft()[index].state_out.clone());
        connect!(&simulation,
            &state_1 => csv.add_source::<SpacecraftStateMsg>(hub_columns("B1")),
            &state_2 => csv.add_source::<SpacecraftStateMsg>(hub_columns("B2")),
            &state_1 => &mut history_1.input,
            &state_2 => &mut history_2.input,
        );
        schedule! { simulation,
            "spacecraftGroup" => &mut group, period=STEP_NANOS, priority=10;
            "csv" => &mut csv, period=STEP_NANOS, priority=0;
            "datLog1" => &mut history_1, period=STEP_NANOS, priority=0;
            "datLog2" => &mut history_2, period=STEP_NANOS, priority=0;
        }
        simulation.run_for(DURATION_NANOS);
    }
    csv.flush().expect("flush scenario CSV");

    let states_1 = history_1.into_samples();
    let states_2 = history_2.into_samples();
    let violations = Violations::new(&geometry, &states_1, &states_2);
    Results {
        csv_path,
        states_1,
        states_2,
        violations,
    }
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let environments = if args.flag("--all") {
        ALL.to_vec()
    } else {
        match args.value("--env", "Gravity".to_string()).as_str() {
            "Gravity" => vec![Environment::Gravity],
            "NoGravity" => vec![Environment::NoGravity],
            other => args.fail(&format!("unknown env {other:?}")),
        }
    };
    for environment in environments {
        let results = run(environment);
        println!(
            "wrote {} (max direction violation {:.3e} m, max attitude violation {:.3e} deg)",
            results.csv_path.display(),
            results.violations.max_direction_m(),
            results.violations.max_attitude_deg(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::{ALL, run};

    /// `test_scenarioConstrainedDynamics.py` runs the `Gravity` case and
    /// checks only that it completes. Both cases here: the run covers the
    /// minute at 0.1 s, the constraint holds the hubs within a millimetre
    /// and a hundredth of a degree throughout, and the pair still rotates
    /// at its initial rate (no hub was torn away).
    #[test]
    fn constraint_holds_the_tumbling_pair_together() {
        for environment in ALL {
            let results = run(environment);
            assert_eq!(results.states_1.len(), 601, "{environment:?}");
            let direction_m = results.violations.max_direction_m();
            let attitude_deg = results.violations.max_attitude_deg();
            assert!(
                direction_m < 1.0e-3,
                "{environment:?}: |psi| {direction_m:e} m"
            );
            assert!(
                attitude_deg < 1.0e-2,
                "{environment:?}: |phi| {attitude_deg:e} deg"
            );
            let (_, last_1) = results.states_1.last().expect("samples");
            let (_, last_2) = results.states_2.last().expect("samples");
            for omega in [last_1.omega_radps, last_2.omega_radps] {
                assert!(
                    (omega.norm() - 0.01 * 3.0_f64.sqrt()).abs() < 1.0e-3,
                    "{environment:?}: final rate {omega:?}"
                );
            }
        }
    }
}
