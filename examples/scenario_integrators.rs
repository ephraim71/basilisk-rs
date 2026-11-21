//! Port of upstream `examples/scenarioIntegrators.py`: a LEO spacecraft
//! about a point-mass Earth, integrated with a coarse 120 s step by one of
//! seven integrators, to compare how far each strays from the true orbit.
//!
//! | `--integrator` | upstream class | here |
//! |---|---|---|
//! | `rk4` (default) | `svIntegratorRK4` | basilisk-rs's default [`basilisk_rs::integrators::Rk4`] |
//! | `rkf45` | `svIntegratorRKF45` | [`Rkf45::upstream`] |
//! | `rkf78` | `svIntegratorRKF78` | [`Rkf78::upstream`] |
//! | `euler` | `svIntegratorEuler` | [`basilisk_rs::integrators::Euler`] |
//! | `rk2` | `svIntegratorRK2` | [`basilisk_rs::integrators::Heun`] |
//! | `rk3` | `svIntegratorRungeKutta` with Kutta's tableau | [`RungeKutta`] of [`RK3`] |
//! | `bogackiShampine` | `svIntegratorAdaptiveRungeKutta` | [`AdaptiveRungeKutta`] of [`BOGACKI_SHAMPINE`] |
//!
//! The adaptive methods use upstream's step-size control
//! ([`basilisk_rs::integrators::StepControl::Upstream`]) with its default
//! tolerances, which upstream's truth depends on. The user-defined
//! `svIntegratorRungeKutta` and `svIntegratorAdaptiveRungeKutta` upstream
//! builds from Python coefficient lists are, here, basilisk-rs's
//! [`RungeKutta`] and [`AdaptiveRungeKutta`] of a
//! [`ButcherTableau`].
//!
//! The run lasts 0.75 orbits and records the state at every 120 s step
//! (upstream's `samplingTime` rounds down to 1 ns, below the task period) in
//! `scenarioIntegrators<integrator>.csv`. `--all` runs the six integrators of
//! `test_scenarioIntegrators.py`, whose check, ported in `tests`, compares
//! five of the recorded positions with upstream's truth to 1 m per
//! component.

#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/scenario_args.rs"]
mod scenario_args;

use basilisk_rs::integrators::{
    AdaptiveRungeKutta, ButcherTableau, Euler, Heun, Integrator, Rkf45, Rkf78, RungeKutta,
};
use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use basilisk_rs::{connect, schedule};
use grav_factory::Body;
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{orbit_period_s, sec2nano};
use recording::{History, sampling_time, scenario_csv, state_columns};
use scenario_args::ScenarioArgs;

const USAGE: &str = "\
scenario_integrators [--integrator rk4|rkf45|rkf78|euler|rk2|rk3|bogackiShampine]
scenario_integrators --all     run every upstream test parametrization";

/// `scenarioIntegrators`' `rk3`: Kutta's third-order method through
/// `svIntegratorRungeKutta`.
pub const RK3: ButcherTableau = ButcherTableau {
    a: &[&[], &[0.5], &[-1.0, 2.0]],
    c: &[0.0, 0.5, 1.0],
    b: &[1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0],
    b_embedded: None,
    error_estimate_order: 3,
};

/// `scenarioIntegrators`' `bogackiShampine` through
/// `svIntegratorAdaptiveRungeKutta(scObject, 3, ...)`: `b` is upstream's
/// `b_coefficients`, `b_embedded` its `b_star_coefficients` (the propagated
/// ones), and `methodLargestOrder = error_estimate_order + 1 = 3`.
pub const BOGACKI_SHAMPINE: ButcherTableau = ButcherTableau {
    a: &[
        &[],
        &[0.5],
        &[0.0, 0.75],
        &[2.0 / 9.0, 1.0 / 3.0, 4.0 / 9.0],
    ],
    c: &[0.0, 0.5, 0.75, 1.0],
    b: &[7.0 / 24.0, 0.25, 1.0 / 3.0, 0.125],
    b_embedded: Some(&[2.0 / 9.0, 1.0 / 3.0, 4.0 / 9.0, 0.0]),
    error_estimate_order: 2,
};

/// Upstream `simulationTimeStep`.
const STEP_NANOS: u64 = 120_000_000_000;

/// Upstream `integratorCase`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntegratorCase {
    Rk4,
    Rkf45,
    Rkf78,
    Euler,
    Rk2,
    Rk3,
    BogackiShampine,
}

impl IntegratorCase {
    pub const ALL: [Self; 7] = [
        Self::Rk4,
        Self::Rkf45,
        Self::Rkf78,
        Self::Euler,
        Self::Rk2,
        Self::Rk3,
        Self::BogackiShampine,
    ];

    /// Upstream's case string, also the CSV suffix.
    pub fn label(self) -> &'static str {
        match self {
            Self::Rk4 => "rk4",
            Self::Rkf45 => "rkf45",
            Self::Rkf78 => "rkf78",
            Self::Euler => "euler",
            Self::Rk2 => "rk2",
            Self::Rk3 => "rk3",
            Self::BogackiShampine => "bogackiShampine",
        }
    }

    fn integrator(self) -> Option<Box<dyn Integrator<Spacecraft>>> {
        match self {
            Self::Rk4 => None,
            Self::Rkf45 => Some(Box::new(Rkf45::upstream())),
            Self::Rkf78 => Some(Box::new(Rkf78::upstream())),
            Self::Euler => Some(Box::new(Euler)),
            Self::Rk2 => Some(Box::new(Heun)),
            Self::Rk3 => Some(Box::new(
                RungeKutta::new(RK3).expect("Kutta's tableau is a valid Runge-Kutta method"),
            )),
            Self::BogackiShampine => Some(Box::new(
                AdaptiveRungeKutta::new(BOGACKI_SHAMPINE)
                    .expect("Bogacki-Shampine is a valid embedded pair"),
            )),
        }
    }
}

/// `test_scenarioIntegrators.py`'s `INTEGRATOR_CASES`.
pub const TEST_CASES: [IntegratorCase; 6] = [
    IntegratorCase::Rk4,
    IntegratorCase::Rkf45,
    IntegratorCase::Rkf78,
    IntegratorCase::Euler,
    IntegratorCase::Rk2,
    IntegratorCase::BogackiShampine,
];

pub struct Results {
    pub csv_path: std::path::PathBuf,
    /// Upstream `dataLog`: the recorded states.
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    /// Upstream's `oe` after `rv2elem`.
    pub initial_elements: ClassicElements,
}

/// Upstream's LEO, before `rv2elem`.
pub fn leo_elements() -> ClassicElements {
    let degrees = std::f64::consts::PI / 180.0;
    ClassicElements {
        semi_major_axis: 7_000.0 * 1_000.0,
        eccentricity: 0.0001,
        inclination_rad: 33.3 * degrees,
        raan_rad: 48.2 * degrees,
        arg_periapsis_rad: 347.8 * degrees,
        true_anomaly_rad: 85.3 * degrees,
        ..Default::default()
    }
}

pub fn run(case: IntegratorCase) -> Results {
    let mu = Body::Earth.mu_m3ps2();
    let (position_m, velocity_mps) = elements_to_state(mu, &leo_elements());
    let elements = state_to_elements(mu, position_m, velocity_mps);
    let duration_nanos = sec2nano(0.75 * orbit_period_s(mu, elements.semi_major_axis));
    // Upstream's recorder samples every `samplingTime`; one below the task
    // period samples every task update.
    let sample_nanos = sampling_time(duration_nanos, STEP_NANOS, 100).max(STEP_NANOS);

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: case.integrator(),
    });
    spacecraft
        .add_grav_body(Body::Earth.point_mass(true))
        .expect("one central gravity body");

    let mut csv = scenario_csv(&format!("scenarioIntegrators{}", case.label()));
    let csv_path = csv.config.output_path.clone();
    let mut history = History::<SpacecraftStateMsg>::new();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        connect!(&simulation,
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &spacecraft.state_out => &mut history.input,
        );
        schedule! { simulation,
            "spacecraft" => &mut spacecraft, period=STEP_NANOS, priority=20;
            "csv" => &mut csv, period=sample_nanos, priority=0;
            "history" => &mut history, period=sample_nanos, priority=0;
        }
        simulation.run_for(duration_nanos);
    }
    csv.flush().expect("flush scenario CSV");
    Results {
        csv_path,
        states: history.into_samples(),
        initial_elements: elements,
    }
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let cases = if args.flag("--all") {
        TEST_CASES.to_vec()
    } else {
        let label = args.value("--integrator", "rk4".to_string());
        vec![
            IntegratorCase::ALL
                .into_iter()
                .find(|case| case.label() == label)
                .unwrap_or_else(|| args.fail(&format!("unknown integrator {label:?}"))),
        ]
    };
    for case in cases {
        let results = run(case);
        let (time, last) = results.states.last().expect("recorded states");
        println!(
            "wrote {} ({} samples, r_BN_N at {:.0} s = {:.3?} km)",
            results.csv_path.display(),
            results.states.len(),
            *time as f64 * 1.0e-9,
            last.position_m / 1_000.0
        );
    }
}

#[cfg(test)]
mod tests {
    use super::{IntegratorCase, TEST_CASES, run};

    /// `test_scenarioIntegrators.py`'s `truePos`: positions (m) at 0, 1080,
    /// 2160, 3240 and 4320 s.
    fn truth(case: IntegratorCase) -> [[f64; 3]; 5] {
        match case {
            IntegratorCase::Rk4 => [
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
            IntegratorCase::Rkf45 => [
                [-2816801.6010234924, 5248174.846916146, 3677157.264677298],
                [-6379400.585755646, -1468867.3018218754, 2480790.289504704],
                [-2230167.4261753717, -6410467.322406256, -1714612.6717163436],
                [4614825.264999291, -3602446.2861098843, -3837075.3154721106],
                [5879278.326923609, 3561255.294074021, -1319777.089191588],
            ],
            IntegratorCase::Euler => [
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
            IntegratorCase::Rk2 => [
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
            IntegratorCase::Rkf78 => [
                [-2816801.601023492, 5248174.846916147, 3677157.2646772973],
                [-6379401.371832086, -1468864.3054097842, 2480791.9863545913],
                [-2230174.317580403, -6410466.966948945, -1714609.1414605032],
                [4614818.45321768, -3602456.431072683, -3837076.4216056713],
                [5879286.370365726, 3561242.507948514, -1319786.6261035257],
            ],
            IntegratorCase::BogackiShampine => [
                [-2816801.601023492, 5248174.846916147, 3677157.2646772973],
                [-6380593.356301512, -1467889.8216560283, 2481802.3429243616],
                [-2234294.4280201513, -6411056.176926798, -1712849.5540222875],
                [4611359.466030476, -3607692.026362182, -3837674.903973699],
                [5882019.471581395, 3556295.378996462, -1323290.9895207654],
            ],
            IntegratorCase::Rk3 => unreachable!("upstream has no rk3 truth"),
        }
    }

    /// `test_scenarioIntegrators.py`, every parametrization: `posData[::skip]`
    /// with `skip = len // 4` against `truePos` with `compareArray` at
    /// `accuracy = 1.0` m (each component).
    #[test]
    fn recorded_positions_match_upstream_truth() {
        for case in TEST_CASES {
            let results = run(case);
            let skip = results.states.len() / 4;
            let sampled: Vec<_> = results.states.iter().step_by(skip).collect();
            assert_eq!(sampled.len(), 5, "{}: sample count", case.label());
            for (row, ((time, state), truth)) in sampled.iter().zip(truth(case)).enumerate() {
                assert_eq!(*time, row as u64 * 1_080_000_000_000);
                for axis in 0..3 {
                    let error = (state.position_m[axis] - truth[axis]).abs();
                    assert!(
                        error <= 1.0,
                        "{} row {row} axis {axis}: {} vs {} (error {error} m)",
                        case.label(),
                        state.position_m[axis],
                        truth[axis]
                    );
                }
            }
        }
    }
}
