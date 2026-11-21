//! Port of upstream `examples/scenarioImpact.py`: a 400 kg cylindrical
//! spacecraft with a flexible solar array of five 2 m sub-panels, each a
//! bending joint (a massless connector, `k = 2000`, `c = 150`) and a
//! torsional joint (the 20 kg panel, `k = 500`, `c = 75`), in a ten-body
//! [`SpinningBodyNDofStateEffector`]. Debris hits the third sub-panel at
//! 10 s: an external force and torque on its bending joint for one 0.01 s
//! step. After the impact the third sub-panel's hinges "break": stiffness
//! 200 N m/rad and no damping, for 20 s more.
//!
//! Upstream applies the impact through an `extForceTorque` attached to the
//! fifth body (`panel.addDynamicEffector(extFTObject, 5)`) whose
//! `extForce_B` and `extTorquePntB_B` it sets between runs; here the
//! [`ExtForceTorque`] is attached with
//! [`SpinningBodyNDofStateEffector::add_dynamic_effector`] (body index 4)
//! and the impact is written between runs to its body force and torque
//! command inputs, which it reads at every evaluation. The hinge changes are
//! made between runs through [`PanelEdits`], a handle on the effector the
//! spacecraft owns, applied before the next integration step as upstream's
//! setters take effect at the next step.
//!
//! The spacecraft flies an 8000 km, e = 0.1 Earth orbit with the Sun's
//! gravity; upstream's `createSpiceInterface` (zero base Earth, epoch
//! 2012 MAY 1 00:28:30) is basilisk-rs's ANISE ephemeris on DE440s
//! (`just fetch-anise-assets`). The angle and rate of the struck sub-panel's
//! two joints (upstream `spinningBodyOutMsgs[4]` and `[5]`) and the hub rate
//! go to `scenarioImpact.csv` at every step.
//!
//! Upstream has no `test_scenarioImpact.py`; `tests` checks what the
//! figures show: nothing moves before the impact, the struck joints deflect
//! and the hub starts to rotate after it, and the broken hinges keep
//! oscillating while the damped ones settle.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;

use std::any::Any;
use std::sync::{Arc, Mutex};

use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::spinning_body_n_dof_state_effector::{
    SpinningBody, SpinningBodyNDofStateEffector,
};
use basilisk_rs::messages::{
    BodyForceCommandMsg, BodyTorqueCommandMsg, HingedRigidBodyMsg, Output, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{
    BackSubMatrices, Spacecraft, SpacecraftConfig, StateEffector, StateEffectorMassProps,
};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{run_until, sec2nano};
use recording::scenario_csv;

const USAGE: &str = "scenario_impact     run the debris impact on the flexible array";

/// Upstream `simulationTimeStep`.
const STEP_NANOS: u64 = 10_000_000;
/// The struck body: upstream segment 5, and the two joints it records.
const STRUCK_BODY: usize = 4;
/// Upstream's impact load: `extForce_B` (N) and `extTorquePntB_B` (N m).
const IMPACT_FORCE_N: [f64; 3] = [3000.0, 4000.0, 5000.0];
const IMPACT_TORQUE_NM: [f64; 3] = [2000.0, 3000.0, 1000.0];

type Edit = Box<dyn FnOnce(&mut SpinningBodyNDofStateEffector) + Send>;

/// Changes to the panel made between runs (upstream's setters on the
/// attached `extForceTorque` and on `getSpinningBody(i)`), applied at the
/// start of the next integration step.
#[derive(Clone, Default)]
pub struct PanelEdits(Arc<Mutex<Vec<Edit>>>);

impl PanelEdits {
    pub fn edit(&self, edit: impl FnOnce(&mut SpinningBodyNDofStateEffector) + Send + 'static) {
        self.0.lock().expect("panel edits").push(Box::new(edit));
    }
}

/// The panel as the spacecraft holds it: the effector, taking the pending
/// [`PanelEdits`] before each step.
pub struct EditablePanel {
    pub inner: SpinningBodyNDofStateEffector,
    edits: PanelEdits,
}

impl StateEffector for EditablePanel {
    fn name(&self) -> &str {
        self.inner.name()
    }
    fn state_len(&self) -> usize {
        self.inner.state_len()
    }
    fn initial_state(&self) -> Vec<f64> {
        self.inner.initial_state()
    }
    fn load_state(&mut self, state: &[f64]) {
        self.inner.load_state(state);
    }
    fn pre_integration(&mut self, current_sim_nanos: u64, dt_seconds: f64) {
        for edit in self.edits.0.lock().expect("panel edits").drain(..) {
            edit(&mut self.inner);
        }
        self.inner.pre_integration(current_sim_nanos, dt_seconds);
    }
    fn update_contributions(
        &self,
        effector_state: &[f64],
        hub_state: &SpacecraftStateMsg,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        self.inner
            .update_contributions(effector_state, hub_state, gravity_body_mps2, back_sub);
    }
    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        self.inner
            .compute_derivatives(effector_state, body_trans_accel_mps2, body_omega_dot_radps2)
    }
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        self.inner.mass_properties(effector_state)
    }
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        self.inner
            .rotational_angular_momentum_body(effector_state, body_omega_radps)
    }
    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        self.inner
            .rotational_energy_j(effector_state, body_omega_radps)
    }
    fn write_outputs(&mut self, current_sim_nanos: u64, hub_state: &SpacecraftStateMsg) {
        self.inner.write_outputs(current_sim_nanos, hub_state);
    }
    fn modify_states(&mut self, current_sim_nanos: u64, effector_state: &mut [f64]) {
        self.inner.modify_states(current_sim_nanos, effector_state);
    }
    fn as_any(&self) -> &dyn Any {
        self.inner.as_any()
    }
    fn begin_stage(&mut self, stage_nanos: u64) {
        self.inner.begin_stage(stage_nanos);
    }
}

/// Upstream's array: five sub-panels, each a massless bending connector
/// (about x) and the panel (torsion about y).
fn solar_array(diameter: f64, height: f64) -> SpinningBodyNDofStateEffector {
    let length = diameter;
    let width = diameter;
    let thickness = 0.2;
    let mass = 20.0;
    let mut panel = SpinningBodyNDofStateEffector::new("panel");
    for index in 0..5 {
        panel.add_spinning_body(SpinningBody {
            mass_kg: 0.0,
            inertia_about_com_spinner_kg_m2: Matrix3::zeros(),
            parent_to_spinner0_dcm: Matrix3::identity(),
            center_of_mass_spinner_m: Vector3::new(0.0, length / 2.0, 0.0),
            origin_parent_m: if index == 0 {
                Vector3::new(0.0, diameter / 2.0, height / 2.0 - thickness / 2.0)
            } else {
                Vector3::new(0.0, length, 0.0)
            },
            spin_axis_spinner: Vector3::x(),
            theta_init_rad: 0.0,
            theta_dot_init_radps: 0.0,
            spring_constant_nm_per_rad: 2000.0,
            damping_nm_s_per_rad: 150.0,
        });
        panel.add_spinning_body(SpinningBody {
            mass_kg: mass,
            inertia_about_com_spinner_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                mass / 12.0 * (length.powi(2) + thickness.powi(2)),
                mass / 12.0 * (width.powi(2) + thickness.powi(2)),
                mass / 12.0 * (width.powi(2) + length.powi(2)),
            )),
            parent_to_spinner0_dcm: Matrix3::identity(),
            center_of_mass_spinner_m: Vector3::new(0.0, length / 2.0, 0.0),
            origin_parent_m: Vector3::zeros(),
            spin_axis_spinner: Vector3::y(),
            theta_init_rad: 0.0,
            theta_dot_init_radps: 0.0,
            spring_constant_nm_per_rad: 500.0,
            damping_nm_s_per_rad: 75.0,
        });
    }
    panel
}

pub struct Results {
    pub csv_path: std::path::PathBuf,
    /// Upstream `theta1Data`, `theta2Data`: the struck joints.
    pub joint1: Vec<(u64, HingedRigidBodyMsg)>,
    pub joint2: Vec<(u64, HingedRigidBodyMsg)>,
    /// Upstream `scStateData`.
    pub states: Vec<(u64, SpacecraftStateMsg)>,
}

pub fn run() -> Results {
    let mass = 400.0;
    let diameter = 2.0;
    let height = 4.0;
    let transverse = mass / 16.0 * diameter * diameter + mass / 12.0 * height * height;
    let earth = Body::Earth;
    let (position_m, velocity_mps) = elements_to_state(
        earth.mu_m3ps2(),
        &ClassicElements {
            semi_major_axis: 8e6,
            eccentricity: 0.1,
            ..Default::default()
        },
    );
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: mass,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(
            transverse,
            transverse,
            mass / 8.0 * diameter * diameter,
        )),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(earth.point_mass(true))
        .expect("unique gravity body");
    spacecraft
        .add_grav_body(Body::Sun.point_mass(false))
        .expect("unique gravity body");
    let mut earth_ephemeris = earth.ephemeris(Body::Earth);
    let mut sun_ephemeris = Body::Sun.ephemeris(Body::Earth);

    let mut panel = solar_array(diameter, height);
    // Upstream `extFTObject`, attached to the struck body; the impact is
    // written to its commands.
    let impact_force = Output::new(BodyForceCommandMsg::default());
    let impact_torque = Output::new(BodyTorqueCommandMsg::default());
    let mut impact = ExtForceTorque::new(ExtForceTorqueConfig {
        name: "extForceTorque".to_string(),
        ..Default::default()
    });
    impact_force.connect_to(&mut impact.cmd_force_body_in_msg);
    impact_torque.connect_to(&mut impact.cmd_torque_in_msg);
    panel.add_dynamic_effector(STRUCK_BODY, impact);
    let edits = PanelEdits::default();
    let mut csv = scenario_csv("scenarioImpact");
    let csv_path = csv.config.output_path.clone();
    let mut joint1_log = MessageLog::<HingedRigidBodyMsg>::default();
    let mut joint2_log = MessageLog::<HingedRigidBodyMsg>::default();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    // Upstream `timeInitString = "2012 MAY 1 00:28:30.0"`.
    let epoch = Epoch::from_gregorian_utc(2012, 5, 1, 0, 28, 30, 0);
    {
        let mut simulation = Simulation::new(epoch, false);
        connect!(&simulation,
            &earth_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("earth").expect("earth"),
            &sun_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("sun").expect("sun"),
            &panel.spinning_body_out[STRUCK_BODY] => csv.add_source::<HingedRigidBodyMsg>(
                CsvSourceConfig::columns([
                    ("theta_rad", "theta1_rad"),
                    ("theta_dot_radps", "theta1Dot_rad_per_s"),
                ]),
            ),
            &panel.spinning_body_out[STRUCK_BODY + 1] => csv.add_source::<HingedRigidBodyMsg>(
                CsvSourceConfig::columns([
                    ("theta_rad", "theta2_rad"),
                    ("theta_dot_radps", "theta2Dot_rad_per_s"),
                ]),
            ),
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(
                CsvSourceConfig::columns(vector_columns("omega_radps", "omega_BN_B_rad_per_s")),
            ),
            &panel.spinning_body_out[STRUCK_BODY] => &mut joint1_log.input,
            &panel.spinning_body_out[STRUCK_BODY + 1] => &mut joint2_log.input,
            &spacecraft.state_out => &mut state_log.input,
        );
        spacecraft.add_state_effector(EditablePanel {
            inner: panel,
            edits: edits.clone(),
        });
        schedule! { simulation,
            "earth_ephemeris" => &mut earth_ephemeris, period=STEP_NANOS, priority=1;
            "sun_ephemeris" => &mut sun_ephemeris, period=STEP_NANOS, priority=1;
            "scObject" => &mut spacecraft, period=STEP_NANOS, priority=0;
            "csv" => &mut csv, period=STEP_NANOS, priority=-1;
            "joint1_log" => &mut joint1_log, period=STEP_NANOS, priority=-1;
            "joint2_log" => &mut joint2_log, period=STEP_NANOS, priority=-1;
            "state_log" => &mut state_log, period=STEP_NANOS, priority=-1;
        }
        let mut stop_nanos = sec2nano(10.0);
        run_until(&mut simulation, stop_nanos);

        // The impact, for one step.
        impact_force.write(BodyForceCommandMsg {
            force_request_body_n: Vector3::from(IMPACT_FORCE_N),
        });
        impact_torque.write(BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::from(IMPACT_TORQUE_NM),
            ..Default::default()
        });
        stop_nanos += STEP_NANOS;
        run_until(&mut simulation, stop_nanos);

        // The hinges of the struck sub-panel break.
        impact_force.write(BodyForceCommandMsg::default());
        impact_torque.write(BodyTorqueCommandMsg::default());
        edits.edit(|panel| {
            for index in [STRUCK_BODY, STRUCK_BODY + 1] {
                let body = panel.spinning_body_mut(index);
                body.spring_constant_nm_per_rad = 200.0;
                body.damping_nm_s_per_rad = 0.0;
            }
        });
        stop_nanos += sec2nano(20.0);
        run_until(&mut simulation, stop_nanos);
    }
    csv.flush().expect("flush scenario CSV");
    Results {
        csv_path,
        joint1: joint1_log.into_samples(),
        joint2: joint2_log.into_samples(),
        states: state_log.into_samples(),
    }
}

fn main() {
    Cli::from_env(USAGE);
    let results = run();
    let peak = |log: &[(u64, HingedRigidBodyMsg)]| {
        log.iter()
            .fold(0.0_f64, |m, (_, joint)| m.max(joint.theta_rad.abs()))
            .to_degrees()
    };
    println!(
        "wrote {} (peak deflections {:.2} and {:.2} deg, final hub rate {:.3e} rad/s)",
        results.csv_path.display(),
        peak(&results.joint1),
        peak(&results.joint2),
        results.states.last().unwrap().1.omega_radps.norm()
    );
}

#[cfg(test)]
mod tests {
    use super::run;

    /// No upstream test. Before the impact nothing moves; it deflects the
    /// struck joints and sets the hub rotating; the broken (undamped)
    /// hinges keep swinging over the last seconds.
    #[test]
    fn impact_deflects_the_panel_and_rotates_the_hub() {
        let results = run();
        let impact = 10_000_000_000;
        let before = |log: &[(u64, basilisk_rs::messages::HingedRigidBodyMsg)]| {
            log.iter()
                .filter(|(t, _)| *t <= impact)
                .fold(0.0_f64, |m, (_, j)| m.max(j.theta_rad.abs()))
        };
        assert!(before(&results.joint1) < 1e-9 && before(&results.joint2) < 1e-9);
        let hub_before = results
            .states
            .iter()
            .filter(|(t, _)| *t <= impact)
            .fold(0.0_f64, |m, (_, s)| m.max(s.omega_radps.norm()));
        assert!(hub_before < 1e-9, "hub rate before the impact {hub_before}");

        for log in [&results.joint1, &results.joint2] {
            let peak = log
                .iter()
                .fold(0.0_f64, |m, (_, j)| m.max(j.theta_rad.abs()));
            assert!(
                peak.to_degrees() > 1.0,
                "peak deflection {} deg",
                peak.to_degrees()
            );
            let late = log
                .iter()
                .filter(|(t, _)| *t > 25_000_000_000)
                .fold(0.0_f64, |m, (_, j)| m.max(j.theta_dot_radps.abs()));
            assert!(late > 1e-3, "broken hinge rate late in the run {late}");
        }
        let hub_after = results.states.last().unwrap().1.omega_radps.norm();
        assert!(hub_after > 1e-4, "hub rate after the impact {hub_after}");
        assert_eq!(results.states.last().unwrap().0, 30_010_000_000);
    }
}
