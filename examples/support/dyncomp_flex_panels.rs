//! The hub with two discretized flexible solar arrays of upstream
//! `scenarioCompareFlexPanels.py` and `scenarioCompareParetoFlexPanels.py`:
//! each array is a serial chain of `N` rigid segments joined by torsional
//! spring-damper hinges, built three ways (upstream `buildBSM`,
//! `buildBsmNDOF`, `buildMujoco`):
//!
//! * the BSM spacecraft with an [`NHingedRigidBodyStateEffector`] per array
//!   (one `HingedPanel` per segment);
//! * the BSM spacecraft with a [`SpinningBodyNDofStateEffector`] per array
//!   (one `SpinningBody` per segment, hinged about `+y` at the previous
//!   segment's far end, `-2d`, its center of mass at `-d`);
//! * with the `mujoco` feature, an `MjScene` with a chain of hinge-jointed
//!   bodies per array.
//!
//! The root segment of each chain starts deflected; the hub starts with
//! upstream's `OMEGA0_B`.

#![allow(dead_code)]

use basilisk_rs::connect;
use basilisk_rs::dynamics::n_hinged_rigid_body_state_effector::{
    HingedPanel, NHingedRigidBodyConfig, NHingedRigidBodyStateEffector,
};
use basilisk_rs::dynamics::spinning_body_n_dof_state_effector::{
    SpinningBody, SpinningBodyNDofStateEffector,
};
use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use nalgebra::{Matrix3, Vector3};

use super::dyncomp::{IntegratorChoice, sec2nano_u64, timed_run};
use super::dyncomp_orbit::epoch;
use super::message_log::MessageLog;

pub const HUB_MASS_KG: f64 = 600.0;
pub const HUB_INERTIA: [f64; 3] = [400.0, 380.0, 360.0];
/// Upstream `PANEL_TOTAL_MASS` \[kg\], `PANEL_TOTAL_LEN` \[m\]: one array.
pub const PANEL_TOTAL_MASS_KG: f64 = 60.0;
pub const PANEL_TOTAL_LEN_M: f64 = 3.0;
/// Upstream `PANEL_HINGE_X` \[m\]: root hinge offset along x.
pub const PANEL_HINGE_X_M: f64 = 0.8;
/// Upstream `ROOT_THETA0` \[rad\]: initial root-segment deflection, 8°.
pub const ROOT_THETA0_RAD: f64 = 8.0 * std::f64::consts::PI / 180.0;
pub const OMEGA0_B: [f64; 3] = [0.010, -0.020, 0.015];

/// One chained-array model: the segment count and the per-hinge
/// stiffness and damping.
#[derive(Clone, Copy, Debug)]
pub struct Chain {
    pub n_segments: usize,
    /// Upstream `PANEL_K` \[N m/rad\].
    pub k: f64,
    /// Upstream `PANEL_C` \[N m s/rad\].
    pub c: f64,
}

impl Chain {
    /// `scenarioCompareFlexPanels`: `k = 600`, `c = 5`.
    pub fn flex_panels(n_segments: usize) -> Self {
        Self {
            n_segments,
            k: 600.0,
            c: 5.0,
        }
    }

    /// `scenarioCompareParetoFlexPanels`: 16 lightly damped segments,
    /// `k = 120`, `c = 0.05`.
    pub fn pareto() -> Self {
        Self {
            n_segments: 16,
            k: 120.0,
            c: 0.05,
        }
    }

    /// Upstream `segmentMass` \[kg\].
    pub fn segment_mass(&self) -> f64 {
        PANEL_TOTAL_MASS_KG / self.n_segments as f64
    }

    /// Upstream `segmentHalfLength` `d` \[m\].
    pub fn segment_half_length(&self) -> f64 {
        PANEL_TOTAL_LEN_M / (2.0 * self.n_segments as f64)
    }

    /// Upstream `segmentInertia`: a slender bar with a small axial inertia
    /// \[kg m^2\].
    pub fn segment_inertia(&self) -> Matrix3<f64> {
        let mass = self.segment_mass();
        let length = 2.0 * self.segment_half_length();
        let bar = mass * length * length / 12.0;
        let axial = (bar * 0.05).max(1.0e-4);
        Matrix3::from_diagonal(&Vector3::new(axial, bar, bar))
    }

    /// The system degrees of freedom: six hub coordinates and one per hinge.
    pub fn dof(&self) -> usize {
        6 + 2 * self.n_segments
    }
}

/// Upstream `panelRootDcm`.
pub fn panel_root_dcm(sign: f64) -> Matrix3<f64> {
    if sign > 0.0 {
        Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0))
    } else {
        Matrix3::identity()
    }
}

/// Which BSM effector models the arrays.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BsmChain {
    /// `nHingedRigidBodyStateEffector`.
    NHinged,
    /// `spinningBodyNDOFStateEffector`.
    SpinningBodyNDof,
}

fn hub(step: u64, integrator: IntegratorChoice) -> Spacecraft {
    Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::from(HUB_INERTIA)),
        integration_step_nanos: step,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::from(OMEGA0_B),
        integrator: Some(integrator.integrator()),
    })
}

fn add_n_hinged_chains(spacecraft: &mut Spacecraft, chain: &Chain) {
    for sign in [1.0, -1.0] {
        let mut effector = NHingedRigidBodyStateEffector::new(NHingedRigidBodyConfig {
            hinge_position_body_m: Vector3::new(sign * PANEL_HINGE_X_M, 0.0, 0.0),
            body_to_hinge_dcm: panel_root_dcm(sign),
            ..NHingedRigidBodyConfig::new(if sign > 0.0 { "chainP" } else { "chainM" })
        });
        for segment in 0..chain.n_segments {
            effector.add_hinged_panel(HingedPanel {
                mass_kg: chain.segment_mass(),
                center_of_mass_offset_m: chain.segment_half_length(),
                spring_constant_nm_per_rad: chain.k,
                damping_nm_s_per_rad: chain.c,
                inertia_about_panel_com_panel_kg_m2: chain.segment_inertia(),
                theta_init_rad: if segment == 0 { ROOT_THETA0_RAD } else { 0.0 },
                ..HingedPanel::default()
            });
        }
        spacecraft.add_state_effector(effector);
    }
}

fn add_spinning_body_chains(spacecraft: &mut Spacecraft, chain: &Chain) {
    let half_length = chain.segment_half_length();
    for sign in [1.0, -1.0] {
        let mut effector = SpinningBodyNDofStateEffector::new(if sign > 0.0 {
            "ndofChainP"
        } else {
            "ndofChainM"
        });
        for segment in 0..chain.n_segments {
            effector.add_spinning_body(SpinningBody {
                mass_kg: chain.segment_mass(),
                spring_constant_nm_per_rad: chain.k,
                damping_nm_s_per_rad: chain.c,
                theta_init_rad: if segment == 0 { ROOT_THETA0_RAD } else { 0.0 },
                theta_dot_init_radps: 0.0,
                // Link 0 hinges on the hub at the root offset, each later one
                // at the previous segment's far end.
                origin_parent_m: if segment == 0 {
                    Vector3::new(sign * PANEL_HINGE_X_M, 0.0, 0.0)
                } else {
                    Vector3::new(-2.0 * half_length, 0.0, 0.0)
                },
                center_of_mass_spinner_m: Vector3::new(-half_length, 0.0, 0.0),
                spin_axis_spinner: Vector3::y(),
                inertia_about_com_spinner_kg_m2: chain.segment_inertia(),
                parent_to_spinner0_dcm: if segment == 0 {
                    panel_root_dcm(sign)
                } else {
                    Matrix3::identity()
                },
            });
        }
        spacecraft.add_state_effector(effector);
    }
}

/// The recorded hub histories.
#[derive(Clone, Debug, Default)]
pub struct HubHistory {
    pub times_nanos: Vec<u64>,
    pub sigma_bn: Vec<Vector3<f64>>,
    pub r_bn_n: Vec<Vector3<f64>>,
}

impl HubHistory {
    fn from_log(log: &MessageLog<SpacecraftStateMsg>) -> Self {
        Self {
            times_nanos: log.times_nanos.clone(),
            sigma_bn: log.values.iter().map(|s| s.sigma_bn).collect(),
            r_bn_n: log.values.iter().map(|s| s.position_m).collect(),
        }
    }
}

/// Upstream `buildBSM` / `buildBsmNDOF`, propagated to `tf_s`: the hub
/// history, recorded every step with `record`, and the propagation seconds.
pub fn run_bsm(
    model: BsmChain,
    chain: &Chain,
    integrator: IntegratorChoice,
    dt_s: f64,
    tf_s: f64,
    record: bool,
) -> (HubHistory, f64) {
    let step = sec2nano_u64(dt_s);
    let mut spacecraft = hub(step, integrator);
    match model {
        BsmChain::NHinged => add_n_hinged_chains(&mut spacecraft, chain),
        BsmChain::SpinningBodyNDof => add_spinning_body_chains(&mut spacecraft, chain),
    }
    let mut log = MessageLog::<SpacecraftStateMsg>::default();
    let seconds = {
        let mut simulation = Simulation::new(epoch(), false);
        if record {
            connect!(&simulation, &spacecraft.state_out => &mut log.input);
        }
        simulation.add_module("hub", &mut spacecraft, step, 10);
        if record {
            simulation.add_module("recorder", &mut log, step, 0);
        }
        timed_run(&mut simulation, sec2nano_u64(tf_s))
    };
    (HubHistory::from_log(&log), seconds)
}

#[cfg(feature = "mujoco")]
#[allow(unused_imports)]
pub use mujoco_side::{mujoco_model, run_mujoco};

#[cfg(feature = "mujoco")]
mod mujoco_side {
    use super::*;
    use crate::dyncomp::PreparedScene;
    use basilisk_rs::mujoco_dynamics::MjScene;

    /// Upstream `mujocoModel(nSegments)`: the hub with two hinged chains.
    pub fn mujoco_model(chain: &Chain) -> String {
        let [ix, iy, iz] = HUB_INERTIA;
        let half_length = chain.segment_half_length();
        let mass = chain.segment_mass();
        let inertia = chain.segment_inertia();
        let (k, c) = (chain.k, chain.c);
        let arm = |sign: f64, tag: &str| {
            let mut body = String::new();
            for segment in (0..chain.n_segments).rev() {
                let name = format!("{tag}_{segment}");
                let pos = if segment == 0 {
                    format!("{} 0 0", sign * PANEL_HINGE_X_M)
                } else {
                    format!("{} 0 0", -2.0 * half_length)
                };
                let root_quat = if segment == 0 && sign > 0.0 {
                    r#" quat="0 0 0 1""#
                } else {
                    ""
                };
                body = format!(
                    r#"<body name="{name}" pos="{pos}"{root_quat}><joint name="{name}" type="hinge" axis="0 1 0" stiffness="{k}" damping="{c}" springref="0"/><inertial pos="{} 0 0" mass="{mass:e}" fullinertia="{:e} {:e} {:e} 0 0 0"/>{body}</body>"#,
                    -half_length,
                    inertia[(0, 0)],
                    inertia[(1, 1)],
                    inertia[(2, 2)],
                );
            }
            body
        };
        format!(
            r#"
<mujoco>
  <option gravity="0 0 0">
    <flag contact="disable" constraint="disable"/>
  </option>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" mass="{HUB_MASS_KG}" fullinertia="{ix} {iy} {iz} 0 0 0"/>
      {}
      {}
    </body>
  </worldbody>
</mujoco>
"#,
            arm(1.0, "pP"),
            arm(-1.0, "pM")
        )
    }

    /// Upstream `buildMujoco(nSegments, dt, record)`, propagated to `tf_s`.
    pub fn run_mujoco(
        chain: &Chain,
        integrator: IntegratorChoice,
        dt_s: f64,
        tf_s: f64,
        record: bool,
    ) -> (HubHistory, f64) {
        let step = sec2nano_u64(dt_s);
        let mut scene = MjScene::new(&mujoco_model(chain)).expect("flexible-panel scene");
        scene.name = "hubMj".to_string();
        scene.extra_eom_call = true;
        scene.set_high_order_attitude_integration(true);
        scene.set_boxed_integrator(integrator.integrator());
        let hub_state = scene.body("hub").origin().state_out_msg.clone();
        let mut scene = PreparedScene::new(scene, |scene| {
            scene
                .body_mut("hub")
                .set_attitude_rate(Vector3::from(OMEGA0_B));
            for tag in ["pP_0", "pM_0"] {
                scene
                    .body_mut(tag)
                    .set_scalar_joint_position(tag, ROOT_THETA0_RAD);
            }
        });
        let mut log = MessageLog::<SpacecraftStateMsg>::default();
        let seconds = {
            let mut simulation = Simulation::new(epoch(), false);
            if record {
                connect!(&simulation, &hub_state => &mut log.input);
            }
            simulation.add_module("hubMj", &mut scene, step, 1);
            if record {
                simulation.add_module("recorder", &mut log, step, 0);
            }
            timed_run(&mut simulation, sec2nano_u64(tf_s))
        };
        (HubHistory::from_log(&log), seconds)
    }
}
