//! The servicing setup of upstream's `scenarioConstrainedDynamicsComponentAnalysis`
//! and `scenarioConstrainedDynamicsFrequencyAnalysis`: a chaser and a target
//! spacecraft rigidly docked through a [`ConstraintDynamicEffector`]
//! (`alpha = beta = gain`), each optionally with a hinged solar panel and the
//! chaser with a fuel tank of three slosh particles, next to a single
//! spacecraft that models the docked stack as one rigid hub carrying the
//! same appendages (upstream's "truth model").
//!
//! As upstream sets them after `InitializeSimulation`, the panel rate and
//! the slosh rates are perturbations: the hub states are converted from the
//! centers of mass ([`hub_init`](super::hub_init)) with the appendages at
//! rest, then the appendages start with the perturbed rates.

#![allow(dead_code)]

use basilisk_rs::dynamics::fuel_tank::{FuelTank, FuelTankConfig, FuelTankModel, FuelTankShape};
use basilisk_rs::dynamics::hinged_rigid_body_state_effector::{
    HingedRigidBodyConfig, HingedRigidBodyStateEffector,
};
use basilisk_rs::dynamics::linear_spring_mass_damper::{
    LinearSpringMassDamper, LinearSpringMassDamperConfig,
};
use basilisk_rs::messages::{HingedRigidBodyMsg, LinearTranslationRigidBodyMsg, Output};
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftGroup};
use basilisk_rs::utilities::rigid_body_kinematics::tilde;
use nalgebra::{Matrix3, Vector3};

use super::constrained::{ConstraintGeometry, ScModel, hub, model_inertia_kg_m2, upstream_rkf45};
use super::hub_init::set_center_of_mass_initial_state;
use super::probed_effector::Probed;

/// Upstream `solar_panel_geometry.mass` \[kg\].
pub const PANEL_MASS_KG: f64 = 100.0;
/// Upstream `solar_panel_geometry.d` \[m\]: hinge to panel center of mass.
pub const PANEL_ARM_M: f64 = 1.0;
/// Upstream `slosh_fuel_tank_geometry.massInit` \[kg\]: each particle.
pub const PARTICLE_MASS_KG: f64 = 50.0;
/// Upstream `slosh_fuel_tank_geometry.propMassInit` \[kg\].
pub const PROPELLANT_MASS_KG: f64 = 350.0;
/// Upstream's mid-simulation panel perturbation: `thetaDot = 7 deg/s` of
/// the chaser's panel and the truth model's first panel.
pub const PANEL_RATE_PERTURBATION_RADPS: f64 = 7.0 * std::f64::consts::PI / 180.0;
/// Upstream's mid-simulation slosh perturbations `rhoDot` \[m/s\] of the
/// three particles.
pub const SLOSH_RATE_PERTURBATIONS_MPS: [f64; 3] = [0.02, -0.025, -0.015];

/// Upstream `solar_panel_geometry.IPntS_S` \[kg m^2\].
pub fn panel_inertia_kg_m2() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(150.0, 300.0, 150.0))
}

/// Upstream `solar_panel_geometry.r_H1B_B` and `r_H2B_B` \[m\].
pub fn hinge_positions_m() -> [Vector3<f64>; 2] {
    [Vector3::new(0.0, 1.0, 1.0), Vector3::new(0.0, -1.0, -1.0)]
}

/// Upstream `solar_panel_geometry.dcm_H1B` and `dcm_H2B`.
pub fn hinge_dcms() -> [Matrix3<f64>; 2] {
    [
        Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        Matrix3::new(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    ]
}

/// Upstream `solar_panel_geometry`: a 2 Hz, 20 % damped hinged panel at
/// `hinge_position_m` with `dcm_HB`, starting at rest at zero angle unless
/// `theta_dot_radps` is set.
pub fn solar_panel(
    name: &str,
    hinge_position_m: Vector3<f64>,
    dcm_hb: Matrix3<f64>,
    theta_dot_radps: f64,
) -> HingedRigidBodyStateEffector {
    let inertia = panel_inertia_kg_m2();
    let xi = 0.2;
    let frequency_radps = 2.0 * 2.0 * std::f64::consts::PI;
    HingedRigidBodyStateEffector::new(HingedRigidBodyConfig {
        mass_kg: PANEL_MASS_KG,
        center_of_mass_offset_m: PANEL_ARM_M,
        spring_constant_nm_per_rad: inertia[(1, 1)] * frequency_radps * frequency_radps,
        damping_nm_s_per_rad: inertia[(1, 1)] * 2.0 * xi * frequency_radps,
        inertia_about_panel_com_panel_kg_m2: inertia,
        hinge_position_body_m: hinge_position_m,
        body_to_hinge_dcm: dcm_hb,
        theta_init_rad: 0.0,
        theta_dot_init_radps: theta_dot_radps,
        ..HingedRigidBodyConfig::new(name)
    })
}

/// Upstream `slosh_fuel_tank_geometry.r_P1B_B`, `r_P2B_B`, `r_P3B_B` \[m\].
pub fn particle_positions_m() -> [Vector3<f64>; 3] {
    [
        Vector3::new(0.01, 0.0, 0.01),
        Vector3::new(-0.01, 0.01, 0.0),
        Vector3::new(0.01, 0.0, 0.01),
    ]
}

/// Upstream `slosh_fuel_tank_geometry.r_TB_B` \[m\].
pub fn tank_position_m() -> Vector3<f64> {
    Vector3::new(0.0, 0.0, 0.1)
}

/// Upstream's fuel tank (`FuelTankModelConstantVolume`, 350 kg in a 0.5 m
/// sphere, `updateOnly`) at `r_TB_B + offset` with three 2 Hz, 20 % damped
/// slosh particles along x̂, ŷ and ẑ at `r_PiB_B + offset`, starting at
/// rest unless `rho_dots_mps` are set. Each particle's `ρ` and `ρ̇` go to
/// `rho_out` after every step (upstream logs them from the state manager).
pub fn slosh_tank(
    name: &str,
    offset_m: Vector3<f64>,
    rho_dots_mps: [f64; 3],
    rho_out: ParticleOutputs,
) -> Probed<FuelTank> {
    let xi = 0.2;
    let frequency_radps = 2.0 * 2.0 * std::f64::consts::PI;
    let model = FuelTankModel::new(
        FuelTankShape::ConstantVolume {
            radius_tank_init_m: 0.5,
        },
        PROPELLANT_MASS_KG,
    );
    let mut tank = FuelTank::new(FuelTankConfig {
        r_tb_b_m: tank_position_m() + offset_m,
        update_only: true,
        ..FuelTankConfig::new(name, model)
    });
    let directions = [Vector3::x(), Vector3::y(), Vector3::z()];
    for (index, (position_m, direction)) in particle_positions_m()
        .into_iter()
        .zip(directions)
        .enumerate()
    {
        tank.push_fuel_slosh_particle(LinearSpringMassDamper::new(LinearSpringMassDamperConfig {
            spring_constant_npm: PARTICLE_MASS_KG * frequency_radps * frequency_radps,
            damping_ns_per_m: PARTICLE_MASS_KG * 2.0 * xi * frequency_radps,
            rho_init_m: 0.0,
            rho_dot_init_mps: rho_dots_mps[index],
            mass_init_kg: PARTICLE_MASS_KG,
            equilibrium_position_body_m: position_m + offset_m,
            direction_body: direction,
            ..LinearSpringMassDamperConfig::new(format!("{name}Particle{}", index + 1))
        }));
    }
    Probed::new(tank, move |tank: &FuelTank, _| {
        for (particle, output) in tank.fuel_slosh_particles().iter().zip(&rho_out) {
            let particle = particle
                .as_any()
                .downcast_ref::<LinearSpringMassDamper>()
                .expect("slosh particle");
            output.write(LinearTranslationRigidBodyMsg {
                rho_m: particle.rho_m,
                rho_dot_mps: particle.rho_dot_mps,
            });
        }
    })
}

/// The `hingedRigidBodyOutMsg` of panels 1 to 4.
pub type PanelOutputs = [Output<HingedRigidBodyMsg>; 4];

/// The `ρ`, `ρ̇` of three slosh particles.
pub type ParticleOutputs = [Output<LinearTranslationRigidBodyMsg>; 3];

/// Which appendages the spacecraft carry (upstream `component_list`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Components {
    /// `"panels"`: a hinged panel on each spacecraft (and two on the truth
    /// model).
    pub panels: bool,
    /// `"slosh"`: the slosh tank on the chaser (and on the truth model).
    pub slosh: bool,
}

/// The servicing setup's parameters.
#[derive(Clone, Copy, Debug)]
pub struct ServicingConfig {
    pub model: ScModel,
    pub components: Components,
    /// Direction from chaser to target (upstream `rHat`).
    pub r_hat: Vector3<f64>,
    /// Constraint `alpha` and `beta`.
    pub gain: f64,
    /// Upstream `dynRate` \[ns\].
    pub step_nanos: u64,
}

/// The docked pair, the truth model and the outputs the scenarios log.
pub struct Servicing {
    pub group: SpacecraftGroup,
    pub truth: Spacecraft,
    pub geometry: ConstraintGeometry,
    /// `hingedRigidBodyOutMsg` of panels 1 (chaser), 2 (target), 3 and 4
    /// (truth), with `panels`.
    pub panel_out: Option<PanelOutputs>,
    /// `ρ`, `ρ̇` of particles 1–3 (chaser), with `slosh`.
    pub rho_out: Option<ParticleOutputs>,
    /// `ρ`, `ρ̇` of particles 4–6 (truth), with `slosh`.
    pub rho_truth_out: Option<ParticleOutputs>,
}

fn particle_outputs() -> ParticleOutputs {
    std::array::from_fn(|_| Output::default())
}

/// Upstream `set_up_spacecraft`, `set_up_initial_conditions` (or
/// `define_initial_conditions`), `set_up_constraint_effector` and
/// `set_up_additional_effectors`, with the perturbations of
/// `run_simulation`.
pub fn servicing(config: &ServicingConfig) -> Servicing {
    let model = config.model.data();
    let inertia = model_inertia_kg_m2();
    let step = config.step_nanos;
    let mut chaser = hub(model.chaser_mass_kg, Vector3::zeros(), inertia, step);
    let mut target = hub(model.target_mass_kg, Vector3::zeros(), inertia, step);

    // The N, B1 and B2 frames are initially aligned.
    let r_cn = Vector3::new(1.0, 0.0, 0.0);
    let geometry = ConstraintGeometry::along(config.r_hat, &model);
    let r_b2b1 = geometry.r_b2b1_m();

    // Center of mass of each spacecraft relative to its body origin, as
    // upstream computes it (its target expression weighs the zero target
    // hub offset with the chaser mass, which leaves it zero).
    let [r_h1b, r_h2b] = hinge_positions_m();
    let [dcm_h1b, dcm_h2b] = hinge_dcms();
    let panel_com =
        |hinge: Vector3<f64>, dcm: &Matrix3<f64>| hinge - dcm.row(0).transpose() * PANEL_ARM_M;
    let mut r_c1b1 = Vector3::zeros();
    let mut r_c2b2 = Vector3::zeros();
    let mut total_mass_1 = model.chaser_mass_kg;
    let mut total_mass_2 = model.target_mass_kg;
    if config.components.panels {
        r_c1b1 = (total_mass_1 * r_c1b1 + PANEL_MASS_KG * panel_com(r_h1b, &dcm_h1b))
            / (total_mass_1 + PANEL_MASS_KG);
        r_c2b2 = (total_mass_1 * r_c2b2 + PANEL_MASS_KG * panel_com(r_h2b, &dcm_h2b))
            / (total_mass_2 + PANEL_MASS_KG);
        total_mass_1 += PANEL_MASS_KG;
        total_mass_2 += PANEL_MASS_KG;
    }
    if config.components.slosh {
        let particles: Vector3<f64> = particle_positions_m().iter().sum();
        r_c1b1 = (total_mass_1 * r_c1b1
            + PARTICLE_MASS_KG * particles
            + PROPELLANT_MASS_KG * tank_position_m())
            / (total_mass_1 + 3.0 * PARTICLE_MASS_KG + PROPELLANT_MASS_KG);
        total_mass_1 += 3.0 * PARTICLE_MASS_KG + PROPELLANT_MASS_KG;
    }
    let r_b1c =
        -(total_mass_1 * r_c1b1 + total_mass_2 * (r_c2b2 + r_b2b1)) / (total_mass_1 + total_mass_2);
    let r_b2c = r_b1c + r_b2b1;

    // Truth model: both hubs as one rigid body about their joint center of
    // mass Bc.
    let (m1, m2) = (model.chaser_mass_kg, model.target_mass_kg);
    let r_bcc = (m1 * r_b1c + m2 * r_b2c) / (m1 + m2);
    let tilde_1 = tilde(r_b1c - r_bcc);
    let tilde_2 = tilde(r_b2c - r_bcc);
    let inertia_truth = inertia - m1 * tilde_1 * tilde_1 + inertia - m2 * tilde_2 * tilde_2;
    let mut truth = hub(m1 + m2, Vector3::zeros(), inertia_truth, step);
    truth.config.integrator = Some(Box::new(upstream_rkf45()));

    let mut panel_out = None;
    let (mut rho_out, mut rho_truth_out) = (None, None);
    let truth_offset_1 = r_b1c - r_bcc;
    let truth_offset_2 = r_b2c - r_bcc;
    // The appendages at the given perturbation scale (0: at rest, 1:
    // perturbed), with their outputs.
    let add_appendages = |scale: f64,
                          chaser: &mut Spacecraft,
                          target: &mut Spacecraft,
                          truth: &mut Spacecraft|
     -> (Option<PanelOutputs>, Option<[ParticleOutputs; 2]>) {
        let mut panels = None;
        if config.components.panels {
            let rate = scale * PANEL_RATE_PERTURBATION_RADPS;
            let panel_1 = solar_panel("solarPanel1", r_h1b, dcm_h1b, rate);
            let panel_2 = solar_panel("solarPanel2", r_h2b, dcm_h2b, 0.0);
            let panel_3 = solar_panel("solarPanel3", r_h1b + truth_offset_1, dcm_h1b, rate);
            let panel_4 = solar_panel("solarPanel4", r_h2b + truth_offset_2, dcm_h2b, 0.0);
            panels = Some(
                [&panel_1, &panel_2, &panel_3, &panel_4]
                    .map(|panel| panel.hinged_rigid_body_out.clone()),
            );
            chaser.add_state_effector(panel_1);
            target.add_state_effector(panel_2);
            truth.add_state_effector(panel_3);
            truth.add_state_effector(panel_4);
        }
        let mut slosh = None;
        if config.components.slosh {
            let rates = SLOSH_RATE_PERTURBATIONS_MPS.map(|rate| scale * rate);
            let (outputs, truth_outputs) = (particle_outputs(), particle_outputs());
            chaser.add_state_effector(slosh_tank(
                "tank1",
                Vector3::zeros(),
                rates,
                outputs.clone(),
            ));
            truth.add_state_effector(slosh_tank(
                "tank2",
                truth_offset_1,
                rates,
                truth_outputs.clone(),
            ));
            slosh = Some([outputs, truth_outputs]);
        }
        (panels, slosh)
    };

    // Hub states from the centers of mass, appendages at rest.
    add_appendages(0.0, &mut chaser, &mut target, &mut truth);
    set_center_of_mass_initial_state(&mut chaser, r_c1b1 + r_b1c + r_cn, Vector3::zeros());
    set_center_of_mass_initial_state(&mut target, r_c2b2 + r_b2c + r_cn, Vector3::zeros());
    set_center_of_mass_initial_state(&mut truth, r_cn, Vector3::zeros());
    for spacecraft in [&mut chaser, &mut target, &mut truth] {
        spacecraft.state_effectors.clear();
    }
    // Then upstream's perturbations, set after `InitializeSimulation`.
    let (panels, slosh) = add_appendages(1.0, &mut chaser, &mut target, &mut truth);
    if let Some(panels) = panels {
        panel_out = Some(panels);
    }
    if let Some([outputs, truth_outputs]) = slosh {
        rho_out = Some(outputs);
        rho_truth_out = Some(truth_outputs);
    }

    let mut group = SpacecraftGroup::new(vec![chaser, target]);
    group.integrator = Some(Box::new(upstream_rkf45()));
    group.add_coupling_effector(geometry.constraint(config.gain, config.gain));
    Servicing {
        group,
        truth,
        geometry,
        panel_out,
        rho_out,
        rho_truth_out,
    }
}
