//! Faceted solar radiation pressure from per-facet geometry messages: port of
//! upstream Basilisk `simulation/dynamics/facetedSRPEffector`.
//!
//! The newer of upstream's two faceted SRP effectors. Where
//! [`FacetSRPDynamicEffector`] holds the facet geometry and articulates it
//! itself, this effector reads each facet's body-frame geometry from a
//! [`FacetElementBodyMsg`] and its sunlit area from a [`ProjectedAreaMsg`]
//! (upstream's `facetedSpacecraftModel` and `facetedSpacecraftProjectedArea`
//! produce them), so it assumes nothing about whether a facet is rigid or
//! articulated.
//!
//! For each facet with a positive projected area `A_p`, `cos θ = A_p / A`, and
//! the facet force and the pressure at the spacecraft's distance from the Sun
//! are the ones [`FacetSRPDynamicEffector`] uses (shared with it), with the
//! current upstream solar flux at 1 AU, [`SOLAR_FLUX_EARTH`]. The force acts at
//! the facet's center of pressure, giving a torque about `B`. The summed force
//! and torque are scaled by the eclipse illumination factor, 1 when the
//! eclipse input is unconnected.
//!
//! Upstream raises when the Sun state or a facet message is not linked or not
//! written. Stand-in: connected ⇒ written, so an unconnected Sun or facet
//! input panics at evaluation, as does a Sun at the spacecraft's position,
//! where the Sun direction is undefined.
//!
//! Upstream publishes no message from this effector (nor from
//! [`FacetSRPDynamicEffector`]); scenarios log its `forceExternal_B` and
//! `torqueExternalPntB_B` members. Here the spacecraft publishes them, for
//! every dynamic effector whose name contains "srp" or "solar", in
//! [`SpacecraftDiagnosticsMsg::srp_force_body_n`] and
//! [`SpacecraftDiagnosticsMsg::srp_torque_body_nm`] (torque about `B`).
//!
//! [`FacetSRPDynamicEffector`]: super::facet_srp_dynamic_effector::FacetSRPDynamicEffector
//! [`SpacecraftDiagnosticsMsg::srp_force_body_n`]: crate::messages::SpacecraftDiagnosticsMsg::srp_force_body_n
//! [`SpacecraftDiagnosticsMsg::srp_torque_body_nm`]: crate::messages::SpacecraftDiagnosticsMsg::srp_torque_body_nm

use std::any::Any;

use nalgebra::Vector3;

use super::facet_srp_dynamic_effector::{facet_srp_force_body, srp_pressure_pa};
use crate::messages::{
    EclipseMsg, FacetElementBodyMsg, Input, ProjectedAreaMsg, SpacecraftStateMsg, SunEphemerisMsg,
};
use crate::spacecraft::{DynamicEffector, EffectorOutput};
use crate::utilities::astro_constants::SOLAR_FLUX_EARTH;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;

/// Below this Sun distance \[m\] no Sun direction can be resolved (upstream
/// `1e-12`).
const MIN_SUN_DISTANCE_M: f64 = 1e-12;

/// User-settable parameters of [`FacetedSrpEffector`].
#[derive(Clone, Debug, Default)]
pub struct FacetedSrpEffectorConfig {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `setNumFacets`: the number of facets, one geometry and one
    /// projected-area input each. Fixed at construction.
    pub num_facets: usize,
}

/// Faceted SRP force and torque: upstream `FacetedSRPEffector`.
#[derive(Clone, Debug)]
pub struct FacetedSrpEffector {
    /// User-settable parameters.
    pub config: FacetedSrpEffectorConfig,
    /// Upstream `facetElementBodyInMsgs`: one facet geometry per facet, body
    /// frame components.
    pub facet_element_body_in_msgs: Vec<Input<FacetElementBodyMsg>>,
    /// Upstream `facetProjectedAreaInMsgs`: each facet's area projected onto
    /// the Sun direction, at the index of its geometry.
    pub facet_projected_area_in_msgs: Vec<Input<ProjectedAreaMsg>>,
    /// Upstream `sunStateInMsg`: the Sun's inertial state (required).
    pub sun_state_in_msg: Input<SunEphemerisMsg>,
    /// Upstream `sunEclipseInMsg`: the Sun's illumination factor (optional).
    pub sun_eclipse_in_msg: Input<EclipseMsg>,
}

impl FacetedSrpEffector {
    /// Builds the effector with `config.num_facets` unconnected facet inputs.
    pub fn new(config: FacetedSrpEffectorConfig) -> Self {
        let num_facets = config.num_facets;
        Self {
            config,
            facet_element_body_in_msgs: (0..num_facets).map(|_| Input::default()).collect(),
            facet_projected_area_in_msgs: (0..num_facets).map(|_| Input::default()).collect(),
            sun_state_in_msg: Input::default(),
            sun_eclipse_in_msg: Input::default(),
        }
    }

    /// Upstream `validateConfiguration`: one geometry and one projected-area
    /// input per facet, and the Sun and every facet input connected.
    pub fn validate(&self) -> Result<(), String> {
        let num_facets = self.config.num_facets;
        if self.facet_element_body_in_msgs.len() != num_facets
            || self.facet_projected_area_in_msgs.len() != num_facets
        {
            return Err(format!(
                "facet_element_body_in_msgs and facet_projected_area_in_msgs must each contain \
                 num_facets = {num_facets} entries"
            ));
        }
        if !self.sun_state_in_msg.is_connected() {
            return Err("sun_state_in_msg is not connected".to_string());
        }
        for (index, (element, area)) in self
            .facet_element_body_in_msgs
            .iter()
            .zip(&self.facet_projected_area_in_msgs)
            .enumerate()
        {
            if !element.is_connected() {
                return Err(format!(
                    "facet_element_body_in_msgs[{index}] is not connected"
                ));
            }
            if !area.is_connected() {
                return Err(format!(
                    "facet_projected_area_in_msgs[{index}] is not connected"
                ));
            }
        }
        Ok(())
    }

    /// Upstream `computeForceTorque`: the SRP force \[N\] and torque about `B`
    /// \[N m\], both in body components, for the spacecraft `state`.
    ///
    /// # Panics
    ///
    /// If [`validate`](Self::validate) fails, or the Sun is at the spacecraft.
    pub fn force_torque_body(&self, state: &SpacecraftStateMsg) -> (Vector3<f64>, Vector3<f64>) {
        if let Err(message) = self.validate() {
            panic!("FacetedSrpEffector '{}': {message}", self.config.name);
        }
        let sun_position_m = self.sun_state_in_msg.read().sun_position_inertial_m;
        let sun_visibility_factor = if self.sun_eclipse_in_msg.is_connected() {
            self.sun_eclipse_in_msg.read().illumination_factor
        } else {
            1.0
        };

        let r_sb_body_m = mrp_to_dcm(state.sigma_bn) * (sun_position_m - state.position_m);
        let sun_distance_m = r_sb_body_m.norm();
        assert!(
            sun_distance_m > MIN_SUN_DISTANCE_M,
            "FacetedSrpEffector '{}': no unique body heading vector can be resolved",
            self.config.name
        );
        let sun_hat_body = r_sb_body_m / sun_distance_m;
        let srp_pressure = srp_pressure_pa(SOLAR_FLUX_EARTH, sun_distance_m);

        let (force_body_n, torque_body_nm) = self
            .facet_element_body_in_msgs
            .iter()
            .zip(&self.facet_projected_area_in_msgs)
            .map(|(element, area)| (element.read(), area.read().area_m2))
            .filter(|(_, projected_area_m2)| *projected_area_m2 > 0.0)
            .fold(
                (Vector3::zeros(), Vector3::zeros()),
                |(force_sum, torque_sum), (facet, projected_area_m2)| {
                    let force = facet_srp_force_body(
                        srp_pressure,
                        projected_area_m2,
                        projected_area_m2 / facet.area_m2,
                        sun_hat_body,
                        facet.normal_body,
                        facet.diffuse_coeff,
                        facet.specular_coeff,
                    );
                    (
                        force_sum + force,
                        torque_sum + facet.r_cop_b_body_m.cross(&force),
                    )
                },
            );
        (
            sun_visibility_factor * force_body_n,
            sun_visibility_factor * torque_body_nm,
        )
    }
}

impl DynamicEffector for FacetedSrpEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let (force_body_n, torque_body_nm) = self.force_torque_body(state);
        EffectorOutput {
            force_inertial_n: mrp_to_dcm(state.sigma_bn).transpose() * force_body_n,
            torque_body_nm,
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `facetedSRPEffector/_UnitTest/test_facetedSRPEffector.py`
    //! over its full parameter grid with its tolerance, and
    //! `test_facetedSRPEffectorValidation.py`. Upstream feeds the
    //! projected areas from `facetedSpacecraftProjectedArea`, which basilisk-rs
    //! does not have yet; a test module computes them with that module's
    //! formula, `A max(0, n̂ · ŝ)`, from the published spacecraft state.

    use hifitime::Epoch;
    use nalgebra::Matrix3;

    use super::*;
    use crate::dynamics::gravity::GravBodyData;
    use crate::messages::{Output, SpacecraftDiagnosticsMsg};
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};
    use crate::test_utils::MessageRecorder;
    use crate::utilities::astro_constants::{AU, MU_SUN, SPEED_LIGHT};
    use crate::{Module, SimulationContext};

    /// Upstream's cubic hub with two circular solar arrays: six square
    /// facets and four circular ones.
    fn facets() -> Vec<FacetElementBodyMsg> {
        let area_1 = 1.5 * 1.5;
        let area_2 = std::f64::consts::PI * (0.5 * 7.5) * (0.5 * 7.5);
        let r_cop = [
            [0.75, 0.0, 0.0],
            [0.0, 0.75, 0.0],
            [-0.75, 0.0, 0.0],
            [0.0, -0.75, 0.0],
            [0.0, 0.0, 0.75],
            [0.0, 0.0, -0.75],
            [4.5, 0.0, 0.75],
            [4.5, 0.0, 0.75],
            [-4.5, 0.0, 0.75],
            [-4.5, 0.0, 0.75],
        ];
        let normals = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
        ];
        (0..10)
            .map(|index| FacetElementBodyMsg {
                area_m2: if index < 6 { area_1 } else { area_2 },
                r_cop_b_body_m: Vector3::from(r_cop[index]),
                normal_body: Vector3::from(normals[index]),
                rotation_axis_body: Vector3::zeros(),
                diffuse_coeff: 0.1,
                specular_coeff: 0.9,
            })
            .collect()
    }

    /// Stand-in for upstream `facetedSpacecraftProjectedArea` with a Sun
    /// heading: each facet's area projected onto the Sun direction.
    struct ProjectedAreas {
        state_in: Input<SpacecraftStateMsg>,
        sun_in: Input<SunEphemerisMsg>,
        facets: Vec<FacetElementBodyMsg>,
        area_outs: Vec<Output<ProjectedAreaMsg>>,
    }

    impl Module for ProjectedAreas {
        fn init(&mut self) {}

        fn update(&mut self, _context: &SimulationContext) {
            let state = self.state_in.read();
            let r_sb_b = mrp_to_dcm(state.sigma_bn)
                * (self.sun_in.read().sun_position_inertial_m - state.position_m);
            let heading_b = r_sb_b / r_sb_b.norm();
            for (facet, area_out) in self.facets.iter().zip(&self.area_outs) {
                area_out.write(ProjectedAreaMsg {
                    area_m2: facet.area_m2 * facet.normal_body.dot(&heading_b).max(0.0),
                });
            }
        }
    }

    /// Upstream `compute_srp_force_torque` for one time step.
    fn truth_force_torque(
        facets: &[FacetElementBodyMsg],
        sigma_bn: Vector3<f64>,
        r_bn_n: Vector3<f64>,
        r_sn_n: Vector3<f64>,
        projected_areas: &[f64],
        illumination_factor: f64,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let dcm_bn = mrp_to_dcm(sigma_bn);
        let r_sb_b = dcm_bn * r_sn_n - dcm_bn * r_bn_n;
        let s_hat = r_sb_b / r_sb_b.norm();
        let num_au = AU * 1000.0 / r_sb_b.norm();
        let srp_pressure = (SOLAR_FLUX_EARTH / SPEED_LIGHT) * num_au * num_au;
        let mut force = Vector3::zeros();
        let mut torque = Vector3::zeros();
        for (facet, projected_area) in facets.iter().zip(projected_areas) {
            if *projected_area > 0.0 {
                let cos_theta = projected_area / facet.area_m2;
                let facet_force = -srp_pressure
                    * projected_area
                    * ((1.0 - facet.specular_coeff) * s_hat
                        + 2.0
                            * ((facet.diffuse_coeff / 3.0) + facet.specular_coeff * cos_theta)
                            * facet.normal_body);
                force += facet_force;
                torque += facet.r_cop_b_body_m.cross(&facet_force);
            }
        }
        (illumination_factor * force, illumination_factor * torque)
    }

    fn connected_effector(
        facets: &[FacetElementBodyMsg],
        facet_outs: &[Output<FacetElementBodyMsg>],
        area_outs: &[Output<ProjectedAreaMsg>],
        sun_out: &Output<SunEphemerisMsg>,
    ) -> FacetedSrpEffector {
        let mut effector = FacetedSrpEffector::new(FacetedSrpEffectorConfig {
            name: "facetedSRPEffector".to_string(),
            num_facets: facets.len(),
        });
        for (index, (facet_out, area_out)) in facet_outs.iter().zip(area_outs).enumerate() {
            facet_out.connect_to(&mut effector.facet_element_body_in_msgs[index]);
            area_out.connect_to(&mut effector.facet_projected_area_in_msgs[index]);
        }
        sun_out.connect_to(&mut effector.sun_state_in_msg);
        effector
    }

    /// Upstream `test_facetedSRPEffector` for one parameter set: 10 ms of
    /// flight at 1 ms, checking the force and torque at every step.
    fn run(
        r_bn_n_init: Vector3<f64>,
        r_sn_n: Vector3<f64>,
        sigma_bn_init: Vector3<f64>,
        omega_bn_b_init: Vector3<f64>,
        illumination_factor: f64,
    ) {
        let step_nanos = 1_000_000;
        let facets = facets();
        let facet_outs: Vec<_> = facets.iter().cloned().map(Output::new).collect();
        let area_outs: Vec<Output<ProjectedAreaMsg>> =
            facets.iter().map(|_| Output::default()).collect();
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: r_sn_n,
            sun_velocity_inertial_mps: Vector3::zeros(),
        });
        let eclipse_out = Output::new(EclipseMsg {
            illumination_factor,
        });
        let mut effector = connected_effector(&facets, &facet_outs, &area_outs, &sun_out);
        eclipse_out.connect_to(&mut effector.sun_eclipse_in_msg);

        // Upstream gives the initial center of mass position, here B's.
        let center_of_mass_body_m = Vector3::new(0.0, 0.0, 1.0);
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 750.0,
            hub_center_of_mass_body_m: center_of_mass_body_m,
            inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
            integration_step_nanos: step_nanos,
            initial_position_m: r_bn_n_init
                - mrp_to_dcm(sigma_bn_init).transpose() * center_of_mass_body_m,
            initial_velocity_mps: Vector3::new(
                -5199.77710904224,
                -3436.681645356935,
                1041.576797498721,
            ),
            initial_sigma_bn: sigma_bn_init,
            initial_omega_radps: omega_bn_b_init,
            integrator: None,
        });
        spacecraft
            .add_grav_body(
                GravBodyData::point_mass("sun", MU_SUN * 1e9, true, r_sn_n, Vector3::zeros())
                    .unwrap(),
            )
            .unwrap();
        let mut projected_areas = ProjectedAreas {
            state_in: Input::default(),
            sun_in: Input::default(),
            facets: facets.clone(),
            area_outs: area_outs.clone(),
        };
        spacecraft
            .state_out
            .connect_to(&mut projected_areas.state_in);
        sun_out.connect_to(&mut projected_areas.sun_in);
        let mut state_log = MessageRecorder::<SpacecraftStateMsg>::default();
        let mut diagnostics_log = MessageRecorder::<SpacecraftDiagnosticsMsg>::default();
        let mut area_logs: Vec<MessageRecorder<ProjectedAreaMsg>> =
            facets.iter().map(|_| MessageRecorder::default()).collect();
        spacecraft.state_out.connect_to(&mut state_log.input_msg);
        spacecraft
            .diagnostics_out
            .connect_to(&mut diagnostics_log.input_msg);
        for (area_out, area_log) in area_outs.iter().zip(&mut area_logs) {
            area_out.connect_to(&mut area_log.input_msg);
        }
        spacecraft.add_dynamic_effector(effector);

        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module(
                "facetedSpacecraftProjectedArea",
                &mut projected_areas,
                step_nanos,
                20,
            );
            sim.add_module("scObject", &mut spacecraft, step_nanos, 10);
            sim.add_module("stateLog", &mut state_log, step_nanos, 0);
            sim.add_module("diagnosticsLog", &mut diagnostics_log, step_nanos, 0);
            for (index, area_log) in area_logs.iter_mut().enumerate() {
                sim.add_module(format!("areaLog{index}"), area_log, step_nanos, 0);
            }
            sim.run_for(10_000_000);
        }

        assert_eq!(state_log.samples.len(), 11);
        let atol = 1e-7;
        for (step, (state, diagnostics)) in state_log
            .samples
            .iter()
            .zip(&diagnostics_log.samples)
            .enumerate()
        {
            let projected: Vec<f64> = area_logs
                .iter()
                .map(|log| log.samples[step].area_m2)
                .collect();
            let (force_truth, torque_truth) = truth_force_torque(
                &facets,
                state.sigma_bn,
                state.position_m,
                r_sn_n,
                &projected,
                illumination_factor,
            );
            assert_eq!(force_truth == Vector3::zeros(), illumination_factor == 0.0);
            for (actual, truth, what) in [
                (diagnostics.srp_force_body_n, force_truth, "force"),
                (diagnostics.srp_torque_body_nm, torque_truth, "torque"),
            ] {
                assert!(
                    (actual - truth).abs().max() <= atol,
                    "{what} at step {step} (r = {r_bn_n_init:?}, sun = {r_sn_n:?}, \
                     sigma = {sigma_bn_init:?}, omega = {omega_bn_b_init:?}, \
                     illumination = {illumination_factor}): {actual:?} vs {truth:?}"
                );
            }
        }
    }

    /// Upstream `test_facetedSRPEffector` over its parameter grid.
    #[test]
    fn force_and_torque_match_the_truth_over_the_parameter_grid() {
        let positions = [
            (
                Vector3::new(-4020338.690396649, 7490566.741852513, 5248299.211589362),
                Vector3::zeros(),
            ),
            (
                Vector3::new(7490566.741852513, 5248299.211589362, -4020338.690396649),
                Vector3::zeros(),
            ),
            (
                Vector3::new(-4020338.690396649, 7490566.741852513, 5248299.211589362),
                Vector3::new(1.0e11, 2.0e10, -3.0e10),
            ),
            (Vector3::new(2.0e11, 0.0, 0.0), Vector3::zeros()),
        ];
        let attitudes = [
            Vector3::zeros(),
            Vector3::new(0.41421356, 0.0, 0.0),
            Vector3::new(0.0, 0.41421356, 0.0),
            Vector3::new(0.1, -0.2, 0.05),
        ];
        let rates = [
            Vector3::zeros(),
            Vector3::new(0.05, 0.0, 0.0),
            Vector3::new(0.0, 0.05, 0.0),
            Vector3::new(0.0, 0.0, 0.05),
            Vector3::new(0.05, -0.05, -0.05),
        ];
        for (r_bn_n, r_sn_n) in positions {
            for sigma_bn in attitudes {
                for omega in rates {
                    for illumination_factor in [1.0, 0.5, 0.0] {
                        // Upstream's spacecraft position is relative to the
                        // central Sun.
                        run(
                            r_sn_n + r_bn_n,
                            r_sn_n,
                            sigma_bn,
                            omega,
                            illumination_factor,
                        );
                    }
                }
            }
        }
    }

    /// The effector's force, rotated to the inertial frame, and its torque
    /// are what it reports in body components; a facet facing away from the
    /// Sun and an unconnected eclipse input contribute nothing and full sun.
    #[test]
    fn ignores_unlit_facets_and_defaults_to_full_sun() {
        let mut facets = facets();
        facets.truncate(2);
        let facet_outs: Vec<_> = facets.iter().cloned().map(Output::new).collect();
        let area_outs = [
            Output::new(ProjectedAreaMsg { area_m2: 1.5 }),
            Output::new(ProjectedAreaMsg { area_m2: 0.0 }),
        ];
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::new(AU * 1000.0, 0.0, 0.0),
            sun_velocity_inertial_mps: Vector3::zeros(),
        });
        let effector = connected_effector(&facets, &facet_outs, &area_outs, &sun_out);
        let state = SpacecraftStateMsg {
            sigma_bn: Vector3::new(0.1, -0.2, 0.05),
            ..Default::default()
        };

        let (force_b, torque_b) = effector.force_torque_body(&state);
        let (force_truth, torque_truth) = truth_force_torque(
            &facets,
            state.sigma_bn,
            state.position_m,
            sun_out.read().sun_position_inertial_m,
            &[1.5, 0.0],
            1.0,
        );
        assert!((force_b - force_truth).norm() <= 1e-15 * force_truth.norm());
        assert!((torque_b - torque_truth).norm() <= 1e-15 * torque_truth.norm());
        let output = effector.compute_output(&state);
        let force_inertial = mrp_to_dcm(state.sigma_bn).transpose() * force_b;
        assert_eq!(output.force_inertial_n, force_inertial);
        assert_eq!(output.torque_body_nm, torque_b);
    }

    /// A one-facet spacecraft with every input connected except `missing`.
    fn one_facet_spacecraft(missing: Option<&str>) -> Spacecraft {
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::new(AU * 1000.0, 0.0, 0.0),
            sun_velocity_inertial_mps: Vector3::zeros(),
        });
        let facet_out = Output::new(FacetElementBodyMsg {
            area_m2: 1.0,
            normal_body: Vector3::x(),
            ..Default::default()
        });
        let area_out = Output::new(ProjectedAreaMsg { area_m2: 1.0 });
        let mut effector = FacetedSrpEffector::new(FacetedSrpEffectorConfig {
            name: "srp".to_string(),
            num_facets: 1,
        });
        if missing != Some("sun") {
            sun_out.connect_to(&mut effector.sun_state_in_msg);
        }
        if missing != Some("facet") {
            facet_out.connect_to(&mut effector.facet_element_body_in_msgs[0]);
        }
        if missing != Some("area") {
            area_out.connect_to(&mut effector.facet_projected_area_in_msgs[0]);
        }
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 100.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(10.0, 20.0, 30.0)),
            integration_step_nanos: 100_000_000,
            initial_position_m: Vector3::zeros(),
            initial_velocity_mps: Vector3::zeros(),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: None,
        });
        spacecraft.add_dynamic_effector(effector);
        spacecraft
    }

    /// Upstream `test_facetedSRPEffector_validation`: initializing the
    /// spacecraft, which evaluates its effectors, rejects every missing
    /// required input.
    #[test]
    fn rejects_every_missing_required_input() {
        for (missing, expected) in [
            ("sun", "sun_state_in_msg"),
            ("facet", "facet_element_body_in_msgs[0]"),
            ("area", "facet_projected_area_in_msgs[0]"),
        ] {
            let mut spacecraft = one_facet_spacecraft(Some(missing));
            let panic =
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| spacecraft.init()))
                    .expect_err("a missing input must be rejected");
            let message = panic.downcast_ref::<String>().unwrap();
            assert!(message.contains(expected), "{message}");
        }
    }

    /// Upstream `test_facetedSRPEffector_spacecraftOnlyComputesForce`: a fully
    /// absorbing unit facet facing the Sun at 1 AU feels the solar pressure.
    #[test]
    fn an_attached_effector_computes_the_force() {
        let mut spacecraft = one_facet_spacecraft(None);
        spacecraft.init();
        let force = spacecraft.diagnostics_out.read().srp_force_body_n;
        let expected = Vector3::new(-SOLAR_FLUX_EARTH / SPEED_LIGHT, 0.0, 0.0);
        for (actual, truth) in force.iter().zip(expected.iter()) {
            assert!(
                (actual - truth).abs() <= 1e-15 + 1e-12 * truth.abs(),
                "{force:?}"
            );
        }
    }

    /// The facet vectors must match the facet count.
    #[test]
    fn requires_inputs_sized_to_the_facet_count() {
        let mut short = FacetedSrpEffector::new(FacetedSrpEffectorConfig {
            name: "srp".to_string(),
            num_facets: 2,
        });
        short.facet_projected_area_in_msgs.pop();
        assert!(short.validate().unwrap_err().contains("num_facets"));
    }

    #[test]
    #[should_panic(expected = "no unique body heading")]
    fn rejects_a_sun_at_the_spacecraft() {
        let facets = facets();
        let facet_outs: Vec<_> = facets.iter().cloned().map(Output::new).collect();
        let area_outs: Vec<Output<ProjectedAreaMsg>> =
            facets.iter().map(|_| Output::default()).collect();
        let sun_out = Output::<SunEphemerisMsg>::default();
        let effector = connected_effector(&facets, &facet_outs, &area_outs, &sun_out);
        effector.force_torque_body(&SpacecraftStateMsg::default());
    }
}
