//! Projected areas of a faceted spacecraft: port of upstream Basilisk
//! `simulation/deviceInterface/facetedSpacecraftProjectedArea`.
//!
//! For a heading `ĥ` in the hub frame, each facet's area projected onto it
//! is `A max(0, n̂_B · ĥ)` ([`projected_area_m2`]): its full area facing the
//! heading, nothing facing away. The module publishes the projected area of
//! every facet, their total, and the total surface area. The facets come in
//! hub-frame components as [`FacetElementBodyMsg`]s, for example from
//! [`FacetedSpacecraftModel`](super::faceted_spacecraft_model::FacetedSpacecraftModel),
//! and the per-facet areas are the sunlit-area inputs of
//! [`FacetedSrpEffector`](crate::dynamics::faceted_srp_effector::FacetedSrpEffector).
//!
//! The heading comes from whichever inputs are connected ([`HeadingSource`]):
//!
//! * the spacecraft state and the Sun state: the direction to the Sun, for
//!   sunlit areas (SRP);
//! * the spacecraft state alone: the direction of the inertial velocity, for
//!   the area exposed to drag;
//! * a [`BodyHeadingMsg`] alone: any heading, used as given.
//!
//! Upstream raises on an inconsistent set of connected headings, and on a
//! zero Sun direction or velocity; here each is reported and the heading is
//! zero, so every projected area is zero. Upstream also raises when an input
//! is linked but not written. Stand-in: connected ⇒ written; an unconnected
//! facet input is reported and keeps its last geometry (none before the
//! first read, as upstream's reset leaves it).

use nalgebra::Vector3;

use crate::messages::{
    BodyHeadingMsg, FacetElementBodyMsg, Input, Output, ProjectedAreaMsg, SpacecraftStateMsg,
    SunEphemerisMsg,
};
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Below this norm \[m\] or \[m/s\] no heading can be resolved (upstream
/// `1e-12`).
const MIN_HEADING_NORM: f64 = 1e-12;

/// Area \[m^2\] of a facet of `area_m2` with unit normal `normal_body`
/// projected onto the unit heading `heading_body`: `A max(0, n̂ · ĥ)`.
pub fn projected_area_m2(
    area_m2: f64,
    normal_body: &Vector3<f64>,
    heading_body: &Vector3<f64>,
) -> f64 {
    area_m2 * normal_body.dot(heading_body).max(0.0)
}

/// The heading the areas are projected onto, chosen by the connected inputs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HeadingSource {
    /// Spacecraft and Sun states: the direction to the Sun.
    Sun,
    /// Spacecraft state alone: the direction of the inertial velocity.
    Velocity,
    /// Body heading alone: the given heading.
    Body,
}

/// User-settable parameters of [`FacetedSpacecraftProjectedArea`].
#[derive(Clone, Debug)]
pub struct FacetedSpacecraftProjectedAreaConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `setNumFacets`: number of facets, one geometry input and
    /// one projected-area output each.
    pub num_facets: usize,
}

/// Facet projected areas for a heading: upstream
/// `FacetedSpacecraftProjectedArea`.
#[derive(Clone, Debug)]
pub struct FacetedSpacecraftProjectedArea {
    /// Module configuration.
    pub config: FacetedSpacecraftProjectedAreaConfig,
    /// Upstream `spacecraftStateInMsg`: attitude, position and velocity of
    /// the hub.
    pub spacecraft_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `sunStateInMsg`: inertial position of the Sun.
    pub sun_state_in_msg: Input<SunEphemerisMsg>,
    /// Upstream `bodyHeadingInMsg`: a hub-frame heading.
    pub body_heading_in_msg: Input<BodyHeadingMsg>,
    /// Upstream `facetElementBodyInMsgs`: each facet, hub-frame components.
    pub facet_element_body_in_msgs: Vec<Input<FacetElementBodyMsg>>,
    /// Upstream `facetProjectedAreaOutMsgs`: each facet's projected area.
    pub facet_projected_area_out_msgs: Vec<Output<ProjectedAreaMsg>>,
    /// Upstream `totalProjectedAreaOutMsg`: sum of the projected areas.
    pub total_projected_area_out_msg: Output<ProjectedAreaMsg>,
    /// Upstream `surfaceAreaOutMsg`: sum of the facet areas.
    pub surface_area_out_msg: Output<ProjectedAreaMsg>,
    facet_areas_m2: Vec<f64>,
    facet_normals_body: Vec<Vector3<f64>>,
    facet_projected_areas_m2: Vec<f64>,
    total_projected_area_m2: f64,
    surface_area_m2: f64,
}

impl FacetedSpacecraftProjectedArea {
    /// Builds the module with `config.num_facets` geometry inputs and
    /// projected-area outputs.
    pub fn new(config: FacetedSpacecraftProjectedAreaConfig) -> Self {
        let num_facets = config.num_facets;
        // Not `vec![_; n]`: cloning a port aliases it.
        Self {
            config,
            spacecraft_state_in_msg: Input::default(),
            sun_state_in_msg: Input::default(),
            body_heading_in_msg: Input::default(),
            facet_element_body_in_msgs: (0..num_facets).map(|_| Input::default()).collect(),
            facet_projected_area_out_msgs: (0..num_facets).map(|_| Output::default()).collect(),
            total_projected_area_out_msg: Output::default(),
            surface_area_out_msg: Output::default(),
            facet_areas_m2: vec![0.0; num_facets],
            facet_normals_body: vec![Vector3::zeros(); num_facets],
            facet_projected_areas_m2: vec![0.0; num_facets],
            total_projected_area_m2: 0.0,
            surface_area_m2: 0.0,
        }
    }

    /// The heading the connected inputs select, as upstream's update picks
    /// it: the spacecraft state wins over a body heading.
    pub fn heading_source(&self) -> Option<HeadingSource> {
        if self.spacecraft_state_in_msg.is_connected() {
            Some(if self.sun_state_in_msg.is_connected() {
                HeadingSource::Sun
            } else {
                HeadingSource::Velocity
            })
        } else if self.body_heading_in_msg.is_connected() {
            Some(HeadingSource::Body)
        } else {
            None
        }
    }

    /// Upstream `Reset`'s checks on the connected inputs.
    fn check_connections(&self) {
        let name = &self.config.name;
        let spacecraft = self.spacecraft_state_in_msg.is_connected();
        let sun = self.sun_state_in_msg.is_connected();
        let body = self.body_heading_in_msg.is_connected();
        if body && (spacecraft || sun) {
            log::error!(
                "facetedSpacecraftProjectedArea '{name}': body_heading_in_msg cannot be used with \
                 spacecraft_state_in_msg and/or sun_state_in_msg"
            );
        }
        if sun && !spacecraft {
            log::error!(
                "facetedSpacecraftProjectedArea '{name}': sun_state_in_msg requires \
                 spacecraft_state_in_msg to be connected"
            );
        }
        if !(spacecraft || sun || body) {
            log::error!("facetedSpacecraftProjectedArea '{name}': no heading source is connected");
        }
        for (index, input) in self.facet_element_body_in_msgs.iter().enumerate() {
            if !input.is_connected() {
                log::error!(
                    "facetedSpacecraftProjectedArea '{name}' facet_element_body_in_msgs[{index}] \
                     is not connected"
                );
            }
        }
    }

    /// Upstream `Reset`: forgets the facet geometry and publishes.
    fn reset_module(&mut self) {
        self.check_connections();
        let num_facets = self.config.num_facets;
        self.facet_areas_m2 = vec![0.0; num_facets];
        self.facet_normals_body = vec![Vector3::zeros(); num_facets];
        self.facet_projected_areas_m2 = vec![0.0; num_facets];
        self.write_outputs();
    }

    /// Upstream `readInputMessages`' heading: unit, hub-frame components,
    /// or zero when none can be resolved.
    fn heading_body(&self) -> Vector3<f64> {
        let name = &self.config.name;
        let unit_or_zero = |vector: Vector3<f64>| {
            let norm = vector.norm();
            if norm > MIN_HEADING_NORM {
                vector / norm
            } else {
                log::error!(
                    "facetedSpacecraftProjectedArea '{name}': no unique body heading vector can \
                     be resolved"
                );
                Vector3::zeros()
            }
        };
        match self.heading_source() {
            Some(HeadingSource::Sun) => {
                let state = self.spacecraft_state_in_msg.read();
                let r_sn_n = self.sun_state_in_msg.read().sun_position_inertial_m;
                unit_or_zero(mrp_to_dcm(state.sigma_bn) * (r_sn_n - state.position_m))
            }
            Some(HeadingSource::Velocity) => {
                let state = self.spacecraft_state_in_msg.read();
                unit_or_zero(mrp_to_dcm(state.sigma_bn) * state.velocity_mps)
            }
            Some(HeadingSource::Body) => self.body_heading_in_msg.read().r_hat_xb_b,
            None => {
                log::error!(
                    "facetedSpacecraftProjectedArea '{name}': no heading source is connected"
                );
                Vector3::zeros()
            }
        }
    }

    /// Upstream `readInputMessages`' facet geometry.
    fn read_facets(&mut self) {
        for (index, input) in self.facet_element_body_in_msgs.iter().enumerate() {
            if input.is_connected() {
                let facet = input.read();
                self.facet_areas_m2[index] = facet.area_m2;
                self.facet_normals_body[index] = facet.normal_body;
            } else {
                log::error!(
                    "facetedSpacecraftProjectedArea '{}' facet_element_body_in_msgs[{index}] is \
                     not connected",
                    self.config.name
                );
            }
        }
    }

    /// Upstream `writeOutputMessages`.
    fn write_outputs(&self) {
        self.total_projected_area_out_msg.write(ProjectedAreaMsg {
            area_m2: self.total_projected_area_m2,
        });
        self.surface_area_out_msg.write(ProjectedAreaMsg {
            area_m2: self.surface_area_m2,
        });
        for (output, &area_m2) in self
            .facet_projected_area_out_msgs
            .iter()
            .zip(&self.facet_projected_areas_m2)
        {
            output.write(ProjectedAreaMsg { area_m2 });
        }
    }
}

impl Module for FacetedSpacecraftProjectedArea {
    fn init(&mut self) {
        self.reset_module();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let heading_body = self.heading_body();
        self.read_facets();
        for ((projected, area), normal) in self
            .facet_projected_areas_m2
            .iter_mut()
            .zip(&self.facet_areas_m2)
            .zip(&self.facet_normals_body)
        {
            *projected = projected_area_m2(*area, normal, &heading_body);
        }
        self.total_projected_area_m2 = self.facet_projected_areas_m2.iter().sum();
        self.surface_area_m2 = self.facet_areas_m2.iter().sum();
        self.write_outputs();
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream
    //! `facetedSpacecraftProjectedArea/_UnitTest/test_facetedSpacecraftProjectedArea.py`
    //! for its three headings, with its truth and tolerance. Adds checks that
    //! [`FacetedSpacecraftModel`] and this module feed
    //! [`FacetedSrpEffector`] the force [`FacetSRPDynamicEffector`] computes
    //! from the same facets and articulation angle, and give
    //! [`FacetDragDynamicEffector`]'s exposed area.

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{
        FacetedSpacecraftProjectedArea, FacetedSpacecraftProjectedAreaConfig, HeadingSource,
    };
    use crate::device_interface::faceted_spacecraft_model::{
        FacetedSpacecraftModel, FacetedSpacecraftModelConfig,
    };
    use crate::dynamics::facet_drag_dynamic_effector::{
        FacetDragDynamicEffector, FacetDragDynamicEffectorConfig,
    };
    use crate::dynamics::facet_srp_dynamic_effector::{
        FacetSRPDynamicEffector, FacetSRPDynamicEffectorConfig,
    };
    use crate::dynamics::faceted_srp_effector::{FacetedSrpEffector, FacetedSrpEffectorConfig};
    use crate::messages::{
        AtmosphereMsg, BodyHeadingMsg, FacetElementBodyMsg, FacetElementMsg, HingedRigidBodyMsg,
        Output, SpacecraftStateMsg, SunEphemerisMsg,
    };
    use crate::spacecraft::DynamicEffector;
    use crate::utilities::astro_constants::{AU, SOLAR_FLUX_EARTH};
    use crate::utilities::rigid_body_kinematics::{mrp_to_dcm, prv_to_dcm, prv_to_mrp};
    use crate::{Module, SimulationContext};

    const D2R: f64 = std::f64::consts::PI / 180.0;
    const STEP_NANOS: u64 = 1_000_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream's two facets in hub-frame components.
    fn facets() -> [FacetElementBodyMsg; 2] {
        [
            FacetElementBodyMsg {
                area_m2: 0.5,
                r_cop_b_body_m: Vector3::new(-0.1, 0.1, -0.1),
                normal_body: Vector3::new(1.0, 0.0, 0.0),
                rotation_axis_body: Vector3::new(0.0, 1.0, 0.0),
                diffuse_coeff: 0.1,
                specular_coeff: 0.9,
            },
            FacetElementBodyMsg {
                area_m2: 1.0,
                r_cop_b_body_m: Vector3::new(0.1, -0.1, -0.1),
                normal_body: Vector3::new(0.0, 1.0, 0.0),
                rotation_axis_body: Vector3::new(1.0, 0.0, 0.0),
                diffuse_coeff: 0.1,
                specular_coeff: 0.9,
            },
        ]
    }

    fn spacecraft_state() -> SpacecraftStateMsg {
        SpacecraftStateMsg {
            position_m: Vector3::new(-4020338.690396649, 7490566.741852513, 5248299.211589362),
            velocity_mps: Vector3::new(-5199.77710904224, -3436.681645356935, 1041.576797498721),
            sigma_bn: prv_to_mrp(10.0 * D2R * Vector3::x()),
            ..Default::default()
        }
    }

    /// Upstream `compute_facet_projected_area`'s heading for `source`.
    fn truth_heading(source: HeadingSource) -> Vector3<f64> {
        let state = spacecraft_state();
        let dcm_bn = mrp_to_dcm(state.sigma_bn);
        let unit = |vector: Vector3<f64>| {
            if vector.norm() > 1e-12 {
                vector / vector.norm()
            } else {
                Vector3::zeros()
            }
        };
        match source {
            HeadingSource::Body => Vector3::x(),
            HeadingSource::Velocity => unit(dcm_bn * state.velocity_mps),
            HeadingSource::Sun => {
                unit(dcm_bn * (Vector3::new(1.0, -10.0, 50.0) - state.position_m))
            }
        }
    }

    /// Upstream `test_facetedSpacecraftProjectedArea` for one heading: the
    /// per-facet, total and surface areas at every step of 2 s.
    #[test]
    fn projected_areas_match_the_truth_for_every_heading() {
        for source in [
            HeadingSource::Body,
            HeadingSource::Sun,
            HeadingSource::Velocity,
        ] {
            let facets = facets();
            let facet_outs: Vec<_> = facets.iter().cloned().map(Output::new).collect();
            let mut module =
                FacetedSpacecraftProjectedArea::new(FacetedSpacecraftProjectedAreaConfig {
                    name: "facetedSpacecraftProjectedArea".to_string(),
                    num_facets: 2,
                });
            for (output, input) in facet_outs
                .iter()
                .zip(&mut module.facet_element_body_in_msgs)
            {
                output.connect_to(input);
            }
            let heading_out = Output::new(BodyHeadingMsg {
                r_hat_xb_b: Vector3::x(),
            });
            let state_out = Output::new(spacecraft_state());
            let sun_out = Output::new(SunEphemerisMsg {
                sun_position_inertial_m: Vector3::new(1.0, -10.0, 50.0),
                sun_velocity_inertial_mps: Vector3::zeros(),
            });
            match source {
                HeadingSource::Body => heading_out.connect_to(&mut module.body_heading_in_msg),
                HeadingSource::Velocity => {
                    state_out.connect_to(&mut module.spacecraft_state_in_msg);
                }
                HeadingSource::Sun => {
                    state_out.connect_to(&mut module.spacecraft_state_in_msg);
                    sun_out.connect_to(&mut module.sun_state_in_msg);
                }
            }
            assert_eq!(module.heading_source(), Some(source));
            module.init();

            let heading = truth_heading(source);
            let truth: Vec<f64> = facets
                .iter()
                .map(|facet| facet.area_m2 * facet.normal_body.dot(&heading).max(0.0))
                .collect();
            let total_truth: f64 = truth.iter().sum();
            let surface_truth: f64 = facets.iter().map(|facet| facet.area_m2).sum();
            let accuracy = 1e-12;
            let close = |actual: f64, desired: f64| {
                (actual - desired).abs() <= accuracy + 1e-7 * desired.abs()
            };
            for second in 0..=2 {
                module.update(&context(second * STEP_NANOS));
                for (output, truth) in module.facet_projected_area_out_msgs.iter().zip(&truth) {
                    let area = output.read().area_m2;
                    assert!(close(area, *truth), "{source:?}: {area} vs {truth}");
                }
                let total = module.total_projected_area_out_msg.read().area_m2;
                assert!(close(total, total_truth), "{source:?}: total {total}");
                let surface = module.surface_area_out_msg.read().area_m2;
                assert!(
                    close(surface, surface_truth),
                    "{source:?}: surface {surface}"
                );
            }
        }
    }

    #[test]
    fn a_degenerate_heading_projects_nothing() {
        let facet_out = Output::new(facets()[0].clone());
        let state_out = Output::new(SpacecraftStateMsg::default());
        let mut module =
            FacetedSpacecraftProjectedArea::new(FacetedSpacecraftProjectedAreaConfig {
                name: "facetedSpacecraftProjectedArea".to_string(),
                num_facets: 1,
            });
        facet_out.connect_to(&mut module.facet_element_body_in_msgs[0]);
        state_out.connect_to(&mut module.spacecraft_state_in_msg);
        module.init();
        module.update(&context(0));
        assert_eq!(module.facet_projected_area_out_msgs[0].read().area_m2, 0.0);
        assert_eq!(module.surface_area_out_msg.read().area_m2, 0.5);
    }

    /// A hub with a solar array: facet 0 articulates about `F` `z`, its
    /// center of pressure on the hinge; facets 1 and 2 are fixed.
    fn articulated_spacecraft() -> Vec<FacetElementMsg> {
        vec![
            FacetElementMsg {
                area_m2: 4.0,
                r_cop_f_facet_m: Vector3::zeros(),
                normal_facet: Vector3::x(),
                rotation_axis_facet: Vector3::z(),
                dcm_facet0_body: prv_to_dcm(30.0 * D2R * Vector3::y()),
                r_fb_body_m: Vector3::new(0.0, 2.0, 0.0),
                diffuse_coeff: 0.2,
                specular_coeff: 0.6,
            },
            FacetElementMsg {
                area_m2: 1.5,
                r_cop_f_facet_m: Vector3::new(0.0, 0.1, 0.2),
                normal_facet: Vector3::x(),
                rotation_axis_facet: Vector3::zeros(),
                dcm_facet0_body: Matrix3::identity(),
                r_fb_body_m: Vector3::new(0.75, 0.0, 0.0),
                diffuse_coeff: 0.1,
                specular_coeff: 0.9,
            },
            FacetElementMsg {
                area_m2: 1.5,
                r_cop_f_facet_m: Vector3::new(0.0, -0.1, 0.0),
                normal_facet: Vector3::y(),
                rotation_axis_facet: Vector3::zeros(),
                dcm_facet0_body: prv_to_dcm(-20.0 * D2R * Vector3::z()),
                r_fb_body_m: Vector3::new(0.0, 0.75, 0.0),
                diffuse_coeff: 0.3,
                specular_coeff: 0.5,
            },
        ]
    }

    /// The faceted model, the sunlit areas and [`FacetedSrpEffector`] give
    /// [`FacetSRPDynamicEffector`]'s force and torque for the same facets
    /// and articulation angle, up to the two effectors' solar flux at 1 AU.
    #[test]
    fn model_and_sunlit_areas_feed_the_faceted_srp_effector() {
        let facets = articulated_spacecraft();
        let facet_outs: Vec<_> = facets.iter().cloned().map(Output::new).collect();
        let angle_out = Output::new(HingedRigidBodyMsg {
            theta_rad: 25.0 * D2R,
            theta_dot_radps: 0.0,
        });
        let state = SpacecraftStateMsg {
            position_m: Vector3::new(0.9 * AU * 1000.0, 0.2 * AU * 1000.0, 0.0),
            sigma_bn: prv_to_mrp(Vector3::new(0.3, -0.2, 2.5)),
            ..Default::default()
        };
        let state_out = Output::new(state.clone());
        let sun_out = Output::new(SunEphemerisMsg::default());

        let mut model = FacetedSpacecraftModel::new(FacetedSpacecraftModelConfig {
            name: "facetedSCModel".to_string(),
            num_facets: facets.len(),
            num_articulated_facets: 1,
        });
        angle_out.connect_to(&mut model.articulated_facet_data_in_msgs[0]);
        for (output, input) in facet_outs.iter().zip(&mut model.facet_element_in_msgs) {
            output.connect_to(input);
        }
        let mut areas = FacetedSpacecraftProjectedArea::new(FacetedSpacecraftProjectedAreaConfig {
            name: "sunlitAreas".to_string(),
            num_facets: facets.len(),
        });
        state_out.connect_to(&mut areas.spacecraft_state_in_msg);
        sun_out.connect_to(&mut areas.sun_state_in_msg);
        let mut srp = FacetedSrpEffector::new(FacetedSrpEffectorConfig {
            name: "facetedSRP".to_string(),
            num_facets: facets.len(),
        });
        sun_out.connect_to(&mut srp.sun_state_in_msg);
        for index in 0..facets.len() {
            let body_out = &model.facet_element_body_out_msgs[index];
            body_out.connect_to(&mut areas.facet_element_body_in_msgs[index]);
            body_out.connect_to(&mut srp.facet_element_body_in_msgs[index]);
            areas.facet_projected_area_out_msgs[index]
                .connect_to(&mut srp.facet_projected_area_in_msgs[index]);
        }

        let mut reference = FacetSRPDynamicEffector::new(FacetSRPDynamicEffectorConfig {
            name: "facetSRP".to_string(),
        });
        sun_out.connect_to(&mut reference.input_sun_msg);
        for facet in &facets {
            let index = reference.add_facet(
                facet.area_m2,
                facet.dcm_facet0_body,
                facet.normal_facet,
                facet.rotation_axis_facet,
                facet.dcm_facet0_body.transpose() * facet.r_cop_f_facet_m + facet.r_fb_body_m,
                facet.diffuse_coeff,
                facet.specular_coeff,
            );
            if index == 0 {
                angle_out.connect_to(&mut reference.facets[index].articulation_angle_in);
            }
        }

        model.init();
        areas.init();
        model.update(&context(0));
        areas.update(&context(0));
        assert!(
            areas
                .facet_projected_area_out_msgs
                .iter()
                .all(|output| output.read().area_m2 > 0.0),
            "every facet should be sunlit"
        );

        let flux_ratio = 1368.0 / SOLAR_FLUX_EARTH;
        let faceted = DynamicEffector::compute_output(&srp, &state);
        let expected = reference.compute_output(&state);
        let force_error =
            (flux_ratio * faceted.force_inertial_n - expected.force_inertial_n).norm();
        let torque_error = (flux_ratio * faceted.torque_body_nm - expected.torque_body_nm).norm();
        assert!(
            force_error <= 1e-12 * expected.force_inertial_n.norm(),
            "{:?} vs {:?}",
            faceted.force_inertial_n,
            expected.force_inertial_n
        );
        assert!(
            torque_error <= 1e-12 * expected.torque_body_nm.norm(),
            "{:?} vs {:?}",
            faceted.torque_body_nm,
            expected.torque_body_nm
        );
    }

    /// The velocity heading's total is the area [`FacetDragDynamicEffector`]
    /// exposes to the flow: its force is `½ ρ C_d v² A_exposed`.
    #[test]
    fn velocity_heading_gives_the_area_exposed_to_drag() {
        let facets = articulated_spacecraft();
        let facet_outs: Vec<_> = facets.iter().cloned().map(Output::new).collect();
        let angle_out = Output::new(HingedRigidBodyMsg {
            theta_rad: -40.0 * D2R,
            theta_dot_radps: 0.0,
        });
        let state = SpacecraftStateMsg {
            position_m: Vector3::new(6_778_000.0, 0.0, 0.0),
            velocity_mps: Vector3::new(-1500.0, 7400.0, 900.0),
            sigma_bn: prv_to_mrp(Vector3::new(-0.4, 1.1, 0.2)),
            ..Default::default()
        };
        let state_out = Output::new(state.clone());
        let mut model = FacetedSpacecraftModel::new(FacetedSpacecraftModelConfig {
            name: "facetedSCModel".to_string(),
            num_facets: facets.len(),
            num_articulated_facets: 1,
        });
        angle_out.connect_to(&mut model.articulated_facet_data_in_msgs[0]);
        for (output, input) in facet_outs.iter().zip(&mut model.facet_element_in_msgs) {
            output.connect_to(input);
        }
        let mut areas = FacetedSpacecraftProjectedArea::new(FacetedSpacecraftProjectedAreaConfig {
            name: "dragAreas".to_string(),
            num_facets: facets.len(),
        });
        state_out.connect_to(&mut areas.spacecraft_state_in_msg);
        for (output, input) in model
            .facet_element_body_out_msgs
            .iter()
            .zip(&mut areas.facet_element_body_in_msgs)
        {
            output.connect_to(input);
        }
        model.init();
        areas.init();
        model.update(&context(0));
        areas.update(&context(0));
        assert_eq!(areas.heading_source(), Some(HeadingSource::Velocity));
        let exposed_m2 = areas.total_projected_area_out_msg.read().area_m2;
        assert!(exposed_m2 > 0.0);

        let density = 1e-12;
        let drag_coeff = 2.2;
        let atmosphere_out = Output::new(AtmosphereMsg {
            neutral_density_kgpm3: density,
            ..Default::default()
        });
        let mut drag = FacetDragDynamicEffector::new(FacetDragDynamicEffectorConfig {
            name: "facetDrag".to_string(),
            planet_rotation_rate_radps: Vector3::zeros(),
        });
        atmosphere_out.connect_to(&mut drag.input_atmosphere_msg);
        for output in &model.facet_element_body_out_msgs {
            let facet = output.read();
            drag.add_facet(
                facet.area_m2,
                drag_coeff,
                facet.normal_body,
                facet.r_cop_b_body_m,
            );
        }
        let force = drag.compute_output(&state).force_inertial_n;
        let speed = state.velocity_mps.norm();
        let expected = 0.5 * density * drag_coeff * speed * speed * exposed_m2;
        assert!((force.norm() - expected).abs() <= 1e-12 * expected);
        assert!((force.normalize() + state.velocity_mps.normalize()).norm() < 1e-12);
    }
}
