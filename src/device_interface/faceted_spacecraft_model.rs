//! Faceted spacecraft geometry in the hub frame: port of upstream Basilisk
//! `simulation/deviceInterface/facetedSpacecraftModel`.
//!
//! Each facet is described once in its own frame `F` by a
//! [`FacetElementMsg`]: area, center of pressure `r_CopF_F`, normal `n̂_F`,
//! articulation axis `ĝ_F`, the initial attitude `[F0 B]` and location
//! `r_FB_B` of its frame, and its optical coefficients. The module publishes
//! every facet in hub-frame components as a [`FacetElementBodyMsg`], the
//! geometry input of
//! [`FacetedSrpEffector`](crate::dynamics::faceted_srp_effector::FacetedSrpEffector)
//! and of [`FacetedSpacecraftProjectedArea`](super::faceted_spacecraft_projected_area::FacetedSpacecraftProjectedArea).
//!
//! The first `num_articulated_facets` facets rotate about their axis by the
//! angle `θ` of their [`HingedRigidBodyMsg`] input, the same articulation
//! input [`FacetSRPDynamicEffector`](crate::dynamics::facet_srp_dynamic_effector::FacetSRPDynamicEffector)
//! reads; the rest are fixed. With `[F F0]` the DCM of the principal rotation
//! `θ ĝ_F` and `[F B] = [F F0][F0 B]`,
//!
//! ```text
//! r_CopB_B = [F B]ᵀ r_CopF_F + r_FB_B,    n̂_B = [F B]ᵀ n̂_F,    ĝ_B = [F B]ᵀ ĝ_F
//! ```
//!
//! ([`facet_in_body`]); a fixed facet uses `θ = 0`, so `[F B] = [F0 B]`. The
//! facet geometry is read at reset, normalizing `n̂_F`; the articulation
//! angles every update.
//!
//! Upstream raises when a facet or articulation message is not linked or not
//! written. Stand-in: connected ⇒ written; an unconnected input is reported
//! and reads as the zeroed payload. The input and output counts are fixed at
//! construction from the configuration, so upstream's size checks cannot
//! fail here.

use crate::messages::{FacetElementBodyMsg, FacetElementMsg, HingedRigidBodyMsg, Input, Output};
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::prv_to_dcm;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`FacetedSpacecraftModel`].
#[derive(Clone, Debug)]
pub struct FacetedSpacecraftModelConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `setNumTotalFacets`: number of facets, one geometry input
    /// and one output each.
    pub num_facets: usize,
    /// Number of articulated facets (upstream calls `addArticulatedFacet`
    /// once per facet): the first `num_articulated_facets` facets, one
    /// articulation input each.
    pub num_articulated_facets: usize,
}

/// `facet` in hub-frame components with its frame rotated by
/// `articulation_angle_rad` about its articulation axis: upstream
/// `UpdateState` (and `Reset`, at zero angle, for a fixed facet).
pub fn facet_in_body(facet: &FacetElementMsg, articulation_angle_rad: f64) -> FacetElementBodyMsg {
    let dcm_facet_facet0 = prv_to_dcm(articulation_angle_rad * facet.rotation_axis_facet);
    let dcm_body_facet = (dcm_facet_facet0 * facet.dcm_facet0_body).transpose();
    FacetElementBodyMsg {
        area_m2: facet.area_m2,
        r_cop_b_body_m: dcm_body_facet * facet.r_cop_f_facet_m + facet.r_fb_body_m,
        normal_body: dcm_body_facet * facet.normal_facet,
        rotation_axis_body: dcm_body_facet * facet.rotation_axis_facet,
        diffuse_coeff: facet.diffuse_coeff,
        specular_coeff: facet.specular_coeff,
    }
}

/// Maps facet geometry to the hub frame: upstream `FacetedSpacecraftModel`.
#[derive(Clone, Debug)]
pub struct FacetedSpacecraftModel {
    /// Module configuration.
    pub config: FacetedSpacecraftModelConfig,
    /// Upstream `facetElementInMsgs`: each facet in its own frame, read at
    /// reset.
    pub facet_element_in_msgs: Vec<Input<FacetElementMsg>>,
    /// Upstream `articulatedFacetDataInMsgs`: the articulation angle
    /// (`theta_rad`) of each articulated facet, in facet order.
    pub articulated_facet_data_in_msgs: Vec<Input<HingedRigidBodyMsg>>,
    /// Upstream `facetElementBodyOutMsgs`: each facet in hub-frame
    /// components.
    pub facet_element_body_out_msgs: Vec<Output<FacetElementBodyMsg>>,
    /// The facet geometry read at reset, normals normalized.
    facets: Vec<FacetElementMsg>,
    /// The hub-frame geometry, the fixed facets' computed at reset.
    facets_body: Vec<FacetElementBodyMsg>,
}

impl FacetedSpacecraftModel {
    /// Builds the module with `config.num_facets` geometry inputs and
    /// outputs and `config.num_articulated_facets` articulation inputs.
    ///
    /// # Panics
    ///
    /// If there are more articulated facets than facets.
    pub fn new(config: FacetedSpacecraftModelConfig) -> Self {
        assert!(
            config.num_articulated_facets <= config.num_facets,
            "facetedSpacecraftModel: num_articulated_facets ({}) cannot be greater than \
             num_facets ({})",
            config.num_articulated_facets,
            config.num_facets
        );
        // Not `vec![_; n]`: cloning a port aliases it.
        let facet_element_in_msgs = (0..config.num_facets).map(|_| Input::default()).collect();
        let articulated_facet_data_in_msgs = (0..config.num_articulated_facets)
            .map(|_| Input::default())
            .collect();
        let facet_element_body_out_msgs =
            (0..config.num_facets).map(|_| Output::default()).collect();
        Self {
            config,
            facet_element_in_msgs,
            articulated_facet_data_in_msgs,
            facet_element_body_out_msgs,
            facets: Vec::new(),
            facets_body: Vec::new(),
        }
    }

    /// Upstream `Reset`: reads the facet geometry and maps the fixed facets.
    fn reset_module(&mut self) {
        let name = &self.config.name;
        for (index, input) in self.articulated_facet_data_in_msgs.iter().enumerate() {
            if !input.is_connected() {
                log::error!(
                    "facetedSpacecraftModel '{name}' articulated_facet_data_in_msgs[{index}] is \
                     not connected"
                );
            }
        }
        self.facets = self
            .facet_element_in_msgs
            .iter()
            .enumerate()
            .map(|(index, input)| {
                if !input.is_connected() {
                    log::error!(
                        "facetedSpacecraftModel '{name}' facet_element_in_msgs[{index}] is not \
                         connected"
                    );
                }
                let mut facet = input.read();
                facet.normal_facet = normalize_or_zero(facet.normal_facet);
                facet
            })
            .collect();
        let num_articulated = self.config.num_articulated_facets;
        self.facets_body = self
            .facets
            .iter()
            .enumerate()
            .map(|(index, facet)| {
                if index < num_articulated {
                    FacetElementBodyMsg::default()
                } else {
                    facet_in_body(facet, 0.0)
                }
            })
            .collect();
    }
}

impl Module for FacetedSpacecraftModel {
    fn init(&mut self) {
        self.reset_module();
    }

    fn update(&mut self, _context: &SimulationContext) {
        for ((facet, facet_body), angle_in) in self
            .facets
            .iter()
            .zip(&mut self.facets_body)
            .zip(&self.articulated_facet_data_in_msgs)
        {
            *facet_body = facet_in_body(facet, angle_in.read().theta_rad);
        }
        for (output, facet_body) in self
            .facet_element_body_out_msgs
            .iter()
            .zip(&self.facets_body)
        {
            output.write(facet_body.clone());
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `facetedSpacecraftModel/_UnitTest/test_facetedSpacecraftModel.py`
    //! over its full parameter grid with its tolerance. Its truth uses
    //! `rbk.PRV2C`, which is [`prv_to_dcm`] here.

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{FacetedSpacecraftModel, FacetedSpacecraftModelConfig};
    use crate::messages::{FacetElementBodyMsg, FacetElementMsg, HingedRigidBodyMsg, Output};
    use crate::utilities::rigid_body_kinematics::prv_to_dcm;
    use crate::{Module, SimulationContext};

    const D2R: f64 = std::f64::consts::PI / 180.0;
    const STEP_NANOS: u64 = 1_000_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// `np.testing.assert_allclose(actual, desired, atol = accuracy)`, whose
    /// `rtol` stays at its default `1e-7`.
    fn assert_allclose(actual: Vector3<f64>, desired: Vector3<f64>, what: &str) {
        let accuracy = 1e-12;
        for axis in 0..3 {
            assert!(
                (actual[axis] - desired[axis]).abs() <= accuracy + 1e-7 * desired[axis].abs(),
                "{what}: {actual:?} vs {desired:?}"
            );
        }
    }

    /// Upstream's two facets: facet 1 articulates about `F` `y`, facet 2 is
    /// fixed.
    fn facets(initial_angle_1: f64, initial_angle_2: f64) -> [FacetElementMsg; 2] {
        let rotation_axes = [Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)];
        [
            FacetElementMsg {
                area_m2: 0.5,
                r_cop_f_facet_m: Vector3::new(-0.1, 0.1, -0.1),
                normal_facet: Vector3::new(1.0, 0.0, 0.0),
                rotation_axis_facet: rotation_axes[0],
                dcm_facet0_body: prv_to_dcm(initial_angle_1 * rotation_axes[0]),
                r_fb_body_m: Vector3::new(0.0, 1.0, 0.0),
                diffuse_coeff: 0.1,
                specular_coeff: 0.9,
            },
            FacetElementMsg {
                area_m2: 1.0,
                r_cop_f_facet_m: Vector3::new(0.1, -0.1, -0.1),
                normal_facet: Vector3::new(0.0, 1.0, 0.0),
                rotation_axis_facet: rotation_axes[1],
                dcm_facet0_body: prv_to_dcm(initial_angle_2 * rotation_axes[1]),
                r_fb_body_m: Vector3::new(0.0, 0.0, -1.0),
                diffuse_coeff: 0.1,
                specular_coeff: 0.9,
            },
        ]
    }

    /// Upstream `compute_facet_body_data` for one facet at one angle:
    /// `r_CopB_B`, `n̂_B` and `ĝ_B`.
    fn truth(facet: &FacetElementMsg, angle: f64) -> [Vector3<f64>; 3] {
        let dcm_ff0: Matrix3<f64> = prv_to_dcm(angle * facet.rotation_axis_facet);
        let dcm_fb = dcm_ff0 * facet.dcm_facet0_body;
        [
            dcm_fb.transpose() * facet.r_cop_f_facet_m + facet.r_fb_body_m,
            dcm_fb.transpose() * facet.normal_facet,
            dcm_fb.transpose() * facet.rotation_axis_facet,
        ]
    }

    fn check(sample: &FacetElementBodyMsg, expected: &[Vector3<f64>; 3], what: &str) {
        assert_allclose(
            sample.r_cop_b_body_m,
            expected[0],
            &format!("{what} r_CopB_B"),
        );
        assert_allclose(sample.normal_body, expected[1], &format!("{what} nHat_B"));
        assert_allclose(
            sample.rotation_axis_body,
            expected[2],
            &format!("{what} rotHat_B"),
        );
    }

    /// Upstream `test_facetedSpacecraftModel` over its grid: the
    /// intermediate articulation angle for 0–2 s, the final one for 3–5 s.
    #[test]
    fn facets_map_to_the_hub_frame_over_the_upstream_grid() {
        for initial_1 in [0.0, 10.0, 75.0, -90.0] {
            for intermediate_1 in [-33.3, 45.2, 90.0, 180.0] {
                for final_1 in [0.0, -5.4, 22.5] {
                    for initial_2 in [0.0, 10.0, 75.0, -90.0] {
                        let what = format!(
                            "initial {initial_1}, intermediate {intermediate_1}, final {final_1}, \
                             fixed initial {initial_2}"
                        );
                        let facet_messages = facets(initial_1 * D2R, initial_2 * D2R);
                        let facet_outs: Vec<_> =
                            facet_messages.iter().cloned().map(Output::new).collect();
                        let angle_out = Output::new(HingedRigidBodyMsg {
                            theta_rad: intermediate_1 * D2R,
                            theta_dot_radps: 0.0,
                        });
                        let mut model = FacetedSpacecraftModel::new(FacetedSpacecraftModelConfig {
                            name: "facetedSCModel".to_string(),
                            num_facets: 2,
                            num_articulated_facets: 1,
                        });
                        angle_out.connect_to(&mut model.articulated_facet_data_in_msgs[0]);
                        for (output, input) in
                            facet_outs.iter().zip(&mut model.facet_element_in_msgs)
                        {
                            output.connect_to(input);
                        }
                        model.init();

                        let mut logs: [Vec<FacetElementBodyMsg>; 2] = Default::default();
                        for second in 0..=5 {
                            if second == 3 {
                                angle_out.write(HingedRigidBodyMsg {
                                    theta_rad: final_1 * D2R,
                                    theta_dot_radps: 0.0,
                                });
                            }
                            model.update(&context(second * STEP_NANOS));
                            for (log, output) in
                                logs.iter_mut().zip(&model.facet_element_body_out_msgs)
                            {
                                log.push(output.read());
                            }
                        }

                        let intermediate = truth(&facet_messages[0], intermediate_1 * D2R);
                        let last = truth(&facet_messages[0], final_1 * D2R);
                        let fixed = truth(&facet_messages[1], 0.0);
                        for (second, sample) in logs[0].iter().enumerate() {
                            let expected = if second < 3 { &intermediate } else { &last };
                            check(sample, expected, &format!("{what}, facet 1 at {second} s"));
                        }
                        for (second, sample) in logs[1].iter().enumerate() {
                            check(sample, &fixed, &format!("{what}, facet 2 at {second} s"));
                        }
                    }
                }
            }
        }
    }

    /// The facet's area and optical coefficients pass through, and `n̂_F`
    /// is normalized at reset.
    #[test]
    fn normalizes_the_normal_and_passes_the_optics_through() {
        let facet = FacetElementMsg {
            area_m2: 2.5,
            normal_facet: Vector3::new(0.0, 0.0, 4.0),
            dcm_facet0_body: Matrix3::identity(),
            diffuse_coeff: 0.3,
            specular_coeff: 0.6,
            ..Default::default()
        };
        let facet_out = Output::new(facet);
        let mut model = FacetedSpacecraftModel::new(FacetedSpacecraftModelConfig {
            name: "facetedSCModel".to_string(),
            num_facets: 1,
            num_articulated_facets: 0,
        });
        facet_out.connect_to(&mut model.facet_element_in_msgs[0]);
        model.init();
        model.update(&context(0));
        let body = model.facet_element_body_out_msgs[0].read();
        assert_eq!(body.normal_body, Vector3::z());
        assert_eq!(
            (body.area_m2, body.diffuse_coeff, body.specular_coeff),
            (2.5, 0.3, 0.6)
        );
    }

    #[test]
    #[should_panic(expected = "num_articulated_facets (3) cannot be greater than num_facets (2)")]
    fn rejects_more_articulated_facets_than_facets() {
        let _ = FacetedSpacecraftModel::new(FacetedSpacecraftModelConfig {
            name: "facetedSCModel".to_string(),
            num_facets: 2,
            num_articulated_facets: 3,
        });
    }
}
