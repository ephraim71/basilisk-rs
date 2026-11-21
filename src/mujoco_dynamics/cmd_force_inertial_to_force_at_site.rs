//! Inertial force command to a site-frame force: port of upstream Basilisk
//! `simulation/mujocoDynamics/cmdForceInertialToForceAtSite`.
//!
//! `F_S = [SN] F_N`, with the site attitude `σ_SN` from the site attitude
//! input when it is connected, else from the site state input (a scene
//! site's `state_out_msg`). The output drives a force actuator at the site.
//!
//! Upstream prefers the attitude input when it is linked and written; ports
//! carry no write flag, so here a connected attitude input counts as
//! written.

use super::task::{SceneModel, SceneView, TaskContext};
use crate::messages::{
    BodyForceCommandMsg, InertialForceCommandMsg, Input, NavigationAttitudeMsg, Output,
    SpacecraftStateMsg,
};
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Upstream `CmdForceInertialToForceAtSite`.
#[derive(Debug)]
pub struct CmdForceInertialToForceAtSite {
    /// Upstream `ModelTag`.
    pub name: String,
    /// Upstream `cmdForceInertialInMsg`: the force in inertial components.
    pub cmd_force_inertial_in_msg: Input<InertialForceCommandMsg>,
    /// Upstream `siteFrameStateInMsg`: the site state, the fallback attitude
    /// source.
    pub site_frame_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `siteAttInMsg`: the site attitude, preferred when connected.
    pub site_att_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `forceOutMsg`: the force in site components (`force_S`).
    pub force_out_msg: Output<BodyForceCommandMsg>,
}

impl Default for CmdForceInertialToForceAtSite {
    fn default() -> Self {
        Self {
            name: "cmdForceInertialToForceAtSite".to_string(),
            cmd_force_inertial_in_msg: Input::default(),
            site_frame_state_in_msg: Input::default(),
            site_att_in_msg: Input::default(),
            force_out_msg: Output::default(),
        }
    }
}

impl CmdForceInertialToForceAtSite {
    /// Upstream `Reset`.
    ///
    /// # Panics
    ///
    /// If the force input, or both attitude sources, are unconnected
    /// (upstream raises `BasiliskError`).
    fn check_inputs(&self) {
        assert!(
            self.cmd_force_inertial_in_msg.is_connected(),
            "CmdForceInertialToForceAtSite: cmdForceInertialInMsg is not linked."
        );
        assert!(
            self.site_frame_state_in_msg.is_connected() || self.site_att_in_msg.is_connected(),
            "CmdForceInertialToForceAtSite: siteFrameStateInMsg and siteAttInMsg are not \
             linked. At least one must be linked."
        );
    }

    /// Upstream `UpdateState`.
    fn convert(&self) {
        let sigma_sn = if self.site_att_in_msg.is_connected() {
            self.site_att_in_msg.read().sigma_bn
        } else {
            self.site_frame_state_in_msg.read().sigma_bn
        };
        let force_n = self
            .cmd_force_inertial_in_msg
            .read()
            .force_request_inertial_n;
        self.force_out_msg.write(BodyForceCommandMsg {
            force_request_body_n: mrp_to_dcm(sigma_sn) * force_n,
        });
    }
}

impl SceneModel for CmdForceInertialToForceAtSite {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn scene_reset(&mut self, _scene: &SceneView<'_>, _context: &SimulationContext) {
        self.check_inputs();
    }

    fn scene_update(&mut self, _context: &mut TaskContext<'_>) {
        self.convert();
    }
}

impl Module for CmdForceInertialToForceAtSite {
    fn init(&mut self) {
        self.check_inputs();
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.convert();
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_cmdForceInertialToForceAtSite.py`.

    use nalgebra::Vector3;

    use super::*;
    use crate::mujoco_dynamics::MjScene;
    use crate::mujoco_dynamics::tests::{run, sec2nano};

    fn run_in_scene(model: CmdForceInertialToForceAtSite) -> Vector3<f64> {
        let out = model.force_out_msg.clone();
        let mut scene = MjScene::new("<mujoco/>").expect("scene");
        scene.add_model_to_dynamics_task(model, -1);
        scene.init();
        run(&mut scene, sec2nano(1.0), sec2nano(1.0), |_, _| {});
        out.read().force_request_body_n
    }

    fn assert_close(actual: Vector3<f64>, expected: Vector3<f64>) {
        assert!(
            (actual - expected).abs().max() <= 1e-12,
            "{actual} vs {expected}"
        );
    }

    /// Upstream `test_cmdForceInertialToForceAtSiteSCStates`.
    #[test]
    fn rotates_with_the_site_state_attitude() {
        let force_n = Vector3::new(10.0, -2.0, 5.0);
        let sigma_bn = Vector3::new(0.1, 0.2, 0.3);
        let command = Output::new(InertialForceCommandMsg {
            force_request_inertial_n: force_n,
        });
        let state = Output::new(SpacecraftStateMsg {
            sigma_bn,
            ..SpacecraftStateMsg::default()
        });
        let mut model = CmdForceInertialToForceAtSite::default();
        command.connect_to(&mut model.cmd_force_inertial_in_msg);
        state.connect_to(&mut model.site_frame_state_in_msg);
        assert_close(run_in_scene(model), mrp_to_dcm(sigma_bn) * force_n);
    }

    /// Upstream `test_cmdForceInertialToForceAtSiteNavAtt`: the attitude
    /// input wins over the state input.
    #[test]
    fn prefers_the_attitude_input() {
        let force_n = Vector3::new(3.0, 4.0, 0.0);
        let sigma_att = Vector3::new(0.05, -0.02, 0.01);
        let command = Output::new(InertialForceCommandMsg {
            force_request_inertial_n: force_n,
        });
        let attitude = Output::new(NavigationAttitudeMsg {
            sigma_bn: sigma_att,
            ..NavigationAttitudeMsg::default()
        });
        let state = Output::new(SpacecraftStateMsg {
            sigma_bn: Vector3::new(0.3, 0.0, 0.0),
            ..SpacecraftStateMsg::default()
        });
        let mut model = CmdForceInertialToForceAtSite::default();
        command.connect_to(&mut model.cmd_force_inertial_in_msg);
        attitude.connect_to(&mut model.site_att_in_msg);
        state.connect_to(&mut model.site_frame_state_in_msg);
        assert_close(run_in_scene(model), mrp_to_dcm(sigma_att) * force_n);
    }

    #[test]
    #[should_panic(expected = "At least one must be linked")]
    fn needs_an_attitude_source() {
        let command = Output::new(InertialForceCommandMsg::default());
        let mut model = CmdForceInertialToForceAtSite::default();
        command.connect_to(&mut model.cmd_force_inertial_in_msg);
        Module::init(&mut model);
    }
}
