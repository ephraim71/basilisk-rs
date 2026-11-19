//! Constraint-compliant slew between two attitudes: port of upstream
//! Basilisk `constrainedAttitudeManeuver`.
//!
//! At reset the module plans the whole maneuver. It discretizes MRP space
//! into a grid of fineness `N` (see [`mrp_grid`]), marks the attitudes at
//! which a keep-out instrument would see the keep-out body, or no keep-in
//! instrument the keep-in body, as blocked, and searches the graph of free
//! nodes from the spacecraft's attitude to the goal attitude with A*:
//!
//! - [`CostFunction::MinimumDistance`]: the shortest path in MRP distance,
//!   exploring at most `4 N²` nodes;
//! - [`CostFunction::MinimumEffort`]: each candidate path, completed by a
//!   direct leg to the goal, is turned into a trajectory and scored by its
//!   control effort `∫ |L| dt`; at most 100 nodes are explored.
//!
//! The waypoints of the path are switched to shadow sets where the path
//! crosses `|σ| = 1`, so the MRPs stay continuous, and time-tagged so the
//! maneuver lasts `4 S / ω_avg` for a path of MRP length `S`. A degree-4
//! B-spline through them (or a least-squares fit) with the start and goal
//! MRP rates gives the reference trajectory, which every update samples at
//! the current simulation time: `σ_R/N`, and `ω_R/N`, `ω̇_R/N` in inertial
//! components. After the maneuver the goal attitude is held with zero rate.
//! The control effort uses the inertia from the vehicle configuration:
//! `L = [I] ω̇ + ω × [I] ω`.
//!
//! # Message write state
//!
//! Upstream reads each input, and applies a celestial-body constraint, only
//! when the message has been written (`isWritten`). The Rust inputs carry no
//! header: a connected input counts as written.
//!
//! # Deviations from upstream
//!
//! - Upstream links the goal's grid neighbours to a copy of the goal that
//!   does not outlive grid generation; here they link to the goal node.
//! - Where a path in the shadow set reaches the origin, upstream maps the
//!   origin itself to its (infinite) shadow set; here it stays at zero.
//! - A reset replans from scratch; upstream adds to the grid and path of the
//!   previous plan, and its effort evaluation overwrites `N` with the number
//!   of trajectory samples.
//! - A least-squares fit needs at least as many waypoints as the spline
//!   degree (4); a shorter path is interpolated instead (upstream's fit is
//!   undefined there).
//! - When no maneuver can be planned an error is logged and the module
//!   holds the initial attitude with zero rate. That covers what upstream
//!   leaves undefined (no free node, a dead end in the search, start and
//!   goal on the same grid node) and a trajectory that is not finite, as
//!   upstream's degree-4 fit gives for a path of two nodes.

pub mod mrp_grid;
#[cfg(test)]
mod upstream_truth;

use nalgebra::{Matrix3, Vector3};

use self::mrp_grid::{Boresight, Constraints, MrpGrid, Node, NodeKey, PlanningError, mrp_distance};
use crate::messages::{
    AttitudeReferenceMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg, VehicleConfigMsg,
};
use crate::time::NANO2SEC;
use crate::utilities::bspline::{
    BSplineInput, BSplineOutput, WaypointTiming, approximate, interpolate,
};
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::{
    d_mrp_to_omega, d_omega_to_mrp, dd_mrp_to_domega, mrp_shadow, mrp_to_dcm,
};
use crate::{Module, SimulationContext};

/// Samples of the planned trajectory (upstream's `Num = 100`).
const TRAJECTORY_SAMPLES: usize = 100;
/// Degree of the trajectory B-spline (upstream's `P = 4`).
const TRAJECTORY_DEGREE: usize = 4;
/// Nodes the effort-based search explores at most (upstream's `Nmax`).
const EFFORT_SEARCH_ITERATIONS: usize = 100;

/// The graph-search cost (upstream `costFcnType`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CostFunction {
    /// Upstream 0: the shortest path in MRP distance.
    #[default]
    MinimumDistance = 0,
    /// Upstream 1: the path of least control effort.
    MinimumEffort = 1,
}

/// How the trajectory is fitted to the waypoints (upstream `BSplineType`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TrajectoryFit {
    /// Upstream 0: interpolation through every waypoint.
    #[default]
    Interpolation = 0,
    /// Upstream 1: least-squares approximation, through the end points only.
    LeastSquares = 1,
}

/// User-settable parameters of [`ConstrainedAttitudeManeuver`], read at
/// reset.
#[derive(Clone, Debug)]
pub struct ConstrainedAttitudeManeuverConfig {
    /// Module instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `N` (the constructor argument): grid fineness, the number of
    /// grid values from 0 to 1 along each axis. At least 2.
    pub n: usize,
    /// Upstream `BSplineType`.
    pub trajectory_fit: TrajectoryFit,
    /// Upstream `costFcnType`.
    pub cost_function: CostFunction,
    /// Upstream `sigma_BN_goal`: goal attitude.
    pub sigma_bn_goal: Vector3<f64>,
    /// Upstream `omega_BN_B_goal` \[rad/s\]: goal angular rate, body
    /// components.
    pub omega_bn_b_goal_radps: Vector3<f64>,
    /// Upstream `avgOmega` \[rad/s\]: average angular rate norm over the
    /// maneuver, which sets its duration.
    pub avg_omega_radps: f64,
    /// Upstream `boresights.keepOut*`: instruments that must not see the
    /// keep-out body. Add them with [`Self::append_keep_out_direction`].
    pub keep_out_boresights: Vec<Boresight>,
    /// Upstream `boresights.keepIn*`: instruments of which one must see the
    /// keep-in body. Add them with [`Self::append_keep_in_direction`].
    pub keep_in_boresights: Vec<Boresight>,
}

impl Default for ConstrainedAttitudeManeuverConfig {
    fn default() -> Self {
        Self {
            name: "constrainedAttitudeManeuver".to_string(),
            n: 5,
            trajectory_fit: TrajectoryFit::Interpolation,
            cost_function: CostFunction::MinimumDistance,
            sigma_bn_goal: Vector3::zeros(),
            omega_bn_b_goal_radps: Vector3::zeros(),
            avg_omega_radps: 0.0,
            keep_out_boresights: Vec::new(),
            keep_in_boresights: Vec::new(),
        }
    }
}

impl ConstrainedAttitudeManeuverConfig {
    /// Upstream `appendKeepOutDirection`: adds a keep-out instrument along
    /// `direction_b` (normalized) with half-angle field of view `fov_rad`.
    pub fn append_keep_out_direction(&mut self, direction_b: Vector3<f64>, fov_rad: f64) {
        self.keep_out_boresights
            .push(Boresight::new(direction_b, fov_rad));
    }

    /// Upstream `appendKeepInDirection`: adds a keep-in instrument along
    /// `direction_b` (normalized) with half-angle field of view `fov_rad`.
    pub fn append_keep_in_direction(&mut self, direction_b: Vector3<f64>, fov_rad: f64) {
        self.keep_in_boresights
            .push(Boresight::new(direction_b, fov_rad));
    }
}

/// Constraint-compliant attitude maneuver planner and reference generator
/// (upstream `constrainedAttitudeManeuver`).
#[derive(Clone, Debug)]
pub struct ConstrainedAttitudeManeuver {
    /// User-settable parameters, read at reset.
    pub config: ConstrainedAttitudeManeuverConfig,
    /// Upstream `scStateInMsg`: the spacecraft position (for the body
    /// directions), initial attitude and rate.
    pub sc_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `vehicleConfigInMsg`: the inertia about point B for the
    /// control effort.
    pub vehicle_config_in_msg: Input<VehicleConfigMsg>,
    /// Upstream `keepOutCelBodyInMsg` (optional): the keep-out body.
    pub keep_out_cel_body_in_msg: Input<PlanetStateMsg>,
    /// Upstream `keepInCelBodyInMsg` (optional): the keep-in body.
    pub keep_in_cel_body_in_msg: Input<PlanetStateMsg>,
    /// Upstream `attRefOutMsg`: the reference attitude, rate and
    /// acceleration.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
    /// Upstream `NodesMap`, `keyS` and `keyG`.
    grid: MrpGrid,
    /// Upstream `path`: node indices from start to goal.
    path: Vec<usize>,
    /// Upstream `Output`: the sampled reference trajectory.
    trajectory: Option<BSplineOutput>,
    /// Upstream `pathCost`.
    path_cost: f64,
    /// The inertia read at reset \[kg m²\].
    inertia_kg_m2: Matrix3<f64>,
    /// The initial attitude, held when no path could be planned.
    sigma_bn_start: Vector3<f64>,
}

/// Everything that turns a path into a trajectory and scores it.
struct TrajectoryPlanner {
    fit: TrajectoryFit,
    avg_omega_radps: f64,
    /// MRP rate at the start, from the spacecraft rate.
    sigma_dot_start: Vector3<f64>,
    /// MRP rate at the goal, from the goal rate.
    sigma_dot_goal: Vector3<f64>,
    inertia_kg_m2: Matrix3<f64>,
}

impl TrajectoryPlanner {
    /// Upstream `pathHandle`: the waypoints of `path`, switched to shadow
    /// sets after each jump of `|Δσ| > 1` (a boundary crossing) and back where
    /// the path crosses the origin, time-tagged by MRP distance so the
    /// maneuver lasts `4 S / ω_avg`.
    fn waypoints(&self, grid: &MrpGrid, path: &[usize]) -> BSplineInput {
        let sigma = |n: usize| grid.nodes[path[n]].sigma_bn;
        let last = path.len() - 1;
        let mut tags = vec![0.0; path.len()];
        let mut length = 0.0;
        for n in 0..last {
            tags[n + 1] = tags[n] + mrp_distance(&sigma(n), &sigma(n + 1));
            length += tags[n + 1] - tags[n];
        }

        let origin = grid.index_of([0, 0, 0]);
        let mut waypoints = vec![Vector3::zeros(); path.len()];
        let mut shadow_set = false;
        for n in 0..last {
            if shadow_set && Some(path[n]) == origin {
                for waypoint in &mut waypoints[..n] {
                    *waypoint = -*waypoint / waypoint.dot(waypoint);
                }
                shadow_set = false;
            }
            waypoints[n] = if shadow_set {
                mrp_shadow(sigma(n))
            } else {
                sigma(n)
            };
            if (sigma(n) - sigma(n + 1)).norm() > 1.0 {
                shadow_set = !shadow_set;
            }
        }
        waypoints[last] = if shadow_set {
            mrp_shadow(sigma(last))
        } else {
            sigma(last)
        };

        let t_last = tags[last];
        let mut input = BSplineInput::new(waypoints);
        input.timing = WaypointTiming::TimeTags(
            tags.iter()
                .map(|tag| tag * 4.0 * length / (t_last * self.avg_omega_radps))
                .collect(),
        );
        input
    }

    /// Upstream `spline`: the trajectory through `input` with the start and
    /// goal MRP rates.
    fn fit(&self, mut input: BSplineInput) -> BSplineOutput {
        input.x_dot_0 = Some(self.sigma_dot_start);
        input.x_dot_n = Some(self.sigma_dot_goal);
        let count = input.waypoints.len();
        match self.fit {
            TrajectoryFit::LeastSquares if count >= TRAJECTORY_DEGREE => {
                approximate(&input, TRAJECTORY_SAMPLES, count, TRAJECTORY_DEGREE)
            }
            _ => interpolate(&input, TRAJECTORY_SAMPLES, TRAJECTORY_DEGREE),
        }
    }

    /// The trajectory along `path`.
    fn trajectory(&self, grid: &MrpGrid, path: &[usize]) -> BSplineOutput {
        self.fit(self.waypoints(grid, path))
    }

    /// Upstream `effortEvaluation`: `∫ |L| dt` over the trajectory by the
    /// trapezoidal rule.
    fn effort(&self, trajectory: &BSplineOutput) -> f64 {
        let mut effort = 0.0;
        let mut torque_a = control_torque(trajectory, 0, &self.inertia_kg_m2).norm();
        for n in 0..trajectory.t.len() - 1 {
            let torque_b = control_torque(trajectory, n + 1, &self.inertia_kg_m2).norm();
            effort += (torque_a + torque_b) * (trajectory.t[n + 1] - trajectory.t[n]) / 2.0;
            torque_a = torque_b;
        }
        effort
    }
}

/// Upstream `computeTorque`: `[I] ω̇ + ω × [I] ω` at trajectory sample `n`.
fn control_torque(trajectory: &BSplineOutput, n: usize, inertia: &Matrix3<f64>) -> Vector3<f64> {
    let (sigma, sigma_dot, sigma_ddot) =
        (trajectory.x[n], trajectory.x_dot[n], trajectory.x_ddot[n]);
    let omega = d_mrp_to_omega(sigma, sigma_dot);
    let omega_dot = dd_mrp_to_domega(sigma, sigma_dot, sigma_ddot);
    inertia * omega_dot + omega.cross(&(inertia * omega))
}

impl ConstrainedAttitudeManeuver {
    /// Creates the module with `config` and unconnected ports; the maneuver
    /// is planned at reset.
    pub fn new(config: ConstrainedAttitudeManeuverConfig) -> Self {
        Self {
            config,
            sc_state_in_msg: Input::default(),
            vehicle_config_in_msg: Input::default(),
            keep_out_cel_body_in_msg: Input::default(),
            keep_in_cel_body_in_msg: Input::default(),
            att_ref_out_msg: Output::default(),
            grid: MrpGrid::default(),
            path: Vec::new(),
            trajectory: None,
            path_cost: 0.0,
            inertia_kg_m2: Matrix3::zeros(),
            sigma_bn_start: Vector3::zeros(),
        }
    }

    /// Upstream `Reset`: reads the inputs, builds the grid, searches it and
    /// fits the trajectory.
    pub fn reset(&mut self) {
        let config = &self.config;
        let name = &config.name;
        // Upstream `ReadInputs`; an unconnected input reads zeros, as
        // upstream's unwritten buffers.
        let sc_state = self.sc_state_in_msg.read();
        self.inertia_kg_m2 = self
            .vehicle_config_in_msg
            .read()
            .inertia_about_point_b_kg_m2;
        let direction_to = |input: &Input<PlanetStateMsg>| {
            input
                .is_connected()
                .then(|| normalize_or_zero(input.read().position_inertial_m - sc_state.position_m))
        };
        let constraints = Constraints {
            keep_out_dir_n: direction_to(&self.keep_out_cel_body_in_msg),
            keep_in_dir_n: direction_to(&self.keep_in_cel_body_in_msg),
            keep_out: config.keep_out_boresights.clone(),
            keep_in: config.keep_in_boresights.clone(),
        };
        if constraints.keep_out_dir_n.is_none() && constraints.keep_in_dir_n.is_none() {
            log::warn!(
                "{name}: no celestial body inputs are connected; there are no rotational constraints"
            );
        }

        let start = Node::new(sc_state.sigma_bn, &constraints);
        let goal = Node::new(config.sigma_bn_goal, &constraints);
        if !start.is_free {
            log::warn!(
                "{name}: the initial attitude of the spacecraft is not constraint-compliant"
            );
        }
        if !goal.is_free {
            log::warn!("{name}: the target attitude of the spacecraft is not constraint-compliant");
        }
        self.sigma_bn_start = sc_state.sigma_bn;
        let planner = TrajectoryPlanner {
            fit: config.trajectory_fit,
            avg_omega_radps: config.avg_omega_radps,
            sigma_dot_start: d_omega_to_mrp(sc_state.sigma_bn, sc_state.omega_radps),
            sigma_dot_goal: d_omega_to_mrp(config.sigma_bn_goal, config.omega_bn_b_goal_radps),
            inertia_kg_m2: self.inertia_kg_m2,
        };

        let planned = self
            .plan(start, goal, &constraints, &planner)
            .and_then(|()| {
                let trajectory = planner.trajectory(&self.grid, &self.path);
                let finite = [&trajectory.x, &trajectory.x_dot, &trajectory.x_ddot]
                    .into_iter()
                    .flatten()
                    .all(|value| value.iter().all(|component| component.is_finite()));
                if finite {
                    Ok(trajectory)
                } else {
                    Err(PlanningError::DegenerateTrajectory)
                }
            });
        match planned {
            Ok(trajectory) => {
                self.path_cost = planner.effort(&trajectory);
                self.trajectory = Some(trajectory);
            }
            Err(error) => {
                log::error!(
                    "{}: {error}; holding the initial attitude",
                    self.config.name
                );
                self.path.clear();
                self.trajectory = None;
                self.path_cost = f64::NAN;
            }
        }
    }

    /// Upstream `GenerateGrid` and `AStar`/`effortBasedAStar`.
    fn plan(
        &mut self,
        start: Node,
        goal: Node,
        constraints: &Constraints,
        planner: &TrajectoryPlanner,
    ) -> Result<(), PlanningError> {
        let n = self.config.n;
        self.grid = MrpGrid::default();
        self.grid = MrpGrid::generate(n, start, goal, constraints)?;
        self.path = match self.config.cost_function {
            CostFunction::MinimumDistance => self.grid.distance_a_star(4 * n * n)?,
            CostFunction::MinimumEffort => self
                .grid
                .effort_a_star(EFFORT_SEARCH_ITERATIONS, |grid, path| {
                    planner.effort(&planner.trajectory(grid, path))
                })?,
        };
        if self.path.len() < 2 {
            return Err(PlanningError::StartIsGoal);
        }
        Ok(())
    }

    /// The attitude of the grid node at `key`, if the grid has one (upstream
    /// `returnNodeCoord`).
    pub fn node_sigma(&self, key: NodeKey) -> Option<Vector3<f64>> {
        self.grid.node(key).map(|node| node.sigma_bn)
    }

    /// Whether the grid node at `key` meets the constraints; `false` when the
    /// grid has none (upstream `returnNodeState`).
    pub fn node_is_free(&self, key: NodeKey) -> bool {
        self.grid.node(key).is_some_and(|node| node.is_free)
    }

    /// The attitudes along the planned path, start to goal (upstream
    /// `path`, `returnPathCoord`).
    pub fn path(&self) -> Vec<Vector3<f64>> {
        self.path
            .iter()
            .map(|&index| self.grid.nodes[index].sigma_bn)
            .collect()
    }

    /// The planned trajectory (upstream `Output`), if planning succeeded.
    pub fn trajectory(&self) -> Option<&BSplineOutput> {
        self.trajectory.as_ref()
    }

    /// Upstream `pathCost`: the control effort of the planned trajectory
    /// \[N m s\]; NaN when planning failed.
    pub fn path_cost(&self) -> f64 {
        self.path_cost
    }

    /// Upstream `computeTorqueNorm`: the control torque norm \[N m\] at
    /// trajectory sample `index`, with the inertia read at reset.
    ///
    /// # Panics
    ///
    /// If no trajectory was planned or `index` is past its last sample.
    pub fn torque_norm(&self, index: usize) -> f64 {
        let trajectory = self.trajectory.as_ref().expect("no trajectory was planned");
        control_torque(trajectory, index, &self.inertia_kg_m2).norm()
    }

    /// Upstream `UpdateState`'s reference at simulation time `t_s`.
    fn reference_at(&self, t_s: f64) -> AttitudeReferenceMsg {
        let Some(trajectory) = &self.trajectory else {
            return AttitudeReferenceMsg {
                sigma_rn: self.sigma_bn_start,
                ..Default::default()
            };
        };
        let state = trajectory.state_at(t_s);
        let omega_rn_r = d_mrp_to_omega(state.x, state.x_dot);
        let omega_dot_rn_r = dd_mrp_to_domega(state.x, state.x_dot, state.x_ddot);
        let dcm_nr = mrp_to_dcm(state.x).transpose();
        AttitudeReferenceMsg {
            sigma_rn: state.x,
            omega_rn_n_radps: dcm_nr * omega_rn_r,
            domega_rn_n_radps2: dcm_nr * omega_dot_rn_r,
        }
    }
}

impl Module for ConstrainedAttitudeManeuver {
    fn init(&mut self) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
        ConstrainedAttitudeManeuver::reset(self);
    }

    fn reset(&mut self, _context: &SimulationContext) {
        ConstrainedAttitudeManeuver::reset(self);
    }

    /// The trajectory is sampled at the simulation time, not the time since
    /// reset, as upstream.
    fn update(&mut self, context: &SimulationContext) {
        let t_s = context.current_sim_nanos as f64 * NANO2SEC;
        self.att_ref_out_msg.write(self.reference_at(t_s));
    }
}

#[cfg(test)]
mod tests {
    //! Ports `constrainedAttitudeManeuver/_UnitTest/test_ConstrainedAttitudeManeuver.py`
    //! for every parametrization (`N` = 5, 6 and `costFcnType` = 0, 1, with
    //! `keepOutFov = 20°`, `keepInFov = 70°`, `accuracy = 1e-12`). The
    //! pytest's truth is its own Python grid, A* and trajectory code, which
    //! [`super::upstream_truth`] reimplements; like the pytest it fits the
    //! trajectory with the shared B-spline utility. The C-wrapped output
    //! check is the same message.

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::upstream_truth::{self, TruthSetup};
    use super::{
        ConstrainedAttitudeManeuver, ConstrainedAttitudeManeuverConfig, CostFunction, TrajectoryFit,
    };
    use crate::messages::{Output, PlanetStateMsg, SpacecraftStateMsg, VehicleConfigMsg};
    use crate::utilities::astro_constants::D2R;
    use crate::utilities::rigid_body_kinematics::{d_mrp_to_omega, dd_mrp_to_domega, mrp_to_dcm};
    use crate::{Module, SimulationContext};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The pytest's inputs.
    fn setup(n: usize, cost_function: CostFunction) -> TruthSetup {
        TruthSetup {
            n,
            cost_function,
            inertia: Matrix3::new(
                0.02 / 3.0,
                0.,
                0.,
                0.,
                0.1256 / 3.0,
                0.,
                0.,
                0.,
                0.1256 / 3.0,
            ),
            planet_position: Vector3::new(10.0, 0.0, 0.0),
            sc_position: Vector3::new(1.0, 0.0, 0.0),
            sigma_start: Vector3::new(0.0, 0.0, -0.5),
            sigma_goal: Vector3::new(0.0, 0.5, 0.0),
            omega_start: Vector3::zeros(),
            omega_goal: Vector3::zeros(),
            avg_omega: 0.03,
            keep_out_boresights: vec![Vector3::new(1.0, 0.0, 0.0)],
            keep_in_boresights: vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)],
            keep_out_fov: 20.0 * D2R,
            keep_in_fov: 70.0 * D2R,
        }
    }

    /// The module as the pytest configures and wires it.
    fn wired_module(
        setup: &TruthSetup,
    ) -> (ConstrainedAttitudeManeuver, [Output<PlanetStateMsg>; 1]) {
        let mut config = ConstrainedAttitudeManeuverConfig {
            name: "testModule".to_string(),
            n: setup.n,
            trajectory_fit: TrajectoryFit::Interpolation,
            cost_function: setup.cost_function,
            sigma_bn_goal: setup.sigma_goal,
            omega_bn_b_goal_radps: setup.omega_goal,
            avg_omega_radps: setup.avg_omega,
            ..Default::default()
        };
        config.append_keep_out_direction(setup.keep_out_boresights[0], setup.keep_out_fov);
        for boresight in &setup.keep_in_boresights {
            config.append_keep_in_direction(*boresight, setup.keep_in_fov);
        }
        let mut module = ConstrainedAttitudeManeuver::new(config);
        let sc_state = Output::new(SpacecraftStateMsg {
            position_m: setup.sc_position,
            sigma_bn: setup.sigma_start,
            omega_radps: setup.omega_start,
            ..Default::default()
        });
        let vehicle = Output::new(VehicleConfigMsg {
            inertia_about_point_b_kg_m2: setup.inertia,
            ..Default::default()
        });
        let planet = Output::new(PlanetStateMsg {
            position_inertial_m: setup.planet_position,
            ..Default::default()
        });
        sc_state.connect_to(&mut module.sc_state_in_msg);
        vehicle.connect_to(&mut module.vehicle_config_in_msg);
        planet.connect_to(&mut module.keep_out_cel_body_in_msg);
        planet.connect_to(&mut module.keep_in_cel_body_in_msg);
        (module, [planet])
    }

    fn assert_vector_close(
        actual: Vector3<f64>,
        expected: Vector3<f64>,
        accuracy: f64,
        what: &str,
    ) {
        assert!(
            (actual - expected).norm() <= accuracy,
            "{what}: {actual} != {expected}"
        );
    }

    /// Upstream `test_constrainedAttitudeManeuver`, every parametrization.
    #[test]
    fn matches_upstream_constrained_attitude_maneuver_unit_test() {
        let accuracy = 1e-12;
        for n in [5, 6] {
            for cost_function in [CostFunction::MinimumDistance, CostFunction::MinimumEffort] {
                let case = format!("N = {n}, {cost_function:?}");
                let setup = setup(n, cost_function);
                let truth = upstream_truth::plan(&setup);

                let (mut module, _planet) = wired_module(&setup);
                module.init();

                // Grid points: same coordinates and constraint state.
                let n = n as i32;
                for i in -n..=n {
                    for j in -n..=n {
                        for k in -n..=n {
                            let Some(node) = truth.node([i, j, k]) else {
                                continue;
                            };
                            let what = format!("{case}: node ({i}, {j}, {k})");
                            let sigma = module
                                .node_sigma([i, j, k])
                                .unwrap_or(Vector3::from_element(1000.0));
                            assert_vector_close(sigma, node.sigma, accuracy, &what);
                            assert_eq!(module.node_is_free([i, j, k]), node.is_free, "{what}");
                        }
                    }
                }

                // The same path.
                let path = module.path();
                assert_eq!(
                    path.len(),
                    truth.path.len(),
                    "{case}: path {path:?} vs {:?}",
                    truth.path
                );
                for (p, (actual, expected)) in path.iter().zip(&truth.path).enumerate() {
                    assert_vector_close(
                        *actual,
                        *expected,
                        accuracy,
                        &format!("{case}: waypoint {p}"),
                    );
                }

                // The same trajectory, torques and cost.
                let trajectory = module.trajectory().unwrap();
                let expected = &truth.trajectory;
                assert_eq!(trajectory.t.len(), expected.t.len());
                for n in 0..expected.t.len() {
                    let what = format!("{case}: trajectory index {n}");
                    assert!(
                        (trajectory.t[n] - expected.t[n]).abs() <= accuracy,
                        "{what}"
                    );
                    assert_vector_close(trajectory.x[n], expected.x[n], accuracy, &what);
                    assert_vector_close(trajectory.x_dot[n], expected.x_dot[n], accuracy, &what);
                    assert_vector_close(trajectory.x_ddot[n], expected.x_ddot[n], accuracy, &what);
                    let torque = upstream_truth::compute_torque(
                        expected.x[n],
                        expected.x_dot[n],
                        expected.x_ddot[n],
                        &setup.inertia,
                    );
                    assert!(
                        (module.torque_norm(n) - torque.norm()).abs() <= accuracy,
                        "{what}"
                    );
                }
                assert!(
                    (module.path_cost() - truth.path_cost).abs() <= accuracy,
                    "{case}: path cost"
                );

                // The reference message every 0.5 s for 2 minutes.
                for tick in 0..=240_u64 {
                    let now = tick * 500_000_000;
                    module.update(&context(now));
                    let t = now as f64 * 1e-9;
                    let state = expected.state_at(t);
                    let rn = mrp_to_dcm(state.x);
                    let omega_rn_n = rn.transpose() * d_mrp_to_omega(state.x, state.x_dot);
                    let omega_dot_rn_n =
                        rn.transpose() * dd_mrp_to_domega(state.x, state.x_dot, state.x_ddot);
                    let output = module.att_ref_out_msg.read();
                    let what = format!("{case}: reference at t = {t}");
                    assert_vector_close(output.sigma_rn, state.x, accuracy, &what);
                    assert_vector_close(output.omega_rn_n_radps, omega_rn_n, accuracy, &what);
                    assert_vector_close(output.domega_rn_n_radps2, omega_dot_rn_n, accuracy, &what);
                }
            }
        }
    }

    /// Not upstream: the planned path avoids the keep-out zone, keeps a
    /// keep-in instrument on the body, and the trajectory runs from the start
    /// to the goal attitude, holding the goal at rest afterwards.
    #[test]
    fn trajectory_meets_the_constraints_at_the_waypoints_and_ends_at_the_goal() {
        let setup = setup(6, CostFunction::MinimumDistance);
        let (mut module, _planet) = wired_module(&setup);
        module.init();
        let constraints = super::Constraints {
            keep_out_dir_n: Some(Vector3::x()),
            keep_in_dir_n: Some(Vector3::x()),
            keep_out: module.config.keep_out_boresights.clone(),
            keep_in: module.config.keep_in_boresights.clone(),
        };
        let path = module.path();
        for sigma in &path[1..path.len() - 1] {
            assert!(constraints.is_compliant(sigma), "waypoint {sigma}");
        }
        let trajectory = module.trajectory().unwrap();
        let duration = *trajectory.t.last().unwrap();
        assert_vector_close(trajectory.x[0], setup.sigma_start, 1e-12, "start");
        assert_vector_close(
            *trajectory.x.last().unwrap(),
            setup.sigma_goal,
            1e-12,
            "goal",
        );

        let after = (duration + 10.0) * 1e9;
        module.update(&context(after as u64));
        let output = module.att_ref_out_msg.read();
        assert_vector_close(output.sigma_rn, setup.sigma_goal, 1e-12, "held goal");
        assert_eq!(output.omega_rn_n_radps, Vector3::zeros());
        assert!(module.path_cost() > 0.0);
    }

    /// Not upstream: the least-squares fit also starts and ends at the path's
    /// end points, and an unplannable maneuver holds the initial attitude.
    #[test]
    fn least_squares_fit_and_planning_failure() {
        // A finer grid, for a path long enough to fit.
        let setup = setup(9, CostFunction::MinimumDistance);
        let (mut module, _planet) = wired_module(&setup);
        module.config.trajectory_fit = TrajectoryFit::LeastSquares;
        module.init();
        assert!(module.path().len() >= 4, "path {:?}", module.path());
        let trajectory = module.trajectory().unwrap();
        assert_vector_close(trajectory.x[0], setup.sigma_start, 1e-9, "start");
        assert_vector_close(
            *trajectory.x.last().unwrap(),
            setup.sigma_goal,
            1e-9,
            "goal",
        );

        // A keep-in instrument that can never see its body: no free node.
        module.config.keep_in_boresights.clear();
        module.config.append_keep_in_direction(Vector3::z(), 0.0);
        module.config.keep_out_boresights.clear();
        module
            .config
            .append_keep_out_direction(Vector3::z(), std::f64::consts::PI);
        module.init();
        assert!(module.trajectory().is_none());
        assert!(module.path_cost().is_nan());
        assert!(module.path().is_empty());
        module.update(&context(1_000_000_000));
        let output = module.att_ref_out_msg.read();
        assert_eq!(output.sigma_rn, setup.sigma_start);
        assert_eq!(output.omega_rn_n_radps, Vector3::zeros());
    }

    /// Not upstream: a goal on the start's grid node, or on a neighbouring
    /// one (a two-node path, which the degree-4 fit cannot follow), cannot be
    /// planned; the initial attitude is held.
    #[test]
    fn goal_at_or_next_to_the_start_holds_the_initial_attitude() {
        for goal in [Vector3::new(0.0, 0.0, -0.5), Vector3::new(0.0, 0.25, -0.25)] {
            let mut module = ConstrainedAttitudeManeuver::new(ConstrainedAttitudeManeuverConfig {
                sigma_bn_goal: goal,
                avg_omega_radps: 0.03,
                ..Default::default()
            });
            let sc_state = Output::new(SpacecraftStateMsg {
                sigma_bn: Vector3::new(0.0, 0.0, -0.5),
                ..Default::default()
            });
            sc_state.connect_to(&mut module.sc_state_in_msg);
            module.init();
            assert!(module.trajectory().is_none(), "goal {goal}");
            module.update(&context(0));
            assert_eq!(
                module.att_ref_out_msg.read().sigma_rn,
                Vector3::new(0.0, 0.0, -0.5)
            );
        }
    }
}
