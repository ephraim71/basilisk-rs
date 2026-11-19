//! The truth of `test_ConstrainedAttitudeManeuver.py`: its Python grid,
//! A*, path handling and effort code, reimplemented as the pytest writes it
//! (an insertion-ordered node dictionary, Python's stable sort, its own
//! constraint check and distance) so the module is checked against an
//! independent implementation. Like the pytest, it fits the trajectory with
//! the shared B-spline utility.

use std::collections::HashMap;

use nalgebra::{Matrix3, Vector3};

use super::CostFunction;
use super::mrp_grid::NodeKey;
use crate::utilities::bspline::{BSplineInput, BSplineOutput, WaypointTiming, interpolate};
use crate::utilities::rigid_body_kinematics::{
    d_mrp_to_omega, d_omega_to_mrp, dd_mrp_to_domega, mrp_switch,
};

/// The pytest's inputs.
pub(super) struct TruthSetup {
    pub n: usize,
    pub cost_function: CostFunction,
    pub inertia: Matrix3<f64>,
    pub planet_position: Vector3<f64>,
    pub sc_position: Vector3<f64>,
    pub sigma_start: Vector3<f64>,
    pub sigma_goal: Vector3<f64>,
    pub omega_start: Vector3<f64>,
    pub omega_goal: Vector3<f64>,
    pub avg_omega: f64,
    pub keep_out_boresights: Vec<Vector3<f64>>,
    pub keep_in_boresights: Vec<Vector3<f64>>,
    pub keep_out_fov: f64,
    pub keep_in_fov: f64,
}

/// The pytest's `node` class.
#[derive(Clone, Debug)]
pub(super) struct PyNode {
    pub sigma: Vector3<f64>,
    pub is_boundary: bool,
    pub is_free: bool,
    /// The `neighbors` dictionary's keys, in insertion order (lookups go
    /// through `nodes[key]`, so the values do not matter).
    neighbors: Vec<NodeKey>,
    heuristic: f64,
    priority: f64,
    back_pointer: Option<NodeKey>,
}

/// The pytest's `constraint` objects and `data` keyword arguments.
struct PyConstraints {
    keep_out_axis: Vector3<f64>,
    keep_in_axis: Vector3<f64>,
    keep_out_b: Vec<Vector3<f64>>,
    keep_out_fov: Vec<f64>,
    keep_in_b: Vec<Vector3<f64>>,
    keep_in_fov: Vec<f64>,
}

/// `shadowSetMap(sigma, True)`.
fn shadow_set_map(sigma: Vector3<f64>) -> Vector3<f64> {
    let s2 = sigma.norm_squared();
    if s2 != 0.0 { -sigma / s2 } else { sigma }
}

impl PyNode {
    /// `node(sigma_BN, constraints, **data)`.
    fn new(sigma_bn: Vector3<f64>, constraints: &PyConstraints) -> Self {
        let mut s = sigma_bn.norm();
        let sigma = if s > 1.0 {
            shadow_set_map(sigma_bn)
        } else {
            sigma_bn
        };
        if (s - 1.0).abs() < 1e-5 {
            s = 1.0;
        }
        let mut node = Self {
            sigma,
            is_boundary: s == 1.0,
            is_free: true,
            neighbors: Vec::new(),
            heuristic: 0.0,
            priority: 0.0,
            back_pointer: None,
        };
        // The DCM from the argument (not the switched set) and the rounded
        // norm, as the pytest builds it.
        let tilde = Matrix3::new(
            0.0,
            -sigma_bn[2],
            sigma_bn[1],
            sigma_bn[2],
            0.0,
            -sigma_bn[0],
            -sigma_bn[1],
            sigma_bn[0],
            0.0,
        );
        let bn = Matrix3::identity()
            + (8.0 * tilde * tilde - 4.0 * (1.0 - s.powi(2)) * tilde) / (1.0 + s.powi(2)).powi(2);
        let nb = bn.transpose();
        for (b_b, fov) in constraints.keep_out_b.iter().zip(&constraints.keep_out_fov) {
            if (nb * b_b).dot(&constraints.keep_out_axis) >= fov.cos() {
                node.is_free = false;
                return node;
            }
        }
        let is_in = constraints
            .keep_in_b
            .iter()
            .zip(&constraints.keep_in_fov)
            .any(|(b_b, fov)| (nb * b_b).dot(&constraints.keep_in_axis) >= fov.cos());
        if !is_in {
            node.is_free = false;
        }
        node
    }
}

/// `distanceCart(n1, n2)`.
fn distance_cart(s1: &Vector3<f64>, s2: &Vector3<f64>) -> f64 {
    let d1 = (s1 - s2).norm();
    let n1 = s1.norm_squared();
    let n2 = s2.norm_squared();
    let s2_ss = shadow_set_map(*s2);
    let s1_ss = shadow_set_map(*s1);
    let d2 = if n2 > 1e-8 { (s1 - s2_ss).norm() } else { d1 };
    let d3 = if n1 > 1e-8 { (s1_ss - s2).norm() } else { d1 };
    let d4 = if n1 > 1e-8 && n2 > 1e-8 {
        (s1_ss - s2_ss).norm()
    } else {
        d1
    };
    // Python's `min` keeps the first of equal values.
    [d2, d3, d4]
        .into_iter()
        .fold(d1, |best, d| if d < best { d } else { best })
}

/// `mirrorFunction(i, j, k)`.
fn mirror_function([i, j, k]: NodeKey) -> [NodeKey; 8] {
    [
        [i, j, k],
        [-i, j, k],
        [i, -j, k],
        [i, j, -k],
        [-i, -j, k],
        [-i, j, -k],
        [i, -j, -k],
        [-i, -j, -k],
    ]
}

/// `neighboringNodes(i, j, k)`.
fn neighboring_nodes([i, j, k]: NodeKey) -> [NodeKey; 26] {
    [
        [i - 1, j, k],
        [i + 1, j, k],
        [i, j - 1, k],
        [i, j + 1, k],
        [i, j, k - 1],
        [i, j, k + 1],
        [i - 1, j - 1, k],
        [i + 1, j - 1, k],
        [i - 1, j + 1, k],
        [i + 1, j + 1, k],
        [i - 1, j, k - 1],
        [i + 1, j, k - 1],
        [i - 1, j, k + 1],
        [i + 1, j, k + 1],
        [i, j - 1, k - 1],
        [i, j + 1, k - 1],
        [i, j - 1, k + 1],
        [i, j + 1, k + 1],
        [i - 1, j - 1, k - 1],
        [i + 1, j - 1, k - 1],
        [i - 1, j + 1, k - 1],
        [i - 1, j - 1, k + 1],
        [i + 1, j + 1, k - 1],
        [i + 1, j - 1, k + 1],
        [i - 1, j + 1, k + 1],
        [i + 1, j + 1, k + 1],
    ]
}

/// `np.sign(m) * u`.
fn sign(m: i32) -> f64 {
    f64::from(m.signum())
}

/// The pytest's `nodes` dictionary.
#[derive(Default)]
struct PyNodes {
    order: Vec<NodeKey>,
    map: HashMap<NodeKey, PyNode>,
}

impl PyNodes {
    fn contains(&self, key: &NodeKey) -> bool {
        self.map.contains_key(key)
    }

    /// `nodes[key] = node`, keeping an existing key's position.
    fn set(&mut self, key: NodeKey, node: PyNode) {
        if self.map.insert(key, node).is_none() {
            self.order.push(key);
        }
    }

    fn get(&self, key: &NodeKey) -> &PyNode {
        &self.map[key]
    }

    fn get_mut(&mut self, key: &NodeKey) -> &mut PyNode {
        self.map.get_mut(key).expect("key in nodes")
    }
}

/// `generateGrid(n_start, n_goal, N, constraints, data)`: the nodes and the
/// keys of the start and goal.
fn generate_grid(
    n_start: PyNode,
    n_goal: PyNode,
    n: usize,
    constraints: &PyConstraints,
) -> (PyNodes, NodeKey, NodeKey) {
    // `np.linspace(0, 1, N)`: `i * step`, the last value exactly 1.
    let step = 1.0 / (n as f64 - 1.0);
    let u: Vec<f64> = (0..n)
        .map(|i| if i == n - 1 { 1.0 } else { i as f64 * step })
        .collect();
    let mut nodes = PyNodes::default();

    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                if u[i].powi(2) + u[j].powi(2) + u[k].powi(2) <= 1.0 {
                    for m in mirror_function([i as i32, j as i32, k as i32]) {
                        if !nodes.contains(&m) {
                            let sigma = Vector3::new(
                                sign(m[0]) * u[i],
                                sign(m[1]) * u[j],
                                sign(m[2]) * u[k],
                            );
                            nodes.set(m, PyNode::new(sigma, constraints));
                        }
                    }
                }
            }
        }
    }
    for i in 0..n - 1 {
        for j in 0..n - 1 {
            for k in 0..n - 1 {
                let key = [i as i32, j as i32, k as i32];
                if !nodes.contains(&key) || nodes.get(&key).is_boundary {
                    continue;
                }
                let [ki, kj, kk] = key;
                if !nodes.contains(&[ki + 1, kj, kk]) {
                    for m in mirror_function([ki + 1, kj, kk]) {
                        if !nodes.contains(&m) {
                            let sigma = Vector3::new(
                                sign(m[0]) * (1.0 - u[j].powi(2) - u[k].powi(2)).powf(0.5),
                                sign(m[1]) * u[j],
                                sign(m[2]) * u[k],
                            );
                            nodes.set(m, PyNode::new(sigma, constraints));
                        }
                    }
                }
                if !nodes.contains(&[ki, kj + 1, kk]) {
                    for m in mirror_function([ki, kj + 1, kk]) {
                        if !nodes.contains(&m) {
                            let sigma = Vector3::new(
                                sign(m[0]) * u[i],
                                sign(m[1]) * (1.0 - u[i].powi(2) - u[k].powi(2)).powf(0.5),
                                sign(m[2]) * u[k],
                            );
                            nodes.set(m, PyNode::new(sigma, constraints));
                        }
                    }
                }
                if !nodes.contains(&[ki, kj, kk + 1]) {
                    for m in mirror_function([ki, kj, kk + 1]) {
                        if !nodes.contains(&m) {
                            let sigma = Vector3::new(
                                sign(m[0]) * u[i],
                                sign(m[1]) * u[j],
                                sign(m[2]) * (1.0 - u[i].powi(2) - u[j].powi(2)).powf(0.5),
                            );
                            nodes.set(m, PyNode::new(sigma, constraints));
                        }
                    }
                }
            }
        }
    }

    // Link nodes, in dictionary order: immediate neighbours, then (for
    // boundary nodes) the antipode's neighbours as linked so far.
    for key1 in nodes.order.clone() {
        for key2 in neighboring_nodes(key1) {
            if nodes.contains(&key2) && nodes.get(&key1).is_free && nodes.get(&key2).is_free {
                let neighbors = &mut nodes.get_mut(&key1).neighbors;
                if !neighbors.contains(&key2) {
                    neighbors.push(key2);
                }
            }
        }
        if nodes.get(&key1).is_boundary {
            let antipode = key1.map(|i| -i);
            if nodes.contains(&antipode) {
                for key2 in nodes.get(&antipode).neighbors.clone() {
                    if nodes.get(&key1).is_free
                        && nodes.get(&key2).is_free
                        && !nodes.get(&key1).neighbors.contains(&key2)
                    {
                        nodes.get_mut(&key1).neighbors.push(key2);
                    }
                }
            }
        }
    }

    // The closest free nodes to the start and the goal.
    let mut ds = 10.0;
    let mut dg = 10.0;
    let mut key_s = None;
    let mut key_g = None;
    for key in nodes.order.clone() {
        let node = nodes.get(&key);
        if !node.is_free {
            continue;
        }
        let d1 = distance_cart(&node.sigma, &n_start.sigma);
        if (d1 - ds).abs() < 1e-6 {
            if node.sigma.norm() < nodes.get(&key_s.unwrap()).sigma.norm() {
                ds = d1;
                key_s = Some(key);
            }
        } else if d1 < ds {
            ds = d1;
            key_s = Some(key);
        }
        let d2 = distance_cart(&node.sigma, &n_goal.sigma);
        if (d2 - dg).abs() < 1e-6 {
            if node.sigma.norm() < nodes.get(&key_g.unwrap()).sigma.norm() {
                dg = d2;
                key_g = Some(key);
            }
        } else if d2 < dg {
            dg = d2;
            key_g = Some(key);
        }
    }
    let (key_s, key_g) = (key_s.unwrap(), key_g.unwrap());
    let mut n_start = n_start;
    n_start.neighbors = nodes.get(&key_s).neighbors.clone();
    for key in nodes.get(&key_g).neighbors.clone() {
        let neighbors = &mut nodes.get_mut(&key).neighbors;
        if !neighbors.contains(&key_g) {
            neighbors.push(key_g);
        }
    }
    nodes.set(key_s, n_start);
    nodes.set(key_g, n_goal);
    (nodes, key_s, key_g)
}

/// `backtrack(n, n_start)`.
fn backtrack(nodes: &PyNodes, key: NodeKey, key_s: NodeKey) -> Vec<NodeKey> {
    if key == key_s {
        vec![key]
    } else {
        let mut path = backtrack(nodes, nodes.get(&key).back_pointer.unwrap(), key_s);
        path.push(key);
        path
    }
}

/// `pathHandle(path, avgOmega)`.
fn path_handle(path: &[Vector3<f64>], avg_omega: f64) -> BSplineInput {
    let mut t = vec![0.0];
    let mut s = 0.0;
    for n in 0..path.len() - 1 {
        t.push(t[n] + distance_cart(&path[n], &path[n + 1]));
        s += t[n + 1] - t[n];
    }
    let mut x: Vec<Vector3<f64>> = Vec::new();
    let mut shadow_set = false;
    for n in 0..path.len() - 1 {
        let sigma = if !shadow_set {
            path[n]
        } else {
            if path[n].norm() <= 1e-6 {
                for waypoint in &mut x {
                    // The pytest divides by the norm, not its square.
                    let s2 = waypoint.norm();
                    *waypoint = -*waypoint / s2;
                }
                shadow_set = !shadow_set;
            }
            mrp_switch(path[n], 0.0)
        };
        if (path[n + 1] - path[n]).norm() > 1.0 {
            shadow_set = !shadow_set;
        }
        x.push(sigma);
    }
    let last = path[path.len() - 1];
    x.push(if shadow_set {
        mrp_switch(last, 0.0)
    } else {
        last
    });

    let t_last = t[t.len() - 1];
    let mut input = BSplineInput::new(x);
    input.timing = WaypointTiming::TimeTags(
        t.iter()
            .map(|tn| tn * 4.0 * s / (t_last * avg_omega))
            .collect(),
    );
    input
}

/// `spline(Input, omegaS, omegaG)`.
fn spline(mut input: BSplineInput, omega_s: Vector3<f64>, omega_g: Vector3<f64>) -> BSplineOutput {
    let sigma_s = input.waypoints[0];
    let sigma_g = input.waypoints[input.waypoints.len() - 1];
    input.x_dot_0 = Some(d_omega_to_mrp(sigma_s, omega_s));
    input.x_dot_n = Some(d_omega_to_mrp(sigma_g, omega_g));
    interpolate(&input, 100, 4)
}

/// `computeTorque(sigma, sigmaDot, sigmaDDot, I)`.
pub(super) fn compute_torque(
    sigma: Vector3<f64>,
    sigma_dot: Vector3<f64>,
    sigma_ddot: Vector3<f64>,
    inertia: &Matrix3<f64>,
) -> Vector3<f64> {
    let omega = d_mrp_to_omega(sigma, sigma_dot);
    let omega_dot = dd_mrp_to_domega(sigma, sigma_dot, sigma_ddot);
    inertia * omega_dot + omega.cross(&(inertia * omega))
}

/// `effortEvaluation(Output, I)`.
fn effort_evaluation(output: &BSplineOutput, inertia: &Matrix3<f64>) -> f64 {
    let mut effort = 0.0;
    let mut l_a = compute_torque(output.x[0], output.x_dot[0], output.x_ddot[0], inertia);
    for n in 0..output.t.len() - 1 {
        let l_b = compute_torque(
            output.x[n + 1],
            output.x_dot[n + 1],
            output.x_ddot[n + 1],
            inertia,
        );
        effort += (l_a.norm() + l_b.norm()) * (output.t[n + 1] - output.t[n]) / 2.0;
        l_a = l_b;
    }
    effort
}

/// `AStar` and `effortBasedAStar`: they differ only in the priority.
fn a_star(
    nodes: &mut PyNodes,
    key_s: NodeKey,
    key_g: NodeKey,
    mut priority: impl FnMut(&PyNodes, NodeKey, NodeKey) -> f64,
) -> Vec<NodeKey> {
    let mut open = vec![key_s];
    let mut closed: Vec<NodeKey> = Vec::new();
    let mut n = 0;
    while open[0] != key_g && n < 10000 {
        n += 1;
        let current = open[0];
        closed.push(current);
        for key in nodes.get(&current).neighbors.clone() {
            if closed.contains(&key) {
                continue;
            }
            let p = priority(nodes, current, key);
            if open.contains(&key) {
                if p < nodes.get(&key).priority {
                    let node = nodes.get_mut(&key);
                    node.priority = p;
                    node.back_pointer = Some(current);
                }
            } else {
                let node = nodes.get_mut(&key);
                node.priority = p;
                node.back_pointer = Some(current);
                open.push(key);
            }
        }
        open.remove(0);
        assert!(!open.is_empty(), "Dead end");
        // Python's `list.sort` is stable.
        open.sort_by(|a, b| {
            nodes
                .get(a)
                .priority
                .partial_cmp(&nodes.get(b).priority)
                .unwrap()
        });
    }
    backtrack(nodes, open[0], key_s)
}

/// The pytest's truth.
pub(super) struct Truth {
    nodes: PyNodes,
    /// `path`'s attitudes.
    pub path: Vec<Vector3<f64>>,
    /// `Output`.
    pub trajectory: BSplineOutput,
    /// `pathCost`.
    pub path_cost: f64,
}

impl Truth {
    /// `nodes[(i, j, k)]`, if present.
    pub fn node(&self, key: NodeKey) -> Option<&PyNode> {
        self.nodes.map.get(&key)
    }
}

/// `CAMTestFunction`'s Python half.
pub(super) fn plan(setup: &TruthSetup) -> Truth {
    let axis = (setup.planet_position - setup.sc_position).normalize();
    let constraints = PyConstraints {
        keep_out_axis: axis,
        keep_in_axis: axis,
        keep_out_b: setup.keep_out_boresights.clone(),
        keep_out_fov: vec![setup.keep_out_fov],
        keep_in_b: setup.keep_in_boresights.clone(),
        keep_in_fov: vec![setup.keep_in_fov; setup.keep_in_boresights.len()],
    };
    let n_start = PyNode::new(setup.sigma_start, &constraints);
    let n_goal = PyNode::new(setup.sigma_goal, &constraints);
    let (mut nodes, key_s, key_g) = generate_grid(n_start, n_goal, setup.n, &constraints);
    let path_keys = match setup.cost_function {
        CostFunction::MinimumDistance => {
            let goal_sigma = nodes.get(&key_g).sigma;
            for node in nodes.map.values_mut() {
                node.heuristic = distance_cart(&node.sigma, &goal_sigma);
            }
            a_star(&mut nodes, key_s, key_g, |nodes, current, key| {
                let (current, key) = (nodes.get(&current), nodes.get(&key));
                key.heuristic + distance_cart(&current.sigma, &key.sigma) + current.priority
                    - current.heuristic
            })
        }
        CostFunction::MinimumEffort => a_star(&mut nodes, key_s, key_g, |nodes, current, key| {
            let mut path: Vec<Vector3<f64>> = backtrack(nodes, current, key_s)
                .iter()
                .map(|k| nodes.get(k).sigma)
                .collect();
            path.push(nodes.get(&key).sigma);
            if key != key_g {
                path.push(nodes.get(&key_g).sigma);
            }
            let input = path_handle(&path, setup.avg_omega);
            let output = spline(input, setup.omega_start, setup.omega_goal);
            effort_evaluation(&output, &setup.inertia)
        }),
    };
    let path: Vec<Vector3<f64>> = path_keys.iter().map(|k| nodes.get(k).sigma).collect();
    let input = path_handle(&path, setup.avg_omega);
    let trajectory = spline(input, setup.omega_start, setup.omega_goal);
    let path_cost = effort_evaluation(&trajectory, &setup.inertia);
    Truth {
        nodes,
        path,
        trajectory,
        path_cost,
    }
}
