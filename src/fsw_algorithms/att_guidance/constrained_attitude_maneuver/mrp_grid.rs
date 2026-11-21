//! The MRP grid and the A* searches of `constrainedAttitudeManeuver`
//! (upstream `Node`, `NodeList`, `GenerateGrid`, `AStar`,
//! `effortBasedAStar`, `backtrack`, `distance`, `mirrorFunction` and
//! `neighboringNodes`).
//!
//! The grid discretizes the ball `|σ| <= 1` of MRP space: integer keys
//! `(i, j, k)` with `|i|, |j|, |k| < N` map to `σ = (±u_i, ±u_j, ±u_k)`,
//! `u_n = n / (N - 1)`, inside the unit sphere, completed by points on it
//! (`|σ| = 1`, where every MRP set meets its shadow set). A node is free when
//! its attitude meets the keep-out and keep-in constraints; free nodes are
//! linked to their free immediate neighbours (26-connectivity), and boundary
//! nodes also to the neighbours of their antipodal node, which is the same
//! attitude. The start and goal attitudes replace the free grid nodes
//! closest to them.
//!
//! Upstream keeps the nodes in a nested `std::map` and links them by
//! pointer; here they live in a vector, linked by index, with the keys in a
//! `BTreeMap`, which iterates in the same `(i, j, k)` order.

use std::collections::BTreeMap;

use nalgebra::Vector3;

use crate::utilities::rigid_body_kinematics::{mrp_shadow, mrp_to_dcm};

/// A grid key `(i, j, k)` (upstream's `NodesMap` indices).
pub type NodeKey = [i32; 3];

/// Upstream `neighboringNodes`: the offsets of the 26 immediate neighbours,
/// in upstream's order (which decides ties in the searches).
const NEIGHBOR_OFFSETS: [NodeKey; 26] = [
    [-1, 0, 0],
    [1, 0, 0],
    [0, -1, 0],
    [0, 1, 0],
    [0, 0, -1],
    [0, 0, 1],
    [-1, -1, 0],
    [1, -1, 0],
    [-1, 1, 0],
    [1, 1, 0],
    [-1, 0, -1],
    [1, 0, -1],
    [-1, 0, 1],
    [1, 0, 1],
    [0, -1, -1],
    [0, 1, -1],
    [0, -1, 1],
    [0, 1, 1],
    [-1, -1, -1],
    [1, -1, -1],
    [-1, 1, -1],
    [-1, -1, 1],
    [1, 1, -1],
    [1, -1, 1],
    [-1, 1, 1],
    [1, 1, 1],
];

/// Upstream `mirrorFunction`: the eight sign combinations of a key.
fn mirrors([i, j, k]: NodeKey) -> [NodeKey; 8] {
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

/// An instrument boresight and its field of view.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Boresight {
    /// Unit boresight direction, body components.
    pub direction_b: Vector3<f64>,
    /// Half-angle field of view \[rad\].
    pub fov_rad: f64,
}

impl Boresight {
    /// The boresight along `direction_b` (normalized, as upstream's
    /// `appendKeepOutDirection`/`appendKeepInDirection`).
    pub fn new(direction_b: Vector3<f64>, fov_rad: f64) -> Self {
        Self {
            direction_b: crate::utilities::linear_algebra::normalize_or_zero(direction_b),
            fov_rad,
        }
    }
}

/// The attitude constraints (upstream `constraintStruct` and
/// `scBoresightStruct`).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Constraints {
    /// Upstream `keepOutDir_N`, present when `keepOut`: inertial unit
    /// direction to the keep-out body.
    pub keep_out_dir_n: Option<Vector3<f64>>,
    /// Upstream `keepInDir_N`, present when `keepIn`: inertial unit
    /// direction to the keep-in body.
    pub keep_in_dir_n: Option<Vector3<f64>>,
    /// Instruments that must not see the keep-out body.
    pub keep_out: Vec<Boresight>,
    /// Instruments of which at least one must see the keep-in body.
    pub keep_in: Vec<Boresight>,
}

impl Constraints {
    /// Upstream `Node`'s constraint check: no keep-out boresight within its
    /// field of view of the keep-out body, and some keep-in boresight within
    /// its field of view of the keep-in body.
    pub fn is_compliant(&self, sigma_bn: &Vector3<f64>) -> bool {
        let dcm_nb = mrp_to_dcm(*sigma_bn).transpose();
        let sees = |boresight: &Boresight, direction_n: &Vector3<f64>| {
            (dcm_nb * boresight.direction_b).dot(direction_n) >= boresight.fov_rad.cos()
        };
        if let Some(direction_n) = &self.keep_out_dir_n
            && self
                .keep_out
                .iter()
                .any(|boresight| sees(boresight, direction_n))
        {
            return false;
        }
        if let Some(direction_n) = &self.keep_in_dir_n {
            return self
                .keep_in
                .iter()
                .any(|boresight| sees(boresight, direction_n));
        }
        true
    }
}

/// A grid node (upstream `Node`).
#[derive(Clone, Debug, PartialEq)]
pub struct Node {
    /// Upstream `sigma_BN`: the node's attitude.
    pub sigma_bn: Vector3<f64>,
    /// Upstream `isBoundary`: the node lies on `|σ| = 1`.
    pub is_boundary: bool,
    /// Upstream `isFree`: the attitude meets the constraints.
    pub is_free: bool,
    /// Upstream `heuristic`: distance to the goal (distance-based A*).
    pub heuristic: f64,
    /// Upstream `priority`: search priority.
    pub priority: f64,
    /// Upstream `neighbors`: indices of the linked nodes.
    pub neighbors: Vec<usize>,
    /// Upstream `backPointer`: the previous node of the best path found.
    pub back_pointer: Option<usize>,
}

impl Node {
    /// Upstream `Node(sigma_BN, constraints, boresights)`.
    pub fn new(sigma_bn: Vector3<f64>, constraints: &Constraints) -> Self {
        Self {
            sigma_bn,
            is_boundary: (sigma_bn.norm() - 1.0).abs() < 1e-5,
            is_free: constraints.is_compliant(&sigma_bn),
            heuristic: 0.0,
            priority: 0.0,
            neighbors: Vec::new(),
            back_pointer: None,
        }
    }
}

/// Upstream `distance`: the Cartesian distance between two MRP sets, the
/// smallest over both sets of each (a set within `1e-8` of zero has no
/// usable shadow set).
pub fn mrp_distance(sigma_1: &Vector3<f64>, sigma_2: &Vector3<f64>) -> f64 {
    let has_shadow_1 = sigma_1.norm() > 1e-8;
    let has_shadow_2 = sigma_2.norm() > 1e-8;
    let direct = (sigma_1 - sigma_2).norm();
    let shadow_1 = has_shadow_1.then(|| mrp_shadow(*sigma_1));
    let shadow_2 = has_shadow_2.then(|| mrp_shadow(*sigma_2));
    let candidates = [
        shadow_2.map_or(direct, |s2| (sigma_1 - s2).norm()),
        shadow_1.map_or(direct, |s1| (sigma_2 - s1).norm()),
        shadow_1
            .zip(shadow_2)
            .map_or(direct, |(s1, s2)| (s1 - s2).norm()),
    ];
    candidates
        .into_iter()
        .fold(direct, |best, d| if d < best { d } else { best })
}

/// Why no path could be planned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlanningError {
    /// The grid needs `N >= 2`.
    GridTooCoarse,
    /// No grid node meets the constraints.
    NoFreeNode,
    /// The search ran out of nodes to explore before reaching the goal.
    DeadEnd,
    /// The start and goal attitudes are on the same grid node.
    StartIsGoal,
    /// The trajectory fitted to the path is not finite.
    DegenerateTrajectory,
}

impl std::fmt::Display for PlanningError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::GridTooCoarse => "the grid fineness N must be at least 2",
            Self::NoFreeNode => "no grid node meets the constraints",
            Self::DeadEnd => "the graph search reached a dead end",
            Self::StartIsGoal => "the start and goal attitudes are on the same grid node",
            Self::DegenerateTrajectory => "the trajectory fitted to the path is not finite",
        })
    }
}

/// The linked MRP grid with the start and goal nodes in place.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MrpGrid {
    /// The nodes, linked by index.
    pub nodes: Vec<Node>,
    /// Upstream `NodesMap`: key to node index, in upstream's iteration order.
    keys: BTreeMap<NodeKey, usize>,
    /// Upstream `keyS`: index of the start node.
    pub start: usize,
    /// Upstream `keyG`: index of the goal node.
    pub goal: usize,
}

impl MrpGrid {
    /// Upstream `GenerateGrid`: the grid of fineness `n`, linked, with
    /// `start` and `goal` replacing their closest free nodes.
    ///
    /// Upstream gives the goal's grid neighbours a link to a local copy of
    /// the goal that does not outlive the call; here they link to the goal
    /// node itself, when not linked already.
    pub fn generate(
        n: usize,
        mut start: Node,
        goal: Node,
        constraints: &Constraints,
    ) -> Result<Self, PlanningError> {
        if n < 2 {
            return Err(PlanningError::GridTooCoarse);
        }
        let u: Vec<f64> = (0..n).map(|i| i as f64 / (n as f64 - 1.0)).collect();
        let mut grid = Self::default();

        // Nodes inside the unit sphere, with all their mirror images.
        for i in 0..n {
            for j in 0..n {
                for k in 0..n {
                    if u[i].powi(2) + u[j].powi(2) + u[k].powi(2) <= 1.0 {
                        let key = [i, j, k].map(|index| index as i32);
                        for mirror in mirrors(key) {
                            let sigma =
                                Vector3::from_fn(|p, _| signed(mirror[p], u[key[p] as usize]));
                            grid.insert(mirror, sigma, constraints);
                        }
                    }
                }
            }
        }

        // Nodes on the unit sphere next to the interior ones.
        for i in 0..n - 1 {
            for j in 0..n - 1 {
                for k in 0..n - 1 {
                    let key = [i, j, k].map(|index| index as i32);
                    let Some(&index) = grid.keys.get(&key) else {
                        continue;
                    };
                    if grid.nodes[index].is_boundary {
                        continue;
                    }
                    for axis in 0..3 {
                        let mut outer = key;
                        outer[axis] += 1;
                        if grid.keys.contains_key(&outer) {
                            continue;
                        }
                        let others: f64 = (0..3)
                            .filter(|&p| p != axis)
                            .map(|p| u[key[p] as usize].powi(2))
                            .fold(1.0, |rest, square| rest - square);
                        for mirror in mirrors(outer) {
                            let sigma = Vector3::from_fn(|p, _| {
                                if p == axis {
                                    f64::from(mirror[p].signum()) * others.sqrt()
                                } else {
                                    signed(mirror[p], u[outer[p] as usize])
                                }
                            });
                            grid.insert(mirror, sigma, constraints);
                        }
                    }
                }
            }
        }

        grid.link_neighbors();
        let (start_key, goal_key) = grid.closest_free_keys(&start, &goal)?;
        let start_index = grid.index(&start, start_key)?;
        let goal_index = grid.index(&goal, goal_key)?;

        start.neighbors = grid.nodes[start_index].neighbors.clone();
        for neighbor in grid.nodes[goal_index].neighbors.clone() {
            if !grid.nodes[neighbor].neighbors.contains(&goal_index) {
                grid.nodes[neighbor].neighbors.push(goal_index);
            }
        }
        grid.nodes[start_index] = start;
        grid.nodes[goal_index] = goal;
        grid.start = start_index;
        grid.goal = goal_index;
        Ok(grid)
    }

    /// Adds the node at `key` unless the grid has one.
    fn insert(&mut self, key: NodeKey, sigma_bn: Vector3<f64>, constraints: &Constraints) {
        if !self.keys.contains_key(&key) {
            self.keys.insert(key, self.nodes.len());
            self.nodes.push(Node::new(sigma_bn, constraints));
        }
    }

    /// Links every free node to its free immediate neighbours, then every
    /// free boundary node to the neighbours of its antipodal node.
    fn link_neighbors(&mut self) {
        for (key, &index) in &self.keys {
            for offset in NEIGHBOR_OFFSETS {
                let neighbor_key = [key[0] + offset[0], key[1] + offset[1], key[2] + offset[2]];
                if let Some(&neighbor) = self.keys.get(&neighbor_key)
                    && self.nodes[index].is_free
                    && self.nodes[neighbor].is_free
                {
                    self.nodes[index].neighbors.push(neighbor);
                }
            }
        }
        for (key, &index) in &self.keys {
            if !(self.nodes[index].is_boundary && self.nodes[index].is_free) {
                continue;
            }
            let Some(&antipode) = self.keys.get(&key.map(|i| -i)) else {
                continue;
            };
            for neighbor in self.nodes[antipode].neighbors.clone() {
                if !self.nodes[index].neighbors.contains(&neighbor) {
                    self.nodes[index].neighbors.push(neighbor);
                }
            }
        }
    }

    /// The keys of the free nodes closest to `start` and `goal`; of nodes
    /// within `1e-6` of the same distance the one with the smaller `|σ|`.
    fn closest_free_keys(
        &self,
        start: &Node,
        goal: &Node,
    ) -> Result<(NodeKey, NodeKey), PlanningError> {
        let mut best: [Option<(f64, NodeKey)>; 2] = [None, None];
        for (key, &index) in &self.keys {
            let node = &self.nodes[index];
            if !node.is_free {
                continue;
            }
            for (target, best) in [start, goal].into_iter().zip(&mut best) {
                let d = mrp_distance(&target.sigma_bn, &node.sigma_bn);
                // Upstream starts from a distance of 10.
                let (best_d, best_key) = best.unwrap_or((10.0, *key));
                let closer = if (d - best_d).abs() < 1e-6 {
                    best.is_some()
                        && node.sigma_bn.norm() < self.nodes[self.keys[&best_key]].sigma_bn.norm()
                } else {
                    d < best_d
                };
                if closer {
                    *best = Some((d, *key));
                }
            }
        }
        match best {
            [Some((_, start_key)), Some((_, goal_key))] => Ok((start_key, goal_key)),
            _ => Err(PlanningError::NoFreeNode),
        }
    }

    /// The index of `key`, with its signs first matched to `node`'s attitude
    /// when that lies on the boundary.
    fn index(&self, node: &Node, mut key: NodeKey) -> Result<usize, PlanningError> {
        if node.is_boundary {
            for (index, sigma) in key.iter_mut().zip(node.sigma_bn.iter()) {
                if f64::from(*index) * sigma < 0.0 {
                    *index = -*index;
                }
            }
        }
        self.keys
            .get(&key)
            .copied()
            .ok_or(PlanningError::NoFreeNode)
    }

    /// The node at `key` (upstream `returnNodeCoord`/`returnNodeState`).
    pub fn node(&self, key: NodeKey) -> Option<&Node> {
        self.keys.get(&key).map(|&index| &self.nodes[index])
    }

    /// The index of the node at `key`.
    pub fn index_of(&self, key: NodeKey) -> Option<usize> {
        self.keys.get(&key).copied()
    }

    /// Upstream `backtrack`: the nodes from the start to `last` along the
    /// back pointers.
    pub fn backtrack(&self, last: usize) -> Vec<usize> {
        let mut path = vec![last];
        let mut node = last;
        while node != self.start {
            node = self.nodes[node]
                .back_pointer
                .expect("every node reached by the search has a back pointer");
            path.push(node);
        }
        path.reverse();
        path
    }

    /// Upstream `AStar`: the shortest path in MRP distance, exploring at most
    /// `max_iterations` nodes. If the goal is not reached in time the path
    /// ends at the most promising open node, as upstream.
    pub fn distance_a_star(&mut self, max_iterations: usize) -> Result<Vec<usize>, PlanningError> {
        let goal_sigma = self.nodes[self.goal].sigma_bn;
        for node in &mut self.nodes {
            node.heuristic = mrp_distance(&node.sigma_bn, &goal_sigma);
        }
        self.search(max_iterations, |grid, current, key| {
            let (current, key) = (&grid.nodes[current], &grid.nodes[key]);
            key.heuristic + mrp_distance(&current.sigma_bn, &key.sigma_bn) + current.priority
                - current.heuristic
        })
    }

    /// Upstream `effortBasedAStar`: the search with the priority of a node
    /// set to `cost` of the path through it, completed by a direct leg to
    /// the goal; explores at most `max_iterations` nodes.
    pub fn effort_a_star(
        &mut self,
        max_iterations: usize,
        mut cost: impl FnMut(&MrpGrid, &[usize]) -> f64,
    ) -> Result<Vec<usize>, PlanningError> {
        self.search(max_iterations, |grid, current, key| {
            let mut path = grid.backtrack(current);
            path.push(key);
            if key != grid.goal {
                path.push(grid.goal);
            }
            cost(grid, &path)
        })
    }

    /// The best-first search both upstream A* variants share: open list `O`
    /// re-sorted after each expansion, closed list `C`.
    fn search(
        &mut self,
        max_iterations: usize,
        mut priority: impl FnMut(&MrpGrid, usize, usize) -> f64,
    ) -> Result<Vec<usize>, PlanningError> {
        let mut open = vec![self.start];
        let mut closed = vec![false; self.nodes.len()];
        let mut iterations = 0;
        while open[0] != self.goal && iterations < max_iterations {
            iterations += 1;
            let current = open[0];
            closed[current] = true;
            for k in 0..self.nodes[current].neighbors.len() {
                let key = self.nodes[current].neighbors[k];
                if closed[key] {
                    continue;
                }
                let p = priority(self, current, key);
                let improves = !open.contains(&key) || p < self.nodes[key].priority;
                if improves {
                    self.nodes[key].priority = p;
                    self.nodes[key].back_pointer = Some(current);
                    if !open.contains(&key) {
                        open.push(key);
                    }
                }
            }
            open.remove(0);
            if open.is_empty() {
                return Err(PlanningError::DeadEnd);
            }
            sort_by_priority(&mut open, &self.nodes);
        }
        Ok(self.backtrack(open[0]))
    }
}

/// `sign(key) * u` as upstream writes it, `key / |key| * u`, and zero for a
/// zero key.
fn signed(key: i32, u: f64) -> f64 {
    if key == 0 {
        0.0
    } else {
        f64::from(key.signum()) * u
    }
}

/// Upstream `NodeList::sort`: a selection sort by priority. It is not
/// stable, and ties keep upstream's order. Priorities from 1e5 up are never
/// selected; upstream then swaps with the slot past the end of the list,
/// here the entry stays.
fn sort_by_priority(open: &mut [usize], nodes: &[Node]) {
    for n in 0..open.len() {
        let mut lowest = 1e5;
        let mut selected = None;
        for (m, &index) in open.iter().enumerate().skip(n) {
            if nodes[index].priority < lowest {
                lowest = nodes[index].priority;
                selected = Some(m);
            }
        }
        if let Some(m) = selected {
            open.swap(n, m);
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::{Constraints, MrpGrid, Node, mrp_distance, sort_by_priority};

    #[test]
    fn distance_takes_the_closest_mrp_sets() {
        let a = Vector3::new(0.9, 0.0, 0.0);
        let b = Vector3::new(-0.9, 0.0, 0.0);
        // b's shadow set is (1/0.9, 0, 0).
        assert!((mrp_distance(&a, &b) - (1.0 / 0.9 - 0.9)).abs() < 1e-15);
        assert_eq!(
            mrp_distance(&Vector3::zeros(), &Vector3::new(0.3, 0.4, 0.0)),
            0.5
        );
        assert_eq!(mrp_distance(&a, &a), 0.0);
    }

    #[test]
    fn unconstrained_grid_is_symmetric_and_fully_linked() {
        let constraints = Constraints::default();
        let start = Node::new(Vector3::new(0.0, 0.0, -0.5), &constraints);
        let goal = Node::new(Vector3::new(0.0, 0.5, 0.0), &constraints);
        let grid = MrpGrid::generate(5, start, goal, &constraints).unwrap();
        for (key, &index) in &grid.keys {
            let node = &grid.nodes[index];
            assert!(node.sigma_bn.norm() <= 1.0 + 1e-12);
            assert_eq!(grid.node(key.map(|i| -i)).unwrap().sigma_bn, -node.sigma_bn);
        }
        // The origin has all 26 neighbours.
        assert_eq!(grid.node([0, 0, 0]).unwrap().neighbors.len(), 26);
        assert_eq!(
            grid.nodes[grid.start].sigma_bn,
            Vector3::new(0.0, 0.0, -0.5)
        );
        assert_eq!(grid.index_of([0, 0, -2]), Some(grid.start));
        assert_eq!(grid.index_of([0, 2, 0]), Some(grid.goal));
        // Boundary nodes also reach the neighbours of their antipode.
        let boundary = grid.index_of([4, 0, 0]).unwrap();
        let antipode = grid.index_of([-4, 0, 0]).unwrap();
        for neighbor in &grid.nodes[antipode].neighbors {
            assert!(grid.nodes[boundary].neighbors.contains(neighbor));
        }
    }

    #[test]
    fn distance_search_without_constraints_goes_straight() {
        let constraints = Constraints::default();
        let start = Node::new(Vector3::new(0.0, 0.0, -0.5), &constraints);
        let goal = Node::new(Vector3::new(0.0, 0.5, 0.0), &constraints);
        let mut grid = MrpGrid::generate(5, start, goal, &constraints).unwrap();
        let path = grid.distance_a_star(100).unwrap();
        let sigmas: Vec<_> = path.iter().map(|&i| grid.nodes[i].sigma_bn).collect();
        assert_eq!(sigmas.first(), Some(&Vector3::new(0.0, 0.0, -0.5)));
        assert_eq!(sigmas.last(), Some(&Vector3::new(0.0, 0.5, 0.0)));
        // Two diagonal steps of 0.25 √2.
        assert_eq!(path.len(), 3);
    }

    #[test]
    fn selection_sort_orders_by_priority_and_skips_large_ones() {
        let constraints = Constraints::default();
        let mut nodes: Vec<Node> = (0..4)
            .map(|_| Node::new(Vector3::zeros(), &constraints))
            .collect();
        for (node, priority) in nodes.iter_mut().zip([3.0, 1.0, 2e5, 2.0]) {
            node.priority = priority;
        }
        let mut open = vec![0, 1, 2, 3];
        sort_by_priority(&mut open, &nodes);
        assert_eq!(open, vec![1, 3, 0, 2]);
    }
}
