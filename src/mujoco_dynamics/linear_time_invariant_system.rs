//! Linear time-invariant systems in a scene's dynamics task: port of
//! upstream Basilisk `simulation/mujocoDynamics/linearTimeInvariantSystem`
//! (`LinearTimeInvariantSystem`, `SingleActuatorLTI`, `ForceAtSiteLTI`).
//!
//! The system
//!
//! ```text
//! ẋ = A x + B u,    y = C x + D u
//! ```
//!
//! integrates its state `x` (registered as `x`, zero at every reset) with
//! the scene. Upstream's abstract base reads `u` and writes `y` through
//! virtual methods; here [`LinearTimeInvariantSystem`] is the matrices and
//! the equations, and each concrete model ([`SingleActuatorLti`],
//! [`ForceAtSiteLti`], [`OrbitalElementControl`](super::orbital_element_control::OrbitalElementControl))
//! owns one and its messages. A missing matrix is a zero block: the sizes
//! come from the matrices that are set (or, for the concrete models, the
//! fixed input and output sizes).
//!
//! Upstream logs a dimension mismatch at `Reset` and invalid second-order
//! parameters with `bskLog(BSK_ERROR)` and carries on; here they are logged
//! with `log::error!` the same way.

use nalgebra::{DMatrix, DVector};

use super::task::{ModelStateSpec, SceneModel, SceneView, TaskContext};
use crate::SimulationContext;
use crate::messages::{BodyForceCommandMsg, Input, Output, SingleActuatorMsg};

/// The matrices and equations of upstream `LinearTimeInvariantSystem`.
#[derive(Clone, Debug, PartialEq)]
pub struct LinearTimeInvariantSystem {
    a: DMatrix<f64>,
    b: DMatrix<f64>,
    c: DMatrix<f64>,
    d: DMatrix<f64>,
    x: DVector<f64>,
}

impl Default for LinearTimeInvariantSystem {
    fn default() -> Self {
        Self {
            a: DMatrix::zeros(0, 0),
            b: DMatrix::zeros(0, 0),
            c: DMatrix::zeros(0, 0),
            d: DMatrix::zeros(0, 0),
            x: DVector::zeros(0),
        }
    }
}

impl LinearTimeInvariantSystem {
    /// Upstream `getA`.
    pub fn a(&self) -> &DMatrix<f64> {
        &self.a
    }

    /// Upstream `setA`: the state matrix (state × state).
    pub fn set_a(&mut self, a: DMatrix<f64>) {
        self.a = a;
    }

    /// Upstream `getB`.
    pub fn b(&self) -> &DMatrix<f64> {
        &self.b
    }

    /// Upstream `setB`: the input matrix (state × input).
    pub fn set_b(&mut self, b: DMatrix<f64>) {
        self.b = b;
    }

    /// Upstream `getC`.
    pub fn c(&self) -> &DMatrix<f64> {
        &self.c
    }

    /// Upstream `setC`: the output matrix (output × state).
    pub fn set_c(&mut self, c: DMatrix<f64>) {
        self.c = c;
    }

    /// Upstream `getD`.
    pub fn d(&self) -> &DMatrix<f64> {
        &self.d
    }

    /// Upstream `setD`: the feedthrough matrix (output × input).
    pub fn set_d(&mut self, d: DMatrix<f64>) {
        self.d = d;
    }

    /// Upstream `configureSecondOrder(wn, zeta, k)`: the single-channel
    /// second-order system `ÿ + 2ζω ẏ + ω² y = k ω² u` with state
    /// `[y, ẏ]`.
    pub fn configure_second_order(&mut self, wn: f64, zeta: f64, k: f64) {
        if wn <= 0.0 {
            log::error!("configureSecondOrder: natural frequency wn must be positive.");
            return;
        }
        if zeta < 0.0 {
            log::error!("configureSecondOrder: damping ratio zeta must be non negative.");
            return;
        }
        let wn2 = wn * wn;
        self.a = DMatrix::from_row_slice(2, 2, &[0.0, 1.0, -wn2, -2.0 * zeta * wn]);
        self.b = DMatrix::from_row_slice(2, 1, &[0.0, k * wn2]);
        self.c = DMatrix::from_row_slice(1, 2, &[1.0, 0.0]);
        self.d = DMatrix::zeros(1, 1);
    }

    /// Upstream `configureSecondOrder(wn, zeta, k)` for vectors: `n`
    /// decoupled second-order channels, channel `i` with states `2i`
    /// (position) and `2i + 1` (rate).
    pub fn configure_second_order_mimo(&mut self, wn: &[f64], zeta: &[f64], k: &[f64]) {
        let n = wn.len();
        if n == 0 {
            log::error!(
                "LinearTimeInvariantSystem::configureSecondOrder(MIMO): input vectors must be \
                 non empty."
            );
            return;
        }
        if zeta.len() != n || k.len() != n {
            log::error!(
                "LinearTimeInvariantSystem::configureSecondOrder(MIMO): size mismatch. Expected \
                 wn, zeta, k to all have length {n}. Received lengths wn={}, zeta={}, k={}.",
                wn.len(),
                zeta.len(),
                k.len()
            );
            return;
        }
        for i in 0..n {
            if wn[i] <= 0.0 {
                log::error!(
                    "LinearTimeInvariantSystem::configureSecondOrder(MIMO): wn({i}) must be \
                     positive, but is {}.",
                    wn[i]
                );
                return;
            }
            if zeta[i] < 0.0 {
                log::error!(
                    "LinearTimeInvariantSystem::configureSecondOrder(MIMO): zeta({i}) must be \
                     non negative, but is {}.",
                    zeta[i]
                );
                return;
            }
        }
        self.a = DMatrix::zeros(2 * n, 2 * n);
        self.b = DMatrix::zeros(2 * n, n);
        self.c = DMatrix::zeros(n, 2 * n);
        self.d = DMatrix::zeros(n, n);
        for i in 0..n {
            let wn2 = wn[i] * wn[i];
            self.a[(2 * i, 2 * i + 1)] = 1.0;
            self.a[(2 * i + 1, 2 * i)] = -wn2;
            self.a[(2 * i + 1, 2 * i + 1)] = -2.0 * zeta[i] * wn[i];
            self.b[(2 * i + 1, i)] = k[i] * wn2;
            self.c[(i, 2 * i)] = 1.0;
        }
    }

    /// Upstream `getInputSize`: from `B`, else `D`.
    pub fn input_size(&self) -> usize {
        if self.b.ncols() > 0 {
            self.b.ncols()
        } else {
            self.d.ncols()
        }
    }

    /// Upstream `getStateSize`: from `A`, else `B`, else `C`.
    pub fn state_size(&self) -> usize {
        if self.a.ncols() > 0 {
            self.a.ncols()
        } else if self.b.nrows() > 0 {
            self.b.nrows()
        } else {
            self.c.ncols()
        }
    }

    /// Upstream `getOutputSize`: from `C`, else `D`.
    pub fn output_size(&self) -> usize {
        if self.c.nrows() > 0 {
            self.c.nrows()
        } else {
            self.d.nrows()
        }
    }

    /// Upstream `getX`: the committed state.
    pub fn x(&self) -> &DVector<f64> {
        &self.x
    }

    /// Upstream `registerStates`: the state `x`, zero (none for a
    /// state-less system).
    pub fn state_specs(&self) -> Vec<ModelStateSpec> {
        let size = self.state_size();
        if size == 0 {
            return Vec::new();
        }
        vec![ModelStateSpec::new("x", vec![0.0; size])]
    }

    /// Hands the committed state back (see [`SceneModel::load_states`]).
    pub fn load_states(&mut self, states: &[&[f64]]) {
        if let Some(state) = states.first() {
            self.x = DVector::from_column_slice(state);
        }
    }

    /// Upstream `Reset`'s dimension checks against the model's input and
    /// output sizes.
    pub fn check_dimensions(&self, input_size: usize, output_size: usize) {
        let state_size = self.state_size();
        for (name, matrix, rows, cols, expected) in [
            ("A", &self.a, state_size, state_size, "getStateSize()"),
            (
                "B",
                &self.b,
                state_size,
                input_size,
                "getStateSize() and getInputSize()",
            ),
            (
                "C",
                &self.c,
                output_size,
                state_size,
                "getOutputSize() and getStateSize()",
            ),
            (
                "D",
                &self.d,
                output_size,
                input_size,
                "getOutputSize() and getInputSize()",
            ),
        ] {
            if !matrix.is_empty() && (matrix.nrows() != rows || matrix.ncols() != cols) {
                log::error!(
                    "LinearTimeInvariantSystem::Reset: matrix {name} has inconsistent \
                     dimensions. Expected size [{rows} x {cols}] based on {expected}, but \
                     received [{} x {}].",
                    matrix.nrows(),
                    matrix.ncols()
                );
            }
        }
    }

    /// Upstream `UpdateState`: sets `ẋ` from the stage state and the input
    /// `u` (of `input_size` entries) and returns the output (of
    /// `output_size` entries).
    pub fn evaluate(
        &self,
        context: &mut TaskContext<'_>,
        u: &DVector<f64>,
        input_size: usize,
        output_size: usize,
    ) -> DVector<f64> {
        if input_size > 0 && u.len() != input_size {
            log::error!(
                "LinearTimeInvariantSystem::UpdateState: readInput returned vector with \
                 incorrect size."
            );
        }
        let state_size = self.state_size();
        let mut y = DVector::zeros(output_size);
        if state_size > 0 {
            let x = DVector::from_column_slice(context.state(0));
            let mut dx = DVector::zeros(state_size);
            if self.a.ncols() > 0 {
                dx += &self.a * &x;
            }
            if self.b.ncols() > 0 && input_size > 0 {
                dx += &self.b * u;
            }
            context.set_derivative(0, dx.as_slice());
            if self.c.ncols() > 0 {
                y += &self.c * &x;
            }
        }
        if !self.d.is_empty() && input_size > 0 {
            y += &self.d * u;
        }
        y
    }
}

/// Upstream `SingleActuatorLTI`: an LTI system from one actuator command to
/// another (for example actuator dynamics in front of a scene actuator).
#[derive(Debug, Default)]
pub struct SingleActuatorLti {
    /// Upstream `ModelTag`.
    pub name: String,
    /// The system matrices (upstream's `LinearTimeInvariantSystem` base).
    pub system: LinearTimeInvariantSystem,
    /// Upstream `inMsg`: `u`.
    pub in_msg: Input<SingleActuatorMsg>,
    /// Upstream `outMsg`: `y`.
    pub out_msg: Output<SingleActuatorMsg>,
}

/// Upstream `ForceAtSiteLTI`: an LTI system from one site force to another,
/// three inputs and outputs (`force_S`, here
/// [`BodyForceCommandMsg::force_request_body_n`], as the
/// [`MjForceActuator`](super::MjForceActuator) input reads it).
#[derive(Debug, Default)]
pub struct ForceAtSiteLti {
    /// Upstream `ModelTag`.
    pub name: String,
    /// The system matrices.
    pub system: LinearTimeInvariantSystem,
    /// Upstream `inMsg`: `u`.
    pub in_msg: Input<BodyForceCommandMsg>,
    /// Upstream `outMsg`: `y`.
    pub out_msg: Output<BodyForceCommandMsg>,
}

impl SingleActuatorLti {
    /// Upstream `getInputSize`: 1.
    pub fn input_size(&self) -> usize {
        1
    }

    /// Upstream `getOutputSize`: 1.
    pub fn output_size(&self) -> usize {
        1
    }
}

impl ForceAtSiteLti {
    /// Upstream `getInputSize`: 3.
    pub fn input_size(&self) -> usize {
        3
    }

    /// Upstream `getOutputSize`: 3.
    pub fn output_size(&self) -> usize {
        3
    }
}

impl SceneModel for SingleActuatorLti {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn state_specs(&self) -> Vec<ModelStateSpec> {
        self.system.state_specs()
    }

    fn load_states(&mut self, states: &[&[f64]]) {
        self.system.load_states(states);
    }

    fn scene_reset(&mut self, _scene: &SceneView<'_>, _context: &SimulationContext) {
        self.system.check_dimensions(1, 1);
    }

    /// Upstream `readInput`, `UpdateState` and `writeOutput`.
    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        let u = DVector::from_element(1, self.in_msg.read().input);
        let y = self.system.evaluate(context, &u, 1, 1);
        self.out_msg.write(SingleActuatorMsg { input: y[0] });
    }
}

impl SceneModel for ForceAtSiteLti {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn state_specs(&self) -> Vec<ModelStateSpec> {
        self.system.state_specs()
    }

    fn load_states(&mut self, states: &[&[f64]]) {
        self.system.load_states(states);
    }

    fn scene_reset(&mut self, _scene: &SceneView<'_>, _context: &SimulationContext) {
        self.system.check_dimensions(3, 3);
    }

    /// Upstream `readInput`, `UpdateState` and `writeOutput`.
    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        let force = self.in_msg.read().force_request_body_n;
        let u = DVector::from_column_slice(force.as_slice());
        let y = self.system.evaluate(context, &u, 3, 3);
        self.out_msg.write(BodyForceCommandMsg {
            force_request_body_n: nalgebra::Vector3::new(y[0], y[1], y[2]),
        });
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_linearTimeInvariantSystem.py` (its Python
    //! subclass variant is the same system as `SingleActuatorLTI`).

    use super::*;
    use crate::Module;
    use crate::mujoco_dynamics::MjScene;
    use crate::mujoco_dynamics::tests::{run, sec2nano};

    fn run_single_actuator(system: LinearTimeInvariantSystem, dt: f64, tf: f64) -> Vec<(f64, f64)> {
        let mut scene = MjScene::new("<mujoco/>").expect("scene");
        let command = Output::new(SingleActuatorMsg { input: 1.0 });
        let mut model = SingleActuatorLti {
            system,
            ..SingleActuatorLti::default()
        };
        command.connect_to(&mut model.in_msg);
        let out = model.out_msg.clone();
        let id = scene.add_model_to_dynamics_task(model, -1);
        scene.init();
        let model = scene.task_model(id);
        assert_eq!((model.input_size(), model.output_size()), (1, 1));
        let mut samples = Vec::new();
        run(&mut scene, sec2nano(dt), sec2nano(tf), |_, now| {
            samples.push((now as f64 * 1e-9, out.read().input));
        });
        samples
    }

    /// Upstream `test_linearTimeInvariantSystemFirstOrder`: `ẋ = -a x + a u`,
    /// `y = x` with a unit step reaches `1 - e^(-a t)`.
    #[test]
    fn first_order_step_response() {
        let (dt, tf, a) = (0.01, 2.0, 1.0);
        let mut system = LinearTimeInvariantSystem::default();
        system.set_a(DMatrix::from_element(1, 1, -a));
        system.set_b(DMatrix::from_element(1, 1, a));
        system.set_c(DMatrix::from_element(1, 1, 1.0));
        system.set_d(DMatrix::from_element(1, 1, 0.0));
        assert_eq!(system.state_size(), 1);
        let samples = run_single_actuator(system, dt, tf);
        let target = 1.0 - (-a * tf).exp();
        let final_value = samples.last().expect("samples").1;
        assert!((final_value - target).abs() <= 1e-2 + 0.02 * target);
    }

    /// Upstream `test_linearTimeInvariantSystemSecondOrder`: a critically
    /// damped second-order system (ω = 10 rad/s) follows
    /// `1 - e^(-ωt) (1 + ωt)` without overshoot.
    #[test]
    fn second_order_step_response() {
        let (dt, tf, wn, zeta) = (0.01, 2.0, 10.0, 1.0);
        let mut system = LinearTimeInvariantSystem::default();
        system.configure_second_order(wn, zeta, 1.0);
        assert_eq!(system.state_size(), 2);
        let samples = run_single_actuator(system, dt, tf);
        let final_value = samples.last().expect("samples").1;
        assert!((final_value - 1.0).abs() <= 0.005);
        assert!(samples.iter().all(|(_, y)| *y <= 1.01));
        for (t, y) in samples {
            let analytic = 1.0 - (-wn * t).exp() * (1.0 + wn * t);
            assert!(
                (y - analytic).abs() <= 0.05 + 0.1 * analytic.abs(),
                "t {t}: {y} vs {analytic}"
            );
        }
    }

    /// `configureSecondOrder` for vectors: decoupled channels, and the
    /// invalid parameters leave the system unchanged.
    #[test]
    fn mimo_second_order_and_invalid_parameters() {
        let mut system = LinearTimeInvariantSystem::default();
        system.configure_second_order_mimo(&[1.0, 2.0], &[0.5, 0.0], &[3.0, 4.0]);
        assert_eq!(
            (
                system.state_size(),
                system.input_size(),
                system.output_size()
            ),
            (4, 2, 2)
        );
        assert_eq!(system.a()[(3, 2)], -4.0);
        assert_eq!(system.a()[(1, 1)], -1.0);
        assert_eq!(system.b()[(3, 1)], 16.0);
        assert_eq!(system.c()[(1, 2)], 1.0);

        let before = system.clone();
        system.configure_second_order(-1.0, 0.5, 1.0);
        system.configure_second_order_mimo(&[1.0], &[-0.5], &[1.0]);
        system.configure_second_order_mimo(&[1.0, 2.0], &[0.5], &[1.0, 1.0]);
        assert_eq!(system, before);
    }

    /// The force LTI filters each axis of a site force.
    #[test]
    fn force_at_site_lti_filters_each_axis() {
        let mut scene = MjScene::new("<mujoco/>").expect("scene");
        let force = Output::new(BodyForceCommandMsg {
            force_request_body_n: nalgebra::Vector3::new(1.0, -2.0, 3.0),
        });
        let mut model = ForceAtSiteLti::default();
        model.system.set_d(DMatrix::identity(3, 3) * 2.0);
        force.connect_to(&mut model.in_msg);
        let out = model.out_msg.clone();
        scene.add_model_to_dynamics_task(model, -1);
        scene.init();
        scene.update(&crate::mujoco_dynamics::tests::context(0));
        assert_eq!(
            out.read().force_request_body_n,
            nalgebra::Vector3::new(2.0, -4.0, 6.0)
        );
    }
}
