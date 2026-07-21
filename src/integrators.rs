pub mod traits;

mod butcher;
mod methods;

pub use traits::DynamicObject;

pub use butcher::{
    // Adaptive step-size control.
    AdaptiveController,
    // Butcher-tableau engine.
    ButcherTableau,
    // Tableau constants (for building custom integrators).
    DOPRI45,
    EULER,
    HEUN,
    MIDPOINT,
    RALSTON,
    RK4,
    RK4_THREE_EIGHTHS,
    RKF45,
    // Low-level drivers.
    propagate_adaptive_rk,
    propagate_explicit_rk,
};

pub use methods::{
    // Built-in integrator structs.
    Dopri45,
    Euler,
    Heun,
    // The extension point + its cloning helper.
    Integrator,
    IntegratorClone,
    Midpoint,
    Ralston,
    Rk4,
    Rk4ThreeEighths,
    Rkf45,
};
