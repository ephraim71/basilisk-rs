//! Classical orbit elements plus an offset: port of upstream Basilisk
//! `fswAlgorithms/orbitControl/orbElemOffset`.
//!
//! Every update adds the offset elements to the main ones, element by element.
//! The anomaly is either added as a true anomaly too, or, with
//! [`OrbElemOffsetConfig::use_mean_anomaly_offset`], the offset's `f` is taken
//! as a mean anomaly increment: the main true anomaly is converted to a mean
//! anomaly with the main eccentricity, incremented, and converted back with
//! the summed eccentricity. Both conversions are elliptic only, so with the
//! mean-anomaly offset an eccentricity outside `[0, 1)` gives a `NaN` anomaly,
//! as upstream. The other fields (radius, inverse semi-major axis, periapsis
//! and apoapsis radii) are left zero.

use crate::messages::{ClassicElementsMsg, Input, Output};
use crate::utilities::orbital_motion::{
    eccentric_to_mean_anomaly, eccentric_to_true_anomaly, mean_to_eccentric_anomaly,
    true_to_eccentric_anomaly,
};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`OrbElemOffset`].
#[derive(Clone, Debug)]
pub struct OrbElemOffsetConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `useMeanAnomalyOffset`: read the offset's `f` as a mean
    /// anomaly increment \[rad\] instead of a true anomaly one (default
    /// false).
    pub use_mean_anomaly_offset: bool,
}

impl Default for OrbElemOffsetConfig {
    fn default() -> Self {
        Self {
            name: "OrbElemOffset".to_string(),
            use_mean_anomaly_offset: false,
        }
    }
}

/// Adds an element offset to classical orbit elements: upstream
/// `OrbElemOffset`.
#[derive(Clone, Debug)]
pub struct OrbElemOffset {
    /// Module configuration.
    pub config: OrbElemOffsetConfig,
    /// Upstream `mainElementsInMsg`: the base elements.
    pub main_elements_in_msg: Input<ClassicElementsMsg>,
    /// Upstream `offsetElementsInMsg`: the offsets to add.
    pub offset_elements_in_msg: Input<ClassicElementsMsg>,
    /// Upstream `elementsOutMsg`: the combined elements.
    pub elements_out_msg: Output<ClassicElementsMsg>,
}

impl OrbElemOffset {
    /// Builds the module with unconnected ports.
    pub fn new(config: OrbElemOffsetConfig) -> Self {
        Self {
            config,
            main_elements_in_msg: Input::default(),
            offset_elements_in_msg: Input::default(),
            elements_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset`: the module keeps no state, so a reset only reports
    /// missing inputs.
    fn check_connections(&self) {
        if !self.main_elements_in_msg.is_connected() {
            log::error!("OrbElemOffset::Reset: main_elements_in_msg is not connected");
        }
        if !self.offset_elements_in_msg.is_connected() {
            log::error!("OrbElemOffset::Reset: offset_elements_in_msg is not connected");
        }
    }

    fn combined(&self) -> ClassicElementsMsg {
        let main = self.main_elements_in_msg.read();
        let offset = self.offset_elements_in_msg.read();
        let eccentricity = main.eccentricity + offset.eccentricity;
        let true_anomaly_rad = if self.config.use_mean_anomaly_offset {
            let e_main = main.eccentricity;
            let mean_main = eccentric_to_mean_anomaly(
                true_to_eccentric_anomaly(main.true_anomaly_rad, e_main),
                e_main,
            );
            let mean = mean_main + offset.true_anomaly_rad;
            eccentric_to_true_anomaly(mean_to_eccentric_anomaly(mean, eccentricity), eccentricity)
        } else {
            main.true_anomaly_rad + offset.true_anomaly_rad
        };
        ClassicElementsMsg {
            semi_major_axis: main.semi_major_axis + offset.semi_major_axis,
            eccentricity,
            inclination_rad: main.inclination_rad + offset.inclination_rad,
            raan_rad: main.raan_rad + offset.raan_rad,
            arg_periapsis_rad: main.arg_periapsis_rad + offset.arg_periapsis_rad,
            true_anomaly_rad,
            ..Default::default()
        }
    }
}

impl Module for OrbElemOffset {
    fn init(&mut self) {
        self.elements_out_msg.write(ClassicElementsMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.elements_out_msg.write(self.combined());
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;

    use super::{OrbElemOffset, OrbElemOffsetConfig};
    use crate::messages::{ClassicElementsMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::orbital_motion::{eccentric_to_mean_anomaly, true_to_eccentric_anomaly};

    /// Upstream test task rate, `macros.sec2nano(0.1)`.
    const TASK_RATE_NANOS: u64 = 100_000_000;

    /// `pytest.approx` with its defaults: relative 1e-6, absolute 1e-12.
    #[track_caller]
    fn assert_approx(actual: f64, expected: f64, what: &str) {
        let tolerance = (1e-6 * expected.abs()).max(1e-12);
        assert!(
            (actual - expected).abs() <= tolerance,
            "{what}: {actual:.17e} vs {expected:.17e}"
        );
    }

    /// Upstream `test_orbElemOffset`: every element is the sum, and the
    /// anomaly is either the sum of true anomalies or the offset is the
    /// difference of mean anomalies (rel and abs 1e-12).
    #[test]
    fn matches_upstream_orb_elem_offset_test() {
        let main = ClassicElementsMsg {
            semi_major_axis: 1.0,
            eccentricity: 0.1,
            inclination_rad: 0.2,
            raan_rad: 0.3,
            arg_periapsis_rad: 0.4,
            true_anomaly_rad: 0.5,
            ..Default::default()
        };
        let offset = ClassicElementsMsg {
            semi_major_axis: 2.0,
            eccentricity: 0.02,
            inclination_rad: 0.03,
            raan_rad: 0.04,
            arg_periapsis_rad: 0.05,
            true_anomaly_rad: 0.06,
            ..Default::default()
        };
        for use_mean_anomaly_offset in [false, true] {
            let main_out = Output::new(main);
            let offset_out = Output::new(offset);
            let mut module = OrbElemOffset::new(OrbElemOffsetConfig {
                use_mean_anomaly_offset,
                ..Default::default()
            });
            let mut recorder = MessageRecorder::<ClassicElementsMsg>::default();
            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            simulation.connect(&main_out, &mut module.main_elements_in_msg);
            simulation.connect(&offset_out, &mut module.offset_elements_in_msg);
            simulation.connect(&module.elements_out_msg, &mut recorder.input_msg);
            simulation.add_module("OrbElemOffset", &mut module, TASK_RATE_NANOS, 0);
            simulation.add_module("recorder", &mut recorder, TASK_RATE_NANOS, 0);
            simulation.run_for(TASK_RATE_NANOS);

            assert!(!recorder.samples.is_empty());
            let out = recorder.samples[0];
            assert_approx(
                out.semi_major_axis,
                main.semi_major_axis + offset.semi_major_axis,
                "a",
            );
            assert_approx(
                out.eccentricity,
                main.eccentricity + offset.eccentricity,
                "e",
            );
            assert_approx(
                out.inclination_rad,
                main.inclination_rad + offset.inclination_rad,
                "i",
            );
            assert_approx(out.raan_rad, main.raan_rad + offset.raan_rad, "Omega");
            assert_approx(
                out.arg_periapsis_rad,
                main.arg_periapsis_rad + offset.arg_periapsis_rad,
                "omega",
            );
            if use_mean_anomaly_offset {
                let mean = |f, e| eccentric_to_mean_anomaly(true_to_eccentric_anomaly(f, e), e);
                let difference = mean(out.true_anomaly_rad, out.eccentricity)
                    - mean(main.true_anomaly_rad, main.eccentricity);
                assert!(
                    (difference - offset.true_anomaly_rad).abs()
                        <= 1e-12f64.max(1e-12 * offset.true_anomaly_rad.abs()),
                    "{difference}"
                );
            } else {
                assert_approx(
                    out.true_anomaly_rad,
                    main.true_anomaly_rad + offset.true_anomaly_rad,
                    "f",
                );
            }
            // The derived fields are left zero.
            assert_eq!(out.orbit_radius, 0.0);
            assert_eq!(out.periapsis_radius, 0.0);
        }
    }
}
