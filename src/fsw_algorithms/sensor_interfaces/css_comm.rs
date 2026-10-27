//! Coarse-sun-sensor calibration: port of upstream Basilisk `cssComm`
//! (`CSSSensorData`).
//!
//! Each update scales the raw reading of every configured sensor by
//! `max_sensor_value`, adds a Chebyshev-series residual correction (Kelly
//! calibration) evaluated at the scaled value, clips the result to `[0, 1]` and
//! publishes the corrected cosines as a new [`CssArraySensorMsg`]. Entries past
//! `num_sensors` and the time tag stay zero, as upstream.

use crate::messages::{CssArraySensorMsg, Input, MAX_NUM_CSS_SENSORS, Output};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`CssComm`]. `Default` is upstream's
/// zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct CssCommConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `numSensors`: number of leading sensors to process. A reset
    /// clamps it to [`MAX_NUM_CSS_SENSORS`] with a warning, as upstream.
    pub num_sensors: usize,
    /// Upstream `maxSensorValue`: raw reading that corresponds to a cosine of
    /// one; readings are divided by it.
    pub max_sensor_value: f64,
    /// Upstream `kellyCheby`: coefficients `c_j` of the Chebyshev residual
    /// `Σ c_j T_j(x)`, lowest order first. Its length is upstream's
    /// `chebyCount`. Upstream stores at most 32 (`MAX_NUM_CHEBY_POLYS`); a
    /// longer series, which upstream would read past its array, is used in
    /// full.
    pub kelly_cheby: Vec<f64>,
}

impl Default for CssCommConfig {
    fn default() -> Self {
        Self {
            name: "cssComm".to_string(),
            num_sensors: 0,
            max_sensor_value: 0.0,
            kelly_cheby: Vec::new(),
        }
    }
}

/// Coarse-sun-sensor calibration and clipping: upstream `cssComm`.
#[derive(Clone, Debug)]
pub struct CssComm {
    /// User-settable parameters.
    pub config: CssCommConfig,
    /// Upstream `sensorListInMsg`: raw CSS readings.
    pub sensor_list_in_msg: Input<CssArraySensorMsg>,
    /// Upstream `cssArrayOutMsg`: corrected cosine values.
    pub css_array_out_msg: Output<CssArraySensorMsg>,
}

impl CssComm {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: CssCommConfig) -> Self {
        Self {
            config,
            sensor_list_in_msg: Input::default(),
            css_array_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_cssProcessTelem`. Upstream also zeroes its
    /// `inputValues` buffer here, which nothing reads, so it is not ported.
    fn check_configuration(&mut self) {
        if !self.sensor_list_in_msg.is_connected() {
            log::error!("cssComm.sensor_list_in_msg is not connected");
        }

        let config = &mut self.config;
        if config.num_sensors > MAX_NUM_CSS_SENSORS {
            log::warn!(
                "cssComm: the configured number of CSS sensors exceeds the maximum, {} > \
                 {MAX_NUM_CSS_SENSORS}; using the maximum",
                config.num_sensors
            );
            config.num_sensors = MAX_NUM_CSS_SENSORS;
        } else if config.num_sensors == 0 {
            log::warn!("cssComm: there are zero CSS configured");
        }

        if config.max_sensor_value == 0.0 {
            log::warn!(
                "cssComm: max_sensor_value is zero, so CSS values are normalized by zero, \
                 inducing faux saturation"
            );
        }
    }

    fn corrected_readings(&self, raw: &CssArraySensorMsg) -> CssArraySensorMsg {
        let config = &self.config;
        let mut output = CssArraySensorMsg::default();
        // The zip stops at the array length, so a count raised past the
        // maximum after a reset (upstream reads out of bounds) is harmless.
        for (corrected, reading) in output
            .cos_values
            .iter_mut()
            .zip(raw.cos_values)
            .take(config.num_sensors)
        {
            let scaled = reading / config.max_sensor_value;
            // NaN passes through `clamp`, as through upstream's comparisons.
            *corrected = (scaled + chebyshev_residual(scaled, &config.kelly_cheby)).clamp(0.0, 1.0);
        }
        output
    }
}

/// `Σ c_j T_j(x)` by the Chebyshev recurrence `T_{j+1} = 2x T_j - T_{j-1}`,
/// summed in upstream's order.
fn chebyshev_residual(x: f64, coefficients: &[f64]) -> f64 {
    let (mut t_previous, mut t_current) = (1.0, x);
    let mut residual = coefficients.first().map_or(0.0, |c0| t_previous * c0);
    if let Some(c1) = coefficients.get(1) {
        residual += t_current * c1;
    }
    let two_x = 2.0 * x;
    for coefficient in coefficients.iter().skip(2) {
        (t_previous, t_current) = (t_current, two_x * t_current - t_previous);
        residual += coefficient * t_current;
    }
    residual
}

impl Module for CssComm {
    fn init(&mut self) {
        self.css_array_out_msg.write(CssArraySensorMsg::default());
        self.check_configuration();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_configuration();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let raw = self.sensor_list_in_msg.read();
        self.css_array_out_msg.write(self.corrected_readings(&raw));
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;

    use super::{CssComm, CssCommConfig, chebyshev_residual};
    use crate::messages::{CssArraySensorMsg, MAX_NUM_CSS_SENSORS, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(0.5)`.
    const TASK_RATE_NANOS: u64 = 500_000_000;

    /// The Kelly fit `test_cssComm.py` configures (trailing zeros dropped).
    const UPSTREAM_CHEBY: [f64; 14] = [
        -1.734963346951471e+06,
        3.294117146099591e+06,
        -2.816333294617512e+06,
        2.163709942144332e+06,
        -1.488025993860025e+06,
        9.107359382775769e+05,
        -4.919712500291216e+05,
        2.318436583511218e+05,
        -9.37610504552901e+04,
        3.177536873430168e+04,
        -8.704033370738143e+03,
        1.8161881081763e+03,
        -2.581556805090373e+02,
        1.88841892428278e+01,
    ];

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn upstream_config(num_sensors: usize) -> CssCommConfig {
        CssCommConfig {
            num_sensors,
            max_sensor_value: 500e-6,
            kelly_cheby: UPSTREAM_CHEBY.to_vec(),
            ..Default::default()
        }
    }

    /// `test_cssComm`, all three parametrizations, with upstream's hard-coded
    /// truth and `accuracy = 1e-6` on all 32 entries. Upstream runs a 0.5 s
    /// task to t = 0.5 s, so two samples are recorded. The second case has
    /// zero sensors and the third asks for 33, which reset clamps to 32.
    #[test]
    fn matches_upstream_css_comm_unit_test() {
        let accuracy = 1e-6;
        let cases: [(usize, Vec<f64>); 3] = [
            (4, vec![-100e-6, 200e-6, 600e-6, 300e-6, 200e-6]),
            (0, vec![-100e-6, 200e-6, 600e-6, 300e-6]),
            (MAX_NUM_CSS_SENSORS + 1, vec![200e-6; MAX_NUM_CSS_SENSORS]),
        ];
        for (num_sensors, sensor_data) in cases {
            let mut truth = [0.0; MAX_NUM_CSS_SENSORS];
            if num_sensors == 4 {
                truth[..4].copy_from_slice(&[0.0, 0.45791653042, 1.0, 0.615444781018]);
            }
            if num_sensors == MAX_NUM_CSS_SENSORS + 1 {
                truth = [0.45791653042; MAX_NUM_CSS_SENSORS];
            }

            let sensor_output = Output::new(CssArraySensorMsg::from_active(&sensor_data));
            let mut module = CssComm::new(upstream_config(num_sensors));
            let mut recorder = MessageRecorder::default();
            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            simulation.connect(&sensor_output, &mut module.sensor_list_in_msg);
            simulation.connect(&module.css_array_out_msg, &mut recorder.input_msg);
            simulation.add_module("cssComm", &mut module, TASK_RATE_NANOS, 0);
            simulation.add_module("recorder", &mut recorder, TASK_RATE_NANOS, 0);
            simulation.run_for(TASK_RATE_NANOS);
            drop(simulation);

            assert_eq!(recorder.samples.len(), 2, "numSensors = {num_sensors}");
            for sample in &recorder.samples {
                for (index, (value, expected)) in sample.cos_values.iter().zip(truth).enumerate() {
                    assert!(
                        (value - expected).abs() <= accuracy,
                        "numSensors = {num_sensors}, cosValues[{index}] = {value}, truth \
                         {expected}"
                    );
                }
                assert_eq!(sample.time_tag_s, 0.0);
            }
            assert_eq!(
                module.config.num_sensors,
                num_sensors.min(MAX_NUM_CSS_SENSORS)
            );
        }
    }

    /// Beyond `test_cssComm.py`: in-range readings that exercise the full
    /// 14-term fit, against the upstream C module compiled with gcc
    /// (`-ffp-contract=off`); the port reproduces it bit for bit.
    #[test]
    fn matches_upstream_c_bit_for_bit() {
        let raw = [
            200e-6, 300e-6, 251e-6, 333e-6, 399e-6, 420e-6, 287.5e-6, 312e-6,
        ];
        let upstream_c = [
            0.4579165375766753,
            0.6154447562941335,
            0.5317378179248468,
            0.6728112488615332,
            0.800352583254483,
            0.8409947881751442,
            0.5937680246323473,
            0.636175201890182,
        ];
        let sensor_output = Output::new(CssArraySensorMsg::from_active(&raw));
        let mut module = CssComm::new(upstream_config(raw.len()));
        sensor_output.connect_to(&mut module.sensor_list_in_msg);
        module.init();
        module.update(&context(0));
        let output = module.css_array_out_msg.read();
        assert_eq!(output.cos_values[..raw.len()], upstream_c);
        assert!(
            output.cos_values[raw.len()..]
                .iter()
                .all(|&value| value == 0.0)
        );
    }

    /// Not upstream: the residual against the closed form
    /// `T_j(x) = cos(j acos x)` for `|x| <= 1`, for every series length from
    /// zero to four coefficients.
    #[test]
    fn chebyshev_residual_matches_the_closed_form_polynomials() {
        let coefficients = [0.3, -0.2, 0.05, 0.7];
        for x in [-0.9, -0.25, 0.0, 0.4, 0.8, 1.0] {
            for count in 0..=coefficients.len() {
                let expected: f64 = coefficients[..count]
                    .iter()
                    .enumerate()
                    .map(|(order, c)| c * (order as f64 * f64::acos(x)).cos())
                    .sum();
                let actual = chebyshev_residual(x, &coefficients[..count]);
                assert!(
                    (actual - expected).abs() <= 1e-14,
                    "x = {x}, {count} terms: {actual} != {expected}"
                );
            }
        }
    }

    /// Not upstream: without a Chebyshev fit the module only scales and clips,
    /// and the configuration is read at update time, not in `new`.
    #[test]
    fn scales_and_clips_without_a_fit_using_config_set_after_new() {
        let sensor_output =
            Output::new(CssArraySensorMsg::from_active(&[1.0, 2.0, 3.0, -1.0, 2.0]));
        let mut module = CssComm::new(CssCommConfig::default());
        module.config.num_sensors = 4;
        module.config.max_sensor_value = 4.0;
        sensor_output.connect_to(&mut module.sensor_list_in_msg);
        module.init();
        module.update(&context(0));

        let output = module.css_array_out_msg.read();
        assert_eq!(output.cos_values[..5], [0.25, 0.5, 0.75, 0.0, 0.0]);

        // Readings above the maximum saturate at one.
        module.config.max_sensor_value = 1.5;
        module.update(&context(TASK_RATE_NANOS));
        let output = module.css_array_out_msg.read();
        assert_eq!(output.cos_values[..4], [1.0 / 1.5, 1.0, 1.0, 0.0]);
    }

    /// Not upstream: a single constant coefficient biases every reading, and
    /// the input time tag is not propagated (upstream writes a zeroed payload).
    #[test]
    fn a_constant_residual_biases_every_reading() {
        let mut readings = CssArraySensorMsg::from_active(&[0.25, 0.5]);
        readings.time_tag_s = 12.5;
        let sensor_output = Output::new(readings);
        let mut module = CssComm::new(CssCommConfig {
            num_sensors: 2,
            max_sensor_value: 1.0,
            kelly_cheby: vec![0.125],
            ..Default::default()
        });
        sensor_output.connect_to(&mut module.sensor_list_in_msg);
        module.init();
        module.update(&context(0));

        let output = module.css_array_out_msg.read();
        assert_eq!(output.cos_values[..3], [0.375, 0.625, 0.0]);
        assert_eq!(output.time_tag_s, 0.0);
    }

    /// Not upstream: an unconnected input reads zeros, which a nonzero
    /// constant residual still lifts, as upstream.
    #[test]
    fn unconnected_input_reads_zero_readings() {
        let mut module = CssComm::new(CssCommConfig {
            num_sensors: 3,
            max_sensor_value: 2.0,
            kelly_cheby: vec![0.5, 1.0],
            ..Default::default()
        });
        module.init();
        module.update(&context(0));
        assert_eq!(
            module.css_array_out_msg.read().cos_values[..4],
            [0.5, 0.5, 0.5, 0.0]
        );
    }
}
