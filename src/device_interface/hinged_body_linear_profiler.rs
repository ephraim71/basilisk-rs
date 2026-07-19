use crate::messages::{HingedRigidBodyMsg, Output};
use crate::{Module, SimulationContext};

const NANO2SEC: f64 = 1.0e-9;

#[derive(Clone, Debug)]
pub struct HingedBodyLinearProfiler {
    pub name: String,
    pub start_time_nanos: u64,
    pub end_time_nanos: u64,
    pub start_theta_rad: f64,
    pub end_theta_rad: f64,
    pub hinged_rigid_body_reference_out: Output<HingedRigidBodyMsg>,
    deployment_slope_radps: f64,
}

impl HingedBodyLinearProfiler {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            start_time_nanos: 0,
            end_time_nanos: 0,
            start_theta_rad: 0.0,
            end_theta_rad: 0.0,
            hinged_rigid_body_reference_out: Output::default(),
            deployment_slope_radps: 0.0,
        }
    }

    pub fn with_profile(
        name: impl Into<String>,
        start_time_nanos: u64,
        end_time_nanos: u64,
        start_theta_rad: f64,
        end_theta_rad: f64,
    ) -> Self {
        let mut profiler = Self::new(name);
        profiler.start_time_nanos = start_time_nanos;
        profiler.end_time_nanos = end_time_nanos;
        profiler.start_theta_rad = start_theta_rad;
        profiler.end_theta_rad = end_theta_rad;
        profiler
    }

    pub fn reference_at(&self, current_sim_nanos: u64) -> HingedRigidBodyMsg {
        if current_sim_nanos < self.start_time_nanos {
            HingedRigidBodyMsg {
                theta_rad: self.start_theta_rad,
                theta_dot_radps: 0.0,
            }
        } else if current_sim_nanos <= self.end_time_nanos {
            let elapsed_seconds = (current_sim_nanos - self.start_time_nanos) as f64 * NANO2SEC;
            HingedRigidBodyMsg {
                theta_rad: self.start_theta_rad + elapsed_seconds * self.deployment_slope_radps,
                theta_dot_radps: self.deployment_slope_radps,
            }
        } else {
            HingedRigidBodyMsg {
                theta_rad: self.end_theta_rad,
                theta_dot_radps: 0.0,
            }
        }
    }
}

impl Module for HingedBodyLinearProfiler {
    fn init(&mut self) {
        assert!(
            self.end_time_nanos > self.start_time_nanos,
            "hinged body profiler '{}' requires end_time_nanos > start_time_nanos",
            self.name
        );
        self.deployment_slope_radps = (self.end_theta_rad - self.start_theta_rad)
            / ((self.end_time_nanos - self.start_time_nanos) as f64 * NANO2SEC);
        self.hinged_rigid_body_reference_out
            .write(self.reference_at(0));
    }

    fn update(&mut self, context: &SimulationContext) {
        self.hinged_rigid_body_reference_out
            .write(self.reference_at(context.current_sim_nanos));
    }
}

#[cfg(test)]
mod tests {
    use super::HingedBodyLinearProfiler;
    use crate::Module;

    #[test]
    fn linear_profiler_produces_expected_trace() {
        let mut profiler = HingedBodyLinearProfiler::with_profile(
            "profiler",
            1_000_000_000,
            2_000_000_000,
            0.0,
            std::f64::consts::PI / 180.0,
        );
        profiler.init();

        let sample_times_nanos = [
            0,
            500_000_000,
            1_000_000_000,
            1_500_000_000,
            2_000_000_000,
            2_500_000_000,
            3_000_000_000,
        ];
        let expected_theta = [
            0.0,
            0.0,
            0.0,
            std::f64::consts::PI / 360.0,
            std::f64::consts::PI / 180.0,
            std::f64::consts::PI / 180.0,
            std::f64::consts::PI / 180.0,
        ];
        let expected_theta_dot = [
            0.0,
            0.0,
            std::f64::consts::PI / 180.0,
            std::f64::consts::PI / 180.0,
            std::f64::consts::PI / 180.0,
            0.0,
            0.0,
        ];

        for ((time_nanos, expected_theta), expected_theta_dot) in sample_times_nanos
            .into_iter()
            .zip(expected_theta)
            .zip(expected_theta_dot)
        {
            let reference = profiler.reference_at(time_nanos);
            assert!((reference.theta_rad - expected_theta).abs() < 1.0e-12);
            assert!((reference.theta_dot_radps - expected_theta_dot).abs() < 1.0e-12);
        }
    }
}
