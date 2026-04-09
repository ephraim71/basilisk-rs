use nalgebra::Vector3;

use crate::messages::{EclipseMsg, Input, SpacecraftStateMsg, SunEphemerisMsg};
use crate::spacecraft::EffectorOutput;

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_693.0;
const SOLAR_FLUX_AT_EARTH_WPM2: f64 = 1372.5398;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Clone, Debug)]
pub struct SolarRadiationPressureConfig {
    pub name: String,
    pub area_m2: f64,
    pub coefficient_reflection: f64,
}

#[derive(Clone, Debug)]
pub struct SolarRadiationPressure {
    pub config: SolarRadiationPressureConfig,
    pub input_sun_msg: Input<SunEphemerisMsg>,
    pub input_eclipse_msg: Input<EclipseMsg>,
}

impl SolarRadiationPressure {
    pub fn new(config: SolarRadiationPressureConfig) -> Self {
        Self {
            config,
            input_sun_msg: Input::default(),
            input_eclipse_msg: Input::default(),
        }
    }

    pub fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let sun = self.input_sun_msg.read();
        let eclipse = self.input_eclipse_msg.read();
        let spacecraft_position = state.position_m;
        let sun_to_spacecraft = spacecraft_position - sun.sun_position_inertial_m;
        let sun_distance_m = sun_to_spacecraft.norm();

        if sun_distance_m == 0.0 {
            return EffectorOutput::default();
        }

        let scale_factor = self.config.coefficient_reflection
            * self.config.area_m2
            * SOLAR_FLUX_AT_EARTH_WPM2
            * ASTRONOMICAL_UNIT_M.powi(2)
            / (SPEED_OF_LIGHT_MPS * sun_distance_m.powi(3));

        EffectorOutput {
            force_inertial_n: scale_factor * sun_to_spacecraft * eclipse.illumination_factor,
            torque_body_nm: Vector3::zeros(),
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use crate::messages::{EclipseMsg, Output, SpacecraftStateMsg, SunEphemerisMsg};

    use super::{
        ASTRONOMICAL_UNIT_M, SOLAR_FLUX_AT_EARTH_WPM2, SPEED_OF_LIGHT_MPS,
        SolarRadiationPressure, SolarRadiationPressureConfig,
    };

    fn make_srp(area: f64, cr: f64) -> (SolarRadiationPressure, Output<SunEphemerisMsg>, Output<EclipseMsg>) {
        let sun_out = Output::new(SunEphemerisMsg::default());
        let eclipse_out = Output::new(EclipseMsg { illumination_factor: 1.0 });
        let mut srp = SolarRadiationPressure::new(SolarRadiationPressureConfig {
            name: "srp".to_string(),
            area_m2: area,
            coefficient_reflection: cr,
        });
        srp.input_sun_msg.connect(sun_out.slot());
        srp.input_eclipse_msg.connect(eclipse_out.slot());
        (srp, sun_out, eclipse_out)
    }

    fn make_state(position_m: Vector3<f64>) -> SpacecraftStateMsg {
        SpacecraftStateMsg { position_m, ..Default::default() }
    }

    /// Spacecraft at sun position → zero division guard → zero force.
    #[test]
    fn sun_at_spacecraft_yields_zero_force() {
        let sc_pos = Vector3::new(1e9, 0.0, 0.0);
        let (_srp, _sun, _ecl) = make_srp(4.0, 1.2);
        // Override sun_out with spacecraft position so sun_distance = 0
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: sc_pos,
            sun_velocity_inertial_mps: Vector3::zeros(),
        });
        let eclipse_out = Output::new(EclipseMsg { illumination_factor: 1.0 });
        let mut srp2 = SolarRadiationPressure::new(SolarRadiationPressureConfig {
            name: "srp".to_string(),
            area_m2: 4.0,
            coefficient_reflection: 1.2,
        });
        srp2.input_sun_msg.connect(sun_out.slot());
        srp2.input_eclipse_msg.connect(eclipse_out.slot());
        let out = srp2.compute_output(&make_state(sc_pos));
        assert_eq!(out.force_inertial_n, Vector3::zeros());
    }

    /// Full eclipse (illumination = 0) → zero force.
    #[test]
    fn full_eclipse_yields_zero_force() {
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::new(ASTRONOMICAL_UNIT_M, 0.0, 0.0),
            sun_velocity_inertial_mps: Vector3::zeros(),
        });
        let eclipse_out = Output::new(EclipseMsg { illumination_factor: 0.0 });
        let mut srp = SolarRadiationPressure::new(SolarRadiationPressureConfig {
            name: "srp".to_string(),
            area_m2: 4.0,
            coefficient_reflection: 1.2,
        });
        srp.input_sun_msg.connect(sun_out.slot());
        srp.input_eclipse_msg.connect(eclipse_out.slot());
        let out = srp.compute_output(&make_state(Vector3::new(-7_000_000.0, 0.0, 0.0)));
        assert_eq!(out.force_inertial_n, Vector3::zeros());
    }

    /// r_N = [-16937711153.5, -138435806556.0, -60051616256.6] m,
    /// sun = [507128401.716, 22652490.9092, -14854379.6232] m,
    /// area = 4 m², Cr = 1.2, illumination = 1.0.
    /// truth = [-2.44694525395e-06, -1.94212316004e-05, -8.42121070088e-06] N (1e-12 tol).
    #[test]
    fn force_cannonball_case() {
        let sc_pos = Vector3::new(-16937711153.5, -138435806556.0, -60051616256.6);
        let sun_pos = Vector3::new(507128401.716, 22652490.9092, -14854379.6232);

        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: sun_pos,
            sun_velocity_inertial_mps: Vector3::zeros(),
        });
        let eclipse_out = Output::new(EclipseMsg { illumination_factor: 1.0 });
        let mut srp = SolarRadiationPressure::new(SolarRadiationPressureConfig {
            name: "srp".to_string(),
            area_m2: 4.0,
            coefficient_reflection: 1.2,
        });
        srp.input_sun_msg.connect(sun_out.slot());
        srp.input_eclipse_msg.connect(eclipse_out.slot());
        let out = srp.compute_output(&make_state(sc_pos));

        // Compute expected analytically using same constants
        let sun_to_sc = sc_pos - sun_pos;
        let d = sun_to_sc.norm();
        let scale = 1.2 * 4.0 * SOLAR_FLUX_AT_EARTH_WPM2 * ASTRONOMICAL_UNIT_M.powi(2)
            / (SPEED_OF_LIGHT_MPS * d.powi(3));
        let expected = scale * sun_to_sc;

        // Basilisk truth: [-2.44694525395e-06, -1.94212316004e-05, -8.42121070088e-06]
        let basilisk_truth = Vector3::new(-2.44694525395e-06, -1.94212316004e-05, -8.42121070088e-06);
        assert!(
            (out.force_inertial_n - expected).norm() < 1e-12,
            "analytic mismatch: {:?} vs {:?}", out.force_inertial_n, expected
        );
        assert!(
            (out.force_inertial_n - basilisk_truth).norm() < 1e-12,
            "basilisk mismatch: {:?} vs {:?}", out.force_inertial_n, basilisk_truth
        );
    }

    /// Partial eclipse (illumination = 0.5) → force is exactly half of the full illumination force.
    #[test]
    fn partial_eclipse_scales_force_linearly() {
        let sc_pos = Vector3::new(-16937711153.5, -138435806556.0, -60051616256.6);
        let sun_pos = Vector3::new(507128401.716, 22652490.9092, -14854379.6232);

        let make = |factor: f64| {
            let sun_out = Output::new(SunEphemerisMsg {
                sun_position_inertial_m: sun_pos,
                sun_velocity_inertial_mps: Vector3::zeros(),
            });
            let eclipse_out = Output::new(EclipseMsg { illumination_factor: factor });
            let mut srp = SolarRadiationPressure::new(SolarRadiationPressureConfig {
                name: "srp".to_string(),
                area_m2: 4.0,
                coefficient_reflection: 1.2,
            });
            srp.input_sun_msg.connect(sun_out.slot());
            srp.input_eclipse_msg.connect(eclipse_out.slot());
            srp.compute_output(&make_state(sc_pos)).force_inertial_n
        };

        let full = make(1.0);
        let half = make(0.5);
        assert!(
            (half - 0.5 * full).norm() < 1e-20,
            "half-eclipse force should be 0.5 * full: {:?} vs {:?}", half, 0.5 * full
        );
    }
}
