/// Connects one or more typed message outputs to inputs.
///
/// Each `output => input` entry expands to a separate
/// [`Simulation::connect`](crate::simulation::Simulation::connect) call.
///
/// # Example
///
/// Given an existing spacecraft, sun sensor, IMU, and simulation:
///
/// ```no_run
/// use basilisk_rs::connect;
/// use basilisk_rs::sensors::imu_sensor::ImuSensor;
/// use basilisk_rs::simulation::Simulation;
/// use basilisk_rs::spacecraft::Spacecraft;
/// use basilisk_rs::sensors::coarse_sun_sensor::CoarseSunSensor;
/// # fn wire(
/// #     sim: &Simulation<'_>,
/// #     spacecraft: &Spacecraft,
/// #     sun_sensor: &mut CoarseSunSensor,
/// #     imu: &mut ImuSensor,
/// # ) {
/// connect!(&sim,
///     &spacecraft.state_out => &mut sun_sensor.input_state_msg,
///     &spacecraft.state_out => &mut imu.input_state_msg,
/// );
/// # }
/// ```
#[macro_export]
macro_rules! connect {
    ($simulation:expr, $( $output:expr => $input:expr ),+ $(,)?) => {{
        $( ($simulation).connect($output, $input); )+
    }};
}

/// Adds one or more modules to a simulation schedule.
///
/// Each entry names a module and its scheduling parameters and expands to a
/// separate
/// [`Simulation::add_module`](crate::simulation::Simulation::add_module) call.
/// Modules with a higher priority run earlier within a tick.
///
/// Two per-entry forms are accepted and may be mixed within one invocation:
///
/// ```text
/// "name" => &mut module, period = <nanos>, priority = <n>;   // labeled
/// "name" => &mut module, <nanos>, <n>;                        // positional
/// ```
///
/// The labeled form makes the period and priority assigned to each module
/// explicit at the call site; the positional form is the terse original.
///
/// # Example
///
/// ```
/// use basilisk_rs::environment::eclipse::{Eclipse, EclipseConfig};
/// use basilisk_rs::schedule;
/// use basilisk_rs::simulation::Simulation;
/// use basilisk_rs::environment::solar_flux::{SolarFlux, SolarFluxConfig};
/// use hifitime::Epoch;
///
/// let mut eclipse = Eclipse::new(EclipseConfig {
///     name: "eclipse".into(),
///     occulting_body_radius_m: 6_371_000.0,
/// });
/// let mut solar_flux = SolarFlux::new(SolarFluxConfig {
///     name: "solar_flux".into(),
/// });
/// let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
///
/// schedule! { sim,
///     "eclipse" => &mut eclipse, period = 10_000_000, priority = 10;
///     "solar_flux" => &mut solar_flux, 10_000_000, 0;
/// }
///
/// assert_eq!(sim.module_names(), vec!["eclipse", "solar_flux"]);
/// ```
#[macro_export]
macro_rules! schedule {
    // Entry point: start the muncher with an empty statement accumulator.
    ($simulation:expr, $($entries:tt)+) => {
        $crate::schedule!(@munch ($simulation) () $($entries)+)
    };

    // No entries left: emit the accumulated `add_module` statements as a block.
    (@munch ($simulation:expr) ($($body:tt)*)) => {{
        $($body)*
    }};

    // Labeled `period = .., priority = ..` form. Listed before the positional
    // arm so `period = <expr>` matches these labels rather than being parsed as
    // an assignment expression by the positional arm.
    (@munch ($simulation:expr) ($($body:tt)*)
        $name:literal => $module:expr, period = $period:expr, priority = $priority:expr
        $(; $($rest:tt)*)?
    ) => {
        $crate::schedule!(@munch ($simulation)
            ($($body)* ($simulation).add_module($name, $module, $period, $priority);)
            $($($rest)*)?
        )
    };

    // Positional `period, priority` form.
    (@munch ($simulation:expr) ($($body:tt)*)
        $name:literal => $module:expr, $period:expr, $priority:expr
        $(; $($rest:tt)*)?
    ) => {
        $crate::schedule!(@munch ($simulation)
            ($($body)* ($simulation).add_module($name, $module, $period, $priority);)
            $($($rest)*)?
        )
    };
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, UnitQuaternion, Vector3};

    use crate::Module;
    use crate::sensors::coarse_sun_sensor::{CoarseSunSensor, CoarseSunSensorConfig};
    use crate::sensors::imu_sensor::{ImuSensor, ImuSensorConfig};
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};

    fn simulation<'a>() -> Simulation<'a> {
        Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false)
    }

    fn spacecraft() -> Spacecraft {
        Spacecraft::new(SpacecraftConfig {
            mass_kg: 12.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::identity(),
            integration_step_nanos: 10_000_000,
            initial_position_m: Vector3::new(7_000_000.0, 0.0, 0.0),
            initial_velocity_mps: Vector3::new(0.0, 7_500.0, 0.0),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::new(0.01, 0.02, 0.03),
            integrator: None,
        })
    }

    fn sun_sensor() -> CoarseSunSensor {
        CoarseSunSensor::new(CoarseSunSensorConfig {
            name: "sun_sensor".into(),
            position_m: Vector3::zeros(),
            body_to_sensor_quaternion: UnitQuaternion::identity(),
            fov_half_angle_rad: std::f64::consts::PI,
            scale_factor: 1.0,
            kelly_factor: 0.0,
            k_power: 1.0,
            bias: 0.0,
            noise_std: 0.0,
            noise_prop: 0.0,
            walk_bounds: 0.0,
            min_output: 0.0,
            max_output: 1.0,
        })
    }

    fn imu() -> ImuSensor {
        ImuSensor::new(ImuSensorConfig {
            name: "imu".into(),
            position_m: Vector3::zeros(),
            body_to_sensor_quaternion: UnitQuaternion::identity(),
            rate_noise_std_radps: Vector3::zeros(),
        })
    }

    #[test]
    fn connect_wires_spacecraft_state_to_multiple_sensor_modules() {
        let mut spacecraft = spacecraft();
        let mut sun_sensor = sun_sensor();
        let mut imu = imu();
        let sim = simulation();

        crate::connect!(&sim,
            &spacecraft.state_out => &mut sun_sensor.input_state_msg,
            &spacecraft.state_out => &mut imu.input_state_msg,
        );

        spacecraft.init();

        let sun_sensor_state = sun_sensor.input_state_msg.read();
        let imu_state = imu.input_state_msg.read();
        assert_eq!(
            sun_sensor_state.position_m,
            spacecraft.config.initial_position_m
        );
        assert_eq!(
            sun_sensor_state.omega_radps,
            spacecraft.config.initial_omega_radps
        );
        assert_eq!(imu_state.position_m, spacecraft.config.initial_position_m);
        assert_eq!(imu_state.omega_radps, spacecraft.config.initial_omega_radps);
    }

    #[test]
    fn schedule_accepts_labeled_form() {
        let mut sun_sensor = sun_sensor();
        let mut imu = imu();
        let mut sim = simulation();

        crate::schedule! { sim,
            "sun_sensor" => &mut sun_sensor, period = 10_000_000, priority = 0;
            "imu" => &mut imu, period = 10_000_000, priority = 10;
        }

        assert_eq!(sim.module_names(), vec!["sun_sensor", "imu"]);
    }

    #[test]
    fn schedule_accepts_positional_form() {
        let mut sun_sensor = sun_sensor();
        let mut imu = imu();
        let mut sim = simulation();

        crate::schedule! { sim,
            "sun_sensor" => &mut sun_sensor, 10_000_000, 0;
            "imu" => &mut imu, 10_000_000, 10;
        }

        assert_eq!(sim.module_names(), vec!["sun_sensor", "imu"]);
    }

    #[test]
    fn schedule_accepts_mixed_forms() {
        let mut sun_sensor = sun_sensor();
        let mut imu = imu();
        let mut sim = simulation();

        crate::schedule! { sim,
            "sun_sensor" => &mut sun_sensor, period = 10_000_000, priority = 0;
            "imu" => &mut imu, 10_000_000, 10;
        }

        assert_eq!(sim.module_names(), vec!["sun_sensor", "imu"]);
    }

    #[test]
    fn labeled_and_positional_forms_schedule_identically() {
        // Reads (name, period, priority) straight from the scheduled modules;
        // this test module is a descendant of `crate::simulation`, so it may
        // touch those private fields.
        fn scheduled_params(sim: &Simulation<'_>) -> Vec<(String, u64, i32)> {
            sim.modules
                .iter()
                .map(|scheduled| {
                    (
                        scheduled.name.clone(),
                        scheduled.period_nanos,
                        scheduled.priority,
                    )
                })
                .collect()
        }

        // Same values, one call site per form.
        let mut labeled_sun = sun_sensor();
        let mut labeled_imu = imu();
        let mut labeled_sim = simulation();
        crate::schedule! { labeled_sim,
            "sun_sensor" => &mut labeled_sun, period = 10_000_000, priority = 0;
            "imu" => &mut labeled_imu, period = 20_000_000, priority = 10;
        }

        let mut positional_sun = sun_sensor();
        let mut positional_imu = imu();
        let mut positional_sim = simulation();
        crate::schedule! { positional_sim,
            "sun_sensor" => &mut positional_sun, 10_000_000, 0;
            "imu" => &mut positional_imu, 20_000_000, 10;
        }

        assert_eq!(
            scheduled_params(&labeled_sim),
            scheduled_params(&positional_sim),
        );
    }
}
