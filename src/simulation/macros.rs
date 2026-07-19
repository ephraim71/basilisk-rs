/// Connects one or more typed message outputs to inputs.
///
/// Each `output => input` entry expands to a separate
/// [`Simulation::connect`](crate::simulation::Simulation::connect) call.
///
/// # Example
///
/// ```
/// use basilisk_rs::connect;
/// use basilisk_rs::messages::{Input, Output};
/// use basilisk_rs::simulation::Simulation;
/// use hifitime::Epoch;
///
/// let sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
/// let output = Output::new(42_u32);
/// let mut input = Input::<u32>::default();
///
/// connect!(&sim, &output => &mut input);
/// assert_eq!(input.read(), 42);
/// ```
#[macro_export]
macro_rules! connect {
    ($simulation:expr, $( $output:expr => $input:expr ),+ $(,)?) => {{
        $( ($simulation).connect($output, $input); )+
    }};
}

/// Adds one or more modules to a simulation schedule.
///
/// Each entry supplies the module name, module value, period in nanoseconds,
/// and priority for a separate
/// [`Simulation::add_module`](crate::simulation::Simulation::add_module) call.
///
/// # Example
///
/// ```
/// use basilisk_rs::{Module, SimulationContext, schedule};
/// use basilisk_rs::simulation::Simulation;
/// use hifitime::Epoch;
///
/// struct Noop;
/// impl Module for Noop {
///     fn init(&mut self) {}
///     fn update(&mut self, _context: &SimulationContext) {}
/// }
///
/// let mut first = Noop;
/// let mut second = Noop;
/// let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
///
/// schedule! { sim,
///     "first" => &mut first, 10_000_000, 0;
///     "second" => &mut second, 20_000_000, 10;
/// }
///
/// assert_eq!(sim.module_names(), vec!["first", "second"]);
/// ```
#[macro_export]
macro_rules! schedule {
    ($simulation:expr, $( $name:literal => $module:expr, $period:expr, $priority:expr );+ $(;)?) => {{
        $( ($simulation).add_module($name, $module, $period, $priority); )+
    }};
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;

    use crate::messages::{Input, Output};
    use crate::simulation::Simulation;
    use crate::{Module, SimulationContext};

    struct Noop;

    impl Module for Noop {
        fn init(&mut self) {}

        fn update(&mut self, _context: &SimulationContext) {}
    }

    fn simulation<'a>() -> Simulation<'a> {
        Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false)
    }

    #[test]
    fn connect_accepts_multiple_typed_connections() {
        let sim = simulation();
        let integer_output = Output::new(42_u32);
        let text_output = Output::new(String::from("ready"));
        let mut integer_input = Input::<u32>::default();
        let mut text_input = Input::<String>::default();

        crate::connect!(&sim,
            &integer_output => &mut integer_input,
            &text_output => &mut text_input,
        );

        assert_eq!(integer_input.read(), 42);
        assert_eq!(text_input.read(), "ready");
    }

    #[test]
    fn schedule_accepts_multiple_modules() {
        let mut first = Noop;
        let mut second = Noop;
        let mut sim = simulation();

        crate::schedule! { sim,
            "first" => &mut first, 10_000_000, 0;
            "second" => &mut second, 20_000_000, 10;
        }

        assert_eq!(sim.module_names(), vec!["first", "second"]);
    }
}
