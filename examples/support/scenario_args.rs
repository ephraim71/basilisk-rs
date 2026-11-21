//! Superseded by [`cli.rs`](cli.rs)'s `Cli`, which every other example uses:
//! `flag("--name")` is `Cli::switch("name")` and `value("--name", default)`
//! is `Cli::value("name", default)`. Kept only for `scenario_integrators`,
//! `scenario_integrators_comparison` and
//! `scenario_variable_time_step_integrators`; delete it once they move to
//! `Cli`.
//!
//! Minimal command-line options for selecting a scenario's upstream
//! parametrization: `--name value` options and `--flag` switches, with
//! `--help` printing the example's usage.

#![allow(dead_code)]

use std::str::FromStr;

/// The arguments after the program name.
#[derive(Clone, Debug)]
pub struct ScenarioArgs {
    usage: &'static str,
    arguments: Vec<String>,
}

impl ScenarioArgs {
    /// Reads `std::env::args()`; prints `usage` and exits on `--help`.
    pub fn from_env(usage: &'static str) -> Self {
        let arguments: Vec<String> = std::env::args().skip(1).collect();
        if arguments
            .iter()
            .any(|argument| argument == "--help" || argument == "-h")
        {
            println!("{usage}");
            std::process::exit(0);
        }
        Self { usage, arguments }
    }

    /// Whether the switch `--name` is present.
    pub fn flag(&self, name: &str) -> bool {
        self.arguments.iter().any(|argument| argument == name)
    }

    /// The value after `--name`, parsed; `default` when absent. Exits with the
    /// usage on a missing or unparsable value.
    pub fn value<T: FromStr>(&self, name: &str, default: T) -> T {
        let Some(index) = self.arguments.iter().position(|argument| argument == name) else {
            return default;
        };
        let Some(text) = self.arguments.get(index + 1) else {
            self.fail(&format!("{name} needs a value"));
        };
        text.parse()
            .unwrap_or_else(|_| self.fail(&format!("invalid value {text:?} for {name}")))
    }

    /// Exits with `message` and the usage.
    pub fn fail(&self, message: &str) -> ! {
        eprintln!("error: {message}\n\n{}", self.usage);
        std::process::exit(2);
    }
}
