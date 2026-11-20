//! Command-line selection of a scenario's upstream `run(...)` parameters.
//!
//! A boolean parameter `useFoo` is `--foo` (true) or `--no-foo` (false); a
//! valued one is `--name=value` or `--name value`. Parameters not given keep
//! the values of the upstream script's `if __name__ == "__main__":` call.
//! `--all` asks for every parametrization upstream's `test_scenario*.py`
//! runs, and `--help` prints the usage.

#![allow(dead_code)]

use std::str::FromStr;

pub struct Cli {
    usage: String,
    arguments: Vec<String>,
}

impl Cli {
    /// The process arguments; prints `usage` and exits on `--help`.
    pub fn from_env(usage: &str) -> Self {
        let arguments: Vec<String> = std::env::args().skip(1).collect();
        if arguments
            .iter()
            .any(|argument| argument == "--help" || argument == "-h")
        {
            println!("{usage}");
            std::process::exit(0);
        }
        Self {
            usage: usage.to_string(),
            arguments,
        }
    }

    /// `--name` or `--no-name`; `default` when neither is given.
    pub fn flag(&mut self, name: &str, default: bool) -> bool {
        let on = format!("--{name}");
        let off = format!("--no-{name}");
        let mut value = default;
        self.arguments.retain(|argument| {
            if *argument == on {
                value = true;
                false
            } else if *argument == off {
                value = false;
                false
            } else {
                true
            }
        });
        value
    }

    /// `--name` given at all.
    pub fn switch(&mut self, name: &str) -> bool {
        let on = format!("--{name}");
        let before = self.arguments.len();
        self.arguments.retain(|argument| *argument != on);
        self.arguments.len() != before
    }

    /// `--name=value` or `--name value`; `default` when not given.
    pub fn value<T: FromStr>(&mut self, name: &str, default: T) -> T {
        let key = format!("--{name}");
        let prefix = format!("--{name}=");
        let mut raw = None;
        let mut index = 0;
        while index < self.arguments.len() {
            if let Some(rest) = self.arguments[index].strip_prefix(&prefix) {
                raw = Some(rest.to_string());
                self.arguments.remove(index);
            } else if self.arguments[index] == key && index + 1 < self.arguments.len() {
                raw = Some(self.arguments.remove(index + 1));
                self.arguments.remove(index);
            } else {
                index += 1;
            }
        }
        match raw {
            None => default,
            Some(raw) => raw
                .parse()
                .unwrap_or_else(|_| self.fail(&format!("bad --{name}: {raw}"))),
        }
    }

    /// Rejects any argument no accessor consumed.
    pub fn finish(self) {
        if let Some(argument) = self.arguments.first() {
            self.fail(&format!("unknown argument {argument}"));
        }
    }

    fn fail(&self, message: &str) -> ! {
        eprintln!("error: {message}\n\n{}", self.usage);
        std::process::exit(2);
    }
}
