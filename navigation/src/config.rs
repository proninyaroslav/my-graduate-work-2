extern crate clap;

use self::clap::{ArgGroup, ArgMatches};
use clap::{App, AppSettings, Arg};
use std::ffi::OsString;
use std::path::PathBuf;

/// Defines the system parameter to be optimized.
#[derive(Debug, Eq, PartialEq)]
pub enum Optimize {
    /// Optimize by intuitive flight **(m)**.
    Intuitive,

    /// Optimize by flying time **(s)**.
    Time,

    /// Optimization of battery capacity usage **(A/h)**.
    Battery,

    /// Optimize by energy consumption **(kJ)**.
    Energy,
}

#[derive(Debug)]
pub struct Config {
    pub params_file: PathBuf,
    pub out_filename: Option<PathBuf>,
    pub out_as_json: bool,
    pub optimize: Optimize,
}

impl Config {
    pub fn from_args() -> Self {
        let matches = Self::make_app().get_matches();

        Self::make_config(matches)
    }

    pub fn from_iter<I, T>(itr: I) -> Self
    where
        I: IntoIterator<Item = T>,
        T: Into<OsString> + Clone,
    {
        let matches = Self::make_app().get_matches_from(itr);

        Self::make_config(matches)
    }

    fn make_config(matches: ArgMatches) -> Self {
        let params_file = PathBuf::from(matches.value_of("params_file").unwrap());
        let out_filename = matches.value_of("out").map(PathBuf::from);
        let out_as_json = matches.is_present("json");
        let optimize = if matches.is_present("intuitive") {
            Optimize::Intuitive
        } else if matches.is_present("time") {
            Optimize::Time
        } else if matches.is_present("battery") {
            Optimize::Battery
        } else if matches.is_present("energy") {
            Optimize::Energy
        } else {
            unreachable!();
        };

        Config {
            params_file,
            out_filename,
            out_as_json,
            optimize,
        }
    }

    fn make_app<'a, 'b>() -> App<'a, 'b> {
        App::new(clap::crate_name!())
            .version(clap::crate_version!())
            .setting(AppSettings::ArgRequiredElseHelp)
            .arg(
                Arg::with_name("params_file")
                    .help("Drone parameters file")
                    .required(true),
            )
            .arg(
                Arg::with_name("out")
                    .long("out")
                    .value_name("filename")
                    .help("Write result to the specified file")
                    .takes_value(true),
            )
            .arg(
                Arg::with_name("json")
                    .long("json")
                    .help("Output result as JSON"),
            )
            .group(
                ArgGroup::with_name("optimize")
                    .args(&["intuitive", "time", "battery", "energy"])
                    .required(true),
            )
            .arg(
                Arg::with_name("intuitive")
                    .help("Optimize by intuitive flight (m)")
                    .long("intuitive")
                    .short("i"),
            )
            .arg(
                Arg::with_name("time")
                    .help("Optimize by flying time (s)")
                    .long("time")
                    .short("t"),
            )
            .arg(
                Arg::with_name("battery")
                    .help("Optimization of battery capacity usage (A/h)")
                    .long("battery")
                    .short("b"),
            )
            .arg(
                Arg::with_name("energy")
                    .help("Optimize by energy consumption (kJ)")
                    .long("energy")
                    .short("e"),
            )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_args_test() {
        let matches = Config::make_app().get_matches_from(&[
            clap::crate_name!(),
            "params.json",
            "--out",
            "result.json",
            "--json",
            "-e",
        ]);
        assert_eq!(Some("params.json"), matches.value_of("params_file"));
        assert_eq!(Some("result.json"), matches.value_of("out"));
        assert!(matches.is_present("json"));
        assert!(matches.is_present("energy"));
        assert!(!matches.is_present("intuitive"));
        assert!(!matches.is_present("battery"));
        assert!(!matches.is_present("time"));
    }

    #[test]
    fn make_config_test() {
        let config = Config::from_iter(&[
            clap::crate_name!(),
            "params.json",
            "--out",
            "result.json",
            "--json",
            "-e",
        ]);
        assert_eq!(Some("params.json"), config.params_file.to_str());
        assert_eq!(Some("result.json"), config.out_filename.unwrap().to_str());
        assert!(config.out_as_json);
        assert_eq!(Optimize::Energy, config.optimize);
    }
}
