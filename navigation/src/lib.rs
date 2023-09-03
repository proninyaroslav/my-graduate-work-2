#[cfg(test)]
#[macro_use]
extern crate float_cmp;
extern crate thiserror;

pub mod config;
mod cost_generator;
mod formatter;
mod path_finder;

use crate::config::{Config, Optimize};
use crate::cost_generator::{CostMatrices, Params};
use formatter::{JsonFormatter, TextFormatter};
use std::fs::File;
use std::io::stdout;
use std::iter::once_with;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("Unable to find path")]
    UnableToFindPath,
}

pub fn run(config: Config) -> Result<(), Box<dyn std::error::Error>> {
    let params = Params::from_file(config.params_file)?;
    let cost_matrices = CostMatrices::generate(&params);

    let cost_matrix = match config.optimize {
        Optimize::Intuitive => &cost_matrices.distance,
        Optimize::Time => &cost_matrices.time,
        Optimize::Battery => &cost_matrices.capacity,
        Optimize::Energy => &cost_matrices.energy,
    };
    let result = path_finder::find(cost_matrix).ok_or(Error::UnableToFindPath)?;

    let path: Vec<formatter::Point> = result
        .get_path()
        .iter()
        .enumerate()
        .map(|(i, p)| (i, p.from(), p.to()))
        .flat_map(|(i, from, to)| {
            let coord = params.coords[from];
            let p_from = once_with(move || formatter::Point {
                point_index: from,
                x: coord.x,
                y: coord.y,
                z: coord.z,
            });
            let coord = params.coords[to];
            let p_to = once_with(move || formatter::Point {
                point_index: to,
                x: coord.x,
                y: coord.y,
                z: coord.z,
            });

            p_from.chain(p_to).skip(if i == 0 { 0 } else { 1 })
        })
        .collect();

    let (energy_cost, time_cost, capacity_cost) = result
        .get_path()
        .iter()
        .map(|p| (p.from(), p.to()))
        .fold((0.0, 0.0, 0.0), |sum, p| {
            (
                sum.0 + cost_matrices.energy[p],
                sum.1 + cost_matrices.time[p],
                sum.2 + cost_matrices.capacity[p],
            )
        });

    let out_data = formatter::OutputData {
        energy: formatter::Matrix(&cost_matrices.energy),
        time: formatter::Matrix(&cost_matrices.time),
        capacity: formatter::Matrix(&cost_matrices.capacity),
        path,
        energy_cost,
        time_cost,
        capacity_cost,
    };

    match (config.out_as_json, config.out_filename.map(File::create)) {
        (true, Some(file)) => out_data.to_writer(file?, JsonFormatter),
        (false, Some(file)) => out_data.to_writer(file?, TextFormatter),
        (true, None) => out_data.to_writer(stdout(), JsonFormatter),
        (false, None) => out_data.to_writer(stdout(), TextFormatter),
    }?;

    Ok(())
}
