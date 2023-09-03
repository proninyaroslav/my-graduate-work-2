extern crate nalgebra;
extern crate serde;
extern crate serde_json;
extern crate thiserror;
use thiserror::Error;

use nalgebra::DMatrix;
use serde::ser::SerializeSeq;
use serde::{Serialize, Serializer};
use std::fmt::{Display, Formatter};
use std::io::Write;

#[derive(Debug, Error)]
pub enum Error {
    #[error("Cannot save result as JSON: {0}")]
    SerializeJson(#[from] serde_json::Error),

    #[error("Cannot save result: {0}")]
    IO(#[from] std::io::Error),
}

type Result = std::result::Result<(), Error>;

#[derive(Debug, Serialize)]
pub struct OutputData<'a> {
    pub energy: Matrix<'a>,
    pub time: Matrix<'a>,
    pub capacity: Matrix<'a>,
    pub path: Vec<Point>,
    pub energy_cost: f64,
    pub time_cost: f64,
    pub capacity_cost: f64,
}

impl<'a> OutputData<'_> {
    pub fn to_writer<W, F>(&self, writer: W, formatter: F) -> Result
    where
        W: Write,
        F: OutputFormatter<W>,
    {
        formatter.fmt(self, writer)
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Serialize)]
pub struct Point {
    #[serde(skip_serializing)]
    pub point_index: usize,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Display for Point {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}: ({}, {}, {})",
            self.point_index + 1,
            self.x,
            self.y,
            self.z
        )
    }
}

#[derive(Debug, Clone)]
pub struct Matrix<'a>(pub &'a DMatrix<f64>);

impl<'a> Display for Matrix<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
}

impl<'a> Serialize for Matrix<'_> {
    fn serialize<S>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut seq = serializer.serialize_seq(Some(self.0.nrows()))?;
        for row in self.0.row_iter() {
            let row: Vec<&f64> = row.iter().collect();
            seq.serialize_element(&row)?
        }

        seq.end()
    }
}

pub trait OutputFormatter<W: Write> {
    fn fmt(&self, data: &OutputData<'_>, writer: W) -> Result;
}

pub struct TextFormatter;

impl<W: Write> OutputFormatter<W> for TextFormatter {
    fn fmt(&self, data: &OutputData<'_>, writer: W) -> Result {
        let mut writer = writer;
        writeln!(
            writer,
            "Row - departure point, cloumn - destination point\n"
        )?;
        write!(writer, "Energy:{:.3}", data.energy)?;
        write!(writer, "Capacity:{:.3}", data.capacity)?;
        write!(writer, "Time:{:.3}", data.time)?;
        writeln!(writer, "Path:")?;
        for p in &data.path {
            writeln!(writer, "{:.3}", p)?;
        }
        writeln!(writer, "\nEnergy: {:.3} kJ", data.energy_cost)?;
        writeln!(writer, "Capacity: {:.3} Ah", data.capacity_cost)?;
        writeln!(writer, "Time: {:.3} s", data.time_cost)?;

        Ok(writer.flush()?)
    }
}

pub struct JsonFormatter;

impl<W: Write> OutputFormatter<W> for JsonFormatter {
    fn fmt(&self, data: &OutputData<'_>, writer: W) -> Result {
        Ok(serde_json::to_writer_pretty(writer, data)?)
    }
}
