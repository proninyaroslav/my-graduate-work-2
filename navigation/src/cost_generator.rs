extern crate nalgebra;
extern crate serde;
extern crate serde_json;
extern crate thiserror;

use nalgebra::DMatrix;
use serde::Deserialize;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use thiserror::Error;

/// Drone parameters and coordinates to be visited
#[derive(Debug, Deserialize)]
pub struct Params {
    pub battery_voltage: f64,

    /// Drone horizontal speed in **[m/s]**
    pub speed_horizontal: f64,

    /// Drone up speed in **[m/s]**
    pub speed_up: f64,

    /// Drone down speed in **[m/s]**
    pub speed_down: f64,

    /// Drone horizontal power consumption in **[W]**
    pub power_horizontal: f64,

    /// Drone up power consumption in **[W]**
    pub power_up: f64,

    /// Drone down power consumption in **[W]**
    pub power_down: f64,

    /// Drone hover power consumption in **[W]**
    pub power_hover: f64,

    /// Required drone hovering time in **[s]**
    pub hover_time: u32,

    /// Coordinates that need to visit
    pub coords: Vec<Point>,
}

#[derive(Debug, Copy, Clone, PartialEq, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Error)]
pub enum ParamsParseError {
    #[error("Cannot open config file: {0}")]
    IO(#[from] std::io::Error),

    #[error("Cannot parse config file: {0}")]
    Deserialize(#[from] serde_json::Error),
}

#[allow(dead_code)]
impl Params {
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Params, ParamsParseError> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);

        Ok(serde_json::from_reader(reader)?)
    }

    pub fn from_str(s: &str) -> Result<Params, ParamsParseError> {
        Ok(serde_json::from_str(s)?)
    }
}

/// Cost matrices (size *N x N*) for each pair of vertices from *i* to *j*.
#[derive(Debug)]
pub struct CostMatrices {
    /// Energy consumption in **kJ**.
    pub energy: DMatrix<f64>,

    /// Flying time in seconds.
    pub time: DMatrix<f64>,

    /// Used battery capacity in **Ah**.
    pub capacity: DMatrix<f64>,

    /// Distance between the visited points for
    /// an intuitive visit (in a circle) in **m**.
    pub distance: DMatrix<f64>,
}

impl CostMatrices {
    const SEC_PER_H: i32 = 3600;

    pub fn generate(params: &Params) -> CostMatrices {
        // Constants that represents the cost of
        // hovering at each visited vertices
        let energy_hover = params.power_hover * params.hover_time as f64;
        let capacity_hover = energy_hover / (params.battery_voltage * Self::SEC_PER_H as f64);

        let n = params.coords.len();
        let mut matrices = CostMatrices {
            energy: DMatrix::repeat(n, n, f64::INFINITY),
            time: DMatrix::repeat(n, n, f64::INFINITY),
            capacity: DMatrix::repeat(n, n, f64::INFINITY),
            distance: DMatrix::repeat(n, n, f64::INFINITY),
        };

        for i in 0..n {
            for j in 0..n {
                if i == j {
                    continue;
                }

                Self::calc_costs(i, j, &mut matrices, params, energy_hover, capacity_hover);
                Self::calc_distance(i, j, &mut matrices, params);
            }
        }

        matrices
    }

    fn calc_costs(
        i: usize,
        j: usize,
        matrices: &mut CostMatrices,
        params: &Params,
        energy_hover: f64,
        capacity_hover: f64,
    ) {
        let coords = &params.coords;

        let (t_ver, power_ver) = if coords[i].z < coords[j].z {
            (
                (coords[j].z - coords[i].z) / params.speed_up,
                params.power_up,
            )
        } else {
            (
                (coords[i].z - coords[j].z) / params.speed_down,
                if coords[i].z == coords[j].z {
                    0.0
                } else {
                    params.power_down
                },
            )
        };
        let t_hor = ((coords[i].x - coords[j].x).powi(2) + (coords[i].y - coords[j].y).powi(2))
            .sqrt()
            / params.speed_horizontal;
        let energy = t_ver * power_ver + t_hor * params.power_horizontal;
        let capacity = energy / (params.battery_voltage * Self::SEC_PER_H as f64);

        let energy_val = (energy + energy_hover) / 1000.0;
        let time_val = t_hor.max(t_ver) + params.hover_time as f64;
        let capacity_val = capacity + capacity_hover;

        matrices.energy[(i, j)] = energy_val;
        matrices.time[(i, j)] = time_val;
        matrices.capacity[(i, j)] = capacity_val;
    }

    fn calc_distance(i: usize, j: usize, matrices: &mut CostMatrices, params: &Params) {
        let x_i = params.coords[i].x;
        let x_j = params.coords[j].x;
        let y_i = params.coords[i].y;
        let y_j = params.coords[j].y;
        matrices.distance[(i, j)] = ((x_i - x_j).powi(2) + (y_i - y_j).powi(2)).sqrt();
    }
}

#[cfg(test)]
#[rustfmt::skip]
mod tests {
    use super::*;
    use float_cmp::{ApproxEq, F64Margin};

    #[test]
    fn parse_params_test() {
        let s = r#"
        {
	        "battery_voltage": 22.8,
	        "speed_horizontal": 12.5,
	        "speed_up": 3.1,
	        "speed_down": 3,
	        "power_horizontal": 486.2,
	        "power_up": 899.04,
	        "power_down": 309.17,
	        "power_hover": 545.8,
	        "hover_time": 10,
	        "coords": [{"x": 0, "y": 0, "z": 0},
			           {"x": 10, "y": 200, "z": 300},
			           {"x": 200, "y": 450, "z": 12},
			           {"x": 400, "y": 460, "z": 350},
			           {"x": 350, "y": 240, "z": 14},
                       {"x": 450, "y": 100, "z": 200}
            ]
        }
        "#;
        let params = Params::from_str(s).unwrap();
        assert_eq!(22.8, params.battery_voltage);
        assert_eq!(12.5, params.speed_horizontal);
        assert_eq!(3.1, params.speed_up);
        assert_eq!(3.0, params.speed_down);
        assert_eq!(486.2, params.power_horizontal);
        assert_eq!(899.04, params.power_up);
        assert_eq!(309.17, params.power_down);
        assert_eq!(545.8, params.power_hover);
        assert_eq!(10, params.hover_time);
        assert_eq!(vec!(
            Point { x: 0.0, y: 0.0, z: 0.0 },
            Point { x: 10.0, y: 200.0, z: 300.0 },
            Point { x: 200.0, y: 450.0, z: 12.0 },
            Point { x: 400.0, y: 460.0, z: 350.0 },
            Point { x: 350.0, y: 240.0, z: 14.0 },
            Point { x: 450.0, y: 100.0, z: 200.0 },
        ),
        params.coords
        );
    }

    #[test]
    #[should_panic]
    fn invalid_params_test() {
        let s = r#"{ "foo": "bar" }"#;
        Params::from_str(s).unwrap();
    }

    #[test]
    #[rustfmt::skip]
    fn generate_test() {
        let s = r#"
        {
	        "battery_voltage": 22.8,
	        "speed_horizontal": 12.5,
	        "speed_up": 3.1,
	        "speed_down": 3,
	        "power_horizontal": 486.2,
	        "power_up": 899.04,
	        "power_down": 309.17,
	        "power_hover": 545.8,
	        "hover_time": 0,
	        "coords": [{"x": 0, "y": 0, "z": 0},
			           {"x": 10, "y": 200, "z": 300},
			           {"x": 200, "y": 450, "z": 12},
			           {"x": 400, "y": 460, "z": 350},
			           {"x": 350, "y": 240, "z": 14},
                       {"x": 450, "y": 100, "z": 200}
            ]
        }
        "#;

        let energy_cost = DMatrix::from_vec(6, 6, vec![
            f64::INFINITY, 94.793, 22.634, 125.215, 20.567, 75.933,
            38.706, f64::INFINITY, 41.894, 32.732, 42.790, 27.856,
            20.391, 95.737, f64::INFINITY, 105.813, 10.618, 71.252,
            59.780, 23.384, 42.622, f64::INFINITY, 43.402, 29.595,
            17.950, 96.260, 10.244, 106.220, f64::INFINITY, 60.634,
            38.542, 46.552, 36.104, 57.639, 25.860, f64::INFINITY
        ]).transpose();

        let time_cost = DMatrix::from_vec(6, 6, vec![
            f64::INFINITY, 96.774, 39.395, 112.903, 33.951, 64.516,
            100.000, f64::INFINITY, 96.000, 37.498, 95.333, 36.098,
            39.395, 92.903, f64::INFINITY, 109.032, 20.646, 60.645,
            116.667, 37.498, 112.667, f64::INFINITY, 112.000, 50.000,
            33.951, 92.258, 20.646, 108.387, f64::INFINITY, 60.000,
            66.667, 36.098, 62.667, 48.387, 62.000, f64::INFINITY
        ]).transpose();

        let capacity_cost = DMatrix::from_vec(6, 6, vec![
            f64::INFINITY, 1.155, 0.276, 1.526, 0.251, 0.925,
            0.472, f64::INFINITY, 0.510, 0.399, 0.521, 0.339,
            0.248, 1.166, f64::INFINITY, 1.289, 0.129, 0.868,
            0.728, 0.285, 0.519, f64::INFINITY, 0.529, 0.361,
            0.219, 1.173, 0.125, 1.294, f64::INFINITY, 0.739,
            0.470, 0.567, 0.440, 0.702, 0.315, f64::INFINITY
        ]).transpose();

        let params = Params::from_str(s).unwrap();
        let matrices = CostMatrices::generate(&params);

        assert!(approx_eq!(Matrix, Matrix(energy_cost), Matrix(matrices.energy)));
        assert!(approx_eq!(Matrix, Matrix(time_cost), Matrix(matrices.time)));
        assert!(approx_eq!(Matrix, Matrix(capacity_cost), Matrix(matrices.capacity)));
    }

    pub struct Matrix(DMatrix<f64>);

    impl From<Matrix> for DMatrix<f64> {
        fn from(m: Matrix) -> DMatrix<f64> {
            m.0
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub struct MatrixMargin {
        pub epsilon: f64,
        pub ulps: i64,
    }

    impl Default for MatrixMargin {
        #[inline]
        fn default() -> MatrixMargin {
            MatrixMargin {
                epsilon: 0.001,
                ulps: 0,
            }
        }
    }

    impl From<MatrixMargin> for F64Margin {
        fn from(m: MatrixMargin) -> F64Margin {
            F64Margin {
                epsilon: m.epsilon,
                ulps: m.ulps,
            }
        }
    }

    impl ApproxEq for Matrix {
        type Margin = MatrixMargin;

        fn approx_eq<M: Into<Self::Margin>>(self, other: Self, margin: M) -> bool {
            let margin = margin.into();
            for (i1, i2) in self.0.iter().zip(other.0.iter()) {
                if !i1.approx_eq(*i2, margin) {
                    return false;
                }
            }

            true
        }
    }
}
