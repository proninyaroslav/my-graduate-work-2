extern crate min_max_heap;
extern crate nalgebra;
extern crate ordered_float;

use min_max_heap::MinMaxHeap;
use nalgebra::DMatrix;
use ordered_float::OrderedFloat;
use std::cmp::{Eq, Ord, Ordering, PartialEq, PartialOrd};
use std::ops::AddAssign;

#[derive(Debug)]
pub struct FindResult {
    cost: f64,
    path: Vec<PathEdge>,
}

#[allow(dead_code)]
impl FindResult {
    pub fn get_cost(&self) -> f64 {
        self.cost
    }

    pub fn get_path(&self) -> &Vec<PathEdge> {
        &self.path
    }
}

/// Contains cost matrix indices vector *(from, to)*.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub struct PathEdge(usize, usize);

impl PathEdge {
    pub fn from(&self) -> usize {
        self.0
    }

    pub fn to(&self) -> usize {
        self.1
    }
}

#[derive(Debug)]
/// A an object that corresponds to visiting vertex *j* from vertex *i*
/// and contains the data calculated at this step.
struct Node {
    path: Vec<PathEdge>,
    reduced_matrix: DMatrix<f64>,
    cost: NodePriority,
    vertex: usize,
    level: usize,
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        self.cost.cmp(&other.cost)
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cost.cmp(&other.cost))
    }
}

impl Eq for Node {}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Node {
    fn new(
        reduced_matrix: DMatrix<f64>,
        level: usize,
        i: usize,
        j: usize,
        path: Vec<PathEdge>,
    ) -> Node {
        let mut path = path.to_vec();
        // Add current edge to path. Skip for root node
        if level != 0 {
            path.push(PathEdge(i, j));
        }

        let mut reduced_matrix = reduced_matrix;
        // Change all entries of row i and column j to infinity.
        // Skip root node
        if level != 0 {
            reduced_matrix.fill_row(i, f64::INFINITY.into());
            reduced_matrix.fill_column(j, f64::INFINITY.into());
        }

        // Start node is 0
        reduced_matrix[(j, 0)] = f64::INFINITY.into();
        let cost = Self::calculate_expected_cost(&mut reduced_matrix).into();

        Node {
            path,
            reduced_matrix,
            cost,
            vertex: j,
            level,
        }
    }

    /// Calculate the lower bound of the path starting at current min node.
    fn calculate_expected_cost(reduced_matrix: &mut DMatrix<f64>) -> f64 {
        // Subtracts the minimum element of each row/column
        // from each element of this row/column
        let subtract = |f: &mut f64, min: f64| {
            if *f != f64::INFINITY && min != f64::INFINITY {
                *f -= min
            }
        };
        let row_min_elems = reduced_matrix
            .row_iter_mut()
            .map(|mut row| {
                let min = row.min();
                row.iter_mut().for_each(|f| subtract(f, min));
                min
            })
            .collect::<Vec<f64>>();

        let col_min_elems = reduced_matrix
            .column_iter_mut()
            .map(|mut col| {
                let min = col.min();
                col.iter_mut().for_each(|f| subtract(f, min));
                min
            })
            .collect::<Vec<f64>>();

        // The total expected cost is the sum of all reductions
        row_min_elems
            .iter()
            .zip(col_min_elems.iter())
            .fold(0.0, |cost, (&row, &col)| {
                let row = if row == f64::INFINITY { 0.0 } else { row };
                let col = if col == f64::INFINITY { 0.0 } else { col };
                cost + row + col
            })
    }
}

#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq)]
struct NodePriority(OrderedFloat<f64>);

impl NodePriority {
    fn new(p: f64) -> NodePriority {
        NodePriority(p.into())
    }
}

impl From<f64> for NodePriority {
    fn from(f: f64) -> Self {
        NodePriority::new(f)
    }
}

impl Into<f64> for NodePriority {
    fn into(self) -> f64 {
        self.0.into_inner()
    }
}

impl AddAssign<f64> for NodePriority {
    fn add_assign(&mut self, other: f64) {
        self.0 = OrderedFloat(self.0.into_inner() + other);
    }
}

/// Solves the traveling salesman problem for a given cost matrix.
pub fn find(cost: &DMatrix<f64>) -> Option<FindResult> {
    let mut queue = MinMaxHeap::new();
    let root = Node::new(cost.clone(), 0, 0, 0, Vec::new());
    queue.push(root);

    // Finds node with least cost, add its children to list of
    // nodes and finally deletes it from the list
    let n = cost.nrows();
    while let Some(mut min) = queue.pop_min() {
        let i = min.vertex;
        // All vertex are visited
        if min.level == n - 1 {
            // Go back to starting vertex
            min.path.push(PathEdge(i, 0));

            return Some(FindResult {
                cost: min.cost.into(),
                path: min.path,
            });
        }

        for (j, col) in min.reduced_matrix.row(i).column_iter().enumerate() {
            let col_val: f64 = col[0].into();
            if col_val == f64::INFINITY {
                continue;
            }

            let mut child = Node::new(
                min.reduced_matrix.clone(),
                min.level + 1,
                i,
                j,
                min.path.to_vec(),
            );
            let min_cost: f64 = min.cost.into();

            child.cost += min_cost + col_val;
            queue.push(child);
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_priority_test() {
        let mut priorities = [
            NodePriority::new(1.0),
            NodePriority::new(2.0),
            NodePriority::new(1.1),
            NodePriority::new(3.5),
            NodePriority::new(2.005),
        ];
        let sorted = [
            NodePriority::new(1.0),
            NodePriority::new(1.1),
            NodePriority::new(2.0),
            NodePriority::new(2.005),
            NodePriority::new(3.5),
        ];
        priorities.sort_by(|a, b| a.cmp(b));

        assert_eq!(sorted, priorities);
    }

    #[test]
    #[rustfmt::skip]
    fn find_test() {
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

        let energy_expected_path = vec![
            PathEdge(0, 2),
            PathEdge(2, 4),
            PathEdge(4, 5),
            PathEdge(5, 3),
            PathEdge(3, 1),
            PathEdge(1, 0),
        ];

        let time_expected_path = vec![
            PathEdge(0, 5),
            PathEdge(5, 3),
            PathEdge(3, 1),
            PathEdge(1, 2),
            PathEdge(2, 4),
            PathEdge(4, 0),
        ];

        let capacity_expected_path = vec![
            PathEdge(0, 2),
            PathEdge(2, 4),
            PathEdge(4, 5),
            PathEdge(5, 3),
            PathEdge(3, 1),
            PathEdge(1, 0),
        ];

        let result = find(&energy_cost).unwrap();
        assert!(approx_eq!(f64, 213.615, result.get_cost(), epsilon = 0.001, ulps = 0));
        assert_eq!(energy_expected_path, *result.get_path());

        let result = find(&time_cost).unwrap();
        assert!(approx_eq!(f64, 300.997, result.get_cost(), epsilon = 0.001, ulps = 0));
        assert_eq!(time_expected_path, *result.get_path());

        let result = find(&capacity_cost).unwrap();
        assert!(approx_eq!(f64, 2.603, result.get_cost(), epsilon = 0.001, ulps = 0));
        assert_eq!(capacity_expected_path, *result.get_path());
    }
}
