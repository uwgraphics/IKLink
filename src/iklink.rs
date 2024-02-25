
use std::vec;

use crate::spacetime::robot::Robot;
use nalgebra::{UnitQuaternion, Vector3};
use crate::spacetime::motion::Motion;
use linfa_clustering::Dbscan;
use linfa::traits::*;
use ndarray::{Array1, Array2};

pub struct Node {
    pub ik: Array1<f64>,
    pub primary_score: f64,
    pub secondary_score: f64,
    pub predecessor: usize,
}

impl Node {
    pub fn new(ik: Array1<f64>) -> Self {
        
        let primary_score = 100000.0;
        let secondary_score = 100000.0;
        let predecessor = 0;

        Node {
            ik,
            primary_score,
            secondary_score,
            predecessor,
        }
    }
}

pub struct IKLink {
    pub robot: Robot,
    pub trajectory: Vec<(f64, Vector3<f64>, UnitQuaternion<f64>)>,

    pub table: Vec<Vec<Node>>,

    pub rng: rand::prelude::ThreadRng,
}

impl IKLink {
    fn dp(&mut self) -> Motion{
        assert!(self.trajectory.len() == self.table.len());

        println!("Running dynamic programming algorithm");

        let n = self.trajectory.len();

        // first column
        for y in 0..self.table[0].len() {
            self.table[0][y].primary_score = 0.0;
            self.table[0][y].secondary_score = 0.0;
            self.table[0][y].predecessor = 0;
        }

        // rest of the columns
        for x in 1..n {

            let delta_t = self.trajectory[x].0 - self.trajectory[x-1].0;

            let mut min_primary_score_with_config = 100000.0;
            let mut min_secondary_score_with_config = 100000.0;
            let mut min_idx_with_config: usize = 0;

            // find best predecessor with an arm reconfiguration
            for y2 in 0..self.table[x-1].len() {
                let primary_score = self.table[x-1][y2].primary_score + 1.0;
                let secondary_score = self.table[x-1][y2].secondary_score;

                if primary_score < min_primary_score_with_config || (primary_score == min_primary_score_with_config && secondary_score < min_secondary_score_with_config) {
                    min_primary_score_with_config = primary_score;
                    min_secondary_score_with_config = secondary_score;
                    min_idx_with_config = y2;
                }
            }

            // find best predecessor with no arm reconfiguration
            for y1 in 0..self.table[x].len() {
                let mut min_primary_score = min_primary_score_with_config;
                let mut min_secondary_score = min_secondary_score_with_config;
                let mut predecessor = min_idx_with_config;
                for y2 in 0..self.table[x-1].len() {

                    if self.robot.check_velocity(&self.table[x][y1].ik, &self.table[x-1][y2].ik, delta_t) {
                        let primary_score = self.table[x-1][y2].primary_score;
                        let secondary_score = self.table[x-1][y2].secondary_score + self.robot.joint_movement(&self.table[x][y1].ik, &self.table[x-1][y2].ik);
                        if primary_score < min_primary_score {
                            min_primary_score = primary_score;
                            min_secondary_score = secondary_score;
                            predecessor = y2;
                        }
                    }
                }
                self.table[x][y1].primary_score = min_primary_score;
                self.table[x][y1].secondary_score = min_secondary_score;
                self.table[x][y1].predecessor = predecessor;
            }            
        }

        // find best score in the last column
        let mut best_primary_score = 100000.0;
        let mut best_secondary_score = 100000.0;
        let mut best_idx = 0;
        for j in 0..self.table[n-1].len() {
            let primary_score = self.table[n-1][j].primary_score;
            let secondary_score = self.table[n-1][j].secondary_score;
            if primary_score < best_primary_score || (primary_score == best_primary_score && secondary_score < best_secondary_score) {
                best_primary_score = primary_score;
                best_secondary_score = secondary_score;
                best_idx = j;
            }
        }

        assert!(best_primary_score < 100000.0, "No valid solution found!");
        println!("Min Num of Reconfig: {}", best_primary_score);

        let mut motion = Motion {
            robot_name: self.robot.robot_name.clone(),
            joint_names: self.robot.ik_solver.vars.robot.joint_names.clone(),
            data: vec![],
        };

        // backtrace
        let mut idx: usize = best_idx;
        for i in (0..n).rev() {
            motion.data.push((self.trajectory[i].0, self.table[i][idx].ik.clone()));
            idx = self.table[i][idx].predecessor;
        }

        motion.data.reverse();
        
        motion
    }


    fn vec_of_arrays_to_2d_array(&self, vec: &mut Vec<Array1<f64>>) -> Array2<f64> {
        if vec.is_empty() {
            return Array2::zeros((0, 0)); // Return an empty 2D array if the input vector is empty
        }

        let nrows = vec.len(); // Number of rows in the 2D array
        let ncols = vec[0].len(); // Number of columns in the 2D array, assuming all 1D arrays are the same size

        // Initialize a 2D array with zeros
        let mut array2d = Array2::<f64>::zeros((nrows, ncols));

        for (i, array) in vec.into_iter().enumerate() {
            // Make sure each 1D array is the correct size; this example does not handle errors
            assert_eq!(array.len(), ncols, "All arrays must have the same size");

            // Copy the elements from the 1D array into the corresponding row of the 2D array
            array2d.row_mut(i).assign(&array);
        }

        array2d
    }

    fn sample_candidates(&mut self) {

        let n = self.trajectory.len();

        let mut tmp_ik_table: Vec<Vec<Array1<f64>>> = Vec::new();
        for _ in 0..n {
            tmp_ik_table.push(vec![]);
            self.table.push(vec![]);
        }

        for i in 0..n {

            println!("Constructing nodes for point {} / {}", i, n);

            // clustering IK solutions using DBSCAN
            let tmp_iks = self.vec_of_arrays_to_2d_array(&mut tmp_ik_table[i]);
            let clusters = Dbscan::params(2).tolerance(0.05).transform(&tmp_iks).unwrap();

            for j in 0..clusters.shape()[0] {
                let node = Node::new(tmp_ik_table[i][j].clone());
                self.table[i].push(node);
            }

            while self.table[i].len() < 300 {
                // random sampling
                let (found_ik, ik) = self.robot.try_to_reach(self.trajectory[i].1, self.trajectory[i].2);
                if !found_ik {
                    continue;
                }
                let node = Node::new(ik);
                self.table[i].push(node);

                // greedy propagation
                let mut r = i;
                while r < n {
                    let (found_ik, ik) = self.robot.try_to_track(self.trajectory[r].1, self.trajectory[r].2);
                    if !found_ik {
                        break;
                    }
                    tmp_ik_table[r].push(Array1::from(ik));
                    r += 1;
                }
            }

        }
    }

    pub fn new(robot_name: &str, traj: Vec<(f64, Vector3<f64>, UnitQuaternion<f64>)>) -> Self {
        
        let robot = Robot::new(robot_name);

        let table = vec![];

        IKLink {
            robot,
            trajectory:  traj.clone(),
            table,
            rng: rand::thread_rng(),
        }
    }

    pub fn solve(&mut self ) -> Motion{

        self.sample_candidates();
        self.dp()

    }
   
}