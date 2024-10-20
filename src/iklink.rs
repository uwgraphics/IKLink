
use std::vec;

use crate::spacetime::robot::Robot;
use nalgebra::{UnitQuaternion, Vector3};
use crate::spacetime::motion::Motion;
use linfa_clustering::Dbscan;
use linfa::traits::*;
use ndarray::Array1;
use crate::nodes::Node::Node;
use crate::utils::vec_of_arrays_to_2d_array;

pub struct IKLink<N: Node> {
    pub robot: Robot,
    pub trajectory: Vec<(f64, Vector3<f64>, UnitQuaternion<f64>)>,

    pub table: Vec<Vec<N>>,

    pub rng: rand::prelude::ThreadRng,
}

impl <N:Node> IKLink<N> {
    fn dp(&mut self) -> Motion{
        assert!(self.trajectory.len() == self.table.len());

        println!("Running dynamic programming algorithm");

        let n = self.trajectory.len();

        // first column
        for y in 0..self.table[0].len() {
            self.table[0][y].reset_for_first_col();
        }

        // rest of the columns
        for x in 1..n {
            for y1 in 0..self.table[x].len() {
                let mut node = self.table[x][y1].clone();
                for y2 in 0..self.table[x-1].len() {
                    node.try_to_connect(  &self.table[x-1][y2], (x-1,y2), &self.robot);
                }
                self.table[x][y1] = node;
            }            
        }

        // find best score in the last column
        let mut best_node = self.table[n-1][0].clone();
        for j in 1..self.table[n-1].len() {
            best_node.compare_and_update_node(&self.table[n-1][j]);
        }

        println!("iklink performance: {}", best_node.get_performance());

        let mut motion = Motion {
            robot_name: self.robot.robot_name.clone(),
            joint_names: self.robot.ik_solver.vars.robot.joint_names.clone(),
            data: vec![],
        };

        // backtrace
        loop {
            motion.data.push((best_node.get_time(), best_node.get_ik().clone()));
            if !best_node.has_predecessor() {
                break;
            }
            let p = best_node.get_predecessor();
            best_node = self.table[p.0][p.1].clone();
        }

       
        motion.data.reverse();
        
        motion
    }

    
    fn sample_candidates(&mut self, num_samples: usize) {

        let n = self.trajectory.len();

        let mut tmp_ik_table: Vec<Vec<Array1<f64>>> = Vec::new();
        for _ in 0..n {
            tmp_ik_table.push(vec![]);
            self.table.push(vec![]);
        }

        for i in 0..n {

            println!("Constructing nodes for point {} / {}", i, n);

            // // clustering IK solutions using DBSCAN
            let tmp_iks = vec_of_arrays_to_2d_array(&mut tmp_ik_table[i]);
            let clusters = Dbscan::params(2).tolerance(0.01).transform(&tmp_iks).unwrap();

            assert!(clusters.shape()[0] == tmp_ik_table[i].len());

            let mut labels = vec![false; clusters.shape()[0]];

            for j in 0..clusters.shape()[0] {
                match clusters[j] {
                    Some(cluster_idx) => {
                        if !labels[cluster_idx] {
                            labels[cluster_idx] = true;
                            let node = Node::new(tmp_ik_table[i][j].clone(), self.trajectory[i].0, "clustering".to_string());
                            self.table[i].push(node);
                        }
                    },
                    None => {
                        let node = Node::new(tmp_ik_table[i][j].clone(), self.trajectory[i].0, "clustering".to_string());
                        self.table[i].push(node);
                    }
                }
            }

            // random sampling
            while self.table[i].len() < num_samples {
                self.robot.reset_random();  
                let (found_ik, ik) = self.robot.try_to_reach(self.trajectory[i].1, self.trajectory[i].2);
                if !found_ik {
                    continue;
                }
                let node = Node::new(ik, self.trajectory[i].0, "random".to_string());
                self.table[i].push(node);
            }

            // greedy propagation
            if i < n-1 {
                for j in 0..self.table[i].len() {
                    self.robot.ik_solver.reset(self.table[i][j].get_ik().to_vec());
                    let (found_ik, ik) = self.robot.try_to_track(self.trajectory[i+1].1, self.trajectory[i+1].2);
                    if !found_ik {
                        continue;
                    }
                    tmp_ik_table[i+1].push(Array1::from(ik));
                }
            }

        }
    }

    pub fn new(robot_name: &str, traj: &Vec<(f64, Vector3<f64>, UnitQuaternion<f64>)>) -> Self {
        
        let robot = Robot::new(robot_name);

        let table = vec![];

        IKLink{
            robot,
            trajectory:  traj.clone(),
            table,
            rng: rand::thread_rng(),
        }
    }

    pub fn solve(&mut self, num_samples: usize) -> Motion{

        self.sample_candidates(num_samples);
        self.dp()

    }
   
}