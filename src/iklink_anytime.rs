
use std::time::Instant;
use std::vec;
use rand::prelude::*;
use rand::distributions::WeightedIndex;

use crate::spacetime::robot::Robot;
use nalgebra::{UnitQuaternion, Vector3};
use crate::spacetime::motion::Motion;
use linfa_clustering::Dbscan;
use linfa::traits::*;
use ndarray::Array1 ;
use crate::nodes::NodeAnytime::NodeAnytime;
use crate::utils::vec_of_arrays_to_2d_array;

fn lerp_config(a: &Array1<f64>, b: &Array1<f64>, t: f64) -> Array1<f64> {
    let mut res = Array1::zeros(a.len());
    for i in 0..a.len() {
        res[i] = a[i] + t * (b[i] - a[i]);
    }
    res
}
pub struct IKLinkAnytime<N: NodeAnytime> {
    pub robot: Robot,
    pub trajectory: Vec<(f64, Vector3<f64>, UnitQuaternion<f64>)>,

    pub table: Vec<Vec<N>>,
    pub connectability: Vec<Vec<Vec<bool>>>,
    pub temporal_step: usize,
    pub sparse_node_per_waypoint: usize,

    pub num_hard_sparse: Vec<usize>,

    pub num_linear_nodes: Vec<usize>,
    pub num_random_nodes : Vec<usize>,

    pub rng: rand::prelude::ThreadRng,
}
impl <N:NodeAnytime> IKLinkAnytime<N>  {
    fn motion_from_best_node(&self, best_node: &N) -> Motion {
        let mut motion = Motion {
            robot_name: self.robot.robot_name.clone(),
            joint_names: self.robot.ik_solver.vars.robot.joint_names.clone(),
            data: vec![],
        };

        let mut node = best_node.clone();

        // backtrace
        loop {
            motion.data.push((node.get_time(), node.get_ik().clone()));
            if !node.has_predecessor() {
                break;
            }
            
            let p = node.get_predecessor();

            node = self.table[p.0][p.1].clone();
        }

        motion.data.reverse();
        
        motion
    }

    fn dp(&mut self) -> Motion{
        assert!(self.trajectory.len() == self.table.len(), "trajectory.len() : {} != table.len() : {}", self.trajectory.len(), self.table.len());

        // println!("Running dynamic programming algorithm");

        let n = self.trajectory.len();

        // first column
        for y in 0..self.table[0].len() {
            self.table[0][y].reset_scores_for_first_col();
        }
        // rest of the columns
        for x in 1..n {
            for y1 in 0..self.table[x].len() {
                self.table[x][y1].reset_scores_for_other_col();
                let mut node = self.table[x][y1].clone();
                for y2 in 0..self.table[x-1].len() {
                   node.try_to_connect( &self.table[x-1][y2], (x-1, y2), &self.robot);
                   node.check_reachability(&self.table[x-1][y2], (x-1, y2), &self.robot);
                }
                self.table[x][y1].copy_from(&node);
            }            
        }

        // find best score in the last column
        let mut best_node = self.table[n-1][0].clone();
        for j in 1..self.table[n-1].len() {
            best_node.compare_and_update_node(&self.table[n-1][j]);
        }

        println!("DP -- {}", best_node.get_performance());

        self.motion_from_best_node(&best_node)
    }

    fn pre_sample_candidates(&mut self, range: std::iter::Chain<std::iter::StepBy<std::ops::Range<usize>>, std::iter::Once<usize>>, num_samples: usize, max_num_samples: usize) {

        let n = self.trajectory.len();

        let mut tmp_ik_table: Vec<Array1<f64>> = Vec::new();

        let mut pre_i = 0;

        let start_i = range.clone().next().unwrap();

        for i in range {

            if self.table[i].len() >= max_num_samples {
                continue;
            }

            let mut num_new_nodes = 0;

            if i != start_i {
                // greedy propagation
                tmp_ik_table.clear();
                
                // println!("Greedy propagation from {} to {}", pre_i, i);
                let start_j = if self.table[pre_i].len() > num_samples  { self.table[pre_i].len() - num_samples } else { 0 };
                for j in start_j..self.table[pre_i].len() {
                    self.robot.ik_solver.reset(self.table[pre_i][j].get_ik().to_vec());
                    let (found_ik, ik) = self.robot.try_to_track(self.trajectory[i].1, self.trajectory[i].2);
                    if !found_ik {
                        continue;
                    }
                    tmp_ik_table.push(Array1::from(ik));
                }

                // clustering IK solutions using DBSCAN
                let tmp_iks = vec_of_arrays_to_2d_array(&mut tmp_ik_table);
                let clusters = Dbscan::params(2).tolerance(0.05).transform(&tmp_iks).unwrap();

                assert!(clusters.shape()[0] == tmp_ik_table.len());

                let mut labels = vec![false; clusters.shape()[0]];

                for j in 0..clusters.shape()[0] {
                    match clusters[j] {
                        Some(cluster_idx) => {
                            if !labels[cluster_idx] {
                                labels[cluster_idx] = true;
                                let node = N::new(tmp_ik_table[j].clone(), self.trajectory[i].0, "sparse_init".to_string(), (i, self.table[i].len()));
                                self.table[i].push(node);
                                self.num_random_nodes[i] += 1;
                                num_new_nodes += 1;
                            }
                        },
                        None => {
                            let node = N::new(tmp_ik_table[j].clone(), self.trajectory[i].0, "sparse_init".to_string(), (i, self.table[i].len()));
                            self.table[i].push(node);
                            self.num_random_nodes[i] += 1;
                            num_new_nodes += 1;
                        }
                    }
                }
            }

            // random sampling
            while num_new_nodes < num_samples {
                self.robot.reset_random();
                let (found_ik, ik) = self.robot.try_to_reach(self.trajectory[i].1, self.trajectory[i].2);
                if !found_ik {
                    continue;
                }
                let node = N::new(ik, self.trajectory[i].0, "sparse_init".to_string(), (i, self.table[i].len()));
                self.table[i].push(node);
                self.num_random_nodes[i] += 1;
                num_new_nodes += 1;
            }

            println!("Sampled {} IK solutions for point {} / {}", self.table[i].len(), i, n-1);
            pre_i = i; 
        }
    }
    
    fn pre_dp(&mut self, step: usize, no_sparse: bool) -> N{
        assert!(self.trajectory.len() == self.table.len());

        // println!("Running pre dynamic programming algorithm");

        let n = self.trajectory.len();

        // first column
        for y in 0..self.table[0].len() {
            self.table[0][y].reset_scores_for_first_col();
        }

        // rest of the columns
        for x in 1..n{
            let pre_x = x - 1;

            let mut sparse_pre_x: usize = 0;
            let mut have_sparse_predecessor = false;

            if x >= step  {
                if x == n-1 && x % step != 0 {
                    sparse_pre_x = x - x % step;
                } else {
                    sparse_pre_x = x - step;
                }
                if self.table[sparse_pre_x].len() > 0 {
                    have_sparse_predecessor = true;
                }
            }

            for y1 in 0..self.table[x].len() {
                self.table[x][y1].reset_scores_for_other_col();

                let mut node = self.table[x][y1].clone();

                // dense graph
                for y2 in 0..self.table[pre_x].len() {
                    node.try_to_connect(  &self.table[pre_x][y2], (pre_x, y2), &self.robot);
                    node.check_reachability(&self.table[pre_x][y2], (pre_x, y2), &self.robot);
                }

                // sparse graph
                if have_sparse_predecessor && !no_sparse {

                    let mut flags = self.table[x][y1].get_is_reachable_from().clone();
                    let mut cost = self.table[x][y1].get_distance_from().clone();
                    let mut i = x-1;

                    while i > sparse_pre_x {

                        let mut tmp_flag = vec![];
                        let mut tmp_cost = vec![];

                        for j in 0..self.table[i].len(){
                            if  j < flags.len() && flags[j] {
                                if tmp_flag.len() == 0 {
                                    for k in 0..self.table[i][j].get_is_reachable_from().len() {
                                        tmp_flag.push(self.table[i][j].get_is_reachable_from()[k]);
                                        tmp_cost.push(self.table[i][j].get_distance_from()[k] + cost[j]);
                                    }
                                } else {
                                    assert!(tmp_flag.len() == self.table[i][j].get_is_reachable_from().len());
                                    for k in 0..tmp_flag.len() {
                                        if self.table[i][j].get_is_reachable_from()[k] {
                                            if !tmp_flag[k] || tmp_cost[k] > self.table[i][j].get_distance_from()[k] + cost[j] {
                                                tmp_flag[k] = true;
                                                tmp_cost[k] = self.table[i][j].get_distance_from()[k] + cost[j];
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        flags = tmp_flag;
                        cost = tmp_cost;
                        i -= 1;
                    }

                    for y2 in 0..self.table[sparse_pre_x].len() {

                        let sparse_cost: f64 = self.robot.joint_movement(&self.table[sparse_pre_x][y2].get_ik(), &self.table[x][y1].get_ik());

                        if y2 < flags.len() && flags[y2] && cost[y2] < sparse_cost * 1.1 {
                            continue;
                        }
                        
                        node.try_to_connect(&self.table[sparse_pre_x][y2], (sparse_pre_x, y2), &self.robot);
                    }
                }

                self.table[x][y1] = node;
            }         
        }

        // find best score in the last column
        let mut best_node = self.table[n-1][0].clone();
        for j in 1..self.table[n-1].len() {
            best_node.compare_and_update_node(&self.table[n-1][j]);
        }

        println!("Pre DP -- {}", best_node.get_performance());

        return best_node;
    }

    pub fn new(robot_name: &str, traj: &Vec<(f64, Vector3<f64>, UnitQuaternion<f64>)>) -> Self {
        
        let robot = Robot::new(robot_name);

        IKLinkAnytime {
            robot,
            trajectory:  traj.clone(),
            table:  vec![],
            connectability: vec![],
            temporal_step: 0,
            sparse_node_per_waypoint: 0,
            rng: rand::thread_rng(),
            num_hard_sparse: vec![],

            num_linear_nodes: vec![],
            num_random_nodes: vec![],
        }
    }

    fn sample_candidates(&mut self, sparse_traj_last_node: N, num_samples_aound_keypoints: usize, iteration: usize, max_num_samples: usize) {

        let n = self.trajectory.len();
        assert!(sparse_traj_last_node.get_time() == self.trajectory[n-1].0);
        assert!(sparse_traj_last_node.get_indexes().0 == n-1);

        let mut curr_node = sparse_traj_last_node.clone();

        let start = Instant::now();

        while curr_node.has_predecessor() {

            let predecessor_idx = curr_node.get_predecessor();

            let pre_node = self.table[predecessor_idx.0][predecessor_idx.1].clone();

            // dense
            if curr_node.get_indexes().0 - 1 == predecessor_idx.0 {
                curr_node = pre_node;
                continue;
            }

            // println!("sparse edge: {}-{} -> {}-{}", pre_node.get_indexes().0, pre_node.get_indexes().1, curr_node.get_indexes().0, curr_node.get_indexes().1);

            // sparse
            let l = pre_node.get_indexes().0;
            let r = curr_node.get_indexes().0;
            let l_idx = pre_node.get_indexes().1;
            let r_idx = curr_node.get_indexes().1;
            let delta_t = curr_node.get_time() - pre_node.get_time();

            let mut end = r;
            if r == n-1 {
                end = n;
                assert! (r == n-1);
            }
            
            // without_reconfig
            if self.robot.check_velocity(&curr_node.get_ik(), &pre_node.get_ik(),  delta_t) {
                let mut offsets: Vec<ndarray::prelude::ArrayBase<ndarray::OwnedRepr<f64>, ndarray::prelude::Dim<[usize; 1]>>> = vec![];
                for _ in 0..num_samples_aound_keypoints {
                    let offset = self.robot.get_random_offsets();
                    offsets.push(offset);
                }

                let mut linear_node_num: Vec<Vec<usize>> = vec![];

                for i in l..end {
                    // println!("i: {}", i);
                    linear_node_num.push(vec![]);

                    let start_config = lerp_config(self.table[l][l_idx].get_ik(), self.table[r][r_idx].get_ik(), (i-l) as f64 / (r-l) as f64);

                    if i != l && i != r {
                        self.robot.ik_solver.reset(start_config.to_vec());
                        let (found_ik, ik) = self.robot.try_to_reach(self.trajectory[i].1, self.trajectory[i].2);
                        if found_ik {
                            // println!("found an interpolated node at {} - {:?}", i, ik.clone());
                            let node = N::new(ik, self.trajectory[i].0, "interpolated".to_string(), (i, self.table[i].len()));
                            linear_node_num[i-l].push(self.table[i].len());
                            self.table[i].push(node);
                            self.num_linear_nodes[i] += 1;
                        }
                    }

                    for j in 0..offsets.len() {
                        let config = self.robot.clip_config(&(start_config.clone() + offsets[j].clone()));
                        self.robot.ik_solver.reset(config.to_vec());
                        let (found_ik, ik) = self.robot.try_to_reach(self.trajectory[i].1, self.trajectory[i].2);
                        if found_ik {
                            // println!("found an around node at {} - {:?}", i, ik.clone());
                            let node = N::new(ik.clone(), self.trajectory[i].0, "around".to_string(), (i, self.table[i].len()));
                            linear_node_num[i-l].push(self.table[i].len());
                            self.table[i].push(node);
                            self.num_linear_nodes[i] += 1;
                           
                        }
                    }
                }

            } else {
                println!("Sparse edge with reconfig between {} and {}", l, r);
                self.pre_sample_candidates((l..r).step_by(1).chain(std::iter::once(r)), 100, max_num_samples);
            }

            curr_node = pre_node;
        }

        let duration = start.elapsed();
        // println!("Time elapsed in linear sampling is: {:?}", duration);

        let mut counter: i64 = self.num_linear_nodes.iter().sum::<usize>() as i64 - self.num_random_nodes.iter().sum::<usize>() as i64;

        // println!("before: num random nodes: {:?}", self.num_random_nodes);
        let start = Instant::now();

        // let total_nodes = self.num_random_nodes.iter().sum::<usize>();
        let mut weights = self.num_random_nodes.iter().map(|x| (- (x.clone() as f64)).exp()).collect::<Vec<f64>>();

        while counter > 0 {
            counter -= 1;
            let idx = WeightedIndex::new(&weights).unwrap().sample(&mut self.rng);

            self.robot.reset_random();
            let (found_ik, ik) = self.robot.try_to_reach(self.trajectory[idx].1, self.trajectory[idx].2);
            if !found_ik {
                continue;
            }
            let node = N::new(ik, self.trajectory[idx].0, "random".to_string(), (idx, self.table[idx].len()));
            self.table[idx].push(node);
            self.num_random_nodes[idx] += 1;

            weights[idx] = (- (self.num_random_nodes[idx] as f64)).exp();
        }

        let duration = start.elapsed();
        // println!("Time elapsed in random sampling is: {:?}", duration);

    }

    pub fn pre_solve(&mut self, step: usize, num_samples: usize, debug: bool) -> Option<Motion> {
        
        self.temporal_step = step;
        self.sparse_node_per_waypoint = num_samples;

        let n = self.trajectory.len();
        
        for _ in 0..n {
            self.table.push(vec![]);
            self.connectability.push(vec![]);
            self.num_hard_sparse.push(0);
            self.num_linear_nodes.push(0);
            self.num_random_nodes.push(0);
        }

        self.pre_sample_candidates((0..n).step_by(step).chain(std::iter::once(n-1)), self.sparse_node_per_waypoint, 300);

        if debug {
            let best_node: N = self.pre_dp(self.temporal_step, false);
            return Some(self.motion_from_best_node(&best_node));
        }
        None
    }

    pub fn solve(&mut self, iteration: usize, max_num_samples: usize) -> Motion{
        
        let sparse_traj_last_node = self.pre_dp(self.temporal_step, false);

        self.sample_candidates(sparse_traj_last_node, 5, iteration, max_num_samples);

        self.dp()
    }
   
}