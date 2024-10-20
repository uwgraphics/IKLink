use ndarray::Array1;
use crate::{spacetime::robot::Robot};
use std::clone::Clone;
use crate::nodes::NodeAnytime::NodeAnytime;

pub struct IKLinkAnytimeNode {
    pub time: f64,
    pub ik: Array1<f64>,
    pub has_predecessor: bool,
    pub predecessor: (usize, usize),
    pub indexes: (usize, usize),

    pub num_reconfig: usize,
    pub num_sparse_edge: usize,  // number of sparse edge
    pub jnt_movement: f64,

    pub creation: String,
    pub is_reachable_from: Vec<bool>,
    pub distance_from: Vec<f64>
}

impl IKLinkAnytimeNode{

    fn compare_scores(&self, num_reconfig: usize, num_sparse_edge: usize, jnt_movement: f64) -> bool {
        if num_reconfig < self.num_reconfig || (num_reconfig == self.num_reconfig && jnt_movement < self.jnt_movement) {
            return true;
        }
        false
    }

    fn compare_and_update_scores(&mut self, num_reconfig: usize, num_sparse_edge: usize, jnt_movement: f64) -> bool {
        if  self.compare_scores(num_reconfig, num_sparse_edge, jnt_movement) {
            self.num_reconfig = num_reconfig;
            self.num_sparse_edge = num_sparse_edge;
            self.jnt_movement = jnt_movement;
            return true;
        }
        false
    }
}

impl NodeAnytime for IKLinkAnytimeNode{

    fn get_indexes(&self) -> (usize, usize) {
        self.indexes
    }

    fn get_creation(&self) -> &String {
        &self.creation
    }

    fn get_time(&self) -> f64 {
        self.time
    }

    fn has_predecessor(&self) -> bool {
        self.has_predecessor
    }

    fn get_predecessor(&self) -> (usize, usize) {
        assert!(self.has_predecessor, "Node has no predecessor");
        self.predecessor
    }

    fn get_ik(&self) -> &Array1<f64> {
        &self.ik
    }

    fn get_performance(&self) -> String {
        format!("num_reconfig: {}, num_sparse_edge: {}, jnt_movement: {}", self.num_reconfig, self.num_sparse_edge, self.jnt_movement)
    }

    fn reset_scores_for_first_col(&mut self) {
        self.num_reconfig = 0;
        self.num_sparse_edge = 0;
        self.jnt_movement = 0.0;
        self.has_predecessor = false;
        self.predecessor = (0, 0);
    }

    fn reset_scores_for_other_col(&mut self) {
        self.num_reconfig = 1000000;
        self.num_sparse_edge = 100000;
        self.jnt_movement = 1000000.0;
        self.has_predecessor = false;
        self.predecessor = (0, 0);
    }

    fn clone(&self) -> Self {
        IKLinkAnytimeNode {
            time: self.time,
            ik: self.ik.clone(),
            has_predecessor: self.has_predecessor,
            indexes: self.indexes,
            predecessor: self.predecessor,
            num_reconfig: self.num_reconfig,
            num_sparse_edge: self.num_sparse_edge,
            jnt_movement: self.jnt_movement,
            creation: self.creation.clone(),
            is_reachable_from: self.is_reachable_from.clone(),
            distance_from: self.distance_from.clone()
        }
    }

    fn copy_from(&mut self, other: &Self) {
        self.time = other.time;
        self.ik = other.ik.clone();
        self.has_predecessor = other.has_predecessor;
        self.indexes = other.indexes;
        self.predecessor = other.predecessor;
        self.num_reconfig = other.num_reconfig;
        self.num_sparse_edge = other.num_sparse_edge;
        self.jnt_movement = other.jnt_movement;
        self.creation = other.creation.clone();
        self.is_reachable_from = other.is_reachable_from.clone();
        self.distance_from = other.distance_from.clone();
    }

    fn new(ik: Array1<f64>, time:f64, creation: String, indexes: (usize, usize)) -> Self {
        
        let predecessor = (0, 0);

        IKLinkAnytimeNode {
            time,
            ik,
            num_reconfig: 1000000,
            num_sparse_edge: 100000,
            jnt_movement: 1000000.0,
            has_predecessor: false,
            indexes,
            predecessor,
            creation: creation,
            distance_from: vec![],
            is_reachable_from: vec![]
        }
    }

    fn try_to_connect(&mut self,  other: &IKLinkAnytimeNode, other_idx: (usize, usize), robot: &Robot) {

        let delta_t = self.time - other.time;
        assert!(delta_t > 0.0, "self.time: {}, other.time: {}", self.time, other.time);

        // find best predecessor with an arm reconfiguration
        let num_sparse_edge = if other_idx.0 + 1 < self.indexes.0 { other.num_sparse_edge + 1 } else { other.num_sparse_edge };
        if self.compare_and_update_scores(other.num_reconfig+1, num_sparse_edge, other.jnt_movement) {
            self.has_predecessor = true;
            self.predecessor = other_idx;
        }

        if robot.check_velocity(&self.ik, &other.ik, delta_t) {
            // find best predecessor with no arm reconfiguration
            let jnt_movement = other.jnt_movement + robot.joint_movement(&self.ik, &other.ik);
            if self.compare_and_update_scores(other.num_reconfig, num_sparse_edge, jnt_movement) {
                self.has_predecessor = true;
                self.predecessor = other_idx;
            }
        } 
    }

    fn check_reachability(&mut self, other: &IKLinkAnytimeNode, other_idx: (usize, usize), robot: &Robot) -> bool {
        assert!(other_idx.0 + 1 == self.indexes.0, "other_idx.0: {}, self.indexes.0: {}", other_idx.0, self.indexes.0);

        if other_idx.1 < self.is_reachable_from.len() {
            return self.is_reachable_from[other_idx.1];
        }

        assert!(other_idx.1 == self.is_reachable_from.len(), "other_idx.1: {}, self.reachable_from.len(): {}", other_idx.1, self.is_reachable_from.len());

        let delta_t = self.time - other.time;
        assert!(delta_t > 0.0, "self.time: {}, other.time: {}", self.time, other.time);

        if robot.check_velocity(&self.ik, &other.ik, delta_t) {
            self.is_reachable_from.push(true);
            self.distance_from.push(robot.joint_movement(&self.ik, &other.ik));
        } else {
            self.is_reachable_from.push(false);
            self.distance_from.push(1000000.0);
        }
        self.is_reachable_from[other_idx.1]
    }

    fn get_is_reachable_from(&self) -> &Vec<bool> {
        &self.is_reachable_from
    }

    fn get_distance_from(&self) -> &Vec<f64> {
        &self.distance_from
    }

    fn compare_and_update_node(&mut self, other: &IKLinkAnytimeNode) -> bool {
        if self.compare_scores(other.num_reconfig, other.num_sparse_edge, other.jnt_movement) {
            self.copy_from(other);
            return true;
        }
        false
    }
}

