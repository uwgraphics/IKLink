use ndarray::Array1;
use crate::{ spacetime::robot::Robot};
use std::clone::Clone;
use crate::nodes::Node::Node;

pub struct IKLinkNode {
    pub time: f64,
    pub ik: Array1<f64>,
    pub has_predecessor: bool,
    pub predecessor: (usize, usize) ,
    pub num_reconfig: usize,
    pub jnt_movement: f64,
}

impl IKLinkNode {

    fn compare_scores(&self, num_reconfig: usize, jnt_movement: f64) -> bool {
        if num_reconfig < self.num_reconfig || (num_reconfig == self.num_reconfig && jnt_movement < self.jnt_movement) {
            return true;
        }
        false
    }
    fn compare_and_update_scores(&mut self, num_reconfig: usize, jnt_movement: f64) -> bool {
        if self.compare_scores(num_reconfig, jnt_movement) {
            self.num_reconfig = num_reconfig;
            self.jnt_movement = jnt_movement;
            return true;
        }
        false
    }
}

impl Node for IKLinkNode{

    fn get_performance(&self) -> String {
        format!("num_reconfig: {}, jnt_movement: {}", self.num_reconfig, self.jnt_movement)
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

    fn reset_for_first_col(&mut self) {
        self.num_reconfig = 0;
        self.jnt_movement = 0.0;
        self.has_predecessor = false;
        self.predecessor = (0,0);
    }

    fn reset_for_other_col(&mut self) {
        self.num_reconfig = 1000000;
        self.jnt_movement = 1000000.0;
        self.has_predecessor = false;
        self.predecessor = (0,0);
    }

    fn clone(&self) -> Self {
        IKLinkNode {
            time: self.time,
            ik: self.ik.clone(),
            has_predecessor: self.has_predecessor,
            predecessor: self.predecessor,
            num_reconfig: self.num_reconfig,
            jnt_movement: self.jnt_movement
        }
    }

    fn copy_from(&mut self, other: &Self) {
        self.time = other.time;
        self.ik = other.ik.clone();
        self.has_predecessor = other.has_predecessor;
        self.predecessor = other.predecessor;
        self.num_reconfig = other.num_reconfig;
        self.jnt_movement = other.jnt_movement;
    }

    fn new(ik: Array1<f64>, time:f64, creation: String) -> Self {
        
        IKLinkNode {
            time,
            ik,
            num_reconfig: 1000000,
            jnt_movement: 1000000.0,
            has_predecessor: false,
            predecessor: (0,0),
        }
    }

    fn try_to_connect(&mut self, other: &IKLinkNode, other_idx: (usize, usize), robot: &Robot) {

        let delta_t = self.time - other.time;
        assert!(delta_t > 0.0, "self.time: {}, other.time: {}", self.time, other.time);

        // find best predecessor with an arm reconfiguration
        if self.compare_and_update_scores(other.num_reconfig + 1, other.jnt_movement) {
            self.has_predecessor = true;
            self.predecessor = other_idx;
        }

        if robot.check_velocity(&self.ik, &other.ik, delta_t) {
            // find best predecessor with no arm reconfiguration
            let jnt_movement = other.jnt_movement + robot.joint_movement(&self.ik, &other.ik);
            if self.compare_and_update_scores(other.num_reconfig, jnt_movement) {
                self.has_predecessor = true;
                self.predecessor = other_idx;
            }
        } 
    }

    fn compare_and_update_node(&mut self, other: &IKLinkNode) -> bool {
        if self.compare_scores(other.num_reconfig, other.jnt_movement) {
            self.copy_from(other);
            return true;
        }
        false
    }
}
