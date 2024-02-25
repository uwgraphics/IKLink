use::nalgebra::{Vector3, UnitQuaternion};
use rand::Rng;
use crate::utils_rust::file_utils::{*};
use crate::relaxed_ik::RelaxedIK;
use ndarray::Array1;

pub struct Robot {
    pub robot_name: String,
    pub ik_solver: RelaxedIK,
    pub rng: rand::prelude::ThreadRng,
    pub arm_num_dofs: usize,
}

impl Robot {

    pub fn new(robot_name: &str) -> Self {
        let path_to_src = get_path_to_src();
        let arm_path_to_setting = path_to_src.clone() +  "configs/example_settings/"+ robot_name + ".yaml";
        let ik_solver = RelaxedIK::load_settings(&arm_path_to_setting);
       
        let arm_num_dofs = ik_solver.vars.robot.num_dofs;
        assert!(ik_solver.vars.robot.chain_indices.len() == 1, "Robot should have only one chain");
        Robot {
            robot_name: robot_name.to_string(),
            ik_solver,
            rng: rand::thread_rng(),
            arm_num_dofs,
        }
    }
    pub fn get_random_arm_config(&mut self) -> Vec<f64> {
        let mut config = vec![];
        for i in 0..self.arm_num_dofs {
            if self.ik_solver.vars.robot.joint_types[i] == "continuous" {
                config.push(self.rng.gen_range(-std::f64::consts::PI..std::f64::consts::PI));
            } else {
                config.push(self.rng.gen_range(self.ik_solver.vars.robot.lower_joint_limits[i]..=self.ik_solver.vars.robot.upper_joint_limits[i]));
            }
        }
        config
    }

    pub fn fk(&self, config: &Array1<f64>) -> (Vector3<f64>, UnitQuaternion<f64>) {
        self.ik_solver.vars.robot.get_ee_pos_and_quat_immutable(&config.to_vec())[0]
    }

    pub fn check_pose(&self, config: &Array1<f64>, pos: Vector3<f64>, quat: UnitQuaternion<f64>) -> bool {
        let (ee_pos, ee_quat) = self.fk(config);
        let pos_diff = (ee_pos - pos).norm();
        let quat_diff = (ee_quat * quat.inverse()).angle();
        pos_diff < 0.001 && quat_diff < 0.01
    }

    pub fn reset_random(&mut self) {
        let config = self.get_random_arm_config();
        self.ik_solver.reset(config);
    }

    pub fn try_to_reach(&mut self, pos: Vector3<f64>, quat: UnitQuaternion<f64>) -> (bool, Array1<f64>) {
        // pos and quat are wrt the base frame
        self.reset_random();
        self.ik_solver.vars.goal_positions = vec![pos];
        self.ik_solver.vars.goal_quats = vec![quat];
        let config = self.ik_solver.solve(false);    
        // println!("config: {:?}", config);
        if self.check_pose(&config, pos, quat) {
            return (true, config);
        } else {
            return (false, Array1::from(vec![]));
        }
    }

    pub fn try_to_track(&mut self, pos: Vector3<f64>, quat: UnitQuaternion<f64>) -> (bool, Array1<f64>) {
        // pos and quat are wrt the base frame
        self.ik_solver.vars.goal_positions = vec![pos];
        self.ik_solver.vars.goal_quats = vec![quat];
        let config = self.ik_solver.solve(true);    
        if self.check_pose(&config, pos, quat) {
            return (true, config);
        } else {
            return (false, Array1::from(vec![]));
        }
    }

    pub fn check_velocity(&self, config: &Array1<f64>, prev_config: &Array1<f64>, delta_t: f64) -> bool {
        assert!(config.len() == prev_config.len(), "config and prev_config should have the same length");
        assert!(config.len() == self.arm_num_dofs, "config and prev_config should have the same length as arm_num_dofs");
        
        // TODO: parse joint velocity limits from URDF
        for i in 0..config.len() {
            if (config[i] - prev_config[i]).abs() > self.ik_solver.vars.robot.joint_velocity_limits[i] * delta_t {
                return false;
            }
        }
        true
    }

    pub fn joint_movement(&self, config1: &Array1<f64>, config2: &Array1<f64>) -> f64 {
        let mut out = 0.0;
        for i in 0..config1.len() {
            out += (config1[i] - config2[i]).powi(2);
        }
        out.sqrt()
    }

}