use crate::groove::vars::RelaxedIKVars;
use crate::groove::groove::OptimizationEngineOpen;
use crate::groove::objective_master::ObjectiveMaster;
use crate::utils_rust::file_utils::{*};
use crate::utils_rust::transformations::{*};
use std::os::raw::{c_double, c_int};
use ndarray::Array1;


#[repr(C)]
pub struct Opt {
    pub data: *const c_double,
    pub length: c_int,
}

pub struct RelaxedIK {
    pub vars: RelaxedIKVars,
    pub om_relaxedik: ObjectiveMaster,
    pub om_standardik: ObjectiveMaster,
    pub groove: OptimizationEngineOpen
}

impl RelaxedIK {
    pub fn load_settings( path_to_setting: &str) -> Self {
        println!("RelaxedIK is using below setting file {}", path_to_setting);

        let vars = RelaxedIKVars::from_local_settings(path_to_setting);        
        let om_relaxedik: ObjectiveMaster = ObjectiveMaster::relaxed_ik(&vars.robot.chain_indices);
        let om_standardik: ObjectiveMaster = ObjectiveMaster::standard_ik(&vars.robot.chain_indices);

        let groove = OptimizationEngineOpen::new(vars.robot.num_dofs.clone());

        Self{vars, om_relaxedik, om_standardik, groove}
    }

    pub fn reset(&mut self, x: Vec<f64>) {
        self.vars.reset( x.clone());
        self.om_relaxedik = ObjectiveMaster::relaxed_ik(&self.vars.robot.chain_indices);
        self.om_standardik = ObjectiveMaster::standard_ik(&self.vars.robot.chain_indices);
        self.groove = OptimizationEngineOpen::new(self.vars.robot.num_dofs.clone());
    }

    pub fn solve(&mut self, constrain_velocity: bool) -> Array1<f64> {
        let mut out_x = self.vars.xopt.clone();

        if constrain_velocity {
            self.groove.optimize(&mut out_x, &self.vars, &self.om_relaxedik, 100);
        } else {
            self.groove.optimize(&mut out_x, &self.vars, &self.om_standardik, 1000);
        }
        // let frames = self.vars.robot.get_frames_immutable(&out_x);

        for i in 0..out_x.len() {
            if out_x[i].is_nan() {
                // println!("No valid solution found! Returning previous solution: {:?}. End effector position goals: {:?}", self.vars.xopt, self.vars.goal_positions);
                return Array1::from(self.vars.xopt.clone());
            }
        }
        self.vars.update(out_x.clone());  
        Array1::from(out_x)
    }
}
