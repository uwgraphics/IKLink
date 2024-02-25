use crate::spacetime::arm_kinematics;
use nalgebra;
use urdf_rs;

#[derive(Clone, Debug)]
pub struct RobotKinematics {
    pub arms: Vec<arm_kinematics::ArmKineamtics>,
    pub num_chains: usize,
    pub num_dofs: usize,
    pub chain_indices: Vec<Vec<usize>>,
    pub lower_joint_limits: Vec<f64>,
    pub upper_joint_limits: Vec<f64>,
    pub joint_velocity_limits: Vec<f64>,
    pub joint_names: Vec<String>,
    pub joint_types: Vec<String>,
}

impl RobotKinematics {
    pub fn from_urdf(urdf: &str, base_links: &[String], ee_links: &[String], joint_ordering: Option<Vec<String>>) -> Self {
        let description : urdf_rs::Robot = urdf_rs::read_from_string(urdf).unwrap();
        let chain: k::Chain<f64> = k::Chain::from(description.clone());

        let mut arms: Vec<arm_kinematics::ArmKineamtics> = Vec::new();
        let num_chains = base_links.len();        
        let mut chain_indices = Vec::new();        
        let mut num_dofs = 0;
        
        let mut lower_joint_limits = Vec::new();
        let mut upper_joint_limits: Vec<f64> = Vec::new();
        let mut joint_velocity_limits: Vec<f64> = Vec::new();
        let mut joint_names = Vec::new();
        let mut joint_types = Vec::new();

        for i in 0..num_chains {
            let base_link = chain.find_link(base_links[i].as_str()).unwrap();
            let ee_link = chain.find_link(ee_links[i].as_str()).unwrap();
            let serial_chain = k::SerialChain::from_end_to_root(&ee_link, &base_link);

            let mut joint_indices = Vec::new();
            let mut articulated_joint_index = 0;

            let mut first_link: bool = true;
            serial_chain.iter().for_each(|node| {
                let joint = node.joint();
                if first_link {
                    first_link = false;
                    return
                } else {
                    let fixed_joint: bool;
                    match joint.joint_type {
                        k::JointType::Fixed => {
                            fixed_joint = true;
                        },
                        k::JointType::Rotational { axis } => {
                            if joint.limits.is_none() {
                                lower_joint_limits.push(-999.0);
                                upper_joint_limits.push(999.0);
                                joint_types.push("continuous".to_string());
                            } else {
                                lower_joint_limits.push(joint.limits.unwrap().min);
                                upper_joint_limits.push(joint.limits.unwrap().max);
                                joint_types.push("revolute".to_string());
                            }
                            fixed_joint = false;
                        },
                        k::JointType::Linear { axis } => {
                            lower_joint_limits.push(joint.limits.unwrap().min);
                            upper_joint_limits.push(joint.limits.unwrap().max);
                            joint_types.push("prismatic".to_string());
                            fixed_joint = false;
                        }
                    }
                    if !fixed_joint {
                        num_dofs += 1;
                        joint_names.push(joint.name.clone());
                        if let Some(ordering) = &joint_ordering {
                            if let Some(joint_index) = ordering.iter().position(|s| *s == joint.name) {
                                joint_indices.push(joint_index);
                            } else {
                                println!("Warning: joint {} not found in joint_ordering provided!", joint.name)
                            }
                        } else {
                            joint_indices.push(articulated_joint_index);
                            articulated_joint_index += 1;
                        }
                    }
                }

            });
            let arm: arm_kinematics::ArmKineamtics = arm_kinematics::ArmKineamtics::from_a_series_chain(&serial_chain, false);
            arms.push(arm.clone());
            chain_indices.push(joint_indices);
        }

        // parse joint velocity limit
        for i in 0..joint_names.len() {
            let joint_name = joint_names[i].clone();
            for j in 0..description.joints.len() {
                if description.joints[j].name == joint_name {
                    joint_velocity_limits.push(description.joints[j].limit.velocity);
                    break;
                }
            }
        }

        println!("Lower_joint_limits: {:?}", lower_joint_limits);
        println!("Upper_joint_limits: {:?}", upper_joint_limits);
        println!("Joint_velocity_limits: {:?}", joint_velocity_limits);

        // Update the number of dofs if joint ordering is provided
        if let Some(ordering) = joint_ordering {
            num_dofs = ordering.len();
            let mut lower_joint_limits_new = Vec::new();
            let mut upper_joint_limits_new = Vec::new();
            ordering.iter().for_each(|name| {
                if let Some(joint_index) = joint_names.iter().position(|s| *s == *name) {
                    lower_joint_limits_new.push(lower_joint_limits[joint_index]);
                    upper_joint_limits_new.push(upper_joint_limits[joint_index]);
                }
            });
            lower_joint_limits = lower_joint_limits_new;
            upper_joint_limits = upper_joint_limits_new;   
        }

        assert!(num_dofs == lower_joint_limits.len());
        assert!(num_dofs == upper_joint_limits.len());
        assert!(num_dofs == joint_names.len());
        assert!(num_dofs == joint_types.len());
        assert!(num_dofs == joint_velocity_limits.len());
        assert!(num_chains == arms.len());
        assert!(num_chains == chain_indices.len());

        println!("Robot created successfully! Number of chains: {}, Number of dofs: {}", num_chains, num_dofs);

        RobotKinematics{arms, num_chains, chain_indices, num_dofs, lower_joint_limits, upper_joint_limits, joint_velocity_limits, joint_names, joint_types}

    }

    pub fn get_frames_immutable(&self, x: &[f64]) -> Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> {
        let mut out: Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> = Vec::new();
        for i in 0..self.num_chains {            
            let chain_values: Vec<f64> = self.chain_indices[i].iter().map(|&i| x[i]).collect();
            out.push( self.arms[i].get_frames_immutable(chain_values.as_slice()) );
        }
        out
    }
    
    pub fn get_manipulability_immutable(&self, x: &[f64]) -> f64 {
        let mut out = 0.0;
        for i in 0..self.num_chains {            
            let chain_values: Vec<f64> = self.chain_indices[i].iter().map(|&i| x[i]).collect();
            out += self.arms[i].get_manipulability_immutable( chain_values.as_slice() );
        }
        out
    }

    pub fn get_ee_pos_and_quat_immutable(&self, x: &[f64]) -> Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> {
        let mut out: Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> = Vec::new();
        for i in 0..self.num_chains {            
            let chain_values: Vec<f64> = self.chain_indices[i].iter().map(|&i| x[i]).collect();
            out.push( self.arms[i].get_ee_pos_and_quat_immutable( chain_values.as_slice() ));
        }
        out
    }
}

