
use glob::glob;
use csv;
use nalgebra::{Vector3, UnitQuaternion, Quaternion};
use relaxed_ik_lib::utils_rust::file_utils::{*};
use relaxed_ik_lib::iklink::IKLink;
use relaxed_ik_lib::iklink_anytime::IKLinkAnytime;

use relaxed_ik_lib::spacetime::motion::Motion;
use relaxed_ik_lib::spacetime::robot::Robot;
use std::time::Instant;
use relaxed_ik_lib::nodes::IKLinkNode::IKLinkNode;
use relaxed_ik_lib::nodes::IKLinkAnytimeNode::IKLinkAnytimeNode;
use nalgebra::Vector6;


fn load_traj(path: &str) -> Vec<(f64, Vector3<f64>, UnitQuaternion<f64>)> {
    // load trajectory from a csv file
    let mut traj = vec![];
    let mut rdr = csv::Reader::from_path(path).unwrap();
    for result in rdr.records() {
        let record = result.unwrap();
        let time = record[0].parse::<f64>().unwrap();
        let pos = Vector3::new(record[1].parse::<f64>().unwrap(), record[2].parse::<f64>().unwrap(), record[3].parse::<f64>().unwrap());
        let q = Quaternion::new(record[7].parse::<f64>().unwrap(), record[4].parse::<f64>().unwrap(), record[5].parse::<f64>().unwrap(), record[6].parse::<f64>().unwrap());
        let quat = UnitQuaternion::from_quaternion(q);
        traj.push((time, pos, quat));
    }
    traj
}

fn save_motion(filename: &str,  motion: Motion, traj: &Vec<(f64, Vector3<f64>, UnitQuaternion<f64>)>, save_target: bool) {
    // save motion to a csv file
    let mut wtr = csv::Writer::from_path(filename).unwrap();

    // write header
    let robot_name = motion.robot_name;
    let mut row = vec!["time".to_string()];
    for i in 0..motion.joint_names.len() {
        row.push(robot_name.clone().to_string() + "-" + &motion.joint_names[i] );
    }

    if save_target {
        row.push("target-POS_X".to_string());
        row.push("target-POS_Y".to_string());
        row.push("target-POS_Z".to_string());
        row.push("target-ROT_X".to_string());
        row.push("target-ROT_Y".to_string());
        row.push("target-ROT_Z".to_string());
        row.push("target-ROT_W".to_string());
    }
    wtr.write_record(&row).unwrap();

    // write data
    let mut j = 0;
    for (time, config) in motion.data.iter() {


        let mut row = vec![time.to_string()];

        for i in config.iter() {
            row.push(i.to_string());
        }

        if save_target {
            assert! (traj[j].0 == *time, "traj[j].0: {}, time: {}", traj[j].0, time);
            row.push(traj[j].1.x.to_string());
            row.push(traj[j].1.y.to_string());
            row.push(traj[j].1.z.to_string());
            row.push(traj[j].2.quaternion().i.to_string());
            row.push(traj[j].2.quaternion().j.to_string());
            row.push(traj[j].2.quaternion().k.to_string());
            row.push(traj[j].2.quaternion().w.to_string());
        }
        j += 1;

        wtr.write_record(&row).unwrap();
    }
}

fn count_num_reconfig(robot: &Robot, motion: &Motion) -> usize {
    // count the number of reconfigurations in the motion
    let mut count = 0;
    for i in 0..motion.data.len() - 1 {
        let config1 = &motion.data[i].1;
        let config2 = &motion.data[i + 1].1;
        let delta_t = motion.data[i + 1].0 - motion.data[i].0;
        if !robot.check_velocity(&config1, &config2, delta_t) {
            count += 1;
            println!("Reconfig between {} and {}", i, i + 1);
        }
    }
    count
}


fn tracking(debug: bool, repeat: usize, time_limit: u64, tolerance: Vector6<f64>) {
    let path_to_src = get_path_to_src();
    let dir = path_to_src.clone() + "input_trajectories/*.csv";

    // get all csv files in the directory
    for entry in glob(&dir).expect("Failed to read glob pattern") {
        match entry {
            Ok(path) => {
                let path_str = path.to_str().unwrap();
                let traj = load_traj(path_str);

                let file_name = path_str.split('/').last().unwrap();
                let robot_name = file_name.split('_').next().unwrap();
                let robot = Robot::new(robot_name);

                for j in 0..repeat {
                        let mut tmp = vec![];
                        let total_start = Instant::now();
                        let mut iklink_anytime = IKLinkAnytime::<IKLinkAnytimeNode>::new(robot_name, &traj);
                        iklink_anytime.robot.set_tolerances(tolerance);
                        let res = iklink_anytime.pre_solve(5, 50, debug);
                        let mut pre_duration = total_start.elapsed();
                        if debug {
                            // println!("Pre-solve result: {:?}", res.unwrap());
                            let output_dir = path_to_src.clone() + "output_motions/" + file_name.split('.').next().unwrap() + j.to_string().as_str() + "+iklink_anytime_pre_computation" + "+" + pre_duration.as_millis().to_string().as_str() + ".csv";
                            save_motion(&output_dir, res.unwrap(), &traj, false);
                            println!("Saved motion to: {}", output_dir)
                        }

                        let mut i: usize = 0;
                        while total_start.elapsed().as_secs() < time_limit {
                            let start = Instant::now();
                            let motion = iklink_anytime.solve(i+1, 300);
                            let duration = start.elapsed();

                            tmp.push((count_num_reconfig(&robot, &motion), duration + pre_duration));

                            pre_duration = pre_duration + duration;

                            let output_dir = path_to_src.clone() + "output_motions/" + file_name.split('.').next().unwrap() + j.to_string().as_str() + "+iklink_anytime" + i.to_string().as_str() + "+" + pre_duration.as_millis().to_string().as_str() + ".csv";
                            save_motion(&output_dir, motion, &traj, true);
                            println!("Saved motion to: {}", output_dir);
                            i += 1;
                        }
                }
            }
            Err(e) => println!("{:?}", e),
        }
    }
}

fn main() {
    tracking(false, 1, 10, Vector6::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
}