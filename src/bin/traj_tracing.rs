
use glob::glob;
use csv;
use nalgebra::{Vector3, UnitQuaternion, Quaternion};
use relaxed_ik_lib::utils_rust::file_utils::{*};
use relaxed_ik_lib::iklink::IKLink;
use relaxed_ik_lib::spacetime::motion::Motion;

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

fn save_motion(filename: &str,  motion: Motion) {
    // save motion to a csv file
    let mut wtr = csv::Writer::from_path(filename).unwrap();

    // write header
    let robot_name = motion.robot_name;
    let mut row = vec!["time".to_string()];
    for i in 0..motion.joint_names.len() {
        row.push(robot_name.clone().to_string() + "-" + &motion.joint_names[i] );
    }
    wtr.write_record(&row).unwrap();

    // write data
    for (time, config) in motion.data.iter() {
        let mut row = vec![time.to_string()];

        for i in config.iter() {
            row.push(i.to_string());
        }
        wtr.write_record(&row).unwrap();
    }
}

fn main() {
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

                let mut ik_link = IKLink::new(robot_name, traj);
                let motion = ik_link.solve();

                let output_dir = path_to_src.clone() + "output_motions/" + file_name;
                save_motion(&output_dir, motion);
                println!("Saved motion to: {}", output_dir);
            }
            Err(e) => println!("{:?}", e),
        }
    }



}