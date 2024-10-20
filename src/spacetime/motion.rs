
use ndarray::Array1;
use std::fmt;

pub struct Motion {
    pub robot_name: String,
    pub joint_names: Vec<String>,
    pub data: Vec<(f64, Array1<f64>)>,
}

impl Motion {
    fn new(robot_name: &str) -> Self {
        Motion {
            robot_name: robot_name.to_string(),
            joint_names: vec![],
            data: vec![],
        }
    }
}

impl fmt::Debug for Motion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Motion {{ robot_name: {}, joint_names: {:?}, data: {:?} }}", self.robot_name, self.joint_names, self.data)
    }
}