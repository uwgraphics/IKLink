use ndarray::Array1;
use crate::{spacetime::robot::Robot};

pub trait Node {    
    fn get_predecessor(&self) -> (usize, usize) ;
    fn get_time(&self) -> f64;
    fn get_ik(&self) -> &Array1<f64>;
    fn get_performance(&self) -> String;
    fn has_predecessor(&self) -> bool;
    fn reset_for_first_col(&mut self);
    fn reset_for_other_col(&mut self);
    fn clone(&self) -> Self;
    fn copy_from(&mut self, other: &Self);
    fn new(ik: Array1<f64>, time:f64, creation: String) -> Self;
    fn try_to_connect(&mut self, other: &Self, other_idx: (usize, usize), robot: &Robot);
    fn compare_and_update_node(&mut self, other: &Self) -> bool;
}
