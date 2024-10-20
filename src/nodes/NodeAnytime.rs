use ndarray::Array1;
use crate::spacetime::robot::Robot;


pub trait NodeAnytime {    
    fn get_indexes(&self) -> (usize, usize);
    fn get_predecessor(&self) -> (usize, usize);
    fn get_time(&self) -> f64;
    fn get_ik(&self) -> &Array1<f64>;
    fn get_creation(&self) -> &String;
    fn get_distance_from(&self) -> &Vec<f64>;
    fn get_is_reachable_from(&self) -> &Vec<bool>;

    fn get_performance(&self) -> String;

    fn has_predecessor(&self) -> bool;
    fn reset_scores_for_first_col(&mut self);
    fn reset_scores_for_other_col(&mut self);
    fn clone(&self) -> Self;
    fn copy_from(&mut self, other: &Self);
    fn new(ik: Array1<f64>, time:f64, creation: String, indexes: (usize, usize)) -> Self;

    fn try_to_connect(&mut self, other: &Self, other_idx: (usize, usize), robot: &Robot);
    fn check_reachability(&mut self, other: &Self, other_idx: (usize, usize), robot: &Robot) -> bool;
    fn compare_and_update_node(&mut self, other: &Self) -> bool;
}
