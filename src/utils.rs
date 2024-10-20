use ndarray::{Array1, Array2};

pub fn vec_of_arrays_to_2d_array( vec: &mut Vec<Array1<f64>>) -> Array2<f64> {
    if vec.is_empty() {
        return Array2::zeros((0, 0)); // Return an empty 2D array if the input vector is empty
    }

    let nrows = vec.len(); // Number of rows in the 2D array
    let ncols = vec[0].len(); // Number of columns in the 2D array, assuming all 1D arrays are the same size

    // Initialize a 2D array with zeros
    let mut array2d = Array2::<f64>::zeros((nrows, ncols));

    for (i, array) in vec.into_iter().enumerate() {
        // Make sure each 1D array is the correct size; this example does not handle errors
        assert_eq!(array.len(), ncols, "All arrays must have the same size");

        // Copy the elements from the 1D array into the corresponding row of the 2D array
        array2d.row_mut(i).assign(&array);
    }

    array2d
}