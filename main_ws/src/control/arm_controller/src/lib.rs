//! arm_controller library helpers for testing
use k::nalgebra as na;

/// Compute a manipulability measure from a Jacobian matrix using SVD.
/// Returns the product of singular values (>= 0).
pub fn manipulability_from_jacobian(jacobian: &na::DMatrix<f64>) -> f64 {
    let svd = jacobian.svd(true, true);
    svd.singular_values.iter().fold(1.0_f64, |acc, &x| acc * x.max(0.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use k::nalgebra as na;

    #[test]
    fn manipulability_nonnegative() {
        let j = na::DMatrix::from_row_slice(3, 3, &[1.0, 0.0, 0.0,
                                                    0.0, 2.0, 0.0,
                                                    0.0, 0.0, 3.0]);
        let m = manipulability_from_jacobian(&j);
        assert!(m >= 0.0);
        // product should be 6.0
        assert!((m - 6.0).abs() < 1e-12);
    }

    #[test]
    fn manipulability_zero_for_rank_deficient() {
        let j = na::DMatrix::from_row_slice(3, 3, &[1.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0]);
        let m = manipulability_from_jacobian(&j);
        assert!(m >= 0.0);
        // product should be zero because two singular values are zero
        assert!(m.abs() < 1e-12);
    }
}
