//! arm_controller library helpers for testing
use k::nalgebra as na;

/// Compute a manipulability measure from a Jacobian matrix using SVD.
/// Returns the product of singular values (>= 0).
pub fn manipulability_from_jacobian(jacobian: &na::DMatrix<f64>) -> f64 {
    let svd = jacobian.clone().svd(true, true);
    svd.singular_values
        .iter()
        .fold(1.0_f64, |acc, &x| acc * x.max(0.0))
}

/// Solve damped least squares: dq = J^T * (J J^T + lambda^2 I)^-1 * v
pub fn dls_solve(
    jacobian: &na::DMatrix<f64>,
    twist: &na::DVector<f64>,
    lambda: f64,
) -> na::DVector<f64> {
    let jt = jacobian.transpose();
    let jjt = jacobian * &jt;
    let n = jjt.nrows();
    let damping = na::DMatrix::<f64>::identity(n, n) * (lambda * lambda);
    let jjt_damped = jjt + damping;
    // solve (J J^T + lambda^2 I) y = twist  => y = (..)^{-1} * twist
    if let Some(y) = jjt_damped.clone().lu().solve(twist) {
        jt * y
    } else {
        na::DVector::zeros(jacobian.ncols())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use k::nalgebra as na;

    #[test]
    fn manipulability_nonnegative() {
        let j = na::DMatrix::from_row_slice(3, 3, &[1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0]);
        let m = manipulability_from_jacobian(&j);
        assert!(m >= 0.0);
        // product should be 6.0
        assert!((m - 6.0).abs() < 1e-12);
    }

    #[test]
    fn manipulability_zero_for_rank_deficient() {
        let j = na::DMatrix::from_row_slice(3, 3, &[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let m = manipulability_from_jacobian(&j);
        assert!(m >= 0.0);
        // product should be zero because two singular values are zero
        assert!(m.abs() < 1e-12);
    }

    #[test]
    fn dls_identity_lambda_zero() {
        // J = I, lambda = 0 -> dq = twist
        let j = na::DMatrix::identity(3, 3);
        let v = na::DVector::from_column_slice(&[0.5, -0.2, 1.0]);
        let dq = dls_solve(&j, &v, 0.0);
        assert!((dq - v).norm() < 1e-12);
    }

    #[test]
    fn dls_identity_with_lambda() {
        // J = I, lambda > 0 -> dq = v / (1 + lambda^2)
        let j = na::DMatrix::identity(3, 3);
        let v = na::DVector::from_column_slice(&[1.0, 2.0, 3.0]);
        let lambda = 0.5;
        let dq = dls_solve(&j, &v, lambda);
        let expected = v.scale(1.0 / (1.0 + lambda * lambda));
        assert!((dq - expected).norm() < 1e-12);
    }

    #[test]
    fn dls_overdetermined_example() {
        // J is 2x3 (2 rows, 3 cols), rank 2. Solve for dq that produces given end-effector twist
        let j = na::DMatrix::from_row_slice(2, 3, &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0]);
        let v = na::DVector::from_column_slice(&[0.5, 0.25]);
        let lambda = 1e-6;
        let dq = dls_solve(&j, &v, lambda);
        // Expected: only first two joints move to match v, third zero
        assert!((dq[0] - 0.5).abs() < 1e-6);
        assert!((dq[1] - 0.25).abs() < 1e-6);
        assert!(dq[2].abs() < 1e-6);
    }
}
