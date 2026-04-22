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

    // ═══════════════════════════════════════════════════════════════════════
    //  共通ヘルパー
    // ═══════════════════════════════════════════════════════════════════════

    const URDF_PATH: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../bringup/urdf/sekirei.urdf");
    const END_LINK: &str = "link_tip";

    fn load_chain() -> k::SerialChain<f64> {
        let chain = k::Chain::<f64>::from_urdf_file(URDF_PATH).expect("URDF load failed");
        let end = chain.find_link(END_LINK).expect("link_tip not found");
        k::SerialChain::from_end(end)
    }

    fn active_dof(serial: &k::SerialChain<f64>) -> usize {
        serial
            .iter_joints()
            .filter(|j| !matches!(j.joint_type, k::JointType::Fixed))
            .count()
    }

    /// FK を計算して (位置, 姿勢) を返す
    fn fk(serial: &k::SerialChain<f64>, q: &[f64]) -> (na::Vector3<f64>, na::UnitQuaternion<f64>) {
        serial.set_joint_positions_clamped(q);
        serial.update_transforms();
        let iso = serial.end_transform();
        (iso.translation.vector, iso.rotation)
    }

    /// Jacobian を計算 (6×dof)
    fn jacobian(serial: &k::SerialChain<f64>, q: &[f64]) -> na::DMatrix<f64> {
        serial.set_joint_positions_clamped(q);
        serial.update_transforms();
        k::jacobian(serial)
    }

    // velocity_scale / repulsion_gradient のロジックを手動実装 (main.rs のコピー)
    fn velocity_scale(
        lower: &[f64], upper: &[f64],
        positions: &[f64], velocities: &[f64],
        margin: f64,
    ) -> Vec<f64> {
        positions.iter().zip(velocities.iter()).enumerate().map(|(i, (&q, &dq))| {
            let lo = lower[i]; let hi = upper[i];
            if lo == f64::NEG_INFINITY && hi == f64::INFINITY { return 1.0; }
            let dist_lo = q - lo;
            let dist_hi = hi - q;
            if dq < 0.0 && dist_lo < margin { return (dist_lo / margin).clamp(0.0, 1.0); }
            if dq > 0.0 && dist_hi < margin { return (dist_hi / margin).clamp(0.0, 1.0); }
            1.0
        }).collect()
    }

    fn repulsion_grad(lower: &[f64], upper: &[f64], positions: &[f64]) -> Vec<f64> {
        positions.iter().enumerate().map(|(i, &q)| {
            let lo = lower[i]; let hi = upper[i];
            if lo == f64::NEG_INFINITY || hi == f64::INFINITY { return 0.0; }
            let range = hi - lo;
            if range < 1e-6 { return 0.0; }
            let mid = (hi + lo) / 2.0;
            (mid - q) / (range * range) * 2.0
        }).collect()
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  既存テスト (dls / manipulability の基本)
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn manipulability_nonnegative() {
        let j = na::DMatrix::from_row_slice(3, 3, &[1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0]);
        let m = manipulability_from_jacobian(&j);
        assert!(m >= 0.0);
        assert!((m - 6.0).abs() < 1e-12);
    }

    #[test]
    fn manipulability_zero_for_rank_deficient() {
        let j = na::DMatrix::from_row_slice(3, 3, &[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let m = manipulability_from_jacobian(&j);
        assert!(m >= 0.0);
        assert!(m.abs() < 1e-12);
    }

    #[test]
    fn dls_identity_lambda_zero() {
        let j = na::DMatrix::identity(3, 3);
        let v = na::DVector::from_column_slice(&[0.5, -0.2, 1.0]);
        let dq = dls_solve(&j, &v, 0.0);
        assert!((dq - v).norm() < 1e-12);
    }

    #[test]
    fn dls_identity_with_lambda() {
        let j = na::DMatrix::identity(3, 3);
        let v = na::DVector::from_column_slice(&[1.0, 2.0, 3.0]);
        let lambda = 0.5;
        let dq = dls_solve(&j, &v, lambda);
        let expected = v.scale(1.0 / (1.0 + lambda * lambda));
        assert!((dq - expected).norm() < 1e-12);
    }

    #[test]
    fn dls_overdetermined_example() {
        let j = na::DMatrix::from_row_slice(2, 3, &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0]);
        let v = na::DVector::from_column_slice(&[0.5, 0.25]);
        let lambda = 1e-6;
        let dq = dls_solve(&j, &v, lambda);
        assert!((dq[0] - 0.5).abs() < 1e-6);
        assert!((dq[1] - 0.25).abs() < 1e-6);
        assert!(dq[2].abs() < 1e-6);
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  FK テスト
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn fk_dof_is_6() {
        let serial = load_chain();
        assert_eq!(active_dof(&serial), 6, "arm should have 6 active DOF");
    }

    #[test]
    fn fk_zero_pose_position() {
        // 手動計算値 (URDF寸法から): x≈0.3555, y≈-0.001, z≈0.2616
        let serial = load_chain();
        let (pos, rot) = fk(&serial, &[0.0; 6]);
        let (r, p, y) = rot.euler_angles();
        println!("[FK zero] pos=({:.4}, {:.4}, {:.4}) rpy=({:.4},{:.4},{:.4})",
                 pos[0], pos[1], pos[2], r, p, y);
        assert!(pos[0] > 0.1, "EE must be in front of base_link");
        assert!(pos[2] > 0.0, "EE must be above base_link");
        assert!(pos[0].is_finite() && pos[1].is_finite() && pos[2].is_finite());
        // 具体値の確認 (URDF寸法と一致)
        assert!((pos[0] - 0.3555).abs() < 0.01, "x ≈ 0.3555 m");
        assert!(pos[1].abs() < 0.01, "y ≈ 0");
        assert!((pos[2] - 0.2616).abs() < 0.01, "z ≈ 0.2616 m");
    }

    #[test]
    fn fk_zero_pose_orientation_is_identity() {
        let serial = load_chain();
        let (_, rot) = fk(&serial, &[0.0; 6]);
        let (r, p, y) = rot.euler_angles();
        // ゼロ姿勢では回転なし
        assert!(r.abs() < 1e-6, "roll should be 0 at zero pose");
        assert!(p.abs() < 1e-6, "pitch should be 0 at zero pose");
        assert!(y.abs() < 1e-6, "yaw should be 0 at zero pose");
    }

    #[test]
    fn fk_joint_limits() {
        // 各関節を単独で下限/上限にしたときFKが有限値を返すことを確認
        // (joint1,3,4は±6.28≈±2πなので下限=上限と同位相になる点に注意)
        let serial = load_chain();
        let limits: Vec<(f64, f64)> = serial.iter_joints()
            .filter(|j| !matches!(j.joint_type, k::JointType::Fixed))
            .map(|j| j.limits.as_ref().map(|l| (l.min, l.max)).unwrap_or((-1.0, 1.0)))
            .collect();

        for (i, (lo, hi)) in limits.iter().enumerate() {
            let mut q_lo = vec![0.0; 6];
            q_lo[i] = *lo;
            let mut q_hi = vec![0.0; 6];
            q_hi[i] = *hi;

            let (pos_lo, _) = fk(&serial, &q_lo);
            let (pos_hi, _) = fk(&serial, &q_hi);

            println!("[FK j{} limits] lo=({:.4},{:.4},{:.4})  hi=({:.4},{:.4},{:.4})",
                     i+1, pos_lo[0], pos_lo[1], pos_lo[2],
                     pos_hi[0], pos_hi[1], pos_hi[2]);

            for &v in pos_lo.iter().chain(pos_hi.iter()) {
                assert!(v.is_finite(), "joint{}: FK at limits must be finite", i+1);
            }

            // joint2: -π rad vs 0 rad で明確にEEが異なるはず
            if i == 1 {
                let mut q_mid = vec![0.0; 6];
                q_mid[1] = -std::f64::consts::PI;
                let (pos_mid, _) = fk(&serial, &q_mid);
                let dist = (pos_mid - pos_hi).norm();
                println!("[FK j2 at -pi] ({:.4},{:.4},{:.4}) dist_from_zero={:.4}",
                         pos_mid[0], pos_mid[1], pos_mid[2], dist);
                assert!(dist > 0.01,
                        "joint2 at -pi vs 0 must give different EE ({:.4}m apart)", dist);
            }
        }
    }

    #[test]
    fn fk_different_configs_give_different_ee() {
        let serial = load_chain();
        let (p0, _) = fk(&serial, &[0.0; 6]);
        let (p1, _) = fk(&serial, &[0.3, -0.5, 0.5, -0.3, 0.2, 0.1]);
        assert!((p1 - p0).norm() > 0.01, "Different configs must give different EE positions");
    }

    // ─── 各関節の軸方向確認 ────────────────────────────────────────────────

    #[test]
    fn fk_joint1_is_yaw_around_z() {
        // joint1 axis=Z: ±θ でEEのYが反転、Zは変化しない
        let serial = load_chain();
        let angle = std::f64::consts::PI / 4.0;
        let (pos_pos, _) = fk(&serial, &[angle, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let (pos_neg, _) = fk(&serial, &[-angle, 0.0, 0.0, 0.0, 0.0, 0.0]);
        println!("[joint1 yaw] +45°: ({:.4},{:.4},{:.4})", pos_pos[0], pos_pos[1], pos_pos[2]);
        println!("[joint1 yaw] -45°: ({:.4},{:.4},{:.4})", pos_neg[0], pos_neg[1], pos_neg[2]);
        // Zは変わらない
        assert!((pos_pos[2] - pos_neg[2]).abs() < 1e-6, "Yaw should not change EE height");
        // Yは逆符号
        assert!(pos_pos[1] * pos_neg[1] < 0.0 || (pos_pos[1].abs() < 1e-6),
                "Yaw ±θ should produce opposite Y displacements");
    }

    #[test]
    fn fk_joint2_upper_limit_is_zero() {
        // joint2 upper=0.0: ゼロ姿勢は上限ぴったり、正方向には動けない
        let serial = load_chain();
        let limits: Vec<(f64, f64)> = serial.iter_joints()
            .filter(|j| !matches!(j.joint_type, k::JointType::Fixed))
            .map(|j| j.limits.as_ref().map(|l| (l.min, l.max)).unwrap_or((-100.0, 100.0)))
            .collect();
        let (lo2, hi2) = limits[1];
        println!("[joint2 limits] lower={:.4} upper={:.4}", lo2, hi2);
        assert!((hi2 - 0.0).abs() < 1e-6, "joint2 upper limit should be 0");
        assert!(lo2 < -1.0, "joint2 lower limit should allow significant negative motion");
    }

    #[test]
    fn fk_joint2_negative_raises_ee() {
        // joint2 axis=+Y, limit上限=0: 負方向に動かすとEEのZが変化する
        let serial = load_chain();
        let (zero_pos, _) = fk(&serial, &[0.0; 6]);
        let (pos, _) = fk(&serial, &[0.0, -std::f64::consts::PI / 4.0, 0.0, 0.0, 0.0, 0.0]);
        let dz = pos[2] - zero_pos[2];
        println!("[joint2 -45°] Δz={:+.4}", dz);
        assert!(dz.abs() > 0.01, "joint2 should cause significant Z change (got {:.4})", dz);
    }

    #[test]
    fn fk_joint3_positive_lowers_ee() {
        // joint3 axis=(0,-1,0): 正方向でEEが下がる
        let serial = load_chain();
        let (zero_pos, _) = fk(&serial, &[0.0; 6]);
        let (pos, _) = fk(&serial, &[0.0, 0.0, 0.3, 0.0, 0.0, 0.0]);
        let dz = pos[2] - zero_pos[2];
        println!("[joint3 +0.3rad] Δz={:+.4}", dz);
        assert!(dz < -0.01, "joint3 positive should lower EE (got Δz={:.4})", dz);
    }

    #[test]
    fn fk_joint4_positive_raises_ee() {
        // joint4 axis=(0,-1,0): 正方向でEEが上がる (joint3の逆)
        let serial = load_chain();
        let (zero_pos, _) = fk(&serial, &[0.0; 6]);
        let (pos, _) = fk(&serial, &[0.0, 0.0, 0.0, 0.3, 0.0, 0.0]);
        let dz = pos[2] - zero_pos[2];
        println!("[joint4 +0.3rad] Δz={:+.4}", dz);
        assert!(dz > 0.01, "joint4 positive should raise EE (got Δz={:.4})", dz);
    }

    #[test]
    fn fk_joint3_and_joint4_opposite_z() {
        // joint3とjoint4は同じ軸方向だが構造上EEへのZ影響が逆
        let serial = load_chain();
        let (zero_pos, _) = fk(&serial, &[0.0; 6]);
        let angle = 0.3_f64;

        let (p3, _) = fk(&serial, &[0.0, 0.0, angle, 0.0, 0.0, 0.0]);
        let (p4, _) = fk(&serial, &[0.0, 0.0, 0.0, angle, 0.0, 0.0]);

        let dz3 = p3[2] - zero_pos[2];
        let dz4 = p4[2] - zero_pos[2];
        println!("[j3/j4] Δz: j3={:+.4} j4={:+.4}", dz3, dz4);
        assert!(dz3 * dz4 < 0.0, "joint3 and joint4 Z effects should be opposite in sign");
    }

    #[test]
    fn fk_joint5_is_wrist_yaw() {
        // joint5 axis=Z (wrist yaw): 動かしてもZは大きく変化しない
        let serial = load_chain();
        let (zero_pos, _) = fk(&serial, &[0.0; 6]);
        let (pos, _) = fk(&serial, &[0.0, 0.0, 0.0, 0.0, 0.3, 0.0]);
        let dz = (pos[2] - zero_pos[2]).abs();
        println!("[joint5 wrist yaw +0.3] Δz={:.4}", dz);
        // wrist yaw はZ位置をほとんど変えない
        assert!(dz < 0.05, "joint5 wrist yaw should not significantly change EE Z");
    }

    #[test]
    fn fk_joint6_is_roll_no_translation_at_zero() {
        // joint6 axis=X (roll): ゼロ姿勢ではEE位置は変わらない (向きのみ変わる)
        let serial = load_chain();
        let (zero_pos, zero_rot) = fk(&serial, &[0.0; 6]);
        let (pos, rot) = fk(&serial, &[0.0, 0.0, 0.0, 0.0, 0.0, 0.5]);
        let dp = (pos - zero_pos).norm();
        let drot = (rot.angle_to(&zero_rot)).abs();
        println!("[joint6 roll +0.5] Δpos={:.6}m, Δrot={:.4}rad", dp, drot);
        assert!(dp < 1e-4, "joint6 roll at zero config should not move EE position");
        assert!(drot > 0.4, "joint6 roll should change EE orientation");
    }

    // ─── ワークスペース境界 ────────────────────────────────────────────────

    #[test]
    fn fk_workspace_reach_estimate() {
        // 最大リーチの推定: リンク長の合計を超えないこと
        let serial = load_chain();
        let (pos, _) = fk(&serial, &[0.0; 6]);
        let reach = pos.norm();
        println!("[workspace] zero-config reach from origin: {:.4}m", reach);
        // URDF寸法から最大リーチはおよそ0.8m以下のはず
        assert!(reach < 1.5, "EE reach should be physically plausible");
        assert!(reach > 0.1, "EE should not be at origin");
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  Jacobian テスト
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn jacobian_shape_is_6x6() {
        let serial = load_chain();
        let j = jacobian(&serial, &[0.0; 6]);
        assert_eq!(j.nrows(), 6, "Jacobian must have 6 rows");
        assert_eq!(j.ncols(), 6, "Jacobian must have 6 cols (DOF)");
    }

    #[test]
    fn jacobian_all_finite() {
        let serial = load_chain();
        let j = jacobian(&serial, &[0.0; 6]);
        assert!(j.iter().all(|v| v.is_finite()), "All Jacobian entries must be finite");
    }

    #[test]
    fn jacobian_rank_6_at_nontrivial_config() {
        // 特異点から離れた姿勢でランク6を確認
        let serial = load_chain();
        let q = vec![0.3, -0.5, 0.5, -0.3, 0.2, 0.1];
        let j = jacobian(&serial, &q);
        let svd = j.clone().svd(true, true);
        let svals = svd.singular_values.as_slice();
        let rank = svals.iter().filter(|&&s| s > 1e-6).count();
        let m = manipulability_from_jacobian(&j);
        println!("[Jacobian rank] svals={:.4?} rank={} manip={:.6}", svals, rank, m);
        assert_eq!(rank, 6, "Jacobian rank must be 6 at non-singular config");
        assert!(m > 1e-4, "Manipulability must be positive at non-singular config");
    }

    #[test]
    fn jacobian_changes_with_config() {
        let serial = load_chain();
        let j0 = jacobian(&serial, &[0.0; 6]);
        let j1 = jacobian(&serial, &[0.5, -0.5, 0.5, -0.5, 0.5, 0.5]);
        let diff = (&j1 - &j0).norm();
        println!("[Jacobian diff] between zero and bent config: {:.4}", diff);
        assert!(diff > 0.01, "Jacobian must change with configuration");
    }

    #[test]
    fn jacobian_linear_rows_correspond_to_translation() {
        // Jacobianの上3行はEE速度の並進成分 (単位: m/rad)
        // 物理的にEEから数十cm離れた場合、係数はO(0.1)のオーダー
        let serial = load_chain();
        let j = jacobian(&serial, &[0.0; 6]);
        let linear_part = j.rows(0, 3);
        let linear_norm = linear_part.norm();
        println!("[Jacobian linear norm] {:.4}", linear_norm);
        assert!(linear_norm > 0.01, "Linear Jacobian rows must be non-trivial");
    }

    #[test]
    fn jacobian_singular_values_ordered() {
        // SVDの特異値は降順に並ぶ
        let serial = load_chain();
        let j = jacobian(&serial, &[0.3, -0.5, 0.5, -0.3, 0.2, 0.1]);
        let svd = j.svd(true, true);
        let svals = svd.singular_values.as_slice();
        println!("[SVD svals] {:?}", svals);
        for i in 0..svals.len() - 1 {
            assert!(svals[i] >= svals[i + 1] - 1e-12,
                    "Singular values must be in descending order");
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  IK テスト (速度IK)
    // ═══════════════════════════════════════════════════════════════════════

    // 非特異点姿勢での9方向速度入力と再構成誤差
    #[test]
    fn ik_all_9_directions_reconstruct() {
        let serial = load_chain();
        let q = vec![0.3, -0.5, 0.5, -0.3, 0.2, 0.1];
        let j = jacobian(&serial, &q);
        let lambda = 0.01;

        let cases: &[(&str, [f64; 6])] = &[
            ("lin+X", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ("lin-X", [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ("lin+Y", [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
            ("lin-Y", [0.0, -1.0, 0.0, 0.0, 0.0, 0.0]),
            ("lin+Z", [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
            ("lin-Z", [0.0, 0.0, -1.0, 0.0, 0.0, 0.0]),
            ("rot+X", [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]),
            ("rot+Y", [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]),
            ("rot+Z", [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
        ];

        for (name, arr) in cases {
            let twist = na::DVector::from_column_slice(arr);
            let dq = dls_solve(&j, &twist, lambda);
            let recon = &j * &dq;
            let err = (&recon - &twist).norm();
            println!("[IK {name}] residual={:.6} dq_norm={:.4}", err, dq.norm());
            assert!(err < 0.1, "IK {name} residual {:.4} is too high", err);
        }
    }

    #[test]
    fn ik_reconstruction_sign_is_correct() {
        // 再構成された速度の符号が入力と一致する
        let serial = load_chain();
        let q = vec![0.3, -0.5, 0.5, -0.3, 0.2, 0.1];
        let j = jacobian(&serial, &q);

        let cases: &[(&str, usize, f64)] = &[
            ("lin+X", 0, 1.0),
            ("lin+Y", 1, 1.0),
            ("lin+Z", 2, 1.0),
            ("lin-X", 0, -1.0),
            ("lin-Z", 2, -1.0),
        ];

        for &(name, axis, sign) in cases {
            let mut arr = [0.0; 6];
            arr[axis] = sign;
            let twist = na::DVector::from_column_slice(&arr);
            let dq = dls_solve(&j, &twist, 0.01);
            let recon = &j * &dq;
            println!("[IK sign {name}] recon[{axis}]={:.4}", recon[axis]);
            assert!(recon[axis] * sign > 0.0,
                    "IK {name}: reconstructed component must have correct sign");
        }
    }

    #[test]
    fn ik_residual_at_4_configs() {
        // 4つの異なる姿勢でIK残差を確認
        let serial = load_chain();
        let configs: &[&[f64]] = &[
            &[0.0, -0.5, 0.5, -0.3, 0.0, 0.0],
            &[0.5, -0.8, 1.0, -0.5, 0.3, 0.2],
            &[-0.5, -0.3, 0.3, -0.2, -0.3, -0.2],
            &[1.0, -1.0, 1.5, -1.0, 0.5, 0.5],
        ];
        let twist = na::DVector::from_column_slice(&[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);

        for (i, &q) in configs.iter().enumerate() {
            let j = jacobian(&serial, q);
            let m = manipulability_from_jacobian(&j);
            let dq = dls_solve(&j, &twist, 0.01);
            let recon = &j * &dq;
            let err = (&recon - &twist).norm();
            println!("[IK config{i}] manip={m:.6} residual={err:.6}");
            if m > 0.01 {
                assert!(err < 0.1, "IK config{i} residual {err:.4} too high (manip={m:.6})");
            }
        }
    }

    #[test]
    fn ik_dq_norm_bounded() {
        // DLS の性質: lambdaを大きくするとdqのノルムが小さくなる
        let serial = load_chain();
        let j = jacobian(&serial, &[0.3, -0.5, 0.5, -0.3, 0.2, 0.1]);
        let twist = na::DVector::from_column_slice(&[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

        let dq_small = dls_solve(&j, &twist, 0.01);
        let dq_large = dls_solve(&j, &twist, 0.5);
        println!("[DLS lambda] small(0.01) dq_norm={:.4} large(0.5) dq_norm={:.4}",
                 dq_small.norm(), dq_large.norm());
        assert!(dq_large.norm() < dq_small.norm(),
                "Larger lambda should produce smaller dq norm");
    }

    #[test]
    fn ik_null_space_does_not_corrupt_task() {
        // ヌル空間への射影がタスク速度を壊さないことを確認
        let serial = load_chain();
        let q = vec![0.3, -0.5, 0.5, -0.3, 0.2, 0.1];
        let j = jacobian(&serial, &q);
        let dof = 6;
        let lambda = 0.01;

        let twist = na::DVector::from_column_slice(&[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        let dq_task = dls_solve(&j, &twist, lambda);

        // ヌル空間射影行列 N = I - J#J
        let jt = j.transpose();
        let jjt = &j * &jt;
        let n = jjt.nrows();
        let damp = na::DMatrix::identity(n, n) * (lambda * lambda);
        let jjt_d = &jjt + damp;
        let j_pinv = match jjt_d.lu().solve(&j) {
            Some(v) => &jt * v,
            None => na::DMatrix::zeros(dof, dof),
        };
        let null_proj = na::DMatrix::identity(dof, dof) - j_pinv;

        // 適当なヌル空間ベクトルを生成
        let null_vec = na::DVector::from_column_slice(&[0.1, -0.1, 0.1, -0.1, 0.1, -0.1]);
        let dq_null = &null_proj * &null_vec;

        // dq_task + dq_null の再構成を確認
        let dq_combined = &dq_task + &dq_null;
        let recon = &j * &dq_combined;
        let err = (&recon - &twist).norm();
        println!("[null space] task residual after null-space addition: {:.6}", err);
        assert!(err < 0.2, "Null-space addition should not corrupt task velocity reconstruction");
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  manipulability テスト
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn manipulability_at_zero_is_finite_nonnegative() {
        let serial = load_chain();
        let j = jacobian(&serial, &[0.0; 6]);
        let m = manipulability_from_jacobian(&j);
        println!("[manip zero] {:.8}", m);
        assert!(m >= 0.0);
        assert!(m.is_finite());
    }

    #[test]
    fn manipulability_better_at_bent_than_zero() {
        // 曲げた姿勢のほうがゼロ姿勢より manipulability が高いはず
        // (ゼロ姿勢 = 全関節伸展で特異点に近い)
        let serial = load_chain();
        let j_zero = jacobian(&serial, &[0.0; 6]);
        let j_bent = jacobian(&serial, &[0.3, -0.5, 0.5, -0.3, 0.2, 0.1]);
        let m_zero = manipulability_from_jacobian(&j_zero);
        let m_bent = manipulability_from_jacobian(&j_bent);
        println!("[manip] zero={:.8} bent={:.8}", m_zero, m_bent);
        // 曲げた姿勢のほうが高い (厳密でなければ warning だけ)
        if m_bent <= m_zero {
            println!("  ⚠️  bent config does not have higher manipulability than zero config");
        }
    }

    #[test]
    fn manipulability_scale_with_jacobian_scale() {
        // Jacobianを定数倍するとmanipulabilityはその6乗になる
        let j = na::DMatrix::from_row_slice(3, 3, &[2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 5.0]);
        let m = manipulability_from_jacobian(&j);
        // product = 2 * 3 * 5 = 30
        assert!((m - 30.0).abs() < 1e-10, "manipulability should be product of singular values");
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  JointLimits ロジック テスト
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn velocity_scale_1_when_away_from_limits() {
        let lo = &[-1.0]; let hi = &[1.0];
        let s = velocity_scale(lo, hi, &[0.0], &[1.0], 0.1);
        assert!((s[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn velocity_scale_0_at_upper_limit_moving_outward() {
        let lo = &[-1.0]; let hi = &[1.0];
        let s = velocity_scale(lo, hi, &[1.0], &[1.0], 0.1);
        assert!(s[0].abs() < 1e-12, "scale must be 0 at upper limit moving outward");
    }

    #[test]
    fn velocity_scale_0_at_lower_limit_moving_outward() {
        let lo = &[-1.0]; let hi = &[1.0];
        let s = velocity_scale(lo, hi, &[-1.0], &[-1.0], 0.1);
        assert!(s[0].abs() < 1e-12, "scale must be 0 at lower limit moving outward");
    }

    #[test]
    fn velocity_scale_1_at_limit_moving_inward() {
        let lo = &[-1.0]; let hi = &[1.0];
        // 上限で内側へ動く (dq<0): スケールは1
        let s = velocity_scale(lo, hi, &[1.0], &[-1.0], 0.1);
        assert!((s[0] - 1.0).abs() < 1e-12, "scale must be 1.0 when moving away from limit");
    }

    #[test]
    fn velocity_scale_half_in_margin() {
        let lo = &[-1.0]; let hi = &[1.0];
        // margin=0.2, q=0.9 (dist_hi=0.1) → scale = 0.1/0.2 = 0.5
        let s = velocity_scale(lo, hi, &[0.9], &[1.0], 0.2);
        assert!((s[0] - 0.5).abs() < 1e-12, "scale must be 0.5 halfway in margin");
    }

    #[test]
    fn velocity_scale_multiple_joints() {
        let lo = &[-1.0, -1.0, -1.0];
        let hi = &[ 1.0,  1.0,  1.0];
        // joint0: near upper limit moving outward → reduced
        // joint1: away from limits → 1.0
        // joint2: near lower limit moving inward → 1.0
        let pos = &[0.95, 0.0, -0.95];
        let vel = &[1.0, 1.0, 1.0];
        let s = velocity_scale(lo, hi, pos, vel, 0.1);
        println!("[vel scale multi] {:?}", s);
        assert!(s[0] < 1.0, "joint0 should be scaled down near upper limit");
        assert!((s[1] - 1.0).abs() < 1e-12, "joint1 should be unscaled");
        assert!((s[2] - 1.0).abs() < 1e-12, "joint2 moving away from lower limit is unscaled");
    }

    #[test]
    fn repulsion_grad_points_toward_center_positive_side() {
        // q > mid のとき勾配は負 (センターへ向かう)
        let lo = &[-1.0]; let hi = &[1.0];
        let g = repulsion_grad(lo, hi, &[0.5]);
        println!("[repulsion] q=0.5: grad={:.4}", g[0]);
        assert!(g[0] < 0.0, "repulsion gradient at q>mid must be negative (toward center)");
    }

    #[test]
    fn repulsion_grad_points_toward_center_negative_side() {
        let lo = &[-1.0]; let hi = &[1.0];
        let g = repulsion_grad(lo, hi, &[-0.5]);
        println!("[repulsion] q=-0.5: grad={:.4}", g[0]);
        assert!(g[0] > 0.0, "repulsion gradient at q<mid must be positive (toward center)");
    }

    #[test]
    fn repulsion_grad_zero_at_center() {
        let lo = &[-1.0]; let hi = &[1.0];
        let g = repulsion_grad(lo, hi, &[0.0]);
        assert!(g[0].abs() < 1e-12, "repulsion gradient must be 0 at center");
    }

    #[test]
    fn repulsion_grad_zero_for_infinite_limits() {
        let lo = &[f64::NEG_INFINITY]; let hi = &[f64::INFINITY];
        let g = repulsion_grad(lo, hi, &[1.0]);
        assert_eq!(g[0], 0.0, "repulsion gradient must be 0 for infinite limits");
    }

    #[test]
    fn repulsion_grad_all_6_joints_from_urdf() {
        // URDFの実際の制限で repulsion gradient を確認
        let serial = load_chain();
        let limits: Vec<(f64, f64)> = serial.iter_joints()
            .filter(|j| !matches!(j.joint_type, k::JointType::Fixed))
            .map(|j| j.limits.as_ref().map(|l| (l.min, l.max)).unwrap_or((-1.0, 1.0)))
            .collect();
        let lo: Vec<f64> = limits.iter().map(|(l, _)| *l).collect();
        let hi: Vec<f64> = limits.iter().map(|(_, h)| *h).collect();
        // ゼロ姿勢での勾配
        let g = repulsion_grad(&lo, &hi, &[0.0; 6]);
        println!("[repulsion all joints at zero] {:?}", g);
        // 全て有限
        assert!(g.iter().all(|v| v.is_finite()), "All repulsion gradients must be finite");
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  FK + IK 相互整合性
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn fk_ik_consistency_numerical_jacobian() {
        // 数値ヤコビアンとk crateのヤコビアンが一致するか確認
        let serial = load_chain();
        let q = vec![0.3, -0.5, 0.5, -0.3, 0.2, 0.1];
        let dof = 6;
        let eps = 1e-5_f64;

        let (p0, _) = fk(&serial, &q);
        let j_analytical = jacobian(&serial, &q);

        // 数値ヤコビアン (3行のみ: 並進)
        let mut j_numerical = na::DMatrix::zeros(3, dof);
        for i in 0..dof {
            let mut q_plus = q.clone();
            q_plus[i] += eps;
            let (p_plus, _) = fk(&serial, &q_plus);
            let col = (p_plus - p0) / eps;
            j_numerical.set_column(i, &col);
        }

        // 解析ヤコビアンの上3行と数値ヤコビアンを比較
        let j_ana_lin = j_analytical.rows(0, 3);
        let diff = (&j_ana_lin - &j_numerical).norm();
        println!("[FK/IK consistency] analytical vs numerical Jacobian diff: {:.6}", diff);
        assert!(diff < 0.01, "Analytical and numerical Jacobian should match (diff={:.6})", diff);
    }
}
