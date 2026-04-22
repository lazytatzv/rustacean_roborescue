/// arm_controller: Professional-grade Jacobian-based velocity IK + Joint Control
///
/// Features: Synchronous Damped Least Squares (DLS) Jacobian IK with joint limit avoidance.

use anyhow::{Context as AnyhowContext, Result};
use geometry_msgs::msg::{PoseStamped, Twist};
use k::prelude::*;
use k::{Chain, SerialChain};
use k::nalgebra as na;
use rclrs::{Context, CreateBasicExecutor, Publisher, SpinOptions, RclrsErrorFilter};
use sensor_msgs::msg::JointState;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};
use std::io::Write;

// ═══════════════════════════════════════════════════════════════════════════
//  Constants
// ═══════════════════════════════════════════════════════════════════════════

const DEFAULT_DLS_LAMBDA_BASE: f64 = 0.01;
const DEFAULT_DLS_LAMBDA_MAX: f64 = 0.15;
const DEFAULT_MANIPULABILITY_THRESHOLD: f64 = 0.005;
const DEFAULT_MANIPULABILITY_LOCKOUT: f64 = 0.0001;
const DEFAULT_WATCHDOG_MS: u64 = 1000;
const DEFAULT_CONTROL_PERIOD_MS: u64 = 20;
const DEFAULT_JOINT_VEL_LIMIT: f64 = 1.5;
const DEFAULT_JOINT_LIMIT_MARGIN: f64 = 0.10;
const DEFAULT_NULL_SPACE_REPULSION_GAIN: f64 = 0.5;

// ═══════════════════════════════════════════════════════════════════════════
//  Types
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone)]
struct IkConfig {
    dls_lambda_base: f64,
    dls_lambda_max: f64,
    manipulability_threshold: f64,
    manipulability_lockout: f64,
    joint_vel_limit: f64,
    joint_limit_margin: f64,
    null_space_repulsion_gain: f64,
}

#[derive(Clone)]
struct JointLimits {
    lower: Vec<f64>,
    upper: Vec<f64>,
}

impl JointLimits {
    fn from_serial_chain(chain: &SerialChain<f64>) -> Self {
        let mut lower = Vec::new();
        let mut upper = Vec::new();
        for joint in chain.iter_joints() {
            if matches!(joint.joint_type, k::JointType::Fixed) {
                continue;
            }
            if let Some(ref limits) = joint.limits {
                lower.push(limits.min);
                upper.push(limits.max);
            } else {
                lower.push(-f64::INFINITY);
                upper.push(f64::INFINITY);
            }
        }
        Self { lower, upper }
    }

    fn velocity_scale(&self, positions: &[f64], velocities: &[f64], margin: f64) -> Vec<f64> {
        positions
            .iter()
            .zip(velocities.iter())
            .enumerate()
            .map(|(i, (&p, &v))| {
                if v > 0.0 {
                    let dist = self.upper[i] - p;
                    if dist < margin { (dist / margin).max(0.0) } else { 1.0 }
                } else if v < 0.0 {
                    let dist = p - self.lower[i];
                    if dist < margin { (dist / margin).max(0.0) } else { 1.0 }
                } else {
                    1.0
                }
            })
            .collect()
    }

    fn repulsion_gradient(&self, positions: &[f64]) -> na::DVector<f64> {
        let n = positions.len();
        let mut grad = na::DVector::zeros(n);
        for i in 0..n {
            let mid = (self.lower[i] + self.upper[i]) / 2.0;
            let range = self.upper[i] - self.lower[i];
            if range > 0.0 {
                grad[i] = -2.0 * (positions[i] - mid) / range.powi(2);
            }
        }
        grad
    }
}

struct NodeState {
    joint_positions: HashMap<String, f64>,
    target_twist: [f64; 6],
    target_joint_vel: Vec<f64>,
    target_positions: Option<Vec<f64>>,
    last_cmd_time: Instant,
    has_joint_feedback: bool,
    is_ik_mode: bool,
}

struct IkResult {
    joint_velocities: na::DVector<f64>,
    manipulability: f64,
}

fn now_stamp() -> builtin_interfaces::msg::Time {
    let now = Instant::now();
    let duration = now.elapsed();
    builtin_interfaces::msg::Time {
        sec: duration.as_secs() as i32,
        nanosec: duration.subsec_nanos(),
    }
}

fn lock_or_recover<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    match mutex.lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    }
}

fn solve_velocity_ik(
    chain: &SerialChain<f64>,
    twist: &na::DVector<f64>,
    limits: &JointLimits,
    joint_positions: &[f64],
    cfg: &IkConfig,
) -> IkResult {
    chain.set_joint_positions_clamped(joint_positions);
    chain.update_transforms();
    let jacobian = k::jacobian(chain);
    let jt = jacobian.transpose();
    let n = jacobian.nrows();
    let dof = jacobian.ncols();

    let svd = jacobian.clone().svd(true, true);
    let svals = svd.singular_values;
    let manipulability = svals.iter().fold(1.0_f64, |acc, &x| acc * x.max(0.0));

    if manipulability < cfg.manipulability_lockout {
        return IkResult { joint_velocities: na::DVector::zeros(dof), manipulability };
    }

    let lambda = if manipulability < cfg.manipulability_threshold {
        let ratio = manipulability / cfg.manipulability_threshold;
        cfg.dls_lambda_base + cfg.dls_lambda_max * (1.0 - ratio)
    } else {
        cfg.dls_lambda_base
    };

    let damping = na::DMatrix::<f64>::identity(n, n) * (lambda * lambda);
    let jjt = &jacobian * &jt;
    let jjt_damped = &jjt + damping;

    let dq_task = match jjt_damped.clone().lu().solve(twist) {
        Some(y) => &jt * y,
        None => return IkResult { joint_velocities: na::DVector::zeros(dof), manipulability }
    };

    let j_pinv = match jjt_damped.lu().solve(&jacobian) {
        Some(inv_jjt_j) => &jt * inv_jjt_j,
        None => na::DMatrix::zeros(dof, dof),
    };
    let null_projector = na::DMatrix::<f64>::identity(dof, dof) - j_pinv;
    let repulsion = limits.repulsion_gradient(joint_positions) * cfg.null_space_repulsion_gain;
    let dq_null = &null_projector * &repulsion;

    IkResult { joint_velocities: dq_task + dq_null, manipulability }
}

fn run() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("arm_controller")?;

    let robot_desc: Arc<str> = node.declare_parameter("robot_description").default(Arc::from("")).mandatory()?.get();
    let urdf_path_param: Arc<str> = node.declare_parameter("urdf_path").default(Arc::from("")).mandatory()?.get();

    // Workaround: Always load via from_urdf_file to ensure compatibility with k-crate traits
    let final_urdf_path = if !robot_desc.is_empty() {
        let temp_path = std::env::temp_dir().join("robot_description_arm.urdf");
        let mut file = std::fs::File::create(&temp_path)?;
        file.write_all(robot_desc.as_bytes())?;
        temp_path.to_string_lossy().to_string()
    } else if !urdf_path_param.is_empty() {
        urdf_path_param.to_string()
    } else {
        return Err(anyhow::anyhow!("No URDF provided."));
    };

    println!("🏗️  Loading robot model from: {}", final_urdf_path);
    let chain_full = Chain::<f64>::from_urdf_file(&final_urdf_path)
        .map_err(|e| anyhow::anyhow!("Failed to load URDF from {}: {}", final_urdf_path, e))?;

    let end_link: Arc<str> = node.declare_parameter("end_link").default(Arc::from("link_tip")).mandatory()?.get();
    let end_link = end_link.to_string();
    let watchdog_ms: i64 = node.declare_parameter("watchdog_timeout_ms").default(DEFAULT_WATCHDOG_MS as i64).mandatory()?.get();
    let watchdog_timeout = Duration::from_millis(watchdog_ms as u64);

    let ik_config = IkConfig {
        dls_lambda_base: node.declare_parameter("dls_lambda_base").default(DEFAULT_DLS_LAMBDA_BASE).mandatory()?.get(),
        dls_lambda_max: node.declare_parameter("dls_lambda_max").default(DEFAULT_DLS_LAMBDA_MAX).mandatory()?.get(),
        manipulability_threshold: node.declare_parameter("manipulability_threshold").default(DEFAULT_MANIPULABILITY_THRESHOLD).mandatory()?.get(),
        manipulability_lockout: node.declare_parameter("manipulability_lockout").default(DEFAULT_MANIPULABILITY_LOCKOUT).mandatory()?.get(),
        joint_vel_limit: node.declare_parameter("joint_vel_limit").default(DEFAULT_JOINT_VEL_LIMIT).mandatory()?.get(),
        joint_limit_margin: node.declare_parameter("joint_limit_margin").default(DEFAULT_JOINT_LIMIT_MARGIN).mandatory()?.get(),
        null_space_repulsion_gain: node.declare_parameter("null_space_repulsion_gain").default(DEFAULT_NULL_SPACE_REPULSION_GAIN).mandatory()?.get(),
    };

    let end_node = chain_full.find_link(&end_link).with_context(|| format!("End link '{}' not found in URDF", end_link))?;
    let serial = SerialChain::from_end(end_node);
    let joint_names: Vec<String> = serial.iter_joints()
        .filter(|j| !matches!(j.joint_type, k::JointType::Fixed))
        .map(|j| j.name.clone()).collect();
    let dof = joint_names.len();
    let limits = JointLimits::from_serial_chain(&serial);

    let state = Arc::new(Mutex::new(NodeState {
        joint_positions: HashMap::new(),
        target_twist: [0.0; 6],
        target_joint_vel: vec![0.0; dof],
        target_positions: None,
        last_cmd_time: Instant::now() - watchdog_timeout * 2,
        has_joint_feedback: false,
        is_ik_mode: true,
    }));

    let estop_flag = Arc::new(AtomicBool::new(false));

    let publisher: Publisher<JointState> = node.create_publisher("/arm_joint_commands")?;
    let pose_pub: Publisher<PoseStamped> = node.create_publisher("/arm_ee_pose")?;
    let diag_manip_pub: Publisher<std_msgs::msg::Float64> = node.create_publisher("/diagnostics/manipulability")?;

    let state_js = Arc::clone(&state);
    let _js_sub = node.create_subscription::<JointState, _>("/joint_states", move |msg: JointState| {
        let mut s = lock_or_recover(&state_js);
        for (name, pos) in msg.name.iter().zip(msg.position.iter()) {
            s.joint_positions.insert(name.clone(), *pos);
        }
        s.has_joint_feedback = true;
    })?;

    let state_twist = Arc::clone(&state);
    let _twist_sub = node.create_subscription::<Twist, _>("/arm_cmd_vel", move |msg: Twist| {
        let mut s = lock_or_recover(&state_twist);
        s.target_twist = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z];
        s.last_cmd_time = Instant::now();
        s.is_ik_mode = true;
    })?;

    let state_jv = Arc::clone(&state);
    let _jv_sub = node.create_subscription::<JointState, _>("/arm_joint_cmd_vel", move |msg: JointState| {
        let mut s = lock_or_recover(&state_jv);
        if msg.velocity.len() == dof {
            s.target_joint_vel = msg.velocity;
            s.last_cmd_time = Instant::now();
            s.is_ik_mode = false;
        }
    })?;

    let estop_sub_flag = Arc::clone(&estop_flag);
    let _estop_sub = node.create_subscription::<std_msgs::msg::Bool, _>("/emergency_stop", move |msg: std_msgs::msg::Bool| {
        estop_sub_flag.store(msg.data, Ordering::Relaxed);
    })?;

    let control_period_ms: i64 = node.declare_parameter("control_period_ms").default(DEFAULT_CONTROL_PERIOD_MS as i64).mandatory()?.get();
    let control_period = Duration::from_millis(control_period_ms as u64);
    let dt = control_period_ms as f64 / 1000.0;
    
    let state_timer = Arc::clone(&state);
    let serial_chain = serial.clone();
    let names_clone = joint_names.clone();
    let cfg_for_timer = ik_config.clone();
    let limits_for_timer = limits.clone();
    let estop_timer = Arc::clone(&estop_flag);
    let mut loop_count = 0;

    let _timer = node.create_timer_repeating(control_period, move || {
        loop_count += 1;
        if estop_timer.load(Ordering::Relaxed) { return; }
        let mut s = lock_or_recover(&state_timer);

        let current_positions: Vec<f64> = names_clone.iter()
            .map(|n| s.joint_positions.get(n).copied().unwrap_or(0.0)).collect();

        if !s.has_joint_feedback { return; }

        if s.target_positions.is_none() { s.target_positions = Some(current_positions.clone()); }
        let target_positions = s.target_positions.clone().unwrap();

        serial_chain.set_joint_positions_clamped(&current_positions);
        serial_chain.update_transforms();
        let iso = serial_chain.end_transform();

        let mut pose_msg = PoseStamped::default();
        pose_msg.header.stamp = now_stamp();
        pose_msg.header.frame_id = "base_link".to_string();
        pose_msg.pose.position.x = iso.translation.vector[0];
        pose_msg.pose.position.y = iso.translation.vector[1];
        pose_msg.pose.position.z = iso.translation.vector[2];
        let q = iso.rotation.quaternion();
        pose_msg.pose.orientation.x = q.coords[0];
        pose_msg.pose.orientation.y = q.coords[1];
        pose_msg.pose.orientation.z = q.coords[2];
        pose_msg.pose.orientation.w = q.coords[3];
        let _ = pose_pub.publish(&pose_msg);

        // --- PERIODIC DIAGNOSTICS (Every 3 seconds) ---
        if loop_count % 150 == 0 {
            println!("📈 [Arm Status] Mode:{} | Pose: x={:.2} y={:.2} z={:.2}", 
                if s.is_ik_mode { "IK" } else { "JOINT" },
                iso.translation.vector[0], iso.translation.vector[1], iso.translation.vector[2]);
        }

        let final_vel: Vec<f64>;
        if s.last_cmd_time.elapsed() > watchdog_timeout {
            final_vel = vec![0.0; dof];
            s.target_positions = Some(current_positions.clone());
        } else if s.is_ik_mode {
            let twist_vec = na::DVector::from_column_slice(&s.target_twist);
            if twist_vec.iter().all(|v: &f64| v.abs() < 1e-6) {
                final_vel = vec![0.0; dof];
            } else {
                let res = solve_velocity_ik(&serial_chain, &twist_vec, &limits_for_timer, &current_positions, &cfg_for_timer);
                let raw_vel: Vec<f64> = res.joint_velocities.iter().copied().collect();
                let scale = limits_for_timer.velocity_scale(&current_positions, &raw_vel, cfg_for_timer.joint_limit_margin);
                
                if loop_count % 50 == 0 {
                    for (i, &sc) in scale.iter().enumerate() {
                        if sc < 0.5 && raw_vel[i].abs() > 1e-3 { println!("⚠️  Limit: {} (Servo #{})", names_clone[i], i+1); }
                    }
                }

                final_vel = raw_vel.iter().zip(scale.iter())
                    .map(|(&v, &sc): (&f64, &f64)| (v * sc).clamp(-cfg_for_timer.joint_vel_limit, cfg_for_timer.joint_vel_limit)).collect();
                if loop_count % 5 == 0 { let _ = diag_manip_pub.publish(&std_msgs::msg::Float64 { data: res.manipulability }); }
            }
        } else {
            let raw_vel = s.target_joint_vel.clone();
            let scale = limits_for_timer.velocity_scale(&current_positions, &raw_vel, cfg_for_timer.joint_limit_margin);
            final_vel = raw_vel.iter().zip(scale.iter())
                .map(|(&v, &sc): (&f64, &f64)| (v * sc).clamp(-cfg_for_timer.joint_vel_limit, cfg_for_timer.joint_vel_limit)).collect();
        }

        let new_targets: Vec<f64> = target_positions.iter().zip(final_vel.iter()).enumerate()
            .map(|(i, (&p, &v))| (p + v * dt).clamp(limits_for_timer.lower[i], limits_for_timer.upper[i])).collect();
        
        let mut js_out = JointState::default();
        js_out.header.stamp = now_stamp();
        js_out.name = names_clone.clone();
        js_out.position = new_targets.clone();
        let _ = publisher.publish(&js_out);
        s.target_positions = Some(new_targets);
    })?;

    println!("🚀 arm_controller started (Synchronous IK)");
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

fn main() {
    if let Err(e) = run() { eprintln!("🔥 fatal: {e:#}"); std::process::exit(1); }
}
