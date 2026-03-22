/// arm_controller: Professional-grade Jacobian-based velocity IK + Joint Control
///
/// - Subscribe: /arm_cmd_vel  (geometry_msgs/Twist)     — end-effector velocity (IK mode)
/// - Subscribe: /arm_joint_cmd_vel (sensor_msgs/JointState) — joint velocities (Direct mode)
/// - Subscribe: /joint_states (sensor_msgs/JointState)  — actual joint positions (feedback)
/// - Publish:   /arm_joint_commands (sensor_msgs/JointState) — target positions + velocities
/// - Publish:   /arm_ee_pose (geometry_msgs/PoseStamped) — current FK result

use anyhow::{Context as AnyhowContext, Result};
use k::{Chain, SerialChain};
use k::nalgebra as na;
use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, Publisher, RclrsErrorFilter, SpinOptions};
use sensor_msgs::msg::JointState;
use geometry_msgs::msg::{PoseStamped, Twist};
use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard};
use std::sync::mpsc::{sync_channel, SyncSender, Receiver};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::atomic::AtomicU64;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Acquire a Mutex lock, recovering from poison.
fn lock_or_recover<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    match mutex.lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    }
}

/// 現在時刻を builtin_interfaces/Time に変換
fn now_stamp() -> builtin_interfaces::msg::Time {
    let dur = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    builtin_interfaces::msg::Time {
        sec: dur.as_secs() as i32,
        nanosec: dur.subsec_nanos(),
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Constants
// ═══════════════════════════════════════════════════════════════════════════

const DEFAULT_DLS_LAMBDA_BASE: f64 = 0.01;
const DEFAULT_DLS_LAMBDA_MAX: f64 = 0.15;
const DEFAULT_MANIPULABILITY_THRESHOLD: f64 = 0.005;
const DEFAULT_MANIPULABILITY_LOCKOUT: f64 = 0.0001;
const DEFAULT_WATCHDOG_MS: u64 = 500;
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
            if matches!(joint.joint_type, k::JointType::Fixed) { continue; }
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
        positions.iter().zip(velocities.iter()).enumerate().map(|(i, (&q, &dq))| {
            let lo = self.lower[i];
            let hi = self.upper[i];
            if lo == -f64::INFINITY && hi == f64::INFINITY { return 1.0; }
            let dist_lo = q - lo;
            let dist_hi = hi - q;
            if dq < 0.0 && dist_lo < margin { return (dist_lo / margin).clamp(0.0, 1.0); }
            if dq > 0.0 && dist_hi < margin { return (dist_hi / margin).clamp(0.0, 1.0); }
            1.0
        }).collect()
    }

    fn repulsion_gradient(&self, positions: &[f64]) -> na::DVector<f64> {
        let n = positions.len();
        let mut grad = na::DVector::zeros(n);
        for i in 0..n {
            let lo = self.lower[i];
            let hi = self.upper[i];
            if lo == -f64::INFINITY || hi == f64::INFINITY { continue; }
            let range = hi - lo;
            if range < 1e-6 { continue; }
            let mid = (hi + lo) / 2.0;
            grad[i] = (mid - positions[i]) / (range * range) * 2.0;
        }
        grad
    }
}

struct SharedState {
    target_twist: [f64; 6],
    target_joint_vel: Vec<f64>,
    is_ik_mode: bool,
    last_cmd_time: Instant,
    joint_positions: HashMap<String, f64>,
    has_joint_feedback: bool,
    target_positions: Option<Vec<f64>>,
}

impl SharedState {
    fn new(dof: usize) -> Self {
        Self {
            target_twist: [0.0; 6],
            target_joint_vel: vec![0.0; dof],
            is_ik_mode: true,
            last_cmd_time: Instant::now(),
            joint_positions: HashMap::new(),
            has_joint_feedback: false,
            target_positions: None,
        }
    }
}

struct IkResult {
    joint_velocities: na::DVector<f64>,
    manipulability: f64,
}

fn solve_velocity_ik(
    chain: &SerialChain<f64>,
    twist: &na::DVector<f64>,
    limits: &JointLimits,
    joint_positions: &[f64],
    cfg: &IkConfig,
) -> IkResult {
    let jacobian = k::jacobian(chain);
    let jt = jacobian.transpose();
    let n = jacobian.nrows();
    let dof = jacobian.ncols();

    // Use SVD for a numerically robust manipulability measure (product of singular values)
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
        None => return IkResult { joint_velocities: na::DVector::zeros(dof), manipulability },
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

    let urdf_path: Arc<str> = node.declare_parameter("urdf_path").default(Arc::from("")).mandatory()?.get();
    let urdf_path = urdf_path.to_string();
    let end_link: Arc<str> = node.declare_parameter("end_link").default(Arc::from("link_tip")).mandatory()?.get();
    let end_link = end_link.to_string();
    let watchdog_ms: i64 = node.declare_parameter("watchdog_timeout_ms").default(DEFAULT_WATCHDOG_MS as i64).mandatory()?.get();
    let watchdog_timeout = Duration::from_millis(watchdog_ms as u64);

    macro_rules! declare_f64 { ($name:expr, $default:expr) => { node.declare_parameter($name).default($default).mandatory()?.get() }; }
    let ik_config = IkConfig {
        dls_lambda_base:          declare_f64!("dls_lambda_base",          DEFAULT_DLS_LAMBDA_BASE),
        dls_lambda_max:           declare_f64!("dls_lambda_max",           DEFAULT_DLS_LAMBDA_MAX),
        manipulability_threshold: declare_f64!("manipulability_threshold", DEFAULT_MANIPULABILITY_THRESHOLD),
        manipulability_lockout:   declare_f64!("manipulability_lockout",   DEFAULT_MANIPULABILITY_LOCKOUT),
        joint_vel_limit:          declare_f64!("joint_vel_limit",          DEFAULT_JOINT_VEL_LIMIT),
        joint_limit_margin:       declare_f64!("joint_limit_margin",       DEFAULT_JOINT_LIMIT_MARGIN),
        null_space_repulsion_gain: declare_f64!("null_space_repulsion_gain", DEFAULT_NULL_SPACE_REPULSION_GAIN),
    };

    if urdf_path.is_empty() { anyhow::bail!("Parameter 'urdf_path' is required but was empty"); }
    let chain = Chain::<f64>::from_urdf_file(&urdf_path).with_context(|| format!("Failed to load URDF: {urdf_path}"))?;
    let end_node = chain.find_link(&end_link).with_context(|| format!("Could not find link '{end_link}'"))?;
    let serial = SerialChain::from_end(end_node);
    let joint_names: Vec<String> = serial.iter_joints().filter(|j| !matches!(j.joint_type, k::JointType::Fixed)).map(|j| j.name.clone()).collect();
    let dof = joint_names.len();
    let limits = JointLimits::from_serial_chain(&serial);

    println!("✅ Arm chain loaded: {dof} DOF, end link = '{end_link}'");

    let state = Arc::new(Mutex::new(SharedState::new(dof)));
    let estop_flag = Arc::new(AtomicBool::new(false));
    let js_counter = Arc::new(AtomicU64::new(0));

    let state_tw = Arc::clone(&state);
    let _sub_tw = node.create_subscription::<Twist, _>("/arm_cmd_vel", move |msg: Twist| {
        let mut s = lock_or_recover(&state_tw);
        s.target_twist = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z];
        s.is_ik_mode = true;
        s.last_cmd_time = Instant::now();
    })?;

    let state_jv = Arc::clone(&state);
    let names_for_jv = joint_names.clone();
    let _sub_jv = node.create_subscription::<JointState, _>("/arm_joint_cmd_vel", move |msg: JointState| {
        let mut s = lock_or_recover(&state_jv);
        s.is_ik_mode = false;
        s.last_cmd_time = Instant::now();
        let mut map = HashMap::new();
        for (i, name) in msg.name.iter().enumerate() { if i < msg.velocity.len() { map.insert(name.clone(), msg.velocity[i]); } }
        for (i, name) in names_for_jv.iter().enumerate() { s.target_joint_vel[i] = *map.get(name).unwrap_or(&0.0); }
    })?;

    let state_js = Arc::clone(&state);
    let arm_names_for_check = joint_names.clone();
    let js_count_clone = Arc::clone(&js_counter);
    let _sub_js = node.create_subscription::<JointState, _>("/joint_states", move |msg: JointState| {
        let mut s = lock_or_recover(&state_js);
        for (i, name) in msg.name.iter().enumerate() { if i < msg.position.len() { s.joint_positions.insert(name.clone(), msg.position[i]); } }
        if !s.has_joint_feedback && arm_names_for_check.iter().all(|n| s.joint_positions.contains_key(n)) {
            s.has_joint_feedback = true;
            println!("✅ Arm controller armed");
        }
        js_count_clone.fetch_add(1, Ordering::Relaxed);
    })?;

    let estop_sub_flag = Arc::clone(&estop_flag);
    let _estop_sub = node.create_subscription::<std_msgs::msg::Bool, _>("/emergency_stop".reliable().transient_local().keep_last(1),
        move |msg: std_msgs::msg::Bool| { if msg.data { estop_sub_flag.store(true, Ordering::Relaxed); } }
    )?;

    let publisher: Publisher<JointState> = node.create_publisher("/arm_joint_commands")?;
    let pose_pub: Publisher<PoseStamped> = node.create_publisher("/arm_ee_pose")?;
    // Diagnostics publishers
    let diag_loop_pub = node.create_publisher::<std_msgs::msg::Float64>("/arm_controller/diagnostics/loop_time_ms")?;
    let diag_manip_pub = node.create_publisher::<std_msgs::msg::Float64>("/arm_controller/diagnostics/manipulability")?;
    let diag_js_rate_pub = node.create_publisher::<std_msgs::msg::Float64>("/arm_controller/diagnostics/joint_state_rate_hz")?;

    // control period parameter (ms)
    let control_period_ms: i64 = node.declare_parameter("control_period_ms").default(DEFAULT_CONTROL_PERIOD_MS as i64).mandatory()?.get();
    let control_period = Duration::from_millis(control_period_ms as u64);
    let dt = control_period_ms as f64 / 1000.0;
    let state_timer = Arc::clone(&state);
    let serial_chain = serial.clone();
    let names_clone = joint_names.clone();
    let cfg = ik_config.clone();
    let estop_timer = Arc::clone(&estop_flag);
    let mut loop_count = 0;
    let mut last_js_count: u64 = 0;
    let mut last_diag_instant = Instant::now();

    // Setup a background worker for IK to avoid blocking the timer callback
    let (req_tx, req_rx): (SyncSender<(na::DVector<f64>, Vec<f64>)>, Receiver<(na::DVector<f64>, Vec<f64>)>) = sync_channel(1);
    let (res_tx, res_rx): (SyncSender<(Vec<f64>, f64)>, Receiver<(Vec<f64>, f64)>) = sync_channel(1);

    let serial_for_worker = serial_chain.clone();
    let limits_for_worker = limits.clone();
    let cfg_for_worker = cfg.clone();
    std::thread::spawn(move || {
        loop {
            match req_rx.recv() {
                Ok((twist_vec, positions)) => {
                    let res = solve_velocity_ik(&serial_for_worker, &twist_vec, &limits_for_worker, &positions, &cfg_for_worker);
                    let vel: Vec<f64> = res.joint_velocities.iter().copied().collect();
                    let _ = res_tx.send((vel, res.manipulability));
                }
                Err(_) => break,
            }
        }
    });

    let mut last_worker_result: Option<(Vec<f64>, f64)> = None;

    let _timer = node.create_timer_repeating(control_period, move || {
        loop_count += 1;
        if estop_timer.load(Ordering::Relaxed) { return; }
        let mut s = lock_or_recover(&state_timer);

        let current_positions: Vec<f64> = names_clone.iter().map(|n| s.joint_positions.get(n).copied().unwrap_or(0.0)).collect();

        // --- FEEDBACK CHECK ---
        if !s.has_joint_feedback {
            if loop_count % 50 == 0 { println!("⏳ Waiting for joint feedback..."); }
            return;
        }

        // --- INITIALIZE TARGET FROM FEEDBACK ---
        if s.target_positions.is_none() {
            s.target_positions = Some(current_positions.clone());
        }
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

        if loop_count % 50 == 0 {
            println!("📊 EE Pose: x={:.3}, y={:.3}, z={:.3}", iso.translation.vector[0], iso.translation.vector[1], iso.translation.vector[2]);
        }

        let mut joint_velocities: na::DVector<f64>;
        if s.last_cmd_time.elapsed() > watchdog_timeout {
            joint_velocities = na::DVector::zeros(dof);
            // Sync target to feedback when stationary to prevent drift buildup
            s.target_positions = Some(current_positions.clone());
        } else if s.is_ik_mode {
            let twist_vec = na::DVector::from_column_slice(&s.target_twist);
            // Try to enqueue a request for the worker (non-blocking)
            let _ = req_tx.try_send((twist_vec.clone(), current_positions.clone()));
            // Try to collect a worker result if available
            if let Ok(res) = res_rx.try_recv() {
                last_worker_result = Some(res);
            }
            if let Some((ref vel_vec, manipulability)) = last_worker_result {
                joint_velocities = na::DVector::from_vec(vel_vec.clone());
                if loop_count % 50 == 0 && manipulability < 0.001 {
                    println!("⚠️ Low manipulability: {:.6}", manipulability);
                }
                // publish manipulability occasionally
                if loop_count % 5 == 0 {
                    let mut mmsg = std_msgs::msg::Float64::default();
                    mmsg.data = manipulability;
                    let _ = diag_manip_pub.publish(&mmsg);
                }
            } else {
                // no worker result yet: fallback to zeros to avoid blocking
                joint_velocities = na::DVector::zeros(dof);
            }
        } else {
            joint_velocities = na::DVector::from_vec(s.target_joint_vel.clone());
        }

        let raw_vel: Vec<f64> = joint_velocities.iter().copied().collect();
        let scale = limits.velocity_scale(&current_positions, &raw_vel, cfg.joint_limit_margin);
        let final_vel: Vec<f64> = raw_vel.iter().zip(scale.iter()).map(|(&v, &sc)| (v * sc).clamp(-cfg.joint_vel_limit, cfg.joint_vel_limit)).collect();

        // Integrate from target_positions for smoothness, but keep it tethered to feedback
        let mut new_targets: Vec<f64> = target_positions.iter().zip(final_vel.iter()).enumerate()
            .map(|(i, (&q, &dq))| (q + dq * dt).clamp(limits.lower[i], limits.upper[i])).collect();

        // Tethering: if target drifts too far from feedback (> 0.2 rad), snap it back partially
        for i in 0..dof {
            let diff = new_targets[i] - current_positions[i];
            if diff.abs() > 0.2 { new_targets[i] = current_positions[i] + diff.signum() * 0.2; }
        }

        s.target_positions = Some(new_targets.clone());
        drop(s);

        let mut msg = JointState::default();
        msg.header.stamp = now_stamp();
        msg.header.frame_id = "base_link".to_string();
        msg.name = names_clone.clone();
        msg.position = new_targets;
        msg.velocity = final_vel;
        let _ = publisher.publish(&msg);

        // diagnostics: loop time and joint_state rate
        let now = Instant::now();
        let loop_dur = now.duration_since(last_diag_instant).as_secs_f64() * 1000.0 / (if loop_count % 1 == 0 { 1.0 } else { 1.0 });
        let mut lmsg = std_msgs::msg::Float64::default();
        lmsg.data = loop_dur;
        let _ = diag_loop_pub.publish(&lmsg);

        let js_now = js_counter.load(Ordering::Relaxed);
        let delta = js_now.saturating_sub(last_js_count) as f64;
        let elapsed = now.duration_since(last_diag_instant).as_secs_f64();
        if elapsed > 0.0 {
            let rate = delta / elapsed;
            let mut rmsg = std_msgs::msg::Float64::default();
            rmsg.data = rate;
            let _ = diag_js_rate_pub.publish(&rmsg);
        }
        last_js_count = js_now;
        last_diag_instant = now;
    })?;

    println!("🚀 arm_controller started");
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

fn main() { if let Err(e) = run() { eprintln!("🔥 fatal: {e:#}"); std::process::exit(1); } }
