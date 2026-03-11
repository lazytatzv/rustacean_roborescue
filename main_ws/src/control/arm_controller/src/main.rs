/// arm_controller: Professional-grade Jacobian-based velocity IK for teleoperation
///
/// - Subscribe: /arm_cmd_vel  (geometry_msgs/Twist)     — end-effector velocity from joystick
/// - Subscribe: /joint_states (sensor_msgs/JointState)  — actual joint positions (feedback)
/// - Publish:   /arm_joint_commands (sensor_msgs/JointState) — target positions + velocities
///
/// Output: JointState with both .position (target) and .velocity (informational).
/// The downstream driver (arm_driver for real HW, arm_gz_bridge for Gazebo) consumes
/// the .position field. Velocity IK is integrated internally:
///   q_target += dq * dt, then clamped to joint limits.
///
/// Safety features:
///   1. Joint position feedback — uses actual encoder values, not open-loop integration
///   2. Joint limit avoidance   — smooth deceleration near limits + null-space repulsion
///   3. Adaptive DLS damping    — λ grows near singularities (Nakamura–Hanafusa method)
///   4. Manipulability monitor  — logs warnings and scales speed near singular configs
///   5. Watchdog                — zero velocities if no command within timeout
///   6. Startup safety          — waits for first /joint_states before accepting commands
///   7. Position integration    — outputs bounded target positions (no velocity drift)

use anyhow::{Context as AnyhowContext, Result};
use k::{Chain, SerialChain};
use k::nalgebra as na;
use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, Publisher, RclrsErrorFilter, SpinOptions};
use sensor_msgs::msg::JointState;
use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard};
use std::sync::atomic::{AtomicBool, Ordering};
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
//  Default constants (overridable via ROS parameters)
// ═══════════════════════════════════════════════════════════════════════════

/// Base DLS damping factor (λ₀). Adaptive: λ = λ₀ + λ_max * (1 - w/w₀)
const DEFAULT_DLS_LAMBDA_BASE: f64 = 0.01;

/// Maximum DLS damping factor near singularity
const DEFAULT_DLS_LAMBDA_MAX: f64 = 0.15;

/// Manipulability threshold below which damping ramps up.
/// w = sqrt(det(J J^T)).  Typical healthy value for this arm ≈ 0.01–0.1.
const DEFAULT_MANIPULABILITY_THRESHOLD: f64 = 0.005;

/// Manipulability below which we refuse to move (near-singular lockout)
const DEFAULT_MANIPULABILITY_LOCKOUT: f64 = 0.0005;

/// Default watchdog timeout [ms]
const DEFAULT_WATCHDOG_MS: u64 = 500;

/// Control loop period (50 Hz)
const CONTROL_PERIOD_MS: u64 = 20;

/// Joint velocity limit [rad/s] — hard clamp on output
const DEFAULT_JOINT_VEL_LIMIT: f64 = 1.0;

/// Safety margin from joint limits [rad].
/// Velocity is smoothly scaled to zero within this margin.
const DEFAULT_JOINT_LIMIT_MARGIN: f64 = 0.10; // ~5.7°

/// Null-space repulsion gain for joint-limit avoidance
const DEFAULT_NULL_SPACE_REPULSION_GAIN: f64 = 0.5;

// ═══════════════════════════════════════════════════════════════════════════
//  IK Configuration (loaded from ROS parameters)
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

// ═══════════════════════════════════════════════════════════════════════════
//  Joint Limits (loaded from URDF by `k` crate)
// ═══════════════════════════════════════════════════════════════════════════

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
                // Continuous joint — no limits
                lower.push(-f64::INFINITY);
                upper.push(f64::INFINITY);
            }
        }
        Self { lower, upper }
    }

    /// Compute a per-joint scaling factor [0, 1] that smoothly reduces velocity
    /// as the joint approaches its limit.
    ///
    /// Returns 1.0 when far from limits, tapers to 0.0 at the limit boundary.
    /// Also blocks motion *toward* the limit (directional).
    fn velocity_scale(&self, positions: &[f64], velocities: &[f64], margin: f64) -> Vec<f64> {
        positions
            .iter()
            .zip(velocities.iter())
            .enumerate()
            .map(|(i, (&q, &dq))| {
                let lo = self.lower[i];
                let hi = self.upper[i];

                if lo == -f64::INFINITY && hi == f64::INFINITY {
                    return 1.0; // continuous joint
                }

                let dist_lo = q - lo;
                let dist_hi = hi - q;

                // If moving toward lower limit and close
                if dq < 0.0 && dist_lo < margin {
                    return (dist_lo / margin).clamp(0.0, 1.0);
                }
                // If moving toward upper limit and close
                if dq > 0.0 && dist_hi < margin {
                    return (dist_hi / margin).clamp(0.0, 1.0);
                }

                1.0
            })
            .collect()
    }

    /// Compute a null-space gradient that pushes joints away from limits.
    /// Uses the midpoint cost: h(q) = -Σ (q_i - q_mid_i)² / (q_max_i - q_min_i)²
    /// Gradient: ∂h/∂q_i pushes toward the midpoint.
    fn repulsion_gradient(&self, positions: &[f64]) -> na::DVector<f64> {
        let n = positions.len();
        let mut grad = na::DVector::zeros(n);
        for i in 0..n {
            let lo = self.lower[i];
            let hi = self.upper[i];
            if lo == -f64::INFINITY || hi == f64::INFINITY {
                continue;
            }
            let range = hi - lo;
            if range < 1e-6 {
                continue;
            }
            let mid = (hi + lo) / 2.0;
            // Gradient toward midpoint, normalized by range
            grad[i] = (mid - positions[i]) / (range * range) * 2.0;
        }
        grad
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Shared state
// ═══════════════════════════════════════════════════════════════════════════

struct SharedState {
    /// Desired end-effector velocity [vx, vy, vz, wx, wy, wz]
    target_twist: [f64; 6],
    /// Timestamp of the last received /arm_cmd_vel
    last_cmd_time: Instant,
    /// Latest joint positions from /joint_states (keyed by joint name)
    joint_positions: HashMap<String, f64>,
    /// Whether we've received at least one /joint_states message
    has_joint_feedback: bool,
    /// Integrated target positions (initialized from first /joint_states)
    target_positions: Option<Vec<f64>>,
}

impl SharedState {
    fn new() -> Self {
        Self {
            target_twist: [0.0; 6],
            last_cmd_time: Instant::now(),
            joint_positions: HashMap::new(),
            has_joint_feedback: false,
            target_positions: None,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Velocity IK solver (adaptive DLS + null-space joint-limit avoidance)
// ═══════════════════════════════════════════════════════════════════════════

struct IkResult {
    joint_velocities: na::DVector<f64>,
    manipulability: f64,
}

/// Compute joint velocities from desired end-effector twist.
///
/// Algorithm:
///   1. Compute Jacobian J and manipulability w = sqrt(det(J J^T))
///   2. Adaptive damping: λ = λ₀                        if w ≥ w_thresh
///                        λ = λ₀ + λ_max·(1 - w/w_thresh)  if w < w_thresh
///   3. DLS: dq_task = J^T (J J^T + λ² I)^{-1} dx
///   4. Null-space projection: dq_null = (I - J^+ J) · gradient
///   5. dq = dq_task + dq_null
fn solve_velocity_ik(
    chain: &SerialChain<f64>,
    twist: &na::DVector<f64>,
    limits: &JointLimits,
    joint_positions: &[f64],
    cfg: &IkConfig,
) -> IkResult {
    let jacobian = k::jacobian(chain);
    let jt = jacobian.transpose();

    let n = jacobian.nrows(); // task-space dim (6)
    let dof = jacobian.ncols();

    // ── Manipulability ──
    let jjt = &jacobian * &jt;
    let det = jjt.determinant();
    let manipulability = if det > 0.0 { det.sqrt() } else { 0.0 };

    // ── Singularity lockout ──
    if manipulability < cfg.manipulability_lockout {
        return IkResult {
            joint_velocities: na::DVector::zeros(dof),
            manipulability,
        };
    }

    // ── Adaptive damping (Nakamura–Hanafusa) ──
    let lambda = if manipulability < cfg.manipulability_threshold {
        let ratio = manipulability / cfg.manipulability_threshold;
        cfg.dls_lambda_base + cfg.dls_lambda_max * (1.0 - ratio)
    } else {
        cfg.dls_lambda_base
    };

    let damping = na::DMatrix::<f64>::identity(n, n) * (lambda * lambda);
    let jjt_damped = &jjt + damping;

    // ── DLS solve: dq_task = J^T (J J^T + λ²I)^{-1} dx ──
    let dq_task = match jjt_damped.clone().lu().solve(twist) {
        Some(y) => &jt * y,
        None => {
            eprintln!("⚠️  DLS solve failed, sending zero velocities");
            return IkResult {
                joint_velocities: na::DVector::zeros(dof),
                manipulability,
            };
        }
    };

    // ── Null-space joint limit avoidance ──
    let j_pinv = match jjt_damped.lu().solve(&jacobian) {
        Some(inv_jjt_j) => &jt * inv_jjt_j,
        None => na::DMatrix::zeros(dof, dof),
    };
    let null_projector = na::DMatrix::<f64>::identity(dof, dof) - j_pinv;

    let repulsion = limits.repulsion_gradient(joint_positions) * cfg.null_space_repulsion_gain;
    let dq_null = &null_projector * &repulsion;

    let dq = dq_task + dq_null;

    IkResult {
        joint_velocities: dq,
        manipulability,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Node
// ═══════════════════════════════════════════════════════════════════════════

fn run() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("arm_controller")?;

    // ── Parameters ──
    let urdf_path: Arc<str> = node
        .declare_parameter("urdf_path")
        .default(Arc::from(""))
        .mandatory()
        .map_err(|e| anyhow::anyhow!("Failed to declare urdf_path: {e}"))?
        .get();
    let urdf_path = urdf_path.to_string();

    let end_link: Arc<str> = node
        .declare_parameter("end_link")
        .default(Arc::from("link_tip"))
        .mandatory()
        .map_err(|e| anyhow::anyhow!("Failed to declare end_link: {e}"))?
        .get();
    let end_link = end_link.to_string();

    let watchdog_ms: i64 = node
        .declare_parameter("watchdog_timeout_ms")
        .default(DEFAULT_WATCHDOG_MS as i64)
        .mandatory()
        .map_err(|e| anyhow::anyhow!("Failed to declare watchdog_timeout_ms: {e}"))?
        .get();
    let watchdog_timeout = Duration::from_millis(watchdog_ms as u64);

    // ── IK tuning parameters (all overridable at launch) ──
    macro_rules! declare_f64 {
        ($name:expr, $default:expr) => {
            node.declare_parameter($name)
                .default($default)
                .mandatory()
                .map_err(|e| anyhow::anyhow!("Failed to declare {}: {e}", $name))?
                .get()
        };
    }

    let ik_config = IkConfig {
        dls_lambda_base:          declare_f64!("dls_lambda_base",          DEFAULT_DLS_LAMBDA_BASE),
        dls_lambda_max:           declare_f64!("dls_lambda_max",           DEFAULT_DLS_LAMBDA_MAX),
        manipulability_threshold: declare_f64!("manipulability_threshold", DEFAULT_MANIPULABILITY_THRESHOLD),
        manipulability_lockout:   declare_f64!("manipulability_lockout",   DEFAULT_MANIPULABILITY_LOCKOUT),
        joint_vel_limit:          declare_f64!("joint_vel_limit",          DEFAULT_JOINT_VEL_LIMIT),
        joint_limit_margin:       declare_f64!("joint_limit_margin",       DEFAULT_JOINT_LIMIT_MARGIN),
        null_space_repulsion_gain: declare_f64!("null_space_repulsion_gain", DEFAULT_NULL_SPACE_REPULSION_GAIN),
    };

    // ── Load kinematic chain from URDF ──
    if urdf_path.is_empty() {
        anyhow::bail!("Parameter 'urdf_path' is required but was empty");
    }
    let chain = Chain::<f64>::from_urdf_file(&urdf_path)
        .with_context(|| format!("Failed to load URDF from: {urdf_path}"))?;

    let end_node = chain
        .find_link(&end_link)
        .with_context(|| format!("Could not find link '{end_link}' in URDF"))?;
    let serial = SerialChain::from_end(end_node);

    let joint_names: Vec<String> = serial
        .iter_joints()
        .filter(|j| !matches!(j.joint_type, k::JointType::Fixed))
        .map(|j| j.name.clone())
        .collect();
    let dof = joint_names.len();

    // ── Extract joint limits from URDF ──
    let limits = JointLimits::from_serial_chain(&serial);

    println!("✅ Arm chain loaded: {dof} DOF, end link = '{end_link}'");
    for (i, name) in joint_names.iter().enumerate() {
        println!(
            "   Joint[{i}]: {name}  limits=[{:.2}, {:.2}] rad",
            limits.lower[i], limits.upper[i]
        );
    }

    // ── E-Stop flag ──
    let estop_flag = Arc::new(AtomicBool::new(false));

    // ── Shared state ──
    let state = Arc::new(Mutex::new(SharedState::new()));

    // ── Subscriber: /arm_cmd_vel ──
    let state_sub = Arc::clone(&state);
    let _sub = node.create_subscription::<geometry_msgs::msg::Twist, _>(
        "/arm_cmd_vel",
        move |msg: geometry_msgs::msg::Twist| {
            let mut s = lock_or_recover(&state_sub);
            s.target_twist = [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z,
            ];
            s.last_cmd_time = Instant::now();
        },
    )?;

    // ── Subscriber: /joint_states (feedback from servo driver / Gazebo) ──
    let state_js = Arc::clone(&state);
    let arm_names_for_check = joint_names.clone();
    let _sub_js = node.create_subscription::<JointState, _>(
        "/joint_states",
        move |msg: JointState| {
            let mut s = lock_or_recover(&state_js);
            for (i, name) in msg.name.iter().enumerate() {
                if i < msg.position.len() {
                    s.joint_positions.insert(name.clone(), msg.position[i]);
                }
            }
            // Only arm once ALL expected arm joints have been received
            if !s.has_joint_feedback
                && arm_names_for_check
                    .iter()
                    .all(|n| s.joint_positions.contains_key(n))
            {
                s.has_joint_feedback = true;
                println!("✅ All arm joints received in /joint_states — arm controller armed");
            }
        },
    )?;

    // ── Subscriber: /emergency_stop (std_msgs/Bool) ──
    let estop_sub_flag = Arc::clone(&estop_flag);
    let _estop_sub = node.create_subscription::<std_msgs::msg::Bool, _>(
        "/emergency_stop".reliable().transient_local().keep_last(1),
        move |msg: std_msgs::msg::Bool| {
            if msg.data {
                estop_sub_flag.store(true, Ordering::Relaxed);
            }
        },
    )?;

    // ── Publisher: /arm_joint_commands ──
    let publisher: Publisher<JointState> =
        node.create_publisher("/arm_joint_commands")?;

    // ── Control loop (50 Hz timer) ──
    let dt = CONTROL_PERIOD_MS as f64 / 1000.0; // integration timestep [s]
    let state_timer = Arc::clone(&state);
    let serial_chain = serial.clone();
    let names_clone = joint_names.clone();
    let mut last_manipulability_warn = Instant::now();
    let mut prev_locked_out = false;
    let cfg = ik_config.clone();
    let estop_timer = Arc::clone(&estop_flag);

    let _timer = node.create_timer_repeating(
        Duration::from_millis(CONTROL_PERIOD_MS),
        move || {
            // ── E-Stop: refuse to publish any commands ──
            if estop_timer.load(Ordering::Relaxed) {
                return;
            }

            let mut s = lock_or_recover(&state_timer);

            // ── Safety: wait for joint feedback before moving ──
            if !s.has_joint_feedback {
                return; // silently wait
            }

            // ── Watchdog: zero velocities if command is stale ──
            let twist = if s.last_cmd_time.elapsed() > watchdog_timeout {
                [0.0; 6]
            } else {
                s.target_twist
            };

            // ── Extract current joint positions in chain order ──
            let current_positions: Vec<f64> = names_clone
                .iter()
                .map(|name| s.joint_positions.get(name).copied().unwrap_or(0.0))
                .collect();

            // ── Initialize target positions from first feedback ──
            if s.target_positions.is_none() {
                s.target_positions = Some(current_positions.clone());
                println!("✅ Target positions initialized from feedback");
            }

            let target_positions = s.target_positions.clone().unwrap();
            drop(s); // release lock before computation

            // ── Update chain with actual joint positions ──
            let pos_vec: Vec<f64> = current_positions.clone();
            serial_chain
                .set_joint_positions_clamped(&pos_vec);
            serial_chain.update_transforms();

            // ── Solve velocity IK ──
            let twist_vec = na::DVector::from_column_slice(&twist);
            let result = solve_velocity_ik(
                &serial_chain,
                &twist_vec,
                &limits,
                &current_positions,
                &cfg,
            );

            // ── Manipulability diagnostics ──
            if result.manipulability < cfg.manipulability_lockout {
                if !prev_locked_out {
                    eprintln!(
                        "⛔ Singularity lockout! manipulability={:.6} < {:.6}",
                        result.manipulability, cfg.manipulability_lockout
                    );
                    prev_locked_out = true;
                }
            } else {
                prev_locked_out = false;
                if result.manipulability < cfg.manipulability_threshold
                    && last_manipulability_warn.elapsed() > Duration::from_secs(2)
                {
                    eprintln!(
                        "⚠️  Near singularity: manipulability={:.6}, damping increased",
                        result.manipulability
                    );
                    last_manipulability_warn = Instant::now();
                }
            }

            // ── Apply joint-limit velocity scaling ──
            let raw_vel: Vec<f64> = result.joint_velocities.iter().copied().collect();
            let scale_factors = limits.velocity_scale(&current_positions, &raw_vel, cfg.joint_limit_margin);

            let final_vel: Vec<f64> = raw_vel
                .iter()
                .zip(scale_factors.iter())
                .map(|(&v, &s)| (v * s).clamp(-cfg.joint_vel_limit, cfg.joint_vel_limit))
                .collect();

            // ── Integrate velocities → target positions ──
            let new_targets: Vec<f64> = target_positions
                .iter()
                .zip(final_vel.iter())
                .enumerate()
                .map(|(i, (&q, &dq))| {
                    let q_new = q + dq * dt;
                    // Hard clamp to joint limits
                    q_new.clamp(limits.lower[i], limits.upper[i])
                })
                .collect();

            // ── Store updated targets ──
            {
                let mut s = lock_or_recover(&state_timer);
                s.target_positions = Some(new_targets.clone());
            }

            // ── Publish (position + velocity) ──
            let mut msg = JointState::default();
            msg.header.stamp = now_stamp();
            msg.header.frame_id = "base_link".to_string();
            msg.name = names_clone.clone();
            msg.position = new_targets;    // ← target positions for driver
            msg.velocity = final_vel;      // ← informational (computed velocities)

            let _ = publisher.publish(&msg);
        },
    )?;

    println!("🚀 arm_controller started (watchdog={watchdog_ms}ms, loop={}Hz, dt={dt:.3}s)",
             1000 / CONTROL_PERIOD_MS);
    println!("   Subscribe: /arm_cmd_vel (geometry_msgs/Twist)");
    println!("   Subscribe: /joint_states (sensor_msgs/JointState) ← feedback");
    println!("   Publish:   /arm_joint_commands (JointState.position + .velocity)");
    println!("   Output:    integrated target positions (clamped to joint limits)");
    println!("   Safety: joint-limit avoidance, adaptive DLS, manipulability monitor");
    println!("   ⏳ Waiting for /joint_states before accepting commands...");

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("🔥 arm_controller fatal: {e:#}");
        std::process::exit(1);
    }
}
