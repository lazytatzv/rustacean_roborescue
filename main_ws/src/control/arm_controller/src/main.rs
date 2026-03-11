/// arm_controller: Professional-grade Jacobian-based velocity IK for teleoperation
///
/// - Subscribe: /arm_cmd_vel  (geometry_msgs/Twist)     — end-effector velocity from joystick
/// - Subscribe: /joint_states (sensor_msgs/JointState)  — actual joint positions (feedback)
/// - Publish:   /arm_joint_commands (sensor_msgs/JointState) — joint velocities to servo driver
///
/// Safety features:
///   1. Joint position feedback — uses actual encoder values, not open-loop integration
///   2. Joint limit avoidance   — smooth deceleration near limits + null-space repulsion
///   3. Adaptive DLS damping    — λ grows near singularities (Nakamura–Hanafusa method)
///   4. Manipulability monitor  — logs warnings and scales speed near singular configs
///   5. Watchdog                — zero velocities if no command within timeout
///   6. Startup safety          — waits for first /joint_states before accepting commands

use anyhow::{Context as AnyhowContext, Result};
use k::{Chain, SerialChain};
use k::nalgebra as na;
use rclrs::{Context, CreateBasicExecutor, Publisher, RclrsErrorFilter, SpinOptions};
use sensor_msgs::msg::JointState;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

// ═══════════════════════════════════════════════════════════════════════════
//  Constants
// ═══════════════════════════════════════════════════════════════════════════

/// Base DLS damping factor (λ₀). Adaptive: λ = λ₀ + λ_max * (1 - w/w₀)
const DLS_LAMBDA_BASE: f64 = 0.01;

/// Maximum DLS damping factor near singularity
const DLS_LAMBDA_MAX: f64 = 0.15;

/// Manipulability threshold below which damping ramps up.
/// w = sqrt(det(J J^T)).  Typical healthy value for this arm ≈ 0.01–0.1.
const MANIPULABILITY_THRESHOLD: f64 = 0.005;

/// Manipulability below which we refuse to move (near-singular lockout)
const MANIPULABILITY_LOCKOUT: f64 = 0.0005;

/// Default watchdog timeout [ms]
const DEFAULT_WATCHDOG_MS: u64 = 500;

/// Control loop period (50 Hz)
const CONTROL_PERIOD_MS: u64 = 20;

/// Joint velocity limit [rad/s] — hard clamp on output
const JOINT_VEL_LIMIT: f64 = 1.0;

/// Safety margin from joint limits [rad].
/// Velocity is smoothly scaled to zero within this margin.
const JOINT_LIMIT_MARGIN: f64 = 0.10; // ~5.7°

/// Null-space repulsion gain for joint-limit avoidance
const NULL_SPACE_REPULSION_GAIN: f64 = 0.5;

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
    fn velocity_scale(&self, positions: &[f64], velocities: &[f64]) -> Vec<f64> {
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
                if dq < 0.0 && dist_lo < JOINT_LIMIT_MARGIN {
                    return (dist_lo / JOINT_LIMIT_MARGIN).clamp(0.0, 1.0);
                }
                // If moving toward upper limit and close
                if dq > 0.0 && dist_hi < JOINT_LIMIT_MARGIN {
                    return (dist_hi / JOINT_LIMIT_MARGIN).clamp(0.0, 1.0);
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
}

impl SharedState {
    fn new() -> Self {
        Self {
            target_twist: [0.0; 6],
            last_cmd_time: Instant::now(),
            joint_positions: HashMap::new(),
            has_joint_feedback: false,
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
    if manipulability < MANIPULABILITY_LOCKOUT {
        return IkResult {
            joint_velocities: na::DVector::zeros(dof),
            manipulability,
        };
    }

    // ── Adaptive damping (Nakamura–Hanafusa) ──
    let lambda = if manipulability < MANIPULABILITY_THRESHOLD {
        let ratio = manipulability / MANIPULABILITY_THRESHOLD;
        DLS_LAMBDA_BASE + DLS_LAMBDA_MAX * (1.0 - ratio)
    } else {
        DLS_LAMBDA_BASE
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
    // Pseudo-inverse: J^+ = J^T (J J^T + λ²I)^{-1}
    // Null-space projector: N = I - J^+ J
    let j_pinv = match jjt_damped.lu().solve(&jacobian) {
        Some(inv_jjt_j) => &jt * inv_jjt_j, // J^T · (J J^T + λ²I)^{-1} · J  ... but that's J^+ · J
        None => na::DMatrix::zeros(dof, dof),
    };
    // j_pinv is already J^+ J (dof×dof), so null projector is I - j_pinv
    let null_projector = na::DMatrix::<f64>::identity(dof, dof) - j_pinv;

    let repulsion = limits.repulsion_gradient(joint_positions) * NULL_SPACE_REPULSION_GAIN;
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

    // ── Shared state ──
    let state = Arc::new(Mutex::new(SharedState::new()));

    // ── Subscriber: /arm_cmd_vel ──
    let state_sub = Arc::clone(&state);
    let _sub = node.create_subscription::<geometry_msgs::msg::Twist, _>(
        "/arm_cmd_vel",
        move |msg: geometry_msgs::msg::Twist| {
            let mut s = state_sub.lock().unwrap();
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
    let _sub_js = node.create_subscription::<JointState, _>(
        "/joint_states",
        move |msg: JointState| {
            let mut s = state_js.lock().unwrap();
            for (i, name) in msg.name.iter().enumerate() {
                if i < msg.position.len() {
                    s.joint_positions.insert(name.clone(), msg.position[i]);
                }
            }
            if !s.has_joint_feedback {
                s.has_joint_feedback = true;
                println!("✅ First /joint_states received — arm controller armed");
            }
        },
    )?;

    // ── Publisher: /arm_joint_commands ──
    let publisher: Publisher<JointState> =
        node.create_publisher("/arm_joint_commands")?;

    // ── Control loop (50 Hz timer) ──
    let state_timer = Arc::clone(&state);
    let serial_chain = serial.clone();
    let names_clone = joint_names.clone();
    let mut last_manipulability_warn = Instant::now();
    let mut prev_locked_out = false;

    let _timer = node.create_timer_repeating(
        Duration::from_millis(CONTROL_PERIOD_MS),
        move || {
            let s = state_timer.lock().unwrap();

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
            );

            // ── Manipulability diagnostics ──
            if result.manipulability < MANIPULABILITY_LOCKOUT {
                if !prev_locked_out {
                    eprintln!(
                        "⛔ Singularity lockout! manipulability={:.6} < {:.6}",
                        result.manipulability, MANIPULABILITY_LOCKOUT
                    );
                    prev_locked_out = true;
                }
            } else {
                prev_locked_out = false;
                if result.manipulability < MANIPULABILITY_THRESHOLD
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
            let scale_factors = limits.velocity_scale(&current_positions, &raw_vel);

            let final_vel: Vec<f64> = raw_vel
                .iter()
                .zip(scale_factors.iter())
                .map(|(&v, &s)| (v * s).clamp(-JOINT_VEL_LIMIT, JOINT_VEL_LIMIT))
                .collect();

            // ── Publish ──
            let mut msg = JointState::default();
            msg.name = names_clone.clone();
            msg.velocity = final_vel;

            let _ = publisher.publish(&msg);
        },
    )?;

    println!("🚀 arm_controller started (watchdog={watchdog_ms}ms, loop={}Hz)",
             1000 / CONTROL_PERIOD_MS);
    println!("   Subscribe: /arm_cmd_vel (geometry_msgs/Twist)");
    println!("   Subscribe: /joint_states (sensor_msgs/JointState) ← feedback");
    println!("   Publish:   /arm_joint_commands (sensor_msgs/JointState)");
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
