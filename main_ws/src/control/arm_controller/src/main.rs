/// arm_controller: Jacobian-based velocity IK for teleoperation
///
/// - Subscribe: /arm_cmd_vel (geometry_msgs/Twist)  — end-effector velocity from joystick
/// - Publish:   /arm_joint_commands (sensor_msgs/JointState) — joint velocities to servo driver
///
/// Uses the `k` crate for kinematics.  Loads a URDF at startup (path via ROS parameter).
/// Implements Damped Least Squares (DLS) to handle singularities.
/// Watchdog stops the arm if no command arrives within the timeout.

use anyhow::{Context as AnyhowContext, Result};
use k::{Chain, SerialChain};
use k::nalgebra as na;
use rclrs::{Context, CreateBasicExecutor, Publisher, RclrsErrorFilter, SpinOptions};
use sensor_msgs::msg::JointState;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// DLS damping factor (λ).  Larger = more stable near singularity but less accurate.
const DLS_LAMBDA: f64 = 0.05;

/// Default watchdog timeout in milliseconds
const DEFAULT_WATCHDOG_MS: u64 = 500;

/// Default control loop period (50 Hz)
const CONTROL_PERIOD_MS: u64 = 20;

/// Joint velocity limit (rad/s) — clamp output for safety
const JOINT_VEL_LIMIT: f64 = 1.0;

// ---------------------------------------------------------------------------
// Shared state between subscriber callback and control loop
// ---------------------------------------------------------------------------

struct SharedState {
    /// Desired end-effector velocity [vx, vy, vz, wx, wy, wz]
    target_twist: [f64; 6],
    /// Timestamp of the last received command
    last_cmd_time: Instant,
}

impl SharedState {
    fn new() -> Self {
        Self {
            target_twist: [0.0; 6],
            last_cmd_time: Instant::now(),
        }
    }
}

// ---------------------------------------------------------------------------
// Velocity IK solver
// ---------------------------------------------------------------------------

/// Compute joint velocities from a desired end-effector twist using DLS.
///
/// dq = J^T (J J^T + λ² I)^{-1} dx
fn solve_velocity_ik(chain: &SerialChain<f64>, twist: &na::DVector<f64>) -> na::DVector<f64> {
    let jacobian: na::DMatrix<f64> = k::jacobian(chain);
    let jt: na::DMatrix<f64> = jacobian.transpose();

    let n = jacobian.nrows(); // task-space dim (6)
    let damping: na::DMatrix<f64> = na::DMatrix::identity(n, n) * (DLS_LAMBDA * DLS_LAMBDA);
    let jjt: na::DMatrix<f64> = &jacobian * &jt + damping;

    // Solve (J J^T + λ²I) y = dx  →  y
    // Then  dq = J^T y
    match jjt.lu().solve(twist) {
        Some(y) => {
            let dq: na::DVector<f64> = &jt * y;
            // Clamp each joint velocity
            na::DVector::from_iterator(
                dq.len(),
                dq.iter().map(|&v: &f64| v.clamp(-JOINT_VEL_LIMIT, JOINT_VEL_LIMIT)),
            )
        }
        None => {
            eprintln!("⚠️  DLS solve failed (singular), sending zero velocities");
            na::DVector::zeros(chain.dof())
        }
    }
}

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------

fn run() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("arm_controller")?;

    // ---- Parameters -------------------------------------------------------
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

    // ---- Load kinematic chain from URDF -----------------------------------
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

    println!("✅ Arm chain loaded: {} DOF, end link = '{end_link}'", dof);
    for (i, name) in joint_names.iter().enumerate() {
        println!("   Joint[{i}]: {name}");
    }

    // ---- Shared state -----------------------------------------------------
    let state = Arc::new(Mutex::new(SharedState::new()));

    // ---- Subscriber: /arm_cmd_vel -----------------------------------------
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

    // ---- Publisher: /arm_joint_commands ------------------------------------
    let publisher: Publisher<JointState> =
        node.create_publisher("/arm_joint_commands")?;

    // ---- Control loop (timer) ---------------------------------------------
    let state_timer = Arc::clone(&state);
    let serial_chain = serial.clone();
    let names_clone = joint_names.clone();

    let _timer = node.create_timer_repeating(
        Duration::from_millis(CONTROL_PERIOD_MS),
        move || {
            let s = state_timer.lock().unwrap();

            // Watchdog — zero velocities if command is stale
            let twist = if s.last_cmd_time.elapsed() > watchdog_timeout {
                [0.0; 6]
            } else {
                s.target_twist
            };
            drop(s); // release lock before heavy math

            let twist_vec = na::DVector::from_column_slice(&twist);

            // Update FK (use current joint angles — in real use these come from encoders)
            serial_chain.update_transforms();

            let dq = solve_velocity_ik(&serial_chain, &twist_vec);

            // Build & publish JointState
            let mut msg = JointState::default();
            msg.name = names_clone.clone();
            msg.velocity = dq.iter().copied().collect();

            let _ = publisher.publish(&msg);
        },
    )?;

    println!("🚀 arm_controller started (watchdog={watchdog_ms}ms, loop={}Hz)",
             1000 / CONTROL_PERIOD_MS);
    println!("   Subscribe: /arm_cmd_vel (geometry_msgs/Twist)");
    println!("   Publish:   /arm_joint_commands (sensor_msgs/JointState)");

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("🔥 arm_controller fatal: {e:#}");
        std::process::exit(1);
    }
}
