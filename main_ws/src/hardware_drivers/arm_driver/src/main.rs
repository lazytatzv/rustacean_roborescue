/// arm_driver: Dynamixel position-control driver for 6-DOF arm
///
/// Hardware abstraction layer between arm_controller and real Dynamixel servos.
///
/// - Subscribe: /arm_joint_commands (sensor_msgs/JointState)  — target positions from IK
/// - Publish:   /joint_states       (sensor_msgs/JointState)  — actual encoder feedback
///
/// Features:
///   1. Position Control Mode (mode 3) — servo PID handles tracking
///   2. SyncWrite for simultaneous joint motion
///   3. SyncRead for low-latency position feedback
///   4. Temperature monitoring with emergency shutdown
///   5. Watchdog — holds last position if no command received
///   6. Communication error recovery with retry logic
///   7. Proper startup: reads initial positions before accepting commands
///
/// Dynamixel XM series protocol 2.0
/// Position resolution: 4096 counts/rev, center=2048 → 0 rad

mod driver;

use anyhow::Result;
use driver::ArmDynamixelDriver;
use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, Publisher, RclrsErrorFilter, SpinOptions};
use sensor_msgs::msg::JointState;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

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

/// Hardware polling & command rate [Hz]
const HW_LOOP_HZ: u64 = 50;

/// Watchdog: if no command for this duration, hold last position
const WATCHDOG_TIMEOUT_MS: u64 = 500;

/// Temperature check interval
const TEMP_CHECK_INTERVAL_SECS: u64 = 5;

/// Maximum communication retries before declaring error
const MAX_COMM_RETRIES: u32 = 3;

/// Dynamixel profile velocity (limits max joint speed during position move).
/// Unit: 0.229 rev/min per LSB.  100 ≈ 22.9 RPM ≈ 2.4 rad/s
const DEFAULT_PROFILE_VELOCITY: u32 = 100;

// ═══════════════════════════════════════════════════════════════════════════
//  Joint configuration: name → Dynamixel ID
//  Must match the URDF joint names exactly.
// ═══════════════════════════════════════════════════════════════════════════

const ARM_JOINTS: &[(&str, u8)] = &[
    ("arm_joint1", 21),
    ("arm_joint2", 22),
    ("arm_joint3", 23),
    ("arm_joint4", 24),
    ("arm_joint5", 25),
    ("arm_joint6", 26),
];

// ═══════════════════════════════════════════════════════════════════════════
//  Hardware thread
// ═══════════════════════════════════════════════════════════════════════════

struct HwCommand {
    /// Target positions in joint order [rad]
    positions: Vec<f64>,
}

/// Acquire a Mutex lock, recovering from poison.
/// In a robotics context, poisoned Mutex data is still usable — we'd
/// rather continue operating than panic the entire hardware thread.
fn lock_or_recover<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    match mutex.lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    }
}

fn hardware_thread(
    port_name: String,
    baud_rate: u32,
    profile_velocity: u32,
    rx_cmd: Receiver<HwCommand>,
    joint_state_pub: Publisher<JointState>,
    joint_names: Vec<String>,
    error_flag: Arc<Mutex<bool>>,
    shutdown_flag: Arc<AtomicBool>,
    estop_flag: Arc<AtomicBool>,
) {
    let ids: Vec<u8> = ARM_JOINTS.iter().map(|(_, id)| *id).collect();

    // ── Initialize driver ──
    println!("🔍 arm_driver: Opening port {} at {} bps...", port_name, baud_rate);
    let mut driver = match ArmDynamixelDriver::new(&port_name, baud_rate, ids.clone()) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("🔥 arm_driver: serial init failed: {e:#}");
            *lock_or_recover(&error_flag) = true;
            return;
        }
    };

    // --- QUICK SCAN (Developer Mode) ---
    println!("🔍 arm_driver: Scanning for all active IDs (1-30)...");
    for id in 1..31 {
        if driver.ping(id).is_ok() {
            println!("   ✨ Found Dynamixel ID: {}", id);
        }
    }

    // Ping all configured motors
    println!("🔍 arm_driver: Pinging configured motors {:?}...", ids);
    if let Err(e) = driver.ping_all() {
        eprintln!("🔥 arm_driver: ping failed: {e:#}");
        *lock_or_recover(&error_flag) = true;
        return;
    }
    println!("✅ arm_driver: All motors responded to ping.");

    // Initialize position control mode
    println!("🔍 arm_driver: Setting motors to position control mode...");
    if let Err(e) = driver.init_position_mode(profile_velocity) {
        eprintln!("🔥 arm_driver: mode init failed: {e:#}");
        *lock_or_recover(&error_flag) = true;
        return;
    }

    // Read initial positions to confirm communication
    let init_positions = match driver.read_positions() {
        Ok(p) => {
            println!("✅ arm_driver: initial positions read OK");
            for (i, name) in joint_names.iter().enumerate() {
                println!("   {name}: {:.3} rad ({:.1}°)", p[i], p[i].to_degrees());
            }
            p
        }
        Err(e) => {
            eprintln!("🔥 arm_driver: initial read failed: {e:#}");
            *lock_or_recover(&error_flag) = true;
            return;
        }
    };

    println!("✅ arm_driver: hardware ready, {HW_LOOP_HZ} Hz loop started");

    let loop_period = Duration::from_millis(1000 / HW_LOOP_HZ);
    let mut last_cmd_time = Instant::now();
    let mut last_temp_check = Instant::now();
    let mut last_log_time = Instant::now();
    let mut _last_positions = init_positions;
    let mut consecutive_errors: u32 = 0;

    while !shutdown_flag.load(Ordering::Relaxed) {
        let loop_start = Instant::now();

        // ── E-Stop check ──
        if estop_flag.load(Ordering::Relaxed) {
            eprintln!("🛑 arm_driver: E-Stop received — emergency stop!");
            driver.emergency_stop();
            // Wait until shutdown signal
            while !shutdown_flag.load(Ordering::Relaxed) {
                thread::sleep(Duration::from_millis(100));
            }
            break;
        }

        // ── 1. Process incoming commands (non-blocking) ──
        let mut latest_cmd: Option<HwCommand> = None;
        while let Ok(cmd) = rx_cmd.try_recv() {
            latest_cmd = Some(cmd);
        }

        // ── 2. Write positions to servos ──
        if let Some(cmd) = latest_cmd {
            last_cmd_time = Instant::now();
            _last_positions = cmd.positions.clone();

            match driver.write_positions(&cmd.positions) {
                Ok(()) => {
                    consecutive_errors = 0;
                }
                Err(e) => {
                    consecutive_errors += 1;
                    eprintln!(
                        "⚠️  arm_driver: write error ({consecutive_errors}/{MAX_COMM_RETRIES}): {e}"
                    );
                    if consecutive_errors >= MAX_COMM_RETRIES {
                        eprintln!("🔥 arm_driver: too many errors, emergency stop");
                        driver.emergency_stop();
                    }
                }
            }
        }

        // ── 3. Read joint states and publish ──
        match driver.read_positions() {
            Ok(positions) => {
                consecutive_errors = 0;

                let mut msg = JointState::default();
                msg.header.stamp = now_stamp();
                msg.header.frame_id = "base_link".to_string();
                msg.name = joint_names.clone();
                msg.position = positions.clone();

                let _ = joint_state_pub.publish(&msg);

                // Periodic status log (every 1s)
                if last_log_time.elapsed() > Duration::from_secs(1) {
                    print!("📊 Arm Positions: ");
                    for p in &positions {
                        print!("{:>7.3} ", p);
                    }
                    println!();
                    last_log_time = Instant::now();
                }
            }
            Err(e) => {
                consecutive_errors += 1;
                eprintln!(
                    "⚠️  arm_driver: read error ({consecutive_errors}/{MAX_COMM_RETRIES}): {e}"
                );
            }
        }

        // ── 4. Temperature monitoring ──
        if last_temp_check.elapsed() > Duration::from_secs(TEMP_CHECK_INTERVAL_SECS) {
            last_temp_check = Instant::now();
            let (warnings, critical) = driver.check_temperatures();

            for (id, temp) in &warnings {
                eprintln!("⚠️  Motor ID {id}: temperature {temp}°C (warn threshold)");
            }
            if !critical.is_empty() {
                for (id, temp) in &critical {
                    eprintln!("🔥 Motor ID {id}: temperature {temp}°C — EMERGENCY STOP!");
                }
                driver.emergency_stop();
                // Temperature shutdown is permanent — wait for node restart
                while !shutdown_flag.load(Ordering::Relaxed) {
                    thread::sleep(Duration::from_millis(100));
                }
                break; // exits main while loop
            }
        }

        // ── 5. Rate limiter ──
        let elapsed = loop_start.elapsed();
        if elapsed < loop_period {
            thread::sleep(loop_period - elapsed);
        }
    }

    // ── Graceful shutdown: disable torque on all servos ──
    println!("🛑 arm_driver: shutting down — disabling torque");
    driver.torque_off_all();
    println!("✅ arm_driver: torque disabled, hardware thread exited cleanly");
}

// ═══════════════════════════════════════════════════════════════════════════
//  ROS 2 Node
// ═══════════════════════════════════════════════════════════════════════════

fn run() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("arm_driver")?;

    // ── Parameters ──
    let port_name: Arc<str> = node
        .declare_parameter("port_name")
        .default(Arc::from("/dev/ttyUSB_arm"))
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();
    let port_name = port_name.to_string();

    let baud_rate: i64 = node
        .declare_parameter("baud_rate")
        .default(115200_i64)
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();

    let profile_velocity: i64 = node
        .declare_parameter("profile_velocity")
        .default(DEFAULT_PROFILE_VELOCITY as i64)
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();

    // ── Joint name list (in URDF order) ──
    let joint_names: Vec<String> = ARM_JOINTS.iter().map(|(name, _)| name.to_string()).collect();

    // ── Publisher: /joint_states ──
    let joint_state_pub: Publisher<JointState> =
        node.create_publisher("/joint_states")?;

    // ── Command channel ──
    let (tx_cmd, rx_cmd): (Sender<HwCommand>, Receiver<HwCommand>) = channel();

    // ── Error flag ──
    let error_flag = Arc::new(Mutex::new(false));

    // ── Shutdown flag (shared with hardware thread for graceful exit) ──
    let shutdown_flag = Arc::new(AtomicBool::new(false));

    // ── E-Stop flag (shared with hardware thread for immediate stop) ──
    let estop_flag = Arc::new(AtomicBool::new(false));

    // ── Start hardware thread ──
    let joint_names_hw = joint_names.clone();
    let pub_clone = joint_state_pub.clone();
    let error_clone = Arc::clone(&error_flag);
    let shutdown_clone = Arc::clone(&shutdown_flag);
    let estop_clone = Arc::clone(&estop_flag);
    let hw_thread = thread::spawn(move || {
        hardware_thread(
            port_name,
            baud_rate as u32,
            profile_velocity as u32,
            rx_cmd,
            pub_clone,
            joint_names_hw,
            error_clone,
            shutdown_clone,
            estop_clone,
        );
    });

    // ── Subscriber: /arm_joint_commands ──
    let names_for_sub = joint_names.clone();
    let _sub = node.create_subscription::<JointState, _>(
        "/arm_joint_commands",
        move |msg: JointState| {
            if msg.name.is_empty() || msg.position.is_empty() {
                return;
            }

            // Reorder incoming JointState to match our ID order,
            // same robust pattern as flipper_driver / gripper_driver
            let mut income_map: HashMap<String, f64> = HashMap::new();
            for (i, name) in msg.name.iter().enumerate() {
                if i < msg.position.len() {
                    income_map.insert(name.clone(), msg.position[i]);
                }
            }

            let positions: Vec<f64> = names_for_sub
                .iter()
                .map(|name| income_map.get(name).copied())
                .collect::<Option<Vec<f64>>>()
                .unwrap_or_else(|| {
                    eprintln!("⚠️  arm_driver: incomplete JointState — missing joints, command ignored");
                    Vec::new()
                });

            if positions.len() != names_for_sub.len() {
                return; // incomplete — do not send to hardware
            }

            let _ = tx_cmd.send(HwCommand { positions });
        },
    )?;

    // ── Subscriber: /emergency_stop (std_msgs/Bool) ──
    // QoS must match joy_controller publisher: transient_local + reliable
    let estop_sub_flag = Arc::clone(&estop_flag);
    let _estop_sub = node.create_subscription::<std_msgs::msg::Bool, _>(
        "/emergency_stop".reliable().transient_local().keep_last(1),
        move |msg: std_msgs::msg::Bool| {
            if msg.data {
                estop_sub_flag.store(true, Ordering::Relaxed);
            }
        },
    )?;

    println!("🚀 arm_driver started");
    println!("   Subscribe: /arm_joint_commands (sensor_msgs/JointState.position)");
    println!("   Subscribe: /emergency_stop     (std_msgs/Bool)");
    println!("   Publish:   /joint_states       (sensor_msgs/JointState)");
    println!("   Motors: {} joints", ARM_JOINTS.len());
    for (name, id) in ARM_JOINTS {
        println!("     {name} → Dynamixel ID {id}");
    }

    executor.spin(SpinOptions::default()).first_error()?;

    // ── Signal hardware thread to stop and wait for clean shutdown ──
    shutdown_flag.store(true, Ordering::Relaxed);
    if hw_thread.join().is_err() {
        eprintln!("⚠️  arm_driver: hardware thread panicked during shutdown");
    }

    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("🔥 arm_driver fatal: {e:#}");
        std::process::exit(1);
    }
}
