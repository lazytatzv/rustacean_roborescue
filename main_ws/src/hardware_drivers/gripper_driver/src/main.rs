/// gripper_driver: Dynamixel current-based position control for the gripper
///
/// - Subscribe: /gripper_cmd    (custom_interfaces/GripperCommand) — position + max_current
/// - Publish:   /gripper_status (custom_interfaces/GripperStatus)  — state + pos + current + temp
///
/// Uses the same dynamixel2 crate as the other Rust hardware drivers.
/// Operates in Current-Based Position Control mode (mode 5) which allows
/// gripping force control via current limiting.
///
/// Status reports: IDLE / MOVING / GRIPPING / OVERLOAD / ERROR
/// Detection logic:
///   - MOVING:   |goal_pos - present_pos| > threshold
///   - GRIPPING: current > grip_threshold AND position near goal
///   - OVERLOAD: current > overload_threshold

use anyhow::{Context as AnyhowContext, Result};
use dynamixel2::Bus;
use dynamixel2::serial2::SerialPort;
use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, Publisher, RclrsErrorFilter, SpinOptions};
use custom_interfaces::msg::{GripperCommand as GripperCommandMsg, GripperStatus as GripperStatusMsg};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

// ---------------------------------------------------------------------------
// Dynamixel XM series Control Table
// ---------------------------------------------------------------------------
const ADDR_OPERATING_MODE: u16 = 11;
const ADDR_TORQUE_ENABLE: u16 = 64;
const ADDR_GOAL_CURRENT: u16 = 102;    // 2 bytes
const ADDR_GOAL_POSITION: u16 = 116;   // 4 bytes
const ADDR_PRESENT_CURRENT: u16 = 126; // 2 bytes (signed)
const ADDR_PRESENT_POSITION: u16 = 132; // 4 bytes
const ADDR_PRESENT_TEMPERATURE: u16 = 146; // 1 byte

/// Dynamixel XM position range (12-bit)
const DXL_POS_MIN: u32 = 0;
const DXL_POS_MAX: u32 = 4095;

/// Current-Based Position Control Mode
const MODE_CURRENT_POSITION: u8 = 5;

// ---------------------------------------------------------------------------
// Gripper state detection thresholds
// ---------------------------------------------------------------------------
const POSITION_THRESHOLD: i32 = 50;      // position error to consider "at goal"
const GRIP_CURRENT_THRESHOLD: i16 = 200; // mA — object likely gripped
const OVERLOAD_CURRENT: i16 = 800;       // mA — overload protection

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
#[derive(Clone, Copy, PartialEq)]
#[allow(dead_code)] // Error variant reserved for future fault handling
enum GripperState {
    Idle     = 0,
    Moving   = 1,
    Gripping = 2,
    Overload = 3,
    Error    = 4,
}

impl GripperState {
    fn as_u8(self) -> u8 {
        self as u8
    }
}

struct GripperCommand {
    position: u32,
    max_current: u16,
}

// Clamp position to valid Dynamixel range
fn clamp_dxl_position(pos: i32) -> u32 {
    pos.clamp(DXL_POS_MIN as i32, DXL_POS_MAX as i32) as u32
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------
struct GripperDriver {
    bus: Bus<Vec<u8>, Vec<u8>>,
    id: u8,
}

impl GripperDriver {
    fn new(port_name: &str, baud_rate: u32, id: u8) -> Result<Self> {
        let port = SerialPort::open(port_name, baud_rate)
            .context("Failed to open serial port")?;
        let bus = Bus::new(port)?;
        Ok(Self { bus, id })
    }

    fn init_current_position_mode(&mut self) -> Result<()> {
        // Torque off before mode change
        self.bus
            .write_u8(self.id, ADDR_TORQUE_ENABLE, 0)
            .map_err(|e| anyhow::anyhow!("Torque off failed: {e:?}"))?;

        // Set current-based position control mode
        self.bus
            .write_u8(self.id, ADDR_OPERATING_MODE, MODE_CURRENT_POSITION)
            .map_err(|e| anyhow::anyhow!("Mode set failed: {e:?}"))?;

        // Torque on
        self.bus
            .write_u8(self.id, ADDR_TORQUE_ENABLE, 1)
            .map_err(|e| anyhow::anyhow!("Torque on failed: {e:?}"))?;

        println!("✅ Gripper motor initialized (Current-Based Position Control)");
        Ok(())
    }

    fn set_goal(&mut self, position: u32, max_current: u16) -> Result<()> {
        self.bus
            .write_u16(self.id, ADDR_GOAL_CURRENT, max_current)
            .map_err(|e| anyhow::anyhow!("Write goal current failed: {e:?}"))?;
        self.bus
            .write_u32(self.id, ADDR_GOAL_POSITION, position)
            .map_err(|e| anyhow::anyhow!("Write goal position failed: {e:?}"))?;
        Ok(())
    }

    fn read_present_position(&mut self) -> Result<i32> {
        let val = self.bus
            .read_u32(self.id, ADDR_PRESENT_POSITION)
            .map_err(|e| anyhow::anyhow!("Read position failed: {e:?}"))?;
        Ok(val.data as i32)
    }

    fn read_present_current(&mut self) -> Result<i16> {
        let val = self.bus
            .read_u16(self.id, ADDR_PRESENT_CURRENT)
            .map_err(|e| anyhow::anyhow!("Read current failed: {e:?}"))?;
        Ok(val.data as i16)
    }

    fn read_temperature(&mut self) -> Result<u8> {
        let val = self.bus
            .read_u8(self.id, ADDR_PRESENT_TEMPERATURE)
            .map_err(|e| anyhow::anyhow!("Read temperature failed: {e:?}"))?;
        Ok(val.data)
    }

    /// Disable torque (safe shutdown)
    fn torque_off(&mut self) {
        let _ = self.bus.write_u8(self.id, ADDR_TORQUE_ENABLE, 0);
    }
}

// ---------------------------------------------------------------------------
// State detection
// ---------------------------------------------------------------------------
fn detect_state(goal_pos: u32, present_pos: i32, current: i16) -> GripperState {
    if current.abs() > OVERLOAD_CURRENT {
        return GripperState::Overload;
    }

    let pos_error = (goal_pos as i32 - present_pos).abs();

    if pos_error > POSITION_THRESHOLD {
        if current.abs() > GRIP_CURRENT_THRESHOLD {
            // Trying to move but high current → something is gripped
            GripperState::Gripping
        } else {
            GripperState::Moving
        }
    } else {
        // At goal position
        if current.abs() > GRIP_CURRENT_THRESHOLD {
            GripperState::Gripping
        } else {
            GripperState::Idle
        }
    }
}

// ---------------------------------------------------------------------------
// ROS 2 Node
// ---------------------------------------------------------------------------
fn run() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("gripper_driver")?;

    // ---- Parameters -------------------------------------------------------
    let port_name: Arc<str> = node
        .declare_parameter("port_name")
        .default(Arc::from("/dev/ttyUSB1"))
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

    let motor_id: i64 = node
        .declare_parameter("motor_id")
        .default(10_i64)
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();

    // ---- Shared command ---------------------------------------------------
    let shared_cmd: Arc<Mutex<Option<GripperCommand>>> = Arc::new(Mutex::new(None));

    // ---- Subscriber: /gripper_cmd (custom_interfaces/GripperCommand) ------
    let shared_sub = Arc::clone(&shared_cmd);
    let _sub = node.create_subscription::<GripperCommandMsg, _>(
        "/gripper_cmd",
        move |msg: GripperCommandMsg| {
            let mut guard = match shared_sub.lock() {
                Ok(g) => g,
                Err(p) => p.into_inner(),
            };
            *guard = Some(GripperCommand {
                position: clamp_dxl_position(msg.position),
                max_current: msg.max_current,
            });
        },
    )?;

    // ---- Publisher: /gripper_status (custom_interfaces/GripperStatus) -----
    let status_pub: Publisher<GripperStatusMsg> = node.create_publisher("/gripper_status")?;

    // ---- Hardware thread --------------------------------------------------
    let shared_hw = Arc::clone(&shared_cmd);
    let status_pub_clone = status_pub.clone();
    let shutdown_flag = Arc::new(AtomicBool::new(false));
    let shutdown_clone = Arc::clone(&shutdown_flag);
    let estop_flag = Arc::new(AtomicBool::new(false));
    let estop_clone = Arc::clone(&estop_flag);

    let hw_thread = std::thread::spawn(move || {
        let mut driver = match GripperDriver::new(&port_name, baud_rate as u32, motor_id as u8) {
            Ok(d) => d,
            Err(e) => {
                eprintln!("🔥 Gripper init failed: {e:#}");
                return;
            }
        };

        if let Err(e) = driver.init_current_position_mode() {
            eprintln!("🔥 Mode init failed: {e:#}");
            return;
        }

        let mut current_goal: u32 = 2048; // center position
        let mut consecutive_overloads: u32 = 0;
        const MAX_OVERLOAD_COUNT: u32 = 3; // 3 consecutive reads (~150ms) before releasing
        println!("✅ Gripper hardware ready, polling started.");

        while !shutdown_clone.load(Ordering::Relaxed) {
            // E-Stop check
            if estop_clone.load(Ordering::Relaxed) {
                eprintln!("🛑 gripper_driver: E-Stop received — disabling torque!");
                driver.torque_off();
                while estop_clone.load(Ordering::Relaxed) && !shutdown_clone.load(Ordering::Relaxed) {
                    std::thread::sleep(Duration::from_millis(100));
                }
                if shutdown_clone.load(Ordering::Relaxed) {
                    break;
                }

                eprintln!("✅ gripper_driver: E-Stop cleared — reinitializing current-position mode");
                if let Err(e) = driver.init_current_position_mode() {
                    eprintln!("🔥 gripper_driver: recovery failed during mode init: {e:#}");
                    break;
                }

                // Re-sync internal goal with current position and discard stale commands.
                if let Ok(pos) = driver.read_present_position() {
                    current_goal = clamp_dxl_position(pos);
                }
                {
                    let mut guard = match shared_hw.lock() {
                        Ok(g) => g,
                        Err(p) => p.into_inner(),
                    };
                    *guard = None;
                }
                consecutive_overloads = 0;
                eprintln!("✅ gripper_driver: recovery complete, loop resumed");
                continue;
            }
            // 1. Check for new commands
            {
                let mut guard = match shared_hw.lock() {
                    Ok(g) => g,
                    Err(p) => p.into_inner(),
                };
                if let Some(cmd) = guard.take() {
                    current_goal = cmd.position;
                    if let Err(e) = driver.set_goal(cmd.position, cmd.max_current) {
                        eprintln!("⚠️  Set goal error: {e}");
                    }
                }
            }

            // 2. Read status
            let position = driver.read_present_position().unwrap_or(0);
            let current = driver.read_present_current().unwrap_or(0);
            let temperature = driver.read_temperature().unwrap_or(0);

            let state = detect_state(current_goal, position, current);

            // 3. Publish status as GripperStatus message
            let mut msg = GripperStatusMsg::default();
            msg.state = state.as_u8();
            msg.position = position;
            msg.current = current;
            msg.temperature = temperature;
            let _ = status_pub_clone.publish(&msg);

            // 4. Overload protection
            if state == GripperState::Overload {
                consecutive_overloads += 1;
                eprintln!("🔥 OVERLOAD detected ({}mA, {}/{})", current, consecutive_overloads, MAX_OVERLOAD_COUNT);
                if consecutive_overloads >= MAX_OVERLOAD_COUNT {
                    eprintln!("🛑 Sustained overload — writing current position as goal to release");
                    // Write current position as goal to stop the motor from pushing further
                    if let Ok(pos) = driver.read_present_position() {
                        current_goal = clamp_dxl_position(pos);
                        let _ = driver.set_goal(current_goal, 200); // reduced current limit
                    }
                    consecutive_overloads = 0;
                }
            } else {
                consecutive_overloads = 0;
            }

            std::thread::sleep(Duration::from_millis(50)); // 20Hz
        }

        // ── Graceful shutdown: disable torque ──
        println!("🛑 gripper_driver: shutting down — disabling torque");
        driver.torque_off();
        println!("✅ gripper_driver: torque disabled, hardware thread exited cleanly");
    });

    // ---- Subscriber: /emergency_stop (std_msgs/Bool) --------------------
    // QoS must match joy_controller publisher: transient_local + reliable
    let estop_sub_flag = Arc::clone(&estop_flag);
    let _estop_sub = node.create_subscription::<std_msgs::msg::Bool, _>(
        "/emergency_stop".reliable().transient_local().keep_last(1),
        move |msg: std_msgs::msg::Bool| {
            estop_sub_flag.store(msg.data, Ordering::Relaxed);
        },
    )?;

    println!("🚀 gripper_driver started");
    println!("   Subscribe: /gripper_cmd    (custom_interfaces/GripperCommand)");
    println!("   Subscribe: /emergency_stop (std_msgs/Bool)");
    println!("   Publish:   /gripper_status (custom_interfaces/GripperStatus)");

    executor.spin(SpinOptions::default()).first_error()?;

    // ── Signal hardware thread to stop and wait for clean shutdown ──
    shutdown_flag.store(true, Ordering::Relaxed);
    if hw_thread.join().is_err() {
        eprintln!("⚠️  gripper_driver: hardware thread panicked during shutdown");
    }

    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("🔥 gripper_driver fatal: {e:#}");
        std::process::exit(1);
    }
}
