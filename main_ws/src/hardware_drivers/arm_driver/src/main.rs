/// arm_driver (Unified): Dynamixel control for 6-DOF arm + Gripper
mod driver;

use anyhow::{Context as AnyhowContext, Result};
use custom_interfaces::msg::{GripperCommand, GripperStatus};
use driver::ArmDynamixelDriver;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions};
use sensor_msgs::msg::JointState;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

const HW_LOOP_HZ: u64 = 50;

struct ArmCommand {
    arm_positions: Option<Vec<f64>>,
    gripper_position: Option<f64>,
}

struct HardwareThreadParams {
    port_name: String,
    baud_rate: u32,
    arm_ids: Vec<u8>,
    gripper_ids: Vec<u8>,
    arm_directions: Vec<f64>,
    gripper_directions: Vec<f64>,
    arm_gear_ratios: Vec<f64>,
    arm_offsets: Vec<f64>,
    gripper_offsets: Vec<f64>,
    profile_velocity: u32,
    gripper_max_current: u16,
    rx_cmd: std::sync::mpsc::Receiver<ArmCommand>,
    tx_status: std::sync::mpsc::SyncSender<JointState>,
    tx_gripper: std::sync::mpsc::SyncSender<GripperStatus>,
    shutdown_flag: Arc<AtomicBool>,
    estop_flag: Arc<AtomicBool>,
}

fn hardware_thread(params: HardwareThreadParams) {
    let loop_period = Duration::from_millis(1000 / HW_LOOP_HZ);
    let mut estop_was_active = false;

    'outer: loop {
        if params.shutdown_flag.load(Ordering::Relaxed) {
            break 'outer;
        }

        if !std::path::Path::new(&params.port_name).exists() {
            eprintln!("🔍 [Arm] Port {} NOT FOUND. Check USB connection.", params.port_name);
            thread::sleep(Duration::from_secs(2));
            continue 'outer;
        }

        let mut driver = match ArmDynamixelDriver::new(
            &params.port_name,
            params.baud_rate,
            params.arm_ids.clone(),
            params.gripper_ids.clone(),
            params.arm_directions.clone(),
            params.gripper_directions.clone(),
            params.arm_gear_ratios.clone(),
            params.arm_offsets.clone(),
            params.gripper_offsets.clone(),
        ) {
            Ok(d) => d,
            Err(e) => {
                eprintln!("❌ [Arm] Port exists but failed to open: {e:#}");
                thread::sleep(Duration::from_secs(2));
                continue 'outer;
            }
        };

        if let Err(e) = driver.init_motors(params.profile_velocity, params.gripper_max_current) {
            eprintln!("❓ [Arm] Motor init failed: {e:#}");
            thread::sleep(Duration::from_secs(2));
            continue 'outer;
        }

        println!("✅ [Arm] Motors Ready!");

        let mut loop_count: u64 = 0;
        let mut grasp_over_counts = vec![0; params.gripper_ids.len()];
        let mut is_holding = vec![false; params.gripper_ids.len()];
        let mut last_target_rad: Option<f64> = None;
        let grasp_threshold_ma = 450.0;
        let consecutive_limit = 5;

        while !params.shutdown_flag.load(Ordering::Relaxed) {
            let loop_start = Instant::now();
            let estop_now = params.estop_flag.load(Ordering::Relaxed);

            if estop_now {
                if !estop_was_active {
                    driver.emergency_stop();
                    estop_was_active = true;
                    eprintln!("🛑 arm_driver: E-Stop active");
                }
                while params.rx_cmd.try_recv().is_ok() {}
                let elapsed = loop_start.elapsed();
                if elapsed < loop_period { thread::sleep(loop_period - elapsed); }
                continue;
            }

            if estop_was_active {
                eprintln!("✅ arm_driver: E-Stop cleared");
                if let Err(e) = driver.init_motors(params.profile_velocity, params.gripper_max_current) {
                    eprintln!("🔥 arm_driver: re-init failed: {e:#}");
                    params.estop_flag.store(true, Ordering::Relaxed);
                    break;
                }
                estop_was_active = false;
            }

            while let Ok(cmd) = params.rx_cmd.try_recv() {
                if let Some(p) = cmd.arm_positions {
                    let _ = driver.write_arm_positions(&p);
                }
                if let Some(p) = cmd.gripper_position {
                    last_target_rad = Some(p);
                    for i in 0..params.gripper_ids.len() {
                        grasp_over_counts[i] = 0;
                        is_holding[i] = false;
                    }
                    let _ = driver.write_gripper_position(p);
                }
            }

            if let Ok(arm_pos) = driver.read_arm_positions() {
                let mut js = JointState::default();
                js.name = vec![
                    "arm_joint1".to_string(), "arm_joint2".to_string(), "arm_joint3".to_string(),
                    "arm_joint4".to_string(), "arm_joint5".to_string(), "arm_joint6".to_string(),
                ];
                js.position = arm_pos;
                let _ = params.tx_status.try_send(js);
            }

            if let Ok(g_stats) = driver.read_gripper_statuses() {
                if let Some(s) = g_stats.first() {
                    let mut gs = GripperStatus::default();
                    gs.position = s.position_ticks;
                    gs.current = s.current_a as i16;
                    gs.temperature = s.temperature_c;
                    let _ = params.tx_gripper.try_send(gs);

                    if last_target_rad.is_some() && !is_holding[0] {
                        if s.current_a.abs() > grasp_threshold_ma / 1000.0 {
                            grasp_over_counts[0] += 1;
                            if grasp_over_counts[0] >= consecutive_limit {
                                let _ = driver.write_gripper_position(s.position_rad);
                                is_holding[0] = true;
                                println!("✊ Grasp detected (HW)");
                            }
                        } else {
                            grasp_over_counts[0] = 0;
                        }
                    }
                }
            }

            loop_count += 1;
            let elapsed = loop_start.elapsed();
            if elapsed < loop_period { thread::sleep(loop_period - elapsed); }
        }
    }
}

fn run() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("arm_driver")?;

    let port_name: Arc<str> = node.declare_parameter("port_name").default(Arc::from("/dev/dynamixel_arm")).mandatory()?.get();
    let baud_rate: i64 = node.declare_parameter("baud_rate").default(1000000).mandatory()?.get();
    let arm_ids_raw: Arc<[i64]> = node.declare_parameter("arm_ids").default(Arc::from([21_i64, 22, 23, 24, 25, 26].as_slice())).mandatory()?.get();
    let gripper_ids_raw: Arc<[i64]> = node.declare_parameter("gripper_ids").default(Arc::from([27_i64, 28].as_slice())).mandatory()?.get();
    let arm_dir_raw: Arc<[f64]> = node.declare_parameter("arm_directions").default(Arc::from([1.0; 6].as_slice())).mandatory()?.get();
    let gripper_dir_raw: Arc<[f64]> = node.declare_parameter("gripper_directions").default(Arc::from([-1.0_f64, 1.0].as_slice())).mandatory()?.get();
    let arm_gear_raw: Arc<[f64]> = node.declare_parameter("arm_gear_ratios").default(Arc::from([1.0; 6].as_slice())).mandatory()?.get();
    let arm_off_raw: Arc<[f64]> = node.declare_parameter("arm_offsets").default(Arc::from([0.0; 6].as_slice())).mandatory()?.get();
    let gripper_off_raw: Arc<[f64]> = node.declare_parameter("gripper_offsets").default(Arc::from([0.0; 2].as_slice())).mandatory()?.get();
    let profile_velocity: i64 = node.declare_parameter("profile_velocity").default(100).mandatory()?.get();
    let gripper_max_current: i64 = node.declare_parameter("gripper_max_current").default(600).mandatory()?.get();

    let (tx_cmd, rx_cmd) = std::sync::mpsc::channel();
    let (tx_status, rx_status) = std::sync::mpsc::sync_channel(1);
    let (tx_gripper, rx_gripper) = std::sync::mpsc::sync_channel(1);
    let shutdown_flag = Arc::new(AtomicBool::new(false));
    let estop_flag = Arc::new(AtomicBool::new(false));

    // Matching arm_controller's successful pattern: create_publisher("/topic") without QoS arg
    let js_pub = node.create_publisher::<JointState>("/joint_states")?;
    let gs_pub = node.create_publisher::<GripperStatus>("/gripper_status")?;

    // Matching arm_controller's successful pattern: create_subscription("/topic", callback) without QoS arg
    let _sub_arm = node.create_subscription::<JointState, _>("/arm_joint_commands", {
        let tx = tx_cmd.clone();
        move |msg: JointState| {
            let _ = tx.send(ArmCommand {
                arm_positions: Some(msg.position),
                gripper_position: None,
            });
        }
    })?;

    let _sub_gripper = node.create_subscription::<GripperCommand, _>("/gripper_cmd", {
        let tx = tx_cmd.clone();
        move |msg: GripperCommand| {
            let _ = tx.send(ArmCommand {
                arm_positions: None,
                gripper_position: Some(msg.position as f64 * (0.088 * std::f64::consts::PI / 180.0)),
            });
        }
    })?;

    let params = HardwareThreadParams {
        port_name: port_name.to_string(),
        baud_rate: baud_rate as u32,
        arm_ids: arm_ids_raw.iter().map(|&x| x as u8).collect(),
        gripper_ids: gripper_ids_raw.iter().map(|&x| x as u8).collect(),
        arm_directions: arm_dir_raw.to_vec(),
        gripper_directions: gripper_dir_raw.to_vec(),
        arm_gear_ratios: arm_gear_raw.to_vec(),
        arm_offsets: arm_off_raw.to_vec(),
        gripper_offsets: gripper_off_raw.to_vec(),
        profile_velocity: profile_velocity as u32,
        gripper_max_current: gripper_max_current as u16,
        rx_cmd, tx_status, tx_gripper, shutdown_flag: shutdown_flag.clone(), estop_flag: estop_flag.clone(),
    };

    let hw_thread = thread::spawn(move || hardware_thread(params));

    thread::spawn({
        let context = context.clone();
        move || {
            while context.ok() {
                if let Ok(js) = rx_status.try_recv() { let _ = js_pub.publish(&js); }
                if let Ok(gs) = rx_gripper.try_recv() { let _ = gs_pub.publish(&gs); }
                thread::sleep(Duration::from_millis(5));
            }
        }
    });

    executor.spin(SpinOptions::default()).first_error()?;

    shutdown_flag.store(true, Ordering::Relaxed);
    let _ = hw_thread.join();
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("🔥 fatal: {e:#}");
    }
}
