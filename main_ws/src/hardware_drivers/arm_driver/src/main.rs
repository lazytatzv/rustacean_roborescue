/// arm_driver (Unified): Dynamixel control for 6-DOF arm + Gripper
mod driver;

use anyhow::Result;
use driver::ArmDynamixelDriver;
use rclrs::{Context, CreateBasicExecutor, Publisher, RclrsErrorFilter, SpinOptions};
use sensor_msgs::msg::JointState;
use custom_interfaces::msg::{GripperCommand, GripperStatus};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const HW_LOOP_HZ: u64 = 50;

struct HwCommand {
    arm_positions: Option<Vec<f64>>,
    gripper_position: Option<f64>,
}

fn now_stamp() -> builtin_interfaces::msg::Time {
    let dur = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
    builtin_interfaces::msg::Time { sec: dur.as_secs() as i32, nanosec: dur.subsec_nanos() }
}

fn hardware_thread(
    port_name: String, baud_rate: u32, profile_velocity: u32,
    arm_ids: Vec<u8>, gripper_id: u8, gripper_max_current: u16,
    rx_cmd: Receiver<HwCommand>,
    joint_state_pub: Publisher<JointState>,
    gripper_status_pub: Publisher<GripperStatus>,
    arm_joint_names: Vec<String>,
    shutdown_flag: Arc<AtomicBool>,
    estop_flag: Arc<AtomicBool>,
) {
    let mut driver = match ArmDynamixelDriver::new(&port_name, baud_rate, arm_ids.clone(), gripper_id) {
        Ok(d) => d,
        Err(e) => { eprintln!("🔥 arm_gripper_driver: init failed: {e:#}"); return; }
    };

    if let Err(e) = driver.init_motors(profile_velocity, gripper_max_current) {
        eprintln!("🔥 arm_gripper_driver: motor init failed: {e:#}"); return;
    }

    let loop_period = Duration::from_millis(1000 / HW_LOOP_HZ);
    while !shutdown_flag.load(Ordering::Relaxed) {
        let loop_start = Instant::now();

        if estop_flag.load(Ordering::Relaxed) {
            driver.emergency_stop();
            while !shutdown_flag.load(Ordering::Relaxed) { thread::sleep(Duration::from_millis(100)); }
            break;
        }

        while let Ok(cmd) = rx_cmd.try_recv() {
            if let Some(p) = cmd.arm_positions { let _ = driver.write_arm_positions(&p); }
            if let Some(p) = cmd.gripper_position { let _ = driver.write_gripper_position(p); }
        }

        if let Ok(p) = driver.read_arm_positions() {
            let mut msg = JointState::default();
            msg.header.stamp = now_stamp();
            msg.name = arm_joint_names.clone();
            msg.position = p;
            let _ = joint_state_pub.publish(&msg);
        }

        if let Ok(s) = driver.read_gripper_status() {
            let mut msg = GripperStatus::default();
            msg.position = s.position_rad as i32;
            msg.current = s.current_a as i16;
            msg.temperature = s.temperature_c as u8;
            let _ = gripper_status_pub.publish(&msg);
        }

        let elapsed = loop_start.elapsed();
        if elapsed < loop_period { thread::sleep(loop_period - elapsed); }
    }
    driver.torque_off_all();
}

fn run() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    // ノード名を変更
    let node = executor.create_node("arm_gripper_driver")?;

    // ros2 parametersの宣言と取得
    // arm_gripper_driver.yamlで管理することができる
    let port_name_arc: Arc<str> = node.declare_parameter("port_name").default(Arc::from("/dev/ttyUSB0")).mandatory()?.get();
    let port_name: String = port_name_arc.to_string();
    let baud_rate: i64 = node.declare_parameter("baud_rate").default(1000000_i64).mandatory()?.get();
    let arm_joints_arr: Arc<[Arc<str>]> = node.declare_parameter("arm_joints").default(Arc::from(vec![Arc::from("arm_joint1")].into_boxed_slice())).mandatory()?.get();
    let arm_joints: Vec<String> = arm_joints_arr.iter().map(|s| s.to_string()).collect();
    let arm_ids_arr: Arc<[i64]> = node.declare_parameter("arm_ids").default(Arc::from(vec![21_i64].into_boxed_slice())).mandatory()?.get();
    let arm_ids: Vec<u8> = arm_ids_arr.iter().copied().map(|id| id as u8).collect();
    let gripper_id: i64 = node.declare_parameter("gripper_id").default(10_i64).mandatory()?.get();
    let gripper_max_current: i64 = node.declare_parameter("gripper_max_current").default(500_i64).mandatory()?.get();
    let profile_velocity: i64 = node.declare_parameter("profile_velocity").default(100_i64).mandatory()?.get();

    let joint_state_pub = node.create_publisher::<JointState>("/joint_states")?;
    let gripper_status_pub = node.create_publisher::<GripperStatus>("/gripper_status")?;
    let (tx_cmd, rx_cmd) = channel::<HwCommand>();
    let shutdown_flag = Arc::new(AtomicBool::new(false));
    let estop_flag = Arc::new(AtomicBool::new(false));

    let tx_arm = tx_cmd.clone();
    let arm_names = arm_joints.clone();
    let _arm_sub = node.create_subscription::<JointState, _>("/arm_joint_commands", move |msg: JointState| {
        let mut map = HashMap::new();
        for (i, name) in msg.name.iter().enumerate() { if i < msg.position.len() { map.insert(name.clone(), msg.position[i]); } }
        let p: Vec<f64> = arm_names.iter().filter_map(|n| map.get(n).copied()).collect();
        if p.len() == arm_names.len() { let _ = tx_arm.send(HwCommand { arm_positions: Some(p), gripper_position: None }); }
    })?;

    let tx_gripper = tx_cmd.clone();
    let _gripper_sub = node.create_subscription::<GripperCommand, _>("/gripper_cmd", move |msg: GripperCommand| {
        let _ = tx_gripper.send(HwCommand { arm_positions: None, gripper_position: Some(msg.position as f64) });
    })?;

    let shutdown_c = Arc::clone(&shutdown_flag);
    let estop_c = Arc::clone(&estop_flag);
    let hw_thread = thread::spawn(move || {
        hardware_thread(port_name, baud_rate as u32, profile_velocity as u32, arm_ids, gripper_id as u8, gripper_max_current as u16, rx_cmd, joint_state_pub, gripper_status_pub, arm_joints, shutdown_c, estop_c);
    });

    executor.spin(SpinOptions::default()).first_error()?;
    shutdown_flag.store(true, Ordering::Relaxed);
    let _ = hw_thread.join();
    Ok(())
}

fn main() { if let Err(e) = run() { eprintln!("🔥 fatal: {e:#}"); } }
