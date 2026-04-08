/// arm_driver (Unified): Dynamixel control for 6-DOF arm + Gripper
mod driver;

use anyhow::Result;
use custom_interfaces::msg::{GripperCommand, GripperStatus};
use driver::ArmDynamixelDriver;
use rclrs::{
    Context, CreateBasicExecutor, IntoPrimitiveOptions, Publisher, RclrsErrorFilter, SpinOptions,
};
use sensor_msgs::msg::JointState;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use std_msgs;

const HW_LOOP_HZ: u64 = 50;

struct HwCommand {
    arm_positions: Option<Vec<f64>>,
    gripper_position: Option<f64>,
}

fn now_stamp() -> builtin_interfaces::msg::Time {
    let dur = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    builtin_interfaces::msg::Time {
        sec: dur.as_secs() as i32,
        nanosec: dur.subsec_nanos(),
    }
}

struct HardwareThreadParams {
    port_name: String,
    baud_rate: u32,
    profile_velocity: u32,
    arm_ids: Vec<u8>,
    gripper_ids: Vec<u8>,
    arm_directions: Vec<f64>,
    gripper_directions: Vec<f64>,
    gripper_max_current: u16,
    arm_offsets: Vec<f64>,
    gripper_offsets: Vec<f64>,
    rx_cmd: Receiver<HwCommand>,
    joint_state_pub: Publisher<JointState>,
    gripper_status_pub: Publisher<GripperStatus>,
    arm_joint_names: Vec<String>,
    shutdown_flag: Arc<AtomicBool>,
    estop_flag: Arc<AtomicBool>,
}

fn hardware_thread(params: HardwareThreadParams) {
    let mut driver = match ArmDynamixelDriver::new(
        &params.port_name,
        params.baud_rate,
        params.arm_ids.clone(),
        params.gripper_ids.clone(),
        params.arm_directions.clone(),
        params.gripper_directions.clone(),
        params.arm_offsets.clone(),
        params.gripper_offsets.clone(),
    ) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("🔥 arm_gripper_driver: init failed: {e:#}");
            // プロセスごと終了してrespawnを確実にトリガーする。
            // shutdown_flagをセットするだけではexecutorが止まらないため。
            std::process::exit(1);
        }
    };

    if let Err(e) = driver.init_motors(params.profile_velocity, params.gripper_max_current) {
        eprintln!("🔥 arm_gripper_driver: motor init failed: {e:#}");
        std::process::exit(1);
    }

    let loop_period = Duration::from_millis(1000 / HW_LOOP_HZ);
    // 温度チェックは HW_LOOP_HZ/5 Hz (= 10Hz) で実施
    const TEMP_CHECK_INTERVAL: u64 = HW_LOOP_HZ / 10;
    let mut loop_count: u64 = 0;
    let mut estop_was_active = false;

    while !params.shutdown_flag.load(Ordering::Relaxed) {
        let loop_start = Instant::now();
        let estop_now = params.estop_flag.load(Ordering::Relaxed);

        if estop_now {
            if !estop_was_active {
                // E-Stop 立ち上がりエッジ: トルクOFF
                driver.emergency_stop();
                estop_was_active = true;
                eprintln!("🛑 arm_driver: E-Stop active — torque disabled");
            }
            // E-Stop 中はコマンド受信のみ drain してモータには送らない
            while params.rx_cmd.try_recv().is_ok() {}
            let elapsed = loop_start.elapsed();
            if elapsed < loop_period {
                thread::sleep(loop_period - elapsed);
            }
            continue;
        }

        if estop_was_active {
            // E-Stop 解除: 現在位置をゴールに再設定してからトルクON
            eprintln!("✅ arm_driver: E-Stop cleared — re-enabling torque at current position");
            if let Err(e) = driver.init_motors(params.profile_velocity, params.gripper_max_current)
            {
                eprintln!("🔥 arm_driver: re-init after E-Stop failed: {e:#}");
                std::process::exit(1);
            }
            estop_was_active = false;
        }

        // コマンド処理
        while let Ok(cmd) = params.rx_cmd.try_recv() {
            if let Some(p) = cmd.arm_positions {
                let _ = driver.write_arm_positions(&p);
            }
            if let Some(p) = cmd.gripper_position {
                let _ = driver.write_gripper_position(p);
            }
        }

        // joint_states publish
        if let Ok(p) = driver.read_arm_positions() {
            let mut msg = JointState::default();
            msg.header.stamp = now_stamp();
            msg.name = params.arm_joint_names.clone();
            msg.position = p;
            let _ = params.joint_state_pub.publish(&msg);
        }

        // gripper_status publish
        if let Ok(s) = driver.read_gripper_status() {
            let msg = GripperStatus {
                position: s.position_rad as i32,
                current: s.current_a as i16,
                temperature: s.temperature_c,
                ..Default::default()
            };
            let _ = params.gripper_status_pub.publish(&msg);
        }

        // 温度監視 (10Hz)
        loop_count += 1;
        if loop_count % TEMP_CHECK_INTERVAL == 0 {
            let (warnings, critical) = driver.check_temperatures();
            for (id, temp) in &warnings {
                eprintln!("⚠️  arm_driver: Dynamixel ID {id} temperature warning: {temp}°C");
            }
            if !critical.is_empty() {
                for (id, temp) in &critical {
                    eprintln!("🔥 arm_driver: Dynamixel ID {id} CRITICAL temperature: {temp}°C — disabling torque");
                }
                driver.torque_off_all();
                params.estop_flag.store(true, Ordering::Relaxed);
            }
        }

        let elapsed = loop_start.elapsed();
        if elapsed < loop_period {
            thread::sleep(loop_period - elapsed);
        }
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
    let port_name_arc: Arc<str> = node
        .declare_parameter("port_name")
        .default(Arc::from("/dev/ttyUSB0"))
        .mandatory()?
        .get();
    let port_name: String = port_name_arc.to_string();
    let baud_rate: i64 = node
        .declare_parameter("baud_rate")
        .default(1000000_i64)
        .mandatory()?
        .get();
    let arm_joints_arr: Arc<[Arc<str>]> = node
        .declare_parameter("arm_joints")
        .default(Arc::from(
            vec![
                Arc::from("arm_joint1"),
                Arc::from("arm_joint2"),
                Arc::from("arm_joint3"),
                Arc::from("arm_joint4"),
                Arc::from("arm_joint5"),
                Arc::from("arm_joint6"),
            ]
            .into_boxed_slice(),
        ))
        .mandatory()?
        .get();
    let arm_joints: Vec<String> = arm_joints_arr.iter().map(|s| s.to_string()).collect();
    let arm_ids_arr: Arc<[i64]> = node
        .declare_parameter("arm_ids")
        .default(Arc::from(
            vec![1_i64, 2_i64, 3_i64, 4_i64, 5_i64, 6_i64].into_boxed_slice(),
        ))
        .mandatory()?
        .get();
    let arm_ids: Vec<u8> = arm_ids_arr.iter().copied().map(|id| id as u8).collect();
    let gripper_ids_arr: Arc<[i64]> = node
        .declare_parameter("gripper_ids")
        .default(Arc::from(vec![27_i64, 28_i64].into_boxed_slice()))
        .mandatory()?
        .get();
    let gripper_ids: Vec<u8> = gripper_ids_arr.iter().copied().map(|id| id as u8).collect();
    let arm_directions_arr: Arc<[f64]> = node
        .declare_parameter("arm_directions")
        .default(Arc::from(vec![1.0_f64; 6].into_boxed_slice()))
        .mandatory()?
        .get();
    let arm_directions: Vec<f64> = arm_directions_arr.to_vec();
    let gripper_directions_arr: Arc<[f64]> = node
        .declare_parameter("gripper_directions")
        .default(Arc::from(vec![1.0_f64, -1.0_f64].into_boxed_slice()))
        .mandatory()?
        .get();
    let gripper_directions: Vec<f64> = gripper_directions_arr.to_vec();
    let gripper_max_current: i64 = node
        .declare_parameter("gripper_max_current")
        .default(500_i64)
        .mandatory()?
        .get();
    let profile_velocity: i64 = node
        .declare_parameter("profile_velocity")
        .default(100_i64)
        .mandatory()?
        .get();
    // per-joint ゼロ点オフセット [rad]: physical_rad = urdf_rad + offset
    // Dynamixel tick 2048 が URDF ゼロと一致しない場合に設定する。
    // arm_gripper_driver.yaml の arm_offsets: [j1, j2, j3, j4, j5, j6] で指定。
    let arm_offsets_arr: Arc<[f64]> = node
        .declare_parameter("arm_offsets")
        .default(Arc::from(vec![0.0_f64; 6].into_boxed_slice()))
        .mandatory()?
        .get();
    let arm_offsets: Vec<f64> = arm_offsets_arr.to_vec();
    let gripper_offsets_arr: Arc<[f64]> = node
        .declare_parameter("gripper_offsets")
        .default(Arc::from(vec![0.0_f64; 2].into_boxed_slice()))
        .mandatory()?
        .get();
    let gripper_offsets: Vec<f64> = gripper_offsets_arr.to_vec();

    let joint_state_pub = node.create_publisher::<JointState>("/joint_states")?;
    let gripper_status_pub = node.create_publisher::<GripperStatus>("/gripper_status")?;
    let (tx_cmd, rx_cmd) = channel::<HwCommand>();
    let shutdown_flag = Arc::new(AtomicBool::new(false));
    let estop_flag = Arc::new(AtomicBool::new(false));

    let tx_arm = tx_cmd.clone();
    let arm_names = arm_joints.clone();
    let _arm_sub = node.create_subscription::<JointState, _>(
        "/arm_joint_commands",
        move |msg: JointState| {
            let mut map = HashMap::new();
            for (i, name) in msg.name.iter().enumerate() {
                if i < msg.position.len() {
                    map.insert(name.clone(), msg.position[i]);
                }
            }
            let p: Vec<f64> = arm_names
                .iter()
                .filter_map(|n| map.get(n).copied())
                .collect();
            if p.len() == arm_names.len() {
                let _ = tx_arm.send(HwCommand {
                    arm_positions: Some(p),
                    gripper_position: None,
                });
            }
        },
    )?;

    let tx_gripper = tx_cmd.clone();
    let _gripper_sub = node.create_subscription::<GripperCommand, _>(
        "/gripper_cmd",
        move |msg: GripperCommand| {
            let _ = tx_gripper.send(HwCommand {
                arm_positions: None,
                gripper_position: Some(msg.position as f64),
            });
        },
    )?;

    // /emergency_stop subscriber (transient_local: 再起動直後も latched 値を受信)
    let estop_sub_flag = Arc::clone(&estop_flag);
    let _estop_sub = node.create_subscription::<std_msgs::msg::Bool, _>(
        "/emergency_stop".reliable().transient_local().keep_last(1),
        move |msg: std_msgs::msg::Bool| {
            estop_sub_flag.store(msg.data, Ordering::Relaxed);
        },
    )?;

    let shutdown_c = Arc::clone(&shutdown_flag);
    let estop_c = Arc::clone(&estop_flag);
    let hw_thread = thread::spawn(move || {
        hardware_thread(HardwareThreadParams {
            port_name,
            baud_rate: baud_rate as u32,
            profile_velocity: profile_velocity as u32,
            arm_ids,
            gripper_ids,
            arm_directions,
            gripper_directions,
            gripper_max_current: gripper_max_current as u16,
            arm_offsets,
            gripper_offsets,
            rx_cmd,
            joint_state_pub,
            gripper_status_pub,
            arm_joint_names: arm_joints,
            shutdown_flag: shutdown_c,
            estop_flag: estop_c,
        });
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
