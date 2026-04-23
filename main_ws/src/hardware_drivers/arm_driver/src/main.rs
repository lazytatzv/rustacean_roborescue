/// arm_driver (Unified): Dynamixel control for 6-DOF arm + Gripper
mod driver;

use anyhow::Result;
use custom_interfaces::msg::{GripperCommand, GripperStatus};
use driver::ArmDynamixelDriver;
use rclrs::{
    Context, CreateBasicExecutor, Publisher, RclrsErrorFilter, SpinOptions,
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
    gripper_positions: Option<Vec<f64>>,
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
    arm_offsets: Vec<f64>,
    gripper_offsets: Vec<f64>,
    arm_reductions: Vec<f64>,
    rx_cmd: Receiver<HwCommand>,
    joint_state_pub: Publisher<JointState>,
    gripper_status_pub: Publisher<GripperStatus>,
    arm_joint_names: Vec<String>,
    gripper_joint_names: Vec<String>,
    use_grasp_detection: bool,
    grasp_effort_threshold: f64,
    grasp_consecutive_count: u32,
    gripper_max_current: u16,
    shutdown_flag: Arc<AtomicBool>,
    torque_off_flag: Arc<AtomicBool>,
    reboot_flag: Arc<AtomicBool>,
    /// 起動時のキャリブ用ホームポジション [URDF rad]。
    /// 起動時はアームが必ずここにいることを前提に goal_position を設定する。
    home_positions: Vec<f64>,
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
        params.arm_reductions.clone(),
    ) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("🔥 arm_gripper_driver: init failed: {e:#}");
            // プロセスごと終了してrespawnを確実にトリガーする。
            // shutdown_flagをセットするだけではexecutorが止まらないため。
            std::process::exit(1);
        }
    };

    if let Err(e) = driver.init_motors(params.profile_velocity, params.gripper_max_current, &params.home_positions) {
        eprintln!("🔥 arm_gripper_driver: motor init failed: {e:#}");
        std::process::exit(1);
    }

    let loop_period = Duration::from_millis(1000 / HW_LOOP_HZ);
    const TEMP_CHECK_INTERVAL: u64 = HW_LOOP_HZ / 10;
    let mut loop_count: u64 = 0;

    // Grasp detection state
    let mut over_count = 0;
    let mut is_holding = false;
    let mut gripper_closing = false;
    // 最後に送ったグリッパー指令（開閉方向の判定用）。初期値=閉じきり
    let mut last_gripper_cmd: Vec<f64> = vec![0.0; params.gripper_ids.len()];

    while !params.shutdown_flag.load(Ordering::Relaxed) {
        let loop_start = Instant::now();

        // トルクOFF / リブートリクエスト処理
        if params.torque_off_flag.swap(false, Ordering::Relaxed) {
            driver.torque_off_all();
        }
        if params.reboot_flag.swap(false, Ordering::Relaxed) {
            driver.reboot_all();
            if let Err(e) = driver.init_motors(params.profile_velocity, params.gripper_max_current, &params.home_positions) {
                eprintln!("⚠️  arm_driver: re-init after reboot failed: {e:#}");
            }
        }

        // コマンド処理
        while let Ok(cmd) = params.rx_cmd.try_recv() {
            if let Some(p) = cmd.arm_positions {
                if let Err(e) = driver.write_arm_positions(&p) {
                    eprintln!("⚠️  arm_driver: write_arm_positions failed: {e:?}");
                }
            }
            if let Some(p) = cmd.gripper_positions {
                is_holding = false;
                over_count = 0;

                // ゼロ（閉じきり）からの距離で開閉方向を判定。
                // 新指令がゼロへ近づく = 閉じる → 把持検出を有効化
                // 新指令がゼロから離れる = 開く → 把持検出を無効化
                let dist_new: f64 = p.iter().map(|&x| x * x).sum::<f64>();
                let dist_cur: f64 = last_gripper_cmd.iter().map(|&x| x * x).sum::<f64>();
                gripper_closing = dist_new < dist_cur;
                last_gripper_cmd = p.clone();

                if let Err(e) = driver.write_gripper_positions(&p) {
                    eprintln!("⚠️  arm_driver: write_gripper_positions failed: {e:?}");
                }
            }
        }

        // joint_states & gripper_status publish
        let mut joint_state_msg = JointState::default();
        joint_state_msg.header.stamp = now_stamp();
        
        // Arm positions
        if let Ok(p) = driver.read_arm_positions() {
            joint_state_msg.name.extend(params.arm_joint_names.clone());
            joint_state_msg.position.extend(p);
        }

        // Gripper statuses
        match driver.read_gripper_statuses() {
            Ok(statuses) => {
                // JointState にグリッパー情報を追加
                for (i, status) in statuses.iter().enumerate() {
                    if i < params.gripper_joint_names.len() {
                        joint_state_msg.name.push(params.gripper_joint_names[i].clone());
                        joint_state_msg.position.push(status.position_rad);
                        joint_state_msg.velocity.push(status.velocity_rad_s);
                        joint_state_msg.effort.push(status.current_a);
                    }
                }

                // GripperStatus (最初のモーター分) の publish
                if let Some(s) = statuses.first() {
                    let msg = GripperStatus {
                        position: driver::rad_to_ticks(s.position_rad) as i32,
                        current: (s.current_a * 1000.0) as i16,
                        temperature: s.temperature_c,
                        ..Default::default()
                    };
                    let _ = params.gripper_status_pub.publish(&msg);
                }

                // 保持検出 (Grasp Detection) ロジック (C++版に準拠: いずれかが閾値超えで全体ホールド)
                if params.use_grasp_detection && gripper_closing && !is_holding {
                    let any_over = statuses.iter().any(|s| s.current_a.abs() > params.grasp_effort_threshold);
                    
                    if any_over {
                        over_count += 1;
                    } else {
                        over_count = 0;
                    }

                    if over_count >= params.grasp_consecutive_count {
                        eprintln!("✊ arm_driver: Grasp detected (any motor over threshold). Holding all gripper joints.");
                        if let Err(e) = driver.write_gripper_hold(&statuses) {
                            eprintln!("⚠️  arm_driver: write_gripper_hold failed: {e:?}");
                        }
                        is_holding = true;
                        gripper_closing = false;
                        over_count = 0;
                    }
                }

                // 詳細ログ (1秒毎)
                if loop_count % HW_LOOP_HZ == 0 {
                    for (i, status) in statuses.iter().enumerate() {
                        let id = params.gripper_ids[i];
                        println!("📊 arm_driver: Gripper ID {id}: pos={:.3} rad, current={:.3} A, temp={} C",
                                 status.position_rad, status.current_a, status.temperature_c);
                    }
                }
            }
            Err(e) => {
                if loop_count % 50 == 0 {
                    eprintln!("⚠️  arm_driver: read_gripper_statuses failed: {e:?}");
                }
            }
        }
        
        let _ = params.joint_state_pub.publish(&joint_state_msg);

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
    
    let gripper_joints_arr: Arc<[Arc<str>]> = node
        .declare_parameter("gripper_joints")
        .default(Arc::from(
            vec![
                Arc::from("gripper_joint7"),
                Arc::from("gripper_joint8"),
            ]
            .into_boxed_slice(),
        ))
        .mandatory()?
        .get();
    let gripper_joints: Vec<String> = gripper_joints_arr.iter().map(|s| s.to_string()).collect();

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
    let arm_reductions_arr: Arc<[f64]> = node
        .declare_parameter("arm_reductions")
        .default(Arc::from(vec![1.0_f64; 6].into_boxed_slice()))
        .mandatory()?
        .get();
    let arm_reductions: Vec<f64> = arm_reductions_arr.to_vec();

    let gripper_max_current: i64 = node
        .declare_parameter("gripper_max_current")
        .default(600_i64)
        .mandatory()?
        .get();

    // Grasp Detection Parameters
    let use_grasp_detection = node.declare_parameter("use_grasp_detection").default(true).mandatory()?.get();
    let grasp_effort_threshold = node.declare_parameter("grasp_effort_threshold").default(0.45).mandatory()?.get();
    let grasp_consecutive_count = node.declare_parameter("grasp_consecutive_count").default(3_i64).mandatory()?.get();

    // 起動時キャリブ用ホームポジション [URDF rad]
    // 起動時はアームが必ずこの位置にいることを前提に goal_position を設定する。
    let home_positions_arr: Arc<[f64]> = node
        .declare_parameter("arm_home_position")
        .default(Arc::from(vec![0.0_f64; 6].into_boxed_slice()))
        .mandatory()?
        .get();
    let home_positions: Vec<f64> = home_positions_arr.to_vec();
    println!("✅ arm_driver: Home position: {:?}", home_positions);

    let joint_state_pub = node.create_publisher::<JointState>("/joint_states")?;
    let gripper_status_pub = node.create_publisher::<GripperStatus>("/gripper_status")?;
    let (tx_cmd, rx_cmd) = channel::<HwCommand>();
    let shutdown_flag   = Arc::new(AtomicBool::new(false));
    let torque_off_flag = Arc::new(AtomicBool::new(false));
    let reboot_flag     = Arc::new(AtomicBool::new(false));

    let tof_sub_flag = Arc::clone(&torque_off_flag);
    let _torque_off_sub = node.create_subscription::<std_msgs::msg::Empty, _>(
        "/arm_torque_off",
        move |_: std_msgs::msg::Empty| { tof_sub_flag.store(true, Ordering::Relaxed); },
    )?;

    let rb_sub_flag = Arc::clone(&reboot_flag);
    let _reboot_sub = node.create_subscription::<std_msgs::msg::Empty, _>(
        "/arm_reboot",
        move |_: std_msgs::msg::Empty| { rb_sub_flag.store(true, Ordering::Relaxed); },
    )?;

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
                    gripper_positions: None,
                });
            }
        },
    )?;

    let tx_gripper = tx_cmd.clone();
    let _gripper_sub = node.create_subscription::<GripperCommand, _>(
        "/gripper_cmd",
        move |msg: GripperCommand| {
            // 単一の position (ticks) を受け取り、全グリッパーに適用
            let rad = driver::ticks_to_rad(msg.position as u32);
            let _ = tx_gripper.send(HwCommand {
                arm_positions: None,
                gripper_positions: Some(vec![rad, rad]), // IDsの数に合わせる必要があるが、とりあえず2つ
            });
        },
    )?;
    
    let tx_gripper_multi = tx_cmd.clone();
    let _gripper_multi_sub = node.create_subscription::<std_msgs::msg::Float64MultiArray, _>(
        "/gripper_controller/commands",
        move |msg: std_msgs::msg::Float64MultiArray| {
            let _ = tx_gripper_multi.send(HwCommand {
                arm_positions: None,
                gripper_positions: Some(msg.data.clone()),
            });
        },
    )?;

    let shutdown_c    = Arc::clone(&shutdown_flag);
    let torque_off_c  = Arc::clone(&torque_off_flag);
    let reboot_c      = Arc::clone(&reboot_flag);
    let gripper_names = gripper_joints.clone();
    let hw_thread = thread::spawn(move || {
        hardware_thread(HardwareThreadParams {
            port_name,
            baud_rate: baud_rate as u32,
            profile_velocity: profile_velocity as u32,
            arm_ids,
            gripper_ids,
            arm_directions,
            gripper_directions,
            arm_offsets,
            gripper_offsets,
            arm_reductions,
            rx_cmd,
            joint_state_pub,
            gripper_status_pub,
            arm_joint_names: arm_joints,
            gripper_joint_names: gripper_names,
            use_grasp_detection,
            grasp_effort_threshold,
            grasp_consecutive_count: grasp_consecutive_count as u32,
            gripper_max_current: gripper_max_current as u16,
            shutdown_flag:   shutdown_c,
            torque_off_flag: torque_off_c,
            reboot_flag:     reboot_c,
            home_positions,
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
