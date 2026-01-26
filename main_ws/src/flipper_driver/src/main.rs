mod driver;

use anyhow::{Context as _, Result};
use driver::DynamixelDriver;
use rclrs::{Context, RclrsErrorFilter, SpinOptions};
use rclrs::CreateBasicExecutor; 

use std::sync::{Arc, Mutex};
use std::thread; // ★追加: スレッド用
use std::time::Duration;

// メッセージの型
use sensor_msgs::msg::JointState;

const PORT_NAME: &str = "/dev/ttyUSB0";
const BAUD_RATE: u32 = 1_000_000;
const FLIPPER_IDS: [u8; 4] = [1, 2, 3, 4];

fn main() -> Result<()> {
    // 1. ハードウェア初期化
    let mut driver = DynamixelDriver::new(PORT_NAME, BAUD_RATE, FLIPPER_IDS.to_vec())
        .context("Failed to initialize Dynamixel driver")?;
    driver.init_velocity_mode()?;
    let shared_driver = Arc::new(Mutex::new(driver));

    // 2. ROS 2 ノードの作成
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("flipper_driver")?;

    // 3. Publisher
    // QoS引数なし（エラーログ対策）
    let publisher = node.create_publisher::<JointState>("/joint_states")?;
    let driver_for_pub = shared_driver.clone();

    // 4. 定期実行ループ（タイマーの代わり）
    // ★修正: create_timer が存在しないため、Rust標準スレッドで代用します。
    // これならAPI変更の影響を受けずに確実に動きます。
    let publisher_clone = publisher.clone();
    thread::spawn(move || {
        loop {
            thread::sleep(Duration::from_millis(100)); // 10Hz

            if let Ok(mut drv) = driver_for_pub.lock() {
                if let Ok(positions) = drv.read_positions() {
                    let mut msg = JointState::default();
                    msg.name = vec![
                        "flipper_FL".to_string(), "flipper_FR".to_string(),
                        "flipper_RL".to_string(), "flipper_RR".to_string()
                    ];
                    msg.position = positions.iter().map(|&p| p as f64).collect();
                    
                    // 送信（エラーは無視）
                    let _ = publisher_clone.publish(&msg);
                }
            }
        }
    });

    // 5. Subscriber
    let driver_for_sub = shared_driver.clone();
    let _subscription = node.create_subscription::<JointState, _>(
        "/flipper_commands",
        move |msg: JointState| {
            if let Ok(mut drv) = driver_for_sub.lock() {
                let velocities: Vec<i32> = msg.velocity.iter().map(|&v| v as i32).collect();
                if !velocities.is_empty() {
                    let _ = drv.write_velocities(&velocities);
                }
            }
        },
    )?;

    println!("Node is running (Thread-based publishing)...");

    // 6. Spin (Subscriberの受信を処理)
    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())
}
