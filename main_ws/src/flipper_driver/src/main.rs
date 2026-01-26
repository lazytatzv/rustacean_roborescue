mod driver;

use std::sync::{Arc, Mutex};
use std::time::Duration;
use anyhow::{Error, Result};
use driver::DynamixelDriver;

// rclrs 0.7.0 API
use rclrs::QOS_PROFILE_DEFAULT;

const PORT_NAME: &str = "/dev/ttyUSB0";
const BAUD_RATE: u32 = 100_0000;
const FLIPPER_IDS: [u8; 4] = [1, 2, 3, 4];

fn main() -> Result<(), Error> {
    // 1. ハードウェア初期化
    let mut driver = DynamixelDriver::new(PORT_NAME, BAUD_RATE, FLIPPER_IDS.to_vec())?;
    driver.init_velocity_mode()?;
    let shared_driver = Arc::new(Mutex::new(driver));

    // 2. ROS 2 ノード作成 (API変更点: InitOptionsが必要)
    let context = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
    
    // API変更点: create_node ではなく Node::new
    let node = rclrs::Node::new(&context, "flipper_driver")?;

    // 3. Publisher
    let publisher = node.create_publisher::<sensor_msgs::msg::JointState>("/joint_states", QOS_PROFILE_DEFAULT)?;
    let driver_for_pub = shared_driver.clone();

    // Timer (API変更点: Nodeから直接タイマーを作るのではなく、clockを使うことが多いが、
    // create_timerはまだNodeにあるはず。なければ loop { spin_once } パターンにする)
    // rclrs 0.7 では create_timer が推奨されます。
    let _timer = node.create_timer(Duration::from_millis(100), move || {
        let mut drv = driver_for_pub.lock().unwrap();
        match drv.read_positions() {
            Ok(positions) => {
                let mut msg = sensor_msgs::msg::JointState::default();
                // timestampの設定は一旦省略（複雑なため）
                msg.name = vec![
                    "flipper_FL".to_string(), "flipper_FR".to_string(),
                    "flipper_RL".to_string(), "flipper_RR".to_string()
                ];
                msg.position = positions.iter().map(|&p| p as f64).collect();
                let _ = publisher.publish(&msg);
            },
            Err(e) => eprintln!("Read error: {:?}", e),
        }
    });

    // 4. Subscriber
    let driver_for_sub = shared_driver.clone();
    let _subscription = node.create_subscription::<sensor_msgs::msg::JointState, _>(
        "/flipper_commands",
        QOS_PROFILE_DEFAULT,
        move |msg: sensor_msgs::msg::JointState| {
            let mut drv = driver_for_sub.lock().unwrap();
            let velocities: Vec<i32> = msg.velocity.iter().map(|&v| v as i32).collect();
            if !velocities.is_empty() {
                let _ = drv.write_velocities(&velocities);
            }
        },
    )?;

    println!("Node is running (rclrs v0.7.0)...");
    
    // 5. Spin (API変更点: rclrs::spin(&node))
    rclrs::spin(&node)?;

    Ok(())
}
