mod driver;

use anyhow::{Context as _, Result};
use driver::DynamixelDriver;
use rclrs::{Context, Node, Publisher, Subscription, SpinOptions};
use rclrs::CreateBasicExecutor;
// 【追加】この行が必要です（エラー E0599 の対策）
use rclrs::RclrsErrorFilter; 

use std::sync::mpsc::{channel, Receiver, Sender};
use std::thread;
use std::time::Duration;
use sensor_msgs::msg::{JointState, Joy};

const PORT_NAME: &str = "/dev/ttyUSB0";
const BAUD_RATE: u32 = 115200; 
const FLIPPER_IDS: [u8; 4] = [1, 2, 3, 4];

struct FlipperNode {
    node: Node,
    _subscription: Subscription<Joy>,
    _publisher: Publisher<JointState>,
    _driver_thread: thread::JoinHandle<()>,
}

impl FlipperNode {
    fn new(node: Node) -> Result<Self> {
        let (tx_cmd, rx_cmd): (Sender<Vec<i32>>, Receiver<Vec<i32>>) = channel();

        // 【修正1】 QoS引数 (rclrs::QOS_PROFILE_DEFAULT) を削除
        let publisher = node.create_publisher::<JointState>("/joint_states")?;
        let publisher_clone = publisher.clone();

        let driver_thread = thread::spawn(move || {
            let mut driver = DynamixelDriver::new(PORT_NAME, BAUD_RATE, FLIPPER_IDS.to_vec())
                .expect("🔥 Driver Init Failed! Check USB & Baudrate.");
            
            if let Err(e) = driver.init_velocity_mode() {
                eprintln!("🔥 Mode Set Failed: {:?}", e);
                return;
            }
            
            println!("✅ Driver Thread Started. (Baud: {})", BAUD_RATE);

            loop {
                while let Ok(velocities) = rx_cmd.try_recv() {
                    if let Err(e) = driver.write_velocities(&velocities) {
                        eprintln!("Write failed: {:?}", e);
                    }
                }

                if let Ok(positions) = driver.read_positions() {
                    let mut msg = JointState::default();
                    msg.name = vec![
                        "flipper_FL".to_string(), "flipper_FR".to_string(),
                        "flipper_RL".to_string(), "flipper_RR".to_string()
                    ];
                    msg.position = positions.iter().map(|&p| p as f64).collect();
                    let _ = publisher_clone.publish(&msg);
                }

                thread::sleep(Duration::from_millis(50));
            }
        });

        // 【修正2】 QoS引数 (rclrs::QOS_PROFILE_DEFAULT) を削除
        let _subscription = node.create_subscription::<Joy, _>(
            "joy",
            move |msg: Joy| {
                if msg.axes.len() <= 1 {
                    return;
                }

                let raw = msg.axes[1];
                let target_velocity = (200.0 * raw) as i32;
                let commands = vec![target_velocity; 4]; 
                
                let _ = tx_cmd.send(commands);
            },
        )?;

        Ok(Self {
            node,
            _subscription,
            _publisher: publisher,
            _driver_thread: driver_thread,
        })
    }
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("flipper_driver")?;

    let _flipper_app = FlipperNode::new(node)?;

    println!("🚀 Flipper Driver Node is Running... Listening on 'joy'");

    // 【完了】use rclrs::RclrsErrorFilter; を入れたのでここでエラーが出なくなります
    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())
}
