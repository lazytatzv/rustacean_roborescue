mod driver;

use anyhow::{Context as _, Result};
use driver::DynamixelDriver;
use rclrs::{Context, Node, Publisher, Subscription, SpinOptions};
use rclrs::RclrsErrorFilter;
// ★これが必要です！これがないと context.create_basic_executor() が呼べません
use rclrs::CreateBasicExecutor; 

use std::sync::mpsc::{channel, Receiver, Sender};
use std::thread;
use std::time::Duration;
use sensor_msgs::msg::JointState;

const PORT_NAME: &str = "/dev/ttyUSB0";
const BAUD_RATE: u32 = 1_000_000;
const FLIPPER_IDS: [u8; 4] = [1, 2, 3, 4];

struct FlipperNode {
    // 最新の rclrs::Node は内部でArcを持っているので、ここでの Arc<> は不要です
    node: Node,
    _subscription: Subscription<JointState>,
    _publisher: Publisher<JointState>,
    _driver_thread: thread::JoinHandle<()>,
}

impl FlipperNode {
    fn new(node: Node) -> Result<Self> {
        let (tx_cmd, rx_cmd): (Sender<Vec<i32>>, Receiver<Vec<i32>>) = channel();

        // ★修正: 最新版では QoS 引数が不要になりました
        // create_publisher<メッセージ型>("トピック名")
        let publisher = node.create_publisher::<JointState>("/joint_states")?;
        let publisher_clone = publisher.clone();

        let driver_thread = thread::spawn(move || {
            let mut driver = match DynamixelDriver::new(PORT_NAME, BAUD_RATE, FLIPPER_IDS.to_vec()) {
                Ok(d) => d,
                Err(e) => {
                    eprintln!("🔥 Driver Init Failed: {:?}", e);
                    return; 
                }
            };
            
            if let Err(e) = driver.init_velocity_mode() {
                eprintln!("🔥 Mode Set Failed: {:?}", e);
                return;
            }
            
            println!("✅ Driver Thread Started.");

            loop {
                while let Ok(velocities) = rx_cmd.try_recv() {
                    let _ = driver.write_velocities(&velocities);
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

                thread::sleep(Duration::from_millis(100));
            }
        });

        // ★修正: QoS 引数を削除
        let _subscription = node.create_subscription::<JointState, _>(
            "/flipper_commands",
            move |msg: JointState| {
                let velocities: Vec<i32> = msg.velocity.iter().map(|&v| v as i32).collect();
                if !velocities.is_empty() {
                    let _ = tx_cmd.send(velocities);
                }
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
    
    // 1. Executorを作成
    // (use rclrs::CreateBasicExecutor; があるので呼べる！)
    let mut executor = context.create_basic_executor();
    
    // 2. ノードを作成 & Executorに登録
    // ソースコードにあった通り、executor.create_node が正解です。
    // これで作られたノードは自動的に Executor の管理下に入ります。
    let node = executor.create_node("flipper_driver")?;

    // 3. アプリ初期化
    let _flipper_app = FlipperNode::new(node)?;

    println!("🚀 Flipper Driver Node is Running...");

    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())
}