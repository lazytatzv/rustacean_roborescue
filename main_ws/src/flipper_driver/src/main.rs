mod driver;

use anyhow::{Context as _, Result};
use driver::DynamixelDriver;
use rclrs::{Context, Node, Publisher, Subscription, SpinOptions, RclrsErrorFilter};
use rclrs::CreateBasicExecutor;

use std::sync::mpsc::{channel, Receiver, Sender};
use std::thread;
use std::time::Duration;
use std::collections::HashMap;
use sensor_msgs::msg::JointState;

// --- 設定 ---
const PORT_NAME: &str = "/dev/ttyUSB0";
const BAUD_RATE: u32 = 115200;

// 関節名とDynamixel IDの対応表
// これがあるおかげで、JointStateの順番がバラバラでも正しく動く
const JOINTS: &[(&str, u8)] = &[
    ("flipper_front_left",  1),
    ("flipper_front_right", 2),
    ("flipper_rear_left",   3),
    ("flipper_rear_right",  4),
];

struct FlipperDriverNode {
    _node: Node,
    _sub: Subscription<JointState>,
    _pub: Publisher<JointState>,
    _driver_handle: thread::JoinHandle<()>,
}

impl FlipperDriverNode {
    fn new(node: Node) -> Result<Self> {
        // Driverスレッドへ指令を送るチャンネル
        let (tx_cmd, rx_cmd): (Sender<Vec<i32>>, Receiver<Vec<i32>>) = channel();

        // ---------------------------------------------------------
        // Publisher: 現在の関節状態 (JointState) を報告する
        // ---------------------------------------------------------
        let publisher = node.create_publisher::<JointState>("/joint_states")?;
        let publisher_clone = publisher.clone();

        // ---------------------------------------------------------
        // Driver Thread: ハードウェア通信専用
        // ---------------------------------------------------------
        let driver_handle = thread::spawn(move || {
            // モーターIDリストを作成
            let ids: Vec<u8> = JOINTS.iter().map(|(_, id)| *id).collect();
            let joint_names: Vec<String> = JOINTS.iter().map(|(name, _)| name.to_string()).collect();

            // ドライバ初期化
            let mut driver = DynamixelDriver::new(PORT_NAME, BAUD_RATE, ids.clone())
                .expect("🔥 Driver Init Failed! Check USB Connection.");
            
            if let Err(e) = driver.init_velocity_mode() {
                eprintln!("🔥 Init Mode Failed: {:?}", e);
                return;
            }
            println!("✅ Hardware Ready: Polling started.");

            loop {
                // 1. 命令があれば書き込む (Non-blocking)
                while let Ok(cmd_velocities) = rx_cmd.try_recv() {
                    // ID順に並んだ速度リストが来ている前提
                    if let Err(e) = driver.write_velocities(&cmd_velocities) {
                        eprintln!("⚠️ Write Error: {:?}", e);
                    }
                }

                // 2. 現在位置を読み込んで Publish
                match driver.read_positions() {
                    Ok(positions) => {
                        let mut msg = JointState::default();
                        // header.stamp は rclrs では現状手動設定が難しいので省略可、あるいはSystemTime使用
                        msg.name = joint_names.clone();
                        // position は float64 なのでキャスト
                        msg.position = positions.iter().map(|&p| p as f64).collect();
                        
                        // Hardwareの生値(velocity/effort)も読み込めるなら入れたほうが良いが今回は省略
                        let _ = publisher_clone.publish(&msg);
                    }
                    Err(e) => eprintln!("⚠️ Read Error: {:?}", e),
                }

                // 制御ループ周期 (例えば 20Hz = 50ms)
                thread::sleep(Duration::from_millis(50));
            }
        });

        // ---------------------------------------------------------
        // Subscriber: 指令 (JointState) を受け取る
        // ---------------------------------------------------------
        let _sub = node.create_subscription::<JointState, _>(
            "/flipper_commands", // 受信トピック名
            move |msg: JointState| {
                // 受信したメッセージが空なら無視
                if msg.name.is_empty() || msg.velocity.is_empty() { return; }

                // 【最強ポイント】
                // 受信したJointStateの順番がどうなっていても、
                // 正しいIDの順番 (1, 2, 3, 4) に並べ直してDriverに送る。
                
                // 1. 受信データを Map にして検索しやすくする
                let mut income_map: HashMap<String, f64> = HashMap::new();
                for (i, name) in msg.name.iter().enumerate() {
                    if i < msg.velocity.len() {
                        income_map.insert(name.clone(), msg.velocity[i]);
                    }
                }

                // 2. 定義済みID順 (1,2,3,4) に速度を抽出する
                let mut ordered_velocities: Vec<i32> = Vec::new();
                for (target_name, _) in JOINTS.iter() {
                    // 名前が一致するデータがあれば取得、なければ 0 (停止)
                    let vel_float = income_map.get(*target_name).copied().unwrap_or(0.0);
                    
                    // float -> int32 変換 (Dynamixelの指令値へ)
                    // 必要に応じて係数を掛ける (例: rad/s からの変換など)
                    // 今回はOperator側で既に生値を送っている前提なら 1.0
                    ordered_velocities.push(vel_float as i32);
                }

                // 3. Driverスレッドへ送信
                let _ = tx_cmd.send(ordered_velocities);
            },
        )?;

        Ok(Self {
            _node: node,
            _sub,
            _pub: publisher,
            _driver_handle: driver_handle,
        })
    }
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("flipper_driver")?;

    let _driver = FlipperDriverNode::new(node)?;

    println!("🚀 Flipper Driver Node Started.");
    println!("   Subscribing: /flipper_commands (sensor_msgs/JointState)");
    println!("   Publishing:  /joint_states     (sensor_msgs/JointState)");

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
