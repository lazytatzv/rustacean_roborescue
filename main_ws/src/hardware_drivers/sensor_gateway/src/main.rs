/// sensor_gateway: STM32 BNO055 serial parser → ROS 2 /imu/data publisher
///
/// Reads CSV lines from the STM32 UART:
///   "heading,roll,pitch,sys_cal,gyr_cal,acc_cal,mag_cal\r\n"
///
/// Converts Euler angles to a quaternion and publishes sensor_msgs/Imu.
/// Calibration status is logged periodically.
///
/// IMU データ処理は `imu` モジュールに委譲。
mod imu;

use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, Publisher, RclrsErrorFilter, SpinOptions};
use sensor_msgs::msg::Imu as ImuMsg;
use std::io::{BufRead, BufReader};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use std_msgs::msg::Bool;

use crate::imu::{build_imu_msg, parse_csv_line, ImuData};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
const DEFAULT_PORT: &str = "/dev/stm32";
const DEFAULT_BAUD: u32 = 115_200;

/// Frame ID for the IMU
const IMU_FRAME_ID: &str = "imu_link";

/// How often to log calibration status (every N messages)
const CALIB_LOG_INTERVAL: u32 = 50;

// ---------------------------------------------------------------------------
// Serial reader thread
// ---------------------------------------------------------------------------

fn serial_reader_thread(
    port_name: String,
    baud_rate: u32,
    shared: Arc<Mutex<Option<ImuData>>>,
    last_recv: Arc<Mutex<Option<Instant>>>,
    shutdown_flag: Arc<AtomicBool>,
) {
    // Configure the serial port using stty
    let stty_result = std::process::Command::new("stty")
        .args([
            "-F",
            &port_name,
            &baud_rate.to_string(),
            "raw",
            "-echo",
            "-echoe",
            "-echok",
        ])
        .status();

    match stty_result {
        Ok(status) if status.success() => {
            println!(
                "✅ Serial port configured: {} @ {}bps",
                port_name, baud_rate
            );
        }
        _ => {
            eprintln!("⚠️  Failed to configure serial port with stty, trying anyway...");
        }
    }

    while !shutdown_flag.load(Ordering::Relaxed) {
        match std::fs::File::open(&port_name) {
            Ok(file) => {
                let reader = BufReader::new(file);
                for line_result in reader.lines() {
                    if shutdown_flag.load(Ordering::Relaxed) {
                        break;
                    }
                    match line_result {
                        Ok(line) => {
                            if let Some(data) = parse_csv_line(&line) {
                                if let Ok(mut guard) = shared.lock() {
                                    *guard = Some(data);
                                }
                                if let Ok(mut t) = last_recv.lock() {
                                    *t = Some(Instant::now());
                                }
                            }
                        }
                        Err(e) => {
                            eprintln!("⚠️  Serial read error: {e}");
                            break; // reopen
                        }
                    }
                }
            }
            Err(e) => {
                eprintln!("⚠️  Cannot open {port_name}: {e}, retrying in 2s...");
            }
        }

        // Sleep in small increments to check shutdown flag promptly
        for _ in 0..20 {
            if shutdown_flag.load(Ordering::Relaxed) {
                break;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    println!("🛑 sensor_gateway: serial reader thread exited cleanly");
}

// ---------------------------------------------------------------------------
// ROS 2 Node
// ---------------------------------------------------------------------------

fn run() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("sensor_gateway")?;

    // ---- Parameters -------------------------------------------------------
    let port_name: Arc<str> = node
        .declare_parameter("serial_port")
        .default(Arc::from(DEFAULT_PORT))
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();
    let port_name = port_name.to_string();

    let baud_rate: i64 = node
        .declare_parameter("baud_rate")
        .default(DEFAULT_BAUD as i64)
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();

    // ---- Shared state -----------------------------------------------------
    let shared: Arc<Mutex<Option<ImuData>>> = Arc::new(Mutex::new(None));
    let last_recv: Arc<Mutex<Option<Instant>>> = Arc::new(Mutex::new(None));

    // ---- Shutdown flag (shared with serial reader thread) -----------------
    let shutdown_flag = Arc::new(AtomicBool::new(false));

    // ---- Serial reader thread ---------------------------------------------
    let shared_reader = Arc::clone(&shared);
    let last_recv_reader = Arc::clone(&last_recv);
    let port_clone = port_name.clone();
    let shutdown_clone = Arc::clone(&shutdown_flag);
    let serial_thread = std::thread::spawn(move || {
        serial_reader_thread(
            port_clone,
            baud_rate as u32,
            shared_reader,
            last_recv_reader,
            shutdown_clone,
        );
    });

    // ---- Publishers -------------------------------------------------------
    let imu_pub: Publisher<ImuMsg> = node.create_publisher("/imu/data")?;
    let health_pub: Publisher<Bool> = node.create_publisher("/imu/health")?;

    // ---- Health timer (1 Hz) ----------------------------------------------
    let last_recv_health = Arc::clone(&last_recv);
    let _health_timer = node.create_timer_repeating(Duration::from_secs(1), move || {
        let healthy = if let Ok(guard) = last_recv_health.lock() {
            guard
                .map(|t| t.elapsed() < Duration::from_secs(2))
                .unwrap_or(false)
        } else {
            false
        };
        let _ = health_pub.publish(&Bool { data: healthy });
    })?;

    // ---- IMU timer (publish at ~50 Hz) ------------------------------------
    let shared_timer = Arc::clone(&shared);
    let mut msg_count: u32 = 0;

    let _timer = node.create_timer_repeating(Duration::from_millis(20), move || {
        let mut guard = match shared_timer.lock() {
            Ok(g) => g,
            Err(poisoned) => poisoned.into_inner(),
        };
        if let Some(ref mut data) = *guard {
            if !data.fresh {
                return;
            }
            data.fresh = false;

            let imu_msg = build_imu_msg(data, IMU_FRAME_ID);
            let _ = imu_pub.publish(&imu_msg);

            msg_count += 1;
            #[allow(clippy::manual_is_multiple_of)]
            if msg_count % CALIB_LOG_INTERVAL == 0 {
                println!(
                    "📡 IMU calib: sys={} gyr={} acc={} mag={} | heading={:.1}° roll={:.1}° pitch={:.1}°",
                    data.sys_cal, data.gyr_cal, data.acc_cal, data.mag_cal,
                    data.heading, data.roll, data.pitch,
                );
            }
        }
    })?;

    println!("🚀 sensor_gateway started");
    println!("   Serial: {} @ {}bps", port_name, baud_rate);
    println!("   Publish: /imu/data (sensor_msgs/Imu)");
    println!("   Publish: /imu/health (std_msgs/Bool) @ 1Hz");

    executor.spin(SpinOptions::default()).first_error()?;

    // ── Signal serial thread to stop and wait for clean shutdown ──
    shutdown_flag.store(true, Ordering::Relaxed);
    if serial_thread.join().is_err() {
        eprintln!("⚠️  sensor_gateway: serial thread panicked during shutdown");
    }

    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("🔥 sensor_gateway fatal: {e:#}");
        std::process::exit(1);
    }
}
