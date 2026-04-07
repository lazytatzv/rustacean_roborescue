/// depth_guard: D435i 深度画像から ROI 別近接障害物距離を publish する。
///
/// ## Parameters
///   enabled         (bool,   default false)
///     true にしない限り subscribe も publish も行わずアイドル動作する。
///     他ノードへの影響ゼロ。
///
///   depth_topic     (string, default "/camera_d435i/depth/image_rect_raw")
///     購読する深度画像トピック。
///
///   min_valid_m     (f64,    default 0.1)
///     有効深度下限 [m]。これ未満は計測不能として無視する。
///
///   max_valid_m     (f64,    default 3.0)
///     有効深度上限 [m]。これ超えは無視する (近接障害物のみ検出)。
///
///   roi_top_ratio   (f64,    default 0.3)
///     有効 ROI の上端 (画像高さの割合、0.0〜1.0)。
///
///   roi_bot_ratio   (f64,    default 0.7)
///     有効 ROI の下端 (画像高さの割合、0.0〜1.0)。
///
///   publish_rate_hz (f64,    default 10.0)
///     proximity トピックの publish 周期 [Hz]。
///
///   timeout_s       (f64,    default 3.0)
///     深度画像がこの秒数届かなかったら health=false にする。
///
/// ## Published Topics
///   /depth_guard/proximity  (std_msgs/Float32MultiArray)
///     [left_m, center_m, right_m]: 左・中央・右 ROI の最短距離 [m]。
///     障害物なしは f32::INFINITY。深度タイムアウト時は publish しない。
///
///   /depth_guard/health     (std_msgs/Bool)  @ 1 Hz
///     深度画像が timeout_s 以内に届いているか。
///
/// ## Supported Encodings
///   "16UC1"  : uint16、単位ミリメートル (D435i デフォルト)
///   "32FC1"  : float32、単位メートル
///   その他   : 警告ログのみ、フレームをスキップ
use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, Publisher, RclrsErrorFilter, SpinOptions};
use sensor_msgs::msg::Image;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use std_msgs::msg::{Bool, Float32MultiArray, MultiArrayLayout};

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

struct Config {
    min_valid_m: f32,
    max_valid_m: f32,
    roi_top_ratio: f32,
    roi_bot_ratio: f32,
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

struct State {
    /// [left_m, center_m, right_m]  INFINITY = 障害物なし / 範囲外
    proximity: [f32; 3],
    last_update: Option<Instant>,
}

impl Default for State {
    fn default() -> Self {
        Self {
            proximity: [f32::INFINITY; 3],
            last_update: None,
        }
    }
}

// ---------------------------------------------------------------------------
// Depth image processing (no unsafe)
// ---------------------------------------------------------------------------

fn process_depth(msg: &Image, cfg: &Config) -> [f32; 3] {
    let width = msg.width as usize;
    let height = msg.height as usize;
    let step = msg.step as usize;

    if width == 0 || height == 0 || step == 0 || msg.data.is_empty() {
        return [f32::INFINITY; 3];
    }

    let row_start = ((height as f32 * cfg.roi_top_ratio) as usize).min(height);
    let row_end = ((height as f32 * cfg.roi_bot_ratio) as usize).min(height);
    if row_start >= row_end {
        return [f32::INFINITY; 3];
    }

    let col_l_end = width / 3;
    let col_r_start = 2 * width / 3;
    let mut mins = [f32::INFINITY; 3];

    match msg.encoding.as_str() {
        "16UC1" => {
            // uint16 little-endian、単位: ミリメートル
            for row in row_start..row_end {
                let base = row * step;
                for col in 0..width {
                    let i = base + col * 2;
                    if i + 1 >= msg.data.len() {
                        break;
                    }
                    let raw = u16::from_le_bytes([msg.data[i], msg.data[i + 1]]);
                    if raw == 0 {
                        continue; // 無効ピクセル
                    }
                    let m = raw as f32 * 1e-3;
                    if m < cfg.min_valid_m || m > cfg.max_valid_m {
                        continue;
                    }
                    let roi = roi_index(col, col_l_end, col_r_start);
                    if m < mins[roi] {
                        mins[roi] = m;
                    }
                }
            }
        }
        "32FC1" => {
            // float32 little-endian、単位: メートル
            for row in row_start..row_end {
                let base = row * step;
                for col in 0..width {
                    let i = base + col * 4;
                    if i + 3 >= msg.data.len() {
                        break;
                    }
                    let m = f32::from_le_bytes([
                        msg.data[i],
                        msg.data[i + 1],
                        msg.data[i + 2],
                        msg.data[i + 3],
                    ]);
                    if !m.is_finite() || m <= 0.0 {
                        continue;
                    }
                    if m < cfg.min_valid_m || m > cfg.max_valid_m {
                        continue;
                    }
                    let roi = roi_index(col, col_l_end, col_r_start);
                    if m < mins[roi] {
                        mins[roi] = m;
                    }
                }
            }
        }
        enc => {
            eprintln!("depth_guard: unsupported encoding '{enc}', skipping frame");
        }
    }

    mins
}

#[inline]
fn roi_index(col: usize, col_l_end: usize, col_r_start: usize) -> usize {
    if col < col_l_end {
        0
    } else if col < col_r_start {
        1
    } else {
        2
    }
}

// ---------------------------------------------------------------------------
// ROS 2 node
// ---------------------------------------------------------------------------

fn run() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("depth_guard")?;

    // ---- Parameters --------------------------------------------------------
    let enabled: bool = node
        .declare_parameter("enabled")
        .default(false)
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();

    if !enabled {
        println!("depth_guard: disabled (param enabled=false), running idle");
        executor.spin(SpinOptions::default()).first_error()?;
        return Ok(());
    }

    let depth_topic: Arc<str> = node
        .declare_parameter("depth_topic")
        .default(Arc::from("/camera_d435i/depth/image_rect_raw"))
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();
    let depth_topic = depth_topic.to_string();

    let min_valid_m: f64 = node
        .declare_parameter("min_valid_m")
        .default(0.1_f64)
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();

    let max_valid_m: f64 = node
        .declare_parameter("max_valid_m")
        .default(3.0_f64)
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();

    let roi_top_ratio: f64 = node
        .declare_parameter("roi_top_ratio")
        .default(0.3_f64)
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();

    let roi_bot_ratio: f64 = node
        .declare_parameter("roi_bot_ratio")
        .default(0.7_f64)
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();

    let publish_rate_hz: f64 = node
        .declare_parameter("publish_rate_hz")
        .default(10.0_f64)
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();

    let timeout_s: f64 = node
        .declare_parameter("timeout_s")
        .default(3.0_f64)
        .mandatory()
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .get();

    let cfg = Arc::new(Config {
        min_valid_m: min_valid_m as f32,
        max_valid_m: max_valid_m as f32,
        roi_top_ratio: roi_top_ratio as f32,
        roi_bot_ratio: roi_bot_ratio as f32,
    });

    // ---- Shared state ------------------------------------------------------
    let state: Arc<Mutex<State>> = Arc::new(Mutex::new(State::default()));

    // ---- Depth subscription -----------------------------------------------
    let state_sub = Arc::clone(&state);
    let cfg_sub = Arc::clone(&cfg);
    let _sub = node.create_subscription::<Image, _>(depth_topic.as_str(), move |msg: Image| {
        let prox = process_depth(&msg, &cfg_sub);
        if let Ok(mut g) = state_sub.lock() {
            g.proximity = prox;
            g.last_update = Some(Instant::now());
        }
    })?;

    // ---- Publishers --------------------------------------------------------
    let prox_pub: Publisher<Float32MultiArray> = node.create_publisher("/depth_guard/proximity")?;
    let health_pub: Publisher<Bool> = node.create_publisher("/depth_guard/health")?;

    // ---- Proximity publish timer ------------------------------------------
    let pub_interval = Duration::from_secs_f64(1.0 / publish_rate_hz.clamp(0.1, 100.0));
    let timeout_dur = Duration::from_secs_f64(timeout_s);
    let state_pub = Arc::clone(&state);

    let _pub_timer = node.create_timer_repeating(pub_interval, move || {
        let (prox, fresh) = match state_pub.lock() {
            Ok(g) => {
                let fresh = g
                    .last_update
                    .map(|t| t.elapsed() < timeout_dur)
                    .unwrap_or(false);
                (g.proximity, fresh)
            }
            Err(_) => ([f32::INFINITY; 3], false),
        };

        // タイムアウト中は publish しない (購読側に古いデータを渡さない)
        if fresh {
            let msg = Float32MultiArray {
                layout: MultiArrayLayout::default(),
                data: prox.to_vec(),
            };
            let _ = prox_pub.publish(&msg);
        }
    })?;

    // ---- Health timer (1 Hz) -----------------------------------------------
    let state_health = Arc::clone(&state);
    let timeout_health = Duration::from_secs_f64(timeout_s);
    let _health_timer = node.create_timer_repeating(Duration::from_secs(1), move || {
        let healthy = match state_health.lock() {
            Ok(g) => g
                .last_update
                .map(|t| t.elapsed() < timeout_health)
                .unwrap_or(false),
            Err(_) => false,
        };
        let _ = health_pub.publish(&Bool { data: healthy });
    })?;

    println!("depth_guard: started");
    println!("  subscribe : {depth_topic}");
    println!("  publish   : /depth_guard/proximity  (Float32MultiArray [left,center,right] m)");
    println!("  publish   : /depth_guard/health     (Bool @ 1 Hz)");
    println!(
        "  config    : valid=[{min_valid_m:.2},{max_valid_m:.2}]m  \
         roi=[{roi_top_ratio:.2},{roi_bot_ratio:.2}]  \
         rate={publish_rate_hz:.0}Hz  timeout={timeout_s:.1}s"
    );

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("depth_guard fatal: {e:#}");
        std::process::exit(1);
    }
}
