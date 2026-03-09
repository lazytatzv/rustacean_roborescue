/// IMU データ処理モジュール
///
/// STM32 + BNO055 から受信する CSV データのパース、
/// Euler 角 → クォータニオン変換、ROS 2 Imu メッセージ生成を担う。

use sensor_msgs::msg::Imu;
use std::time::{SystemTime, UNIX_EPOCH};

// ---------------------------------------------------------------------------
// IMU Data
// ---------------------------------------------------------------------------

/// STM32 から受信した BNO055 のデータ
#[derive(Debug, Clone)]
pub struct ImuData {
    /// ヨー角 (度)
    pub heading: f64,
    /// ロール角 (度)
    pub roll: f64,
    /// ピッチ角 (度)
    pub pitch: f64,
    /// システムキャリブレーション (0–3)
    pub sys_cal: u8,
    /// ジャイロキャリブレーション (0–3)
    pub gyr_cal: u8,
    /// 加速度キャリブレーション (0–3)
    pub acc_cal: u8,
    /// 磁気キャリブレーション (0–3)
    pub mag_cal: u8,
    /// 新着フラグ (publish 済みなら false)
    pub fresh: bool,
}

impl ImuData {
    /// キャリブレーションがすべて完了しているか
    pub fn is_fully_calibrated(&self) -> bool {
        self.sys_cal == 3 && self.gyr_cal == 3 && self.acc_cal == 3 && self.mag_cal == 3
    }
}

// ---------------------------------------------------------------------------
// CSV Parsing
// ---------------------------------------------------------------------------

/// STM32 からの CSV 行をパースする。
///
/// フォーマット: "heading,roll,pitch,sys_cal,gyr_cal,acc_cal,mag_cal\r\n"
pub fn parse_csv_line(line: &str) -> Option<ImuData> {
    let parts: Vec<&str> = line.trim().split(',').collect();
    if parts.len() < 7 {
        return None;
    }
    Some(ImuData {
        heading: parts[0].parse().ok()?,
        roll:    parts[1].parse().ok()?,
        pitch:   parts[2].parse().ok()?,
        sys_cal: parts[3].parse().ok()?,
        gyr_cal: parts[4].parse().ok()?,
        acc_cal: parts[5].parse().ok()?,
        mag_cal: parts[6].parse().ok()?,
        fresh: true,
    })
}

// ---------------------------------------------------------------------------
// Euler → Quaternion
// ---------------------------------------------------------------------------

/// Euler 角 (ZYX intrinsic / aerospace convention) をクォータニオンに変換する。
///
/// # 引数
/// - `heading_deg`: ヨー (度)
/// - `roll_deg`: ロール (度)
/// - `pitch_deg`: ピッチ (度)
///
/// # 戻り値
/// (w, x, y, z)
pub fn euler_to_quaternion(heading_deg: f64, roll_deg: f64, pitch_deg: f64) -> (f64, f64, f64, f64) {
    let yaw   = heading_deg.to_radians();
    let roll  = roll_deg.to_radians();
    let pitch = pitch_deg.to_radians();

    let (sy, cy) = (yaw   * 0.5).sin_cos();
    let (sp, cp) = (pitch * 0.5).sin_cos();
    let (sr, cr) = (roll  * 0.5).sin_cos();

    let w = cr * cp * cy + sr * sp * sy;
    let x = sr * cp * cy - cr * sp * sy;
    let y = cr * sp * cy + sr * cp * sy;
    let z = cr * cp * sy - sr * sp * cy;

    (w, x, y, z)
}

// ---------------------------------------------------------------------------
// ROS 2 Imu Message Builder
// ---------------------------------------------------------------------------

/// 現在時刻を builtin_interfaces/Time に変換
pub fn now_stamp() -> builtin_interfaces::msg::Time {
    let dur = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    builtin_interfaces::msg::Time {
        sec: dur.as_secs() as i32,
        nanosec: dur.subsec_nanos(),
    }
}

/// BNO055 NDOF モードの orientation covariance
const ORIENTATION_COV: [f64; 9] = [
    0.01, 0.0,  0.0,
    0.0,  0.01, 0.0,
    0.0,  0.0,  0.01,
];

/// angular_velocity / linear_acceleration が未知の場合の covariance
/// REP-145: covariance[0] = -1.0 → "unknown"
const UNKNOWN_COV: [f64; 9] = [
    -1.0, 0.0, 0.0,
     0.0, 0.0, 0.0,
     0.0, 0.0, 0.0,
];

/// ImuData から sensor_msgs/Imu メッセージを生成する。
pub fn build_imu_msg(data: &ImuData, frame_id: &str) -> Imu {
    let (w, x, y, z) = euler_to_quaternion(data.heading, data.roll, data.pitch);

    let mut msg = Imu::default();
    msg.header.stamp = now_stamp();
    msg.header.frame_id = frame_id.to_string();

    msg.orientation.w = w;
    msg.orientation.x = x;
    msg.orientation.y = y;
    msg.orientation.z = z;
    msg.orientation_covariance = ORIENTATION_COV;

    // STM32 からまだ gyro / accel は送信されていないので unknown 表記
    msg.angular_velocity_covariance = UNKNOWN_COV;
    msg.linear_acceleration_covariance = UNKNOWN_COV;

    msg
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_csv_valid() {
        let line = "123.4,5.6,7.8,3,3,2,1\r\n";
        let data = parse_csv_line(line).unwrap();
        assert!((data.heading - 123.4).abs() < 1e-6);
        assert!((data.roll - 5.6).abs() < 1e-6);
        assert!((data.pitch - 7.8).abs() < 1e-6);
        assert_eq!(data.sys_cal, 3);
        assert_eq!(data.gyr_cal, 3);
        assert_eq!(data.acc_cal, 2);
        assert_eq!(data.mag_cal, 1);
    }

    #[test]
    fn test_parse_csv_too_short() {
        assert!(parse_csv_line("1,2,3").is_none());
    }

    #[test]
    fn test_euler_to_quat_identity() {
        let (w, x, y, z) = euler_to_quaternion(0.0, 0.0, 0.0);
        assert!((w - 1.0).abs() < 1e-9);
        assert!(x.abs() < 1e-9);
        assert!(y.abs() < 1e-9);
        assert!(z.abs() < 1e-9);
    }

    #[test]
    fn test_euler_to_quat_unit_norm() {
        let (w, x, y, z) = euler_to_quaternion(45.0, 30.0, -10.0);
        let norm = (w * w + x * x + y * y + z * z).sqrt();
        assert!((norm - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_is_fully_calibrated() {
        let mut data = parse_csv_line("0,0,0,3,3,3,3").unwrap();
        assert!(data.is_fully_calibrated());
        data.sys_cal = 2;
        assert!(!data.is_fully_calibrated());
    }
}
