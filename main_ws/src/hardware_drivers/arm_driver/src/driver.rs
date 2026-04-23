#![allow(dead_code)]

use anyhow::{Context, Result};
use dynamixel2::serial2::SerialPort;
use dynamixel2::{instructions::SyncWriteData, Bus};

// ═══════════════════════════════════════════════════════════════════════════
//  Dynamixel XM series — Control Table Addresses
// ═══════════════════════════════════════════════════════════════════════════

const ADDR_OPERATING_MODE: u16 = 11; // 1 byte
const ADDR_TORQUE_ENABLE: u16 = 64; // 1 byte
const ADDR_GOAL_CURRENT: u16 = 102; // 2 bytes — for current-based position mode
const ADDR_PROFILE_VELOCITY: u16 = 112; // 4 bytes — limits speed during position move
const ADDR_GOAL_POSITION: u16 = 116; // 4 bytes
const ADDR_PRESENT_CURRENT: u16 = 126; // 2 bytes (signed)
const ADDR_PRESENT_VELOCITY: u16 = 128; // 4 bytes (signed)
const ADDR_PRESENT_POSITION: u16 = 132; // 4 bytes
const ADDR_CURRENT_LIMIT: u16 = 38; // 2 bytes (EEPROM)
const ADDR_PRESENT_TEMPERATURE: u16 = 146; // 1 byte

/// Extended Position Control Mode (multi-turn, signed 32-bit ticks, no ±180° limit)
const MODE_EXTENDED_POSITION: u8 = 4;
/// Current-based Position Control Mode
const MODE_CURRENT_BASED_POSITION: u8 = 5;

/// Dynamixel position resolution: 4096 counts per revolution
const COUNTS_PER_REV: f64 = 4096.0;

/// Dynamixel velocity unit: 0.229 rev/min per LSB
const VEL_UNIT_RPM: f64 = 0.229;

/// Temperature threshold for warning [°C]
const TEMP_WARN_THRESHOLD: u8 = 65;
/// Temperature threshold for emergency shutdown [°C]
const TEMP_SHUTDOWN_THRESHOLD: u8 = 75;

// ═══════════════════════════════════════════════════════════════════════════
//  Conversion helpers
// ═══════════════════════════════════════════════════════════════════════════

/// グリッパー用 (Position mode, 0-4095, center=2048)
pub fn rad_to_ticks(rad: f64) -> u32 {
    let ticks = (rad / (2.0 * std::f64::consts::PI) * COUNTS_PER_REV) + (COUNTS_PER_REV / 2.0);
    (ticks.round() as i64).clamp(0, 4095) as u32
}

/// グリッパー用 (Position mode, 0-4095, center=2048)
pub fn ticks_to_rad(ticks: u32) -> f64 {
    (ticks as f64 - COUNTS_PER_REV / 2.0) / COUNTS_PER_REV * 2.0 * std::f64::consts::PI
}

/// Extended Position mode用 (center=2048, Position modeと同じ原点、クランプなし)
pub fn rad_to_ticks_ext(rad: f64) -> i32 {
    (rad / (2.0 * std::f64::consts::PI) * COUNTS_PER_REV + COUNTS_PER_REV / 2.0).round() as i32
}

/// Extended Position mode用
pub fn ticks_to_rad_ext(ticks: i32) -> f64 {
    (ticks as f64 - COUNTS_PER_REV / 2.0) / COUNTS_PER_REV * 2.0 * std::f64::consts::PI
}

pub fn raw_vel_to_rad_s(raw: i32) -> f64 {
    raw as f64 * VEL_UNIT_RPM * 2.0 * std::f64::consts::PI / 60.0
}

pub fn raw_current_to_a(raw: i16) -> f64 {
    raw as f64 * 2.69e-3
}

// ═══════════════════════════════════════════════════════════════════════════
//  Driver
// ═══════════════════════════════════════════════════════════════════════════

pub struct ArmDynamixelDriver {
    bus: Bus<Vec<u8>, Vec<u8>>,
    arm_ids: Vec<u8>,
    gripper_ids: Vec<u8>,
    /// arm_directions[i]: +1.0 or -1.0. motor_rad = (urdf_rad + offset) * direction * reduction
    arm_directions: Vec<f64>,
    /// gripper_directions[i]: +1.0 or -1.0
    gripper_directions: Vec<f64>,
    /// per-joint offset [rad]
    arm_offsets: Vec<f64>,
    gripper_offsets: Vec<f64>,
    /// 減速比 [j1..jN]: motor_rad = (urdf_rad + offset) * direction * reduction
    arm_reductions: Vec<f64>,
}

pub struct MotorStatus {
    pub position_rad: f64,
    pub velocity_rad_s: f64,
    pub current_a: f64,
    pub temperature_c: u8,
}

type TemperatureReport = (u8, u8);

impl ArmDynamixelDriver {
    pub fn new(
        port_name: &str,
        baud_rate: u32,
        arm_ids: Vec<u8>,
        gripper_ids: Vec<u8>,
        arm_directions: Vec<f64>,
        gripper_directions: Vec<f64>,
        arm_offsets: Vec<f64>,
        gripper_offsets: Vec<f64>,
        arm_reductions: Vec<f64>,
    ) -> Result<Self> {
        let port = SerialPort::open(port_name, baud_rate).context("Failed to open serial port")?;
        let bus = Bus::new(port)?;
        let mut arm_off = arm_offsets;
        arm_off.resize(arm_ids.len(), 0.0);
        let mut g_off = gripper_offsets;
        g_off.resize(gripper_ids.len(), 0.0);
        let mut a_dir = arm_directions;
        a_dir.resize(arm_ids.len(), 1.0);
        let mut g_dir = gripper_directions;
        g_dir.resize(gripper_ids.len(), 1.0);
        let mut a_red = arm_reductions;
        a_red.resize(arm_ids.len(), 1.0);
        Ok(Self {
            bus,
            arm_ids,
            gripper_ids,
            arm_directions: a_dir,
            gripper_directions: g_dir,
            arm_offsets: arm_off,
            gripper_offsets: g_off,
            arm_reductions: a_red,
        })
    }

    pub fn ping(&mut self, id: u8) -> Result<()> {
        self.bus
            .ping(id)
            .map(|_| ())
            .map_err(|e| anyhow::anyhow!("Ping ID {} failed: {:?}", id, e))
    }

    /// モーター初期化。起動時はアームが必ずホームポジションにいることを前提に、
    /// home_urdf をゴール位置として書き込んでからトルクONする。
    /// これにより電源断後のマルチターンリセット問題を自動解決する。
    /// モーター初期化。起動時はアームが必ずホームポジションにいることを前提に、
    /// home_urdf をゴール位置として書き込んでからトルクONする。
    /// これにより電源断後のマルチターンリセット問題を自動解決する。
    pub fn init_motors(&mut self, profile_velocity: u32, gripper_max_current: u16, home_urdf: &[f64]) -> Result<()> {
        println!("⚙️  arm_driver: Starting motor initialization...");

        // Ping all motors first to verify connection
        let mut all_ids = self.arm_ids.clone();
        all_ids.extend_from_slice(&self.gripper_ids);
        for &id in &all_ids {
            if let Err(e) = self.ping(id) {
                eprintln!("❌ arm_driver: Failed to ping Dynamixel ID {id}: {e:?}");
                return Err(e);
            }
        }
        println!("✅ arm_driver: All motors (Arm + Gripper) responded to ping.");

        // Torque OFF (operating mode / profile_velocity は torque OFF 中でないと変更不可)
        for &id in &self.arm_ids {
            let _ = self.bus.write_u8(id, ADDR_TORQUE_ENABLE, 0);
        }
        for &id in &self.gripper_ids {
            let _ = self.bus.write_u8(id, ADDR_TORQUE_ENABLE, 0);
        }

        // アーム: Extended Position Mode (マルチターン対応)
        for &id in &self.arm_ids {
            self.bus
                .write_u8(id, ADDR_OPERATING_MODE, MODE_EXTENDED_POSITION)
                .map_err(|e| anyhow::anyhow!("ID {id} set operating_mode=ExtendedPosition failed: {e:?}"))?;
        }
        // グリッパー: Current-based Position Mode (電流=握力上限を設定できる)
        for &id in &self.gripper_ids {
            self.bus
                .write_u8(id, ADDR_OPERATING_MODE, MODE_CURRENT_BASED_POSITION)
                .map_err(|e| anyhow::anyhow!("gripper ID {id} set operating_mode=CurrentBasedPosition failed: {e:?}"))?;
        }

        // アーム: profile_velocity
        for &id in &self.arm_ids {
            self.bus
                .write_u32(id, ADDR_PROFILE_VELOCITY, profile_velocity)
                .map_err(|e| anyhow::anyhow!("ID {id} set profile_velocity={profile_velocity} failed: {e:?}"))?;
        }
        // グリッパー: 握力上限 (GOAL_CURRENT) — Current_Limit (EEPROM) を超えないようクランプ
        for &id in &self.gripper_ids {
            let effective = match self.bus.read_u16(id, ADDR_CURRENT_LIMIT) {
                Ok(resp) => {
                    if gripper_max_current > resp.data {
                        eprintln!("⚠️  gripper ID {id}: gripper_max_current={gripper_max_current} > Current_Limit={}, clamping", resp.data);
                        resp.data
                    } else {
                        gripper_max_current
                    }
                }
                Err(_) => gripper_max_current,
            };
            self.bus
                .write_u16(id, ADDR_GOAL_CURRENT, effective)
                .map_err(|e| anyhow::anyhow!("gripper ID {id} set goal_current={effective} failed: {e:?}"))?;
        }

        // アーム: ホームポジションのUDRF角度からモーターtickを計算してgoal_positionに書く。
        // 起動時はアームが必ずホームにいるという運用上の前提により、
        // 電源断後のマルチターンリセット問題を解決する。
        let home_len = home_urdf.len().min(self.arm_ids.len());
        let arm_commands: Vec<SyncWriteData<u32>> = self.arm_ids[..home_len].iter().enumerate()
            .map(|(i, &id)| {
                let ticks = rad_to_ticks_ext(
                    (home_urdf[i] + self.arm_offsets[i]) * self.arm_directions[i] * self.arm_reductions[i]
                ) as u32;
                println!("✅ arm_driver: joint{} home → ticks={}", i + 1, ticks as i32);
                SyncWriteData { motor_id: id, data: ticks }
            })
            .collect();
        self.bus.sync_write_u32(ADDR_GOAL_POSITION, &arm_commands)
            .map_err(|e| anyhow::anyhow!("sync_write home positions failed: {:?}", e))?;

        // グリッパー: 現在位置をそのままゴールに（シングルターンで問題なし）
        for &id in &self.gripper_ids {
            if let Ok(resp) = self.bus.read_u32(id, ADDR_PRESENT_POSITION) {
                let _ = self.bus.write_u32(id, ADDR_GOAL_POSITION, resp.data);
            }
        }

        // Torque ON
        for &id in &self.arm_ids {
            self.bus
                .write_u8(id, ADDR_TORQUE_ENABLE, 1)
                .map_err(|e| anyhow::anyhow!("ID {id} torque enable failed: {e:?}"))?;
        }
        for &id in &self.gripper_ids {
            self.bus
                .write_u8(id, ADDR_TORQUE_ENABLE, 1)
                .map_err(|e| anyhow::anyhow!("gripper ID {id} torque enable failed: {e:?}"))?;
        }

        println!("✅ arm_driver: Motor initialization complete.");
        Ok(())
    }

    pub fn write_arm_positions(&mut self, positions_rad: &[f64]) -> Result<()> {
        // URDF → モーター角度 (offset → direction → reduction) → extended ticks (i32 as u32 bits)
        let commands: Vec<SyncWriteData<u32>> = self
            .arm_ids
            .iter()
            .zip(positions_rad.iter())
            .zip(self.arm_offsets.iter())
            .zip(self.arm_directions.iter())
            .zip(self.arm_reductions.iter())
            .map(|((((  &id, &rad), &offset), &dir), &red)| SyncWriteData {
                motor_id: id,
                data: rad_to_ticks_ext((rad + offset) * dir * red) as u32,
            })
            .collect();
        self.bus
            .sync_write_u32(ADDR_GOAL_POSITION, &commands)
            .map_err(|e| anyhow::anyhow!("{:?}", e))
    }

    pub fn write_gripper_positions(&mut self, positions_rad: &[f64]) -> Result<()> {
        let commands: Vec<SyncWriteData<u32>> = self
            .gripper_ids
            .iter()
            .zip(positions_rad.iter())
            .zip(self.gripper_offsets.iter())
            .zip(self.gripper_directions.iter())
            .map(|(((&id, &rad), &offset), &dir)| SyncWriteData {
                motor_id: id,
                data: rad_to_ticks_ext((rad + offset) * dir) as u32,
            })
            .collect();
        self.bus
            .sync_write_u32(ADDR_GOAL_POSITION, &commands)
            .map_err(|e| anyhow::anyhow!("sync_write gripper positions failed: {:?}", e))
    }

    pub fn read_arm_positions(&mut self) -> Result<Vec<f64>> {
        let resp = self
            .bus
            .sync_read_u32(&self.arm_ids, ADDR_PRESENT_POSITION)
            .map_err(|e| anyhow::anyhow!("sync_read arm positions failed: {:?}", e))?;
        // extended ticks (u32 bits → i32) → モーター角度 → URDF 角度 (reduction → direction → offset)
        Ok(resp
            .iter()
            .zip(self.arm_offsets.iter())
            .zip(self.arm_directions.iter())
            .zip(self.arm_reductions.iter())
            .map(|(((r, &offset), &dir), &red)| ticks_to_rad_ext(r.data as i32) / dir / red - offset)
            .collect())
    }

    /// 全グリッパーサーボのステータスを返す
    pub fn read_gripper_statuses(&mut self) -> Result<Vec<MotorStatus>> {
        // Sync Read を使用して効率化 (Position, Velocity, Current をまとめて読むには本来別々だが、ここでは単純化のため個別または個別SyncRead)
        // 実際には ADDR_PRESENT_CURRENT から 2+4+4 = 10 bytes 連続しているので SyncRead で一気に読める
        // ADDR_PRESENT_CURRENT(126), PRESENT_VELOCITY(128), PRESENT_POSITION(132)
        // ここでは実装の単純化のため、個別に読むが、エラーハンドリングを改善
        let mut statuses = Vec::new();
        for (i, &id) in self.gripper_ids.iter().enumerate() {
            let dir = self.gripper_directions.get(i).copied().unwrap_or(1.0);
            let offset = self.gripper_offsets.get(i).copied().unwrap_or(0.0);

            let pos = self.bus.read_u32(id, ADDR_PRESENT_POSITION).context("read pos")?;
            let vel = self.bus.read_u32(id, ADDR_PRESENT_VELOCITY).context("read vel")?;
            let cur = self.bus.read_u16(id, ADDR_PRESENT_CURRENT).context("read cur")?;
            let tmp = self.bus.read_u8(id, ADDR_PRESENT_TEMPERATURE).context("read tmp")?;

            statuses.push(MotorStatus {
                position_rad: ticks_to_rad_ext(pos.data as i32) / dir - offset,
                velocity_rad_s: raw_vel_to_rad_s(vel.data as i32) / dir,
                current_a: raw_current_to_a(cur.data as i16),
                temperature_c: tmp.data,
            });
        }
        Ok(statuses)
    }

    /// 最初のグリッパーサーボのステータスを返す (後方互換用)
    pub fn read_gripper_status(&mut self) -> Result<MotorStatus> {
        let statuses = self.read_gripper_statuses()?;
        statuses.into_iter().next().ok_or_else(|| anyhow::anyhow!("no gripper ids"))
    }

    /// 各グリッパーモーターをそれぞれの現在位置でホールドする。
    /// write_gripper_position は全モーターに同一URDF角を送るため独立ホールドに使えない。
    pub fn write_gripper_hold(&mut self, statuses: &[MotorStatus]) -> Result<()> {
        let mut last_err = None;
        for (i, (status, &id)) in statuses.iter().zip(self.gripper_ids.iter()).enumerate() {
            let offset = self.gripper_offsets.get(i).copied().unwrap_or(0.0);
            let dir    = self.gripper_directions.get(i).copied().unwrap_or(1.0);
            let ticks  = rad_to_ticks_ext((status.position_rad + offset) * dir) as u32;
            if let Err(e) = self.bus.write_u32(id, ADDR_GOAL_POSITION, ticks) {
                last_err = Some(anyhow::anyhow!("gripper hold ID {id}: {:?}", e));
            }
        }
        last_err.map_or(Ok(()), Err)
    }

    pub fn emergency_stop(&mut self) {
        self.torque_off_all();
    }

    pub fn check_temperatures(&mut self) -> (Vec<TemperatureReport>, Vec<TemperatureReport>) {
        let mut warnings = Vec::new();
        let mut critical = Vec::new();
        let mut all_ids = self.arm_ids.clone();
        all_ids.extend_from_slice(&self.gripper_ids);
        for id in all_ids {
            if let Ok(resp) = self.bus.read_u8(id, ADDR_PRESENT_TEMPERATURE) {
                let temp = resp.data;
                if temp >= TEMP_SHUTDOWN_THRESHOLD {
                    critical.push((id, temp));
                } else if temp >= TEMP_WARN_THRESHOLD {
                    warnings.push((id, temp));
                }
            }
        }
        (warnings, critical)
    }

    pub fn torque_off_all(&mut self) {
        for &id in &self.arm_ids {
            let _ = self.bus.write_u8(id, ADDR_TORQUE_ENABLE, 0);
        }
        for &id in &self.gripper_ids {
            let _ = self.bus.write_u8(id, ADDR_TORQUE_ENABLE, 0);
        }
        println!("🔴 arm_driver: All torques OFF");
    }

    pub fn reboot_all(&mut self) {
        let mut all_ids = self.arm_ids.clone();
        all_ids.extend_from_slice(&self.gripper_ids);
        for id in all_ids {
            match self.bus.reboot(id) {
                Ok(_) => println!("🔄 arm_driver: Reboot sent to ID {id}"),
                Err(e) => eprintln!("⚠️  arm_driver: Reboot ID {id} failed: {e:?}"),
            }
        }
        // モーター再起動待機
        std::thread::sleep(std::time::Duration::from_millis(2000));
        println!("✅ arm_driver: Reboot complete");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Unit tests — ハードウェア不要、変換関数の数学的正しさを検証
// ═══════════════════════════════════════════════════════════════════════════
#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const COUNTS: f64 = COUNTS_PER_REV; // 4096.0
    const HALF: f64 = COUNTS_PER_REV / 2.0; // 2048.0

    // ─── rad_to_ticks_ext / ticks_to_rad_ext 基本 ────────────────────────

    #[test]
    fn ext_zero_rad_is_2048_ticks() {
        assert_eq!(rad_to_ticks_ext(0.0), 2048);
    }

    #[test]
    fn ext_2pi_rad_is_6144_ticks() {
        // 2π → (2π / 2π) * 4096 + 2048 = 4096 + 2048 = 6144
        assert_eq!(rad_to_ticks_ext(2.0 * PI), 6144);
    }

    #[test]
    fn ext_neg_2pi_rad_is_neg2048_ticks() {
        // -2π → -4096 + 2048 = -2048
        assert_eq!(rad_to_ticks_ext(-2.0 * PI), -2048);
    }

    #[test]
    fn ext_pi_rad_is_4096_ticks() {
        assert_eq!(rad_to_ticks_ext(PI), 4096);
    }

    #[test]
    fn ext_neg_pi_rad_is_zero_ticks() {
        assert_eq!(rad_to_ticks_ext(-PI), 0);
    }

    #[test]
    fn ext_ticks_center_is_zero_rad() {
        assert!((ticks_to_rad_ext(2048) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn ext_ticks_6144_is_2pi_rad() {
        assert!((ticks_to_rad_ext(6144) - 2.0 * PI).abs() < 1e-9);
    }

    #[test]
    fn ext_ticks_neg2048_is_neg_2pi_rad() {
        assert!((ticks_to_rad_ext(-2048) - (-2.0 * PI)).abs() < 1e-9);
    }

    // ─── ラウンドトリップ: rad → ticks → rad ─────────────────────────────

    #[test]
    fn ext_roundtrip_various_angles() {
        // 量子化誤差上限: 2π / 4096 ≈ 0.001534 rad
        let max_err = 2.0 * PI / COUNTS;
        let angles = [
            -5.0 * PI, -3.0 * PI, -PI, -PI / 2.0, -PI / 4.0,
            0.0, PI / 4.0, PI / 2.0, PI, 3.0 * PI, 5.0 * PI,
        ];
        for &a in &angles {
            let t = rad_to_ticks_ext(a);
            let back = ticks_to_rad_ext(t);
            let err = (back - a).abs();
            assert!(
                err <= max_err + 1e-12,
                "roundtrip error for {:.4} rad: {:.6} (ticks={})", a, err, t
            );
        }
    }

    #[test]
    fn ext_roundtrip_monotone() {
        // rad_to_ticks_ext は単調増加であることを確認
        let angles: Vec<f64> = (-10..=10).map(|i| i as f64 * PI / 5.0).collect();
        let ticks: Vec<i32> = angles.iter().map(|&a| rad_to_ticks_ext(a)).collect();
        for i in 0..ticks.len() - 1 {
            assert!(
                ticks[i] <= ticks[i + 1],
                "rad_to_ticks_ext must be monotone: [{i}]={} > [{}]={}",
                ticks[i], i + 1, ticks[i + 1]
            );
        }
    }

    #[test]
    fn ext_quantization_bound() {
        // 1 tick = 2π/4096 rad ≈ 0.001534 rad (< 0.088°)
        let one_tick_rad = 2.0 * PI / COUNTS;
        println!("[quant] 1 tick = {:.6} rad = {:.4}°", one_tick_rad, one_tick_rad.to_degrees());
        assert!(one_tick_rad < 0.002);
    }

    // ─── グリッパー用 rad_to_ticks / ticks_to_rad ────────────────────────

    #[test]
    fn gripper_zero_is_2048() {
        assert_eq!(rad_to_ticks(0.0), 2048);
    }

    #[test]
    fn gripper_clamps_to_0_4095() {
        assert_eq!(rad_to_ticks(-100.0), 0);
        assert_eq!(rad_to_ticks(100.0), 4095);
    }

    #[test]
    fn gripper_roundtrip() {
        let angles = [-PI / 2.0, -PI / 4.0, 0.0, PI / 4.0, PI / 2.0];
        let max_err = 2.0 * PI / COUNTS;
        for &a in &angles {
            let t = rad_to_ticks(a);
            let back = ticks_to_rad(t);
            let err = (back - a).abs();
            assert!(err <= max_err + 1e-12,
                    "gripper roundtrip error for {:.4} rad: {:.6}", a, err);
        }
    }

    // ─── 角速度変換 raw_vel_to_rad_s ─────────────────────────────────────

    #[test]
    fn vel_zero_is_zero() {
        assert_eq!(raw_vel_to_rad_s(0), 0.0);
    }

    #[test]
    fn vel_1lsb_equals_0229rpm_in_rad_s() {
        // 0.229 RPM = 0.229 * 2π / 60 rad/s
        let expected = VEL_UNIT_RPM * 2.0 * PI / 60.0;
        let result = raw_vel_to_rad_s(1);
        println!("[vel] 1 LSB = {:.6} rad/s", result);
        assert!((result - expected).abs() < 1e-12);
    }

    #[test]
    fn vel_sign_symmetric() {
        let pos = raw_vel_to_rad_s(100);
        let neg = raw_vel_to_rad_s(-100);
        assert!((pos + neg).abs() < 1e-12);
    }

    #[test]
    fn vel_proportional() {
        // 2倍の生値 → 2倍のrad/s
        let v1 = raw_vel_to_rad_s(50);
        let v2 = raw_vel_to_rad_s(100);
        assert!((v2 / v1 - 2.0).abs() < 1e-12);
    }

    // ─── 電流変換 raw_current_to_a ────────────────────────────────────────

    #[test]
    fn current_zero_is_zero() {
        assert_eq!(raw_current_to_a(0), 0.0);
    }

    #[test]
    fn current_1lsb_is_2p69ma() {
        let result = raw_current_to_a(1);
        assert!((result - 0.00269).abs() < 1e-12);
    }

    #[test]
    fn current_sign_symmetric() {
        let pos = raw_current_to_a(500);
        let neg = raw_current_to_a(-500);
        assert!((pos + neg).abs() < 1e-12);
    }

    // ─── 減速比・オフセット・方向の変換計算 ──────────────────────────────
    // 公式: motor_rad = (urdf_rad + offset) * direction * reduction
    //       urdf_rad  = ticks_to_rad_ext(ticks) / direction / reduction - offset

    fn to_motor(urdf: f64, offset: f64, dir: f64, red: f64) -> f64 {
        (urdf + offset) * dir * red
    }

    fn from_motor(motor: f64, offset: f64, dir: f64, red: f64) -> f64 {
        motor / dir / red - offset
    }

    #[test]
    fn joint1_red4_zero_maps_to_zero() {
        // joint1: offset=0, dir=1, red=4
        assert!((to_motor(0.0, 0.0, 1.0, 4.0) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn joint1_red4_one_rad_maps_to_4rad() {
        assert!((to_motor(1.0, 0.0, 1.0, 4.0) - 4.0).abs() < 1e-12);
    }

    #[test]
    fn joint2_red10_zero_maps_to_zero() {
        assert!((to_motor(0.0, 0.0, 1.0, 10.0) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn joint2_red10_lower_limit() {
        // joint2 lower=-6.28 → motor = -62.8 rad
        let motor = to_motor(-6.28, 0.0, 1.0, 10.0);
        assert!((motor - (-62.8)).abs() < 1e-9, "motor={}", motor);
    }

    #[test]
    fn joint3_offset_red10_zero() {
        // joint3: offset=-2.4222, red=10, dir=1
        // URDF=0 → motor = (0 + (-2.4222)) * 1 * 10 = -24.222
        let motor = to_motor(0.0, -2.4222, 1.0, 10.0);
        assert!((motor - (-24.222)).abs() < 1e-9, "motor={}", motor);
    }

    #[test]
    fn joint3_zero_urdf_produces_large_negative_ticks() {
        // joint3 URDF=0 → motor=-24.222 rad → 大きな負のtick値
        let motor = to_motor(0.0, -2.4222, 1.0, 10.0);
        let ticks = rad_to_ticks_ext(motor);
        let expected = (motor / (2.0 * PI) * COUNTS + HALF).round() as i32;
        println!("[joint3 zero] motor={:.3}rad ticks={} expected={}", motor, ticks, expected);
        assert_eq!(ticks, expected);
        assert!(ticks < -10000, "joint3 URDF=0 must produce large negative ticks (got {})", ticks);
    }

    #[test]
    fn all_6_joints_roundtrip_urdf_to_ticks_and_back() {
        // 実際のパラメータ設定で全関節のラウンドトリップを確認
        // arm_gripper_driver.yaml の値
        let offsets    = [0.0,     0.0,     -2.4222, 0.0, 0.0, 0.0];
        let directions = [1.0_f64; 6];
        let reductions = [4.0,    10.0,     10.0,    1.0, 1.0, 1.0];

        // joint2はupper=0なので負の値のみ, 他は±1.0
        let test_angles: &[&[f64]] = &[
            &[-1.0, -0.5, 0.0, 0.5],      // j1
            &[-1.0, -0.5, 0.0],            // j2 (upper=0)
            &[-1.0, -0.5, 0.0, 0.5],      // j3
            &[-1.0, -0.5, 0.0, 0.5],      // j4
            &[-1.0, -0.5, 0.0, 0.5],      // j5
            &[-1.0, -0.5, 0.0, 0.5],      // j6
        ];

        for j in 0..6 {
            let off = offsets[j];
            let dir = directions[j];
            let red = reductions[j];
            let max_err = 2.0 * PI / (COUNTS * red); // 減速比によって分解能向上

            for &urdf in test_angles[j] {
                let motor = to_motor(urdf, off, dir, red);
                let ticks = rad_to_ticks_ext(motor);
                let motor_back = ticks_to_rad_ext(ticks);
                let urdf_back = from_motor(motor_back, off, dir, red);
                let err = (urdf_back - urdf).abs();
                println!("[j{} red={:.0}] URDF={:.3} → ticks={} → back={:.6} err={:.8}",
                         j + 1, red, urdf, ticks, urdf_back, err);
                assert!(
                    err <= max_err + 1e-12,
                    "joint{} roundtrip error {:.8} exceeds max {:.8} (red={})",
                    j + 1, err, max_err, red
                );
            }
        }
    }

    #[test]
    fn resolution_improves_with_reduction() {
        // 減速比が大きいほど角度分解能が高い
        let res_direct = 2.0 * PI / COUNTS;       // reduction=1: ≈ 0.001534 rad/tick
        let res_red4   = 2.0 * PI / (COUNTS * 4.0);  // joint1: ≈ 0.000383 rad/tick
        let res_red10  = 2.0 * PI / (COUNTS * 10.0); // joint2,3: ≈ 0.000153 rad/tick
        println!("[resolution] direct={:.6} red4={:.6} red10={:.6}", res_direct, res_red4, res_red10);
        assert!(res_red4 < res_direct,  "reduction=4 should give finer resolution");
        assert!(res_red10 < res_red4,   "reduction=10 should give finer resolution than 4");
        assert!(res_red10 < 0.0002,     "joint2,3 resolution < 0.2 mrad");
    }

    #[test]
    fn direction_inversion_flips_sign() {
        // direction=-1 のとき ticks が逆方向になる
        let urdf = 1.0_f64;
        let ticks_pos = rad_to_ticks_ext(to_motor(urdf, 0.0,  1.0, 1.0));
        let ticks_neg = rad_to_ticks_ext(to_motor(urdf, 0.0, -1.0, 1.0));
        // center = 2048 → 2048 + delta と 2048 - delta
        let delta_pos = ticks_pos - 2048;
        let delta_neg = ticks_neg - 2048;
        println!("[dir invert] ticks_pos={} ticks_neg={}", ticks_pos, ticks_neg);
        assert_eq!(delta_pos, -delta_neg, "direction inversion must flip tick delta");
    }

    #[test]
    fn gripper_mirror_symmetric() {
        // グリッパー: directions=[-1, +1] で同じURDF角度がミラー対称なtickに
        let urdf = 0.5_f64;
        let offset = 0.0;
        let t0 = rad_to_ticks_ext(to_motor(urdf, offset, -1.0, 1.0));
        let t1 = rad_to_ticks_ext(to_motor(urdf, offset,  1.0, 1.0));
        let d0 = t0 - 2048;
        let d1 = t1 - 2048;
        println!("[gripper mirror] t0={} t1={} d0={} d1={}", t0, t1, d0, d1);
        assert_eq!(d0, -d1, "gripper motors must be mirror-symmetric in ticks");
    }

    #[test]
    fn write_read_logical_roundtrip_all_joints() {
        // write_arm_positions と read_arm_positions のロジックを関数レベルで確認
        // (ハードウェアなし: ticks変換のみを検証)
        let offsets    = [0.0,  0.0,  -2.4222, 0.0, 0.0, 0.0];
        let directions = [1.0_f64; 6];
        let reductions = [4.0, 10.0,  10.0,    1.0, 1.0, 1.0];

        let input_positions = [-0.5_f64, -0.5, 0.3, -0.3, 0.5, 0.2];

        // write: urdf → ticks
        let ticks: Vec<i32> = (0..6).map(|i| {
            rad_to_ticks_ext((input_positions[i] + offsets[i]) * directions[i] * reductions[i])
        }).collect();

        // read: ticks → urdf
        let recovered: Vec<f64> = (0..6).map(|i| {
            ticks_to_rad_ext(ticks[i]) / directions[i] / reductions[i] - offsets[i]
        }).collect();

        for i in 0..6 {
            let err = (recovered[i] - input_positions[i]).abs();
            let max_err = 2.0 * PI / (COUNTS * reductions[i]);
            println!("[write→read j{}] in={:.4} out={:.6} err={:.8} max={:.8}",
                     i + 1, input_positions[i], recovered[i], err, max_err);
            assert!(err <= max_err + 1e-12,
                    "joint{} write→read error {:.8} exceeds {:.8}", i + 1, err, max_err);
        }
    }

    #[test]
    fn joint3_offset_effect_on_zero_position() {
        // offset=-2.4222があることでURDF 0 rad ≠ DXL tick 2048 になることを確認
        // (offset=0のjoint1との比較)
        let ticks_j1_zero = rad_to_ticks_ext(to_motor(0.0, 0.0,    1.0, 4.0));
        let ticks_j3_zero = rad_to_ticks_ext(to_motor(0.0, -2.4222, 1.0, 10.0));
        println!("[offset effect] j1 URDF=0 → ticks={}", ticks_j1_zero);
        println!("[offset effect] j3 URDF=0 → ticks={}", ticks_j3_zero);
        // j1のゼロはDXL center (2048), j3のゼロはずれている
        assert_eq!(ticks_j1_zero, 2048, "joint1 URDF=0 must be DXL center tick");
        assert_ne!(ticks_j3_zero, 2048, "joint3 URDF=0 must NOT be DXL center tick due to offset");
    }
}
