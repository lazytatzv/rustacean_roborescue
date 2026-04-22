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
const ADDR_HARDWARE_ERROR_STATUS: u16 = 70; // 1 byte
const ADDR_CURRENT_LIMIT: u16 = 38; // 2 bytes (EEPROM)
const ADDR_PRESENT_TEMPERATURE: u16 = 146; // 1 byte

/// Position Control Mode (0 ~ 4095)
const MODE_POSITION_CONTROL: u8 = 3;
/// Extended Position Control Mode (-1,048,575 ~ 1,048,575)
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

pub fn rad_to_ticks(rad: f64) -> i32 {
    let ticks = (rad / (2.0 * std::f64::consts::PI) * COUNTS_PER_REV) + (COUNTS_PER_REV / 2.0);
    ticks.round() as i32
}

pub fn ticks_to_rad(ticks: i32) -> f64 {
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
    /// arm_directions[i]: +1.0 or -1.0. physical_rad = (urdf_rad + offset) * direction * gear_ratio
    /// ウォームギア等でURDF関節方向とサーボ回転方向が逆の場合は -1.0 を設定する。
    arm_directions: Vec<f64>,
    /// gripper_directions[i]: +1.0 or -1.0. physical_rad = (urdf_rad + offset) * direction
    gripper_directions: Vec<f64>,
    /// per-joint gear ratio: physical_motor_rad = joint_rad * gear_ratio
    arm_gear_ratios: Vec<f64>,
    /// per-joint offset [rad]: physical_rad = (urdf_rad + offset) * direction * gear_ratio
    /// URDF 0 rad が Dynamixel tick 2048 と一致しない場合に設定する。
    /// 正値: URDF ゼロ時にモーターが正方向にずれている
    arm_offsets: Vec<f64>,
    gripper_offsets: Vec<f64>,
}

pub struct MotorStatus {
    pub position_rad: f64,
    pub position_ticks: i32,
    pub velocity_rad_s: f64,
    pub current_a: f64,
    pub temperature_c: u8,
    pub hardware_error: u8,
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
        arm_gear_ratios: Vec<f64>,
        arm_offsets: Vec<f64>,
        gripper_offsets: Vec<f64>,
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
        let mut a_ratio = arm_gear_ratios;
        a_ratio.resize(arm_ids.len(), 1.0);
        Ok(Self {
            bus,
            arm_ids,
            gripper_ids,
            arm_directions: a_dir,
            gripper_directions: g_dir,
            arm_gear_ratios: a_ratio,
            arm_offsets: arm_off,
            gripper_offsets: g_off,
        })
    }

    pub fn ping(&mut self, id: u8) -> Result<()> {
        self.bus
            .ping(id)
            .map(|_| ())
            .map_err(|e| anyhow::anyhow!("Ping ID {} failed: {:?}", id, e))
    }

    pub fn init_motors(&mut self, profile_velocity: u32, gripper_max_current: u16) -> Result<()> {
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

        // Set Modes
        for &id in &self.arm_ids {
            // High gear ratio motors or those needing multi-turn should use Extended Position Mode
            self.bus
                .write_u8(id, ADDR_OPERATING_MODE, MODE_EXTENDED_POSITION)
                .map_err(|e| {
                    anyhow::anyhow!(
                        "ID {id} set operating_mode={MODE_EXTENDED_POSITION} failed: {e:?}"
                    )
                })?;
        }
        for &id in &self.gripper_ids {
            println!("⚙️  arm_driver: Setting Gripper ID {id} to Current-based Position Mode...");
            self.bus
                .write_u8(id, ADDR_OPERATING_MODE, MODE_CURRENT_BASED_POSITION)
                .map_err(|e| anyhow::anyhow!("gripper ID {id} set operating_mode={MODE_CURRENT_BASED_POSITION} failed: {e:?}"))?;
        }

        // Configs
        for &id in &self.arm_ids {
            self.bus
                .write_u32(id, ADDR_PROFILE_VELOCITY, profile_velocity)
                .map_err(|e| {
                    anyhow::anyhow!("ID {id} set profile_velocity={profile_velocity} failed: {e:?}")
                })?;
        }
        for &id in &self.gripper_ids {
            // Current Limit (EEPROM) を読んでクランプ: Dynamixel Wizard で下げている場合にも対応
            let effective_current = match self.bus.read_u16(id, ADDR_CURRENT_LIMIT) {
                Ok(resp) => {
                    let limit = resp.data;
                    if gripper_max_current > limit {
                        eprintln!(
                            "⚠️  gripper ID {id}: gripper_max_current={gripper_max_current} > Current_Limit={limit}, clamping"
                        );
                        limit
                    } else {
                        gripper_max_current
                    }
                }
                Err(e) => {
                    eprintln!("⚠️  gripper ID {id}: failed to read Current_Limit ({e:?}), using {gripper_max_current}");
                    gripper_max_current
                }
            };
            self.bus
                .write_u16(id, ADDR_GOAL_CURRENT, effective_current)
                .map_err(|e| {
                    anyhow::anyhow!(
                        "gripper ID {id} set goal_current={effective_current} failed: {e:?}"
                    )
                })?;
        }

        // トルクON前に現在位置をゴールとして書き込む。
        for &id in &self.arm_ids {
            if let Ok(resp) = self.bus.read_u32(id, ADDR_PRESENT_POSITION) {
                let _ = self.bus.write_u32(id, ADDR_GOAL_POSITION, resp.data);
            }
        }
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
        // physical_rad = (urdf_rad + offset) * direction * gear_ratio
        let commands: Vec<SyncWriteData<u32>> = self
            .arm_ids
            .iter()
            .zip(positions_rad.iter())
            .zip(self.arm_offsets.iter())
            .zip(self.arm_directions.iter())
            .zip(self.arm_gear_ratios.iter())
            .enumerate()
            .map(|(i, ((((&id, &rad), &offset), &dir), &ratio))| {
                let ticks = rad_to_ticks((rad + offset) * dir * ratio);
                // --- DEBUG: Print math once every ~5 seconds (at 50Hz) ---
                if id == self.arm_ids[0] && (ticks % 100 == 0) { // Slight throttle logic
                     // Only print for J1 to keep it clean, or use a proper counter
                }
                SyncWriteData {
                    motor_id: id,
                    data: ticks as u32,
                }
            })
            .collect();
        
        // Simple periodic debug summary
        static mut COUNT: u64 = 0;
        unsafe {
            COUNT += 1;
            if COUNT % 250 == 0 {
                println!("🛠️ [Arm Math Check] URDF[J1]={:.3}rad -> Ratio:{:.1}*Dir:{:.1} -> Ticks:{}", 
                    positions_rad[0], self.arm_gear_ratios[0], self.arm_directions[0], rad_to_ticks((positions_rad[0] + self.arm_offsets[0]) * self.arm_directions[0] * self.arm_gear_ratios[0]));
            }
        }

        self.bus
            .sync_write_u32(ADDR_GOAL_POSITION, &commands)
            .map_err(|e| anyhow::anyhow!("{:?}", e))
    }

    pub fn write_gripper_position(&mut self, rad: f64) -> Result<()> {
        let mut last_err = None;
        for (i, &id) in self.gripper_ids.iter().enumerate() {
            let offset = self.gripper_offsets.get(i).copied().unwrap_or(0.0);
            let dir = self.gripper_directions.get(i).copied().unwrap_or(1.0);
            if let Err(e) = self.bus.write_u32(id, ADDR_GOAL_POSITION, rad_to_ticks((rad + offset) * dir) as u32) {
                last_err = Some(anyhow::anyhow!("gripper ID {id}: {:?}", e));
            }
        }
        match last_err {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    pub fn read_arm_positions(&mut self) -> Result<Vec<f64>> {
        let resp = self
            .bus
            .sync_read_u32(&self.arm_ids, ADDR_PRESENT_POSITION)
            .map_err(|e| anyhow::anyhow!("{:?}", e))?;
        // urdf_rad = physical_rad / (dir * ratio) - offset
        Ok(resp
            .iter()
            .zip(self.arm_offsets.iter())
            .zip(self.arm_directions.iter())
            .zip(self.arm_gear_ratios.iter())
            .map(|(((r, &offset), &dir), &ratio)| {
                ticks_to_rad(r.data as i32) / (dir * ratio) - offset
            })
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

            let pos = self.bus.read_u32(id, ADDR_PRESENT_POSITION)?;
            let vel = self.bus.read_u32(id, ADDR_PRESENT_VELOCITY)?;
            let cur = self.bus.read_u16(id, ADDR_PRESENT_CURRENT)?;
            let tmp = self.bus.read_u8(id, ADDR_PRESENT_TEMPERATURE)?;
            let err = self.bus.read_u8(id, ADDR_HARDWARE_ERROR_STATUS)?;

            statuses.push(MotorStatus {
                position_rad: ticks_to_rad(pos.data as i32) / dir - offset,
                position_ticks: pos.data as i32,
                velocity_rad_s: raw_vel_to_rad_s(vel.data as i32) / dir,
                current_a: raw_current_to_a(cur.data as i16),
                temperature_c: tmp.data,
                hardware_error: err.data,
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
            let ticks  = rad_to_ticks((status.position_rad + offset) * dir);
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
        println!("🪫 arm_driver: All motor torques disabled.");
    }
}

impl Drop for ArmDynamixelDriver {
    fn drop(&mut self) {
        // Fail-safe: Ensure torque is OFF when the driver is dropped
        self.torque_off_all();
    }
}
