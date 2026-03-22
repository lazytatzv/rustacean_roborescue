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
const ADDR_PRESENT_TEMPERATURE: u16 = 146; // 1 byte

/// Position Control Mode
const MODE_POSITION_CONTROL: u8 = 3;
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

pub fn rad_to_ticks(rad: f64) -> u32 {
    let ticks = (rad / (2.0 * std::f64::consts::PI) * COUNTS_PER_REV) + (COUNTS_PER_REV / 2.0);
    (ticks.round() as i64).clamp(0, 4095) as u32
}

pub fn ticks_to_rad(ticks: u32) -> f64 {
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
    gripper_id: u8,
}

pub struct MotorStatus {
    pub position_rad: f64,
    pub velocity_rad_s: f64,
    pub current_a: f64,
    pub temperature_c: u8,
}

impl ArmDynamixelDriver {
    pub fn new(port_name: &str, baud_rate: u32, arm_ids: Vec<u8>, gripper_id: u8) -> Result<Self> {
        let port = SerialPort::open(port_name, baud_rate).context("Failed to open serial port")?;
        let bus = Bus::new(port)?;
        Ok(Self {
            bus,
            arm_ids,
            gripper_id,
        })
    }

    pub fn ping(&mut self, id: u8) -> Result<()> {
        self.bus
            .ping(id)
            .map(|_| ())
            .map_err(|e| anyhow::anyhow!("Ping ID {} failed: {:?}", id, e))
    }

    pub fn init_motors(&mut self, profile_velocity: u32, gripper_max_current: u16) -> Result<()> {
        // Torque OFF
        for &id in &self.arm_ids {
            let _ = self.bus.write_u8(id, ADDR_TORQUE_ENABLE, 0);
        }
        let _ = self.bus.write_u8(self.gripper_id, ADDR_TORQUE_ENABLE, 0);

        // Set Modes
        for &id in &self.arm_ids {
            self.bus
                .write_u8(id, ADDR_OPERATING_MODE, MODE_POSITION_CONTROL)?;
        }
        self.bus.write_u8(
            self.gripper_id,
            ADDR_OPERATING_MODE,
            MODE_CURRENT_BASED_POSITION,
        )?;

        // Configs
        for &id in &self.arm_ids {
            self.bus
                .write_u32(id, ADDR_PROFILE_VELOCITY, profile_velocity)?;
        }
        self.bus
            .write_u16(self.gripper_id, ADDR_GOAL_CURRENT, gripper_max_current)?;

        // Torque ON
        for &id in &self.arm_ids {
            self.bus.write_u8(id, ADDR_TORQUE_ENABLE, 1)?;
        }
        self.bus.write_u8(self.gripper_id, ADDR_TORQUE_ENABLE, 1)?;

        Ok(())
    }

    pub fn write_arm_positions(&mut self, positions_rad: &[f64]) -> Result<()> {
        let commands: Vec<SyncWriteData<u32>> = self
            .arm_ids
            .iter()
            .zip(positions_rad.iter())
            .map(|(&id, &rad)| SyncWriteData {
                motor_id: id,
                data: rad_to_ticks(rad),
            })
            .collect();
        self.bus
            .sync_write_u32(ADDR_GOAL_POSITION, &commands)
            .map_err(|e| anyhow::anyhow!("{:?}", e))
    }

    pub fn write_gripper_position(&mut self, rad: f64) -> Result<()> {
        self.bus
            .write_u32(self.gripper_id, ADDR_GOAL_POSITION, rad_to_ticks(rad))
            .map(|_| ())
            .map_err(|e| anyhow::anyhow!("{:?}", e))
    }

    pub fn read_arm_positions(&mut self) -> Result<Vec<f64>> {
        let resp = self
            .bus
            .sync_read_u32(&self.arm_ids, ADDR_PRESENT_POSITION)
            .map_err(|e| anyhow::anyhow!("{:?}", e))?;
        Ok(resp.iter().map(|r| ticks_to_rad(r.data)).collect())
    }

    pub fn read_gripper_status(&mut self) -> Result<MotorStatus> {
        let id = self.gripper_id;
        let pos = self.bus.read_u32(id, ADDR_PRESENT_POSITION)?;
        let vel = self.bus.read_u32(id, ADDR_PRESENT_VELOCITY)?;
        let cur = self.bus.read_u16(id, ADDR_PRESENT_CURRENT)?;
        let tmp = self.bus.read_u8(id, ADDR_PRESENT_TEMPERATURE)?;
        Ok(MotorStatus {
            position_rad: ticks_to_rad(pos.data),
            velocity_rad_s: raw_vel_to_rad_s(vel.data as i32),
            current_a: raw_current_to_a(cur.data as i16),
            temperature_c: tmp.data,
        })
    }

    pub fn emergency_stop(&mut self) {
        self.torque_off_all();
    }

    pub fn check_temperatures(&mut self) -> (Vec<(u8, u8)>, Vec<(u8, u8)>) {
        let mut warnings = Vec::new();
        let mut critical = Vec::new();
        let mut all_ids = self.arm_ids.clone();
        all_ids.push(self.gripper_id);
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
        let _ = self.bus.write_u8(self.gripper_id, ADDR_TORQUE_ENABLE, 0);
    }
}
