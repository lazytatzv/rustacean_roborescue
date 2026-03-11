#![allow(dead_code)]

use anyhow::{Context, Result};
use dynamixel2::serial2::SerialPort;
use dynamixel2::{instructions::SyncWriteData, Bus};

// ═══════════════════════════════════════════════════════════════════════════
//  Dynamixel XM series — Control Table Addresses
// ═══════════════════════════════════════════════════════════════════════════

const ADDR_OPERATING_MODE: u16 = 11; // 1 byte
const ADDR_TORQUE_ENABLE: u16 = 64; // 1 byte
const ADDR_PROFILE_VELOCITY: u16 = 112; // 4 bytes — limits speed during position move
const ADDR_GOAL_POSITION: u16 = 116; // 4 bytes
const ADDR_PRESENT_CURRENT: u16 = 126; // 2 bytes (signed)
const ADDR_PRESENT_VELOCITY: u16 = 128; // 4 bytes (signed)
const ADDR_PRESENT_POSITION: u16 = 132; // 4 bytes
const ADDR_PRESENT_TEMPERATURE: u16 = 146; // 1 byte

/// Position Control Mode
const MODE_POSITION_CONTROL: u8 = 3;

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

/// Radians → Dynamixel position ticks.
/// XM series: 0~4095, center=2048 corresponds to 0 rad.
/// Full range: -π ~ +π → 0 ~ 4095
pub fn rad_to_ticks(rad: f64) -> u32 {
    let ticks = (rad / (2.0 * std::f64::consts::PI) * COUNTS_PER_REV) + (COUNTS_PER_REV / 2.0);
    (ticks.round() as i64).clamp(0, 4095) as u32
}

/// Dynamixel position ticks → radians.
pub fn ticks_to_rad(ticks: u32) -> f64 {
    (ticks as f64 - COUNTS_PER_REV / 2.0) / COUNTS_PER_REV * 2.0 * std::f64::consts::PI
}

/// Dynamixel velocity (signed 32-bit) → rad/s.
/// Raw unit: 0.229 rev/min per LSB
pub fn raw_vel_to_rad_s(raw: i32) -> f64 {
    raw as f64 * VEL_UNIT_RPM * 2.0 * std::f64::consts::PI / 60.0
}

/// Dynamixel current (signed 16-bit) → Amperes.
/// XM series: 2.69 mA per LSB
pub fn raw_current_to_a(raw: i16) -> f64 {
    raw as f64 * 2.69e-3
}

// ═══════════════════════════════════════════════════════════════════════════
//  Driver
// ═══════════════════════════════════════════════════════════════════════════

pub struct ArmDynamixelDriver {
    bus: Bus<Vec<u8>, Vec<u8>>,
    ids: Vec<u8>,
}

/// Per-motor status snapshot
pub struct MotorStatus {
    pub position_rad: f64,
    pub velocity_rad_s: f64,
    pub current_a: f64,
    pub temperature_c: u8,
}

impl ArmDynamixelDriver {
    pub fn new(port_name: &str, baud_rate: u32, ids: Vec<u8>) -> Result<Self> {
        let port =
            SerialPort::open(port_name, baud_rate).context("Failed to open serial port")?;
        let bus = Bus::new(port)?;
        Ok(Self { bus, ids })
    }

    /// Ping all motors and confirm communication
    pub fn ping_all(&mut self) -> Result<()> {
        for &id in &self.ids {
            self.bus
                .ping(id)
                .map_err(|e| anyhow::anyhow!("Ping motor ID {} failed: {:?}", id, e))?;
        }
        Ok(())
    }

    /// Initialize all motors in Position Control Mode.
    ///
    /// Sequence: torque off → set mode → set profile velocity → torque on
    pub fn init_position_mode(&mut self, profile_velocity: u32) -> Result<()> {
        // Torque OFF (required before mode change)
        for &id in &self.ids {
            let _ = self.bus.write_u8(id, ADDR_TORQUE_ENABLE, 0);
        }

        // Set Position Control Mode
        for &id in &self.ids {
            self.bus
                .write_u8(id, ADDR_OPERATING_MODE, MODE_POSITION_CONTROL)
                .map_err(|e| {
                    anyhow::anyhow!("Set Position Mode for ID {} failed: {:?}", id, e)
                })?;
        }

        // Set profile velocity (limits max joint speed during position move)
        for &id in &self.ids {
            self.bus
                .write_u32(id, ADDR_PROFILE_VELOCITY, profile_velocity)
                .map_err(|e| {
                    anyhow::anyhow!("Set Profile Velocity for ID {} failed: {:?}", id, e)
                })?;
        }

        // Torque ON
        for &id in &self.ids {
            self.bus
                .write_u8(id, ADDR_TORQUE_ENABLE, 1)
                .map_err(|e| anyhow::anyhow!("Torque ON for ID {} failed: {:?}", id, e))?;
        }

        Ok(())
    }

    /// Write goal positions for all joints in one SyncWrite.
    /// `positions_rad` must be in the same order as `self.ids`.
    pub fn write_positions(&mut self, positions_rad: &[f64]) -> Result<()> {
        if positions_rad.len() != self.ids.len() {
            anyhow::bail!(
                "Position count ({}) != motor count ({})",
                positions_rad.len(),
                self.ids.len()
            );
        }

        let commands: Vec<SyncWriteData<u32>> = self
            .ids
            .iter()
            .zip(positions_rad.iter())
            .map(|(&id, &rad)| SyncWriteData {
                motor_id: id,
                data: rad_to_ticks(rad),
            })
            .collect();

        self.bus
            .sync_write_u32(ADDR_GOAL_POSITION, &commands)
            .map_err(|e| anyhow::anyhow!("SyncWrite goal positions failed: {:?}", e))?;
        Ok(())
    }

    /// Read present position of all motors via SyncRead.
    pub fn read_positions(&mut self) -> Result<Vec<f64>> {
        let responses = self
            .bus
            .sync_read_u32(&self.ids, ADDR_PRESENT_POSITION)
            .map_err(|e| anyhow::anyhow!("SyncRead positions failed: {:?}", e))?;

        Ok(responses
            .iter()
            .map(|r| ticks_to_rad(r.data))
            .collect())
    }

    /// Read full status (position, velocity, current, temperature) for all motors.
    /// Falls back gracefully on individual read failures.
    pub fn read_full_status(&mut self) -> Vec<Option<MotorStatus>> {
        self.ids
            .iter()
            .map(|&id| {
                let pos = self.bus.read_u32(id, ADDR_PRESENT_POSITION).ok()?;
                let vel = self.bus.read_u32(id, ADDR_PRESENT_VELOCITY).ok()?;
                let cur = self.bus.read_u16(id, ADDR_PRESENT_CURRENT).ok()?;
                let tmp = self.bus.read_u8(id, ADDR_PRESENT_TEMPERATURE).ok()?;

                Some(MotorStatus {
                    position_rad: ticks_to_rad(pos.data),
                    velocity_rad_s: raw_vel_to_rad_s(vel.data as i32),
                    current_a: raw_current_to_a(cur.data as i16),
                    temperature_c: tmp.data,
                })
            })
            .collect()
    }

    /// Emergency stop: disable torque on all motors (immediate stop, gravity will
    /// cause the arm to fall — but this is safer than continuing at max speed).
    pub fn emergency_stop(&mut self) {
        // Disable torque — motors stop immediately and go to free-wheel state.
        // In a rescue robot context, a falling arm is preferable to a runaway arm.
        self.torque_off_all();
    }

    /// Check temperatures and return (warnings, critical) for each motor
    pub fn check_temperatures(&mut self) -> (Vec<(u8, u8)>, Vec<(u8, u8)>) {
        let mut warnings = Vec::new();
        let mut critical = Vec::new();
        for &id in &self.ids {
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

    /// Disable torque on all motors (safe shutdown)
    pub fn torque_off_all(&mut self) {
        for &id in &self.ids {
            let _ = self.bus.write_u8(id, ADDR_TORQUE_ENABLE, 0);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Unit Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_rad_to_ticks_center() {
        // 0 rad should map to center (2048)
        assert_eq!(rad_to_ticks(0.0), 2048);
    }

    #[test]
    fn test_rad_to_ticks_positive_pi() {
        // +π should map to 4095
        assert_eq!(rad_to_ticks(PI), 4095);
    }

    #[test]
    fn test_rad_to_ticks_negative_pi() {
        // -π should map to 0
        assert_eq!(rad_to_ticks(-PI), 0);
    }

    #[test]
    fn test_ticks_to_rad_center() {
        let rad = ticks_to_rad(2048);
        assert!(rad.abs() < 1e-6, "expected ~0.0, got {rad}");
    }

    #[test]
    fn test_roundtrip_rad_ticks() {
        // Roundtrip: rad → ticks → rad should be close to original
        for angle_deg in [-170, -90, -45, 0, 45, 90, 170] {
            let rad = (angle_deg as f64).to_radians();
            let ticks = rad_to_ticks(rad);
            let rad_back = ticks_to_rad(ticks);
            let error = (rad - rad_back).abs();
            assert!(
                error < 0.002, // ~0.1° resolution
                "Roundtrip error for {angle_deg}°: {error:.6} rad"
            );
        }
    }

    #[test]
    fn test_rad_to_ticks_clamp() {
        // Beyond ±π should clamp to 0..4095
        assert_eq!(rad_to_ticks(-10.0), 0);
        assert_eq!(rad_to_ticks(10.0), 4095);
    }

    #[test]
    fn test_raw_vel_to_rad_s_zero() {
        assert!((raw_vel_to_rad_s(0) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_raw_current_to_a_zero() {
        assert!((raw_current_to_a(0) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_raw_current_to_a_positive() {
        // 1000 LSB × 2.69 mA = 2.69 A
        let amps = raw_current_to_a(1000);
        assert!((amps - 2.69).abs() < 1e-6);
    }
}
