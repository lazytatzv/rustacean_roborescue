use anyhow::{Context, Result};
use dynamixel2::{Bus, instructions::SyncWriteData};
// 修正1: dynamixel2 経由で serial2 を使う
use dynamixel2::serial2::SerialPort;

// XM540 Control Table Addresses
const ADDR_OPERATING_MODE: u16 = 11;
const ADDR_TORQUE_ENABLE: u16 = 64;
const ADDR_GOAL_VELOCITY: u16 = 104;
const ADDR_PRESENT_POSITION: u16 = 132;

const MODE_VELOCITY_CONTROL: u8 = 1;

pub struct DynamixelDriver {
    // 修正2: Bus の型定義をコンパイラの指示通りに修正
    // (SerialPort ではなく、内部バッファの型を指定)
    bus: Bus<Vec<u8>, Vec<u8>>, 
    ids: Vec<u8>,
}

impl DynamixelDriver {
    pub fn new(port_name: &str, baud_rate: u32, ids: Vec<u8>) -> Result<Self> {
        let port = SerialPort::open(port_name, baud_rate)
            .context("Failed to open serial port")?;
        
        // Bus::new(port) は Result<Bus<Vec<u8>, Vec<u8>>> を返します
        let bus = Bus::new(port); 
        
        Ok(Self { bus: bus?, ids })
    }

    pub fn init_velocity_mode(&mut self) -> Result<()> {
        for &id in &self.ids {
            let _ = self.bus.write_u8(id, ADDR_TORQUE_ENABLE, 0);
        }

        for &id in &self.ids {
            self.bus.write_u8(id, ADDR_OPERATING_MODE, MODE_VELOCITY_CONTROL)
                .map_err(|e| anyhow::anyhow!("Failed to set Velocity Mode for ID {}: {:?}", id, e))?;
        }

        for &id in &self.ids {
            self.bus.write_u8(id, ADDR_TORQUE_ENABLE, 1)
                .map_err(|e| anyhow::anyhow!("Failed to enable torque for ID {}: {:?}", id, e))?;
        }

        println!("Dynamixel initialized in VELOCITY CONTROL mode.");
        Ok(())
    }

    pub fn write_velocities(&mut self, velocities: &[i32]) -> Result<()> {
        if velocities.len() != self.ids.len() {
            return Err(anyhow::anyhow!("Velocity count does not match motor count"));
        }

        let mut commands = Vec::with_capacity(self.ids.len());
        for (i, &id) in self.ids.iter().enumerate() {
            commands.push(SyncWriteData {
                motor_id: id,
                data: velocities[i] as u32,
            });
        }

        self.bus.sync_write_u32(ADDR_GOAL_VELOCITY, &commands)
            .map_err(|e| anyhow::anyhow!("Sync write failed: {:?}", e))?;
        Ok(())
    }

    pub fn read_positions(&mut self) -> Result<Vec<i32>> {
        let responses = self.bus.sync_read_u32(&self.ids, ADDR_PRESENT_POSITION)
            .map_err(|e| anyhow::anyhow!("Sync read failed: {:?}", e))?;

        let mut positions = Vec::new();
        for response in responses {
             positions.push(response.data as i32);
        }

        Ok(positions)
    }
}
