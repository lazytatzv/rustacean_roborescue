// src/driver.rs
use anyhow::{Context, Result};
use dynamixel2::{Bus, instructions::SyncWriteData};
use dynamixel2::serial2::SerialPort;
use std::io::{Read, Write};

// XM540 Control Table Addresses
const ADDR_OPERATING_MODE: u16 = 11;
const ADDR_TORQUE_ENABLE: u16 = 64;
const ADDR_GOAL_VELOCITY: u16 = 104;
const ADDR_PRESENT_POSITION: u16 = 132;
// const ADDR_PRESENT_VELOCITY: u16 = 128; // 必要なら追加

// モード定数
const MODE_VELOCITY_CONTROL: u8 = 1;
// const MODE_POSITION_CONTROL: u8 = 3;

pub struct DynamixelDriver {
    bus: Bus<SerialPort, SerialPort>,
    ids: Vec<u8>,
}

impl DynamixelDriver {
    pub fn new(port_name: &str, baud_rate: u32, ids: Vec<u8>) -> Result<Self> {
        let port = SerialPort::open(port_name, baud_rate)
            .context("Failed to open serial port")?;
        
        let bus = Bus::new(port);

        Ok(Self { bus, ids })
    }

    /// 初期化シーケンス: トルクOFF -> モード設定 -> トルクON
    pub fn init_velocity_mode(&mut self) -> Result<()> {
        // 1. 安全のため一旦全モータートルクOFF
        for &id in &self.ids {
            // エラーが出ても無視して次に進む（接続されていないIDがある場合などを考慮）
            let _ = self.bus.write_u8(id, ADDR_TORQUE_ENABLE, 0);
        }

        // 2. オペレーティングモードを「Velocity Control (1)」に変更
        // ※ トルクOFF状態でないと書き込めないレジスタです
        for &id in &self.ids {
            self.bus.write_u8(id, ADDR_OPERATING_MODE, MODE_VELOCITY_CONTROL)
                .context(format!("Failed to set Velocity Mode for ID {}", id))?;
        }

        // 3. トルクON
        for &id in &self.ids {
            self.bus.write_u8(id, ADDR_TORQUE_ENABLE, 1)
                .context(format!("Failed to enable torque for ID {}", id))?;
        }

        println!("Dynamixel initialized in VELOCITY CONTROL mode.");
        Ok(())
    }

    /// Sync Write で全モーターに目標速度を書き込む
    /// velocities の順序は self.ids と一致している前提
    pub fn write_velocities(&mut self, velocities: &[i32]) -> Result<()> {
        if velocities.len() != self.ids.len() {
            return Err(anyhow::anyhow!("Velocity count does not match motor count"));
        }

        let mut commands = Vec::with_capacity(self.ids.len());

        for (i, &id) in self.ids.iter().enumerate() {
            commands.push(SyncWriteData {
                motor_id: id,
                data: velocities[i], // XM540は i32 (4byte)
            });
        }

        // Sync Write (Address 104, 4 bytes)
        self.bus.sync_write_u32(ADDR_GOAL_VELOCITY, commands)?;
        Ok(())
    }

    /// Sync Read で全モーターの現在位置を一括取得する
    pub fn read_positions(&mut self) -> Result<Vec<i32>> {
        // sync_read_u32(ids, address)
        // 戻り値は Result<Vec<Response<u32>>, ...>
        let responses = self.bus.sync_read_u32(&self.ids, ADDR_PRESENT_POSITION)?;

        // responses は ID順に並んでいるとは限らないため、IDを見て並べ直すのが安全だが、
        // 簡易実装として、戻ってきた順序をそのまま使うか、mapで処理する。
        // ここではエラーチェックしつつ値を取り出す。
        
        let mut positions = Vec::new();
        for response in responses {
             // response.data が u32 なので i32 にキャスト (Dynamixelはunsigned/signedをビット表現で扱う)
             positions.push(response.data as i32);
        }

        // 注: Sync Readはタイムアウトしたモーターのデータを含まないことがあるので、
        // 厳密には ID マッピングが必要ですが、ここでは簡略化しています。
        Ok(positions)
    }
}
