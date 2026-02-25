#![no_std]
#![no_main]

// --- インポート ---
use panic_halt as _;
use core::fmt::{self, Write};
use cortex_m_rt::entry;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;
use stm32f7xx_hal::{
    pac,
    prelude::*,
    spi::{self, Spi},
    serial::{self, Serial},
};

// ==========================================================
// NUCLEO-F767ZI + ICM-20948 SPI通信
// ==========================================================
//
// 配線 (NUCLEO-F767ZI Arduino Header):
//   PA5  = SPI1_SCK   (CN7 D13)
//   PA6  = SPI1_MISO  (CN7 D12)
//   PA7  = SPI1_MOSI  (CN7 D11)
//   PA4  = CS (GPIO)   (CN7 A2)  ← 任意のGPIOでOK
//   VCC  = 3.3V
//   GND  = GND
//
// デバッグ用UART: USART3 (ST-Link VCP)
//   PD8  = USART3_TX
//   PD9  = USART3_RX
//
// ==========================================================

// --- ICM-20948 レジスタ定義 ---
// データシート: https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf

// User Bank 0
const REG_WHO_AM_I: u8       = 0x00; // WHO_AM_I (固定値 0xEA)
const REG_USER_CTRL: u8      = 0x03; // ユーザー制御
const REG_PWR_MGMT_1: u8     = 0x06; // 電源管理1
const REG_PWR_MGMT_2: u8     = 0x07; // 電源管理2
const REG_ACCEL_XOUT_H: u8   = 0x2D; // 加速度X 上位バイト
const REG_GYRO_XOUT_H: u8    = 0x33; // ジャイロX 上位バイト
const REG_TEMP_OUT_H: u8     = 0x39; // 温度 上位バイト
const REG_BANK_SEL: u8       = 0x7F; // ユーザーバンク選択

// User Bank 2
const REG_GYRO_CONFIG_1: u8  = 0x01; // ジャイロ設定
const REG_ACCEL_CONFIG: u8   = 0x14; // 加速度設定

// --- 固定値 ---
const WHO_AM_I_VAL: u8       = 0xEA; // ICM-20948のチップID

// --- SPI制御ビット ---
const SPI_READ: u8           = 0x80; // bit7=1: 読み出し
                                     // bit7=0: 書き込み

// --- 設定値 ---
// PWR_MGMT_1
const PWR_RESET: u8          = 0x80; // bit7: デバイスリセット
const PWR_WAKEUP: u8         = 0x01; // CLKSEL=1: 自動ベストクロック選択
                                     // SLEEP=0, LP_EN=0, TEMP_DIS=0

// PWR_MGMT_2
const PWR2_ALL_ON: u8        = 0x00; // 全軸有効 (加速度3軸 + ジャイロ3軸)

// GYRO_CONFIG_1: ±2000 dps, DLPF有効
const GYRO_FS_2000DPS: u8    = 0x06 | 0x01; // FS_SEL=3 (±2000dps) << 1 | FCHOICE=1
// ACCEL_CONFIG: ±16g, DLPF有効
const ACCEL_FS_16G: u8       = 0x06 | 0x01; // FS_SEL=3 (±16g) << 1 | FCHOICE=1

// --- 感度定数 ---
// ±16g  → 2048 LSB/g
const ACCEL_SENSITIVITY: f32 = 2048.0;
// ±2000 dps → 16.4 LSB/(°/s)
const GYRO_SENSITIVITY: f32  = 16.4;
// 温度: temp_degC = (raw / 333.87) + 21.0
const TEMP_SENSITIVITY: f32  = 333.87;
const TEMP_OFFSET: f32       = 21.0;

// ==========================================================
// UART出力ヘルパー (core::fmt::Write の実装)
// ==========================================================
// stm32f7xx-halのSerialにはcore::fmt::Writeが実装されていないため
// ラッパー構造体を用意する
struct UartWriter<'a, U: serial::Instance, PINS>(&'a mut Serial<U, PINS>);

impl<U: serial::Instance, PINS> fmt::Write for UartWriter<'_, U, PINS> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.bytes() {
            nb::block!(embedded_hal::serial::Write::write(self.0, byte))
                .map_err(|_| fmt::Error)?;
        }
        Ok(())
    }
}

/// UART出力用マクロ
macro_rules! uprint {
    ($serial:expr, $($arg:tt)*) => {
        write!(UartWriter(&mut $serial), $($arg)*).ok()
    };
}
macro_rules! uprintln {
    ($serial:expr) => {
        uprint!($serial, "\r\n")
    };
    ($serial:expr, $($arg:tt)*) => {
        uprint!($serial, $($arg)*);
        uprint!($serial, "\r\n")
    };
}

// ==========================================================
// ICM-20948 SPI ヘルパー関数
// ==========================================================

/// バンク選択 (User Bank 0〜3)
fn icm_select_bank<SPI, CS, E>(
    spi: &mut SPI,
    cs: &mut CS,
    bank: u8,
) where
    SPI: Transfer<u8, Error = E>,
    CS: OutputPin,
{
    // REG_BANK_SEL は全バンク共通 (0x7F)
    // bits[5:4] = bank number
    cs.set_low().ok();
    let mut buf = [REG_BANK_SEL, (bank & 0x03) << 4];
    spi.transfer(&mut buf).ok();
    cs.set_high().ok();
}

/// 1バイト読み出し
fn icm_read_reg<SPI, CS, E>(
    spi: &mut SPI,
    cs: &mut CS,
    reg: u8,
) -> u8
where
    SPI: Transfer<u8, Error = E>,
    CS: OutputPin,
{
    cs.set_low().ok();
    let mut buf = [reg | SPI_READ, 0x00];
    spi.transfer(&mut buf).ok();
    cs.set_high().ok();
    buf[1]
}

/// 1バイト書き込み
fn icm_write_reg<SPI, CS, E>(
    spi: &mut SPI,
    cs: &mut CS,
    reg: u8,
    val: u8,
) where
    SPI: Transfer<u8, Error = E>,
    CS: OutputPin,
{
    cs.set_low().ok();
    let mut buf = [reg & 0x7F, val]; // bit7=0 for write
    spi.transfer(&mut buf).ok();
    cs.set_high().ok();
}

/// 複数バイト連続読み出し (6バイト: 加速度/ジャイロ用)
fn icm_read_6bytes<SPI, CS, E>(
    spi: &mut SPI,
    cs: &mut CS,
    start_reg: u8,
) -> [u8; 6]
where
    SPI: Transfer<u8, Error = E>,
    CS: OutputPin,
{
    cs.set_low().ok();
    // 先頭: レジスタアドレス(READ) + 6バイトのダミー
    let mut buf = [start_reg | SPI_READ, 0, 0, 0, 0, 0, 0];
    spi.transfer(&mut buf).ok();
    cs.set_high().ok();
    [buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]]
}

/// 温度用: 2バイト読み出し
fn icm_read_2bytes<SPI, CS, E>(
    spi: &mut SPI,
    cs: &mut CS,
    start_reg: u8,
) -> [u8; 2]
where
    SPI: Transfer<u8, Error = E>,
    CS: OutputPin,
{
    cs.set_low().ok();
    let mut buf = [start_reg | SPI_READ, 0, 0];
    spi.transfer(&mut buf).ok();
    cs.set_high().ok();
    [buf[1], buf[2]]
}

// ==========================================================
// メインエントリ
// ==========================================================
#[entry]
fn main() -> ! {
    // 1. 周辺機器の取得
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // 2. クロック設定 (216MHz: F767ZIの最大クロック)
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    // 遅延用 (SysTick利用)
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().to_Hz());

    // 3. GPIO設定
    let gpioa = dp.GPIOA.split();
    let gpiod = dp.GPIOD.split();

    // --- SPI1 ピン設定 ---
    // PA5: SCK, PA6: MISO, PA7: MOSI (全てAF5)
    let sck  = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();

    // CS: PA4 (手動制御, プッシュプル出力)
    let mut cs = gpioa.pa4.into_push_pull_output();
    cs.set_high(); // CS初期状態: HIGH (非選択)

    // SPI Mode 0 (CPOL=0, CPHA=0) - ICM-20948対応
    // Mode 3 (CPOL=1, CPHA=1) でも動作可
    let spi_mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition,
    };

    // SPI初期化: 1MHz (安定重視。最大7MHzまで対応)
    let mut spi = Spi::new(dp.SPI1, (sck, miso, mosi))
        .enable::<u8>(spi_mode, 1.MHz(), &clocks, &mut rcc.apb2);

    // --- UART設定 (デバッグ出力: ST-Link VCP) ---
    // PD8: USART3_TX, PD9: USART3_RX
    let tx_pin = gpiod.pd8.into_alternate();
    let rx_pin = gpiod.pd9.into_alternate();

    let serial_config = serial::Config {
        baud_rate: 115_200.bps(),
        ..Default::default()
    };

    let mut serial = Serial::new(dp.USART3, (tx_pin, rx_pin), &clocks, serial_config);

    // =======================================================
    // ICM-20948 初期化シーケンス
    // =======================================================
    uprintln!(serial, "\r\n=== ICM-20948 SPI Reader Start ===");

    // ICM-20948は電源投入後に少し待機が必要
    delay.delay_ms(100_u32);

    // --- デバイスリセット ---
    uprintln!(serial, "Resetting ICM-20948...");
    icm_select_bank(&mut spi, &mut cs, 0);
    icm_write_reg(&mut spi, &mut cs, REG_PWR_MGMT_1, PWR_RESET);
    delay.delay_ms(100_u32); // リセット完了待ち

    // --- ウェイクアップ (SLEEP解除 + Auto Clock) ---
    uprintln!(serial, "Waking up...");
    icm_write_reg(&mut spi, &mut cs, REG_PWR_MGMT_1, PWR_WAKEUP);
    delay.delay_ms(50_u32);

    // --- WHO_AM_I 確認 ---
    loop {
        let who = icm_read_reg(&mut spi, &mut cs, REG_WHO_AM_I);
        if who == WHO_AM_I_VAL {
            uprintln!(serial, "ICM-20948 Found! WHO_AM_I: 0x{:02X}", who);
            break;
        } else {
            uprintln!(serial, "WHO_AM_I: 0x{:02X} (expected 0xEA). Retrying...", who);
            delay.delay_ms(500_u32);
            // リトライ: もう一度ウェイクアップを試みる
            icm_write_reg(&mut spi, &mut cs, REG_PWR_MGMT_1, PWR_WAKEUP);
            delay.delay_ms(50_u32);
        }
    }

    // --- 全センサー有効化 ---
    uprintln!(serial, "Enabling all sensors...");
    icm_write_reg(&mut spi, &mut cs, REG_PWR_MGMT_2, PWR2_ALL_ON);
    delay.delay_ms(10_u32);

    // --- ジャイロ設定 (Bank 2) ---
    uprintln!(serial, "Configuring Gyro: +/-2000 dps");
    icm_select_bank(&mut spi, &mut cs, 2);
    icm_write_reg(&mut spi, &mut cs, REG_GYRO_CONFIG_1, GYRO_FS_2000DPS);

    // --- 加速度設定 (Bank 2) ---
    uprintln!(serial, "Configuring Accel: +/-16g");
    icm_write_reg(&mut spi, &mut cs, REG_ACCEL_CONFIG, ACCEL_FS_16G);

    // --- Bank 0 に戻す (データ読み出し用) ---
    icm_select_bank(&mut spi, &mut cs, 0);
    delay.delay_ms(20_u32);

    uprintln!(serial, "Initialization Complete.");
    uprintln!(serial, "Format: ax,ay,az,gx,gy,gz,temp");
    uprintln!(serial, "Units: [g], [dps], [degC]");
    uprintln!(serial);

    // =======================================================
    // メインループ: センサデータ取得 & UART送信
    // =======================================================
    loop {
        // --- 加速度データ読み出し (6バイト, Big Endian) ---
        let acc_raw = icm_read_6bytes(&mut spi, &mut cs, REG_ACCEL_XOUT_H);
        let ax_raw = i16::from_be_bytes([acc_raw[0], acc_raw[1]]);
        let ay_raw = i16::from_be_bytes([acc_raw[2], acc_raw[3]]);
        let az_raw = i16::from_be_bytes([acc_raw[4], acc_raw[5]]);

        // --- ジャイロデータ読み出し (6バイト, Big Endian) ---
        let gyr_raw = icm_read_6bytes(&mut spi, &mut cs, REG_GYRO_XOUT_H);
        let gx_raw = i16::from_be_bytes([gyr_raw[0], gyr_raw[1]]);
        let gy_raw = i16::from_be_bytes([gyr_raw[2], gyr_raw[3]]);
        let gz_raw = i16::from_be_bytes([gyr_raw[4], gyr_raw[5]]);

        // --- 温度データ読み出し (2バイト, Big Endian) ---
        let temp_raw_bytes = icm_read_2bytes(&mut spi, &mut cs, REG_TEMP_OUT_H);
        let temp_raw = i16::from_be_bytes([temp_raw_bytes[0], temp_raw_bytes[1]]);

        // --- 物理値変換 ---
        let ax = ax_raw as f32 / ACCEL_SENSITIVITY; // [g]
        let ay = ay_raw as f32 / ACCEL_SENSITIVITY;
        let az = az_raw as f32 / ACCEL_SENSITIVITY;

        let gx = gx_raw as f32 / GYRO_SENSITIVITY;  // [dps]
        let gy = gy_raw as f32 / GYRO_SENSITIVITY;
        let gz = gz_raw as f32 / GYRO_SENSITIVITY;

        let temp = (temp_raw as f32 / TEMP_SENSITIVITY) + TEMP_OFFSET; // [°C]

        // --- UART出力 (CSV形式) ---
        // ax, ay, az [g], gx, gy, gz [dps], temp [°C]
        uprintln!(
            serial,
            "{:.3},{:.3},{:.3},{:.1},{:.1},{:.1},{:.1}",
            ax, ay, az, gx, gy, gz, temp
        );

        // --- ループ周期 ---
        // 100ms待機 (10Hz更新)
        delay.delay_ms(100_u32);
    }
}
