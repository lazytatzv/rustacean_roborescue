#include <termios.h>

#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <custom_interfaces/msg/crawler_velocity.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <thread>

using namespace std::chrono_literals;

// ──────────────────────────────────────────────
// Roboclaw protocol constants
// ──────────────────────────────────────────────
constexpr uint8_t ROBOCLAW_ADDRESS = 0x80;
constexpr uint8_t CMD_MIXED_SPEED = 37;
constexpr uint8_t CMD_SET_M1_PID = 28;
constexpr uint8_t CMD_SET_M2_PID = 29;
constexpr uint8_t CMD_RESET_ENCODERS = 20;
constexpr uint8_t CMD_READ_M1_SPEED = 30;
constexpr uint8_t CMD_READ_M2_SPEED = 31;
constexpr uint8_t CMD_READ_CURRENTS = 49;
constexpr uint8_t CMD_READ_TEMPERATURE = 47;
constexpr uint8_t CMD_READ_MAIN_VOLTAGE = 90;
constexpr uint8_t CMD_READ_STATUS = 24;
constexpr uint8_t CMD_SET_M1_MAX_CURRENT = 133;
constexpr uint8_t CMD_SET_M2_MAX_CURRENT = 134;
constexpr uint8_t CMD_SET_SERIAL_TIMEOUT = 14;
constexpr int DEFAULT_SERIAL_BAUD_RATE = 115200;

// ──────────────────────────────────────────────
// Roboclaw status flags (command 24)
// ──────────────────────────────────────────────
constexpr uint32_t STATUS_M1_OVERCURRENT = (1u << 0);
constexpr uint32_t STATUS_M2_OVERCURRENT = (1u << 1);
constexpr uint32_t STATUS_TEMP_ERROR = (1u << 3);
constexpr uint32_t STATUS_MAIN_BATT_LOW = (1u << 6);
constexpr uint32_t STATUS_M1_FAULT = (1u << 8);
constexpr uint32_t STATUS_M2_FAULT = (1u << 9);
constexpr uint32_t STATUS_TEMP_WARNING = (1u << 10);

// ──────────────────────────────────────────────
// Telemetry snapshot
// ──────────────────────────────────────────────
struct RoboclawTelemetry
{
  float m1_current_ma = 0.0f;
  float m2_current_ma = 0.0f;
  int32_t m1_speed_qpps = 0;
  int32_t m2_speed_qpps = 0;
  float temperature_c = 0.0f;
  float battery_volts = 0.0f;
  uint32_t status_flags = 0;
  bool valid = false;
};

// ──────────────────────────────────────────────
// Fixed-size packet builder (zero heap allocation)
// ──────────────────────────────────────────────
struct Packet
{
  static constexpr size_t kMaxSize = 32;
  std::array<uint8_t, kMaxSize> buf{};
  size_t len = 0;

  Packet(uint8_t addr, uint8_t cmd)
  {
    buf[len++] = addr;
    buf[len++] = cmd;
  }

  void push(uint8_t b) { buf[len++] = b; }
  void pushByte(uint8_t b) { buf[len++] = b; }

  void pushInt32(int32_t v)
  {
    for (int i = 3; i >= 0; --i) push(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
  }
  void pushUInt32(uint32_t v)
  {
    for (int i = 3; i >= 0; --i) push(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
  }
  void pushFloat32(float f)
  {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    pushUInt32(bits);
  }
  void pushCRC()
  {
    uint16_t crc = crc16();
    push(static_cast<uint8_t>(crc >> 8));
    push(static_cast<uint8_t>(crc & 0xFF));
  }
  uint16_t crc16() const
  {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; ++i)
    {
      crc ^= static_cast<uint16_t>(buf[i]) << 8;
      for (int j = 0; j < 8; ++j) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
  }
};

// ──────────────────────────────────────────────
// Synchronous Roboclaw serial driver
// すべての I/O は呼び出し元スレッドをブロックする。
// 呼び出し元は必ず専用スレッドから呼ぶこと (ROS executor スレッド禁止)。
// ──────────────────────────────────────────────
class RoboclawDriver
{
 public:
  RoboclawDriver() : io_(), serial_(io_) {}

  void disconnect()
  {
    boost::system::error_code ec;
    if (serial_.is_open()) serial_.close(ec);
  }

  bool connect(const std::string &port, int baud_rate)
  {
    boost::system::error_code ec;
    serial_.open(port, ec);
    if (ec) return false;

    using spb = boost::asio::serial_port_base;
    serial_.set_option(spb::baud_rate(baud_rate));
    serial_.set_option(spb::character_size(8));
    serial_.set_option(spb::parity(spb::parity::none));
    serial_.set_option(spb::stop_bits(spb::stop_bits::one));
    serial_.set_option(spb::flow_control(spb::flow_control::none));

    int fd = serial_.native_handle();
    struct termios tio;
    if (tcgetattr(fd, &tio) == 0)
    {
      tio.c_cc[VMIN] = 0;
      tio.c_cc[VTIME] = 2;  // 200ms per-byte timeout
      tcsetattr(fd, TCSANOW, &tio);
    }
    return true;
  }

  // ── Write (ACK-based) ─────────────────────────────────────────────────────

  bool sendPacket(const Packet &pkt)
  {
    boost::system::error_code ec;
    boost::asio::write(serial_, boost::asio::buffer(pkt.buf.data(), pkt.len), ec);
    if (ec)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RoboclawDriver"), "Write error: %s", ec.message().c_str());
      return false;
    }
    uint8_t ack = 0;
    size_t n = boost::asio::read(serial_, boost::asio::buffer(&ack, 1), ec);
    if (ec || n == 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RoboclawDriver"), "ACK timeout");
      return false;
    }
    return true;
  }

  /// M1・M2 を1トランザクションで原子的に送信 (cmd=37)
  bool setMixedSpeed(int32_t m1_qpps, int32_t m2_qpps)
  {
    Packet pkt(ROBOCLAW_ADDRESS, CMD_MIXED_SPEED);
    pkt.pushInt32(m1_qpps);
    pkt.pushInt32(m2_qpps);
    pkt.pushCRC();
    return sendPacket(pkt);
  }

  bool setPIDConstants(uint8_t cmd, float Kp, float Ki, float Kd, int qpps)
  {
    Packet pkt(ROBOCLAW_ADDRESS, cmd);
    pkt.pushFloat32(Kd);
    pkt.pushFloat32(Kp);
    pkt.pushFloat32(Ki);
    pkt.pushInt32(qpps);
    pkt.pushCRC();
    return sendPacket(pkt);
  }

  bool setMaxCurrent(uint8_t cmd, float max_ma)
  {
    Packet pkt(ROBOCLAW_ADDRESS, cmd);
    pkt.pushUInt32(static_cast<uint32_t>(max_ma / 10.0f));  // max
    pkt.pushUInt32(0);                                      // min = 0
    pkt.pushCRC();
    return sendPacket(pkt);
  }

  bool resetEncoders()
  {
    Packet pkt(ROBOCLAW_ADDRESS, CMD_RESET_ENCODERS);
    pkt.pushCRC();
    return sendPacket(pkt);
  }

  bool setSerialTimeoutDs(uint8_t timeout_ds)
  {
    Packet pkt(ROBOCLAW_ADDRESS, CMD_SET_SERIAL_TIMEOUT);
    pkt.push(timeout_ds);
    pkt.pushCRC();
    return sendPacket(pkt);
  }

  // ── Read ──────────────────────────────────────────────────────────────────

  /// [addr][cmd] を送信し、data_bytes + CRC2 を受信して CRC 検証する
  bool readCommand(uint8_t cmd, uint8_t *out, size_t data_bytes)
  {
    const uint8_t req[2] = {ROBOCLAW_ADDRESS, cmd};
    boost::system::error_code ec;
    boost::asio::write(serial_, boost::asio::buffer(req, 2), ec);
    if (ec) return false;

    const size_t total = data_bytes + 2;
    size_t n = boost::asio::read(serial_, boost::asio::buffer(out, total), ec);
    if (ec || n != total)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RoboclawDriver"),
                   "readCommand(cmd=%u): read failed ec='%s' n=%zu expected=%zu",
                   cmd, ec.message().c_str(), n, total);
      return false;
    }

    // CRC 検証: [addr][cmd][data...] の crc16
    Packet crc_pkt(ROBOCLAW_ADDRESS, cmd);
    for (size_t i = 0; i < data_bytes; ++i) crc_pkt.push(out[i]);
    const uint16_t expected = crc_pkt.crc16();
    const uint16_t received = (static_cast<uint16_t>(out[data_bytes]) << 8) | out[data_bytes + 1];
    if (expected != received)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RoboclawDriver"),
                   "readCommand(cmd=%u): CRC mismatch expected=0x%04X received=0x%04X bytes=[%s]",
                   cmd, expected, received,
                   [&]()
                   {
                     std::string s;
                     for (size_t i = 0; i < total; ++i)
                     {
                       char buf[4];
                       snprintf(buf, sizeof(buf), "%02X ", out[i]);
                       s += buf;
                     }
                     return s;
                   }()
                       .c_str());
      return false;
    }
    return true;
  }

  bool readMotorCurrents(float &m1_ma, float &m2_ma)
  {
    uint8_t buf[6];
    if (!readCommand(CMD_READ_CURRENTS, buf, 4)) return false;
    m1_ma = static_cast<float>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]) * 10.0f;
    m2_ma = static_cast<float>((static_cast<uint16_t>(buf[2]) << 8) | buf[3]) * 10.0f;
    return true;
  }

  bool readMotorSpeed(uint8_t cmd, int32_t &speed_qpps)
  {
    uint8_t buf[7];
    if (!readCommand(cmd, buf, 5)) return false;
    speed_qpps = (static_cast<int32_t>(buf[0]) << 24) | (static_cast<int32_t>(buf[1]) << 16) |
                 (static_cast<int32_t>(buf[2]) << 8) | static_cast<int32_t>(buf[3]);
    if (buf[4] == 0) speed_qpps = -speed_qpps;  // 0=後退
    return true;
  }

  bool readTemperature(float &temp_c)
  {
    uint8_t buf[4];
    if (!readCommand(CMD_READ_TEMPERATURE, buf, 2)) return false;
    temp_c = static_cast<float>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]) / 10.0f;
    return true;
  }

  bool readBatteryVoltage(float &volts)
  {
    uint8_t buf[4];
    if (!readCommand(CMD_READ_MAIN_VOLTAGE, buf, 2)) return false;
    volts = static_cast<float>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]) / 10.0f;
    return true;
  }

  bool readStatus(uint32_t &status)
  {
    uint8_t buf[6];
    if (!readCommand(CMD_READ_STATUS, buf, 4)) return false;
    status = (static_cast<uint32_t>(buf[0]) << 24) | (static_cast<uint32_t>(buf[1]) << 16) |
             (static_cast<uint32_t>(buf[2]) << 8) | static_cast<uint32_t>(buf[3]);
    return true;
  }

  RoboclawTelemetry readTelemetry()
  {
    RoboclawTelemetry t;
    t.valid = true;
    if (!readMotorCurrents(t.m1_current_ma, t.m2_current_ma)) t.valid = false;
    if (!readMotorSpeed(CMD_READ_M1_SPEED, t.m1_speed_qpps)) t.valid = false;
    if (!readMotorSpeed(CMD_READ_M2_SPEED, t.m2_speed_qpps)) t.valid = false;
    if (!readTemperature(t.temperature_c)) t.valid = false;
    if (!readBatteryVoltage(t.battery_volts)) t.valid = false;
    if (!readStatus(t.status_flags)) t.valid = false;
    return t;
  }

 private:
  boost::asio::io_context io_;
  boost::asio::serial_port serial_;
};

// ──────────────────────────────────────────────
// スタック検出: 速度誤差スコアで出力をスケーリング
// ──────────────────────────────────────────────
struct StallState
{
  double score = 0.0;  // 0.0 ~ stall_window_sec
  double scale = 1.0;  // 出力スケール 1.0 ~ (1 - max_reduction)
};

// ──────────────────────────────────────────────
// ROS 2 Crawler Driver Node
//
// スレッド分離:
//   ROS executor スレッド: cmd_vel/driver 受信 → コマンドスロット更新 (ノンブロッキング)
//                          watchdog / telemetry publish / estop 処理
//   serial スレッド       : すべての Roboclaw I/O (コマンド送信 + テレメトリ読み取り)
//                          接続 / 再接続 / スタック検出 も担当
// ──────────────────────────────────────────────
class CrawlerDriver : public rclcpp::Node
{
 public:
  CrawlerDriver() : Node("crawler_driver")
  {
    this->declare_parameter<std::string>("serial_port", "/dev/roboclaw");
    declare_parameter("baud_rate", DEFAULT_SERIAL_BAUD_RATE);
    declare_parameter("crawler_circumference", 0.39);
    declare_parameter("counts_per_rev", 256);
    declare_parameter("gearhead_ratio", 66);
    declare_parameter("pulley_ratio", 2);
    declare_parameter("watchdog_timeout_ms", 500);
    declare_parameter("track_width", 0.4);
    declare_parameter("m1_kp", 0.464);
    declare_parameter("m1_ki", 0.021);
    declare_parameter("m1_kd", 0.0);
    declare_parameter("m1_qpps", 53250);
    declare_parameter("m2_kp", 0.438);
    declare_parameter("m2_ki", 0.020);
    declare_parameter("m2_kd", 0.0);
    declare_parameter("m2_qpps", 50062);
    declare_parameter("m1_direction_sign", 1);
    declare_parameter("m2_direction_sign", 1);
    declare_parameter("reset_encoders_on_connect", true);
    declare_parameter("reset_encoders_on_reconnect", false);
    declare_parameter("stall_error_ratio", 0.5);
    declare_parameter("stall_cmd_threshold_qpps", 500);
    declare_parameter("stall_window_sec", 2.0);
    declare_parameter("stall_max_reduction", 0.5);
    declare_parameter("m1_max_current_ma", 15000.0);
    declare_parameter("m2_max_current_ma", 15000.0);
    declare_parameter("telemetry_hz", 10);
    declare_parameter("serial_timeout_ds", 5);
    declare_parameter("debug_io", false);
    declare_parameter("debug_log_period_ms", 200);
    declare_parameter("connect_retry_sec", 3.0);

    initParams();
    initPublishers();

    subscription_ = create_subscription<custom_interfaces::msg::CrawlerVelocity>(
        "/crawler_driver", 10,
        std::bind(&CrawlerDriver::driver_callback, this, std::placeholders::_1));

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&CrawlerDriver::cmd_vel_callback, this, std::placeholders::_1));

    watchdog_timer_ = create_wall_timer(std::chrono::milliseconds(watchdog_timeout_ms_),
                                        std::bind(&CrawlerDriver::watchdog_callback, this));

    // テレメトリ publish は ROS タイマーで行う。
    // 実際の serial 読み取りは serial スレッド側で行い、shared_telemetry_ に格納する。
    const int telem_ms = static_cast<int>(1000.0 / static_cast<double>(telemetry_hz_));
    telemetry_timer_ =
        create_wall_timer(std::chrono::milliseconds(telem_ms),
                          std::bind(&CrawlerDriver::telemetry_publish_callback, this));

    auto estop_qos = rclcpp::QoS(1).transient_local().reliable();
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop", estop_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
          const bool was_estop = estop_active_;
          estop_active_ = msg->data;
          if (estop_active_ && !was_estop)
          {
            force_stop_.store(true, std::memory_order_release);
            cmd_cv_.notify_one();
            RCLCPP_FATAL(get_logger(), "⛔ EMERGENCY STOP — motors halted");
          }
          else if (!estop_active_ && was_estop)
          {
            RCLCPP_WARN(get_logger(), "✅ E-STOP cleared");
          }
        });

    // serial スレッド起動 (接続 / 再接続 / コマンド送信 / テレメトリ読み取りを担当)
    running_.store(true, std::memory_order_release);
    serial_thread_ = std::thread(&CrawlerDriver::serialThreadFunc, this);
  }

  ~CrawlerDriver() override
  {
    running_.store(false, std::memory_order_release);
    cmd_cv_.notify_all();
    if (serial_thread_.joinable()) serial_thread_.join();
  }

 private:
  // ── Roboclaw ドライバ (serial スレッド専用) ────────────────────────────────
  RoboclawDriver roboclaw_;

  // ── パラメータ (initParams で設定、以降読み取り専用) ──────────────────────
  double counts_per_meter_;
  int watchdog_timeout_ms_;
  int baud_rate_;
  bool debug_io_;
  int debug_log_period_ms_;
  double track_width_;
  double m1_kp_, m1_ki_, m1_kd_;
  int m1_qpps_;
  double m2_kp_, m2_ki_, m2_kd_;
  int m2_qpps_;
  int m1_direction_sign_;
  int m2_direction_sign_;
  bool reset_encoders_on_connect_;
  bool reset_encoders_on_reconnect_;
  double stall_error_ratio_;
  int stall_cmd_threshold_qpps_;
  double stall_window_sec_;
  double stall_max_reduction_;
  float m1_max_current_ma_;
  float m2_max_current_ma_;
  int telemetry_hz_;
  double telemetry_dt_sec_;
  int serial_timeout_ds_;
  double connect_retry_sec_;
  std::string serial_port_;

  // ── serial スレッド ────────────────────────────────────────────────────────
  std::thread serial_thread_;
  std::atomic<bool> running_{false};

  // ── コマンドスロット (latest-wins) ────────────────────────────────────────
  // ROS スレッドが書き、serial スレッドが読む。
  struct MotorCmd
  {
    int32_t m1 = 0;
    int32_t m2 = 0;
  };
  MotorCmd pending_cmd_;
  bool pending_valid_ = false;
  std::mutex cmd_mutex_;
  std::condition_variable cmd_cv_;

  // ── フォース停止フラグ (watchdog / estop) ────────────────────────────────
  std::atomic<bool> force_stop_{false};

  // ── 共有テレメトリ (serial スレッドが書き、ROS スレッドが publish する) ──
  RoboclawTelemetry shared_telemetry_;
  std::mutex telemetry_mutex_;

  // ── スタールスケール (serial スレッドが書き、ROS スレッドが publish する) ─
  std::atomic<double> m1_stall_scale_{1.0};
  std::atomic<double> m2_stall_scale_{1.0};

  // ── HW 接続状態 ───────────────────────────────────────────────────────────
  std::atomic<bool> hw_connected_{false};

  // ── ROS スレッド専用状態 ──────────────────────────────────────────────────
  rclcpp::Time last_cmd_time_;
  bool cmd_vel_is_zero_ = true;
  bool estop_active_ = false;

  // ── タイマー / サブスクリプション ─────────────────────────────────────────
  rclcpp::Subscription<custom_interfaces::msg::CrawlerVelocity>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;

  // ── パブリッシャ ───────────────────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_m1_current_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_m2_current_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_m1_speed_actual_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_m2_speed_actual_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_temperature_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_battery_voltage_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_status_flags_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_m1_stall_scale_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_m2_stall_scale_;

  // ════════════════════════════════════════════════════════════════════════════
  // serial スレッド
  // ════════════════════════════════════════════════════════════════════════════

  void serialThreadFunc()
  {
    using clk = std::chrono::steady_clock;

    // serial スレッド専用状態 (他スレッドからアクセス不可)
    bool has_connected_once = false;
    bool motors_stopped = true;
    StallState m1_stall, m2_stall;
    int32_t m1_cmd_qpps = 0, m2_cmd_qpps = 0;
    const auto telem_period =
        std::chrono::duration_cast<clk::duration>(std::chrono::duration<double>(telemetry_dt_sec_));
    auto telem_deadline = clk::now();

    while (running_.load(std::memory_order_relaxed))
    {
      // ── 1. 未接続時: 接続を試みる ─────────────────────────────────────────
      if (!hw_connected_.load(std::memory_order_acquire))
      {
        if (roboclaw_.connect(serial_port_, baud_rate_))
        {
          // 接続成功: 状態リセット後にハードウェア初期化
          motors_stopped = true;
          m1_stall = StallState{};
          m2_stall = StallState{};
          m1_cmd_qpps = 0;
          m2_cmd_qpps = 0;
          telem_deadline = clk::now();

          if (serialInitHardware(has_connected_once))
          {
            has_connected_once = true;
            hw_connected_.store(true, std::memory_order_release);
            RCLCPP_INFO(get_logger(), "Connected to Roboclaw on %s @ %d bps", serial_port_.c_str(),
                        baud_rate_);
          }
          else
          {
            RCLCPP_ERROR(get_logger(), "Hardware init failed, retrying...");
            roboclaw_.disconnect();
            // initHardware 失敗: retry 前に待機
            serialSleep();
          }
        }
        else
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                               "Failed to connect to Roboclaw on %s @ %d bps. Retrying...",
                               serial_port_.c_str(), baud_rate_);
          serialSleep();
        }
        continue;
      }

      // ── 2. コマンド受信待ち (テレメトリ期限まで) ──────────────────────────
      MotorCmd cmd;
      bool has_cmd = false;
      bool do_stop = false;

      {
        std::unique_lock<std::mutex> lk(cmd_mutex_);
        cmd_cv_.wait_until(lk, telem_deadline,
                           [this]
                           {
                             return !running_.load(std::memory_order_relaxed) || pending_valid_ ||
                                    force_stop_.load(std::memory_order_relaxed);
                           });

        if (!running_.load(std::memory_order_relaxed)) break;

        do_stop = force_stop_.exchange(false, std::memory_order_acq_rel);
        if (pending_valid_)
        {
          cmd = pending_cmd_;
          pending_valid_ = false;
          has_cmd = true;
        }
      }

      // ── 3. コマンド実行 ────────────────────────────────────────────────────
      if (do_stop)
      {
        // watchdog / estop によるフォース停止: 無条件に送信
        m1_cmd_qpps = 0;
        m2_cmd_qpps = 0;
        motors_stopped = true;
        if (!roboclaw_.setMixedSpeed(0, 0))
        {
          serialHandleFailure("force stop failed");
          continue;
        }
      }
      else if (has_cmd)
      {
        if (cmd.m1 == 0 && cmd.m2 == 0)
        {
          // 停止指令: 遷移時のみ送信。以降は Roboclaw の serial_timeout_ds に任せる。
          if (!motors_stopped)
          {
            m1_cmd_qpps = 0;
            m2_cmd_qpps = 0;
            motors_stopped = true;
            if (!roboclaw_.setMixedSpeed(0, 0))
            {
              serialHandleFailure("stop command failed");
              continue;
            }
          }
        }
        else
        {
          // 走行指令: スタールスケールを適用して送信
          m1_cmd_qpps = cmd.m1;
          m2_cmd_qpps = cmd.m2;
          const int32_t m1_out = static_cast<int32_t>(static_cast<double>(cmd.m1) * m1_stall.scale);
          const int32_t m2_out = static_cast<int32_t>(static_cast<double>(cmd.m2) * m2_stall.scale);

          motors_stopped = false;

          if (debug_io_)
          {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), debug_log_period_ms_,
                                 "[debug_io] qpps_cmd=(%d,%d) qpps_out=(%d,%d) stall=(%.2f,%.2f)",
                                 cmd.m1, cmd.m2, m1_out, m2_out, m1_stall.scale, m2_stall.scale);
          }

          if (!roboclaw_.setMixedSpeed(m1_out, m2_out))
          {
            serialHandleFailure("command send failed");
            continue;
          }
        }
      }

      // ── 4. テレメトリ読み取り (telemetry_hz_ に従う) ──────────────────────
      if (clk::now() >= telem_deadline)
      {
        // 次の期限を更新。積算遅延を防ぐため now() より過去にはしない。
        telem_deadline += telem_period;
        if (telem_deadline < clk::now()) telem_deadline = clk::now() + telem_period;

        const RoboclawTelemetry t = roboclaw_.readTelemetry();
        if (!t.valid)
        {
          serialHandleFailure("telemetry read failed");
          continue;
        }

        {
          std::lock_guard<std::mutex> lk(telemetry_mutex_);
          shared_telemetry_ = t;
        }

        // スタール検出 (telemetry_hz_ ごとに実行 → dt が安定)
        const int32_t m1_actual = t.m1_speed_qpps * m1_direction_sign_;
        const int32_t m2_actual = t.m2_speed_qpps * m2_direction_sign_;
        updateStall(m1_stall, m1_cmd_qpps, m1_actual);
        updateStall(m2_stall, m2_cmd_qpps, m2_actual);
        m1_stall_scale_.store(m1_stall.scale, std::memory_order_relaxed);
        m2_stall_scale_.store(m2_stall.scale, std::memory_order_relaxed);
      }
    }
  }

  /// running_ が false になるまで connect_retry_sec_ 待機する
  void serialSleep()
  {
    std::unique_lock<std::mutex> lk(cmd_mutex_);
    cmd_cv_.wait_for(lk, std::chrono::duration<double>(connect_retry_sec_),
                     [this] { return !running_.load(std::memory_order_relaxed); });
  }

  void serialHandleFailure(const char *reason)
  {
    RCLCPP_ERROR(get_logger(),
                 "Roboclaw comm failure (%s). Disconnecting and scheduling reconnect.", reason);
    hw_connected_.store(false, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lk(telemetry_mutex_);
      shared_telemetry_.valid = false;
    }
    m1_stall_scale_.store(1.0, std::memory_order_relaxed);
    m2_stall_scale_.store(1.0, std::memory_order_relaxed);
    roboclaw_.disconnect();
    serialSleep();
  }

  bool serialInitHardware(bool has_connected_once)
  {
    bool ok = true;
    ok &= roboclaw_.setMixedSpeed(0, 0);
    ok &= roboclaw_.setPIDConstants(CMD_SET_M1_PID, m1_kp_, m1_ki_, m1_kd_, m1_qpps_);
    ok &= roboclaw_.setPIDConstants(CMD_SET_M2_PID, m2_kp_, m2_ki_, m2_kd_, m2_qpps_);
    if (ok)
    {
      RCLCPP_INFO(get_logger(),
                  "PID: M1(Kp=%.4f Ki=%.4f Kd=%.4f qpps=%d) M2(Kp=%.4f Ki=%.4f Kd=%.4f qpps=%d)",
                  m1_kp_, m1_ki_, m1_kd_, m1_qpps_, m2_kp_, m2_ki_, m2_kd_, m2_qpps_);
    }

    const bool should_reset =
        reset_encoders_on_connect_ && (!has_connected_once || reset_encoders_on_reconnect_);
    if (should_reset)
    {
      ok &= roboclaw_.resetEncoders();
      RCLCPP_INFO(get_logger(), "Encoders reset (%s)",
                  has_connected_once ? "reconnect" : "startup");
    }

    if (serial_timeout_ds_ >= 0 && serial_timeout_ds_ <= 255)
    {
      if (roboclaw_.setSerialTimeoutDs(static_cast<uint8_t>(serial_timeout_ds_)))
      {
        RCLCPP_INFO(get_logger(), "Roboclaw serial timeout: %d ds (%d ms)", serial_timeout_ds_,
                    serial_timeout_ds_ * 100);
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Failed to set Roboclaw serial timeout");
        ok = false;
      }
    }

    if (m1_max_current_ma_ > 0.0f)
    {
      ok &= roboclaw_.setMaxCurrent(CMD_SET_M1_MAX_CURRENT, m1_max_current_ma_);
      RCLCPP_INFO(get_logger(), "M1 max current: %.0f mA", m1_max_current_ma_);
    }
    if (m2_max_current_ma_ > 0.0f)
    {
      ok &= roboclaw_.setMaxCurrent(CMD_SET_M2_MAX_CURRENT, m2_max_current_ma_);
      RCLCPP_INFO(get_logger(), "M2 max current: %.0f mA", m2_max_current_ma_);
    }

    if (ok) RCLCPP_INFO(get_logger(), "Hardware initialized");
    return ok;
  }

  /// スタール検出。telemetry_hz_ ごとに呼ぶこと (dt = telemetry_dt_sec_ で固定)
  void updateStall(StallState &state, int32_t cmd_qpps, int32_t actual_qpps)
  {
    const double dt = telemetry_dt_sec_;
    const bool cmd_significant = std::abs(cmd_qpps) > stall_cmd_threshold_qpps_;
    const bool stalling = cmd_significant && (std::abs(cmd_qpps - actual_qpps) >
                                              std::abs(cmd_qpps) * stall_error_ratio_);

    if (stalling)
      state.score = std::min(state.score + dt, stall_window_sec_);
    else
      state.score = std::max(state.score - dt * 2.0, 0.0);  // 回復は 2 倍速

    const double ratio = state.score / stall_window_sec_;
    state.scale = 1.0 - ratio * stall_max_reduction_;

    if (stalling && ratio > 0.8)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "⚠️  Motor stall detected (scale=%.2f)",
                           state.scale);
  }

  // ════════════════════════════════════════════════════════════════════════════
  // ROS スレッド コールバック
  // ════════════════════════════════════════════════════════════════════════════

  static constexpr double kZeroVelThresh = 1e-4;  // [m/s] 停止扱いとするしきい値

  /// コマンドをスロットに書き込んで serial スレッドを起こす。
  /// zero は「一度だけ送る」ロジックを ROS スレッド側で処理してから呼ぶこと。
  void pushCommand(int32_t m1, int32_t m2)
  {
    {
      std::lock_guard<std::mutex> lk(cmd_mutex_);
      pending_cmd_ = {m1, m2};
      pending_valid_ = true;
    }
    cmd_cv_.notify_one();
  }

  /// m/s → qpps 変換 (方向符号適用済み) してスロットを更新する共通ロジック
  void handleVelocity(double m1_vel, double m2_vel)
  {
    const int32_t m1_qpps = static_cast<int32_t>(m1_vel * counts_per_meter_) * m1_direction_sign_;
    const int32_t m2_qpps = static_cast<int32_t>(m2_vel * counts_per_meter_) * m2_direction_sign_;

    if (std::abs(m1_vel) < kZeroVelThresh && std::abs(m2_vel) < kZeroVelThresh)
    {
      // 停止遷移時のみ一度だけプッシュ。以降は無言 (serial_timeout_ds が最終保証)。
      if (!cmd_vel_is_zero_)
      {
        cmd_vel_is_zero_ = true;
        pushCommand(0, 0);
      }
      return;
    }
    cmd_vel_is_zero_ = false;
    pushCommand(m1_qpps, m2_qpps);
  }

  void driver_callback(const custom_interfaces::msg::CrawlerVelocity &msg)
  {
    if (estop_active_) return;
    last_cmd_time_ = now();
    handleVelocity(msg.m1_vel, msg.m2_vel);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist &msg)
  {
    if (estop_active_) return;
    last_cmd_time_ = now();

    const double half_track = track_width_ / 2.0;
    handleVelocity(msg.linear.x - msg.angular.z * half_track,
                   msg.linear.x + msg.angular.z * half_track);
  }

  void watchdog_callback()
  {
    if (!hw_connected_.load(std::memory_order_acquire)) return;

    const double elapsed = (now() - last_cmd_time_).seconds() * 1000.0;
    if (elapsed > static_cast<double>(watchdog_timeout_ms_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Watchdog: no command for %.0f ms, stopping motors", elapsed);
      force_stop_.store(true, std::memory_order_release);
      cmd_cv_.notify_one();
    }
  }

  // テレメトリ publish のみ (serial I/O なし)
  void telemetry_publish_callback()
  {
    RoboclawTelemetry t;
    {
      std::lock_guard<std::mutex> lk(telemetry_mutex_);
      t = shared_telemetry_;
    }
    if (!t.valid) return;

    auto pub_f64 = [](auto &pub, double val)
    {
      std_msgs::msg::Float64 msg;
      msg.data = val;
      pub->publish(msg);
    };

    pub_f64(pub_m1_current_, t.m1_current_ma);
    pub_f64(pub_m2_current_, t.m2_current_ma);
    pub_f64(pub_m1_speed_actual_, static_cast<double>(t.m1_speed_qpps));
    pub_f64(pub_m2_speed_actual_, static_cast<double>(t.m2_speed_qpps));
    pub_f64(pub_temperature_, t.temperature_c);
    pub_f64(pub_battery_voltage_, t.battery_volts);
    pub_f64(pub_m1_stall_scale_, m1_stall_scale_.load(std::memory_order_relaxed));
    pub_f64(pub_m2_stall_scale_, m2_stall_scale_.load(std::memory_order_relaxed));

    {
      std_msgs::msg::UInt32 msg;
      msg.data = t.status_flags;
      pub_status_flags_->publish(msg);
    }

    const auto &sf = t.status_flags;
    if (sf & STATUS_TEMP_ERROR)
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "🌡️  Roboclaw temperature error (%.1f°C)", t.temperature_c);
    else if (sf & STATUS_TEMP_WARNING)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "🌡️  Roboclaw temperature warning (%.1f°C)", t.temperature_c);
    if (sf & STATUS_M1_OVERCURRENT)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "⚡ M1 overcurrent (%.0f mA)",
                           t.m1_current_ma);
    if (sf & STATUS_M2_OVERCURRENT)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "⚡ M2 overcurrent (%.0f mA)",
                           t.m2_current_ma);
    if (sf & STATUS_MAIN_BATT_LOW)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "🔋 Main battery low (%.1f V)",
                           t.battery_volts);
    if (sf & STATUS_M1_FAULT)
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "💥 M1 driver fault");
    if (sf & STATUS_M2_FAULT)
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "💥 M2 driver fault");
  }

  // ════════════════════════════════════════════════════════════════════════════
  // 初期化ヘルパ
  // ════════════════════════════════════════════════════════════════════════════

  void initParams()
  {
    serial_port_ = get_parameter("serial_port").as_string();
    baud_rate_ = get_parameter("baud_rate").as_int();
    debug_io_ = get_parameter("debug_io").as_bool();
    debug_log_period_ms_ = get_parameter("debug_log_period_ms").as_int();
    if (debug_log_period_ms_ <= 0) debug_log_period_ms_ = 200;
    track_width_ = get_parameter("track_width").as_double();
    watchdog_timeout_ms_ = get_parameter("watchdog_timeout_ms").as_int();
    connect_retry_sec_ = get_parameter("connect_retry_sec").as_double();

    const double circumference = get_parameter("crawler_circumference").as_double();
    const int counts_per_rev = get_parameter("counts_per_rev").as_int();
    const int gearhead = get_parameter("gearhead_ratio").as_int();
    const int pulley = get_parameter("pulley_ratio").as_int();
    counts_per_meter_ = (counts_per_rev * gearhead * pulley) / circumference;

    m1_kp_ = get_parameter("m1_kp").as_double();
    m1_ki_ = get_parameter("m1_ki").as_double();
    m1_kd_ = get_parameter("m1_kd").as_double();
    m1_qpps_ = get_parameter("m1_qpps").as_int();
    m2_kp_ = get_parameter("m2_kp").as_double();
    m2_ki_ = get_parameter("m2_ki").as_double();
    m2_kd_ = get_parameter("m2_kd").as_double();
    m2_qpps_ = get_parameter("m2_qpps").as_int();

    m1_direction_sign_ = (get_parameter("m1_direction_sign").as_int() >= 0) ? 1 : -1;
    m2_direction_sign_ = (get_parameter("m2_direction_sign").as_int() >= 0) ? 1 : -1;

    reset_encoders_on_connect_ = get_parameter("reset_encoders_on_connect").as_bool();
    reset_encoders_on_reconnect_ = get_parameter("reset_encoders_on_reconnect").as_bool();

    stall_error_ratio_ = get_parameter("stall_error_ratio").as_double();
    stall_cmd_threshold_qpps_ = get_parameter("stall_cmd_threshold_qpps").as_int();
    stall_window_sec_ = get_parameter("stall_window_sec").as_double();
    stall_max_reduction_ = get_parameter("stall_max_reduction").as_double();

    m1_max_current_ma_ = static_cast<float>(get_parameter("m1_max_current_ma").as_double());
    m2_max_current_ma_ = static_cast<float>(get_parameter("m2_max_current_ma").as_double());

    telemetry_hz_ = get_parameter("telemetry_hz").as_int();
    if (telemetry_hz_ <= 0)
    {
      RCLCPP_WARN(get_logger(), "Invalid telemetry_hz=%d, using 10 Hz", telemetry_hz_);
      telemetry_hz_ = 10;
    }
    telemetry_dt_sec_ = 1.0 / static_cast<double>(telemetry_hz_);

    serial_timeout_ds_ = get_parameter("serial_timeout_ds").as_int();

    if (baud_rate_ <= 0)
    {
      RCLCPP_WARN(get_logger(), "Invalid baud_rate=%d, using %d", baud_rate_,
                  DEFAULT_SERIAL_BAUD_RATE);
      baud_rate_ = DEFAULT_SERIAL_BAUD_RATE;
    }

    RCLCPP_INFO(get_logger(), "Motor direction signs: m1=%d m2=%d", m1_direction_sign_,
                m2_direction_sign_);

    last_cmd_time_ = now();
  }

  void initPublishers()
  {
    const std::string p = "~/diagnostics/";
    pub_m1_current_ = create_publisher<std_msgs::msg::Float64>(p + "m1_current_ma", 10);
    pub_m2_current_ = create_publisher<std_msgs::msg::Float64>(p + "m2_current_ma", 10);
    pub_m1_speed_actual_ = create_publisher<std_msgs::msg::Float64>(p + "m1_speed_actual", 10);
    pub_m2_speed_actual_ = create_publisher<std_msgs::msg::Float64>(p + "m2_speed_actual", 10);
    pub_temperature_ = create_publisher<std_msgs::msg::Float64>(p + "temperature_c", 10);
    pub_battery_voltage_ = create_publisher<std_msgs::msg::Float64>(p + "battery_volts", 10);
    pub_status_flags_ = create_publisher<std_msgs::msg::UInt32>(p + "status_flags", 10);
    pub_m1_stall_scale_ = create_publisher<std_msgs::msg::Float64>(p + "m1_stall_scale", 10);
    pub_m2_stall_scale_ = create_publisher<std_msgs::msg::Float64>(p + "m2_stall_scale", 10);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrawlerDriver>());
  rclcpp::shutdown();
  return 0;
}
