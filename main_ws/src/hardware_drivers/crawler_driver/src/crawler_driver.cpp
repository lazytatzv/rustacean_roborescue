#include <termios.h>

#include <boost/asio.hpp>
#include <chrono>
#include <cstring>
#include <custom_interfaces/msg/crawler_velocity.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;

// ──────────────────────────────────────────────
// Roboclaw protocol constants
// ──────────────────────────────────────────────
constexpr uint8_t ROBOCLAW_ADDRESS = 0x80;
constexpr uint8_t CMD_M1_VELOCITY = 35;
constexpr uint8_t CMD_M2_VELOCITY = 36;
constexpr uint8_t CMD_SET_M1_PID = 28;
constexpr uint8_t CMD_SET_M2_PID = 29;
constexpr uint8_t CMD_RESET_ENCODERS = 20;
constexpr uint8_t CMD_READ_M1_SPEED = 18;
constexpr uint8_t CMD_READ_M2_SPEED = 19;
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
constexpr uint32_t STATUS_ESTOP = (1u << 2);
constexpr uint32_t STATUS_TEMP_ERROR = (1u << 3);
constexpr uint32_t STATUS_MAIN_BATT_HIGH = (1u << 5);
constexpr uint32_t STATUS_MAIN_BATT_LOW = (1u << 6);
constexpr uint32_t STATUS_M1_FAULT = (1u << 8);
constexpr uint32_t STATUS_M2_FAULT = (1u << 9);
constexpr uint32_t STATUS_TEMP_WARNING = (1u << 10);
constexpr uint32_t STATUS_MAIN_BATT_WARN = (1u << 11);

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
// Synchronous Roboclaw serial driver
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

  /// シリアルポートを開く。失敗した場合は false を返す (例外を投げない)
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
      tio.c_cc[VTIME] = 2;
      tcsetattr(fd, TCSANOW, &tio);
    }
    return true;
  }

  // ── Write commands (expect 1-byte ACK) ───────────────────────────────────

  bool sendCommand(const std::vector<uint8_t> &data)
  {
    boost::system::error_code ec;
    boost::asio::write(serial_, boost::asio::buffer(data), ec);
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

  bool setMotorVelocity(uint8_t command, int32_t counts_per_sec)
  {
    std::vector<uint8_t> data = {ROBOCLAW_ADDRESS, command};
    appendInt32(data, counts_per_sec);
    appendCRC(data);
    return sendCommand(data);
  }

  bool setPIDConstants(uint8_t command, float Kp, float Ki, float Kd, int qpps)
  {
    std::vector<uint8_t> data = {ROBOCLAW_ADDRESS, command};
    appendFloat32(data, Kd);
    appendFloat32(data, Kp);
    appendFloat32(data, Ki);
    appendInt32(data, qpps);
    appendCRC(data);
    return sendCommand(data);
  }

  /// モーター最大電流制限を設定する (単位: mA, Roboclaw内部は10mA単位)
  /// min は常に 0 (アンダーカレント保護なし)
  bool setMaxCurrent(uint8_t cmd, float max_ma)
  {
    const uint32_t raw = static_cast<uint32_t>(max_ma / 10.0f);
    std::vector<uint8_t> data = {ROBOCLAW_ADDRESS, cmd};
    // max current (4 bytes big-endian)
    for (int i = 3; i >= 0; --i) data.push_back(static_cast<uint8_t>((raw >> (8 * i)) & 0xFF));
    // min current = 0 (4 bytes)
    for (int i = 0; i < 4; ++i) data.push_back(0x00);
    appendCRC(data);
    return sendCommand(data);
  }

  bool resetEncoders()
  {
    std::vector<uint8_t> data = {ROBOCLAW_ADDRESS, CMD_RESET_ENCODERS};
    appendCRC(data);
    return sendCommand(data);
  }

  /// 通信タイムアウトを設定する (単位: 100ms)
  /// タイムアウトを過ぎると Roboclaw が自動停止する。
  bool setSerialTimeoutDs(uint8_t timeout_ds)
  {
    std::vector<uint8_t> data = {ROBOCLAW_ADDRESS, CMD_SET_SERIAL_TIMEOUT, timeout_ds};
    appendCRC(data);
    return sendCommand(data);
  }

  // ── Read commands ─────────────────────────────────────────────────────────

  /// 汎用読み出し: [address][cmd] を送信し data_bytes + CRC2 を受信して CRC 検証する
  bool readCommand(uint8_t cmd, std::vector<uint8_t> &out, size_t data_bytes)
  {
    const std::vector<uint8_t> req = {ROBOCLAW_ADDRESS, cmd};
    boost::system::error_code ec;
    boost::asio::write(serial_, boost::asio::buffer(req), ec);
    if (ec) return false;

    out.resize(data_bytes + 2);
    size_t n = boost::asio::read(serial_, boost::asio::buffer(out), ec);
    if (ec || n != data_bytes + 2) return false;

    // CRC は [address][cmd][data...] 全体に対して計算する
    std::vector<uint8_t> crc_input = {ROBOCLAW_ADDRESS, cmd};
    crc_input.insert(crc_input.end(), out.begin(), out.begin() + data_bytes);
    uint16_t expected = calculateCRC(crc_input);
    uint16_t received = (static_cast<uint16_t>(out[data_bytes]) << 8) | out[data_bytes + 1];
    return expected == received;
  }

  /// M1/M2 電流 [mA] (10mA 単位で返る)
  bool readMotorCurrents(float &m1_ma, float &m2_ma)
  {
    std::vector<uint8_t> out;
    if (!readCommand(CMD_READ_CURRENTS, out, 4)) return false;
    m1_ma = static_cast<float>((static_cast<uint16_t>(out[0]) << 8) | out[1]) * 10.0f;
    m2_ma = static_cast<float>((static_cast<uint16_t>(out[2]) << 8) | out[3]) * 10.0f;
    return true;
  }

  /// M1 または M2 実速度 [qpps]。方向バイト込みで符号付きで返す
  bool readMotorSpeed(uint8_t cmd, int32_t &speed_qpps)
  {
    std::vector<uint8_t> out;
    if (!readCommand(cmd, out, 5)) return false;  // 4 byte speed + 1 byte direction
    speed_qpps = (static_cast<int32_t>(out[0]) << 24) | (static_cast<int32_t>(out[1]) << 16) |
                 (static_cast<int32_t>(out[2]) << 8) | static_cast<int32_t>(out[3]);
    if (out[4] == 0) speed_qpps = -speed_qpps;  // 0=後退
    return true;
  }

  /// 基板温度 [°C]
  bool readTemperature(float &temp_c)
  {
    std::vector<uint8_t> out;
    if (!readCommand(CMD_READ_TEMPERATURE, out, 2)) return false;
    temp_c = static_cast<float>((static_cast<uint16_t>(out[0]) << 8) | out[1]) / 10.0f;
    return true;
  }

  /// メインバッテリ電圧 [V]
  bool readBatteryVoltage(float &volts)
  {
    std::vector<uint8_t> out;
    if (!readCommand(CMD_READ_MAIN_VOLTAGE, out, 2)) return false;
    volts = static_cast<float>((static_cast<uint16_t>(out[0]) << 8) | out[1]) / 10.0f;
    return true;
  }

  /// ステータスフラグ (32-bit)
  bool readStatus(uint32_t &status)
  {
    std::vector<uint8_t> out;
    if (!readCommand(CMD_READ_STATUS, out, 4)) return false;
    status = (static_cast<uint32_t>(out[0]) << 24) | (static_cast<uint32_t>(out[1]) << 16) |
             (static_cast<uint32_t>(out[2]) << 8) | static_cast<uint32_t>(out[3]);
    return true;
  }

  /// 全テレメトリをまとめて取得。読み取れなかった項目は前回値を保持
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

  static uint16_t calculateCRC(const std::vector<uint8_t> &data)
  {
    uint16_t crc = 0;
    for (auto byte : data)
    {
      crc ^= static_cast<uint16_t>(byte) << 8;
      for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
  }

  static void appendCRC(std::vector<uint8_t> &data)
  {
    uint16_t crc = calculateCRC(data);
    data.push_back(static_cast<uint8_t>(crc >> 8));
    data.push_back(static_cast<uint8_t>(crc & 0xFF));
  }

  static void appendInt32(std::vector<uint8_t> &data, int32_t value)
  {
    for (int i = 3; i >= 0; --i) data.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFF));
  }

  static void appendFloat32(std::vector<uint8_t> &data, float value)
  {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    for (int i = 3; i >= 0; --i) data.push_back(static_cast<uint8_t>((bits >> (8 * i)) & 0xFF));
  }
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

    // ── スタック保護パラメータ ────────────────────────────────────────────
    // 指令速度に対して実速度がこの割合以下ならスタックとみなす
    declare_parameter("stall_error_ratio", 0.5);
    // 指令速度がこの qpps 以下の場合はスタック判定しない (停止指令時の誤検知防止)
    declare_parameter("stall_cmd_threshold_qpps", 500);
    // スタックスコアがこの時間 [s] 分たまると最大絞りになる
    declare_parameter("stall_window_sec", 2.0);
    // スタック時の最大出力削減率 (0.5 = 最大50%削減)
    declare_parameter("stall_max_reduction", 0.5);

    // ── 電流制限 (Roboclaw ハードウェア保護) ─────────────────────────────
    // 0 を指定すると設定をスキップする
    declare_parameter("m1_max_current_ma", 15000.0);
    declare_parameter("m2_max_current_ma", 15000.0);

    // ── テレメトリ周期 ────────────────────────────────────────────────────
    declare_parameter("telemetry_hz", 10);
    // Roboclaw の通信タイムアウト (100ms 単位)。
    // 例: 5 = 500ms で指令断時にハード側が自動停止。
    declare_parameter("serial_timeout_ds", 5);
    // ── デバッグ出力 ──────────────────────────────────────────────────────
    declare_parameter("debug_io", false);
    declare_parameter("debug_log_period_ms", 200);
    // ── ハード接続リトライ間隔 ────────────────────────────────────────────
    declare_parameter("connect_retry_sec", 3.0);

    initParams();
    initPublishers();

    // サブスクリプションはハード未接続でも安全に設定できる
    subscription_ = create_subscription<custom_interfaces::msg::CrawlerVelocity>(
        "/crawler_driver", 10,
        std::bind(&CrawlerDriver::driver_callback, this, std::placeholders::_1));

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&CrawlerDriver::cmd_vel_callback, this, std::placeholders::_1));

    watchdog_timer_ = create_wall_timer(std::chrono::milliseconds(watchdog_timeout_ms_),
                                        std::bind(&CrawlerDriver::watchdog_callback, this));

    const int telem_ms = 1000 / get_parameter("telemetry_hz").as_int();
    telemetry_timer_ = create_wall_timer(std::chrono::milliseconds(telem_ms),
                                         std::bind(&CrawlerDriver::telemetry_callback, this));

    auto estop_qos = rclcpp::QoS(1).transient_local().reliable();
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop", estop_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
          const bool was_estop = estop_active_;
          estop_active_ = msg->data;
          if (estop_active_ && !was_estop)
          {
            if (hw_connected_) stopMotors();
            RCLCPP_FATAL(get_logger(), "⛔ EMERGENCY STOP — motors halted");
          }
          else if (!estop_active_ && was_estop)
          {
            RCLCPP_WARN(get_logger(), "✅ E-STOP cleared");
          }
        });

    // ハードウェア接続をリトライ付きで試みる
    const double retry_sec = get_parameter("connect_retry_sec").as_double();
    connect_timer_ = create_wall_timer(std::chrono::duration<double>(retry_sec),
                                       std::bind(&CrawlerDriver::tryConnect, this));
    tryConnect();
  }

  ~CrawlerDriver() override
  {
    if (hw_connected_) stopMotors();
  }

 private:
  RoboclawDriver roboclaw_;

  // ── パラメータ ────────────────────────────────────────────────────────────
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

  double stall_error_ratio_;
  int stall_cmd_threshold_qpps_;
  double stall_window_sec_;
  double stall_max_reduction_;

  // ── 状態 ──────────────────────────────────────────────────────────────────
  rclcpp::Time last_cmd_time_;
  bool estop_active_ = false;
  bool hw_connected_ = false;
  StallState m1_stall_, m2_stall_;
  int32_t m1_cmd_qpps_ = 0, m2_cmd_qpps_ = 0;
  RoboclawTelemetry telemetry_;

  // ── タイマー / サブスクリプション ─────────────────────────────────────────
  rclcpp::Subscription<custom_interfaces::msg::CrawlerVelocity>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp::TimerBase::SharedPtr connect_timer_;

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

  // ─────────────────────────────────────────────────────────────────────────

  void initParams()
  {
    const double circumference = get_parameter("crawler_circumference").as_double();
    const int counts_per_rev = get_parameter("counts_per_rev").as_int();
    const int gearhead_ratio = get_parameter("gearhead_ratio").as_int();
    const int pulley_ratio = get_parameter("pulley_ratio").as_int();
    watchdog_timeout_ms_ = get_parameter("watchdog_timeout_ms").as_int();
    baud_rate_ = get_parameter("baud_rate").as_int();
    debug_io_ = get_parameter("debug_io").as_bool();
    debug_log_period_ms_ = get_parameter("debug_log_period_ms").as_int();
    track_width_ = get_parameter("track_width").as_double();

    m1_kp_ = get_parameter("m1_kp").as_double();
    m1_ki_ = get_parameter("m1_ki").as_double();
    m1_kd_ = get_parameter("m1_kd").as_double();
    m1_qpps_ = get_parameter("m1_qpps").as_int();
    m2_kp_ = get_parameter("m2_kp").as_double();
    m2_ki_ = get_parameter("m2_ki").as_double();
    m2_kd_ = get_parameter("m2_kd").as_double();
    m2_qpps_ = get_parameter("m2_qpps").as_int();

    stall_error_ratio_ = get_parameter("stall_error_ratio").as_double();
    stall_cmd_threshold_qpps_ = get_parameter("stall_cmd_threshold_qpps").as_int();
    stall_window_sec_ = get_parameter("stall_window_sec").as_double();
    stall_max_reduction_ = get_parameter("stall_max_reduction").as_double();

    counts_per_meter_ = (counts_per_rev * gearhead_ratio * pulley_ratio) / circumference;
    last_cmd_time_ = now();

    if (baud_rate_ <= 0)
    {
      RCLCPP_WARN(get_logger(), "Invalid baud_rate=%d. Falling back to %d", baud_rate_,
                  DEFAULT_SERIAL_BAUD_RATE);
      baud_rate_ = DEFAULT_SERIAL_BAUD_RATE;
    }

    if (debug_log_period_ms_ <= 0) debug_log_period_ms_ = 200;
  }

  void tryConnect()
  {
    if (hw_connected_) return;

    const std::string port = get_parameter("serial_port").as_string();
    if (!roboclaw_.connect(port, baud_rate_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Failed to connect to Roboclaw on %s @ %d bps. Retrying...", port.c_str(),
                           baud_rate_);
      return;
    }

    if (debug_io_)
    {
      RCLCPP_INFO(get_logger(), "[debug_io] Roboclaw link established: port=%s baud=%d", port.c_str(),
                  baud_rate_);
    }

    hw_connected_ = true;
    connect_timer_->cancel();
    last_cmd_time_ = now();
    initHardware();
    RCLCPP_INFO(get_logger(), "Connected to Roboclaw on %s @ %d bps", port.c_str(), baud_rate_);
  }

  void initHardware()
  {
    roboclaw_.setMotorVelocity(CMD_M1_VELOCITY, 0);
    roboclaw_.setMotorVelocity(CMD_M2_VELOCITY, 0);
    roboclaw_.setPIDConstants(CMD_SET_M1_PID, m1_kp_, m1_ki_, m1_kd_, m1_qpps_);
    roboclaw_.setPIDConstants(CMD_SET_M2_PID, m2_kp_, m2_ki_, m2_kd_, m2_qpps_);
    roboclaw_.resetEncoders();

    // ハードウェア電流制限 (0 はスキップ)
    const float m1_max_ma = static_cast<float>(get_parameter("m1_max_current_ma").as_double());
    const float m2_max_ma = static_cast<float>(get_parameter("m2_max_current_ma").as_double());
    const int serial_timeout_ds = get_parameter("serial_timeout_ds").as_int();

    if (serial_timeout_ds >= 0 && serial_timeout_ds <= 255)
    {
      const bool ok = roboclaw_.setSerialTimeoutDs(static_cast<uint8_t>(serial_timeout_ds));
      if (ok)
      {
        RCLCPP_INFO(get_logger(), "Roboclaw serial timeout set to %d ds (%d ms)", serial_timeout_ds,
                    serial_timeout_ds * 100);
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Failed to set Roboclaw serial timeout");
      }
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Invalid serial_timeout_ds=%d. Skipping Roboclaw timeout setup.",
                  serial_timeout_ds);
    }

    if (m1_max_ma > 0.0f)
    {
      roboclaw_.setMaxCurrent(CMD_SET_M1_MAX_CURRENT, m1_max_ma);
      RCLCPP_INFO(get_logger(), "M1 max current set to %.0f mA", m1_max_ma);
    }
    if (m2_max_ma > 0.0f)
    {
      roboclaw_.setMaxCurrent(CMD_SET_M2_MAX_CURRENT, m2_max_ma);
      RCLCPP_INFO(get_logger(), "M2 max current set to %.0f mA", m2_max_ma);
    }

    RCLCPP_INFO(get_logger(), "Hardware initialized");
  }

  void initPublishers()
  {
    const std::string prefix = "~/diagnostics/";
    pub_m1_current_ = create_publisher<std_msgs::msg::Float64>(prefix + "m1_current_ma", 10);
    pub_m2_current_ = create_publisher<std_msgs::msg::Float64>(prefix + "m2_current_ma", 10);
    pub_m1_speed_actual_ = create_publisher<std_msgs::msg::Float64>(prefix + "m1_speed_actual", 10);
    pub_m2_speed_actual_ = create_publisher<std_msgs::msg::Float64>(prefix + "m2_speed_actual", 10);
    pub_temperature_ = create_publisher<std_msgs::msg::Float64>(prefix + "temperature_c", 10);
    pub_battery_voltage_ = create_publisher<std_msgs::msg::Float64>(prefix + "battery_volts", 10);
    pub_status_flags_ = create_publisher<std_msgs::msg::UInt32>(prefix + "status_flags", 10);
    pub_m1_stall_scale_ = create_publisher<std_msgs::msg::Float64>(prefix + "m1_stall_scale", 10);
    pub_m2_stall_scale_ = create_publisher<std_msgs::msg::Float64>(prefix + "m2_stall_scale", 10);
  }

  // ── テレメトリ取得 + publish + 警告ログ ──────────────────────────────────
  void telemetry_callback()
  {
    if (!hw_connected_) return;

    telemetry_ = roboclaw_.readTelemetry();
    if (!telemetry_.valid) return;

    auto pub_f64 = [](auto &pub, double val)
    {
      std_msgs::msg::Float64 msg;
      msg.data = val;
      pub->publish(msg);
    };

    pub_f64(pub_m1_current_, telemetry_.m1_current_ma);
    pub_f64(pub_m2_current_, telemetry_.m2_current_ma);
    pub_f64(pub_m1_speed_actual_, static_cast<double>(telemetry_.m1_speed_qpps));
    pub_f64(pub_m2_speed_actual_, static_cast<double>(telemetry_.m2_speed_qpps));
    pub_f64(pub_temperature_, telemetry_.temperature_c);
    pub_f64(pub_battery_voltage_, telemetry_.battery_volts);

    {
      std_msgs::msg::UInt32 msg;
      msg.data = telemetry_.status_flags;
      pub_status_flags_->publish(msg);
    }

    // ステータスフラグ警告ログ
    const auto &sf = telemetry_.status_flags;
    if (sf & STATUS_TEMP_ERROR)
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "🌡️  Roboclaw temperature error (%.1f°C)", telemetry_.temperature_c);
    else if (sf & STATUS_TEMP_WARNING)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "🌡️  Roboclaw temperature warning (%.1f°C)", telemetry_.temperature_c);
    if (sf & STATUS_M1_OVERCURRENT)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "⚡ M1 overcurrent (%.0f mA)",
                           telemetry_.m1_current_ma);
    if (sf & STATUS_M2_OVERCURRENT)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "⚡ M2 overcurrent (%.0f mA)",
                           telemetry_.m2_current_ma);
    if (sf & STATUS_MAIN_BATT_LOW)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "🔋 Main battery low (%.1f V)",
                           telemetry_.battery_volts);
    if (sf & STATUS_M1_FAULT)
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "💥 M1 driver fault");
    if (sf & STATUS_M2_FAULT)
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "💥 M2 driver fault");
  }

  // ── スタック検出: 速度誤差スコアで出力スケールを更新 ──────────────────────
  // 実速度は telemetry_ から読む (telemetry_callback が別タイマーで更新)
  void updateStall(StallState &state, int32_t cmd_qpps, int32_t actual_qpps, double dt)
  {
    const bool cmd_significant = std::abs(cmd_qpps) > stall_cmd_threshold_qpps_;
    const bool stalling = cmd_significant && (std::abs(cmd_qpps - actual_qpps) >
                                              std::abs(cmd_qpps) * stall_error_ratio_);

    if (stalling)
      state.score = std::min(state.score + dt, stall_window_sec_);
    else
      state.score = std::max(state.score - dt * 2.0, 0.0);  // 回復は2倍速

    const double ratio = state.score / stall_window_sec_;
    state.scale = 1.0 - ratio * stall_max_reduction_;

    if (stalling && ratio > 0.8)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "⚠️  Motor stall detected (scale=%.2f)",
                           state.scale);
  }

  void stopMotors()
  {
    if (!hw_connected_) return;
    const bool ok1 = roboclaw_.setMotorVelocity(CMD_M1_VELOCITY, 0);
    const bool ok2 = roboclaw_.setMotorVelocity(CMD_M2_VELOCITY, 0);
    if (!ok1 || !ok2)
    {
      handleCommFailure("failed to send zero-velocity stop command");
    }
  }

  void sendMotorCommands(double m1_vel, double m2_vel)
  {
    if (!hw_connected_) return;

    m1_cmd_qpps_ = static_cast<int32_t>(m1_vel * counts_per_meter_);
    m2_cmd_qpps_ = static_cast<int32_t>(m2_vel * counts_per_meter_);

    // スタック検出 (テレメトリが有効な場合のみ)
    const double dt = watchdog_timeout_ms_ / 1000.0;  // 近似 dt
    if (telemetry_.valid)
    {
      updateStall(m1_stall_, m1_cmd_qpps_, telemetry_.m1_speed_qpps, dt);
      updateStall(m2_stall_, m2_cmd_qpps_, telemetry_.m2_speed_qpps, dt);
    }

    // stall scale を publish
    {
      std_msgs::msg::Float64 msg;
      msg.data = m1_stall_.scale;
      pub_m1_stall_scale_->publish(msg);
      msg.data = m2_stall_.scale;
      pub_m2_stall_scale_->publish(msg);
    }

    const auto m1 = static_cast<int32_t>(m1_cmd_qpps_ * m1_stall_.scale);
    const auto m2 = static_cast<int32_t>(m2_cmd_qpps_ * m2_stall_.scale);

    if (debug_io_)
    {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), debug_log_period_ms_,
          "[debug_io] cmd_vel=(%.3f, %.3f) qpps_raw=(%d, %d) qpps_out=(%d, %d) actual=(%d, %d)"
          " stall=(%.2f, %.2f)",
          m1_vel, m2_vel, m1_cmd_qpps_, m2_cmd_qpps_, m1, m2, telemetry_.m1_speed_qpps,
          telemetry_.m2_speed_qpps, m1_stall_.scale, m2_stall_.scale);
    }

    const bool ok1 = roboclaw_.setMotorVelocity(CMD_M1_VELOCITY, m1);
    const bool ok2 = roboclaw_.setMotorVelocity(CMD_M2_VELOCITY, m2);
    if (!ok1) RCLCPP_ERROR(get_logger(), "Failed to send M1 command");
    if (!ok2) RCLCPP_ERROR(get_logger(), "Failed to send M2 command");
    if (!ok1 || !ok2)
    {
      handleCommFailure("motor command write failure");
    }
  }

  void driver_callback(const custom_interfaces::msg::CrawlerVelocity &msg)
  {
    if (estop_active_) return;
    last_cmd_time_ = now();
    sendMotorCommands(msg.m1_vel, msg.m2_vel);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist &msg)
  {
    if (estop_active_) return;
    last_cmd_time_ = now();
    const double half_track = track_width_ / 2.0;
    sendMotorCommands(msg.linear.x - msg.angular.z * half_track,
                      msg.linear.x + msg.angular.z * half_track);
  }

  void watchdog_callback()
  {
    if (!hw_connected_) return;

    const double elapsed = (now() - last_cmd_time_).seconds() * 1000.0;
    if (elapsed > watchdog_timeout_ms_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Watchdog: no command for %.0f ms, stopping motors", elapsed);
      stopMotors();
    }
  }

  void handleCommFailure(const char *reason)
  {
    if (!hw_connected_) return;

    const int32_t last_m1_cmd_qpps = m1_cmd_qpps_;
    const int32_t last_m2_cmd_qpps = m2_cmd_qpps_;
    const int32_t last_m1_actual_qpps = telemetry_.m1_speed_qpps;
    const int32_t last_m2_actual_qpps = telemetry_.m2_speed_qpps;

    // 片側だけ指令が通っているケースを減らすため、切断前にベストエフォート停止。
    (void)roboclaw_.setMotorVelocity(CMD_M1_VELOCITY, 0);
    (void)roboclaw_.setMotorVelocity(CMD_M2_VELOCITY, 0);

    hw_connected_ = false;
    telemetry_.valid = false;
    m1_cmd_qpps_ = 0;
    m2_cmd_qpps_ = 0;
    m1_stall_ = StallState{};
    m2_stall_ = StallState{};

    roboclaw_.disconnect();
    if (connect_timer_) connect_timer_->reset();

    RCLCPP_ERROR(get_logger(),
                 "Communication with Roboclaw lost (%s). Entering fail-safe and retrying reconnect.",
                 reason);

    if (debug_io_)
    {
      RCLCPP_ERROR(get_logger(),
                   "[debug_io] link dropped: last_cmd_qpps=(%d, %d) last_actual_qpps=(%d, %d)",
                   last_m1_cmd_qpps, last_m2_cmd_qpps, last_m1_actual_qpps, last_m2_actual_qpps);
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrawlerDriver>());
  rclcpp::shutdown();
  return 0;
}
