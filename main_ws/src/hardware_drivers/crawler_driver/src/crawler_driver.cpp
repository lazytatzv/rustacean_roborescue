#include <boost/asio.hpp>
#include <chrono>
#include <cstring>
#include <custom_interfaces/msg/crawler_velocity.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;

// ──────────────────────────────────────────────
// Roboclaw protocol constants
// ──────────────────────────────────────────────
constexpr uint8_t ROBOCLAW_ADDRESS             = 0x80;
constexpr uint8_t M1_MOTOR_COMMAND             = 35;
constexpr uint8_t M2_MOTOR_COMMAND             = 36;
constexpr uint8_t M1_SET_PID_CONSTANTS_COMMAND = 28;
constexpr uint8_t M2_SET_PID_CONSTANTS_COMMAND = 29;
constexpr int     SERIAL_BAUD_RATE             = 38400; // 115200だと安定しない可能性もある
constexpr uint8_t RESET_QUAD_ENCODER           = 20;
constexpr int     M1_QPPS                      = 53250; // これは実測する必要がある
constexpr int     M2_QPPS                      = 50062;

// ──────────────────────────────────────────────
// Synchronous Roboclaw serial driver
// ──────────────────────────────────────────────
class RoboclawDriver
{
 public:
  explicit RoboclawDriver(const std::string &port)
      : io_(), serial_(io_, port)
  {
    using spb = boost::asio::serial_port_base;
    serial_.set_option(spb::baud_rate(SERIAL_BAUD_RATE));
    serial_.set_option(spb::character_size(8));
    serial_.set_option(spb::parity(spb::parity::none));
    serial_.set_option(spb::stop_bits(spb::stop_bits::one));
    serial_.set_option(spb::flow_control(spb::flow_control::none));
  }

  /// Send a command and wait for the 1-byte ACK. Returns true on success.
  bool sendCommand(const std::vector<uint8_t> &data)
  {
    boost::system::error_code ec;
    boost::asio::write(serial_, boost::asio::buffer(data), ec);
    if (ec)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RoboclawDriver"),
                   "Serial write error: %s", ec.message().c_str());
      return false;
    }

    uint8_t ack = 0;
    boost::asio::read(serial_, boost::asio::buffer(&ack, 1), ec);
    if (ec)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RoboclawDriver"),
                   "Serial read error: %s", ec.message().c_str());
      return false;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("RoboclawDriver"),
                 "ACK: 0x%02X", ack);
    return true;
  }

  bool setMotorVelocity(uint8_t command, int32_t counts_per_sec)
  {
    std::vector<uint8_t> data = {ROBOCLAW_ADDRESS, command};
    appendInt32(data, counts_per_sec);
    appendCRC(data);
    return sendCommand(data);
  }

  bool setPIDConstants(uint8_t command, float K_p, float K_i, float K_d, int qpps)
  {
    std::vector<uint8_t> data = {ROBOCLAW_ADDRESS, command};
    appendFloat32(data, K_d);
    appendFloat32(data, K_p);
    appendFloat32(data, K_i);
    appendInt32(data, qpps);
    appendCRC(data);
    return sendCommand(data);
  }

  bool resetEncoders()
  {
    std::vector<uint8_t> data = {ROBOCLAW_ADDRESS, RESET_QUAD_ENCODER};
    appendCRC(data);
    return sendCommand(data);
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
      for (int i = 0; i < 8; i++)
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
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
    for (int i = 3; i >= 0; --i)
      data.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFF));
  }

  static void appendFloat32(std::vector<uint8_t> &data, float value)
  {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    for (int i = 3; i >= 0; --i)
      data.push_back(static_cast<uint8_t>((bits >> (8 * i)) & 0xFF));
  }
};

// ──────────────────────────────────────────────
// ROS 2 Crawler Driver Node
// ──────────────────────────────────────────────
class CrawlerDriver : public rclcpp::Node
{
 public:
  CrawlerDriver()
      : Node("crawler_driver"),
        roboclaw_(this->declare_parameter<std::string>("serial_port", "/dev/roboclaw"))
  {
    declare_parameter("crawler_circumference", 0.39);
    declare_parameter("counts_per_rev", 256);
    declare_parameter("gearhead_ratio", 66);
    declare_parameter("pulley_ratio", 2);
    declare_parameter("watchdog_timeout_ms", 500);
    declare_parameter("track_width", 0.4);  // クローラ間距離 [m]

    initParams();
    initHardware();

    subscription_ = create_subscription<custom_interfaces::msg::CrawlerVelocity>(
        "/crawler_driver", 10,
        std::bind(&CrawlerDriver::driver_callback, this, std::placeholders::_1));

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&CrawlerDriver::cmd_vel_callback, this, std::placeholders::_1));

    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(watchdog_timeout_ms_),
        std::bind(&CrawlerDriver::watchdog_callback, this));
  }

 private:
  RoboclawDriver roboclaw_;
  double         crawler_circumference_;
  int            counts_per_rev_;
  int            gearhead_ratio_;
  int            pulley_ratio_;
  double         counts_per_meter_;
  int            watchdog_timeout_ms_;
  double         track_width_;

  rclcpp::Subscription<custom_interfaces::msg::CrawlerVelocity>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_cmd_time_;

  double velocity_to_counts_per_sec(double velocity) const
  {
    return velocity * counts_per_meter_;
  }

  void initParams()
  {
    crawler_circumference_ = get_parameter("crawler_circumference").as_double();
    counts_per_rev_        = get_parameter("counts_per_rev").as_int();
    gearhead_ratio_        = get_parameter("gearhead_ratio").as_int();
    pulley_ratio_          = get_parameter("pulley_ratio").as_int();
    watchdog_timeout_ms_   = get_parameter("watchdog_timeout_ms").as_int();
    track_width_            = get_parameter("track_width").as_double();
    counts_per_meter_ =
        (counts_per_rev_ * gearhead_ratio_ * pulley_ratio_) / crawler_circumference_;
    last_cmd_time_ = now();
  }

  void initHardware()
  {
    if (!roboclaw_.setMotorVelocity(M1_MOTOR_COMMAND, 0))
      RCLCPP_ERROR(get_logger(), "Failed to initialize M1 motor");
    if (!roboclaw_.setMotorVelocity(M2_MOTOR_COMMAND, 0))
      RCLCPP_ERROR(get_logger(), "Failed to initialize M2 motor");

    if (!roboclaw_.setPIDConstants(M1_SET_PID_CONSTANTS_COMMAND, 0.464f, 0.021f, 0.0f, M1_QPPS))
      RCLCPP_ERROR(get_logger(), "Failed to set PID for M1");
    if (!roboclaw_.setPIDConstants(M2_SET_PID_CONSTANTS_COMMAND, 0.438f, 0.020f, 0.0f, M2_QPPS))
      RCLCPP_ERROR(get_logger(), "Failed to set PID for M2");

    if (!roboclaw_.resetEncoders())
      RCLCPP_ERROR(get_logger(), "Failed to reset encoders");

    RCLCPP_INFO(get_logger(), "Hardware initialized");
  }

  void stopMotors()
  {
    roboclaw_.setMotorVelocity(M1_MOTOR_COMMAND, 0);
    roboclaw_.setMotorVelocity(M2_MOTOR_COMMAND, 0);
  }

  void sendMotorCommands(double m1_vel, double m2_vel)
  {
    auto m1 = static_cast<int32_t>(velocity_to_counts_per_sec(m1_vel));
    auto m2 = static_cast<int32_t>(velocity_to_counts_per_sec(m2_vel));

    if (!roboclaw_.setMotorVelocity(M1_MOTOR_COMMAND, m1))
      RCLCPP_ERROR(get_logger(), "Failed to send M1 command");
    if (!roboclaw_.setMotorVelocity(M2_MOTOR_COMMAND, m2))
      RCLCPP_ERROR(get_logger(), "Failed to send M2 command");
  }

  void driver_callback(const custom_interfaces::msg::CrawlerVelocity &msg)
  {
    last_cmd_time_ = now();
    sendMotorCommands(msg.m1_vel, msg.m2_vel);
  }

  /// Nav2 の /cmd_vel → 差動駆動変換
  void cmd_vel_callback(const geometry_msgs::msg::Twist &msg)
  {
    last_cmd_time_ = now();

    double linear  = msg.linear.x;
    double angular = msg.angular.z;
    double half_track = track_width_ / 2.0;

    double m1_vel = linear - angular * half_track;  // 左
    double m2_vel = linear + angular * half_track;  // 右

    sendMotorCommands(m1_vel, m2_vel);
  }

  void watchdog_callback()
  {
    auto elapsed = (now() - last_cmd_time_).seconds() * 1000.0;
    if (elapsed > watchdog_timeout_ms_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                            "Watchdog: no command for %.0f ms, stopping motors", elapsed);
      stopMotors();
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
