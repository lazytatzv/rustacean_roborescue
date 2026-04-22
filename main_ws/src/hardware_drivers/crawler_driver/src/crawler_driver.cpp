#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <custom_interfaces/msg/crawler_velocity.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ──────────────────────────────────────────────
// Roboclaw Constants
// ──────────────────────────────────────────────
constexpr uint8_t ADDR = 0x80;
constexpr uint8_t M1_SPEED_CMD = 35;
constexpr uint8_t M2_SPEED_CMD = 36;
constexpr uint8_t GET_ENCODERS = 78; // Read both M1/M2 encoders

class SimpleRoboclaw
{
 public:
  SimpleRoboclaw(const std::string &port, int baud) : io_(), serial_(io_)
  {
    serial_.open(port);
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    struct termios tty;
    tcgetattr(serial_.lowest_layer().native_handle(), &tty);
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;
    tcsetattr(serial_.lowest_layer().native_handle(), TCSANOW, &tty);
  }

  void sendCommand(uint8_t cmd, int32_t qpps)
  {
    std::vector<uint8_t> pkt;
    pkt.push_back(ADDR);
    pkt.push_back(cmd);
    pkt.push_back(static_cast<uint8_t>((qpps >> 24) & 0xFF));
    pkt.push_back(static_cast<uint8_t>((qpps >> 16) & 0xFF));
    pkt.push_back(static_cast<uint8_t>((qpps >> 8) & 0xFF));
    pkt.push_back(static_cast<uint8_t>(qpps & 0xFF));

    uint16_t crc = calculateCRC(pkt);
    pkt.push_back(static_cast<uint8_t>(crc >> 8));
    pkt.push_back(static_cast<uint8_t>(crc & 0xFF));

    try {
      boost::asio::write(serial_, boost::asio::buffer(pkt));
      uint8_t ack = 0;
      boost::asio::read(serial_, boost::asio::buffer(&ack, 1));
    } catch (...) {}
  }

  bool readEncoders(int32_t &m1, int32_t &m2)
  {
    std::vector<uint8_t> pkt = {ADDR, GET_ENCODERS};
    try {
      boost::asio::write(serial_, boost::asio::buffer(pkt));
      std::array<uint8_t, 10> resp; // 4(m1) + 1(status) + 4(m2) + 1(status)
      size_t n = boost::asio::read(serial_, boost::asio::buffer(resp));
      if (n < 10) return false;

      m1 = (resp[0] << 24) | (resp[1] << 16) | (resp[2] << 8) | resp[3];
      m2 = (resp[5] << 24) | (resp[6] << 16) | (resp[7] << 8) | resp[8];
      return true;
    } catch (...) {
      return false;
    }
  }

 private:
  uint16_t calculateCRC(const std::vector<uint8_t> &pkt)
  {
    uint16_t crc = 0;
    for (auto b : pkt) {
      crc ^= (static_cast<uint16_t>(b) << 8);
      for (int j = 0; j < 8; j++) {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
      }
    }
    return crc;
  }

  boost::asio::io_context io_;
  boost::asio::serial_port serial_;
};

class CrawlerDriver : public rclcpp::Node
{
 public:
  CrawlerDriver() : Node("crawler_driver")
  {
    declare_parameter<std::string>("serial_port", "/dev/roboclaw");
    declare_parameter<double>("counts_per_rev", 2000.0);
    declare_parameter<int>("m1_qpps", 61000);
    declare_parameter<int>("m2_qpps", 61000);
    declare_parameter<double>("track_width", 0.4);
    declare_parameter<int>("m1_direction_sign", -1);
    declare_parameter<int>("m2_direction_sign", -1);
    declare_parameter<double>("counts_per_meter", 86646.0);

    counts_per_rev_ = get_parameter("counts_per_rev").as_double();
    m1_qpps_limit_ = get_parameter("m1_qpps").as_int();
    m2_qpps_limit_ = get_parameter("m2_qpps").as_int();
    track_width_ = get_parameter("track_width").as_double();
    m1_sign_ = get_parameter("m1_direction_sign").as_int();
    m2_sign_ = get_parameter("m2_direction_sign").as_int();
    counts_per_meter_ = get_parameter("counts_per_meter").as_double();

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    sub_raw_ = create_subscription<custom_interfaces::msg::CrawlerVelocity>(
        "/crawler_driver", 10, [this](const custom_interfaces::msg::CrawlerVelocity::SharedPtr msg)
        { this->process_velocity(msg->m1_vel, msg->m2_vel); });

    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
          const double half = this->track_width_ / 2.0;
          double m1_v = msg->linear.x - msg->angular.z * half;
          double m2_v = msg->linear.x + msg->angular.z * half;
          this->process_velocity(m1_v, m2_v);
        });

    running_ = true;
    worker_ = std::thread([this]() { this->worker_loop(); });
    
    RCLCPP_INFO(this->get_logger(), "Crawler Driver started (Waiting for Roboclaw)");
  }

  ~CrawlerDriver()
  {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    if (roboclaw_) {
      roboclaw_->sendCommand(M1_SPEED_CMD, 0);
      roboclaw_->sendCommand(M2_SPEED_CMD, 0);
      RCLCPP_INFO(this->get_logger(), "🛑 Crawler stopped.");
    }
  }

 private:
  void worker_loop()
  {
    std::string port = get_parameter("serial_port").as_string();

    while (this->running_) {
      if (!this->roboclaw_) {
        if (!rcpputils::fs::exists(port)) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "🔍 [Crawler] Port %s NOT FOUND. Check USB.", port.c_str());
        } else {
          try {
            roboclaw_ = std::make_unique<SimpleRoboclaw>(port, 38400);
            RCLCPP_INFO(this->get_logger(), "✅ [Crawler] Connected to Roboclaw on %s", port.c_str());
          } catch (...) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
              "❌ [Crawler] Failed to open %s. Permission?", port.c_str());
          }
        }
        if (!this->roboclaw_) {
          std::this_thread::sleep_for(std::chrono::seconds(2));
          continue;
        }
      }

      int32_t cmd_m1, cmd_m2;
      bool has_cmd = false;
      {
        std::lock_guard<std::mutex> lk(this->mtx_);
        if (this->has_new_cmd_) {
          cmd_m1 = this->latest_m1_;
          cmd_m2 = this->latest_m2_;
          this->has_new_cmd_ = false;
          has_cmd = true;
        }
      }

      if (has_cmd) {
        this->roboclaw_->sendCommand(M1_SPEED_CMD, cmd_m1);
        this->roboclaw_->sendCommand(M2_SPEED_CMD, cmd_m2);
      }

      int32_t enc_m1, enc_m2;
      if (this->roboclaw_->readEncoders(enc_m1, enc_m2)) {
        auto js = sensor_msgs::msg::JointState();
        js.header.stamp = this->now();
        js.name = {"left_wheel_joint", "right_wheel_joint"};
        double rad_m1 = (enc_m1 * m1_sign_) * (2.0 * M_PI / counts_per_rev_);
        double rad_m2 = (enc_m2 * m2_sign_) * (2.0 * M_PI / counts_per_rev_);
        js.position = {rad_m1, rad_m2};
        this->joint_state_pub_->publish(js);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void process_velocity(double m1_vel, double m2_vel)
  {
    int32_t m1 = static_cast<int32_t>(m1_vel * counts_per_meter_) * m1_sign_;
    int32_t m2 = static_cast<int32_t>(m2_vel * counts_per_meter_) * m2_sign_;
    m1 = std::max<int32_t>(-m1_qpps_limit_, std::min<int32_t>(m1, m1_qpps_limit_));
    m2 = std::max<int32_t>(-m2_qpps_limit_, std::min<int32_t>(m2, m2_qpps_limit_));
    {
      std::lock_guard<std::mutex> lk(mtx_);
      latest_m1_ = m1; latest_m2_ = m2;
      has_new_cmd_ = true;
    }
  }

  std::unique_ptr<SimpleRoboclaw> roboclaw_;
  double counts_per_meter_, track_width_, counts_per_rev_;
  int32_t m1_qpps_limit_, m2_qpps_limit_, m1_sign_, m2_sign_;
  std::thread worker_;
  std::atomic<bool> running_;
  std::mutex mtx_;
  int32_t latest_m1_ = 0, latest_m2_ = 0;
  bool has_new_cmd_ = false;
  rclcpp::Subscription<custom_interfaces::msg::CrawlerVelocity>::SharedPtr sub_raw_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrawlerDriver>());
  rclcpp::shutdown();
  return 0;
}
