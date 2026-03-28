#ifndef HWT901B_IMU__HWT901B_IMU_NODE_HPP_
#define HWT901B_IMU__HWT901B_IMU_NODE_HPP_

#include <array>
#include <cstdint>
#include <deque>
#include <string>

#include <boost/asio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

class HWT901BIMUNode : public rclcpp::Node
{
public:
  HWT901BIMUNode();

private:
  static constexpr uint8_t IMU_HEADER = 0x55;
  static constexpr uint8_t ACCELERATION = 0x51;
  static constexpr uint8_t ANGULAR_VELOCITY = 0x52;
  static constexpr uint8_t ANGLE = 0x53;
  static constexpr uint8_t MAGNETIC_FIELD = 0x54;
  static constexpr uint8_t QUATERNION_CMD = 0x59;
  static constexpr size_t FRAME_SIZE = 11;

  static constexpr double G = 9.80665;
  static constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
  static constexpr double GAUSS_TO_TESLA = 1e-4;

  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_;

  std::string port_name_;
  std::string frame_id_;
  int baud_rate_;
  int poll_interval_ms_;

  std::deque<uint8_t> rx_buffer_;

  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::MagneticField mag_msg_;
  bool imu_received_ = false;
  bool mag_received_ = false;

  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;

  void init_messages();
  void open_serial();
  void poll_serial();

  static int16_t to_int16(uint8_t low, uint8_t high);
  static bool valid_checksum(const std::array<uint8_t, FRAME_SIZE> & frame);

  void parse_frames();
  void stamp_headers();
  void decode_frame(const std::array<uint8_t, FRAME_SIZE> & frame);
};

#endif  // HWT901B_IMU__HWT901B_IMU_NODE_HPP_
