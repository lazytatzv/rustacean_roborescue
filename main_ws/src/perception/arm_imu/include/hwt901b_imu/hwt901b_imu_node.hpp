#ifndef HWT901B_IMU__HWT901B_IMU_NODE_HPP_
#define HWT901B_IMU__HWT901B_IMU_NODE_HPP_

#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <thread>

#include <boost/asio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

class HWT901BIMUNode : public rclcpp::Node
{
public:
  HWT901BIMUNode();
  ~HWT901BIMUNode() override;

private:
  static constexpr uint8_t IMU_HEADER = 0x55;
  static constexpr uint8_t MAGNETIC_FIELD = 0x54;
  static constexpr size_t FRAME_SIZE = 11;

  static constexpr double GAUSS_TO_TESLA = 1e-4;

  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_;

  std::string port_name_;
  std::string frame_id_;
  int baud_rate_;
  double mag_scale_;

  std::deque<uint8_t> rx_buffer_;
  std::array<uint8_t, 256> read_buf_{};
  std::thread io_thread_;

  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;

  void load_parameters();
  bool configure_serial();
  void start_io_thread();
  void stop_io_thread();
  void start_async_read();
  void on_read(const boost::system::error_code & ec, std::size_t n);
  void consume_frame();

  static int16_t to_int16(uint8_t low, uint8_t high);
  static bool valid_checksum(const std::array<uint8_t, FRAME_SIZE> & frame);

  void parse_frames();
  void decode_frame(const std::array<uint8_t, FRAME_SIZE> & frame);
};

#endif  // HWT901B_IMU__HWT901B_IMU_NODE_HPP_
