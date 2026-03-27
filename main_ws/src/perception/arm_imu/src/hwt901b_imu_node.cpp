#include "hwt901b_imu/hwt901b_imu_node.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <chrono>

HWT901BIMUNode::HWT901BIMUNode()
: Node("hwt901b_imu_node"), serial_(io_context_)
{
  port_name_ = this->declare_parameter<std::string>("port", "/dev/hwt901b");
  baud_rate_ = this->declare_parameter<int>("baud_rate", 9600);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "hwt901b_link");
  poll_interval_ms_ = this->declare_parameter<int>("poll_interval_ms", 5);

  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/hwt901b/imu", 10);
  mag_publisher_ =
    this->create_publisher<sensor_msgs::msg::MagneticField>("/hwt901b/magnetic_field", 10);

  init_messages();
  open_serial();

  poll_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(poll_interval_ms_),
    std::bind(&HWT901BIMUNode::poll_serial, this));

  RCLCPP_INFO(
    this->get_logger(), "HWT901B IMU node started: port=%s baud=%d", port_name_.c_str(),
    baud_rate_);
}

void HWT901BIMUNode::init_messages()
{
  imu_msg_.orientation_covariance[0] = -1.0;
}

void HWT901BIMUNode::open_serial()
{
  boost::system::error_code ec;
  serial_.open(port_name_, ec);
  if (ec) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to open %s: %s", port_name_.c_str(), ec.message().c_str());
    return;
  }

  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  serial_.set_option(boost::asio::serial_port_base::character_size(8));
  serial_.set_option(
    boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  serial_.set_option(
    boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  serial_.set_option(
    boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  serial_.non_blocking(true);
}

void HWT901BIMUNode::poll_serial()
{
  if (!serial_.is_open()) {
    return;
  }

  std::array<uint8_t, 256> buf{};
  boost::system::error_code ec;
  const auto n = serial_.read_some(boost::asio::buffer(buf), ec);

  if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) {
    return;
  }
  if (ec) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Serial read error: %s", ec.message().c_str());
    return;
  }
  if (n == 0) {
    return;
  }

  rx_buffer_.insert(rx_buffer_.end(), buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(n));
  parse_frames();
}

int16_t HWT901BIMUNode::to_int16(uint8_t low, uint8_t high)
{
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8U) | static_cast<uint16_t>(low));
}

bool HWT901BIMUNode::valid_checksum(const std::array<uint8_t, FRAME_SIZE> & frame)
{
  uint16_t sum = 0;
  for (size_t i = 0; i < FRAME_SIZE - 1; ++i) {
    sum += frame[i];
  }
  return static_cast<uint8_t>(sum & 0xFFU) == frame[FRAME_SIZE - 1];
}

void HWT901BIMUNode::parse_frames()
{
  while (rx_buffer_.size() >= FRAME_SIZE) {
    auto header_it = std::find(rx_buffer_.begin(), rx_buffer_.end(), IMU_HEADER);
    if (header_it == rx_buffer_.end()) {
      rx_buffer_.clear();
      return;
    }

    if (header_it != rx_buffer_.begin()) {
      rx_buffer_.erase(rx_buffer_.begin(), header_it);
    }
    if (rx_buffer_.size() < FRAME_SIZE) {
      return;
    }

    std::array<uint8_t, FRAME_SIZE> frame{};
    std::copy_n(rx_buffer_.begin(), FRAME_SIZE, frame.begin());

    if (!valid_checksum(frame)) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    decode_frame(frame);
    rx_buffer_.erase(
      rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(FRAME_SIZE));
  }
}

void HWT901BIMUNode::stamp_headers()
{
  const auto now = this->get_clock()->now();
  imu_msg_.header.stamp = now;
  imu_msg_.header.frame_id = frame_id_;
  mag_msg_.header.stamp = now;
  mag_msg_.header.frame_id = frame_id_;
}

void HWT901BIMUNode::decode_frame(const std::array<uint8_t, FRAME_SIZE> & frame)
{
  stamp_headers();

  switch (frame[1]) {
    case ACCELERATION: {
      imu_msg_.linear_acceleration.x =
        static_cast<double>(to_int16(frame[2], frame[3])) / 32768.0 * 16.0 * G;
      imu_msg_.linear_acceleration.y =
        static_cast<double>(to_int16(frame[4], frame[5])) / 32768.0 * 16.0 * G;
      imu_msg_.linear_acceleration.z =
        static_cast<double>(to_int16(frame[6], frame[7])) / 32768.0 * 16.0 * G;
      imu_received_ = true;
      imu_publisher_->publish(imu_msg_);
      break;
    }
    case ANGULAR_VELOCITY: {
      imu_msg_.angular_velocity.x =
        static_cast<double>(to_int16(frame[2], frame[3])) / 32768.0 * 2000.0 * DEG_TO_RAD;
      imu_msg_.angular_velocity.y =
        static_cast<double>(to_int16(frame[4], frame[5])) / 32768.0 * 2000.0 * DEG_TO_RAD;
      imu_msg_.angular_velocity.z =
        static_cast<double>(to_int16(frame[6], frame[7])) / 32768.0 * 2000.0 * DEG_TO_RAD;
      imu_received_ = true;
      imu_publisher_->publish(imu_msg_);
      break;
    }
    case ANGLE: {
      const auto roll_deg = static_cast<double>(to_int16(frame[2], frame[3])) / 32768.0 * 180.0;
      const auto pitch_deg = static_cast<double>(to_int16(frame[4], frame[5])) / 32768.0 * 180.0;
      const auto yaw_deg = static_cast<double>(to_int16(frame[6], frame[7])) / 32768.0 * 180.0;
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Angle frame [deg] roll=%.2f pitch=%.2f yaw=%.2f", roll_deg, pitch_deg, yaw_deg);
      break;
    }
    case MAGNETIC_FIELD: {
      mag_msg_.magnetic_field.x = static_cast<double>(to_int16(frame[2], frame[3])) * GAUSS_TO_TESLA;
      mag_msg_.magnetic_field.y = static_cast<double>(to_int16(frame[4], frame[5])) * GAUSS_TO_TESLA;
      mag_msg_.magnetic_field.z = static_cast<double>(to_int16(frame[6], frame[7])) * GAUSS_TO_TESLA;
      mag_received_ = true;
      mag_publisher_->publish(mag_msg_);
      break;
    }
    case QUATERNION_CMD: {
      imu_msg_.orientation.w = static_cast<double>(to_int16(frame[2], frame[3])) / 32768.0;
      imu_msg_.orientation.x = static_cast<double>(to_int16(frame[4], frame[5])) / 32768.0;
      imu_msg_.orientation.y = static_cast<double>(to_int16(frame[6], frame[7])) / 32768.0;
      imu_msg_.orientation.z = static_cast<double>(to_int16(frame[8], frame[9])) / 32768.0;
      imu_received_ = true;
      imu_publisher_->publish(imu_msg_);
      break;
    }
    default:
      break;
  }

  if (imu_received_ && mag_received_) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000, "Receiving IMU feedback from %s",
      port_name_.c_str());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HWT901BIMUNode>());
  rclcpp::shutdown();
  return 0;
}






