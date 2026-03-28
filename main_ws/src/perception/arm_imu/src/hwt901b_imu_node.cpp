#include "hwt901b_imu/hwt901b_imu_node.hpp"
#include <algorithm>
#include <functional>
#include <memory>
#include <chrono>

/**
 * HWT901B AHRS IMU センサノード
 * 加速度、角速度、磁気、クォータニオンを取得し、ROS 2トピックとしてpublish
 */
HWT901BIMUNode::HWT901BIMUNode()
: Node("hwt901b_imu_node"), serial_(io_context_)
{
  // parameter
  port_name_ = this->declare_parameter<std::string>("port", "/dev/hwt901b");
  baud_rate_ = this->declare_parameter<int>("baud_rate", 9600);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "hwt901b_link");
  poll_interval_ms_ = this->declare_parameter<int>("poll_interval_ms", 5);

  // initialize publisher
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/hwt901b/imu", 10);
  mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/hwt901b/magnetic_field", 10);

  init_messages();
  open_serial();

  // setting timer
  poll_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(poll_interval_ms_),
    std::bind(&HWT901BIMUNode::poll_serial, this));

  RCLCPP_INFO(this->get_logger(), "HWT901B IMU node started: port=%s baud=%d", port_name_.c_str(), baud_rate_);
}

void HWT901BIMUNode::init_messages()
{
  // 共分散を不明として-1.0に設定（ROS 2規格）
  imu_msg_.orientation_covariance[0] = -1.0;
  imu_msg_.angular_velocity_covariance[0] = -1.0;
  imu_msg_.linear_acceleration_covariance[0] = -1.0;

  // 磁気センサの共分散も不明として-1.0に設定
  mag_msg_.magnetic_field_covariance[0] = -1.0;
}

void HWT901BIMUNode::open_serial()
{
  boost::system::error_code ec;
  serial_.open(port_name_, ec);
  if (ec) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open %s: %s", port_name_.c_str(), ec.message().c_str());
    return;
  }

  // 通信設定: データシートに基づき、パリティなし、ストップビット1、フロー制御なしで固定します。 
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  serial_.set_option(boost::asio::serial_port_base::character_size(8));
  serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none)); // parityなし
  serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one)); // stop bitは1
  serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none)); // flow制御なし
  serial_.non_blocking(true);
}

void HWT901BIMUNode::poll_serial()
{
  if (!serial_.is_open()) return;

  std::array<uint8_t, 256> buf{};
  boost::system::error_code ec;
  const auto n = serial_.read_some(boost::asio::buffer(buf), ec);

  if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) return;
  if (ec) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Serial read error: %s", ec.message().c_str());
    return;
  }
  if (n == 0) return;

  // 受信データをバッファに追加し、フレーム解析へ
  rx_buffer_.insert(rx_buffer_.end(), buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(n));
  parse_frames();
}

// これを結合し、負の数を正しく扱えるよう int16_t (signed short) にキャストします
int16_t HWT901BIMUNode::to_int16(uint8_t low, uint8_t high)
{
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8U) | static_cast<uint16_t>(low));
}

// SUM check
bool HWT901BIMUNode::valid_checksum(const std::array<uint8_t, FRAME_SIZE> & frame)
{
  uint16_t sum = 0;
  for (size_t i = 0; i < FRAME_SIZE - 1; ++i) {
    sum += frame[i];
  }
  return static_cast<uint8_t>(sum & 0xFFU) == frame[FRAME_SIZE - 1];
}

/*
  このコードだと受信が詰まった際に計算量が増えるので std::vectorから std::dequeに変更したほうがいい
  しかし多分動きはするためこれで行って後々修正しようかなと
  これでも200Hzくらいなら支障ないと考えた
*/
void HWT901BIMUNode::parse_frames()
{
  while (rx_buffer_.size() >= FRAME_SIZE) {
    // ヘッダー(0x55)を探索
    auto header_it = std::find(rx_buffer_.begin(), rx_buffer_.end(), IMU_HEADER);
    
    if (header_it == rx_buffer_.end()) {
      rx_buffer_.clear(); // ヘッダーがない場合は全破棄
      return;
    }

    if (header_it != rx_buffer_.begin()) {
      rx_buffer_.erase(rx_buffer_.begin(), header_it); // 不正なデータを先頭から削除
    }
    
    if (rx_buffer_.size() < FRAME_SIZE) return;

    std::array<uint8_t, FRAME_SIZE> frame{};
    std::copy_n(rx_buffer_.begin(), FRAME_SIZE, frame.begin());

    if (!valid_checksum(frame)) {
      rx_buffer_.erase(rx_buffer_.begin()); // チェックサムエラーなら1バイトずらして再試行
      continue;
    }

    decode_frame(frame);
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(FRAME_SIZE));
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

/**
 * フレームのデコードとパブリッシュ
 * センサーから別々に送られてくる各データを一時保持し、
 * 通信帯域の節約とデータの同時性を担保するため、特定のパケット受信をトリガーにまとめて配信します。
 */
void HWT901BIMUNode::decode_frame(const std::array<uint8_t, FRAME_SIZE> & frame)
{
  switch (frame[1]) {
    case ACCELERATION: { // 0x51: 加速度
      // 計算式: (結合データ / 32768) * 16g
      imu_msg_.linear_acceleration.x = static_cast<double>(to_int16(frame[2], frame[3])) / 32768.0 * 16.0 * G;
      imu_msg_.linear_acceleration.y = static_cast<double>(to_int16(frame[4], frame[5])) / 32768.0 * 16.0 * G;
      imu_msg_.linear_acceleration.z = static_cast<double>(to_int16(frame[6], frame[7])) / 32768.0 * 16.0 * G;
      break;
    }
    case ANGULAR_VELOCITY: { // 0x52: 角速度
      // 計算式: (結合データ / 32768) * 2000 degree/s
      // ROS 2規格に合わせるため rad/s へ変換
      imu_msg_.angular_velocity.x = static_cast<double>(to_int16(frame[2], frame[3])) / 32768.0 * 2000.0 * DEG_TO_RAD;
      imu_msg_.angular_velocity.y = static_cast<double>(to_int16(frame[4], frame[5])) / 32768.0 * 2000.0 * DEG_TO_RAD;
      imu_msg_.angular_velocity.z = static_cast<double>(to_int16(frame[6], frame[7])) / 32768.0 * 2000.0 * DEG_TO_RAD;
      break;
    }
    case MAGNETIC_FIELD: { // 0x54: 磁気
      // 磁気データは計測周期が異なる場合があるため、受信の度に配信
      mag_msg_.magnetic_field.x = static_cast<double>(to_int16(frame[2], frame[3])) * GAUSS_TO_TESLA;
      mag_msg_.magnetic_field.y = static_cast<double>(to_int16(frame[4], frame[5])) * GAUSS_TO_TESLA;
      mag_msg_.magnetic_field.z = static_cast<double>(to_int16(frame[6], frame[7])) * GAUSS_TO_TESLA;
      
      mag_msg_.header.stamp = this->get_clock()->now();
      mag_msg_.header.frame_id = frame_id_;
      mag_publisher_->publish(mag_msg_);
      mag_received_ = true;
      break;
    }
    case QUATERNION_CMD: { // 0x59: Quaternion
      // 計算式: 結合データ / 32768
      imu_msg_.orientation.w = static_cast<double>(to_int16(frame[2], frame[3])) / 32768.0;
      imu_msg_.orientation.x = static_cast<double>(to_int16(frame[4], frame[5])) / 32768.0;
      imu_msg_.orientation.y = static_cast<double>(to_int16(frame[6], frame[7])) / 32768.0;
      imu_msg_.orientation.z = static_cast<double>(to_int16(frame[8], frame[9])) / 32768.0;
      
      // パブリッシュ
      // 0x59は出力シーケンスの最後の方に来るため、これをトリガーにして
      // 「加速度・角速度・四元数」が揃った最新のIMUメッセージを一度にpublish
      stamp_headers(); 
      imu_publisher_->publish(imu_msg_);
      imu_received_ = true;
      break;
    }
    default:
      break;
  }

  // デバッグ用: 両方のメッセージが受信できていることを一定間隔で表示
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