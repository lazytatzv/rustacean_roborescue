#include "hwt901b_imu/hwt901b_imu_node.hpp"
#include <algorithm>
#include <memory>
#include <thread>

namespace
{
constexpr char kMagneticTopic[] = "/hwt901b/magnetic_field";
constexpr int kPublisherQueueDepth = 10;
constexpr double kMagStddevTesla = 0.15e-6;

void set_magnetic_covariance(sensor_msgs::msg::MagneticField & msg)
{
  const double var = kMagStddevTesla * kMagStddevTesla;
  msg.magnetic_field_covariance[0] = var;
  msg.magnetic_field_covariance[4] = var;
  msg.magnetic_field_covariance[8] = var;
}
}  // namespace

/**
 * HWT901B AHRS IMU センサノード
 * 磁気センサー値のみ取得し、ROS 2トピックとしてpublish
 */

HWT901BIMUNode::HWT901BIMUNode()
: Node("hwt901b_imu_node"), serial_(io_context_)
{
  RCLCPP_INFO(this->get_logger(), "[startup] HWT901B magnetic node initialization begin");

  load_parameters();

  if (baud_rate_ <= 9600) {
    RCLCPP_WARN(this->get_logger(),
      "baud_rate=%d. High-rate outputでは115200推奨です。", baud_rate_);
  }

  // 磁気データのみ publish する。
  mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
    kMagneticTopic, kPublisherQueueDepth);
  RCLCPP_INFO(
    this->get_logger(),
    "[startup] publisher ready: topic=%s depth=%d",
    kMagneticTopic, kPublisherQueueDepth);

  // シリアル初期化に成功したときだけ非同期受信を開始する。
  if (configure_serial()) {
    start_async_read();
    start_io_thread();
  } else {
    RCLCPP_ERROR(this->get_logger(), "[startup] serial initialization failed; async receive disabled");
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[startup] HWT901B magnetic node started: port=%s baud=%d frame_id=%s scale=%.6f",
    port_name_.c_str(), baud_rate_, frame_id_.c_str(), mag_scale_);
  RCLCPP_INFO(this->get_logger(), "[startup] initialization end");
}

HWT901BIMUNode::~HWT901BIMUNode()
{
  // readコールバックより先にI/Oループを停止して安全に終了する。
  stop_io_thread();
}

void HWT901BIMUNode::load_parameters()
{
  // 宣言と取得を分離し、外部上書き後の最終値を読む。
  this->declare_parameter<std::string>("port", "/dev/hwt901b");
  this->declare_parameter<int>("baud_rate", 9600);
  this->declare_parameter<std::string>("frame_id", "hwt901b_link");
  this->declare_parameter<double>("mag_scale", 1.0);

  this->get_parameter("port", port_name_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("mag_scale", mag_scale_);

  RCLCPP_INFO(
    this->get_logger(),
    "[startup] parameters: port=%s baud_rate=%d frame_id=%s mag_scale=%.6f",
    port_name_.c_str(), baud_rate_, frame_id_.c_str(), mag_scale_);
}

bool HWT901BIMUNode::configure_serial()
{
  RCLCPP_INFO(this->get_logger(), "[startup] opening serial: %s", port_name_.c_str());

  boost::system::error_code ec;
  serial_.open(port_name_, ec);
  if (ec) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open %s: %s", port_name_.c_str(), ec.message().c_str());
    return false;
  }

  // HWT901Bデータシート準拠の通信設定。
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  serial_.set_option(boost::asio::serial_port_base::character_size(8));
  serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  RCLCPP_INFO(
    this->get_logger(),
    "[startup] serial configured: baud=%d data_bits=8 parity=none stop_bits=1 flow_control=none",
    baud_rate_);
  return true;
}

void HWT901BIMUNode::start_io_thread()
{
  RCLCPP_INFO(this->get_logger(), "[startup] starting io_context thread");
  io_thread_ = std::thread([this]() { io_context_.run(); });
}

void HWT901BIMUNode::stop_io_thread()
{
  RCLCPP_INFO(this->get_logger(), "[shutdown] stopping io_context and serial");
  io_context_.stop();
  if (serial_.is_open()) {
    boost::system::error_code ec;
    serial_.cancel(ec);
    serial_.close(ec);
  }
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  RCLCPP_INFO(this->get_logger(), "[shutdown] io_context thread stopped");
}

void HWT901BIMUNode::start_async_read()
{
  if (!serial_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "[startup] cannot arm async read: serial is not open");
    return;
  }

  RCLCPP_INFO_ONCE(this->get_logger(), "[startup] async read armed");

  serial_.async_read_some(
    boost::asio::buffer(read_buf_),
    [this](const boost::system::error_code & ec, std::size_t n) { on_read(ec, n); });
}

void HWT901BIMUNode::on_read(const boost::system::error_code & ec, std::size_t n)
{
  if (ec == boost::asio::error::operation_aborted) return;

  if (ec) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Serial read error: %s", ec.message().c_str());
    start_async_read();
    return;
  }

  if (n > 0) {
    // 受信データをバッファに追加
    rx_buffer_.insert(
      rx_buffer_.end(),
      read_buf_.begin(),
      read_buf_.begin() + static_cast<std::ptrdiff_t>(n));

    // 解析実行
    parse_frames();
  }

  // 次の読み取りを予約
  start_async_read();
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
    // ヘッダ同期が取れるまで1byteずつ捨てる。
    if (rx_buffer_.front() != IMU_HEADER) {
      rx_buffer_.pop_front();
      continue;
    }

    std::array<uint8_t, FRAME_SIZE> frame{};
    std::copy_n(rx_buffer_.begin(), FRAME_SIZE, frame.begin());

    if (!valid_checksum(frame)) {
      // ヘッダ候補が偽陽性のときは1byte進めて再同期する。
      rx_buffer_.pop_front();
      continue;
    }

    decode_frame(frame);
    consume_frame();
  }
}

void HWT901BIMUNode::consume_frame()
{
  for (size_t i = 0; i < FRAME_SIZE; ++i) {
    rx_buffer_.pop_front();
  }
}

void HWT901BIMUNode::decode_frame(const std::array<uint8_t, FRAME_SIZE> & frame)
{
  if (frame[1] != MAGNETIC_FIELD) return;

  // コールバックローカルでメッセージを構築する。
  sensor_msgs::msg::MagneticField msg;
  set_magnetic_covariance(msg);

  // データ変換
  msg.magnetic_field.x = static_cast<double>(to_int16(frame[2], frame[3])) * GAUSS_TO_TESLA * mag_scale_;
  msg.magnetic_field.y = static_cast<double>(to_int16(frame[4], frame[5])) * GAUSS_TO_TESLA * mag_scale_;
  msg.magnetic_field.z = static_cast<double>(to_int16(frame[6], frame[7])) * GAUSS_TO_TESLA * mag_scale_;

  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = frame_id_;

  mag_publisher_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HWT901BIMUNode>());
  rclcpp::shutdown();
  return 0;
}