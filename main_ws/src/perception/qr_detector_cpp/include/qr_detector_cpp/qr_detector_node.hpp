#ifndef QR_DETECTOR_CPP__QR_DETECTOR_NODE_HPP_
#define QR_DETECTOR_CPP__QR_DETECTOR_NODE_HPP_

#include <zbar.h>

#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/wechat_qrcode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

namespace qr_detector_cpp
{

class QrDetectorNode : public rclcpp::Node
{
 public:
  explicit QrDetectorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  // Parameters
  std::string model_dir_;
  bool publish_compressed_;
  int jpeg_quality_;
  int detection_interval_;

  // WeChatQRCode detector (高精度; Protobuf付きOpenCV必須)
  std::unique_ptr<cv::wechat_qrcode::WeChatQRCode> detector_;
  // zbar スキャナー (WeChatQRCode使用不可時のフォールバック)
  zbar::ImageScanner zbar_scanner_;

  // Publishers and Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;

  int frame_count_ = 0;
};

}  // namespace qr_detector_cpp

#endif  // QR_DETECTOR_CPP__QR_DETECTOR_NODE_HPP_
