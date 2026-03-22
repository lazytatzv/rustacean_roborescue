#include <memory>

#include "qr_detector_cpp/qr_detector_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // Default NodeOptions (e.g. for parameters from CLI)
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<qr_detector_cpp::QrDetectorNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
