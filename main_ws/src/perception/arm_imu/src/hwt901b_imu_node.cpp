#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <functional>

// ros2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>

class HWT901BIMUNode : public rclcpp::Node
{
public:
  HWT901BIMUNode() : Node("hwt901b_imu_node")
  {
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/hwt901b/imu", 10);
    mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/hwt901b/magnetic_field", 10);
