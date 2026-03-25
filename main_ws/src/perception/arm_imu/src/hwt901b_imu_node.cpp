//standard libraries
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <functional>

// ros2
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/quaternion.hpp>

class HWT901BIMUNode : public rclcpp::Node
{
  public:
    HWT901BIMUNode() : Node("hwt901b_imu_node")
    {
      // Initialize ROS publishers
      imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/hwt901b/imu", 10);
      mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/hwt901b/magnetic_field", 10);

      RCLCPP_INFO(this->get_logger(), "HWT901B IMU node started");
    }


  private:
    // ROS publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;

    // HWT901B command bytes
    static constexpr uint8_t IMU_HEADER = 0x55;
    static constexpr uint8_t ACCELERATION = 0x51;
    static constexpr uint8_t ANGULAR_VELOCITY = 0x52;
    static constexpr uint8_t ANGLE = 0x53;
    static constexpr uint8_t MAGNETIC_FIELD = 0x54;
    static constexpr uint8_t QUATERNION = 0x59;
    
};






