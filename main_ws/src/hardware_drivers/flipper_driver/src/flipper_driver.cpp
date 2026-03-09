#include <chrono>
#include <custom_interfaces/msg/flipper_velocity.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

class FlipperDriver : public rclcpp::Node
{
 public:
  FlipperDriver() : Node("flipper_driver")
  {
    declare_parameter("port_name", "/dev/dynamixel");
    declare_parameter("baud_rate", 115200);
    declare_parameter("dynamixel_ids", std::vector<int>({1, 2, 3, 4}));
    declare_parameter("watchdog_timeout_ms", 500);

    initParams();

    if (!initDynamixel())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize Dynamixel Workbench");
      rclcpp::shutdown();
      return;
    }

    subscription_ = create_subscription<custom_interfaces::msg::FlipperVelocity>(
        "/flipper_driver", 10,
        std::bind(&FlipperDriver::driver_callback, this, std::placeholders::_1));

    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(watchdog_timeout_ms_),
        std::bind(&FlipperDriver::watchdog_callback, this));
  }

 private:
  DynamixelWorkbench    dxl_wb_;
  std::string           port_name_;
  int                   baud_rate_;
  std::vector<long int> dynamixel_ids_;
  int                   watchdog_timeout_ms_;

  rclcpp::Subscription<custom_interfaces::msg::FlipperVelocity>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_cmd_time_;

  void initParams()
  {
    port_name_           = get_parameter("port_name").as_string();
    baud_rate_           = get_parameter("baud_rate").as_int();
    dynamixel_ids_       = get_parameter("dynamixel_ids").as_integer_array();
    watchdog_timeout_ms_ = get_parameter("watchdog_timeout_ms").as_int();
    last_cmd_time_       = now();
  }

  bool initDynamixel()
  {
    if (!dxl_wb_.init(port_name_.c_str(), baud_rate_))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize Dynamixel Workbench");
      return false;
    }

    for (const auto &id : dynamixel_ids_)
    {
      if (!dxl_wb_.ping(id))
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to ping Dynamixel motor with ID %ld", id);
        return false;
      }

      dxl_wb_.wheelMode(id, 0);
      dxl_wb_.itemWrite(id, "Velocity_Limit", 1023);
    }

    RCLCPP_INFO(this->get_logger(), "Dynamixel Workbench initialized successfully");
    return true;
  }

  void stopMotors()
  {
    for (const auto &id : dynamixel_ids_)
      dxl_wb_.goalVelocity(id, 0);
  }

  void driver_callback(const custom_interfaces::msg::FlipperVelocity &msg)
  {
    last_cmd_time_ = now();

    if (msg.flipper_vel.size() < dynamixel_ids_.size())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "flipper_vel size (%zu) < dynamixel_ids size (%zu)",
                   msg.flipper_vel.size(), dynamixel_ids_.size());
      return;
    }

    for (size_t i = 0; i < dynamixel_ids_.size(); ++i)
    {
      if (!dxl_wb_.goalVelocity(dynamixel_ids_[i], msg.flipper_vel[i]))
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to set goal velocity for Dynamixel motor with ID %ld",
                     dynamixel_ids_[i]);
      }
    }
  }

  void watchdog_callback()
  {
    auto elapsed = (now() - last_cmd_time_).seconds() * 1000.0;
    if (elapsed > watchdog_timeout_ms_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                            "Watchdog: no command for %.0f ms, stopping motors", elapsed);
      stopMotors();
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlipperDriver>());
  rclcpp::shutdown();
  return 0;
}
