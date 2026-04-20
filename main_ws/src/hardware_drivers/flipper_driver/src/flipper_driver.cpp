#include <chrono>
#include <custom_interfaces/msg/flipper_velocity.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

using namespace std::chrono_literals;

class FlipperDriver : public rclcpp::Node
{
 public:
  FlipperDriver() : Node("flipper_driver")
  {
    declare_parameter("port_name", "/dev/dynamixel_flipper");
    declare_parameter("baud_rate", 115200);
    declare_parameter("dynamixel_ids", std::vector<int>({1, 3, 4, 2}));
    declare_parameter("velocity_limit", 1023);
    declare_parameter("init_retry_sec", 3.0);
    declare_parameter("watchdog_timeout_ms", 500);
    declare_parameter("servo_inverted", std::vector<bool>({true, true, true, true}));

    initParams();

    retry_timer_ = create_wall_timer(
        std::chrono::duration<double>(get_parameter("init_retry_sec").as_double()),
        std::bind(&FlipperDriver::tryInitDynamixel, this));
    
    tryInitDynamixel();
  }

  ~FlipperDriver() override
  {
    if (initialized_) stopMotors();
  }

 private:
  DynamixelWorkbench dxl_wb_;
  std::string port_name_;
  int baud_rate_;
  std::vector<long int> dynamixel_ids_;
  int velocity_limit_;
  int watchdog_timeout_ms_;
  std::vector<bool> servo_inverted_;
  bool initialized_ = false;
  bool estop_active_ = false;
  rclcpp::Time last_cmd_time_;

  rclcpp::Subscription<custom_interfaces::msg::FlipperVelocity>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  void initParams()
  {
    port_name_ = get_parameter("port_name").as_string();
    baud_rate_ = get_parameter("baud_rate").as_int();
    dynamixel_ids_ = get_parameter("dynamixel_ids").as_integer_array();
    velocity_limit_ = get_parameter("velocity_limit").as_int();
    watchdog_timeout_ms_ = get_parameter("watchdog_timeout_ms").as_int();
    servo_inverted_ = get_parameter("servo_inverted").as_bool_array();
    if (servo_inverted_.size() < dynamixel_ids_.size()) {
      servo_inverted_.resize(dynamixel_ids_.size(), true);
    }
  }

  void tryInitDynamixel()
  {
    if (initialized_) return;

    if (!dxl_wb_.init(port_name_.c_str(), baud_rate_)) return;

    for (const auto &id : dynamixel_ids_)
    {
      if (!dxl_wb_.ping(id)) return;
      
      // トルクを一旦切って設定を反映
      dxl_wb_.torqueOff(id);
      dxl_wb_.wheelMode(id, 0);
      // トルクを明示的にON
      dxl_wb_.torqueOn(id);
      dxl_wb_.goalVelocity(id, 0);
    }

    initialized_ = true;
    last_cmd_time_ = now();
    retry_timer_->cancel();
    setupSubscriptions();
    
    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(watchdog_timeout_ms_),
        [this]() {
          if (!initialized_ || estop_active_) return;
          if ((now() - last_cmd_time_).seconds() * 1000.0 > watchdog_timeout_ms_) {
            stopMotors();
          }
        });

    RCLCPP_INFO(get_logger(), "✅ Flipper Dynamixels Ready (Torque ON / 115200bps)");
  }

  void setupSubscriptions()
  {
    subscription_ = create_subscription<custom_interfaces::msg::FlipperVelocity>(
        "/flipper_driver", 10,
        [this](const custom_interfaces::msg::FlipperVelocity &msg)
        {
          if (estop_active_ || !initialized_) return;
          if (msg.flipper_vel.empty()) return;

          last_cmd_time_ = now();
          for (size_t i = 0; i < dynamixel_ids_.size() && i < msg.flipper_vel.size(); ++i)
          {
            const int32_t vel = servo_inverted_[i] ? -msg.flipper_vel[i] : msg.flipper_vel[i];
            dxl_wb_.goalVelocity(dynamixel_ids_[i], vel);
          }
        });

    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop", rclcpp::QoS(1).transient_local().reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
          const bool was_estop = estop_active_;
          estop_active_ = msg->data;
          if (estop_active_ && !was_estop)
          {
            stopMotors();
            for (const auto &id : dynamixel_ids_) dxl_wb_.torqueOff(id);
            RCLCPP_FATAL(get_logger(), "⛔ EMERGENCY STOP - Torque Disabled");
          }
          else if (!estop_active_ && was_estop)
          {
            for (const auto &id : dynamixel_ids_) dxl_wb_.torqueOn(id);
            RCLCPP_INFO(get_logger(), "✅ EMERGENCY STOP Cleared - Torque Enabled");
          }
        });
  }

  void stopMotors()
  {
    for (const auto &id : dynamixel_ids_) dxl_wb_.goalVelocity(id, 0);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlipperDriver>());
  rclcpp::shutdown();
  return 0;
}
