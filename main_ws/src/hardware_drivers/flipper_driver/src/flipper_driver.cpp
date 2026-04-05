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
    declare_parameter("port_name", "/dev/dynamixel");
    declare_parameter("baud_rate", 115200);
    declare_parameter("dynamixel_ids", std::vector<int>({1, 2, 3, 4}));
    declare_parameter("watchdog_timeout_ms", 500);
    declare_parameter("velocity_limit", 1023);
    declare_parameter("init_retry_sec", 3.0);

    initParams();

    // ハードウェア初期化をリトライ付きで行う。
    // 電源投入直後など Dynamixel がまだ起動していない場合でもノードを落とさない。
    retry_timer_ = create_wall_timer(
        std::chrono::duration<double>(get_parameter("init_retry_sec").as_double()),
        std::bind(&FlipperDriver::tryInitDynamixel, this));
    tryInitDynamixel();  // 起動直後に即 1 回試みる
  }

  ~FlipperDriver() override
  {
    if (initialized_)
    {
      RCLCPP_INFO(get_logger(), "Shutting down — stopping motors");
      stopMotors();
    }
  }

 private:
  DynamixelWorkbench dxl_wb_;
  std::string port_name_;
  int baud_rate_;
  std::vector<long int> dynamixel_ids_;
  int watchdog_timeout_ms_;
  int velocity_limit_;
  bool initialized_ = false;

  rclcpp::Subscription<custom_interfaces::msg::FlipperVelocity>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::Time last_cmd_time_;
  bool estop_active_ = false;

  void initParams()
  {
    port_name_ = get_parameter("port_name").as_string();
    baud_rate_ = get_parameter("baud_rate").as_int();
    dynamixel_ids_ = get_parameter("dynamixel_ids").as_integer_array();
    watchdog_timeout_ms_ = get_parameter("watchdog_timeout_ms").as_int();
    velocity_limit_ = get_parameter("velocity_limit").as_int();
  }

  // 初期化成功後に一度だけ呼び出す
  void setupSubscriptions()
  {
    last_cmd_time_ = now();

    subscription_ = create_subscription<custom_interfaces::msg::FlipperVelocity>(
        "/flipper_driver", 10,
        std::bind(&FlipperDriver::driver_callback, this, std::placeholders::_1));

    watchdog_timer_ = create_wall_timer(std::chrono::milliseconds(watchdog_timeout_ms_),
                                        std::bind(&FlipperDriver::watchdog_callback, this));

    auto estop_qos = rclcpp::QoS(1).transient_local().reliable();
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop", estop_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
          const bool was_estop = estop_active_;
          estop_active_ = msg->data;
          if (estop_active_ && !was_estop)
          {
            stopMotors();
            RCLCPP_FATAL(get_logger(), "EMERGENCY STOP activated — motors halted");
          }
          else if (!estop_active_ && was_estop)
          {
            RCLCPP_WARN(get_logger(), "EMERGENCY STOP cleared — command acceptance resumed");
          }
        });
  }

  void tryInitDynamixel()
  {
    if (initialized_) return;

    if (!dxl_wb_.init(port_name_.c_str(), baud_rate_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Dynamixel init failed (port=%s). Retrying...", port_name_.c_str());
      return;
    }

    for (const auto &id : dynamixel_ids_)
    {
      if (!dxl_wb_.ping(id))
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "Dynamixel ID %ld not responding. Retrying...", id);
        return;
      }
      if (!dxl_wb_.wheelMode(id, 0))
      {
        RCLCPP_WARN(get_logger(), "Failed to set wheel mode for ID %ld. Retrying...", id);
        return;
      }
      //if (!dxl_wb_.itemWrite(id, "Velocity_Limit", velocity_limit_))
      //{
      //  RCLCPP_WARN(get_logger(), "Failed to set velocity limit for ID %ld. Retrying...", id);
      //  return;
     // }
    }

    initialized_ = true;
    retry_timer_->cancel();
    setupSubscriptions();
    RCLCPP_INFO(get_logger(), "Dynamixel initialized successfully");
  }

  void stopMotors()
  {
    for (const auto &id : dynamixel_ids_)
    {
      if (!dxl_wb_.goalVelocity(id, 0))
        RCLCPP_ERROR(get_logger(), "Failed to stop Dynamixel motor ID %ld", id);
    }
  }

  void driver_callback(const custom_interfaces::msg::FlipperVelocity &msg)
  {
    if (estop_active_) return;
    last_cmd_time_ = now();

    if (msg.flipper_vel.size() < dynamixel_ids_.size())
    {
      RCLCPP_ERROR(get_logger(), "flipper_vel size (%zu) < dynamixel_ids size (%zu)",
                   msg.flipper_vel.size(), dynamixel_ids_.size());
      return;
    }

    for (size_t i = 0; i < dynamixel_ids_.size(); ++i)
    {
      if (!dxl_wb_.goalVelocity(dynamixel_ids_[i], msg.flipper_vel[i]))
        RCLCPP_ERROR(get_logger(), "Failed to set goal velocity for ID %ld", dynamixel_ids_[i]);
    }
  }

  void watchdog_callback()
  {
    const double elapsed = (now() - last_cmd_time_).seconds() * 1000.0;
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
