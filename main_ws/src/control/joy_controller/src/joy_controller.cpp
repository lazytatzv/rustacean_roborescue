#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "custom_interfaces/msg/crawler_velocity.hpp"
#include "custom_interfaces/msg/flipper_velocity.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JoyController : public rclcpp::Node
{
 public:
   JoyController() : Node("joy_controller"), mode_(Mode::STOP)
  {
    this->declare_parameter("max_speed", 0.7);
    this->declare_parameter("deadzone", 0.1);
    this->declare_parameter("flipper_speed", 1000);
    this->declare_parameter("arm_linear_scale", 0.6); // Increased to 0.6
    this->declare_parameter("arm_angular_scale", 0.5);
    this->declare_parameter("joint_speed_scale", 0.5);

    max_speed_     = static_cast<float>(this->get_parameter("max_speed").as_double());
    deadzone_      = static_cast<float>(this->get_parameter("deadzone").as_double());
    flipper_speed_ = static_cast<int>(this->get_parameter("flipper_speed").as_int());
    arm_lin_scale_ = static_cast<float>(this->get_parameter("arm_linear_scale").as_double());
    arm_ang_scale_ = static_cast<float>(this->get_parameter("arm_angular_scale").as_double());
    joint_scale_   = static_cast<float>(this->get_parameter("joint_speed_scale").as_double());

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyController::joy_callback, this, std::placeholders::_1));

    crawler_publisher_ = this->create_publisher<custom_interfaces::msg::CrawlerVelocity>("/crawler_driver", 10);
    flipper_publisher_ = this->create_publisher<custom_interfaces::msg::FlipperVelocity>("/flipper_driver", 10);
    arm_publisher_     = this->create_publisher<geometry_msgs::msg::Twist>("/arm_cmd_vel", 10);
    joint_publisher_   = this->create_publisher<sensor_msgs::msg::JointState>("/arm_joint_cmd_vel", 10);
    estop_publisher_   = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", rclcpp::QoS(1).transient_local().reliable());

    RCLCPP_INFO(this->get_logger(), "joy_controller started (IK + Joint support)");
  }

 private:
  enum class Mode { STOP, DRIVE, ARM, JOINT };

  float max_speed_, deadzone_, arm_lin_scale_, arm_ang_scale_, joint_scale_;
  int flipper_speed_;

  static constexpr size_t BUTTON_SHARE = 8, BUTTON_OPTIONS = 9, BUTTON_PS = 10;
  static constexpr size_t AXIS_LEFT_X = 0, AXIS_LEFT_Y = 1, AXIS_RIGHT_X = 3, AXIS_RIGHT_Y = 4;
  static constexpr size_t AXIS_L2 = 2, AXIS_R2 = 5, AXIS_DPAD_Y = 7;
  static constexpr size_t BUTTON_CROSS = 0, BUTTON_SQUARE = 2, BUTTON_L1 = 4, BUTTON_R1 = 5;

  Mode mode_;
  bool estop_latched_ = false;
  int prev_ps_ = 0, prev_options_ = 0, prev_share_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<custom_interfaces::msg::CrawlerVelocity>::SharedPtr crawler_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::FlipperVelocity>::SharedPtr flipper_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher_;

  struct ArmInput
  {
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
  };

  static float apply_deadzone(float val, float threshold) {return (std::abs(val) < threshold) ? 0.0f : val; }
  static int button(const sensor_msgs::msg::Joy &msg, size_t idx) { return (idx < msg.buttons.size()) ? msg.buttons[idx] : 0; }
  static float axis(const sensor_msgs::msg::Joy &msg, size_t idx) { return (idx < msg.axes.size()) ? msg.axes[idx] : 0.0f; }

  ArmInput compute_arm_input(const sensor_msgs::msg::Joy &msg) const
  {
    const float l2 = (1.0f - axis(msg, AXIS_L2)) / 2.0f;
    const float r2 = (1.0f - axis(msg, AXIS_R2)) / 2.0f;
    return ArmInput{
      -apply_deadzone(axis(msg, AXIS_LEFT_Y), deadzone_),
      apply_deadzone(axis(msg, AXIS_LEFT_X), deadzone_),
      apply_deadzone(axis(msg, AXIS_RIGHT_Y), deadzone_),
      (r2 - l2),
      apply_deadzone(axis(msg, AXIS_DPAD_Y), 0.5f),
      -apply_deadzone(axis(msg, AXIS_RIGHT_X), deadzone_)
    };
  }

  void update_mode(const sensor_msgs::msg::Joy &msg)
  {
    const int ps = button(msg, BUTTON_PS);
    const int options = button(msg, BUTTON_OPTIONS);
    const int share = button(msg, BUTTON_SHARE);

    // Detect button press events
    const bool ps_pressed = (ps == 1 && prev_ps_ == 0);
    const bool options_pressed = (options == 1 && prev_options_ == 0);
    const bool share_pressed = (share == 1 && prev_share_ == 0);
    const bool unlock_estop_latched = (options == 1 && share == 1);
    const bool unlock_combo_pressed = unlock_estop_latched && !(prev_options_ == 1 && prev_share_ == 1);

    // PS is a dedicated hard-stop trigger.
    if (ps_pressed) {
      estop_latched_ = true;
      mode_ = Mode::STOP;
      auto estop_msg = std_msgs::msg::Bool();
      estop_msg.data = true;
      estop_publisher_->publish(estop_msg);
      RCLCPP_WARN(this->get_logger(), "⛔ EMERGENCY STOP");
    } else if (estop_latched_) {
      // Clear requires OPTIONS+SHARE combo to avoid accidental release.
      if (unlock_combo_pressed) {
        estop_latched_ = false;
        auto estop_msg = std_msgs::msg::Bool();
        estop_msg.data = false;
        estop_publisher_->publish(estop_msg);
        RCLCPP_INFO(this->get_logger(), "✅ E-STOP CLEARED (OPTIONS+SHARE)");
      }
    } else {
      if (options_pressed) {
        mode_ = Mode::DRIVE;
        RCLCPP_INFO(this->get_logger(), "Mode: DRIVE");
      } else if (share_pressed) {
        if (mode_ == Mode::ARM) {
          mode_ = Mode::JOINT;
          RCLCPP_INFO(this->get_logger(), "Mode: JOINT (Direct Control)");
        } else {
          mode_ = Mode::ARM;
          RCLCPP_INFO(this->get_logger(), "Mode: ARM (IK Control)");
        }
      }
    }

    prev_ps_ = ps;
    prev_options_ = options;
    prev_share_ = share;
  }

  void joy_callback(const sensor_msgs::msg::Joy &msg)
  {
    update_mode(msg);
    publish_crawler(msg);
    publish_arm(msg);
  }

  void publish_crawler(const sensor_msgs::msg::Joy &msg)
  {
    auto crawler_msg = custom_interfaces::msg::CrawlerVelocity();
    auto flipper_msg = custom_interfaces::msg::FlipperVelocity();
    if (mode_ == Mode::DRIVE) {
      float m1 = std::clamp(-apply_deadzone(axis(msg, AXIS_LEFT_Y), deadzone_) * max_speed_, -max_speed_, max_speed_);
      float m2 = std::clamp(apply_deadzone(axis(msg, AXIS_RIGHT_Y), deadzone_) * max_speed_, -max_speed_, max_speed_);
      crawler_msg.m1_vel = m1; crawler_msg.m2_vel = m2;
      int f1 = (axis(msg, AXIS_DPAD_Y) < -0.5f) ? 1 : (axis(msg, AXIS_DPAD_Y) > 0.5f ? -1 : 0);
      int f2 = (button(msg, BUTTON_CROSS) == 1) ? 1 : (button(msg, BUTTON_SQUARE) == 1 ? -1 : 0);
      int f3 = (button(msg, BUTTON_L1) == 1) ? 1 : (axis(msg, AXIS_L2) < -0.9f ? -1 : 0);
      int f4 = (button(msg, BUTTON_R1) == 1) ? 1 : (axis(msg, AXIS_R2) < -0.9f ? -1 : 0);
      flipper_msg.flipper_vel = {static_cast<int32_t>(f1 * flipper_speed_), static_cast<int32_t>(f2 * flipper_speed_), static_cast<int32_t>(f3 * flipper_speed_), static_cast<int32_t>(f4 * flipper_speed_)};
    }
    crawler_publisher_->publish(crawler_msg);
    flipper_publisher_->publish(flipper_msg);
  }

  void publish_arm(const sensor_msgs::msg::Joy &msg)
  {
    const auto input = compute_arm_input(msg);

    if (mode_ == Mode::ARM) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = input.linear_x * arm_lin_scale_;
      twist.linear.y = input.linear_y * arm_lin_scale_;
      twist.linear.z = input.linear_z * arm_lin_scale_;
      twist.angular.x = input.angular_x * arm_ang_scale_;
      twist.angular.y = input.angular_y * arm_ang_scale_;
      twist.angular.z = input.angular_z * arm_ang_scale_;
      arm_publisher_->publish(twist);
    } else if (mode_ == Mode::JOINT) {
      auto j_msg = sensor_msgs::msg::JointState();
      j_msg.name = {"arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6"};
      j_msg.velocity = {
        static_cast<double>(input.linear_x * joint_scale_),
        static_cast<double>(input.linear_y * joint_scale_),
        static_cast<double>(input.linear_z * joint_scale_),
        static_cast<double>(input.angular_z * joint_scale_),
        static_cast<double>(input.angular_x * joint_scale_),
        static_cast<double>(input.angular_y * joint_scale_)
      };
      joint_publisher_->publish(j_msg);
    }
  }
};

int main(int argc, char *argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<JoyController>());
   rclcpp::shutdown();
   return 0;
}
