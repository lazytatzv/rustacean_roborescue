#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "custom_interfaces/msg/crawler_velocity.hpp"
#include "custom_interfaces/msg/flipper_velocity.hpp"
#include "custom_interfaces/msg/gripper_command.hpp"
#include "custom_interfaces/msg/gripper_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

class JoyController : public rclcpp::Node
{
 public:
  JoyController() : Node("joy_controller"), mode_(Mode::STOP)
  {
    // --- Parameters ---
    max_speed_         = declare_parameter("max_speed", 0.7);
    deadzone_          = declare_parameter("deadzone", 0.1);
    flipper_speed_     = declare_parameter("flipper_speed", 1000);
    arm_lin_scale_     = declare_parameter("arm_linear_scale", 0.6);
    arm_ang_scale_     = declare_parameter("arm_angular_scale", 0.5);
    joint_scale_       = declare_parameter("joint_speed_scale", 0.5);
    
    // Gripper Calibration
    gripper_open_pos_  = declare_parameter("gripper_open_position", 1500);
    gripper_close_pos_ = declare_parameter("gripper_close_position", 2600);
    gripper_max_cur_   = declare_parameter("gripper_max_current", 600);
    effort_threshold_  = declare_parameter("gripper_effort_threshold", 400); // mA
    
    joy_timeout_sec_   = declare_parameter("joy_timeout_sec", 0.25);
    alpha_             = declare_parameter("alpha", 0.2); 
    max_acc_           = declare_parameter("max_acc", 2.0);

    // --- ROS 2 Setup ---
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::SensorDataQoS(),
        std::bind(&JoyController::joy_callback, this, std::placeholders::_1));

    crawler_pub_     = create_publisher<custom_interfaces::msg::CrawlerVelocity>("/crawler_driver", 10);
    flipper_pub_     = create_publisher<custom_interfaces::msg::FlipperVelocity>("/flipper_driver", 10);
    arm_twist_pub_   = create_publisher<geometry_msgs::msg::Twist>("/arm_cmd_vel", 10);
    arm_joint_pub_   = create_publisher<sensor_msgs::msg::JointState>("/arm_joint_cmd_vel", 10);
    gripper_pub_     = create_publisher<custom_interfaces::msg::GripperCommand>("/gripper_cmd", 10);
    estop_pub_       = create_publisher<std_msgs::msg::Bool>("/emergency_stop", rclcpp::QoS(1).transient_local());

    gripper_status_sub_ = create_subscription<custom_interfaces::msg::GripperStatus>(
        "/gripper_status", 10,
        [this](const custom_interfaces::msg::GripperStatus::SharedPtr msg) {
          last_gripper_status_ = *msg;
          have_gripper_status_ = true;
        });

    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&JoyController::control_loop, this));

    RCLCPP_INFO(get_logger(), "🚀 Pro JoyController Ready (OPTIONS=DRIVE, SHARE=ARM/JOINT)");
  }

 private:
  enum class Mode { STOP, DRIVE, ARM, JOINT };

  // Button mapping (PS4)
  static constexpr size_t BTN_CROSS     = 0;
  static constexpr size_t BTN_CIRCLE    = 1;
  static constexpr size_t BTN_TRIANGLE  = 2;
  static constexpr size_t BTN_SQUARE    = 3;
  static constexpr size_t BTN_L1        = 4;
  static constexpr size_t BTN_R1        = 5;
  static constexpr size_t BTN_SHARE     = 8;
  static constexpr size_t BTN_OPTIONS   = 9;
  static constexpr size_t BTN_PS        = 10;
  static constexpr size_t AX_LEFT_X     = 0;
  static constexpr size_t AX_LEFT_Y     = 1;
  static constexpr size_t AX_L2         = 2;
  static constexpr size_t AX_RIGHT_X    = 3;
  static constexpr size_t AX_RIGHT_Y    = 4;
  static constexpr size_t AX_R2         = 5;
  static constexpr size_t AX_DPAD_X     = 6;
  static constexpr size_t AX_DPAD_Y     = 7;

  double max_speed_, deadzone_, arm_lin_scale_, arm_ang_scale_, joint_scale_;
  int flipper_speed_, gripper_open_pos_, gripper_close_pos_, gripper_max_cur_, effort_threshold_;
  double joy_timeout_sec_, alpha_, max_acc_;

  Mode mode_;
  int prev_ps_{0}, prev_options_{0}, prev_share_{0}, prev_triangle_{0}, prev_square_{0};
  bool gripper_closing_{false};
  int grasp_count_{0};

  std::mutex joy_mtx_;
  sensor_msgs::msg::Joy last_joy_;
  rclcpp::Time last_joy_time_;
  bool have_joy_{false};

  custom_interfaces::msg::GripperStatus last_gripper_status_;
  bool have_gripper_status_{false};

  std::array<double, 6> prev_t_{}, prev_j_{};

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<custom_interfaces::msg::CrawlerVelocity>::SharedPtr crawler_pub_;
  rclcpp::Publisher<custom_interfaces::msg::FlipperVelocity>::SharedPtr flipper_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_joint_pub_;
  rclcpp::Publisher<custom_interfaces::msg::GripperCommand>::SharedPtr gripper_pub_;
  rclcpp::Subscription<custom_interfaces::msg::GripperStatus>::SharedPtr gripper_status_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float ax(const sensor_msgs::msg::Joy& m, size_t i) { return (i < m.axes.size()) ? m.axes[i] : 0.0f; }
  int btn(const sensor_msgs::msg::Joy& m, size_t i) { return (i < m.buttons.size()) ? m.buttons[i] : 0; }
  float dz(float v) { return (std::abs(v) < deadzone_) ? 0.0f : std::copysign((std::abs(v) - deadzone_) / (1.0f - deadzone_), v); }
  double nonlin(double x) { return std::copysign(x * x, x); }
  double smooth(double raw, double& prev) {
    double d = std::clamp(raw - prev, -max_acc_*0.02, max_acc_*0.02);
    prev = (1.0 - alpha_) * prev + alpha_ * (prev + d);
    return (std::abs(prev) < 1e-4) ? 0.0 : prev;
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(joy_mtx_);
    last_joy_ = *msg; last_joy_time_ = now(); have_joy_ = true;
  }

  void control_loop() {
    sensor_msgs::msg::Joy joy; rclcpp::Time stamp;
    { std::lock_guard<std::mutex> lk(joy_mtx_); if (!have_joy_) return; joy = last_joy_; stamp = last_joy_time_; }
    
    bool timed_out = (now() - stamp).seconds() > joy_timeout_sec_;
    update_mode(joy);
    
    if (mode_ == Mode::DRIVE) {
      custom_interfaces::msg::CrawlerVelocity c;
      c.m1_vel = dz(ax(joy, AX_RIGHT_Y)) * max_speed_;
      c.m2_vel = dz(ax(joy, AX_LEFT_Y)) * max_speed_;
      crawler_pub_->publish(c);

      custom_interfaces::msg::FlipperVelocity f;
      int f1 = (ax(joy, AX_DPAD_Y) < -0.5) ? 1 : (ax(joy, AX_DPAD_Y) > 0.5 ? -1 : 0);
      int f2 = (btn(joy, BTN_CROSS)) ? 1 : (btn(joy, BTN_TRIANGLE) ? -1 : 0);
      int f3 = (btn(joy, BTN_L1)) ? 1 : (ax(joy, AX_L2) < -0.9 ? -1 : 0);
      int f4 = (btn(joy, BTN_R1)) ? 1 : (ax(joy, AX_R2) < -0.9 ? -1 : 0);
      f.flipper_vel = {f1 * flipper_speed_, f2 * flipper_speed_, f3 * flipper_speed_, f4 * flipper_speed_};
      flipper_pub_->publish(f);
    }

    if (mode_ == Mode::ARM || mode_ == Mode::JOINT) {
      double p = btn(joy, BTN_CROSS) ? 0.25 : 1.0;
      if (mode_ == Mode::ARM) {
        geometry_msgs::msg::Twist t;
        t.linear.x = smooth(nonlin(dz(-ax(joy, AX_LEFT_Y))) * arm_lin_scale_ * p, prev_t_[0]);
        t.linear.y = smooth(nonlin(dz( ax(joy, AX_LEFT_X))) * arm_lin_scale_ * p, prev_t_[1]);
        t.linear.z = smooth(nonlin(dz( ax(joy, AX_RIGHT_Y))) * arm_lin_scale_ * p, prev_t_[2]);
        t.angular.z = smooth(nonlin(dz(-ax(joy, AX_RIGHT_X))) * arm_ang_scale_ * p, prev_t_[5]);
        arm_twist_pub_->publish(t);
      } else {
        sensor_msgs::msg::JointState js;
        js.name = {"arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6"};
        js.velocity = {
          smooth(nonlin(dz(-ax(joy, AX_LEFT_Y))) * joint_scale_ * p, prev_j_[0]),
          smooth(nonlin(dz( ax(joy, AX_LEFT_X))) * joint_scale_ * p, prev_j_[1]),
          smooth(nonlin(dz( ax(joy, AX_RIGHT_Y))) * joint_scale_ * p, prev_j_[2]),
          smooth(nonlin(dz(-ax(joy, AX_RIGHT_X))) * joint_scale_ * p, prev_j_[3]),
          smooth((ax(joy, AX_R2) - ax(joy, AX_L2)) * joint_scale_ * p, prev_j_[4]),
          smooth(dz(ax(joy, AX_DPAD_Y)) * joint_scale_ * p, prev_j_[5])
        };
        arm_joint_pub_->publish(js);
      }
      handle_gripper(joy);
    }
  }

  void update_mode(const sensor_msgs::msg::Joy& joy) {
    if (btn(joy, BTN_PS) && !prev_ps_) {
      mode_ = Mode::STOP; std_msgs::msg::Bool m; m.data = true; estop_pub_->publish(m);
      RCLCPP_WARN(get_logger(), "EMERGENCY STOP (Torque OFF)");
    } else if (btn(joy, BTN_OPTIONS) && !prev_options_) {
      mode_ = Mode::DRIVE; std_msgs::msg::Bool m; m.data = false; estop_pub_->publish(m);
      RCLCPP_INFO(get_logger(), "Mode: DRIVE");
    } else if (btn(joy, BTN_SHARE) && !prev_share_) {
      mode_ = (mode_ == Mode::ARM) ? Mode::JOINT : Mode::ARM;
      RCLCPP_INFO(get_logger(), "Mode: %s", (mode_ == Mode::ARM) ? "ARM (IK)" : "JOINT");
    }
    prev_ps_ = btn(joy, BTN_PS); prev_options_ = btn(joy, BTN_OPTIONS); prev_share_ = btn(joy, BTN_SHARE);
  }

  void handle_gripper(const sensor_msgs::msg::Joy& joy) {
    // Grasp Detection: Only active while closing
    if (gripper_closing_ && have_gripper_status_) {
      if (std::abs(last_gripper_status_.current) > effort_threshold_) {
        if (++grasp_count_ >= 3) {
          RCLCPP_INFO(get_logger(), "✊ Object Grasped! Holding at pos %d", last_gripper_status_.position);
          send_gripper_raw(last_gripper_status_.position); // Stay here
          gripper_closing_ = false;
        }
      } else { grasp_count_ = 0; }
    }

    // Trigger on Rising Edge
    if (btn(joy, BTN_TRIANGLE) && !prev_triangle_) {
      RCLCPP_INFO(get_logger(), "📤 Opening Gripper...");
      send_gripper_raw(gripper_open_pos_);
      gripper_closing_ = false;
    } else if (btn(joy, BTN_SQUARE) && !prev_square_) {
      RCLCPP_INFO(get_logger(), "📥 Closing Gripper...");
      send_gripper_raw(gripper_close_pos_);
      gripper_closing_ = true;
      grasp_count_ = 0;
    }
    prev_triangle_ = btn(joy, BTN_TRIANGLE); prev_square_ = btn(joy, BTN_SQUARE);
  }

  void send_gripper_raw(int pos) {
    custom_interfaces::msg::GripperCommand m;
    m.position = pos; m.max_current = gripper_max_cur_;
    gripper_pub_->publish(m);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyController>());
  rclcpp::shutdown();
  return 0;
}
