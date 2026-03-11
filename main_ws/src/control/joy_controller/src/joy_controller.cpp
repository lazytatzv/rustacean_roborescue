#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>

#include "custom_interfaces/msg/crawler_velocity.hpp"
#include "custom_interfaces/msg/flipper_velocity.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyController : public rclcpp::Node
{
 public:
  JoyController() : Node("joy_controller"), mode_(Mode::STOP)
  {
    // ── ROS Parameters (runtime configurable) ──
    this->declare_parameter("max_speed", 0.7);
    this->declare_parameter("deadzone", 0.1);
    this->declare_parameter("flipper_speed", 1000);
    this->declare_parameter("arm_linear_scale", 0.3);
    this->declare_parameter("arm_angular_scale", 0.5);

    max_speed_     = static_cast<float>(this->get_parameter("max_speed").as_double());
    deadzone_      = static_cast<float>(this->get_parameter("deadzone").as_double());
    flipper_speed_ = static_cast<int>(this->get_parameter("flipper_speed").as_int());
    arm_lin_scale_ = static_cast<float>(this->get_parameter("arm_linear_scale").as_double());
    arm_ang_scale_ = static_cast<float>(this->get_parameter("arm_angular_scale").as_double());

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyController::joy_callback, this, std::placeholders::_1));

    crawler_publisher_ =
        this->create_publisher<custom_interfaces::msg::CrawlerVelocity>("/crawler_driver", 10);
    flipper_publisher_ =
        this->create_publisher<custom_interfaces::msg::FlipperVelocity>("/flipper_driver", 10);
    arm_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/arm_cmd_vel", 10);
    estop_publisher_ =
        this->create_publisher<std_msgs::msg::Bool>(
            "/emergency_stop",
            rclcpp::QoS(1).transient_local().reliable());

    RCLCPP_INFO(this->get_logger(),
        "joy_controller started (max_speed=%.2f, deadzone=%.2f, flipper_speed=%d)",
        max_speed_, deadzone_, flipper_speed_);
  }

 private:

  enum class Mode
  {
    STOP,
    DRIVE,
    ARM
  };

  // ── Runtime parameters ──
  float max_speed_;
  float deadzone_;
  int   flipper_speed_;
  float arm_lin_scale_;
  float arm_ang_scale_;

  // ボタン/軸インデックス定数
  static constexpr size_t BUTTON_SHARE   = 8;
  static constexpr size_t BUTTON_OPTIONS = 9;
  static constexpr size_t BUTTON_PS      = 10;
  static constexpr size_t AXIS_LEFT_X    = 0;
  static constexpr size_t AXIS_LEFT_Y    = 1;
  static constexpr size_t AXIS_RIGHT_X   = 3;
  static constexpr size_t AXIS_RIGHT_Y   = 4;
  static constexpr size_t AXIS_L2        = 2;
  static constexpr size_t AXIS_R2        = 5;
  static constexpr size_t AXIS_DPAD_Y    = 7;
  static constexpr size_t BUTTON_CROSS   = 0;
  static constexpr size_t BUTTON_SQUARE  = 2;
  static constexpr size_t BUTTON_L1      = 4;
  static constexpr size_t BUTTON_R1      = 5;

  // 現在の動作モード
  // デフォルトは安全のためSTOP
  Mode mode_;
  bool estop_latched_ = false;  // E-Stop is permanent until restart

  // ── Button edge detection (previous frame state) ──
  // Prevents repeated action on button hold
  int prev_ps_      = 0;
  int prev_options_ = 0;
  int prev_share_   = 0;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<custom_interfaces::msg::CrawlerVelocity>::SharedPtr crawler_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::FlipperVelocity>::SharedPtr flipper_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher_;

  static float apply_deadzone(float val, float threshold)
  {
    return (std::abs(val) < threshold) ? 0.0f : val;
  }

  /// ボタン安全アクセス（範囲外は 0 を返す）
  static int button(const sensor_msgs::msg::Joy &msg, size_t idx)
  {
    return (idx < msg.buttons.size()) ? msg.buttons[idx] : 0;
  }

  /// 軸安全アクセス（範囲外は 0.0 を返す）
  static float axis(const sensor_msgs::msg::Joy &msg, size_t idx)
  {
    return (idx < msg.axes.size()) ? msg.axes[idx] : 0.0f;
  }

  // const参照でok
  void update_mode(const sensor_msgs::msg::Joy &msg)
  {
    int ps      = button(msg, BUTTON_PS);
    int options = button(msg, BUTTON_OPTIONS);
    int share   = button(msg, BUTTON_SHARE);

    // ── E-Stop: rising edge only, latched permanently ──
    if (ps == 1 && prev_ps_ == 0)
    {
      estop_latched_ = true;
      mode_ = Mode::STOP;
      RCLCPP_WARN(this->get_logger(), "⛔ EMERGENCY STOP activated (latched)");

      auto estop_msg = std_msgs::msg::Bool();
      estop_msg.data = true;
      estop_publisher_->publish(estop_msg);
    }
    // ── Mode switches: only on rising edge, blocked after E-Stop ──
    else if (!estop_latched_)
    {
      if (options == 1 && prev_options_ == 0)
      {
        mode_ = Mode::DRIVE;
        RCLCPP_INFO(this->get_logger(), "Mode switched to DRIVE");
      }
      else if (share == 1 && prev_share_ == 0)
      {
        mode_ = Mode::ARM;
        RCLCPP_INFO(this->get_logger(), "Mode switched to ARM");
      }
    }

    // ── Update previous button state ──
    prev_ps_      = ps;
    prev_options_ = options;
    prev_share_   = share;
  }

  /// アーム手先速度を計算（ARMモード時のみ非ゼロ）
  geometry_msgs::msg::Twist compute_arm_twist(const sensor_msgs::msg::Joy &msg) const
  {
    geometry_msgs::msg::Twist twist;
    if (mode_ != Mode::ARM)
    {
      return twist; // all zeros
    }

    // 左スティック: XY平面移動
    float lx = apply_deadzone(axis(msg, AXIS_LEFT_X), deadzone_);
    float ly = apply_deadzone(axis(msg, AXIS_LEFT_Y), deadzone_);
    // 右スティック: Z軸 + Yaw回転
    float rx = apply_deadzone(axis(msg, AXIS_RIGHT_X), deadzone_);
    float ry = apply_deadzone(axis(msg, AXIS_RIGHT_Y), deadzone_);
    // L2/R2 トリガー: Roll/Pitch 回転
    // L2/R2 軸は 1.0(離した状態) → -1.0(押し込み) なので正規化
    float l2 = (1.0f - axis(msg, AXIS_L2)) / 2.0f; // 0.0~1.0
    float r2 = (1.0f - axis(msg, AXIS_R2)) / 2.0f; // 0.0~1.0

    twist.linear.x  = -ly * arm_lin_scale_; // 前後
    twist.linear.y  =  lx * arm_lin_scale_; // 左右
    twist.linear.z  =  ry * arm_lin_scale_; // 上下
    twist.angular.x =  (r2 - l2) * arm_ang_scale_; // Roll  (R2で正, L2で負)
    twist.angular.y =  apply_deadzone(axis(msg, AXIS_DPAD_Y), 0.5f) * arm_ang_scale_; // Pitch (D-Pad上で正, 下で負)
    twist.angular.z = -rx * arm_ang_scale_; // Yaw

    return twist;
  }

  /// クローラ速度を計算（STOP/ARM時は {0, 0}）
  std::pair<float, float> compute_crawler_velocity(const sensor_msgs::msg::Joy &msg) const
  {
    if (mode_ != Mode::DRIVE)
    {
      return {0.0f, 0.0f};
    }

    float m1_axis = apply_deadzone(std::clamp(axis(msg, AXIS_LEFT_Y), -0.95f, 0.95f), deadzone_);
    float m2_axis = apply_deadzone(std::clamp(axis(msg, AXIS_RIGHT_Y), -0.95f, 0.95f), deadzone_);

    float m1 = std::clamp(-m1_axis * max_speed_, -max_speed_, max_speed_);
    float m2 = std::clamp(m2_axis * max_speed_, -max_speed_, max_speed_);

    return {m1, m2};
  }

  /// フリッパー符号を判定するヘルパー（正ボタン/負ボタンの2入力 → -1, 0, +1）
  static int flipper_sign(bool positive_input, bool negative_input)
  {
    if (positive_input && !negative_input) return 1;
    if (negative_input && !positive_input) return -1;
    return 0;
  }

  /// 4つのフリッパー速度を計算（DRIVEモード時のみ非ゼロ）
  std::array<int, 4> compute_flipper_speeds(const sensor_msgs::msg::Joy &msg) const
  {
    if (mode_ != Mode::DRIVE)
    {
      return {0, 0, 0, 0};
    }

    // Left backward: D-Pad Up → +1, D-Pad Down → -1
    int f1 = flipper_sign(axis(msg, AXIS_DPAD_Y) < -0.5f,
                          axis(msg, AXIS_DPAD_Y) > 0.5f);
    // Right backward: Cross → +1, Square → -1
    int f2 = flipper_sign(button(msg, BUTTON_CROSS) == 1,
                          button(msg, BUTTON_SQUARE) == 1);
    // Left forward: L1 → +1, L2 pressed → -1
    int f3 = flipper_sign(button(msg, BUTTON_L1) == 1,
                          axis(msg, AXIS_L2) < -0.9f);
    // Right forward: R1 → +1, R2 pressed → -1
    int f4 = flipper_sign(button(msg, BUTTON_R1) == 1,
                          axis(msg, AXIS_R2) < -0.9f);

    return {flipper_speed_ * f1, flipper_speed_ * f2,
            flipper_speed_ * f3, flipper_speed_ * f4};
  }

  void joy_callback(const sensor_msgs::msg::Joy &msg)
  {
    update_mode(msg);

    // クローラ & フリッパー（DRIVEモード時のみ非ゼロ）
    auto [m1_vel, m2_vel] = compute_crawler_velocity(msg);
    auto flipper_speeds    = compute_flipper_speeds(msg);

    auto crawler_msg      = custom_interfaces::msg::CrawlerVelocity();
    crawler_msg.m1_vel    = m1_vel;
    crawler_msg.m2_vel    = m2_vel;
    crawler_publisher_->publish(crawler_msg);

    auto flipper_msg      = custom_interfaces::msg::FlipperVelocity();
    flipper_msg.flipper_vel.assign(flipper_speeds.begin(), flipper_speeds.end());
    flipper_publisher_->publish(flipper_msg);

    // アーム（ARMモード時のみ非ゼロ）
    auto arm_twist = compute_arm_twist(msg);
    arm_publisher_->publish(arm_twist);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyController>());
  rclcpp::shutdown();
  return 0;
}
