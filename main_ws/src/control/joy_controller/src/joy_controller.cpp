#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>

#include "custom_interfaces/msg/crawler_velocity.hpp"
#include "custom_interfaces/msg/flipper_velocity.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyController : public rclcpp::Node
{
 public:
  JoyController() : Node("joy_controller"), mode_(Mode::STOP)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyController::joy_callback, this, std::placeholders::_1));

    crawler_publisher_ =
        this->create_publisher<custom_interfaces::msg::CrawlerVelocity>("/crawler_driver", 10);
    flipper_publisher_ =
        this->create_publisher<custom_interfaces::msg::FlipperVelocity>("/flipper_driver", 10);
    arm_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/arm_cmd_vel", 10);
  }

 private:

  enum class Mode
  {
    STOP,
    DRIVE,
    ARM
  };

  static constexpr float MAX_SPEED     = 0.7f; // たぶんm/s
  static constexpr float DEADZONE      = 0.1f; // joystickのdeadzone
  static constexpr int   FLIPPER_SPEED = 1000; // 定数速度
  static constexpr float ARM_LIN_SCALE = 0.3f; // アーム直線速度スケール (m/s)
  static constexpr float ARM_ANG_SCALE = 0.5f; // アーム回転速度スケール (rad/s)

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

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<custom_interfaces::msg::CrawlerVelocity>::SharedPtr crawler_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::FlipperVelocity>::SharedPtr flipper_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_publisher_;

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
    if (button(msg, BUTTON_PS) == 1)
    {
      mode_ = Mode::STOP;
      RCLCPP_INFO(this->get_logger(), "Mode switched to STOP");
    }
    else if (button(msg, BUTTON_OPTIONS) == 1)
    {
      mode_ = Mode::DRIVE;
      RCLCPP_INFO(this->get_logger(), "Mode switched to DRIVE");
    }
    else if (button(msg, BUTTON_SHARE) == 1)
    {
      mode_ = Mode::ARM;
      RCLCPP_INFO(this->get_logger(), "Mode switched to ARM");
    }
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
    float lx = apply_deadzone(axis(msg, AXIS_LEFT_X), DEADZONE);
    float ly = apply_deadzone(axis(msg, AXIS_LEFT_Y), DEADZONE);
    // 右スティック: Z軸 + Yaw回転
    float rx = apply_deadzone(axis(msg, AXIS_RIGHT_X), DEADZONE);
    float ry = apply_deadzone(axis(msg, AXIS_RIGHT_Y), DEADZONE);

    twist.linear.x  = -ly * ARM_LIN_SCALE; // 前後
    twist.linear.y  =  lx * ARM_LIN_SCALE; // 左右
    twist.linear.z  =  ry * ARM_LIN_SCALE; // 上下
    twist.angular.z = -rx * ARM_ANG_SCALE; // Yaw

    return twist;
  }

  /// クローラ速度を計算（STOP/ARM時は {0, 0}）
  std::pair<float, float> compute_crawler_velocity(const sensor_msgs::msg::Joy &msg) const
  {
    if (mode_ != Mode::DRIVE)
    {
      return {0.0f, 0.0f};
    }

    float m1_axis = apply_deadzone(std::clamp(axis(msg, AXIS_LEFT_Y), -0.95f, 0.95f), DEADZONE);
    float m2_axis = apply_deadzone(std::clamp(axis(msg, AXIS_RIGHT_Y), -0.95f, 0.95f), DEADZONE);

    float m1 = std::clamp(-m1_axis * MAX_SPEED, -MAX_SPEED, MAX_SPEED);
    float m2 = std::clamp(m2_axis * MAX_SPEED, -MAX_SPEED, MAX_SPEED);

    return {m1, m2};
  }

  /// フリッパー符号を判定するヘルパー（正ボタン/負ボタンの2入力 → -1, 0, +1）
  static int flipper_sign(bool positive_input, bool negative_input)
  {
    if (positive_input && !negative_input) return 1;
    if (negative_input && !positive_input) return -1;
    return 0;
  }

  /// 4つのフリッパー速度を計算
  std::array<int, 4> compute_flipper_speeds(const sensor_msgs::msg::Joy &msg) const
  {
    // Left backward: D-Pad Up → +1, D-Pad Down → -1
    int f1 = flipper_sign(axis(msg, AXIS_DPAD_Y) == -1.0f,
                          axis(msg, AXIS_DPAD_Y) == 1.0f);
    // Right backward: Cross → +1, Square → -1
    int f2 = flipper_sign(button(msg, BUTTON_CROSS) == 1,
                          button(msg, BUTTON_SQUARE) == 1);
    // Left forward: L1 → +1, L2 → -1
    int f3 = flipper_sign(button(msg, BUTTON_L1) == 1,
                          axis(msg, AXIS_L2) == -1.0f);
    // Right forward: R1 → +1, R2 → -1
    int f4 = flipper_sign(button(msg, BUTTON_R1) == 1,
                          axis(msg, AXIS_R2) == -1.0f);

    return {FLIPPER_SPEED * f1, FLIPPER_SPEED * f2,
            FLIPPER_SPEED * f3, FLIPPER_SPEED * f4};
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
