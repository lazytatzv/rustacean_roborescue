#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <custom_interfaces/msg/gripper_command.hpp>
#include <custom_interfaces/msg/gripper_status.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>
#include <functional>

/**
 * @brief GripperJoyTeleop (Updated for Rust arm_driver)
 * 
 * このノードは Joy 入力を受け取り、/gripper_cmd トピックを通じて
 * Rust 版 arm_driver に指令を送ります。
 */
class GripperJoyTeleop : public rclcpp::Node
{
public:
  GripperJoyTeleop() : Node("gripper_joy_teleop")
  {
    // パラメータ設定 (デフォルト値を現在の Rust システムに合わせる)
    joy_topic_          = declare_parameter<std::string>("joy_topic", "/joy");
    command_topic_      = declare_parameter<std::string>("command_topic", "/gripper_cmd");
    status_topic_       = declare_parameter<std::string>("status_topic", "/gripper_status");

    // ターゲット位置 (Ticks単位: 0-4095)
    // Rust側 driver::rad_to_ticks は 2048 を 0 rad としている
    open_target_ticks_  = declare_parameter<int>("open_target_ticks", 2500); // 以前の 0.9 rad 相当
    close_target_ticks_ = declare_parameter<int>("close_target_ticks", 1500); // 以前の 0.0 rad 相当

    open_button_  = declare_parameter<int>("open_button", 1);  // 〇ボタン
    close_button_ = declare_parameter<int>("close_button", 0); // ×ボタン

    // 出力
    cmd_pub_ = create_publisher<custom_interfaces::msg::GripperCommand>(command_topic_, 10);

    // 入力
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10,
      std::bind(&GripperJoyTeleop::onJoy, this, std::placeholders::_1));

    status_sub_ = create_subscription<custom_interfaces::msg::GripperStatus>(
      status_topic_, 10,
      [this](const custom_interfaces::msg::GripperStatus::SharedPtr msg) {
        last_status_ = *msg;
        have_status_ = true;
      });

    RCLCPP_INFO(get_logger(), "🚀 GripperJoyTeleop started.");
    RCLCPP_INFO(get_logger(), "   - Pub: %s", command_topic_.c_str());
    RCLCPP_INFO(get_logger(), "   - Open Button: %d, Close Button: %d", open_button_, close_button_);
  }

private:
  std::string joy_topic_;
  std::string command_topic_;
  std::string status_topic_;

  int open_target_ticks_;
  int close_target_ticks_;
  int open_button_;
  int close_button_;

  std::vector<int> last_buttons_;
  custom_interfaces::msg::GripperStatus last_status_;
  bool have_status_{false};

  rclcpp::Publisher<custom_interfaces::msg::GripperCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<custom_interfaces::msg::GripperStatus>::SharedPtr status_sub_;

  void sendCommand(int ticks, const std::string& label)
  {
    custom_interfaces::msg::GripperCommand msg;
    msg.position = ticks;
    cmd_pub_->publish(msg);
    
    RCLCPP_INFO(get_logger(), "📤 Sent Gripper Command [%s]: %d ticks", label.c_str(), ticks);
    
    if (have_status_) {
      RCLCPP_INFO(get_logger(), "   - Current Status: pos=%d, cur=%d mA, temp=%d C", 
                  last_status_.position, last_status_.current, last_status_.temperature);
      if (std::abs(last_status_.current) > 250) {
        RCLCPP_WARN(get_logger(), "   ⚠️ High current detected! Check for stall or overload.");
      }
    } else {
      RCLCPP_WARN(get_logger(), "   ⚠️ No status received from arm_driver yet.");
    }
  }

  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const auto& b = msg->buttons;
    if (last_buttons_.empty()) {
      last_buttons_ = b;
      return;
    }

    // 押し下げエッジ検出
    bool do_open  = (b[open_button_] != 0 && last_buttons_[open_button_] == 0);
    bool do_close = (b[close_button_] != 0 && last_buttons_[close_button_] == 0);
    last_buttons_ = b;

    if (do_open) {
      sendCommand(open_target_ticks_, "OPEN");
    } else if (do_close) {
      sendCommand(close_target_ticks_, "CLOSE");
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperJoyTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
