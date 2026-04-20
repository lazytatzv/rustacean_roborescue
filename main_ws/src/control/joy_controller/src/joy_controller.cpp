#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_jog.hpp"
#include "custom_interfaces/msg/crawler_velocity.hpp"
#include "custom_interfaces/msg/flipper_velocity.hpp"
#include "custom_interfaces/msg/gripper_command.hpp"
#include "custom_interfaces/msg/gripper_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#ifdef HAVE_MOVEIT_MSGS
#include "moveit_msgs/srv/servo_command_type.hpp"
#endif
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

// Mode cycle (SHARE button):
//   STOP/DRIVE  ->  ARM
//   ARM         ->  JOINT
//   JOINT       ->  SERVO  (when use_servo_mode=true)
//   SERVO       ->  ARM
//
// SERVO mode uses MoveIt Servo: requires arm_backend:=ros2_control
// and moveit_servo.launch.py to be running.
// Inputs auto-switch between TwistStamped (IK) and JointJog (per-joint):
//   Twist : LY=fwd/back, LX=left/right, RY=up/down
//   Joint : DPad-X=J1, DPad-Y=J2, DPad-Y+△=J3, DPad-Y+□=J4, L1/R1=J5, L2/R2=J6
//
// Precision mode (ARM/JOINT/SERVO): hold × for 0.25x scale.
// Gripper (ARM/JOINT): △=open, □=close.
// Gripper (SERVO): △/□ are joint modifiers; use ○ to toggle open/close.

class JoyController : public rclcpp::Node
{
 public:
  JoyController() : Node("joy_controller"), mode_(Mode::STOP)
  {
    declare_parameter("max_speed", 0.7);
    declare_parameter("deadzone", 0.1);
    declare_parameter("flipper_speed", 1000);
    declare_parameter("arm_linear_scale", 0.6);
    declare_parameter("arm_angular_scale", 0.5);
    declare_parameter("joint_speed_scale", 0.5);
    declare_parameter("servo_twist_scale", 0.6);
    declare_parameter("servo_joint_scale", 0.8);
    declare_parameter("gripper_open_position", 1500);
    declare_parameter("gripper_close_position", 2600);
    declare_parameter("gripper_max_current", 300);
    declare_parameter("gripper_effort_threshold", 250); // mA
    declare_parameter("use_grasp_detection", true);
    declare_parameter("use_servo_mode", true);
    declare_parameter("servo_command_frame", std::string("arm6_link"));
    declare_parameter("joy_timeout_sec", 0.25);
    declare_parameter("alpha", 0.2);        // low-pass coefficient
    declare_parameter("max_acc", 2.0);      // rad/s^2 acceleration limit
    declare_parameter("decay_rate", 0.85);  // velocity decay on timeout

    max_speed_         = static_cast<float>(get_parameter("max_speed").as_double());
    deadzone_          = static_cast<float>(get_parameter("deadzone").as_double());
    flipper_speed_     = static_cast<int>(get_parameter("flipper_speed").as_int());
    arm_lin_scale_     = static_cast<float>(get_parameter("arm_linear_scale").as_double());
    arm_ang_scale_     = static_cast<float>(get_parameter("arm_angular_scale").as_double());
    joint_scale_       = static_cast<float>(get_parameter("joint_speed_scale").as_double());
    servo_twist_scale_ = static_cast<float>(get_parameter("servo_twist_scale").as_double());
    servo_joint_scale_ = static_cast<float>(get_parameter("servo_joint_scale").as_double());
    gripper_open_pos_  = static_cast<uint16_t>(get_parameter("gripper_open_position").as_int());
    gripper_close_pos_ = static_cast<uint16_t>(get_parameter("gripper_close_position").as_int());
    gripper_max_cur_   = static_cast<uint16_t>(get_parameter("gripper_max_current").as_int());
    effort_threshold_  = static_cast<int16_t>(get_parameter("gripper_effort_threshold").as_int());
    use_grasp_detection_ = get_parameter("use_grasp_detection").as_bool();
    use_servo_mode_    = get_parameter("use_servo_mode").as_bool();
    command_frame_     = get_parameter("servo_command_frame").as_string();
    joy_timeout_sec_   = get_parameter("joy_timeout_sec").as_double();
    alpha_             = get_parameter("alpha").as_double();
    max_acc_           = get_parameter("max_acc").as_double();
    decay_rate_        = get_parameter("decay_rate").as_double();

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::SensorDataQoS(),
        std::bind(&JoyController::joy_callback, this, std::placeholders::_1));

    crawler_pub_     = create_publisher<custom_interfaces::msg::CrawlerVelocity>("/crawler_driver", 10);
    flipper_pub_     = create_publisher<custom_interfaces::msg::FlipperVelocity>("/flipper_driver", 10);
    arm_twist_pub_   = create_publisher<geometry_msgs::msg::Twist>("/arm_cmd_vel", 10);
    arm_joint_pub_   = create_publisher<sensor_msgs::msg::JointState>("/arm_joint_cmd_vel", 10);
    gripper_pub_     = create_publisher<custom_interfaces::msg::GripperCommand>("/gripper_cmd", 10);
    gripper_status_sub_ = create_subscription<custom_interfaces::msg::GripperStatus>(
        "/gripper_status", 10,
        [this](const custom_interfaces::msg::GripperStatus::SharedPtr msg) {
          last_gripper_status_ = *msg;
          have_gripper_status_ = true;
        });
    estop_pub_       = create_publisher<std_msgs::msg::Bool>(
        "/emergency_stop", rclcpp::QoS(1).transient_local().reliable());
    servo_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);
    servo_joint_pub_ = create_publisher<control_msgs::msg::JointJog>(
        "/servo_node/delta_joint_cmds", 10);

#ifdef HAVE_MOVEIT_MSGS
    servo_switch_client_ = create_client<moveit_msgs::srv::ServoCommandType>(
        "/servo_node/switch_command_type");
#endif

    // 50 Hz control loop
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / 50.0)),
        std::bind(&JoyController::control_loop, this));

    RCLCPP_INFO(get_logger(),
                "joy_controller started (STOP; OPTIONS=DRIVE, SHARE=ARM/JOINT/SERVO)");
  }

 private:
  enum class Mode { STOP, DRIVE, ARM, JOINT, SERVO };

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
  static constexpr size_t AX_RIGHT_X    = 3;
  static constexpr size_t AX_RIGHT_Y    = 4;
  static constexpr size_t AX_L2         = 2;
  static constexpr size_t AX_R2         = 5;
  static constexpr size_t AX_DPAD_X     = 6;
  static constexpr size_t AX_DPAD_Y     = 7;
  static constexpr double kActiveEps    = 1e-4;
  static constexpr int    kJointJog     = 0;
  static constexpr int    kTwist        = 1;

  // Parameters
  float    max_speed_, deadzone_, arm_lin_scale_, arm_ang_scale_, joint_scale_;
  float    servo_twist_scale_, servo_joint_scale_;
  int      flipper_speed_;
  uint16_t gripper_open_pos_, gripper_close_pos_, gripper_max_cur_;
  int16_t  effort_threshold_;
  bool     use_grasp_detection_;
  bool     use_servo_mode_;
  std::string command_frame_;
  double   joy_timeout_sec_, alpha_, max_acc_, decay_rate_;

  // Mode state
  Mode mode_;
  bool estop_latched_{false};
  int  prev_ps_{0}, prev_options_{0}, prev_share_{0};
  int  prev_triangle_{0}, prev_square_{0}, prev_circle_{0};
  bool gripper_open_{true};  // toggle state for SERVO mode gripper
  bool gripper_closing_{false};
  int  grasp_consecutive_count_{0};
  int  servo_cmd_type_{-1};  // -1=unknown, kJointJog or kTwist

  // Joy message (written by callback, read by timer)
  std::mutex            joy_mtx_;
  sensor_msgs::msg::Joy last_joy_;
  rclcpp::Time          last_joy_time_;
  bool                  have_joy_{false};

  // Gripper Status
  custom_interfaces::msg::GripperStatus last_gripper_status_;
  bool have_gripper_status_{false};

  // Filter state (written and read only in timer, no mutex needed)
  std::array<double, 6> prev_t_{};  // twist: tx,ty,tz,rx,ry,rz
  std::array<double, 6> prev_j_{};  // joint: j1-j6

  // ROS handles
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<custom_interfaces::msg::CrawlerVelocity>::SharedPtr crawler_pub_;
  rclcpp::Publisher<custom_interfaces::msg::FlipperVelocity>::SharedPtr flipper_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_joint_pub_;
  rclcpp::Publisher<custom_interfaces::msg::GripperCommand>::SharedPtr gripper_pub_;
  rclcpp::Subscription<custom_interfaces::msg::GripperStatus>::SharedPtr gripper_status_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr servo_joint_pub_;
#ifdef HAVE_MOVEIT_MSGS
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr servo_switch_client_;
#endif
  rclcpp::TimerBase::SharedPtr timer_;

  // ──────────────────── Input helpers ────────────────────────────────────

  static int btn(const sensor_msgs::msg::Joy& m, size_t i)
  {
    return (i < m.buttons.size()) ? m.buttons[i] : 0;
  }

  static float ax(const sensor_msgs::msg::Joy& m, size_t i)
  {
    return (i < m.axes.size()) ? m.axes[i] : 0.0f;
  }

  // PS trigger: resting=+1, full press=-1 → normalize to [0,1]
  static float trig(const sensor_msgs::msg::Joy& m, size_t i)
  {
    return (1.0f - ax(m, i)) * 0.5f;
  }

  // Deadzone with rescaling so output reaches ±1 at full deflection
  float dz(float v) const
  {
    if (std::abs(v) < deadzone_) return 0.0f;
    return std::copysign((std::abs(v) - deadzone_) / (1.0f - deadzone_), v);
  }

  static double nonlin(double x) { return std::copysign(x * x, x); }

  // Low-pass + acceleration-limit filter; updates prev in place
  double smooth(double raw, double& prev) const
  {
    const double max_d = max_acc_ * (1.0 / 50.0);
    const double accel_clipped = prev + std::clamp(raw - prev, -max_d, max_d);
    const double filtered = (1.0 - alpha_) * prev + alpha_ * accel_clipped;
    prev = filtered;
    return (std::abs(filtered) < kActiveEps) ? 0.0 : filtered;
  }

  static bool active(double v) { return std::abs(v) > kActiveEps; }

  void decay_filters()
  {
    for (auto& p : prev_t_) p *= decay_rate_;
    for (auto& p : prev_j_) p *= decay_rate_;
  }

  // ──────────────────── MoveIt Servo ─────────────────────────────────────

  void request_servo_type(int type)
  {
    if (servo_cmd_type_ == type) return;
#ifdef HAVE_MOVEIT_MSGS
    if (!servo_switch_client_->service_is_ready()) return;
    auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    req->command_type = type;
    servo_switch_client_->async_send_request(
        req, [this, type](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture f) {
          if (f.get()->success) {
            servo_cmd_type_ = type;
          }
        });
#else
    servo_cmd_type_ = type;
#endif
  }

  void publish_servo_zero_twist(const rclcpp::Time& t)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp    = t;
    msg.header.frame_id = command_frame_;
    servo_twist_pub_->publish(msg);
  }

  void publish_servo_zero_joint(const rclcpp::Time& t)
  {
    control_msgs::msg::JointJog jog;
    jog.header.stamp    = t;
    jog.duration        = 1.0 / 50.0;
    jog.joint_names     = {"arm_joint1", "arm_joint2", "arm_joint3",
                           "arm_joint4", "arm_joint5", "arm_joint6"};
    jog.velocities      = {0, 0, 0, 0, 0, 0};
    servo_joint_pub_->publish(jog);
  }

  // ──────────────────── Mode management ──────────────────────────────────

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(joy_mtx_);
    last_joy_      = *msg;
    last_joy_time_ = now();
    have_joy_      = true;
  }

  void update_mode(const sensor_msgs::msg::Joy& joy)
  {
    const int ps      = btn(joy, BTN_PS);
    const int options = btn(joy, BTN_OPTIONS);
    const int share   = btn(joy, BTN_SHARE);

    if (ps == 1 && prev_ps_ == 0) {
      estop_latched_ = true;
      mode_          = Mode::STOP;
      std_msgs::msg::Bool msg;
      msg.data = true;
      estop_pub_->publish(msg);
      RCLCPP_WARN(get_logger(), "EMERGENCY STOP");
    } else if (options == 1 && prev_options_ == 0) {
      if (estop_latched_) {
        estop_latched_ = false;
        std_msgs::msg::Bool msg;
        msg.data = false;
        estop_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "E-STOP CLEARED -> DRIVE");
      } else {
        RCLCPP_INFO(get_logger(), "Mode: DRIVE");
      }
      mode_ = Mode::DRIVE;
    } else if (share == 1 && prev_share_ == 0) {
      if (estop_latched_) {
        estop_latched_ = false;
        std_msgs::msg::Bool msg;
        msg.data = false;
        estop_pub_->publish(msg);
        mode_ = Mode::ARM;
        RCLCPP_INFO(get_logger(), "E-STOP CLEARED -> ARM");
      } else {
        switch (mode_) {
          case Mode::ARM:
            mode_ = Mode::JOINT;
            RCLCPP_INFO(get_logger(), "Mode: JOINT (Rust direct)");
            break;
          case Mode::JOINT:
            if (use_servo_mode_) {
              mode_          = Mode::SERVO;
              servo_cmd_type_ = -1;
              RCLCPP_INFO(get_logger(), "Mode: SERVO (MoveIt Servo; needs moveit_servo.launch.py)");
            } else {
              mode_ = Mode::ARM;
              RCLCPP_INFO(get_logger(), "Mode: ARM (Rust IK)");
            }
            break;
          case Mode::SERVO:
            mode_ = Mode::ARM;
            RCLCPP_INFO(get_logger(), "Mode: ARM (Rust IK)");
            break;
          default:
            mode_ = Mode::ARM;
            RCLCPP_INFO(get_logger(), "Mode: ARM (Rust IK)");
            break;
        }
      }
    }

    prev_ps_      = ps;
    prev_options_ = options;
    prev_share_   = share;
  }

  // ──────────────────── 50 Hz control loop ───────────────────────────────

  void control_loop()
  {
    sensor_msgs::msg::Joy joy;
    rclcpp::Time          stamp;
    {
      std::lock_guard<std::mutex> lk(joy_mtx_);
      if (!have_joy_) return;
      joy   = last_joy_;
      stamp = last_joy_time_;
    }

    const rclcpp::Time now_t      = now();
    const bool         timed_out  = (now_t - stamp).seconds() > joy_timeout_sec_;

    update_mode(joy);
    publish_crawler(joy, timed_out);
    publish_arm(joy, now_t, timed_out);
    handle_gripper(joy);
  }

  // ──────────────────── Crawler / flipper ────────────────────────────────

  void publish_crawler(const sensor_msgs::msg::Joy& joy, bool timed_out)
  {
    custom_interfaces::msg::CrawlerVelocity c_msg;
    custom_interfaces::msg::FlipperVelocity f_msg;
    c_msg.m1_vel = 0.0f;
    c_msg.m2_vel = 0.0f;
    f_msg.flipper_vel = {0, 0, 0, 0};

    if (mode_ == Mode::DRIVE && !timed_out) {
      c_msg.m1_vel = std::clamp(dz(ax(joy, AX_RIGHT_Y)) * max_speed_, -max_speed_, max_speed_);
      c_msg.m2_vel = std::clamp(dz(ax(joy, AX_LEFT_Y)) * max_speed_, -max_speed_, max_speed_);
      int f1 = (ax(joy, AX_DPAD_Y) < -0.5f) ? 1 : (ax(joy, AX_DPAD_Y) > 0.5f ? -1 : 0);
      int f2 = (btn(joy, BTN_CROSS) == 1) ? 1 : (btn(joy, BTN_TRIANGLE) == 1 ? -1 : 0);
      int f3 = (btn(joy, BTN_L1) == 1) ? 1 : (ax(joy, AX_L2) < -0.9f ? -1 : 0);
      int f4 = (btn(joy, BTN_R1) == 1) ? 1 : (ax(joy, AX_R2) < -0.9f ? -1 : 0);
      f_msg.flipper_vel = {
          static_cast<int32_t>(f1 * flipper_speed_), static_cast<int32_t>(f2 * flipper_speed_),
          static_cast<int32_t>(f3 * flipper_speed_), static_cast<int32_t>(f4 * flipper_speed_)};
    } else if (mode_ == Mode::DRIVE && timed_out) {
      // Joystick lost; send explicit zero to stop immediately
    } else {
      const bool wants_drive = std::abs(ax(joy, AX_RIGHT_Y)) > deadzone_ ||
                               std::abs(ax(joy, AX_LEFT_Y)) > deadzone_;
      if (wants_drive) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Drive input ignored – not in DRIVE mode (press OPTIONS)");
      }
    }
    crawler_pub_->publish(c_msg);
    flipper_pub_->publish(f_msg);
  }

  // ──────────────────── Arm control ──────────────────────────────────────

  void publish_arm(const sensor_msgs::msg::Joy& joy, const rclcpp::Time& t, bool timed_out)
  {
    if (mode_ != Mode::ARM && mode_ != Mode::JOINT && mode_ != Mode::SERVO) {
      decay_filters();
      return;
    }

    if (timed_out) {
      decay_filters();
      if (mode_ == Mode::SERVO) {
        // Keep servo alive with zero commands to prevent timeout
        if (servo_cmd_type_ == kJointJog) {
          publish_servo_zero_joint(t);
        } else {
          publish_servo_zero_twist(t);
        }
      }
      return;
    }

    const float precision = (btn(joy, BTN_CROSS) == 1) ? 0.25f : 1.0f;
    const float l2        = trig(joy, AX_L2);
    const float r2        = trig(joy, AX_R2);

    if (mode_ == Mode::ARM) {
      geometry_msgs::msg::Twist msg;
      msg.linear.x  = smooth(nonlin(dz(-ax(joy, AX_LEFT_Y)))  * arm_lin_scale_ * precision, prev_t_[0]);
      msg.linear.y  = smooth(nonlin(dz( ax(joy, AX_LEFT_X)))  * arm_lin_scale_ * precision, prev_t_[1]);
      msg.linear.z  = smooth(nonlin(dz( ax(joy, AX_RIGHT_Y))) * arm_lin_scale_ * precision, prev_t_[2]);
      msg.angular.x = smooth(static_cast<double>(r2 - l2)     * arm_ang_scale_ * precision, prev_t_[3]);
      msg.angular.y = smooth(nonlin(dz( ax(joy, AX_DPAD_Y)))  * arm_ang_scale_ * precision, prev_t_[4]);
      msg.angular.z = smooth(nonlin(dz(-ax(joy, AX_RIGHT_X))) * arm_ang_scale_ * precision, prev_t_[5]);
      arm_twist_pub_->publish(msg);

    } else if (mode_ == Mode::JOINT) {
      sensor_msgs::msg::JointState msg;
      msg.name     = {"arm_joint1", "arm_joint2", "arm_joint3",
                      "arm_joint4", "arm_joint5", "arm_joint6"};
      msg.velocity = {
          smooth(nonlin(dz(-ax(joy, AX_LEFT_Y)))  * joint_scale_ * precision, prev_j_[0]),
          smooth(nonlin(dz( ax(joy, AX_LEFT_X)))  * joint_scale_ * precision, prev_j_[1]),
          smooth(nonlin(dz( ax(joy, AX_RIGHT_Y))) * joint_scale_ * precision, prev_j_[2]),
          smooth(nonlin(dz(-ax(joy, AX_RIGHT_X))) * joint_scale_ * precision, prev_j_[3]),
          smooth(static_cast<double>(r2 - l2)     * joint_scale_ * precision, prev_j_[4]),
          smooth(nonlin(dz( ax(joy, AX_DPAD_Y)))  * joint_scale_ * precision, prev_j_[5]),
      };
      arm_joint_pub_->publish(msg);

    } else {  // SERVO
      publish_servo(joy, t, l2, r2, precision);
    }
  }

  void publish_servo(const sensor_msgs::msg::Joy& joy, const rclcpp::Time& t,
                     float l2, float r2, float precision)
  {
    // Twist inputs
    const double tx_r = nonlin(dz(ax(joy, AX_LEFT_Y)))  * servo_twist_scale_ * precision;
    const double ty_r = nonlin(dz(ax(joy, AX_LEFT_X)))  * servo_twist_scale_ * precision;
    const double tz_r = nonlin(dz(ax(joy, AX_RIGHT_Y))) * servo_twist_scale_ * precision;

    // Joint jog inputs (DPad-Y + modifier buttons select which joint)
    const double dpy = nonlin(dz(-ax(joy, AX_DPAD_Y)));
    const int    tri = btn(joy, BTN_TRIANGLE);
    const int    sq  = btn(joy, BTN_SQUARE);
    const double j1_r = nonlin(dz(ax(joy, AX_DPAD_X))) * servo_joint_scale_ * precision;
    const double j2_r = dpy * servo_joint_scale_ * precision * (1 - sq) * (1 - tri);
    const double j3_r = dpy * servo_joint_scale_ * precision * (1 - sq) * tri;
    const double j4_r = dpy * servo_joint_scale_ * precision * sq * (1 - tri);
    const double j5_r = static_cast<double>(btn(joy, BTN_L1) - btn(joy, BTN_R1))
                        * servo_joint_scale_ * precision;
    const double j6_r = nonlin(static_cast<double>(r2 - l2)) * servo_joint_scale_ * precision;

    const double tx = smooth(tx_r, prev_t_[0]);
    const double ty = smooth(ty_r, prev_t_[1]);
    const double tz = smooth(tz_r, prev_t_[2]);
    const double j1 = smooth(j1_r, prev_j_[0]);
    const double j2 = smooth(j2_r, prev_j_[1]);
    const double j3 = smooth(j3_r, prev_j_[2]);
    const double j4 = smooth(j4_r, prev_j_[3]);
    const double j5 = smooth(j5_r, prev_j_[4]);
    const double j6 = smooth(j6_r, prev_j_[5]);

    const bool twist_active = active(tx) || active(ty) || active(tz);
    const bool joint_active = active(j1) || active(j2) || active(j3) ||
                               active(j4) || active(j5) || active(j6);

    if (joint_active) {
      if (servo_cmd_type_ != kJointJog) {
        request_servo_type(kJointJog);
        publish_servo_zero_twist(t);
        return;
      }
      control_msgs::msg::JointJog jog;
      jog.header.stamp    = t;
      jog.header.frame_id = "base_link";
      jog.duration        = 1.0 / 50.0;
      jog.joint_names     = {"arm_joint1", "arm_joint2", "arm_joint3",
                             "arm_joint4", "arm_joint5", "arm_joint6"};
      jog.velocities      = {j1, j2, j3, j4, j5, j6};
      servo_joint_pub_->publish(jog);
    } else if (twist_active) {
      if (servo_cmd_type_ != kTwist) {
        request_servo_type(kTwist);
        publish_servo_zero_joint(t);
        return;
      }
      geometry_msgs::msg::TwistStamped twist;
      twist.header.stamp    = t;
      twist.header.frame_id = command_frame_;
      twist.twist.linear.x  = tx;
      twist.twist.linear.y  = ty;
      twist.twist.linear.z  = tz;
      servo_twist_pub_->publish(twist);
    } else {
      // Idle: send zero to keep servo from timing out
      if (servo_cmd_type_ == kJointJog) {
        publish_servo_zero_joint(t);
      } else {
        publish_servo_zero_twist(t);
      }
    }
  }

  // ──────────────────── Gripper ──────────────────────────────────────────

  void handle_gripper(const sensor_msgs::msg::Joy& joy)
  {
    if (mode_ != Mode::ARM && mode_ != Mode::JOINT && mode_ != Mode::SERVO) {
      prev_triangle_ = btn(joy, BTN_TRIANGLE);
      prev_square_   = btn(joy, BTN_SQUARE);
      prev_circle_   = btn(joy, BTN_CIRCLE);
      return;
    }

    // Grasp detection logic (only when closing)
    if (use_grasp_detection_ && gripper_closing_ && have_gripper_status_) {
      if (std::abs(last_gripper_status_.current) > effort_threshold_) {
        grasp_consecutive_count_++;
        if (grasp_consecutive_count_ >= 3) {
          // Grasp detected: Hold current position
          RCLCPP_INFO(get_logger(), "✊ Grasp detected! Holding position at %d ticks (Current: %d mA)", 
                      last_gripper_status_.position, last_gripper_status_.current);
          
          custom_interfaces::msg::GripperCommand g_msg;
          g_msg.max_current = gripper_max_cur_;
          g_msg.position    = last_gripper_status_.position;
          gripper_pub_->publish(g_msg);
          
          gripper_closing_ = false;
          grasp_consecutive_count_ = 0;
        }
      } else {
        grasp_consecutive_count_ = 0;
      }
    }

    if (mode_ == Mode::SERVO) {
      // △/□ are joint modifiers in SERVO mode; ○ toggles gripper
      const int circle = btn(joy, BTN_CIRCLE);
      if (circle == 1 && prev_circle_ == 0) {
        gripper_open_ = !gripper_open_;
        send_gripper(gripper_open_);
      }
      prev_circle_ = circle;
      prev_triangle_ = btn(joy, BTN_TRIANGLE);
      prev_square_   = btn(joy, BTN_SQUARE);
      return;
    }

    // ARM / JOINT: △=open, □=close (rising edge)
    const int tri = btn(joy, BTN_TRIANGLE);
    const int sq  = btn(joy, BTN_SQUARE);
    if (tri == 1 && prev_triangle_ == 0) {
      send_gripper(true);
    } else if (sq == 1 && prev_square_ == 0) {
      send_gripper(false);
    }
    prev_triangle_ = tri;
    prev_square_   = sq;
    prev_circle_   = btn(joy, BTN_CIRCLE);
  }

  void send_gripper(bool open)
  {
    custom_interfaces::msg::GripperCommand g_msg;
    g_msg.max_current = gripper_max_cur_;
    g_msg.position    = open ? gripper_open_pos_ : gripper_close_pos_;
    gripper_pub_->publish(g_msg);
    
    gripper_closing_ = !open;
    grasp_consecutive_count_ = 0;

    RCLCPP_INFO(get_logger(), "📤 Gripper command sent: %s (%d ticks)", 
                open ? "OPEN" : "CLOSE", g_msg.position);
    
    if (have_gripper_status_) {
      RCLCPP_INFO(get_logger(), "   - Current Status: Pos=%d, Current=%d mA, Temp=%d C", 
                  last_gripper_status_.position, last_gripper_status_.current, last_gripper_status_.temperature);
    }
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyController>());
  rclcpp::shutdown();
  return 0;
}
