#include <math.h>

#include <string>

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

class ArmState
{
 private:
  sensor_msgs::JointState js0;
  ros::NodeHandle nh;
  ros::Subscriber sub_joints;
  ros::Publisher joint_pub;

 public:
  ArmState();

  void Callback(const std_msgs::Float64MultiArray &msg);
};

ArmState::ArmState()
{
  sub_joints = nh.subscribe("arm_state", 10, &ArmState::Callback, this);
  joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

  js0.name.resize(6);
  js0.name[0] = "arm_joint1";
  js0.name[1] = "arm_joint2";
  js0.name[2] = "arm_joint3";
  js0.name[3] = "arm_joint4";
  js0.name[4] = "arm_joint5";
  js0.name[5] = "arm_joint6";
  js0.position.resize(6);
}

void ArmState::Callback(const std_msgs::Float64MultiArray &msg)
{
  js0.header.stamp = ros::Time::now();

  js0.position[0] = msg.data[0] / -4;  // ポジション読み出し    loop_rate.sleep();
  js0.position[1] = msg.data[1] / 10;
  js0.position[2] = msg.data[2] / 10;
  js0.position[3] = msg.data[3];
  js0.position[4] = msg.data[4];
  js0.position[5] = msg.data[5];

  joint_pub.publish(js0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_state");

  ArmState arm_state;

  ros::spin();

  //  ros::Rate loop_rate(10);
  //
  //  while (ros::ok())
  //  {
  //    ros::spinOnce();
  //    loop_rate.sleep();
  //  }

  return 0;
}
