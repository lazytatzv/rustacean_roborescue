
#include <arm_controller.hpp>

#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_controller");

    ros::Time::init();
    
    ros::Rate loop_rate(60);

    ArmCtlNode arm_node(60);

    int status_flag = 0;

    while (ros::ok()) {
        ros::spinOnce();

        if (status_flag > 2) {
            arm_node.Get_Current();
            arm_node.Check_Error();
            arm_node.Publish_Current();
            status_flag = 0;
        } else {
            status_flag++;
        }

        if (!arm_node.Error_Flag()) {
            // arm_node.GetPresentInfomation();
            // arm_node.Publish_Current();
            arm_node.GetPresentPosition();
            arm_node.update();
        } else {
            arm_node.StopAllMotor();
            arm_node.Reboot_Error_Motor();
        }
        loop_rate.sleep();
    }
    return 0;
}