#include "jsk_rviz_plugins/OverlayText.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"

#define ARM_PUB_SIZE 8
#define MAXON_PUB_SIZE 7

class Operator {
   private:
    std_msgs::Float32 leftjoypad1, leftjoypad2, rightjoypad1, rightjoypad2;
    std_msgs::Int32 cross, square, triangle, circle;
    std_msgs::Int32 up, down, right, left;
    std_msgs::Int32 start, seleCt, L1, L2, R1, R2, PS;

    ros::NodeHandle nh;

    ros::Subscriber joysub;
    ros::Publisher arm_pub;
    ros::Publisher maxon_pub;
    ros::Publisher text_pub;

    std_msgs::Int32MultiArray maxon_vel, arm;
    jsk_rviz_plugins::OverlayText text;

    int mode;
    bool allStop;
    std::string menu[7];
    int s, n, h;
    bool ps4;

   public:
    Operator();
    ~Operator(){};
    void Callback(const sensor_msgs::Joy::ConstPtr &joy_msg);
    void InitArmData();
    void InitMaxonData();
    void InputJoy(const sensor_msgs::Joy::ConstPtr &joy_msg);
    void MaxonOpe();
    void ArmOpe();
};

Operator::Operator() : menu{"Stop", "Drive", "Arm1", "Arm2", "Arm_Start", "Arm_Init", "Arm_Normal"} {
    joysub = nh.subscribe("joy", 10, &Operator::Callback, this);
    arm_pub = nh.advertise<std_msgs::Int32MultiArray>("arm", 1);
    maxon_pub = nh.advertise<std_msgs::Int32MultiArray>("maxon", 100);
    text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("text", 10);

    mode = 0;
    allStop = false;

    maxon_vel.data.resize(MAXON_PUB_SIZE);
    arm.data.resize(ARM_PUB_SIZE);

    s = 10;
    n = 60000;
    h = 150000;

    text.action = jsk_rviz_plugins::OverlayText::ADD;
    text.line_width = 1;
    text.text_size = 14;
    text.font = "Ubuntu";
    text.width = 400;
    text.height = 100;
    text.left = 0;
    text.top = 0;

    std_msgs::ColorRGBA color1, color2;
    color1.r = 0;
    color1.g = 0;
    color1.b = 0;
    color1.a = 0.4;
    text.bg_color = color1;

    color2.r = 0 / 255;
    color2.g = 255.0 / 255;
    color2.b = 255.0 / 255;
    color2.a = 1;
    text.fg_color = color2;

    ps4 = false;
}

void Operator::InitArmData() {
    for (int i = 0; i < ARM_PUB_SIZE; i++) {
        arm.data[i] = 0;
    }
}

void Operator::InitMaxonData() {
    for (int i = 0; i < MAXON_PUB_SIZE; i++) {
        maxon_vel.data[i] = 0;
    }
}

void Operator::InputJoy(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    if (ps4) {
        leftjoypad1.data = joy_msg->axes[0];
        leftjoypad2.data = joy_msg->axes[1];
        rightjoypad1.data = joy_msg->axes[3];
        rightjoypad2.data = joy_msg->axes[4];
        if (joy_msg->axes[7] == 1) {
            up.data = 1;
            down.data = 0;
        } else if (joy_msg->axes[7] == -1) {
            down.data = 1;
            up.data = 0;
        } else {
            up.data = 0;
            down.data = 0;
        }
        if (joy_msg->axes[6] == 1) {
            left.data = 1;
            right.data = 0;
        } else if (joy_msg->axes[6] == -1) {
            left.data = 0;
            right.data = 1;
        } else {
            left.data = 0;
            right.data = 0;
        }

        cross.data = joy_msg->buttons[0];
        circle.data = joy_msg->buttons[1];
        triangle.data = joy_msg->buttons[2];
        square.data = joy_msg->buttons[3];
        start.data = joy_msg->buttons[9];   // option
        seleCt.data = joy_msg->buttons[8];  // share
        L1.data = joy_msg->buttons[4];
        R1.data = joy_msg->buttons[5];
        L2.data = joy_msg->buttons[6];
        R2.data = joy_msg->buttons[7];
        PS.data = joy_msg->buttons[10];
    } else {
        leftjoypad1.data = joy_msg->axes[0];
        leftjoypad2.data = joy_msg->axes[1];
        rightjoypad1.data = joy_msg->axes[3];
        rightjoypad2.data = joy_msg->axes[4];

        cross.data = joy_msg->buttons[0];
        circle.data = joy_msg->buttons[1];
        triangle.data = joy_msg->buttons[2];
        square.data = joy_msg->buttons[3];
        up.data = joy_msg->buttons[13];
        down.data = joy_msg->buttons[14];
        left.data = joy_msg->buttons[15];
        right.data = joy_msg->buttons[16];
        start.data = joy_msg->buttons[9];
        seleCt.data = joy_msg->buttons[8];
        L1.data = joy_msg->buttons[4];
        R1.data = joy_msg->buttons[5];
        L2.data = joy_msg->buttons[6];
        R2.data = joy_msg->buttons[7];
        PS.data = joy_msg->buttons[10];
    }
}

void Operator::MaxonOpe() {
    if (abs(leftjoypad1.data) > 0.4) {
        maxon_vel.data[1] = n * leftjoypad1.data;
        maxon_vel.data[2] = n * leftjoypad1.data;
    } else if (abs(leftjoypad2.data) > 0.4) {
        maxon_vel.data[1] = n * leftjoypad2.data;
        maxon_vel.data[2] = -n * leftjoypad2.data;
    } else {
        maxon_vel.data[1] = 0;
        maxon_vel.data[2] = 0;
    }

    if (rightjoypad2.data == 1 || rightjoypad2.data == -1) {
        if (R2.data) {
            maxon_vel.data[3] = -h * rightjoypad2.data;
        } else {
            maxon_vel.data[3] = 0;
        }
        if (R1.data) {
            maxon_vel.data[4] = -h * rightjoypad2.data;
        } else {
            maxon_vel.data[4] = 0;
        }
        if (L2.data) {
            maxon_vel.data[5] = -h * rightjoypad2.data;
        } else {
            maxon_vel.data[5] = 0;
        }
        if (L1.data) {
            maxon_vel.data[6] = -h * rightjoypad2.data;
        } else {
            maxon_vel.data[6] = 0;
        }
    } else {
        for (int i = 3; i < 7; i++) {
            maxon_vel.data[i] = 0;
        }
    }

    maxon_vel.data[0] = mode;
}

void Operator::ArmOpe() {
    if (mode == 2 || mode == 3 || mode==4 || mode==5) {//逆運動学2~5
    /*2:上向きモード
      3:水平前向きモード
      4:下向きモード
      5:手先関節自由移動モード
    */
        // x軸
        if (abs(leftjoypad1.data) == 1) {
            arm.data[1] = -leftjoypad1.data;
        } else {
            arm.data[1] = 0;
        }
        // y軸
        if (abs(leftjoypad2.data) == 1) {
            arm.data[2] = leftjoypad2.data;
        } else {
            arm.data[2] = 0;
        }
        // z軸
        if (abs(rightjoypad2.data) == 1) {
            arm.data[3] = rightjoypad2.data;
        } else {
            arm.data[3] = 0;
        }
        // 手首上下
        if (mode == 5) {
            if (up.data) {
                arm.data[4] = -1;
            } else if (down.data) {
                arm.data[4] = 1;
            } else {
                arm.data[4] = 0;
            }
        } else {
            arm.data[4] = 0;
        }
        // 手首左右
        if (mode == 3){
            arm.data[5] = 0;
        }else {
            if (left.data) {
                arm.data[5] = 1;
            } else if (right.data) {
                arm.data[5] = -1;
            } else {
                arm.data[5] = 0;
            }
        }
        //　回転
        if (L1.data) {
            arm.data[6] = -1;
        } else if (R1.data) {
            arm.data[6] = 1;
        } else {
            arm.data[6] = 0;
        }
        //　開閉
        if (L2.data) {
            arm.data[7] = -1;
        } else if (R2.data) {
            arm.data[7] = 1;
        } else {
            arm.data[7] = 0;
        }
    } else if (mode == 6) {
        //　根本
        if (abs(leftjoypad1.data) == 1) {
            arm.data[1] = -leftjoypad1.data;
        } else {
            arm.data[1] = 0;
        }
        // 肩
        if (abs(leftjoypad2.data) == 1) {
            arm.data[2] = leftjoypad2.data;
        } else {
            arm.data[2] = 0;
        }
        // 肘
        if (abs(rightjoypad2.data) == 1) {
            arm.data[3] = rightjoypad2.data;
        } else {
            arm.data[3] = 0;
        }

        // 手首上下
        if (up.data) {
            arm.data[4] = 1;
        } else if (down.data) {
            arm.data[4] = -1;
        } else {
            arm.data[4] = 0;
        }
        // 手首左右
        if (left.data) {
            arm.data[5] = 1;
        } else if (right.data) {
            arm.data[5] = -1;
        } else {
            arm.data[5] = 0;
        }
        //　回転
        if (L1.data) {
            arm.data[6] = -1;
        } else if (R1.data) {
            arm.data[6] = 1;
        } else {
            arm.data[6] = 0;
        }
        //　開閉
        if (L2.data) {
            arm.data[7] = -1;
        } else if (R2.data) {
            arm.data[7] = 1;
        } else {
            arm.data[7] = 0;
        }
    } else {
        InitArmData();
    }

    arm.data[0] = mode;
}

void Operator::Callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    InputJoy(joy_msg);

    // mode setting
    if (seleCt.data) {
        allStop = true;
        if (rightjoypad2.data == 1) {
            if (mode > 0) {
                mode--;
            }
            ros::Duration(0.4).sleep();
        } else if (rightjoypad2.data == -1) {
            if (mode < 6) {
                mode++;
            }
            ros::Duration(0.4).sleep();
        }
    } else {
        allStop = false;
    }

    if (allStop) {
        InitArmData();
        InitMaxonData();
    } else {
        switch (mode) {
            case 0:
                InitArmData();
                InitMaxonData();
                break;
            case 1:
                MaxonOpe();
                break;
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
                ArmOpe();
                break;
            default:
                break;
        }
    }

    ROS_INFO("mode:%d", mode);

    text.text = menu[mode];

    // usleep(1000);
    arm_pub.publish(arm);
    maxon_pub.publish(maxon_vel);
    text_pub.publish(text);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Operator");

    Operator ope;

    ros::Rate loop_rate(60);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}