#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "rodep_msgs/Arm_Current.h"
#include "rodep_msgs/Emergency_State.h"

#include <cmath>
#include <string>

#include "arm_ik.h"
#include "rodep_msgs/Arm_Target.h"
#include "rodep_msgs/Mode_Interrupt.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <geometry_msgs/PointStamped.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <string>

#include "arm_ik.h"

#define UNDER_ARM_NUM 3
#define HAND_NUM 3
#define ARM_NUM UNDER_ARM_NUM + HAND_NUM
#define FINGER_NUM 2
#define MOTOR_NUM ARM_NUM + FINGER_NUM
#define FIRST_MOTOR_NUM 21

#define START_BUTTON ARM_NUM + 2

#define LEFT_FINGER 27
#define RIGHT_FINGER 28

#define VELOCITY_LIMIT 120

class ArmCtlNode {
private:
    DynamixelWorkbench dxl_wb;
    uint8_t dxl_id[MOTOR_NUM];

    ros::Rate* loop_rate;

    const char *log;
    bool result;

    enum MOTOR_MODE {
        WHEEL_MODE,
        CURRENT_BASED_POSITION_MODE,
        SETUP_FAILED
    };

    enum ARM_POSITION_STATE {
        HAND_EXPAND_POSITION,//アーム展開しました
        HAND_RESET_POSITION, //ハンドだけ収納しました
        INITIAL_POSITION,    //どちらも収納済みです
        UNKNOWN,
    };

    ARM_POSITION_STATE arm_position = UNKNOWN;

    struct MOTOR_INFORMATION{
        int32_t position_value;
        int32_t angle_value;
        float angle_rad;
        float current;
        uint8_t hardware_error;

        float current_percentage;
        MOTOR_MODE mode;

        void calculate(){
            angle_value = position_value - reset_pos;
            angle_rad = angle_value / gear_ratio / 2048.0 * M_PI;
            current_percentage = current/current_limit * 100;
        }

        void init(int _reset_pos, float _gear_ratio){
            reset_pos = _reset_pos;
            gear_ratio = _gear_ratio;
        }

        int32_t get_current_limit(){
            return current_limit / 2.69;
        }
    private:
        int32_t reset_pos;
        float gear_ratio;

        const float current_limit = 1024 * 2.69;
        //Current [mA] = Value * CurrentScalingFactor [mA]
        //Value : 0~2047, CurrentScalingFactor:2.69mA

        //実装予定
//        float max_rad,min_rad;
    };

    MOTOR_INFORMATION motor_present_info[MOTOR_NUM];

    int32_t reset_pos[MOTOR_NUM];//初回起動時の収納状態の角度保存変数 初回のみ変更可能
    //goal_pos_to_velで多く使用されてる

    int32_t finger_pos[FINGER_NUM];

    ros::NodeHandle nh;
    ros::Subscriber sub_msgs, emergency_sub;//from Operator
    ros::Publisher state_pub, mode_int_pub; //JointState Publisher用
    ros::Publisher current_pub;             //電流デバッグ用

    // Read Handler id
    const uint8_t present_position_handler_id = 0;
    const uint8_t present_current_id = 1;
    const uint8_t hardware_error_handler_id = 2;
    const uint8_t indirect_read_handler_id = 3;
    const uint8_t present_voltage_handler_id = 4;

    // Write Handler id
    const uint8_t goal_velocity_handler_id = 0;
    const uint8_t goal_position_handler_id = 1;

    //
    const uint16_t READ_INDIRECT_MEMORY_FIRST = 168;
    const uint16_t READ_INDIRECT_MEMORY_DATA_FIRST = 224;
    const uint16_t WRITE_INDIRECT_MEMORY_FIRST = 578; //とりあえず未使用

    uint16_t READ_INDIRECT_MEMORY_SIZE = 0;

    const int32_t motor_limit[MOTOR_NUM] = {800 * 4, 900 * 10, 1500 * 10, 4096, 1300, 2048, 900, 900};
    const int32_t MOTOR_LIMIT[MOTOR_NUM][2] = {{-800 * 4, 800*4}, {-900 * 10, 900*10}, {-1500 * 10, 1500 * 10}, {-4000, 4500}, {-1100, 1100}, {-2048, 1500}, {-50,900}, {-900, 50},};
    //motorの限界回転角
    const float arm_gear_ratio[MOTOR_NUM] = {4, 10, 10, 1, 1, 1, 1, 1};
    //Value to Rad に用いるギア比


    const float arm_lengths[5] = {0, 66.5, 458.9, 500, 0};

    ArmMock arm_mock;
    ArmSolver arm_solver;
    geometry_msgs::Point target_point;
    Angle4D init_angles;      //コンストラクタ実行用、実質使われてない？
    float mode2_angle4 = M_PI;//mode2のときワールド座標に対して手先の角度を指定する変数.
    float mode3_angle4 = 0;

    int32_t send_data_value[MOTOR_NUM];     //最終的に送信するデータ、0~4096までのデータ

    bool error_flag = false;

    bool Emergency_flag = false;

    int32_t ex_mode = 0;// mode切り替え検知用

    rodep_msgs::Arm_Target stored_msg;

    tf2_ros::TransformBroadcaster tf_broadcaster;

    void InitDynamixel();                                             // dynamixel workbenchの初期化処理
    void SetupSyncHandler();                                          // sync handlerのセットアップ
    void SetCurrentBasedPositionMode(int motor_index, bool save_mode);// motorごとにCurrentBasedPositionModeにセット
    void SetWheelMode(int motor_index, bool save_mode);               // motorごとにWheelModeにセット
    void SetNormalMode();                                             // 基本wheel mode,指先 current based position mode
    void SetInverseMode();                                            // 全部currentbasedPositionModeにする
    void SetPreviousMode(int motor_index);  
    void SetupPositionPIDGain(int motor_index);
    void SetupIndirectAddress(int motor_index);


    void SetResetPos();//初回起動時の収納状態の角度を保存

    void MoveHandOpenPos();// handを前に突き出した位置にする

    void MoveFingerResetPos();//指を閉じて初期位置に
    void MoveArmResetPos();   // Armを閉じた位置に収納
    void MoveHandResetPos();  // handを閉じた位置に収納

    void SetupArmSolver();// Arm Solverに現在の関節角度を渡す
    Angle4D CalcInversePosition(const geometry_msgs::Vector3 target_addition);

    void Callback(const rodep_msgs::Arm_TargetConstPtr &msg);// OperatorからのCallback
    void Emergency_Callback(const rodep_msgs::Emergency_State &msg);

    void Send_TF_Target(geometry_msgs::Point target_point);//逆運動学の目標地点をpublishする関数

    double value_to_rad(int32_t value, int id) {
        return value / arm_gear_ratio[id] / 2048;
    }

    void ModeRequest(int mode) {
        rodep_msgs::Mode_Interrupt mode_req_data;
        mode_req_data.set_mode = mode;//順運動モード
        mode_req_data.request_node = "arm_controller";
        mode_int_pub.publish(mode_req_data);
    }

public:
    ArmCtlNode(int looprate);
    ~ArmCtlNode(){};

    void GetAngle(int32_t angles[]);

    void SendState();
    void SendState(int32_t angles[]);

    void Reboot_Error_Motor();// ErrorStatusがあるモータのみReboot
    void StopAllMotor();      // 全モータストップ
    void TorqueOffAllMotor(); // 全モータートルクオフ
    void update();            // 定期的に回されるべき処理
    void Publish_Current();   // 電流情報をpublish

    void GetPresentInfomation();

    void Get_Current();
    void GetPresentPosition();
    void Check_Error();

    bool Error_Flag() { return error_flag; }
    bool Emergency_Flag() { return Emergency_flag; }
};

#endif// ARM_CONTROLLER_HPP