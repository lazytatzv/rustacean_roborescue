#include "arm_controller.hpp"
#include <cstdlib>
#include <iostream>
#include <string>

/*
[ INFO] [1665189935.095116709]: id : 21, model name : XM540-W270
[ INFO] [1665189935.095125191]: id : 22, model name : XM540-W270
[ INFO] [1665189935.095131832]: id : 23, model name : XM540-W270
[ INFO] [1665189935.095138432]: id : 24, model name : XM540-W270
[ INFO] [1665189935.095144942]: id : 25, model name : XM540-W270
[ INFO] [1665189935.095151452]: id : 26, model name : XM430-W350
[ INFO] [1665189935.095157899]: id : 27, model name : XM430-W350
[ INFO] [1665189935.095164052]: id : 28, model name : XM430-W350
*/

void goal_pos_to_vel(int32_t goal_pos[], int32_t pos[], float result_vel[], int size) {
    int overPos = 5;
    const float kp = 0.2;
    for (int i = 0; i < size; i++) {
        if (abs(goal_pos[i] - pos[i]) > overPos) {
            result_vel[i] = kp * (goal_pos[i] - pos[i]);
            if (result_vel[i] > VELOCITY_LIMIT) {
                result_vel[i] = VELOCITY_LIMIT;
            } else if (result_vel[i] < -VELOCITY_LIMIT) {
                result_vel[i] = -VELOCITY_LIMIT;
            }
        } else {
            result_vel[i] = 0;
        }
    }
}

bool is_zero_array(float array[], int size) {
    for (int i = 0; i < size; i++) {
        if (array[i] != 0) {
            return false;
        }
    }
    return true;
}

ArmCtlNode::ArmCtlNode(int looprate)
    : arm_mock(arm_lengths), arm_solver(arm_lengths) {
    InitDynamixel();
    SetupSyncHandler();

    loop_rate = new ros::Rate(looprate);

    emergency_sub = nh.subscribe("emergency_state", 1, &ArmCtlNode::Emergency_Callback, this);
    sub_msgs = nh.subscribe("arm", 1, &ArmCtlNode::Callback, this);
    state_pub = nh.advertise<std_msgs::Float64MultiArray>("arm_state", 1);

    arm_mock.setAngle(init_angles);
    target_point = arm_mock.getTargetPoint();//現在の指先座標を計算

    current_pub = nh.advertise<rodep_msgs::Arm_Current>("arm_current", 1);
    mode_int_pub = nh.advertise<rodep_msgs::Mode_Interrupt>("request_mode", 1);

    SetNormalMode();
    SetResetPos();
    MoveFingerResetPos();
    MoveHandResetPos();
}

void ArmCtlNode::InitDynamixel() {
    std::string port_name = "/dev/ttyUSB2";
    // int baud_rate = 57600;
    int baud_rate = 1000000;
    uint16_t model_number = 0;

    XmlRpc::XmlRpcValue my_list;
    ros::NodeHandle nhh("~");

    if (nhh.hasParam("/arm")) {
        nhh.getParam("/arm", my_list);
        port_name = static_cast<std::string>(my_list["port"]);
        baud_rate = my_list["baudrate"];
        ROS_INFO("port_name %s %d", port_name.c_str(), baud_rate);
    } else {
        ROS_ERROR("can not load params, default is used.");
    }

    for (int i = 0; i < MOTOR_NUM; i++) {
        dxl_id[i] = FIRST_MOTOR_NUM + i;
    }

    result = dxl_wb.init(port_name.c_str(), baud_rate, &log);
    if (result == false) {
        ROS_ERROR("%s", log);
        ROS_ERROR("Failed to init");
    } else {
        ROS_INFO("Succeed to init(%d)", baud_rate);
    }

    for (int i = 0; i < MOTOR_NUM; i++) {
        result = dxl_wb.ping(dxl_id[i], &model_number, &log);
        if (result == false) {
            ROS_ERROR("%s", log);
            ROS_ERROR("Failed to ping id:%d", dxl_id[i]);
            while (!result && ros::ok()) {
                ROS_ERROR("Can not find id: %d",dxl_id[i]);
                result = dxl_wb.reboot(dxl_id[i], &log);
                sleep(1);
                result = dxl_wb.ping(dxl_id[i], &model_number, &log);
            }
            ROS_INFO("Succeed to ping");
            ROS_INFO("id : %d, model_number : %d", dxl_id[i], model_number);
        } else {
            ROS_INFO("Succeed to ping");
            ROS_INFO("id : %d, model_number : %d", dxl_id[i], model_number);
        }
    }
    // Indirect Memoryの設定
    for (int i = 0; i < MOTOR_NUM; i++) {
        SetupPositionPIDGain(i);
        SetupIndirectAddress(i);
    }
}

void ArmCtlNode::SetupPositionPIDGain(int motor_index){
    result = dxl_wb.itemWrite(dxl_id[motor_index], "Position_I_Gain", 800, &log);
    if (result == false) {
        ROS_ERROR("%s", log);
        ROS_ERROR("Failed to torque off id:%d", dxl_id[motor_index]);
    }
}

void ArmCtlNode::SetupIndirectAddress(int motor_index){

    result = dxl_wb.torqueOff(dxl_id[motor_index], &log);
    if (result == false) {
        ROS_ERROR("%s", log);
        ROS_ERROR("Failed to torque off id:%d", dxl_id[motor_index]);
    }

    //[Read]
    //Present_Position int32 128~131
    //Present_Current int16 126~127
    //Hardware_Error_Status uint8 70

    std::vector<uint16_t> read_memory_map_vector;
    read_memory_map_vector.insert(read_memory_map_vector.end(), {132, 133, 134, 135});//Present_Position int32 128~131
    read_memory_map_vector.insert(read_memory_map_vector.end(), {126, 127});          //Present_Current int16 126~127
    read_memory_map_vector.insert(read_memory_map_vector.end(), {70});                //Hardware_Error_Status uint8 70

    int read_memory_map_size = read_memory_map_vector.size();
    READ_INDIRECT_MEMORY_SIZE = read_memory_map_size * 2;

    uint8_t read_memory_map_array[READ_INDIRECT_MEMORY_SIZE];

    for (int j = 0; j < READ_INDIRECT_MEMORY_SIZE; j += 2) {
        read_memory_map_array[j] = read_memory_map_vector.at(j / 2) & 0xFF;
        read_memory_map_array[j+1] = read_memory_map_vector.at(j / 2) >> 8;
    }

    result = dxl_wb.writeRegister(dxl_id[motor_index], READ_INDIRECT_MEMORY_FIRST, READ_INDIRECT_MEMORY_SIZE, read_memory_map_array, &log);
    if (result == false) {
        ROS_ERROR("%s", log);
        ROS_ERROR("Failed to write indirect memory id:%d", dxl_id[motor_index]);
    } else {
        ROS_INFO("Succeed to set read indirect address");
        // ROS_INFO("id : %d, indirect %d to %d", dxl_id[i], READ_INDIRECT_MEMORY_FIRST , read_memory_map_vector.at(j / 2));
    }

    result = dxl_wb.torqueOn(dxl_id[motor_index],&log);

    if (result == false) {
        ROS_ERROR("%s", log);
        ROS_ERROR("Failed to torque on id:%d", dxl_id[motor_index]);
    }
}

void ArmCtlNode::SetupSyncHandler() {
    uint8_t read_handler_num = dxl_wb.getTheNumberOfSyncReadHandler();
    uint8_t write_handler_num = dxl_wb.getTheNumberOfSyncWriteHandler();

    ROS_INFO("readHnadlerNum: %d, writeHandlerNum: %d", read_handler_num, write_handler_num);

    if (read_handler_num == 0) {
        result = dxl_wb.addSyncReadHandler(dxl_id[0], "Present_Position", &log);
        result = dxl_wb.addSyncReadHandler(dxl_id[0], "Present_Current", &log);
        result = dxl_wb.addSyncReadHandler(dxl_id[0], "Hardware_Error_Status", &log);
        result = dxl_wb.addSyncReadHandler(READ_INDIRECT_MEMORY_DATA_FIRST, READ_INDIRECT_MEMORY_SIZE, &log);
        result = dxl_wb.addSyncReadHandler(dxl_id[0], "Present_Input_Voltage", &log);
    }
    if (write_handler_num == 0) {
        result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Velocity", &log);
        result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
    }
}

void ArmCtlNode::SetCurrentBasedPositionMode(int motor_index, bool save_mode) {
    result = dxl_wb.currentBasedPositionMode(dxl_id[motor_index], motor_present_info[motor_index].get_current_limit(), &log);

    if (!result) {
        ROS_ERROR("%s", log);
        ROS_ERROR("Failed to change currentBasedPositionMode");
        motor_present_info[motor_index].mode = SETUP_FAILED;
    } else {
        ROS_INFO("Succeed Set currentBasedPositionMode\n");
        if (save_mode) motor_present_info[motor_index].mode = CURRENT_BASED_POSITION_MODE;
    }
}

void ArmCtlNode::SetWheelMode(int motor_index, bool save_mode) {
    result = dxl_wb.wheelMode(dxl_id[motor_index], 0, &log);
    if (!result) {
        ROS_ERROR("%s", log);
        ROS_ERROR("Failed to change wheel mode");
        motor_present_info[motor_index].mode = SETUP_FAILED;
    } else {
        ROS_INFO("Succeed Set WheelMode\n");
        usleep(10000);
        dxl_wb.goalVelocity(dxl_id[motor_index], (int32_t) 0);
        if (save_mode) motor_present_info[motor_index].mode = WHEEL_MODE;
    }
}

void ArmCtlNode::SetNormalMode() {
    bool save_mode = true;
    for (int i = 0; i < ARM_NUM; i++) {
        SetWheelMode(i, save_mode);
    }
    for (int i = ARM_NUM; i < ARM_NUM + FINGER_NUM; i++) {
        SetCurrentBasedPositionMode(i, save_mode);
    }
    sleep(1);
}
void ArmCtlNode::SetInverseMode() {
    bool save_mode = true;
    for (int i = 0; i < ARM_NUM + FINGER_NUM; i++) {
        SetCurrentBasedPositionMode(i, save_mode);
    }
    sleep(1);
}

void ArmCtlNode::SetPreviousMode(int motor_index) {
    bool save_mode = false;
    if (motor_present_info[motor_index].mode == WHEEL_MODE) {
        SetWheelMode(motor_index, save_mode);
    } else if (motor_present_info[motor_index].mode == CURRENT_BASED_POSITION_MODE) {
        SetCurrentBasedPositionMode(motor_index, save_mode);
    }
}

void ArmCtlNode::SetupArmSolver() {
    //ポジション計算
    Angle4D temp_angle4D;
    temp_angle4D.angle1 = (float) motor_present_info[0].angle_rad;
    temp_angle4D.angle2 = (float) motor_present_info[1].angle_rad;
    temp_angle4D.angle3 = (float) motor_present_info[2].angle_rad;
    temp_angle4D.angle4 = (float) motor_present_info[3].angle_rad;

    arm_mock.setAngle(temp_angle4D);
    target_point = arm_mock.getTargetPoint();

    ROS_INFO("[SetupArmSolver] Current Finger Position x: %lf, y: %lf, z: %lf", target_point.x, target_point.y, target_point.z);

    //インクリメントさせて動かす、指先と手首の現在角度を保存
    send_data_value[4] = motor_present_info[4].position_value;
    send_data_value[5] = motor_present_info[5].position_value;
    send_data_value[6] = motor_present_info[6].position_value;
    send_data_value[7] = motor_present_info[7].position_value;
}


//初期位置を設定(初期位置は起動したときの位置　2048ではない)
void ArmCtlNode::SetResetPos() {
    GetAngle(reset_pos);

    /* reset_pos[0] = 2047;// id:21
    reset_pos[1] = 2259;// id:22
    reset_pos[2] = 3167;// id:23
    reset_pos[3] = 2024;// id:24
    reset_pos[4] = 2030;// id:25
    reset_pos[5] = 1027;// id:26 */

    for (int i = 0; i < MOTOR_NUM; i++) {
        ROS_INFO("ID%d : %d", dxl_id[i], reset_pos[i]);
        motor_present_info[i].init(reset_pos[i], arm_gear_ratio[i]);
    }

}

void ArmCtlNode::MoveHandOpenPos() {
    ROS_INFO("Move Hand Open Pos...");
    int32_t goal_pos[HAND_NUM] = {0, 2048, 1024}, angles[HAND_NUM], vel[HAND_NUM] = {0};
    float result_vel[HAND_NUM] = {0};
    bool init_position_reached = false;

    result = dxl_wb.syncRead(present_position_handler_id, &log);
    result = dxl_wb.getSyncReadData(present_position_handler_id, &dxl_id[UNDER_ARM_NUM], HAND_NUM, angles, &log);
    if (!result) {
        ROS_ERROR("%s", log);
        return;
    }

    while (!init_position_reached && ros::ok()) {
        goal_pos_to_vel(goal_pos, angles, result_vel, HAND_NUM);

        ROS_INFO("[ARM_CONROLELR] result_vel : %lf, %lf, %lf", result_vel[0], result_vel[1], result_vel[2]);
        ROS_INFO("[ARM_CONROLELR] current_pos : %d, %d, %d", angles[0], angles[1], angles[2]);
        ROS_INFO("[ARM_CONROLELR] goal_pos : %d, %d, %d", goal_pos[0], goal_pos[1], goal_pos[2]);

        vel[0] = result_vel[0];
        if (angles[0] > 1152) {// 手首がアームの中にいる場合は回転等してはいけない
            vel[1] = 0;
            vel[2] = 0;
        } else {
            vel[1] = result_vel[1];
            vel[2] = result_vel[2];
        }

        if (is_zero_array(result_vel, HAND_NUM)) {
            init_position_reached = true;
        }

        dxl_wb.syncWrite(goal_velocity_handler_id, &dxl_id[UNDER_ARM_NUM], HAND_NUM, vel, 1, &log);

        result = dxl_wb.syncRead(present_position_handler_id, &log);
        result = dxl_wb.getSyncReadData(present_position_handler_id, &dxl_id[UNDER_ARM_NUM], HAND_NUM, angles, &log);

        GetPresentInfomation();//関数内のSyncReadを統合したら消す
        SendState();

        loop_rate->sleep();
        ros::spinOnce();
        if (Emergency_Flag()) {
            TorqueOffAllMotor();
            break;
        }
    }

    ROS_INFO("Done Hand Open Pos.");
}

void ArmCtlNode::MoveFingerResetPos() {
    ROS_INFO("Set Finger Init Pos...");
    finger_pos[0] = 20;
    finger_pos[1] = -20;
    send_data_value[ARM_NUM] = (int32_t) finger_pos[0];
    send_data_value[ARM_NUM + 1] = (int32_t) finger_pos[1];
    dxl_wb.syncWrite(goal_position_handler_id, &dxl_id[ARM_NUM], FINGER_NUM, finger_pos, 1, &log);
    ROS_INFO("Done Set Finger Init Pos.");
}

void ArmCtlNode::MoveHandResetPos() {
    MoveFingerResetPos();

    ROS_INFO("Move Hand Reset Pos...");
    int32_t wrist_safe_angle = 1152;
    int32_t angles[HAND_NUM], vel[HAND_NUM] = {0}, wrist_angle[1] = {wrist_safe_angle};
    float result_vel[HAND_NUM] = {0};
    bool init_position_reached = false, init_wrist_position = false;

    result = dxl_wb.syncRead(present_position_handler_id, &log);
    if (!result) {
        ROS_ERROR("%s", log);
        return;
    }
    result = dxl_wb.getSyncReadData(present_position_handler_id, &dxl_id[UNDER_ARM_NUM], HAND_NUM, angles, &log);
    if (!result) {
        ROS_ERROR("%s", log);
        return;
    }

    goal_pos_to_vel(&reset_pos[UNDER_ARM_NUM], angles, result_vel, HAND_NUM);


    if (angles[0] > wrist_angle[0] && angles[0] < 1920 && (result_vel[1] != 0 || result_vel[2] != 0)) {
        while (!init_wrist_position && ros::ok()) {
            goal_pos_to_vel(wrist_angle, angles, result_vel, 1);

            // ROS_INFO("[ARM_CONROLELR] MoveHandResetPos loop1 result_vel : %lf, %lf, %lf", result_vel[0], result_vel[1], result_vel[2]);
            // ROS_INFO("[ARM_CONROLELR] MoveHandResetPos loop1 current_pos : %d, %d, %d", angles[0], angles[1], angles[2]);

            vel[0] = result_vel[0];

            if (is_zero_array(result_vel, 1)) {
                init_wrist_position = true;
            }

            dxl_wb.syncWrite(goal_velocity_handler_id, &dxl_id[UNDER_ARM_NUM], 1, vel, 1, &log);

            result = dxl_wb.syncRead(present_position_handler_id, &log);
            result = dxl_wb.getSyncReadData(present_position_handler_id, &dxl_id[UNDER_ARM_NUM], HAND_NUM, angles, &log);

            GetPresentInfomation();//関数内のSyncReadを統合したら消す
            SendState();

            loop_rate->sleep();
            ros::spinOnce();
            if (Emergency_Flag()) {
                TorqueOffAllMotor();
                break;
            }
        }
    }


    while (!init_position_reached && ros::ok()) {
        goal_pos_to_vel(&reset_pos[UNDER_ARM_NUM], angles, result_vel, HAND_NUM);

        // ROS_INFO("[ARM_CONROLELR] MoveHandResetPos loop2 result_vel : %lf, %lf, %lf", result_vel[0], result_vel[1], result_vel[2]);
        // ROS_INFO("[ARM_CONROLELR] MoveHandResetPos loop2 current_pos : %d, %d, %d", angles[0], angles[1], angles[2]);

        vel[0] = result_vel[0];
        vel[1] = result_vel[1];
        vel[2] = result_vel[2];


        if (/*angles[0] > wrist_safe_angle && */ (result_vel[1] != 0 || result_vel[2] != 0)) {
            //危険域で一回他の収束を待つ
            vel[0] = 0;
        }

        if (is_zero_array(result_vel, HAND_NUM)) {
            init_position_reached = true;
        }

        dxl_wb.syncWrite(goal_velocity_handler_id, &dxl_id[UNDER_ARM_NUM], HAND_NUM, vel, 1, &log);

        result = dxl_wb.syncRead(present_position_handler_id, &log);
        result = dxl_wb.getSyncReadData(present_position_handler_id, &dxl_id[UNDER_ARM_NUM], HAND_NUM, angles, &log);

        GetPresentInfomation();//関数内のSyncReadを統合したら消す
        SendState();

        loop_rate->sleep();
        ros::spinOnce();
        if (Emergency_Flag()) {
            TorqueOffAllMotor();
            break;
        }
    }

    ROS_INFO("Done Hand Reset Pos.");
}

void ArmCtlNode::MoveArmResetPos() {
    ROS_INFO("Move Arm Reset Pos...");
    int32_t angles[UNDER_ARM_NUM], vel[UNDER_ARM_NUM] = {0};
    float result_vel[UNDER_ARM_NUM] = {0};
    float zero_array[UNDER_ARM_NUM] = {0};
    bool init_position_reached = false;

    result = dxl_wb.syncRead(present_position_handler_id, &log);
    result = dxl_wb.getSyncReadData(present_position_handler_id, dxl_id, UNDER_ARM_NUM, angles, &log);
    if (!result) {
        ROS_ERROR("%s", log);
        return;
    }

    while (!init_position_reached && ros::ok()) {
        goal_pos_to_vel(reset_pos, angles, result_vel, UNDER_ARM_NUM);

        vel[0] = result_vel[0];
        vel[1] = result_vel[1];
        if (result_vel[1] != 0 && angles[2] < -2000) {
            vel[2] = 0;
        } else {
            vel[2] = result_vel[2];
        }

        if (is_zero_array(result_vel, HAND_NUM)) {
            init_position_reached = true;
        }

        dxl_wb.syncWrite(goal_velocity_handler_id, dxl_id, UNDER_ARM_NUM, vel, 1, &log);

        result = dxl_wb.syncRead(present_position_handler_id, &log);
        result = dxl_wb.getSyncReadData(present_position_handler_id, dxl_id, UNDER_ARM_NUM, angles, &log);
        GetPresentInfomation();//関数内のSyncReadを統合したら消す
        SendState();

        loop_rate->sleep();
        ros::spinOnce();
        if (Emergency_Flag()) {
            TorqueOffAllMotor();
            break;
        }
    }

    ROS_INFO("Done Arm Reset Pos.");
}

void ArmCtlNode::Check_Error() {
    int32_t error[MOTOR_NUM];
    result = dxl_wb.syncRead(hardware_error_handler_id, &log);
    if (!result) {
        ROS_ERROR("%s", log);
        return;
    }

    dxl_wb.getSyncReadData(hardware_error_handler_id, dxl_id, MOTOR_NUM, error, &log);

    error_flag = false;

    for (int i = 0; i < MOTOR_NUM; i++) {
        motor_present_info[i].hardware_error = (uint8_t)error[i];
        if(motor_present_info[i].hardware_error != 0){
            error_flag = true;
        }
    }
}


void ArmCtlNode::GetPresentPosition(){
    int32_t angle[MOTOR_NUM];
    GetAngle(angle);

    for (int i = 0; i < MOTOR_NUM; i++) {
        motor_present_info[i].position_value = angle[i];
        motor_present_info[i].calculate();
    }

}

void ArmCtlNode::GetAngle(int32_t angles[]) {
    result = dxl_wb.syncRead(present_position_handler_id, &log);
    if (!result) {
        ROS_ERROR("%s", log);
        return;
    }
    result = dxl_wb.getSyncReadData(present_position_handler_id, dxl_id, MOTOR_NUM, angles, &log);

    if (!result) {
        ROS_ERROR("%s", log);
        return;
    }
}

void ArmCtlNode::Get_Current(){

    result = dxl_wb.syncRead(present_current_id, &log);
    if (!result) {
        ROS_ERROR("%s", log);
        return;
    } else {
        //        ROS_INFO("%s", log);
    }

    int32_t current_value[MOTOR_NUM];
    dxl_wb.getSyncReadData(present_current_id, dxl_id, MOTOR_NUM, current_value, &log);

    for (int i = 0; i < MOTOR_NUM; i++) {
        motor_present_info[i].current = (int16_t) current_value[i] * 2.69;
        motor_present_info[i].calculate();
    }

    result = dxl_wb.syncRead(present_voltage_handler_id, &log);
    if (!result) {
        ROS_ERROR("%s", log);
        return;
    } else {
        //        ROS_INFO("%s", log);
    }

    int32_t voltage_value[MOTOR_NUM];
    dxl_wb.getSyncReadData(present_voltage_handler_id, dxl_id, MOTOR_NUM, voltage_value, &log);

    for (int i = 0; i < MOTOR_NUM; i++) {
        ROS_INFO("id: %d, voltage: %lf", dxl_id[i], (float)voltage_value[i] * 0.1);
    }
}

void ArmCtlNode::GetPresentInfomation() {
    result = dxl_wb.syncRead(indirect_read_handler_id, &log);

    if (!result) {
        ROS_ERROR("%s", log);
    } else {
        ROS_INFO("%s", log);
    }

    //[Read]
    //Present_Position int32
    //Present_Current int16
    //Hardware_Error_Status uint8
    
    //Present Positionの取得
    uint8_t memory_data[100];

    result = dxl_wb.getSyncReadData(indirect_read_handler_id, dxl_id, MOTOR_NUM,
                                    READ_INDIRECT_MEMORY_DATA_FIRST, 4, (int32_t *) memory_data, &log);
    if (!result) {
        ROS_ERROR("%s", log);
    } else {
        // ROS_INFO("%s", log);
    }

    for (int i = 0; i < MOTOR_NUM; i++) {
        const int index = i * 4;
        motor_present_info[i].position_value = (int32_t) (memory_data[3 + index] << 24 |
                                                            memory_data[2 + index] << 16 |
                                                            memory_data[1 + index] << 8 | memory_data[0 + index]);
    }
//Present Currentの取得
    int32_t memory_data_u32[100];
    result = dxl_wb.getSyncReadData(indirect_read_handler_id, dxl_id, MOTOR_NUM,
                                    READ_INDIRECT_MEMORY_DATA_FIRST + 4, 2, memory_data_u32, &log);

    if (!result) {
        ROS_ERROR("%s", log);
    } else {
        // ROS_INFO("%s", log);
    }

    for (int i = 0; i < MOTOR_NUM; i++) {
        const int index = i * 2;
        motor_present_info[i].current =
                (int16_t)memory_data_u32[i] * 2.69;
    }

//Hardware Errorの取得
    result = dxl_wb.getSyncReadData(indirect_read_handler_id, dxl_id, MOTOR_NUM,
                                    READ_INDIRECT_MEMORY_DATA_FIRST + 6, 1, memory_data_u32, &log);

    if (!result) {
        ROS_ERROR("%s", log);
    } else {
        // ROS_INFO("%s", log);
    }
    error_flag = false;
    for (int i = 0; i < MOTOR_NUM; i++) {
        motor_present_info[i].hardware_error = (uint8_t)memory_data_u32[i];
        if (motor_present_info[i].hardware_error!=0){
            error_flag = true;
        }
    }

    for (int i = 0; i < MOTOR_NUM; i++) {
        motor_present_info[i].calculate();
        ROS_INFO("id: %d, current: %f[mA], posotion: %d, error_status: %d\n", dxl_id[i], motor_present_info[i].current, motor_present_info[i].position_value, motor_present_info[i].hardware_error);
    }
}

void ArmCtlNode::Reboot_Error_Motor() {
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (motor_present_info[i].hardware_error == 0) continue;

        ROS_ERROR("ERROR id: %d, status: %d", dxl_id[i], motor_present_info[i].hardware_error);

   
        while(error_flag && ros::ok()){

            if(motor_present_info[i].hardware_error != 0b00100000 && motor_present_info[i].hardware_error != 0b00000001 ){
                ROS_ERROR("Critical Hardware Error Occur id: %d, status: %d", dxl_id[i], motor_present_info[i].hardware_error);
            }

            ROS_INFO("Rebooting...");

            result = dxl_wb.reboot(dxl_id[i], &log);

            if (!result) {
                ROS_ERROR("%s", log);
                return;
            } else {
                ROS_INFO("Succeed Reboot_Error_Motor\n");
                ROS_INFO("%s", log);
            }
            
            sleep(1);

            Check_Error();
        }

        SetupPositionPIDGain(i);
        SetupIndirectAddress(i);


        
        if (i == MOTOR_NUM - 1 || i == MOTOR_NUM - 2) {//指先のモータならそのまま続行
            SetPreviousMode(i);
        } else {           //それ以外のモータが死んだ場合、復旧時怖いためmode6にしてNormalModeにする
            ModeRequest(6);//Normal Mode
            GetPresentInfomation();
            if (!error_flag) {
                MoveFingerResetPos();
            }
            SetNormalMode();
        }
    }

    GetPresentInfomation();

    if (!error_flag) {
        MoveFingerResetPos();
    }
}

// rvizにアームの位置を送信
void ArmCtlNode::SendState() {
    std_msgs::Float64MultiArray state;
    state.data.resize(MOTOR_NUM);

    for (int i = 0; i < MOTOR_NUM; i++) {
        state.data[i] = (float) (motor_present_info[i].angle_value) * M_PI / 2048.0;
        // ROS_INFO("id: %d statedata: %f", FIRST_MOTOR_NUM + i, state.data[i]);
    }

    state_pub.publish(state);
}

void ArmCtlNode::SendState(int32_t angles[]) {
    std_msgs::Float64MultiArray state;
    state.data.resize(MOTOR_NUM);

    for (int i = 0; i < MOTOR_NUM; i++) {
        state.data[i] = (float) (angles[i] - reset_pos[i]) * M_PI / 2048.0;
    }
    state_pub.publish(state);
}

Angle4D ArmCtlNode::CalcInversePosition(const geometry_msgs::Vector3 target_addition) {
    Angle4D angles;

    target_point.x += target_addition.x;
    target_point.y += target_addition.y;
    target_point.z += target_addition.z;

    if (!arm_solver.solve(target_point, 0, angles)) {// 1_calc
        ROS_INFO("can not solve");
        ModeRequest(6);//Normal Mode
    }

    Send_TF_Target(target_point);

    return angles;
}

void ArmCtlNode::Callback(const rodep_msgs::Arm_TargetConstPtr &msg) { stored_msg = *msg; }

void ArmCtlNode::Emergency_Callback(const rodep_msgs::Emergency_State &msg) { Emergency_flag = msg.emergency_stop; }

void ArmCtlNode::Send_TF_Target(geometry_msgs::Point target_point) {
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "arm_base_link";// TODO:どこ基準ですか
    transformStamped.child_frame_id = "ik_target";
    transformStamped.transform.translation.x = -target_point.y / 1000;
    transformStamped.transform.translation.y = target_point.x / 1000;
    transformStamped.transform.translation.z = target_point.z / 1000;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf_broadcaster.sendTransform(transformStamped);
}

void ArmCtlNode::StopAllMotor() {
    int32_t vel[ARM_NUM] = {0};
    result = dxl_wb.syncWrite(goal_velocity_handler_id, dxl_id, ARM_NUM, vel, 1, &log);// dynamixelに送信

    if (!result) {
        ROS_ERROR("%s", log);
    } else {
        //        ROS_INFO("%s", log);
    }
    result = dxl_wb.syncWrite(goal_position_handler_id, &dxl_id[ARM_NUM], FINGER_NUM, finger_pos, 1, &log);

    if (!result) {
        ROS_ERROR("%s", log);
    } else {
        //        ROS_INFO("%s", log);
    }
}

void ArmCtlNode::TorqueOffAllMotor() {
    ROS_INFO("Torque Off All Motor...");
    for (int i = 0; i < MOTOR_NUM; i++) {
        result = dxl_wb.torqueOff(dxl_id[i], &log);
        if (!result) {
            ROS_ERROR("%s", log);
        } else {
            ROS_INFO("%s", log);
        }
    }
}

void ArmCtlNode::Publish_Current() {
    rodep_msgs::Arm_Current send_current_data;

    send_current_data.arm_current_value.resize(MOTOR_NUM);
    send_current_data.arm_current_percentage.resize(MOTOR_NUM);

    for (int i = 0; i < MOTOR_NUM; i++) {
        send_current_data.arm_current_value.at(i) = motor_present_info[i].current;
        send_current_data.arm_current_percentage.at(i) = motor_present_info[i].current_percentage;
    }
    current_pub.publish(send_current_data);
}

void ArmCtlNode::update() {

    SendState();

    if (Emergency_flag) {
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("[%s] Emergency_flag : true", node_name.c_str());
        //緊急停止の動作
        TorqueOffAllMotor();
        return;
    }

    if (stored_msg.mode != ex_mode) {
        if (stored_msg.mode == 6) {
            ROS_INFO("SetNormalMode");
            SetNormalMode();
        } else {
            ROS_INFO("SetInverseMode");
            SetupArmSolver();
            SetInverseMode();
        }
    }

    int mode = stored_msg.mode;


    switch (mode) {
        case 2:
        case 3:
        case 4:
        case 5: {
            Angle4D target_angles;

            float write_angle_rad[8];//モーター目標角度[rad]　アームの関節角度ではない！

            target_angles = CalcInversePosition(stored_msg.ik_target);

            write_angle_rad[0] = target_angles.angle1 * arm_gear_ratio[0];
            write_angle_rad[1] = target_angles.angle2 * arm_gear_ratio[1];
            write_angle_rad[2] = target_angles.angle3 * arm_gear_ratio[2];

            int32_t stored_send_data[MOTOR_NUM];
            for (int i = 0; i < MOTOR_NUM; i++) {
                stored_send_data[i] = send_data_value[i];
            }

            // 折り畳まれた状態がすべての関節角度 0
            // index:3 = 上下方向手首関節
            // index:4 = 左右方向手首関節

            switch (mode) {
                case 2://手首上下方向任意モード
                    //初期は水平方向
                    mode2_angle4 += (float) stored_msg.wrist_pitch / 40.0;
                    write_angle_rad[3] = target_angles.angle4 - mode2_angle4;
                    break;
                case 3:                                              //水平モード
                    mode3_angle4 += (float) stored_msg.wrist_pitch / 80.0;
                    write_angle_rad[3] = target_angles.angle4 - M_PI - mode3_angle4;//上下方向を水平に固定
                    break;
                case 4:                                                    //下向きモード
                    write_angle_rad[3] = target_angles.angle4 - 3 * M_PI_2;//上下方向を下に固定
                    break;
                case 5:                                                //上向きモード
                    write_angle_rad[3] = target_angles.angle4 - M_PI_2;//上下方向を上に固定
                    break;
                default:
                    break;
            }

            // 手先は相対司令
            if (ex_mode != mode) {
                send_data_value[5] = motor_present_info[5].position_value; //手首回転角
                stored_send_data[4] = motor_present_info[4].position_value;//手首左右角度
            } else {
                send_data_value[5] += stored_msg.wrist_roll * 5;//手首回転角
            }

            if (mode == 2 || mode==4 || mode==5) {
                // 任意・下・上のモードで手首左右角度の設定
                // 前回分の制御量に少し足す
                // 手先以外の制御量計算
                //                write_angle_rad[4] = -target_angles.angle1;
                for (int i = 0; i < 5; i++) {
                    send_data_value[i] = reset_pos[i] + (int32_t) ((write_angle_rad[i] / M_PI) * 2048);
                }
                send_data_value[4] = stored_send_data[4] - stored_msg.wrist_yaw * 5;//左右角度
            } else {
                // 水平モードの手首左右角度を平行に維持
                write_angle_rad[4] = -target_angles.angle1;
                // 手先以外の制御量計算
                for (int i = 0; i < 5; i++) {
                    send_data_value[i] = reset_pos[i] + (int32_t) ((write_angle_rad[i] / M_PI) * 2048);
                }
            }

            send_data_value[6] += stored_msg.glip_value * 3;
            send_data_value[7] -= stored_msg.glip_value * 3;

            //限界角判定
            bool maxrotflag = false;
            for (int i = 0; i < MOTOR_NUM; i++) {
                bool maxrotflag_temp = abs(send_data_value[i] - reset_pos[i]) > motor_limit[i];
                if (maxrotflag_temp) {
                    ROS_ERROR("[Arm_Controller] Max Rotate Angle!! ID:%d LIMIT:%lf[(abs)rad], Current : %lf[rad]", i, value_to_rad(motor_limit[i], i), value_to_rad(send_data_value[i] - reset_pos[i], i));
                    maxrotflag = true;
                }
            }
            if (maxrotflag) {
                for (int i = 0; i < MOTOR_NUM; i++) {
                    send_data_value[i] = stored_send_data[i];
                }
            }

            ROS_INFO("target_point---------");
            ROS_INFO("%f", target_point.x);
            ROS_INFO("%f", target_point.y);
            ROS_INFO("%f", target_point.z);
            ROS_INFO("-----------");

            dxl_wb.syncWrite(goal_position_handler_id, dxl_id, MOTOR_NUM, send_data_value, 1, &log);
            if (!result) {
                ROS_ERROR("%s", log);
            } else {
                //                ROS_INFO("%s", log);
            }
            break;
        }

        case 6: {
            for (int i = 0; i < ARM_NUM; i++) {
                send_data_value[i] = (int32_t) (stored_msg.arm_velocity[i] * 25);
            }

            for (int i = 0; i < MOTOR_NUM; i++) {
                if (motor_present_info[i].angle_value < MOTOR_LIMIT[i][0]) {
                    if (send_data_value[i] < 0) {
                        send_data_value[i] = 0;
                        ROS_ERROR("[Arm_Controller] Min Rotate Angle!! ID:%d LIMIT:%lf[(abs)rad], Current : %lf[rad]", i, value_to_rad(MOTOR_LIMIT[i][0], i), motor_present_info[i].angle_rad);
                    }
                } else if (motor_present_info[i].angle_value > MOTOR_LIMIT[i][1]) {
                    if (send_data_value[i] > 0) {
                        send_data_value[i] = 0;
                        ROS_ERROR("[Arm_Controller] Max Rotate Angle!! ID:%d LIMIT:%lf[(abs)rad], Current : %lf[rad]", i, value_to_rad(MOTOR_LIMIT[i][1], i), motor_present_info[i].angle_rad);
                    }
                }
            }

            float finger_pos_glip = stored_msg.glip_value;

            send_data_value[ARM_NUM] += (int32_t) (finger_pos_glip * 3);
            send_data_value[ARM_NUM + 1] -= (int32_t) (finger_pos_glip * 3);

            dxl_wb.syncWrite(goal_velocity_handler_id, dxl_id, ARM_NUM, send_data_value, 1, &log);// dynamixelに送信
            dxl_wb.syncWrite(goal_position_handler_id, &dxl_id[ARM_NUM], FINGER_NUM, &send_data_value[ARM_NUM], 1, &log);

            if (!result) {
                ROS_ERROR("%s", log);
            } else {
                //                ROS_INFO("%s", log);
            }
            break;
        }
        default:
            break;
    }

    int start_handle = stored_msg.start_button;

    if (start_handle == 1) {
        bool save_mode = false;
        for (int i = 0; i < ARM_NUM; i++) {
            SetWheelMode(i, save_mode);
        }
        std::string ex_pos, new_pos;

        switch (arm_position) {
            case UNKNOWN:
                ex_pos = "UNKNOWN";
            case INITIAL_POSITION:
                ex_pos = "INITIAL_POSITION";
                MoveHandOpenPos();
                arm_position = HAND_EXPAND_POSITION;
                new_pos = "HAND_EXPAND_POSITION";
                break;
            case HAND_EXPAND_POSITION:
                ex_pos = "HAND_EXPAND_POSITION";
                MoveHandResetPos();
                arm_position = HAND_RESET_POSITION;
                new_pos = "HAND_RESET_POSITION";
                break;
            case HAND_RESET_POSITION:
                ex_pos = "HAND_RESET_POSITION";
                MoveArmResetPos();
                arm_position = INITIAL_POSITION;
                new_pos = "INITIAL_POSITION";
                break;
            default:
                ex_pos = "UNKNOWN";
                arm_position = UNKNOWN;
                new_pos = "UNKNOWN";
        }

        ROS_INFO("[ARM_CONTROLLER] ex:%s new:%s", ex_pos.c_str(), new_pos.c_str());

        for (int i = 0; i < ARM_NUM; i++) {
            SetPreviousMode(i);
        }
    }

    ex_mode = mode;
}
