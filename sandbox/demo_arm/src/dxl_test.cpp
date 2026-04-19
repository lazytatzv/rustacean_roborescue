#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <string>

#include "arm_ik.h"
#include "rodep_msgs/Arm_Current.h"
#include "rodep_msgs/Arm_Target.h"
#include "rodep_msgs/Emergency_State.h"
#include "rodep_msgs/Mode_Interrupt.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"

#define MOTOR_NUM 2
#define FIRST_MOTOR_NUM 12

// Read Handler id
const uint8_t present_position_handler_id = 0;
const uint8_t present_current_id = 1;
const uint8_t hardware_error_handler_id = 2;
const uint8_t indirect_read_handler_id = 3;

// Write Handler id
const uint8_t goal_velocity_handler_id = 0;
const uint8_t goal_position_handler_id = 1;

//
const uint16_t READ_INDIRECT_MEMORY_FIRST = 168;
const uint16_t READ_INDIRECT_MEMORY_DATA_FIRST = 224;
const uint16_t WRITE_INDIRECT_MEMORY_FIRST = 578;  // とりあえず未使用

uint16_t READ_INDIRECT_MEMORY_SIZE = 0;

DynamixelWorkbench dxl_wb;
uint8_t dxl_id[MOTOR_NUM];

struct MOTOR_INFORMATION
{
  int32_t position_value;
  int32_t angle_value;
  float angle_rad;
  float current;
  uint8_t hardware_error;

  float current_percentage;
  //    MOTOR_MODE mode;

  void calculate()
  {
    angle_value = position_value - reset_pos;
    angle_rad = angle_value / gear_ratio / 2048.0;
    current_percentage = current / current_limit * 100;
  }

  void init(int _reset_pos, float _gear_ratio)
  {
    reset_pos = _reset_pos;
    gear_ratio = _gear_ratio;
  }

  int32_t get_current_limit() { return current_limit / 2.69; }

 private:
  int32_t reset_pos;
  float gear_ratio;

  const float current_limit = 1024 * 2.69;
  // Current [mA] = Value * CurrentScalingFactor [mA]
  // Value : 0~2047, CurrentScalingFactor:2.69mA

  // 実装予定
  //        float max_rad,min_rad;
};

MOTOR_INFORMATION motor_present_info[MOTOR_NUM];

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_controller");

  const char *log;
  bool result;

  std::string port_name = "/dev/ttyUSB0";
  // int baud_rate = 57600;
  int baud_rate = 1000000;
  uint16_t model_number = 0;

  XmlRpc::XmlRpcValue my_list;
  ros::NodeHandle nhh("~");

  if (nhh.hasParam("/arm"))
  {
    nhh.getParam("/arm", my_list);
    port_name = static_cast<std::string>(my_list["port"]);
    baud_rate = my_list["baudrate"];
    ROS_INFO("port_name %s %d", port_name.c_str(), baud_rate);
  }
  else
  {
    ROS_ERROR("can not load params, default is used.");
  }

  for (int i = 0; i < MOTOR_NUM; i++)
  {
    dxl_id[i] = FIRST_MOTOR_NUM + i;
  }

  result = dxl_wb.init(port_name.c_str(), baud_rate, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    ROS_ERROR("Failed to init");
  }
  else
  {
    ROS_INFO("Succeed to init(%d)", baud_rate);
  }

  for (int i = 0; i < MOTOR_NUM; i++)
  {
    result = dxl_wb.ping(dxl_id[i], &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to ping id:%d", dxl_id[i]);
      while (!result && ros::ok())
      {
        result = dxl_wb.reboot(dxl_id[i], &log);
      }
    }
    else
    {
      ROS_INFO("Succeed to ping");
      ROS_INFO("id : %d, model_number : %d", dxl_id[i], model_number);
    }
  }
  // Indirect Memoryの設定
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    result = dxl_wb.torqueOff(dxl_id[i], &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to torque off id:%d", dxl_id[i]);
    }

    //[Read]
    // Present_Position int32 128~131
    // Present_Current int16 126~127
    // Hardware_Error_Status uint8 70

    std::vector<uint16_t> read_memory_map_vector;
    read_memory_map_vector.insert(read_memory_map_vector.end(),
                                  {132, 133, 134, 135});  // Present_Position int32 128~131
    read_memory_map_vector.insert(read_memory_map_vector.end(),
                                  {126, 127});  // Present_Current int16 126~127
    read_memory_map_vector.insert(read_memory_map_vector.end(),
                                  {70});  // Hardware_Error_Status uint8 70

    int read_memory_map_size = read_memory_map_vector.size();
    READ_INDIRECT_MEMORY_SIZE = read_memory_map_size * 2;

    uint8_t read_memory_map_array[READ_INDIRECT_MEMORY_SIZE];

    for (int j = 0; j < READ_INDIRECT_MEMORY_SIZE; j += 2)
    {
      read_memory_map_array[j] = read_memory_map_vector.at(j / 2) & 0xFF;
      read_memory_map_array[j + 1] = read_memory_map_vector.at(j / 2) >> 8;
    }

    result = dxl_wb.writeRegister(dxl_id[i], READ_INDIRECT_MEMORY_FIRST, READ_INDIRECT_MEMORY_SIZE,
                                  read_memory_map_array, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to write indirect memory id:%d", dxl_id[i]);
    }
    else
    {
      ROS_INFO("Succeed to set read indirect address");
      // ROS_INFO("id : %d, indirect %d to %d", dxl_id[i], READ_INDIRECT_MEMORY_FIRST ,
      // read_memory_map_vector.at(j / 2));
    }

    result = dxl_wb.torqueOn(dxl_id[i], &log);

    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to torque on id:%d", dxl_id[i]);
    }
  }

  uint8_t read_handler_num = dxl_wb.getTheNumberOfSyncReadHandler();
  uint8_t write_handler_num = dxl_wb.getTheNumberOfSyncWriteHandler();

  ROS_INFO("readHnadlerNum: %d, writeHandlerNum: %d", read_handler_num, write_handler_num);

  if (read_handler_num == 0)
  {
    result = dxl_wb.addSyncReadHandler(dxl_id[0], "Present_Position", &log);
    result = dxl_wb.addSyncReadHandler(dxl_id[0], "Present_Current", &log);
    result = dxl_wb.addSyncReadHandler(dxl_id[0], "Hardware_Error_Status", &log);
    result =
        dxl_wb.addSyncReadHandler(READ_INDIRECT_MEMORY_DATA_FIRST, READ_INDIRECT_MEMORY_SIZE, &log);
  }
  if (write_handler_num == 0)
  {
    result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Velocity", &log);
    result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
  }

  for (int i = 0; i < MOTOR_NUM; i++)
  {
    result = dxl_wb.wheelMode(dxl_id[i], 0, &log);
    if (!result)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to change wheel mode");
    }
    else
    {
      ROS_INFO("Succeed Set WheelMode\n");
      dxl_wb.goalVelocity(dxl_id[i], (int32_t)0);
    }
  }

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    ros::spinOnce();
    result = dxl_wb.syncRead(indirect_read_handler_id, &log);

    if (!result)
    {
      ROS_ERROR("%s", log);
    }
    else
    {
      ROS_INFO("%s", log);
    }

    //[Read]
    // Present_Position int32
    // Present_Current int16
    // Hardware_Error_Status uint8

    {
      uint8_t memory_data[4 * MOTOR_NUM];  // 数は適当

      result =
          dxl_wb.getSyncReadData(indirect_read_handler_id, dxl_id, MOTOR_NUM,
                                 READ_INDIRECT_MEMORY_DATA_FIRST, 4, (int32_t *)memory_data, &log);
      if (!result)
      {
        ROS_ERROR("%s", log);
      }
      else
      {
        ROS_INFO("%s", log);
      }

      for (int i = 0; i < MOTOR_NUM; i++)
      {
        const int index = i * 4;
        motor_present_info[i].position_value =
            (int32_t)(memory_data[3 + index] << 24 | memory_data[2 + index] << 16 |
                      memory_data[1 + index] << 8 | memory_data[0 + index]);
      }
    }

    {
      int32_t memory_data[MOTOR_NUM];  // 数は適当
      result = dxl_wb.getSyncReadData(indirect_read_handler_id, dxl_id, MOTOR_NUM,
                                      READ_INDIRECT_MEMORY_DATA_FIRST + 4, 2, memory_data, &log);

      if (!result)
      {
        ROS_ERROR("%s", log);
      }
      else
      {
        ROS_INFO("%s", log);
      }

      for (int i = 0; i < MOTOR_NUM; i++)
      {
        const int index = i * 2;
        motor_present_info[i].current = (int16_t)memory_data[i] * 2.69;
      }
    }

    {
      int32_t memory_data[MOTOR_NUM];  // 数は適当
      result = dxl_wb.getSyncReadData(indirect_read_handler_id, dxl_id, MOTOR_NUM,
                                      READ_INDIRECT_MEMORY_DATA_FIRST + 6, 1, memory_data, &log);

      if (!result)
      {
        ROS_ERROR("%s", log);
      }
      else
      {
        ROS_INFO("%s", log);
      }

      for (int i = 0; i < MOTOR_NUM; i++)
      {
        motor_present_info[i].hardware_error = (int16_t)memory_data[i];
      }
    }

    for (int i = 0; i < MOTOR_NUM; i++)
    {
      motor_present_info[i].calculate();
      ROS_INFO("id: %d, current: %f[mA], posotion: %d, error_status: %d\n", dxl_id[i],
               motor_present_info[i].current, motor_present_info[i].position_value,
               motor_present_info[i].hardware_error);
    }

    loop_rate.sleep();
  }
  return 0;
}
