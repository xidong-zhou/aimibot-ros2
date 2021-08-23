#ifndef __MESSAGE_CALLBACK__H
#define __MESSAGE_CALLBACK__H
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "data_struct.hpp"
#include "std_msgs/msg/empty.hpp"
#include "mymsgs/msg/cliff.h"
#include "mymsgs/msg/controller_info.h"
#include "mymsgs/msg/core_sensors.h"
#include "mymsgs/msg/current.h"
#include "mymsgs/msg/dock_infra_red.h"
#include "mymsgs/msg/gp_input.h"
#include "mymsgs/msg/inertia.h"
#include "mymsgs/msg/three_axis_gyro.h"
#include "mymsgs/msg/unique_device_id.h"
#include "mymsgs/msg/led.h"
#include "mymsgs/msg/digital_output.h"
#include "mymsgs/msg/sound.h"
#include "mymsgs/msg/motor_power.h"
#include "mymsgs/msg/ultrasonic.h"

struct command_id
{
  uint8_t base_control=0x01;uint8_t base_control_lenght=0x04;
  uint8_t sound_control=0x03;uint8_t sound_control_lenght=0x03;
  uint8_t power_control=0x08;uint8_t power_control_lenght=0x02;
  uint8_t requestex_control=0x09;uint8_t requestex_control_lenght=0x02;
  uint8_t gpout_control=0x0c;uint8_t gpout_control_lenght=0x02;
};
void subscribeVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr msg);

#endif
