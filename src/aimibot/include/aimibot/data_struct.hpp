#ifndef __DATA_STRUCT__H
#define __DATA_STRUCT__H

#include "mymsgs/msg/cliff.hpp"
#include "mymsgs/msg/controller_info.hpp"
#include "mymsgs/msg/core_sensors.hpp"
#include "mymsgs/msg/current.hpp"
#include "mymsgs/msg/dock_infra_red.hpp"
#include "mymsgs/msg/gp_input.hpp"
#include "mymsgs/msg/inertia.hpp"
#include "mymsgs/msg/three_axis_gyro.hpp"
#include "mymsgs/msg/unique_device_id.hpp"
#include "mymsgs/msg/led.hpp"
#include "mymsgs/msg/digital_output.hpp"
#include "mymsgs/msg/sound.hpp"
#include "mymsgs/msg/motor_power.hpp"
#include "mymsgs/msg/ultrasonic.hpp"
#include "mymsgs/msg/hardware.hpp"
struct CoreSensors_data
{
    uint16_t time_stamp;
    uint8_t bumper;
    uint8_t wheel_drop;
    uint8_t cliff;
    uint16_t left_encoder;
    uint16_t right_encoder;
    char left_pwm;
    char right_pwm;
    uint8_t buttons;
    uint8_t charger;
    uint8_t battery;
    uint8_t over_current;
};
struct DockInfraRed_data
{
    uint8_t Docking[3];
};
struct Inertia_data 
{
    int16_t angle;
    int16_t angle_rate;
    unsigned char acc[3];
};
struct Cliff_data 
{
   uint16_t bottom[3];
};
struct Current_data 
{
    uint8_t left_motor;
    uint8_t right_motor;
};

struct version_
{
  uint8_t patch;
  uint8_t minor;
  uint8_t major;
};
struct Hardware_data 
{
    version_ version;
};
struct Firmware_data 
{
    version_ version;
};
struct Ultrasonic
{
  unsigned short DISL1;
  unsigned short DISL2;
  unsigned short DISL3;
  unsigned short DISL4;
  unsigned short DISL5;
};
#define MAX_DATA_SIZE (3*8) //derived from ST_GYRO_MAX_DATA_SIZE in firmware
struct Raw_gyro
{
  unsigned short x_axis;
  unsigned short y_axis;
  unsigned short z_axis;
  uint8_t frame_id;
};
struct ThreeAxisGyro_data 
{
    unsigned char frame_id;
    unsigned char followed_data_length;
    Raw_gyro data[MAX_DATA_SIZE];
};

struct GpInput_data 
{
    uint16_t digital_input;
    uint16_t analog_input[4];
};
struct UniqueDeviceID_data 
{
    uint32_t udid0;
    uint32_t udid1;
    uint32_t udid2;
};

struct ControllerInfo_data 
{
    unsigned char type;
    unsigned int p_gain; //default value: 100 * 1000
    unsigned int i_gain; //default value: 0.1 * 1000
    unsigned int d_gain; //default value:   2 * 1000
};
#endif
