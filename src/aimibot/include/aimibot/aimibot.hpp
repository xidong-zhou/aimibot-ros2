#ifndef __AIMIBOT__H
#define __AIMIBOT__H

#include "rclcpp/rclcpp.hpp" 
#include <serial/serial.h>
#include <std_msgs/msg/string.hpp> 
#include <std_msgs/msg/empty.hpp> 
#include "message_callback.hpp"
#include <ecl/math.hpp>
#include <ecl/geometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "diff_driver.hpp"
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "data_struct.hpp"
#include "odometry.hpp"
#include "math.h"

#define pi 3.1415926

using namespace std;
using ecl::Angle;
using ecl::wrap_angle;



class aimibot  {
 public:
    serial::Serial sp;
    aimibot() ;
    mymsgs::msg::Cliff cliff_pubdata;
	mymsgs::msg::CoreSensors coresense_pubdata;
	mymsgs::msg::Hardware hardware_pubdata;
	mymsgs::msg::ControllerInfo controlinfo_pubdata;
	mymsgs::msg::DockInfraRed dockin_pubdata;
	mymsgs::msg::Inertia iner_pubdata;
	mymsgs::msg::ThreeAxisGyro threeag_pubdata;
	mymsgs::msg::UniqueDeviceID uniid_pubdata;
	mymsgs::msg::Current cur_pubdata;
	mymsgs::msg::GpInput gpin_pubdata;
	mymsgs::msg::Ultrasonic ultrasonic_pubdata;
	sensor_msgs::msg::JointState joint_states;
	
	CoreSensors_data coresensors_data;
	ControllerInfo_data controlinfo_data;
	UniqueDeviceID_data uniqid_data;
	GpInput_data gpin_data;
	ThreeAxisGyro_data threeAG_data;
	Firmware_data firmware_Data;
	Hardware_data hardware_data;
	Current_data current_data;
	Cliff_data cliff_Data;
	Inertia_data iner_data;
	DockInfraRed_data dockin_data;
	Ultrasonic ultrasonic_data;
	rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("aimibot") ;    
 private: 
 
    
	Aimi::Odom odome_try;
	Aimi::DiffDriver diff_drive;
    void subscribeVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr msg);
    //~ void subscribeLed1Command(const mymsgs::msg::Led::SharedPtr led1_command_subscriber);
    //~ void subscribeLed2Command(const mymsgs::msg::Led::SharedPtr led2_command_subscriber);
    //~ void subscribeDigitalOutputCommand(const mymsgs::msg::DigitalOutput::SharedPtr digital_output_command_subscriber);
    //~ void subscribeExternalPowerCommand(const mymsgs::msg::DigitalOutput::SharedPtr external_power_command_subscriber);
    //~ void subscribeSoundCommand(const mymsgs::msg::beep_sound::SharedPtr sound_command_subscriber);
    //~ void subscribeResetOdometry(const std_msgs::Empty::SharedPtr reset_odometry_subscriber);
    //~ void subscribeMotorPower(const mymsgs::msg::MotorPower::SharedPtr motor_power_subscriber);
    //~ void subscribeControllerInfoCommand(const mymsgs::msg::ControllerInfo::SharedPtr controller_info_command_subscriber);



	rclcpp::Publisher<mymsgs::msg::Hardware>::SharedPtr version_info_publisher;
	rclcpp::Publisher<mymsgs::msg::CoreSensors>::SharedPtr coresensor_publisher;
	rclcpp::Publisher<mymsgs::msg::DockInfraRed>::SharedPtr dock_ir_publisher;
	rclcpp::Publisher<mymsgs::msg::Inertia>::SharedPtr inertia_publisher;
	rclcpp::Publisher<mymsgs::msg::ThreeAxisGyro>::SharedPtr raw_imu_data_publisher;
	rclcpp::Publisher<mymsgs::msg::UniqueDeviceID>::SharedPtr unique_id_publisher;
	rclcpp::Publisher<mymsgs::msg::Current>::SharedPtr current_publisher;
	rclcpp::Publisher<mymsgs::msg::GpInput>::SharedPtr gpinput_publisher;
	rclcpp::Publisher<mymsgs::msg::Cliff>::SharedPtr cliff_state_publisher;
	rclcpp::Publisher<mymsgs::msg::ControllerInfo>::SharedPtr controller_info_publisher;
	rclcpp::Publisher<mymsgs::msg::Ultrasonic>::SharedPtr ultrasonic_data_publisher;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
    
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_subscriber;
	//~ rclcpp::Subscription<mymsgs::msg::Led>::SharedPtr led1_command_subscriber;
	//~ rclcpp::Subscription<mymsgs::msg::Led>::SharedPtr led2_command_subscriber;
	//~ rclcpp::Subscription<mymsgs::msg::DigitalOutput>::SharedPtr digital_output_command_subscriber;
	//~ rclcpp::Subscription<mymsgs::msg::DigitalOutput>::SharedPtr external_power_command_subscriber;
	//~ rclcpp::Subscription<mymsgs::msg::Sound>::SharedPtr sound_command_subscriber;
	//~ rclcpp::Subscription<std_msgs::Empty>::SharedPtr reset_odometry_subscriber;
	//~ rclcpp::Subscription<mymsgs::msg::MotorPower>::SharedPtr motor_power_subscriber;
	//~ rclcpp::Subscription<mymsgs::msg::ControllerInfo>::SharedPtr controller_info_command_subscriber;
	
    std::string port = "/dev/ttyUSB0";
    	
	bool first_read=false;
	bool head_init = false;
	bool findheader0=false;
	bool findheader1=false;
	bool isconnect=false;	
	uint8_t ready_get=1;   //defalut for read header[0] header[1]
	uint8_t hasread=0;
	uint8_t payload_size=0;	
	uint8_t temp_buffer[256];
    uint8_t buffer[256];
    uint8_t i =0;
    double heading_offset=0.0;
    const double digit_to_dps = 0.00875;
    
    ecl::Angle<double> gethand;   
    ecl::LegacyPose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    
    
	rclcpp::Clock clock;
    //~ rclcpp::Duration duration;
	double init_head();
	double getHeading(); 
    double wrap_angle(double angle); 
	void reset_odometry();

    
    void shutdown(int sig);
    
    void init();
    int seial_get();
    int start();
    
	double getAngularVelocity() ;
	void rostopic_pub(CoreSensors_data coresensors_data_,
		  Ultrasonic ultrasonic_data_,
		  GpInput_data gpin_data_,
		  ThreeAxisGyro_data threeAG_data_,
		  Firmware_data firmware_Data_,
		  Hardware_data hardware_data_,
		  Current_data current_data_,
		  Cliff_data cliff_Data_,
		  Inertia_data iner_data_,
		  DockInfraRed_data dockin_data_);
		  
	bool findpack_update(const unsigned char * incoming, unsigned int numberOfIncoming);
	
	enum packetFinderState
    {
		waitingForHead,
		waitingForPayloadSize,
		waitingForPayload,
		waitingForCheckSum
	};packetFinderState state=waitingForHead;
	
	class IDHeader {
public:
  enum PayloadType {
  // Streamed payloads
  CoreSensors = 1, DockInfraRed = 3, Inertia = 4, Cliff = 5, Current = 6,

  // Service Payloads
  Hardware = 10, Firmware = 11, ThreeAxisGyro = 13, Eeprom = 15, GpInput = 16,

  UniqueDeviceID = 19, Reserved = 20, Ultrasonic = 21
  };
};

};







#endif
