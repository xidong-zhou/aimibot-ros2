/*
 * Author: ZHOU XIDONG
 * Data:2021.3.8
*/

#include <iostream>
#include "../include/aimibot/aimibot.hpp"
#include <signal.h> 
#include <memory>


serial::Serial sp;
namespace ecl 
{
	const double& wrap_angle(double &angle) {
		if ( (  angle <= pi ) && ( angle >= -pi ) ) {
			return angle; // nothing to do.
		}
		if ( angle < 0.0 ) {
			angle = fmod(angle-pi,2.0*pi)+pi;
		} else {
			angle = fmod(angle+pi,2.0*pi)-pi;
		}
		return angle;
	}
} // namespace ecl


double aimibot::wrap_angle(double angle) 
{
		if ( (  angle <= pi ) && ( angle >= -pi ) ) {
			return angle; // nothing to do.
		}
		if ( angle < 0.0 ) {
			angle = fmod(angle-pi,2.0*pi)+pi;
		} else {
			angle = fmod(angle+pi,2.0*pi)-pi;
		}
		return angle;
}


aimibot::aimibot() 
{		
	cliff_state_publisher = nh->create_publisher<mymsgs::msg::Cliff>("/aimibot/cliff",100); 
	coresensor_publisher = nh->create_publisher<mymsgs::msg::CoreSensors>("/aimibot/CoreSensors",100); 
	version_info_publisher = nh->create_publisher<mymsgs::msg::Hardware>("/aimibot/version_info",100); 
	controller_info_publisher = nh->create_publisher<mymsgs::msg::ControllerInfo>("/aimibot/controller_info",100); 
	dock_ir_publisher = nh->create_publisher<mymsgs::msg::DockInfraRed>("/aimibot/sensors/dock_ir",100); 
	inertia_publisher = nh->create_publisher<mymsgs::msg::Inertia>("/aimibot/sensors/imu_data",100); 
	raw_imu_data_publisher = nh->create_publisher<mymsgs::msg::ThreeAxisGyro>("/aimibot/sensors/imu_data_raw",100); 
	unique_id_publisher = nh->create_publisher<mymsgs::msg::UniqueDeviceID>("/aimibot/unique_id",100); 
	current_publisher = nh->create_publisher<mymsgs::msg::Current>("/aimibot/current",100); 
	gpinput_publisher = nh->create_publisher<mymsgs::msg::GpInput>("/aimibot/GpInput",100); 
	ultrasonic_data_publisher = nh->create_publisher<mymsgs::msg::Ultrasonic>("/aimibot/Ultrasonic",100);
	joint_state_publisher = nh->create_publisher<sensor_msgs::msg::JointState>("/joint_states",100);

	velocity_command_subscriber = nh->create_subscription<geometry_msgs::msg::Twist>("/aimibot/commands/velocity",10,std::bind(&aimibot::subscribeVelocityCommand, this, std::placeholders::_1));
	 //~ led1_command_subscriber = this->create_subscription<mymsgs::msg::Led>("/aimibot/Ultrasonic",10,std::bind(&aimibot::subscribeLed1Command, this, _1));
	 //~ led2_command_subscriber = this->create_subscription<mymsgs::msg::Led>("/aimibot/Ultrasonic",10,std::bind(&aimibot::subscribeLed2Command, this, _1));
	 //~ digital_output_command_subscriber = this->create_subscription<mymsgs::msg::DigitalOutput>("/aimibot/Ultrasonic",10,std::bind(&aimibot::subscribeDigitalOutputCommand, this, _1));
	 //~ external_power_command_subscriber = this->create_subscription<mymsgs::msg::DigitalOutput>("/aimibot/Ultrasonic",10,std::bind(&aimibot::subscribeExternalPowerCommand, this, _1));
	 //~ sound_command_subscriber = this->create_subscription<mymsgs::msg::Sound>("/aimibot/Ultrasonic",10,std::bind(&aimibot::subscribeSoundCommandk, this, _1));
	 //~ reset_odometry_subscriber = this->create_subscription<std_msgs::Empty>("/aimibot/Ultrasonic",10,std::bind(&aimibot::subscribeResetOdometry, this, _1));
	 //~ motor_power_subscriber = this->create_subscription<mymsgs::msg::MotorPower>("/aimibot/Ultrasonic",10,std::bind(&aimibot::subscribeMotorPower, this, _1));
	 //~ controller_info_command_subscriber = this->create_subscription<mymsgs::msg::ControllerInfo>("/aimibot/Ultrasonic",10,std::bind(&aimibot::subscribeControllerInfoCommand, this, _1));

	 init();
}


/**初始化 机器人头部朝向***/
double aimibot::init_head()
{
  return  (static_cast<double>(iner_data.angle) / 100.0) * pi / 180.0;
}
/**返回 机器人偏航角***/
double aimibot::getHeading() 
{
  double heading;
  // raw data angles are in hundredths of a degree, convert to radians.
  heading = (static_cast<double>(iner_data.angle) / 100.0) * pi / 180.0;
  return wrap_angle(heading - heading_offset);
}
void aimibot::reset_odometry()
{
  //~ diff_drive.reset();
  heading_offset = (static_cast<double>(iner_data.angle) / 100.0) * pi / 180.0; 
}
double aimibot::getAngularVelocity() 
{
  return (static_cast<double>(iner_data.angle_rate) / 100.0) * pi / 180.0;
}


void aimibot::rostopic_pub(CoreSensors_data coresensors_data_,
		  Ultrasonic ultrasonic_data_,
		  GpInput_data gpin_data_,
		  ThreeAxisGyro_data threeAG_data_,
		  Firmware_data firmware_Data_,
		  Hardware_data hardware_data_,
		  Current_data current_data_,
		  Cliff_data cliff_Data_,
		  Inertia_data iner_data_,
		  DockInfraRed_data dockin_data_)
{
	  int k;
	  short int angle_temp , anglerate_temp;
	  ultrasonic_pubdata.disl1=ultrasonic_data.DISL1;
	  ultrasonic_pubdata.disl2=ultrasonic_data.DISL2;
	  ultrasonic_pubdata.disl3=ultrasonic_data.DISL3;
	  ultrasonic_pubdata.disl4=ultrasonic_data.DISL4;
	  ultrasonic_pubdata.disl5=ultrasonic_data.DISL5;
	  cliff_pubdata.bottom[0] = cliff_Data_.bottom[0];
	  cliff_pubdata.bottom[1] =  cliff_Data_.bottom[1];
	  cliff_pubdata.bottom[2] =  cliff_Data_.bottom[2];

	  coresense_pubdata.time_stamp= coresensors_data_.time_stamp; 
	  coresense_pubdata.bumper =coresensors_data_.bumper ;
	  coresense_pubdata.wheel_drop=coresensors_data_.wheel_drop;
	  coresense_pubdata.cliff =coresensors_data_.cliff;
	  coresense_pubdata.left_encoder =coresensors_data_.left_encoder ; 
	  coresense_pubdata.right_encoder= coresensors_data_.right_encoder; 
	  coresense_pubdata.left_pwm =coresensors_data_.left_pwm;
	  coresense_pubdata.right_pwm =coresensors_data_.right_pwm;
	  coresense_pubdata.buttons =coresensors_data_.buttons;
	  coresense_pubdata.charger =coresensors_data_.charger;
	  coresense_pubdata.battery =0.12*coresensors_data_.battery;
	  coresense_pubdata.over_current =coresensors_data_.over_current;


	  hardware_pubdata.hw_major = firmware_Data_.version.major;
	  hardware_pubdata.hw_minor = firmware_Data_.version.minor;
	  hardware_pubdata.hw_patch = firmware_Data_.version.patch;

	  dockin_pubdata.docking[0]=dockin_data_.Docking[0];
	  dockin_pubdata.docking[1]=dockin_data_.Docking[1];
	  dockin_pubdata.docking[2]=dockin_data_.Docking[2];
	  iner_pubdata.angle= getHeading();
	  iner_pubdata.angle_rate= (iner_data_.angle_rate / 100.0)*pi / 180.0;
	  iner_pubdata.acc[0]== iner_data_.acc[0];
	  iner_pubdata.acc[1]== iner_data_.acc[1];
	  iner_pubdata.acc[2]== iner_data_.acc[2];
	  threeag_pubdata.angular_velocity_x=threeag_pubdata.angular_velocity_y=threeag_pubdata.angular_velocity_z=0;
	  for(k=0;k<threeAG_data_.followed_data_length;k++)
	    {
	      threeag_pubdata.angular_velocity_x +=      threeAG_data_.data[k].x_axis;
	      threeag_pubdata.angular_velocity_y +=      threeAG_data_.data[k].y_axis ;
	      threeag_pubdata.angular_velocity_z +=      threeAG_data_.data[k].z_axis;	      
	    }
	  threeag_pubdata.frame_id  =    threeAG_data_.data[k-1].frame_id;
	  threeag_pubdata.angular_velocity_x = threeag_pubdata.angular_velocity_x/k;
	  threeag_pubdata.angular_velocity_y = threeag_pubdata.angular_velocity_y/k;
	  threeag_pubdata.angular_velocity_z = threeag_pubdata.angular_velocity_z/k;

	  cur_pubdata.left_motor =current_data_.left_motor;
	  cur_pubdata.right_motor =current_data_.right_motor;
	  gpin_pubdata.digital_input =gpin_data_.digital_input;
	  gpin_pubdata.analog_input[0] =gpin_data_.analog_input[0];
	  gpin_pubdata.analog_input[1] =gpin_data_.analog_input[1];
	  gpin_pubdata.analog_input[2] =gpin_data_.analog_input[2];
	  gpin_pubdata.analog_input[3] =gpin_data_.analog_input[3];

	  cliff_state_publisher->publish(cliff_pubdata);
	  coresensor_publisher->publish(coresense_pubdata);
	  version_info_publisher->publish(hardware_pubdata);
	  controller_info_publisher->publish(controlinfo_pubdata);
	  dock_ir_publisher->publish(dockin_pubdata); 
	  inertia_publisher->publish(iner_pubdata); 
	  raw_imu_data_publisher->publish(threeag_pubdata); 
	  unique_id_publisher->publish(uniid_pubdata); 
	  gpinput_publisher->publish(gpin_pubdata);
	  ultrasonic_data_publisher->publish(ultrasonic_pubdata);
}




bool aimibot::findpack_update(const unsigned char * incoming, unsigned int numberOfIncoming)
{
	if (!(numberOfIncoming > 0))
	{
		findheader0=findheader1=false;
		state=waitingForHead;
		return false;
	}

	bool found_packet(false);
	static unsigned char cs(0);
	uint8_t subpay_size=0;
	uint8_t offset=0;
	uint8_t add=0;
	uint8_t k;
	uint8_t location;
	uint8_t checksum=0;
	if(state==waitingForPayload)
	{
		coresensors_data={0};
		controlinfo_data={0};
		uniqid_data={0};
		gpin_data={0};
		threeAG_data={0};
		firmware_Data={0};
		hardware_data={0};
		current_data={0};
		cliff_Data={0};
		iner_data={0};
		dockin_data={0};
	}
	switch(state)
	{
		case(waitingForHead): 
			if((incoming[0]==0xaa)&&findheader0==false)  
			{
				findheader0=true;ready_get=1;
			}
			else if((findheader0==true)&&(incoming[0]==0x55)&&(findheader1==false))  
			{ 
				findheader1=true; state=waitingForPayloadSize;   
			}
			break;
		case(waitingForPayloadSize): 
			if(incoming[0]<3) 
			{ 
				std::cout<< "payload_size is too short" << std::endl; 
				break;
			}
			    
			payload_size = incoming[0];
			ready_get=  incoming[0];
			state=waitingForPayload;
			cs^=incoming[0];
			break;
		case(waitingForPayload): 
	
			while (offset!=(payload_size))
			{
				add=offset+1;
				switch(incoming[offset])
				{
					case IDHeader::CoreSensors: 
						subpay_size = incoming[add];
						coresensors_data.time_stamp =(incoming[offset+3]<<8)|incoming[offset+2] ; 
						coresensors_data.bumper = incoming[offset+4];
						coresensors_data.wheel_drop= incoming[offset+5];
						coresensors_data.cliff= incoming[offset+6];
						coresensors_data.left_encoder= (incoming[offset+8]<<8)|incoming[offset+7] ; 
						coresensors_data.right_encoder= (incoming[offset+10]<<8)|incoming[offset+9] ; 
						coresensors_data.left_pwm= incoming[offset+11];
						coresensors_data.right_pwm= incoming[offset+12]; 
						coresensors_data.buttons= incoming[offset+13];
						coresensors_data.charger= incoming[offset+14];
						coresensors_data.battery= incoming[offset+15];
						coresensors_data.over_current= incoming[offset+16];
						break;
					case IDHeader::DockInfraRed:
						subpay_size = incoming[add];
						dockin_data.Docking[0]=incoming[offset+2];
						dockin_data.Docking[1]=incoming[offset+3];
						dockin_data.Docking[2]=incoming[offset+4];
						break;
					case IDHeader::Inertia:
						subpay_size = incoming[add];
						iner_data.angle = (int16_t)(incoming[offset+3]<<8)|incoming[offset+2] ;
						iner_data.angle_rate = (int16_t)(incoming[offset+5]<<8)|incoming[offset+4] ;
						iner_data.acc[0] = incoming[offset+6];
						iner_data.acc[1] = incoming[offset+7];
						iner_data.acc[2] = incoming[offset+8];
						break;
					case IDHeader::Cliff:
						subpay_size = incoming[add]; 
						cliff_Data.bottom[0]= (incoming[offset+3]<<8)|incoming[offset+2] ;
						cliff_Data.bottom[1]= (incoming[offset+5]<<8)|incoming[offset+4] ;
						cliff_Data.bottom[2]= (incoming[offset+7]<<8)|incoming[offset+6] ;
						break;
					case IDHeader::Current:
						subpay_size = incoming[add]; 
						current_data.left_motor= (incoming[offset+3]<<8)|incoming[offset+2] ;
						current_data.right_motor= (incoming[offset+5]<<8)|incoming[offset+4] ;
						break;
					case IDHeader::ThreeAxisGyro:
						subpay_size = incoming[add];
						threeAG_data.frame_id = incoming[offset+2];
						threeAG_data.followed_data_length = incoming[offset+3]/3 ;
						for(k=0;k<threeAG_data.followed_data_length;k++)
						{
						  threeAG_data.data[k].x_axis= -digit_to_dps * (short)( (incoming[offset+6*k+5]<<8)|incoming[offset+6*k+4]) ;
						  threeAG_data.data[k].y_axis= digit_to_dps * (short)((incoming[offset+6*k+7]<<8)|incoming[offset+6*k+6]) ;
						  threeAG_data.data[k].z_axis= digit_to_dps * (short)((incoming[offset+6*k+9]<<8)|incoming[offset+6*k+8]) ;
						  threeAG_data.data[k].frame_id=threeAG_data.frame_id;
						}
						break;
					case IDHeader::GpInput:
						subpay_size = incoming[add];
						gpin_data.digital_input=  (incoming[offset+3]<<8)|incoming[offset+2] ;
						gpin_data.analog_input[0]=  (incoming[offset+5]<<8)|incoming[offset+4] ;
						gpin_data.analog_input[1]=  (incoming[offset+7]<<8)|incoming[offset+6] ;
						gpin_data.analog_input[2]=  (incoming[offset+9]<<8)|incoming[offset+8] ;
						gpin_data.analog_input[3]=  (incoming[offset+11]<<8)|incoming[offset+10] ;
						break;
					case IDHeader::Hardware:
						subpay_size = incoming[add];
						hardware_data.version.patch=  (incoming[offset+3]<<8)|incoming[offset+2] ;
						hardware_data.version.minor=  (incoming[offset+5]<<8)|incoming[offset+4] ;
						hardware_data.version.major=  (incoming[offset+7]<<8)|incoming[offset+6] ;
						break;
					case IDHeader::Firmware: 
						subpay_size = incoming[add];
						firmware_Data.version.patch=  (incoming[offset+3]<<8)|incoming[offset+2] ;
						firmware_Data.version.minor=  (incoming[offset+5]<<8)|incoming[offset+4] ;
						firmware_Data.version.major=  (incoming[offset+7]<<8)|incoming[offset+6] ;
						break;
					case IDHeader::UniqueDeviceID:
						subpay_size = incoming[add];
						break;
					case IDHeader::Ultrasonic: 
						subpay_size = incoming[add];
						ultrasonic_data.DISL1=  (incoming[offset+3]<<8)|incoming[offset+2] ;
						ultrasonic_data.DISL2=  (incoming[offset+5]<<8)|incoming[offset+4] ;
						ultrasonic_data.DISL3=  (incoming[offset+7]<<8)|incoming[offset+6] ;
						ultrasonic_data.DISL4=  (incoming[offset+9]<<8)|incoming[offset+8] ;
						ultrasonic_data.DISL5=  (incoming[offset+11]<<8)|incoming[offset+10] ;
						break;
					default:
						subpay_size=payload_size-offset-2;
					break;
				}
				offset+=(subpay_size+2);
			}
			for(k=0;k<payload_size;k++)
				cs^=incoming[k];
			//printf("%x\n",cs);
			state=waitingForCheckSum;
			ready_get=1;
			break;
		case(waitingForCheckSum):
			checksum=incoming[0];
			//printf("%x\n",checksum);
			if(checksum!=cs) 
			{
				RCLCPP_INFO(nh->get_logger(),"PACK FOUND ERR");
				found_packet=false;
			}
			else  
			{ 
				found_packet=true; 
			}
			cs=0;
			findheader0=findheader1=false;
			state=waitingForHead;
			isconnect=true;
			break;
		default: 
			break;
	}

  return found_packet;
}

void aimibot::init()
{
    odome_try.init(nh);
    reset_odometry();
    joint_states.name.push_back("leftwheel_joint");
    joint_states.name.push_back("rightwheel_joint");
    seial_get();
}



int aimibot::seial_get()
{
	 try
    {
       //创建timeout
       serial::Timeout to = serial::Timeout::simpleTimeout(1000);
       //设置要打开的串口名称
       sp.setPort(port);
       //设置串口通信的波特率
       sp.setBaudrate(115200);
       //串口设置timeout
       sp.setTimeout(to);
       //打开串口
       sp.open();
    }
    catch(serial::IOException& e)
    {
         std::cout<< "Unable to open port."<<std::endl;
         return -1;
    }
    
    //~ //判断串口是否打开成功
    if(sp.isOpen())
    {
         std::cout<< "Device is opened."<<std::endl;
    }
    else
    {
         return -1;
    }
    
    rclcpp::WallRate loop_rate(100);   
    auto last_getdata =  clock.now();    
    while(rclcpp::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.read(temp_buffer, ready_get); 
         
        //数据异常
        if(n==0) 
         {
           std::cout<<"waiting for data 10s" << std::endl;
           if(clock.now() - last_getdata > rclcpp::Duration(10,0))
            {
              std::cout<< "TIME OUT!CLOSE SERIAL!"<<std::endl;
              sp.close();
              rclcpp::shutdown(); 
            }continue;
          }
          
        //读出数据  
        if(findpack_update(temp_buffer,n)) 
	    {
			   last_getdata = clock.now();
		       if(!head_init) { heading_offset = init_head(); head_init = true;}
		       
					rostopic_pub(coresensors_data,ultrasonic_data,gpin_data,threeAG_data,firmware_Data,hardware_data,current_data,cliff_Data,iner_data,dockin_data);	 
					joint_states.position.resize(2,0.0);
					joint_states.velocity.resize(2,0.0);
					joint_states.effort.resize(2,0.0);

					diff_drive.update(coresensors_data.time_stamp, coresensors_data.left_encoder, coresensors_data.right_encoder,pose_update, pose_update_rates);
					diff_drive.getWheelJointStates(joint_states.position[0], joint_states.velocity[0],joint_states.position[1], joint_states.velocity[1]);
					odome_try.update(pose_update,pose_update_rates,getHeading(),getAngularVelocity());
					if(rclcpp::ok())
					{
						joint_states.header.stamp = clock.now();
						joint_state_publisher->publish(joint_states);     
					}
			
		}
        rclcpp::spin_some(nh);
	}
    //关闭串口
	sp.close();
	return 0;
}



void shutdown(int sig)  
{  
       sp.close();
       rclcpp::shutdown();  
}

int main(int argc, char** argv)
{      
    rclcpp::init(argc, argv);
    
	signal(SIGINT, shutdown);

	aimibot aimibot_;

	return 0;
}
