#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "../include/aimibot/sub.h"


MinimalSubscriber::MinimalSubscriber() 
    {
	
		subscription_ = nh->create_subscription<geometry_msgs::msg::Twist>("/aimibot/commands/velocity", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
		loop();
	  
    }

void MinimalSubscriber::loop()
{
	
	rclcpp::WallRate loop_rate(2);
	  while(rclcpp::ok())
      {
		  RCLCPP_INFO(nh->get_logger(), "loop");
		  rclcpp::spin_some(nh);
		  
	  }
}

void MinimalSubscriber::topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      RCLCPP_INFO(nh->get_logger(), "I heard: '%f'", msg->linear.x);
    }


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  //rclcpp::spin(std::make_shared<MinimalSubscriber>());
  MinimalSubscriber  minimalSubscriber;
  //rclcpp::shutdown();
  return 0;
}
