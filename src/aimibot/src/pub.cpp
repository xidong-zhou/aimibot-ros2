#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mymsgs/msg/cliff.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/TwistStamped.h> 

int main(int argc, char * argv[])
{
    //ros::init(argc, argv, "talker");
    rclcpp::init(argc, argv);

    //ros::NodeHandle n;
    auto node = rclcpp::Node::make_shared("talker");


    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    auto chatter_pub = node->create_publisher<geometry_msgs::msg::Twist>("Current",100);

    //ros::Rate loop_rate(10);
    rclcpp::WallRate loop_rate(2);

    geometry_msgs::msg::Twist msg;
    //auto i = 1;

    //while (ros::ok())
    while (rclcpp::ok()) 
    {
        //msg.data = "Hello World: " + std::to_string(i++);
        //std::cout << "Publishing: '" << msg.data << "'" << std::endl;
	//msg.le = 30;
        //chatter_pub.publish(msg);
        chatter_pub->publish(msg);

        //ros::spinOnce();
        rclcpp::spin_some(node);

        //loop_rate.sleep();
        loop_rate.sleep();
    }

    return 0;
}

