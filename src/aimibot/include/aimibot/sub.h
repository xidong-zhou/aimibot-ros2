#include "std_msgs/msg/string.hpp"
#include "mymsgs/msg/cliff.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class MinimalSubscriber {
 public:
	MinimalSubscriber();
	rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("nho") ;

 private: 
	void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
 	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
	void loop();
 
};






