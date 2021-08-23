

#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <ecl/geometry/legacy_pose2d.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Aimi {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief  Odometry for the kobuki node.
 **/
class Odom{
public:
  Odom();
  void init(rclcpp::Node::SharedPtr& node );
  bool commandTimeout() const;
  void update(const ecl::LegacyPose2D<double> &pose_update_, ecl::linear_algebra::Vector3d &pose_update_rates, double imu_heading, double imu_angular_velocity);
  rclcpp::Node::SharedPtr nh_o = rclcpp::Node::make_shared("odom") ;

private:
  geometry_msgs::msg::TransformStamped odom_trans;
  ecl::LegacyPose2D<double> pose;
  std::string odom_frame;
  std::string base_frame;
  rclcpp::Clock clock;
  bool publish_tf;
  bool use_imu_heading;
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;

  void publishTransform(geometry_msgs::msg::Quaternion& odom_quat);
  void publishOdometry(geometry_msgs::msg::Quaternion& odom_quat,const ecl::linear_algebra::Vector3d &pose_update_rates);
};
} 
