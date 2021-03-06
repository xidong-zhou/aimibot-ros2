cmake_minimum_required(VERSION 3.5)
project(aimibot2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(mymsgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#add_executable(talker src/pub.cpp)
#ament_target_dependencies(talker rclcpp std_msgs mymsgs geometry_msgs nav_msgs tf turtlesim)
add_executable(listener src/sub.cpp)
ament_target_dependencies(listener rclcpp std_msgs mymsgs geometry_msgs)
add_executable(aimibot src/main.cpp src/message_callback.cpp src/odometry src/diff_driver.cpp)
ament_target_dependencies(aimibot rclcpp std_msgs mymsgs sensor_msgs geometry_msgs nav_msgs tf2_ros tf2 serial)
#add_executable(UltrasnoicObstacle src/UltrasnoicObstacle.cpp src/call.cpp)
#ament_target_dependencies(UltrasnoicObstacle rclcpp std_msgs mymsgs geometry_msgs nav_msgs tf ecl)

install(TARGETS
  #talker
  listener
  aimibot
  #UltrasnoicObstacle
  DESTINATION
  lib/${PROJECT_NAME})

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
