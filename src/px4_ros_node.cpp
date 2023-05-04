#include "px4_ros_com/px4_ros.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4ROS>());
  rclcpp::shutdown();
  return 0;
}