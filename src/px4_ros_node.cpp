#include "px4_ros_com/px4_ros.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackerROS>());
  rclcpp::shutdown();
  return 0;
}