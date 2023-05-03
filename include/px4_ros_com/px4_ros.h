#ifndef PX4_ROS_H
#define PX4_ROS_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
// #include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
// #include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros_com/frame_transforms.h> /*Transform conversions*/

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace px4_ros_com::frame_transforms;

class PX4ROS : public rclcpp::Node
{
public:
    PX4ROS(/* args */);
    ~PX4ROS();
private:
    nav_msgs::msg::Odometry ros_odom_msg_; /* ROS odometry msg to be published*/

    rclcpp::TimerBase::SharedPtr tf_timer_; /* TF timer */

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; /**< Publisher for Odometry ROS msg  */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_; /**< Publisher for pose stamped ROS msg  */

    // rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr att_sp_pub_; /**< Publishes to px4 fmu/in/vehicle_attitude_setpoint topic  */
    // rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_sp_pub_; /**< Publishes to px4 fmu/in/vehicle_rates_setpoint  */
    // rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_sp_pub_; /**< Publishes to px4 fmu/in/trajectory_setpoint  */

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;/**< Subscriber to px4 odometry. */

    /**
     * @brief Callback for PX4 odom. Converts PX4 odom to ROS odom
     * @param msg [px4_msgs::msg::VehicleOdometry] 
    */
    void px4OdomCallback(const px4_msgs::msg::VehicleOdometry & msg);

    /**
    * @brief Publishes ROS odometry topic. Converts from PX4 to ROS.
    */
    void pubOdom(void);

    /**
     * @brief Brodacst TF based on PX4 odometry topic
     */
    void pubTF(void);
  

};
/**
 * @brief PX4ROS Class definition
*/
PX4ROS::PX4ROS(/* args */): Node("tracker_ros")
{
    tf_timer_ = this->create_wall_timer(
      50ms, std::bind(&PX4ROS::pubTF, this));

      odom_pub_ =  this->create_publisher<nav_msgs::msg::Odometry>("px4_ros/odom", 10);
      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("px4_ros/pose", 10);

      px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "fmu/out/vehicle_odometry", 10, std::bind(&PX4ROS::px4OdomCallback, this, _1));

}

void
PX4ROS::px4OdomCallback(const px4_msgs::msg::VehicleOdometry & msg)
{

}

void
PX4ROS::pubOdom(void)
{

}

void
PX4ROS::pubTF(void)
{

}

#endif