#ifndef PX4_ROS_H
#define PX4_ROS_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <px4_msgs/msg/vehicle_odometry.hpp>
// #include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
// #include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros_com/frame_transforms.h> /*Transform conversions*/

#include <nav_msgs/msg/odometry.hpp> // ROS odom -> PX4 odom

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace px4_ros_com::frame_transforms;

class PX4ROS : public rclcpp::Node
{
public:
    PX4ROS(/* args */);
    ~PX4ROS();
private:
    std::string odom_frame_id_; /* Frame name of the odom msg, and TF */
    std::string baselink_frame_id_;
    nav_msgs::msg::Odometry ros_odom_msg_; /* ROS odometry msg to be published*/
    geometry_msgs::msg::PoseStamped ros_pose_msg_;

    rclcpp::Time last_tf_pub_time_;
    double tf_pub_period_;
    bool publish_tf_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; /**< Publisher for Odometry ROS msg  */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_; /**< Publisher for pose stamped ROS msg  */
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vis_odom_pub_; /**< Publisher for fmu/in/vehicle_visual_odometry */

    // rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr att_sp_pub_; /**< Publishes to px4 fmu/in/vehicle_attitude_setpoint topic  */
    // rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_sp_pub_; /**< Publishes to px4 fmu/in/vehicle_rates_setpoint  */
    // rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_sp_pub_; /**< Publishes to px4 fmu/in/trajectory_setpoint  */

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;/**< Subscriber to px4 odometry. */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ros_vis_odom_sub_;/**< Subscriber to ROS visual odometry. */

    /**
     * @brief Callback for PX4 odom. Converts PX4 odom to ROS odom
     * @param msg [px4_msgs::msg::VehicleOdometry] 
    */
    void px4OdomCallback(const px4_msgs::msg::VehicleOdometry & msg);

    /**
     * @brief Converts ROS odom topic to PX4 visual odom topic.
     *  typically used to inject visual odometry info
     * @param msg [nav_msgs::msg::Odometry] 
    */
    void rosVisualOdomCallback(const nav_msgs::msg::Odometry & msg);
};

/**
 * @brief PX4ROS Class definition
*/
PX4ROS::~PX4ROS()
{

}
PX4ROS::PX4ROS(/* args */): Node("px4_ros")
{
    last_tf_pub_time_ = this->get_clock()->now();    

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    this->declare_parameter("odom_frame", "odom_link");
    odom_frame_id_ = this->get_parameter("odom_frame").get_parameter_value().get<std::string>();

    this->declare_parameter("baselink_frame", "base_link");
    baselink_frame_id_ = this->get_parameter("baselink_frame").get_parameter_value().get<std::string>();

    this->declare_parameter("tf_pub_period", 0.05);
    tf_pub_period_ = this->get_parameter("tf_pub_period").get_parameter_value().get<double>();

    this->declare_parameter("publish_tf", true);
    publish_tf_ = this->get_parameter("publish_tf").get_parameter_value().get<bool>();
   
    odom_pub_ =  this->create_publisher<nav_msgs::msg::Odometry>("px4_ros/odom", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("px4_ros/pose", 10);
    vis_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("fmu/in/vehicle_visual_odometry", 10);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "fmu/out/vehicle_odometry", qos, std::bind(&PX4ROS::px4OdomCallback, this, _1));

    ros_vis_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "vio/ros_odom", qos, std::bind(&PX4ROS::rosVisualOdomCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "%s has started ..." ,this->get_name());

    RCLCPP_INFO(this->get_logger(), "Namespace %s" ,this->get_namespace());

}

void
PX4ROS::px4OdomCallback(const px4_msgs::msg::VehicleOdometry & msg)
{
    Eigen::Vector3d position_ned(msg.position[0], msg.position[1], msg.position[2]);
    Eigen::Vector3d velocity_ned(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
    Eigen::Quaterniond q_ned(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
    Eigen::Vector3d angular_vel_px4(msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]);
    Covariance3d cov_pos_px4 = {
        msg.position_variance[0], 0.0, 0.0,
        0.0, msg.position_variance[1], 0.0,
        0.0, 0.0, msg.position_variance[2]
    };
    Covariance3d cov_rot_px4 = {
        msg.orientation_variance[0], 0.0, 0.0,
        0.0, msg.orientation_variance[1], 0.0,
        0.0, 0.0, msg.orientation_variance[2]
    };
    Covariance3d cov_vel_px4 = {
        msg.velocity_variance[0], 0.0, 0.0,
        0.0, msg.velocity_variance[1], 0.0,
        0.0, 0.0, msg.velocity_variance[2]
    };

    // Convert px4 -> ros
    Eigen::Quaterniond q_enu = px4_to_ros_orientation(q_ned);
    Eigen::Vector3d position_enu = ned_to_enu_local_frame(position_ned);
    Eigen::Vector3d velocity_enu = ned_to_enu_local_frame(velocity_ned);
    Eigen::Vector3d angular_vel_ros = aircraft_to_baselink_body_frame(angular_vel_px4);
    Covariance3d cov_pos_ros = transform_static_frame(cov_pos_px4, StaticTF::NED_TO_ENU);
    Covariance3d cov_vel_ros = transform_static_frame(cov_vel_px4, StaticTF::NED_TO_ENU);
    Covariance3d cov_rot_ros = transform_frame(cov_rot_px4, q_enu);

    // Fill Odometry msg
    ros_odom_msg_.pose.covariance = {
        cov_pos_ros[0], cov_pos_ros[1], cov_pos_ros[2], 0., 0., 0.,
        cov_pos_ros[3], cov_pos_ros[4], cov_pos_ros[5], 0., 0., 0.,
        cov_pos_ros[6], cov_pos_ros[7], cov_pos_ros[8], 0., 0., 0.,
        0., 0., 0., cov_rot_ros[0], cov_rot_ros[1], cov_rot_ros[2],
        0., 0., 0., cov_rot_ros[3], cov_rot_ros[4], cov_rot_ros[5],
        0., 0., 0., cov_rot_ros[6], cov_rot_ros[7], cov_rot_ros[8]
    };

    ros_odom_msg_.twist.covariance = {
        cov_vel_ros[0], cov_vel_ros[1], cov_vel_ros[2], 0., 0., 0.,
        cov_vel_ros[3], cov_vel_ros[4], cov_vel_ros[5], 0., 0., 0.,
        cov_vel_ros[6], cov_vel_ros[7], cov_vel_ros[8], 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.
    };
    
    ros_odom_msg_.pose.pose.position.x = position_enu.x();
    ros_odom_msg_.pose.pose.position.y = position_enu.y();
    ros_odom_msg_.pose.pose.position.z = position_enu.z();
    ros_odom_msg_.pose.pose.orientation.w = q_enu.w();
    ros_odom_msg_.pose.pose.orientation.x = q_enu.x();
    ros_odom_msg_.pose.pose.orientation.y = q_enu.y();
    ros_odom_msg_.pose.pose.orientation.z = q_enu.z();
    ros_odom_msg_.twist.twist.linear.x = velocity_enu.x();
    ros_odom_msg_.twist.twist.linear.y = velocity_enu.y();
    ros_odom_msg_.twist.twist.linear.z = velocity_enu.z();
    ros_odom_msg_.twist.twist.angular.x = angular_vel_ros.x();
    ros_odom_msg_.twist.twist.angular.y = angular_vel_ros.y();
    ros_odom_msg_.twist.twist.angular.z = angular_vel_ros.z();

    if(std::string(this->get_namespace()) == "/")
        ros_odom_msg_.header.frame_id = odom_frame_id_;
    else
        ros_odom_msg_.header.frame_id =std::string(this->get_namespace())+"/"+ odom_frame_id_;

    ros_odom_msg_.header.stamp = this->get_clock()->now();
    ros_odom_msg_.child_frame_id = baselink_frame_id_;

    odom_pub_->publish(ros_odom_msg_);

    // Fill PoseStamped
    ros_pose_msg_.header = ros_odom_msg_.header;
    ros_pose_msg_.pose.position = ros_odom_msg_.pose.pose.position;
    ros_pose_msg_.pose.orientation = ros_odom_msg_.pose.pose.orientation;
    pose_pub_->publish(ros_pose_msg_);

    // Publish TF
    if (publish_tf_)
    {
        rclcpp::Duration elapsed_time = this->get_clock()->now() - last_tf_pub_time_;
        if (elapsed_time.seconds() > tf_pub_period_)
        {
            last_tf_pub_time_ = this->get_clock()->now();
            // @todo publish TF
            geometry_msgs::msg::TransformStamped t;
            t.header = ros_odom_msg_.header;
            if(std::string(this->get_namespace()) == "/"){
                // RCLCPP_INFO(this->get_logger(), "Namspace shoud be empty is %s" , this->get_namespace());
                t.child_frame_id = baselink_frame_id_;
            }
            else{
                // RCLCPP_INFO(this->get_logger(), "Namspace is %s" , this->get_namespace());
                t.child_frame_id = std::string(this->get_namespace()) +"/" + baselink_frame_id_;
            }
                

            t.transform.translation.x = ros_pose_msg_.pose.position.x;
            t.transform.translation.y = ros_pose_msg_.pose.position.y;
            t.transform.translation.z = ros_pose_msg_.pose.position.z;
            t.transform.rotation.w = ros_pose_msg_.pose.orientation.w;
            t.transform.rotation.x = ros_pose_msg_.pose.orientation.x;
            t.transform.rotation.y = ros_pose_msg_.pose.orientation.y;
            t.transform.rotation.z = ros_pose_msg_.pose.orientation.z;
            tf_broadcaster_->sendTransform(t);
        }
    }
}

void
PX4ROS::rosVisualOdomCallback(const nav_msgs::msg::Odometry & msg)
{
    px4_msgs::msg::VehicleOdometry px4_odom_msg;
    // It seems that time is updated automatically on PX4 side
    // after v1.14
    px4_odom_msg.timestamp = 0; //static_cast<uint64_t>(msg.header.stamp.sec*1e6) + static_cast<uint64_t>(msg.header.stamp.nanosec/1e3);
    px4_odom_msg.timestamp_sample = 0;

    px4_odom_msg.pose_frame = px4_odom_msg.POSE_FRAME_FRD;
    px4_odom_msg.velocity_frame = px4_odom_msg.VELOCITY_FRAME_FRD;

    // position
    Eigen::Vector3d position_enu(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    Eigen::Vector3d position_ned = enu_to_ned_local_frame(position_enu);
    // Velocity
    Eigen::Vector3d velocity_enu(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
    Eigen::Vector3d velocity_ned = enu_to_ned_local_frame(velocity_enu);
    // Orientation
    Eigen::Quaterniond q_enu(msg.pose.pose.orientation.w,
                            msg.pose.pose.orientation.x,
                            msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z);
    Eigen::Quaterniond q_ned = ros_to_px4_orientation(q_enu);

    // Angular Velocity
    Eigen::Vector3d angular_vel_ros(msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z);
    Eigen::Vector3d angular_vel_px4 = baselink_to_aircraft_body_frame(angular_vel_ros);
    

    // Pose covariance
    Covariance3d cov_pos_ros = {
        msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[2],
        msg.pose.covariance[6], msg.pose.covariance[7], msg.pose.covariance[8],
        msg.pose.covariance[12], msg.pose.covariance[13], msg.pose.covariance[14]
    };

    Covariance3d cov_rot_ros = {
        msg.pose.covariance[21], msg.pose.covariance[22], msg.pose.covariance[23],
        msg.pose.covariance[27], msg.pose.covariance[28], msg.pose.covariance[29],
        msg.pose.covariance[33], msg.pose.covariance[34], msg.pose.covariance[35]
    };

    // Velocity covariance

    Covariance3d cov_vel_ros = {
        msg.twist.covariance[0], msg.twist.covariance[1], msg.twist.covariance[2],
        msg.twist.covariance[6], msg.twist.covariance[7], msg.twist.covariance[8],
        msg.twist.covariance[12], msg.twist.covariance[13], msg.twist.covariance[14]
    };

    Covariance3d cov_pos_px4 = transform_static_frame(cov_pos_ros, StaticTF::ENU_TO_NED);
    Covariance3d cov_vel_px4 = transform_static_frame(cov_vel_ros, StaticTF::ENU_TO_NED);
    Covariance3d cov_rot_px4 = transform_frame(cov_rot_ros, q_ned);

    px4_odom_msg.position[0] = position_ned.x();
    px4_odom_msg.position[1] = position_ned.y();
    px4_odom_msg.position[2] = position_ned.z();

    px4_odom_msg.velocity[0] = velocity_ned.x();
    px4_odom_msg.velocity[1] = velocity_ned.y();
    px4_odom_msg.velocity[2] = velocity_ned.z();
    
    px4_odom_msg.q[0] = q_ned.w();
    px4_odom_msg.q[1] = q_ned.x();
    px4_odom_msg.q[2] = q_ned.y();
    px4_odom_msg.q[3] = q_ned.z();

    px4_odom_msg.angular_velocity[0] = angular_vel_px4.x();
    px4_odom_msg.angular_velocity[1] = angular_vel_px4.y();
    px4_odom_msg.angular_velocity[2] = angular_vel_px4.z();

    px4_odom_msg.position_variance[0] = cov_pos_px4[0]; // diagonal elements
    px4_odom_msg.position_variance[1] = cov_pos_px4[4];
    px4_odom_msg.position_variance[2] = cov_pos_px4[8];

    px4_odom_msg.velocity_variance[0] = cov_vel_px4[0];
    px4_odom_msg.velocity_variance[1] = cov_vel_px4[4];
    px4_odom_msg.velocity_variance[2] = cov_vel_px4[8];

    px4_odom_msg.orientation_variance[0] = cov_rot_px4[0];
    px4_odom_msg.orientation_variance[1] = cov_rot_px4[4];
    px4_odom_msg.orientation_variance[2] = cov_rot_px4[8];
    
    vis_odom_pub_->publish(px4_odom_msg);
}

#endif