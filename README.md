# Modifications
The following modifications are made to the original repository.
* A C++ node is created [header](include/px4_ros_com/px4_ros.h), [source](src/px4_ros_node.cpp).
    * This node subscribes to the PX4 vehicle odometry topic (defaults to `fmu/out/vehicle_odometry`) of type `px_msgs/msg/VehicleOdometry`, and transforms it to ROS convention and publishes it to `px4_ros/odom` topic of type `nav_msgs/msg/Odometry`
    * Subscribes to an odometry topic `vio/ros_odom` of type `nav_msgs/msg/Odometry` and transform it to a message of type `px_msgs/msg/VehicleOdometry` and publishes it to `fmu/in/vehicle_visual_odometry` topic. This is useful to feed visual odometry coming from a visual SLAM (or mocap) system to PX4. **NOTE** The input topic must follow ROS Odometry convention.
# PX4-ROS2 bridge

[![GitHub license](https://img.shields.io/github/license/PX4/px4_ros_com.svg)](https://github.com/PX4/px4_ros_com/blob/master/LICENSE) [![GitHub (pre-)release](https://img.shields.io/github/release-pre/PX4/px4_ros_com.svg)](https://github.com/PX4/px4_ros_com/releases/tag/beta) [![DOI](https://zenodo.org/badge/142936318.svg)](https://zenodo.org/badge/latestdoi/142936318) [![Build and Test package](https://github.com/PX4/px4_ros_com/workflows/Build%20and%20Test%20package/badge.svg?branch=master)](https://github.com/PX4/px4_ros_com/actions)

This package materializes the ROS2 side of PX4 DDS bridge, establishing a bridge between the PX4 autopilot stack through an XRCE-DDS bridge. It has a straight dependency on the [`px4_msgs`](https://github.com/PX4/px4_msgs) package.

## Install, build and usage

Check the [ROS2 Interface](https://dev.px4.io/en/middleware/micrortps.html) section on the PX4 Devguide for details on how to install the required dependencies, build the package (composed by the two branches) and use it.

## Bug tracking and feature requests

Use the [Issues](https://github.com/PX4/px4_ros_com/issues) section to create a new issue. Report your issue or feature request [here](https://github.com/PX4/px4_ros_com/issues/new).

## Questions and troubleshooting

Reach the PX4 development team on the `#messaging` or `#ros` PX4 Slack channels:
[![Slack](https://px4-slack.herokuapp.com/badge.svg)](http://slack.px4.io)
