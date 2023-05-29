#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ns = LaunchConfiguration('ns')
    ns_launch_arg = DeclareLaunchArgument(
        'ns',
        default_value=''
    )

    odom_frame = LaunchConfiguration('odom_frame')
    odom_frame_launch_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom'
    )

    base_link = LaunchConfiguration('base_link')
    base_link_launch_arg = DeclareLaunchArgument(
        'base_link',
        default_value='px4_base_link'
    )

    tf_period = LaunchConfiguration('tf_period')
    tf_period_launch_arg = DeclareLaunchArgument(
        'tf_period',
        default_value='0.02'
    )
    
    publish_tf = LaunchConfiguration('publish_tf')
    publish_tf_launch_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='True'
    )

    vio_topic = LaunchConfiguration('vio_topic')
    vio_topic_launch_arg = DeclareLaunchArgument(
        'vio_topic',
        default_value='/fmu/in/vehicle_visual_odometry'
    )

    ## TODO Need to add parameter to enable/disable tf
    ## This is important to avoid conflicts with the TF published by the SLAM system
    ##  
    px4_ros_node = Node(
        package='px4_ros_com',
        executable='px4_ros',
        output='screen',
        name='px4_ros_com',
        namespace=ns,
        parameters=[{'publish_tf': publish_tf},
                    {'tf_pub_period': tf_period}, 
                    {'odom_frame': odom_frame},
                    {'baselink_frame': base_link}
                    ],
        remappings=[('vio/ros_odom', vio_topic)]
    )
    
    ld = LaunchDescription()
    ld.add_action(ns_launch_arg)
    ld.add_action(odom_frame_launch_arg)
    ld.add_action(base_link_launch_arg)
    ld.add_action(tf_period_launch_arg)
    ld.add_action(publish_tf_launch_arg)
    ld.add_action(px4_ros_node)
    ld.add_action(vio_topic_launch_arg)

    return ld