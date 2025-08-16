import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions.node import Node
from launch.actions import ExecuteProcess
from ament_index_python import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # Pixhawk 1 IMU link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.1355", "0", "0.095", "1.57079632679", "0", "0", "base_link", "pixhawk_1_imu"]
        ))
    
    # Pixhawk 1 GPS link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.1355", "0", "0.1065", "1.57079632679", "0", "3.14159265359", "base_link", "pixhawk_1_gps"]
        ))

    # Front Camera Link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.246", "0", "-0.005", "-3.14159265359", "-1.57079632679", "0", "front_base_link", "front"]
        ))
    
    # Perception Front Camera Link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.0", "0.0", "0.44", "-3.14159265359", "-1.57079632679", "0", "front_base_link", "perception_front"]
        ))

    # Perception Right Side Camera Link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.0", "0.0", "0.44", "0", "-1.57079632679", "1.57079632679", "front_base_link", "perception_right"]
        ))

    # Perception Left Side Camera Link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.0", "0.0", "0.44", "-1.57079632679", "-1.57079632679", "0", "front_base_link", "perception_left"]
        ))
    
    # Perception Back Camera Link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.0", "0.0", "0.44", "0.0", "-1.57079632679", "0.0", "front_base_link", "perception_back"]
        ))

    # Down Camera Link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.130", "0", "-0.130", "0", "1.57079632679", "0", "front_base_link", "down"]
        ))

    # Left Internal Camera Link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.132", "0.246", "0.005", "0", "-1.57079632679", "-2.35619449019", "base_link", "left_internal"]
        ))

    # Right Internal Camera Link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.132", "-0.246", "0.005", "0", "-1.57079632679", "-3.92699081699 ", "base_link", "right_internal"]
        ))

    # Back Camera Link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["-0.173", "0", "-0.127", "0", "1.57079632679", "3.14159265359", "base_link", "back"]
        ))

    # Fum Unda Camera Link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["-0.061", "0", "-0.127", "0", "1.57079632679", "0", "base_link", "camera"]
        ))

    # Front Base Link to Base Link (Static for now)
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.595", "0", "0", "0", "0", "0", "base_link", "front_base_link"]
        ))
        
    # Test Addition to Front Base Link
    # ld.add_action(
    #     Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         output="screen",
    #         # X, Y, Z, Roll, Pitch, Yaw
    #         arguments=["0", "0", "-1", "0", "0", "0", "front_base_link", "test"]
    #     ))
    
    # ld.add_action(
    #     Node(
    #         package="tf2_ros",
    #         executable="joint_state_publisher",
    #         output="screen",
    #         parameters=[
    #             {
    #                 # Each string in this array represents a topic name.
    #                 # For each string, create a subscription to the named topic of type sensor_msgs/msg/JointStates.
    #                 # publication to that topic will update the joints named in the message.
    #                 "source_list": ["base_link_front", "base_link_rear"],
    #                 # How much to automatically move joints during each iteration. Defaults to 0.0 of ommited.
    #                 "delta": 0,

    #             }
    #         ],
    #     ))
    return ld
