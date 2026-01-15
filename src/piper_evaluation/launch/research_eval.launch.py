import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Launch arguments
    base_frame_arg = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="Base frame of the robot for TF lookup.",
    )
    ee_link_arg = DeclareLaunchArgument(
        "ee_link",
        default_value="gripper_base",
        description="End-effector link of the robot for TF lookup.",
    )
    duration_arg = DeclareLaunchArgument(
        "evaluation_duration_s",
        default_value="30.0",
        description="Duration of the evaluation run in seconds.",
    )

    # --- Node 1: piper_ee_pose_publisher ---
    # Calculates the FK pose
    ee_pose_publisher_node = Node(
        package="piper_ik_to_controller",
        executable="piper_ee_pose_publisher",
        name="piper_ee_pose_publisher",
        parameters=[
            {
                "base_frame": LaunchConfiguration("base_frame"),
                "ee_link": LaunchConfiguration("ee_link"),
                "rate_hz": 100.0,
            }
        ],
    )

    # --- Node 2: piper_research_logger_node ---
    # The new V3 Research Grade Logger
    logger_node = Node(
        package="piper_evaluation",
        executable="piper_research_logger_node",
        name="piper_research_logger_node",
        output="screen",
        parameters=[
            {
                "evaluation_duration_s": LaunchConfiguration("evaluation_duration_s"),
                "sync_slop_s": 0.15,
                # 'teleop_metrics_topic': '/piper/teleop_metrics_raw',
                # 'kinematics_pose_topic': '/piper/ee_pose'
            }
        ],
    )

    return LaunchDescription(
        [base_frame_arg, ee_link_arg, duration_arg, ee_pose_publisher_node, logger_node]
    )
