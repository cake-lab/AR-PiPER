import os
from ament_index_python.packages import get_package_share_directory
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
        default_value="gripper_base",  # <-- FIX: Changed 'tool0' to 'gripper_base'
        description="End-effector link of the robot for TF lookup.",
    )
    duration_arg = DeclareLaunchArgument(
        "evaluation_duration_s",
        default_value="60.0",
        description="Duration of the evaluation run in seconds.",
    )

    # --- Node 1: piper_ee_pose_publisher ---
    # This node is from the 'piper_ik_to_controller' package (Phase 2)
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

    # --- Node 2: piper_evaluator_node ---
    # This is the new, independent evaluator node
    evaluator_node = Node(
        package="piper_evaluation",
        executable="piper_evaluator_node",
        name="piper_evaluator_node",
        output="screen",
        parameters=[
            {
                "evaluation_duration_s": LaunchConfiguration("evaluation_duration_s"),
                "sync_slop_s": 0.15,
                # Topic names are set by default in the node,
                # but you could override them here if needed:
                # 'teleop_metrics_topic': '/piper/teleop_metrics_raw',
                # 'kinematics_pose_topic': '/piper/ee_pose'
            }
        ],
    )

    return LaunchDescription(
        [
            base_frame_arg,
            ee_link_arg,
            duration_arg,
            ee_pose_publisher_node,
            evaluator_node,
        ]
    )
