# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration


# def generate_launch_description():

#     # --- Arguments ---
#     user_id_arg = DeclareLaunchArgument(
#         "user_id", default_value="U0", description="User ID folder name (e.g., U1)"
#     )

#     target_file_arg = DeclareLaunchArgument(
#         "target_file",
#         default_value="H1.csv",
#         description="The specific human CSV file to replay",
#     )

#     base_frame_arg = DeclareLaunchArgument(
#         "base_frame", default_value="base_link", description="Base frame of the robot"
#     )
#     ee_link_arg = DeclareLaunchArgument(
#         "ee_link",
#         default_value="gripper_base",
#         description="End-effector frame to track",
#     )

#     # --- Node 1: The Smart Recorder (Evaluator) ---
#     # This replaces the old 'piper_recorder_node' and 'websocket_bridge'
#     # It listens to the player's triggers and logs the data.
#     evaluator_node = Node(
#         package="piper_evaluation",
#         executable="piper_evaluator_node",
#         name="piper_evaluator_node",
#         output="screen",
#         parameters=[
#             {
#                 "user_id": LaunchConfiguration("user_id"),
#                 "data_root": "/home/harshrocks/eval/piper/userstudy",
#                 "sync_slop_s": 0.1,
#             }
#         ],
#     )

#     # --- Node 2: The Conductor (Player) ---
#     # Loads the CSV, resets the robot, and plays the trajectory.
#     player_node = Node(
#         package="piper_evaluation",
#         executable="piper_player_node",
#         name="player",
#         output="screen",
#         parameters=[
#             {
#                 "user_id": LaunchConfiguration("user_id"),
#                 "target_file": LaunchConfiguration("target_file"),
#                 "data_root": "/home/harshrocks/eval/piper/userstudy",
#                 "repeat_count": 5,
#                 "interval_seconds": 10.0,
#             }
#         ],
#     )

#     # --- Node 3: The Observer (Pose Publisher) ---
#     # Necessary for calculating Kinematic Pose from TF
#     ee_pose_publisher_node = Node(
#         package="piper_ik_to_controller",
#         executable="piper_ee_pose_publisher",
#         name="piper_ee_pose_publisher",
#         parameters=[
#             {
#                 "base_frame": LaunchConfiguration("base_frame"),
#                 "ee_link": LaunchConfiguration("ee_link"),
#                 "rate_hz": 100.0,
#             }
#         ],
#     )

#     return LaunchDescription(
#         [
#             user_id_arg,
#             target_file_arg,
#             base_frame_arg,
#             ee_link_arg,
#             evaluator_node,
#             player_node,
#             ee_pose_publisher_node,
#         ]
#     )


import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # --- Arguments ---
    user_id_arg = DeclareLaunchArgument(
        "user_id", default_value="U3", description="User ID folder name (e.g., U1)"
    )

    # NOTE: Ensure this file exists in /home/harshrocks/eval/piper/userstudy/<user_id>/
    target_file_arg = DeclareLaunchArgument(
        "target_file",
        default_value="H1.csv",  # Defaulting to likely output of Phase 1
        description="The specific human CSV file to replay",
    )

    base_frame_arg = DeclareLaunchArgument(
        "base_frame", default_value="base_link", description="Base frame of the robot"
    )
    ee_link_arg = DeclareLaunchArgument(
        "ee_link",
        default_value="gripper_base",
        description="End-effector frame to track",
    )

    # --- Node 1: The Recorder (User Study Logic) ---
    # FIX: Switched from 'piper_evaluator_node' (Research V3) to 'piper_recorder_node'
    # This node listens for "START:ROBOT" from the player and saves 'robot_1.csv'.
    recorder_node = Node(
        package="piper_evaluation",
        executable="piper_recorder_node",  # <--- CRITICAL FIX
        name="piper_recorder_node",
        output="screen",
        parameters=[
            {
                "user_id": LaunchConfiguration("user_id"),
                "data_root": "/home/harshrocks/eval/piper/userstudy",
                "sync_slop_s": 0.1,
            }
        ],
    )

    # --- Node 2: The Player (Conductor) ---
    # Plays the CSV 5 times and sends "START:ROBOT" triggers.
    player_node = Node(
        package="piper_evaluation",
        executable="piper_player_node",
        name="player",
        output="screen",
        parameters=[
            {
                "user_id": LaunchConfiguration("user_id"),
                "target_file": LaunchConfiguration("target_file"),
                "data_root": "/home/harshrocks/eval/piper/userstudy",
                "repeat_count": 5,
                "interval_seconds": 5.0,
            }
        ],
    )

    # --- Node 3: Pose Publisher (The Observer) ---
    # Publishes /piper/ee_pose from TF for the recorder to capture actual robot state.
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

    return LaunchDescription(
        [
            user_id_arg,
            target_file_arg,
            base_frame_arg,
            ee_link_arg,
            recorder_node,
            player_node,
            ee_pose_publisher_node,
        ]
    )
