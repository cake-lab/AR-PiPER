import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Get Package Paths ---
    pkg_piper_ik = get_package_share_directory("piper_ik_to_controller")
    pkg_piper_description = get_package_share_directory("piper_description")

    # --- Load Configuration and Robot Model ---
    # We use the same config file because the parameters (gains, limits) are shared
    params_file = os.path.join(pkg_piper_ik, "config", "piper_ik.yaml")
    urdf_path = os.path.join(pkg_piper_description, "urdf", "piper_description.xacro")
    robot_description = Command(["xacro ", urdf_path])

    # --- Define Nodes ---
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Pose Filter Node (Unchanged)
    # Subscribes to "/target_pose_raw" -> Publishes "/target_pose"
    pose_filter_node = Node(
        package="piper_ik_to_controller",
        executable="pose_filter_node",
        name="pose_filter_node",
        output="screen",
        parameters=[{"filter_type": "kalman"}],
    )

    # --- UPDATED: Position Controller Node ---
    # Executable changed to 'piper_ik_to_controller_position'
    piper_ik_node = Node(
        package="piper_ik_to_controller",
        executable="piper_ik_to_controller_position",  # <--- CHANGED HERE
        name="piper_ik_to_controller",
        output="screen",
        parameters=[params_file, {"robot_description": robot_description}],
    )

    # --- Create and Return the Launch Description ---
    return LaunchDescription(
        [
            robot_state_publisher_node,
            pose_filter_node,
            piper_ik_node,
        ]
    )
