#!/bin/bash
# This script creates a new, standalone package 'piper_servo_test'
# to test moveit_servo without modifying any existing packages.
# Run it from your workspace 'src' directory (e.g., ~/piper_ros/src)

set -e # Exit immediately if a command fails

PKG_NAME="piper_servo_test"
PKG_PYTHON_DIR="$PKG_NAME/$PKG_NAME"
PKG_LAUNCH_DIR="$PKG_NAME/launch"
PKG_CONFIG_DIR="$PKG_NAME/config"

echo "--- [1/10] Creating package '$PKG_NAME' ---"
ros2 pkg create $PKG_NAME \
  --build-type ament_python \
  --dependencies rclpy geometry_msgs launch_ros moveit_configs_utils

echo "--- [2/10] Creating Python, launch, and config directories ---"
mkdir -p $PKG_PYTHON_DIR
mkdir -p $PKG_LAUNCH_DIR
mkdir -p $PKG_CONFIG_DIR

# ==============================================================================
# 3. Write package.xml
# ==============================================================================
echo "--- [3/10] Writing $PKG_NAME/package.xml ---"
cat << 'EOF' > $PKG_NAME/package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>piper_servo_test</name>
  <version>0.1.0</version>
  <description>Test package for moveit_servo on the Piper arm.</description>
  <maintainer email="harshchhajed30@gmail.com">harshrocks</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>launch_ros</depend>
  <depend>moveit_configs_utils</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# ==============================================================================
# 4. Write setup.py
# ==============================================================================
echo "--- [4/10] Writing $PKG_NAME/setup.py ---"
cat << 'EOF' > $PKG_NAME/setup.py
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'piper_servo_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harshrocks',
    maintainer_email='harshchhajed30@gmail.com',
    description='Test package for moveit_servo on the Piper arm.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop_node = piper_servo_test.keyboard_teleop_node:main',
        ],
    },
)
EOF

# ==============================================================================
# 5. Write the MODIFIED ros2_control.xacro
#    This adds the <command_interface name="velocity"/> to all 7 joints
# ==============================================================================
echo "--- [5/10] Writing $PKG_CONFIG_DIR/piper.ros2_control.xacro ---"
cat << 'EOF' > $PKG_CONFIG_DIR/piper.ros2_control.xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- This is a MODIFIED file for servo testing -->
    <!-- It adds the 'velocity' command_interface to all joints -->
    <xacro:macro name="piper_ros2_control" params="name initial_positions_file">
        <xacro:property name="initial_positions" value="${load_yaml(initial_positions_file)['initial_positions']}"/>

        <ros2_control name="${name}" type="system">
            <hardware>
                <plugin>mock_components/GenericSystem</plugin>
            </hardware>
            <joint name="joint1">
                <command_interface name="position"/>
                <command_interface name="velocity"/> <!-- ADDED FOR SERVO -->
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint1']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <joint name="joint2">
                <command_interface name="position"/>
                <command_interface name="velocity"/> <!-- ADDED FOR SERVO -->
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint2']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <joint name="joint3">
                <command_interface name="position"/>
                <command_interface name="velocity"/> <!-- ADDED FOR SERVO -->
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint3']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <joint name="joint4">
                <command_interface name="position"/>
                <command_interface name="velocity"/> <!-- ADDED FOR SERVO -->
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint4']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <joint name="joint5">
                <command_interface name="position"/>
                <command_interface name="velocity"/> <!-- ADDED FOR SERVO -->
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint5']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <joint name="joint6">
                <command_interface name="position"/>
                <command_interface name="velocity"/> <!-- ADDED FOR SERVO -->
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint6']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <joint name="joint7">
                <command_interface name="position"/>
                <command_interface name="velocity"/> <!-- ADDED FOR SERVO -->
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint7']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>

        </ros2_control>
    </xacro:macro>
</robot>
EOF

# ==============================================================================
# 6. Write the MODIFIED ros2_controllers.yaml
#    This adds the 'arm_velocity_controller'
# ==============================================================================
echo "--- [6/10] Writing $PKG_CONFIG_DIR/ros2_controllers.yaml ---"
cat << 'EOF' > $PKG_CONFIG_DIR/ros2_controllers.yaml
# This is a MODIFIED file for servo testing
# It adds 'arm_velocity_controller'
controller_manager:
  ros__parameters:
    update_rate: 500

    # Original controller for MoveIt planning
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    # Original gripper controller
    gripper_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    # NEW controller for MoveIt Servo
    arm_velocity_controller:
      type: joint_group_velocity_controller/JointGroupVelocityController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

# --- Original Controller Params ---
arm_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
    command_interfaces: [position]
    state_interfaces: [position, velocity]

gripper_controller:
  ros__parameters:
    joints: [joint7]
    command_interfaces: [position]
    state_interfaces: [position, velocity]

# --- NEW Controller Params ---
arm_velocity_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
    command_interfaces: [velocity] # This controller uses the velocity interface
    state_interfaces: [position, velocity]

joint_state_broadcaster:
  ros__parameters:
    publish_topic: "/joint_states"
EOF

# ==============================================================================
# 7. Write the NEW servo.yaml
#    This configures the moveit_servo node
# ==============================================================================
echo "--- [7/10] Writing $PKG_CONFIG_DIR/servo.yaml ---"
cat << 'EOF' > $PKG_CONFIG_DIR/servo.yaml
# This is the main configuration file for the moveit_servo node
servo_node:
  ros__parameters:
    # Input Topics (what we publish to)
    cartesian_command_in_topic: /servo_node/twist_stamped
    joint_command_in_topic: /servo_node/joint_jog
    pose_command_in_topic: /servo_node/pose_target # This is our "position stamp" topic

    # Output Controller
    # This MUST match the name in ros2_controllers.yaml
    controller_names: ["arm_velocity_controller"]
    command_out_type: "velocity"

    # Frames (from your piper.srdf)
    robot_link_command_frame: "base_link"
    end_effector_frame: "link6"

    # Planning Group (from your piper.srdf)
    planning_group_name: "arm"

    # General Settings
    publish_period: 0.02 # 50 Hz
    low_pass_filter_coeff: 2.0
    
    # Safety Limits (from your pilz_cartesian_limits.yaml)
    max_linear_velocity: 1.0
    max_rotational_velocity: 1.57
    
    # Start at 10% speed for safety
    linear_scale: 0.1
    rotational_scale: 0.1
EOF

# ==============================================================================
# 8. Write the NEW servo_demo.launch.py (*** THIS IS THE FIXED PART ***)
#    This manually launches the servo_node
# ==============================================================================
echo "--- [8/10] Writing $PKG_LAUNCH_DIR/servo_demo.launch.py ---"
cat << 'EOF' > $PKG_LAUNCH_DIR/servo_demo.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
# --- MODIFIED IMPORT ---
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    
    # Get the path to this package
    pkg_piper_servo_test = get_package_share_directory("piper_servo_test")
    
    # --- Define Paths to our NEW/MODIFIED Configs ---
    # We will tell MoveItConfigsBuilder to use THESE files instead of the default ones
    
    # Path to our modified xacro (with velocity interfaces)
    robot_description_xacro_path = os.path.join(
        pkg_piper_servo_test, "config", "piper.ros2_control.xacro"
    )

    # Path to our modified controllers (with velocity controller)
    ros2_controllers_path = os.path.join(
        pkg_piper_servo_test, "config", "ros2_controllers.yaml"
    )

    # --- Build the MoveIt Configuration ---
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="piper", 
            package_name="piper_with_gripper_moveit" # Use the ORIGINAL package for most files
        )
        .robot_description(
            # CRITICAL: Override the robot description to use our modified xacro
            file_path=robot_description_xacro_path
        )
        .ros2_controllers(
            # CRITICAL: Override the controllers to use our modified yaml
            file_path=ros2_controllers_path
        )
        # All other files (SRDF, kinematics.yaml, etc.)
        # will be loaded from the original 'piper_with_gripper_moveit' package
        .to_moveit_configs()
    )

    # 1. Generate the standard demo launch (RViz, MoveGroup, ros2_control)
    # This will now load our OVERRIDDEN config files
    demo_launch = generate_demo_launch(moveit_config)

    # 2. --- MANUALLY DEFINE THE SERVO NODE ---
    # This replaces the 'generate_servo_launch' function which doesn't exist in Humble
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            moveit_config.servo_config,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_scene_monitor,
        ],
        output="screen",
    )

    # 3. Return a LaunchDescription containing both
    return LaunchDescription(
        [
            demo_launch,
            servo_node  # Add the manually defined node
        ]
    )
EOF

# ==============================================================================
# 9. Write the NEW keyboard_teleop_node.py
#    This is the Python node for testing the "position" interface
# ==============================================================================
echo "--- [9/10] Writing $PKG_PYTHON_DIR/keyboard_teleop_node.py ---"
cat << 'EOF' > $PKG_PYTHON_DIR/keyboard_teleop_node.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import termios, sys, tty

# Define the key bindings
key_bindings = {
    'i': ( 0.01,  0.0,  0.0), # Move +X (forward)
    'k': (-0.01,  0.0,  0.0), # Move -X (backward)
    'j': ( 0.0,  0.01,  0.0), # Move +Y (left)
    'l': ( 0.0, -0.01,  0.0), # Move -Y (right)
    'u': ( 0.0,  0.0,  0.01), # Move +Z (up)
    'm': ( 0.0,  0.0, -0.01), # Move -Z (down)
}

# Simple instruction message
instructions = """
Reading from keyboard
---------------------------
i/k : Move Forward/Backward (+/- X)
j/l : Move Left/Right (+/- Y)
u/m : Move Up/Down (+/- Z)

q : Quit

Press a key to start publishing.
"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        self.get_logger().info(instructions)

        # Create publisher for the "position stamp"
        self.pose_pub = self.create_publisher(
            PoseStamped, '/servo_node/pose_target', 10
        )
        
        # Create subscriber for the "current pose" feedback
        self.pose_sub = self.create_subscription(
            PoseStamped, '/servo_node/pose_stamped_current', self.pose_callback, 10
        )

        self.target_pose = None
        self.last_target_pose = None
        self.has_received_pose = False

        # Create a timer to publish the target pose at a steady rate
        self.publish_timer = self.create_timer(0.02, self.publish_loop) # 50 Hz

    def pose_callback(self, msg: PoseStamped):
        # On the first message, initialize our target pose
        if not self.has_received_pose:
            self.target_pose = msg
            self.last_target_pose = msg
            self.has_received_pose = True
            self.get_logger().info("Received initial pose. Ready for teleop.")

    def publish_loop(self):
        # Don't publish until we have a pose
        if not self.has_received_pose:
            return

        # Republish the target pose at a steady rate
        # moveit_servo needs a constant stream of commands
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.target_pose)

    def run_teleop(self, settings):
        while rclpy.ok():
            key = get_key(settings)
            
            if not self.has_received_pose:
                self.get_logger().warn("No pose received yet. Waiting...")
                continue

            if key == 'q':
                self.get_logger().info("Quitting...")
                break
            
            if key in key_bindings:
                delta = key_bindings[key]
                self.target_pose.pose.position.x += delta[0]
                self.target_pose.pose.position.y += delta[1]
                self.target_pose.pose.position.z += delta[2]
                
                # Log the new target
                pos = self.target_pose.pose.position
                self.get_logger().info(f"New Target: X={pos.x:.3f} Y={pos.y:.3f} Z={pos.z:.3f}")

            else:
                pass # Ignore other keys

def main(args=None):
    rclpy.init(args=args)
    settings = termios.tcgetattr(sys.stdin)
    
    node = KeyboardTeleopNode()
    
    try:
        # We run the teleop key-checking in the main thread
        node.run_teleop(settings)
    except Exception as e:
        print(f"Error in teleop loop: {e}")
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
EOF

# ==============================================================================
# 10. Write the teleop launch file and make scripts executable
# ==============================================================================
echo "--- [10/10] Writing launch files and finishing ---"
touch $PKG_PYTHON_DIR/__init__.py
chmod +x $PKG_PYTHON_DIR/keyboard_teleop_node.py

# Write the teleop launch file
cat << 'EOF' > $PKG_LAUNCH_DIR/keyboard_teleop.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # This node needs to be launched in a terminal that can read keystrokes
    # Use 'prefix="xterm -e"' to launch in a new terminal
    teleop_node = Node(
        package='piper_servo_test',
        executable='keyboard_teleop_node.py',
        name='keyboard_teleop_node',
        output='screen',
        prefix='xterm -e' 
    )

    return LaunchDescription([
        teleop_node
    ])
EOF

echo "---"
echo "--- Successfully created package '$PKG_NAME' ---"
echo "---"
echo "--- NEXT STEPS: ---"
echo "1. Run 'colcon build --packages-select piper_servo_test'"
echo "2. Run 'source install/setup.bash'"
echo "3. In one terminal, launch the servo: 'ros2 launch piper_servo_test servo_demo.launch.py'"
echo "4. In another terminal, launch the teleop: 'ros2 launch piper_servo_test keyboard_teleop.launch.py'"
echo "---"


