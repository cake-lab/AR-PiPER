#!/bin/bash
# This script creates the independent 'piper_evaluation' package
# Run it from your 'src' directory (e.g., ~/piper_ros/src)

set -e # Exit immediately if a command fails

PKG_NAME="piper_evaluation"
PKG_PYTHON_DIR="$PKG_NAME/$PKG_NAME"
PKG_LAUNCH_DIR="$PKG_NAME/launch"

# 1. Create the package with dependencies
echo "--- Creating package '$PKG_NAME' ---"
# --- FIX: Moved $PKG_NAME (positional arg) before --dependencies ---
ros2 pkg create --build-type ament_python \
  $PKG_NAME \
  --dependencies rclpy geometry_msgs std_msgs message_filters piper_eval_msgs

# 2. Create the inner Python module directory
echo "--- Creating Python module directory ---"
mkdir -p $PKG_PYTHON_DIR

# 3. Create the launch directory
echo "--- Creating launch directory ---"
mkdir -p $PKG_LAUNCH_DIR

# 4. Write package.xml
# (Overwrites the one created by ros2 pkg create to ensure it's correct)
echo "--- Writing package.xml ---"
cat << 'EOF' > $PKG_NAME/package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>piper_evaluation</name>
  <version>0.1.0</version>
  <description>Independent evaluation node for the Piper robot.</description>
  <maintainer email="harshchhajed30@gmail.com">harshrocks</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>message_filters</depend>
  <depend>piper_eval_msgs</depend> <!-- Depends on our internal msgs -->

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# 5. Write setup.py
echo "--- Writing setup.py ---"
cat << 'EOF' > $PKG_NAME/setup.py
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'piper_evaluation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harshrocks',
    maintainer_email='harshchhajed30@gmail.com',
    description='Independent evaluation node for the Piper robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'piper_evaluator_node = piper_evaluation.piper_evaluator_node:main',
        ],
    },
)
EOF

# 6. Write the __init__.py file
echo "--- Writing $PKG_PYTHON_DIR/__init__.py ---"
touch $PKG_PYTHON_DIR/__init__.py

# 7. Write the main evaluator node
echo "--- Writing $PKG_PYTHON_DIR/piper_evaluator_node.py ---"
cat << 'EOF' > $PKG_PYTHON_DIR/piper_evaluator_node.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This node is an adaptation of the 'master_evaluator_node.py' from the 
omx workspace, modified to be fully independent and work 
with Piper-specific topics and messages from the piper_ros workspace.
"""

import csv, os, atexit
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose, PoseStamped  # <-- CHANGED: Import PoseStamped
from piper_eval_msgs.msg import PiperTeleopMetric # <-- CHANGED: Import PiperTeleopMetric
import message_filters


def _shutdown_safely(node: Node):
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass


class PiperEvaluatorNode(Node):  # <-- CHANGED: Renamed class
    """
    Subscribes:
      - /piper/teleop_metrics_raw (PiperTeleopMetric: carries camera t0 + IK delay)
      - /piper/ee_pose (PoseStamped: actual EE pose from TF)

    Logs per synchronized sample:
      capture_timestamp (s since epoch),
      target pose (xyz qxqyqzqw),
      kinematic pose (xyz qxqyqzqw),
      ik_solver_delay_ms,
      e2e_latency_ms = (now - capture_timestamp) measured here.
    """

    def __init__(self):
        super().__init__('piper_evaluator_node') # <-- CHANGED: Renamed node

        # -------- Parameters --------
        # <-- CHANGED: Updated default topic names -->
        self.declare_parameter('teleop_metrics_topic', '/piper/teleop_metrics_raw')
        self.declare_parameter('kinematics_pose_topic', '/piper/ee_pose')
        self.declare_parameter('evaluation_duration_s', 60.0)
        self.declare_parameter('sync_slop_s', 0.15)
        self.declare_parameter('qos_depth', 50)

        self.teleop_topic = self.get_parameter('teleop_metrics_topic').value
        self.kin_topic = self.get_parameter('kinematics_pose_topic').value
        self.eval_duration = float(self.get_parameter('evaluation_duration_s').value)
        self.sync_slop = float(self.get_parameter('sync_slop_s').value)
        self.qos_depth = int(self.get_parameter('qos_depth').value)

        # -------- Output path: ~/eval/piper (or fallback) --------
        try:
            # <-- FIX: Changed output directory to ~/eval/piper -->
            self.output_dir = str(Path.home() / 'eval' / 'piper') 
            os.makedirs(self.output_dir, exist_ok=True)
        except Exception as e:
            # <-- FIX: Changed fallback directory -->
            self.get_logger().error(f"Failed to create {Path.home() / 'eval' / 'piper'}: {e}. Falling back to ~/.ros/piper_evals/piper")
            self.output_dir = str(Path.home() / '.ros' / 'piper_evals' / 'piper')
            os.makedirs(self.output_dir, exist_ok=True)

        ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        # <-- CHANGED: File prefix is now piper_run -->
        self.output_path = os.path.join(self.output_dir, f"piper_run_{ts}.csv")

        # -------- QoS --------
        qos = QoSProfile(depth=self.qos_depth)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        # -------- Subscribers + Synchronizer --------
        # <-- CHANGED: Use PiperTeleopMetric -->
        self.teleop_sub = message_filters.Subscriber(self, PiperTeleopMetric, self.teleop_topic, qos_profile=qos)
        # <-- CHANGED: Use PoseStamped (from Phase 2) -->
        self.kin_sub = message_filters.Subscriber(self, PoseStamped, self.kin_topic, qos_profile=qos)

        # We keep allow_headerless=False because both messages now have headers.
        # The synchronizer will use the stamps from both messages.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.teleop_sub, self.kin_sub],
            queue_size=self.qos_depth,
            slop=self.sync_slop
        )
        self.sync.registerCallback(self._synced_cb)

        # -------- Accumulators --------
        self.rows = []
        self.ik_delay_sum_ms = 0.0
        self.ik_delay_count = 0
        self.e2e_sum_ms = 0.0
        self.e2e_count = 0
        self._last_log_ns = 0
        self._first_sync_received = False # <-- ADDED: Diagnostic flag

        # Timed finish
        self._done = False
        self._finish_timer = self.create_timer(self.eval_duration, self._on_evaluation_done)

        # Always try to save on interpreter shutdown too
        atexit.register(self._save_if_needed)

        self.get_logger().info(
            "PiperEvaluatorNode started.\n"
            f"  teleop: {self.teleop_topic}\n"
            f"  kin:    {self.kin_topic}\n"
            f"  duration: {self.eval_duration:.1f}s  slop: {self.sync_slop:.3f}s\n"
            f"  writing to: {self.output_path}"
        )
        # <-- ADDED: Diagnostic log -->
        self.get_logger().info("Waiting for first synchronized message pair...")

    @staticmethod
    def _pose_to_list(p: Pose):
        return [float(p.position.x), float(p.position.y), float(p.position.z),
                float(p.orientation.x), float(p.orientation.y), float(p.orientation.z), float(p.orientation.w)]

    # <-- CHANGED: Updated type hints for new messages -->
    def _synced_cb(self, teleop_msg: PiperTeleopMetric, kin_msg: PoseStamped):
        # <-- ADDED: One-time diagnostic log -->
        if not self._first_sync_received:
            self.get_logger().info("First synchronized message pair received! Starting data logging.")
            self._first_sync_received = True

        # Capture timestamp from camera (via TeleopMetric.header.stamp)
        cap = teleop_msg.header.stamp
        capture_ts = float(cap.sec) + float(cap.nanosec) * 1e-9

        # E2E latency (ms): now - capture
        now_ns = self.get_clock().now().nanoseconds
        cap_ns = cap.sec * 1_000_000_000 + cap.nanosec
        e2e_ms = (now_ns - cap_ns) / 1e6
        if e2e_ms == e2e_ms:  # NaN-safe
            self.e2e_sum_ms += e2e_ms
            self.e2e_count += 1 # <-- FIX: Added 1
        
        tgt_list = self._pose_to_list(teleop_msg.target_pose)
        # <-- CHANGED: Extract pose from .pose attribute of PoseStamped -->
        kin_list = self._pose_to_list(kin_msg.pose) 

        # IK delay
        ik_ms = float(teleop_msg.ik_solver_delay_ms)
        if ik_ms == ik_ms:
            self.ik_delay_sum_ms += ik_ms
            self.ik_delay_count += 1

        # Append row
        row = [capture_ts] + tgt_list + kin_list + [ik_ms, e2e_ms]
        self.rows.append(row)

        # Throttled status
        if now_ns - self._last_log_ns > 1_000_000_000:
            avg_ik = (self.ik_delay_sum_ms / max(1, self.ik_delay_count))
            avg_e2e = (self.e2e_sum_ms / max(1, self.e2e_count))
            self.get_logger().info(f"E2E ~ {avg_e2e:.1f} ms | Avg IK ~ {avg_ik:.1f} ms")
            self._last_log_ns = now_ns

    def _on_evaluation_done(self):
        if self._done:
            return
        self._done = True
        self.get_logger().info("Evaluation timer finished. Saving and exiting.")
        self._save_and_exit()

    # ---------- Save helpers (unchanged logic) ----------
    def _csv_header(self):
        return [
            'capture_timestamp',
            'target_pos_x','target_pos_y','target_pos_z','target_orient_x','target_orient_y','target_orient_z','target_orient_w',
            'kinematic_pos_x','kinematic_pos_y','kinematic_pos_z','kinematic_orient_x','kinematic_orient_y','kinematic_orient_z','kinematic_orient_w',
            'ik_solver_delay_ms',
            'e2e_latency_ms'
        ]

    def _write_csv(self):
        if not self.rows:
            self.get_logger().warn("No data collected. Writing empty CSV.")
        
        try:
            with open(self.output_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self._csv_header())
                if self.rows:
                    writer.writerows(self.rows)
            self.get_logger().info(f"Saved {len(self.rows)} rows to {self.output_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV to {self.output_path}: {e}")

    def _save_if_needed(self):
        if not self._done:
            self._done = True
            self.get_logger().info("Shutdown hook triggered. Saving data...")
            self._write_csv()

    def _save_and_exit(self):
        if self._done:
            # Already saved, just shut down
            _shutdown_safely(self)
            return
        
        self._done = True
        self._write_csv()
        self.get_logger().info("Evaluation complete. Shutting down.")
        _shutdown_safely(self)


def main(args=None):
    rclpy.init(args=args)
    # <-- CHANGED: Instantiate new class name -->
    node = PiperEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received — saving and shutting down.")
        node._save_and_exit()
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
        # Ensure data is saved even on unexpected crash
        node._save_if_needed()
        _shutdown_safely(node)


if __name__ == '__main__':
    main()
EOF

# 8. Write the launch file
echo "--- Writing $PKG_LAUNCH_DIR/piper_evaluation.launch.py ---"
cat << 'EOF' > $PKG_LAUNCH_DIR/piper_evaluation.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Launch arguments
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame of the robot for TF lookup.'
    )
    ee_link_arg = DeclareLaunchArgument(
        'ee_link',
        default_value='gripper_base', # <-- FIX: Changed 'tool0' to 'gripper_base'
        description='End-effector link of the robot for TF lookup.'
    )
    duration_arg = DeclareLaunchArgument(
        'evaluation_duration_s',
        default_value='30.0',
        description='Duration of the evaluation run in seconds.'
    )

    # --- Node 1: piper_ee_pose_publisher ---
    # This node is from the 'piper_ik_to_controller' package (Phase 2)
    ee_pose_publisher_node = Node(
        package='piper_ik_to_controller',
        executable='piper_ee_pose_publisher',
        name='piper_ee_pose_publisher',
        parameters=[{
            'base_frame': LaunchConfiguration('base_frame'),
            'ee_link': LaunchConfiguration('ee_link'),
            'rate_hz': 100.0
        }]
    )

    # --- Node 2: piper_evaluator_node ---
    # This is the new, independent evaluator node
    evaluator_node = Node(
        package='piper_evaluation',
        executable='piper_evaluator_node',
        name='piper_evaluator_node',
        output='screen',
        parameters=[{
            'evaluation_duration_s': LaunchConfiguration('evaluation_duration_s'),
            'sync_slop_s': 0.15
            # Topic names are set by default in the node,
            # but you could override them here if needed:
            # 'teleop_metrics_topic': '/piper/teleop_metrics_raw',
            # 'kinematics_pose_topic': '/piper/ee_pose'
        }]
    )

    return LaunchDescription([
        base_frame_arg,
        ee_link_arg,
        duration_arg,
        ee_pose_publisher_node,
        evaluator_node
    ])
EOF

# 9. Make the python node executable
chmod +x $PKG_PYTHON_DIR/piper_evaluator_node.py

echo "--- Successfully created package '$PKG_NAME' ---"
echo "--- Review the files and then run 'colcon build' ---"


