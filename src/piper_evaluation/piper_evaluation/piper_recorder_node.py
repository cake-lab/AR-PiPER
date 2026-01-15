#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import time
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import message_filters


class PiperEvaluatorNode(Node):
    def __init__(self):
        super().__init__("piper_evaluator_node")

        # --- Parameters ---
        self.declare_parameter("user_id", "TEST")
        self.declare_parameter("data_root", "piper/userstudy")
        self.declare_parameter("sync_slop_s", 0.1)  # 100ms sync window

        self.user_id = self.get_parameter("user_id").get_parameter_value().string_value
        self.data_root = (
            self.get_parameter("data_root").get_parameter_value().string_value
        )
        self.slop = self.get_parameter("sync_slop_s").get_parameter_value().double_value

        # --- Folder Setup ---
        self.save_dir = os.path.join(self.data_root, self.user_id)
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f"Evaluator ready. Saving to: {self.save_dir}")

        # --- State ---
        self._is_recording = False
        self._csv_buffer = []
        self._lock = threading.Lock()
        self._current_filename = None
        self._last_data_time = 0.0

        # --- Subscribers ---
        # 1. Trigger Listener
        self.create_subscription(
            String, "/piper/experiment_trigger", self.trigger_callback, 10
        )

        # 2. Synchronized Data Recording
        # We need both TARGET (from Player) and ACTUAL (from Robot Driver/TF)
        self.target_sub = message_filters.Subscriber(
            self, PoseStamped, "/target_pose_raw"
        )
        self.actual_sub = message_filters.Subscriber(
            self, PoseStamped, "/piper/ee_pose"
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.target_sub, self.actual_sub], queue_size=50, slop=self.slop
        )
        self.ts.registerCallback(self.data_callback)

        # --- Diagnostics ---
        self.create_timer(2.0, self.check_health)

    def trigger_callback(self, msg):
        command = msg.data.upper()

        if command.startswith("START"):
            parts = command.split(":")
            prefix = parts[1].lower() if len(parts) > 1 else "data"
            self.start_new_session(prefix)

        elif command == "STOP":
            self.stop_current_session()

    def start_new_session(self, prefix):
        with self._lock:
            # Auto-increment filename
            i = 1
            while True:
                fname = f"{prefix}_{i}.csv"
                if not os.path.exists(os.path.join(self.save_dir, fname)):
                    self._current_filename = fname
                    break
                i += 1

            self._csv_buffer = []
            self._is_recording = True
            self._last_data_time = time.time()
            self.get_logger().info(
                f"--- STARTED RECORDING SESSION: {self._current_filename} ---"
            )

    def stop_current_session(self):
        with self._lock:
            if not self._is_recording:
                return

            self._is_recording = False
            self.get_logger().info(
                f"--- STOP TRIGGER RECEIVED. Buffer size: {len(self._csv_buffer)} ---"
            )
            self.save_buffer_to_disk()

    def check_health(self):
        if self._is_recording:
            time_since_last = time.time() - self._last_data_time
            if time_since_last > 2.0:
                self.get_logger().warn(
                    f"RECORDING ACTIVE BUT NO DATA SYNCED for {time_since_last:.1f}s! "
                    "Check if Robot Driver is running (ros2 launch piper_bringup ...)."
                )

    def data_callback(self, target_msg, actual_msg):
        if not self._is_recording:
            return

        with self._lock:
            self._last_data_time = time.time()

            # --- Sync Delta Calculation ---
            t_target_ns = (
                target_msg.header.stamp.sec * 1e9 + target_msg.header.stamp.nanosec
            )
            t_actual_ns = (
                actual_msg.header.stamp.sec * 1e9 + actual_msg.header.stamp.nanosec
            )

            # How far apart are the timestamps? (ms)
            sync_delta = (t_actual_ns - t_target_ns) / 1e6

            row = {
                "capture_timestamp": self.get_clock().now().nanoseconds / 1e9,
                # Target
                "target_pos_x": target_msg.pose.position.x,
                "target_pos_y": target_msg.pose.position.y,
                "target_pos_z": target_msg.pose.position.z,
                "target_orient_x": target_msg.pose.orientation.x,
                "target_orient_y": target_msg.pose.orientation.y,
                "target_orient_z": target_msg.pose.orientation.z,
                "target_orient_w": target_msg.pose.orientation.w,
                # Kinematic
                "kinematic_pos_x": actual_msg.pose.position.x,
                "kinematic_pos_y": actual_msg.pose.position.y,
                "kinematic_pos_z": actual_msg.pose.position.z,
                "kinematic_orient_x": actual_msg.pose.orientation.x,
                "kinematic_orient_y": actual_msg.pose.orientation.y,
                "kinematic_orient_z": actual_msg.pose.orientation.z,
                "kinematic_orient_w": actual_msg.pose.orientation.w,
                # Metrics
                "ik_solver_delay_ms": 0.0,  # Not applicable in replay
                "e2e_latency_at_control_ms": 0.0,  # Not applicable in replay
                "e2e_latency_at_eval_ms": 0.0,  # Not applicable in replay
                "sync_delta_ms": sync_delta,  # <--- This will now show up
            }
            self._csv_buffer.append(row)

    def save_buffer_to_disk(self):
        if not self._current_filename:
            return

        if not self._csv_buffer:
            self.get_logger().warn(
                f"Buffer EMPTY for {self._current_filename}. NO CSV CREATED."
            )
            return

        full_path = os.path.join(self.save_dir, self._current_filename)
        keys = self._csv_buffer[0].keys()

        try:
            with open(full_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                writer.writerows(self._csv_buffer)
            self.get_logger().info(
                f"SUCCESS: Saved {len(self._csv_buffer)} rows to {full_path}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to save CSV: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PiperEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_current_session()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
