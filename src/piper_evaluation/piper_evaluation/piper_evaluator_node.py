# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import csv
# import os
# import time
# import threading
# from pathlib import Path

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import String
# import message_filters


# class PiperEvaluatorNode(Node):
#     def __init__(self):
#         super().__init__("piper_evaluator_node")

#         # --- Parameters ---
#         self.declare_parameter("user_id", "U0")
#         self.declare_parameter("data_root", "piper/userstudy")
#         self.declare_parameter("sync_slop_s", 0.1)

#         self.user_id = self.get_parameter("user_id").get_parameter_value().string_value
#         self.data_root = (
#             self.get_parameter("data_root").get_parameter_value().string_value
#         )
#         self.slop = self.get_parameter("sync_slop_s").get_parameter_value().double_value

#         # --- Folder Setup ---
#         self.save_dir = os.path.join(self.data_root, self.user_id)
#         os.makedirs(self.save_dir, exist_ok=True)
#         self.get_logger().info(f"Evaluator ready. Saving to: {self.save_dir}")

#         # --- State ---
#         self._is_recording = False
#         self._csv_buffer = []
#         self._lock = threading.Lock()
#         self._current_filename = None
#         self._last_data_time = 0.0

#         # --- Subscribers ---
#         # 1. Trigger Listener
#         self.create_subscription(
#             String, "/piper/experiment_trigger", self.trigger_callback, 10
#         )

#         # 2. Synchronized Data Recording
#         self.target_sub = message_filters.Subscriber(
#             self, PoseStamped, "/target_pose_raw"
#         )
#         self.actual_sub = message_filters.Subscriber(
#             self, PoseStamped, "/piper/ee_pose"
#         )

#         self.ts = message_filters.ApproximateTimeSynchronizer(
#             [self.target_sub, self.actual_sub], queue_size=50, slop=self.slop
#         )
#         self.ts.registerCallback(self.data_callback)

#         # --- Diagnostics ---
#         self.create_timer(2.0, self.check_health)

#     def trigger_callback(self, msg):
#         command = msg.data.upper()

#         if command.startswith("START"):
#             parts = command.split(":")
#             prefix = parts[1].lower() if len(parts) > 1 else "data"
#             self.start_new_session(prefix)

#         elif command == "STOP":
#             self.stop_current_session()

#     def start_new_session(self, prefix):
#         with self._lock:
#             i = 1
#             while True:
#                 fname = f"{prefix}_{i}.csv"
#                 if not os.path.exists(os.path.join(self.save_dir, fname)):
#                     self._current_filename = fname
#                     break
#                 i += 1

#             self._csv_buffer = []
#             self._is_recording = True
#             self._last_data_time = time.time()
#             self.get_logger().info(
#                 f"--- STARTED RECORDING SESSION: {self._current_filename} ---"
#             )

#     def stop_current_session(self):
#         with self._lock:
#             if not self._is_recording:
#                 return

#             self._is_recording = False
#             self.get_logger().info(
#                 f"--- STOP TRIGGER RECEIVED. Buffer size: {len(self._csv_buffer)} ---"
#             )
#             self.save_buffer_to_disk()

#     def check_health(self):
#         if self._is_recording:
#             time_since_last = time.time() - self._last_data_time
#             if time_since_last > 2.0:
#                 self.get_logger().warn(
#                     f"RECORDING ACTIVE BUT NO DATA SYNCED for {time_since_last:.1f}s! "
#                     "Check if Robot Driver is running (ros2 launch piper_bringup ...)."
#                 )

#     def data_callback(self, target_msg, actual_msg):
#         if not self._is_recording:
#             return

#         with self._lock:
#             self._last_data_time = time.time()

#             t_target_ns = (
#                 target_msg.header.stamp.sec * 1e9 + target_msg.header.stamp.nanosec
#             )
#             t_actual_ns = (
#                 actual_msg.header.stamp.sec * 1e9 + actual_msg.header.stamp.nanosec
#             )
#             sync_delta = (t_actual_ns - t_target_ns) / 1e6

#             row = {
#                 "capture_timestamp": self.get_clock().now().nanoseconds / 1e9,
#                 "target_pos_x": target_msg.pose.position.x,
#                 "target_pos_y": target_msg.pose.position.y,
#                 "target_pos_z": target_msg.pose.position.z,
#                 "target_orient_x": target_msg.pose.orientation.x,
#                 "target_orient_y": target_msg.pose.orientation.y,
#                 "target_orient_z": target_msg.pose.orientation.z,
#                 "target_orient_w": target_msg.pose.orientation.w,
#                 "kinematic_pos_x": actual_msg.pose.position.x,
#                 "kinematic_pos_y": actual_msg.pose.position.y,
#                 "kinematic_pos_z": actual_msg.pose.position.z,
#                 "kinematic_orient_x": actual_msg.pose.orientation.x,
#                 "kinematic_orient_y": actual_msg.pose.orientation.y,
#                 "kinematic_orient_z": actual_msg.pose.orientation.z,
#                 "kinematic_orient_w": actual_msg.pose.orientation.w,
#                 "ik_solver_delay_ms": 0.0,
#                 "e2e_latency_at_control_ms": 0.0,
#                 "e2e_latency_at_eval_ms": 0.0,
#                 "sync_delta_ms": sync_delta,
#             }
#             self._csv_buffer.append(row)

#     def save_buffer_to_disk(self):
#         if not self._current_filename:
#             return
#         if not self._csv_buffer:
#             self.get_logger().warn(
#                 f"Buffer EMPTY for {self._current_filename}. NO CSV CREATED."
#             )
#             return

#         full_path = os.path.join(self.save_dir, self._current_filename)
#         keys = self._csv_buffer[0].keys()

#         try:
#             with open(full_path, "w", newline="") as f:
#                 writer = csv.DictWriter(f, fieldnames=keys)
#                 writer.writeheader()
#                 writer.writerows(self._csv_buffer)
#             self.get_logger().info(
#                 f"SUCCESS: Saved {len(self._csv_buffer)} rows to {full_path}"
#             )
#         except Exception as e:
#             self.get_logger().error(f"Failed to save CSV: {e}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = PiperEvaluatorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.stop_current_session()
#     finally:
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This node is an adaptation of the 'master_evaluator_node.py' from the
omx workspace, modified to be fully independent and work
with Piper-specific topics and messages from the piper_ros workspace.

--- V3 (Research Grade) ---
- Logs both controller-side and evaluator-side E2E latencies.
- Logs the synchronization delta between the two message filters.
"""

import csv, os, atexit
from datetime import datetime
from pathlib import Path
import threading  # <-- FIX: Import threading for lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose, PoseStamped
from piper_eval_msgs.msg import PiperTeleopMetric
import message_filters


def _shutdown_safely(node: Node):
    """Safely attempt to destroy node and shutdown rclpy."""
    try:
        if node:
            node.destroy_node()
    except Exception:
        pass  # Node may already be destroyed
    try:
        rclpy.shutdown()
    except Exception:
        pass  # RCLPY may already be shutdown


class PiperEvaluatorNode(Node):
    """
    Subscribes:
      - /piper/teleop_metrics_raw (PiperTeleopMetric: carries camera t0 + IK delay)
      - /piper/ee_pose (PoseStamped: actual EE pose from TF)
    ...
    """

    def __init__(self):
        super().__init__("piper_evaluator_node")

        # -------- Parameters --------
        self.declare_parameter("teleop_metrics_topic", "/piper/teleop_metrics_raw")
        self.declare_parameter("kinematics_pose_topic", "/piper/ee_pose")
        self.declare_parameter("evaluation_duration_s", 60.0)
        self.declare_parameter("sync_slop_s", 0.15)
        self.declare_parameter("qos_depth", 50)

        self.teleop_topic = self.get_parameter("teleop_metrics_topic").value
        self.kin_topic = self.get_parameter("kinematics_pose_topic").value
        self.eval_duration = float(self.get_parameter("evaluation_duration_s").value)
        self.sync_slop = float(self.get_parameter("sync_slop_s").value)
        self.qos_depth = int(self.get_parameter("qos_depth").value)

        # -------- Output path: ~/eval/piper (or fallback) --------
        try:
            self.output_dir = str(Path.home() / "eval" / "piper" / "userstudy" / "U7")
            os.makedirs(self.output_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().error(
                f"Failed to create {Path.home() / 'eval' / 'piper'}: {e}. Falling back to ~/.ros/piper_evals/piper"
            )
            self.output_dir = str(Path.home() / ".ros" / "piper_evals" / "piper")
            os.makedirs(self.output_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.output_path = os.path.join(self.output_dir, f"piper_run_{ts}.csv")

        # -------- QoS --------
        qos = QoSProfile(depth=self.qos_depth)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        # -------- Subscribers + Synchronizer --------
        self.teleop_sub = message_filters.Subscriber(
            self, PiperTeleopMetric, self.teleop_topic, qos_profile=qos
        )
        self.kin_sub = message_filters.Subscriber(
            self, PoseStamped, self.kin_topic, qos_profile=qos
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.teleop_sub, self.kin_sub],
            queue_size=self.qos_depth,
            slop=self.sync_slop,
        )
        self.sync.registerCallback(self._synced_cb)

        # -------- Accumulators --------
        self.rows = []
        self.ik_delay_sum_ms = 0.0
        self.ik_delay_count = 0
        self.e2e_sum_ms = 0.0
        self.e2e_count = 0
        self._last_log_ns = 0
        self._first_sync_received = False

        # -------- FIX: Robust Shutdown Logic --------
        self._done = False
        self._exit_lock = threading.Lock()
        self._finish_timer = self.create_timer(
            self.eval_duration, self._on_evaluation_timer
        )
        atexit.register(self.cleanup_and_exit)  # Register new exit handler
        # ----------------------------------------

        self.get_logger().info(
            "PiperEvaluatorNode started.\n"
            f"  teleop: {self.teleop_topic}\n"
            f"  kin:    {self.kin_topic}\n"
            f"  duration: {self.eval_duration:.1f}s  slop: {self.sync_slop:.3f}s\n"
            f"  writing to: {self.output_path}"
        )
        self.get_logger().info("Waiting for first synchronized message pair...")

    @staticmethod
    def _pose_to_list(p: Pose):
        return [
            float(p.position.x),
            float(p.position.y),
            float(p.position.z),
            float(p.orientation.x),
            float(p.orientation.y),
            float(p.orientation.z),
            float(p.orientation.w),
        ]

    def _synced_cb(self, teleop_msg: PiperTeleopMetric, kin_msg: PoseStamped):
        if not self._first_sync_received:
            self.get_logger().info(
                "First synchronized message pair received! Starting data logging."
            )
            self._first_sync_received = True

        # --- Capture timestamps ---
        cam_stamp = teleop_msg.header.stamp
        cam_ts_sec = float(cam_stamp.sec) + float(cam_stamp.nanosec) * 1e-9
        cam_ts_ns = cam_stamp.sec * 1_000_000_000 + cam_stamp.nanosec

        kin_stamp = kin_msg.header.stamp
        kin_ts_sec = float(kin_stamp.sec) + float(kin_stamp.nanosec) * 1e-9

        # --- NEW: Calculate sync delta (how far apart are the messages) ---
        sync_delta_ms = (kin_ts_sec - cam_ts_sec) * 1000.0

        # --- NEW: Calculate both E2E latencies ---
        now_ns = self.get_clock().now().nanoseconds

        # 1. Latency from camera to joint command publish (from controller)
        e2e_at_control_ms = float(teleop_msg.e2e_latency_ms_at_control)

        # 2. Latency from camera to this evaluator node
        e2e_at_eval_ms = (now_ns - cam_ts_ns) / 1e6

        # Add to accumulators for logging
        if e2e_at_eval_ms == e2e_at_eval_ms:  # NaN-safe
            self.e2e_sum_ms += e2e_at_eval_ms
            self.e2e_count += 1

        # --- Get Poses ---
        tgt_list = self._pose_to_list(teleop_msg.target_pose)
        kin_list = self._pose_to_list(kin_msg.pose)

        # --- Get IK delay ---
        ik_ms = float(teleop_msg.ik_solver_delay_ms)
        if ik_ms == ik_ms:
            self.ik_delay_sum_ms += ik_ms
            self.ik_delay_count += 1

        # --- NEW: Append all metrics to the row ---
        row = [
            cam_ts_sec,
            *tgt_list,
            *kin_list,
            ik_ms,
            e2e_at_control_ms,  # New column
            e2e_at_eval_ms,  # Renamed for clarity
            sync_delta_ms,  # New column
        ]
        self.rows.append(row)

        # Throttled status (uses evaluator-side latency for logging)
        if now_ns - self._last_log_ns > 1_000_000_000:
            avg_ik = self.ik_delay_sum_ms / max(1, self.ik_delay_count)
            avg_e2e = self.e2e_sum_ms / max(1, self.e2e_count)
            self.get_logger().info(
                f"E2E (at eval) ~ {avg_e2e:.1f} ms | Avg IK ~ {avg_ik:.1f} ms | Sync Δ ~ {sync_delta_ms:.1f} ms"
            )
            self._last_log_ns = now_ns

    # ---------- FIX: New Shutdown Logic ----------
    def _on_evaluation_timer(self):
        """Callback for when the timer finishes."""
        self.get_logger().info("Evaluation timer finished.")
        self.cleanup_and_exit()

    def _csv_header(self):
        # --- NEW: Updated header to match new row structure ---
        return [
            "capture_timestamp",
            "target_pos_x",
            "target_pos_y",
            "target_pos_z",
            "target_orient_x",
            "target_orient_y",
            "target_orient_z",
            "target_orient_w",
            "kinematic_pos_x",
            "kinematic_pos_y",
            "kinematic_pos_z",
            "kinematic_orient_x",
            "kinematic_orient_y",
            "kinematic_orient_z",
            "kinematic_orient_w",
            "ik_solver_delay_ms",
            "e2e_latency_at_control_ms",  # NEW
            "e2e_latency_at_eval_ms",  # NEW
            "sync_delta_ms",  # NEW
        ]

    def _write_csv(self):
        """Internal function to write data. Assumes lock is held."""
        if not self.rows:
            self.get_logger().warn(
                f"No data collected. Writing empty CSV to {self.output_path}"
            )
        else:
            self.get_logger().info(
                f"Writing {len(self.rows)} rows to {self.output_path}..."
            )

        try:
            with open(self.output_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(self._csv_header())
                if self.rows:
                    writer.writerows(self.rows)
            self.get_logger().info(f"Successfully saved CSV to {self.output_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV to {self.output_path}: {e}")

    def cleanup_and_exit(self):
        """
        Primary, thread-safe exit function.
        This is called by the timer, atexit, and Ctrl+C.
        """
        with self._exit_lock:
            if self._done:
                return  # Already cleaned up

            self.get_logger().info("Cleanup triggered. Saving data...")

            # Mark as done immediately to prevent re-entry
            self._done = True

            # Stop the timer if it's still running
            if self._finish_timer:
                self._finish_timer.cancel()
                self._finish_timer = None

            # Write the actual data
            self._write_csv()

            # Politely shut down the node
            self.get_logger().info("Shutting down node.")
            _shutdown_safely(self)

    # ---------- End of New Shutdown Logic ----------


def main(args=None):
    rclpy.init(args=args)
    node = PiperEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # <-- FIX: Call new handler -->
        node.get_logger().info("Ctrl+C received.")
        node.cleanup_and_exit()
    except Exception as e:
        # <-- FIX: Call new handler -->
        node.get_logger().error(f"Unhandled exception: {e}")
        node.cleanup_and_exit()


if __name__ == "__main__":
    main()
