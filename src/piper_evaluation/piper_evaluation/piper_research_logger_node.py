#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This node is an adaptation of the 'master_evaluator_node.py' from the omx workspace, modified to be fully independent and work
with Piper-specific topics and messages from the piper_ros workspace.

--- V3 (Research Grade) ---
- Logs both controller-side and evaluator-side E2E latencies.
- Logs the synchronization delta between the two message filters.
"""

import csv, os, atexit
from datetime import datetime
from pathlib import Path
import threading

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


class PiperResearchLoggerNode(Node):
    """
    Subscribes:
      - /piper/teleop_metrics_raw (PiperTeleopMetric: carries camera t0 + IK delay)
      - /piper/ee_pose (PoseStamped: actual EE pose from TF)
    """

    def __init__(self):
        super().__init__("piper_research_logger_node")

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
            self.output_dir = str(Path.home() / "eval" / "piper")
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

        # -------- Robust Shutdown Logic --------
        self._done = False
        self._exit_lock = threading.Lock()
        self._finish_timer = self.create_timer(
            self.eval_duration, self._on_evaluation_timer
        )
        atexit.register(self.cleanup_and_exit)

        self.get_logger().info(
            "PiperResearchLoggerNode started.\n"
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

        # --- Calculate sync delta ---
        sync_delta_ms = (kin_ts_sec - cam_ts_sec) * 1000.0

        # --- Calculate both E2E latencies ---
        now_ns = self.get_clock().now().nanoseconds

        # 1. Latency from camera to joint command publish
        e2e_at_control_ms = float(teleop_msg.e2e_latency_ms_at_control)

        # 2. Latency from camera to this evaluator node
        e2e_at_eval_ms = (now_ns - cam_ts_ns) / 1e6

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

        # --- Append Metrics ---
        row = [
            cam_ts_sec,
            *tgt_list,
            *kin_list,
            ik_ms,
            e2e_at_control_ms,
            e2e_at_eval_ms,
            sync_delta_ms,
        ]
        self.rows.append(row)

        # Throttled Logging
        if now_ns - self._last_log_ns > 1_000_000_000:
            avg_ik = self.ik_delay_sum_ms / max(1, self.ik_delay_count)
            avg_e2e = self.e2e_sum_ms / max(1, self.e2e_count)
            self.get_logger().info(
                f"E2E (at eval) ~ {avg_e2e:.1f} ms | Avg IK ~ {avg_ik:.1f} ms | Sync Δ ~ {sync_delta_ms:.1f} ms"
            )
            self._last_log_ns = now_ns

    def _on_evaluation_timer(self):
        """Callback for when the timer finishes."""
        self.get_logger().info("Evaluation timer finished.")
        self.cleanup_and_exit()

    def _csv_header(self):
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
            "e2e_latency_at_control_ms",
            "e2e_latency_at_eval_ms",
            "sync_delta_ms",
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
        """Primary, thread-safe exit function."""
        with self._exit_lock:
            if self._done:
                return

            self.get_logger().info("Cleanup triggered. Saving data...")
            self._done = True

            if self._finish_timer:
                self._finish_timer.cancel()
                self._finish_timer = None

            self._write_csv()
            self.get_logger().info("Shutting down node.")
            _shutdown_safely(self)


def main(args=None):
    rclpy.init(args=args)
    node = PiperResearchLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received.")
        node.cleanup_and_exit()
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
        node.cleanup_and_exit()


if __name__ == "__main__":
    main()
