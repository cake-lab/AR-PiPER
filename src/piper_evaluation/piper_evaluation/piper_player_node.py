#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import csv
import time
import os
import sys


class PiperPlayerNode(Node):
    def __init__(self):
        super().__init__("piper_player_node")

        # --- Settings ---
        self.declare_parameter("data_root", "/home/harshrocks/eval/piper/userstudy")
        self.declare_parameter("user_id", "U3")
        self.declare_parameter("target_file", "H1.csv")
        self.declare_parameter("repeat_count", 5)
        self.declare_parameter("interval_seconds", 30.0)

        self.data_root = (
            self.get_parameter("data_root").get_parameter_value().string_value
        )
        self.user_id = self.get_parameter("user_id").get_parameter_value().string_value
        self.target_file = (
            self.get_parameter("target_file").get_parameter_value().string_value
        )
        self.repeat_count = (
            self.get_parameter("repeat_count").get_parameter_value().integer_value
        )
        self.interval_s = (
            self.get_parameter("interval_seconds").get_parameter_value().double_value
        )

        # --- Publishers ---
        self.pose_publisher = self.create_publisher(PoseStamped, "/target_pose_raw", 10)
        self.trigger_publisher = self.create_publisher(
            String, "/piper/experiment_trigger", 10
        )

        # --- Load Data ---
        self.trajectory = self.load_csv()
        if not self.trajectory:
            self.get_logger().error("Aborting: No data found.")
            sys.exit(1)

        self.get_logger().info(
            f"Loaded {len(self.trajectory)} points. Starting experiment..."
        )

        # Give subscribers time to connect
        time.sleep(2.0)

        # Run Sequence
        self.run_sequence()

    def load_csv(self):
        path = os.path.join(self.data_root, self.user_id, self.target_file)
        self.get_logger().info(f"Loading Human File: {path}")
        data = []
        try:
            with open(path, "r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    data.append(
                        {
                            "t": float(row["capture_timestamp"]),
                            "x": float(row["target_pos_x"]),
                            "y": float(row["target_pos_y"]),
                            "z": float(row["target_pos_z"]),
                            "qx": float(row["target_orient_x"]),
                            "qy": float(row["target_orient_y"]),
                            "qz": float(row["target_orient_z"]),
                            "qw": float(row["target_orient_w"]),
                        }
                    )
        except Exception as e:
            self.get_logger().error(f"Read Failed: {e}")
        return data

    def run_sequence(self):
        for run_id in range(1, self.repeat_count + 1):
            self.get_logger().info(
                f"=== PREPARING RUN {run_id}/{self.repeat_count} ==="
            )

            # --- FIX: ROBUST RESET ---
            # Stream the first point for 5 seconds to force the robot
            # to travel there smoothly before recording starts.
            self.get_logger().info("Resetting to start position (Holding 5s)...")
            start_pose = self.trajectory[0]
            reset_end_time = time.time() + 5.0

            while time.time() < reset_end_time:
                self.publish_point(start_pose)
                time.sleep(0.033)  # 30Hz streaming

            # 2. Cooldown
            self.get_logger().info(f"Cooling down {self.interval_s}s...")
            time.sleep(self.interval_s)

            # 3. Start Recording
            self.get_logger().info("Starting Recording...")
            self.trigger_publisher.publish(String(data="START:ROBOT"))
            time.sleep(0.5)  # Allow evaluator to open file

            # 4. Replay Trajectory
            self.get_logger().info("Replaying Trajectory...")
            self.replay_motion()

            # 5. Stop Recording
            self.get_logger().info("Stopping Recording...")
            self.trigger_publisher.publish(String(data="STOP"))
            time.sleep(1.0)  # Ensure write completes

        self.get_logger().info("ALL RUNS COMPLETE.")
        sys.exit(0)

    def publish_point(self, p):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.pose.position.x = p["x"]
        msg.pose.position.y = p["y"]
        msg.pose.position.z = p["z"]
        msg.pose.orientation.x = p["qx"]
        msg.pose.orientation.y = p["qy"]
        msg.pose.orientation.z = p["qz"]
        msg.pose.orientation.w = p["qw"]

        self.pose_publisher.publish(msg)

    def replay_motion(self):
        start_t = self.trajectory[0]["t"]
        local_start = time.time()

        for p in self.trajectory:
            # Time Sync logic
            target_delay = p["t"] - start_t
            elapsed = time.time() - local_start
            sleep_time = target_delay - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)

            self.publish_point(p)


def main(args=None):
    rclpy.init(args=args)
    node = PiperPlayerNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
