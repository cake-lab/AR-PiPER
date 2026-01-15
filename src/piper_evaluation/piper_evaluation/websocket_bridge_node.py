#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import Float32MultiArray, String
import websocket
import json
import threading
import time
from threading import Lock


class WebSocketBridgeNode(Node):
    def __init__(self):
        super().__init__("websocket_bridge_node")

        # Parameters
        self.declare_parameter("websocket_url", "ws://localhost:8765")
        self.declare_parameter("control_frequency", 30.0)

        # Topics
        self.declare_parameter(
            "pose_topic", "/target_pose_raw"
        )  # UPDATED to PoseStamped
        self.declare_parameter("velocity_topic", "/cmd_twist")
        self.declare_parameter("angular_velocity_topic", "/angular_velocity")
        self.declare_parameter("trigger_topic", "/piper/experiment_trigger")

        # Get parameters
        self.websocket_url = (
            self.get_parameter("websocket_url").get_parameter_value().string_value
        )
        self.control_freq = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )

        # Publishers
        self.twist_publisher = self.create_publisher(
            Twist,
            self.get_parameter("velocity_topic").get_parameter_value().string_value,
            10,
        )

        # CHANGED: Now publishing PoseStamped to capture Orientation too
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            self.get_parameter("pose_topic").get_parameter_value().string_value,
            10,
        )

        self.angular_vel_publisher = self.create_publisher(
            Float32MultiArray,
            self.get_parameter("angular_velocity_topic")
            .get_parameter_value()
            .string_value,
            10,
        )

        self.trigger_publisher = self.create_publisher(
            String,
            self.get_parameter("trigger_topic").get_parameter_value().string_value,
            10,
        )

        # Data storage
        self.data_lock = Lock()
        self.latest_data = None
        self.last_data_time = time.time()
        self.connection_active = False
        self.prev_recording_active = False

        # WebSocket setup
        self.ws = None
        self.ws_thread = None

        self.create_timer(1.0 / self.control_freq, self.control_callback)
        self.connect_websocket()

        self.get_logger().info("WebSocket Bridge Node initialized")

    def connect_websocket(self):
        try:
            self.ws = websocket.WebSocketApp(
                self.websocket_url,
                on_message=self.on_message,
                on_error=self.on_error,
                on_close=self.on_close,
                on_open=self.on_open,
            )
            self.ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
            self.ws_thread.start()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to WebSocket: {e}")

    def on_open(self, ws):
        self.connection_active = True
        self.get_logger().info(f"WebSocket connected to {self.websocket_url}")

    def on_message(self, ws, message):
        try:
            data = json.loads(message)
            with self.data_lock:
                self.latest_data = data
                self.last_data_time = time.time()
        except Exception:
            pass

    def on_error(self, ws, error):
        self.get_logger().error(f"WebSocket error: {error}")
        self.connection_active = False

    def on_close(self, ws, close_status_code, close_msg):
        self.connection_active = False
        self.get_logger().warn("WebSocket connection closed")
        time.sleep(2.0)
        self.connect_websocket()

    def control_callback(self):
        with self.data_lock:
            if self.latest_data is None:
                return

            if time.time() - self.last_data_time > 0.5:
                if self.prev_recording_active:
                    self.trigger_publisher.publish(String(data="STOP"))
                    self.prev_recording_active = False
                self.publish_zero_velocity()
                return

            current_data = self.latest_data.copy()

        try:
            self.process_websocket_data(current_data)
        except Exception as e:
            self.get_logger().error(f"Error processing control data: {e}")

    def process_websocket_data(self, data):
        # --- TRIGGER LOGIC ---
        current_recording_active = data.get("recording_active", False)

        if current_recording_active and not self.prev_recording_active:
            msg = String()
            msg.data = "START:HUMAN"
            self.trigger_publisher.publish(msg)
            self.get_logger().info("Trigger: START:HUMAN")

        elif not current_recording_active and self.prev_recording_active:
            msg = String()
            msg.data = "STOP"
            self.trigger_publisher.publish(msg)
            self.get_logger().info("Trigger: STOP")

        self.prev_recording_active = current_recording_active

        # --- CONTROL LOGIC ---
        # 1. Position & Orientation
        if "position" in data and data["position"] is not None:
            pos_data = data["position"]

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "base_link"

            # Position
            pose_msg.pose.position.x = float(pos_data.get("x", 0.0))
            pose_msg.pose.position.y = float(pos_data.get("y", 0.0))
            pose_msg.pose.position.z = float(pos_data.get("z", 0.0))

            # Orientation (Quaternions)
            pose_msg.pose.orientation.x = float(pos_data.get("qx", 0.0))
            pose_msg.pose.orientation.y = float(pos_data.get("qy", 0.0))
            pose_msg.pose.orientation.z = float(pos_data.get("qz", 0.0))
            pose_msg.pose.orientation.w = float(pos_data.get("qw", 1.0))

            self.pose_publisher.publish(pose_msg)

        # 2. Velocity
        if "velocity" in data and data["velocity"] is not None:
            vel_data = data["velocity"]
            twist_msg = Twist()
            twist_msg.linear.x = float(vel_data.get("vx", 0.0))
            twist_msg.linear.y = float(vel_data.get("vy", 0.0))
            twist_msg.linear.z = float(vel_data.get("vz", 0.0))
            if "wx" in vel_data:
                twist_msg.angular.x = float(vel_data.get("wx", 0.0))
                twist_msg.angular.y = float(vel_data.get("wy", 0.0))
                twist_msg.angular.z = float(vel_data.get("wz", 0.0))
            self.twist_publisher.publish(twist_msg)

            if all(key in vel_data for key in ["wx", "wy", "wz"]):
                angular_vel_msg = Float32MultiArray()
                angular_vel_msg.data = [
                    float(vel_data["wx"]),
                    float(vel_data["wy"]),
                    float(vel_data["wz"]),
                ]
                self.angular_vel_publisher.publish(angular_vel_msg)

    def publish_zero_velocity(self):
        twist_msg = Twist()
        self.twist_publisher.publish(twist_msg)

    def destroy_node(self):
        if self.ws:
            self.ws.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
