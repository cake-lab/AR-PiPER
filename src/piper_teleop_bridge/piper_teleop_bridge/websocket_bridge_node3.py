"""

This node acts as a bidirectional bridge between a WebSocket-based teleoperation client and a robot control system.
Incoming pose data received over a WebSocket connection is converted into ROS 2 PoseStamped messages and published to a  target pose topic at a fixed control frequency. 
In the opposite direction, robot feedback metrics (TeleopMetric) are subscribed to, serialized into JSON, and streamed back to 
the WebSocket server for visualization or monitoring.
The node runs WebSocket communication in a dedicated background thread, ensuring non-blocking integration with the ROS 2  executor, and is designed 
for real-time teleoperation and performance evaluation workflows.

"""

#!/usr/bin/env python3
import json, threading, time
from threading import Lock
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time as TimeMsg
import websocket  # type: ignore

# UPDATED: Use the message type consistent with your other nodes and package.xml
from piper_eval_msgs.msg import PiperTeleopMetric


def _shutdown_safely(node: Optional[Node] = None):
    try:
        if node is not None:
            node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass


class WebSocketBridgeNode3(Node):
    def __init__(self) -> None:
        super().__init__("websocket_bridge_node_3")

        self.declare_parameter("websocket_url", "ws://localhost:8765")
        self.declare_parameter("pose_topic", "/target_pose_raw")
        self.declare_parameter("feedback_topic", "/piper/teleop_metrics_raw")
        self.declare_parameter("control_frequency", 30.0)
        self.declare_parameter("robot_frame_id", "base_link")

        self.websocket_url: str = (
            self.get_parameter("websocket_url").get_parameter_value().string_value
        )
        self.pose_topic: str = (
            self.get_parameter("pose_topic").get_parameter_value().string_value
        )
        self.feedback_topic: str = (
            self.get_parameter("feedback_topic").get_parameter_value().string_value
        )
        self.control_freq: float = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )
        self.robot_frame_id: str = (
            self.get_parameter("robot_frame_id").get_parameter_value().string_value
        )

        qos = QoSProfile(depth=30)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        # Publisher for Target Pose (Client -> Robot)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, qos)

        # Subscriber for Feedback (Robot -> Client)
        # UPDATED: Listening for TeleopMetric
        self.feedback_sub = self.create_subscription(
            TeleopMetric, self.feedback_topic, self._feedback_cb, qos
        )

        self._data_lock = Lock()
        self._latest_pose: Optional[dict] = None
        self._latest_capture_ts: Optional[float] = None

        self.ws_app: Optional[websocket.WebSocketApp] = None  # Hold reference to WS app

        self._ws_thread = threading.Thread(target=self._ws_thread_fn, daemon=True)
        self._ws_thread.start()

        self._timer = self.create_timer(
            1.0 / max(self.control_freq, 1.0), self._publish_timer_cb
        )
        self.get_logger().info(
            f"WebSocketBridgeNode up. URL={self.websocket_url} topic={self.pose_topic} freq={self.control_freq} Hz"
        )

    # ---------------- WebSocket thread ----------------
    def _ws_thread_fn(self) -> None:
        while rclpy.ok():
            self.get_logger().info(f"Connecting to {self.websocket_url}…")
            try:
                self.ws_app = websocket.WebSocketApp(
                    self.websocket_url,
                    on_open=self._on_ws_open,
                    on_message=self._on_ws_message,
                    on_close=self._on_ws_close,
                    on_error=self._on_ws_error,
                )
                self.ws_app.run_forever(ping_interval=10, ping_timeout=5)
            except Exception as e:
                self.get_logger().error(f"WebSocket run_forever error: {e}")
            finally:
                self.ws_app = None

            # Use plain time.sleep so we don't touch rclpy after shutdown
            if rclpy.ok():
                self.get_logger().info("WS reconnect in 1s…")
                time.sleep(1.0)

    def _on_ws_open(self, ws):
        self.get_logger().info("WebSocket connected.")

    def _on_ws_close(self, ws, code, msg):
        self.get_logger().warn(f"WebSocket closed (code={code}, msg={msg}).")

    def _on_ws_error(self, ws, err):
        self.get_logger().error(f"WebSocket error: {err}")

    def _on_ws_message(self, ws, message: str):
        try:
            data = json.loads(message)
        except Exception:
            self.get_logger().warn("Invalid JSON frame from WS; ignoring.")
            return

        pos = data.get("position")
        if not pos:
            return

        cap_ts = data.get("capture_timestamp", 0.0) or 0.0
        try:
            cap_ts = float(cap_ts)
        except Exception:
            cap_ts = 0.0

        with self._data_lock:
            self._latest_pose = pos
            self._latest_capture_ts = cap_ts

    # ---------------- ROS Feedback Callback ----------------
    def _feedback_cb(self, msg: PiperTeleopMetric):
        # Received feedback from the robot (TeleopMetric).
        # Extract fields and send to WebSocket server for visualization.
        
        try:
            # Extract relevant fields based on the structure provided
            # msg.target_pose is likely geometry_msgs/Pose based on other nodes usage
            
            payload = {
                "type": "feedback",
                "x": msg.target_pose.position.x,
                "y": msg.target_pose.position.y,
                "z": msg.target_pose.position.z,
                # Use getattr just in case the field name varies slightly in your version
                "ik_delay": getattr(msg, "ik_solver_delay_ms", 0.0),
            }

            # Send via WebSocket if connected
            if self.ws_app and self.ws_app.sock and self.ws_app.sock.connected:
                self.ws_app.send(json.dumps(payload))

        except AttributeError as e:
            self.get_logger().error(f"Failed to parse TeleopMetric: {e}")
        except Exception as e:
            self.get_logger().error(f"Error sending feedback via WS: {e}")

    # ---------------- ROS publish timer ----------------
    def _publish_timer_cb(self) -> None:
        with self._data_lock:
            pos = self._latest_pose
            cap_ts = self._latest_capture_ts
        if not pos:
            return

        msg = PoseStamped()
        msg.header.frame_id = self.robot_frame_id

        if isinstance(cap_ts, float) and cap_ts > 0.0:
            secs = int(cap_ts)
            nsecs = int((cap_ts - secs) * 1e9)
            msg.header.stamp = TimeMsg(sec=secs, nanosec=nsecs)
        else:
            msg.header.stamp = self.get_clock().now().to_msg()

        try:
            msg.pose.position.x = float(pos.get("x", 0.0))
            msg.pose.position.y = float(pos.get("y", 0.0))
            msg.pose.position.z = float(pos.get("z", 0.0))
            msg.pose.orientation.x = float(pos.get("qx", 0.0))
            msg.pose.orientation.y = float(pos.get("qy", 0.0))
            msg.pose.orientation.z = float(pos.get("qz", 0.0))
            msg.pose.orientation.w = float(pos.get("qw", 1.0))
        except Exception as e:
            self.get_logger().error(f"Failed to parse pose fields: {e}")
            return

        self.pose_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebSocketBridgeNode3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown_safely(node)


if __name__ == "__main__":
    main()
