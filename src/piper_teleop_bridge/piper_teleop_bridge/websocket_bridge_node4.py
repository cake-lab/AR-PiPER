"""

This ROS 2 node acts as a WebSocket-to-ROS bridge for AR-based teleoperation.
It hosts a WebSocket server that receives real-time pose and gripper data (from the ARCore App), converts the incoming  JSON messages into ROS 2 messages, 
and publishes them to appropriate ROS topics.

Specifically, the node:
- Receives end-effector position and orientation data over WebSocket
- Publishes the pose as a geometry_msgs/PoseStamped message
- Publishes gripper commands as a std_msgs/Float32 message
- Applies a user-defined default orientation when rotation tracking is disabled
- Runs the WebSocket server asynchronously in a separate thread to avoid blocking the ROS 2 executor
- Sends robot feedback (TeleopMetric) back to the connected AR client(s) for visualization

Enables low-latency, network-based teleoperation of a robotic manipulator using external AR or vision-based interfaces.

"""

#!/usr/bin/env python3
import json
import threading
import time
from threading import Lock
from typing import Optional, Set
import socket

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from builtin_interfaces.msg import Time as TimeMsg

# Import the correct message type
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


class WebSocketBridgeNode4(Node):
    def __init__(self) -> None:
        super().__init__("WebSocketBridgeNode4")

        # --- Configuration ---
        self.declare_parameter("server_port", 8765)
        self.declare_parameter("pose_topic", "/target_pose")  # Output Topic 1
        self.declare_parameter("gripper_topic", "/gripper")  # Output Topic 2
        self.declare_parameter("feedback_topic", "/teleop_metrics") # Input Topic (Feedback)
        self.declare_parameter("robot_frame_id", "base_link")

        self.port = self.get_parameter("server_port").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.gripper_topic = self.get_parameter("gripper_topic").value
        self.feedback_topic = self.get_parameter("feedback_topic").value
        self.frame_id = self.get_parameter("robot_frame_id").value

        # --- Default Orientation ---
        # User requested default: [x, y, z, w]
        self.default_orientation = [0.0, 0.675, 0.0, 0.738]

        # --- QoS ---
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        # --- Publishers ---
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, qos)
        self.gripper_pub = self.create_publisher(Float32, self.gripper_topic, qos)

        # --- Subscriber (Feedback) ---
        self.feedback_sub = self.create_subscription(
            PiperTeleopMetric, self.feedback_topic, self._feedback_cb, qos
        )

        # --- Server State ---
        self.connected_clients = set()
        self.loop = None # Asyncio loop reference

        # Get local IP
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
        except:
            local_ip = "127.0.0.1"

        self.get_logger().info(
            f"ARPose Bridge Node Initialized. Server IP: {local_ip}, Port: {self.port}"
        )
        self.get_logger().info(
            f"Publishing to: {self.pose_topic}, {self.gripper_topic}"
        )
        self.get_logger().info(
            f"Subscribed to feedback: {self.feedback_topic}"
        )

        # Run the event loop in a separate thread so ROS spin isn't blocked
        self.ws_thread = threading.Thread(target=self.run_async_server, daemon=True)
        self.ws_thread.start()

    def run_async_server(self):
        # Asyncio Loop for WebSocket Server
        import asyncio
        import websockets

        # Create a new event loop for this thread
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        async def runner():
            # This must be inside the async function so 'serve' finds the running loop
            async with websockets.serve(self.websocket_handler, "0.0.0.0", self.port):
                await asyncio.Future()  # Run forever

        try:
            self.loop.run_until_complete(runner())
        except Exception as e:
            self.get_logger().error(f"Asyncio loop error: {e}")

    async def websocket_handler(self, websocket):
        self.get_logger().info(f"New Client Connected: {websocket.remote_address}")
        import websockets
        import json
        
        # Register client
        self.connected_clients.add(websocket)

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    self.process_json_data(data)
                except json.JSONDecodeError:
                    self.get_logger().warn("Received Invalid JSON")
                except Exception as e:
                    self.get_logger().error(f"Error processing message: {e}")
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info("Client Disconnected")
        except Exception as e:
            self.get_logger().error(f"Socket Error: {e}")
        finally:
            # Unregister client
            self.connected_clients.remove(websocket)

    def process_json_data(self, data):
        """
        Parses ARPose JSON and publishes to ROS.
        """
        pos_data = data.get("position", {})
        gripper_val = data.get("gripper", 0.0)

        # 1. Publish PoseStamped
        msg_pose = PoseStamped()
        msg_pose.header.frame_id = self.frame_id
        msg_pose.header.stamp = self.get_clock().now().to_msg()

        try:
            # Position
            msg_pose.pose.position.x = float(pos_data.get("x", 0.0))
            msg_pose.pose.position.y = float(pos_data.get("y", 0.0))
            msg_pose.pose.position.z = float(pos_data.get("z", 0.0))

            # Orientation
            qx = float(pos_data.get("qx", 0.0))
            qy = float(pos_data.get("qy", 0.0))
            qz = float(pos_data.get("qz", 0.0))
            qw = float(pos_data.get("qw", 1.0))

            # Check if App is sending Identity (Rotation Tracking OFF)
            if (
                abs(qx) < 1e-6
                and abs(qy) < 1e-6
                and abs(qz) < 1e-6
                and abs(qw - 1.0) < 1e-6
            ):
                # Use User-Defined Default
                msg_pose.pose.orientation.x = self.default_orientation[0]
                msg_pose.pose.orientation.y = self.default_orientation[1]
                msg_pose.pose.orientation.z = self.default_orientation[2]
                msg_pose.pose.orientation.w = self.default_orientation[3]
            else:
                # Use Live Data
                msg_pose.pose.orientation.x = qx
                msg_pose.pose.orientation.y = qy
                msg_pose.pose.orientation.z = qz
                msg_pose.pose.orientation.w = qw

            self.pose_pub.publish(msg_pose)

            # 2. Publish Gripper
            msg_grip = Float32()
            msg_grip.data = float(gripper_val)
            self.gripper_pub.publish(msg_grip)

        except Exception as e:
            self.get_logger().error(f"Failed to parse/publish data: {e}")

    def _feedback_cb(self, msg: PiperTeleopMetric):
        """
        Sends robot feedback back to connected WebSocket clients.
        """
        import asyncio
        
        if not self.connected_clients or self.loop is None:
            return

        try:
            # Construct feedback payload
            payload = {
                "type": "feedback",
                "x": msg.target_pose.position.x,
                "y": msg.target_pose.position.y,
                "z": msg.target_pose.position.z,
                "ik_delay": getattr(msg, "ik_solver_delay_ms", 0.0),
            }
            json_str = json.dumps(payload)

            # Broadcast to all connected clients
            # Since we are in a ROS thread, we must use run_coroutine_threadsafe to schedule on the asyncio loop
            # Iterate over a copy to avoid modification issues during iteration
            clients = list(self.connected_clients)
            for ws in clients:
                if ws.open:
                     asyncio.run_coroutine_threadsafe(ws.send(json_str), self.loop)

        except Exception as e:
            self.get_logger().error(f"Error sending feedback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketBridgeNode4()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown_safely(node)


if __name__ == "__main__":
    main()
