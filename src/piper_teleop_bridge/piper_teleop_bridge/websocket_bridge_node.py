#!/usr/bin/env python3
import json
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
import websocket  # This requires the 'websocket-client' pip package

class WebSocketBridgeNode(Node):
    """
    Connects to a WebSocket server (like the one in apture.py), receives JSON data  containing 6-DoF pose information, and publishes it as a ROS 2 PoseStamped message.
    
    This node is designed for real-time teleoperation with several key features:
    - Runs the WebSocket client in a separate thread to avoid blocking the ROS node.
    - Uses a thread-safe queue to pass data, decoupling the network from ROS publishing.
    - Automatically attempts to reconnect if the WebSocket connection is lost.
    - Rate-limits the output to a consistent frequency.
    - Drops stale messages that are too old to be useful for real-time control.
    """
    def __init__(self):
        super().__init__('websocket_bridge_node')

        # --- Parameters ---
        # These parameters make the node easily configurable from a launch file.
        self.declare_parameter('ws_url', 'ws://localhost:8765')
        self.declare_parameter('output_topic', '/target_pose')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('max_age_ms', 150.0)      
        self.declare_parameter('rate_limit_hz', 60.0)
        self.declare_parameter('qos_depth', 10)

        self.ws_url = self.get_parameter('ws_url').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.max_age_ms = self.get_parameter('max_age_ms').get_parameter_value().double_value
        self.rate_limit_hz = self.get_parameter('rate_limit_hz').get_parameter_value().double_value
        self.qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value

        # --- ROS 2 Publisher ---
        # A reliable QoS is chosen to ensure commands are not dropped, but for very
        # high-frequency control, BEST_EFFORT might be considered.
        qos = QoSProfile(
            depth=self.qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE, # We only care about live data
        )
        self.publisher_ = self.create_publisher(PoseStamped, self.output_topic, qos)

        # --- Threading and Queues ---
        # A thread-safe queue decouples the websocket I/O thread from the ROS timer thread.
        # This prevents backpressure and blocking.
        self._in_q = queue.Queue(maxsize=5)

        # The timer is the "heartbeat" that publishes messages at a steady rate.
        self._publish_timer = self.create_timer(1.0 / self.rate_limit_hz, self._publish_latest)

        # The WebSocket client runs in its own thread to handle network I/O.
        self._ws_thread = threading.Thread(target=self._ws_main_loop, daemon=True)
        self._ws_thread.start()

        self.get_logger().info(
            f"WebSocket Bridge configured for '{self.ws_url}' and publishing to '{self.output_topic}'"
        )

    def _ws_main_loop(self):
        """
        The main loop for the WebSocket thread. Includes an auto-reconnect mechanism.
        """
        while rclpy.ok():
            try:
                # We use WebSocketApp for its convenient callback system
                ws = websocket.WebSocketApp(
                    self.ws_url,
                    on_open=self._on_ws_open,
                    on_message=self._on_ws_message,
                    on_close=self._on_ws_close,
                    on_error=self._on_ws_error,
                )
                # This call blocks until the connection is lost
                ws.run_forever(ping_interval=5, ping_timeout=2)
            except Exception as e:
                self.get_logger().error(f"WebSocketApp exception: {e}")
            
            if rclpy.ok():
                self.get_logger().warn("WebSocket disconnected. Reconnecting in 2 seconds...")
                time.sleep(2.0)

    def _on_ws_open(self, ws):
        self.get_logger().info(f"WebSocket connection established.")

    def _on_ws_close(self, ws, status_code, msg):
        self.get_logger().warn(f"WebSocket connection closed.")

    def _on_ws_error(self, ws, error):
        self.get_logger().error(f"WebSocket error: {error}")

    def _on_ws_message(self, ws, message: str):
        """
        Callback for when a new message is received from the WebSocket.
        It parses the JSON, validates it, and puts the data onto the thread-safe queue.
        """
        try:
            data = json.loads(message)
            pos = data.get('position')
            # Basic validation: ensure the 'position' key exists
            if not pos:
                self.get_logger().warn_once("Received WS message without 'position' key. Skipping.")
                return

            # For real-time control, the timestamp from the capture script is crucial
            # We use 'timestamp' as it's the most recent one from apture.py
            timestamp = float(data.get('timestamp', 0.0))
            if timestamp == 0.0:
                 self.get_logger().warn_once("Received WS message without 'timestamp' key. Stale message checks will be inaccurate.")

            sample = {
                'x': float(pos.get('x', 0.0)), 'y': float(pos.get('y', 0.0)), 'z': float(pos.get('z', 0.0)),
                'qx': float(pos.get('qx', 0.0)), 'qy': float(pos.get('qy', 0.0)),
                'qz': float(pos.get('qz', 0.0)), 'qw': float(pos.get('qw', 1.0)),
                'timestamp': timestamp,
            }

            # If the queue is full, drop the oldest message to make room for the new one.
            # This prioritizes low latency.
            if self._in_q.full():
                self._in_q.get_nowait()
            self._in_q.put_nowait(sample)

        except (json.JSONDecodeError, TypeError, ValueError) as e:
            self.get_logger().error(f"Failed to parse WebSocket message: {e}")

    def _publish_latest(self):
        """
        This method is called periodically by the ROS timer. It drains the queue
        to get only the most recent message and publishes it. It also performs
        a critical safety check to drop stale data.
        """
        latest_sample = None
        try:
            # Drain the queue to ensure we are always processing the most recent data
            while True:
                latest_sample = self._in_q.get_nowait()
        except queue.Empty:
            # This is normal; it just means no new data has arrived since the last tick
            pass

        if not latest_sample:
            return

        # --- Critical Safety Check ---
        # Check the age of the data. If it's too old, we drop it to prevent the
        # robot from executing a command that is dangerously out of date.
        age_ms = (time.time() - latest_sample['timestamp']) * 1000.0
        if age_ms > self.max_age_ms:
            self.get_logger().warn(
                f"Dropping stale pose from WebSocket: age={age_ms:.1f}ms > {self.max_age_ms:.1f}ms"
            )
            return

        # --- Construct and Publish the Message ---
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        # We stamp the message with the current ROS time. This is important because
        # downstream nodes like MoveIt Servo might reject messages with old timestamps.
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = latest_sample['x']
        msg.pose.position.y = latest_sample['y']
        msg.pose.position.z = latest_sample['z']
        msg.pose.orientation.x = latest_sample['qx']
        msg.pose.orientation.y = latest_sample['qy']
        msg.pose.orientation.z = latest_sample['qz']
        msg.pose.orientation.w = latest_sample['qw']

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WebSocketBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
