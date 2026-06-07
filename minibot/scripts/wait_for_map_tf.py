#!/usr/bin/env python3
"""Wait until a map transform becomes available before letting Nav2 start."""

import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros


class WaitForMapTf(Node):
    """Simple node that polls TF until the requested transform is available."""

    def __init__(self) -> None:
        super().__init__('wait_for_map_tf')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'odom')
        self.declare_parameter('timeout', 30.0)
        self.declare_parameter('poll_rate', 5.0)

        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self, spin_thread=True)

    def wait(self) -> bool:
        parent = self.get_parameter('parent_frame').value
        child = self.get_parameter('child_frame').value
        timeout = float(self.get_parameter('timeout').value)
        poll_rate = float(self.get_parameter('poll_rate').value)

        poll_rate = max(poll_rate, 0.1)
        deadline = (
            self.get_clock().now() + Duration(seconds=timeout)
            if timeout >= 0.0
            else None
        )
        sleep_period = 1.0 / poll_rate
        while rclpy.ok():
            # ``can_transform`` throws when the TF tree is empty, so guard it.
            try:
                if self._buffer.can_transform(
                    parent,
                    child,
                    rclpy.time.Time(),
                    timeout_sec=0.0,
                ):
                    self.get_logger().info(f"Found transform {parent} -> {child}")
                    return True
            except tf2_ros.LookupException:
                pass
            except tf2_ros.ExtrapolationException:
                pass
            except tf2_ros.ConnectivityException:
                pass

            if deadline and self.get_clock().now() >= deadline:
                self.get_logger().error(f"Timed out waiting for TF {parent} -> {child}")
                return False

            time.sleep(sleep_period)

        self.get_logger().error('ROS shutdown detected before TF became available')
        return False


def main() -> int:
    rclpy.init(args=sys.argv)
    node = WaitForMapTf()
    try:
        success = node.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
