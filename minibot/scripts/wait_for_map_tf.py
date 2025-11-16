#!/usr/bin/env python3
"""
Wait for the TF from 'map' to 'base_link' to become available, then exit.
This allows launch files to start navigation only after the map frame is present.
"""
import time

import rclpy
from rclpy.node import Node
import tf2_ros


class WaitForMapTF(Node):
    def __init__(self):
        super().__init__('wait_for_map_tf')

        # parameters
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('timeout', 30.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.timeout = float(self.get_parameter('timeout').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'Waiting for transform {self.target_frame} -> {self.source_frame} (timeout {self.timeout}s)')

    def wait(self):
        start = time.time()
        r = self.get_clock().rate(5)
        while rclpy.ok():
            try:
                now = rclpy.time.Time()
                # use can_transform with timeout 0 to check availability
                if self.tf_buffer.can_transform(self.target_frame, self.source_frame, now, timeout=rclpy.duration.Duration(seconds=0.0)):
                    self.get_logger().info(f'Transform {self.target_frame} -> {self.source_frame} available')
                    return 0
            except Exception:
                # ignore and retry
                pass

            if (time.time() - start) > self.timeout:
                self.get_logger().error(f'Timed out waiting for transform {self.target_frame} -> {self.source_frame}')
                return 2

            r.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = WaitForMapTF()
    try:
        rc = node.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(rc)


if __name__ == '__main__':
    main()
