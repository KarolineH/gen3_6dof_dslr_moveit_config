#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.
    This example publishes transforms from `base_frame` to a scan object (sphere) centre.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self):
        super().__init__('static_sphere_centre_broadcaster')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.make_transforms()

    def make_transforms(self):
        t = TransformStamped()
        logger = rclpy.logging.get_logger('logger')
        logger.info("StaticFramePublisher make_transforms")

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'sphere_centre'

        t.transform.translation.x = float(0.25)
        t.transform.translation.y = float(0.0)
        t.transform.translation.z = float(0.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(t)
        logger.info("StaticFramePublisher make_transforms done")

def main():
    print("Starting static tf publisher")
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
