#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as Rotation
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticFramePublisher(Node):
    """
    Broadcast (imaginary) frame to serve as a pivot rod for the camera. 
    You can keep a fixed distance between the camera (image frame) and the centre of the object,
    by specifying the planning frame as an extension of the camera into the scene.
    It's origin is at the centre of the object, it's offset is the desired distance between the camera and the object.
    You can pivot the camera around the object by rotations only in the pivot frame.
    """

    def __init__(self):
        super().__init__('static_pivot_tf_broadcaster')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.declare_parameter('pivot_offset', 0.4)  # Default value: 0.4 meters
        pivot_offset = self.get_parameter('pivot_offset').get_parameter_value().double_value

        self.make_transforms(pivot_offset)

         # TODO make a service that changes the offset distance

    def make_transforms(self, offset):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'dslr_frame'
        t.child_frame_id = 'pivot_frame'
        t.transform.translation.z = float(offset)

        self.tf_static_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()