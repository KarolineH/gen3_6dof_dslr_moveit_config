#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
import numpy as np
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

JOY_TOPIC = "/joy"
TWIST_TOPIC = "/servo_node/delta_twist_cmds"
BASE_FRAME_ID = "base_link"
EE_FRAME_ID = "dslr_frame"
OBJECT_CENTRE_FRAME_ID = "sphere_centre"

# Indeces for axes
LEFT_STICK_X = 0 
LEFT_STICK_Y = 1
LEFT_TRIGGER = 2
RIGHT_STICK_X = 3
RIGHT_STICK_Y = 4
RIGHT_TRIGGER = 5
D_PAD_X = 6
D_PAD_Y = 7
# Indeces for buttons
A = 0
B = 1
X = 2
Y = 3
LEFT_BUMPER = 4
RIGHT_BUMPER = 5
CHANGE_VIEW = 6
MENU = 7

AXIS_DEFAULTS = {'LEFT_TRIGGER': 1.0, 'RIGHT_TRIGGER': 1.0}

class JoyToServoPub(Node):
    def __init__(self):
        super().__init__("joy_to_twist_publisher")
        self.frame_to_publish = BASE_FRAME_ID
        self.ee_frame = EE_FRAME_ID
        self.sphere_centre_frame = OBJECT_CENTRE_FRAME_ID

        # subscribe to joystick topic
        self.joy_sub = self.create_subscription(Joy, JOY_TOPIC, self.joy_callback, 10)
        # get a transform listener to keep up with the current robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # publish to twist topic
        self.twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        # need to switch to twist command type at the beginning
        self.servo_start_client = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        self.switch_command_type(1) # at the beginning, switch to twist command type 

    def switch_command_type(self, ctype=1):
        # ctype 1 for twist, 2 for joint
        while not self.servo_start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = ServoCommandType.Request()
        request.command_type=ctype
        future = self.servo_start_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def joy_callback(self, msg):
        twist_msg = TwistStamped()

        if self.convert_joy_to_cmd(msg.axes, msg.buttons, twist_msg):
            twist_msg.header.frame_id = self.frame_to_publish
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(twist_msg)
        else:
            rclpy.logging.get_logger("joy_to_twist_publisher").info("Something wrong with the joy command received")

    def convert_joy_to_cmd(self, axes, buttons, twist):
        # sphere definition
        radius = 0.1
        speed_scalar = 0.2
        k_r = 0.0 # radial correction gain
        radius_tolerance = 0.01

        # current robot pose in spherical coordinates
        theta_now = np.pi/4
        phi_now = np.pi/3

        t=None
        while t is None:
            t = self.get_current_pose()

        r_actual = np.sqrt(t.transform.translation.x**2 + t.transform.translation.y**2 + t.transform.translation.z**2)
        delta_r = r_actual - radius
        if abs(delta_r) > radius_tolerance:
            v_r = -k_r * delta_r
        else:
            v_r = 0.0

        sin_theta = np.sin(theta_now)
        cos_theta = np.cos(theta_now)
        sin_phi = np.sin(phi_now)
        cos_phi = np.cos(phi_now)
        sin_theta_cos_theta = sin_theta * cos_theta
        sin_theta_squared = sin_theta**2

        # inputs from the joystick == desired velocities in spherical coordinates
        delta_theta = axes[LEFT_STICK_X] * speed_scalar
        delta_phi = axes[RIGHT_STICK_Y] * speed_scalar
        
        # compute linear velocities assuming sphere at the origin
        twist.twist.linear.x = sin_theta * cos_phi * v_r + cos_theta * cos_phi * delta_theta - sin_phi * delta_phi
        twist.twist.linear.y = sin_theta * sin_phi * v_r + cos_theta * sin_phi * delta_theta + cos_phi * delta_phi
        twist.twist.linear.z = cos_theta * v_r - sin_theta * delta_theta
        # compute angular velocities
        twist.twist.angular.x = sin_phi * delta_theta + sin_theta_cos_theta * cos_phi * delta_phi
        twist.twist.angular.y = -cos_phi * delta_theta + sin_theta_cos_theta * sin_phi * delta_phi
        twist.twist.angular.z = -sin_theta_squared * delta_phi
        return twist
    
    def get_current_pose(self):
        self.sphere_centre_frame
        self.ee_frame
        # fetch the latest transform from sphere centre to end effector (camera)
        try:
            t = self.tf_buffer.lookup_transform(
                self.ee_frame,
                self.sphere_centre_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.ee_frame,} to {self.sphere_centre_frame}: {ex}')
            return
        return t

def main():
    rclpy.init()
    node = JoyToServoPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()







