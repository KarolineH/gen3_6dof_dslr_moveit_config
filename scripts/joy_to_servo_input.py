#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType

JOY_TOPIC = "/joy"
TWIST_TOPIC = "/servo_node/delta_twist_cmds"
BASE_FRAME_ID = "base_link"

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

        # subscribe to joystick topic
        self.joy_sub = self.create_subscription(Joy, JOY_TOPIC, self.joy_callback, 10)
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
        speed_scalar = 0.2
        twist.twist.linear.z = axes[RIGHT_STICK_Y] * speed_scalar
        twist.twist.linear.y = axes[RIGHT_STICK_X] * speed_scalar
        twist.twist.linear.x = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS['RIGHT_TRIGGER']) + \
                               0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS['LEFT_TRIGGER']) * speed_scalar
        twist.twist.angular.y = axes[LEFT_STICK_Y] * speed_scalar
        twist.twist.angular.x = axes[LEFT_STICK_X] * speed_scalar
        twist.twist.angular.z = float(buttons[RIGHT_BUMPER]) - float(buttons[LEFT_BUMPER]) * speed_scalar
        return twist

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
