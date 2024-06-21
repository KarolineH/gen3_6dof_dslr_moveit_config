import rclpy
from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit.core.robot_state import RobotState
from shape_msgs.msg import SolidPrimitive



rclpy.init()
logger = get_logger("moveit_py.pose_goal")
mvp = MoveItPy(node_name="moveit_py_planning_scene")
arm = mvp.get_planning_component("manipulator")
planning_scene_monitor = arm.get_planning_scene_monitor()


with planning_scene_monitor.read_write() as scene:

        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "01"

        box_pose = Pose()
        box_pose.position.x = 0.3
        box_pose.position.y = 0.3
        box_pose.position.z = -0.72

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = []

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        scene.apply_collision_object(collision_object)
        scene.current_state.update()  # Important to ensure the scene is updated