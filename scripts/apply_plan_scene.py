#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from rclpy.logging import get_logger
import numpy as np


class SceneClient(Node):

    def __init__(self):
        super().__init__('scene_client')
        self.get_cli = self.create_client(GetPlanningScene, 'get_planning_scene')
        self.set_cli = self.create_client(ApplyPlanningScene, 'apply_planning_scene')

        while not self.get_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.set_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.get_req = GetPlanningScene.Request()
        self.set_req = ApplyPlanningScene.Request()

    def get_scene(self):
        self.get_req.components.components = 0
        self.future = self.get_cli.call_async(self.get_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def set_scene(self, scene):
        self.set_req.scene = scene
        self.future = self.set_cli.call_async(self.set_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def define_scene_manual(self):
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "office_furniture"

        positions = [[0.3,0.3,-0.72], [1.26,-0.06,0.0], [-1.24,0.55,-0.74], [-1.1,-0.42,0.29], [1.13, 0.06, 0.0], [1.42,-1.01, 0.0], [0.0,0.52,0.0]]
        dimensions = [[0.8, 1, 1.5], [2, 0.7, 0.05], [1.5, 1.5, 1.5], [0.5, 2, 2], [2, 0.7, 0.05], [2, 3, 0.05], [3, 0.1, 3]]

        for pos,dim in zip(positions, dimensions):
            box_pose = Pose()
            box_pose.position.x = pos[0]
            box_pose.position.y = pos[1]
            box_pose.position.z = pos[2]
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dim

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

        return collision_object
    
    def read_scene_from_file(self, path):

        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "office_furniture"

        with open(path, 'r') as file:
            data = file.read()

        geometry_info = data.split('*')[1:]
        for obj in geometry_info:
            if obj.split('\n')[4]=='box':
                pose=np.asarray(obj.split('\n')[1].split(' '), dtype=np.float64)
                dim=np.asarray(obj.split('\n')[5].split(' '), dtype=np.float64)
                box_pose = Pose()
                box_pose.position.x = pose[0]
                box_pose.position.y = pose[1]
                box_pose.position.z = pose[2]
                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = list(dim)

                collision_object.primitives.append(box)
                collision_object.primitive_poses.append(box_pose)
                collision_object.operation = CollisionObject.ADD

            else:
                print('Collision object other than type "box" detected. Skipping...')
        return collision_object

def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("planning_scene_publisher")
    logger.info("Creating initial planning scene from file...")
    scene_file = str(sys.argv[-1])
    #scene_file = '/home/karo/rosws/src/gen3_6dof_dslr_moveit_config/scene/desk_scene.scene'

    scene_client = SceneClient()
    col_obj = scene_client.read_scene_from_file(scene_file)
    response1 = scene_client.get_scene()
    response1.scene.world.collision_objects.append(col_obj)
    response2 = scene_client.set_scene(response1.scene)
    logger.info("Planning scene initialised and published.")

    scene_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()