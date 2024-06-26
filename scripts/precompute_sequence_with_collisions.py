#!/usr/bin/env python3

import rclpy
from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.core.robot_state import RobotState
from moveit.planning import PlanRequestParameters
import numpy as np


'''
Initialises a MoveItPy node
Also intialises a pre-defined planning scene with collision objects
Then plans a sequence of movements through a list of desired joint states and records the planned trajectories
'''

class MoveItIF:

    def __init__(self, robot_ip, use_fake_hardware, planner, velocity_scaling, scene_file=None):
        moveit_config = self.configure_moveit(robot_ip, use_fake_hardware)

        self.logger = get_logger("moveit_py_IF")
        self.mvp = MoveItPy(node_name="moveit_py", config_dict=moveit_config)
        self.arm = self.mvp.get_planning_component("manipulator")
        self.plan_params = PlanRequestParameters(self.mvp, planner) # change planner settings here
        self.plan_params.max_velocity_scaling_factor = velocity_scaling

        # define collision scene
        pcm = self.mvp.get_planning_scene_monitor()
        if scene_file is None:
            collision_object = self.define_scene_manual()
        else:
            collision_object = self.read_scene_from_file(scene_file)
        self.initialise_collision_scene(pcm, collision_object)

        # get robot model from initial state
        initial_state = self.arm.get_start_state()
        self.robot_model = initial_state.robot_model

    def configure_moveit(self, robot_ip, use_fake_hardware):
        # MoveItPy Configuration
        # If this node is launched from the main launch file, this manual definition is not necessary
        launch_arguments = {
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "gripper": "false",
            "dof": "6",
            "vision": "true",
            }
        moveit_config = (
                MoveItConfigsBuilder(robot_name="gen3", package_name="gen3_6dof_dslr_moveit_config")
                .robot_description(mappings=launch_arguments)
                #.planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
                .planning_scene_monitor(publish_robot_description_semantic=True)
                .moveit_cpp(file_path=get_package_share_directory("gen3_6dof_dslr_moveit_config") + "/config/moveit_cpp.yaml")
            .to_moveit_configs()
            ).to_dict()
        return moveit_config


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

    def initialise_collision_scene(self, pcm, collision_object):
        with pcm.read_write() as scene:
            scene.apply_collision_object(collision_object)
            scene.current_state.update()  # Important to ensure the scene is updated
        return

    def define_state(self, robot_model, angles):
        state = RobotState(robot_model)
        state.set_joint_group_positions("manipulator", np.radians(angles))
        return state

    def plan_state_to_state(self, state_list):
        plans = []
        for state in state_list:
            joint_state = self.define_state(self.robot_model, state)
            self.arm.set_goal_state(robot_state=joint_state)
            # plan
            plan_result = self.arm.plan(self.plan_params)
            if plan_result:
                #record
                plans.append(plan_result)
                # set the target as the new start state
                self.arm.set_start_state(robot_state=joint_state)
            else:
                self.logger.error("Failed to plan")
        return plans

    def execute_plans(self, plans):
        for plan in plans:
            self.mvp.execute(plan.trajectory, controllers=[]) # only execute plans if you are sure of the outcome
        return
    
if __name__ == "__main__":

    #User Parameters
    robot_ip = "yyy.yyy.yyy.yyy"
    use_fake_hardware = "true"
    planner = "ompl_rrtc" # pilz_lin, ompl_rrtc, chomp_planner
    velocity_scaling = 0.5
    scene_file ='/home/karo/rosws/src/gen3_6dof_dslr_moveit_config/scene/desk_scene.scene'

    # Initialise MoveItPy node
    rclpy.init()
    IF = MoveItIF(robot_ip, use_fake_hardware, planner, velocity_scaling, scene_file)

    state_list = [[29,5,85,-86,-57,6], [-21,11,-69,84,71,-8],[47,93,-27,61,56,-49],[65,121,-20,68,78,-36]]
    plans = IF.plan_state_to_state(state_list)
    #IF.execute_plans(plans)