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

def configure_moveit(robot_ip, use_fake_hardware):
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

def initialise_collision_scene(pcm):
    with pcm.read_write() as scene:
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

        scene.apply_collision_object(collision_object)
        scene.current_state.update()  # Important to ensure the scene is updated

def define_state(robot_model, angles):
    state = RobotState(robot_model)
    state.set_joint_group_positions("manipulator", np.radians(angles))
    return state

def main(moveit_config, planner, velocity_scaling):
    # initialise
    rclpy.init()
    logger = get_logger("moveit_py_planning_scene")
    mvp = MoveItPy(node_name="moveit_py", config_dict=moveit_config)
    pcm = mvp.get_planning_scene_monitor()
    initialise_collision_scene(pcm)
    arm = mvp.get_planning_component("manipulator")
    params = PlanRequestParameters(mvp, planner) # change planner settings here
    params.max_velocity_scaling_factor = velocity_scaling

    initial_state = arm.get_start_state()
    robot_model = initial_state.robot_model

    # a list of joint states, in order
    state_list = [[29,5,85,-86,-57,6], [-21,11,-69,84,71,-8],[47,93,-27,61,56,-49],[65,121,-20,68,78,-36]]
    movements = []

    for state in state_list:
        joint_state = define_state(robot_model, state)
        arm.set_goal_state(robot_state=joint_state)
        # plan
        plan_result = arm.plan(params)
        if plan_result:
            #record
            movements.append(plan_result)
            # set the target as the new start state
            arm.set_start_state(robot_state=joint_state)
            #mvp.execute(plan_result.trajectory, controllers=[]) # only execute plans if you are sure of the outcome
        else:
            logger.error("Failed to plan")

    return movements

if __name__ == "__main__":

    #User Parameters
    robot_ip = "yyy.yyy.yyy.yyy"
    use_fake_hardware = "true"
    planner = "ompl_rrtc"
    velocity_scaling = 0.1

    #MoveItPy Configuration
    config_dict = configure_moveit(robot_ip, use_fake_hardware)

    plans = main(config_dict, planner, velocity_scaling)
    print(plans)
