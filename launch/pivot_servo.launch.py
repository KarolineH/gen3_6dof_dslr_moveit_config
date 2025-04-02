
import os
import yaml

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.actions import Node
import launch_ros
from launch_param_builder import ParameterBuilder

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(file_path):
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    
def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    planning_scene_file = get_package_share_directory("gen3_6dof_dslr_moveit_config") + "/scene/desk_scene.scene"
    pivot_offset = 0.4

    launch_arguments = {
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "gripper": "false",
        "dof": "6",
        "vision": "true",
    }

    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="gen3_6dof_dslr_moveit_config")
        .robot_description(mappings=launch_arguments)
        #.planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(publish_robot_description_semantic=True)
        .moveit_cpp(file_path=get_package_share_directory("gen3_6dof_dslr_moveit_config") + "/config/moveit_cpp.yaml")
        .to_moveit_configs()
    )

    moveit_config.moveit_cpp.update({"use_sim_time": use_sim_time.perform(context) == "true"})

    scene_pub_node = Node(
        name="scene_publisher",
        package="gen3_6dof_dslr_moveit_config",
        executable="apply_plan_scene.py",
        output="both",
        parameters=[planning_scene_file],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # # pivot frame broadcaster
    # pivot_frame_broadcaster = Node(
    #     package="gen3_6dof_dslr_moveit_config",
    #     executable="pivot_tf_broadcaster.py",
    #     parameters=[{'pivot_offset': pivot_offset}]
    # )

    ### SERVO STUFF HERE #######
    servo_config_file = get_package_share_directory("gen3_6dof_dslr_moveit_config") + "/config/servo.yaml"
    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml(servo_config_file)
        .to_dict()
    }
    print(f"Built servo params: {servo_params}")

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "manipulator"}
    
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    joy_node = Node(
        name="joystick_node",
        package="joy",
        executable="joy_node",
    )

    joy_to_servo_node = Node(
        name="joy_to_servo_node",
        package="gen3_6dof_dslr_moveit_config",
        executable="joy_to_servo_input.py",
    )

    ############################

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("gen3_6dof_dslr_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.to_dict(), ros2_controllers_path],
        output="both",
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # robot_pos_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["twist_controller", "--inactive", "-c", "/controller_manager"],
    # )

    # fault_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["fault_controller", "-c", "/controller_manager"],
    # )

    # rviz with moveit configuration
    rviz_config_file = (
        get_package_share_directory("gen3_6dof_dslr_moveit_config")
        + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    nodes_to_start = [
        #moveit_py_node,
        scene_pub_node,
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        robot_traj_controller_spawner,
        # robot_pos_controller_spawner,
        #fault_controller_spawner,
        move_group_node,
        static_tf,
        servo_node,
        joy_node,
        joy_to_servo_node,
        #pivot_frame_broadcaster,
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated clock",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
