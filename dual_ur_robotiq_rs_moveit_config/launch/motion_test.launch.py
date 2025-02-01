from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
)

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='dual_ur_robotiq_rs_moveit_config',
            description='Package with the controller\'s configuration in "config" folder. \
                         Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value='controllers.yaml',
            description='YAML file with the controllers configuration.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='dual_ur_robotiq_rs_moveit_config',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='dual_ur_robotiq_rs.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gazebo_world_file_path',
            default_value=os.path.join(
                get_package_share_directory("dual_ur_robotiq_rs_moveit_config"),
                    "gazebo",
                    "workstation.world",
                ),
            description='gazebo world file with the robot',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<prefix>/"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim_gazebo',
            default_value='true',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_planning',
            default_value='true',
            description='Start robot with Moveit2 `move_group` planning \
                         config for Pilz and OMPL.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'left_arm_controller',
            default_value='left_ur_arm_controller',
            description='arm controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'left_gripper_controller',
            default_value='left_gripper_controller',
            description='gripper controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'right_arm_controller',
            default_value='right_ur_arm_controller',
            description='arm controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'right_gripper_controller',
            default_value='right_gripper_controller',
            description='gripper controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file', 
            default_value='dual_ur_robotiq_rs.rviz',
            description='Rviz file'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'left_initial_positions_file',
            default_value='left_initial_positions.yaml',
            description='Configuration file of robot initial positions for simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'right_initial_positions_file',
            default_value='right_initial_positions.yaml',
            description='Configuration file of robot initial positions for simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'base_frame_file',
            default_value='base_frame.yaml',
            description='Configuration file of robot base frame wrt World.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='dual_ur_robotiq_rs_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_launch_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("dual_ur_robotiq_rs_moveit_config"),
                    "launch",
                    "dual_ur_robotiq_rs_moveit.launch.py",
                ]
            ),
            description="Absolute path for MoveIt launch file, part of a config package with robot SRDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom moveit config.",
        )
    )
    return LaunchDescription(declared_arguments)


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_entity(generate_launch_description())


    dual_ur_control_and_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dual_ur_robotiq_rs_moveit_config"), "launch", "dual_ur_robotiq_rs.launch.py"]
            )
        ),
    )
    ld.add_action(dual_ur_control_and_moveit)



    kg_planner_data_gen_node = DeclareLaunchArgument(
        "motion_test",
        default_value="motion_simple_test",
        description="Python API tutorial file name",
    )
    ld.add_action(kg_planner_data_gen_node)

    moveit_py_node = Node(
        name="moveit_py",
        package="dual_ur_robotiq_rs_moveit_config",
        executable=LaunchConfiguration("motion_test"),
        output="both",
        # parameters=[moveit_config.to_dict()],
    )
    ld.add_action(moveit_py_node)


    return ld