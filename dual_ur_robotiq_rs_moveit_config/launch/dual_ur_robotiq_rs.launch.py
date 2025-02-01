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
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    gazebo_world_file_path = LaunchConfiguration('gazebo_world_file_path')
    prefix = LaunchConfiguration('prefix')
    namespace = LaunchConfiguration('namespace')
    sim_gazebo = LaunchConfiguration('sim_gazebo')
    use_planning = LaunchConfiguration('use_planning')
    left_arm_controller = LaunchConfiguration('left_arm_controller')
    left_gripper_controller = LaunchConfiguration('left_gripper_controller')
    right_arm_controller = LaunchConfiguration('right_arm_controller')
    right_gripper_controller = LaunchConfiguration('right_gripper_controller')
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    left_initial_positions_file = LaunchConfiguration('left_initial_positions_file')
    right_initial_positions_file = LaunchConfiguration('right_initial_positions_file')
    base_frame_file = LaunchConfiguration('base_frame_file')
    moveit_launch_file = LaunchConfiguration('moveit_launch_file')



    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dual_ur_robotiq_rs_moveit_config"), "launch", "dual_ur_robotiq_rs_control.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "launch_rviz": "false",
        }.items(),
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments={
            "use_sim_time": "true",
            "launch_rviz": "true",
        }.items(),
    )

    nodes_to_launch = [
        ur_control_launch,
        ur_moveit_launch,
    ]

    return nodes_to_launch