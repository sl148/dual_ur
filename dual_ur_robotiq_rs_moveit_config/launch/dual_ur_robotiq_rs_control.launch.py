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
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch import LaunchContext
from launch.utilities import perform_substitutions

import shutil

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
)
import subprocess

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
            'gazebo_world_file',
            default_value="workstation.sdf",
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
            'use_sim_time',
            default_value='true',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )


    # Initialize Arguments
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    gazebo_world_file = LaunchConfiguration('gazebo_world_file')
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
    use_sim_time = LaunchConfiguration('use_sim_time')

    # File path
    controllers_file_path = PathJoinSubstitution(
        [FindPackageShare(description_package), 'config', controllers_file,]
    )
    left_initial_positions_file_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', left_initial_positions_file,]
    )
    right_initial_positions_file_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', right_initial_positions_file,]
    )
    rviz_config_file_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'rviz', rviz_config_file]
    )
    base_frame_file_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', base_frame_file]
    )


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]
            ), 
            ' ',
            'ur_type:=',
            'ur5',
            ' ',
            'sim_gazebo:=', 
            sim_gazebo, 
            ' ', 
            'simulation_controllers:=',
            controllers_file_path,
            ' ',
            'left_initial_positions_file:=', 
            left_initial_positions_file_path,
            ' ', 
            'right_initial_positions_file:=',
            right_initial_positions_file_path,
            ' ', 
            'base_frame_file:=',
            base_frame_file_path,
        ]
    )
    robot_description = {'robot_description': robot_description_content}


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   [namespace, 'controller_manager']],
    )
    


    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[left_arm_controller, '--controller-manager', [namespace, 'controller_manager']],
    )

    left_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[left_gripper_controller, '--controller-manager', [namespace, 'controller_manager']],
    )

    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[right_arm_controller, '--controller-manager', [namespace, 'controller_manager']],
    )

    right_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[right_gripper_controller, '--controller-manager', [namespace, 'controller_manager']],
    )

    both_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["scaled_joint_trajectory_controller", '--controller-manager', [namespace, 'controller_manager']],
    )

    
    # Delay start of robot controllers after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner, left_gripper_controller_spawner, right_arm_controller_spawner, right_gripper_controller_spawner],
        )
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            # "-name",
            # "ur",
            # "-allow_renaming",
            # "true",
        ],
    )

    # world_path = os.path.join(get_package_share_directory('dual_ur_robotiq_rs_moveit_config'),'gazebo','workstation.world')
    # destination_path = "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds"
    # shutil.copy(world_path, destination_path)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                sim_gazebo,
                if_value=[" -r -v 4 ", gazebo_world_file],
                else_value=[" -s -r -v 4 ", gazebo_world_file],
            )
        }.items(),
    )

    # Make the /clock topic available in ROS
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )


    nodes = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,

        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,

        spawn_entity,
        gazebo,
        gz_sim_bridge,

    ]

    return LaunchDescription(declared_arguments + nodes)