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

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='ur_robotiq_rs_moveit_config',
            description='Package with the controller\'s configuration in "config" folder. \
                         Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='ur_robotiq_rs_moveit_config',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='ur_robotiq_rs.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
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
            'start_rviz',
            default_value='false',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file', 
            default_value='ur_robotiq_rs.rviz',
            description='Rviz file'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'base_frame_file',
            default_value='base_frame.yaml',
            description='Configuration file of robot base frame wrt World.',
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    base_frame_file = LaunchConfiguration('base_frame_file')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # File path
    rviz_config_file_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'rviz', rviz_config_file]
    )
    base_frame_file_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', base_frame_file]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]
            ),
            ' ',
            'ur_type:=',
            'ur5e',
            ' ',
            'tf_prefix:=',
            prefix,
            ' ',
            'base_frame_file:=',
            base_frame_file_path,
        ]
    )

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
        # it is the save way to wrap the xacro output
        # ref: https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/
    }

    # Get SRDF via xacro
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "srdf", "ur_robotiq_rs.srdf.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )

    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # Get planning parameters
    robot_description_planning_joint_limits = PathJoinSubstitution([
            FindPackageShare(description_package), "moveit2", "joint_limits.yaml",
        ]
    )

    robot_description_planning_cartesian_limits = PathJoinSubstitution([
            FindPackageShare(description_package), "moveit2", "cartesian_limits.yaml",
        ]
    )

    move_group_capabilities = {
        "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction \
            pilz_industrial_motion_planner/MoveGroupSequenceService"""
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(description_package), "moveit2", "kinematics.yaml"]
    )

    planning_pipelines_config = PathJoinSubstitution([
            FindPackageShare(description_package), "moveit2", "planning_pipelines_config.yaml",
        ]
    )

    ompl_planning_config = PathJoinSubstitution([
            FindPackageShare(description_package), "moveit2", "ompl_planning.yaml",
        ]
    )

    moveit_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package),
            "moveit2", "moveit_controllers.yaml"]
    )

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description":True,
        "publish_robot_description_semantic":True,
    }



    moveit_config = (
        MoveItConfigsBuilder("ur_robotiq_rs")
        .robot_description(file_path='urdf/ur_robotiq_rs.urdf.xacro')
        .robot_description_semantic(file_path='srdf/ur_robotiq_rs.srdf.xacro')
        .robot_description_kinematics(file_path='moveit2/kinematics.yaml')
        .joint_limits(file_path='moveit2/joint_limits.yaml')
        .trajectory_execution(file_path='moveit2/moveit_controllers.yaml')
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # robot_description,
            # robot_description_semantic,
            # robot_description_kinematics,
            # robot_description_planning_cartesian_limits,
            # robot_description_planning_joint_limits,
            # planning_pipelines_config,
            # ompl_planning_config,
            # trajectory_execution,
            # moveit_controllers,
            # planning_scene_monitor_parameters,
            # move_group_capabilities,
            # {"use_sim_time": use_sim_time},
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory("ur_robotiq_rs_description"),
        "rviz/ur_robotiq_rs.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    
    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ur_robotiq_rs_moveit_config"),
        "config/",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
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

 



    # # rviz_node = Node(
    # #     package='rviz2',
    # #     executable='rviz2',
    # #     name='rviz2',
    # #     output='log',
    # #     arguments=['-d', rviz_config_file_path],
    # #     parameters=[
    # #         robot_description,
    # #         robot_description_semantic,
    # #         robot_description_planning_cartesian_limits,
    # #         robot_description_planning_joint_limits,
    # #         robot_description_kinematics,
    # #         planning_pipelines_config,
    # #         ompl_planning_config,
    # #     ],
    # #     condition=IfCondition(start_rviz),
    # # )

    nodes = [
        rviz_node,
        # static_tf_node,
        # robot_state_publisher,
        move_group_node,
        # ros2_control_node,
        # joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
