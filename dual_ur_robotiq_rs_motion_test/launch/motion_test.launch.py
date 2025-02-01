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
from launch.actions import TimerAction

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
)

def declare_arguments():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "package_name",
            default_value="dual_ur_robotiq_rs_motion_test",
            description="Python API tutorial file name",
        ),
        DeclareLaunchArgument(
            "test_name",
            default_value="motion_test_simple",
            description="Python API tutorial file name",
        ),
    ]

    return LaunchDescription(declared_arguments)


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_entity(declare_arguments())


    # Launch Controllers and Moveit Interface
    dual_ur_control_and_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dual_ur_robotiq_rs_moveit_config"), "launch", "dual_ur_robotiq_rs.launch.py"]
            )
        ),
    )
    ld.add_action(dual_ur_control_and_moveit)



    # Launch Tester
    moveit_py_node = Node(
        name="moveit_py",
        package=LaunchConfiguration("package_name"),
        executable=LaunchConfiguration("test_name"),
        output="screen",
        # parameters=[moveit_config.to_dict(),
        #             ],
    )
    ld.add_action(TimerAction(period=10.0, actions=[moveit_py_node]))
    # ld.add_action(moveit_py_node)



    return ld