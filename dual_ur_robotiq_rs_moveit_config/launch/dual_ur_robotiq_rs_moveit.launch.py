from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

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

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import ExecuteProcess
from pathlib import Path
from launch.event_handlers import OnProcessExit
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


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
            default_value="",
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
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
            'use_sim_time',
            default_value='true',
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
    declared_arguments.append(
        DeclareLaunchArgument(
            'warehouse_sqlite_path',
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description='Configuration file of robot base frame wrt World.',
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo", default_value="false", description="Launch Servo?"
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "test_mode",
            default_value="true",
            description="Python API tutorial file name",
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "test_name",
            default_value="motion_test_arm",
            description="Python API tutorial file name",
        ),
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
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')
    launch_servo = LaunchConfiguration('launch_servo')
    test_name = LaunchConfiguration('test_name')
    test_mode = LaunchConfiguration('test_mode')

    

    #
    base_package_name = "dual_ur_robotiq_rs_moveit_config"
    moveit_config = (
        MoveItConfigsBuilder(robot_name="dual_ur_robotiq_rs", package_name=base_package_name)
        .robot_description_semantic(Path("srdf") / "dual_ur_robotiq_rs.srdf.xacro")
        .moveit_cpp(
            file_path=get_package_share_directory(base_package_name)
            + "/config/moveit_cpp.yaml"
        )
        .planning_pipelines("ompl", ["ompl"])
        # .robot_description_kinematics(
        #     file_path=get_package_share_directory(base_package_name)
        #     + "/config/kinematics.yaml"
        # )
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )
    # 



    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description": True,
                "publish_robot_description_semantic": True,
            }
        ],
    )

    servo_yaml = load_yaml(base_package_name, "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
        output="screen",
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare(base_package_name),
        "rviz","dual_ur_robotiq_rs.rviz"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time
            },
        ],
        condition=IfCondition(start_rviz),
    )

    # Motion Tester
    motion_test_node = Node(
        name="motion_test_node",
        package="dual_ur_robotiq_rs_motion_test",
        executable="motion_test_dual_ur",
        output="both",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": use_sim_time}
                    ],
        condition=IfCondition(test_mode),
    )

    ld = LaunchDescription()
    ld.add_entity(LaunchDescription(declared_arguments))
    ld.add_action(wait_robot_description)
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group_node, motion_test_node, rviz_node, servo_node],
            )
        ),
    )



    return ld