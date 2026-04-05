import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the launch directory
    description_pkg_path = get_package_share_directory('ovis_description')

    # Get the URDF file (robot)
    urdf_path = os.path.join(description_pkg_path, 'config', 'ovis.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path, ' hardware_type:=', "ovis"]), value_type=str)

    # Takes the description and joint angles as inputs and publishes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {"use_sim_time": True, }
        ]
    )

    # kinova driver
    kinova_joint_driver = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_desc},
            os.path.join(description_pkg_path, 'config', 'ros2_controllers.yaml'),

        ],
    )

    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg_path, 'launch', 'spawn_controllers.launch.py')
        )
    )

    return LaunchDescription([
            robot_state_publisher,
            kinova_joint_driver,
            spawn_controllers_launch,
            ])
