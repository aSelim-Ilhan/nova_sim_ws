from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('nova_description')
    urdf_file = os.path.join(pkg, 'urdf', 'nova.urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'nova.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz,
    ])
