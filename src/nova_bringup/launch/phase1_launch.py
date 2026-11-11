from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nova_localization'),
                'launch', 'localization_launch.py'
            )
        )
    )

    dvl_node = Node(
        package='nova_drivers',
        executable='dvl_node',
        name='dvl_node',
        output='screen',
    )

    ping360_node = Node(
        package='nova_drivers',
        executable='ping360_node',
        name='ping360_node',
        output='screen',
    )

    pid_controller = Node(
        package='nova_control',
        executable='pid_controller',
        name='pid_controller',
        output='screen',
    )

    thruster_allocator = Node(
        package='nova_control',
        executable='thruster_allocator',
        name='thruster_allocator',
        output='screen',
    )

    return LaunchDescription([
        localization_launch,
        dvl_node,
        ping360_node,
        pid_controller,
        thruster_allocator,
    ])
