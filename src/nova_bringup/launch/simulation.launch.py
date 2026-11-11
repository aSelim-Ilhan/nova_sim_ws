import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    desc_pkg = get_package_share_directory('nova_description')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    urdf_file  = os.path.join(desc_pkg, 'urdf', 'nova.urdf.xacro')
    world_file = os.path.join(desc_pkg, 'worlds', 'underwater.world')
    rviz_file  = os.path.join(desc_pkg, 'rviz', 'nova.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Gazebo'ya mesh path'ini bildir
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(desc_pkg, '..') + ':' +
        os.environ.get('GAZEBO_MODEL_PATH', '')
    )
    gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        desc_pkg + ':' +
        os.environ.get('GAZEBO_RESOURCE_PATH', '')
    )

    # Gazebo Classic başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items(),
    )

    # Robot modelini Gazebo'ya spawn et
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'nova',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    cmd_vel_to_wrench = Node(
        package='nova_control',
        executable='cmd_vel_to_wrench',
    )

    thruster_allocator = Node(
        package='nova_control',
        executable='thruster_allocator',
    )

    gazebo_bridge = Node(
        package='nova_control',
        executable='gazebo_bridge',
        output='screen',
    )

    hydrodynamics = Node(
        package='nova_control',
        executable='hydrodynamics',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo_resource_path,
        gazebo,
        robot_state_publisher,
        spawn,
        cmd_vel_to_wrench,
        thruster_allocator,
        hydrodynamics,
        rviz,
    ])
