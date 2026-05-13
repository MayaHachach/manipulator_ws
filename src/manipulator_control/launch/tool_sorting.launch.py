import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('manipulator_control')

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'spawn_robot.launch.py')
        )
    )

    # Delay so Gazebo + bridge are ready before spawn_tools uses SpawnEntity
    spawn_tools = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='manipulator_control',
                executable='spawn_tools',
                name='tool_spawner',
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        spawn_robot,
        spawn_tools,
    ])
