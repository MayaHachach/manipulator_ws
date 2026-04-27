import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('manipulator_control')

    # ── 1. Launch robot + world + camera + controllers ────────────────────────
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'spawn_robot.launch.py')
        )
    )

    # ── 2. Spawn colored blocks after Gazebo is fully ready ───────────────────
    #       8s delay gives Gazebo + controllers time to initialize
    spawn_blocks = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='manipulator_control',
                executable='spawn_blocks',
                name='block_spawner',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        spawn_robot,   # starts everything: robot, world, camera, controllers
        spawn_blocks,  # then spawns the 5 colored blocks
    ])
