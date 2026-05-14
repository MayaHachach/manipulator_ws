import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('manipulator_control')

    # ── 1. Launch robot + world with color-sorting camera pose ───────────
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'cam_x':     '0.3',
            'cam_y':     '0.0',
            'cam_z':     '1.5',
            'cam_roll':  '0.0',
            'cam_pitch': '1.5708',
            'cam_yaw':   '0.0',
        }.items()
    )

    # ── 2. Spawn colored blocks ───────────────────────────────────────────
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
        spawn_robot,
        spawn_blocks,
    ])