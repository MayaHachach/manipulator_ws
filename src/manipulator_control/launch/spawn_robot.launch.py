import os
import xacro
import tempfile
from jinja2 import Template
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    # ── Resolve camera pose from launch arguments ─────────────────────────
    cam_x     = context.perform_substitution(LaunchConfiguration('cam_x'))
    cam_y     = context.perform_substitution(LaunchConfiguration('cam_y'))
    cam_z     = context.perform_substitution(LaunchConfiguration('cam_z'))
    cam_roll  = context.perform_substitution(LaunchConfiguration('cam_roll'))
    cam_pitch = context.perform_substitution(LaunchConfiguration('cam_pitch'))
    cam_yaw   = context.perform_substitution(LaunchConfiguration('cam_yaw'))

    # ── Paths ─────────────────────────────────────────────────────────────
    urdf_path = os.path.join(
        get_package_share_directory('open_manipulator_description'),
        'urdf', 'open_manipulator_x', 'open_manipulator_x.urdf.xacro'
    )
    world_template_path = os.path.join(
        get_package_share_directory('manipulator_control'),
        'worlds', 'table_world.sdf.jinja'
    )

    robot_description = xacro.process_file(urdf_path).toxml()

    # ── Render Jinja template → temporary plain SDF ───────────────────────
    with open(world_template_path, 'r') as f:
        template = Template(f.read())

    rendered_sdf = template.render(
        cam_x     = float(cam_x),
        cam_y     = float(cam_y),
        cam_z     = float(cam_z),
        cam_roll  = float(cam_roll),
        cam_pitch = float(cam_pitch),
        cam_yaw   = float(cam_yaw),
    )

    # Write rendered SDF to a temp file — gz sim reads a normal .sdf
    tmp_sdf = tempfile.NamedTemporaryFile(
        mode='w', suffix='.sdf', delete=False, prefix='table_world_'
    )
    tmp_sdf.write(rendered_sdf)
    tmp_sdf.flush()
    tmp_sdf_path = tmp_sdf.name
    tmp_sdf.close()

    print(f'[spawn_robot] Camera pose: x={cam_x} y={cam_y} z={cam_z} '
          f'roll={cam_roll} pitch={cam_pitch} yaw={cam_yaw}')
    print(f'[spawn_robot] Rendered world SDF: {tmp_sdf_path}')

    return [
        # 1. Start Gazebo with rendered plain SDF
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', tmp_sdf_path],
            output='screen'
        ),

        # 2. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        # 3. Spawn robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'open_manipulator_x',
                '-string', robot_description,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.76',
            ],
            output='screen'
        ),

        # 4. Controllers
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                ),
            ]
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['arm_controller'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['gripper_controller'],
                    output='screen'
                ),
            ]
        ),
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '--once',
                        '/arm_controller/joint_trajectory',
                        'trajectory_msgs/msg/JointTrajectory',
                        '{"joint_names": ["joint1", "joint2", "joint3", "joint4"], '
                        '"points": [{"positions": [3.14159, 0.0, 0.0, 0.0], '
                        '"time_from_start": {"sec": 2, "nanosec": 0}}]}'
                    ],
                    output='screen'
                )
            ]
        ),

        # 5. Bridge camera topic
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/gz/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/world/manipulator_world/create@ros_gz_interfaces/srv/SpawnEntity',
            ],
            output='screen'
        ),
    ]


def generate_launch_description():
    return LaunchDescription([

        # ── Camera pose arguments ─────────────────────────────────────────
        DeclareLaunchArgument('cam_x',     default_value='1.0'),
        DeclareLaunchArgument('cam_y',     default_value='0.0'),
        DeclareLaunchArgument('cam_z',     default_value='1.15'),
        DeclareLaunchArgument('cam_roll',  default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.6'),
        DeclareLaunchArgument('cam_yaw',   default_value='3.1415'),

        OpaqueFunction(function=launch_setup),
    ])