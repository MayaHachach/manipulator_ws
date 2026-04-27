import os
import xacro
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Paths
    urdf_path = os.path.join(
        get_package_share_directory('open_manipulator_description'),
        'urdf', 'open_manipulator_x', 'open_manipulator_x.urdf.xacro'
    )

    world_path = os.path.join(
        get_package_share_directory('manipulator_control'),
        'worlds', 'table_world.sdf'
    )

    # Read URDF as string
    #with open(urdf_path, 'r') as f:
    #    robot_description = f.read()
    
    controller_config = os.path.join(
        get_package_share_directory('open_manipulator_bringup'),
        'config', 'open_manipulator_x', 'hardware_controller_manager.yaml'
    )

    robot_description = xacro.process_file(urdf_path).toxml()

    return LaunchDescription([

        # 1. Start Gazebo with our world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
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

        # 4. Controller Manager — load controllers
        # Wait 3 seconds for Gazebo to fully spawn the robot first
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
            period=2.0,
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

        # Bridge camera topic from Gazebo to ROS 2
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

    ])
