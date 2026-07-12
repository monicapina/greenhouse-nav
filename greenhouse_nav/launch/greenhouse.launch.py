import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('greenhouse_nav')
    world_file = os.path.join(pkg_dir, 'worlds', 'greenhouse.world')

    return LaunchDescription([

        # Set TurtleBot3 model
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        # Launch Gazebo with our world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(
                    os.path.join(
                        get_package_share_directory('turtlebot3_gazebo'),
                        'urdf', 'turtlebot3_burger.urdf'
                    )
                ).read(),
                'use_sim_time': True
            }]
        ),

        # Spawn TurtleBot3 in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'burger',
                '-file', os.path.join(
                    get_package_share_directory('turtlebot3_gazebo'),
                    'models', 'turtlebot3_burger', 'model.sdf'
                ),
                '-x', '-2.5',
                '-y', '0.0',
                '-z', '0.05',
            ],
            output='screen'
        ),

    ])