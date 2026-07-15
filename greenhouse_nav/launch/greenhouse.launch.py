import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable, 
                            ExecuteProcess, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

"""Launch file for greenhouse navigation simulation.

This launch description starts Gazebo with the greenhouse.world, spawns a
TurtleBot3 Burger model, and then brings up Nav2 with the provided map and
parameter file. It also sets required environment variables for TurtleBot3
and Gazebo.
"""
"""
run command:

cd ~/repositories/greenhouse-nav
colcon build --symlink-install
conda deactivate
source /opt/ros/humble/setup.bash
source ~/repositories/greenhouse-nav/install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch greenhouse_nav greenhouse.launch.py



"""

def generate_launch_description():

    #Finds where the package is installed.
    pkg_dir = get_package_share_directory('greenhouse_nav') 
    nav2_bringup = get_package_share_directory('nav2_bringup')

    world_file = os.path.join(pkg_dir, 'worlds', 'greenhouse.world')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_dir, 'maps', 'greenhouse_map.yaml')

    # Load the TurtleBot3 URDF file.
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf', 'turtlebot3_burger.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # Environment variables
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            '/opt/ros/humble/share/turtlebot3_gazebo/models'
        ),

        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen' #or 'log'
        ),

        # Robot state publisher. Publish the TF tree, the geometric relationships between links.
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

        # Spawn TurtleBot3. Simulate the robot physically. Add '-timeout', '60' to avoid Gazebo spawn issues using Docker.
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'burger',
                        '-file', os.path.join(
                            get_package_share_directory('turtlebot3_gazebo'),
                            'models', 'turtlebot3_burger', 'model.sdf'
                        ),
                        '-x', '0.0', 
                        '-y', '0.0',
                        '-z', '0.05',
                         
                    ],
                    output='screen'
                ),
            ]
        ),

        # Launch Nav2 after Gazebo is ready
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
                    ),
                    launch_arguments={
                        'map': map_file,
                        'params_file': nav2_params,
                        'use_sim_time': 'true',
                    }.items(),
                ),
            ]
        ),      
    ])