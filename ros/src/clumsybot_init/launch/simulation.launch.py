from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


import os
os.environ['LIBGL_ALWAYS_INDIRECT'] = "0" # Fix for Gazebo black screen

def generate_launch_description():
    return LaunchDescription(
        start_simulation()
    )


def start_simulation():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'houses.model'  #'turtlebot3_houses/waffle.model'
    world = os.path.join(get_package_share_directory('clumsybot_init'), 'worlds', world_file_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    print('Using %s as a world file' % world)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        Node( # Read from /robot_description topic and spawn entity in Gazebo
            name='urdf_spawner',
            namespace='',
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments='-x 0 -y 0.2 -z 0 -R 0 -P 0 -Y 0 -unpause -entity clumsybot -topic robot_description'.split()
        ),
    ]
