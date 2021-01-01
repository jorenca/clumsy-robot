import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro

import os
os.environ['LC_NUMERIC'] = "en_US.UTF-8" # Fix for URDF not showing correctly in RViz

USE_SIM_TIME = LaunchConfiguration('use_sim_time', default='true')

def generate_launch_description():
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription(
        [
            # DeclareLaunchArgument(
            #     'use_sim_time',
            #     default_value='true',
            #     description='Use simulation (Gazebo) clock if true'),

            get_state_publisher_node(),

            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': USE_SIM_TIME}],
                # arguments=['-configuration_directory', cartographer_config_dir,
                           # '-configuration_basename', configuration_basename]
           ),

            Node(
                package='cartographer_ros',
                executable='occupancy_grid_node',
                name='occupancy_grid_node',
                output='screen',
                parameters=[{'use_sim_time': USE_SIM_TIME}],
                # arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
            ),

            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([nav2_launch_file_dir, '/nav2_bringup_launch.py']),
            #     launch_arguments={
            #         'map': './mapout.yaml',
            #         'use_sim_time': USE_SIM_TIME,
            #         # 'params': param_dir
            #         }.items(),
            # ),
        ]
    )


def get_state_publisher_node():
    xacro_file = os.path.join(
        get_package_share_directory('clumsybot_description'),
        'urdf',
        'clumsybot.urdf.xacro')

    urdf_contents = xacro.process_file(xacro_file).toprettyxml(indent='  ')
    print("URDF robot description file :" + xacro_file)
    print(urdf_contents)

    return Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': USE_SIM_TIME,
                'robot_description': urdf_contents
            }])
