import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro

import os
os.environ['LC_NUMERIC'] = "en_US.UTF-8" # Fix for URDF not showing correctly in RViz


def generate_launch_description():
    return LaunchDescription([
        get_state_publisher_gui_node(),
        get_rviz_node(),
        get_state_publisher_node(),
    ])


def get_rviz_node():
    rviz_config_dir = os.path.join(
        get_package_share_directory('clumsybot_description'),
        'rviz',
        'model.rviz')

    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')


def get_state_publisher_gui_node():
    return Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        # condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

def get_state_publisher_node():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
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
                'use_sim_time': use_sim_time,
                'robot_description': urdf_contents
            }])
