
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bag_record = LaunchConfiguration('bag_record')
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_record',
            default_value='False'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'turtlebot3_gazebo'), 'launch'), '/turtlebot3_world.launch.py'
            ])
        ),

        Node(
            package='walker_algorithm_robot',
            executable='roomba_walker',
        ),

        ExecuteProcess(
            condition=IfCondition(bag_record),
            cmd=['ros2', 'bag', 'record', '-a', '-x /camera.+'],
            shell=True
        )

    ])