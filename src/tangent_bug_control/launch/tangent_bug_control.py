import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tangent_bug_control = get_package_share_directory('tangent_bug_control')

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_tangent_bug_control, 'rviz', 'tangent_bug_control.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Tangent Bug Control
    tangent_bug_control = Node(
        package='tangent_bug_control',
        executable='tangent_bug_control',
        name='tangent_bug_control',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_tangent_bug_control, 'worlds', 'world_6.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        gazebo,
        tangent_bug_control,
        rviz
    ])
