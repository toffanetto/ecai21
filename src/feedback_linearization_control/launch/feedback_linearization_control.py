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
    pkg_feedback_linearization_control = get_package_share_directory('feedback_linearization_control')

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
        arguments=['-d', os.path.join(pkg_feedback_linearization_control, 'rviz', 'feedback_linearization_control.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Feedback Linearization Control
    feedback_linearization_control = Node(
        package='feedback_linearization_control',
        executable='feedback_linearization_control',
        name='feedback_linearization_control',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_feedback_linearization_control, 'worlds', 'world_empty.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        gazebo,
        feedback_linearization_control,
        rviz
    ])
