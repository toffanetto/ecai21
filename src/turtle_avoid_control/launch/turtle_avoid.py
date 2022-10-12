from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():

        turtlesim_node = Node(
                package='turtlesim',
                executable='turtlesim_node',
                name='turtlesim'
        )

        turtle_avoid_node = Node(
                package='turtle_avoid_control',
                executable='turtle_avoid',
                name='turtle_avoid',
                output='screen'
        )

        return LaunchDescription([
                turtlesim_node,
                turtle_avoid_node
        ])