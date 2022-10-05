from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():

    teleop_node = Node(
                package='teleop_cmd_vel',
                executable='teleop',
                name='teleop',
                output='screen',
                prefix=["konsole -e"]
    )
    
    turtlesim_node = Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
    )

    return LaunchDescription([
            teleop_node,
            turtlesim_node
        ])