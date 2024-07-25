import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare the on_turtlebot launch argument
    on_turtlebot = DeclareLaunchArgument(
        name='on_turtlebot',
        default_value='true',
        description='Whether to run the test on Turtlebot or using the PC webcam.'
    )
    
    # Create a list of actions for the launch description
    actions = [
        on_turtlebot,
        Node(
            package='qr_commander',
            executable='qr_scanner',
            name='qr_scanner',
            output='screen',
            parameters=[
                {'on_turtlebot': LaunchConfiguration('on_turtlebot')}
            ]
        ),
        Node(
            package='qr_commander',
            executable='camera',
            name='camera',
            output='screen'
        )
    ]
       
    # Append the actions to the launch description
    ld = LaunchDescription(actions)
    return ld