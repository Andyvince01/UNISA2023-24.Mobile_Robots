import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    ## LANDMARKS FILE PATH ARGUMENT ##
    landmarks_arg = DeclareLaunchArgument(
        name='landmarks',
        default_value='src/nav_pkg/landmarks/diem_landmarks.json',
        description='Path to the landmarks file.'
    )
    
    # Declare the simulation launch argument
    on_turtlebot = DeclareLaunchArgument(
        name='on_turtlebot',
        default_value='true',
        description='Whether to run the test on Turtlebot or using the PC webcam.'
    )
    
    ## CREATE A LIST OF ACTIONS FOR THE LAUNCH DESCRIPTION ##
    actions = [
        landmarks_arg,
        on_turtlebot,
        Node(
            package='nav_pkg',
            executable='navigation',
            name='navigation',
            output='screen',
            parameters=[
                {'file_path': LaunchConfiguration('landmarks')},
                {'on_turtlebot': LaunchConfiguration('on_turtlebot')}
            ]
        ),
        Node(
            package='qr_commander',
            executable='qr_scanner',
            name='qr_scanner',
            output='screen',
            parameters=[
                {'on_turtlebot': LaunchConfiguration('on_turtlebot')}
            ]
        )
    ]
       
    ## APPEND THE ACTIONS TO THE LAUNCH DESCRIPTION ##
    ld = LaunchDescription(actions)
    return ld