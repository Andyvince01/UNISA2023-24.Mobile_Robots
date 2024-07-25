# Load the map
ros2 launch turtlebot4_navigation localization.launch.py map:=src/map/diem_map.yaml

# Run navigation stack
ros2 launch turtlebot4_navigation nav2.launch.py

# Open Rviz
ros2 launch turtlebot4_viz view_robot.launch.py

# Run our navigation implementation for @Diem map
ros2 launch nav_pkg nav1.launch.py