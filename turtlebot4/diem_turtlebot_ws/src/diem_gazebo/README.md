# How to run

## Run Empty Map
1. ``` source install/setup.bash ```
2. ``` ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py ```

## Run Square Map
1. ``` source install/setup.bash ```
2. ``` ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py world:=square ```

### Run Square Map Rviz
1. ``` source install/setup.bash ```
2. ``` ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py world:=square rviz:=true ```

## Run Warehouse Map
1. ``` source install/setup.bash ```
2. ``` ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py world:=warehouse ```