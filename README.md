# UNISA2023-24.Mobile_Robots

The primary objective of this project is to design and develop an advanced software system that enables the **Turtlebot4** to autonomously navigate within a predefined environment at the *DIEM* (Department of Industrial Engineering and Management). The project focuses on leveraging a priori knowledge of the environment's map, known as `@DIEM map` to facilitate the Turtlebot4's movement from any given starting position, ensuring it can discover and follow a path during navigation by interpreting commands provided through QR codes. 

![waypoints](https://github.com/user-attachments/assets/41d49bb8-b0f3-4a09-969a-b6ff68fcee9d)

This initiative employs a command-based navigation strategy where QR codes placed at various intersections within the environment serve as navigation beacons. These QR codes encode specific commands that direct the Turtlebot to perform actions such as `turning right`, `turning left`, `moving straight`, or `stopping`. The Turtlebot4's ability to autonomously read and interpret these commands is crucial for its successful navigation. The robot must traverse corridors, approach intersections, read the QR codes, and make informed decisions to follow the encoded instructions. This system is designed to handle an indeterminate number of obstacles that might be placed along the robot's path, ensuring robustness and adaptability in dynamic environments.


https://github.com/user-attachments/assets/b00867f3-f4d7-42aa-9545-7ef81df9b3e3


### 📁 File Hierarchy

- **Turtlebot4**: This directory houses scripts used throughout the course, including the exam script. Of particular interest for the project are files located in the subdirectory `turtlebot4/diem_turlebot_ws/src/`.

To navigate to this directory, use the command:
```bash
cd turtlebot4/diem_turlebot_ws/src/
```

Within this folder, various packages have been created for the course. Of these, the **`qr_commander`** package contains scripts for executing QR code detectors, while the **`nav_pkg`** package includes scripts for the navigation system. For the project, the essential files from the `nav_pkg` package are **`navigation.py`** and **`utils.py`**.

## 📷 Testing the QRScanner
This section aims to illustrate the steps necessary to test the `QRScanner` implementation using both the Turtlebot4's camera and the PC's camera.

### Testing using Turtlebot4's Camera
To test the code using the Turtlebot4's Camera you need to execture the following commands:

``` sh
source install/setup.bash

ros2 launch qr_commaner qr_launch.py on_turtlebot:=True
```

### Testing using PC's Camera
To test the code using the PC Camera you need to execture the following commands:

``` sh
source install/setup.bash

ros2 launch qr_commaner qr_launch.py on_turtlebot:=False
```

## 🛠 Testing on Turtlebot

This section aims to illustrate the steps necessary to connect and subsequently operate the Turtlebot4, ensuring that you can efficiently test the developed code on the robot.

### Connect to the Turtlebot

To test the code on the Turtlebot4, you need to connect to it first. A dedicated shell script (`turtlebot_init.sh`) has been created to streamline this process by including the essential commands needed to establish the connection.

After connecting to the same network as the Turtlebot4, execute the following command in the terminal:

```sh
./turtlebot_init.sh
```

This script performs the following steps:

```sh
# Install the `ntpdate` package used to synchronize the system clock with a remote time server.
sudo apt install -y ntpdate

# Synchronize the system clock with the ntp.ubuntu.com time server. Accurate timekeeping is essential for various applications.
sudo ntpdate ntp.ubuntu.com

# Download and execute a shell script for configuring the Turtlebot4.
wget -qO - https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/humble/turtlebot4_discovery/configure_discovery.sh | bash <(cat) </dev/tty

# Reload the user's bash configuration to apply any changes made by the script.
source ~/.bashrc

# Restart the ROS 2 daemon to ensure all nodes are managed correctly.
ros2 daemon stop; ros2 daemon start
```

### Move the Turtlebot

To move the Turtlebot4, you need to execute the following commands in separate terminal tabs:

1. **Load the Map:**

   Launch the `localization.launch.py` file from the `turtlebot4_navigation` package and specify the map file `diem_map.yaml` located in the `src/map` directory.

   ```sh
   ros2 launch turtlebot4_navigation localization.launch.py map:=src/map/diem_map.yaml
   ```

2. **Run Navigation Stack:**

   Launch the `nav2.launch.py` file from the `turtlebot4_navigation` package to start the navigation functionality of the robot.

   ```sh
   ros2 launch turtlebot4_navigation nav2.launch.py
   ```

3. **Open RViz:**

   Launch the `view_robot.launch.py` file from the `turtlebot4_viz` package to set up the visualization environment for the robot.

   ```sh
   ros2 launch turtlebot4_viz view_robot.launch.py
   ```

4. **Run Custom Navigation Implementation:**

   Launch the `nav1.launch.py` file from the `nav_pkg` package to run your specific navigation implementation for the `@DIEM` map.

   ```sh
   source install/setup.bash
   ros2 launch nav_pkg nav1.launch.py 
   ```

   It is also possible to specifiy the filepath of the file containg the coordinate of the landmarks (`filepath:=/home/andyv/Desktop/turtlebot4/diem_turtlebot_ws/src/nav_pkg/landmarks/diem_landmarks.json`) and the angular speed of the Turtlebot4 (`angular_speed:=0.3`).

After executing these commands, you need to select the Turtlebot4's initial position and orientation in RViz. By default, the Turtlebot4 will start moving straight with the command `Commands.Straighton`.
