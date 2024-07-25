import json, rclpy
import numpy as np

from geometry_msgs.msg import Twist, Point, Quaternion
from launch_ros.actions import Node
from rclpy.clock import Duration
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from ..utils._pose import Pose
from ..utils._orientation import Orientation
from ..utils._euler import Euler
from ..utils.misc import *

# =============================================================================
# Handler class to move Turtlebot4 trought the landmarks in the map.
# =============================================================================
class Handler(Node):
    '''
    Handler class to move Turtlebot4 trought the landmarks in the map.
    
    Attributes:
        - angular_speed (float): Angular speed in rad/s (default: 1.57 rad/s - maximal speed: 1.90 rad/s).
        - cmd_pub (Publisher): Publisher to publish the Twist message.
        - landmarks (list): List of landmarks.
        - linear_speed (float): Linear speed in m/s (default: 0.25 m/s - maximal speed: 0.31 m/s).
        - mean (float): Mean value of the noise (default: 0).
        - msg (Twist): Twist message.
        - std_dev (float): Standard deviation of the noise (default: 0 - no noise).
        - ok (bool): Flag to control the movement of the robot.
    '''
    __slots__ = ['angular_speed', 'cmd_pub', 'landmarks', 'linear_speed', 'lock', 'mean', 'msg', 'std_dev', 'ok']

    ## --- CONSTRUCTOR --- ##
    def __init__(self):
        '''
        Initialize the Handler class.
        '''
        try:
            ## INITIALIZATION OF THE NODE ##
            super().__init__('nav1')
            self.get_logger().set_level(LoggingSeverity.DEBUG)      # Set the logging level of the node
            
            ## READING PARAMETERS FROM THE LAUNCH FILE ##
                        
            # Get the file path from the parameter server 
            self.declare_parameter('file_path', '/home/andyv/Desktop/turtlebot4/diem_turtlebot_ws/src/nav_pkg/Landmarks/landmarks.json')
            file_path = self.get_parameter('file_path').get_parameter_value().string_value

            # Get the linear speed from the parameter server
            self.declare_parameter('linear_speed', 0.1)
            self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value if self.get_parameter('linear_speed').get_parameter_value().double_value <= 0.31 else 0.31

            # Get the angular speed from the parameter server
            self.declare_parameter('angular_speed', 0.1)
            self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value if self.get_parameter('angular_speed').get_parameter_value().double_value <= 1.90 else 1.90

            # Get the mean value from the parameter server
            self.declare_parameter('mean', 0.0)
            self.mean = self.get_parameter('mean').get_parameter_value().double_value
            
            # Get the standard deviation from the parameter server (default value: 0 - no noise)
            self.declare_parameter('std_dev', 1.0)
            self.std_dev = self.get_parameter('std_dev').get_parameter_value().double_value
           
            ## READING THE LANDMARKS FROM THE JSON FILE ##
            self.landmarks = self._read_landmarks(file_path=file_path, verbose=False)
                        
            ## INITIALIZATION OF THE PUBLISHER##
            self.cmd_pub = self.create_publisher(msg_type=Twist, topic="/cmd_vel", qos_profile=10)
            
            ## INITIALIZATION OF THE TIMER ##
            self.create_timer(timer_period_sec=0.1, callback=self._action_callback)
            
            ## INITIALIZATION OF THE FLAG 'OK' IN ORDER TO CONTROL THE MOVEMENT OF THE ROBOT ##
            self.ok = False
            
        except Exception as e:
            self.get_logger().error(f"Error during the initialization of the Turtlebot4: {e}")        
    
    ## --- PUBLIC METHODS --- ## 
    def loop(self) -> None:
        '''
        Loop to move the robot through the landmarks in the map.
        '''
        self.get_logger().info("Starting the navigation...")
        self._sleep(1)
        
        # Current pose of the robot
        current_pose : Pose = self.landmarks[0]

        # Move the robot through the landmarks in the map
        for target_pose in self.landmarks[1:]:

            # Add Gaussian noise to the target pose (default: no noise)
            target_pose = add_gaussian_noise(pose=target_pose, mean=self.mean, std_dev=self.std_dev)
            
            self.get_logger().info(f"Moving the robot from the current pose {current_pose} to the target pose {target_pose}...")
            
            # First Rotation of the robot to the target pose 
            angle_rot1 = np.arctan2(target_pose.position.y - current_pose.position.y, target_pose.position.x - current_pose.position.x) - current_pose.orientation.euler.yaw
            self._rotate(angle_rot1)
            
            # Moving the robot to the target pose
            distance : float = distance_between_poses(current_pose, target_pose)
            self._move(distance)
                       
            # Second Rotation of the robot to the target pose
            angle_rot2 = target_pose.orientation.euler.yaw - current_pose.orientation.euler.yaw
            self._rotate(angle_rot2)
            
            # Update the current pose
            current_pose = target_pose
            
            # Sleep for 1 second 
            self._sleep(1) 

        self.get_logger().info("Navigation completed.")
    
    def stop(self) -> None:
        '''
        Stop the robot.        
        '''
        try:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.ok = True
            self.cmd_pub.publish(msg)
            self.ok = False
        except Exception as e:
            self.get_logger().error("Error during the stop of the robot".format(e))
    
    ## --- PRIVATE METHODS --- ##
    def _action_callback(self) -> None:
        '''
        Callback function to publish the Twist message.
        '''
        if self.ok:
            # self.get_logger().debug("Publishing the Twist message {}".format(self.msg))
            self.cmd_pub.publish(self.msg)
    
    def _read_landmarks(self, file_path: str, verbose: bool = False) -> list:
        '''
        Read the landmarks from a JSON file.
        
        args:
            - file_path (str): Path to the JSON file.
            - verbose (bool): Verbose mode.
        
        return:
            - landmarks (list): List of landmarks.
        '''
        landmarks = []
        with open(file_path, 'r') as file:
            poses = json.load(file)
            for _, value in poses.items():
                # Get the position and orientation of the landmark
                pos, ori = value['pose']['position'], value['pose']['orientation']
                # Create a Point object with the position of the landmark
                pos = {k: float(v) for k, v in pos.items()}
                position = Point(**pos)
                # Create an Orientation object with the orientation of the landmark
                ori = {k: float(v) for k, v in ori.items()}
                # If the orientation is given only in quaternion, then create an Orientation object with the quaternion
                if 'x' in ori and 'y' in ori and 'z' in ori and 'w' in ori and len(ori) == 4:
                    orientation = Orientation(quaternion=Quaternion(**ori))
                elif 'roll' in ori and 'pitch' in ori and 'yaw' in ori and len(ori) == 3:
                    orientation = Orientation(euler=Euler(**ori))
                else:
                    orientation = Orientation(euler=Euler(**ori), quaternion=Quaternion(**ori))
                # Create a Pose object with the position and orientation of the landmark
                pose = Pose(position=position, orientation=orientation)
                landmarks.append(pose)
                  
        if verbose:
            self.get_logger().debug(f"Landmarks: {landmarks}")
        
        return landmarks
    
    def _move(self, distance : float) -> None:
        '''
        Move the robot to the target pose.
        
        args:
            - distance (float): Distance in meters to move the robot.
        '''
        # If the distance is 0, the robot is already in the target pose
        if distance == 0:
            return
        
        # Time to reach the target pose
        move_duration : float = distance / self.linear_speed
                
        self.get_logger().debug('Moving the robot to the target pose {:.2f} m away at a speed of {:.4f} m/s... for {:.4f} s'.format(distance, self.linear_speed, move_duration))
        
        try:
            msg = Twist()
            msg.linear.x = self.linear_speed
            self.msg = msg
            self.ok = True
            self._sleep(move_duration)
            self.ok = False
        except Exception as e:
            self.get_logger().error("Error during the movement of the robot: {}".format(e))
                
    def _rotate(self, angle_rot : float) -> None:
        '''
        Rotate the robot to the target orientation.
        
        args:
            - angle_rot (float): Angle in radians to rotate the robot.
        '''
        # If the angle is 0, the robot is already in the target orientation
        if angle_rot == 0:
            return
        
        # Time to rotate the robot to the target orientation
        rotate_duration : float = float(abs(angle_rot) / self.angular_speed)
        
        self.get_logger().debug('Angle to rotate: {:.4f} rad'.format(angle_rot))
        
        self.get_logger().debug('Rotating the robot at a speed of {:.4f} rad/s... for {:.4f} s'.format(self.angular_speed, rotate_duration))
        
        try:
            msg = Twist()
            msg.angular.z = self.angular_speed * np.sign(angle_rot)
            self.msg = msg
            self.ok = True
            self._sleep(rotate_duration)
            self.ok = False
        except Exception as e:
            self.get_logger().error("Error during the rotation of the robot: {}".format(e))
            
    def _sleep(self, time_seconds: float) -> None:
        '''
        Sleep for a given time in seconds.
        
        args:
            - time_seconds (float): Time in seconds.
        '''
        self.get_clock().sleep_for(Duration(seconds=time_seconds))
    
def main():
    '''
    Main function to move the Turtlebot4 through the landmarks in the map.
    '''
    # Initialize the ROS 2 node
    rclpy.init()
    # Create the executor
    executor = MultiThreadedExecutor()
    # Create the handler
    handller = Handler()
    # Set the parameter of the node for the simulation. This forces the node to use the ROS clock instead of the system clock. 
    # Necessary to synchronize with the Gazebo clock.
    param = rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)    # Create the parameter of the node 'use_sim_time'
    handller.set_parameters([param])                                            # Set the parameter of the node 'use_sim_time'
    executor.add_node(handller)                                                 # Add the node to the executor
    executor.create_task(handller.loop)                                         # Create a task with the callable provided as input
    try:
        executor.spin()                                                         # Execute the loop - blocking call
    except KeyboardInterrupt:
        handller.stop()
    # Stop the robot
    handller.destroy_node()                                                     # Good practice to destroy the node
    rclpy.shutdown()                                                            # Shutdown the ROS 2 node

if __name__ == '__main__':
    main()