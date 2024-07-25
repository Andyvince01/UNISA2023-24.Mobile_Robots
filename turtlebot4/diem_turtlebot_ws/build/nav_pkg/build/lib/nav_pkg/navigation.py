import json, rclpy
from typing import Tuple
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np

from geometry_msgs.msg import Quaternion, Pose, PoseWithCovarianceStamped, Twist 
from launch_ros.actions import Node
from rclpy.clock import Duration
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

from .utils import Commands, read_graph, get_next_waypoint, convert_to_pose

INITIAL_COVARIANCE = [0.2369682970362255, 0.009736844469753965, 0.0, 0.0, 0.0, 0.0, 0.009736844469753968, 0.23796459081181498, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06790577539928601]

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
    __slots__ = ['_graph', 'angular_speed', 'cmd_pub', 'current_cmd', 'current_pose', 'current_wp', 'linear_speed', 'ok']

    ## --- CONSTRUCTOR --- ##
    def __init__(self, simulation : bool = False, debug: bool = False):
        '''
        Initialize the Handler class.
        '''
        try:
            # Initialize the node
            super().__init__('navigation')
            # Set the logging level of the node
            self.get_logger().set_level(LoggingSeverity.INFO)
                                   
            # Get the file path from the parameter server (default: '/home/andyv/Desktop/turtlebot4/diem_turtlebot_ws/src/nav_pkg/Landmarks/diem_landmarks.json')
            self.declare_parameter('file_path', '/home/andyv/Desktop/turtlebot4/diem_turtlebot_ws/src/nav_pkg/landmarks/diem_landmarks.json')
            file_path = self.get_parameter('file_path').get_parameter_value().string_value

            # Create a graph from the JSON file of the DIEM map of the landmarks
            V, E = read_graph(file_path=file_path)
            self._graph = nx.Graph()
            self._graph.add_nodes_from(V)
            self._graph.add_edges_from(E)
            
            if debug:
                pos = nx.spring_layout(self._graph)
                nx.draw(self._graph, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=20, font_color='black', font_weight='bold', font_family='sans-serif')
                plt.show()
                
            self.simulation = simulation    
            
            self.current_cmd = Commands.STRAIGHTON
            self.current_pose = None
            self.last_wp = None
                                       
            # Initialize the publisher
            self.rviz_pub = self.create_publisher(msg_type=PoseWithCovarianceStamped, topic="/initialpose", qos_profile=10)
            
            # Initialize the subscriber(s)
            self.qr_sub = self.create_subscription(msg_type=String, topic="/cmd_qrscan", callback=self.__get_command, qos_profile=10)
            
            self.amcl_sub = self.create_subscription(msg_type=PoseWithCovarianceStamped, topic="/amcl_pose", callback=self.__get_pose, qos_profile=10)
                        
            # Initialize the flag to control the movement of the robot
            self.ok = False
            
            # Initialize the Turtlebot4 Navigator
            self.navigator = TurtleBot4Navigator()
            
        except Exception as e:
            self.get_logger().error(f"Error during the initialization of the Turtlebot4: {e}")        
    
    ## --- PUBLIC METHODS --- ## 
    def loop(self) -> None:
        ''' Loop to move the robot through the landmarks in the map. '''
        self.get_logger().info("Starting the navigation...")
        self._sleep(1)

        # Start on dock
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before intialising pose')
            self.navigator.dock()
        
        while not self.current_pose:
            self.get_logger().info("Waiting for the AMCL pose...")
            self._sleep(1)    
        
        # Set the initial pose of the robot
        initial_pose = self.navigator.getPoseStamped((self.current_pose.position.x, self.current_pose.position.y), TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        # Wait for Nav2 to be active (that is, the navigation stack is ready to receive goals)
        # NOTE: This function makes the node wait until the localizer and the navigator are ready to receive goals.
        # It also waits for the initial pose to be set.
        self.navigator.waitUntilNav2Active()
        
        # Set the goal poses of the robot to move through the landmarks in the map depending on the command received from the QR code
        # NOTE: The default command is STRAIGHTON. This command will be executed until a new command is received from the QR code.
        while self.current_cmd != Commands.STOP:
            # If the command is not received, wait for the command
            if not self.current_cmd:
                self.get_logger().info("Waiting for the command...")
                self._sleep(1)
                continue
                        
            # If the robot is kidnapped, then repostion the robot to the nearest waypoint
            if self.current_pose.position.z > 0.1:
                self.get_logger().info("[KIDNAPPED] Repositioning the robot to the nearest waypoint...")
                pose_stamped = PoseWithCovarianceStamped()
                pose_stamped.pose.pose.position = self.last_wp.position
                pose_stamped.pose.pose.orientation = self.last_wp.orientation
                pose_stamped.pose.covariance = INITIAL_COVARIANCE
                self.rviz_pub.publish(pose_stamped)
                self.navigator.clearAllCostmaps()
                continue
                                       
            # Get the nearest waypoint from the current pose.
            # NOTE: The robot may not be exactly on the waypoint, so we need to find the nearest waypoint to the current pose.
            vertices = list(self._graph.nodes)
            distances = [np.linalg.norm(np.array([self.current_pose.position.x, self.current_pose.position.y]) - np.array([v[0], v[1]])) for v in vertices]
            nearest_vertex = vertices[np.argmin(distances)]
            current_wp = nearest_vertex

            self.get_logger().info("Last waypoint: ", self.last_wp)

            # Get the next waypoint (position, rotation) to move the robot
            self.get_logger().info(f"Direction: {self.current_cmd.value}")
            next_position, next_rotation = get_next_waypoint(point=self.current_pose, nearest_waypoint=current_wp, graph=self._graph, direction=self.current_cmd.value)
            self.get_logger().info(f"Next waypoint: {next_position}. Next rotation: {next_rotation}")
            # # Move the robot to the next waypoint
            
            self.get_logger().info(f"Moving the robot to the next waypoint: {next_position}.")
            self.get_logger().info(f"Rotation: {next_rotation}.")
            
            if next_position is not None and next_rotation is not None:
                # Move the robot to the next waypoint
                self.navigator.startToPose(self.navigator.getPoseStamped(position=next_position, rotation=next_rotation))
                # Update the current waypoint
                self.last_wp = convert_to_pose(position=current_wp, orientation=[0.0, 0.0, next_rotation]) 
                # Clear costmaps
                self.navigator.clearAllCostmaps()
                
            self.get_logger().info("Last waypoint: ", self.last_wp)
        
        self.get_logger().info("Navigation finished.")
    
    def __get_command(self, msg: String) -> None:
        '''This function gets the command from the QR code and stores it in the variable 'command'.

        Parameters
        ----------
        msg : String
            The message received from the QR code (e.g., forward, backward, left, right, stop, etc.)
        '''
        
        # Get the command from the QR code
        command = msg.data
        self.current_cmd = Commands(command)
    
    def __get_pose(self, msg: PoseWithCovarianceStamped) -> None:
        '''This function gets the pose of the robot and stores it in the variable 'pose'.

        Parameters
        ----------
        msg : PoseWithCovarianceStamped
            The message received from the AMCL pose.
        '''        
        # Get the pose of the robot
        pose = msg.pose.pose
        self.current_pose = pose
        
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
        pass

    handller.destroy_node()                                                     # Good practice to destroy the node
    rclpy.shutdown()                                                            # Shutdown the ROS 2 node

if __name__ == '__main__':
    main()