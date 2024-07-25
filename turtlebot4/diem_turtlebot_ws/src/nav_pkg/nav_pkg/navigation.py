import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import rclpy

from geometry_msgs.msg import Quaternion, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Twist
from irobot_create_msgs.msg import KidnapStatus
from launch_ros.actions import Node
from rclpy.clock import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import String
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TaskResult
from typing import Tuple

from .utils import Commands, read_graph, get_next_waypoint, convert_to_pose, euler_from_quaternion

class MessageWrapper:
    ''' MessageWrapper class to wrap the message received from the topic. '''
    
    def __init__(self, topic_name : str, message : PoseWithCovarianceStamped) -> None:
        ''' Initialize the MessageWrapper class. '''        
        self.topic_name = topic_name
        self.message = message

class Handler(Node):
    ''' Handler class to move Turtlebot4 trought the landmarks in the @DIEM map.  '''
    
    # Define the slots of the class to optimize the memory usage
    __slots__ = ['angular_speed', 'current_cmd', 'current_pose', 'initial_covariance', 'is_kidnapped', 'last_cmd', 'last_kidnapped', 'last_wp', 'next_wp', 'navigator', 'ok', 'rviz', 'cmd_pub', 'rviz_pub', 'amcl_sub', 'initial_pose_sub', 'kidnapped_sub', 'qr_sub', '_graph', 'msg']
    
    # Define the constructor of the class
    def __init__(self, debug: bool = False):
        ''' Initialize the Handler class.

        :param bool debug: True to show the graph of the landmarks in the map, False otherwise (defaults to False)
        '''
        try:
            # Initialize the node
            super().__init__('navigation')
            self.get_logger().set_level(LoggingSeverity.INFO)                           # Set the logging level of the node
                                   
            # Get the file path from the parameter server (default: '/home/andyv/Desktop/turtlebot4/diem_turtlebot_ws/src/nav_pkg/Landmarks/diem_landmarks.json')
            self.declare_parameter('file_path', '/home/andyv/Desktop/turtlebot4/diem_turtlebot_ws/src/nav_pkg/landmarks/diem_landmarks.json')
            file_path : str = self.get_parameter('file_path').get_parameter_value().string_value

            # Get the angular speed from the parameter server
            self.declare_parameter('angular_speed', 0.3)
            self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value if self.get_parameter('angular_speed').get_parameter_value().double_value <= 1.90 else 1.90

            # Create a graph from the JSON file of the @DIEM map of the landmarks
            V, E = read_graph(file_path=file_path)
            self._graph = nx.Graph()
            self._graph.add_nodes_from(V)
            self._graph.add_edges_from(E)
            
            if debug:
                pos = nx.spring_layout(self._graph)
                nx.draw(self._graph, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=20, font_color='black', font_weight='bold', font_family='sans-serif')
                plt.show()
        
            # Initialize the Turtlebot4 Navigator
            self.navigator = TurtleBot4Navigator()
                   
            # Initialize the parameters for the movement of the robot (current command, current pose, kidnapped status, last command, last kidnapped status, last waypoint, next waypoint, and rviz)
            self.current_cmd = Commands.STRAIGHTON
            self.current_pose : PoseWithCovariance = None
            self.initial_covariance = None
            self.is_kidnapped : bool = False
            self.last_cmd : Commands = Commands.STRAIGHTON
            self.last_kidnapped : bool = False
            self.last_wp : Pose = None
            self.next_wp : Pose = None
            self.rviz : bool = False
                                       
            # Initialize the publisher(s)
            self.cmd_pub = self.create_publisher(
                msg_type=Twist, 
                topic="/cmd_vel", 
                qos_profile=10
            )
            
            self.rviz_pub = self.create_publisher(
                msg_type=PoseWithCovarianceStamped,
                topic="/initialpose",
                qos_profile=10
            )
            
            # Initialize the subscriber(s)
            self.amcl_sub = self.create_subscription(
                msg_type=PoseWithCovarianceStamped, 
                topic="/amcl_pose", 
                callback=lambda msg: self._get_pose(MessageWrapper('/amcl_pose', msg)),
                qos_profile=10
            )
            
            self.initial_pose_sub = self.create_subscription(
                msg_type=PoseWithCovarianceStamped,
                topic="/initialpose",
                callback=lambda msg: self._get_pose(MessageWrapper('/initialpose', msg)),
                qos_profile=10
            )
            
            self.kidnapped_sub = self.create_subscription(
                msg_type=KidnapStatus, 
                topic="/kidnap_status", 
                callback=self._get_kidnapped,
                qos_profile=QoSProfile(depth=10, history=2, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
            )

            self.qr_sub = self.create_subscription(
                msg_type=String, 
                topic="/cmd_qr", 
                callback=self._get_command, 
                qos_profile=10
            )
            
            # Initialize the timer to publish the Twist message
            self.create_timer(timer_period_sec=0.1, callback=self._action_callback)
                        
            # Initialize the flag to control the movement of the robot
            self.ok = False
        
        except Exception as e:
            self.get_logger().error("Error during the initialization of the Turtlebot4: {e}".format(e=e))        
    
    ## --- PUBLIC METHODS --- ## 
    def loop(self) -> None:
        ''' Loop to move the robot through the landmarks in the map. 
        
        This function initializes the robot's position, waits for the navigation stack to be ready, and then 
        enters a loop where it continuously navigates through a series of waypoints. The robot follows commands 
        received from QR codes, with a default behavior of moving straight ahead until a new command is received. 
        
        If the robot is detected to be kidnapped (unexpectedly moved), it will reposition itself to the previous 
        known waypoint and reset its pose.
        '''
        self.get_logger().info("[Turtlebot4 Navigation] Starting the navigation 12345...")
        self._sleep(5)
                
        # Wait for the AMCL pose to be received
        while not self.current_pose:
            continue
                
        # Set the initial pose of the robot to the current pose received from the AMCL.
        # NOTE: The initial pose is set only once at the beginning of the navigation. 
        self.get_logger().info("[Turtlebot4 Navigation] Setting the initial pose... {}".format(self.current_pose.pose))
        initial_pose : PoseStamped = self.navigator.getPoseStamped([self.current_pose.pose.position.x, self.current_pose.pose.position.y], np.rad2deg(euler_from_quaternion(self.current_pose.pose.orientation)[2]))
        self.initial_covariance = self.current_pose.covariance
        self.navigator.setInitialPose(initial_pose)
        
        self.navigator.clearAllCostmaps()
                
        # Wait for Nav2 to be active (that is, the navigation stack is ready to receive goals)
        # NOTE: This function makes the node wait until the localizer and the navigator are ready to receive goals.
        # It also waits for the initial pose to be set.
        self.navigator.waitUntilNav2Active()
                        
        # Set the goal poses of the robot to move through the landmarks in the map depending on the command received from the QR code
        # NOTE: The default command is STRAIGHTON. This command will be executed until a new command is received from the QR code.
        while self.current_cmd != Commands.STOP:
                
            # Not doing anything if the robot is kidnapped and the call to the _kidnapped_manager() is triggered by the callback
            if self.last_kidnapped is True or self.is_kidnapped is True:
                continue
       
            # Handle the case when the robot doesn't receive the command from the QR code
            self._null_command_manager()
                            
            # Inform the user about the current command
            self.get_logger().info("[Turtlebot4 Navigation] Current command: {}".format(self.current_cmd))
                           
            # If the command is STOP, stop the robot! 
            if self.current_cmd == Commands.STOP:
                self.get_logger().info("[Turtlebot4 Navigation] Stop command received. Stopping the robot...")
                self.navigator.clearAllCostmaps()
                break
                                                                                                              
            # Get the nearest waypoint from the current pose.
            # NOTE: The robot may not be exactly on the waypoint, so we need to find the nearest waypoint to the current pose.
            vertices = list(self._graph.nodes)            
            distances = [np.linalg.norm(np.array([self.current_pose.pose.position.x, self.current_pose.pose.position.y]) - np.array([v[0], v[1]])) for v in vertices]
            current_wp = vertices[np.argmin(distances)]

            self.get_logger().info("[Turtlebot4 Navigation] Current waypoint: {current_wp}".format(current_wp=current_wp))

            # Get the next waypoint (position, rotation) to move the robot
            try:
                next_position, next_rotation = get_next_waypoint(point=self.current_pose.pose, nearest_waypoint=current_wp, graph=self._graph, direction=self.current_cmd.value)
            except ValueError:
                self.get_logger().error("[Turtlebot4 Navigation] Error during the calculation of the next waypoint. The robot is not aligned with the waypoints!")
                self._kidnapped_manager()
                continue
            
            # Move the robot to the next waypoint
            if next_position is None or next_rotation is None:
                self.get_logger().info("[Turtlebot4 Navigation] No next waypoint found. Stopping the robot...")
                self._kidnapped_manager()
                continue

            self.get_logger().info("[Turtlebot4 Navigation] Next waypoint: {next_position}, {next_rotation}".format(next_position=next_position, next_rotation=next_rotation))

            # Reset the current command to None after moving to the next waypoint.
            # Update the last waypoint and the last command variables by saving the current waypoint and the current command.
            self.last_wp = convert_to_pose(position=list(current_wp), orientation=list([0.0, 0.0, next_rotation])) if next_rotation else None
            self.next_wp = next_position
            self.last_cmd = self.current_cmd
            self.current_cmd = None
            self.rviz = False

            # Move the robot to the next waypoint
            self._navigation_manager(next_position, next_rotation)
            
            # Clear costmaps to avoid obstacles in the path and replan the path to the goal.
            self.navigator.clearAllCostmaps()
            
            # Pause for 2 seconds before the next waypoint
            self._sleep(2)
            
        # Stop the robot. The robot has reached the final destination.            
        self.get_logger().info("[Turtlebot4 Navigation] Navigation finished.")
            
    # ==============================================================================================================
    # PRIVATE METHODS
    # ==============================================================================================================
    def _action_callback(self) -> None:
        ''' Callback function to publish the Twist message. '''
        if self.ok:
            # self.get_logger().info("Publishing the Twist message {}".format(self.msg))
            self.cmd_pub.publish(self.msg)
    
    def _attempt_to_acquire_command(self) -> None:
        ''' This function attempts to acquire the command from the QR code by rotating the robot of 30 degrees for 6 times.'''       
        for i in range(-3, 4):
            # Check if the attempt to acquire the command is successful. If successful, break the loop.
            if self.current_cmd:
                self.get_logger().info("--- [Acquire Command] Command acquired successfully: {}".format(self.current_cmd))
                break
            # Perform a 30-degree rotation to acquire the command from the QR code.
            angle_rot = - np.pi / 6 * np.sign(i) if i != 0 else - np.pi / 2
            # Rotate the robot by 30 degrees.
            self._rotate(angle_rot)
            # Pause for 2 seconds before the next rotation.
            self._sleep(2) 

    def _get_command(self, msg: String) -> None:
        '''This function gets the command from the QR code and stores it in the variable `self.command`.

        :param String msg: The message received from the QR code (e.g., `STRAIGHTON`, `GOBACK`, `LEFT`, `RIGHT` and `STOP`.)
        '''
        
        # Get the command from the QR code
        command = msg.data
        
        # Check if the command is valid or not
        if not command:
            return
                
        try:    
            # We can acquire the command from the QR code only if the robot is navigating to the next waypoint (current_cmd is None and last_cmd is not None)       
            if self.current_cmd is None and self.last_cmd is not None:
                # If the robot is in the junction, then acquire the command from the QR code
                self.get_logger().info("--- [Command Callback] Current Pose: {current_pose}".format(current_pose=self.current_pose.pose))
                if self._is_in_junction(self.current_pose.pose, self.next_wp):
                    # Cancel the last task if the robot is doing any task of the navigation stack
                    self.navigator.cancelTask()
                    # Set the current command to the command received from the QR code
                    self.current_cmd = Commands(command)
    
        except ValueError:
            self.get_logger().error("*** [Command Callback] Invalid command received from the QR code: {}".format(command))
            self.current_cmd = None
            
    def _get_kidnapped(self, msg: KidnapStatus) -> None:
        '''This function gets the kidnapped status of the robot and stores it in the variable `self.is_kidnapped`.
        
        :param KidnapStatus msg: The message received from the kidnapped status.
        '''       
         
        # Store the last kidnapped status
        self.last_kidnapped = self.is_kidnapped
        # Store the current kidnapped status
        self.is_kidnapped = msg.is_kidnapped        
                             
        # Handle the case when the robot is kidnapped while navigating through the landmarks in the map
        if (self.last_kidnapped is False and self.is_kidnapped is True) and not self.navigator.isTaskComplete():
            self.get_logger().info("--- [Kidnap Status] Robot has been kidnapped while navigating through the landmarks in the map...")
        elif (self.last_kidnapped is True and self.is_kidnapped is False) and not self.navigator.isTaskComplete():
            self.get_logger().info("--- [Kidnap Status] Robot has been repositioned after being kidnapped while navigating through the landmarks in the map...")
            self._kidnapped_manager()
    
    def _get_pose(self, wrapped_msg: MessageWrapper) -> None:
        '''This function gets the pose of the robot and stores it in the variable `self.current_pose`.
        The pose of the robot is received from the `/amcl` or the `/initialpose` topic. When the robot is kidnapped, 
        the pose of the robot is not read from the `/amcl` topic. This is because the AMCL pose is not reliable when
        the robot is kidnapped. In this case, the pose of the robot is read from the `/initialpose` topic.   

        :param MessageWrapper wrapped_msg: The message received from the topic.
        
        '''
        
        if self.is_kidnapped is True:
            # Not reading the pose of the robot from the AMCL if the robot is kidnapped
            if wrapped_msg.topic_name == '/amcl_pose':
                return
            # Update the initial pose of the robot if the robot is kidnapped
            elif wrapped_msg.topic_name == '/initialpose':
                self.rviz = True
    
        # self.get_logger().info("*** [Pose Callback] Message received from the topic {topic_name}: {msg}".format(topic_name=topic_name, msg=msg))       
        self.current_pose : PoseWithCovariance = wrapped_msg.message.pose
    
    def _is_in_junction(self, current_pose : Pose, next_wp : Tuple[float, float]) -> bool:
        '''This function checks if the robot is in a junction. If the robot is in a junction, then it sends a Service Request to the camera node to detect the QR code.
        In this way, the robot can acquire the command from the QR code and navigate to the next waypoint. The junction is the area where the QR code is placed. 
        The robot needs to detect the QR code to acquire the command and navigate to the next waypoint. We represent the junction as a rectangular area where the robot
        can detect the QR code, where the center of the junction is the next waypoint.

        :param Pose current_pose: This is the current waypoint where the robot is located.
        :param Tuple[float, float] next_wp: This is the next waypoint where the robot needs to move.
        :return bool: True if the robot is in a junction, False otherwise.
        '''
                        
        # Get the current position of the robot
        x, y = current_pose.position.x, current_pose.position.y
        
        # Get the next position of the robot
        x_next, y_next = next_wp[0], next_wp[1]
        
        # self.get_logger().info("--- [Junction] Current position: ({x}, {y}) - Next position: ({x_next}, {y_next})".format(x=x, y=y, x_next=x_next, y_next=y_next))
        
        # Create a rectangular area around the next waypoint to detect the QR code. The rectangular area is defined by the width (7.0 m) and the height (5.0 m).
        if (x_next - 3.5) <= x <= (x_next + 3.5) and (y_next - 2.5) <= y <= (y_next + 2.5):
            self.get_logger().info("--- [Junction] Turtlebot is in the junction.")
            return True
        
        return False
    
    # def _move(self, distance : float) -> None:
    #     '''This function is responsible for moving the robot to the target pose.

    #     :param float distance: The distance in meters to move the robot to the target pose.
    #     '''

    #     # If the distance is 0, the robot is already in the target pose
    #     if distance < 0.1:
    #         return
        
    #     # Time to reach the target pose
    #     move_duration : float = distance / self.linear_speed
                
    #     self.get_logger().debug('Moving the robot to the target pose {:.2f} m away at a speed of {:.4f} m/s... for {:.4f} s'.format(distance, self.linear_speed, move_duration))
        
    #     try:
    #         msg = Twist()
    #         msg.linear.x = self.linear_speed
    #         self.msg = msg
    #         self.ok = True
    #         self._sleep(move_duration)
    #         self.ok = False
    #     except Exception as e:
    #         self.get_logger().error("Error during the movement of the robot: {}".format(e))
    
    def _kidnapped_manager(self) -> None:
        ''' This function repositions the robot to the previous waypoint if the robot is detected to be kidnapped.
        
        This function is triggered when the robot's position indicates that it has been moved unexpectedly (kidnapped). 
        It repositions the robot to the position specified in RVIZ. If a position is not specified in RVIZ, the future 
        position of the robot is assumed to be the last known waypoint.
        '''         
          
        # Wait for the robot to be kidnapped
        i = 0
        while not self.is_kidnapped and not self.last_kidnapped:
            if i == 0:
                self.get_logger().info("--- [Kidnapped Manager] Waiting for the robot to be kidnapped...")
            if self.current_cmd is not None:
                return
            i += 1
            continue

        # self.get_logger().info("--- [Kidnapped Manager] Robot is kidnapped.")

        # Wait for the robot to be repositioned on the floor after being kidnapped (kidnapped = False and last_kidnapped = True)
        i = 0
        while not (self.last_kidnapped is True and self.is_kidnapped is False):
            if i == 0:
                self.get_logger().info("--- [Kidnapped Manager] Turtlebot is kidnapped. Waiting for the robot to be repositioned on the floor...") 
            if self.current_cmd is not None:
                return
            i += 1
            continue
                
        self.get_logger().info("--- [Kidnapped Manager] Robot is repositioned on the floor.")
        self._sleep(2)            

        if self.current_cmd is not None:
                return

        # SCENARIO 1: THE NEW POSE IS SET IN RVIZ
        # If the robot is repositioned to the new pose in RVIZ, then the robot will have two possible scenarios:
        #    1. THE ROBOT WAS KIDNAPPED WHILE NAVIGATING THROUGH THE LANDMARKS IN THE MAP: In this case, the task is not canceled, and the robot will continue to navigate,
        #       but from the new pose specified in RVIZ.
        #    2. THE ROBOT WAS KIDNAPPED BECAUSE IT DIDN'T RECEIVE THE COMMAND FROM THE QR CODE: In this case, the robot is expected to acquire the command from the QR code.
        #       If the robot doesn't receive the command from the QR code, then the robot will ask again to be kidnapped.
        if self.rviz:
            # self._rotate(0.1)
            self.get_logger().info("--- [Kidnapped Manager] RVIZ is set. Repositioning the robot to the new pose... {}".format(self.current_cmd))
            return
        
        # SCENARIO 2: THE NEW POSE IS NOT SET IN RVIZ
        # In this case, we ASSUME that the robot is repositioned to the last known waypoint. n this case the current command is STRAIGHTON. This is due to the fact that the robot
        # is repositioned with the orientation of the next waypoint. If the robot was kidnapped while navigating through the landmarks in the map, then the task is canceled, and the
        # robot will start a new navigation task from the new pose. 
        self.get_logger().info("--- [Kidnapped Manager] RVIZ is not set. Repositioning the robot to the last known waypoint...")
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = self.last_wp.position.x
        pose.pose.pose.position.y = self.last_wp.position.y
        pose.pose.pose.position.z = self.last_wp.position.z
        pose.pose.pose.orientation.x = self.last_wp.orientation.x
        pose.pose.pose.orientation.y = self.last_wp.orientation.y
        pose.pose.pose.orientation.z = self.last_wp.orientation.z
        pose.pose.pose.orientation.w = self.last_wp.orientation.w
        pose.pose.covariance = self.initial_covariance
        # Publish the pose to the RVIZ
        self.rviz_pub.publish(pose)
        # Set the current command to STRAIGHTON
        self.current_cmd = Commands.STRAIGHTON
        # Cancel the last task if the robot is doing any task of the navigation stack
        if not self.navigator.isTaskComplete():
            self.get_logger().info("--- [Kidnapped Manager] Canceling the last task...")
            self.navigator.cancelTask()           
        # Pause for 2 seconds before the next waypoint
        self._sleep(2)  
    
    def _navigation_manager(self, next_position : Pose, next_rotation : Quaternion) -> None:
        '''This function is responsible for moving the robot to the next waypoint in the map. 
        The function `startToPose` sends the goal to the navigator to move the robot to the next waypoint in the map.
        It is used for direct navigation to a single goal pose, focusing on reaching one final destination efficiently.

        :param Pose next_position: The next position to move the robot to.
        :param Quaternion next_rotation: The next rotation to move the robot to.
        '''
        
        # Create the goal pose to move the robot to the next waypoint in the map.
        goal = self.navigator.getPoseStamped(position=next_position, rotation=next_rotation)
        # Send the goal to the navigator to move the robot to the next waypoint in the map.
        # NOTE: The robot will move to next waypoint by adjusting its speed and orientation in order to reach the goal.
        # When the robot reaches the goal, the navigator will stop the robot.
        self.navigator.startToPose(goal)
        self._sleep(2)
        
        # self._logger.info("--- [Navigation Manager] Task result: {}".format(self.navigator.getResult()))
        
        # Adjust the robot orientation to the target orientation.
        # NOTE: The robot may not be exactly in the target orientation, so we need to adjust the orientation of the robot.
        if self.navigator.getResult() == TaskResult.SUCCEEDED or self.navigator.getResult() == TaskResult.CANCELED:
            angle_rot = next_rotation - euler_from_quaternion(self.current_pose.pose.orientation)[2]
            # Normalize the angle between -pi and pi
            angle_rot += -np.sign(angle_rot) * 2 * np.pi if abs(angle_rot) > np.pi else 0
            self.get_logger().info("--- [Navigation Manager] Adjusting the robot orientation by {angle_rot} rad...".format(angle_rot=angle_rot))
            # Rotate the robot to the target orientation
            self._rotate(angle_rot)
            self._sleep(2)
           
    def _null_command_manager(self) -> None:
        ''' This function manages the case when the robot doesn't receive the command from the QR code. '''
        
        # If the current command is not None, then not doing anything
        if self.current_cmd:
            return        
        
        # if self.navigator.getResult() == TaskResult.UNKNOWN or self.navigator.getResult() == TaskResult.FAILED:
        #     self.get_logger().info("[Turtlebot4 Navigation] Task result: {}".format(self.navigator.getResult()))
        #     self.current_cmd = self.last_cmd
        #     return                        
        
        self.get_logger().info("[Turtlebot4 Navigation] ATTENTION! Command not received. Implementing the strategy to acquire the command from the QR code...")
        
        # Attempt to acquire the command from the QR code iff the robot has completed the last task (TaskResult.SUCCEEDED) and the QR code is not yet detected
        if self.navigator.getResult() == TaskResult.SUCCEEDED:
            self.get_logger().info("--- [Null Command Manager] Attempting to acquire the command from the QR code by rotating the robot...")
            self._attempt_to_acquire_command()
        
        self._sleep(2)
        
        if self.current_cmd:
            return
        
        # If the command is still not received, reposition the robot (Kidnapped Manager)
        i = 0
        while self.current_cmd is None:
            if i == 0:
                self.get_logger().info("--- [Null Command Manager] Attempts to acquire the command from the QR code failed. Robot must be kidnapped.")
            i += 1
            # Not doing anything if the robot is not kidnapped
            self._kidnapped_manager()
        
    def _rotate(self, angle_rot : float) -> None:
        ''' This function is responsible for rotating the robot to the target orientation.

        :param float angle_rot: The angle in radians to rotate the robot to the target orientation.
        '''
        # If the angle is around 0, the robot is already in the target orientation
        if abs(angle_rot) < 0.1:
            return
        
        # Time to rotate the robot to the target orientation
        rotate_duration : float = float(abs(angle_rot) / self.angular_speed) 
        self.get_logger().info('------ [Rotate] Rotating the robot at a speed of {:.4f} rad/s... for {:.4f} s'.format(self.angular_speed, rotate_duration))
        
        try:
            # Create the Twist message to rotate the robot
            msg = Twist()
            # Set the angular speed of the robot.
            # NOTE: The angular speed is positive for a clockwise rotation and negative for a counterclockwise rotation.
            msg.angular.z = self.angular_speed * np.sign(angle_rot)
            # Publish the Twist message
            self.msg = msg
            self.ok = True
            # Sleep for the duration of the rotation
            self._sleep(rotate_duration)
            # Stop the robot's rotation
            self.ok = False
        except Exception as e:
            self.get_logger().error("Error during the rotation of the robot: {}".format(e))
    
    def _sleep(self, time_seconds: float) -> None:
        '''This function is responsible for sleeping the robot for a certain amount of time.

        :param float time_seconds: The time in seconds to sleep the robot.
        '''
        
        self.get_clock().sleep_for(Duration(seconds=time_seconds))    
        
def main():
    ''' Main function to move the Turtlebot4 through the landmarks in the map. '''
    # Initialize the ROS 2 node
    rclpy.init()
    # Create the executor
    executor = MultiThreadedExecutor()
    # Create the handler
    handller = Handler()
    executor.add_node(handller)                                                 # Add the node to the executor
    executor.create_task(handller.loop)                                         # Create a task with the callable provided as input
    param = rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, False)   # Create the parameter of the node 'use_sim_time'
    handller.set_parameters([param])                                            # Set the parameter of the node 'use_sim_time'
    executor.add_node(handller)                                                 # Add the node to the executor
    try:
        executor.spin()                                                         # Execute the loop - blocking call
    except KeyboardInterrupt:
        pass

    handller.destroy_node()                                                     # Good practice to destroy the node
    rclpy.shutdown()                                                            # Shutdown the ROS 2 node

if __name__ == '__main__':
    main()