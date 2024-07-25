import matplotlib.pyplot as plt
import math
import networkx as nx
import numpy as np
import json

from enum import Enum
from typing import *
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Twist, Quaternion
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions

# =============================================================================
# Enumerations
# =============================================================================
class Commands(Enum):
    '''
    This class is an enumeration of the possible commands that can be sent to the robot.
    
    Attributes
    ----------
    STOP : str
        The command to stop the robot.
        
    RIGHT : str
        The command to turn the robot to the right.
        
    LEFT : str
        The command to turn the robot to the left.
        
    STRAIGHTON : str
        The command to move the robot straight on.
        
    GOBACK : str
        The command to move the robot back.
    
    REPOSITIONING : str
        The command to reposition the robot to the nearest landmark.
    '''
    STOP = "STOP"
    RIGHT = "RIGHT"
    LEFT = "LEFT"
    STRAIGHTON = "STRAIGHTON"
    GOBACK = "GOBACK"
    REPOSITIONING = "REPOSITIONING"
        
    @staticmethod
    def get_angle(command: str) -> TurtleBot4Directions:
        '''This function returns the angle of the robot based on the command.
        
        Parameters
        ----------
        command : str
            The command to be executed by the robot.
        
        Returns
        -------
        TurtleBot4Directions
            The angle of the robot.
        
        Examples
        --------
        >>> Commands.get_angle("RIGHT")
        TurtleBot4Directions.WEST
        '''
        command_to_angle = {
            Commands.RIGHT.value: TurtleBot4Directions.WEST,
            Commands.LEFT.value: TurtleBot4Directions.EAST,
            Commands.STRAIGHTON.value: TurtleBot4Directions.NORTH,
            Commands.GOBACK.value: TurtleBot4Directions.SOUTH
        }
            
        return command_to_angle[command]        

# =============================================================================
# Functions
# =============================================================================
def read_graph(file_path: str) -> Tuple[set, dict]:
    '''
    This function reads a JSON file containing the graph and returns a `networkx` graph.
    
    Parameters
    ----------
    file_path : str
        The path to the JSON file containing the graph data.

    Returns
    -------
    Tuple[set, dict]
        A tuple containing the set of vertices and the dictionary of edges of the graph.
          
    Notes
    -----
    The JSON file must have the following structure:
    {
        "edge_id": {
            "source": {
                "x": 0,
                "y": 0
            },
            "target": {
                "x": 1,
                "y": 1
            }
        },
        ...
    }  
    
    Examples
    --------
    >>> read_graph('nav_pkg/landmarks/diem_landmarks.json')
    V = {(0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5)}
    E = {((0, 0), (1, 1)): 1, ((1, 1), (2, 2)): 1, ((2, 2), (3, 3)): 1, ((3, 3), (4, 4)): 1, ((4, 4), (5, 5)): 1}
    '''
    
    # Reading the JSON file
    with open(file_path) as f:
        data = json.load(f)

    # Initialized a set for the vertices and a dictionary for the edges
    V = set()
    E = dict()
    
    # Iterating over the pairs of vertices
    for pairs in data.values():
        # Get the source and target vertices
        source = (pairs['source']['x'], pairs['source']['y'])
        target = (pairs['target']['x'], pairs['target']['y'])
        # Add the vertices to the set of vertices
        V.add(source)
        V.add(target)
        # Add the edge to the dictionary of edges
        E[(source, target)] = 1

    return V, E

def euler_from_quaternion(quaternion : Quaternion) -> Tuple[float, float, float]:
    '''This function converts a quaternion to euler angles (roll, pitch, yaw).

    Parameters
    ----------
    quaternion : Quaternion
        The quaternion representing the orientation of the pose.

    Returns
    -------
    Tuple[float, float, float]
        The euler angles (roll, pitch, yaw) in radians.
        
    Notes
    -----
    It is based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles.
    
    Examples
    --------
    >>> euler_from_quaternion(Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    (0.0, 0.0, 0.0)
    '''
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    ''' This function converts euler roll, pitch, yaw to quaternion (x, y, z, w). 

    Parameters
    ----------
    roll : float
        The roll angle in radians.
    pitch : float
        The pitch angle in radians.
    yaw : float
        The yaw angle in radians.
        
    Returns
    -------
    Quaternion
        The quaternion representing the orientation of the pose.    
    
    Notes
    -----
    It is based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles.
    
    Examples
    --------
    >>> quaternion_from_euler(roll=0.0, pitch=0.0, yaw=0.0)
    Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    '''
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q 

def get_next_waypoint(point : Pose, nearest_waypoint : Tuple[float, float], graph : nx.Graph, direction : str) -> Tuple[Tuple[float, float], float]:
    '''This function returns the next waypoint that the robot has to reach based on the current vertex and the direction.
    
    Parameters
    ----------
    point : Pose
        The current vertex (point) of the robot. It contains the position and orientation of the robot.
    nearest_waypoint : Tuple[float, float]
        The nearest waypoint from the current vertex.
    graph : nx.Graph
        The graph representing the environment.
    direction : str
        The direction of the robot.

    Returns
    -------
    Tuple[Tuple[float, float], float]
        The next waypoint that the robot has to reach and the rotation of the robot.
    
    Notes
    -----
    The direction can be one of the following:
    - RIGHT
    - LEFT
    - STRAIGHTON
    - GOBACK
    
    The cross product between the vector of the orientation of the robot and the vector between the current
    vertex and the next vertex is used to determine the direction of the next vertex.
    
    u x v = [
        i, j, k,
        cos(yaw), sin(yaw), 0,
        next_wp.x - current_wp.x, next_wp.y - current_wp.y, 0
    ]
    = [0, 0, cos(yaw) * (next_wp.y - current_wp.y) - sin(yaw) * (next_wp.x - current_wp.x)]
    
    The direction is determined by the sign of the cross product:
    - If the cross product is positive, the direction is RIGHT.
    - If the cross product is negative, the direction is LEFT.
    - If the cross product is zero, the direction is STRAIGHTON.
    
    Instead, the dot product between the vector of the orientation of the robot and the vector between the current 
    is used to determine if the robot has to go back or straight on.
    
    u . v = cos(yaw) * (next_wp.x - current_wp.x) + sin(yaw) * (next_wp.y - current_wp.y)
    
    Examples
    --------
    >>> graph = nx.Graph()
    >>> graph.add_edge((0, 0), (0, 1))
    >>> graph.add_edge((0, 0), (1, 0))
    >>> graph.add_edge((0, 0), (-1, 0))
    >>> graph.add_edge((-1, 0), (-1, 1))
    >>> point = Pose()
    >>> point.position.x = 0.0
    >>> point.position.y = 0.0
    >>> point.orientation.z = 0.0
    >>> get_next_waypoint(point, graph, "RIGHT")
    ((1.0, 0.0), 90)
    '''    
    # Initialize a dictionary of the neighbors of the current vertex (point).
    neighbors = {enum: None for enum in Commands._member_names_}
        
    # Get the incident edges of the nearest waypoint
    edges = list(graph.edges(nearest_waypoint, data=False))
    
    # Get the orientation of the robot and convert it to the euler angles
    euler = euler_from_quaternion(point.orientation)
    yaw = euler[2]

    # Compute the vector of cosine and sine of the orientation of the robot
    a = np.array([np.cos(yaw), np.sin(yaw)])
        
    # Get the current vertex    
    cw = (point.position.x, point.position.y)

    # Get the next vertex
    for edge in edges:
        # Compute the vector between the current vertex and the next vertex
        b = np.array([edge[1][0] - cw[0], edge[1][1] - cw[1]])
        # Compute the dot product between the two vectors
        inner = np.inner(a, b)
        # Compute the cross product between the two vectors
        cross = np.cross(a, b)
            
        # Check the direction of the vertex at the extreme of the edge
        if abs(cross) > abs(inner):
            neighbors[Commands.LEFT.name if cross > 0 else Commands.RIGHT.name] = edge[1]
        else:
            neighbors[Commands.STRAIGHTON.name if inner >= 0 else Commands.GOBACK.name] = edge[1]

    if not neighbors[direction]:
        return None, None

    # Get the next waypoint and the next rotation of the robot
    next_waypoint = (float(neighbors[direction][0]), float(neighbors[direction][1]))
    next_rotation = np.rad2deg(np.arctan2(next_waypoint[1] - nearest_waypoint[1], next_waypoint[0] - nearest_waypoint[0]))
    next_rotation = (next_rotation + 360) % 360                                         # Map rotation to [0, 360] degrees

    return next_waypoint, Commands.get_angle(direction)
                    
def convert_to_pose(position = [0, 0, 0], orientation = Tuple[float, float, float]) -> Pose:
    '''This function converts a position and orientation to a Pose message.
    
    Parameters
    ----------
    position : List[float]
        The position of the robot.
    orientation : List[float]
        The orientation of the robot. It is represented by the euler angles (roll, pitch, yaw).

    Returns
    -------
    Pose
        The Pose message containing the position and orientation of the robot.
    
    Examples
    --------
    >>> convert_to_pose([0, 0, 0], [0, 0, 0, 1])
    position: 
      x: 0.0
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
    '''
    # Pre-process the position and orientation
    if len(position) < 3:
        position.extend([0])
    if isinstance(orientation, list) and len(orientation) == 3:
        quaternion = quaternion_from_euler(roll=float(orientation[0]), pitch=float(orientation[1]), yaw=float(orientation[2]))
    elif isinstance(orientation, Quaternion):
        quaternion = orientation
        
    # Create a Point message
    point = Point()
    point.x = float(position[0])
    point.y = float(position[1])
    point.z = float(position[2])
        
    # Create a Pose message
    pose = Pose()
    pose.position = point
    pose.orientation = quaternion
    
    return pose
                    
# if __name__ == '__main__':   
    # # Create a sample graph
    # graph = nx.Graph()
       
    # graph.add_edge((0, 0), (0, 1))
    # graph.add_edge((0, 0), (1, 0))
    # graph.add_edge((0, 0), (-1, 0))
    # graph.add_edge((-1, 0), (-1, 1))
    
    # # Create a sample point
    # point = Pose()
    # point.position.x = 0.0
    # point.position.y = 0.0
    # point.orientation.z = 0.0
    
    # next_wp = get_next_waypoint(graph=graph, point=point, nearest_waypoint=(0, 0), direction=Commands.STRAIGHTON.name)
    # print("Next waypoint: ", next_wp)

    # # Convert a position and orientation to a Pose message
    # pose = convert_to_pose([0, 0, 0], [0, 0, 0])
    # print("Pose: ", pose)
