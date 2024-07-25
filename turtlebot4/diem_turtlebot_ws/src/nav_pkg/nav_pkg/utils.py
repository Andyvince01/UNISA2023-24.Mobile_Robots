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
    '''This class is an enumeration of the possible commands that can be sent to the robot.

    :cvar STOP: The command to stop the robot.
    :cvar RIGHT: The command to turn the robot to the right.
    :cvar LEFT: The command to turn the robot to the left.
    :cvar STRAIGHTON: The command to move the robot straight on.
    :cvar GOBACK: The command to move the robot back.
    :cvar REPOSITIONING: The command to reposition the robot to the nearest landmark.
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

        :param str command: The command to be executed by the robot.
        :return TurtleBot4Directions: The angle of the robot.
        
        :Example:
        .. code-block:: python
            Commands.get_angle("RIGHT")     # TurtleBot4Directions.WEST
        '''
        command_to_angle = {
            Commands.RIGHT.value: TurtleBot4Directions.EAST,
            Commands.LEFT.value: TurtleBot4Directions.WEST,
            Commands.STRAIGHTON.value: TurtleBot4Directions.NORTH,
            Commands.GOBACK.value: TurtleBot4Directions.SOUTH
        }
            
        return command_to_angle[command]        

# =============================================================================
# Functions
# =============================================================================
def read_graph(file_path: str) -> Tuple[set, dict]:
    '''This function reads a JSON file containing the graph and returns the set of vertices and the dictionary of edges of the graph.   
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
    
    :param str file_path: The path to the JSON file containing the graph data.
    :return Tuple[set, dict]: A tuple containing the set of vertices and the dictionary of edges of the graph.
        
    :Example:
    .. code-block:: python
    >>> read_graph('nav_pkg/landmarks/diem_landmarks.json')
    # V = {(0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5)}
    # E = {((0, 0), (1, 1)): 1, ((1, 1), (2, 2)): 1, ((2, 2), (3, 3)): 1, ((3, 3), (4, 4)): 1, ((4, 4), (5, 5)): 1}
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
    It is based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles.
        
    :param Quaternion quaternion: The quaternion representing the orientation of the pose.
    :return Tuple[float, float, float]: The euler angles (roll, pitch, yaw) in radians.
    
    :Example:
    .. code-block:: python
    >>> euler_from_quaternion(Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))     # (0.0, 0.0, 0.0)
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
    '''This function converts euler roll, pitch, yaw to quaternion (x, y, z, w).
    
    :param float roll: The roll angle in radians.
    :param float pitch: The pitch angle in radians.
    :param float yaw: The yaw angle in radians.
    :return Quaternion: The quaternion representing the orientation of the pose.
    
    :Example:
    .. code-block:: python
    >>> quaternion_from_euler(0.0, 0.0, 0.0)     # Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
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

def get_next_waypoint(
    graph : nx.Graph, 
    point : Pose,
    nearest_waypoint : Tuple[float, float], 
    direction : str,
    debug : bool = False
) -> Tuple[Tuple[float, float], float]:
    '''This function returns the next waypoint that the robot has to reach based on the current vertex and the direction.
    
    :param Pose point: The current vertex (point) of the robot. It contains the position and orientation of the robot.
    :param Tuple[float, float] nearest_waypoint: The nearest waypoint from the current vertex.
    :param nx.Graph graph: The graph representing the environment.
    :param str direction: The direction of the robot.
    :param bool debug: A flag to print the debug information.
    :return Tuple[Tuple[float, float], float]: The next waypoint that the robot has to reach and the rotation of the robot.
    
    .. note::
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
        = [0, 0, cos(yaw) * (next_wp.y - current_wp.y) - sin(yaw) * (next_wp.x - current_wp.x)]^T
        
        The direction is determined by the sign of the cross product:
        - If the cross product is positive, the direction is RIGHT.
        - If the cross product is negative, the direction is LEFT.
        - If the cross product is zero, the direction is STRAIGHTON.
        
        Instead, the dot product between the vector of the orientation of the robot and the vector between the current 
        is used to determine if the robot has to go back or straight on.
        
        u . v = cos(yaw) * (next_wp.x - current_wp.x) + sin(yaw) * (next_wp.y - current_wp.y)
        
    :Example:
    .. code-block:: python
    >>> graph = nx.Graph()
    >>> graph.add_edge((0, 0), (0, 1))
    >>> graph.add_edge((0, 0), (1, 0))
    >>> graph.add_edge((0, 0), (-1, 0))
    >>> graph.add_edge((-1, 0), (-1, 1))
    >>> point = Pose()
    >>> point.position.x = 0.0
    >>> point.position.y = 0.0
    >>> point.orientation.z = 0.0
    >>> get_next_waypoint(point, graph, "RIGHT")        #Output: ((1.0, 0.0), 90)
    '''   
    # Initialize a dictionary of the neighbors of the current vertex (point).
    neighbors = {enum: None for enum in Commands._member_names_}
        
    # Get the incident edges of the nearest waypoint
    edges = list(graph.edges(nearest_waypoint, data=False))
    
    # Get the orientation of the robot and convert it to the euler angles
    yaw = euler_from_quaternion(point.orientation)[2]
    
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
    
        # Determine the direction of the next vertex        
        if abs(cross) > abs(inner):
            neighbors[Commands.LEFT.name if cross > 0 else Commands.RIGHT.name] = edge[1]
        else:
            neighbors[Commands.STRAIGHTON.name if inner >= 0 else Commands.GOBACK.name] = edge[1]

        if debug:
            print("a vector: ", a)
            print("b vector: ", b)
            print("Edge: ", edge)
            print("Cross product: ", cross)
            print("Dot product: ", inner)
    
    print("Neighbors[{direction}]: {neighbors}".format(direction=direction, neighbors=neighbors[direction]))

    if not neighbors[direction]:
        return None, None

    # Get the next waypoint and the next rotation of the robot
    next_waypoint = (float(neighbors[direction][0]), float(neighbors[direction][1]))
    next_rotation = np.arctan2(next_waypoint[1] - nearest_waypoint[1], next_waypoint[0] - nearest_waypoint[0]) 

    return next_waypoint, next_rotation
                    
def convert_to_pose(position = [0, 0, 0], orientation = Tuple[float, float, float]) -> Pose:
    '''This function converts a position and orientation to a Pose message.
    
    :param List[float] position: The position of the robot.
    :param List[float] orientation: The orientation of the robot. It is represented by the euler angles (roll, pitch, yaw).
    :return Pose: The Pose message containing the position and orientation of the robot.
    
    :Example:
    .. code-block:: python
    >>> convert_to_pose([0, 0, 0], [0, 0, 0, 1])
    # Output:
    # position:
    #   x: 0.0
    #   y: 0.0
    #   z: 0.0
    # orientation:
    #   x: 0.0
    #   y: 0.0
    #   z: 0.0
    #   w: 1.0
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
                    
if __name__ == '__main__':   
    
    V, E = read_graph('diem_turtlebot_ws/src/nav_pkg/landmarks/diem_landmarks.json')
    graph = nx.Graph()
    graph.add_nodes_from(V)
    graph.add_edges_from(E)
    
    debug = False
               
    if debug:
        pos = nx.spring_layout(graph)
        nx.draw(graph, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=20, font_color='black', font_weight='bold', font_family='sans-serif')
        plt.show()

    # Create a sample point
    point = Pose()
    point.position.x = -23.0
    point.position.y = -10.0
    point.orientation.z = 1.57
    
    next_wp = get_next_waypoint(graph=graph, point=point, nearest_waypoint=(-23, -0.3), direction=Commands.STRAIGHTON.name, debug=True)
    print("Next waypoint: ", next_wp)

    # Calcolare la differenza angolare non normalizzata
    angle_difference = (next_wp[1] % (2 * np.pi)) - (point.orientation.z % (2 * np.pi)) - 2 * np.pi
    # angle_difference += -np.sign(angle_difference) * 2 * np.pi if abs(angle_difference) > np.pi else 0
    # Normalizzare la differenza angolare nel range [-pi, pi]
    # angle_difference = ((angle_difference + np.pi) % (2 * np.pi)) - np.pi
    print("Angle difference: ", angle_difference)

    # # Convert a position and orientation to a Pose message
    # pose = convert_to_pose([0, 0, 0], [0, 0, 0])
    # print("Pose: ", pose)