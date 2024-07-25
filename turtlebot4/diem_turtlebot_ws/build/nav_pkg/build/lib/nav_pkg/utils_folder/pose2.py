import math
import numpy as np
from enum import IntEnum
from geometry_msgs.msg import Point, Pose, Quaternion


class Pose2:
    '''
    Class to represent a 2D pose with position and orientation.
    
    Attributes:
        - id: unique identifier for the pose
        - position: Point object representing the position of the pose
        - orientation: Orientation object representing the orientation of the pose
    '''
    
    __slots__ = ['id', 'position', 'orientation']
    
    
    ## --- NESTED CLASSES --- ##
    class Orientation:
        '''
        Class to represent the orientation of a pose.
        
        Attributes:
            - roll: roll angle in radians
            - pitch: pitch angle in radians
            - yaw: yaw angle in radians
            - quaternion: Quaternion object representing the orientation of the pose (x, y, z, w)
        '''
        __slots__ = ['roll', 'pitch', 'yaw', 'quaternion']
        
        ## --- CONSTRUCTOR --- ##
        def __init__(self) -> None:
            '''
            Initialize the orientation with default values for roll, pitch, yaw and quaternion.
            '''
            self.roll = 0
            self.pitch = 0
            self.yaw = 0
            self.quaternion = Quaternion()
        
        # --- MAGIC METHODS --- #
        def __repr__(self) -> str:
            '''
            Return the string representation of the orientation.
        
            return:
                - string: string representation of the orientation
            '''
            return self.__str__()
        
        def __str__(self) -> str:
            '''
            Return the string representation of the orientation.
            
            return:
                - string: string representation of the orientation
            '''
            euler = f'({math.degrees(self.roll):.4f}, {math.degrees(self.pitch):.4f}, {math.degrees(self.yaw):.4f})°'
            quat = f'({self.quaternion.x:.4f}, {self.quaternion.y:.4f}, {self.quaternion.z:.4f}, {self.quaternion.w:.4f})'
            data = '{' + f"\n\t(roll, pitch, yaw): {euler}," + f"\n\tquaternion: {quat}\n" + '}'
            return data
    
    ## --- CONSTRUCTOR --- ##
    def __init__(self, id : str = 'point', position : dict = None, orientation : dict = None) -> None:
        '''
        Initialize the pose with default values for id, position and orientation.
        
        args:
            - id: unique identifier for the pose
            - position: dictionary with x, y and z coordinates
            - orientation: dictionary with roll, pitch and yaw angles in degrees
        '''
        # Id of the pose
        self.id = id
        
        # Position of the pose
        self.position = Point()
        if position is not None:
            if isinstance(position, Point):
                self.position = position
            else:
                position = {key: float(value) for key, value in position.items()}
                self.position = Point(**position) 
        
        # Orientation of the pose
        self.orientation = self.Orientation()
        if orientation is not None:
            if isinstance(orientation, Quaternion):
                self.set_orientation_from_quaternion(orientation)
            else:
                self.set_orientation_from_angles(orientation)
    
    ## --- PUBLIC METHODS --- ##
    def set_orientation_from_quaternion(self, quaternion : Quaternion) -> None:
        '''
        Set the orientation of the pose from a quaternion.
        
        args:
            - quaternion: Quaternion object representing the orientation of the pose (x, y, z, w)
        '''
        self.orientation.quaternion = quaternion
        roll, pitch, yaw = self.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.orientation.roll = roll
        self.orientation.pitch = pitch
        self.orientation.yaw = yaw
        
    def set_orientation_from_angles(self, angles : dict) -> None:
        '''
        Set the orientation of the pose from roll, pitch and yaw angles.
        
        args:
            - angles: dictionary with roll, pitch and yaw angles in degrees
            
        '''
        self.orientation.roll = math.radians(angles['roll'])
        self.orientation.pitch = math.radians(angles['pitch'])
        self.orientation.yaw = math.radians(angles['yaw'])
        self.orientation.quaternion = self.quaternion_from_euler(self.orientation.roll, self.orientation.pitch, self.orientation.yaw)
                              
    ## --- STATIC METHODS --- ##
    @staticmethod
    def from_pose_to_pose2(pose: Pose) -> 'Pose2':
        '''
        Convert a Pose object to a Pose2 object.
        
        args:
            - pose: Pose object
        
        return:
            - Pose2: Pose2 object
        '''
        return Pose2(position=pose.position, orientation=pose.orientation)
    
    @staticmethod
    def from_pose2_to_pose(pose2: 'Pose2') -> Pose:
        '''
        Convert a Pose2 object to a Pose object.
        
        args:
            - pose2: Pose2 object
            
        return:
            - Pose: Pose object
        '''
        return Pose(position=pose2.position, orientation=pose2.orientation.quaternion)
    
    @staticmethod
    def euler_from_quaternion(quaternion : Quaternion) -> tuple:
        '''
        Converts quaternion (w in last place) to euler roll, pitch, yaw quaternion = [x, y, z, w]. 
        Based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        
        args:
            - quaternion: Quaternion object representing the orientation of the pose (x, y, z, w)
            
        return:
            - tuple: tuple with roll, pitch, yaw angles in radians        
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

    @ staticmethod
    def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
        '''
        Converts euler roll, pitch, yaw to quaternion. 
        Based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

        args:
            - roll: roll angle in radians
            - pitch: pitch angle in radians
            - yaw: yaw angle in radians
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
    
    @staticmethod
    def distance(*poses: 'Pose2') -> float:
        '''
        Calculate the distance between two poses
        
        args:
            - poses: a list of Pose2 objects
            
        return:
            - float: distance between the two poses
        '''
        if len(poses) != 2:
            raise ValueError('Two poses are required to calculate distance.')
        
        x = np.array([pose.position.x for pose in poses])
        y = np.array([pose.position.y for pose in poses])
        z = np.array([pose.position.z for pose in poses])           # z coordinate is not used
        return np.linalg.norm([np.diff(x), np.diff(y)])
    
    @staticmethod
    def orientation_diff(*poses: 'Pose2') -> float:
        '''
        Calculate the difference between two orientations.
        
        args:
            - poses: a list of Pose2 objects
            
        return:
            - float: difference between the two orientations
        '''
        if len(poses) != 2:
            raise ValueError('Two poses are required to calculate orientation difference.')
        
        roll = np.radians([pose.orientation.roll for pose in poses])    # roll is not used
        pitch = np.radians([pose.orientation.pitch for pose in poses])  # pitch is not used
        yaw = np.array([pose.orientation.yaw for pose in poses])
        
        orientation_diff = np.diff(yaw)
        return orientation_diff if orientation_diff < np.pi else 2 * np.pi - orientation_diff
    
    @staticmethod
    def add_gaussian_noise(pose: 'Pose2', mean : float = 0.0, std_dev : float = 0.1) -> 'Pose2':
        '''
        Add Gaussian noise to the position of the pose.
        
        args:
            - pose: Pose2 object
            - mean: mean of the Gaussian distribution
            - std_dev: standard deviation of the Gaussian distribution
            
        return:
            - Pose2: Pose2 object with noise added to the position
        '''
        pose.position.x += np.random.normal(mean, std_dev)
        pose.position.y += np.random.normal(mean, std_dev)
        # pose.position.z += np.random.normal(mean, std_dev)
        return pose
        
    # --- MAGIC METHODS --- #
    def __repr__(self) -> str:
        '''
        Return the string representation of the pose (debugging purposes).
        '''
        return self.__str__()
        
    def __str__(self) -> str:
        '''
        Return the string representation of the pose
        '''
        pos = f"({self.position.x:.2f}, {self.position.y:.2f}, {self.position.z:.2f})"
        data = '{ ' + f'id: {self.id}\n' + \
                f'  position: {pos}\n' + \
                f'  orientation: {self.orientation}' + '}\n'
        return data