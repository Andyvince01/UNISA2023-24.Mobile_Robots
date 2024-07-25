import builtins                     # noqa: E402, I100
import math                         # noqa: E402, I100
import rosidl_parser.definition     # noqa: E402, I100
import numpy as np                  # noqa: E402, I100

from geometry_msgs.msg import Quaternion
from ._euler import Euler

class Orientation(builtins.object):
    # Declare the class instance attributes.
    __slots__ = [
        '_euler',
        '_quaternion',
    ]

    # Mapping of field names to their types.
    _fields_and_field_types = {
        'euler': '._euler/Euler',
        'quaternion': 'geometry_msgs/Quaternion'
    }

    # Define the types of the fields using rosidl_parser.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType([], 'Euler'),                               # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Quaternion')     # noqa: E501
    )
    
    # Constructor for the Orientation class.
    def __init__(self, **kwargs):
        # Check if the arguments passed to the constructor are valid.
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Argomenti non validi passati al costruttore: %s' % \
            ', '.join(sorted('_' + k for k in kwargs.keys() if '_' + k not in self.__slots__))
        
        # Initialize the euler and quaternion fields to the values passed as arguments.
        self._euler = kwargs.get('euler', None)
        self._quaternion = kwargs.get('quaternion', None)
                
        # Check if the euler and quaternion fields are set correctly.
        if self._euler is None and self._quaternion is None:
            pass
        # Check if the euler angles are set and the quaternion is not. In this case, set the quaternion.
        elif self._euler is None and self._quaternion is not None:
            self._euler = self.quaternion_to_euler(quaternion=self._quaternion)
        # Check if the quaternion is set and the euler angles are not. In this case, set the euler angles.
        elif self._euler is not None and self._quaternion is None:
            self._quaternion = self.euler_to_quaternion(euler=self._euler)
        # Check if both the euler angles and the quaternion are set.
        else:
            # Check if the quaternion match the euler angles.
            quaternion = self.euler_to_quaternion(euler=self._euler)
            if quaternion != self._quaternion:
                raise ValueError('Quaternion does not match euler angles.')
            # Check if the euler angles match the quaternion.
            euler = self.quaternion_to_euler(quaternion=self._quaternion)
            if euler != self._euler:
                raise ValueError('Euler angles do not match quaternion.')      
        
    # Representation of the Orientation class.
    def __repr__(self):
        # Get the name of the class.
        typename = self.__class__.__module__.split('.')
        # Remove the last element of the list.
        typename.pop()
        # Append the name of the class to the list
        typename.append(self.__class__.__name__)
        # Initialize an empty list to store the arguments.
        args = []
        # Iterate over the slots and slot types of the class.
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            # Get the value of the field.
            field = getattr(self, s)
            # Get the string representation of the field.
            fieldstr = repr(field)
            # Check if the field is a basic type.
            if (
                isinstance(t, rosidl_parser.definition.BasicType) and
                t.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                # Check if the field is an empty list.
                if isinstance(field, list) and len(field) == 0 :
                    fieldstr = '[]'
            # Append the field to the list of arguments.
            args.append(s[1:] + '=' + fieldstr)
        
        # Return the string representation of the class.
        return '%s(%s)' % ('\n'.join(typename), ', '.join(args))
    
    # Check if two instances of the class are equal.
    def __eq__(self, other):
        # Check if the other instance is of the same class.
        if not isinstance(other, self.__class__):
            return False
        # Check if the euler field is equal.
        if self.euler != other.euler:
            return False
        # Check if the quaternion field is equal.
        if self.quaternion != other.quaternion:
            return False
        return True

    # Get the fields and field types of the class.
    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)
    
    # Getter for the euler field.
    @builtins.property
    def euler(self):
        return self._euler
    
    # Setter for the euler field.
    @euler.setter
    def euler(self, value):
        self._euler = value
        
    # Getter for the quaternion field.
    @builtins.property
    def quaternion(self):
        return self._quaternion
    
    # Setter for the quaternion field.
    @quaternion.setter
    def quaternion(self, value):
        self._quaternion = value
        
    # Convert euler angles to quaternion.
    @ staticmethod
    def euler_to_quaternion(euler: Euler) -> Quaternion:
        '''
        Converts euler roll, pitch, yaw to quaternion. 
        Based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

        args:
            - roll: roll angle in radians
            - pitch: pitch angle in radians
            - yaw: yaw angle in radians
        '''
        cy = math.cos(euler.yaw * 0.5)
        sy = math.sin(euler.yaw * 0.5)
        cp = math.cos(euler.pitch * 0.5)
        sp = math.sin(euler.pitch * 0.5)
        cr = math.cos(euler.roll * 0.5)
        sr = math.sin(euler.roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        
        return q 
    
    # Convert quaternion to euler angles.
    @staticmethod
    def quaternion_to_euler(quaternion: Quaternion) -> Euler:
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
        
        e = Euler()
        e.roll = roll
        e.pitch = pitch
        e.yaw = yaw

        return e