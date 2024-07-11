# generated from rosidl_generator_py/resource/_idl.py.em
# with input from inertial_sense_ros2:msg/RTKRel.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RTKRel(type):
    """Metaclass of message 'RTKRel'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'GPS_STATUS_FIX_3D': 1,
        'GPS_STATUS_FIX_RTK_SINGLE': 2,
        'GPS_STATUS_FIX_RTK_FLOAT': 3,
        'GPS_STATUS_FIX_RTK_FIX': 4,
        'GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD': 5,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('inertial_sense_ros2')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'inertial_sense_ros2.msg.RTKRel')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__rtk_rel
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__rtk_rel
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__rtk_rel
            cls._TYPE_SUPPORT = module.type_support_msg__msg__rtk_rel
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__rtk_rel

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'GPS_STATUS_FIX_3D': cls.__constants['GPS_STATUS_FIX_3D'],
            'GPS_STATUS_FIX_RTK_SINGLE': cls.__constants['GPS_STATUS_FIX_RTK_SINGLE'],
            'GPS_STATUS_FIX_RTK_FLOAT': cls.__constants['GPS_STATUS_FIX_RTK_FLOAT'],
            'GPS_STATUS_FIX_RTK_FIX': cls.__constants['GPS_STATUS_FIX_RTK_FIX'],
            'GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD': cls.__constants['GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD'],
        }

    @property
    def GPS_STATUS_FIX_3D(self):
        """Message constant 'GPS_STATUS_FIX_3D'."""
        return Metaclass_RTKRel.__constants['GPS_STATUS_FIX_3D']

    @property
    def GPS_STATUS_FIX_RTK_SINGLE(self):
        """Message constant 'GPS_STATUS_FIX_RTK_SINGLE'."""
        return Metaclass_RTKRel.__constants['GPS_STATUS_FIX_RTK_SINGLE']

    @property
    def GPS_STATUS_FIX_RTK_FLOAT(self):
        """Message constant 'GPS_STATUS_FIX_RTK_FLOAT'."""
        return Metaclass_RTKRel.__constants['GPS_STATUS_FIX_RTK_FLOAT']

    @property
    def GPS_STATUS_FIX_RTK_FIX(self):
        """Message constant 'GPS_STATUS_FIX_RTK_FIX'."""
        return Metaclass_RTKRel.__constants['GPS_STATUS_FIX_RTK_FIX']

    @property
    def GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD(self):
        """Message constant 'GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD'."""
        return Metaclass_RTKRel.__constants['GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD']


class RTKRel(metaclass=Metaclass_RTKRel):
    """
    Message class 'RTKRel'.

    Constants:
      GPS_STATUS_FIX_3D
      GPS_STATUS_FIX_RTK_SINGLE
      GPS_STATUS_FIX_RTK_FLOAT
      GPS_STATUS_FIX_RTK_FIX
      GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD
    """

    __slots__ = [
        '_header',
        '_differential_age',
        '_ar_ratio',
        '_e_gps_status_fix',
        '_vector_base_to_rover',
        '_distance_base_to_rover',
        '_heading_base_to_rover',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'differential_age': 'float',
        'ar_ratio': 'float',
        'e_gps_status_fix': 'uint8',
        'vector_base_to_rover': 'geometry_msgs/Vector3',
        'distance_base_to_rover': 'float',
        'heading_base_to_rover': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.differential_age = kwargs.get('differential_age', float())
        self.ar_ratio = kwargs.get('ar_ratio', float())
        self.e_gps_status_fix = kwargs.get('e_gps_status_fix', int())
        from geometry_msgs.msg import Vector3
        self.vector_base_to_rover = kwargs.get('vector_base_to_rover', Vector3())
        self.distance_base_to_rover = kwargs.get('distance_base_to_rover', float())
        self.heading_base_to_rover = kwargs.get('heading_base_to_rover', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.differential_age != other.differential_age:
            return False
        if self.ar_ratio != other.ar_ratio:
            return False
        if self.e_gps_status_fix != other.e_gps_status_fix:
            return False
        if self.vector_base_to_rover != other.vector_base_to_rover:
            return False
        if self.distance_base_to_rover != other.distance_base_to_rover:
            return False
        if self.heading_base_to_rover != other.heading_base_to_rover:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def differential_age(self):
        """Message field 'differential_age'."""
        return self._differential_age

    @differential_age.setter
    def differential_age(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'differential_age' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'differential_age' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._differential_age = value

    @builtins.property
    def ar_ratio(self):
        """Message field 'ar_ratio'."""
        return self._ar_ratio

    @ar_ratio.setter
    def ar_ratio(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'ar_ratio' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ar_ratio' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ar_ratio = value

    @builtins.property
    def e_gps_status_fix(self):
        """Message field 'e_gps_status_fix'."""
        return self._e_gps_status_fix

    @e_gps_status_fix.setter
    def e_gps_status_fix(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'e_gps_status_fix' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'e_gps_status_fix' field must be an unsigned integer in [0, 255]"
        self._e_gps_status_fix = value

    @builtins.property
    def vector_base_to_rover(self):
        """Message field 'vector_base_to_rover'."""
        return self._vector_base_to_rover

    @vector_base_to_rover.setter
    def vector_base_to_rover(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'vector_base_to_rover' field must be a sub message of type 'Vector3'"
        self._vector_base_to_rover = value

    @builtins.property
    def distance_base_to_rover(self):
        """Message field 'distance_base_to_rover'."""
        return self._distance_base_to_rover

    @distance_base_to_rover.setter
    def distance_base_to_rover(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'distance_base_to_rover' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'distance_base_to_rover' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._distance_base_to_rover = value

    @builtins.property
    def heading_base_to_rover(self):
        """Message field 'heading_base_to_rover'."""
        return self._heading_base_to_rover

    @heading_base_to_rover.setter
    def heading_base_to_rover(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'heading_base_to_rover' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'heading_base_to_rover' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._heading_base_to_rover = value
