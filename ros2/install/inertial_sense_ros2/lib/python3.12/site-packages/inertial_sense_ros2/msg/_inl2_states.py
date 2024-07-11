# generated from rosidl_generator_py/resource/_idl.py.em
# with input from inertial_sense_ros2:msg/INL2States.idl
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


class Metaclass_INL2States(type):
    """Metaclass of message 'INL2States'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
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
                'inertial_sense_ros2.msg.INL2States')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__inl2_states
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__inl2_states
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__inl2_states
            cls._TYPE_SUPPORT = module.type_support_msg__msg__inl2_states
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__inl2_states

            from geometry_msgs.msg import Quaternion
            if Quaternion.__class__._TYPE_SUPPORT is None:
                Quaternion.__class__.__import_type_support__()

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
        }


class INL2States(metaclass=Metaclass_INL2States):
    """Message class 'INL2States'."""

    __slots__ = [
        '_header',
        '_quat_ecef',
        '_vel_ecef',
        '_pos_ecef',
        '_gyro_bias',
        '_accel_bias',
        '_baro_bias',
        '_mag_dec',
        '_mag_inc',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'quat_ecef': 'geometry_msgs/Quaternion',
        'vel_ecef': 'geometry_msgs/Vector3',
        'pos_ecef': 'geometry_msgs/Vector3',
        'gyro_bias': 'geometry_msgs/Vector3',
        'accel_bias': 'geometry_msgs/Vector3',
        'baro_bias': 'float',
        'mag_dec': 'float',
        'mag_inc': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Quaternion'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        from geometry_msgs.msg import Quaternion
        self.quat_ecef = kwargs.get('quat_ecef', Quaternion())
        from geometry_msgs.msg import Vector3
        self.vel_ecef = kwargs.get('vel_ecef', Vector3())
        from geometry_msgs.msg import Vector3
        self.pos_ecef = kwargs.get('pos_ecef', Vector3())
        from geometry_msgs.msg import Vector3
        self.gyro_bias = kwargs.get('gyro_bias', Vector3())
        from geometry_msgs.msg import Vector3
        self.accel_bias = kwargs.get('accel_bias', Vector3())
        self.baro_bias = kwargs.get('baro_bias', float())
        self.mag_dec = kwargs.get('mag_dec', float())
        self.mag_inc = kwargs.get('mag_inc', float())

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
        if self.quat_ecef != other.quat_ecef:
            return False
        if self.vel_ecef != other.vel_ecef:
            return False
        if self.pos_ecef != other.pos_ecef:
            return False
        if self.gyro_bias != other.gyro_bias:
            return False
        if self.accel_bias != other.accel_bias:
            return False
        if self.baro_bias != other.baro_bias:
            return False
        if self.mag_dec != other.mag_dec:
            return False
        if self.mag_inc != other.mag_inc:
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
    def quat_ecef(self):
        """Message field 'quat_ecef'."""
        return self._quat_ecef

    @quat_ecef.setter
    def quat_ecef(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Quaternion
            assert \
                isinstance(value, Quaternion), \
                "The 'quat_ecef' field must be a sub message of type 'Quaternion'"
        self._quat_ecef = value

    @builtins.property
    def vel_ecef(self):
        """Message field 'vel_ecef'."""
        return self._vel_ecef

    @vel_ecef.setter
    def vel_ecef(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'vel_ecef' field must be a sub message of type 'Vector3'"
        self._vel_ecef = value

    @builtins.property
    def pos_ecef(self):
        """Message field 'pos_ecef'."""
        return self._pos_ecef

    @pos_ecef.setter
    def pos_ecef(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'pos_ecef' field must be a sub message of type 'Vector3'"
        self._pos_ecef = value

    @builtins.property
    def gyro_bias(self):
        """Message field 'gyro_bias'."""
        return self._gyro_bias

    @gyro_bias.setter
    def gyro_bias(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'gyro_bias' field must be a sub message of type 'Vector3'"
        self._gyro_bias = value

    @builtins.property
    def accel_bias(self):
        """Message field 'accel_bias'."""
        return self._accel_bias

    @accel_bias.setter
    def accel_bias(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'accel_bias' field must be a sub message of type 'Vector3'"
        self._accel_bias = value

    @builtins.property
    def baro_bias(self):
        """Message field 'baro_bias'."""
        return self._baro_bias

    @baro_bias.setter
    def baro_bias(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'baro_bias' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'baro_bias' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._baro_bias = value

    @builtins.property
    def mag_dec(self):
        """Message field 'mag_dec'."""
        return self._mag_dec

    @mag_dec.setter
    def mag_dec(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'mag_dec' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'mag_dec' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._mag_dec = value

    @builtins.property
    def mag_inc(self):
        """Message field 'mag_inc'."""
        return self._mag_inc

    @mag_inc.setter
    def mag_inc(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'mag_inc' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'mag_inc' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._mag_inc = value
