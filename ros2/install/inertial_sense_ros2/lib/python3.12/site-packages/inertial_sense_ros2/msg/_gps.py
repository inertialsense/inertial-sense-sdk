# generated from rosidl_generator_py/resource/_idl.py.em
# with input from inertial_sense_ros2:msg/GPS.idl
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


class Metaclass_GPS(type):
    """Metaclass of message 'GPS'."""

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
                'inertial_sense_ros2.msg.GPS')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gps
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gps
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gps
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gps
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gps

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


class GPS(metaclass=Metaclass_GPS):
    """Message class 'GPS'."""

    __slots__ = [
        '_header',
        '_week',
        '_num_sat',
        '_status',
        '_cno',
        '_latitude',
        '_longitude',
        '_altitude',
        '_pos_ecef',
        '_vel_ecef',
        '_hmsl',
        '_hacc',
        '_vacc',
        '_sacc',
        '_pdop',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'week': 'uint32',
        'num_sat': 'int8',
        'status': 'uint32',
        'cno': 'int32',
        'latitude': 'double',
        'longitude': 'double',
        'altitude': 'double',
        'pos_ecef': 'geometry_msgs/Vector3',
        'vel_ecef': 'geometry_msgs/Vector3',
        'hmsl': 'float',
        'hacc': 'float',
        'vacc': 'float',
        'sacc': 'float',
        'pdop': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        self.week = kwargs.get('week', int())
        self.num_sat = kwargs.get('num_sat', int())
        self.status = kwargs.get('status', int())
        self.cno = kwargs.get('cno', int())
        self.latitude = kwargs.get('latitude', float())
        self.longitude = kwargs.get('longitude', float())
        self.altitude = kwargs.get('altitude', float())
        from geometry_msgs.msg import Vector3
        self.pos_ecef = kwargs.get('pos_ecef', Vector3())
        from geometry_msgs.msg import Vector3
        self.vel_ecef = kwargs.get('vel_ecef', Vector3())
        self.hmsl = kwargs.get('hmsl', float())
        self.hacc = kwargs.get('hacc', float())
        self.vacc = kwargs.get('vacc', float())
        self.sacc = kwargs.get('sacc', float())
        self.pdop = kwargs.get('pdop', float())

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
        if self.week != other.week:
            return False
        if self.num_sat != other.num_sat:
            return False
        if self.status != other.status:
            return False
        if self.cno != other.cno:
            return False
        if self.latitude != other.latitude:
            return False
        if self.longitude != other.longitude:
            return False
        if self.altitude != other.altitude:
            return False
        if self.pos_ecef != other.pos_ecef:
            return False
        if self.vel_ecef != other.vel_ecef:
            return False
        if self.hmsl != other.hmsl:
            return False
        if self.hacc != other.hacc:
            return False
        if self.vacc != other.vacc:
            return False
        if self.sacc != other.sacc:
            return False
        if self.pdop != other.pdop:
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
    def week(self):
        """Message field 'week'."""
        return self._week

    @week.setter
    def week(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'week' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'week' field must be an unsigned integer in [0, 4294967295]"
        self._week = value

    @builtins.property
    def num_sat(self):
        """Message field 'num_sat'."""
        return self._num_sat

    @num_sat.setter
    def num_sat(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'num_sat' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'num_sat' field must be an integer in [-128, 127]"
        self._num_sat = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'status' field must be an unsigned integer in [0, 4294967295]"
        self._status = value

    @builtins.property
    def cno(self):
        """Message field 'cno'."""
        return self._cno

    @cno.setter
    def cno(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'cno' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'cno' field must be an integer in [-2147483648, 2147483647]"
        self._cno = value

    @builtins.property
    def latitude(self):
        """Message field 'latitude'."""
        return self._latitude

    @latitude.setter
    def latitude(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'latitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'latitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._latitude = value

    @builtins.property
    def longitude(self):
        """Message field 'longitude'."""
        return self._longitude

    @longitude.setter
    def longitude(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'longitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'longitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._longitude = value

    @builtins.property
    def altitude(self):
        """Message field 'altitude'."""
        return self._altitude

    @altitude.setter
    def altitude(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'altitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'altitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._altitude = value

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
    def hmsl(self):
        """Message field 'hmsl'."""
        return self._hmsl

    @hmsl.setter
    def hmsl(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'hmsl' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'hmsl' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._hmsl = value

    @builtins.property
    def hacc(self):
        """Message field 'hacc'."""
        return self._hacc

    @hacc.setter
    def hacc(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'hacc' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'hacc' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._hacc = value

    @builtins.property
    def vacc(self):
        """Message field 'vacc'."""
        return self._vacc

    @vacc.setter
    def vacc(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'vacc' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vacc' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vacc = value

    @builtins.property
    def sacc(self):
        """Message field 'sacc'."""
        return self._sacc

    @sacc.setter
    def sacc(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'sacc' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'sacc' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._sacc = value

    @builtins.property
    def pdop(self):
        """Message field 'pdop'."""
        return self._pdop

    @pdop.setter
    def pdop(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'pdop' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pdop' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pdop = value
