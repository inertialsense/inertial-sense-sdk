# generated from rosidl_generator_py/resource/_idl.py.em
# with input from inertial_sense_ros2:msg/GNSSObservation.idl
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


class Metaclass_GNSSObservation(type):
    """Metaclass of message 'GNSSObservation'."""

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
                'inertial_sense_ros2.msg.GNSSObservation')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gnss_observation
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gnss_observation
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gnss_observation
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gnss_observation
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gnss_observation

            from inertial_sense_ros2.msg import GTime
            if GTime.__class__._TYPE_SUPPORT is None:
                GTime.__class__.__import_type_support__()

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


class GNSSObservation(metaclass=Metaclass_GNSSObservation):
    """Message class 'GNSSObservation'."""

    __slots__ = [
        '_header',
        '_time',
        '_sat',
        '_rcv',
        '_snrr',
        '_lli',
        '_code',
        '_qual_l',
        '_qual_p',
        '_l',
        '_p',
        '_d',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'time': 'inertial_sense_ros2/GTime',
        'sat': 'uint8',
        'rcv': 'uint8',
        'snrr': 'uint8',
        'lli': 'uint8',
        'code': 'uint8',
        'qual_l': 'uint8',
        'qual_p': 'uint8',
        'l': 'double',
        'p': 'double',
        'd': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['inertial_sense_ros2', 'msg'], 'GTime'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
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
        from inertial_sense_ros2.msg import GTime
        self.time = kwargs.get('time', GTime())
        self.sat = kwargs.get('sat', int())
        self.rcv = kwargs.get('rcv', int())
        self.snrr = kwargs.get('snrr', int())
        self.lli = kwargs.get('lli', int())
        self.code = kwargs.get('code', int())
        self.qual_l = kwargs.get('qual_l', int())
        self.qual_p = kwargs.get('qual_p', int())
        self.l = kwargs.get('l', float())
        self.p = kwargs.get('p', float())
        self.d = kwargs.get('d', float())

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
        if self.time != other.time:
            return False
        if self.sat != other.sat:
            return False
        if self.rcv != other.rcv:
            return False
        if self.snrr != other.snrr:
            return False
        if self.lli != other.lli:
            return False
        if self.code != other.code:
            return False
        if self.qual_l != other.qual_l:
            return False
        if self.qual_p != other.qual_p:
            return False
        if self.l != other.l:
            return False
        if self.p != other.p:
            return False
        if self.d != other.d:
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
    def time(self):
        """Message field 'time'."""
        return self._time

    @time.setter
    def time(self, value):
        if self._check_fields:
            from inertial_sense_ros2.msg import GTime
            assert \
                isinstance(value, GTime), \
                "The 'time' field must be a sub message of type 'GTime'"
        self._time = value

    @builtins.property
    def sat(self):
        """Message field 'sat'."""
        return self._sat

    @sat.setter
    def sat(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'sat' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'sat' field must be an unsigned integer in [0, 255]"
        self._sat = value

    @builtins.property
    def rcv(self):
        """Message field 'rcv'."""
        return self._rcv

    @rcv.setter
    def rcv(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'rcv' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'rcv' field must be an unsigned integer in [0, 255]"
        self._rcv = value

    @builtins.property
    def snrr(self):
        """Message field 'snrr'."""
        return self._snrr

    @snrr.setter
    def snrr(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'snrr' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'snrr' field must be an unsigned integer in [0, 255]"
        self._snrr = value

    @builtins.property
    def lli(self):
        """Message field 'lli'."""
        return self._lli

    @lli.setter
    def lli(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'lli' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'lli' field must be an unsigned integer in [0, 255]"
        self._lli = value

    @builtins.property
    def code(self):
        """Message field 'code'."""
        return self._code

    @code.setter
    def code(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'code' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'code' field must be an unsigned integer in [0, 255]"
        self._code = value

    @builtins.property
    def qual_l(self):
        """Message field 'qual_l'."""
        return self._qual_l

    @qual_l.setter
    def qual_l(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'qual_l' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'qual_l' field must be an unsigned integer in [0, 255]"
        self._qual_l = value

    @builtins.property
    def qual_p(self):
        """Message field 'qual_p'."""
        return self._qual_p

    @qual_p.setter
    def qual_p(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'qual_p' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'qual_p' field must be an unsigned integer in [0, 255]"
        self._qual_p = value

    @builtins.property
    def l(self):
        """Message field 'l'."""
        return self._l

    @l.setter
    def l(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._l = value

    @builtins.property
    def p(self):
        """Message field 'p'."""
        return self._p

    @p.setter
    def p(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'p' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'p' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._p = value

    @builtins.property
    def d(self):
        """Message field 'd'."""
        return self._d

    @d.setter
    def d(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'd' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'd' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._d = value
