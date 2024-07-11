# generated from rosidl_generator_py/resource/_idl.py.em
# with input from inertial_sense_ros2:msg/DIDINS1.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'theta'
# Member 'uvw'
# Member 'lla'
# Member 'ned'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DIDINS1(type):
    """Metaclass of message 'DIDINS1'."""

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
                'inertial_sense_ros2.msg.DIDINS1')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__didins1
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__didins1
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__didins1
            cls._TYPE_SUPPORT = module.type_support_msg__msg__didins1
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__didins1

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


class DIDINS1(metaclass=Metaclass_DIDINS1):
    """Message class 'DIDINS1'."""

    __slots__ = [
        '_header',
        '_week',
        '_time_of_week',
        '_ins_status',
        '_hdw_status',
        '_theta',
        '_uvw',
        '_lla',
        '_ned',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'week': 'uint32',
        'time_of_week': 'double',
        'ins_status': 'uint32',
        'hdw_status': 'uint32',
        'theta': 'float[3]',
        'uvw': 'float[3]',
        'lla': 'double[3]',
        'ned': 'float[3]',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
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
        self.time_of_week = kwargs.get('time_of_week', float())
        self.ins_status = kwargs.get('ins_status', int())
        self.hdw_status = kwargs.get('hdw_status', int())
        if 'theta' not in kwargs:
            self.theta = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.theta = numpy.array(kwargs.get('theta'), dtype=numpy.float32)
            assert self.theta.shape == (3, )
        if 'uvw' not in kwargs:
            self.uvw = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.uvw = numpy.array(kwargs.get('uvw'), dtype=numpy.float32)
            assert self.uvw.shape == (3, )
        if 'lla' not in kwargs:
            self.lla = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.lla = numpy.array(kwargs.get('lla'), dtype=numpy.float64)
            assert self.lla.shape == (3, )
        if 'ned' not in kwargs:
            self.ned = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.ned = numpy.array(kwargs.get('ned'), dtype=numpy.float32)
            assert self.ned.shape == (3, )

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
        if self.time_of_week != other.time_of_week:
            return False
        if self.ins_status != other.ins_status:
            return False
        if self.hdw_status != other.hdw_status:
            return False
        if all(self.theta != other.theta):
            return False
        if all(self.uvw != other.uvw):
            return False
        if all(self.lla != other.lla):
            return False
        if all(self.ned != other.ned):
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
    def time_of_week(self):
        """Message field 'time_of_week'."""
        return self._time_of_week

    @time_of_week.setter
    def time_of_week(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'time_of_week' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'time_of_week' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._time_of_week = value

    @builtins.property
    def ins_status(self):
        """Message field 'ins_status'."""
        return self._ins_status

    @ins_status.setter
    def ins_status(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'ins_status' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'ins_status' field must be an unsigned integer in [0, 4294967295]"
        self._ins_status = value

    @builtins.property
    def hdw_status(self):
        """Message field 'hdw_status'."""
        return self._hdw_status

    @hdw_status.setter
    def hdw_status(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'hdw_status' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'hdw_status' field must be an unsigned integer in [0, 4294967295]"
        self._hdw_status = value

    @builtins.property
    def theta(self):
        """Message field 'theta'."""
        return self._theta

    @theta.setter
    def theta(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float32, \
                    "The 'theta' numpy.ndarray() must have the dtype of 'numpy.float32'"
                assert value.size == 3, \
                    "The 'theta' numpy.ndarray() must have a size of 3"
                self._theta = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'theta' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._theta = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def uvw(self):
        """Message field 'uvw'."""
        return self._uvw

    @uvw.setter
    def uvw(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float32, \
                    "The 'uvw' numpy.ndarray() must have the dtype of 'numpy.float32'"
                assert value.size == 3, \
                    "The 'uvw' numpy.ndarray() must have a size of 3"
                self._uvw = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'uvw' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._uvw = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def lla(self):
        """Message field 'lla'."""
        return self._lla

    @lla.setter
    def lla(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'lla' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 3, \
                    "The 'lla' numpy.ndarray() must have a size of 3"
                self._lla = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'lla' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._lla = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def ned(self):
        """Message field 'ned'."""
        return self._ned

    @ned.setter
    def ned(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float32, \
                    "The 'ned' numpy.ndarray() must have the dtype of 'numpy.float32'"
                assert value.size == 3, \
                    "The 'ned' numpy.ndarray() must have a size of 3"
                self._ned = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'ned' field must be a set or sequence with length 3 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._ned = numpy.array(value, dtype=numpy.float32)
