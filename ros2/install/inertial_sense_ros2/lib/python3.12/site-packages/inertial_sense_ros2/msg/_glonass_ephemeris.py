# generated from rosidl_generator_py/resource/_idl.py.em
# with input from inertial_sense_ros2:msg/GlonassEphemeris.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'pos'
# Member 'vel'
# Member 'acc'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GlonassEphemeris(type):
    """Metaclass of message 'GlonassEphemeris'."""

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
                'inertial_sense_ros2.msg.GlonassEphemeris')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__glonass_ephemeris
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__glonass_ephemeris
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__glonass_ephemeris
            cls._TYPE_SUPPORT = module.type_support_msg__msg__glonass_ephemeris
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__glonass_ephemeris

            from inertial_sense_ros2.msg import GTime
            if GTime.__class__._TYPE_SUPPORT is None:
                GTime.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GlonassEphemeris(metaclass=Metaclass_GlonassEphemeris):
    """Message class 'GlonassEphemeris'."""

    __slots__ = [
        '_sat',
        '_iode',
        '_frq',
        '_svh',
        '_sva',
        '_age',
        '_toe',
        '_tof',
        '_pos',
        '_vel',
        '_acc',
        '_taun',
        '_gamn',
        '_dtaun',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'sat': 'int32',
        'iode': 'int32',
        'frq': 'int32',
        'svh': 'int32',
        'sva': 'int32',
        'age': 'int32',
        'toe': 'inertial_sense_ros2/GTime',
        'tof': 'inertial_sense_ros2/GTime',
        'pos': 'double[3]',
        'vel': 'double[3]',
        'acc': 'double[3]',
        'taun': 'double',
        'gamn': 'double',
        'dtaun': 'double',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['inertial_sense_ros2', 'msg'], 'GTime'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['inertial_sense_ros2', 'msg'], 'GTime'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
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
        self.sat = kwargs.get('sat', int())
        self.iode = kwargs.get('iode', int())
        self.frq = kwargs.get('frq', int())
        self.svh = kwargs.get('svh', int())
        self.sva = kwargs.get('sva', int())
        self.age = kwargs.get('age', int())
        from inertial_sense_ros2.msg import GTime
        self.toe = kwargs.get('toe', GTime())
        from inertial_sense_ros2.msg import GTime
        self.tof = kwargs.get('tof', GTime())
        if 'pos' not in kwargs:
            self.pos = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.pos = numpy.array(kwargs.get('pos'), dtype=numpy.float64)
            assert self.pos.shape == (3, )
        if 'vel' not in kwargs:
            self.vel = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.vel = numpy.array(kwargs.get('vel'), dtype=numpy.float64)
            assert self.vel.shape == (3, )
        if 'acc' not in kwargs:
            self.acc = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.acc = numpy.array(kwargs.get('acc'), dtype=numpy.float64)
            assert self.acc.shape == (3, )
        self.taun = kwargs.get('taun', float())
        self.gamn = kwargs.get('gamn', float())
        self.dtaun = kwargs.get('dtaun', float())

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
        if self.sat != other.sat:
            return False
        if self.iode != other.iode:
            return False
        if self.frq != other.frq:
            return False
        if self.svh != other.svh:
            return False
        if self.sva != other.sva:
            return False
        if self.age != other.age:
            return False
        if self.toe != other.toe:
            return False
        if self.tof != other.tof:
            return False
        if all(self.pos != other.pos):
            return False
        if all(self.vel != other.vel):
            return False
        if all(self.acc != other.acc):
            return False
        if self.taun != other.taun:
            return False
        if self.gamn != other.gamn:
            return False
        if self.dtaun != other.dtaun:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
            assert value >= -2147483648 and value < 2147483648, \
                "The 'sat' field must be an integer in [-2147483648, 2147483647]"
        self._sat = value

    @builtins.property
    def iode(self):
        """Message field 'iode'."""
        return self._iode

    @iode.setter
    def iode(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'iode' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'iode' field must be an integer in [-2147483648, 2147483647]"
        self._iode = value

    @builtins.property
    def frq(self):
        """Message field 'frq'."""
        return self._frq

    @frq.setter
    def frq(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'frq' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'frq' field must be an integer in [-2147483648, 2147483647]"
        self._frq = value

    @builtins.property
    def svh(self):
        """Message field 'svh'."""
        return self._svh

    @svh.setter
    def svh(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'svh' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'svh' field must be an integer in [-2147483648, 2147483647]"
        self._svh = value

    @builtins.property
    def sva(self):
        """Message field 'sva'."""
        return self._sva

    @sva.setter
    def sva(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'sva' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'sva' field must be an integer in [-2147483648, 2147483647]"
        self._sva = value

    @builtins.property
    def age(self):
        """Message field 'age'."""
        return self._age

    @age.setter
    def age(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'age' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'age' field must be an integer in [-2147483648, 2147483647]"
        self._age = value

    @builtins.property
    def toe(self):
        """Message field 'toe'."""
        return self._toe

    @toe.setter
    def toe(self, value):
        if self._check_fields:
            from inertial_sense_ros2.msg import GTime
            assert \
                isinstance(value, GTime), \
                "The 'toe' field must be a sub message of type 'GTime'"
        self._toe = value

    @builtins.property
    def tof(self):
        """Message field 'tof'."""
        return self._tof

    @tof.setter
    def tof(self, value):
        if self._check_fields:
            from inertial_sense_ros2.msg import GTime
            assert \
                isinstance(value, GTime), \
                "The 'tof' field must be a sub message of type 'GTime'"
        self._tof = value

    @builtins.property
    def pos(self):
        """Message field 'pos'."""
        return self._pos

    @pos.setter
    def pos(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'pos' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 3, \
                    "The 'pos' numpy.ndarray() must have a size of 3"
                self._pos = value
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
                "The 'pos' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._pos = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def vel(self):
        """Message field 'vel'."""
        return self._vel

    @vel.setter
    def vel(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'vel' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 3, \
                    "The 'vel' numpy.ndarray() must have a size of 3"
                self._vel = value
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
                "The 'vel' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._vel = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def acc(self):
        """Message field 'acc'."""
        return self._acc

    @acc.setter
    def acc(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'acc' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 3, \
                    "The 'acc' numpy.ndarray() must have a size of 3"
                self._acc = value
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
                "The 'acc' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._acc = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def taun(self):
        """Message field 'taun'."""
        return self._taun

    @taun.setter
    def taun(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'taun' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'taun' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._taun = value

    @builtins.property
    def gamn(self):
        """Message field 'gamn'."""
        return self._gamn

    @gamn.setter
    def gamn(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'gamn' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'gamn' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._gamn = value

    @builtins.property
    def dtaun(self):
        """Message field 'dtaun'."""
        return self._dtaun

    @dtaun.setter
    def dtaun(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'dtaun' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'dtaun' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._dtaun = value
