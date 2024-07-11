# generated from rosidl_generator_py/resource/_idl.py.em
# with input from inertial_sense_ros2:msg/GNSSEphemeris.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'tgd'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GNSSEphemeris(type):
    """Metaclass of message 'GNSSEphemeris'."""

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
                'inertial_sense_ros2.msg.GNSSEphemeris')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gnss_ephemeris
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gnss_ephemeris
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gnss_ephemeris
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gnss_ephemeris
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gnss_ephemeris

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


class GNSSEphemeris(metaclass=Metaclass_GNSSEphemeris):
    """Message class 'GNSSEphemeris'."""

    __slots__ = [
        '_header',
        '_sat',
        '_iode',
        '_iodc',
        '_sva',
        '_svh',
        '_week',
        '_code',
        '_flag',
        '_toe',
        '_toc',
        '_ttr',
        '_a',
        '_e',
        '_i0',
        '_omg_0',
        '_omg',
        '_m_0',
        '_deln',
        '_omg_d',
        '_idot',
        '_crc',
        '_crs',
        '_cuc',
        '_cus',
        '_cic',
        '_cis',
        '_toes',
        '_fit',
        '_f0',
        '_f1',
        '_f2',
        '_tgd',
        '_a_dot',
        '_ndot',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'sat': 'int32',
        'iode': 'int32',
        'iodc': 'int32',
        'sva': 'int32',
        'svh': 'int32',
        'week': 'int32',
        'code': 'int32',
        'flag': 'int32',
        'toe': 'inertial_sense_ros2/GTime',
        'toc': 'inertial_sense_ros2/GTime',
        'ttr': 'inertial_sense_ros2/GTime',
        'a': 'double',
        'e': 'double',
        'i0': 'double',
        'omg_0': 'double',
        'omg': 'double',
        'm_0': 'double',
        'deln': 'double',
        'omg_d': 'double',
        'idot': 'double',
        'crc': 'double',
        'crs': 'double',
        'cuc': 'double',
        'cus': 'double',
        'cic': 'double',
        'cis': 'double',
        'toes': 'double',
        'fit': 'double',
        'f0': 'double',
        'f1': 'double',
        'f2': 'double',
        'tgd': 'double[4]',
        'a_dot': 'double',
        'ndot': 'double',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['inertial_sense_ros2', 'msg'], 'GTime'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['inertial_sense_ros2', 'msg'], 'GTime'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['inertial_sense_ros2', 'msg'], 'GTime'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 4),  # noqa: E501
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
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.sat = kwargs.get('sat', int())
        self.iode = kwargs.get('iode', int())
        self.iodc = kwargs.get('iodc', int())
        self.sva = kwargs.get('sva', int())
        self.svh = kwargs.get('svh', int())
        self.week = kwargs.get('week', int())
        self.code = kwargs.get('code', int())
        self.flag = kwargs.get('flag', int())
        from inertial_sense_ros2.msg import GTime
        self.toe = kwargs.get('toe', GTime())
        from inertial_sense_ros2.msg import GTime
        self.toc = kwargs.get('toc', GTime())
        from inertial_sense_ros2.msg import GTime
        self.ttr = kwargs.get('ttr', GTime())
        self.a = kwargs.get('a', float())
        self.e = kwargs.get('e', float())
        self.i0 = kwargs.get('i0', float())
        self.omg_0 = kwargs.get('omg_0', float())
        self.omg = kwargs.get('omg', float())
        self.m_0 = kwargs.get('m_0', float())
        self.deln = kwargs.get('deln', float())
        self.omg_d = kwargs.get('omg_d', float())
        self.idot = kwargs.get('idot', float())
        self.crc = kwargs.get('crc', float())
        self.crs = kwargs.get('crs', float())
        self.cuc = kwargs.get('cuc', float())
        self.cus = kwargs.get('cus', float())
        self.cic = kwargs.get('cic', float())
        self.cis = kwargs.get('cis', float())
        self.toes = kwargs.get('toes', float())
        self.fit = kwargs.get('fit', float())
        self.f0 = kwargs.get('f0', float())
        self.f1 = kwargs.get('f1', float())
        self.f2 = kwargs.get('f2', float())
        if 'tgd' not in kwargs:
            self.tgd = numpy.zeros(4, dtype=numpy.float64)
        else:
            self.tgd = numpy.array(kwargs.get('tgd'), dtype=numpy.float64)
            assert self.tgd.shape == (4, )
        self.a_dot = kwargs.get('a_dot', float())
        self.ndot = kwargs.get('ndot', float())

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
        if self.sat != other.sat:
            return False
        if self.iode != other.iode:
            return False
        if self.iodc != other.iodc:
            return False
        if self.sva != other.sva:
            return False
        if self.svh != other.svh:
            return False
        if self.week != other.week:
            return False
        if self.code != other.code:
            return False
        if self.flag != other.flag:
            return False
        if self.toe != other.toe:
            return False
        if self.toc != other.toc:
            return False
        if self.ttr != other.ttr:
            return False
        if self.a != other.a:
            return False
        if self.e != other.e:
            return False
        if self.i0 != other.i0:
            return False
        if self.omg_0 != other.omg_0:
            return False
        if self.omg != other.omg:
            return False
        if self.m_0 != other.m_0:
            return False
        if self.deln != other.deln:
            return False
        if self.omg_d != other.omg_d:
            return False
        if self.idot != other.idot:
            return False
        if self.crc != other.crc:
            return False
        if self.crs != other.crs:
            return False
        if self.cuc != other.cuc:
            return False
        if self.cus != other.cus:
            return False
        if self.cic != other.cic:
            return False
        if self.cis != other.cis:
            return False
        if self.toes != other.toes:
            return False
        if self.fit != other.fit:
            return False
        if self.f0 != other.f0:
            return False
        if self.f1 != other.f1:
            return False
        if self.f2 != other.f2:
            return False
        if all(self.tgd != other.tgd):
            return False
        if self.a_dot != other.a_dot:
            return False
        if self.ndot != other.ndot:
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
    def iodc(self):
        """Message field 'iodc'."""
        return self._iodc

    @iodc.setter
    def iodc(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'iodc' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'iodc' field must be an integer in [-2147483648, 2147483647]"
        self._iodc = value

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
    def week(self):
        """Message field 'week'."""
        return self._week

    @week.setter
    def week(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'week' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'week' field must be an integer in [-2147483648, 2147483647]"
        self._week = value

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
            assert value >= -2147483648 and value < 2147483648, \
                "The 'code' field must be an integer in [-2147483648, 2147483647]"
        self._code = value

    @builtins.property
    def flag(self):
        """Message field 'flag'."""
        return self._flag

    @flag.setter
    def flag(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'flag' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'flag' field must be an integer in [-2147483648, 2147483647]"
        self._flag = value

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
    def toc(self):
        """Message field 'toc'."""
        return self._toc

    @toc.setter
    def toc(self, value):
        if self._check_fields:
            from inertial_sense_ros2.msg import GTime
            assert \
                isinstance(value, GTime), \
                "The 'toc' field must be a sub message of type 'GTime'"
        self._toc = value

    @builtins.property
    def ttr(self):
        """Message field 'ttr'."""
        return self._ttr

    @ttr.setter
    def ttr(self, value):
        if self._check_fields:
            from inertial_sense_ros2.msg import GTime
            assert \
                isinstance(value, GTime), \
                "The 'ttr' field must be a sub message of type 'GTime'"
        self._ttr = value

    @builtins.property
    def a(self):
        """Message field 'a'."""
        return self._a

    @a.setter
    def a(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'a' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'a' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._a = value

    @builtins.property
    def e(self):
        """Message field 'e'."""
        return self._e

    @e.setter
    def e(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'e' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'e' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._e = value

    @builtins.property
    def i0(self):
        """Message field 'i0'."""
        return self._i0

    @i0.setter
    def i0(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'i0' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'i0' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._i0 = value

    @builtins.property
    def omg_0(self):
        """Message field 'omg_0'."""
        return self._omg_0

    @omg_0.setter
    def omg_0(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'omg_0' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'omg_0' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._omg_0 = value

    @builtins.property
    def omg(self):
        """Message field 'omg'."""
        return self._omg

    @omg.setter
    def omg(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'omg' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'omg' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._omg = value

    @builtins.property
    def m_0(self):
        """Message field 'm_0'."""
        return self._m_0

    @m_0.setter
    def m_0(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'm_0' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'm_0' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._m_0 = value

    @builtins.property
    def deln(self):
        """Message field 'deln'."""
        return self._deln

    @deln.setter
    def deln(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'deln' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'deln' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._deln = value

    @builtins.property
    def omg_d(self):
        """Message field 'omg_d'."""
        return self._omg_d

    @omg_d.setter
    def omg_d(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'omg_d' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'omg_d' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._omg_d = value

    @builtins.property
    def idot(self):
        """Message field 'idot'."""
        return self._idot

    @idot.setter
    def idot(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'idot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'idot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._idot = value

    @builtins.property
    def crc(self):
        """Message field 'crc'."""
        return self._crc

    @crc.setter
    def crc(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'crc' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'crc' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._crc = value

    @builtins.property
    def crs(self):
        """Message field 'crs'."""
        return self._crs

    @crs.setter
    def crs(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'crs' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'crs' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._crs = value

    @builtins.property
    def cuc(self):
        """Message field 'cuc'."""
        return self._cuc

    @cuc.setter
    def cuc(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'cuc' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'cuc' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._cuc = value

    @builtins.property
    def cus(self):
        """Message field 'cus'."""
        return self._cus

    @cus.setter
    def cus(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'cus' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'cus' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._cus = value

    @builtins.property
    def cic(self):
        """Message field 'cic'."""
        return self._cic

    @cic.setter
    def cic(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'cic' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'cic' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._cic = value

    @builtins.property
    def cis(self):
        """Message field 'cis'."""
        return self._cis

    @cis.setter
    def cis(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'cis' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'cis' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._cis = value

    @builtins.property
    def toes(self):
        """Message field 'toes'."""
        return self._toes

    @toes.setter
    def toes(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'toes' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'toes' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._toes = value

    @builtins.property
    def fit(self):
        """Message field 'fit'."""
        return self._fit

    @fit.setter
    def fit(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'fit' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fit' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fit = value

    @builtins.property
    def f0(self):
        """Message field 'f0'."""
        return self._f0

    @f0.setter
    def f0(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'f0' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'f0' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._f0 = value

    @builtins.property
    def f1(self):
        """Message field 'f1'."""
        return self._f1

    @f1.setter
    def f1(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'f1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'f1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._f1 = value

    @builtins.property
    def f2(self):
        """Message field 'f2'."""
        return self._f2

    @f2.setter
    def f2(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'f2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'f2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._f2 = value

    @builtins.property
    def tgd(self):
        """Message field 'tgd'."""
        return self._tgd

    @tgd.setter
    def tgd(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'tgd' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 4, \
                    "The 'tgd' numpy.ndarray() must have a size of 4"
                self._tgd = value
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
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'tgd' field must be a set or sequence with length 4 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._tgd = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def a_dot(self):
        """Message field 'a_dot'."""
        return self._a_dot

    @a_dot.setter
    def a_dot(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'a_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'a_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._a_dot = value

    @builtins.property
    def ndot(self):
        """Message field 'ndot'."""
        return self._ndot

    @ndot.setter
    def ndot(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'ndot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ndot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ndot = value
