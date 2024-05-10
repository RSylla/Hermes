# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hermes_interfaces:msg/GpsFixed.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GpsFixed(type):
    """Metaclass of message 'GpsFixed'."""

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
            module = import_type_support('hermes_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hermes_interfaces.msg.GpsFixed')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gps_fixed
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gps_fixed
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gps_fixed
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gps_fixed
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gps_fixed

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GpsFixed(metaclass=Metaclass_GpsFixed):
    """Message class 'GpsFixed'."""

    __slots__ = [
        '_is_corrected',
        '_diff_age',
        '_message_id',
        '_utc_time',
        '_latitude',
        '_longtitude',
        '_north_south',
        '_east_west',
        '_nav_status',
        '_hor_accuracy',
        '_ver_accuracy',
        '_speed_over_ground_kmh',
        '_course_over_ground_deg',
        '_vertical_vel_ms',
        '_num_sat',
    ]

    _fields_and_field_types = {
        'is_corrected': 'boolean',
        'diff_age': 'float',
        'message_id': 'string',
        'utc_time': 'string',
        'latitude': 'double',
        'longtitude': 'double',
        'north_south': 'string',
        'east_west': 'string',
        'nav_status': 'string',
        'hor_accuracy': 'float',
        'ver_accuracy': 'float',
        'speed_over_ground_kmh': 'float',
        'course_over_ground_deg': 'float',
        'vertical_vel_ms': 'float',
        'num_sat': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.is_corrected = kwargs.get('is_corrected', bool())
        self.diff_age = kwargs.get('diff_age', float())
        self.message_id = kwargs.get('message_id', str())
        self.utc_time = kwargs.get('utc_time', str())
        self.latitude = kwargs.get('latitude', float())
        self.longtitude = kwargs.get('longtitude', float())
        self.north_south = kwargs.get('north_south', str())
        self.east_west = kwargs.get('east_west', str())
        self.nav_status = kwargs.get('nav_status', str())
        self.hor_accuracy = kwargs.get('hor_accuracy', float())
        self.ver_accuracy = kwargs.get('ver_accuracy', float())
        self.speed_over_ground_kmh = kwargs.get('speed_over_ground_kmh', float())
        self.course_over_ground_deg = kwargs.get('course_over_ground_deg', float())
        self.vertical_vel_ms = kwargs.get('vertical_vel_ms', float())
        self.num_sat = kwargs.get('num_sat', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.is_corrected != other.is_corrected:
            return False
        if self.diff_age != other.diff_age:
            return False
        if self.message_id != other.message_id:
            return False
        if self.utc_time != other.utc_time:
            return False
        if self.latitude != other.latitude:
            return False
        if self.longtitude != other.longtitude:
            return False
        if self.north_south != other.north_south:
            return False
        if self.east_west != other.east_west:
            return False
        if self.nav_status != other.nav_status:
            return False
        if self.hor_accuracy != other.hor_accuracy:
            return False
        if self.ver_accuracy != other.ver_accuracy:
            return False
        if self.speed_over_ground_kmh != other.speed_over_ground_kmh:
            return False
        if self.course_over_ground_deg != other.course_over_ground_deg:
            return False
        if self.vertical_vel_ms != other.vertical_vel_ms:
            return False
        if self.num_sat != other.num_sat:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def is_corrected(self):
        """Message field 'is_corrected'."""
        return self._is_corrected

    @is_corrected.setter
    def is_corrected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_corrected' field must be of type 'bool'"
        self._is_corrected = value

    @builtins.property
    def diff_age(self):
        """Message field 'diff_age'."""
        return self._diff_age

    @diff_age.setter
    def diff_age(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'diff_age' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'diff_age' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._diff_age = value

    @builtins.property
    def message_id(self):
        """Message field 'message_id'."""
        return self._message_id

    @message_id.setter
    def message_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message_id' field must be of type 'str'"
        self._message_id = value

    @builtins.property
    def utc_time(self):
        """Message field 'utc_time'."""
        return self._utc_time

    @utc_time.setter
    def utc_time(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'utc_time' field must be of type 'str'"
        self._utc_time = value

    @builtins.property
    def latitude(self):
        """Message field 'latitude'."""
        return self._latitude

    @latitude.setter
    def latitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'latitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'latitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._latitude = value

    @builtins.property
    def longtitude(self):
        """Message field 'longtitude'."""
        return self._longtitude

    @longtitude.setter
    def longtitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'longtitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'longtitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._longtitude = value

    @builtins.property
    def north_south(self):
        """Message field 'north_south'."""
        return self._north_south

    @north_south.setter
    def north_south(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'north_south' field must be of type 'str'"
        self._north_south = value

    @builtins.property
    def east_west(self):
        """Message field 'east_west'."""
        return self._east_west

    @east_west.setter
    def east_west(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'east_west' field must be of type 'str'"
        self._east_west = value

    @builtins.property
    def nav_status(self):
        """Message field 'nav_status'."""
        return self._nav_status

    @nav_status.setter
    def nav_status(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'nav_status' field must be of type 'str'"
        self._nav_status = value

    @builtins.property
    def hor_accuracy(self):
        """Message field 'hor_accuracy'."""
        return self._hor_accuracy

    @hor_accuracy.setter
    def hor_accuracy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'hor_accuracy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'hor_accuracy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._hor_accuracy = value

    @builtins.property
    def ver_accuracy(self):
        """Message field 'ver_accuracy'."""
        return self._ver_accuracy

    @ver_accuracy.setter
    def ver_accuracy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ver_accuracy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ver_accuracy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ver_accuracy = value

    @builtins.property
    def speed_over_ground_kmh(self):
        """Message field 'speed_over_ground_kmh'."""
        return self._speed_over_ground_kmh

    @speed_over_ground_kmh.setter
    def speed_over_ground_kmh(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed_over_ground_kmh' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'speed_over_ground_kmh' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._speed_over_ground_kmh = value

    @builtins.property
    def course_over_ground_deg(self):
        """Message field 'course_over_ground_deg'."""
        return self._course_over_ground_deg

    @course_over_ground_deg.setter
    def course_over_ground_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'course_over_ground_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'course_over_ground_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._course_over_ground_deg = value

    @builtins.property
    def vertical_vel_ms(self):
        """Message field 'vertical_vel_ms'."""
        return self._vertical_vel_ms

    @vertical_vel_ms.setter
    def vertical_vel_ms(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vertical_vel_ms' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vertical_vel_ms' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vertical_vel_ms = value

    @builtins.property
    def num_sat(self):
        """Message field 'num_sat'."""
        return self._num_sat

    @num_sat.setter
    def num_sat(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_sat' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'num_sat' field must be an integer in [-2147483648, 2147483647]"
        self._num_sat = value
