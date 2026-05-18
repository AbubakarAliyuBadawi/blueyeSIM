# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vortex_msgs:msg/KMBinary.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_KMBinary(type):
    """Metaclass of message 'KMBinary'."""

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
            module = import_type_support('vortex_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vortex_msgs.msg.KMBinary')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__km_binary
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__km_binary
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__km_binary
            cls._TYPE_SUPPORT = module.type_support_msg__msg__km_binary
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__km_binary

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class KMBinary(metaclass=Metaclass_KMBinary):
    """Message class 'KMBinary'."""

    __slots__ = [
        '_utc_seconds',
        '_utc_nanoseconds',
        '_status',
        '_latitude',
        '_longitude',
        '_ellipsoid_height',
        '_roll',
        '_pitch',
        '_heading',
        '_heave',
        '_roll_rate',
        '_pitch_rate',
        '_yaw_rate',
        '_north_velocity',
        '_east_velocity',
        '_down_velocity',
        '_latitude_error',
        '_longitude_error',
        '_height_error',
        '_roll_error',
        '_pitch_error',
        '_heading_error',
        '_heave_error',
        '_north_acceleration',
        '_east_acceleration',
        '_down_acceleration',
        '_delayed_heave_utc_seconds',
        '_delayed_heave_utc_nanoseconds',
        '_delayed_heave',
    ]

    _fields_and_field_types = {
        'utc_seconds': 'uint32',
        'utc_nanoseconds': 'uint32',
        'status': 'uint32',
        'latitude': 'double',
        'longitude': 'double',
        'ellipsoid_height': 'float',
        'roll': 'float',
        'pitch': 'float',
        'heading': 'float',
        'heave': 'float',
        'roll_rate': 'float',
        'pitch_rate': 'float',
        'yaw_rate': 'float',
        'north_velocity': 'float',
        'east_velocity': 'float',
        'down_velocity': 'float',
        'latitude_error': 'float',
        'longitude_error': 'float',
        'height_error': 'float',
        'roll_error': 'float',
        'pitch_error': 'float',
        'heading_error': 'float',
        'heave_error': 'float',
        'north_acceleration': 'float',
        'east_acceleration': 'float',
        'down_acceleration': 'float',
        'delayed_heave_utc_seconds': 'uint32',
        'delayed_heave_utc_nanoseconds': 'uint32',
        'delayed_heave': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.utc_seconds = kwargs.get('utc_seconds', int())
        self.utc_nanoseconds = kwargs.get('utc_nanoseconds', int())
        self.status = kwargs.get('status', int())
        self.latitude = kwargs.get('latitude', float())
        self.longitude = kwargs.get('longitude', float())
        self.ellipsoid_height = kwargs.get('ellipsoid_height', float())
        self.roll = kwargs.get('roll', float())
        self.pitch = kwargs.get('pitch', float())
        self.heading = kwargs.get('heading', float())
        self.heave = kwargs.get('heave', float())
        self.roll_rate = kwargs.get('roll_rate', float())
        self.pitch_rate = kwargs.get('pitch_rate', float())
        self.yaw_rate = kwargs.get('yaw_rate', float())
        self.north_velocity = kwargs.get('north_velocity', float())
        self.east_velocity = kwargs.get('east_velocity', float())
        self.down_velocity = kwargs.get('down_velocity', float())
        self.latitude_error = kwargs.get('latitude_error', float())
        self.longitude_error = kwargs.get('longitude_error', float())
        self.height_error = kwargs.get('height_error', float())
        self.roll_error = kwargs.get('roll_error', float())
        self.pitch_error = kwargs.get('pitch_error', float())
        self.heading_error = kwargs.get('heading_error', float())
        self.heave_error = kwargs.get('heave_error', float())
        self.north_acceleration = kwargs.get('north_acceleration', float())
        self.east_acceleration = kwargs.get('east_acceleration', float())
        self.down_acceleration = kwargs.get('down_acceleration', float())
        self.delayed_heave_utc_seconds = kwargs.get('delayed_heave_utc_seconds', int())
        self.delayed_heave_utc_nanoseconds = kwargs.get('delayed_heave_utc_nanoseconds', int())
        self.delayed_heave = kwargs.get('delayed_heave', float())

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
        if self.utc_seconds != other.utc_seconds:
            return False
        if self.utc_nanoseconds != other.utc_nanoseconds:
            return False
        if self.status != other.status:
            return False
        if self.latitude != other.latitude:
            return False
        if self.longitude != other.longitude:
            return False
        if self.ellipsoid_height != other.ellipsoid_height:
            return False
        if self.roll != other.roll:
            return False
        if self.pitch != other.pitch:
            return False
        if self.heading != other.heading:
            return False
        if self.heave != other.heave:
            return False
        if self.roll_rate != other.roll_rate:
            return False
        if self.pitch_rate != other.pitch_rate:
            return False
        if self.yaw_rate != other.yaw_rate:
            return False
        if self.north_velocity != other.north_velocity:
            return False
        if self.east_velocity != other.east_velocity:
            return False
        if self.down_velocity != other.down_velocity:
            return False
        if self.latitude_error != other.latitude_error:
            return False
        if self.longitude_error != other.longitude_error:
            return False
        if self.height_error != other.height_error:
            return False
        if self.roll_error != other.roll_error:
            return False
        if self.pitch_error != other.pitch_error:
            return False
        if self.heading_error != other.heading_error:
            return False
        if self.heave_error != other.heave_error:
            return False
        if self.north_acceleration != other.north_acceleration:
            return False
        if self.east_acceleration != other.east_acceleration:
            return False
        if self.down_acceleration != other.down_acceleration:
            return False
        if self.delayed_heave_utc_seconds != other.delayed_heave_utc_seconds:
            return False
        if self.delayed_heave_utc_nanoseconds != other.delayed_heave_utc_nanoseconds:
            return False
        if self.delayed_heave != other.delayed_heave:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def utc_seconds(self):
        """Message field 'utc_seconds'."""
        return self._utc_seconds

    @utc_seconds.setter
    def utc_seconds(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'utc_seconds' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'utc_seconds' field must be an unsigned integer in [0, 4294967295]"
        self._utc_seconds = value

    @builtins.property
    def utc_nanoseconds(self):
        """Message field 'utc_nanoseconds'."""
        return self._utc_nanoseconds

    @utc_nanoseconds.setter
    def utc_nanoseconds(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'utc_nanoseconds' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'utc_nanoseconds' field must be an unsigned integer in [0, 4294967295]"
        self._utc_nanoseconds = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'status' field must be an unsigned integer in [0, 4294967295]"
        self._status = value

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
    def longitude(self):
        """Message field 'longitude'."""
        return self._longitude

    @longitude.setter
    def longitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'longitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'longitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._longitude = value

    @builtins.property
    def ellipsoid_height(self):
        """Message field 'ellipsoid_height'."""
        return self._ellipsoid_height

    @ellipsoid_height.setter
    def ellipsoid_height(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ellipsoid_height' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ellipsoid_height' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ellipsoid_height = value

    @builtins.property
    def roll(self):
        """Message field 'roll'."""
        return self._roll

    @roll.setter
    def roll(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'roll' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._roll = value

    @builtins.property
    def pitch(self):
        """Message field 'pitch'."""
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch = value

    @builtins.property
    def heading(self):
        """Message field 'heading'."""
        return self._heading

    @heading.setter
    def heading(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'heading' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'heading' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._heading = value

    @builtins.property
    def heave(self):
        """Message field 'heave'."""
        return self._heave

    @heave.setter
    def heave(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'heave' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'heave' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._heave = value

    @builtins.property
    def roll_rate(self):
        """Message field 'roll_rate'."""
        return self._roll_rate

    @roll_rate.setter
    def roll_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll_rate' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'roll_rate' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._roll_rate = value

    @builtins.property
    def pitch_rate(self):
        """Message field 'pitch_rate'."""
        return self._pitch_rate

    @pitch_rate.setter
    def pitch_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch_rate' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch_rate' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch_rate = value

    @builtins.property
    def yaw_rate(self):
        """Message field 'yaw_rate'."""
        return self._yaw_rate

    @yaw_rate.setter
    def yaw_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_rate' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw_rate' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw_rate = value

    @builtins.property
    def north_velocity(self):
        """Message field 'north_velocity'."""
        return self._north_velocity

    @north_velocity.setter
    def north_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'north_velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'north_velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._north_velocity = value

    @builtins.property
    def east_velocity(self):
        """Message field 'east_velocity'."""
        return self._east_velocity

    @east_velocity.setter
    def east_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'east_velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'east_velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._east_velocity = value

    @builtins.property
    def down_velocity(self):
        """Message field 'down_velocity'."""
        return self._down_velocity

    @down_velocity.setter
    def down_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'down_velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'down_velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._down_velocity = value

    @builtins.property
    def latitude_error(self):
        """Message field 'latitude_error'."""
        return self._latitude_error

    @latitude_error.setter
    def latitude_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'latitude_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'latitude_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._latitude_error = value

    @builtins.property
    def longitude_error(self):
        """Message field 'longitude_error'."""
        return self._longitude_error

    @longitude_error.setter
    def longitude_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'longitude_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'longitude_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._longitude_error = value

    @builtins.property
    def height_error(self):
        """Message field 'height_error'."""
        return self._height_error

    @height_error.setter
    def height_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'height_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'height_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._height_error = value

    @builtins.property
    def roll_error(self):
        """Message field 'roll_error'."""
        return self._roll_error

    @roll_error.setter
    def roll_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'roll_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._roll_error = value

    @builtins.property
    def pitch_error(self):
        """Message field 'pitch_error'."""
        return self._pitch_error

    @pitch_error.setter
    def pitch_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch_error = value

    @builtins.property
    def heading_error(self):
        """Message field 'heading_error'."""
        return self._heading_error

    @heading_error.setter
    def heading_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'heading_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'heading_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._heading_error = value

    @builtins.property
    def heave_error(self):
        """Message field 'heave_error'."""
        return self._heave_error

    @heave_error.setter
    def heave_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'heave_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'heave_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._heave_error = value

    @builtins.property
    def north_acceleration(self):
        """Message field 'north_acceleration'."""
        return self._north_acceleration

    @north_acceleration.setter
    def north_acceleration(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'north_acceleration' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'north_acceleration' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._north_acceleration = value

    @builtins.property
    def east_acceleration(self):
        """Message field 'east_acceleration'."""
        return self._east_acceleration

    @east_acceleration.setter
    def east_acceleration(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'east_acceleration' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'east_acceleration' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._east_acceleration = value

    @builtins.property
    def down_acceleration(self):
        """Message field 'down_acceleration'."""
        return self._down_acceleration

    @down_acceleration.setter
    def down_acceleration(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'down_acceleration' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'down_acceleration' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._down_acceleration = value

    @builtins.property
    def delayed_heave_utc_seconds(self):
        """Message field 'delayed_heave_utc_seconds'."""
        return self._delayed_heave_utc_seconds

    @delayed_heave_utc_seconds.setter
    def delayed_heave_utc_seconds(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'delayed_heave_utc_seconds' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'delayed_heave_utc_seconds' field must be an unsigned integer in [0, 4294967295]"
        self._delayed_heave_utc_seconds = value

    @builtins.property
    def delayed_heave_utc_nanoseconds(self):
        """Message field 'delayed_heave_utc_nanoseconds'."""
        return self._delayed_heave_utc_nanoseconds

    @delayed_heave_utc_nanoseconds.setter
    def delayed_heave_utc_nanoseconds(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'delayed_heave_utc_nanoseconds' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'delayed_heave_utc_nanoseconds' field must be an unsigned integer in [0, 4294967295]"
        self._delayed_heave_utc_nanoseconds = value

    @builtins.property
    def delayed_heave(self):
        """Message field 'delayed_heave'."""
        return self._delayed_heave

    @delayed_heave.setter
    def delayed_heave(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'delayed_heave' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'delayed_heave' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._delayed_heave = value
