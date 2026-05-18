# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vortex_msgs:msg/ReferenceFilter.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ReferenceFilter(type):
    """Metaclass of message 'ReferenceFilter'."""

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
                'vortex_msgs.msg.ReferenceFilter')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__reference_filter
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__reference_filter
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__reference_filter
            cls._TYPE_SUPPORT = module.type_support_msg__msg__reference_filter
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__reference_filter

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


class ReferenceFilter(metaclass=Metaclass_ReferenceFilter):
    """Message class 'ReferenceFilter'."""

    __slots__ = [
        '_header',
        '_x',
        '_y',
        '_z',
        '_roll',
        '_pitch',
        '_yaw',
        '_x_dot',
        '_y_dot',
        '_z_dot',
        '_roll_dot',
        '_pitch_dot',
        '_yaw_dot',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'x': 'double',
        'y': 'double',
        'z': 'double',
        'roll': 'double',
        'pitch': 'double',
        'yaw': 'double',
        'x_dot': 'double',
        'y_dot': 'double',
        'z_dot': 'double',
        'roll_dot': 'double',
        'pitch_dot': 'double',
        'yaw_dot': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
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
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.z = kwargs.get('z', float())
        self.roll = kwargs.get('roll', float())
        self.pitch = kwargs.get('pitch', float())
        self.yaw = kwargs.get('yaw', float())
        self.x_dot = kwargs.get('x_dot', float())
        self.y_dot = kwargs.get('y_dot', float())
        self.z_dot = kwargs.get('z_dot', float())
        self.roll_dot = kwargs.get('roll_dot', float())
        self.pitch_dot = kwargs.get('pitch_dot', float())
        self.yaw_dot = kwargs.get('yaw_dot', float())

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
        if self.header != other.header:
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        if self.roll != other.roll:
            return False
        if self.pitch != other.pitch:
            return False
        if self.yaw != other.yaw:
            return False
        if self.x_dot != other.x_dot:
            return False
        if self.y_dot != other.y_dot:
            return False
        if self.z_dot != other.z_dot:
            return False
        if self.roll_dot != other.roll_dot:
            return False
        if self.pitch_dot != other.pitch_dot:
            return False
        if self.yaw_dot != other.yaw_dot:
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
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y = value

    @builtins.property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._z = value

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
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'roll' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
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
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pitch' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pitch = value

    @builtins.property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yaw = value

    @builtins.property
    def x_dot(self):
        """Message field 'x_dot'."""
        return self._x_dot

    @x_dot.setter
    def x_dot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x_dot = value

    @builtins.property
    def y_dot(self):
        """Message field 'y_dot'."""
        return self._y_dot

    @y_dot.setter
    def y_dot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y_dot = value

    @builtins.property
    def z_dot(self):
        """Message field 'z_dot'."""
        return self._z_dot

    @z_dot.setter
    def z_dot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'z_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._z_dot = value

    @builtins.property
    def roll_dot(self):
        """Message field 'roll_dot'."""
        return self._roll_dot

    @roll_dot.setter
    def roll_dot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'roll_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'roll_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._roll_dot = value

    @builtins.property
    def pitch_dot(self):
        """Message field 'pitch_dot'."""
        return self._pitch_dot

    @pitch_dot.setter
    def pitch_dot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pitch_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pitch_dot = value

    @builtins.property
    def yaw_dot(self):
        """Message field 'yaw_dot'."""
        return self._yaw_dot

    @yaw_dot.setter
    def yaw_dot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yaw_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yaw_dot = value
