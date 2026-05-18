# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mundus_mir_msgs:msg/ControllerState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ControllerState(type):
    """Metaclass of message 'ControllerState'."""

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
            module = import_type_support('mundus_mir_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mundus_mir_msgs.msg.ControllerState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__controller_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__controller_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__controller_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__controller_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__controller_state

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


class ControllerState(metaclass=Metaclass_ControllerState):
    """Message class 'ControllerState'."""

    __slots__ = [
        '_header',
        '_q',
        '_m1',
        '_m2',
        '_m3',
        '_m4',
        '_d1',
        '_d2',
        '_d3',
        '_d4',
        '_d5',
        '_d6',
        '_d7',
        '_d8',
        '_bias_x',
        '_bias_y',
        '_bias_z',
        '_bias_ang1',
        '_bias_ang2',
        '_bias_ang3',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'q': 'int64',
        'm1': 'double',
        'm2': 'double',
        'm3': 'double',
        'm4': 'double',
        'd1': 'double',
        'd2': 'double',
        'd3': 'double',
        'd4': 'double',
        'd5': 'double',
        'd6': 'double',
        'd7': 'double',
        'd8': 'double',
        'bias_x': 'double',
        'bias_y': 'double',
        'bias_z': 'double',
        'bias_ang1': 'double',
        'bias_ang2': 'double',
        'bias_ang3': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
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
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.q = kwargs.get('q', int())
        self.m1 = kwargs.get('m1', float())
        self.m2 = kwargs.get('m2', float())
        self.m3 = kwargs.get('m3', float())
        self.m4 = kwargs.get('m4', float())
        self.d1 = kwargs.get('d1', float())
        self.d2 = kwargs.get('d2', float())
        self.d3 = kwargs.get('d3', float())
        self.d4 = kwargs.get('d4', float())
        self.d5 = kwargs.get('d5', float())
        self.d6 = kwargs.get('d6', float())
        self.d7 = kwargs.get('d7', float())
        self.d8 = kwargs.get('d8', float())
        self.bias_x = kwargs.get('bias_x', float())
        self.bias_y = kwargs.get('bias_y', float())
        self.bias_z = kwargs.get('bias_z', float())
        self.bias_ang1 = kwargs.get('bias_ang1', float())
        self.bias_ang2 = kwargs.get('bias_ang2', float())
        self.bias_ang3 = kwargs.get('bias_ang3', float())

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
        if self.q != other.q:
            return False
        if self.m1 != other.m1:
            return False
        if self.m2 != other.m2:
            return False
        if self.m3 != other.m3:
            return False
        if self.m4 != other.m4:
            return False
        if self.d1 != other.d1:
            return False
        if self.d2 != other.d2:
            return False
        if self.d3 != other.d3:
            return False
        if self.d4 != other.d4:
            return False
        if self.d5 != other.d5:
            return False
        if self.d6 != other.d6:
            return False
        if self.d7 != other.d7:
            return False
        if self.d8 != other.d8:
            return False
        if self.bias_x != other.bias_x:
            return False
        if self.bias_y != other.bias_y:
            return False
        if self.bias_z != other.bias_z:
            return False
        if self.bias_ang1 != other.bias_ang1:
            return False
        if self.bias_ang2 != other.bias_ang2:
            return False
        if self.bias_ang3 != other.bias_ang3:
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
    def q(self):
        """Message field 'q'."""
        return self._q

    @q.setter
    def q(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'q' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'q' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._q = value

    @builtins.property
    def m1(self):
        """Message field 'm1'."""
        return self._m1

    @m1.setter
    def m1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'm1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'm1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._m1 = value

    @builtins.property
    def m2(self):
        """Message field 'm2'."""
        return self._m2

    @m2.setter
    def m2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'm2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'm2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._m2 = value

    @builtins.property
    def m3(self):
        """Message field 'm3'."""
        return self._m3

    @m3.setter
    def m3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'm3' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'm3' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._m3 = value

    @builtins.property
    def m4(self):
        """Message field 'm4'."""
        return self._m4

    @m4.setter
    def m4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'm4' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'm4' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._m4 = value

    @builtins.property
    def d1(self):
        """Message field 'd1'."""
        return self._d1

    @d1.setter
    def d1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d1 = value

    @builtins.property
    def d2(self):
        """Message field 'd2'."""
        return self._d2

    @d2.setter
    def d2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d2 = value

    @builtins.property
    def d3(self):
        """Message field 'd3'."""
        return self._d3

    @d3.setter
    def d3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd3' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd3' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d3 = value

    @builtins.property
    def d4(self):
        """Message field 'd4'."""
        return self._d4

    @d4.setter
    def d4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd4' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd4' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d4 = value

    @builtins.property
    def d5(self):
        """Message field 'd5'."""
        return self._d5

    @d5.setter
    def d5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd5' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd5' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d5 = value

    @builtins.property
    def d6(self):
        """Message field 'd6'."""
        return self._d6

    @d6.setter
    def d6(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd6' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd6' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d6 = value

    @builtins.property
    def d7(self):
        """Message field 'd7'."""
        return self._d7

    @d7.setter
    def d7(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd7' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd7' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d7 = value

    @builtins.property
    def d8(self):
        """Message field 'd8'."""
        return self._d8

    @d8.setter
    def d8(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd8' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd8' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d8 = value

    @builtins.property
    def bias_x(self):
        """Message field 'bias_x'."""
        return self._bias_x

    @bias_x.setter
    def bias_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bias_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'bias_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._bias_x = value

    @builtins.property
    def bias_y(self):
        """Message field 'bias_y'."""
        return self._bias_y

    @bias_y.setter
    def bias_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bias_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'bias_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._bias_y = value

    @builtins.property
    def bias_z(self):
        """Message field 'bias_z'."""
        return self._bias_z

    @bias_z.setter
    def bias_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bias_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'bias_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._bias_z = value

    @builtins.property
    def bias_ang1(self):
        """Message field 'bias_ang1'."""
        return self._bias_ang1

    @bias_ang1.setter
    def bias_ang1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bias_ang1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'bias_ang1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._bias_ang1 = value

    @builtins.property
    def bias_ang2(self):
        """Message field 'bias_ang2'."""
        return self._bias_ang2

    @bias_ang2.setter
    def bias_ang2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bias_ang2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'bias_ang2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._bias_ang2 = value

    @builtins.property
    def bias_ang3(self):
        """Message field 'bias_ang3'."""
        return self._bias_ang3

    @bias_ang3.setter
    def bias_ang3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bias_ang3' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'bias_ang3' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._bias_ang3 = value
