# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mundus_mir_msgs:msg/ActuatorInput.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ActuatorInput(type):
    """Metaclass of message 'ActuatorInput'."""

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
                'mundus_mir_msgs.msg.ActuatorInput')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__actuator_input
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__actuator_input
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__actuator_input
            cls._TYPE_SUPPORT = module.type_support_msg__msg__actuator_input
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__actuator_input

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


class ActuatorInput(metaclass=Metaclass_ActuatorInput):
    """Message class 'ActuatorInput'."""

    __slots__ = [
        '_header',
        '_thrust1',
        '_thrust2',
        '_thrust3',
        '_thrust4',
        '_thrust5',
        '_thrust6',
        '_thrust7',
        '_thrust8',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'thrust1': 'double',
        'thrust2': 'double',
        'thrust3': 'double',
        'thrust4': 'double',
        'thrust5': 'double',
        'thrust6': 'double',
        'thrust7': 'double',
        'thrust8': 'double',
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
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.thrust1 = kwargs.get('thrust1', float())
        self.thrust2 = kwargs.get('thrust2', float())
        self.thrust3 = kwargs.get('thrust3', float())
        self.thrust4 = kwargs.get('thrust4', float())
        self.thrust5 = kwargs.get('thrust5', float())
        self.thrust6 = kwargs.get('thrust6', float())
        self.thrust7 = kwargs.get('thrust7', float())
        self.thrust8 = kwargs.get('thrust8', float())

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
        if self.thrust1 != other.thrust1:
            return False
        if self.thrust2 != other.thrust2:
            return False
        if self.thrust3 != other.thrust3:
            return False
        if self.thrust4 != other.thrust4:
            return False
        if self.thrust5 != other.thrust5:
            return False
        if self.thrust6 != other.thrust6:
            return False
        if self.thrust7 != other.thrust7:
            return False
        if self.thrust8 != other.thrust8:
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
    def thrust1(self):
        """Message field 'thrust1'."""
        return self._thrust1

    @thrust1.setter
    def thrust1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thrust1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'thrust1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._thrust1 = value

    @builtins.property
    def thrust2(self):
        """Message field 'thrust2'."""
        return self._thrust2

    @thrust2.setter
    def thrust2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thrust2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'thrust2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._thrust2 = value

    @builtins.property
    def thrust3(self):
        """Message field 'thrust3'."""
        return self._thrust3

    @thrust3.setter
    def thrust3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thrust3' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'thrust3' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._thrust3 = value

    @builtins.property
    def thrust4(self):
        """Message field 'thrust4'."""
        return self._thrust4

    @thrust4.setter
    def thrust4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thrust4' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'thrust4' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._thrust4 = value

    @builtins.property
    def thrust5(self):
        """Message field 'thrust5'."""
        return self._thrust5

    @thrust5.setter
    def thrust5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thrust5' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'thrust5' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._thrust5 = value

    @builtins.property
    def thrust6(self):
        """Message field 'thrust6'."""
        return self._thrust6

    @thrust6.setter
    def thrust6(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thrust6' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'thrust6' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._thrust6 = value

    @builtins.property
    def thrust7(self):
        """Message field 'thrust7'."""
        return self._thrust7

    @thrust7.setter
    def thrust7(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thrust7' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'thrust7' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._thrust7 = value

    @builtins.property
    def thrust8(self):
        """Message field 'thrust8'."""
        return self._thrust8

    @thrust8.setter
    def thrust8(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thrust8' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'thrust8' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._thrust8 = value
