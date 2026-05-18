# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mundus_mir_msgs:msg/ControlActions.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ControlActions(type):
    """Metaclass of message 'ControlActions'."""

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
                'mundus_mir_msgs.msg.ControlActions')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__control_actions
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__control_actions
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__control_actions
            cls._TYPE_SUPPORT = module.type_support_msg__msg__control_actions
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__control_actions

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


class ControlActions(metaclass=Metaclass_ControlActions):
    """Message class 'ControlActions'."""

    __slots__ = [
        '_header',
        '_prop_action_linear',
        '_deriv_action_linear',
        '_integral_action_linear',
        '_prop_action_angular',
        '_deriv_action_angular',
        '_integral_action_angular',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'prop_action_linear': 'geometry_msgs/Vector3',
        'deriv_action_linear': 'geometry_msgs/Vector3',
        'integral_action_linear': 'geometry_msgs/Vector3',
        'prop_action_angular': 'double',
        'deriv_action_angular': 'double',
        'integral_action_angular': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
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
        from geometry_msgs.msg import Vector3
        self.prop_action_linear = kwargs.get('prop_action_linear', Vector3())
        from geometry_msgs.msg import Vector3
        self.deriv_action_linear = kwargs.get('deriv_action_linear', Vector3())
        from geometry_msgs.msg import Vector3
        self.integral_action_linear = kwargs.get('integral_action_linear', Vector3())
        self.prop_action_angular = kwargs.get('prop_action_angular', float())
        self.deriv_action_angular = kwargs.get('deriv_action_angular', float())
        self.integral_action_angular = kwargs.get('integral_action_angular', float())

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
        if self.prop_action_linear != other.prop_action_linear:
            return False
        if self.deriv_action_linear != other.deriv_action_linear:
            return False
        if self.integral_action_linear != other.integral_action_linear:
            return False
        if self.prop_action_angular != other.prop_action_angular:
            return False
        if self.deriv_action_angular != other.deriv_action_angular:
            return False
        if self.integral_action_angular != other.integral_action_angular:
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
    def prop_action_linear(self):
        """Message field 'prop_action_linear'."""
        return self._prop_action_linear

    @prop_action_linear.setter
    def prop_action_linear(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'prop_action_linear' field must be a sub message of type 'Vector3'"
        self._prop_action_linear = value

    @builtins.property
    def deriv_action_linear(self):
        """Message field 'deriv_action_linear'."""
        return self._deriv_action_linear

    @deriv_action_linear.setter
    def deriv_action_linear(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'deriv_action_linear' field must be a sub message of type 'Vector3'"
        self._deriv_action_linear = value

    @builtins.property
    def integral_action_linear(self):
        """Message field 'integral_action_linear'."""
        return self._integral_action_linear

    @integral_action_linear.setter
    def integral_action_linear(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'integral_action_linear' field must be a sub message of type 'Vector3'"
        self._integral_action_linear = value

    @builtins.property
    def prop_action_angular(self):
        """Message field 'prop_action_angular'."""
        return self._prop_action_angular

    @prop_action_angular.setter
    def prop_action_angular(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'prop_action_angular' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'prop_action_angular' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._prop_action_angular = value

    @builtins.property
    def deriv_action_angular(self):
        """Message field 'deriv_action_angular'."""
        return self._deriv_action_angular

    @deriv_action_angular.setter
    def deriv_action_angular(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'deriv_action_angular' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'deriv_action_angular' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._deriv_action_angular = value

    @builtins.property
    def integral_action_angular(self):
        """Message field 'integral_action_angular'."""
        return self._integral_action_angular

    @integral_action_angular.setter
    def integral_action_angular(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'integral_action_angular' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'integral_action_angular' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._integral_action_angular = value
