# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vortex_msgs:msg/Pwm.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'pins'
# Member 'positive_width_us'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Pwm(type):
    """Metaclass of message 'Pwm'."""

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
                'vortex_msgs.msg.Pwm')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__pwm
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__pwm
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__pwm
            cls._TYPE_SUPPORT = module.type_support_msg__msg__pwm
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__pwm

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Pwm(metaclass=Metaclass_Pwm):
    """Message class 'Pwm'."""

    __slots__ = [
        '_pins',
        '_positive_width_us',
    ]

    _fields_and_field_types = {
        'pins': 'sequence<uint16>',
        'positive_width_us': 'sequence<uint16>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.pins = array.array('H', kwargs.get('pins', []))
        self.positive_width_us = array.array('H', kwargs.get('positive_width_us', []))

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
        if self.pins != other.pins:
            return False
        if self.positive_width_us != other.positive_width_us:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pins(self):
        """Message field 'pins'."""
        return self._pins

    @pins.setter
    def pins(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'H', \
                "The 'pins' array.array() must have the type code of 'H'"
            self._pins = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'pins' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._pins = array.array('H', value)

    @builtins.property
    def positive_width_us(self):
        """Message field 'positive_width_us'."""
        return self._positive_width_us

    @positive_width_us.setter
    def positive_width_us(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'H', \
                "The 'positive_width_us' array.array() must have the type code of 'H'"
            self._positive_width_us = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'positive_width_us' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._positive_width_us = array.array('H', value)
