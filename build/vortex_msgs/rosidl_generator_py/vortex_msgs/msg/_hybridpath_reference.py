# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vortex_msgs:msg/HybridpathReference.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HybridpathReference(type):
    """Metaclass of message 'HybridpathReference'."""

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
                'vortex_msgs.msg.HybridpathReference')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__hybridpath_reference
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__hybridpath_reference
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__hybridpath_reference
            cls._TYPE_SUPPORT = module.type_support_msg__msg__hybridpath_reference
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__hybridpath_reference

            from geometry_msgs.msg import Pose2D
            if Pose2D.__class__._TYPE_SUPPORT is None:
                Pose2D.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class HybridpathReference(metaclass=Metaclass_HybridpathReference):
    """Message class 'HybridpathReference'."""

    __slots__ = [
        '_w',
        '_v_s',
        '_v_ss',
        '_eta_d',
        '_eta_d_s',
        '_eta_d_ss',
    ]

    _fields_and_field_types = {
        'w': 'double',
        'v_s': 'double',
        'v_ss': 'double',
        'eta_d': 'geometry_msgs/Pose2D',
        'eta_d_s': 'geometry_msgs/Pose2D',
        'eta_d_ss': 'geometry_msgs/Pose2D',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose2D'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose2D'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose2D'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.w = kwargs.get('w', float())
        self.v_s = kwargs.get('v_s', float())
        self.v_ss = kwargs.get('v_ss', float())
        from geometry_msgs.msg import Pose2D
        self.eta_d = kwargs.get('eta_d', Pose2D())
        from geometry_msgs.msg import Pose2D
        self.eta_d_s = kwargs.get('eta_d_s', Pose2D())
        from geometry_msgs.msg import Pose2D
        self.eta_d_ss = kwargs.get('eta_d_ss', Pose2D())

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
        if self.w != other.w:
            return False
        if self.v_s != other.v_s:
            return False
        if self.v_ss != other.v_ss:
            return False
        if self.eta_d != other.eta_d:
            return False
        if self.eta_d_s != other.eta_d_s:
            return False
        if self.eta_d_ss != other.eta_d_ss:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def w(self):
        """Message field 'w'."""
        return self._w

    @w.setter
    def w(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'w' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'w' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._w = value

    @builtins.property
    def v_s(self):
        """Message field 'v_s'."""
        return self._v_s

    @v_s.setter
    def v_s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'v_s' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'v_s' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._v_s = value

    @builtins.property
    def v_ss(self):
        """Message field 'v_ss'."""
        return self._v_ss

    @v_ss.setter
    def v_ss(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'v_ss' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'v_ss' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._v_ss = value

    @builtins.property
    def eta_d(self):
        """Message field 'eta_d'."""
        return self._eta_d

    @eta_d.setter
    def eta_d(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose2D
            assert \
                isinstance(value, Pose2D), \
                "The 'eta_d' field must be a sub message of type 'Pose2D'"
        self._eta_d = value

    @builtins.property
    def eta_d_s(self):
        """Message field 'eta_d_s'."""
        return self._eta_d_s

    @eta_d_s.setter
    def eta_d_s(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose2D
            assert \
                isinstance(value, Pose2D), \
                "The 'eta_d_s' field must be a sub message of type 'Pose2D'"
        self._eta_d_s = value

    @builtins.property
    def eta_d_ss(self):
        """Message field 'eta_d_ss'."""
        return self._eta_d_ss

    @eta_d_ss.setter
    def eta_d_ss(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose2D
            assert \
                isinstance(value, Pose2D), \
                "The 'eta_d_ss' field must be a sub message of type 'Pose2D'"
        self._eta_d_ss = value
