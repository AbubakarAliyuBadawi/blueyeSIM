# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mundus_mir_msgs:srv/AddWaypointAlt.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AddWaypointAlt_Request(type):
    """Metaclass of message 'AddWaypointAlt_Request'."""

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
                'mundus_mir_msgs.srv.AddWaypointAlt_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__add_waypoint_alt__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__add_waypoint_alt__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__add_waypoint_alt__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__add_waypoint_alt__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__add_waypoint_alt__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AddWaypointAlt_Request(metaclass=Metaclass_AddWaypointAlt_Request):
    """Message class 'AddWaypointAlt_Request'."""

    __slots__ = [
        '_x',
        '_y',
        '_z',
        '_desired_velocity',
        '_fixed_heading',
        '_heading',
        '_altitude_mode',
        '_target_altitude',
    ]

    _fields_and_field_types = {
        'x': 'float',
        'y': 'float',
        'z': 'float',
        'desired_velocity': 'float',
        'fixed_heading': 'boolean',
        'heading': 'float',
        'altitude_mode': 'boolean',
        'target_altitude': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.z = kwargs.get('z', float())
        self.desired_velocity = kwargs.get('desired_velocity', float())
        self.fixed_heading = kwargs.get('fixed_heading', bool())
        self.heading = kwargs.get('heading', float())
        self.altitude_mode = kwargs.get('altitude_mode', bool())
        self.target_altitude = kwargs.get('target_altitude', float())

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
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        if self.desired_velocity != other.desired_velocity:
            return False
        if self.fixed_heading != other.fixed_heading:
            return False
        if self.heading != other.heading:
            return False
        if self.altitude_mode != other.altitude_mode:
            return False
        if self.target_altitude != other.target_altitude:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
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
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
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
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z = value

    @builtins.property
    def desired_velocity(self):
        """Message field 'desired_velocity'."""
        return self._desired_velocity

    @desired_velocity.setter
    def desired_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'desired_velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'desired_velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._desired_velocity = value

    @builtins.property
    def fixed_heading(self):
        """Message field 'fixed_heading'."""
        return self._fixed_heading

    @fixed_heading.setter
    def fixed_heading(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fixed_heading' field must be of type 'bool'"
        self._fixed_heading = value

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
    def altitude_mode(self):
        """Message field 'altitude_mode'."""
        return self._altitude_mode

    @altitude_mode.setter
    def altitude_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'altitude_mode' field must be of type 'bool'"
        self._altitude_mode = value

    @builtins.property
    def target_altitude(self):
        """Message field 'target_altitude'."""
        return self._target_altitude

    @target_altitude.setter
    def target_altitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_altitude' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'target_altitude' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._target_altitude = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_AddWaypointAlt_Response(type):
    """Metaclass of message 'AddWaypointAlt_Response'."""

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
                'mundus_mir_msgs.srv.AddWaypointAlt_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__add_waypoint_alt__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__add_waypoint_alt__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__add_waypoint_alt__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__add_waypoint_alt__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__add_waypoint_alt__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AddWaypointAlt_Response(metaclass=Metaclass_AddWaypointAlt_Response):
    """Message class 'AddWaypointAlt_Response'."""

    __slots__ = [
        '_accepted',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())

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
        if self.accepted != other.accepted:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value


class Metaclass_AddWaypointAlt(type):
    """Metaclass of service 'AddWaypointAlt'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mundus_mir_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mundus_mir_msgs.srv.AddWaypointAlt')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__add_waypoint_alt

            from mundus_mir_msgs.srv import _add_waypoint_alt
            if _add_waypoint_alt.Metaclass_AddWaypointAlt_Request._TYPE_SUPPORT is None:
                _add_waypoint_alt.Metaclass_AddWaypointAlt_Request.__import_type_support__()
            if _add_waypoint_alt.Metaclass_AddWaypointAlt_Response._TYPE_SUPPORT is None:
                _add_waypoint_alt.Metaclass_AddWaypointAlt_Response.__import_type_support__()


class AddWaypointAlt(metaclass=Metaclass_AddWaypointAlt):
    from mundus_mir_msgs.srv._add_waypoint_alt import AddWaypointAlt_Request as Request
    from mundus_mir_msgs.srv._add_waypoint_alt import AddWaypointAlt_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
