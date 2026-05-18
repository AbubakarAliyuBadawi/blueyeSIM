# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mundus_mir_msgs:srv/GoToWaypoints.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GoToWaypoints_Request(type):
    """Metaclass of message 'GoToWaypoints_Request'."""

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
                'mundus_mir_msgs.srv.GoToWaypoints_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__go_to_waypoints__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__go_to_waypoints__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__go_to_waypoints__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__go_to_waypoints__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__go_to_waypoints__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GoToWaypoints_Request(metaclass=Metaclass_GoToWaypoints_Request):
    """Message class 'GoToWaypoints_Request'."""

    __slots__ = [
        '_run',
    ]

    _fields_and_field_types = {
        'run': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.run = kwargs.get('run', bool())

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
        if self.run != other.run:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def run(self):
        """Message field 'run'."""
        return self._run

    @run.setter
    def run(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'run' field must be of type 'bool'"
        self._run = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GoToWaypoints_Response(type):
    """Metaclass of message 'GoToWaypoints_Response'."""

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
                'mundus_mir_msgs.srv.GoToWaypoints_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__go_to_waypoints__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__go_to_waypoints__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__go_to_waypoints__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__go_to_waypoints__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__go_to_waypoints__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GoToWaypoints_Response(metaclass=Metaclass_GoToWaypoints_Response):
    """Message class 'GoToWaypoints_Response'."""

    __slots__ = [
        '_accepted',
        '_status_code',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'status_code': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        self.status_code = kwargs.get('status_code', str())

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
        if self.status_code != other.status_code:
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

    @builtins.property
    def status_code(self):
        """Message field 'status_code'."""
        return self._status_code

    @status_code.setter
    def status_code(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'status_code' field must be of type 'str'"
        self._status_code = value


class Metaclass_GoToWaypoints(type):
    """Metaclass of service 'GoToWaypoints'."""

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
                'mundus_mir_msgs.srv.GoToWaypoints')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__go_to_waypoints

            from mundus_mir_msgs.srv import _go_to_waypoints
            if _go_to_waypoints.Metaclass_GoToWaypoints_Request._TYPE_SUPPORT is None:
                _go_to_waypoints.Metaclass_GoToWaypoints_Request.__import_type_support__()
            if _go_to_waypoints.Metaclass_GoToWaypoints_Response._TYPE_SUPPORT is None:
                _go_to_waypoints.Metaclass_GoToWaypoints_Response.__import_type_support__()


class GoToWaypoints(metaclass=Metaclass_GoToWaypoints):
    from mundus_mir_msgs.srv._go_to_waypoints import GoToWaypoints_Request as Request
    from mundus_mir_msgs.srv._go_to_waypoints import GoToWaypoints_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
