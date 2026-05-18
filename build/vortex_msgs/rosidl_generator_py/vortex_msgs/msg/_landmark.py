# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vortex_msgs:msg/Landmark.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Landmark(type):
    """Metaclass of message 'Landmark'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'REMOVE_ACTION': 0,
        'ADD_ACTION': 1,
        'UPDATE_ACTION': 2,
        'NONE': 0,
        'BUOY': 1,
        'BOAT': 2,
        'WALL': 69,
        'UNKNOWN': 0,
        'RED_BUOY': 1,
        'GREEN_BUOY': 2,
        'NORTH_MARK': 3,
        'SOUTH_MARK': 4,
        'EAST_MARK': 5,
        'WEST_MARK': 6,
        'MOVING_BOAT': 7,
        'STATIC_BOAT': 8,
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
                'vortex_msgs.msg.Landmark')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__landmark
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__landmark
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__landmark
            cls._TYPE_SUPPORT = module.type_support_msg__msg__landmark
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__landmark

            from nav_msgs.msg import Odometry
            if Odometry.__class__._TYPE_SUPPORT is None:
                Odometry.__class__.__import_type_support__()

            from shape_msgs.msg import SolidPrimitive
            if SolidPrimitive.__class__._TYPE_SUPPORT is None:
                SolidPrimitive.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'REMOVE_ACTION': cls.__constants['REMOVE_ACTION'],
            'ADD_ACTION': cls.__constants['ADD_ACTION'],
            'UPDATE_ACTION': cls.__constants['UPDATE_ACTION'],
            'NONE': cls.__constants['NONE'],
            'BUOY': cls.__constants['BUOY'],
            'BOAT': cls.__constants['BOAT'],
            'WALL': cls.__constants['WALL'],
            'UNKNOWN': cls.__constants['UNKNOWN'],
            'RED_BUOY': cls.__constants['RED_BUOY'],
            'GREEN_BUOY': cls.__constants['GREEN_BUOY'],
            'NORTH_MARK': cls.__constants['NORTH_MARK'],
            'SOUTH_MARK': cls.__constants['SOUTH_MARK'],
            'EAST_MARK': cls.__constants['EAST_MARK'],
            'WEST_MARK': cls.__constants['WEST_MARK'],
            'MOVING_BOAT': cls.__constants['MOVING_BOAT'],
            'STATIC_BOAT': cls.__constants['STATIC_BOAT'],
            'LANDMARK_TYPE__DEFAULT': 0,
            'ACTION__DEFAULT': 1,
            'CLASSIFICATION__DEFAULT': 0,
        }

    @property
    def REMOVE_ACTION(self):
        """Message constant 'REMOVE_ACTION'."""
        return Metaclass_Landmark.__constants['REMOVE_ACTION']

    @property
    def ADD_ACTION(self):
        """Message constant 'ADD_ACTION'."""
        return Metaclass_Landmark.__constants['ADD_ACTION']

    @property
    def UPDATE_ACTION(self):
        """Message constant 'UPDATE_ACTION'."""
        return Metaclass_Landmark.__constants['UPDATE_ACTION']

    @property
    def NONE(self):
        """Message constant 'NONE'."""
        return Metaclass_Landmark.__constants['NONE']

    @property
    def BUOY(self):
        """Message constant 'BUOY'."""
        return Metaclass_Landmark.__constants['BUOY']

    @property
    def BOAT(self):
        """Message constant 'BOAT'."""
        return Metaclass_Landmark.__constants['BOAT']

    @property
    def WALL(self):
        """Message constant 'WALL'."""
        return Metaclass_Landmark.__constants['WALL']

    @property
    def UNKNOWN(self):
        """Message constant 'UNKNOWN'."""
        return Metaclass_Landmark.__constants['UNKNOWN']

    @property
    def RED_BUOY(self):
        """Message constant 'RED_BUOY'."""
        return Metaclass_Landmark.__constants['RED_BUOY']

    @property
    def GREEN_BUOY(self):
        """Message constant 'GREEN_BUOY'."""
        return Metaclass_Landmark.__constants['GREEN_BUOY']

    @property
    def NORTH_MARK(self):
        """Message constant 'NORTH_MARK'."""
        return Metaclass_Landmark.__constants['NORTH_MARK']

    @property
    def SOUTH_MARK(self):
        """Message constant 'SOUTH_MARK'."""
        return Metaclass_Landmark.__constants['SOUTH_MARK']

    @property
    def EAST_MARK(self):
        """Message constant 'EAST_MARK'."""
        return Metaclass_Landmark.__constants['EAST_MARK']

    @property
    def WEST_MARK(self):
        """Message constant 'WEST_MARK'."""
        return Metaclass_Landmark.__constants['WEST_MARK']

    @property
    def MOVING_BOAT(self):
        """Message constant 'MOVING_BOAT'."""
        return Metaclass_Landmark.__constants['MOVING_BOAT']

    @property
    def STATIC_BOAT(self):
        """Message constant 'STATIC_BOAT'."""
        return Metaclass_Landmark.__constants['STATIC_BOAT']

    @property
    def LANDMARK_TYPE__DEFAULT(cls):
        """Return default value for message field 'landmark_type'."""
        return 0

    @property
    def ACTION__DEFAULT(cls):
        """Return default value for message field 'action'."""
        return 1

    @property
    def CLASSIFICATION__DEFAULT(cls):
        """Return default value for message field 'classification'."""
        return 0


class Landmark(metaclass=Metaclass_Landmark):
    """
    Message class 'Landmark'.

    Constants:
      REMOVE_ACTION
      ADD_ACTION
      UPDATE_ACTION
      NONE
      BUOY
      BOAT
      WALL
      UNKNOWN
      RED_BUOY
      GREEN_BUOY
      NORTH_MARK
      SOUTH_MARK
      EAST_MARK
      WEST_MARK
      MOVING_BOAT
      STATIC_BOAT
    """

    __slots__ = [
        '_landmark_type',
        '_id',
        '_action',
        '_classification',
        '_odom',
        '_shape',
    ]

    _fields_and_field_types = {
        'landmark_type': 'uint8',
        'id': 'uint32',
        'action': 'uint8',
        'classification': 'uint8',
        'odom': 'nav_msgs/Odometry',
        'shape': 'shape_msgs/SolidPrimitive',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['nav_msgs', 'msg'], 'Odometry'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['shape_msgs', 'msg'], 'SolidPrimitive'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.landmark_type = kwargs.get(
            'landmark_type', Landmark.LANDMARK_TYPE__DEFAULT)
        self.id = kwargs.get('id', int())
        self.action = kwargs.get(
            'action', Landmark.ACTION__DEFAULT)
        self.classification = kwargs.get(
            'classification', Landmark.CLASSIFICATION__DEFAULT)
        from nav_msgs.msg import Odometry
        self.odom = kwargs.get('odom', Odometry())
        from shape_msgs.msg import SolidPrimitive
        self.shape = kwargs.get('shape', SolidPrimitive())

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
        if self.landmark_type != other.landmark_type:
            return False
        if self.id != other.id:
            return False
        if self.action != other.action:
            return False
        if self.classification != other.classification:
            return False
        if self.odom != other.odom:
            return False
        if self.shape != other.shape:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def landmark_type(self):
        """Message field 'landmark_type'."""
        return self._landmark_type

    @landmark_type.setter
    def landmark_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'landmark_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'landmark_type' field must be an unsigned integer in [0, 255]"
        self._landmark_type = value

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'id' field must be an unsigned integer in [0, 4294967295]"
        self._id = value

    @builtins.property
    def action(self):
        """Message field 'action'."""
        return self._action

    @action.setter
    def action(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'action' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'action' field must be an unsigned integer in [0, 255]"
        self._action = value

    @builtins.property
    def classification(self):
        """Message field 'classification'."""
        return self._classification

    @classification.setter
    def classification(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'classification' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'classification' field must be an unsigned integer in [0, 255]"
        self._classification = value

    @builtins.property
    def odom(self):
        """Message field 'odom'."""
        return self._odom

    @odom.setter
    def odom(self, value):
        if __debug__:
            from nav_msgs.msg import Odometry
            assert \
                isinstance(value, Odometry), \
                "The 'odom' field must be a sub message of type 'Odometry'"
        self._odom = value

    @builtins.property
    def shape(self):
        """Message field 'shape'."""
        return self._shape

    @shape.setter
    def shape(self, value):
        if __debug__:
            from shape_msgs.msg import SolidPrimitive
            assert \
                isinstance(value, SolidPrimitive), \
                "The 'shape' field must be a sub message of type 'SolidPrimitive'"
        self._shape = value
