# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mundus_mir_msgs:msg/ReturnRecommendation.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'consumption_rates'
# Member 'speeds'
# Member 'timestamps'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ReturnRecommendation(type):
    """Metaclass of message 'ReturnRecommendation'."""

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
                'mundus_mir_msgs.msg.ReturnRecommendation')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__return_recommendation
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__return_recommendation
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__return_recommendation
            cls._TYPE_SUPPORT = module.type_support_msg__msg__return_recommendation
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__return_recommendation

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ReturnRecommendation(metaclass=Metaclass_ReturnRecommendation):
    """Message class 'ReturnRecommendation'."""

    __slots__ = [
        '_stamp',
        '_should_return',
        '_current_battery_level',
        '_distance_to_dock',
        '_current_speed',
        '_current_consumption_rate',
        '_estimated_return_energy',
        '_estimated_time_to_return',
        '_minimum_battery_needed',
        '_safety_margin_percent',
        '_battery_safety_threshold',
        '_consumption_rates',
        '_speeds',
        '_timestamps',
    ]

    _fields_and_field_types = {
        'stamp': 'builtin_interfaces/Time',
        'should_return': 'boolean',
        'current_battery_level': 'double',
        'distance_to_dock': 'double',
        'current_speed': 'double',
        'current_consumption_rate': 'double',
        'estimated_return_energy': 'double',
        'estimated_time_to_return': 'double',
        'minimum_battery_needed': 'double',
        'safety_margin_percent': 'double',
        'battery_safety_threshold': 'double',
        'consumption_rates': 'sequence<double>',
        'speeds': 'sequence<double>',
        'timestamps': 'sequence<double>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())
        self.should_return = kwargs.get('should_return', bool())
        self.current_battery_level = kwargs.get('current_battery_level', float())
        self.distance_to_dock = kwargs.get('distance_to_dock', float())
        self.current_speed = kwargs.get('current_speed', float())
        self.current_consumption_rate = kwargs.get('current_consumption_rate', float())
        self.estimated_return_energy = kwargs.get('estimated_return_energy', float())
        self.estimated_time_to_return = kwargs.get('estimated_time_to_return', float())
        self.minimum_battery_needed = kwargs.get('minimum_battery_needed', float())
        self.safety_margin_percent = kwargs.get('safety_margin_percent', float())
        self.battery_safety_threshold = kwargs.get('battery_safety_threshold', float())
        self.consumption_rates = array.array('d', kwargs.get('consumption_rates', []))
        self.speeds = array.array('d', kwargs.get('speeds', []))
        self.timestamps = array.array('d', kwargs.get('timestamps', []))

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
        if self.stamp != other.stamp:
            return False
        if self.should_return != other.should_return:
            return False
        if self.current_battery_level != other.current_battery_level:
            return False
        if self.distance_to_dock != other.distance_to_dock:
            return False
        if self.current_speed != other.current_speed:
            return False
        if self.current_consumption_rate != other.current_consumption_rate:
            return False
        if self.estimated_return_energy != other.estimated_return_energy:
            return False
        if self.estimated_time_to_return != other.estimated_time_to_return:
            return False
        if self.minimum_battery_needed != other.minimum_battery_needed:
            return False
        if self.safety_margin_percent != other.safety_margin_percent:
            return False
        if self.battery_safety_threshold != other.battery_safety_threshold:
            return False
        if self.consumption_rates != other.consumption_rates:
            return False
        if self.speeds != other.speeds:
            return False
        if self.timestamps != other.timestamps:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value

    @builtins.property
    def should_return(self):
        """Message field 'should_return'."""
        return self._should_return

    @should_return.setter
    def should_return(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'should_return' field must be of type 'bool'"
        self._should_return = value

    @builtins.property
    def current_battery_level(self):
        """Message field 'current_battery_level'."""
        return self._current_battery_level

    @current_battery_level.setter
    def current_battery_level(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_battery_level' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'current_battery_level' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._current_battery_level = value

    @builtins.property
    def distance_to_dock(self):
        """Message field 'distance_to_dock'."""
        return self._distance_to_dock

    @distance_to_dock.setter
    def distance_to_dock(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance_to_dock' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'distance_to_dock' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._distance_to_dock = value

    @builtins.property
    def current_speed(self):
        """Message field 'current_speed'."""
        return self._current_speed

    @current_speed.setter
    def current_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'current_speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._current_speed = value

    @builtins.property
    def current_consumption_rate(self):
        """Message field 'current_consumption_rate'."""
        return self._current_consumption_rate

    @current_consumption_rate.setter
    def current_consumption_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_consumption_rate' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'current_consumption_rate' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._current_consumption_rate = value

    @builtins.property
    def estimated_return_energy(self):
        """Message field 'estimated_return_energy'."""
        return self._estimated_return_energy

    @estimated_return_energy.setter
    def estimated_return_energy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'estimated_return_energy' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'estimated_return_energy' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._estimated_return_energy = value

    @builtins.property
    def estimated_time_to_return(self):
        """Message field 'estimated_time_to_return'."""
        return self._estimated_time_to_return

    @estimated_time_to_return.setter
    def estimated_time_to_return(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'estimated_time_to_return' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'estimated_time_to_return' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._estimated_time_to_return = value

    @builtins.property
    def minimum_battery_needed(self):
        """Message field 'minimum_battery_needed'."""
        return self._minimum_battery_needed

    @minimum_battery_needed.setter
    def minimum_battery_needed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'minimum_battery_needed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'minimum_battery_needed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._minimum_battery_needed = value

    @builtins.property
    def safety_margin_percent(self):
        """Message field 'safety_margin_percent'."""
        return self._safety_margin_percent

    @safety_margin_percent.setter
    def safety_margin_percent(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'safety_margin_percent' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'safety_margin_percent' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._safety_margin_percent = value

    @builtins.property
    def battery_safety_threshold(self):
        """Message field 'battery_safety_threshold'."""
        return self._battery_safety_threshold

    @battery_safety_threshold.setter
    def battery_safety_threshold(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_safety_threshold' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'battery_safety_threshold' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._battery_safety_threshold = value

    @builtins.property
    def consumption_rates(self):
        """Message field 'consumption_rates'."""
        return self._consumption_rates

    @consumption_rates.setter
    def consumption_rates(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'consumption_rates' array.array() must have the type code of 'd'"
            self._consumption_rates = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'consumption_rates' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._consumption_rates = array.array('d', value)

    @builtins.property
    def speeds(self):
        """Message field 'speeds'."""
        return self._speeds

    @speeds.setter
    def speeds(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'speeds' array.array() must have the type code of 'd'"
            self._speeds = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'speeds' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._speeds = array.array('d', value)

    @builtins.property
    def timestamps(self):
        """Message field 'timestamps'."""
        return self._timestamps

    @timestamps.setter
    def timestamps(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'timestamps' array.array() must have the type code of 'd'"
            self._timestamps = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'timestamps' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._timestamps = array.array('d', value)
