# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mundus_mir_msgs:msg/BatteryStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BatteryStatus(type):
    """Metaclass of message 'BatteryStatus'."""

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
                'mundus_mir_msgs.msg.BatteryStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__battery_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__battery_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__battery_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__battery_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__battery_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BatteryStatus(metaclass=Metaclass_BatteryStatus):
    """Message class 'BatteryStatus'."""

    __slots__ = [
        '_total_voltage',
        '_cell_1_voltage',
        '_cell_2_voltage',
        '_cell_3_voltage',
        '_cell_4_voltage',
        '_average_temperature',
        '_cell_1_temperature',
        '_cell_2_temperature',
        '_cell_3_temperature',
        '_cell_4_temperature',
        '_initialization',
        '_error_status',
        '_current',
        '_average_current',
        '_state_of_charge',
        '_remaining_capacity',
        '_full_charge_capacity',
        '_runtime_to_empty',
        '_average_time_to_empty',
        '_average_time_to_full',
        '_charging_current',
        '_charging_voltage',
        '_cycle_count',
        '_design_capacity',
        '_manufacturer_name',
        '_device_name',
        '_device_chemistry',
        '_calculated_state_of_charge',
    ]

    _fields_and_field_types = {
        'total_voltage': 'float',
        'cell_1_voltage': 'float',
        'cell_2_voltage': 'float',
        'cell_3_voltage': 'float',
        'cell_4_voltage': 'float',
        'average_temperature': 'float',
        'cell_1_temperature': 'float',
        'cell_2_temperature': 'float',
        'cell_3_temperature': 'float',
        'cell_4_temperature': 'float',
        'initialization': 'boolean',
        'error_status': 'string',
        'current': 'float',
        'average_current': 'float',
        'state_of_charge': 'float',
        'remaining_capacity': 'float',
        'full_charge_capacity': 'float',
        'runtime_to_empty': 'int32',
        'average_time_to_empty': 'int32',
        'average_time_to_full': 'int32',
        'charging_current': 'float',
        'charging_voltage': 'float',
        'cycle_count': 'int32',
        'design_capacity': 'float',
        'manufacturer_name': 'string',
        'device_name': 'string',
        'device_chemistry': 'string',
        'calculated_state_of_charge': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.total_voltage = kwargs.get('total_voltage', float())
        self.cell_1_voltage = kwargs.get('cell_1_voltage', float())
        self.cell_2_voltage = kwargs.get('cell_2_voltage', float())
        self.cell_3_voltage = kwargs.get('cell_3_voltage', float())
        self.cell_4_voltage = kwargs.get('cell_4_voltage', float())
        self.average_temperature = kwargs.get('average_temperature', float())
        self.cell_1_temperature = kwargs.get('cell_1_temperature', float())
        self.cell_2_temperature = kwargs.get('cell_2_temperature', float())
        self.cell_3_temperature = kwargs.get('cell_3_temperature', float())
        self.cell_4_temperature = kwargs.get('cell_4_temperature', float())
        self.initialization = kwargs.get('initialization', bool())
        self.error_status = kwargs.get('error_status', str())
        self.current = kwargs.get('current', float())
        self.average_current = kwargs.get('average_current', float())
        self.state_of_charge = kwargs.get('state_of_charge', float())
        self.remaining_capacity = kwargs.get('remaining_capacity', float())
        self.full_charge_capacity = kwargs.get('full_charge_capacity', float())
        self.runtime_to_empty = kwargs.get('runtime_to_empty', int())
        self.average_time_to_empty = kwargs.get('average_time_to_empty', int())
        self.average_time_to_full = kwargs.get('average_time_to_full', int())
        self.charging_current = kwargs.get('charging_current', float())
        self.charging_voltage = kwargs.get('charging_voltage', float())
        self.cycle_count = kwargs.get('cycle_count', int())
        self.design_capacity = kwargs.get('design_capacity', float())
        self.manufacturer_name = kwargs.get('manufacturer_name', str())
        self.device_name = kwargs.get('device_name', str())
        self.device_chemistry = kwargs.get('device_chemistry', str())
        self.calculated_state_of_charge = kwargs.get('calculated_state_of_charge', float())

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
        if self.total_voltage != other.total_voltage:
            return False
        if self.cell_1_voltage != other.cell_1_voltage:
            return False
        if self.cell_2_voltage != other.cell_2_voltage:
            return False
        if self.cell_3_voltage != other.cell_3_voltage:
            return False
        if self.cell_4_voltage != other.cell_4_voltage:
            return False
        if self.average_temperature != other.average_temperature:
            return False
        if self.cell_1_temperature != other.cell_1_temperature:
            return False
        if self.cell_2_temperature != other.cell_2_temperature:
            return False
        if self.cell_3_temperature != other.cell_3_temperature:
            return False
        if self.cell_4_temperature != other.cell_4_temperature:
            return False
        if self.initialization != other.initialization:
            return False
        if self.error_status != other.error_status:
            return False
        if self.current != other.current:
            return False
        if self.average_current != other.average_current:
            return False
        if self.state_of_charge != other.state_of_charge:
            return False
        if self.remaining_capacity != other.remaining_capacity:
            return False
        if self.full_charge_capacity != other.full_charge_capacity:
            return False
        if self.runtime_to_empty != other.runtime_to_empty:
            return False
        if self.average_time_to_empty != other.average_time_to_empty:
            return False
        if self.average_time_to_full != other.average_time_to_full:
            return False
        if self.charging_current != other.charging_current:
            return False
        if self.charging_voltage != other.charging_voltage:
            return False
        if self.cycle_count != other.cycle_count:
            return False
        if self.design_capacity != other.design_capacity:
            return False
        if self.manufacturer_name != other.manufacturer_name:
            return False
        if self.device_name != other.device_name:
            return False
        if self.device_chemistry != other.device_chemistry:
            return False
        if self.calculated_state_of_charge != other.calculated_state_of_charge:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def total_voltage(self):
        """Message field 'total_voltage'."""
        return self._total_voltage

    @total_voltage.setter
    def total_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'total_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._total_voltage = value

    @builtins.property
    def cell_1_voltage(self):
        """Message field 'cell_1_voltage'."""
        return self._cell_1_voltage

    @cell_1_voltage.setter
    def cell_1_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cell_1_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cell_1_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cell_1_voltage = value

    @builtins.property
    def cell_2_voltage(self):
        """Message field 'cell_2_voltage'."""
        return self._cell_2_voltage

    @cell_2_voltage.setter
    def cell_2_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cell_2_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cell_2_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cell_2_voltage = value

    @builtins.property
    def cell_3_voltage(self):
        """Message field 'cell_3_voltage'."""
        return self._cell_3_voltage

    @cell_3_voltage.setter
    def cell_3_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cell_3_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cell_3_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cell_3_voltage = value

    @builtins.property
    def cell_4_voltage(self):
        """Message field 'cell_4_voltage'."""
        return self._cell_4_voltage

    @cell_4_voltage.setter
    def cell_4_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cell_4_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cell_4_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cell_4_voltage = value

    @builtins.property
    def average_temperature(self):
        """Message field 'average_temperature'."""
        return self._average_temperature

    @average_temperature.setter
    def average_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'average_temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'average_temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._average_temperature = value

    @builtins.property
    def cell_1_temperature(self):
        """Message field 'cell_1_temperature'."""
        return self._cell_1_temperature

    @cell_1_temperature.setter
    def cell_1_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cell_1_temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cell_1_temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cell_1_temperature = value

    @builtins.property
    def cell_2_temperature(self):
        """Message field 'cell_2_temperature'."""
        return self._cell_2_temperature

    @cell_2_temperature.setter
    def cell_2_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cell_2_temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cell_2_temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cell_2_temperature = value

    @builtins.property
    def cell_3_temperature(self):
        """Message field 'cell_3_temperature'."""
        return self._cell_3_temperature

    @cell_3_temperature.setter
    def cell_3_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cell_3_temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cell_3_temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cell_3_temperature = value

    @builtins.property
    def cell_4_temperature(self):
        """Message field 'cell_4_temperature'."""
        return self._cell_4_temperature

    @cell_4_temperature.setter
    def cell_4_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cell_4_temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cell_4_temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cell_4_temperature = value

    @builtins.property
    def initialization(self):
        """Message field 'initialization'."""
        return self._initialization

    @initialization.setter
    def initialization(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'initialization' field must be of type 'bool'"
        self._initialization = value

    @builtins.property
    def error_status(self):
        """Message field 'error_status'."""
        return self._error_status

    @error_status.setter
    def error_status(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'error_status' field must be of type 'str'"
        self._error_status = value

    @builtins.property
    def current(self):
        """Message field 'current'."""
        return self._current

    @current.setter
    def current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'current' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._current = value

    @builtins.property
    def average_current(self):
        """Message field 'average_current'."""
        return self._average_current

    @average_current.setter
    def average_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'average_current' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'average_current' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._average_current = value

    @builtins.property
    def state_of_charge(self):
        """Message field 'state_of_charge'."""
        return self._state_of_charge

    @state_of_charge.setter
    def state_of_charge(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'state_of_charge' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'state_of_charge' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._state_of_charge = value

    @builtins.property
    def remaining_capacity(self):
        """Message field 'remaining_capacity'."""
        return self._remaining_capacity

    @remaining_capacity.setter
    def remaining_capacity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'remaining_capacity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'remaining_capacity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._remaining_capacity = value

    @builtins.property
    def full_charge_capacity(self):
        """Message field 'full_charge_capacity'."""
        return self._full_charge_capacity

    @full_charge_capacity.setter
    def full_charge_capacity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'full_charge_capacity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'full_charge_capacity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._full_charge_capacity = value

    @builtins.property
    def runtime_to_empty(self):
        """Message field 'runtime_to_empty'."""
        return self._runtime_to_empty

    @runtime_to_empty.setter
    def runtime_to_empty(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'runtime_to_empty' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'runtime_to_empty' field must be an integer in [-2147483648, 2147483647]"
        self._runtime_to_empty = value

    @builtins.property
    def average_time_to_empty(self):
        """Message field 'average_time_to_empty'."""
        return self._average_time_to_empty

    @average_time_to_empty.setter
    def average_time_to_empty(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'average_time_to_empty' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'average_time_to_empty' field must be an integer in [-2147483648, 2147483647]"
        self._average_time_to_empty = value

    @builtins.property
    def average_time_to_full(self):
        """Message field 'average_time_to_full'."""
        return self._average_time_to_full

    @average_time_to_full.setter
    def average_time_to_full(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'average_time_to_full' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'average_time_to_full' field must be an integer in [-2147483648, 2147483647]"
        self._average_time_to_full = value

    @builtins.property
    def charging_current(self):
        """Message field 'charging_current'."""
        return self._charging_current

    @charging_current.setter
    def charging_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'charging_current' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'charging_current' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._charging_current = value

    @builtins.property
    def charging_voltage(self):
        """Message field 'charging_voltage'."""
        return self._charging_voltage

    @charging_voltage.setter
    def charging_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'charging_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'charging_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._charging_voltage = value

    @builtins.property
    def cycle_count(self):
        """Message field 'cycle_count'."""
        return self._cycle_count

    @cycle_count.setter
    def cycle_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cycle_count' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'cycle_count' field must be an integer in [-2147483648, 2147483647]"
        self._cycle_count = value

    @builtins.property
    def design_capacity(self):
        """Message field 'design_capacity'."""
        return self._design_capacity

    @design_capacity.setter
    def design_capacity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'design_capacity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'design_capacity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._design_capacity = value

    @builtins.property
    def manufacturer_name(self):
        """Message field 'manufacturer_name'."""
        return self._manufacturer_name

    @manufacturer_name.setter
    def manufacturer_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'manufacturer_name' field must be of type 'str'"
        self._manufacturer_name = value

    @builtins.property
    def device_name(self):
        """Message field 'device_name'."""
        return self._device_name

    @device_name.setter
    def device_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'device_name' field must be of type 'str'"
        self._device_name = value

    @builtins.property
    def device_chemistry(self):
        """Message field 'device_chemistry'."""
        return self._device_chemistry

    @device_chemistry.setter
    def device_chemistry(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'device_chemistry' field must be of type 'str'"
        self._device_chemistry = value

    @builtins.property
    def calculated_state_of_charge(self):
        """Message field 'calculated_state_of_charge'."""
        return self._calculated_state_of_charge

    @calculated_state_of_charge.setter
    def calculated_state_of_charge(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'calculated_state_of_charge' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'calculated_state_of_charge' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._calculated_state_of_charge = value
