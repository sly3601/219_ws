# generated from rosidl_generator_py/resource/_idl.py.em
# with input from control_input_msgs:msg/Inputs.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Inputs(type):
    """Metaclass of message 'Inputs'."""

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
            module = import_type_support('control_input_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'control_input_msgs.msg.Inputs')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__inputs
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__inputs
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__inputs
            cls._TYPE_SUPPORT = module.type_support_msg__msg__inputs
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__inputs

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Inputs(metaclass=Metaclass_Inputs):
    """Message class 'Inputs'."""

    __slots__ = [
        '_command',
        '_lx',
        '_ly',
        '_rx',
        '_ry',
    ]

    _fields_and_field_types = {
        'command': 'int32',
        'lx': 'float',
        'ly': 'float',
        'rx': 'float',
        'ry': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.command = kwargs.get('command', int())
        self.lx = kwargs.get('lx', float())
        self.ly = kwargs.get('ly', float())
        self.rx = kwargs.get('rx', float())
        self.ry = kwargs.get('ry', float())

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
        if self.command != other.command:
            return False
        if self.lx != other.lx:
            return False
        if self.ly != other.ly:
            return False
        if self.rx != other.rx:
            return False
        if self.ry != other.ry:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def command(self):
        """Message field 'command'."""
        return self._command

    @command.setter
    def command(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'command' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'command' field must be an integer in [-2147483648, 2147483647]"
        self._command = value

    @builtins.property
    def lx(self):
        """Message field 'lx'."""
        return self._lx

    @lx.setter
    def lx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lx' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'lx' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._lx = value

    @builtins.property
    def ly(self):
        """Message field 'ly'."""
        return self._ly

    @ly.setter
    def ly(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ly' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ly' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ly = value

    @builtins.property
    def rx(self):
        """Message field 'rx'."""
        return self._rx

    @rx.setter
    def rx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rx' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'rx' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._rx = value

    @builtins.property
    def ry(self):
        """Message field 'ry'."""
        return self._ry

    @ry.setter
    def ry(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ry' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ry' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ry = value
