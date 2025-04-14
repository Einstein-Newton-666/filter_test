# generated from rosidl_generator_py/resource/_idl.py.em
# with input from auto_aim_interfaces:msg/Anglesolver.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Anglesolver(type):
    """Metaclass of message 'Anglesolver'."""

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
            module = import_type_support('auto_aim_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'auto_aim_interfaces.msg.Anglesolver')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__anglesolver
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__anglesolver
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__anglesolver
            cls._TYPE_SUPPORT = module.type_support_msg__msg__anglesolver
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__anglesolver

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Anglesolver(metaclass=Metaclass_Anglesolver):
    """Message class 'Anglesolver'."""

    __slots__ = [
        '_abs_angle',
        '_aim_pose_diff',
        '_predict_time',
        '_current_velocity',
        '_last_velocity',
        '_velocity_diff',
    ]

    _fields_and_field_types = {
        'abs_angle': 'float',
        'aim_pose_diff': 'float',
        'predict_time': 'float',
        'current_velocity': 'float',
        'last_velocity': 'float',
        'velocity_diff': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.abs_angle = kwargs.get('abs_angle', float())
        self.aim_pose_diff = kwargs.get('aim_pose_diff', float())
        self.predict_time = kwargs.get('predict_time', float())
        self.current_velocity = kwargs.get('current_velocity', float())
        self.last_velocity = kwargs.get('last_velocity', float())
        self.velocity_diff = kwargs.get('velocity_diff', float())

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
        if self.abs_angle != other.abs_angle:
            return False
        if self.aim_pose_diff != other.aim_pose_diff:
            return False
        if self.predict_time != other.predict_time:
            return False
        if self.current_velocity != other.current_velocity:
            return False
        if self.last_velocity != other.last_velocity:
            return False
        if self.velocity_diff != other.velocity_diff:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def abs_angle(self):
        """Message field 'abs_angle'."""
        return self._abs_angle

    @abs_angle.setter
    def abs_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'abs_angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'abs_angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._abs_angle = value

    @builtins.property
    def aim_pose_diff(self):
        """Message field 'aim_pose_diff'."""
        return self._aim_pose_diff

    @aim_pose_diff.setter
    def aim_pose_diff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aim_pose_diff' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'aim_pose_diff' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._aim_pose_diff = value

    @builtins.property
    def predict_time(self):
        """Message field 'predict_time'."""
        return self._predict_time

    @predict_time.setter
    def predict_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'predict_time' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'predict_time' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._predict_time = value

    @builtins.property
    def current_velocity(self):
        """Message field 'current_velocity'."""
        return self._current_velocity

    @current_velocity.setter
    def current_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'current_velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._current_velocity = value

    @builtins.property
    def last_velocity(self):
        """Message field 'last_velocity'."""
        return self._last_velocity

    @last_velocity.setter
    def last_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'last_velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'last_velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._last_velocity = value

    @builtins.property
    def velocity_diff(self):
        """Message field 'velocity_diff'."""
        return self._velocity_diff

    @velocity_diff.setter
    def velocity_diff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity_diff' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'velocity_diff' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._velocity_diff = value
