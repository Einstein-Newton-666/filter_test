# generated from rosidl_generator_py/resource/_idl.py.em
# with input from auto_aim_interfaces:msg/RecieveData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RecieveData(type):
    """Metaclass of message 'RecieveData'."""

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
                'auto_aim_interfaces.msg.RecieveData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__recieve_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__recieve_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__recieve_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__recieve_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__recieve_data

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


class RecieveData(metaclass=Metaclass_RecieveData):
    """Message class 'RecieveData'."""

    __slots__ = [
        '_header',
        '_pitch',
        '_yaw',
        '_shoot_speed',
        '_current_color',
        '_big_yaw',
        '_enemies_blood_0',
        '_enemies_blood_1',
        '_enemies_blood_2',
        '_enemies_blood_3',
        '_enemies_blood_4',
        '_enemies_blood_5',
        '_enemies_outpost',
        '_another_priority',
        '_game_progress',
        '_game_type',
        '_if_attack_engineer',
        '_mode',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'pitch': 'float',
        'yaw': 'float',
        'shoot_speed': 'float',
        'current_color': 'int32',
        'big_yaw': 'float',
        'enemies_blood_0': 'int32',
        'enemies_blood_1': 'int32',
        'enemies_blood_2': 'int32',
        'enemies_blood_3': 'int32',
        'enemies_blood_4': 'int32',
        'enemies_blood_5': 'int32',
        'enemies_outpost': 'int32',
        'another_priority': 'int32',
        'game_progress': 'int32',
        'game_type': 'int32',
        'if_attack_engineer': 'int32',
        'mode': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.pitch = kwargs.get('pitch', float())
        self.yaw = kwargs.get('yaw', float())
        self.shoot_speed = kwargs.get('shoot_speed', float())
        self.current_color = kwargs.get('current_color', int())
        self.big_yaw = kwargs.get('big_yaw', float())
        self.enemies_blood_0 = kwargs.get('enemies_blood_0', int())
        self.enemies_blood_1 = kwargs.get('enemies_blood_1', int())
        self.enemies_blood_2 = kwargs.get('enemies_blood_2', int())
        self.enemies_blood_3 = kwargs.get('enemies_blood_3', int())
        self.enemies_blood_4 = kwargs.get('enemies_blood_4', int())
        self.enemies_blood_5 = kwargs.get('enemies_blood_5', int())
        self.enemies_outpost = kwargs.get('enemies_outpost', int())
        self.another_priority = kwargs.get('another_priority', int())
        self.game_progress = kwargs.get('game_progress', int())
        self.game_type = kwargs.get('game_type', int())
        self.if_attack_engineer = kwargs.get('if_attack_engineer', int())
        self.mode = kwargs.get('mode', int())

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
        if self.pitch != other.pitch:
            return False
        if self.yaw != other.yaw:
            return False
        if self.shoot_speed != other.shoot_speed:
            return False
        if self.current_color != other.current_color:
            return False
        if self.big_yaw != other.big_yaw:
            return False
        if self.enemies_blood_0 != other.enemies_blood_0:
            return False
        if self.enemies_blood_1 != other.enemies_blood_1:
            return False
        if self.enemies_blood_2 != other.enemies_blood_2:
            return False
        if self.enemies_blood_3 != other.enemies_blood_3:
            return False
        if self.enemies_blood_4 != other.enemies_blood_4:
            return False
        if self.enemies_blood_5 != other.enemies_blood_5:
            return False
        if self.enemies_outpost != other.enemies_outpost:
            return False
        if self.another_priority != other.another_priority:
            return False
        if self.game_progress != other.game_progress:
            return False
        if self.game_type != other.game_type:
            return False
        if self.if_attack_engineer != other.if_attack_engineer:
            return False
        if self.mode != other.mode:
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
    def pitch(self):
        """Message field 'pitch'."""
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch = value

    @builtins.property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw = value

    @builtins.property
    def shoot_speed(self):
        """Message field 'shoot_speed'."""
        return self._shoot_speed

    @shoot_speed.setter
    def shoot_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'shoot_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'shoot_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._shoot_speed = value

    @builtins.property
    def current_color(self):
        """Message field 'current_color'."""
        return self._current_color

    @current_color.setter
    def current_color(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_color' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'current_color' field must be an integer in [-2147483648, 2147483647]"
        self._current_color = value

    @builtins.property
    def big_yaw(self):
        """Message field 'big_yaw'."""
        return self._big_yaw

    @big_yaw.setter
    def big_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'big_yaw' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'big_yaw' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._big_yaw = value

    @builtins.property
    def enemies_blood_0(self):
        """Message field 'enemies_blood_0'."""
        return self._enemies_blood_0

    @enemies_blood_0.setter
    def enemies_blood_0(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enemies_blood_0' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'enemies_blood_0' field must be an integer in [-2147483648, 2147483647]"
        self._enemies_blood_0 = value

    @builtins.property
    def enemies_blood_1(self):
        """Message field 'enemies_blood_1'."""
        return self._enemies_blood_1

    @enemies_blood_1.setter
    def enemies_blood_1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enemies_blood_1' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'enemies_blood_1' field must be an integer in [-2147483648, 2147483647]"
        self._enemies_blood_1 = value

    @builtins.property
    def enemies_blood_2(self):
        """Message field 'enemies_blood_2'."""
        return self._enemies_blood_2

    @enemies_blood_2.setter
    def enemies_blood_2(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enemies_blood_2' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'enemies_blood_2' field must be an integer in [-2147483648, 2147483647]"
        self._enemies_blood_2 = value

    @builtins.property
    def enemies_blood_3(self):
        """Message field 'enemies_blood_3'."""
        return self._enemies_blood_3

    @enemies_blood_3.setter
    def enemies_blood_3(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enemies_blood_3' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'enemies_blood_3' field must be an integer in [-2147483648, 2147483647]"
        self._enemies_blood_3 = value

    @builtins.property
    def enemies_blood_4(self):
        """Message field 'enemies_blood_4'."""
        return self._enemies_blood_4

    @enemies_blood_4.setter
    def enemies_blood_4(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enemies_blood_4' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'enemies_blood_4' field must be an integer in [-2147483648, 2147483647]"
        self._enemies_blood_4 = value

    @builtins.property
    def enemies_blood_5(self):
        """Message field 'enemies_blood_5'."""
        return self._enemies_blood_5

    @enemies_blood_5.setter
    def enemies_blood_5(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enemies_blood_5' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'enemies_blood_5' field must be an integer in [-2147483648, 2147483647]"
        self._enemies_blood_5 = value

    @builtins.property
    def enemies_outpost(self):
        """Message field 'enemies_outpost'."""
        return self._enemies_outpost

    @enemies_outpost.setter
    def enemies_outpost(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enemies_outpost' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'enemies_outpost' field must be an integer in [-2147483648, 2147483647]"
        self._enemies_outpost = value

    @builtins.property
    def another_priority(self):
        """Message field 'another_priority'."""
        return self._another_priority

    @another_priority.setter
    def another_priority(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'another_priority' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'another_priority' field must be an integer in [-2147483648, 2147483647]"
        self._another_priority = value

    @builtins.property
    def game_progress(self):
        """Message field 'game_progress'."""
        return self._game_progress

    @game_progress.setter
    def game_progress(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'game_progress' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'game_progress' field must be an integer in [-2147483648, 2147483647]"
        self._game_progress = value

    @builtins.property
    def game_type(self):
        """Message field 'game_type'."""
        return self._game_type

    @game_type.setter
    def game_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'game_type' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'game_type' field must be an integer in [-2147483648, 2147483647]"
        self._game_type = value

    @builtins.property
    def if_attack_engineer(self):
        """Message field 'if_attack_engineer'."""
        return self._if_attack_engineer

    @if_attack_engineer.setter
    def if_attack_engineer(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'if_attack_engineer' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'if_attack_engineer' field must be an integer in [-2147483648, 2147483647]"
        self._if_attack_engineer = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'mode' field must be an integer in [-2147483648, 2147483647]"
        self._mode = value
