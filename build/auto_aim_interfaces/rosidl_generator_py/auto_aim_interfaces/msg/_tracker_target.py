# generated from rosidl_generator_py/resource/_idl.py.em
# with input from auto_aim_interfaces:msg/TrackerTarget.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TrackerTarget(type):
    """Metaclass of message 'TrackerTarget'."""

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
                'auto_aim_interfaces.msg.TrackerTarget')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__tracker_target
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__tracker_target
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__tracker_target
            cls._TYPE_SUPPORT = module.type_support_msg__msg__tracker_target
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__tracker_target

            from auto_aim_interfaces.msg import ArmorInfo
            if ArmorInfo.__class__._TYPE_SUPPORT is None:
                ArmorInfo.__class__.__import_type_support__()

            from auto_aim_interfaces.msg import EnemyInfo
            if EnemyInfo.__class__._TYPE_SUPPORT is None:
                EnemyInfo.__class__.__import_type_support__()

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


class TrackerTarget(metaclass=Metaclass_TrackerTarget):
    """Message class 'TrackerTarget'."""

    __slots__ = [
        '_header',
        '_id',
        '_armors_num',
        '_armors',
        '_enemy',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'id': 'string',
        'armors_num': 'int32',
        'armors': 'auto_aim_interfaces/ArmorInfo[2]',
        'enemy': 'auto_aim_interfaces/EnemyInfo',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['auto_aim_interfaces', 'msg'], 'ArmorInfo'), 2),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['auto_aim_interfaces', 'msg'], 'EnemyInfo'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.id = kwargs.get('id', str())
        self.armors_num = kwargs.get('armors_num', int())
        from auto_aim_interfaces.msg import ArmorInfo
        self.armors = kwargs.get(
            'armors',
            [ArmorInfo() for x in range(2)]
        )
        from auto_aim_interfaces.msg import EnemyInfo
        self.enemy = kwargs.get('enemy', EnemyInfo())

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
        if self.id != other.id:
            return False
        if self.armors_num != other.armors_num:
            return False
        if self.armors != other.armors:
            return False
        if self.enemy != other.enemy:
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

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'id' field must be of type 'str'"
        self._id = value

    @builtins.property
    def armors_num(self):
        """Message field 'armors_num'."""
        return self._armors_num

    @armors_num.setter
    def armors_num(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'armors_num' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'armors_num' field must be an integer in [-2147483648, 2147483647]"
        self._armors_num = value

    @builtins.property
    def armors(self):
        """Message field 'armors'."""
        return self._armors

    @armors.setter
    def armors(self, value):
        if __debug__:
            from auto_aim_interfaces.msg import ArmorInfo
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
                 len(value) == 2 and
                 all(isinstance(v, ArmorInfo) for v in value) and
                 True), \
                "The 'armors' field must be a set or sequence with length 2 and each value of type 'ArmorInfo'"
        self._armors = value

    @builtins.property
    def enemy(self):
        """Message field 'enemy'."""
        return self._enemy

    @enemy.setter
    def enemy(self, value):
        if __debug__:
            from auto_aim_interfaces.msg import EnemyInfo
            assert \
                isinstance(value, EnemyInfo), \
                "The 'enemy' field must be a sub message of type 'EnemyInfo'"
        self._enemy = value
