# generated from rosidl_generator_py/resource/_idl.py.em
# with input from auto_aim_interfaces:msg/CameraMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'id_1'
# Member 'id_2'
# Member 'id_3'
# Member 'id_4'
# Member 'speed_x'
# Member 'speed_y'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CameraMsg(type):
    """Metaclass of message 'CameraMsg'."""

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
                'auto_aim_interfaces.msg.CameraMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__camera_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__camera_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__camera_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__camera_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__camera_msg

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CameraMsg(metaclass=Metaclass_CameraMsg):
    """Message class 'CameraMsg'."""

    __slots__ = [
        '_id_1',
        '_enemy_pose_1',
        '_id_2',
        '_enemy_pose_2',
        '_id_3',
        '_enemy_pose_3',
        '_id_4',
        '_enemy_pose_4',
        '_speed_x',
        '_speed_y',
    ]

    _fields_and_field_types = {
        'id_1': 'int32[5]',
        'enemy_pose_1': 'geometry_msgs/PoseStamped[5]',
        'id_2': 'int32[5]',
        'enemy_pose_2': 'geometry_msgs/PoseStamped[5]',
        'id_3': 'int32[5]',
        'enemy_pose_3': 'geometry_msgs/PoseStamped[5]',
        'id_4': 'int32[5]',
        'enemy_pose_4': 'geometry_msgs/PoseStamped[5]',
        'speed_x': 'float[5]',
        'speed_y': 'float[5]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int32'), 5),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'), 5),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int32'), 5),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'), 5),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int32'), 5),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'), 5),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int32'), 5),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'), 5),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 5),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 5),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'id_1' not in kwargs:
            self.id_1 = numpy.zeros(5, dtype=numpy.int32)
        else:
            self.id_1 = numpy.array(kwargs.get('id_1'), dtype=numpy.int32)
            assert self.id_1.shape == (5, )
        from geometry_msgs.msg import PoseStamped
        self.enemy_pose_1 = kwargs.get(
            'enemy_pose_1',
            [PoseStamped() for x in range(5)]
        )
        if 'id_2' not in kwargs:
            self.id_2 = numpy.zeros(5, dtype=numpy.int32)
        else:
            self.id_2 = numpy.array(kwargs.get('id_2'), dtype=numpy.int32)
            assert self.id_2.shape == (5, )
        from geometry_msgs.msg import PoseStamped
        self.enemy_pose_2 = kwargs.get(
            'enemy_pose_2',
            [PoseStamped() for x in range(5)]
        )
        if 'id_3' not in kwargs:
            self.id_3 = numpy.zeros(5, dtype=numpy.int32)
        else:
            self.id_3 = numpy.array(kwargs.get('id_3'), dtype=numpy.int32)
            assert self.id_3.shape == (5, )
        from geometry_msgs.msg import PoseStamped
        self.enemy_pose_3 = kwargs.get(
            'enemy_pose_3',
            [PoseStamped() for x in range(5)]
        )
        if 'id_4' not in kwargs:
            self.id_4 = numpy.zeros(5, dtype=numpy.int32)
        else:
            self.id_4 = numpy.array(kwargs.get('id_4'), dtype=numpy.int32)
            assert self.id_4.shape == (5, )
        from geometry_msgs.msg import PoseStamped
        self.enemy_pose_4 = kwargs.get(
            'enemy_pose_4',
            [PoseStamped() for x in range(5)]
        )
        if 'speed_x' not in kwargs:
            self.speed_x = numpy.zeros(5, dtype=numpy.float32)
        else:
            self.speed_x = numpy.array(kwargs.get('speed_x'), dtype=numpy.float32)
            assert self.speed_x.shape == (5, )
        if 'speed_y' not in kwargs:
            self.speed_y = numpy.zeros(5, dtype=numpy.float32)
        else:
            self.speed_y = numpy.array(kwargs.get('speed_y'), dtype=numpy.float32)
            assert self.speed_y.shape == (5, )

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
        if all(self.id_1 != other.id_1):
            return False
        if self.enemy_pose_1 != other.enemy_pose_1:
            return False
        if all(self.id_2 != other.id_2):
            return False
        if self.enemy_pose_2 != other.enemy_pose_2:
            return False
        if all(self.id_3 != other.id_3):
            return False
        if self.enemy_pose_3 != other.enemy_pose_3:
            return False
        if all(self.id_4 != other.id_4):
            return False
        if self.enemy_pose_4 != other.enemy_pose_4:
            return False
        if all(self.speed_x != other.speed_x):
            return False
        if all(self.speed_y != other.speed_y):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def id_1(self):
        """Message field 'id_1'."""
        return self._id_1

    @id_1.setter
    def id_1(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int32, \
                "The 'id_1' numpy.ndarray() must have the dtype of 'numpy.int32'"
            assert value.size == 5, \
                "The 'id_1' numpy.ndarray() must have a size of 5"
            self._id_1 = value
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
                 len(value) == 5 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'id_1' field must be a set or sequence with length 5 and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._id_1 = numpy.array(value, dtype=numpy.int32)

    @builtins.property
    def enemy_pose_1(self):
        """Message field 'enemy_pose_1'."""
        return self._enemy_pose_1

    @enemy_pose_1.setter
    def enemy_pose_1(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
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
                 len(value) == 5 and
                 all(isinstance(v, PoseStamped) for v in value) and
                 True), \
                "The 'enemy_pose_1' field must be a set or sequence with length 5 and each value of type 'PoseStamped'"
        self._enemy_pose_1 = value

    @builtins.property
    def id_2(self):
        """Message field 'id_2'."""
        return self._id_2

    @id_2.setter
    def id_2(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int32, \
                "The 'id_2' numpy.ndarray() must have the dtype of 'numpy.int32'"
            assert value.size == 5, \
                "The 'id_2' numpy.ndarray() must have a size of 5"
            self._id_2 = value
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
                 len(value) == 5 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'id_2' field must be a set or sequence with length 5 and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._id_2 = numpy.array(value, dtype=numpy.int32)

    @builtins.property
    def enemy_pose_2(self):
        """Message field 'enemy_pose_2'."""
        return self._enemy_pose_2

    @enemy_pose_2.setter
    def enemy_pose_2(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
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
                 len(value) == 5 and
                 all(isinstance(v, PoseStamped) for v in value) and
                 True), \
                "The 'enemy_pose_2' field must be a set or sequence with length 5 and each value of type 'PoseStamped'"
        self._enemy_pose_2 = value

    @builtins.property
    def id_3(self):
        """Message field 'id_3'."""
        return self._id_3

    @id_3.setter
    def id_3(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int32, \
                "The 'id_3' numpy.ndarray() must have the dtype of 'numpy.int32'"
            assert value.size == 5, \
                "The 'id_3' numpy.ndarray() must have a size of 5"
            self._id_3 = value
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
                 len(value) == 5 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'id_3' field must be a set or sequence with length 5 and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._id_3 = numpy.array(value, dtype=numpy.int32)

    @builtins.property
    def enemy_pose_3(self):
        """Message field 'enemy_pose_3'."""
        return self._enemy_pose_3

    @enemy_pose_3.setter
    def enemy_pose_3(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
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
                 len(value) == 5 and
                 all(isinstance(v, PoseStamped) for v in value) and
                 True), \
                "The 'enemy_pose_3' field must be a set or sequence with length 5 and each value of type 'PoseStamped'"
        self._enemy_pose_3 = value

    @builtins.property
    def id_4(self):
        """Message field 'id_4'."""
        return self._id_4

    @id_4.setter
    def id_4(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int32, \
                "The 'id_4' numpy.ndarray() must have the dtype of 'numpy.int32'"
            assert value.size == 5, \
                "The 'id_4' numpy.ndarray() must have a size of 5"
            self._id_4 = value
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
                 len(value) == 5 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'id_4' field must be a set or sequence with length 5 and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._id_4 = numpy.array(value, dtype=numpy.int32)

    @builtins.property
    def enemy_pose_4(self):
        """Message field 'enemy_pose_4'."""
        return self._enemy_pose_4

    @enemy_pose_4.setter
    def enemy_pose_4(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
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
                 len(value) == 5 and
                 all(isinstance(v, PoseStamped) for v in value) and
                 True), \
                "The 'enemy_pose_4' field must be a set or sequence with length 5 and each value of type 'PoseStamped'"
        self._enemy_pose_4 = value

    @builtins.property
    def speed_x(self):
        """Message field 'speed_x'."""
        return self._speed_x

    @speed_x.setter
    def speed_x(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'speed_x' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 5, \
                "The 'speed_x' numpy.ndarray() must have a size of 5"
            self._speed_x = value
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
                 len(value) == 5 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'speed_x' field must be a set or sequence with length 5 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._speed_x = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def speed_y(self):
        """Message field 'speed_y'."""
        return self._speed_y

    @speed_y.setter
    def speed_y(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'speed_y' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 5, \
                "The 'speed_y' numpy.ndarray() must have a size of 5"
            self._speed_y = value
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
                 len(value) == 5 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'speed_y' field must be a set or sequence with length 5 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._speed_y = numpy.array(value, dtype=numpy.float32)
