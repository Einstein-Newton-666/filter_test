# generated from rosidl_generator_py/resource/_idl.py.em
# with input from filter_test:msg/Result.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Result(type):
    """Metaclass of message 'Result'."""

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
            module = import_type_support('filter_test')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'filter_test.msg.Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__result
            cls._TYPE_SUPPORT = module.type_support_msg__msg__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__result

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

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


class Result(metaclass=Metaclass_Result):
    """Message class 'Result'."""

    __slots__ = [
        '_header',
        '_position',
        '_velocity',
        '_yaw',
        '_v_yaw',
        '_radius_1',
        '_radius_2',
        '_dz',
        '_position_x_diff',
        '_position_y_diff',
        '_position_z1_diff',
        '_position_z2_diff',
        '_position_yaw_diff',
        '_velocity_x_diff',
        '_velocity_y_diff',
        '_velocity_yaw_diff',
        '_r1_diff',
        '_r2_diff',
        '_match_size',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'position': 'geometry_msgs/Point',
        'velocity': 'geometry_msgs/Vector3',
        'yaw': 'double',
        'v_yaw': 'double',
        'radius_1': 'double',
        'radius_2': 'double',
        'dz': 'double',
        'position_x_diff': 'double',
        'position_y_diff': 'double',
        'position_z1_diff': 'double',
        'position_z2_diff': 'double',
        'position_yaw_diff': 'double',
        'velocity_x_diff': 'double',
        'velocity_y_diff': 'double',
        'velocity_yaw_diff': 'double',
        'r1_diff': 'double',
        'r2_diff': 'double',
        'match_size': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from geometry_msgs.msg import Point
        self.position = kwargs.get('position', Point())
        from geometry_msgs.msg import Vector3
        self.velocity = kwargs.get('velocity', Vector3())
        self.yaw = kwargs.get('yaw', float())
        self.v_yaw = kwargs.get('v_yaw', float())
        self.radius_1 = kwargs.get('radius_1', float())
        self.radius_2 = kwargs.get('radius_2', float())
        self.dz = kwargs.get('dz', float())
        self.position_x_diff = kwargs.get('position_x_diff', float())
        self.position_y_diff = kwargs.get('position_y_diff', float())
        self.position_z1_diff = kwargs.get('position_z1_diff', float())
        self.position_z2_diff = kwargs.get('position_z2_diff', float())
        self.position_yaw_diff = kwargs.get('position_yaw_diff', float())
        self.velocity_x_diff = kwargs.get('velocity_x_diff', float())
        self.velocity_y_diff = kwargs.get('velocity_y_diff', float())
        self.velocity_yaw_diff = kwargs.get('velocity_yaw_diff', float())
        self.r1_diff = kwargs.get('r1_diff', float())
        self.r2_diff = kwargs.get('r2_diff', float())
        self.match_size = kwargs.get('match_size', int())

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
        if self.position != other.position:
            return False
        if self.velocity != other.velocity:
            return False
        if self.yaw != other.yaw:
            return False
        if self.v_yaw != other.v_yaw:
            return False
        if self.radius_1 != other.radius_1:
            return False
        if self.radius_2 != other.radius_2:
            return False
        if self.dz != other.dz:
            return False
        if self.position_x_diff != other.position_x_diff:
            return False
        if self.position_y_diff != other.position_y_diff:
            return False
        if self.position_z1_diff != other.position_z1_diff:
            return False
        if self.position_z2_diff != other.position_z2_diff:
            return False
        if self.position_yaw_diff != other.position_yaw_diff:
            return False
        if self.velocity_x_diff != other.velocity_x_diff:
            return False
        if self.velocity_y_diff != other.velocity_y_diff:
            return False
        if self.velocity_yaw_diff != other.velocity_yaw_diff:
            return False
        if self.r1_diff != other.r1_diff:
            return False
        if self.r2_diff != other.r2_diff:
            return False
        if self.match_size != other.match_size:
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
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'position' field must be a sub message of type 'Point'"
        self._position = value

    @builtins.property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'velocity' field must be a sub message of type 'Vector3'"
        self._velocity = value

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
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yaw = value

    @builtins.property
    def v_yaw(self):
        """Message field 'v_yaw'."""
        return self._v_yaw

    @v_yaw.setter
    def v_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'v_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'v_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._v_yaw = value

    @builtins.property
    def radius_1(self):
        """Message field 'radius_1'."""
        return self._radius_1

    @radius_1.setter
    def radius_1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'radius_1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'radius_1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._radius_1 = value

    @builtins.property
    def radius_2(self):
        """Message field 'radius_2'."""
        return self._radius_2

    @radius_2.setter
    def radius_2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'radius_2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'radius_2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._radius_2 = value

    @builtins.property
    def dz(self):
        """Message field 'dz'."""
        return self._dz

    @dz.setter
    def dz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dz' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'dz' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._dz = value

    @builtins.property
    def position_x_diff(self):
        """Message field 'position_x_diff'."""
        return self._position_x_diff

    @position_x_diff.setter
    def position_x_diff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'position_x_diff' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'position_x_diff' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._position_x_diff = value

    @builtins.property
    def position_y_diff(self):
        """Message field 'position_y_diff'."""
        return self._position_y_diff

    @position_y_diff.setter
    def position_y_diff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'position_y_diff' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'position_y_diff' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._position_y_diff = value

    @builtins.property
    def position_z1_diff(self):
        """Message field 'position_z1_diff'."""
        return self._position_z1_diff

    @position_z1_diff.setter
    def position_z1_diff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'position_z1_diff' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'position_z1_diff' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._position_z1_diff = value

    @builtins.property
    def position_z2_diff(self):
        """Message field 'position_z2_diff'."""
        return self._position_z2_diff

    @position_z2_diff.setter
    def position_z2_diff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'position_z2_diff' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'position_z2_diff' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._position_z2_diff = value

    @builtins.property
    def position_yaw_diff(self):
        """Message field 'position_yaw_diff'."""
        return self._position_yaw_diff

    @position_yaw_diff.setter
    def position_yaw_diff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'position_yaw_diff' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'position_yaw_diff' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._position_yaw_diff = value

    @builtins.property
    def velocity_x_diff(self):
        """Message field 'velocity_x_diff'."""
        return self._velocity_x_diff

    @velocity_x_diff.setter
    def velocity_x_diff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity_x_diff' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'velocity_x_diff' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._velocity_x_diff = value

    @builtins.property
    def velocity_y_diff(self):
        """Message field 'velocity_y_diff'."""
        return self._velocity_y_diff

    @velocity_y_diff.setter
    def velocity_y_diff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity_y_diff' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'velocity_y_diff' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._velocity_y_diff = value

    @builtins.property
    def velocity_yaw_diff(self):
        """Message field 'velocity_yaw_diff'."""
        return self._velocity_yaw_diff

    @velocity_yaw_diff.setter
    def velocity_yaw_diff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity_yaw_diff' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'velocity_yaw_diff' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._velocity_yaw_diff = value

    @builtins.property
    def r1_diff(self):
        """Message field 'r1_diff'."""
        return self._r1_diff

    @r1_diff.setter
    def r1_diff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r1_diff' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'r1_diff' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._r1_diff = value

    @builtins.property
    def r2_diff(self):
        """Message field 'r2_diff'."""
        return self._r2_diff

    @r2_diff.setter
    def r2_diff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r2_diff' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'r2_diff' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._r2_diff = value

    @builtins.property
    def match_size(self):
        """Message field 'match_size'."""
        return self._match_size

    @match_size.setter
    def match_size(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'match_size' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'match_size' field must be an integer in [-2147483648, 2147483647]"
        self._match_size = value
