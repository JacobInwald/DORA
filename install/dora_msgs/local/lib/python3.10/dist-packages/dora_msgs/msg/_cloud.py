# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dora_msgs:msg/Cloud.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Cloud(type):
    """Metaclass of message 'Cloud'."""

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
            module = import_type_support('dora_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dora_msgs.msg.Cloud')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__cloud
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__cloud
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__cloud
            cls._TYPE_SUPPORT = module.type_support_msg__msg__cloud
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__cloud

            from dora_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

            from sensor_msgs.msg import LaserScan
            if LaserScan.__class__._TYPE_SUPPORT is None:
                LaserScan.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Cloud(metaclass=Metaclass_Cloud):
    """Message class 'Cloud'."""

    __slots__ = [
        '_position',
        '_scan',
        '_max_range',
    ]

    _fields_and_field_types = {
        'position': 'dora_msgs/Pose',
        'scan': 'sensor_msgs/LaserScan',
        'max_range': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['dora_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'LaserScan'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from dora_msgs.msg import Pose
        self.position = kwargs.get('position', Pose())
        from sensor_msgs.msg import LaserScan
        self.scan = kwargs.get('scan', LaserScan())
        self.max_range = kwargs.get('max_range', float())

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
        if self.position != other.position:
            return False
        if self.scan != other.scan:
            return False
        if self.max_range != other.max_range:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if __debug__:
            from dora_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'position' field must be a sub message of type 'Pose'"
        self._position = value

    @builtins.property
    def scan(self):
        """Message field 'scan'."""
        return self._scan

    @scan.setter
    def scan(self, value):
        if __debug__:
            from sensor_msgs.msg import LaserScan
            assert \
                isinstance(value, LaserScan), \
                "The 'scan' field must be a sub message of type 'LaserScan'"
        self._scan = value

    @builtins.property
    def max_range(self):
        """Message field 'max_range'."""
        return self._max_range

    @max_range.setter
    def max_range(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_range' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'max_range' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._max_range = value
