# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dora_msgs:msg/Map.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Map(type):
    """Metaclass of message 'Map'."""

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
                'dora_msgs.msg.Map')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__map
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__map
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__map
            cls._TYPE_SUPPORT = module.type_support_msg__msg__map
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__map

            from dora_msgs.msg import Cloud
            if Cloud.__class__._TYPE_SUPPORT is None:
                Cloud.__class__.__import_type_support__()

            from dora_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Map(metaclass=Metaclass_Map):
    """Message class 'Map'."""

    __slots__ = [
        '_offset',
        '_clouds',
    ]

    _fields_and_field_types = {
        'offset': 'dora_msgs/Pose',
        'clouds': 'sequence<dora_msgs/Cloud>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['dora_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['dora_msgs', 'msg'], 'Cloud')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from dora_msgs.msg import Pose
        self.offset = kwargs.get('offset', Pose())
        self.clouds = kwargs.get('clouds', [])

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
        if self.offset != other.offset:
            return False
        if self.clouds != other.clouds:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def offset(self):
        """Message field 'offset'."""
        return self._offset

    @offset.setter
    def offset(self, value):
        if __debug__:
            from dora_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'offset' field must be a sub message of type 'Pose'"
        self._offset = value

    @builtins.property
    def clouds(self):
        """Message field 'clouds'."""
        return self._clouds

    @clouds.setter
    def clouds(self, value):
        if __debug__:
            from dora_msgs.msg import Cloud
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
                 all(isinstance(v, Cloud) for v in value) and
                 True), \
                "The 'clouds' field must be a set or sequence and each value of type 'Cloud'"
        self._clouds = value
