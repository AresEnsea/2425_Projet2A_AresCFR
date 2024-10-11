# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2_serial_msgs:msg/SerialMapping.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'serial_mappings'
# Member 'direction'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SerialMapping(type):
    """Metaclass of message 'SerialMapping'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'SERIALTOROS2': 0,
        'ROS2TOSERIAL': 1,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ros2_serial_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2_serial_msgs.msg.SerialMapping')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__serial_mapping
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__serial_mapping
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__serial_mapping
            cls._TYPE_SUPPORT = module.type_support_msg__msg__serial_mapping
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__serial_mapping

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SERIALTOROS2': cls.__constants['SERIALTOROS2'],
            'ROS2TOSERIAL': cls.__constants['ROS2TOSERIAL'],
        }

    @property
    def SERIALTOROS2(self):
        """Message constant 'SERIALTOROS2'."""
        return Metaclass_SerialMapping.__constants['SERIALTOROS2']

    @property
    def ROS2TOSERIAL(self):
        """Message constant 'ROS2TOSERIAL'."""
        return Metaclass_SerialMapping.__constants['ROS2TOSERIAL']


class SerialMapping(metaclass=Metaclass_SerialMapping):
    """
    Message class 'SerialMapping'.

    Constants:
      SERIALTOROS2
      ROS2TOSERIAL
    """

    __slots__ = [
        '_topic_names',
        '_serial_mappings',
        '_types',
        '_direction',
    ]

    _fields_and_field_types = {
        'topic_names': 'sequence<string>',
        'serial_mappings': 'sequence<uint64>',
        'types': 'sequence<string>',
        'direction': 'sequence<uint8>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint64')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.topic_names = kwargs.get('topic_names', [])
        self.serial_mappings = array.array('Q', kwargs.get('serial_mappings', []))
        self.types = kwargs.get('types', [])
        self.direction = array.array('B', kwargs.get('direction', []))

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
        if self.topic_names != other.topic_names:
            return False
        if self.serial_mappings != other.serial_mappings:
            return False
        if self.types != other.types:
            return False
        if self.direction != other.direction:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def topic_names(self):
        """Message field 'topic_names'."""
        return self._topic_names

    @topic_names.setter
    def topic_names(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'topic_names' field must be a set or sequence and each value of type 'str'"
        self._topic_names = value

    @builtins.property
    def serial_mappings(self):
        """Message field 'serial_mappings'."""
        return self._serial_mappings

    @serial_mappings.setter
    def serial_mappings(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'Q', \
                "The 'serial_mappings' array.array() must have the type code of 'Q'"
            self._serial_mappings = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 18446744073709551616 for val in value)), \
                "The 'serial_mappings' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 18446744073709551615]"
        self._serial_mappings = array.array('Q', value)

    @builtins.property
    def types(self):
        """Message field 'types'."""
        return self._types

    @types.setter
    def types(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'types' field must be a set or sequence and each value of type 'str'"
        self._types = value

    @builtins.property
    def direction(self):
        """Message field 'direction'."""
        return self._direction

    @direction.setter
    def direction(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'direction' array.array() must have the type code of 'B'"
            self._direction = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'direction' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._direction = array.array('B', value)
