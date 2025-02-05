# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2_rt_eval_dep:srv/Vector.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'input_vector'
# Member 'client_id_vector'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Vector_Request(type):
    """Metaclass of message 'Vector_Request'."""

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
            module = import_type_support('ros2_rt_eval_dep')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2_rt_eval_dep.srv.Vector_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__vector__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__vector__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__vector__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__vector__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__vector__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Vector_Request(metaclass=Metaclass_Vector_Request):
    """Message class 'Vector_Request'."""

    __slots__ = [
        '_input_vector',
        '_client_id_vector',
    ]

    _fields_and_field_types = {
        'input_vector': 'sequence<int16>',
        'client_id_vector': 'sequence<int16>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int16')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.input_vector = array.array('h', kwargs.get('input_vector', []))
        self.client_id_vector = array.array('h', kwargs.get('client_id_vector', []))

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
        if self.input_vector != other.input_vector:
            return False
        if self.client_id_vector != other.client_id_vector:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def input_vector(self):
        """Message field 'input_vector'."""
        return self._input_vector

    @input_vector.setter
    def input_vector(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'h', \
                "The 'input_vector' array.array() must have the type code of 'h'"
            self._input_vector = value
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
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'input_vector' field must be a set or sequence and each value of type 'int' and each integer in [-32768, 32767]"
        self._input_vector = array.array('h', value)

    @builtins.property
    def client_id_vector(self):
        """Message field 'client_id_vector'."""
        return self._client_id_vector

    @client_id_vector.setter
    def client_id_vector(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'h', \
                "The 'client_id_vector' array.array() must have the type code of 'h'"
            self._client_id_vector = value
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
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'client_id_vector' field must be a set or sequence and each value of type 'int' and each integer in [-32768, 32767]"
        self._client_id_vector = array.array('h', value)


# Import statements for member types

# Member 'output_vector'
# already imported above
# import array

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Vector_Response(type):
    """Metaclass of message 'Vector_Response'."""

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
            module = import_type_support('ros2_rt_eval_dep')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2_rt_eval_dep.srv.Vector_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__vector__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__vector__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__vector__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__vector__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__vector__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Vector_Response(metaclass=Metaclass_Vector_Response):
    """Message class 'Vector_Response'."""

    __slots__ = [
        '_output_vector',
        '_duration',
        '_t2',
        '_t3',
    ]

    _fields_and_field_types = {
        'output_vector': 'sequence<int16>',
        'duration': 'int64',
        't2': 'int64',
        't3': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int16')),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.output_vector = array.array('h', kwargs.get('output_vector', []))
        self.duration = kwargs.get('duration', int())
        self.t2 = kwargs.get('t2', int())
        self.t3 = kwargs.get('t3', int())

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
        if self.output_vector != other.output_vector:
            return False
        if self.duration != other.duration:
            return False
        if self.t2 != other.t2:
            return False
        if self.t3 != other.t3:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def output_vector(self):
        """Message field 'output_vector'."""
        return self._output_vector

    @output_vector.setter
    def output_vector(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'h', \
                "The 'output_vector' array.array() must have the type code of 'h'"
            self._output_vector = value
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
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'output_vector' field must be a set or sequence and each value of type 'int' and each integer in [-32768, 32767]"
        self._output_vector = array.array('h', value)

    @builtins.property
    def duration(self):
        """Message field 'duration'."""
        return self._duration

    @duration.setter
    def duration(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'duration' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'duration' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._duration = value

    @builtins.property
    def t2(self):
        """Message field 't2'."""
        return self._t2

    @t2.setter
    def t2(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 't2' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 't2' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._t2 = value

    @builtins.property
    def t3(self):
        """Message field 't3'."""
        return self._t3

    @t3.setter
    def t3(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 't3' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 't3' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._t3 = value


class Metaclass_Vector(type):
    """Metaclass of service 'Vector'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ros2_rt_eval_dep')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2_rt_eval_dep.srv.Vector')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__vector

            from ros2_rt_eval_dep.srv import _vector
            if _vector.Metaclass_Vector_Request._TYPE_SUPPORT is None:
                _vector.Metaclass_Vector_Request.__import_type_support__()
            if _vector.Metaclass_Vector_Response._TYPE_SUPPORT is None:
                _vector.Metaclass_Vector_Response.__import_type_support__()


class Vector(metaclass=Metaclass_Vector):
    from ros2_rt_eval_dep.srv._vector import Vector_Request as Request
    from ros2_rt_eval_dep.srv._vector import Vector_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
