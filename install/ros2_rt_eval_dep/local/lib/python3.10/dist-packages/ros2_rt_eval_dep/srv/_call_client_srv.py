# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2_rt_eval_dep:srv/CallClientSrv.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CallClientSrv_Request(type):
    """Metaclass of message 'CallClientSrv_Request'."""

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
                'ros2_rt_eval_dep.srv.CallClientSrv_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__call_client_srv__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__call_client_srv__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__call_client_srv__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__call_client_srv__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__call_client_srv__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CallClientSrv_Request(metaclass=Metaclass_CallClientSrv_Request):
    """Message class 'CallClientSrv_Request'."""

    __slots__ = [
        '_num_calls',
    ]

    _fields_and_field_types = {
        'num_calls': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.num_calls = kwargs.get('num_calls', int())

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
        if self.num_calls != other.num_calls:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def num_calls(self):
        """Message field 'num_calls'."""
        return self._num_calls

    @num_calls.setter
    def num_calls(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_calls' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'num_calls' field must be an integer in [-2147483648, 2147483647]"
        self._num_calls = value


# Import statements for member types

# Member 'latencies'
import array  # noqa: E402, I100

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CallClientSrv_Response(type):
    """Metaclass of message 'CallClientSrv_Response'."""

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
                'ros2_rt_eval_dep.srv.CallClientSrv_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__call_client_srv__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__call_client_srv__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__call_client_srv__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__call_client_srv__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__call_client_srv__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CallClientSrv_Response(metaclass=Metaclass_CallClientSrv_Response):
    """Message class 'CallClientSrv_Response'."""

    __slots__ = [
        '_latencies',
    ]

    _fields_and_field_types = {
        'latencies': 'sequence<int64>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int64')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.latencies = array.array('q', kwargs.get('latencies', []))

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
        if self.latencies != other.latencies:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def latencies(self):
        """Message field 'latencies'."""
        return self._latencies

    @latencies.setter
    def latencies(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'q', \
                "The 'latencies' array.array() must have the type code of 'q'"
            self._latencies = value
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
                 all(val >= -9223372036854775808 and val < 9223372036854775808 for val in value)), \
                "The 'latencies' field must be a set or sequence and each value of type 'int' and each integer in [-9223372036854775808, 9223372036854775807]"
        self._latencies = array.array('q', value)


class Metaclass_CallClientSrv(type):
    """Metaclass of service 'CallClientSrv'."""

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
                'ros2_rt_eval_dep.srv.CallClientSrv')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__call_client_srv

            from ros2_rt_eval_dep.srv import _call_client_srv
            if _call_client_srv.Metaclass_CallClientSrv_Request._TYPE_SUPPORT is None:
                _call_client_srv.Metaclass_CallClientSrv_Request.__import_type_support__()
            if _call_client_srv.Metaclass_CallClientSrv_Response._TYPE_SUPPORT is None:
                _call_client_srv.Metaclass_CallClientSrv_Response.__import_type_support__()


class CallClientSrv(metaclass=Metaclass_CallClientSrv):
    from ros2_rt_eval_dep.srv._call_client_srv import CallClientSrv_Request as Request
    from ros2_rt_eval_dep.srv._call_client_srv import CallClientSrv_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
