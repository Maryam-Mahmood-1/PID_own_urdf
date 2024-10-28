# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_interfaces:srv/SetJointStates.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetJointStates_Request(type):
    """Metaclass of message 'SetJointStates_Request'."""

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
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.srv.SetJointStates_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_joint_states__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_joint_states__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_joint_states__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_joint_states__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_joint_states__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetJointStates_Request(metaclass=Metaclass_SetJointStates_Request):
    """Message class 'SetJointStates_Request'."""

    __slots__ = [
        '_rq1',
        '_rq2',
        '_rq3',
    ]

    _fields_and_field_types = {
        'rq1': 'double',
        'rq2': 'double',
        'rq3': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.rq1 = kwargs.get('rq1', float())
        self.rq2 = kwargs.get('rq2', float())
        self.rq3 = kwargs.get('rq3', float())

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
        if self.rq1 != other.rq1:
            return False
        if self.rq2 != other.rq2:
            return False
        if self.rq3 != other.rq3:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def rq1(self):
        """Message field 'rq1'."""
        return self._rq1

    @rq1.setter
    def rq1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rq1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rq1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rq1 = value

    @builtins.property
    def rq2(self):
        """Message field 'rq2'."""
        return self._rq2

    @rq2.setter
    def rq2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rq2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rq2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rq2 = value

    @builtins.property
    def rq3(self):
        """Message field 'rq3'."""
        return self._rq3

    @rq3.setter
    def rq3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rq3' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rq3' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rq3 = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_SetJointStates_Response(type):
    """Metaclass of message 'SetJointStates_Response'."""

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
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.srv.SetJointStates_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_joint_states__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_joint_states__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_joint_states__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_joint_states__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_joint_states__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetJointStates_Response(metaclass=Metaclass_SetJointStates_Response):
    """Message class 'SetJointStates_Response'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


class Metaclass_SetJointStates(type):
    """Metaclass of service 'SetJointStates'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.srv.SetJointStates')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_joint_states

            from custom_interfaces.srv import _set_joint_states
            if _set_joint_states.Metaclass_SetJointStates_Request._TYPE_SUPPORT is None:
                _set_joint_states.Metaclass_SetJointStates_Request.__import_type_support__()
            if _set_joint_states.Metaclass_SetJointStates_Response._TYPE_SUPPORT is None:
                _set_joint_states.Metaclass_SetJointStates_Response.__import_type_support__()


class SetJointStates(metaclass=Metaclass_SetJointStates):
    from custom_interfaces.srv._set_joint_states import SetJointStates_Request as Request
    from custom_interfaces.srv._set_joint_states import SetJointStates_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
