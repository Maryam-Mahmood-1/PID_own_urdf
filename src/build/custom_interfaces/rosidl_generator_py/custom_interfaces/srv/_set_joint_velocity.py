# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_interfaces:srv/SetJointVelocity.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetJointVelocity_Request(type):
    """Metaclass of message 'SetJointVelocity_Request'."""

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
                'custom_interfaces.srv.SetJointVelocity_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_joint_velocity__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_joint_velocity__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_joint_velocity__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_joint_velocity__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_joint_velocity__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetJointVelocity_Request(metaclass=Metaclass_SetJointVelocity_Request):
    """Message class 'SetJointVelocity_Request'."""

    __slots__ = [
        '_vq1',
        '_vq2',
        '_vq3',
    ]

    _fields_and_field_types = {
        'vq1': 'double',
        'vq2': 'double',
        'vq3': 'double',
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
        self.vq1 = kwargs.get('vq1', float())
        self.vq2 = kwargs.get('vq2', float())
        self.vq3 = kwargs.get('vq3', float())

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
        if self.vq1 != other.vq1:
            return False
        if self.vq2 != other.vq2:
            return False
        if self.vq3 != other.vq3:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def vq1(self):
        """Message field 'vq1'."""
        return self._vq1

    @vq1.setter
    def vq1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vq1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vq1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vq1 = value

    @builtins.property
    def vq2(self):
        """Message field 'vq2'."""
        return self._vq2

    @vq2.setter
    def vq2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vq2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vq2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vq2 = value

    @builtins.property
    def vq3(self):
        """Message field 'vq3'."""
        return self._vq3

    @vq3.setter
    def vq3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vq3' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vq3' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vq3 = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_SetJointVelocity_Response(type):
    """Metaclass of message 'SetJointVelocity_Response'."""

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
                'custom_interfaces.srv.SetJointVelocity_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_joint_velocity__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_joint_velocity__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_joint_velocity__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_joint_velocity__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_joint_velocity__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetJointVelocity_Response(metaclass=Metaclass_SetJointVelocity_Response):
    """Message class 'SetJointVelocity_Response'."""

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


class Metaclass_SetJointVelocity(type):
    """Metaclass of service 'SetJointVelocity'."""

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
                'custom_interfaces.srv.SetJointVelocity')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_joint_velocity

            from custom_interfaces.srv import _set_joint_velocity
            if _set_joint_velocity.Metaclass_SetJointVelocity_Request._TYPE_SUPPORT is None:
                _set_joint_velocity.Metaclass_SetJointVelocity_Request.__import_type_support__()
            if _set_joint_velocity.Metaclass_SetJointVelocity_Response._TYPE_SUPPORT is None:
                _set_joint_velocity.Metaclass_SetJointVelocity_Response.__import_type_support__()


class SetJointVelocity(metaclass=Metaclass_SetJointVelocity):
    from custom_interfaces.srv._set_joint_velocity import SetJointVelocity_Request as Request
    from custom_interfaces.srv._set_joint_velocity import SetJointVelocity_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
