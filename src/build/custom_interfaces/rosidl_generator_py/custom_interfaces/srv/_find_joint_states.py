# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_interfaces:srv/FindJointStates.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_FindJointStates_Request(type):
    """Metaclass of message 'FindJointStates_Request'."""

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
                'custom_interfaces.srv.FindJointStates_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__find_joint_states__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__find_joint_states__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__find_joint_states__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__find_joint_states__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__find_joint_states__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class FindJointStates_Request(metaclass=Metaclass_FindJointStates_Request):
    """Message class 'FindJointStates_Request'."""

    __slots__ = [
        '_x',
        '_y',
        '_z',
    ]

    _fields_and_field_types = {
        'x': 'double',
        'y': 'double',
        'z': 'double',
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
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.z = kwargs.get('z', float())

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
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y = value

    @builtins.property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._z = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_FindJointStates_Response(type):
    """Metaclass of message 'FindJointStates_Response'."""

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
                'custom_interfaces.srv.FindJointStates_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__find_joint_states__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__find_joint_states__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__find_joint_states__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__find_joint_states__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__find_joint_states__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class FindJointStates_Response(metaclass=Metaclass_FindJointStates_Response):
    """Message class 'FindJointStates_Response'."""

    __slots__ = [
        '_q11',
        '_q21',
        '_q31',
        '_q12',
        '_q22',
        '_q32',
    ]

    _fields_and_field_types = {
        'q11': 'double',
        'q21': 'double',
        'q31': 'double',
        'q12': 'double',
        'q22': 'double',
        'q32': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.q11 = kwargs.get('q11', float())
        self.q21 = kwargs.get('q21', float())
        self.q31 = kwargs.get('q31', float())
        self.q12 = kwargs.get('q12', float())
        self.q22 = kwargs.get('q22', float())
        self.q32 = kwargs.get('q32', float())

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
        if self.q11 != other.q11:
            return False
        if self.q21 != other.q21:
            return False
        if self.q31 != other.q31:
            return False
        if self.q12 != other.q12:
            return False
        if self.q22 != other.q22:
            return False
        if self.q32 != other.q32:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def q11(self):
        """Message field 'q11'."""
        return self._q11

    @q11.setter
    def q11(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q11' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'q11' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._q11 = value

    @builtins.property
    def q21(self):
        """Message field 'q21'."""
        return self._q21

    @q21.setter
    def q21(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q21' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'q21' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._q21 = value

    @builtins.property
    def q31(self):
        """Message field 'q31'."""
        return self._q31

    @q31.setter
    def q31(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q31' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'q31' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._q31 = value

    @builtins.property
    def q12(self):
        """Message field 'q12'."""
        return self._q12

    @q12.setter
    def q12(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q12' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'q12' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._q12 = value

    @builtins.property
    def q22(self):
        """Message field 'q22'."""
        return self._q22

    @q22.setter
    def q22(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q22' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'q22' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._q22 = value

    @builtins.property
    def q32(self):
        """Message field 'q32'."""
        return self._q32

    @q32.setter
    def q32(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q32' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'q32' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._q32 = value


class Metaclass_FindJointStates(type):
    """Metaclass of service 'FindJointStates'."""

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
                'custom_interfaces.srv.FindJointStates')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__find_joint_states

            from custom_interfaces.srv import _find_joint_states
            if _find_joint_states.Metaclass_FindJointStates_Request._TYPE_SUPPORT is None:
                _find_joint_states.Metaclass_FindJointStates_Request.__import_type_support__()
            if _find_joint_states.Metaclass_FindJointStates_Response._TYPE_SUPPORT is None:
                _find_joint_states.Metaclass_FindJointStates_Response.__import_type_support__()


class FindJointStates(metaclass=Metaclass_FindJointStates):
    from custom_interfaces.srv._find_joint_states import FindJointStates_Request as Request
    from custom_interfaces.srv._find_joint_states import FindJointStates_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
