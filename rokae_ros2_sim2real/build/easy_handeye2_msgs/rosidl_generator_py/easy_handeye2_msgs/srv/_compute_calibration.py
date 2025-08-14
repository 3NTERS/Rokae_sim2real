# generated from rosidl_generator_py/resource/_idl.py.em
# with input from easy_handeye2_msgs:srv/ComputeCalibration.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ComputeCalibration_Request(type):
    """Metaclass of message 'ComputeCalibration_Request'."""

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
            module = import_type_support('easy_handeye2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'easy_handeye2_msgs.srv.ComputeCalibration_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__compute_calibration__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__compute_calibration__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__compute_calibration__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__compute_calibration__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__compute_calibration__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeCalibration_Request(metaclass=Metaclass_ComputeCalibration_Request):
    """Message class 'ComputeCalibration_Request'."""

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


# Import statements for member types

import builtins  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_ComputeCalibration_Response(type):
    """Metaclass of message 'ComputeCalibration_Response'."""

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
            module = import_type_support('easy_handeye2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'easy_handeye2_msgs.srv.ComputeCalibration_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__compute_calibration__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__compute_calibration__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__compute_calibration__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__compute_calibration__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__compute_calibration__response

            from easy_handeye2_msgs.msg import HandeyeCalibration
            if HandeyeCalibration.__class__._TYPE_SUPPORT is None:
                HandeyeCalibration.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeCalibration_Response(metaclass=Metaclass_ComputeCalibration_Response):
    """Message class 'ComputeCalibration_Response'."""

    __slots__ = [
        '_valid',
        '_calibration',
    ]

    _fields_and_field_types = {
        'valid': 'boolean',
        'calibration': 'easy_handeye2_msgs/HandeyeCalibration',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['easy_handeye2_msgs', 'msg'], 'HandeyeCalibration'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.valid = kwargs.get('valid', bool())
        from easy_handeye2_msgs.msg import HandeyeCalibration
        self.calibration = kwargs.get('calibration', HandeyeCalibration())

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
        if self.valid != other.valid:
            return False
        if self.calibration != other.calibration:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def valid(self):
        """Message field 'valid'."""
        return self._valid

    @valid.setter
    def valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'valid' field must be of type 'bool'"
        self._valid = value

    @builtins.property
    def calibration(self):
        """Message field 'calibration'."""
        return self._calibration

    @calibration.setter
    def calibration(self, value):
        if __debug__:
            from easy_handeye2_msgs.msg import HandeyeCalibration
            assert \
                isinstance(value, HandeyeCalibration), \
                "The 'calibration' field must be a sub message of type 'HandeyeCalibration'"
        self._calibration = value


class Metaclass_ComputeCalibration(type):
    """Metaclass of service 'ComputeCalibration'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('easy_handeye2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'easy_handeye2_msgs.srv.ComputeCalibration')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__compute_calibration

            from easy_handeye2_msgs.srv import _compute_calibration
            if _compute_calibration.Metaclass_ComputeCalibration_Request._TYPE_SUPPORT is None:
                _compute_calibration.Metaclass_ComputeCalibration_Request.__import_type_support__()
            if _compute_calibration.Metaclass_ComputeCalibration_Response._TYPE_SUPPORT is None:
                _compute_calibration.Metaclass_ComputeCalibration_Response.__import_type_support__()


class ComputeCalibration(metaclass=Metaclass_ComputeCalibration):
    from easy_handeye2_msgs.srv._compute_calibration import ComputeCalibration_Request as Request
    from easy_handeye2_msgs.srv._compute_calibration import ComputeCalibration_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
