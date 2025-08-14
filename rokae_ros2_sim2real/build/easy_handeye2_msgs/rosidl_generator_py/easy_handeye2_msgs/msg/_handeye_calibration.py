# generated from rosidl_generator_py/resource/_idl.py.em
# with input from easy_handeye2_msgs:msg/HandeyeCalibration.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HandeyeCalibration(type):
    """Metaclass of message 'HandeyeCalibration'."""

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
                'easy_handeye2_msgs.msg.HandeyeCalibration')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__handeye_calibration
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__handeye_calibration
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__handeye_calibration
            cls._TYPE_SUPPORT = module.type_support_msg__msg__handeye_calibration
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__handeye_calibration

            from easy_handeye2_msgs.msg import HandeyeCalibrationParameters
            if HandeyeCalibrationParameters.__class__._TYPE_SUPPORT is None:
                HandeyeCalibrationParameters.__class__.__import_type_support__()

            from geometry_msgs.msg import Transform
            if Transform.__class__._TYPE_SUPPORT is None:
                Transform.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class HandeyeCalibration(metaclass=Metaclass_HandeyeCalibration):
    """Message class 'HandeyeCalibration'."""

    __slots__ = [
        '_parameters',
        '_transform',
    ]

    _fields_and_field_types = {
        'parameters': 'easy_handeye2_msgs/HandeyeCalibrationParameters',
        'transform': 'geometry_msgs/Transform',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['easy_handeye2_msgs', 'msg'], 'HandeyeCalibrationParameters'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Transform'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from easy_handeye2_msgs.msg import HandeyeCalibrationParameters
        self.parameters = kwargs.get('parameters', HandeyeCalibrationParameters())
        from geometry_msgs.msg import Transform
        self.transform = kwargs.get('transform', Transform())

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
        if self.parameters != other.parameters:
            return False
        if self.transform != other.transform:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def parameters(self):
        """Message field 'parameters'."""
        return self._parameters

    @parameters.setter
    def parameters(self, value):
        if __debug__:
            from easy_handeye2_msgs.msg import HandeyeCalibrationParameters
            assert \
                isinstance(value, HandeyeCalibrationParameters), \
                "The 'parameters' field must be a sub message of type 'HandeyeCalibrationParameters'"
        self._parameters = value

    @builtins.property
    def transform(self):
        """Message field 'transform'."""
        return self._transform

    @transform.setter
    def transform(self, value):
        if __debug__:
            from geometry_msgs.msg import Transform
            assert \
                isinstance(value, Transform), \
                "The 'transform' field must be a sub message of type 'Transform'"
        self._transform = value
