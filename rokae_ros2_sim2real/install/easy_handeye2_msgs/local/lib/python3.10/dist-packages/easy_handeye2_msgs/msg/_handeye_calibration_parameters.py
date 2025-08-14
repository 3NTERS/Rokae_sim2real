# generated from rosidl_generator_py/resource/_idl.py.em
# with input from easy_handeye2_msgs:msg/HandeyeCalibrationParameters.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HandeyeCalibrationParameters(type):
    """Metaclass of message 'HandeyeCalibrationParameters'."""

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
                'easy_handeye2_msgs.msg.HandeyeCalibrationParameters')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__handeye_calibration_parameters
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__handeye_calibration_parameters
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__handeye_calibration_parameters
            cls._TYPE_SUPPORT = module.type_support_msg__msg__handeye_calibration_parameters
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__handeye_calibration_parameters

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MOVE_GROUP_NAMESPACE__DEFAULT': '/',
            'MOVE_GROUP__DEFAULT': 'manipulator',
        }

    @property
    def MOVE_GROUP_NAMESPACE__DEFAULT(cls):
        """Return default value for message field 'move_group_namespace'."""
        return '/'

    @property
    def MOVE_GROUP__DEFAULT(cls):
        """Return default value for message field 'move_group'."""
        return 'manipulator'


class HandeyeCalibrationParameters(metaclass=Metaclass_HandeyeCalibrationParameters):
    """Message class 'HandeyeCalibrationParameters'."""

    __slots__ = [
        '_name',
        '_calibration_type',
        '_robot_base_frame',
        '_robot_effector_frame',
        '_tracking_base_frame',
        '_tracking_marker_frame',
        '_freehand_robot_movement',
        '_move_group_namespace',
        '_move_group',
    ]

    _fields_and_field_types = {
        'name': 'string',
        'calibration_type': 'string',
        'robot_base_frame': 'string',
        'robot_effector_frame': 'string',
        'tracking_base_frame': 'string',
        'tracking_marker_frame': 'string',
        'freehand_robot_movement': 'boolean',
        'move_group_namespace': 'string',
        'move_group': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.name = kwargs.get('name', str())
        self.calibration_type = kwargs.get('calibration_type', str())
        self.robot_base_frame = kwargs.get('robot_base_frame', str())
        self.robot_effector_frame = kwargs.get('robot_effector_frame', str())
        self.tracking_base_frame = kwargs.get('tracking_base_frame', str())
        self.tracking_marker_frame = kwargs.get('tracking_marker_frame', str())
        self.freehand_robot_movement = kwargs.get('freehand_robot_movement', bool())
        self.move_group_namespace = kwargs.get(
            'move_group_namespace', HandeyeCalibrationParameters.MOVE_GROUP_NAMESPACE__DEFAULT)
        self.move_group = kwargs.get(
            'move_group', HandeyeCalibrationParameters.MOVE_GROUP__DEFAULT)

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
        if self.name != other.name:
            return False
        if self.calibration_type != other.calibration_type:
            return False
        if self.robot_base_frame != other.robot_base_frame:
            return False
        if self.robot_effector_frame != other.robot_effector_frame:
            return False
        if self.tracking_base_frame != other.tracking_base_frame:
            return False
        if self.tracking_marker_frame != other.tracking_marker_frame:
            return False
        if self.freehand_robot_movement != other.freehand_robot_movement:
            return False
        if self.move_group_namespace != other.move_group_namespace:
            return False
        if self.move_group != other.move_group:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'name' field must be of type 'str'"
        self._name = value

    @builtins.property
    def calibration_type(self):
        """Message field 'calibration_type'."""
        return self._calibration_type

    @calibration_type.setter
    def calibration_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'calibration_type' field must be of type 'str'"
        self._calibration_type = value

    @builtins.property
    def robot_base_frame(self):
        """Message field 'robot_base_frame'."""
        return self._robot_base_frame

    @robot_base_frame.setter
    def robot_base_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'robot_base_frame' field must be of type 'str'"
        self._robot_base_frame = value

    @builtins.property
    def robot_effector_frame(self):
        """Message field 'robot_effector_frame'."""
        return self._robot_effector_frame

    @robot_effector_frame.setter
    def robot_effector_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'robot_effector_frame' field must be of type 'str'"
        self._robot_effector_frame = value

    @builtins.property
    def tracking_base_frame(self):
        """Message field 'tracking_base_frame'."""
        return self._tracking_base_frame

    @tracking_base_frame.setter
    def tracking_base_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'tracking_base_frame' field must be of type 'str'"
        self._tracking_base_frame = value

    @builtins.property
    def tracking_marker_frame(self):
        """Message field 'tracking_marker_frame'."""
        return self._tracking_marker_frame

    @tracking_marker_frame.setter
    def tracking_marker_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'tracking_marker_frame' field must be of type 'str'"
        self._tracking_marker_frame = value

    @builtins.property
    def freehand_robot_movement(self):
        """Message field 'freehand_robot_movement'."""
        return self._freehand_robot_movement

    @freehand_robot_movement.setter
    def freehand_robot_movement(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'freehand_robot_movement' field must be of type 'bool'"
        self._freehand_robot_movement = value

    @builtins.property
    def move_group_namespace(self):
        """Message field 'move_group_namespace'."""
        return self._move_group_namespace

    @move_group_namespace.setter
    def move_group_namespace(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'move_group_namespace' field must be of type 'str'"
        self._move_group_namespace = value

    @builtins.property
    def move_group(self):
        """Message field 'move_group'."""
        return self._move_group

    @move_group.setter
    def move_group(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'move_group' field must be of type 'str'"
        self._move_group = value
