# generated from rosidl_generator_py/resource/_idl.py.em
# with input from easy_handeye2_msgs:msg/TargetPoseList.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TargetPoseList(type):
    """Metaclass of message 'TargetPoseList'."""

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
                'easy_handeye2_msgs.msg.TargetPoseList')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__target_pose_list
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__target_pose_list
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__target_pose_list
            cls._TYPE_SUPPORT = module.type_support_msg__msg__target_pose_list
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__target_pose_list

            from easy_handeye2_msgs.msg import HandeyeCalibrationParameters
            if HandeyeCalibrationParameters.__class__._TYPE_SUPPORT is None:
                HandeyeCalibrationParameters.__class__.__import_type_support__()

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TargetPoseList(metaclass=Metaclass_TargetPoseList):
    """Message class 'TargetPoseList'."""

    __slots__ = [
        '_parameters',
        '_home_pose',
        '_target_poses',
        '_current_target_pose_index',
    ]

    _fields_and_field_types = {
        'parameters': 'easy_handeye2_msgs/HandeyeCalibrationParameters',
        'home_pose': 'geometry_msgs/PoseStamped',
        'target_poses': 'sequence<geometry_msgs/PoseStamped>',
        'current_target_pose_index': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['easy_handeye2_msgs', 'msg'], 'HandeyeCalibrationParameters'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped')),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from easy_handeye2_msgs.msg import HandeyeCalibrationParameters
        self.parameters = kwargs.get('parameters', HandeyeCalibrationParameters())
        from geometry_msgs.msg import PoseStamped
        self.home_pose = kwargs.get('home_pose', PoseStamped())
        self.target_poses = kwargs.get('target_poses', [])
        self.current_target_pose_index = kwargs.get('current_target_pose_index', int())

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
        if self.home_pose != other.home_pose:
            return False
        if self.target_poses != other.target_poses:
            return False
        if self.current_target_pose_index != other.current_target_pose_index:
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
    def home_pose(self):
        """Message field 'home_pose'."""
        return self._home_pose

    @home_pose.setter
    def home_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'home_pose' field must be a sub message of type 'PoseStamped'"
        self._home_pose = value

    @builtins.property
    def target_poses(self):
        """Message field 'target_poses'."""
        return self._target_poses

    @target_poses.setter
    def target_poses(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
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
                 all(isinstance(v, PoseStamped) for v in value) and
                 True), \
                "The 'target_poses' field must be a set or sequence and each value of type 'PoseStamped'"
        self._target_poses = value

    @builtins.property
    def current_target_pose_index(self):
        """Message field 'current_target_pose_index'."""
        return self._current_target_pose_index

    @current_target_pose_index.setter
    def current_target_pose_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_target_pose_index' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'current_target_pose_index' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._current_target_pose_index = value
