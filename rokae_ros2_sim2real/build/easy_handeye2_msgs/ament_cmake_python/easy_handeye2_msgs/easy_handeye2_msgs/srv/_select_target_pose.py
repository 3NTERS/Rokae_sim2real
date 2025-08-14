# generated from rosidl_generator_py/resource/_idl.py.em
# with input from easy_handeye2_msgs:srv/SelectTargetPose.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SelectTargetPose_Request(type):
    """Metaclass of message 'SelectTargetPose_Request'."""

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
                'easy_handeye2_msgs.srv.SelectTargetPose_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__select_target_pose__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__select_target_pose__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__select_target_pose__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__select_target_pose__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__select_target_pose__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SelectTargetPose_Request(metaclass=Metaclass_SelectTargetPose_Request):
    """Message class 'SelectTargetPose_Request'."""

    __slots__ = [
        '_target_pose_index',
    ]

    _fields_and_field_types = {
        'target_pose_index': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.target_pose_index = kwargs.get('target_pose_index', int())

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
        if self.target_pose_index != other.target_pose_index:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def target_pose_index(self):
        """Message field 'target_pose_index'."""
        return self._target_pose_index

    @target_pose_index.setter
    def target_pose_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'target_pose_index' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'target_pose_index' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._target_pose_index = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SelectTargetPose_Response(type):
    """Metaclass of message 'SelectTargetPose_Response'."""

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
                'easy_handeye2_msgs.srv.SelectTargetPose_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__select_target_pose__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__select_target_pose__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__select_target_pose__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__select_target_pose__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__select_target_pose__response

            from easy_handeye2_msgs.msg import TargetPoseList
            if TargetPoseList.__class__._TYPE_SUPPORT is None:
                TargetPoseList.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SelectTargetPose_Response(metaclass=Metaclass_SelectTargetPose_Response):
    """Message class 'SelectTargetPose_Response'."""

    __slots__ = [
        '_success',
        '_target_poses',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'target_poses': 'easy_handeye2_msgs/TargetPoseList',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['easy_handeye2_msgs', 'msg'], 'TargetPoseList'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        from easy_handeye2_msgs.msg import TargetPoseList
        self.target_poses = kwargs.get('target_poses', TargetPoseList())

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
        if self.success != other.success:
            return False
        if self.target_poses != other.target_poses:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def target_poses(self):
        """Message field 'target_poses'."""
        return self._target_poses

    @target_poses.setter
    def target_poses(self, value):
        if __debug__:
            from easy_handeye2_msgs.msg import TargetPoseList
            assert \
                isinstance(value, TargetPoseList), \
                "The 'target_poses' field must be a sub message of type 'TargetPoseList'"
        self._target_poses = value


class Metaclass_SelectTargetPose(type):
    """Metaclass of service 'SelectTargetPose'."""

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
                'easy_handeye2_msgs.srv.SelectTargetPose')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__select_target_pose

            from easy_handeye2_msgs.srv import _select_target_pose
            if _select_target_pose.Metaclass_SelectTargetPose_Request._TYPE_SUPPORT is None:
                _select_target_pose.Metaclass_SelectTargetPose_Request.__import_type_support__()
            if _select_target_pose.Metaclass_SelectTargetPose_Response._TYPE_SUPPORT is None:
                _select_target_pose.Metaclass_SelectTargetPose_Response.__import_type_support__()


class SelectTargetPose(metaclass=Metaclass_SelectTargetPose):
    from easy_handeye2_msgs.srv._select_target_pose import SelectTargetPose_Request as Request
    from easy_handeye2_msgs.srv._select_target_pose import SelectTargetPose_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
