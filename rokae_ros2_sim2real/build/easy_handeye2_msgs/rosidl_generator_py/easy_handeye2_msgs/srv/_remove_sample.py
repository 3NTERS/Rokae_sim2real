# generated from rosidl_generator_py/resource/_idl.py.em
# with input from easy_handeye2_msgs:srv/RemoveSample.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RemoveSample_Request(type):
    """Metaclass of message 'RemoveSample_Request'."""

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
                'easy_handeye2_msgs.srv.RemoveSample_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__remove_sample__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__remove_sample__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__remove_sample__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__remove_sample__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__remove_sample__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RemoveSample_Request(metaclass=Metaclass_RemoveSample_Request):
    """Message class 'RemoveSample_Request'."""

    __slots__ = [
        '_sample_index',
    ]

    _fields_and_field_types = {
        'sample_index': 'int8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.sample_index = kwargs.get('sample_index', int())

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
        if self.sample_index != other.sample_index:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def sample_index(self):
        """Message field 'sample_index'."""
        return self._sample_index

    @sample_index.setter
    def sample_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sample_index' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'sample_index' field must be an integer in [-128, 127]"
        self._sample_index = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_RemoveSample_Response(type):
    """Metaclass of message 'RemoveSample_Response'."""

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
                'easy_handeye2_msgs.srv.RemoveSample_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__remove_sample__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__remove_sample__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__remove_sample__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__remove_sample__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__remove_sample__response

            from easy_handeye2_msgs.msg import SampleList
            if SampleList.__class__._TYPE_SUPPORT is None:
                SampleList.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RemoveSample_Response(metaclass=Metaclass_RemoveSample_Response):
    """Message class 'RemoveSample_Response'."""

    __slots__ = [
        '_samples',
    ]

    _fields_and_field_types = {
        'samples': 'easy_handeye2_msgs/SampleList',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['easy_handeye2_msgs', 'msg'], 'SampleList'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from easy_handeye2_msgs.msg import SampleList
        self.samples = kwargs.get('samples', SampleList())

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
        if self.samples != other.samples:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def samples(self):
        """Message field 'samples'."""
        return self._samples

    @samples.setter
    def samples(self, value):
        if __debug__:
            from easy_handeye2_msgs.msg import SampleList
            assert \
                isinstance(value, SampleList), \
                "The 'samples' field must be a sub message of type 'SampleList'"
        self._samples = value


class Metaclass_RemoveSample(type):
    """Metaclass of service 'RemoveSample'."""

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
                'easy_handeye2_msgs.srv.RemoveSample')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__remove_sample

            from easy_handeye2_msgs.srv import _remove_sample
            if _remove_sample.Metaclass_RemoveSample_Request._TYPE_SUPPORT is None:
                _remove_sample.Metaclass_RemoveSample_Request.__import_type_support__()
            if _remove_sample.Metaclass_RemoveSample_Response._TYPE_SUPPORT is None:
                _remove_sample.Metaclass_RemoveSample_Response.__import_type_support__()


class RemoveSample(metaclass=Metaclass_RemoveSample):
    from easy_handeye2_msgs.srv._remove_sample import RemoveSample_Request as Request
    from easy_handeye2_msgs.srv._remove_sample import RemoveSample_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
