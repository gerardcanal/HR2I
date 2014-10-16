"""autogenerated by genpy from roswifibot/vn100_msg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class vn100_msg(genpy.Message):
  _md5sum = "a95abc59ede2a93c5088a3b41defbc41"
  _type = "roswifibot/vn100_msg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 yaw
float32 roll
float32 pitch


"""
  __slots__ = ['yaw','roll','pitch']
  _slot_types = ['float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       yaw,roll,pitch

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(vn100_msg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.yaw is None:
        self.yaw = 0.
      if self.roll is None:
        self.roll = 0.
      if self.pitch is None:
        self.pitch = 0.
    else:
      self.yaw = 0.
      self.roll = 0.
      self.pitch = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3f.pack(_x.yaw, _x.roll, _x.pitch))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 12
      (_x.yaw, _x.roll, _x.pitch,) = _struct_3f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3f.pack(_x.yaw, _x.roll, _x.pitch))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 12
      (_x.yaw, _x.roll, _x.pitch,) = _struct_3f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3f = struct.Struct("<3f")
