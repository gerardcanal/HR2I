"""autogenerated by genpy from roswifibot/VnMatrix3x3.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class VnMatrix3x3(genpy.Message):
  _md5sum = "3fd32eb56edcd11d815b05ae3708757f"
  _type = "roswifibot/VnMatrix3x3"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 c00
float64 c01
float64 c02
float64 c10
float64 c11
float64 c12
float64 c20
float64 c21
float64 c22

"""
  __slots__ = ['c00','c01','c02','c10','c11','c12','c20','c21','c22']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       c00,c01,c02,c10,c11,c12,c20,c21,c22

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(VnMatrix3x3, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.c00 is None:
        self.c00 = 0.
      if self.c01 is None:
        self.c01 = 0.
      if self.c02 is None:
        self.c02 = 0.
      if self.c10 is None:
        self.c10 = 0.
      if self.c11 is None:
        self.c11 = 0.
      if self.c12 is None:
        self.c12 = 0.
      if self.c20 is None:
        self.c20 = 0.
      if self.c21 is None:
        self.c21 = 0.
      if self.c22 is None:
        self.c22 = 0.
    else:
      self.c00 = 0.
      self.c01 = 0.
      self.c02 = 0.
      self.c10 = 0.
      self.c11 = 0.
      self.c12 = 0.
      self.c20 = 0.
      self.c21 = 0.
      self.c22 = 0.

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
      buff.write(_struct_9d.pack(_x.c00, _x.c01, _x.c02, _x.c10, _x.c11, _x.c12, _x.c20, _x.c21, _x.c22))
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
      end += 72
      (_x.c00, _x.c01, _x.c02, _x.c10, _x.c11, _x.c12, _x.c20, _x.c21, _x.c22,) = _struct_9d.unpack(str[start:end])
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
      buff.write(_struct_9d.pack(_x.c00, _x.c01, _x.c02, _x.c10, _x.c11, _x.c12, _x.c20, _x.c21, _x.c22))
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
      end += 72
      (_x.c00, _x.c01, _x.c02, _x.c10, _x.c11, _x.c12, _x.c20, _x.c21, _x.c22,) = _struct_9d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_9d = struct.Struct("<9d")
