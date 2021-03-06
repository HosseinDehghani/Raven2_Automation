"""autogenerated by genmsg_py from raven_master.msg. Do not edit."""
import roslib.message
import struct


class raven_master(roslib.message.Message):
  _md5sum = "b43de1c82175a6f8af3685eeb0096e3c"
  _type = "raven_2/raven_master"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 sequence
int32      pactyp
int32      version
int32[2]	 delx
int32[2]	 dely
int32[2]	 delz
float32[2]	 Qx
float32[2]	 Qy
float32[2]	 Qz
float32[2]	 Qw
int32[2]	 buttonstate
int32[2]	 grasp
int32	 surgeon_mode
int32	 checksum

"""
  __slots__ = ['sequence','pactyp','version','delx','dely','delz','Qx','Qy','Qz','Qw','buttonstate','grasp','surgeon_mode','checksum']
  _slot_types = ['int32','int32','int32','int32[2]','int32[2]','int32[2]','float32[2]','float32[2]','float32[2]','float32[2]','int32[2]','int32[2]','int32','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       sequence,pactyp,version,delx,dely,delz,Qx,Qy,Qz,Qw,buttonstate,grasp,surgeon_mode,checksum
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(raven_master, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.sequence is None:
        self.sequence = 0
      if self.pactyp is None:
        self.pactyp = 0
      if self.version is None:
        self.version = 0
      if self.delx is None:
        self.delx = [0,0]
      if self.dely is None:
        self.dely = [0,0]
      if self.delz is None:
        self.delz = [0,0]
      if self.Qx is None:
        self.Qx = [0.,0.]
      if self.Qy is None:
        self.Qy = [0.,0.]
      if self.Qz is None:
        self.Qz = [0.,0.]
      if self.Qw is None:
        self.Qw = [0.,0.]
      if self.buttonstate is None:
        self.buttonstate = [0,0]
      if self.grasp is None:
        self.grasp = [0,0]
      if self.surgeon_mode is None:
        self.surgeon_mode = 0
      if self.checksum is None:
        self.checksum = 0
    else:
      self.sequence = 0
      self.pactyp = 0
      self.version = 0
      self.delx = [0,0]
      self.dely = [0,0]
      self.delz = [0,0]
      self.Qx = [0.,0.]
      self.Qy = [0.,0.]
      self.Qz = [0.,0.]
      self.Qw = [0.,0.]
      self.buttonstate = [0,0]
      self.grasp = [0,0]
      self.surgeon_mode = 0
      self.checksum = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_3i.pack(_x.sequence, _x.pactyp, _x.version))
      buff.write(_struct_2i.pack(*self.delx))
      buff.write(_struct_2i.pack(*self.dely))
      buff.write(_struct_2i.pack(*self.delz))
      buff.write(_struct_2f.pack(*self.Qx))
      buff.write(_struct_2f.pack(*self.Qy))
      buff.write(_struct_2f.pack(*self.Qz))
      buff.write(_struct_2f.pack(*self.Qw))
      buff.write(_struct_2i.pack(*self.buttonstate))
      buff.write(_struct_2i.pack(*self.grasp))
      _x = self
      buff.write(_struct_2i.pack(_x.surgeon_mode, _x.checksum))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 12
      (_x.sequence, _x.pactyp, _x.version,) = _struct_3i.unpack(str[start:end])
      start = end
      end += 8
      self.delx = _struct_2i.unpack(str[start:end])
      start = end
      end += 8
      self.dely = _struct_2i.unpack(str[start:end])
      start = end
      end += 8
      self.delz = _struct_2i.unpack(str[start:end])
      start = end
      end += 8
      self.Qx = _struct_2f.unpack(str[start:end])
      start = end
      end += 8
      self.Qy = _struct_2f.unpack(str[start:end])
      start = end
      end += 8
      self.Qz = _struct_2f.unpack(str[start:end])
      start = end
      end += 8
      self.Qw = _struct_2f.unpack(str[start:end])
      start = end
      end += 8
      self.buttonstate = _struct_2i.unpack(str[start:end])
      start = end
      end += 8
      self.grasp = _struct_2i.unpack(str[start:end])
      _x = self
      start = end
      end += 8
      (_x.surgeon_mode, _x.checksum,) = _struct_2i.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_3i.pack(_x.sequence, _x.pactyp, _x.version))
      buff.write(self.delx.tostring())
      buff.write(self.dely.tostring())
      buff.write(self.delz.tostring())
      buff.write(self.Qx.tostring())
      buff.write(self.Qy.tostring())
      buff.write(self.Qz.tostring())
      buff.write(self.Qw.tostring())
      buff.write(self.buttonstate.tostring())
      buff.write(self.grasp.tostring())
      _x = self
      buff.write(_struct_2i.pack(_x.surgeon_mode, _x.checksum))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 12
      (_x.sequence, _x.pactyp, _x.version,) = _struct_3i.unpack(str[start:end])
      start = end
      end += 8
      self.delx = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=2)
      start = end
      end += 8
      self.dely = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=2)
      start = end
      end += 8
      self.delz = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=2)
      start = end
      end += 8
      self.Qx = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=2)
      start = end
      end += 8
      self.Qy = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=2)
      start = end
      end += 8
      self.Qz = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=2)
      start = end
      end += 8
      self.Qw = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=2)
      start = end
      end += 8
      self.buttonstate = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=2)
      start = end
      end += 8
      self.grasp = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=2)
      _x = self
      start = end
      end += 8
      (_x.surgeon_mode, _x.checksum,) = _struct_2i.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3i = struct.Struct("<3i")
_struct_2i = struct.Struct("<2i")
_struct_2f = struct.Struct("<2f")
