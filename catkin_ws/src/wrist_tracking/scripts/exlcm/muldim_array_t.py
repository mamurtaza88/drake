"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class muldim_array_t(object):
    __slots__ = ["size_a", "size_b", "size_c", "data", "strarray"]

    __typenames__ = ["int32_t", "int32_t", "int32_t", "int32_t", "string"]

    __dimensions__ = [None, None, None, ["size_a", "size_b", "size_c"], [2, "size_c"]]

    def __init__(self):
        self.size_a = 0
        self.size_b = 0
        self.size_c = 0
        self.data = []
        self.strarray = [ [] for dim0 in range(2) ]

    def encode(self):
        buf = BytesIO()
        buf.write(muldim_array_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">iii", self.size_a, self.size_b, self.size_c))
        for i0 in range(self.size_a):
            for i1 in range(self.size_b):
                buf.write(struct.pack('>%di' % self.size_c, *self.data[i0][i1][:self.size_c]))
        for i0 in range(2):
            for i1 in range(self.size_c):
                __strarray_encoded = self.strarray[i0][i1].encode('utf-8')
                buf.write(struct.pack('>I', len(__strarray_encoded)+1))
                buf.write(__strarray_encoded)
                buf.write(b"\0")

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != muldim_array_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return muldim_array_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = muldim_array_t()
        self.size_a, self.size_b, self.size_c = struct.unpack(">iii", buf.read(12))
        self.data = []
        for i0 in range(self.size_a):
            self.data.append([])
            for i1 in range(self.size_b):
                self.data[i0].append(struct.unpack('>%di' % self.size_c, buf.read(self.size_c * 4)))
        self.strarray = []
        for i0 in range(2):
            self.strarray.append ([])
            for i1 in range(self.size_c):
                __strarray_len = struct.unpack('>I', buf.read(4))[0]
                self.strarray[i0].append(buf.read(__strarray_len)[:-1].decode('utf-8', 'replace'))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if muldim_array_t in parents: return 0
        tmphash = (0x1e012473deb4cfbb) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if muldim_array_t._packed_fingerprint is None:
            muldim_array_t._packed_fingerprint = struct.pack(">Q", muldim_array_t._get_hash_recursive([]))
        return muldim_array_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", muldim_array_t._get_packed_fingerprint())[0]

