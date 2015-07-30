"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

import cStringIO as StringIO
import struct

class goal_t(object):
    __slots__ = ["robot_id", "vertex_id", "decision_time", "estimate_time"]

    def __init__(self):
        self.robot_id = 0
        self.vertex_id = 0
        self.decision_time = 0.0
        self.estimate_time = 0.0

    def encode(self):
        buf = StringIO.StringIO()
        buf.write(goal_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">iidd", self.robot_id, self.vertex_id, self.decision_time, self.estimate_time))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = StringIO.StringIO(data)
        if buf.read(8) != goal_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return goal_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = goal_t()
        self.robot_id, self.vertex_id, self.decision_time, self.estimate_time = struct.unpack(">iidd", buf.read(24))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if goal_t in parents: return 0
        tmphash = (0x5e9a527b06348676) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if goal_t._packed_fingerprint is None:
            goal_t._packed_fingerprint = struct.pack(">Q", goal_t._get_hash_recursive([]))
        return goal_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
