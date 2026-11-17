"""
IMC messages over UDP, as much as the DUNE bridge and the plan tool need: encode and decode any message of the IMC
definition shipped with the package (dune/IMC.xml, version 5.5.4.9, the one of the DUNE build that runs Marie).

An IMC packet is a 20-byte header (sync, message id, payload size, time stamp, source, source entity, destination,
destination entity), the payload in the order of the fields in IMC.xml, and a CRC-16-IBM footer; little endian here.
Variable fields: plaintext and rawdata carry a 16-bit length; an inline message carries its 16-bit id (0xFFFF: none);
a message list carries a 16-bit count and its messages, each with its id.

  encode("Goto", source, {"lat": ..., "speed": 1.2})       values left out are zero / empty
  encode("PlanControl", source, {"arg": ("PlanSpecification", {...}), ...})      inline message: (name, values)
  decode(datagram) -> [Message(name, source, source_entity, time, values)]
"""

import os
import struct
import time
import xml.etree.ElementTree as ET
from collections import namedtuple

PRIMITIVES = {"int8_t": "b", "uint8_t": "B", "int16_t": "h", "uint16_t": "H", "int32_t": "i", "uint32_t": "I",
              "int64_t": "q", "fp32_t": "f", "fp64_t": "d"}
HEADER = "<HHHdHBHB"
NO_MESSAGE = 0xFFFF
BROADCAST = 0xFFFF

Message = namedtuple("Message", "name source source_entity time values")


def _definition_file():
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.path.join(here, "..", "dune", "IMC.xml")]                 # source tree
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, os.path.join(get_package_share_directory("lauv_marie_control"), "dune", "IMC.xml"))
    except Exception:                                                          # not in a ROS environment
        pass
    for path in candidates:
        if os.path.exists(path):
            return path
    raise FileNotFoundError("IMC.xml of lauv_marie_control not found")


_root = ET.parse(_definition_file()).getroot()
SYNC = int(_root.find("header").find("field[@abbrev='sync']").get("value"), 16)
FIELDS, IDS, NAMES = {}, {}, {}
for _m in _root.iter("message"):
    FIELDS[_m.get("abbrev")] = [(f.get("abbrev"), f.get("type")) for f in _m.findall("field")]
    IDS[_m.get("abbrev")] = int(_m.get("id"))
    NAMES[int(_m.get("id"))] = _m.get("abbrev")


def crc16(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _payload(name, values):
    out = b""
    for field, kind in FIELDS[name]:
        value = values.get(field)
        if kind in PRIMITIVES:
            out += struct.pack("<" + PRIMITIVES[kind], value or 0)
        elif kind in ("plaintext", "rawdata"):
            data = value.encode() if isinstance(value, str) else (value or b"")
            out += struct.pack("<H", len(data)) + data
        elif kind == "message":
            out += struct.pack("<H", NO_MESSAGE) if value is None else struct.pack("<H", IDS[value[0]]) + _payload(*value)
        elif kind == "message-list":
            out += struct.pack("<H", len(value or []))
            out += b"".join(struct.pack("<H", IDS[n]) + _payload(n, v) for n, v in value or [])
    return out


def encode(name, source, values=None, source_entity=0xFF, destination=BROADCAST, destination_entity=0xFF, stamp=None):
    payload = _payload(name, values or {})
    head = struct.pack(HEADER, SYNC, IDS[name], len(payload), time.time() if stamp is None else stamp, source,
                       source_entity, destination, destination_entity)
    return head + payload + struct.pack("<H", crc16(head + payload))


def _fields(name, buf, offset):
    values = {}
    for field, kind in FIELDS[name]:
        if kind in PRIMITIVES:
            values[field] = struct.unpack_from("<" + PRIMITIVES[kind], buf, offset)[0]
            offset += struct.calcsize(PRIMITIVES[kind])
        elif kind in ("plaintext", "rawdata"):
            size = struct.unpack_from("<H", buf, offset)[0]
            data = bytes(buf[offset + 2:offset + 2 + size])
            values[field] = data.decode(errors="replace") if kind == "plaintext" else data
            offset += 2 + size
        elif kind == "message":
            mid = struct.unpack_from("<H", buf, offset)[0]
            offset += 2
            values[field] = None
            if mid != NO_MESSAGE:
                inner, offset = _fields(NAMES[mid], buf, offset)
                values[field] = (NAMES[mid], inner)
        elif kind == "message-list":
            count = struct.unpack_from("<H", buf, offset)[0]
            offset += 2
            values[field] = []
            for _ in range(count):
                mid = struct.unpack_from("<H", buf, offset)[0]
                inner, offset = _fields(NAMES[mid], buf, offset + 2)
                values[field].append((NAMES[mid], inner))
    return values, offset


def decode(datagram, wanted=None):
    """the messages of a datagram; `wanted`: names to decode (the others are skipped, which is much faster)"""
    out, offset = [], 0
    size_of_header = struct.calcsize(HEADER)
    while offset + size_of_header + 2 <= len(datagram):
        sync, mid, size, stamp, source, source_entity, _, _ = struct.unpack_from(HEADER, datagram, offset)
        if sync != SYNC:
            break
        name = NAMES.get(mid)
        if name is not None and (wanted is None or name in wanted):
            try:
                out.append(Message(name, source, source_entity, stamp, _fields(name, datagram, offset + size_of_header)[0]))
            except (struct.error, KeyError):
                pass                                                           # a truncated or unknown inline message
        offset += size_of_header + size + 2
    return out
