"""Transport format for Webots sensor frames.

Messages are length-delimited metadata JSON plus opaque binary payload.
This module only uses Python's standard library so it can be imported by both
Webots' Windows Python controller and a Linux ROS 2 bridge.
"""

import json
import math
import struct

MAGIC = b'WBL2'
VERSION = 1
HEADER = struct.Struct('!4sBII')
POINT = struct.Struct('<fffIBBB')  # xyz / offset_time_ns / reflectivity / tag / line
MAX_META = 65536
MAX_PAYLOAD = 16 * 1024 * 1024


class FrameError(ValueError):
    pass


def encode_frame(metadata, payload=b''):
    if not isinstance(metadata, dict):
        raise TypeError('metadata must be an object')
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError('payload must be bytes')
    info = dict(metadata)
    info['protocol_version'] = VERSION
    meta = json.dumps(info, separators=(',', ':'), allow_nan=False).encode('utf-8')
    payload = bytes(payload)
    if len(meta) > MAX_META or len(payload) > MAX_PAYLOAD:
        raise FrameError('frame exceeds transport limits')
    return HEADER.pack(MAGIC, VERSION, len(meta), len(payload)) + meta + payload


def decode_header(header):
    if len(header) != HEADER.size:
        raise FrameError('truncated header')
    magic, version, meta_len, data_len = HEADER.unpack(header)
    if magic != MAGIC or version != VERSION:
        raise FrameError('wrong protocol/version')
    if meta_len > MAX_META or data_len > MAX_PAYLOAD:
        raise FrameError('frame length out of range')
    return meta_len, data_len


def decode_frame_bytes(data):
    meta_len, body_len = decode_header(data[:HEADER.size])
    if len(data) != HEADER.size + meta_len + body_len:
        raise FrameError('wrong frame size')
    m = json.loads(data[HEADER.size:HEADER.size + meta_len].decode('utf-8'))
    return m, data[HEADER.size + meta_len:]


def recv_exact(sock, count):
    parts = []
    while count:
        chunk = sock.recv(count)
        if not chunk:
            raise EOFError('connection closed')
        parts.append(chunk)
        count -= len(chunk)
    return b''.join(parts)


def recv_frame(sock):
    meta_len, body_len = decode_header(recv_exact(sock, HEADER.size))
    meta = json.loads(recv_exact(sock, meta_len).decode('utf-8'))
    if meta.get('protocol_version') != VERSION:
        raise FrameError('metadata version mismatch')
    return meta, recv_exact(sock, body_len)


def pack_point_cloud(points, horizontal_resolution, scan_duration_ns=0, reflectivity=180, native_start=None):
    """Pack Webots point objects as an *approximate* Livox-compatible point set.

    native_start selects Webots layer IDs and absolute per-ray simulation times.
    scan_duration_ns==0 keeps all offsets at zero (default; no fabricated times).
    Nonzero scan_duration_ns is *assumed* scan duration, not measured timing.
    """
    hres = max(1, int(horizontal_resolution))
    scan_ns = max(0, int(scan_duration_ns))
    out = bytearray()
    count = 0
    for index, p in enumerate(points):
        x, y, z = float(p.x), float(p.y), float(p.z)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if x*x + y*y + z*z <= .01:
            continue
        layer = min(255, index // hres)
        offset = int((index % hres) * scan_ns / hres) if scan_ns else 0
        if native_start is not None:
            layer = int(p.layer)
            offset = round((p.time - native_start) * 1e9)
            if not 0 <= offset <= 0xffffffff or not 0 <= layer <= 255:
                raise FrameError("invalid native LiDAR timestamp or layer")
        out.extend(POINT.pack(x, y, z, offset, int(reflectivity), 0, layer))
        count += 1
    if native_start is not None:
        records = sorted(POINT.iter_unpack(out), key=lambda record: record[3])
        return b"".join(POINT.pack(*record) for record in records), count
    return bytes(out), count


def unpack_points(data):
    if len(data) % POINT.size:
        raise FrameError('invalid point-cloud payload length')
    return list(POINT.iter_unpack(data))
