"""Decoder for StampFly binary telemetry frames.

Frame format (must match common/stampfly_link/src/telemetry_frame.hpp):

    A5 5A | type u8 | len u8 | payload[len] | fletcher16 (LE) over type,len,payload

    type 0x01 schema chunk: u8 version, u8 total_fields, u8 first_index,
                            u8 fields_in_chunk, ASCII names joined with ','
    type 0x02 data:   u32 time_us, u16 seq, field_count x float32 (LE)

The same frames arrive over the drone's USB port and over the joystick's USB
port (forwarded from ESP-NOW). Any other bytes (debug text) are skipped.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator, List, Optional, Union

SYNC = b"\xA5\x5A"
TYPE_SCHEMA = 0x01
TYPE_DATA = 0x02
FORMAT_VERSION = 1
HEADER_SIZE = 4
CHECKSUM_SIZE = 2


def fletcher16(data: bytes) -> int:
    s1 = s2 = 0
    for b in data:
        s1 = (s1 + b) % 255
        s2 = (s2 + s1) % 255
    return (s2 << 8) | s1


def encode_frame(ftype: int, payload: bytes) -> bytes:
    if len(payload) > 240:
        raise ValueError("payload too long")
    body = bytes([ftype, len(payload)]) + payload
    return SYNC + body + struct.pack("<H", fletcher16(body))


def encode_schema(names: List[str], max_payload: int = 240) -> List[bytes]:
    """Encodes the field list as one or more schema chunk frames."""
    frames = []
    first = 0
    while first < len(names):
        chunk: List[str] = []
        size = 4
        for name in names[first:]:
            add = len(name) + (1 if chunk else 0)
            if size + add > max_payload:
                break
            chunk.append(name)
            size += add
        if not chunk:
            raise ValueError("field name too long: %r" % names[first])
        payload = bytes([FORMAT_VERSION, len(names), first, len(chunk)]) + ",".join(chunk).encode("ascii")
        frames.append(encode_frame(TYPE_SCHEMA, payload))
        first += len(chunk)
    return frames


def encode_data(time_us: int, seq: int, values: List[float]) -> bytes:
    return encode_frame(TYPE_DATA, struct.pack("<IH", time_us, seq) + struct.pack("<%df" % len(values), *values))


@dataclass
class SchemaChunk:
    version: int
    total: int
    first: int
    names: List[str]


@dataclass
class Schema:
    version: int
    names: List[str]


@dataclass
class DataFrame:
    time_us: int
    seq: int
    values: List[float]


Frame = Union[SchemaChunk, DataFrame]


def decode_payload(ftype: int, payload: bytes) -> Optional[Frame]:
    if ftype == TYPE_SCHEMA and len(payload) >= 4:
        version, total, first, count = payload[0], payload[1], payload[2], payload[3]
        names = payload[4:].decode("ascii", errors="replace").split(",") if count else []
        if len(names) != count or first + count > total:
            return None
        return SchemaChunk(version, total, first, names)
    if ftype == TYPE_DATA and len(payload) >= 6 and (len(payload) - 6) % 4 == 0:
        time_us, seq = struct.unpack_from("<IH", payload, 0)
        n = (len(payload) - 6) // 4
        return DataFrame(time_us, seq, list(struct.unpack_from("<%df" % n, payload, 6)))
    return None


class FrameParser:
    """Incremental parser: feed() arbitrary byte chunks, iterate complete frames."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.bad_checksums = 0
        self.skipped_bytes = 0

    def feed(self, data: bytes) -> Iterator[Frame]:
        self._buf.extend(data)
        while True:
            start = self._buf.find(SYNC)
            if start < 0:
                # keep a possible first sync byte at the end
                keep = 1 if self._buf[-1:] == SYNC[:1] else 0
                self.skipped_bytes += len(self._buf) - keep
                del self._buf[: len(self._buf) - keep]
                return
            if start:
                self.skipped_bytes += start
                del self._buf[:start]
            if len(self._buf) < HEADER_SIZE:
                return
            length = self._buf[3]
            total = HEADER_SIZE + length + CHECKSUM_SIZE
            if len(self._buf) < total:
                return
            body = bytes(self._buf[2 : HEADER_SIZE + length])
            (chk,) = struct.unpack_from("<H", self._buf, HEADER_SIZE + length)
            if chk != fletcher16(body):
                self.bad_checksums += 1
                del self._buf[:1]  # resync on the next sync pattern
                continue
            frame = decode_payload(body[0], body[2:])
            del self._buf[:total]
            if frame is not None:
                yield frame


class TelemetryDecoder:
    """Turns frames into rows (dicts) once a schema has been received."""

    def __init__(self) -> None:
        self.parser = FrameParser()
        self.schema: Optional[Schema] = None
        self._pending: dict = {}  # first_index -> SchemaChunk of the schema being assembled
        self.rows = 0
        self.lost = 0
        self._last_seq: Optional[int] = None

    def feed(self, data: bytes) -> Iterator[dict]:
        for frame in self.parser.feed(data):
            if isinstance(frame, SchemaChunk):
                self._add_schema_chunk(frame)
                continue
            if self.schema is None or len(frame.values) != len(self.schema.names):
                continue  # wait for a matching schema
            if self._last_seq is not None:
                gap = (frame.seq - self._last_seq - 1) & 0xFFFF
                if gap < 1000:
                    self.lost += gap
            self._last_seq = frame.seq
            self.rows += 1
            row = {"time_s": frame.time_us * 1e-6, "seq": frame.seq}
            row.update(zip(self.schema.names, frame.values))
            yield row

    def _add_schema_chunk(self, chunk: SchemaChunk) -> None:
        if chunk.first == 0:
            self._pending = {}
        self._pending[chunk.first] = chunk
        names: List[str] = []
        while len(names) in self._pending:
            part = self._pending[len(names)]
            if part.total != chunk.total:
                return
            names.extend(part.names)
        if len(names) == chunk.total:
            self.schema = Schema(chunk.version, names)
            self._pending = {}
