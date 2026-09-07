"""Bounded, fail-closed JSON-line framing; no transport or motion semantics.

A frame is a strict UTF-8 JSON object followed by LF. CRLF is also accepted,
but other literal CR/LF characters are forbidden inside the JSON text. The
4096-byte limit includes the terminator. Containers may nest at most 16 levels
(the root object is level one), and integers must fit a signed 64-bit value.

FrameDecoder retains at most one bounded partial frame. A feed call returns
its complete messages atomically: any malformed frame discards that call's
results and faults the decoder, rather than trying to resume at a later line.
Messages returned by an earlier successful call cannot be retracted. A caller
must handle WireError by stopping the associated receiver/session.
"""

from __future__ import annotations

import json
import math
from typing import Any

MAX_FRAME_BYTES = 4096
MAX_NESTING_DEPTH = 16
MIN_INTEGER = -(2**63)
MAX_INTEGER = 2**63 - 1


class WireError(ValueError):
    """The wire frame is invalid, incomplete, or the decoder is closed."""


def _validate_json(message: dict) -> None:
    if type(message) is not dict:
        raise WireError("wire message must be a JSON object")
    remaining_nodes = MAX_FRAME_BYTES

    def visit(value: Any, depth: int) -> None:
        nonlocal remaining_nodes
        remaining_nodes -= 1
        if remaining_nodes < 0:
            raise WireError("JSON structure exceeds frame budget")
        value_type = type(value)
        if value is None or value_type is bool:
            return
        if value_type is int:
            if not MIN_INTEGER <= value <= MAX_INTEGER:
                raise WireError("JSON integer is outside signed 64-bit range")
            return
        if value_type is float:
            if not math.isfinite(value):
                raise WireError("JSON numbers must be finite")
            return
        if value_type is str:
            if len(value) > MAX_FRAME_BYTES:
                raise WireError("JSON string exceeds frame budget")
            try:
                value.encode("utf-8", errors="strict")
            except UnicodeError as error:
                raise WireError("JSON strings must be valid Unicode") from error
            return
        if value_type not in (dict, list):
            raise WireError("message contains a non-JSON value")
        if depth >= MAX_NESTING_DEPTH:
            raise WireError("JSON nesting exceeds 16 container levels")
        if len(value) > MAX_FRAME_BYTES:
            raise WireError("JSON container exceeds frame budget")
        if value_type is dict:
            for key, child in value.items():
                if type(key) is not str:
                    raise WireError("JSON object keys must be strings")
                visit(key, depth + 1)
                visit(child, depth + 1)
        else:
            for child in value:
                visit(child, depth + 1)

    visit(message, 0)


def _unique_object(pairs: list[tuple[str, Any]]) -> dict:
    result = {}
    for key, value in pairs:
        if key in result:
            raise WireError("duplicate JSON object key")
        result[key] = value
    return result


def _integer(value: str) -> int:
    if len(value) > 20:
        raise WireError("JSON integer is outside signed 64-bit range")
    result = int(value)
    if not MIN_INTEGER <= result <= MAX_INTEGER:
        raise WireError("JSON integer is outside signed 64-bit range")
    return result


def _nonfinite(value: str) -> None:
    raise WireError("JSON numbers must be finite")


def encode_frame(message: dict) -> bytes:
    """Encode one object as compact UTF-8 JSON plus LF; never mutate input."""
    _validate_json(message)
    output = bytearray()
    encoder = json.JSONEncoder(ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    try:
        for fragment in encoder.iterencode(message):
            encoded = fragment.encode("utf-8", errors="strict")
            if len(output) + len(encoded) + 1 > MAX_FRAME_BYTES:
                raise WireError("wire frame exceeds 4096 bytes including terminator")
            output.extend(encoded)
    except (TypeError, ValueError, UnicodeError, RecursionError) as error:
        if isinstance(error, WireError):
            raise
        raise WireError("message cannot be encoded as strict JSON") from error
    output.append(10)
    return bytes(output)


def decode_frame(frame: bytes) -> dict:
    """Decode exactly one terminated frame; never accept a partial or batch."""
    if type(frame) is not bytes:
        raise WireError("wire frame must be bytes")
    if len(frame) > MAX_FRAME_BYTES:
        raise WireError("wire frame exceeds 4096 bytes including terminator")
    if not frame.endswith(b"\n") or frame.count(b"\n") != 1:
        raise WireError("wire frame must contain exactly one terminal LF")
    payload = frame[:-1]
    if payload.endswith(b"\r"):
        payload = payload[:-1]
    if b"\r" in payload:
        raise WireError("literal CR is only allowed in the CRLF terminator")
    try:
        message = json.loads(
            payload.decode("utf-8", errors="strict"),
            object_pairs_hook=_unique_object,
            parse_int=_integer,
            parse_constant=_nonfinite,
        )
        _validate_json(message)
    except (ValueError, UnicodeError, RecursionError) as error:
        if isinstance(error, WireError):
            raise
        raise WireError("wire payload is not a strict UTF-8 JSON object") from error
    return message


class FrameDecoder:
    """Incremental bounded framing with a fault latch and explicit EOF."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._closed = False
        self._faulted = False

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def faulted(self) -> bool:
        return self._faulted

    def _fault(self) -> None:
        self._buffer.clear()
        self._faulted = True
        self._closed = True

    def feed(self, chunk: bytes) -> list[dict]:
        """Return all complete frames, or fault without returning this batch.

        The input may contain arbitrarily many valid frames. Only the bounded
        current frame is copied into the raw buffer; the returned list naturally
        occupies memory proportional to the number of decoded messages.
        """
        if self._closed:
            raise WireError("decoder is closed; explicitly reset before reuse")
        if type(chunk) is not bytes:
            self._fault()
            raise WireError("wire chunk must be bytes")
        messages = []
        offset = 0
        try:
            while offset < len(chunk):
                end = min(len(chunk), offset + MAX_FRAME_BYTES - len(self._buffer))
                newline = chunk.find(b"\n", offset, end)
                if newline < 0:
                    self._buffer.extend(chunk[offset:end])
                    offset = end
                    if len(self._buffer) >= MAX_FRAME_BYTES:
                        raise WireError("wire frame exceeds 4096 bytes including terminator")
                    continue
                self._buffer.extend(chunk[offset:newline + 1])
                message = decode_frame(bytes(self._buffer))
                self._buffer.clear()
                messages.append(message)
                offset = newline + 1
        except WireError:
            self._fault()
            raise
        return messages

    def finish(self) -> None:
        """Close on EOF, rejecting any unfinished frame. Repeat EOF is safe."""
        if self._faulted:
            raise WireError("decoder is faulted; explicitly reset before reuse")
        if self._buffer:
            self._fault()
            raise WireError("EOF interrupted an unfinished wire frame")
        self._closed = True

    def reset(self) -> None:
        """Discard partial input and latch state for a new trusted session.

        This does not establish or authorize the receiver's protocol session;
        callers must perform the independent protocol handshake after recovery.
        """
        self._buffer.clear()
        self._closed = False
        self._faulted = False
