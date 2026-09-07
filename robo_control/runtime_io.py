"""Bounded, nonblocking process handoff and asynchronous JSONL recording."""

from __future__ import annotations

import json
import queue
import threading
from pathlib import Path


class LatestRecordMailbox:
    """One JSON record in shared memory; images never cross this boundary.

    Both sides try the lock without waiting. Killing a producer while it owns
    the lock cannot freeze the supervisor: polls return no record and the
    observation watchdog expires. Replacement bounds latency and memory.
    Only use this between the trusted local child and its own supervisor.
    """

    def __init__(self, context, capacity=262144):
        if type(capacity) is not int or not 1024 <= capacity <= 2097152:
            raise ValueError("Mailbox capacity must be 1 KiB..2 MiB")
        self.capacity = capacity
        self.buffer = context.RawArray("B", capacity)
        self.length = context.RawValue("I", 0)
        self.revision = context.RawValue("Q", 0)
        self.lock = context.Lock()

    def publish(self, record):
        payload = json.dumps(record, ensure_ascii=False, allow_nan=False,
                             separators=(",", ":")).encode("utf-8")
        if len(payload) > self.capacity:
            raise ValueError("Detection record exceeds bounded mailbox capacity")
        if not self.lock.acquire(False):
            return False
        try:
            self.buffer[:len(payload)] = payload
            self.length.value = len(payload)
            self.revision.value += 1
        finally:
            self.lock.release()
        return True

    def poll(self, after_revision=0):
        if not self.lock.acquire(False):
            return None
        try:
            revision = self.revision.value
            if revision <= after_revision:
                return None
            size = self.length.value
            if not 0 < size <= self.capacity:
                raise ValueError("Invalid mailbox record length")
            payload = bytes(self.buffer[:size])
        finally:
            self.lock.release()
        return revision, json.loads(payload)


class AsyncJsonlReport:
    """A slow/full/failed report never blocks the control tick.

    Backpressure is an explicit runtime failure, not an unbounded queue.
    The final stop event is also in the CLI summary if the writer has failed.
    """

    def __init__(self, path: str | Path, *, capacity=128):
        if type(capacity) is not int or capacity < 2:
            raise ValueError("Report queue needs at least two slots")
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Exclusive creation, including symlink/hardlink aliases of inputs.
        self.handle = self.path.open("x", encoding="utf-8")
        self.pending = queue.Queue(maxsize=capacity)
        self.error = None
        self.closed = False
        self.thread = threading.Thread(target=self._write, name="robo-jsonl-writer", daemon=True)
        self.thread.start()

    def submit(self, event):
        if self.closed or self.error is not None:
            return False
        try:
            line = json.dumps(event, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
            self.pending.put_nowait(line)
        except (ValueError, TypeError, queue.Full) as exc:
            self.error = f"report_backpressure_or_encoding: {type(exc).__name__}"
            return False
        return True

    def _write(self):
        try:
            while True:
                line = self.pending.get()
                if line is None:
                    break
                self.handle.write(line + "\n")
                self.handle.flush()
        except Exception as exc:  # noqa: BLE001 - record any writer failure for the supervisor
            self.error = f"report_write_failed: {type(exc).__name__}: {exc}"
        finally:
            try:
                self.handle.close()
            except OSError as exc:
                self.error = self.error or f"report_close_failed: {exc}"

    def finish(self, timeout_s=2.0):
        if not self.closed:
            self.closed = True
            # Called only AFTER the actuator is stopped. The tick never waits
            # for this final flush, and even shutdown has a finite deadline.
            try:
                self.pending.put(None, timeout=timeout_s / 2)
            except queue.Full:
                self.error = self.error or "report_shutdown_queue_full"
                return False
        self.thread.join(timeout_s / 2)
        if self.thread.is_alive():
            self.error = self.error or "report_flush_timeout"
        return self.error is None and not self.thread.is_alive()
