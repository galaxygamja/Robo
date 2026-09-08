"""Durable guard around the DRAFT synthetic sender, not a live device adapter."""
from __future__ import annotations

from .manipulator_wire import ManipulatorSender, validate_operation
from .operation_journal import (
    ACCEPTED_ACK,
    EXECUTE_INTENT,
    STOP_ACK,
    STOP_REQUESTED,
    JournalError,
    OperationJournal,
)
from .wire_codec import decode_frame


class JournaledManipulatorSender:
    def __init__(self, robot_id, *, session_id, journal):
        if not isinstance(journal, OperationJournal) or robot_id not in journal.fleet:
            raise JournalError("A matching, explicitly opened journal is required")
        self._sender = ManipulatorSender(robot_id, session_id=session_id)
        self._journal = journal
        self._work_id = None
        self._pending_kind = None
        self._storage_fault = None

    def _mark(self, flag):
        self._journal.mark(self._work_id, robot_id=self._sender.robot_id,
                           session_id=self._sender.session_id, flag=flag)

    def begin(self, operation, now_s, *, work_id):
        if self._storage_fault:
            raise JournalError("Storage fault; inspect before constructing a new sender")
        self._sender.poll(now_s)
        if self._sender.state not in {"idle", "closed"}:
            raise JournalError("Previous sender still active")
        operation = validate_operation(operation)
        try:
            self._journal.reserve(work_id=work_id, robot_id=self._sender.robot_id,
                                  session_id=self._sender.session_id, operation=operation)
            self._work_id = work_id
            message = self._sender.begin(operation, now_s)
            self._pending_kind = "hello"
            return message
        except (ValueError, OSError) as exc:
            self._storage_fault = str(exc)
            raise

    def execute(self, now_s):
        if self._storage_fault:
            raise JournalError("Storage fault prevents execute")
        message = self._sender.execute(now_s)
        try:
            self._mark(EXECUTE_INTENT)  # Committed before the caller receives bytes.
        except (ValueError, OSError) as exc:
            self._storage_fault = str(exc)
            self._sender.stop(now_s)  # No execute was handed to this caller.
            raise
        self._pending_kind = "execute"
        return message

    def sample(self, now_s):
        if self._storage_fault:
            raise JournalError("Storage fault prevents new requests")
        result = self._sender.sample(now_s)
        self._pending_kind = "sample"
        return result

    def stop(self, now_s, *, kind="stop"):
        # No disk IO on the stop path: even a lock timeout must not delay it.
        # A missing stop ACK leaves the durable attempt unresolved as before.
        message = self._sender.stop(now_s, kind=kind)
        self._pending_kind = kind if message is not None else None
        return message

    def accept_response(self, response, now_s):
        kind = self._pending_kind
        accepted = self._sender.accept_response(response, now_s)
        self._pending_kind = None
        if accepted and kind in {"execute", "cancel", "stop", "estop"}:
            try:
                if kind != "execute":
                    self._mark(STOP_REQUESTED)
                self._mark(ACCEPTED_ACK if kind == "execute" else STOP_ACK)
            except (ValueError, OSError) as exc:
                self._storage_fault = str(exc)
                return False
        return accepted

    def accept_frame(self, frame, now_s):
        try:
            response = decode_frame(frame)
        except ValueError:
            self._pending_kind = None
            return self._sender.accept_frame(frame, now_s)
        return self.accept_response(response, now_s)

    def poll(self, now_s):
        self._sender.poll(now_s)
        return self.snapshot()

    def snapshot(self):
        return {**self._sender.snapshot(), "journal_id": self._journal.journal_id,
                "work_id": self._work_id, "storage_fault": self._storage_fault,
                "automatic_replay_enabled": False}
